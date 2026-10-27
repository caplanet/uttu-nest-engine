//! 모듈 3 — 부착 체인 구축.
//!
//! 설계 §모듈 3 을 그대로 옮긴다.
//!
//! ```text
//!   표면 레코드를 연속 쌍 (i, i+1) 으로 순회
//!     -> 각 표면 쌍의 피스에 바인딩 참조 기록
//!     -> 표면 쌍 중심 거리 sqrt(dx^2+dy^2) 계산 -> NFP 양방향 선계산
//!     -> 피스별 선행 참조 리스트 작성 -> 부착 그래프 완성
//!   결과: 부착 대상이 배치될 때까지 대기하는 "체인" 이 생기고,
//!         배치 시점에는 NFP 를 **이진탐색으로 조회만** 한다.
//! ```
//!
//! # 이 모듈이 정하는 것과 정하지 않는 것
//!
//! **정한다**: 무엇이 무엇에 붙는가(부착 그래프), 어느 것이 먼저 놓이는가(표면 키),
//! 첫 조각이 어디서 출발하는가(루트 시드), 어떤 NFP 쌍이 필요한가(쌍 목록).
//!
//! **정하지 않는다**: NFP 를 실제로 어떻게 계산하는가. 그건 기하 커널의 일이고,
//! 이 저장소에서 성능·정확성이 가장 민감한 코드다. 여기서는 **쌍을 열거하고 색인을
//! 만들 뿐**이고, 계산은 호출자가 넘긴 함수가 한다([`NfpPairIndex::precompute`]).
//! 커널을 포크하지 않는 것이 이 코드베이스의 확립된 규칙이다.

use std::collections::{BTreeMap, HashMap};

use crate::refplace::record::{PieceRecord, PlaceState, RotationEntry, SurfaceRecord};

// ────────────────────────────────────────────────────────────────────────────
// 그룹 정의 (§1 원칙 1)
// ────────────────────────────────────────────────────────────────────────────

/// 사용자가 정의한 번들·그룹 하나.
///
/// 설계 §1 원칙 1: **"순서는 형상 크기 정렬이 아니라 그룹 정의 기반"** — 같은 마커·묶음이
/// 곧 배치 순서의 뼈대다.
///
/// 입력 JSON 의 아이템별 `domain` 블록(`family_id` / `size_group_id` / `piece_name` /
/// `material`)이 이 그룹의 재료다. 그 필드들은 지금 솔버 로더가 통째로 버리므로
/// (jagua-rs `ExtItem` 이 파싱하지 않는다), 호출자가 따로 읽어 넘겨야 한다.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GroupSpec {
    /// 그룹 식별자. 정렬이 동률일 때 **결정론적 타이브레이크**로도 쓴다.
    pub id: String,
    /// 이 그룹에 속한 피스 레코드 인덱스. 문서 순서를 그대로 보존한다 —
    /// 그 순서가 곧 연속 쌍 `(i, i+1)` 의 순서이고, 체인의 모양을 정한다.
    pub members: Vec<usize>,
}

impl GroupSpec {
    pub fn new(id: impl Into<String>, members: Vec<usize>) -> Self {
        Self {
            id: id.into(),
            members,
        }
    }
}

/// 그룹 정의가 없을 때 쓸 대체 규칙.
///
/// 설계는 그룹을 전제하지만, 실제 입력에 그룹 메타데이터가 없을 수 있다. 그때
/// **조용히 크기 정렬로 돌아가면 안 된다** — 그러면 설계의 제1원칙이 소리 없이
/// 사라지고, 나중에 "왜 그룹이 안 먹지" 를 추적할 수 없다.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum GroupFallback {
    /// 피스 하나가 곧 그룹 하나. 체인이 생기지 않고 전부 독립 배치가 된다.
    ///
    /// 기본값이다. **동작이 크게 달라지는 대체를 기본으로 두지 않는다.**
    #[default]
    EachPieceAlone,
    /// 같은 `item_id` 사본끼리 한 그룹. 수요가 2 이상인 조각이 함께 배치된다.
    ByItemId,
}

// ────────────────────────────────────────────────────────────────────────────
// 표면 정렬 키 (§모듈 4)
// ────────────────────────────────────────────────────────────────────────────

/// 표면 정렬 키의 고정소수 배율.
///
/// 설계 §모듈 4 의 산출 체인은 `피스 bbox w²+h² 최대값 -> SQRT -> 64비트 정수 변환
/// 헬퍼로 고정소수화` 다. 배율 자체는 문서에 없으므로 여기서 정한다.
///
/// `1000` = 1 µm 눈금. mm 좌표에서 f64 잡음보다 훨씬 크고 기하 해상도보다 훨씬 작다.
/// **정수로 떨어뜨리는 것이 핵심**이다 — 부동소수 키로 정렬하면 같은 입력이
/// 훑는 순서에 따라 다른 순서를 낼 수 있다(이 저장소가 병렬 결정론에서 확인한 함정).
pub const SORT_KEY_SCALE: f64 = 1000.0;

/// 피스 bbox `(w, h)` 에서 대각 제곱을 만든다. 설계의 `w²+h²`.
#[inline]
pub fn diagonal_squared(w: f64, h: f64) -> f64 {
    w * w + h * h
}

/// 설계 §모듈 4 의 산출 체인 그대로: `max(w²+h²) -> sqrt -> 고정소수 i64`.
///
/// 비유한 값은 `0` 이다 — 정렬 키에 `NaN` 이 섞이면 전순서가 깨진다.
pub fn surface_sort_key(max_diagonal_squared: f64) -> i64 {
    if !max_diagonal_squared.is_finite() || max_diagonal_squared <= 0.0 {
        return 0;
    }
    (max_diagonal_squared.sqrt() * SORT_KEY_SCALE).round() as i64
}

// ────────────────────────────────────────────────────────────────────────────
// NFP 쌍 색인 (§모듈 3 선계산, §모듈 6 이진탐색 조회)
// ────────────────────────────────────────────────────────────────────────────

/// 선계산할 NFP 쌍의 키.
///
/// **방향이 있다.** 설계가 "NFP **양방향** 선계산" 이라고 적는 이유는
/// `NFP(고정 A, 이동 B)` 와 `NFP(고정 B, 이동 A)` 가 서로 점대칭이라 하나로부터 다른
/// 하나를 얻을 수 있지만, 조회 시점에 부호를 뒤집는 일을 하지 않으려는 것이다.
/// 배치 스텝은 **조회만** 해야 한다.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct NfpPairKey {
    /// 고정 조각의 아이템 id.
    pub fixed_item: usize,
    /// 고정 조각의 자세.
    pub fixed_pose: PoseKey,
    /// 이동 조각의 아이템 id.
    pub moving_item: usize,
    /// 이동 조각의 자세.
    pub moving_pose: PoseKey,
}

/// [`RotationEntry`] 를 정렬 가능한 정수 키로 눕힌 것.
///
/// `RotationEntry` 를 그대로 쓰면 `Ord` 를 손으로 정의해야 하고, 그 순서가 곧
/// 이진탐색의 전제가 된다. 정수 하나로 눕히면 그 위험이 사라진다.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PoseKey(pub i64);

impl PoseKey {
    /// 자세를 정수로 눕힌다. 반전 두 비트 + 밀리도.
    pub fn of(entry: &RotationEntry) -> Self {
        let flips = i64::from(u8::from(entry.flip_x)) | (i64::from(u8::from(entry.flip_y)) << 1);
        // 밀리도는 `360_000` 주기로 정규화한다. `-90_000` 과 `270_000` 이 같은 자세인데
        // 다른 키가 되면 NFP 준비본이 두 벌 생긴다.
        let deg = i64::from(entry.millideg).rem_euclid(360_000);
        Self(deg * 4 + flips)
    }
}

/// 선계산된 NFP 쌍 색인.
///
/// 키를 **정렬해 보관**하고 이진탐색으로 조회한다 — 설계 §모듈 6 이 "배치 시점에는
/// NFP 를 이진탐색으로 조회만 한다" 고 정한 그대로다. 해시맵이 아니라 정렬 배열인 것이
/// 의도적이다: 조회 순서가 결정론적이고, 캐시 지역성이 좋고, 무엇보다 **조회 시점에
/// 새 항목이 생길 수 없다**(선계산이 끝났다는 뜻).
#[derive(Clone, Debug, Default)]
pub struct NfpPairIndex<T> {
    keys: Vec<NfpPairKey>,
    values: Vec<T>,
}

/// 조회 결과 상태. 설계 §모듈 6 의 "NFP 판독 상태 코드" 에 대응한다.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NfpLookup {
    /// 정상 — 경계 배치로 간다.
    Ready,
    /// 정점 0 — 이 쌍은 붙을 수 없다. 부착을 해제한다.
    Empty,
    /// 전체 겹침 — 어느 자리에도 못 놓는다. 부착을 해제한다.
    FullOverlap,
    /// 선계산에 없다. 체인 구축이 빠뜨린 쌍이라는 뜻이므로 **결함 신호**다.
    Missing,
}

impl<T> NfpPairIndex<T> {
    /// 빈 색인.
    pub fn new() -> Self {
        Self {
            keys: Vec::new(),
            values: Vec::new(),
        }
    }

    /// 담긴 쌍 수.
    pub fn len(&self) -> usize {
        self.keys.len()
    }

    pub fn is_empty(&self) -> bool {
        self.keys.is_empty()
    }

    /// 이진탐색 조회. **선계산 이후에는 이것만 쓴다.**
    pub fn get(&self, key: &NfpPairKey) -> Option<&T> {
        self.keys.binary_search(key).ok().map(|i| &self.values[i])
    }

    /// Add an already-computed attachment batch entry. Lookup remains read-only;
    /// Draft calls this only in preparation, never while evaluating positions.
    pub(crate) fn insert_prepared(&mut self, key: NfpPairKey, value: T) -> Option<T> {
        match self.keys.binary_search(&key) {
            Ok(i) => Some(std::mem::replace(&mut self.values[i], value)),
            Err(i) => {
                self.keys.insert(i, key);
                self.values.insert(i, value);
                None
            }
        }
    }

    pub(crate) fn values(&self) -> impl Iterator<Item = &T> {
        self.values.iter()
    }

    /// 색인이 정렬돼 있는가. 이진탐색의 전제라 테스트가 확인한다.
    pub fn is_sorted(&self) -> bool {
        self.keys.windows(2).all(|w| w[0] < w[1])
    }
}

/// 선계산할 쌍 목록. 체인 구축이 만들고, 그 뒤에 값이 채워진다.
#[derive(Clone, Debug, Default)]
pub struct NfpPairPlan {
    /// 정렬·중복 제거된 쌍 목록.
    pub pairs: Vec<NfpPairKey>,
    /// 쌍마다의 표면 중심 거리(mm). 설계 §모듈 3 의 `sqrt(dx²+dy²)`.
    ///
    /// 계산 순서를 정하는 데 쓴다 — 가까운 쌍이 실제로 붙을 가능성이 높으므로,
    /// 예산이 모자라면 먼 쌍부터 버린다.
    pub center_distance: Vec<f64>,
}

impl NfpPairPlan {
    /// 쌍마다 `compute` 를 불러 색인을 만든다.
    ///
    /// `compute` 가 `None` 을 돌려주면 그 쌍은 색인에 들어가지 않고, 조회 시
    /// [`NfpLookup::Missing`] 이 된다 — 호출부가 부착을 해제하면 된다.
    ///
    /// **여기가 유일한 계산 지점**이다. 배치 스텝은 조회만 한다.
    pub fn precompute<T>(
        &self,
        mut compute: impl FnMut(&NfpPairKey) -> Option<T>,
    ) -> NfpPairIndex<T> {
        let mut keys = Vec::with_capacity(self.pairs.len());
        let mut values = Vec::with_capacity(self.pairs.len());
        // `pairs` 는 이미 정렬·중복 제거돼 있으므로 그대로 넣으면 정렬 불변이 유지된다.
        for key in &self.pairs {
            if let Some(v) = compute(key) {
                keys.push(*key);
                values.push(v);
            }
        }
        NfpPairIndex { keys, values }
    }

    /// 중심 거리가 가까운 순으로 쌍을 훑는 순서.
    ///
    /// 예산이 모자랄 때 먼 쌍을 버리기 위한 것이다. 거리가 같으면 키 순서로 갈라
    /// **결정론**을 유지한다.
    pub fn by_distance(&self) -> Vec<usize> {
        let mut order: Vec<usize> = (0..self.pairs.len()).collect();
        order.sort_by(|&a, &b| {
            let da = self.center_distance.get(a).copied().unwrap_or(f64::MAX);
            let db = self.center_distance.get(b).copied().unwrap_or(f64::MAX);
            // 비유한 값은 가장 멀다고 본다 — `total_cmp` 가 NaN 을 끝으로 보낸다.
            da.total_cmp(&db)
                .then_with(|| self.pairs[a].cmp(&self.pairs[b]))
        });
        order
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 체인 구축
// ────────────────────────────────────────────────────────────────────────────

/// 부착 체인 구축 결과.
#[derive(Clone, Debug, Default)]
pub struct AttachChain {
    /// 표면 키 **내림차순**으로 정렬된 표면 인덱스.
    ///
    /// 설계 §모듈 5: "키 내림차순 정렬" = 큰 조각 우선.
    pub surface_order: Vec<usize>,
    /// 체인 루트 — 선행 참조가 없어 독립 배치가 되는 피스들. 표면 순서를 따른다.
    ///
    /// 설계 §모듈 3: "표면 키 내림차순 정렬의 **맨 앞 피스가 첫 배치(루트)**".
    /// 즉 `roots[0]` 이 첫 배치다.
    pub roots: Vec<usize>,
    /// 선계산할 NFP 쌍.
    pub plan: NfpPairPlan,
}

impl AttachChain {
    /// 첫 배치가 될 피스. 루트가 없으면 `None`(놓을 것이 없다).
    pub fn first_placement(&self) -> Option<usize> {
        self.roots.first().copied()
    }
}

/// 체인 구축에 필요한, 피스별 기하 요약.
///
/// 실제 폴리곤을 여기까지 끌고 오지 않는다 — 체인 구축은 **위상**의 일이고, 필요한
/// 기하는 bbox 뿐이다. 그래야 이 모듈이 기하 커널 없이 테스트된다.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PieceExtent {
    /// bbox 폭(mm).
    pub width: f64,
    /// bbox 높이(mm).
    pub height: f64,
}

impl PieceExtent {
    pub fn new(width: f64, height: f64) -> Self {
        Self { width, height }
    }
}

/// 그룹 정의에서 표면을 재료화하고 부착 체인을 세운다.
///
/// # 순서
///
/// 1. 그룹마다 표면 하나를 만들고 멤버를 넣는다 (§모듈 3 "그룹 정의에서 재료화").
/// 2. 표면 키를 계산한다 — 멤버 bbox 대각의 최댓값 (§모듈 4).
/// 3. 표면을 키 **내림차순**으로 정렬한다 (§모듈 5).
/// 4. 각 표면 안에서 **연속 쌍 `(i, i+1)`** 을 돌며 바인딩과 선행 참조를 기록한다.
/// 5. 쌍마다 중심 거리를 재고 **양방향** NFP 쌍을 계획에 넣는다.
/// 6. 선행 참조가 없는 피스를 루트로 모은다.
///
/// # 왜 연속 쌍인가
///
/// 설계가 "체인은 표면 연속쌍으로만 이어진다" 고 못박는다. 그룹 안의 모든 조합을
/// 잇지 않는 이유는 NFP 선계산 비용이 `O(n²)` 로 폭발하기 때문이다. 연속 쌍이면
/// `O(n)` 이고, 그룹 정의 순서가 곧 "누가 누구 옆에 오는가" 라는 사용자 의도다.
pub fn build_chain(
    pieces: &mut [PieceRecord],
    extents: &[PieceExtent],
    groups: &[GroupSpec],
    surfaces: &mut Vec<SurfaceRecord>,
    container: (f64, f64),
) -> AttachChain {
    debug_assert_eq!(pieces.len(), extents.len());

    // ── 1) 그룹 -> 표면 ───────────────────────────────────────────────────
    surfaces.clear();
    let mut group_of_surface: Vec<&GroupSpec> = Vec::with_capacity(groups.len());
    for group in groups {
        let mut surface = SurfaceRecord::new(container.0, container.1);
        surface.pieces = group
            .members
            .iter()
            .copied()
            .filter(|&i| i < pieces.len())
            .collect();
        // ── 2) 표면 키 = 멤버 bbox 대각의 최댓값 ──
        let max_d2 = surface
            .pieces
            .iter()
            .map(|&i| diagonal_squared(extents[i].width, extents[i].height))
            .fold(0.0f64, f64::max);
        surface.sort_key = surface_sort_key(max_d2) as f64;
        // 시드 기본값 = 컨테이너 원점(좌하단). LOCKED 조각이 있으면 그 좌표로 덮는다.
        surface.seed = (0.0, 0.0);
        for &i in &surface.pieces {
            if pieces[i].state == PlaceState::Locked {
                if let Some(pose) = pieces[i].pose {
                    surface.seed = (pose.x, pose.y);
                    break;
                }
            }
        }
        surfaces.push(surface);
        group_of_surface.push(group);
    }

    // ── 3) 표면 키 내림차순 ───────────────────────────────────────────────
    let mut surface_order: Vec<usize> = (0..surfaces.len()).collect();
    surface_order.sort_by(|&a, &b| {
        surfaces[b]
            .sort_key
            .total_cmp(&surfaces[a].sort_key)
            // 동률이면 그룹 id 로 갈라 **결정론**을 유지한다. 인덱스로 갈라도 되지만
            // 그러면 입력 순서가 바뀔 때 결과가 바뀐다 — id 가 더 안정적이다.
            .then_with(|| group_of_surface[a].id.cmp(&group_of_surface[b].id))
    });

    // ── 4·5) 연속 쌍 -> 바인딩 · 선행 참조 · NFP 쌍 ───────────────────────
    let mut pair_set: BTreeMap<NfpPairKey, f64> = BTreeMap::new();
    for &s in &surface_order {
        let members = surfaces[s].pieces.clone();
        for w in members.windows(2) {
            let (a, b) = (w[0], w[1]);
            // 바인딩 참조 — 서로를 가리킨다. **조건 없이** 기록한다(설계 §모듈 3).
            pieces[a].bound_with = Some(b);
            pieces[b].bound_with = Some(a);

            // ── 부착은 **인덱스 순서**로만 성립한다 ──
            //
            // 설계 §모듈 3(갱신): "부착은 참조 피스가 자기보다 앞 인덱스일 때만
            // 성립한다 **(배치 상태가 아니라 인덱스 순서 기준)**". 그리고 "참조 피스가
            // 자기보다 뒤 인덱스(자기보다 먼저 처리될 수 없는 순서)인 피스는 독립 배치
            // 후보가 된다".
            //
            // 왜 인덱스인가: 그룹 멤버가 인덱스 오름차순으로 들어온다는 보장이 없다.
            // 배치 상태로 판정하면 `a -> b -> a` 같은 **순환**이 생길 수 있고, 그러면
            // 두 조각이 서로를 기다리며 영영 안 놓인다. 인덱스는 전순서라 순환이
            // 구조적으로 불가능하다 — 체인이 항상 비순환임을 **증명 없이 보장**한다.
            //
            // 뒤 인덱스를 참조하는 쌍은 부착을 **만들지 않는다**. 그 조각은 루트가 되어
            // 독립 배치되고, 쌍은 바인딩 참조로만 남는다.
            if a >= b {
                continue;
            }
            pieces[b].attach_to = Some(a);
            if !pieces[b].predecessors.contains(&a) {
                pieces[b].predecessors.push(a);
            }

            // 표면 쌍 중심 거리 — 설계의 sqrt(dx²+dy²).
            //
            // 아직 배치 전이라 "중심" 은 bbox 크기에서 만든 대표값이다. 절대 위치가
            // 아니라 **쌍 사이의 상대 크기 차이**를 재는 값이고, 그 용도는 선계산
            // 순서를 정하는 것뿐이다.
            let dx = extents[a].width - extents[b].width;
            let dy = extents[a].height - extents[b].height;
            let dist = (dx * dx + dy * dy).sqrt();

            // **양방향** 선계산 — 조회 시점에 부호를 뒤집는 일을 하지 않기 위해서다.
            for (fixed, moving) in [(a, b), (b, a)] {
                for f_pose in pieces[fixed].rotations.entries().to_vec() {
                    for m_pose in pieces[moving].rotations.entries().to_vec() {
                        let key = NfpPairKey {
                            fixed_item: pieces[fixed].item_id,
                            fixed_pose: PoseKey::of(&f_pose),
                            moving_item: pieces[moving].item_id,
                            moving_pose: PoseKey::of(&m_pose),
                        };
                        // 같은 (아이템, 자세) 쌍은 사본이 달라도 NFP 가 같다.
                        // `BTreeMap` 이 중복을 걷어내고 정렬까지 해 준다 —
                        // 그 정렬이 곧 이진탐색의 전제다.
                        pair_set.entry(key).or_insert(dist);
                    }
                }
            }
        }
    }

    let mut plan = NfpPairPlan::default();
    plan.pairs.reserve(pair_set.len());
    plan.center_distance.reserve(pair_set.len());
    for (key, dist) in pair_set {
        plan.pairs.push(key);
        plan.center_distance.push(dist);
    }

    // ── 6) 루트 ───────────────────────────────────────────────────────────
    //
    // 설계 §모듈 3(갱신): "선행 참조가 없거나 참조 피스가 **자기보다 뒤 인덱스**
    // (자기보다 먼저 처리될 수 없는 순서)인 피스는 독립 배치 후보".
    //
    // 위 루프가 이미 뒤 인덱스 참조로는 부착을 만들지 않으므로, 두 조건이 여기서
    // 합쳐진다. 그래도 **두 조건을 다 쓴다** — 나중에 다른 경로가 선행 참조를 추가할 때
    // 이 판정이 조용히 틀리지 않도록.
    //
    // 순서는 표면 순서를 따른다 — 그래야 `roots[0]` 이 "표면 키 내림차순 맨 앞 피스",
    // 즉 첫 배치가 된다.
    let mut roots = Vec::new();
    for &s in &surface_order {
        for &i in &surfaces[s].pieces {
            let independent =
                pieces[i].predecessors.is_empty() || pieces[i].predecessors.iter().all(|&q| q > i);
            if independent {
                roots.push(i);
            }
        }
    }

    AttachChain {
        surface_order,
        roots,
        plan,
    }
}

/// 그룹 정의가 없을 때 대체 그룹을 만든다.
///
/// [`GroupFallback`] 문서 참조 — **조용히 크기 정렬로 돌아가지 않는다.**
pub fn fallback_groups(pieces: &[PieceRecord], rule: GroupFallback) -> Vec<GroupSpec> {
    match rule {
        GroupFallback::EachPieceAlone => pieces
            .iter()
            .enumerate()
            .map(|(i, p)| GroupSpec::new(format!("solo-{}-{}", p.item_id, p.copy_index), vec![i]))
            .collect(),
        GroupFallback::ByItemId => {
            // `BTreeMap` 이라 그룹 순서가 item_id 순으로 **결정론적**이다.
            let mut by_item: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
            for (i, p) in pieces.iter().enumerate() {
                by_item.entry(p.item_id).or_default().push(i);
            }
            by_item
                .into_iter()
                .map(|(item, members)| GroupSpec::new(format!("item-{item}"), members))
                .collect()
        }
    }
}

/// 입력의 `domain` 메타데이터에서 그룹을 만든다.
///
/// 키는 `(family_id, size_group_id, material)` 이다 — 같은 패턴 조각의 같은 사이즈,
/// 같은 원단이 한 묶음이다. 이 필드들은 아이템 순서대로 넘어와야 하고, 없는 아이템은
/// `None` 으로 둔다(그러면 `fallback` 규칙이 그 조각을 처리한다).
///
/// 그룹 **안의 순서**는 입력 순서를 그대로 보존한다. 그 순서가 연속 쌍을 정하므로
/// 곧 체인의 모양이다 — 정렬로 바꾸면 사용자 의도가 사라진다.
pub fn groups_from_domain(
    pieces: &[PieceRecord],
    domain_of_item: &HashMap<usize, (String, String, String)>,
    fallback: GroupFallback,
) -> Vec<GroupSpec> {
    let mut grouped: BTreeMap<(String, String, String), Vec<usize>> = BTreeMap::new();
    let mut ungrouped: Vec<usize> = Vec::new();
    for (i, p) in pieces.iter().enumerate() {
        match domain_of_item.get(&p.item_id) {
            Some(key) => grouped.entry(key.clone()).or_default().push(i),
            None => ungrouped.push(i),
        }
    }
    let mut out: Vec<GroupSpec> = grouped
        .into_iter()
        .map(|((family, size, material), members)| {
            GroupSpec::new(format!("{family}|{size}|{material}"), members)
        })
        .collect();
    if !ungrouped.is_empty() {
        // 메타데이터가 없는 조각은 대체 규칙으로 처리한다. **조용히 버리지 않는다.**
        let subset: Vec<PieceRecord> = ungrouped.iter().map(|&i| pieces[i].clone()).collect();
        for mut spec in fallback_groups(&subset, fallback) {
            spec.members = spec.members.into_iter().map(|k| ungrouped[k]).collect();
            spec.id = format!("nodomain-{}", spec.id);
            out.push(spec);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn incremental_batches_keep_directional_binary_lookup_sorted() {
        let key = |a, b| NfpPairKey {
            fixed_item: a,
            fixed_pose: PoseKey(0),
            moving_item: b,
            moving_pose: PoseKey(360_000),
        };
        let mut index = NfpPairIndex::new();
        assert_eq!(index.insert_prepared(key(8, 2), Some(82)), None);
        index.insert_prepared(key(2, 8), None);
        index.insert_prepared(key(4, 1), Some(41));
        assert!(index.is_sorted());
        assert_eq!(index.get(&key(8, 2)), Some(&Some(82)));
        assert_eq!(index.get(&key(2, 8)), Some(&None));
        assert_eq!(index.get(&key(8, 3)), None);
        assert_eq!(index.insert_prepared(key(4, 1), Some(42)), Some(Some(41)));
        assert_eq!(index.len(), 3);
        assert_eq!(index.get(&key(4, 1)), Some(&Some(42)));
    }
    use crate::refplace::record::{PlacedPose, RotationList};

    fn piece(item_id: usize, copy_index: usize) -> PieceRecord {
        PieceRecord::new(item_id, copy_index, RotationList::fixed())
    }

    /// 표면 키는 설계의 산출 체인 그대로 — `max(w²+h²) -> sqrt -> 고정소수 정수`.
    #[test]
    fn the_surface_key_follows_the_documented_chain() {
        // 3-4-5 삼각형: w=3, h=4 이면 대각 5.
        let k = surface_sort_key(diagonal_squared(3.0, 4.0));
        assert_eq!(k, (5.0 * SORT_KEY_SCALE) as i64);
        // 비유한 값은 0 — NaN 이 정렬 키에 섞이면 전순서가 깨진다.
        assert_eq!(surface_sort_key(f64::NAN), 0);
        assert_eq!(surface_sort_key(-1.0), 0);
    }

    /// 표면은 키 **내림차순**이고, 맨 앞 피스가 첫 배치(루트)다.
    #[test]
    fn surfaces_sort_descending_and_the_front_piece_is_the_first_placement() {
        let mut pieces = vec![piece(0, 0), piece(1, 0), piece(2, 0)];
        let extents = vec![
            PieceExtent::new(10.0, 10.0),   // 작다
            PieceExtent::new(100.0, 100.0), // 크다
            PieceExtent::new(50.0, 50.0),
        ];
        let groups = vec![
            GroupSpec::new("small", vec![0]),
            GroupSpec::new("big", vec![1]),
            GroupSpec::new("mid", vec![2]),
        ];
        let mut surfaces = Vec::new();
        let chain = build_chain(
            &mut pieces,
            &extents,
            &groups,
            &mut surfaces,
            (1000.0, 500.0),
        );

        // 큰 것부터.
        let keys: Vec<f64> = chain
            .surface_order
            .iter()
            .map(|&s| surfaces[s].sort_key)
            .collect();
        assert!(
            keys[0] > keys[1] && keys[1] > keys[2],
            "내림차순이어야 한다: {keys:?}"
        );
        // 맨 앞 표면의 피스가 첫 배치.
        assert_eq!(chain.first_placement(), Some(1), "가장 큰 조각이 첫 배치다");
    }

    /// 체인은 **연속 쌍으로만** 이어진다 — 그룹 안 모든 조합이 아니다.
    ///
    /// 조합으로 이으면 NFP 선계산이 O(n²) 로 폭발한다.
    #[test]
    fn the_chain_links_consecutive_pairs_only() {
        let mut pieces = vec![piece(0, 0), piece(1, 0), piece(2, 0), piece(3, 0)];
        let extents = vec![PieceExtent::new(10.0, 10.0); 4];
        let groups = vec![GroupSpec::new("g", vec![0, 1, 2, 3])];
        let mut surfaces = Vec::new();
        let chain = build_chain(
            &mut pieces,
            &extents,
            &groups,
            &mut surfaces,
            (100.0, 100.0),
        );

        // 0 <- 1 <- 2 <- 3 사슬. 0 만 선행 참조가 없다.
        assert!(pieces[0].predecessors.is_empty());
        assert_eq!(pieces[1].predecessors, vec![0]);
        assert_eq!(pieces[2].predecessors, vec![1]);
        assert_eq!(pieces[3].predecessors, vec![2]);
        assert_eq!(pieces[1].attach_to, Some(0));
        assert_eq!(chain.roots, vec![0], "사슬의 머리 하나만 루트다");

        // 연속 쌍 3개 x 양방향 = 아이템 쌍 6개. 조합이었다면 6쌍 x 2 = 12 였을 것이다.
        assert_eq!(chain.plan.pairs.len(), 6);
    }

    /// NFP 는 **양방향**으로 선계산된다.
    #[test]
    fn nfp_pairs_are_precomputed_in_both_directions() {
        let mut pieces = vec![piece(7, 0), piece(9, 0)];
        let extents = vec![PieceExtent::new(10.0, 20.0), PieceExtent::new(30.0, 40.0)];
        let groups = vec![GroupSpec::new("g", vec![0, 1])];
        let mut surfaces = Vec::new();
        let chain = build_chain(
            &mut pieces,
            &extents,
            &groups,
            &mut surfaces,
            (100.0, 100.0),
        );

        let pose = PoseKey::of(&RotationEntry::IDENTITY);
        let fwd = NfpPairKey {
            fixed_item: 7,
            fixed_pose: pose,
            moving_item: 9,
            moving_pose: pose,
        };
        let rev = NfpPairKey {
            fixed_item: 9,
            fixed_pose: pose,
            moving_item: 7,
            moving_pose: pose,
        };
        assert!(chain.plan.pairs.contains(&fwd));
        assert!(chain.plan.pairs.contains(&rev));
    }

    /// 색인은 **정렬**돼 있어야 한다 — 이진탐색의 전제다.
    #[test]
    fn the_index_is_sorted_so_binary_search_is_valid() {
        let mut pieces = vec![piece(5, 0), piece(2, 0), piece(8, 0)];
        let extents = vec![PieceExtent::new(10.0, 10.0); 3];
        let groups = vec![GroupSpec::new("g", vec![0, 1, 2])];
        let mut surfaces = Vec::new();
        let chain = build_chain(
            &mut pieces,
            &extents,
            &groups,
            &mut surfaces,
            (100.0, 100.0),
        );

        let index = chain
            .plan
            .precompute(|k| Some(k.fixed_item + k.moving_item));
        assert!(
            index.is_sorted(),
            "정렬이 깨지면 조회가 조용히 틀린 값을 준다"
        );
        assert_eq!(index.len(), chain.plan.pairs.len());
        // 조회가 실제로 맞는 값을 준다.
        for key in &chain.plan.pairs {
            assert_eq!(index.get(key), Some(&(key.fixed_item + key.moving_item)));
        }
    }

    /// 선계산에서 빠진 쌍은 조회에서 **없음**으로 나온다 — 조용히 계산하지 않는다.
    #[test]
    fn a_pair_dropped_at_precompute_time_is_simply_absent() {
        let mut pieces = vec![piece(1, 0), piece(2, 0)];
        let extents = vec![PieceExtent::new(10.0, 10.0); 2];
        let groups = vec![GroupSpec::new("g", vec![0, 1])];
        let mut surfaces = Vec::new();
        let chain = build_chain(
            &mut pieces,
            &extents,
            &groups,
            &mut surfaces,
            (100.0, 100.0),
        );

        // 전부 실패시킨다.
        let index: NfpPairIndex<()> = chain.plan.precompute(|_| None);
        assert!(index.is_empty());
        assert!(index.get(&chain.plan.pairs[0]).is_none());
    }

    /// 자세 키는 `360_000` 밀리도 주기로 정규화된다 — 같은 자세가 두 키가 되면 안 된다.
    #[test]
    fn equivalent_poses_collapse_to_the_same_key() {
        let a = RotationEntry::turn(270_000);
        let b = RotationEntry::turn(-90_000);
        assert_eq!(PoseKey::of(&a), PoseKey::of(&b));
        // 반전이 다르면 다른 키다.
        let flipped = RotationEntry { flip_x: true, ..a };
        assert_ne!(PoseKey::of(&a), PoseKey::of(&flipped));
    }

    /// LOCKED 조각의 좌표가 표면 시드가 된다 (§모듈 3).
    #[test]
    fn a_locked_piece_seeds_its_surface() {
        let mut pieces = vec![piece(0, 0), piece(1, 0)];
        pieces[1].state = PlaceState::Locked;
        pieces[1].pose = Some(PlacedPose {
            rotation: RotationEntry::IDENTITY,
            extra_rad: 0.0,
            x: 123.0,
            y: 45.0,
        });
        let extents = vec![PieceExtent::new(10.0, 10.0); 2];
        let groups = vec![GroupSpec::new("g", vec![0, 1])];
        let mut surfaces = Vec::new();
        build_chain(
            &mut pieces,
            &extents,
            &groups,
            &mut surfaces,
            (900.0, 500.0),
        );
        assert_eq!(surfaces[0].seed, (123.0, 45.0));
    }

    /// 잠금이 없으면 시드는 컨테이너 원점(좌하단)이다.
    #[test]
    fn without_a_lock_the_seed_is_the_container_origin() {
        let mut pieces = vec![piece(0, 0)];
        let extents = vec![PieceExtent::new(10.0, 10.0)];
        let groups = vec![GroupSpec::new("g", vec![0])];
        let mut surfaces = Vec::new();
        build_chain(
            &mut pieces,
            &extents,
            &groups,
            &mut surfaces,
            (900.0, 500.0),
        );
        assert_eq!(surfaces[0].seed, (0.0, 0.0));
    }

    /// 그룹 정의가 없을 때 **조용히 크기 정렬로 돌아가지 않는다.**
    #[test]
    fn the_group_fallback_does_not_silently_become_size_sorting() {
        let pieces = vec![piece(0, 0), piece(0, 1), piece(1, 0)];
        // 기본 대체 = 각자 하나씩. 체인이 생기지 않는다.
        let solo = fallback_groups(&pieces, GroupFallback::default());
        assert_eq!(solo.len(), 3);
        assert!(solo.iter().all(|g| g.members.len() == 1));
        // item_id 대체는 사본을 묶는다.
        let by_item = fallback_groups(&pieces, GroupFallback::ByItemId);
        assert_eq!(by_item.len(), 2);
        assert_eq!(by_item[0].members, vec![0, 1]);
    }

    /// `domain` 메타데이터가 있는 조각은 묶이고, 없는 조각도 **버려지지 않는다**.
    #[test]
    fn pieces_without_domain_metadata_still_get_a_group() {
        let pieces = vec![piece(0, 0), piece(1, 0), piece(2, 0)];
        let mut domain = HashMap::new();
        domain.insert(0, ("piece-앞1".into(), "size-M".into(), "G".into()));
        domain.insert(1, ("piece-앞1".into(), "size-M".into(), "G".into()));
        let groups = groups_from_domain(&pieces, &domain, GroupFallback::EachPieceAlone);
        // 0,1 은 한 묶음. 2 는 대체 그룹.
        let total: usize = groups.iter().map(|g| g.members.len()).sum();
        assert_eq!(total, 3, "조각이 하나도 사라지면 안 된다");
        assert!(groups.iter().any(|g| g.members == vec![0, 1]));
        assert!(groups.iter().any(|g| g.id.starts_with("nodomain-")));
    }

    /// 거리 순서는 결정론적이다 — 동률이면 키로 가른다.
    #[test]
    fn the_distance_order_is_deterministic_on_ties() {
        let pose = PoseKey::of(&RotationEntry::IDENTITY);
        let mk = |f: usize, m: usize| NfpPairKey {
            fixed_item: f,
            fixed_pose: pose,
            moving_item: m,
            moving_pose: pose,
        };
        let plan = NfpPairPlan {
            pairs: vec![mk(3, 4), mk(1, 2), mk(5, 6)],
            center_distance: vec![1.0, 1.0, 1.0],
        };
        // 전부 같은 거리 -> 키 순서. mk(1,2) < mk(3,4) < mk(5,6).
        assert_eq!(plan.by_distance(), vec![1, 0, 2]);
    }
}
