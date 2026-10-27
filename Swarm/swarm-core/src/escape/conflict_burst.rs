//! Strip 최소값 근처에서 광범위한 충돌에 갇혔을 때의 돌파 전략
//! **Conflict Burst + Partial Rebuild**.
//!
//! ```text
//! 현재 Strip 최소값 근처
//!         ↓
//! 광범위한 Collision 발생
//!         ↓
//! 개별 Move / Rotate 실패 반복
//!         ↓
//! 충돌 그래프 생성
//!         ↓
//! 충돌 중심 패턴군 선택
//!         ↓
//! 일시적으로 Strip 제약 완화
//!         ↓
//! 패턴군 강제 분리
//!         ↓
//! 재삽입 + 재압축
//!         ↓
//! 다시 Strip 최소화
//! ```
//!
//! 핵심은 **Strip 최소값을 무조건 유지하려 하지 않는 것**이다.
//!
//! ```text
//! L → 충돌 정체 → L + ΔL 허용 → 충돌군 해체 → 재배치 → 다시 L 이하로 압축 시도
//! ```
//!
//! ΔL은 collision ratio에 따라 동적으로 준다.
//!
//! ```text
//! collision ratio < 10%   → ΔL = 1~2%
//! collision ratio 10~30%  → ΔL = 3~5%
//! collision ratio > 30%   → ΔL = 5~10%
//! ```
//!
//! **모든 패턴을 풀어버리지 않는다.** 각 패턴을 노드로 보고, 충돌하는 패턴끼리 edge를
//! 연결해 충돌 그래프를 만든다.
//!
//! ```text
//! A ─ B ─ C
//!     │   │
//!     D ─ E
//!
//! F ─ G
//! ```
//!
//! 큰 connected component인 `A~E`만 꺼내서 재배치하고, `F~G`처럼 안정적인 부분은 고정한다.
//!
//! 우선 제거 대상은 다음 다섯 축으로 점수화한다.
//!
//! ```text
//! ConflictEscapeScore(i)
//! =
//! 0.35 × CollisionDegree
//! + 0.25 × StripBoundaryContribution
//! + 0.20 × BlockingScore
//! + 0.10 × Area
//! + 0.10 × Concavity
//! ```
//!
//! 충돌 방향 기반 강제 이동은 겹친 두 패턴 사이의 최소 분리 벡터로 서로 밀어낸다.
//!
//! ```text
//! EscapeVector(i)
//! =
//! Σ w_ij × SeparationVector(i,j)
//! ```
//!
//! 작은 step으로 계속 튕기는 게 아니라 **의도적으로 큰 step**을 준다.
//!
//! ```text
//! 평소 Local Move    1~3 mm
//! Escape Mode        10~50 mm 또는 bbox 크기의 20~50%
//! ```
//!
//! 전체는 3단계다.
//!
//! ```text
//! Stage 1  Collision Push       → 충돌 벡터 기반 강제 분리
//! Stage 2  Strip Relaxation     → 현재 길이보다 3~8% 일시 확장
//! Stage 3  Destroy & Repair     → 충돌 중심 패턴 5~15% 제거 후 재삽입
//!                               → 이후 Left/Bottom Compaction
//! ```
//!
//! 탈출 직후 바로 원래 Strip 길이를 강제하면 다시 같은 상태로 돌아가므로 점진적으로 줄인다
//! (**Adaptive Strip Relaxation**).
//!
//! ```text
//! L + 6% → L + 4% → L + 2% → L → L - ε 시도
//! ```

use crate::config::RuinRecreateConfig;
use crate::escape::{
    COLLISION_RATIO_HIGH, COLLISION_RATIO_LOW, CONFLICT_REMOVE_RATIO_MAX,
    CONFLICT_REMOVE_RATIO_MIN, DELTA_L_HIGH, DELTA_L_LOW, DELTA_L_MID, ESCAPE_MOVE_BBOX_RATIO,
    ESCAPE_MOVE_MM, GUIDED_WEIGHT_NEUTRAL, LOCAL_MOVE_MM, STRIP_RELAXATION_EPSILON,
    STRIP_RELAXATION_LADDER, W_BLOCKING_SCORE, W_COLLISION_DEGREE, W_CONCAVITY, W_CONFLICT_AREA,
    W_STRIP_BOUNDARY_CONTRIBUTION, lerp_ratio,
};
use crate::nfp_lbf::{NfpKey, NfpLbfCache};
use crate::optimizer::lns::{BandTabu, RuinStrategy, ruin_and_recreate};
use crate::optimizer::separator::Separator;
use crate::quantify::tracker::{CTEntry, CollisionTracker};
use crate::util::terminator::Terminator;
use jagua_rs::entities::PItemKey;
use jagua_rs::geometry::primitives::Rect;
use jagua_rs::probs::spp::entities::SPSolution;
use slotmap::SecondaryMap;
use swarm_nfp::{PolygonSet, Ring};

/// 폭/스텝 비교에 쓰는 수치 허용 오차(mm).
const GEOM_EPS: f32 = 1.0e-6;

/// Stage 1에서 밀어낼 아이템 수의 기본 상한.
const DEFAULT_MAX_PUSH_ITEMS: usize = 8;

// ────────────────────────────────────────────────────────────────────────────
// Adaptive Strip Relaxation — collision ratio 구간
// ────────────────────────────────────────────────────────────────────────────

/// collision ratio 구간. 구간마다 허용할 ΔL 범위가 다르다.
///
/// ```text
/// collision ratio < 10%   → ΔL = 1~2%
/// collision ratio 10~30%  → ΔL = 3~5%
/// collision ratio > 30%   → ΔL = 5~10%
/// ```
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CollisionRatioBand {
    /// `collision ratio < 10%`
    Low,
    /// `collision ratio 10~30%`
    Mid,
    /// `collision ratio > 30%`
    High,
}

impl CollisionRatioBand {
    /// [`COLLISION_RATIO_LOW`] 미만이면 `Low`, [`COLLISION_RATIO_HIGH`] 이하면 `Mid`,
    /// 그보다 크면 `High`. NaN은 가장 보수적인 `Low`로 취급한다.
    pub fn classify(collision_ratio: f32) -> Self {
        if collision_ratio.is_nan() || collision_ratio < COLLISION_RATIO_LOW {
            CollisionRatioBand::Low
        } else if collision_ratio <= COLLISION_RATIO_HIGH {
            CollisionRatioBand::Mid
        } else {
            CollisionRatioBand::High
        }
    }

    /// 구간별 ΔL 범위 `(min, max)`.
    pub fn delta_l_range(self) -> (f32, f32) {
        match self {
            CollisionRatioBand::Low => DELTA_L_LOW,
            CollisionRatioBand::Mid => DELTA_L_MID,
            CollisionRatioBand::High => DELTA_L_HIGH,
        }
    }

    /// 정체 강도 `t ∈ [0, 1]`로 구간 안을 선형 보간한 ΔL.
    pub fn delta_l(self, t: f32) -> f32 {
        lerp_ratio(self.delta_l_range(), t)
    }
}

/// collision ratio를 `t ∈ [0, 1]` 강도로 환산한다.
///
/// `COLLISION_RATIO_HIGH`(30%)에서 1.0으로 포화한다. 구간 안에서의 위치가 아니라
/// **전역 강도**이므로, 같은 band 안에서도 충돌이 많을수록 더 큰 ΔL·step을 받는다.
#[inline]
fn burst_intensity(collision_ratio: f32) -> f32 {
    if !collision_ratio.is_finite() {
        return 0.0;
    }
    (collision_ratio / COLLISION_RATIO_HIGH.max(GEOM_EPS)).clamp(0.0, 1.0)
}

// ────────────────────────────────────────────────────────────────────────────
// ConflictEscapeScore
// ────────────────────────────────────────────────────────────────────────────

/// ConflictEscapeScore의 다섯 항. 모두 `0.0~1.0`으로 정규화되어 있다.
///
/// ```text
/// ConflictEscapeScore(i)
/// =
/// 0.35 × CollisionDegree
/// + 0.25 × StripBoundaryContribution
/// + 0.20 × BlockingScore
/// + 0.10 × Area
/// + 0.10 × Concavity
/// ```
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ConflictEscapeScoreTerms {
    /// 충돌 그래프 차수를 그래프 내 최대 차수로 정규화한 값.
    pub collision_degree: f32,
    /// bbox 우측 끝이 strip 폭의 어디까지 밀고 있는지 (`0` = 좌측, `1` = 우측 경계).
    pub strip_boundary_contribution: f32,
    /// 이동 공간을 막는 정도. bbox 대각(크기)과 길쭉함(elongation)의 평균.
    pub blocking_score: f32,
    /// 배치된 아이템 중 최대 면적 대비 면적비.
    pub area: f32,
    /// bbox를 얼마나 못 채우는지 = `1 − area / bbox.area`. 오목할수록 1에 가깝다.
    pub concavity: f32,
}

impl ConflictEscapeScoreTerms {
    /// 문서가 지정한 가중치를 그대로 곱해 더한다. 다섯 가중치의 합은 `1.00`이므로
    /// 결과도 `0.0~1.0` 범위다.
    pub fn score(&self) -> f32 {
        W_COLLISION_DEGREE * self.collision_degree
            + W_STRIP_BOUNDARY_CONTRIBUTION * self.strip_boundary_contribution
            + W_BLOCKING_SCORE * self.blocking_score
            + W_CONFLICT_AREA * self.area
            + W_CONCAVITY * self.concavity
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 충돌 그래프
// ────────────────────────────────────────────────────────────────────────────

/// 충돌 그래프의 connected component 하나. 멤버는 ConflictEscapeScore 내림차순이다.
#[derive(Clone, Debug)]
pub struct ConflictComponent {
    /// 이 성분에 속한 배치 키들 (점수 내림차순).
    pub members: Vec<PItemKey>,
    /// 멤버 ConflictEscapeScore의 합.
    pub total_score: f32,
}

/// 배치된 아이템을 노드로, 충돌하는 쌍을 edge로 갖는 그래프.
///
/// ```text
/// A ─ B ─ C
///     │   │
///     D ─ E
///
/// F ─ G
/// ```
///
/// [`Self::build`] 시점의 점수를 함께 보관하므로, 이후 배치가 바뀌어도
/// [`Self::components`]는 "그 시점의 충돌 구조"를 일관되게 돌려준다.
#[derive(Clone, Debug)]
pub struct ConflictGraph {
    /// 노드 인덱스 → 배치 키.
    keys: Vec<PItemKey>,
    /// 배치 키 → 노드 인덱스.
    index_of: SecondaryMap<PItemKey, usize>,
    /// 노드별 인접 노드 인덱스.
    adjacency: Vec<Vec<usize>>,
    /// 노드별 ConflictEscapeScore 항 (build 시점 스냅샷).
    terms: Vec<ConflictEscapeScoreTerms>,
    /// 충돌(쌍 또는 컨테이너)에 관여하는 노드 수 (build 시점 스냅샷).
    n_conflicting: usize,
    /// **정확한 NFP 기반 분리 벡터** (`② exact_separation_vector`).
    ///
    /// `adjacency`와 **같은 모양**의 병렬 배열이다. `exact_sep[i][k]`는
    /// `adjacency[i][k]`가 가리키는 이웃에 대한 값이며, `i`를 그 이웃에서 떼어내는
    /// 방향이다. `None`은 "그 쌍의 NFP를 못 얻었다"(→ bbox MTV 폴백)를 뜻하고,
    /// `Some((0,0))`은 "NFP 기준으로는 이미 떨어져 있다"를 뜻한다.
    ///
    /// [`ConflictGraph::build`](Self::build)로 만들면 **항상 비어 있다**.
    /// 비어 있으면 분리 벡터 경로가 기존(bbox MTV)과 한 비트도 다르지 않다.
    exact_sep: Vec<Vec<Option<(f32, f32)>>>,
}

impl ConflictGraph {
    /// `sep.ct`의 `pair_collisions[(i, j)].loss > 0`인 쌍마다 edge를 만든다.
    ///
    /// pair matrix는 상삼각 행이 **연속 구간**이므로 쌍마다 인덱스를 다시 계산하지 않고
    /// 행 슬라이스를 통째로 훑는다([`scan_pair_conflicts`]). `simd` feature가 켜지면
    /// 그 행 스캔이 벡터화된다.
    pub fn build(sep: &Separator) -> Self {
        Self::build_with_nfp(sep, None)
    }

    /// [`Self::build`]와 같되, `cache`가 `Some`이면 edge마다 **정확한 NFP 기반 분리
    /// 벡터**를 미리 계산해 그래프에 실어 둔다.
    ///
    /// # 왜 build 시점에 계산하나
    ///
    /// [`Self::separation_vector`]는 `&self`다. NFP 캐시는 miss 시 계산·저장을 해야 해서
    /// `&mut`가 필요하므로, 질의 시점에는 캐시를 들고 들어갈 수 없다. 그래서 캐시를 쓸 수
    /// 있는 유일한 시점(build)에 edge 수만큼만 질의해 결과를 스냅샷으로 보관한다.
    /// 무향 edge당 **한 번만** 질의하고 반대 방향은 부호만 뒤집는다
    /// (`NFP(A,B) = -NFP(B,A)`라 정확히 같은 값이다).
    ///
    /// `cache`가 `None`이면 [`Self::build`]와 **완전히 동일**하다. 호출자는
    /// [`crate::config::NfpIntegrationConfig::exact_separation_vector`]가 켜졌을 때만
    /// `Some`을 넘긴다.
    pub fn build_with_nfp(sep: &Separator, cache: Option<&mut NfpLbfCache>) -> Self {
        let layout = &sep.prob.layout();
        let keys: Vec<PItemKey> = layout.placed_items().keys().collect();
        let n = keys.len();

        let mut index_of: SecondaryMap<PItemKey, usize> = SecondaryMap::new();
        for (i, &pk) in keys.iter().enumerate() {
            index_of.insert(pk, i);
        }

        let ct = &sep.ct;
        let (edges, deg) = scan_pair_conflicts(ct, &keys);

        // 차수를 미리 알고 있으므로 정확한 용량으로 한 번에 잡는다.
        let mut adjacency: Vec<Vec<usize>> = deg
            .iter()
            .map(|&d| Vec::with_capacity(d as usize))
            .collect();
        for &(a, b) in &edges {
            adjacency[a as usize].push(b as usize);
            adjacency[b as usize].push(a as usize);
        }
        // 행 순서 스캔이라 삽입 순서가 노드 순서와 다를 수 있다. 전수 이중 루프와
        // **완전히 동일한** 오름차순 인접 리스트로 맞춰 둔다
        // (`escape_vector`의 f32 누산 순서까지 같아진다).
        for adj in adjacency.iter_mut() {
            adj.sort_unstable();
        }

        // "충돌에 관여" 판정에는 컨테이너 경계 침범도 포함한다. 우측 경계를 뚫고 나간
        // 아이템만 남은 상태를 collision ratio 0으로 보면 ΔL이 과소평가된다.
        let n_conflicting = (0..n)
            .filter(|&i| !adjacency[i].is_empty() || container_conflicting(ct, keys[i]))
            .count();

        let terms = compute_terms(sep, &keys, &adjacency);

        // 캐시가 없으면 빈 배열 그대로 — 분리 벡터는 기존 bbox MTV 경로를 탄다.
        let exact_sep = match cache {
            Some(cache) => compute_exact_separations(sep, &keys, &adjacency, cache),
            None => Vec::new(),
        };

        Self {
            keys,
            index_of,
            adjacency,
            terms,
            n_conflicting,
            exact_sep,
        }
    }

    /// 노드의 충돌 차수. 그래프에 없는 키는 0.
    pub fn degree(&self, pk: PItemKey) -> usize {
        self.index_of
            .get(pk)
            .map(|&i| self.adjacency[i].len())
            .unwrap_or(0)
    }

    /// connected component들을 `total_score` 내림차순으로 돌려준다.
    /// 단일 노드(고립 아이템)는 제외한다.
    pub fn components(&self) -> Vec<ConflictComponent> {
        let n = self.keys.len();
        let mut parent: Vec<usize> = (0..n).collect();
        for (i, neighbours) in self.adjacency.iter().enumerate() {
            for &j in neighbours {
                let ri = uf_find(&mut parent, i);
                let rj = uf_find(&mut parent, j);
                if ri != rj {
                    parent[ri] = rj;
                }
            }
        }

        // root → 성분 인덱스
        let mut slot: Vec<Option<usize>> = vec![None; n];
        let mut buckets: Vec<Vec<usize>> = Vec::new();
        for i in 0..n {
            if self.adjacency[i].is_empty() {
                continue;
            }
            let root = uf_find(&mut parent, i);
            let existing = slot[root];
            let idx = match existing {
                Some(idx) => idx,
                None => {
                    buckets.push(Vec::new());
                    slot[root] = Some(buckets.len() - 1);
                    buckets.len() - 1
                }
            };
            buckets[idx].push(i);
        }

        let mut components: Vec<ConflictComponent> = buckets
            .into_iter()
            .filter(|nodes| nodes.len() >= 2)
            .map(|mut nodes| {
                nodes.sort_by(|&a, &b| self.terms[b].score().total_cmp(&self.terms[a].score()));
                let total_score = nodes.iter().map(|&i| self.terms[i].score()).sum();
                ConflictComponent {
                    members: nodes.into_iter().map(|i| self.keys[i]).collect(),
                    total_score,
                }
            })
            .collect();

        components.sort_by(|a, b| b.total_score.total_cmp(&a.total_score));
        components
    }

    /// 멤버 수가 가장 많은 성분(동률이면 `total_score`가 큰 쪽).
    /// 문서의 `큰 connected component인 A~E만 꺼내서 재배치한다`에 대응한다.
    pub fn largest_component(&self) -> Option<ConflictComponent> {
        self.components().into_iter().max_by(|a, b| {
            a.members
                .len()
                .cmp(&b.members.len())
                .then_with(|| a.total_score.total_cmp(&b.total_score))
        })
    }

    /// `충돌에 관여하는 아이템 수 / 전체 배치 아이템 수`.
    pub fn collision_ratio(&self) -> f32 {
        if self.keys.is_empty() {
            return 0.0;
        }
        self.n_conflicting() as f32 / self.keys.len() as f32
    }

    /// 충돌(쌍 또는 컨테이너)에 관여하는 아이템 수. build 시점에 이미 세어 둔 값이다.
    pub fn n_conflicting(&self) -> usize {
        self.n_conflicting
    }

    /// 두 아이템의 최소 분리 벡터. `a`를 `b`에서 밀어내는 방향이며, 겹치지 않으면
    /// `(0.0, 0.0)`이다.
    ///
    /// [`Self::build_with_nfp`]로 만들었고 그 쌍의 NFP를 얻었다면 **정확한 NFP 기반**
    /// 값을, 그렇지 않으면 기존처럼 **bbox MTV 근사**를 쓴다. 오목한 의류 패턴에서
    /// bbox MTV는 실제 최소 분리 방향과 크게 어긋난다 — 축 정렬 두 방향밖에 못 내놓기
    /// 때문이다.
    pub fn separation_vector(&self, sep: &Separator, a: PItemKey, b: PItemKey) -> (f32, f32) {
        // 정확 경로가 꺼져 있으면(= 스냅샷이 비었으면) 여기서 즉시 None이 나와
        // 아래 기존 경로로 떨어진다.
        if let Some(exact) = self.stored_separation(a, b) {
            return exact;
        }
        let items = &sep.prob.layout().placed_items();
        let (Some(pa), Some(pb)) = (items.get(a), items.get(b)) else {
            return (0.0, 0.0);
        };
        bbox_mtv(pa.shape().bbox(), pb.shape().bbox())
    }

    /// build 시점에 저장해 둔 정확 분리 벡터. 다음 중 하나라도 아니면 `None`이다.
    ///
    /// * [`Self::build_with_nfp`]에 캐시를 주지 않았다 (= 플래그 꺼짐)
    /// * `a`/`b`가 그래프에 없거나 서로 충돌 edge가 아니다
    /// * 그 쌍의 NFP를 못 얻었다 (라이브러리 미등록 형상·회전, 계산 실패 등)
    fn stored_separation(&self, a: PItemKey, b: PItemKey) -> Option<(f32, f32)> {
        if self.exact_sep.is_empty() {
            return None;
        }
        let &ia = self.index_of.get(a)?;
        let &ib = self.index_of.get(b)?;
        // `adjacency[ia]`는 build에서 오름차순 정렬되므로 이분 탐색이 유효하다.
        let k = self.adjacency.get(ia)?.binary_search(&ib).ok()?;
        *self.exact_sep.get(ia)?.get(k)?
    }

    /// ```text
    /// EscapeVector(i) = Σ w_ij × SeparationVector(i,j)
    /// ```
    ///
    /// `w_ij`는 GLS가 학습한 쌍 가중치([`CollisionTracker::get_pair_weight`])다.
    /// 반복해서 풀리지 않은 쌍일수록 큰 가중치를 가지므로, 탈출 방향이 그쪽으로 기운다.
    pub fn escape_vector(&self, sep: &Separator, pk: PItemKey) -> (f32, f32) {
        let Some(&i) = self.index_of.get(pk) else {
            return (0.0, 0.0);
        };
        let mut acc = (0.0f32, 0.0f32);
        for &j in &self.adjacency[i] {
            let other = self.keys[j];
            let w = if sep.ct.pk_idx_map.contains_key(pk) && sep.ct.pk_idx_map.contains_key(other) {
                sep.ct.get_pair_weight(pk, other)
            } else {
                GUIDED_WEIGHT_NEUTRAL
            };
            let w = if w.is_finite() && w > 0.0 {
                w
            } else {
                GUIDED_WEIGHT_NEUTRAL
            };
            let (dx, dy) = self.separation_vector(sep, pk, other);
            acc.0 += w * dx;
            acc.1 += w * dy;
        }
        acc
    }

    /// 현재 `sep` 상태로 다시 계산한 노드별 ConflictEscapeScore 항.
    pub fn scores(&self, sep: &Separator) -> Vec<(PItemKey, ConflictEscapeScoreTerms)> {
        let terms = compute_terms(sep, &self.keys, &self.adjacency);
        self.keys.iter().copied().zip(terms).collect()
    }
}

// ────────────────────────────────────────────────────────────────────────────
// pair matrix 전수 스캔
// ────────────────────────────────────────────────────────────────────────────
//
// `pair_collisions`는 대칭 삼각 행렬이라 156 아이템이면 12,090 엔트리다. 기존에는 노드
// 쌍마다 `PairMatrix`의 인덱스를 다시 계산했지만(`row ≤ col` 분기 + 곱셈/나눗셈),
// 상삼각 한 행은 메모리상 **연속**이므로 행 슬라이스를 통째로 훑는 편이 훨씬 싸다.
// 게다가 대부분의 엔트리는 `loss == 0`이라, SIMD 비교 비트마스크가 0인 블록을 통째로
// 건너뛸 수 있다.

/// 행 → 노드 매핑에서 "이 행에 대응하는 노드가 없음"을 뜻하는 sentinel.
const NO_NODE: u32 = u32::MAX;

/// 컨테이너 경계 침범 여부.
#[inline]
fn container_conflicting(ct: &CollisionTracker, pk: PItemKey) -> bool {
    ct.pk_idx_map
        .get(pk)
        .map(|&row| ct.container_collisions()[row].loss > 0.0)
        .unwrap_or(false)
}

/// ct 행 인덱스 → 노드 인덱스. 대응 노드가 없는 행은 [`NO_NODE`].
///
/// `pk_idx_map`은 단사이므로 이 역매핑은 잘 정의된다.
fn row_nodes(ct: &CollisionTracker, keys: &[PItemKey]) -> Vec<u32> {
    let size = ct.pair_collisions().size;
    let mut nodes = vec![NO_NODE; size];
    for (i, &pk) in keys.iter().enumerate() {
        if let Some(&row) = ct.pk_idx_map.get(pk) {
            if row < size {
                nodes[row] = i as u32;
            }
        }
    }
    nodes
}

/// 상삼각 행렬 `row` 행의 `(row, row+1..size)` 구간에 해당하는 연속 슬라이스.
///
/// `PairMatrix`의 인덱스 규칙은 `row ≤ col`일 때 `row*size + col − row(row+1)/2`이므로
/// 한 행은 연속 구간이다.
#[inline]
fn upper_row_slice(data: &[CTEntry], size: usize, row: usize) -> &[CTEntry] {
    debug_assert!(row < size);
    debug_assert_eq!(data.len(), size * (size + 1) / 2);
    let start = row * size + (row + 1) - (row * (row + 1)) / 2;
    let len = size - row - 1;
    &data[start..start + len]
}

/// 한 행에서 찾은 `loss > 0` 오프셋 목록을 edge로 환산한다. 스칼라/SIMD 공용.
#[inline]
fn emit_row_edges(
    row: usize,
    ia: u32,
    hits: &[u32],
    nodes: &[u32],
    edges: &mut Vec<(u32, u32)>,
    deg: &mut [u32],
) {
    for &off in hits {
        let ib = nodes[row + 1 + off as usize];
        if ib == NO_NODE {
            continue;
        }
        edges.push((ia, ib));
        deg[ia as usize] += 1;
        deg[ib as usize] += 1;
    }
}

/// 행 스캐너를 주입받아 전 행을 훑는다. `row_scan`이 스칼라/SIMD 갈림길이다.
#[inline]
fn scan_rows_with(
    data: &[CTEntry],
    size: usize,
    nodes: &[u32],
    edges: &mut Vec<(u32, u32)>,
    deg: &mut [u32],
    mut row_scan: impl FnMut(&[CTEntry], &mut Vec<u32>),
) {
    let mut hits: Vec<u32> = Vec::with_capacity(size);
    for row in 0..size {
        let ia = nodes[row];
        if ia == NO_NODE {
            continue;
        }
        let seg = upper_row_slice(data, size, row);
        if seg.is_empty() {
            continue;
        }
        hits.clear();
        row_scan(seg, &mut hits);
        emit_row_edges(row, ia, &hits, nodes, edges, deg);
    }
}

/// 스칼라 행 스캐너. `simd` feature가 꺼지면 이 경로만 쓰인다.
#[cfg_attr(feature = "simd", allow(dead_code))]
fn positive_offsets_scalar(seg: &[CTEntry], out: &mut Vec<u32>) {
    for (off, e) in seg.iter().enumerate() {
        if e.loss > 0.0 {
            out.push(off as u32);
        }
    }
}

#[cfg(feature = "simd")]
mod simd_scan {
    use super::{CTEntry, scan_rows_with};
    use std::simd::Simd;
    use std::simd::prelude::SimdPartialOrd;

    // `CTEntry`는 `f32` 두 개뿐이라 패딩이 없다 → `[f32; 2]`로 재해석해도 안전하다.
    const _: () = assert!(std::mem::size_of::<CTEntry>() == 2 * std::mem::size_of::<f32>());
    const _: () = assert!(std::mem::align_of::<CTEntry>() == std::mem::align_of::<f32>());

    /// 재해석했을 때 `loss`가 놓이는 레인(0 또는 1). 필드 선언 순서에 의존하지 않는다.
    const LOSS_LANE: usize = std::mem::offset_of!(CTEntry, loss) / std::mem::size_of::<f32>();

    /// loss 레인만 남기는 비트마스크. `weight`(보통 1.0 > 0)를 걸러내기 위한 것.
    const LOSS_BITS: u64 = 0x5555_5555_5555_5555u64 << LOSS_LANE;

    /// `&[CTEntry]` → `&[f32]` 재해석. 위 const assert가 레이아웃을 보장한다.
    #[inline(always)]
    fn as_f32(seg: &[CTEntry]) -> &[f32] {
        // SAFETY: CTEntry는 f32 2개짜리 패딩 없는 POD이고 정렬도 f32와 같다(위 assert).
        unsafe { std::slice::from_raw_parts(seg.as_ptr().cast::<f32>(), seg.len() * 2) }
    }

    /// 한 행 구간에서 `loss > 0`인 엔트리 오프셋을 모은다.
    ///
    /// `loss`/`weight`가 교차 배치된 AoS를 그대로 로드한 뒤 비트마스크에서 loss 레인만
    /// 남긴다(gather나 deinterleave보다 싸다). 대부분의 블록은 전부 0이라 `bits == 0`
    /// 조기 탈출이 실익의 대부분을 만든다.
    #[inline(always)]
    fn positive_offsets<const W: usize>(seg: &[CTEntry], out: &mut Vec<u32>) {
        let f = as_f32(seg);
        let zero = Simd::<f32, W>::splat(0.0);
        // W는 항상 짝수이므로 청크 경계가 엔트리 경계와 어긋나지 않는다.
        let full = (f.len() / W) * W;
        let mut k = 0;
        while k < full {
            let v = Simd::<f32, W>::from_slice(&f[k..k + W]);
            let mut bits = v.simd_gt(zero).to_bitmask() & LOSS_BITS;
            while bits != 0 {
                let lane = bits.trailing_zeros() as usize;
                // loss의 f32 인덱스 → 엔트리 인덱스. LOSS_LANE이 1이어도 절삭으로 맞는다.
                out.push(((k + lane) / 2) as u32);
                bits &= bits - 1;
            }
            k += W;
        }
        // 잔여 엔트리는 스칼라로.
        for (off, e) in seg.iter().enumerate().skip(full / 2) {
            if e.loss > 0.0 {
                out.push(off as u32);
            }
        }
    }

    /// 폭에 제네릭한 커널 한 벌.
    #[inline(always)]
    fn kernel<const W: usize>(
        data: &[CTEntry],
        size: usize,
        nodes: &[u32],
        edges: &mut Vec<(u32, u32)>,
        deg: &mut [u32],
    ) {
        scan_rows_with(data, size, nodes, edges, deg, |seg, out| {
            positive_offsets::<W>(seg, out)
        });
    }

    fn kernel_4(
        data: &[CTEntry],
        size: usize,
        nodes: &[u32],
        edges: &mut Vec<(u32, u32)>,
        deg: &mut [u32],
    ) {
        kernel::<4>(data, size, nodes, edges, deg)
    }

    #[cfg(target_arch = "x86_64")]
    #[target_feature(enable = "avx2")]
    unsafe fn kernel_8(
        data: &[CTEntry],
        size: usize,
        nodes: &[u32],
        edges: &mut Vec<(u32, u32)>,
        deg: &mut [u32],
    ) {
        kernel::<8>(data, size, nodes, edges, deg)
    }

    #[cfg(target_arch = "x86_64")]
    #[target_feature(enable = "avx512f")]
    unsafe fn kernel_16(
        data: &[CTEntry],
        size: usize,
        nodes: &[u32],
        edges: &mut Vec<(u32, u32)>,
        deg: &mut [u32],
    ) {
        kernel::<16>(data, size, nodes, edges, deg)
    }

    /// 런타임 디스패치: `avx512f` → W=16, `avx2` → W=8, 그 외 → W=4.
    /// 디스패치는 전수 스캔당 한 번만 한다.
    pub(super) fn scan_rows(
        data: &[CTEntry],
        size: usize,
        nodes: &[u32],
        edges: &mut Vec<(u32, u32)>,
        deg: &mut [u32],
    ) {
        #[cfg(target_arch = "x86_64")]
        {
            if std::arch::is_x86_feature_detected!("avx512f") {
                return unsafe { kernel_16(data, size, nodes, edges, deg) };
            }
            if std::arch::is_x86_feature_detected!("avx2") {
                return unsafe { kernel_8(data, size, nodes, edges, deg) };
            }
        }
        kernel_4(data, size, nodes, edges, deg)
    }

    /// 테스트에서 폭을 고정해 호출하기 위한 창구.
    #[cfg(test)]
    pub(super) fn scan_rows_width<const W: usize>(
        data: &[CTEntry],
        size: usize,
        nodes: &[u32],
        edges: &mut Vec<(u32, u32)>,
        deg: &mut [u32],
    ) {
        kernel::<W>(data, size, nodes, edges, deg)
    }
}

/// pair matrix 전수 스캔. `(edges, degree)`를 돌려준다.
///
/// `edges`는 무향 edge를 한 번씩만 담고(양쪽 `deg`는 모두 증가), 순서는 ct 행 순서다.
fn scan_pair_conflicts(ct: &CollisionTracker, keys: &[PItemKey]) -> (Vec<(u32, u32)>, Vec<u32>) {
    let n = keys.len();
    let mut deg = vec![0u32; n];
    let mut edges: Vec<(u32, u32)> = Vec::new();
    if n < 2 {
        return (edges, deg);
    }

    let nodes = row_nodes(ct, keys);
    let size = ct.pair_collisions().size;
    let data = &ct.pair_collisions().data;
    if data.len() != size * (size + 1) / 2 {
        debug_assert!(false, "pair matrix size mismatch");
        return (edges, deg);
    }

    #[cfg(feature = "simd")]
    simd_scan::scan_rows(data, size, &nodes, &mut edges, &mut deg);
    #[cfg(not(feature = "simd"))]
    scan_rows_with(
        data,
        size,
        &nodes,
        &mut edges,
        &mut deg,
        positive_offsets_scalar,
    );

    (edges, deg)
}

/// path compression union-find.
fn uf_find(parent: &mut [usize], x: usize) -> usize {
    let mut root = x;
    while parent[root] != root {
        root = parent[root];
    }
    let mut cur = x;
    while parent[cur] != root {
        let next = parent[cur];
        parent[cur] = root;
        cur = next;
    }
    root
}

/// 겹치는 두 축 정렬 bbox의 MTV. `a`를 밀어내는 방향의 벡터를 돌려준다.
/// x축 겹침량과 y축 겹침량 중 **작은 쪽 축**으로, 중심 상대 위치 부호를 따른다.
fn bbox_mtv(a: Rect, b: Rect) -> (f32, f32) {
    let overlap_x = a.x_max.min(b.x_max) - a.x_min.max(b.x_min);
    let overlap_y = a.y_max.min(b.y_max) - a.y_min.max(b.y_min);
    if overlap_x.is_nan() || overlap_y.is_nan() || overlap_x <= 0.0 || overlap_y <= 0.0 {
        return (0.0, 0.0);
    }
    let (ca_x, ca_y) = ((a.x_min + a.x_max) * 0.5, (a.y_min + a.y_max) * 0.5);
    let (cb_x, cb_y) = ((b.x_min + b.x_max) * 0.5, (b.y_min + b.y_max) * 0.5);
    if overlap_x <= overlap_y {
        let sign = if ca_x < cb_x { -1.0 } else { 1.0 };
        (sign * overlap_x, 0.0)
    } else {
        let sign = if ca_y < cb_y { -1.0 } else { 1.0 };
        (0.0, sign * overlap_y)
    }
}

// ────────────────────────────────────────────────────────────────────────────
// ② 정확한 NFP 기반 분리 벡터
// ────────────────────────────────────────────────────────────────────────────
//
// bbox MTV는 축 정렬 두 방향밖에 못 내놓는다. 오목한 의류 패턴에서는 실제 최소 분리
// 방향이 대각인 경우가 대부분이라 방향이 크게 어긋난다.
//
// 정확한 답은 NFP가 준다. `NFP(b, a)`는 "b가 고정일 때 a의 앵커가 놓이면 안 되는
// 영역"이고, 두 조각이 겹쳐 있다는 것은 곧 현재 a의 앵커가 그 영역 **내부**에 있다는
// 뜻이다. 따라서 a를 b에서 떼어내는 최소 이동 = 그 점에서 NFP 경계까지의 최단 벡터다.
// 경계에 닿은 위치는 "접촉(touching)"이라 겹침이 아니다.
//
// 좌표계: 캐시의 NFP는 **로컬 좌표**(고정 조각이 원점)다. 그래서 질의점은
// `t_a - t_b`(앵커 차)이고, 나온 벡터는 그대로 월드 평행이동 증분이다.

/// 배치된 아이템의 회전각(도)을 라이브러리 키로 쓸 정수로 환산한다.
///
/// 반올림 오차가 `ROTATION_MATCH_EPS_DEG`를 넘으면 라이브러리에 없는 각도라는 뜻이므로
/// `None`을 돌려 폴백시킨다. 살짝 다른 각도의 NFP를 쓰면 **틀린** 분리 벡터가 나온다.
fn rotation_degrees(rotation_rad: f32) -> Option<i64> {
    let deg = (rotation_rad as f64).to_degrees();
    if !deg.is_finite() {
        return None;
    }
    let rounded = deg.round();
    if (deg - rounded).abs() > ROTATION_MATCH_EPS_DEG {
        return None;
    }
    Some((rounded as i64).rem_euclid(360))
}

/// [`rotation_degrees`]가 허용하는 반올림 오차(도).
///
/// 배치 회전각은 `i64` 도 → `f32` 라디안 → `f64` 도를 왕복하므로 `1e-4°` 수준의
/// 오차만 생긴다. 그보다 크면 라이브러리에 없는 연속 회전으로 본다.
const ROTATION_MATCH_EPS_DEG: f64 = 1.0e-3;

/// 무향 edge마다 정확한 분리 벡터를 한 번씩만 구해 `adjacency` 모양으로 돌려준다.
///
/// `(i, j)`와 `(j, i)`는 정확히 부호만 반대다(`NFP(A,B) = -NFP(B,A)`이고 질의점도
/// 부호가 뒤집힌다). 그래서 `i < j`인 방향만 실제로 질의하고 반대편은 부호 반전으로
/// 채운다 — NFP 질의 횟수가 절반이 된다.
fn compute_exact_separations(
    sep: &Separator,
    keys: &[PItemKey],
    adjacency: &[Vec<usize>],
    cache: &mut NfpLbfCache,
) -> Vec<Vec<Option<(f32, f32)>>> {
    let mut out: Vec<Vec<Option<(f32, f32)>>> =
        adjacency.iter().map(|adj| vec![None; adj.len()]).collect();

    // 1) i < j 방향만 질의한다.
    for (i, adj) in adjacency.iter().enumerate() {
        for (k, &j) in adj.iter().enumerate() {
            if i >= j {
                continue;
            }
            out[i][k] = nfp_separation_vector(sep, cache, keys[i], keys[j]);
        }
    }

    // 2) 반대 방향은 부호만 뒤집어 채운다.
    for (i, adj) in adjacency.iter().enumerate() {
        for (k, &j) in adj.iter().enumerate() {
            if i <= j {
                continue;
            }
            // `adjacency[j]`도 오름차순이므로 i의 자리는 이분 탐색으로 찾는다.
            let Ok(pos) = adjacency[j].binary_search(&i) else {
                continue;
            };
            out[i][k] = out[j][pos].map(|(dx, dy)| (-dx, -dy));
        }
    }

    out
}

/// `a`를 `b`에서 떼어내는 정확한 최소 분리 벡터.
///
/// `None`은 "NFP를 못 얻었다"는 뜻이고 호출자는 **조용히 bbox MTV로 폴백**한다.
/// 못 얻는 경우는 셋이다 — 라이브러리에 없는 형상(단순화 실패 등), 라이브러리에 없는
/// 회전각, NFP 계산 실패.
///
/// 얻은 경우 `Some((0,0))`은 "NFP 기준으로는 이미 떨어져 있다"는 정상적인 답이다.
/// NFP는 `shape_cd`에 오목 단순화와 clearance를 얹어 만들어져 실제 형상의 **상위집합**
/// 이므로, CDE가 겹쳤다고 본 쌍이 NFP에서 떨어져 나오는 일은 사실상 없다.
fn nfp_separation_vector(
    sep: &Separator,
    cache: &mut NfpLbfCache,
    a: PItemKey,
    b: PItemKey,
) -> Option<(f32, f32)> {
    let items = &sep.prob.layout().placed_items();
    let (pa, pb) = (items.get(a)?, items.get(b)?);

    let rot_a = rotation_degrees(*pa.d_transf().rotation)?;
    let rot_b = rotation_degrees(*pb.d_transf().rotation)?;
    let hash_a = cache.library().hash_of_item(pa.item().idx())?;
    let hash_b = cache.library().hash_of_item(pb.item().idx())?;

    // 고정 = b, 이동 = a.
    let key = NfpKey::new(hash_b, rot_b, hash_a, rot_a);
    // 정규 키로 직접 질의한다. 부호 반전본을 캐시에 새로 만들지 않기 위해서다
    // (`get_or_compute`는 비정규 키를 받으면 호출마다 반전본을 새로 할당한다).
    // 반전이 필요하면 질의점과 결과를 우리가 뒤집는다 — 기하학적으로 동일하다.
    let (canonical, flipped) = key.canonical();
    let region = cache.get_or_compute(canonical)?;

    let dx = (*pa.d_transf().translation.0 - *pb.d_transf().translation.0) as f64;
    let dy = (*pa.d_transf().translation.1 - *pb.d_transf().translation.1) as f64;
    if !dx.is_finite() || !dy.is_finite() {
        return None;
    }

    let (qx, qy) = if flipped { (-dx, -dy) } else { (dx, dy) };
    let (vx, vy) = exit_vector(&region, qx, qy);
    let (vx, vy) = if flipped { (-vx, -vy) } else { (vx, vy) };
    if !vx.is_finite() || !vy.is_finite() {
        return None;
    }
    Some((vx as f32, vy as f32))
}

/// 점 `(px, py)`가 `region` 내부면 경계까지의 최단 벡터를, 밖(또는 hole 안)이면
/// `(0.0, 0.0)`을 돌려준다.
///
/// hole은 NFP의 **구멍**, 즉 그 안쪽은 배치 가능한 포켓이다. 그래서 hole 안의 점은
/// 겹침이 아니고, hole 경계는 탈출 목표가 될 수 있다(가장 가까운 경계가 hole 쪽이면
/// 포켓으로 밀어 넣는 것이 최소 이동이다).
fn exit_vector(region: &PolygonSet, px: f64, py: f64) -> (f64, f64) {
    if !point_in_polygon_set(region, px, py) {
        return (0.0, 0.0);
    }
    let mut best: Option<(f64, (f64, f64))> = None;
    for component in &region.components {
        for ring in std::iter::once(&component.outer).chain(component.holes.iter()) {
            if let Some((d2, v)) = nearest_on_ring(ring, px, py) {
                if best.is_none_or(|(bd2, _)| d2 < bd2) {
                    best = Some((d2, v));
                }
            }
        }
    }
    best.map(|(_, v)| v).unwrap_or((0.0, 0.0))
}

/// ring의 모든 변에 대한 점-선분 최단거리 중 최소. `(거리², 점→최근접점 벡터)`.
fn nearest_on_ring(ring: &Ring, px: f64, py: f64) -> Option<(f64, (f64, f64))> {
    let v = &ring.vertices;
    if v.len() < 2 {
        return None;
    }
    let mut best: Option<(f64, (f64, f64))> = None;
    let mut prev = v.len() - 1;
    for cur in 0..v.len() {
        let (ax, ay) = (v[prev].x, v[prev].y);
        let (bx, by) = (v[cur].x, v[cur].y);
        prev = cur;
        let (ex, ey) = (bx - ax, by - ay);
        let len2 = ex * ex + ey * ey;
        // 퇴화 변(중복 정점)은 시작점 하나로 취급한다.
        let t = if len2 > 0.0 {
            (((px - ax) * ex + (py - ay) * ey) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let (qx, qy) = (ax + t * ex, ay + t * ey);
        let (dx, dy) = (qx - px, qy - py);
        let d2 = dx * dx + dy * dy;
        if !d2.is_finite() {
            continue;
        }
        if best.is_none_or(|(bd2, _)| d2 < bd2) {
            best = Some((d2, (dx, dy)));
        }
    }
    best
}

/// 점이 `set`의 내부(= 어떤 component의 outer 안이면서 그 hole 밖)인가.
fn point_in_polygon_set(set: &PolygonSet, px: f64, py: f64) -> bool {
    set.components.iter().any(|component| {
        point_in_ring(&component.outer, px, py)
            && !component
                .holes
                .iter()
                .any(|hole| point_in_ring(hole, px, py))
    })
}

/// crossing number(ray casting) 방식 점-다각형 내부 판정. winding 방향에 무관하다.
///
/// 경계 위의 점은 판정이 모호하지만, 그 경우 [`exit_vector`]가 내놓는 값이 어차피
/// 길이 0이라 결과에 영향이 없다.
fn point_in_ring(ring: &Ring, px: f64, py: f64) -> bool {
    let v = &ring.vertices;
    if v.len() < 3 {
        return false;
    }
    let mut inside = false;
    let mut prev = v.len() - 1;
    for cur in 0..v.len() {
        let (xi, yi) = (v[cur].x, v[cur].y);
        let (xj, yj) = (v[prev].x, v[prev].y);
        prev = cur;
        if (yi > py) != (yj > py) {
            let dy = yj - yi;
            if dy == 0.0 {
                continue;
            }
            let x_cross = xi + (py - yi) / dy * (xj - xi);
            if px < x_cross {
                inside = !inside;
            }
        }
    }
    inside
}

/// bbox 대각 길이.
#[inline]
fn bbox_diagonal(bbox: Rect) -> f32 {
    let (w, h) = (bbox.width(), bbox.height());
    (w * w + h * h).sqrt()
}

/// 노드별 ConflictEscapeScore 항을 계산한다. 모든 항은 배치 전체를 기준으로 정규화된다.
fn compute_terms(
    sep: &Separator,
    keys: &[PItemKey],
    adjacency: &[Vec<usize>],
) -> Vec<ConflictEscapeScoreTerms> {
    let layout = &sep.prob.layout();
    let strip = layout.container().outer_cd().bbox();
    let strip_w = strip.width().max(GEOM_EPS);

    let max_degree = adjacency.iter().map(|a| a.len()).max().unwrap_or(0).max(1) as f32;

    // 정규화 상수(최대 면적·최대 대각)와 본 계산이 각각 slotmap을 훑던 것을 한 번으로
    // 합친다. `keys`는 build 시점 스냅샷이라 이미 사라진 키가 섞일 수 있으므로 Option이다.
    let mut max_area = GEOM_EPS;
    let mut max_diag = GEOM_EPS;
    let mut placed: Vec<Option<(f32, Rect)>> = Vec::with_capacity(keys.len());
    for &pk in keys {
        match layout.placed_items().get(pk) {
            Some(pi) => {
                let (area, bbox) = (pi.shape().area(), pi.shape().bbox());
                max_area = max_area.max(area);
                max_diag = max_diag.max(bbox_diagonal(bbox));
                placed.push(Some((area, bbox)));
            }
            None => placed.push(None),
        }
    }

    placed
        .iter()
        .enumerate()
        .map(|(i, slot)| {
            let Some((area, bbox)) = *slot else {
                return ConflictEscapeScoreTerms::default();
            };
            let (w, h) = (bbox.width().max(0.0), bbox.height().max(0.0));
            let longest = w.max(h).max(GEOM_EPS);
            let shortest = w.min(h).max(0.0);
            // 길쭉할수록 1에 가깝다 (정사각형 0).
            let elongation = (1.0 - shortest / longest).clamp(0.0, 1.0);
            let relative_size = (bbox_diagonal(bbox) / max_diag).clamp(0.0, 1.0);

            ConflictEscapeScoreTerms {
                collision_degree: (adjacency[i].len() as f32 / max_degree).clamp(0.0, 1.0),
                strip_boundary_contribution: ((bbox.x_max - strip.x_min) / strip_w).clamp(0.0, 1.0),
                blocking_score: (0.5 * elongation + 0.5 * relative_size).clamp(0.0, 1.0),
                area: (area / max_area).clamp(0.0, 1.0),
                concavity: (1.0 - area / bbox.area().max(GEOM_EPS)).clamp(0.0, 1.0),
            }
        })
        .collect()
}

// ────────────────────────────────────────────────────────────────────────────
// Adaptive Strip Relaxation 사다리
// ────────────────────────────────────────────────────────────────────────────

/// 탈출 직후 원래 Strip 길이로 **점진 복귀**하는 사다리.
///
/// ```text
/// L + 6% → L + 4% → L + 2% → L → L - ε 시도
/// ```
///
/// [`STRIP_RELAXATION_LADDER`]의 네 비율을 차례로 내보낸 뒤 마지막으로 한 번
/// `base_width × (1 − STRIP_RELAXATION_EPSILON)`을 내보내고 종료한다.
#[derive(Clone, Copy, Debug)]
pub struct StripRelaxationLadder {
    base_width: f32,
    step: usize,
}

impl StripRelaxationLadder {
    pub fn new(base_width: f32) -> Self {
        Self {
            base_width,
            step: 0,
        }
    }

    /// 다음 목표 폭. 사다리를 모두 소진하면 `None`.
    pub fn next(&mut self) -> Option<f32> {
        let n = STRIP_RELAXATION_LADDER.len();
        let width = if self.step < n {
            self.base_width * (1.0 + STRIP_RELAXATION_LADDER[self.step])
        } else if self.step == n {
            self.base_width * (1.0 - STRIP_RELAXATION_EPSILON)
        } else {
            return None;
        };
        self.step += 1;
        Some(width)
    }

    /// 처음으로 되돌린다.
    pub fn reset(&mut self) {
        self.step = 0;
    }

    /// 복귀 기준이 되는 원래 폭 `L`.
    pub fn base_width(&self) -> f32 {
        self.base_width
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Conflict Burst 실행
// ────────────────────────────────────────────────────────────────────────────

/// Conflict Burst의 튜닝 축.
#[derive(Clone, Copy, Debug)]
pub struct ConflictBurstConfig {
    /// `상위 5~15% 정도를 제거한 다음 다시 삽입한다` — 하한.
    pub remove_ratio_min: f32,
    /// 위 범위의 상한.
    pub remove_ratio_max: f32,
    /// `평소 Local Move  1~3 mm` (참조값; burst는 escape step을 쓴다).
    pub local_move_mm: (f32, f32),

    /// `Escape Mode  10~50 mm`
    pub escape_move_mm: (f32, f32),
    /// `Escape Mode  ... 또는 bbox 크기의 20~50%`
    pub escape_move_bbox_ratio: (f32, f32),
    /// Stage 1에서 강제 분리할 아이템 수 상한.
    pub max_push_items: usize,
}

impl Default for ConflictBurstConfig {
    fn default() -> Self {
        Self {
            remove_ratio_min: CONFLICT_REMOVE_RATIO_MIN,
            remove_ratio_max: CONFLICT_REMOVE_RATIO_MAX,
            local_move_mm: LOCAL_MOVE_MM,
            escape_move_mm: ESCAPE_MOVE_MM,
            escape_move_bbox_ratio: ESCAPE_MOVE_BBOX_RATIO,
            max_push_items: DEFAULT_MAX_PUSH_ITEMS,
        }
    }
}

/// Conflict Burst 1회의 결과.
#[derive(Clone, Debug)]
pub struct ConflictBurstOutcome {
    /// 최종 충돌 loss가 0인가.
    pub resolved: bool,
}

/// **Conflict Burst + Partial Rebuild** 한 사이클.
///
/// ```text
/// Stage 1  Collision Push       → 충돌 벡터 기반 강제 분리
/// Stage 2  Strip Relaxation     → L + ΔL 로 일시 확장
/// Stage 3  Destroy & Repair     → 충돌 중심 패턴 5~15% 제거 후 재삽입 + 재압축
/// 마무리    L + 6% → L + 4% → L + 2% → L → L − ε 로 점진 복귀
/// ```
///
/// 각 Stage 후 loss가 0이면 조기 종료한다. `term`이 중단을 요청하거나, 해소되지 않은 채
/// 시작보다 loss가 나빠지면 폭까지 포함해 호출 전 상태로 전부 되돌린다.
///
/// 개별 이동 가능한 조각이 없으면 Stage 3를 건너뛰고
/// Stage 1+2만 수행한다.
pub fn conflict_burst_partial_rebuild(
    sep: &mut Separator,
    cfg: &ConflictBurstConfig,
    rnr_cfg: &RuinRecreateConfig,
    tabu: &mut BandTabu,
    term: &impl Terminator,
) -> ConflictBurstOutcome {
    conflict_burst_partial_rebuild_with_nfp(sep, cfg, rnr_cfg, tabu, term, None)
}

/// [`conflict_burst_partial_rebuild`]와 같되, `nfp`가 `Some`이면 Stage 1의 분리 벡터를
/// **정확한 NFP 기반**으로 계산한다 (`② exact_separation_vector`).
///
/// 호출자는 [`crate::config::NfpIntegrationConfig::exact_separation_vector`]가 켜졌을
/// 때만 `Some`을 넘긴다. `None`이면 [`conflict_burst_partial_rebuild`]와 **완전히
/// 동일**하다 — Stage 2/3와 복귀 사다리는 이 플래그와 무관하다.
pub fn conflict_burst_partial_rebuild_with_nfp(
    sep: &mut Separator,
    cfg: &ConflictBurstConfig,
    rnr_cfg: &RuinRecreateConfig,
    tabu: &mut BandTabu,
    term: &impl Terminator,
    nfp: Option<&mut NfpLbfCache>,
) -> ConflictBurstOutcome {
    let base_width = sep.prob.strip_width();
    let before_sol = sep.prob.save();
    let before_ct = sep.ct.save();
    let loss_before = sep.ct.get_total_loss();

    let graph = ConflictGraph::build_with_nfp(sep, nfp);
    let ratio_before = graph.collision_ratio();
    let t = burst_intensity(ratio_before);

    let mut outcome = ConflictBurstOutcome {
        resolved: loss_before == 0.0,
    };

    if term.kill() {
        restore_full(sep, &before_sol, &before_ct, base_width);
        return outcome;
    }
    if loss_before == 0.0 {
        return outcome;
    }

    // ── Stage 1 — Collision Push ────────────────────────────────────────────
    stage_collision_push(sep, cfg, &graph, t, term);

    if term.kill() {
        restore_full(sep, &before_sol, &before_ct, base_width);
        outcome.resolved = sep.ct.get_total_loss() == 0.0;
        return outcome;
    }

    if sep.ct.get_total_loss() == 0.0 {
        // 폭을 건드리지 않았으므로 복귀 사다리가 필요 없다.
        outcome.resolved = true;
        return outcome;
    }

    // ── Stage 2 — Strip Relaxation ──────────────────────────────────────────
    let delta_l = CollisionRatioBand::classify(ratio_before).delta_l(t);
    let relaxed_width = base_width * (1.0 + delta_l);
    if relaxed_width > base_width + GEOM_EPS && relaxed_width.is_finite() {
        sep.change_strip_width(relaxed_width, None);
        // 폭이 바뀌면 밴드 좌표계가 이동하므로 tabu 이력을 버린다.
        tabu.clear();
    }

    if term.kill() {
        restore_full(sep, &before_sol, &before_ct, base_width);
        outcome.resolved = sep.ct.get_total_loss() == 0.0;
        return outcome;
    }

    // ── Stage 3 — Destroy & Repair ──────────────────────────────────────────
    if sep.ct.get_total_loss() > 0.0
        && sep
            .prob
            .layout()
            .placed_items()
            .values()
            .any(|pi| sep.can_move_individually(pi.item().idx()))
    {
        let remove_ratio = lerp_ratio((cfg.remove_ratio_min, cfg.remove_ratio_max), t);
        let n_placed = sep.prob.layout().placed_items().len();
        let max_items = ((n_placed as f32 * remove_ratio).round() as usize).max(2);
        let burst_rnr = RuinRecreateConfig {
            period: rnr_cfg.period,
            band_fraction: rnr_cfg.band_fraction,
            max_items,
            // burst 는 자기 remove_ratio 로 이미 규모를 정한다. 새 재배열
            // 노브는 상속하지 않아 기존 동작이 비트 단위로 유지된다.
            ..crate::config::DEFAULT_SWARM_CONFIG.expl_cfg.rnr
        };
        // 실제 제거·재삽입은 검증된 repair 경로에 위임한다.
        ruin_and_recreate(sep, &burst_rnr, RuinStrategy::Fragmented, tabu, term);

        // Left/Bottom Compaction — 이동 없이 남는 폭만 회수한다.
        sep.refit_strip();
    }

    if term.kill() {
        restore_full(sep, &before_sol, &before_ct, base_width);
        outcome.resolved = sep.ct.get_total_loss() == 0.0;
        return outcome;
    }

    // ── 마무리 — Adaptive Strip Relaxation 점진 복귀 ─────────────────────────
    descend_relaxation_ladder(sep, base_width, term);

    let loss_after = sep.ct.get_total_loss();
    outcome.resolved = loss_after == 0.0;
    let final_width = sep.prob.strip_width();

    // 해소되지 않았다면 두 경우에 통째로 되돌린다.
    //  * 시작보다 loss가 나빠졌다.
    //  * ΔL을 갚지 못하고 폭이 `L`보다 넓은 채로 끝났다 (strip 길이 후퇴 방지).
    if !outcome.resolved && (loss_after > loss_before || final_width > base_width + GEOM_EPS) {
        restore_full(sep, &before_sol, &before_ct, base_width);
    }
    outcome
}

/// Stage 1 — 최대 성분의 상위 점수 아이템을 escape vector 방향으로 크게 밀어낸다.
///
/// ```text
/// 평소 Local Move    1~3 mm
/// Escape Mode        10~50 mm 또는 bbox 크기의 20~50%
/// ```
fn stage_collision_push(
    sep: &mut Separator,
    cfg: &ConflictBurstConfig,
    graph: &ConflictGraph,
    t: f32,
    term: &impl Terminator,
) {
    let Some(component) = graph.largest_component() else {
        return;
    };

    // 이동은 PItemKey를 교체하므로(그룹 이동은 여러 개를 한 번에), 방향과 step은
    // **이동 전 스냅샷**에서 모두 먼저 계산한다.
    let mut plans: Vec<(PItemKey, f32, f32)> = Vec::new();
    for &pk in component.members.iter().take(cfg.max_push_items.max(1)) {
        // bbox/item_id를 값으로 복사해 두어 `sep` 대여를 즉시 끝낸다.
        let Some((item_id, bbox)) = sep
            .prob
            .layout()
            .placed_items()
            .get(pk)
            .map(|pi| (pi.item().idx(), pi.shape().bbox()))
        else {
            continue;
        };
        if sep.frozen_mask.get(item_id).copied().unwrap_or(false) {
            continue;
        }
        let (vx, vy) = graph.escape_vector(sep, pk);
        let norm = (vx * vx + vy * vy).sqrt();
        if !norm.is_finite() || norm <= GEOM_EPS {
            continue;
        }
        // 문서가 정한 두 스케일의 대비를 그대로 지킨다.
        //
        // ```text
        // 평소 Local Move    1~3 mm
        // Escape Mode        10~50 mm 또는 bbox 크기의 20~50%
        // ```
        //
        // 이 단계는 "작은 step 으로 계속 튕기는" 국소 이동이 아니라 **의도적으로 큰 step**
        // 이어야 한다. 그래서 평소 국소 이동의 상한(`local_move_mm.1`)을 하한으로 깔아,
        // 밀도나 bbox 가 아무리 작아도 국소 이동 수준으로 퇴화하지 않게 한다.
        let local_ceiling = cfg.local_move_mm.1.max(cfg.local_move_mm.0);
        let step = lerp_ratio(cfg.escape_move_mm, t)
            .max(bbox_diagonal(bbox) * lerp_ratio(cfg.escape_move_bbox_ratio, t))
            .max(local_ceiling);
        if !step.is_finite() || step <= GEOM_EPS {
            continue;
        }
        plans.push((pk, vx / norm * step, vy / norm * step));
    }

    for (pk, dx, dy) in plans {
        if term.kill() {
            break;
        }
        // 앞선 (그룹) 이동으로 키가 사라졌을 수 있다.
        let Some(current) = sep.prob.layout().placed_items().get(pk).map(|pi| pi.d_transf()) else {
            continue;
        };
        let target = current.compose().translate((dx, dy)).decompose();
        let _ = sep.move_item(pk, target);
    }
}

/// `L + 6% → L + 4% → L + 2% → L → L − ε` 로 점진 복귀한다.
///
/// 각 단계에서 폭을 좁힌 뒤 loss가 0이면 다음(더 좁은) 단계로 내려가고, 0이 아니면
/// **마지막 성공 폭**으로 정확히 되돌린 뒤 멈춘다.
fn descend_relaxation_ladder(sep: &mut Separator, base_width: f32, term: &impl Terminator) {
    let mut ladder = StripRelaxationLadder::new(base_width);
    while let Some(target) = ladder.next() {
        if term.kill() {
            return;
        }
        let current = sep.prob.strip_width();
        // 사다리는 좁히기만 한다. 현재보다 넓은 단계는 건너뛴다.
        if !target.is_finite() || target <= 0.0 || target >= current - GEOM_EPS {
            continue;
        }
        // 이 단계가 실패하면 정확히 여기로 돌아온다.
        let good_sol = sep.prob.save();
        let good_ct = sep.ct.save();
        let good_width = current;

        sep.change_strip_width(target, None);
        if sep.ct.get_total_loss() == 0.0 {
            continue;
        }
        restore_full(sep, &good_sol, &good_ct, good_width);
        return;
    }
}

/// 폭까지 포함해 전체 상태를 스냅샷으로 되돌린다.
///
/// [`Separator::change_strip_width`]를 한 번 거쳐 worker들의 컨테이너 폭까지 동기화한 뒤
/// (worker의 `load`는 `sol.strip_width() == prob.strip_width()`를 단언한다) 정확한
/// 배치·트래커를 복원한다.
fn restore_full(sep: &mut Separator, sol: &SPSolution, ct: &CollisionTracker, width: f32) {
    if width.is_finite() && width > 0.0 && (sep.prob.strip_width() - width).abs() > GEOM_EPS {
        sep.change_strip_width(width, None);
    }
    sep.prob.restore(sol);
    sep.ct.copy_from(ct);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(x_min: f32, y_min: f32, x_max: f32, y_max: f32) -> Rect {
        Rect {
            x_min,
            y_min,
            x_max,
            y_max,
        }
    }

    #[test]
    fn collision_ratio_band_uses_documented_thresholds() {
        // collision ratio < 10% → Low
        assert_eq!(CollisionRatioBand::classify(0.0), CollisionRatioBand::Low);
        assert_eq!(CollisionRatioBand::classify(0.099), CollisionRatioBand::Low);
        // 10~30% → Mid (경계 포함)
        assert_eq!(
            CollisionRatioBand::classify(COLLISION_RATIO_LOW),
            CollisionRatioBand::Mid
        );
        assert_eq!(
            CollisionRatioBand::classify(COLLISION_RATIO_HIGH),
            CollisionRatioBand::Mid
        );
        // > 30% → High
        assert_eq!(
            CollisionRatioBand::classify(COLLISION_RATIO_HIGH + 1.0e-3),
            CollisionRatioBand::High
        );
        assert_eq!(CollisionRatioBand::classify(1.0), CollisionRatioBand::High);
        // NaN은 가장 보수적인 구간으로
        assert_eq!(
            CollisionRatioBand::classify(f32::NAN),
            CollisionRatioBand::Low
        );
    }

    #[test]
    fn delta_l_matches_documented_ranges() {
        assert_eq!(CollisionRatioBand::Low.delta_l_range(), DELTA_L_LOW);
        assert_eq!(CollisionRatioBand::Mid.delta_l_range(), DELTA_L_MID);
        assert_eq!(CollisionRatioBand::High.delta_l_range(), DELTA_L_HIGH);

        // ΔL = 1~2% / 3~5% / 5~10%
        assert!((CollisionRatioBand::Low.delta_l(0.0) - 0.01).abs() < 1.0e-6);
        assert!((CollisionRatioBand::Low.delta_l(1.0) - 0.02).abs() < 1.0e-6);
        assert!((CollisionRatioBand::Mid.delta_l(0.0) - 0.03).abs() < 1.0e-6);
        assert!((CollisionRatioBand::Mid.delta_l(1.0) - 0.05).abs() < 1.0e-6);
        assert!((CollisionRatioBand::High.delta_l(0.0) - 0.05).abs() < 1.0e-6);
        assert!((CollisionRatioBand::High.delta_l(1.0) - 0.10).abs() < 1.0e-6);
        // 중간값은 선형 보간
        assert!((CollisionRatioBand::High.delta_l(0.5) - 0.075).abs() < 1.0e-6);
        // t는 클램프된다
        assert_eq!(
            CollisionRatioBand::Mid.delta_l(-5.0),
            CollisionRatioBand::Mid.delta_l(0.0)
        );
        assert_eq!(
            CollisionRatioBand::Mid.delta_l(5.0),
            CollisionRatioBand::Mid.delta_l(1.0)
        );
    }

    #[test]
    fn burst_intensity_saturates_at_the_high_band() {
        assert_eq!(burst_intensity(0.0), 0.0);
        assert!((burst_intensity(COLLISION_RATIO_HIGH) - 1.0).abs() < 1.0e-6);
        assert_eq!(burst_intensity(0.9), 1.0);
        assert_eq!(burst_intensity(f32::NAN), 0.0);
        assert!(burst_intensity(0.15) > burst_intensity(0.05));
    }

    #[test]
    fn conflict_escape_score_uses_documented_weights() {
        let all_ones = ConflictEscapeScoreTerms {
            collision_degree: 1.0,
            strip_boundary_contribution: 1.0,
            blocking_score: 1.0,
            area: 1.0,
            concavity: 1.0,
        };
        // 0.35 + 0.25 + 0.20 + 0.10 + 0.10 = 1.00
        assert!((all_ones.score() - 1.0).abs() < 1.0e-6);
        assert_eq!(ConflictEscapeScoreTerms::default().score(), 0.0);

        let only_degree = ConflictEscapeScoreTerms {
            collision_degree: 1.0,
            ..ConflictEscapeScoreTerms::default()
        };
        assert!((only_degree.score() - W_COLLISION_DEGREE).abs() < 1.0e-6);

        let mixed = ConflictEscapeScoreTerms {
            collision_degree: 0.5,
            strip_boundary_contribution: 1.0,
            blocking_score: 0.25,
            area: 0.0,
            concavity: 0.8,
        };
        let expected = 0.35 * 0.5 + 0.25 * 1.0 + 0.20 * 0.25 + 0.10 * 0.0 + 0.10 * 0.8;
        assert!((mixed.score() - expected).abs() < 1.0e-6);

        // CollisionDegree가 가장 무거운 항이어야 한다.
        assert!(W_COLLISION_DEGREE > W_STRIP_BOUNDARY_CONTRIBUTION);
        assert!(W_STRIP_BOUNDARY_CONTRIBUTION > W_BLOCKING_SCORE);
        assert!(W_BLOCKING_SCORE > W_CONFLICT_AREA);
        assert!((W_CONFLICT_AREA - W_CONCAVITY).abs() < 1.0e-6);
    }

    #[test]
    fn bbox_mtv_pushes_along_the_shallower_axis() {
        // x축으로 2, y축으로 8 겹친다 → x축으로 밀어낸다.
        let a = rect(0.0, 0.0, 10.0, 10.0);
        let b = rect(8.0, 2.0, 18.0, 12.0);
        let (dx, dy) = bbox_mtv(a, b);
        assert_eq!(dy, 0.0);
        // a의 중심이 b의 왼쪽 → 왼쪽(-x)으로 밀린다.
        assert!((dx - (-2.0)).abs() < 1.0e-6, "dx = {dx}");

        // 반대 방향은 부호가 뒤집힌다.
        let (dx_rev, dy_rev) = bbox_mtv(b, a);
        assert_eq!(dy_rev, 0.0);
        assert!((dx_rev - 2.0).abs() < 1.0e-6, "dx_rev = {dx_rev}");

        // y축이 더 얕으면 y축으로 밀어낸다.
        let c = rect(0.0, 0.0, 10.0, 10.0);
        let d = rect(2.0, 9.0, 12.0, 19.0);
        let (dx2, dy2) = bbox_mtv(c, d);
        assert_eq!(dx2, 0.0);
        assert!((dy2 - (-1.0)).abs() < 1.0e-6, "dy2 = {dy2}");

        // 겹치지 않으면 (0, 0)
        assert_eq!(
            bbox_mtv(rect(0.0, 0.0, 1.0, 1.0), rect(2.0, 2.0, 3.0, 3.0)),
            (0.0, 0.0)
        );
        // 접촉만 하는 경우도 분리 벡터가 없다
        assert_eq!(
            bbox_mtv(rect(0.0, 0.0, 1.0, 1.0), rect(1.0, 0.0, 2.0, 1.0)),
            (0.0, 0.0)
        );
    }

    #[test]
    fn strip_relaxation_ladder_walks_back_to_base_then_below() {
        let base = 100.0f32;
        let mut ladder = StripRelaxationLadder::new(base);
        assert_eq!(ladder.base_width(), base);

        // L + 6% → L + 4% → L + 2% → L → L - ε
        let seq: Vec<f32> = std::iter::from_fn(|| ladder.next()).collect();
        assert_eq!(seq.len(), STRIP_RELAXATION_LADDER.len() + 1);
        assert!((seq[0] - 106.0).abs() < 1.0e-3, "{seq:?}");
        assert!((seq[1] - 104.0).abs() < 1.0e-3, "{seq:?}");
        assert!((seq[2] - 102.0).abs() < 1.0e-3, "{seq:?}");
        assert!((seq[3] - 100.0).abs() < 1.0e-3, "{seq:?}");
        assert!(
            (seq[4] - base * (1.0 - STRIP_RELAXATION_EPSILON)).abs() < 1.0e-3,
            "{seq:?}"
        );
        // 단조 감소
        assert!(seq.windows(2).all(|w| w[1] < w[0]), "{seq:?}");
        // 소진되면 None
        assert_eq!(ladder.next(), None);

        ladder.reset();
        assert!((ladder.next().unwrap() - 106.0).abs() < 1.0e-3);
    }

    #[test]
    fn default_burst_config_follows_the_document() {
        let cfg = ConflictBurstConfig::default();
        // 상위 5~15% 제거
        assert_eq!(cfg.remove_ratio_min, CONFLICT_REMOVE_RATIO_MIN);
        assert_eq!(cfg.remove_ratio_max, CONFLICT_REMOVE_RATIO_MAX);
        assert!((cfg.remove_ratio_min - 0.05).abs() < 1.0e-6);
        assert!((cfg.remove_ratio_max - 0.15).abs() < 1.0e-6);
        // Escape Mode 10~50 mm 또는 bbox의 20~50%
        assert_eq!(cfg.escape_move_mm, (10.0, 50.0));
        assert_eq!(cfg.escape_move_bbox_ratio, (0.20, 0.50));
        // 평소 Local Move는 1~3 mm — escape step이 언제나 더 크다
        assert_eq!(cfg.local_move_mm, (1.0, 3.0));
        assert!(lerp_ratio(cfg.escape_move_mm, 0.0) > lerp_ratio(cfg.local_move_mm, 1.0));
        assert!(cfg.max_push_items > 0);
    }

    #[test]
    fn escape_step_never_degrades_to_a_local_move() {
        // 문서: Escape Mode 는 평소 Local Move(1~3mm)보다 반드시 커야 한다.
        let cfg = ConflictBurstConfig::default();
        let local_ceiling = cfg.local_move_mm.1.max(cfg.local_move_mm.0);
        for t in [0.0f32, 0.5, 1.0] {
            for diag in [0.0f32, 0.1, 1.0, 500.0] {
                let step = lerp_ratio(cfg.escape_move_mm, t)
                    .max(diag * lerp_ratio(cfg.escape_move_bbox_ratio, t))
                    .max(local_ceiling);
                assert!(
                    step >= local_ceiling,
                    "t={t} diag={diag}: escape step {step} must not fall to a local move"
                );
            }
        }
    }

    #[test]
    fn escape_step_scales_with_intensity_and_bbox() {
        let cfg = ConflictBurstConfig::default();
        let step_of = |t: f32, diag: f32| {
            lerp_ratio(cfg.escape_move_mm, t).max(diag * lerp_ratio(cfg.escape_move_bbox_ratio, t))
        };
        // 작은 아이템은 mm 하한이 지배한다
        assert!((step_of(0.0, 10.0) - 10.0).abs() < 1.0e-6);
        // 큰 아이템은 bbox 비율이 지배한다
        assert!((step_of(0.0, 500.0) - 100.0).abs() < 1.0e-6);
        // 강도가 커지면 step도 커진다
        assert!(step_of(1.0, 10.0) > step_of(0.0, 10.0));
        assert!(step_of(1.0, 500.0) > step_of(0.0, 500.0));
    }

    #[test]
    fn bbox_diagonal_is_the_euclidean_extent() {
        assert!((bbox_diagonal(rect(0.0, 0.0, 3.0, 4.0)) - 5.0).abs() < 1.0e-6);
    }

    // ── pair matrix 전수 스캔 ────────────────────────────────────────────────

    /// `PairMatrix::calc_idx`와 동일한 인덱스 규칙 (참조 구현).
    fn ref_idx(row: usize, col: usize, size: usize) -> usize {
        if row <= col {
            row * size + col - (row * (row + 1)) / 2
        } else {
            col * size + row - (col * (col + 1)) / 2
        }
    }

    /// 크기 `size`의 상삼각 데이터를 만들고 지정한 쌍에만 loss를 넣는다.
    pub(super) fn tri(size: usize, pairs: &[(usize, usize, f32)]) -> Vec<CTEntry> {
        let mut data = vec![
            CTEntry {
                loss: 0.0,
                weight: 1.0,
            };
            size * (size + 1) / 2
        ];
        for &(r, c, loss) in pairs {
            data[ref_idx(r, c, size)].loss = loss;
        }
        data
    }

    /// `(edges, deg)` → 오름차순 인접 리스트 (`ConflictGraph::build`와 같은 방식).
    pub(super) fn to_adjacency(edges: &[(u32, u32)], deg: &[u32]) -> Vec<Vec<usize>> {
        let mut adj: Vec<Vec<usize>> = deg
            .iter()
            .map(|&d| Vec::with_capacity(d as usize))
            .collect();
        for &(a, b) in edges {
            adj[a as usize].push(b as usize);
            adj[b as usize].push(a as usize);
        }
        for a in adj.iter_mut() {
            a.sort_unstable();
        }
        adj
    }

    /// 최적화 이전의 전수 이중 루프 (참조 구현).
    fn naive_adjacency(data: &[CTEntry], size: usize, nodes: &[u32], n: usize) -> Vec<Vec<usize>> {
        // 노드 → 행 역매핑
        let mut row_of = vec![usize::MAX; n];
        for (row, &node) in nodes.iter().enumerate() {
            if node != NO_NODE {
                row_of[node as usize] = row;
            }
        }
        let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
        for i in 0..n {
            let row_i = row_of[i];
            if row_i == usize::MAX {
                continue;
            }
            for j in (i + 1)..n {
                let row_j = row_of[j];
                if row_j == usize::MAX {
                    continue;
                }
                if data[ref_idx(row_i, row_j, size)].loss > 0.0 {
                    adj[i].push(j);
                    adj[j].push(i);
                }
            }
        }
        adj
    }

    #[test]
    fn upper_row_slice_matches_pair_matrix_indexing() {
        for size in 1..12usize {
            let len = size * (size + 1) / 2;
            let data: Vec<CTEntry> = (0..len)
                .map(|k| CTEntry {
                    loss: k as f32,
                    weight: 1.0,
                })
                .collect();
            for row in 0..size {
                let seg = upper_row_slice(&data, size, row);
                assert_eq!(seg.len(), size - row - 1, "size {size} row {row}");
                for (off, e) in seg.iter().enumerate() {
                    let col = row + 1 + off;
                    assert_eq!(e.loss, ref_idx(row, col, size) as f32, "({row},{col})");
                }
            }
        }
    }

    #[test]
    fn row_scan_matches_the_naive_pair_loop() {
        let size = 9;
        let nodes: Vec<u32> = vec![0, 1, 2, 3, 4, 5, 6, 7, 8];
        let pairs = [
            (0usize, 1usize, 0.5f32),
            (1, 2, 2.0),
            (1, 3, 1.0e-7),
            (3, 4, 7.0),
            (6, 8, 0.25),
        ];
        let data = tri(size, &pairs);

        let mut edges = Vec::new();
        let mut deg = vec![0u32; size];
        scan_rows_with(
            &data,
            size,
            &nodes,
            &mut edges,
            &mut deg,
            positive_offsets_scalar,
        );

        assert_eq!(
            to_adjacency(&edges, &deg),
            naive_adjacency(&data, size, &nodes, size)
        );
        // 고립 노드(5, 7)는 차수 0
        assert_eq!(deg[5], 0);
        assert_eq!(deg[7], 0);
        // A─B─C, D─E 구조가 그대로 잡혔는가
        assert_eq!(to_adjacency(&edges, &deg)[1], vec![0, 2, 3]);
    }

    #[test]
    fn row_scan_honours_permuted_and_missing_rows() {
        // 행 순서가 노드 순서와 다르고, 3번 행은 어떤 노드에도 대응하지 않는다.
        let size = 6;
        let nodes: Vec<u32> = vec![2, 0, 4, NO_NODE, 1, 3];
        let data = tri(
            size,
            &[
                (0, 1, 1.0),
                (0, 3, 9.0), // NO_NODE 행과의 충돌 → edge 없음
                (2, 5, 1.0),
                (4, 5, 1.0),
            ],
        );

        let n = 5;
        let mut edges = Vec::new();
        let mut deg = vec![0u32; n];
        scan_rows_with(
            &data,
            size,
            &nodes,
            &mut edges,
            &mut deg,
            positive_offsets_scalar,
        );

        let adj = to_adjacency(&edges, &deg);
        assert_eq!(adj, naive_adjacency(&data, size, &nodes, n));
        // (0,1) → 노드 2 ─ 노드 0
        assert!(adj[2].contains(&0) && adj[0].contains(&2));
        // NO_NODE 행과의 충돌은 무시된다
        assert_eq!(adj[2], vec![0]);
    }

    // ── ② 정확한 NFP 기반 분리 벡터 ─────────────────────────────────────────

    /// 정점 목록으로 outer ring 하나짜리 `PolygonSet`을 만든다.
    fn poly(vertices: &[(f64, f64)]) -> PolygonSet {
        PolygonSet::from_outer(Ring::new(
            vertices.iter().map(|&(x, y)| swarm_nfp::Point::new(x, y)),
        ))
    }

    /// outer + hole 하나짜리 `PolygonSet`.
    fn poly_with_hole(outer: &[(f64, f64)], hole: &[(f64, f64)]) -> PolygonSet {
        PolygonSet::new(vec![swarm_nfp::Polygon {
            outer: Ring::new(outer.iter().map(|&(x, y)| swarm_nfp::Point::new(x, y))),
            holes: vec![Ring::new(
                hole.iter().map(|&(x, y)| swarm_nfp::Point::new(x, y)),
            )],
        }])
    }

    /// 그래프를 직접 조립한다 (Separator 없이 조회 계층만 검증하기 위한 것).
    fn graph_of(
        keys: Vec<PItemKey>,
        adjacency: Vec<Vec<usize>>,
        exact_sep: Vec<Vec<Option<(f32, f32)>>>,
    ) -> ConflictGraph {
        let mut index_of: SecondaryMap<PItemKey, usize> = SecondaryMap::new();
        for (i, &pk) in keys.iter().enumerate() {
            index_of.insert(pk, i);
        }
        let terms = vec![ConflictEscapeScoreTerms::default(); keys.len()];
        ConflictGraph {
            keys,
            index_of,
            adjacency,
            terms,
            n_conflicting: 0,
            exact_sep,
        }
    }

    fn two_keys() -> (slotmap::SlotMap<PItemKey, ()>, PItemKey, PItemKey) {
        let mut sm: slotmap::SlotMap<PItemKey, ()> = slotmap::SlotMap::with_key();
        let a = sm.insert(());
        let b = sm.insert(());
        (sm, a, b)
    }

    /// (a) 플래그가 꺼지면 — `build`(캐시 없음) 경로 — 저장된 정확 벡터가 아예 없고,
    /// `separation_vector`는 기존 bbox MTV 분기로 떨어진다.
    #[test]
    fn exact_separation_is_absent_without_a_cache() {
        let (_sm, a, b) = two_keys();
        let graph = graph_of(vec![a, b], vec![vec![1], vec![0]], Vec::new());
        assert!(graph.exact_sep.is_empty());
        assert_eq!(graph.stored_separation(a, b), None);
        assert_eq!(graph.stored_separation(b, a), None);
    }

    /// (b) 스냅샷이 있으면 그 값이 그대로 쓰이고, 반대 방향은 부호가 뒤집힌다.
    #[test]
    fn stored_exact_separation_is_used_and_antisymmetric() {
        let (_sm, a, b) = two_keys();
        let graph = graph_of(
            vec![a, b],
            vec![vec![1], vec![0]],
            vec![vec![Some((3.0, -4.0))], vec![Some((-3.0, 4.0))]],
        );
        assert_eq!(graph.stored_separation(a, b), Some((3.0, -4.0)));
        assert_eq!(graph.stored_separation(b, a), Some((-3.0, 4.0)));

        // NFP를 못 얻은 edge는 None → 호출부가 bbox MTV로 폴백한다.
        let fallback = graph_of(
            vec![a, b],
            vec![vec![1], vec![0]],
            vec![vec![None], vec![None]],
        );
        assert_eq!(fallback.stored_separation(a, b), None);

        // 충돌 edge가 아닌 쌍도 저장값이 없다.
        let disjoint = graph_of(
            vec![a, b],
            vec![Vec::new(), Vec::new()],
            vec![Vec::new(), Vec::new()],
        );
        assert_eq!(disjoint.stored_separation(a, b), None);
    }

    #[test]
    fn point_in_ring_is_winding_agnostic() {
        let ccw = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        let cw: Vec<(f64, f64)> = ccw.iter().rev().copied().collect();
        for verts in [ccw.to_vec(), cw] {
            let ring = Ring::new(verts.iter().map(|&(x, y)| swarm_nfp::Point::new(x, y)));
            assert!(point_in_ring(&ring, 5.0, 5.0));
            assert!(!point_in_ring(&ring, 15.0, 5.0));
            assert!(!point_in_ring(&ring, 5.0, -0.5));
        }
    }

    /// 오목(대각 변) 도형에서는 최단 분리 방향이 축 정렬이 아니다 — bbox MTV가
    /// 원리적으로 낼 수 없는 답이다.
    #[test]
    fn exit_vector_is_diagonal_where_bbox_mtv_can_only_be_axis_aligned() {
        // 빗변 x + y = 10 을 가진 직각삼각형 NFP.
        let region = poly(&[(0.0, 0.0), (10.0, 0.0), (0.0, 10.0)]);
        // (4,4)에서 두 축 변까지는 4, 빗변까지는 |4+4-10|/√2 ≈ 1.414 → 빗변이 최단.
        let (dx, dy) = exit_vector(&region, 4.0, 4.0);
        assert!((dx - 1.0).abs() < 1.0e-9, "dx = {dx}");
        assert!((dy - 1.0).abs() < 1.0e-9, "dy = {dy}");

        // 축 정렬이 아니다 = bbox MTV가 절대 못 내는 방향이다.
        assert!(dx.abs() > 1.0e-9 && dy.abs() > 1.0e-9);
        // 같은 NFP를 bbox로 근사하면(0..10 × 0..10) 최소 이동은 축으로 4가 된다.
        // 정확 벡터는 그보다 훨씬 짧다 — 과잉 이동을 하지 않는다는 뜻이다.
        let exact_norm = (dx * dx + dy * dy).sqrt();
        assert!(exact_norm < 4.0, "exact {exact_norm} vs bbox 4.0");

        // 밖에 있는 점은 분리할 필요가 없다.
        assert_eq!(exit_vector(&region, 8.0, 8.0), (0.0, 0.0));
    }

    #[test]
    fn exit_vector_picks_the_nearest_edge_of_a_concave_notch() {
        // 위쪽에 깊은 만입부가 있는 U자 NFP.
        let region = poly(&[
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (7.0, 10.0),
            (7.0, 3.0),
            (3.0, 3.0),
            (3.0, 10.0),
            (0.0, 10.0),
        ]);
        // 만입부 바로 아래의 점: 가장 가까운 경계는 만입부 바닥(y = 3)이다.
        let (dx, dy) = exit_vector(&region, 5.0, 2.5);
        assert!(dx.abs() < 1.0e-9, "dx = {dx}");
        assert!((dy - 0.5).abs() < 1.0e-9, "dy = {dy}");

        // 만입부 안(= NFP 밖)은 겹침이 아니다.
        assert_eq!(exit_vector(&region, 5.0, 6.0), (0.0, 0.0));

        // 왼쪽 기둥 안의 점은 가장 가까운 세로 변으로 빠진다.
        let (dx2, dy2) = exit_vector(&region, 2.5, 8.0);
        assert!((dx2 - 0.5).abs() < 1.0e-9, "dx2 = {dx2}");
        assert!(dy2.abs() < 1.0e-9, "dy2 = {dy2}");
    }

    #[test]
    fn exit_vector_treats_holes_as_free_pockets() {
        let region = poly_with_hole(
            &[(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)],
            &[(8.0, 8.0), (12.0, 8.0), (12.0, 12.0), (8.0, 12.0)],
        );
        // hole 안 = NFP 밖 = 겹침 아님.
        assert_eq!(exit_vector(&region, 10.0, 10.0), (0.0, 0.0));
        // hole 바로 바깥은 hole 경계가 가장 가깝다 → 포켓으로 밀어 넣는 것이 최소 이동.
        let (dx, dy) = exit_vector(&region, 10.0, 7.5);
        assert!(dx.abs() < 1.0e-9, "dx = {dx}");
        assert!((dy - 0.5).abs() < 1.0e-9, "dy = {dy}");
    }

    /// `NFP(A,B) = -NFP(B,A)`. 부호 반전 경로가 같은 답을 내는지 확인한다
    /// (`nfp_separation_vector`가 정규 키만 질의하고 스스로 뒤집는 근거).
    #[test]
    fn exit_vector_is_antisymmetric_under_region_negation() {
        let region = poly(&[(0.0, 0.0), (10.0, 0.0), (0.0, 10.0)]);
        let negated = poly(&[(0.0, 0.0), (-10.0, 0.0), (0.0, -10.0)]);
        let (dx, dy) = exit_vector(&region, 4.0, 4.0);
        let (nx, ny) = exit_vector(&negated, -4.0, -4.0);
        assert!((nx + dx).abs() < 1.0e-9, "{nx} vs {dx}");
        assert!((ny + dy).abs() < 1.0e-9, "{ny} vs {dy}");
    }

    #[test]
    fn rotation_degrees_matches_the_library_grid() {
        // i64 도 → f32 라디안 왕복은 허용 오차 안이다.
        for deg in [0i64, 90, 180, 270, 45] {
            let rad = (deg as f64).to_radians() as f32;
            assert_eq!(rotation_degrees(rad), Some(deg));
        }
        // 음수 각은 [0, 360)으로 정규화된다.
        let rad = (-90.0f64).to_radians() as f32;
        assert_eq!(rotation_degrees(rad), Some(270));
        // 라이브러리에 없는 연속 회전은 폴백시킨다.
        assert_eq!(rotation_degrees(0.5f32.to_radians() + 0.001), None);
        assert_eq!(rotation_degrees(f32::NAN), None);
        assert_eq!(rotation_degrees(f32::INFINITY), None);
    }

    #[test]
    fn union_find_merges_transitive_conflicts() {
        // A ─ B ─ C 그리고 D ─ E, 고립된 F
        let mut parent: Vec<usize> = (0..6).collect();
        for (a, b) in [(0usize, 1usize), (1, 2), (3, 4)] {
            let (ra, rb) = (uf_find(&mut parent, a), uf_find(&mut parent, b));
            parent[ra] = rb;
        }
        assert_eq!(uf_find(&mut parent, 0), uf_find(&mut parent, 2));
        assert_eq!(uf_find(&mut parent, 3), uf_find(&mut parent, 4));
        assert_ne!(uf_find(&mut parent, 0), uf_find(&mut parent, 3));
        assert_ne!(uf_find(&mut parent, 5), uf_find(&mut parent, 0));
    }
}

#[cfg(all(test, feature = "simd"))]
mod simd_tests {
    use super::tests::{to_adjacency, tri};
    use super::*;

    /// 스칼라 경로 결과.
    fn scalar(
        data: &[CTEntry],
        size: usize,
        nodes: &[u32],
        n: usize,
    ) -> (Vec<(u32, u32)>, Vec<u32>) {
        let mut edges = Vec::new();
        let mut deg = vec![0u32; n];
        scan_rows_with(
            data,
            size,
            nodes,
            &mut edges,
            &mut deg,
            positive_offsets_scalar,
        );
        (edges, deg)
    }

    /// 폭 `W`의 SIMD 커널 결과.
    fn simd_w<const W: usize>(
        data: &[CTEntry],
        size: usize,
        nodes: &[u32],
        n: usize,
    ) -> (Vec<(u32, u32)>, Vec<u32>) {
        let mut edges = Vec::new();
        let mut deg = vec![0u32; n];
        simd_scan::scan_rows_width::<W>(data, size, nodes, &mut edges, &mut deg);
        (edges, deg)
    }

    /// 런타임 디스패치 결과.
    fn simd_dispatch(
        data: &[CTEntry],
        size: usize,
        nodes: &[u32],
        n: usize,
    ) -> (Vec<(u32, u32)>, Vec<u32>) {
        let mut edges = Vec::new();
        let mut deg = vec![0u32; n];
        simd_scan::scan_rows(data, size, nodes, &mut edges, &mut deg);
        (edges, deg)
    }

    /// 결정적인 유사 난수 (테스트 재현성 확보용).
    fn lcg(state: &mut u64) -> u32 {
        *state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1);
        (*state >> 33) as u32
    }

    /// `loss > 0` 판정은 누산이 아니라 순수 비교이므로 두 경로가 **완전히** 같아야 한다.
    #[test]
    fn simd_and_scalar_row_scans_are_bit_identical() {
        let mut state = 0x5EED_1234u64;
        // 잔여 처리(size가 W의 배수가 아닌 경우)까지 덮도록 다양한 크기를 돈다.
        for size in 1..=40usize {
            for density in [0u32, 3, 25, 100] {
                let mut pairs: Vec<(usize, usize, f32)> = Vec::new();
                for r in 0..size {
                    for c in (r + 1)..size {
                        if lcg(&mut state) % 100 < density {
                            // 아주 작은 값도 0이 아니면 edge다.
                            let loss = 1.0e-30 + (lcg(&mut state) % 1000) as f32;
                            pairs.push((r, c, loss));
                        }
                    }
                }
                let data = tri(size, &pairs);
                let nodes: Vec<u32> = (0..size as u32).collect();

                let expected = scalar(&data, size, &nodes, size);
                assert_eq!(
                    simd_w::<4>(&data, size, &nodes, size),
                    expected,
                    "W=4 size={size} d={density}"
                );
                assert_eq!(
                    simd_w::<8>(&data, size, &nodes, size),
                    expected,
                    "W=8 size={size} d={density}"
                );
                assert_eq!(
                    simd_w::<16>(&data, size, &nodes, size),
                    expected,
                    "W=16 size={size} d={density}"
                );
                assert_eq!(
                    simd_dispatch(&data, size, &nodes, size),
                    expected,
                    "dispatch size={size} d={density}"
                );
                // 인접 리스트(그래프 구조)까지 동일한가
                assert_eq!(
                    to_adjacency(&expected.0, &expected.1),
                    to_adjacency(&simd_dispatch(&data, size, &nodes, size).0, &expected.1)
                );
            }
        }
    }

    /// `weight`(기본 1.0 > 0)가 loss로 오인되지 않아야 한다.
    #[test]
    fn simd_scan_never_mistakes_weight_for_loss() {
        let size = 33;
        // loss는 전부 0, weight만 큰 값
        let mut data = tri(size, &[]);
        for e in data.iter_mut() {
            e.weight = 12_345.0;
        }
        let nodes: Vec<u32> = (0..size as u32).collect();

        for got in [
            simd_w::<4>(&data, size, &nodes, size),
            simd_w::<8>(&data, size, &nodes, size),
            simd_w::<16>(&data, size, &nodes, size),
            simd_dispatch(&data, size, &nodes, size),
        ] {
            assert!(got.0.is_empty(), "weight를 loss로 오인했다: {:?}", got.0);
            assert!(got.1.iter().all(|&d| d == 0));
        }
    }

    /// 음수/NaN loss는 edge가 아니다 (`> 0.0` 비교와 정확히 일치).
    #[test]
    fn simd_scan_rejects_non_positive_losses() {
        let size = 20;
        let data = tri(
            size,
            &[
                (0, 1, -1.0),
                (0, 2, f32::NAN),
                (0, 3, -0.0),
                (0, 4, 0.0),
                (0, 5, f32::MIN_POSITIVE),
                (1, 19, f32::INFINITY),
            ],
        );
        let nodes: Vec<u32> = (0..size as u32).collect();
        let expected = scalar(&data, size, &nodes, size);
        assert_eq!(expected.0, vec![(0, 5), (1, 19)]);
        assert_eq!(simd_w::<4>(&data, size, &nodes, size), expected);
        assert_eq!(simd_w::<8>(&data, size, &nodes, size), expected);
        assert_eq!(simd_w::<16>(&data, size, &nodes, size), expected);
        assert_eq!(simd_dispatch(&data, size, &nodes, size), expected);
    }
}
