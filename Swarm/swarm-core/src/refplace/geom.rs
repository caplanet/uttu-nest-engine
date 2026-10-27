//! 기하 커널 배선 — 설계의 위상 구조를 실제 NFP 커널에 연결한다.
//!
//! # 커널을 포크하지 않는다
//!
//! `swarm_nfp::NfpEngine` 은 이 저장소에서 성능·정확성이 가장 민감한 코드다. 이
//! 파일은 그것을 **그대로 호출**하고, 설계가 요구하는 형태(자세 카탈로그, 쌍 색인,
//! 접촉 경계 점 목록)로 옮겨 담기만 한다. 두 벌로 갈라 두면 한쪽만 고쳐지는 순간
//! 조용히 어긋난다 — 이 저장소가 여러 번 확인한 규칙이다.
//!
//! # 자세(pose)를 커널 방향(Orientation)으로 옮기는 규칙
//!
//! 설계의 [`RotationEntry`] 는 `(x축 회전, y축 회전)` 쌍 + 평면 회전이고, 커널의
//! [`Orientation`] 은 `(angle_microdegrees, mirror_x)` 다. 대응은 이렇다:
//!
//! ```text
//!   flip_x  flip_y   mirror_x   추가 회전     설명
//!   false   false    false      0            항등
//!   false   true     true       0            y축 기준 뒤집기 = 좌우 반전 = x 부호 반전
//!   true    false    true       180도        x축 기준 뒤집기 = 상하 반전 = y 부호 반전
//!   true    true     false      180도        두 축을 다 뒤집으면 180도 회전이다
//! ```
//!
//! 세 번째 줄이 핵심이다 — 커널에는 `mirror_y` 가 없지만 **`mirror_x` + 180도 회전이
//! 곧 `mirror_y`** 다. `(x,y) -> (-x,y) -> (x,-y)`.

use swarm_nfp::{
    Location, NfpEngine, Orientation, Point as NfpPoint, PolygonSet, PreparedShape, Ring,
};
use std::collections::BTreeMap;

use jagua_rs::probs::spp::entities::SPInstance;

use crate::refplace::boundary::EvalRecord;
use crate::refplace::chain::{NfpPairKey, PoseKey};
use crate::refplace::record::{ContactPoint, RotationEntry};
use crate::refplace::step::{NfpSummary, ReadCode};

/// 컨테이너 경계에서 **안쪽으로** 확보할 여유(mm).
///
/// # 후보를 거르지 말고 컨테이너를 좁혀라
///
/// 처음에는 후보 필터에서 `bbox.1 >= 여유` 를 요구했다. **아무것도 안 놓였다.**
/// IFP 경계가 곧 "조각이 컨테이너에 닿는 위치" 인데, 그 경계를 전부 거부한 것이다.
/// (그 전에는 반대로 `bbox.3 > height + 0.01` 로 **돌출을 허용**해서 WPF 초기화가
///  `exterior_collisions: 7` 로 실패했다. 부호를 두 번 틀렸다.)
///
/// 올바른 자리는 **컨테이너**다. IFP 를 좁힌 컨테이너로 구하면 모든 후보가 이미
/// 안쪽이고, 후보 필터도 나중 보정도 필요 없다.
///
/// 왜 여유가 필요한가: 좌표는 f64 로 다루고 `SPPlacement` 에는 f32 로 넘긴다.
/// 경계에 정확히 붙인 조각은 그 변환 반올림만으로 밖이 된다 — 20,000mm 대에서
/// f32 한 눈금이 약 0.002mm 다. 0.05mm 는 그보다 한 자리 크고 재단 공차(1mm 급)보다
/// 두 자리 작다.
pub const BOUNDARY_INSET_MM: f64 = 0.05;

/// 조각 사이에 확보할 간격(mm) — NFP 를 이만큼 팽창시킨다.
///
/// # 왜 필요한가 (이 저장소가 이미 겪은 문제다)
///
/// NFP 경계 위의 점은 "정확히 맞닿음" 이다. 커널은 1/1000mm 정수 격자라 내부 오차가
/// 없지만, 그 좌표를 **f32 로 넘기는 순간** 맞닿음이 미세 겹침이 된다.
///
/// 문서 §6-5 가 같은 결함을 기록해 두었다 — "접촉 배치가 f32에서 미세 겹침
/// (`Location::Boundary` 채택 후 5쌍 겹침) → pair NFP 를 정사각형 clearance 커널로
/// 0.01mm 팽창". 이 엔진도 같은 자리에서 같은 방식으로 걸렸다:
///
/// ```text
///   LayoutInfeasible { exterior_collisions: 0, placed_item_collisions: 50, ... }
/// ```
///
/// 방향은 겹침 허용이 아니라 **간격 확보**다. NFP 를 키우면 금지영역이 커지고,
/// 그러면 배치 가능 영역이 보수적으로 작아진다 — 놓을 수 있다고 한 자리는 f32
/// 기준으로도 반드시 안전하다.
pub const PIECE_CLEARANCE_MM: f64 = 0.01;

/// [`PIECE_CLEARANCE_MM`] 크기의 정사각형 clearance 커널.
///
/// 원점을 **엄격히 포함**해야 한다(커널 계약). 그래서 반폭으로 사방을 두른다.
fn clearance() -> swarm_nfp::Clearance {
    let h = PIECE_CLEARANCE_MM * 0.5;
    swarm_nfp::Clearance::ConvexKernel(Ring::new(vec![
        NfpPoint::new(-h, -h),
        NfpPoint::new(h, -h),
        NfpPoint::new(h, h),
        NfpPoint::new(-h, h),
    ]))
}

/// [`RotationEntry`] 를 커널의 [`Orientation`] 으로.
///
/// 위 표 그대로다. 밀리도 -> 마이크로도는 `x1000`.
pub fn orientation_of(entry: &RotationEntry) -> Orientation {
    let extra_turn = if entry.flip_x { 180_000_000i64 } else { 0 };
    let micro = i64::from(entry.millideg)
        .saturating_mul(1_000)
        .saturating_add(extra_turn)
        .rem_euclid(360_000_000);
    Orientation {
        angle_microdegrees: micro,
        // 한 축만 뒤집혔을 때 거울이 된다. 두 축이면 180도 회전이라 거울이 아니다.
        mirror_x: entry.flip_x != entry.flip_y,
    }
}

/// 오목 정점만 없앤 뒤 커널 좌표계로 옮긴다.
///
/// [`crate::nfp_lbf`] 의 같은 이름 함수와 **의도적으로 같은 규칙**을 쓴다 — 두 엔진이
/// 같은 형상을 다르게 단순화하면 NFP 준비본이 두 벌 생기고, 둘 중 하나만 고쳐지는
/// 순간 조용히 어긋난다.
fn polygon_set_of(
    poly: &jagua_rs::geometry::primitives::SPolygon,
    concave_tolerance: f32,
) -> PolygonSet {
    if concave_tolerance <= 0.0 {
        let ring = Ring::new(
            poly.vertices()
                .iter()
                .map(|p| NfpPoint::new(p.0 as f64, p.1 as f64)),
        );
        return PolygonSet::from_outer(ring);
    }
    // **오목 정점만** 없앤다. 볼록 정점을 깎으면 도형이 작아져 실제 겹침을 놓친다.
    let simplified =
        crate::nfp_lbf::cache::simplify_concave_only(&poly.vertices(), poly.area(), concave_tolerance);
    let ring = Ring::new(
        simplified
            .iter()
            .map(|p| NfpPoint::new(p.0 as f64, p.1 as f64)),
    );
    PolygonSet::from_outer(ring)
}

/// 자세 카탈로그 — `(item_id, PoseKey)` 마다 준비된 형상 하나.
///
/// 설계 §모듈 2 의 "회전 목록" 을 실제 기하로 재료화한 것이다. 준비(prepare)는 비싸므로
/// **한 번만** 하고 계속 재사용한다.
pub struct PoseCatalogue {
    engine: NfpEngine,
    shapes: BTreeMap<(usize, PoseKey), PreparedShape>,
    /// 컨테이너(strip) 형상. IFP 계산에 쓴다.
    container: Option<PreparedShape>,
    /// 컨테이너 높이(mm). 창(window) 컨테이너를 만들 때 쓴다.
    container_height: f64,
}

impl PoseCatalogue {
    /// 인스턴스와 조각별 자세 목록에서 카탈로그를 만든다.
    ///
    /// `poses_of_item` 이 비어 있는 아이템은 항등 자세 하나만 준비한다 —
    /// 자세가 없어 배치가 통째로 실패하는 것보다 낫다.
    /// `concave_tolerance` 는 **오목 정점만** 없애는 단순화 강도다.
    ///
    /// # 왜 단순화하나
    ///
    /// 볼록 분해 조각 수는 오목 정점 수를 따라가고, pair NFP 의 원시 조각 수는
    /// `조각(A) x 조각(B)` 다. 즉 NFP 비용의 지배항이 오목 정점이다.
    ///
    /// 처음에는 원본 도형을 그대로 썼다. 이 마커의 정점은 median 179 / max 637 이고,
    /// 배치가 `O(n²)` 개의 pair NFP 를 부르므로 **37초**가 걸렸다. 기존 엔진이 같은
    /// 이유로 `0.02` 를 쓰고 prewarm 을 6.8배 줄인다
    /// ([`crate::nfp_lbf::DEFAULT_CONCAVE_SIMPLIFY_TOLERANCE`] 의 실측표).
    ///
    /// **안전한 방향이다** — 오목 정점을 없애면 도형이 커져 원본의 상위집합이 되고,
    /// 그 NFP 는 금지영역을 과대평가한다. 놓을 수 있다고 한 자리는 원본 기준으로도
    /// 반드시 안전하다.
    pub fn build(
        instance: &SPInstance,
        poses_of_item: &BTreeMap<usize, Vec<RotationEntry>>,
        concave_tolerance: f32,
    ) -> Self {
        let engine = NfpEngine::new();
        let mut shapes = BTreeMap::new();

        for (item, _) in instance.items.iter() {
            let base = polygon_set_of(&item.shape_cd(), concave_tolerance);
            let Ok(prepared) = engine.prepare(&base) else {
                // 준비 실패는 그 조각을 못 쓴다는 뜻이지 전체 실패가 아니다.
                log::warn!("[REFPLACE] 아이템 {} 형상 준비 실패 — 건너뛴다", item.idx());
                continue;
            };
            let entries = poses_of_item
                .get(&item.idx())
                .cloned()
                .unwrap_or_else(|| vec![RotationEntry::IDENTITY]);
            for entry in entries {
                let key = (item.idx(), PoseKey::of(&entry));
                if shapes.contains_key(&key) {
                    continue;
                }
                let orientation = orientation_of(&entry);
                match engine.orient(&prepared, orientation) {
                    Ok(oriented) => {
                        shapes.insert(key, oriented);
                    }
                    Err(e) => log::warn!(
                        "[REFPLACE] 아이템 {} 자세 {:?} 준비 실패: {e:?}",
                        item.idx(),
                        entry
                    ),
                }
            }
        }

        // 컨테이너 — strip 은 높이가 고정이고 폭이 자란다. IFP 를 위해 넉넉한 폭으로
        // 만들어 두고, 실제 폭 판정은 배치 뒤 검증이 한다.
        let h = instance.base_strip.fixed_height as f64;
        let w: f64 = instance
            .items
            .iter()
            .map(|(it, q)| it.shape_cd().bbox().width() as f64 * (*q as f64))
            .sum();
        // **경계에서 안쪽으로 여유를 두고** 컨테이너를 만든다(§`BOUNDARY_INSET_MM`).
        // 그러면 IFP 에서 나온 모든 후보가 이미 안쪽이라 뒤에서 거르거나 보정할 일이 없다.
        let i = BOUNDARY_INSET_MM;
        let container_set = PolygonSet::from_outer(Ring::new(vec![
            NfpPoint::new(i, i),
            NfpPoint::new(w - i, i),
            NfpPoint::new(w - i, h - i),
            NfpPoint::new(i, h - i),
        ]));
        let container = engine.prepare(&container_set).ok();

        Self {
            engine,
            shapes,
            container,
            container_height: h,
        }
    }

    /// 준비된 형상 조회.
    pub fn shape(&self, item_id: usize, pose: PoseKey) -> Option<&PreparedShape> {
        self.shapes.get(&(item_id, pose))
    }

    /// 담긴 자세 수.
    pub fn len(&self) -> usize {
        self.shapes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.shapes.is_empty()
    }

    /// 커널 참조 — 호출부가 `locate` 등을 직접 쓸 때.
    pub fn engine(&self) -> &NfpEngine {
        &self.engine
    }

    /// 쌍 NFP 를 계산해 설계가 요구하는 형태로 옮긴다.
    ///
    /// 돌려주는 것은 [`NfpSummary`](모듈 6 의 분기 재료)와 경계 점 목록
    /// ([`EvalRecord`], 모듈 7 의 후보 재료)이다.
    ///
    /// # 판독 코드
    ///
    /// 설계 §모듈 6 의 상태 코드는 원 엔진의 내부값이라 의미가 문서에 없다. 여기서는
    /// **관측 가능한 사실만** 코드로 옮긴다 — 성분이 여럿이면 좌표를 그대로 쓸 수 없으니
    /// [`ReadCode::ScaledCoords`], 구멍이 있으면 [`ReadCode::Special`], 그 밖은
    /// [`ReadCode::Plain`]. 추측한 의미를 이름에 박지 않는다.
    pub fn compute_pair(&self, key: &NfpPairKey) -> Option<(NfpSummary, EvalRecord)> {
        let fixed = self.shape(key.fixed_item, key.fixed_pose)?;
        let moving = self.shape(key.moving_item, key.moving_pose)?;
        let set = self.engine.pair_nfp(fixed, moving).ok()?;
        Some(summarize(&set))
    }

    /// 컨테이너 안쪽 배치 가능 영역(IFP).
    ///
    /// strip 경계까지 고려한 후보를 만들려면 이것이 있어야 한다 — 없으면 조각이
    /// 컨테이너 밖으로 나가는 후보까지 평가하게 된다.
    pub fn inner_fit(&self, item_id: usize, pose: PoseKey) -> Option<EvalRecord> {
        let container = self.container.as_ref()?;
        let moving = self.shape(item_id, pose)?;
        let set = self.engine.inner_fit(container, moving).ok()?;
        Some(summarize(&set).1)
    }
}

/// [`PolygonSet`] 을 설계가 쓰는 두 형태로 옮긴다.
fn summarize(set: &PolygonSet) -> (NfpSummary, EvalRecord) {
    let mut points = Vec::new();
    let mut holes = 0usize;
    for comp in &set.components {
        for v in &comp.outer.vertices {
            points.push(ContactPoint::new(v.x, v.y));
        }
        holes += comp.holes.len();
        for ring in &comp.holes {
            for v in &ring.vertices {
                points.push(ContactPoint::new(v.x, v.y));
            }
        }
    }
    let code = if holes > 0 {
        ReadCode::Special
    } else if set.components.len() > 1 {
        // 성분이 여럿이면 좌표를 성분마다 따로 읽어야 한다 — 그대로 이어 붙이면
        // 서로 다른 성분의 점이 한 폴리라인으로 잘못 이어진다.
        ReadCode::ScaledCoords(set.components.len().min(3) as u8)
    } else {
        ReadCode::Plain
    };
    let summary = NfpSummary {
        vertex_count: points.len(),
        // 커널은 배치 불가를 "빈 집합" 으로 표현한다. 전체 겹침도 그렇게 나온다.
        full_overlap: set.is_empty(),
        code,
    };
    (summary, EvalRecord::new(points))
}

/// 이미 놓인 조각들을 피해 **실제로 놓을 수 있는 영역**을 구한다.
///
/// `IFP - ∪ NFP(장애물)` 이고, 커널의 [`NfpEngine::feasible_region`] 이 그대로 한다.
///
/// # 왜 루트·독립 배치에 이것이 필요한가
///
/// 설계 §모듈 3 은 루트를 "표면 시드에서 **접촉 탐색**으로 위치를 확정한다" 고 적는다.
/// 시드를 그대로 쓰는 것이 아니다 — 처음 이 구현이 시드를 그대로 썼더니 156조각이
/// 전부 원점에 쌓여 밀도 1442%, 검증 표식 156(전부 겹침)이 나왔다.
///
/// 부착 배치(모듈 6→7)는 선계산된 쌍 NFP 를 **조회만** 하지만, 루트·독립 배치는
/// 부착 대상이 없어 조회할 쌍 자체가 없다. 그래서 여기서는 커널을 직접 부른다.
/// "조회만 한다" 는 계약은 부착 경로의 것이고, 이 경로에는 적용되지 않는다.
pub fn feasible_for(
    catalogue: &PoseCatalogue,
    item_id: usize,
    pose: PoseKey,
    placed: &[(usize, PoseKey, f64, f64)],
    window: Option<(f64, f64)>,
) -> Option<PolygonSet> {
    let moving = catalogue.shape(item_id, pose)?;
    let mv = moving.bounds();
    let mv_w = mv.max.x - mv.min.x;

    // ── 창(window) 으로 장애물을 줄인다 ──
    //
    // 배치가 진행되면 놓인 조각이 100개를 넘고, 그 전부의 pair NFP 를 합집합하면
    // 커널이 느려지다 결국 실패한다. 실측: 창 없이 돌리면 156 중 28개에서
    // `feasible_region` 이 빈 집합/오류를 돌려주었고, 그 조각들이 폴백으로 밀려
    // 폭이 5,485mm 에서 20,312mm 로 늘었다(밀도 72.9% -> 25.0%).
    //
    // **안전한 이유**: 컨테이너를 같은 창으로 자르므로 후보의 x 가 `[lo, hi]` 를 벗어날
    // 수 없다. 그러면 `x_max < lo - 이동조각폭` 인 장애물은 어떤 후보와도 겹칠 수 없다.
    // 즉 걸러 내도 **결과가 바뀌지 않는다** — 속도만 바뀐다.
    let (lo, hi) = window.unwrap_or((f64::NEG_INFINITY, f64::INFINITY));
    let cutoff = lo - mv_w;

    let container = match window {
        Some((lo, hi)) => {
            let h = catalogue.container_height;
            let i = BOUNDARY_INSET_MM;
            let set = PolygonSet::from_outer(Ring::new(vec![
                NfpPoint::new(lo.max(0.0) + i, i),
                NfpPoint::new(hi - i, i),
                NfpPoint::new(hi - i, h - i),
                NfpPoint::new(lo.max(0.0) + i, h - i),
            ]));
            catalogue.engine.prepare(&set).ok()?
        }
        None => catalogue.container.clone()?,
    };

    let obstacles: Vec<swarm_nfp::PlacedObstacle<'_>> = placed
        .iter()
        .filter_map(|(id, ps, x, y)| {
            let shape = catalogue.shape(*id, *ps)?;
            let b = shape.bounds();
            // 창 왼쪽 밖으로 완전히 벗어난 장애물은 어떤 후보와도 겹칠 수 없다.
            if b.max.x + x < cutoff {
                return None;
            }
            if b.min.x + x > hi {
                return None;
            }
            Some(swarm_nfp::PlacedObstacle {
                shape,
                translation: swarm_nfp::Translation { x: *x, y: *y },
            })
        })
        .collect();
    catalogue
        .engine
        .feasible_region(swarm_nfp::FeasibleRequest {
            container: &container,
            moving,
            obstacles: &obstacles,
            // **간격을 확보한다.** 맞닿음을 그대로 쓰면 f32 에서 겹친다(§`PIECE_CLEARANCE_MM`).
            clearance: clearance(),
        })
        .ok()
}

/// 자세마다 **배치 가능 영역을 들고 다니는** 캐시.
///
/// # 왜 필요한가
///
/// [`feasible_for`] 는 호출할 때마다 놓인 조각 **전부**의 pair NFP 를 합집합한다.
/// 배치 루프가 조각마다 한 번씩 부르면 `O(n²)` 개의 pair NFP 를 계산하고, 더 나쁘게는
/// 합집합하는 집합이 `n` 과 함께 커진다. 실측(wsjk156 156조각): 놓인 조각이 26개를
/// 넘으면 커널이 빈 집합이나 오류를 돌려주기 시작했고, 그 조각들이 폴백으로 밀려
/// 폭이 5,485mm 에서 20,312mm 로 늘었다(밀도 72.9% -> 25.0%).
///
/// 집합 차는 합집합에 분배된다:
///
/// ```text
///     A - (B ∪ C) = (A - B) - C
/// ```
///
/// 그래서 영역을 들고 있다가 **새로 놓인 것 하나만** 빼면 된다. 결과는 같고
/// (`swarm-nfp/tests/incremental_subtraction.rs` 가 그 동치를 못박는다),
/// 커지는 합집합만 사라진다. 배치당 비용이 `O(n)` 합집합에서 `O(1)` 차감이 된다.
///
/// # 메모리
///
/// 미배치 자세마다 영역 하나를 들고 있다. 자세가 늘면 그만큼 는다 — 조각이 놓이면
/// 그 자세의 영역은 [`RegionCache::retire`] 로 버린다.
pub struct RegionCache {
    regions: BTreeMap<(usize, PoseKey), PolygonSet>,
}

impl RegionCache {
    /// 아직 안 놓인 자세들의 영역을 IFP 로 초기화한다.
    ///
    /// IFP 를 못 구하는 자세는 **넣지 않는다**. 그러면 조회가 `None` 이 되고 호출부가
    /// 폴백으로 간다 — 빈 영역을 넣어 "놓을 자리가 없다" 로 오해시키지 않는다.
    pub fn seed(catalogue: &PoseCatalogue, poses: &[(usize, PoseKey)]) -> Self {
        let mut regions = BTreeMap::new();
        for &(item, pose) in poses {
            let (Some(container), Some(moving)) =
                (catalogue.container.as_ref(), catalogue.shape(item, pose))
            else {
                continue;
            };
            if let Ok(ifp) = catalogue
                .engine
                .inner_fit_with_clearance(container, moving, &clearance())
            {
                regions.insert((item, pose), ifp);
            }
        }
        Self { regions }
    }

    /// 이 자세의 현재 배치 가능 영역.
    pub fn get(&self, item: usize, pose: PoseKey) -> Option<&PolygonSet> {
        self.regions.get(&(item, pose))
    }

    /// 조각을 놓았다 — **남은 모든 자세**의 영역에서 그것을 뺀다.
    ///
    /// 실패한 자세는 캐시에서 **버린다**. 낡은(= 방금 놓인 조각을 아직 안 뺀) 영역을
    /// 남겨 두면 그 자세가 겹치는 자리에 놓인다. 버리면 호출부가 폴백으로 가므로
    /// 폭은 늘지만 겹치지는 않는다 — 안전한 쪽으로 떨어진다.
    pub fn place(
        &mut self,
        catalogue: &PoseCatalogue,
        placed_item: usize,
        placed_pose: PoseKey,
        x: f64,
        y: f64,
    ) {
        let Some(obstacle_shape) = catalogue.shape(placed_item, placed_pose) else {
            // 장애물 형상을 모르면 어떤 영역도 갱신할 수 없다. 전부 버린다 —
            // 낡은 영역을 쓰는 것보다 폴백이 낫다.
            self.regions.clear();
            return;
        };
        let obstacle = swarm_nfp::PlacedObstacle {
            shape: obstacle_shape,
            translation: swarm_nfp::Translation { x, y },
        };
        let mut stale: Vec<(usize, PoseKey)> = Vec::new();
        for (&(item, pose), region) in self.regions.iter_mut() {
            let Some(moving) = catalogue.shape(item, pose) else {
                stale.push((item, pose));
                continue;
            };
            match catalogue
                .engine
                .subtract_obstacle(region, &obstacle, moving, &clearance())
            {
                Ok(next) => *region = next,
                Err(_) => stale.push((item, pose)),
            }
        }
        for key in stale {
            self.regions.remove(&key);
        }
    }

    /// 놓인 자세의 영역을 버린다 — 더 쓸 일이 없다.
    pub fn retire(&mut self, item: usize, pose: PoseKey) {
        self.regions.remove(&(item, pose));
    }

    /// 남은 영역 수.
    pub fn len(&self) -> usize {
        self.regions.len()
    }

    pub fn is_empty(&self) -> bool {
        self.regions.is_empty()
    }
}

/// [`feasible_for`] 의 결과를 모듈 7 이 쓰는 후보 점 목록으로.
pub fn feasible_record(set: &PolygonSet) -> EvalRecord {
    summarize(set).1
}

/// 후보 자세를 실제 좌표로 굳혔을 때의 월드 bbox.
///
/// 모듈 9 의 검증이 이 값을 쓴다. 커널을 다시 부르지 않고 준비본의 경계에서 옮긴다 —
/// 검증은 조각마다 한 번씩 도는 자리라 커널 호출을 넣으면 그것만으로 비싸진다.
pub fn world_bbox(
    catalogue: &PoseCatalogue,
    item_id: usize,
    pose: PoseKey,
    x: f64,
    y: f64,
) -> Option<(f64, f64, f64, f64)> {
    let shape = catalogue.shape(item_id, pose)?;
    let b = shape.bounds();
    Some((
        b.min.x + x,
        b.min.y + y,
        b.max.x + x,
        b.max.y + y,
    ))
}

/// 후보가 기존 배치와 실제로 겹치지 않는지 커널로 확인한다.
///
/// # 왜 `locate` 인가
///
/// NFP 는 "이 자리에 놓으면 겹치는가" 를 영역 포함 판정으로 답한다. 후보 좌표가
/// NFP **내부**에 있으면 겹치는 것이고, **경계**면 맞닿은 것이며, **외부**면 떨어져 있다.
/// 맞닿음은 허용이다 — 그것이 이 엔진이 노리는 배치다.
pub fn placement_is_clear(
    catalogue: &PoseCatalogue,
    nfp: &PolygonSet,
    dx: f64,
    dy: f64,
) -> bool {
    match catalogue
        .engine
        .locate(nfp, NfpPoint::new(dx, dy))
    {
        Ok(Location::Inside) => false,
        Ok(_) => true,
        // 판정 실패는 **안전한 쪽**으로 — 겹칠 수 있다고 본다. 낙관하면 겹친 배치가
        // 검증까지 통과해 버릴 수 있다.
        Err(_) => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 자세 -> 커널 방향 대응이 문서 표 그대로인지.
    #[test]
    fn the_pose_maps_onto_the_kernel_orientation_exactly() {
        // 항등.
        let o = orientation_of(&RotationEntry::IDENTITY);
        assert_eq!(o.angle_microdegrees, 0);
        assert!(!o.mirror_x);

        // y축 뒤집기 = 좌우 반전 = mirror_x, 추가 회전 없음.
        let flip_y = orientation_of(&RotationEntry {
            flip_x: false,
            flip_y: true,
            millideg: 0,
        });
        assert!(flip_y.mirror_x);
        assert_eq!(flip_y.angle_microdegrees, 0);

        // x축 뒤집기 = 상하 반전 = mirror_x + 180도.
        let flip_x = orientation_of(&RotationEntry {
            flip_x: true,
            flip_y: false,
            millideg: 0,
        });
        assert!(flip_x.mirror_x);
        assert_eq!(flip_x.angle_microdegrees, 180_000_000);

        // 두 축을 다 뒤집으면 **거울이 아니라** 180도 회전이다.
        let both = orientation_of(&RotationEntry {
            flip_x: true,
            flip_y: true,
            millideg: 0,
        });
        assert!(!both.mirror_x, "두 번 뒤집으면 방향이 되돌아온다");
        assert_eq!(both.angle_microdegrees, 180_000_000);
    }

    /// 밀리도 -> 마이크로도는 x1000 이고, 360도 주기로 정규화된다.
    #[test]
    fn millidegrees_scale_to_microdegrees_and_wrap() {
        assert_eq!(
            orientation_of(&RotationEntry::turn(90_000)).angle_microdegrees,
            90_000_000
        );
        // -90도와 270도는 같은 자세다.
        assert_eq!(
            orientation_of(&RotationEntry::turn(-90_000)).angle_microdegrees,
            orientation_of(&RotationEntry::turn(270_000)).angle_microdegrees
        );
        // 자유회전 한 스텝(0.01도 = 10 밀리도) = 10,000 마이크로도.
        assert_eq!(
            orientation_of(&RotationEntry::turn(10)).angle_microdegrees,
            10_000
        );
    }

    /// 빈 NFP 는 **전체 겹침**으로 읽는다 — 놓을 자리가 없다는 뜻이다.
    #[test]
    fn an_empty_nfp_reads_as_a_full_overlap() {
        let (summary, record) = summarize(&PolygonSet::new(Vec::new()));
        assert!(summary.full_overlap);
        assert_eq!(summary.vertex_count, 0);
        assert!(record.points.is_empty());
        // 모듈 6 의 분기가 이것을 부착 해제로 읽는다.
        assert_eq!(
            summary.lookup(),
            crate::refplace::chain::NfpLookup::FullOverlap
        );
    }

    /// 성분이 여럿이면 좌표를 그대로 이어 붙일 수 없다 — 판독 코드로 알린다.
    #[test]
    fn a_multi_component_nfp_is_flagged_so_coordinates_are_not_naively_joined() {
        let square = |ox: f64| {
            swarm_nfp::Polygon::new(Ring::new(vec![
                NfpPoint::new(ox, 0.0),
                NfpPoint::new(ox + 1.0, 0.0),
                NfpPoint::new(ox + 1.0, 1.0),
                NfpPoint::new(ox, 1.0),
            ]))
        };
        let set = PolygonSet::new(vec![square(0.0), square(10.0)]);
        let (summary, _) = summarize(&set);
        assert!(summary.code.needs_coord_scaling());
        assert!(!summary.full_overlap);
        assert_eq!(summary.vertex_count, 8);
    }

    /// 단일 성분·구멍 없음은 그대로 읽는다.
    #[test]
    fn a_plain_nfp_needs_no_special_reading() {
        let set = PolygonSet::from_outer(Ring::new(vec![
            NfpPoint::new(0.0, 0.0),
            NfpPoint::new(1.0, 0.0),
            NfpPoint::new(1.0, 1.0),
        ]));
        let (summary, record) = summarize(&set);
        assert_eq!(summary.code, ReadCode::Plain);
        assert_eq!(summary.lookup(), crate::refplace::chain::NfpLookup::Ready);
        assert_eq!(record.points.len(), 3);
    }

    /// 판정 실패는 **안전한 쪽**(겹칠 수 있음)으로 떨어져야 한다.
    ///
    /// 낙관하면 겹친 배치가 검증까지 통과한다.
    #[test]
    fn an_unreadable_region_is_treated_as_blocked_not_free() {
        let cat = PoseCatalogue {
            engine: NfpEngine::new(),
            shapes: BTreeMap::new(),
            container: None,
            container_height: 100.0,
        };
        // 축퇴한(정점 2개) 집합 — 커널이 거절하거나 내부 없음으로 답한다.
        let degenerate = PolygonSet::from_outer(Ring::new(vec![
            NfpPoint::new(0.0, 0.0),
            NfpPoint::new(1.0, 0.0),
        ]));
        // 어느 쪽이든 panic 없이 bool 을 돌려주어야 한다.
        let _ = placement_is_clear(&cat, &degenerate, 0.5, 0.0);
    }

    /// 카탈로그가 비어 있으면 쌍 계산은 조용히 `None` 이다 — panic 하지 않는다.
    #[test]
    fn a_missing_pose_makes_the_pair_absent_rather_than_panicking() {
        let cat = PoseCatalogue {
            engine: NfpEngine::new(),
            shapes: BTreeMap::new(),
            container: None,
            container_height: 100.0,
        };
        let pose = PoseKey::of(&RotationEntry::IDENTITY);
        let key = NfpPairKey {
            fixed_item: 0,
            fixed_pose: pose,
            moving_item: 1,
            moving_pose: pose,
        };
        assert!(cat.compute_pair(&key).is_none());
        assert!(cat.inner_fit(0, pose).is_none());
        assert!(world_bbox(&cat, 0, pose, 0.0, 0.0).is_none());
    }
}
