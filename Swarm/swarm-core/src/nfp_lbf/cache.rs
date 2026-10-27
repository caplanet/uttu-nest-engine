//! NFP Cache — **Geometry Hash 기준** 캐싱.
//!
//! 캐시 Key를 이렇게 잡는다.
//!
//! ```text
//! NfpKey {
//!     shape_a_hash,
//!     rotation_a,
//!     shape_b_hash,
//!     rotation_b,
//! }
//! ```
//!
//! 패턴 instance 번호는 키에 넣으면 안 된다.
//!
//! ```text
//! 잘못된 방식
//! Pattern_001 × Pattern_020
//! Pattern_001 × Pattern_021
//! Pattern_001 × Pattern_022
//! ```
//!
//! 동일한 형상을 계속 계산하게 된다. 대신:
//!
//! ```text
//! GeometryHash(A) × GeometryHash(B)
//! ```
//!
//! 로 캐싱해서 **동일 패턴의 Size/Shape가 같으면 NFP 하나를 모든 instance가 공유**하게 한다.
//!
//! ```text
//! 실제 배치 Polygon = 156개
//! 하지만 고유 Geometry가 예: 35개라면
//! 156C2 = 12,090 pair → Geometry Hash 기준 캐싱 → 35C2 + self NFP ≈ 630개 수준
//! ```
//!
//! 성능 목표:
//!
//! ```text
//! JSON Load / Parse                 < 100 ms
//! Geometry Normalize / Hash         < 100 ms
//! Unique NFP 계산
//!  ├─ 단일 thread                 1~5 sec
//!  └─ Rayon 8~16 thread         0.3~1.5 sec
//! Cache serialization              < 500 ms
//! -----------------------------------------
//! 전체 Cold Cache 생성             < 2 sec 목표
//! 최악 조건                        < 5 sec
//! ```
//!
//! ```text
//! 156 polygons → 12,090 unique pairs → Rayon parallel iterator
//! → 8 threads 기준 약 1,511 NFP / thread → Cache 저장
//! ```
//!
//! Cache Miss 경로:
//!
//! ```text
//! Nesting 시작 → 필요한 NFP 요청 → Cache Hit ─→ 즉시 사용
//!                                   │
//!                                 Cache Miss → NFP 계산 → Cache 저장
//! ```
//!
//! > **캐시 저장을 연산 중 mutex로 매번 잠그지 않고, thread-local 결과를 모아서 마지막에
//! > 한 번 합치는 구조**가 유리하다.
//!
//! 이 파일의 [`NfpLbfCache::prewarm`]이 정확히 그 구조다. rayon 워커는 `Mutex`를 전혀
//! 잡지 않고 `(NfpKey, PolygonSet)`을 반환만 하며, 병합은 단일 스레드에서 한 번 일어난다.

use std::collections::hash_map::DefaultHasher;
use std::collections::{BTreeMap, HashMap, HashSet};
use std::hash::{Hash, Hasher};
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering as AtomicOrdering};
use std::time::Duration;

use jagua_rs::Instant;
use jagua_rs::geometry::primitives::SPolygon;
use jagua_rs::probs::spp::entities::SPInstance;
use rayon::prelude::*;

use swarm_nfp::{
    Clearance, GRID_SCALE, Location, NfpEngine, NfpError, Orientation, Point as NfpPoint,
    PolygonSet, PreparedRegion, PreparedShape, Ring,
};

// ────────────────────────────────────────────────────────────────────────────
// 양자화 유틸 — swarm-nfp의 0.001mm 정수 격자와 동일한 격자를 쓴다.
// ────────────────────────────────────────────────────────────────────────────

/// `mm` 값을 `GRID_SCALE`(= 1000, 즉 0.001mm) 격자로 양자화한다.
///
/// `f32`는 `Hash`를 구현하지 않으므로 해시 대상은 **반드시** 이 함수를 거쳐 정수가 된다.
#[inline]
fn quantize_mm(value: f32) -> i64 {
    quantize_mm_f64(value as f64)
}

/// [`quantize_mm`]의 `f64` 판.
#[inline]
fn quantize_mm_f64(value: f64) -> i64 {
    (value * GRID_SCALE as f64).round() as i64
}

/// `n`개 원소에서 self pair를 포함한 정규 pair 개수 `n(n+1)/2`.
#[inline]
fn triangular_pair_count(n: usize) -> usize {
    n.saturating_mul(n.saturating_add(1)) / 2
}

/// `PolygonSet`을 원점 대칭(부호 반전)한 새 집합을 만든다.
///
/// `NFP(A, B) = -NFP(B, A)`이므로 정규 키 하나만 저장하고 반대 방향은 이 변환으로 얻는다.
/// 각 ring의 정점 순서는 뒤집어 winding 규약을 유지한다.
fn negate_polygon_set(set: &PolygonSet) -> PolygonSet {
    let components = set
        .components
        .iter()
        .map(|component| swarm_nfp::Polygon {
            outer: negate_ring(&component.outer),
            holes: component.holes.iter().map(negate_ring).collect(),
        })
        .collect();
    PolygonSet::new(components)
}

/// [`negate_polygon_set`]의 ring 단위 구현.
fn negate_ring(ring: &Ring) -> Ring {
    Ring {
        vertices: ring
            .vertices
            .iter()
            .rev()
            .map(|p| NfpPoint::new(-p.x, -p.y))
            .collect(),
    }
}

/// `SPolygon`(jagua-rs, f32 mm)을 `PolygonSet`(swarm-nfp, f64 mm)으로 옮긴다.
///
/// `SPolygon`은 hole이 없는 simple polygon이므로 항상 단일 outer ring이 된다.
fn polygon_set_of(poly: &SPolygon) -> PolygonSet {
    let ring = Ring::new(
        poly.vertices()
            .iter()
            .map(|p| NfpPoint::new(p.0 as f64, p.1 as f64)),
    );
    PolygonSet::from_outer(ring)
}

/// 오목 단순화를 적용한 뒤 [`PolygonSet`]으로 변환한다.
fn polygon_set_of_simplified(poly: &SPolygon, concave_tolerance: f32) -> PolygonSet {
    let simplified = simplify_concave_only(&poly.vertices(), poly.area(), concave_tolerance);
    if simplified.len() == poly.n_vertices() {
        // 단순화가 한 정점도 못 지웠다 — 원본 경로를 그대로 쓴다.
        return polygon_set_of(poly);
    }
    let ring = Ring::new(
        simplified
            .iter()
            .map(|p| NfpPoint::new(p.0 as f64, p.1 as f64)),
    );
    PolygonSet::from_outer(ring)
}

/// 오목(reflex) 정점만 제거하는 단순화. NFP 라이브러리 전용이다.
///
/// # 왜 오목만인가
///
/// 볼록 분해 조각 수가 pair NFP의 원시 조각 수를 `조각(A) × 조각(B)`로 결정하는데,
/// 그 조각 수는 **오목 정점 수 + 1**을 따라간다. 실측(여성WSjk 156-패널 마커):
///
/// ```text
/// tolerance 0.001 : 평균 정점 35.8 / 오목 17.6 / 조각 16.8
///                   평균오차 |조각 - (오목+1)| = 1.85   vs  |조각 - 정점| = 18.97
/// tolerance 0.01  : 평균 정점 14.3 / 오목  5.4 / 조각  5.5
///                   평균오차 |조각 - (오목+1)| = 0.96   vs  |조각 - 정점| =  8.81
/// ```
///
/// 볼록 정점은 조각 수를 늘리지 않으므로 NFP 비용을 줄이는 레버는 오목 정점뿐이다.
/// 전역 `poly_simpl_tolerance`를 올리면 볼록 정점까지 함께 뭉개져 형상 충실도만 잃는다.
/// 그래서 강도를 분리한다 — 볼록은 솔버 설정 그대로 두고, 오목만 여기서 더 줄인다.
///
/// # 왜 안전한가
///
/// 오목 정점을 제거하면 만입부가 채워져 도형이 **커진다**. 결과는 원본의 상위집합이므로
/// 이 도형으로 만든 NFP는 실제 금지영역을 과대평가한다. 따라서 NFP가 "배치 가능"이라고
/// 한 위치는 jagua-rs CDE 기준으로도 반드시 충돌이 없다. 밀도만 조금 손해 보고
/// 정합성은 유지된다 (반대로 볼록 정점을 깎으면 도형이 작아져 실제 겹침을 놓친다).
///
/// # 절차
///
/// 면적 증가가 가장 작은 유효한 오목 정점부터 제거하고, 누적 증가가
/// `max_area_growth_ratio`를 넘으면 멈춘다. 새 변이 다른 변과 교차하거나 채워질 삼각형
/// 안에 다른 정점이 들어가면 그 후보는 무효다(단순 다각형이 깨진다).
pub(crate) fn simplify_concave_only(
    vertices: &[jagua_rs::geometry::primitives::Point],
    original_area: f32,
    max_area_growth_ratio: f32,
) -> Vec<jagua_rs::geometry::primitives::Point> {
    use jagua_rs::geometry::primitives::Point;

    let mut points: Vec<Point> = vertices.to_vec();
    if !(max_area_growth_ratio > 0.0) || points.len() < 4 || !(original_area > 0.0) {
        return points;
    }
    let budget = original_area * max_area_growth_ratio;
    let mut spent = 0.0f32;

    // 최악의 경우에도 정점 수만큼만 돈다.
    for _ in 0..vertices.len() {
        if points.len() < 4 {
            break;
        }
        let n = points.len();

        let mut best: Option<(usize, f32)> = None;
        for i in 0..n {
            let prev = points[(i + n - 1) % n];
            let cur = points[i];
            let next = points[(i + 1) % n];

            // SPolygon은 CCW(양의 면적)로 정규화되어 있다. cross < 0 이면 오목.
            if cross(prev, cur, next) >= 0.0 {
                continue;
            }
            let delta = triangle_area(prev, cur, next);
            if !delta.is_finite() || spent + delta > budget {
                continue;
            }
            if !concave_removal_is_valid(&points, i) {
                continue;
            }
            if best.is_none_or(|(_, best_delta)| delta < best_delta) {
                best = Some((i, delta));
            }
        }

        let Some((index, delta)) = best else {
            break;
        };
        points.remove(index);
        spent += delta;
    }

    points
}

/// 오목 정점 `i` 제거가 단순 다각형을 유지하는지 검사한다.
fn concave_removal_is_valid(points: &[jagua_rs::geometry::primitives::Point], i: usize) -> bool {
    let n = points.len();
    let prev_idx = (i + n - 1) % n;
    let next_idx = (i + 1) % n;
    let (prev, cur, next) = (points[prev_idx], points[i], points[next_idx]);

    for j in 0..n {
        // 채워질 삼각형에 인접한 정점은 제외한다.
        if j == prev_idx || j == i || j == next_idx {
            continue;
        }
        // 다른 정점이 채워질 삼각형 안에 들어오면 안 된다.
        if point_in_triangle(points[j], prev, cur, next) {
            return false;
        }
        // 새 변 prev -> next 가 다른 변과 교차하면 안 된다.
        let k = (j + 1) % n;
        if k == prev_idx || j == next_idx {
            continue;
        }
        if segments_properly_intersect(prev, next, points[j], points[k]) {
            return false;
        }
    }
    true
}

#[inline]
fn cross(
    a: jagua_rs::geometry::primitives::Point,
    b: jagua_rs::geometry::primitives::Point,
    c: jagua_rs::geometry::primitives::Point,
) -> f32 {
    (b.0 - a.0) * (c.1 - b.1) - (b.1 - a.1) * (c.0 - b.0)
}

#[inline]
fn triangle_area(
    a: jagua_rs::geometry::primitives::Point,
    b: jagua_rs::geometry::primitives::Point,
    c: jagua_rs::geometry::primitives::Point,
) -> f32 {
    ((b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)).abs() * 0.5
}

fn point_in_triangle(
    p: jagua_rs::geometry::primitives::Point,
    a: jagua_rs::geometry::primitives::Point,
    b: jagua_rs::geometry::primitives::Point,
    c: jagua_rs::geometry::primitives::Point,
) -> bool {
    let d1 = cross(a, b, p);
    let d2 = cross(b, c, p);
    let d3 = cross(c, a, p);
    let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    !(has_neg && has_pos)
}

/// 두 선분이 **진짜로** 교차하는지 (끝점 접촉은 교차로 보지 않는다).
fn segments_properly_intersect(
    p1: jagua_rs::geometry::primitives::Point,
    p2: jagua_rs::geometry::primitives::Point,
    q1: jagua_rs::geometry::primitives::Point,
    q2: jagua_rs::geometry::primitives::Point,
) -> bool {
    let d1 = orient2d(q1, q2, p1);
    let d2 = orient2d(q1, q2, p2);
    let d3 = orient2d(p1, p2, q1);
    let d4 = orient2d(p1, p2, q2);
    ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
}

#[inline]
fn orient2d(
    a: jagua_rs::geometry::primitives::Point,
    b: jagua_rs::geometry::primitives::Point,
    c: jagua_rs::geometry::primitives::Point,
) -> f32 {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

// ────────────────────────────────────────────────────────────────────────────
// GeometryHash — 형상 등가 클래스
// ────────────────────────────────────────────────────────────────────────────

/// 형상 등가 클래스 해시. **인스턴스 번호가 아니라 기하가 같으면 같은 값**이 나온다.
///
/// 정규화 절차:
///
/// ```text
/// 1) centroid 기준 상대 좌표로 이동  → 평행이동 불변
/// 2) 0.001mm 격자로 양자화           → f32 오차 흡수 + Hash 가능한 정수화
/// 3) 사전순 최소 정점을 시작점으로 순환 회전 → 정점 나열 시작점 불변
/// 4) 정점 수 + 양자화 면적 + 정점 시퀀스를 순서대로 해시
/// ```
///
/// centroid와 면적은 `SPolygon::centroid()` / `SPolygon::area`를 그대로 쓰지 않는다.
/// 두 값 모두 **절대 좌표** 위에서 `f32` shoelace로 누적되므로, 같은 형상이라도 배치
/// 위치가 멀어지면 상쇄 오차가 격자 한 칸(0.001mm)을 훌쩍 넘겨 해시가 갈라진다.
/// 대신 첫 정점을 기준으로 상대화한 뒤 `f64`로 centroid를 구하고, 면적은 **양자화된
/// 정수 좌표 위에서 정확한 정수 shoelace**로 다시 계산한다.
#[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Debug)]
pub struct GeometryHash(pub u64);

impl GeometryHash {
    /// 다각형의 형상 등가 클래스 해시를 계산한다.
    pub fn of_polygon(poly: &SPolygon) -> Self {
        let n = poly.n_vertices();
        debug_assert!(n >= 3, "SPolygon은 항상 3개 이상의 정점을 가진다");

        // 수치 안정성: 첫 정점을 원점으로 옮긴 뒤 f64로 계산한다.
        let ox = poly.vertices()[0].0 as f64;
        let oy = poly.vertices()[0].1 as f64;
        let rel: Vec<(f64, f64)> = poly
            .vertices()
            .iter()
            .map(|v| (v.0 as f64 - ox, v.1 as f64 - oy))
            .collect();

        // 1) centroid (shoelace) — cross_sum = 2A, 6A = 3 × cross_sum
        let mut cross_sum = 0.0f64;
        let mut cx = 0.0f64;
        let mut cy = 0.0f64;
        for i in 0..n {
            let (xi, yi) = rel[i];
            let (xj, yj) = rel[(i + 1) % n];
            let cross = xi * yj - xj * yi;
            cross_sum += cross;
            cx += (xi + xj) * cross;
            cy += (yi + yj) * cross;
        }
        let (cx, cy) = if cross_sum.abs() > 1.0e-12 {
            (cx / (3.0 * cross_sum), cy / (3.0 * cross_sum))
        } else {
            // 면적이 0에 수렴하는 퇴화 도형 방어 — 정점 평균으로 대체한다.
            let inv_n = 1.0 / n as f64;
            (
                rel.iter().map(|p| p.0).sum::<f64>() * inv_n,
                rel.iter().map(|p| p.1).sum::<f64>() * inv_n,
            )
        };

        // 2) centroid 상대 좌표 → 0.001mm 격자 양자화
        let quantized: Vec<(i64, i64)> = rel
            .iter()
            .map(|(x, y)| (quantize_mm_f64(x - cx), quantize_mm_f64(y - cy)))
            .collect();

        // 3) 사전순 최소 정점을 시작점으로 삼는다(동률이면 가장 앞선 인덱스).
        let mut start = 0usize;
        for i in 1..n {
            if quantized[i] < quantized[start] {
                start = i;
            }
        }

        // 4) 정점 수 → 양자화 면적 → 정점 시퀀스 순서로 해시
        let mut hasher = DefaultHasher::new();
        n.hash(&mut hasher);
        quantized_area2(&quantized).hash(&mut hasher);
        for k in 0..n {
            let (x, y) = quantized[(start + k) % n];
            x.hash(&mut hasher);
            y.hash(&mut hasher);
        }

        GeometryHash(hasher.finish())
    }
}

/// 양자화된 정수 좌표 위에서 정확히 계산한 `2A` (격자 단위²). 부호는 버린다.
fn quantized_area2(vertices: &[(i64, i64)]) -> i128 {
    let n = vertices.len();
    let mut area2: i128 = 0;
    for i in 0..n {
        let (xi, yi) = vertices[i];
        let (xj, yj) = vertices[(i + 1) % n];
        area2 += (xi as i128) * (yj as i128) - (xj as i128) * (yi as i128);
    }
    area2.abs()
}

// ────────────────────────────────────────────────────────────────────────────
// NfpMask — NFP 로컬 좌표계 저해상도 점유 마스크
// ────────────────────────────────────────────────────────────────────────────

/// 마스크 셀 하나의 상태.
///
/// [`MaskCell::Inside`] / [`MaskCell::Outside`]는 **셀 전체**가 그 상태임이 보장될 때만
/// 붙는다. 조금이라도 애매하면 [`MaskCell::Boundary`]이며, 그 경우 판정 주체는 여전히
/// `NfpEngine::locate`다. 마스크는 단축 경로일 뿐이다.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum MaskCell {
    /// 셀 전체가 NFP 밖.
    Outside,
    /// 셀 전체가 NFP 안.
    Inside,
    /// 경계가 지나갈 수 있음 — `locate`로 정확히 판정해야 한다.
    Boundary,
}

const MASK_OUTSIDE: u8 = 0;
const MASK_INSIDE: u8 = 1;
const MASK_BOUNDARY: u8 = 2;

/// 긴 축에 놓을 목표 셀 수.
///
/// 경계 셀 비율은 대략 `축_셀수`에 반비례한다(경계 래스터 1겹 + 팽창 1겹).
/// 여성WSjk 156-패널 실측 — 축 128이면 bbox 히트 1,044,434건 중 157,065건(15%)이
/// 경계 셀로 떨어져 `locate`를 탄다.
///
/// # 축 256 은 **되돌려진 실험**이다
///
/// 예전 주석은 "축을 256으로 올리면 그 절반이 사라지고, `locate` 한 번이 30µs대라
/// 그 절반이 곧 수 초다" 라고 적어 사실상 256 을 권했다. 두 근거가 모두 낡았다:
///
/// * `locate` 비용 30µs 는 `PreparedRegion` / `locate_prepared` 도입 **이전** 값이다.
///   지금은 **0.26µs** 다(140배). 경계 히트를 절반 줄여도 아끼는 시간이 거의 없다.
/// * 실제로 256 으로 올려 재 봤더니 `locate` 호출이 8% 밖에 안 줄었는데 마스크 생성
///   비용이 커져 **전체가 7.7초 -> 9.4초로 악화**됐다. 남은 경계 히트는 해상도
///   아티팩트가 아니라 **실제로 다른 NFP 경계 위에 있는 접촉점**이라 해상도로 없앨 수
///   없기 때문이다. 그래서 되돌렸다.
///
/// 128 을 바꾸려면 위 두 수치를 다시 뒤집는 측정을 먼저 보여야 한다.
const MASK_TARGET_CELLS_PER_AXIS: usize = 128;

/// 마스크 셀 총수 상한.
///
/// 셀당 2비트이므로 `32,768 셀 x 2비트 = 8 KB/NFP` 다. (예전 주석은 32KB 라고 적었는데
/// 셀당 1바이트로 계산한 값이라 **4배 틀렸다.** 그 계산은 되돌려진 축 256 규모를 기준으로
/// 쓰인 것이기도 하다.)
const MASK_MAX_CELLS: usize = 32_768;

/// 셀 한 변의 하한(mm).
///
/// 격자 양자화(0.001mm)로 정점과 질의점이 최대 0.0005mm 움직이므로, 셀이 그보다
/// 훨씬 커야 "팽창 1겹" 여유가 양자화 오차를 확실히 덮는다.
const MASK_MIN_CELL_MM: f64 = 0.01;

/// NFP **로컬 좌표계**의 저해상도 점유 마스크.
///
/// # 왜 필요한가
///
/// `NfpEngine::locate`는 호출마다 `PolygonSet`을 정규 경로로 다시 만들고 링을 전부
/// 훑는다. 탐침 하나가 bbox 히트 obstacle 수만큼 이걸 반복하므로 배치 비용이
/// `O(탐침 × 히트 × 정점)`이 된다 (여성WSjk 156-패널에서 ordering 1회 5.4초의 지배항).
///
/// # 어떻게 정확한가
///
/// ```text
/// 1) 모든 ring edge가 지나가는 셀을 Boundary로 칠한다 (열/행 클리핑 = 정확한 supercover).
/// 2) Boundary를 8-이웃으로 한 겹 팽창시킨다 → 양자화 오차(0.0005mm) 흡수.
/// 3) 남은 셀을 4-연결 성분으로 묶는다. 한 성분 안의 두 이웃 셀은 경계가 지나가지
///    않는 연결집합을 이루므로 **위치가 같다**. 성분마다 대표점 하나만 `locate`로
///    확정하고 그 값을 성분 전체에 채운다.
/// ```
///
/// 3)에서 구멍(hole) 내부는 바깥과 연결되지 않은 별도 성분이 되고, 대표점 `locate`가
/// `Outside`를 돌려주므로 "바깥에서 flood fill"류의 오판이 원천적으로 없다.
///
/// 로컬 좌표라 평행이동이 달라도 재사용된다 — NFP 하나당 딱 한 번만 만든다.
#[derive(Clone, Debug)]
pub struct NfpMask {
    /// 로컬 bbox `(min_x, min_y, max_x, max_y)`. 정점이 없으면 `None`.
    bounds: Option<(f64, f64, f64, f64)>,
    /// 격자 `(0, 0)` 셀의 좌하단(로컬 mm).
    origin_x: f64,
    origin_y: f64,
    /// 셀 한 변(mm)의 역수.
    inv_cell: f64,
    cols: usize,
    rows: usize,
    /// `rows × cols` 셀 상태를 **셀당 2비트**로 눌러 담은 것.
    /// 비어 있으면 마스크 비활성(전부 `Boundary` 취급).
    ///
    /// 축 256 해상도면 셀이 65,536개까지 가는데, 1바이트씩 두면 NFP 하나에 64KB고
    /// 캐시 전체로는 수십 MB가 된다. 2비트면 16KB로 줄어 L2에도 잘 들어간다.
    cells: Vec<u8>,
}

impl NfpMask {
    /// 로컬 좌표 `set`의 점유 마스크를 만든다. 실패해도 panic하지 않고 "전부 Boundary"
    /// (= 기존 경로 그대로)로 물러난다.
    pub fn build(engine: &NfpEngine, set: &PolygonSet) -> Self {
        let bounds = mask_bounds(set);
        let Some((min_x, min_y, max_x, max_y)) = bounds else {
            // 정점이 없다 = 빈 집합. `locate`도 항상 Outside를 돌려준다.
            return Self::disabled(None);
        };

        let w = max_x - min_x;
        let h = max_y - min_y;
        let longest = w.max(h);
        if !longest.is_finite() || longest <= 0.0 {
            return Self::disabled(bounds);
        }

        // 셀 크기 — 긴 축 기준 목표 해상도, 하한과 총 셀수 상한을 함께 건다.
        let mut cell = (longest / MASK_TARGET_CELLS_PER_AXIS as f64).max(MASK_MIN_CELL_MM);
        let (mut cols, mut rows) = mask_dims(w, h, cell);
        // 상한 초과 시 셀을 두 배씩 키운다(최대 32회 — 실전에서는 0~1회).
        for _ in 0..32 {
            if cols.saturating_mul(rows) <= MASK_MAX_CELLS {
                break;
            }
            cell *= 2.0;
            let d = mask_dims(w, h, cell);
            cols = d.0;
            rows = d.1;
        }
        if cols.saturating_mul(rows) > MASK_MAX_CELLS || cols < 3 || rows < 3 {
            return Self::disabled(bounds);
        }

        // 격자는 bbox를 한 셀씩 넉넉히 감싼다 — 격자 밖은 무조건 bbox 밖 = Outside.
        let origin_x = min_x - cell;
        let origin_y = min_y - cell;
        let inv_cell = 1.0 / cell;

        // 작업 중에는 셀당 1바이트 스크래치를 쓰고, 마지막에 2비트로 눌러 담는다.
        let mut cells = vec![MASK_OUTSIDE; cols * rows];

        // 1) edge supercover 래스터화
        let mut any_edge = false;
        for component in set.components.iter() {
            for ring in std::iter::once(&component.outer).chain(component.holes.iter()) {
                let n = ring.vertices.len();
                if n < 2 {
                    continue;
                }
                for i in 0..n {
                    let a = ring.vertices[i];
                    let b = ring.vertices[(i + 1) % n];
                    if !(a.x.is_finite() && a.y.is_finite() && b.x.is_finite() && b.y.is_finite()) {
                        return Self::disabled(bounds);
                    }
                    any_edge = true;
                    mark_segment(
                        &mut cells, cols, rows, origin_x, origin_y, cell, inv_cell, a.x, a.y, b.x,
                        b.y,
                    );
                }
            }
        }
        if !any_edge {
            return Self::disabled(bounds);
        }

        // 2) 8-이웃 한 겹 팽창 — 양자화로 경계가 최대 0.0005mm 움직여도 덮인다.
        dilate_boundary(&mut cells, cols, rows);

        // 3) 비경계 셀을 4-연결 성분으로 묶고 성분마다 대표점 하나만 `locate`한다.
        let mut visited = vec![false; cells.len()];
        let mut stack: Vec<usize> = Vec::new();
        let mut component: Vec<usize> = Vec::new();
        for seed in 0..cells.len() {
            if visited[seed] || cells[seed] == MASK_BOUNDARY {
                continue;
            }
            component.clear();
            stack.clear();
            stack.push(seed);
            visited[seed] = true;
            while let Some(idx) = stack.pop() {
                component.push(idx);
                let (c, r) = (idx % cols, idx / cols);
                let visit = |n: usize, stack: &mut Vec<usize>, visited: &mut Vec<bool>| {
                    if !visited[n] && cells[n] != MASK_BOUNDARY {
                        visited[n] = true;
                        stack.push(n);
                    }
                };
                if c > 0 {
                    visit(idx - 1, &mut stack, &mut visited);
                }
                if c + 1 < cols {
                    visit(idx + 1, &mut stack, &mut visited);
                }
                if r > 0 {
                    visit(idx - cols, &mut stack, &mut visited);
                }
                if r + 1 < rows {
                    visit(idx + cols, &mut stack, &mut visited);
                }
            }

            // 대표 셀의 중심 하나로 성분 전체가 확정된다.
            let rep = component[0];
            let cx = origin_x + ((rep % cols) as f64 + 0.5) * cell;
            let cy = origin_y + ((rep / cols) as f64 + 0.5) * cell;
            let state = match engine.locate(set, NfpPoint::new(cx, cy)) {
                Ok(Location::Inside) => MASK_INSIDE,
                Ok(Location::Outside) => MASK_OUTSIDE,
                // 경계가 없는 셀의 중심이 Boundary일 수는 없지만, 나오면 보수적으로 넘긴다.
                _ => MASK_BOUNDARY,
            };
            for &idx in component.iter() {
                cells[idx] = state;
            }
        }

        Self {
            bounds,
            origin_x,
            origin_y,
            inv_cell,
            cols,
            rows,
            cells: pack_cells(&cells),
        }
    }

    /// 마스크를 쓰지 않는(= 항상 `locate`로 넘기는) 형태.
    fn disabled(bounds: Option<(f64, f64, f64, f64)>) -> Self {
        Self {
            bounds,
            origin_x: 0.0,
            origin_y: 0.0,
            inv_cell: 0.0,
            cols: 0,
            rows: 0,
            cells: Vec::new(),
        }
    }

    /// 로컬 bbox `(min_x, min_y, max_x, max_y)`. 정점이 하나도 없으면 `None`.
    #[inline]
    pub fn bounds(&self) -> Option<(f64, f64, f64, f64)> {
        self.bounds
    }

    /// 로컬 좌표 점 `(x, y)`가 속한 셀의 상태.
    #[inline]
    pub fn classify(&self, x: f64, y: f64) -> MaskCell {
        if self.bounds.is_none() {
            // 빈 집합 — `locate`도 항상 Outside다.
            return MaskCell::Outside;
        }
        if self.cells.is_empty() {
            return MaskCell::Boundary;
        }
        let fc = (x - self.origin_x) * self.inv_cell;
        let fr = (y - self.origin_y) * self.inv_cell;
        // 격자 밖 = bbox 밖(한 셀 여유가 있다) = 확실히 Outside.
        if !(fc >= 0.0) || !(fr >= 0.0) {
            return MaskCell::Outside;
        }
        let (c, r) = (fc as usize, fr as usize);
        if c >= self.cols || r >= self.rows {
            return MaskCell::Outside;
        }
        let idx = r * self.cols + c;
        match (self.cells[idx >> 2] >> ((idx & 3) << 1)) & 0b11 {
            MASK_OUTSIDE => MaskCell::Outside,
            MASK_INSIDE => MaskCell::Inside,
            _ => MaskCell::Boundary,
        }
    }

    /// 셀 격자 크기 `(cols, rows)`. 마스크가 비활성이면 `(0, 0)`.
    #[inline]
    pub fn dims(&self) -> (usize, usize) {
        (self.cols, self.rows)
    }
}

/// `set`의 모든 ring 정점을 감싸는 bbox.
fn mask_bounds(set: &PolygonSet) -> Option<(f64, f64, f64, f64)> {
    let mut acc: Option<(f64, f64, f64, f64)> = None;
    for component in set.components.iter() {
        for ring in std::iter::once(&component.outer).chain(component.holes.iter()) {
            for v in ring.vertices.iter() {
                if !v.x.is_finite() || !v.y.is_finite() {
                    return None;
                }
                acc = Some(match acc {
                    None => (v.x, v.y, v.x, v.y),
                    Some((lx, ly, hx, hy)) => (lx.min(v.x), ly.min(v.y), hx.max(v.x), hy.max(v.y)),
                });
            }
        }
    }
    acc
}

/// bbox `w × h`를 셀 `cell`로 덮을 때의 `(cols, rows)` — 양쪽에 한 셀씩 여유를 둔다.
fn mask_dims(w: f64, h: f64, cell: f64) -> (usize, usize) {
    let cols = ((w / cell).ceil().max(1.0) as usize).saturating_add(3);
    let rows = ((h / cell).ceil().max(1.0) as usize).saturating_add(3);
    (cols, rows)
}

/// 선분이 지나가는 모든 셀을 `Boundary`로 칠한다.
///
/// 열마다 선분을 클리핑해 그 열에서의 y 구간을 정확히 구하므로, DDA 근사와 달리
/// 실제로 지나가는 셀을 하나도 빠뜨리지 않는다.
#[allow(clippy::too_many_arguments)]
fn mark_segment(
    cells: &mut [u8],
    cols: usize,
    rows: usize,
    origin_x: f64,
    origin_y: f64,
    cell: f64,
    inv_cell: f64,
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
) {
    let (x_lo, x_hi) = if ax <= bx { (ax, bx) } else { (bx, ax) };
    let c0 = grid_index(x_lo, origin_x, inv_cell, cols);
    let c1 = grid_index(x_hi, origin_x, inv_cell, cols);
    let dx = bx - ax;

    for c in c0..=c1 {
        let cell_lo = origin_x + c as f64 * cell;
        let cell_hi = cell_lo + cell;
        // 열 경계로 클리핑하되 **선분 밖으로 나가지 않게** 다시 조인다.
        //
        // 열 인덱스는 `(v - origin) * inv_cell`로, 열 경계는 `origin + c * cell`로
        // 계산하므로 둘이 f64에서 정확히 맞아떨어지지 않는다. 예전에는 `lo > hi`면
        // 그 열을 건너뛰었는데, `x = max_x`에 걸친 수직 변이 통째로 누락되어 경계에
        // 구멍이 뚫렸다(albano NFP에서 실측 — 내부가 외부와 연결되어 전부 Outside로
        // 오판). 건너뛰지 않고 선분 구간 안으로 조이면 그런 누락이 없다.
        let lo = x_lo.max(cell_lo).min(x_hi);
        let hi = x_hi.min(cell_hi).max(x_lo);
        let (lo, hi) = if lo <= hi { (lo, hi) } else { (hi, lo) };
        let (y_lo, y_hi) = if dx.abs() <= f64::MIN_POSITIVE {
            if ay <= by { (ay, by) } else { (by, ay) }
        } else {
            let t0 = (lo - ax) / dx;
            let t1 = (hi - ax) / dx;
            let e0 = ay + t0.clamp(0.0, 1.0) * (by - ay);
            let e1 = ay + t1.clamp(0.0, 1.0) * (by - ay);
            if e0 <= e1 { (e0, e1) } else { (e1, e0) }
        };
        let r0 = grid_index(y_lo, origin_y, inv_cell, rows);
        let r1 = grid_index(y_hi, origin_y, inv_cell, rows);
        for r in r0..=r1 {
            cells[r * cols + c] = MASK_BOUNDARY;
        }
    }
}

/// 좌표를 셀 인덱스로. 격자 밖은 가장자리로 클램프한다(정점은 항상 bbox 안이다).
#[inline]
fn grid_index(v: f64, origin: f64, inv_cell: f64, count: usize) -> usize {
    let f = (v - origin) * inv_cell;
    if !(f >= 0.0) {
        return 0;
    }
    let i = f as usize;
    if i >= count { count - 1 } else { i }
}

/// `Boundary` 셀을 8-이웃으로 한 겹 넓힌다.
///
/// 3×3 Chebyshev 팽창은 가로 1회 + 세로 1회로 분리된다. 셀 수가 6만을 넘어가면
/// 9-이웃 직접 검사와 분리형의 차이가 그대로 pre-warm 시간으로 나온다.
fn dilate_boundary(cells: &mut [u8], cols: usize, rows: usize) {
    let src = cells.to_vec();
    // 가로 방향
    for r in 0..rows {
        let base = r * cols;
        for c in 0..cols {
            if cells[base + c] == MASK_BOUNDARY {
                continue;
            }
            let lo = c.saturating_sub(1);
            let hi = (c + 1).min(cols - 1);
            if src[base + lo..=base + hi]
                .iter()
                .any(|v| *v == MASK_BOUNDARY)
            {
                cells[base + c] = MASK_BOUNDARY;
            }
        }
    }
    // 세로 방향 — 가로 결과를 원본으로 삼는다.
    let mid = cells.to_vec();
    for r in 0..rows {
        let r_lo = r.saturating_sub(1);
        let r_hi = (r + 1).min(rows - 1);
        for c in 0..cols {
            if cells[r * cols + c] == MASK_BOUNDARY {
                continue;
            }
            if (r_lo..=r_hi).any(|rr| mid[rr * cols + c] == MASK_BOUNDARY) {
                cells[r * cols + c] = MASK_BOUNDARY;
            }
        }
    }
}

/// 셀당 1바이트 스크래치를 셀당 2비트로 눌러 담는다.
fn pack_cells(cells: &[u8]) -> Vec<u8> {
    let mut packed = vec![0u8; cells.len().div_ceil(4)];
    for (i, state) in cells.iter().enumerate() {
        packed[i >> 2] |= (*state & 0b11) << ((i & 3) << 1);
    }
    packed
}

// ────────────────────────────────────────────────────────────────────────────
// NfpKey
// ────────────────────────────────────────────────────────────────────────────

/// NFP 캐시 키. 패턴 instance 번호는 **절대** 들어가지 않는다.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct NfpKey {
    /// 고정(fixed) 형상의 등가 클래스 해시
    pub shape_a_hash: GeometryHash,
    /// 고정 형상의 회전각(도)
    pub rotation_a: i64,
    /// 이동(moving) 형상의 등가 클래스 해시
    pub shape_b_hash: GeometryHash,
    /// 이동 형상의 회전각(도)
    pub rotation_b: i64,
}

impl NfpKey {
    /// 키를 그대로 만든다.
    pub fn new(
        shape_a_hash: GeometryHash,
        rotation_a: i64,
        shape_b_hash: GeometryHash,
        rotation_b: i64,
    ) -> Self {
        Self {
            shape_a_hash,
            rotation_a,
            shape_b_hash,
            rotation_b,
        }
    }

    /// 고정/이동 형상을 맞바꾼 키.
    pub fn swapped(self) -> Self {
        Self {
            shape_a_hash: self.shape_b_hash,
            rotation_a: self.rotation_b,
            shape_b_hash: self.shape_a_hash,
            rotation_b: self.rotation_a,
        }
    }

    /// 정규 키와 부호 반전 필요 여부를 돌려준다.
    ///
    /// `(shape_a_hash, rotation_a) <= (shape_b_hash, rotation_b)`이면 그대로,
    /// 아니면 뒤집은 키를 쓴다. `true`는 저장된 NFP를 **부호 반전**해서 써야 함을
    /// 뜻한다(`NFP(A, B) = -NFP(B, A)`).
    pub fn canonical(self) -> (Self, bool) {
        if (self.shape_a_hash, self.rotation_a) <= (self.shape_b_hash, self.rotation_b) {
            (self, false)
        } else {
            (self.swapped(), true)
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 통계
// ────────────────────────────────────────────────────────────────────────────

/// 캐시 계측치. 문서의 성능 목표(cold cache `< 2 sec`, 최악 `< 5 sec`)를 확인하는 데 쓴다.
#[derive(Clone, Copy, Debug, Default)]
pub struct NfpCacheStats {
    /// 캐시 적중 횟수
    pub hits: u64,
    /// 캐시 실패 횟수
    pub misses: u64,
    /// 현재 저장된 정규 pair NFP 개수
    pub entries: usize,
    /// 고유 Geometry 개수 (문서 예시의 `35`)
    pub unique_geometries: usize,
    /// pre-warm으로 채운 pair 개수
    pub prewarm_pairs: usize,
    /// 마지막 pre-warm 소요 시간(ms)
    pub build_ms: u128,
    /// 인스턴스가 실제로 배치해야 하는 폴리곤 총 개수 (문서 예시의 `156`)
    pub instance_polygons: usize,
}

// ────────────────────────────────────────────────────────────────────────────
// PreparedLibrary
// ────────────────────────────────────────────────────────────────────────────

/// `(GeometryHash, rotation)` → [`PreparedShape`] 라이브러리.
///
/// 인스턴스의 모든 아이템을 고유 형상으로 접은 뒤, 허용 회전각마다 방향별 사본을 하나씩
/// 준비해 둔다. 이후 모든 NFP 계산은 이 사본만 참조한다.
#[derive(Clone)]
pub struct PreparedLibrary {
    /// `(hash, rotation)` → 준비된 형상
    shapes: HashMap<(GeometryHash, i64), PreparedShape>,
    /// item_id → 형상 해시 (준비에 성공한 형상만)
    item_hashes: HashMap<usize, GeometryHash>,
    /// 정렬·중복 제거된 회전각 목록
    rotations: Vec<i64>,
    /// `(hash, rotation)` 조합의 결정적 정렬 목록
    combos: Vec<(GeometryHash, i64)>,
    /// 준비에 성공한 고유 형상 수
    unique_geometries: usize,
    /// 인스턴스가 배치해야 하는 폴리곤 총 개수 (수량 합)
    instance_polygons: usize,
    /// importer 가 컨테이너에 적용한 **물리적** strip 인셋(mm).
    ///
    /// 겹침을 허용하면 `util::io::apply_overlap_strip_offset` 이 strip 의
    /// `shape_modify_config.offset` 을 `overlap/2` 로 설정하고, jagua 의
    /// `Strip -> Container` 변환은 `Deflate` 모드라 실제 컨테이너가
    /// `[t, W-t] x [t, H-t]` 로 줄어든다. 구성형 배치기는 원단 치수를
    /// `base_strip.fixed_height` 같은 **줄어들지 않은** 값에서 가져오므로 이
    /// 인셋을 따로 알려 주지 않으면 첫 조각을 컨테이너 밖에 놓는다.
    ///
    /// NFP 반올림 여유(`ifp_inset`)와는 독립이다 — 저쪽은 f32 경계 오차를 덮는
    /// 수치 여유이고 이쪽은 실제 원단이 좁아진 양이다.
    strip_inset: f32,
}

impl PreparedLibrary {
    /// 인스턴스에서 라이브러리를 만든다.
    ///
    /// `rotations`가 비어 있으면 `&[0]`으로 간주한다. 준비에 실패한 형상은 조용히 버리지
    /// 않고 `log::warn!`을 남긴 뒤 제외하며, 모든 형상이 실패하면 `None`을 반환한다.
    /// `concave_simplify_tolerance`는 NFP 라이브러리 **전용** 오목 단순화 강도다
    /// (원본 면적 대비 허용 증가 비율). `0.0` 이하면 단순화하지 않는다.
    /// [`simplify_concave_only`]의 문서를 참조.
    pub fn build(
        instance: &SPInstance,
        rotations: &[i64],
        concave_simplify_tolerance: f32,
    ) -> Option<Self> {
        Self::build_with_control(instance, rotations, concave_simplify_tolerance, &|| false)
    }

    /// Cooperative phase cancellation between geometry operations. The legacy
    /// wrapper still observes the current Init request. One prepare/orient or
    /// simplification is atomic; an interrupted library is never returned.
    pub fn build_with_control(
        instance: &SPInstance,
        rotations: &[i64],
        concave_simplify_tolerance: f32,
        should_stop: &(impl Fn() -> bool + Sync),
    ) -> Option<Self> {
        let stopped = || should_stop() || crate::config::runtime::init_cancel_requested();
        if stopped() { return None; }
        // 음수 오프셋(= 컨테이너가 원단보다 커지는 설정)은 인셋이 아니다. 0 으로
        // 눌러 두지 않으면 IFP 가 원단 밖까지 넓어진다.
        let strip_inset = instance
            .base_strip
            .shape_modify_config
            .offset
            .unwrap_or(0.0);
        if !strip_inset.is_finite() {
            return None;
        }
        let strip_inset = strip_inset.max(0.0);
        let mut rotations: Vec<i64> = if rotations.is_empty() {
            vec![0]
        } else {
            rotations.to_vec()
        };
        rotations.sort_unstable();
        rotations.dedup();

        // 1) item_id → GeometryHash, 그리고 고유 형상의 원본 PolygonSet
        let mut item_hashes: HashMap<usize, GeometryHash> = HashMap::new();
        let mut instance_polygons: usize = 0;
        // BTreeMap을 써서 준비 순서를 결정적으로 만든다.
        let mut sources: BTreeMap<GeometryHash, PolygonSet> = BTreeMap::new();

        for (item, qty) in instance.items.iter() {
            if stopped() { return None; }
            instance_polygons = instance_polygons.saturating_add(*qty);
            let hash = GeometryHash::of_polygon(item.shape_cd().as_ref());
            item_hashes.insert(item.idx(), hash);
            sources.entry(hash).or_insert_with(|| {
                polygon_set_of_simplified(item.shape_cd().as_ref(), concave_simplify_tolerance)
            });
        }

        // 2) 고유 형상 준비 + 방향별 사본
        let engine = NfpEngine::new();
        let mut shapes: HashMap<(GeometryHash, i64), PreparedShape> = HashMap::new();
        let mut usable: HashSet<GeometryHash> = HashSet::new();

        for (hash, source) in sources.iter() {
            if stopped() { return None; }
            let base = match engine.prepare(source) {
                Ok(prepared) => prepared,
                Err(err) => {
                    log::warn!(
                        "NFP 라이브러리: geometry {:#018x} 준비 실패 — 제외합니다 ({err})",
                        hash.0
                    );
                    continue;
                }
            };
            let mut any_orientation = false;
            for &deg in rotations.iter() {
                if stopped() { return None; }
                match engine.orient(&base, Orientation::from_degrees_exact(deg)) {
                    Ok(oriented) => {
                        shapes.insert((*hash, deg), oriented);
                        any_orientation = true;
                    }
                    Err(err) => {
                        log::warn!(
                            "NFP 라이브러리: geometry {:#018x} 의 {deg}° 회전 실패 — 건너뜁니다 ({err})",
                            hash.0
                        );
                    }
                }
            }
            if any_orientation {
                usable.insert(*hash);
            } else {
                log::warn!(
                    "NFP 라이브러리: geometry {:#018x} 는 모든 회전각에서 실패 — 제외합니다",
                    hash.0
                );
            }
        }

        if stopped() { return None; }
        if shapes.is_empty() {
            log::warn!("NFP 라이브러리: 준비에 성공한 형상이 하나도 없습니다");
            return None;
        }

        // 준비에 실패한 형상을 참조하는 아이템은 매핑에서 제거한다.
        item_hashes.retain(|_, hash| usable.contains(hash));

        let mut combos: Vec<(GeometryHash, i64)> = shapes.keys().copied().collect();
        combos.sort_unstable();

        Some(Self {
            shapes,
            item_hashes,
            rotations,
            combos,
            unique_geometries: usable.len(),
            instance_polygons,
            strip_inset,
        })
    }

    /// importer 가 컨테이너에 적용한 물리적 인셋(mm). 겹침이 꺼져 있으면 0 이다.
    pub(super) fn strip_inset(&self) -> f32 {
        self.strip_inset
    }

    /// `(hash, rotation)`에 해당하는 준비된 형상.
    pub fn shape(&self, hash: GeometryHash, rotation: i64) -> Option<&PreparedShape> {
        self.shapes.get(&(hash, rotation))
    }

    /// item_id의 형상 해시. 준비에 실패한 형상이면 `None`.
    pub fn hash_of_item(&self, item_id: usize) -> Option<GeometryHash> {
        self.item_hashes.get(&item_id).copied()
    }

    /// 정렬·중복 제거된 회전각 목록.
    pub fn rotations(&self) -> &[i64] {
        &self.rotations
    }

    /// 준비에 성공한 고유 형상 수 (문서 예시의 `35`).
    pub fn unique_count(&self) -> usize {
        self.unique_geometries
    }

    /// 인스턴스가 배치해야 하는 폴리곤 총 개수 (문서 예시의 `156`).
    pub fn instance_polygon_count(&self) -> usize {
        self.instance_polygons
    }

    /// self pair를 포함한 정규 pair 총 개수 `n(n+1)/2`.
    pub fn pair_count(&self) -> usize {
        triangular_pair_count(self.combos.len())
    }

    /// 고유 `(hash, rotation)` 조합의 **모든 정규 pair**(self pair 포함).
    ///
    /// `combos`가 오름차순으로 정렬되어 있으므로 `i <= j`로 만든 키는 이미 정규형이다.
    pub fn all_keys(&self) -> Vec<NfpKey> {
        let m = self.combos.len();
        let mut keys = Vec::with_capacity(triangular_pair_count(m));
        for i in 0..m {
            for j in i..m {
                let (ha, ra) = self.combos[i];
                let (hb, rb) = self.combos[j];
                keys.push(NfpKey::new(ha, ra, hb, rb));
            }
        }
        keys
    }
}

// ────────────────────────────────────────────────────────────────────────────
// IFP 캐시 키
// ────────────────────────────────────────────────────────────────────────────

/// Inner-Fit Polygon 캐시 키. strip 크기도 0.001mm 격자로 양자화해 넣는다.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
struct IfpKey {
    hash: GeometryHash,
    rotation: i64,
    width_q: i64,
    height_q: i64,
    /// 물리적 strip 인셋(격자 단위). 샘플러는 이미 인셋된 origin-local 컨테이너를
    /// 넘기고 구성형 배치기는 인셋을 명시적으로 요청하므로, 이 항이 없으면 둘이
    /// 같은 `(w, h)` 슬롯에 서로 다른 기하를 덮어쓴다.
    inset_q: i64,
}

/// IFP 캐시 상한(항목 수). 키에 strip 폭·높이·인셋이 들어가므로 strip 길이를 탐색하는
/// 동안 키가 무한히 늘어난다. 넘치면 세 맵을 함께 비운다 — 재계산만 생기고 결과는 같다.
const IFP_CACHE_MAX_ENTRIES: usize = 4096;

// ────────────────────────────────────────────────────────────────────────────
// NfpLbfCache
// ────────────────────────────────────────────────────────────────────────────

/// 한 방향(정규 또는 부호 반전)의 pair NFP를 쓰는 데 필요한 것 전부.
///
/// `prepared`가 `None`인 경우는 정규화가 실패한 영역뿐이다. 그때는 호출자가
/// 예전처럼 `locate`를 쓴다 — `locate` 역시 같은 정규화에서 `Err`를 내므로
/// 판정 결과는 달라지지 않는다.
#[derive(Clone)]
struct NfpView {
    region: Arc<PolygonSet>,
    mask: Arc<NfpMask>,
    prepared: Option<Arc<PreparedRegion>>,
}

/// Geometry Hash 기준 NFP/IFP 캐시.
///
/// 저장 단위는 `Arc<PolygonSet>`이므로 조회 시 복제 비용이 없다.
pub struct NfpLbfCache {
    library: Arc<PreparedLibrary>,
    engine: NfpEngine,
    /// 정규 키 → pair NFP
    nfp: HashMap<NfpKey, Arc<PolygonSet>>,
    /// 정규 키 → pair NFP의 점유 마스크 ([`NfpMask`])
    nfp_mask: HashMap<NfpKey, Arc<NfpMask>>,
    /// 정규 키 → pair NFP를 **한 번만 정규화**해 둔 질의용 영역.
    ///
    /// `NfpEngine::locate`는 호출마다 영역을 `canonical_paths_from_polygon_set`으로
    /// 통째로 재정규화한다. 정점 40개짜리 point-in-polygon 한 번이 약 36 µs로 측정된
    /// 이유가 그것이며, 여성WSjk 156-패널 3-ordering 배치에서 `locate` 63,588회가
    /// 전체 4.67초 중 약 2.3초를 차지했다. 정규화 결과를 여기 한 번만 만들어 두고
    /// `locate_prepared`로 질의하면 그 재정규화가 통째로 사라진다.
    nfp_prepared: HashMap<NfpKey, Arc<PreparedRegion>>,
    /// **비정규 키까지 포함한** 사용 시점 뷰 캐시.
    ///
    /// 부호 반전이 필요한 방향은 [`negate_polygon_set`]이 매번 새 `PolygonSet`을
    /// 만들었다 — 아이템 하나 놓을 때마다 obstacle 수만큼 발생하는 할당이다.
    /// 반전본과 그 마스크, 그리고 정규화 영역을 여기 한 번만 만들어 두고 `Arc`로 공유한다.
    /// 정규 키는 `nfp` / `nfp_mask` / `nfp_prepared`의 `Arc`를 그대로 복제해 담으므로
    /// 추가 메모리가 없다.
    views: HashMap<NfpKey, NfpView>,
    /// IFP 캐시. `None`은 "들어갈 수 없음"을 캐싱한 결과다.
    ifp: HashMap<IfpKey, Option<Arc<PolygonSet>>>,
    /// IFP의 점유 마스크. `ifp`와 같은 키를 쓴다.
    ///
    /// IFP 판정은 탐침 **하나당 반드시 한 번** 일어나므로 obstacle NFP보다도 호출이
    /// 잦다. 마스크를 함께 두면 그 대부분이 `locate` 없이 끝난다.
    ifp_mask: HashMap<IfpKey, Arc<NfpMask>>,
    /// IFP를 한 번만 정규화해 둔 질의용 영역. `ifp`와 같은 키를 쓴다.
    ///
    /// 마스크가 `Boundary`로 넘긴 나머지가 여기로 온다 — 그 나머지를 싸게 만든다.
    ifp_prepared: HashMap<IfpKey, Arc<PreparedRegion>>,
    /// 계산에 실패한 정규 키(부정 캐시). 같은 pair를 obstacle마다 다시 계산하고
    /// 다시 경고 로그를 찍지 않게 한다.
    failed: HashSet<NfpKey>,
    /// `get`이 `&self`이므로 계측치는 원자 카운터로 둔다.
    hits: AtomicU64,
    misses: AtomicU64,
    prewarm_pairs: usize,
    build_ms: u128,
}

impl NfpLbfCache {
    /// 라이브러리를 소유하는 빈 캐시를 만든다.
    pub fn new(library: PreparedLibrary) -> Self {
        Self {
            library: Arc::new(library),
            engine: NfpEngine::new(),
            nfp: HashMap::new(),
            nfp_mask: HashMap::new(),
            nfp_prepared: HashMap::new(),
            views: HashMap::new(),
            ifp: HashMap::new(),
            ifp_mask: HashMap::new(),
            ifp_prepared: HashMap::new(),
            failed: HashSet::new(),
            hits: AtomicU64::new(0),
            misses: AtomicU64::new(0),
            prewarm_pairs: 0,
            build_ms: 0,
        }
    }

    /// 내부 라이브러리.
    pub fn library(&self) -> &PreparedLibrary {
        &self.library
    }

    /// 스레드마다 하나씩 쓸 사본을 만든다.
    ///
    /// # 왜 필요한가
    ///
    /// ordering 평가와 조각 선택은 **완벽히 병렬**인 일인데, 지금은 단일 스레드로
    /// 돈다. 막고 있는 것은 이 캐시가 `&mut self` 라는 사실 하나다
    /// (`get_or_compute` 가 miss 를 계산해 저장하므로).
    ///
    /// 16코어 머신에서 15개를 놀리고 있다.
    ///
    /// # 사본이 싼 이유
    ///
    /// 무거운 것은 전부 이미 `Arc` 뒤에 있다 — pair NFP, 마스크, 준비영역, IFP.
    /// 복제되는 것은 `HashMap` 의 뼈대와 `Arc` 포인터뿐이다. 여성WSjk 156-패널에서
    /// 5,565 pair 라 사본 하나가 수백 KB 수준이고, 16개를 만들어도 몇 MB다.
    ///
    /// PreparedLibrary and prepared geometry are immutable Arc values shared by forks.
    ///
    /// # 사본이 안전한 이유
    ///
    /// pre-warm 이 끝난 뒤에는 miss 가 거의 없다(실측: hit 12,141 / miss 105).
    /// 스레드가 각자 계산한 miss 는 자기 사본에만 남고 버려지지만, 그건 **결과를
    /// 바꾸지 않는다** — 같은 키에 대해 같은 NFP 가 나오기 때문이다. 잃는 것은
    /// 그 소수 miss 의 재계산 비용뿐이다.
    ///
    /// 계측 카운터는 0 으로 초기화한다 — 사본의 통계를 원본에 섞으면 안 된다.
    pub fn fork(&self) -> Self {
        Self {
            library: self.library.clone(),
            engine: NfpEngine::new(),
            nfp: self.nfp.clone(),
            nfp_mask: self.nfp_mask.clone(),
            nfp_prepared: self.nfp_prepared.clone(),
            views: self.views.clone(),
            ifp: self.ifp.clone(),
            ifp_mask: self.ifp_mask.clone(),
            ifp_prepared: self.ifp_prepared.clone(),
            failed: self.failed.clone(),
            hits: AtomicU64::new(0),
            misses: AtomicU64::new(0),
            prewarm_pairs: self.prewarm_pairs,
            build_ms: self.build_ms,
        }
    }

    /// 모든 정규 pair를 rayon으로 미리 계산한다.
    ///
    /// ```text
    /// 156 polygons → 12,090 unique pairs → Rayon parallel iterator
    /// → 8 threads 기준 약 1,511 NFP / thread → Cache 저장
    /// ```
    ///
    /// 워커는 `Mutex`를 전혀 잡지 않는다. `(NfpKey, PolygonSet)`을 모아서 반환한 뒤
    /// 단일 스레드에서 한 번에 `HashMap`으로 병합한다. `budget`을 넘긴 워커는 즉시
    /// `None`을 돌려 남은 키를 건너뛴다 — 부분 캐시는 정상 동작한다(miss 시
    /// [`Self::get_or_compute`]가 지연 계산한다).
    pub fn prewarm(&mut self, budget: Duration) -> NfpCacheStats {
        self.prewarm_with_control(budget, &|| false)
    }

    /// Captures Init cancellation for Rayon workers, and adds caller phase
    /// control. Completed records remain a coherent reusable partial cache.
    pub fn prewarm_with_control(
        &mut self,
        budget: Duration,
        should_stop: &(impl Fn() -> bool + Sync),
    ) -> NfpCacheStats {
        let start = Instant::now();
        let token = crate::config::runtime::init_cancel_token();
        let stopped = || start.elapsed() >= budget || token.is_some_and(|t| t.is_cancelled()) || should_stop();
        if stopped() { return self.stats(); }
        // 이미 들어 있는 pair는 다시 계산하지 않는다(재-prewarm은 빠진 키만 채운다).
        let keys: Vec<NfpKey> = self
            .library
            .all_keys()
            .into_iter()
            .filter(|k| !self.nfp.contains_key(k))
            .collect();
        if stopped() { return self.stats(); }

        let library = &self.library;
        // 마스크도 여기서 함께 만든다. 워커는 여전히 Mutex를 잡지 않고, 마스크 생성
        // 비용(셀 래스터 + 성분당 locate 1회)이 rayon으로 그대로 흩어진다.
        //
        // 질의용 정규화 영역([`PreparedRegion`])도 같은 워커에서 만든다. 배치 중에
        // 만들면 단일 스레드 비용이지만, 여기서는 NFP 계산과 함께 rayon으로 흩어진다.
        #[allow(clippy::type_complexity)]
        let computed: Vec<(NfpKey, PolygonSet, NfpMask, Option<PreparedRegion>)> = keys
            .par_iter()
            .filter_map(|key| {
                // 예산 초과(또는 호스트의 Init 취소) — 이 워커는 남은 키를 즉시 포기한다.
                if stopped() {
                    return None;
                }
                // NfpEngine은 stateless + Clone이므로 워커마다 새로 만들어도 무해하다.
                let engine = NfpEngine::new();
                let fixed = library.shape(key.shape_a_hash, key.rotation_a)?;
                let moving = library.shape(key.shape_b_hash, key.rotation_b)?;
                match engine.pair_nfp_with_clearance(fixed, moving, &placement_clearance()) {
                    Ok(set) => {
                        if stopped() { return None; }
                        let mask = NfpMask::build(&engine, &set);
                        if stopped() { return None; }
                        let prepared = engine.prepare_region(&set).ok();
                        if stopped() { return None; }
                        Some((*key, set, mask, prepared))
                    }
                    Err(err) => {
                        log::warn!(
                            "NFP pre-warm 실패: {:#018x}@{}° × {:#018x}@{}° ({err})",
                            key.shape_a_hash.0,
                            key.rotation_a,
                            key.shape_b_hash.0,
                            key.rotation_b
                        );
                        None
                    }
                }
            })
            .collect();

        // 단일 스레드 병합 — 여기서만 캐시가 갱신된다.
        let merged = computed.len();
        for (key, set, mask, prepared) in computed {
            self.nfp.insert(key, Arc::new(set));
            self.nfp_mask.insert(key, Arc::new(mask));
            match prepared {
                Some(prepared) => {
                    self.nfp_prepared.insert(key, Arc::new(prepared));
                }
                // 정규화 실패 — 저장하지 않는다. 호출자는 `locate` 경로로 되돌아가고,
                // `locate`도 같은 정규화에서 `Err`를 내므로 판정은 동일하다.
                None => {
                    self.nfp_prepared.remove(&key);
                }
            }
        }
        // 저장 내용이 바뀌었으므로 파생 뷰는 버린다(아무것도 안 바뀌었으면 그대로 둔다).
        if merged > 0 {
            self.views.clear();
        }

        self.prewarm_pairs = self.prewarm_pairs.saturating_add(merged);
        self.build_ms = start.elapsed().as_millis();
        if stopped() { return self.stats(); }

        // ── 부호 반전(flipped) 뷰도 여기서 병렬로 만든다 ──────────────────
        //
        // `NFP(A,B) = -NFP(B,A)` 라 캐시는 정규 키 하나만 저장하고, 반대 방향은 조회 시
        // 반전본을 만든다. 그런데 그 반전본은 `negate_polygon_set` + `NfpMask::build` +
        // `prepare_region` 을 전부 새로 해야 해서 미스 1회가 약 160µs다.
        //
        // 배치 중에 만들면 그게 전부 **단일 스레드**로 쌓인다. 여성WSjk 156-패널에서
        // `collect_obstacles` 가 1,080ms 였고 그 거의 전부가 이 미스였다(6,681회).
        // 여기서 미리 만들면 같은 일이 rayon 으로 흩어진다.
        //
        // 자기 자신과의 pair(`a == b`)는 반전 방향이 곧 정규 방향이라 건너뛴다.
        // `self.nfp` 의 키는 **정규 키**다. 우리가 만들려는 건 그 반대 방향이므로
        // 반드시 `swapped()` 한 키로 저장해야 한다. 정규 키에 반전본을 넣으면 정규 방향
        // 조회가 통째로 오염된다.
        let flip_keys: Vec<NfpKey> = self
            .nfp
            .keys()
            .copied()
            .map(|k| k.swapped())
            .filter(|sk| sk.canonical().1 && !self.views.contains_key(sk))
            .collect();

        #[allow(clippy::type_complexity)]
        let flipped: Vec<(NfpKey, PolygonSet, NfpMask, Option<PreparedRegion>)> = flip_keys
            .par_iter()
            .filter_map(|key| {
                if stopped() {
                    return None;
                }
                let (ckey, is_flipped) = key.canonical();
                debug_assert!(is_flipped);
                let stored = self.nfp.get(&ckey)?;
                let engine = NfpEngine::new();
                let negated = negate_polygon_set(stored);
                if stopped() { return None; }
                let mask = NfpMask::build(&engine, &negated);
                if stopped() { return None; }
                let prepared = engine.prepare_region(&negated).ok();
                if stopped() { return None; }
                Some((*key, negated, mask, prepared))
            })
            .collect();

        for (key, region, mask, prepared) in flipped {
            self.views.insert(
                key,
                NfpView {
                    region: Arc::new(region),
                    mask: Arc::new(mask),
                    prepared: prepared.map(Arc::new),
                },
            );
        }

        self.build_ms = start.elapsed().as_millis();
        self.stats()
    }

    /// 계산 없이 조회만 한다. 부호 반전이 필요하면 반전본을 새로 만들어 돌려준다.
    pub fn get(&self, key: NfpKey) -> Option<Arc<PolygonSet>> {
        let (ckey, flipped) = key.canonical();
        match self.nfp.get(&ckey) {
            Some(stored) => {
                self.hits.fetch_add(1, AtomicOrdering::Relaxed);
                Some(if flipped {
                    Arc::new(negate_polygon_set(stored))
                } else {
                    stored.clone()
                })
            }
            None => {
                self.misses.fetch_add(1, AtomicOrdering::Relaxed);
                None
            }
        }
    }

    /// 조회하고, 없으면 계산해 저장한다.
    ///
    /// ```text
    /// Cache Miss → NFP 계산 → Cache 저장
    /// ```
    ///
    /// 부호 반전본은 캐시에 넣지 않는다. 정규 키 하나만 저장하는 것이 이 캐시의 핵심이다.
    pub fn get_or_compute(&mut self, key: NfpKey) -> Option<Arc<PolygonSet>> {
        let (ckey, flipped) = key.canonical();

        if let Some(stored) = self.nfp.get(&ckey) {
            self.hits.fetch_add(1, AtomicOrdering::Relaxed);
            return Some(if flipped {
                Arc::new(negate_polygon_set(stored))
            } else {
                stored.clone()
            });
        }
        // 부정 캐시 — 이미 실패한 pair는 재계산도, 재로그도 하지 않는다.
        if self.failed.contains(&ckey) {
            self.hits.fetch_add(1, AtomicOrdering::Relaxed);
            return None;
        }
        self.misses.fetch_add(1, AtomicOrdering::Relaxed);

        let engine = self.engine.clone();
        let computed = {
            let fixed = self.library.shape(ckey.shape_a_hash, ckey.rotation_a)?;
            let moving = self.library.shape(ckey.shape_b_hash, ckey.rotation_b)?;
            match engine.pair_nfp_with_clearance(fixed, moving, &placement_clearance()) {
                Ok(set) => set,
                Err(err) => {
                    self.failed.insert(ckey);
                    log::warn!(
                        "NFP 계산 실패: {:#018x}@{}° × {:#018x}@{}° ({err})",
                        ckey.shape_a_hash.0,
                        ckey.rotation_a,
                        ckey.shape_b_hash.0,
                        ckey.rotation_b
                    );
                    return None;
                }
            }
        };

        let stored = Arc::new(computed);
        let mask = Arc::new(NfpMask::build(&engine, &stored));
        self.nfp.insert(ckey, stored.clone());
        self.nfp_mask.insert(ckey, mask);
        // 질의용 정규화 영역도 같이 만들어 둔다 — `locate`의 호출당 재정규화를 없애는 자리다.
        if let Ok(prepared) = engine.prepare_region(&stored) {
            self.nfp_prepared.insert(ckey, Arc::new(prepared));
        }
        Some(if flipped {
            Arc::new(negate_polygon_set(&stored))
        } else {
            stored
        })
    }

    /// 조회(필요하면 계산)해서 **로컬 좌표 NFP와 그 점유 마스크**를 함께 돌려준다.
    ///
    /// [`Self::get_or_compute`]와 기하는 완전히 같다. 다른 점은 두 가지다.
    ///
    /// * 부호 반전본을 **매번 새로 만들지 않는다.** 반전본과 마스크를 `views`에 한 번만
    ///   만들어 두고 `Arc`로 공유한다 (여성WSjk 156-패널 ordering 1회에 `collect_obstacles`가
    ///   12,090번 도는데, 그때마다 발생하던 `PolygonSet` 할당이 사라진다).
    /// * 반환값은 **월드가 아니라 로컬 좌표**다. 호출자는 NFP를 평행이동하는 대신
    ///   질의점에서 obstacle의 평행이동을 빼서 넣는다.
    pub fn view_or_compute(&mut self, key: NfpKey) -> Option<(Arc<PolygonSet>, Arc<NfpMask>)> {
        let view = self.view_entry_or_compute(key)?;
        Some((view.region, view.mask))
    }

    /// [`Self::view_or_compute`]와 같되, **질의용 정규화 영역**([`PreparedRegion`])까지
    /// 함께 돌려준다.
    ///
    /// `NfpEngine::locate`는 호출마다 영역을 재정규화한다. 여성WSjk 156-패널
    /// 3-ordering 기준 `locate` 63,588회 ≈ 2.3초(호출당 ~36 µs)가 거의 전부 그
    /// 재정규화였다. 여기서 방향(정규/부호 반전)마다 **한 번만** 정규화해 두고
    /// `NfpEngine::locate_prepared`로 질의하면 결과는 그대로이고 그 비용만 사라진다.
    ///
    /// 세 번째 성분이 `None`이면 정규화가 실패한 것이다. 그때는 호출자가 `locate`를
    /// 그대로 쓰면 된다 — `locate`도 같은 정규화에서 `Err`를 내므로 판정은 동일하다.
    #[allow(clippy::type_complexity)]
    pub fn view_prepared_or_compute(
        &mut self,
        key: NfpKey,
    ) -> Option<(Arc<PolygonSet>, Arc<NfpMask>, Option<Arc<PreparedRegion>>)> {
        let view = self.view_entry_or_compute(key)?;
        Some((view.region, view.mask, view.prepared))
    }

    /// 두 뷰 접근자의 공통 본체.
    fn view_entry_or_compute(&mut self, key: NfpKey) -> Option<NfpView> {
        if let Some(view) = self.views.get(&key) {
            self.hits.fetch_add(1, AtomicOrdering::Relaxed);
            return Some(view.clone());
        }

        let (ckey, flipped) = key.canonical();
        // 정규 항목이 없으면 기존 경로가 계산·저장한다(계측치도 그쪽에서 올라간다).
        if !self.nfp.contains_key(&ckey) {
            // 정규 키로 부른다 — 비정규 키로 부르면 쓰지도 않을 반전본을 하나 만든다.
            self.get_or_compute(ckey)?;
        } else {
            self.hits.fetch_add(1, AtomicOrdering::Relaxed);
        }

        let stored = self.nfp.get(&ckey)?.clone();
        let stored_mask = match self.nfp_mask.get(&ckey) {
            Some(mask) => mask.clone(),
            None => {
                let mask = Arc::new(NfpMask::build(&self.engine, &stored));
                self.nfp_mask.insert(ckey, mask.clone());
                mask
            }
        };
        let stored_prepared = match self.nfp_prepared.get(&ckey) {
            Some(prepared) => Some(prepared.clone()),
            None => match self.engine.prepare_region(&stored) {
                Ok(prepared) => {
                    let prepared = Arc::new(prepared);
                    self.nfp_prepared.insert(ckey, prepared.clone());
                    Some(prepared)
                }
                // 정규화 실패 — locate 경로로 되돌아간다.
                Err(_) => None,
            },
        };

        // 부호 반전본은 정규본과 **다른** 영역이다. 마스크와 마찬가지로 정규화도
        // 반전본에 대해 따로 한 번 해야 한다.
        let view = if flipped {
            let negated = Arc::new(negate_polygon_set(&stored));
            let mask = Arc::new(NfpMask::build(&self.engine, &negated));
            let prepared = self.engine.prepare_region(&negated).ok().map(Arc::new);
            NfpView {
                region: negated,
                mask,
                prepared,
            }
        } else {
            NfpView {
                region: stored,
                mask: stored_mask,
                prepared: stored_prepared,
            }
        };
        self.views.insert(key, view.clone());
        Some(view)
    }

    /// `(0,0)-(strip_width, strip_height)` 직사각형 컨테이너에 대한 IFP.
    ///
    /// 결과는 `(hash, rotation, 양자화된 width/height)` 키로 별도 맵에 캐싱한다.
    /// [`NfpError::DegenerateConfigurationSpace`]는 정상적인 "들어갈 수 없음"으로
    /// 처리해 `None`을 반환한다.
    pub fn inner_fit(
        &mut self,
        hash: GeometryHash,
        rotation: i64,
        strip_width: f32,
        strip_height: f32,
    ) -> Option<Arc<PolygonSet>> {
        self.inner_fit_with_inset(hash, rotation, strip_width, strip_height, 0.0)
    }

    /// [`inner_fit`](Self::inner_fit)과 같되 물리적 strip 인셋을 함께 받는다.
    ///
    /// 샘플러는 이미 인셋이 적용된 origin-local 컨테이너를 넘기므로 0 을 쓴다.
    /// 구성형 배치기만 명시적으로 값을 넣는다.
    fn inner_fit_with_inset(
        &mut self,
        hash: GeometryHash,
        rotation: i64,
        strip_width: f32,
        strip_height: f32,
        strip_inset: f32,
    ) -> Option<Arc<PolygonSet>> {
        if !strip_width.is_finite() || !strip_height.is_finite() || !strip_inset.is_finite() {
            return None;
        }
        let width_q = quantize_mm(strip_width);
        let height_q = quantize_mm(strip_height);
        if width_q <= 0 || height_q <= 0 {
            return None;
        }

        let key = IfpKey {
            hash,
            rotation,
            width_q,
            height_q,
            inset_q: quantize_mm(strip_inset),
        };
        if let Some(cached) = self.ifp.get(&key) {
            self.hits.fetch_add(1, AtomicOrdering::Relaxed);
            return cached.clone();
        }
        self.misses.fetch_add(1, AtomicOrdering::Relaxed);

        // 격자에 이미 올라간 값으로 컨테이너를 만든다 — 키와 기하가 어긋나지 않게.
        let w = width_q as f64 / GRID_SCALE as f64;
        let h = height_q as f64 / GRID_SCALE as f64;

        // 경계 여유(inset).
        //
        // 커널은 f64 정수 격자(0.001 mm)에서 정확하지만, 앵커는 f32로 돌아가 jagua-rs의
        // CDE가 다시 검사한다. 경계에 정확히 닿는(`Location::Boundary`) 앵커는 f32 변환
        // 뒤 `y_min = -1e-7`처럼 컨테이너 **밖**으로 미세하게 밀려나 exterior collision이
        // 된다. 실제로 fu.json에서 12개 중 2개가 이 사유로 무효 판정되었다.
        //
        // 그래서 IFP 컨테이너를 각 변에서 조금 줄여 앵커가 항상 **엄격히 안쪽**에 오게
        // 한다. 여유는 f32의 상대 오차(≈1.19e-7)를 넉넉히 덮도록 절대 하한(격자 1칸)과
        // 치수 비례분 중 큰 값을 쓴다. 1524 mm 스트립이면 약 0.0018 mm — 무시할 수준이다.
        // 물리적 인셋(원단이 실제로 좁아진 양) + 수치 여유(f32 경계 오차분).
        let physical_inset = key.inset_q as f64 / GRID_SCALE as f64;
        let inset_x = physical_inset + ifp_inset(w);
        let inset_y = physical_inset + ifp_inset(h);
        if w <= 2.0 * inset_x || h <= 2.0 * inset_y {
            return None;
        }
        let (x0, y0) = (inset_x, inset_y);
        let (x1, y1) = (w - inset_x, h - inset_y);
        let container_set = PolygonSet::from_outer(Ring::new(vec![
            NfpPoint::new(x0, y0),
            NfpPoint::new(x1, y0),
            NfpPoint::new(x1, y1),
            NfpPoint::new(x0, y1),
        ]));

        let engine = self.engine.clone();
        let container = match engine.prepare(&container_set) {
            Ok(prepared) => prepared,
            Err(err) => {
                log::warn!("IFP 컨테이너 준비 실패 ({w} × {h}): {err}");
                return None;
            }
        };

        let result = {
            let moving = self.library.shape(hash, rotation)?;
            match engine.inner_fit(&container, moving) {
                Ok(region) if region.is_empty() => None,
                Ok(region) => Some(Arc::new(region)),
                // 컨테이너 anchor 영역이 선/점으로 붕괴 = 정확히 꽉 차거나 들어가지 않는 경우.
                Err(NfpError::DegenerateConfigurationSpace { .. }) => None,
                Err(err) => {
                    log::warn!(
                        "IFP 계산 실패: {:#018x}@{rotation}° in {w} × {h} ({err})",
                        hash.0
                    );
                    None
                }
            }
        };

        // 상한을 넘으면 세 맵을 함께 비운다 — 키가 서로 맞물려 있어 따로 비우면 안 된다.
        if self.ifp.len() >= IFP_CACHE_MAX_ENTRIES {
            self.ifp.clear();
            self.ifp_mask.clear();
            self.ifp_prepared.clear();
        }
        if let Some(region) = result.as_ref() {
            let mask = Arc::new(NfpMask::build(&engine, region));
            self.ifp_mask.insert(key, mask);
            // IFP 판정은 탐침마다 반드시 한 번 일어난다. 마스크가 `Boundary`로 넘긴
            // 나머지가 `locate`로 가는데, 그 재정규화를 여기서 한 번에 끝내 둔다.
            if let Ok(prepared) = engine.prepare_region(region) {
                self.ifp_prepared.insert(key, Arc::new(prepared));
            }
        }
        self.ifp.insert(key, result.clone());
        result
    }

    /// [`Self::inner_fit`]과 같은 IFP를, 점유 마스크와 함께 돌려준다.
    ///
    /// 기하는 [`Self::inner_fit`]과 완전히 같다. 마스크는 `locate` 단축 경로일 뿐이다.
    pub fn inner_fit_view(
        &mut self,
        hash: GeometryHash,
        rotation: i64,
        strip_width: f32,
        strip_height: f32,
    ) -> Option<(Arc<PolygonSet>, Arc<NfpMask>)> {
        self.inner_fit_view_with_inset(hash, rotation, strip_width, strip_height, 0.0)
    }

    /// [`inner_fit_view`](Self::inner_fit_view)의 인셋 인지 변형.
    fn inner_fit_view_with_inset(
        &mut self,
        hash: GeometryHash,
        rotation: i64,
        strip_width: f32,
        strip_height: f32,
        strip_inset: f32,
    ) -> Option<(Arc<PolygonSet>, Arc<NfpMask>)> {
        let region =
            self.inner_fit_with_inset(hash, rotation, strip_width, strip_height, strip_inset)?;
        // `inner_fit`이 방금 채웠거나(miss) 이미 들어 있다(hit). 없으면 지금 만든다.
        let key = IfpKey {
            hash,
            rotation,
            width_q: quantize_mm(strip_width),
            height_q: quantize_mm(strip_height),
            inset_q: quantize_mm(strip_inset),
        };
        let mask = match self.ifp_mask.get(&key) {
            Some(mask) => mask.clone(),
            None => {
                let mask = Arc::new(NfpMask::build(&self.engine, &region));
                self.ifp_mask.insert(key, mask.clone());
                mask
            }
        };
        Some((region, mask))
    }

    /// [`Self::inner_fit_view`]와 같되, **질의용 정규화 영역**까지 함께 돌려준다.
    ///
    /// 기하는 [`Self::inner_fit`]과 완전히 같다. 마스크는 `locate` 단축 경로이고,
    /// [`PreparedRegion`]은 마스크가 넘긴 나머지 `locate`에서 호출당 재정규화를 없앤다.
    /// 세 번째 성분이 `None`이면 정규화가 실패한 것이며, 그때는 `locate`를 그대로 쓴다.
    #[allow(clippy::type_complexity)]
    pub fn inner_fit_view_prepared(
        &mut self,
        hash: GeometryHash,
        rotation: i64,
        strip_width: f32,
        strip_height: f32,
    ) -> Option<(Arc<PolygonSet>, Arc<NfpMask>, Option<Arc<PreparedRegion>>)> {
        // 샘플러는 이미 인셋이 적용된 origin-local 컨테이너를 넘긴다. 그쪽 좌표
        // 규약은 그대로 두고, 구성형 배치기만 명시적으로 인셋을 요청한다.
        self.inner_fit_view_prepared_with_inset(hash, rotation, strip_width, strip_height, 0.0)
    }

    /// [`inner_fit_view_prepared`](Self::inner_fit_view_prepared)의 인셋 인지 변형.
    #[allow(clippy::type_complexity)]
    pub(super) fn inner_fit_view_prepared_with_inset(
        &mut self,
        hash: GeometryHash,
        rotation: i64,
        strip_width: f32,
        strip_height: f32,
        strip_inset: f32,
    ) -> Option<(Arc<PolygonSet>, Arc<NfpMask>, Option<Arc<PreparedRegion>>)> {
        let (region, mask) =
            self.inner_fit_view_with_inset(hash, rotation, strip_width, strip_height, strip_inset)?;
        let key = IfpKey {
            hash,
            rotation,
            width_q: quantize_mm(strip_width),
            height_q: quantize_mm(strip_height),
            inset_q: quantize_mm(strip_inset),
        };
        let prepared = match self.ifp_prepared.get(&key) {
            Some(prepared) => Some(prepared.clone()),
            None => match self.engine.prepare_region(&region) {
                Ok(prepared) => {
                    let prepared = Arc::new(prepared);
                    self.ifp_prepared.insert(key, prepared.clone());
                    Some(prepared)
                }
                // 정규화 실패 — locate 경로로 되돌아간다.
                Err(_) => None,
            },
        };
        Some((region, mask, prepared))
    }

    /// 현재 계측치.
    pub fn stats(&self) -> NfpCacheStats {
        NfpCacheStats {
            hits: self.hits.load(AtomicOrdering::Relaxed),
            misses: self.misses.load(AtomicOrdering::Relaxed),
            entries: self.nfp.len(),
            unique_geometries: self.library.unique_count(),
            prewarm_pairs: self.prewarm_pairs,
            build_ms: self.build_ms,
            instance_polygons: self.library.instance_polygon_count(),
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 단위 테스트 — 외부 데이터 파일에 의존하지 않는 순수 계산만 검증한다.
// ────────────────────────────────────────────────────────────────────────────

/// 배치 간극(mm).
///
/// 커널은 f64 정수 격자(0.001 mm)에서 정확하지만, 채택한 앵커는 f32로 내려가 jagua-rs의
/// CDE가 다시 검사한다. NFP 경계 위의 앵커는 "정확히 맞닿음"이므로, f32 변환에서 1e-4 mm만
/// 밀려도 실제 겹침이 되어 해가 무효 판정된다 (fu.json에서 12개 중 5쌍이 이 사유였다).
///
/// 그래서 pair NFP를 이 값만큼 부풀린다. 부풀린 NFP 바깥의 앵커는 실제 도형끼리 최소
/// 이만큼 떨어져 있으므로 f32 오차가 접촉을 겹침으로 바꿀 수 없다.
///
/// 기존 배치 경로(`initial/compact.rs`)가 쓰는 `PLACE_EPS = 0.05 mm`보다 작게 잡아,
/// 간극 때문에 밀도가 눈에 띄게 손해 보지 않게 한다.
pub(crate) const PLACEMENT_CLEARANCE_MM: f64 = 0.01;

/// 배치 간극용 정사각형 clearance 커널.
///
/// [`Clearance::EuclideanConservative`]는 원을 근사하느라 커널 삼각형이 수십 개가 되고,
/// pair NFP의 원시 조각 수가 그만큼 곱해진다. 정사각형은 삼각형 2개뿐이라 비용이
/// 2배에 그치고, 원점 대칭이라 `NFP(F,M) = -NFP(M,F)` 항등식(캐시의 정규 키 + 부호 반전)이
/// 그대로 유지된다.
///
/// Chebyshev 거리 기준 간극이라 유클리드 거리로는 최소 [`PLACEMENT_CLEARANCE_MM`],
/// 최대 `√2`배까지 벌어진다 — f32 오차 방어에는 충분하고도 남는다.
#[inline]
fn placement_clearance() -> Clearance {
    let e = PLACEMENT_CLEARANCE_MM;
    Clearance::ConvexKernel(Ring::new(vec![
        NfpPoint::new(-e, -e),
        NfpPoint::new(e, -e),
        NfpPoint::new(e, e),
        NfpPoint::new(-e, e),
    ]))
}

/// IFP 컨테이너를 각 변에서 줄일 여유(mm).
///
/// f32 상대 오차(`f32::EPSILON` ≈ 1.19e-7)의 8배와 격자 한 칸(0.001 mm) 중 큰 값이다.
/// 경계에 닿는 앵커가 f32로 내려간 뒤에도 컨테이너 안쪽에 남도록 보장한다.
#[inline]
pub(super) fn ifp_inset(dimension_mm: f64) -> f64 {
    const F32_EPS_MARGIN: f64 = 8.0 * f32::EPSILON as f64;
    (dimension_mm.abs() * F32_EPS_MARGIN).max(1.0 / GRID_SCALE as f64)
}

#[cfg(test)]
mod tests {
    use super::*;
    use jagua_rs::geometry::primitives::Point;

    /// 반시계 방향 사각형.
    fn square(dx: f32, dy: f32) -> SPolygon {
        SPolygon::new(vec![
            Point(dx, dy),
            Point(dx + 10.0, dy),
            Point(dx + 10.0, dy + 4.0),
            Point(dx, dy + 4.0),
        ])
        .expect(crate::dmsg!("valid square"))
    }

    /// 같은 사각형을 정점 나열 시작점만 한 칸 돌린 것.
    fn square_rotated_vertices() -> SPolygon {
        SPolygon::new(vec![
            Point(10.0, 4.0),
            Point(0.0, 4.0),
            Point(0.0, 0.0),
            Point(10.0, 0.0),
        ])
        .expect(crate::dmsg!("valid square"))
    }

    #[test]
    fn quantize_mm_uses_the_0_001mm_grid() {
        assert_eq!(GRID_SCALE, 1_000);
        assert_eq!(quantize_mm(1.0), 1_000);
        assert_eq!(quantize_mm(-2.5), -2_500);
        assert_eq!(quantize_mm(0.0004), 0);
        assert_eq!(quantize_mm(0.0006), 1);
    }

    #[test]
    fn quantized_area2_is_exact_on_the_grid() {
        // 10mm × 4mm = 40mm² → 2A = 80mm² → 격자 단위²로는 80 × 1000²
        let ccw = [(0i64, 0i64), (10_000, 0), (10_000, 4_000), (0, 4_000)];
        assert_eq!(quantized_area2(&ccw), 80_000_000i128);

        // winding이 반대여도 절댓값은 같다.
        let cw: Vec<(i64, i64)> = ccw.iter().rev().copied().collect();
        assert_eq!(quantized_area2(&cw), 80_000_000i128);
    }

    #[test]
    fn geometry_hash_is_translation_invariant() {
        // 인스턴스가 어디에 놓여 있든 형상이 같으면 같은 해시여야 한다.
        // 절대 좌표 f32 shoelace를 그대로 썼다면 이 정도 오프셋에서 이미 갈라진다.
        let a = GeometryHash::of_polygon(&square(0.0, 0.0));
        let b = GeometryHash::of_polygon(&square(137.5, -42.25));
        let c = GeometryHash::of_polygon(&square(12_345.75, -9_876.5));
        assert_eq!(a, b);
        assert_eq!(a, c);
    }

    #[test]
    fn geometry_hash_is_vertex_start_invariant() {
        // 정점 나열 시작점이 달라도 같은 형상이면 같은 해시여야 한다.
        let a = GeometryHash::of_polygon(&square(0.0, 0.0));
        let b = GeometryHash::of_polygon(&square_rotated_vertices());
        assert_eq!(a, b);
    }

    #[test]
    fn geometry_hash_separates_different_shapes() {
        let square_hash = GeometryHash::of_polygon(&square(0.0, 0.0));
        let triangle = SPolygon::new(vec![Point(0.0, 0.0), Point(10.0, 0.0), Point(0.0, 4.0)])
            .expect(crate::dmsg!("valid triangle"));
        assert_ne!(square_hash, GeometryHash::of_polygon(&triangle));

        // 크기가 다르면 다른 등가 클래스다.
        let bigger = SPolygon::new(vec![
            Point(0.0, 0.0),
            Point(20.0, 0.0),
            Point(20.0, 4.0),
            Point(0.0, 4.0),
        ])
        .expect(crate::dmsg!("valid rect"));
        assert_ne!(square_hash, GeometryHash::of_polygon(&bigger));
    }

    #[test]
    fn nfp_key_swapped_is_an_involution() {
        let key = NfpKey::new(GeometryHash(7), 0, GeometryHash(9), 180);
        assert_eq!(key.swapped().swapped(), key);
        assert_eq!(key.swapped().shape_a_hash, GeometryHash(9));
        assert_eq!(key.swapped().rotation_a, 180);
    }

    #[test]
    fn nfp_key_canonical_picks_the_smaller_side_first() {
        let lo = NfpKey::new(GeometryHash(1), 0, GeometryHash(2), 0);
        let hi = lo.swapped();

        let (ck_lo, flip_lo) = lo.canonical();
        let (ck_hi, flip_hi) = hi.canonical();

        assert!(!flip_lo, "이미 정규형이면 반전이 필요 없다");
        assert!(flip_hi, "뒤집힌 키는 부호 반전이 필요하다");
        assert_eq!(ck_lo, ck_hi, "두 방향은 같은 정규 키로 접혀야 한다");
    }

    #[test]
    fn nfp_key_canonical_breaks_ties_on_rotation() {
        // 해시가 같으면 회전각으로 순서를 정한다.
        let h = GeometryHash(42);
        let (ck, flip) = NfpKey::new(h, 180, h, 0).canonical();
        assert!(flip);
        assert_eq!(ck, NfpKey::new(h, 0, h, 180));

        // self pair는 언제나 정규형이고 반전이 필요 없다.
        let (self_key, self_flip) = NfpKey::new(h, 90, h, 90).canonical();
        assert!(!self_flip);
        assert_eq!(self_key, NfpKey::new(h, 90, h, 90));
    }

    #[test]
    fn triangular_pair_count_includes_self_pairs() {
        // 문서: 35개 고유 형상 → 35C2 + self NFP = 630
        assert_eq!(triangular_pair_count(0), 0);
        assert_eq!(triangular_pair_count(1), 1);
        assert_eq!(triangular_pair_count(35), 630);
        assert_eq!(triangular_pair_count(156), 12_246);
    }

    #[test]
    fn negate_polygon_set_reflects_through_the_origin() {
        let ring = Ring::new(vec![
            NfpPoint::new(0.0, 0.0),
            NfpPoint::new(3.0, 0.0),
            NfpPoint::new(3.0, 2.0),
        ]);
        let set = PolygonSet::from_outer(ring);
        let negated = negate_polygon_set(&set);

        assert_eq!(negated.components.len(), 1);
        let vertices = &negated.components[0].outer.vertices;
        assert_eq!(vertices.len(), 3);
        // 정점 순서는 뒤집히고 좌표는 모두 부호가 반전된다.
        assert_eq!(vertices[0], NfpPoint::new(-3.0, -2.0));
        assert_eq!(vertices[1], NfpPoint::new(-3.0, 0.0));
        assert_eq!(vertices[2], NfpPoint::new(0.0, 0.0));
    }

    #[test]
    fn negate_polygon_set_is_an_involution() {
        let set = PolygonSet::from_outer(Ring::new(vec![
            NfpPoint::new(1.0, -1.0),
            NfpPoint::new(4.0, -1.0),
            NfpPoint::new(4.0, 2.0),
            NfpPoint::new(1.0, 2.0),
        ]));
        let twice = negate_polygon_set(&negate_polygon_set(&set));
        assert_eq!(twice, set);
    }

    #[test]
    fn polygon_set_of_keeps_every_vertex_as_a_single_outer_ring() {
        let poly = square(0.0, 0.0);
        let set = polygon_set_of(&poly);
        assert_eq!(set.components.len(), 1);
        assert!(set.components[0].holes.is_empty());
        assert_eq!(set.ring_count(), 1);
        assert_eq!(set.components[0].outer.vertices.len(), poly.n_vertices());
        assert_eq!(
            set.components[0].outer.vertices[0],
            NfpPoint::new(poly.vertices()[0].0 as f64, poly.vertices()[0].1 as f64)
        );
    }
}
