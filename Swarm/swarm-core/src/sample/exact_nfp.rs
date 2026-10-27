//! capmods 전용: exact NFP — 래스터 후보를 접촉 경계까지 슬라이딩.
//!
//! 래스터 NFP가 건네는 격자 위치는 겹치지는 않지만 실제 도형 경계에서
//! 평균 셀 크기의 절반 정도 떨어져 있다. 이 모듈은 각 후보를 좌측, 그다음
//! 하단으로 CDE 이분 탐색으로 밀어붙여 정확한 접촉 위치까지 옮긴다.
//! #5 접촉 슬랙 해소 — 후보별 20+20번 CDE surrogate 체크로 O(40) 추가 평가.
//!
//! # 알고리즘
//!
//! 후보 하나당:
//! 1. 좌측으로 이분 탐색(20회) — 접촉 직전까지 민다.
//! 2. 그 위치에서 하단으로 이분 탐색(20회).
//! 3. 실제 LBF 손실을 계산해 정렬.
//!
//! 각 스텝은 CDE surrogate collision(~수 μs)으로 빠르게 판정하고,
//! 무충돌일 때만 poly collision으로 재확인한다.

use jagua_rs::collision_detection::hazards::filter::NoFilter;
use jagua_rs::entities::{Item, Layout};
use jagua_rs::geometry::geo_traits::TransformableFrom;
use jagua_rs::geometry::primitives::SPolygon;
use jagua_rs::geometry::{DTransformation, Transformation};
use ordered_float::OrderedFloat;

/// 이분 탐색 반복 횟수. 20 → range / 2^20 ≈ range의 1ppm 정밀도.
const BINSRCH_ITERS: usize = 20;

/// 후보별 최소 이동 거리 (mm). 이 값보다 작게 움직였으면 원본을 유지한다.
const MIN_MOVE: f32 = 0.0005;

/// 래스터 후보를 접촉 경계로 슬라이딩하고, 중복을 제거한 뒤
/// 실제 LBF 손실 순으로 정렬해 최대 `max_count`개 반환한다.
pub fn refine_candidates(
    layout: &Layout,
    item: &Item,
    candidates: &[DTransformation],
    x_mult: f32,
    y_mult: f32,
    max_count: usize,
) -> Vec<DTransformation> {
    #[cfg(feature = "init-profile")]
    let _profile = crate::init_profile::Scope::new("detail.exact_nfp_refine");

    let cde = layout.cde();
    let container = layout.container().outer_cd().bbox();

    let mut shape_buf = item.shape_cd().as_ref().clone();
    let item_bbox = item.shape_cd().bbox();

    let mut refined: Vec<(OrderedFloat<f32>, DTransformation)> =
        Vec::with_capacity(candidates.len());

    for &dt in candidates {
        let (r, (x, y)) = (dt.rotation(), dt.translation());

        // 1. 좌측 슬라이딩
        let x_slid = slide_axis(
            &mut shape_buf,
            item,
            cde,
            r,
            x,
            y,
            container.x_min,
            true, /* is_x → x 방향, y 고정 */
        );

        // 2. 하단 슬라이딩
        let y_slid = slide_axis(
            &mut shape_buf,
            item,
            cde,
            r,
            y,
            x_slid,
            container.y_min,
            false, /* !is_x → y 방향, x 고정 */
        );

        let slid_dt = if (x_slid - x).abs() > MIN_MOVE || (y_slid - y).abs() > MIN_MOVE {
            DTransformation::new(r, (x_slid, y_slid))
        } else {
            dt // 이동량이 너무 작으면 원본 유지 (불필요한 정밀도 손실 방지)
        };

        let loss = lbf_loss(layout, item, slid_dt, x_mult, y_mult);
        refined.push((OrderedFloat(loss), slid_dt));
    }

    refined.sort_unstable_by_key(|(loss, _)| *loss);

    // 중복 제거: 아이템 bbox 대각선의 5% 이내 위치는 하나만 남긴다.
    let merge_eps = (item_bbox.width().powi(2) + item_bbox.height().powi(2)).sqrt() * 0.05;
    let mut out = Vec::with_capacity(max_count);
    for (_, dt) in refined {
        if out.len() >= max_count {
            break;
        }
        let too_close = out.iter().any(|&prev: &DTransformation| {
            let (px, py) = prev.translation();
            let (cx, cy) = dt.translation();
            (px - cx).abs() < merge_eps
                && (py - cy).abs() < merge_eps
                && (prev.rotation() - dt.rotation()).abs() < 1e-4
        });
        if !too_close {
            out.push(dt);
        }
    }
    out
}

/// 한 축 방향으로 이분 탐색 슬라이딩.
///
/// `start`에서 `limit`(컨테이너 벽) 방향으로 이동하며 접촉 경계를 찾는다.
/// `start`는 CDE 충돌이 없는 위치여야 한다. `limit`는 컨테이너 경계.
///
/// `is_x`: true이면 x방향(y는 `fixed_coord`로 고정), false이면 y방향(x 고정).
/// `fixed_coord`: is_x일 때 y값, !is_x일 때 x값.
#[allow(clippy::too_many_arguments)]
fn slide_axis(
    shape_buf: &mut SPolygon,
    item: &Item,
    cde: &jagua_rs::collision_detection::CDEngine,
    r: f32,
    start: f32,
    fixed_coord: f32,
    limit: f32,
    is_x: bool,
) -> f32 {
    // start는 충돌 없음, limit 쪽으로 이동하며 접촉점을 찾는다.
    // lo: 충돌 가능 구간, hi: 충돌 없는 구간
    let (mut lo, mut hi) = if limit < start {
        (limit, start) // 좌측/하단 슬라이딩
    } else {
        (start, limit) // limit가 start보다 크면 슬라이딩할 여지가 없어 start가 그대로 답이 된다
    };
    let mut best = start;

    for _ in 0..BINSRCH_ITERS {
        let mid = lo + (hi - lo) * 0.5;
        let pos = if is_x {
            (mid, fixed_coord)
        } else {
            (fixed_coord, mid)
        };
        let transf: Transformation = DTransformation::new(r, pos).into();

        if cde.detect_surrogate_collision(item.shape_cd().surrogate(), &transf, &NoFilter) {
            // surrogate 충돌 → 확실히 충돌
            lo = mid;
        } else {
            shape_buf.transform_from(&item.shape_cd(), &transf);
            if cde.detect_poly_collision(shape_buf, &NoFilter) {
                lo = mid;
            } else {
                best = mid;
                hi = mid;
            }
        }
    }

    best
}

/// LBF 손실: 좌·하단 방향 가중합.
/// [`crate::eval::lbf_evaluator::LBFEvaluator`]의 loss 계산과 동일.
fn lbf_loss(_layout: &Layout, item: &Item, dt: DTransformation, x_mult: f32, y_mult: f32) -> f32 {
    let transf: Transformation = dt.into();
    let mut shape = item.shape_cd().as_ref().clone();
    shape.transform_from(&item.shape_cd(), &transf);
    let poi = shape.poi().center;
    let bbox_corner = shape.bbox().corners()[0];
    x_mult * (poi.0 + bbox_corner.0) + y_mult * (poi.1 + bbox_corner.1)
}
