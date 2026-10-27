use jagua_rs::geometry::fail_fast::SPSurrogate;
use jagua_rs::geometry::geo_traits::DistanceTo;
use std::f32::consts::PI;

/// Calculates a proxy for the overlap area between two simple polygons (using poles).
/// Algorithm 3 from https://doi.org/10.48550/arXiv.2509.13329
#[inline(always)]
pub fn overlap_area_proxy(sp1: &SPSurrogate, sp2: &SPSurrogate, epsilon: f32) -> f32 {
    let mut total_overlap = 0.0;
    for p1 in &sp1.poles {
        for p2 in &sp2.poles {
            // Penetration depth between the two poles (circles)
            let pd = (p1.radius + p2.radius) - p1.center.distance_to(&p2.center);
            let pd_decay = pole_overlap_decay(pd, epsilon);

            total_overlap += pd_decay * f32::min(p1.radius, p2.radius);
        }
    }
    total_overlap *= PI;
    debug_assert!(total_overlap.is_finite() && total_overlap >= 0.0);

    total_overlap
}

/// 폴이 겹치지 않아도(`penetration_depth <= 0`) 쌍곡 감쇠를 그대로 적용한다 — 논문
/// Algorithm 3 및 상류 swarm 와 같은 정의다. 0 으로 자르면 얕은 충돌(폴이 겹치지
/// 않는 충돌)의 손실이 상수가 되어 분리기가 기울기를 잃는다.
/// 실측은 `docs/OVERLAP_PROXY_DECAY_2026-09-30.md`.
#[inline(always)]
fn pole_overlap_decay(penetration_depth: f32, epsilon: f32) -> f32 {
    if penetration_depth >= epsilon {
        penetration_depth
    } else {
        epsilon.powi(2) / (2.0 * epsilon - penetration_depth)
    }
}
