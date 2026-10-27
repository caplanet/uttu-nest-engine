use crate::consts::OVERLAP_PROXY_EPSILON_DIAM_RATIO;
use crate::quantify::calc_shape_penalty;
use crate::quantify::simd::circles_soa::CirclesSoA;
use crate::quantify::simd::overlap_proxy_simd::{
    poles_overlap_area_proxy_simd, poles_overlap_area_proxy_simd_until,
};
use jagua_rs::geometry::primitives::SPolygon;
use std::f32::consts::PI;

pub mod circles_soa;
pub(crate) mod gls_weights;
pub mod overlap_proxy_simd;

/// Quantifies a collision between two simple polygons using SIMD.
/// Mirrors the functionality of `quantify_collision_poly_poly` but leverages SIMD instructions.
#[inline(always)]
pub fn quantify_collision_poly_poly_simd(s1: &SPolygon, s2: &SPolygon, poles2: &CirclesSoA) -> f32 {
    let epsilon = f32::max(s1.diameter(), s2.diameter()) * OVERLAP_PROXY_EPSILON_DIAM_RATIO;

    let overlap_proxy =
        poles_overlap_area_proxy_simd(s1.surrogate(), s2.surrogate(), epsilon, poles2)
            + epsilon.powi(2);

    debug_assert!(overlap_proxy.is_normal());

    let penalty = calc_shape_penalty(s1, s2);

    overlap_proxy.sqrt() * penalty
}

/// 손실 상한이 있는 판. 결과가 `max_loss` 를 넘는 것이 확정되면 계산을 중단하고
/// `None` 을 돌려준다.
///
/// 상한을 폴 루프 안까지 밀어 넣는 것이 요점이다. 지금 분리기는 쌍 하나를 끝까지
/// 정량화한 뒤 바깥에서 버리는데, 초과가 확정된 시점 이후의 폴 순회가 통째로 낭비다.
///
/// 역산 상한은 빠른 비교용 힌트다. 실제 탈락은 원래 정방향 f32 계산으로
/// 확인하므로 상한과 같은 후보를 반올림 오차 때문에 버리지 않는다.
#[inline(always)]
pub fn quantify_collision_poly_poly_simd_bounded(
    s1: &SPolygon,
    s2: &SPolygon,
    poles2: &CirclesSoA,
    max_loss: f32,
) -> Option<f32> {
    quantify_collision_poly_poly_simd_bounded_by(s1, s2, poles2, max_loss, |loss| loss > max_loss)
}

/// `max_loss_hint` only gates the more expensive forward comparison. It never
/// decides rejection: inverse f32 arithmetic can round across an exact bound.
/// `rejects` must be monotone for nonnegative loss (including weighted sums).
#[inline(always)]
pub(crate) fn quantify_collision_poly_poly_simd_bounded_by(
    s1: &SPolygon,
    s2: &SPolygon,
    poles2: &CirclesSoA,
    max_loss_hint: f32,
    rejects: impl Fn(f32) -> bool,
) -> Option<f32> {
    let epsilon = f32::max(s1.diameter(), s2.diameter()) * OVERLAP_PROXY_EPSILON_DIAM_RATIO;
    let epsilon_sq = epsilon * epsilon;
    let penalty = calc_shape_penalty(s1, s2);

    // loss = sqrt(overlap * PI + eps^2) * penalty 를 overlap 에 대해 뒤집는다.
    // 힌트가 음수(이미 상한을 넘긴 누적)면 제곱이 양수로 튀어 조기 탈출이 꺼진다 — 0 으로 묶는다.
    let max_unscaled_overlap = {
        let max_sqrt_proxy = (max_loss_hint / penalty).max(0.0);
        let overlap = (max_sqrt_proxy * max_sqrt_proxy - epsilon_sq) / PI;
        overlap.max(0.0)
    };

    // 커널이 돌려주는 값에는 PI 가 이미 곱해져 있다(상한 비교만 곱하기 전 기준이다).
    let overlap_proxy = poles_overlap_area_proxy_simd_until(
        s1.surrogate(),
        s2.surrogate(),
        epsilon,
        poles2,
        |partial| {
            partial >= max_unscaled_overlap && rejects((partial * PI + epsilon_sq).sqrt() * penalty)
        },
    )? + epsilon_sq;

    let loss = overlap_proxy.sqrt() * penalty;
    (!rejects(loss)).then_some(loss)
}

#[inline(always)]
pub fn quantify_collision_poly_poly_simd_with_penalty(
    s1: &SPolygon,
    s2: &SPolygon,
    poles2: &CirclesSoA,
    penalty: f32,
) -> f32 {
    let epsilon = f32::max(s1.diameter(), s2.diameter()) * OVERLAP_PROXY_EPSILON_DIAM_RATIO;

    let overlap_proxy =
        poles_overlap_area_proxy_simd(s1.surrogate(), s2.surrogate(), epsilon, poles2)
            + epsilon.powi(2);

    debug_assert!(overlap_proxy.is_normal());

    overlap_proxy.sqrt() * penalty
}

#[inline(always)]
pub(crate) fn quantify_collision_poly_poly_simd_bounded_by_with_penalty(
    s1: &SPolygon,
    s2: &SPolygon,
    poles2: &CirclesSoA,
    penalty: f32,
    max_loss_hint: f32,
    rejects: impl Fn(f32) -> bool,
) -> Option<f32> {
    let epsilon = f32::max(s1.diameter(), s2.diameter()) * OVERLAP_PROXY_EPSILON_DIAM_RATIO;
    let epsilon_sq = epsilon * epsilon;

    // loss = sqrt(overlap * PI + eps^2) * penalty 를 overlap 에 대해 뒤집는다.
    // 힌트가 음수(이미 상한을 넘긴 누적)면 제곱이 양수로 튀어 조기 탈출이 꺼진다 — 0 으로 묶는다.
    let max_unscaled_overlap = {
        let max_sqrt_proxy = (max_loss_hint / penalty).max(0.0);
        let overlap = (max_sqrt_proxy * max_sqrt_proxy - epsilon_sq) / PI;
        overlap.max(0.0)
    };

    // 커널이 돌려주는 값에는 PI 가 이미 곱해져 있다(상한 비교만 곱하기 전 기준이다).
    let overlap_proxy = poles_overlap_area_proxy_simd_until(
        s1.surrogate(),
        s2.surrogate(),
        epsilon,
        poles2,
        |partial| {
            partial >= max_unscaled_overlap && rejects((partial * PI + epsilon_sq).sqrt() * penalty)
        },
    )? + epsilon_sq;

    let loss = overlap_proxy.sqrt() * penalty;
    (!rejects(loss)).then_some(loss)
}

#[cfg(test)]
mod bounded_tests {
    use super::*;
    use jagua_rs::geometry::fail_fast::SPSurrogateConfig;
    use jagua_rs::geometry::primitives::Point;

    /// 서로게이트까지 만들어 둔 사각형. SIMD 경로는 폴을 쓰므로 서로게이트가 필수다.
    fn rect(x: f32, y: f32, w: f32, h: f32) -> SPolygon {
        let mut p = SPolygon::new(vec![
            Point(x, y),
            Point(x + w, y),
            Point(x + w, y + h),
            Point(x, y + h),
        ])
        .unwrap();
        p.generate_surrogate(SPSurrogateConfig {
            n_pole_limits: [(64, 0.0), (16, 0.8), (8, 0.9)],
            ff_pole_area_ratio: 0.001,
            n_ff_piers: 0,
        })
        .unwrap();
        p
    }

    fn soa(p: &SPolygon) -> CirclesSoA {
        let mut s = CirclesSoA::new();
        s.load(&p.surrogate().poles);
        s
    }

    #[test]
    fn prepared_penalty_preserves_simd_and_adjacent_weighted_bounds() {
        use crate::prepared_geometry::ShapeFactor;
        let a = rect(0., 0., 100., 60.);
        let b = rect(49.7, 7.3, 100., 60.);
        let poles = soa(&b);
        let penalty =
            (ShapeFactor::new(&a).root_for(&a) * ShapeFactor::new(&b).root_for(&b)).sqrt();
        let exact = quantify_collision_poly_poly_simd(&a, &b, &poles);
        assert_eq!(
            exact.to_bits(),
            quantify_collision_poly_poly_simd_with_penalty(&a, &b, &poles, penalty).to_bits()
        );
        for weight in [0.001, 1.0, 123.45, 100000.] {
            let total = 31.27 + (0.03 + exact * weight);
            for bound in [total.next_down(), total, total.next_up()] {
                let old = quantify_collision_poly_poly_simd_bounded_by(
                    &a,
                    &b,
                    &poles,
                    (bound - 31.27 - 0.03) / weight,
                    |loss| 31.27 + (0.03 + loss * weight) > bound,
                );
                let new = quantify_collision_poly_poly_simd_bounded_by_with_penalty(
                    &a,
                    &b,
                    &poles,
                    penalty,
                    (bound - 31.27 - 0.03) / weight,
                    |loss| 31.27 + (0.03 + loss * weight) > bound,
                );
                assert_eq!(old.map(f32::to_bits), new.map(f32::to_bits));
            }
        }
    }

    #[test]
    fn 상한이_넉넉하면_무제한판과_같은_값을_준다() {
        let a = rect(0.0, 0.0, 100.0, 60.0);
        let b = rect(50.0, 20.0, 100.0, 60.0);
        let poles_b = soa(&b);

        let exact = quantify_collision_poly_poly_simd(&a, &b, &poles_b);
        assert!(
            exact > 0.0,
            "겹친 두 사각형의 손실은 양수여야 한다: {exact}"
        );

        let bounded = quantify_collision_poly_poly_simd_bounded(&a, &b, &poles_b, exact * 2.0);
        let got = bounded.expect(crate::dmsg!("상한이 결과의 2배면 끝까지 계산되어야 한다"));
        assert!(
            (got - exact).abs() <= exact * 1e-4,
            "무제한 {exact} 과 달라졌다: {got}"
        );
    }

    #[test]
    fn 상한이_모자라면_중단한다() {
        let a = rect(0.0, 0.0, 100.0, 60.0);
        let b = rect(50.0, 20.0, 100.0, 60.0);
        let poles_b = soa(&b);

        let exact = quantify_collision_poly_poly_simd(&a, &b, &poles_b);
        assert_eq!(
            quantify_collision_poly_poly_simd_bounded(&a, &b, &poles_b, exact * 0.5),
            None,
            "상한이 결과의 절반이면 조기 탈출해야 한다"
        );
    }

    #[test]
    fn 상한이_0이면_양의_손실을_거절한다() {
        let a = rect(0.0, 0.0, 100.0, 60.0);
        let b = rect(50.0, 20.0, 100.0, 60.0);
        assert_eq!(
            quantify_collision_poly_poly_simd_bounded(&a, &b, &soa(&b), 0.0),
            None
        );
    }

    #[test]
    fn 겹치지_않으면_상한과_무관하게_같은_값이다() {
        let a = rect(0.0, 0.0, 50.0, 50.0);
        let b = rect(500.0, 500.0, 50.0, 50.0);
        let poles_b = soa(&b);

        let exact = quantify_collision_poly_poly_simd(&a, &b, &poles_b);
        let got = quantify_collision_poly_poly_simd_bounded(&a, &b, &poles_b, exact * 2.0)
            .expect(crate::dmsg!("떨어진 조각은 상한에 걸릴 이유가 없다"));
        assert!((got - exact).abs() <= exact.max(1.0) * 1e-4);
    }

    #[test]
    fn bounded_preserves_exact_and_adjacent_float_bounds() {
        let a = rect(0.0, 0.0, 100.0, 60.0);
        for x in [-110.0, -30.3, 0.0, 0.125, 49.7, 99.99, 120.0] {
            let b = rect(x, 7.3, 100.0, 60.0);
            let poles = soa(&b);
            let exact = quantify_collision_poly_poly_simd(&a, &b, &poles);
            for bound in [
                0.0,
                exact.next_down(),
                exact,
                exact.next_up(),
                f32::INFINITY,
            ] {
                let result = quantify_collision_poly_poly_simd_bounded(&a, &b, &poles, bound);
                assert_eq!(
                    result.is_none(),
                    exact > bound,
                    "x={x}, loss={exact}, bound={bound}"
                );
                if let Some(loss) = result {
                    assert_eq!(loss.to_bits(), exact.to_bits());
                }
            }
            for weight in [0.001, 0.7, 1.0, 123.45, 100000.0] {
                for (cached, extra) in [(0.0, 0.0), (31.27, 0.03), (1e12, 1e5)] {
                    let total: f32 = cached + (extra + exact * weight);
                    for bound in [total.next_down(), total, total.next_up()] {
                        let result = quantify_collision_poly_poly_simd_bounded_by(
                            &a,
                            &b,
                            &poles,
                            (bound - cached - extra) / weight,
                            |loss| cached + (extra + loss * weight) > bound,
                        );
                        assert_eq!(result.is_none(), total > bound);
                        if let Some(loss) = result {
                            assert_eq!(loss.to_bits(), exact.to_bits());
                        }
                    }
                }
            }
        }
    }
}
