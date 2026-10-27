use crate::quantify::overlap_proxy::overlap_area_proxy;
use crate::quantify::simd::circles_soa::CirclesSoA;
use float_cmp::approx_eq;
use jagua_rs::geometry::fail_fast::SPSurrogate;
use jagua_rs::geometry::geo_traits::DistanceTo;
use jagua_rs::geometry::primitives::{Circle, Point};
use std::f32::consts::PI;
use std::simd::{Select, Simd};

type F32xN<const W: usize> = Simd<f32, W>;

// region: Runtime dispatch

/// poles_overlap_area_proxy 의 SIMD 버전.
/// 런타임에서 CPU 에따라 자동 선택 AVX-512 (16-wide) → AVX2 (8) → SSE (4).
#[inline]
pub fn poles_overlap_area_proxy_simd(
    sp1: &SPSurrogate,
    sp2: &SPSurrogate,
    epsilon: f32,
    p2: &CirclesSoA,
) -> f32 {
    poles_overlap_area_proxy_simd_bounded(sp1, sp2, epsilon, p2, f32::INFINITY)
        .expect(crate::dmsg!("unbounded call cannot early-exit"))
}

/// 상한이 있는 판. 누적 겹침이 `max_unscaled_overlap` 을 넘으면 즉시 `None` 을 돌려준다.
///
/// # 왜 이게 필요한가
///
/// 분리기 평가는 손실 상한을 넘긴 표본을 어차피 버린다. 그런데 지금 구조는 쌍 하나의
/// 정량화를 **끝까지 계산한 뒤** 바깥에서 버린다 — 이미 초과가 확정된 뒤에도 남은
/// 폴을 계속 훑는다. 상한을 커널 안으로 밀어 넣으면 그 낭비가 사라진다.
///
/// 값은 PI 를 곱하기 **전** 기준이다(호출부가 `max_loss` 에서 역산해 넘긴다).
/// 검사는 `sp1` 폴 하나를 끝낼 때마다 한 번 — 청크마다 검사하면 분기 비용이
/// SIMD 이득을 깎아먹는다.
///
/// 상류 swarm 0.2.0 에서 가져온 기법이고, 런타임 폭 선택(상류는 4-wide 고정)은
/// 우리 커널 것을 그대로 둔다. 음수 침투(`pd <= 0`)의 감쇠는 상류·논문과 같다 —
/// 예전의 `pd <= 0` 클램프는 얕은 충돌의 기울기를 없애서 걷어냈다
/// (`docs/OVERLAP_PROXY_DECAY_2026-09-30.md`).
#[inline]
pub fn poles_overlap_area_proxy_simd_bounded(
    sp1: &SPSurrogate,
    sp2: &SPSurrogate,
    epsilon: f32,
    p2: &CirclesSoA,
    max_unscaled_overlap: f32,
) -> Option<f32> {
    poles_overlap_area_proxy_simd_until(sp1, sp2, epsilon, p2, |overlap| {
        overlap > max_unscaled_overlap
    })
}

/// The predicate sees a monotone partial sum, before multiplication by PI.
/// Callers can verify a cutoff with the same forward arithmetic as their loss.
#[inline]
pub fn poles_overlap_area_proxy_simd_until(
    sp1: &SPSurrogate,
    sp2: &SPSurrogate,
    epsilon: f32,
    p2: &CirclesSoA,
    should_stop: impl Fn(f32) -> bool,
) -> Option<f32> {
    #[cfg(target_arch = "x86_64")]
    {
        // With fewer poles than lanes a wide kernel runs no chunk at all and
        // falls back to the scalar remainder; use the narrower kernel instead.
        let n = p2.x.len();
        if n >= 16 && std::arch::is_x86_feature_detected!("avx512f") {
            return unsafe { kernel_16(sp1, sp2, epsilon, p2, should_stop) };
        }
        if n >= 8 && std::arch::is_x86_feature_detected!("avx2") {
            return unsafe { kernel_8(sp1, sp2, epsilon, p2, should_stop) };
        }
    }
    kernel_4(sp1, sp2, epsilon, p2, should_stop)
}

// endregion

// region: Kernel (generic over SIMD_WIDTH)

#[inline(always)]
fn simd_kernel<const W: usize>(
    sp1: &SPSurrogate,
    sp2: &SPSurrogate,
    epsilon: f32,
    p2: &CirclesSoA,
    should_stop: impl Fn(f32) -> bool,
) -> Option<f32> {
    use std::simd::StdFloat;
    use std::simd::prelude::{SimdFloat, SimdPartialOrd};

    let e_n = F32xN::<W>::splat(epsilon);
    let e_sq_n = F32xN::<W>::splat(epsilon * epsilon);
    let two_e_n = F32xN::<W>::splat(2.0 * epsilon);

    let mut total_overlap = 0.0;
    for p1 in sp1.poles.iter() {
        let r1 = p1.radius;
        let x1_n = F32xN::<W>::splat(p1.center.x());
        let y1_n = F32xN::<W>::splat(p1.center.y());
        let r1_n = F32xN::<W>::splat(r1);

        let chunks = p2.x.len() / W;

        for chunk in 0..chunks {
            let idx = chunk * W;
            let x2 = F32xN::<W>::from_slice(&p2.x[idx..idx + W]);
            let y2 = F32xN::<W>::from_slice(&p2.y[idx..idx + W]);
            let r2 = F32xN::<W>::from_slice(&p2.r[idx..idx + W]);

            let dx = x1_n - x2;
            let dy = y1_n - y2;
            let pd = r1_n + r2 - (dx * dx + dy * dy).sqrt();

            // `pd <= 0` 도 감쇠한다 (순차 구현 `pole_overlap_decay` 와 같은 정의).
            let above_epsilon = pd.simd_ge(e_n);
            let decay_values = e_sq_n / (-pd + two_e_n);
            let pd_decay = above_epsilon.select(pd, decay_values);

            let min_r = r1_n.simd_min(r2);
            total_overlap += (pd_decay * min_r).reduce_sum();
        }

        let remaining_idx = chunks * W;
        for j in remaining_idx..p2.x.len() {
            let p2 = Circle {
                center: Point(p2.x[j], p2.y[j]),
                radius: p2.r[j],
            };
            let pd = (p1.radius + p2.radius) - p1.center.distance_to(&p2.center);
            let pd_decay = if pd >= epsilon {
                pd
            } else {
                epsilon.powi(2) / (2.0 * epsilon - pd)
            };
            total_overlap += pd_decay * f32::min(p1.radius, p2.radius);
        }

        // sp1 폴 하나를 끝낼 때마다 한 번만 본다. 청크마다 검사하면 분기 비용이
        // SIMD 이득을 깎는다. 누적값은 단조 증가(모든 항이 음이 아니다)라
        // 한 번 넘으면 다시 내려올 수 없다 — 조기 탈출이 안전하다.
        if should_stop(total_overlap) {
            return None;
        }
    }

    total_overlap *= PI;

    // 순차 구현과의 동치 검사는 **끝까지 돈 경우에만** 성립한다.
    // 조기 탈출 경로는 애초에 값을 돌려주지 않으므로 비교 대상이 아니다.
    debug_assert!(
        approx_eq!(
            f32,
            total_overlap,
            overlap_area_proxy(sp1, sp2, epsilon),
            epsilon = total_overlap * 1e-3
        ),
        "SIMD and SEQ results do not match: {} vs {}",
        total_overlap,
        overlap_area_proxy(sp1, sp2, epsilon)
    );
    debug_assert!(total_overlap.is_finite() && total_overlap >= 0.0);
    Some(total_overlap)
}

// endregion

// region: Target-feature

fn kernel_4(
    sp1: &SPSurrogate,
    sp2: &SPSurrogate,
    epsilon: f32,
    p2: &CirclesSoA,
    should_stop: impl Fn(f32) -> bool,
) -> Option<f32> {
    simd_kernel::<4>(sp1, sp2, epsilon, p2, should_stop)
}

#[target_feature(enable = "avx2")]
unsafe fn kernel_8(
    sp1: &SPSurrogate,
    sp2: &SPSurrogate,
    epsilon: f32,
    p2: &CirclesSoA,
    should_stop: impl Fn(f32) -> bool,
) -> Option<f32> {
    simd_kernel::<8>(sp1, sp2, epsilon, p2, should_stop)
}

#[target_feature(enable = "avx512f")]
unsafe fn kernel_16(
    sp1: &SPSurrogate,
    sp2: &SPSurrogate,
    epsilon: f32,
    p2: &CirclesSoA,
    should_stop: impl Fn(f32) -> bool,
) -> Option<f32> {
    simd_kernel::<16>(sp1, sp2, epsilon, p2, should_stop)
}

// endregion
