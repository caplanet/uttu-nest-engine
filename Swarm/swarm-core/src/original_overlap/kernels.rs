//! Exact f64 coordinate arithmetic and conservative bounds filtering.
//! Clipper intersection/erosion remains the final authority. No FMA or f32
//! approximation is used to change the acceptance boundary.

use jagua_rs::geometry::primitives::Point;

pub(super) fn transform(points: &[Point], pose: [f64; 4]) -> Result<Vec<(f64, f64)>, String> {
    #[cfg(feature = "simd")]
    {
        #[cfg(target_arch = "x86_64")]
        {
            if std::arch::is_x86_feature_detected!("avx512f") {
                // SAFETY: target features are checked on this CPU.
                return unsafe { transform_avx512(points, pose) };
            }
            if std::arch::is_x86_feature_detected!("avx2") {
                return unsafe { transform_avx2(points, pose) };
            }
        }
        transform_simd::<2>(points, pose)
    }
    #[cfg(not(feature = "simd"))]
    transform_scalar(points, pose)
}

fn valid(x: f64, y: f64) -> bool {
    x.is_finite() && y.is_finite() && x.abs() <= 1e8 && y.abs() <= 1e8
}
#[cfg_attr(feature = "simd", allow(dead_code))]
fn transform_scalar(
    points: &[Point],
    [sin, cos, tx, ty]: [f64; 4],
) -> Result<Vec<(f64, f64)>, String> {
    points
        .iter()
        .map(|&Point(px, py)| {
            let x = cos * px as f64 - sin * py as f64 + tx;
            let y = sin * px as f64 + cos * py as f64 + ty;
            if valid(x, y) {
                Ok((x, y))
            } else {
                Err("original coordinate out of exact grid range".into())
            }
        })
        .collect()
}

#[cfg(feature = "simd")]
#[inline(always)]
fn transform_simd<const W: usize>(
    points: &[Point],
    [sin, cos, tx, ty]: [f64; 4],
) -> Result<Vec<(f64, f64)>, String> {
    use std::simd::{
        Simd,
        prelude::{SimdFloat, SimdPartialOrd},
    };
    let mut result = Vec::with_capacity(points.len());
    let mut chunks = points.chunks_exact(W);
    for chunk in &mut chunks {
        let px = Simd::<f64, W>::from_array(std::array::from_fn(|i| chunk[i].0 as f64));
        let py = Simd::<f64, W>::from_array(std::array::from_fn(|i| chunk[i].1 as f64));
        let x = Simd::splat(cos) * px - Simd::splat(sin) * py + Simd::splat(tx);
        let y = Simd::splat(sin) * px + Simd::splat(cos) * py + Simd::splat(ty);
        if !(x.is_finite()
            & y.is_finite()
            & x.abs().simd_le(Simd::splat(1e8))
            & y.abs().simd_le(Simd::splat(1e8)))
        .all()
        {
            return Err("original coordinate out of exact grid range".into());
        }
        result.extend(x.to_array().into_iter().zip(y.to_array()));
    }
    for &Point(px, py) in chunks.remainder() {
        let x = cos * px as f64 - sin * py as f64 + tx;
        let y = sin * px as f64 + cos * py as f64 + ty;
        if !valid(x, y) {
            return Err("original coordinate out of exact grid range".into());
        }
        result.push((x, y));
    }
    Ok(result)
}

#[cfg(all(feature = "simd", target_arch = "x86_64"))]
#[target_feature(enable = "avx2")]
unsafe fn transform_avx2(points: &[Point], pose: [f64; 4]) -> Result<Vec<(f64, f64)>, String> {
    transform_simd::<4>(points, pose)
}
#[cfg(all(feature = "simd", target_arch = "x86_64"))]
#[target_feature(enable = "avx512f")]
unsafe fn transform_avx512(points: &[Point], pose: [f64; 4]) -> Result<Vec<(f64, f64)>, String> {
    transform_simd::<8>(points, pose)
}

/// Up to four consecutive boxes from the x-sorted sweep. Bounds are finite,
/// computed from the same quantized Clipper coordinates as the scalar check.
#[inline]
pub(super) fn candidate_mask(a: [f64; 4], boxes: &[[f64; 4]], depth: f64) -> u8 {
    debug_assert!(boxes.len() <= 4);
    #[cfg(feature = "simd")]
    {
        use std::simd::{
            Simd,
            prelude::{SimdFloat, SimdPartialOrd},
        };
        let v: [Simd<f64, 4>; 4] = std::array::from_fn(|axis| {
            Simd::from_array(std::array::from_fn(|i| {
                boxes.get(i).map_or(0., |b| b[axis])
            }))
        });
        let x = Simd::splat(a[2]).simd_min(v[2]) - Simd::splat(a[0]).simd_max(v[0]);
        let y = Simd::splat(a[3]).simd_min(v[3]) - Simd::splat(a[1]).simd_max(v[1]);
        ((x.simd_gt(Simd::splat(depth)) & y.simd_gt(Simd::splat(depth))).to_bitmask() as u8)
            & ((1u8 << boxes.len()) - 1)
    }
    #[cfg(not(feature = "simd"))]
    candidate_mask_scalar(a, boxes, depth)
}
#[cfg_attr(feature = "simd", allow(dead_code))]
fn candidate_mask_scalar(a: [f64; 4], boxes: &[[f64; 4]], depth: f64) -> u8 {
    boxes.iter().enumerate().fold(0, |mask, (i, b)| {
        let x = a[2].min(b[2]) - a[0].max(b[0]);
        let y = a[3].min(b[3]) - a[1].max(b[1]);
        mask | (u8::from(x > depth && y > depth) << i)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn transforms_match_scalar_bits_including_tails_and_extreme_coordinates() {
        let points: Vec<_> = (0..257)
            .map(|i| {
                Point(
                    ((i * 7919) % 13001) as f32 - 6500.25,
                    ((i * 997) % 8501) as f32 - 4200.125,
                )
            })
            .collect();
        for n in [0, 1, 2, 3, 4, 7, 8, 9, 17, 257] {
            for angle in [0f64, 0.31, std::f64::consts::FRAC_PI_2, -2.7] {
                let (s, c) = angle.sin_cos();
                let pose = [s, c, 812.0023, -401.987];
                let expected = transform_scalar(&points[..n], pose).unwrap();
                let got = transform(&points[..n], pose).unwrap();
                assert_eq!(
                    expected
                        .iter()
                        .map(|(x, y)| (x.to_bits(), y.to_bits()))
                        .collect::<Vec<_>>(),
                    got.iter()
                        .map(|(x, y)| (x.to_bits(), y.to_bits()))
                        .collect::<Vec<_>>()
                );
                #[cfg(feature = "simd")]
                for result in [
                    transform_simd::<2>(&points[..n], pose),
                    transform_simd::<4>(&points[..n], pose),
                    transform_simd::<8>(&points[..n], pose),
                ] {
                    assert_eq!(expected, result.unwrap());
                }
            }
        }
        for points in [
            vec![Point(f32::NAN, 0.)],
            vec![Point(f32::INFINITY, 1.); 9],
            vec![Point(1e9, 1.); 8],
        ] {
            assert!(transform(&points, [0., 1., 0., 0.]).is_err());
            assert!(transform_scalar(&points, [0., 1., 0., 0.]).is_err());
        }
    }
    #[test]
    fn bounds_masks_match_scalar_at_allowance_boundaries_and_tails() {
        let a = [-10., -20., 30., 40.];
        for depth in [0., 0.001, 0.999, 4.999, 5., 100.] {
            for n in 0..=4 {
                for delta in [-0.00000001, 0., 0.00000001, 1., -1.] {
                    let boxes = [
                        [30. - depth + delta, 0., 50., 50.],
                        [-50., 40. - depth + delta, 50., 80.],
                        [0., 0., 1., 1.],
                        [-1e8, -1e8, 1e8, 1e8],
                    ];
                    assert_eq!(
                        candidate_mask(a, &boxes[..n], depth),
                        candidate_mask_scalar(a, &boxes[..n], depth)
                    );
                }
            }
        }
    }
}
