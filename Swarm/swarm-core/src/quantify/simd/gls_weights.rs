//! Lane-independent GLS updates. No floating-point sum is reassociated.
use crate::optimizer::separator::GlsWeightConfig;
#[cfg(test)]
use crate::quantify::tracker::update_weights_scalar;
use crate::quantify::tracker::{CTEntry, updated_gls_weight_capped};
use std::simd::prelude::*;
use std::simd::{Select, Simd};

/// Called only for sufficiently large matrices by the tracker. `gls` must
/// have been sanitized exactly once, as in the scalar tracker.
#[inline]
pub(crate) fn update_weights(
    pairs: &mut [CTEntry],
    containers: &mut [CTEntry],
    gls: GlsWeightConfig,
) {
    #[cfg(target_arch = "x86_64")]
    if std::arch::is_x86_feature_detected!("avx2") {
        unsafe { update_avx2(pairs, containers, gls) };
        return;
    }
    update_width::<4>(pairs, containers, gls);
}

#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx2")]
unsafe fn update_avx2(pairs: &mut [CTEntry], containers: &mut [CTEntry], gls: GlsWeightConfig) {
    update_width::<8>(pairs, containers, gls);
}

#[inline(always)]
fn max_loss<const W: usize>(entries: &[CTEntry]) -> f32 {
    let zero = Simd::<f32, W>::splat(0.);
    let mut maximum = zero;
    let mut chunks = entries.chunks_exact(W);
    for chunk in &mut chunks {
        let losses = Simd::from_array(std::array::from_fn(|i| chunk[i].loss));
        let usable = losses.is_finite() & losses.simd_gt(zero);
        maximum = maximum.simd_max(usable.select(losses, zero));
    }
    chunks
        .remainder()
        .iter()
        .map(|e| e.loss)
        .filter(|loss| loss.is_finite() && *loss > 0.)
        .fold(maximum.reduce_max(), f32::max)
}

#[inline(always)]
fn update_width<const W: usize>(
    pairs: &mut [CTEntry],
    containers: &mut [CTEntry],
    gls: GlsWeightConfig,
) {
    let maximum = max_loss::<W>(pairs).max(max_loss::<W>(containers));
    update_entries::<W>(pairs, maximum, gls);
    update_entries::<W>(containers, maximum, gls);
}

#[inline(always)]
fn update_entries<const W: usize>(entries: &mut [CTEntry], maximum: f32, gls: GlsWeightConfig) {
    let zero = Simd::<f32, W>::splat(0.);
    let one = Simd::<f32, W>::splat(1.);
    let cap = Simd::<f32, W>::splat(gls.max_weight);
    let decay = Simd::<f32, W>::splat(gls.decay);
    let min_inc = Simd::<f32, W>::splat(gls.min_inc);
    let inc_delta = Simd::<f32, W>::splat(gls.max_inc - gls.min_inc);
    let max_loss = Simd::<f32, W>::splat(maximum);
    let mut chunks = entries.chunks_exact_mut(W);
    for chunk in &mut chunks {
        let losses = Simd::from_array(std::array::from_fn(|i| chunk[i].loss));
        let weights = Simd::from_array(std::array::from_fn(|i| chunk[i].weight));
        let mut multiplier = if maximum > 0. {
            let colliding = losses.is_finite() & losses.simd_gt(zero);
            colliding.select(min_inc + inc_delta * (losses / max_loss), decay)
        } else {
            decay
        };
        multiplier = (multiplier.is_finite() & multiplier.simd_gt(zero)).select(multiplier, one);
        let neutral = weights.is_nan() | weights.simd_le(zero);
        let weights = neutral.select(one, weights).simd_max(one).simd_min(cap);
        let updated = weights * multiplier;
        let invalid = updated.is_nan() | updated.simd_eq(Simd::splat(f32::NEG_INFINITY));
        let updated = invalid
            .select(one, updated)
            .simd_max(one)
            .simd_min(cap)
            .to_array();
        for (entry, weight) in chunk.iter_mut().zip(updated) {
            entry.weight = weight;
        }
    }
    for entry in chunks.into_remainder() {
        let multiplier = if entry.loss.is_finite() && entry.loss > 0. && maximum > 0. {
            gls.min_inc + (gls.max_inc - gls.min_inc) * (entry.loss / maximum)
        } else {
            gls.decay
        };
        entry.weight = updated_gls_weight_capped(entry.weight, multiplier, gls.max_weight);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entries(size: usize, colliding: bool) -> Vec<CTEntry> {
        (0..size)
            .map(|i| CTEntry {
                loss: if colliding && i % 7 == 0 {
                    (i % 127) as f32 + 0.37
                } else {
                    0.
                },
                weight: 1. + (i % 11) as f32 * 0.13,
            })
            .collect()
    }

    #[test]
    fn every_vector_width_preserves_normal_and_all_clear_updates_across_tails() {
        for size in 0..145 {
            for colliding in [false, true] {
                let pairs = entries(size, colliding);
                let containers = entries(17, colliding);
                let mut expected_pairs = pairs.clone();
                let mut expected_containers = containers.clone();
                update_weights_scalar(
                    &mut expected_pairs,
                    &mut expected_containers,
                    GlsWeightConfig::DEFAULT,
                );
                for width in [4, 8] {
                    let mut actual_pairs = pairs.clone();
                    let mut actual_containers = containers.clone();
                    if width == 4 {
                        update_width::<4>(
                            &mut actual_pairs,
                            &mut actual_containers,
                            GlsWeightConfig::DEFAULT,
                        );
                    } else {
                        update_width::<8>(
                            &mut actual_pairs,
                            &mut actual_containers,
                            GlsWeightConfig::DEFAULT,
                        );
                    }
                    for (a, b) in actual_pairs
                        .iter()
                        .chain(&actual_containers)
                        .zip(expected_pairs.iter().chain(&expected_containers))
                    {
                        assert_eq!(a.loss.to_bits(), b.loss.to_bits());
                        assert_eq!(
                            a.weight.to_bits(),
                            b.weight.to_bits(),
                            "size={size}, width={width}, colliding={colliding}"
                        );
                    }
                }
            }
        }
    }

    /// Fixed work; alternate measurement order to avoid favoring the second run.
    #[test]
    #[ignore]
    fn gls_kernel_benchmark() {
        for size in [128, 12402, 131328] {
            for colliding in [false, true] {
                let initial_pairs = entries(size, colliding);
                let initial_containers = entries(12, colliding);
                let iterations = (20_000_000 / size).max(100);
                for repeat in 0..5 {
                    let mut times = [0.; 2];
                    let mut results = Vec::new();
                    for optimized in if repeat % 2 == 0 {
                        [false, true]
                    } else {
                        [true, false]
                    } {
                        let mut pairs = initial_pairs.clone();
                        let mut containers = initial_containers.clone();
                        let start = std::time::Instant::now();
                        for _ in 0..iterations {
                            if optimized {
                                update_weights(
                                    std::hint::black_box(&mut pairs),
                                    std::hint::black_box(&mut containers),
                                    GlsWeightConfig::DEFAULT,
                                );
                            } else {
                                update_weights_scalar(
                                    std::hint::black_box(&mut pairs),
                                    std::hint::black_box(&mut containers),
                                    GlsWeightConfig::DEFAULT,
                                );
                            }
                        }
                        times[usize::from(optimized)] = start.elapsed().as_secs_f64();
                        results.push(
                            pairs
                                .iter()
                                .chain(&containers)
                                .map(|e| e.weight.to_bits())
                                .collect::<Vec<_>>(),
                        );
                    }
                    assert_eq!(results[0], results[1]);
                    println!(
                        "GLS size={size} colliding={colliding} repeat={repeat} scalar_s={:.6} simd_s={:.6} speedup={:.3}",
                        times[0],
                        times[1],
                        times[0] / times[1]
                    );
                }
            }
        }
    }
}
