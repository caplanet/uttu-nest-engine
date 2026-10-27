// 학습률 상수는 이제 `GlsWeightConfig::DEFAULT` 를 거쳐 들어온다. 상한만 여기서
// 직접 쓰는데, 설정이 비정상일 때 되돌아갈 최후의 기본값이기 때문이다.
use crate::consts::GLS_WEIGHT_MAX;
use crate::prepared_geometry::ShapeFactor;
use crate::quantify::pair_matrix::PairMatrix;
use crate::quantify::{
    quantify_collision_poly_container_with_penalty, quantify_collision_poly_poly_with_penalty,
};
use crate::util::assertions::tracker_matches_layout;
use jagua_rs::collision_detection::hazards::BasicHazardEntity;
use jagua_rs::collision_detection::hazards::filter::NoFilter;
use jagua_rs::collision_detection::hazards::collector::BasicHazardCollector;
use jagua_rs::entities::{Layout, PItemKey};
use jagua_rs::geometry::primitives::SPolygon;
use ordered_float::Float;
use slotmap::SecondaryMap;
use std::sync::OnceLock;

/// Lazy exact sums: each invalidated value is recomputed in the original order,
/// avoiding the rounding drift of incremental subtraction/addition.
#[derive(Debug, Clone, Default)]
struct LossCache {
    rows: Vec<OnceLock<f32>>,
    weighted_rows: Vec<OnceLock<f32>>,
    total: OnceLock<f32>,
    weighted_total: OnceLock<f32>,
}

impl LossCache {
    fn new(size: usize) -> Self {
        Self {
            rows: vec![OnceLock::new(); size],
            weighted_rows: vec![OnceLock::new(); size],
            ..Self::default()
        }
    }
    fn clear(&mut self) {
        for row in &mut self.rows {
            row.take();
        }
        self.clear_weights();
        self.total.take();
    }
    fn copy_from(&mut self, source: &Self) {
        self.rows.clone_from(&source.rows);
        self.weighted_rows.clone_from(&source.weighted_rows);
        self.total.clone_from(&source.total);
        self.weighted_total.clone_from(&source.weighted_total);
    }
    fn clear_weights(&mut self) {
        for row in &mut self.weighted_rows {
            row.take();
        }
        self.weighted_total.take();
    }
    fn invalidate_pair(&mut self, a: usize, b: usize) {
        self.rows[a].take();
        self.rows[b].take();
        self.weighted_rows[a].take();
        self.weighted_rows[b].take();
        self.total.take();
        self.weighted_total.take();
    }
}

/// Tracker of both collisions between pair of items and collisions with the container.
/// It also stores the weights for every pair of hazards and is used as a cache for collisions.
#[derive(Debug)]
pub struct CollisionTracker {
    pub place_groups: crate::optimizer::place_groups::PlaceGroups,
    pub size: usize,
    pub pk_idx_map: SecondaryMap<PItemKey, usize>,
    pair_collisions: PairMatrix,
    container_collisions: Vec<CTEntry>,
    cache: LossCache,
    shape_factors: Vec<ShapeFactor>,
    collector: BasicHazardCollector,
    /// idx별 고정 여부. 고정-고정 쌍 및 고정-컨테이너 충돌은 어떤 이동으로도
    /// 해소할 수 없는 상수이므로 loss 집계에서 제외한다.
    pub frozen: Vec<bool>,
}

impl Clone for CollisionTracker {
    fn clone(&self) -> Self {
        Self {
            place_groups: self.place_groups.clone(),
            size: self.size,
            shape_factors: self.shape_factors.clone(),
            pk_idx_map: self.pk_idx_map.clone(),
            pair_collisions: self.pair_collisions.clone(),
            container_collisions: self.container_collisions.clone(),
            frozen: self.frozen.clone(),
            cache: self.cache.clone(),
            // Scratch has no semantic state and need not be copied into snapshots.
            collector: BasicHazardCollector::new(),
        }
    }
}

pub type CTSnapshot = CollisionTracker;

impl CollisionTracker {
    pub fn new(l: &Layout) -> Self {
        Self::new_with_frozen(l, &[])
    }

    /// `frozen_item_mask`는 item_id로 인덱싱된 고정 마스크.
    pub fn new_with_frozen(l: &Layout, frozen_item_mask: &[bool]) -> Self {
        Self::new_impl(
            l,
            frozen_item_mask,
            None,
            &crate::optimizer::place_groups::PlaceGroups::default(),
        )
    }

    pub(crate) fn new_with_frozen_prepared(
        l: &Layout,
        frozen_item_mask: &[bool],
        prepared: &crate::prepared_geometry::PreparedGeometry,
    ) -> Self {
        Self::new_impl(
            l,
            frozen_item_mask,
            Some(prepared),
            &crate::optimizer::place_groups::PlaceGroups::default(),
        )
    }

    pub fn new_with_frozen_and_groups(
        l: &Layout,
        frozen: &[bool],
        groups: &crate::optimizer::place_groups::PlaceGroups,
    ) -> Self {
        Self::new_impl(l, frozen, None, groups)
    }

    pub(crate) fn new_with_frozen_prepared_and_groups(
        l: &Layout,
        frozen: &[bool],
        prepared: &crate::prepared_geometry::PreparedGeometry,
        groups: &crate::optimizer::place_groups::PlaceGroups,
    ) -> Self {
        Self::new_impl(l, frozen, Some(prepared), groups)
    }

    fn new_impl(
        l: &Layout,
        frozen_item_mask: &[bool],
        prepared: Option<&crate::prepared_geometry::PreparedGeometry>,
        groups: &crate::optimizer::place_groups::PlaceGroups,
    ) -> Self {
        let size = l.placed_items().len();

        // Create the tracker
        let mut ot = Self {
            place_groups: groups.clone(),
            cache: LossCache::new(size),
            shape_factors: l
                .placed_items()
                .values()
                .map(|pi| {
                    prepared.map_or_else(
                        || ShapeFactor::new(&pi.shape()),
                        |prepared| prepared.shape_factor(pi.item().idx(), &pi.shape()),
                    )
                })
                .collect(),
            collector: BasicHazardCollector::with_capacity(size + 1),
            size,
            pk_idx_map: l
                .placed_items()
                .keys()
                .enumerate()
                .map(|(i, pk)| (pk, i))
                .collect(),
            pair_collisions: PairMatrix::new(size),
            container_collisions: vec![
                CTEntry {
                    weight: 1.0,
                    loss: 0.0
                };
                size
            ],
            frozen: l
                .placed_items()
                .iter()
                .map(|(_, pi)| frozen_item_mask.get(pi.item().idx()).copied().unwrap_or(false))
                .collect(),
        };

        // Recompute the loss for all items
        l.placed_items()
            .keys()
            .for_each(|pk| ot.recompute_loss_for_item(pk, l));

        debug_assert!(tracker_matches_layout(&ot, l));

        ot
    }

    fn recompute_loss_for_item(&mut self, pk: PItemKey, l: &Layout) {
        let idx = self.pk_idx_map[pk];
        let pi = &l.placed_items()[pk];
        // The envelope (convex hull) carries the shape factor; a multipart item's
        // parts only contribute to the overlap proxy, summed over part pairs.
        let hull = &pi.shape();
        self.shape_factors[idx].refresh(hull);

        // Reset all current loss values for the item
        for i in 0..self.size {
            if self.pair_collisions[(idx, i)].loss != 0.0 {
                self.cache.invalidate_pair(idx, i);
                self.pair_collisions[(idx, i)].loss = 0.0;
            }
        }
        self.cache.invalidate_pair(idx, idx);
        self.container_collisions[idx].loss = 0.0;

        for shape in pi.shapes() {
            // Compute which hazards are currently colliding with this part
            let collector = &mut self.collector;
            collector.clear();
            l.cde().collect_poly_collisions(shape, &NoFilter, collector);

            // For each colliding hazard, quantify the collision and store it in the tracker
            for (_, haz) in collector.iter() {
                match haz {
                    BasicHazardEntity::PlacedItem {
                        pk: other_pk,
                        part: other_part,
                        ..
                    } => {
                        // The placement's own parts are not collisions.
                        if *other_pk == pk {
                            continue;
                        }
                        let other = &l.placed_items()[*other_pk];
                        if self.place_groups.same_group(pi.item().idx(), other.item().idx()) {
                            continue;
                        }
                        let shape_other = &other.shapes()[*other_part];
                        let idx_other = self.pk_idx_map[*other_pk];

                        // 고정-고정 충돌은 해소 불가능한 상수 → loss에서 제외
                        if self.frozen[idx] && self.frozen[idx_other] {
                            continue;
                        }

                        let penalty = (self.shape_factors[idx].root_for(hull)
                            * self.shape_factors[idx_other].root_for(&other.shape()))
                        .sqrt();
                        let loss =
                            quantify_collision_poly_poly_with_penalty(shape, shape_other, penalty);
                        assert!(loss > 0.0, "loss for a collision should be > 0.0");
                        self.pair_collisions[(idx, idx_other)].loss += loss;
                        self.cache.invalidate_pair(idx, idx_other);
                    }
                    BasicHazardEntity::Exterior => {
                        // 고정 아이템과 컨테이너 경계의 충돌은 해소 불가능한 상수 → loss에서 제외
                        if self.frozen[idx] {
                            continue;
                        }

                        let root = self.shape_factors[idx].root_for(hull);
                        let loss = quantify_collision_poly_container_with_penalty(
                            shape,
                            l.container().outer_cd().bbox(),
                            (root * root).sqrt(),
                        );
                        assert!(loss > 0.0, "loss for a collision should be > 0.0");
                        self.container_collisions[idx].loss += loss;
                    }
                    _ => unimplemented!("unsupported hazard entity"),
                }
            }
        }
    }

    /// Share the immutable pair matrix; a worker detaches it on its first write.
    /// Keys, frozen policy and scalar caches still belong to each tracker.
    pub fn copy_from(&mut self, src: &Self) {
        self.cache.copy_from(&src.cache);
        self.shape_factors.clone_from(&src.shape_factors);
        self.place_groups.clone_from(&src.place_groups);
        self.size = src.size;
        self.pk_idx_map.clone_from(&src.pk_idx_map);
        self.pair_collisions.size = src.pair_collisions.size;
        self.pair_collisions
            .data
            .clone_from(&src.pair_collisions.data);
        self.container_collisions.clear();
        self.container_collisions
            .extend_from_slice(&src.container_collisions);
        self.frozen.clear();
        self.frozen.extend_from_slice(&src.frozen);
    }

    pub fn restore_but_keep_weights(&mut self, cts: &CTSnapshot, layout: &Layout) {
        self.cache.clear();
        //Copy the loss and keys, but keep the weights
        self.pk_idx_map.clone_from(&cts.pk_idx_map);
        self.shape_factors.clone_from(&cts.shape_factors);
        self.frozen.clone_from(&cts.frozen);
        self.place_groups.clone_from(&cts.place_groups);
        self.pair_collisions
            .data
            .iter_mut()
            .zip(cts.pair_collisions.data.iter())
            .for_each(|(a, b)| a.loss = b.loss);
        self.container_collisions
            .iter_mut()
            .zip(cts.container_collisions.iter())
            .for_each(|(a, b)| a.loss = b.loss);
        debug_assert!(tracker_matches_layout(self, layout));
    }

    pub fn save(&self) -> CTSnapshot {
        self.clone()
    }

    /// capmods/adaptive 전용: 재구축된 트래커에 이전 트래커의 GLS 가중치를 이월한다.
    ///
    /// 양쪽 모두에 존재하는(= PItemKey가 유지된, 즉 이동하지 않은) 아이템의
    /// 컨테이너 가중치와, 그런 아이템끼리의 쌍 가중치만 복사한다. 제거 후 재삽입되어
    /// 키가 바뀐 아이템은 위치 자체가 달라졌으므로 기본 가중치 1.0에서 다시 시작한다.
    pub fn inherit_weights_from(&mut self, old: &Self) {
        self.cache.clear_weights();
        // (pk, 새 idx, 옛 idx) — 양쪽에 다 있는 아이템만
        let survivors: Vec<(usize, usize)> = self
            .pk_idx_map
            .iter()
            .filter_map(|(pk, &new_idx)| old.pk_idx_map.get(pk).map(|&old_idx| (new_idx, old_idx)))
            .collect();

        for &(new_idx, old_idx) in &survivors {
            self.container_collisions[new_idx].weight = old.container_collisions[old_idx].weight;
        }
        for (a, &(n1, o1)) in survivors.iter().enumerate() {
            for &(n2, o2) in &survivors[a + 1..] {
                // PairMatrix는 대칭 삼각 행렬이라 한 방향만 쓰면 된다
                self.pair_collisions[(n1, n2)].weight = old.pair_collisions[(o1, o2)].weight;
            }
        }
    }

    pub fn register_item_move(&mut self, l: &Layout, old_pk: PItemKey, new_pk: PItemKey) {
        //swap the keys in the pk_idx_map
        let idx = self.pk_idx_map.remove(old_pk).unwrap();
        self.pk_idx_map.insert(new_pk, idx);

        self.recompute_loss_for_item(new_pk, l);

        debug_assert!(tracker_matches_layout(self, l));
    }

    /// Algorithm 8 from https://doi.org/10.48550/arXiv.2509.13329
    ///
    /// `consts.rs` 의 기본 학습률을 쓴다. 탐색 경로는 설정 가능한
    /// [`update_weights_with`](Self::update_weights_with) 를 부른다.
    pub fn update_weights(&mut self) {
        self.update_weights_with(crate::optimizer::separator::GlsWeightConfig::DEFAULT);
    }

    /// [`update_weights`](Self::update_weights)와 같되 학습률·상한을 설정으로 받는다.
    ///
    /// 기본값에서는 상수를 쓰던 기존 코드와 한 비트도 다르지 않다.
    pub fn update_weights_with(&mut self, gls: crate::optimizer::separator::GlsWeightConfig) {
        let gls = gls.sanitized();
        self.cache.clear_weights();
        #[cfg(feature = "simd")]
        if self.pair_collisions.data.len() + self.container_collisions.len() >= 128 {
            crate::quantify::simd::gls_weights::update_weights(
                &mut self.pair_collisions.data,
                &mut self.container_collisions,
                gls,
            );
            return;
        }

        // Retain the original scalar body here. Short matrices should not pay
        // for an extra dispatch/fallback call or lose its existing inlining.
        let max_loss = self
            .pair_collisions
            .data
            .iter()
            .chain(self.container_collisions.iter())
            .map(|e| e.loss)
            .filter(|loss| loss.is_finite() && *loss > 0.0)
            .fold(0.0, |a, b| a.max(b));
        for e in self
            .pair_collisions
            .data
            .iter_mut()
            .chain(self.container_collisions.iter_mut())
        {
            let multiplier = if e.loss.is_finite() && e.loss > 0.0 && max_loss > 0.0 {
                gls.min_inc + (gls.max_inc - gls.min_inc) * (e.loss / max_loss)
            } else {
                gls.decay
            };
            e.weight = updated_gls_weight_capped(e.weight, multiplier, gls.max_weight);
        }
    }

    #[cfg(test)]
    pub(crate) fn update_weights_scalar_for_benchmark(
        &mut self,
        gls: crate::optimizer::separator::GlsWeightConfig,
    ) {
        self.cache.clear_weights();
        update_weights_scalar(
            &mut self.pair_collisions.data,
            &mut self.container_collisions,
            gls.sanitized(),
        );
    }

    pub fn get_pair_weight(&self, pk1: PItemKey, pk2: PItemKey) -> f32 {
        let (idx1, idx2) = (self.pk_idx_map[pk1], self.pk_idx_map[pk2]);
        self.pair_collisions[(idx1, idx2)].weight
    }

    pub fn get_container_weight(&self, pk: PItemKey) -> f32 {
        let idx = self.pk_idx_map[pk];
        self.container_collisions[idx].weight
    }

    /// Algorithm 1 from https://doi.org/10.48550/arXiv.2509.13329
    /// Evaluations between item pairs are stored in this data-structure for quick and easy retrieval.
    pub fn get_pair_loss(&self, pk1: PItemKey, pk2: PItemKey) -> f32 {
        let (idx1, idx2) = (self.pk_idx_map[pk1], self.pk_idx_map[pk2]);
        self.pair_collisions[(idx1, idx2)].loss
    }

    pub fn get_container_loss(&self, pk: PItemKey) -> f32 {
        let idx = self.pk_idx_map[pk];
        self.container_collisions[idx].loss
    }

    pub fn get_loss(&self, pk: PItemKey) -> f32 {
        let idx = self.pk_idx_map[pk];
        *self.cache.rows[idx].get_or_init(|| {
            let pair_loss = (0..self.size)
                .map(|i| self.pair_collisions[(idx, i)].loss)
                .sum::<f32>();

            self.container_collisions[idx].loss + pair_loss
        })
    }

    /// Reuses invariant roots, falling back if a candidate/layout was rebound.
    #[inline(always)]
    pub fn shape_root(&self, pk: PItemKey, shape: &SPolygon) -> f32 {
        self.pk_idx_map
            .get(pk)
            .and_then(|&idx| self.shape_factors.get(idx))
            .map_or_else(
                || shape.surrogate().convex_hull_area.sqrt(),
                |factor| factor.root_for(shape),
            )
    }

    #[inline(always)]
    pub fn shape_penalty(&self, a: PItemKey, sa: &SPolygon, b: PItemKey, sb: &SPolygon) -> f32 {
        (self.shape_root(a, sa) * self.shape_root(b, sb)).sqrt()
    }

    pub fn get_weighted_loss(&self, pk: PItemKey) -> f32 {
        let idx = self.pk_idx_map[pk];
        *self.cache.weighted_rows[idx].get_or_init(|| {
            let w_pair_loss = (0..self.size)
                .map(|i| self.pair_collisions[(idx, i)].weighted_loss())
                .sum::<f32>();

            self.container_collisions[idx].weighted_loss() + w_pair_loss
        })
    }

    pub fn get_total_loss(&self) -> f32 {
        *self.cache.total.get_or_init(|| {
            let cont_o = self
                .container_collisions
                .iter()
                .map(|e| e.loss)
                .sum::<f32>();

            let pair_o = self
                .pair_collisions
                .data
                .iter()
                .map(|e| e.loss)
                .sum::<f32>();

            cont_o + pair_o
        })
    }

    pub fn get_total_weighted_loss(&self) -> f32 {
        *self.cache.weighted_total.get_or_init(|| {
            let cont_w_o = self
                .container_collisions
                .iter()
                .map(|e| e.weighted_loss())
                .sum::<f32>();

            let pair_w_o = self
                .pair_collisions
                .data
                .iter()
                .map(|e| e.weighted_loss())
                .sum::<f32>();

            cont_w_o + pair_w_o
        })
    }

    pub fn pair_collisions(&self) -> &PairMatrix {
        &self.pair_collisions
    }
    pub fn container_collisions(&self) -> &[CTEntry] {
        &self.container_collisions
    }

    /// Exclusive mutation invalidates every dependent sum before exposing data.
    pub fn pair_collisions_mut(&mut self) -> &mut PairMatrix {
        self.cache.clear();
        &mut self.pair_collisions
    }
    pub fn container_collisions_mut(&mut self) -> &mut [CTEntry] {
        self.cache.clear();
        &mut self.container_collisions
    }
}

/// Original scalar test oracle. `gls` has been sanitized exactly once.
#[cfg(test)]
pub(super) fn update_weights_scalar(
    pairs: &mut [CTEntry],
    containers: &mut [CTEntry],
    gls: crate::optimizer::separator::GlsWeightConfig,
) {
    // Find the maximum usable loss across all entries. Invalid geometry
    // must not turn the GLS multiplier and learned weight into NaN.
    let max_loss = pairs
        .iter()
        .chain(containers.iter())
        .map(|e| e.loss)
        .filter(|loss| loss.is_finite() && *loss > 0.0)
        .fold(0.0, |a, b| a.max(b));

    // Go over all entries (pairs) and modify their weights.
    for e in pairs.iter_mut().chain(containers.iter_mut()) {
        let multiplier = match e.loss.is_finite() && e.loss > 0.0 && max_loss > 0.0 {
            false => {
                // No collision at the moment, slowly decay the weight back to 1.0
                gls.decay
            }
            true => {
                // Collision detected, increase the weight based on 'how bad' the collision is relative to the worst collision
                gls.min_inc + (gls.max_inc - gls.min_inc) * (e.loss / max_loss)
            }
        };
        e.weight = updated_gls_weight_capped(e.weight, multiplier, gls.max_weight);
    }
}

#[derive(Debug, Clone, Copy)]
pub struct CTEntry {
    pub loss: f32,
    pub weight: f32,
}

impl CTEntry {
    pub fn weighted_loss(&self) -> f32 {
        self.weight * self.loss
    }
}

/// Apply one GLS update while keeping the learned weight numerically usable.
///
/// A positive infinity is treated as a saturated weight. NaN, negative
/// infinity, and non-positive values have no meaningful learning history and
/// restart at the neutral weight. The result is checked independently so a
/// future multiplier change cannot reintroduce an overflow/NaN path.
fn updated_gls_weight(weight: f32, multiplier: f32) -> f32 {
    updated_gls_weight_capped(weight, multiplier, GLS_WEIGHT_MAX)
}

/// [`updated_gls_weight`]와 같되 상한을 호출자가 정한다.
pub(super) fn updated_gls_weight_capped(weight: f32, multiplier: f32, max_weight: f32) -> f32 {
    let max_weight = if max_weight.is_finite() && max_weight > 1.0 {
        max_weight
    } else {
        GLS_WEIGHT_MAX
    };
    let weight = if weight.is_nan() || weight == f32::NEG_INFINITY || weight <= 0.0 {
        1.0
    } else if weight == f32::INFINITY {
        max_weight
    } else {
        weight.clamp(1.0, max_weight)
    };
    let multiplier = if multiplier.is_finite() && multiplier > 0.0 {
        multiplier
    } else {
        1.0
    };
    let updated = weight * multiplier;

    if updated.is_nan() || updated == f32::NEG_INFINITY {
        1.0
    } else if updated == f32::INFINITY {
        max_weight
    } else {
        updated.clamp(1.0, max_weight)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // 테스트는 상수를 직접 참조해 기본 학습률의 계약을 고정한다.
    use crate::consts::GLS_WEIGHT_DECAY;

    #[cfg(feature = "simd")]
    #[test]
    fn simd_gls_matches_scalar_bits_for_invalid_entries_tails_and_custom_rates() {
        use crate::optimizer::separator::GlsWeightConfig;
        let values = [
            f32::NAN,
            f32::INFINITY,
            f32::NEG_INFINITY,
            -1.,
            -0.,
            0.,
            f32::from_bits(1),
            0.001,
            1.,
            1.000001,
            100.,
            f32::MAX,
        ];
        let configs = [
            GlsWeightConfig::DEFAULT,
            GlsWeightConfig {
                min_inc: 1.001,
                max_inc: 2.37,
                decay: 0.91,
                max_weight: 18.31,
            },
            GlsWeightConfig {
                min_inc: f32::MAX,
                max_inc: 0.,
                decay: f32::NAN,
                max_weight: f32::MAX,
            },
            GlsWeightConfig {
                min_inc: -1.,
                max_inc: 0.,
                decay: 1.,
                max_weight: 1.0001,
            },
        ];
        for size in 0..145 {
            let pairs: Vec<_> = (0..size)
                .map(|i| CTEntry {
                    loss: values[i % values.len()],
                    weight: values[(i / values.len()) % values.len()],
                })
                .collect();
            let containers: Vec<_> = (0..17)
                .map(|i| CTEntry {
                    loss: values[(i * 7) % values.len()],
                    weight: values[i % values.len()],
                })
                .collect();
            for cfg in configs {
                let cfg = cfg.sanitized();
                let max_loss = pairs
                    .iter()
                    .chain(&containers)
                    .map(|e| e.loss)
                    .filter(|loss| loss.is_finite() && *loss > 0.)
                    .fold(0., f32::max);
                let expected: Vec<_> = pairs
                    .iter()
                    .chain(&containers)
                    .map(|e| {
                        let multiplier = if e.loss.is_finite() && e.loss > 0. && max_loss > 0. {
                            cfg.min_inc + (cfg.max_inc - cfg.min_inc) * (e.loss / max_loss)
                        } else {
                            cfg.decay
                        };
                        updated_gls_weight_capped(e.weight, multiplier, cfg.max_weight).to_bits()
                    })
                    .collect();
                let mut actual_pairs = pairs.clone();
                let mut actual_containers = containers.clone();
                crate::quantify::simd::gls_weights::update_weights(
                    &mut actual_pairs,
                    &mut actual_containers,
                    cfg,
                );
                for ((actual, original), expected) in actual_pairs
                    .iter()
                    .chain(&actual_containers)
                    .zip(pairs.iter().chain(&containers))
                    .zip(expected)
                {
                    assert_eq!(actual.loss.to_bits(), original.loss.to_bits());
                    assert_eq!(
                        actual.weight.to_bits(),
                        expected,
                        "size={size}, cfg={cfg:?}"
                    );
                }
            }
        }
    }

    fn tracker_with_one_collision(loss: f32) -> CollisionTracker {
        CollisionTracker {
            place_groups: crate::optimizer::place_groups::PlaceGroups::default(),
            cache: LossCache::new(1),
            shape_factors: Vec::new(),
            collector: BasicHazardCollector::new(),
            size: 1,
            pk_idx_map: SecondaryMap::new(),
            pair_collisions: PairMatrix::new(1),
            container_collisions: vec![CTEntry { loss, weight: 1.0 }],
            frozen: vec![false],
        }
    }

    #[test]
    fn gls_weight_update_recovers_non_finite_weights() {
        let mut tracker = tracker_with_one_collision(1.0);
        tracker.container_collisions[0].weight = f32::NAN;
        tracker.pair_collisions.data[0].weight = f32::INFINITY;

        tracker.update_weights();

        for entry in tracker
            .pair_collisions
            .data
            .iter()
            .chain(tracker.container_collisions.iter())
        {
            assert!(entry.weight.is_finite());
            assert!((1.0..=GLS_WEIGHT_MAX).contains(&entry.weight));
        }

        assert_eq!(updated_gls_weight(f32::NEG_INFINITY, GLS_WEIGHT_DECAY), 1.0);
        assert_eq!(updated_gls_weight(10.0, f32::NAN), 10.0);
        assert_eq!(
            updated_gls_weight(f32::MAX, f32::MAX),
            GLS_WEIGHT_MAX,
            "a non-finite multiplication result must saturate at the upper bound"
        );
    }

    #[test]
    fn long_running_gls_updates_keep_weighted_loss_finite() {
        let mut tracker = tracker_with_one_collision(2.0);

        for _ in 0..10_000 {
            tracker.update_weights();
        }

        let weight = tracker.container_collisions[0].weight;
        assert_eq!(weight, GLS_WEIGHT_MAX);
        assert!(tracker.get_total_weighted_loss().is_finite());
    }
}
