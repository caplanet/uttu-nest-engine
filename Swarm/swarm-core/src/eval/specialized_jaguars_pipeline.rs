use crate::quantify::quantify_collision_poly_container_with_penalty;
#[cfg(not(feature = "simd"))]
use crate::quantify::quantify_collision_poly_poly_with_penalty;
#[cfg(feature = "simd")]
use crate::quantify::simd::circles_soa::CirclesSoA;
#[cfg(feature = "simd")]
use crate::quantify::simd::quantify_collision_poly_poly_simd_bounded_by_with_penalty;
#[cfg(feature = "simd")]
use crate::quantify::simd::quantify_collision_poly_poly_simd_with_penalty;
use crate::quantify::tracker::CollisionTracker;
use crate::util::assertions;
use crate::util::bit_reversal_iterator::BitReversalIterator;
use float_cmp::approx_eq;
use jagua_rs::collision_detection::CDEngine;
use jagua_rs::collision_detection::hazards::filter::NoFilter;
use jagua_rs::collision_detection::hazards::collector::HazardCollector;
use jagua_rs::collision_detection::hazards::{HazKey, BasicHazardEntity};
use jagua_rs::collision_detection::quadtree::QTHazPresence;
use jagua_rs::entities::Layout;
use jagua_rs::entities::PItemKey;
use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::geo_traits::TransformableFrom;
use jagua_rs::geometry::primitives::SPolygon;
use slotmap::{Key, SecondaryMap};
use std::f32::consts::PI;

/// Our specialized counterpart to [`CDEngine::collect_poly_collisions`] with early return:
/// collection stops as soon as the loss exceeds the detector's `loss_bound`. Over 90% of the
/// run is spent here, so the specialization pays for itself.
///
/// jagua-rs 0.8 gained a native early-return variant (`collect_poly_collisions_until`), but we
/// keep this one: it also carries the SIMD pole check, the incremental loss cache and the
/// bit-reversal edge order, none of which the native path provides.
///
/// The invariant is **not** set equality with the native pipeline but `native ⊆ custom` —
/// we may report extra touching hazards (quantified loss ~0) but never miss a real collision.
/// See `util::assertions::custom_pipeline_matches_jaguars`. Unlike the native path we have no
/// bbox-based Exterior shortcut; `cde.detect_containment_collision` below covers that case.
pub fn collect_poly_collisions_in_detector_custom(
    cde: &CDEngine,
    dt: &DTransformation,
    shape_buffer: &mut SPolygon,
    reference_shape: &SPolygon,
    collector: &mut SpecializedHazardCollector,
) {
    collect_poly_collisions_with_edge_order(
        cde,
        dt,
        shape_buffer,
        reference_shape,
        collector,
        BitReversalIterator::new(reference_shape.n_vertices()),
    );
}

/// Same collision pipeline with an item-specific traversal prepared by the
/// evaluator. The caller must supply the complete original bit-reversal order.
/// Generic iteration keeps the cached path free of per-edge mode branches.
pub(crate) fn collect_poly_collisions_with_edge_order(
    cde: &CDEngine,
    dt: &DTransformation,
    shape_buffer: &mut SPolygon,
    reference_shape: &SPolygon,
    collector: &mut SpecializedHazardCollector,
    edge_order: impl Iterator<Item = usize>,
) {
    let t = dt.compose();
    // transform the shape buffer to the new position
    let shape = shape_buffer.transform_from(reference_shape, &t);

    #[cfg(feature = "simd")]
    collector.poles_soa.load(&shape.surrogate().poles);

    {
        // We start off by checking a few poles in order to detect obvious collisions quickly and quickly raise the loss.
        // Potentially allows us to fail fast (early terminate) without checking all edges.
        // We check poles until the area of the poles checked exceeds 50% of the shape.
        let area_threshold = shape.area() * 0.5 / PI;
        let mut area_sum = 0.0;
        for pole in shape.surrogate().poles.iter() {
            let _ = cde.quadtree.collect_collisions_until(pole, &NoFilter, collector, &mut |_| false);
            if collector.early_terminate(shape) {
                return;
            }
            area_sum += pole.radius * pole.radius;
            if area_sum > area_threshold {
                break;
            }
        }
    }

    // Find the virtual root of the quadtree for the shape's bounding box. So we do not have to start from the root every time.
    let v_quadtree = cde.get_virtual_root(shape.bbox());

    // Collect collisions for each edge of the polygon.
    // 비트 반전 순서로 훑어 새 hazard 를 최대한 일찍 만난다 — 조기 종료가 그만큼 빨리 걸린다.
    //
    // 실측(wsjk156, 120s+30s, 3시드)으로 확정된 우리 자산이다. 상류·jagua 네이티브가 쓰는
    // 인덱스 순서로 바꾸면 최종 밀도 78.27% → 77.80% (−0.47%p, ±0.4%p 밴드 밖)이고
    // 탐색 개선 횟수가 50.0 → 46.7 회로 줄었다. 즉 밀도가 아니라 **처리량**이 먼저 깎인다.
    // 네이티브 파이프라인으로 갈아타려면 이 순서를 함께 들고 가야 한다.
    for i in edge_order {
        let edge = shape.edge(i);
        let _ = v_quadtree.collect_collisions_until(&edge, &NoFilter, collector, &mut |_| false);
        if collector.early_terminate(shape) {
            return;
        }
    }

    // Check if there are any other collisions due to containment
    for qt_haz in v_quadtree.hazards.iter() {
        match &qt_haz.presence {
            // No need to check these, guaranteed to be detected by edge intersection
            QTHazPresence::None | QTHazPresence::Entire => {}
            QTHazPresence::Partial(_) => {
                if !collector.contains_key(qt_haz.hkey) {
                    let h_shape = &cde.hazards_map()[qt_haz.hkey].shape;
                    if CDEngine::<BasicHazardEntity>::detect_containment_collision(shape, h_shape, qt_haz.entity) {
                        collector.insert(qt_haz.hkey, qt_haz.entity);
                        if collector.early_terminate(shape) {
                            return;
                        }
                    }
                }
            }
        }
    }

    // At this point, all collisions should be present in the detector.
    debug_assert!(
        assertions::custom_pipeline_matches_jaguars(shape, collector),
        "Custom pipeline deviates from native jagua-rs pipeline"
    );
}

/// Specialized version of [`HazardCollector`]
/// This struct computes the loss incrementally on the fly and caches the result.
/// Allows for early termination if the loss exceeds a certain upperbound.
pub struct SpecializedHazardCollector<'a> {
    pub layout: &'a Layout,
    pub ct: &'a CollisionTracker,
    pub current_pk: PItemKey,
    pub current_haz_key: HazKey,
    /// Hazard keys of the moving item's other parts (empty for an ordinary item).
    pub(crate) own_keys: Vec<HazKey>,
    /// The moving item's envelope; it carries the shape factor for every part.
    pub current_hull: &'a SPolygon,
    pub detected: SecondaryMap<HazKey, (BasicHazardEntity, usize)>,
    pub idx_counter: usize,
    pub loss_cache: (usize, f32),
    pub loss_bound: f32,
    pub(crate) pending: Vec<HazKey>,
    pub(crate) touched: Vec<HazKey>,
    rejected: bool,
    #[cfg(feature = "simd")]
    pub poles_soa: CirclesSoA,
}

fn clear_touched_hazards(
    detected: &mut SecondaryMap<HazKey, (BasicHazardEntity, usize)>,
    touched: &mut Vec<HazKey>,
) {
    // Keys include their generation. Removed/reinserted hazards can leave an
    // older key in this list, which must not remove a new generation's value.
    for key in touched.drain(..) {
        detected.remove(key);
    }
    debug_assert!(detected.is_empty());
}

impl<'a> SpecializedHazardCollector<'a> {
    pub fn new(layout: &'a Layout, ct: &'a CollisionTracker, current_pk: PItemKey) -> Self {
        Self::new_reusing(
            layout,
            ct,
            current_pk,
            SecondaryMap::with_capacity(layout.placed_items().len() + 1),
            Vec::new(),
            Vec::new(),
        )
    }

    pub(crate) fn new_reusing(
        layout: &'a Layout,
        ct: &'a CollisionTracker,
        current_pk: PItemKey,
        mut detected: SecondaryMap<HazKey, (BasicHazardEntity, usize)>,
        mut pending: Vec<HazKey>,
        mut touched: Vec<HazKey>,
    ) -> Self {
        clear_touched_hazards(&mut detected, &mut touched);
        pending.clear();
        let mut own_keys = layout.haz_keys(current_pk);
        assert!(!own_keys.is_empty(), "placed item should be registered in the CDE");
        let current_haz_key = own_keys.remove(0);
        let current_hull = layout.placed_items()[current_pk].item().shape_cd().as_ref();
        Self {
            layout,
            ct,
            current_pk,
            current_haz_key,
            own_keys,
            current_hull,
            detected,
            idx_counter: 0,
            loss_cache: (0, 0.0),
            loss_bound: f32::INFINITY,
            pending,
            touched,
            rejected: false,
            #[cfg(feature = "simd")]
            poles_soa: CirclesSoA::new(),
        }
    }

    pub fn reload(&mut self, loss_bound: f32) {
        clear_touched_hazards(&mut self.detected, &mut self.touched);
        self.idx_counter = 0;
        self.loss_cache = (0, 0.0);
        self.loss_bound = loss_bound;
        self.pending.clear();
        self.rejected = false;
    }

    pub fn iter_with_index(&self) -> impl Iterator<Item = &(BasicHazardEntity, usize)> {
        self.detected.values()
    }

    pub fn early_terminate(&mut self, shape: &SPolygon) -> bool {
        self.loss(shape) > self.loss_bound
    }

    pub fn loss(&mut self, shape: &SPolygon) -> f32 {
        if self.rejected {
            return f32::INFINITY;
        }
        let (cache_idx, cached_loss) = self.loss_cache;
        if cache_idx < self.idx_counter {
            // SecondaryMap iterates by slot index, not insertion order. Keep
            // that exact order within each batch, without rescanning old slots.
            self.pending
                .sort_unstable_by_key(|key| key.data().as_ffi() as u32);
            let mut extra_loss = 0.0;
            for key in &self.pending {
                let (hazard, _) = &self.detected[*key];
                #[cfg(feature = "simd")]
                if let BasicHazardEntity::PlacedItem { pk: other_pk, .. } = hazard {
                    let weight = self.ct.get_pair_weight(self.current_pk, *other_pk);
                    if self.loss_bound.is_finite() && weight.is_finite() && weight > 0.0 {
                        let hint = (self.loss_bound - cached_loss - extra_loss) / weight;
                        let other = &self.layout.placed_items()[*other_pk];
                        let other_shape = match hazard {
                            BasicHazardEntity::PlacedItem { part, .. } => &other.shapes()[*part],
                            _ => unreachable!(),
                        };
                        let loss = quantify_collision_poly_poly_simd_bounded_by_with_penalty(
                            other_shape,
                            shape,
                            &self.poles_soa,
                            self.ct.shape_penalty(
                                *other_pk,
                                &other.shape(),
                                self.current_pk,
                                self.current_hull,
                            ),
                            hint,
                            |loss| cached_loss + (extra_loss + loss * weight) > self.loss_bound,
                        );
                        let Some(loss) = loss else {
                            // Only rejection is observable for an over-bound
                            // sample; never cache a partial sum as an exact loss.
                            self.rejected = true;
                            return f32::INFINITY;
                        };
                        extra_loss += loss * weight;
                        continue;
                    }
                }
                extra_loss += self.calc_weighted_loss(hazard, shape);
                // 상한을 넘긴 표본은 기각만 관측된다 — 남은 쌍은 볼 필요가 없다.
                // (위의 SIMD 경로는 `rejects` 가 같은 기준으로 이미 기각했다.)
                if cached_loss + extra_loss > self.loss_bound {
                    self.rejected = true;
                    return f32::INFINITY;
                }
            }
            self.pending.clear();
            self.loss_cache = (self.idx_counter, cached_loss + extra_loss);
        }
        debug_assert!(approx_eq!(
            f32,
            self.loss_cache.1,
            self.iter()
                .map(|(_, he)| self.calc_weighted_loss(he, shape))
                .sum()
        ));
        self.loss_cache.1
    }

    fn calc_weighted_loss(&self, haz: &BasicHazardEntity, shape: &SPolygon) -> f32 {
        match haz {
            BasicHazardEntity::PlacedItem {
                pk: other_pk,
                part: other_part,
                ..
            } => {
                let other = &self.layout.placed_items()[*other_pk];
                let other_shape = &other.shapes()[*other_part];
                let penalty = self.ct.shape_penalty(
                    *other_pk,
                    &other.shape(),
                    self.current_pk,
                    self.current_hull,
                );

                #[cfg(not(feature = "simd"))]
                let loss = quantify_collision_poly_poly_with_penalty(other_shape, shape, penalty);
                #[cfg(feature = "simd")]
                let loss = quantify_collision_poly_poly_simd_with_penalty(
                    other_shape,
                    shape,
                    &self.poles_soa,
                    penalty,
                );

                let weight = self.ct.get_pair_weight(self.current_pk, *other_pk);
                loss * weight
            }
            BasicHazardEntity::Exterior => {
                let loss = quantify_collision_poly_container_with_penalty(
                    shape,
                    self.layout.container().outer_cd().bbox(),
                    self.ct.shape_penalty(
                        self.current_pk,
                        self.current_hull,
                        self.current_pk,
                        self.current_hull,
                    ),
                );
                let weight = self.ct.get_container_weight(self.current_pk);
                loss * weight
            }
            _ => unimplemented!("unsupported hazard entity"),
        }
    }
}

impl<'a> HazardCollector for SpecializedHazardCollector<'a> {
    type Entity = BasicHazardEntity;

    fn contains_key(&self, hkey: HazKey) -> bool {
        // 이동 중인 조각 자신의 hazard 는 순회 중 거의 모든 노드에서 마주친다.
        // `insert` 의 debug_assert 가 그것을 `detected` 에 넣지 못하게 막으므로 두 항은
        // 상호배타적이고, 싼 쪽(레지스터 비교)을 먼저 두면 그때마다 SecondaryMap
        // 조회 한 번이 절약된다. 결과는 비트 단위로 동일하다.
        hkey == self.current_haz_key
            || self.own_keys.contains(&hkey)
            || self.detected.contains_key(hkey)
    }

    fn insert(&mut self, hkey: HazKey, entity: BasicHazardEntity) {
        debug_assert!(!self.contains_key(hkey));
        self.detected.insert(hkey, (entity, self.idx_counter));
        self.touched.push(hkey);
        self.pending.push(hkey);
        self.idx_counter += 1;
    }

    fn remove_by_key(&mut self, hkey: HazKey) {
        let (_, idx) = self
            .detected
            .remove(hkey)
            .expect(crate::dmsg!("key should be present in the collector"));
        self.pending.retain(|key| *key != hkey);
        if idx < self.loss_cache.0 || self.rejected {
            //wipe the cache if a hazard was removed that was in it
            self.loss_cache = (0, 0.0);
            self.rejected = false;
            self.pending.clear();
            self.pending.extend(self.detected.keys());
        }
    }

    fn is_empty(&self) -> bool {
        self.detected.is_empty()
    }

    fn len(&self) -> usize {
        self.detected.len()
    }

    fn iter(&self) -> impl Iterator<Item = (HazKey, &BasicHazardEntity)> {
        self.detected.iter().map(|(k, (h, _))| (k, h))
    }
}

#[cfg(test)]
mod sparse_reset_tests {
    use super::*;
    use slotmap::SlotMap;

    #[test]
    fn sparse_reset_handles_removed_and_reused_slot_generations() {
        let mut keys: SlotMap<HazKey, ()> = SlotMap::with_key();
        let allocated: Vec<_> = (0..4096).map(|_| keys.insert(())).collect();
        let first = allocated[3];
        let last = allocated[4095];
        let mut detected = SecondaryMap::new();
        let mut touched = vec![first, last];
        detected.insert(first, (BasicHazardEntity::Exterior, 0));
        detected.insert(last, (BasicHazardEntity::Exterior, 1));
        // A collector removal followed by slot reuse must not leave stale data.
        detected.remove(first);
        keys.remove(first);
        let replacement = keys.insert(());
        assert_ne!(replacement, first);
        detected.insert(replacement, (BasicHazardEntity::Exterior, 2));
        touched.push(replacement);
        clear_touched_hazards(&mut detected, &mut touched);
        assert!(detected.is_empty());
        assert!(touched.is_empty());
        // Repeated empty reset and a new search retain valid generation semantics.
        clear_touched_hazards(&mut detected, &mut touched);
        detected.insert(last, (BasicHazardEntity::Exterior, 0));
        touched.push(last);
        assert!(!detected.contains_key(first));
        clear_touched_hazards(&mut detected, &mut touched);
        assert!(detected.is_empty());
    }
}
