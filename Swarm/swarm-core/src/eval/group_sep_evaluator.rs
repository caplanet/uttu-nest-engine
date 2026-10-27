//! Exact separation evaluator for a rigid multi-item placement unit.

use crate::eval::sample_eval::{SampleEval, SampleEvaluator};
use crate::optimizer::place_groups::PlaceGroups;
use crate::quantify::tracker::CollisionTracker;
use crate::quantify::{
    quantify_collision_poly_container_with_penalty, quantify_collision_poly_poly_with_penalty,
};
use jagua_rs::probs::spp::entities::SPInstance;
use jagua_rs::collision_detection::hazards::collector::{BasicHazardCollector, HazardCollector};
use jagua_rs::collision_detection::hazards::filter::NoFilter;
use jagua_rs::collision_detection::hazards::{HazKey, BasicHazardEntity};
use jagua_rs::entities::{Layout, PItemKey};
use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::geo_traits::TransformableFrom;
use jagua_rs::geometry::primitives::SPolygon;

/// Evaluates one anchor transform by applying its world-space delta to every
/// member and summing the members' weighted external collision losses.
pub struct GroupSeparationEvaluator<'a> {
    layout: &'a Layout,
    instance: &'a SPInstance,
    groups: &'a PlaceGroups,
    ct: &'a CollisionTracker,
    anchor_item_id: usize,
    member_keys: Vec<(usize, PItemKey, HazKey)>,
    shape_buffers: Vec<(usize, SPolygon)>,
    // Fixed for this anchor; built once so evaluate_sample does not reallocate.
    ignored_keys: Vec<HazKey>,
    collector: BasicHazardCollector,
    n_evals: usize,
}

impl<'a> GroupSeparationEvaluator<'a> {
    pub fn new(
        layout: &'a Layout,
        instance: &'a SPInstance,
        groups: &'a PlaceGroups,
        anchor_item_id: usize,
        ct: &'a CollisionTracker,
    ) -> Self {
        let member_keys: Vec<(usize, PItemKey, HazKey)> = groups
            .member_item_ids(anchor_item_id)
            .into_iter()
            .map(|item_id| {
                let pk = groups
                    .placement_key(layout, item_id)
                    .expect(crate::dmsg!("validated group member must be present"));
                let haz_key = crate::util::haz_key(layout, pk)
                    .expect(crate::dmsg!("placed group member must be registered in CDE"));
                (item_id, pk, haz_key)
            })
            .collect();
        let ignored_keys = member_keys.iter().map(|(_, _, hk)| *hk).collect();
        let collector = BasicHazardCollector::with_capacity(layout.placed_items().len() + 1);
        let shape_buffers = groups
            .member_item_ids(anchor_item_id)
            .into_iter()
            .map(|item_id| (item_id, instance.item(item_id).shape_cd().as_ref().clone()))
            .collect();
        Self {
            layout,
            instance,
            groups,
            ct,
            anchor_item_id,
            member_keys,
            shape_buffers,
            ignored_keys,
            collector,
            n_evals: 0,
        }
    }

    fn weighted_loss(
        layout: &Layout,
        ct: &CollisionTracker,
        current_pk: PItemKey,
        shape: &SPolygon,
        hazard: &BasicHazardEntity,
    ) -> f32 {
        match hazard {
            BasicHazardEntity::PlacedItem { pk: other_pk, .. } => {
                let other_shape = &layout.placed_items()[*other_pk].shape();
                quantify_collision_poly_poly_with_penalty(
                    other_shape,
                    shape,
                    ct.shape_penalty(*other_pk, other_shape, current_pk, shape),
                ) * ct.get_pair_weight(current_pk, *other_pk)
            }
            BasicHazardEntity::Exterior => {
                quantify_collision_poly_container_with_penalty(
                    shape,
                    layout.container().outer_cd().bbox(),
                    ct.shape_penalty(current_pk, shape, current_pk, shape),
                ) * ct.get_container_weight(current_pk)
            }
            _ => unimplemented!("unsupported hazard entity in rigid group evaluator"),
        }
    }
}

impl SampleEvaluator for GroupSeparationEvaluator<'_> {
    fn evaluate_sample(
        &mut self,
        dt: DTransformation,
        upper_bound: Option<SampleEval>,
    ) -> SampleEval {
        self.n_evals += 1;
        let loss_bound = match upper_bound {
            Some(SampleEval::Collision { loss }) => loss,
            Some(SampleEval::Clear { .. }) => 0.0,
            _ => f32::INFINITY,
        };
        let targets = self
            .groups
            .target_transforms(Some(self.layout), self.anchor_item_id, dt)
            .expect(crate::dmsg!("validated place group became incomplete"));
        let mut total_loss = 0.0;

        for (item_id, target) in targets {
            let current_pk = self
                .member_keys
                .iter()
                .find(|(id, _, _)| *id == item_id)
                .map(|(_, pk, _)| *pk)
                .expect(crate::dmsg!("member key must exist"));
            let item = self.instance.item(item_id);
            let (_, shape) = self
                .shape_buffers
                .iter_mut()
                .find(|(id, _)| *id == item_id)
                .expect(crate::dmsg!("shape buffer must exist"));
            shape.transform_from(&item.shape_cd(), &target.compose());

            // The sampler keeps only the representative inside the container;
            // a rigidly attached member can land entirely off the material.
            // Such a pose is never useful, and placing a shape wholly outside
            // the quadtree root breaks a jagua-rs invariant (its debug check
            // `constricted_hazards ... > 0`). Reject it outright.
            let container = self.layout.container().outer_cd().bbox();
            let b = shape.bbox();
            if b.x_min >= container.x_max
                || b.x_max <= container.x_min
                || b.y_min >= container.y_max
                || b.y_max <= container.y_min
            {
                return SampleEval::Invalid;
            }

            // Preloading every current group hazard makes the collector act as
            // a filter for the old member positions. They are deliberately not
            // part of the returned loss; internal overlaps are user-defined
            // and remain invariant under a rigid delta.
            let collector = &mut self.collector;
            collector.clear();
            for &(_, pk, hkey) in &self.member_keys {
                let pi = &self.layout.placed_items()[pk];
                HazardCollector::insert(collector, hkey, (pk, pi).into());
            }
            self.layout.cde().collect_poly_collisions(shape, &NoFilter, collector);
            let ignored_keys = &self.ignored_keys;
            total_loss += collector
                .iter()
                .filter(|(hkey, _)| !ignored_keys.contains(hkey))
                .map(|(_, hazard)| {
                    Self::weighted_loss(self.layout, self.ct, current_pk, shape, hazard)
                })
                .sum::<f32>();
            if total_loss > loss_bound {
                return SampleEval::Invalid;
            }
        }

        if total_loss == 0.0 {
            SampleEval::Clear { loss: 0.0 }
        } else {
            SampleEval::Collision { loss: total_loss }
        }
    }

    fn n_evals(&self) -> usize {
        self.n_evals
    }
}
