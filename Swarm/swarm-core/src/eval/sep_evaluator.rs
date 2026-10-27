use crate::eval::sample_eval::{SampleEval, SampleEvaluator};
use crate::eval::specialized_jaguars_pipeline::{
    SpecializedHazardCollector, collect_poly_collisions_with_edge_order,
};
use crate::quantify::tracker::CollisionTracker;
use crate::util::bit_reversal_iterator::BitReversalIterator;
use jagua_rs::collision_detection::hazards::collector::HazardCollector;
use jagua_rs::collision_detection::hazards::{HazKey, BasicHazardEntity};
use jagua_rs::entities::Item;
use jagua_rs::entities::Layout;
use jagua_rs::entities::PItemKey;
use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::primitives::SPolygon;
use slotmap::SecondaryMap;
use std::cell::RefCell;

/// Worker-local owned buffers. No layout, tracker or placement references survive
/// a search, so the pool can also be reused after rollback and NFP fallback.
#[derive(Default)]
pub struct SeparationEvaluatorPool(RefCell<Option<SeparationBuffers>>);

struct SeparationBuffers {
    /// One buffer per part of the item being searched.
    shapes: Vec<SPolygon>,
    detected: SecondaryMap<HazKey, (BasicHazardEntity, usize)>,
    pending: Vec<HazKey>,
    touched: Vec<HazKey>,
    edge_orders: Vec<Vec<usize>>,
    #[cfg(feature = "simd")]
    poles: crate::quantify::simd::circles_soa::CirclesSoA,
}

pub struct PooledSeparationEvaluator<'a> {
    evaluator: Option<SeparationEvaluator<'a>>,
    pool: &'a SeparationEvaluatorPool,
}

impl<'a> PooledSeparationEvaluator<'a> {
    pub fn new(
        layout: &'a Layout,
        item: &'a Item,
        pk: PItemKey,
        ct: &'a CollisionTracker,
        pool: &'a SeparationEvaluatorPool,
    ) -> Self {
        let evaluator = match pool.0.borrow_mut().take() {
            None => SeparationEvaluator::new(layout, item, pk, ct),
            Some(buffers) => {
                // SPolygon::transform_from assumes matching buffer dimensions
                // and leaves invariant metadata untouched. Reset every field
                // when switching items, retaining the large vertex allocation.
                // jagua-rs 1.x keeps SPolygon fields private; clone_from resets every
                // field from the reference geometry and reuses the vertex allocation.
                let mut shapes = buffers.shapes;
                shapes.resize_with(item.n_parts(), || item.shape_cd().as_ref().clone());
                for (buffer, part) in shapes.iter_mut().zip(item.parts_cd()) {
                    buffer.clone_from(part.as_ref());
                }
                let mut edge_orders = buffers.edge_orders;
                edge_orders.resize_with(item.n_parts(), Vec::new);
                for (order, part) in edge_orders.iter_mut().zip(item.parts_cd()) {
                    order.clear();
                    order.extend(BitReversalIterator::new(part.n_vertices()));
                }
                let collector = SpecializedHazardCollector::new_reusing(
                    layout,
                    ct,
                    pk,
                    buffers.detected,
                    buffers.pending,
                    buffers.touched,
                );
                #[cfg(feature = "simd")]
                let collector = {
                    let mut collector = collector;
                    collector.poles_soa = buffers.poles;
                    collector
                };
                SeparationEvaluator {
                    layout,
                    item,
                    collector,
                    shape_buffs: shapes,
                    edge_orders,
                    n_evals: 0,
                }
            }
        };
        Self {
            evaluator: Some(evaluator),
            pool,
        }
    }
}

impl SampleEvaluator for PooledSeparationEvaluator<'_> {
    fn evaluate_sample(&mut self, dt: DTransformation, bound: Option<SampleEval>) -> SampleEval {
        self.evaluator.as_mut().unwrap().evaluate_sample(dt, bound)
    }
    fn n_evals(&self) -> usize {
        self.evaluator.as_ref().unwrap().n_evals()
    }
}

impl Drop for PooledSeparationEvaluator<'_> {
    fn drop(&mut self) {
        let evaluator = self.evaluator.take().unwrap();
        *self.pool.0.borrow_mut() = Some(SeparationBuffers {
            shapes: evaluator.shape_buffs,
            detected: evaluator.collector.detected,
            pending: evaluator.collector.pending,
            touched: evaluator.collector.touched,
            edge_orders: evaluator.edge_orders,
            #[cfg(feature = "simd")]
            poles: evaluator.collector.poles_soa,
        });
    }
}

pub struct SeparationEvaluator<'a> {
    layout: &'a Layout,
    item: &'a Item,
    collector: SpecializedHazardCollector<'a>,
    /// One buffer per part (a single entry for an ordinary item).
    shape_buffs: Vec<SPolygon>,
    // Fixed for this search's item; reused across candidate transforms/bounds.
    edge_orders: Vec<Vec<usize>>,
    n_evals: usize,
}

impl<'a> SeparationEvaluator<'a> {
    pub fn new(
        layout: &'a Layout,
        item: &'a Item,
        current_pk: PItemKey,
        ct: &'a CollisionTracker,
    ) -> Self {
        let collector = SpecializedHazardCollector::new(layout, ct, current_pk);

        Self {
            layout,
            item,
            collector,
            shape_buffs: item.parts_cd().iter().map(|part| part.as_ref().clone()).collect(),
            edge_orders: item
                .parts_cd()
                .iter()
                .map(|part| BitReversalIterator::new(part.n_vertices()).collect())
                .collect(),
            n_evals: 0,
        }
    }
}

impl<'a> SampleEvaluator for SeparationEvaluator<'a> {
    /// Evaluates a transformation. An upper bound can be provided to early terminate the process.
    /// Algorithm 7 from https://doi.org/10.48550/arXiv.2509.13329
    fn evaluate_sample(
        &mut self,
        dt: DTransformation,
        upper_bound: Option<SampleEval>,
    ) -> SampleEval {
        self.n_evals += 1;
        let cde = self.layout.cde();

        // Calculate an upper bound of quantification, above which samples are guaranteed to be rejected (because they are dominated by previous ones).
        let loss_bound = match upper_bound {
            Some(SampleEval::Collision { loss }) => loss,
            Some(SampleEval::Clear { .. }) => 0.0,
            _ => f32::INFINITY,
        };

        // Every part is collected and quantified on its own; the losses add up and
        // the remaining bound shrinks accordingly. An ordinary item runs one round.
        let mut total_loss = 0.0;
        let mut any_collision = false;
        for (part, (buffer, edge_order)) in self
            .item
            .parts_cd()
            .iter()
            .zip(self.shape_buffs.iter_mut().zip(&self.edge_orders))
        {
            // Reload the hazard collector to prepare for a new query
            self.collector.reload(loss_bound - total_loss);

            // Perform the collision detection and the collision quantification. The 'collector' will be populated with the results.
            collect_poly_collisions_with_edge_order(
                cde,
                &dt,
                buffer,
                part.as_ref(),
                &mut self.collector,
                edge_order.iter().copied(),
            );

            if self.collector.early_terminate(buffer) {
                // The total quantification of the collisions exceeded the upper bound and the process was terminated early.
                // Note that we might have exited before detecting/quantifying all collisions.
                // However, since we can asure that this sample will always be rejected, we don't need to spend any more time on it and just return `Invalid`.
                return SampleEval::Invalid;
            }
            if !self.collector.is_empty() {
                any_collision = true;
                total_loss += self.collector.loss(buffer);
            }
        }
        if !any_collision {
            // No collisions detected, return clear
            SampleEval::Clear { loss: 0.0 }
        } else {
            // Some collisions detected but withing the upper bound, return collision with total loss
            SampleEval::Collision { loss: total_loss }
        }
    }

    fn n_evals(&self) -> usize {
        self.n_evals
    }
}
