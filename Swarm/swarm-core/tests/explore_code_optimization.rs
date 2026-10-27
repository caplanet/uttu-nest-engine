use jagua_rs::geometry::DTransformation;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem};
use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
use rand::{SeedableRng, rngs::Xoshiro256PlusPlus};
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::eval::sample_eval::{SampleEval, SampleEvaluator};
use swarm_core::eval::sep_evaluator::{
    PooledSeparationEvaluator, SeparationEvaluator, SeparationEvaluatorPool,
};
use swarm_core::optimizer::separator::Separator;
use swarm_core::quantify::tracker::CollisionTracker;
use swarm_core::util::assertions::tracker_matches_layout;

fn fixture() -> (SPInstance, SPProblem) {
    let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
        "name":"explore-code-regression", "strip_height":30.0,
        "items":[
            {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{"type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":8.0,"height":8.0}}},
            {"id":1,"demand":1,"allowed_orientations":[0.0],"shape":{"type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":7.0,"height":5.0}}},
            {"id":2,"demand":1,"allowed_orientations":[0.0],"shape":{"type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":3.0}}}
        ]})).unwrap();
    let cfg = DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(cfg.cde_config, cfg.poly_simpl_tolerance, cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
    let instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext).unwrap();
    let mut prob = SPProblem::new(instance.clone()).expect("strip forms a valid container");
    prob.change_strip_width(40.0).expect("strip width must be positive");
    for id in 0..3 {
        prob.place_item(SPPlacement {
            item_idx: id,
            d_transf: DTransformation::new(0.0, (10.0 + id as f32 * 2.0, 10.0)),
        });
    }
    (instance, prob)
}

fn assert_sums(ct: &CollisionTracker) {
    let raw = ct
        .container_collisions()
        .iter()
        .map(|e| e.loss)
        .sum::<f32>()
        + ct.pair_collisions()
            .data
            .iter()
            .map(|e| e.loss)
            .sum::<f32>();
    let weighted = ct
        .container_collisions()
        .iter()
        .map(|e| e.weighted_loss())
        .sum::<f32>()
        + ct.pair_collisions()
            .data
            .iter()
            .map(|e| e.weighted_loss())
            .sum::<f32>();
    for _ in 0..2 {
        assert_eq!(ct.get_total_loss().to_bits(), raw.to_bits());
        assert_eq!(ct.get_total_weighted_loss().to_bits(), weighted.to_bits());
        for (pk, &idx) in &ct.pk_idx_map {
            let row = ct.container_collisions()[idx].loss
                + (0..ct.size)
                    .map(|i| ct.pair_collisions()[(idx, i)].loss)
                    .sum::<f32>();
            let wrow = ct.container_collisions()[idx].weighted_loss()
                + (0..ct.size)
                    .map(|i| ct.pair_collisions()[(idx, i)].weighted_loss())
                    .sum::<f32>();
            assert_eq!(ct.get_loss(pk).to_bits(), row.to_bits());
            assert_eq!(ct.get_weighted_loss(pk).to_bits(), wrow.to_bits());
        }
    }
}

#[test]
fn exact_sum_cache_survives_moves_weight_updates_copy_and_rollback() {
    let (instance, prob) = fixture();
    let mut sep = Separator::new(
        instance,
        prob,
        Xoshiro256PlusPlus::seed_from_u64(17),
        DEFAULT_SWARM_CONFIG.expl_cfg.separator_config,
    );
    let original = sep.prob.save();
    let snapshot = sep.ct.save();
    assert_sums(&sep.ct);
    for i in 0..8 {
        let pk = sep.prob.layout().placed_items().keys().next().unwrap();
        sep.move_item(pk, DTransformation::new(0.0, (5.0 + i as f32 * 2.0, 10.0)));
        assert_sums(&sep.ct);
        sep.ct.update_weights();
        assert_sums(&sep.ct);
        let mut copied = snapshot.clone();
        copied.copy_from(&sep.ct);
        assert_sums(&copied);
    }
    sep.ct.pair_collisions_mut()[(0, 1)].weight = 7.3;
    assert_sums(&sep.ct);
    sep.ct.container_collisions_mut()[0].weight = 2.5;
    assert_sums(&sep.ct);
    sep.rollback(&original, Some(&snapshot));
    assert_sums(&sep.ct);
    assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));
    sep.freeze_items(&[0, 1]);
    assert_sums(&sep.ct);
    sep.ct.inherit_weights_from(&snapshot);
    assert_sums(&sep.ct);
    sep.rollback(&original, None);
    assert_sums(&sep.ct);
}

fn signature(eval: SampleEval) -> (u8, u32) {
    match eval {
        SampleEval::Invalid => (0, 0),
        SampleEval::Clear { loss } => (1, loss.to_bits()),
        SampleEval::Collision { loss } => (2, loss.to_bits()),
    }
}

#[test]
fn pooled_evaluator_matches_fresh_across_items_and_layout_changes() {
    let (instance, mut prob) = fixture();
    let pool = SeparationEvaluatorPool::default();
    for round in 0..3 {
        let ct = CollisionTracker::new(&prob.layout());
        for (pk, pi) in prob.layout().placed_items() {
            let item = instance.item(pi.item().idx());
            let mut fresh = SeparationEvaluator::new(&prob.layout(), item, pk, &ct);
            let mut pooled = PooledSeparationEvaluator::new(&prob.layout(), item, pk, &ct, &pool);
            for x in 0..40 {
                let dt = DTransformation::new(0.0, (x as f32 + 0.125, 10.0));
                let full = fresh.evaluate_sample(dt, None);
                assert_eq!(signature(full), signature(pooled.evaluate_sample(dt, None)));
                for bound in [0.0, 0.125, 10.0, 100.0, f32::INFINITY] {
                    let upper = Some(SampleEval::Collision { loss: bound });
                    assert_eq!(
                        signature(fresh.evaluate_sample(dt, upper)),
                        signature(pooled.evaluate_sample(dt, upper))
                    );
                }
            }
        }
        let pk = prob.layout().placed_items().keys().next().unwrap();
        let mut placement = prob.remove_item(pk);
        placement.d_transf = DTransformation::new(0.0, (22.0 + round as f32, 20.0));
        prob.place_item(placement);
    }
}

#[test]
fn pending_hazards_preserve_map_order_and_removal_contract() {
    use jagua_rs::collision_detection::hazards::{BasicHazardEntity, collector::HazardCollector};
    use swarm_core::eval::specialized_jaguars_pipeline::SpecializedHazardCollector;
    let (_, prob) = fixture();
    let ct = CollisionTracker::new(&prob.layout());
    let pk = prob.layout().placed_items().keys().next().unwrap();
    let shape = &prob.layout().placed_items()[pk].shape();
    let mut collector = SpecializedHazardCollector::new(&prob.layout(), &ct, pk);
    #[cfg(feature = "simd")]
    collector.poles_soa.load(&shape.surrogate().poles);
    let mut hazards: Vec<_> = prob
        .layout()
        .cde()
        .hazards_map()
        .iter()
        .filter(|(key, _)| *key != collector.current_haz_key)
        .map(|(key, haz)| (key, haz.entity))
        .collect();
    hazards.reverse();
    let reference = |collector: &SpecializedHazardCollector| -> f32 {
        collector
            .detected
            .values()
            .map(|(haz, _)| match haz {
                BasicHazardEntity::PlacedItem { pk: other, .. } => {
                    let other_shape = &prob.layout().placed_items()[*other].shape();
                    #[cfg(feature = "simd")]
                    let loss = swarm_core::quantify::simd::quantify_collision_poly_poly_simd(
                        other_shape,
                        shape,
                        &collector.poles_soa,
                    );
                    #[cfg(not(feature = "simd"))]
                    let loss =
                        swarm_core::quantify::quantify_collision_poly_poly(other_shape, shape);
                    loss * ct.get_pair_weight(pk, *other)
                }
                BasicHazardEntity::Exterior => {
                    swarm_core::quantify::quantify_collision_poly_container(
                        shape,
                        prob.layout().container().outer_cd().bbox(),
                    ) * ct.get_container_weight(pk)
                }
                _ => unreachable!(),
            })
            .sum()
    };
    // Reverse insertion order must still use the original slot order for sums.
    for &(key, hazard) in &hazards {
        collector.insert(key, hazard);
    }
    assert_eq!(
        collector.loss(shape).to_bits(),
        reference(&collector).to_bits()
    );
    let (removed, entity) = hazards[0];
    collector.remove_by_key(removed); // removal of a previously cached entry
    assert_eq!(
        collector.loss(shape).to_bits(),
        reference(&collector).to_bits()
    );
    collector.reload(f32::INFINITY);
    for &(key, hazard) in &hazards {
        collector.insert(key, hazard);
    }
    collector.remove_by_key(removed); // removal before the pending batch is read
    assert_eq!(
        collector.loss(shape).to_bits(),
        reference(&collector).to_bits()
    );
    collector.insert(removed, entity);
    assert!((collector.loss(shape) - reference(&collector)).abs() <= 0.001);
    collector.reload(0.0);
    for &(key, hazard) in &hazards {
        collector.insert(key, hazard);
    }
    assert!(collector.early_terminate(shape));
    // A rejected partial evaluation must not survive removal or the next query.
    for &(key, _) in &hazards {
        collector.remove_by_key(key);
    }
    assert_eq!(collector.loss(shape), 0.0);
    collector.reload(f32::INFINITY);
    assert_eq!(collector.loss(shape), 0.0);
}
