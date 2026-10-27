use jagua_rs::collision_detection::hazards::collector::HazardCollector;
use jagua_rs::entities::{Item, Layout, PItemKey};
use jagua_rs::geometry::DTransformation;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem};
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::eval::sample_eval::{SampleEval, SampleEvaluator};
use swarm_core::eval::sep_evaluator::{
    PooledSeparationEvaluator, SeparationEvaluator, SeparationEvaluatorPool,
};
use swarm_core::eval::specialized_jaguars_pipeline::{
    SpecializedHazardCollector, collect_poly_collisions_in_detector_custom,
};
use swarm_core::quantify::tracker::CollisionTracker;

fn fixture() -> (SPInstance, SPProblem) {
    // Different edge counts catch reuse of the previous item's traversal.
    let polygons = [
        vec![[0., 0.], [10., 0.], [0., 9.]],
        vec![
            [0., 0.],
            [12., 0.],
            [12., 3.],
            [4., 3.],
            [4., 8.],
            [12., 8.],
            [12., 12.],
            [0., 12.],
        ],
        vec![[0., 0.], [9., 0.], [11., 5.], [5., 11.], [0., 6.]],
    ];
    let ext = serde_json::from_value(serde_json::json!({
        "name":"edge-order-regression", "strip_height":40.0,
        "items":polygons.iter().enumerate().map(|(id,p)| serde_json::json!({
            "id":id,"demand":1,"allowed_orientations":[0.,90.,180.,270.],
            "shape":{"type":"simple_polygon","data":p}
        })).collect::<Vec<_>>()
    }))
    .unwrap();
    let importer = swarm_core::util::io::new_importer(DEFAULT_SWARM_CONFIG.cde_config, None, None, None);
    let instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext).unwrap();
    let mut prob = SPProblem::new(instance.clone()).expect("strip forms a valid container");
    prob.change_strip_width(45.).expect("strip width must be positive");
    for id in 0..3 {
        prob.place_item(SPPlacement {
            item_idx: id,
            d_transf: DTransformation::new(0., (12. + id as f32 * 3., 14.)),
        });
    }
    (instance, prob)
}

fn signature(eval: SampleEval) -> (u8, u32) {
    match eval {
        SampleEval::Invalid => (0, 0),
        SampleEval::Clear { loss } => (1, loss.to_bits()),
        SampleEval::Collision { loss } => (2, loss.to_bits()),
    }
}

fn uncached(
    layout: &Layout,
    item: &Item,
    pk: PItemKey,
    ct: &CollisionTracker,
    dt: DTransformation,
    bound: f32,
) -> SampleEval {
    let mut shape = item.shape_cd().as_ref().clone();
    let mut collector = SpecializedHazardCollector::new(layout, ct, pk);
    collector.reload(bound);
    // Public one-shot API retains the original on-demand bit-reversal iterator.
    collect_poly_collisions_in_detector_custom(
        layout.cde(),
        &dt,
        &mut shape,
        item.shape_cd().as_ref(),
        &mut collector,
    );
    if collector.early_terminate(&shape) {
        SampleEval::Invalid
    } else if collector.is_empty() {
        SampleEval::Clear { loss: 0. }
    } else {
        SampleEval::Collision {
            loss: collector.loss(&shape),
        }
    }
}

#[test]
fn cached_search_matches_uncached_for_changed_shapes_rotations_bounds_and_layouts() {
    let (instance, mut prob) = fixture();
    let original = prob.save();
    let pool = SeparationEvaluatorPool::default();
    let mut observed = [false; 3];
    for round in 0..3 {
        if round == 1 {
            prob.change_strip_width(31.).expect("strip width must be positive");
        }
        if round == 2 {
            prob.restore(&original);
        }
        let ct = CollisionTracker::new(&prob.layout());
        for (pk, pi) in prob.layout().placed_items() {
            let item = instance.item(pi.item().idx());
            let mut fresh = SeparationEvaluator::new(&prob.layout(), item, pk, &ct);
            let mut pooled = PooledSeparationEvaluator::new(&prob.layout(), item, pk, &ct, &pool);
            for angle in [0., 0.37, std::f32::consts::FRAC_PI_2] {
                for x in [-12., 0., 6., 11., 15., 20., 28., 39., 48.] {
                    for y in [-8., 5., 14., 22., 35., 43.] {
                        let dt = DTransformation::new(angle, (x, y));
                        for bound in [0., 1., 100., 10_000., f32::INFINITY] {
                            let expected =
                                signature(uncached(&prob.layout(), item, pk, &ct, dt, bound));
                            observed[expected.0 as usize] = true;
                            let upper = Some(SampleEval::Collision { loss: bound });
                            assert_eq!(signature(fresh.evaluate_sample(dt, upper)), expected);
                            assert_eq!(
                                signature(pooled.evaluate_sample(dt, upper)),
                                expected,
                                "round={round}, item={}, angle={angle}, x={x}, y={y}, bound={bound}",
                                pi.item().idx()
                            );
                        }
                    }
                }
            }
        }
    }
    assert_eq!(
        observed, [true; 3],
        "exercise rejected, clear and colliding candidates"
    );
}
