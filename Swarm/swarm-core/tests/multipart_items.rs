//! Multipart items (jagua-rs fork change 10): one item made of several disjoint
//! polygons that move as a rigid body. The engine registers a hazard per part,
//! so the gap between parts is free for other pieces, every part is a
//! collision source, and separation / certification see the parts exactly.
use jagua_rs::collision_detection::hazards::filter::NoFilter;
use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::geo_traits::Transformable;
use jagua_rs::probs::spp::entities::{SPPlacement, SPProblem};
use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::optimizer::separator::Separator;
use swarm_core::quantify::tracker::CollisionTracker;
use swarm_core::structure::pipeline::{partial_solution_validation_error, solution_validation_error};
use swarm_core::util::assertions::tracker_matches_layout;
use swarm_core::util::terminator::{BasicTerminator, Terminator};

/// Item 0: two 10x10 squares with a 20 wide gap (a 40x10 envelope).
/// Item 1: a 10x10 square. Item 2: a 30x8 bar.
fn instance() -> jagua_rs::probs::spp::entities::SPInstance {
    let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
        "name": "multipart",
        "strip_height": 60.0,
        "items": [
            {"id": 0, "demand": 1, "allowed_orientations": [0.0],
             "shape": {"type": "multi_polygon", "data": [
                 {"outer": [[0.0,0.0],[10.0,0.0],[10.0,10.0],[0.0,10.0]]},
                 {"outer": [[30.0,0.0],[40.0,0.0],[40.0,10.0],[30.0,10.0]]}
             ]}},
            {"id": 1, "demand": 1, "allowed_orientations": [0.0],
             "shape": {"type": "rectangle", "data": {"x_min": 0.0, "y_min": 0.0, "width": 10.0, "height": 10.0}}},
            {"id": 2, "demand": 1, "allowed_orientations": [0.0],
             "shape": {"type": "rectangle", "data": {"x_min": 0.0, "y_min": 0.0, "width": 30.0, "height": 8.0}}}
        ]
    }))
    .unwrap();
    swarm_core::util::io::import_spp_instance(&ext, DEFAULT_SWARM_CONFIG.cde_config, None, None, None)
        .unwrap()
}

fn problem(width: f32) -> SPProblem {
    let mut prob = SPProblem::new(instance()).unwrap();
    prob.change_strip_width(width).unwrap();
    prob
}

fn place(prob: &mut SPProblem, item: usize, x: f32, y: f32) -> jagua_rs::entities::PItemKey {
    prob.place_item(SPPlacement {
        item_idx: item,
        d_transf: DTransformation::new(0.0, (x, y)),
    })
}

#[test]
fn the_gap_between_parts_is_free_and_each_part_is_a_hazard() {
    let inst = instance();
    assert_eq!(inst.item(0).n_parts(), 2);
    assert!((inst.item(0).area() - 200.0).abs() < 1e-3);
    let mut prob = problem(100.0);
    let pk = place(&mut prob, 0, 30.0, 30.0); // parts at x in [10,20] and [40,50]
    assert!(prob.layout().is_collision_free());
    let square = inst.item(1).shape_cd();
    let in_gap = square.transform_clone(&DTransformation::new(0.0, (30.0, 30.0)).compose());
    assert!(!prob.layout().cde().detect_poly_collision(&in_gap, &NoFilter));
    let on_part = square.transform_clone(&DTransformation::new(0.0, (15.0, 30.0)).compose());
    assert!(prob.layout().cde().detect_poly_collision(&on_part, &NoFilter));
    assert_eq!(prob.layout().haz_keys(pk).len(), 2);

    place(&mut prob, 1, 30.0, 30.0);
    assert!(prob.layout().is_collision_free());
    let sol = prob.save();
    let err = partial_solution_validation_error(&inst, &sol);
    assert!(err.is_none(), "an item in the gap certifies: {err:?}");
}

#[test]
fn certification_rejects_an_item_on_a_part_but_not_in_the_gap() {
    let inst = instance();
    let mut prob = problem(100.0);
    place(&mut prob, 0, 30.0, 30.0);
    place(&mut prob, 1, 15.0, 30.0); // on the first part
    let sol = prob.save();
    assert!(partial_solution_validation_error(&inst, &sol).is_some());
}

#[test]
fn tracker_sums_losses_over_parts_and_separation_resolves_them() {
    let inst = instance();
    let mut prob = problem(80.0);
    place(&mut prob, 0, 30.0, 30.0); // parts at [10,20] and [40,50]
    place(&mut prob, 1, 15.0, 30.0); // overlaps part 0
    place(&mut prob, 2, 45.0, 30.0); // bar over part 1
    let ct = CollisionTracker::new(prob.layout());
    assert!(ct.get_total_loss() > 0.0);
    assert!(tracker_matches_layout(&ct, prob.layout()));

    let mut cfg = DEFAULT_SWARM_CONFIG.expl_cfg.separator_config;
    cfg.n_workers = 1;
    let mut sep = Separator::new(inst.clone(), prob, Xoshiro256PlusPlus::seed_from_u64(7), cfg);
    let mut term = BasicTerminator::new();
    term.new_timeout(std::time::Duration::from_secs(20));
    let (solution, snapshot) = sep.separate(&term, &mut None);
    assert_eq!(snapshot.get_total_loss(), 0.0, "the separator resolves the overlaps");
    assert!(solution_validation_error(&inst, &solution).is_none());
    let parts = solution.layout_snapshot().placed_items().values().find(|p| p.item().idx() == 0).unwrap();
    assert_eq!(parts.n_parts(), 2);
}
