use jagua_rs::{
    geometry::{DTransformation, geo_enums::RotationRange},
    probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution},
};
use swarm_core::{
    config::DEFAULT_SWARM_CONFIG as C,
    structure::pipeline::{
        partial_solution_validation_error, solution_validation_error,
        solution_validation_error_with_frozen,
    },
    util::io::import_spp_instance_with_mm,
};

fn fixture() -> (SPInstance, SPSolution) {
    let ext = serde_json::from_value(serde_json::json!({"name":"certificate","strip_height":100,
        "items":([0,1].map(|id| serde_json::json!({"id":id,"demand":1,"allowed_orientations":[0],
            "shape":{"type":"simple_polygon","data":[[0,0],[10,0],[10,10],[0,10]]}})))}))
    .unwrap();
    let inst = import_spp_instance_with_mm(&ext, C.cde_config, None, None, None, None).unwrap();
    let mut p = SPProblem::new(inst.clone()).expect("strip forms a valid container");
    p.change_strip_width(100.).expect("strip width must be positive");
    for (id, x) in [(0, 20.), (1, 50.)] {
        p.place_item(SPPlacement {
            item_idx: id,
            d_transf: DTransformation::new(0., (x, 20.)),
        });
    }
    (inst, p.save())
}

#[test]
fn unchanged_certificate_hits_but_every_authoritative_change_invalidates() {
    let (inst, sol) = fixture();
    assert!(solution_validation_error(&inst, &sol).is_none());
    assert!(solution_validation_error(&inst, &sol.clone()).is_none());
    let mut moved = sol.clone();
    for p in moved.layout_snapshot_mut().placed_items_mut().values_mut() {
        p.set_d_transf(DTransformation::new(0., (20., 20.)));
    }
    assert!(solution_validation_error(&inst, &moved).is_some());
    let mut short = sol.clone();
    short.strip_mut().width = 10.;
    assert!(solution_validation_error(&inst, &short).is_some());
    let mut missing = inst.clone();
    missing.items[0].1 = 2;
    assert!(solution_validation_error(&missing, &sol).is_some());
    let mut rotation = inst.clone();
    rotation.items[0].0 = std::sync::Arc::new(rotation.items[0].0.as_ref().clone().with_allowed_orientations(
        jagua_rs::geometry::AllowedOrientations::new(RotationRange::Discrete(vec![1.]), vec![]).unwrap(),
    ));
    assert!(solution_validation_error(&rotation, &sol).is_some());
    let mut geometry = inst.clone();
    geometry.items[0].0 = std::sync::Arc::new(geometry.items[0].0.as_ref().clone().with_shape_cd(
        std::sync::Arc::new(geometry.items[0].0.shape_cd().as_ref().clone()),
    ));
    assert!(solution_validation_error(&geometry, &sol).is_none());
    let mut original = inst.clone();
    original.items[0].0 = std::sync::Arc::new(original.items[0].0.as_ref().clone().with_shape_orig(
        std::sync::Arc::new(original.items[0].0.shape_orig().as_ref().clone()),
    ));
    assert!(solution_validation_error(&original, &sol).is_none());
}

#[test]
fn frozen_and_quantity_exemptions_cannot_leak_into_strict_certificates() {
    let (inst, mut sol) = fixture();
    for p in sol.layout_snapshot_mut().placed_items_mut().values_mut() {
        p.set_d_transf(DTransformation::new(0., (-20., 20.)));
    }
    assert!(solution_validation_error_with_frozen(&inst, &sol, &[true, true]).is_none());
    assert!(solution_validation_error_with_frozen(&inst, &sol, &[true, false]).is_some());
    assert!(solution_validation_error(&inst, &sol).is_some());
    let empty = SPProblem::new(inst.clone()).expect("strip forms a valid container").save();
    assert!(partial_solution_validation_error(&inst, &empty).is_none());
    assert!(solution_validation_error(&inst, &empty).is_some());
}
