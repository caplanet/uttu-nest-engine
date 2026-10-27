// Standalone review probe; does not modify the solver or its test targets.
use jagua_rs::geometry::DTransformation;
use jagua_rs::io::import::Importer;
use jagua_rs::probs::spp::entities::{SPPlacement, SPProblem};
use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::optimizer::separator::Separator;
use swarm_core::util::terminator::{BasicTerminator, Terminator};
use std::time::Duration;

fn main() {
    let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
        "name":"review-frozen-ordering", "strip_height":12.0,
        "items":[
            {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}}},
            {"id":1,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}}}
        ]
    })).unwrap();
    let mut cfg = DEFAULT_SWARM_CONFIG;
    if std::env::args().any(|arg| arg == "--depth-zero") {
        cfg.cde_config.quadtree_depth = 0;
    }
    let importer = Importer::new(cfg.cde_config, cfg.poly_simpl_tolerance,
        cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
    let instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext).unwrap();
    let mut prob = SPProblem::new(instance.clone());
    prob.change_strip_width(30.0);
    let locked = DTransformation::new(0.0, (20.0, 5.0));
    prob.place_item(SPPlacement { item_id: 0, d_transf: locked });
    prob.place_item(SPPlacement { item_id: 1, d_transf: DTransformation::new(0.0, (5.0, 5.0)) });
    assert!(swarm_core::structure::pipeline::solution_validation_error(&instance, &prob.save()).is_none());
    let mut expl = cfg.expl_cfg;
    expl.time_limit = Duration::from_secs(1);
    expl.separator_config.n_workers = 1;
    let mut sep = Separator::new(instance.clone(), prob,
        Xoshiro256PlusPlus::seed_from_u64(17), expl.separator_config);
    sep.freeze_items(&[0]);
    let mut term = BasicTerminator::new();
    term.new_timeout(expl.time_limit);
    let mut nfp = cfg.nfp_lbf;
    nfp.enabled = true;
    nfp.cache_budget_ms = 50;
    let sols = swarm_core::nfp_lbf::nfp_ordering_exploration_phase(
        &instance, &mut sep, &mut None, &term, &expl, &nfp);
    let result = sols.last().unwrap();
    let actual = result.layout_snapshot.placed_items.values()
        .find(|p| p.item_id == 0).unwrap().d_transf;
    let exact = swarm_core::structure::pipeline::solution_validation_error(&instance, result).is_none();
    println!("locked_before={:?}; locked_after={:?}; width={}; exact={}; frozen_preserved={}",
        locked, actual, result.strip_width(), exact,
        locked.translation() == actual.translation() && locked.rotation() == actual.rotation());
    assert!(exact);
    assert_eq!(locked, actual, "frozen placement must be preserved");
}
