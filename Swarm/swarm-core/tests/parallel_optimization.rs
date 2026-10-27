use jagua_rs::geometry::DTransformation;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution};
use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
use rand::{SeedableRng, rngs::Xoshiro256PlusPlus};
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::optimizer::separator::Separator;
use swarm_core::structure::pipeline::{
    solution_validation_error, solution_validation_error_with_frozen,
};
use swarm_core::util::assertions::{frozen_poses_preserved, tracker_matches_layout};
use swarm_core::util::terminator::{BasicTerminator, Terminator};
use std::time::Duration;

fn fixture(n: usize, depth: u8) -> (SPInstance, SPProblem) {
    let items: Vec<_> = (0..n)
        .map(|id| {
            serde_json::json!({
        "id":id,"demand":1,"allowed_orientations":[0.0],"shape":{
            "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}}})
        })
        .collect();
    let external: ExtSPInstance = serde_json::from_value(serde_json::json!({
        "name":"optimization-regression", "strip_height":12.0, "items":items}))
    .unwrap();
    let mut cfg = DEFAULT_SWARM_CONFIG;
    cfg.cde_config.quadtree_depth = depth;
    let importer = swarm_core::util::io::new_importer(cfg.cde_config, cfg.poly_simpl_tolerance, cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
    let instance = jagua_rs::probs::spp::io::import_instance(&importer, &external).unwrap();
    let mut problem = SPProblem::new(instance.clone()).expect("strip forms a valid container");
    problem.change_strip_width(12.0 + n as f32 * 6.0).expect("strip width must be positive");
    for id in 0..n {
        problem.place_item(SPPlacement {
            item_idx: id,
            d_transf: DTransformation::new(0.0, (8.0 + id as f32 * 6.0, 5.0)),
        });
    }
    assert!(solution_validation_error(&instance, &problem.save()).is_none());
    (instance, problem)
}
fn separator(instance: SPInstance, problem: SPProblem, workers: usize) -> Separator {
    let mut cfg = DEFAULT_SWARM_CONFIG.expl_cfg.separator_config;
    cfg.n_workers = workers;
    cfg.iter_no_imprv_limit = 8;
    cfg.strike_limit = 1;
    let mut sep = Separator::new(
        instance,
        problem,
        Xoshiro256PlusPlus::seed_from_u64(17),
        cfg,
    );
    sep.rng = Xoshiro256PlusPlus::seed_from_u64(101);
    sep
}
fn poses(sol: &SPSolution) -> Vec<(usize, u32, u32, u32)> {
    let mut out: Vec<_> = sol
        .layout_snapshot()
        .placed_items()
        .values()
        .map(|p| {
            let (x, y) = p.d_transf().translation();
            (
                p.item().idx(),
                p.d_transf().rotation().to_bits(),
                x.to_bits(),
                y.to_bits(),
            )
        })
        .collect();
    out.sort_unstable();
    out
}
#[test]
fn nfp_ordering_preserves_frozen_poses_with_and_without_quadtree() {
    for depth in [0, 4] {
        let (instance, mut problem) = fixture(2, depth);
        let old = problem
            .layout()
            .placed_items()
            .iter()
            .find(|(_, p)| p.item().idx() == 0)
            .unwrap()
            .0;
        problem.remove_item(old);
        problem.change_strip_width(30.0).expect("strip width must be positive");
        problem.place_item(SPPlacement {
            item_idx: 0,
            d_transf: DTransformation::new(0.0, (24.0, 5.0)),
        });
        let initial = problem.save();
        let mut sep = separator(instance.clone(), problem, 1);
        sep.freeze_items(&[0]);
        let mut cfg = DEFAULT_SWARM_CONFIG.expl_cfg;
        cfg.time_limit = Duration::from_millis(100);
        let mut term = BasicTerminator::new();
        term.new_timeout(cfg.time_limit);
        let results = swarm_core::nfp_lbf::nfp_ordering_exploration_phase(
            &instance,
            &mut sep,
            &mut None,
            &term,
            &cfg,
            &DEFAULT_SWARM_CONFIG.nfp_lbf,
        );
        assert!(!results.is_empty());
        for result in results {
            assert!(frozen_poses_preserved(&initial, &result, &sep.frozen_mask));
            assert!(
                solution_validation_error_with_frozen(&instance, &result, &sep.frozen_mask)
                    .is_none()
            );
        }
    }
}
#[test]
fn cancelled_width_batch_restores_layout_and_tracker() {
    let (instance, problem) = fixture(8, 4);
    let initial = problem.save();
    let mut sep = separator(instance, problem, 2);
    let mut checks = 0;
    assert!(!sep.change_strip_width_with_control(
        initial.strip_width() * 0.95,
        Some(0.0),
        &mut || {
            checks += 1;
            checks >= 4
        }
    ));
    assert_eq!(poses(&initial), poses(&sep.prob.save()));
    assert_eq!(initial.strip_width(), sep.prob.strip_width());
    assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));
}
#[test]
fn rollback_installs_a_narrower_snapshot_without_shifting_a_frozen_item() {
    let (instance, problem) = fixture(2, 4);
    let mut sep = separator(instance.clone(), problem, 1);
    sep.freeze_items(&[1]);
    let wide = sep.prob.save();
    sep.refit_strip();
    let narrow = sep.prob.save();
    assert!(narrow.strip_width() < wide.strip_width());
    sep.rollback(&wide, None);
    sep.rollback(&narrow, None);
    assert_eq!(poses(&wide), poses(&sep.prob.save()));
    assert!(
        solution_validation_error_with_frozen(&instance, &sep.prob.save(), &sep.frozen_mask)
            .is_none()
    );
}
#[test]
fn grouped_polish_translates_rigid_units_and_preserves_locks() {
    use swarm_core::optimizer::place_groups::{PlaceGroupMember, PlaceGroupSpec, PlaceGroups};
    let (instance, problem) = fixture(3, 4);
    let initial = problem.save();
    let mut sep = separator(instance.clone(), problem, 2);
    let members = (1..3)
        .map(|id| PlaceGroupMember {
            item_id: id,
            initial_transform: initial
                .layout_snapshot()
                .placed_items()
                .values()
                .find(|p| p.item().idx() == id)
                .unwrap()
                .d_transf(),
        })
        .collect();
    sep.set_place_groups(
        PlaceGroups::new(&instance, vec![PlaceGroupSpec { id: 77, members }]).unwrap(),
    )
    .unwrap();
    sep.freeze_items(&[0]);
    let mut cfg = DEFAULT_SWARM_CONFIG.polish_cfg.clone();
    cfg.reclaim.enabled = false;
    let mut term = BasicTerminator::new();
    term.new_timeout(Duration::from_secs(2));
    let result = swarm_core::optimizer::polish::polish_phase_with_nfp(
        &instance,
        &mut sep,
        &initial,
        &mut None,
        &term,
        &cfg,
        &DEFAULT_SWARM_CONFIG.nfp_integration,
        &DEFAULT_SWARM_CONFIG.lp_solver,
    );
    assert!(frozen_poses_preserved(&initial, &result, &sep.frozen_mask));
    assert!(solution_validation_error_with_frozen(&instance, &result, &sep.frozen_mask).is_none());
    assert!(result.strip_width() < initial.strip_width() - 0.1);
    let dt = |id| {
        result
            .layout_snapshot()
            .placed_items()
            .values()
            .find(|p| p.item().idx() == id)
            .unwrap()
            .d_transf()
    };
    assert!((dt(2).translation().0 - dt(1).translation().0 - 6.0).abs() < 1e-4);
    assert!((dt(2).translation().1 - dt(1).translation().1).abs() < 1e-4);
    assert_eq!(dt(1).rotation(), dt(2).rotation());
    assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));
}
#[cfg(any(feature = "adaptive", feature = "capmods"))]
#[test]
fn retired_rnr_preserves_state_across_worker_counts() {
    use swarm_core::optimizer::lns::{BandTabu, RuinStrategy, ruin_and_recreate};
    let (instance, problem) = fixture(8, 4);
    let initial = problem.save();
    let cfg = swarm_core::config::RuinRecreateConfig {
        period: 1,
        band_fraction: 1.0,
        max_items: 8,
        ..swarm_core::config::DEFAULT_SWARM_CONFIG.expl_cfg.rnr
    };
    let mut results = Vec::new();
    for workers in [1, 2, 4] {
        let mut restored = SPProblem::new(instance.clone()).expect("strip forms a valid container");
        restored.restore(&initial);
        let mut sep = separator(instance.clone(), restored, workers);
        let n = ruin_and_recreate(
            &mut sep,
            &cfg,
            RuinStrategy::GapTarget,
            &mut BandTabu::new(),
            &BasicTerminator::new(),
        );
        assert_eq!(n, 0);
        assert_eq!(poses(&initial), poses(&sep.prob.save()));
        assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));
        results.push((poses(&sep.prob.save()), sep.ct.get_total_loss()));
    }
    assert_eq!(results[0], results[1]);
    assert_eq!(results[0], results[2]);
}

#[test]
fn disabled_reclaim_preserves_poses_width_and_rng() {
    use rand::RngExt;
    let (instance, problem) = fixture(2, 4);
    let initial = problem.save();
    let mut sep = separator(instance.clone(), problem, 1);
    let mut expected_rng = sep.rng.clone();
    let c = DEFAULT_SWARM_CONFIG;
    let mut cfg = c.polish_cfg.reclaim;
    cfg.enabled = false;
    let (result, stats) = swarm_core::optimizer::reclaim::reclaim_leftover(
        &instance,
        &mut sep,
        initial.clone(),
        jagua_rs::Instant::now() + Duration::from_secs(1),
        &cfg,
        &c.cmpr_cfg,
        &c.expl_cfg,
        &c.polish_cfg,
        &c.nfp_integration,
        &c.escape,
        &c.lp_solver,
        &BasicTerminator::new(),
    );
    assert_eq!(poses(&initial), poses(&result));
    assert_eq!(poses(&initial), poses(&sep.prob.save()));
    assert_eq!(initial.strip_width(), sep.prob.strip_width());
    assert_eq!(stats.cycles, 0);
    assert_eq!(expected_rng.random::<u64>(), sep.rng.random::<u64>());
}

#[test]
fn polish_shrinks_past_frozen_right_edge_and_keeps_movable_improvement() {
    let (instance, mut problem) = fixture(3, 4);
    problem.fit_strip().expect("strip width must be positive");
    let initial = problem.save();
    let mut sep = separator(instance.clone(), problem, 1);
    sep.freeze_items(&[2]);
    let mut cfg = DEFAULT_SWARM_CONFIG.polish_cfg.clone();
    cfg.lp.enabled = false;
    cfg.hole_fill.enabled = false;
    cfg.n_rounds = 1;
    cfg.time_limit = Duration::from_millis(100);
    cfg.reclaim.enabled = false;
    let result = swarm_core::optimizer::polish::polish_phase_with_nfp(
        &instance,
        &mut sep,
        &initial,
        &mut None,
        &BasicTerminator::new(),
        &cfg,
        &DEFAULT_SWARM_CONFIG.nfp_integration,
        &DEFAULT_SWARM_CONFIG.lp_solver,
    );
    assert!(result.strip_width() < initial.strip_width());
    assert!(solution_validation_error(&instance, &result).is_some());
    let before = initial
        .layout_snapshot()
        .placed_items()
        .values()
        .find(|p| p.item().idx() == 0)
        .unwrap()
        .d_transf();
    let after = result
        .layout_snapshot()
        .placed_items()
        .values()
        .find(|p| p.item().idx() == 0)
        .unwrap()
        .d_transf();
    assert!(after.translation().0 < before.translation().0 - 0.1);
    assert!(frozen_poses_preserved(&initial, &result, &sep.frozen_mask));
    assert!(solution_validation_error_with_frozen(&instance, &result, &sep.frozen_mask).is_none());
}
