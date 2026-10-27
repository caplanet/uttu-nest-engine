use jagua_rs::geometry::DTransformation;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution};
use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::{DEFAULT_POLISH_CONFIG, DEFAULT_SWARM_CONFIG};
use swarm_core::optimizer::place_groups::{PlaceGroupMember, PlaceGroupSpec, PlaceGroups};
use swarm_core::optimizer::polish::lp::lp_compaction_pass;
use swarm_core::optimizer::polish::polish_phase_measured;
use swarm_core::optimizer::separator::Separator;
use swarm_core::util::assertions::tracker_matches_layout;
use swarm_core::util::terminator::BasicTerminator;

fn fixture(grouped: bool) -> (SPInstance, Separator) {
    let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
        "name":"rigid-unit-lp", "strip_height":16.,
        "items": (0..3).map(|id| serde_json::json!({"id":id,"demand":1,
            "allowed_orientations":[0.,90.],"shape":{"type":"rectangle",
            "data":{"x_min":0.,"y_min":0.,"width":4.,"height":4.}}})).collect::<Vec<_>>()
    }))
    .unwrap();
    let cfg = DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(cfg.cde_config, cfg.poly_simpl_tolerance, cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
    let instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext).unwrap();
    let mut problem = SPProblem::new(instance.clone()).expect("strip forms a valid container");
    problem.change_strip_width(32.).expect("strip width must be positive");
    for (item_id, x, y, angle) in [
        (0, 16., 4., 0.),
        (1, 24., 10., std::f32::consts::FRAC_PI_2),
        (2, 4., 4., 0.),
    ] {
        problem.place_item(SPPlacement {
            item_idx: item_id,
            d_transf: DTransformation::new(angle, (x, y)),
        });
    }
    assert!(problem.layout().is_collision_free());
    let mut sc = cfg.expl_cfg.separator_config;
    sc.n_workers = 1;
    let mut sep = Separator::new(
        instance.clone(),
        problem,
        Xoshiro256PlusPlus::seed_from_u64(909),
        sc,
    );
    if grouped {
        let members = [0, 1]
            .map(|id| PlaceGroupMember {
                item_id: id,
                initial_transform: pose(&sep.prob.save(), id),
            })
            .to_vec();
        sep.set_place_groups(
            PlaceGroups::new(&instance, vec![PlaceGroupSpec { id: 9, members }]).unwrap(),
        )
        .unwrap();
    }
    (instance, sep)
}

fn pose(solution: &SPSolution, id: usize) -> DTransformation {
    solution
        .layout_snapshot()
        .placed_items()
        .values()
        .find(|p| p.item().idx() == id)
        .unwrap()
        .d_transf()
}

fn assert_certified(sep: &Separator, before: &SPSolution) {
    let after = sep.prob.save();
    assert!(sep.solution_is_certified(&after, before));
    assert_eq!(after.layout_snapshot().placed_items().len(), 3);
    assert!(after.strip_width() <= before.strip_width());
    assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));
}

#[test]
fn rigid_group_lp_compresses_and_preserves_member_offsets_and_rotations() {
    let (_, mut sep) = fixture(true);
    let before = sep.prob.save();
    let mut cfg = DEFAULT_POLISH_CONFIG.lp;
    cfg.n_iterations = 1;
    cfg.trust_region_ratio = 0.2;
    assert_eq!(
        lp_compaction_pass(&mut sep, &cfg, &BasicTerminator::new()),
        1
    );
    let after = sep.prob.save();
    assert!(after.strip_width() < before.strip_width() - 1.);
    let mut deltas = Vec::new();
    for id in [0, 1] {
        let (b, a) = (pose(&before, id), pose(&after, id));
        assert_eq!(a.rotation(), b.rotation());
        deltas.push((
            a.translation().0 - b.translation().0,
            a.translation().1 - b.translation().1,
        ));
    }
    assert!(deltas[0].0 < -1.);
    assert!((deltas[0].0 - deltas[1].0).abs() < 1e-5);
    assert!((deltas[0].1 - deltas[1].1).abs() < 1e-5);
    assert_certified(&sep, &before);
}

#[test]
fn frozen_member_pins_the_entire_lp_unit_while_other_items_can_compress() {
    let (_, mut sep) = fixture(true);
    sep.freeze_items(&[1]);
    let before = sep.prob.save();
    let mut cfg = DEFAULT_POLISH_CONFIG.lp;
    cfg.n_iterations = 1;
    cfg.trust_region_ratio = 0.2;
    assert_eq!(
        lp_compaction_pass(&mut sep, &cfg, &BasicTerminator::new()),
        1
    );
    let after = sep.prob.save();
    for id in [0, 1] {
        assert_eq!(pose(&before, id), pose(&after, id));
    }
    assert!(pose(&after, 2).translation().0 < pose(&before, 2).translation().0);
    assert_certified(&sep, &before);
}

#[test]
fn group_lp_rejects_external_collision_when_contact_rows_are_omitted() {
    let (_, mut sep) = fixture(true);
    sep.freeze_items(&[2]);
    let before = sep.prob.save();
    let mut cfg = DEFAULT_POLISH_CONFIG.lp;
    cfg.n_iterations = 1;
    cfg.trust_region_ratio = 0.5;
    cfg.max_constraints_per_pair = 0;
    cfg.max_backtracks = 0;
    // Unconstrained full move puts member 0 on the frozen obstacle. The
    // surrogate/exact acceptance gate must restore the whole transaction.
    assert_eq!(
        lp_compaction_pass(&mut sep, &cfg, &BasicTerminator::new()),
        0
    );
    let after = sep.prob.save();
    assert_eq!(after.strip_width(), before.strip_width());
    for id in 0..3 {
        assert_eq!(pose(&after, id), pose(&before, id));
    }
    assert_certified(&sep, &before);
    // The same proposal succeeds only after backtracking to a safe unit move.
    cfg.max_backtracks = 4;
    assert_eq!(
        lp_compaction_pass(&mut sep, &cfg, &BasicTerminator::new()),
        1
    );
    assert!(sep.prob.strip_width() < before.strip_width() - 1.);
    assert_eq!(pose(&sep.prob.save(), 2), pose(&before, 2));
    assert_certified(&sep, &before);
}

#[test]
fn ungrouped_lp_still_compresses_independent_items() {
    let (_, mut sep) = fixture(false);
    let before = sep.prob.save();
    let mut cfg = DEFAULT_POLISH_CONFIG.lp;
    cfg.n_iterations = 1;
    cfg.trust_region_ratio = 0.2;
    assert_eq!(
        lp_compaction_pass(&mut sep, &cfg, &BasicTerminator::new()),
        1
    );
    assert!(sep.prob.strip_width() < before.strip_width() - 1.);
    assert_certified(&sep, &before);
}

#[test]
fn polish_lp_only_runs_for_rigid_groups() {
    let (instance, mut sep) = fixture(true);
    let before = sep.prob.save();
    let mut cfg = DEFAULT_POLISH_CONFIG;
    cfg.enabled = true;
    cfg.lp.enabled = true;
    cfg.lp.n_iterations = 1;
    cfg.lp.trust_region_ratio = 0.2;
    cfg.slide.enabled = false;
    cfg.hole_fill.enabled = false;
    cfg.n_rounds = 1;
    cfg.time_limit = std::time::Duration::from_millis(100);
    let (after, stats) = polish_phase_measured(
        &instance,
        &mut sep,
        &before,
        &mut None,
        &BasicTerminator::new(),
        &cfg,
        &DEFAULT_SWARM_CONFIG.nfp_integration,
        &DEFAULT_SWARM_CONFIG.lp_solver,
    );
    assert!(stats.rounds > 0);
    assert!(stats.changes > 0);
    assert!(after.strip_width() < before.strip_width() - 1.);
    assert_certified(&sep, &before);
}

#[test]
fn cancellation_keeps_the_published_rigid_lp_incumbent() {
    use swarm_core::util::listener::{ReportType, SolutionListener};
    use swarm_core::util::terminator::Terminator;
    use std::sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    };
    struct Stop(Arc<AtomicBool>);
    impl Terminator for Stop {
        fn kill(&self) -> bool {
            self.0.load(Ordering::Relaxed)
        }
        fn new_timeout(&mut self, _: std::time::Duration) {}
        fn timeout_at(&self) -> Option<jagua_rs::Instant> {
            None
        }
    }
    struct Listener {
        stop: Arc<AtomicBool>,
        published: Option<SPSolution>,
    }
    impl SolutionListener for Listener {
        fn report(&mut self, kind: ReportType, solution: &SPSolution, _: &SPInstance) {
            if kind == ReportType::CmprFeas {
                self.published = Some(solution.clone());
                self.stop.store(true, Ordering::Relaxed);
            }
        }
    }
    let (instance, mut sep) = fixture(true);
    let before = sep.prob.save();
    let flag = Arc::new(AtomicBool::new(false));
    let term = Stop(flag.clone());
    let mut listener = Listener {
        stop: flag,
        published: None,
    };
    let mut cfg = DEFAULT_POLISH_CONFIG;
    cfg.enabled = true;
    cfg.lp.enabled = true;
    cfg.lp.n_iterations = 1;
    cfg.lp.trust_region_ratio = 0.2;
    cfg.slide.enabled = false;
    cfg.hole_fill.enabled = false;
    let (result, _) = polish_phase_measured(
        &instance,
        &mut sep,
        &before,
        &mut Some(&mut listener),
        &term,
        &cfg,
        &DEFAULT_SWARM_CONFIG.nfp_integration,
        &DEFAULT_SWARM_CONFIG.lp_solver,
    );
    let published = listener
        .published
        .expect("rigid LP improvement must be published");
    assert!(result.strip_width() < before.strip_width() - 1.);
    assert_eq!(result.strip_width(), published.strip_width());
    for id in 0..3 {
        assert_eq!(pose(&result, id), pose(&published, id));
    }
    assert_certified(&sep, &before);
}

#[test]
fn cancellation_inside_lp_keeps_its_unpublished_certified_iteration() {
    use swarm_core::util::terminator::Terminator;
    use std::sync::atomic::{AtomicUsize, Ordering};
    struct CountStop {
        calls: AtomicUsize,
        limit: usize,
    }
    impl Terminator for CountStop {
        fn kill(&self) -> bool {
            self.calls.fetch_add(1, Ordering::Relaxed) >= self.limit
        }
        fn new_timeout(&mut self, _: std::time::Duration) {}
        fn timeout_at(&self) -> Option<jagua_rs::Instant> {
            None
        }
    }
    let mut cfg = DEFAULT_POLISH_CONFIG;
    cfg.enabled = true;
    cfg.lp.enabled = true;
    cfg.lp.n_iterations = 1;
    cfg.lp.trust_region_ratio = 0.2;
    cfg.slide.enabled = false;
    cfg.hole_fill.enabled = false;
    // Calibrate the exact first-iteration polling count without wall-clock races.
    let (_, mut direct) = fixture(true);
    let count = CountStop {
        calls: AtomicUsize::new(0),
        limit: usize::MAX,
    };
    assert_eq!(lp_compaction_pass(&mut direct, &cfg.lp, &count), 1);
    let first = direct.prob.save();
    let stop = CountStop {
        calls: AtomicUsize::new(0),
        // Polish polls once before initial scans and once at round entry.
        limit: count.calls.load(Ordering::Relaxed) + 2,
    };
    let (instance, mut sep) = fixture(true);
    let before = sep.prob.save();
    cfg.lp.n_iterations = 2;
    let (result, _) = polish_phase_measured(
        &instance,
        &mut sep,
        &before,
        &mut None,
        &stop,
        &cfg,
        &DEFAULT_SWARM_CONFIG.nfp_integration,
        &DEFAULT_SWARM_CONFIG.lp_solver,
    );
    assert!(stop.kill());
    assert_eq!(result.strip_width(), first.strip_width());
    for id in 0..3 {
        assert_eq!(pose(&result, id), pose(&first, id));
    }
    assert_certified(&sep, &before);
}
