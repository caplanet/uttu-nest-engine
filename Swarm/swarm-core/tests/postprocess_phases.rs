use jagua_rs::geometry::DTransformation;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution};
use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::{DEFAULT_POLISH_CONFIG, DEFAULT_SWARM_CONFIG, ShrinkDecayStrategy};
use swarm_core::optimizer::compress::compression_phase;
use swarm_core::optimizer::optimize_configured;
use swarm_core::optimizer::polish::holes::hole_fill_pass;
use swarm_core::optimizer::polish::lp::lp_compaction_pass;
use swarm_core::optimizer::separator::Separator;
use swarm_core::structure::pipeline::solution_validation_error;
use swarm_core::util::assertions::tracker_matches_layout;
use swarm_core::util::terminator::BasicTerminator;

fn import_instance(value: serde_json::Value) -> SPInstance {
    let external: ExtSPInstance = serde_json::from_value(value).unwrap();
    let config = DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(config.cde_config, config.poly_simpl_tolerance, config.min_item_separation, config.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&importer, &external).unwrap()
}

#[test]
fn empty_polish_has_zero_rounds_and_preserves_certified_poses() {
    use swarm_core::optimizer::place_groups::{PlaceGroupMember, PlaceGroupSpec, PlaceGroups};
    use swarm_core::optimizer::polish::{PolishExit, polish_phase_measured};
    let instance = import_instance(serde_json::json!({"name":"empty-polish","strip_height":12.,
        "items":([0,1].map(|id| serde_json::json!({"id":id,"demand":1,"allowed_orientations":[0.],
            "shape":{"type":"rectangle","data":{"x_min":0.,"y_min":0.,"width":4.,"height":4.}}})))}));
    for (enabled, lp, grouped) in [
        (false, true, false),
        (true, false, false),
        (true, false, true),
    ] {
        let problem = placed_problem(&instance, 30., &[(0, 3., 3.), (1, 20., 8.)]);
        let initial = problem.save();
        let mut sep = separator(instance.clone(), problem, 101);
        if grouped {
            let members = initial
                .layout_snapshot()
                .placed_items()
                .values()
                .map(|p| PlaceGroupMember {
                    item_id: p.item().idx(),
                    initial_transform: p.d_transf(),
                })
                .collect();
            sep.set_place_groups(
                PlaceGroups::new(&instance, vec![PlaceGroupSpec { id: 1, members }]).unwrap(),
            )
            .unwrap();
        }
        let mut cfg = DEFAULT_POLISH_CONFIG;
        cfg.enabled = enabled;
        cfg.lp.enabled = lp;
        cfg.slide.enabled = false;
        cfg.hole_fill.enabled = false;
        cfg.time_limit = std::time::Duration::from_secs(2);
        cfg.n_rounds = 100; // No work must not become 100 rounds of validation/signatures.
        let started = std::time::Instant::now();
        let (result, stats) = polish_phase_measured(
            &instance,
            &mut sep,
            &initial,
            &mut None,
            &BasicTerminator::new(),
            &cfg,
            &DEFAULT_SWARM_CONFIG.nfp_integration,
            &DEFAULT_SWARM_CONFIG.lp_solver,
        );
        assert!(started.elapsed() < std::time::Duration::from_millis(500));
        assert_eq!(stats.rounds, 0);
        assert_eq!(stats.changes, 0);
        assert_eq!(stats.exit, PolishExit::NoChanges);
        assert_eq!(result.strip(), initial.strip());
        for (pk, p) in initial.layout_snapshot().placed_items() {
            assert_eq!(result.layout_snapshot().placed_items()[pk].d_transf(), p.d_transf());
        }
        assert_complete_exact(&instance, &result);
        assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));
    }
}

fn placed_problem(
    instance: &SPInstance,
    strip_width: f32,
    placements: &[(usize, f32, f32)],
) -> SPProblem {
    let mut problem = SPProblem::new(instance.clone()).expect("strip forms a valid container");
    problem.change_strip_width(strip_width).expect("strip width must be positive");
    for &(item_id, x, y) in placements {
        problem.place_item(SPPlacement {
            item_idx: item_id,
            d_transf: DTransformation::new(0.0, (x, y)),
        });
    }
    assert!(
        problem.layout().is_collision_free(),
        "synthetic fixture must be feasible: {:?}",
        problem
            .layout()
            .placed_items()
            .values()
            .map(|placed| (placed.item().idx(), placed.shape().bbox()))
            .collect::<Vec<_>>()
    );
    assert!(problem.item_demand_qtys().iter().all(|qty| *qty == 0));
    problem
}

fn separator(instance: SPInstance, problem: SPProblem, seed: u64) -> Separator {
    let mut config = DEFAULT_SWARM_CONFIG.expl_cfg.separator_config;
    config.n_workers = 1;
    config.iter_no_imprv_limit = 20;
    config.strike_limit = 1;
    Separator::new(
        instance,
        problem,
        Xoshiro256PlusPlus::seed_from_u64(seed),
        config,
    )
}

fn assert_complete_exact(instance: &SPInstance, solution: &SPSolution) {
    assert_eq!(
        solution.layout_snapshot().placed_items().len(),
        instance.total_item_qty(),
        "postprocess output must retain every demanded copy"
    );
    assert!(
        solution_validation_error(instance, solution).is_none(),
        "postprocess output must pass the transform-authoritative exact validator"
    );
}

fn assert_separator_sync(sep: &Separator) {
    assert!(
        tracker_matches_layout(&sep.ct, &sep.prob.layout()),
        "collision tracker must describe the current layout"
    );
    assert!(sep.prob.item_demand_qtys().iter().all(|qty| *qty == 0));
}

#[test]
fn active_polish_ends_at_the_proven_fixed_point_of_frozen_groups() {
    use swarm_core::optimizer::place_groups::{PlaceGroupMember, PlaceGroupSpec, PlaceGroups};
    use swarm_core::optimizer::polish::{PolishExit, polish_phase_measured};
    use std::time::{Duration, Instant};

    let instance = import_instance(
        serde_json::json!({"name":"polish-budget","strip_height":12.,
        "items":([0,1].map(|id| serde_json::json!({"id":id,"demand":1,"allowed_orientations":[0.],
            "shape":{"type":"rectangle","data":{"x_min":0.,"y_min":0.,"width":4.,"height":4.}}})))}),
    );
    for lp in [false, true] {
        let problem = placed_problem(&instance, 24., &[(0, 3., 3.), (1, 20., 8.)]);
        let initial = problem.save();
        let mut sep = separator(instance.clone(), problem, 101);
        let members = initial
            .layout_snapshot()
            .placed_items()
            .values()
            .map(|p| PlaceGroupMember {
                item_id: p.item().idx(),
                initial_transform: p.d_transf(),
            })
            .collect();
        sep.set_place_groups(
            PlaceGroups::new(&instance, vec![PlaceGroupSpec { id: 7, members }]).unwrap(),
        )
        .unwrap();
        sep.freeze_items(&[0, 1]);
        let mut cfg = DEFAULT_POLISH_CONFIG;
        cfg.lp.enabled = lp;
        cfg.slide.enabled = !lp;
        cfg.n_rounds = 1;
        // 고정점이 증명되면 Polish 는 예산을 기다리지 않고 끝난다 — 남은 시간은
        // 호출부(Reclaim)의 몫이다. 예산을 넉넉히 줘서 "일찍 끝났다" 를 확인한다.
        cfg.time_limit = Duration::from_secs(2);
        let started = Instant::now();
        let (result, stats) = polish_phase_measured(
            &instance,
            &mut sep,
            &initial,
            &mut None,
            &BasicTerminator::new(),
            &cfg,
            &DEFAULT_SWARM_CONFIG.nfp_integration,
            &DEFAULT_SWARM_CONFIG.lp_solver,
        );
        let elapsed = started.elapsed();
        assert!(
            elapsed < Duration::from_secs(1),
            "proven fixed point must end Polish before its budget: {elapsed:?}, {stats:?}"
        );
        assert_eq!(
            stats.rounds, 1,
            "proven frozen fixed point must not repeat real passes"
        );
        assert_eq!(stats.exit, PolishExit::NoChanges);
        for (key, p) in initial.layout_snapshot().placed_items() {
            assert_eq!(
                result.layout_snapshot().placed_items()[key].d_transf(),
                p.d_transf()
            );
        }
        assert!(sep.solution_is_certified(&result, &initial));
        assert_separator_sync(&sep);
    }
}

#[test]
fn active_polish_respects_a_shorter_parent_deadline() {
    use swarm_core::optimizer::polish::{PolishExit, polish_phase_measured};
    use swarm_core::util::terminator::Terminator;
    use std::time::{Duration, Instant};
    let instance = import_instance(
        serde_json::json!({"name":"polish-parent","strip_height":12.,
        "items":[{"id":0,"demand":1,"allowed_orientations":[0.],
            "shape":{"type":"rectangle","data":{"x_min":0.,"y_min":0.,"width":4.,"height":4.}}}]}),
    );
    let problem = placed_problem(&instance, 10., &[(0, 3., 3.)]);
    let initial = problem.save();
    let mut sep = separator(instance.clone(), problem, 101);
    sep.freeze_items(&[0]);
    let mut cfg = DEFAULT_POLISH_CONFIG;
    cfg.time_limit = Duration::from_secs(2);
    cfg.n_rounds = 1;
    cfg.lp.enabled = false;
    let mut term = BasicTerminator::new();
    let started = Instant::now();
    term.new_timeout(Duration::from_millis(40));
    let (result, stats) = polish_phase_measured(
        &instance,
        &mut sep,
        &initial,
        &mut None,
        &term,
        &cfg,
        &DEFAULT_SWARM_CONFIG.nfp_integration,
        &DEFAULT_SWARM_CONFIG.lp_solver,
    );
    let elapsed = started.elapsed();
    assert!(
        elapsed < Duration::from_millis(500),
        "local budget extended the parent: {elapsed:?}"
    );
    assert_eq!(stats.rounds, 1, "one round proves the fixed point: {stats:?}");
    // 라운드가 부모 마감(40ms) 안에 끝나면 고정점으로, 넘기면 마감으로 끝난다.
    assert!(
        matches!(stats.exit, PolishExit::NoChanges | PolishExit::Killed),
        "{stats:?}"
    );
    assert!(sep.solution_is_certified(&result, &initial));
    assert_separator_sync(&sep);
}

#[test]
fn stalled_polish_ends_at_its_fixed_point_without_waiting_for_stop() {
    use swarm_core::optimizer::polish::{PolishExit, polish_phase_measured};
    use swarm_core::util::terminator::Terminator;
    use std::sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    };
    use std::time::{Duration, Instant};
    struct Stop(Arc<AtomicBool>);
    impl Terminator for Stop {
        fn kill(&self) -> bool {
            self.0.load(Ordering::Relaxed)
        }
        fn new_timeout(&mut self, _: Duration) {}
        fn timeout_at(&self) -> Option<jagua_rs::Instant> {
            None
        }
    }
    let instance = import_instance(serde_json::json!({"name":"polish-stop","strip_height":12.,
        "items":[{"id":0,"demand":1,"allowed_orientations":[0.],
            "shape":{"type":"rectangle","data":{"x_min":0.,"y_min":0.,"width":4.,"height":4.}}}]}));
    let problem = placed_problem(&instance, 10., &[(0, 3., 3.)]);
    let initial = problem.save();
    let mut sep = separator(instance.clone(), problem, 101);
    sep.freeze_items(&[0]);
    let mut cfg = DEFAULT_POLISH_CONFIG;
    cfg.time_limit = Duration::from_secs(2);
    cfg.lp.enabled = false;
    cfg.n_rounds = 1;
    let flag = Arc::new(AtomicBool::new(false));
    let stop_flag = flag.clone();
    let stop_thread = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(40));
        stop_flag.store(true, Ordering::Relaxed);
    });
    let started = Instant::now();
    let (result, stats) = polish_phase_measured(
        &instance,
        &mut sep,
        &initial,
        &mut None,
        &Stop(flag.clone()),
        &cfg,
        &DEFAULT_SWARM_CONFIG.nfp_integration,
        &DEFAULT_SWARM_CONFIG.lp_solver,
    );
    let elapsed = started.elapsed();
    assert!(
        elapsed < Duration::from_millis(500),
        "fixed-point Polish must not wait for its budget or a stop: {elapsed:?}"
    );
    assert_eq!(
        stats.rounds, 1,
        "a proven fixed point must not repeat real passes"
    );
    // 고정점 증명이 정지 신호(40ms)보다 늦으면 정지로 끝난다.
    assert!(
        matches!(stats.exit, PolishExit::NoChanges | PolishExit::Killed),
        "{stats:?}"
    );
    assert!(sep.solution_is_certified(&result, &initial));
    assert_separator_sync(&sep);
    stop_thread.join().unwrap();
}

#[test]
fn zero_budget_polish_returns_without_moving_or_running_rounds() {
    use swarm_core::optimizer::polish::{PolishExit, polish_phase_measured};
    let instance = import_instance(serde_json::json!({"name":"polish-zero","strip_height":12.,
        "items":[{"id":0,"demand":1,"allowed_orientations":[0.],
            "shape":{"type":"rectangle","data":{"x_min":0.,"y_min":0.,"width":4.,"height":4.}}}]}));
    let problem = placed_problem(&instance, 30., &[(0, 20., 8.)]);
    let initial = problem.save();
    let mut sep = separator(instance.clone(), problem, 101);
    let mut cfg = DEFAULT_POLISH_CONFIG;
    cfg.time_limit = std::time::Duration::ZERO;
    let (result, stats) = polish_phase_measured(
        &instance,
        &mut sep,
        &initial,
        &mut None,
        &BasicTerminator::new(),
        &cfg,
        &DEFAULT_SWARM_CONFIG.nfp_integration,
        &DEFAULT_SWARM_CONFIG.lp_solver,
    );
    assert_eq!(stats.rounds, 0);
    assert_eq!(stats.changes, 0);
    assert_eq!(stats.exit, PolishExit::Killed);
    assert_eq!(result.strip(), initial.strip());
    for (key, p) in initial.layout_snapshot().placed_items() {
        assert_eq!(
            result.layout_snapshot().placed_items()[key].d_transf(),
            p.d_transf()
        );
    }
    assert_separator_sync(&sep);
}

#[test]
fn infeasible_polish_input_skips_the_active_time_budget() {
    use swarm_core::optimizer::polish::{PolishExit, polish_phase_measured};
    use std::time::{Duration, Instant};
    let instance = import_instance(
        serde_json::json!({"name":"polish-infeasible","strip_height":12.,
        "items":([0,1].map(|id| serde_json::json!({"id":id,"demand":1,"allowed_orientations":[0.],
            "shape":{"type":"rectangle","data":{"x_min":0.,"y_min":0.,"width":4.,"height":4.}}})))}),
    );
    let mut problem = SPProblem::new(instance.clone()).expect("strip forms a valid container");
    problem.change_strip_width(30.).expect("strip width must be positive");
    for item_id in 0..2 {
        problem.place_item(SPPlacement {
            item_idx: item_id,
            d_transf: DTransformation::new(0., (3., 3.)),
        });
    }
    let initial = problem.save();
    let mut sep = separator(instance.clone(), problem, 101);
    let mut cfg = DEFAULT_POLISH_CONFIG;
    cfg.time_limit = Duration::from_secs(2);
    let started = Instant::now();
    let (result, stats) = polish_phase_measured(
        &instance,
        &mut sep,
        &initial,
        &mut None,
        &BasicTerminator::new(),
        &cfg,
        &DEFAULT_SWARM_CONFIG.nfp_integration,
        &DEFAULT_SWARM_CONFIG.lp_solver,
    );
    assert!(started.elapsed() < Duration::from_millis(500));
    assert_eq!(stats.rounds, 0);
    assert_eq!(stats.changes, 0);
    assert_eq!(stats.exit, PolishExit::RolledBackInfeasible);
    assert_eq!(result.strip(), initial.strip());
    for (key, p) in initial.layout_snapshot().placed_items() {
        assert_eq!(
            result.layout_snapshot().placed_items()[key].d_transf(),
            p.d_transf()
        );
    }
    assert_separator_sync(&sep);
}

#[test]
fn polish_stop_after_pass_checkpoint_keeps_the_published_improvement() {
    use swarm_core::util::{
        listener::{ReportType, SolutionListener},
        terminator::Terminator,
    };
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
        best: Option<SPSolution>,
    }
    impl SolutionListener for Listener {
        fn report(&mut self, r: ReportType, s: &SPSolution, i: &SPInstance) {
            if r == ReportType::CmprFeas {
                assert_complete_exact(i, s);
                self.best = Some(s.clone());
                self.stop.store(true, Ordering::Relaxed);
            }
        }
    }
    let instance = import_instance(serde_json::json!({"name":"checkpoint","strip_height":12.,
        "items":([0,1].map(|id| serde_json::json!({"id":id,"demand":1,"allowed_orientations":[0.],
            "shape":{"type":"rectangle","data":{"x_min":0.,"y_min":0.,"width":4.,"height":4.}}})))}));
    let problem = placed_problem(&instance, 30., &[(0, 3., 3.), (1, 20., 8.)]);
    let initial = problem.save();
    let mut sep = separator(instance.clone(), problem, 101);
    let flag = Arc::new(AtomicBool::new(false));
    let term = Stop(flag.clone());
    let mut listener = Listener {
        stop: flag,
        best: None,
    };
    let mut config = DEFAULT_POLISH_CONFIG;
    config.enabled = true;
    config.lp.enabled = false;
    config.slide.enabled = true;
    config.hole_fill.enabled = true;
    let result = swarm_core::optimizer::polish::polish_phase_with_nfp(
        &instance,
        &mut sep,
        &initial,
        &mut Some(&mut listener),
        &term,
        &config,
        &DEFAULT_SWARM_CONFIG.nfp_integration,
        &DEFAULT_SWARM_CONFIG.lp_solver,
    );
    let published = listener
        .best
        .expect("slide must publish its completed checkpoint before hole fill");
    assert!(result.strip_width() < initial.strip_width());
    assert_eq!(result.strip_width(), published.strip_width());
    for (key, p) in published.layout_snapshot().placed_items() {
        assert_eq!(
            result.layout_snapshot().placed_items()[key].d_transf(),
            p.d_transf()
        );
    }
    assert_complete_exact(&instance, &result);
    assert_separator_sync(&sep);
}

#[test]
fn compression_positive_path_reduces_width_and_returns_an_exact_complete_solution() {
    let instance = import_instance(serde_json::json!({
        "name":"compression-positive",
        "strip_height":12.0,
        "items":[
            {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
            }},
            {"id":1,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
            }}
        ]
    }));
    let problem = placed_problem(&instance, 30.0, &[(0, 3.0, 3.0), (1, 20.0, 8.0)]);
    let initial = problem.save();
    let mut sep = separator(instance.clone(), problem, 101);
    let mut config = DEFAULT_SWARM_CONFIG.cmpr_cfg;
    config.shrink_range = (0.10, 0.10);
    config.shrink_decay = ShrinkDecayStrategy::FailureBased(0.5);
    config.separator_config = sep.config;

    let result = compression_phase(
        &instance,
        &mut sep,
        &initial,
        &mut None,
        &BasicTerminator::new(),
        &config,
    );

    assert_complete_exact(&instance, &result);
    assert_separator_sync(&sep);
    assert!(
        result.strip_width() < initial.strip_width() - 1.0e-4,
        "positive compression fixture must exercise a successful shrink: {} -> {}",
        initial.strip_width(),
        result.strip_width()
    );
}

/// I-07: slide 단독 회귀 — 우측에 떨어져 있는 조각이 좌/하 슬라이딩으로 밀착되어
/// 폭이 줄어야 한다 (P4 사이클이 slide_pass 를 단독으로 쓰는 경로 보호).
#[test]
fn slide_positive_path_pulls_a_detached_item_left_and_reduces_width() {
    use swarm_core::optimizer::polish::slide::slide_pass;

    let instance = import_instance(serde_json::json!({
        "name":"slide-positive",
        "strip_height":12.0,
        "items":[
            {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
            }},
            {"id":1,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
            }}
        ]
    }));
    // item 1 은 우측에 떠 있다 — 좌측(그리고 하단)으로 끌려와야 한다.
    let problem = placed_problem(&instance, 30.0, &[(0, 3.0, 3.0), (1, 20.0, 8.0)]);
    let initial = problem.save();
    let mut sep = separator(instance.clone(), problem, 7);

    let moved = slide_pass(
        &mut sep,
        &DEFAULT_POLISH_CONFIG.slide,
        &BasicTerminator::new(),
    );
    sep.refit_strip();
    let result = sep.prob.save();

    assert!(
        moved > 0,
        "detached fixture must trigger at least one slide move"
    );
    assert_complete_exact(&instance, &result);
    assert_separator_sync(&sep);
    assert!(
        result.strip_width() < initial.strip_width() - 1.0e-4,
        "slide must reduce the strip width: {} -> {}",
        initial.strip_width(),
        result.strip_width()
    );
}

/// I-07: 이미 밀착된 레이아웃에서 slide 는 no-op 이어야 한다 —
/// 1차 호출로 수렴시킨 뒤 2차 호출이 min_move_ratio 미만의 미세 이동을
/// 새 이동으로 세지 않는지 확인한다 (수렴 안정성).
#[test]
fn slide_is_a_noop_on_an_already_converged_layout() {
    use swarm_core::optimizer::polish::slide::slide_pass;

    let instance = import_instance(serde_json::json!({
        "name":"slide-noop",
        "strip_height":12.0,
        "items":[
            {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
            }},
            {"id":1,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
            }}
        ]
    }));
    let problem = placed_problem(&instance, 30.0, &[(0, 3.0, 3.0), (1, 20.0, 8.0)]);
    let mut sep = separator(instance.clone(), problem, 7);
    let term = BasicTerminator::new();

    // 1차: 밀착 수렴 (내부적으로 이동이 없어질 때까지 pass 를 반복한다)
    let _ = slide_pass(&mut sep, &DEFAULT_POLISH_CONFIG.slide, &term);
    sep.refit_strip();
    let converged = sep.prob.save();

    // 2차: 수렴 상태에서는 어떤 이동도 등록되면 안 된다
    let moved = slide_pass(&mut sep, &DEFAULT_POLISH_CONFIG.slide, &term);
    let result = sep.prob.save();

    assert_eq!(
        moved, 0,
        "converged layout must not register further slide moves"
    );
    assert_complete_exact(&instance, &result);
    assert!(
        (result.strip_width() - converged.strip_width()).abs() < 1.0e-4,
        "no-op slide must leave the width unchanged"
    );
}

#[test]
fn retired_hole_fill_preserves_closed_pocket_layout() {
    let instance = import_instance(serde_json::json!({
        "name":"hole-fill-positive",
        "strip_height":20.0,
        "items":[
            {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":2.0,"height":12.0}
            }},
            {"id":1,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":2.0,"height":12.0}
            }},
            {"id":2,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":7.8,"height":2.0}
            }},
            {"id":3,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":7.8,"height":2.0}
            }},
            {"id":4,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":2.0,"height":2.0}
            }}
        ]
    }));
    let problem = placed_problem(
        &instance,
        30.0,
        &[
            (0, 5.0, 10.0),
            (1, 15.0, 10.0),
            (2, 10.0, 5.0),
            (3, 10.0, 15.0),
            (4, 25.0, 9.0),
        ],
    );
    let mut sep = separator(instance.clone(), problem, 202);
    let candidate_before = sep
        .prob
        .layout()
        .placed_items()
        .values()
        .find(|placed| placed.item().idx() == 4)
        .unwrap()
        .shape()
        .bbox()
        .x_max;
    let mut config = DEFAULT_POLISH_CONFIG.hole_fill;
    config.cell_size_ratio = 0.5;
    config.max_cells = 20_000;
    config.candidate_band = 0.5;
    config.n_samples_per_pocket = 256;
    config.n_coord_descents = 1;

    let moved = hole_fill_pass(&mut sep, &config, &BasicTerminator::new());
    let candidate_after = sep
        .prob
        .layout()
        .placed_items()
        .values()
        .find(|placed| placed.item().idx() == 4)
        .unwrap()
        .shape()
        .bbox()
        .x_max;
    let result = sep.prob.save();

    assert_eq!(moved, 0);
    assert_eq!(candidate_after, candidate_before);
    assert_complete_exact(&instance, &result);
    assert!(sep.prob.layout().is_collision_free());
    assert_separator_sync(&sep);
}

#[test]
fn retired_hole_fill_preserves_right_open_layout() {
    let instance = import_instance(serde_json::json!({
        "name":"bay-fill-positive",
        "strip_height":20.0,
        "items":[
            {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":2.0,"height":12.0}
            }},
            {"id":1,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":7.8,"height":2.0}
            }},
            {"id":2,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":7.8,"height":2.0}
            }},
            {"id":3,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":2.0,"height":2.0}
            }}
        ]
    }));
    // Items 0..2 form a U-shaped concavity whose mouth is open to the right.
    // There is deliberately no fully enclosed pocket in this fixture.
    let problem = placed_problem(
        &instance,
        30.0,
        &[
            (0, 5.0, 10.0),
            (1, 10.0, 5.0),
            (2, 10.0, 15.0),
            (3, 25.0, 9.0),
        ],
    );
    let mut sep = separator(instance.clone(), problem, 212);
    let candidate_before = sep
        .prob
        .layout()
        .placed_items()
        .values()
        .find(|placed| placed.item().idx() == 3)
        .unwrap()
        .shape()
        .bbox()
        .x_max;
    let mut config = DEFAULT_POLISH_CONFIG.hole_fill;
    config.cell_size_ratio = 0.5;
    config.max_cells = 20_000;
    config.candidate_band = 0.5;
    config.n_samples_per_pocket = 256;
    config.n_coord_descents = 3;

    let moved = hole_fill_pass(&mut sep, &config, &BasicTerminator::new());
    let candidate_after = sep
        .prob
        .layout()
        .placed_items()
        .values()
        .find(|placed| placed.item().idx() == 3)
        .unwrap()
        .shape()
        .bbox()
        .x_max;
    let result = sep.prob.save();

    assert_eq!(moved, 0);
    assert_eq!(candidate_after, candidate_before);
    assert_complete_exact(&instance, &result);
    assert!(sep.prob.layout().is_collision_free());
    assert_separator_sync(&sep);
}

#[test]
fn lp_positive_path_moves_items_left_and_reduces_the_refitted_width() {
    let instance = import_instance(serde_json::json!({
        "name":"lp-positive",
        "strip_height":16.0,
        "items":[
            {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
            }},
            {"id":1,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
            }}
        ]
    }));
    let problem = placed_problem(&instance, 30.0, &[(0, 8.0, 3.0), (1, 20.0, 11.0)]);
    let width_before = problem.strip_width();
    let x_before: Vec<f32> = problem
        .layout()
        .placed_items()
        .values()
        .map(|placed| placed.shape().bbox().x_min)
        .collect();
    let mut sep = separator(instance.clone(), problem, 303);
    let mut config = DEFAULT_POLISH_CONFIG.lp;
    config.n_iterations = 1;
    config.trust_region_ratio = 0.2;

    let applied = lp_compaction_pass(&mut sep, &config, &BasicTerminator::new());
    let x_after: Vec<f32> = sep
        .prob
        .layout()
        .placed_items()
        .values()
        .map(|placed| placed.shape().bbox().x_min)
        .collect();
    let result = sep.prob.save();

    assert_eq!(applied, 1, "the synthetic LP must apply its solved step");
    assert!(sep.prob.strip_width() < width_before - 1.0e-4);
    assert!(
        x_after
            .iter()
            .zip(x_before.iter())
            .any(|(after, before)| after < &(before - 1.0e-4)),
        "LP must move at least one item left"
    );
    assert_complete_exact(&instance, &result);
    assert!(sep.prob.layout().is_collision_free());
    assert_separator_sync(&sep);
}

#[test]
fn direct_optimizer_rejects_an_incomplete_warm_start_and_returns_exact_output() {
    let instance = import_instance(serde_json::json!({
        "name":"invalid-warm-start",
        "strip_height":12.0,
        "items":[
            {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":3.0,"height":3.0}
            }},
            {"id":1,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":3.0,"height":3.0}
            }}
        ]
    }));
    let mut incomplete = SPProblem::new(instance.clone()).expect("strip forms a valid container");
    incomplete.change_strip_width(20.0).expect("strip width must be positive");
    incomplete.place_item(SPPlacement {
        item_idx: 0,
        d_transf: DTransformation::new(0.0, (2.0, 2.0)),
    });
    let warm_start = incomplete.save();

    let mut config = DEFAULT_SWARM_CONFIG;
    config.structure.enabled = false;
    config.expl_cfg.time_limit = std::time::Duration::ZERO;
    config.cmpr_cfg.time_limit = std::time::Duration::ZERO;
    config.polish_cfg.enabled = false;
    config.polish_cfg.time_limit = std::time::Duration::ZERO;
    config.expl_cfg.separator_config.n_workers = 1;
    config.cmpr_cfg.separator_config.n_workers = 1;
    let mut terminator = BasicTerminator::new();

    let (solution, report) = optimize_configured(
        instance.clone(),
        Xoshiro256PlusPlus::seed_from_u64(404),
        None,
        &mut terminator,
        &config,
        Some(&warm_start),
    );

    assert!(report.is_none());
    assert_complete_exact(&instance, &solution);
}

#[test]
fn retired_hole_fill_preserves_grouped_walls_and_donors() {
    let instance = import_instance(serde_json::json!({
        "name":"hole-fill-positive",
        "strip_height":20.0,
        "items":[
            {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":2.0,"height":12.0}
            }},
            {"id":1,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":2.0,"height":12.0}
            }},
            {"id":2,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":7.8,"height":2.0}
            }},
            {"id":3,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":7.8,"height":2.0}
            }},
            {"id":4,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":2.0,"height":2.0}
            }}
        ]
    }));
    let problem = placed_problem(
        &instance,
        30.0,
        &[
            (0, 5.0, 10.0),
            (1, 15.0, 10.0),
            (2, 10.0, 5.0),
            (3, 10.0, 15.0),
            (4, 25.0, 9.0),
        ],
    );
    let mut sep = separator(instance.clone(), problem, 202);
    use swarm_core::optimizer::place_groups::{PlaceGroupMember, PlaceGroupSpec, PlaceGroups};
    let members = sep
        .prob
        .layout()
        .placed_items()
        .values()
        .filter(|pi| pi.item().idx() < 4)
        .map(|pi| PlaceGroupMember {
            item_id: pi.item().idx(),
            initial_transform: pi.d_transf(),
        })
        .collect();
    sep.set_place_groups(
        PlaceGroups::new(&instance, vec![PlaceGroupSpec { id: 7, members }]).unwrap(),
    )
    .unwrap();
    // The pocket walls are static obstacles; a group alone is now movable by
    // the joint pass. Freezing one member pins the entire rigid unit.
    sep.freeze_items(&[0]);
    let initial = sep.prob.save();
    let candidate_before = sep
        .prob
        .layout()
        .placed_items()
        .values()
        .find(|placed| placed.item().idx() == 4)
        .unwrap()
        .shape()
        .bbox()
        .x_max;
    let mut config = DEFAULT_POLISH_CONFIG.hole_fill;
    config.cell_size_ratio = 0.5;
    config.max_cells = 20_000;
    config.candidate_band = 0.5;
    config.n_samples_per_pocket = 256;
    config.n_coord_descents = 1;

    let moved = hole_fill_pass(&mut sep, &config, &BasicTerminator::new());
    let candidate_after = sep
        .prob
        .layout()
        .placed_items()
        .values()
        .find(|placed| placed.item().idx() == 4)
        .unwrap()
        .shape()
        .bbox()
        .x_max;
    let result = sep.prob.save();

    assert_eq!(moved, 0);
    assert_eq!(candidate_after, candidate_before);
    assert_complete_exact(&instance, &result);
    assert!(sep.prob.layout().is_collision_free());
    assert_separator_sync(&sep);
    assert!(sep.solution_is_certified(&sep.prob.save(), &initial));
    for pi in initial
        .layout_snapshot()
        .placed_items()
        .values()
        .filter(|pi| pi.item().idx() < 4)
    {
        assert_eq!(
            pi.d_transf(),
            sep.prob
                .layout()
                .placed_items()
                .values()
                .find(|q| q.item().idx() == pi.item().idx())
                .unwrap()
                .d_transf()
        );
    }
}

#[test]
fn nfp_left_jumps_over_a_frozen_blocker_that_stops_slide() {
    use swarm_core::optimizer::polish::nfp_left::nfp_left_pass;
    use swarm_core::optimizer::polish::slide::slide_pass;
    use swarm_core::optimizer::separator::build_nfp_sampler_cache;
    let rect = |id: usize, w: f32, h: f32| {
        serde_json::json!({"id":id,"demand":1,"allowed_orientations":[0.],
            "shape":{"type":"rectangle","data":{"x_min":0.,"y_min":0.,"width":w,"height":h}}})
    };
    let instance = import_instance(
        serde_json::json!({"name":"nfp-left-jump","strip_height":12.,
        "items":[rect(0, 4., 8.), rect(1, 4., 4.)]}),
    );
    // 자세는 도형 중심 기준이다. 고정된 기둥(0, x 10~14)이 조각(1, x 16~20)의 왼쪽 길을
    // 막고, 그 너머(x 0~10)는 비어 있다.
    let layout = [(0, 12., 6.), (1, 18., 6.)];
    let x_of = |sep: &Separator, id: usize| {
        sep.prob
            .layout()
            .placed_items()
            .values()
            .find(|p| p.item().idx() == id)
            .unwrap()
            .d_transf()
            .translation()
            .0
    };

    // 똑바로 미는 slide 는 기둥 오른쪽에서 멈춘다.
    let mut sep = separator(instance.clone(), placed_problem(&instance, 30., &layout), 7);
    sep.freeze_items(&[0]);
    let mut slide = DEFAULT_POLISH_CONFIG.slide;
    slide.slide_down = false;
    slide_pass(&mut sep, &slide, &BasicTerminator::new());
    assert!(
        x_of(&sep, 1) >= 16.0 - 1.0e-3,
        "slide must stop at the blocker"
    );

    // NFP 왼쪽 재배치는 기둥 너머의 빈자리로 건너뛴다.
    let mut sep = separator(instance.clone(), placed_problem(&instance, 30., &layout), 7);
    sep.freeze_items(&[0]);
    let mut cache = build_nfp_sampler_cache(&instance).expect("NFP library");
    let moved = nfp_left_pass(
        &mut sep,
        &DEFAULT_POLISH_CONFIG.nfp_left,
        &mut cache,
        &BasicTerminator::new(),
    );
    assert_eq!(moved, 1);
    assert!(
        x_of(&sep, 1) + 2.0 <= 10.0 + 1.0e-3,
        "item must land left of the blocker, got x={}",
        x_of(&sep, 1)
    );
    assert_eq!(x_of(&sep, 0), 12.0, "frozen blocker must not move");
    assert_eq!(sep.ct.get_total_loss(), 0.0);
    assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));
    assert_complete_exact(&instance, &sep.prob.save());
}

#[test]
fn issue8_converged_lp_retries_caps_and_a_changed_new_call() {
    use swarm_core::optimizer::polish::{PolishExit, polish_phase_measured};
    use std::time::{Duration, Instant};
    let instance = import_instance(
        serde_json::json!({"name":"polish-fixed-point","strip_height":12.,
        "items":[{"id":0,"demand":1,"allowed_orientations":[0.],
        "shape":{"type":"rectangle","data":{"x_min":0.,"y_min":0.,"width":4.,"height":4.}}}]}),
    );
    for cap in [40, 1] {
        // Imported shapes are centered around their reference point.
        let problem = placed_problem(&instance, 10., &[(0, 2.0000002, 2.0000002)]);
        let initial = problem.save();
        let mut sep = separator(instance.clone(), problem, 101);
        let mut cfg = DEFAULT_POLISH_CONFIG;
        cfg.lp.enabled = true;
        cfg.slide.enabled = false;
        cfg.nfp_left.enabled = false;
        cfg.lp.left_pull_weight = 0.;
        cfg.lp.n_iterations = cap;
        cfg.time_limit = Duration::from_millis(80);
        let started = Instant::now();
        let (result, stats) = polish_phase_measured(
            &instance,
            &mut sep,
            &initial,
            &mut None,
            &BasicTerminator::new(),
            &cfg,
            &DEFAULT_SWARM_CONFIG.nfp_integration,
            &DEFAULT_SWARM_CONFIG.lp_solver,
        );
        if cap == 40 {
            // 증명된 고정점은 예산을 기다리지 않고 끝난다.
            assert_eq!(stats.exit, PolishExit::NoChanges, "{stats:?}");
            assert_eq!(
                stats.rounds, 1,
                "complete builtin LP should prove the fixed point: {stats:?}"
            );
        } else {
            assert!(started.elapsed() >= cfg.time_limit);
            assert_eq!(stats.exit, PolishExit::Killed);
            assert!(
                stats.rounds > 1,
                "iteration cap must retain real retries: {stats:?}"
            );
        }
        assert_eq!(result.strip_width(), initial.strip_width());
        for (key, old) in initial.layout_snapshot().placed_items() {
            assert_eq!(
                result.layout_snapshot().placed_items()[key].d_transf(),
                old.d_transf()
            );
        }
        assert_complete_exact(&instance, &result);
        assert_separator_sync(&sep);
        if cap == 40 {
            let key = sep.prob.layout().placed_items().keys().next().unwrap();
            sep.move_item(key, DTransformation::new(0., (4., 4.)));
            let changed = sep.prob.save();
            // Give the changed input time for useful work independently of the
            // original 80ms fixed-point and iteration-cap contracts above.
            // Certified LP steps can survive deadline interruption even before
            // the first round completes, so rounds > 0 is not required here.
            let mut fresh_cfg = cfg;
            fresh_cfg.time_limit = Duration::from_millis(500);
            let fresh_started = Instant::now();
            let (fresh, stats) = polish_phase_measured(
                &instance,
                &mut sep,
                &changed,
                &mut None,
                &BasicTerminator::new(),
                &fresh_cfg,
                &DEFAULT_SWARM_CONFIG.nfp_integration,
                &DEFAULT_SWARM_CONFIG.lp_solver,
            );
            eprintln!(
                "Issue8 fresh-call elapsed={:?} stats={stats:?} widths={}->{} poses={:?}",
                fresh_started.elapsed(),
                changed.strip_width(),
                fresh.strip_width(),
                fresh
                    .layout_snapshot()
                    .placed_items()
                    .values()
                    .map(|p| p.d_transf())
                    .collect::<Vec<_>>()
            );
            // 바뀐 입력은 압축된 뒤 새 고정점에서 끝나거나 마감으로 끝난다.
            assert!(
                matches!(stats.exit, PolishExit::NoChanges | PolishExit::Killed),
                "{stats:?}"
            );
            assert!(
                stats.changes > 0,
                "fresh call: {stats:?}, width {} -> {}",
                changed.strip_width(),
                fresh.strip_width()
            );
            assert!(fresh.strip_width() < changed.strip_width());
            assert_complete_exact(&instance, &fresh);
            assert_separator_sync(&sep);
        }
    }
}
