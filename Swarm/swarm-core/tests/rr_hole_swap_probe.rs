//! Enabled RNR / separator swap / rigid-group exclusion review probes.
//! Uses diagnostic fixtures and budgets; production configuration is unchanged.
#![cfg(all(feature = "capmods", feature = "adaptive"))]

use jagua_rs::probs::spp::entities::SPInstance;
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::optimizer::lbf::LBFBuilder;
#[cfg(feature = "capmods")]
use swarm_core::optimizer::lns::swap_pass;
#[cfg(feature = "adaptive")]
use swarm_core::optimizer::lns::{BandTabu, RuinStrategy, ruin_and_recreate};
use swarm_core::optimizer::separator::Separator;
use swarm_core::util::assertions::tracker_matches_layout;
use swarm_core::util::io::read_spp_input;
use std::path::Path;

fn load_fu_instance() -> SPInstance {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input/fu.json");
    let (ext_inst, _) = read_spp_input(&path).expect("failed to read fu.json");
    let cfg = DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(cfg.cde_config, cfg.poly_simpl_tolerance, cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&importer, &ext_inst)
        .expect("failed to import instance")
}

/// LBF 초기해를 만들고 strip을 강제로 좁혀 충돌 상태의 Separator를 준비한다.
fn build_colliding_separator(seed: u64) -> Separator {
    let instance = load_fu_instance();
    let rng = Xoshiro256PlusPlus::seed_from_u64(seed);
    let builder = LBFBuilder::new(instance.clone(), rng, LBF_SAMPLE_CONFIG, 10.0, 1.0).construct();

    let sep_cfg = DEFAULT_SWARM_CONFIG.expl_cfg.separator_config;
    let mut sep = Separator::new(
        instance,
        builder.prob,
        Xoshiro256PlusPlus::seed_from_u64(seed + 1),
        sep_cfg,
    );

    // 폭을 15% 줄여 우측 아이템들을 좌측으로 밀어 넣는다 → 충돌 발생
    let new_width = sep.prob.strip_width() * 0.85;
    sep.change_strip_width(new_width, None);
    assert!(
        sep.ct.get_total_loss() > 0.0,
        "shrinking the strip should create collisions"
    );
    sep
}

// Diagnostic checks for the enabled-operator review, 2026-09-08.
fn poses(
    sep: &Separator,
) -> std::collections::BTreeMap<usize, jagua_rs::geometry::DTransformation> {
    sep.prob
        .layout()
        .placed_items()
        .values()
        .map(|pi| (pi.item().idx(), pi.d_transf()))
        .collect()
}
#[test]
fn retired_rnr_strategies_preserve_placements() {
    use swarm_core::util::terminator::BasicTerminator;
    for strategy in [
        RuinStrategy::Rightmost,
        RuinStrategy::LowDensity,
        RuinStrategy::Fragmented,
        RuinStrategy::GapTarget,
        RuinStrategy::Random,
        RuinStrategy::DensitySeam,
    ] {
        let mut changed_runs = 0;
        for seed in [7, 42, 103] {
            let mut sep = build_colliding_separator(seed);
            let before = poses(&sep);
            let before_loss = sep.ct.get_total_loss();
            let count = ruin_and_recreate(
                &mut sep,
                &DEFAULT_SWARM_CONFIG.expl_cfg.rnr,
                strategy,
                &mut BandTabu::new(),
                &BasicTerminator::new(),
            );
            let after = poses(&sep);
            let changed = before
                .iter()
                .filter(|(id, dt)| after.get(id) != Some(dt))
                .count();
            println!(
                "RNR {strategy:?} seed={seed} returned={count} changed={changed} loss={before_loss:.4}->{:.4}",
                sep.ct.get_total_loss()
            );
            assert_eq!(before.len(), after.len());
            assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));
            changed_runs += usize::from(changed > 0);
        }
        assert_eq!(changed_runs, 0, "retired {strategy:?} must be inert");
    }
}
#[test]
fn retired_rnr_preserves_grouped_and_ungrouped_items() {
    use swarm_core::optimizer::place_groups::{PlaceGroupMember, PlaceGroupSpec, PlaceGroups};
    use swarm_core::util::terminator::BasicTerminator;
    let mut sep = build_colliding_separator(42);
    let members = sep
        .prob
        .layout()
        .placed_items()
        .values()
        .take(2)
        .map(|pi| PlaceGroupMember {
            item_id: pi.item().idx(),
            initial_transform: pi.d_transf(),
        })
        .collect();
    let groups = PlaceGroups::new(&sep.instance, vec![PlaceGroupSpec { id: 1, members }]).unwrap();
    sep.set_place_groups(groups).unwrap();
    let before = poses(&sep);
    let term = BasicTerminator::new();
    let count = ruin_and_recreate(
        &mut sep,
        &DEFAULT_SWARM_CONFIG.expl_cfg.rnr,
        RuinStrategy::Rightmost,
        &mut BandTabu::new(),
        &term,
    );
    assert_eq!(count, 0);
    let after = poses(&sep);
    assert_eq!(before, after);
    for (id, dt) in &before {
        if sep.place_groups.group_id_of(*id).is_some() {
            assert_eq!(after.get(id), Some(dt));
        }
    }
    assert!(
        sep.place_groups
            .solution_preserves_relative_poses(&sep.prob.save())
    );
    assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));
}

#[test]
fn retired_swap_preserves_grouped_and_ungrouped_items() {
    use swarm_core::optimizer::place_groups::{PlaceGroupMember, PlaceGroupSpec, PlaceGroups};
    use swarm_core::util::terminator::BasicTerminator;
    let mut accepted = 0;
    for seed in [7, 42, 103] {
        let mut sep = build_colliding_separator(seed);
        let members = sep
            .prob
            .layout()
            .placed_items()
            .values()
            .take(2)
            .map(|pi| PlaceGroupMember {
                item_id: pi.item().idx(),
                initial_transform: pi.d_transf(),
            })
            .collect();
        sep.set_place_groups(
            PlaceGroups::new(&sep.instance, vec![PlaceGroupSpec { id: 1, members }]).unwrap(),
        )
        .unwrap();
        let before = poses(&sep);
        for _ in 0..160 {
            let raw = sep.ct.get_total_loss();
            let weighted = sep.ct.get_total_weighted_loss();
            accepted += swap_pass(&mut sep, 1, &BasicTerminator::new());
            assert!(
                sep.ct.get_total_loss() <= raw + raw.abs().max(1.0) * 1e-5,
                "raw {raw} -> {}",
                sep.ct.get_total_loss()
            );
            assert!(sep.ct.get_total_weighted_loss() <= weighted + weighted.abs().max(1.0) * 1e-5);
        }
        let after = poses(&sep);
        for (id, dt) in &before {
            if sep.place_groups.group_id_of(*id).is_some() {
                assert_eq!(after.get(id), Some(dt));
            }
        }
        assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));
    }
    assert_eq!(accepted, 0);
    println!("Grouped separator swap accepted={accepted}");
}

#[test]
fn zero_tracker_loss_cannot_publish_an_infeasible_initial_layout() {
    use swarm_core::optimizer::explore_mods::exploration_phase_adaptive;
    use swarm_core::quantify::tracker::CollisionTracker;
    use swarm_core::util::listener::{ReportType, SolutionListener};
    use swarm_core::util::terminator::{BasicTerminator, Terminator};
    struct Listener(usize);
    impl SolutionListener for Listener {
        fn report(
            &mut self,
            kind: ReportType,
            solution: &jagua_rs::probs::spp::entities::SPSolution,
            instance: &SPInstance,
        ) {
            if kind == ReportType::ExplFeas {
                assert!(
                    swarm_core::structure::pipeline::solution_validation_error(
                        instance, solution
                    )
                    .is_none()
                );
                self.0 += 1;
            }
        }
    }
    let mut sep = build_colliding_separator(42);
    let initial = sep.prob.save();
    assert!(!sep.solution_is_certified(&initial, &initial));
    // Keep valid tracker keys, but simulate stale collision exemptions.
    sep.ct = CollisionTracker::new_with_frozen(
        &sep.prob.layout(),
        &vec![true; sep.instance.total_item_qty()],
    );
    let mut listener = Listener(0);
    let instance = sep.instance.clone();
    let mut term = BasicTerminator::new();
    term.new_timeout(std::time::Duration::from_millis(100));
    let sols = exploration_phase_adaptive(
        &instance,
        &mut sep,
        &mut Some(&mut listener),
        &term,
        &DEFAULT_SWARM_CONFIG.expl_cfg,
    );
    // A failed certification is retried within the bounded phase budget.
    for sol in &sols {
        assert!(sep.solution_is_certified(sol, &initial));
    }
    assert_eq!(listener.0, sols.len());
    assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));
}

#[test]
fn retired_rnr_is_not_observed_in_the_adaptive_explore_path() {
    use swarm_core::optimizer::explore_mods::exploration_phase_adaptive;
    use swarm_core::util::terminator::{BasicTerminator, Terminator};
    use std::time::Duration;

    let instance = load_fu_instance();
    let rng = Xoshiro256PlusPlus::seed_from_u64(7);
    let builder = LBFBuilder::new(instance.clone(), rng, LBF_SAMPLE_CONFIG, 10.0, 1.0).construct();
    let mut expl_cfg = DEFAULT_SWARM_CONFIG.expl_cfg;
    // Stale presets must not reactivate the removed operator.
    expl_cfg.rnr.period = 1;
    // 디버그 빌드에서 separate 1회가 시간 예산을 다 먹지 않도록 한도를 줄여
    // 바깥 루프(성공/실패/disruption)가 여러 번 돌게 만든다.
    expl_cfg.separator_config.iter_no_imprv_limit = 20;
    expl_cfg.separator_config.strike_limit = 1;
    let mut sep = Separator::new(
        instance.clone(),
        builder.prob,
        Xoshiro256PlusPlus::seed_from_u64(8),
        expl_cfg.separator_config,
    );

    // 한계 밀도(~87%) 근처까지 미리 조여, 분리 실패 → disruption(ALNS)/폭 진동 경로가
    // 시간 안에 실제로 실행되도록 만든다.
    let (w0, d0) = (sep.prob.strip_width(), sep.prob.density());
    let w_tight = w0 * (d0 / 0.87).min(1.0);
    if w_tight < w0 {
        sep.change_strip_width(w_tight, None);
    }

    let mut term = BasicTerminator::new();
    term.new_timeout(Duration::from_secs(15));
    let sols = exploration_phase_adaptive(&instance, &mut sep, &mut None, &term, &expl_cfg);

    assert!(
        !sols.is_empty(),
        "this fixture must reach a certified solution"
    );
    for sol in &sols {
        assert!(sep.solution_is_certified(sol, sol));
    }
    assert!(sep.solution_is_certified(&sep.prob.save(), sols.last().unwrap()));
    let n_items = instance.total_item_qty();
    let last = sols.last().unwrap();
    assert_eq!(
        last.layout_snapshot().placed_items().len(),
        n_items,
        "all items must remain placed"
    );
    assert!(
        last.strip_width() <= sols.first().unwrap().strip_width(),
        "width must not regress"
    );
}
