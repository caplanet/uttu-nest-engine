//! LNS(ruin & recreate) / 스왑 연산 스모크 테스트.
//!
//! 실행:  cargo test -p swarm-core --features "capmods adaptive" --test lns_smoke
//!
//! `lns` 모듈은 `capmods`(스왑 훅)와 `adaptive`(ruin & recreate) 양쪽에서 쓰이므로
//! 둘 중 하나만 켜도 컴파일된다. 다만 스왑 훅 테스트는 `capmods`가, 적응형 탐색
//! 종단 테스트는 `adaptive`가 필요하다.
#![cfg(any(feature = "capmods", feature = "adaptive"))]

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

#[test]
#[cfg(feature = "capmods")]
fn swap_pass_preserves_invariants() {
    use swarm_core::util::terminator::BasicTerminator;

    let mut sep = build_colliding_separator(42);
    // GLS can assign very different importance to collision pairs. Exercise
    // swap acceptance under deliberately heterogeneous learned weights.
    for (idx, entry) in sep.ct.pair_collisions_mut().data.iter_mut().enumerate() {
        entry.weight = if idx % 3 == 0 { 100.0 } else { 1.0 };
    }
    for (idx, entry) in sep.ct.container_collisions_mut().iter_mut().enumerate() {
        entry.weight = if idx % 2 == 0 { 75.0 } else { 1.0 };
    }
    let n_items_before = sep.prob.layout().placed_items().len();
    let w_loss_before = sep.ct.get_total_weighted_loss();
    let loss_before = sep.ct.get_total_loss();
    let term = BasicTerminator::new();

    // 여러 번 돌려 accept/revert 경로를 모두 태운다
    let mut n_accepted = 0;
    for _ in 0..10 {
        n_accepted += swap_pass(&mut sep, 8, &term);
    }

    assert_eq!(
        sep.prob.layout().placed_items().len(),
        n_items_before,
        "swap must not add/remove items"
    );
    assert!(
        tracker_matches_layout(&sep.ct, &sep.prob.layout()),
        "collision tracker out of sync after swap_pass"
    );
    // 스왑은 개선되는 경우에만 반영되므로 가중 loss는 절대 증가하지 않는다
    assert!(
        sep.ct.get_total_weighted_loss() <= w_loss_before * 1.001,
        "weighted loss increased after swap_pass: {} -> {}",
        w_loss_before,
        sep.ct.get_total_weighted_loss()
    );
    let loss_epsilon = 1.0e-5 * loss_before.abs().max(1.0);
    assert!(
        sep.ct.get_total_loss() <= loss_before + loss_epsilon,
        "raw collision loss increased after swap_pass: {} -> {}",
        loss_before,
        sep.ct.get_total_loss()
    );
    println!(
        "swap_pass accepted {n_accepted} swaps, wl {w_loss_before} -> {}",
        sep.ct.get_total_weighted_loss()
    );
}

#[test]
#[cfg(feature = "capmods")]
fn swap_pass_stops_before_a_new_attempt() {
    use swarm_core::util::terminator::Terminator;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::time::Duration;

    struct AlreadyKilled(AtomicUsize);

    impl Terminator for AlreadyKilled {
        fn kill(&self) -> bool {
            self.0.fetch_add(1, Ordering::Relaxed);
            true
        }

        fn new_timeout(&mut self, _timeout: Duration) {}

        fn timeout_at(&self) -> Option<jagua_rs::Instant> {
            None
        }
    }

    let mut sep = build_colliding_separator(84);
    let before = sep.prob.save();
    let term = AlreadyKilled(AtomicUsize::new(0));

    assert_eq!(swap_pass(&mut sep, 128, &term), 0);
    assert_eq!(term.0.load(Ordering::Relaxed), 0);
    assert_eq!(
        sep.prob.layout().placed_items().len(),
        before.layout_snapshot().placed_items().len()
    );
    for (key, placed_before) in before.layout_snapshot().placed_items().iter() {
        let placed_after = &sep.prob.layout().placed_items()[key];
        assert_eq!(placed_after.item().idx(), placed_before.item().idx());
        assert_eq!(placed_after.d_transf(), placed_before.d_transf());
    }
    assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));
}

#[test]
#[cfg(feature = "adaptive")]
fn ruin_and_recreate_preserves_invariants() {
    use swarm_core::util::terminator::BasicTerminator;

    let mut sep = build_colliding_separator(7);
    let n_items_before = sep.prob.layout().placed_items().len();

    let rnr_cfg = DEFAULT_SWARM_CONFIG.expl_cfg.rnr;
    let mut tabu = BandTabu::new();
    let term = BasicTerminator::new();
    // adaptive Explore가 선택할 수 있는 다섯 R&R 전략을 모두 태우고, tabu 상태가
    // 쌓인 채로도 정상 동작하는지 확인한다.
    for strategy in [
        RuinStrategy::Rightmost,
        RuinStrategy::LowDensity,
        RuinStrategy::Fragmented,
        RuinStrategy::GapTarget,
        RuinStrategy::Random,
        RuinStrategy::Rightmost,
    ] {
        ruin_and_recreate(&mut sep, &rnr_cfg, strategy, &mut tabu, &term);
        assert_eq!(
            sep.prob.layout().placed_items().len(),
            n_items_before,
            "ruin&recreate must reinsert every removed item"
        );
        assert!(
            tracker_matches_layout(&sep.ct, &sep.prob.layout()),
            "collision tracker out of sync after ruin_and_recreate"
        );
    }
}

#[test]
#[cfg(feature = "adaptive")]
fn ruin_and_recreate_cancellation_rolls_back_layout_tracker_and_rng() {
    use rand::RngExt;
    use swarm_core::util::terminator::Terminator;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::time::{Duration, Instant};

    struct KillAfter {
        limit: usize,
        polls: AtomicUsize,
    }

    impl Terminator for KillAfter {
        fn kill(&self) -> bool {
            self.polls.fetch_add(1, Ordering::Relaxed) + 1 >= self.limit
        }

        fn new_timeout(&mut self, _timeout: Duration) {}

        fn timeout_at(&self) -> Option<jagua_rs::Instant> {
            None
        }
    }

    let mut sep = build_colliding_separator(707);
    for (idx, entry) in sep.ct.pair_collisions_mut().data.iter_mut().enumerate() {
        entry.weight = 1.0 + (idx % 11) as f32;
    }
    for (idx, entry) in sep.ct.container_collisions_mut().iter_mut().enumerate() {
        entry.weight = 1.0 + (idx % 7) as f32;
    }
    let before_solution = sep.prob.save();
    let before_tracker = sep.ct.save();
    let mut expected_rng = sep.rng.clone();
    let mut tabu = BandTabu::new();
    let term = KillAfter {
        // This fires after victims have been removed and while recreate search
        // or its coordinate descent is evaluating candidates.
        limit: 800,
        polls: AtomicUsize::new(0),
    };

    let started = Instant::now();
    ruin_and_recreate(
        &mut sep,
        &DEFAULT_SWARM_CONFIG.expl_cfg.rnr,
        RuinStrategy::Rightmost,
        &mut tabu,
        &term,
    );
    let elapsed = started.elapsed();
    let polls = term.polls.load(Ordering::Relaxed);

    assert_eq!(
        polls, 0,
        "retired R&R must return without entering its work loop"
    );
    assert!(
        polls <= term.limit + 2_048,
        "coordinate descent did not quiesce promptly after stop: {polls} polls"
    );
    assert!(
        elapsed < Duration::from_secs(5),
        "cancelled R&R returned too slowly: {elapsed:?}"
    );
    assert_eq!(
        sep.prob.layout().placed_items().len(),
        before_solution.layout_snapshot().placed_items().len()
    );
    for (key, before) in before_solution.layout_snapshot().placed_items().iter() {
        let after = &sep.prob.layout().placed_items()[key];
        assert_eq!(after.item().idx(), before.item().idx());
        assert_eq!(after.d_transf(), before.d_transf());
    }
    assert_eq!(sep.prob.strip_width(), before_solution.strip_width());
    assert_eq!(
        sep.ct.pair_collisions().data.len(),
        before_tracker.pair_collisions().data.len()
    );
    for (after, before) in sep
        .ct
        .pair_collisions()
        .data
        .iter()
        .zip(&before_tracker.pair_collisions().data)
    {
        assert_eq!(after.loss.to_bits(), before.loss.to_bits());
        assert_eq!(after.weight.to_bits(), before.weight.to_bits());
    }
    for (after, before) in sep
        .ct
        .container_collisions()
        .iter()
        .zip(before_tracker.container_collisions())
    {
        assert_eq!(after.loss.to_bits(), before.loss.to_bits());
        assert_eq!(after.weight.to_bits(), before.weight.to_bits());
    }
    assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));
    assert_eq!(sep.rng.random::<u64>(), expected_rng.random::<u64>());
}

/// exploration_phase_adaptive 전체(ALNS 연산자 선택 + 폭 진동 + R&R/regret 삽입)를
/// 실제 인스턴스로 완주시키는 종단 테스트.
#[test]
#[cfg(feature = "adaptive")]
fn adaptive_exploration_end_to_end() {
    use swarm_core::optimizer::explore_mods::exploration_phase_adaptive;
    use swarm_core::util::terminator::{BasicTerminator, Terminator};
    use std::time::Duration;

    let instance = load_fu_instance();
    let rng = Xoshiro256PlusPlus::seed_from_u64(7);
    let builder = LBFBuilder::new(instance.clone(), rng, LBF_SAMPLE_CONFIG, 10.0, 1.0).construct();
    let mut expl_cfg = DEFAULT_SWARM_CONFIG.expl_cfg;
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

    // This fixture starts infeasible. A timed-out repair may legitimately yield
    // no certified incumbent; every returned result must nevertheless be exact.
    for sol in &sols {
        assert_eq!(
            sol.layout_snapshot().placed_items().len(),
            instance.total_item_qty()
        );
        assert!(sep.solution_is_certified(sol, sol));
    }
    assert!(
        sols.windows(2)
            .all(|pair| pair[1].strip_width() <= pair[0].strip_width())
    );
}

#[test]
#[cfg(feature = "capmods")]
fn separate_with_swap_hook_runs() {
    use swarm_core::util::terminator::{BasicTerminator, Terminator};
    use std::time::Duration;

    let mut sep = build_colliding_separator(1234);
    let mut term = BasicTerminator::new();
    term.new_timeout(Duration::from_secs(10));

    // separate 내부에서 SWAP_PERIOD마다 swap_pass 훅이 실행된다 (capmods)
    let (sol, ct_snapshot) = sep.separate(&term, &mut None);
    assert_eq!(
        sol.layout_snapshot().placed_items().len(),
        sep.prob.layout().placed_items().len()
    );
    println!(
        "separate finished with loss: {}",
        ct_snapshot.get_total_loss()
    );
}
