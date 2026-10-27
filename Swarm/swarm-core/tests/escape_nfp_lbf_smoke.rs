//! 정체 감지 기반 적응형 탈출 + NFP-LBF 초기 배치 종단 스모크 테스트.
//!
//! 실행:
//! ```text
//! cargo test -p swarm-core --features "simd,capmods,adaptive,structure-first" \
//!            --test escape_nfp_lbf_smoke -- --nocapture
//! ```
//!
//! 단위 테스트가 수식·임계값을 검증한다면, 이 파일은 **실제 인스턴스에서 파이프라인이
//! 끝까지 도는지**를 본다.
//!
//! 1. NFP Cache -> Multiple Ordering -> Sequential Placement -> Best Initial Nest
//! 2. Best Initial Nest를 seed로 넣은 탈출 기반 탐색 단계
//! 3. 문서가 정한 성능 목표(콜드 캐시 < 5초) 확인
#![cfg(feature = "adaptive")]

use jagua_rs::probs::spp::entities::SPInstance;
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::escape::{AdaptiveEscapeConfig, escape_driven_exploration_phase};
use swarm_core::nfp_lbf::{NfpLbfConfig, build_best_initial_nest};
use swarm_core::optimizer::lbf::LBFBuilder;
use swarm_core::optimizer::separator::Separator;
use swarm_core::structure::pipeline::solution_validation_error;
use swarm_core::util::io::read_spp_input;
use swarm_core::util::terminator::{BasicTerminator, Terminator};
use std::path::Path;
use std::time::Duration;

fn load(name: &str) -> SPInstance {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../data/input")
        .join(name);
    let (ext_inst, _) = read_spp_input(&path).unwrap_or_else(|e| panic!("failed to read {name}: {e}"));
    let cfg = DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(cfg.cde_config, cfg.poly_simpl_tolerance, cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&importer, &ext_inst)
        .unwrap_or_else(|e| panic!("failed to import {name}: {e}"))
}

fn nfp_lbf_config(n_orderings: usize) -> NfpLbfConfig {
    NfpLbfConfig {
        n_orderings,
        cache_budget: Duration::from_secs(5),
        total_budget: Duration::from_secs(20),
        concave_simplify_tolerance: 0.02,
        ..NfpLbfConfig::default()
    }
}

/// NFP Cache -> Ordering -> Sequential Placement -> Best Initial Nest 가
/// 실제 인스턴스에서 **유효한 해**를 만든다.
#[test]
fn nfp_lbf_builds_a_valid_initial_nest() {
    for name in ["fu.json", "albano.json"] {
        let instance = load(name);
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(0xC0FFEE);
        let cfg = nfp_lbf_config(8);

        let nest = build_best_initial_nest(&instance, &cfg, &mut rng)
            .unwrap_or_else(|| panic!("{name}: NFP-LBF produced no initial nest"));

        // 검증을 통과한 해만 optimizer로 넘어가야 한다.
        assert!(
            solution_validation_error(&instance, &nest.solution).is_none(),
            "{name}: NFP-LBF seed must be an exact, valid solution"
        );

        let r = &nest.report;
        assert!(r.orderings_evaluated > 0, "{name}: no ordering was evaluated");
        assert!(
            r.best_strip_length > 0.0 && r.best_strip_length.is_finite(),
            "{name}: strip length must be finite and positive"
        );
        assert!(
            r.best_density > 0.0 && r.best_density <= 1.0,
            "{name}: density {} out of range",
            r.best_density
        );
        assert!(
            r.cache.unique_geometries > 0,
            "{name}: the prepared library must hold at least one geometry"
        );

        // 문서 목표: 콜드 캐시 < 2초 목표 / 최악 < 5초.
        assert!(
            r.cache.build_ms <= 5_000,
            "{name}: cold NFP cache took {} ms (document budget: 5000 ms)",
            r.cache.build_ms
        );

        println!(
            "[{name}] unique geom {} / pairs {} / prewarm {} in {} ms | orderings {} (prefix reuse {}) \
             | best '{}' len {:.3} dens {:.2}% | cache {} hit / {} miss | total {} ms | unplaced {}",
            r.cache.unique_geometries,
            r.cache.instance_polygons,
            r.cache.prewarm_pairs,
            r.cache.build_ms,
            r.orderings_evaluated,
            r.prefix_reuse_hits,
            r.best_ordering_label,
            r.best_strip_length,
            r.best_density * 100.0,
            r.cache.hits,
            r.cache.misses,
            r.elapsed_ms,
            nest.placement.unplaced.len(),
        );
    }
}

/// 같은 시드로 두 번 돌리면 같은 결과가 나온다 (결정론성).
#[test]
fn nfp_lbf_is_deterministic_for_a_fixed_seed() {
    let instance = load("fu.json");
    let cfg = nfp_lbf_config(6);

    let mut rng_a = Xoshiro256PlusPlus::seed_from_u64(7);
    let a = build_best_initial_nest(&instance, &cfg, &mut rng_a).expect("run A produced no nest");
    let mut rng_b = Xoshiro256PlusPlus::seed_from_u64(7);
    let b = build_best_initial_nest(&instance, &cfg, &mut rng_b).expect("run B produced no nest");

    assert_eq!(a.report.best_ordering_label, b.report.best_ordering_label);
    assert_eq!(a.placement.pieces.len(), b.placement.pieces.len());
    assert!(
        (a.report.best_strip_length - b.report.best_strip_length).abs() < 1.0e-6,
        "same seed must give the same strip length: {} vs {}",
        a.report.best_strip_length,
        b.report.best_strip_length
    );
}

/// 탈출 기반 탐색 단계가 실제로 돌고, feasible 해를 하나 이상 남긴다.
#[test_case::test_case(false; "repair_off")]
#[test_case::test_case(true; "repair_on")]
fn escape_exploration_phase_runs_and_returns_feasible_solutions(repair_enabled: bool) {
    let instance = load("fu.json");
    let rng = Xoshiro256PlusPlus::seed_from_u64(11);
    let builder = LBFBuilder::new(instance.clone(), rng, LBF_SAMPLE_CONFIG, 10.0, 1.0).construct();

    let mut expl_cfg = DEFAULT_SWARM_CONFIG.expl_cfg;
    expl_cfg.repair.direction_enabled = repair_enabled;
    expl_cfg.repair.cluster_lp_enabled = repair_enabled;
    expl_cfg.repair.chain_enabled = repair_enabled;
    expl_cfg.repair.stagnation_iterations = 1;
    let mut sep = Separator::new(
        instance.clone(),
        builder.prob,
        Xoshiro256PlusPlus::seed_from_u64(12),
        expl_cfg.separator_config,
    );

    let start_width = sep.prob.strip_width();

    let mut term = BasicTerminator::new();
    term.new_timeout(Duration::from_secs(6));

    let escape_cfg = AdaptiveEscapeConfig::default();
    let sols = escape_driven_exploration_phase(
        &instance,
        &mut sep,
        &mut None,
        &term,
        &expl_cfg,
        &escape_cfg,
    );

    assert!(
        !sols.is_empty(),
        "the escape-driven exploration phase must return at least one feasible solution"
    );

    let best = sols.last().unwrap();
    assert!(
        solution_validation_error(&instance, best).is_none(),
        "every returned solution must be exact and valid"
    );
    assert!(
        best.strip_width() <= start_width + 1.0e-5,
        "the phase must never widen past its starting width: {} > {}",
        best.strip_width(),
        start_width
    );

    println!(
        "[escape] {} feasible solutions, width {:.3} -> {:.3} ({:.2}% dens)",
        sols.len(),
        start_width,
        best.strip_width(),
        best.density() * 100.0
    );
}

/// Conflict Burst / Swarm 다양화를 끈 구성도 동일하게 동작해야 한다
/// (두 기능이 파이프라인의 필수 전제가 아님을 확인).
#[test]
fn escape_exploration_phase_runs_with_optional_stages_disabled() {
    let instance = load("fu.json");
    let rng = Xoshiro256PlusPlus::seed_from_u64(21);
    let builder = LBFBuilder::new(instance.clone(), rng, LBF_SAMPLE_CONFIG, 10.0, 1.0).construct();

    let expl_cfg = DEFAULT_SWARM_CONFIG.expl_cfg;
    let mut sep = Separator::new(
        instance.clone(),
        builder.prob,
        Xoshiro256PlusPlus::seed_from_u64(22),
        expl_cfg.separator_config,
    );

    let mut term = BasicTerminator::new();
    term.new_timeout(Duration::from_secs(4));

    let escape_cfg = AdaptiveEscapeConfig {
        enable_conflict_burst: false,
        enable_swarm_diversification: false,
        ..AdaptiveEscapeConfig::default()
    };
    let sols = escape_driven_exploration_phase(
        &instance,
        &mut sep,
        &mut None,
        &term,
        &expl_cfg,
        &escape_cfg,
    );

    assert!(!sols.is_empty());
    assert!(solution_validation_error(&instance, sols.last().unwrap()).is_none());
}

/// NFP-LBF seed -> 탈출 탐색까지 이어 붙인 전체 경로.
///
/// ```text
/// NFP Cache -> Multiple Item Ordering -> NFP Sequential Placement
///   -> Best Initial Nest -> Swarm / Local Improvement
/// ```
#[test]
fn nfp_lbf_seed_feeds_the_escape_exploration_phase() {
    let instance = load("fu.json");
    let mut seed_rng = Xoshiro256PlusPlus::seed_from_u64(31);
    let nest = build_best_initial_nest(&instance, &nfp_lbf_config(6), &mut seed_rng)
        .expect("NFP-LBF produced no initial nest");

    let mut prob = jagua_rs::probs::spp::entities::SPProblem::new(instance.clone()).expect("strip forms a valid container");
    prob.restore(&nest.solution);

    let expl_cfg = DEFAULT_SWARM_CONFIG.expl_cfg;
    let mut sep = Separator::new(
        instance.clone(),
        prob,
        Xoshiro256PlusPlus::seed_from_u64(32),
        expl_cfg.separator_config,
    );

    let seed_width = sep.prob.strip_width();

    let mut term = BasicTerminator::new();
    term.new_timeout(Duration::from_secs(6));

    let sols = escape_driven_exploration_phase(
        &instance,
        &mut sep,
        &mut None,
        &term,
        &expl_cfg,
        &AdaptiveEscapeConfig::default(),
    );

    assert!(!sols.is_empty());
    let best = sols.last().unwrap();
    assert!(solution_validation_error(&instance, best).is_none());

    println!(
        "[seed->escape] NFP-LBF width {:.3} -> escape width {:.3} ({:.2}% dens)",
        seed_width,
        best.strip_width(),
        best.density() * 100.0
    );
}
