//! 폭 완화 복구(width relief)를 켠 탐색 단계 종단 테스트.
//!
//! 실행:  cargo test -p swarm-core --features "capmods adaptive" --test width_relief
//!
//! 단위 테스트(`optimizer::explore_mods::tests`)가 복구 함수 자체를 검증한다면,
//! 여기서는 탐색 루프에 배선된 상태 — 폭 카운터 / infeasible 풀 / 인컴번트 — 가
//! 복구 전후로 깨지지 않는지 본다.
#![cfg(feature = "adaptive")]

use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::{DEFAULT_SWARM_CONFIG, ExplorationConfig};
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::optimizer::explore_mods::exploration_phase_adaptive;
use swarm_core::optimizer::lbf::LBFBuilder;
use swarm_core::optimizer::separator::Separator;
use swarm_core::structure::pipeline::solution_validation_error;
use swarm_core::util::io::read_spp_input;
use swarm_core::util::terminator::{BasicTerminator, Terminator};
use std::path::Path;
use std::time::Duration;

fn load_fu_instance() -> SPInstance {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input/fu.json");
    let (external, _) = read_spp_input(&path).expect("failed to read fu.json");
    let cfg = DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(cfg.cde_config, cfg.poly_simpl_tolerance, cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&importer, &external)
        .expect("failed to import fu.json")
}

/// 탐색 단계를 짧은 예산으로 한 번 돌린다. 반환값은 (초기 폭, feasible 해 목록).
fn run_exploration(seed: u64, config: &ExplorationConfig, budget: Duration) -> (f32, Vec<SPSolution>) {
    let instance = load_fu_instance();
    let builder = LBFBuilder::new(
        instance.clone(),
        Xoshiro256PlusPlus::seed_from_u64(seed),
        LBF_SAMPLE_CONFIG,
        10.0,
        1.0,
    )
    .construct();

    let mut sep = Separator::new(
        instance.clone(),
        builder.prob,
        Xoshiro256PlusPlus::seed_from_u64(seed + 1),
        config.separator_config,
    );
    let initial_width = sep.prob.strip_width();

    let mut term = BasicTerminator::new();
    term.new_timeout(budget);

    let solutions = exploration_phase_adaptive(&instance, &mut sep, &mut None, &term, config);
    (initial_width, solutions)
}

/// 인컴번트 계약: 반환된 해는 전부 정확히 feasible 하고, 폭은 단조 감소한다.
fn assert_incumbent_contract(instance: &SPInstance, initial_width: f32, solutions: &[SPSolution]) {
    assert!(!solutions.is_empty(), "탐색은 최소한 초기 해를 돌려줘야 한다");

    let mut previous = f32::INFINITY;
    for (i, solution) in solutions.iter().enumerate() {
        assert!(
            solution_validation_error(instance, solution).is_none(),
            "해 {i}이 정확히 feasible 하지 않다: {:?}",
            solution_validation_error(instance, solution)
        );
        let width = solution.strip_width();
        assert!(
            width < previous,
            "폭은 단조 감소해야 한다: 해 {i} = {width}, 직전 = {previous}"
        );
        previous = width;
    }

    assert!(
        solutions.last().unwrap().strip_width() <= initial_width + 1.0e-5,
        "최종 폭이 초기 폭보다 넓어질 수 없다"
    );
}

/// 좁은 폭에서 자주 막히도록 축소 스텝을 크게 잡은 설정.
fn stuck_prone_config() -> ExplorationConfig {
    ExplorationConfig {
        // 큰 스텝 = 한 번에 많이 조인다 = 금방 못 푸는 폭에 도달한다.
        shrink_step: 0.03,
        adaptive_shrink_gain: 0.0,
        // 짧은 분리 예산으로 실패를 빨리 만든다.
        separator_config: swarm_core::optimizer::separator::SeparatorConfig {
            strike_limit: 1,
            iter_no_imprv_limit: 30,
            n_workers: 2,
            ..DEFAULT_SWARM_CONFIG.expl_cfg.separator_config
        },
        rnr: swarm_core::config::RuinRecreateConfig {
            period: 0,
            ..DEFAULT_SWARM_CONFIG.expl_cfg.rnr
        },
        ..DEFAULT_SWARM_CONFIG.expl_cfg
    }
}

#[test]
fn relief_enabled_exploration_keeps_the_incumbent_contract() {
    let instance = load_fu_instance();
    let config = ExplorationConfig {
        relief_fail_threshold: 1,
        relief_step: 0.01,
        relief_max_ratio: 0.04,
        relief_max_rounds: 4,
        relief_strike_limit: 1,
        ..stuck_prone_config()
    };

    let (initial_width, solutions) = run_exploration(11, &config, Duration::from_secs(6));
    assert_incumbent_contract(&instance, initial_width, &solutions);
}

#[test]
fn relief_disabled_exploration_is_unchanged() {
    let instance = load_fu_instance();
    // 기본값(relief off)에서는 기존 폭 진동 경로가 그대로 돈다.
    let config = stuck_prone_config();
    assert_eq!(config.relief_fail_threshold, 0);
    assert!(config.osc_fail_threshold > 0);

    let (initial_width, solutions) = run_exploration(11, &config, Duration::from_secs(6));
    assert_incumbent_contract(&instance, initial_width, &solutions);
}

/// 복구가 실패해도 `max_conseq_failed_attempts` 종료가 살아 있어야 한다.
///
/// 복구는 폭 카운터(`n_fails_at_width`)만 리셋하고 연속 실패 카운터는 건드리지
/// 않는다. 둘을 같이 리셋했다면 이 테스트는 시간 제한까지 돌다가 실패한다.
#[test]
fn unresolvable_relief_still_terminates_on_consecutive_failures() {
    let instance = load_fu_instance();
    let config = ExplorationConfig {
        relief_fail_threshold: 1,
        // 0.01%까지만 넓힌다 = 사실상 절대 못 푼다.
        relief_step: 0.00005,
        relief_max_ratio: 0.0001,
        relief_max_rounds: 2,
        relief_strike_limit: 1,
        max_conseq_failed_attempts: Some(3),
        ..stuck_prone_config()
    };

    let budget = Duration::from_secs(60);
    let started = std::time::Instant::now();
    let (initial_width, solutions) = run_exploration(23, &config, budget);

    assert!(
        started.elapsed() < budget,
        "연속 실패 상한으로 끝나야 한다 (시간 제한까지 돌면 실패): {:?}",
        started.elapsed()
    );
    assert_incumbent_contract(&instance, initial_width, &solutions);

    // 두 번 돌려도 같은 지점에서 멈춘다 (시간이 아니라 실패 횟수로 끝나므로 결정적).
    let (_, again) = run_exploration(23, &config, budget);
    let widths = |sols: &[SPSolution]| sols.iter().map(|s| s.strip_width()).collect::<Vec<_>>();
    assert_eq!(widths(&solutions), widths(&again));
}
