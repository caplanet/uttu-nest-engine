use crate::config::*;
use crate::consts::LBF_SAMPLE_CONFIG;
use crate::optimizer::compress::compression_phase;
#[cfg(not(feature = "adaptive"))]
use crate::optimizer::explore::exploration_phase;
use crate::optimizer::lbf::LBFBuilder;
use crate::optimizer::separator::Separator;
use crate::util::listener::{ReportType, SolutionListener};
use crate::util::terminator::Terminator;
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use log::warn;
use rand::rngs::Xoshiro256PlusPlus;
use rand::{Rng, SeedableRng};

pub mod compress;
/// LBF 배치 순서용 방향성 오목도 분석 (`LBF수정사항.md`).
pub mod concavity;
pub mod explore;
pub mod lbf;
pub mod place_groups;
pub mod polish;
pub mod reclaim;
pub mod separator;
mod worker;

/// Configuration-oriented entry point used by the CLI and non-step FFI.
/// It preserves [`optimize`] as the stable legacy API while allowing an
/// explicit runtime opt-in to the feature-gated structure-first pipeline.
pub fn optimize_configured(
    mut instance: SPInstance,
    rng: Xoshiro256PlusPlus,
    sol_listener: Option<&mut dyn SolutionListener>,
    terminator: &mut impl Terminator,
    config: &SwarmConfig,
    initial_solution: Option<&SPSolution>,
) -> (
    SPSolution,
    Option<crate::structure::pipeline::StructureRunReport>,
) {
    let _run_scope =
        crate::config::runtime::enter_if_unset(crate::config::runtime::current_control());
    crate::original_overlap::apply_policy(&mut instance, config.original_overlap_policy);
    let draft_seed =
        if initial_solution.is_none() && config.refplace.enabled && config.refplace.column_draft {
            match crate::refplace::columns::build_column_draft(
                &instance,
                crate::refplace::columns::ColumnDraftConfig {
                    preserve_input_contour: config.simplify_max_deviation_mm.is_some(),
                    ..Default::default()
                },
            ) {
                Ok((solution, _)) => Some(solution),
                Err(error) => {
                    warn!("[INIT] Column Draft failed: {error}; using configured fallback");
                    None
                }
            }
        } else {
            None
        };
    let initial_solution = initial_solution.or(draft_seed.as_ref());
    #[cfg(feature = "structure-first")]
    if config.structure.enabled {
        let (solution, report) = crate::structure::pipeline::optimize_structure_first(
            instance,
            rng,
            sol_listener,
            terminator,
            &config.expl_cfg,
            &config.cmpr_cfg,
            &config.polish_cfg,
            &config.nfp_integration,
            &config.lp_solver,
            initial_solution,
            &[],
            config.lbf_x_multiplier,
            config.lbf_y_multiplier,
            &config.lbf_raster,
            config.lbf_order,
            &config.structure,
        );
        return (solution, Some(report));
    }

    #[cfg(not(feature = "structure-first"))]
    if config.structure.enabled {
        log::warn!(
            "structure-first requested, but this binary was built without the structure-first feature; using legacy optimizer"
        );
    }

    let nfp_lbf_cfg = config
        .nfp_lbf
        .enabled
        .then(|| config.nfp_lbf.to_config(config.nfp_strategy));
    let escape_cfg = config.escape_enabled.then_some(&config.escape);
    let solution = optimize_with_escape(
        instance,
        rng,
        sol_listener,
        terminator,
        &config.expl_cfg,
        &config.cmpr_cfg,
        &config.polish_cfg,
        initial_solution,
        config.lbf_x_multiplier,
        config.lbf_y_multiplier,
        &config.lbf_raster,
        config.lbf_order,
        escape_cfg,
        nfp_lbf_cfg.as_ref(),
        &config.nfp_integration,
        &config.lp_solver,
    );
    (solution, None)
}

// CA Planet custom algorithm variants.
//
// `adaptive` : 탐색 단계를 적응형 변형으로 교체한다 (explore_mods).
// `capmods`  : 그 밖의 커스텀 변형들 (래스터 NFP, 좌측 편향 샘플링 등).
// `lns` only retains inert compatibility entrypoints for older callers.
#[cfg(feature = "adaptive")]
pub mod explore_mods;
pub mod lns;

///Algorithm 11 from https://doi.org/10.48550/arXiv.2509.13329
///
/// 기존 공개 API. 탈출 기법과 NFP-LBF 시드는 꺼진 상태로 동작한다.
#[allow(clippy::too_many_arguments)]
pub fn optimize(
    instance: SPInstance,
    rng: Xoshiro256PlusPlus,
    sol_listener: Option<&mut dyn SolutionListener>,
    terminator: &mut impl Terminator,
    expl_config: &ExplorationConfig,
    cmpr_config: &CompressionConfig,
    polish_config: &PolishConfig,
    initial_solution: Option<&SPSolution>,
    lbf_x_multiplier: f32,
    lbf_y_multiplier: f32,
    lbf_raster: &RasterNfpConfig,
) -> SPSolution {
    optimize_with_escape(
        instance,
        rng,
        sol_listener,
        terminator,
        expl_config,
        cmpr_config,
        polish_config,
        initial_solution,
        lbf_x_multiplier,
        lbf_y_multiplier,
        lbf_raster,
        crate::optimizer::lbf::LbfOrderMode::LargestFirst,
        None,
        None,
        &crate::config::DEFAULT_NFP_INTEGRATION_CONFIG,
        &crate::lp_solver::DEFAULT_LP_SOLVER_CONFIG,
    )
}

/// [`optimize`]에 두 가지 선택적 기능을 얹은 진입점.
///
/// * `escape_config`가 `Some`이면 탐색 단계를
///   [`crate::escape::escape_driven_exploration_phase`]로 교체한다
///   (정체 감지 기반 적응형 국소 최적해 탈출 기법).
/// * `nfp_lbf_config`가 `Some`이고 `initial_solution`이 없으면
///   NFP Cache -> Multiple Item Ordering -> NFP Sequential Placement 로 만든
///   Best Initial Nest를 seed로 쓴다. 실패하면 기존 LBF 생성기로 폴백한다.
#[allow(clippy::too_many_arguments)]
pub fn optimize_with_escape(
    instance: SPInstance,
    mut rng: Xoshiro256PlusPlus,
    mut sol_listener: Option<&mut dyn SolutionListener>,
    terminator: &mut impl Terminator,
    expl_config: &ExplorationConfig,
    cmpr_config: &CompressionConfig,
    polish_config: &PolishConfig,
    initial_solution: Option<&SPSolution>,
    lbf_x_multiplier: f32,
    lbf_y_multiplier: f32,
    lbf_raster: &RasterNfpConfig,
    lbf_order: crate::optimizer::lbf::LbfOrderMode,
    escape_config: Option<&crate::escape::AdaptiveEscapeConfig>,
    nfp_lbf_config: Option<&crate::nfp_lbf::NfpLbfConfig>,
    nfp_integration: &crate::config::NfpIntegrationConfig,
    lp_solver: &crate::lp_solver::LpSolverConfig,
) -> SPSolution {
    let _run_scope =
        crate::config::runtime::enter_if_unset(crate::config::runtime::current_control());
    let mut next_rng = || Xoshiro256PlusPlus::seed_from_u64(rng.next_u64());

    // NFP-LBF 시드. 검증을 통과한 해만 warm start로 넘긴다.
    let nfp_seed = match (initial_solution, nfp_lbf_config) {
        (None, Some(cfg)) => {
            let mut seed_rng = next_rng();
            match crate::nfp_lbf::build_best_initial_nest(&instance, cfg, &mut seed_rng) {
                Some(nest) => Some(nest.solution),
                None => {
                    warn!("[NFP-LBF] no usable initial nest; falling back to the LBF constructor");
                    None
                }
            }
        }
        _ => None,
    };
    let initial_solution = initial_solution.or(nfp_seed.as_ref());

    // First build an initial solution if none is provided
    let mut start_prob = match initial_solution {
        None => {
            let builder = LBFBuilder::new(
                instance.clone(),
                next_rng(),
                LBF_SAMPLE_CONFIG,
                lbf_x_multiplier,
                lbf_y_multiplier,
            )
            .with_order_mode(lbf_order)
            .with_raster_config(*lbf_raster)
            .construct();
            builder.prob
        }
        Some(init_sol) => {
            if crate::structure::pipeline::solution_validation_error(&instance, init_sol).is_none()
            {
                let mut prob = jagua_rs::probs::spp::entities::SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
                prob.restore(init_sol);
                prob
            } else {
                warn!("[OPT] rejected invalid warm start; rebuilding an exact LBF baseline");
                LBFBuilder::new(
                    instance.clone(),
                    next_rng(),
                    LBF_SAMPLE_CONFIG,
                    lbf_x_multiplier,
                    lbf_y_multiplier,
                )
                .with_order_mode(lbf_order)
                .with_raster_config(*lbf_raster)
                .construct()
                .prob
            }
        }
    };
    terminator.new_timeout(std::time::Duration::from_secs(10));
    let _ = crate::original_overlap::repair_problem(
        &mut start_prob,
        &[],
        &place_groups::PlaceGroups::default(),
        terminator,
    );
    let mut exact_incumbent = start_prob.save();

    // Begin by executing the exploration phase
    terminator.new_timeout(expl_config.time_limit);
    let mut expl_separator = Separator::new(
        instance.clone(),
        start_prob,
        next_rng(),
        expl_config.separator_config,
    );
    expl_separator.configure_reinsertion(nfp_integration);

    // 탐색 단계 선택.
    //
    //   escape_config = Some     -> escape_driven_exploration_phase (정체 탈출 기법)
    //   feature "adaptive"       -> exploration_phase_adaptive
    //   그 외                    -> exploration_phase (upstream 원본)
    //
    // 탈출 기법은 런타임 스위치라 feature 조합과 무관하게 우선한다.
    // 순수 NFP 순서 탐색이 켜지면 그것이 최우선이다 — 겹침을 한 번도 허용하지 않는
    // 완전히 다른 탐색 계열이라 다른 옵션과 섞이지 않는다.
    let solutions = if nfp_integration.ordering_exploration {
        let seed_cfg = nfp_lbf_config
            .cloned()
            .unwrap_or_else(crate::nfp_lbf::NfpLbfConfig::default);
        let seed_c = crate::config::NfpLbfSeedConfig {
            enabled: true,
            n_orderings: seed_cfg.n_orderings,
            cache_budget_ms: seed_cfg.cache_budget.as_millis() as u64,
            total_budget_ms: seed_cfg.total_budget.as_millis() as u64,
            allow_180: seed_cfg.rotations.contains(&180),
            enable_prefix_reuse: seed_cfg.enable_prefix_reuse,
            score_weights: seed_cfg.weights,
            concave_simplify_tolerance: seed_cfg.concave_simplify_tolerance,
            max_candidate_probes: seed_cfg.max_candidate_probes,
        };
        crate::nfp_lbf::nfp_ordering_exploration_phase(
            &instance,
            &mut expl_separator,
            &mut sol_listener,
            terminator,
            expl_config,
            &seed_c,
        )
    } else {
        match escape_config {
            Some(escape_config) => {
                crate::escape::pipeline::escape_driven_exploration_phase_with_nfp(
                    &instance,
                    &mut expl_separator,
                    &mut sol_listener,
                    terminator,
                    expl_config,
                    escape_config,
                    nfp_integration,
                )
            }
            None => {
                // 일반 탐색 기능
                #[cfg(not(feature = "adaptive"))]
                let solutions = exploration_phase(
                    &instance,
                    &mut expl_separator,
                    &mut sol_listener,
                    terminator,
                    expl_config,
                );

                // 적응형 탐색 Adaptive 기능
                #[cfg(feature = "adaptive")]
                let solutions = crate::optimizer::explore_mods::exploration_phase_adaptive(
                    &instance,
                    &mut expl_separator,
                    &mut sol_listener,
                    terminator,
                    expl_config,
                );

                solutions
            }
        }
    };
    let expl_candidate = solutions
        .last()
        .cloned()
        .unwrap_or_else(|| exact_incumbent.clone());
    let final_explore_sol =
        retain_exact_no_worse(&instance, &exact_incumbent, expl_candidate, "EXPL");
    exact_incumbent = final_explore_sol.clone();
    expl_separator.rollback(&final_explore_sol, None);

    // Start the compression phase from the final solution from the exploration phase
    terminator.new_timeout(cmpr_config.time_limit);
    let mut cmpr_separator = Separator::new(
        expl_separator.instance,
        expl_separator.prob,
        next_rng(),
        cmpr_config.separator_config,
    );
    cmpr_separator.configure_reinsertion(nfp_integration);
    let cmpr_candidate = compression_phase(
        &instance,
        &mut cmpr_separator,
        &final_explore_sol,
        &mut sol_listener,
        terminator,
        cmpr_config,
    );
    let cmpr_sol = retain_exact_no_worse(&instance, &exact_incumbent, cmpr_candidate, "CMPR");
    exact_incumbent = cmpr_sol.clone();
    cmpr_separator.rollback(&cmpr_sol, None);

    // 압축이 끝난 feasible 해를 결정론적으로 다듬는다 (슬라이딩 / hole filling / LP compaction)
    terminator.new_timeout(polish_config.time_limit);
    // 폴리시 마감을 붙잡아 둔다. 폴리시가 일찍 수렴하면 그 남은 시간이 회수 루프의
    // 예산이 된다 — 호출자가 준 적 없는 시간은 쓰지 않는다.
    let polish_deadline = terminator.timeout_at();
    let polish_candidate = if crate::optimizer::polish::has_active_passes(
        polish_config,
        cmpr_separator.has_place_groups(),
    ) {
        crate::optimizer::polish::polish_phase_with_nfp(
            &instance,
            &mut cmpr_separator,
            &cmpr_sol,
            &mut sol_listener,
            terminator,
            polish_config,
            nfp_integration,
            lp_solver,
        )
    } else {
        // Compression already committed an exact input; no local pass can
        // change it. Reclaim below retains the independently assigned budget.
        cmpr_sol.clone()
    };
    // 폴리시 결과를 **회수 앞에서** 인컴번트로 확정한다. 회수 뒤에 한 번만 걸면 회수의 늦은
    // 실패 하나가 압축 해로 물러나게 만들어 폴리시가 얻은 것까지 버린다.
    let polish_sol = retain_exact_no_worse(&instance, &exact_incumbent, polish_candidate, "PLSH");
    let final_sol = match polish_deadline {
        Some(deadline) if polish_config.reclaim.enabled => {
            let reclaimed = crate::optimizer::reclaim::reclaim_leftover(
                &instance,
                &mut cmpr_separator,
                polish_sol.clone(),
                deadline,
                &polish_config.reclaim,
                cmpr_config,
                expl_config,
                polish_config,
                nfp_integration,
                escape_config.unwrap_or(&crate::config::DEFAULT_ADAPTIVE_ESCAPE_CONFIG),
                lp_solver,
                terminator,
            )
            .0;
            retain_exact_no_worse(&instance, &polish_sol, reclaimed, "RECLAIM")
        }
        _ => polish_sol,
    };

    if let Some(l) = sol_listener {
        l.report(ReportType::Final, &final_sol, &instance);
    }

    // Return the final polished solution
    final_sol
}

/// Direct/native optimization keeps one transform-authoritative exact incumbent
/// across phase boundaries. Approximate evaluators may propose candidates, but
/// an invalid or wider candidate can never replace the public best-so-far.
fn retain_exact_no_worse(
    instance: &SPInstance,
    incumbent: &SPSolution,
    candidate: SPSolution,
    phase: &str,
) -> SPSolution {
    if crate::structure::pipeline::solution_validation_error(instance, &candidate).is_some() {
        warn!("[{phase}] rejected non-exact phase result; retaining incumbent");
        return incumbent.clone();
    }
    if candidate.strip_width() > incumbent.strip_width() + 1.0e-5 {
        warn!(
            "[{phase}] rejected wider phase result ({:.3} > {:.3}); retaining incumbent",
            candidate.strip_width(),
            incumbent.strip_width()
        );
        return incumbent.clone();
    }
    candidate
}
