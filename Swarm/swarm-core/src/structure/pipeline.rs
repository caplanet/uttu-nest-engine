//! P0-P5 structure-first orchestration.
//!
//! The legacy optimiser remains untouched and is still the default. This
//! module composes existing exact Swarm phases in a different order while
//! preserving an exact-feasible global incumbent after every stage.

use crate::config::{
    CompressionConfig, ExplorationConfig, PolishConfig, RasterNfpConfig, RuinRecreateConfig,
    ShrinkDecayStrategy,
};
use crate::consts::LBF_SAMPLE_CONFIG;
use crate::optimizer::compress::compression_phase;
#[cfg(not(feature = "adaptive"))]
use crate::optimizer::explore::exploration_phase;
use crate::optimizer::lbf::{LBFBuilder, StructureHintStats};
use crate::optimizer::lns::{BandTabu, RuinStrategy, ruin_and_recreate_profile_cancellable};
use crate::optimizer::polish::{polish_phase_with_nfp, slide};
use crate::optimizer::separator::Separator;
use crate::structure::config::StructureFirstConfig;
use crate::structure::evaluator::ProfilePlacementConfig;
use crate::structure::motif::{MotifGenerationStats, MotifGenerator, MotifSelectionStats};
use crate::util::listener::{ReportType, SolutionListener};
use crate::util::terminator::Terminator;
use jagua_rs::Instant;
use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::geo_enums::RotationRange;
use jagua_rs::geometry::geo_traits::Transformable;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution};
use log::warn;
use rand::rngs::Xoshiro256PlusPlus;
use rand::{RngExt, SeedableRng};
use serde::Serialize;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum StructureStage {
    P0Preprocess,
    P1Construct,
    P2Compress,
    P3Explore,
    P4Refine,
    P5Finish,
}

#[derive(Clone, Debug, Serialize)]
pub struct StageCheckpoint {
    pub stage: StructureStage,
    pub elapsed_secs: f32,
    pub width: f32,
    pub density: f32,
    pub is_global_incumbent: bool,
    pub incumbent_width: f32,
    pub incumbent_density: f32,
}

/// P4 state/improvement tuple retained as future operator-learning data.
#[derive(Clone, Debug, Serialize)]
pub struct RepairCycleRecord {
    pub cycle: usize,
    pub elapsed_secs: f32,
    pub width_before: f32,
    pub width_after: f32,
    pub density_before: f32,
    pub density_after: f32,
    pub feasible: bool,
    pub improved_global_incumbent: bool,
}

/// Origin of a P1 candidate retained in the audit trail.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum P1CandidateSource {
    WarmStart,
    LegacyBaseline,
    Experimental,
    EmergencyLegacyFallback,
    GroupedLaneBaseline,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum P1ValidationError {
    Cancelled,
    UnknownItemId {
        item_id: usize,
    },
    QuantityMismatch {
        item_id: usize,
        expected: usize,
        actual: usize,
    },
    RotationNotAllowed {
        item_id: usize,
    },
    LayoutInfeasible {
        flat_exact_feasible: bool,
        exterior_collisions: usize,
        placed_item_collisions: usize,
        other_collisions: usize,
        first_query_item_id: Option<usize>,
        first_hazard_item_id: Option<usize>,
    },
}

/// One independently evaluable P1 candidate. Partial/cancelled experimental
/// starts remain visible, but width and density are published only for complete
/// exact-valid solutions.
#[derive(Clone, Debug, Serialize)]
pub struct P1CandidateRecord {
    pub source: P1CandidateSource,
    pub start_index: Option<usize>,
    pub elapsed_secs: f32,
    pub complete: bool,
    pub exact_valid: bool,
    pub validation_error: Option<P1ValidationError>,
    pub stopped: bool,
    pub width: Option<f32>,
    pub density: Option<f32>,
    pub improved_incumbent: bool,
    pub selected_as_p1_incumbent: bool,
    pub structure_hints: Option<StructureHintStats>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
pub struct P1WinnerRecord {
    pub source: P1CandidateSource,
    pub start_index: Option<usize>,
}

#[derive(Clone, Debug, Serialize)]
pub struct StructureRunReport {
    pub schema_version: u32,
    pub cancelled: bool,
    pub p0_elapsed_secs: f32,
    pub p0_stopped: bool,
    pub motif_generation_stats: MotifGenerationStats,
    pub motif_singles: usize,
    pub motif_pairs: usize,
    pub motif_blocks: usize,
    pub motif_bands: usize,
    pub motif_fft_pairs: usize,
    pub pair_matching_method: String,
    pub pair_matching_vertices: usize,
    pub pair_matching_edges: usize,
    pub pair_matching_dp_states: usize,
    pub pair_matching_selected_pairs: usize,
    pub motif_selection_stats: MotifSelectionStats,
    pub construction_starts_requested: usize,
    pub construction_starts_attempted: usize,
    pub construction_starts_completed: usize,
    pub p1_selected_legacy_baseline: bool,
    pub p1_control_candidate: Option<P1CandidateRecord>,
    pub p1_start_stats: Vec<P1CandidateRecord>,
    pub p1_winner: Option<P1WinnerRecord>,
    pub p1_deadline_exhausted: bool,
    pub checkpoints: Vec<StageCheckpoint>,
    pub repair_cycles: Vec<RepairCycleRecord>,
    pub best_stage: StructureStage,
    pub final_width: f32,
    pub final_density: f32,
    pub elapsed_secs: f32,
}

/// Explicit failure returned by the checked Structure-first entry points.
///
/// The audit report is retained so rejected P1 candidates remain inspectable
/// even when no complete exact-feasible incumbent can be constructed.
#[derive(Clone, Debug)]
pub struct StructurePipelineFailure {
    pub stage: StructureStage,
    pub message: String,
    pub report: StructureRunReport,
}

impl std::fmt::Display for StructurePipelineFailure {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{:?}: {}", self.stage, self.message)
    }
}

impl std::error::Error for StructurePipelineFailure {}

impl StructureRunReport {
    fn new() -> Self {
        Self {
            schema_version: 3,
            cancelled: false,
            p0_elapsed_secs: 0.0,
            p0_stopped: false,
            motif_generation_stats: MotifGenerationStats::default(),
            motif_singles: 0,
            motif_pairs: 0,
            motif_blocks: 0,
            motif_bands: 0,
            motif_fft_pairs: 0,
            pair_matching_method: "none".to_string(),
            pair_matching_vertices: 0,
            pair_matching_edges: 0,
            pair_matching_dp_states: 0,
            pair_matching_selected_pairs: 0,
            motif_selection_stats: MotifSelectionStats::default(),
            construction_starts_requested: 0,
            construction_starts_attempted: 0,
            construction_starts_completed: 0,
            p1_selected_legacy_baseline: false,
            p1_control_candidate: None,
            p1_start_stats: vec![],
            p1_winner: None,
            p1_deadline_exhausted: false,
            checkpoints: vec![],
            repair_cycles: vec![],
            best_stage: StructureStage::P1Construct,
            final_width: f32::INFINITY,
            final_density: 0.0,
            elapsed_secs: 0.0,
        }
    }

    pub fn cancelled_baseline(_instance: &SPInstance, solution: &SPSolution) -> Self {
        let mut report = Self::new();
        report.cancelled = true;
        report.final_width = solution.strip_width();
        report.final_density = solution.density();
        report
    }
}

/// Runs the experimental structure-first pipeline and returns both the exact
/// feasible result and its diagnostic/checkpoint audit trail.
#[allow(clippy::too_many_arguments)]
pub fn optimize_structure_first(
    instance: SPInstance,
    rng: Xoshiro256PlusPlus,
    sol_listener: Option<&mut dyn SolutionListener>,
    terminator: &mut impl Terminator,
    expl_config: &ExplorationConfig,
    cmpr_config: &CompressionConfig,
    polish_config: &PolishConfig,
    nfp_integration: &crate::config::NfpIntegrationConfig,
    lp_solver: &crate::lp_solver::LpSolverConfig,
    initial_solution: Option<&SPSolution>,
    frozen_item_ids: &[usize],
    lbf_x_multiplier: f32,
    lbf_y_multiplier: f32,
    lbf_raster: &RasterNfpConfig,
    lbf_order: crate::optimizer::lbf::LbfOrderMode,
    structure_config: &StructureFirstConfig,
) -> (SPSolution, StructureRunReport) {
    try_optimize_structure_first(
        instance,
        rng,
        sol_listener,
        terminator,
        expl_config,
        cmpr_config,
        polish_config,
        nfp_integration,
        lp_solver,
        initial_solution,
        frozen_item_ids,
        lbf_x_multiplier,
        lbf_y_multiplier,
        lbf_raster,
        lbf_order,
        structure_config,
    )
    .unwrap_or_else(|failure| panic!("Structure-first failed: {failure}"))
}

/// Checked counterpart of [`optimize_structure_first`]. New integration
/// boundaries should use this entry point so a P1 construction failure is
/// reported explicitly instead of being confused with an exact solution.
#[allow(clippy::too_many_arguments)]
pub fn try_optimize_structure_first(
    instance: SPInstance,
    rng: Xoshiro256PlusPlus,
    sol_listener: Option<&mut dyn SolutionListener>,
    terminator: &mut impl Terminator,
    expl_config: &ExplorationConfig,
    cmpr_config: &CompressionConfig,
    polish_config: &PolishConfig,
    nfp_integration: &crate::config::NfpIntegrationConfig,
    lp_solver: &crate::lp_solver::LpSolverConfig,
    initial_solution: Option<&SPSolution>,
    frozen_item_ids: &[usize],
    lbf_x_multiplier: f32,
    lbf_y_multiplier: f32,
    lbf_raster: &RasterNfpConfig,
    lbf_order: crate::optimizer::lbf::LbfOrderMode,
    structure_config: &StructureFirstConfig,
) -> Result<(SPSolution, StructureRunReport), StructurePipelineFailure> {
    try_optimize_structure_first_with_milestones(
        instance,
        rng,
        sol_listener,
        terminator,
        expl_config,
        cmpr_config,
        polish_config,
        nfp_integration,
        lp_solver,
        initial_solution,
        frozen_item_ids,
        lbf_x_multiplier,
        lbf_y_multiplier,
        lbf_raster,
        lbf_order,
        structure_config,
        None,
    )
}

/// Runs structure-first while exposing exact stage-boundary solutions separately from the
/// throttled optimizer listener.
///
/// The ordinary [`SolutionListener`] still receives candidate and in-stage progress.  When a
/// milestone listener is supplied, P1-P5 boundaries are sent only through that listener so an
/// FFI adapter can force a complete JSON checkpoint without also producing a scalar-only
/// duplicate. Callers that do not need this distinction should use [`optimize_structure_first`].
#[allow(clippy::too_many_arguments)]
pub fn optimize_structure_first_with_milestones(
    instance: SPInstance,
    rng: Xoshiro256PlusPlus,
    sol_listener: Option<&mut dyn SolutionListener>,
    terminator: &mut impl Terminator,
    expl_config: &ExplorationConfig,
    cmpr_config: &CompressionConfig,
    polish_config: &PolishConfig,
    nfp_integration: &crate::config::NfpIntegrationConfig,
    lp_solver: &crate::lp_solver::LpSolverConfig,
    initial_solution: Option<&SPSolution>,
    frozen_item_ids: &[usize],
    lbf_x_multiplier: f32,
    lbf_y_multiplier: f32,
    lbf_raster: &RasterNfpConfig,
    lbf_order: crate::optimizer::lbf::LbfOrderMode,
    structure_config: &StructureFirstConfig,
    milestone_listener: Option<
        &mut dyn FnMut(StructureStage, ReportType, &SPSolution, &SPInstance),
    >,
) -> (SPSolution, StructureRunReport) {
    try_optimize_structure_first_with_milestones(
        instance,
        rng,
        sol_listener,
        terminator,
        expl_config,
        cmpr_config,
        polish_config,
        nfp_integration,
        lp_solver,
        initial_solution,
        frozen_item_ids,
        lbf_x_multiplier,
        lbf_y_multiplier,
        lbf_raster,
        lbf_order,
        structure_config,
        milestone_listener,
    )
    .unwrap_or_else(|failure| panic!("Structure-first failed: {failure}"))
}

/// Checked milestone-aware Structure-first entry point.
#[allow(clippy::too_many_arguments)]
pub fn try_optimize_structure_first_with_milestones(
    instance: SPInstance,
    mut rng: Xoshiro256PlusPlus,
    mut sol_listener: Option<&mut dyn SolutionListener>,
    terminator: &mut impl Terminator,
    expl_config: &ExplorationConfig,
    cmpr_config: &CompressionConfig,
    polish_config: &PolishConfig,
    nfp_integration: &crate::config::NfpIntegrationConfig,
    lp_solver: &crate::lp_solver::LpSolverConfig,
    initial_solution: Option<&SPSolution>,
    frozen_item_ids: &[usize],
    lbf_x_multiplier: f32,
    lbf_y_multiplier: f32,
    lbf_raster: &RasterNfpConfig,
    lbf_order: crate::optimizer::lbf::LbfOrderMode,
    structure_config: &StructureFirstConfig,
    mut milestone_listener: Option<
        &mut dyn FnMut(StructureStage, ReportType, &SPSolution, &SPInstance),
    >,
) -> Result<(SPSolution, StructureRunReport), StructurePipelineFailure> {
    let config = structure_config.clone().sanitized();
    let frozen_mask = crate::util::frozen::mask(instance.items.len(), frozen_item_ids);
    let solution_is_valid = |instance: &SPInstance, solution: &SPSolution| {
        solution_validation_error_with_frozen(instance, solution, &frozen_mask).is_none()
    };
    let consider_incumbent = |instance: &SPInstance,
                              incumbent: &mut SPSolution,
                              candidate: &SPSolution| {
        if !solution_is_valid(instance, candidate)
            || !crate::util::assertions::frozen_poses_preserved(incumbent, candidate, &frozen_mask)
        {
            return false;
        }
        if !solution_is_valid(instance, incumbent)
            || candidate.strip_width() + 1.0e-5 < incumbent.strip_width()
        {
            *incumbent = candidate.clone();
            true
        } else {
            false
        }
    };
    let started = Instant::now();
    let mut report = StructureRunReport::new();
    let profile_cfg = profile_config(&instance, &config);
    // Validate a supplied incumbent before deciding whether P0 has any work.
    // Validation consumes no RNG, and remains inside the shared P0/P1
    // deadline so invalid warm starts retain the existing budget contract.
    terminator.new_timeout(config.p1_time_limit);
    let validated_initial = initial_solution.filter(|solution| {
        let valid = solution_is_valid(&instance, solution);
        if !valid {
            warn!(
                "[STRUCT:P1] supplied warm start is incomplete, colliding, or off-angle; rebuilding"
            );
        }
        valid
    });
    // Frozen poses are only known through the supplied warm start. Rebuilding
    // from scratch would silently drop them, so fail explicitly instead.
    if initial_solution.is_some() && validated_initial.is_none() && !frozen_item_ids.is_empty() {
        report.elapsed_secs = started.elapsed().as_secs_f32();
        return Err(StructurePipelineFailure {
            stage: StructureStage::P1Construct,
            message: "frozen items require an exact-valid warm start; the supplied one was rejected"
                .to_string(),
            report,
        });
    }

    // P0 and all P1 starts share one absolute budget. Motif generation checks
    // the same terminator at its bounded inner-loop checkpoints, and is
    // additionally capped at a quarter of that budget so a slow catalog can
    // never starve every P1 start.
    let selected_motifs = if validated_initial.is_some() {
        // StructureRunReport::new() intentionally represents a skipped P0:
        // zero elapsed/counts, no cancellation and matching method "none".
        vec![]
    } else {
        let p0_started = Instant::now();
        let p0_deadline = p0_started + config.p1_time_limit / 4;
        let motif_generation = {
            let mut should_stop = || terminator.kill() || Instant::now() > p0_deadline;
            MotifGenerator::new(motif_config_for(&instance, &config))
                .generate_with_control_and_stats(&instance, &mut should_stop)
        };
        let motif_catalog = motif_generation.catalog;
        report.motif_generation_stats = motif_generation.stats;
        report.motif_singles = motif_catalog.singles.len();
        report.motif_pairs = motif_catalog.pairs.len();
        report.motif_blocks = motif_catalog.blocks.len();
        report.motif_bands = motif_catalog.bands.len();
        report.motif_fft_pairs = motif_catalog.pairs.len();
        let (selected_motifs, selection_stats) = if terminator.kill() {
            (vec![], MotifSelectionStats::default())
        } else {
            let motif_selection =
                motif_catalog.disjoint_ranked_with_stats(config.max_active_motifs);
            (
                motif_selection
                    .motifs
                    .into_iter()
                    .cloned()
                    .collect::<Vec<_>>(),
                motif_selection.stats,
            )
        };
        report.pair_matching_method = selection_stats.pair_matching_method.as_str().to_string();
        report.pair_matching_vertices = selection_stats.pair_vertices;
        report.pair_matching_edges = selection_stats.pair_edges_considered;
        report.pair_matching_dp_states = selection_stats.pair_dp_states;
        report.pair_matching_selected_pairs = selection_stats.selected_pairs;
        report.motif_selection_stats = selection_stats;
        report.p0_elapsed_secs = p0_started.elapsed().as_secs_f32();
        report.p0_stopped = report.motif_generation_stats.generation_cancelled;
        selected_motifs
    };
    report.construction_starts_requested = config.construction_starts;

    let mut next_rng = || Xoshiro256PlusPlus::seed_from_u64(rng.random());
    // Reserve the same first four child streams used by the legacy pipeline:
    // LBF, Explore, Compress, Polish. Experimental P1 starts and P4 cycles use
    // later streams, so changing their count cannot silently perturb every
    // downstream phase in a paired A/B run.
    let legacy_rng = next_rng();
    let p3_explore_rng = next_rng();
    let p2_compress_rng = next_rng();
    let p5_polish_rng = next_rng();

    // P1: supplied hints remain authoritative. Fresh runs use independent
    // stochastic top-k constructors and retain the narrowest exact solution.
    let p1_solution = match validated_initial {
        Some(solution) => {
            let winner = P1WinnerRecord {
                source: P1CandidateSource::WarmStart,
                start_index: None,
            };
            report.p1_control_candidate = Some(P1CandidateRecord {
                source: winner.source,
                start_index: None,
                elapsed_secs: 0.0,
                complete: true,
                exact_valid: true,
                validation_error: None,
                stopped: false,
                width: Some(solution.strip_width()),
                density: Some(solution.density()),
                improved_incumbent: true,
                selected_as_p1_incumbent: true,
                structure_hints: None,
            });
            report.p1_winner = Some(winner);
            solution.clone()
        }
        None => {
            let mut best: Option<SPSolution> = None;
            let mut winner: Option<P1WinnerRecord> = None;
            // Fork the control constructor before experimental starts consume
            // RNG state. This is the same first child seed used by `optimize`,
            // which makes paired legacy/structure comparisons reproducible.
            if config.p1_include_legacy_baseline && !terminator.kill() {
                let control_started = Instant::now();
                let (legacy, constructor_complete, stopped) = {
                    let mut stopped = false;
                    let mut builder = {
                        let mut should_stop = || {
                            let requested = terminator.kill();
                            stopped |= requested;
                            requested
                        };
                        LBFBuilder::new(
                            instance.clone(),
                            legacy_rng.clone(),
                            LBF_SAMPLE_CONFIG,
                            lbf_x_multiplier,
                            lbf_y_multiplier,
                        )
                        .with_order_mode(lbf_order)
                        .with_raster_config(*lbf_raster)
                        .construct_with_control(&mut should_stop)
                    };
                    let constructor_complete =
                        builder.prob.item_demand_qtys().iter().all(|qty| *qty == 0);
                    if constructor_complete && !stopped {
                        repair_p1_original_overlap(
                            &mut builder.prob,
                            frozen_item_ids,
                            &*terminator,
                        );
                    }
                    (builder.prob.save(), constructor_complete, stopped)
                };
                let control_source = P1CandidateSource::LegacyBaseline;
                let (mut control, selectable) = evaluate_p1_candidate(
                    &instance,
                    &legacy,
                    control_source,
                    None,
                    control_started.elapsed().as_secs_f32(),
                    constructor_complete,
                    stopped,
                    None,
                );
                control.improved_incumbent = selectable;
                if selectable {
                    if let Some(listener) = sol_listener.as_deref_mut() {
                        listener.report(ReportType::ExplFeas, &legacy, &instance);
                    }
                    winner = Some(P1WinnerRecord {
                        source: control_source,
                        start_index: None,
                    });
                    best = Some(legacy);
                } else {
                    warn!(
                        "[STRUCT:P1] rejected invalid legacy control: {:?}",
                        control.validation_error
                    );
                }
                report.p1_control_candidate = Some(control);
            }
            for start in 0..config.construction_starts {
                if terminator.kill() {
                    break;
                }
                report.construction_starts_attempted += 1;
                let candidate_started = Instant::now();
                let mut builder = {
                    let mut should_stop = || terminator.kill();
                    LBFBuilder::new(
                        instance.clone(),
                        next_rng(),
                        LBF_SAMPLE_CONFIG,
                        lbf_x_multiplier,
                        lbf_y_multiplier,
                    )
                    .with_order_mode(lbf_order)
                    .with_raster_config(*lbf_raster)
                    .with_preselected_structure_hints(profile_cfg, selected_motifs.clone())
                    .construct_with_control(&mut should_stop)
                };
                let structure_hints = builder.structure_hint_stats().cloned();
                let constructor_complete = structure_hints
                    .as_ref()
                    .is_none_or(|stats| stats.start.completed);
                let stopped = structure_hints
                    .as_ref()
                    .is_some_and(|stats| stats.start.stopped);
                if constructor_complete
                    && !stopped
                    && builder.prob.item_demand_qtys().iter().all(|qty| *qty == 0)
                {
                    repair_p1_original_overlap(&mut builder.prob, frozen_item_ids, &*terminator);
                }
                let candidate = builder.prob.save();
                let (mut candidate_record, selectable) = evaluate_p1_candidate(
                    &instance,
                    &candidate,
                    P1CandidateSource::Experimental,
                    Some(start),
                    candidate_started.elapsed().as_secs_f32(),
                    constructor_complete,
                    stopped,
                    structure_hints,
                );
                let improved = selectable
                    && best
                        .as_ref()
                        .is_none_or(|current| candidate.strip_width() < current.strip_width());
                candidate_record.improved_incumbent = improved;
                let should_stop_after_rejection = candidate_record.stopped || terminator.kill();
                let rejection = candidate_record.validation_error;
                report.p1_start_stats.push(candidate_record);
                if !selectable {
                    warn!("[STRUCT:P1] rejected start {start}: {rejection:?}");
                    if should_stop_after_rejection {
                        break;
                    }
                    continue;
                }
                report.construction_starts_completed += 1;
                if improved {
                    if let Some(listener) = sol_listener.as_deref_mut() {
                        listener.report(ReportType::ExplFeas, &candidate, &instance);
                    }
                    best = Some(candidate);
                    winner = Some(P1WinnerRecord {
                        source: P1CandidateSource::Experimental,
                        start_index: Some(start),
                    });
                }
            }
            // If no legacy control was attempted, retain the historical
            // emergency constructor. When a legacy control was attempted and
            // rejected, repeating the same cloned RNG stream would only
            // duplicate the same invalid candidate and overwrite its audit.
            if best.is_none() && report.p1_control_candidate.is_none() && !terminator.kill() {
                let control_started = Instant::now();
                let (fallback, constructor_complete, stopped) = {
                    let mut stopped = false;
                    let mut builder = {
                        let mut should_stop = || {
                            let requested = terminator.kill();
                            stopped |= requested;
                            requested
                        };
                        LBFBuilder::new(
                            instance.clone(),
                            legacy_rng,
                            LBF_SAMPLE_CONFIG,
                            lbf_x_multiplier,
                            lbf_y_multiplier,
                        )
                        .with_order_mode(lbf_order)
                        .with_raster_config(*lbf_raster)
                        .construct_with_control(&mut should_stop)
                    };
                    let constructor_complete =
                        builder.prob.item_demand_qtys().iter().all(|qty| *qty == 0);
                    if constructor_complete && !stopped {
                        repair_p1_original_overlap(
                            &mut builder.prob,
                            frozen_item_ids,
                            &*terminator,
                        );
                    }
                    (builder.prob.save(), constructor_complete, stopped)
                };
                let control_source = P1CandidateSource::EmergencyLegacyFallback;
                let (mut control, selectable) = evaluate_p1_candidate(
                    &instance,
                    &fallback,
                    control_source,
                    None,
                    control_started.elapsed().as_secs_f32(),
                    constructor_complete,
                    stopped,
                    None,
                );
                control.improved_incumbent = selectable;
                if selectable {
                    if let Some(listener) = sol_listener.as_deref_mut() {
                        listener.report(ReportType::ExplFeas, &fallback, &instance);
                    }
                    winner = Some(P1WinnerRecord {
                        source: control_source,
                        start_index: None,
                    });
                    best = Some(fallback);
                } else {
                    warn!(
                        "[STRUCT:P1] rejected invalid emergency control: {:?}",
                        control.validation_error
                    );
                }
                report.p1_control_candidate = Some(control);
            }

            report.p1_deadline_exhausted = terminator.kill() && !terminator.stop_requested();
            report.cancelled = terminator.stop_requested();
            let (selected, finalized_report) =
                finalize_p1_selection(report, best, winner, started)?;
            report = finalized_report;
            selected
        }
    };
    report.p1_deadline_exhausted = terminator.kill() && !terminator.stop_requested();

    let mut incumbent = p1_solution.clone();
    let mut best_stage = StructureStage::P1Construct;
    push_checkpoint(
        &mut report,
        StructureStage::P1Construct,
        &p1_solution,
        &incumbent,
        &instance,
        started,
    );
    if terminator.stop_requested() {
        return Ok(finish_structure_report(
            report, incumbent, best_stage, &instance, started, true,
        ));
    }
    report_stage_milestone(
        &mut sol_listener,
        &mut milestone_listener,
        StructureStage::P1Construct,
        ReportType::ExplFeas,
        &p1_solution,
        &instance,
    );
    if terminator.stop_requested() {
        return Ok(finish_structure_report(
            report, incumbent, best_stage, &instance, started, true,
        ));
    }

    // P2: a short failure-decay compression gives the UI a useful preview.
    let mut p2_prob = restored_problem(&instance, &p1_solution);
    let mut p2_cfg = *cmpr_config;
    p2_cfg.time_limit = config.p2_time_limit;
    p2_cfg.shrink_range = config.p2_shrink_range;
    p2_cfg.shrink_decay = ShrinkDecayStrategy::FailureBased(config.p2_failure_decay);
    // P5 cannot make an off-grid incumbent monotone by width alone. Keep the
    // entire structure path on the supplied discrete angles from P2 onward.
    p2_cfg.separator_config.sample_config.tilt_range = 0.0;
    let mut p2_sep = Separator::new(
        instance.clone(),
        p2_prob.clone(),
        p2_compress_rng,
        p2_cfg.separator_config,
    );
    p2_sep.configure_reinsertion(nfp_integration);
    p2_sep.freeze_items(frozen_item_ids);
    terminator.new_timeout(p2_cfg.time_limit);
    let p2_solution = compression_phase(
        &instance,
        &mut p2_sep,
        &p1_solution,
        &mut sol_listener,
        terminator,
        &p2_cfg,
    );
    if consider_incumbent(&instance, &mut incumbent, &p2_solution) {
        best_stage = StructureStage::P2Compress;
    }
    push_checkpoint(
        &mut report,
        StructureStage::P2Compress,
        &p2_solution,
        &incumbent,
        &instance,
        started,
    );
    if terminator.stop_requested() {
        return Ok(finish_structure_report(
            report, incumbent, best_stage, &instance, started, true,
        ));
    }
    report_stage_milestone(
        &mut sol_listener,
        &mut milestone_listener,
        StructureStage::P2Compress,
        ReportType::CmprFeas,
        &incumbent,
        &instance,
    );
    if terminator.stop_requested() {
        return Ok(finish_structure_report(
            report, incumbent, best_stage, &instance, started, true,
        ));
    }

    // P3: widen only the container (placements are unchanged), then let the
    // existing exploration machinery restructure and shrink from that warm
    // start. The P2 incumbent remains available if the experiment adds no value.
    let p2_for_p3 = if solution_is_valid(&instance, &p2_solution) {
        p2_solution.clone()
    } else {
        incumbent.clone()
    };
    crate::util::frozen::restore(&mut p2_prob, &p2_for_p3);
    let slack_width = p2_for_p3.strip_width() * (1.0 + config.p3_slack_ratio);
    if slack_width > p2_prob.strip_width() {
        crate::util::frozen::change_strip_width(&mut p2_prob, slack_width);
    }
    let mut p3_cfg = *expl_config;
    if let Some(limit) = config.p3_time_limit {
        p3_cfg.time_limit = limit;
    }
    p3_cfg.separator_config.sample_config.tilt_range = 0.0;
    let mut p3_sep = Separator::new(
        instance.clone(),
        p2_prob,
        p3_explore_rng,
        p3_cfg.separator_config,
    );
    p3_sep.configure_reinsertion(nfp_integration);
    p3_sep.freeze_items(frozen_item_ids);
    terminator.new_timeout(p3_cfg.time_limit);
    if config.p3_slack_ratio > 0.0 && !terminator.kill() {
        // Widening the right boundary alone does not change any neighbourhood.
        // Inject one exact-feasible profile repair while slack exists so P3
        // genuinely starts from a different structure instead of immediately
        // replaying the same shrink trajectory.
        let mut slack_tabu = BandTabu::new();
        let slack_rnr = RuinRecreateConfig {
            period: 1,
            band_fraction: config.p4_ruin_fraction,
            max_items: config.p4_max_items,
            ..crate::config::DEFAULT_SWARM_CONFIG.expl_cfg.rnr
        };
        ruin_and_recreate_profile_cancellable(
            &mut p3_sep,
            &slack_rnr,
            RuinStrategy::GapTarget,
            &mut slack_tabu,
            profile_cfg,
            terminator,
        );
    }
    #[cfg(not(feature = "adaptive"))]
    let p3_solutions = exploration_phase(
        &instance,
        &mut p3_sep,
        &mut sol_listener,
        terminator,
        &p3_cfg,
    );
    #[cfg(feature = "adaptive")]
    let p3_solutions = crate::optimizer::explore_mods::exploration_phase_adaptive(
        &instance,
        &mut p3_sep,
        &mut sol_listener,
        terminator,
        &p3_cfg,
    );
    let p3_solution = narrowest(&p3_solutions).unwrap_or(&p2_for_p3).clone();
    if consider_incumbent(&instance, &mut incumbent, &p3_solution) {
        best_stage = StructureStage::P3Explore;
    }
    push_checkpoint(
        &mut report,
        StructureStage::P3Explore,
        &p3_solution,
        &incumbent,
        &instance,
        started,
    );
    if terminator.stop_requested() {
        return Ok(finish_structure_report(
            report, incumbent, best_stage, &instance, started, true,
        ));
    }
    report_stage_milestone(
        &mut sol_listener,
        &mut milestone_listener,
        StructureStage::P3Explore,
        ReportType::ExplFeas,
        &incumbent,
        &instance,
    );
    if terminator.stop_requested() {
        return Ok(finish_structure_report(
            report, incumbent, best_stage, &instance, started, true,
        ));
    }

    // P4: gap-target D&R, exact separation, exact slide, then resume normal
    // exploration. Each failed cycle is rolled back to the global incumbent.
    let mut tabu = BandTabu::new();
    let mut stagnant = 0usize;
    for cycle in 0..config.p4_cycles {
        if stagnant >= config.p4_stagnation_cycles {
            break;
        }
        terminator.new_timeout(config.p4_cycle_time_limit);
        if terminator.kill() {
            break;
        }
        let before = incumbent.clone();
        if terminator.kill() {
            break;
        }
        let mut cycle_sep_cfg = expl_config.separator_config;
        cycle_sep_cfg.sample_config.tilt_range = 0.0;
        let mut cycle_sep = Separator::new(
            instance.clone(),
            restored_problem(&instance, &before),
            next_rng(),
            cycle_sep_cfg,
        );
        cycle_sep.configure_reinsertion(nfp_integration);
        cycle_sep.freeze_items(frozen_item_ids);
        let rnr_cfg = RuinRecreateConfig {
            period: 1,
            band_fraction: config.p4_ruin_fraction,
            max_items: config.p4_max_items,
            ..crate::config::DEFAULT_SWARM_CONFIG.expl_cfg.rnr
        };
        ruin_and_recreate_profile_cancellable(
            &mut cycle_sep,
            &rnr_cfg,
            config.p4_ruin_strategy,
            &mut tabu,
            profile_cfg,
            terminator,
        );
        let (separated, snapshot) = cycle_sep.separate_with_repair(
            terminator,
            &mut sol_listener,
            &expl_config.repair,
            false,
        );
        let mut feasible = snapshot.get_total_loss() == 0.0;
        let mut cycle_solution = before.clone();
        if feasible {
            cycle_sep.rollback(&separated, Some(&snapshot));
            slide::slide_pass(&mut cycle_sep, &polish_config.slide, terminator);
            cycle_sep.refit_strip();
            cycle_solution = cycle_sep.prob.save();

            if !terminator.kill() {
                let mut cycle_expl = *expl_config;
                cycle_expl.time_limit = config.p4_cycle_time_limit;
                cycle_expl.separator_config.sample_config.tilt_range = 0.0;
                #[cfg(not(feature = "adaptive"))]
                let resumed = exploration_phase(
                    &instance,
                    &mut cycle_sep,
                    &mut sol_listener,
                    terminator,
                    &cycle_expl,
                );
                #[cfg(feature = "adaptive")]
                let resumed = crate::optimizer::explore_mods::exploration_phase_adaptive(
                    &instance,
                    &mut cycle_sep,
                    &mut sol_listener,
                    terminator,
                    &cycle_expl,
                );
                if let Some(best) = narrowest(&resumed)
                    && best.strip_width() < cycle_solution.strip_width()
                {
                    cycle_solution = best.clone();
                }
            }
            // Only exact-feasible snapshots returned by separation/exploration
            // reach this point. Slide itself also verifies every move.
            feasible = true;
        }

        let improved = feasible && consider_incumbent(&instance, &mut incumbent, &cycle_solution);
        if improved {
            stagnant = 0;
            best_stage = StructureStage::P4Refine;
        } else {
            stagnant += 1;
        }
        report.repair_cycles.push(RepairCycleRecord {
            cycle,
            elapsed_secs: started.elapsed().as_secs_f32(),
            width_before: before.strip_width(),
            width_after: if feasible {
                cycle_solution.strip_width()
            } else {
                before.strip_width()
            },
            density_before: before.density(),
            density_after: if feasible {
                cycle_solution.density()
            } else {
                before.density()
            },
            feasible,
            improved_global_incumbent: improved,
        });
    }
    if terminator.stop_requested() {
        return Ok(finish_structure_report(
            report, incumbent, best_stage, &instance, started, true,
        ));
    }
    push_checkpoint(
        &mut report,
        StructureStage::P4Refine,
        &incumbent,
        &incumbent,
        &instance,
        started,
    );
    report_stage_milestone(
        &mut sol_listener,
        &mut milestone_listener,
        StructureStage::P4Refine,
        ReportType::ExplImproving,
        &incumbent,
        &instance,
    );
    if terminator.stop_requested() {
        return Ok(finish_structure_report(
            report, incumbent, best_stage, &instance, started, true,
        ));
    }

    // P5: snap any tilt back to the supplied allowed angles, repair if that
    // creates collisions, then run the existing exact deterministic polish.
    let before_p5 = incumbent.clone();
    let mut p5_sep_cfg = cmpr_config.separator_config;
    p5_sep_cfg.sample_config.tilt_range = 0.0;
    let mut p5_sep = Separator::new(
        instance.clone(),
        restored_problem(&instance, &before_p5),
        p5_polish_rng.clone(),
        p5_sep_cfg,
    );
    p5_sep.configure_reinsertion(nfp_integration);
    p5_sep.freeze_items(frozen_item_ids);
    let mut p5_seed = before_p5.clone();
    // Snap repair and polish share one absolute P5 budget.
    terminator.new_timeout(polish_config.time_limit);
    if config.p5_snap_rotations && snap_to_allowed_rotations(&mut p5_sep) > 0 {
        let (snapped, snapshot) = p5_sep.separate(terminator, &mut sol_listener);
        // Zero surrogate loss is necessary but not sufficient: the snapped seed
        // must also pass the exact, frozen-aware validation before P5 polishes
        // from it. Otherwise fall through to the restore branch.
        let repaired_seed = (snapshot.get_total_loss() == 0.0)
            .then(|| {
                p5_sep.rollback(&snapped, Some(&snapshot));
                p5_sep.refit_strip();
                p5_sep.prob.save()
            })
            .filter(|seed| solution_is_valid(&instance, seed));
        if let Some(seed) = repaired_seed {
            p5_seed = seed;
            consider_p5_incumbent(
                &instance,
                &mut incumbent,
                &p5_seed,
                &mut best_stage,
                &frozen_mask,
            );
        } else {
            warn!("[STRUCT:P5] angle snap could not be repaired; restoring incumbent");
            p5_sep = Separator::new(
                instance.clone(),
                restored_problem(&instance, &before_p5),
                p5_polish_rng,
                p5_sep_cfg,
            );
            p5_sep.configure_reinsertion(nfp_integration);
            p5_sep.freeze_items(frozen_item_ids);
            p5_seed = before_p5.clone();
        }
    }
    let polished = polish_phase_with_nfp(
        &instance,
        &mut p5_sep,
        &p5_seed,
        &mut sol_listener,
        terminator,
        polish_config,
        nfp_integration,
        lp_solver,
    );
    consider_p5_incumbent(
        &instance,
        &mut incumbent,
        &polished,
        &mut best_stage,
        &frozen_mask,
    );
    if terminator.stop_requested() {
        return Ok(finish_structure_report(
            report, incumbent, best_stage, &instance, started, true,
        ));
    }
    push_checkpoint(
        &mut report,
        StructureStage::P5Finish,
        &polished,
        &incumbent,
        &instance,
        started,
    );

    report_stage_milestone(
        &mut sol_listener,
        &mut milestone_listener,
        StructureStage::P5Finish,
        ReportType::Final,
        &incumbent,
        &instance,
    );
    Ok(finish_structure_report(
        report, incumbent, best_stage, &instance, started, false,
    ))
}

fn finish_structure_report(
    mut report: StructureRunReport,
    incumbent: SPSolution,
    best_stage: StructureStage,
    _instance: &SPInstance,
    started: Instant,
    cancelled: bool,
) -> (SPSolution, StructureRunReport) {
    report.cancelled = cancelled;
    report.best_stage = best_stage;
    report.final_width = incumbent.strip_width();
    report.final_density = incumbent.density();
    report.elapsed_secs = started.elapsed().as_secs_f32();
    (incumbent, report)
}

fn report_stage_milestone(
    sol_listener: &mut Option<&mut dyn SolutionListener>,
    milestone_listener: &mut Option<
        &mut dyn FnMut(StructureStage, ReportType, &SPSolution, &SPInstance),
    >,
    stage: StructureStage,
    report: ReportType,
    solution: &SPSolution,
    instance: &SPInstance,
) {
    if let Some(listener) = milestone_listener.as_deref_mut() {
        listener(stage, report, solution, instance);
    } else if let Some(listener) = sol_listener.as_deref_mut() {
        listener.report(report, solution, instance);
    }
}

/// P0 모티프 생성 설정. 조각 겹침 허용이 켜진 입력에서는 모티프 **안쪽** 조각 사이에도
/// 구성기와 같은 최소 간격을 둔다 — 간격 0 인 띠·블록은 충돌 도형이 정확히 맞닿아 원본 겹침
/// 계약을 어기고, 구성기가 그 모티프를 통째로 버리게 된다. 겹침 허용이 없으면 설정 그대로다.
#[cfg_attr(not(feature = "cpguard-policy-off"), cpguard::obfuscate)]
fn motif_config_for(
    instance: &SPInstance,
    config: &StructureFirstConfig,
) -> crate::structure::motif::MotifConfig {
    let mut motif = config.motif.clone();
    if crate::original_overlap::OriginalOverlapGuard::is_active(instance) {
        motif.band_gap = motif
            .band_gap
            .max(crate::optimizer::lbf::CONTRACT_BACKOFF_MM[0]);
    }
    motif
}

#[cfg_attr(not(feature = "cpguard-policy-off"), cpguard::obfuscate)]
fn profile_config(instance: &SPInstance, config: &StructureFirstConfig) -> ProfilePlacementConfig {
    let min_dimension = instance
        .items
        .iter()
        .map(|(item, _)| item.shape_cd().bbox().width().min(item.shape_cd().bbox().height()))
        .filter(|dimension| dimension.is_finite() && *dimension > 0.0)
        .fold(f32::INFINITY, f32::min);
    let row_height = if min_dimension.is_finite() {
        (min_dimension * config.profile_row_height_ratio).max(1.0e-4)
    } else {
        1.0
    };
    ProfilePlacementConfig {
        row_height,
        max_rows: config.profile_max_rows,
        weights: config.profile_weights,
        top_k: config.profile_top_k,
        item_beam_width: config.item_beam_width,
        candidate_pool_cap: config.candidate_pool_cap,
        slide_refine_steps: config.slide_refine_steps,
    }
}

fn restored_problem(instance: &SPInstance, solution: &SPSolution) -> SPProblem {
    let mut problem = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
    crate::util::frozen::restore(&mut problem, solution);
    problem
}

fn narrowest(solutions: &[SPSolution]) -> Option<&SPSolution> {
    solutions
        .iter()
        .min_by(|a, b| a.strip_width().total_cmp(&b.strip_width()))
}

fn consider_p5_incumbent(
    instance: &SPInstance,
    incumbent: &mut SPSolution,
    candidate: &SPSolution,
    best_stage: &mut StructureStage,
    frozen_mask: &[bool],
) -> bool {
    if solution_validation_error_with_frozen(instance, candidate, frozen_mask).is_some()
        || !crate::util::assertions::frozen_poses_preserved(incumbent, candidate, frozen_mask)
    {
        return false;
    }
    // P5 also retains exact interior movement when a barrier pins the width.
    // The shared Polish ordering never accepts a wider compact candidate.
    let improved = solution_validation_error_with_frozen(instance, incumbent, frozen_mask)
        .is_some()
        || crate::optimizer::polish::prefers_compact_solution(candidate, incumbent);
    if improved {
        *incumbent = candidate.clone();
        *best_stage = StructureStage::P5Finish;
    }
    improved
}

pub fn solution_validation_error(
    instance: &SPInstance,
    solution: &SPSolution,
) -> Option<P1ValidationError> {
    solution_validation_error_with_quantity_policy(
        instance,
        solution,
        true,
        &[],
        &crate::optimizer::place_groups::PlaceGroups::default(),
    )
}

/// Validate a partial seed layout. Missing demand is allowed because LBF will
/// place it afterwards, but duplicates beyond demand are rejected before
/// `SPProblem::place_item` can underflow its demand counter.
pub fn partial_solution_validation_error(
    instance: &SPInstance,
    solution: &SPSolution,
) -> Option<P1ValidationError> {
    solution_validation_error_with_quantity_policy(
        instance,
        solution,
        false,
        &[],
        &crate::optimizer::place_groups::PlaceGroups::default(),
    )
}

pub fn solution_validation_error_with_frozen(
    instance: &SPInstance,
    solution: &SPSolution,
    frozen: &[bool],
) -> Option<P1ValidationError> {
    solution_validation_error_with_quantity_policy(
        instance,
        solution,
        true,
        frozen,
        &crate::optimizer::place_groups::PlaceGroups::default(),
    )
}

pub fn solution_validation_error_with_frozen_ids(
    instance: &SPInstance,
    solution: &SPSolution,
    ids: &[usize],
) -> Option<P1ValidationError> {
    solution_validation_error_with_frozen(
        instance,
        solution,
        &crate::util::frozen::mask(instance.items.len(), ids),
    )
}

pub fn partial_solution_validation_error_with_frozen_ids(
    instance: &SPInstance,
    solution: &SPSolution,
    ids: &[usize],
) -> Option<P1ValidationError> {
    solution_validation_error_with_quantity_policy(
        instance,
        solution,
        false,
        &crate::util::frozen::mask(instance.items.len(), ids),
        &crate::optimizer::place_groups::PlaceGroups::default(),
    )
}

pub fn solution_validation_error_with_frozen_and_groups(
    instance: &SPInstance,
    solution: &SPSolution,
    frozen: &[bool],
    groups: &crate::optimizer::place_groups::PlaceGroups,
) -> Option<P1ValidationError> {
    solution_validation_error_with_quantity_policy(instance, solution, true, frozen, groups)
}

pub fn solution_validation_error_with_frozen_ids_and_groups(
    instance: &SPInstance,
    solution: &SPSolution,
    ids: &[usize],
    groups: &crate::optimizer::place_groups::PlaceGroups,
) -> Option<P1ValidationError> {
    solution_validation_error_with_frozen_and_groups(
        instance,
        solution,
        &crate::util::frozen::mask(instance.items.len(), ids),
        groups,
    )
}

pub fn partial_solution_validation_error_with_frozen_ids_and_groups(
    instance: &SPInstance,
    solution: &SPSolution,
    ids: &[usize],
    groups: &crate::optimizer::place_groups::PlaceGroups,
) -> Option<P1ValidationError> {
    solution_validation_error_with_quantity_policy(
        instance,
        solution,
        false,
        &crate::util::frozen::mask(instance.items.len(), ids),
        groups,
    )
}

fn solution_validation_error_with_quantity_policy(
    instance: &SPInstance,
    solution: &SPSolution,
    require_complete: bool,
    frozen: &[bool],
    groups: &crate::optimizer::place_groups::PlaceGroups,
) -> Option<P1ValidationError> {
    #[cfg(feature = "init-profile")]
    let _profile = crate::init_profile::Scope::new("detail.validation_total");

    if crate::config::runtime::init_cancel_requested() {
        return Some(P1ValidationError::Cancelled);
    }
    if super::certification::contains(instance, solution, require_complete, frozen, groups) {
        #[cfg(feature = "init-profile")]
        crate::init_profile::count("validation.cache_hits", 1);
        return None;
    }

    #[cfg(feature = "init-profile")]
    let mut _profile_validation = crate::init_profile::Scope::new("detail.validation_metadata");
    let mut counts = vec![0usize; instance.items.len()];
    for (_, placed) in solution.layout_snapshot().placed_items().iter() {
        if crate::config::runtime::init_cancel_requested() { return Some(P1ValidationError::Cancelled); }
        let Some(count) = counts.get_mut(placed.item().idx()) else {
            return Some(P1ValidationError::UnknownItemId {
                item_id: placed.item().idx(),
            });
        };
        *count += 1;
        let (x, y) = placed.d_transf().translation();
        if !x.is_finite() || !y.is_finite() || !placed.d_transf().rotation().is_finite() {
            return Some(P1ValidationError::LayoutInfeasible {
                flat_exact_feasible: false,
                exterior_collisions: 1,
                placed_item_collisions: 0,
                other_collisions: 0,
                first_query_item_id: Some(placed.item().idx()),
                first_hazard_item_id: None,
            });
        }
        let allowed = match instance.item(placed.item().idx()).allowed_orientations().rotations() {
            RotationRange::None => angular_distance(placed.d_transf().rotation(), 0.0) <= 1.0e-5,
            RotationRange::Continuous => true,
            RotationRange::Discrete(angles) => angles
                .iter()
                .any(|angle| angular_distance(placed.d_transf().rotation(), *angle) <= 1.0e-5),
        };
        if !allowed {
            return Some(P1ValidationError::RotationNotAllowed {
                item_id: placed.item().idx(),
            });
        }
    }
    for (item_id, actual) in counts.into_iter().enumerate() {
        let expected = instance.item_qty(item_id);
        if (require_complete && actual != expected) || (!require_complete && actual > expected) {
            return Some(P1ValidationError::QuantityMismatch {
                item_id,
                expected,
                actual,
            });
        }
    }

    let groups_preserved = if require_complete {
        groups.solution_preserves_relative_poses(solution)
    } else {
        groups.partial_solution_preserves_relative_poses(solution)
    };
    if !groups_preserved {
        return Some(P1ValidationError::LayoutInfeasible {
            flat_exact_feasible: false,
            exterior_collisions: 0,
            placed_item_collisions: 0,
            other_collisions: 1,
            first_query_item_id: None,
            first_hazard_item_id: None,
        });
    }

    // `d_transf` is the value exported through the public API. Rebuild every
    // collision shape and dynamic hazard from it instead of trusting the
    // redundant shape/CDE copies stored in LayoutSnapshot. This also catches
    // accidental direct PlacedItem mutation that would otherwise validate the
    // old geometry while exporting the new transformation.
    let strip_width = solution.strip_width();
    if !strip_width.is_finite() || strip_width <= 0.0 {
        return Some(P1ValidationError::LayoutInfeasible {
            flat_exact_feasible: false,
            exterior_collisions: 1,
            placed_item_collisions: 0,
            other_collisions: 0,
            first_query_item_id: None,
            first_hazard_item_id: None,
        });
    }
    #[cfg(feature = "init-profile")]
    _profile_validation.next("detail.validation_rebuild");
    let mut rebuilt = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
    rebuilt.change_strip_width(strip_width).expect(crate::dmsg!("strip width must be positive"));
    let container_bbox = rebuilt.layout().container().outer_cd().bbox();
    let container_bounds = [
        container_bbox.x_min,
        container_bbox.x_max,
        container_bbox.y_min,
        container_bbox.y_max,
    ];
    let mut exterior_collisions = 0usize;
    let mut boxes = Vec::with_capacity(solution.layout_snapshot().placed_items().len());
    for placed in solution.layout_snapshot().placed_items().values() {
        if crate::config::runtime::init_cancel_requested() { return Some(P1ValidationError::Cancelled); }
        let item = instance.item(placed.item().idx());
        let shape = item.shape_cd().transform_clone(&placed.d_transf().compose());
        let bbox = shape.bbox();
        boxes.push(bbox);
        if !placed.d_transf().rotation().is_finite()
            || ![bbox.x_min, bbox.x_max, bbox.y_min, bbox.y_max]
                .iter()
                .all(|v| v.is_finite())
            || (!crate::util::frozen::is_frozen(frozen, placed.item().idx())
                && !bbox_is_contained_mm(
                    container_bounds,
                    [bbox.x_min, bbox.x_max, bbox.y_min, bbox.y_max],
                ))
        {
            exterior_collisions += 1;
        }
    }
    if exterior_collisions != 0 {
        return Some(P1ValidationError::LayoutInfeasible {
            flat_exact_feasible: false,
            exterior_collisions,
            placed_item_collisions: 0,
            other_collisions: 0,
            first_query_item_id: None,
            first_hazard_item_id: None,
        });
    }
    crate::util::frozen::prepare_placements(&mut rebuilt, boxes);
    for placed in solution.layout_snapshot().placed_items().values() {
        if crate::config::runtime::init_cancel_requested() { return Some(P1ValidationError::Cancelled); }
        rebuilt.place_item(SPPlacement {
            item_idx: placed.item().idx(),
            d_transf: placed.d_transf(),
        });
    }
    #[cfg(feature = "init-profile")]
    _profile_validation.next("detail.validation_collisions");
    let layout = &rebuilt.layout();
    let (exterior_collisions, placed_item_collisions, other_collisions, first_placed_pair) =
        match crate::eval::lbf_evaluator::flat_layout_collision_counts_with_control_and_groups(
            layout,
            frozen,
            groups,
            &crate::config::runtime::init_cancel_requested,
        ) {
            Some(counts) => counts,
            None => return Some(P1ValidationError::Cancelled),
        };
    // Jagua's exterior hazard treats exact contact with a rectangular strip
    // boundary as a collision. The public hint contract permits x=0/y=0/top
    // contact, so bbox containment is authoritative for the strip exterior;
    // item pairs, holes and inferior-quality zones remain exact rejection
    // causes.
    let flat_exact_feasible = placed_item_collisions == 0 && other_collisions == 0;
    #[cfg(feature = "init-profile")]
    _profile_validation.next("detail.validation_original");
    if flat_exact_feasible {
        if let Err(error) =
            crate::original_overlap::validate_with_groups(instance, solution, frozen, groups)
        {
            warn!("[ORIGINAL:VALIDATE] {error}");
            return Some(P1ValidationError::LayoutInfeasible {
                flat_exact_feasible: false,
                exterior_collisions: 0,
                placed_item_collisions: 1,
                other_collisions: 0,
                first_query_item_id: None,
                first_hazard_item_id: None,
            });
        }
        if crate::config::runtime::init_cancel_requested() {
            return Some(P1ValidationError::Cancelled);
        }
        super::certification::insert(instance, solution, require_complete, frozen, groups);
        None
    } else {
        Some(P1ValidationError::LayoutInfeasible {
            flat_exact_feasible,
            exterior_collisions,
            placed_item_collisions,
            other_collisions,
            first_query_item_id: first_placed_pair.map(|pair| pair.0),
            first_hazard_item_id: first_placed_pair.map(|pair| pair.1),
        })
    }
}

/// Public strip-boundary policy shared by validation, callback loss and
/// collision-status reporting. Coordinates are millimetres in
/// `[x_min, x_max, y_min, y_max]` order.
///
/// The absolute term preserves the documented exact-contact tolerance at
/// ordinary garment coordinates. The ULP-aware term prevents a mathematically
/// interior placement from being rejected merely because `f32` can no longer
/// represent `boundary + 1e-4` at large strip coordinates.
pub fn bbox_is_contained_mm(container: [f32; 4], shape: [f32; 4]) -> bool {
    const ABS_TOLERANCE_MM: f32 = 1.0e-4;
    const ULP_MULTIPLIER: f32 = 2.0;
    let x_scale = [container[0], container[1], shape[0], shape[1]]
        .into_iter()
        .map(f32::abs)
        .fold(1.0f32, f32::max);
    let y_scale = [container[2], container[3], shape[2], shape[3]]
        .into_iter()
        .map(f32::abs)
        .fold(1.0f32, f32::max);
    let x_tolerance = ABS_TOLERANCE_MM.max(ULP_MULTIPLIER * f32::EPSILON * x_scale);
    let y_tolerance = ABS_TOLERANCE_MM.max(ULP_MULTIPLIER * f32::EPSILON * y_scale);
    shape[0] >= container[0] - x_tolerance
        && shape[1] <= container[1] + x_tolerance
        && shape[2] >= container[2] - y_tolerance
        && shape[3] <= container[3] + y_tolerance
}

#[cfg(test)]
mod boundary_policy_tests {
    use super::{
        P1CandidateSource, P1ValidationError, StructureRunReport, StructureStage,
        bbox_is_contained_mm, consider_p5_incumbent, evaluate_p1_candidate, finalize_p1_selection,
        partial_solution_validation_error, solution_validation_error, try_optimize_structure_first,
    };
    use crate::config::DEFAULT_SWARM_CONFIG;
    use crate::structure::config::DEFAULT_STRUCTURE_FIRST_CONFIG;
    use crate::util::terminator::{BasicTerminator, Terminator};
    use jagua_rs::Instant;
    use jagua_rs::geometry::DTransformation;
    use jagua_rs::probs::spp::entities::{SPPlacement, SPProblem};
    use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
    use rand::SeedableRng;
    use rand::rngs::Xoshiro256PlusPlus;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::time::Duration;

    struct StopOnNthPoll {
        polls: AtomicUsize,
        stop_at: usize,
        timeout: Option<Instant>,
    }

    impl StopOnNthPoll {
        fn new(stop_at: usize) -> Self {
            Self {
                polls: AtomicUsize::new(0),
                stop_at,
                timeout: None,
            }
        }
    }

    impl Terminator for StopOnNthPoll {
        fn kill(&self) -> bool {
            self.polls.fetch_add(1, Ordering::Relaxed) + 1 >= self.stop_at
        }

        fn stop_requested(&self) -> bool {
            self.polls.load(Ordering::Relaxed) >= self.stop_at
        }

        fn new_timeout(&mut self, timeout: Duration) {
            self.timeout = Some(Instant::now() + timeout);
        }

        fn timeout_at(&self) -> Option<Instant> {
            self.timeout
        }
    }

    fn two_rectangle_problem() -> SPProblem {
        let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
            "name":"validation-rebuild",
            "strip_height":20.0,
            "items":[
                {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                    "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
                }},
                {"id":1,"demand":1,"allowed_orientations":[0.0],"shape":{
                    "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
                }}
            ]
        }))
        .unwrap();
        let config = DEFAULT_SWARM_CONFIG;
        let importer = crate::util::io::new_importer(config.cde_config, config.poly_simpl_tolerance, config.min_item_separation, config.narrow_concavity_cutoff_ratio);
        let instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext).unwrap();
        let mut problem = SPProblem::new(instance).expect(crate::dmsg!("strip forms a valid container"));
        problem.change_strip_width(30.0).expect(crate::dmsg!("strip width must be positive"));
        problem.place_item(SPPlacement {
            item_idx: 0,
            d_transf: DTransformation::new(0.0, (5.0, 5.0)),
        });
        problem.place_item(SPPlacement {
            item_idx: 1,
            d_transf: DTransformation::new(0.0, (15.0, 5.0)),
        });
        problem
    }

    fn run_accounting_pipeline(
        problem: &SPProblem,
        initial: &jagua_rs::probs::spp::entities::SPSolution,
    ) -> (
        jagua_rs::probs::spp::entities::SPSolution,
        StructureRunReport,
    ) {
        let mut swarm_config = DEFAULT_SWARM_CONFIG;
        swarm_config.expl_cfg.time_limit = Duration::ZERO;
        swarm_config.polish_cfg.enabled = false;
        swarm_config.polish_cfg.time_limit = Duration::ZERO;
        let mut structure_config = DEFAULT_STRUCTURE_FIRST_CONFIG;
        structure_config.enabled = true;
        structure_config.construction_starts = 1;
        structure_config.p1_include_legacy_baseline = true;
        structure_config.p1_time_limit = Duration::from_secs(2);
        structure_config.p2_time_limit = Duration::ZERO;
        structure_config.p3_time_limit = Some(Duration::ZERO);
        structure_config.p4_cycles = 0;
        let mut terminator = BasicTerminator::new();

        try_optimize_structure_first(
            problem.instance().clone(),
            Xoshiro256PlusPlus::seed_from_u64(41),
            None,
            &mut terminator,
            &swarm_config.expl_cfg,
            &swarm_config.cmpr_cfg,
            &swarm_config.polish_cfg,
            &swarm_config.nfp_integration,
            &swarm_config.lp_solver,
            Some(initial),
            &[],
            swarm_config.lbf_x_multiplier,
            swarm_config.lbf_y_multiplier,
            &swarm_config.lbf_raster,
            swarm_config.lbf_order,
            &structure_config,
        )
        .unwrap()
    }

    #[test]
    fn exact_valid_warm_start_skips_p0_with_zeroed_accounting() {
        let problem = two_rectangle_problem();
        let initial = problem.save();
        let (_, report) = run_accounting_pipeline(&problem, &initial);

        assert_eq!(
            report.p1_winner.unwrap().source,
            P1CandidateSource::WarmStart
        );
        assert_eq!(report.p0_elapsed_secs, 0.0);
        assert!(!report.p0_stopped);
        assert_eq!(report.motif_generation_stats, Default::default());
        assert_eq!(report.motif_selection_stats, Default::default());
        assert_eq!(
            (
                report.motif_singles,
                report.motif_pairs,
                report.motif_blocks,
                report.motif_bands,
                report.motif_fft_pairs,
            ),
            (0, 0, 0, 0, 0)
        );
        assert_eq!(report.pair_matching_method, "none");
        assert_eq!(
            (
                report.pair_matching_vertices,
                report.pair_matching_edges,
                report.pair_matching_dp_states,
                report.pair_matching_selected_pairs,
            ),
            (0, 0, 0, 0)
        );
    }

    #[test]
    fn invalid_warm_start_still_runs_p0_before_reconstruction() {
        let mut problem = two_rectangle_problem();
        let missing_key = problem
            .layout()
            .placed_items()
            .iter()
            .find_map(|(key, placed)| (placed.item().idx() == 1).then_some(key))
            .unwrap();
        problem.remove_item(missing_key);
        let invalid = problem.save();
        let (solution, report) = run_accounting_pipeline(&problem, &invalid);

        assert_eq!(report.motif_generation_stats.input_parts, 2);
        assert!(report.motif_singles > 0);
        assert!(!report.p0_stopped);
        assert_ne!(
            report.p1_winner.unwrap().source,
            P1CandidateSource::WarmStart
        );
        assert!(solution_validation_error(&problem.instance(), &solution).is_none());
    }

    #[test]
    fn issue3_p5_retains_equal_width_interior_progress() {
        let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
            "name": "issue3-frozen-barrier", "strip_height": 20.0,
            "items": (0..3).map(|id| serde_json::json!({
                "id": id, "demand": 1, "allowed_orientations": [0.0],
                "shape": {"type": "rectangle", "data": {
                    "x_min": 0.0, "y_min": 0.0, "width": 10.0,
                    "height": if id == 2 { 4.0 } else { 18.0 }
                }}
            })).collect::<Vec<_>>()
        }))
        .unwrap();
        let mut cfg = DEFAULT_SWARM_CONFIG;
        let importer = crate::util::io::new_importer(cfg.cde_config, cfg.poly_simpl_tolerance, cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
        let instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext).unwrap();
        let mut problem = SPProblem::new(instance).expect(crate::dmsg!("strip forms a valid container"));
        problem.change_strip_width(120.0).expect(crate::dmsg!("strip width must be positive"));
        for (item_id, x, y) in [(0, 90.0, 1.0), (1, 100.001175, 1.0), (2, 50.0, 3.0)] {
            let bbox = problem.instance().items[item_id].0.shape_cd().bbox();
            problem.place_item(SPPlacement {
                item_idx: item_id,
                d_transf: DTransformation::new(0.0, (x - bbox.x_min, y - bbox.y_min)),
            });
        }
        crate::util::frozen::fit_strip_with_frozen(&mut problem, &[true, false, false]);
        let initial = problem.save();
        assert!(
            super::solution_validation_error_with_frozen(
                &problem.instance(),
                &initial,
                &[true, false, false]
            )
            .is_none()
        );
        cfg.polish_cfg.time_limit = Duration::from_millis(200);
        cfg.polish_cfg.lp.enabled = false;
        cfg.polish_cfg.nfp_left.enabled = false;
        cfg.polish_cfg.reclaim.enabled = false;
        let mut structure = DEFAULT_STRUCTURE_FIRST_CONFIG;
        structure.enabled = true;
        structure.p1_time_limit = Duration::from_secs(1);
        structure.p2_time_limit = Duration::ZERO;
        structure.p3_time_limit = Some(Duration::ZERO);
        structure.p4_cycles = 0;
        let (solution, report) = try_optimize_structure_first(
            problem.instance().clone(),
            Xoshiro256PlusPlus::seed_from_u64(41),
            None,
            &mut BasicTerminator::new(),
            &cfg.expl_cfg,
            &cfg.cmpr_cfg,
            &cfg.polish_cfg,
            &cfg.nfp_integration,
            &cfg.lp_solver,
            Some(&initial),
            &[0],
            cfg.lbf_x_multiplier,
            cfg.lbf_y_multiplier,
            &cfg.lbf_raster,
            cfg.lbf_order,
            &structure,
        )
        .unwrap();
        assert_eq!(solution.strip_width(), initial.strip_width());
        let interior = solution
            .layout_snapshot()
            .placed_items()
            .values()
            .find(|p| p.item().idx() == 2)
            .unwrap();
        assert!(
            interior.shape().bbox().x_min < 1.0,
            "P5 must retain left packing even at equal width: {}",
            interior.shape().bbox().x_min
        );
        assert_eq!(report.best_stage, StructureStage::P5Finish);
        let frozen = |s: &jagua_rs::probs::spp::entities::SPSolution| {
            s.layout_snapshot()
                .placed_items()
                .values()
                .find(|p| p.item().idx() == 0)
                .unwrap()
                .d_transf()
        };
        assert_eq!(frozen(&solution), frozen(&initial));
        assert!(
            super::solution_validation_error_with_frozen(
                &problem.instance(),
                &solution,
                &[true, false, false]
            )
            .is_none()
        );
    }

    #[test]
    fn p5_snap_improvement_keeps_provenance_when_polish_is_unchanged() {
        let problem = two_rectangle_problem();
        let mut incumbent = problem.save();
        let mut narrower_problem = two_rectangle_problem();
        narrower_problem.change_strip_width(25.0).expect(crate::dmsg!("strip width must be positive"));
        let snapped = narrower_problem.save();
        assert!(solution_validation_error(&problem.instance(), &snapped).is_none());

        let mut best_stage = StructureStage::P4Refine;
        assert!(consider_p5_incumbent(
            &problem.instance(),
            &mut incumbent,
            &snapped,
            &mut best_stage,
            &[],
        ));
        let polished_without_improvement = snapped.clone();
        assert!(!consider_p5_incumbent(
            &problem.instance(),
            &mut incumbent,
            &polished_without_improvement,
            &mut best_stage,
            &[],
        ));
        assert_eq!(best_stage, StructureStage::P5Finish);
        assert_eq!(incumbent.strip_width(), snapped.strip_width());
    }

    #[test]
    fn exact_contact_and_large_coordinate_rounding_are_contained() {
        assert!(bbox_is_contained_mm(
            [0.0, 10_000.0, 0.0, 3_000.0],
            [0.0, 100.0, 10.0, 3_000.0]
        ));
        // At this scale a mathematically interior f64 result can round one
        // f32 ULP above the boundary. The shared policy accepts that rounding.
        assert!(bbox_is_contained_mm(
            [0.0, 10_000.0, 0.0, 3_000.0],
            [10.0, 100.0, 10.0, 3_000.0002]
        ));
    }

    #[test]
    fn material_outside_placement_is_rejected() {
        assert!(!bbox_is_contained_mm(
            [0.0, 10_000.0, 0.0, 1_800.0],
            [0.0, 100.0, -0.01, 100.0]
        ));
    }

    #[test]
    fn validation_rebuilds_shapes_from_public_transformations() {
        let problem = two_rectangle_problem();
        let mut solution = problem.save();
        let second_key = solution
            .layout_snapshot()
            .placed_items()
            .iter()
            .find(|(_, placed)| placed.item().idx() == 1)
            .map(|(key, _)| key)
            .unwrap();

        // Reproduce the legacy stripe bug: only the exported transformation
        // changes while the cached collision shape and CDE remain at x=15.
        solution.layout_snapshot_mut().placed_items_mut()[second_key].set_d_transf(DTransformation::new(0.0, (5.0, 5.0)));

        assert!(matches!(
            solution_validation_error(&problem.instance(), &solution),
            Some(P1ValidationError::LayoutInfeasible {
                placed_item_collisions: 1,
                ..
            })
        ));
    }

    #[test]
    fn partial_validation_allows_missing_but_not_excess_demand() {
        let mut problem = two_rectangle_problem();
        let second_key = problem
            .layout()
            .placed_items()
            .iter()
            .find(|(_, placed)| placed.item().idx() == 1)
            .map(|(key, _)| key)
            .unwrap();
        problem.remove_item(second_key);

        assert!(partial_solution_validation_error(&problem.instance(), &problem.save()).is_none());
        assert!(matches!(
            solution_validation_error(&problem.instance(), &problem.save()),
            Some(P1ValidationError::QuantityMismatch {
                item_id: 1,
                expected: 1,
                actual: 0,
            })
        ));
    }

    #[test]
    fn legacy_and_emergency_controls_keep_exact_rejections_in_the_audit() {
        let mut problem = two_rectangle_problem();
        let missing_key = problem
            .layout()
            .placed_items()
            .iter()
            .find(|(_, placed)| placed.item().idx() == 1)
            .map(|(key, _)| key)
            .unwrap();
        problem.remove_item(missing_key);
        let invalid = problem.save();

        for source in [
            P1CandidateSource::LegacyBaseline,
            P1CandidateSource::EmergencyLegacyFallback,
        ] {
            let (record, selectable) = evaluate_p1_candidate(
                &problem.instance(),
                &invalid,
                source,
                None,
                0.0,
                true,
                false,
                None,
            );
            assert!(!selectable);
            assert!(!record.complete);
            assert!(!record.exact_valid);
            assert!(record.width.is_none());
            assert!(record.density.is_none());
            assert!(matches!(
                record.validation_error,
                Some(P1ValidationError::QuantityMismatch {
                    item_id: 1,
                    expected: 1,
                    actual: 0,
                })
            ));
        }
    }

    #[test]
    fn p1_selection_returns_an_explicit_failure_with_the_rejected_audit() {
        let mut problem = two_rectangle_problem();
        let missing_key = problem
            .layout()
            .placed_items()
            .iter()
            .find(|(_, placed)| placed.item().idx() == 1)
            .map(|(key, _)| key)
            .unwrap();
        problem.remove_item(missing_key);
        let invalid = problem.save();
        let (record, selectable) = evaluate_p1_candidate(
            &problem.instance(),
            &invalid,
            P1CandidateSource::LegacyBaseline,
            None,
            0.0,
            true,
            false,
            None,
        );
        assert!(!selectable);
        let mut report = StructureRunReport::new();
        report.p1_control_candidate = Some(record);

        let failure =
            finalize_p1_selection(report, None, None, jagua_rs::Instant::now()).unwrap_err();
        assert_eq!(failure.stage, StructureStage::P1Construct);
        assert!(failure.report.p1_winner.is_none());
        let rejected = failure.report.p1_control_candidate.unwrap();
        assert!(!rejected.exact_valid);
        assert!(!rejected.selected_as_p1_incumbent);
    }

    #[test]
    fn legacy_p1_control_observes_stop_inside_construction() {
        let problem = two_rectangle_problem();
        let swarm_config = DEFAULT_SWARM_CONFIG;
        let mut structure_config = DEFAULT_STRUCTURE_FIRST_CONFIG;
        structure_config.enabled = true;
        structure_config.motif.enabled = false;
        structure_config.construction_starts = 1;
        structure_config.p1_include_legacy_baseline = true;
        structure_config.p1_time_limit = Duration::from_secs(30);

        // With motifs disabled, the seventh poll is the first poll made by
        // the legacy LBF constructor. A cooperative control must stop there
        // and must never publish the resulting partial layout as a baseline.
        let mut terminator = StopOnNthPoll::new(7);
        let failure = try_optimize_structure_first(
            problem.instance().clone(),
            Xoshiro256PlusPlus::seed_from_u64(1),
            None,
            &mut terminator,
            &swarm_config.expl_cfg,
            &swarm_config.cmpr_cfg,
            &swarm_config.polish_cfg,
            &swarm_config.nfp_integration,
            &swarm_config.lp_solver,
            None,
            &[],
            swarm_config.lbf_x_multiplier,
            swarm_config.lbf_y_multiplier,
            &swarm_config.lbf_raster,
            swarm_config.lbf_order,
            &structure_config,
        )
        .unwrap_err();

        assert_eq!(failure.stage, StructureStage::P1Construct);
        let control = failure.report.p1_control_candidate.unwrap();
        assert_eq!(control.source, P1CandidateSource::LegacyBaseline);
        assert!(control.stopped);
        assert!(!control.complete);
        assert!(!control.exact_valid);
        assert!(matches!(
            control.validation_error,
            Some(P1ValidationError::QuantityMismatch { .. })
        ));
    }

    #[test]
    fn stopped_p1_does_not_start_uncancellable_emergency_fallback() {
        let problem = two_rectangle_problem();
        let swarm_config = DEFAULT_SWARM_CONFIG;
        let mut structure_config = DEFAULT_STRUCTURE_FIRST_CONFIG;
        structure_config.enabled = true;
        structure_config.motif.enabled = false;
        structure_config.construction_starts = 1;
        structure_config.p1_include_legacy_baseline = false;
        structure_config.p1_time_limit = Duration::from_secs(30);

        let mut terminator = StopOnNthPoll::new(1);
        let failure = try_optimize_structure_first(
            problem.instance().clone(),
            Xoshiro256PlusPlus::seed_from_u64(2),
            None,
            &mut terminator,
            &swarm_config.expl_cfg,
            &swarm_config.cmpr_cfg,
            &swarm_config.polish_cfg,
            &swarm_config.nfp_integration,
            &swarm_config.lp_solver,
            None,
            &[],
            swarm_config.lbf_x_multiplier,
            swarm_config.lbf_y_multiplier,
            &swarm_config.lbf_raster,
            swarm_config.lbf_order,
            &structure_config,
        )
        .unwrap_err();

        assert_eq!(failure.stage, StructureStage::P1Construct);
        assert!(failure.report.cancelled);
        assert!(failure.report.p1_control_candidate.is_none());
        assert!(failure.report.p1_winner.is_none());
    }

    /// 정확히 맞닿은 두 조각: 쿼드트리 판정은 접촉을 충돌로 세고 인증은 접촉을 허용한다.
    /// 폭 맞춤은 인증 기준을 따라야 한다 — 예전에는 쿼드트리 판정만 보고 조용히 물러났다.
    #[test]
    fn refit_strip_accepts_a_certified_contact_layout_the_quadtree_rejects() {
        let mut problem = two_rectangle_problem();
        let second = problem
            .layout()
            .placed_items()
            .iter()
            .find(|(_, placed)| placed.item().idx() == 1)
            .map(|(key, _)| key)
            .unwrap();
        problem.remove_item(second);
        // 4x4 조각 둘을 x 로 정확히 4 만큼 떨어뜨려 놓는다: 변이 맞닿는다.
        problem.place_item(SPPlacement {
            item_idx: 1,
            d_transf: DTransformation::new(0.0, (9.0, 5.0)),
        });
        assert!(
            !problem.layout().is_collision_free(),
            "exact contact must be a quadtree collision for this test to mean anything"
        );
        assert!(solution_validation_error(&problem.instance(), &problem.save()).is_none());

        let right = problem
            .layout()
            .placed_items()
            .values()
            .map(|placed| placed.shape().bbox().x_max)
            .fold(f32::NEG_INFINITY, f32::max);
        let mut config = DEFAULT_SWARM_CONFIG.expl_cfg.separator_config;
        config.n_workers = 1;
        let mut sep = crate::optimizer::separator::Separator::new(
            problem.instance().clone(),
            problem,
            Xoshiro256PlusPlus::seed_from_u64(0),
            config,
        );
        assert!(sep.refit_strip());
        assert!((sep.prob.strip_width() - right * 1.00001).abs() < 1.0e-3);
    }

    /// 구조 힌트 구성기가 만든 P1 후보 하나를 실제 입력에서 재현하고, 원본 겹침 보정이
    /// 왜 실패하는지 본다. `AB_INPUT` 이 없으면 건너뛴다.
    ///
    /// 환경 변수: `AB_INPUT`, `AB_OVERLAP_MM`(0.4), `AB_SEED`(1), `AB_OUT`(후보 배치 저장 경로).
    #[test]
    #[ignore = "needs AB_INPUT; diagnostic harness"]
    fn structure_hint_start_repair_diagnosis_on_an_input() {
        use crate::consts::LBF_SAMPLE_CONFIG;
        use crate::optimizer::lbf::LBFBuilder;
        use crate::optimizer::separator::Separator;
        use crate::original_overlap::OriginalOverlapGuard;
        use crate::structure::motif::MotifGenerator;

        let Ok(input) = std::env::var("AB_INPUT") else {
            println!("[A/B] AB_INPUT missing; skipped");
            return;
        };
        let env = |key: &str, default: &str| std::env::var(key).unwrap_or_else(|_| default.into());
        let overlap: f32 = env("AB_OVERLAP_MM", "0.4").parse().unwrap();
        let seed: u64 = env("AB_SEED", "1").parse().unwrap();

        let c = DEFAULT_SWARM_CONFIG;
        let (ext, _) = crate::util::io::read_spp_input(std::path::Path::new(&input)).unwrap();
        let instance = crate::util::io::import_spp_instance(
            &ext,
            c.cde_config,
            c.poly_simpl_tolerance,
            (overlap > 0.0).then_some(-overlap),
            c.narrow_concavity_cutoff_ratio,
        )
        .unwrap();
        let config = DEFAULT_STRUCTURE_FIRST_CONFIG.clone().sanitized();
        let profile_cfg = super::profile_config(&instance, &config);
        let mut never = || false;
        let generation = MotifGenerator::new(super::motif_config_for(&instance, &config))
            .generate_with_control_and_stats(&instance, &mut never);
        let selected: Vec<_> = generation
            .catalog
            .disjoint_ranked_with_stats(config.max_active_motifs)
            .motifs
            .into_iter()
            .cloned()
            .collect();
        println!("[A/B] selected motifs: {}", selected.len());

        let started = Instant::now();
        let mut builder = LBFBuilder::new(
            instance.clone(),
            Xoshiro256PlusPlus::seed_from_u64(seed),
            LBF_SAMPLE_CONFIG,
            c.lbf_x_multiplier,
            c.lbf_y_multiplier,
        )
        .with_order_mode(c.lbf_order)
        .with_raster_config(c.lbf_raster)
        .with_preselected_structure_hints(profile_cfg, selected)
        .construct_with_control(&mut never);
        let candidate = builder.prob.save();
        println!(
            "[A/B] hint start: {:.1}s, width {:.3} ({:.3}%), stats {:?}",
            started.elapsed().as_secs_f32(),
            candidate.strip_width(),
            candidate.density() * 100.0,
            builder.structure_hint_stats().map(|stats| stats.start.completed),
        );
        if let Ok(path) = std::env::var("AB_OUT") {
            let ext = jagua_rs::probs::spp::io::export(&candidate, *crate::EPOCH);
            std::fs::write(path, serde_json::to_string(&ext).unwrap()).unwrap();
        }
        println!(
            "[A/B] validation before repair: {:?}",
            solution_validation_error(&instance, &candidate)
        );

        let mut sep_config = c.expl_cfg.separator_config;
        sep_config.n_workers = 1;
        sep_config.nfp_sampler = false;
        let mut sep = Separator::new(
            instance.clone(),
            builder.prob.clone(),
            Xoshiro256PlusPlus::seed_from_u64(0),
            sep_config,
        );
        println!(
            "[A/B] surrogate loss of the candidate: {} (quadtree feasible: {})",
            sep.ct.get_total_loss(),
            sep.prob.layout().is_collision_free()
        );
        let term = BasicTerminator::new();
        let mut guard = OriginalOverlapGuard::new(&instance);
        let frozen = sep.frozen_mask.clone();
        let conflicts = guard.inspect(&candidate, &frozen, &term).unwrap();
        println!("[A/B] original-overlap conflicts: {}", conflicts.len());
        for v in conflicts.iter().take(10) {
            let a = &candidate.layout_snapshot().placed_items()[v.a];
            let b = &candidate.layout_snapshot().placed_items()[v.b];
            println!(
                "[A/B] conflict items {}/{} area {:.6} poses {:?} / {:?} cd-bbox {:?} / {:?}",
                a.item().idx(),
                b.item().idx(),
                v.area,
                a.d_transf().translation(),
                b.d_transf().translation(),
                a.shape().bbox(),
                b.shape().bbox(),
            );
        }
        // 충돌 쌍의 각 조각을 한 방향씩 움직였을 때 대리 손실과 원본 겹침이 어떻게 되는지.
        if let Some(worst) = conflicts.iter().max_by(|a, b| a.area.total_cmp(&b.area)) {
            for key in [worst.a, worst.b] {
                let pose = candidate.layout_snapshot().placed_items()[key].d_transf();
                let item_id = candidate.layout_snapshot().placed_items()[key].item().idx();
                let (x, y) = pose.translation();
                for step in [0.005f32, 0.1, 0.5, 1.0, 2.0, 4.0] {
                    for (dx, dy) in [(1., 0.), (-1., 0.), (0., 1.), (0., -1.)] {
                        let pk = sep.placement_key(item_id).unwrap();
                        sep.move_item(
                            pk,
                            DTransformation::new(pose.rotation(), (x + dx * step, y + dy * step)),
                        );
                        let loss = sep.ct.get_total_loss();
                        let moved = sep.prob.save();
                        let cost: f64 = guard
                            .inspect(&moved, &frozen, &term)
                            .unwrap()
                            .iter()
                            .map(|v| v.area)
                            .sum();
                        println!(
                            "[A/B] item {item_id} step {step} dir ({dx},{dy}): surrogate loss {loss}, original overlap {cost:.6}"
                        );
                        sep.rollback(&candidate, None);
                    }
                }
            }
        }
        println!("[A/B] guard.repair without width growth: {:?}", guard.repair(&mut sep, &term, false));
        println!("[A/B] guard.repair with width growth: {:?}", guard.repair(&mut sep, &term, true));
        super::repair_p1_original_overlap(&mut builder.prob, &[], &term);
        println!(
            "[A/B] validation after repair: {:?}",
            solution_validation_error(&instance, &builder.prob.save())
        );
    }
}

#[allow(clippy::too_many_arguments)]
/// P1 후보에 일반 Init 경로와 같은 원본 겹침 보정을 건다.
///
/// 조각 겹침 허용이 켜진 입력에서 구성기는 충돌 도형(침식된 외곽) 기준으로만 조각을 놓는다.
/// 그 배치가 원본 외곽 기준의 겹침 계약을 어기는 경우가 있고(실측: babel 겉감 200조각,
/// 허용 0.4mm 에서 한 쌍), 일반 경로는 Init 직후 이를 보정한다. P1 은 보정 없이 후보를
/// 검증해서 legacy control 까지 기각했고, 파이프라인 전체가 «P1 후보 없음» 으로 실패했다.
///
/// 계약을 이미 지키는 후보에는 아무 일도 하지 않는다(검증 한 번) — 겹침 허용이 없는 입력의
/// 결과와 난수 흐름은 그대로다. 보정에 실패하면 후보를 건드리지 않고, 뒤의 검증이 기각한다.
fn repair_p1_original_overlap(
    prob: &mut jagua_rs::probs::spp::entities::SPProblem,
    frozen_item_ids: &[usize],
    term: &impl Terminator,
) {
    let groups = crate::optimizer::place_groups::PlaceGroups::default();
    match crate::original_overlap::repair_problem(prob, frozen_item_ids, &groups, term) {
        Ok(true) => {}
        Ok(false) => warn!("[STRUCT:P1] original-overlap repair did not converge"),
        Err(error) => warn!("[STRUCT:P1] original-overlap repair failed: {error}"),
    }
}

fn evaluate_p1_candidate(
    instance: &SPInstance,
    solution: &SPSolution,
    source: P1CandidateSource,
    start_index: Option<usize>,
    elapsed_secs: f32,
    constructor_complete: bool,
    stopped: bool,
    structure_hints: Option<StructureHintStats>,
) -> (P1CandidateRecord, bool) {
    let validation_error = solution_validation_error(instance, solution);
    let quantity_complete = !matches!(
        validation_error,
        Some(P1ValidationError::UnknownItemId { .. } | P1ValidationError::QuantityMismatch { .. })
    );
    let complete = constructor_complete && quantity_complete;
    let exact_valid = validation_error.is_none();
    let selectable = complete && exact_valid;
    (
        P1CandidateRecord {
            source,
            start_index,
            elapsed_secs,
            complete,
            exact_valid,
            validation_error,
            stopped,
            width: selectable.then(|| solution.strip_width()),
            density: selectable.then(|| solution.density()),
            improved_incumbent: false,
            selected_as_p1_incumbent: false,
            structure_hints,
        },
        selectable,
    )
}

fn finalize_p1_selection(
    mut report: StructureRunReport,
    best: Option<SPSolution>,
    winner: Option<P1WinnerRecord>,
    started: Instant,
) -> Result<(SPSolution, StructureRunReport), StructurePipelineFailure> {
    let (Some(best), Some(winner)) = (best, winner) else {
        report.p1_winner = None;
        report.p1_selected_legacy_baseline = false;
        report.elapsed_secs = started.elapsed().as_secs_f32();
        return Err(StructurePipelineFailure {
            stage: StructureStage::P1Construct,
            message: "no complete exact-feasible P1 candidate was produced".to_string(),
            report,
        });
    };

    if let Some(control) = report.p1_control_candidate.as_mut() {
        control.selected_as_p1_incumbent =
            control.source == winner.source && winner.start_index.is_none();
    }
    for start in &mut report.p1_start_stats {
        start.selected_as_p1_incumbent =
            start.source == winner.source && start.start_index == winner.start_index;
    }
    report.p1_selected_legacy_baseline = matches!(
        winner.source,
        P1CandidateSource::LegacyBaseline | P1CandidateSource::EmergencyLegacyFallback
    );
    report.p1_winner = Some(winner);
    Ok((best, report))
}

pub fn solution_is_valid(instance: &SPInstance, solution: &SPSolution) -> bool {
    solution_validation_error(instance, solution).is_none()
}

#[allow(clippy::too_many_arguments)]
fn push_checkpoint(
    report: &mut StructureRunReport,
    stage: StructureStage,
    stage_solution: &SPSolution,
    incumbent: &SPSolution,
    _instance: &SPInstance,
    started: Instant,
) {
    report.checkpoints.push(StageCheckpoint {
        stage,
        elapsed_secs: started.elapsed().as_secs_f32(),
        width: stage_solution.strip_width(),
        density: stage_solution.density(),
        is_global_incumbent: (stage_solution.strip_width() - incumbent.strip_width()).abs()
            < 1.0e-5,
        incumbent_width: incumbent.strip_width(),
        incumbent_density: incumbent.density(),
    });
}

fn snap_to_allowed_rotations(separator: &mut Separator) -> usize {
    let placements = separator
        .prob
        .layout()
        .placed_items()
        .iter()
        .map(|(key, placed)| (key, placed.item().idx(), placed.d_transf()))
        .collect::<Vec<_>>();
    let mut changed = 0;
    for (key, item_id, transform) in placements {
        if separator.frozen_mask.get(item_id).copied().unwrap_or(false) {
            continue;
        }
        let target = match separator.instance.item(item_id).allowed_orientations().rotations() {
            RotationRange::None => 0.0,
            RotationRange::Continuous => continue,
            RotationRange::Discrete(angles) => angles
                .iter()
                .copied()
                .min_by(|a, b| {
                    angular_distance(transform.rotation(), *a)
                        .total_cmp(&angular_distance(transform.rotation(), *b))
                })
                .unwrap_or(transform.rotation()),
        };
        if angular_distance(transform.rotation(), target) <= 1.0e-6 {
            continue;
        }
        separator.move_item(key, DTransformation::new(target, transform.translation()));
        changed += 1;
    }
    changed
}

fn angular_distance(a: f32, b: f32) -> f32 {
    let delta = a - b;
    delta.sin().atan2(delta.cos()).abs()
}
