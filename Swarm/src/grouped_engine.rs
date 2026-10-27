//! One-handle execution engine for the v2 all-item/size-region input.
#[cfg(test)]
use crate::structure::pipeline::solution_validation_error;

use crate::config::{CompressionConfig, ExplorationConfig, PolishConfig, SwarmConfig};
use crate::consts::LBF_SAMPLE_CONFIG;
use crate::grouped_input::{NestingMode, PlacedHintInput, ValidatedInput};
use crate::optimizer::compress::compression_phase;
use crate::optimizer::lbf::LBFBuilder;
use crate::optimizer::lns::{BandTabu, RuinStrategy, ruin_and_recreate_profile_cancellable};
use crate::group_expansion::GroupExpansion;
use crate::optimizer::place_groups::{PlaceGroupMember, PlaceGroupSpec, PlaceGroups};
use crate::optimizer::polish::{polish_phase_with_nfp, prefers_compact_solution, slide};
use crate::optimizer::separator::Separator;
use crate::quantify::tracker::CollisionTracker;
use crate::structure::evaluator::ProfilePlacementConfig;
use crate::structure::motif::{
    MotifGenerationStats, MotifGenerator, MotifSelectionStats, PlacementMotif,
};
use crate::structure::pipeline::{
    P1CandidateRecord, P1CandidateSource, P1WinnerRecord, RepairCycleRecord, StageCheckpoint,
    StructureRunReport, StructureStage,
};
use crate::util::listener::{ReportType, SolutionListener};
use crate::util::terminator::Terminator;
use jagua_rs::Instant;
use jagua_rs::geometry::geo_enums::RotationRange;
use jagua_rs::geometry::geo_traits::{Transformable, TransformableFrom};
use jagua_rs::geometry::primitives::SPolygon;
use jagua_rs::geometry::{DTransformation, Transformation};
use jagua_rs::io::ext_repr::{ExtLayout, ExtPlacedItem};
use jagua_rs::io::import::ext_to_int_transformation;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution};
use jagua_rs::probs::spp::io::ext_repr::ExtSPSolution;
use rand::rngs::Xoshiro256PlusPlus;
use rand::{RngExt, SeedableRng};
use rayon::prelude::*;
use serde::Serialize;
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GroupPhase {
    Init,
    Exploration,
    Compression,
    Polish,
    Structure,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StructureProgressKind {
    InitialBaseline,
    CandidatePreview,
    LaneCompleted,
    SchedulerRound,
    StageCompleted,
}

#[derive(Clone)]
struct RegionLane {
    instance: SPInstance,
    prob: SPProblem,
    solution: SPSolution,
    rng: Xoshiro256PlusPlus,
    local_to_global: Vec<u64>,
    place_groups: PlaceGroups,
    frozen_ids: Vec<usize>,
    frozen_reference: HashMap<usize, DTransformation>,
    gap_after_mm: f32,
    config: SwarmConfig,
    force_single_worker: bool,
    /// Size-region scheduling recreates a `Separator` for every quantum. Keep
    /// only the GLS weight donor from the previous slice of the same phase;
    /// the next separator still rebuilds losses and frozen state from the
    /// authoritative lane problem before inheriting survivor weights.
    tracker_session: Option<LaneTrackerSession>,
    /// Per-phase scheduling feedback for size-region lanes. A timed-out slice
    /// that leaves the exact incumbent width unchanged earns a larger slice on
    /// its next turn; successful progress keeps the learned level.
    slice_adaptation: Option<LaneSliceAdaptation>,
    structure_incumbent: Option<SPSolution>,
    structure_report: Option<StructureRunReport>,
    structure_fallback: Option<String>,
    /// P3 injects exactly one GapTarget profile repair after the slack
    /// boundary is opened. Size-region scheduling may resume Explore across
    /// several quanta, so this flag prevents reseeding every slice.
    structure_p3_repair_pending: bool,
    structure_tabu: BandTabu,
    structure_started_at: Option<Instant>,
}

struct P1LaneSession {
    selected_motifs: Option<Vec<PlacementMotif>>,
    profile_cfg: ProfilePlacementConfig,
    construction_starts: usize,
    next_start_index: usize,
    incumbent: SPSolution,
    incumbent_exact_valid: bool,
    report: StructureRunReport,
    stopped_after_invalid_candidate: bool,
}

struct P1LaneWorkResult {
    candidate_preview: Option<SPSolution>,
    finished: bool,
}

struct LaneSolutionForwarder<'a> {
    callback: &'a mut dyn FnMut(ReportType, &SPInstance, &SPSolution),
}

impl SolutionListener for LaneSolutionForwarder<'_> {
    fn report(&mut self, report: ReportType, solution: &SPSolution, instance: &SPInstance) {
        (self.callback)(report, instance, solution);
    }
}

pub struct GroupedStepState {
    mode: NestingMode,
    full_instance: SPInstance,
    full_place_groups: PlaceGroups,
    /// Multipolygon place groups: the engine works on compact ids, the public
    /// boundary expands them to member ids and poses.
    expansion: Option<std::sync::Arc<GroupExpansion>>,
    frozen_global_ids: Vec<usize>,
    lanes: Vec<RegionLane>,
    fixed_placements: Vec<ExtPlacedItem>,
    fixed_right: f32,
    locked_prefix_gap_mm: f32,
    group_workers: usize,
    effective_group_workers: usize,
    quantum: Duration,
    structure_enabled: bool,
    /// Lane solutions that most recently passed the merged full-instance
    /// authority. Structure cancellation or a globally invalid stage rolls
    /// back this set atomically.
    structure_public_incumbents: Option<Vec<SPSolution>>,
    phase: GroupPhase,
    started_at: Instant,
    #[cfg(test)]
    panic_next_legacy_phase: bool,
    #[cfg(test)]
    observed_group_pool_workers: Vec<usize>,
}

#[derive(Clone, Debug, Serialize)]
struct GroupedLaneStructureReport {
    schema_version: u32,
    lane_index: usize,
    global_item_ids: Vec<u64>,
    place_group_count: usize,
    frozen_item_count: usize,
    p0_p1_soft_structure_applied: bool,
    fallback_reason: Option<String>,
    report: StructureRunReport,
}

#[derive(Clone, Debug, Serialize)]
struct GroupedStructureReport {
    schema_version: u32,
    nesting_mode: &'static str,
    group_workers: usize,
    effective_group_workers: usize,
    quantum_ms: u64,
    fixed_prefix_item_count: usize,
    fixed_prefix_right_mm: f32,
    locked_prefix_gap_mm: f32,
    required_region_gap_mm: f32,
    lane_count: usize,
    lanes: Vec<GroupedLaneStructureReport>,
    final_strip_width: f32,
    final_density: f32,
}

impl GroupedStepState {
    pub fn new(mut input: ValidatedInput, config: SwarmConfig) -> Result<Self, String> {
        if config.allow_180_rotation {
            crate::util::io::augment_orientations_180(&mut input.instance);
        }
        check_init_cancel()?;
        restrict_group_orientations(&mut input)?;
        // Multipolygon mode: every place group becomes one multipart item. The
        // engine only ever sees the compact instance; `expansion` restores the
        // public ids and member poses at the public boundary.
        let expansion = if config.place_group_mode == crate::config::PlaceGroupMode::MultiPolygon
            && !input.place_groups.is_empty()
        {
            let (compact, expansion) = crate::group_expansion::compact_input(input)?;
            input = compact;
            Some(std::sync::Arc::new(expansion))
        } else {
            None
        };

        // Jagua simplifies item collision polygons by inflating them and the
        // strip polygon by deflating it.  This search geometry is conservative,
        // so Structure can keep the configured simplification instead of
        // multiplying the vertex cost for every lane and every P0-P5 stage.
        // 겹침 허용일 때 원단 경계를 열지 않는 import — 근거는
        // `util::io::import_spp_instance` 문서. 전체 인스턴스와 레인이 **같은**
        // 설정을 써야 한다. 갈리면 레인만 원단 밖으로 나간다.
        let import_config = ImportConfig {
            cde_config: config.cde_config,
            poly_simpl_tolerance: config.poly_simpl_tolerance,
            simplify_max_deviation_mm: config.simplify_max_deviation_mm,
            min_item_separation: config.min_item_separation,
            narrow_concavity_cutoff_ratio: config.narrow_concavity_cutoff_ratio,
        };
        let mut full_instance = crate::util::io::import_spp_instance_with_mm(
            &input.instance,
            import_config.cde_config,
            import_config.poly_simpl_tolerance,
            import_config.min_item_separation,
            import_config.narrow_concavity_cutoff_ratio,
            import_config.simplify_max_deviation_mm,
        )
        .map_err(|e| format!("failed to import v2 instance: {e}"))?;
        // Lanes clone these items, so the policy reaches every lane.
        crate::original_overlap::apply_policy(&mut full_instance, config.original_overlap_policy);
        check_init_cancel()?;
        let full_group_specs = input
            .place_groups
            .iter()
            .map(|group| {
                let members = group
                    .member_item_ids
                    .iter()
                    .map(|&item_id| {
                        Ok(PlaceGroupMember {
                            item_id: item_id as usize,
                            initial_transform: hint_to_internal(
                                &full_instance,
                                item_id as usize,
                                &input.hints[&item_id],
                            )?,
                        })
                    })
                    .collect::<Result<Vec<_>, String>>()?;
                Ok(PlaceGroupSpec {
                    id: group.id as u64,
                    members,
                })
            })
            .collect::<Result<Vec<_>, String>>()?;
        validate_group_geometry(
            &full_instance,
            &full_group_specs,
            &input
                .frozen_item_ids()
                .iter()
                .map(|&id| id as usize)
                .collect::<Vec<_>>(),
        )?;
        let full_place_groups =
            PlaceGroups::new(&full_instance, full_group_specs).map_err(|e| e.to_string())?;
        let frozen_global = input.frozen_item_ids();
        // A fixed prefix is a size-region rule only. In all_items mode frozen
        // placements remain in the single lane and are already exported by
        // that lane; adding them here would duplicate item ids in the result.
        let (fixed_placements, fixed_right) = match input.nesting.mode {
            NestingMode::SizeRegions => {
                build_fixed_prefix(&full_instance, &input.hints, &frozen_global)?
            }
            NestingMode::AllItems => (vec![], 0.0),
        };

        let mut seed_rng = match config.rng_seed {
            Some(seed) => Xoshiro256PlusPlus::seed_from_u64(seed as u64),
            None => Xoshiro256PlusPlus::seed_from_u64(rand::random()),
        };

        let region_ids: Vec<(Vec<u64>, f32)> = match input.nesting.mode {
            NestingMode::AllItems => vec![(
                input
                    .instance
                    .items
                    .iter()
                    .map(|item| item.base.id)
                    .collect(),
                0.0,
            )],
            NestingMode::SizeRegions => input
                .regions
                .iter()
                .map(|region| {
                    (
                        region
                            .item_ids
                            .iter()
                            .filter(|id| !frozen_global.contains(id))
                            .copied()
                            .collect::<Vec<_>>(),
                        region.gap_after_mm,
                    )
                })
                .filter(|(ids, _)| !ids.is_empty())
                .collect(),
        };

        let force_single_worker = input.nesting.mode == NestingMode::SizeRegions;
        let mut lanes = Vec::with_capacity(region_ids.len());
        for (region_index, (global_ids, gap_after_mm)) in region_ids.into_iter().enumerate() {
            check_init_cancel()?;
            let mut lane_config = config;
            if force_single_worker {
                lane_config.expl_cfg.separator_config.n_workers = 1;
                lane_config.cmpr_cfg.separator_config.n_workers = 1;
            }
            let lane_seed = derive_lane_seed(seed_rng.random(), region_index as u64);
            let lane = build_lane(
                &input,
                global_ids,
                gap_after_mm,
                lane_seed,
                lane_config,
                &full_instance,
                force_single_worker,
            )?;
            check_init_cancel()?;
            lanes.push(lane);
        }

        if lanes.is_empty() && fixed_placements.is_empty() {
            return Err("v2 input produced neither movable lanes nor fixed placements".into());
        }

        // Capture once: affinity and the environment are established before handle
        // creation, and the report must describe the same budget as both pools.
        let effective_group_workers =
            runtime_group_worker_budget(input.nesting.group_workers, lanes.len());
        Ok(Self {
            frozen_global_ids: frozen_global.iter().map(|&id| id as usize).collect(),
            mode: input.nesting.mode,
            full_instance,
            full_place_groups,
            expansion,
            lanes,
            fixed_placements,
            fixed_right,
            locked_prefix_gap_mm: input.nesting.locked_prefix_gap_mm,
            group_workers: input.nesting.group_workers.max(1),
            effective_group_workers,
            quantum: Duration::from_millis(input.nesting.quantum_ms),
            structure_enabled: config.structure.enabled,
            structure_public_incumbents: None,
            phase: GroupPhase::Init,
            started_at: Instant::now(),
            #[cfg(test)]
            panic_next_legacy_phase: false,
            #[cfg(test)]
            observed_group_pool_workers: Vec::new(),
        })
    }

    pub fn phase(&self) -> GroupPhase {
        self.phase
    }

    pub fn run_exploration(&mut self, stop: &AtomicBool) -> Result<(), String> {
        self.run_exploration_with_progress(stop, &mut |_| {})
    }

    /// Runs the legacy exploration phase and publishes coherent, globally
    /// merged size-region snapshots after scheduler rounds.
    pub fn run_exploration_with_progress(
        &mut self,
        stop: &AtomicBool,
        on_round: &mut dyn FnMut(&GroupedStepState),
    ) -> Result<(), String> {
        self.run_exploration_impl(stop, on_round, None)
    }

    /// Runs exploration without slicing the `all_items` compatibility lane,
    /// while forwarding that lane's native solver reports to the caller.
    pub fn run_exploration_with_callbacks(
        &mut self,
        stop: &AtomicBool,
        on_round: &mut dyn FnMut(&GroupedStepState),
        on_all_items_solution: &mut dyn FnMut(ReportType, &SPInstance, &SPSolution),
    ) -> Result<(), String> {
        self.run_exploration_impl(stop, on_round, Some(on_all_items_solution))
    }

    fn run_exploration_impl(
        &mut self,
        stop: &AtomicBool,
        on_round: &mut dyn FnMut(&GroupedStepState),
        on_all_items_solution: Option<&mut dyn FnMut(ReportType, &SPInstance, &SPSolution)>,
    ) -> Result<(), String> {
        // 탐색·압축·후처리는 순서와 횟수가 자유다 — 레인은 언제나 자기 현재 해에서
        // 출발한다. Structure 는 별도 경로라 그 뒤에는 이을 수 없다.
        if self.phase == GroupPhase::Structure {
            return Err("exploration is not valid after structure".into());
        }
        self.run_certified_legacy_phase_transaction(
            GroupPhase::Exploration,
            "grouped exploration boundary failed",
            |state| {
                state.run_scheduled_with_callbacks(
                    stop,
                    LanePhase::Exploration,
                    on_round,
                    on_all_items_solution,
                )
            },
        )
    }

    pub fn run_compression(&mut self, stop: &AtomicBool) -> Result<(), String> {
        self.run_compression_with_progress(stop, &mut |_| {})
    }

    /// Runs the legacy compression phase and publishes coherent, globally
    /// merged size-region snapshots after scheduler rounds.
    pub fn run_compression_with_progress(
        &mut self,
        stop: &AtomicBool,
        on_round: &mut dyn FnMut(&GroupedStepState),
    ) -> Result<(), String> {
        self.run_compression_impl(stop, on_round, None)
    }

    /// Runs compression and forwards native progress from an uninterrupted
    /// `all_items` lane. Size-region callers continue to receive merged rounds.
    pub fn run_compression_with_callbacks(
        &mut self,
        stop: &AtomicBool,
        on_round: &mut dyn FnMut(&GroupedStepState),
        on_all_items_solution: &mut dyn FnMut(ReportType, &SPInstance, &SPSolution),
    ) -> Result<(), String> {
        self.run_compression_impl(stop, on_round, Some(on_all_items_solution))
    }

    fn run_compression_impl(
        &mut self,
        stop: &AtomicBool,
        on_round: &mut dyn FnMut(&GroupedStepState),
        on_all_items_solution: Option<&mut dyn FnMut(ReportType, &SPInstance, &SPSolution)>,
    ) -> Result<(), String> {
        if self.phase == GroupPhase::Structure {
            return Err("compression is not valid after structure".into());
        }
        self.run_certified_legacy_phase_transaction(
            GroupPhase::Compression,
            "grouped compression boundary failed",
            |state| {
                state.run_scheduled_with_callbacks(
                    stop,
                    LanePhase::Compression,
                    on_round,
                    on_all_items_solution,
                )
            },
        )
    }

    pub fn run_polish(&mut self, stop: &AtomicBool) -> Result<(), String> {
        self.run_polish_with_progress(stop, &mut |_| {})
    }

    /// Runs the legacy polish phase and publishes coherent, globally merged
    /// size-region snapshots after scheduler rounds.
    pub fn run_polish_with_progress(
        &mut self,
        stop: &AtomicBool,
        on_round: &mut dyn FnMut(&GroupedStepState),
    ) -> Result<(), String> {
        self.run_polish_impl(stop, on_round, None)
    }

    /// Runs polish and forwards native progress from an uninterrupted
    /// `all_items` lane. A rigid PlaceGroup may intentionally skip polish.
    pub fn run_polish_with_callbacks(
        &mut self,
        stop: &AtomicBool,
        on_round: &mut dyn FnMut(&GroupedStepState),
        on_all_items_solution: &mut dyn FnMut(ReportType, &SPInstance, &SPSolution),
    ) -> Result<(), String> {
        self.run_polish_impl(stop, on_round, Some(on_all_items_solution))
    }

    fn run_polish_impl(
        &mut self,
        stop: &AtomicBool,
        on_round: &mut dyn FnMut(&GroupedStepState),
        on_all_items_solution: Option<&mut dyn FnMut(ReportType, &SPInstance, &SPSolution)>,
    ) -> Result<(), String> {
        if self.phase == GroupPhase::Structure {
            return Err("polish is not valid after structure".into());
        }
        // No lane mutates in this case: certify the public boundary directly,
        // without cloning a transaction or allocating a scheduler/Separators.
        // StructureFinish has its own path and rotation snap is unaffected.
        if self.lanes.iter().all(|lane| {
            !crate::optimizer::polish::has_active_passes(
                &lane.config.polish_cfg,
                !lane.place_groups.is_empty(),
            )
        }) {
            self.export_certified_solution()
                .map_err(|e| format!("grouped polish boundary failed: {e}"))?;
            self.phase = GroupPhase::Polish;
            return Ok(());
        }
        self.run_certified_legacy_phase_transaction(
            GroupPhase::Polish,
            "grouped polish boundary failed",
            |state| {
                state.run_scheduled_with_callbacks(
                    stop,
                    LanePhase::Polish,
                    on_round,
                    on_all_items_solution,
                )
            },
        )
    }

    /// Runs one legacy grouped phase as an atomic lane transaction. A scheduler
    /// failure or a rejected merged boundary restores every lane, including its
    /// problem, solution, RNG stream and resumable state, together with the
    /// previously certified phase.
    fn run_certified_legacy_phase_transaction(
        &mut self,
        next_phase: GroupPhase,
        boundary_context: &'static str,
        run: impl FnOnce(&mut GroupedStepState) -> Result<(), String>,
    ) -> Result<(), String> {
        let lanes_before = self.lanes.clone();
        let phase_before = self.phase;
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            #[cfg(test)]
            if self.panic_next_legacy_phase {
                self.panic_next_legacy_phase = false;
                panic!("{}", swarm_core::dmsg!("injected grouped legacy phase panic"));
            }
            run(self).and_then(|_| {
                self.export_certified_solution()
                    .map(|_| ())
                    .map_err(|error| format!("{boundary_context}: {error}"))
            })
        }));

        match result {
            Ok(Ok(())) => {
                self.phase = next_phase;
                Ok(())
            }
            Ok(Err(error)) => {
                self.lanes = lanes_before;
                self.phase = phase_before;
                Err(error)
            }
            Err(payload) => {
                self.lanes = lanes_before;
                self.phase = phase_before;
                // The FFI boundary owns panic-to-error-code translation. Keep
                // this transaction responsible only for restoring state, then
                // preserve the original panic payload for its outer catcher.
                std::panic::resume_unwind(payload)
            }
        }
    }

    /// Run the grouped Structure pipeline as one public-state transaction.
    /// Worker/scheduler errors may occur after several lanes have advanced;
    /// none of that transient state may become the next public incumbent.
    fn run_structure_transaction(
        &mut self,
        run: impl FnOnce(&mut GroupedStepState) -> Result<(), String>,
    ) -> Result<(), String> {
        let lanes_before = self.lanes.clone();
        let phase_before = self.phase;
        let public_incumbents_before = self.structure_public_incumbents.clone();
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| run(self)));
        match result {
            Ok(Ok(())) => Ok(()),
            Ok(Err(error)) => {
                self.lanes = lanes_before;
                self.phase = phase_before;
                self.structure_public_incumbents = public_incumbents_before;
                Err(error)
            }
            Err(payload) => {
                self.lanes = lanes_before;
                self.phase = phase_before;
                self.structure_public_incumbents = public_incumbents_before;
                // Preserve the payload so the public FFI boundary translates
                // the panic into its stable 9001 phase error.
                std::panic::resume_unwind(payload)
            }
        }
    }

    fn restore_structure_incumbents(&mut self) {
        let certified = self.structure_public_incumbents.clone();
        for (lane_index, lane) in self.lanes.iter_mut().enumerate() {
            let incumbent = certified
                .as_ref()
                .and_then(|solutions| solutions.get(lane_index))
                .cloned()
                .or_else(|| lane.structure_incumbent.clone())
                .unwrap_or_else(|| lane.solution.clone());
            lane.solution = incumbent.clone();
            lane.prob = restored_lane_problem(&lane.instance, &incumbent);
            lane.structure_incumbent = Some(incumbent);
            if let Some(report) = lane.structure_report.as_mut() {
                report.cancelled = true;
            } else {
                lane.structure_report = Some(StructureRunReport::cancelled_baseline(
                    &lane.instance,
                    &lane.solution,
                ));
            }
        }
        self.phase = GroupPhase::Structure;
    }

    fn certify_structure_public_state(&mut self) -> bool {
        if self.export_certified_solution().is_ok() {
            self.structure_public_incumbents = Some(
                self.lanes
                    .iter()
                    .map(|lane| lane.solution.clone())
                    .collect(),
            );
            return true;
        }

        let certified = self.structure_public_incumbents.clone().unwrap_or_default();
        for (lane, incumbent) in self.lanes.iter_mut().zip(certified) {
            lane.solution = incumbent.clone();
            lane.prob = restored_lane_problem(&lane.instance, &incumbent);
            lane.structure_incumbent = Some(incumbent);
        }
        false
    }

    /// Export and re-import the merged public layout, then apply the same
    /// symmetric authority used by Structure candidates. This final global
    /// check also covers fixed-prefix/lane-offset integration.
    pub fn export_certified_solution(&self) -> Result<ExtSPSolution, String> {
        if self
            .lanes
            .iter()
            .any(|lane| !lane.constraints_valid(&lane.solution))
        {
            return Err("public solution changed a frozen pose or rigid group".to_string());
        }
        let ext = self.export_solution();
        if !ext.strip_width.is_finite() || ext.strip_width <= 0.0 {
            return Err("merged public solution has an invalid strip width".to_string());
        }
        let solution = crate::util::frozen::import_solution(&self.full_instance, &ext).save();
        if let Some(error) =
            crate::structure::pipeline::solution_validation_error_with_frozen_ids_and_groups(
                &self.full_instance,
                &solution,
                &self.frozen_global_ids,
                &self.full_place_groups,
            )
        {
            return Err(format!(
                "merged public solution is not exact-feasible: {error:?}"
            ));
        }
        Ok(ext)
    }

    /// Runs the experimental P0-P5 path independently inside every size lane.
    ///
    /// Soft motifs are generated from the lane's local instance, so a motif
    /// can never span a size-region boundary. User PlaceGroups and frozen
    /// hints remain hard contracts: those lanes retain their already-built
    /// group/freeze-aware LBF solution and only enter the exact group-aware
    /// downstream phases.
    pub fn run_structure(&mut self, stop: &AtomicBool) -> Result<(), String> {
        self.run_structure_with_stage_callback(stop, &mut |_, _| {})
    }

    /// Runs structure-first and publishes one coherent merged solution after
    /// each completed P1-P5 stage. Lane-local solutions are never exposed: a
    /// callback always observes global item IDs and the required region gaps.
    pub fn run_structure_with_stage_callback(
        &mut self,
        stop: &AtomicBool,
        on_stage: &mut dyn FnMut(StructureStage, &GroupedStepState),
    ) -> Result<(), String> {
        self.run_structure_with_progress_callback(stop, &mut |stage, kind, state| {
            if kind == StructureProgressKind::StageCompleted {
                on_stage(stage, state);
            }
        })
    }

    /// Runs structure-first with coherent merged progress snapshots. P1 emits
    /// the Init baseline immediately, interval-throttled construction-start
    /// candidates, and one global snapshot per completed lane. Later stages
    /// emit scheduler-round previews. Every callback identifies its milestone
    /// through [`StructureProgressKind`].
    pub fn run_structure_with_progress_callback(
        &mut self,
        stop: &AtomicBool,
        on_progress: &mut dyn FnMut(StructureStage, StructureProgressKind, &GroupedStepState),
    ) -> Result<(), String> {
        self.run_structure_impl(stop, on_progress, None)
    }

    /// Runs structure-first while also forwarding native in-stage solutions
    /// from the uninterrupted `all_items` lane. Size-region lanes never leak
    /// lane-local IDs through this callback; they use merged scheduler rounds.
    pub fn run_structure_with_callbacks(
        &mut self,
        stop: &AtomicBool,
        on_progress: &mut dyn FnMut(StructureStage, StructureProgressKind, &GroupedStepState),
        on_all_items_solution: &mut dyn FnMut(StructureStage, ReportType, &SPInstance, &SPSolution),
    ) -> Result<(), String> {
        self.run_structure_impl(stop, on_progress, Some(on_all_items_solution))
    }

    fn run_structure_impl(
        &mut self,
        stop: &AtomicBool,
        on_progress: &mut dyn FnMut(StructureStage, StructureProgressKind, &GroupedStepState),
        on_all_items_solution: Option<
            &mut dyn FnMut(StructureStage, ReportType, &SPInstance, &SPSolution),
        >,
    ) -> Result<(), String> {
        self.run_structure_transaction(|state| {
            state.run_structure_impl_inner(stop, on_progress, on_all_items_solution)
        })
    }

    fn run_structure_impl_inner(
        &mut self,
        stop: &AtomicBool,
        on_progress: &mut dyn FnMut(StructureStage, StructureProgressKind, &GroupedStepState),
        mut on_all_items_solution: Option<
            &mut dyn FnMut(StructureStage, ReportType, &SPInstance, &SPSolution),
        >,
    ) -> Result<(), String> {
        if !cfg!(feature = "structure-first") {
            return Err(
                "structure-first requested, but the library was built without the structure-first feature"
                    .into(),
            );
        }
        if self.phase != GroupPhase::Init {
            return Err("structure-first is only valid after init".into());
        }
        if !self.structure_enabled {
            return Err("structure-first is disabled in SwarmConfig".into());
        }

        if stop.load(Ordering::Relaxed) {
            self.restore_structure_incumbents();
            return Ok(());
        }
        if let Some((lane_index, error)) = self.lanes.iter().enumerate().find_map(
            |(lane_index, lane)| {
                crate::structure::pipeline::solution_validation_error_with_frozen_ids_and_groups(
                    &lane.instance,
                    &lane.solution,
                    &lane.frozen_ids,
                    &lane.place_groups,
                )
                .map(|error| (lane_index, error))
            },
        ) {
            return Err(format!(
                "grouped Init baseline lane {lane_index} is not exact-feasible: {error:?}"
            ));
        }
        self.export_certified_solution()
            .map_err(|error| format!("grouped Init baseline is not globally exact: {error}"))?;
        self.structure_public_incumbents = Some(
            self.lanes
                .iter()
                .map(|lane| lane.solution.clone())
                .collect(),
        );

        // Every lane owns a complete, exact baseline from Init. Emit it
        // before P0/P1 so callers never wait through all lane constructions
        // without seeing a coherent globally merged layout.
        on_progress(
            StructureStage::P1Construct,
            StructureProgressKind::InitialBaseline,
            self,
        );
        if stop.load(Ordering::Relaxed) {
            self.restore_structure_incumbents();
            return Ok(());
        }

        if !self.lanes.is_empty() {
            let workers = self.effective_group_workers;
            let lane_count = self.lanes.len();
            let p1_started = Instant::now();
            // P0 and every lane's P1 construction starts share one phase
            // deadline.  In particular, adding lanes must not multiply the
            // configured P1 wall-clock budget.
            let p1_budget = self.lanes[0].config.structure.sanitized().p1_time_limit;
            let p1_deadline = p1_started + p1_budget;
            let pool = rayon::ThreadPoolBuilder::new()
                .num_threads(workers)
                .build()
                .map_err(|e| format!("failed to build structure P1 lane pool: {e}"))?;
            #[cfg(test)]
            self.observed_group_pool_workers.push(pool.current_num_threads());

            // Motif generation is cooperatively cancellable but not resumable,
            // so it is the preparation work unit.  Construction is resumable
            // between independent starts: each scheduler round gives every
            // unfinished lane at most one start, and the pool limits the
            // number of concurrently running lanes to the captured worker budget.
            let run_control = swarm_core::config::runtime::current_control();
            let session_results = pool.install(|| {
                self.lanes
                    .par_iter_mut()
                    .map(|lane| {
                        let _scope = swarm_core::config::runtime::enter(run_control.clone());
                        lane.begin_structure_p1(stop, p1_deadline)
                    })
                    .collect::<Vec<_>>()
            });
            if stop.load(Ordering::Relaxed) {
                self.restore_structure_incumbents();
                return Ok(());
            }
            let mut sessions = session_results
                .into_iter()
                .collect::<Result<Vec<_>, _>>()?
                .into_iter()
                .map(Some)
                .collect::<Vec<_>>();
            let mut completed = vec![false; lane_count];
            let mut completed_lanes = 0;

            while completed_lanes < lane_count {
                if stop.load(Ordering::Relaxed) {
                    self.restore_structure_incumbents();
                    return Ok(());
                }
                let work_results = pool.install(|| {
                    self.lanes
                        .par_iter_mut()
                        .zip(sessions.par_iter_mut())
                        .zip(completed.par_iter())
                        .map(|((lane, session), is_completed)| {
                            let _scope = swarm_core::config::runtime::enter(run_control.clone());
                            if *is_completed {
                                return Ok(None);
                            }
                            lane.advance_structure_p1(
                                session
                                    .as_mut()
                                    .expect(swarm_core::dmsg!("unfinished P1 lane must own a session")),
                                stop,
                                p1_deadline,
                            )
                            .map(Some)
                        })
                        .collect::<Vec<_>>()
                });
                if stop.load(Ordering::Relaxed) {
                    self.restore_structure_incumbents();
                    return Ok(());
                }

                for (lane_index, work_result) in work_results.into_iter().enumerate() {
                    if stop.load(Ordering::Relaxed) {
                        self.restore_structure_incumbents();
                        return Ok(());
                    }
                    let Some(work_result) = work_result? else {
                        continue;
                    };
                    if let Some(candidate) = work_result.candidate_preview {
                        self.lanes[lane_index].solution = candidate;
                        on_progress(
                            StructureStage::P1Construct,
                            StructureProgressKind::CandidatePreview,
                            self,
                        );
                        // A preview is public transient state only. Restore the
                        // lane's exact incumbent immediately so another lane's
                        // completion callback cannot accidentally merge a
                        // wider preview from this unfinished lane.
                        self.lanes[lane_index].solution = sessions[lane_index]
                            .as_ref()
                            .expect(swarm_core::dmsg!("previewing P1 lane must own a session"))
                            .incumbent
                            .clone();
                        if stop.load(Ordering::Relaxed) {
                            self.restore_structure_incumbents();
                            return Ok(());
                        }
                    }
                    if work_result.finished {
                        let session = sessions[lane_index]
                            .take()
                            .expect(swarm_core::dmsg!("finishing P1 lane must own a session"));
                        self.lanes[lane_index].finish_structure_p1(
                            session,
                            stop,
                            p1_started,
                            p1_deadline,
                        )?;
                        completed[lane_index] = true;
                        completed_lanes += 1;
                        // The final lane is represented by the mandatory P1
                        // stage boundary callback below.
                        if completed_lanes < lane_count {
                            on_progress(
                                StructureStage::P1Construct,
                                StructureProgressKind::LaneCompleted,
                                self,
                            );
                            if stop.load(Ordering::Relaxed) {
                                self.restore_structure_incumbents();
                                return Ok(());
                            }
                        }
                    }
                }
            }
        }
        if stop.load(Ordering::Relaxed) {
            self.restore_structure_incumbents();
            return Ok(());
        }
        if !self.certify_structure_public_state() {
            log::warn!("[STRUCT] rolled P1 back to the last globally certified lane set");
        }
        on_progress(
            StructureStage::P1Construct,
            StructureProgressKind::StageCompleted,
            self,
        );
        if stop.load(Ordering::Relaxed) {
            self.restore_structure_incumbents();
            return Ok(());
        }

        {
            let mut on_round = |state: &GroupedStepState| {
                on_progress(
                    StructureStage::P2Compress,
                    StructureProgressKind::SchedulerRound,
                    state,
                );
            };
            if let Some(callback) = on_all_items_solution.as_mut() {
                let mut on_solution = |report, instance: &SPInstance, solution: &SPSolution| {
                    (**callback)(StructureStage::P2Compress, report, instance, solution);
                };
                self.run_scheduled_with_callbacks(
                    stop,
                    LanePhase::StructureCompression,
                    &mut on_round,
                    Some(&mut on_solution),
                )?;
            } else {
                self.run_scheduled_with_callbacks(
                    stop,
                    LanePhase::StructureCompression,
                    &mut on_round,
                    None,
                )?;
            }
        }
        if stop.load(Ordering::Relaxed) {
            self.restore_structure_incumbents();
            return Ok(());
        }
        self.finish_structure_stage(StructureStage::P2Compress);
        if !self.certify_structure_public_state() {
            log::warn!("[STRUCT] rolled P2 back to the last globally certified lane set");
        }
        on_progress(
            StructureStage::P2Compress,
            StructureProgressKind::StageCompleted,
            self,
        );
        if stop.load(Ordering::Relaxed) {
            self.restore_structure_incumbents();
            return Ok(());
        }

        for lane in &mut self.lanes {
            lane.prepare_structure_slack();
        }
        {
            let mut on_round = |state: &GroupedStepState| {
                on_progress(
                    StructureStage::P3Explore,
                    StructureProgressKind::SchedulerRound,
                    state,
                );
            };
            if let Some(callback) = on_all_items_solution.as_mut() {
                let mut on_solution = |report, instance: &SPInstance, solution: &SPSolution| {
                    (**callback)(StructureStage::P3Explore, report, instance, solution);
                };
                self.run_scheduled_with_callbacks(
                    stop,
                    LanePhase::StructureExploration,
                    &mut on_round,
                    Some(&mut on_solution),
                )?;
            } else {
                self.run_scheduled_with_callbacks(
                    stop,
                    LanePhase::StructureExploration,
                    &mut on_round,
                    None,
                )?;
            }
        }
        if stop.load(Ordering::Relaxed) {
            self.restore_structure_incumbents();
            return Ok(());
        }
        self.finish_structure_stage(StructureStage::P3Explore);
        if !self.certify_structure_public_state() {
            log::warn!("[STRUCT] rolled P3 back to the last globally certified lane set");
        }
        on_progress(
            StructureStage::P3Explore,
            StructureProgressKind::StageCompleted,
            self,
        );
        if stop.load(Ordering::Relaxed) {
            self.restore_structure_incumbents();
            return Ok(());
        }

        let p4_config = self
            .lanes
            .first()
            .map(|lane| lane.config.structure.sanitized());
        if self
            .lanes
            .iter()
            .any(RegionLane::supports_structure_gap_repair)
            && let Some(p4_config) = p4_config
        {
            let mut stagnant = 0usize;
            for _ in 0..p4_config.p4_cycles {
                if stagnant >= p4_config.p4_stagnation_cycles {
                    break;
                }
                if stop.load(Ordering::Relaxed) {
                    break;
                }
                let widths_before = self
                    .lanes
                    .iter()
                    .map(|lane| {
                        lane.structure_incumbent
                            .as_ref()
                            .map_or(f32::INFINITY, SPSolution::strip_width)
                    })
                    .collect::<Vec<_>>();
                let mut on_round = |state: &GroupedStepState| {
                    on_progress(
                        StructureStage::P4Refine,
                        StructureProgressKind::SchedulerRound,
                        state,
                    );
                };
                if let Some(callback) = on_all_items_solution.as_mut() {
                    let mut on_solution = |report, instance: &SPInstance, solution: &SPSolution| {
                        (**callback)(StructureStage::P4Refine, report, instance, solution);
                    };
                    self.run_scheduled_with_callbacks(
                        stop,
                        LanePhase::StructureGapRepair,
                        &mut on_round,
                        Some(&mut on_solution),
                    )?;
                } else {
                    self.run_scheduled_with_callbacks(
                        stop,
                        LanePhase::StructureGapRepair,
                        &mut on_round,
                        None,
                    )?;
                }
                if stop.load(Ordering::Relaxed) {
                    break;
                }
                let globally_exact = self.certify_structure_public_state();
                if !globally_exact {
                    log::warn!(
                        "[STRUCT] rolled a grouped P4 cycle back to the last globally certified lane set"
                    );
                }
                let improved = globally_exact
                    && self.lanes.iter().zip(widths_before).any(|(lane, before)| {
                        lane.structure_incumbent
                            .as_ref()
                            .is_some_and(|incumbent| incumbent.strip_width() + 1.0e-5 < before)
                    });
                if improved {
                    stagnant = 0;
                } else {
                    stagnant += 1;
                }
            }
        }
        if stop.load(Ordering::Relaxed) {
            self.restore_structure_incumbents();
            return Ok(());
        }
        self.finish_structure_stage(StructureStage::P4Refine);
        if !self.certify_structure_public_state() {
            log::warn!("[STRUCT] rolled P4 back to the last globally certified lane set");
        }
        on_progress(
            StructureStage::P4Refine,
            StructureProgressKind::StageCompleted,
            self,
        );
        if stop.load(Ordering::Relaxed) {
            self.restore_structure_incumbents();
            return Ok(());
        }

        {
            let mut on_round = |state: &GroupedStepState| {
                on_progress(
                    StructureStage::P5Finish,
                    StructureProgressKind::SchedulerRound,
                    state,
                );
            };
            if let Some(callback) = on_all_items_solution.as_mut() {
                let mut on_solution = |report, instance: &SPInstance, solution: &SPSolution| {
                    (**callback)(StructureStage::P5Finish, report, instance, solution);
                };
                self.run_scheduled_with_callbacks(
                    stop,
                    LanePhase::StructureFinish,
                    &mut on_round,
                    Some(&mut on_solution),
                )?;
            } else {
                self.run_scheduled_with_callbacks(
                    stop,
                    LanePhase::StructureFinish,
                    &mut on_round,
                    None,
                )?;
            }
        }
        if stop.load(Ordering::Relaxed) {
            self.restore_structure_incumbents();
            return Ok(());
        }
        self.finish_structure_stage(StructureStage::P5Finish);
        if !self.certify_structure_public_state() {
            log::warn!("[STRUCT] rolled P5 back to the last globally certified lane set");
        }
        for lane in &mut self.lanes {
            if let Some(report) = lane.structure_report.as_mut() {
                report.cancelled = false;
            }
        }
        on_progress(
            StructureStage::P5Finish,
            StructureProgressKind::StageCompleted,
            self,
        );
        self.export_certified_solution()?;
        self.phase = GroupPhase::Structure;
        Ok(())
    }

    /// Returns a self-contained lane audit after [`Self::run_structure`].
    /// The aggregate deliberately reports required region/fixed-prefix spacing
    /// separately from the per-lane algorithmic diagnostics.
    pub fn structure_report_json(&self) -> Option<String> {
        let lanes = self
            .lanes
            .iter()
            .enumerate()
            .map(|(lane_index, lane)| {
                Some(GroupedLaneStructureReport {
                    schema_version: 3,
                    lane_index,
                    global_item_ids: lane.local_to_global.clone(),
                    place_group_count: lane.place_groups.len(),
                    frozen_item_count: lane.frozen_ids.len(),
                    p0_p1_soft_structure_applied: lane.structure_fallback.is_none(),
                    fallback_reason: lane.structure_fallback.clone(),
                    report: lane.structure_report.clone()?,
                })
            })
            .collect::<Option<Vec<_>>>()?;
        let solution = self.export_solution();
        let required_region_gap_mm = self
            .lanes
            .iter()
            .take(self.lanes.len().saturating_sub(1))
            .map(|lane| lane.gap_after_mm)
            .sum();
        let report = GroupedStructureReport {
            schema_version: 3,
            nesting_mode: match self.mode {
                NestingMode::AllItems => "all_items",
                NestingMode::SizeRegions => "size_regions",
            },
            group_workers: self.group_workers,
            effective_group_workers: self.effective_group_workers,
            quantum_ms: self.quantum.as_millis().min(u64::MAX as u128) as u64,
            fixed_prefix_item_count: self.fixed_placements.len(),
            fixed_prefix_right_mm: self.fixed_right,
            locked_prefix_gap_mm: self.locked_prefix_gap_mm,
            required_region_gap_mm,
            lane_count: lanes.len(),
            lanes,
            final_strip_width: solution.strip_width,
            final_density: solution.density,
        };
        serde_json::to_string(&report).ok()
    }

    fn finish_structure_stage(&mut self, stage: StructureStage) {
        for lane in &mut self.lanes {
            lane.finish_structure_stage(stage);
        }
    }

    fn run_scheduled_with_callbacks(
        &mut self,
        stop: &AtomicBool,
        phase: LanePhase,
        on_round: &mut dyn FnMut(&GroupedStepState),
        on_all_items_solution: Option<&mut dyn FnMut(ReportType, &SPInstance, &SPSolution)>,
    ) -> Result<(), String> {
        if self.lanes.is_empty() || stop.load(Ordering::Relaxed) {
            return Ok(());
        }
        let budget = self.phase_budget(phase);
        if budget.is_zero() {
            return Ok(());
        }

        // all_items is the compatibility branch: one uninterrupted call and
        // the caller-provided internal Separator worker count are retained.
        if self.mode == NestingMode::AllItems {
            let term = SliceTerminator::new(stop, budget);
            let lane = &mut self.lanes[0];
            if let Some(on_solution) = on_all_items_solution {
                // Seed only an exact-feasible incumbent. Invalid movable warm
                // hints are repaired during Init, but this guard keeps the
                // public callback contract safe if a future initializer adds
                // another hinted path.
                if lane.candidate_is_valid(&lane.solution) {
                    let report = match phase {
                        LanePhase::Compression
                        | LanePhase::Polish
                        | LanePhase::StructureCompression
                        | LanePhase::StructureFinish => ReportType::CmprFeas,
                        _ => ReportType::ExplFeas,
                    };
                    on_solution(report, &lane.instance, &lane.solution);
                }
                lane.run_slice_with_progress(phase, &term, budget, on_solution)?;
            } else {
                // A no-op callback is not equivalent to no listener: Jagua
                // snapshots every reported candidate when a listener exists.
                // Preserve the legacy no-callback performance path.
                let mut listener = None;
                lane.run_slice_with_listener(phase, &term, budget, &mut listener)?;
            }
            return Ok(());
        }

        let deadline = Instant::now() + budget;
        let lane_count = self.lanes.len();
        let workers = self.effective_group_workers;
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(workers)
            .build()
            .map_err(|e| format!("failed to build group lane pool: {e}"))?;
        #[cfg(test)]
        self.observed_group_pool_workers.push(pool.current_num_threads());

        let run_control = swarm_core::config::runtime::current_control();

        // A P4 cycle is one ruin/recreate + separation/slide + resumed
        // Explore pipeline under one shared deadline. Running it once per
        // scheduler quantum would accidentally multiply repair cycles and
        // reset the intended same-cycle search state.
        //
        // P5 Finish with no active polish pass has nothing to iterate on:
        // every slice would only re-run the rotation snap and return at once,
        // busy-looping until the deadline. Run it as a single wave as well.
        let finish_without_passes = phase == LanePhase::StructureFinish
            && self.lanes.iter().all(|lane| {
                !crate::optimizer::polish::has_active_passes(
                    &lane.config.polish_cfg,
                    !lane.place_groups.is_empty(),
                )
            });
        if phase == LanePhase::StructureGapRepair || finish_without_passes {
            // One cycle per lane, with time reserved for each queued worker
            // wave. A first wave must not consume the whole shared deadline.
            // Preserve the existing WASM scheduling policy.
            let lane_slice = if cfg!(target_arch = "wasm32") {
                budget
            } else {
                fair_lane_slice(budget, lane_count, workers)
            };
            let errors = pool.install(|| {
                self.lanes
                    .par_iter_mut()
                    .map(|lane| {
                        let _scope = swarm_core::config::runtime::enter(run_control.clone());
                        let term = SliceTerminator::new_capped(stop, lane_slice, deadline);
                        lane.run_slice(phase, &term, lane_slice)
                    })
                    .collect::<Vec<_>>()
            });
            if let Some(error) = errors.into_iter().find_map(Result::err) {
                return Err(error);
            }
            on_round(self);
            return Ok(());
        }

        while !stop.load(Ordering::Relaxed) && Instant::now() < deadline {
            let remaining = deadline
                .checked_duration_since(Instant::now())
                .unwrap_or(Duration::ZERO);
            if remaining.is_zero() {
                break;
            }
            let slice = self.quantum.min(remaining);
            // The configured scheduler quantum defines the fair base slice,
            // not a fresh allowance for every queued lane. Split that base
            // across worker waves; per-lane adaptation may grow it later, but
            // still reserves every wave's share of the global phase deadline.
            let base_lane_slice = fair_lane_slice(slice, lane_count, workers);
            let effective_slices = self
                .lanes
                .iter_mut()
                .map(|lane| {
                    adaptive_lane_slice(
                        base_lane_slice,
                        lane.slice_adaptation_level(phase),
                        self.quantum,
                        remaining,
                        lane_count,
                        workers,
                    )
                })
                .collect::<Vec<_>>();
            let outcomes = pool.install(|| {
                self.lanes
                    .par_iter_mut()
                    .zip(effective_slices.into_par_iter())
                    .map(|(lane, effective_slice)| {
                        let _scope = swarm_core::config::runtime::enter(run_control.clone());
                        let term = SliceTerminator::new_capped(stop, effective_slice, deadline);
                        lane.run_slice_with_outcome(phase, &term, effective_slice)
                    })
                    .collect::<Vec<_>>()
            });
            for (lane_index, outcome) in outcomes.into_iter().enumerate() {
                self.lanes[lane_index].observe_slice_outcome(phase, outcome?);
            }
            on_round(self);
        }
        Ok(())
    }

    fn phase_budget(&self, phase: LanePhase) -> Duration {
        let Some(lane) = self.lanes.first() else {
            return Duration::ZERO;
        };
        match phase {
            LanePhase::Exploration => lane.config.expl_cfg.time_limit,
            LanePhase::Compression => lane.config.cmpr_cfg.time_limit,
            LanePhase::Polish => lane.config.polish_cfg.time_limit,
            LanePhase::StructureCompression => lane.config.structure.p2_time_limit,
            LanePhase::StructureExploration => lane
                .config
                .structure
                .p3_time_limit
                .unwrap_or(lane.config.expl_cfg.time_limit),
            LanePhase::StructureGapRepair => lane.config.structure.p4_cycle_time_limit,
            LanePhase::StructureFinish => lane.config.polish_cfg.time_limit,
        }
    }

    pub fn export_solution(&self) -> ExtSPSolution {
        let mut placed_items = self.fixed_placements.clone();
        let has_movable = !self.lanes.is_empty();
        let mut offset = if self.fixed_placements.is_empty() {
            0.0
        } else if has_movable {
            self.fixed_right + self.locked_prefix_gap_mm
        } else {
            self.fixed_right
        };

        for (index, lane) in self.lanes.iter().enumerate() {
            let ext =
                jagua_rs::probs::spp::io::export(&lane.solution, *crate::EPOCH);
            for mut placement in ext.layout.placed_items {
                placement.item_id = lane.local_to_global[placement.item_id as usize];
                placement.transformation.translation.0 += offset;
                placed_items.push(placement);
            }
            offset += lane.solution.strip_width();
            if index + 1 < self.lanes.len() {
                offset += lane.gap_after_mm;
            }
        }
        placed_items.sort_by_key(|placement| placement.item_id);

        let strip_width = if self.lanes.is_empty() {
            // All locked pieces may be left of x=0. Keep the original positive
            // width in that case; export/import must never create a zero strip.
            if self.fixed_right > 0.0 {
                self.fixed_right
            } else {
                self.full_instance.base_strip.width
            }
        } else {
            offset
        };
        let density = if strip_width > 0.0 {
            self.full_instance.item_area()
                / (self.full_instance.base_strip.fixed_height * strip_width)
        } else {
            0.0
        };
        ExtSPSolution {
            strip_width,
            layout: ExtLayout {
                container_id: 0,
                placed_items,
                density,
            },
            density,
            run_time_sec: self.started_at.elapsed().as_secs(),
        }
    }

    pub fn restored_problem(&self) -> SPProblem {
        let ext = self.export_solution();
        self.restored_problem_from_exported(&ext)
    }

    /// Restore a merged solution that the caller already exported. Callback
    /// reporting uses this to avoid repeating lane merge, ID remap and sort.
    pub fn restored_problem_from_exported(&self, ext: &ExtSPSolution) -> SPProblem {
        crate::util::frozen::import_solution(&self.full_instance, ext)
    }

    pub fn frozen_ids(&self) -> &[usize] {
        &self.frozen_global_ids
    }

    pub fn place_groups(&self) -> &PlaceGroups {
        &self.full_place_groups
    }

    pub fn full_instance(&self) -> &SPInstance {
        &self.full_instance
    }

    /// Public ids behind the engine's compact ids (multipolygon place groups).
    pub fn expansion(&self) -> Option<&std::sync::Arc<GroupExpansion>> {
        self.expansion.as_ref()
    }

    /// Expands a compact export to the public item ids and member poses.
    pub fn expand_public(&self, ext: ExtSPSolution) -> ExtSPSolution {
        match &self.expansion {
            Some(expansion) => expansion.expand_solution(ext),
            None => ext,
        }
    }

    /// [`Self::export_solution`] in public ids.
    pub fn export_public_solution(&self) -> ExtSPSolution {
        self.expand_public(self.export_solution())
    }

    /// [`Self::export_certified_solution`] in public ids.
    pub fn export_certified_public_solution(&self) -> Result<ExtSPSolution, String> {
        self.export_certified_solution()
            .map(|ext| self.expand_public(ext))
    }

    /// Expands per-item flags (collision status) to public ids.
    pub fn expand_flags(&self, flags: Vec<(usize, bool)>) -> Vec<(usize, bool)> {
        match &self.expansion {
            Some(expansion) => expansion.expand_flags(flags),
            None => flags,
        }
    }

    #[cfg(test)]
    pub(crate) fn corrupt_first_lane_public_transform_for_test(&mut self) -> bool {
        let Some(lane) = self.lanes.first_mut() else {
            return false;
        };
        let mut placements = lane.solution.layout_snapshot().placed_items().iter();
        let Some((_, first)) = placements.next() else {
            return false;
        };
        let first_transform = first.d_transf();
        let Some(second_key) = placements.next().map(|(key, _)| key) else {
            return false;
        };
        lane.solution.layout_snapshot_mut().placed_items_mut()[second_key].set_d_transf(first_transform);
        true
    }

    #[cfg(test)]
    pub(crate) fn panic_next_legacy_phase_for_test(&mut self) {
        self.panic_next_legacy_phase = true;
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LanePhase {
    Exploration,
    Compression,
    Polish,
    StructureCompression,
    StructureExploration,
    StructureGapRepair,
    StructureFinish,
}

#[derive(Clone)]
struct LaneTrackerSession {
    phase: LanePhase,
    tracker: CollisionTracker,
}

#[derive(Clone, Copy, Debug)]
struct LaneSliceAdaptation {
    phase: LanePhase,
    level: u32,
}

impl LaneSliceAdaptation {
    fn new(phase: LanePhase) -> Self {
        Self { phase, level: 0 }
    }

    /// The learned level intentionally survives improvement.
    fn observe(&mut self, outcome: LaneSliceOutcome) {
        if !outcome.stop_requested
            && outcome.timed_out
            && exact_width_unchanged(outcome.before_width, outcome.after_width)
        {
            self.level = self.level.saturating_add(1);
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct LaneSliceOutcome {
    before_width: f32,
    after_width: f32,
    timed_out: bool,
    stop_requested: bool,
}

#[cfg(test)]
std::thread_local! {
    static ISSUE7_SETUP_CALLS: std::cell::Cell<[usize; 3]> =
        const { std::cell::Cell::new([0; 3]) };
}

#[cfg(test)]
fn issue7_record_setup_call(index: usize) {
    ISSUE7_SETUP_CALLS.with(|cell| {
        let mut counts = cell.get();
        counts[index] += 1;
        cell.set(counts);
    });
}

impl RegionLane {
    fn constraints_valid(&self, solution: &SPSolution) -> bool {
        if self.frozen_reference.is_empty() {
            return self
                .place_groups
                .solution_preserves_relative_poses(solution);
        }
        let mut matched = 0;
        for pi in solution.layout_snapshot().placed_items().values() {
            if let Some(expected) = self.frozen_reference.get(&pi.item().idx()) {
                if pi.d_transf() != *expected {
                    return false;
                }
                matched += 1;
            }
        }
        matched == self.frozen_reference.len()
            && self
                .place_groups
                .solution_preserves_relative_poses(solution)
    }

    fn candidate_is_valid(&self, solution: &SPSolution) -> bool {
        self.constraints_valid(solution)
            && crate::structure::pipeline::solution_validation_error_with_frozen_ids_and_groups(
                &self.instance,
                solution,
                &self.frozen_ids,
                &self.place_groups,
            )
            .is_none()
    }

    /// The narrowest exact candidate, ties resolved to the earliest one.
    /// Candidates are certified in width order, stopping at the first exact one.
    fn narrowest_valid<'a>(&self, solutions: &'a [SPSolution]) -> Option<&'a SPSolution> {
        let mut order = (0..solutions.len()).collect::<Vec<_>>();
        order.sort_by(|&a, &b| {
            solutions[a]
                .strip_width()
                .total_cmp(&solutions[b].strip_width())
        });
        order
            .into_iter()
            .map(|index| &solutions[index])
            .find(|solution| self.candidate_is_valid(solution))
    }

    fn supports_structure_gap_repair(&self) -> bool {
        self.place_groups.is_empty() && self.instance.total_item_qty() >= 4
    }

    fn run_slice(
        &mut self,
        phase: LanePhase,
        term: &impl Terminator,
        slice: Duration,
    ) -> Result<(), String> {
        self.run_slice_with_listener(phase, term, slice, &mut None)
    }

    fn run_slice_with_outcome(
        &mut self,
        phase: LanePhase,
        term: &impl Terminator,
        slice: Duration,
    ) -> Result<LaneSliceOutcome, String> {
        let before_width = self.solution.strip_width();
        self.run_slice(phase, term, slice)?;
        let stop_requested = term.stop_requested();
        let timed_out = !stop_requested
            && term
                .timeout_at()
                .is_some_and(|deadline| Instant::now() >= deadline);
        Ok(LaneSliceOutcome {
            before_width,
            after_width: self.solution.strip_width(),
            timed_out,
            stop_requested,
        })
    }

    fn run_slice_with_progress(
        &mut self,
        phase: LanePhase,
        term: &impl Terminator,
        slice: Duration,
        on_solution: &mut dyn FnMut(ReportType, &SPInstance, &SPSolution),
    ) -> Result<(), String> {
        let mut forwarder = LaneSolutionForwarder {
            callback: on_solution,
        };
        let mut listener: Option<&mut dyn SolutionListener> = Some(&mut forwarder);
        self.run_slice_with_listener(phase, term, slice, &mut listener)
    }

    fn run_slice_with_listener(
        &mut self,
        phase: LanePhase,
        term: &impl Terminator,
        slice: Duration,
        listener: &mut Option<&mut dyn SolutionListener>,
    ) -> Result<(), String> {
        // A queued lane may expire while another wave runs. Avoid resetting its
        // GLS session or allocating a Separator after Stop/the shared deadline.
        if term.kill() {
            return Ok(());
        }
        self.begin_tracker_phase(phase);
        match phase {
            LanePhase::Exploration => self.explore_slice(term, slice, listener),
            LanePhase::Compression => self.compress_slice(term, slice, listener),
            LanePhase::Polish => self.polish_slice(term, slice, listener),
            LanePhase::StructureCompression => self.structure_compress_slice(term, slice, listener),
            LanePhase::StructureExploration => self.structure_explore_slice(term, slice, listener),
            LanePhase::StructureGapRepair => self.structure_gap_repair_slice(term, slice, listener),
            LanePhase::StructureFinish => self.structure_finish_slice(term, slice, listener),
        }
    }

    fn begin_tracker_phase(&mut self, phase: LanePhase) {
        if self.force_single_worker
            && self
                .tracker_session
                .as_ref()
                .is_some_and(|session| session.phase != phase)
        {
            self.tracker_session = None;
        }
    }

    fn slice_adaptation_level(&mut self, phase: LanePhase) -> u32 {
        if self
            .slice_adaptation
            .as_ref()
            .is_none_or(|adaptation| adaptation.phase != phase)
        {
            self.slice_adaptation = Some(LaneSliceAdaptation::new(phase));
        }
        self.slice_adaptation
            .as_ref()
            .expect(swarm_core::dmsg!("slice adaptation must exist for the active phase"))
            .level
    }

    fn observe_slice_outcome(&mut self, phase: LanePhase, outcome: LaneSliceOutcome) {
        self.slice_adaptation
            .as_mut()
            .filter(|adaptation| adaptation.phase == phase)
            .expect(swarm_core::dmsg!("slice adaptation must be initialized before observing an outcome"))
            .observe(outcome);
    }

    fn remember_tracker(&mut self, phase: LanePhase, tracker: &CollisionTracker) {
        if !self.force_single_worker {
            return;
        }
        debug_assert!(
            self.tracker_session
                .as_ref()
                .is_none_or(|session| session.phase == phase),
            "tracker session must be reset before changing lane phase"
        );
        self.tracker_session = Some(LaneTrackerSession {
            phase,
            tracker: tracker.clone(),
        });
    }

    fn rollback_slice_seed(&self, sep: &mut Separator, seed: &SPSolution) {
        if self.force_single_worker
            && let Some(session) = &self.tracker_session
        {
            // Rollback rebuilds a neutral tracker for the restored layout.
            // Inherit directly from the remembered session instead of cloning
            // the whole pre-rollback tracker (which was itself only a
            // projection of that same session onto the fresh Separator).
            sep.rollback(seed, None);
            sep.ct.inherit_weights_from(&session.tracker);
        } else {
            // Keep the all-items compatibility path byte-for-byte equivalent
            // at the separator boundary (including its neutral GLS reset).
            sep.rollback(seed, None);
        }
    }

    fn separator(&mut self, mut config: crate::optimizer::separator::SeparatorConfig) -> Separator {
        if self.force_single_worker {
            config.n_workers = 1;
        }
        #[cfg(test)]
        issue7_record_setup_call(0);
        let mut sep = Separator::new(
            self.instance.clone(),
            self.prob.clone(),
            next_rng(&mut self.rng),
            config,
        );
        sep.configure_reinsertion(&self.config.nfp_integration);
        // This separator is fresh and neutral; empty settings cannot change
        // its constraints or reset learned weights before donor inheritance.
        if !self.place_groups.is_empty() {
            #[cfg(test)]
            issue7_record_setup_call(1);
            sep.set_place_groups(self.place_groups.clone())
                .expect(swarm_core::dmsg!("validated place groups must fit the lane layout"));
        } else {
            // Retain the lane's exact empty policy identity for certification.
            sep.place_groups = self.place_groups.clone();
        }
        if !self.frozen_ids.is_empty() {
            #[cfg(test)]
            issue7_record_setup_call(2);
            sep.freeze_items(&self.frozen_ids);
        }
        if self.force_single_worker
            && let Some(session) = &self.tracker_session
        {
            sep.ct.inherit_weights_from(&session.tracker);
        }
        sep
    }

    fn begin_structure_p1(
        &mut self,
        stop: &AtomicBool,
        p1_deadline: Instant,
    ) -> Result<P1LaneSession, String> {
        let structure = self.config.structure.sanitized();
        let profile_cfg = structure_profile_config(&self.instance, &structure);
        let mut fallback_reasons = vec![];
        if !self.place_groups.is_empty() {
            fallback_reasons.push(
                "user_place_groups_active: kept rigid group-aware LBF; skipped soft P0/P1, individual P4 repair and P5 angle snap",
            );
        }
        if !self.frozen_ids.is_empty() {
            fallback_reasons.push("frozen_items_active: kept freeze-aware hinted LBF for P0/P1");
        }
        self.structure_fallback =
            (!fallback_reasons.is_empty()).then(|| fallback_reasons.join("; "));

        let incumbent = self.solution.clone();
        let baseline_validation_error =
            crate::structure::pipeline::solution_validation_error_with_frozen_ids_and_groups(
                &self.instance,
                &incumbent,
                &self.frozen_ids,
                &self.place_groups,
            );
        let baseline_exact_valid =
            baseline_validation_error.is_none() && self.constraints_valid(&incumbent);
        let incumbent_exact_valid = baseline_exact_valid;
        let baseline_complete =
            incumbent.layout_snapshot().placed_items().len() == self.instance.total_item_qty();
        if self.structure_fallback.is_some() && !baseline_exact_valid {
            return Err(format!(
                "group/freeze-aware structure baseline is not exact-feasible: {:?}",
                baseline_validation_error.expect(swarm_core::dmsg!("invalid baseline has a reason"))
            ));
        }
        let baseline_source = P1CandidateSource::GroupedLaneBaseline;
        let baseline_winner = P1WinnerRecord {
            source: baseline_source,
            start_index: None,
        };
        let baseline_candidate = P1CandidateRecord {
            source: baseline_source,
            start_index: None,
            elapsed_secs: 0.0,
            complete: baseline_complete,
            exact_valid: baseline_exact_valid,
            validation_error: baseline_validation_error,
            stopped: false,
            width: baseline_exact_valid.then(|| incumbent.strip_width()),
            density: baseline_exact_valid.then(|| incumbent.density()),
            improved_incumbent: baseline_exact_valid,
            selected_as_p1_incumbent: baseline_exact_valid,
            structure_hints: None,
        };

        // A rigid PlaceGroup or a frozen hinted lane has an authoritative
        // group/freeze-aware baseline. Decide that fallback before P0 so the
        // audit cannot claim soft motifs were generated or applied there.
        let (selected_motifs, generation_stats, selection_stats, motif_counts, p0_elapsed_secs) =
            if self.structure_fallback.is_some() {
                (
                    None,
                    MotifGenerationStats::default(),
                    MotifSelectionStats::default(),
                    (0, 0, 0, 0),
                    0.0,
                )
            } else {
                let p0_started = Instant::now();
                let generation = {
                    let mut should_stop =
                        || stop.load(Ordering::Relaxed) || Instant::now() >= p1_deadline;
                    MotifGenerator::new(structure.motif)
                        .generate_with_control_and_stats(&self.instance, &mut should_stop)
                };
                // Selection can still be substantial on large motif catalogs.
                // Do not spend time past the one shared P0/P1 deadline after
                // cooperative generation has already stopped.
                let (selected, selection_stats) =
                    if stop.load(Ordering::Relaxed) || Instant::now() >= p1_deadline {
                        (vec![], MotifSelectionStats::default())
                    } else {
                        let selection = generation
                            .catalog
                            .disjoint_ranked_with_stats(structure.max_active_motifs);
                        (
                            selection.motifs.into_iter().cloned().collect::<Vec<_>>(),
                            selection.stats,
                        )
                    };
                let counts = (
                    generation.catalog.singles.len(),
                    generation.catalog.pairs.len(),
                    generation.catalog.blocks.len(),
                    generation.catalog.bands.len(),
                );
                (
                    Some(selected),
                    generation.stats,
                    selection_stats,
                    counts,
                    p0_started.elapsed().as_secs_f32(),
                )
            };
        let (motif_singles, motif_pairs, motif_blocks, motif_bands) = motif_counts;
        let report = StructureRunReport {
            schema_version: 3,
            cancelled: false,
            p0_elapsed_secs,
            p0_stopped: generation_stats.generation_cancelled,
            motif_generation_stats: generation_stats,
            motif_singles,
            motif_pairs,
            motif_blocks,
            motif_bands,
            motif_fft_pairs: motif_pairs,
            pair_matching_method: if self.structure_fallback.is_some() {
                "none".to_string()
            } else {
                selection_stats.pair_matching_method.as_str().to_string()
            },
            pair_matching_vertices: selection_stats.pair_vertices,
            pair_matching_edges: selection_stats.pair_edges_considered,
            pair_matching_dp_states: selection_stats.pair_dp_states,
            pair_matching_selected_pairs: selection_stats.selected_pairs,
            motif_selection_stats: selection_stats,
            construction_starts_requested: if self.structure_fallback.is_some() {
                0
            } else {
                structure.construction_starts
            },
            construction_starts_attempted: 0,
            construction_starts_completed: 0,
            p1_selected_legacy_baseline: baseline_exact_valid,
            p1_control_candidate: Some(baseline_candidate),
            p1_start_stats: vec![],
            p1_winner: baseline_exact_valid.then_some(baseline_winner),
            p1_deadline_exhausted: false,
            checkpoints: vec![],
            repair_cycles: vec![],
            best_stage: StructureStage::P1Construct,
            final_width: self.solution.strip_width(),
            final_density: self.solution.density(),
            elapsed_secs: 0.0,
        };

        Ok(P1LaneSession {
            selected_motifs,
            profile_cfg,
            construction_starts: structure.construction_starts,
            next_start_index: 0,
            incumbent,
            incumbent_exact_valid,
            report,
            stopped_after_invalid_candidate: false,
        })
    }

    /// Performs at most one independent construction start.  This is the
    /// smallest safe resumable boundary exposed by the LBF constructor: an
    /// individual start cooperatively observes the shared phase deadline,
    /// while lane fairness is enforced between starts by the outer scheduler.
    fn advance_structure_p1(
        &mut self,
        session: &mut P1LaneSession,
        stop: &AtomicBool,
        p1_deadline: Instant,
    ) -> Result<P1LaneWorkResult, String> {
        if session.selected_motifs.is_none()
            || session.stopped_after_invalid_candidate
            || session.next_start_index >= session.construction_starts
            || stop.load(Ordering::Relaxed)
            || Instant::now() >= p1_deadline
        {
            return Ok(P1LaneWorkResult {
                candidate_preview: None,
                finished: true,
            });
        }

        let start_index = session.next_start_index;
        session.next_start_index += 1;
        session.report.construction_starts_attempted += 1;
        let candidate_started = Instant::now();
        let builder = {
            let mut should_stop = || stop.load(Ordering::Relaxed) || Instant::now() >= p1_deadline;
            LBFBuilder::new(
                self.instance.clone(),
                next_rng(&mut self.rng),
                LBF_SAMPLE_CONFIG,
                self.config.lbf_x_multiplier,
                self.config.lbf_y_multiplier,
            )
            .with_order_mode(self.config.lbf_order)
            .with_raster_config(self.config.lbf_raster)
            .with_preselected_structure_hints(
                session.profile_cfg,
                session
                    .selected_motifs
                    .as_ref()
                    .expect(swarm_core::dmsg!("active P1 lane owns selected motifs"))
                    .clone(),
            )
            .construct_with_control(&mut should_stop)
        };
        let structure_hints = builder.structure_hint_stats().cloned();
        let candidate = builder.prob.save();
        let validation_error =
            crate::structure::pipeline::solution_validation_error_with_frozen_ids_and_groups(
                &self.instance,
                &candidate,
                &self.frozen_ids,
                &self.place_groups,
            );
        let exact_valid = validation_error.is_none() && self.constraints_valid(&candidate);
        let complete = structure_hints
            .as_ref()
            .is_none_or(|stats| stats.start.completed);
        let improved = exact_valid
            && (!session.incumbent_exact_valid
                || candidate.strip_width() + 1.0e-5 < session.incumbent.strip_width());
        session.report.p1_start_stats.push(P1CandidateRecord {
            source: P1CandidateSource::Experimental,
            start_index: Some(start_index),
            elapsed_secs: candidate_started.elapsed().as_secs_f32(),
            complete,
            exact_valid,
            validation_error,
            stopped: structure_hints
                .as_ref()
                .is_some_and(|stats| stats.start.stopped),
            width: exact_valid.then(|| candidate.strip_width()),
            density: exact_valid.then(|| candidate.density()),
            improved_incumbent: improved,
            selected_as_p1_incumbent: false,
            structure_hints,
        });
        if exact_valid && complete {
            session.report.construction_starts_completed += 1;
        }

        // A preview may be wider than the lane incumbent. It is a complete
        // exact-feasible construction shown only as P1 progress; final
        // selection remains width-monotone below.
        let candidate_preview = (exact_valid && complete).then(|| candidate.clone());
        if improved {
            session.incumbent = candidate;
            session.incumbent_exact_valid = true;
            session.report.p1_selected_legacy_baseline = false;
            session.report.p1_winner = Some(P1WinnerRecord {
                source: P1CandidateSource::Experimental,
                start_index: Some(start_index),
            });
        }
        if !exact_valid {
            // Preserve the old lane-local behaviour: one invalid construction
            // aborts the remaining starts for that lane, but other lanes keep
            // their fair turns while the shared phase budget remains.
            session.stopped_after_invalid_candidate = true;
        }

        Ok(P1LaneWorkResult {
            candidate_preview,
            finished: session.stopped_after_invalid_candidate
                || session.next_start_index >= session.construction_starts
                || stop.load(Ordering::Relaxed)
                || Instant::now() >= p1_deadline,
        })
    }

    fn finish_structure_p1(
        &mut self,
        mut session: P1LaneSession,
        _stop: &AtomicBool,
        p1_started: Instant,
        p1_deadline: Instant,
    ) -> Result<(), String> {
        session.report.p1_deadline_exhausted = Instant::now() >= p1_deadline;
        let Some(winner) = session.report.p1_winner else {
            return Err("grouped structure P1 produced no exact-feasible candidate".to_string());
        };
        if let Some(control) = session.report.p1_control_candidate.as_mut() {
            control.selected_as_p1_incumbent = winner.source == control.source;
        }
        for start in &mut session.report.p1_start_stats {
            start.selected_as_p1_incumbent =
                winner.source == start.source && winner.start_index == start.start_index;
        }

        self.prob = restored_lane_problem(&self.instance, &session.incumbent);
        self.solution = session.incumbent.clone();
        self.structure_incumbent = Some(session.incumbent);
        self.structure_started_at = Some(p1_started);
        self.structure_report = Some(session.report);
        self.push_structure_checkpoint(StructureStage::P1Construct);
        Ok(())
    }

    fn prepare_structure_slack(&mut self) {
        self.structure_p3_repair_pending = false;
        let Some(incumbent) = self.structure_incumbent.clone() else {
            return;
        };
        self.prob = restored_lane_problem(&self.instance, &incumbent);
        let slack_width =
            incumbent.strip_width() * (1.0 + self.config.structure.sanitized().p3_slack_ratio);
        if slack_width > self.prob.strip_width() {
            let mut sep = self.separator(self.config.expl_cfg.separator_config);
            sep.change_strip_width(slack_width, None);
            self.solution = sep.prob.save();
            self.prob = sep.prob;
            self.structure_p3_repair_pending = self.place_groups.is_empty();
        } else {
            self.solution = incumbent;
        }
    }

    fn finish_structure_stage(&mut self, stage: StructureStage) {
        let Some(incumbent) = self.structure_incumbent.clone() else {
            return;
        };
        self.solution = incumbent.clone();
        self.prob = restored_lane_problem(&self.instance, &incumbent);
        self.push_structure_checkpoint(stage);
    }

    fn push_structure_checkpoint(&mut self, stage: StructureStage) {
        let Some(solution) = self.structure_incumbent.as_ref() else {
            return;
        };
        let elapsed = self
            .structure_started_at
            .map_or(0.0, |started| started.elapsed().as_secs_f32());
        let Some(report) = self.structure_report.as_mut() else {
            return;
        };
        report.checkpoints.push(StageCheckpoint {
            stage,
            elapsed_secs: elapsed,
            width: solution.strip_width(),
            density: solution.density(),
            is_global_incumbent: true,
            incumbent_width: solution.strip_width(),
            incumbent_density: solution.density(),
        });
        report.final_width = solution.strip_width();
        report.final_density = solution.density();
        report.elapsed_secs = elapsed;
    }

    /// Installs only a complete exact candidate as the lane's public progress
    /// state. Invalid native snapshots fall back to the last exact incumbent,
    /// so size-region scheduler callbacks never merge an unverified lane.
    fn apply_structure_candidate(
        &mut self,
        candidate: SPSolution,
        stage: StructureStage,
    ) -> (bool, bool) {
        let exact = self.candidate_is_valid(&candidate);
        let mut improved = false;
        if exact && let Some(incumbent) = self.structure_incumbent.as_mut() {
            improved = candidate.strip_width() + 1.0e-5 < incumbent.strip_width();
            let compact_progress = stage == StructureStage::P5Finish
                && prefers_compact_solution(&candidate, incumbent);
            if improved || compact_progress {
                *incumbent = candidate.clone();
                if improved && let Some(report) = self.structure_report.as_mut() {
                    report.best_stage = stage;
                }
            }
        }
        self.solution = if exact {
            candidate
        } else {
            self.structure_incumbent
                .clone()
                .expect(swarm_core::dmsg!("Structure lane must retain an exact incumbent"))
        };
        self.prob = restored_lane_problem(&self.instance, &self.solution);
        (exact, improved)
    }

    fn structure_compress_slice(
        &mut self,
        term: &impl Terminator,
        slice: Duration,
        sol_listener: &mut Option<&mut dyn SolutionListener>,
    ) -> Result<(), String> {
        let mut config: CompressionConfig = self.config.cmpr_cfg;
        let structure = self.config.structure.sanitized();
        config.time_limit = slice;
        config.shrink_range = structure.p2_shrink_range;
        config.shrink_decay =
            crate::config::ShrinkDecayStrategy::FailureBased(structure.p2_failure_decay);
        config.separator_config.sample_config.tilt_range = 0.0;
        if self.force_single_worker {
            config.separator_config.n_workers = 1;
        }
        let seed = self.solution.clone();
        let mut sep = self.separator(config.separator_config);
        let candidate =
            compression_phase(&self.instance, &mut sep, &seed, sol_listener, term, &config);
        self.remember_tracker(LanePhase::StructureCompression, &sep.ct);
        self.apply_structure_candidate(candidate, StructureStage::P2Compress);
        Ok(())
    }

    fn structure_explore_slice(
        &mut self,
        term: &impl Terminator,
        slice: Duration,
        sol_listener: &mut Option<&mut dyn SolutionListener>,
    ) -> Result<(), String> {
        let structure = self.config.structure.sanitized();
        let mut config: ExplorationConfig = self.config.expl_cfg;
        config.time_limit = slice;
        config.separator_config.sample_config.tilt_range = 0.0;
        if self.force_single_worker {
            config.separator_config.n_workers = 1;
        }
        let seed = self.solution.clone();
        let inject_gap_target = self.structure_p3_repair_pending;
        self.structure_p3_repair_pending = false;
        let mut sep = self.separator(config.separator_config);
        self.rollback_slice_seed(&mut sep, &seed);

        if inject_gap_target && !term.kill() {
            let rnr_cfg = crate::config::RuinRecreateConfig {
                period: 1,
                band_fraction: structure.p4_ruin_fraction,
                max_items: structure.p4_max_items,
                ..swarm_core::config::DEFAULT_SWARM_CONFIG.expl_cfg.rnr
            };
            let mut slack_tabu = BandTabu::new();
            ruin_and_recreate_profile_cancellable(
                &mut sep,
                &rnr_cfg,
                RuinStrategy::GapTarget,
                &mut slack_tabu,
                structure_profile_config(&self.instance, &structure),
                term,
            );
        }

        #[cfg(not(feature = "adaptive"))]
        let solutions = crate::optimizer::explore::exploration_phase(
            &self.instance,
            &mut sep,
            sol_listener,
            term,
            &config,
        );
        #[cfg(feature = "adaptive")]
        let solutions = crate::optimizer::explore_mods::exploration_phase_adaptive(
            &self.instance,
            &mut sep,
            sol_listener,
            term,
            &config,
        );
        let candidate = solutions
            .iter()
            .min_by(|a, b| a.strip_width().total_cmp(&b.strip_width()))
            .cloned()
            .unwrap_or(seed);
        self.remember_tracker(LanePhase::StructureExploration, &sep.ct);
        self.apply_structure_candidate(candidate, StructureStage::P3Explore);
        Ok(())
    }

    fn structure_gap_repair_slice(
        &mut self,
        term: &impl Terminator,
        slice: Duration,
        sol_listener: &mut Option<&mut dyn SolutionListener>,
    ) -> Result<(), String> {
        if !self.place_groups.is_empty() || term.kill() {
            return Ok(());
        }
        let Some(before) = self.structure_incumbent.clone() else {
            return Ok(());
        };
        let structure = self.config.structure.sanitized();
        self.prob = restored_lane_problem(&self.instance, &before);
        self.solution = before.clone();
        let mut explore_config: ExplorationConfig = self.config.expl_cfg;
        explore_config.time_limit = slice;
        explore_config.separator_config.sample_config.tilt_range = 0.0;
        if self.force_single_worker {
            explore_config.separator_config.n_workers = 1;
        }
        let mut sep = self.separator(explore_config.separator_config);
        let rnr_cfg = crate::config::RuinRecreateConfig {
            period: 1,
            band_fraction: structure.p4_ruin_fraction,
            max_items: structure.p4_max_items,
            ..swarm_core::config::DEFAULT_SWARM_CONFIG.expl_cfg.rnr
        };
        let profile_cfg = structure_profile_config(&self.instance, &structure);
        ruin_and_recreate_profile_cancellable(
            &mut sep,
            &rnr_cfg,
            structure.p4_ruin_strategy,
            &mut self.structure_tabu,
            profile_cfg,
            term,
        );
        let (separated, snapshot) =
            sep.separate_with_repair(term, sol_listener, &explore_config.repair, false);
        let separated_feasible = snapshot.get_total_loss() == 0.0;
        let mut candidate = if separated_feasible {
            sep.rollback(&separated, Some(&snapshot));
            slide::slide_pass(&mut sep, &self.config.polish_cfg.slide, term);
            sep.refit_strip();
            sep.prob.save()
        } else {
            before.clone()
        };
        if separated_feasible && !term.kill() {
            // Resume the regular exploration state on the same separator and
            // under the same SliceTerminator deadline as this repair cycle.
            // Do not call new_timeout here: that would grant P4 extra time.
            #[cfg(not(feature = "adaptive"))]
            let resumed = crate::optimizer::explore::exploration_phase(
                &self.instance,
                &mut sep,
                sol_listener,
                term,
                &explore_config,
            );
            #[cfg(feature = "adaptive")]
            let resumed = crate::optimizer::explore_mods::exploration_phase_adaptive(
                &self.instance,
                &mut sep,
                sol_listener,
                term,
                &explore_config,
            );
            if let Some(best) = self.narrowest_valid(&resumed)
                && best.strip_width() < candidate.strip_width()
            {
                candidate = best.clone();
            }
        }
        self.remember_tracker(LanePhase::StructureGapRepair, &sep.ct);
        let (candidate_exact, candidate_improved) =
            self.apply_structure_candidate(candidate.clone(), StructureStage::P4Refine);
        let feasible = separated_feasible && candidate_exact;
        let improved = feasible && candidate_improved;
        let elapsed_secs = self
            .structure_started_at
            .map_or(0.0, |started| started.elapsed().as_secs_f32());
        if let Some(report) = self.structure_report.as_mut() {
            let cycle = report.repair_cycles.len();
            report.repair_cycles.push(RepairCycleRecord {
                cycle,
                elapsed_secs,
                width_before: before.strip_width(),
                width_after: if feasible {
                    candidate.strip_width()
                } else {
                    before.strip_width()
                },
                density_before: before.density(),
                density_after: if feasible {
                    candidate.density()
                } else {
                    before.density()
                },
                feasible,
                improved_global_incumbent: improved,
            });
        }
        // P4 publishes only the exact best-so-far. A valid but wider repair
        // candidate is audit data, not the next cycle's warm start.
        let incumbent = self
            .structure_incumbent
            .clone()
            .expect(swarm_core::dmsg!("Structure lane must retain an exact incumbent"));
        self.solution = incumbent.clone();
        self.prob = restored_lane_problem(&self.instance, &incumbent);
        Ok(())
    }

    fn structure_finish_slice(
        &mut self,
        term: &impl Terminator,
        slice: Duration,
        sol_listener: &mut Option<&mut dyn SolutionListener>,
    ) -> Result<(), String> {
        let Some(seed) = self.structure_incumbent.clone() else {
            return Ok(());
        };
        self.prob = restored_lane_problem(&self.instance, &seed);
        self.solution = seed.clone();
        let mut sep = self.separator(self.config.cmpr_cfg.separator_config);
        let structure = self.config.structure.sanitized();
        let mut polish_seed = seed;
        if structure.p5_snap_rotations && self.place_groups.is_empty() {
            let changed = snap_lane_rotations(&mut sep);
            if changed > 0 {
                let (snapped, snapshot) = sep.separate(term, sol_listener);
                if snapshot.get_total_loss() == 0.0 {
                    sep.rollback(&snapped, Some(&snapshot));
                    sep.refit_strip();
                    let snapped_seed = sep.prob.save();
                    if self.candidate_is_valid(&snapped_seed) {
                        polish_seed = snapped_seed;
                    } else {
                        sep = self.separator(self.config.cmpr_cfg.separator_config);
                    }
                } else {
                    sep = self.separator(self.config.cmpr_cfg.separator_config);
                }
            }
        }
        let mut config: PolishConfig = self.config.polish_cfg;
        config.time_limit = slice;
        let candidate = polish_phase_with_nfp(
            &self.instance,
            &mut sep,
            &polish_seed,
            sol_listener,
            term,
            &config,
            &self.config.nfp_integration,
            &self.config.lp_solver,
        );
        self.remember_tracker(LanePhase::StructureFinish, &sep.ct);
        self.apply_structure_candidate(candidate, StructureStage::P5Finish);
        Ok(())
    }

    fn explore_slice(
        &mut self,
        term: &impl Terminator,
        slice: Duration,
        sol_listener: &mut Option<&mut dyn SolutionListener>,
    ) -> Result<(), String> {
        let fallback = self.solution.clone();
        let mut config: ExplorationConfig = self.config.expl_cfg;
        config.time_limit = slice;
        if self.force_single_worker {
            config.separator_config.n_workers = 1;
        }
        let mut sep = self.separator(config.separator_config);
        self.rollback_slice_seed(&mut sep, &self.solution);

        let solutions = crate::ffi_step::select_exploration_phase(
            self.config.nfp_integration,
            self.config.escape_enabled,
            &self.config.escape,
            &self.config.nfp_lbf,
            &self.instance,
            &mut sep,
            sol_listener,
            term,
            &config,
        );
        let committed = self
            .narrowest_valid(&solutions)
            // Ordering reconstruction can be wider than its certified entry.
            // Keep the phase incumbent even when that wider candidate is exact.
            .filter(|candidate| candidate.strip_width() <= fallback.strip_width())
            .cloned()
            .unwrap_or(fallback);
        self.remember_tracker(LanePhase::Exploration, &sep.ct);
        sep.rollback(&committed, None);
        self.solution = committed;
        self.prob = sep.prob;
        Ok(())
    }

    fn compress_slice(
        &mut self,
        term: &impl Terminator,
        slice: Duration,
        sol_listener: &mut Option<&mut dyn SolutionListener>,
    ) -> Result<(), String> {
        let fallback = self.solution.clone();
        let mut config: CompressionConfig = self.config.cmpr_cfg;
        config.time_limit = slice;
        if self.force_single_worker {
            config.separator_config.n_workers = 1;
        }
        let mut sep = self.separator(config.separator_config);
        let candidate = compression_phase(
            &self.instance,
            &mut sep,
            &self.solution,
            sol_listener,
            term,
            &config,
        );
        self.remember_tracker(LanePhase::Compression, &sep.ct);
        let committed = if self.candidate_is_valid(&candidate) {
            candidate
        } else {
            log::warn!("[GROUPED:CMPR] rejected a non-exact lane candidate");
            fallback
        };
        sep.rollback(&committed, None);
        self.solution = committed;
        self.prob = sep.prob;
        Ok(())
    }

    fn polish_slice(
        &mut self,
        term: &impl Terminator,
        slice: Duration,
        sol_listener: &mut Option<&mut dyn SolutionListener>,
    ) -> Result<(), String> {
        // Mixed active/inactive lane schedules must not allocate workers for
        // inactive lanes either. Never assume a lane's incoming pose is valid.
        if !crate::optimizer::polish::has_active_passes(
            &self.config.polish_cfg,
            !self.place_groups.is_empty(),
        ) && self.candidate_is_valid(&self.solution)
        {
            return Ok(());
        }
        let fallback = self.solution.clone();
        let mut config: PolishConfig = self.config.polish_cfg;
        config.time_limit = slice;
        let mut sep = self.separator(self.config.cmpr_cfg.separator_config);
        let candidate = polish_phase_with_nfp(
            &self.instance,
            &mut sep,
            &self.solution,
            sol_listener,
            term,
            &config,
            &self.config.nfp_integration,
            &self.config.lp_solver,
        );
        self.remember_tracker(LanePhase::Polish, &sep.ct);
        let committed = if self.candidate_is_valid(&candidate) {
            candidate
        } else {
            log::warn!("[GROUPED:PLSH] rejected a non-exact lane candidate");
            fallback
        };
        sep.rollback(&committed, None);
        self.solution = committed;
        self.prob = sep.prob;
        Ok(())
    }
}

fn restored_lane_problem(instance: &SPInstance, solution: &SPSolution) -> SPProblem {
    let mut problem = SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
    crate::util::frozen::restore(&mut problem, solution);
    problem
}

fn structure_profile_config(
    instance: &SPInstance,
    config: &crate::structure::config::StructureFirstConfig,
) -> ProfilePlacementConfig {
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

fn snap_lane_rotations(separator: &mut Separator) -> usize {
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
                    lane_angular_distance(transform.rotation(), *a)
                        .total_cmp(&lane_angular_distance(transform.rotation(), *b))
                })
                .unwrap_or(transform.rotation()),
        };
        if lane_angular_distance(transform.rotation(), target) <= 1.0e-6 {
            continue;
        }
        separator.move_item(key, DTransformation::new(target, transform.translation()));
        changed += 1;
    }
    changed
}

fn lane_angular_distance(a: f32, b: f32) -> f32 {
    let delta = a - b;
    delta.sin().atan2(delta.cos()).abs()
}

fn rotation_range_allows(range: &RotationRange, rotation: f32) -> bool {
    match range {
        RotationRange::None => lane_angular_distance(rotation, 0.0) <= 1.0e-5,
        RotationRange::Continuous => true,
        RotationRange::Discrete(angles) => angles
            .iter()
            .any(|allowed| lane_angular_distance(rotation, *allowed) <= 1.0e-5),
    }
}

/// Validates the geometry already present in a hinted problem without
/// requiring complete item quantities. Exact contact with the rectangular
/// strip boundary follows the public hint contract and is accepted through
/// bbox containment; item pairs and non-exterior hazards remain exact.
fn hinted_layout_validation_error(
    instance: &SPInstance,
    prob: &SPProblem,
    frozen: &[usize],
    groups: &PlaceGroups,
) -> Option<String> {
    crate::structure::pipeline::partial_solution_validation_error_with_frozen_ids_and_groups(
        instance,
        &prob.save(),
        frozen,
        groups,
    )
    .map(|error| format!("{error:?}"))
}

fn rebuild_lane_from_frozen_hints(
    instance: &SPInstance,
    hinted_placements: &[(usize, DTransformation, bool)],
    construct: &mut impl FnMut(SPProblem) -> Result<SPProblem, String>,
) -> Result<SPProblem, String> {
    let mut frozen_prob = SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
    let mut required_width = frozen_prob.strip_width();
    for &(item_id, dt, frozen) in hinted_placements {
        if !frozen {
            continue;
        }
        let item = instance.item(item_id);
        if !rotation_range_allows(&item.allowed_orientations().rotations(), dt.rotation()) {
            return Err(format!(
                "invalid frozen all_items baseline: item {item_id} uses a rotation outside its allowed orientations"
            ));
        }
        let mut shape = item.shape_cd().as_ref().clone();
        shape.transform_from(&item.shape_cd(), &dt.compose());
        required_width = required_width.max(shape.bbox().x_max);
    }
    let strip_offset = frozen_prob.strip().shape_modify_config.offset.unwrap_or(0.0);
    required_width = (required_width + strip_offset) * 1.00001;
    if required_width > frozen_prob.strip_width() {
        frozen_prob.change_strip_width(required_width).expect(swarm_core::dmsg!("strip width must be positive"));
    }
    let boxes = hinted_placements
        .iter()
        .filter(|p| p.2)
        .map(|&(item_id, dt, _)| {
            let mut shape = instance.item(item_id).shape_cd().as_ref().clone();
            shape.transform_from(&instance.item(item_id).shape_cd(), &dt.compose());
            shape.bbox()
        })
        .collect::<Vec<_>>();
    crate::util::frozen::prepare_placements(&mut frozen_prob, boxes);
    for &(item_id, dt, frozen) in hinted_placements {
        if frozen {
            frozen_prob.place_item(SPPlacement {
                item_idx: item_id,
                d_transf: dt,
            });
        }
    }
    if let Some(error) = hinted_layout_validation_error(
        instance,
        &frozen_prob,
        &hinted_placements
            .iter()
            .filter(|p| p.2)
            .map(|p| p.0)
            .collect::<Vec<_>>(),
        &PlaceGroups::default(),
    ) {
        return Err(format!("invalid frozen all_items baseline: {error}"));
    }
    construct(frozen_prob)
}

/// 인스턴스 import 에 필요한 설정 묶음.
///
/// `jagua_rs::io::import::Importer` 를 그대로 돌리지 않는 이유: 겹침 허용
/// (`min_item_separation < 0`)일 때 그 타입은 컨테이너까지 부풀려 **원단 경계를
/// 연다.** 전체 인스턴스와 레인이 같은 보정을 받도록 설정만 들고 다닌다.
#[derive(Clone, Copy)]
struct ImportConfig {
    cde_config: jagua_rs::collision_detection::CDEConfig,
    poly_simpl_tolerance: Option<f32>,
    simplify_max_deviation_mm: Option<f32>,
    min_item_separation: Option<f32>,
    narrow_concavity_cutoff_ratio: Option<(f32, f32)>,
}

fn check_init_cancel() -> Result<(), String> {
    if crate::config::runtime::init_cancel_requested() {
        Err("initialization cancelled".into())
    } else { Ok(()) }
}

fn build_lane(
    input: &ValidatedInput,
    global_ids: Vec<u64>,
    gap_after_mm: f32,
    seed: u64,
    config: SwarmConfig,
    full_instance: &SPInstance,
    force_single_worker: bool,
) -> Result<RegionLane, String> {
    let global_to_local: HashMap<u64, usize> = global_ids
        .iter()
        .enumerate()
        .map(|(local, &global)| (global, local))
        .collect();
    // The full import already applied rotation restrictions, offsets, either
    // simplifier and surrogate preparation. Share those immutable Arcs; the
    // helper recomputes the lane strip and its width-dependent offset cap.
    let instance = crate::util::io::prepared_spp_subset(
        full_instance,
        &global_ids.iter().map(|&id| id as usize).collect::<Vec<_>>(),
        config.min_item_separation,
    )
    .map_err(|e| format!("failed to import region lane: {e}"))?;

    let group_specs = input
        .place_groups
        .iter()
        .filter(|group| {
            group
                .member_item_ids
                .iter()
                .all(|id| global_to_local.contains_key(id))
        })
        .map(|group| {
            let members = group
                .member_item_ids
                .iter()
                .map(|global_id| {
                    let local_id = global_to_local[global_id];
                    let hint = input.hints[global_id];
                    Ok(PlaceGroupMember {
                        item_id: local_id,
                        initial_transform: hint_to_internal(&instance, local_id, &hint)?,
                    })
                })
                .collect::<Result<Vec<_>, String>>()?;
            Ok(PlaceGroupSpec {
                id: group.id as u64,
                members,
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    validate_group_geometry(
        &instance,
        &group_specs,
        &input
            .frozen_item_ids()
            .iter()
            .filter_map(|id| global_to_local.get(id).copied())
            .collect::<Vec<_>>(),
    )?;
    let place_groups = PlaceGroups::new(&instance, group_specs)
        .map_err(|e| format!("invalid lane place group: {e}"))?;

    let mut prob = SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
    let mut frozen_ids = vec![];
    let mut hinted_placements = Vec::<(usize, DTransformation, bool)>::new();
    if input.nesting.mode == NestingMode::AllItems {
        let mut required_width = prob.strip_width();
        for (&global_id, hint) in &input.hints {
            let Some(&local_id) = global_to_local.get(&global_id) else {
                continue;
            };
            let dt = hint_to_internal(&instance, local_id, hint)?;
            let item = instance.item(local_id);
            let mut shape = item.shape_cd().as_ref().clone();
            shape.transform_from(&item.shape_cd(), &dt.compose());
            let bbox = shape.bbox();
            required_width = required_width.max(bbox.x_max);
            let frozen = hint.is_freeze != 0;
            hinted_placements.push((local_id, dt, frozen));
            if frozen {
                frozen_ids.push(local_id);
            }
        }
        // `input.hints` is a HashMap; its iteration order must not decide
        // placement order (and therefore placement keys) in the lane.
        hinted_placements.sort_by_key(|&(local_id, _, _)| global_ids[local_id]);
        frozen_ids.sort_by_key(|&local_id| global_ids[local_id]);
        let strip_offset = prob.strip().shape_modify_config.offset.unwrap_or(0.0);
        required_width = (required_width + strip_offset) * 1.00001;
        if required_width > prob.strip_width() {
            prob.change_strip_width(required_width).expect(swarm_core::dmsg!("strip width must be positive"));
        }
        let boxes = hinted_placements
            .iter()
            .map(|&(item_id, dt, _)| {
                let mut shape = instance.item(item_id).shape_cd().as_ref().clone();
                shape.transform_from(&instance.item(item_id).shape_cd(), &dt.compose());
                shape.bbox()
            })
            .collect::<Vec<_>>();
        crate::util::frozen::prepare_placements(&mut prob, boxes);
        for &(item_id, dt, _) in &hinted_placements {
            prob.place_item(SPPlacement {
                item_idx: item_id,
                d_transf: dt,
            });
        }
    }

    let mut rng = Xoshiro256PlusPlus::seed_from_u64(seed);
    let mut construct = |start_prob: SPProblem| -> Result<SPProblem, String> {
        Ok(LBFBuilder::from_problem(
            instance.clone(),
            start_prob,
            next_rng(&mut rng),
            LBF_SAMPLE_CONFIG,
            config.lbf_x_multiplier,
            config.lbf_y_multiplier,
        )
        .with_order_mode(config.lbf_order)
        .with_raster_config(config.lbf_raster)
        .with_place_groups(place_groups.clone())
        .map_err(|e| format!("failed to initialize place groups in LBF: {e}"))?
        .construct()
        .prob)
    };

    let warm_start_valid =
        hinted_layout_validation_error(&instance, &prob, &frozen_ids, &place_groups).is_none();
    let mut prob = if warm_start_valid {
        construct(prob)?
    } else {
        rebuild_lane_from_frozen_hints(&instance, &hinted_placements, &mut construct)?
    };
    check_init_cancel()?;
    let mut original_term = crate::util::terminator::InitTerminator::new(
        crate::util::terminator::BasicTerminator::new(),
    );
    original_term.new_timeout(std::time::Duration::from_secs(10));
    let _ = swarm_core::original_overlap::repair_problem(
        &mut prob,
        &frozen_ids,
        &place_groups,
        &original_term,
    );
    check_init_cancel()?;
    let mut solution = prob.save();
    if crate::structure::pipeline::solution_validation_error_with_frozen_ids_and_groups(
        &instance,
        &solution,
        &frozen_ids,
        &place_groups,
    )
    .is_some()
        && !hinted_placements.is_empty()
    {
        // A partial warm start can be collision-free itself yet lead to an
        // invalid complete layout on the current Jagua path. Rebuild once from
        // the authoritative frozen subset (or from an empty problem) so an
        // invalid movable seed is never exposed as the Init incumbent.
        prob = rebuild_lane_from_frozen_hints(&instance, &hinted_placements, &mut construct)?;
        let _ = swarm_core::original_overlap::repair_problem(
            &mut prob,
            &frozen_ids,
            &place_groups,
            &original_term,
        );
        check_init_cancel()?;
        solution = prob.save();
    }
    check_init_cancel()?;
    if let Some(error) =
        crate::structure::pipeline::solution_validation_error_with_frozen_ids_and_groups(
            &instance,
            &solution,
            &frozen_ids,
            &place_groups,
        )
    {
        return Err(format!(
            "failed to construct an exact-feasible lane baseline: {error:?}"
        ));
    }

    let frozen_reference = solution
        .layout_snapshot()
        .placed_items()
        .values()
        .filter(|pi| frozen_ids.contains(&pi.item().idx()))
        .map(|pi| (pi.item().idx(), pi.d_transf()))
        .collect();
    Ok(RegionLane {
        frozen_reference,
        instance,
        prob,
        solution,
        rng,
        local_to_global: global_ids,
        place_groups,
        frozen_ids,
        gap_after_mm,
        config,
        force_single_worker,
        tracker_session: None,
        slice_adaptation: None,
        structure_incumbent: None,
        structure_report: None,
        structure_fallback: None,
        structure_p3_repair_pending: false,
        structure_tabu: BandTabu::new(),
        structure_started_at: None,
    })
}

/// A rigid delta cannot change member-member overlap. Height feasibility is
/// checked across the anchor's allowed discrete rotations; a continuous anchor
/// is left to LBF's full-member CDE evaluator. Only pairs within the same group
/// are checked here; overlaps between placement units are left to separation.
fn validate_group_geometry(
    instance: &SPInstance,
    groups: &[PlaceGroupSpec],
    frozen: &[usize],
) -> Result<(), String> {
    for group in groups {
        if group.members.iter().all(|m| frozen.contains(&m.item_id)) {
            continue;
        }
        let shapes: Vec<(usize, SPolygon)> = group
            .members
            .iter()
            .map(|member| {
                let item = instance.item(member.item_id);
                let mut shape = item.shape_cd().as_ref().clone();
                shape.transform_from(&item.shape_cd(), &member.initial_transform.compose());
                (member.item_id, shape)
            })
            .collect();

        if !group_has_height_feasible_orientation(instance, group, &shapes) {
            return Err(format!(
                "place group {} has no allowed orientation whose vertical span fits strip height {:.3}",
                group.id, instance.base_strip.fixed_height
            ));
        }
    }
    Ok(())
}

fn group_has_height_feasible_orientation(
    instance: &SPInstance,
    group: &PlaceGroupSpec,
    initial_shapes: &[(usize, SPolygon)],
) -> bool {
    let anchor = &group.members[0];
    let candidates = match instance.item(anchor.item_id).allowed_orientations().rotations() {
        // Do not reject continuous rotation from one arbitrary hint pose. LBF
        // evaluates every member at the sampled rotation before accepting it.
        RotationRange::Continuous => return true,
        RotationRange::None => vec![0.0],
        RotationRange::Discrete(values) => values.clone(),
    };
    let anchor_initial = anchor.initial_transform.rotation();
    candidates.into_iter().any(|anchor_final| {
        let delta = Transformation::from_rotation(anchor_final - anchor_initial);
        let (y_min, y_max) = initial_shapes.iter().fold(
            (f32::INFINITY, f32::NEG_INFINITY),
            |(y_min, y_max), (_, shape)| {
                let rotated = shape.transform_clone(&delta);
                (y_min.min(rotated.bbox().y_min), y_max.max(rotated.bbox().y_max))
            },
        );
        y_max - y_min <= instance.base_strip.fixed_height + 1e-4
    })
}

fn build_fixed_prefix(
    instance: &SPInstance,
    hints: &HashMap<u64, PlacedHintInput>,
    frozen: &HashSet<u64>,
) -> Result<(Vec<ExtPlacedItem>, f32), String> {
    if frozen.is_empty() {
        return Ok((vec![], 0.0));
    }
    let mut prob = SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
    let mut placements = Vec::with_capacity(frozen.len());
    let mut required_width = prob.strip_width();
    for &global_id in frozen {
        let hint = hints
            .get(&global_id)
            .ok_or_else(|| format!("frozen item {global_id} has no hint"))?;
        let dt = hint_to_internal(instance, global_id as usize, hint)?;
        let item = instance.item(global_id as usize);
        if !rotation_range_allows(&item.allowed_orientations().rotations(), dt.rotation()) {
            return Err(format!(
                "invalid frozen prefix: item {global_id} uses a rotation outside its allowed orientations"
            ));
        }
        let mut shape = item.shape_cd().as_ref().clone();
        shape.transform_from(&item.shape_cd(), &dt.compose());
        required_width = required_width.max(shape.bbox().x_max);
        placements.push(SPPlacement {
            item_idx: global_id as usize,
            d_transf: dt,
        });
    }
    let strip_offset = prob.strip().shape_modify_config.offset.unwrap_or(0.0);
    required_width = (required_width + strip_offset) * 1.00001;
    if required_width > prob.strip_width() {
        prob.change_strip_width(required_width).expect(swarm_core::dmsg!("strip width must be positive"));
    }
    let boxes = placements
        .iter()
        .map(|p| {
            let mut shape = instance.item(p.item_idx).shape_cd().as_ref().clone();
            shape.transform_from(&instance.item(p.item_idx).shape_cd(), &p.d_transf.compose());
            shape.bbox()
        })
        .collect::<Vec<_>>();
    crate::util::frozen::prepare_placements(&mut prob, boxes);
    for placement in placements {
        prob.place_item(placement);
    }
    crate::util::frozen::fit_strip(&mut prob);
    if let Some(error) =
        crate::structure::pipeline::partial_solution_validation_error_with_frozen_ids(
            instance,
            &prob.save(),
            &frozen.iter().map(|&id| id as usize).collect::<Vec<_>>(),
        )
    {
        return Err(format!("invalid frozen prefix: {error:?}"));
    }
    let right = prob
        .layout()
        .placed_items()
        .values()
        .map(|placed| placed.shape().bbox().x_max)
        .fold(0.0f32, f32::max);
    let solution = prob.save();
    let ext = jagua_rs::probs::spp::io::export(&solution, *crate::EPOCH);
    Ok((ext.layout.placed_items, right))
}

fn hint_to_internal(
    instance: &SPInstance,
    item_id: usize,
    hint: &PlacedHintInput,
) -> Result<DTransformation, String> {
    let item = instance.item(item_id);
    let raw_bbox = item.shape_orig().shape.bbox();
    let (cos, sin) = (hint.rot_angle_in_rad.cos(), hint.rot_angle_in_rad.sin());
    let ext = DTransformation::new(
        hint.rot_angle_in_rad,
        (
            hint.tr_x - (cos * raw_bbox.x_min - sin * raw_bbox.y_min),
            hint.tr_y - (sin * raw_bbox.x_min + cos * raw_bbox.y_min),
        ),
    );
    Ok(ext_to_int_transformation(
        &ext,
        &item.shape_orig().pre_transform,
    ))
}

fn restrict_group_orientations(input: &mut ValidatedInput) -> Result<(), String> {
    const EPS: f32 = 1e-3;
    for group in &input.place_groups {
        let anchor_id = group.member_item_ids[0] as usize;
        let anchor_initial = input.hints[&(anchor_id as u64)]
            .rot_angle_in_rad
            .to_degrees()
            .rem_euclid(360.0);
        let mut candidates = orientation_values(&input.instance.items[anchor_id].base.orientation);
        if candidates.is_none() {
            // A discrete member seeds the finite anchor candidates. Preserve
            // the hint-relative pose: member_final = member_initial +
            // (anchor_final - anchor_initial), therefore
            // anchor_final = anchor_initial + allowed_member - member_initial.
            if let Some((&member_id, discrete)) = group.member_item_ids.iter().find_map(|id| {
                orientation_values(&input.instance.items[*id as usize].base.orientation)
                .map(|values| (id, values))
            }) {
                let member_initial = input.hints[&member_id]
                    .rot_angle_in_rad
                    .to_degrees()
                    .rem_euclid(360.0);
                candidates = Some(
                    discrete
                        .into_iter()
                        .map(|allowed_member| {
                            (anchor_initial + allowed_member - member_initial).rem_euclid(360.0)
                        })
                        .collect(),
                );
            }
        }
        if let Some(mut candidates) = candidates {
            for &member_id in &group.member_item_ids {
                let allowed = orientation_values(&input.instance.items[member_id as usize].base.orientation);
                if let Some(allowed) = allowed {
                    let member_initial = input.hints[&member_id]
                        .rot_angle_in_rad
                        .to_degrees()
                        .rem_euclid(360.0);
                    let relative_angle = member_initial - anchor_initial;
                    candidates.retain(|candidate| {
                        let member_final = (*candidate + relative_angle).rem_euclid(360.0);
                        allowed
                            .iter()
                            .any(|angle| angle_distance(member_final, *angle) <= EPS)
                    });
                }
            }
            // Several member orientations can map to the same anchor angle.
            let mut unique = Vec::with_capacity(candidates.len());
            for candidate in candidates {
                if !unique
                    .iter()
                    .any(|existing| angle_distance(*existing, candidate) <= EPS)
                {
                    unique.push(candidate);
                }
            }
            let candidates = unique;
            if candidates.is_empty() {
                return Err(format!(
                    "place group {} has no common allowed orientation",
                    group.id
                ));
            }
            input.instance.items[anchor_id].base.orientation.rotation =
                jagua_rs::io::ext_repr::ExtRotation::Discrete { angles: candidates };
        }
    }
    Ok(())
}

/// Finite allowed angles in degrees, or `None` for continuous rotation.
/// (jagua-rs 1.x orientation object; a stepped rotation is expanded.)
fn orientation_values(orientation: &jagua_rs::io::ext_repr::ExtOrientation) -> Option<Vec<f32>> {
    use jagua_rs::io::ext_repr::ExtRotation;
    match &orientation.rotation {
        ExtRotation::Continuous {} => None,
        ExtRotation::Discrete { angles } if angles.is_empty() => Some(vec![0.0]),
        ExtRotation::Discrete { angles } => Some(angles.clone()),
        ExtRotation::Stepped { step } => {
            let count = (360.0 / step).round().max(1.0) as usize;
            Some((0..count).map(|i| i as f32 * 360.0 / count as f32).collect())
        }
    }
}

fn angle_distance(a: f32, b: f32) -> f32 {
    let delta = (a - b).rem_euclid(360.0);
    delta.min(360.0 - delta)
}

fn derive_lane_seed(base: u64, lane: u64) -> u64 {
    let mut x = base ^ lane.wrapping_mul(0x9E37_79B9_7F4A_7C15);
    x ^= x >> 30;
    x = x.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    x ^= x >> 27;
    x = x.wrapping_mul(0x94D0_49BB_1331_11EB);
    x ^ (x >> 31)
}

fn next_rng(rng: &mut Xoshiro256PlusPlus) -> Xoshiro256PlusPlus {
    Xoshiro256PlusPlus::seed_from_u64(rng.random())
}

// Native group pools use an explicit count, independent of Rayon's global pool.
#[cfg(any(test, not(target_arch = "wasm32")))]
fn group_worker_budget(
    requested: usize,
    lanes: usize,
    rayon_env: Option<&str>,
    available: Option<usize>,
) -> usize {
    let requested = requested.min(lanes).max(1);
    let explicit = rayon_env
        .and_then(|value| value.parse::<usize>().ok())
        .filter(|&value| value > 0);
    [explicit, available.filter(|&value| value > 0)]
        .into_iter()
        .flatten()
        .fold(requested, usize::min)
}

fn runtime_group_worker_budget(requested: usize, lanes: usize) -> usize {
    #[cfg(not(target_arch = "wasm32"))]
    {
        let rayon_env = std::env::var("RAYON_NUM_THREADS").ok();
        let available = std::thread::available_parallelism().ok().map(usize::from);
        group_worker_budget(requested, lanes, rayon_env.as_deref(), available)
    }
    #[cfg(target_arch = "wasm32")]
    {
        requested.min(lanes).max(1)
    }
}

fn fair_lane_slice(round_budget: Duration, lane_count: usize, worker_count: usize) -> Duration {
    let workers = worker_count.max(1).min(lane_count.max(1));
    let waves = lane_count.max(1).div_ceil(workers);
    round_budget
        .checked_div(waves.min(u32::MAX as usize) as u32)
        .unwrap_or(round_budget)
}

const ADAPTIVE_SLICE_CAP: Duration = Duration::from_secs(30);
const EXACT_WIDTH_ABS_EPSILON: f32 = 1.0e-5;
const EXACT_WIDTH_REL_EPSILON: f32 = 1.0e-6;

fn adaptive_lane_slice(
    base_lane_slice: Duration,
    adaptive_level: u32,
    configured_quantum: Duration,
    remaining_phase: Duration,
    lane_count: usize,
    worker_count: usize,
) -> Duration {
    let multiplier = 1u32.checked_shl(adaptive_level).unwrap_or(u32::MAX);
    let requested = base_lane_slice
        .checked_mul(multiplier)
        .unwrap_or(Duration::MAX);
    let configured_cap = configured_quantum.max(ADAPTIVE_SLICE_CAP);
    // Reserve an equal worst-case allowance for every remaining worker wave.
    // Even when adaptive learning grows a lane's slice beyond the configured
    // base quantum, the whole phase retains its one shared hard deadline.
    let remaining_wave_cap = fair_lane_slice(remaining_phase, lane_count, worker_count);
    requested.min(configured_cap).min(remaining_wave_cap)
}

fn exact_width_epsilon(before: f32, after: f32) -> f32 {
    EXACT_WIDTH_ABS_EPSILON.max(before.abs().max(after.abs()).max(1.0) * EXACT_WIDTH_REL_EPSILON)
}

fn exact_width_improved(before: f32, after: f32) -> bool {
    before.is_finite() && after.is_finite() && after < before - exact_width_epsilon(before, after)
}

fn exact_width_unchanged(before: f32, after: f32) -> bool {
    before.is_finite()
        && after.is_finite()
        && !exact_width_improved(before, after)
        && (after - before).abs() <= exact_width_epsilon(before, after)
}

struct SliceTerminator<'a> {
    stop: &'a AtomicBool,
    deadline: Instant,
    hard_deadline: Instant,
}

impl<'a> SliceTerminator<'a> {
    fn new(stop: &'a AtomicBool, duration: Duration) -> Self {
        let deadline = Instant::now() + duration;
        Self {
            stop,
            deadline,
            hard_deadline: deadline,
        }
    }

    fn until(stop: &'a AtomicBool, deadline: Instant) -> Self {
        Self {
            stop,
            deadline,
            hard_deadline: deadline,
        }
    }

    fn new_capped(stop: &'a AtomicBool, duration: Duration, hard_deadline: Instant) -> Self {
        let requested = Instant::now() + duration;
        let deadline = if requested < hard_deadline {
            requested
        } else {
            hard_deadline
        };
        Self {
            stop,
            deadline,
            hard_deadline,
        }
    }
}

impl Terminator for SliceTerminator<'_> {
    fn kill(&self) -> bool {
        self.stop.load(Ordering::Relaxed) || Instant::now() >= self.deadline
    }

    fn stop_requested(&self) -> bool {
        self.stop.load(Ordering::Relaxed)
    }

    fn new_timeout(&mut self, timeout: Duration) {
        let requested = Instant::now() + timeout;
        self.deadline = if requested < self.hard_deadline {
            requested
        } else {
            self.hard_deadline
        };
    }

    fn timeout_at(&self) -> Option<Instant> {
        Some(self.deadline)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grouped_input::parse_and_validate;

    fn curved_v2_input(vertex_count: usize) -> String {
        let radius = 8.0f32;
        let center = 10.0f32;
        let points = (0..vertex_count)
            .map(|index| {
                let angle = std::f32::consts::TAU * index as f32 / vertex_count as f32;
                [center + radius * angle.cos(), center + radius * angle.sin()]
            })
            .collect::<Vec<_>>();
        serde_json::json!({
            "schema_version": 2,
            "nesting": {
                "mode": "all_items",
                "group_workers": 1,
                "quantum_ms": 1000,
                "locked_prefix_gap_mm": 0.0
            },
            "instance": {
                "name": "grouped-structure-simplification",
                "strip_height": 24.0,
                "items": [{
                    "id": 0,
                    "demand": 1,
                    "allowed_orientations": [0.0],
                    "shape": {"type": "simple_polygon", "data": points}
                }]
            },
            "regions": [],
            "place_groups": []
        })
        .to_string()
    }

    #[test]
    fn grouped_structure_keeps_conservative_polygon_simplification() {
        let input = parse_and_validate(&curved_v2_input(96)).unwrap();
        let mut config = crate::config::DEFAULT_SWARM_CONFIG;
        config.rng_seed = Some(19);
        config.structure.enabled = true;
        config.poly_simpl_tolerance = Some(0.01);

        let engine = GroupedStepState::new(input, config).unwrap();
        assert!(
            engine.full_instance.item(0).shape_cd().n_vertices() < 96,
            "Structure must retain Jagua's conservative simplification"
        );
    }

    #[test]
    fn scheduler_quantum_is_divided_by_worker_waves() {
        let quantum = Duration::from_millis(60);
        assert_eq!(fair_lane_slice(quantum, 4, 1), Duration::from_millis(15));
        assert_eq!(fair_lane_slice(quantum, 4, 2), Duration::from_millis(30));
        assert_eq!(fair_lane_slice(quantum, 4, 4), quantum);
        assert_eq!(fair_lane_slice(quantum, 2, 8), quantum);
        assert_eq!(
            fair_lane_slice(Duration::from_millis(50), 4, 1),
            Duration::from_micros(12_500)
        );
    }

    #[test]
    fn adaptive_slice_doubles_but_reserves_every_worker_wave() {
        let quantum = Duration::from_millis(50);
        let remaining = Duration::from_secs(60);
        let base = fair_lane_slice(quantum, 4, 1);
        assert_eq!(base, Duration::from_micros(12_500));
        assert_eq!(adaptive_lane_slice(base, 0, quantum, remaining, 4, 1), base);
        assert_eq!(
            adaptive_lane_slice(base, 1, quantum, remaining, 4, 1),
            Duration::from_millis(25)
        );
        assert_eq!(
            adaptive_lane_slice(base, 2, quantum, remaining, 4, 1),
            Duration::from_millis(50)
        );
        assert_eq!(
            adaptive_lane_slice(base, 9, quantum, remaining, 4, 1),
            Duration::from_millis(6_400)
        );
        // Growth is capped by ADAPTIVE_SLICE_CAP once the phase has room for it.
        let long_remaining = Duration::from_secs(600);
        assert_eq!(
            adaptive_lane_slice(base, 20, quantum, long_remaining, 4, 1),
            ADAPTIVE_SLICE_CAP
        );

        let short_remaining = Duration::from_millis(300);
        let capped = adaptive_lane_slice(base, 20, quantum, short_remaining, 4, 1);
        assert_eq!(capped, Duration::from_millis(75));
        assert_eq!(capped.checked_mul(4), Some(short_remaining));

        let two_worker_cap = adaptive_lane_slice(base, 20, quantum, short_remaining, 4, 2);
        assert_eq!(two_worker_cap, Duration::from_millis(150));
        assert_eq!(two_worker_cap.checked_mul(2), Some(short_remaining));

        // A configured quantum above ADAPTIVE_SLICE_CAP remains the cap.
        let long_quantum = Duration::from_secs(40);
        let long_base = fair_lane_slice(long_quantum, 4, 1);
        assert_eq!(
            adaptive_lane_slice(long_base, 20, long_quantum, long_remaining, 4, 1),
            long_quantum
        );
    }

    #[test]
    fn exact_width_progress_uses_absolute_and_relative_epsilon() {
        assert!(!exact_width_improved(1.0, 1.0 - 5.0e-6));
        assert!(exact_width_unchanged(1.0, 1.0 - 5.0e-6));
        assert!(exact_width_improved(1.0, 1.0 - 2.0e-5));

        assert!(!exact_width_improved(1_000.0, 999.9995));
        assert!(exact_width_unchanged(1_000.0, 999.9995));
        assert!(exact_width_improved(1_000.0, 999.998));
        assert!(!exact_width_unchanged(f32::NAN, 1.0));
    }

    #[test]
    fn adaptive_learning_ignores_stop_and_keeps_level_after_improvement() {
        let mut adaptation = LaneSliceAdaptation::new(LanePhase::Exploration);
        let unchanged_timeout = LaneSliceOutcome {
            before_width: 100.0,
            after_width: 100.0,
            timed_out: true,
            stop_requested: false,
        };
        adaptation.observe(unchanged_timeout);
        adaptation.observe(unchanged_timeout);
        adaptation.observe(unchanged_timeout);
        assert_eq!(adaptation.level, 3);

        let improved_timeout = LaneSliceOutcome {
            after_width: 99.0,
            ..unchanged_timeout
        };
        adaptation.observe(improved_timeout);
        assert_eq!(adaptation.level, 3);

        let stopped = LaneSliceOutcome {
            stop_requested: true,
            ..unchanged_timeout
        };
        adaptation.observe(stopped);
        assert_eq!(adaptation.level, 3);
    }

    #[test]
    fn slice_timeout_cannot_extend_the_phase_hard_deadline() {
        let stop = AtomicBool::new(false);
        let hard_deadline = Instant::now() + Duration::from_millis(50);
        let mut term = SliceTerminator::new_capped(&stop, Duration::from_secs(1), hard_deadline);
        assert_eq!(term.timeout_at(), Some(hard_deadline));

        term.new_timeout(Duration::from_secs(10));
        assert_eq!(term.timeout_at(), Some(hard_deadline));
    }

    #[test]
    fn structure_finish_preserves_equal_width_leftward_progress() {
        let input = size_region_geometry_input(
            30.0,
            vec![
                positioned_rectangle(0, 5.01, 0.01),
                positioned_rectangle(1, 25.01, 0.01),
            ],
            serde_json::json!([]),
        );
        let mut engine = GroupedStepState::new(input, fast_structure_config()).unwrap();
        let lane = &mut engine.lanes[0];
        let mut sep = lane.separator(lane.config.cmpr_cfg.separator_config);
        let width = lane.solution.strip_width();
        sep.prob.change_strip_width(width + 20.0).expect(swarm_core::dmsg!("strip width must be positive"));
        let poses: Vec<_> = sep
            .prob
            .layout()
            .placed_items()
            .iter()
            .map(|(k, p)| (k, p.d_transf()))
            .collect();
        for (key, pose) in &poses {
            sep.move_item(
                *key,
                DTransformation::new(
                    pose.rotation(),
                    (pose.translation().0 + 10.0, pose.translation().1),
                ),
            );
        }
        let signature = |solution: &SPSolution| {
            let mut poses: Vec<_> = solution
                .layout_snapshot()
                .placed_items()
                .values()
                .map(|p| {
                    (
                        p.item().idx(),
                        p.d_transf().rotation().to_bits(),
                        p.d_transf().translation().0.to_bits(),
                        p.d_transf().translation().1.to_bits(),
                    )
                })
                .collect();
            poses.sort_unstable();
            poses
        };
        let baseline = sep.prob.save();
        assert!(lane.candidate_is_valid(&baseline));
        lane.solution = baseline.clone();
        lane.structure_incumbent = Some(baseline.clone());
        let shifted: Vec<_> = sep
            .prob
            .layout()
            .placed_items()
            .iter()
            .map(|(k, p)| (k, p.d_transf()))
            .collect();
        for (key, pose) in shifted {
            sep.move_item(
                key,
                DTransformation::new(
                    pose.rotation(),
                    (pose.translation().0 - 4.0, pose.translation().1),
                ),
            );
        }
        let compact = sep.prob.save();
        assert_eq!(compact.strip_width(), baseline.strip_width());
        assert!(lane.candidate_is_valid(&compact));
        let (exact, narrower) =
            lane.apply_structure_candidate(compact.clone(), StructureStage::P5Finish);
        assert!(
            exact && !narrower,
            "secondary progress is not a narrower-width result"
        );
        lane.finish_structure_stage(StructureStage::P5Finish);
        assert_eq!(signature(&compact), signature(&lane.solution));
        assert_eq!(compact.strip_width(), lane.solution.strip_width());

        // A later equal-width pass must not undo the retained interior movement.
        lane.apply_structure_candidate(baseline, StructureStage::P5Finish);
        lane.finish_structure_stage(StructureStage::P5Finish);
        assert_eq!(signature(&compact), signature(&lane.solution));
        assert_eq!(compact.strip_width(), lane.solution.strip_width());

        // Even a more compact interior cannot justify a wider container.
        let mut wider = restored_lane_problem(&lane.instance, &compact);
        wider.change_strip_width(compact.strip_width() + 1.0).expect(swarm_core::dmsg!("strip width must be positive"));
        lane.apply_structure_candidate(wider.save(), StructureStage::P5Finish);
        lane.finish_structure_stage(StructureStage::P5Finish);
        assert_eq!(signature(&compact), signature(&lane.solution));
        assert_eq!(compact.strip_width(), lane.solution.strip_width());
    }

    #[test]
    fn issue2_killed_queued_lane_retains_tracker_rng_and_incumbent() {
        let input = size_region_geometry_input(
            30.0,
            vec![
                positioned_rectangle(0, 0.01, 0.01),
                positioned_rectangle(1, 12.01, 0.01),
            ],
            serde_json::json!([]),
        );
        let mut engine =
            GroupedStepState::new(input, crate::config::DEFAULT_SWARM_CONFIG).unwrap();
        let lane = &mut engine.lanes[0];
        lane.begin_tracker_phase(LanePhase::Exploration);
        let sep = lane.separator(lane.config.expl_cfg.separator_config);
        lane.remember_tracker(LanePhase::Exploration, &sep.ct);
        let before = lane.solution.clone();
        let mut expected_rng = lane.rng.clone();
        let stop = AtomicBool::new(false);
        let term = SliceTerminator::new(&stop, Duration::ZERO);
        lane.run_slice(LanePhase::Compression, &term, Duration::from_millis(1)).unwrap();
        assert!(lane.tracker_session.as_ref().is_some_and(|s| s.phase == LanePhase::Exploration),
            "a killed queued lane must not reset the live GLS session");
        assert_eq!(lane.rng.random::<u64>(), expected_rng.random::<u64>(),
            "a killed queued lane must not consume separator setup RNG");
        assert_eq!(lane.solution.strip_width(), before.strip_width());
        for (key, item) in before.layout_snapshot().placed_items() {
            assert_eq!(lane.solution.layout_snapshot().placed_items()[key].d_transf(), item.d_transf());
        }
        assert!(lane.candidate_is_valid(&lane.solution));
        // An explicit Stop in a queued lane has the same no-setup contract.
        stop.store(true, Ordering::Relaxed);
        let term = SliceTerminator::new(&stop, Duration::from_secs(60));
        let mut expected_rng = lane.rng.clone();
        lane.run_slice(LanePhase::Polish, &term, Duration::from_millis(1)).unwrap();
        assert!(lane.tracker_session.as_ref().is_some_and(|s| s.phase == LanePhase::Exploration));
        assert_eq!(lane.rng.random::<u64>(), expected_rng.random::<u64>());
        assert_eq!(lane.solution.strip_width(), before.strip_width());
    }

    #[test]
    fn size_region_tracker_weights_survive_only_same_phase_slices() {
        let input = size_region_geometry_input(
            30.0,
            vec![
                positioned_rectangle(0, 0.01, 0.01),
                positioned_rectangle(1, 12.01, 0.01),
            ],
            serde_json::json!([]),
        );
        let mut engine =
            GroupedStepState::new(input, crate::config::DEFAULT_SWARM_CONFIG).unwrap();
        let lane = &mut engine.lanes[0];
        assert!(lane.force_single_worker);
        let exploration_config = lane.config.expl_cfg.separator_config;
        let compression_config = lane.config.cmpr_cfg.separator_config;

        // First exploration slice learns a non-neutral weight for one key.
        lane.begin_tracker_phase(LanePhase::Exploration);
        let mut first = lane.separator(exploration_config);
        let survivor_item_id = 0;
        let first_pk = first
            .prob
            .layout()
            .placed_items()
            .iter()
            .find_map(|(pk, item)| (item.item().idx() == survivor_item_id).then_some(pk))
            .unwrap();
        let first_idx = first.ct.pk_idx_map[first_pk];
        first.ct.container_collisions_mut()[first_idx].weight = 3.25;
        lane.remember_tracker(LanePhase::Exploration, &first.ct);

        // Re-entering the same phase builds exact losses afresh and copies the
        // learned value only for placement keys that survived the boundary.
        lane.begin_tracker_phase(LanePhase::Exploration);
        let mut second = lane.separator(exploration_config);
        let second_seed = lane.solution.clone();
        lane.rollback_slice_seed(&mut second, &second_seed);
        let second_pk = second
            .prob
            .layout()
            .placed_items()
            .iter()
            .find_map(|(pk, item)| (item.item().idx() == survivor_item_id).then_some(pk))
            .unwrap();
        assert_eq!(second.ct.get_container_weight(second_pk), 3.25);

        // A phase boundary starts a new GLS session rather than leaking the
        // exploration landscape into compression.
        lane.begin_tracker_phase(LanePhase::Compression);
        assert!(lane.tracker_session.is_none());
        let compression = lane.separator(compression_config);
        let compression_pk = compression
            .prob
            .layout()
            .placed_items()
            .iter()
            .find_map(|(pk, item)| (item.item().idx() == survivor_item_id).then_some(pk))
            .unwrap();
        assert_eq!(compression.ct.get_container_weight(compression_pk), 1.0);
    }

    #[test]
    fn group_worker_budget_respects_explicit_and_available_cpu_limits() {
        assert_eq!(group_worker_budget(4, 4, Some("2"), Some(8)), 2);
        assert_eq!(group_worker_budget(8, 6, Some("4"), Some(2)), 2);
        assert_eq!(group_worker_budget(1, 4, Some("2"), Some(8)), 1);
        assert_eq!(group_worker_budget(8, 1, Some("2"), Some(8)), 1);
    }

    #[test]
    fn group_worker_budget_ignores_unusable_limits_and_never_returns_zero() {
        for env in [None, Some(""), Some("0"), Some("invalid"), Some("-2"), Some("999999999999999999999999999999")] {
            assert_eq!(group_worker_budget(4, 4, env, Some(2)), 2);
            assert_eq!(group_worker_budget(4, 4, env, None), 4);
        }
        assert_eq!(group_worker_budget(4, 4, Some("2"), None), 2);
        assert_eq!(group_worker_budget(4, 4, None, Some(0)), 4);
        assert_eq!(group_worker_budget(0, 0, Some("2"), Some(8)), 1);
        assert_eq!(group_worker_budget(4, 0, Some("2"), Some(8)), 1);
    }

    #[cfg(all(feature = "structure-first", not(target_arch = "wasm32")))]
    #[test]
    fn group_worker_cap_wires_both_pools_and_gives_each_p4_lane_one_cycle() {
        let items: Vec<_> = (0..8).map(|id| rectangle(id, false, 0.0)).collect();
        let regions: Vec<_> = (0..4).map(|id| serde_json::json!({
            "id": format!("lane-{id}"), "kind":"size_bundle",
            "item_ids":[id*2,id*2+1], "gap_after_mm":1.0
        })).collect();
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"size_regions","group_workers":4,"quantum_ms":1,"locked_prefix_gap_mm":0.0},
            "instance":{"name":"worker-cap-p4","strip_height":30.0,"items":items},
            "regions":regions,"place_groups":[]
        }).to_string();
        let mut config = fast_structure_config();
        config.structure.p4_cycle_time_limit = Duration::from_millis(400);
        let mut engine = GroupedStepState::new(parse_and_validate(&json).unwrap(), config).unwrap();
        // Inject limits without mutating the process-wide environment.
        engine.effective_group_workers = group_worker_budget(4, 4, Some("2"), Some(8));
        let stop = AtomicBool::new(false);
        engine.run_structure(&stop).unwrap();
        assert!(engine.observed_group_pool_workers.len() >= 3,
            "P1 and the later stage pools must both be exercised");
        assert!(engine.observed_group_pool_workers.iter().all(|&workers| workers == 2));
        let before: Vec<_> = engine.lanes.iter().map(|lane| lane.solution.strip_width()).collect();
        engine.run_scheduled_with_callbacks(&stop, LanePhase::StructureGapRepair, &mut |_| {}, None).unwrap();
        for (lane, before_width) in engine.lanes.iter().zip(before) {
            assert_eq!(lane.structure_report.as_ref().unwrap().repair_cycles.len(), 1,
                "each queued lane gets one P4 cycle, not one cycle per quantum");
            assert!(lane.solution.strip_width() <= before_width + 1.0e-5);
            assert!(lane.candidate_is_valid(&lane.solution));
        }
        assert!(engine.export_certified_solution().is_ok());
        let report: serde_json::Value = serde_json::from_str(&engine.structure_report_json().unwrap()).unwrap();
        assert_eq!(report["group_workers"], 4);
        assert_eq!(report["effective_group_workers"], 2);
        assert_eq!(report["lanes"].as_array().unwrap().len(), 4);

        // Stop before the next queued cycle must preserve all lane state.
        let expected: Vec<_> = engine.lanes.iter().map(|lane| {
            let mut rng = lane.rng.clone();
            (rng.random::<u64>(), lane.solution.strip_width())
        }).collect();
        let pools_before = engine.observed_group_pool_workers.len();
        stop.store(true, Ordering::Relaxed);
        engine.run_scheduled_with_callbacks(&stop, LanePhase::StructureGapRepair, &mut |_| {}, None).unwrap();
        assert_eq!(engine.observed_group_pool_workers.len(), pools_before);
        for (lane, (random, width)) in engine.lanes.iter_mut().zip(expected) {
            assert_eq!(lane.rng.random::<u64>(), random);
            assert_eq!(lane.solution.strip_width(), width);
            assert_eq!(lane.structure_report.as_ref().unwrap().repair_cycles.len(), 1);
        }
    }

    #[test]
    fn group_worker_cap_preserves_all_items_internal_workers() {
        let mut config = fast_structure_config();
        config.expl_cfg.time_limit = Duration::from_millis(2);
        config.expl_cfg.separator_config.n_workers = 3;
        let input = parse_and_validate(&curved_v2_input(4)).unwrap();
        let mut engine = GroupedStepState::new(input, config).unwrap();
        engine.effective_group_workers = 1;
        engine.run_scheduled_with_callbacks(&AtomicBool::new(false), LanePhase::Exploration, &mut |_| {}, None).unwrap();
        assert!(engine.observed_group_pool_workers.is_empty());
        assert_eq!(engine.lanes[0].config.expl_cfg.separator_config.n_workers, 3);
        assert!(!engine.lanes[0].force_single_worker);
        assert!(engine.export_certified_solution().is_ok());
    }

    fn fast_structure_config() -> SwarmConfig {
        let mut config = crate::config::DEFAULT_SWARM_CONFIG;
        config.structure.enabled = true;
        config.structure.construction_starts = 1;
        config.structure.p2_time_limit = Duration::from_millis(2);
        config.structure.p3_time_limit = Some(Duration::from_millis(2));
        config.structure.p4_cycles = 0;
        config.polish_cfg.enabled = false;
        config.polish_cfg.time_limit = Duration::from_millis(1);
        config
    }

    #[cfg(feature = "structure-first")]
    fn assert_schema_v3_lane_invariants(lane: &serde_json::Value) {
        assert_eq!(lane["schema_version"], 3);
        let report = &lane["report"];
        assert_eq!(report["schema_version"], 3);
        assert_eq!(
            report["motif_generation_stats"]["singles"]["emitted"],
            report["motif_singles"]
        );
        assert_eq!(
            report["motif_generation_stats"]["pairs"]["emitted"],
            report["motif_pairs"]
        );
        assert_eq!(
            report["motif_generation_stats"]["rectangle"]["emitted"],
            report["motif_blocks"]
        );
        assert_eq!(
            report["motif_generation_stats"]["bands"]["emitted"],
            report["motif_bands"]
        );
        assert_eq!(
            report["motif_selection_stats"]["pair_edges_considered"],
            report["pair_matching_edges"]
        );
        assert_eq!(
            report["motif_selection_stats"]["selected_pairs"],
            report["pair_matching_selected_pairs"]
        );
        let starts = report["p1_start_stats"].as_array().unwrap();
        assert_eq!(
            report["construction_starts_attempted"].as_u64().unwrap() as usize,
            starts.len()
        );
        let completed = starts
            .iter()
            .filter(|start| start["complete"] == true && start["exact_valid"] == true)
            .count();
        assert_eq!(
            report["construction_starts_completed"].as_u64().unwrap() as usize,
            completed
        );
        let selected =
            usize::from(report["p1_control_candidate"]["selected_as_p1_incumbent"] == true)
                + starts
                    .iter()
                    .filter(|start| start["selected_as_p1_incumbent"] == true)
                    .count();
        assert_eq!(selected, 1);
        assert!(report["p1_winner"].is_object());
    }

    fn rectangle(id: u64, frozen: bool, x: f32) -> serde_json::Value {
        serde_json::json!({
            "id":id,"demand":1,"allowed_orientations":[0.0,180.0],
            "shape":{"type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":10.0,"height":10.0}},
            "placed_hint":{"item_id":id,"is_freeze":if frozen {1}else{0},"tr_x":x,"tr_y":0.0,"rot_angle_in_rad":0.0,"stripe_interval_y":0.0,"stripe_interval_x":0.0}
        })
    }

    fn orientation_input(
        anchor_allowed: Option<Vec<f32>>,
        anchor_initial_deg: f32,
        member_allowed: Option<Vec<f32>>,
        member_initial_deg: f32,
    ) -> ValidatedInput {
        fn oriented_rectangle(
            id: u64,
            allowed: Option<Vec<f32>>,
            initial_deg: f32,
            x: f32,
        ) -> serde_json::Value {
            serde_json::json!({
                "id":id,"demand":1,"allowed_orientations":allowed,
                "shape":{"type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":10.0,"height":10.0}},
                "placed_hint":{"item_id":id,"is_freeze":0,"tr_x":x,"tr_y":0.0,"rot_angle_in_rad":initial_deg.to_radians(),"stripe_interval_y":0.0,"stripe_interval_x":0.0}
            })
        }

        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items","group_workers":1,"quantum_ms":5,"locked_prefix_gap_mm":0.0},
            "instance":{"name":"orientation-group","strip_height":40.0,"items":[
                oriented_rectangle(0, anchor_allowed, anchor_initial_deg, 0.0),
                oriented_rectangle(1, member_allowed, member_initial_deg, 12.0)
            ]},
            "regions":[],
            "place_groups":[{"id":9,"member_item_ids":[0,1]}]
        })
        .to_string();
        parse_and_validate(&json).unwrap()
    }

    fn issue7_reference_separator(
        lane: &mut RegionLane,
        mut config: crate::optimizer::separator::SeparatorConfig,
    ) -> Separator {
        if lane.force_single_worker {
            config.n_workers = 1;
        }
        let mut sep = Separator::new(
            lane.instance.clone(), lane.prob.clone(), next_rng(&mut lane.rng), config,
        );
        sep.configure_reinsertion(&lane.config.nfp_integration);
        sep.set_place_groups(lane.place_groups.clone()).unwrap();
        sep.freeze_items(&lane.frozen_ids);
        if lane.force_single_worker && let Some(session) = &lane.tracker_session {
            sep.ct.inherit_weights_from(&session.tracker);
        }
        sep
    }

    fn issue7_assert_place_group_policy_equal(
        expected: &PlaceGroups,
        actual: &PlaceGroups,
        layout: &jagua_rs::entities::Layout,
    ) {
        // An empty validated table has N None slots; Default has none. Both
        // expose exactly the same no-group placement policy.
        assert_eq!(expected.is_empty(), actual.is_empty());
        if !expected.is_empty() {
            assert_eq!(expected, actual);
        }
        for placed in layout.placed_items().values() {
            let id = placed.item().idx();
            assert_eq!(expected.group_id_of(id), actual.group_id_of(id));
            assert_eq!(expected.representative(id), actual.representative(id));
            assert_eq!(expected.member_item_ids(id), actual.member_item_ids(id));
            for other in layout.placed_items().values() {
                assert_eq!(expected.same_group(id, other.item().idx()), actual.same_group(id, other.item().idx()));
            }
        }
    }

    fn issue7_assert_tracker_equal(
        expected: &CollisionTracker,
        actual: &CollisionTracker,
        layout: &jagua_rs::entities::Layout,
    ) {
        assert!(swarm_core::util::assertions::tracker_matches_layout(actual, layout));
        assert_eq!(expected.size, actual.size);
        assert_eq!(expected.frozen, actual.frozen);
        issue7_assert_place_group_policy_equal(&expected.place_groups, &actual.place_groups, layout);
        assert_eq!(expected.get_total_loss().to_bits(), actual.get_total_loss().to_bits());
        assert_eq!(expected.get_total_weighted_loss().to_bits(), actual.get_total_weighted_loss().to_bits());
        let keys: Vec<_> = layout.placed_items().keys().collect();
        for (i, &key) in keys.iter().enumerate() {
            assert!(expected.pk_idx_map.contains_key(key));
            assert!(actual.pk_idx_map.contains_key(key));
            assert_eq!(expected.get_loss(key).to_bits(), actual.get_loss(key).to_bits());
            assert_eq!(expected.get_container_weight(key).to_bits(), actual.get_container_weight(key).to_bits());
            for &other in &keys[i + 1..] {
                assert_eq!(expected.get_pair_weight(key, other).to_bits(), actual.get_pair_weight(key, other).to_bits());
            }
        }
    }

    fn issue7_assert_separator_equal(expected: &Separator, actual: &Separator) {
        assert_same_solution(&expected.prob.save(), &actual.prob.save());
        assert_eq!(expected.frozen_mask, actual.frozen_mask);
        assert_eq!(expected.place_groups, actual.place_groups);
        assert_eq!(expected.rng.clone().random::<[u64; 4]>(), actual.rng.clone().random::<[u64; 4]>());
        issue7_assert_tracker_equal(&expected.ct, &actual.ct, &actual.prob.layout());
        assert_eq!(expected.workers.len(), actual.workers.len());
        for (old, new) in expected.workers.iter().zip(&actual.workers) {
            assert_same_solution(&old.prob.save(), &new.prob.save());
            assert_eq!(old.frozen_mask, new.frozen_mask);
            issue7_assert_place_group_policy_equal(&old.place_groups, &new.place_groups, &new.prob.layout());
            assert_eq!(old.rng.clone().random::<[u64; 4]>(), new.rng.clone().random::<[u64; 4]>());
            issue7_assert_tracker_equal(&old.ct, &new.ct, &new.prob.layout());
        }
    }

    #[test]
    fn issue7_native_fresh_lane_setup_calls_and_state_equivalence() {
        use crate::optimizer::place_groups::{PlaceGroupMember, PlaceGroupSpec};
        for size_regions in [false, true] {
            for requested_workers in [1, 2] {
                for has_groups in [false, true] {
                    for has_frozen in [false, true] {
                        let items = vec![positioned_rectangle(0, 0.01, 0.01),
                                         positioned_rectangle(1, 12.01, 0.01)];
                        let input = if size_regions {
                            size_region_geometry_input(30.0, items, serde_json::json!([]))
                        } else {
                            geometry_input(30.0, items, serde_json::json!([]))
                        };
                        let mut cfg = crate::config::DEFAULT_SWARM_CONFIG;
                        cfg.rng_seed = Some(41);
                        let mut engine = GroupedStepState::new(input, cfg).unwrap();
                        let lane = &mut engine.lanes[0];
                        if has_groups {
                            lane.place_groups = PlaceGroups::new(&lane.instance, vec![PlaceGroupSpec {
                                id: 42,
                                members: lane.prob.layout().placed_items().values().map(|p| PlaceGroupMember {
                                    item_id: p.item().idx(), initial_transform: p.d_transf(),
                                }).collect(),
                            }]).unwrap();
                        }
                        if has_frozen { lane.frozen_ids = vec![0]; }
                        let mut old_lane = lane.clone();
                        let mut sep_cfg = lane.config.expl_cfg.separator_config;
                        sep_cfg.n_workers = requested_workers;
                        sep_cfg.nfp_sampler = false;
                        let expected = issue7_reference_separator(&mut old_lane, sep_cfg);
                        ISSUE7_SETUP_CALLS.with(|c| c.set([0; 3]));
                        let actual = lane.separator(sep_cfg);
                        let calls = ISSUE7_SETUP_CALLS.with(|c| c.get());
                        assert_eq!(calls, [1, usize::from(has_groups), usize::from(has_frozen)],
                            "size_regions={size_regions} workers={requested_workers} groups={has_groups} frozen={has_frozen}");
                        issue7_assert_separator_equal(&expected, &actual);
                        assert_eq!(old_lane.rng.clone().random::<[u64; 4]>(), lane.rng.clone().random::<[u64; 4]>());
                        if has_groups && has_frozen {
                            assert!(actual.frozen_mask[0] && actual.frozen_mask[1]);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn issue7_native_empty_setup_preserves_legacy_donor_inheritance() {
        for phase in [LanePhase::Exploration, LanePhase::Compression,
                      LanePhase::StructureCompression, LanePhase::StructureExploration] {
            let input = size_region_geometry_input(30.0,
                vec![positioned_rectangle(0, 0.01, 0.01), positioned_rectangle(1, 12.01, 0.01)],
                serde_json::json!([]));
            let mut cfg = crate::config::DEFAULT_SWARM_CONFIG;
            cfg.rng_seed = Some(41);
            let mut engine = GroupedStepState::new(input, cfg).unwrap();
            let lane = &mut engine.lanes[0];
            lane.begin_tracker_phase(phase);
            let mut sep_cfg = lane.config.expl_cfg.separator_config;
            sep_cfg.nfp_sampler = false;
            let mut donor = lane.separator(sep_cfg);
            let keys: Vec<_> = donor.prob.layout().placed_items().keys().collect();
            let a = donor.ct.pk_idx_map[keys[0]];
            let b = donor.ct.pk_idx_map[keys[1]];
            donor.ct.container_collisions_mut()[a].weight = 3.25;
            donor.ct.container_collisions_mut()[b].weight = 4.5;
            donor.ct.pair_collisions_mut()[(a, b)].weight = 2.5;
            lane.remember_tracker(phase, &donor.ct);
            let mut old_lane = lane.clone();
            let expected = issue7_reference_separator(&mut old_lane, sep_cfg);
            ISSUE7_SETUP_CALLS.with(|c| c.set([0; 3]));
            let actual = lane.separator(sep_cfg);
            assert_eq!(ISSUE7_SETUP_CALLS.with(|c| c.get()), [1, 0, 0]);
            issue7_assert_separator_equal(&expected, &actual);
            assert_eq!(actual.ct.get_container_weight(keys[0]), 3.25);
            assert_eq!(actual.ct.get_pair_weight(keys[0], keys[1]), 2.5);
        }
    }

    #[cfg(target_os = "windows")]
    mod issue7_windows_measurement {
        use super::Separator;
        use std::ffi::c_void;
        #[repr(C)]
        #[derive(Default)]
        struct FileTime { low: u32, high: u32 }
        #[link(name = "kernel32")]
        unsafe extern "system" {
            fn GetCurrentProcess() -> *mut c_void;
            fn GetProcessTimes(process: *mut c_void, creation: *mut FileTime,
                exit: *mut FileTime, kernel: *mut FileTime, user: *mut FileTime) -> i32;
            fn GetProcessAffinityMask(process: *mut c_void, process_mask: *mut usize,
                system_mask: *mut usize) -> i32;
            fn SetProcessAffinityMask(process: *mut c_void, mask: usize) -> i32;
        }
        pub struct TwoCoreAffinity { original: usize }
        impl TwoCoreAffinity {
            pub fn enter() -> Self {
                let mut original=0usize;
                let mut system=0usize;
                let process=unsafe { GetCurrentProcess() };
                assert_ne!(unsafe { GetProcessAffinityMask(process,&mut original,&mut system) },0);
                let first=original & original.wrapping_neg();
                let remainder=original & !first;
                assert_ne!(remainder,0,"measurement requires two available CPU cores");
                let second=remainder & remainder.wrapping_neg();
                assert_ne!(unsafe { SetProcessAffinityMask(process,first|second) },0);
                Self { original }
            }
        }
        impl Drop for TwoCoreAffinity {
            fn drop(&mut self) {
                // Only this isolated test process is constrained; restore on unwind too.
                unsafe { SetProcessAffinityMask(GetCurrentProcess(),self.original); }
            }
        }
        fn cpu_seconds() -> f64 {
            let (mut creation,mut exit,mut kernel,mut user)=(FileTime::default(),
                FileTime::default(),FileTime::default(),FileTime::default());
            assert_ne!(unsafe { GetProcessTimes(GetCurrentProcess(),&mut creation,&mut exit,
                &mut kernel,&mut user) },0);
            let ticks=|t:FileTime| ((t.high as u64)<<32) | t.low as u64;
            (ticks(kernel)+ticks(user)) as f64*1.0e-7
        }
        pub fn measure(setup:impl FnOnce()->Separator)->(Separator,f64,f64) {
            let cpu_start=cpu_seconds();
            let wall_start=std::time::Instant::now();
            let separator=setup();
            let wall=wall_start.elapsed().as_secs_f64();
            let cpu=cpu_seconds()-cpu_start;
            (separator,cpu,wall)
        }
    }

    #[cfg(target_os = "windows")]
    #[test]
    #[ignore = "bounded Windows setup measurement; run alone with Rayon2 and serialized tests"]
    fn issue7_measure_fresh_empty_setup_cpu_windows() {
        assert_eq!(std::env::var("RAYON_NUM_THREADS").unwrap(),"2");
        let _affinity=issue7_windows_measurement::TwoCoreAffinity::enter();
        for n in [16,64,156] {
            let items=(0..n).map(|id|positioned_rectangle(id as u64,
                0.01+12.0*id as f32,0.01)).collect();
            let input=geometry_input(30.0,items,serde_json::json!([]));
            let mut cfg=crate::config::DEFAULT_SWARM_CONFIG;
            cfg.rng_seed=Some(41);
            let engine=GroupedStepState::new(input,cfg).unwrap();
            let template=&engine.lanes[0];
            assert!(!template.force_single_worker);
            assert_eq!(template.prob.layout().placed_items().len(),n);
            assert!(template.place_groups.is_empty() && template.frozen_ids.is_empty());
            for workers in [1,2] {
                let mut sep_cfg=template.config.expl_cfg.separator_config;
                sep_cfg.n_workers=workers;
                sep_cfg.nfp_sampler=false; // isolate tracker setup, not NFP preparation
                let mut old_cpu=0.0; let mut new_cpu=0.0;
                let mut old_wall=0.0; let mut new_wall=0.0;
                // Warm both constructors before measuring the paired samples.
                let mut warm_old=template.clone();
                let mut warm_new=template.clone();
                drop(issue7_reference_separator(&mut warm_old,sep_cfg));
                drop(warm_new.separator(sep_cfg));
                for repeat in 0..24 {
                    let mut old_lane=template.clone();
                    let mut new_lane=template.clone();
                    let (old,new)=if repeat%2==0 {
                        (issue7_windows_measurement::measure(||issue7_reference_separator(&mut old_lane,sep_cfg)),
                         issue7_windows_measurement::measure(||new_lane.separator(sep_cfg)))
                    } else {
                        let new=issue7_windows_measurement::measure(||new_lane.separator(sep_cfg));
                        let old=issue7_windows_measurement::measure(||issue7_reference_separator(&mut old_lane,sep_cfg));
                        (old,new)
                    };
                    // Validation/cloning/import/drop are outside the measured call.
                    issue7_assert_separator_equal(&old.0,&new.0);
                    assert_eq!(old_lane.rng.clone().random::<[u64;4]>(),new_lane.rng.clone().random::<[u64;4]>());
                    old_cpu+=old.1; new_cpu+=new.1;
                    old_wall+=old.2; new_wall+=new.2;
                }
                println!("ISSUE7_SETUP_CPU n={n} workers={workers} repetitions=24 old_cpu_s={old_cpu:.9} new_cpu_s={new_cpu:.9} old_wall_s={old_wall:.9} new_wall_s={new_wall:.9}");
            }
        }
        // No timing threshold: process CPU quantization and thread startup noise
        // cannot become a functional pass/fail criterion or a yield claim.
    }

    fn positioned_rectangle(id: u64, x: f32, y: f32) -> serde_json::Value {
        serde_json::json!({
            "id":id,"demand":1,"allowed_orientations":[0.0,180.0],
            "shape":{"type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":10.0,"height":10.0}},
            "placed_hint":{"item_id":id,"is_freeze":0,"tr_x":x,"tr_y":y,"rot_angle_in_rad":0.0,"stripe_interval_y":0.0,"stripe_interval_x":0.0}
        })
    }

    fn geometry_input(
        strip_height: f32,
        items: Vec<serde_json::Value>,
        groups: serde_json::Value,
    ) -> ValidatedInput {
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items","group_workers":1,"quantum_ms":5,"locked_prefix_gap_mm":0.0},
            "instance":{"name":"geometry-group","strip_height":strip_height,"items":items},
            "regions":[],
            "place_groups":groups
        })
        .to_string();
        parse_and_validate(&json).unwrap()
    }

    fn size_region_geometry_input(
        strip_height: f32,
        items: Vec<serde_json::Value>,
        groups: serde_json::Value,
    ) -> ValidatedInput {
        let item_ids = (0..items.len() as u64).collect::<Vec<_>>();
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"size_regions","group_workers":1,"quantum_ms":5,"locked_prefix_gap_mm":0.0},
            "instance":{"name":"rotatable-height-group","strip_height":strip_height,"items":items},
            "regions":[{"id":"all","kind":"size_group","item_ids":item_ids,"gap_after_mm":0.0}],
            "place_groups":groups
        })
        .to_string();
        parse_and_validate(&json).unwrap()
    }

    fn legacy_transaction_engine() -> GroupedStepState {
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items","group_workers":1,"quantum_ms":1,"locked_prefix_gap_mm":0.0},
            "instance":{"name":"legacy-phase-transaction","strip_height":30.0,"items":[
                rectangle(0,false,0.01), rectangle(1,false,12.01)
            ]},
            "regions":[],
            "place_groups":[]
        })
        .to_string();
        GroupedStepState::new(
            parse_and_validate(&json).unwrap(),
            crate::config::DEFAULT_SWARM_CONFIG,
        )
        .unwrap()
    }

    fn assert_same_solution(expected: &SPSolution, actual: &SPSolution) {
        assert_eq!(expected.strip_width(), actual.strip_width());
        assert_eq!(
            expected.layout_snapshot().placed_items().len(),
            actual.layout_snapshot().placed_items().len()
        );
        for (key, placed) in expected.layout_snapshot().placed_items().iter() {
            assert_eq!(
                actual.layout_snapshot().placed_items()[key].item().idx(),
                placed.item().idx()
            );
            assert_eq!(
                actual.layout_snapshot().placed_items()[key].d_transf(),
                placed.d_transf()
            );
        }
    }

    fn mutate_legacy_transaction_state(state: &mut GroupedStepState) {
        state.phase = GroupPhase::Polish;
        let lane = &mut state.lanes[0];
        let keys = lane
            .solution
            .layout_snapshot()
            .placed_items()
            .keys()
            .collect::<Vec<_>>();
        let first_transform = lane.solution.layout_snapshot().placed_items()[keys[0]].d_transf();
        lane.solution.layout_snapshot_mut().placed_items_mut()[keys[1]].set_d_transf(first_transform);
        lane.prob.change_strip_width(lane.prob.strip_width() + 17.0).expect(swarm_core::dmsg!("strip width must be positive"));
        let _: u64 = lane.rng.random();
    }

    fn assert_legacy_transaction_restored(
        state: &mut GroupedStepState,
        expected_phase: GroupPhase,
        expected_solution: &SPSolution,
        expected_problem: &SPSolution,
        expected_next_rng: u64,
    ) {
        assert_eq!(state.phase, expected_phase);
        assert_same_solution(expected_solution, &state.lanes[0].solution);
        assert_same_solution(expected_problem, &state.lanes[0].prob.save());
        assert_eq!(state.lanes[0].rng.random::<u64>(), expected_next_rng);
    }

    #[test]
    fn locked_prefix_and_region_gap_are_merged_once() {
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"size_regions","group_workers":1,"quantum_ms":5,"locked_prefix_gap_mm":7.0},
            "instance":{"name":"g","strip_height":30.0,"items":[
                rectangle(0,true,5.0), rectangle(1,false,20.0), rectangle(2,false,30.0)
            ]},
            "regions":[
                {"id":"s","kind":"size_group","item_ids":[0,1],"gap_after_mm":3.0},
                {"id":"rest","kind":"remainder","item_ids":[2],"gap_after_mm":0.0}
            ],
            "place_groups":[]
        }).to_string();
        let input = parse_and_validate(&json).unwrap();
        let engine = GroupedStepState::new(input, crate::config::DEFAULT_SWARM_CONFIG).unwrap();
        let solution = engine.export_solution();
        assert_eq!(solution.layout.placed_items.len(), 3);
        assert!(solution.strip_width >= 5.0 + 10.0 + 7.0 + 10.0 + 3.0 + 10.0);
    }

    #[cfg(feature = "structure-first")]
    #[test]
    fn structure_first_keeps_motifs_lane_local_and_region_gap_external() {
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"size_regions","group_workers":1,"quantum_ms":1,"locked_prefix_gap_mm":0.0},
            "instance":{"name":"lane-local","strip_height":30.0,"items":[
                rectangle(0,false,0.0), rectangle(1,false,0.0),
                rectangle(2,false,0.0), rectangle(3,false,0.0)
            ]},
            "regions":[
                {"id":"sm","kind":"size_group","item_ids":[0,1],"gap_after_mm":6.0},
                {"id":"rest","kind":"remainder","item_ids":[2,3],"gap_after_mm":0.0}
            ],
            "place_groups":[]
        })
        .to_string();
        let input = parse_and_validate(&json).unwrap();
        let mut engine = GroupedStepState::new(input, fast_structure_config()).unwrap();
        engine.run_structure(&AtomicBool::new(false)).unwrap();

        let report: serde_json::Value =
            serde_json::from_str(&engine.structure_report_json().unwrap()).unwrap();
        assert_eq!(report["schema_version"], 3);
        assert_eq!(report["group_workers"], 1);
        assert_eq!(report["quantum_ms"], 1);
        assert_eq!(report["lanes"].as_array().unwrap().len(), 2);
        assert_eq!(
            report["lanes"][0]["global_item_ids"],
            serde_json::json!([0, 1])
        );
        assert_eq!(
            report["lanes"][1]["global_item_ids"],
            serde_json::json!([2, 3])
        );
        assert_eq!(report["lanes"][0]["p0_p1_soft_structure_applied"], true);
        assert_eq!(report["lanes"][1]["p0_p1_soft_structure_applied"], true);
        assert_schema_v3_lane_invariants(&report["lanes"][0]);
        assert_schema_v3_lane_invariants(&report["lanes"][1]);
        assert_eq!(
            report["lanes"][0]["report"]["construction_starts_requested"],
            1
        );
        assert_eq!(
            report["lanes"][0]["report"]["p1_start_stats"]
                .as_array()
                .unwrap()
                .len(),
            1
        );
        assert!(engine.export_solution().strip_width >= 26.0 - 1.0e-3);
    }

    #[cfg(feature = "structure-first")]
    #[test]
    fn structure_p1_reports_baseline_and_each_completed_lane() {
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"size_regions","group_workers":2,"quantum_ms":1,"locked_prefix_gap_mm":0.0},
            "instance":{"name":"p1-progress","strip_height":30.0,"items":[
                rectangle(0,false,0.0), rectangle(1,false,0.0),
                rectangle(2,false,0.0), rectangle(3,false,0.0),
                rectangle(4,false,0.0), rectangle(5,false,0.0)
            ]},
            "regions":[
                {"id":"s","kind":"size_bundle","item_ids":[0,1],"gap_after_mm":2.0},
                {"id":"m","kind":"size_bundle","item_ids":[2,3],"gap_after_mm":2.0},
                {"id":"l","kind":"size_bundle","item_ids":[4,5],"gap_after_mm":0.0}
            ],
            "place_groups":[]
        })
        .to_string();
        let input = parse_and_validate(&json).unwrap();
        let mut engine = GroupedStepState::new(input, fast_structure_config()).unwrap();
        let mut milestone_events = vec![];
        let mut candidate_previews = 0;

        engine
            .run_structure_with_progress_callback(
                &AtomicBool::new(false),
                &mut |stage, kind, state| {
                    if stage == StructureStage::P1Construct {
                        let ready_lanes = state
                            .lanes
                            .iter()
                            .filter(|lane| lane.structure_report.is_some())
                            .count();
                        assert_eq!(state.export_solution().layout.placed_items.len(), 6);
                        if kind == StructureProgressKind::CandidatePreview {
                            candidate_previews += 1;
                        } else {
                            milestone_events.push((kind, ready_lanes));
                        }
                    }
                },
            )
            .unwrap();

        assert_eq!(
            milestone_events,
            vec![
                (StructureProgressKind::InitialBaseline, 0),
                (StructureProgressKind::LaneCompleted, 1),
                (StructureProgressKind::LaneCompleted, 2),
                (StructureProgressKind::StageCompleted, 3),
            ]
        );
        assert!(candidate_previews >= 1);
    }

    #[cfg(feature = "structure-first")]
    #[test]
    fn structure_p1_uses_one_deadline_and_round_robins_construction_starts() {
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"size_regions","group_workers":1,"quantum_ms":1,"locked_prefix_gap_mm":0.0},
            "instance":{"name":"p1-shared-deadline","strip_height":30.0,"items":[
                rectangle(0,false,0.0), rectangle(1,false,0.0),
                rectangle(2,false,0.0), rectangle(3,false,0.0),
                rectangle(4,false,0.0), rectangle(5,false,0.0),
                rectangle(6,false,0.0), rectangle(7,false,0.0)
            ]},
            "regions":[
                {"id":"s","kind":"size_bundle","item_ids":[0,1],"gap_after_mm":1.0},
                {"id":"m","kind":"size_bundle","item_ids":[2,3],"gap_after_mm":1.0},
                {"id":"l","kind":"size_bundle","item_ids":[4,5],"gap_after_mm":1.0},
                {"id":"xl","kind":"size_bundle","item_ids":[6,7],"gap_after_mm":0.0}
            ],
            "place_groups":[]
        })
        .to_string();
        let input = parse_and_validate(&json).unwrap();
        let mut config = fast_structure_config();
        config.structure.motif.enabled = false;
        config.structure.construction_starts = 100_000;
        config.structure.p1_time_limit = Duration::from_millis(150);
        config.structure.p2_time_limit = Duration::ZERO;
        config.structure.p3_time_limit = Some(Duration::ZERO);
        config.polish_cfg.time_limit = Duration::ZERO;
        let mut engine = GroupedStepState::new(input, config).unwrap();
        let started = Instant::now();
        let mut p1_elapsed = None;

        engine
            .run_structure_with_progress_callback(&AtomicBool::new(false), &mut |stage, kind, _| {
                if stage == StructureStage::P1Construct
                    && kind == StructureProgressKind::StageCompleted
                {
                    p1_elapsed = Some(started.elapsed());
                }
            })
            .unwrap();

        // A per-lane 150 ms deadline would take roughly 600 ms for four lanes.
        // Keep generous headroom for loaded Windows test hosts while still
        // detecting accidental budget multiplication.
        assert!(
            p1_elapsed.unwrap() < Duration::from_millis(400),
            "P1 exceeded its shared phase deadline by a per-lane multiple"
        );
        let report: serde_json::Value =
            serde_json::from_str(&engine.structure_report_json().unwrap()).unwrap();
        let attempts = report["lanes"]
            .as_array()
            .unwrap()
            .iter()
            .map(|lane| {
                assert_eq!(lane["report"]["p1_deadline_exhausted"], true);
                lane["report"]["construction_starts_attempted"]
                    .as_u64()
                    .unwrap()
            })
            .collect::<Vec<_>>();
        assert!(attempts.iter().sum::<u64>() > 0);
        assert!(
            attempts.iter().max().unwrap() - attempts.iter().min().unwrap() <= 1,
            "one worker must give every lane at most one construction start per round: {attempts:?}"
        );
    }

    #[cfg(feature = "structure-first")]
    #[test]
    fn structure_first_starts_after_locked_prefix_and_reports_required_spacing() {
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"size_regions","group_workers":2,"quantum_ms":1,"locked_prefix_gap_mm":7.0},
            "instance":{"name":"locked-structure","strip_height":30.0,"items":[
                rectangle(0,true,40.0), rectangle(1,false,0.0), rectangle(2,false,0.0)
            ]},
            "regions":[
                {"id":"s","kind":"size_group","item_ids":[0,1],"gap_after_mm":3.0},
                {"id":"rest","kind":"remainder","item_ids":[2],"gap_after_mm":0.0}
            ],
            "place_groups":[]
        })
        .to_string();
        let input = parse_and_validate(&json).unwrap();
        let mut engine = GroupedStepState::new(input, fast_structure_config()).unwrap();
        engine.run_structure(&AtomicBool::new(false)).unwrap();
        let solution = engine.export_solution();
        let locked = solution
            .layout
            .placed_items
            .iter()
            .find(|placement| placement.item_id == 0)
            .unwrap();
        assert!((locked.transformation.translation.0 - 40.0).abs() < 1.0e-3);
        assert!(solution.strip_width >= 50.0 + 7.0 + 10.0 + 3.0 + 10.0 - 1.0e-3);

        let report: serde_json::Value =
            serde_json::from_str(&engine.structure_report_json().unwrap()).unwrap();
        assert_eq!(report["fixed_prefix_item_count"], 1);
        assert_eq!(report["locked_prefix_gap_mm"], 7.0);
        assert_eq!(report["required_region_gap_mm"], 3.0);
        assert_eq!(report["lane_count"], 2);
        assert_schema_v3_lane_invariants(&report["lanes"][0]);
        assert_schema_v3_lane_invariants(&report["lanes"][1]);
    }

    #[cfg(feature = "structure-first")]
    #[test]
    fn structure_first_place_group_fallback_preserves_rigid_relative_pose() {
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items","group_workers":1,"quantum_ms":1,"locked_prefix_gap_mm":0.0},
            "instance":{"name":"rigid-structure","strip_height":40.0,"items":[
                positioned_rectangle(0,0.01,0.01), positioned_rectangle(1,12.01,0.01),
                positioned_rectangle(2,0.01,15.0), positioned_rectangle(3,12.0,15.0)
            ]},
            "regions":[],
            "place_groups":[{"id":42,"member_item_ids":[0,1]}]
        })
        .to_string();
        let input = parse_and_validate(&json).unwrap();
        let mut engine = GroupedStepState::new(input, {
            // the P0/P1 fallback below is specific to rigid groups
            let mut config = fast_structure_config();
            config.place_group_mode = crate::config::PlaceGroupMode::Rigid;
            config
        })
        .unwrap();
        engine.run_structure(&AtomicBool::new(false)).unwrap();
        let solution = engine.export_public_solution();
        let first = solution
            .layout
            .placed_items
            .iter()
            .find(|placement| placement.item_id == 0)
            .unwrap();
        let second = solution
            .layout
            .placed_items
            .iter()
            .find(|placement| placement.item_id == 1)
            .unwrap();
        let dx = second.transformation.translation.0 - first.transformation.translation.0;
        let dy = second.transformation.translation.1 - first.transformation.translation.1;
        assert!(((dx * dx + dy * dy).sqrt() - 12.0).abs() < 1.0e-3);
        assert!(
            lane_angular_distance(
                first.transformation.rotation,
                second.transformation.rotation
            ) < 1.0e-4
        );

        let report: serde_json::Value =
            serde_json::from_str(&engine.structure_report_json().unwrap()).unwrap();
        assert_eq!(report["lanes"][0]["p0_p1_soft_structure_applied"], false);
        assert!(
            report["lanes"][0]["fallback_reason"]
                .as_str()
                .unwrap()
                .contains("user_place_groups_active")
        );
        assert_schema_v3_lane_invariants(&report["lanes"][0]);
        let lane_report = &report["lanes"][0]["report"];
        assert_eq!(lane_report["motif_generation_stats"]["input_parts"], 0);
        assert_eq!(lane_report["motif_selection_stats"]["covered_members"], 0);
        assert!(lane_report["p1_start_stats"].as_array().unwrap().is_empty());
        assert_eq!(lane_report["construction_starts_requested"], 0);
        assert_eq!(
            lane_report["p1_control_candidate"]["source"],
            "grouped_lane_baseline"
        );
        assert_eq!(lane_report["p1_winner"]["source"], "grouped_lane_baseline");
    }


    #[cfg(feature = "structure-first")]
    #[test]
    fn structure_first_runs_with_multipolygon_place_groups_and_keeps_member_poses() {
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items","group_workers":1,"quantum_ms":1,"locked_prefix_gap_mm":0.0},
            "instance":{"name":"multipart-structure","strip_height":40.0,"items":[
                positioned_rectangle(0,0.01,0.01), positioned_rectangle(1,12.01,0.01),
                positioned_rectangle(2,0.01,15.0), positioned_rectangle(3,12.0,15.0)
            ]},
            "regions":[],
            "place_groups":[{"id":42,"member_item_ids":[0,1]}]
        })
        .to_string();
        let input = parse_and_validate(&json).unwrap();
        // Default mode: the group is one multipart item, so structure-first is not
        // disabled by user place groups and the public result still lists members.
        let mut engine = GroupedStepState::new(input, fast_structure_config()).unwrap();
        assert!(engine.expansion().is_some());
        engine.run_structure(&AtomicBool::new(false)).unwrap();
        let solution = engine.export_public_solution();
        let mut ids: Vec<u64> = solution.layout.placed_items.iter().map(|p| p.item_id).collect();
        ids.sort_unstable();
        assert_eq!(ids, vec![0, 1, 2, 3]);
        let first = solution.layout.placed_items.iter().find(|p| p.item_id == 0).unwrap();
        let second = solution.layout.placed_items.iter().find(|p| p.item_id == 1).unwrap();
        let dx = second.transformation.translation.0 - first.transformation.translation.0;
        let dy = second.transformation.translation.1 - first.transformation.translation.1;
        assert!(((dx * dx + dy * dy).sqrt() - 12.0).abs() < 1.0e-3);
        assert!(lane_angular_distance(first.transformation.rotation, second.transformation.rotation) < 1.0e-4);
        let report: serde_json::Value =
            serde_json::from_str(&engine.structure_report_json().unwrap()).unwrap();
        assert!(
            !report["lanes"][0]["fallback_reason"]
                .as_str()
                .unwrap_or("")
                .contains("user_place_groups_active"),
            "{report}"
        );
    }
    #[cfg(feature = "structure-first")]
    #[test]
    fn structure_first_frozen_all_items_lane_uses_empty_baseline_metrics() {
        let mut frozen = rectangle(0, true, 5.0);
        let mut movable = rectangle(1, false, 20.0);
        frozen["placed_hint"]["tr_y"] = serde_json::json!(0.01);
        movable["placed_hint"]["tr_y"] = serde_json::json!(0.01);
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items","group_workers":1,"quantum_ms":1,"locked_prefix_gap_mm":0.0},
            "instance":{"name":"frozen-structure","strip_height":30.0,"items":[
                frozen, movable
            ]},
            "regions":[],
            "place_groups":[]
        })
        .to_string();
        let input = parse_and_validate(&json).unwrap();
        let mut engine = GroupedStepState::new(input, fast_structure_config()).unwrap();
        engine.run_structure(&AtomicBool::new(false)).unwrap();
        let report: serde_json::Value =
            serde_json::from_str(&engine.structure_report_json().unwrap()).unwrap();
        let lane = &report["lanes"][0];
        assert_eq!(lane["p0_p1_soft_structure_applied"], false);
        assert!(
            lane["fallback_reason"]
                .as_str()
                .unwrap()
                .contains("frozen_items_active")
        );
        assert_schema_v3_lane_invariants(lane);
        assert_eq!(lane["report"]["motif_generation_stats"]["input_parts"], 0);
        assert!(
            lane["report"]["p1_start_stats"]
                .as_array()
                .unwrap()
                .is_empty()
        );
        assert_eq!(
            lane["report"]["p1_winner"]["source"],
            "grouped_lane_baseline"
        );
        let locked = engine
            .export_solution()
            .layout
            .placed_items
            .into_iter()
            .find(|placement| placement.item_id == 0)
            .unwrap();
        assert!((locked.transformation.translation.0 - 5.0).abs() < 1.0e-3);
    }

    #[test]
    fn structure_first_accepts_colliding_frozen_baseline() {
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items","group_workers":1,"quantum_ms":1,"locked_prefix_gap_mm":0.0},
            "instance":{"name":"invalid-frozen-structure","strip_height":30.0,"items":[
                rectangle(0,true,5.0), rectangle(1,true,5.0)
            ]},
            "regions":[],
            "place_groups":[]
        })
        .to_string();
        let input = parse_and_validate(&json).unwrap();
        let engine = GroupedStepState::new(input, fast_structure_config()).unwrap();
        assert!(engine.export_certified_solution().is_ok());
    }

    #[test]
    fn all_items_repairs_overlapping_movable_warm_hints_before_init_export() {
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items","group_workers":1,"quantum_ms":1,"locked_prefix_gap_mm":0.0},
            "instance":{"name":"repair-movable-hints","strip_height":30.0,"items":[
                rectangle(0,false,5.0), rectangle(1,false,5.0)
            ]},
            "regions":[],
            "place_groups":[]
        })
        .to_string();
        let input = parse_and_validate(&json).unwrap();
        let engine = GroupedStepState::new(input, fast_structure_config()).unwrap();
        assert_eq!(engine.lanes.len(), 1);
        assert!(
            solution_validation_error(&engine.lanes[0].instance, &engine.lanes[0].solution)
                .is_none(),
            "Init must never expose an overlapping movable warm-start baseline"
        );
    }

    #[test]
    fn legacy_phase_transaction_restores_lane_and_rng_after_runner_error() {
        let mut engine = legacy_transaction_engine();
        let expected_phase = engine.phase;
        let expected_solution = engine.lanes[0].solution.clone();
        let expected_problem = engine.lanes[0].prob.save();
        let mut expected_rng = engine.lanes[0].rng.clone();
        let expected_next_rng = expected_rng.random::<u64>();

        let error = engine
            .run_certified_legacy_phase_transaction(
                GroupPhase::Exploration,
                "injected boundary",
                |state| {
                    mutate_legacy_transaction_state(state);
                    Err("injected scheduler failure".to_string())
                },
            )
            .unwrap_err();

        assert_eq!(error, "injected scheduler failure");
        assert_legacy_transaction_restored(
            &mut engine,
            expected_phase,
            &expected_solution,
            &expected_problem,
            expected_next_rng,
        );
    }

    #[test]
    fn legacy_phase_transaction_restores_lane_and_rng_after_boundary_rejection() {
        let mut engine = legacy_transaction_engine();
        let expected_phase = engine.phase;
        let expected_solution = engine.lanes[0].solution.clone();
        let expected_problem = engine.lanes[0].prob.save();
        let mut expected_rng = engine.lanes[0].rng.clone();
        let expected_next_rng = expected_rng.random::<u64>();

        let error = engine
            .run_certified_legacy_phase_transaction(
                GroupPhase::Exploration,
                "injected boundary",
                |state| {
                    mutate_legacy_transaction_state(state);
                    Ok(())
                },
            )
            .unwrap_err();

        assert!(
            error.starts_with("injected boundary: merged public solution is not exact-feasible")
        );
        assert_legacy_transaction_restored(
            &mut engine,
            expected_phase,
            &expected_solution,
            &expected_problem,
            expected_next_rng,
        );
    }

    #[test]
    fn legacy_phase_transaction_restores_lane_and_rng_before_resuming_panic() {
        let mut engine = legacy_transaction_engine();
        let expected_phase = engine.phase;
        let expected_solution = engine.lanes[0].solution.clone();
        let expected_problem = engine.lanes[0].prob.save();
        let mut expected_rng = engine.lanes[0].rng.clone();
        let expected_next_rng = expected_rng.random::<u64>();

        let panic = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _ = engine.run_certified_legacy_phase_transaction(
                GroupPhase::Exploration,
                "injected boundary",
                |state| {
                    mutate_legacy_transaction_state(state);
                    panic!("{}", swarm_core::dmsg!("injected scheduler panic"));
                },
            );
        }));

        assert!(
            panic.is_err(),
            "the original panic must reach the FFI catcher"
        );
        assert_legacy_transaction_restored(
            &mut engine,
            expected_phase,
            &expected_solution,
            &expected_problem,
            expected_next_rng,
        );
    }

    #[test]
    fn structure_transaction_restores_all_public_state_after_worker_error() {
        let mut engine = legacy_transaction_engine();
        let expected_phase = engine.phase;
        let expected_solution = engine.lanes[0].solution.clone();
        let expected_problem = engine.lanes[0].prob.save();
        let mut expected_rng = engine.lanes[0].rng.clone();
        let expected_next_rng = expected_rng.random::<u64>();
        engine.structure_public_incumbents = Some(vec![expected_solution.clone()]);

        let error = engine
            .run_structure_transaction(|state| {
                mutate_legacy_transaction_state(state);
                state.structure_public_incumbents = None;
                Err("injected Structure worker failure".to_string())
            })
            .unwrap_err();

        assert_eq!(error, "injected Structure worker failure");
        assert_legacy_transaction_restored(
            &mut engine,
            expected_phase,
            &expected_solution,
            &expected_problem,
            expected_next_rng,
        );
        let restored_public = engine
            .structure_public_incumbents
            .as_ref()
            .expect(swarm_core::dmsg!("the certified public incumbent set must be restored"));
        assert_eq!(restored_public.len(), 1);
        assert_same_solution(&expected_solution, &restored_public[0]);
    }

    #[test]
    fn structure_transaction_restores_all_public_state_before_resuming_panic() {
        let mut engine = legacy_transaction_engine();
        let expected_phase = engine.phase;
        let expected_solution = engine.lanes[0].solution.clone();
        let expected_problem = engine.lanes[0].prob.save();
        let mut expected_rng = engine.lanes[0].rng.clone();
        let expected_next_rng = expected_rng.random::<u64>();
        engine.structure_public_incumbents = Some(vec![expected_solution.clone()]);

        let panic = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _ = engine.run_structure_transaction(|state| {
                mutate_legacy_transaction_state(state);
                state.structure_public_incumbents = None;
                panic!("{}", swarm_core::dmsg!("injected Structure worker panic"));
            });
        }));

        assert!(
            panic.is_err(),
            "the original panic must reach the FFI catcher"
        );
        assert_legacy_transaction_restored(
            &mut engine,
            expected_phase,
            &expected_solution,
            &expected_problem,
            expected_next_rng,
        );
        let restored_public = engine
            .structure_public_incumbents
            .as_ref()
            .expect(swarm_core::dmsg!("the certified public incumbent set must be restored"));
        assert_eq!(restored_public.len(), 1);
        assert_same_solution(&expected_solution, &restored_public[0]);
    }

    #[test]
    fn public_candidates_reject_lock_or_rigid_pose_drift_even_without_collisions() {
        for frozen in [true, false] {
            let json=serde_json::json!({"schema_version":2,
                "nesting":{"mode":"all_items","group_workers":1,"quantum_ms":1,"locked_prefix_gap_mm":0.0},
                "instance":{"name":"constraint-drift","strip_height":30.0,"items":[rectangle(0,frozen,0.01),rectangle(1,false,12.01)]},
                "regions":[],"place_groups":if frozen {serde_json::json!([])} else {serde_json::json!([{"id":7,"member_item_ids":[0,1]}])}}).to_string();
            let mut engine =
                GroupedStepState::new(parse_and_validate(&json).unwrap(), {
                    // rigid-group pose drift only exists in the rigid mode
                    let mut config = fast_structure_config();
                    config.place_group_mode = crate::config::PlaceGroupMode::Rigid;
                    config
                })
                    .unwrap();
            let baseline = engine.lanes[0].solution.clone();
            if !frozen {
                let mut rigid = baseline.clone();
                let delta = DTransformation::new(0.37, (4.0, 3.0)).compose();
                for pi in rigid.layout_snapshot_mut().placed_items_mut().values_mut() {
                    pi.set_d_transf(pi.d_transf().compose().transform(&delta).decompose());
                }
                assert!(
                    engine.lanes[0].constraints_valid(&rigid),
                    "a common rigid delta must remain allowed"
                );
                let first = rigid
                    .layout_snapshot_mut()
                    .placed_items_mut()
                    .values_mut()
                    .next()
                    .unwrap();
                first.set_d_transf(DTransformation::new(f32::INFINITY, (0.0, 0.0)));
                assert!(
                    !engine.lanes[0].constraints_valid(&rigid),
                    "nonfinite poses must be rejected without matrix panics"
                );
            }
            let mut drifted = baseline.clone();
            let p = drifted
                .layout_snapshot_mut()
                .placed_items_mut()
                .values_mut()
                .find(|p| p.item().idx() == 0)
                .unwrap();
            let (x, y) = p.d_transf().translation();
            p.set_d_transf(DTransformation::new(p.d_transf().rotation(), (x + 1.0, y)));
            assert!(solution_validation_error(&engine.lanes[0].instance, &drifted).is_none());
            engine.lanes[0].solution = drifted.clone();
            assert!(
                engine.export_certified_solution().is_err(),
                "frozen={frozen}: drifted pose exported"
            );
            let lane = &mut engine.lanes[0];
            lane.solution = baseline.clone();
            lane.structure_incumbent = Some(baseline.clone());
            let (exact, improved) =
                lane.apply_structure_candidate(drifted, StructureStage::P4Refine);
            assert!(!exact && !improved);
            assert_eq!(
                lane.solution
                    .layout_snapshot()
                    .placed_items()
                    .values()
                    .map(|p| p.d_transf())
                    .collect::<Vec<_>>(),
                baseline
                    .layout_snapshot()
                    .placed_items()
                    .values()
                    .map(|p| p.d_transf())
                    .collect::<Vec<_>>()
            );
        }
    }

    #[test]
    fn structure_candidate_rejects_invalid_geometry_and_restores_exact_incumbent() {
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items","group_workers":1,"quantum_ms":1,"locked_prefix_gap_mm":0.0},
            "instance":{"name":"structure-candidate-rollback","strip_height":30.0,"items":[
                rectangle(0,false,0.01), rectangle(1,false,12.01)
            ]},
            "regions":[],
            "place_groups":[]
        })
        .to_string();
        let input = parse_and_validate(&json).unwrap();
        let mut engine = GroupedStepState::new(input, fast_structure_config()).unwrap();
        let lane = &mut engine.lanes[0];
        let incumbent = lane.solution.clone();
        lane.structure_incumbent = Some(incumbent.clone());

        let mut invalid = incumbent.clone();
        let keys = invalid
            .layout_snapshot()
            .placed_items()
            .keys()
            .collect::<Vec<_>>();
        let first_transform = invalid.layout_snapshot().placed_items()[keys[0]].d_transf();
        invalid.layout_snapshot_mut().placed_items_mut()[keys[1]].set_d_transf(first_transform);
        let (exact, improved) = lane.apply_structure_candidate(invalid, StructureStage::P4Refine);

        assert!(!exact);
        assert!(!improved);
        assert!(solution_validation_error(&lane.instance, &lane.solution).is_none());
        for (key, placed) in incumbent.layout_snapshot().placed_items().iter() {
            assert_eq!(
                lane.solution.layout_snapshot().placed_items()[key].d_transf(),
                placed.d_transf()
            );
        }
    }

    #[cfg(feature = "structure-first")]
    #[test_case::test_case(false; "repair_off")]
    #[test_case::test_case(true; "repair_on")]
    fn grouped_p3_seed_is_one_shot_and_p4_stops_after_stagnation(repair_enabled: bool) {
        let mut items = [
            rectangle(0, false, 0.01),
            rectangle(1, false, 10.02),
            rectangle(2, false, 20.03),
            rectangle(3, false, 30.04),
        ];
        for item in &mut items {
            item["placed_hint"]["tr_y"] = serde_json::json!(0.01);
        }
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items","group_workers":1,"quantum_ms":1,"locked_prefix_gap_mm":0.0},
            "instance":{"name":"grouped-structure-stagnation","strip_height":10.02,"items":items},
            "regions":[],
            "place_groups":[]
        })
        .to_string();
        let input = parse_and_validate(&json).unwrap();
        let mut config = fast_structure_config();
        config.expl_cfg.repair.direction_enabled = repair_enabled;
        config.expl_cfg.repair.cluster_lp_enabled = repair_enabled;
        config.expl_cfg.repair.chain_enabled = repair_enabled;
        config.expl_cfg.repair.stagnation_iterations = 1;
        config.expl_cfg.separator_config.sample_config.tilt_range = 0.35;
        config.structure.p2_time_limit = Duration::from_millis(3);
        config.structure.p3_time_limit = Some(Duration::from_millis(5));
        config.structure.p3_slack_ratio = 0.05;
        config.structure.p4_cycles = 4;
        config.structure.p4_cycle_time_limit = Duration::from_millis(10);
        config.structure.p4_stagnation_cycles = 1;
        config.structure.p4_ruin_strategy = RuinStrategy::Rightmost;
        config.polish_cfg.time_limit = Duration::ZERO;
        let mut engine = GroupedStepState::new(input, config).unwrap();

        engine.run_structure(&AtomicBool::new(false)).unwrap();

        let lane = &engine.lanes[0];
        assert!(!lane.structure_p3_repair_pending);
        let repair_count = lane.structure_report.as_ref().unwrap().repair_cycles.len();
        assert!(
            (1..config.structure.p4_cycles).contains(&repair_count),
            "repair_count={repair_count}"
        );
        assert!(engine.export_certified_solution().is_ok());
        assert!(solution_validation_error(&lane.instance, &lane.solution).is_none());
    }

    #[test]
    fn size_region_rejects_off_angle_frozen_prefix_during_init() {
        let mut frozen = rectangle(0, true, 5.0);
        frozen["allowed_orientations"] = serde_json::json!([0.0]);
        frozen["placed_hint"]["rot_angle_in_rad"] = serde_json::json!(std::f32::consts::FRAC_PI_2);
        frozen["placed_hint"]["tr_y"] = serde_json::json!(10.0);
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"size_regions","group_workers":1,"quantum_ms":1,"locked_prefix_gap_mm":0.0},
            "instance":{"name":"off-angle-prefix","strip_height":30.0,"items":[frozen]},
            "regions":[{"id":"s","kind":"size_bundle","item_ids":[0],"gap_after_mm":0.0}],
            "place_groups":[]
        })
        .to_string();
        let input = parse_and_validate(&json).unwrap();
        let error = GroupedStepState::new(input, fast_structure_config())
            .err()
            .expect(swarm_core::dmsg!("off-angle frozen prefix must fail during Init"));
        assert!(error.contains("rotation outside its allowed orientations"));
    }

    #[test]
    fn far_right_locked_item_keeps_its_bounds_without_quadtree_failure() {
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"size_regions","group_workers":1,"quantum_ms":5,"locked_prefix_gap_mm":7.0},
            "instance":{"name":"far-lock","strip_height":30.0,"items":[
                rectangle(0,true,500.0), rectangle(1,false,20.0)
            ]},
            "regions":[
                {"id":"all","kind":"size_group","item_ids":[0,1],"gap_after_mm":0.0}
            ],
            "place_groups":[]
        }).to_string();
        let input = parse_and_validate(&json).unwrap();
        let engine = GroupedStepState::new(input, crate::config::DEFAULT_SWARM_CONFIG).unwrap();
        let solution = engine.export_solution();
        let locked = solution
            .layout
            .placed_items
            .iter()
            .find(|placement| placement.item_id == 0)
            .unwrap();
        assert!((locked.transformation.translation.0 - 500.0).abs() < 1e-3);
        assert!(solution.strip_width >= 510.0 + 7.0 + 10.0);
    }

    #[test]
    fn all_items_exports_locked_items_only_through_its_lane() {
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items","group_workers":1,"quantum_ms":5,"locked_prefix_gap_mm":7.0},
            "instance":{"name":"all-locked","strip_height":30.0,"items":[
                rectangle(0,true,5.0), rectangle(1,false,20.0)
            ]},
            "regions":[],
            "place_groups":[]
        }).to_string();
        let input = parse_and_validate(&json).unwrap();
        let engine = GroupedStepState::new(input, crate::config::DEFAULT_SWARM_CONFIG).unwrap();
        let solution = engine.export_solution();
        let ids = solution
            .layout
            .placed_items
            .iter()
            .map(|placement| placement.item_id)
            .collect::<Vec<_>>();
        assert_eq!(ids.len(), 2);
        assert_eq!(ids, vec![0, 1]);
    }

    #[test]
    fn group_orientation_intersection_uses_hint_relative_angles() {
        // The member starts 90 degrees ahead of the anchor. Anchor 0/180 map
        // to member 90/270 respectively and are therefore the valid choices.
        let mut input = orientation_input(
            Some(vec![0.0, 90.0, 180.0, 270.0]),
            0.0,
            Some(vec![90.0, 270.0]),
            90.0,
        );
        restrict_group_orientations(&mut input).unwrap();
        assert_eq!(
            orientation_values(&input.instance.items[0].base.orientation),
            Some(vec![0.0, 180.0])
        );
    }

    #[test]
    fn discrete_member_restricts_a_continuous_anchor_with_relative_angles() {
        let mut input = orientation_input(None, 0.0, Some(vec![90.0, 270.0]), 90.0);
        restrict_group_orientations(&mut input).unwrap();
        assert_eq!(
            orientation_values(&input.instance.items[0].base.orientation),
            Some(vec![0.0, 180.0])
        );
    }

    #[test]
    fn group_without_a_relative_orientation_intersection_is_rejected() {
        let mut input = orientation_input(Some(vec![0.0]), 0.0, Some(vec![0.0]), 90.0);
        let error = restrict_group_orientations(&mut input).unwrap_err();
        assert!(error.contains("no common allowed orientation"));
    }

    #[test]
    fn overlapping_members_inside_one_place_group_are_accepted() {
        for region in [false, true] {
            let items = vec![
                positioned_rectangle(0, 0.0, 0.0),
                positioned_rectangle(1, 5.0, 0.0),
            ];
            let groups = serde_json::json!([{"id":10,"member_item_ids":[0,1]}]);
            let input = if region {
                size_region_geometry_input(40.0, items, groups)
            } else {
                geometry_input(40.0, items, groups)
            };
            let engine = GroupedStepState::new(input, crate::config::DEFAULT_SWARM_CONFIG)
                .expect(swarm_core::dmsg!("user-defined internal overlap must be accepted"));
            engine
                .export_certified_solution()
                .expect(swarm_core::dmsg!("internal overlap must certify"));
            for lane in &engine.lanes {
                assert!(
                    lane.place_groups
                        .solution_preserves_relative_poses(&lane.solution)
                );
            }
        }
    }

    #[test]
    fn place_group_vertical_span_larger_than_strip_is_rejected() {
        let input = geometry_input(
            40.0,
            vec![
                positioned_rectangle(0, 0.0, 0.0),
                positioned_rectangle(1, 12.0, 35.0),
            ],
            serde_json::json!([{"id":11,"member_item_ids":[0,1]}]),
        );
        let error = GroupedStepState::new(input, crate::config::DEFAULT_SWARM_CONFIG)
            .err()
            .expect(swarm_core::dmsg!("too-tall rigid group must fail preflight"));
        assert!(error.contains("vertical span"));
        assert!(error.contains("strip height"));
    }

    #[test]
    fn a_discrete_group_is_accepted_when_an_allowed_rotation_fits_height() {
        let mut first = positioned_rectangle(0, 0.0, 0.0);
        let mut second = positioned_rectangle(1, 0.0, 35.0);
        first["allowed_orientations"] = serde_json::json!([0.0, 90.0]);
        second["allowed_orientations"] = serde_json::json!([0.0, 90.0]);
        let input = size_region_geometry_input(
            20.0,
            vec![first, second],
            serde_json::json!([{"id":14,"member_item_ids":[0,1]}]),
        );
        GroupedStepState::new(input, crate::config::DEFAULT_SWARM_CONFIG)
            .expect(swarm_core::dmsg!("the 90-degree group orientation fits the strip"));
    }

    #[test]
    fn a_continuous_group_is_not_rejected_from_its_tall_hint_pose() {
        let mut first = positioned_rectangle(0, 0.0, 0.0);
        let mut second = positioned_rectangle(1, 0.0, 35.0);
        first["allowed_orientations"] = serde_json::Value::Null;
        second["allowed_orientations"] = serde_json::Value::Null;
        let input = size_region_geometry_input(
            20.0,
            vec![first, second],
            serde_json::json!([{"id":15,"member_item_ids":[0,1]}]),
        );
        GroupedStepState::new(input, crate::config::DEFAULT_SWARM_CONFIG)
            .expect(swarm_core::dmsg!("continuous rotation must not be rejected from the hint span alone"));
    }

    #[test]
    fn overlap_between_different_place_groups_is_not_a_preflight_error() {
        let input = geometry_input(
            40.0,
            vec![
                positioned_rectangle(0, 0.0, 0.0),
                positioned_rectangle(1, 12.0, 0.0),
                positioned_rectangle(2, 0.0, 0.0),
                positioned_rectangle(3, 12.0, 0.0),
            ],
            serde_json::json!([
                {"id":12,"member_item_ids":[0,1]},
                {"id":13,"member_item_ids":[2,3]}
            ]),
        );
        GroupedStepState::new(input, crate::config::DEFAULT_SWARM_CONFIG)
            .expect(swarm_core::dmsg!("different groups may begin overlapped for separation to resolve"));
    }

    struct Issue6PollBudget(std::sync::atomic::AtomicUsize);
    impl Terminator for Issue6PollBudget {
        fn kill(&self) -> bool { self.0.fetch_add(1,Ordering::Relaxed)>=512 }
        fn stop_requested(&self) -> bool { self.0.load(Ordering::Relaxed)>512 }
        fn new_timeout(&mut self,_:Duration) { panic!("{}", swarm_core::dmsg!("owner deadline must not reset")) }
        fn timeout_at(&self) -> Option<Instant> { None }
    }
    #[test]
    fn issue6_grouped_real_ordering_dispatch_exits_for_single_item() {
        // Actual ordinary lane dispatch: adaptive/upstream burns the poll budget;
        // ordering reconstructs and certifies the real single-item solution then exits.
        for region in [false,true] {
            let items=vec![positioned_rectangle(0,20.0,10.0)];
            let input=if region {size_region_geometry_input(30.0,items,serde_json::json!([]))} else {geometry_input(30.0,items,serde_json::json!([]))};
            let mut config=crate::config::DEFAULT_SWARM_CONFIG;
            config.rng_seed=Some(7);
            config.nfp_integration.ordering_exploration=true;
            config.escape_enabled=true; // Ordering has priority over escape.
            config.nfp_lbf.cache_budget_ms=0;
            config.expl_cfg.separator_config.n_workers=1;
            let mut engine=GroupedStepState::new(input,config).unwrap();
            let lane=&mut engine.lanes[0];
            let before=lane.solution.strip_width();
            let term=Issue6PollBudget(std::sync::atomic::AtomicUsize::new(0));
            lane.run_slice(LanePhase::Exploration,&term,Duration::from_secs(5)).unwrap();
            assert!(term.0.load(Ordering::Relaxed)<512,"ordinary V2 Explore ignored ordering flag: region={region}");
            assert!(lane.solution.strip_width()<=before);
            assert!(lane.candidate_is_valid(&lane.solution));
        }
    }


    #[test]
    fn issue6_actual_wider_nfp_result_retains_certified_grouped_incumbent() {
        let mut item=positioned_rectangle(0,0.0,0.0);
        item["allowed_orientations"]=serde_json::json!([0.0]);
        let input=geometry_input(30.0,vec![item],serde_json::json!([]));
        let mut config=crate::config::DEFAULT_SWARM_CONFIG;
        config.rng_seed=Some(7);config.nfp_integration.ordering_exploration=true;config.nfp_lbf.cache_budget_ms=0;
        config.expl_cfg.separator_config.n_workers=1;
        let mut engine=GroupedStepState::new(input,config).unwrap();
        let lane=&mut engine.lanes[0];let entry=lane.solution.clone();
        assert!(lane.candidate_is_valid(&entry),"boundary-packed entry must be exact-certified; fit adds its own margin");
        let mut sep=lane.separator(config.expl_cfg.separator_config);
        sep.rollback(&entry,None);
        let term=Issue6PollBudget(std::sync::atomic::AtomicUsize::new(0));
        let mut expl=config.expl_cfg;expl.time_limit=Duration::from_secs(5);
        let candidates=crate::ffi_step::select_exploration_phase(config.nfp_integration,false,&config.escape,&config.nfp_lbf,&lane.instance,&mut sep,&mut None,&term,&expl);
        let wider=lane.narrowest_valid(&candidates).unwrap();
        assert!(wider.strip_width()>entry.strip_width(),"fixture must actually return a wider exact NFP reconstruction: {}",wider.strip_width());
        let term=Issue6PollBudget(std::sync::atomic::AtomicUsize::new(0));
        lane.run_slice(LanePhase::Exploration,&term,Duration::from_secs(5)).unwrap();
        assert_same_solution(&entry,&lane.solution);
        assert!(lane.candidate_is_valid(&lane.solution));
    }

}
