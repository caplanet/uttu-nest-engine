//! Runtime controls for the experimental P0-P5 pipeline.

use crate::optimizer::lns::RuinStrategy;
use crate::structure::motif::MotifConfig;
use crate::structure::profile::ProfileScoreWeights;
use std::time::Duration;

/// Configuration for the complete "structure first, search assisted" path.
///
/// The compile-time `structure-first` feature makes the implementation
/// available. `enabled` is a second, runtime switch used by FFI/CLI callers so
/// the legacy optimiser remains the default and an exact A/B control.
#[derive(Clone, Copy, Debug)]
pub struct StructureFirstConfig {
    pub enabled: bool,

    /// Minimum wall-clock interval between streamed previews from Structure
    /// and the public Explore/Compress/Polish phases (default 2 s, floor 500 ms).
    /// Stage-boundary callbacks are always emitted regardless of this value.
    pub progress_callback_interval: Duration,

    /// P0 reversible motif discovery.
    pub motif: MotifConfig,
    /// Number of disjoint motif hypotheses admitted to any one construction.
    pub max_active_motifs: usize,

    /// P1 independent multi-start constructions.
    pub construction_starts: usize,
    /// Also evaluate one legacy Raster/LBF constructor and keep it when the
    /// experimental profile starts are worse. This is the documented P1
    /// fallback and makes fresh construction monotone against the old seed.
    pub p1_include_legacy_baseline: bool,
    /// Shared wall-clock budget for P0 motif discovery and all P1 starts.
    pub p1_time_limit: Duration,
    /// Profile row height = smallest item dimension × this ratio.
    pub profile_row_height_ratio: f32,
    /// Hard cap preventing tiny geometry from exploding the x(y) raster.
    pub profile_max_rows: usize,
    pub profile_weights: ProfileScoreWeights,
    pub profile_top_k: usize,
    /// Number of remaining item/copy choices compared per P1 placement step.
    pub item_beam_width: usize,
    /// Exact-authorized profile/raster/random proposals retained per item.
    pub candidate_pool_cap: usize,
    /// Maximum exact slide-to-contact refinement steps per axis.
    pub slide_refine_steps: usize,

    /// P2 short adaptive compression shown as the first preview.
    pub p2_time_limit: Duration,
    pub p2_shrink_range: (f32, f32),
    pub p2_failure_decay: f32,

    /// P3 exploration warm-start slack (e.g. 0.05 = +5%).
    pub p3_slack_ratio: f32,
    /// If `None`, use the normal exploration budget supplied by Swarm.
    pub p3_time_limit: Option<Duration>,

    /// P4 gap-target ruin/recreate cycles.
    pub p4_cycles: usize,
    pub p4_cycle_time_limit: Duration,
    pub p4_ruin_fraction: f32,
    pub p4_max_items: usize,
    /// Stop P4 after this many cycles without a narrower feasible incumbent.
    pub p4_stagnation_cycles: usize,
    /// Which ruin strategy the P4 repair cycles use. `DensitySeam` adds a suffix
    /// rigid-slide on top of `GapTarget` seam selection (A/B switch).
    pub p4_ruin_strategy: RuinStrategy,

    /// P5 allowed-angle snap/repair before the normal exact polish passes.
    pub p5_snap_rotations: bool,
}

impl Default for StructureFirstConfig {
    fn default() -> Self {
        DEFAULT_STRUCTURE_FIRST_CONFIG
    }
}

pub const DEFAULT_STRUCTURE_FIRST_CONFIG: StructureFirstConfig = StructureFirstConfig {
    // Safe rollout: callers must explicitly opt in.
    enabled: false,
    progress_callback_interval: Duration::from_secs(2),
    motif: MotifConfig {
        enabled: true,
        continuous_orientation_samples: 4,
        single_orientations_per_part: 4,
        top_k_singles: 256,
        pair_pool_size: 24,
        pair_orientations_per_part: 3,
        pair_raster_divisions: 12,
        pair_max_axis_cells: 28,
        pair_shortlist_per_orientation: 8,
        pair_max_offset_candidates: 256,
        pair_max_orientation_work: 2048,
        top_k_pairs: 128,
        min_band_members: 4,
        max_band_members: 16,
        band_gap: 0.0,
        family_area_tolerance: 0.05,
        family_aspect_tolerance: 0.08,
        family_rectangularity_tolerance: 0.10,
        top_k_bands: 48,
        score_weights: crate::structure::motif::MotifScoreWeights {
            envelope_fill: 0.48,
            boundary_match: 0.22,
            rectangularity: 0.12,
            repetition: 0.12,
            member_bonus: 0.06,
        },
    },
    max_active_motifs: 64,
    construction_starts: 8,
    p1_include_legacy_baseline: true,
    p1_time_limit: Duration::from_secs(10),
    profile_row_height_ratio: 0.12,
    profile_max_rows: 512,
    profile_weights: ProfileScoreWeights {
        gap: 1.0,
        contact: 0.35,
        boundary_anchor: 0.20,
        overlap: 10.0,
        contact_tolerance: 0.05,
    },
    profile_top_k: 3,
    item_beam_width: 12,
    candidate_pool_cap: 96,
    slide_refine_steps: 24,
    p2_time_limit: Duration::from_secs(5),
    p2_shrink_range: (0.003, 0.00005),
    p2_failure_decay: 0.55,
    p3_slack_ratio: 0.05,
    p3_time_limit: None,
    p4_cycles: 3,
    p4_cycle_time_limit: Duration::from_secs(5),
    p4_ruin_fraction: 0.20,
    p4_max_items: 48,
    p4_stagnation_cycles: 2,
    p4_ruin_strategy: RuinStrategy::GapTarget,
    p5_snap_rotations: true,
};

impl StructureFirstConfig {
    /// Clamp unsafe/nonsensical tuning values while keeping the API infallible
    /// at the FFI boundary.
    pub fn sanitized(mut self) -> Self {
        self.progress_callback_interval = self
            .progress_callback_interval
            .max(Duration::from_millis(500));
        self.max_active_motifs = self.max_active_motifs.max(1);
        self.construction_starts = self.construction_starts.max(1);
        if !self.profile_row_height_ratio.is_finite() || self.profile_row_height_ratio <= 0.0 {
            self.profile_row_height_ratio = 0.12;
        }
        self.profile_top_k = self.profile_top_k.clamp(1, 16);
        self.profile_max_rows = self.profile_max_rows.clamp(16, 4096);
        self.item_beam_width = self.item_beam_width.clamp(1, 64);
        self.candidate_pool_cap = self.candidate_pool_cap.clamp(8, 4096);
        self.slide_refine_steps = self.slide_refine_steps.clamp(1, 128);
        if !self.profile_weights.is_valid() {
            self.profile_weights = ProfileScoreWeights::default();
        }
        if !self.p2_shrink_range.0.is_finite()
            || !self.p2_shrink_range.1.is_finite()
            || self.p2_shrink_range.0 <= 0.0
            || self.p2_shrink_range.1 <= 0.0
        {
            self.p2_shrink_range = (0.003, 0.00005);
        }
        if self.p2_shrink_range.0 < self.p2_shrink_range.1 {
            self.p2_shrink_range = (self.p2_shrink_range.1, self.p2_shrink_range.0);
        }
        if !self.p2_failure_decay.is_finite() || !(0.0..1.0).contains(&self.p2_failure_decay) {
            self.p2_failure_decay = 0.55;
        }
        if !self.p3_slack_ratio.is_finite() || self.p3_slack_ratio < 0.0 {
            self.p3_slack_ratio = 0.05;
        }
        self.p3_slack_ratio = self.p3_slack_ratio.min(0.50);
        self.p4_ruin_fraction = if self.p4_ruin_fraction.is_finite() {
            self.p4_ruin_fraction.clamp(0.05, 0.80)
        } else {
            0.20
        };
        self.p4_max_items = self.p4_max_items.max(2);
        self.p4_stagnation_cycles = self.p4_stagnation_cycles.max(1);
        self
    }
}
