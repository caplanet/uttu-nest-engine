//! Reversible, solver-internal placement motifs used by the structure-first constructor.
//!
//! A motif is deliberately **not** a public `PlaceGroup`: it is an advisory list of
//! relative poses which may be accepted, partially used, or dissolved back into
//! independent items at any time.  Nothing in this module mutates an instance or
//! constrains a later neighbourhood move.
//!
//! Candidate generation has four independent channels:
//! - [`MotifKind::Single`] approximates every allowed orientation by its AABB.
//! - [`MotifKind::Pair`] aligns rasterised boundary cells with an in-module bounded
//!   radix-2 2D FFT, rejects coarse
//!   occupancy overlap, and then performs an exact polygon-interior overlap check.
//! - [`MotifKind::RectangleBlock`] runs a bounded bottom-left rectangle subsolver
//!   over near-rectangular copies, including mixed item types.
//! - [`MotifKind::RepeatedBand`] detects repeated/similar families and proposes
//!   horizontal, vertical, and staggered lattice layouts.
//!
//! The final constructor should still validate every translated candidate against
//! its live CDE.  [`MotifPoseValidator`] is provided so callers can also inject an
//! exact/CDE-specific preflight while the catalogue is built.

use jagua_rs::geometry::geo_enums::RotationRange;
use jagua_rs::geometry::geo_traits::Transformable;
use jagua_rs::geometry::primitives::{Point, Rect, SPolygon};
use jagua_rs::geometry::{DTransformation, Transformation, normalize_rotation};
use jagua_rs::probs::spp::entities::SPInstance;
use ordered_float::OrderedFloat;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap, HashSet};
use std::f32::consts::TAU;
use std::sync::Arc;

/// Stable identity of one demanded copy of an item type.
///
/// `item_id` is Jagua's item-type id; `copy_index` disambiguates its demand.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct MotifItemRef {
    pub item_id: usize,
    pub copy_index: usize,
}

/// Shape and orientation information used during motif discovery.
#[derive(Clone, Debug)]
pub struct MotifPart {
    pub key: MotifItemRef,
    pub shape: Arc<SPolygon>,
    /// Allowed absolute rotations in radians. Values are normalised and deduplicated.
    pub allowed_rotations: Vec<f32>,
}

impl MotifPart {
    pub fn new(
        key: MotifItemRef,
        shape: Arc<SPolygon>,
        allowed_rotations: impl IntoIterator<Item = f32>,
    ) -> Self {
        let allowed_rotations = normalise_angles(allowed_rotations);
        Self {
            key,
            shape,
            allowed_rotations: if allowed_rotations.is_empty() {
                vec![0.0]
            } else {
                allowed_rotations
            },
        }
    }
}

/// One item pose relative to the motif's lower-left envelope origin.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MotifMember {
    pub item: MotifItemRef,
    pub relative_pose: DTransformation,
}

/// A borrowed pose passed to an optional exact validator.
#[derive(Clone, Copy, Debug)]
pub struct MotifPoseRef<'a> {
    pub part: &'a MotifPart,
    pub relative_pose: DTransformation,
}

/// Hook for a caller-specific exact/CDE check.
///
/// The module has already rejected polygon-interior overlap when this is called.
/// A live-layout adapter can additionally enforce buffering, numeric tolerance,
/// container rules, or another engine's definition of contact.
pub trait MotifPoseValidator {
    fn is_feasible(&mut self, placements: &[MotifPoseRef<'_>]) -> bool;
}

#[derive(Default)]
struct AcceptAllValidator;

impl MotifPoseValidator for AcceptAllValidator {
    fn is_feasible(&mut self, _placements: &[MotifPoseRef<'_>]) -> bool {
        true
    }
}

/// Latches the first cooperative stop request so a one-shot callback cannot
/// accidentally resume later loops. A cancelled generator returns only motifs
/// which were completely constructed and validated before the request.
struct GenerationControl<'a> {
    should_stop: &'a mut dyn FnMut() -> bool,
    stopped: bool,
}

impl<'a> GenerationControl<'a> {
    fn new(should_stop: &'a mut dyn FnMut() -> bool) -> Self {
        Self {
            should_stop,
            stopped: false,
        }
    }

    fn stop_requested(&mut self) -> bool {
        if !self.stopped {
            self.stopped = (self.should_stop)();
        }
        self.stopped
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BandPattern {
    Horizontal,
    Vertical,
    StaggeredLattice,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ApproxRectangle {
    pub orientation: f32,
    pub width: f32,
    pub height: f32,
    /// Shape area divided by the oriented AABB area.
    pub fill_ratio: f32,
}

/// Pair-search audit data. Relative poses themselves live in [`PlacementMotif::members`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PairSearchMetadata {
    pub raster_cell: f32,
    pub offset_cells: (i32, i32),
    pub boundary_contacts: usize,
    /// Correlation implementation which proposed this offset.
    pub correlation_method: PairCorrelationMethod,
    /// Zero-padded radix-2 transform dimensions used by the correlation.
    pub fft_grid: (usize, usize),
    /// Boundary-pair support at the parent correlation displacement.
    pub correlation_peak: usize,
    /// True means the coarse proposal passed an exact polygon-interior check.
    pub exact_polygon_checked: bool,
    /// Always true: the consuming layout must still run its CDE at the final anchor.
    pub requires_cde_recheck: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PairCorrelationMethod {
    Radix2Fft2d,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BandMetadata {
    pub pattern: BandPattern,
    pub rows: usize,
    pub columns: usize,
    pub gap: f32,
    /// Mean pairwise similarity to the family's representative, in `[0, 1]`.
    pub family_similarity: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RectangleSubsolverMethod {
    BoundedBottomLeft,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RectangleBlockMetadata {
    pub method: RectangleSubsolverMethod,
    /// Width constraint used by the best bounded subsolver run.
    pub target_width: f32,
    /// Candidate rectangle poses inspected by that run.
    pub search_evaluations: usize,
    pub rows: usize,
    pub columns: usize,
    pub item_type_count: usize,
    pub exact_polygon_checked: bool,
    pub requires_cde_recheck: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub enum MotifKind {
    Single(ApproxRectangle),
    Pair(PairSearchMetadata),
    RectangleBlock(RectangleBlockMetadata),
    RepeatedBand(BandMetadata),
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotifScore {
    /// Weighted score used for ranking; higher is better.
    pub total: f32,
    pub envelope_fill: f32,
    pub boundary_match: f32,
    pub rectangularity: f32,
    pub repetition: f32,
    pub member_bonus: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotifEnvelope {
    /// Normalised envelope; its lower-left is always `(0, 0)`.
    pub bbox: Rect,
    pub member_area: f32,
    pub fill_ratio: f32,
}

/// A reversible construction hint.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacementMotif {
    pub kind: MotifKind,
    pub members: Vec<MotifMember>,
    pub envelope: MotifEnvelope,
    pub score: MotifScore,
}

impl PlacementMotif {
    /// Explicit marker for integrations: motifs never create a hard grouping constraint.
    pub const fn is_reversible(&self) -> bool {
        true
    }

    /// Materialise the suggested poses at a translated world-space lower-left anchor.
    ///
    /// The candidate's rotations are already absolute allowed rotations, so this method
    /// intentionally translates but does not rotate the motif as a rigid body.
    pub fn translated_placements(&self, anchor: (f32, f32)) -> Vec<MotifMember> {
        self.members
            .iter()
            .map(|member| {
                let (x, y) = member.relative_pose.translation();
                MotifMember {
                    item: member.item,
                    relative_pose: DTransformation::new(
                        member.relative_pose.rotation(),
                        (x + anchor.0, y + anchor.1),
                    ),
                }
            })
            .collect()
    }

    /// Dissolve the hint into ordinary independent item poses.
    pub fn dissolve(self) -> Vec<MotifMember> {
        self.members
    }
}

/// Per-kind storage keeps all independent hypothesis channels alive simultaneously.
#[derive(Clone, Debug, Default)]
pub struct MotifCatalog {
    pub singles: Vec<PlacementMotif>,
    pub pairs: Vec<PlacementMotif>,
    pub blocks: Vec<PlacementMotif>,
    pub bands: Vec<PlacementMotif>,
}

/// Bounded counters for the single-item generation channel.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(default)]
pub struct SingleMotifGenerationStats {
    /// Oriented AABB candidates which reached construction.
    pub attempted: usize,
    /// Complete candidates produced before per-channel ranking and truncation.
    pub generated: usize,
    /// Candidates retained in the returned catalogue.
    pub emitted: usize,
    /// The channel was entered but stopped before it completed.
    pub cancelled: bool,
}

/// Bounded counters for the FFT pair generation channel.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(default)]
pub struct PairMotifGenerationStats {
    /// Copy-pair and orientation combinations admitted by the global work cap.
    pub orientation_work: usize,
    /// Cheap-score offsets retained for coarse overlap/contact inspection.
    pub offsets_shortlisted: usize,
    pub coarse_overlap_rejected: usize,
    pub zero_contact_rejected: usize,
    /// Candidates which reached exact polygon/CDE validation.
    pub exact_validation_attempts: usize,
    pub overlap_rejected: usize,
    pub validator_rejected: usize,
    /// Complete candidates produced before deduplication/ranking/truncation.
    pub generated: usize,
    /// Candidates retained in the returned catalogue.
    pub emitted: usize,
    /// The channel was entered but stopped before it completed.
    pub cancelled: bool,
}

/// Bounded counters for the rectangle block subsolver channel.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(default)]
pub struct RectangleMotifGenerationStats {
    /// Demanded copies with at least one near-rectangular allowed orientation.
    pub eligible_parts: usize,
    pub solve_attempts: usize,
    /// Subsolver attempts which placed every admitted rectangle.
    pub solve_completed: usize,
    /// Completed layouts with at least two rows and two columns.
    pub layouts_2d: usize,
    pub overlap_rejected: usize,
    pub validator_rejected: usize,
    /// The fixed rectangle-subsolver evaluation budget reached zero.
    pub work_exhausted: bool,
    /// Complete candidates produced before deduplication/ranking/truncation.
    pub generated: usize,
    /// Candidates retained in the returned catalogue.
    pub emitted: usize,
    /// The channel was entered but stopped before it completed.
    pub cancelled: bool,
}

/// Bounded counters for the repeated-band generation channel.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(default)]
pub struct BandMotifGenerationStats {
    pub families: usize,
    pub layout_attempts: usize,
    pub overlap_rejected: usize,
    pub validator_rejected: usize,
    /// Complete candidates produced before deduplication/ranking/truncation.
    pub generated: usize,
    /// Candidates retained in the returned catalogue.
    pub emitted: usize,
    /// The channel was entered but stopped before it completed.
    pub cancelled: bool,
}

/// Auditable, fixed-size motif generation counters.
///
/// `generated` counters are measured before deduplication and top-k truncation;
/// `emitted` counters describe the returned [`MotifCatalog`]. No candidate-sized
/// trace is retained, so instrumentation memory remains constant as work grows.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(default)]
pub struct MotifGenerationStats {
    pub input_parts: usize,
    /// Any cooperative stop request latched during demand expansion or generation.
    pub generation_cancelled: bool,
    pub singles: SingleMotifGenerationStats,
    pub pairs: PairMotifGenerationStats,
    pub rectangle: RectangleMotifGenerationStats,
    pub bands: BandMotifGenerationStats,
}

/// Catalogue plus opt-in audit counters. Existing catalogue-returning APIs remain
/// source-compatible and discard only this fixed-size stats value.
#[derive(Clone, Debug)]
pub struct MotifGenerationResult {
    pub catalog: MotifCatalog,
    pub stats: MotifGenerationStats,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum PairMatchingMethod {
    #[default]
    None,
    /// Exact maximum-weight matching with a cardinality bound on a small bitmask graph.
    ExactBitmaskDp,
    /// Deterministic bounded greedy fallback for a larger copy graph.
    GreedyBoundedFallback,
}

impl PairMatchingMethod {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::ExactBitmaskDp => "exact_bitmask_dp",
            Self::GreedyBoundedFallback => "greedy_bounded_fallback",
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(default)]
pub struct MotifSelectionStats {
    pub pair_matching_method: PairMatchingMethod,
    pub pair_vertices: usize,
    pub pair_edges_considered: usize,
    pub pair_dp_states: usize,
    pub selected_singles: usize,
    pub selected_pairs: usize,
    pub selected_blocks: usize,
    pub selected_bands: usize,
    /// Number of demanded copies covered by the disjoint selection.
    pub covered_members: usize,
}

pub struct MotifSelection<'a> {
    pub motifs: Vec<&'a PlacementMotif>,
    pub stats: MotifSelectionStats,
}

impl MotifCatalog {
    pub fn len(&self) -> usize {
        self.singles.len() + self.pairs.len() + self.blocks.len() + self.bands.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn iter(&self) -> impl Iterator<Item = &PlacementMotif> {
        self.singles
            .iter()
            .chain(self.pairs.iter())
            .chain(self.blocks.iter())
            .chain(self.bands.iter())
    }

    /// All channels merged by score without changing the per-kind catalogues.
    pub fn ranked(&self) -> Vec<&PlacementMotif> {
        let mut ranked: Vec<_> = self.iter().collect();
        ranked.sort_by(compare_motifs);
        ranked
    }

    /// Deterministic weighted set-packing proposal with no demanded copy used twice.
    ///
    /// This is still advisory: accepting this list does not create a `PlaceGroup`, and
    /// every returned motif can be dissolved independently. Multi-member motifs are
    /// selected before singles so an attractive one-item AABB can never consume a copy
    /// needed by a pair, rectangle block, or repeated band. Pair selection is an exact
    /// maximum-weight matching for small copy graphs and a bounded deterministic fallback
    /// for larger graphs. Higher-arity motifs remain a reversible set-packing layer with
    /// deterministic local improvement.
    pub fn disjoint_ranked(&self, limit: usize) -> Vec<&PlacementMotif> {
        self.disjoint_ranked_with_stats(limit).motifs
    }

    /// Same selection as [`Self::disjoint_ranked`], with auditable matching statistics.
    pub fn disjoint_ranked_with_stats(&self, limit: usize) -> MotifSelection<'_> {
        if limit == 0 {
            return MotifSelection {
                motifs: Vec::new(),
                stats: MotifSelectionStats::default(),
            };
        }

        let (mut selected, mut stats) = maximum_weight_pair_matching(&self.pairs, limit);
        let mut higher_order = self
            .blocks
            .iter()
            .chain(self.bands.iter())
            .collect::<Vec<_>>();
        higher_order.sort_by(compare_motifs);

        let mut occupied = HashSet::new();
        rebuild_occupied(&selected, &mut occupied);
        for motif in higher_order.iter().copied() {
            if selected.len() >= limit {
                break;
            }
            if motif
                .members
                .iter()
                .all(|member| !occupied.contains(&member.item))
            {
                occupied.extend(motif.members.iter().map(|member| member.item));
                selected.push(motif);
            }
        }

        // A larger soft block/band can be globally better than all pair/block/band
        // motifs it conflicts with. Deterministic 1-for-many replacement preserves
        // reversibility while improving the bounded set-packing objective.
        loop {
            let mut improved = false;
            for candidate in higher_order.iter().copied() {
                if selected
                    .iter()
                    .any(|selected| std::ptr::eq(*selected, candidate))
                {
                    continue;
                }
                let conflict_indices = selected
                    .iter()
                    .enumerate()
                    .filter(|(_, selected)| motifs_overlap(selected, candidate))
                    .map(|(index, _)| index)
                    .collect::<Vec<_>>();
                if conflict_indices.is_empty() {
                    if selected.len() < limit {
                        selected.push(candidate);
                        rebuild_occupied(&selected, &mut occupied);
                        improved = true;
                        break;
                    }
                    continue;
                }
                let old_weight = conflict_indices
                    .iter()
                    .map(|&index| motif_selection_weight(selected[index]))
                    .sum::<f32>();
                if motif_selection_weight(candidate) <= old_weight + 1.0e-6 {
                    continue;
                }
                for &index in conflict_indices.iter().rev() {
                    selected.remove(index);
                }
                selected.push(candidate);
                rebuild_occupied(&selected, &mut occupied);
                improved = true;
                break;
            }
            if !improved {
                break;
            }
        }

        // A higher-arity replacement can free a motif slot or copies unrelated to
        // that block. Refill those holes from the remaining ranked pair channel.
        let mut remaining_pairs = self.pairs.iter().collect::<Vec<_>>();
        remaining_pairs.sort_by(compare_motifs);
        for motif in remaining_pairs {
            if selected.len() >= limit {
                break;
            }
            if motif
                .members
                .iter()
                .all(|member| !occupied.contains(&member.item))
            {
                occupied.extend(motif.members.iter().map(|member| member.item));
                selected.push(motif);
            }
        }

        selected.sort_by(compare_motifs);

        // Singles fill unused capacity only after every composite decision.
        let mut singles = self.singles.iter().collect::<Vec<_>>();
        singles.sort_by(compare_motifs);
        for motif in singles {
            if selected.len() >= limit {
                break;
            }
            if motif
                .members
                .iter()
                .all(|member| !occupied.contains(&member.item))
            {
                occupied.extend(motif.members.iter().map(|member| member.item));
                selected.push(motif);
            }
        }
        stats.selected_singles = selected
            .iter()
            .filter(|motif| matches!(motif.kind, MotifKind::Single(_)))
            .count();
        stats.selected_pairs = selected
            .iter()
            .filter(|motif| matches!(motif.kind, MotifKind::Pair(_)))
            .count();
        stats.selected_blocks = selected
            .iter()
            .filter(|motif| matches!(motif.kind, MotifKind::RectangleBlock(_)))
            .count();
        stats.selected_bands = selected
            .iter()
            .filter(|motif| matches!(motif.kind, MotifKind::RepeatedBand(_)))
            .count();
        stats.covered_members = selected.iter().map(|motif| motif.members.len()).sum();
        MotifSelection {
            motifs: selected,
            stats,
        }
    }
}

const EXACT_MATCHING_MAX_VERTICES: usize = 16;
const MATCHING_FALLBACK_MAX_EDGES: usize = 4096;

struct MatchingEdge<'a> {
    u: usize,
    v: usize,
    motif: &'a PlacementMotif,
    weight: f32,
}

#[derive(Clone, Copy)]
struct MatchingDpCell {
    weight: f32,
    /// -2 is uncomputed, -1 skips the first live vertex, otherwise an edge index.
    choice: i32,
}

fn maximum_weight_pair_matching<'a>(
    pairs: &'a [PlacementMotif],
    limit: usize,
) -> (Vec<&'a PlacementMotif>, MotifSelectionStats) {
    if limit == 0 || pairs.is_empty() {
        return (Vec::new(), MotifSelectionStats::default());
    }

    let mut ranked = pairs
        .iter()
        .filter(|motif| {
            matches!(motif.kind, MotifKind::Pair(_))
                && motif.members.len() == 2
                && motif.members[0].item != motif.members[1].item
        })
        .collect::<Vec<_>>();
    ranked.sort_by(compare_motifs);
    ranked.truncate(MATCHING_FALLBACK_MAX_EDGES);

    // Multiple relative poses can connect the same two demanded copies. Only the
    // highest-weight deterministic representative can occur in an optimal matching.
    let mut best_by_copies = BTreeMap::<(MotifItemRef, MotifItemRef), &PlacementMotif>::new();
    for motif in ranked {
        let mut endpoints = [motif.members[0].item, motif.members[1].item];
        endpoints.sort_unstable();
        best_by_copies
            .entry((endpoints[0], endpoints[1]))
            .or_insert(motif);
    }
    if best_by_copies.is_empty() {
        return (Vec::new(), MotifSelectionStats::default());
    }

    let mut vertex_ids = best_by_copies
        .keys()
        .flat_map(|&(a, b)| [a, b])
        .collect::<Vec<_>>();
    vertex_ids.sort_unstable();
    vertex_ids.dedup();
    let vertex_index = vertex_ids
        .iter()
        .enumerate()
        .map(|(index, &item)| (item, index))
        .collect::<BTreeMap<_, _>>();
    let edges = best_by_copies
        .into_iter()
        .map(|((a, b), motif)| MatchingEdge {
            u: vertex_index[&a],
            v: vertex_index[&b],
            motif,
            weight: motif_selection_weight(motif),
        })
        .collect::<Vec<_>>();

    let mut stats = MotifSelectionStats {
        pair_vertices: vertex_ids.len(),
        pair_edges_considered: edges.len(),
        ..MotifSelectionStats::default()
    };
    if vertex_ids.len() <= EXACT_MATCHING_MAX_VERTICES {
        stats.pair_matching_method = PairMatchingMethod::ExactBitmaskDp;
        let cardinality = limit.min(vertex_ids.len() / 2);
        let stride = cardinality + 1;
        let state_count = (1usize << vertex_ids.len()) * stride;
        let mut memo = vec![
            MatchingDpCell {
                weight: 0.0,
                choice: -2,
            };
            state_count
        ];
        let mut adjacency = vec![Vec::new(); vertex_ids.len()];
        for (edge_index, edge) in edges.iter().enumerate() {
            adjacency[edge.u].push(edge_index);
            adjacency[edge.v].push(edge_index);
        }
        let full_mask = (1_u64 << vertex_ids.len()) - 1;
        let mut evaluated_states = 0usize;
        solve_matching_dp(
            full_mask,
            cardinality,
            stride,
            &edges,
            &adjacency,
            &mut memo,
            &mut evaluated_states,
        );
        stats.pair_dp_states = evaluated_states;

        let mut selected = Vec::new();
        let mut mask = full_mask;
        let mut remaining = cardinality;
        while mask != 0 && remaining > 0 {
            let first = mask.trailing_zeros() as usize;
            let cell = memo[mask as usize * stride + remaining];
            if cell.choice < 0 {
                mask &= !(1_u64 << first);
                continue;
            }
            let edge = &edges[cell.choice as usize];
            selected.push(edge.motif);
            mask &= !(1_u64 << edge.u);
            mask &= !(1_u64 << edge.v);
            remaining -= 1;
        }
        stats.selected_pairs = selected.len();
        return (selected, stats);
    }

    stats.pair_matching_method = PairMatchingMethod::GreedyBoundedFallback;
    let mut edge_indices = (0..edges.len()).collect::<Vec<_>>();
    edge_indices.sort_by(|&lhs, &rhs| {
        OrderedFloat(edges[rhs].weight)
            .cmp(&OrderedFloat(edges[lhs].weight))
            .then_with(|| compare_motifs(&edges[lhs].motif, &edges[rhs].motif))
    });
    let mut occupied = HashSet::new();
    let mut selected = Vec::new();
    for edge_index in edge_indices {
        if selected.len() >= limit {
            break;
        }
        let edge = &edges[edge_index];
        if occupied.insert(edge.u) {
            if occupied.insert(edge.v) {
                selected.push(edge.motif);
            } else {
                occupied.remove(&edge.u);
            }
        }
    }
    stats.selected_pairs = selected.len();
    (selected, stats)
}

fn solve_matching_dp(
    mask: u64,
    remaining: usize,
    stride: usize,
    edges: &[MatchingEdge<'_>],
    adjacency: &[Vec<usize>],
    memo: &mut [MatchingDpCell],
    evaluated_states: &mut usize,
) -> f32 {
    if mask == 0 || remaining == 0 {
        return 0.0;
    }
    let memo_index = mask as usize * stride + remaining;
    if memo[memo_index].choice != -2 {
        return memo[memo_index].weight;
    }
    *evaluated_states += 1;

    let first = mask.trailing_zeros() as usize;
    let without_first = mask & !(1_u64 << first);
    let mut best_weight = solve_matching_dp(
        without_first,
        remaining,
        stride,
        edges,
        adjacency,
        memo,
        evaluated_states,
    );
    let mut best_choice = -1_i32;
    for &edge_index in &adjacency[first] {
        let edge = &edges[edge_index];
        let other = if edge.u == first { edge.v } else { edge.u };
        if mask & (1_u64 << other) == 0 {
            continue;
        }
        let candidate = edge.weight
            + solve_matching_dp(
                without_first & !(1_u64 << other),
                remaining - 1,
                stride,
                edges,
                adjacency,
                memo,
                evaluated_states,
            );
        if candidate > best_weight + 1.0e-6
            || ((candidate - best_weight).abs() <= 1.0e-6
                && (best_choice < 0 || edge_index < best_choice as usize))
        {
            best_weight = candidate;
            best_choice = edge_index as i32;
        }
    }
    memo[memo_index] = MatchingDpCell {
        weight: best_weight,
        choice: best_choice,
    };
    best_weight
}

fn motifs_overlap(a: &PlacementMotif, b: &PlacementMotif) -> bool {
    a.members
        .iter()
        .any(|member| b.members.iter().any(|other| member.item == other.item))
}

fn motif_selection_weight(motif: &PlacementMotif) -> f32 {
    motif.score.total.max(0.0) * (motif.members.len() as f32).sqrt()
}

fn rebuild_occupied(selected: &[&PlacementMotif], occupied: &mut HashSet<MotifItemRef>) {
    occupied.clear();
    occupied.extend(
        selected
            .iter()
            .flat_map(|motif| motif.members.iter().map(|member| member.item)),
    );
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct MotifScoreWeights {
    pub envelope_fill: f32,
    pub boundary_match: f32,
    pub rectangularity: f32,
    pub repetition: f32,
    pub member_bonus: f32,
}

impl Default for MotifScoreWeights {
    fn default() -> Self {
        Self {
            envelope_fill: 0.48,
            boundary_match: 0.22,
            rectangularity: 0.12,
            repetition: 0.12,
            member_bonus: 0.06,
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct MotifConfig {
    pub enabled: bool,
    /// Number of samples used for continuously rotatable items.
    pub continuous_orientation_samples: usize,
    /// Per-copy orientation candidates kept for singles.
    pub single_orientations_per_part: usize,
    pub top_k_singles: usize,
    /// Largest-area demanded copies admitted to the O(n²) pair pool.
    pub pair_pool_size: usize,
    pub pair_orientations_per_part: usize,
    pub pair_raster_divisions: usize,
    pub pair_max_axis_cells: usize,
    /// Maximum unique offsets which survive the deterministic O(1) cheap-score
    /// shortlist for one oriented pair. Only these offsets run occupancy/contact
    /// checks and exact polygon validation.
    pub pair_max_offset_candidates: usize,
    /// Maximum copy-pair x orientation combinations processed by one catalogue
    /// generation. This bounds total pair work independently of demand size.
    pub pair_max_orientation_work: usize,
    pub pair_shortlist_per_orientation: usize,
    pub top_k_pairs: usize,
    pub min_band_members: usize,
    pub max_band_members: usize,
    pub band_gap: f32,
    pub family_area_tolerance: f32,
    pub family_aspect_tolerance: f32,
    pub family_rectangularity_tolerance: f32,
    pub top_k_bands: usize,
    pub score_weights: MotifScoreWeights,
}

impl Default for MotifConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            continuous_orientation_samples: 4,
            single_orientations_per_part: 4,
            top_k_singles: 256,
            pair_pool_size: 24,
            pair_orientations_per_part: 3,
            pair_raster_divisions: 12,
            pair_max_axis_cells: 28,
            pair_max_offset_candidates: 256,
            pair_max_orientation_work: 2048,
            pair_shortlist_per_orientation: 8,
            top_k_pairs: 128,
            min_band_members: 4,
            max_band_members: 16,
            band_gap: 0.0,
            family_area_tolerance: 0.05,
            family_aspect_tolerance: 0.08,
            family_rectangularity_tolerance: 0.10,
            top_k_bands: 48,
            score_weights: MotifScoreWeights::default(),
        }
    }
}

/// Deterministic soft-motif discovery.
#[derive(Clone, Debug)]
pub struct MotifGenerator {
    config: MotifConfig,
}

impl MotifGenerator {
    pub fn new(config: MotifConfig) -> Self {
        Self {
            config: sanitise_config(config),
        }
    }

    pub fn config(&self) -> &MotifConfig {
        &self.config
    }

    /// Expand Jagua demand quantities into individually addressable motif parts.
    pub fn parts_from_instance(&self, instance: &SPInstance) -> Vec<MotifPart> {
        let mut never_stop = || false;
        let mut control = GenerationControl::new(&mut never_stop);
        self.parts_from_instance_controlled(instance, &mut control)
    }

    pub fn generate(&self, instance: &SPInstance) -> MotifCatalog {
        let mut never_stop = || false;
        self.generate_with_control(instance, &mut never_stop)
    }

    /// Generate a catalogue and fixed-size audit counters without cancellation.
    pub fn generate_with_stats(&self, instance: &SPInstance) -> MotifGenerationResult {
        let mut never_stop = || false;
        self.generate_with_control_and_stats(instance, &mut never_stop)
    }

    /// Cooperative counterpart of [`Self::generate`]. The callback is polled
    /// inside demand expansion and every expensive motif loop. Once it returns
    /// true, generation stops permanently and returns the safe partial catalog.
    pub fn generate_with_control(
        &self,
        instance: &SPInstance,
        should_stop: &mut dyn FnMut() -> bool,
    ) -> MotifCatalog {
        self.generate_with_control_and_stats(instance, should_stop)
            .catalog
    }

    /// Stats-returning counterpart of [`Self::generate_with_control`].
    pub fn generate_with_control_and_stats(
        &self,
        instance: &SPInstance,
        should_stop: &mut dyn FnMut() -> bool,
    ) -> MotifGenerationResult {
        let mut accept_all = AcceptAllValidator;
        self.generate_with_validator_and_control_and_stats(instance, &mut accept_all, should_stop)
    }

    pub fn generate_with_validator(
        &self,
        instance: &SPInstance,
        validator: &mut dyn MotifPoseValidator,
    ) -> MotifCatalog {
        self.generate_with_validator_and_stats(instance, validator)
            .catalog
    }

    /// Stats-returning counterpart of [`Self::generate_with_validator`].
    pub fn generate_with_validator_and_stats(
        &self,
        instance: &SPInstance,
        validator: &mut dyn MotifPoseValidator,
    ) -> MotifGenerationResult {
        let mut never_stop = || false;
        self.generate_with_validator_and_control_and_stats(instance, validator, &mut never_stop)
    }

    pub fn generate_with_validator_and_control(
        &self,
        instance: &SPInstance,
        validator: &mut dyn MotifPoseValidator,
        should_stop: &mut dyn FnMut() -> bool,
    ) -> MotifCatalog {
        self.generate_with_validator_and_control_and_stats(instance, validator, should_stop)
            .catalog
    }

    /// Full stats-returning generation entry point.
    pub fn generate_with_validator_and_control_and_stats(
        &self,
        instance: &SPInstance,
        validator: &mut dyn MotifPoseValidator,
        should_stop: &mut dyn FnMut() -> bool,
    ) -> MotifGenerationResult {
        let mut control = GenerationControl::new(should_stop);
        let parts = self.parts_from_instance_controlled(instance, &mut control);
        let mut result = self.generate_parts_controlled(&parts, validator, &mut control);
        result.stats.input_parts = instance.total_item_qty();
        result.stats.generation_cancelled = control.stopped;
        result
    }

    pub fn generate_parts(&self, parts: &[MotifPart]) -> MotifCatalog {
        let mut accept_all = AcceptAllValidator;
        let mut never_stop = || false;
        self.generate_parts_with_validator_and_control(parts, &mut accept_all, &mut never_stop)
    }

    /// Generate directly supplied parts and fixed-size audit counters.
    pub fn generate_parts_with_stats(&self, parts: &[MotifPart]) -> MotifGenerationResult {
        let mut accept_all = AcceptAllValidator;
        let mut never_stop = || false;
        self.generate_parts_with_validator_and_control_and_stats(
            parts,
            &mut accept_all,
            &mut never_stop,
        )
    }

    pub fn generate_parts_with_validator(
        &self,
        parts: &[MotifPart],
        validator: &mut dyn MotifPoseValidator,
    ) -> MotifCatalog {
        self.generate_parts_with_validator_and_stats(parts, validator)
            .catalog
    }

    /// Stats-returning counterpart of [`Self::generate_parts_with_validator`].
    pub fn generate_parts_with_validator_and_stats(
        &self,
        parts: &[MotifPart],
        validator: &mut dyn MotifPoseValidator,
    ) -> MotifGenerationResult {
        let mut never_stop = || false;
        self.generate_parts_with_validator_and_control_and_stats(parts, validator, &mut never_stop)
    }

    pub fn generate_parts_with_control(
        &self,
        parts: &[MotifPart],
        should_stop: &mut dyn FnMut() -> bool,
    ) -> MotifCatalog {
        self.generate_parts_with_control_and_stats(parts, should_stop)
            .catalog
    }

    /// Stats-returning counterpart of [`Self::generate_parts_with_control`].
    pub fn generate_parts_with_control_and_stats(
        &self,
        parts: &[MotifPart],
        should_stop: &mut dyn FnMut() -> bool,
    ) -> MotifGenerationResult {
        let mut accept_all = AcceptAllValidator;
        self.generate_parts_with_validator_and_control_and_stats(
            parts,
            &mut accept_all,
            should_stop,
        )
    }

    pub fn generate_parts_with_validator_and_control(
        &self,
        parts: &[MotifPart],
        validator: &mut dyn MotifPoseValidator,
        should_stop: &mut dyn FnMut() -> bool,
    ) -> MotifCatalog {
        self.generate_parts_with_validator_and_control_and_stats(parts, validator, should_stop)
            .catalog
    }

    /// Full stats-returning entry point for directly supplied parts.
    pub fn generate_parts_with_validator_and_control_and_stats(
        &self,
        parts: &[MotifPart],
        validator: &mut dyn MotifPoseValidator,
        should_stop: &mut dyn FnMut() -> bool,
    ) -> MotifGenerationResult {
        let mut control = GenerationControl::new(should_stop);
        self.generate_parts_controlled(parts, validator, &mut control)
    }

    fn parts_from_instance_controlled(
        &self,
        instance: &SPInstance,
        control: &mut GenerationControl<'_>,
    ) -> Vec<MotifPart> {
        let mut parts = Vec::with_capacity(instance.total_item_qty());
        for (item, qty) in &instance.items {
            if control.stop_requested() {
                break;
            }
            let rotations = rotations_for_range(
                &item.allowed_orientations().rotations(),
                self.config.continuous_orientation_samples,
            );
            for copy_index in 0..*qty {
                if control.stop_requested() {
                    break;
                }
                parts.push(MotifPart::new(
                    MotifItemRef {
                        item_id: item.idx(),
                        copy_index,
                    },
                    Arc::clone(&item.shape_cd()),
                    rotations.iter().copied(),
                ));
            }
        }
        parts
    }

    fn generate_parts_controlled(
        &self,
        parts: &[MotifPart],
        validator: &mut dyn MotifPoseValidator,
        control: &mut GenerationControl<'_>,
    ) -> MotifGenerationResult {
        let mut stats = MotifGenerationStats {
            input_parts: parts.len(),
            ..MotifGenerationStats::default()
        };
        if !self.config.enabled || parts.is_empty() {
            stats.generation_cancelled = control.stopped;
            return MotifGenerationResult {
                catalog: MotifCatalog::default(),
                stats,
            };
        }

        let mut catalog = MotifCatalog {
            singles: self.generate_singles(parts, control, &mut stats),
            pairs: vec![],
            blocks: vec![],
            bands: vec![],
        };
        if !control.stop_requested() {
            catalog.blocks = self.generate_rectangle_blocks(parts, validator, control, &mut stats);
        }
        if !control.stop_requested() {
            catalog.pairs = self.generate_pairs(parts, validator, control, &mut stats);
        }
        if !control.stop_requested() {
            catalog.bands = self.generate_bands(parts, validator, control, &mut stats);
        }
        rank_and_truncate(&mut catalog.singles, self.config.top_k_singles);
        rank_and_truncate(&mut catalog.pairs, self.config.top_k_pairs);
        rank_and_truncate(&mut catalog.blocks, self.config.top_k_bands);
        rank_and_truncate(&mut catalog.bands, self.config.top_k_bands);
        stats.singles.emitted = catalog.singles.len();
        stats.pairs.emitted = catalog.pairs.len();
        stats.rectangle.emitted = catalog.blocks.len();
        stats.bands.emitted = catalog.bands.len();
        stats.generation_cancelled = control.stopped;
        MotifGenerationResult { catalog, stats }
    }

    fn generate_singles(
        &self,
        parts: &[MotifPart],
        control: &mut GenerationControl<'_>,
        stats: &mut MotifGenerationStats,
    ) -> Vec<PlacementMotif> {
        let mut motifs = Vec::new();
        for part in parts {
            if control.stop_requested() {
                stats.singles.cancelled = true;
                break;
            }
            let mut oriented = oriented_variants(part);
            oriented.sort_by(|a, b| {
                OrderedFloat(b.rect.fill_ratio).cmp(&OrderedFloat(a.rect.fill_ratio))
            });
            oriented.truncate(self.config.single_orientations_per_part);

            for variant in oriented {
                if control.stop_requested() {
                    stats.singles.cancelled = true;
                    break;
                }
                stats.singles.attempted += 1;
                let poses = [MotifPoseRef {
                    part,
                    relative_pose: variant.pose,
                }];
                let envelope = envelope_for(&poses);
                let components = ScoreComponents {
                    envelope_fill: envelope.fill_ratio,
                    boundary_match: 0.0,
                    rectangularity: variant.rect.fill_ratio,
                    repetition: 0.0,
                    member_bonus: member_bonus(1),
                };
                motifs.push(PlacementMotif {
                    kind: MotifKind::Single(variant.rect),
                    members: to_members(&poses),
                    envelope,
                    score: score(components, self.config.score_weights),
                });
                stats.singles.generated += 1;
            }
        }
        motifs
    }

    fn generate_rectangle_blocks(
        &self,
        parts: &[MotifPart],
        validator: &mut dyn MotifPoseValidator,
        control: &mut GenerationControl<'_>,
        stats: &mut MotifGenerationStats,
    ) -> Vec<PlacementMotif> {
        const MIN_RECTANGULARITY: f32 = 0.88;
        const MAX_SUBSOLVER_MEMBERS: usize = 16;
        const MAX_SUBSOLVER_EVALUATIONS: usize = 50_000;

        let mut rectangle_parts = Vec::new();
        for part in parts {
            if control.stop_requested() {
                stats.rectangle.cancelled = true;
                return Vec::new();
            }
            let mut variants = oriented_variants(part)
                .into_iter()
                .filter(|variant| variant.rect.fill_ratio >= MIN_RECTANGULARITY)
                .collect::<Vec<_>>();
            variants.sort_by(rect_fill_desc);
            variants
                .dedup_by(|a, b| angular_distance(a.pose.rotation(), b.pose.rotation()) <= 1.0e-6);
            variants.truncate(2);
            if !variants.is_empty() {
                stats.rectangle.eligible_parts += 1;
                rectangle_parts.push(RectanglePart { part, variants });
            }
        }
        rectangle_parts.sort_by(|a, b| {
            a.part
                .key
                .copy_index
                .cmp(&b.part.key.copy_index)
                .then_with(|| OrderedFloat(b.part.shape.area()).cmp(&OrderedFloat(a.part.shape.area())))
                .then_with(|| a.part.key.item_id.cmp(&b.part.key.item_id))
        });
        rectangle_parts.truncate(self.config.max_band_members.min(MAX_SUBSOLVER_MEMBERS));
        if rectangle_parts.len() < self.config.min_band_members {
            return Vec::new();
        }

        let orders = rectangle_orders(&rectangle_parts);
        let target_widths = rectangle_target_widths(&rectangle_parts);
        let mut remaining_work = MAX_SUBSOLVER_EVALUATIONS;
        let mut motifs = Vec::new();
        'orders: for order in &orders {
            for &target_width in &target_widths {
                if control.stop_requested() {
                    stats.rectangle.cancelled = true;
                    break 'orders;
                }
                if remaining_work == 0 {
                    stats.rectangle.work_exhausted = true;
                    break 'orders;
                }
                stats.rectangle.solve_attempts += 1;
                let Some(mut layout) = solve_rectangle_block(
                    &rectangle_parts,
                    order,
                    target_width,
                    self.config.band_gap,
                    &mut remaining_work,
                    control,
                ) else {
                    stats.rectangle.cancelled |= control.stopped;
                    stats.rectangle.work_exhausted |= remaining_work == 0;
                    continue;
                };
                stats.rectangle.solve_completed += 1;
                let (rows, columns) = rectangle_grid_shape(&layout.rects);
                if rows < 2 || columns < 2 {
                    continue;
                }
                stats.rectangle.layouts_2d += 1;
                normalise_poses(&mut layout.poses);
                if has_polygon_interior_overlap(&layout.poses) {
                    stats.rectangle.overlap_rejected += 1;
                    continue;
                }
                if !validator.is_feasible(&layout.poses) {
                    stats.rectangle.validator_rejected += 1;
                    continue;
                }
                if control.stop_requested() {
                    stats.rectangle.cancelled = true;
                    break 'orders;
                }
                let envelope = envelope_for(&layout.poses);
                let rectangularity = layout
                    .poses
                    .iter()
                    .map(|pose| {
                        rectangle_parts
                            .iter()
                            .find(|candidate| candidate.part.key == pose.part.key)
                            .map(|candidate| {
                                candidate
                                    .variants
                                    .iter()
                                    .find(|variant| {
                                        angular_distance(
                                            variant.pose.rotation(),
                                            pose.relative_pose.rotation(),
                                        ) <= 1.0e-6
                                    })
                                    .map(|variant| variant.rect.fill_ratio)
                                    .unwrap_or(0.0)
                            })
                            .unwrap_or(0.0)
                    })
                    .sum::<f32>()
                    / layout.poses.len() as f32;
                let item_type_count = layout
                    .poses
                    .iter()
                    .map(|pose| pose.part.key.item_id)
                    .collect::<HashSet<_>>()
                    .len();
                let components = ScoreComponents {
                    envelope_fill: envelope.fill_ratio,
                    boundary_match: rectangle_contact_proxy(&layout.rects, self.config.band_gap),
                    rectangularity,
                    repetition: 0.0,
                    member_bonus: member_bonus(layout.poses.len()),
                };
                motifs.push(PlacementMotif {
                    kind: MotifKind::RectangleBlock(RectangleBlockMetadata {
                        method: RectangleSubsolverMethod::BoundedBottomLeft,
                        target_width,
                        search_evaluations: layout.evaluations,
                        rows,
                        columns,
                        item_type_count,
                        exact_polygon_checked: true,
                        requires_cde_recheck: true,
                    }),
                    members: to_members(&layout.poses),
                    envelope,
                    score: score(components, self.config.score_weights),
                });
                stats.rectangle.generated += 1;
            }
        }
        stats.rectangle.work_exhausted |= remaining_work == 0;
        stats.rectangle.cancelled |= control.stopped;
        deduplicate_motifs(motifs)
    }

    fn generate_pairs(
        &self,
        parts: &[MotifPart],
        validator: &mut dyn MotifPoseValidator,
        control: &mut GenerationControl<'_>,
        stats: &mut MotifGenerationStats,
    ) -> Vec<PlacementMotif> {
        let pool = select_pair_pool(parts, self.config.pair_pool_size, control);
        if pool.len() < 2
            || self.config.pair_max_orientation_work == 0
            || self.config.pair_max_offset_candidates == 0
            || control.stop_requested()
        {
            stats.pairs.cancelled |= control.stopped;
            return Vec::new();
        }
        let Some(cell) = shared_pair_cell(&pool, &self.config, control) else {
            stats.pairs.cancelled |= control.stopped;
            return Vec::new();
        };

        let mut motifs = Vec::new();
        let mut masks = HashMap::<PairMaskKey, RasterMask>::new();
        let mut orientation_work = 0usize;
        // Orientation variants depend only on the part, so build them once per
        // pool entry instead of re-rotating both shapes for every (i, j) pair.
        let pool_variants: Vec<Vec<OrientedPart<'_>>> = pool
            .iter()
            .map(|&part| {
                let mut variants = oriented_variants(part);
                variants.sort_by(rect_fill_desc);
                variants.truncate(self.config.pair_orientations_per_part);
                variants
            })
            .collect();
        'pairs: for i in 0..pool.len() {
            if control.stop_requested() {
                stats.pairs.cancelled = true;
                break;
            }
            for j in (i + 1)..pool.len() {
                if control.stop_requested() {
                    stats.pairs.cancelled = true;
                    break 'pairs;
                }
                let a_variants = &pool_variants[i];
                let b_variants = &pool_variants[j];

                for av in a_variants {
                    for bv in b_variants {
                        if control.stop_requested() {
                            stats.pairs.cancelled = true;
                            break 'pairs;
                        }
                        if orientation_work >= self.config.pair_max_orientation_work {
                            break 'pairs;
                        }
                        orientation_work += 1;
                        stats.pairs.orientation_work += 1;
                        motifs.extend(
                            self.correlate_pair(
                                av, bv, cell, &mut masks, validator, control, stats,
                            ),
                        );
                    }
                }
            }
        }
        stats.pairs.cancelled |= control.stopped;
        deduplicate_motifs(motifs)
    }

    fn correlate_pair(
        &self,
        a: &OrientedPart<'_>,
        b: &OrientedPart<'_>,
        cell: f32,
        masks: &mut HashMap<PairMaskKey, RasterMask>,
        validator: &mut dyn MotifPoseValidator,
        control: &mut GenerationControl<'_>,
        stats: &mut MotifGenerationStats,
    ) -> Vec<PlacementMotif> {
        if control.stop_requested() {
            stats.pairs.cancelled = true;
            return Vec::new();
        }
        let a_key = PairMaskKey::new(a.part.key.item_id, a.pose.rotation());
        let b_key = PairMaskKey::new(b.part.key.item_id, b.pose.rotation());
        masks
            .entry(a_key)
            .or_insert_with(|| RasterMask::from_shape(&a.shape, cell));
        masks
            .entry(b_key)
            .or_insert_with(|| RasterMask::from_shape(&b.shape, cell));
        let am = masks.get(&a_key).expect(crate::dmsg!("inserted pair mask"));
        let bm = masks.get(&b_key).expect(crate::dmsg!("inserted pair mask"));
        if am.occupied.is_empty() || bm.occupied.is_empty() {
            return Vec::new();
        }

        // Boundary correlation itself is cheap compared with an occupancy scan.
        // Rank all unique adjacency offsets by an O(1) envelope-fill proxy and
        // hard-cap the offsets before overlap/contact or polygon checks.
        let Some(correlation) = fft_boundary_correlation_offsets(am, bm, control) else {
            stats.pairs.cancelled |= control.stopped;
            return Vec::new();
        };
        let occupied_cell_area = (am.occupied.len() + bm.occupied.len()) as f32 * cell * cell;
        let mut cheap = correlation
            .offsets
            .into_iter()
            .map(|(offset, correlation_peak)| {
                let (grid_w, grid_h) = union_grid_extent(am, bm, offset);
                let envelope_area = grid_w as f32 * grid_h as f32 * cell * cell;
                (
                    OrderedFloat(
                        (occupied_cell_area / envelope_area.max(f32::EPSILON)).clamp(0.0, 1.0),
                    ),
                    offset,
                    correlation_peak,
                )
            })
            .collect::<Vec<_>>();
        cheap.sort_by(|lhs, rhs| {
            rhs.0
                .cmp(&lhs.0)
                .then_with(|| rhs.2.cmp(&lhs.2))
                .then_with(|| lhs.1.cmp(&rhs.1))
        });
        cheap.truncate(self.config.pair_max_offset_candidates);
        stats.pairs.offsets_shortlisted += cheap.len();

        let mut coarse = Vec::new();
        for (_, offset, correlation_peak) in cheap {
            if control.stop_requested() {
                stats.pairs.cancelled = true;
                return Vec::new();
            }
            if am.overlaps(bm, offset) {
                stats.pairs.coarse_overlap_rejected += 1;
                continue;
            }
            let contacts = am.contact_count(bm, offset);
            if contacts == 0 {
                stats.pairs.zero_contact_rejected += 1;
                continue;
            }
            let (grid_w, grid_h) = union_grid_extent(&am, &bm, offset);
            let envelope_area = grid_w as f32 * grid_h as f32 * cell * cell;
            let fill = ((a.part.shape.area() + b.part.shape.area()) / envelope_area).clamp(0.0, 1.0);
            let contact = contact_ratio(contacts, am.boundary.len(), bm.boundary.len());
            coarse.push((
                OrderedFloat(fill + 0.25 * contact),
                offset,
                contacts,
                correlation_peak,
            ));
        }
        coarse.sort_by(|lhs, rhs| rhs.0.cmp(&lhs.0));
        coarse.truncate(self.config.pair_shortlist_per_orientation);

        let mut motifs = Vec::new();
        for (_, offset, contacts, correlation_peak) in coarse {
            if control.stop_requested() {
                stats.pairs.cancelled = true;
                break;
            }
            let (bx, by) = b.pose.translation();
            let b_pose = DTransformation::new(
                b.pose.rotation(),
                (bx + offset.0 as f32 * cell, by + offset.1 as f32 * cell),
            );
            let mut poses = vec![
                MotifPoseRef {
                    part: a.part,
                    relative_pose: a.pose,
                },
                MotifPoseRef {
                    part: b.part,
                    relative_pose: b_pose,
                },
            ];
            normalise_poses(&mut poses);
            stats.pairs.exact_validation_attempts += 1;
            if has_polygon_interior_overlap(&poses) {
                stats.pairs.overlap_rejected += 1;
                continue;
            }
            if !validator.is_feasible(&poses) {
                stats.pairs.validator_rejected += 1;
                continue;
            }

            let envelope = envelope_for(&poses);
            let contact = contact_ratio(contacts, am.boundary.len(), bm.boundary.len());
            let rectangularity = (a.rect.fill_ratio + b.rect.fill_ratio) * 0.5;
            let components = ScoreComponents {
                envelope_fill: envelope.fill_ratio,
                boundary_match: contact,
                rectangularity,
                repetition: family_similarity(a.part, b.part, &self.config),
                member_bonus: member_bonus(2),
            };
            motifs.push(PlacementMotif {
                kind: MotifKind::Pair(PairSearchMetadata {
                    raster_cell: cell,
                    offset_cells: offset,
                    boundary_contacts: contacts,
                    correlation_method: PairCorrelationMethod::Radix2Fft2d,
                    fft_grid: correlation.fft_grid,
                    correlation_peak,
                    exact_polygon_checked: true,
                    requires_cde_recheck: true,
                }),
                members: to_members(&poses),
                envelope,
                score: score(components, self.config.score_weights),
            });
            stats.pairs.generated += 1;
        }
        motifs
    }

    fn generate_bands(
        &self,
        parts: &[MotifPart],
        validator: &mut dyn MotifPoseValidator,
        control: &mut GenerationControl<'_>,
        stats: &mut MotifGenerationStats,
    ) -> Vec<PlacementMotif> {
        let families = discover_families(parts, &self.config, control);
        stats.bands.families += families.len();
        stats.bands.cancelled |= control.stopped;
        let mut motifs = Vec::new();
        for family in families {
            if control.stop_requested() {
                stats.bands.cancelled = true;
                break;
            }
            let members: Vec<_> = family
                .into_iter()
                .take(self.config.max_band_members)
                .collect();
            if members.len() < self.config.min_band_members {
                continue;
            }
            let mut variants = Vec::with_capacity(members.len());
            for part in &members {
                if control.stop_requested() {
                    stats.bands.cancelled = true;
                    break;
                }
                variants.push(best_oriented_variant(part));
            }
            if control.stop_requested() || variants.len() != members.len() {
                stats.bands.cancelled |= control.stopped;
                break;
            }
            let similarity = mean_family_similarity(&members, &self.config);

            for pattern in [
                BandPattern::Horizontal,
                BandPattern::Vertical,
                BandPattern::StaggeredLattice,
            ] {
                if control.stop_requested() {
                    stats.bands.cancelled = true;
                    break;
                }
                stats.bands.layout_attempts += 1;
                let mut poses = layout_band(&variants, pattern, self.config.band_gap);
                normalise_poses(&mut poses);
                if has_polygon_interior_overlap(&poses) {
                    stats.bands.overlap_rejected += 1;
                    continue;
                }
                if !validator.is_feasible(&poses) {
                    stats.bands.validator_rejected += 1;
                    continue;
                }
                let envelope = envelope_for(&poses);
                let rectangularity =
                    variants.iter().map(|v| v.rect.fill_ratio).sum::<f32>() / variants.len() as f32;
                let (rows, columns) = band_grid_shape(pattern, variants.len());
                let components = ScoreComponents {
                    envelope_fill: envelope.fill_ratio,
                    boundary_match: band_contact_proxy(&poses, self.config.band_gap),
                    rectangularity,
                    repetition: similarity,
                    member_bonus: member_bonus(poses.len()),
                };
                motifs.push(PlacementMotif {
                    kind: MotifKind::RepeatedBand(BandMetadata {
                        pattern,
                        rows,
                        columns,
                        gap: self.config.band_gap,
                        family_similarity: similarity,
                    }),
                    members: to_members(&poses),
                    envelope,
                    score: score(components, self.config.score_weights),
                });
                stats.bands.generated += 1;
            }
        }
        stats.bands.cancelled |= control.stopped;
        deduplicate_motifs(motifs)
    }
}

/// Fill the expensive pair pool copy-index round by copy-index. This prevents one
/// high-demand large item from consuming the entire pool before mirror/complementary
/// item types receive a candidate, while still preferring large shapes within a round.
fn select_pair_pool<'a>(
    parts: &'a [MotifPart],
    limit: usize,
    control: &mut GenerationControl<'_>,
) -> Vec<&'a MotifPart> {
    if limit == 0 {
        return Vec::new();
    }

    // Group once instead of rescanning every demanded copy for each copy-index
    // round. Besides reducing the old O(demand^2) selection path to O(demand),
    // this gives demand expansion a cooperative cancellation point per copy.
    let mut rounds = BTreeMap::<usize, Vec<&MotifPart>>::new();
    for part in parts {
        if control.stop_requested() {
            return Vec::new();
        }
        rounds.entry(part.key.copy_index).or_default().push(part);
    }

    let mut pool = Vec::with_capacity(limit.min(parts.len()));
    for (_, mut round) in rounds {
        if control.stop_requested() {
            return Vec::new();
        }
        round.sort_by(|a, b| {
            OrderedFloat(b.shape.area())
                .cmp(&OrderedFloat(a.shape.area()))
                .then_with(|| a.key.item_id.cmp(&b.key.item_id))
        });
        for part in round {
            if control.stop_requested() {
                return Vec::new();
            }
            pool.push(part);
            if pool.len() == limit {
                return pool;
            }
        }
    }
    pool
}

#[derive(Clone)]
struct OrientedPart<'a> {
    part: &'a MotifPart,
    pose: DTransformation,
    shape: SPolygon,
    rect: ApproxRectangle,
}

struct RectanglePart<'a> {
    part: &'a MotifPart,
    variants: Vec<OrientedPart<'a>>,
}

struct RectangleLayout<'a> {
    poses: Vec<MotifPoseRef<'a>>,
    rects: Vec<Rect>,
    evaluations: usize,
}

fn rectangle_orders(parts: &[RectanglePart<'_>]) -> Vec<Vec<usize>> {
    let mut orders = Vec::new();
    for mode in 0..3 {
        let mut order = (0..parts.len()).collect::<Vec<_>>();
        order.sort_by(|&lhs, &rhs| {
            let left = &parts[lhs];
            let right = &parts[rhs];
            let metric = |part: &RectanglePart<'_>| match mode {
                0 => part.part.shape.area(),
                1 => part
                    .variants
                    .iter()
                    .map(|variant| variant.rect.height)
                    .fold(0.0_f32, f32::max),
                _ => part
                    .variants
                    .iter()
                    .map(|variant| variant.rect.width)
                    .fold(0.0_f32, f32::max),
            };
            OrderedFloat(metric(right))
                .cmp(&OrderedFloat(metric(left)))
                .then_with(|| left.part.key.cmp(&right.part.key))
        });
        if !orders.contains(&order) {
            orders.push(order);
        }
    }
    orders
}

fn rectangle_target_widths(parts: &[RectanglePart<'_>]) -> Vec<f32> {
    let minimum_widths = parts
        .iter()
        .map(|part| {
            part.variants
                .iter()
                .map(|variant| variant.rect.width)
                .fold(f32::INFINITY, f32::min)
        })
        .collect::<Vec<_>>();
    let max_minimum_width = minimum_widths.iter().copied().fold(0.0_f32, f32::max);
    let sum_minimum_width = minimum_widths.iter().sum::<f32>();
    let total_envelope_area = parts
        .iter()
        .map(|part| {
            part.variants
                .iter()
                .map(|variant| variant.rect.width * variant.rect.height)
                .fold(f32::INFINITY, f32::min)
        })
        .sum::<f32>();
    let square = total_envelope_area.sqrt();
    let mut widths = vec![
        max_minimum_width,
        square * 0.85,
        square,
        square * 1.25,
        square * 1.6,
        sum_minimum_width * 0.5,
        sum_minimum_width,
    ];
    widths.retain(|width| width.is_finite() && *width >= max_minimum_width - 1.0e-6);
    widths.sort_by_key(|width| OrderedFloat(*width));
    widths.dedup_by(|a, b| (*a - *b).abs() <= 1.0e-5 * a.abs().max(b.abs()).max(1.0));
    widths
}

fn solve_rectangle_block<'a>(
    parts: &[RectanglePart<'a>],
    order: &[usize],
    target_width: f32,
    gap: f32,
    remaining_work: &mut usize,
    control: &mut GenerationControl<'_>,
) -> Option<RectangleLayout<'a>> {
    let mut poses = Vec::with_capacity(order.len());
    let mut rects: Vec<Rect> = Vec::with_capacity(order.len());
    let mut evaluations = 0usize;
    for &part_index in order {
        if control.stop_requested() {
            return None;
        }
        let mut x_candidates = vec![0.0_f32];
        let mut y_candidates = vec![0.0_f32];
        for rect in &rects {
            x_candidates.push(rect.x_max + gap);
            y_candidates.push(rect.y_max + gap);
        }
        x_candidates.sort_by_key(|value| OrderedFloat(*value));
        y_candidates.sort_by_key(|value| OrderedFloat(*value));
        x_candidates.dedup_by(|a, b| (*a - *b).abs() <= 1.0e-6);
        y_candidates.dedup_by(|a, b| (*a - *b).abs() <= 1.0e-6);

        let mut best: Option<(
            (OrderedFloat<f32>, OrderedFloat<f32>, OrderedFloat<f32>, i64),
            Rect,
            MotifPoseRef<'a>,
        )> = None;
        for variant in &parts[part_index].variants {
            for &y in &y_candidates {
                for &x in &x_candidates {
                    if control.stop_requested() || *remaining_work == 0 {
                        return None;
                    }
                    *remaining_work -= 1;
                    evaluations += 1;
                    let candidate = Rect {
                        x_min: x,
                        y_min: y,
                        x_max: x + variant.rect.width,
                        y_max: y + variant.rect.height,
                    };
                    if candidate.x_max > target_width + 1.0e-5
                        || rects
                            .iter()
                            .any(|placed| rectangle_gap_conflict(candidate, *placed, gap))
                    {
                        continue;
                    }
                    let resulting_height = rects
                        .iter()
                        .map(|rect| rect.y_max)
                        .fold(candidate.y_max, f32::max);
                    let key = (
                        OrderedFloat(y),
                        OrderedFloat(x),
                        OrderedFloat(resulting_height),
                        quantise(variant.pose.rotation()),
                    );
                    if best.as_ref().is_some_and(|(current, _, _)| key >= *current) {
                        continue;
                    }
                    let (px, py) = variant.pose.translation();
                    best = Some((
                        key,
                        candidate,
                        MotifPoseRef {
                            part: variant.part,
                            relative_pose: DTransformation::new(
                                variant.pose.rotation(),
                                (px + x, py + y),
                            ),
                        },
                    ));
                }
            }
        }
        let (_, rect, pose) = best?;
        rects.push(rect);
        poses.push(pose);
    }
    Some(RectangleLayout {
        poses,
        rects,
        evaluations,
    })
}

fn rectangle_gap_conflict(a: Rect, b: Rect, gap: f32) -> bool {
    const EPS: f32 = 1.0e-6;
    a.x_min < b.x_max + gap - EPS
        && a.x_max > b.x_min - gap + EPS
        && a.y_min < b.y_max + gap - EPS
        && a.y_max > b.y_min - gap + EPS
}

fn rectangle_grid_shape(rects: &[Rect]) -> (usize, usize) {
    let mut rows = BTreeMap::<i64, usize>::new();
    for rect in rects {
        *rows.entry(quantise(rect.y_min)).or_default() += 1;
    }
    let columns = rows.values().copied().max().unwrap_or(0);
    (rows.len(), columns)
}

fn rectangle_contact_proxy(rects: &[Rect], gap: f32) -> f32 {
    if rects.len() < 2 {
        return 0.0;
    }
    let tolerance = 1.0e-5
        * rects
            .iter()
            .map(|rect| rect.width().max(rect.height()))
            .fold(1.0_f32, f32::max);
    let mut contacts = 0usize;
    for i in 0..rects.len() {
        for b in &rects[(i + 1)..] {
            let a = rects[i];
            let horizontal_overlap = a.x_min < b.x_max - tolerance && a.x_max > b.x_min + tolerance;
            let vertical_overlap = a.y_min < b.y_max - tolerance && a.y_max > b.y_min + tolerance;
            let vertical_contact = ((a.y_max + gap) - b.y_min).abs() <= tolerance
                || ((b.y_max + gap) - a.y_min).abs() <= tolerance;
            let horizontal_contact = ((a.x_max + gap) - b.x_min).abs() <= tolerance
                || ((b.x_max + gap) - a.x_min).abs() <= tolerance;
            if (horizontal_overlap && vertical_contact) || (vertical_overlap && horizontal_contact)
            {
                contacts += 1;
            }
        }
    }
    (contacts as f32 / (rects.len() - 1) as f32).clamp(0.0, 1.0)
}

/// Copy-independent key for the shared pair raster. All pair masks in one
/// catalogue use one physical cell size, so `(item type, absolute rotation)`
/// completely identifies reusable raster geometry.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
struct PairMaskKey {
    item_id: usize,
    rotation_bits: u32,
}

impl PairMaskKey {
    fn new(item_id: usize, rotation: f32) -> Self {
        Self {
            item_id,
            rotation_bits: normalize_rotation(rotation).to_bits(),
        }
    }
}

fn shared_pair_cell(
    pool: &[&MotifPart],
    config: &MotifConfig,
    control: &mut GenerationControl<'_>,
) -> Option<f32> {
    let mut seen_item_types = HashSet::new();
    let mut smallest_dimension = f32::INFINITY;
    let mut largest_dimension = 0.0_f32;
    for part in pool {
        if control.stop_requested() {
            return None;
        }
        if !seen_item_types.insert(part.key.item_id) {
            continue;
        }
        let mut variants = oriented_variants(part);
        variants.sort_by(rect_fill_desc);
        variants.truncate(config.pair_orientations_per_part);
        for variant in variants {
            smallest_dimension = smallest_dimension
                .min(variant.rect.width)
                .min(variant.rect.height);
            largest_dimension = largest_dimension
                .max(variant.rect.width)
                .max(variant.rect.height);
        }
    }
    if !smallest_dimension.is_finite() || largest_dimension <= 0.0 {
        return None;
    }
    Some(
        (smallest_dimension.max(1.0e-4) / config.pair_raster_divisions as f32)
            .max(largest_dimension / config.pair_max_axis_cells as f32)
            .max(1.0e-4),
    )
}

fn oriented_variants(part: &MotifPart) -> Vec<OrientedPart<'_>> {
    part.allowed_rotations
        .iter()
        .copied()
        .map(|orientation| {
            let rotation = Transformation::from_rotation(orientation);
            let mut rotated = (*part.shape).clone();
            rotated.transform(&rotation);
            let rb = rotated.bbox();
            let pose = DTransformation::new(orientation, (-rb.x_min, -rb.y_min));
            let mut normalised = (*part.shape).clone();
            normalised.transform(&Transformation::from(&pose));
            let bbox = normalised.bbox();
            let area = bbox.area().max(f32::EPSILON);
            OrientedPart {
                part,
                pose,
                shape: normalised,
                rect: ApproxRectangle {
                    orientation,
                    width: bbox.width(),
                    height: bbox.height(),
                    fill_ratio: (part.shape.area() / area).clamp(0.0, 1.0),
                },
            }
        })
        .collect()
}

fn best_oriented_variant(part: &MotifPart) -> OrientedPart<'_> {
    oriented_variants(part)
        .into_iter()
        .max_by(rect_fill_asc)
        .expect(crate::dmsg!("MotifPart always has at least one allowed rotation"))
}

fn rect_fill_asc(a: &OrientedPart<'_>, b: &OrientedPart<'_>) -> std::cmp::Ordering {
    OrderedFloat(a.rect.fill_ratio).cmp(&OrderedFloat(b.rect.fill_ratio))
}

fn rect_fill_desc(a: &OrientedPart<'_>, b: &OrientedPart<'_>) -> std::cmp::Ordering {
    rect_fill_asc(b, a)
}

#[derive(Clone, Debug)]
struct RasterMask {
    width: i32,
    height: i32,
    /// Occupied cells for iteration (and cell counting).
    occupied: HashSet<(i32, i32)>,
    /// Dense row-major occupancy (`width * height`) for O(1) membership probes;
    /// the correlation hot loops query this instead of hashing every cell.
    grid: Vec<bool>,
    boundary: Vec<(i32, i32)>,
}

impl RasterMask {
    fn from_shape(shape: &SPolygon, cell: f32) -> Self {
        let width = (shape.bbox().width() / cell).ceil().max(1.0) as i32;
        let height = (shape.bbox().height() / cell).ceil().max(1.0) as i32;
        let mut occupied = HashSet::new();
        for y in 0..height {
            for x in 0..width {
                let point = Point(
                    shape.bbox().x_min + (x as f32 + 0.5) * cell,
                    shape.bbox().y_min + (y as f32 + 0.5) * cell,
                );
                if point_in_polygon_inclusive(point, shape) {
                    occupied.insert((x, y));
                }
            }
        }

        // Thin parts can miss every cell centre. Conservatively seed cells containing vertices.
        if occupied.is_empty() {
            for point in shape.vertices() {
                let x = ((point.0 - shape.bbox().x_min) / cell).floor() as i32;
                let y = ((point.1 - shape.bbox().y_min) / cell).floor() as i32;
                occupied.insert((x.clamp(0, width - 1), y.clamp(0, height - 1)));
            }
        }

        let mut grid = vec![false; (width as usize) * (height as usize)];
        for &(x, y) in &occupied {
            grid[(y as usize) * (width as usize) + x as usize] = true;
        }
        let has = |x: i32, y: i32| {
            x >= 0
                && y >= 0
                && x < width
                && y < height
                && grid[(y as usize) * (width as usize) + x as usize]
        };

        const DIRS: [(i32, i32); 4] = [(1, 0), (-1, 0), (0, 1), (0, -1)];
        let mut boundary: Vec<_> = occupied
            .iter()
            .copied()
            .filter(|&(x, y)| DIRS.iter().any(|&(dx, dy)| !has(x + dx, y + dy)))
            .collect();
        boundary.sort_unstable();
        Self {
            width,
            height,
            occupied,
            grid,
            boundary,
        }
    }

    /// Dense membership probe; out-of-range cells are never occupied.
    #[inline]
    fn has(&self, x: i32, y: i32) -> bool {
        x >= 0
            && y >= 0
            && x < self.width
            && y < self.height
            && self.grid[(y as usize) * (self.width as usize) + x as usize]
    }

    fn overlaps(&self, other: &Self, offset: (i32, i32)) -> bool {
        let (small, large, inverse) = if self.occupied.len() <= other.occupied.len() {
            (self, other, false)
        } else {
            (other, self, true)
        };
        small.occupied.iter().any(|&(x, y)| {
            if inverse {
                // Iterating B: its translated cell is `b + offset` in A's frame.
                large.has(x + offset.0, y + offset.1)
            } else {
                // Iterating A: the corresponding local B cell is `a - offset`.
                large.has(x - offset.0, y - offset.1)
            }
        })
    }

    fn contact_count(&self, other: &Self, offset: (i32, i32)) -> usize {
        const DIRS: [(i32, i32); 4] = [(1, 0), (-1, 0), (0, 1), (0, -1)];
        self.boundary
            .iter()
            .filter(|&&(x, y)| {
                DIRS.iter()
                    .any(|&(dx, dy)| other.has(x + dx - offset.0, y + dy - offset.1))
            })
            .count()
    }
}

struct FftBoundaryCorrelation {
    offsets: Vec<((i32, i32), usize)>,
    fft_grid: (usize, usize),
}

#[derive(Clone, Copy, Debug, Default)]
struct Complex32 {
    re: f32,
    im: f32,
}

impl Complex32 {
    fn mul(self, rhs: Self) -> Self {
        Self {
            re: self.re * rhs.re - self.im * rhs.im,
            im: self.re * rhs.im + self.im * rhs.re,
        }
    }
}

/// Linear boundary cross-correlation through a zero-padded radix-2 2D FFT.
///
/// Convolving A with spatially reversed B yields support at `(ax - bx, ay - by)`.
/// Four axis neighbours turn those coincident-boundary displacements into the same
/// contact offsets the old direct O(|A boundary|*|B boundary|) loop generated. The
/// transform dimensions are bounded by the sanitised raster-axis cap.
fn fft_boundary_correlation_offsets(
    a: &RasterMask,
    b: &RasterMask,
    control: &mut GenerationControl<'_>,
) -> Option<FftBoundaryCorrelation> {
    if control.stop_requested() || a.boundary.is_empty() || b.boundary.is_empty() {
        return None;
    }
    let linear_width = (a.width + b.width - 1) as usize;
    let linear_height = (a.height + b.height - 1) as usize;
    let fft_width = linear_width.checked_next_power_of_two()?;
    let fft_height = linear_height.checked_next_power_of_two()?;
    const MAX_FFT_CELLS: usize = 128 * 128;
    if fft_width.checked_mul(fft_height)? > MAX_FFT_CELLS {
        return None;
    }

    let mut lhs = vec![Complex32::default(); fft_width * fft_height];
    let mut rhs = vec![Complex32::default(); fft_width * fft_height];
    for &(x, y) in &a.boundary {
        if control.stop_requested() {
            return None;
        }
        lhs[y as usize * fft_width + x as usize].re = 1.0;
    }
    for &(x, y) in &b.boundary {
        if control.stop_requested() {
            return None;
        }
        let reverse_x = (b.width - 1 - x) as usize;
        let reverse_y = (b.height - 1 - y) as usize;
        rhs[reverse_y * fft_width + reverse_x].re = 1.0;
    }

    if !fft2d_radix2(&mut lhs, fft_width, fft_height, false, control)
        || !fft2d_radix2(&mut rhs, fft_width, fft_height, false, control)
    {
        return None;
    }
    for index in 0..lhs.len() {
        if index % 256 == 0 && control.stop_requested() {
            return None;
        }
        lhs[index] = lhs[index].mul(rhs[index]);
    }
    if !fft2d_radix2(&mut lhs, fft_width, fft_height, true, control) {
        return None;
    }

    const DIRS: [(i32, i32); 4] = [(1, 0), (-1, 0), (0, 1), (0, -1)];
    let mut offset_support = BTreeMap::<(i32, i32), usize>::new();
    for y in 0..linear_height {
        if control.stop_requested() {
            return None;
        }
        for x in 0..linear_width {
            let support = lhs[y * fft_width + x].re.round().max(0.0) as usize;
            if support == 0 {
                continue;
            }
            let displacement = (x as i32 - (b.width - 1), y as i32 - (b.height - 1));
            for (dx, dy) in DIRS {
                offset_support
                    .entry((displacement.0 + dx, displacement.1 + dy))
                    .and_modify(|current| *current = (*current).max(support))
                    .or_insert(support);
            }
        }
    }
    Some(FftBoundaryCorrelation {
        offsets: offset_support.into_iter().collect(),
        fft_grid: (fft_width, fft_height),
    })
}

fn fft2d_radix2(
    values: &mut [Complex32],
    width: usize,
    height: usize,
    inverse: bool,
    control: &mut GenerationControl<'_>,
) -> bool {
    if !width.is_power_of_two() || !height.is_power_of_two() || values.len() != width * height {
        return false;
    }
    for row in values.chunks_exact_mut(width) {
        if !fft1d_radix2(row, inverse, control) {
            return false;
        }
    }
    let mut column = vec![Complex32::default(); height];
    for x in 0..width {
        if control.stop_requested() {
            return false;
        }
        for y in 0..height {
            column[y] = values[y * width + x];
        }
        if !fft1d_radix2(&mut column, inverse, control) {
            return false;
        }
        for y in 0..height {
            values[y * width + x] = column[y];
        }
    }
    true
}

fn fft1d_radix2(
    values: &mut [Complex32],
    inverse: bool,
    control: &mut GenerationControl<'_>,
) -> bool {
    let n = values.len();
    if !n.is_power_of_two() {
        return false;
    }
    let mut j = 0usize;
    for i in 1..n {
        if i % 64 == 0 && control.stop_requested() {
            return false;
        }
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            values.swap(i, j);
        }
    }

    let mut length = 2usize;
    while length <= n {
        if control.stop_requested() {
            return false;
        }
        let angle = if inverse { TAU } else { -TAU } / length as f32;
        let (sin, cos) = angle.sin_cos();
        let root = Complex32 { re: cos, im: sin };
        for start in (0..n).step_by(length) {
            if control.stop_requested() {
                return false;
            }
            let mut twiddle = Complex32 { re: 1.0, im: 0.0 };
            for offset in 0..(length / 2) {
                let even = values[start + offset];
                let odd = values[start + offset + length / 2].mul(twiddle);
                values[start + offset] = Complex32 {
                    re: even.re + odd.re,
                    im: even.im + odd.im,
                };
                values[start + offset + length / 2] = Complex32 {
                    re: even.re - odd.re,
                    im: even.im - odd.im,
                };
                twiddle = twiddle.mul(root);
            }
        }
        length <<= 1;
    }
    if inverse {
        let scale = n as f32;
        for value in values {
            value.re /= scale;
            value.im /= scale;
        }
    }
    true
}

fn union_grid_extent(a: &RasterMask, b: &RasterMask, offset: (i32, i32)) -> (i32, i32) {
    let min_x = 0.min(offset.0);
    let min_y = 0.min(offset.1);
    let max_x = a.width.max(offset.0 + b.width);
    let max_y = a.height.max(offset.1 + b.height);
    ((max_x - min_x).max(1), (max_y - min_y).max(1))
}

fn contact_ratio(contacts: usize, a_boundary: usize, b_boundary: usize) -> f32 {
    let scale = (a_boundary.min(b_boundary)).max(1) as f32;
    (contacts as f32 / scale).clamp(0.0, 1.0)
}

fn layout_band<'a>(
    variants: &'a [OrientedPart<'a>],
    pattern: BandPattern,
    gap: f32,
) -> Vec<MotifPoseRef<'a>> {
    let mut result = Vec::with_capacity(variants.len());
    match pattern {
        BandPattern::Horizontal => {
            let mut x = 0.0;
            for variant in variants {
                let (px, py) = variant.pose.translation();
                result.push(MotifPoseRef {
                    part: variant.part,
                    relative_pose: DTransformation::new(variant.pose.rotation(), (px + x, py)),
                });
                x += variant.rect.width + gap;
            }
        }
        BandPattern::Vertical => {
            let mut y = 0.0;
            for variant in variants {
                let (px, py) = variant.pose.translation();
                result.push(MotifPoseRef {
                    part: variant.part,
                    relative_pose: DTransformation::new(variant.pose.rotation(), (px, py + y)),
                });
                y += variant.rect.height + gap;
            }
        }
        BandPattern::StaggeredLattice => {
            let columns = (variants.len() as f32).sqrt().ceil() as usize;
            let cell_w = variants
                .iter()
                .map(|variant| variant.rect.width)
                .fold(0.0_f32, f32::max)
                + gap;
            let cell_h = variants
                .iter()
                .map(|variant| variant.rect.height)
                .fold(0.0_f32, f32::max)
                + gap;
            for (index, variant) in variants.iter().enumerate() {
                let row = index / columns;
                let column = index % columns;
                let stagger = if row % 2 == 1 { cell_w * 0.5 } else { 0.0 };
                let (px, py) = variant.pose.translation();
                result.push(MotifPoseRef {
                    part: variant.part,
                    relative_pose: DTransformation::new(
                        variant.pose.rotation(),
                        (
                            px + column as f32 * cell_w + stagger,
                            py + row as f32 * cell_h,
                        ),
                    ),
                });
            }
        }
    }
    result
}

fn band_grid_shape(pattern: BandPattern, count: usize) -> (usize, usize) {
    match pattern {
        BandPattern::Horizontal => (1, count),
        BandPattern::Vertical => (count, 1),
        BandPattern::StaggeredLattice => {
            let columns = (count as f32).sqrt().ceil() as usize;
            (count.div_ceil(columns), columns)
        }
    }
}

fn discover_families<'a>(
    parts: &'a [MotifPart],
    config: &MotifConfig,
    control: &mut GenerationControl<'_>,
) -> Vec<Vec<&'a MotifPart>> {
    let mut remaining: Vec<_> = parts.iter().collect();
    remaining.sort_by(|a, b| {
        a.key
            .item_id
            .cmp(&b.key.item_id)
            .then_with(|| a.key.copy_index.cmp(&b.key.copy_index))
    });
    let mut used = vec![false; remaining.len()];
    let mut families = Vec::new();
    for i in 0..remaining.len() {
        if control.stop_requested() {
            break;
        }
        if used[i] {
            continue;
        }
        used[i] = true;
        let representative = remaining[i];
        let mut family = vec![representative];
        for j in (i + 1)..remaining.len() {
            if control.stop_requested() {
                break;
            }
            if !used[j]
                && (representative.key.item_id == remaining[j].key.item_id
                    || family_similarity(representative, remaining[j], config) > 0.0)
            {
                used[j] = true;
                family.push(remaining[j]);
            }
        }
        if family.len() >= config.min_band_members {
            families.push(family);
        }
    }
    families
}

fn family_similarity(a: &MotifPart, b: &MotifPart, config: &MotifConfig) -> f32 {
    if a.key.item_id == b.key.item_id {
        return 1.0;
    }
    let sa = shape_signature(a);
    let sb = shape_signature(b);
    let area_delta = relative_delta(sa.area, sb.area);
    let aspect_delta = relative_delta(sa.aspect, sb.aspect);
    let rect_delta = (sa.rectangularity - sb.rectangularity).abs();
    if area_delta > config.family_area_tolerance
        || aspect_delta > config.family_aspect_tolerance
        || rect_delta > config.family_rectangularity_tolerance
    {
        return 0.0;
    }
    let area_score = 1.0 - area_delta / config.family_area_tolerance.max(1.0e-6);
    let aspect_score = 1.0 - aspect_delta / config.family_aspect_tolerance.max(1.0e-6);
    let rect_score = 1.0 - rect_delta / config.family_rectangularity_tolerance.max(1.0e-6);
    ((area_score + aspect_score + rect_score) / 3.0).clamp(0.0, 1.0)
}

fn mean_family_similarity(parts: &[&MotifPart], config: &MotifConfig) -> f32 {
    let Some(representative) = parts.first() else {
        return 0.0;
    };
    parts
        .iter()
        .map(|part| family_similarity(representative, part, config))
        .sum::<f32>()
        / parts.len() as f32
}

#[derive(Clone, Copy)]
struct ShapeSignature {
    area: f32,
    aspect: f32,
    rectangularity: f32,
}

fn shape_signature(part: &MotifPart) -> ShapeSignature {
    let bbox = part.shape.bbox();
    let short = bbox.width().min(bbox.height()).max(1.0e-6);
    let long = bbox.width().max(bbox.height());
    ShapeSignature {
        area: part.shape.area(),
        aspect: long / short,
        rectangularity: (part.shape.area() / bbox.area().max(1.0e-6)).clamp(0.0, 1.0),
    }
}

fn relative_delta(a: f32, b: f32) -> f32 {
    (a - b).abs() / a.abs().max(b.abs()).max(1.0e-6)
}

fn band_contact_proxy(poses: &[MotifPoseRef<'_>], gap: f32) -> f32 {
    if poses.len() < 2 {
        return 0.0;
    }
    let bboxes: Vec<_> = poses.iter().map(placed_bbox).collect();
    let scale = bboxes
        .iter()
        .map(|bbox| bbox.width().min(bbox.height()))
        .sum::<f32>()
        / bboxes.len() as f32;
    (1.0 - gap.max(0.0) / scale.max(1.0e-6)).clamp(0.0, 1.0)
}

fn normalise_poses(poses: &mut [MotifPoseRef<'_>]) {
    let min_x = poses
        .iter()
        .map(|pose| placed_bbox(pose).x_min)
        .fold(f32::INFINITY, f32::min);
    let min_y = poses
        .iter()
        .map(|pose| placed_bbox(pose).y_min)
        .fold(f32::INFINITY, f32::min);
    for pose in poses {
        let (x, y) = pose.relative_pose.translation();
        pose.relative_pose =
            DTransformation::new(pose.relative_pose.rotation(), (x - min_x, y - min_y));
    }
}

fn envelope_for(poses: &[MotifPoseRef<'_>]) -> MotifEnvelope {
    let mut bboxes = poses.iter().map(placed_bbox);
    let first = bboxes.next().expect(crate::dmsg!("a motif has at least one member"));
    let bbox = bboxes.fold(first, Rect::bounding_rect);
    let member_area = poses.iter().map(|pose| pose.part.shape.area()).sum::<f32>();
    MotifEnvelope {
        bbox,
        member_area,
        fill_ratio: (member_area / bbox.area().max(1.0e-6)).clamp(0.0, 1.0),
    }
}

fn placed_bbox(pose: &MotifPoseRef<'_>) -> Rect {
    let mut shape = (*pose.part.shape).clone();
    shape.transform(&Transformation::from(&pose.relative_pose));
    shape.bbox()
}

fn transformed_polygon(pose: &MotifPoseRef<'_>) -> SPolygon {
    let mut shape = (*pose.part.shape).clone();
    shape.transform(&Transformation::from(&pose.relative_pose));
    shape
}

fn has_polygon_interior_overlap(poses: &[MotifPoseRef<'_>]) -> bool {
    // Transform each pose once; the pair loop previously re-transformed every
    // `b` polygon for every `a`, i.e. O(n^2) clones instead of O(n).
    let polygons: Vec<SPolygon> = poses.iter().map(transformed_polygon).collect();
    for (i, a) in polygons.iter().enumerate() {
        for b in &polygons[(i + 1)..] {
            if polygons_interior_overlap(a, b) {
                return true;
            }
        }
    }
    false
}

/// Exact (non-raster) polygon-interior overlap. Boundary-only contact is permitted here;
/// the consumer's CDE remains authoritative about buffers and contact tolerance.
fn polygons_interior_overlap(a: &SPolygon, b: &SPolygon) -> bool {
    if Rect::intersection(a.bbox(), b.bbox()).is_none() {
        return false;
    }
    for ea in a.edge_iter() {
        for eb in b.edge_iter() {
            if segments_properly_intersect(ea.start, ea.end, eb.start, eb.end) {
                return true;
            }
        }
    }
    a.vertices()
        .iter()
        .copied()
        .any(|point| point_in_polygon_strict(point, b))
        || b.vertices()
            .iter()
            .copied()
            .any(|point| point_in_polygon_strict(point, a))
        // Vertex-only tests miss coincident polygons and collinear, partially
        // overlapping rectangles. The pole and edge midpoints add known boundary/
        // interior witnesses without treating a shared boundary as area overlap.
        || point_in_polygon_strict(a.poi().center, b)
        || point_in_polygon_strict(b.poi().center, a)
        || a.edge_iter().any(|edge| {
            point_in_polygon_strict(
                Point(
                    (edge.start.0 + edge.end.0) * 0.5,
                    (edge.start.1 + edge.end.1) * 0.5,
                ),
                b,
            )
        })
        || b.edge_iter().any(|edge| {
            point_in_polygon_strict(
                Point(
                    (edge.start.0 + edge.end.0) * 0.5,
                    (edge.start.1 + edge.end.1) * 0.5,
                ),
                a,
            )
        })
}

/// `cross` 부호 판정용 **상대** 허용오차 계수.
///
/// f32 상대정밀도(약 1.2e-7)에 여유를 준 값이다. 절대 임계가 아니라 피연산자 크기에
/// 곱해 쓰므로, 좌표계가 mm 든 m 든 인치든 판정이 같다.
const CROSS_REL_EPS: f32 = 1.0e-5;

/// 두 선분이 **제대로** 교차하는가 (끝점 접촉·공선은 제외).
///
/// # 왜 상대 허용오차인가
///
/// [`cross`] 는 mm² 차원이다. 예전에는 고정 절대 임계 `1.0e-6` 을 썼는데, 그러면
/// 임계의 의미가 좌표 크기에 따라 달라진다. 의류 마커(좌표 ~7000mm, 변 길이 ~100mm)
/// 에서 `cross` 의 f32 계산 오차만 해도
///
/// ```text
/// 7000 x 100 x 1.2e-7 ~= 0.08 mm²
/// ```
///
/// 이라, `1e-6` 임계는 **잡음보다 다섯 자리 작다.** 거의 공선인 두 변이 잡음만으로
/// "제대로 교차한다" 로 판정될 수 있고, 그러면 겹치지 않는 모티프 구성원이 겹친 것으로
/// 걸러진다. 반대로 작은 인스턴스(`fu.json`, 좌표 ~30mm)에서는 같은 임계가 잡음보다
/// 훨씬 커서 실제 교차를 놓친다. **같은 코드가 인스턴스 크기에 따라 다르게 동작한다.**
///
/// 임계를 피연산자 크기에 비례시키면 그 의존이 사라진다.
fn segments_properly_intersect(a: Point, b: Point, c: Point, d: Point) -> bool {
    let ab_c = cross(a, b, c);
    let ab_d = cross(a, b, d);
    let cd_a = cross(c, d, a);
    let cd_b = cross(c, d, b);
    // 각 외적의 크기 눈금 = 두 변 벡터의 L1 노름 곱. `cross` 가 계산하는 곱의 크기와
    // 같은 차원이라, 여기에 상대정밀도를 곱하면 그 계산의 잡음 상한이 된다.
    let ab_scale = span(a, b) * span(a, c).max(span(a, d));
    let cd_scale = span(c, d) * span(c, a).max(span(c, b));
    let eps_ab = CROSS_REL_EPS * ab_scale;
    let eps_cd = CROSS_REL_EPS * cd_scale;
    ((ab_c > eps_ab && ab_d < -eps_ab) || (ab_c < -eps_ab && ab_d > eps_ab))
        && ((cd_a > eps_cd && cd_b < -eps_cd) || (cd_a < -eps_cd && cd_b > eps_cd))
}

/// 두 점을 잇는 벡터의 L1 노름. 외적 크기의 눈금을 잡는 데만 쓴다.
#[inline]
fn span(p: Point, q: Point) -> f32 {
    (q.0 - p.0).abs() + (q.1 - p.1).abs()
}

fn cross(a: Point, b: Point, c: Point) -> f32 {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

fn point_in_polygon_inclusive(point: Point, polygon: &SPolygon) -> bool {
    // Boundary test once, then the raw ray cast; going through the strict
    // variant would repeat the O(edges) boundary scan for interior points.
    point_on_polygon_boundary(point, polygon) || ray_cast_inside(point, polygon)
}

fn point_in_polygon_strict(point: Point, polygon: &SPolygon) -> bool {
    !point_on_polygon_boundary(point, polygon) && ray_cast_inside(point, polygon)
}

/// Even-odd ray cast without any boundary handling; callers decide how
/// on-boundary points are classified.
fn ray_cast_inside(point: Point, polygon: &SPolygon) -> bool {
    let mut inside = false;
    let mut previous = *polygon.vertices().last().expect(crate::dmsg!("polygon has vertices"));
    for &current in polygon.vertices() {
        let straddles = (current.1 > point.1) != (previous.1 > point.1);
        if straddles {
            let x = (previous.0 - current.0) * (point.1 - current.1) / (previous.1 - current.1)
                + current.0;
            if point.0 < x {
                inside = !inside;
            }
        }
        previous = current;
    }
    inside
}

fn point_on_polygon_boundary(point: Point, polygon: &SPolygon) -> bool {
    polygon
        .edge_iter()
        .any(|edge| point_on_segment(point, edge.start, edge.end))
}

fn point_on_segment(point: Point, a: Point, b: Point) -> bool {
    let scale = (b.0 - a.0).abs().max((b.1 - a.1).abs()).max(1.0);
    let eps = 1.0e-6 * scale;
    cross(a, b, point).abs() <= eps
        && point.0 >= a.0.min(b.0) - eps
        && point.0 <= a.0.max(b.0) + eps
        && point.1 >= a.1.min(b.1) - eps
        && point.1 <= a.1.max(b.1) + eps
}

fn to_members(poses: &[MotifPoseRef<'_>]) -> Vec<MotifMember> {
    poses
        .iter()
        .map(|pose| MotifMember {
            item: pose.part.key,
            relative_pose: pose.relative_pose,
        })
        .collect()
}

#[derive(Clone, Copy)]
struct ScoreComponents {
    envelope_fill: f32,
    boundary_match: f32,
    rectangularity: f32,
    repetition: f32,
    member_bonus: f32,
}

fn score(components: ScoreComponents, weights: MotifScoreWeights) -> MotifScore {
    let total = components.envelope_fill * weights.envelope_fill
        + components.boundary_match * weights.boundary_match
        + components.rectangularity * weights.rectangularity
        + components.repetition * weights.repetition
        + components.member_bonus * weights.member_bonus;
    MotifScore {
        total,
        envelope_fill: components.envelope_fill,
        boundary_match: components.boundary_match,
        rectangularity: components.rectangularity,
        repetition: components.repetition,
        member_bonus: components.member_bonus,
    }
}

fn member_bonus(count: usize) -> f32 {
    ((count as f32).ln_1p() / 16.0_f32.ln_1p()).clamp(0.0, 1.0)
}

fn compare_motifs(a: &&PlacementMotif, b: &&PlacementMotif) -> std::cmp::Ordering {
    OrderedFloat(b.score.total)
        .cmp(&OrderedFloat(a.score.total))
        .then_with(|| b.members.len().cmp(&a.members.len()))
        .then_with(|| motif_identity(a).cmp(&motif_identity(b)))
}

fn motif_identity(motif: &PlacementMotif) -> Vec<(MotifItemRef, i64, i64, i64)> {
    motif
        .members
        .iter()
        .map(|member| {
            let (x, y) = member.relative_pose.translation();
            (
                member.item,
                quantise(member.relative_pose.rotation()),
                quantise(x),
                quantise(y),
            )
        })
        .collect()
}

fn quantise(value: f32) -> i64 {
    (value as f64 * 1_000_000.0).round() as i64
}

fn rank_and_truncate(motifs: &mut Vec<PlacementMotif>, top_k: usize) {
    motifs.sort_by(|a, b| compare_motifs(&a, &b));
    motifs.truncate(top_k);
}

fn deduplicate_motifs(motifs: Vec<PlacementMotif>) -> Vec<PlacementMotif> {
    let mut seen = HashSet::new();
    motifs
        .into_iter()
        .filter(|motif| seen.insert(motif_identity(motif)))
        .collect()
}

fn rotations_for_range(range: &RotationRange, continuous_samples: usize) -> Vec<f32> {
    match range {
        RotationRange::None => vec![0.0],
        RotationRange::Discrete(rotations) => normalise_angles(rotations.iter().copied()),
        RotationRange::Continuous => (0..continuous_samples.max(1))
            .map(|index| index as f32 * TAU / continuous_samples.max(1) as f32)
            .collect(),
    }
}

fn normalise_angles(angles: impl IntoIterator<Item = f32>) -> Vec<f32> {
    let mut result = Vec::new();
    for angle in angles {
        if !angle.is_finite() {
            continue;
        }
        let angle = normalize_rotation(angle);
        if !result
            .iter()
            .any(|existing: &f32| angular_distance(*existing, angle) <= 1.0e-6)
        {
            result.push(angle);
        }
    }
    result.sort_by_key(|angle| OrderedFloat(*angle));
    result
}

fn angular_distance(a: f32, b: f32) -> f32 {
    let delta = (a - b).abs() % TAU;
    delta.min(TAU - delta)
}

fn sanitise_config(mut config: MotifConfig) -> MotifConfig {
    const ABSOLUTE_OFFSET_CAP: usize = 4096;
    const ABSOLUTE_ORIENTATION_WORK_CAP: usize = 16_384;
    const ABSOLUTE_PAIR_AXIS_CAP: usize = 64;

    config.continuous_orientation_samples = config.continuous_orientation_samples.max(1);
    config.single_orientations_per_part = config.single_orientations_per_part.max(1);
    config.pair_raster_divisions = config.pair_raster_divisions.max(2);
    config.pair_max_axis_cells = config.pair_max_axis_cells.clamp(4, ABSOLUTE_PAIR_AXIS_CAP);
    config.pair_max_offset_candidates = config.pair_max_offset_candidates.min(ABSOLUTE_OFFSET_CAP);
    config.pair_max_orientation_work = config
        .pair_max_orientation_work
        .min(ABSOLUTE_ORIENTATION_WORK_CAP);
    config.pair_shortlist_per_orientation = config.pair_shortlist_per_orientation.max(1);
    config.pair_orientations_per_part = config.pair_orientations_per_part.max(1);
    config.min_band_members = config.min_band_members.max(2);
    config.max_band_members = config.max_band_members.max(config.min_band_members);
    config.band_gap = config.band_gap.max(0.0);
    config.family_area_tolerance = config.family_area_tolerance.max(0.0);
    config.family_aspect_tolerance = config.family_aspect_tolerance.max(0.0);
    config.family_rectangularity_tolerance = config.family_rectangularity_tolerance.max(0.0);
    config
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    fn polygon(points: &[(f32, f32)]) -> Arc<SPolygon> {
        Arc::new(
            SPolygon::new(points.iter().copied().map(Point::from).collect())
                .expect(crate::dmsg!("valid test polygon")),
        )
    }

    fn rectangle(width: f32, height: f32) -> Arc<SPolygon> {
        polygon(&[(0.0, 0.0), (width, 0.0), (width, height), (0.0, height)])
    }

    fn l_shape() -> Arc<SPolygon> {
        polygon(&[
            (0.0, 0.0),
            (4.0, 0.0),
            (4.0, 1.0),
            (1.0, 1.0),
            (1.0, 4.0),
            (0.0, 4.0),
        ])
    }

    fn part(item_id: usize, copy_index: usize, shape: Arc<SPolygon>) -> MotifPart {
        MotifPart::new(
            MotifItemRef {
                item_id,
                copy_index,
            },
            shape,
            [0.0, PI / 2.0],
        )
    }

    fn synthetic_motif(kind: MotifKind, items: &[(usize, usize)], total: f32) -> PlacementMotif {
        let unit = rectangle(1.0, 1.0);
        PlacementMotif {
            kind,
            members: items
                .iter()
                .map(|&(item_id, copy_index)| MotifMember {
                    item: MotifItemRef {
                        item_id,
                        copy_index,
                    },
                    relative_pose: DTransformation::new(0.0, (copy_index as f32, 0.0)),
                })
                .collect(),
            envelope: MotifEnvelope {
                bbox: unit.bbox(),
                member_area: items.len() as f32,
                fill_ratio: 1.0,
            },
            score: MotifScore {
                total,
                ..MotifScore::default()
            },
        }
    }

    fn synthetic_pair(items: [(usize, usize); 2], total: f32) -> PlacementMotif {
        synthetic_motif(
            MotifKind::Pair(PairSearchMetadata {
                raster_cell: 1.0,
                offset_cells: (1, 0),
                boundary_contacts: 1,
                correlation_method: PairCorrelationMethod::Radix2Fft2d,
                fft_grid: (2, 2),
                correlation_peak: 1,
                exact_polygon_checked: true,
                requires_cde_recheck: true,
            }),
            &items,
            total,
        )
    }

    #[test]
    fn singles_cover_allowed_orientations_with_aabb_fill() {
        let part = part(0, 0, rectangle(4.0, 2.0));
        let mut config = MotifConfig::default();
        config.top_k_singles = 8;
        config.pair_pool_size = 0;
        config.min_band_members = 4;
        let catalog = MotifGenerator::new(config).generate_parts(&[part]);

        assert_eq!(catalog.singles.len(), 2);
        assert!(catalog.pairs.is_empty());
        assert!(catalog.bands.is_empty());
        for motif in &catalog.singles {
            let MotifKind::Single(rect) = &motif.kind else {
                panic!("{}", crate::dmsg!("expected a single motif"))
            };
            assert!((rect.fill_ratio - 1.0).abs() < 1.0e-5);
            assert!(motif.is_reversible());
            assert_eq!(motif.members.len(), 1);
        }
    }

    #[test]
    fn pair_search_uses_relative_poses_and_exact_overlap_filter() {
        let parts = vec![part(0, 0, l_shape()), part(1, 0, l_shape())];
        let mut config = MotifConfig::default();
        config.single_orientations_per_part = 1;
        config.pair_orientations_per_part = 1;
        config.pair_raster_divisions = 8;
        config.pair_max_axis_cells = 16;
        config.pair_shortlist_per_orientation = 12;
        config.top_k_pairs = 12;
        config.min_band_members = 4;
        let catalog = MotifGenerator::new(config).generate_parts(&parts);

        assert!(!catalog.singles.is_empty());
        assert!(!catalog.pairs.is_empty());
        for motif in &catalog.pairs {
            let MotifKind::Pair(meta) = motif.kind else {
                panic!("{}", crate::dmsg!("expected pair metadata"))
            };
            assert!(meta.exact_polygon_checked);
            assert!(meta.requires_cde_recheck);
            assert_eq!(meta.correlation_method, PairCorrelationMethod::Radix2Fft2d);
            assert!(meta.fft_grid.0.is_power_of_two());
            assert!(meta.fft_grid.1.is_power_of_two());
            assert!(meta.correlation_peak > 0);
            assert_eq!(motif.members.len(), 2);
            let poses: Vec<_> = motif
                .members
                .iter()
                .map(|member| MotifPoseRef {
                    part: parts.iter().find(|part| part.key == member.item).unwrap(),
                    relative_pose: member.relative_pose,
                })
                .collect();
            assert!(!has_polygon_interior_overlap(&poses));
        }
    }

    #[test]
    fn repeated_family_emits_band_and_lattice_alternatives() {
        let shape = rectangle(3.0, 1.0);
        let parts: Vec<_> = (0..6)
            .map(|copy_index| part(0, copy_index, Arc::clone(&shape)))
            .collect();
        let mut config = MotifConfig::default();
        config.pair_pool_size = 0;
        config.top_k_bands = 8;
        let catalog = MotifGenerator::new(config).generate_parts(&parts);

        assert_eq!(catalog.bands.len(), 3);
        let patterns: HashSet<_> = catalog
            .bands
            .iter()
            .map(|motif| match motif.kind {
                MotifKind::RepeatedBand(meta) => meta.pattern,
                _ => panic!("{}", crate::dmsg!("expected band")),
            })
            .collect();
        assert!(patterns.contains(&BandPattern::Horizontal));
        assert!(patterns.contains(&BandPattern::Vertical));
        assert!(patterns.contains(&BandPattern::StaggeredLattice));
        assert!(catalog.bands.iter().all(|motif| motif.members.len() == 6));
    }

    struct RejectMulti;

    impl MotifPoseValidator for RejectMulti {
        fn is_feasible(&mut self, placements: &[MotifPoseRef<'_>]) -> bool {
            placements.len() == 1
        }
    }

    #[test]
    fn external_validator_can_reject_compounds_without_removing_singles() {
        let shape = rectangle(2.0, 1.0);
        let parts: Vec<_> = (0..4)
            .map(|copy_index| part(0, copy_index, Arc::clone(&shape)))
            .collect();
        let mut validator = RejectMulti;
        let catalog = MotifGenerator::new(MotifConfig::default())
            .generate_parts_with_validator(&parts, &mut validator);

        assert!(!catalog.singles.is_empty());
        assert!(catalog.pairs.is_empty());
        assert!(catalog.blocks.is_empty());
        assert!(catalog.bands.is_empty());
    }

    #[test]
    fn generation_stats_are_serializable_and_match_the_emitted_catalog() {
        let shape = rectangle(2.0, 1.0);
        let parts = (0..6)
            .map(|copy_index| part(0, copy_index, Arc::clone(&shape)))
            .collect::<Vec<_>>();
        let generator = MotifGenerator::new(MotifConfig::default());
        let legacy_catalog = generator.generate_parts(&parts);
        let result = generator.generate_parts_with_stats(&parts);

        assert_eq!(result.catalog.singles, legacy_catalog.singles);
        assert_eq!(result.catalog.pairs, legacy_catalog.pairs);
        assert_eq!(result.catalog.blocks, legacy_catalog.blocks);
        assert_eq!(result.catalog.bands, legacy_catalog.bands);
        assert_eq!(result.stats.input_parts, parts.len());
        assert!(!result.stats.generation_cancelled);
        assert_eq!(result.stats.singles.emitted, result.catalog.singles.len());
        assert_eq!(result.stats.pairs.emitted, result.catalog.pairs.len());
        assert_eq!(result.stats.rectangle.emitted, result.catalog.blocks.len());
        assert_eq!(result.stats.bands.emitted, result.catalog.bands.len());
        assert!(result.stats.singles.generated >= result.stats.singles.emitted);
        assert_eq!(result.stats.rectangle.eligible_parts, parts.len());
        assert!(result.stats.rectangle.solve_attempts > 0);
        assert!(result.stats.rectangle.solve_completed > 0);
        assert!(result.stats.rectangle.layouts_2d > 0);
        assert!(result.stats.rectangle.generated >= result.stats.rectangle.emitted);
        assert!(result.stats.pairs.orientation_work > 0);
        assert!(result.stats.pairs.offsets_shortlisted > 0);
        assert!(result.stats.bands.layout_attempts > 0);

        let json = serde_json::to_string(&result.stats).expect(crate::dmsg!("serialize generation stats"));
        let decoded: MotifGenerationStats =
            serde_json::from_str(&json).expect(crate::dmsg!("deserialize generation stats"));
        assert_eq!(decoded, result.stats);
        assert_eq!(
            serde_json::from_str::<MotifGenerationStats>("{}").unwrap(),
            MotifGenerationStats::default()
        );
    }

    #[test]
    fn generation_stats_count_exact_validator_rejections() {
        let shape = rectangle(2.0, 1.0);
        let parts = (0..4)
            .map(|copy_index| part(0, copy_index, Arc::clone(&shape)))
            .collect::<Vec<_>>();
        let mut validator = RejectMulti;
        let result = MotifGenerator::new(MotifConfig::default())
            .generate_parts_with_validator_and_stats(&parts, &mut validator);

        assert!(result.catalog.pairs.is_empty());
        assert!(result.catalog.blocks.is_empty());
        assert!(result.catalog.bands.is_empty());
        assert!(result.stats.pairs.validator_rejected > 0);
        assert!(result.stats.rectangle.validator_rejected > 0);
        assert!(result.stats.bands.validator_rejected > 0);
        assert_eq!(result.stats.pairs.generated, 0);
        assert_eq!(result.stats.rectangle.generated, 0);
        assert_eq!(result.stats.bands.generated, 0);
    }

    #[test]
    fn top_k_is_per_kind_and_disjoint_selection_never_reuses_a_copy() {
        let shape = rectangle(2.0, 1.0);
        let parts: Vec<_> = (0..6)
            .map(|copy_index| part(0, copy_index, Arc::clone(&shape)))
            .collect();
        let mut config = MotifConfig::default();
        config.top_k_singles = 3;
        config.top_k_pairs = 2;
        config.top_k_bands = 1;
        let catalog = MotifGenerator::new(config).generate_parts(&parts);
        assert!(catalog.singles.len() <= 3);
        assert!(catalog.pairs.len() <= 2);
        assert!(catalog.blocks.len() <= 1);
        assert!(catalog.bands.len() <= 1);

        let selected = catalog.disjoint_ranked(8);
        let mut used = HashSet::new();
        for motif in selected {
            for member in &motif.members {
                assert!(used.insert(member.item));
            }
        }
    }

    #[test]
    fn singles_never_block_a_composite_selection() {
        let single = synthetic_motif(
            MotifKind::Single(ApproxRectangle {
                orientation: 0.0,
                width: 1.0,
                height: 1.0,
                fill_ratio: 1.0,
            }),
            &[(0, 0)],
            100.0,
        );
        let pair = synthetic_motif(
            MotifKind::Pair(PairSearchMetadata {
                raster_cell: 1.0,
                offset_cells: (1, 0),
                boundary_contacts: 1,
                correlation_method: PairCorrelationMethod::Radix2Fft2d,
                fft_grid: (2, 2),
                correlation_peak: 1,
                exact_polygon_checked: true,
                requires_cde_recheck: true,
            }),
            &[(0, 0), (1, 0)],
            0.5,
        );
        let catalog = MotifCatalog {
            singles: vec![single],
            pairs: vec![pair],
            blocks: vec![],
            bands: vec![],
        };

        let selected = catalog.disjoint_ranked(1);
        assert_eq!(selected.len(), 1);
        assert!(matches!(selected[0].kind, MotifKind::Pair(_)));
    }

    #[test]
    fn weighted_local_improvement_can_replace_a_pair_with_a_better_band() {
        let pair = synthetic_motif(
            MotifKind::Pair(PairSearchMetadata {
                raster_cell: 1.0,
                offset_cells: (1, 0),
                boundary_contacts: 1,
                correlation_method: PairCorrelationMethod::Radix2Fft2d,
                fft_grid: (2, 2),
                correlation_peak: 1,
                exact_polygon_checked: true,
                requires_cde_recheck: true,
            }),
            &[(0, 0), (1, 0)],
            1.0,
        );
        let band = synthetic_motif(
            MotifKind::RepeatedBand(BandMetadata {
                pattern: BandPattern::Horizontal,
                rows: 1,
                columns: 4,
                gap: 0.0,
                family_similarity: 1.0,
            }),
            &[(0, 0), (2, 0), (3, 0), (4, 0)],
            0.8,
        );
        let catalog = MotifCatalog {
            singles: vec![],
            pairs: vec![pair],
            blocks: vec![],
            bands: vec![band],
        };

        let selected = catalog.disjoint_ranked(1);
        assert_eq!(selected.len(), 1);
        assert!(matches!(selected[0].kind, MotifKind::RepeatedBand(_)));
    }

    #[test]
    fn pair_mask_cache_key_is_copy_independent() {
        let first = PairMaskKey::new(7, PI / 2.0);
        let second_copy = PairMaskKey::new(7, PI / 2.0);
        let other_rotation = PairMaskKey::new(7, 0.0);
        assert_eq!(first, second_copy);
        assert_ne!(first, other_rotation);
    }

    #[test]
    fn radix2_fft_boundary_correlation_matches_direct_support_counts() {
        let a = RasterMask::from_shape(&l_shape(), 1.0);
        let b = RasterMask::from_shape(&rectangle(3.0, 2.0), 1.0);
        let mut never_stop = || false;
        let mut control = GenerationControl::new(&mut never_stop);
        let fft = fft_boundary_correlation_offsets(&a, &b, &mut control)
            .expect(crate::dmsg!("bounded FFT correlation"));

        let mut displacements = BTreeMap::<(i32, i32), usize>::new();
        for &(ax, ay) in &a.boundary {
            for &(bx, by) in &b.boundary {
                *displacements.entry((ax - bx, ay - by)).or_default() += 1;
            }
        }
        let mut expected = BTreeMap::<(i32, i32), usize>::new();
        for ((x, y), support) in displacements {
            for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                expected
                    .entry((x + dx, y + dy))
                    .and_modify(|current| *current = (*current).max(support))
                    .or_insert(support);
            }
        }
        assert_eq!(
            fft.offsets.into_iter().collect::<BTreeMap<_, _>>(),
            expected
        );
        assert!(fft.fft_grid.0.is_power_of_two());
        assert!(fft.fft_grid.1.is_power_of_two());
    }

    #[test]
    fn radix2_fft_correlation_honors_cooperative_cancel() {
        let a = RasterMask::from_shape(&rectangle(20.0, 20.0), 1.0);
        let b = RasterMask::from_shape(&l_shape(), 0.25);
        let stop_after = 1 + a.boundary.len() + b.boundary.len() + 10;
        let mut polls = 0usize;
        let mut should_stop = || {
            polls += 1;
            polls >= stop_after
        };
        let mut control = GenerationControl::new(&mut should_stop);
        assert!(fft_boundary_correlation_offsets(&a, &b, &mut control).is_none());
        assert!(polls >= stop_after);
    }

    #[test]
    fn rectangle_subsolver_emits_mixed_type_two_dimensional_blocks() {
        let wide = rectangle(2.0, 1.0);
        let square = rectangle(1.0, 1.0);
        let long = rectangle(3.0, 1.0);
        let parts = vec![
            part(0, 0, Arc::clone(&wide)),
            part(1, 0, Arc::clone(&square)),
            part(2, 0, Arc::clone(&long)),
            part(0, 1, wide),
            part(1, 1, square),
            part(2, 1, long),
        ];
        let config = MotifConfig {
            pair_pool_size: 0,
            top_k_bands: 32,
            ..MotifConfig::default()
        };
        let catalog = MotifGenerator::new(config).generate_parts(&parts);
        assert!(!catalog.blocks.is_empty());
        for motif in &catalog.blocks {
            let MotifKind::RectangleBlock(metadata) = motif.kind else {
                panic!("{}", crate::dmsg!("expected rectangle-block metadata"))
            };
            assert_eq!(metadata.method, RectangleSubsolverMethod::BoundedBottomLeft);
            assert!(metadata.search_evaluations > 0);
            assert!(metadata.search_evaluations <= 50_000);
            assert!(metadata.rows >= 2 && metadata.columns >= 2);
            assert!(metadata.item_type_count >= 2);
            assert!(metadata.exact_polygon_checked);
            assert!(metadata.requires_cde_recheck);
            let poses = motif
                .members
                .iter()
                .map(|member| MotifPoseRef {
                    part: parts.iter().find(|part| part.key == member.item).unwrap(),
                    relative_pose: member.relative_pose,
                })
                .collect::<Vec<_>>();
            assert!(!has_polygon_interior_overlap(&poses));
        }
    }

    #[test]
    fn small_pair_graph_uses_exact_maximum_weight_matching() {
        // Greedy chooses AB=9; the exact optimum is AC+BD=16.
        let catalog = MotifCatalog {
            singles: vec![],
            pairs: vec![
                synthetic_pair([(0, 0), (1, 0)], 9.0),
                synthetic_pair([(0, 0), (2, 0)], 8.0),
                synthetic_pair([(1, 0), (3, 0)], 8.0),
            ],
            blocks: vec![],
            bands: vec![],
        };
        let selection = catalog.disjoint_ranked_with_stats(2);
        assert_eq!(
            selection.stats.pair_matching_method,
            PairMatchingMethod::ExactBitmaskDp
        );
        assert_eq!(selection.stats.pair_vertices, 4);
        assert_eq!(selection.stats.pair_edges_considered, 3);
        assert!(selection.stats.pair_dp_states > 0);
        assert_eq!(selection.stats.selected_pairs, 2);
        assert_eq!(selection.motifs.len(), 2);
        assert_eq!(
            selection
                .motifs
                .iter()
                .map(|motif| motif.score.total)
                .sum::<f32>(),
            16.0
        );
    }

    #[test]
    fn large_pair_graph_reports_bounded_matching_fallback() {
        let pairs = (0..9)
            .map(|index| synthetic_pair([(index * 2, 0), (index * 2 + 1, 0)], 1.0))
            .collect::<Vec<_>>();
        let catalog = MotifCatalog {
            singles: vec![],
            pairs,
            blocks: vec![],
            bands: vec![],
        };
        let selection = catalog.disjoint_ranked_with_stats(9);
        assert_eq!(
            selection.stats.pair_matching_method,
            PairMatchingMethod::GreedyBoundedFallback
        );
        assert_eq!(selection.stats.pair_vertices, 18);
        assert_eq!(selection.stats.selected_pairs, 9);
    }

    #[test]
    fn selection_stats_count_every_channel_and_covered_members() {
        let single = synthetic_motif(
            MotifKind::Single(ApproxRectangle {
                orientation: 0.0,
                width: 1.0,
                height: 1.0,
                fill_ratio: 1.0,
            }),
            &[(0, 0)],
            1.0,
        );
        let pair = synthetic_pair([(1, 0), (2, 0)], 1.0);
        let block = synthetic_motif(
            MotifKind::RectangleBlock(RectangleBlockMetadata {
                method: RectangleSubsolverMethod::BoundedBottomLeft,
                target_width: 2.0,
                search_evaluations: 3,
                rows: 2,
                columns: 2,
                item_type_count: 3,
                exact_polygon_checked: true,
                requires_cde_recheck: true,
            }),
            &[(3, 0), (4, 0), (5, 0)],
            1.0,
        );
        let band = synthetic_motif(
            MotifKind::RepeatedBand(BandMetadata {
                pattern: BandPattern::StaggeredLattice,
                rows: 2,
                columns: 2,
                gap: 0.0,
                family_similarity: 1.0,
            }),
            &[(6, 0), (7, 0), (8, 0), (9, 0)],
            1.0,
        );
        let catalog = MotifCatalog {
            singles: vec![single],
            pairs: vec![pair],
            blocks: vec![block],
            bands: vec![band],
        };

        let selection = catalog.disjoint_ranked_with_stats(4);
        assert_eq!(selection.motifs.len(), 4);
        assert_eq!(selection.stats.selected_singles, 1);
        assert_eq!(selection.stats.selected_pairs, 1);
        assert_eq!(selection.stats.selected_blocks, 1);
        assert_eq!(selection.stats.selected_bands, 1);
        assert_eq!(selection.stats.covered_members, 10);

        let json = serde_json::to_string(&selection.stats).expect(crate::dmsg!("serialize selection stats"));
        let decoded: MotifSelectionStats =
            serde_json::from_str(&json).expect(crate::dmsg!("deserialize selection stats"));
        assert_eq!(decoded, selection.stats);
    }

    struct CountingValidator {
        calls: usize,
    }

    impl MotifPoseValidator for CountingValidator {
        fn is_feasible(&mut self, _placements: &[MotifPoseRef<'_>]) -> bool {
            self.calls += 1;
            true
        }
    }

    #[test]
    fn pair_work_and_offset_caps_bound_exact_validation() {
        let shape = l_shape();
        let parts = vec![
            part(0, 0, Arc::clone(&shape)),
            part(1, 0, Arc::clone(&shape)),
            part(2, 0, shape),
        ];
        let config = MotifConfig {
            single_orientations_per_part: 1,
            pair_orientations_per_part: 2,
            pair_max_orientation_work: 1,
            pair_max_offset_candidates: 1,
            pair_shortlist_per_orientation: 8,
            min_band_members: 4,
            ..MotifConfig::default()
        };
        let mut validator = CountingValidator { calls: 0 };
        let catalog =
            MotifGenerator::new(config).generate_parts_with_validator(&parts, &mut validator);

        assert!(validator.calls <= 1);
        assert!(catalog.pairs.len() <= 1);
    }

    #[test]
    fn cooperative_cancel_returns_a_safe_partial_catalog() {
        let shape = rectangle(2.0, 1.0);
        let parts = (0..12)
            .map(|copy_index| part(0, copy_index, Arc::clone(&shape)))
            .collect::<Vec<_>>();
        let mut polls = 0usize;
        let mut should_stop = || {
            polls += 1;
            polls >= 8
        };
        let catalog = MotifGenerator::new(MotifConfig::default())
            .generate_parts_with_control(&parts, &mut should_stop);

        assert!(polls >= 8);
        assert!(catalog.singles.len() < parts.len() * 2);
        assert!(catalog.pairs.is_empty());
        assert!(catalog.bands.is_empty());
        assert!(
            catalog
                .iter()
                .all(|motif| motif.members.iter().all(|member| member.item.item_id == 0))
        );
    }

    #[test]
    fn cooperative_cancel_is_reported_without_discarding_safe_singles() {
        let shape = rectangle(2.0, 1.0);
        let parts = (0..12)
            .map(|copy_index| part(0, copy_index, Arc::clone(&shape)))
            .collect::<Vec<_>>();
        let mut polls = 0usize;
        let mut should_stop = || {
            polls += 1;
            polls >= 8
        };
        let result = MotifGenerator::new(MotifConfig::default())
            .generate_parts_with_control_and_stats(&parts, &mut should_stop);

        assert!(result.stats.generation_cancelled);
        assert!(result.stats.singles.cancelled);
        assert_eq!(result.stats.singles.generated, result.catalog.singles.len());
        assert_eq!(result.stats.singles.emitted, result.catalog.singles.len());
        assert!(!result.catalog.singles.is_empty());
        assert_eq!(result.stats.rectangle.solve_attempts, 0);
        assert_eq!(result.stats.pairs.orientation_work, 0);
        assert_eq!(result.stats.bands.layout_attempts, 0);
    }

    #[test]
    fn translated_placements_do_not_create_rigid_rotation() {
        let part = part(0, 0, rectangle(4.0, 2.0));
        let catalog = MotifGenerator::new(MotifConfig::default()).generate_parts(&[part]);
        let motif = &catalog.singles[0];
        let before = motif.members[0].relative_pose;
        let after = motif.translated_placements((10.0, 20.0))[0].relative_pose;
        assert_eq!(before.rotation(), after.rotation());
        assert_eq!(
            after.translation(),
            (before.translation().0 + 10.0, before.translation().1 + 20.0)
        );
        assert_eq!(motif.clone().dissolve().len(), 1);
    }

    #[test]
    fn exact_overlap_distinguishes_shared_boundary_from_shared_interior() {
        let base = rectangle(2.0, 2.0);
        let transformed = |translation: (f32, f32)| {
            let mut shape = (*base).clone();
            shape.transform(&Transformation::from_translation(translation));
            shape
        };
        let a = transformed((0.0, 0.0));
        let touching = transformed((2.0, 0.0));
        let collinear_overlap = transformed((1.0, 0.0));
        let coincident = transformed((0.0, 0.0));

        assert!(!polygons_interior_overlap(&a, &touching));
        assert!(polygons_interior_overlap(&a, &collinear_overlap));
        assert!(polygons_interior_overlap(&a, &coincident));
    }

    #[test]
    fn pair_pool_prefers_type_diversity_before_later_demand_copies() {
        let large = rectangle(10.0, 10.0);
        let small = rectangle(1.0, 1.0);
        let mut parts: Vec<_> = (0..8)
            .map(|copy_index| part(0, copy_index, Arc::clone(&large)))
            .collect();
        parts.push(part(1, 0, small));

        let mut never_stop = || false;
        let mut control = GenerationControl::new(&mut never_stop);
        let pool = select_pair_pool(&parts, 2, &mut control);
        let ids: HashSet<_> = pool.iter().map(|part| part.key.item_id).collect();
        assert_eq!(ids, HashSet::from([0, 1]));
    }
}
