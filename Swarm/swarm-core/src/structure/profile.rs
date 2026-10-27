//! Profile-matching primitives for the experimental structure-first builder.
//!
//! This module intentionally does **not** decide geometric feasibility.  A
//! [`ProfileFrontier`] is a cheap, row-sampled model used to propose and rank
//! placements.  Every placement returned as [`AuthorizedPlacement`] has been
//! accepted by a caller supplied [`ExactPlacementAuthority`].  In production
//! that authority must be backed by Jagua's exact polygon CDE (including the
//! container), not by this sampled profile.

use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::primitives::{Point, SPolygon};
use std::cmp::Ordering;
use std::fmt::{Display, Formatter};

/// **mm 길이** 비교용 허용오차. 구간 인접·수평변 판정처럼 길이 차원에만 쓴다.
const GEOMETRY_EPS: f32 = 1.0e-6;

/// **mm² 면적** 비교용 허용오차.
///
/// 길이용 상수를 면적에 쓰면 임계의 의미가 좌표 크기에 따라 달라진다 —
/// 이 코드베이스가 반복해 밟은 함정이다. 축퇴 폴리곤을 거르는 용도이므로
/// 실제 조각 면적(mm² 단위로 최소 수백)보다 훨씬 작으면 충분하다.
const AREA_EPS: f32 = 1.0e-3;

/// Coarse boundary shape used by the profile contact score.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum EdgeClass {
    /// A line-like boundary.  It receives a bonus only against another
    /// straight boundary.
    #[default]
    Straight,
    /// An outward/convex boundary.
    Convex,
    /// An inward/concave boundary.
    Concave,
}

/// Semantic tag attached to one polygon edge.
///
/// `notch` is kept separate from [`EdgeClass`] because a notch can be encoded
/// as either a straight or curved segment by the DXF tessellator.  A notch is
/// deliberately excluded from all contact rewards: coarse profile overlap is
/// not sufficient evidence that another piece truly keys into the notch.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct EdgeTag {
    pub class: EdgeClass,
    pub notch: bool,
}

impl EdgeTag {
    pub const fn new(class: EdgeClass) -> Self {
        Self {
            class,
            notch: false,
        }
    }

    pub const fn notch(class: EdgeClass) -> Self {
        Self { class, notch: true }
    }

    /// Contact affinity used by the P1 score, in `[0, 1]`.
    ///
    /// Straight/straight and convex/concave are complementary.  All other
    /// pairs, and every pair containing a notch, receive no bonus.
    pub const fn contact_affinity(self, other: Self) -> f32 {
        if self.notch || other.notch {
            return 0.0;
        }
        match (self.class, other.class) {
            (EdgeClass::Straight, EdgeClass::Straight)
            | (EdgeClass::Convex, EdgeClass::Concave)
            | (EdgeClass::Concave, EdgeClass::Convex) => 1.0,
            _ => 0.0,
        }
    }
}

/// A simple polygon whose edge semantics survive transforms.
#[derive(Clone, Debug)]
pub struct TaggedPolygon {
    vertices: Vec<Point>,
    edge_tags: Vec<EdgeTag>,
    area: f32,
    perimeter: f32,
}

impl TaggedPolygon {
    /// Creates a tagged polygon. `edge_tags[i]` describes the edge from
    /// `vertices[i]` to `vertices[(i + 1) % len]`.
    pub fn new(vertices: Vec<Point>, edge_tags: Vec<EdgeTag>) -> Result<Self, ProfileError> {
        if vertices.len() < 3 {
            return Err(ProfileError::TooFewVertices(vertices.len()));
        }
        if vertices.len() != edge_tags.len() {
            return Err(ProfileError::TagCountMismatch {
                vertices: vertices.len(),
                tags: edge_tags.len(),
            });
        }
        if vertices
            .iter()
            .any(|p| !p.0.is_finite() || !p.1.is_finite())
        {
            return Err(ProfileError::NonFiniteCoordinate);
        }

        let signed_twice_area = vertices
            .iter()
            .zip(vertices.iter().cycle().skip(1))
            .take(vertices.len())
            .map(|(a, b)| a.0 * b.1 - b.0 * a.1)
            .sum::<f32>();
        let area = 0.5 * signed_twice_area.abs();
        if area <= AREA_EPS {
            return Err(ProfileError::DegeneratePolygon);
        }
        let perimeter = vertices
            .iter()
            .zip(vertices.iter().cycle().skip(1))
            .take(vertices.len())
            .map(|(a, b)| (b.0 - a.0).hypot(b.1 - a.1))
            .sum::<f32>();

        Ok(Self {
            vertices,
            edge_tags,
            area,
            perimeter,
        })
    }

    /// Copies a Jagua polygon while taking explicit edge tags from the DXF
    /// preprocessing stage.
    pub fn from_spolygon(
        polygon: &SPolygon,
        edge_tags: Vec<EdgeTag>,
    ) -> Result<Self, ProfileError> {
        Self::new(polygon.vertices().to_vec(), edge_tags)
    }

    /// Convenience for unclassified polygon data.  It is intentionally
    /// conservative: callers with arc/turn metadata should supply real tags.
    pub fn all_straight(polygon: &SPolygon) -> Result<Self, ProfileError> {
        Self::from_spolygon(
            polygon,
            vec![EdgeTag::new(EdgeClass::Straight); polygon.n_vertices()],
        )
    }

    pub fn vertices(&self) -> &[Point] {
        &self.vertices
    }

    pub fn edge_tags(&self) -> &[EdgeTag] {
        &self.edge_tags
    }

    pub fn area(&self) -> f32 {
        self.area
    }

    pub fn perimeter(&self) -> f32 {
        self.perimeter
    }

    /// Applies a rotation followed by a translation, matching
    /// [`DTransformation`] semantics.
    pub fn transformed(&self, transform: DTransformation) -> Self {
        let (sin, cos) = transform.rotation().sin_cos();
        let (tx, ty) = transform.translation();
        let vertices = self
            .vertices
            .iter()
            .map(|p| Point(cos * p.0 - sin * p.1 + tx, sin * p.0 + cos * p.1 + ty))
            .collect();
        Self {
            vertices,
            edge_tags: self.edge_tags.clone(),
            area: self.area,
            perimeter: self.perimeter,
        }
    }

    fn edges(&self) -> impl Iterator<Item = TaggedEdge> + '_ {
        (0..self.vertices.len()).map(|i| TaggedEdge {
            start: self.vertices[i],
            end: self.vertices[(i + 1) % self.vertices.len()],
            tag: self.edge_tags[i],
        })
    }
}

#[derive(Clone, Copy, Debug)]
struct TaggedEdge {
    start: Point,
    end: Point,
    tag: EdgeTag,
}

/// Fixed strip bounds represented by the profile.  `x_max` is retained for
/// rejecting obviously out-of-bounds candidates, but only the left, bottom,
/// and top boundaries receive anchor rewards; the right edge is the open end
/// of strip packing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProfileBounds {
    pub x_min: f32,
    pub x_max: f32,
    pub y_min: f32,
    pub y_max: f32,
}

impl ProfileBounds {
    pub fn new(x_min: f32, x_max: f32, y_min: f32, y_max: f32) -> Result<Self, ProfileError> {
        let bounds = Self {
            x_min,
            x_max,
            y_min,
            y_max,
        };
        if [x_min, x_max, y_min, y_max].iter().any(|v| !v.is_finite())
            || x_max <= x_min
            || y_max <= y_min
        {
            return Err(ProfileError::InvalidBounds);
        }
        Ok(bounds)
    }

    pub fn height(self) -> f32 {
        self.y_max - self.y_min
    }
}

/// One occupied x interval at a row sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OccupancyInterval {
    pub x_min: f32,
    pub x_max: f32,
    /// Boundary visible when approaching from the left.
    pub left_tag: EdgeTag,
    /// Boundary visible when approaching from the right.
    pub right_tag: EdgeTag,
}

impl OccupancyInterval {
    fn width(self) -> f32 {
        (self.x_max - self.x_min).max(0.0)
    }
}

/// Sampled occupancy for one horizontal row.
#[derive(Clone, Debug)]
pub struct ProfileRow {
    pub y_center: f32,
    intervals: Vec<OccupancyInterval>,
}

impl ProfileRow {
    pub fn intervals(&self) -> &[OccupancyInterval] {
        &self.intervals
    }

    /// The rightmost occupied boundary, i.e. the classic `x(y)` frontier.
    pub fn frontier_x(&self) -> Option<f32> {
        self.intervals.last().map(|i| i.x_max)
    }
}

/// Row-sampled candidate used by scoring and translation proposals.
#[derive(Clone, Debug)]
pub struct CandidateFootprint {
    rows: Vec<Vec<OccupancyInterval>>,
    area: f32,
    perimeter: f32,
    boundary_anchor_length: f32,
    outside_area: f32,
}

impl CandidateFootprint {
    pub fn row_intervals(&self, row: usize) -> &[OccupancyInterval] {
        self.rows.get(row).map(Vec::as_slice).unwrap_or(&[])
    }

    pub fn area(&self) -> f32 {
        self.area
    }

    pub fn perimeter(&self) -> f32 {
        self.perimeter
    }
}

/// Incremental `x(y)` frontier plus full row occupancy intervals.
#[derive(Clone, Debug)]
pub struct ProfileFrontier {
    bounds: ProfileBounds,
    row_height: f32,
    rows: Vec<ProfileRow>,
}

impl ProfileFrontier {
    pub fn new(bounds: ProfileBounds, row_height: f32) -> Result<Self, ProfileError> {
        if !row_height.is_finite() || row_height <= 0.0 {
            return Err(ProfileError::InvalidRowHeight(row_height));
        }
        let row_count = ((bounds.y_max - bounds.y_min) / row_height).ceil().max(1.0) as usize;
        let rows = (0..row_count)
            .map(|row| ProfileRow {
                y_center: (bounds.y_min + (row as f32 + 0.5) * row_height)
                    .min(bounds.y_max - GEOMETRY_EPS),
                intervals: Vec::new(),
            })
            .collect();
        Ok(Self {
            bounds,
            row_height,
            rows,
        })
    }

    pub fn bounds(&self) -> ProfileBounds {
        self.bounds
    }

    pub fn row_height(&self) -> f32 {
        self.row_height
    }

    pub fn rows(&self) -> &[ProfileRow] {
        &self.rows
    }

    /// Rebuilds all sampled rows from the supplied placed polygons.
    pub fn rebuild<'a>(&mut self, polygons: impl IntoIterator<Item = &'a TaggedPolygon>) {
        for row in &mut self.rows {
            row.intervals.clear();
        }
        for polygon in polygons {
            self.add_polygon(polygon);
        }
    }

    /// Adds one already-placed, world-space polygon to the occupancy model.
    pub fn add_polygon(&mut self, polygon: &TaggedPolygon) {
        let sampled = self.sample_polygon(polygon);
        for (row, additions) in self.rows.iter_mut().zip(sampled.rows) {
            if additions.is_empty() {
                continue;
            }
            row.intervals.extend(additions);
            merge_intervals(&mut row.intervals);
        }
    }

    /// Samples a world-space candidate on this profile's rows.  Sampling can
    /// be optimistic or pessimistic; exact CDE remains authoritative.
    pub fn candidate_footprint(&self, polygon: &TaggedPolygon) -> CandidateFootprint {
        self.sample_polygon(polygon)
    }

    /// Returns an inexpensive left-slide proposal.  The shift is limited by
    /// the nearest occupied interval (or the left strip boundary) on every
    /// sampled row.  It is only a proposal and must be passed to exact CDE.
    pub fn suggested_left_shift(&self, candidate: &CandidateFootprint) -> f32 {
        let mut clearance = f32::INFINITY;
        let mut saw_occupied_row = false;

        for (profile_row, candidate_row) in self.rows.iter().zip(&candidate.rows) {
            for c in candidate_row {
                saw_occupied_row = true;
                let mut anchor = self.bounds.x_min;
                for occupied in &profile_row.intervals {
                    if occupied.x_max <= c.x_min + GEOMETRY_EPS {
                        anchor = anchor.max(occupied.x_max);
                    } else if occupied.x_min < c.x_max - GEOMETRY_EPS {
                        // The sampled candidate already overlaps occupancy.
                        return 0.0;
                    } else {
                        break;
                    }
                }
                clearance = clearance.min((c.x_min - anchor).max(0.0));
            }
        }

        if saw_occupied_row && clearance.is_finite() {
            clearance
        } else {
            0.0
        }
    }

    /// Scores a sampled candidate. Lower is better.
    pub fn score(
        &self,
        candidate: &CandidateFootprint,
        weights: ProfileScoreWeights,
    ) -> ProfileScore {
        let mut gap_area = 0.0;
        let mut overlap_area = candidate.outside_area;
        let mut compatible_contact_length = 0.0;

        for (profile_row, candidate_row) in self.rows.iter().zip(&candidate.rows) {
            let mut row_gap_recorded = false;
            for c in candidate_row {
                let mut left = None;
                let mut right = None;
                for occupied in &profile_row.intervals {
                    let overlap =
                        (c.x_max.min(occupied.x_max) - c.x_min.max(occupied.x_min)).max(0.0);
                    overlap_area += overlap * self.row_height;

                    if occupied.x_max <= c.x_min + GEOMETRY_EPS {
                        left = Some(*occupied);
                    } else if occupied.x_min >= c.x_max - GEOMETRY_EPS {
                        right = Some(*occupied);
                        break;
                    }
                }

                let left_anchor = left.map_or(self.bounds.x_min, |i| i.x_max);
                let gap = (c.x_min - left_anchor).max(0.0);
                // Gap area is measured against the candidate's left envelope
                // once per row. Counting every interval would charge a
                // concave piece for its own internal notch.
                if !row_gap_recorded {
                    gap_area += gap * self.row_height;
                    row_gap_recorded = true;
                }

                // A strip boundary is scored by the separate anchor term. It
                // must not also masquerade as a straight piece edge.
                if gap <= weights.contact_tolerance && left.is_some() {
                    let anchor_tag = left.expect(crate::dmsg!("checked above")).right_tag;
                    compatible_contact_length +=
                        anchor_tag.contact_affinity(c.left_tag) * self.row_height;
                }
                if let Some(right) = right {
                    let right_gap = (right.x_min - c.x_max).max(0.0);
                    if right_gap <= weights.contact_tolerance {
                        compatible_contact_length +=
                            c.right_tag.contact_affinity(right.left_tag) * self.row_height;
                    }
                }
            }
        }

        let area_denominator = candidate.area.max(GEOMETRY_EPS);
        let perimeter_denominator = candidate.perimeter.max(GEOMETRY_EPS);
        let normalized_gap = gap_area / (area_denominator + gap_area);
        let normalized_contact =
            (compatible_contact_length / perimeter_denominator).clamp(0.0, 1.0);
        let normalized_boundary_anchor =
            (candidate.boundary_anchor_length / perimeter_denominator).clamp(0.0, 1.0);
        let normalized_overlap = (overlap_area / area_denominator).clamp(0.0, 1.0);
        let total = weights.gap * normalized_gap
            - weights.contact * normalized_contact
            - weights.boundary_anchor * normalized_boundary_anchor
            + weights.overlap * normalized_overlap;

        ProfileScore {
            total,
            normalized_gap,
            normalized_contact,
            normalized_boundary_anchor,
            normalized_overlap,
            gap_area,
            compatible_contact_length,
            boundary_anchor_length: candidate.boundary_anchor_length,
            overlap_area,
        }
    }

    fn sample_polygon(&self, polygon: &TaggedPolygon) -> CandidateFootprint {
        let mut rows = Vec::with_capacity(self.rows.len());
        for profile_row in &self.rows {
            let intervals = scanline_intervals(polygon, profile_row.y_center);
            rows.push(intervals);
        }

        let boundary_anchor_length = polygon
            .edges()
            .map(|edge| boundary_anchor_length(edge, self.bounds, self.row_height * 0.55))
            .sum();
        let outside_area = sampled_outside_y_area(polygon, self.bounds, self.row_height)
            + sampled_outside_x_area(&rows, self.bounds, self.row_height);

        CandidateFootprint {
            rows,
            area: polygon.area,
            perimeter: polygon.perimeter,
            boundary_anchor_length,
            outside_area,
        }
    }
}

/// Normalized profile score weights.  Contact tolerance uses geometry units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProfileScoreWeights {
    pub gap: f32,
    pub contact: f32,
    pub boundary_anchor: f32,
    /// Large enough to keep sampled collisions out of the top-k while still
    /// leaving exact CDE as the final authority.
    pub overlap: f32,
    pub contact_tolerance: f32,
}

impl Default for ProfileScoreWeights {
    fn default() -> Self {
        Self {
            gap: 1.0,
            contact: 0.35,
            boundary_anchor: 0.20,
            overlap: 10.0,
            contact_tolerance: 0.05,
        }
    }
}

impl ProfileScoreWeights {
    /// Useful at configuration boundaries before running a multi-start.
    pub fn is_valid(self) -> bool {
        [
            self.gap,
            self.contact,
            self.boundary_anchor,
            self.overlap,
            self.contact_tolerance,
        ]
        .iter()
        .all(|value| value.is_finite() && *value >= 0.0)
    }
}

/// Full score decomposition for diagnostics and A/B logging.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ProfileScore {
    pub total: f32,
    pub normalized_gap: f32,
    pub normalized_contact: f32,
    pub normalized_boundary_anchor: f32,
    pub normalized_overlap: f32,
    pub gap_area: f32,
    pub compatible_contact_length: f32,
    pub boundary_anchor_length: f32,
    pub overlap_area: f32,
}

/// Stable identity used as the final deterministic tie-breaker.
#[derive(Clone, Copy, Debug, Default, Eq, Ord, PartialEq, PartialOrd)]
pub struct CandidateIdentity {
    pub item_id: usize,
    pub rotation_ordinal: u32,
    pub seed_ordinal: u32,
}

/// Candidate record shared by deterministic beam and top-k stochastic paths.
#[derive(Clone, Debug, PartialEq)]
pub struct ProfileCandidate {
    pub identity: CandidateIdentity,
    pub transform: DTransformation,
    pub score: ProfileScore,
}

/// Sorts candidates in a reproducible total order.  The same set receives the
/// same ordering regardless of discovery order, which lets beam search and a
/// seeded stochastic top-k policy share one candidate pool.
pub fn rank_candidates(candidates: &mut [ProfileCandidate]) {
    candidates.sort_by(compare_candidates);
}

/// Sorts and truncates a candidate pool for either a deterministic beam or a
/// seeded stochastic selection over the returned top-k entries.
pub fn deterministic_top_k(
    mut candidates: Vec<ProfileCandidate>,
    k: usize,
) -> Vec<ProfileCandidate> {
    rank_candidates(&mut candidates);
    candidates.truncate(k.min(candidates.len()));
    candidates
}

fn compare_candidates(a: &ProfileCandidate, b: &ProfileCandidate) -> Ordering {
    a.score
        .total
        .total_cmp(&b.score.total)
        .then_with(|| {
            a.score
                .normalized_overlap
                .total_cmp(&b.score.normalized_overlap)
        })
        .then_with(|| a.score.normalized_gap.total_cmp(&b.score.normalized_gap))
        .then_with(|| {
            b.score
                .normalized_contact
                .total_cmp(&a.score.normalized_contact)
        })
        .then_with(|| {
            b.score
                .normalized_boundary_anchor
                .total_cmp(&a.score.normalized_boundary_anchor)
        })
        .then_with(|| a.identity.cmp(&b.identity))
        .then_with(|| a.transform.rotation().total_cmp(&b.transform.rotation()))
        .then_with(|| {
            let (ax, ay) = a.transform.translation();
            let (bx, by) = b.transform.translation();
            ax.total_cmp(&bx).then_with(|| ay.total_cmp(&by))
        })
}

/// Final feasibility boundary for profile candidates.
///
/// Production implementations must transform the exact polygon and call the
/// exact CDE against both placed hazards and the container.  The blanket
/// closure implementation keeps integration with existing evaluators small.
pub trait ExactPlacementAuthority {
    fn is_feasible(&mut self, transform: DTransformation) -> bool;
}

impl<F> ExactPlacementAuthority for F
where
    F: FnMut(DTransformation) -> bool,
{
    fn is_feasible(&mut self, transform: DTransformation) -> bool {
        self(transform)
    }
}

/// A placement token that can only be constructed in this module after an
/// exact-authority check.
#[derive(Clone, Debug)]
pub struct AuthorizedPlacement {
    candidate: ProfileCandidate,
    exact_checks: usize,
}

impl AuthorizedPlacement {
    pub fn candidate(&self) -> &ProfileCandidate {
        &self.candidate
    }

    pub fn transform(&self) -> DTransformation {
        self.candidate.transform
    }

    pub fn exact_checks(&self) -> usize {
        self.exact_checks
    }
}

/// Ranks a pool and returns the first exact-feasible candidate among its top-k
/// members.  A profile score alone can never produce an authorized placement.
pub fn authorize_top_k<A: ExactPlacementAuthority>(
    candidates: Vec<ProfileCandidate>,
    k: usize,
    authority: &mut A,
) -> Option<AuthorizedPlacement> {
    let candidates = deterministic_top_k(candidates, k);
    for (index, candidate) in candidates.into_iter().enumerate() {
        if authority.is_feasible(candidate.transform) {
            return Some(AuthorizedPlacement {
                candidate,
                exact_checks: index + 1,
            });
        }
    }
    None
}

/// Exact-authorizes one candidate selected by a seeded stochastic policy from
/// [`deterministic_top_k`].
pub fn authorize_candidate<A: ExactPlacementAuthority>(
    candidate: ProfileCandidate,
    authority: &mut A,
) -> Option<AuthorizedPlacement> {
    authority
        .is_feasible(candidate.transform)
        .then_some(AuthorizedPlacement {
            candidate,
            exact_checks: 1,
        })
}

/// Exact slide-to-contact configuration for candidate translation refinement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TranslationRefinementConfig {
    /// Distance between exact probes while looking for the first blocked point.
    pub coarse_step: f32,
    /// Maximum leftward travel.
    pub max_left: f32,
    /// Maximum downward travel after the left refinement.
    pub max_down: f32,
    /// Stop bisection when the feasible/infeasible bracket is this small.
    pub tolerance: f32,
    /// Hard guard against an unbounded or faulty authority implementation.
    pub max_coarse_steps_per_axis: usize,
    pub max_bisection_steps: usize,
}

impl Default for TranslationRefinementConfig {
    fn default() -> Self {
        Self {
            coarse_step: 1.0,
            max_left: f32::INFINITY,
            max_down: f32::INFINITY,
            tolerance: 1.0e-4,
            max_coarse_steps_per_axis: 4096,
            max_bisection_steps: 24,
        }
    }
}

/// Result of exact translation refinement.  The final transform has been
/// checked once more after both axes, so callers never receive an unchecked
/// profile proposal.
#[derive(Clone, Copy, Debug)]
pub struct RefinedTranslation {
    transform: DTransformation,
    exact_checks: usize,
}

impl RefinedTranslation {
    pub fn transform(self) -> DTransformation {
        self.transform
    }

    pub fn exact_checks(self) -> usize {
        self.exact_checks
    }
}

/// Slides left and then down using exact feasibility probes.  The profile may
/// supply the starting transform (for example after
/// [`ProfileFrontier::suggested_left_shift`]), but no sampled decision is
/// trusted. Returns `None` when the starting transform is not exact-feasible or
/// the configuration is invalid.
pub fn refine_translation_exact<A: ExactPlacementAuthority>(
    start: DTransformation,
    config: TranslationRefinementConfig,
    authority: &mut A,
) -> Option<RefinedTranslation> {
    if !config.coarse_step.is_finite()
        || config.coarse_step <= 0.0
        || !config.tolerance.is_finite()
        || config.tolerance <= 0.0
        || config.max_left.is_nan()
        || config.max_down.is_nan()
        || config.max_left < 0.0
        || config.max_down < 0.0
    {
        return None;
    }

    let mut exact_checks = 1;
    if !authority.is_feasible(start) {
        return None;
    }

    let mut current = start;
    current = refine_axis(
        current,
        (-1.0, 0.0),
        config.max_left,
        config,
        authority,
        &mut exact_checks,
    );
    current = refine_axis(
        current,
        (0.0, -1.0),
        config.max_down,
        config,
        authority,
        &mut exact_checks,
    );

    exact_checks += 1;
    authority
        .is_feasible(current)
        .then_some(RefinedTranslation {
            transform: current,
            exact_checks,
        })
}

fn refine_axis<A: ExactPlacementAuthority>(
    start: DTransformation,
    direction: (f32, f32),
    max_distance: f32,
    config: TranslationRefinementConfig,
    authority: &mut A,
    exact_checks: &mut usize,
) -> DTransformation {
    if max_distance == 0.0 {
        return start;
    }
    let mut feasible = start;
    let mut travelled = 0.0;

    for _ in 0..config.max_coarse_steps_per_axis {
        if travelled >= max_distance {
            break;
        }
        let step = config.coarse_step.min(max_distance - travelled);
        if !step.is_finite() || step <= 0.0 {
            break;
        }
        let trial = translated(feasible, direction, step);
        *exact_checks += 1;
        if authority.is_feasible(trial) {
            feasible = trial;
            travelled += step;
            continue;
        }

        // `lo` is known feasible and `hi` is known infeasible.  Refine the
        // first contact from the current exact-feasible point.
        let mut lo = 0.0;
        let mut hi = step;
        for _ in 0..config.max_bisection_steps {
            if hi - lo <= config.tolerance {
                break;
            }
            let mid = lo + 0.5 * (hi - lo);
            let probe = translated(feasible, direction, mid);
            *exact_checks += 1;
            if authority.is_feasible(probe) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return translated(feasible, direction, lo);
    }
    feasible
}

fn translated(transform: DTransformation, direction: (f32, f32), distance: f32) -> DTransformation {
    let (x, y) = transform.translation();
    DTransformation::new(
        transform.rotation(),
        (x + direction.0 * distance, y + direction.1 * distance),
    )
}

fn scanline_intervals(polygon: &TaggedPolygon, y: f32) -> Vec<OccupancyInterval> {
    let mut crossings = Vec::new();
    for edge in polygon.edges() {
        let y_min = edge.start.1.min(edge.end.1);
        let y_max = edge.start.1.max(edge.end.1);
        // Half-open rule counts a shared vertex exactly once. Horizontal edges
        // do not cross a row and are accounted for by boundary anchors.
        if (edge.end.1 - edge.start.1).abs() <= GEOMETRY_EPS || y < y_min || y >= y_max {
            continue;
        }
        let t = (y - edge.start.1) / (edge.end.1 - edge.start.1);
        crossings.push((edge.start.0 + t * (edge.end.0 - edge.start.0), edge.tag));
    }
    crossings.sort_by(|a, b| a.0.total_cmp(&b.0));

    let mut intervals = Vec::with_capacity(crossings.len() / 2);
    for pair in crossings.chunks_exact(2) {
        if pair[1].0 - pair[0].0 > GEOMETRY_EPS {
            intervals.push(OccupancyInterval {
                x_min: pair[0].0,
                x_max: pair[1].0,
                left_tag: pair[0].1,
                right_tag: pair[1].1,
            });
        }
    }
    intervals
}

fn merge_intervals(intervals: &mut Vec<OccupancyInterval>) {
    intervals.sort_by(|a, b| a.x_min.total_cmp(&b.x_min));
    let mut merged: Vec<OccupancyInterval> = Vec::with_capacity(intervals.len());
    for next in intervals.drain(..) {
        let Some(last) = merged.last_mut() else {
            merged.push(next);
            continue;
        };
        if next.x_min <= last.x_max + GEOMETRY_EPS {
            if next.x_max > last.x_max {
                last.x_max = next.x_max;
                last.right_tag = next.right_tag;
            }
        } else {
            merged.push(next);
        }
    }
    *intervals = merged;
}

fn boundary_anchor_length(edge: TaggedEdge, bounds: ProfileBounds, tolerance: f32) -> f32 {
    if edge.tag.notch {
        return 0.0;
    }
    let length = (edge.end.0 - edge.start.0).hypot(edge.end.1 - edge.start.1);
    let on_left = (edge.start.0 - bounds.x_min).abs() <= tolerance
        && (edge.end.0 - bounds.x_min).abs() <= tolerance;
    let on_bottom = (edge.start.1 - bounds.y_min).abs() <= tolerance
        && (edge.end.1 - bounds.y_min).abs() <= tolerance;
    let on_top = (edge.start.1 - bounds.y_max).abs() <= tolerance
        && (edge.end.1 - bounds.y_max).abs() <= tolerance;
    if on_left || on_bottom || on_top {
        length
    } else {
        0.0
    }
}

fn sampled_outside_x_area(
    rows: &[Vec<OccupancyInterval>],
    bounds: ProfileBounds,
    row_height: f32,
) -> f32 {
    rows.iter()
        .flatten()
        .map(|i| {
            ((bounds.x_min - i.x_min).max(0.0) + (i.x_max - bounds.x_max).max(0.0)) * row_height
        })
        .sum()
}

fn sampled_outside_y_area(polygon: &TaggedPolygon, bounds: ProfileBounds, row_height: f32) -> f32 {
    let y_min = polygon
        .vertices
        .iter()
        .map(|p| p.1)
        .fold(f32::INFINITY, f32::min);
    let y_max = polygon
        .vertices
        .iter()
        .map(|p| p.1)
        .fold(f32::NEG_INFINITY, f32::max);
    let mut outside = 0.0;

    let mut sample_band = |start: f32, end: f32| {
        if end <= start {
            return;
        }
        let n = ((end - start) / row_height).ceil().max(1.0) as usize;
        let sample_height = (end - start) / n as f32;
        for row in 0..n {
            let y = start + (row as f32 + 0.5) * sample_height;
            outside += scanline_intervals(polygon, y)
                .iter()
                .map(|i| i.width())
                .sum::<f32>()
                * sample_height;
        }
    };

    sample_band(y_min, y_max.min(bounds.y_min));
    sample_band(y_min.max(bounds.y_max), y_max);
    outside
}

#[derive(Clone, Debug, PartialEq)]
pub enum ProfileError {
    InvalidBounds,
    InvalidRowHeight(f32),
    TooFewVertices(usize),
    TagCountMismatch { vertices: usize, tags: usize },
    NonFiniteCoordinate,
    DegeneratePolygon,
}

impl Display for ProfileError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidBounds => write!(f, "profile bounds must be finite and strictly ordered"),
            Self::InvalidRowHeight(row_height) => {
                write!(f, "profile row height is invalid: {row_height}")
            }
            Self::TooFewVertices(n) => {
                write!(f, "a profile polygon needs at least 3 vertices, got {n}")
            }
            Self::TagCountMismatch { vertices, tags } => write!(
                f,
                "profile polygon has {vertices} vertices but {tags} edge tags"
            ),
            Self::NonFiniteCoordinate => {
                write!(f, "profile polygon contains a non-finite coordinate")
            }
            Self::DegeneratePolygon => write!(f, "profile polygon has zero area"),
        }
    }
}

impl std::error::Error for ProfileError {}

impl From<f32> for ProfileError {
    fn from(row_height: f32) -> Self {
        Self::InvalidRowHeight(row_height)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bounds() -> ProfileBounds {
        ProfileBounds::new(0.0, 30.0, 0.0, 10.0).unwrap()
    }

    fn rectangle(
        x_min: f32,
        y_min: f32,
        x_max: f32,
        y_max: f32,
        left: EdgeTag,
        right: EdgeTag,
    ) -> TaggedPolygon {
        // CCW: bottom, right, top, left.
        TaggedPolygon::new(
            vec![
                Point(x_min, y_min),
                Point(x_max, y_min),
                Point(x_max, y_max),
                Point(x_min, y_max),
            ],
            vec![
                EdgeTag::new(EdgeClass::Straight),
                right,
                EdgeTag::new(EdgeClass::Straight),
                left,
            ],
        )
        .unwrap()
    }

    #[test]
    fn edge_affinity_excludes_notches() {
        let straight = EdgeTag::new(EdgeClass::Straight);
        let convex = EdgeTag::new(EdgeClass::Convex);
        let concave = EdgeTag::new(EdgeClass::Concave);
        assert_eq!(straight.contact_affinity(straight), 1.0);
        assert_eq!(convex.contact_affinity(concave), 1.0);
        assert_eq!(convex.contact_affinity(convex), 0.0);
        assert_eq!(
            convex.contact_affinity(EdgeTag::notch(EdgeClass::Concave)),
            0.0
        );
    }

    #[test]
    fn frontier_keeps_merged_occupancy_intervals_and_x_of_y() {
        let mut profile = ProfileFrontier::new(bounds(), 1.0).unwrap();
        let tag = EdgeTag::new(EdgeClass::Straight);
        profile.add_polygon(&rectangle(0.0, 0.0, 4.0, 3.0, tag, tag));
        profile.add_polygon(&rectangle(3.0, 0.0, 7.0, 3.0, tag, tag));
        profile.add_polygon(&rectangle(10.0, 0.0, 12.0, 3.0, tag, tag));

        assert_eq!(profile.rows()[0].intervals().len(), 2);
        assert_eq!(profile.rows()[0].intervals()[0].x_min, 0.0);
        assert_eq!(profile.rows()[0].intervals()[0].x_max, 7.0);
        assert_eq!(profile.rows()[0].frontier_x(), Some(12.0));
        assert!(profile.rows()[5].intervals().is_empty());
    }

    #[test]
    fn score_rewards_matching_contact_and_anchors_but_not_notches() {
        let mut profile = ProfileFrontier::new(bounds(), 1.0).unwrap();
        profile.add_polygon(&rectangle(
            0.0,
            0.0,
            4.0,
            4.0,
            EdgeTag::new(EdgeClass::Straight),
            EdgeTag::new(EdgeClass::Convex),
        ));
        let weights = ProfileScoreWeights {
            contact_tolerance: 0.001,
            ..Default::default()
        };

        let matching = profile.candidate_footprint(&rectangle(
            4.0,
            0.0,
            7.0,
            4.0,
            EdgeTag::new(EdgeClass::Concave),
            EdgeTag::new(EdgeClass::Straight),
        ));
        let mismatching = profile.candidate_footprint(&rectangle(
            4.0,
            0.0,
            7.0,
            4.0,
            EdgeTag::new(EdgeClass::Convex),
            EdgeTag::new(EdgeClass::Straight),
        ));
        let notch = profile.candidate_footprint(&rectangle(
            4.0,
            0.0,
            7.0,
            4.0,
            EdgeTag::notch(EdgeClass::Concave),
            EdgeTag::new(EdgeClass::Straight),
        ));
        let far = profile.candidate_footprint(&rectangle(
            8.0,
            2.0,
            11.0,
            6.0,
            EdgeTag::new(EdgeClass::Concave),
            EdgeTag::new(EdgeClass::Straight),
        ));

        let matching_score = profile.score(&matching, weights);
        let mismatching_score = profile.score(&mismatching, weights);
        let notch_score = profile.score(&notch, weights);
        let far_score = profile.score(&far, weights);
        assert!(matching_score.normalized_contact > 0.0);
        assert_eq!(notch_score.normalized_contact, 0.0);
        assert!(matching_score.total < mismatching_score.total);
        assert!(matching_score.total < far_score.total);
        assert!(matching_score.normalized_boundary_anchor > 0.0);
        for value in [
            matching_score.normalized_gap,
            matching_score.normalized_contact,
            matching_score.normalized_boundary_anchor,
            matching_score.normalized_overlap,
        ] {
            assert!((0.0..=1.0).contains(&value));
        }
    }

    #[test]
    fn left_shift_uses_nearest_occupancy_or_strip_boundary() {
        let mut profile = ProfileFrontier::new(bounds(), 1.0).unwrap();
        let tag = EdgeTag::new(EdgeClass::Straight);
        profile.add_polygon(&rectangle(0.0, 0.0, 4.0, 4.0, tag, tag));
        let footprint = profile.candidate_footprint(&rectangle(9.0, 0.0, 12.0, 4.0, tag, tag));
        assert!((profile.suggested_left_shift(&footprint) - 5.0).abs() < 1.0e-6);
    }

    fn candidate(id: usize, total: f32) -> ProfileCandidate {
        ProfileCandidate {
            identity: CandidateIdentity {
                item_id: id,
                rotation_ordinal: 0,
                seed_ordinal: 0,
            },
            transform: DTransformation::new(0.0, (id as f32, 0.0)),
            score: ProfileScore {
                total,
                ..Default::default()
            },
        }
    }

    #[test]
    fn ranking_is_deterministic_and_exact_authority_has_final_say() {
        let input_a = vec![candidate(3, 1.0), candidate(1, 1.0), candidate(2, 0.5)];
        let mut input_b = input_a.clone();
        input_b.reverse();
        let ranked_a = deterministic_top_k(input_a.clone(), 3);
        let ranked_b = deterministic_top_k(input_b, 3);
        let ids_a: Vec<_> = ranked_a.iter().map(|c| c.identity.item_id).collect();
        let ids_b: Vec<_> = ranked_b.iter().map(|c| c.identity.item_id).collect();
        assert_eq!(ids_a, vec![2, 1, 3]);
        assert_eq!(ids_a, ids_b);

        let mut exact = |dt: DTransformation| dt.translation().0 == 3.0;
        let authorized = authorize_top_k(input_a, 3, &mut exact).unwrap();
        assert_eq!(authorized.candidate().identity.item_id, 3);
        assert_eq!(authorized.exact_checks(), 3);
    }

    #[test]
    fn refinement_returns_only_a_final_exact_feasible_transform() {
        let start = DTransformation::new(0.25, (10.0, 8.0));
        let mut exact = |dt: DTransformation| {
            let (x, y) = dt.translation();
            x >= 3.0 && y >= 2.0
        };
        let result = refine_translation_exact(
            start,
            TranslationRefinementConfig {
                coarse_step: 2.0,
                max_left: 20.0,
                max_down: 20.0,
                tolerance: 1.0e-5,
                max_coarse_steps_per_axis: 100,
                max_bisection_steps: 32,
            },
            &mut exact,
        )
        .unwrap();
        let (x, y) = result.transform().translation();
        assert!((x - 3.0).abs() < 2.0e-5, "x={x}");
        assert!((y - 2.0).abs() < 2.0e-5, "y={y}");
        assert_eq!(result.transform().rotation(), 0.25);
        assert!(exact(result.transform()));
        assert!(result.exact_checks() > 2);

        let mut never = |_dt: DTransformation| false;
        assert!(refine_translation_exact(start, Default::default(), &mut never).is_none());
    }
}
