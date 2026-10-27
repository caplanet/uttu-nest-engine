//! Exact-authorized profile scoring for P1 construction and P4 repair.
//!
//! [`ProfileFrontier`] is deliberately only a ranking device.  This evaluator
//! first runs the same surrogate plus depth-zero exact polygon checks as the
//! legacy LBF evaluator and only then assigns a profile score.  Consequently a
//! coarse row height can change search quality, but never admit an infeasible
//! placement.

use crate::eval::lbf_evaluator::flat_exact_cde;
use crate::eval::sample_eval::{SampleEval, SampleEvaluator};
use crate::structure::profile::{
    EdgeClass, EdgeTag, ProfileBounds, ProfileError, ProfileFrontier, ProfileScoreWeights,
    TaggedPolygon, TranslationRefinementConfig, refine_translation_exact,
};
use jagua_rs::collision_detection::CDEngine;
use jagua_rs::collision_detection::hazards::filter::NoFilter;
use jagua_rs::entities::{Item, Layout};
use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::geo_enums::RotationRange;
use jagua_rs::geometry::geo_traits::TransformableFrom;
use jagua_rs::geometry::primitives::SPolygon;

/// Runtime settings for the shared P1/P4 profile ranker.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProfilePlacementConfig {
    /// Height of one horizontal profile row in geometry units.
    pub row_height: f32,
    pub max_rows: usize,
    pub weights: ProfileScoreWeights,
    /// Number of ranked basins eligible for stochastic P1 selection.
    pub top_k: usize,
    /// Maximum number of distinct remaining item/copy choices compared at one
    /// construction step. This is the explicit P1 all-remaining beam bound.
    pub item_beam_width: usize,
    /// Maximum number of seed plus random proposals evaluated for one item.
    /// Coordinate-descent probes are separately bounded by `SampleConfig`.
    pub candidate_pool_cap: usize,
    /// Exact slide-to-contact bisection/coarse-step guard per axis.
    pub slide_refine_steps: usize,
}

impl Default for ProfilePlacementConfig {
    fn default() -> Self {
        Self {
            row_height: 1.0,
            max_rows: 512,
            weights: ProfileScoreWeights::default(),
            top_k: 3,
            item_beam_width: 12,
            candidate_pool_cap: 96,
            slide_refine_steps: 24,
        }
    }
}

/// Generates a bounded set of deterministic P1 profile-alignment proposals
/// for every allowed discrete angle. Continuous rotation is sampled at four
/// cardinal angles; the ordinary random sampler remains the insurance source
/// for the rest of the continuous domain.
///
/// These are proposals only. Every returned transform must still pass the
/// exact-authorized evaluator before it can be committed.
pub fn profile_alignment_candidates(
    frontier: &ProfileFrontier,
    item: &Item,
    max_candidates: usize,
) -> Vec<DTransformation> {
    if max_candidates == 0 {
        return vec![];
    }
    let rotations: Vec<f32> = match item.allowed_orientations().rotations() {
        RotationRange::None => vec![0.0],
        RotationRange::Discrete(rotations) => rotations.clone(),
        RotationRange::Continuous => vec![
            0.0,
            std::f32::consts::FRAC_PI_2,
            std::f32::consts::PI,
            3.0 * std::f32::consts::FRAC_PI_2,
        ],
    };
    let bounds = frontier.bounds();
    // Edge classification is invariant under rigid transforms. Complex DXF
    // outlines previously repeated this O(vertices) pass for every alignment
    // anchor, multiplying it by beam width and candidate cap.
    let Ok(tagged_item) = tag_spolygon(&item.shape_cd()) else {
        return vec![];
    };
    let per_rotation = max_candidates.div_ceil(rotations.len().max(1)).max(1);
    let mut proposals = Vec::with_capacity(max_candidates);

    for rotation in rotations {
        if proposals.len() >= max_candidates {
            break;
        }
        let rotation_only = DTransformation::new(rotation, (0.0, 0.0));
        let mut rotated = item.shape_cd().as_ref().clone();
        rotated.transform_from(&item.shape_cd(), &rotation_only.compose());
        let bbox = rotated.bbox();
        if bbox.height() > bounds.height() + 1.0e-5 {
            continue;
        }
        let base_x = bounds.x_min - bbox.x_min;
        let base_y = bounds.y_min - bbox.y_min;
        let y_min = bounds.y_min - bbox.y_min;
        // A full-height item (or one within the 1e-5 tolerance above) can make
        // `bounds.y_max - bbox.y_max` fall below `y_min` by rounding; `clamp`
        // panics on an inverted range, so pin the upper bound to the lower one.
        let y_max = (bounds.y_max - bbox.y_max).max(y_min);
        let mut anchors = vec![(base_x, base_y)];

        // Sample the maintained frontier uniformly. Each selected row adds a
        // contact proposal at the right side of one occupied interval.
        let row_stride = frontier.rows().len().div_ceil(per_rotation).max(1);
        for row in frontier.rows().iter().step_by(row_stride) {
            for interval in row.intervals().iter().rev().take(2) {
                let x = interval.x_max - bbox.x_min;
                let y = (row.y_center - bbox.y_min).clamp(y_min, y_max);
                anchors.push((x, y));
                if anchors.len() >= per_rotation {
                    break;
                }
            }
            if anchors.len() >= per_rotation {
                break;
            }
        }

        for (x, y) in anchors {
            if proposals.len() >= max_candidates {
                break;
            }
            let initial = DTransformation::new(rotation, (x, y));
            let tagged = tagged_item.transformed(initial);
            let footprint = frontier.candidate_footprint(&tagged);
            let shift = frontier.suggested_left_shift(&footprint);
            let aligned = DTransformation::new(rotation, (x - shift, y));
            if !proposals.iter().any(|existing: &DTransformation| {
                angular_distance(existing.rotation(), aligned.rotation()) < 1.0e-5
                    && (existing.translation().0 - aligned.translation().0).abs() < 1.0e-4
                    && (existing.translation().1 - aligned.translation().1).abs() < 1.0e-4
            }) {
                proposals.push(aligned);
            }
        }
    }
    proposals
}

fn angular_distance(a: f32, b: f32) -> f32 {
    let delta = a - b;
    delta.sin().atan2(delta.cos()).abs()
}

/// Upper bound of the geometric tie-breaker added to [`ProfileScore::total`].
///
/// The tie-breaker is normalized by the profile span, so this constant alone bounds it:
/// the added term is always in `[0, TIE_BREAK_SCALE]` regardless of instance size.
/// It sits three orders below the smallest structural difference the score can express,
/// so it only decides genuine ties.
const TIE_BREAK_SCALE: f32 = 1.0e-6;


/// A [`SampleEvaluator`] that keeps Jagua exact CDE as the feasibility
/// authority while replacing LBF's coordinate bias with the P1 profile score.
pub struct ProfilePlacementEvaluator<'a> {
    layout: &'a Layout,
    item: &'a Item,
    item_tagged: TaggedPolygon,
    shape_buffer: SPolygon,
    /// One buffer per part for the exact collision check.
    parts: crate::util::parts::PartBuffers,
    frontier: FrontierStorage<'a>,
    weights: ProfileScoreWeights,
    exact_cde: CDEngine,
    n_evals: usize,
}

enum FrontierStorage<'a> {
    Owned(ProfileFrontier),
    Borrowed(&'a ProfileFrontier),
}

impl FrontierStorage<'_> {
    fn get(&self) -> &ProfileFrontier {
        match self {
            Self::Owned(frontier) => frontier,
            Self::Borrowed(frontier) => frontier,
        }
    }
}

impl<'a> ProfilePlacementEvaluator<'a> {
    pub fn new(
        layout: &'a Layout,
        item: &'a Item,
        config: ProfilePlacementConfig,
    ) -> Result<Self, ProfileError> {
        let frontier = build_profile_frontier(layout, config)?;

        Ok(Self {
            layout,
            item,
            item_tagged: tag_spolygon(&item.shape_cd())?,
            shape_buffer: item.shape_cd().as_ref().clone(),
            parts: crate::util::parts::PartBuffers::new(item),
            frontier: FrontierStorage::Owned(frontier),
            weights: config.weights,
            exact_cde: flat_exact_cde(layout),
            n_evals: 0,
        })
    }

    /// Reuses an incrementally maintained frontier. The caller must update it
    /// after every accepted placement and rebuild it when the strip bounds
    /// change.
    pub fn with_frontier(
        layout: &'a Layout,
        item: &'a Item,
        frontier: &'a ProfileFrontier,
        config: ProfilePlacementConfig,
    ) -> Result<Self, ProfileError> {
        Ok(Self {
            layout,
            item,
            item_tagged: tag_spolygon(&item.shape_cd())?,
            shape_buffer: item.shape_cd().as_ref().clone(),
            parts: crate::util::parts::PartBuffers::new(item),
            frontier: FrontierStorage::Borrowed(frontier),
            weights: config.weights,
            exact_cde: flat_exact_cde(layout),
            n_evals: 0,
        })
    }
}

impl SampleEvaluator for ProfilePlacementEvaluator<'_> {
    fn evaluate_sample(
        &mut self,
        transform: DTransformation,
        _upper_bound: Option<SampleEval>,
    ) -> SampleEval {
        self.n_evals += 1;
        let matrix = transform.compose();
        // Surrogate fast path and exact contour, part by part.
        if crate::util::parts::item_collides(
            self.layout.cde(),
            &self.exact_cde,
            self.item,
            &mut self.parts,
            &matrix,
            &NoFilter,
        ) {
            return SampleEval::Invalid;
        }
        self.shape_buffer
            .transform_from(&self.item.shape_cd(), &matrix);

        let tagged = self.item_tagged.transformed(transform);
        let footprint = self.frontier.get().candidate_footprint(&tagged);
        let score = self.frontier.get().score(&footprint, self.weights);

        // Stable geometric tie-breaker.
        //
        // It must stay below the resolution of the profile score, and the score is a
        // weighted sum of **normalized 0..1 terms** (`ProfileScore::total`). The previous
        // form multiplied raw millimetre coordinates by 1e-6, which is not dimensionless:
        // on the 156-panel garment marker (x up to ~7000mm, y 1524mm) it reached
        //
        //     1e-6 * (7000 + 1524) = 8.5e-3
        //
        // while two candidates differing by 0.001 in a normalized term differ by ~1e-3 in
        // `total`. The "tie-breaker" therefore outranked real structural differences —
        // and it did so **only on large instances**, since on `fu.json` (x ~ 30mm) the
        // same expression yields 3e-5. Identical code, different meaning per instance.
        // That is exactly the recurring trap: an absolute millimetre quantity compared
        // against normalized values.
        //
        // Normalizing by the profile span makes it dimensionless and instance-independent,
        // so `TIE_BREAK_SCALE` alone bounds it — it can never reach the score's resolution.
        let poi = self.shape_buffer.poi().center;
        let bounds = self.frontier.get().bounds();
        let span = ((bounds.x_max - bounds.x_min) + (bounds.y_max - bounds.y_min)).max(1.0e-6);
        let tie_break = TIE_BREAK_SCALE * ((poi.0.max(0.0) + poi.1.max(0.0)) / span);
        SampleEval::Clear {
            loss: score.total + tie_break,
        }
    }

    fn n_evals(&self) -> usize {
        self.n_evals
    }
}

/// Exact-authority slide-to-contact used immediately before a P1 candidate is
/// accepted. The cooperative callback is checked on every CDE probe. Returning
/// `None` therefore means either collision or cancellation; callers must not
/// commit the unrefined proposal after cancellation.
pub fn refine_profile_candidate_with_control(
    layout: &Layout,
    item: &Item,
    start: DTransformation,
    row_height: f32,
    max_steps: usize,
    should_stop: &mut dyn FnMut() -> bool,
) -> Option<DTransformation> {
    if should_stop() {
        return None;
    }
    let mut parts = crate::util::parts::PartBuffers::new(item);
    let exact_cde = flat_exact_cde(layout);
    let mut authority = |transform: DTransformation| {
        if should_stop() {
            return false;
        }
        let matrix = transform.compose();
        !crate::util::parts::item_collides(
            layout.cde(),
            &exact_cde,
            item,
            &mut parts,
            &matrix,
            &NoFilter,
        )
    };
    let bounds = layout.container().outer_cd().bbox();
    let (x, y) = start.translation();
    let steps = max_steps.clamp(1, 128);
    let config = TranslationRefinementConfig {
        coarse_step: row_height.max(1.0e-4),
        max_left: (x - bounds.x_min).max(0.0),
        max_down: (y - bounds.y_min).max(0.0),
        tolerance: 1.0e-4,
        max_coarse_steps_per_axis: steps,
        max_bisection_steps: steps,
    };
    refine_translation_exact(start, config, &mut authority).map(|result| result.transform())
}

pub fn build_profile_frontier(
    layout: &Layout,
    config: ProfilePlacementConfig,
) -> Result<ProfileFrontier, ProfileError> {
    let bbox = layout.container().outer_cd().bbox();
    let bounds = ProfileBounds::new(bbox.x_min, bbox.x_max, bbox.y_min, bbox.y_max)?;
    let bounded_row_height = config
        .row_height
        .max(bounds.height() / config.max_rows.max(1) as f32);
    let mut frontier = ProfileFrontier::new(bounds, bounded_row_height)?;
    for (_, placed) in layout.placed_items().iter() {
        frontier.add_polygon(&tag_spolygon(&placed.shape())?);
    }
    Ok(frontier)
}

/// Infers conservative curve/concavity tags from a tessellated polygon.
///
/// Jagua receives line segments rather than original DXF arc records.  Long
/// segments bounded by sharp turns are tagged straight; runs of small left
/// turns are convex, and any right turn is concave.  True DXF semantic tags
/// can later replace this heuristic without changing the scorer API.
pub fn tag_spolygon(polygon: &SPolygon) -> Result<TaggedPolygon, ProfileError> {
    let n = polygon.vertices().len();
    let mut vertex_turns = Vec::with_capacity(n);
    for i in 0..n {
        let prev = polygon.vertices()[(i + n - 1) % n];
        let curr = polygon.vertices()[i];
        let next = polygon.vertices()[(i + 1) % n];
        let a = (curr.0 - prev.0, curr.1 - prev.1);
        let b = (next.0 - curr.0, next.1 - curr.1);
        let cross = a.0 * b.1 - a.1 * b.0;
        let dot = a.0 * b.0 + a.1 * b.1;
        vertex_turns.push(cross.atan2(dot));
    }

    let curve_turn_limit = 20.0_f32.to_radians();
    let straight_turn_epsilon = 0.25_f32.to_radians();
    let tags = (0..n)
        .map(|i| {
            let before = vertex_turns[i];
            let after = vertex_turns[(i + 1) % n];
            let class = if before < -1.0e-4 || after < -1.0e-4 {
                EdgeClass::Concave
            } else if before.abs() <= straight_turn_epsilon || after.abs() <= straight_turn_epsilon
            {
                // Tessellators often keep intermediate collinear vertices.
                // Zero curvature is a straight run, not a convex arc.
                EdgeClass::Straight
            } else if before.abs() < curve_turn_limit && after.abs() < curve_turn_limit {
                EdgeClass::Convex
            } else {
                EdgeClass::Straight
            };
            EdgeTag::new(class)
        })
        .collect();
    TaggedPolygon::new(polygon.vertices().to_vec(), tags)
}

#[cfg(test)]
mod tests {
    use super::*;
    use jagua_rs::geometry::OriginalShape;
    use jagua_rs::geometry::fail_fast::SPSurrogateConfig;
    use jagua_rs::geometry::primitives::Point;
    use jagua_rs::geometry::shape_modification::{ShapeModifyConfig, ShapeModifyMode};

    #[test]
    fn full_height_item_with_half_turn_does_not_panic() {
        // Item height exceeds the strip height by less than the 1e-5 admission
        // tolerance, so `bounds.y_max - bbox.y_max < y_min` and the anchor
        // clamp must not see an inverted range for either rotation.
        const STRIP_H: f32 = 10.0;
        let shape = SPolygon::new(vec![
            Point(0.0, 0.0),
            Point(3.0, 0.0),
            Point(3.0, STRIP_H + 5.0e-6),
            Point(0.0, STRIP_H + 5.0e-6),
        ])
        .unwrap();
        let item = Item::new(0, (0) as u64, OriginalShape {
                shape,
                pre_transform: DTransformation::empty(),
                modify_mode: ShapeModifyMode::Inflate,
                modify_config: ShapeModifyConfig::default(),
            }, jagua_rs::geometry::AllowedOrientations::new(RotationRange::Discrete(vec![0.0, std::f32::consts::PI]), vec![]).unwrap(), None, SPSurrogateConfig {
                n_pole_limits: [(64, 0.0), (16, 0.8), (8, 0.9)],
                ff_pole_area_ratio: 0.5,
                n_ff_piers: 0,
            }).map(std::sync::Arc::new)
        .unwrap();
        let bounds = ProfileBounds::new(0.0, 100.0, 0.0, STRIP_H).unwrap();
        let frontier = ProfileFrontier::new(bounds, 1.0).unwrap();
        let proposals = profile_alignment_candidates(&frontier, &item, 8);
        assert!(!proposals.is_empty());
        assert!(proposals.iter().all(|p| p.translation().1.is_finite()));
    }

    #[test]
    fn rectangle_edges_are_tagged_straight() {
        let polygon = SPolygon::new(vec![
            Point(0.0, 0.0),
            Point(2.0, 0.0),
            Point(2.0, 1.0),
            Point(0.0, 1.0),
        ])
        .unwrap();
        let tagged = tag_spolygon(&polygon).unwrap();
        assert!(
            tagged
                .edge_tags()
                .iter()
                .all(|tag| tag.class == EdgeClass::Straight)
        );
    }

    #[test]
    fn inward_corner_marks_adjacent_edges_concave() {
        let polygon = SPolygon::new(vec![
            Point(0.0, 0.0),
            Point(2.0, 0.0),
            Point(2.0, 2.0),
            Point(1.0, 1.0),
            Point(0.0, 2.0),
        ])
        .unwrap();
        let tagged = tag_spolygon(&polygon).unwrap();
        assert!(
            tagged
                .edge_tags()
                .iter()
                .any(|tag| tag.class == EdgeClass::Concave)
        );
    }
}
