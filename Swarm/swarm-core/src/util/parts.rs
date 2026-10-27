//! Helpers for multipart items (jagua-rs fork change 10).
//!
//! A multipart item is several disjoint polygons that move as one rigid body.
//! The engine registers one hazard per part, so every exact collision question
//! has to look at all parts of a placement; the item's `shape_cd()` is only the
//! convex hull (an envelope for sampling ranges and bounding boxes). Ordinary
//! items have one part that *is* the hull, so these helpers cost nothing extra
//! for them.
use jagua_rs::collision_detection::CDEngine;
use jagua_rs::collision_detection::hazards::BasicHazardEntity;
use jagua_rs::collision_detection::hazards::filter::{HazardFilter, PlacedItemFilter};
use jagua_rs::entities::{Item, Layout, PItemKey, PlacedItem};
use jagua_rs::geometry::Transformation;
use jagua_rs::geometry::geo_traits::TransformableFrom;
use jagua_rs::geometry::primitives::SPolygon;

/// Transformed collision contours of an item's parts, reused across candidate poses.
pub struct PartBuffers {
    shapes: Vec<SPolygon>,
}

impl PartBuffers {
    pub fn new(item: &Item) -> Self {
        Self {
            shapes: item.parts_cd().iter().map(|part| part.as_ref().clone()).collect(),
        }
    }

    /// Re-targets the buffers to another item, reusing the vertex allocations.
    pub fn reset(&mut self, item: &Item) {
        self.shapes.resize_with(item.n_parts(), || item.shape_cd().as_ref().clone());
        for (buffer, part) in self.shapes.iter_mut().zip(item.parts_cd()) {
            buffer.clone_from(part.as_ref());
        }
    }

    /// Transforms every part of `item` by `t` into the buffers.
    pub fn transform(&mut self, item: &Item, t: &Transformation) -> &[SPolygon] {
        for (buffer, part) in self.shapes.iter_mut().zip(item.parts_cd()) {
            buffer.transform_from(part.as_ref(), t);
        }
        &self.shapes
    }

    pub fn shapes(&self) -> &[SPolygon] {
        &self.shapes
    }

    pub fn shapes_mut(&mut self) -> &mut [SPolygon] {
        &mut self.shapes
    }

    pub fn len(&self) -> usize {
        self.shapes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.shapes.is_empty()
    }
}

/// Ignores every hazard of one placement (all of its parts).
pub fn self_filter(pk: PItemKey) -> PlacedItemFilter {
    PlacedItemFilter(pk)
}

/// Whether `item` placed with `t` collides, checking the cheap surrogate of every
/// part on `surrogate_cde` first and then the exact contour on `exact_cde`.
pub fn item_collides(
    surrogate_cde: &CDEngine,
    exact_cde: &CDEngine,
    item: &Item,
    buffers: &mut PartBuffers,
    t: &Transformation,
    filter: &impl HazardFilter<BasicHazardEntity>,
) -> bool {
    for part in item.parts_cd() {
        if surrogate_cde.detect_surrogate_collision(part.surrogate(), t, filter) {
            return true;
        }
    }
    buffers
        .transform(item, t)
        .iter()
        .any(|shape| exact_cde.detect_poly_collision(shape, filter))
}

/// Whether any part of a placed item collides with a relevant hazard.
pub fn placed_collides(
    cde: &CDEngine,
    pi: &PlacedItem,
    filter: &impl HazardFilter<BasicHazardEntity>,
) -> bool {
    pi.shapes()
        .iter()
        .any(|shape| cde.detect_poly_collision(shape, filter))
}

/// Whether two placements collide under `predicate`, over every pair of parts.
pub fn placed_pair_collide(
    a: &PlacedItem,
    b: &PlacedItem,
    mut predicate: impl FnMut(&SPolygon, &SPolygon) -> bool,
) -> bool {
    a.shapes()
        .iter()
        .any(|sa| b.shapes().iter().any(|sb| predicate(sa, sb)))
}

/// The hazard keys of every part of a placed item.
pub fn haz_keys(layout: &Layout, pk: PItemKey) -> Vec<jagua_rs::collision_detection::hazards::HazKey> {
    layout.haz_keys(pk)
}

/// Square root of the convex-hull area of an item's envelope: the shape factor
/// the collision tracker uses for penalties, identical for every part.
pub fn hull_root(pi: &PlacedItem) -> f32 {
    pi.shape().surrogate().convex_hull_area.sqrt()
}
