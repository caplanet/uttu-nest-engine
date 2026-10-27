use crate::backend::{
    GridPaths, canonical_paths_cmp, locate_primitive_union, primitive_paths_to_polygon_set,
    quantize_mm, reflect_convex_primitives,
};
use crate::{Location, NfpResult, Point, PolygonSet};

/// Complete pair NFP represented as an exact union of convex grid primitives.
///
/// Boolean contour construction has to round rational edge intersections back
/// to an integer grid. The primitive union avoids that second rounding step:
/// without clearance, every primitive is an exact convex Minkowski difference
/// of one fixed and one moving material triangle. A clearance result further
/// sums those primitives with clearance-kernel triangles. Holes and
/// disconnected regions are implicit in the union and its gaps.
#[derive(Debug, Clone)]
pub struct CompleteNfp {
    compact_contour: PolygonSet,
    primitives: PolygonSet,
    pub(crate) primitive_paths: GridPaths,
}

impl CompleteNfp {
    pub(crate) fn new(compact_contour: PolygonSet, primitive_paths: GridPaths) -> Self {
        let primitives = primitive_paths_to_polygon_set(&primitive_paths);
        Self {
            compact_contour,
            primitives,
            primitive_paths,
        }
    }

    /// Compact, canonical shell/hole contour for visualization and broad-phase
    /// queries. Clipper rounds derived intersections to the 0.001 mm grid, so
    /// use [`Self::locate`] for authoritative membership/contact semantics.
    pub fn compact_contour(&self) -> &PolygonSet {
        &self.compact_contour
    }

    /// Exact convex configuration-obstacle primitives. Components
    /// intentionally overlap; their set union is the complete NFP.
    ///
    /// This is an inspection/diagnostic view, not canonical shell/hole
    /// topology or a persistence contract.
    /// Passing it through a regular polygon union intentionally removes
    /// contact-only internal seams. Use [`Self::locate`] for exact membership.
    pub fn primitives(&self) -> &PolygonSet {
        &self.primitives
    }

    pub fn primitive_count(&self) -> usize {
        self.primitive_paths.len()
    }

    /// Return whether `self` is exactly the point reflection of `other` at the
    /// primitive-multiset level.
    ///
    /// Unlike comparing compact contours, this does not Boolean-union the
    /// primitives or round newly derived intersections. It is intended for
    /// deterministic diagnostics such as the WPF NFP laboratory's
    /// `NFP(F,M) = -NFP(M,F)` verification.
    pub fn is_point_reflection_of(&self, other: &Self) -> NfpResult<bool> {
        let reflected = reflect_convex_primitives(&other.primitive_paths)?;
        Ok(canonical_paths_cmp(&self.primitive_paths, &reflected) == std::cmp::Ordering::Equal)
    }

    /// Locate a translation in the exact primitive union on the input grid.
    ///
    /// Without clearance, `Inside` means at least one material-triangle pair
    /// has positive-area overlap and `Boundary` means contact only. With a
    /// caller-supplied convex kernel, these mean kernel-defined clearance
    /// violation and limit. A conservative Euclidean kernel intentionally may
    /// reject an additional band no wider than its configured excess. This
    /// also retains contact-only seams hidden inside the compact contour.
    pub fn locate(&self, translation: Point) -> NfpResult<Location> {
        let x = quantize_mm(translation.x, "complete NFP location x")?;
        let y = quantize_mm(translation.y, "complete NFP location y")?;
        locate_primitive_union(&self.primitive_paths, x, y)
    }

    pub fn into_compact_contour(self) -> PolygonSet {
        self.compact_contour
    }
}
