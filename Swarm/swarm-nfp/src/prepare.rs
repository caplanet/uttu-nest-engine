// Implemented separately from the optimizer so normalization, topology and
// triangulation can be tested without any Swarm placement state.

use crate::{Bounds, NfpResult, Orientation, PolygonSet};

use crate::backend::{
    GridPath, GridPaths, prepare_geometry_from_paths, prepare_geometry_from_polygon_set,
};

#[derive(Debug, Clone)]
pub struct PreparedShape {
    pub(crate) geometry: PolygonSet,
    pub(crate) canonical_paths: GridPaths,
    pub(crate) triangles: Vec<GridPath>,
    pub(crate) bounds: Bounds,
    pub(crate) area_mm2: f64,
    #[allow(dead_code)]
    pub(crate) area2: i128,
    #[allow(dead_code)]
    pub(crate) max_abs: i64,
}

impl PreparedShape {
    pub fn geometry(&self) -> &PolygonSet {
        &self.geometry
    }

    pub fn bounds(&self) -> Bounds {
        self.bounds
    }

    pub fn area_mm2(&self) -> f64 {
        self.area_mm2
    }

    /// Number of canonical material triangles cached for Minkowski products.
    ///
    /// A pair NFP creates at most `fixed.triangle_count() *
    /// moving.triangle_count()` convex atoms before their Boolean union, so
    /// this is also a useful cost estimate for callers.
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Number of vertices after grid quantization, union and canonicalization.
    pub fn vertex_count(&self) -> usize {
        self.geometry
            .components
            .iter()
            .map(|component| {
                component.outer.vertices.len()
                    + component
                        .holes
                        .iter()
                        .map(|hole| hole.vertices.len())
                        .sum::<usize>()
            })
            .sum()
    }

    pub(crate) fn from_polygon_set(input: &PolygonSet, context: &'static str) -> NfpResult<Self> {
        let prepared = prepare_geometry_from_polygon_set(input, context)?;
        Ok(Self::from_backend(prepared))
    }

    pub(crate) fn from_paths(paths: GridPaths, context: &'static str) -> NfpResult<Self> {
        let prepared = prepare_geometry_from_paths(paths, context)?;
        Ok(Self::from_backend(prepared))
    }

    fn from_backend(prepared: crate::backend::PreparedGeometry) -> Self {
        Self {
            geometry: prepared.geometry,
            canonical_paths: prepared.canonical_paths,
            triangles: prepared.triangles,
            bounds: prepared.bounds,
            area_mm2: prepared.area2 as f64
                / (2.0 * crate::GRID_SCALE as f64 * crate::GRID_SCALE as f64),
            area2: prepared.area2,
            max_abs: prepared.max_abs,
        }
    }

    pub fn oriented(&self, orientation: Orientation) -> NfpResult<Self> {
        crate::engine::NfpEngine.orient(self, orientation)
    }
}

pub(crate) fn prepare_polygon_set(
    input: &PolygonSet,
    context: &'static str,
) -> NfpResult<PreparedShape> {
    PreparedShape::from_polygon_set(input, context)
}

pub(crate) fn prepare_from_paths(
    paths: GridPaths,
    context: &'static str,
) -> NfpResult<PreparedShape> {
    PreparedShape::from_paths(paths, context)
}
