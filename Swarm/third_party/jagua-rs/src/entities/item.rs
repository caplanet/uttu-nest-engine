use std::sync::Arc;

use crate::geometry::convex_hull::convex_hull_from_points;
use crate::geometry::fail_fast::SPSurrogateConfig;
use crate::geometry::primitives::{Point, SPolygon};
use crate::geometry::{AllowedOrientations, OriginalShape};

use crate::entities::N_QUALITIES;
use anyhow::{Result, ensure};

/// Item to be produced.
#[derive(Clone, Debug)]
pub struct Item {
    /// Dense index in the owning instance's item list.
    pub(crate) idx: usize,
    /// Original caller-supplied identifier, preserved on export.
    pub(crate) external_id: u64,
    /// Original contour of the item as defined in the input
    pub(crate) shape_orig: Arc<OriginalShape>,
    /// Contour of the item to be used for collision detection
    pub(crate) shape_cd: Arc<SPolygon>,
    /// CAPLANET: original contour of every part. One entry that is `shape_orig` for a
    /// single polygon; several for a multipolygon, all in one shared frame.
    pub(crate) parts_orig: Vec<Arc<OriginalShape>>,
    /// CAPLANET: collision contour of every part; `shape_cd` is their convex hull
    /// (identical to the single entry for a single polygon).
    pub(crate) parts_cd: Vec<Arc<SPolygon>>,
    /// CAPLANET: area of the original parts (sum).
    pub(crate) area: f32,
    /// Allowed rotations and reflections in which the item can be placed
    pub(crate) allowed_orientations: AllowedOrientations,
    /// The minimum quality the item should be produced out of, if `None` the item requires full quality
    pub(crate) min_quality: Option<usize>,
    /// Configuration for the surrogate generation
    pub(crate) surrogate_config: SPSurrogateConfig,
}

impl Item {
    /// Returns an error for invalid geometry or a minimum quality outside `0..N_QUALITIES`.
    pub fn new(
        idx: usize,
        external_id: u64,
        original_shape: OriginalShape,
        allowed_orientations: AllowedOrientations,
        min_quality: Option<usize>,
        surrogate_config: SPSurrogateConfig,
    ) -> Result<Item> {
        ensure!(
            min_quality.is_none_or(|quality| quality < N_QUALITIES),
            "Item minimum quality must be below {N_QUALITIES}"
        );
        let shape_orig = Arc::new(original_shape);
        let shape_int = {
            let mut shape_int = shape_orig.convert_to_internal()?;
            shape_int.generate_surrogate(surrogate_config)?;
            Arc::new(shape_int)
        };
        Ok(Item {
            idx,
            external_id,
            area: shape_orig.area(),
            parts_orig: vec![shape_orig.clone()],
            parts_cd: vec![shape_int.clone()],
            shape_orig,
            shape_cd: shape_int,
            allowed_orientations,
            min_quality,
            surrogate_config,
        })
    }

    /// CAPLANET: an item made of several disjoint polygons that move as one rigid body.
    ///
    /// Every part must share the same `pre_transform`, `modify_mode` and `modify_config`
    /// (the importer centres the union once). `shape_orig()` / `shape_cd()` return the
    /// convex hull of the parts, which sampling and bounding-box code can use as the
    /// item's envelope; collision detection registers one hazard per part.
    pub fn new_multipart(
        idx: usize,
        external_id: u64,
        parts: Vec<OriginalShape>,
        allowed_orientations: AllowedOrientations,
        min_quality: Option<usize>,
        surrogate_config: SPSurrogateConfig,
    ) -> Result<Item> {
        ensure!(!parts.is_empty(), "multipart item needs at least one part");
        ensure!(
            min_quality.is_none_or(|quality| quality < N_QUALITIES),
            "Item minimum quality must be below {N_QUALITIES}"
        );
        ensure!(
            parts.iter().all(|p| p.pre_transform == parts[0].pre_transform),
            "multipart item parts must share one pre-transform"
        );
        if parts.len() == 1 {
            let part = parts.into_iter().next().expect("one part");
            return Item::new(idx, external_id, part, allowed_orientations, min_quality, surrogate_config);
        }
        let parts_orig: Vec<Arc<OriginalShape>> = parts.into_iter().map(Arc::new).collect();
        let mut parts_cd = Vec::with_capacity(parts_orig.len());
        for part in &parts_orig {
            let mut shape = part.convert_to_internal()?;
            shape.generate_surrogate(surrogate_config)?;
            parts_cd.push(Arc::new(shape));
        }
        let hull_points = |shapes: &mut dyn Iterator<Item = &[Point]>| {
            convex_hull_from_points(shapes.flat_map(|v| v.iter().copied()).collect())
        };
        let shape_cd = {
            let mut hull = SPolygon::new(hull_points(&mut parts_cd.iter().map(|p| p.vertices())))?;
            hull.generate_surrogate(surrogate_config)?;
            Arc::new(hull)
        };
        let shape_orig = {
            let template = &parts_orig[0];
            Arc::new(OriginalShape {
                shape: SPolygon::new(hull_points(&mut parts_orig.iter().map(|p| p.shape.vertices())))?,
                pre_transform: template.pre_transform,
                modify_mode: template.modify_mode,
                modify_config: template.modify_config,
            })
        };
        let area = parts_orig.iter().map(|p| p.area()).sum();
        Ok(Item {
            idx,
            external_id,
            shape_orig,
            shape_cd,
            parts_orig,
            parts_cd,
            area,
            allowed_orientations,
            min_quality,
            surrogate_config,
        })
    }

    /// CAPLANET: number of rigidly connected parts (1 for an ordinary item).
    #[must_use]
    pub fn n_parts(&self) -> usize {
        self.parts_cd.len()
    }

    /// CAPLANET: true when the item consists of several polygons.
    #[must_use]
    pub fn is_multipart(&self) -> bool {
        self.parts_cd.len() > 1
    }

    /// CAPLANET: collision contours of the parts, in the item's frame.
    #[must_use]
    pub fn parts_cd(&self) -> &[Arc<SPolygon>] {
        &self.parts_cd
    }

    /// CAPLANET: original contours of the parts.
    #[must_use]
    pub fn parts_orig(&self) -> &[Arc<OriginalShape>] {
        &self.parts_orig
    }

    /// CAPLANET: same item with other orientation permissions (validation tests).
    #[must_use]
    pub fn with_allowed_orientations(mut self, allowed_orientations: AllowedOrientations) -> Self {
        self.allowed_orientations = allowed_orientations;
        self
    }

    /// CAPLANET: same item under another external id (lane-local ids equal the index).
    #[must_use]
    pub fn with_external_id(mut self, external_id: u64) -> Self {
        self.external_id = external_id;
        self
    }

    /// CAPLANET: same item under another dense index (subset instances).
    #[must_use]
    pub fn with_idx(mut self, idx: usize) -> Self {
        self.idx = idx;
        self
    }

    /// CAPLANET: replaces the collision shape, keeping the original shape and metadata
    /// (swarm-core's millimetre-bounded simplification runs after import).
    #[must_use]
    pub fn with_shape_cd(mut self, shape_cd: Arc<SPolygon>) -> Self {
        if self.parts_cd.len() == 1 {
            self.parts_cd = vec![shape_cd.clone()];
        }
        self.shape_cd = shape_cd;
        self
    }

    /// CAPLANET: replaces the collision contours of all parts and rebuilds the hull.
    /// Panics if the number of parts changes.
    #[must_use]
    pub fn with_parts_cd(mut self, parts_cd: Vec<Arc<SPolygon>>) -> Self {
        assert_eq!(parts_cd.len(), self.parts_cd.len(), "part count must not change");
        if parts_cd.len() == 1 {
            self.shape_cd = parts_cd[0].clone();
        } else {
            let mut hull = SPolygon::new(convex_hull_from_points(
                parts_cd.iter().flat_map(|p| p.vertices().iter().copied()).collect(),
            ))
            .expect("hull of valid parts is a valid polygon");
            hull.generate_surrogate(self.surrogate_config)
                .expect("hull surrogate");
            self.shape_cd = Arc::new(hull);
        }
        self.parts_cd = parts_cd;
        self
    }

    /// CAPLANET: replaces the original shape record (for example to drop a negative
    /// offset from its modify config) without touching the collision shape.
    #[must_use]
    pub fn with_shape_orig(mut self, shape_orig: Arc<OriginalShape>) -> Self {
        if self.parts_orig.len() == 1 {
            self.parts_orig = vec![shape_orig.clone()];
        }
        self.shape_orig = shape_orig;
        self
    }

    /// CAPLANET: replaces the original contours of all parts (same count); the hull
    /// record keeps its own contour but takes the first part's modify config.
    #[must_use]
    pub fn with_parts_orig(mut self, parts_orig: Vec<Arc<OriginalShape>>) -> Self {
        assert_eq!(parts_orig.len(), self.parts_orig.len(), "part count must not change");
        if parts_orig.len() == 1 {
            self.shape_orig = parts_orig[0].clone();
        } else {
            let mut hull = self.shape_orig.as_ref().clone();
            hull.modify_mode = parts_orig[0].modify_mode;
            hull.modify_config = parts_orig[0].modify_config;
            hull.pre_transform = parts_orig[0].pre_transform;
            self.shape_orig = Arc::new(hull);
        }
        self.parts_orig = parts_orig;
        self
    }

    /// Area of the original contour(s); the sum of the parts for a multipart item.
    #[must_use]
    pub fn area(&self) -> f32 {
        self.area
    }

    /// Dense index in the owning instance's item list.
    #[must_use]
    pub fn idx(&self) -> usize {
        self.idx
    }

    /// Original caller-supplied identifier, preserved on export.
    #[must_use]
    pub fn external_id(&self) -> u64 {
        self.external_id
    }

    /// Original contour of the item as defined in the input
    #[must_use]
    pub fn shape_orig(&self) -> &Arc<OriginalShape> {
        &self.shape_orig
    }

    /// Contour of the item to be used for collision detection
    #[must_use]
    pub fn shape_cd(&self) -> &Arc<SPolygon> {
        &self.shape_cd
    }

    /// Allowed rotations and reflections in which the item can be placed
    #[must_use]
    pub fn allowed_orientations(&self) -> &AllowedOrientations {
        &self.allowed_orientations
    }

    /// The minimum quality the item should be produced out of, if `None` the item requires full quality
    #[must_use]
    pub fn min_quality(&self) -> Option<usize> {
        self.min_quality
    }

    /// Configuration for the surrogate generation
    #[must_use]
    pub fn surrogate_config(&self) -> SPSurrogateConfig {
        self.surrogate_config
    }
}
