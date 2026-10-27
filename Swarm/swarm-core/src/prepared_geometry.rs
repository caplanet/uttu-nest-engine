//! Immutable, run-owned geometry preparation. No global pointer-keyed cache.
use jagua_rs::entities::Item;
use jagua_rs::geometry::Transformation;
use jagua_rs::geometry::geo_enums::RotationRange;
use jagua_rs::geometry::geo_traits::TransformableFrom;
use jagua_rs::geometry::primitives::{Rect, SPolygon};
use jagua_rs::probs::spp::entities::SPInstance;
use std::sync::Arc;

/// Invariant under rigid transforms, with a bit-exact fallback for replacement
/// geometry. Keep the pair expression `(root1 * root2).sqrt()` at the caller.
#[derive(Clone, Copy, Debug)]
pub struct ShapeFactor {
    area_bits: u32,
    root: f32,
}
impl ShapeFactor {
    pub fn new(shape: &SPolygon) -> Self {
        let area = shape.surrogate().convex_hull_area;
        Self {
            area_bits: area.to_bits(),
            root: area.sqrt(),
        }
    }
    #[inline(always)]
    pub fn root_for(self, shape: &SPolygon) -> f32 {
        let area = shape.surrogate().convex_hull_area;
        if area.to_bits() == self.area_bits {
            self.root
        } else {
            area.sqrt()
        }
    }
    pub(crate) fn refresh(&mut self, shape: &SPolygon) {
        if self.area_bits != shape.surrogate().convex_hull_area.to_bits() {
            *self = Self::new(shape);
        }
    }
}

#[derive(Debug)]
pub struct PreparedItemGeometry {
    // Owning the Arc prevents allocator address reuse from reviving stale data.
    shape: Arc<SPolygon>,
    rotations: RotationRange,
    pub(crate) rotation_bboxes: Vec<(f32, Rect)>,
    pub shape_factor: ShapeFactor,
}
impl PreparedItemGeometry {
    pub fn new(item: &Item) -> Self {
        let rotations: Vec<f32> = match item.allowed_orientations().rotations() {
            RotationRange::None => vec![0.0],
            RotationRange::Discrete(values) => values.clone(),
            RotationRange::Continuous => {
                // Preserve the original f32 operation and proposal order.
                let step = (2.0 * std::f32::consts::PI) / 16.0;
                (0..16).map(|i| i as f32 * step).collect()
            }
        };
        let mut buffer = item.shape_cd().as_ref().clone();
        let rotation_bboxes = rotations
            .into_iter()
            .map(|r| {
                let bbox = buffer
                    .transform_from(item.shape_cd().as_ref(), &Transformation::from_rotation(r))
                    .bbox();
                (r, bbox)
            })
            .collect();
        Self {
            shape: item.shape_cd().clone(),
            rotations: item.allowed_orientations().rotations().clone(),
            rotation_bboxes,
            shape_factor: ShapeFactor::new(&item.shape_cd()),
        }
    }
    pub fn matches(&self, item: &Item) -> bool {
        Arc::ptr_eq(&self.shape, &item.shape_cd())
            && match (&self.rotations, &item.allowed_orientations().rotations()) {
                (RotationRange::None, RotationRange::None)
                | (RotationRange::Continuous, RotationRange::Continuous) => true,
                (RotationRange::Discrete(a), RotationRange::Discrete(b)) => {
                    a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.to_bits() == y.to_bits())
                }
                _ => false,
            }
    }
}

#[derive(Debug)]
pub struct PreparedGeometry {
    items: Vec<PreparedItemGeometry>,
}
impl PreparedGeometry {
    pub fn new(instance: &SPInstance) -> Self {
        Self {
            items: instance
                .items
                .iter()
                .map(|(item, _)| PreparedItemGeometry::new(item))
                .collect(),
        }
    }
    pub fn get(&self, item: &Item) -> Option<&PreparedItemGeometry> {
        self.items.get(item.idx()).filter(|entry| entry.matches(item))
    }
    pub(crate) fn shape_factor(&self, item_id: usize, shape: &SPolygon) -> ShapeFactor {
        match self.items.get(item_id) {
            Some(entry) => {
                let mut factor = entry.shape_factor;
                factor.refresh(shape);
                factor
            }
            None => ShapeFactor::new(shape),
        }
    }
}
