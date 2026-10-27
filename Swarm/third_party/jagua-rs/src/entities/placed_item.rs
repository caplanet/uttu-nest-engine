use crate::collision_detection::hazards::{BasicHazardEntity, PItemKey};
use crate::entities::Item;
use crate::geometry::DTransformation;
use crate::geometry::geo_traits::Transformable;
use crate::geometry::primitives::SPolygon;
use std::borrow::Borrow;
use std::sync::Arc;

#[cfg(doc)]
use crate::entities::Layout;

/// Represents an [`Item`] that has been placed in a [`Layout`]
#[derive(Clone, Debug)]
pub struct PlacedItem {
    /// Shared immutable item definition, retained by layouts and snapshots.
    pub(crate) item: Arc<Item>,
    /// The transformation that was applied to the `Item` before it was placed
    pub(crate) d_transf: DTransformation,
    /// The shape of the `Item` after it has been transformed and placed in a `Layout`
    /// (CAPLANET: the convex hull for a multipart item)
    pub(crate) shape: Arc<SPolygon>,
    /// CAPLANET: every part after the transformation; shares `shape` for a single polygon.
    pub(crate) shapes: Vec<Arc<SPolygon>>,
}

impl PlacedItem {
    #[must_use]
    pub fn new(item: &Arc<Item>, d_transf: DTransformation) -> Self {
        let transf = d_transf.compose();
        let shape = Arc::new(item.shape_cd.transform_clone(&transf));
        let shapes = if item.is_multipart() {
            item.parts_cd
                .iter()
                .map(|part| Arc::new(part.transform_clone(&transf)))
                .collect()
        } else {
            vec![shape.clone()]
        };

        PlacedItem {
            item: Arc::clone(item),
            d_transf,
            shape,
            shapes,
        }
    }

    /// CAPLANET: every part after the transformation (one entry for an ordinary item).
    #[must_use]
    pub fn shapes(&self) -> &[Arc<SPolygon>] {
        &self.shapes
    }

    /// CAPLANET: number of parts registered as hazards for this placement.
    #[must_use]
    pub fn n_parts(&self) -> usize {
        self.shapes.len()
    }

    /// CAPLANET: the hazard entity of one part of this placement.
    #[must_use]
    pub fn part_entity(&self, pk: PItemKey, part: usize) -> BasicHazardEntity {
        BasicHazardEntity::PlacedItem {
            id: self.item.idx,
            dt: self.d_transf,
            pk,
            part,
        }
    }

    /// Shared immutable item definition, retained by layouts and snapshots.
    #[must_use]
    pub fn item(&self) -> &Arc<Item> {
        &self.item
    }

    /// The transformation that was applied to the `Item` before it was placed
    #[must_use]
    pub fn d_transf(&self) -> DTransformation {
        self.d_transf
    }

    /// The shape of the `Item` after it has been transformed and placed in a `Layout`
    #[must_use]
    pub fn shape(&self) -> &Arc<SPolygon> {
        &self.shape
    }

    /// CAPLANET: overwrites the recorded pose only; the cached shape is left as is.
    /// Used by validation tests that reproduce a pose/shape mismatch.
    #[doc(hidden)]
    pub fn set_d_transf(&mut self, d_transf: DTransformation) {
        self.d_transf = d_transf;
    }

    /// CAPLANET: swaps the item definition only; the cached shape is left as is.
    /// Used by validation tests that fabricate duplicate or unknown items.
    #[doc(hidden)]
    pub fn set_item(&mut self, item: Arc<Item>) {
        self.item = item;
    }
}

impl<T> From<(PItemKey, T)> for BasicHazardEntity
where
    T: Borrow<PlacedItem>,
{
    fn from((pk, pi): (PItemKey, T)) -> Self {
        // CAPLANET: part 0; multipart placements have one entity per part.
        BasicHazardEntity::PlacedItem {
            id: pi.borrow().item.idx,
            dt: pi.borrow().d_transf,
            pk,
            part: 0,
        }
    }
}
