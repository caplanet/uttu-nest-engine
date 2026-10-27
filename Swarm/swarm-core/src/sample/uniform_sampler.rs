use itertools::Itertools;
use jagua_rs::entities::Item;
use jagua_rs::geometry::geo_enums::RotationRange;
use jagua_rs::geometry::primitives::Rect;
use jagua_rs::geometry::{DTransformation, normalize_rotation};
use ordered_float::OrderedFloat;
use rand::prelude::IndexedRandom;
use rand::{Rng, RngExt};
use std::ops::Range;

use crate::prepared_geometry::PreparedItemGeometry;

/// A sampler that creates uniform samples for an item within a bounding box
#[derive(Clone, Debug)]
pub struct UniformBBoxSampler {
    /// The list of possible rotations and their corresponding x and y ranges
    rot_entries: Vec<RotEntry>,
}

#[derive(Clone, Debug)]
struct RotEntry {
    pub r: f32,
    pub x_range: Range<f32>,
    pub y_range: Range<f32>,
}

impl UniformBBoxSampler {
    pub fn new(sample_bbox: Rect, item: &Item, container_bbox: Rect) -> Option<Self> {
        let prepared = PreparedItemGeometry::new(item);
        Self::from_prepared(sample_bbox, container_bbox, &prepared)
    }

    /// Validates geometry ownership and exact rotation bits before reuse.
    pub fn new_with_prepared(
        sample_bbox: Rect,
        item: &Item,
        container_bbox: Rect,
        prepared: &PreparedItemGeometry,
    ) -> Option<Self> {
        if prepared.matches(item) {
            Self::from_prepared(sample_bbox, container_bbox, prepared)
        } else {
            Self::new(sample_bbox, item, container_bbox)
        }
    }

    pub(crate) fn from_prepared(
        sample_bbox: Rect,
        container_bbox: Rect,
        prepared: &PreparedItemGeometry,
    ) -> Option<Self> {
        let sample_x_range = sample_bbox.x_min..sample_bbox.x_max;
        let sample_y_range = sample_bbox.y_min..sample_bbox.y_max;

        // for each possible rotation, calculate the sample ranges (x and y)
        // where the item resides fully inside the container and is within the sample bounding box
        let rot_entries = prepared
            .rotation_bboxes
            .iter()
            .filter_map(|&(r, r_shape_bbox)| {
                //narrow the container range to account for the rotated shape
                let cont_x_range = (container_bbox.x_min - r_shape_bbox.x_min)
                    ..(container_bbox.x_max - r_shape_bbox.x_max);
                let cont_y_range = (container_bbox.y_min - r_shape_bbox.y_min)
                    ..(container_bbox.y_max - r_shape_bbox.y_max);

                //intersect with the sample bbox
                let x_range = intersect_range(&cont_x_range, &sample_x_range);
                let y_range = intersect_range(&cont_y_range, &sample_y_range);

                //make sure the ranges are not empty
                if !valid_closed_range(&x_range) || !valid_closed_range(&y_range) {
                    None
                } else {
                    Some(RotEntry {
                        r,
                        x_range,
                        y_range,
                    })
                }
            })
            .collect_vec();

        match rot_entries.is_empty() {
            true => None,
            false => Some(Self { rot_entries }),
        }
    }

    pub fn sample(&self, rng: &mut impl Rng) -> DTransformation {
        // randomly select a rotation
        let r_entry = self.rot_entries.choose(rng).unwrap();

        // sample a random x and y value within the valid range
        let r = r_entry.r;
        let x_sample = sample_range(&r_entry.x_range, rng);
        let y_sample = sample_range(&r_entry.y_range, rng);

        DTransformation::new(r, (x_sample, y_sample))
    }

    /// [`Self::sample`]의 좌측 편향 변형 (fork 전용, capmods 경로에서 사용).
    /// x를 균등 분포 대신 `u^bias_k` (u ~ U[0,1), `bias_k >= 1`)로 변환해 뽑아
    /// x_min(좌측) 쪽에 확률 질량을 몰아준다. `bias_k == 1`이면 균등 분포와 동일.
    pub fn sample_x_biased(&self, rng: &mut impl Rng, bias_k: f32) -> DTransformation {
        let r_entry = self.rot_entries.choose(rng).unwrap();

        let r = r_entry.r;
        let u: f32 = rng.random_range(0.0..1.0);
        let x_sample =
            r_entry.x_range.start + (r_entry.x_range.end - r_entry.x_range.start) * u.powf(bias_k);
        let y_sample = sample_range(&r_entry.y_range, rng);

        DTransformation::new(r, (x_sample, y_sample))
    }
}

// A closed feasible interval may contain exactly one coordinate. rand's
// half-open range cannot represent that case, so sample its fixed endpoint.
fn valid_closed_range(range: &Range<f32>) -> bool {
    range.start.is_finite() && range.end.is_finite() && range.start <= range.end
}

fn sample_range(range: &Range<f32>, rng: &mut impl Rng) -> f32 {
    if range.start == range.end {
        range.start
    } else {
        rng.random_range(range.clone())
    }
}

fn intersect_range(a: &Range<f32>, b: &Range<f32>) -> Range<f32> {
    let min = f32::max(a.start, b.start);
    let max = f32::min(a.end, b.end);
    min..max
}

/// Converts a sample transformation to the closest feasible transformation. (for now just mapping rotation to the closest allowed one)
pub fn convert_sample_to_closest_feasible(dt: DTransformation, item: &Item) -> DTransformation {
    let feasible_rotation = match item.allowed_orientations().rotations() {
        RotationRange::None => 0.0,
        RotationRange::Discrete(v) => {
            // find the closest rotation in the discrete set
            v.iter()
                .min_by_key(|&&r| {
                    // make sure to normalize the delta to the range [-PI, PI]
                    let norm_delta = normalize_rotation(dt.rotation() - r);
                    OrderedFloat(norm_delta.abs())
                })
                .cloned()
                .unwrap()
        }
        RotationRange::Continuous => {
            // for continuous rotation, we can just use the sample rotation
            dt.rotation()
        }
    };
    DTransformation::new(feasible_rotation, dt.translation())
}
