//! Place groups as multipart items.
//!
//! In [`PlaceGroupMode::MultiPolygon`](crate::config::PlaceGroupMode) every V2
//! place group becomes **one** item whose parts are the members' original
//! outlines at their hint poses. The engine then nests a compact instance in
//! which group members no longer exist as separate items. Everything inside the
//! engine (lanes, certification, diagnostics) works on compact ids; this module
//! expands results back to the public contract (member ids, member poses) at
//! the public boundary.
//!
//! Member pose: the group item's original frame is the world frame of the hints,
//! so a public group placement `T` turns member `m` (original outline, hint pose
//! `H_m` into that frame) into `T ∘ H_m`.
use crate::grouped_input::{PlacedHintInput, ValidatedInput};
use jagua_rs::geometry::{DTransformation, Transformation};
use jagua_rs::io::ext_repr::{ExtOrientation, ExtPlacedItem, ExtPolygon, ExtRotation, ExtSPolygon, ExtShape};
use jagua_rs::probs::spp::io::ext_repr::{ExtItem, ExtSPInstance, ExtSPSolution};
use std::collections::HashMap;

#[derive(Debug, Clone)]
struct GroupMember {
    item_id: u64,
    /// External transform of the member's original outline into the group frame.
    pose: DTransformation,
}

#[derive(Debug, Clone)]
enum CompactItem {
    Single(u64),
    Group { members: Vec<GroupMember> },
}

/// Maps compact (engine) item ids back to the public ids of a V2 input.
#[derive(Debug, Clone)]
pub struct GroupExpansion {
    items: Vec<CompactItem>,
}

impl GroupExpansion {
    pub fn n_compact_items(&self) -> usize {
        self.items.len()
    }

    /// Public ids behind one compact id (one for an ordinary item).
    pub fn public_ids(&self, compact_id: usize) -> Vec<u64> {
        match self.items.get(compact_id) {
            Some(CompactItem::Single(id)) => vec![*id],
            Some(CompactItem::Group { members, .. }) => members.iter().map(|m| m.item_id).collect(),
            None => vec![compact_id as u64],
        }
    }

    pub fn is_group(&self, compact_id: usize) -> bool {
        matches!(self.items.get(compact_id), Some(CompactItem::Group { .. }))
    }

    /// Replaces every group placement by its members' placements.
    pub fn expand_placements(&self, placed: Vec<ExtPlacedItem>) -> Vec<ExtPlacedItem> {
        let mut out = Vec::with_capacity(placed.len());
        for placement in placed {
            match self.items.get(placement.item_id as usize) {
                Some(CompactItem::Single(id)) => out.push(ExtPlacedItem {
                    item_id: *id,
                    transformation: placement.transformation,
                }),
                Some(CompactItem::Group { members, .. }) => {
                    let group: DTransformation = placement.transformation.clone().into();
                    for member in members {
                        let pose = Transformation::empty()
                            .transform_from_decomposed(&member.pose)
                            .transform_from_decomposed(&group)
                            .decompose();
                        out.push(ExtPlacedItem {
                            item_id: member.item_id,
                            transformation: pose.into(),
                        });
                    }
                }
                None => out.push(placement),
            }
        }
        out.sort_by_key(|p| p.item_id);
        out
    }

    pub fn expand_solution(&self, mut ext: ExtSPSolution) -> ExtSPSolution {
        ext.layout.placed_items = self.expand_placements(std::mem::take(&mut ext.layout.placed_items));
        ext
    }

    /// Expands per-item flags (collision status) to the public ids.
    pub fn expand_flags(&self, flags: Vec<(usize, bool)>) -> Vec<(usize, bool)> {
        let mut out = Vec::with_capacity(flags.len());
        for (compact_id, flag) in flags {
            for id in self.public_ids(compact_id) {
                out.push((id as usize, flag));
            }
        }
        out.sort_by_key(|(id, _)| *id);
        out
    }
}

/// Original outline of an external shape as a point list (holes rejected).
fn ext_shape_points(shape: &ExtShape, item_id: u64) -> Result<Vec<(f32, f32)>, String> {
    match shape {
        ExtShape::Rectangle { x_min, y_min, width, height } => Ok(vec![
            (*x_min, *y_min),
            (x_min + width, *y_min),
            (x_min + width, y_min + height),
            (*x_min, y_min + height),
        ]),
        ExtShape::SimplePolygon(ExtSPolygon(points)) => Ok(points.clone()),
        ExtShape::Polygon(polygon) if polygon.inner.is_empty() => Ok(polygon.outer.0.clone()),
        ExtShape::Polygon(_) => Err(format!("place group member {item_id} has holes")),
        ExtShape::MultiPolygon(_) => Err(format!(
            "place group member {item_id} is itself a multipolygon"
        )),
    }
}

/// External transform of a hint: rotate the original outline by the hint angle
/// and put the rotated raw bounding-box corner at `(tr_x, tr_y)`.
fn hint_pose(points: &[(f32, f32)], hint: &PlacedHintInput) -> DTransformation {
    let (x_min, y_min) = points.iter().fold((f32::INFINITY, f32::INFINITY), |(x, y), p| {
        (x.min(p.0), y.min(p.1))
    });
    let (sin, cos) = (hint.rot_angle_in_rad as f64).sin_cos();
    let tx = hint.tr_x as f64 - (cos * x_min as f64 - sin * y_min as f64);
    let ty = hint.tr_y as f64 - (sin * x_min as f64 + cos * y_min as f64);
    DTransformation::new(hint.rot_angle_in_rad, (tx as f32, ty as f32))
}

fn transform_points(points: &[(f32, f32)], pose: &DTransformation) -> Vec<(f32, f32)> {
    let (sin, cos) = (pose.rotation() as f64).sin_cos();
    let (tx, ty) = pose.translation();
    points
        .iter()
        .map(|&(x, y)| {
            let (x, y) = (x as f64, y as f64);
            ((cos * x - sin * y + tx as f64) as f32, (sin * x + cos * y + ty as f64) as f32)
        })
        .collect()
}

fn discrete_degrees(orientation: &ExtOrientation) -> Option<Vec<f32>> {
    match &orientation.rotation {
        ExtRotation::Continuous {} => None,
        ExtRotation::Discrete { angles } if angles.is_empty() => Some(vec![0.0]),
        ExtRotation::Discrete { angles } => Some(angles.clone()),
        ExtRotation::Stepped { step } => {
            let count = (360.0 / step).round().max(1.0) as usize;
            Some((0..count).map(|i| i as f32 * 360.0 / count as f32).collect())
        }
    }
}

/// Rewrites a validated V2 input so that every place group is one multipart
/// item. Must run after the anchor's orientation restriction
/// (`restrict_group_orientations`): the group's rotations are the anchor's
/// candidates relative to its hint angle.
pub fn compact_input(input: ValidatedInput) -> Result<(ValidatedInput, GroupExpansion), String> {
    let ValidatedInput {
        nesting,
        instance,
        regions,
        place_groups,
        hints,
        item_to_region: _,
    } = input;
    let mut member_of: HashMap<u64, usize> = HashMap::new();
    for (index, group) in place_groups.iter().enumerate() {
        for &id in &group.member_item_ids {
            if member_of.insert(id, index).is_some() {
                return Err(format!("item {id} belongs to two place groups"));
            }
        }
    }
    let by_id: HashMap<u64, &ExtItem> = instance.items.iter().map(|item| (item.base.id, item)).collect();

    let mut items: Vec<ExtItem> = Vec::new();
    let mut compact: Vec<CompactItem> = Vec::new();
    let mut new_hints: HashMap<u64, PlacedHintInput> = HashMap::new();
    let mut public_to_compact: HashMap<u64, u64> = HashMap::new();

    // Ordinary items keep their order and get dense ids.
    for item in &instance.items {
        if member_of.contains_key(&item.base.id) {
            continue;
        }
        let new_id = items.len() as u64;
        let mut base = item.base.clone();
        base.id = new_id;
        items.push(ExtItem { base, demand: item.demand });
        if let Some(hint) = hints.get(&item.base.id) {
            let mut hint = *hint;
            hint.item_id = new_id as i64;
            new_hints.insert(new_id, hint);
        }
        public_to_compact.insert(item.base.id, new_id);
        compact.push(CompactItem::Single(item.base.id));
    }

    // Each group becomes one multipart item in the hints' world frame.
    for group in &place_groups {
        let new_id = items.len() as u64;
        let anchor_id = group.member_item_ids[0];
        let anchor_hint = hints
            .get(&anchor_id)
            .ok_or_else(|| format!("place group {} anchor {anchor_id} has no hint", group.id))?;
        let anchor_initial_deg = anchor_hint.rot_angle_in_rad.to_degrees().rem_euclid(360.0);
        let mut parts = Vec::with_capacity(group.member_item_ids.len());
        let mut members = Vec::with_capacity(group.member_item_ids.len());
        let mut all_frozen = true;
        let (mut min_x, mut min_y) = (f32::INFINITY, f32::INFINITY);
        for &member_id in &group.member_item_ids {
            let item = by_id
                .get(&member_id)
                .ok_or_else(|| format!("place group {} references missing item {member_id}", group.id))?;
            let hint = hints
                .get(&member_id)
                .ok_or_else(|| format!("place group {} member {member_id} has no hint", group.id))?;
            all_frozen &= hint.is_freeze != 0;
            let points = ext_shape_points(&item.base.shape, member_id)?;
            let pose = hint_pose(&points, hint);
            let world = transform_points(&points, &pose);
            for &(x, y) in &world {
                min_x = min_x.min(x);
                min_y = min_y.min(y);
            }
            parts.push(ExtPolygon {
                outer: ExtSPolygon(world),
                inner: vec![],
            });
            members.push(GroupMember { item_id: member_id, pose });
        }
        let anchor = by_id[&anchor_id];
        let rotation = match discrete_degrees(&anchor.base.orientation) {
            None => ExtRotation::Continuous {},
            Some(candidates) => {
                let mut angles: Vec<f32> = Vec::new();
                for candidate in candidates {
                    let relative = (candidate - anchor_initial_deg).rem_euclid(360.0);
                    let relative = if relative > 359.999 { 0.0 } else { relative };
                    if !angles.iter().any(|a| (a - relative).abs() <= 1.0e-3) {
                        angles.push(relative);
                    }
                }
                ExtRotation::Discrete { angles }
            }
        };
        // Preflight, as the rigid path does: some allowed group rotation must fit the
        // strip height. Continuous rotation is not rejected from one hint pose.
        if let ExtRotation::Discrete { angles } = &rotation {
            let world: Vec<(f32, f32)> = parts.iter().flat_map(|p| p.outer.0.iter().copied()).collect();
            let fits = angles.iter().any(|deg| {
                let (sin, cos) = (deg.to_radians() as f64).sin_cos();
                let (lo, hi) = world.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &(x, y)| {
                    let ry = sin * x as f64 + cos * y as f64;
                    (lo.min(ry), hi.max(ry))
                });
                hi - lo <= instance.strip_height as f64 + 1.0e-5
            });
            if !fits {
                return Err(format!(
                    "place group {} has no allowed orientation whose vertical span fits strip height {:.3}",
                    group.id, instance.strip_height
                ));
            }
        }
        items.push(ExtItem {
            base: jagua_rs::io::ext_repr::ExtItem {
                id: new_id,
                orientation: ExtOrientation {
                    rotation,
                    reflection_axes: vec![],
                },
                shape: ExtShape::MultiPolygon(parts),
                min_quality: None,
            },
            demand: 1,
        });
        new_hints.insert(
            new_id,
            PlacedHintInput {
                ver: anchor_hint.ver,
                item_id: new_id as i64,
                is_freeze: i32::from(all_frozen),
                tr_x: min_x,
                tr_y: min_y,
                rot_angle_in_rad: 0.0,
                stripe_interval_y: 0.0,
                stripe_interval_x: 0.0,
            },
        );
        for &member_id in &group.member_item_ids {
            public_to_compact.insert(member_id, new_id);
        }
        log::debug!(
            "[GROUPS] place group {} -> compact multipart item {new_id} with {} parts",
            group.id,
            members.len()
        );
        compact.push(CompactItem::Group { members });
    }

    // Regions refer to public ids; a group collapses to its item once.
    let mut new_regions = regions;
    for region in &mut new_regions {
        let mut mapped: Vec<u64> = Vec::with_capacity(region.item_ids.len());
        for id in &region.item_ids {
            let compact_id = public_to_compact[id];
            if !mapped.contains(&compact_id) {
                mapped.push(compact_id);
            }
        }
        region.item_ids = mapped;
    }
    let item_to_region: HashMap<u64, usize> = match nesting.mode {
        crate::grouped_input::NestingMode::AllItems => (0..items.len() as u64).map(|id| (id, 0)).collect(),
        crate::grouped_input::NestingMode::SizeRegions => new_regions
            .iter()
            .enumerate()
            .flat_map(|(index, region)| region.item_ids.iter().map(move |&id| (id, index)))
            .collect(),
    };

    let compact_instance = ExtSPInstance {
        name: instance.name.clone(),
        min_item_separation: instance.min_item_separation,
        items,
        strip_height: instance.strip_height,
    };
    Ok((
        ValidatedInput {
            nesting,
            instance: compact_instance,
            regions: new_regions,
            place_groups: Vec::new(),
            hints: new_hints,
            item_to_region,
        },
        GroupExpansion { items: compact },
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grouped_input::{NestingMode, NestingOptions, PlaceGroupInput};

    fn rect(id: u64, w: f32, h: f32) -> ExtItem {
        ExtItem {
            base: jagua_rs::io::ext_repr::ExtItem {
                id,
                orientation: ExtOrientation {
                    rotation: ExtRotation::Discrete { angles: vec![0.0, 90.0] },
                    reflection_axes: vec![],
                },
                shape: ExtShape::Rectangle { x_min: 0.0, y_min: 0.0, width: w, height: h },
                min_quality: None,
            },
            demand: 1,
        }
    }

    fn hint(id: u64, x: f32, y: f32, rot: f32, frozen: bool) -> PlacedHintInput {
        PlacedHintInput {
            ver: 0,
            item_id: id as i64,
            is_freeze: i32::from(frozen),
            tr_x: x,
            tr_y: y,
            rot_angle_in_rad: rot,
            stripe_interval_y: 0.0,
            stripe_interval_x: 0.0,
        }
    }

    fn input() -> ValidatedInput {
        let mut hints = HashMap::new();
        hints.insert(1, hint(1, 10.0, 5.0, 0.0, false));
        hints.insert(2, hint(2, 30.0, 5.0, std::f32::consts::FRAC_PI_2, false));
        ValidatedInput {
            nesting: NestingOptions { mode: NestingMode::AllItems, group_workers: 1, quantum_ms: 1000, locked_prefix_gap_mm: 0.0 },
            instance: ExtSPInstance {
                name: "g".into(),
                min_item_separation: 0.0,
                items: vec![rect(0, 4.0, 4.0), rect(1, 10.0, 5.0), rect(2, 6.0, 2.0)],
                strip_height: 100.0,
            },
            regions: vec![],
            place_groups: vec![PlaceGroupInput { id: 7, member_item_ids: vec![1, 2] }],
            hints,
            item_to_region: HashMap::new(),
        }
    }

    #[test]
    fn group_becomes_one_multipart_item_in_the_hint_frame() {
        let (compact, expansion) = compact_input(input()).unwrap();
        assert_eq!(compact.instance.items.len(), 2);
        assert!(compact.place_groups.is_empty());
        assert_eq!(compact.instance.items[0].base.id, 0);
        let group = &compact.instance.items[1].base;
        assert_eq!(group.id, 1);
        let ExtShape::MultiPolygon(parts) = &group.shape else { panic!("{}", swarm_core::dmsg!("multipolygon")) };
        assert_eq!(parts.len(), 2);
        // member 1 at (10,5) unrotated: x in [10,20], y in [5,10]
        assert!(parts[0].outer.0.iter().all(|p| p.0 >= 10.0 - 1e-4 && p.0 <= 20.0 + 1e-4));
        // member 2 rotated 90 degrees: the hint puts the rotated origin corner of
        // its 6x2 box at (30,5), so it spans x in [28,30], y in [5,11]
        assert!(parts[1].outer.0.iter().all(|p| p.0 >= 28.0 - 1e-3 && p.0 <= 30.0 + 1e-3));
        assert!(parts[1].outer.0.iter().all(|p| p.1 >= 5.0 - 1e-3 && p.1 <= 11.0 + 1e-3));
        let hint = compact.hints[&1];
        assert_eq!((hint.tr_x, hint.tr_y, hint.rot_angle_in_rad), (10.0, 5.0, 0.0));
        assert_eq!(hint.is_freeze, 0);
        assert_eq!(expansion.public_ids(1), vec![1, 2]);
        assert_eq!(expansion.public_ids(0), vec![0]);
        // anchor candidates [0, 90] relative to its hint angle 0 stay [0, 90]
        assert_eq!(group.orientation.rotation, ExtRotation::Discrete { angles: vec![0.0, 90.0] });
    }

    #[test]
    fn expanding_a_group_placement_composes_member_poses() {
        let (_, expansion) = compact_input(input()).unwrap();
        // Group moved by (+100, +7) and rotated 90 degrees.
        let placed = vec![
            ExtPlacedItem { item_id: 0, transformation: DTransformation::new(0.0, (1.0, 2.0)).into() },
            ExtPlacedItem { item_id: 1, transformation: DTransformation::new(std::f32::consts::FRAC_PI_2, (100.0, 7.0)).into() },
        ];
        let out = expansion.expand_placements(placed);
        assert_eq!(out.iter().map(|p| p.item_id).collect::<Vec<_>>(), vec![0, 1, 2]);
        // member 1: pose (0, (10,5)) then group (90deg, (100,7)): rotation 90, translation R(90)*(10,5) + (100,7) = (-5,10)+(100,7) = (95,17)
        let m1 = &out[1].transformation;
        assert!((m1.rotation - 90.0).abs() < 1e-3, "{}", m1.rotation);
        assert!((m1.translation.0 - 95.0).abs() < 1e-3 && (m1.translation.1 - 17.0).abs() < 1e-3, "{:?}", m1.translation);
        // member 2: its own 90 plus the group's 90 = 180
        let m2 = &out[2].transformation;
        assert!((m2.rotation.rem_euclid(360.0) - 180.0).abs() < 1e-3, "{}", m2.rotation);
        assert_eq!(expansion.expand_flags(vec![(1, true), (0, false)]), vec![(0, false), (1, true), (2, true)]);
    }
}
