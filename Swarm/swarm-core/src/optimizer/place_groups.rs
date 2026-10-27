//! Rigid placement-unit metadata shared by the Swarm optimizers.
//!
//! Jagua currently stores one polygon per placed item.  A `PlaceGroup` therefore
//! remains a set of ordinary Jagua items; this module only defines which items
//! must receive the same world-space rigid delta.

use jagua_rs::entities::{Layout, PItemKey};
use jagua_rs::geometry::DTransformation;
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use std::error::Error;
use std::fmt::{Display, Formatter};

/// External identifier of a rigid place group.
pub type PlaceGroupId = u64;

/// One member of a rigid place group.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PlaceGroupMember {
    pub item_id: usize,
    /// Initial internal Jagua transform.  It defines the relative pose used by
    /// the fresh-LBF path.  Runtime moves derive the delta from the current
    /// layout so rollback/warm starts remain exact.
    pub initial_transform: DTransformation,
}

/// Input definition for one rigid place group.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PlaceGroupSpec {
    pub id: PlaceGroupId,
    pub members: Vec<PlaceGroupMember>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct PlaceGroup {
    id: PlaceGroupId,
    members: Vec<PlaceGroupMember>,
}

/// Validated, immutable lookup table used by LBF, Separator and its workers.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PlaceGroups {
    groups: Vec<PlaceGroup>,
    item_to_group: Vec<Option<usize>>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PlaceGroupError {
    TooFewMembers {
        group_id: PlaceGroupId,
    },
    UnknownItem {
        group_id: PlaceGroupId,
        item_id: usize,
    },
    RepeatedItem {
        item_id: usize,
    },
    UnsupportedDemand {
        item_id: usize,
        demand: usize,
    },
    MissingPlacement {
        group_id: PlaceGroupId,
        item_id: usize,
    },
    RepeatedPlacement {
        group_id: PlaceGroupId,
        item_id: usize,
    },
    PartiallyPlaced {
        group_id: PlaceGroupId,
    },
}

impl Display for PlaceGroupError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::TooFewMembers { group_id } => {
                write!(f, "place group {group_id} must contain at least two items")
            }
            Self::UnknownItem { group_id, item_id } => write!(
                f,
                "place group {group_id} references unknown item {item_id}"
            ),
            Self::RepeatedItem { item_id } => {
                write!(f, "item {item_id} occurs in more than one place group")
            }
            Self::UnsupportedDemand { item_id, demand } => {
                write!(
                    f,
                    "place-group mode item {item_id} must have demand 1, got {demand}"
                )
            }
            Self::MissingPlacement { group_id, item_id } => {
                write!(f, "place group {group_id} member {item_id} is not placed")
            }
            Self::RepeatedPlacement { group_id, item_id } => write!(
                f,
                "place group {group_id} member {item_id} is placed more than once"
            ),
            Self::PartiallyPlaced { group_id } => {
                write!(f, "place group {group_id} is only partially placed")
            }
        }
    }
}

impl Error for PlaceGroupError {}

impl PlaceGroups {
    /// Validates group membership against an imported Swarm instance.
    ///
    /// Grouped item kinds deliberately require `demand == 1`: Jagua placements
    /// are keyed independently, while a group definition is keyed by item id.
    /// Callers that start with repeated item kinds must expand them to unique
    /// demand-one items before constructing this table.
    pub fn new(instance: &SPInstance, specs: Vec<PlaceGroupSpec>) -> Result<Self, PlaceGroupError> {
        let mut item_to_group = vec![None; instance.items.len()];
        let mut groups = Vec::with_capacity(specs.len());

        // Placement units are addressed by item id. In grouped mode every
        // placement, including an ungrouped one, must therefore have a unique
        // demand-one item id. This is the representation emitted by Uttu.
        if !specs.is_empty() {
            for item_id in 0..instance.items.len() {
                let demand = instance.item_qty(item_id);
                if demand != 1 {
                    return Err(PlaceGroupError::UnsupportedDemand { item_id, demand });
                }
            }
        }

        for spec in specs {
            if spec.members.len() < 2 {
                return Err(PlaceGroupError::TooFewMembers { group_id: spec.id });
            }
            let group_idx = groups.len();
            for member in &spec.members {
                if member.item_id >= instance.items.len() {
                    return Err(PlaceGroupError::UnknownItem {
                        group_id: spec.id,
                        item_id: member.item_id,
                    });
                }
                if item_to_group[member.item_id].replace(group_idx).is_some() {
                    return Err(PlaceGroupError::RepeatedItem {
                        item_id: member.item_id,
                    });
                }
            }
            groups.push(PlaceGroup {
                id: spec.id,
                members: spec.members,
            });
        }

        Ok(Self {
            groups,
            item_to_group,
        })
    }

    pub fn is_empty(&self) -> bool {
        self.groups.is_empty()
    }

    pub fn len(&self) -> usize {
        self.groups.len()
    }

    pub fn group_id_of(&self, item_id: usize) -> Option<PlaceGroupId> {
        self.group_index(item_id).map(|idx| self.groups[idx].id)
    }

    /// Exempt only members of the same explicit placement unit. None/None
    /// remains a strict item pair. Membership uses item ids, never SlotMap keys.
    #[inline]
    pub fn same_group(&self, a: usize, b: usize) -> bool {
        self.group_index(a)
            .is_some_and(|group| self.group_index(b) == Some(group))
    }

    pub fn representative(&self, item_id: usize) -> usize {
        self.group_index(item_id)
            .map(|idx| self.groups[idx].members[0].item_id)
            .unwrap_or(item_id)
    }

    pub fn is_representative(&self, item_id: usize) -> bool {
        self.representative(item_id) == item_id
    }

    pub fn members(&self, item_id: usize) -> &[PlaceGroupMember] {
        match self.group_index(item_id) {
            Some(idx) => &self.groups[idx].members,
            None => &[],
        }
    }

    pub fn member_item_ids(&self, item_id: usize) -> Vec<usize> {
        match self.group_index(item_id) {
            Some(idx) => self.groups[idx].members.iter().map(|m| m.item_id).collect(),
            None => vec![item_id],
        }
    }

    /// If one member is frozen, every member becomes frozen.
    pub fn expand_frozen_mask(&self, frozen: &mut [bool]) {
        for group in &self.groups {
            let group_frozen = group
                .members
                .iter()
                .any(|m| frozen.get(m.item_id).copied().unwrap_or(false));
            if group_frozen {
                for member in &group.members {
                    if let Some(slot) = frozen.get_mut(member.item_id) {
                        *slot = true;
                    }
                }
            }
        }
    }

    /// Verifies the all-or-none and exactly-once placement invariant needed by
    /// transactional group moves.
    pub fn validate_layout(&self, layout: &Layout) -> Result<(), PlaceGroupError> {
        for group in &self.groups {
            let mut n_present = 0;
            for member in &group.members {
                let count = layout
                    .placed_items()
                    .values()
                    .filter(|pi| pi.item().idx() == member.item_id)
                    .count();
                match count {
                    0 => {}
                    1 => n_present += 1,
                    _ => {
                        return Err(PlaceGroupError::RepeatedPlacement {
                            group_id: group.id,
                            item_id: member.item_id,
                        });
                    }
                }
            }
            if n_present != 0 && n_present != group.members.len() {
                return Err(PlaceGroupError::PartiallyPlaced { group_id: group.id });
            }
        }
        Ok(())
    }

    /// Checks a complete candidate against the original rigid relative poses.
    /// Geometry validation alone cannot detect a member moving within a group.
    pub fn solution_preserves_relative_poses(&self, solution: &SPSolution) -> bool {
        self.solution_preserves_relative_poses_impl(solution, false)
    }

    /// A partial seed may omit a whole placement unit. Once any member is
    /// present, every member must occur exactly once and retain its rigid pose.
    pub fn partial_solution_preserves_relative_poses(&self, solution: &SPSolution) -> bool {
        self.solution_preserves_relative_poses_impl(solution, true)
    }

    fn solution_preserves_relative_poses_impl(
        &self,
        solution: &SPSolution,
        allow_absent_groups: bool,
    ) -> bool {
        if self.is_empty() {
            return true;
        }
        let mut poses = vec![None; self.item_to_group.len()];
        for pi in solution.layout_snapshot().placed_items().values() {
            if self.group_index(pi.item().idx()).is_some() {
                if poses[pi.item().idx()].replace(pi.d_transf()).is_some() {
                    return false;
                }
            }
        }
        let finite = |dt: DTransformation| {
            let (x, y) = dt.translation();
            x.is_finite() && y.is_finite() && dt.rotation().is_finite()
        };
        self.groups.iter().all(|group| {
            if allow_absent_groups
                && group
                    .members
                    .iter()
                    .all(|member| poses[member.item_id].is_none())
            {
                return true;
            }
            let anchor = group.members[0];
            let Some(current_anchor) = poses[anchor.item_id] else {
                return false;
            };
            if !finite(anchor.initial_transform) || !finite(current_anchor) {
                return false;
            }
            // Compute the common delta once in f64. Composing f32/NotNan
            // matrices can overflow before an invalid candidate is rejected.
            let delta_angle =
                current_anchor.rotation() as f64 - anchor.initial_transform.rotation() as f64;
            let (sin, cos) = delta_angle.sin_cos();
            let (ix, iy) = anchor.initial_transform.translation();
            let (cx, cy) = current_anchor.translation();
            group.members.iter().all(|member| {
                let Some(actual) = poses[member.item_id] else {
                    return false;
                };
                if !finite(actual) || !finite(member.initial_transform) {
                    return false;
                }
                let (mx, my) = member.initial_transform.translation();
                let dx = mx as f64 - ix as f64;
                let dy = my as f64 - iy as f64;
                let ex = cx as f64 + cos * dx - sin * dy;
                let ey = cy as f64 + sin * dx + cos * dy;
                let (ax, ay) = actual.translation();
                let (ax, ay) = (ax as f64, ay as f64);
                let angle = (actual.rotation() as f64
                    - member.initial_transform.rotation() as f64
                    - delta_angle)
                    .rem_euclid(std::f64::consts::TAU);
                let tolerance = 1.0e-4
                    + 4.0
                        * f32::EPSILON as f64
                        * ax.abs().max(ay.abs()).max(ex.abs()).max(ey.abs());
                angle.min(std::f64::consts::TAU - angle) <= 1.0e-5
                    && (ax - ex).abs() <= tolerance
                    && (ay - ey).abs() <= tolerance
            })
        })
    }

    pub fn placement_key(&self, layout: &Layout, item_id: usize) -> Option<PItemKey> {
        layout
            .placed_items()
            .iter()
            .find_map(|(pk, pi)| (pi.item().idx() == item_id).then_some(pk))
    }

    pub(crate) fn placed_unit_representatives(&self, layout: &Layout) -> Vec<usize> {
        layout
            .placed_items()
            .values()
            .map(|pi| self.representative(pi.item().idx()))
            .collect::<std::collections::BTreeSet<_>>()
            .into_iter()
            .collect()
    }

    /// Returns all target member transforms when `anchor_item_id` is moved to
    /// `target`. With `layout`, every member must currently be placed there
    /// (otherwise [`PlaceGroupError::MissingPlacement`]); `None` is used by a
    /// fresh LBF where the members are not placed yet.
    ///
    /// Member targets are derived from the anchor target and the group's
    /// **initial** relative poses, never from the members' current poses.
    /// Composing each move onto the current f32 poses accumulated rounding
    /// drift: after a few hundred separator moves the members no longer matched
    /// their rigid relative pose within the 1e-4 mm certification tolerance, so
    /// zero-loss separations were rejected as failed attempts (about a third of
    /// them on a 35-piece marker with eight 2-3 piece groups). The computation
    /// mirrors [`Self::solution_preserves_relative_poses`] in f64, so a fresh
    /// target always certifies and any earlier drift is repaired by the next move.
    pub(crate) fn target_transforms(
        &self,
        layout: Option<&Layout>,
        anchor_item_id: usize,
        target: DTransformation,
    ) -> Result<Vec<(usize, DTransformation)>, PlaceGroupError> {
        let Some(group_idx) = self.group_index(anchor_item_id) else {
            return Ok(vec![(anchor_item_id, target)]);
        };
        let group = &self.groups[group_idx];
        if let Some(layout) = layout {
            for member in &group.members {
                if self.placement_key(layout, member.item_id).is_none() {
                    return Err(PlaceGroupError::MissingPlacement {
                        group_id: group.id,
                        item_id: member.item_id,
                    });
                }
            }
        }
        let anchor_initial = group
            .members
            .iter()
            .find(|m| m.item_id == anchor_item_id)
            .expect(crate::dmsg!("item_to_group and group members must agree"))
            .initial_transform;

        // Rigid delta from the anchor's initial pose to its target, in f64.
        let delta_angle = target.rotation() as f64 - anchor_initial.rotation() as f64;
        let (sin, cos) = delta_angle.sin_cos();
        let (ix, iy) = anchor_initial.translation();
        let (tx, ty) = target.translation();
        Ok(group
            .members
            .iter()
            .map(|member| {
                if member.item_id == anchor_item_id {
                    return (member.item_id, target);
                }
                let (mx, my) = member.initial_transform.translation();
                let dx = mx as f64 - ix as f64;
                let dy = my as f64 - iy as f64;
                let x = tx as f64 + cos * dx - sin * dy;
                let y = ty as f64 + sin * dx + cos * dy;
                let rotation = if delta_angle == 0.0 {
                    member.initial_transform.rotation()
                } else {
                    wrap_angle(member.initial_transform.rotation() as f64 + delta_angle) as f32
                };
                (member.item_id, DTransformation::new(rotation, (x as f32, y as f32)))
            })
            .collect())
    }

    fn group_index(&self, item_id: usize) -> Option<usize> {
        self.item_to_group.get(item_id).copied().flatten()
    }
}

#[cfg(test)]
mod collision_policy_tests {
    use super::*;
    use crate::config::DEFAULT_SWARM_CONFIG;
    use crate::optimizer::separator::Separator;
    use crate::quantify::tracker::CollisionTracker;
    use crate::structure::pipeline::{
        partial_solution_validation_error_with_frozen_ids_and_groups, solution_validation_error,
        solution_validation_error_with_frozen_and_groups,
    };
    use jagua_rs::probs::spp::entities::{SPPlacement, SPProblem};
    use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
    use rand::{SeedableRng, rngs::Xoshiro256PlusPlus};

    fn fixture(overlap_allowance: bool) -> SPProblem {
        let items: Vec<_> = (0..4).map(|id| serde_json::json!({
            "id":id, "demand":1, "allowed_orientations":[0.0],
            "shape":{"type":"simple_polygon", "data":[[0.0,0.0],[10.0,0.0],[10.0,10.0],[0.0,10.0]]}
        })).collect();
        let ext: ExtSPInstance = serde_json::from_value(
            serde_json::json!({"name":"policy", "strip_height":40.0,"items":items}),
        )
        .unwrap();
        let c = DEFAULT_SWARM_CONFIG;
        let instance = crate::util::io::import_spp_instance(
            &ext,
            c.cde_config,
            None,
            overlap_allowance.then_some(-0.4),
            None,
        )
        .unwrap();
        let mut p = SPProblem::new(instance).expect(crate::dmsg!("strip forms a valid container"));
        p.change_strip_width(200.0).expect(crate::dmsg!("strip width must be positive"));
        for (id, x) in [(0, 20.0), (1, 25.0), (2, 100.0), (3, 150.0)] {
            p.place_item(SPPlacement {
                item_idx: id,
                d_transf: DTransformation::new(0.0, (x, 20.0)),
            });
        }
        p
    }
    fn groups(p: &SPProblem, pairs: &[[usize; 2]]) -> PlaceGroups {
        PlaceGroups::new(
            &p.instance(),
            pairs
                .iter()
                .enumerate()
                .map(|(index, ids)| PlaceGroupSpec {
                    id: index as u64 + 42,
                    members: ids
                        .iter()
                        .map(|&id| PlaceGroupMember {
                            item_id: id,
                            initial_transform: p
                                .layout()
                                .placed_items()
                                .values()
                                .find(|pi| pi.item().idx() == id)
                                .unwrap()
                                .d_transf(),
                        })
                        .collect(),
                })
                .collect(),
        )
        .unwrap()
    }
    #[test]
    fn explicit_group_policy_isolated_from_empty_different_and_partial_membership() {
        for allowance in [false, true] {
            let p = fixture(allowance);
            let s = p.save();
            let g = groups(&p, &[[0, 1]]);
            let reject = || {
                assert!(solution_validation_error(&p.instance(), &s).is_some());
                for other in [
                    PlaceGroups::default(),
                    groups(&p, &[[0, 2], [1, 3]]),
                    groups(&p, &[[0, 2]]),
                ] {
                    assert!(
                        solution_validation_error_with_frozen_and_groups(
                            &p.instance(),
                            &s,
                            &[],
                            &other
                        )
                        .is_some()
                    );
                    if allowance {
                        assert!(
                            crate::original_overlap::validate_with_groups(
                                &p.instance(),
                                &s,
                                &[],
                                &other
                            )
                            .is_err()
                        );
                    }
                }
            };
            reject();
            assert!(g.same_group(0, 1));
            assert!(!g.same_group(2, 3));
            assert!(!g.same_group(0, 2));
            assert!(!g.same_group(99, 100));
            assert!(
                solution_validation_error_with_frozen_and_groups(&p.instance(), &s, &[], &g)
                    .is_none()
            );
            assert!(
                partial_solution_validation_error_with_frozen_ids_and_groups(
                    &p.instance(),
                    &s,
                    &[],
                    &g
                )
                .is_none()
            );
            assert!(
                crate::original_overlap::validate_with_groups(&p.instance(), &s, &[], &g).is_ok()
            );
            reject(); // A grouped successful certificate must never authorize strict validation.
            assert!(
                solution_validation_error_with_frozen_and_groups(&p.instance(), &s, &[], &g)
                    .is_none()
            );
        }
    }
    #[test]
    fn external_group_single_and_group_group_collisions_remain_rejected() {
        for other_group in [false, true] {
            let mut p = fixture(true);
            let key = p
                .layout()
                .placed_items()
                .iter()
                .find(|(_, pi)| pi.item().idx() == 2)
                .unwrap()
                .0;
            p.remove_item(key);
            p.place_item(SPPlacement {
                item_idx: 2,
                d_transf: DTransformation::new(0.0, (22.0, 20.0)),
            });
            let g = groups(
                &p,
                if other_group {
                    &[[0, 1], [2, 3]]
                } else {
                    &[[0, 1]]
                },
            );
            assert!(
                solution_validation_error_with_frozen_and_groups(&p.instance(), &p.save(), &[], &g)
                    .is_some()
            );
            assert!(
                crate::original_overlap::validate_with_groups(&p.instance(), &p.save(), &[], &g)
                    .is_err()
            );
            assert!(
                CollisionTracker::new_with_frozen_and_groups(&p.layout(), &[], &g).get_total_loss()
                    > 0.0
            );
        }
    }
    #[test]
    fn partial_group_validation_allows_absent_whole_group() {
        let p = fixture(false);
        let g = groups(&p, &[[0, 1]]);
        let mut s = p.save();
        s.layout_snapshot_mut()
            .placed_items_mut()
            .retain(|_, pi| pi.item().idx() >= 2);
        assert!(
            partial_solution_validation_error_with_frozen_ids_and_groups(&p.instance(), &s, &[], &g)
                .is_none()
        );
        assert!(
            solution_validation_error_with_frozen_and_groups(&p.instance(), &s, &[], &g).is_some()
        );
    }

    #[test]
    fn partial_group_validation_accepts_rigid_present_group_in_incomplete_layout() {
        let p = fixture(false);
        let g = groups(&p, &[[0, 1]]);
        let mut s = p.save();
        s.layout_snapshot_mut()
            .placed_items_mut()
            .retain(|_, pi| pi.item().idx() < 2);
        assert!(
            partial_solution_validation_error_with_frozen_ids_and_groups(&p.instance(), &s, &[], &g)
                .is_none()
        );
        assert!(
            solution_validation_error_with_frozen_and_groups(&p.instance(), &s, &[], &g).is_some()
        );
    }

    #[test]
    fn partial_group_validation_rejects_member_missing_from_present_group() {
        let p = fixture(false);
        let g = groups(&p, &[[0, 1]]);
        for missing_id in [0, 1] {
            let mut s = p.save();
            s.layout_snapshot_mut()
                .placed_items_mut()
                .retain(|_, pi| pi.item().idx() != missing_id);
            for _ in 0..2 {
                assert!(
                    partial_solution_validation_error_with_frozen_ids_and_groups(
                        &p.instance(),
                        &s,
                        &[],
                        &g
                    )
                    .is_some(),
                    "partly present group must not receive a partial certificate"
                );
            }
        }
    }

    #[test]
    fn partial_group_validation_rejects_drift_without_reusing_policy_cache() {
        let p = fixture(false);
        let g = groups(&p, &[[0, 1]]);
        let valid = p.save();
        assert!(
            partial_solution_validation_error_with_frozen_ids_and_groups(
                &p.instance(),
                &valid,
                &[],
                &g
            )
            .is_none()
        );
        for incomplete in [false, true] {
            let mut drift = valid.clone();
            if incomplete {
                drift
                    .layout_snapshot_mut()
                    .placed_items_mut()
                    .retain(|_, pi| pi.item().idx() < 2);
            }
            for pi in drift.layout_snapshot_mut().placed_items_mut().values_mut() {
                if pi.item().idx() == 1 {
                    pi.set_d_transf(DTransformation::new(0.0, (26.0, 20.0)));
                }
            }
            let redefined = PlaceGroups::new(
                &p.instance(),
                vec![PlaceGroupSpec {
                    id: 42,
                    members: drift
                        .layout_snapshot()
                        .placed_items()
                        .values()
                        .filter(|pi| pi.item().idx() < 2)
                        .map(|pi| PlaceGroupMember {
                            item_id: pi.item().idx(),
                            initial_transform: pi.d_transf(),
                        })
                        .collect(),
                }],
            )
            .unwrap();
            assert!(
                partial_solution_validation_error_with_frozen_ids_and_groups(
                    &p.instance(),
                    &drift,
                    &[],
                    &redefined
                )
                .is_none()
            );
            for _ in 0..2 {
                assert!(
                    partial_solution_validation_error_with_frozen_ids_and_groups(
                        &p.instance(),
                        &drift,
                        &[],
                        &g
                    )
                    .is_some(),
                    "cached certificate under another relative-pose definition must not accept group drift"
                );
            }
        }
        assert!(
            partial_solution_validation_error_with_frozen_ids_and_groups(
                &p.instance(),
                &valid,
                &[],
                &g
            )
            .is_none()
        );
    }

    #[test]
    fn group_policy_keeps_boundary_rotation_quantity_and_relative_pose_checks() {
        let p = fixture(false);
        let g = groups(&p, &[[0, 1]]);
        let s = p.save();
        let validate = |candidate: &SPSolution| {
            solution_validation_error_with_frozen_and_groups(&p.instance(), candidate, &[], &g)
        };
        assert!(validate(&s).is_none());
        let key = |id| {
            s.layout_snapshot()
                .placed_items()
                .iter()
                .find(|(_, pi)| pi.item().idx() == id)
                .unwrap()
                .0
        };
        let mut drift = s.clone();
        drift.layout_snapshot_mut().placed_items_mut()[key(1)].set_d_transf(DTransformation::new(0.0, (26.0, 20.0)));
        assert!(validate(&drift).is_some());
        let mut angle = s.clone();
        angle.layout_snapshot_mut().placed_items_mut()[key(0)].set_d_transf(DTransformation::new(0.3, (20.0, 20.0)));
        assert!(validate(&angle).is_some());
        let mut missing = s.clone();
        missing.layout_snapshot_mut().placed_items_mut().remove(key(1));
        assert!(validate(&missing).is_some());
        let mut duplicate = s.clone();
        { let pi = &mut duplicate.layout_snapshot_mut().placed_items_mut()[key(2)]; pi.set_item(std::sync::Arc::new(pi.item().as_ref().clone().with_idx(1))); }
        assert!(validate(&duplicate).is_some());
        let mut unknown = s.clone();
        { let pi = &mut unknown.layout_snapshot_mut().placed_items_mut()[key(2)]; pi.set_item(std::sync::Arc::new(pi.item().as_ref().clone().with_idx(99))); }
        assert!(validate(&unknown).is_some());
        let mut outside = s.clone();
        for id in [0, 1] {
            let dt = outside.layout_snapshot().placed_items()[key(id)].d_transf();
            outside.layout_snapshot_mut().placed_items_mut()[key(id)].set_d_transf(DTransformation::new(0.0, (dt.translation().0, -20.0)));
        }
        assert!(validate(&outside).is_some());
        // Preserve the existing frozen boundary exemption without permitting pose drift.
        let frozen = [true, true, false, false];
        assert!(
            solution_validation_error_with_frozen_and_groups(&p.instance(), &outside, &frozen, &g)
                .is_none()
        );
        assert!(!crate::util::assertions::frozen_poses_preserved(
            &s, &outside, &frozen
        ));
    }
    #[test]
    fn grouped_tracker_survives_clone_copy_rekey_rollback_resync_and_freeze() {
        let p = fixture(false);
        let g = groups(&p, &[[0, 1]]);
        assert!(CollisionTracker::new(&p.layout()).get_total_loss() > 0.0);
        let ct = CollisionTracker::new_with_frozen_and_groups(&p.layout(), &[], &g);
        assert_eq!(ct.get_total_loss(), 0.0);
        let mut copy = CollisionTracker::new(&p.layout());
        copy.copy_from(&ct.clone());
        assert_eq!(copy.get_total_loss(), 0.0);
        assert_eq!(copy.place_groups, g);
        let mut config = DEFAULT_SWARM_CONFIG.expl_cfg.separator_config;
        config.n_workers = 2;
        config.nfp_sampler = false;
        let mut sep = Separator::new(
            p.instance().clone(),
            p,
            Xoshiro256PlusPlus::seed_from_u64(11),
            config,
        );
        sep.set_place_groups(g.clone()).unwrap();
        assert_eq!(sep.ct.get_total_loss(), 0.0);
        let saved = sep.prob.save();
        let snap = sep.ct.save();
        let old_key = sep.placement_key(0).unwrap();
        let pose = sep.prob.layout().placed_items()[old_key].d_transf();
        let new_key = sep.move_item(
            old_key,
            DTransformation::new(0.0, (pose.translation().0 + 5.0, pose.translation().1)),
        );
        assert_ne!(old_key, new_key);
        assert_eq!(sep.ct.get_total_loss(), 0.0);
        assert!(g.solution_preserves_relative_poses(&sep.prob.save()));
        sep.rollback(&saved, Some(&snap));
        assert_eq!(sep.ct.get_total_loss(), 0.0);
        sep.rollback(&saved, None);
        assert_eq!(sep.ct.get_total_loss(), 0.0);
        sep.change_strip_width(210.0, Some(190.0));
        assert_eq!(sep.ct.get_total_loss(), 0.0);
        assert_eq!(sep.ct.place_groups, g);
        assert!(sep.solution_is_certified(&sep.prob.save(), &saved));
        sep.freeze_items(&[1]);
        assert!(sep.frozen_mask[0] && sep.frozen_mask[1]);
        assert_eq!(sep.ct.get_total_loss(), 0.0);
        let frozen = sep.prob.save();
        let key = sep.placement_key(0).unwrap();
        sep.move_item(key, DTransformation::new(0.0, (80.0, 20.0)));
        assert!(crate::util::assertions::frozen_poses_preserved(
            &frozen,
            &sep.prob.save(),
            &sep.frozen_mask
        ));
    }
}

/// Wraps an angle into (-pi, pi], the range `decompose` produces.
fn wrap_angle(angle: f64) -> f64 {
    use std::f64::consts::{PI, TAU};
    let wrapped = (angle + PI).rem_euclid(TAU) - PI;
    if wrapped <= -PI { wrapped + TAU } else { wrapped }
}

#[cfg(test)]
mod drift_tests {
    use super::*;
    use crate::config::DEFAULT_SWARM_CONFIG;
    use crate::optimizer::separator::Separator;
    use jagua_rs::probs::spp::entities::{SPPlacement, SPProblem};
    use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
    use rand::{RngExt, SeedableRng, rngs::Xoshiro256PlusPlus};

    /// Many separator moves of one group must keep its rigid relative pose
    /// exactly enough to certify. Composing each move onto the members'
    /// current f32 poses drifted past the 1e-4 mm tolerance.
    #[test]
    fn repeated_group_moves_keep_the_relative_pose_certifiable() {
        let items: Vec<_> = (0..4)
            .map(|id| serde_json::json!({
                "id":id, "demand":1, "allowed_orientations":[0.0, 90.0, 180.0, 270.0],
                "shape":{"type":"simple_polygon",
                         "data":[[0.0,0.0],[37.3,0.0],[41.9,23.7],[12.1,31.3],[0.0,17.9]]}
            }))
            .collect();
        let ext: ExtSPInstance = serde_json::from_value(
            serde_json::json!({"name":"drift","strip_height":600.0,"items":items}),
        )
        .unwrap();
        let c = DEFAULT_SWARM_CONFIG;
        let instance = crate::util::io::import_spp_instance(&ext, c.cde_config, None, None, None).unwrap();
        let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
        prob.change_strip_width(1500.0).expect(crate::dmsg!("strip width must be positive"));
        let initial = [(0, 100.3, 200.7), (1, 143.9, 201.1), (2, 121.7, 241.3), (3, 900.0, 300.0)];
        for (id, x, y) in initial {
            prob.place_item(SPPlacement { item_idx: id, d_transf: DTransformation::new(0.0, (x, y)) });
        }
        let groups = PlaceGroups::new(
            &instance,
            vec![PlaceGroupSpec {
                id: 1,
                members: initial[..3]
                    .iter()
                    .map(|&(id, x, y)| PlaceGroupMember {
                        item_id: id,
                        initial_transform: DTransformation::new(0.0, (x, y)),
                    })
                    .collect(),
            }],
        )
        .unwrap();
        let mut cfg = c.expl_cfg.separator_config;
        cfg.n_workers = 1;
        let mut sep = Separator::new(instance, prob, Xoshiro256PlusPlus::seed_from_u64(5), cfg);
        sep.set_place_groups(groups.clone()).unwrap();
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(9);
        let angles = [0.0f32, std::f32::consts::FRAC_PI_2, std::f32::consts::PI, -std::f32::consts::FRAC_PI_2];
        for _ in 0..3000 {
            let pk = groups.placement_key(&sep.prob.layout(), 0).unwrap();
            let rotation = angles[rng.random_range(0..angles.len())];
            let (x, y) = (rng.random_range(150.0f32..1300.0), rng.random_range(150.0f32..450.0));
            sep.move_item(pk, DTransformation::new(rotation, (x, y)));
        }
        assert!(
            groups.solution_preserves_relative_poses(&sep.prob.save()),
            "group relative pose drifted after repeated moves"
        );
    }

    #[test]
    fn wrap_angle_matches_the_decompose_range() {
        use std::f64::consts::PI;
        for (input, expected) in [(0.0, 0.0), (PI, PI), (-PI, PI), (3.0 * PI, PI), (2.5 * PI, 0.5 * PI), (-0.5 * PI, -0.5 * PI)] {
            assert!((wrap_angle(input) - expected).abs() < 1e-12, "{input} -> {}", wrap_angle(input));
        }
    }
}
