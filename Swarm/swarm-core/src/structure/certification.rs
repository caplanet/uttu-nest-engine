//! Successful certificates only. Own geometry Arcs so address reuse cannot hit
//! an old certificate; compare complete inputs rather than probabilistic hashes.
use jagua_rs::{
    entities::PItemKey,
    geometry::DTransformation,
    probs::spp::entities::{SPInstance, SPSolution},
};
use std::{cell::RefCell, sync::Arc};

struct Certificate {
    instance: SPInstance,
    strip: jagua_rs::probs::spp::entities::Strip,
    poses: Vec<(PItemKey, usize, DTransformation)>,
    frozen: Vec<bool>,
    groups: crate::optimizer::place_groups::PlaceGroups,
    complete: bool,
}
thread_local! { static CACHE: RefCell<Vec<Certificate>> = const { RefCell::new(Vec::new()) }; }

/// Frozen masks compare with "missing = false" semantics: callers pass either
/// a full per-item mask or `&[]` when nothing is frozen, and both must match a
/// certificate recorded with an all-false mask of any length.
fn frozen_equal(a: &[bool], b: &[bool]) -> bool {
    let at = |m: &[bool], i: usize| m.get(i).copied().unwrap_or(false);
    (0..a.len().max(b.len())).all(|i| at(a, i) == at(b, i))
}

fn matches(
    c: &Certificate,
    inst: &SPInstance,
    sol: &SPSolution,
    complete: bool,
    frozen: &[bool],
    groups: &crate::optimizer::place_groups::PlaceGroups,
) -> bool {
    c.complete == complete
        && frozen_equal(&c.frozen, frozen)
        && &c.groups == groups
        && c.strip == *sol.strip()
        && c.instance.base_strip == inst.base_strip
        && c.instance.items.len() == inst.items.len()
        && c.instance
            .items
            .iter()
            .zip(&inst.items)
            .all(|((a, aq), (b, bq))| {
                aq == bq
                    && a.idx() == b.idx()
                    && a.allowed_orientations().rotations() == b.allowed_orientations().rotations()
                    && a.min_quality() == b.min_quality()
                    && Arc::ptr_eq(&a.shape_orig(), &b.shape_orig())
                    && Arc::ptr_eq(&a.shape_cd(), &b.shape_cd())
            })
        && c.poses.iter().copied().eq(sol
            .layout_snapshot()
            .placed_items()
            .iter()
            .map(|(key, p)| (key, p.item().idx(), p.d_transf())))
}

pub(super) fn contains(
    inst: &SPInstance,
    sol: &SPSolution,
    complete: bool,
    frozen: &[bool],
    groups: &crate::optimizer::place_groups::PlaceGroups,
) -> bool {
    CACHE.with(|cache| {
        cache
            .borrow()
            .iter()
            .any(|c| matches(c, inst, sol, complete, frozen, groups))
    })
}

pub(super) fn insert(
    inst: &SPInstance,
    sol: &SPSolution,
    complete: bool,
    frozen: &[bool],
    groups: &crate::optimizer::place_groups::PlaceGroups,
) {
    CACHE.with(|cache| {
        let mut cache = cache.borrow_mut();
        if cache.len() == 4 {
            cache.remove(0);
        }
        cache.push(Certificate {
            instance: inst.clone(),
            strip: *sol.strip(),
            complete,
            poses: sol
                .layout_snapshot()
                .placed_items()
                .iter()
                .map(|(k, p)| (k, p.item().idx(), p.d_transf()))
                .collect(),
            frozen: frozen.to_vec(),
            groups: groups.clone(),
        });
    });
}

#[cfg(test)]
mod tests {
    use super::frozen_equal;

    #[test]
    fn empty_mask_matches_all_false_mask() {
        assert!(frozen_equal(&[], &[false, false, false]));
        assert!(frozen_equal(&[false, false], &[]));
        assert!(frozen_equal(&[], &[]));
    }

    #[test]
    fn differing_frozen_bits_do_not_match() {
        assert!(!frozen_equal(&[], &[false, true]));
        assert!(!frozen_equal(&[true], &[]));
        assert!(!frozen_equal(&[true, false], &[false, true]));
        assert!(frozen_equal(&[true, false], &[true]));
    }
}
