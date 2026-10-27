//! User locks remain obstacles, but lock/lock and lock/exterior conflicts are
//! permitted. The spatial index may extend beyond the physical strip.
use jagua_rs::collision_detection::CDEngine;
use jagua_rs::entities::Container;
use jagua_rs::geometry::primitives::Rect;
use jagua_rs::probs::spp::entities::{SPProblem, SPSolution};
use std::sync::Arc;

pub fn mask(size: usize, ids: &[usize]) -> Vec<bool> {
    let mut result = vec![false; size];
    for &id in ids {
        if let Some(value) = result.get_mut(id) {
            *value = true;
        }
    }
    result
}

pub fn is_frozen(mask: &[bool], id: usize) -> bool {
    mask.get(id).copied().unwrap_or(false)
}

fn include(root: &mut Rect, bbox: Rect) {
    root.x_min = root.x_min.min(bbox.x_min);
    root.y_min = root.y_min.min(bbox.y_min);
    root.x_max = root.x_max.max(bbox.x_max);
    root.y_max = root.y_max.max(bbox.y_max);
}

fn expand(container: &mut Container, boxes: impl IntoIterator<Item = Rect>) -> bool {
    let original = container.base_cde().bbox();
    let mut root = original;
    for bbox in boxes {
        include(&mut root, bbox);
    }
    if root == original {
        return false;
    }
    // This only changes index coverage. Exterior geometry and CDE configuration
    // stay identical, so movable pieces still see the original strip boundary.
    let source = container.base_cde();
    let widened = Arc::new(CDEngine::new(
        root.inflate_to_square(),
        source.hazards().cloned().collect(),
        source.config(),
    ));
    container.set_base_cde(widened);
    true
}

pub fn prepare_placements(prob: &mut SPProblem, boxes: impl IntoIterator<Item = Rect>) {
    let mut container = prob.layout().container().clone();
    if expand(&mut container, boxes) {
        prob.layout_mut().swap_container(container);
    }
}

pub fn change_strip_width(prob: &mut SPProblem, width: f32) {
    // Widen the new container's index root before the placed items are
    // re-registered, so locked items outside the strip stay valid hazards.
    let boxes: Vec<Rect> = prob
        .layout()
        .placed_items()
        .values()
        .map(|p| p.shape().bbox())
        .collect();
    prob.change_strip_width_with(width, |container| {
        expand(container, boxes);
    })
    .expect(crate::dmsg!("strip width must be positive"));
}

pub fn restore(prob: &mut SPProblem, solution: &SPSolution) {
    // jagua-rs 1.x `SPProblem::restore` rebuilds the layout itself when the
    // snapshot's container (strip width or widened index root) differs.
    prob.restore(solution);
}

/// Equivalent fitting arithmetic without requiring exempt locks to become
/// physically feasible. Empty/all-left layouts retain a positive strip width.
pub fn fit_strip(prob: &mut SPProblem) {
    fit_strip_with_frozen(prob, &[]);
}

pub fn fit_strip_with_frozen(prob: &mut SPProblem, frozen: &[bool]) {
    if let Some(right) = prob
        .layout()
        .placed_items()
        .values()
        .filter(|p| !is_frozen(frozen, p.item().idx()))
        .map(|p| p.shape().bbox().x_max)
        .reduce(f32::max)
    {
        let width = right * 1.00001 + prob.strip().shape_modify_config.offset.unwrap_or(0.0);
        if width.is_finite() && width > 0.0 {
            change_strip_width(prob, width);
        }
    }
}

/// Rebuild an already validated public solution without importing its locks
/// into an index restricted to the strip rectangle.
pub fn import_solution(
    instance: &jagua_rs::probs::spp::entities::SPInstance,
    solution: &jagua_rs::probs::spp::io::ext_repr::ExtSPSolution,
) -> SPProblem {
    use jagua_rs::geometry::{DTransformation, geo_traits::Transformable};
    use jagua_rs::probs::spp::entities::SPPlacement;
    let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
    change_strip_width(&mut prob, solution.strip_width);
    let placements = solution
        .layout
        .placed_items
        .iter()
        .map(|placed| {
            let item_id = placed.item_id as usize;
            let dt = jagua_rs::io::import::ext_to_int_transformation(
                &DTransformation::from(placed.transformation.clone()),
                &instance.item(item_id).shape_orig().pre_transform,
            );
            SPPlacement {
                item_idx: item_id,
                d_transf: dt,
            }
        })
        .collect::<Vec<_>>();
    let boxes = placements
        .iter()
        .map(|p| {
            instance
                .item(p.item_idx)
                .shape_cd()
                .transform_clone(&p.d_transf.compose())
                .bbox()
        })
        .collect::<Vec<_>>();
    prepare_placements(&mut prob, boxes);
    for placement in placements {
        prob.place_item(placement);
    }
    prob
}
