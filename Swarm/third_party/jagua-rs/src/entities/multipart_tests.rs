// CAPLANET: tests for multipart items (change 10 in CAPLANET_CHANGES.md).
use crate::collision_detection::CDEConfig;
use crate::collision_detection::hazards::BasicHazardEntity;
use crate::collision_detection::hazards::filter::{NoFilter, PlacedItemFilter};
use crate::entities::Layout;
use crate::geometry::DTransformation;
use crate::geometry::fail_fast::SPSurrogateConfig;
use crate::geometry::geo_traits::Transformable;
use crate::io::ext_repr::{
    ExtContainer, ExtItem, ExtOrientation, ExtPolygon, ExtRotation, ExtSPolygon, ExtShape,
};
use crate::io::import::Importer;
use std::sync::Arc;

fn importer() -> Importer {
    Importer::new(
        CDEConfig {
            quadtree_depth: 5,
            cd_threshold: 96,
            item_surrogate_config: SPSurrogateConfig::none(),
        },
        None,
        None,
    )
}

fn square(x0: f32, y0: f32, w: f32) -> ExtPolygon {
    ExtPolygon {
        outer: ExtSPolygon(vec![(x0, y0), (x0 + w, y0), (x0 + w, y0 + w), (x0, y0 + w)]),
        inner: vec![],
    }
}

fn fixed() -> ExtOrientation {
    ExtOrientation {
        rotation: ExtRotation::Discrete { angles: vec![0.0] },
        reflection_axes: vec![],
    }
}

/// Two unit squares with a gap of two units between them.
fn two_squares(idx: usize, id: u64) -> Arc<crate::entities::Item> {
    importer()
        .import_item(
            &ExtItem {
                id,
                orientation: fixed(),
                shape: ExtShape::MultiPolygon(vec![square(0.0, 0.0, 1.0), square(3.0, 0.0, 1.0)]),
                min_quality: None,
            },
            idx,
        )
        .unwrap()
}

fn unit_square(idx: usize, id: u64) -> Arc<crate::entities::Item> {
    importer()
        .import_item(
            &ExtItem {
                id,
                orientation: fixed(),
                shape: ExtShape::Rectangle { x_min: 0.0, y_min: 0.0, width: 1.0, height: 1.0 },
                min_quality: None,
            },
            idx,
        )
        .unwrap()
}

fn layout() -> Layout {
    let container = importer()
        .import_container(&ExtContainer {
            id: 0,
            shape: ExtShape::Rectangle { x_min: 0.0, y_min: 0.0, width: 10.0, height: 10.0 },
            zones: vec![],
        })
        .unwrap();
    Layout::new(container)
}

#[test]
fn multipart_item_keeps_parts_in_one_centred_frame_with_a_hull_envelope() {
    let item = two_squares(0, 7);
    assert_eq!(item.n_parts(), 2);
    assert!(item.is_multipart());
    assert!((item.area() - 2.0).abs() < 1e-6, "area is the sum of the parts");
    let hull = item.shape_cd().bbox();
    assert!((hull.width() - 4.0).abs() < 1e-5 && (hull.height() - 1.0).abs() < 1e-5);
    assert!((hull.x_min + 2.0).abs() < 1e-5, "the union is centred");
    let gap = item.parts_cd()[1].bbox().x_min - item.parts_cd()[0].bbox().x_max;
    assert!((gap - 2.0).abs() < 1e-5);
    for part in item.parts_orig() {
        assert_eq!(part.pre_transform, item.shape_orig().pre_transform);
    }
    let single = unit_square(1, 8);
    assert_eq!(single.n_parts(), 1);
    assert!(Arc::ptr_eq(&single.parts_cd()[0], single.shape_cd()));
}

#[test]
fn instance_item_area_counts_the_parts_not_the_hull() {
    use crate::geometry::shape_modification::ShapeModifyConfig;
    use crate::probs::spp::entities::{SPInstance, Strip};
    let strip = Strip::new(
        10.0,
        importer().cde_config,
        ShapeModifyConfig { offset: None, simplify_tolerance: None, narrow_concavity_cutoff: None },
        10.0,
    )
    .unwrap();
    let instance = SPInstance::new(vec![(two_squares(0, 7), 1)], strip);
    assert!((instance.item_area() - 2.0).abs() < 1e-6, "hull would be 4.0");
}

#[test]
fn a_placed_multipart_item_registers_one_hazard_per_part_and_frees_the_gap() {
    let mut layout = layout();
    let item = two_squares(0, 7);
    let pk = layout.place_item(&item, DTransformation::new(0.0, (5.0, 5.0)));
    // exterior + two parts
    assert_eq!(layout.cde().hazards().count(), 3);
    assert_eq!(layout.haz_keys(pk).len(), 2);
    assert_eq!(layout.placed_items()[pk].n_parts(), 2);
    assert!(layout.is_collision_free());

    let probe = unit_square(1, 8);
    let in_gap = probe.shape_cd().transform_clone(&DTransformation::new(0.0, (5.0, 5.0)).compose());
    assert!(!layout.cde().detect_poly_collision(&in_gap, &NoFilter), "the gap between parts is free");
    let on_part = probe.shape_cd().transform_clone(&DTransformation::new(0.0, (3.5, 5.0)).compose());
    assert!(layout.cde().detect_poly_collision(&on_part, &NoFilter), "a part is a hazard");
    assert!(
        !layout.cde().detect_poly_collision(&on_part, &PlacedItemFilter(pk)),
        "the placement's own parts are ignored together"
    );

    let probe_pk = layout.place_item(&probe, DTransformation::new(0.0, (5.0, 5.0)));
    assert!(layout.is_collision_free());
    layout.remove_item(probe_pk);
    layout.place_item(&probe, DTransformation::new(0.0, (3.5, 5.0)));
    assert!(!layout.is_collision_free());
}

#[test]
fn multipart_snapshots_restore_every_part_and_removal_deregisters_them() {
    let mut layout = layout();
    let item = two_squares(0, 7);
    let empty = layout.save();
    let pk = layout.place_item(&item, DTransformation::new(0.0, (5.0, 5.0)));
    let placed = layout.save();
    assert_eq!(placed.placed_items().len(), 1);
    layout.restore(&empty).unwrap();
    assert_eq!(layout.cde().hazards().count(), 1);
    layout.restore(&placed).unwrap();
    assert_eq!(layout.cde().hazards().count(), 3);
    let entities: Vec<_> = layout.cde().hazards().map(|h| h.entity).collect();
    for part in 0..2 {
        assert!(entities.iter().any(|e| matches!(e, BasicHazardEntity::PlacedItem { pk: p, part: q, .. } if *p == pk && *q == part)));
    }
    layout.remove_item(pk);
    assert_eq!(layout.cde().hazards().count(), 1);
}
