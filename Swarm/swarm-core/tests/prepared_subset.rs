use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
use swarm_core::{
    config::DEFAULT_SWARM_CONFIG,
    util::io::{import_spp_instance_with_mm, prepared_spp_subset},
};
use std::sync::Arc;

fn input() -> ExtSPInstance {
    serde_json::from_value(serde_json::json!({"name":"prepared", "strip_height":1000,
        "items":(0..3).map(|id| serde_json::json!({"id":id,"demand":id+1,
            "allowed_orientations":[0,180], "shape":{"type":"simple_polygon",
            "data":[[10,20],[50,19.8],[110,20],[110,120],[10,120]]}})).collect::<Vec<_>>()}))
    .unwrap()
}

#[test]
fn shared_subset_matches_reimport_for_both_simplifiers_and_preserves_identity() {
    for mm in [None, Some(0.5)] {
        let ext = input();
        let cde = DEFAULT_SWARM_CONFIG.cde_config;
        let full =
            import_spp_instance_with_mm(&ext, cde, Some(0.01), Some(-2.2), None, mm).unwrap();
        let ids = [2, 0];
        let shared = prepared_spp_subset(&full, &ids, Some(-2.2)).unwrap();
        let mut local = ext.clone();
        local.items = ids
            .iter()
            .enumerate()
            .map(|(id, &global)| {
                let mut item = ext.items[global].clone();
                item.base.id = id as u64;
                item
            })
            .collect();
        let fresh =
            import_spp_instance_with_mm(&local, cde, Some(0.01), Some(-2.2), None, mm).unwrap();
        assert_eq!(shared.base_strip, fresh.base_strip);
        for (id, &global) in ids.iter().enumerate() {
            let (a, qty) = &shared.items[id];
            let (b, bqty) = &fresh.items[id];
            assert_eq!(a.idx(), id);
            assert_eq!(qty, bqty);
            assert!(Arc::ptr_eq(&a.shape_orig(), &full.items[global].0.shape_orig()));
            assert!(Arc::ptr_eq(&a.shape_cd(), &full.items[global].0.shape_cd()));
            assert_eq!(a.shape_cd().vertices(), b.shape_cd().vertices());
            assert_eq!(a.shape_orig().pre_transform, b.shape_orig().pre_transform);
            assert_eq!(a.allowed_orientations().rotations(), b.allowed_orientations().rotations());
            assert_eq!(a.min_quality(), b.min_quality());
        }
    }
}

#[test]
fn subset_recomputes_the_narrow_strip_offset_cap() {
    let mut ext = input();
    ext.items.truncate(2);
    for item in &mut ext.items {
        item.demand = 1;
    }
    let cde = DEFAULT_SWARM_CONFIG.cde_config;
    let full = import_spp_instance_with_mm(&ext, cde, None, Some(-12.), None, None).unwrap();
    let subset = prepared_spp_subset(&full, &[0], Some(-12.)).unwrap();
    assert_eq!(full.base_strip.shape_modify_config.offset, Some(6.));
    assert_eq!(
        subset.base_strip.shape_modify_config.offset,
        Some(0.45 * subset.base_strip.width)
    );
    assert!(subset.base_strip.shape_modify_config.offset.unwrap() < 6.);
}

#[test]
fn invalid_or_duplicate_subset_ids_are_errors() {
    let ext = input();
    let full = import_spp_instance_with_mm(
        &ext,
        DEFAULT_SWARM_CONFIG.cde_config,
        None,
        None,
        None,
        None,
    )
    .unwrap();
    for ids in [&[][..], &[0, 0], &[3]] {
        assert!(prepared_spp_subset(&full, ids, None).is_err());
    }
}
