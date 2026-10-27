#![cfg(feature = "structure-first")]

use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::primitives::Rect;
use jagua_rs::probs::spp::entities::SPInstance;
use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::optimizer::lbf::{LBFBuilder, StructureSelectionSource};
use swarm_core::sample::search::SampleConfig;
use swarm_core::structure::evaluator::ProfilePlacementConfig;
use swarm_core::structure::motif::MotifCatalog;
use swarm_core::structure::motif::{
    MotifEnvelope, MotifItemRef, MotifKind, MotifMember, MotifScore, PairCorrelationMethod,
    PairSearchMetadata, PlacementMotif,
};

fn two_rectangles() -> SPInstance {
    let external: ExtSPInstance = serde_json::from_value(serde_json::json!({
        "name": "p1-beam-order",
        "strip_height": 10.0,
        "items": [
            {
                "id": 0,
                "demand": 1,
                "allowed_orientations": [0.0],
                "shape": { "type": "simple_polygon", "data": [
                    [0.0, 0.0], [8.0, 0.0], [8.0, 8.0], [0.0, 8.0], [0.0, 0.0]
                ]}
            },
            {
                "id": 1,
                "demand": 1,
                "allowed_orientations": [0.0],
                "shape": { "type": "simple_polygon", "data": [
                    [0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0], [0.0, 0.0]
                ]}
            }
        ]
    }))
    .unwrap();
    let cfg = DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(cfg.cde_config, cfg.poly_simpl_tolerance, cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&importer, &external).unwrap()
}

fn one_unplaceable_then_one_placeable() -> SPInstance {
    let external: ExtSPInstance = serde_json::from_value(serde_json::json!({
        "name": "p1-bounded-recovery",
        "strip_height": 10.0,
        "items": [
            {
                "id": 0,
                "demand": 1,
                "allowed_orientations": [0.0],
                "shape": { "type": "simple_polygon", "data": [
                    [0.0, 0.0], [12.0, 0.0], [12.0, 12.0], [0.0, 12.0], [0.0, 0.0]
                ]}
            },
            {
                "id": 1,
                "demand": 1,
                "allowed_orientations": [0.0],
                "shape": { "type": "simple_polygon", "data": [
                    [0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0], [0.0, 0.0]
                ]}
            }
        ]
    }))
    .unwrap();
    let cfg = DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(cfg.cde_config, cfg.poly_simpl_tolerance, cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&importer, &external).unwrap()
}

fn samples() -> SampleConfig {
    SampleConfig {
        n_container_samples: 8,
        n_focussed_samples: 0,
        n_coord_descents: 2,
        left_bias_gain: 0.0,
        tilt_range: 0.0,
    }
}

fn profile() -> ProfilePlacementConfig {
    ProfilePlacementConfig {
        row_height: 0.25,
        max_rows: 128,
        top_k: 1,
        item_beam_width: 8,
        candidate_pool_cap: 32,
        slide_refine_steps: 12,
        ..ProfilePlacementConfig::default()
    }
}

fn builder(instance: SPInstance, seed: u64) -> LBFBuilder {
    LBFBuilder::new(
        instance,
        Xoshiro256PlusPlus::seed_from_u64(seed),
        samples(),
        10.0,
        1.0,
    )
}

#[test]
fn structure_beam_can_choose_a_different_item_than_fixed_area_order() {
    let instance = two_rectangles();
    let legacy = builder(instance.clone(), 7).construct();
    let structured = builder(instance, 7)
        .with_structure_hints(profile(), MotifCatalog::default(), 1)
        .construct();

    let legacy_first = legacy
        .prob
        .layout()
        .placed_items()
        .iter()
        .next()
        .unwrap()
        .1
        .item().idx();
    let structure_first = structured
        .prob
        .layout()
        .placed_items()
        .iter()
        .next()
        .unwrap()
        .1
        .item().idx();
    assert_eq!(legacy_first, 0, "legacy must retain descending area order");
    assert_eq!(
        structure_first, 1,
        "P1 must compare remaining item choices instead of consuming fixed order"
    );
}

#[test]
fn structure_constructor_commits_only_exact_feasible_placements() {
    let instance = two_rectangles();
    let built = builder(instance.clone(), 11)
        .with_structure_hints(profile(), MotifCatalog::default(), 1)
        .construct();
    assert_eq!(
        built.prob.layout().placed_items().len(),
        instance.total_item_qty()
    );
    assert!(built.prob.layout().is_collision_free());
    assert!(built.prob.item_demand_qtys().iter().all(|qty| *qty == 0));
}

#[test]
fn structure_constructor_never_leaks_relaxed_tilt_into_discrete_items() {
    let instance = two_rectangles();
    let mut relaxed = samples();
    relaxed.tilt_range = 0.25;
    let built = LBFBuilder::new(
        instance,
        Xoshiro256PlusPlus::seed_from_u64(12),
        relaxed,
        10.0,
        1.0,
    )
    .with_structure_hints(profile(), MotifCatalog::default(), 1)
    .construct();
    assert!(built.prob.layout().placed_items().values().all(|placed| {
        let rotation = placed.d_transf().rotation();
        rotation.sin().atan2(rotation.cos()).abs() <= 1.0e-4
    }));
}

#[test]
fn structure_constructor_honours_cooperative_stop_inside_candidate_search() {
    let instance = two_rectangles();
    let mut polls = 0usize;
    let mut stop = || {
        polls += 1;
        polls >= 5
    };
    let built = builder(instance.clone(), 13)
        .with_structure_hints(profile(), MotifCatalog::default(), 1)
        .construct_with_control(&mut stop);
    assert!(
        polls < 64,
        "bounded P1 search must poll and return promptly"
    );
    assert!(built.prob.layout().placed_items().len() < instance.total_item_qty());
    let stats = built
        .structure_hint_stats()
        .expect("the structure branch must expose its stopped start");
    assert!(stats.start.stopped);
    assert!(!stats.start.completed);
    assert_eq!(
        stats.start.committed_members,
        built.prob.layout().placed_items().len()
    );
}

#[test]
fn beam_one_skips_a_stalled_anchor_and_returns_bounded_partial() {
    let instance = one_unplaceable_then_one_placeable();
    let mut recovery_profile = profile();
    recovery_profile.item_beam_width = 1;
    let built = builder(instance, 31)
        .with_structure_hints(recovery_profile, MotifCatalog::default(), 1)
        .construct();
    let stats = built.structure_hint_stats().unwrap();

    assert_eq!(
        built.prob.layout().placed_items().len(),
        1,
        "beam=1 must defer the impossible first anchor and try the next copy"
    );
    assert_eq!(
        built
            .prob
            .layout()
            .placed_items()
            .values()
            .next()
            .unwrap()
            .item().idx(),
        1
    );
    assert!(!stats.start.completed);
    assert!(!stats.start.stopped);
    assert!(stats.start.recovery_exhausted);
    assert_eq!(stats.start.strip_expansions, 4);
    assert!(stats.start.recovery_rounds >= 6);
}

#[test]
fn legacy_path_keeps_fixed_order_and_complete_feasible_semantics() {
    let instance = two_rectangles();
    let built = builder(instance.clone(), 17).construct();
    assert_eq!(
        built
            .prob
            .layout()
            .placed_items()
            .iter()
            .next()
            .unwrap()
            .1
            .item().idx(),
        0
    );
    assert_eq!(
        built.prob.layout().placed_items().len(),
        instance.total_item_qty()
    );
    assert!(built.prob.layout().is_collision_free());
    assert!(
        built.structure_hint_stats().is_none(),
        "legacy construction must not allocate or report P1 instrumentation"
    );
}

#[test]
fn structure_stats_are_complete_consistent_and_serializable() {
    let instance = two_rectangles();
    let built = builder(instance.clone(), 23)
        .with_structure_hints(profile(), MotifCatalog::default(), 1)
        .construct();
    let stats = built
        .structure_hint_stats()
        .expect("the structure branch must retain one start audit record");

    assert_eq!(stats.start.requested_members, instance.total_item_qty());
    assert_eq!(stats.start.initially_placed_members, 0);
    assert!(stats.start.completed);
    assert!(!stats.start.stopped);
    assert_eq!(stats.start.committed_members, instance.total_item_qty());
    assert_eq!(
        stats.start.committed_units,
        stats.start.selection_order.len()
    );
    assert_eq!(
        stats.sources.single.committed_members,
        instance.total_item_qty()
    );
    assert_eq!(stats.sources.motif, Default::default());
    assert!(stats.sources.single.attempted >= stats.sources.single.exact_feasible);
    assert!(
        stats.sources.single.exact_feasible >= stats.sources.single.committed_units,
        "every committed single must have passed exact-authorized evaluation"
    );
    for (index, selection) in stats.start.selection_order.iter().enumerate() {
        assert_eq!(selection.selection_index, index);
        assert_eq!(selection.source, StructureSelectionSource::Single);
        assert_eq!(selection.motif_kind, None);
        assert_eq!(selection.member_count, 1);
    }
    assert!((stats.start.width - built.prob.strip_width()).abs() <= 1.0e-5);
    assert!((stats.start.density - built.prob.density()).abs() <= 1.0e-5);
    assert!(stats.start.elapsed_secs.is_finite());

    let json = serde_json::to_value(stats).expect("P1 stats must serialize for pipeline reports");
    assert_eq!(json["start"]["completed"], true);
    assert_eq!(json["sources"]["single"]["committed_members"], 2);
    assert!(json["proposals"]["random_sample_budget"].is_number());
}

#[test]
fn structure_stats_attribute_attempts_to_the_exact_motif_kind() {
    let instance = two_rectangles();
    let pair = PlacementMotif {
        kind: MotifKind::Pair(PairSearchMetadata {
            raster_cell: 1.0,
            offset_cells: (8, 0),
            boundary_contacts: 1,
            correlation_method: PairCorrelationMethod::Radix2Fft2d,
            fft_grid: (16, 16),
            correlation_peak: 1,
            exact_polygon_checked: true,
            requires_cde_recheck: true,
        }),
        members: vec![
            MotifMember {
                item: MotifItemRef {
                    item_id: 0,
                    copy_index: 0,
                },
                relative_pose: DTransformation::new(0.0, (0.0, 0.0)),
            },
            MotifMember {
                item: MotifItemRef {
                    item_id: 1,
                    copy_index: 0,
                },
                relative_pose: DTransformation::new(0.0, (8.0, 0.0)),
            },
        ],
        envelope: MotifEnvelope {
            bbox: Rect::try_new(0.0, 0.0, 10.0, 8.0).unwrap(),
            member_area: 68.0,
            fill_ratio: 0.85,
        },
        score: MotifScore {
            total: 1.0,
            ..MotifScore::default()
        },
    };
    let catalog = MotifCatalog {
        pairs: vec![pair],
        ..MotifCatalog::default()
    };
    let built = builder(instance, 29)
        .with_structure_hints(profile(), catalog, 1)
        .construct();
    let stats = built.structure_hint_stats().unwrap();

    assert!(stats.motif_kinds.pair.attempted > 0);
    assert_eq!(stats.motif_kinds.single.attempted, 0);
    assert_eq!(stats.motif_kinds.rectangle_block.attempted, 0);
    assert_eq!(stats.motif_kinds.repeated_band.attempted, 0);
    assert_eq!(
        stats.sources.motif.attempted,
        stats.motif_kinds.pair.attempted
    );
    assert_eq!(
        stats.sources.motif.exact_feasible,
        stats.motif_kinds.pair.exact_feasible
    );
}
