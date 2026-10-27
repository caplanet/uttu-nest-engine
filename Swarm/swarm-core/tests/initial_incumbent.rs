use jagua_rs::{
    geometry::DTransformation,
    probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution},
};
use swarm_core::{
    config::DEFAULT_SWARM_CONFIG,
    initial::incumbent::candidate_is_improvement,
    optimizer::place_groups::{PlaceGroupMember, PlaceGroupSpec, PlaceGroups},
    util::io::import_spp_instance,
};

fn scene() -> (SPInstance, SPSolution) {
    let ext=serde_json::from_value(serde_json::json!({"name":"incumbent","strip_height":20,
        "items":(0..3).map(|id|serde_json::json!({"id":id,"demand":1,"allowed_orientations":[0],
            "shape":{"type":"simple_polygon","data":[[0,0],[4,0],[4,4],[0,4]]}})).collect::<Vec<_>>()})).unwrap();
    let instance =
        import_spp_instance(&ext, DEFAULT_SWARM_CONFIG.cde_config, None, None, None).unwrap();
    let mut prob = SPProblem::new(instance.clone()).expect("strip forms a valid container");
    prob.change_strip_width(32.).expect("strip width must be positive");
    for id in 0..3 {
        prob.place_item(SPPlacement {
            item_idx: id,
            d_transf: DTransformation::new(0., (4. + id as f32 * 8., 4.)),
        });
    }
    (instance, prob.save())
}

#[test]
fn competition_rejects_wider_partial_and_colliding_candidates() {
    let (instance, before) = scene();
    let groups = PlaceGroups::default();
    let mut p = SPProblem::new(instance.clone()).expect("strip forms a valid container");
    p.restore(&before);
    p.change_strip_width(30.).expect("strip width must be positive");
    assert!(candidate_is_improvement(
        &instance,
        &p.save(),
        &before,
        &[],
        &groups
    ));
    p.change_strip_width(34.).expect("strip width must be positive");
    assert!(!candidate_is_improvement(
        &instance,
        &p.save(),
        &before,
        &[],
        &groups
    ));
    p.change_strip_width(30.).expect("strip width must be positive");
    let key = p.layout().placed_items().keys().next().unwrap();
    p.remove_item(key);
    assert!(!candidate_is_improvement(
        &instance,
        &p.save(),
        &before,
        &[],
        &groups
    ));
    p.place_item(SPPlacement {
        item_idx: 0,
        d_transf: DTransformation::new(0., (12., 4.)),
    });
    assert!(!candidate_is_improvement(
        &instance,
        &p.save(),
        &before,
        &[],
        &groups
    ));
}

#[test]
fn competition_rejects_changed_frozen_pose_and_broken_group() {
    let (instance, before) = scene();
    let members = before
        .layout_snapshot()
        .placed_items()
        .values()
        .filter(|p| p.item().idx() < 2)
        .map(|p| PlaceGroupMember {
            item_id: p.item().idx(),
            initial_transform: p.d_transf(),
        })
        .collect();
    let groups = PlaceGroups::new(&instance, vec![PlaceGroupSpec { id: 7, members }]).unwrap();
    let mut after = before.clone();
    after.strip_mut().width = 30.;
    let pi = after
        .layout_snapshot_mut()
        .placed_items_mut()
        .values_mut()
        .find(|p| p.item().idx() == 0)
        .unwrap();
    pi.set_d_transf(DTransformation::new(0., (5., 4.)));
    assert!(!candidate_is_improvement(
        &instance,
        &after,
        &before,
        &[0],
        &PlaceGroups::default()
    ));
    assert!(!candidate_is_improvement(
        &instance,
        &after,
        &before,
        &[],
        &groups
    ));
}

#[test]
fn seeded_initial_improves_movable_units_and_keeps_locks_groups_and_stop_result() {
    use rand::{SeedableRng, rngs::Xoshiro256PlusPlus};
    use swarm_core::{
        initial::incumbent::improve_seeded_units,
        optimizer::separator::Separator,
        util::terminator::{BasicTerminator, Terminator},
    };
    let (instance, before) = scene();
    let members = before
        .layout_snapshot()
        .placed_items()
        .values()
        .filter(|p| p.item().idx() > 0)
        .map(|p| PlaceGroupMember {
            item_id: p.item().idx(),
            initial_transform: p.d_transf(),
        })
        .collect();
    let groups = PlaceGroups::new(&instance, vec![PlaceGroupSpec { id: 9, members }]).unwrap();
    let mut p = SPProblem::new(instance.clone()).expect("strip forms a valid container");
    p.restore(&before);
    let mut cfg = DEFAULT_SWARM_CONFIG.expl_cfg.separator_config;
    cfg.n_workers = 1;
    let mut sep = Separator::new(
        instance.clone(),
        p,
        Xoshiro256PlusPlus::seed_from_u64(5),
        cfg,
    );
    sep.freeze_items(&[0]);
    sep.set_place_groups(groups.clone()).unwrap();
    let mut term = BasicTerminator::new();
    term.new_timeout(std::time::Duration::from_secs(2));
    let result = improve_seeded_units(
        &mut sep,
        &before,
        &swarm_core::initial::SlideCompactCfg::default(),
        2,
        &term,
        None,
    )
    .unwrap();
    assert!(result.strip_width() < before.strip_width());
    assert!(groups.solution_preserves_relative_poses(&result));
    assert!(swarm_core::util::assertions::frozen_poses_preserved(
        &before,
        &result,
        &[true, false, false]
    ));
    assert!(sep.solution_is_certified(&result, &before));
    term.new_timeout(std::time::Duration::ZERO);
    let stopped = improve_seeded_units(
        &mut sep,
        &result,
        &swarm_core::initial::SlideCompactCfg::default(),
        2,
        &term,
        None,
    )
    .unwrap();
    assert_eq!(stopped.strip_width(), result.strip_width());
    for (a, b) in stopped
        .layout_snapshot()
        .placed_items()
        .values()
        .zip(result.layout_snapshot().placed_items().values())
    {
        assert_eq!(a.d_transf(), b.d_transf());
    }
}

#[test]
fn seeded_n06_zero_budget_returns_the_complete_incumbent() {
    let (instance, before) = scene();
    let (result, stats, _, _) = swarm_core::initial::shrink_compact_seeded_ctl(
        &instance,
        &swarm_core::initial::AttachCfg::n02c(),
        &swarm_core::initial::EscapeCfg::default(),
        &swarm_core::initial::SlideCompactCfg::default(),
        2,
        1,
        0.02,
        0.001,
        20,
        32,
        1.1,
        0,
        42,
        None,
        None,
        swarm_core::initial::incumbent::InitialSeed {
            solution: &before,
            frozen_ids: &[],
            groups: &PlaceGroups::default(),
        },
    )
    .unwrap();
    assert_eq!(result.strip_width(), before.strip_width());
    assert_eq!(stats.placed, 3);
    for (a, b) in result
        .layout_snapshot()
        .placed_items()
        .values()
        .zip(before.layout_snapshot().placed_items().values())
    {
        assert_eq!(a.d_transf(), b.d_transf());
    }
}

#[test]
fn seeded_n06_stops_at_the_last_certified_callback() {
    use std::sync::atomic::{AtomicBool, Ordering};
    let (instance, before) = scene();
    let stop = AtomicBool::new(false);
    let mut published = None;
    let mut callback = |sol: &SPSolution| {
        assert!(
            swarm_core::structure::pipeline::solution_validation_error(&instance, sol).is_none()
        );
        published = Some(sol.clone());
        stop.store(true, Ordering::Relaxed);
    };
    let (result, _, _, _) = swarm_core::initial::shrink_compact_seeded_ctl(
        &instance,
        &swarm_core::initial::AttachCfg::n02c(),
        &swarm_core::initial::EscapeCfg::default(),
        &swarm_core::initial::SlideCompactCfg::default(),
        2,
        1,
        0.02,
        0.001,
        20,
        32,
        1.1,
        2000,
        42,
        Some(&stop),
        Some(&mut callback),
        swarm_core::initial::incumbent::InitialSeed {
            solution: &before,
            frozen_ids: &[],
            groups: &PlaceGroups::default(),
        },
    )
    .unwrap();
    let published = published.expect("seed has obvious removable slack");
    assert!(result.strip_width() < before.strip_width());
    assert_eq!(result.strip_width(), published.strip_width());
    for (a, b) in result
        .layout_snapshot()
        .placed_items()
        .values()
        .zip(published.layout_snapshot().placed_items().values())
    {
        assert_eq!(a.d_transf(), b.d_transf());
    }
}

#[test]
fn certified_shrink_is_published_before_optional_compaction() {
    use std::sync::atomic::{AtomicBool, Ordering};
    let (instance, before) = scene();
    let stop = AtomicBool::new(false);
    let mut reports = Vec::new();
    let mut callback = |sol: &SPSolution| {
        reports.push(sol.clone());
        if reports.len() == 2 {
            stop.store(true, Ordering::Relaxed);
        }
    };
    let compact = swarm_core::initial::SlideCompactCfg {
        passes_max: 0,
        reseat_top: 0,
        ..Default::default()
    };
    let (result, _, report, compacts) = swarm_core::initial::shrink_compact_seeded_ctl(
        &instance,
        &swarm_core::initial::AttachCfg::n02c(),
        &swarm_core::initial::EscapeCfg::default(),
        &compact,
        1,
        0,
        0.02,
        0.001,
        100,
        64,
        1.1,
        5000,
        42,
        Some(&stop),
        Some(&mut callback),
        swarm_core::initial::incumbent::InitialSeed {
            solution: &before,
            frozen_ids: &[],
            groups: &PlaceGroups::default(),
        },
    )
    .unwrap();
    assert_eq!(
        reports.len(),
        2,
        "fixture must reach a certified shrink after initial refit"
    );
    assert_eq!(report.ok, 1);
    assert_eq!(
        compacts.len(),
        1,
        "Stop from the shrink callback must prevent optional compaction"
    );
    assert_eq!(result.strip_width(), reports.last().unwrap().strip_width());
}
