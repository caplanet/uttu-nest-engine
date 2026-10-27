use jagua_rs::{
    geometry::DTransformation,
    probs::spp::entities::{SPInstance, SPPlacement, SPProblem},
};
use rand::{SeedableRng, rngs::Xoshiro256PlusPlus};
use swarm_core::{
    config::DEFAULT_SWARM_CONFIG as C,
    optimizer::separator::Separator,
    original_overlap::OriginalOverlapGuard,
    util::{
        io::import_spp_instance_with_mm,
        terminator::{BasicTerminator, Terminator},
    },
};
fn fixture(points: serde_json::Value, centers: &[(f32, f32)], rot: f32) -> (SPInstance, Separator) {
    fixture_budget(points, centers, rot, 5.)
}
fn fixture_budget(
    points: serde_json::Value,
    centers: &[(f32, f32)],
    rot: f32,
    budget: f32,
) -> (SPInstance, Separator) {
    let ext=serde_json::from_value(serde_json::json!({"name":"original-guard-trial","strip_height":200,"items":centers.iter().enumerate().map(|(id,_)|serde_json::json!({"id":id,"demand":1,"allowed_orientations":[0,90],"shape":{"type":"simple_polygon","data":points}})).collect::<Vec<_>>()})).unwrap();
    let inst =
        import_spp_instance_with_mm(&ext, C.cde_config, Some(0.01), Some(-budget), None, None)
            .unwrap();
    let mut prob = SPProblem::new(inst.clone()).expect("strip forms a valid container");
    prob.change_strip_width(200.).expect("strip width must be positive");
    for (id, &center) in centers.iter().enumerate() {
        prob.place_item(SPPlacement {
            item_idx: id,
            d_transf: DTransformation::new(rot, center),
        });
    }
    let mut config = C.expl_cfg.separator_config;
    config.n_workers = 1;
    let sep = Separator::new(
        inst.clone(),
        prob,
        Xoshiro256PlusPlus::seed_from_u64(71),
        config,
    );
    (inst, sep)
}
#[test]
fn original_limit_follows_existing_overlap_value_for_each_instance() {
    for budget in [1., 3., 5.] {
        let (inst, mut sep) = fixture_budget(
            serde_json::json!([[0, 0], [40, 0], [40, 40], [0, 40]]),
            &[(25., 25.), (65. - budget + 0.02, 25.)],
            0.,
            budget,
        );
        assert!(swarm_core::original_overlap::validate(&inst, &sep.prob.save(), &[]).is_ok());
        let pk = sep.placement_key(1).unwrap();
        sep.move_item(pk, DTransformation::new(0., (65. - budget - 0.5, 25.)));
        assert!(swarm_core::original_overlap::validate(&inst, &sep.prob.save(), &[]).is_err());
    }
}
#[test]
fn original_guard_detects_excess_overlap_and_repairs_without_moving_frozen() {
    let (inst, mut sep) = fixture(
        serde_json::json!([[0, 0], [40, 0], [40, 40], [0, 40]]),
        &[(25., 25.), (59., 25.)],
        0.,
    );
    sep.frozen_mask[0] = true;
    let frozen = sep
        .prob
        .save()
        .layout_snapshot()
        .placed_items()
        .values()
        .find(|p| p.item().idx() == 0)
        .unwrap()
        .d_transf();
    assert!(sep.ct.get_total_loss() > 0.);
    let mut guard = OriginalOverlapGuard::new(&inst);
    let term = BasicTerminator::new();
    assert_eq!(
        guard
            .inspect(&sep.prob.save(), &sep.frozen_mask, &term)
            .unwrap()
            .len(),
        1
    );
    assert!(guard.repair(&mut sep, &term, false).unwrap());
    assert!(
        guard
            .inspect(&sep.prob.save(), &sep.frozen_mask, &term)
            .unwrap()
            .is_empty()
    );
    assert_eq!(
        sep.prob
            .layout()
            .placed_items()
            .values()
            .find(|p| p.item().idx() == 0)
            .unwrap()
            .d_transf(),
        frozen
    );
    assert!(sep.solution_is_certified(&sep.prob.save(), &sep.prob.save()));
}
#[test]
fn original_guard_keeps_frozen_exceptions_and_invalidates_pose_cache() {
    let (inst, mut sep) = fixture(
        serde_json::json!([[0, 0], [40, 0], [40, 40], [0, 40]]),
        &[(25., 25.), (64.01, 25.)],
        0.,
    );
    let mut guard = OriginalOverlapGuard::new(&inst);
    let term = BasicTerminator::new();
    assert!(
        guard
            .inspect(&sep.prob.save(), &[], &term)
            .unwrap()
            .is_empty()
    );
    let transforms = guard.stats.transformed;
    assert!(
        guard
            .inspect(&sep.prob.save(), &[], &term)
            .unwrap()
            .is_empty()
    );
    assert_eq!(guard.stats.transformed, transforms);
    let pk = sep.placement_key(1).unwrap();
    sep.move_item(pk, DTransformation::new(0., (60., 25.)));
    assert_eq!(
        guard.inspect(&sep.prob.save(), &[], &term).unwrap().len(),
        1
    );
    assert!(
        guard
            .inspect(&sep.prob.save(), &[true, true], &term)
            .unwrap()
            .is_empty()
    );
}
#[test]
fn cancellation_restores_incoming_state_and_does_not_claim_certification() {
    let (inst, mut sep) = fixture(
        serde_json::json!([[0, 0], [40, 0], [40, 40], [0, 40]]),
        &[(25., 25.), (60., 25.)],
        0.,
    );
    let before = sep.prob.save();
    let mut guard = OriginalOverlapGuard::new(&inst);
    let mut term = BasicTerminator::new();
    term.new_timeout(std::time::Duration::ZERO);
    assert!(guard.repair(&mut sep, &term, false).is_err());
    assert_eq!(before.strip_width(), sep.prob.strip_width());
    for (key, p) in before.layout_snapshot().placed_items() {
        assert_eq!(p.d_transf(), sep.prob.layout().placed_items()[key].d_transf());
    }
}

#[test]
fn original_pair_scan_checks_cancellation_within_one_simd_batch() {
    use std::sync::atomic::{AtomicUsize, Ordering};

    struct CheckBudget(AtomicUsize);
    impl Terminator for CheckBudget {
        fn kill(&self) -> bool {
            self.0.fetch_add(1, Ordering::Relaxed) >= 34
        }
        fn new_timeout(&mut self, _: std::time::Duration) {}
        fn timeout_at(&self) -> Option<jagua_rs::Instant> {
            None
        }
    }

    // Dense overlap is the worst case for a row-level cancellation check:
    // every remaining original outline requires an exact Clipper operation.
    let (inst, sep) = fixture(
        serde_json::json!([[0, 0], [40, 0], [40, 40], [0, 40]]),
        &[(50., 50.); 32],
        0.,
    );
    let mut guard = OriginalOverlapGuard::new(&inst);
    let term = CheckBudget(AtomicUsize::new(0));
    assert!(guard.inspect(&sep.prob.save(), &[], &term).is_err());
    assert!(
        guard.stats.exact_pairs <= 4,
        "one cancellation polling interval evaluated {} exact pairs",
        guard.stats.exact_pairs
    );
    // A cancelled scan is not a certificate; a later uninterrupted scan must
    // still discover every pair, including those absent from the partial cache.
    assert_eq!(
        guard
            .inspect(&sep.prob.save(), &[], &BasicTerminator::new())
            .unwrap()
            .len(),
        32 * 31 / 2
    );
}

#[test]
fn original_guard_handles_crossing_edges_and_rotation() {
    // A narrow plus with an intersection which cannot be detected by testing
    // only whether an original vertex lies inside the other polygon.
    let (inst, mut sep) = fixture(
        serde_json::json!([[0, 0], [40, 0], [40, 10], [0, 10]]),
        &[(50., 50.), (50., 50.)],
        0.,
    );
    let pk = sep.placement_key(1).unwrap();
    sep.move_item(
        pk,
        DTransformation::new(std::f32::consts::FRAC_PI_2, (50., 50.)),
    );
    let mut guard = OriginalOverlapGuard::new(&inst);
    assert_eq!(
        guard
            .inspect(&sep.prob.save(), &[], &BasicTerminator::new())
            .unwrap()
            .len(),
        1
    );
}

#[test]
fn parallel_original_pairs_match_serial_cold_and_warm_inspection() {
    let points: Vec<_> = (0..96)
        .map(|i| {
            let angle = i as f64 * std::f64::consts::TAU / 96.;
            [20. + 20. * angle.cos(), 20. + 20. * angle.sin()]
        })
        .collect();
    let (inst, sep) = fixture(serde_json::json!(points), &[(50., 50.); 64], 0.);
    let sol = sep.prob.save();
    let run = |threads| {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap();
        pool.install(|| {
            let mut guard = OriginalOverlapGuard::new(&inst);
            let normalize = |v: Vec<swarm_core::original_overlap::Violation>| {
                let mut pairs: Vec<_> = v
                    .into_iter()
                    .map(|v| {
                        let a = sol.layout_snapshot().placed_items()[v.a].item().idx();
                        let b = sol.layout_snapshot().placed_items()[v.b].item().idx();
                        (a.min(b), a.max(b), v.area.to_bits())
                    })
                    .collect();
                pairs.sort_unstable();
                pairs
            };
            let first = normalize(guard.inspect(&sol, &[], &BasicTerminator::new()).unwrap());
            let exact = guard.stats.exact_pairs;
            assert_eq!(
                first,
                normalize(guard.inspect(&sol, &[], &BasicTerminator::new()).unwrap())
            );
            assert_eq!(guard.stats.exact_pairs, exact);
            (first, guard.stats.parallel_pairs)
        })
    };
    let serial = run(1);
    let parallel = run(4);
    assert_eq!(serial.0.len(), 64 * 63 / 2);
    assert_eq!(serial.0, parallel.0);
    assert_eq!(serial.1, 0);
    assert!(parallel.1 > 0);
}
#[test]
fn rigid_group_correction_preserves_relative_poses() {
    use swarm_core::optimizer::place_groups::{PlaceGroupMember, PlaceGroupSpec, PlaceGroups};
    let (inst, mut sep) = fixture(
        serde_json::json!([[0, 0], [40, 0], [40, 40], [0, 40]]),
        &[(25., 25.), (59., 25.), (105., 25.)],
        0.,
    );
    let members = [1, 2]
        .map(|id| PlaceGroupMember {
            item_id: id,
            initial_transform: sep.prob.layout().placed_items()[sep.placement_key(id).unwrap()]
                .d_transf(),
        })
        .to_vec();
    let groups = PlaceGroups::new(&inst, vec![PlaceGroupSpec { id: 17, members }]).unwrap();
    sep.set_place_groups(groups.clone()).unwrap();
    sep.freeze_items(&[0]);
    let mut guard = OriginalOverlapGuard::new(&inst);
    assert!(
        guard
            .repair(&mut sep, &BasicTerminator::new(), false)
            .unwrap()
    );
    assert!(groups.solution_preserves_relative_poses(&sep.prob.save()));
    assert!(
        guard
            .inspect(&sep.prob.save(), &sep.frozen_mask, &BasicTerminator::new())
            .unwrap()
            .is_empty()
    );
}

#[test]
fn width_recovery_reacquires_keys_after_transactional_moves() {
    let (inst, mut sep) = fixture(
        serde_json::json!([[0, 0], [40, 0], [40, 40], [0, 40]]),
        &[(20., 25.), (54., 25.)],
        0.,
    );
    // No fixed-width move can clear the pair: the first item is frozen and
    // the second is already at the right edge. Exercise the growth fallback.
    sep.freeze_items(&[0]);
    sep.change_strip_width(70., Some(f32::INFINITY));
    let mut term = BasicTerminator::new();
    term.new_timeout(std::time::Duration::from_secs(2));
    let mut guard = OriginalOverlapGuard::new(&inst);
    assert!(guard.repair(&mut sep, &term, true).unwrap());
    assert!(sep.prob.strip_width() > 70.);
    assert!(
        guard
            .inspect(&sep.prob.save(), &[], &BasicTerminator::new())
            .unwrap()
            .is_empty()
    );
    assert!(sep.solution_is_certified(&sep.prob.save(), &sep.prob.save()));
}
