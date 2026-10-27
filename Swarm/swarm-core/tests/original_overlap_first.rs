use jagua_rs::{
    geometry::DTransformation,
    probs::spp::entities::{SPPlacement, SPProblem},
};
use swarm_core::{
    config::DEFAULT_SWARM_CONFIG,
    original_overlap::OriginalOverlapGuard,
    util::{
        io::import_spp_instance,
        terminator::{BasicTerminator, Terminator},
    },
};

fn scene(overlapping: bool, count: usize) -> SPProblem {
    let ext = serde_json::from_value(serde_json::json!({"name":"first-violation","strip_height":100,
        "items":(0..count).map(|id| serde_json::json!({"id":id,"demand":1,"allowed_orientations":[0],
            "shape":{"type":"simple_polygon","data":[[0,0],[40,0],[40,40],[0,40]]}})).collect::<Vec<_>>()})).unwrap();
    let inst = import_spp_instance(
        &ext,
        DEFAULT_SWARM_CONFIG.cde_config,
        None,
        Some(-2.),
        None,
    )
    .unwrap();
    let mut p = SPProblem::new(inst).expect("strip forms a valid container");
    p.change_strip_width((count * 50 + 100) as f32).expect("strip width must be positive");
    for id in 0..count {
        p.place_item(SPPlacement {
            item_idx: id,
            d_transf: DTransformation::new(
                0.,
                (
                    if overlapping {
                        25.
                    } else {
                        25. + id as f32 * 50.
                    },
                    25.,
                ),
            ),
        });
    }
    p
}

#[test]
fn first_violation_matches_full_order_and_skips_remaining_exact_pairs() {
    let p = scene(true, 70);
    let sol = p.save();
    let term = BasicTerminator::new();
    let mut first = OriginalOverlapGuard::new(&p.instance());
    let mut full = OriginalOverlapGuard::new(&p.instance());
    let violation = first.first_violation(&sol, &[], &term).unwrap().unwrap();
    let all = full.inspect(&sol, &[], &term).unwrap();
    assert_eq!(
        (violation.a, violation.b, violation.area),
        (all[0].a, all[0].b, all[0].area)
    );
    assert_eq!(first.stats.exact_pairs, 1);
    assert_eq!(first.stats.parallel_pairs, 0);
    assert!(full.stats.exact_pairs > first.stats.exact_pairs);
    let checks = first.stats.exact_pairs;
    assert!(first.first_violation(&sol, &[], &term).unwrap().is_some());
    assert_eq!(first.stats.exact_pairs, checks);
    assert!(first.stats.cached_pairs > 0);
}

#[test]
fn success_and_frozen_exceptions_are_identical_in_both_modes() {
    for overlapping in [false, true] {
        let p = scene(overlapping, 4);
        let sol = p.save();
        let term = BasicTerminator::new();
        for frozen in [vec![], vec![true; 4], vec![true, false, true, true]] {
            let mut guard = OriginalOverlapGuard::new(&p.instance());
            let full = guard.inspect(&sol, &frozen, &term).unwrap();
            let first = guard.first_violation(&sol, &frozen, &term).unwrap();
            assert_eq!(first.is_some(), !full.is_empty());
        }
    }
}

#[test]
fn first_mode_never_turns_cancellation_or_invalid_coordinates_into_success() {
    struct Cancel;
    impl Terminator for Cancel {
        fn kill(&self) -> bool {
            true
        }
        fn new_timeout(&mut self, _: std::time::Duration) {}
        fn timeout_at(&self) -> Option<jagua_rs::Instant> {
            None
        }
    }
    let p = scene(false, 3);
    let mut sol = p.save();
    let mut guard = OriginalOverlapGuard::new(&p.instance());
    assert!(guard.first_violation(&sol, &[], &Cancel).is_err());
    let first = sol
        .layout_snapshot_mut()
        .placed_items_mut()
        .values_mut()
        .next()
        .unwrap();
    // Jagua rejects NaN at construction, but infinity reaches the guard.
    first.set_d_transf(DTransformation::new(0., (f32::INFINITY, 25.)));
    assert!(
        guard
            .first_violation(&sol, &[], &BasicTerminator::new())
            .is_err()
    );
}
