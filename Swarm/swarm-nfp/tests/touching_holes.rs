//! Regions returned by this crate may contain holes that *touch* their shell
//! or a sibling hole at a vertex: Clipper keeps such point contacts and
//! `feasible_region` / `subtract_obstacle` canonicalize them as shell + hole.
//! Input validation must accept exactly that topology again, otherwise a
//! placement loop that feeds one result back into `subtract_obstacle`,
//! `prepare_region` or `locate` fails with `HoleOutsideShell` /
//! `InvalidHoleRelation` on a perfectly valid region.

mod common;

use common::{area, assert_near, polygon, rect};
use swarm_nfp::{Clearance, Location, NfpEngine, PlacedObstacle, Point, PolygonSet, Translation};

const OUTER: [(f64, f64); 4] = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)];

/// A diamond hole whose bottom vertex lies on the shell's bottom edge.
fn hole_touching_shell_edge() -> PolygonSet {
    polygon(
        &OUTER,
        &[&[(10.0, 0.0), (14.0, 5.0), (10.0, 10.0), (6.0, 5.0)]],
    )
}

/// A triangular hole sharing the shell's corner vertex.
fn hole_sharing_shell_vertex() -> PolygonSet {
    polygon(&OUTER, &[&[(0.0, 0.0), (6.0, 2.0), (2.0, 6.0)]])
}

/// Two holes meeting at one vertex, neither touching the shell.
fn holes_touching_each_other() -> PolygonSet {
    polygon(
        &OUTER,
        &[
            &[(4.0, 4.0), (9.0, 4.0), (9.0, 9.0), (4.0, 9.0)],
            &[(9.0, 9.0), (14.0, 9.0), (14.0, 14.0), (9.0, 14.0)],
        ],
    )
}

fn hole_area(region: &PolygonSet) -> f64 {
    region.components[0]
        .holes
        .iter()
        .map(|hole| common::signed_ring_area(hole).abs())
        .sum()
}

/// `(name, region, a point strictly inside one of its holes)`.
fn fixtures() -> Vec<(&'static str, PolygonSet, Point)> {
    vec![
        (
            "hole touching shell edge",
            hole_touching_shell_edge(),
            Point::new(10.0, 5.0),
        ),
        (
            "hole sharing shell vertex",
            hole_sharing_shell_vertex(),
            Point::new(2.0, 2.0),
        ),
        (
            "holes touching each other",
            holes_touching_each_other(),
            Point::new(6.0, 6.0),
        ),
    ]
}

#[test]
fn touching_holes_round_trip_through_prepare_region_and_locate() {
    let engine = NfpEngine::new();
    for (name, region, in_hole) in fixtures() {
        let prepared = engine
            .prepare_region(&region)
            .unwrap_or_else(|error| panic!("{name}: prepare_region failed: {error}"));
        assert!(!prepared.is_empty(), "{name}: region must not be empty");

        for (point, expected) in [
            (Point::new(17.0, 17.0), Location::Inside),
            (in_hole, Location::Outside),
            (Point::new(0.0, 0.0), Location::Boundary),
        ] {
            let via_prepared = engine.locate_prepared(&prepared, point).unwrap();
            let via_locate = engine
                .locate(&region, point)
                .unwrap_or_else(|error| panic!("{name}: locate failed: {error}"));
            assert_eq!(via_prepared, expected, "{name}: locate_prepared at {point:?}");
            assert_eq!(via_locate, expected, "{name}: locate at {point:?}");
        }
    }
}

#[test]
fn touching_holes_round_trip_through_subtract_obstacle() {
    let engine = NfpEngine::new();
    let moving = engine.prepare(&rect(0.0, 0.0, 1.0, 1.0)).unwrap();
    let obstacle_shape = engine.prepare(&rect(0.0, 0.0, 1.0, 1.0)).unwrap();
    let obstacle = PlacedObstacle {
        shape: &obstacle_shape,
        translation: Translation::new(17.0, 17.0),
    };
    for (name, region, in_hole) in fixtures() {
        let result = engine
            .subtract_obstacle(&region, &obstacle, &moving, &Clearance::None)
            .unwrap_or_else(|error| panic!("{name}: subtract_obstacle failed: {error}"));
        // NFP(1x1, 1x1) is the 2x2 square centred on the obstacle anchor, so
        // exactly 4 mm^2 of interior material becomes one more hole; the
        // touching hole(s) survive unchanged.
        assert_near(area(&result), area(&region) - 4.0);
        assert_eq!(result.components.len(), 1, "{name}: one component expected");
        assert_eq!(
            result.components[0].holes.len(),
            region.components[0].holes.len() + 1,
            "{name}: the subtracted square must become an extra hole"
        );
        assert_near(hole_area(&result), hole_area(&region) + 4.0);
        assert_eq!(engine.locate(&result, in_hole).unwrap(), Location::Outside);
        assert_eq!(
            engine.locate(&result, Point::new(17.0, 17.0)).unwrap(),
            Location::Outside
        );

        // The result itself is such a region: it must round-trip once more.
        let again = engine
            .subtract_obstacle(&result, &obstacle, &moving, &Clearance::None)
            .unwrap_or_else(|error| panic!("{name}: second subtract_obstacle failed: {error}"));
        assert_near(area(&again), area(&result));
        engine
            .prepare_region(&again)
            .unwrap_or_else(|error| panic!("{name}: prepare_region of result failed: {error}"));
    }
}

#[test]
fn touching_holes_round_trip_through_translate_region() {
    let engine = NfpEngine::new();
    for (name, region, _) in fixtures() {
        let translated = engine
            .translate_region(&region, Translation::new(3.0, -2.0))
            .unwrap_or_else(|error| panic!("{name}: translate_region failed: {error}"));
        assert_near(area(&translated), area(&region));
        assert_eq!(translated.components[0].holes.len(), region.components[0].holes.len());
    }
}

#[test]
fn crossing_holes_are_still_rejected() {
    let engine = NfpEngine::new();
    // Hole edge properly crosses the shell edge.
    let crossing_shell = polygon(&OUTER, &[&[(18.0, 5.0), (22.0, 5.0), (22.0, 9.0), (18.0, 9.0)]]);
    assert!(matches!(
        engine.prepare_region(&crossing_shell),
        Err(swarm_nfp::NfpError::HoleOutsideShell { .. })
    ));
    // Hole entirely outside its shell, merely touching at a corner.
    let outside_touching = polygon(&OUTER, &[&[(20.0, 20.0), (24.0, 20.0), (24.0, 24.0)]]);
    assert!(matches!(
        engine.prepare_region(&outside_touching),
        Err(swarm_nfp::NfpError::HoleOutsideShell { .. })
    ));
    // Two holes that overlap with positive area.
    let overlapping = polygon(
        &OUTER,
        &[
            &[(4.0, 4.0), (10.0, 4.0), (10.0, 10.0), (4.0, 10.0)],
            &[(8.0, 8.0), (14.0, 8.0), (14.0, 14.0), (8.0, 14.0)],
        ],
    );
    assert!(matches!(
        engine.prepare_region(&overlapping),
        Err(swarm_nfp::NfpError::InvalidHoleRelation { .. })
    ));
}
