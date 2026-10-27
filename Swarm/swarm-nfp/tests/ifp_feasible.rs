mod common;

use common::*;
use swarm_nfp::{
    Clearance, FeasibleRequest, Location, NfpEngine, NfpError, PlacedObstacle, Point, Translation,
};

#[test]
fn rectangular_inner_fit_is_the_analytic_anchor_translation_rectangle() {
    let engine = NfpEngine::new();
    let container = engine.prepare(&rect(0.0, 0.0, 10.0, 8.0)).unwrap();
    let moving = engine.prepare(&rect(0.0, 0.0, 2.0, 3.0)).unwrap();
    let ifp = engine.inner_fit(&container, &moving).unwrap();

    assert_eq!(ifp.components.len(), 1);
    assert_eq!(ifp.ring_count(), 1);
    assert_bounds(&ifp, (0.0, 0.0, 8.0, 5.0));
    assert_near(area(&ifp), 40.0);
    assert_location(&engine, &ifp, (4.0, 2.0), Location::Inside);
    assert_location(&engine, &ifp, (0.0, 0.0), Location::Boundary);
    assert_location(&engine, &ifp, (8.0, 5.0), Location::Boundary);
    assert_location(&engine, &ifp, (8.001, 5.0), Location::Outside);
}

#[test]
fn too_large_moving_shape_has_an_empty_inner_fit() {
    let engine = NfpEngine::new();
    let container = engine.prepare(&rect(0.0, 0.0, 10.0, 8.0)).unwrap();
    let too_wide = engine.prepare(&rect(0.0, 0.0, 11.0, 1.0)).unwrap();
    let too_tall = engine.prepare(&rect(0.0, 0.0, 1.0, 9.0)).unwrap();

    assert!(engine.inner_fit(&container, &too_wide).unwrap().is_empty());
    assert!(engine.inner_fit(&container, &too_tall).unwrap().is_empty());
}

#[test]
fn zero_area_exact_fit_point_or_line_returns_a_typed_model_limit() {
    let engine = NfpEngine::new();
    let container = engine.prepare(&rect(0.0, 0.0, 10.0, 8.0)).unwrap();
    let exact_both = engine.prepare(&rect(0.0, 0.0, 10.0, 8.0)).unwrap();
    let exact_width = engine.prepare(&rect(0.0, 0.0, 10.0, 2.0)).unwrap();
    let exact_height = engine.prepare(&rect(0.0, 0.0, 2.0, 8.0)).unwrap();

    // PolygonSet represents filled 2-D regions only. A mathematically valid
    // isolated anchor point or line is reported explicitly, never conflated
    // with an ordinary no-fit result.
    for exact in [&exact_both, &exact_width, &exact_height] {
        assert!(matches!(
            engine.inner_fit(&container, exact),
            Err(NfpError::DegenerateConfigurationSpace {
                operation: "inner fit"
            })
        ));
    }
}

#[test]
fn concave_container_inner_fit_keeps_its_reentrant_exclusion() {
    let engine = NfpEngine::new();
    let container = engine.prepare(&l_shape(5.0, 5.0, 2.0)).unwrap();
    let moving = engine.prepare(&rect(0.0, 0.0, 1.0, 1.0)).unwrap();
    let ifp = engine.inner_fit(&container, &moving).unwrap();

    assert_eq!(ifp.components.len(), 1);
    assert_eq!(ifp.ring_count(), 1);
    assert_bounds(&ifp, (0.0, 0.0, 4.0, 4.0));
    assert_near(area(&ifp), 7.0);
    assert_location(&engine, &ifp, (0.5, 3.0), Location::Inside);
    assert_location(&engine, &ifp, (3.0, 0.5), Location::Inside);
    assert_location(&engine, &ifp, (2.0, 2.0), Location::Outside);
}

#[test]
fn container_hole_becomes_a_forbidden_hole_in_inner_fit() {
    let engine = NfpEngine::new();
    let container = engine
        .prepare(&donut((0.0, 0.0, 10.0, 10.0), (4.0, 4.0, 6.0, 6.0)))
        .unwrap();
    let moving = engine.prepare(&rect(0.0, 0.0, 1.0, 1.0)).unwrap();
    let ifp = engine.inner_fit(&container, &moving).unwrap();

    assert_eq!(ifp.components.len(), 1);
    assert_eq!(ifp.components[0].holes.len(), 1);
    assert_bounds(&ifp, (0.0, 0.0, 9.0, 9.0));
    assert_near(area(&ifp), 72.0);
    assert_location(&engine, &ifp, (1.0, 1.0), Location::Inside);
    assert_location(&engine, &ifp, (4.5, 4.5), Location::Outside);
    assert_location(&engine, &ifp, (3.0, 4.5), Location::Boundary);
    assert_location(&engine, &ifp, (2.999, 4.5), Location::Inside);
}

#[test]
fn inner_fit_membership_matches_independent_unit_cell_containment() {
    let engine = NfpEngine::new();
    let container_input = donut((0.0, 0.0, 10.0, 9.0), (3.0, 3.0, 7.0, 6.0));
    let moving_input = l_shape(2.0, 2.0, 1.0);
    let container = engine.prepare(&container_input).unwrap();
    let moving = engine.prepare(&moving_input).unwrap();
    let ifp = engine.inner_fit(&container, &moving).unwrap();
    let container_cells = CellShape::from_polygon_set(&container_input);
    let moving_cells = CellShape::from_polygon_set(&moving_input);

    for ty in -3..=11 {
        for tx in -3..=12 {
            let expected = contained(&container_cells, &moving_cells, tx, ty);
            let actual = engine
                .locate(&ifp, Point::new(tx as f64, ty as f64))
                .unwrap();
            assert_eq!(
                actual != Location::Outside,
                expected,
                "containment mismatch at translation ({tx}, {ty}), got {actual:?}"
            );
        }
    }
}

#[test]
fn feasible_region_without_obstacles_is_exactly_the_inner_fit() {
    let engine = NfpEngine::new();
    let container = engine.prepare(&rect(0.0, 0.0, 12.0, 6.0)).unwrap();
    let moving = engine.prepare(&rect(0.0, 0.0, 1.0, 1.0)).unwrap();
    let ifp = engine.inner_fit(&container, &moving).unwrap();
    let feasible = engine
        .feasible_region(FeasibleRequest {
            container: &container,
            moving: &moving,
            obstacles: &[],
            clearance: Clearance::None,
        })
        .unwrap();

    assert_eq!(
        serde_json::to_string(&feasible).unwrap(),
        serde_json::to_string(&ifp).unwrap()
    );
}

#[test]
fn clearance_is_applied_to_both_container_and_obstacle_boundaries() {
    let engine = NfpEngine::new();
    let container = engine.prepare(&rect(0.0, 0.0, 10.0, 10.0)).unwrap();
    let moving = engine.prepare(&rect(0.0, 0.0, 1.0, 1.0)).unwrap();
    let obstacle = engine.prepare(&rect(0.0, 0.0, 1.0, 1.0)).unwrap();
    let kernel =
        Clearance::ConvexKernel(ring(&[(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)]));

    let ifp = engine
        .inner_fit_with_clearance(&container, &moving, &kernel)
        .unwrap();
    assert_bounds(&ifp, (0.5, 0.5, 8.5, 8.5));

    let obstacles = [PlacedObstacle {
        shape: &obstacle,
        translation: Translation::new(5.0, 5.0),
    }];
    let feasible = engine
        .feasible_region(FeasibleRequest {
            container: &container,
            moving: &moving,
            obstacles: &obstacles,
            clearance: kernel,
        })
        .unwrap();
    assert_location(&engine, &feasible, (0.5, 0.5), Location::Boundary);
    assert_location(&engine, &feasible, (3.499, 5.0), Location::Inside);
    assert_location(&engine, &feasible, (3.5, 5.0), Location::Boundary);
    assert_location(&engine, &feasible, (3.501, 5.0), Location::Outside);
}

#[test]
fn overlapping_obstacle_nfps_are_unioned_before_feasible_subtraction() {
    let engine = NfpEngine::new();
    let container = engine.prepare(&rect(0.0, 0.0, 12.0, 6.0)).unwrap();
    let moving = engine.prepare(&rect(0.0, 0.0, 1.0, 1.0)).unwrap();
    let obstacle = engine.prepare(&rect(0.0, 0.0, 2.0, 2.0)).unwrap();
    let obstacles = [
        PlacedObstacle {
            shape: &obstacle,
            translation: Translation::new(3.0, 2.0),
        },
        PlacedObstacle {
            shape: &obstacle,
            translation: Translation::new(4.0, 2.0),
        },
    ];
    let feasible = engine
        .feasible_region(FeasibleRequest {
            container: &container,
            moving: &moving,
            obstacles: &obstacles,
            clearance: Clearance::None,
        })
        .unwrap();

    assert_eq!(feasible.components.len(), 1);
    assert_eq!(feasible.components[0].holes.len(), 1);
    assert_bounds(&feasible, (0.0, 0.0, 11.0, 5.0));
    assert_near(area(&feasible), 43.0);
    assert_location(&engine, &feasible, (0.5, 0.5), Location::Inside);
    assert_location(&engine, &feasible, (2.5, 2.0), Location::Outside);
    assert_location(&engine, &feasible, (5.5, 2.0), Location::Outside);
    assert_location(&engine, &feasible, (2.0, 2.0), Location::Boundary);
}

#[test]
fn a_forbidden_barrier_splits_the_feasible_region_into_components() {
    let engine = NfpEngine::new();
    let container = engine.prepare(&rect(0.0, 0.0, 12.0, 6.0)).unwrap();
    let moving = engine.prepare(&rect(0.0, 0.0, 1.0, 1.0)).unwrap();
    let barrier = engine.prepare(&rect(0.0, 0.0, 1.0, 6.0)).unwrap();
    let obstacles = [PlacedObstacle {
        shape: &barrier,
        translation: Translation::new(5.0, 0.0),
    }];
    let feasible = engine
        .feasible_region(FeasibleRequest {
            container: &container,
            moving: &moving,
            obstacles: &obstacles,
            clearance: Clearance::None,
        })
        .unwrap();

    assert_eq!(feasible.components.len(), 2);
    assert_eq!(feasible.ring_count(), 2);
    assert_bounds(&feasible, (0.0, 0.0, 11.0, 5.0));
    assert_near(area(&feasible), 45.0);
    assert_location(&engine, &feasible, (2.0, 2.0), Location::Inside);
    assert_location(&engine, &feasible, (5.0, 2.0), Location::Outside);
    assert_location(&engine, &feasible, (8.0, 2.0), Location::Inside);
    assert_location(&engine, &feasible, (4.0, 2.0), Location::Boundary);
}

#[test]
fn an_obstacle_covering_the_container_yields_an_empty_feasible_region() {
    let engine = NfpEngine::new();
    let container = engine.prepare(&rect(0.0, 0.0, 12.0, 6.0)).unwrap();
    let moving = engine.prepare(&rect(0.0, 0.0, 1.0, 1.0)).unwrap();
    let covering = engine.prepare(&rect(0.0, 0.0, 12.0, 6.0)).unwrap();
    let obstacles = [PlacedObstacle {
        shape: &covering,
        translation: Translation::ZERO,
    }];
    let feasible = engine
        .feasible_region(FeasibleRequest {
            container: &container,
            moving: &moving,
            obstacles: &obstacles,
            clearance: Clearance::None,
        })
        .unwrap();

    assert!(feasible.is_empty());
}

#[test]
fn contact_only_line_residual_is_empty_by_polygon_set_contract() {
    let engine = NfpEngine::new();
    let container_input = rect(0.0, 0.0, 12.0, 10.0);
    let moving_input = l_shape(2.0, 2.0, 1.0);
    let obstacle_input = rect(0.0, 0.0, 1.0, 3.0);
    let container = engine.prepare(&container_input).unwrap();
    let moving = engine.prepare(&moving_input).unwrap();
    let obstacle = engine.prepare(&obstacle_input).unwrap();
    let obstacles = [PlacedObstacle {
        shape: &obstacle,
        translation: Translation::new(9.0, 1.0),
    }];
    let feasible = engine
        .feasible_region(FeasibleRequest {
            container: &container,
            moving: &moving,
            obstacles: &obstacles,
            clearance: Clearance::None,
        })
        .unwrap();

    let container_cells = CellShape::from_polygon_set(&container_input);
    let moving_cells = CellShape::from_polygon_set(&moving_input);
    let obstacle_cells = CellShape::from_polygon_set(&obstacle_input);
    assert!(contained(&container_cells, &moving_cells, 8, 0));
    assert!(!positive_area_overlap(
        &obstacle_cells,
        &moving_cells,
        -1,
        -1
    ));
    assert!(!regularized_feasible_at(
        &container_cells,
        &moving_cells,
        &[(&obstacle_cells, 9.0, 1.0)],
        8.0,
        0.0,
    ));
    // This anchor is feasible only along a zero-area contact line: below exits
    // the container and above overlaps the obstacle. PolygonSet intentionally
    // contains only filled 2-D strata, matching exact-fit IFP behavior.
    assert_location(&engine, &feasible, (8.0, 0.0), Location::Outside);
}

#[test]
fn feasible_membership_matches_containment_minus_positive_obstacle_overlap() {
    let engine = NfpEngine::new();
    let container_input = donut((0.0, 0.0, 12.0, 10.0), (4.0, 3.0, 8.0, 7.0));
    let moving_input = l_shape(2.0, 2.0, 1.0);
    let obstacle_a_input = rect(0.0, 0.0, 2.0, 2.0);
    let obstacle_b_input = rect(0.0, 0.0, 1.0, 3.0);
    let container = engine.prepare(&container_input).unwrap();
    let moving = engine.prepare(&moving_input).unwrap();
    let obstacle_a = engine.prepare(&obstacle_a_input).unwrap();
    let obstacle_b = engine.prepare(&obstacle_b_input).unwrap();
    let obstacles = [
        PlacedObstacle {
            shape: &obstacle_a,
            translation: Translation::new(1.0, 6.0),
        },
        PlacedObstacle {
            shape: &obstacle_b,
            translation: Translation::new(9.0, 1.0),
        },
    ];
    let feasible = engine
        .feasible_region(FeasibleRequest {
            container: &container,
            moving: &moving,
            obstacles: &obstacles,
            clearance: Clearance::None,
        })
        .unwrap();
    let container_cells = CellShape::from_polygon_set(&container_input);
    let moving_cells = CellShape::from_polygon_set(&moving_input);
    let obstacle_a_cells = CellShape::from_polygon_set(&obstacle_a_input);
    let obstacle_b_cells = CellShape::from_polygon_set(&obstacle_b_input);

    for ty in -3..=12 {
        for tx in -3..=14 {
            let expected = regularized_feasible_at(
                &container_cells,
                &moving_cells,
                &[(&obstacle_a_cells, 1.0, 6.0), (&obstacle_b_cells, 9.0, 1.0)],
                tx as f64,
                ty as f64,
            );
            let actual = engine
                .locate(&feasible, Point::new(tx as f64, ty as f64))
                .unwrap();
            assert_eq!(
                actual != Location::Outside,
                expected,
                "feasible mismatch at translation ({tx}, {ty}), got {actual:?}"
            );
        }
    }

    let repeated = engine
        .feasible_region(FeasibleRequest {
            container: &container,
            moving: &moving,
            obstacles: &obstacles,
            clearance: Clearance::None,
        })
        .unwrap();
    assert_eq!(
        serde_json::to_string(&feasible).unwrap(),
        serde_json::to_string(&repeated).unwrap()
    );
}
