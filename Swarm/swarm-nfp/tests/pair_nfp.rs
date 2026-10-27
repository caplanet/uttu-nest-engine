mod common;

use common::*;
use swarm_nfp::{
    Clearance, Location, MAX_DERIVED_GRID_COORD, NfpEngine, NfpError, Orientation, PairRelation,
    Point, Polygon, PolygonSet, Ring, Translation,
};

#[test]
fn analytic_rectangle_pair_has_exact_bounds_area_and_contact_semantics() {
    let engine = NfpEngine::new();
    let fixed = engine.prepare(&rect(0.0, 0.0, 4.0, 3.0)).unwrap();
    let moving = engine.prepare(&rect(0.0, 0.0, 2.0, 1.0)).unwrap();
    let nfp = engine.pair_nfp(&fixed, &moving).unwrap();

    assert_eq!(nfp.components.len(), 1);
    assert_eq!(nfp.ring_count(), 1);
    assert_bounds(&nfp, (-2.0, -1.0, 4.0, 3.0));
    assert_near(area(&nfp), 24.0);

    assert_location(&engine, &nfp, (0.0, 0.0), Location::Inside);
    assert_location(&engine, &nfp, (4.0, 0.5), Location::Boundary);
    assert_location(&engine, &nfp, (3.999, 0.5), Location::Inside);
    assert_location(&engine, &nfp, (4.001, 0.5), Location::Outside);
    assert_location(&engine, &nfp, (-2.0, -1.0), Location::Boundary);
}

#[test]
fn complete_primitive_count_is_available_before_expensive_construction() {
    let engine = NfpEngine::new();
    let fixed = engine.prepare(&rect(0.0, 0.0, 4.0, 3.0)).unwrap();
    let moving = engine.prepare(&rect(0.0, 0.0, 2.0, 1.0)).unwrap();

    // Preparation emits a *convex decomposition*, not a triangulation: a
    // rectangle is already convex, so it stays one piece.
    assert_eq!(fixed.triangle_count(), 1);
    assert_eq!(moving.triangle_count(), 1);

    // The estimate is exactly the atom product the pair NFP would build.
    assert_eq!(
        engine
            .complete_pair_primitive_count(&fixed, &moving, &Clearance::None)
            .unwrap(),
        fixed.triangle_count() * moving.triangle_count()
    );

    let kernel_ring = ring(&[(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)]);
    let clearance = Clearance::ConvexKernel(kernel_ring.clone());
    let kernel_pieces = engine
        .prepare(&PolygonSet::from_outer(kernel_ring))
        .unwrap()
        .triangle_count();
    assert_eq!(
        engine
            .complete_pair_primitive_count(&fixed, &moving, &clearance)
            .unwrap(),
        fixed.triangle_count() * moving.triangle_count() * kernel_pieces
    );
}

#[test]
fn concave_l_pair_preserves_the_reentrant_region() {
    let engine = NfpEngine::new();
    let fixed = engine.prepare(&l_shape(4.0, 4.0, 1.0)).unwrap();
    let moving = engine.prepare(&rect(0.0, 0.0, 1.0, 1.0)).unwrap();
    let nfp = engine.pair_nfp(&fixed, &moving).unwrap();

    assert_eq!(nfp.components.len(), 1);
    assert_eq!(nfp.ring_count(), 1);
    assert_bounds(&nfp, (-1.0, -1.0, 4.0, 4.0));
    assert_near(area(&nfp), 16.0);
    assert_location(&engine, &nfp, (0.0, 2.0), Location::Inside);
    assert_location(&engine, &nfp, (2.0, 0.0), Location::Inside);
    assert_location(&engine, &nfp, (1.0, 2.0), Location::Boundary);
    assert_location(&engine, &nfp, (2.0, 2.0), Location::Outside);
}

#[test]
fn a_fixed_donut_produces_a_true_safe_cavity() {
    let engine = NfpEngine::new();
    let fixed = engine
        .prepare(&donut((0.0, 0.0, 10.0, 10.0), (3.0, 3.0, 7.0, 7.0)))
        .unwrap();
    let moving = engine.prepare(&rect(0.0, 0.0, 2.0, 2.0)).unwrap();
    let nfp = engine.pair_nfp(&fixed, &moving).unwrap();

    assert_eq!(nfp.components.len(), 1);
    assert_eq!(nfp.components[0].holes.len(), 1);
    assert_bounds(&nfp, (-2.0, -2.0, 10.0, 10.0));
    assert_near(area(&nfp), 140.0);
    assert_location(&engine, &nfp, (4.0, 4.0), Location::Outside);
    assert_location(&engine, &nfp, (3.0, 4.0), Location::Boundary);
    assert_location(&engine, &nfp, (2.999, 4.0), Location::Inside);
    assert_location(&engine, &nfp, (5.001, 4.0), Location::Inside);
}

#[test]
fn a_moving_donut_produces_a_true_safe_cavity() {
    let engine = NfpEngine::new();
    let fixed = engine.prepare(&rect(0.0, 0.0, 1.0, 1.0)).unwrap();
    let moving = engine
        .prepare(&donut((0.0, 0.0, 10.0, 10.0), (3.0, 3.0, 7.0, 7.0)))
        .unwrap();
    let nfp = engine.pair_nfp(&fixed, &moving).unwrap();

    assert_eq!(nfp.components.len(), 1);
    assert_eq!(nfp.components[0].holes.len(), 1);
    assert_bounds(&nfp, (-10.0, -10.0, 1.0, 1.0));
    assert_near(area(&nfp), 112.0);
    assert_location(&engine, &nfp, (-4.5, -4.5), Location::Outside);
    assert_location(&engine, &nfp, (-6.0, -4.5), Location::Boundary);
    assert_location(&engine, &nfp, (-6.001, -4.5), Location::Inside);
    assert_location(&engine, &nfp, (-2.999, -4.5), Location::Inside);
}

#[test]
fn holes_on_both_shapes_are_supported_and_match_an_independent_cell_oracle() {
    let engine = NfpEngine::new();
    let fixed_input = donut((0.0, 0.0, 8.0, 8.0), (2.0, 2.0, 6.0, 6.0));
    let moving_input = donut((0.0, 0.0, 5.0, 5.0), (1.0, 1.0, 4.0, 4.0));
    let fixed = engine.prepare(&fixed_input).unwrap();
    let moving = engine.prepare(&moving_input).unwrap();
    let nfp = engine.pair_nfp(&fixed, &moving).unwrap();
    let complete = engine.complete_pair_nfp(&fixed, &moving).unwrap();
    let fixed_cells = CellShape::from_polygon_set(&fixed_input);
    let moving_cells = CellShape::from_polygon_set(&moving_input);

    assert_bounds(&nfp, (-5.0, -5.0, 8.0, 8.0));
    assert_eq!(
        complete.primitive_count(),
        fixed.triangle_count() * moving.triangle_count()
    );
    for ty in -7..=10 {
        for tx in -7..=10 {
            let expected_overlap = positive_area_overlap(&fixed_cells, &moving_cells, tx, ty);
            let expected_contact = closed_contact(&fixed_cells, &moving_cells, tx, ty);
            let actual = engine
                .locate(&nfp, Point::new(tx as f64, ty as f64))
                .unwrap();
            let complete_location = complete.locate(Point::new(tx as f64, ty as f64)).unwrap();
            assert_eq!(
                complete_location == Location::Inside,
                expected_overlap,
                "complete positive-overlap mismatch at translation ({tx}, {ty}), got {complete_location:?}"
            );
            if !expected_overlap && expected_contact {
                assert_eq!(
                    complete_location,
                    Location::Boundary,
                    "complete contact mismatch at translation ({tx}, {ty})"
                );
            }
            // This orthogonal fixture has no internal union seam at the probe
            // grid, so its compact contour should agree too.
            assert_eq!(actual, complete_location);
        }
    }
}

#[test]
fn disconnected_fixed_components_remain_disconnected_in_pair_nfp() {
    let engine = NfpEngine::new();
    let fixed_input = PolygonSet::new(vec![
        Polygon::new(ring(&[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)])),
        Polygon::new(ring(&[(10.0, 0.0), (12.0, 0.0), (12.0, 2.0), (10.0, 2.0)])),
    ]);
    let fixed = engine.prepare(&fixed_input).unwrap();
    let moving = engine.prepare(&rect(0.0, 0.0, 1.0, 1.0)).unwrap();
    let nfp = engine.pair_nfp(&fixed, &moving).unwrap();

    assert_eq!(nfp.components.len(), 2);
    assert_eq!(nfp.ring_count(), 2);
    assert_bounds(&nfp, (-1.0, -1.0, 12.0, 2.0));
    assert_near(area(&nfp), 18.0);
    assert_location(&engine, &nfp, (0.0, 0.0), Location::Inside);
    assert_location(&engine, &nfp, (6.0, 0.0), Location::Outside);
    assert_location(&engine, &nfp, (10.0, 0.0), Location::Inside);
}

#[test]
fn reverse_pair_relation_is_negation_for_asymmetric_shapes() {
    let engine = NfpEngine::new();
    let a = engine.prepare(&l_shape(5.0, 4.0, 1.0)).unwrap();
    let b = engine
        .prepare(&polygon(&[(1.0, -1.0), (4.0, -1.0), (2.0, 2.0)], &[]))
        .unwrap()
        .oriented(Orientation {
            angle_microdegrees: 33_333_333,
            mirror_x: true,
        })
        .unwrap();
    let ab = engine.pair_nfp(&a, &b).unwrap();
    let ba = engine.pair_nfp(&b, &a).unwrap();
    let reflected_ba = engine.prepare(&point_reflect(&ba)).unwrap();

    assert_eq!(
        serde_json::to_string(&ab).unwrap(),
        serde_json::to_string(reflected_ba.geometry()).unwrap(),
        "canonical operand ordering must make swap-negation byte-stable"
    );

    assert_near(area(&ab), area(&ba));
    assert_eq!(ab.components.len(), ba.components.len());
    assert_eq!(ab.ring_count(), ba.ring_count());
    let ab_bounds = bounds(&ab).unwrap();
    let ba_bounds = bounds(&ba).unwrap();
    assert_near(ab_bounds.min.x, -ba_bounds.max.x);
    assert_near(ab_bounds.min.y, -ba_bounds.max.y);
    assert_near(ab_bounds.max.x, -ba_bounds.min.x);
    assert_near(ab_bounds.max.y, -ba_bounds.min.y);

    for yi in -16..=16 {
        for xi in -16..=16 {
            let x = xi as f64 * 0.5;
            let y = yi as f64 * 0.5;
            let lhs = engine.locate(&ab, Point::new(x, y)).unwrap();
            let rhs = engine.locate(&ba, Point::new(-x, -y)).unwrap();
            assert_eq!(lhs, rhs, "reverse mismatch at ({x}, {y})");
        }
    }
}

#[test]
fn complete_nfp_membership_is_deterministic_and_exactly_negates_when_swapped() {
    let engine = NfpEngine::new();
    let a = engine.prepare(&l_shape(5.0, 4.0, 1.0)).unwrap();
    let b = engine
        .prepare(&polygon(
            &[(-0.5, -1.0), (3.5, -0.5), (2.0, 2.5), (0.0, 1.5)],
            &[],
        ))
        .unwrap()
        .oriented(Orientation {
            angle_microdegrees: 17_250_000,
            mirror_x: true,
        })
        .unwrap();

    let ab = engine.complete_pair_nfp(&a, &b).unwrap();
    let ab_repeat = engine.complete_pair_nfp(&a, &b).unwrap();
    let ba = engine.complete_pair_nfp(&b, &a).unwrap();
    let compact_ab = engine.pair_nfp(&a, &b).unwrap();

    assert_eq!(ab.primitive_count(), ba.primitive_count());
    assert_eq!(
        serde_json::to_string(ab.primitives()).unwrap(),
        serde_json::to_string(ab_repeat.primitives()).unwrap(),
        "exact primitive ordering must be deterministic"
    );
    assert_eq!(
        serde_json::to_string(ab.compact_contour()).unwrap(),
        serde_json::to_string(ab_repeat.compact_contour()).unwrap(),
        "complete compact view must be deterministic"
    );
    assert_eq!(
        serde_json::to_string(ab.compact_contour()).unwrap(),
        serde_json::to_string(&compact_ab).unwrap(),
        "standalone compact API and complete compact view must agree"
    );

    for yi in -24..=24 {
        for xi in -24..=24 {
            let x = xi as f64 * 0.25;
            let y = yi as f64 * 0.25;
            assert_eq!(
                ab.locate(Point::new(x, y)).unwrap(),
                ba.locate(Point::new(-x, -y)).unwrap(),
                "complete reverse mismatch at ({x}, {y})"
            );
        }
    }
}

#[test]
fn disconnected_internal_seam_is_classified_as_contact_not_overlap() {
    let engine = NfpEngine::new();
    let fixed = engine
        .prepare(&PolygonSet::new(vec![
            Polygon::new(ring(&[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)])),
            Polygon::new(ring(&[(2.0, 0.0), (3.0, 0.0), (3.0, 1.0), (2.0, 1.0)])),
        ]))
        .unwrap();
    let moving = engine.prepare(&rect(0.0, 0.0, 1.0, 1.0)).unwrap();
    let nfp = engine.pair_nfp(&fixed, &moving).unwrap();
    let complete = engine.complete_pair_nfp(&fixed, &moving).unwrap();

    // The two component NFPs meet at x=1. Their regularized union makes this
    // seam topologically Inside, but the actual shapes only touch on both sides.
    assert_location(&engine, &nfp, (1.0, 0.0), Location::Inside);
    assert_eq!(
        complete.locate(Point::new(1.0, 0.0)).unwrap(),
        Location::Boundary
    );
    assert_eq!(
        engine
            .classify_pair(&fixed, &moving, Translation::new(1.0, 0.0))
            .unwrap(),
        PairRelation::Contact
    );
    assert_eq!(
        engine
            .classify_pair(&fixed, &moving, Translation::ZERO)
            .unwrap(),
        PairRelation::Overlap
    );
    assert_eq!(
        engine
            .classify_pair(&fixed, &moving, Translation::new(4.001, 0.0))
            .unwrap(),
        PairRelation::Disjoint
    );
}

#[test]
fn self_pair_nfp_is_centrally_symmetric_after_canonicalization() {
    let engine = NfpEngine::new();
    let shape = engine
        .prepare(&donut((0.0, 0.0, 8.0, 7.0), (1.0, 2.0, 5.0, 5.0)))
        .unwrap();
    let nfp = engine.pair_nfp(&shape, &shape).unwrap();
    let reflected = engine.prepare(&point_reflect(&nfp)).unwrap();
    assert_eq!(
        serde_json::to_string(&nfp).unwrap(),
        serde_json::to_string(reflected.geometry()).unwrap()
    );
}

#[test]
fn locate_canonicalizes_overlapping_and_nested_public_components() {
    let engine = NfpEngine::new();
    let overlapping = PolygonSet::new(vec![
        Polygon::new(ring(&[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)])),
        Polygon::new(ring(&[(1.0, 0.0), (3.0, 0.0), (3.0, 2.0), (1.0, 2.0)])),
    ]);
    assert_eq!(
        engine.locate(&overlapping, Point::new(1.0, 1.0)).unwrap(),
        Location::Inside
    );

    let nested_solids = PolygonSet::new(vec![
        Polygon::new(ring(&[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)])),
        Polygon::new(ring(&[(2.0, 2.0), (8.0, 2.0), (8.0, 8.0), (2.0, 8.0)])),
    ]);
    assert_eq!(
        engine.locate(&nested_solids, Point::new(5.0, 5.0)).unwrap(),
        Location::Inside
    );
}

#[test]
fn rotation_and_mirror_are_applied_about_the_origin_in_documented_order() {
    let engine = NfpEngine::new();
    let source = engine
        .prepare(&polygon(&[(0.0, 0.0), (3.0, 0.0), (0.0, 2.0)], &[]))
        .unwrap();

    let rotated = source
        .oriented(Orientation::from_degrees_exact(90))
        .unwrap();
    assert_bounds(rotated.geometry(), (-2.0, 0.0, 0.0, 3.0));
    assert_near(rotated.area_mm2(), 3.0);

    let mirrored = source
        .oriented(Orientation {
            angle_microdegrees: 0,
            mirror_x: true,
        })
        .unwrap();
    assert_bounds(mirrored.geometry(), (-3.0, 0.0, 0.0, 2.0));

    let mirrored_then_rotated = source
        .oriented(Orientation {
            angle_microdegrees: 90_000_000,
            mirror_x: true,
        })
        .unwrap();
    assert_bounds(mirrored_then_rotated.geometry(), (-2.0, -3.0, 0.0, 0.0));

    let arbitrary_angle = 33.333_333_f64;
    let arbitrary = source
        .oriented(Orientation {
            angle_microdegrees: 33_333_333,
            mirror_x: true,
        })
        .unwrap();
    let radians = arbitrary_angle.to_radians();
    assert_near(arbitrary.bounds().min.x, -3.0 * radians.cos());
    assert_near(arbitrary.bounds().min.y, -3.0 * radians.sin());
    assert_near(arbitrary.area_mm2(), 3.0);
}

#[test]
fn orientation_rejects_a_hole_that_collapses_on_the_grid() {
    let engine = NfpEngine::new();
    let tiny_hole = engine
        .prepare(&polygon(
            &[
                (-0.200, -0.200),
                (0.200, -0.200),
                (0.200, 0.200),
                (-0.200, 0.200),
            ],
            &[&[(-0.004, -0.004), (-0.005, -0.004), (-0.005, 0.100)]],
        ))
        .unwrap();
    assert_eq!(tiny_hole.geometry().components[0].holes.len(), 1);

    let error = tiny_hole
        .oriented(Orientation::from_degrees_exact(50))
        .unwrap_err();
    assert!(matches!(
        error,
        NfpError::TooFewVertices { .. }
            | NfpError::ZeroAreaRing { .. }
            | NfpError::SelfIntersection { .. }
            | NfpError::HoleOutsideShell { .. }
    ));
}

#[test]
fn normalization_removes_closing_consecutive_and_quantized_duplicate_vertices() {
    let engine = NfpEngine::new();
    let canonical = engine.prepare(&rect(0.0, 0.0, 4.0, 3.0)).unwrap();
    let noisy = PolygonSet::from_outer(Ring::new([
        (0.0, 0.0),
        (1.0, 0.0),
        (2.0, 0.0),
        (2.0, 0.000_4),
        (4.0, 0.0),
        (4.0, 0.0),
        (4.0, 3.0),
        (0.0, 3.0),
        (0.0, 0.0),
    ]));
    let normalized = engine.prepare(&noisy).unwrap();

    assert_near(normalized.area_mm2(), canonical.area_mm2());
    assert_eq!(normalized.bounds(), canonical.bounds());
    assert_eq!(
        serde_json::to_string(normalized.geometry()).unwrap(),
        serde_json::to_string(canonical.geometry()).unwrap()
    );
}

#[test]
fn canonical_output_is_independent_of_winding_component_order_and_repetition() {
    let engine = NfpEngine::new();
    let left = vec![(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)];
    let right = vec![(8.0, 1.0), (11.0, 1.0), (11.0, 3.0), (8.0, 3.0)];
    let forward = PolygonSet::new(vec![Polygon::new(ring(&left)), Polygon::new(ring(&right))]);
    let reversed = PolygonSet::new(vec![
        Polygon::new(ring(&reverse_ring(right))),
        Polygon::new(ring(&reverse_ring(left))),
    ]);
    let moving = engine.prepare(&rect(-1.0, 0.0, 1.0, 1.0)).unwrap();
    let forward = engine.prepare(&forward).unwrap();
    let reversed = engine.prepare(&reversed).unwrap();

    assert_eq!(
        serde_json::to_string(forward.geometry()).unwrap(),
        serde_json::to_string(reversed.geometry()).unwrap()
    );
    let first = engine.pair_nfp(&forward, &moving).unwrap();
    let second = engine.pair_nfp(&reversed, &moving).unwrap();
    let third = engine.pair_nfp(&forward, &moving).unwrap();
    assert_eq!(
        serde_json::to_string(&first).unwrap(),
        serde_json::to_string(&second).unwrap()
    );
    assert_eq!(
        serde_json::to_string(&first).unwrap(),
        serde_json::to_string(&third).unwrap()
    );
}

#[test]
fn invalid_geometry_and_coordinate_overflow_are_typed() {
    let engine = NfpEngine::new();

    assert!(matches!(
        engine.prepare(&PolygonSet::default()),
        Err(NfpError::EmptyShape)
    ));
    assert!(matches!(
        engine.prepare(&PolygonSet::from_outer(ring(&[
            (0.0, 0.0),
            (f64::NAN, 1.0),
            (1.0, 0.0),
        ]))),
        Err(NfpError::NonFiniteCoordinate { .. })
    ));
    assert!(matches!(
        engine.prepare(&PolygonSet::from_outer(ring(&[
            (0.0, 0.0),
            (0.000_4, 0.000_4),
            (0.0, 0.0),
        ]))),
        Err(NfpError::TooFewVertices { .. })
    ));
    assert!(matches!(
        engine.prepare(&PolygonSet::from_outer(ring(&[
            (0.0, 0.0),
            (2.0, 0.0),
            (4.0, 0.0),
        ]))),
        Err(NfpError::TooFewVertices { .. })
    ));
    assert!(matches!(
        engine.prepare(&PolygonSet::from_outer(ring(&[
            (0.0, 0.0),
            (4.0, 3.0),
            (0.0, 4.0),
            (3.0, 0.0),
        ]))),
        Err(NfpError::SelfIntersection { .. })
    ));

    let outside_hole = polygon(
        &[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)],
        &[&[(3.0, 3.0), (5.0, 3.0), (5.0, 5.0), (3.0, 5.0)]],
    );
    assert!(matches!(
        engine.prepare(&outside_hole),
        Err(NfpError::HoleOutsideShell { .. })
    ));

    let over_mm = MAX_DERIVED_GRID_COORD as f64 / 1_000.0 + 1.0;
    assert!(matches!(
        engine.prepare(&rect(over_mm, 0.0, over_mm + 1.0, 1.0)),
        Err(NfpError::CoordinateOverflow { .. })
    ));
    assert!(matches!(
        Orientation::from_degrees(f64::NAN),
        Err(NfpError::InvalidOrientation { .. })
    ));
    assert!(matches!(
        Orientation::from_degrees(f64::INFINITY),
        Err(NfpError::InvalidOrientation { .. })
    ));
}

#[test]
fn convex_and_euclidean_clearance_expand_the_forbidden_region_conservatively() {
    let engine = NfpEngine::new();
    let fixed = engine.prepare(&rect(0.0, 0.0, 2.0, 2.0)).unwrap();
    let moving = engine.prepare(&rect(0.0, 0.0, 1.0, 1.0)).unwrap();

    let square_kernel =
        Clearance::ConvexKernel(ring(&[(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)]));
    let expanded = engine
        .pair_nfp_with_clearance(&fixed, &moving, &square_kernel)
        .unwrap();
    assert_bounds(&expanded, (-1.5, -1.5, 2.5, 2.5));
    assert_near(area(&expanded), 16.0);

    // An asymmetric kernel catches an accidental reflection in true
    // Minkowski dilation: base [-1,-1]-[2,2] plus K [-.25,-.5]-[1,.5].
    let asymmetric_kernel = Clearance::ConvexKernel(ring(&[
        (-0.25, -0.5),
        (1.0, -0.5),
        (1.0, 0.5),
        (-0.25, 0.5),
    ]));
    let asymmetric = engine
        .pair_nfp_with_clearance(&fixed, &moving, &asymmetric_kernel)
        .unwrap();
    assert_bounds(&asymmetric, (-1.25, -1.5, 3.0, 2.5));

    let euclidean = engine
        .pair_nfp_with_clearance(
            &fixed,
            &moving,
            &Clearance::EuclideanConservative {
                distance_mm: 1.0,
                max_excess_mm: 0.01,
            },
        )
        .unwrap();
    assert_location(&engine, &euclidean, (2.999, 0.5), Location::Inside);
    assert_location(&engine, &euclidean, (3.02, 0.5), Location::Outside);

    assert!(matches!(
        engine.pair_nfp_with_clearance(
            &fixed,
            &moving,
            &Clearance::EuclideanConservative {
                distance_mm: -1.0,
                max_excess_mm: 0.01,
            },
        ),
        Err(NfpError::InvalidClearance)
    ));
    assert!(matches!(
        engine.pair_nfp_with_clearance(
            &fixed,
            &moving,
            &Clearance::EuclideanConservative {
                distance_mm: 1.0,
                max_excess_mm: 0.0,
            },
        ),
        Err(NfpError::InvalidClearanceTolerance)
    ));
}

#[test]
fn complete_clearance_locate_distinguishes_violation_limit_and_safe_gap() {
    let engine = NfpEngine::new();
    let fixed = engine.prepare(&rect(0.0, 0.0, 2.0, 2.0)).unwrap();
    let moving = engine.prepare(&rect(0.0, 0.0, 1.0, 1.0)).unwrap();
    let clearance =
        Clearance::ConvexKernel(ring(&[(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)]));

    let complete = engine
        .complete_pair_nfp_with_clearance(&fixed, &moving, &clearance)
        .unwrap();

    let kernel_pieces = engine
        .prepare(&PolygonSet::from_outer(ring(&[
            (-0.5, -0.5),
            (0.5, -0.5),
            (0.5, 0.5),
            (-0.5, 0.5),
        ])))
        .unwrap()
        .triangle_count();
    assert_eq!(
        complete.primitive_count(),
        fixed.triangle_count() * moving.triangle_count() * kernel_pieces
    );
    assert_eq!(
        complete.locate(Point::new(2.25, 0.5)).unwrap(),
        Location::Inside,
        "a 0.25 mm material gap violates the requested 0.5 mm clearance"
    );
    assert_eq!(
        complete.locate(Point::new(2.5, 0.5)).unwrap(),
        Location::Boundary
    );
    assert_eq!(
        complete.locate(Point::new(2.501, 0.5)).unwrap(),
        Location::Outside
    );

    let euclidean = Clearance::EuclideanConservative {
        distance_mm: 0.1,
        max_excess_mm: 0.003,
    };
    let euclidean_ab = engine
        .complete_pair_nfp_with_clearance(&fixed, &moving, &euclidean)
        .unwrap();
    let euclidean_ba = engine
        .complete_pair_nfp_with_clearance(&moving, &fixed, &euclidean)
        .unwrap();
    let compact_ab = engine
        .pair_nfp_with_clearance(&fixed, &moving, &euclidean)
        .unwrap();
    let reflected_compact_ba = engine
        .prepare(&point_reflect(euclidean_ba.compact_contour()))
        .unwrap();

    // The exact count depends on how many convex pieces each operand and the
    // conservative disc kernel decompose into, so assert against the estimator
    // the kernel publishes for exactly this purpose rather than a literal.
    assert_eq!(
        euclidean_ab.primitive_count(),
        engine
            .complete_pair_primitive_count(&fixed, &moving, &euclidean)
            .unwrap()
    );
    assert_eq!(
        primitive_signatures(&engine, euclidean_ab.primitives(), false),
        primitive_signatures(&engine, euclidean_ba.primitives(), true),
        "the Euclidean-clearance primitive decomposition must point-reflect exactly"
    );
    assert_eq!(
        serde_json::to_string(euclidean_ab.compact_contour()).unwrap(),
        serde_json::to_string(&compact_ab).unwrap()
    );
    assert_eq!(
        serde_json::to_string(euclidean_ab.compact_contour()).unwrap(),
        serde_json::to_string(reflected_compact_ba.geometry()).unwrap(),
        "a conservative Euclidean clearance kernel must remain centrally symmetric"
    );

    for yi in -60..=60 {
        for xi in -60..=60 {
            let x = xi as f64 * 0.05;
            let y = yi as f64 * 0.05;
            assert_eq!(
                euclidean_ab.locate(Point::new(x, y)).unwrap(),
                euclidean_ba.locate(Point::new(-x, -y)).unwrap(),
                "Euclidean-clearance reverse mismatch at ({x}, {y})"
            );
        }
    }

    // This tolerance previously selected a 13-sided/14-sided transition and
    // exposed a kernel-triangulation decomposition asymmetry even when union
    // membership happened to agree.
    let tight_euclidean = Clearance::EuclideanConservative {
        distance_mm: 0.01,
        max_excess_mm: 0.001,
    };
    let tight_ab = engine
        .complete_pair_nfp_with_clearance(&fixed, &moving, &tight_euclidean)
        .unwrap();
    let tight_ba = engine
        .complete_pair_nfp_with_clearance(&moving, &fixed, &tight_euclidean)
        .unwrap();
    assert_eq!(
        tight_ab.primitive_count(),
        engine
            .complete_pair_primitive_count(&fixed, &moving, &tight_euclidean)
            .unwrap()
    );
    assert_eq!(
        primitive_signatures(&engine, tight_ab.primitives(), false),
        primitive_signatures(&engine, tight_ba.primitives(), true)
    );
}
