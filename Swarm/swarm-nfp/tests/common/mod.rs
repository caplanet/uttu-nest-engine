#![allow(dead_code)]

use swarm_nfp::{Bounds, Location, NfpEngine, Point, Polygon, PolygonSet, Ring};
use std::collections::HashSet;

pub const EPS: f64 = 0.002;

pub fn ring(points: &[(f64, f64)]) -> Ring {
    Ring::new(points.iter().copied())
}

pub fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> PolygonSet {
    PolygonSet::from_outer(ring(&[(x0, y0), (x1, y0), (x1, y1), (x0, y1)]))
}

pub fn polygon(outer: &[(f64, f64)], holes: &[&[(f64, f64)]]) -> PolygonSet {
    PolygonSet::new(vec![Polygon::with_holes(
        ring(outer),
        holes.iter().map(|points| ring(points)).collect(),
    )])
}

pub fn donut(outer: (f64, f64, f64, f64), hole: (f64, f64, f64, f64)) -> PolygonSet {
    let (ox0, oy0, ox1, oy1) = outer;
    let (hx0, hy0, hx1, hy1) = hole;
    polygon(
        &[(ox0, oy0), (ox1, oy0), (ox1, oy1), (ox0, oy1)],
        &[&[(hx0, hy0), (hx0, hy1), (hx1, hy1), (hx1, hy0)]],
    )
}

pub fn l_shape(width: f64, height: f64, arm: f64) -> PolygonSet {
    PolygonSet::from_outer(ring(&[
        (0.0, 0.0),
        (width, 0.0),
        (width, arm),
        (arm, arm),
        (arm, height),
        (0.0, height),
    ]))
}

pub fn reverse_ring(mut points: Vec<(f64, f64)>) -> Vec<(f64, f64)> {
    points.reverse();
    points
}

pub fn point_reflect(input: &PolygonSet) -> PolygonSet {
    PolygonSet::new(
        input
            .components
            .iter()
            .map(|component| {
                let reflect_ring = |source: &Ring| {
                    Ring::new(source.vertices.iter().map(|point| (-point.x, -point.y)))
                };
                Polygon::with_holes(
                    reflect_ring(&component.outer),
                    component.holes.iter().map(reflect_ring).collect(),
                )
            })
            .collect(),
    )
}

/// Canonical per-component signatures without unioning overlapping NFP
/// primitives. This is used to compare primitive multisets while preserving
/// internal contact seams.
pub fn primitive_signatures(
    engine: &NfpEngine,
    primitives: &PolygonSet,
    reflect: bool,
) -> Vec<String> {
    let mut signatures = primitives
        .components
        .iter()
        .map(|component| {
            let one = PolygonSet::new(vec![component.clone()]);
            let one = if reflect { point_reflect(&one) } else { one };
            let prepared = engine
                .prepare(&one)
                .expect("an exported convex NFP primitive must prepare independently");
            assert_eq!(prepared.geometry().components.len(), 1);
            serde_json::to_string(prepared.geometry()).unwrap()
        })
        .collect::<Vec<_>>();
    signatures.sort();
    signatures
}

pub fn signed_ring_area(ring: &Ring) -> f64 {
    if ring.vertices.len() < 3 {
        return 0.0;
    }
    let mut twice = 0.0;
    for index in 0..ring.vertices.len() {
        let a = ring.vertices[index];
        let b = ring.vertices[(index + 1) % ring.vertices.len()];
        twice += a.x * b.y - b.x * a.y;
    }
    twice * 0.5
}

pub fn area(region: &PolygonSet) -> f64 {
    region
        .components
        .iter()
        .map(|component| {
            signed_ring_area(&component.outer).abs()
                - component
                    .holes
                    .iter()
                    .map(|hole| signed_ring_area(hole).abs())
                    .sum::<f64>()
        })
        .sum()
}

pub fn bounds(region: &PolygonSet) -> Option<Bounds> {
    let mut points = region
        .components
        .iter()
        .flat_map(|component| std::iter::once(&component.outer).chain(component.holes.iter()))
        .flat_map(|ring| ring.vertices.iter().copied());
    let first = points.next()?;
    let mut min = first;
    let mut max = first;
    for point in points {
        min.x = min.x.min(point.x);
        min.y = min.y.min(point.y);
        max.x = max.x.max(point.x);
        max.y = max.y.max(point.y);
    }
    Some(Bounds { min, max })
}

pub fn assert_near(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= EPS,
        "expected {expected:.6}, got {actual:.6} (tolerance {EPS})"
    );
}

pub fn assert_bounds(region: &PolygonSet, expected: (f64, f64, f64, f64)) {
    let actual = bounds(region).expect("expected a non-empty region");
    assert_near(actual.min.x, expected.0);
    assert_near(actual.min.y, expected.1);
    assert_near(actual.max.x, expected.2);
    assert_near(actual.max.y, expected.3);
}

pub fn assert_location(
    engine: &NfpEngine,
    region: &PolygonSet,
    point: (f64, f64),
    expected: Location,
) {
    let actual = engine
        .locate(region, Point::new(point.0, point.1))
        .expect("point location should succeed");
    assert_eq!(actual, expected, "unexpected location at {point:?}");
}

#[derive(Debug, Clone)]
pub struct CellShape {
    pub cells: HashSet<(i32, i32)>,
}

impl CellShape {
    /// Builds an independent unit-cell model for integer-coordinate orthogonal
    /// fixtures. Cell centres deliberately avoid all polygon boundaries.
    pub fn from_polygon_set(shape: &PolygonSet) -> Self {
        let b = bounds(shape).expect("cell fixture must not be empty");
        let mut cells = HashSet::new();
        for y in (b.min.y.floor() as i32)..(b.max.y.ceil() as i32) {
            for x in (b.min.x.floor() as i32)..(b.max.x.ceil() as i32) {
                let point = Point::new(x as f64 + 0.5, y as f64 + 0.5);
                if material_contains(shape, point) {
                    cells.insert((x, y));
                }
            }
        }
        Self { cells }
    }

    pub fn translated_cells(&self, tx: i32, ty: i32) -> HashSet<(i32, i32)> {
        self.cells.iter().map(|&(x, y)| (x + tx, y + ty)).collect()
    }
}

pub fn positive_area_overlap(fixed: &CellShape, moving: &CellShape, tx: i32, ty: i32) -> bool {
    moving
        .cells
        .iter()
        .any(|&(x, y)| fixed.cells.contains(&(x + tx, y + ty)))
}

pub fn positive_area_overlap_at(fixed: &CellShape, moving: &CellShape, tx: f64, ty: f64) -> bool {
    const TOLERANCE: f64 = 1.0e-9;
    fixed.cells.iter().any(|&(fx, fy)| {
        let fixed_x0 = fx as f64;
        let fixed_y0 = fy as f64;
        moving.cells.iter().any(|&(mx, my)| {
            let moving_x0 = mx as f64 + tx;
            let moving_y0 = my as f64 + ty;
            fixed_x0.max(moving_x0) + TOLERANCE < (fixed_x0 + 1.0).min(moving_x0 + 1.0)
                && fixed_y0.max(moving_y0) + TOLERANCE < (fixed_y0 + 1.0).min(moving_y0 + 1.0)
        })
    })
}

pub fn closed_contact(fixed: &CellShape, moving: &CellShape, tx: i32, ty: i32) -> bool {
    fixed.cells.iter().any(|&(fx, fy)| {
        moving.cells.iter().any(|&(mx, my)| {
            let dx = (fx - (mx + tx)).abs();
            let dy = (fy - (my + ty)).abs();
            dx <= 1 && dy <= 1
        })
    })
}

pub fn contained(container: &CellShape, moving: &CellShape, tx: i32, ty: i32) -> bool {
    moving
        .cells
        .iter()
        .all(|&(x, y)| container.cells.contains(&(x + tx, y + ty)))
}

pub fn contained_at(container: &CellShape, moving: &CellShape, tx: f64, ty: f64) -> bool {
    const TOLERANCE: f64 = 1.0e-9;
    moving.cells.iter().all(|&(mx, my)| {
        let x0 = mx as f64 + tx;
        let y0 = my as f64 + ty;
        let x1 = x0 + 1.0;
        let y1 = y0 + 1.0;
        let mut covered_area = 0.0;
        for cy in (y0.floor() as i32)..(y1.ceil() as i32) {
            for cx in (x0.floor() as i32)..(x1.ceil() as i32) {
                if !container.cells.contains(&(cx, cy)) {
                    continue;
                }
                let overlap_x = x1.min(cx as f64 + 1.0) - x0.max(cx as f64);
                let overlap_y = y1.min(cy as f64 + 1.0) - y0.max(cy as f64);
                if overlap_x > 0.0 && overlap_y > 0.0 {
                    covered_area += overlap_x * overlap_y;
                }
            }
        }
        covered_area + TOLERANCE >= 1.0
    })
}

pub fn regularized_feasible_at(
    container: &CellShape,
    moving: &CellShape,
    obstacles: &[(&CellShape, f64, f64)],
    tx: f64,
    ty: f64,
) -> bool {
    fn point_feasible(
        container: &CellShape,
        moving: &CellShape,
        obstacles: &[(&CellShape, f64, f64)],
        tx: f64,
        ty: f64,
    ) -> bool {
        contained_at(container, moving, tx, ty)
            && obstacles.iter().all(|(obstacle, ox, oy)| {
                !positive_area_overlap_at(obstacle, moving, tx - ox, ty - oy)
            })
    }

    if !point_feasible(container, moving, obstacles, tx, ty) {
        return false;
    }

    // PolygonSet stores regularized, positive-area 2-D regions. A valid
    // placement that exists only on an isolated line or point is deliberately
    // absent. Integer orthogonal fixtures have all events on integer lines, so
    // quarter-cell probes independently distinguish a 2-D incident sector.
    const OFFSETS: [f64; 3] = [-0.25, 0.0, 0.25];
    OFFSETS.iter().copied().any(|dy| {
        OFFSETS.iter().copied().any(|dx| {
            dx != 0.0 && dy != 0.0 && point_feasible(container, moving, obstacles, tx + dx, ty + dy)
        })
    })
}

pub fn material_contains(shape: &PolygonSet, point: Point) -> bool {
    shape.components.iter().any(|component| {
        point_in_ring_strict(&component.outer, point)
            && !component
                .holes
                .iter()
                .any(|hole| point_in_ring_strict(hole, point))
    })
}

fn point_in_ring_strict(ring: &Ring, point: Point) -> bool {
    let mut inside = false;
    let n = ring.vertices.len();
    for index in 0..n {
        let a = ring.vertices[index];
        let b = ring.vertices[(index + 1) % n];
        let crosses = (a.y > point.y) != (b.y > point.y)
            && point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x;
        if crosses {
            inside = !inside;
        }
    }
    inside
}
