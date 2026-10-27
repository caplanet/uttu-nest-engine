//! Checked grid conversion, canonical topology, Boolean operations and
//! material triangulation shared by the public NFP operations.

use std::cmp::Ordering;
use std::collections::{HashMap, HashSet};

use clipper2::{FillRule, Milli, Path, Paths, Point as ClipperPoint};

use crate::{
    Bounds, GRID_SCALE, Location, MAX_DERIVED_GRID_COORD, NfpError, NfpResult, Orientation,
    PairRelation, Point, Polygon, PolygonSet, Ring,
};

pub(crate) type GridPoint = ClipperPoint<Milli>;
pub(crate) type GridPath = Path<Milli>;
pub(crate) type GridPaths = Paths<Milli>;

#[derive(Clone)]
struct GridRing {
    points: Vec<GridPoint>,
    area2: i128,
}

#[derive(Clone)]
struct GridComponent {
    outer: GridRing,
    holes: Vec<GridRing>,
    /// Set by input validation when a hole touches the shell or a sibling
    /// hole at a vertex without crossing it. Such topology is produced by
    /// Clipper output canonicalization and may defeat ear-cut triangulation,
    /// so the representability proof in `merge_interacting_components` is
    /// advisory for these components.
    touching: bool,
}

pub(crate) struct PreparedGeometry {
    pub(crate) geometry: PolygonSet,
    pub(crate) canonical_paths: GridPaths,
    pub(crate) triangles: Vec<GridPath>,
    pub(crate) bounds: Bounds,
    pub(crate) area2: i128,
    pub(crate) max_abs: i64,
}

pub(crate) fn quantize_mm(value: f64, stage: &'static str) -> NfpResult<i64> {
    if !value.is_finite() {
        return Err(NfpError::NonFiniteCoordinate {
            context: stage,
            ring: 0,
            vertex: 0,
        });
    }
    let scaled = value * GRID_SCALE as f64;
    let rounded = scaled.round();
    if !scaled.is_finite() || rounded.abs() > MAX_DERIVED_GRID_COORD as f64 {
        return Err(NfpError::CoordinateOverflow {
            stage,
            value: rounded as i128,
            limit: MAX_DERIVED_GRID_COORD,
        });
    }
    Ok(rounded as i64)
}

fn checked_grid_point(
    point: Point,
    context: &'static str,
    ring: usize,
    vertex: usize,
) -> NfpResult<GridPoint> {
    if !point.x.is_finite() || !point.y.is_finite() {
        return Err(NfpError::NonFiniteCoordinate {
            context,
            ring,
            vertex,
        });
    }
    let x = quantize_coordinate(point.x, context, ring, vertex)?;
    let y = quantize_coordinate(point.y, context, ring, vertex)?;
    Ok(GridPoint::from_scaled(x, y))
}

fn quantize_coordinate(
    value: f64,
    context: &'static str,
    ring: usize,
    vertex: usize,
) -> NfpResult<i64> {
    if !value.is_finite() {
        return Err(NfpError::NonFiniteCoordinate {
            context,
            ring,
            vertex,
        });
    }
    let scaled = value * GRID_SCALE as f64;
    let rounded = scaled.round();
    if !scaled.is_finite() || rounded.abs() > MAX_DERIVED_GRID_COORD as f64 {
        return Err(NfpError::CoordinateOverflow {
            stage: context,
            value: rounded as i128,
            limit: MAX_DERIVED_GRID_COORD,
        });
    }
    Ok(rounded as i64)
}

fn x(point: GridPoint) -> i64 {
    point.x_scaled()
}

fn y(point: GridPoint) -> i64 {
    point.y_scaled()
}

fn point_key(point: GridPoint) -> (i64, i64) {
    (x(point), y(point))
}

fn orient(a: GridPoint, b: GridPoint, c: GridPoint) -> i128 {
    let abx = i128::from(x(b)) - i128::from(x(a));
    let aby = i128::from(y(b)) - i128::from(y(a));
    let acx = i128::from(x(c)) - i128::from(x(a));
    let acy = i128::from(y(c)) - i128::from(y(a));
    abx * acy - aby * acx
}

fn ring_area2(points: &[GridPoint]) -> NfpResult<i128> {
    let mut area2 = 0i128;
    for index in 0..points.len() {
        let a = points[index];
        let b = points[(index + 1) % points.len()];
        let cross = i128::from(x(a))
            .checked_mul(i128::from(y(b)))
            .and_then(|lhs| {
                i128::from(y(a))
                    .checked_mul(i128::from(x(b)))
                    .and_then(|rhs| lhs.checked_sub(rhs))
            })
            .ok_or(NfpError::CoordinateOverflow {
                stage: "area accumulation",
                value: i128::MAX,
                limit: MAX_DERIVED_GRID_COORD,
            })?;
        area2 = area2
            .checked_add(cross)
            .ok_or(NfpError::CoordinateOverflow {
                stage: "area accumulation",
                value: i128::MAX,
                limit: MAX_DERIVED_GRID_COORD,
            })?;
    }
    Ok(area2)
}

fn is_collinear_middle(previous: GridPoint, current: GridPoint, next: GridPoint) -> bool {
    if orient(previous, current, next) != 0 {
        return false;
    }
    let incoming_x = i128::from(x(current)) - i128::from(x(previous));
    let incoming_y = i128::from(y(current)) - i128::from(y(previous));
    let outgoing_x = i128::from(x(next)) - i128::from(x(current));
    let outgoing_y = i128::from(y(next)) - i128::from(y(current));
    incoming_x * outgoing_x + incoming_y * outgoing_y >= 0
}

fn remove_redundant_vertices(points: &mut Vec<GridPoint>) {
    while points.len() > 1 && points.first() == points.last() {
        points.pop();
    }

    let mut deduplicated = Vec::with_capacity(points.len());
    for point in points.drain(..) {
        if deduplicated.last().copied() != Some(point) {
            deduplicated.push(point);
        }
    }
    while deduplicated.len() > 1 && deduplicated.first() == deduplicated.last() {
        deduplicated.pop();
    }
    *points = deduplicated;

    // Drop forward-collinear middle vertices in a single in-place pass and
    // repeat until stable. Removing such a vertex leaves the ring's edge
    // geometry unchanged, so whether any other vertex is removable does not
    // depend on the order of removals; the fixed point is unique.
    loop {
        if points.len() < 3 {
            return;
        }
        let len = points.len();
        let mut kept = 0usize;
        for index in 0..len {
            let previous = if kept == 0 {
                points[len - 1]
            } else {
                points[kept - 1]
            };
            let current = points[index];
            let next = points[(index + 1) % len];
            if !is_collinear_middle(previous, current, next) {
                points[kept] = current;
                kept += 1;
            }
        }
        if kept == len {
            return;
        }
        points.truncate(kept);
    }
}

fn on_segment(point: GridPoint, a: GridPoint, b: GridPoint) -> bool {
    orient(a, b, point) == 0
        && x(point) >= x(a).min(x(b))
        && x(point) <= x(a).max(x(b))
        && y(point) >= y(a).min(y(b))
        && y(point) <= y(a).max(y(b))
}

fn segments_intersect(a: GridPoint, b: GridPoint, c: GridPoint, d: GridPoint) -> bool {
    let ab_c = orient(a, b, c);
    let ab_d = orient(a, b, d);
    let cd_a = orient(c, d, a);
    let cd_b = orient(c, d, b);

    if ((ab_c > 0 && ab_d < 0) || (ab_c < 0 && ab_d > 0))
        && ((cd_a > 0 && cd_b < 0) || (cd_a < 0 && cd_b > 0))
    {
        return true;
    }
    (ab_c == 0 && on_segment(c, a, b))
        || (ab_d == 0 && on_segment(d, a, b))
        || (cd_a == 0 && on_segment(a, c, d))
        || (cd_b == 0 && on_segment(b, c, d))
}

fn ring_self_intersects(points: &[GridPoint]) -> bool {
    let edge_count = points.len();
    // Sweep edges by their minimum x. Two edges whose x-ranges are disjoint
    // cannot share a point, so once a later edge starts past the current
    // edge's maximum x no further edge in sorted order can intersect it.
    let mut edges: Vec<(i64, i64, usize)> = (0..edge_count)
        .map(|index| {
            let a = points[index];
            let b = points[(index + 1) % edge_count];
            (x(a).min(x(b)), x(a).max(x(b)), index)
        })
        .collect();
    edges.sort_unstable();
    for sorted_first in 0..edge_count {
        let (_, first_max_x, first) = edges[sorted_first];
        let a = points[first];
        let b = points[(first + 1) % edge_count];
        for sorted_second in (sorted_first + 1)..edge_count {
            let (second_min_x, _, second) = edges[sorted_second];
            if second_min_x > first_max_x {
                break;
            }
            let c = points[second];
            let d = points[(second + 1) % edge_count];
            let adjacent = second == first + 1
                || first == second + 1
                || (first == 0 && second + 1 == edge_count)
                || (second == 0 && first + 1 == edge_count);
            if !segments_intersect(a, b, c, d) {
                continue;
            }
            if !adjacent {
                return true;
            }

            // Adjacent edges may share exactly their common endpoint, but may
            // not reverse over one another.
            if orient(a, b, c) == 0 && orient(a, b, d) == 0 {
                let shared = if b == c {
                    b
                } else if a == d {
                    a
                } else {
                    return true;
                };
                for endpoint in [a, b, c, d] {
                    if endpoint != shared
                        && on_segment(endpoint, a, b)
                        && on_segment(endpoint, c, d)
                    {
                        return true;
                    }
                }
            }
        }
    }
    false
}

/// Whether two segments cross properly: each straddles the other's supporting
/// line with strict orientation signs. Shared endpoints, an endpoint lying on
/// the other segment, and collinear overlap are all *not* proper crossings.
fn segments_cross(a: GridPoint, b: GridPoint, c: GridPoint, d: GridPoint) -> bool {
    let ab_c = orient(a, b, c);
    let ab_d = orient(a, b, d);
    let cd_a = orient(c, d, a);
    let cd_b = orient(c, d, b);
    ((ab_c > 0 && ab_d < 0) || (ab_c < 0 && ab_d > 0))
        && ((cd_a > 0 && cd_b < 0) || (cd_a < 0 && cd_b > 0))
}

/// Whether any edge of `first` properly crosses any edge of `second`.
/// Touching topology (a shared vertex, or a vertex on the other ring's edge)
/// is deliberately allowed: Clipper output canonicalized by
/// `canonical_components_from_paths` legitimately contains holes that touch
/// their shell or each other, and such regions must round-trip as input.
fn rings_cross(first: &[GridPoint], second: &[GridPoint]) -> bool {
    for first_index in 0..first.len() {
        let a = first[first_index];
        let b = first[(first_index + 1) % first.len()];
        for second_index in 0..second.len() {
            let c = second[second_index];
            let d = second[(second_index + 1) % second.len()];
            if segments_cross(a, b, c, d) {
                return true;
            }
        }
    }
    false
}

/// Side of `ring` on which the non-boundary vertices of `probe` lie.
///
/// Returns `Some(Inside)` / `Some(Outside)` when every vertex of `probe` that
/// is not on the boundary of `ring` agrees, and `None` when they disagree or
/// when every vertex touches the boundary. Combined with [`rings_cross`] this
/// classifies touching-but-non-crossing rings exactly.
fn ring_probe(ring: &[GridPoint], probe: &[GridPoint]) -> RingProbe {
    let mut side: Option<Location> = None;
    let mut touches = false;
    for point in probe {
        match point_in_ring(ring, *point) {
            Location::Boundary => touches = true,
            location => match side {
                None => side = Some(location),
                Some(previous) if previous == location => {}
                Some(_) => return RingProbe { side: None, touches },
            },
        }
    }
    RingProbe { side, touches }
}

/// Result of [`ring_probe`].
#[derive(Clone, Copy)]
struct RingProbe {
    /// Consistent side of the non-boundary vertices, if any.
    side: Option<Location>,
    /// Whether at least one probed vertex lies on the ring boundary.
    touches: bool,
}

fn rings_intersect(first: &[GridPoint], second: &[GridPoint]) -> bool {
    for first_index in 0..first.len() {
        let a = first[first_index];
        let b = first[(first_index + 1) % first.len()];
        for second_index in 0..second.len() {
            let c = second[second_index];
            let d = second[(second_index + 1) % second.len()];
            if segments_intersect(a, b, c, d) {
                return true;
            }
        }
    }
    false
}

fn point_in_ring(points: &[GridPoint], point: GridPoint) -> Location {
    let mut winding = 0i32;
    for index in 0..points.len() {
        let a = points[index];
        let b = points[(index + 1) % points.len()];
        if on_segment(point, a, b) {
            return Location::Boundary;
        }
        if y(a) <= y(point) {
            if y(b) > y(point) && orient(a, b, point) > 0 {
                winding += 1;
            }
        } else if y(b) <= y(point) && orient(a, b, point) < 0 {
            winding -= 1;
        }
    }
    if winding == 0 {
        Location::Outside
    } else {
        Location::Inside
    }
}

fn rotate_to_canonical_start(points: &mut [GridPoint]) {
    if let Some((index, _)) = points
        .iter()
        .enumerate()
        .min_by_key(|(_, point)| point_key(**point))
    {
        points.rotate_left(index);
    }
}

fn normalize_grid_ring(mut points: Vec<GridPoint>, ccw: bool) -> NfpResult<GridRing> {
    let mut area2 = ring_area2(&points)?;
    if (ccw && area2 < 0) || (!ccw && area2 > 0) {
        points.reverse();
        area2 = -area2;
    }
    rotate_to_canonical_start(&mut points);
    Ok(GridRing { points, area2 })
}

fn clean_input_ring(
    ring: &Ring,
    context: &'static str,
    ring_index: usize,
    ccw: bool,
) -> NfpResult<GridRing> {
    let mut points = ring
        .vertices
        .iter()
        .copied()
        .enumerate()
        .map(|(vertex, point)| checked_grid_point(point, context, ring_index, vertex))
        .collect::<NfpResult<Vec<_>>>()?;
    remove_redundant_vertices(&mut points);
    if points.len() < 3 {
        return Err(NfpError::TooFewVertices {
            context,
            ring: ring_index,
        });
    }
    if ring_self_intersects(&points) {
        return Err(NfpError::SelfIntersection {
            context,
            ring: ring_index,
        });
    }
    let area2 = ring_area2(&points)?;
    if area2 == 0 {
        return Err(NfpError::ZeroAreaRing {
            context,
            ring: ring_index,
        });
    }
    normalize_grid_ring(points, ccw)
}

fn validated_input_components(
    input: &PolygonSet,
    context: &'static str,
) -> NfpResult<Vec<GridComponent>> {
    if input.components.is_empty() {
        return Err(NfpError::EmptyShape);
    }
    let mut ring_index = 0usize;
    let mut components = Vec::with_capacity(input.components.len());
    for (component_index, component) in input.components.iter().enumerate() {
        let outer = clean_input_ring(&component.outer, context, ring_index, true)?;
        ring_index += 1;
        let mut holes = Vec::with_capacity(component.holes.len());
        for hole in &component.holes {
            holes.push(clean_input_ring(hole, context, ring_index, false)?);
            ring_index += 1;
        }

        // A hole may touch its shell or a sibling hole at a vertex or along a
        // vertex-on-edge contact: `canonical_components_from_paths` emits
        // exactly that topology from Clipper output, and every region this
        // crate returns must be accepted again as input. Only proper edge
        // crossings and wrong-side placement are rejected.
        let mut touching = false;
        for (hole_index, hole) in holes.iter().enumerate() {
            let probe = ring_probe(&outer.points, &hole.points);
            if probe.side != Some(Location::Inside) || rings_cross(&outer.points, &hole.points) {
                return Err(NfpError::HoleOutsideShell {
                    component: component_index,
                    hole: hole_index,
                });
            }
            touching |= probe.touches;
        }
        for first in 0..holes.len() {
            for second in (first + 1)..holes.len() {
                let forward = ring_probe(&holes[first].points, &holes[second].points);
                let backward = ring_probe(&holes[second].points, &holes[first].points);
                let invalid = forward.side != Some(Location::Outside)
                    || backward.side != Some(Location::Outside)
                    || rings_cross(&holes[first].points, &holes[second].points);
                if invalid {
                    return Err(NfpError::InvalidHoleRelation {
                        component: component_index,
                        first,
                        second,
                    });
                }
                touching |= forward.touches || backward.touches;
            }
        }
        holes.sort_by(ring_cmp);
        components.push(GridComponent {
            outer,
            holes,
            touching,
        });
    }
    components.sort_by(component_cmp);
    Ok(components)
}

fn ring_cmp(first: &GridRing, second: &GridRing) -> Ordering {
    // Lexicographic over point keys, shorter-prefix first: identical to
    // comparing the collected `Vec<(i64, i64)>` keys, without allocating.
    first
        .points
        .iter()
        .copied()
        .map(point_key)
        .cmp(second.points.iter().copied().map(point_key))
}

fn component_cmp(first: &GridComponent, second: &GridComponent) -> Ordering {
    ring_cmp(&first.outer, &second.outer)
        .then_with(|| first.holes.len().cmp(&second.holes.len()))
        .then_with(|| {
            first
                .holes
                .iter()
                .zip(&second.holes)
                .find_map(|(a, b)| {
                    let ordering = ring_cmp(a, b);
                    (ordering != Ordering::Equal).then_some(ordering)
                })
                .unwrap_or(Ordering::Equal)
        })
}

fn locate_component(component: &GridComponent, point: GridPoint) -> Location {
    match point_in_ring(&component.outer.points, point) {
        Location::Outside => return Location::Outside,
        Location::Boundary => return Location::Boundary,
        Location::Inside => {}
    }
    for hole in &component.holes {
        match point_in_ring(&hole.points, point) {
            Location::Boundary => return Location::Boundary,
            Location::Inside => return Location::Outside,
            Location::Outside => {}
        }
    }
    Location::Inside
}

/// Axis-aligned bounds `(min_x, min_y, max_x, max_y)` of one ring.
fn ring_bounds(points: &[GridPoint]) -> (i64, i64, i64, i64) {
    let mut bounds = (i64::MAX, i64::MAX, i64::MIN, i64::MIN);
    for point in points {
        bounds.0 = bounds.0.min(x(*point));
        bounds.1 = bounds.1.min(y(*point));
        bounds.2 = bounds.2.max(x(*point));
        bounds.3 = bounds.3.max(y(*point));
    }
    bounds
}

fn components_interact(first: &GridComponent, second: &GridComponent) -> bool {
    // Every hole lies within its shell, so two components whose shell boxes
    // are strictly disjoint can neither touch nor contain one another.
    let first_box = ring_bounds(&first.outer.points);
    let second_box = ring_bounds(&second.outer.points);
    if first_box.2 < second_box.0
        || second_box.2 < first_box.0
        || first_box.3 < second_box.1
        || second_box.3 < first_box.1
    {
        return false;
    }
    let first_rings = std::iter::once(&first.outer).chain(&first.holes);
    let second_rings = || std::iter::once(&second.outer).chain(&second.holes);
    for first_ring in first_rings {
        for second_ring in second_rings() {
            if rings_intersect(&first_ring.points, &second_ring.points) {
                return true;
            }
        }
    }
    locate_component(first, second.outer.points[0]) != Location::Outside
        || locate_component(second, first.outer.points[0]) != Location::Outside
}

/// Union only connected groups of components whose material touches or
/// overlaps. Isolated topology is preserved directly so Clipper cannot
/// regularize away a valid minimum-area hole or disconnected component merely
/// because an unnecessary union was requested.
fn merge_interacting_components(components: Vec<GridComponent>) -> NfpResult<Vec<GridComponent>> {
    let mut visited = vec![false; components.len()];
    let mut merged = Vec::new();
    for start in 0..components.len() {
        if visited[start] {
            continue;
        }
        visited[start] = true;
        let mut stack = vec![start];
        let mut group = Vec::new();
        while let Some(current) = stack.pop() {
            group.push(current);
            for candidate in 0..components.len() {
                if !visited[candidate]
                    && components_interact(&components[current], &components[candidate])
                {
                    visited[candidate] = true;
                    stack.push(candidate);
                }
            }
        }
        if group.len() == 1 {
            merged.push(components[group[0]].clone());
            continue;
        }
        // Prove that every source component is representable before a Boolean
        // union is allowed to merge their topology. This prevents an invalid
        // minimum-scale ring from disappearing as an incidental side effect.
        //
        // A component whose holes touch their shell was produced by this
        // crate's own output canonicalization (input validation accepts that
        // topology precisely so results round-trip), so its rings are already
        // known to be representable. The proof itself cannot run on it: the
        // Boolean-equality check unions the ear-cut triangles, and Clipper
        // emits the pinch at the touch vertex as one self-touching path that
        // `clean_output_path` rejects. Skip the proof for such components.
        for index in &group {
            if components[*index].touching {
                continue;
            }
            let _ = triangulate_component(&components[*index], *index)?;
        }
        let mut paths = Vec::new();
        for index in group {
            paths.extend(components_to_paths(std::slice::from_ref(
                &components[index],
            )));
        }
        let unioned = union_paths(GridPaths::new(paths))?;
        merged.extend(canonical_components_from_paths(&unioned)?);
    }
    merged.sort_by(component_cmp);
    Ok(merged)
}

fn components_to_paths(components: &[GridComponent]) -> GridPaths {
    let mut paths = Vec::new();
    for component in components {
        paths.push(GridPath::new(component.outer.points.clone()));
        for hole in &component.holes {
            paths.push(GridPath::new(hole.points.clone()));
        }
    }
    GridPaths::new(paths)
}

fn grid_ring_to_public(ring: &GridRing) -> Ring {
    Ring::new(ring.points.iter().map(|point| {
        Point::new(
            point.x_scaled() as f64 / GRID_SCALE as f64,
            point.y_scaled() as f64 / GRID_SCALE as f64,
        )
    }))
}

fn grid_path_to_public(path: &GridPath) -> Ring {
    Ring::new(path.iter().map(|point| {
        Point::new(
            point.x_scaled() as f64 / GRID_SCALE as f64,
            point.y_scaled() as f64 / GRID_SCALE as f64,
        )
    }))
}

fn components_to_polygon_set(components: &[GridComponent]) -> PolygonSet {
    PolygonSet::new(
        components
            .iter()
            .map(|component| {
                Polygon::with_holes(
                    grid_ring_to_public(&component.outer),
                    component.holes.iter().map(grid_ring_to_public).collect(),
                )
            })
            .collect(),
    )
}

pub(crate) fn canonical_paths_from_polygon_set(
    input: &PolygonSet,
    context: &'static str,
) -> NfpResult<GridPaths> {
    let validated = validated_input_components(input, context)?;
    Ok(components_to_paths(&merge_interacting_components(
        validated,
    )?))
}

fn clean_output_path(path: &GridPath, path_index: usize) -> NfpResult<GridRing> {
    let mut points = path.iter().copied().collect::<Vec<_>>();
    remove_redundant_vertices(&mut points);
    if points.len() < 3 {
        return Err(NfpError::InvalidOutputTopology {
            reason: format!("path {path_index} has fewer than three vertices"),
        });
    }
    let area2 = ring_area2(&points)?;
    if area2 == 0 || ring_self_intersects(&points) {
        return Err(NfpError::InvalidOutputTopology {
            reason: format!("path {path_index} is degenerate or self-intersecting"),
        });
    }
    rotate_to_canonical_start(&mut points);
    Ok(GridRing { points, area2 })
}

fn canonical_components_from_paths(paths: &GridPaths) -> NfpResult<Vec<GridComponent>> {
    if paths.is_empty() {
        return Ok(Vec::new());
    }
    let mut rings = paths
        .iter()
        .enumerate()
        .map(|(index, path)| clean_output_path(path, index))
        .collect::<NfpResult<Vec<_>>>()?;

    // A ring inside (or touching) another lies within its bounding box, so a
    // candidate whose box does not enclose the child's box can be skipped
    // before any point-in-ring work.
    let bounds = rings
        .iter()
        .map(|ring| ring_bounds(&ring.points))
        .collect::<Vec<_>>();
    let mut parent = vec![None; rings.len()];
    for child in 0..rings.len() {
        let child_area = rings[child].area2.unsigned_abs();
        let child_box = bounds[child];
        let probe = rings[child].points[0];
        let mut best: Option<(usize, u128)> = None;
        for candidate in 0..rings.len() {
            if candidate == child || rings[candidate].area2.unsigned_abs() <= child_area {
                continue;
            }
            let candidate_box = bounds[candidate];
            if candidate_box.0 > child_box.0
                || candidate_box.1 > child_box.1
                || candidate_box.2 < child_box.2
                || candidate_box.3 < child_box.3
            {
                continue;
            }
            match point_in_ring(&rings[candidate].points, probe) {
                Location::Inside => {
                    let area = rings[candidate].area2.unsigned_abs();
                    if best.is_none_or(|(_, best_area)| area < best_area) {
                        best = Some((candidate, area));
                    }
                }
                Location::Boundary => {
                    // A Clipper union may retain point-touching outer paths.
                    // Another non-boundary vertex determines containment; if
                    // none does, these are sibling touching components.
                    let mut relation = Location::Boundary;
                    for point in &rings[child].points {
                        relation = point_in_ring(&rings[candidate].points, *point);
                        if relation != Location::Boundary {
                            break;
                        }
                    }
                    if relation == Location::Inside {
                        let area = rings[candidate].area2.unsigned_abs();
                        if best.is_none_or(|(_, best_area)| area < best_area) {
                            best = Some((candidate, area));
                        }
                    }
                }
                Location::Outside => {}
            }
        }
        parent[child] = best.map(|(index, _)| index);
    }

    let mut depth = vec![0usize; rings.len()];
    for index in 0..rings.len() {
        let mut cursor = parent[index];
        let mut seen = 0usize;
        while let Some(parent_index) = cursor {
            seen += 1;
            if seen > rings.len() {
                return Err(NfpError::InvalidOutputTopology {
                    reason: "ring containment cycle".to_owned(),
                });
            }
            cursor = parent[parent_index];
        }
        depth[index] = seen;
    }

    let mut components = Vec::new();
    for outer_index in 0..rings.len() {
        if depth[outer_index] % 2 != 0 {
            continue;
        }
        let outer = normalize_grid_ring(rings[outer_index].points.clone(), true)?;
        let mut holes = Vec::new();
        for hole_index in 0..rings.len() {
            if parent[hole_index] == Some(outer_index) && depth[hole_index] % 2 == 1 {
                holes.push(normalize_grid_ring(
                    rings[hole_index].points.clone(),
                    false,
                )?);
            }
        }
        holes.sort_by(ring_cmp);
        // Only validated input components feed `merge_interacting_components`,
        // so the touching flag is not needed on canonicalized output.
        components.push(GridComponent {
            outer,
            holes,
            touching: false,
        });
    }
    components.sort_by(component_cmp);
    rings.clear();
    Ok(components)
}

pub(crate) fn paths_to_polygon_set(paths: &GridPaths) -> NfpResult<PolygonSet> {
    Ok(components_to_polygon_set(&canonical_components_from_paths(
        paths,
    )?))
}

fn canonicalize_paths(paths: &GridPaths) -> NfpResult<GridPaths> {
    Ok(components_to_paths(&canonical_components_from_paths(
        paths,
    )?))
}

fn boolean_error(operation: &'static str, error: impl std::fmt::Display) -> NfpError {
    NfpError::Boolean {
        operation,
        message: error.to_string(),
    }
}

/// Above this many input paths, union in chunks and then union the partial
/// results instead of handing everything to one sweep.
///
/// A pair NFP feeds `pieces(fixed) * pieces(moving)` heavily overlapping convex
/// atoms into a single union. On a 156-panel garment marker that reached 4489
/// atoms for one pair and the single sweep took ~1.5 s, because the number of
/// edge-edge intersections a sweepline must resolve grows far faster than the
/// input size when the input is this densely overlapping. Unioning in chunks
/// collapses most of that overlap early, so each later sweep sees far simpler
/// geometry.
const UNION_CHUNK_THRESHOLD: usize = 64;

/// Paths per chunk once [`UNION_CHUNK_THRESHOLD`] is exceeded.
const UNION_CHUNK_SIZE: usize = 32;

pub(crate) fn union_paths(paths: GridPaths) -> NfpResult<GridPaths> {
    if paths.is_empty() {
        return Ok(paths);
    }
    if paths.len() <= UNION_CHUNK_THRESHOLD {
        return union_paths_single(paths);
    }

    // Chunk boundaries follow the caller's path order, which is already
    // canonical (`sort_by(path_cmp)`), so the reduction is deterministic.
    let mut level: Vec<GridPath> = paths.iter().cloned().collect();
    while level.len() > UNION_CHUNK_THRESHOLD {
        let mut next: Vec<GridPath> = Vec::with_capacity(level.len() / UNION_CHUNK_SIZE + 1);
        for chunk in level.chunks(UNION_CHUNK_SIZE) {
            let unioned = union_paths_single(GridPaths::new(chunk.to_vec()))?;
            next.extend(unioned.iter().cloned());
        }
        // A level that cannot shrink would loop forever; finish it in one sweep.
        if next.len() >= level.len() {
            return union_paths_single(GridPaths::new(next));
        }
        level = next;
    }
    union_paths_single(GridPaths::new(level))
}

fn union_paths_single(paths: GridPaths) -> NfpResult<GridPaths> {
    if paths.is_empty() {
        return Ok(paths);
    }
    let result = paths
        .to_clipper_subject()
        .add_clip(GridPaths::default())
        .union(FillRule::NonZero)
        .map_err(|error| boolean_error("union", error))?;
    canonicalize_paths(&result)
}

pub(crate) fn difference_paths(subject: &GridPaths, clip: &GridPaths) -> NfpResult<GridPaths> {
    if subject.is_empty() {
        return Ok(GridPaths::default());
    }
    if clip.is_empty() {
        return union_paths(subject.clone());
    }
    let result = subject
        .to_clipper_subject()
        .add_clip(clip.clone())
        .difference(FillRule::NonZero)
        .map_err(|error| boolean_error("difference", error))?;
    canonicalize_paths(&result)
}

fn projection_interval(
    triangle: &GridPath,
    axis_x: i128,
    axis_y: i128,
    offset_x: i64,
    offset_y: i64,
) -> (i128, i128) {
    let mut projections = triangle.iter().map(|point| {
        (i128::from(point.x_scaled()) + i128::from(offset_x)) * axis_x
            + (i128::from(point.y_scaled()) + i128::from(offset_y)) * axis_y
    });
    let first = projections
        .next()
        .expect(crate::dmsg!("prepared material triangles always contain three vertices"));
    projections.fold((first, first), |(min, max), projection| {
        (min.min(projection), max.max(projection))
    })
}

fn convex_interiors_overlap(
    fixed: &GridPath,
    moving: &GridPath,
    moving_dx: i64,
    moving_dy: i64,
) -> bool {
    for axis_source in [fixed, moving] {
        let axis_points = axis_source.iter().as_slice();
        for edge_index in 0..axis_points.len() {
            let a = axis_points[edge_index];
            let b = axis_points[(edge_index + 1) % axis_points.len()];
            let axis_x = -(i128::from(b.y_scaled()) - i128::from(a.y_scaled()));
            let axis_y = i128::from(b.x_scaled()) - i128::from(a.x_scaled());
            if axis_x == 0 && axis_y == 0 {
                continue;
            }
            let (fixed_min, fixed_max) = projection_interval(fixed, axis_x, axis_y, 0, 0);
            let (moving_min, moving_max) =
                projection_interval(moving, axis_x, axis_y, moving_dx, moving_dy);
            if fixed_max <= moving_min || moving_max <= fixed_min {
                return false;
            }
        }
    }
    true
}

/// Classify two positioned filled material sets without Boolean intersection
/// rounding. Strict triangle SAT detects positive-area overlap entirely with
/// i128 arithmetic; exact segment predicates retain zero-area contact.
pub(crate) fn classify_paths(
    fixed: &GridPaths,
    positioned_moving: &GridPaths,
    fixed_triangles: &[GridPath],
    moving_triangles: &[GridPath],
    moving_dx: i64,
    moving_dy: i64,
) -> PairRelation {
    for fixed_triangle in fixed_triangles {
        for moving_triangle in moving_triangles {
            if convex_interiors_overlap(fixed_triangle, moving_triangle, moving_dx, moving_dy) {
                return PairRelation::Overlap;
            }
        }
    }
    for fixed_ring in fixed.iter() {
        let fixed_points = fixed_ring.iter().as_slice();
        for moving_ring in positioned_moving.iter() {
            let moving_points = moving_ring.iter().as_slice();
            if rings_intersect(fixed_points, moving_points) {
                return PairRelation::Contact;
            }
        }
    }
    PairRelation::Disjoint
}

fn checked_coordinate(value: i128, stage: &'static str) -> NfpResult<i64> {
    if value.unsigned_abs() > MAX_DERIVED_GRID_COORD as u128 {
        return Err(NfpError::CoordinateOverflow {
            stage,
            value,
            limit: MAX_DERIVED_GRID_COORD,
        });
    }
    Ok(value as i64)
}

pub(crate) fn translate_paths(paths: &GridPaths, dx: i64, dy: i64) -> NfpResult<GridPaths> {
    let mut translated = Vec::with_capacity(paths.len());
    for path in paths.iter() {
        let points = path
            .iter()
            .map(|point| {
                let translated_x = checked_coordinate(
                    i128::from(point.x_scaled()) + i128::from(dx),
                    "translation",
                )?;
                let translated_y = checked_coordinate(
                    i128::from(point.y_scaled()) + i128::from(dy),
                    "translation",
                )?;
                Ok(GridPoint::from_scaled(translated_x, translated_y))
            })
            .collect::<NfpResult<Vec<_>>>()?;
        translated.push(GridPath::new(points));
    }
    Ok(GridPaths::new(translated))
}

pub(crate) fn reflect_paths(paths: &GridPaths) -> NfpResult<GridPaths> {
    let reflected = map_paths(paths, "point reflection", |px, py| {
        (-i128::from(px), -i128::from(py))
    })?;
    canonicalize_paths(&reflected)
}

fn map_paths(
    paths: &GridPaths,
    stage: &'static str,
    mut transform: impl FnMut(i64, i64) -> (i128, i128),
) -> NfpResult<GridPaths> {
    let mut transformed = Vec::with_capacity(paths.len());
    for path in paths.iter() {
        let mut points = Vec::with_capacity(path.len());
        for point in path.iter() {
            let (tx, ty) = transform(point.x_scaled(), point.y_scaled());
            points.push(GridPoint::from_scaled(
                checked_coordinate(tx, stage)?,
                checked_coordinate(ty, stage)?,
            ));
        }
        transformed.push(GridPath::new(points));
    }
    Ok(GridPaths::new(transformed))
}

pub(crate) fn transform_paths(paths: &GridPaths, orientation: Orientation) -> NfpResult<GridPaths> {
    const FULL_TURN: i64 = 360_000_000;
    const QUARTER_TURN: i64 = 90_000_000;
    let angle = orientation.angle_microdegrees.rem_euclid(FULL_TURN);

    if angle % QUARTER_TURN == 0 {
        let quadrant = angle / QUARTER_TURN;
        return map_paths(paths, "orientation", |px, py| {
            let px = if orientation.mirror_x {
                -i128::from(px)
            } else {
                i128::from(px)
            };
            let py = i128::from(py);
            match quadrant {
                0 => (px, py),
                1 => (-py, px),
                2 => (-px, -py),
                3 => (py, -px),
                _ => unreachable!(),
            }
        });
    }

    let radians = angle as f64 * std::f64::consts::PI / 180_000_000.0;
    if !radians.is_finite() {
        return Err(NfpError::InvalidOrientation {
            reason: "angle is not finite",
        });
    }
    let (sin, cos) = radians.sin_cos();
    map_paths(paths, "orientation", |px, py| {
        let px = if orientation.mirror_x { -px } else { px } as f64;
        let py = py as f64;
        let tx = (px * cos - py * sin).round();
        let ty = (px * sin + py * cos).round();
        (tx as i128, ty as i128)
    })
}

/// Transform canonical public topology without unioning it first. Keeping
/// every intended shell, hole, and component separate lets input validation
/// reject a ring that collapses or changes relation after grid rounding.
pub(crate) fn transform_polygon_set(
    input: &PolygonSet,
    orientation: Orientation,
) -> NfpResult<PolygonSet> {
    // Validation sorts components and holes into canonical order; walk that
    // same validated sequence so every hole is zipped with its own shell.
    let validated = validated_input_components(input, "orientation source")?;
    let source = components_to_paths(&validated);
    let transformed = transform_paths(&source, orientation)?;
    let mut paths = transformed.iter();
    let mut components = Vec::with_capacity(validated.len());
    for component in &validated {
        let outer = paths.next().map(grid_path_to_public).ok_or_else(|| {
            NfpError::InvalidOutputTopology {
                reason: "orientation lost an outer ring".to_owned(),
            }
        })?;
        let mut holes = Vec::with_capacity(component.holes.len());
        for _ in &component.holes {
            holes.push(paths.next().map(grid_path_to_public).ok_or_else(|| {
                NfpError::InvalidOutputTopology {
                    reason: "orientation lost a hole ring".to_owned(),
                }
            })?);
        }
        components.push(Polygon::with_holes(outer, holes));
    }
    if paths.next().is_some() {
        return Err(NfpError::InvalidOutputTopology {
            reason: "orientation produced unexpected extra rings".to_owned(),
        });
    }
    Ok(PolygonSet::new(components))
}

pub(crate) fn bounds_scaled(paths: &GridPaths) -> NfpResult<(i64, i64, i64, i64)> {
    let mut points = paths.iter().flat_map(|path| path.iter());
    let first = points.next().ok_or(NfpError::EmptyShape)?;
    let (mut min_x, mut min_y, mut max_x, mut max_y) = (
        first.x_scaled(),
        first.y_scaled(),
        first.x_scaled(),
        first.y_scaled(),
    );
    for point in points {
        min_x = min_x.min(point.x_scaled());
        min_y = min_y.min(point.y_scaled());
        max_x = max_x.max(point.x_scaled());
        max_y = max_y.max(point.y_scaled());
    }
    Ok((min_x, min_y, max_x, max_y))
}

pub(crate) fn rect_paths(min_x: i64, min_y: i64, max_x: i64, max_y: i64) -> NfpResult<GridPaths> {
    if min_x >= max_x || min_y >= max_y {
        return Ok(GridPaths::default());
    }
    for value in [min_x, min_y, max_x, max_y] {
        checked_coordinate(i128::from(value), "rectangle")?;
    }
    Ok(GridPath::new(vec![
        GridPoint::from_scaled(min_x, min_y),
        GridPoint::from_scaled(max_x, min_y),
        GridPoint::from_scaled(max_x, max_y),
        GridPoint::from_scaled(min_x, max_y),
    ])
    .into())
}

pub(crate) fn locate_paths(paths: &GridPaths, px: i64, py: i64) -> NfpResult<Location> {
    let point = GridPoint::from_scaled(px, py);
    let mut winding = 0i32;
    for path in paths.iter() {
        let points = path.iter().as_slice();
        match point_in_ring(points, point) {
            Location::Boundary => return Ok(Location::Boundary),
            Location::Inside => {
                winding += if ring_area2(points)? > 0 { 1 } else { -1 };
            }
            Location::Outside => {}
        }
    }
    if winding == 0 {
        Ok(Location::Outside)
    } else {
        Ok(Location::Inside)
    }
}

fn convex_hull(mut points: Vec<GridPoint>) -> NfpResult<GridPath> {
    points.sort_by_key(|point| point_key(*point));
    points.dedup();
    if points.len() < 3 {
        return Err(NfpError::EmptyNfp);
    }
    let mut lower = Vec::new();
    for point in &points {
        while lower.len() >= 2
            && orient(lower[lower.len() - 2], lower[lower.len() - 1], *point) <= 0
        {
            lower.pop();
        }
        lower.push(*point);
    }
    let mut upper = Vec::new();
    for point in points.iter().rev() {
        while upper.len() >= 2
            && orient(upper[upper.len() - 2], upper[upper.len() - 1], *point) <= 0
        {
            upper.pop();
        }
        upper.push(*point);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    if lower.len() < 3 || ring_area2(&lower)? == 0 {
        return Err(NfpError::EmptyNfp);
    }
    rotate_to_canonical_start(&mut lower);
    Ok(GridPath::new(lower))
}

fn convex_atom_sum(
    first: &GridPath,
    second: &GridPath,
    reflect_second: bool,
) -> NfpResult<GridPath> {
    let mut sums = Vec::with_capacity(first.len().saturating_mul(second.len()));
    for first_point in first.iter() {
        for second_point in second.iter() {
            let second_x = i128::from(second_point.x_scaled());
            let second_y = i128::from(second_point.y_scaled());
            let sx = checked_coordinate(
                i128::from(first_point.x_scaled())
                    + if reflect_second { -second_x } else { second_x },
                if reflect_second {
                    "NFP atom sum"
                } else {
                    "Minkowski atom sum"
                },
            )?;
            let sy = checked_coordinate(
                i128::from(first_point.y_scaled())
                    + if reflect_second { -second_y } else { second_y },
                if reflect_second {
                    "NFP atom sum"
                } else {
                    "Minkowski atom sum"
                },
            )?;
            sums.push(GridPoint::from_scaled(sx, sy));
        }
    }
    convex_hull(sums)
}

#[allow(dead_code)]
pub(crate) fn convex_nfp_atom(fixed: &GridPath, moving: &GridPath) -> NfpResult<GridPath> {
    convex_atom_sum(fixed, moving, true)
}

#[allow(dead_code)]
pub(crate) fn convex_minkowski_atom(first: &GridPath, second: &GridPath) -> NfpResult<GridPath> {
    convex_atom_sum(first, second, false)
}

fn atom_primitives(
    first: &[GridPath],
    second: &[GridPath],
    reflect_second: bool,
) -> NfpResult<GridPaths> {
    if first.is_empty() || second.is_empty() {
        return Err(NfpError::EmptyNfp);
    }
    let mut atom_sums = Vec::with_capacity(first.len().saturating_mul(second.len()));
    for first_atom in first {
        for second_atom in second {
            atom_sums.push(convex_atom_sum(first_atom, second_atom, reflect_second)?);
        }
    }
    atom_sums.sort_by(path_cmp);
    Ok(GridPaths::new(atom_sums))
}

fn combine_atoms(
    first: &[GridPath],
    second: &[GridPath],
    reflect_second: bool,
) -> NfpResult<GridPaths> {
    // Clipper's integer intersection rounding can be sensitive to insertion
    // order. Canonical primitive ordering makes compact contour generation
    // deterministic independently of triangulation traversal order.
    let result = union_paths(atom_primitives(first, second, reflect_second)?)?;
    if result.is_empty() {
        Err(NfpError::EmptyNfp)
    } else {
        Ok(result)
    }
}

/// Exact convex primitives whose set union is `fixed + (-moving)`. No Boolean
/// contour rounding is applied to this representation.
pub(crate) fn nfp_atom_primitives(fixed: &[GridPath], moving: &[GridPath]) -> NfpResult<GridPaths> {
    atom_primitives(fixed, moving, true)
}

/// Exact convex primitives whose set union is `first + second`.
pub(crate) fn minkowski_atom_primitives(
    first: &[GridPath],
    second: &[GridPath],
) -> NfpResult<GridPaths> {
    atom_primitives(first, second, false)
}

/// Point-reflect convex primitives produced by [`atom_primitives`].
///
/// Every input ring is a strictly convex CCW hull in canonical form. Point
/// reflection preserves orientation, convexity and the cyclic vertex order,
/// so re-running the hull is unnecessary: negating and rotating to the
/// canonical start yields exactly the hull of the negated vertex set.
pub(crate) fn reflect_convex_primitives(paths: &GridPaths) -> NfpResult<GridPaths> {
    let mut reflected = Vec::with_capacity(paths.len());
    for path in paths.iter() {
        let mut points = path
            .iter()
            .map(|point| {
                Ok(GridPoint::from_scaled(
                    checked_coordinate(-i128::from(point.x_scaled()), "NFP primitive reflection")?,
                    checked_coordinate(-i128::from(point.y_scaled()), "NFP primitive reflection")?,
                ))
            })
            .collect::<NfpResult<Vec<_>>>()?;
        debug_assert!(
            points.len() >= 3 && is_convex_ccw_ring(&points),
            "convex primitives must be convex CCW rings"
        );
        rotate_to_canonical_start(&mut points);
        reflected.push(GridPath::new(points));
    }
    reflected.sort_by(path_cmp);
    Ok(GridPaths::new(reflected))
}

pub(crate) fn primitive_paths_to_polygon_set(paths: &GridPaths) -> PolygonSet {
    PolygonSet::new(
        paths
            .iter()
            .map(|path| Polygon::new(grid_path_to_public(path)))
            .collect(),
    )
}

pub(crate) fn locate_primitive_union(paths: &GridPaths, px: i64, py: i64) -> NfpResult<Location> {
    let point = GridPoint::from_scaled(px, py);
    let mut touches = false;
    for path in paths.iter() {
        match point_in_ring(path.iter().as_slice(), point) {
            Location::Inside => return Ok(Location::Inside),
            Location::Boundary => touches = true,
            Location::Outside => {}
        }
    }
    Ok(if touches {
        Location::Boundary
    } else {
        Location::Outside
    })
}

/// Filled configuration obstacle `fixed + (-moving)`.
pub(crate) fn nfp_from_atoms(fixed: &[GridPath], moving: &[GridPath]) -> NfpResult<GridPaths> {
    combine_atoms(fixed, moving, true)
}

/// True filled Minkowski sum `first + second`.
pub(crate) fn minkowski_sum_atoms(first: &[GridPath], second: &[GridPath]) -> NfpResult<GridPaths> {
    combine_atoms(first, second, false)
}

fn triangulate_component(
    component: &GridComponent,
    component_index: usize,
) -> NfpResult<Vec<GridPath>> {
    let mut flat_points = component.outer.points.clone();
    let mut vertices = Vec::with_capacity(flat_points.len() * 2);
    for point in &flat_points {
        vertices.extend([point.x_scaled() as f64, point.y_scaled() as f64]);
    }
    let mut hole_indices = Vec::with_capacity(component.holes.len());
    for hole in &component.holes {
        hole_indices.push(flat_points.len());
        flat_points.extend(hole.points.iter().copied());
        for point in &hole.points {
            vertices.extend([point.x_scaled() as f64, point.y_scaled() as f64]);
        }
    }
    let indices =
        earcutr::earcut(&vertices, &hole_indices, 2).map_err(|error| NfpError::Triangulation {
            component: component_index,
            reason: error.to_string(),
        })?;
    if indices.len() % 3 != 0 || indices.is_empty() {
        return Err(NfpError::Triangulation {
            component: component_index,
            reason: "earcut returned an incomplete or empty index list".to_owned(),
        });
    }
    let mut triangles = Vec::with_capacity(indices.len() / 3);
    let mut actual_area2 = 0i128;
    for chunk in indices.chunks_exact(3) {
        let mut triangle = vec![
            flat_points[chunk[0]],
            flat_points[chunk[1]],
            flat_points[chunk[2]],
        ];
        let mut area2 = ring_area2(&triangle)?;
        if area2 == 0 {
            return Err(NfpError::Triangulation {
                component: component_index,
                reason: "earcut returned a zero-area triangle".to_owned(),
            });
        }
        if area2 < 0 {
            triangle.swap(1, 2);
            area2 = -area2;
        }
        rotate_to_canonical_start(&mut triangle);
        actual_area2 = actual_area2
            .checked_add(area2)
            .ok_or(NfpError::CoordinateOverflow {
                stage: "triangulation area accumulation",
                value: i128::MAX,
                limit: MAX_DERIVED_GRID_COORD,
            })?;
        triangles.push(GridPath::new(triangle));
    }
    let expected_area2 = component
        .holes
        .iter()
        .try_fold(component.outer.area2, |area, hole| {
            area.checked_add(hole.area2)
        })
        .ok_or(NfpError::CoordinateOverflow {
            stage: "component area accumulation",
            value: i128::MAX,
            limit: MAX_DERIVED_GRID_COORD,
        })?;
    if expected_area2 != actual_area2 {
        return Err(NfpError::TriangulationAreaMismatch {
            component: component_index,
            expected: expected_area2,
            actual: actual_area2,
        });
    }

    let component_paths = components_to_paths(std::slice::from_ref(component));
    let triangle_union = union_paths(GridPaths::new(triangles.clone()))?;
    if !difference_paths(&component_paths, &triangle_union)?.is_empty()
        || !difference_paths(&triangle_union, &component_paths)?.is_empty()
    {
        return Err(NfpError::Triangulation {
            component: component_index,
            reason: "triangle union is not Boolean-equal to the component".to_owned(),
        });
    }
    // Merge the triangulation into larger convex pieces.
    //
    // The pair-NFP atom count is `pieces(fixed) * pieces(moving)`, so the
    // triangulation - the *finest* convex decomposition - is also the most
    // expensive one. A 104-triangle garment panel against a 101-triangle one
    // produced 10504 atoms, and unioning that many heavily overlapping convex
    // polygons took 6.6 s for a single pair. Merging first is exact (the pieces
    // still tile the component) and cuts the atom count quadratically.
    let mut merged = merge_convex_pieces(triangles)?;
    merged.sort_by(path_cmp);
    Ok(merged)
}

/// Merge adjacent convex pieces across shared edges while the result stays
/// convex (Hertel-Mehlhorn).
///
/// The input tiles a component exactly and every piece is a CCW convex ring on
/// the integer grid. Merging two pieces that share exactly one edge preserves
/// that invariant, so the union of the output is identical to the union of the
/// input - only the decomposition is coarser. Every predicate is exact i128
/// arithmetic and the traversal order is deterministic, so the result is
/// reproducible across runs and platforms.
fn merge_convex_pieces(pieces: Vec<GridPath>) -> NfpResult<Vec<GridPath>> {
    if pieces.len() < 2 {
        return Ok(pieces);
    }

    let mut rings: Vec<Option<Vec<GridPoint>>> = pieces
        .into_iter()
        .map(|path| Some(path.iter().copied().collect::<Vec<_>>()))
        .collect();

    // Directed edge (a -> b) -> owning piece. Neighbouring CCW pieces traverse
    // a shared edge in opposite directions, so the neighbour of edge a -> b is
    // whoever owns b -> a.
    let mut owner: HashMap<(i64, i64, i64, i64), usize> = HashMap::new();
    for (index, ring) in rings.iter().enumerate() {
        let Some(ring) = ring else { continue };
        for position in 0..ring.len() {
            let a = ring[position];
            let b = ring[(position + 1) % ring.len()];
            owner.insert(directed_edge_key(a, b), index);
        }
    }

    let mut changed = true;
    while changed {
        changed = false;
        for index in 0..rings.len() {
            loop {
                let Some(ring) = rings[index].clone() else {
                    break;
                };
                let mut merged_one = false;
                for position in 0..ring.len() {
                    let a = ring[position];
                    let b = ring[(position + 1) % ring.len()];
                    let Some(&other) = owner.get(&directed_edge_key(b, a)) else {
                        continue;
                    };
                    if other == index {
                        continue;
                    }
                    let Some(other_ring) = rings[other].clone() else {
                        continue;
                    };
                    let Some(candidate) = splice_across_edge(&ring, position, &other_ring, a, b)?
                    else {
                        continue;
                    };

                    for source in [&ring, &other_ring] {
                        for edge in 0..source.len() {
                            let p = source[edge];
                            let q = source[(edge + 1) % source.len()];
                            owner.remove(&directed_edge_key(p, q));
                        }
                    }
                    for edge in 0..candidate.len() {
                        let p = candidate[edge];
                        let q = candidate[(edge + 1) % candidate.len()];
                        owner.insert(directed_edge_key(p, q), index);
                    }
                    rings[other] = None;
                    rings[index] = Some(candidate);
                    merged_one = true;
                    changed = true;
                    break;
                }
                if !merged_one {
                    break;
                }
            }
        }
    }

    let mut out = Vec::with_capacity(rings.len());
    for ring in rings.into_iter().flatten() {
        let mut ring = ring;
        rotate_to_canonical_start(&mut ring);
        out.push(GridPath::new(ring));
    }
    Ok(out)
}

/// Key for one directed grid edge.
#[inline]
fn directed_edge_key(a: GridPoint, b: GridPoint) -> (i64, i64, i64, i64) {
    (a.x_scaled(), a.y_scaled(), b.x_scaled(), b.y_scaled())
}

/// Splice `other` into `ring` across the shared edge `a -> b`, returning the
/// merged ring only when it is a valid convex piece of the same total area.
///
/// Returns `Ok(None)` whenever the merge is not admissible - a non-convex
/// result, a repeated vertex (the two pieces shared more than one edge), or an
/// area mismatch. Rejecting is always safe: the caller then keeps both pieces.
fn splice_across_edge(
    ring: &[GridPoint],
    position: usize,
    other: &[GridPoint],
    a: GridPoint,
    b: GridPoint,
) -> NfpResult<Option<Vec<GridPoint>>> {
    let Some(other_position) = (0..other.len()).find(|&index| {
        let p = other[index];
        let q = other[(index + 1) % other.len()];
        point_key(p) == point_key(b) && point_key(q) == point_key(a)
    }) else {
        return Ok(None);
    };

    // Walk `ring` from b all the way round to a, then the neighbour strictly
    // between a and b. Both shared endpoints appear exactly once.
    let mut merged = Vec::with_capacity(ring.len() + other.len());
    for offset in 0..ring.len() {
        let index = (position + 1 + offset) % ring.len();
        merged.push(ring[index]);
        if index == position {
            break;
        }
    }
    let skip_start = (other_position + 2) % other.len();
    let interior = other.len().saturating_sub(2);
    for offset in 0..interior {
        merged.push(other[(skip_start + offset) % other.len()]);
    }

    if merged.len() < 3 {
        return Ok(None);
    }
    let mut seen = HashSet::with_capacity(merged.len());
    for point in &merged {
        if !seen.insert(point_key(*point)) {
            return Ok(None);
        }
    }

    let area2 = ring_area2(&merged)?;
    let expected = ring_area2(ring)?
        .checked_add(ring_area2(other)?)
        .ok_or(NfpError::CoordinateOverflow {
            stage: "convex merge area accumulation",
            value: i128::MAX,
            limit: MAX_DERIVED_GRID_COORD,
        })?;
    if area2 != expected || area2 <= 0 {
        return Ok(None);
    }
    if !is_convex_ccw_ring(&merged) {
        return Ok(None);
    }
    Ok(Some(merged))
}

/// Exact convexity test for a CCW ring. Collinear vertices stay convex; they
/// only add a redundant vertex.
fn is_convex_ccw_ring(points: &[GridPoint]) -> bool {
    if points.len() < 3 {
        return false;
    }
    for index in 0..points.len() {
        let a = points[index];
        let b = points[(index + 1) % points.len()];
        let c = points[(index + 2) % points.len()];
        if orient(a, b, c) < 0 {
            return false;
        }
    }
    true
}

fn path_cmp(first: &GridPath, second: &GridPath) -> Ordering {
    first
        .iter()
        .copied()
        .map(point_key)
        .cmp(second.iter().copied().map(point_key))
}

/// Total order over canonical geometry. Pair NFP construction uses one
/// operand order so `NFP(A,B) = -NFP(B,A)` cannot drift through
/// operand-order-dependent Boolean intersection rounding.
pub(crate) fn canonical_paths_cmp(first: &GridPaths, second: &GridPaths) -> Ordering {
    for (first_path, second_path) in first.iter().zip(second.iter()) {
        let ordering = path_cmp(first_path, second_path);
        if ordering != Ordering::Equal {
            return ordering;
        }
    }
    first.len().cmp(&second.len())
}

fn prepared_geometry_from_components(
    components: Vec<GridComponent>,
) -> NfpResult<PreparedGeometry> {
    if components.is_empty() {
        return Err(NfpError::EmptyShape);
    }
    let canonical_paths = components_to_paths(&components);
    let geometry = components_to_polygon_set(&components);
    let mut triangles = Vec::new();
    for (index, component) in components.iter().enumerate() {
        triangles.extend(triangulate_component(component, index)?);
    }
    triangles.sort_by(path_cmp);

    let area2 = components
        .iter()
        .try_fold(0i128, |set_area, component| {
            let component_area = component
                .holes
                .iter()
                .try_fold(component.outer.area2, |area, hole| {
                    area.checked_add(hole.area2)
                })?;
            set_area.checked_add(component_area)
        })
        .ok_or(NfpError::CoordinateOverflow {
            stage: "shape area accumulation",
            value: i128::MAX,
            limit: MAX_DERIVED_GRID_COORD,
        })?;
    let (min_x, min_y, max_x, max_y) = bounds_scaled(&canonical_paths)?;
    let max_abs = canonical_paths
        .iter()
        .flat_map(|path| path.iter())
        .map(|point| {
            point
                .x_scaled()
                .unsigned_abs()
                .max(point.y_scaled().unsigned_abs())
        })
        .max()
        .unwrap_or(0) as i64;
    Ok(PreparedGeometry {
        geometry,
        canonical_paths,
        triangles,
        bounds: Bounds {
            min: Point::new(
                min_x as f64 / GRID_SCALE as f64,
                min_y as f64 / GRID_SCALE as f64,
            ),
            max: Point::new(
                max_x as f64 / GRID_SCALE as f64,
                max_y as f64 / GRID_SCALE as f64,
            ),
        },
        area2,
        max_abs,
    })
}

pub(crate) fn prepare_geometry_from_paths(
    paths: GridPaths,
    _context: &'static str,
) -> NfpResult<PreparedGeometry> {
    let canonical_paths = union_paths(paths)?;
    if canonical_paths.is_empty() {
        return Err(NfpError::EmptyShape);
    }
    prepared_geometry_from_components(canonical_components_from_paths(&canonical_paths)?)
}

pub(crate) fn prepare_geometry_from_polygon_set(
    input: &PolygonSet,
    context: &'static str,
) -> NfpResult<PreparedGeometry> {
    let validated = validated_input_components(input, context)?;
    prepared_geometry_from_components(merge_interacting_components(validated)?)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Ring {
        Ring::new([
            (min_x, min_y),
            (max_x, min_y),
            (max_x, max_y),
            (min_x, max_y),
        ])
    }

    #[test]
    fn cleans_and_unions_touching_components() {
        let input = PolygonSet::new(vec![
            Polygon::new(square(0.0, 0.0, 1.0, 1.0)),
            Polygon::new(square(1.0, 0.0, 2.0, 1.0)),
        ]);
        let prepared = prepare_geometry_from_polygon_set(&input, "test").unwrap();
        assert_eq!(prepared.geometry.components.len(), 1);
        assert_eq!(prepared.area2, 4_000_000);
        // The union is a 2x1 rectangle, so the convex-merge pass collapses the
        // two ear-cut triangles back into a single convex piece.
        assert_eq!(prepared.triangles.len(), 1);
        assert_eq!(pieces_area2(&prepared.triangles), prepared.area2);
    }

    /// Total signed area of a convex decomposition.
    fn pieces_area2(pieces: &[GridPath]) -> i128 {
        pieces
            .iter()
            .map(|piece| {
                let points = piece.iter().copied().collect::<Vec<_>>();
                ring_area2(&points).unwrap()
            })
            .sum()
    }

    /// Every emitted piece must stay convex and CCW - `combine_atoms` takes the
    /// convex hull of vertex differences and is only exact for convex operands.
    fn assert_pieces_are_convex(pieces: &[GridPath]) {
        for piece in pieces {
            let points = piece.iter().copied().collect::<Vec<_>>();
            assert!(points.len() >= 3, "a piece must have at least 3 vertices");
            assert!(
                ring_area2(&points).unwrap() > 0,
                "a piece must be CCW with positive area"
            );
            assert!(
                is_convex_ccw_ring(&points),
                "the merge pass must only emit convex pieces: {points:?}"
            );
        }
    }

    #[test]
    fn convex_merge_keeps_area_and_convexity_on_a_concave_shape() {
        // L shape: concave, so it cannot collapse to one piece.
        let outer = Ring::new([
            (0.0, 0.0),
            (30.0, 0.0),
            (30.0, 10.0),
            (10.0, 10.0),
            (10.0, 30.0),
            (0.0, 30.0),
        ]);
        let prepared =
            prepare_geometry_from_polygon_set(&PolygonSet::from_outer(outer), "test").unwrap();

        assert_pieces_are_convex(&prepared.triangles);
        assert_eq!(
            pieces_area2(&prepared.triangles),
            prepared.area2,
            "the merged pieces must still tile the component exactly"
        );
        // An L decomposes into exactly two convex rectangles; ear cutting alone
        // would leave four triangles.
        assert_eq!(prepared.triangles.len(), 2);
    }

    #[test]
    fn convex_merge_preserves_holes() {
        let prepared = prepare_geometry_from_polygon_set(
            &PolygonSet::new(vec![Polygon::with_holes(
                square(0.0, 0.0, 10.0, 10.0),
                vec![square(2.0, 2.0, 8.0, 8.0)],
            )]),
            "test",
        )
        .unwrap();

        assert_pieces_are_convex(&prepared.triangles);
        assert_eq!(
            pieces_area2(&prepared.triangles),
            prepared.area2,
            "a holed component must not gain area through merging"
        );
    }

    #[test]
    fn convex_merge_never_grows_the_piece_count() {
        // A staircase forces many ear-cut triangles; merging must only ever
        // reduce their number.
        let mut vertices = vec![(0.0, 0.0)];
        for step in 0..8 {
            let x = (step + 1) as f64 * 4.0;
            let y = (step + 1) as f64 * 4.0;
            vertices.push((x, y - 4.0));
            vertices.push((x, y));
        }
        vertices.push((0.0, 32.0));
        let prepared =
            prepare_geometry_from_polygon_set(&PolygonSet::from_outer(Ring::new(vertices)), "test")
                .unwrap();

        assert_pieces_are_convex(&prepared.triangles);
        assert_eq!(pieces_area2(&prepared.triangles), prepared.area2);
        let n_vertices = prepared.canonical_paths.iter().map(|p| p.len()).sum::<usize>();
        assert!(
            prepared.triangles.len() <= n_vertices.saturating_sub(2),
            "the decomposition must not exceed the ear-cut triangle bound"
        );
    }

    #[test]
    fn triangulates_hole_without_filling_it() {
        let input = PolygonSet::new(vec![Polygon::with_holes(
            square(0.0, 0.0, 10.0, 10.0),
            vec![square(2.0, 2.0, 8.0, 8.0)],
        )]);
        let prepared = prepare_geometry_from_polygon_set(&input, "test").unwrap();
        assert_eq!(prepared.geometry.components[0].holes.len(), 1);
        assert_eq!(prepared.area2, 128_000_000);
        assert_eq!(
            locate_paths(&prepared.canonical_paths, 5_000, 5_000).unwrap(),
            Location::Outside
        );
    }

    #[test]
    fn convex_atom_sum_is_filled_hull() {
        let atom = GridPath::new(vec![
            GridPoint::from_scaled(0, 0),
            GridPoint::from_scaled(1_000, 0),
            GridPoint::from_scaled(0, 1_000),
        ]);
        let result =
            nfp_from_atoms(std::slice::from_ref(&atom), std::slice::from_ref(&atom)).unwrap();
        assert_eq!(result.len(), 1);
        assert_eq!(locate_paths(&result, 0, 0).unwrap(), Location::Inside);
    }

    #[test]
    fn canonicalizes_winding_and_disconnected_components() {
        let mut outer = square(10.0, 0.0, 14.0, 4.0);
        outer.vertices.reverse();
        let mut hole = square(11.0, 1.0, 13.0, 3.0);
        // Deliberately give the hole the same winding as an ordinary shell.
        if ring_area2(
            &hole
                .vertices
                .iter()
                .map(|point| checked_grid_point(*point, "test", 0, 0).unwrap())
                .collect::<Vec<_>>(),
        )
        .unwrap()
            < 0
        {
            hole.vertices.reverse();
        }
        let input = PolygonSet::new(vec![
            Polygon::with_holes(outer, vec![hole]),
            Polygon::new(square(-3.0, 0.0, -1.0, 2.0)),
        ]);
        let prepared = prepare_geometry_from_polygon_set(&input, "test").unwrap();
        assert_eq!(prepared.geometry.components.len(), 2);
        assert_eq!(prepared.geometry.components[1].holes.len(), 1);
        let signed = prepared
            .canonical_paths
            .iter()
            .map(GridPath::signed_area)
            .collect::<Vec<_>>();
        assert!(signed[0] > 0.0);
        assert!(signed[1] > 0.0);
        assert!(signed[2] < 0.0);
        assert_eq!(
            locate_paths(&prepared.canonical_paths, 12_000, 2_000).unwrap(),
            Location::Outside
        );
    }

    #[test]
    fn preserves_island_nested_inside_a_hole() {
        let input = PolygonSet::new(vec![
            Polygon::with_holes(
                square(0.0, 0.0, 10.0, 10.0),
                vec![square(2.0, 2.0, 8.0, 8.0)],
            ),
            Polygon::new(square(4.0, 4.0, 6.0, 6.0)),
        ]);
        let prepared = prepare_geometry_from_polygon_set(&input, "test").unwrap();
        assert_eq!(prepared.geometry.components.len(), 2);
        assert_eq!(prepared.geometry.components[0].holes.len(), 1);
        assert_eq!(
            locate_paths(&prepared.canonical_paths, 3_000, 3_000).unwrap(),
            Location::Outside
        );
        assert_eq!(
            locate_paths(&prepared.canonical_paths, 5_000, 5_000).unwrap(),
            Location::Inside
        );
    }

    #[test]
    fn rejects_invalid_holes_and_non_finite_coordinates() {
        let outside_hole = PolygonSet::new(vec![Polygon::with_holes(
            square(0.0, 0.0, 4.0, 4.0),
            vec![square(3.0, 3.0, 5.0, 5.0)],
        )]);
        assert!(matches!(
            prepare_geometry_from_polygon_set(&outside_hole, "test"),
            Err(NfpError::HoleOutsideShell { .. })
        ));

        let non_finite =
            PolygonSet::from_outer(Ring::new([(0.0, 0.0), (f64::NAN, 0.0), (0.0, 1.0)]));
        assert!(matches!(
            prepare_geometry_from_polygon_set(&non_finite, "test"),
            Err(NfpError::NonFiniteCoordinate { .. })
        ));
    }

    #[test]
    fn distinguishes_nfp_from_true_minkowski_sum() {
        let first = GridPath::new(vec![
            GridPoint::from_scaled(1_000, 0),
            GridPoint::from_scaled(2_000, 0),
            GridPoint::from_scaled(1_000, 1_000),
        ]);
        let second = GridPath::new(vec![
            GridPoint::from_scaled(3_000, 0),
            GridPoint::from_scaled(4_000, 0),
            GridPoint::from_scaled(3_000, 1_000),
        ]);
        let nfp =
            nfp_from_atoms(std::slice::from_ref(&first), std::slice::from_ref(&second)).unwrap();
        let sum = minkowski_sum_atoms(std::slice::from_ref(&first), std::slice::from_ref(&second))
            .unwrap();
        assert_eq!(bounds_scaled(&nfp).unwrap().0, -3_000);
        assert_eq!(bounds_scaled(&sum).unwrap().0, 4_000);
    }

    #[test]
    fn point_location_uses_exact_area_sign_at_large_coordinates() {
        // Exact area2 is +100, while a direct f64 shoelace evaluation at this
        // offset is negative on the test platform because of cancellation.
        let triangle = GridPath::new(vec![
            GridPoint::from_scaled(100_000_000_000, 100_000_000_000),
            GridPoint::from_scaled(633_162_911_730, 429_512_800_990),
            GridPoint::from_scaled(962_675_712_720, 633_162_911_730),
        ]);
        let points = triangle.iter().copied().collect::<Vec<_>>();
        assert_eq!(ring_area2(&points).unwrap(), 100);
        assert_eq!(
            locate_paths(
                &GridPaths::new(vec![triangle]),
                239_583_862_445,
                186_267_571_272,
            )
            .unwrap(),
            Location::Inside
        );
    }
}
