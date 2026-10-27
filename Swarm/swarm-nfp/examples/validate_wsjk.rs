//! Bounded, deterministic real-data validation for the standalone NFP kernel.
//!
//! This example deliberately does not call Swarm's optimizer. It prepares
//! every polygon in a Swarm JSON input, then exercises a small representative
//! pair matrix whose fixed side is the smallest polygon. That keeps the
//! quadratic triangle-atom construction bounded while still covering the
//! low/median/high vertex-count strata in the supplied WSjk data set.

use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use serde::Deserialize;
use swarm_nfp::{
    Bounds, CompleteNfp, Location, NfpEngine, Orientation, PairRelation, Point, PolygonSet,
    PreparedShape, Ring, Translation,
};

const DEFAULT_INPUT: &str = "C:\\Caplanet\\AiWorks\\InputData\\\u{c5ec}\u{c131}WSjk_\u{be44}\u{ad50}\u{c6a9}-G_New_SwarmV1.json";
const GRID_EPSILON_MM: f64 = 0.001;
const AREA_ABSOLUTE_EPSILON_MM2: f64 = 0.000_001;
const AREA_RELATIVE_EPSILON: f64 = 0.000_000_01;
const MEMBERSHIP_GRID_SIDE: usize = 21;
const MAX_EXACT_CONTACT_PROBES: usize = 64;

#[derive(Debug, Deserialize)]
struct SwarmInput {
    name: String,
    items: Vec<SwarmItem>,
}

#[derive(Debug, Deserialize)]
struct SwarmItem {
    id: i64,
    shape: SwarmShape,
}

#[derive(Debug, Deserialize)]
struct SwarmShape {
    #[serde(rename = "type")]
    kind: String,
    data: Vec<[f64; 2]>,
}

struct PreparedItem {
    id: i64,
    raw_vertices: usize,
    prepared: PreparedShape,
    prepare_time: Duration,
}

#[derive(Debug, Clone, Copy)]
struct Topology {
    components: usize,
    holes: usize,
    rings: usize,
    vertices: usize,
    triangles: usize,
}

fn main() -> Result<(), Box<dyn Error>> {
    let input_path = env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(DEFAULT_INPUT));
    let total_started = Instant::now();

    let read_started = Instant::now();
    let bytes = fs::read(&input_path)?;
    let read_elapsed = read_started.elapsed();
    let parse_started = Instant::now();
    let input: SwarmInput = serde_json::from_slice(&bytes)?;
    let parse_elapsed = parse_started.elapsed();

    println!("WSJK_NFP_VALIDATION version=1");
    println!("input={}", input_path.display());
    println!("input_name={}", input.name);
    println!(
        "input_bytes={} items={} read_ms={:.3} parse_ms={:.3}",
        bytes.len(),
        input.items.len(),
        millis(read_elapsed),
        millis(parse_elapsed)
    );

    let engine = NfpEngine::new();
    let prepare_all_started = Instant::now();
    let mut items = Vec::with_capacity(input.items.len());
    for item in input.items {
        if item.shape.kind != "simple_polygon" {
            return Err(format!(
                "item {} has unsupported shape type {:?}",
                item.id, item.shape.kind
            )
            .into());
        }
        let raw_vertices = item.shape.data.len();
        let geometry = PolygonSet::from_outer(Ring::new(
            item.shape
                .data
                .into_iter()
                .map(|point| (point[0], point[1])),
        ));
        let started = Instant::now();
        let prepared = engine
            .prepare(&geometry)
            .map_err(|error| format!("preparing item {} failed: {error}", item.id))?;
        items.push(PreparedItem {
            id: item.id,
            raw_vertices,
            prepared,
            prepare_time: started.elapsed(),
        });
    }
    let prepare_all_elapsed = prepare_all_started.elapsed();
    if items.is_empty() {
        return Err("input contains no items".into());
    }

    items.sort_by_key(|item| (item.raw_vertices, item.id));
    let prepare_sum: Duration = items.iter().map(|item| item.prepare_time).sum();
    let prepare_max = items
        .iter()
        .max_by_key(|item| item.prepare_time)
        .expect("non-empty items");
    let raw_total: usize = items.iter().map(|item| item.raw_vertices).sum();
    let normalized_total: usize = items
        .iter()
        .map(|item| topology(item.prepared.geometry()).vertices)
        .sum();
    let triangle_total: usize = items
        .iter()
        .map(|item| topology(item.prepared.geometry()).triangles)
        .sum();
    println!(
        "prepare_all=PASS elapsed_ms={:.3} measured_sum_ms={:.3} max_item_id={} max_item_ms={:.3}",
        millis(prepare_all_elapsed),
        millis(prepare_sum),
        prepare_max.id,
        millis(prepare_max.prepare_time)
    );
    println!(
        "prepared_raw_vertices={} normalized_vertices={} topology_derived_triangles={}",
        raw_total, normalized_total, triangle_total
    );

    let last = items.len() - 1;
    let low_index = 0;
    let median_index = last / 2;
    let high_index = last;
    let strata = [
        ("low", low_index),
        ("median", median_index),
        ("high", high_index),
    ];
    for (name, index) in strata {
        let item = &items[index];
        let topo = topology(item.prepared.geometry());
        println!(
            "stratum={} sorted_index={} id={} raw_vertices={} normalized_vertices={} triangles={} area_mm2={:.6}",
            name,
            index,
            item.id,
            item.raw_vertices,
            topo.vertices,
            topo.triangles,
            item.prepared.area_mm2()
        );
    }

    // Each row computes both directions and a repeated forward result. Keeping
    // the low-vertex item on one side bounds atom pairs even for the 637-vertex
    // high stratum; reverse construction has the same atom count.
    let pairs = [
        ("low-low", low_index, low_index),
        ("low-median", low_index, median_index),
        ("low-high", low_index, high_index),
    ];
    let pairs_started = Instant::now();
    for (label, fixed_index, moving_index) in pairs {
        validate_pair(&engine, label, &items[fixed_index], &items[moving_index])?;
    }
    println!(
        "representative_pairs=PASS elapsed_ms={:.3}",
        millis(pairs_started.elapsed())
    );
    println!(
        "validation=PASS total_elapsed_ms={:.3}",
        millis(total_started.elapsed())
    );
    Ok(())
}

fn validate_pair(
    engine: &NfpEngine,
    label: &str,
    fixed: &PreparedItem,
    moving: &PreparedItem,
) -> Result<(), Box<dyn Error>> {
    let fixed_triangles = topology(fixed.prepared.geometry()).triangles;
    let moving_triangles = topology(moving.prepared.geometry()).triangles;
    let atom_pairs = fixed_triangles
        .checked_mul(moving_triangles)
        .ok_or("triangle atom-pair count overflow")?;

    let forward_started = Instant::now();
    let forward_complete = engine.complete_pair_nfp(&fixed.prepared, &moving.prepared)?;
    let forward = forward_complete.compact_contour();
    let forward_elapsed = forward_started.elapsed();
    let reverse_started = Instant::now();
    let reverse_complete = engine.complete_pair_nfp(&moving.prepared, &fixed.prepared)?;
    let reverse = reverse_complete.compact_contour();
    let reverse_elapsed = reverse_started.elapsed();
    let repeat_started = Instant::now();
    let repeat_complete = engine.complete_pair_nfp(&fixed.prepared, &moving.prepared)?;
    let repeat = repeat_complete.compact_contour();
    let repeat_elapsed = repeat_started.elapsed();

    let forward_json = serde_json::to_vec(forward)?;
    let repeat_json = serde_json::to_vec(repeat)?;
    if forward_json != repeat_json {
        return Err(format!("{label}: repeated forward serialization differs").into());
    }

    // NFP(F, M) = -(NFP(M, F)). Re-prepare and cardinally rotate the reverse
    // output to exercise the public API and its canonicalization, not private
    // backend paths. Do not require byte-identical serialization across the
    // operand swap: an exact Boolean union may split the same boundary into a
    // different sequence of collinear/intersection vertices depending on atom
    // order. Set membership, bounds and area are the relevant invariants.
    let reverse_prepared = engine.prepare(reverse)?;
    let negated_reverse = engine.orient(&reverse_prepared, Orientation::from_degrees_exact(180))?;
    let negated_reverse_json = serde_json::to_vec(negated_reverse.geometry())?;
    if forward_json != negated_reverse_json {
        return Err(
            format!("{label}: canonical serialization differs after reverse-negation").into(),
        );
    }
    let forward_prepared = engine.prepare(forward)?;
    assert_reflected_bounds(label, forward_prepared.bounds(), reverse_prepared.bounds())?;
    let reverse_area_delta = (forward_prepared.area_mm2() - negated_reverse.area_mm2()).abs();
    let reverse_area_tolerance = AREA_ABSOLUTE_EPSILON_MM2
        + AREA_RELATIVE_EPSILON * forward_prepared.area_mm2().max(negated_reverse.area_mm2());
    if reverse_area_delta > reverse_area_tolerance {
        return Err(format!(
            "{label}: reverse-negated area differs: forward={:.9}, negated_reverse={:.9}, delta={reverse_area_delta:.9}, tolerance={reverse_area_tolerance:.9}",
            forward_prepared.area_mm2(),
            negated_reverse.area_mm2()
        )
        .into());
    }

    if forward_complete.primitive_count() != atom_pairs
        || reverse_complete.primitive_count() != atom_pairs
    {
        return Err(format!("{label}: complete primitive count differs from atom pairs").into());
    }

    let forward_boundary = boundary_vertices(forward);
    let reverse_boundary = boundary_vertices(reverse);
    for vertex in &forward_boundary {
        require_reverse_boundary(engine, label, reverse, *vertex)?;
    }
    for vertex in &reverse_boundary {
        require_reverse_boundary(engine, label, forward, *vertex)?;
    }
    let boundary_membership_probes = forward_boundary.len() + reverse_boundary.len();
    let exact_relation_probes = require_complete_matches_classifier(
        engine,
        label,
        &forward_complete,
        fixed,
        moving,
        &forward_boundary,
    )?;
    require_exact_outside_disjoint(engine, label, fixed, moving, forward_prepared.bounds())?;
    let membership_probes = require_reflected_membership_grid(
        engine,
        label,
        forward,
        reverse,
        forward_prepared.bounds(),
    )?;

    let output = topology(forward);
    println!(
        "pair={} fixed_id={} moving_id={} input_triangles={}x{} atom_pairs={} forward_ms={:.3} reverse_ms={:.3} repeat_ms={:.3}",
        label,
        fixed.id,
        moving.id,
        fixed_triangles,
        moving_triangles,
        atom_pairs,
        millis(forward_elapsed),
        millis(reverse_elapsed),
        millis(repeat_elapsed)
    );
    println!(
        "pair_result={} components={} holes={} rings={} vertices={} triangles={} exact_primitives={} area_mm2={:.6} reverse_area_delta_mm2={:.9} json_bytes={} json_fnv1a64={:016x} boundary_membership_probes={} grid_membership_probes={} exact_relation_probes={} exact_disjoint_probes=4 reverse_negation=PASS deterministic_serialization=PASS complete_primitive_union=PASS exact_pair_classifier=PASS",
        label,
        output.components,
        output.holes,
        output.rings,
        output.vertices,
        output.triangles,
        forward_complete.primitive_count(),
        forward_prepared.area_mm2(),
        reverse_area_delta,
        forward_json.len(),
        fnv1a64(&forward_json),
        boundary_membership_probes,
        membership_probes,
        exact_relation_probes
    );
    Ok(())
}

fn require_reverse_boundary(
    engine: &NfpEngine,
    label: &str,
    reverse: &PolygonSet,
    forward_vertex: Point,
) -> Result<(), Box<dyn Error>> {
    let negated = Point::new(-forward_vertex.x, -forward_vertex.y);
    let location = engine.locate(reverse, negated)?;
    if location != Location::Boundary {
        return Err(format!(
            "{label}: reverse-negated vertex ({:.3}, {:.3}) is {location:?}, expected Boundary",
            negated.x, negated.y
        )
        .into());
    }
    Ok(())
}

fn require_complete_matches_classifier(
    engine: &NfpEngine,
    label: &str,
    complete: &CompleteNfp,
    fixed: &PreparedItem,
    moving: &PreparedItem,
    vertices: &[Point],
) -> Result<usize, Box<dyn Error>> {
    let probe_count = vertices.len().min(MAX_EXACT_CONTACT_PROBES);
    if probe_count == 0 {
        return Err(format!("{label}: NFP contains no boundary vertices").into());
    }
    for probe_index in 0..probe_count {
        let vertex_index = probe_index * vertices.len() / probe_count;
        let point = vertices[vertex_index];
        let relation = engine.classify_pair(
            &fixed.prepared,
            &moving.prepared,
            Translation::new(point.x, point.y),
        )?;
        let complete_location = complete.locate(point)?;
        let expected_location = match relation {
            PairRelation::Disjoint => Location::Outside,
            PairRelation::Contact => Location::Boundary,
            PairRelation::Overlap => Location::Inside,
        };
        if complete_location != expected_location {
            return Err(format!(
                "{label}: complete primitive union is {complete_location:?} at ({:.3}, {:.3}), direct classifier is {relation:?}",
                point.x, point.y,
            )
            .into());
        }
    }
    Ok(probe_count)
}

fn require_exact_outside_disjoint(
    engine: &NfpEngine,
    label: &str,
    fixed: &PreparedItem,
    moving: &PreparedItem,
    bounds: Bounds,
) -> Result<(), Box<dyn Error>> {
    let probes = [
        Translation::new(bounds.min.x - 1.0, bounds.min.y - 1.0),
        Translation::new(bounds.min.x - 1.0, bounds.max.y + 1.0),
        Translation::new(bounds.max.x + 1.0, bounds.min.y - 1.0),
        Translation::new(bounds.max.x + 1.0, bounds.max.y + 1.0),
    ];
    for translation in probes {
        let relation = engine.classify_pair(&fixed.prepared, &moving.prepared, translation)?;
        if relation != PairRelation::Disjoint {
            return Err(format!(
                "{label}: outside translation ({:.3}, {:.3}) classified as {relation:?}, expected Disjoint",
                translation.x, translation.y
            )
            .into());
        }
    }
    Ok(())
}

fn require_reflected_membership_grid(
    engine: &NfpEngine,
    label: &str,
    forward: &PolygonSet,
    reverse: &PolygonSet,
    bounds: Bounds,
) -> Result<usize, Box<dyn Error>> {
    let min_x = to_grid(bounds.min.x);
    let min_y = to_grid(bounds.min.y);
    let max_x = to_grid(bounds.max.x);
    let max_y = to_grid(bounds.max.y);
    let denominator = (MEMBERSHIP_GRID_SIDE - 1) as i128;
    let mut probes = 0usize;
    for row in 0..MEMBERSHIP_GRID_SIDE {
        for column in 0..MEMBERSHIP_GRID_SIDE {
            let x = interpolate_grid(min_x, max_x, column as i128, denominator);
            let y = interpolate_grid(min_y, max_y, row as i128, denominator);
            let point = Point::new(from_grid(x), from_grid(y));
            let reflected = Point::new(from_grid(-x), from_grid(-y));
            let forward_location = engine.locate(forward, point)?;
            let reverse_location = engine.locate(reverse, reflected)?;
            if forward_location != reverse_location {
                return Err(format!(
                    "{label}: reflected membership differs at ({:.3}, {:.3}): forward={forward_location:?}, reverse={reverse_location:?}",
                    point.x, point.y
                )
                .into());
            }
            probes += 1;
        }
    }
    Ok(probes)
}

fn to_grid(value_mm: f64) -> i64 {
    (value_mm * 1_000.0).round() as i64
}

fn from_grid(value: i64) -> f64 {
    value as f64 / 1_000.0
}

fn interpolate_grid(min: i64, max: i64, numerator: i128, denominator: i128) -> i64 {
    let value =
        min as i128 + ((max as i128 - min as i128) * numerator + denominator / 2) / denominator;
    value as i64
}

fn assert_reflected_bounds(
    label: &str,
    forward: Bounds,
    reverse: Bounds,
) -> Result<(), Box<dyn Error>> {
    let comparisons = [
        ("min_x", forward.min.x, -reverse.max.x),
        ("min_y", forward.min.y, -reverse.max.y),
        ("max_x", forward.max.x, -reverse.min.x),
        ("max_y", forward.max.y, -reverse.min.y),
    ];
    for (axis, actual, expected) in comparisons {
        if (actual - expected).abs() > GRID_EPSILON_MM {
            return Err(format!(
                "{label}: reflected {axis} differs: forward={actual:.6}, reflected_reverse={expected:.6}"
            )
            .into());
        }
    }
    Ok(())
}

fn boundary_vertices(region: &PolygonSet) -> Vec<Point> {
    let mut result = Vec::new();
    for component in &region.components {
        result.extend(component.outer.vertices.iter().copied());
        for hole in &component.holes {
            result.extend(hole.vertices.iter().copied());
        }
    }
    result
}

fn topology(region: &PolygonSet) -> Topology {
    let components = region.components.len();
    let holes = region
        .components
        .iter()
        .map(|polygon| polygon.holes.len())
        .sum();
    let rings = components + holes;
    let vertices = region
        .components
        .iter()
        .map(|polygon| {
            polygon.outer.vertices.len()
                + polygon
                    .holes
                    .iter()
                    .map(|hole| hole.vertices.len())
                    .sum::<usize>()
        })
        .sum();
    // Euler triangulation count for each valid polygon component. The kernel
    // uses ear clipping with this same topology; no private crate fields are
    // needed by the standalone example.
    let triangles = region
        .components
        .iter()
        .map(|polygon| {
            let component_vertices = polygon.outer.vertices.len()
                + polygon
                    .holes
                    .iter()
                    .map(|hole| hole.vertices.len())
                    .sum::<usize>();
            component_vertices + 2 * polygon.holes.len() - 2
        })
        .sum();
    Topology {
        components,
        holes,
        rings,
        vertices,
        triangles,
    }
}

fn millis(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1_000.0
}

fn fnv1a64(bytes: &[u8]) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325u64;
    for byte in bytes {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    hash
}
