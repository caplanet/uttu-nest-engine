//! Column Draft: build an attachment chain bottom-to-top, fill its pockets,
//! then start the next column. Input placement hints are deliberately unavailable
//! at this API boundary. See docs/DRAFT_COLUMN_INIT.md for the inferred rules.
use std::collections::{BTreeMap, BTreeSet};
use std::time::{Duration, Instant};

use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::geo_traits::TransformableFrom;
use jagua_rs::geometry::primitives::SPolygon;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution};
use serde::Serialize;
use swarm_nfp::Point;

use super::chain::{NfpPairIndex, NfpPairKey, PoseKey};
use super::driver::rotations_from_instance;
use super::geom::PoseCatalogue;
use super::record::RotationEntry;
use crate::eval::lbf_evaluator::polygons_collide_bidirectionally;

#[path = "column_score.rs"]
mod scoring;

const EPS: f64 = 0.02;
const MAX_PROBES: usize = 10_000;
const ATTACH_LIMIT: usize = 10;
const SPATIAL_CELL_MM: f64 = 128.0;

#[derive(Clone, Copy, Debug)]
pub struct ColumnDraftConfig {
    /// Includes geometry preparation and contact search. Completion and final
    /// validation still run after expiry; this is not a hard wall-clock limit.
    pub budget: Duration,
    /// Retained for Rust source compatibility. Ordered Draft no longer compares
    /// a size-diverse shortlist of pieces before committing the current piece.
    pub shortlist: usize,
    /// Imported contour already has a certified mm bound: do not add the
    /// legacy area-ratio proposal simplification on top of it.
    pub preserve_input_contour: bool,
}

impl Default for ColumnDraftConfig {
    fn default() -> Self {
        Self {
            budget: Duration::from_secs(8),
            shortlist: 8,
            preserve_input_contour: false,
        }
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct ColumnPlacement {
    pub item_id: usize,
    pub copy_index: usize,
    pub column: usize,
    /// Index into this report's placement sequence, never an item id.
    pub attach_to: Option<usize>,
    pub backfill: bool,
    pub completion: bool,
    pub x: f64,
    pub y: f64,
    pub angle_rad: f32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub scoring: Option<DraftScore>,
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct DraftScore {
    pub placed_area_mm2: f64,
    pub piece_count: usize,
    pub reference_area_mm2: f64,
    pub strip_count: usize,
    pub value: f64,
    pub avg_s: f64,
    /// Maximum value in this rotation's bounded position proposal batch.
    pub v_max: f64,
    pub shortfall: f64,
    /// The specified curve, reported independently: the literal final formula
    /// contains no weight term and does not discard a zero-weight best value.
    pub weight: f64,
    pub final_score: f64,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct ColumnDraftReport {
    pub placements: Vec<ColumnPlacement>,
    pub columns: usize,
    pub nfp_pairs: usize,
    pub nfp_failures: usize,
    pub nfp_failure_examples: Vec<String>,
    pub candidate_probes: usize,
    pub candidate_attempts: usize,
    pub max_piece_attempts: usize,
    pub held_full_overlap: usize,
    pub held_no_attachment: usize,
    pub probe_limit_hits: usize,
    pub budget_exhausted: bool,
    pub preparation_ms: u128,
    pub elapsed_ms: u128,
    pub width: f32,
    pub density: f32,
}

struct Pose {
    item: usize,
    rotation: RotationEntry,
    angle: f32,
    original: SPolygon,
    bbox: [f64; 4],
    fabric_bbox: [f64; 4],
    area: f64,
    priority: f64,
    scoring_ring: Vec<Point>,
}

struct Placed {
    pose: usize,
    x: f64,
    y: f64,
    polygon: SPolygon,
    original_outline: Option<crate::original_overlap::PlacedOutline>,
}

#[derive(Clone)]
struct Boundary {
    segments: Vec<(Point, Point)>,
    bbox: [f64; 4],
    parent: usize,
}

impl Boundary {
    /// A full-overlap marker applies to this bounded translation window, not
    /// to the reusable pair NFP. No boundary may enter/touch the window: corner
    /// membership alone would miss holes or a concave notch through its centre.
    fn covers_window(&self, limits: [f64; 4]) -> bool {
        let corners = [
            (limits[0], limits[1]),
            (limits[2], limits[1]),
            (limits[2], limits[3]),
            (limits[0], limits[3]),
        ];
        corners.iter().all(|&(x, y)| self.contains(x, y))
            && self.segments.iter().all(|&(a, b)| {
                !overlaps(
                    limits,
                    [a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y)],
                )
            })
    }
    /// Even/odd membership over preserved ring edges (components and holes).
    /// This is a conservative proposal filter only; exact f32 shapes still
    /// authorize every accepted placement. Boundary points remain candidates.
    fn contains(&self, x: f64, y: f64) -> bool {
        if x < self.bbox[0] || x > self.bbox[2] || y < self.bbox[1] || y > self.bbox[3] {
            return false;
        }
        let mut inside = false;
        for &(a, b) in &self.segments {
            let d = cross(sub(b, a), Point::new(x - a.x, y - a.y));
            if d.abs() <= 1e-5
                && x >= a.x.min(b.x) - 1e-6
                && x <= a.x.max(b.x) + 1e-6
                && y >= a.y.min(b.y) - 1e-6
                && y <= a.y.max(b.y) + 1e-6
            {
                return false;
            }
            if (a.y > y) != (b.y > y) && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x {
                inside = !inside;
            }
        }
        inside
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct Spot {
    pose: usize,
    x: f64,
    y: f64,
    parent: Option<usize>,
    scoring: Option<DraftScore>,
}

#[derive(Default)]
struct ProbeBudget {
    used: usize,
}
impl ProbeBudget {
    fn take(&mut self) -> bool {
        if self.used >= MAX_PROBES {
            return false;
        }
        self.used += 1;
        true
    }
}

enum PairNfp {
    Ready(Vec<Vec<Point>>),
    Empty,
    Failed,
}
enum Attempt {
    Placed(Spot),
    Invalid,
    Held,
}
impl Attempt {
    fn placed(self) -> Option<Spot> {
        if let Self::Placed(s) = self {
            Some(s)
        } else {
            None
        }
    }
}

/// Shoelace products use i128 intermediates, with an i64 area at 0.01mm
/// resolution. This avoids the reference's overflowing i32 candidate area.
fn area_mm2(poly: &SPolygon) -> f64 {
    let q: Vec<_> = poly
        .vertices()
        .iter()
        .map(|p| {
            (
                (p.0 as f64 * 100.0).round() as i64,
                (p.1 as f64 * 100.0).round() as i64,
            )
        })
        .collect();
    let twice: i128 = q
        .iter()
        .zip(q.iter().cycle().skip(1))
        .take(q.len())
        .map(|(&(ax, ay), &(bx, by))| ax as i128 * by as i128 - bx as i128 * ay as i128)
        .sum();
    let area = i64::try_from(twice.abs() / 2).unwrap_or(i64::MAX);
    area as f64 / 10_000.0
}

fn bbox(p: &SPolygon) -> [f64; 4] {
    [
        p.bbox().x_min as f64,
        p.bbox().y_min as f64,
        p.bbox().x_max as f64,
        p.bbox().y_max as f64,
    ]
}

fn overlaps(a: [f64; 4], b: [f64; 4]) -> bool {
    a[0] <= b[2] && b[0] <= a[2] && a[1] <= b[3] && b[1] <= a[3]
}

fn shifted(b: [f64; 4], x: f64, y: f64) -> [f64; 4] {
    [b[0] + x, b[1] + y, b[2] + x, b[3] + y]
}

/// The shared predicate detects proper crossings and POI containment, but a
/// shallow overlap of aligned rectangles can have neither. Jagua polygons are
/// CCW: coincident edges running in the same direction have material on the
/// same side, whereas an allowed shared contact runs in opposite directions.
fn material_overlap(a: &SPolygon, b: &SPolygon) -> bool {
    if polygons_collide_bidirectionally(a, b) {
        return true;
    }
    let scale = bbox(a)
        .into_iter()
        .chain(bbox(b))
        .map(f64::abs)
        .fold(1.0, f64::max);
    let tol = 1e-4f64.max(2.0 * f32::EPSILON as f64 * scale);
    let ab = bbox(a);
    let bb = bbox(b);
    if (ab[2].min(bb[2]) - ab[0].max(bb[0])).min(ab[3].min(bb[3]) - ab[1].max(bb[1])) <= tol {
        return false;
    }
    for e in a.edge_iter() {
        let p = Point::new(e.start.0 as f64, e.start.1 as f64);
        let v = Point::new((e.end.0 - e.start.0) as f64, (e.end.1 - e.start.1) as f64);
        let len = v.x.hypot(v.y);
        if len <= tol {
            continue;
        }
        for f in b.edge_iter() {
            let q = Point::new(f.start.0 as f64, f.start.1 as f64);
            let r = Point::new(f.end.0 as f64, f.end.1 as f64);
            let w = sub(r, q);
            if v.x * w.x + v.y * w.y <= 0.0 {
                continue;
            }
            if cross(v, sub(q, p)).abs() > tol * len || cross(v, sub(r, p)).abs() > tol * len {
                continue;
            }
            let project = |q: Point| ((q.x - p.x) * v.x + (q.y - p.y) * v.y) / len;
            let t0 = project(q);
            let t1 = project(r);
            if t0.max(0.0) + tol < t1.min(len) {
                return true;
            }
        }
    }
    false
}

fn cross(a: Point, b: Point) -> f64 {
    a.x * b.y - a.y * b.x
}
fn sub(a: Point, b: Point) -> Point {
    Point::new(a.x - b.x, a.y - b.y)
}

fn intersection(a: Point, b: Point, c: Point, d: Point) -> Option<Point> {
    let ab = sub(b, a);
    let cd = sub(d, c);
    let det = cross(ab, cd);
    if det.abs() < 1e-10 {
        return None;
    }
    let ca = sub(c, a);
    let t = cross(ca, cd) / det;
    let u = cross(ca, ab) / det;
    if !(-1e-9..=1.0 + 1e-9).contains(&t) || !(-1e-9..=1.0 + 1e-9).contains(&u) {
        return None;
    }
    Some(Point::new(a.x + t * ab.x, a.y + t * ab.y))
}

/// Full boundary events, including NFP/NFP and NFP/wall intersections. No
/// vertex subsampling and no Boolean union over the complete placed layout.
fn contact_events(
    records: &[Boundary],
    limits: [f64; 4],
    max_events: usize,
    expired: impl Fn() -> bool,
) -> Vec<(Point, Option<usize>)> {
    if max_events == 0 {
        return Vec::new();
    }
    let mut out = vec![(Point::new(limits[0], limits[1]), None)];
    let walls = [
        (
            Point::new(limits[0], limits[1]),
            Point::new(limits[2], limits[1]),
        ),
        (
            Point::new(limits[0], limits[1]),
            Point::new(limits[0], limits[3]),
        ),
        (
            Point::new(limits[2], limits[1]),
            Point::new(limits[2], limits[3]),
        ),
        (
            Point::new(limits[0], limits[3]),
            Point::new(limits[2], limits[3]),
        ),
    ];
    for r in records {
        for &(a, b) in &r.segments {
            if out.len() >= max_events || expired() {
                return out;
            }
            out.push((a, Some(r.parent)));
            for &(c, d) in &walls {
                if out.len() >= max_events {
                    return out;
                }
                if let Some(p) = intersection(a, b, c, d) {
                    out.push((p, Some(r.parent)));
                }
            }
        }
    }
    let mut work = Vec::new();
    for i in 0..records.len() {
        for j in i + 1..records.len() {
            if overlaps(records[i].bbox, records[j].bbox) {
                let a = records[i].bbox;
                let b = records[j].bbox;
                let dist2 =
                    (a[0] + a[2] - b[0] - b[2]).powi(2) + (a[1] + a[3] - b[1] - b[3]).powi(2);
                work.push((dist2, i, j));
            }
        }
    }
    work.sort_by(|a, b| a.0.total_cmp(&b.0).then((a.1, a.2).cmp(&(b.1, b.2))));
    for (_, i, j) in work {
        for &(a, b) in &records[i].segments {
            if out.len() >= max_events || expired() {
                return out;
            }
            let ab = [a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y)];
            // A segment that misses the whole record's bbox cannot intersect any
            // of its segments: skip the inner loop instead of testing each one.
            if !overlaps(ab, records[j].bbox) {
                continue;
            }
            for &(c, d) in &records[j].segments {
                if out.len() >= max_events {
                    return out;
                }
                if overlaps(ab, [c.x.min(d.x), c.y.min(d.y), c.x.max(d.x), c.y.max(d.y)]) {
                    if let Some(p) = intersection(a, b, c, d) {
                        out.push((p, Some(records[i].parent)));
                    }
                }
            }
        }
    }
    out
}

struct Search<'a> {
    poses: &'a [Pose],
    catalogue: &'a PoseCatalogue,
    // Ordered, directional pose keys. Inverse queries cannot accidentally use
    // an unreflected NFP. Missing pairs are cached too.
    pairs: NfpPairIndex<PairNfp>,
    placed: Vec<Placed>,
    spatial: BTreeMap<(i32, i32), Vec<usize>>,
    bounds: [f64; 4],
    fabric_height: f64,
    placed_area_mm2: f64,
    occupied_right: f64,
    original_guard: Option<crate::original_overlap::OriginalOverlapGuard>,
    started: Instant,
    config: ColumnDraftConfig,
    report: ColumnDraftReport,
}

impl Search<'_> {
    fn consume_probe(&mut self, budget: &mut ProbeBudget) -> bool {
        if !budget.take() {
            return false;
        }
        self.report.candidate_attempts += 1;
        self.report.max_piece_attempts = self.report.max_piece_attempts.max(budget.used);
        true
    }
    fn expired(&self) -> bool {
        crate::config::runtime::init_cancel_requested() || self.started.elapsed() >= self.config.budget
    }

    fn spatial_members(&self, b: [f64; 4]) -> Vec<usize> {
        let cell = |v: f64| (v / SPATIAL_CELL_MM).floor() as i32;
        let mut found = BTreeSet::new();
        for x in cell(b[0])..=cell(b[2]) {
            for y in cell(b[1])..=cell(b[3]) {
                if let Some(ids) = self.spatial.get(&(x, y)) {
                    found.extend(ids.iter().copied());
                }
            }
        }
        found
            .into_iter()
            .filter(|&i| overlaps(b, self.placed_bounds(i)))
            .collect()
    }

    fn placed_bounds(&self, i: usize) -> [f64; 4] {
        let p = &self.placed[i];
        let cd = bbox(&p.polygon);
        let raw = shifted(self.poses[p.pose].fabric_bbox, p.x, p.y);
        [
            cd[0].min(raw[0]),
            cd[1].min(raw[1]),
            cd[2].max(raw[2]),
            cd[3].max(raw[3]),
        ]
    }

    /// Select contacts by geometry, including older pieces still bordering the
    /// open column. Placement recency is not a substitute for the contour.
    fn contour_contacts(
        &self,
        moving: usize,
        left: f64,
        max_left: f64,
        floor: f64,
        seed: &[usize],
    ) -> Vec<usize> {
        let b = self.poses[moving].bbox;
        let w = b[2] - b[0];
        let mut ids = self.spatial_members([
            left - EPS,
            floor - EPS,
            max_left + w + EPS,
            self.bounds[3] + EPS,
        ]);
        ids.sort_by(|&a, &b| {
            let distance = |i: usize| {
                let bb = bbox(&self.placed[i].polygon);
                let dx = (bb[0] - left).max(0.0);
                let dy = (bb[1] - floor).max(0.0);
                dx * dx + dy * dy
            };
            distance(a).total_cmp(&distance(b)).then(a.cmp(&b))
        });
        ids.truncate(ATTACH_LIMIT);
        // Explicit parent/root contacts remain eligible if locally relevant.
        for &i in seed {
            if !ids.contains(&i)
                && overlaps(
                    bbox(&self.placed[i].polygon),
                    [
                        left - EPS,
                        floor - EPS,
                        max_left + w + EPS,
                        self.bounds[3] + EPS,
                    ],
                )
            {
                ids.push(i);
                if ids.len() >= ATTACH_LIMIT + 2 {
                    break;
                }
            }
        }
        ids
    }

    fn pair_key(&self, fixed: usize, moving: usize) -> NfpPairKey {
        let a = &self.poses[fixed];
        let b = &self.poses[moving];
        NfpPairKey {
            fixed_item: a.item,
            fixed_pose: PoseKey::of(&a.rotation),
            moving_item: b.item,
            moving_pose: PoseKey::of(&b.rotation),
        }
    }

    /// Preparation may extend the sorted cache; boundary lookup below cannot.
    fn prepare_pairs(&mut self, moving: usize, chain: &[usize]) {
        for &parent in chain {
            if self.expired() {
                break;
            }
            let fixed = &self.placed[parent];
            let key = self.pair_key(fixed.pose, moving);
            if self.pairs.get(&key).is_some() {
                continue;
            }
            let value = (|| {
                let a = &self.poses[fixed.pose];
                let b = &self.poses[moving];
                let (Some(fixed_shape), Some(moving_shape)) = (
                    self.catalogue.shape(a.item, PoseKey::of(&a.rotation)),
                    self.catalogue.shape(b.item, PoseKey::of(&b.rotation)),
                ) else {
                    return PairNfp::Failed;
                };
                let result = self.catalogue.engine().pair_nfp(fixed_shape, moving_shape);
                let set = match result {
                    Ok(set) => set,
                    Err(error) => {
                        if self.report.nfp_failure_examples.len() < 4 {
                            self.report
                                .nfp_failure_examples
                                .push(format!("{} -> {}: {error}", a.item, b.item));
                        }
                        return PairNfp::Failed;
                    }
                };
                let rings: Vec<Vec<Point>> = set
                    .components
                    .iter()
                    .flat_map(|p| std::iter::once(&p.outer).chain(p.holes.iter()))
                    .filter(|r| r.vertices.len() >= 3)
                    .map(|r| r.vertices.clone())
                    .collect();
                if rings.is_empty() {
                    PairNfp::Empty
                } else {
                    PairNfp::Ready(rings)
                }
            })();
            self.pairs.insert_prepared(key, value);
        }
    }

    fn boundaries(&self, moving: usize, chain: &[usize]) -> Vec<Boundary> {
        let mut out = Vec::new();
        for &parent in chain {
            let fixed = &self.placed[parent];
            let Some(PairNfp::Ready(rings)) = self.pairs.get(&self.pair_key(fixed.pose, moving))
            else {
                continue;
            };
            let mut segs = Vec::new();
            let mut bounds = [
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ];
            for ring in rings {
                for (a, b) in ring
                    .iter()
                    .zip(ring.iter().cycle().skip(1))
                    .take(ring.len())
                {
                    let a = Point::new(a.x + fixed.x, a.y + fixed.y);
                    let b = Point::new(b.x + fixed.x, b.y + fixed.y);
                    bounds[0] = bounds[0].min(a.x);
                    bounds[1] = bounds[1].min(a.y);
                    bounds[2] = bounds[2].max(a.x);
                    bounds[3] = bounds[3].max(a.y);
                    segs.push((a, b));
                }
            }
            out.push(Boundary {
                segments: segs,
                bbox: bounds,
                parent,
            });
        }
        out
    }

    fn feasible(&self, pose: usize, x: f64, y: f64) -> Option<SPolygon> {
        let p = &self.poses[pose];
        let mut shape = p.original.clone();
        shape.transform_from(
            &p.original,
            &DTransformation::new(p.angle, (x as f32, y as f32)).into(),
        );
        let b = bbox(&shape);
        let raw = shifted(p.fabric_bbox, x as f32 as f64, y as f32 as f64);
        if raw[0] < -1e-5 || raw[1] < -1e-5 || raw[3] > self.fabric_height + 1e-5 {
            return None;
        }
        if b[0] < self.bounds[0] - 1e-5
            || b[1] < self.bounds[1] - 1e-5
            || b[2] > self.bounds[2] + 1e-5
            || b[3] > self.bounds[3] + 1e-5
        {
            return None;
        }
        let query = [
            b[0].min(raw[0]),
            b[1].min(raw[1]),
            b[2].max(raw[2]),
            b[3].max(raw[3]),
        ];
        let neighbors = self.spatial_members(query);
        for &index in &neighbors {
            let other = &self.placed[index];
            if overlaps(b, bbox(&other.polygon)) && material_overlap(&shape, &other.polygon) {
                return None;
            }
        }
        if let Some(guard) = &self.original_guard {
            let candidate = guard
                .outline(p.item, DTransformation::new(p.angle, (x as f32, y as f32)))
                .ok()?;
            for index in neighbors {
                let other = self.placed[index].original_outline.as_ref()?;
                if !guard.outlines_allowed(&candidate, other).ok()? {
                    return None;
                }
            }
        }
        Some(shape)
    }

    fn best_spot(
        &mut self,
        pose: usize,
        chain: &[usize],
        left: f64,
        right: f64,
        floor: f64,
        max_left: Option<f64>,
        budget: &mut ProbeBudget,
        root: bool,
    ) -> Attempt {
        if !root && chain.is_empty() {
            self.report.held_no_attachment += 1;
            return Attempt::Held;
        }
        let p = &self.poses[pose];
        let b = p.bbox;
        let limits = [
            (left - b[0]).max(-p.fabric_bbox[0]),
            (floor - b[1]).max(-p.fabric_bbox[1]),
            (right - b[2]).min(max_left.unwrap_or(right - (b[2] - b[0])) - b[0]),
            (self.bounds[3] - b[3]).min(self.fabric_height - p.fabric_bbox[3]),
        ];
        if limits[0] > limits[2] + 1e-6 || limits[1] > limits[3] + 1e-6 {
            return Attempt::Invalid;
        }
        let contacts = self.contour_contacts(
            pose,
            left,
            max_left.unwrap_or(right - (b[2] - b[0])),
            floor,
            chain,
        );
        // Keep an explicit attachment when a spatial contour query has no
        // local result. A non-root piece is never silently made independent.
        let contacts = if contacts.is_empty() && !root {
            chain.iter().rev().take(1).copied().collect()
        } else {
            contacts
        };
        self.prepare_pairs(pose, &contacts);
        let records = self.boundaries(pose, &contacts);
        if self.expired() {
            return Attempt::Invalid;
        }
        if !contacts.is_empty() && records.is_empty() {
            return Attempt::Invalid; // zero vertices / failed lookup: next rotation
        }
        if records.iter().any(|r| r.covers_window(limits)) {
            self.report.held_full_overlap += 1;
            return Attempt::Held; // retry from a changed column/window
        }
        let events = contact_events(&records, limits, MAX_PROBES - budget.used, || {
            self.expired()
        });
        let mut spots = Vec::new();
        let mut positions = BTreeSet::new();
        'events: for (point, parent) in events {
            if self.expired() {
                break;
            }
            // Four outward directions around a contact, plus exact contact for
            // full-height pieces. Final f32 geometry authorizes every result.
            for (dx, dy) in [
                (0.0, 0.0),
                (EPS, EPS),
                (-EPS, EPS),
                (EPS, -EPS),
                (-EPS, -EPS),
            ] {
                let x = (point.x + dx) as f32 as f64;
                let y = (point.y + dy) as f32 as f64;
                if x < limits[0] - 1e-6
                    || x > limits[2] + 1e-6
                    || y < limits[1] - 1e-6
                    || y > limits[3] + 1e-6
                {
                    continue;
                }
                if !positions.insert(((x as f32).to_bits(), (y as f32).to_bits())) {
                    continue;
                }
                if !self.consume_probe(budget) {
                    break 'events;
                }
                let Some(scoring) = self.score_spot(pose, x, y) else {
                    continue;
                };
                spots.push(Spot {
                    pose,
                    x,
                    y,
                    parent: parent.or_else(|| contacts.first().copied()),
                    scoring: Some(scoring),
                });
            }
        }
        spots.sort_by(|a, b| {
            let sa = a.scoring.unwrap();
            let sb = b.scoring.unwrap();
            sb.final_score
                .total_cmp(&sa.final_score)
                .then(sb.value.total_cmp(&sa.value))
                .then(a.x.total_cmp(&b.x))
                .then(a.y.total_cmp(&b.y))
        });
        spots.dedup_by(|a, b| (a.x - b.x).abs() < 0.001 && (a.y - b.y).abs() < 0.001);
        // v_max is common to this bounded proposal batch, hence sorting by
        // -8*avg_s before adding v_max gives the same highest-score order.
        let v_max = spots
            .iter()
            .map(|s| s.scoring.unwrap().value)
            .fold(0.0, f64::max);
        for mut s in spots {
            if self.expired() {
                return Attempt::Invalid;
            }
            if records.iter().any(|r| r.contains(s.x, s.y)) {
                continue;
            }
            self.report.candidate_probes += 1;
            if self.feasible(pose, s.x, s.y).is_some() {
                s = self.refine_contact(s, left, floor, budget);
                let mut score = self.score_spot(s.pose, s.x, s.y).unwrap();
                score.v_max = v_max.max(score.value);
                score.shortfall = super::score::shortfall(score.value, score.v_max);
                score.weight = super::score::weight(score.shortfall);
                score.final_score = scoring::candidate_score(score.v_max, &[score.avg_s]).unwrap();
                s.scoring = Some(score);
                return Attempt::Placed(s);
            }
        }
        if budget.used == MAX_PROBES {
            self.report.probe_limit_hits += 1;
            Attempt::Held
        } else {
            Attempt::Invalid
        }
    }

    fn score_spot(&self, pose: usize, x: f64, y: f64) -> Option<DraftScore> {
        let p = &self.poses[pose];
        let b = shifted(p.bbox, x, y);
        let right = self.occupied_right.max(b[2]);
        let rect = [self.bounds[0], self.bounds[1], right, self.bounds[3]];
        // Almost all proposals are contained: reuse the per-pose polygon area.
        // Clip the actual ring only when a strip boundary crosses the proposal.
        let area = if b[0] >= rect[0] && b[1] >= rect[1] && b[2] <= rect[2] && b[3] <= rect[3] {
            p.area
        } else {
            scoring::clipped_area(&p.scoring_ring, (x, y), rect)?
        };
        let placed_area_mm2 = self.placed_area_mm2 + area;
        let piece_count = self.placed.len() + 1;
        let reference_area_mm2 = (right - rect[0]) * (rect[3] - rect[1]);
        let value = scoring::value(placed_area_mm2, piece_count, reference_area_mm2)?;
        // SPInstance is one physical fabric strip. Columns are attachment
        // search regions and are not silently counted as additional fabrics.
        let strip_count = 1;
        let avg_s = scoring::surface_average(placed_area_mm2, strip_count)?;
        Some(DraftScore {
            placed_area_mm2,
            piece_count,
            reference_area_mm2,
            strip_count,
            value,
            avg_s,
            v_max: 0.0,
            shortfall: 0.0,
            weight: 0.0,
            final_score: scoring::candidate_score(0.0, &[avg_s])?,
        })
    }

    /// Recover small gaps introduced by conservative proposal simplification.
    /// In particular this lets the configured overlap offset, already embedded
    /// in shape_cd, affect the actual contact. Never change the tolerance here.
    fn refine_contact(
        &mut self,
        mut s: Spot,
        left: f64,
        floor: f64,
        budget: &mut ProbeBudget,
    ) -> Spot {
        let b = self.poses[s.pose].bbox;
        for axis in 0..2 {
            if self.expired() {
                break;
            }
            let current = if axis == 0 { s.x } else { s.y };
            let wall = if axis == 0 {
                (left - b[0]).max(-self.poses[s.pose].fabric_bbox[0])
            } else {
                (floor - b[1]).max(-self.poses[s.pose].fabric_bbox[1])
            };
            let mut lo = (current - 5.0).max(wall);
            let mut hi = current;
            let endpoint = if axis == 0 { (lo, s.y) } else { (s.x, lo) };
            if !self.consume_probe(budget) {
                break;
            }
            self.report.candidate_probes += 1;
            if self.feasible(s.pose, endpoint.0, endpoint.1).is_some() {
                hi = lo;
            }
            for _ in 0..10 {
                if hi - lo < 0.001 {
                    break;
                }
                let mid = (lo + hi) * 0.5;
                let (x, y) = if axis == 0 { (mid, s.y) } else { (s.x, mid) };
                if self.expired() || !self.consume_probe(budget) {
                    break;
                }
                self.report.candidate_probes += 1;
                if self.feasible(s.pose, x, y).is_some() {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            if axis == 0 {
                s.x = hi;
            } else {
                s.y = hi;
            }
        }
        s.x = s.x as f32 as f64;
        s.y = s.y as f32 as f64;
        s
    }

    fn commit(
        &mut self,
        mut spot: Spot,
        copy: usize,
        column: usize,
        backfill: bool,
        completion: bool,
        prob: &mut SPProblem,
    ) -> Result<(), String> {
        spot.x = spot.x as f32 as f64;
        spot.y = spot.y as f32 as f64;
        let p = &self.poses[spot.pose];
        let polygon = self
            .feasible(spot.pose, spot.x, spot.y)
            .ok_or("Draft selected an infeasible contact")?;
        prob.place_item(SPPlacement {
            item_idx: p.item,
            d_transf: DTransformation::new(p.angle, (spot.x as f32, spot.y as f32)),
        });
        self.report.placements.push(ColumnPlacement {
            item_id: p.item,
            copy_index: copy,
            column,
            attach_to: spot.parent,
            backfill,
            completion,
            x: spot.x,
            y: spot.y,
            angle_rad: p.angle,
            scoring: spot.scoring,
        });
        let index = self.placed.len();
        let cd = bbox(&polygon);
        let raw = shifted(p.fabric_bbox, spot.x, spot.y);
        let b = [
            cd[0].min(raw[0]),
            cd[1].min(raw[1]),
            cd[2].max(raw[2]),
            cd[3].max(raw[3]),
        ];
        let cell = |v: f64| (v / SPATIAL_CELL_MM).floor() as i32;
        for x in cell(b[0])..=cell(b[2]) {
            for y in cell(b[1])..=cell(b[3]) {
                self.spatial.entry((x, y)).or_default().push(index);
            }
        }
        let original_outline = self
            .original_guard
            .as_ref()
            .map(|g| {
                g.outline(
                    p.item,
                    DTransformation::new(p.angle, (spot.x as f32, spot.y as f32)),
                )
            })
            .transpose()?;
        self.placed_area_mm2 += p.area;
        self.occupied_right = self.occupied_right.max(cd[2]);
        self.placed.push(Placed {
            pose: spot.pose,
            x: spot.x,
            y: spot.y,
            polygon,
            original_outline,
        });
        Ok(())
    }
}

/// Construct a fresh, complete layout. Existing seeds/frozen/grouped placements
/// belong to the caller's existing constructor, not this independent Draft.
pub fn build_column_draft(
    instance: &SPInstance,
    config: ColumnDraftConfig,
) -> Result<(SPSolution, ColumnDraftReport), String> {
    build_column_draft_impl(instance, config, None, false)
}

/// Diagnostic entry point: an explicit permutation changes order only, never
/// supplies coordinates or rotations. `strict` prevents lookahead/backfill from
/// choosing a later member first. This is not selected by the production UI.
pub fn build_column_draft_with_order(
    instance: &SPInstance,
    config: ColumnDraftConfig,
    order: &[(usize, usize)],
    strict: bool,
) -> Result<(SPSolution, ColumnDraftReport), String> {
    let expected: std::collections::BTreeSet<_> = instance
        .items
        .iter()
        .flat_map(|(item, qty)| (0..*qty).map(move |copy| (item.idx(), copy)))
        .collect();
    let supplied: std::collections::BTreeSet<_> = order.iter().copied().collect();
    if supplied != expected || order.len() != expected.len() {
        return Err("Draft order must contain each demanded (item, copy) exactly once".into());
    }
    build_column_draft_impl(instance, config, Some(order), strict)
}

fn build_column_draft_impl(
    instance: &SPInstance,
    config: ColumnDraftConfig,
    supplied_order: Option<&[(usize, usize)]>,
    strict_order: bool,
) -> Result<(SPSolution, ColumnDraftReport), String> {
    let started = Instant::now();
    if instance.items.is_empty() {
        return Err("Column Draft needs at least one item".into());
    }
    let mut rotations = rotations_from_instance(instance);
    // Continuous means 24 discrete proposals; discrete declarations (including
    // [0]) are never broadened, and mirror poses are never invented.
    for (item, _) in &instance.items {
        if crate::config::runtime::init_cancel_requested() { return Err("initialization cancelled".into()); }
        if matches!(
            item.allowed_orientations().rotations(),
            jagua_rs::geometry::geo_enums::RotationRange::Continuous
        ) {
            rotations.insert(
                item.idx(),
                (0..24).map(|i| RotationEntry::turn(i * 15_000)).collect(),
            );
        }
    }
    let catalogue = PoseCatalogue::build(
        instance,
        &rotations,
        if config.preserve_input_contour {
            0.0
        } else {
            0.005
        },
    );
    let mut poses = Vec::new();
    let mut item_poses = vec![Vec::new(); instance.items.len()];
    for (item, _) in &instance.items {
        if crate::config::runtime::init_cancel_requested() { return Err("initialization cancelled".into()); }
        for &r in &rotations[&item.idx()] {
            let angle = r.radians() as f32;
            let mut polygon = item.shape_cd().as_ref().clone();
            polygon.transform_from(
                &item.shape_cd(),
                &DTransformation::new(angle, (0.0, 0.0)).into(),
            );
            let b = bbox(&polygon);
            let mut fabric = item.shape_orig().shape.clone();
            fabric.transform_from(
                &item.shape_orig().shape,
                &item.shape_orig().pre_transform.compose(),
            );
            let centered = fabric.clone();
            fabric.transform_from(&centered, &DTransformation::new(angle, (0.0, 0.0)).into());
            let concavity = crate::optimizer::concavity::analyze_spolygon(&polygon);
            item_poses[item.idx()].push(poses.len());
            poses.push(Pose {
                item: item.idx(),
                rotation: r,
                angle,
                bbox: b,
                fabric_bbox: bbox(&fabric),
                area: area_mm2(&polygon),
                scoring_ring: polygon
                    .vertices()
                    .iter()
                    .map(|p| Point::new(p.0 as f64, p.1 as f64))
                    .collect(),
                priority: (b[2] - b[0]).hypot(b[3] - b[1])
                    * (1.0 + 5.0 * concavity.left_score + 2.0 * concavity.bottom_score),
                original: item.shape_cd().as_ref().clone(),
            });
        }
    }
    let cap: f64 = instance
        .items
        .iter()
        .map(|(it, qty)| {
            item_poses[it.idx()]
                .iter()
                .map(|&i| {
                    (poses[i].bbox[2] - poses[i].bbox[0])
                        .max(poses[i].fabric_bbox[2] - poses[i].fabric_bbox[0])
                })
                .fold(0.0, f64::max)
                * *qty as f64
        })
        .sum::<f64>()
        + instance.items.iter().map(|(_, q)| *q).sum::<usize>() as f64
        + 10.0;
    let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
    prob.change_strip_width(cap as f32).expect(crate::dmsg!("strip width must be positive"));
    let bounds = bbox(&prob.layout().container().outer_cd());
    for ids in &mut item_poses {
        ids.retain(|&i| {
            let p = &poses[i];
            (bounds[1] - p.bbox[1]).max(-p.fabric_bbox[1])
                <= (bounds[3] - p.bbox[3])
                    .min(instance.base_strip.fixed_height as f64 - p.fabric_bbox[3])
                    + 1e-6
        });
        if ids.is_empty() {
            return Err("An item exceeds strip height in every allowed rotation".into());
        }
    }
    let mut pending: Vec<(usize, usize)> = instance
        .items
        .iter()
        .flat_map(|(it, q)| (0..*q).map(move |c| (it.idx(), c)))
        .collect();
    // Large, directionally concave shapes supply the initial worklist. Then
    // process pieces in that order; a held piece remains for backfill/new column.
    // No sort key or coordinate is read from the reference placement.
    pending.sort_by(|a, b| {
        let size = |id: usize| {
            let p = &poses[item_poses[id][0]];
            p.priority
        };
        size(b.0).total_cmp(&size(a.0)).then(a.cmp(b))
    });
    if let Some(order) = supplied_order {
        pending = order.to_vec();
    }
    let mut search = Search {
        poses: &poses,
        catalogue: &catalogue,
        pairs: NfpPairIndex::new(),
        placed: Vec::new(),
        spatial: BTreeMap::new(),
        bounds,
        fabric_height: instance.base_strip.fixed_height as f64,
        placed_area_mm2: 0.0,
        occupied_right: bounds[0],
        original_guard: crate::original_overlap::OriginalOverlapGuard::is_active(instance)
            .then(|| crate::original_overlap::OriginalOverlapGuard::new(instance)),
        started,
        config,
        report: ColumnDraftReport::default(),
    };
    search.report.preparation_ms = started.elapsed().as_millis();
    let mut right = bounds[0];
    let mut previous = Vec::<usize>::new();
    let mut column = 0;
    while !pending.is_empty() {
        if crate::config::runtime::init_cancel_requested() { return Err("initialization cancelled".into()); }
        let (item, copy) = pending.remove(0);
        let root_pose = *item_poses[item]
            .iter()
            .min_by(|&&a, &&b| {
                let width = |i: usize| poses[i].bbox[2] - poses[i].bbox[0];
                width(a).total_cmp(&width(b)).then(a.cmp(&b))
            })
            .unwrap();
        let p = &poses[root_pose];
        let root_width = p.bbox[2] - p.bbox[0];
        let left = right;
        // New columns restart at the bottom. Contact search can interlock with
        // the preceding column, but does not reopen all previous columns.
        let start_left = previous
            .iter()
            .map(|&i| bbox(&search.placed[i].polygon)[0])
            .fold(left, f64::min)
            .max(bounds[0]);
        let trial = if !search.expired() {
            search
                .best_spot(
                    root_pose,
                    &previous,
                    start_left,
                    left + root_width + EPS,
                    bounds[1],
                    None,
                    &mut ProbeBudget::default(),
                    true,
                )
                .placed()
        } else {
            None
        };
        let root = trial.unwrap_or(Spot {
            pose: root_pose,
            x: (left + EPS - p.bbox[0]).max(
                search
                    .placed
                    .iter()
                    .map(|placed| placed.x + poses[placed.pose].fabric_bbox[2])
                    .fold(0.0, f64::max)
                    + EPS
                    - p.fabric_bbox[0],
            ),
            y: (bounds[1] - p.bbox[1]).max(-p.fabric_bbox[1]),
            parent: None,
            scoring: None,
        });
        let column_left = (root.x + p.bbox[0]).max(bounds[0]);
        right = right.max(root.x + p.bbox[2] + EPS);
        search.commit(root, copy, column, false, search.expired(), &mut prob)?;
        let mut chain = vec![search.placed.len() - 1];
        let mut cursor = bounds[1];
        let mut backfill = false;
        loop {
            if crate::config::runtime::init_cancel_requested() { return Err("initialization cancelled".into()); }
            if pending.is_empty() {
                break;
            }
            if search.expired() {
                search.report.budget_exhausted = true;
                // Deterministic vertical shelf completion: retain all demands,
                // use actual collision checks and never report partial success.
                let top = chain
                    .iter()
                    .map(|&i| bbox(&search.placed[i].polygon)[3])
                    .fold(bounds[1], f64::max);
                let fit = pending
                    .iter()
                    .enumerate()
                    .take(if strict_order { 1 } else { pending.len() })
                    .find_map(|(idx, (id, copy))| {
                        item_poses[*id].iter().find_map(|&pose| {
                            let b = poses[pose].bbox;
                            if b[2] - b[0] > right - column_left
                                || top + EPS + b[3] - b[1] > bounds[3]
                            {
                                return None;
                            }
                            let spot = Spot {
                                pose,
                                x: (column_left - b[0]).max(-poses[pose].fabric_bbox[0]),
                                y: (top + EPS - b[1]).max(-poses[pose].fabric_bbox[1]),
                                parent: chain.last().copied(),
                                scoring: None,
                            };
                            search
                                .feasible(pose, spot.x, spot.y)
                                .map(|_| (idx, *copy, spot))
                        })
                    });
                if let Some((idx, copy, s)) = fit {
                    pending.remove(idx);
                    search.commit(s, copy, column, false, true, &mut prob)?;
                    chain.push(search.placed.len() - 1);
                    continue;
                }
                break;
            }
            let mut attachments: Vec<_> = chain.iter().rev().take(ATTACH_LIMIT).copied().collect();
            if !attachments.contains(&chain[0]) {
                attachments.push(chain[0]);
            }
            attachments.extend(previous.iter().rev().take(ATTACH_LIMIT / 2).copied());
            let mut best: Option<(usize, usize, Spot)> = None;
            let order: Vec<_> = if strict_order {
                vec![0]
            } else {
                (0..pending.len()).collect()
            };
            for idx in order {
                let (id, copy) = pending[idx];
                let floor = if backfill { bounds[1] } else { cursor };
                let can_fit = item_poses[id].iter().any(|&i| {
                    let b = poses[i].bbox;
                    b[3] - b[1] <= bounds[3] - floor + 1e-6
                });
                if !can_fit {
                    continue;
                }
                let mut probes = ProbeBudget::default();
                for &pose in &item_poses[id] {
                    match search.best_spot(
                        pose,
                        &attachments,
                        column_left,
                        (right + poses[pose].bbox[2] - poses[pose].bbox[0]).min(bounds[2]),
                        if backfill { bounds[1] } else { cursor },
                        Some(column_left + root_width * 0.5),
                        &mut probes,
                        false,
                    ) {
                        Attempt::Placed(s) => {
                            best = Some((idx, copy, s));
                            break; // first successful rotation of the current piece
                        }
                        Attempt::Held => break,
                        Attempt::Invalid => {}
                    }
                    if search.expired() {
                        break;
                    }
                }
                if best.is_some() || search.expired() {
                    break;
                }
            }
            if let Some((idx, copy, s)) = best {
                pending.remove(idx);
                cursor = cursor.max(s.y + poses[s.pose].bbox[1]);
                search.commit(s, copy, column, backfill, false, &mut prob)?;
                right = right.max(s.x + poses[s.pose].bbox[2] + EPS);
                chain.push(search.placed.len() - 1);
            } else if search.expired() {
                continue;
            } else if !backfill {
                backfill = true;
            } else {
                break;
            }
        }
        previous = chain;
        column += 1;
    }
    prob.fit_strip().expect(crate::dmsg!("strip width must be positive"));
    // Acute corners retract farther along an axis than the pair offset/2.
    // The original outline must fit the fabric as well as the CD geometry.
    let fabric_right = search
        .placed
        .iter()
        .map(|p| p.x + poses[p.pose].fabric_bbox[2])
        .fold(0.0, f64::max);
    if fabric_right > prob.strip_width() as f64 {
        prob.change_strip_width((fabric_right + EPS) as f32).expect(crate::dmsg!("strip width must be positive"));
    }
    let solution = prob.save();
    if let Some(error) = crate::structure::pipeline::solution_validation_error(instance, &solution)
    {
        return Err(format!("Column Draft failed final validation: {error:?}"));
    }
    search.report.columns = column;
    search.report.nfp_pairs = search
        .pairs
        .values()
        .filter(|v| matches!(v, PairNfp::Ready(_)))
        .count();
    search.report.nfp_failures = search
        .pairs
        .values()
        .filter(|v| matches!(v, PairNfp::Failed))
        .count();
    search.report.elapsed_ms = started.elapsed().as_millis();
    search.report.width = prob.strip_width();
    search.report.density = prob.density();
    Ok((solution, search.report))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn actual_position_path_cannot_spend_a_refinement_after_its_last_probe() {
        let ext = serde_json::from_value(serde_json::json!({"name":"cap", "strip_height":100,
            "items":[{"id":0,"demand":1,"allowed_orientations":[0],"shape":{
                "type":"simple_polygon","data":[[0,0],[20,0],[20,20],[0,20]]}}]}))
        .unwrap();
        let instance = crate::util::io::import_spp_instance(
            &ext,
            crate::config::DEFAULT_SWARM_CONFIG.cde_config,
            None,
            None,
            None,
        )
        .unwrap();
        let catalogue = PoseCatalogue::build(&instance, &rotations_from_instance(&instance), 0.0);
        let shape = instance.items[0].0.shape_cd().as_ref().clone();
        let poses = [Pose {
            item: 0,
            rotation: RotationEntry::turn(0),
            angle: 0.0,
            bbox: bbox(&shape),
            fabric_bbox: bbox(&shape),
            area: area_mm2(&shape),
            priority: 1.0,
            scoring_ring: shape
                .vertices()
                .iter()
                .map(|p| Point::new(p.0 as f64, p.1 as f64))
                .collect(),
            original: shape,
        }];
        let mut prob = SPProblem::new(instance).expect(crate::dmsg!("strip forms a valid container"));
        prob.change_strip_width(100.0).expect(crate::dmsg!("strip width must be positive"));
        let bounds = bbox(&prob.layout().container().outer_cd());
        let mut search = Search {
            poses: &poses,
            catalogue: &catalogue,
            pairs: NfpPairIndex::new(),
            placed: Vec::new(),
            spatial: BTreeMap::new(),
            bounds,
            fabric_height: 100.0,
            placed_area_mm2: 0.0,
            occupied_right: bounds[0],
            original_guard: None,
            started: Instant::now(),
            config: ColumnDraftConfig::default(),
            report: ColumnDraftReport::default(),
        };
        let mut budget = ProbeBudget {
            used: MAX_PROBES - 1,
        };
        let result = search.best_spot(
            0,
            &[],
            bounds[0],
            bounds[2],
            bounds[1],
            None,
            &mut budget,
            true,
        );
        assert!(matches!(result, Attempt::Placed(_)));
        assert_eq!(budget.used, MAX_PROBES);
        assert_eq!(search.report.candidate_attempts, 1);
        assert_eq!(
            search.report.candidate_probes, 1,
            "refinement has no budget left"
        );
    }
    #[test]
    fn position_limit_is_shared_across_rotation_attempts() {
        let mut budget = ProbeBudget::default();
        for _ in 0..3 {
            for _ in 0..4000 {
                budget.take();
            }
        }
        assert_eq!(budget.used, 10_000);
        assert!(!budget.take());
    }

    #[test]
    fn full_overlap_window_is_not_inferred_from_four_corners_alone() {
        let make = |points: &[(f64, f64)]| Boundary {
            parent: 0,
            bbox: [0., 0., 10., 10.],
            segments: points
                .iter()
                .zip(points.iter().cycle().skip(1))
                .take(points.len())
                .map(|(&(x, y), &(u, v))| (Point::new(x, y), Point::new(u, v)))
                .collect(),
        };
        let square = make(&[(0., 0.), (10., 0.), (10., 10.), (0., 10.)]);
        assert!(square.covers_window([2., 2., 8., 8.]));
        assert!(!square.covers_window([0., 0., 10., 10.])); // contact is legal
        let notch = make(&[
            (0., 0.),
            (10., 0.),
            (10., 10.),
            (6., 10.),
            (6., 5.),
            (4., 5.),
            (4., 10.),
            (0., 10.),
        ]);
        assert!(!notch.covers_window([2., 2., 8., 8.]));
    }
    #[test]
    fn contact_pair_intersection_is_not_limited_to_vertices() {
        let p = intersection(
            Point::new(0.0, 0.0),
            Point::new(10.0, 10.0),
            Point::new(0.0, 10.0),
            Point::new(10.0, 0.0),
        )
        .unwrap();
        assert_eq!((p.x, p.y), (5.0, 5.0));
    }
}
