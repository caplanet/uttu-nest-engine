//! Conservative simplification with an absolute, cumulative boundary bound.
//!
//! Same three geometric operations as Jagua: remove a concave/collinear corner,
//! or replace consecutive convex corners by the intersection of their extended
//! edges. Each edge retains the original edges it absorbed. Acceptance checks
//! continuous segments in BOTH directions against that original boundary, not
//! just vertex distances or the last operation's displacement. The capsule
//! coverage test also catches long spikes created by convex-corner extensions.
use jagua_rs::geometry::primitives::{Point, SPolygon};
use jagua_rs::geometry::shape_modification::ShapeModifyMode;
use std::rc::Rc;

#[derive(Clone, Copy, Debug)]
struct P(f64, f64);
fn sub(a: P, b: P) -> P {
    P(a.0 - b.0, a.1 - b.1)
}
fn dot(a: P, b: P) -> f64 {
    a.0 * b.0 + a.1 * b.1
}
fn cross(a: P, b: P) -> f64 {
    a.0 * b.1 - a.1 * b.0
}
fn turn(a: P, b: P, c: P) -> f64 {
    cross(sub(b, a), sub(c, b))
}
fn distance(p: P, a: P, b: P) -> f64 {
    let v = sub(b, a);
    let t = (dot(sub(p, a), v) / dot(v, v).max(1e-30)).clamp(0.0, 1.0);
    let d = sub(p, P(a.0 + t * v.0, a.1 + t * v.1));
    dot(d, d).sqrt()
}

#[derive(Clone)]
struct Node {
    p: P,
    sources: Rc<Vec<usize>>,
}
struct Candidate {
    nodes: Vec<Node>,
    affected: Vec<usize>,
    new_edges: usize,
}

// Rank a local edit without copying the whole contour for every rejected
// proposal. Only a proposal undergoing the exact checks is materialized.
struct Proposal {
    start: usize,
    removed: usize,
    new_vertex: Option<P>,
    affected: Vec<usize>,
    score: f64,
}
impl Proposal {
    fn materialize(self, nodes: &[Node]) -> Candidate {
        let inherited = Rc::new(self.affected.clone());
        let mut next = Vec::with_capacity(nodes.len() - 1);
        next.push(Node { p: nodes[self.start].p, sources: inherited.clone() });
        if let Some(p) = self.new_vertex {
            next.push(Node { p, sources: inherited });
        }
        next.extend((self.removed..nodes.len()).map(|j| nodes[(self.start + j) % nodes.len()].clone()));
        Candidate { nodes: next, affected: self.affected, new_edges: if self.new_vertex.is_some() { 2 } else { 1 } }
    }
}

fn clip_linear(lo: &mut f64, hi: &mut f64, value: f64, slope: f64, min: f64, max: f64) -> bool {
    if slope.abs() < 1e-20 {
        return value >= min && value <= max;
    }
    let a = (min - value) / slope;
    let b = (max - value) / slope;
    *lo = lo.max(a.min(b));
    *hi = hi.min(a.max(b));
    *lo <= *hi
}

/// Interval of a moving point a+t(b-a) inside a closed segment's radius-r
/// capsule. Rectangle + endpoint discs cover the capsule exactly.
fn capsule_interval(a: P, b: P, q: P, s: P, r: f64) -> Option<(f64, f64)> {
    if a.0.min(b.0) > q.0.max(s.0) + r
        || a.0.max(b.0) < q.0.min(s.0) - r
        || a.1.min(b.1) > q.1.max(s.1) + r
        || a.1.max(b.1) < q.1.min(s.1) - r
    {
        return None;
    }
    let v = sub(b, a);
    let w = sub(s, q);
    let ww = dot(w, w);
    let mut intervals = Vec::with_capacity(3);
    if ww > 1e-24 {
        let mut lo = 0.0f64;
        let mut hi = 1.0f64;
        if clip_linear(&mut lo, &mut hi, dot(sub(a, q), w), dot(v, w), 0.0, ww)
            && clip_linear(
                &mut lo,
                &mut hi,
                cross(w, sub(a, q)),
                cross(w, v),
                -r * ww.sqrt(),
                r * ww.sqrt(),
            )
        {
            intervals.push((lo, hi));
        }
    }
    let aa = dot(v, v);
    for center in [q, s] {
        let delta = sub(a, center);
        let bb = dot(delta, v);
        let cc = dot(delta, delta) - r * r;
        if aa <= 1e-24 {
            if cc <= 0.0 {
                intervals.push((0.0, 1.0));
            }
        } else {
            let disc = bb * bb - aa * cc;
            if disc >= 0.0 {
                let lo = ((-bb - disc.sqrt()) / aa).max(0.0);
                let hi = ((-bb + disc.sqrt()) / aa).min(1.0);
                if lo <= hi {
                    intervals.push((lo, hi));
                }
            }
        }
    }
    if intervals.is_empty() {
        None
    } else {
        Some((
            intervals.iter().map(|x| x.0).fold(1.0, f64::min),
            intervals.iter().map(|x| x.1).fold(0.0, f64::max),
        ))
    }
}

fn covered(a: P, b: P, edges: impl Iterator<Item = (P, P)>, r: f64) -> bool {
    let mut intervals: Vec<_> = edges
        .filter_map(|(q, s)| capsule_interval(a, b, q, s, r))
        .collect();
    intervals.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut end = 0.0;
    for (lo, hi) in intervals {
        if lo > end + 1e-11 {
            return false;
        }
        end = end.max(hi);
        if end >= 1.0 - 1e-11 {
            return true;
        }
    }
    false
}

fn certified(c: &Candidate, original: &[P], mm: f64) -> bool {
    let n = c.nodes.len();
    let m = original.len();
    for i in 0..c.new_edges {
        if !covered(
            c.nodes[i].p,
            c.nodes[(i + 1) % n].p,
            c.nodes[i]
                .sources
                .iter()
                .map(|&j| (original[j], original[(j + 1) % m])),
            mm,
        ) {
            return false;
        }
    }
    for &j in &c.affected {
        if !covered(
            original[j],
            original[(j + 1) % m],
            (0..n)
                .filter(|&i| c.nodes[i].sources.contains(&j))
                .map(|i| (c.nodes[i].p, c.nodes[(i + 1) % n].p)),
            mm,
        ) {
            return false;
        }
    }
    true
}

fn intersect(a: P, b: P, c: P, d: P) -> bool {
    let orient = |a, b, c| cross(sub(b, a), sub(c, a));
    let (u, v, w, z) = (
        orient(a, b, c),
        orient(a, b, d),
        orient(c, d, a),
        orient(c, d, b),
    );
    if u * v < 0.0 && w * z < 0.0 {
        return true;
    }
    let on = |p: P, a: P, b: P| {
        p.0 >= a.0.min(b.0) && p.0 <= a.0.max(b.0) && p.1 >= a.1.min(b.1) && p.1 <= a.1.max(b.1)
    };
    (u.abs() < 1e-12 && on(c, a, b))
        || (v.abs() < 1e-12 && on(d, a, b))
        || (w.abs() < 1e-12 && on(a, c, d))
        || (z.abs() < 1e-12 && on(b, c, d))
}

fn valid(c: &Candidate) -> bool {
    let n = c.nodes.len();
    for i in 0..c.new_edges {
        let a = c.nodes[i].p;
        let b = c.nodes[(i + 1) % n].p;
        if dot(sub(b, a), sub(b, a)) < 1e-16 {
            return false;
        }
        for j in 0..n {
            if j == i || j == (i + 1) % n || (j + 1) % n == i {
                continue;
            }
            if intersect(a, b, c.nodes[j].p, c.nodes[(j + 1) % n].p) {
                return false;
            }
        }
    }
    true
}

fn proposal(nodes: &[Node], i: usize, pair: bool, original: &[P], owners: &[Vec<usize>]) -> Option<Proposal> {
    let n = nodes.len();
    let start = (i + n - 1) % n;
    let at = |j: usize| &nodes[(start + j) % n];
    let a = at(0).p;
    let b = at(1).p;
    let c = at(2).p;
    let (removed, new_vertex) = if pair {
        let d = at(3).p;
        if turn(a, b, c) <= 0.0 || turn(b, c, d) <= 0.0 {
            return None;
        }
        let v = sub(b, a);
        let w = sub(c, d);
        let det = cross(v, w);
        if det.abs() < 1e-15 {
            return None;
        }
        let t = cross(sub(d, a), w) / det;
        let u = cross(sub(d, a), v) / det;
        if t <= 1.0 || u <= 1.0 {
            return None;
        }
        // Certify the actual f32 vertex that will enter SPolygon.
        let x = P((a.0 + t * v.0) as f32 as f64, (a.1 + t * v.1) as f32 as f64);
        // Rounding must not reverse the conservative modification direction.
        if !x.0.is_finite() || !x.1.is_finite() || turn(a, b, x) > 1e-8 || turn(x, c, d) > 1e-8 {
            return None;
        }
        (3, Some(x))
    } else {
        if turn(a, b, c) > 0.0 {
            return None;
        }
        (2, None)
    };
    let mut affected: Vec<_> = (0..removed)
        .flat_map(|j| at(j).sources.iter().copied())
        .collect();
    affected.sort_unstable();
    affected.dedup();
    let local = [a, new_vertex.unwrap_or(at(removed).p), at(removed).p];
    let new_edges = if pair { 2 } else { 1 };
    // Distance lower bound for ordering, followed by continuous certification.
    let mut score = 0.0f64;
    for &p in local.iter().take(new_edges + 1) {
        score = score.max(
            affected
                .iter()
                .map(|&j| distance(p, original[j], original[(j + 1) % original.len()]))
                .fold(f64::INFINITY, f64::min),
        );
    }
    for &j in &affected {
        score = score.max(
            (0..new_edges)
                .map(|i| distance(original[j], local[i], local[i + 1]))
                .chain(owners[j].iter().copied()
                    .filter(|&k| (k + n - start) % n >= removed)
                    .map(|k| distance(original[j], nodes[k].p, nodes[(k + 1) % n].p)))
                .fold(f64::INFINITY, f64::min),
        );
    }
    Some(Proposal {
        start,
        removed,
        new_vertex,
        affected,
        score,
    })
}

/// Superset for items, subset for containers. `mm == 0` preserves the exact
/// shape. Invalid thresholds return an error rather than silently widening it.
/// The bound is relative to this function's input (after any explicit offset).
pub fn simplify_shape_mm(
    shape: &SPolygon,
    mode: ShapeModifyMode,
    mm: f32,
) -> anyhow::Result<SPolygon> {
    anyhow::ensure!(
        mm.is_finite() && mm >= 0.0,
        "simplification maximum deviation must be finite and nonnegative"
    );
    if mm == 0.0 {
        return Ok(shape.clone());
    }
    let mut original: Vec<_> = shape
        .vertices()
        .iter()
        .map(|p| P(p.0 as f64, p.1 as f64))
        .collect();
    if mode == ShapeModifyMode::Deflate {
        original.reverse();
    }
    let mut nodes: Vec<_> = original
        .iter()
        .enumerate()
        .map(|(i, &p)| Node {
            p,
            sources: Rc::new(vec![i]),
        })
        .collect();
    while nodes.len() >= 4 {
        // Reverse provenance index: only edges actually carrying an original
        // segment can contribute to its distance. Rebuilt once per accepted
        // edit, rather than scanning every edge for every ranked proposal.
        let mut owners = vec![Vec::new(); original.len()];
        for (edge, node) in nodes.iter().enumerate() {
            for &source in node.sources.iter() { owners[source].push(edge); }
        }
        let mut candidates = Vec::new();
        for i in 0..nodes.len() {
            for pair in [false, true] {
                if let Some(c) = proposal(&nodes, i, pair, &original, &owners) {
                    if c.score <= mm as f64 {
                        candidates.push(c);
                    }
                }
            }
        }
        candidates.sort_by(|a, b| a.score.total_cmp(&b.score));
        let best = candidates
            .into_iter()
            .map(|p| p.materialize(&nodes))
            .find(|c| valid(c) && certified(c, &original, mm as f64));
        match best {
            Some(c) => nodes = c.nodes,
            None => break,
        }
    }
    let mut points: Vec<_> = nodes
        .iter()
        .map(|v| Point(v.p.0 as f32, v.p.1 as f32))
        .collect();
    if mode == ShapeModifyMode::Deflate {
        points.reverse();
    }
    Ok(SPolygon::new(points)?)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn polygon(points: &[(f32, f32)]) -> SPolygon {
        SPolygon::new(points.iter().map(|&(x, y)| Point(x, y)).collect()).unwrap()
    }
    #[test]
    fn continuous_segments_not_just_endpoints_are_certified() {
        assert!(!covered(
            P(0.0, 0.0),
            P(10.0, 0.0),
            [(P(0.0, 0.0), P(0.0, 5.0)), (P(10.0, 0.0), P(10.0, 5.0))].into_iter(),
            1.0
        ));
        assert!(covered(
            P(0.0, 0.0),
            P(10.0, 0.0),
            [(P(0.0, 0.2), P(10.0, 0.2))].into_iter(),
            0.3
        ));
    }
    #[test]
    fn corner_merge_and_absolute_scale_limit() {
        let p = polygon(&[(0., 0.), (10., 0.), (10., 9.8), (9.8, 10.), (0., 10.)]);
        let s = simplify_shape_mm(&p, ShapeModifyMode::Inflate, 0.3).unwrap();
        assert_eq!(s.vertices().len(), 4);
        assert!(s.area() >= p.area());
        let big = polygon(&[(0., 0.), (100., 0.), (100., 98.), (98., 100.), (0., 100.)]);
        assert_eq!(
            simplify_shape_mm(&big, ShapeModifyMode::Inflate, 0.3)
                .unwrap()
                .vertices()
                .len(),
            5
        );
    }
    #[test]
    fn concave_removal_and_deflation_are_opposite() {
        let p = polygon(&[(0., 0.), (10., 0.), (10., 10.), (5., 9.8), (0., 10.)]);
        assert_eq!(
            simplify_shape_mm(&p, ShapeModifyMode::Inflate, 0.3)
                .unwrap()
                .vertices()
                .len(),
            4
        );
        let d = simplify_shape_mm(&p, ShapeModifyMode::Deflate, 0.3).unwrap();
        assert!(d.area() <= p.area());
        assert_eq!(
            simplify_shape_mm(&p, ShapeModifyMode::Inflate, 0.)
                .unwrap()
                .vertices(),
            p.vertices()
        );
        assert!(simplify_shape_mm(&p, ShapeModifyMode::Inflate, f32::NAN).is_err());
    }
}
