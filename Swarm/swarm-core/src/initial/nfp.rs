//! NFP(No-Fit Polygon) 생성 + 캐시.
//!
//! jagua-rs 0.7.2 에는 민코프스키 합이 없다. 여기서 직접 만든다.
//!
//! `NFP(고정 도형 B, 이동 도형 A) = B ⊕ (−A)`
//! 결과 폴리곤의 경계 위 점은 "A 의 로컬 원점을 그 좌표에 두었을 때
//! A 와 B 가 접하되 겹치지 않는" 위치다. 즉 후보 평행이동 그 자체다.

use std::collections::HashMap;
use std::sync::Arc;

use jagua_rs::geometry::convex_hull::convex_hull_from_points;
use jagua_rs::geometry::primitives::{Point, SPolygon};

/// NFP 한 개. F단계에서 비볼록 NFP를 볼록조각 배열로 유지하기 위해
/// B단계부터 조각 배열 형태로 둔다(B단계에서는 항상 조각 1개).
#[derive(Clone, Debug, Default)]
pub struct Nfp {
    pub pieces: Vec<Vec<Point>>,
    /// 조각 배열 경계에서 뽑은 후보 평행이동 점.
    /// **다른 조각의 내부에 있는 점은 제거되어 있다** — 합집합 폴리곤을 만들지 않고도
    /// 실제 NFP 경계 위의 점만 남긴다. 캐시에 넣기 전 한 번만 계산한다.
    pub cands: Vec<Point>,
}

impl Nfp {
    pub fn n_verts(&self) -> usize {
        self.pieces.iter().map(|p| p.len()).sum()
    }
    pub fn is_empty(&self) -> bool {
        self.pieces.is_empty()
    }
    /// 부호 반전본. `NFP(A,B) = −NFP(B,A)` 재사용에 쓴다.
    pub fn negated(&self) -> Nfp {
        Nfp {
            pieces: self
                .pieces
                .iter()
                .map(|p| p.iter().map(|q| Point(-q.0, -q.1)).collect())
                .collect(),
            cands: self.cands.iter().map(|q| Point(-q.0, -q.1)).collect(),
        }
    }
}

/// 키: (도형 등가 클래스 b, 회전 b, 도형 등가 클래스 a, 회전 a).
/// 도형이 같으면 NFP도 같다 -> 아이템 id가 아니라 shape 등가 클래스로 키를 만든다.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct NfpKey {
    pub class_b: u32,
    pub rot_b: u16,
    pub class_a: u32,
    pub rot_a: u16,
}

impl NfpKey {
    fn swapped(self) -> NfpKey {
        NfpKey {
            class_b: self.class_a,
            rot_b: self.rot_a,
            class_a: self.class_b,
            rot_a: self.rot_b,
        }
    }
}

#[derive(Default)]
pub struct NfpCache {
    map: HashMap<NfpKey, Arc<Nfp>>,
    pub hits: u64,
    pub misses: u64,
    pub verts_total: u64,
    pub count: u64,
    pub pieces_total: u64,
    pub hull_fallbacks: u64,
}

impl NfpCache {
    pub fn new() -> Self {
        Self::default()
    }

    /// 조회 실패 시 `build`로 계산해 삽입한다(lazy). 사전 전량 계산은 하지 않는다.
    /// 실패 시 `NFP(A,B) = −NFP(B,A)` 로 역방향 키도 함께 저장해, 이후 역방향
    /// 적중에서 매번 부호 반전본을 새로 할당하지 않는다. `count`/`verts_total`
    /// 은 실제로 계산한 NFP 1개만 센다.
    pub fn get_or_build<F>(&mut self, key: NfpKey, build: F) -> Arc<Nfp>
    where
        F: FnOnce() -> Nfp,
    {
        if let Some(v) = self.map.get(&key) {
            self.hits += 1;
            return v.clone();
        }
        self.misses += 1;
        let built = build();
        self.count += 1;
        self.verts_total += built.n_verts() as u64;
        self.pieces_total += built.pieces.len() as u64;
        let arc = Arc::new(built);
        let rkey = key.swapped();
        if rkey != key {
            self.map.insert(rkey, Arc::new(arc.negated()));
        }
        self.map.insert(key, arc.clone());
        arc
    }

    pub fn hit_rate(&self) -> f64 {
        let t = self.hits + self.misses;
        if t == 0 { 0.0 } else { self.hits as f64 / t as f64 }
    }
}

// ---------------------------------------------------------------------------
// 볼록 기하
// ---------------------------------------------------------------------------

pub fn signed_area(pts: &[Point]) -> f32 {
    let n = pts.len();
    if n < 3 {
        return 0.0;
    }
    let mut s = 0.0f32;
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        s += a.0 * b.1 - b.0 * a.1;
    }
    s * 0.5
}

/// 반시계(CCW) 방향으로 맞춘다.
pub fn to_ccw(mut pts: Vec<Point>) -> Vec<Point> {
    if signed_area(&pts) < 0.0 {
        pts.reverse();
    }
    pts
}

/// 도형의 볼록 껍질을 CCW 로 반환.
pub fn hull_ccw(shape: &SPolygon) -> Vec<Point> {
    to_ccw(dedup(convex_hull_from_points(shape.vertices().to_vec())))
}

/// −A: 정점 부호 반전(방향이 뒤집히므로 CCW 로 되돌린다).
pub fn negate_ccw(pts: &[Point]) -> Vec<Point> {
    to_ccw(pts.iter().map(|p| Point(-p.0, -p.1)).collect())
}

pub fn dedup_pub(pts: Vec<Point>) -> Vec<Point> {
    dedup(pts)
}

fn dedup(mut pts: Vec<Point>) -> Vec<Point> {
    pts.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-6 && (a.1 - b.1).abs() < 1e-6);
    if pts.len() >= 2 {
        let f = pts[0];
        let l = pts[pts.len() - 1];
        if (f.0 - l.0).abs() < 1e-6 && (f.1 - l.1).abs() < 1e-6 {
            pts.pop();
        }
    }
    pts
}

#[inline(always)]
fn cross(ax: f32, ay: f32, bx: f32, by: f32) -> f32 {
    ax * by - ay * bx
}

/// 최하단-최좌측 정점의 인덱스.
fn start_index(p: &[Point]) -> usize {
    let mut k = 0usize;
    for i in 1..p.len() {
        if p[i].1 < p[k].1 || (p[i].1 == p[k].1 && p[i].0 < p[k].0) {
            k = i;
        }
    }
    k
}

/// NFP(고정 도형 B, 이동 도형 A) = B ⊕ (−A)
/// 볼록 x 볼록이면 에지 각도 병합으로 O(n+m)에 정확히 나온다.
/// 두 입력 모두 CCW 여야 하고, `a_hull_neg` 는 이미 부호 반전된 CCW 껍질이다.
/// 각도 비교는 외적 부호로만 하고 삼각함수를 쓰지 않는다. 결과는 CCW.
pub fn nfp_convex(b_hull: &[Point], a_hull_neg: &[Point]) -> Vec<Point> {
    let n = b_hull.len();
    let m = a_hull_neg.len();
    if n < 3 || m < 3 {
        return Vec::new();
    }
    let ib = start_index(b_hull);
    let ia = start_index(a_hull_neg);
    let bp = |k: usize| b_hull[(ib + k) % n];
    let ap = |k: usize| a_hull_neg[(ia + k) % m];

    let mut out: Vec<Point> = Vec::with_capacity(n + m);
    let (mut i, mut j) = (0usize, 0usize);
    while i < n || j < m {
        let (p, q) = (bp(i), ap(j));
        out.push(Point(p.0 + q.0, p.1 + q.1));
        let eb = {
            let a = bp(i);
            let b = bp(i + 1);
            (b.0 - a.0, b.1 - a.1)
        };
        let ea = {
            let a = ap(j);
            let b = ap(j + 1);
            (b.0 - a.0, b.1 - a.1)
        };
        let c = cross(eb.0, eb.1, ea.0, ea.1);
        let can_b = i < n;
        let can_a = j < m;
        if can_b && can_a {
            if c > 0.0 {
                i += 1;
            } else if c < 0.0 {
                j += 1;
            } else {
                i += 1;
                j += 1;
            }
        } else if can_b {
            i += 1;
        } else {
            j += 1;
        }
    }
    let out = dedup(out);
    if out.len() < 3 { Vec::new() } else { out }
}

/// CCW 볼록 폴리곤을 바깥으로 `eps` 만큼 민다(정점별 인접 에지 법선 평균).
/// NFP 경계는 "정확히 접하는" 위치라 그대로 쓰면 CDE 경계 판정이 축퇴한다.
/// 캐시에 넣기 전에 한 번만 적용한다(사용할 때마다 하면 할당이 폭발한다).
pub fn inflate_ccw(pts: &[Point], eps: f32) -> Vec<Point> {
    let n = pts.len();
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let prev = pts[(i + n - 1) % n];
        let cur = pts[i];
        let next = pts[(i + 1) % n];
        // CCW 에서 에지 (a->b) 의 바깥 법선은 (dy, -dx)
        let n1 = unit(cur.1 - prev.1, -(cur.0 - prev.0));
        let n2 = unit(next.1 - cur.1, -(next.0 - cur.0));
        let (nx, ny) = unit(n1.0 + n2.0, n1.1 + n2.1);
        out.push(Point(cur.0 + nx * eps, cur.1 + ny * eps));
    }
    out
}

#[inline(always)]
fn unit(x: f32, y: f32) -> (f32, f32) {
    let l = (x * x + y * y).sqrt();
    if l > 1e-9 { (x / l, y / l) } else { (0.0, 0.0) }
}

/// 볼록 껍질 두 개로 NFP 한 조각을 만든다(바깥으로 `eps` 만큼 부풀린 상태로).
pub fn nfp_hull_pair(b_hull: &[Point], a_hull_neg: &[Point], eps: f32, splits: u32) -> Nfp {
    let p = nfp_convex(b_hull, a_hull_neg);
    if p.len() < 3 {
        return Nfp::default();
    }
    let raw = vec![p];
    let pieces = vec![inflate_ccw(&raw[0], eps)];
    let cands = union_boundary_cands(&raw, &pieces, splits);
    Nfp { pieces, cands }
}

/// 조각 배열의 **합집합 경계** 위 후보 점을 뽑는다.
/// 합집합 폴리곤은 만들지 않는다. 각 조각의 경계 표본점 중
/// 다른 조각의 내부에 들어가는 것만 버린다.
pub fn union_boundary_cands(
    raw: &[Vec<Point>],
    inflated: &[Vec<Point>],
    splits: u32,
) -> Vec<Point> {
    let n = raw.len();
    let boxes: Vec<(f32, f32, f32, f32)> = raw
        .iter()
        .map(|q| {
            let mut b = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);
            for t in q {
                b.0 = b.0.min(t.0);
                b.1 = b.1.min(t.1);
                b.2 = b.2.max(t.0);
                b.3 = b.3.max(t.1);
            }
            b
        })
        .collect();
    let splits = splits.max(1);
    let mut out: Vec<Point> = Vec::new();
    for i in 0..n {
        let poly = &inflated[i];
        let m = poly.len();
        if m < 3 {
            continue;
        }
        for k in 0..m {
            let a = poly[k];
            let b = poly[(k + 1) % m];
            for t in 0..splits {
                let f = t as f32 / splits as f32;
                let pt = Point(a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f);
                let mut buried = false;
                for j in 0..n {
                    if j == i {
                        continue;
                    }
                    let bb = boxes[j];
                    if pt.0 < bb.0 || pt.0 > bb.2 || pt.1 < bb.1 || pt.1 > bb.3 {
                        continue;
                    }
                    if strictly_inside_convex(&raw[j], pt) {
                        buried = true;
                        break;
                    }
                }
                if !buried {
                    out.push(pt);
                }
            }
        }
    }
    out
}


// ---------------------------------------------------------------------------
// F단계: 비볼록 도형의 볼록 분해 + 조각 배열 NFP
// ---------------------------------------------------------------------------

/// 볼록 조각 병합에서 허용하는 "도형 밖으로 넘치는 면적" 비율 상한.
/// 이 값을 넘기면 그 도형은 껍질 근사로 폴백한다.
// ponytail: 2%. 폴백 비율이 높게 나오면 그때 올린다.
pub const MERGE_SLACK_RATIO: f32 = 0.02;

#[inline(always)]
fn cross3(a: Point, b: Point, c: Point) -> f32 {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

fn point_in_triangle(p: Point, a: Point, b: Point, c: Point) -> bool {
    let d1 = cross3(a, b, p);
    let d2 = cross3(b, c, p);
    let d3 = cross3(c, a, p);
    let neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    let pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    !(neg && pos)
}

/// CCW 단순 다각형의 ear clipping 삼각분할. 인덱스 삼각형 배열을 돌려준다.
fn ear_clip(poly: &[Point]) -> Vec<[usize; 3]> {
    let n = poly.len();
    let mut out = Vec::with_capacity(n.saturating_sub(2));
    if n < 3 {
        return out;
    }
    let mut idx: Vec<usize> = (0..n).collect();
    let mut guard = 0usize;
    while idx.len() > 3 {
        guard += 1;
        if guard > 4 * n {
            break; // 축퇴 입력 방어
        }
        let m = idx.len();
        let mut clipped = false;
        for k in 0..m {
            let (i0, i1, i2) = (idx[(k + m - 1) % m], idx[k], idx[(k + 1) % m]);
            let (a, b, c) = (poly[i0], poly[i1], poly[i2]);
            if cross3(a, b, c) <= 0.0 {
                continue; // reflex 또는 일직선
            }
            let mut ok = true;
            for &j in idx.iter() {
                if j == i0 || j == i1 || j == i2 {
                    continue;
                }
                if point_in_triangle(poly[j], a, b, c) {
                    ok = false;
                    break;
                }
            }
            if ok {
                out.push([i0, i1, i2]);
                idx.remove(k);
                clipped = true;
                break;
            }
        }
        if !clipped {
            break;
        }
    }
    for k in 1..idx.len().saturating_sub(1) {
        out.push([idx[0], idx[k], idx[k + 1]]);
    }
    out
}

fn is_convex_ccw(pts: &[Point]) -> bool {
    let n = pts.len();
    if n < 3 {
        return false;
    }
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        let c = pts[(i + 2) % n];
        if cross3(a, b, c) < -1e-4 {
            return false;
        }
    }
    true
}

#[inline(always)]
fn same(a: Point, b: Point) -> bool {
    (a.0 - b.0).abs() < 1e-5 && (a.1 - b.1).abs() < 1e-5
}

/// 두 조각이 공유 에지를 가지면 병합된 정점 고리를 돌려준다.
fn merge_on_shared_edge(p: &[Point], q: &[Point]) -> Option<Vec<Point>> {
    let (np, nq) = (p.len(), q.len());
    for i in 0..np {
        let a = p[i];
        let b = p[(i + 1) % np];
        for j in 0..nq {
            let c = q[j];
            let d = q[(j + 1) % nq];
            // 공유 에지는 방향이 반대다: (a,b) == (d,c)
            if same(a, d) && same(b, c) {
                let mut out = Vec::with_capacity(np + nq - 2);
                for k in 0..np {
                    out.push(p[(i + 1 + k) % np]);
                }
                for k in 1..nq {
                    out.push(q[(j + 1 + k) % nq]);
                }
                return Some(dedup(out));
            }
        }
    }
    None
}

/// 볼록 분해. ear clipping 후 인접 삼각형을 볼록성이 유지되는 한 병합한다.
/// 조각 수가 `cap` 을 넘으면 인접 쌍을 껍질 병합으로 더 줄인다(도형 밖으로
/// 넘치는 면적이 `MERGE_SLACK_RATIO` 를 넘으면 None -> 껍질 폴백).
// ponytail: 삼각분할+병합. 조각 수가 성능을 먹으면 그때 최적 분해.
pub fn decompose_convex(poly: &[Point], cap: usize) -> Option<Vec<Vec<Point>>> {
    let poly = to_ccw(dedup(poly.to_vec()));
    if poly.len() < 3 {
        return None;
    }
    let tris = ear_clip(&poly);
    if tris.is_empty() {
        return None;
    }
    let mut pieces: Vec<Vec<Point>> = tris
        .iter()
        .map(|t| vec![poly[t[0]], poly[t[1]], poly[t[2]]])
        .collect();

    // 1) Hertel-Mehlhorn: 볼록성을 지키며 인접 조각 병합
    let mut changed = true;
    while changed {
        changed = false;
        let mut done = false;
        for i in 0..pieces.len() {
            if done {
                break;
            }
            for j in (i + 1)..pieces.len() {
                if let Some(m) = merge_on_shared_edge(&pieces[i], &pieces[j]) {
                    if m.len() >= 3 && is_convex_ccw(&m) {
                        pieces[i] = m;
                        pieces.remove(j);
                        changed = true;
                        done = true;
                        break;
                    }
                }
            }
        }
    }

    // 2) 조각 수 상한: 인접 쌍을 껍질로 합치되 넘치는 면적을 누적 감시
    let shape_area = signed_area(&poly).abs();
    let mut slack = 0.0f32;
    while pieces.len() > cap {
        let mut best: Option<(usize, usize, Vec<Point>, f32)> = None;
        for i in 0..pieces.len() {
            for j in (i + 1)..pieces.len() {
                if merge_on_shared_edge(&pieces[i], &pieces[j]).is_none() {
                    continue;
                }
                let mut pts = pieces[i].clone();
                pts.extend_from_slice(&pieces[j]);
                let h = to_ccw(dedup(convex_hull_from_points(pts)));
                if h.len() < 3 {
                    continue;
                }
                let add = signed_area(&h).abs()
                    - signed_area(&pieces[i]).abs()
                    - signed_area(&pieces[j]).abs();
                if best.as_ref().map_or(true, |(_, _, _, b)| add < *b) {
                    best = Some((i, j, h, add));
                }
            }
        }
        let Some((i, j, h, add)) = best else { break };
        slack += add.max(0.0);
        if slack > shape_area * MERGE_SLACK_RATIO {
            return None; // 껍질 폴백
        }
        pieces[i] = h;
        pieces.remove(j);
    }

    if pieces.len() > cap {
        return None;
    }
    for pc in pieces.iter_mut() {
        *pc = to_ccw(std::mem::take(pc));
    }
    Some(pieces)
}

/// 조각 배열끼리의 NFP. 합집합을 계산하지 않고 조각 Vec 으로 그대로 둔다.
/// `NFP(B, A) = union over (b_piece, a_piece) of  b_piece + (-a_piece)`
pub fn nfp_piece_array(
    b_pieces: &[Vec<Point>],
    a_pieces_neg: &[Vec<Point>],
    eps: f32,
    splits: u32,
) -> Nfp {
    let mut raw = Vec::with_capacity(b_pieces.len() * a_pieces_neg.len());
    for b in b_pieces {
        for a in a_pieces_neg {
            let pc = nfp_convex(b, a);
            if pc.len() >= 3 {
                raw.push(pc);
            }
        }
    }
    let pieces: Vec<Vec<Point>> = raw.iter().map(|q| inflate_ccw(q, eps)).collect();
    let cands = union_boundary_cands(&raw, &pieces, splits);
    Nfp { pieces, cands }
}

/// 볼록 폴리곤(CCW) 내부 판정 (경계 제외).
pub fn strictly_inside_convex(poly: &[Point], p: Point) -> bool {
    let n = poly.len();
    if n < 3 {
        return false;
    }
    for i in 0..n {
        if cross3(poly[i], poly[(i + 1) % n], p) <= 1e-6 {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sq(cx: f32, cy: f32, h: f32) -> Vec<Point> {
        to_ccw(vec![
            Point(cx - h, cy - h),
            Point(cx + h, cy - h),
            Point(cx + h, cy + h),
            Point(cx - h, cy + h),
        ])
    }

    fn bbox_of(p: &[Point]) -> (f32, f32, f32, f32) {
        let mut r = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);
        for q in p {
            r.0 = r.0.min(q.0);
            r.1 = r.1.min(q.1);
            r.2 = r.2.max(q.0);
            r.3 = r.3.max(q.1);
        }
        r
    }

    /// 정사각형 ⊕ −정사각형 = 2배 정사각형
    #[test]
    fn square_minkowski_is_double_square() {
        let b = sq(0.0, 0.0, 1.0); // 2x2
        let a = sq(0.0, 0.0, 1.0);
        let a_neg = negate_ccw(&a);
        let nfp = nfp_convex(&b, &a_neg);
        assert_eq!(nfp.len(), 4, "NFP 정점 4개여야 한다: {nfp:?}");
        let (x0, y0, x1, y1) = bbox_of(&nfp);
        assert!((x0 + 2.0).abs() < 1e-4, "x0={x0}");
        assert!((y0 + 2.0).abs() < 1e-4, "y0={y0}");
        assert!((x1 - 2.0).abs() < 1e-4, "x1={x1}");
        assert!((y1 - 2.0).abs() < 1e-4, "y1={y1}");
        assert!(
            (signed_area(&nfp) - 16.0).abs() < 1e-3,
            "면적 16 이어야 한다: {}",
            signed_area(&nfp)
        );
    }

    /// 중심이 어긋난 정사각형: NFP 는 B중심 − A중심 만큼 평행이동한다.
    #[test]
    fn square_offset_shifts_nfp() {
        let b = sq(10.0, 5.0, 1.0);
        let a = sq(0.0, 0.0, 1.0);
        let nfp = nfp_convex(&b, &negate_ccw(&a));
        let (x0, y0, x1, y1) = bbox_of(&nfp);
        assert!((x0 - 8.0).abs() < 1e-4, "x0={x0}");
        assert!((y0 - 3.0).abs() < 1e-4, "y0={y0}");
        assert!((x1 - 12.0).abs() < 1e-4, "x1={x1}");
        assert!((y1 - 7.0).abs() < 1e-4, "y1={y1}");
    }

    /// 삼각형 쌍 손검증.
    /// B = (0,0),(4,0),(0,3)  A = (0,0),(1,0),(0,1)
    /// −A = (0,0),(−1,0),(0,−1)
    /// B ⊕ (−A) 는 볼록 6각형이고 면적 = area(B)+area(−A)+혼합항 이다.
    /// 민코프스키 합 면적 = |B| + |A| + 2*mixedarea. 여기서는 정점을 직접 검증한다.
    #[test]
    fn triangle_pair_by_hand() {
        let b = to_ccw(vec![Point(0.0, 0.0), Point(4.0, 0.0), Point(0.0, 3.0)]);
        let a = to_ccw(vec![Point(0.0, 0.0), Point(1.0, 0.0), Point(0.0, 1.0)]);
        let nfp = nfp_convex(&b, &negate_ccw(&a));
        // 기대 정점 집합 (B의 정점 + −A의 정점 조합 중 껍질 위에 남는 것)
        let expect = [
            Point(-1.0, 0.0),
            Point(0.0, -1.0),
            Point(4.0, -1.0),
            Point(4.0, 0.0),
            Point(0.0, 3.0),
            Point(-1.0, 3.0),
        ];
        assert_eq!(nfp.len(), expect.len(), "정점 수 불일치: {nfp:?}");
        for e in expect.iter() {
            assert!(
                nfp.iter()
                    .any(|p| (p.0 - e.0).abs() < 1e-4 && (p.1 - e.1).abs() < 1e-4),
                "기대 정점 {e:?} 가 없다: {nfp:?}"
            );
        }
        // CCW 확인
        assert!(signed_area(&nfp) > 0.0, "CCW 가 아니다");
        // 면적: 6각형 넓이를 직접 계산 = 20 - 0.5(좌상 잘림) ... shoelace 로 검증
        let expect_area = signed_area(&to_ccw(expect.to_vec()));
        assert!(
            (signed_area(&nfp) - expect_area).abs() < 1e-3,
            "면적 불일치 {} vs {}",
            signed_area(&nfp),
            expect_area
        );
    }

    /// 캐시: 역방향 재사용은 부호 반전이어야 한다.
    #[test]
    fn cache_reverse_reuse_negates() {
        let mut c = NfpCache::new();
        let b = sq(10.0, 5.0, 1.0);
        let a = sq(0.0, 0.0, 1.0);
        let k = NfpKey {
            class_b: 0,
            rot_b: 0,
            class_a: 1,
            rot_a: 0,
        };
        let fwd = c.get_or_build(k, || nfp_hull_pair(&b, &negate_ccw(&a), 0.0, 1));
        let rev = c.get_or_build(k.swapped(), || panic!("{}", crate::dmsg!("역방향은 캐시에서 나와야 한다")));
        assert_eq!(c.misses, 1);
        assert_eq!(c.hits, 1);
        assert_eq!(c.count, 1, "실제 계산은 1개만 센다");
        // 역방향 적중은 저장된 Arc 를 그대로 돌려준다 (매번 부호 반전본을 만들지 않는다)
        let rev2 = c.get_or_build(k.swapped(), || panic!("{}", crate::dmsg!("역방향은 캐시에서 나와야 한다")));
        assert!(Arc::ptr_eq(&rev, &rev2));
        assert_eq!(c.hits, 2);
        let f = &fwd.pieces[0];
        let r = &rev.pieces[0];
        assert_eq!(f.len(), r.len());
        for p in f.iter() {
            assert!(
                r.iter()
                    .any(|q| (q.0 + p.0).abs() < 1e-4 && (q.1 + p.1).abs() < 1e-4),
                "부호 반전이 아니다"
            );
        }
    }

    /// F단계 존재 이유: ㄴ자 폴리곤 + (-정사각형) 에서 오목부 안쪽 후보가 생겨야 한다.
    /// (껍질 NFP 내부 = 껍질 버전에서는 만들 수 없던 위치)
    #[test]
    fn l_shape_concave_pocket_has_candidate() {
        let l = to_ccw(vec![
            Point(0.0, 0.0),
            Point(3.0, 0.0),
            Point(3.0, 1.0),
            Point(1.0, 1.0),
            Point(1.0, 3.0),
            Point(0.0, 3.0),
        ]);
        let sqr = sq(0.0, 0.0, 0.4);
        let sqr_neg = negate_ccw(&sqr);

        let hull_l = to_ccw(convex_hull_from_points(l.clone()));
        let hull_nfp = nfp_convex(&hull_l, &sqr_neg);
        assert!(hull_nfp.len() >= 3);

        let pieces = decompose_convex(&l, 8).expect(crate::dmsg!("ㄴ자는 분해되어야 한다"));
        assert!(pieces.len() >= 2, "ㄴ자가 볼록 1조각으로 나왔다: {pieces:?}");
        let neg_pieces = vec![sqr_neg.clone()];
        let nfp = nfp_piece_array(&pieces, &neg_pieces, 0.0, 1);
        assert!(!nfp.is_empty());

        let mut found = None;
        for piece in nfp.pieces.iter() {
            for pt in piece.iter() {
                if strictly_inside_convex(&hull_nfp, *pt) {
                    found = Some(*pt);
                    break;
                }
            }
            if found.is_some() {
                break;
            }
        }
        assert!(
            found.is_some(),
            "오목부 안쪽 후보가 생기지 않았다. pieces={}",
            nfp.pieces.len()
        );
        let c = found.unwrap();
        assert!(c.0 > 1.0 || c.1 > 1.0, "찾은 후보가 오목부 쪽이 아니다: {c:?}");
    }

    /// 볼록 도형은 1조각으로 분해되어야 한다.
    #[test]
    fn convex_decomposes_to_one_piece() {
        let s = sq(0.0, 0.0, 1.0);
        let pcs = decompose_convex(&s, 8).unwrap();
        assert_eq!(pcs.len(), 1, "정사각형은 1조각: {pcs:?}");
    }
}
