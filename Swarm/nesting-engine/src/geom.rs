//! 격자 정수 기하 (§0).
//!
//! 길이 단위 mm, 내부 표현은 0.01mm 격자의 `i64`. 외적·면적은 `i128` 로 계산해 넘침을
//! 없앤다(§13.1 "`i32 grid²` 는 약 214,748mm² 에서 넘친다"). 점 판정은 전부 정확한 정수
//! 연산이라, 접촉(경계 위)과 겹침(내부)을 오차 없이 가른다.

/// 1mm 당 격자 칸 수 (§0: 격자 = 0.01mm).
pub const GRID_PER_MM: f64 = 100.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub struct Pt {
    pub x: i64,
    pub y: i64,
}

impl Pt {
    pub const fn new(x: i64, y: i64) -> Self {
        Self { x, y }
    }
}

impl std::ops::Add for Pt {
    type Output = Pt;
    fn add(self, o: Pt) -> Pt {
        Pt::new(self.x + o.x, self.y + o.y)
    }
}

impl std::ops::Sub for Pt {
    type Output = Pt;
    fn sub(self, o: Pt) -> Pt {
        Pt::new(self.x - o.x, self.y - o.y)
    }
}

impl std::ops::Neg for Pt {
    type Output = Pt;
    fn neg(self) -> Pt {
        Pt::new(-self.x, -self.y)
    }
}

#[inline]
pub fn cross(a: Pt, b: Pt) -> i128 {
    a.x as i128 * b.y as i128 - a.y as i128 * b.x as i128
}

#[inline]
pub fn dot(a: Pt, b: Pt) -> i128 {
    a.x as i128 * b.x as i128 + a.y as i128 * b.y as i128
}

/// `c` 가 `a→b` 의 왼쪽이면 양수, 오른쪽이면 음수, 한 직선 위면 0.
#[inline]
pub fn orient(a: Pt, b: Pt, c: Pt) -> i128 {
    cross(b - a, c - a)
}

/// 부호 있는 넓이의 두 배. 반시계(CCW) 가 양수.
pub fn area2(poly: &[Pt]) -> i128 {
    let n = poly.len();
    (0..n).map(|i| cross(poly[i], poly[(i + 1) % n])).sum()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BBox {
    pub x0: i64,
    pub y0: i64,
    pub x1: i64,
    pub y1: i64,
}

impl BBox {
    pub fn of(poly: &[Pt]) -> BBox {
        let mut b = BBox { x0: i64::MAX, y0: i64::MAX, x1: i64::MIN, y1: i64::MIN };
        for p in poly {
            b.x0 = b.x0.min(p.x);
            b.y0 = b.y0.min(p.y);
            b.x1 = b.x1.max(p.x);
            b.y1 = b.y1.max(p.y);
        }
        b
    }

    pub fn translate(self, d: Pt) -> BBox {
        BBox { x0: self.x0 + d.x, y0: self.y0 + d.y, x1: self.x1 + d.x, y1: self.y1 + d.y }
    }

    /// 닫힌 사각형끼리 교차(맞닿음 포함).
    pub fn overlaps(&self, o: &BBox) -> bool {
        self.x0 <= o.x1 && o.x0 <= self.x1 && self.y0 <= o.y1 && o.y0 <= self.y1
    }

    pub fn contains(&self, p: Pt) -> bool {
        self.x0 <= p.x && p.x <= self.x1 && self.y0 <= p.y && p.y <= self.y1
    }

    pub fn center(&self) -> Pt {
        Pt::new((self.x0 + self.x1) / 2, (self.y0 + self.y1) / 2)
    }
}

/// 점이 선분 `a-b` 위(끝점 포함)에 있는가.
#[inline]
pub fn on_segment(p: Pt, a: Pt, b: Pt) -> bool {
    orient(a, b, p) == 0
        && p.x >= a.x.min(b.x)
        && p.x <= a.x.max(b.x)
        && p.y >= a.y.min(b.y)
        && p.y <= a.y.max(b.y)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Loc {
    Inside,
    Outside,
    Boundary,
}

/// 단순 다각형(방향 무관)에 대한 점 위치. 경계 위면 `Boundary`.
pub fn locate(p: Pt, poly: &[Pt]) -> Loc {
    let n = poly.len();
    let mut inside = false;
    for i in 0..n {
        let a = poly[i];
        let b = poly[(i + 1) % n];
        if on_segment(p, a, b) {
            return Loc::Boundary;
        }
        // 반열린 규칙으로 수평 광선 교차를 센다 — 꼭짓점을 두 번 세지 않는다.
        if (a.y > p.y) != (b.y > p.y) {
            let o = orient(a, b, p);
            // a→b 가 위로 가면(b.y > a.y) 교차점이 p 의 오른쪽일 때 o > 0.
            if (b.y > a.y) == (o > 0) {
                inside = !inside;
            }
        }
    }
    if inside { Loc::Inside } else { Loc::Outside }
}

/// 중복 꼭짓점·일직선 꼭짓점·180° 가시(spike)를 지우고 반시계로 맞춘다.
///
/// 가시와 일직선 꼭짓점은 NFP 합성곱에서 방향 원뿔이 비거나(0°) 정의되지 않게(180°)
/// 만든다. 진행이 없을 때까지 반복한다(§3.3.2-4 "퇴화 모서리 정리").
pub fn cleanup(poly: &[Pt]) -> Vec<Pt> {
    let mut v: Vec<Pt> = poly.to_vec();
    loop {
        let n = v.len();
        if n < 3 {
            return v;
        }
        let mut out = Vec::with_capacity(n);
        let mut changed = false;
        for i in 0..n {
            let prev = v[(i + n - 1) % n];
            let cur = v[i];
            let next = v[(i + 1) % n];
            if cur == prev {
                changed = true;
                continue;
            }
            if orient(prev, cur, next) == 0 {
                // 일직선(같은 방향) 또는 가시(반대 방향) — 둘 다 지운다.
                changed = true;
                continue;
            }
            out.push(cur);
        }
        v = out;
        if !changed {
            break;
        }
    }
    if area2(&v) < 0 {
        v.reverse();
    }
    v
}

pub fn translate(poly: &[Pt], d: Pt) -> Vec<Pt> {
    poly.iter().map(|&p| p + d).collect()
}

/// 선분 `a-b` 와 `c-d` 의 교점. 평행이거나 만나지 않으면 `None`.
/// 교점은 일반적으로 격자 위에 있지 않으므로 f64 로 돌려준다.
pub fn segment_intersection(a: Pt, b: Pt, c: Pt, d: Pt) -> Option<(f64, f64)> {
    let r = b - a;
    let s = d - c;
    let den = cross(r, s);
    if den == 0 {
        return None;
    }
    let qp = c - a;
    let t_num = cross(qp, s);
    let u_num = cross(qp, r);
    let (t_ok, u_ok) = if den > 0 {
        (t_num >= 0 && t_num <= den, u_num >= 0 && u_num <= den)
    } else {
        (t_num <= 0 && t_num >= den, u_num <= 0 && u_num >= den)
    };
    if !(t_ok && u_ok) {
        return None;
    }
    let t = t_num as f64 / den as f64;
    Some((a.x as f64 + t * r.x as f64, a.y as f64 + t * r.y as f64))
}

/// 다각형 중 `x <= x_max` 인 부분의 넓이(격자²). §3.5.6 의 "경계 클리핑 보정"
/// — 사다리꼴 적분으로, 현재 스트립 폭을 넘는 부분은 넓이에 넣지 않는다.
pub fn area_left_of(poly: &[Pt], x_max: i64) -> f64 {
    let n = poly.len();
    let xm = x_max as f64;
    let mut clipped: Vec<(f64, f64)> = Vec::with_capacity(n + 4);
    for i in 0..n {
        let a = poly[i];
        let b = poly[(i + 1) % n];
        let (ax, ay, bx, by) = (a.x as f64, a.y as f64, b.x as f64, b.y as f64);
        let a_in = ax <= xm;
        let b_in = bx <= xm;
        if a_in {
            clipped.push((ax, ay));
        }
        if a_in != b_in {
            let t = (xm - ax) / (bx - ax);
            clipped.push((xm, ay + t * (by - ay)));
        }
    }
    let m = clipped.len();
    if m < 3 {
        return 0.0;
    }
    let mut s = 0.0;
    for i in 0..m {
        let (x1, y1) = clipped[i];
        let (x2, y2) = clipped[(i + 1) % m];
        s += (y1 + y2) * (x1 - x2);
    }
    (s / 2.0).abs()
}

/// 격자 두 점의 거리². §3.3 "대표점 간 점-점 거리의 제곱" (64비트 곱).
#[inline]
pub fn dist2(a: Pt, b: Pt) -> i128 {
    let d = a - b;
    dot(d, d)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sq(x0: i64, y0: i64, s: i64) -> Vec<Pt> {
        vec![Pt::new(x0, y0), Pt::new(x0 + s, y0), Pt::new(x0 + s, y0 + s), Pt::new(x0, y0 + s)]
    }

    #[test]
    fn locate_separates_inside_boundary_outside() {
        let q = sq(0, 0, 10);
        assert_eq!(locate(Pt::new(5, 5), &q), Loc::Inside);
        assert_eq!(locate(Pt::new(10, 5), &q), Loc::Boundary);
        assert_eq!(locate(Pt::new(0, 0), &q), Loc::Boundary);
        assert_eq!(locate(Pt::new(11, 5), &q), Loc::Outside);
        assert_eq!(locate(Pt::new(5, -1), &q), Loc::Outside);
    }

    #[test]
    fn cleanup_drops_collinear_and_spikes_and_orients_ccw() {
        let cw = vec![Pt::new(0, 0), Pt::new(0, 10), Pt::new(0, 20), Pt::new(10, 20), Pt::new(10, 0)];
        let c = cleanup(&cw);
        assert_eq!(c.len(), 4);
        assert!(area2(&c) > 0);
    }

    #[test]
    fn area_left_of_clips_at_the_strip_edge() {
        let q = sq(0, 0, 10);
        assert_eq!(area_left_of(&q, 10), 100.0);
        assert_eq!(area_left_of(&q, 4), 40.0);
        assert_eq!(area_left_of(&q, -1), 0.0);
    }
}
