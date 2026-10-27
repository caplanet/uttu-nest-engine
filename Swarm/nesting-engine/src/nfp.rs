//! NFP 생성과 캐시 (§2.2).
//!
//! # 방식 — 환형 정점 순회 + 누적 병합
//!
//! 지시서는 볼록 분해와 합집합 t-구간 피복을 금지하고, "정점 배열을 환형으로 돌며
//! 리스트에 누적 병합하고, 각 단계가 성공하면 루프 처음으로 되돌아간다" 고 적는다.
//! 여기서는 그것을 **합성곱(convolution) 사이클**로 구현한다.
//!
//! 1. A 의 각 꼭짓점을 환형(`idx mod N`, §12.1.21 AG2)으로 돌며, 그 꼭짓점이 쓸고 가는
//!    방향 원뿔 안에 드는 B 의 모서리를 선분으로 누적한다. B 의 꼭짓점과 A 의 모서리도
//!    똑같이 한다. 오목(시계방향) 꼭짓점에서 쓸린 선분은 역방향으로 넣는다.
//! 2. 선분 끝점은 정확히 꼭짓점 합이라 격자 위에서 오차 없이 맞물린다. 끝점 등가
//!    판정(허용오차는 인자)으로 선분을 이어 닫힌 사이클로 꿰맨다. 한 끝점에서 선분이
//!    여러 개 뻗으면 다음 것으로 되돌아가 계속한다 — Hierholzer 알고리즘이고, 지시서의
//!    "성공 시 루프 처음으로 복귀, 한 정점에서 여러 에지" 에 해당한다.
//! 3. 두 단순 다각형의 민코프스키 합은 합성곱 사이클에 대한 **감김수가 양수인 점들의
//!    집합**이다(Guibas–Ramshaw–Stolfi). 사이클들을 `clipper2` 에 `FillRule::Positive`
//!    로 합쳐 그 영역의 경계를 얻는다.
//!
//! 평행 모서리 퇴화는 "같은 각도면 A 의 방향이 B 보다 먼저" 라는 기호 섭동으로 방향에
//! 전순서를 준다. 그러면 모든 꼭짓점의 쓸기가 반열린 구간으로 정확히 나뉘어, 사이클
//! 그래프가 언제나 균형(들어오는 수 = 나가는 수)을 이룬다.
//!
//! 볼록 분해를 하지 않으므로 오목 형상의 오목부가 그대로 NFP 에 남는다.

use crate::geom::{BBox, Loc, Pt, area2, cross, dot, locate};
use clipper2::{FillRule, One, Paths, union};
use std::collections::HashMap;

/// NFP 영역. 바깥 경계는 넓이가 양수(반시계), 구멍은 음수인 고리다.
#[derive(Clone, Debug)]
pub struct Nfp {
    pub loops: Vec<Vec<Pt>>,
    pub bbox: BBox,
}

impl Nfp {
    /// 기준점 위치 `p` 가 NFP 의 안(겹침)·경계(접촉)·밖(떨어짐) 중 어디인가.
    pub fn locate(&self, p: Pt) -> Loc {
        if !self.bbox.contains(p) {
            return Loc::Outside;
        }
        let mut winding = 0i32;
        for lp in &self.loops {
            match locate(p, lp) {
                Loc::Boundary => return Loc::Boundary,
                Loc::Inside => winding += if area2(lp) > 0 { 1 } else { -1 },
                Loc::Outside => {}
            }
        }
        if winding > 0 { Loc::Inside } else { Loc::Outside }
    }

    /// 바깥 경계 고리만. §3.3: 구멍(오목부 안쪽 포켓) 후보는 만들지 않는다.
    pub fn outer_loops(&self) -> impl Iterator<Item = &Vec<Pt>> {
        self.loops.iter().filter(|lp| area2(lp) > 0)
    }

    pub fn vertex_count(&self) -> usize {
        self.loops.iter().map(|l| l.len()).sum()
    }
}

#[inline]
fn same_dir(a: Pt, b: Pt) -> bool {
    cross(a, b) == 0 && dot(a, b) > 0
}

/// `w` 가 반시계 호 `start → end`(호 < π)의 **열린** 구간 안인가.
#[inline]
fn in_open_arc(start: Pt, end: Pt, w: Pt) -> bool {
    cross(start, w) > 0 && cross(w, end) > 0
}

/// 다각형 `x` 의 꼭짓점들과 다각형 `y` 의 모서리들로 만든 합성곱 선분을 누적한다.
/// `x_first` 는 기호 섭동에서 `x` 의 방향이 먼저인지(= A 인지)다.
fn accumulate(x: &[Pt], y: &[Pt], x_first: bool, out: &mut Vec<(Pt, Pt)>) {
    let n = x.len();
    let m = y.len();
    for i in 0..n {
        let u = x[i] - x[(i + n - 1) % n];
        let v = x[(i + 1) % n] - x[i];
        let c = cross(u, v);
        if c == 0 {
            continue; // 일직선 — 쓸리는 방향이 없다(정리 단계에서 이미 지웠다).
        }
        // 반시계 꺾임은 (u, v] 를 +로, 시계 꺾임은 (v, u] 를 −로 쓴다.
        let (start, end, positive) = if c > 0 { (u, v, true) } else { (v, u, false) };
        for j in 0..m {
            let w = y[(j + 1) % m] - y[j];
            let tie = if x_first { same_dir(w, start) } else { same_dir(w, end) };
            if !(in_open_arc(start, end, w) || tie) {
                continue;
            }
            let p0 = x[i] + y[j];
            let p1 = x[i] + y[(j + 1) % m];
            out.push(if positive { (p0, p1) } else { (p1, p0) });
        }
    }
}

/// 두 반시계 단순 다각형의 합성곱 선분 (A 꼭짓점×B 모서리, B 꼭짓점×A 모서리).
pub fn convolution(a: &[Pt], b: &[Pt]) -> Vec<(Pt, Pt)> {
    let mut segs = Vec::new();
    accumulate(a, b, true, &mut segs);
    accumulate(b, a, false, &mut segs);
    segs
}

/// 끝점 등가 판정 (§2.2 ②). 허용오차는 인자로 받는다 — 상수 하드코딩 금지.
/// 합성곱 끝점은 꼭짓점 합이라 격자에서 정확히 같으므로 기본값은 0 이다.
fn node_key(p: Pt, tol: i64) -> Pt {
    if tol <= 0 {
        p
    } else {
        let s = tol + 1;
        Pt::new(p.x.div_euclid(s), p.y.div_euclid(s))
    }
}

/// 방향 선분 묶음을 닫힌 사이클들로 꿰맨다 (Hierholzer).
pub fn stitch(segs: &[(Pt, Pt)], tol: i64) -> Result<Vec<Vec<Pt>>, String> {
    let mut out_adj: HashMap<Pt, Vec<usize>> = HashMap::new();
    let mut balance: HashMap<Pt, i64> = HashMap::new();
    for (k, &(s, e)) in segs.iter().enumerate() {
        out_adj.entry(node_key(s, tol)).or_default().push(k);
        *balance.entry(node_key(s, tol)).or_default() += 1;
        *balance.entry(node_key(e, tol)).or_default() -= 1;
    }
    if let Some((p, b)) = balance.iter().find(|(_, b)| **b != 0) {
        return Err(format!("합성곱 그래프가 균형이 아니다: 끝점 {p:?} 차이 {b}"));
    }
    let mut used = vec![false; segs.len()];
    let mut cursor: HashMap<Pt, usize> = HashMap::new();
    let mut cycles = Vec::new();
    for start in 0..segs.len() {
        if used[start] {
            continue;
        }
        // 한 사이클을 따라가며 막히면 스택으로 되돌아가 곁가지를 이어 붙인다.
        let mut stack = vec![start];
        used[start] = true;
        let mut circuit: Vec<usize> = Vec::new();
        while let Some(&top) = stack.last() {
            let node = node_key(segs[top].1, tol);
            let list = out_adj.get(&node).map(|v| v.as_slice()).unwrap_or(&[]);
            let c = cursor.entry(node).or_insert(0);
            while *c < list.len() && used[list[*c]] {
                *c += 1;
            }
            if *c < list.len() {
                let nxt = list[*c];
                used[nxt] = true;
                stack.push(nxt);
            } else {
                circuit.push(stack.pop().unwrap());
            }
        }
        circuit.reverse();
        let path: Vec<Pt> = circuit.iter().map(|&k| segs[k].0).collect();
        if path.len() >= 3 {
            cycles.push(path);
        }
    }
    Ok(cycles)
}

/// A 에 대한 P 의 NFP = A ⊕ (−P). 두 입력은 정리된 반시계 다각형이어야 한다.
///
/// 결과 좌표계: A 의 기준점이 원점일 때 P 의 기준점이 놓일 수 있는 위치. 실제 배치에서는
/// A 의 위치를 더해 쓴다.
pub fn nfp(a: &[Pt], p: &[Pt], tol: i64) -> Result<Nfp, String> {
    let neg_p: Vec<Pt> = p.iter().map(|&q| -q).collect();
    let segs = convolution(a, &neg_p);
    let cycles = stitch(&segs, tol)?;
    let subject: Vec<Vec<(f64, f64)>> = cycles
        .iter()
        .map(|c| c.iter().map(|q| (q.x as f64, q.y as f64)).collect())
        .collect();
    let empty: Vec<Vec<(f64, f64)>> = Vec::new();
    let merged: Paths<One> =
        union::<One>(subject, empty, FillRule::Positive).map_err(|e| format!("NFP 합집합 실패: {e:?}"))?;
    let mut loops = Vec::new();
    for path in merged.iter() {
        let lp: Vec<Pt> = path.iter().map(|q| Pt::new(q.x().round() as i64, q.y().round() as i64)).collect();
        if lp.len() >= 3 && area2(&lp) != 0 {
            loops.push(lp);
        }
    }
    if loops.is_empty() {
        return Err("NFP 가 비었다".into());
    }
    let all: Vec<Pt> = loops.iter().flatten().copied().collect();
    Ok(Nfp { bbox: BBox::of(&all), loops })
}

/// (고정 형상 키, 이동 형상 키) → NFP. §2.2 "조회는 이진탐색".
///
/// 복제는 값이 `Arc` 라 싸다 — Complete 워커(§4)가 Draft 의 캐시를 이어받는다.
#[derive(Default, Clone)]
pub struct NfpCache {
    keys: Vec<(usize, usize)>,
    values: Vec<std::sync::Arc<Nfp>>,
    pub computed: usize,
    pub hits: usize,
}

impl NfpCache {
    /// 조회만 한다 — 캐시에 있으면 히트로 세고 돌려준다. 계산도 실패 집합 조회도 없는
    /// 싼 경로라 `Draft::nfp_of` 가 시계를 읽기 전에 먼저 부른다.
    pub fn get(&mut self, key: (usize, usize)) -> Option<std::sync::Arc<Nfp>> {
        let i = self.keys.binary_search(&key).ok()?;
        self.hits += 1;
        Some(self.values[i].clone())
    }

    pub fn get_or_compute(
        &mut self,
        key: (usize, usize),
        make: impl FnOnce() -> Result<Nfp, String>,
    ) -> Result<std::sync::Arc<Nfp>, String> {
        match self.keys.binary_search(&key) {
            Ok(i) => {
                self.hits += 1;
                Ok(self.values[i].clone())
            }
            Err(i) => {
                let v = std::sync::Arc::new(make()?);
                self.keys.insert(i, key);
                self.values.insert(i, v.clone());
                self.computed += 1;
                Ok(v)
            }
        }
    }

    pub fn len(&self) -> usize {
        self.keys.len()
    }

    pub fn is_empty(&self) -> bool {
        self.keys.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::{cleanup, translate};

    /// 두 다각형의 **내부**가 겹치는가 — `clipper2` 교집합 넓이로 판정한다.
    ///
    /// "교차 선분 또는 꼭짓점 내부" 방식은 모서리가 정확히 겹치는 퇴화 배치(축 정렬
    /// 정수 형상에서 흔하다)에서 겹침을 놓친다. 교집합은 NFP 구성(합성곱 + 합집합)과
    /// 다른 연산이라 독립 오라클로 쓸 수 있다.
    ///
    /// 배율은 1000(`Milli`)이다. 1.0 으로 잡으면 넓이 0.25칸짜리 가는 교집합의 꼭짓점이
    /// 정수로 반올림돼 넓이가 0 으로 무너진다 — 실제로 그렇게 오판했다(shapely 로 확인).
    fn interiors_overlap(a: &[Pt], b: &[Pt]) -> bool {
        use clipper2::{Milli, intersect};
        let pa: Paths<Milli> = vec![a.iter().map(|q| (q.x as f64, q.y as f64)).collect::<Vec<_>>()].into();
        let pb: Paths<Milli> = vec![b.iter().map(|q| (q.x as f64, q.y as f64)).collect::<Vec<_>>()].into();
        let common = intersect(pa, pb, FillRule::NonZero).expect("intersect");
        common.signed_area().abs() > 1e-9
    }

    /// 닫힌 꺾은선의 부호 있는 감김수 (clipper 합집합 전의 원시 합성곱 진단용).
    fn raw_winding(p: Pt, c: &[Pt]) -> i64 {
        let mut w = 0;
        for i in 0..c.len() {
            let (a, b) = (c[i], c[(i + 1) % c.len()]);
            if a.y <= p.y {
                if b.y > p.y && crate::geom::orient(a, b, p) > 0 {
                    w += 1;
                }
            } else if b.y <= p.y && crate::geom::orient(a, b, p) < 0 {
                w -= 1;
            }
        }
        w
    }

    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }
        fn range(&mut self, lo: i64, hi: i64) -> i64 {
            lo + (self.next() % (hi - lo) as u64) as i64
        }
    }

    /// 무작위 별 모양(오목) 단순 다각형.
    fn star(rng: &mut Rng, k: usize, r0: i64, r1: i64) -> Vec<Pt> {
        let pts: Vec<Pt> = (0..k)
            .map(|i| {
                let th = std::f64::consts::TAU * i as f64 / k as f64;
                let r = rng.range(r0, r1) as f64;
                Pt::new((r * th.cos()).round() as i64, (r * th.sin()).round() as i64)
            })
            .collect();
        cleanup(&pts)
    }

    fn check_against_oracle(a: &[Pt], p: &[Pt], rng: &mut Rng, samples: usize) {
        let n = nfp(a, p, 0).expect("nfp");
        let span = n.bbox;
        let mut checked = 0;
        for _ in 0..samples * 4 {
            let t = Pt::new(rng.range(span.x0 - 50, span.x1 + 50), rng.range(span.y0 - 50, span.y1 + 50));
            let loc = n.locate(t);
            if loc == Loc::Boundary {
                continue;
            }
            let moved = translate(p, t);
            let truth = interiors_overlap(a, &moved);
            if (loc == Loc::Inside) != truth {
                let neg: Vec<Pt> = p.iter().map(|&q| -q).collect();
                let cycles = stitch(&convolution(a, &neg), 0).unwrap();
                let w: i64 = cycles.iter().map(|c| raw_winding(t, c)).sum();
                panic!(
                    "t={t:?} nfp={loc:?} oracle={truth} raw_winding={w} cycles={}
A={a:?}
P={p:?}",
                    cycles.len()
                );
            }
            checked += 1;
            if checked >= samples {
                break;
            }
        }
        assert!(checked >= samples / 2);
    }

    #[test]
    fn squares_give_the_expected_square_nfp() {
        let a = cleanup(&[Pt::new(0, 0), Pt::new(10, 0), Pt::new(10, 10), Pt::new(0, 10)]);
        let p = cleanup(&[Pt::new(0, 0), Pt::new(4, 0), Pt::new(4, 4), Pt::new(0, 4)]);
        let n = nfp(&a, &p, 0).unwrap();
        assert_eq!(n.bbox, BBox { x0: -4, y0: -4, x1: 10, y1: 10 });
        assert_eq!(n.locate(Pt::new(3, 3)), Loc::Inside);
        assert_eq!(n.locate(Pt::new(10, 3)), Loc::Boundary);
        assert_eq!(n.locate(Pt::new(11, 3)), Loc::Outside);
    }

    #[test]
    fn a_u_shape_keeps_its_pocket_as_a_hole() {
        // U 자 안쪽 폭 20, 작은 사각형 폭 10 — 오목부 안에 들어가는 자리가 NFP 구멍으로 남는다.
        let u = cleanup(&[
            Pt::new(0, 0), Pt::new(40, 0), Pt::new(40, 40), Pt::new(30, 40),
            Pt::new(30, 10), Pt::new(10, 10), Pt::new(10, 40), Pt::new(0, 40),
        ]);
        let p = cleanup(&[Pt::new(0, 0), Pt::new(10, 0), Pt::new(10, 10), Pt::new(0, 10)]);
        let n = nfp(&u, &p, 0).unwrap();
        assert_eq!(n.locate(Pt::new(15, 20)), Loc::Outside, "오목부 안은 겹치지 않는다");
        assert_eq!(n.locate(Pt::new(5, 5)), Loc::Inside);
        let mut rng = Rng(7);
        check_against_oracle(&u, &p, &mut rng, 400);
    }

    #[test]
    fn random_concave_pairs_agree_with_the_brute_force_oracle() {
        let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
        for _ in 0..40 {
            let ka = rng.range(5, 14) as usize;
            let kp = rng.range(5, 14) as usize;
            let a = star(&mut rng, ka, 30, 200);
            let p = star(&mut rng, kp, 20, 150);
            if a.len() < 3 || p.len() < 3 {
                continue;
            }
            check_against_oracle(&a, &p, &mut rng, 150);
        }
    }

    #[test]
    fn cache_uses_binary_search_and_counts_hits() {
        let a = cleanup(&[Pt::new(0, 0), Pt::new(10, 0), Pt::new(10, 10), Pt::new(0, 10)]);
        let mut c = NfpCache::default();
        for key in [(3, 1), (1, 2), (3, 1), (2, 2), (1, 2)] {
            c.get_or_compute(key, || nfp(&a, &a, 0)).unwrap();
        }
        assert_eq!((c.computed, c.hits, c.len()), (3, 2, 3));
    }
}
