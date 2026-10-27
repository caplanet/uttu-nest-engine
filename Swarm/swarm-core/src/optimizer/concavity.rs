//! LBF 초기 배치 순서용 **방향성 오목도(directional concavity)** 분석.
//!
//! `LBF수정사항.md` 의 설계를 구현한다. 요지는 이렇다 —
//!
//! > 단순히 `볼록껍질 면적 − 다각형 면적` 을 쓰지 않는다.
//! > Polygon → Reflex vertex 검출 → (Hull − Polygon) 으로 오목 포켓 추출 →
//! > 포켓마다 **열린 방향** 계산 → Left 10 / Bottom 1 가중 점수.
//!
//! "오목한 도형을 왼쪽에 놓는다" 가 아니라 **"왼쪽에 놓았을 때 다른 조각이 들어갈
//! 가능성이 높은 오목부를 가진 도형을 왼쪽에 먼저 놓는다"** 이다. 그래서 오목부가
//! 어느 쪽으로 열려 있는지가 점수의 핵심이다.
//!
//! # 최종 점수 (설계 §22)
//!
//! ```text
//! Score = 10·S_L + S_B + 2·S_C + 0.5·S_D
//!
//! S_L = Σ (A_i / A_P) · L_i        왼쪽으로 열린 오목도
//! S_B = Σ (A_i / A_P) · B_i        아래로 열린 오목도
//! S_C = (A_hull − A_P) / A_hull    전체 오목도
//! S_D = Σ (A_i / A_P) · depth_i / bbox_diag   오목부 깊이
//! ```
//!
//! `A_i` 는 포켓 면적, `A_P` 는 다각형 면적, `L_i = max(−dir_x, 0)`,
//! `B_i = max(−dir_y, 0)` 이다. 면적을 `A_P` 로 나누는 이유는 크기 차이를 지우기
//! 위해서다(§15) — 안 나누면 큰 조각이 무조건 이긴다.
//!
//! # 설계와 다르게 한 곳 — 포켓 분해
//!
//! 설계는 `geo` 크레이트의 불리언 차집합(`hull.difference(&polygon)`)으로 포켓을
//! 얻고, 각 포켓을 **가장 가까운** reflex vertex 에 붙인다. 그러고 나서 §20 에서
//! 스스로 지적한다 — 포켓 하나가 여러 reflex vertex 에 걸치면 그 연결이 정확하지
//! 않다고.
//!
//! 여기서는 불리언 연산 없이 **정확하게** 푼다. 단순 다각형에서 볼록껍질 정점은
//! 다각형 정점의 부분집합이고, 껍질의 연속한 두 정점 `a → b` 사이를 다각형 경계가
//! 여러 정점을 거쳐 지나가면 그 사슬과 껍질 간선 `b → a` 가 닫는 영역이 **포켓
//! 하나**다. 이 분해는
//!
//! * `Hull − Polygon` 과 정확히 같은 집합이고(면적 합이 `A_hull − A_P` 와 일치 —
//!   테스트로 못 박았다),
//! * 포켓의 **열린 방향**이 껍질 간선의 바깥 법선으로 자연스럽게 정의되며,
//! * 사슬 안의 reflex vertex 가 곧 그 포켓의 reflex vertex 라 §20 의 문제가 없고,
//! * 깊이(사슬 정점에서 껍질 간선까지의 최대 거리)가 공짜로 나온다.
//!
//! 외부 크레이트를 끌어오지 않는 이유도 있다 — `geo` 의 불리언 연산은 부동소수
//! 견고성 문제가 있고, 이 코드베이스는 병렬 결정론을 비트 단위로 검증한다.
//!
//! # 어느 도형으로 계산하는가 — `shape_orig`
//!
//! **충돌용 도형(`shape_cd`)으로 계산하면 안 된다.** importer 의
//! `narrow_concavity_cutoff` 가 좁은 오목부를 **메워 버리므로**, 여기서 재려는
//! 포켓이 정확히 사라진다. 원본(`shape_orig.shape`)을 쓴다. jagua 는 원본을
//! CCW 로 정규화하지만 여기서도 한 번 더 보장한다(§2).
//!
//! # 회전
//!
//! 방향 점수는 자세에 따라 달라진다. 이 분석은 **입력 자세(회전 0)** 기준이다.
//! 기준 마커(wsjk156)는 회전 `[0]` 고정이라 문제가 없고, 회전이 허용되는
//! 인스턴스에서는 "회전 0 으로 놓았을 때의 오목도" 로 읽어야 한다.

use jagua_rs::geometry::primitives::SPolygon;

/// 왼쪽으로 열린 오목도의 가중치. 설계 §22.
pub const W_LEFT: f64 = 10.0;
/// 아래로 열린 오목도의 가중치.
pub const W_BOTTOM: f64 = 1.0;
/// 전체 오목도(`(hull − poly) / hull`)의 가중치.
pub const W_TOTAL: f64 = 2.0;
/// 깊이 점수의 가중치.
pub const W_DEPTH: f64 = 0.5;

/// 공선·영길이 판정 허용치. 좌표는 mm 이고 f64 로 계산하므로 넉넉하다.
const EPS: f64 = 1e-9;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pt {
    pub x: f64,
    pub y: f64,
}

/// 내부각이 180° 를 넘는 정점.
#[derive(Debug, Clone, PartialEq)]
pub struct ReflexVertex {
    /// (중복 제거 후) 정점 번호.
    pub index: usize,
    pub point: Pt,
    /// 내부각 (radian, `π < θ < 2π`).
    pub reflex_angle: f64,
    /// 두 인접 변 단위벡터의 합을 정규화한 것 (설계 §4).
    pub dir_x: f64,
    pub dir_y: f64,
}

/// 볼록껍질 간선 하나가 덮는 오목 영역.
#[derive(Debug, Clone, PartialEq)]
pub struct ConcavityPocket {
    pub area: f64,
    pub centroid: Pt,
    /// 이 포켓을 닫는 껍질 간선의 양 끝 정점 번호 `(a, b)`. 다각형 경계는
    /// `a → a+1 → … → b` 로 포켓 안쪽을 돈다.
    pub hull_edge: (usize, usize),
    /// 포켓이 **열린 방향** — 껍질 간선의 바깥 법선(단위벡터).
    pub dir_x: f64,
    pub dir_y: f64,
    /// `max(−dir_x, 0)`. 1 이면 정확히 왼쪽으로 열렸다.
    pub left_factor: f64,
    /// `max(−dir_y, 0)`.
    pub bottom_factor: f64,
    /// 사슬 정점에서 껍질 간선까지의 최대 수직 거리. 좁고 깊은 오목부를
    /// 얕고 넓은 것과 구분한다(설계 §23).
    pub depth: f64,
    /// 이 포켓 사슬 안의 reflex vertex 들 (`reflex_vertices` 의 번호).
    pub reflex_indices: Vec<usize>,
    /// 그중 가장 큰 내부각. 설계 §17 의 각도 인자는 여기서 계산할 수 있다 —
    /// 최종 점수(§22)에는 들어가지 않으므로 값만 남긴다.
    pub max_reflex_angle: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ConcavityScore {
    pub polygon_area: f64,
    pub hull_area: f64,
    /// 포켓 면적 합 = `hull_area − polygon_area`.
    pub total_area: f64,
    /// `S_C` — `total_area / hull_area`.
    pub normalized_area: f64,
    /// `S_L`.
    pub left_score: f64,
    /// `S_B`.
    pub bottom_score: f64,
    /// `S_D`.
    pub depth_score: f64,
    /// `10·S_L + S_B + 2·S_C + 0.5·S_D`.
    pub final_score: f64,
    pub reflex_vertices: Vec<ReflexVertex>,
    pub pockets: Vec<ConcavityPocket>,
}

impl ConcavityScore {
    fn zero() -> Self {
        Self {
            polygon_area: 0.0,
            hull_area: 0.0,
            total_area: 0.0,
            normalized_area: 0.0,
            left_score: 0.0,
            bottom_score: 0.0,
            depth_score: 0.0,
            final_score: 0.0,
            reflex_vertices: vec![],
            pockets: vec![],
        }
    }

    /// 설계 §17 의 각도 인자 — `(θ − π) / π`, 0~1. 최종 점수에는 안 쓴다.
    pub fn reflex_angle_factor(angle: f64) -> f64 {
        ((angle - std::f64::consts::PI) / std::f64::consts::PI).clamp(0.0, 1.0)
    }
}

/// jagua 다각형에서 바로 분석한다. **`shape_orig.shape` 를 넘겨라** — 모듈 문서 참조.
pub fn analyze_spolygon(shape: &SPolygon) -> ConcavityScore {
    let pts: Vec<Pt> = shape
        .vertices()
        .iter()
        .map(|p| Pt {
            x: p.0 as f64,
            y: p.1 as f64,
        })
        .collect();
    analyze_concavity(&pts)
}

/// 다각형 하나의 방향성 오목도.
pub fn analyze_concavity(input: &[Pt]) -> ConcavityScore {
    let points = ensure_ccw(dedup_consecutive(input));
    let n = points.len();
    if n < 3 {
        return ConcavityScore::zero();
    }
    let polygon_area = signed_area(&points);
    if polygon_area <= EPS {
        return ConcavityScore::zero();
    }

    // 1. reflex vertex
    let reflex_vertices = find_reflex_vertices(&points);

    // 2. 볼록껍질 (CCW, 다각형 정점 번호)
    let hull = convex_hull_indices(&points);
    if hull.len() < 3 {
        return ConcavityScore::zero();
    }
    let hull_pts: Vec<Pt> = hull.iter().map(|&i| points[i]).collect();
    let hull_area = signed_area(&hull_pts);
    if hull_area <= EPS {
        return ConcavityScore::zero();
    }

    // 3. 껍질 간선마다 포켓
    let bbox_diag = bbox_diagonal(&points).max(EPS);
    let mut pockets = Vec::new();
    for k in 0..hull.len() {
        let a = hull[k];
        let b = hull[(k + 1) % hull.len()];
        let chain_len = (b + n - a) % n; // a 에서 b 까지 변의 수
        if chain_len <= 1 {
            continue; // 껍질 간선이 곧 다각형 변 — 포켓 없음
        }
        let chain: Vec<usize> = (0..=chain_len).map(|s| (a + s) % n).collect();
        let chain_pts: Vec<Pt> = chain.iter().map(|&i| points[i]).collect();
        let area = signed_area(&chain_pts).abs();
        if area <= EPS {
            continue;
        }
        let centroid = polygon_centroid(&chain_pts).unwrap_or(points[a]);

        // 열린 방향 = 껍질 간선 a→b 의 바깥 법선. CCW 껍질에서 바깥은 진행 방향의 오른쪽.
        let (ex, ey) = (points[b].x - points[a].x, points[b].y - points[a].y);
        let elen = (ex * ex + ey * ey).sqrt();
        let (dir_x, dir_y) = if elen > EPS {
            (ey / elen, -ex / elen)
        } else {
            (0.0, 0.0)
        };

        // 깊이 = 사슬 정점에서 간선까지의 최대 수직 거리.
        let depth = chain[1..chain.len() - 1]
            .iter()
            .map(|&i| {
                let p = points[i];
                ((p.x - points[a].x) * ey - (p.y - points[a].y) * ex).abs() / elen.max(EPS)
            })
            .fold(0.0_f64, f64::max);

        let interior: Vec<usize> = chain[1..chain.len() - 1].to_vec();
        let reflex_indices: Vec<usize> = reflex_vertices
            .iter()
            .enumerate()
            .filter(|(_, r)| interior.contains(&r.index))
            .map(|(ri, _)| ri)
            .collect();
        let max_reflex_angle = reflex_indices
            .iter()
            .map(|&ri| reflex_vertices[ri].reflex_angle)
            .fold(0.0_f64, f64::max);

        pockets.push(ConcavityPocket {
            area,
            centroid,
            hull_edge: (a, b),
            dir_x,
            dir_y,
            left_factor: (-dir_x).max(0.0),
            bottom_factor: (-dir_y).max(0.0),
            depth,
            reflex_indices,
            max_reflex_angle,
        });
    }

    // 4. 점수
    let total_area: f64 = pockets.iter().map(|p| p.area).sum();
    let normalized_area = total_area / hull_area;
    let (mut left_score, mut bottom_score, mut depth_score) = (0.0, 0.0, 0.0);
    for p in &pockets {
        let ratio = p.area / polygon_area;
        left_score += ratio * p.left_factor;
        bottom_score += ratio * p.bottom_factor;
        depth_score += ratio * (p.depth / bbox_diag);
    }
    let final_score =
        W_LEFT * left_score + W_BOTTOM * bottom_score + W_TOTAL * normalized_area + W_DEPTH * depth_score;

    ConcavityScore {
        polygon_area,
        hull_area,
        total_area,
        normalized_area,
        left_score,
        bottom_score,
        depth_score,
        final_score,
        reflex_vertices,
        pockets,
    }
}

// ───────────────────────── 기하 보조 ─────────────────────────

fn dedup_consecutive(input: &[Pt]) -> Vec<Pt> {
    let mut out: Vec<Pt> = Vec::with_capacity(input.len());
    for &p in input {
        if let Some(last) = out.last() {
            if (last.x - p.x).abs() <= EPS && (last.y - p.y).abs() <= EPS {
                continue;
            }
        }
        out.push(p);
    }
    // 마지막이 처음과 같으면(닫힌 표기) 떼어 낸다.
    while out.len() > 1 {
        let (f, l) = (out[0], out[out.len() - 1]);
        if (f.x - l.x).abs() <= EPS && (f.y - l.y).abs() <= EPS {
            out.pop();
        } else {
            break;
        }
    }
    out
}

pub(crate) fn signed_area(points: &[Pt]) -> f64 {
    let n = points.len();
    let mut area = 0.0;
    for i in 0..n {
        let a = points[i];
        let b = points[(i + 1) % n];
        area += a.x * b.y - b.x * a.y;
    }
    area * 0.5
}

fn ensure_ccw(mut points: Vec<Pt>) -> Vec<Pt> {
    if signed_area(&points) < 0.0 {
        points.reverse();
    }
    points
}

fn cross(a: Pt, b: Pt, c: Pt) -> f64 {
    (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x)
}

fn find_reflex_vertices(points: &[Pt]) -> Vec<ReflexVertex> {
    let n = points.len();
    let mut out = Vec::new();
    for i in 0..n {
        let prev = points[(i + n - 1) % n];
        let curr = points[i];
        let next = points[(i + 1) % n];
        let cr = cross(prev, curr, next);
        if cr >= -EPS {
            continue; // convex 또는 공선
        }
        let (v1x, v1y) = (prev.x - curr.x, prev.y - curr.y);
        let (v2x, v2y) = (next.x - curr.x, next.y - curr.y);
        let (l1, l2) = ((v1x * v1x + v1y * v1y).sqrt(), (v2x * v2x + v2y * v2y).sqrt());
        if l1 < EPS || l2 < EPS {
            continue;
        }
        let dot = ((v1x * v2x + v1y * v2y) / (l1 * l2)).clamp(-1.0, 1.0);
        let convex_angle = dot.acos();
        let reflex_angle = 2.0 * std::f64::consts::PI - convex_angle;
        let (ux, uy) = (v1x / l1 + v2x / l2, v1y / l1 + v2y / l2);
        let ul = (ux * ux + uy * uy).sqrt();
        let (dir_x, dir_y) = if ul < 1e-12 { (0.0, 0.0) } else { (ux / ul, uy / ul) };
        out.push(ReflexVertex {
            index: i,
            point: curr,
            reflex_angle,
            dir_x,
            dir_y,
        });
    }
    out
}

/// Andrew 단조 사슬. **정점 번호**를 CCW 순서로 돌려준다.
///
/// jagua 의 `convex_hull_indices` 는 좌표 일치로 번호를 되찾는데 중복 정점이 있으면
/// 어긋날 수 있다. 여기서는 번호를 직접 들고 간다.
fn convex_hull_indices(points: &[Pt]) -> Vec<usize> {
    let n = points.len();
    if n < 3 {
        return (0..n).collect();
    }
    let mut idx: Vec<usize> = (0..n).collect();
    idx.sort_by(|&a, &b| {
        points[a]
            .x
            .total_cmp(&points[b].x)
            .then(points[a].y.total_cmp(&points[b].y))
            .then(a.cmp(&b))
    });
    let turn = |o: usize, a: usize, b: usize| -> f64 {
        (points[a].x - points[o].x) * (points[b].y - points[o].y)
            - (points[a].y - points[o].y) * (points[b].x - points[o].x)
    };
    let mut lower: Vec<usize> = Vec::new();
    for &p in &idx {
        while lower.len() >= 2 && turn(lower[lower.len() - 2], lower[lower.len() - 1], p) <= EPS {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper: Vec<usize> = Vec::new();
    for &p in idx.iter().rev() {
        while upper.len() >= 2 && turn(upper[upper.len() - 2], upper[upper.len() - 1], p) <= EPS {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

fn polygon_centroid(points: &[Pt]) -> Option<Pt> {
    let a = signed_area(points);
    if a.abs() <= EPS {
        return None;
    }
    let n = points.len();
    let (mut cx, mut cy) = (0.0, 0.0);
    for i in 0..n {
        let p = points[i];
        let q = points[(i + 1) % n];
        let w = p.x * q.y - q.x * p.y;
        cx += (p.x + q.x) * w;
        cy += (p.y + q.y) * w;
    }
    Some(Pt {
        x: cx / (6.0 * a),
        y: cy / (6.0 * a),
    })
}

fn bbox_diagonal(points: &[Pt]) -> f64 {
    let (mut x0, mut y0, mut x1, mut y1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    for p in points {
        x0 = x0.min(p.x);
        y0 = y0.min(p.y);
        x1 = x1.max(p.x);
        y1 = y1.max(p.y);
    }
    ((x1 - x0).powi(2) + (y1 - y0).powi(2)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pts(v: &[(f64, f64)]) -> Vec<Pt> {
        v.iter().map(|&(x, y)| Pt { x, y }).collect()
    }

    /// 10x10 정사각형에서 **왼쪽 변**에 5x4 홈을 판 "ㄷ" 자. 홈은 −x 로 열린다.
    fn notch_left() -> Vec<Pt> {
        pts(&[
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 7.0),
            (5.0, 7.0),
            (5.0, 3.0),
            (0.0, 3.0),
        ])
    }

    fn mirror_x(v: &[Pt]) -> Vec<Pt> {
        v.iter().map(|p| Pt { x: 10.0 - p.x, y: p.y }).collect()
    }

    fn swap_xy(v: &[Pt]) -> Vec<Pt> {
        v.iter().map(|p| Pt { x: p.y, y: p.x }).collect()
    }

    #[test]
    fn convex_shapes_have_no_concavity() {
        for shape in [
            pts(&[(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0)]),
            pts(&[(0.0, 0.0), (5.0, 0.0), (2.0, 4.0)]),
        ] {
            let s = analyze_concavity(&shape);
            assert!(s.pockets.is_empty(), "볼록 도형에 포켓이 있다: {:?}", s.pockets);
            assert!(s.reflex_vertices.is_empty());
            assert_eq!(s.final_score, 0.0);
            assert!((s.hull_area - s.polygon_area).abs() < 1e-9);
        }
    }

    /// 설계 §12·§16 의 숫자를 손으로 계산한 값과 맞춘다.
    #[test]
    fn a_left_facing_notch_scores_exactly_as_the_design_says() {
        let s = analyze_concavity(&notch_left());
        assert_eq!(s.pockets.len(), 1, "홈 하나 = 포켓 하나");
        let p = &s.pockets[0];
        assert!((s.polygon_area - 80.0).abs() < 1e-9);
        assert!((s.hull_area - 100.0).abs() < 1e-9);
        assert!((p.area - 20.0).abs() < 1e-9, "포켓 면적 {}", p.area);
        // 열린 방향은 정확히 −x.
        assert!((p.dir_x + 1.0).abs() < 1e-9 && p.dir_y.abs() < 1e-9, "dir {:?}", (p.dir_x, p.dir_y));
        assert!((p.left_factor - 1.0).abs() < 1e-9);
        assert_eq!(p.bottom_factor, 0.0);
        assert!((p.depth - 5.0).abs() < 1e-9, "깊이 {}", p.depth);
        // S_L = 20/80, S_B = 0, S_C = 20/100, S_D = 0.25 · 5/√200
        assert!((s.left_score - 0.25).abs() < 1e-9);
        assert_eq!(s.bottom_score, 0.0);
        assert!((s.normalized_area - 0.2).abs() < 1e-9);
        let expected_depth = 0.25 * 5.0 / 200f64.sqrt();
        assert!((s.depth_score - expected_depth).abs() < 1e-9);
        let expected = 10.0 * 0.25 + 0.0 + 2.0 * 0.2 + 0.5 * expected_depth;
        assert!((s.final_score - expected).abs() < 1e-9, "{} vs {expected}", s.final_score);
        // reflex vertex 는 홈 안쪽 두 모서리이고 내부각은 270°.
        assert_eq!(s.reflex_vertices.len(), 2);
        for r in &s.reflex_vertices {
            assert!((r.reflex_angle - 1.5 * std::f64::consts::PI).abs() < 1e-6);
        }
        assert_eq!(p.reflex_indices.len(), 2);
    }

    /// 같은 홈이라도 **어느 쪽으로 열렸는가**가 점수를 가른다. 이것이 이 모듈의 존재 이유다.
    #[test]
    fn opening_direction_orders_left_over_bottom_over_right() {
        let left = analyze_concavity(&notch_left()).final_score;
        let right = analyze_concavity(&mirror_x(&notch_left())).final_score;
        // x↔y 를 바꾸면 −x 로 열린 홈이 −y 로 열린다.
        let bottom = analyze_concavity(&swap_xy(&notch_left())).final_score;
        assert!(left > bottom, "왼쪽 {left} 이 아래 {bottom} 보다 커야 한다");
        assert!(bottom > right, "아래 {bottom} 가 오른쪽 {right} 보다 커야 한다");
        // 오른쪽으로 열린 홈은 방향 점수가 0 — 남는 건 전체 오목도와 깊이뿐.
        let r = analyze_concavity(&mirror_x(&notch_left()));
        assert_eq!(r.left_score, 0.0);
        assert_eq!(r.bottom_score, 0.0);
        assert!(r.normalized_area > 0.0);
    }

    /// 포켓 면적 합은 정확히 `hull − polygon` 이어야 한다(설계 §8). 이 항등식이
    /// 깨지면 포켓 분해가 영역을 빠뜨리거나 겹쳐 세고 있는 것이다.
    #[test]
    fn pocket_areas_sum_to_hull_minus_polygon() {
        // 홈 두 개 (왼쪽·아래) + 얕은 빗면 하나.
        let shape = pts(&[
            (0.0, 0.0),
            (3.0, 0.0),
            (3.0, 2.0),
            (6.0, 2.0),
            (6.0, 0.0),
            (10.0, 0.0),
            (10.0, 6.0),
            (8.0, 8.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 7.0),
            (4.0, 7.0),
            (4.0, 4.0),
            (0.0, 4.0),
        ]);
        let s = analyze_concavity(&shape);
        assert_eq!(s.pockets.len(), 3, "{:?}", s.pockets.iter().map(|p| p.hull_edge).collect::<Vec<_>>());
        let sum: f64 = s.pockets.iter().map(|p| p.area).sum();
        assert!(
            (sum - (s.hull_area - s.polygon_area)).abs() < 1e-9,
            "포켓 합 {sum} != hull−poly {}",
            s.hull_area - s.polygon_area
        );
        assert!((s.total_area - sum).abs() < 1e-12);
        for p in &s.pockets {
            assert!(p.depth / 200f64.sqrt() <= 1.0 + 1e-12, "깊이 정규화가 1 을 넘는다");
            assert!(p.left_factor >= 0.0 && p.left_factor <= 1.0);
            assert!(p.bottom_factor >= 0.0 && p.bottom_factor <= 1.0);
        }
    }

    /// 감기 방향과 평행이동에 무관해야 한다. 그렇지 않으면 같은 도형이 입력 표기에 따라
    /// 다른 순서를 받는다.
    #[test]
    fn score_is_invariant_to_winding_and_translation() {
        let base = analyze_concavity(&notch_left());
        let mut reversed = notch_left();
        reversed.reverse();
        let rev = analyze_concavity(&reversed);
        assert!((base.final_score - rev.final_score).abs() < 1e-9);
        assert!((base.left_score - rev.left_score).abs() < 1e-9);

        let moved: Vec<Pt> = notch_left().iter().map(|p| Pt { x: p.x + 123.0, y: p.y - 45.0 }).collect();
        let mv = analyze_concavity(&moved);
        assert!((base.final_score - mv.final_score).abs() < 1e-9);
    }

    /// 퇴화 입력에서 패닉하지 않는다 — 초기 배치는 모든 조각을 거치므로 하나만 죽어도
    /// 전체가 죽는다.
    #[test]
    fn degenerate_inputs_do_not_panic() {
        assert_eq!(analyze_concavity(&[]).final_score, 0.0);
        assert_eq!(analyze_concavity(&pts(&[(0.0, 0.0)])).final_score, 0.0);
        assert_eq!(analyze_concavity(&pts(&[(0.0, 0.0), (1.0, 1.0)])).final_score, 0.0);
        // 공선 세 점 — 면적 0.
        assert_eq!(analyze_concavity(&pts(&[(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)])).final_score, 0.0);
        // 연속 중복 정점과 닫힌 표기(마지막 = 처음)는 같은 결과를 내야 한다.
        let mut dup = notch_left();
        dup.insert(2, dup[2]);
        dup.push(dup[0]);
        let a = analyze_concavity(&notch_left());
        let b = analyze_concavity(&dup);
        assert!((a.final_score - b.final_score).abs() < 1e-9);
        assert_eq!(a.pockets.len(), b.pockets.len());
    }

    #[test]
    fn angle_factor_matches_the_design_table() {
        use std::f64::consts::PI;
        let f = ConcavityScore::reflex_angle_factor;
        assert!((f(PI) - 0.0).abs() < 1e-12);
        assert!((f(1.25 * PI) - 0.25).abs() < 1e-12);
        assert!((f(1.5 * PI) - 0.5).abs() < 1e-12);
        assert!((f(2.0 * PI) - 1.0).abs() < 1e-12);
        assert_eq!(f(0.5 * PI), 0.0, "180° 미만은 0 으로 잘린다");
    }
}
