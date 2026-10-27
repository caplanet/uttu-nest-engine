//! G단계 — 배치가 끝난 해를 LP로 조인다.
//!
//! 구성(construction)만으로는 상한이 막힌다. 배치 완료 해에 대해 압축을 반복한다.
//!
//! ```text
//! 변수:  피스별 Δx, Δy  (경계 = 각 피스의 이동 허용 박스) + 폭 변수 W
//! 목적:  min W
//! 제약:  ① 모든 피스 i:  x_i + Δx_i + w_i  ≤  W
//!        ② 인접 쌍 (i,j): 축별 간격 유지(G1) 또는 NFP 에지 반평면(G2)
//!        ③ 스트립 경계
//! ```
//!
//! Δ 적용 후에는 **반드시** CDE 정확 검사를 한다. 겹치면 적용 전으로 되돌린다.

use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::primitives::{Point, Rect};
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution};
use microlp::{ComparisonOp, LinearExpr, OptimizationDirection, Problem, SolveOutcome, Variable};

use super::nfp;
use jagua_rs::Instant;

/// 압축용 볼록 분해 조각 수 상한. 인접 쌍마다 조각 곱만큼 NFP를 만들기 때문에
/// 배치 단계(`nfp_max_pieces`)보다 작게 잡는다.
// ponytail: 64 (probe: 조각 평균 17.1, 면적오차 +0.02%, 껍질 폴백 0/156).
// 이보다 작게 잡으면 껍질 병합이 도형 밖을 덮어 원점이 NFP 조각 안으로 들어간다.
const COMPACT_PIECE_CAP: usize = 64;

/// 스트립 경계 여유(mm). 배치 단계의 `PLACE_EPS`와 같은 목적이다.
const BOUND_EPS: f32 = 0.05;

/// 반평면/축별 제약 우변에서 빼는 여유(mm).
/// LP 는 최적점에서 제약을 등식까지 민다 -> 조각이 정확히 맞닿아 CDE 판정이 축퇴한다.
/// 배치 단계가 만들어 놓은 간격(`PLACE_EPS`)을 그대로 지켜야 하므로 같은 값을 쓴다.
const LP_MARGIN: f32 = 0.05;

/// 한 번의 압축에서 기존 간격을 닫을 수 있는 최대 비율.
/// 여유가 큰 쌍도 완전히 붙지 않게 해서 축퇴 접촉을 막는다.
const KEEP_RATIO: f32 = 0.9;

/// 이 거리(mm)보다 여유가 크면 사실상 무제한이라 행을 만들지 않는다.
const BIG_SLACK: f32 = 1.0e4;

#[derive(Clone, Copy, Debug)]
pub struct CompactCfg {
    pub max_iters: u32,
    pub improve_tol: f32,
    pub move_tol: f32,
    pub relax_factor: f32,
    pub lp_time_limit_ms: u64,
    pub restart_count: u32,
    /// 인접 쌍 후보를 뽑는 bbox 여유 반경(mm)
    pub neighbor_slack: f32,
    /// 반복 1회에서 허용하는 최대 이동량(mm) = LP 변수 경계
    pub move_max: f32,
    /// true = G2 이상(NFP 에지 반평면), false = G1(축별 간격)
    pub edge_halfplanes: bool,
    /// true = G3 이상(반복 중 최선 해 보관)
    pub rollback: bool,
}

impl Default for CompactCfg {
    fn default() -> Self {
        CompactCfg {
            max_iters: 20,
            // 한 반복의 최대 이동량이 `move_max`(15mm) 라 100mm 는 1회 후 항상 종료였다.
            improve_tol: 0.05,
            move_tol: 30.0,
            relax_factor: 1.0,
            lp_time_limit_ms: 200,
            restart_count: 3,
            neighbor_slack: 40.0,
            move_max: 15.0,
            edge_halfplanes: true,
            rollback: true,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct CompactStats {
    pub lp_fail: usize,
    pub lp_iters_used: usize,
    pub lp_interrupted: usize,
    pub rows: usize,
    pub pairs: usize,
    /// 축별 제약을 걸 수 없어 건너뛴 쌍 (양축 모두 bbox가 겹침)
    pub skipped_pairs: usize,
    pub applied: usize,
    pub reverted: usize,
    pub width_before: f32,
    pub width_after: f32,
    /// 폭을 결정하는 조각의 item id 와 그 조각이 좌측으로 움직일 수 있는 거리
    pub blocking_item: Option<usize>,
    pub blocking_travel: f32,
    /// 여유가 음수라 0 으로 고정한 반평면 행 수
    pub neg_slack_rows: usize,
    /// 반평면으로 표현할 수 없어 상대 이동을 고정한 쌍 수
    pub frozen_pairs: usize,
}

struct Snap {
    item_id: usize,
    dt: DTransformation,
    bbox: Rect,
    pieces: Vec<Vec<Point>>,
}

fn piece_cap() -> usize {
    std::env::var("NEST_COMPACT_CAP")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(COMPACT_PIECE_CAP)
}

fn snapshot(
    prob: &SPProblem,
    need_pieces: bool,
    should_stop: &(impl Fn() -> bool + Sync),
) -> Option<Vec<Snap>> {
    let cap = piece_cap();
    let mut snaps = Vec::with_capacity(prob.layout().placed_items().len());
    for (_, pi) in prob.layout().placed_items().iter() {
        if should_stop() { return None; }
        let pieces = if need_pieces {
            // One decomposition is atomic; poll again before building rows.
            nfp::decompose_convex(&pi.shape().vertices(), cap)
                .unwrap_or_else(|| vec![nfp::hull_ccw(&pi.shape())])
        } else { Vec::new() };
        snaps.push(Snap { item_id: pi.item().idx(), dt: pi.d_transf(), bbox: pi.shape().bbox(), pieces });
    }
    if should_stop() { return None; }
    Some(snaps)
}

fn rebuild(instance: &SPInstance, width: f32, snaps: &[Snap], dxy: &[(f32, f32)],
    should_stop: &(impl Fn() -> bool + Sync),
) -> Option<SPProblem> {
    if should_stop() { return None; }
    let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
    prob.change_strip_width(width).expect(crate::dmsg!("strip width must be positive"));
    for (s, (dx, dy)) in snaps.iter().zip(dxy.iter()) {
        if should_stop() { return None; }
        let (tx, ty) = s.dt.translation();
        prob.place_item(SPPlacement {
            item_idx: s.item_id,
            d_transf: DTransformation::new(s.dt.rotation(), (tx + dx, ty + dy)),
        });
    }
    if should_stop() { return None; }
    Some(prob)
}

/// bbox 교차 + 여유 반경으로 인접 쌍을 뽑는다. 전 쌍은 행이 폭발한다.
// ponytail: bbox 교차 + 여유 반경. 부족하면 그때 확장.
fn neighbor_pairs(snaps: &[Snap], slack: f32, should_stop: &(impl Fn() -> bool + Sync)) -> Option<Vec<(usize, usize)>> {
    let n = snaps.len();
    let mut out = Vec::new();
    for i in 0..n {
        if should_stop() { return None; }
        let a = snaps[i].bbox;
        for j in (i + 1)..n {
            let b = snaps[j].bbox;
            if a.x_min - slack <= b.x_max
                && b.x_min - slack <= a.x_max
                && a.y_min - slack <= b.y_max
                && b.y_min - slack <= a.y_max
            {
                out.push((i, j));
            }
        }
    }
    Some(out)
}

/// 압축 1회. 성공하면 새 해와 통계를, LP가 실패하거나 겹치면 입력 해를 그대로 돌려준다.
fn compact_iteration(
    instance: &SPInstance,
    prob: &SPProblem,
    cfg: &CompactCfg,
    stats: &mut CompactStats,
    deadline: Option<Instant>,
    should_stop: &(impl Fn() -> bool + Sync),
) -> Option<SPProblem> {
    if should_stop() { return None; }
    let strip_h = instance.base_strip.fixed_height;
    let snaps = snapshot(prob, cfg.edge_halfplanes, should_stop)?;
    let n = snaps.len();
    if n == 0 {
        return None;
    }

    let mut lp = Problem::new(OptimizationDirection::Minimize);
    let w_var = lp.add_var(1.0, (0.0, 1.0e9));
    // Δ 에 아주 작은 좌측 견인을 준다. W 를 결정하지 않는 조각도 왼쪽으로 모인다.
    let pull = 1.0e-4;
    let dx: Vec<Variable> = (0..n)
        .map(|_| lp.add_var(pull, (-(cfg.move_max as f64), cfg.move_max as f64)))
        .collect();
    let dy: Vec<Variable> = (0..n)
        .map(|_| lp.add_var(0.0, (-(cfg.move_max as f64), cfg.move_max as f64)))
        .collect();

    let mut rows = 0usize;

    // ① 폭 정의 + ③ 스트립 경계
    for i in 0..n {
        if should_stop() { return None; }
        let b = snaps[i].bbox;
        let mut e = LinearExpr::empty();
        e.add(dx[i], 1.0);
        e.add(w_var, -1.0);
        lp.add_constraint(e, ComparisonOp::Le, -(b.x_max as f64));
        rows += 1;

        let mut e = LinearExpr::empty();
        e.add(dx[i], -1.0);
        lp.add_constraint(e, ComparisonOp::Le, (b.x_min - BOUND_EPS).max(0.0) as f64);
        rows += 1;

        let mut e = LinearExpr::empty();
        e.add(dy[i], -1.0);
        lp.add_constraint(e, ComparisonOp::Le, (b.y_min - BOUND_EPS).max(0.0) as f64);
        rows += 1;

        let mut e = LinearExpr::empty();
        e.add(dy[i], 1.0);
        lp.add_constraint(e, ComparisonOp::Le, (strip_h - BOUND_EPS - b.y_max).max(0.0) as f64);
        rows += 1;
    }

    // ② 인접 쌍
    let slack = cfg.neighbor_slack.max(2.0 * cfg.move_max + 1.0);
    let pairs = neighbor_pairs(&snaps, slack, should_stop)?;
    stats.pairs = pairs.len();
    let reach = 2.0 * cfg.move_max;
    for &(i, j) in pairs.iter() {
            if should_stop() { return None; }
        if !cfg.edge_halfplanes {
            // G1: 축별 간격 유지. 지금 분리되어 있는 축 중 여유가 가장 큰 쪽을 잡는다.
            let (a, b) = (snaps[i].bbox, snaps[j].bbox);
            let cands = [
                (b.x_min - a.x_max, 0usize), // j 가 i 의 오른쪽
                (a.x_min - b.x_max, 1usize), // i 가 j 의 오른쪽
                (b.y_min - a.y_max, 2usize), // j 가 i 의 위
                (a.y_min - b.y_max, 3usize), // i 가 j 의 위
            ];
            let best = cands
                .iter()
                .cloned()
                .fold((f32::MIN, 4usize), |acc, c| if c.0 > acc.0 { c } else { acc });
            if best.0 < 0.0 {
                // 양축 모두 bbox 가 겹친다 -> 축별 제약으로 표현 불가
                stats.skipped_pairs += 1;
                continue;
            }
            let mut e = LinearExpr::empty();
            match best.1 {
                0 => {
                    e.add(dx[i], 1.0);
                    e.add(dx[j], -1.0);
                }
                1 => {
                    e.add(dx[j], 1.0);
                    e.add(dx[i], -1.0);
                }
                2 => {
                    e.add(dy[i], 1.0);
                    e.add(dy[j], -1.0);
                }
                _ => {
                    e.add(dy[j], 1.0);
                    e.add(dy[i], -1.0);
                }
            }
            lp.add_constraint(e, ComparisonOp::Le, ((best.0 - LP_MARGIN).max(0.0)).min(best.0 * KEEP_RATIO) as f64);
            rows += 1;
            continue;
        }

        // G2: F단계 NFP 조각의 에지 법선을 그대로 제약으로 쓴다.
        // 세계 좌표에서 계산하므로 현재 상대 위치는 원점이고,
        // 변수는 Δrel = (Δx_j - Δx_i, Δy_j - Δy_i) 다.
        //
        // 1패스: 도달 범위 안의 조각 쌍마다 "원점에서 여유가 가장 큰 에지"를 모은다.
        // 여유가 음수인 조각이 하나라도 있으면 원점이 그 조각 내부라는 뜻이고
        // (볼록 분해의 과대근사에서 나온다) 그 쌍은 반평면으로 표현할 수 없다.
        // 그런 쌍은 상대 이동을 고정해 새 겹침이 생기지 않게 한다.
        let mut halfplanes: Vec<(f32, f32, f32)> = Vec::new(); // (nx, ny, d)
        let mut degenerate = false;
        for pb in snaps[i].pieces.iter() {
            if should_stop() { return None; }
            let bb_b = bbox_of(pb);
            for pa in snaps[j].pieces.iter() {
            if should_stop() { return None; }
                let bb_a = bbox_of(pa);
                // NFP 조각의 bbox = B ⊕ (−A). 이동 가능 범위와 안 겹치면 건너뛴다.
                let nx0 = bb_b.x_min - bb_a.x_max;
                let ny0 = bb_b.y_min - bb_a.y_max;
                let nx1 = bb_b.x_max - bb_a.x_min;
                let ny1 = bb_b.y_max - bb_a.y_min;
                if nx1 < -reach || nx0 > reach || ny1 < -reach || ny0 > reach {
                    continue;
                }
                let a_neg = nfp::negate_ccw(pa);
                let poly = nfp::nfp_convex(pb, &a_neg);
                if poly.len() < 3 {
                    continue;
                }
                let mut best: Option<(f32, f32, f32)> = None; // (d, nx, ny)
                let m = poly.len();
                for k in 0..m {
                    let p1 = poly[k];
                    let p2 = poly[(k + 1) % m];
                    let enx = p2.1 - p1.1;
                    let eny = p1.0 - p2.0;
                    let c = p2.1 * p1.0 - p2.0 * p1.1;
                    let len = (enx * enx + eny * eny).sqrt();
                    if len < 1e-9 {
                        continue;
                    }
                    let d = -c / len;
                    if best.map_or(true, |(bd, _, _)| d > bd) {
                        best = Some((d, enx / len, eny / len));
                    }
                }
                let Some((d, enx, eny)) = best else { continue };
                if d < -1e-3 {
                    degenerate = true;
                    break;
                }
                halfplanes.push((enx, eny, d));
            }
            if degenerate {
                break;
            }
        }

        if degenerate {
            // 상대 이동 고정: Δx_i = Δx_j, Δy_i = Δy_j
            stats.frozen_pairs += 1;
            for (a, b) in [(dx[i], dx[j]), (dy[i], dy[j])] {
                let mut e = LinearExpr::empty();
                e.add(a, 1.0);
                e.add(b, -1.0);
                lp.add_constraint(e, ComparisonOp::Eq, 0.0);
                rows += 1;
            }
            continue;
        }

        for (enx, eny, d) in halfplanes {
            if d >= BIG_SLACK {
                continue; // 사실상 무제한 -> 행을 만들지 않는다
            }
            let rhs = (((d - LP_MARGIN).max(0.0)).min(d * KEEP_RATIO) * cfg.relax_factor) as f64;
            let mut e = LinearExpr::empty();
            e.add(dx[j], -enx as f64);
            e.add(dx[i], enx as f64);
            e.add(dy[j], -eny as f64);
            e.add(dy[i], eny as f64);
            lp.add_constraint(e, ComparisonOp::Le, rhs);
            rows += 1;
        }
    }
    stats.rows = rows;

    if should_stop() { return None; }
    let mut solve_limit = std::time::Duration::from_millis(cfg.lp_time_limit_ms);
    if let Some(deadline) = deadline {
        solve_limit = solve_limit.min(deadline.checked_duration_since(Instant::now()).unwrap_or_default());
    }
    if solve_limit.is_zero() { return None; }
    // microlp's own deadline is cooperative; external Stop is detected after
    // this atomic solve, whose budget cannot exceed the absolute phase budget.
    lp.set_time_limit(solve_limit);
    let sol = match lp.solve() {
        Ok(SolveOutcome::Solution(s)) => s,
        Ok(SolveOutcome::Interrupted(_)) => {
            stats.lp_interrupted += 1;
            return None;
        }
        Err(_) => {
            stats.lp_fail += 1;
            return None;
        }
    };

    if should_stop() { return None; }
    let dxy: Vec<(f32, f32)> = (0..n)
        .map(|i| (sol[dx[i]] as f32, sol[dy[i]] as f32))
        .collect();
    let new_w = (sol[w_var] as f32).max(1.0);

    // 폭 결정 조각과 그 이동 여유를 기록한다.
    {
        let mut worst = (f32::MIN, 0usize);
        for i in 0..n {
            let x = snaps[i].bbox.x_max + dxy[i].0;
            if x > worst.0 {
                worst = (x, i);
            }
        }
        stats.blocking_item = Some(snaps[worst.1].item_id);
        stats.blocking_travel = dxy[worst.1].0;
    }

    // Δ 적용 후 CDE 정확 검사. 겹치면 적용 전으로 되돌린다.
    let need = snaps
        .iter()
        .zip(dxy.iter())
        .map(|(s, d)| s.bbox.x_max + d.0)
        .fold(0.0f32, f32::max);
    let cand = rebuild(instance, need.max(new_w) * 1.00002, &snaps, &dxy, should_stop)?;
    if !cand.layout().is_collision_free() {
        stats.reverted += 1;
        return None;
    }
    if should_stop() { return None; }
    stats.applied += 1;
    Some(cand)
}

/// 압축 반복. `iters` 회까지 돌리고, `rollback` 이면 최선 해를 채택한다.
pub fn compact(
    instance: &SPInstance,
    sol: &SPSolution,
    cfg: &CompactCfg,
    iters: u32,
) -> (SPSolution, CompactStats) {
    compact_with_control(instance, sol, cfg, iters, None, &|| false)
}

/// Cancellation-safe counterpart of compact. Interrupted work never installs
/// a partial rebuild; the prior completed iteration/input is retained.
pub fn compact_with_control(
    instance: &SPInstance,
    sol: &SPSolution,
    cfg: &CompactCfg,
    iters: u32,
    deadline: Option<Instant>,
    should_stop: &(impl Fn() -> bool + Sync),
) -> (SPSolution, CompactStats) {
    let stopped = || should_stop() || deadline.is_some_and(|dl| Instant::now() >= dl);
    let mut stats = CompactStats::default();
    stats.width_before = sol.strip_width();
    stats.width_after = sol.strip_width();
    if stopped() { return (sol.clone(), stats); }
    let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
    prob.restore(sol);
    stats.width_before = prob.strip_width();

    let mut best = sol.clone();
    let mut best_w = prob.strip_width();
    let mut prev_w = best_w;

    for it in 0..iters {
        if stopped() { break; }
        let Some(next) = compact_iteration(instance, &prob, cfg, &mut stats, deadline, &stopped) else {
            break;
        };
        if stopped() { break; }
        stats.lp_iters_used = (it + 1) as usize;
        let mut next = next;
        next.fit_strip().expect(crate::dmsg!("strip width must be positive"));
        let w = next.strip_width();
        if cfg.rollback && w < best_w {
            best_w = w;
            best = next.save();
        }
        let improved = prev_w - w;
        prob = next;
        prev_w = w;
        if improved < cfg.improve_tol {
            break;
        }
    }

    if !cfg.rollback {
        prob.fit_strip().expect(crate::dmsg!("strip width must be positive"));
        best = prob.save();
        best_w = prob.strip_width();
    }
    stats.width_after = best_w;
    (best, stats)
}

fn bbox_of(pts: &[Point]) -> Rect {
    let mut r = Rect {
        x_min: f32::MAX,
        y_min: f32::MAX,
        x_max: f32::MIN,
        y_max: f32::MIN,
    };
    for p in pts {
        r.x_min = r.x_min.min(p.0);
        r.y_min = r.y_min.min(p.1);
        r.x_max = r.x_max.max(p.0);
        r.y_max = r.y_max.max(p.1);
    }
    r
}

/// 남은 압축 여지 측정용: 각 조각을 좌/하단으로 재슬라이딩했을 때 이동 가능 거리의 합.
/// **해를 바꾸지 않는다** (측정만 하고 되돌린다).
pub fn slack_sum(instance: &SPInstance, sol: &SPSolution, iters: u32) -> f64 {
    use jagua_rs::collision_detection::hazards::filter::NoFilter;
    use jagua_rs::geometry::Transformation;
    use jagua_rs::geometry::geo_traits::{Transformable, TransformableFrom};

    let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
    prob.restore(sol);
    let keys: Vec<_> = prob
        .layout()
        .placed_items()
        .iter()
        .map(|(pk, _)| pk)
        .collect();
    let strip_h = instance.base_strip.fixed_height;
    let mut total = 0.0f64;
    for pk in keys {
        let pl = prob.remove_item(pk);
        let item = instance.item(pl.item_idx);
        let base = (*item.shape_cd()).transform_clone(&Transformation::from(pl.d_transf));
        let mut buff = base.clone();
        let bb = base.bbox();
        // 원래 위치를 (0,0) 오프셋으로 보고 좌/하로 얼마나 갈 수 있는지 이분 탐색
        for axis_x in [true, false] {
            let max_travel = if axis_x {
                bb.x_min - BOUND_EPS
            } else {
                bb.y_min - BOUND_EPS
            };
            if max_travel <= 0.01 {
                continue;
            }
            let (mut lo, mut hi) = (0.0f32, max_travel);
            for _ in 0..iters {
                if hi - lo < 0.01 {
                    break;
                }
                let mid = 0.5 * (lo + hi);
                let t = if axis_x {
                    Transformation::from_translation((-mid, 0.0))
                } else {
                    Transformation::from_translation((0.0, -mid))
                };
                buff.transform_from(&base, &t);
                let hit = buff.bbox().y_max > strip_h - BOUND_EPS
                    || prob.layout().cde().detect_poly_collision(&buff, &NoFilter);
                if hit { hi = mid } else { lo = mid }
            }
            total += lo as f64;
        }
        prob.place_item(SPPlacement {
            item_idx: pl.item_idx,
            d_transf: pl.d_transf,
        });
    }
    total
}
