//! 잔여조각 LP (§6) — 진입 가드 §5.14.
//!
//! 잔여 조각(Draft·Complete 뒤에도 놓이지 못한 조각)이 없으면 그대로 통과한다(§4.1, §5.14).
//! 있으면 한 조각씩, 시도마다 LP 를 새로 만들고 풀고 버린다.
//!
//! - 열 3개 (Δx, Δy, t), 목적 = t 최소화.
//! - 모서리 반평면 행: §5.3 과 같은 계수(여기서는 안전한 [`TrustScheme::Sound`] 행).
//! - 24방향 지지함수 행: `−cos_i·Δx − sin_i·Δy − t ≤ −(rx·cos_i + ry·sin_i)` — 곧
//!   `t ≥ (r − Δ)·u_i` 이고, 남은 변위의 정24각형 근사 거리를 최소화한다.
//!
//! 조각은 현재 사용 폭 오른쪽 바닥(언제나 비어 있다)에서 출발하고, `r` = 원단 왼쪽 끝(같은
//! 높이)까지 남은 변위다. 지시서의 간격 완화 재시도 인자(`(요구 − 기준) × K`)는 요구·기준
//! 값이 정의되지 않아 구현하지 않았다.

use crate::compact::{self, CompactConfig, coll_boxes, inside_fabric, nfp_between, violations, width_grid};
use crate::draft::State;
use crate::geom::{BBox, GRID_PER_MM, Pt};
use crate::io::{Instance, Placement};
use crate::lp::{self, Edge, Lp, LpStatus, SpecRows, TrustScheme};
use crate::nfp::NfpCache;
use std::f64::consts::PI;
use std::time::Instant;

/// 지지함수 방향 수 (15° 간격).
pub const DIRS: usize = 24;

#[derive(Clone, Debug, Default)]
pub struct Report {
    /// 진입 시 잔여 조각 수. 0 이면 통과했다.
    pub residual: usize,
    pub placed: usize,
    pub lp_solves: usize,
    pub infeasible: usize,
    pub moved_mm: f64,
}

pub fn run(
    inst: &Instance,
    placed: &mut Vec<Placement>,
    states: &mut [State],
    cache: &mut NfpCache,
    cfg: &CompactConfig,
    deadline: Instant,
) -> Report {
    let residual: Vec<usize> = states
        .iter()
        .enumerate()
        .filter(|(_, s)| matches!(s, State::Failed | State::Pending | State::Held))
        .map(|(k, _)| k)
        .collect();
    if residual.is_empty() {
        return Report::default();
    }
    let boxes = coll_boxes(inst);
    let mut rep = Report { residual: residual.len(), ..Default::default() };
    for k in residual {
        if Instant::now() >= deadline {
            break;
        }
        let Some(&shape) = inst.pieces[k].shapes.first() else { continue };
        let start = Placement { piece: k, shape, pos: Pt::new(width_grid(inst, placed), 0) };
        if !inside_fabric(inst, &start) {
            continue; // 원단보다 높은 조각
        }
        placed.push(start);
        let idx = placed.len() - 1;
        if violations(inst, placed, cache, &boxes, Some(idx)) > 0 {
            placed.pop();
            continue;
        }
        states[k] = State::Placed;
        rep.placed += 1;

        let mut step = cfg.step_box;
        for _ in 0..cfg.max_iter {
            if Instant::now() >= deadline {
                break;
            }
            let cur = placed[idx];
            let target = Pt::new(-cur.pos.x, 0);
            let b = inst.shapes[shape].bbox;
            let mut lp = Lp::new(3);
            lp.set_col_bounds(0, (-step).max(-cur.pos.x).min(0) as f64, step as f64);
            let hi = step.min(inst.strip_height - b.y1 - cur.pos.y).max(0);
            lp.set_col_bounds(1, (-step).max(-cur.pos.y).min(0) as f64, hi as f64);
            lp.set_col_bounds(2, 0.0, lp::INF);
            lp.set_obj(2, 1.0);
            let me = boxes[shape].translate(cur.pos);
            for (q_idx, q) in placed.iter().enumerate() {
                if q_idx == idx {
                    continue;
                }
                let other = boxes[q.shape].translate(q.pos);
                let gx = (other.x0 - me.x1).max(me.x0 - other.x1).max(0);
                let gy = (other.y0 - me.y1).max(me.y0 - other.y1).max(0);
                if gx > cfg.active_margin || gy > cfg.active_margin {
                    continue;
                }
                let Some(nf) = nfp_between(inst, cache, q, shape) else { continue };
                let r = cur.pos - q.pos;
                let win = BBox { x0: r.x - step, y0: r.y - step, x1: r.x + step, y1: r.y + step };
                if !nf.bbox.overlaps(&win) {
                    continue;
                }
                for ring in &nf.loops {
                    let m = ring.len();
                    for e in 0..m {
                        let (q1, q2) = (ring[e], ring[(e + 1) % m]);
                        let sb = BBox { x0: q1.x.min(q2.x), y0: q1.y.min(q2.y), x1: q1.x.max(q2.x), y1: q1.y.max(q2.y) };
                        if !sb.overlaps(&win) {
                            continue;
                        }
                        let edge = Edge::new(q2, q1);
                        lp::edge_rows(&mut lp, TrustScheme::Sound, SpecRows::default(), [None, Some(0), None, Some(1)], &edge, r, None);
                    }
                }
            }
            for i in 0..DIRS {
                let a = 2.0 * PI * i as f64 / DIRS as f64;
                let (s, c) = a.sin_cos();
                lp.new_row();
                lp.add_coef(0, -c);
                lp.add_coef(1, -s);
                lp.add_coef(2, -1.0);
                lp.set_bounds(None, lp::FREE_LOWER, -(target.x as f64 * c + target.y as f64 * s));
            }
            let solved = lp.solve(deadline.saturating_duration_since(Instant::now()));
            rep.lp_solves += 1;
            if solved.status != LpStatus::Optimal {
                rep.infeasible += 1;
                break;
            }
            let d = Pt::new(solved.x[0].trunc() as i64, solved.x[1].trunc() as i64);
            if d.x.abs().max(d.y.abs()) < cfg.move_min {
                break;
            }
            placed[idx].pos = cur.pos + d;
            if violations(inst, placed, cache, &boxes, Some(idx)) > 0 {
                placed[idx] = cur;
                step /= 2;
                if step < cfg.move_min {
                    break;
                }
                continue;
            }
            rep.moved_mm += ((d.x * d.x + d.y * d.y) as f64).sqrt() / GRID_PER_MM;
        }
    }
    let _ = compact::width_grid; // 모듈 연결 확인용
    rep
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io;
    use std::time::Duration;

    #[test]
    fn passes_through_when_nothing_is_left() {
        let path = std::env::temp_dir().join(format!("nesting_engine_residual_{}.json", std::process::id()));
        std::fs::write(
            &path,
            r#"{"name":"t","strip_height":10,"items":[{"id":0,"demand":2,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[4,0],[4,4],[0,4]]}}]}"#,
        )
        .unwrap();
        let inst = io::load(&path, 0.0).unwrap();
        let _ = std::fs::remove_file(&path);
        let mut cache = NfpCache::default();
        let cfg = CompactConfig::default();
        let deadline = Instant::now() + Duration::from_secs(5);

        let mut placed = vec![Placement { piece: 0, shape: 0, pos: Pt::new(0, 0) }];
        let mut states = vec![State::Placed, State::Placed];
        let rep = run(&inst, &mut placed, &mut states, &mut cache, &cfg, deadline);
        assert_eq!(rep.residual, 0);
        assert_eq!(placed.len(), 1);

        // 두 번째 조각이 실패로 남았으면 오른쪽에서 출발해 첫 조각에 붙는다.
        let mut states = vec![State::Placed, State::Failed];
        let rep = run(&inst, &mut placed, &mut states, &mut cache, &cfg, deadline);
        assert_eq!((rep.residual, rep.placed), (1, 1));
        assert_eq!(states[1], State::Placed);
        assert_eq!(placed[1].pos.x, 400, "{placed:?} {rep:?}");
        assert!(crate::validate::certify(&inst, &placed).overlaps.is_empty());
    }
}
