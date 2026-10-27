//! 신규 Init 전체 (§0.1, §4.1).
//!
//! ```text
//! 전처리(NFP) → Draft → [재시작 + LP] → Complete → 잔여조각 LP → 밀어내기 압축 → 최종 검증
//! ```
//!
//! - 전체 20초, 단조 시계. 측정 시작 = 전처리 진입(호출자가 넘기는 `start`), 끝 = 최종 검증.
//! - 단계마다 원본 기하로 인증하고, 인증을 통과한 스냅샷만 best 로 받는다(§8.1). 새 결과가
//!   더 나쁘거나 인증에 실패하면 이전 best 를 유지한다 — 게이트를 낮추거나 겹침을 허용하는
//!   전역 폴백은 없다.
//! - "더 좋다" = 인증 통과 → 배치 수가 많다 → 같은 배치 수에서 사용 폭이 좁다.

use crate::chain::{Chain, Refine};
use crate::compact::{self, CompactConfig};
use crate::complete;
use crate::draft::{self, State};
use crate::io::{Instance, Placement};
use crate::nfp::NfpCache;
use crate::pushin::{self, FinalCommit, PointArrays};
use crate::residual;
use crate::state::EngineState;
use crate::validate::{self, Report};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// 전체 상한 (§0.1.1).
pub const TOTAL_BUDGET: Duration = Duration::from_secs(20);
/// 최종 검증 몫. 20초는 최종 검증(겹침 0 확인) 완료까지이므로 탐색 단계는 이만큼 일찍 멈춘다.
pub const FINAL_RESERVE: Duration = Duration::from_millis(500);

#[derive(Clone, Debug)]
pub struct PipelineConfig {
    pub draft: draft::Config,
    pub lp: CompactConfig,
    pub lp_enabled: bool,
    pub complete: bool,
    pub complete_wait: Duration,
    pub residual: bool,
    pub budget: Duration,
    /// 원단 밖 허용 (mm) — 입력 계약.
    pub fabric_budget_mm: f64,
    /// 초기 배치 **이후**를 무엇으로 잇는가 (§12.1.32).
    pub chain: Chain,
    /// ★★★ §12.1.64 AU172 — **개선 단계**를 무엇으로 하는가 (§12.1.32 의 자매 축).
    ///
    /// `chain`  = **초기 배치** (Draft→Complete / swarm LBF)
    /// `refine` = **개선 단계** (Complete 이후 ④~⑦ / swarm exp→comp→polish)
    ///
    /// 두 열은 대응하며 서로 독립이다. 같은 이름(`swarm`)이 다른 뜻으로 쓰이므로 축을 분리했다.
    pub refine: Refine,
}

impl Default for PipelineConfig {
    fn default() -> Self {
        Self {
            draft: draft::Config::default(),
            lp: CompactConfig::default(),
            lp_enabled: true,
            complete: true,
            complete_wait: complete::WORKER_WAIT,
            residual: true,
            budget: TOTAL_BUDGET,
            fabric_budget_mm: 0.1,
            chain: Chain::Complete,
            refine: Refine::AutoNest,
        }
    }
}

/// 단계 보고 한 줄 (§11.2: 효율·사용 폭·배치 수·겹침 수).
#[derive(Clone, Debug)]
pub struct Stage {
    pub name: &'static str,
    pub density: f64,
    pub width_mm: f64,
    pub placed: usize,
    pub total: usize,
    pub overlaps: usize,
    pub overrun_mm: f64,
    /// 인증 통과 + 전부 배치.
    pub certified: bool,
    /// 이 단계 결과를 best 로 받았는가.
    pub adopted: bool,
    pub at_ms: u128,
    pub note: String,
}

pub struct Outcome {
    pub placements: Vec<Placement>,
    pub final_report: Report,
    pub stages: Vec<Stage>,
    pub draft: draft::Stats,
    pub lp: Option<compact::Report>,
    pub complete_lp: Option<compact::Report>,
    /// 원천 Complete 가 산출한 항목별 이동변수 상자 (§12.1.34 AU42). 재배치가 아니다.
    pub complete_boxes: Option<Vec<(i64, i64)>>,
    pub residual: residual::Report,
    pub commit: FinalCommit,
    pub elapsed_ms: u128,
}

fn ok(r: &Report, fabric: f64) -> bool {
    r.overlaps.is_empty() && r.overrun.iter().all(|&o| o <= fabric)
}

/// `a` 가 `b` 보다 좋은가.
fn better(a: &Report, b: &Report, fabric: f64) -> bool {
    if !ok(a, fabric) {
        return false;
    }
    if !ok(b, fabric) {
        return true;
    }
    if a.placed != b.placed {
        return a.placed > b.placed;
    }
    a.used_width_mm < b.used_width_mm - 1e-9
}

fn stage(name: &'static str, r: &Report, fabric: f64, start: Instant, adopted: bool, note: String) -> Stage {
    Stage {
        name,
        density: r.density,
        width_mm: r.used_width_mm,
        placed: r.placed,
        total: r.total,
        overlaps: r.overlaps.len(),
        overrun_mm: r.overrun.iter().copied().fold(0.0, f64::max),
        certified: ok(r, fabric) && r.placed == r.total,
        adopted,
        at_ms: start.elapsed().as_millis(),
        note,
    }
}

pub fn run(inst: Arc<Instance>, mut cfg: PipelineConfig, start: Instant) -> Outcome {
    let deadline = start + cfg.budget;
    let search_deadline = deadline.checked_sub(FINAL_RESERVE).filter(|d| *d > start).unwrap_or(deadline);
    let fabric = cfg.fabric_budget_mm;
    let mut stages = Vec::new();

    // ★★★ §12.1.64 AU172 — 개선 단계 선택을 **부분 이동 전에** 확정한다.
    //   `cfg.draft` 가 아래에서 이동되므로 여기서 읽어 둔다.
    //   ★ `Refine::AutoNest` 일 때만 원천 ⑤-a/⑤-b 를 켠다(AU170.5 실측: 켜면 악화).
    let refine = cfg.refine;
    {
        let mut draft_cfg = cfg.draft.clone();
        if refine.runs_autonest() {
            // 원천 ⑤-a 그룹 LP 는 «미배치가 남은 중간 상태» 를 전제하므로 기본은 끈다.
            // 대조 실험을 위해 `--canonical-group-lp` 로 따로 켤 수 있다.
            draft_cfg.canonical_slack = true;
        }
        cfg.draft = draft_cfg;
    }

    // ① Draft
    //
    // ★★★ §12.1.80 AU189 — **시작점별 방향 왕복** (원천 `FUN_1002a930`).
    //
    // 원천은 배치 중 `+0x368 = 2`(역방향) ↔ `1`(정방향) 을 **번갈아** 돌린다:
    // ```c
    // do {
    //     +0x368 = 2;  cos/sin 반전;  FUN_1004ac70();   // ★ 역방향
    //     +0x368 = 1;                  FUN_1004ac70();   // ★ 정방향
    //     iVar6++;
    // } while (iVar6 < 512);
    // ```
    // 역방향의 실체는 **LP 목적계수 미설정** 이다 (`compact.rs` `reverse_mode`).
    //
    // **왕복 단위는 «시작점 1개» 다** — `draft::Config::bidir_swap` 이 담당한다.
    // 전체 스윕을 단위로 하면 1회 왕복이 예산을 다 먹는다(실측 AU189: 정방향 100초
    // + 역방향 79초 = 179초, 왕복 1회로 종료).
    let dcfg = draft::Config { deadline: Some(search_deadline), ..cfg.draft };
    let draft::Outcome { placements, states, stats: draft_stats, cache, timed_out, .. } =
        draft::run_with(&inst, dcfg.clone(), None, NfpCache::default());
    let mut cache = cache;
    let mut states = states;
    let draft_rep = validate::certify(&inst, &placements);
    stages.push(stage(
        "Draft",
        &draft_rep,
        fabric,
        start,
        true,
        format!("NFP {} ({}ms, 히트 {}) · 전역 후보 배치 {} · 시간초과 {timed_out}", draft_stats.nfp_computed, draft_stats.nfp_ms, draft_stats.nfp_hits, draft_stats.global_placed),
    ));
    let mut best = placements;
    let mut best_rep = draft_rep;

    // ② 재시작 + LP
    let mut lp_adopted = false;
    let lp = if cfg.lp_enabled {
        let mut cand = best.clone();
        let r = compact::restart_improve(&inst, &mut cand, &mut cache, &cfg.lp, search_deadline);
        let rep = validate::certify(&inst, &cand);
        let adopt = better(&rep, &best_rep, fabric);
        stages.push(stage(
            "Draft+LP",
            &rep,
            fabric,
            start,
            adopt,
            format!(
                "반복 {} · x/y 수락 {}/{} · 거절 {} · 반감 {} · 불능 {} · LP {}ms · 최대 행 {} · 종료: {}",
                r.iterations, r.x_accepted, r.y_accepted, r.rejected, r.halvings, r.infeasible, r.lp_ms, r.max_rows, r.stop
            ),
        ));
        if adopt {
            best = cand;
            best_rep = rep;
            lp_adopted = true;
        }
        Some(r)
    } else {
        None
    };

    // ③ Complete — `chain` 이 신규 엔진 후처리를 돌릴 때만 (§12.1.32).
    //   `--chain none|swarm` 은 Draft 에서 멈춘다(`Chain::runs_complete`). 종전에는 파싱만
    //   하고 읽지 않아 `--chain` 이 무효였다.
    let mut complete_lp = None;
    let mut complete_boxes: Option<Vec<(i64, i64)>> = None;
    let mut complete_adopted = false;
    if cfg.complete && refine.runs_autonest() && cfg.chain.runs_complete() {
        match complete::run(
            Arc::clone(&inst),
            &best,
            cache.clone(),
            cfg.lp.clone(),
            cfg.lp_enabled,
            search_deadline,
            cfg.complete_wait,
        ) {
            complete::Outcome::Done(d) => {
                let adopt = better(&d.report, &best_rep, fabric);
                // 원천 Complete 는 «이동변수 상자 산출 + LP» 다 — 재배치가 아니다(§12.1.34 AU41).
                let n_items = d.boxes.iter().filter(|&&(a, b)| a != b).count();
                stages.push(stage(
                    "Complete",
                    &d.report,
                    fabric,
                    start,
                    adopt,
                    format!("이동변수 상자 {n_items}개 항목 + LP · 워커 {}ms", d.elapsed_ms),
                ));
                let done = *d;
                complete_lp = done.lp;
                complete_boxes = Some(done.boxes);
                if adopt {
                    best = done.placements;
                    best_rep = done.report;
                    complete_adopted = true;
                }
            }
            complete::Outcome::TimedOut { waited_ms } => {
                stages.push(stage(
                    "Complete",
                    &best_rep,
                    fabric,
                    start,
                    false,
                    format!("still running — {waited_ms}ms 대기 초과, 현재 best 유지"),
                ));
            }
            complete::Outcome::Skipped(why) => {
                stages.push(stage("Complete", &best_rep, fabric, start, false, why));
            }
        }
    }

    // ④ 잔여조각 LP (§5.14: 잔여 조각이 없으면 통과)
    let mut residual_adopted = false;
    let residual = if cfg.residual && refine.runs_autonest() {
        let mut cand = best.clone();
        let mut st = states.clone();
        let r = residual::run(&inst, &mut cand, &mut st, &mut cache, &cfg.lp, search_deadline);
        if r.residual > 0 {
            let rep = validate::certify(&inst, &cand);
            let adopt = better(&rep, &best_rep, fabric);
            stages.push(stage("Residual", &rep, fabric, start, adopt, format!("잔여 {} → 배치 {}", r.residual, r.placed)));
            if adopt {
                best = cand;
                states = st;
                residual_adopted = true;
            }
        }
        r
    } else {
        residual::Report::default()
    };
    let _ = states.iter().filter(|s| **s == State::Placed).count();

    // ⑥ 최종 검증 (원본 기하)
    let final_report = validate::certify(&inst, &best);

    // chain=swarm/both 이면 이 배치가 warm start 로 넘어간다 (§12.1.32).
    // 실제 파일 쓰기는 호출자(main)가 한다 — 여기서는 어느 배치가 넘어가는지만 확정한다.
    if refine.exports_warmstart() {
        stages.push(stage(
            "WarmStart",
            &final_report,
            fabric,
            start,
            true,
            format!(
                "초기해 확정 — swarm(exp→comp→polish) warm start 대상 · 폭 {:.2}mm",
                final_report.used_width_mm
            ),
        ));
    }

    // ⑤ 밀어내기 압축 (§5.11) — 최종 경로(0x200)만 부른다. 루프 안 경로(`pushin::push_in`)는
    // 부르지 않는다: 원천의 점 배열은 누적 오프셋이라 이 엔진의 배치 좌표에 대응되지 않고,
    // 제17차 AB6 을 따르면 배치수 = 전체수 라 언제나 무동작이다.
    // 점수 이력은 채택된 단계의 것만 쓰고 끝에 최종 폭을 붙인다(잔여조각이 폭을 넓혔으면 최종
    // 폭만). LP 가 시간 상한에 걸려 멈췄으면(상태 0x100) 압축 단계 완료로 확정하지 않는다.
    let pts: Vec<(i64, i64)> = best.iter().map(|p| (p.pos.x, p.pos.y)).collect();
    let mut arrays = PointArrays::new(pts.clone(), pts);
    let mut hist: Vec<f64> = Vec::new();
    if !residual_adopted {
        if lp_adopted && let Some(r) = &lp {
            hist.extend_from_slice(&r.score_hist);
        }
        if complete_adopted && let Some(r) = &complete_lp {
            hist.extend_from_slice(&r.score_hist);
        }
    }
    hist.push(final_report.used_width_mm);
    let partial = [&lp, &complete_lp].iter().any(|r| r.as_ref().is_some_and(|r| r.state == Some(EngineState::Partial)));
    let mut commit = pushin::final_commit(&hist, &mut arrays);
    if partial {
        commit.state = EngineState::Partial;
    }
    let over = start.elapsed() > cfg.budget;
    stages.push(stage(
        "Final",
        &final_report,
        fabric,
        start,
        true,
        format!("상태 0x{:x}{}", commit.state.code(), if over { " · 20초 예산 초과" } else { "" }),
    ));

    Outcome {
        placements: best,
        final_report,
        stages,
        draft: draft_stats,
        lp,
        complete_lp,
        complete_boxes,
        residual,
        commit,
        elapsed_ms: start.elapsed().as_millis(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io;

    #[test]
    fn the_whole_pipeline_keeps_every_piece_and_never_overlaps() {
        let path = std::env::temp_dir().join(format!("nesting_engine_pipeline_{}.json", std::process::id()));
        std::fs::write(
            &path,
            r#"{"name":"t","strip_height":10,"items":[
                {"id":0,"demand":2,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[4,0],[4,6],[0,6]]}},
                {"id":1,"demand":3,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[3,0],[3,3],[0,3]]}},
                {"id":2,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[6,0],[6,2],[3,5],[0,2]]}}
            ]}"#,
        )
        .unwrap();
        let inst = Arc::new(io::load(&path, 0.8).unwrap());
        let _ = std::fs::remove_file(&path);
        let out = run(Arc::clone(&inst), PipelineConfig::default(), Instant::now());
        assert_eq!(out.final_report.placed, inst.pieces.len());
        assert!(out.final_report.overlaps.is_empty(), "{:?}", out.final_report.overlaps);
        assert_eq!(out.stages.first().unwrap().name, "Draft");
        assert_eq!(out.stages.last().unwrap().name, "Final");
        // 최종은 Draft 보다 나쁘지 않다.
        let draft = &out.stages[0];
        assert!(out.final_report.used_width_mm <= draft.width_mm + 1e-9);
    }

    /// `--chain none` 은 Complete 단계를 돌리지 않는다 (종전에는 `chain` 이 무시됐다).
    #[test]
    fn chain_none_skips_the_complete_stage() {
        let path = std::env::temp_dir().join(format!("nesting_engine_pipeline_chain_{}.json", std::process::id()));
        std::fs::write(
            &path,
            r#"{"name":"t","strip_height":10,"items":[
                {"id":0,"demand":2,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[4,0],[4,6],[0,6]]}},
                {"id":1,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[3,0],[3,3],[0,3]]}}
            ]}"#,
        )
        .unwrap();
        let inst = Arc::new(io::load(&path, 0.0).unwrap());
        let _ = std::fs::remove_file(&path);
        let cfg = PipelineConfig { chain: Chain::None, ..PipelineConfig::default() };
        let out = run(Arc::clone(&inst), cfg, Instant::now());
        assert!(out.stages.iter().all(|s| s.name != "Complete"), "{:?}", out.stages.iter().map(|s| s.name).collect::<Vec<_>>());
        assert!(out.complete_boxes.is_none());
        let with = run(Arc::clone(&inst), PipelineConfig::default(), Instant::now());
        assert!(with.stages.iter().any(|s| s.name == "Complete"));
    }
}
