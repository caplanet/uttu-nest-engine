use crate::config::PolishConfig;
use crate::optimizer::separator::Separator;
use crate::util::listener::{ReportType, SolutionListener};
use crate::util::terminator::Terminator;
use jagua_rs::Instant;
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use log::info;
use std::time::Duration;

pub mod holes;
pub mod lp;
pub mod nfp_left;
pub mod probe;
pub mod slide;

/// 후처리(polish) 단계.
///
/// 탐색/압축이 끝난 **feasible** 해를 입력으로 받아, 확률적 재배치가 놓치는 두 가지를 메운다:
///
/// 1. [`lp::lp_compaction_pass`] — 강체 단위별 평행이동과 폭을 변수로 하는 선형계획을 풀어
///    전역적으로 폭을 먼저 줄인다.
/// 2. [`slide::slide_pass`] — 결정론적 좌측(및 하단) 슬라이딩. 겹치지 않는 아이템은 swarm의
///    separation 대상이 아니라 feasible 해에 남은 자잘한 틈이 방치되는데, 이를 직접 회수한다.
/// 두 패스 모두 이동을 반영하기 전에 충돌이 없음을 검증하므로 입력 해보다 나빠지지 않는다.
/// 고정(frozen) 아이템은 어느 패스에서도 이동하지 않는다.
/// # 기본값 래퍼를 두지 않는 이유
///
/// 예전에는 설정을 안 받고 `DEFAULT_NFP_INTEGRATION_CONFIG` / `DEFAULT_LP_SOLVER_CONFIG` 를
/// 하드코딩하는 `polish_phase` 래퍼가 있었다. 편의를 위한 것이었는데, 결과는
/// **프로덕션 경로 전부가 그 래퍼로 흘러 사용자 설정이 조용히 버려지는 것**이었다 —
/// step API(WPF·JNI), structure-first, grouped 엔진 셋 다였다. WPF 에서 Clarabel 을 골라도
/// microlp 가 돌았고, `nfp_hole_filling` 체크박스는 아무 일도 하지 않았다.
///
/// 빌드도 테스트도 통과했다. **기본값을 채워 주는 래퍼가 있는 한 배선 누락은 컴파일러에
/// 보이지 않기 때문이다.** 그래서 래퍼를 지웠다. 이제 후처리를 부르려면 두 설정을 반드시
/// 손에 쥐고 있어야 하고, 없으면 컴파일이 안 된다.
///
/// 새 설정 블록을 추가할 때도 같은 규칙을 지킬 것 — 기본값 래퍼를 만들지 마라.
///
#[allow(clippy::too_many_arguments)]
/// 폴리시 단계가 **왜** 끝났는가.
///
/// Active LP/slide rounds continue until the effective time budget or external stop.
/// Disabled/empty phases and infeasible inputs still return immediately.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolishExit {
    /// Local/parent time budget expired, or an external stop was requested.
    Killed,
    /// Disabled, no active passes, or a proven fixed point (the rest of the budget goes to Reclaim).
    NoChanges,
    /// Legacy diagnostic variant, retained for compatibility; no active round cap.
    RoundCapWithoutImprovement,
    /// Input could not be certified. Failed active rounds roll back and retry until the deadline.
    RolledBackInfeasible,
}

/// 폴리시 계측치.
#[derive(Debug, Clone, Copy)]
pub struct PolishStats {
    /// 완주한 라운드 수.
    pub rounds: usize,
    /// 모든 패스가 보고한 변경 건수의 합.
    pub changes: usize,
    /// 검증에 실패해 통째로 되돌린 라운드 수.
    pub failed_rounds: usize,
    /// 종료 사유.
    pub exit: PolishExit,
}

/// NFP 왼쪽 재배치를 켜는 데 필요한 남은 Polish 예산. 라이브러리 준비와 첫 쌍 NFP 계산이
/// 수백 ms 걸리므로, 이보다 짧은 호출에서는 slide 만 돈다.
const NFP_LEFT_MIN_BUDGET: Duration = Duration::from_secs(2);

/// Actual local passes, independent of Reclaim and Structure P5 rotation work.
/// Callers must preserve their certification and public phase-boundary contract.
pub fn has_active_passes(config: &PolishConfig, _grouped: bool) -> bool {
    config.enabled && (config.lp.enabled || config.slide.enabled)
}

pub fn polish_phase_with_nfp(
    instance: &SPInstance,
    sep: &mut Separator,
    init_sol: &SPSolution,
    sol_listener: &mut Option<&mut dyn SolutionListener>,
    term: &impl Terminator,
    config: &PolishConfig,
    _nfp_cfg: &crate::config::NfpIntegrationConfig,
    lp_solver: &crate::lp_solver::LpSolverConfig,
) -> SPSolution {
    polish_phase_measured(
        instance,
        sep,
        init_sol,
        sol_listener,
        term,
        config,
        _nfp_cfg,
        lp_solver,
    )
    .0
}

/// [`polish_phase_with_nfp`] 와 같되 계측치를 함께 낸다.
#[allow(clippy::too_many_arguments)]
pub fn polish_phase_measured(
    instance: &SPInstance,
    sep: &mut Separator,
    init_sol: &SPSolution,
    sol_listener: &mut Option<&mut dyn SolutionListener>,
    term: &impl Terminator,
    config: &PolishConfig,
    _nfp_cfg: &crate::config::NfpIntegrationConfig,
    lp_solver: &crate::lp_solver::LpSolverConfig,
) -> (SPSolution, PolishStats) {
    if !config.enabled {
        return (
            init_sol.clone(),
            PolishStats {
                rounds: 0,
                changes: 0,
                failed_rounds: 0,
                exit: PolishExit::NoChanges,
            },
        );
    }

    // Direct core callers need not install a parent timeout. Always enforce the
    // configured phase budget, while retaining a shorter parent deadline/stop.
    let phase_term = FractionalTerminator::with_budget(term, config.time_limit);
    let term = &phase_term;
    if term.kill() {
        return (
            init_sol.clone(),
            PolishStats {
                rounds: 0,
                changes: 0,
                failed_rounds: 0,
                exit: PolishExit::Killed,
            },
        );
    }

    // separator가 어떤 상태로 넘어왔든 입력 해로 되돌리고 시작한다.
    sep.rollback(init_sol, None);
    let corrected_initial = if crate::original_overlap::OriginalOverlapGuard::is_active(instance)
        && sep.ct.get_total_loss() == 0.0
        && sep.repair_original_overlap(term)
    {
        Some(sep.prob.save())
    } else {
        None
    };
    let init_sol = corrected_initial.as_ref().unwrap_or(init_sol);

    let initial_check_started = Instant::now();
    if sep.ct.get_total_loss() != 0.0 || !sep.solution_is_certified(init_sol, init_sol) {
        // 입력이 feasible하지 않으면 후처리의 전제가 깨진다.
        info!("[PLSH] initial solution is not feasible, skipping");
        return (
            init_sol.clone(),
            PolishStats {
                rounds: 0,
                changes: 0,
                failed_rounds: 0,
                exit: PolishExit::RolledBackInfeasible,
            },
        );
    }

    // An enabled-but-empty phase still accepts only a certified input (including
    // any required original repair above), but has no rounds to execute.
    if !has_active_passes(config, sep.has_place_groups()) {
        return (
            init_sol.clone(),
            PolishStats {
                rounds: 0,
                changes: 0,
                failed_rounds: 0,
                exit: PolishExit::NoChanges,
            },
        );
    }

    let mut checkpoint_cost = initial_check_started
        .elapsed()
        .max(Duration::from_millis(1));
    // LP and slide move rigid units together.
    let mut best_sol = init_sol.clone();

    // n_rounds remains in the public configuration for compatibility. Active
    // passes use the entire bounded budget, including after unchanged rounds.
    let mut round = 0;
    let mut total_changes = 0usize;
    let mut failed_rounds = 0usize;
    let mut exit = PolishExit::Killed;
    // NFP 왼쪽 재배치는 slide 와 함께 도는 보강이다. 쌍 NFP 는 라운드를 넘어 재사용하지만
    // 라이브러리는 호출마다 새로 만들므로, 예산이 짧은 호출(그룹 lane 조각, 회수 사이클)은
    // 건너뛴다. 묶음(place group)은 강체 단위라 이 패스가 다루지 않는다.
    let enough_budget = term.timeout_at().is_none_or(|deadline| {
        deadline.saturating_duration_since(Instant::now()) >= NFP_LEFT_MIN_BUDGET
    });
    let nfp_left_eligible = nfp_left_eligible(config, enough_budget, sep.has_place_groups());
    let mut nfp_cache = if nfp_left_eligible {
        crate::optimizer::separator::build_nfp_sampler_cache(instance)
    } else {
        None
    };
    loop {
        if term.kill() {
            break;
        }
        let round_started = Instant::now();
        let before_pose = pose_signature(&sep.prob.save());
        let before_state = round_state(sep);
        let mut lp_complete = !config.lp.enabled;
        let mut slide_complete = !config.slide.enabled;
        // nfp_left proves its own completion only when it ran to the end and moved nothing.
        let mut nfp_complete = nfp_cache.is_none();
        let round_start_width = best_sol.strip_width();
        let best_pose_at_round_start = pose_signature(&best_sol);
        let mut changes = 0;

        // 전역 LP가 큰 방향을 먼저 잡고, 결정론적 slide가 LP 선형화 뒤의 작은 틈을 회수한다.
        // Reserve part of the remaining time for slide. Microlp polls
        // the parent cancellation signal between resumable solver slices.
        if config.lp.enabled {
            let lp_fraction = if config.slide.enabled { 0.60 } else { 1.0 };
            let lp_term = FractionalTerminator::new(term, lp_fraction);
            let report = lp::lp_compaction_pass_report(sep, &config.lp, lp_solver, &lp_term);
            lp_complete = report.proves_completion(lp_solver);
            let applied = report.applied;
            changes += applied;
            // LP certifies every retained transaction; interrupted later trials
            // restore that result. Preserve it even when cancellation happened
            // inside the pass. No new geometry scan is needed here.
            // Certified equal-width moves report `applied == 0` but still change
            // the poses; keep those too so a later failed slide cannot drop them.
            let certified = sep.prob.save();
            if applied > 0 || pose_signature(&certified) != before_pose {
                let shorter = certified.strip_width() < best_sol.strip_width();
                if candidate_improves(&certified, &best_sol, sep, config) {
                    best_sol = certified;
                    if shorter {
                        if let Some(listener) = sol_listener.as_deref_mut() {
                            listener.report(ReportType::CmprFeas, &best_sol, instance);
                        }
                    }
                }
            }
        }
        if config.slide.enabled && !term.kill() {
            // 막힌 구간 너머의 왼쪽 빈자리로 먼저 옮기고, slide 가 남은 틈을 민다.
            if let Some(cache) = nfp_cache.as_mut() {
                let nfp_term = FractionalTerminator::new(term, 0.5);
                let moved_left = nfp_left::nfp_left_pass(sep, &config.nfp_left, cache, &nfp_term);
                nfp_complete = moved_left == 0 && !nfp_term.kill();
                changes += moved_left;
            }
            let moved = slide::slide_pass(sep, &config.slide, term);
            slide_complete = moved == 0 && !term.kill();
            changes += moved;
            if !term.kill() {
                sep.refit_strip();
            }
            {
                checkpoint_pass(
                    instance,
                    sep,
                    &mut best_sol,
                    sol_listener,
                    term,
                    &mut checkpoint_cost,
                );
            }
        }
        // On cancellation retain the already certified incumbent. Do not start
        // another geometry scan or refit for a partially completed round.
        if term.kill() {
            total_changes += changes;
            break;
        }
        // 어떤 패스도 실행 가능성을 깨뜨리면 안 된다. 만약 깨졌다면 마지막 정상 해로 되돌린다.
        //
        // A failed slide transaction cannot discard a certified LP incumbent.
        // Restore that incumbent and continue real passes for the remaining budget.
        // 슬라이드가 충돌을 남겼거나, 원본 겹침 복구 또는 정확 검증에 실패한 라운드.
        let round_failed = if sep.ct.get_total_loss() != 0.0 {
            true
        } else if !sep.repair_original_overlap(term) {
            if term.kill() {
                sep.rollback(&best_sol, None);
                break;
            }
            true
        } else {
            false
        };
        let sol = sep.prob.save();
        let round_failed = round_failed
            || (candidate_improves(&sol, &best_sol, sep, config)
                && !sep.solution_is_certified(&sol, &best_sol));
        if round_failed {
            failed_rounds += 1;
            sep.rollback(&best_sol, None);
            let best_changed = pose_signature(&best_sol) != best_pose_at_round_start;
            total_changes += usize::from(best_changed);
            round += 1;
            if !best_changed {
                pace_stalled_round(round_started, term);
            }
            continue;
        }
        let pose_changed = pose_signature(&sol) != before_pose;
        let width_improved = sol.strip_width() < round_start_width;
        // Keep equal-width left/down rearrangements for the next compression
        // cycle, but never accept a wider layout for a secondary objective.
        if candidate_improves(&sol, &best_sol, sep, config) {
            best_sol = sol;
            if width_improved {
                if let Some(l) = sol_listener.as_deref_mut() {
                    l.report(ReportType::CmprFeas, &best_sol, instance);
                }
            }
        }
        total_changes += changes.max(usize::from(pose_changed));
        round += 1;
        if term.kill() {
            break;
        }
        // This proof lives only in this call. Keys/traversal, geometry and live
        // width must match the input used by the deterministic operators.
        // A proven fixed point ends the phase immediately so the caller (Reclaim)
        // receives the leftover budget instead of this phase sleeping to the deadline.
        #[cfg(not(target_arch = "wasm32"))]
        if lp_complete
            && slide_complete
            && nfp_complete
            && changes == 0
            && before_state == round_state(sep)
        {
            exit = PolishExit::NoChanges;
            break;
        }
        #[cfg(target_arch = "wasm32")]
        let _ = (lp_complete, slide_complete, nfp_complete, before_state);
        if pose_signature(&best_sol) == best_pose_at_round_start {
            pace_stalled_round(round_started, term);
        }
    }

    // Initial and every accepted incumbent were certified before publication.
    // 최종적으로 가장 좋은 해로 맞춰 둔다.
    sep.rollback(&best_sol, None);

    // Active rounds end only at the effective deadline or an external stop.
    (
        best_sol,
        PolishStats {
            rounds: round,
            changes: total_changes,
            failed_rounds,
            exit,
        },
    )
}

fn nfp_left_eligible(config: &PolishConfig, enough_budget: bool, grouped: bool) -> bool {
    config.slide.enabled && config.nfp_left.enabled && enough_budget && !grouped
}

#[derive(Debug, PartialEq, Eq)]
struct RoundState {
    width: u32,
    container: [u32; 4],
    // Preserve actual traversal and placement identity, including duplicate ids.
    placements: Vec<(jagua_rs::entities::PItemKey, usize, [u32; 3], [u32; 4])>,
}

fn bbox_bits(bbox: jagua_rs::geometry::primitives::Rect) -> [u32; 4] {
    [
        bbox.x_min.to_bits(),
        bbox.y_min.to_bits(),
        bbox.x_max.to_bits(),
        bbox.y_max.to_bits(),
    ]
}

fn round_state(sep: &Separator) -> RoundState {
    RoundState {
        width: sep.prob.strip_width().to_bits(),
        container: bbox_bits(sep.prob.layout().container().outer_cd().bbox()),
        placements: sep
            .prob
            .layout()
            .placed_items()
            .iter()
            .map(|(key, p)| {
                let (x, y) = p.d_transf().translation();
                (
                    key,
                    p.item().idx(),
                    [p.d_transf().rotation().to_bits(), x.to_bits(), y.to_bits()],
                    bbox_bits(p.shape().bbox()),
                )
            })
            .collect(),
    }
}

/// Retry actual enabled passes, but avoid an unbounded busy loop on tiny/frozen
/// layouts. The pause is at most one millisecond and never starts after stop.
fn pace_stalled_round(started: Instant, term: &impl Terminator) {
    if term.kill() {
        return;
    }
    #[cfg(not(target_arch = "wasm32"))]
    if let Some(mut pause) = Duration::from_millis(1).checked_sub(started.elapsed()) {
        if let Some(deadline) = term.timeout_at() {
            pause = pause.min(deadline.saturating_duration_since(Instant::now()));
        }
        std::thread::sleep(pause);
    }
    #[cfg(target_arch = "wasm32")]
    let _ = started;
}

/// Certify a useful completed pass before spending the rest of the budget.
/// Never begin this optional scan after cancellation or without a measured reserve.
fn checkpoint_pass(
    instance: &SPInstance,
    sep: &mut Separator,
    best: &mut SPSolution,
    listener: &mut Option<&mut dyn SolutionListener>,
    term: &impl Terminator,
    cost: &mut Duration,
) {
    if term.kill() || sep.prob.strip_width() >= best.strip_width() || sep.ct.get_total_loss() != 0.0
    {
        return;
    }
    if term.timeout_at().is_some_and(|deadline| {
        deadline.saturating_duration_since(Instant::now()) < cost.saturating_mul(2)
    }) {
        return;
    }
    let started = Instant::now();
    if !sep.repair_original_overlap(term) || term.kill() {
        return;
    }
    let candidate = sep.prob.save();
    if candidate.strip_width() < best.strip_width() && sep.solution_is_certified(&candidate, best) {
        *best = candidate;
        if let Some(listener) = listener.as_deref_mut() {
            listener.report(ReportType::CmprFeas, best, instance);
        }
    }
    *cost = started.elapsed().max(Duration::from_millis(1));
}

fn pose_signature(sol: &SPSolution) -> Vec<(usize, u32, u32, u32)> {
    let mut poses: Vec<_> = sol
        .layout_snapshot()
        .placed_items()
        .values()
        .map(|p| {
            let (x, y) = p.d_transf().translation();
            (
                p.item().idx(),
                p.d_transf().rotation().to_bits(),
                x.to_bits(),
                y.to_bits(),
            )
        })
        .collect();
    poses.sort_unstable();
    poses
}

/// Use one secondary objective throughout a round when joint assignment is
/// available. A later pass must not undo its right-edge gain for a lower sum.
fn candidate_improves(
    candidate: &SPSolution,
    best: &SPSolution,
    _sep: &Separator,
    _config: &PolishConfig,
) -> bool {
    prefers_compact_solution(candidate, best)
}

/// Share Polish's width-first ordering with callers that retain phase checkpoints.
/// Equal-width interior progress is useful to the next slice, but never admits a wider layout.
pub fn prefers_compact_solution(candidate: &SPSolution, best: &SPSolution) -> bool {
    candidate.strip_width() < best.strip_width()
        || (candidate.strip_width() <= best.strip_width()
            && compactness(candidate) < compactness(best))
}

fn compactness(
    sol: &SPSolution,
) -> (
    ordered_float::OrderedFloat<f64>,
    ordered_float::OrderedFloat<f64>,
) {
    let mut poses: Vec<_> = sol
        .layout_snapshot()
        .placed_items()
        .values()
        .map(|p| {
            (
                p.item().idx(),
                p.shape().bbox().x_max as f64,
                p.shape().bbox().y_max as f64,
            )
        })
        .collect();
    poses.sort_by(|a, b| {
        a.0.cmp(&b.0)
            .then_with(|| a.1.total_cmp(&b.1))
            .then_with(|| a.2.total_cmp(&b.2))
    });
    (
        ordered_float::OrderedFloat(poses.iter().map(|p| p.1).sum()),
        ordered_float::OrderedFloat(poses.iter().map(|p| p.2).sum()),
    )
}

/// 부모 phase deadline과 외부 stop을 그대로 존중하면서 한 pass에 남은 시간의 일부만 허용한다.
/// 특히 `microlp::solve()`가 이 deadline을 직접 받아 장시간 simplex 호출을 제한한다.
struct FractionalTerminator<'a, T: Terminator + ?Sized> {
    parent: &'a T,
    deadline: Option<Instant>,
}

impl<'a, T: Terminator + ?Sized> FractionalTerminator<'a, T> {
    fn with_budget(parent: &'a T, budget: Duration) -> Self {
        // An unrepresentable (effectively infinite) budget falls back to the parent deadline.
        let deadline = match (Instant::now().checked_add(budget), parent.timeout_at()) {
            (Some(requested), Some(outer)) => Some(outer.min(requested)),
            (Some(requested), None) => Some(requested),
            (None, outer) => outer,
        };
        Self { parent, deadline }
    }

    fn new(parent: &'a T, fraction: f32) -> Self {
        let now = Instant::now();
        let deadline = parent
            .timeout_at()
            .map(|outer| fractional_deadline(now, outer, fraction));
        Self { parent, deadline }
    }
}

impl<T: Terminator + ?Sized> Terminator for FractionalTerminator<'_, T> {
    fn kill(&self) -> bool {
        self.parent.kill()
            || self
                .deadline
                .is_some_and(|deadline| Instant::now() >= deadline)
    }

    fn stop_requested(&self) -> bool {
        self.parent.stop_requested()
    }

    fn new_timeout(&mut self, timeout: Duration) {
        // An unrepresentable (effectively infinite) timeout keeps the current deadline.
        let local = match (self.deadline, Instant::now().checked_add(timeout)) {
            (Some(current), Some(requested)) => Some(current.min(requested)),
            (current, requested) => current.or(requested),
        };
        self.deadline = match (local, self.parent.timeout_at()) {
            (Some(local), Some(parent)) => Some(local.min(parent)),
            (Some(local), None) => Some(local),
            (None, parent) => parent,
        };
    }

    fn timeout_at(&self) -> Option<Instant> {
        match (self.deadline, self.parent.timeout_at()) {
            (Some(local), Some(parent)) => Some(local.min(parent)),
            (Some(local), None) => Some(local),
            (None, parent) => parent,
        }
    }
}

fn fractional_deadline(now: Instant, outer: Instant, fraction: f32) -> Instant {
    let remaining = outer.checked_duration_since(now).unwrap_or_default();
    now + remaining.mul_f32(fraction.clamp(0.0, 1.0))
}

#[cfg(test)]
mod tests {
    #[test]
    fn issue8_state_preserves_keys_traversal_width_and_nfp_eligibility() {
        use super::*;
        let mut keys: slotmap::SlotMap<jagua_rs::entities::PItemKey, ()> =
            slotmap::SlotMap::with_key();
        let key = keys.insert(());
        let other = keys.insert(());
        let state = |key, width| RoundState {
            width,
            container: [0; 4],
            placements: vec![(key, 0, [0; 3], [0; 4]), (other, 0, [0; 3], [0; 4])],
        };
        let a = state(key, 10f32.to_bits());
        keys.remove(key);
        let replacement = keys.insert(());
        assert_ne!(a, state(replacement, 10f32.to_bits()));
        assert_ne!(a, state(key, 11f32.to_bits()));
        let mut reversed = state(key, 10f32.to_bits());
        reversed.placements.reverse();
        assert_ne!(a, reversed);
        let mut cfg = crate::config::DEFAULT_POLISH_CONFIG;
        cfg.slide.enabled = true;
        cfg.nfp_left.enabled = true;
        assert!(nfp_left_eligible(&cfg, true, false));
        assert!(!nfp_left_eligible(&cfg, false, false));
        assert!(!nfp_left_eligible(&cfg, true, true));
        cfg.slide.enabled = false;
        assert!(!nfp_left_eligible(&cfg, true, false));
    }

    use super::fractional_deadline;
    use jagua_rs::Instant;
    use std::time::Duration;

    #[test]
    fn lp_fraction_reserves_time_for_following_passes() {
        let now = Instant::now();
        let outer = now + Duration::from_secs(10);

        let actual = fractional_deadline(now, outer, 0.60);
        let expected = now + Duration::from_secs(6);
        let rounding_error = actual
            .checked_duration_since(expected)
            .or_else(|| expected.checked_duration_since(actual))
            .unwrap_or_default();
        assert!(rounding_error < Duration::from_micros(1));
        assert_eq!(fractional_deadline(now, outer, 2.0), outer);
    }
}
