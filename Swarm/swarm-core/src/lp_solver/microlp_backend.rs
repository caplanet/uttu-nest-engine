//! 단체법(simplex) 백엔드 — 순수 Rust, 의존성이 이미 들어와 있어 **항상** 쓸 수 있다.
//!
//! 이 백엔드는 다른 모든 백엔드의 폴백 대상이다([`super::backend_for`]). 그래서
//! "쓸 수 없다"는 결과가 나오면 안 되고([`LpBackend::availability`] 는 언제나 `Ok`),
//! 어떤 입력에도 panic 하지 않아야 한다 — 폴백 대상이 죽으면 물러설 곳이 없다.
//!
//! # [`LpProblem`] → microlp 모델 변환
//!
//! microlp 는 행을 `expr {<=,>=,==} rhs` 한 방향으로만 받는다. 반면 [`LpRow`] 는
//! `lower <= expr <= upper` 양방향이다. 그래서 **양쪽이 다 유한한 행은 두 개의
//! 부등식으로 쪼개서** 넣는다. 한쪽만 유한하면 그쪽 부등식 하나만 넣고, 양쪽 다
//! 무한이면 아무 제약도 아니므로 행 자체를 버린다.
//!
//! 변수 경계는 `add_var` 가 `(min, max)` 로 그대로 받고 무한대도 허용하므로 손댈 게 없다.
//!
//! # Irrelevant free columns
//!
//! microlp 0.6 can incorrectly report a bounded problem as unbounded when a
//! second, unused free column is present. An unconstrained column with zero
//! objective is algebraically irrelevant: choose its representative value 0.
//! Only those columns are fixed. Free columns with a nonzero objective or an
//! effective constraint retain their original bounds, including true unboundedness.
//!
//! # Cancellation and deadline
//!
//! Model assembly and the simplex solve share one deadline. The simplex state
//! is resumed in 50 ms slices so a caller can cancel between slices without
//! restarting optimization. A single pivot remains indivisible; this is a soft
//! deadline, not a hard real-time guarantee. Expired or cancelled results are
//! returned as `Limit` and are not committed.
//!
//! # 2차 목적항
//!
//! 단체법은 2차항을 못 푼다([`LpBackend::supports_quadratic`] = `false`).
//! `problem.quadratic > 0` 이면 경고 한 줄을 남기고 **1차항만** 최소화한다.
//! 돌려주는 목적값도 실제로 최소화한 1차항 `c^T x` 다 — 풀지도 않은 2차항을 더해서
//! 보고하면 다른 백엔드와 비교할 때 "microlp 가 더 나쁜 해를 냈다"는 착시가 생긴다.

use jagua_rs::Instant;
use microlp::{ComparisonOp, Error as MicrolpError, LinearExpr, OptimizationDirection, Problem};
use microlp::{SolveOutcome, Variable};
use std::time::Duration;

use super::{LpBackend, LpProblem, LpRow, LpSolution, LpStatus};

/// 로그와 [`LpSolution::solved_by`] 에 그대로 나가는 이름.
const NAME: &str = "microlp(단체법)";

/// microlp 단체법 백엔드. 상태가 없어 필드도 없다 — 매 호출이 독립적이다.
pub struct MicrolpBackend;

impl MicrolpBackend {
    /// [`LpProblem`] 을 microlp 모델로 옮긴다.
    ///
    /// 생성한 변수를 만든 순서 그대로 돌려준다. 해를 다시 읽을 때 이 순서가
    /// [`LpProblem::objective`] 의 인덱스와 1:1 로 대응한다.
    ///
    /// `problem` 은 이미 [`LpProblem::validate`] 를 통과했다고 가정한다.
    fn build(
        problem: &LpProblem,
        should_stop: &dyn Fn() -> bool,
    ) -> Option<(Problem, Vec<Variable>)> {
        let mut lp = Problem::new(OptimizationDirection::Minimize);
        let n = problem.n_vars();

        // Ordinary compaction models have finite trust bounds, so avoid an
        // additional allocation/pass unless an irrelevant free column is possible.
        let used_columns = if (0..n).any(|i| {
            problem.objective[i] == 0.0
                && problem.lower[i] == f64::NEG_INFINITY
                && problem.upper[i] == f64::INFINITY
        }) {
            let mut used = vec![false; n];
            for row in &problem.rows {
                if should_stop() {
                    return None;
                }
                if row.lower.is_finite() || row.upper.is_finite() {
                    for (&col, &value) in row.cols.iter().zip(&row.vals) {
                        if value != 0.0 {
                            used[col] = true;
                        }
                    }
                }
            }
            Some(used)
        } else {
            None
        };

        let mut vars = Vec::with_capacity(n);
        for i in 0..n {
            if i % 64 == 0 && should_stop() {
                return None;
            }
            // validate() 가 길이를 보장하지만, 인덱싱 대신 get 을 써서 어떤 경우에도
            // panic 하지 않게 한다. 폴백 대상은 죽으면 안 된다.
            let c = problem.objective.get(i).copied().unwrap_or(0.0);
            let lo = problem.lower.get(i).copied().unwrap_or(f64::NEG_INFINITY);
            let hi = problem.upper.get(i).copied().unwrap_or(f64::INFINITY);
            let irrelevant_free = c == 0.0
                && lo == f64::NEG_INFINITY
                && hi == f64::INFINITY
                && used_columns.as_ref().is_some_and(|used| !used[i]);
            vars.push(lp.add_var(
                c,
                if irrelevant_free {
                    (0.0, 0.0)
                } else {
                    (lo, hi)
                },
            ));
        }

        for row in problem.rows.iter() {
            if should_stop() {
                return None;
            }
            let terms = coalesced_terms(row, &vars);
            if terms.is_empty() {
                continue;
            }
            let has_lower = row.lower.is_finite();
            let has_upper = row.upper.is_finite();
            // 양쪽이 다 유한하면 두 부등식으로 쪼갠다. 등식(lower == upper)도 같은
            // 처리로 충분하다 — Ge/Le 쌍은 단체법에서 등식과 같은 다면체를 만든다.
            if has_lower {
                lp.add_constraint(expr_from(&terms), ComparisonOp::Ge, row.lower);
            }
            if has_upper {
                lp.add_constraint(expr_from(&terms), ComparisonOp::Le, row.upper);
            }
            // 양쪽 다 무한이면 제약이 아니다 — 넣지 않는다.
        }

        Some((lp, vars))
    }
}

/// 한 행의 `(변수, 계수)` 목록. 같은 열이 두 번 나오면 계수를 합쳐 하나로 만든다.
///
/// microlp 의 `add_constraint` 는 **같은 변수가 두 번 나오면 panic 한다.**
/// Public validation rejects duplicate columns; coalescing remains a defensive
/// safeguard for this private conversion helper.
fn coalesced_terms(row: &LpRow, vars: &[Variable]) -> Vec<(Variable, f64)> {
    let mut terms: Vec<(Variable, f64)> = Vec::with_capacity(row.cols.len());
    for (idx, &col) in row.cols.iter().enumerate() {
        let Some(&var) = vars.get(col) else { continue };
        let Some(&val) = row.vals.get(idx) else {
            continue;
        };
        match terms.iter_mut().find(|(v, _)| v.idx() == var.idx()) {
            Some((_, acc)) => *acc += val,
            None => terms.push((var, val)),
        }
    }
    terms
}

/// `(변수, 계수)` 목록을 microlp 표현식으로. 두 부등식에 같은 좌변을 넣어야 해서
/// 소비하지 않고 매번 새로 만든다.
fn expr_from(terms: &[(Variable, f64)]) -> LinearExpr {
    let mut e = LinearExpr::empty();
    for &(var, coeff) in terms {
        e.add(var, coeff);
    }
    e
}

impl LpBackend for MicrolpBackend {
    fn name(&self) -> &'static str {
        NAME
    }

    /// 빌드에 항상 포함되므로 실패할 수 없다.
    fn availability(&self) -> Result<(), String> {
        Ok(())
    }

    /// 단체법은 2차 목적항을 풀지 못한다.
    fn supports_quadratic(&self) -> bool {
        false
    }

    fn solve(&self, problem: &LpProblem, time_limit: Duration) -> LpSolution {
        self.solve_with_control(problem, time_limit, &|| false)
    }

    fn solve_with_control(
        &self,
        problem: &LpProblem,
        time_limit: Duration,
        should_stop: &dyn Fn() -> bool,
    ) -> LpSolution {
        let limited = || LpSolution {
            status: LpStatus::Limit,
            x: Vec::new(),
            objective: f64::NAN,
            solved_by: NAME,
        };
        if should_stop() {
            return limited();
        }
        if let Err(reason) = problem.validate() {
            log::warn!("[LP] {NAME}: 모델이 유효하지 않다 ({reason})");
            return LpSolution::failed(NAME);
        }
        if problem.quadratic > 0.0 {
            log::warn!(
                "[LP] {NAME}: 2차 정규화 {}을 풀 수 없다 — 1차항만 최소화한다",
                problem.quadratic
            );
        }

        let started = Instant::now();
        let stopped =
            || should_stop() || (!time_limit.is_zero() && started.elapsed() >= time_limit);
        let Some((mut lp, vars)) = Self::build(problem, &stopped) else {
            return limited();
        };
        let slice = || {
            let quantum = Duration::from_millis(50);
            if time_limit.is_zero() {
                quantum
            } else {
                quantum.min(time_limit.saturating_sub(started.elapsed()))
            }
        };
        if stopped() {
            return limited();
        }
        lp.set_time_limit(slice());
        let mut outcome = lp.solve();
        // Resume the same simplex state instead of restarting the model. A single
        // pivot remains indivisible, but no full multi-second solve hides stop.
        loop {
            if stopped() {
                return limited();
            }
            outcome = match outcome {
                Ok(interrupted @ SolveOutcome::Interrupted(_)) => {
                    let mut options = microlp::ResumeOptions::default();
                    options.time_limit = Some(slice());
                    interrupted.resume_with(options)
                }
                done => {
                    outcome = done;
                    break;
                }
            };
        }

        match outcome {
            Ok(SolveOutcome::Solution(sol)) => {
                // 변수는 만든 순서대로 나온다 — 그게 곧 열 인덱스다.
                let mut x: Vec<f64> = sol.iter().map(|(_, v)| v).collect();
                x.truncate(vars.len());
                let objective = sol.objective();
                LpSolution {
                    status: LpStatus::Optimal,
                    x,
                    objective,
                    solved_by: NAME,
                }
                .checked(problem)
            }
            Ok(SolveOutcome::Interrupted(_)) => {
                // 제한에 걸려 채택 가능한 해가 아예 없는 경우. x 는 비운다.
                LpSolution {
                    status: LpStatus::Limit,
                    x: Vec::new(),
                    objective: f64::NAN,
                    solved_by: NAME,
                }
            }
            Err(MicrolpError::Infeasible) => LpSolution {
                status: LpStatus::Infeasible,
                x: Vec::new(),
                objective: f64::NAN,
                solved_by: NAME,
            },
            Err(MicrolpError::Unbounded) => LpSolution {
                status: LpStatus::Unbounded,
                x: Vec::new(),
                objective: f64::NAN,
                solved_by: NAME,
            },
            Err(other) => {
                log::warn!("[LP] {NAME}: 내부 오류 ({other})");
                LpSolution::failed(NAME)
            }
        }
    }
}

/// 예산을 넘겼는가. 예산이 0이면 제한을 걸지 않은 것이므로 언제나 `false`.
#[cfg(test)]
fn over_budget(elapsed: Duration, limit: Duration) -> bool {
    !limit.is_zero() && elapsed > limit
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lp_solver::LpRow;

    /// 넉넉한 예산 — 시간 제한 경로가 테스트에 끼어들지 않게 한다.
    fn budget() -> Duration {
        Duration::from_secs(30)
    }

    /// `min x0 + x1`, `x0 + x1 >= 2`, `0 <= x <= 10`.
    fn tiny_lp() -> LpProblem {
        LpProblem {
            objective: vec![1.0, 1.0],
            quadratic: 0.0,
            lower: vec![0.0, 0.0],
            upper: vec![10.0, 10.0],
            rows: vec![LpRow::greater_or_equal(vec![0, 1], vec![1.0, 1.0], 2.0)],
        }
    }

    #[test]
    fn the_simplex_backend_is_always_available_and_never_claims_quadratic_support() {
        assert!(MicrolpBackend.availability().is_ok());
        assert!(!MicrolpBackend.supports_quadratic());
        assert_eq!(MicrolpBackend.name(), NAME);
    }

    #[test]
    fn a_tiny_lp_reaches_the_known_optimum_of_two() {
        let sol = MicrolpBackend.solve(&tiny_lp(), budget());
        assert_eq!(sol.status, LpStatus::Optimal);
        assert!(
            (sol.objective - 2.0).abs() < 1.0e-6,
            "목적값 {} != 2",
            sol.objective
        );
        assert_eq!(sol.x.len(), 2);
        let sum: f64 = sol.x.iter().sum();
        assert!((sum - 2.0).abs() < 1.0e-6, "x 합 {sum} != 2");
    }

    /// 한 행에 `1 <= x0 <= 3` 을 통째로 넣었을 때, 목적의 방향에 따라 **양쪽 경계가
    /// 모두** 걸려야 한다. 한쪽만 넣으면 둘 중 하나가 반드시 틀린다.
    #[test]
    fn a_row_with_both_bounds_pins_the_variable_from_each_side() {
        let two_sided = LpRow {
            cols: vec![0],
            vals: vec![1.0],
            lower: 1.0,
            upper: 3.0,
        };

        // min x0  ->  행 하한 1 에 붙는다 (변수 하한 0 이 아니라).
        let mut down = tiny_lp();
        down.objective = vec![1.0, 0.0];
        down.rows = vec![two_sided.clone()];
        let sol = MicrolpBackend.solve(&down, budget());
        assert_eq!(sol.status, LpStatus::Optimal);
        assert!(
            (sol.x.first().copied().unwrap_or(f64::NAN) - 1.0).abs() < 1.0e-6,
            "x0 {:?} != 1",
            sol.x.first()
        );

        // min -x0  ->  행 상한 3 에 붙는다 (변수 상한 10 이 아니라).
        let mut up = tiny_lp();
        up.objective = vec![-1.0, 0.0];
        up.rows = vec![two_sided];
        let sol = MicrolpBackend.solve(&up, budget());
        assert_eq!(sol.status, LpStatus::Optimal);
        assert!(
            (sol.x.first().copied().unwrap_or(f64::NAN) - 3.0).abs() < 1.0e-6,
            "x0 {:?} != 3",
            sol.x.first()
        );
    }

    #[test]
    fn contradictory_rows_are_reported_as_infeasible() {
        let mut p = tiny_lp();
        // 변수 경계로 넣으면 validate() 가 먼저 거절하므로 **행**으로 넣는다.
        p.rows = vec![
            LpRow::greater_or_equal(vec![0], vec![1.0], 5.0),
            LpRow::less_or_equal(vec![0], vec![1.0], 1.0),
        ];
        let sol = MicrolpBackend.solve(&p, budget());
        assert_eq!(sol.status, LpStatus::Infeasible);
        assert!(sol.x.is_empty());
    }

    #[test]
    fn an_unbounded_objective_is_reported_as_unbounded() {
        let p = LpProblem {
            objective: vec![-1.0],
            quadratic: 0.0,
            lower: vec![0.0],
            upper: vec![f64::INFINITY],
            rows: vec![LpRow::greater_or_equal(vec![0], vec![1.0], 0.0)],
        };
        assert_eq!(
            MicrolpBackend.solve(&p, budget()).status,
            LpStatus::Unbounded
        );
    }

    #[test]
    fn a_problem_that_fails_validation_returns_failed_without_panicking() {
        let mut broken = tiny_lp();
        broken.objective.clear(); // 변수 0개
        let sol = MicrolpBackend.solve(&broken, budget());
        assert_eq!(sol.status, LpStatus::Failed);
        assert!(sol.x.is_empty());

        let mut ragged = tiny_lp();
        ragged.rows[0].vals.pop(); // cols/vals 길이 불일치
        assert_eq!(
            MicrolpBackend.solve(&ragged, budget()).status,
            LpStatus::Failed
        );

        let mut flipped = tiny_lp();
        flipped.lower[0] = 5.0;
        flipped.upper[0] = 1.0;
        assert_eq!(
            MicrolpBackend.solve(&flipped, budget()).status,
            LpStatus::Failed
        );
    }

    /// 중복 열은 **계약 층에서 거절된다** — [`LpProblem::validate`] 참조.
    ///
    /// microlp 의 `add_constraint` 는 내부에서 `CsVec::new_from_unsorted(..).unwrap()`
    /// 을 부르므로 중복 인덱스에 panic 하고, CSC 로 조립하는 백엔드에서는 좌표가
    /// 중복된 채 조용히 틀린 행렬이 된다. 백엔드마다 합쳐서 막을 수도 있지만 그러면
    /// 새 백엔드를 추가하는 사람이 반드시 같은 함정을 다시 밟으므로, 한 곳에서 막는다.
    ///
    /// 이 백엔드의 `coalesced_terms` 는 그래서 **방어선이 하나 더 있는 것**이지
    /// 정상 경로가 아니다. `solve` 를 직접 부르는 호출자가 생겨도 panic 하지 않는다.
    #[test]
    fn duplicate_columns_are_rejected_by_the_contract_not_silently_merged() {
        let mut p = tiny_lp();
        p.objective = vec![1.0, 0.0];
        p.rows = vec![LpRow::greater_or_equal(vec![0, 0], vec![1.0, 1.0], 2.0)];

        assert!(p.validate().is_err(), "중복 열은 계약 층에서 걸러야 한다");
        // 그래도 panic 하지 않고 Failed 로 돌아온다.
        assert_eq!(MicrolpBackend.solve(&p, budget()).status, LpStatus::Failed);
    }

    /// 등식 행(`lower == upper`)은 Ge/Le 두 부등식으로 쪼개 넣는다. 모듈 문서가
    /// "그 쌍이 등식과 같은 다면체를 만든다"고 주장하므로, 목적을 양방향으로 걸어
    /// **어느 쪽에서도 빠져나가지 못하는지** 확인한다. 한쪽만 들어갔다면 반대 방향
    /// 목적에서 변수 경계(0 또는 10)까지 흘러가 버린다.
    #[test]
    fn an_equality_row_pins_the_variable_from_both_directions() {
        let equality = LpRow {
            cols: vec![0],
            vals: vec![1.0],
            lower: 2.0,
            upper: 2.0,
        };
        for objective in [vec![1.0, 0.0], vec![-1.0, 0.0]] {
            let mut p = tiny_lp();
            p.objective = objective.clone();
            p.rows = vec![equality.clone()];
            let sol = MicrolpBackend.solve(&p, budget());
            assert_eq!(sol.status, LpStatus::Optimal, "목적 {objective:?}");
            let x0 = sol.x.first().copied().unwrap_or(f64::NAN);
            assert!(
                (x0 - 2.0).abs() < 1.0e-6,
                "목적 {objective:?} 에서 x0 {x0} != 2"
            );
        }
    }

    /// 양쪽이 다 무한한 행은 아무것도 제약하지 않으므로 모델에서 빠져야 한다.
    /// 빠지지 않으면 microlp 에 `-inf <= expr` 같은 행이 들어가 답이 흔들린다.
    /// 행이 빠졌는지는 밖에서 셀 수 없으므로 **답으로** 확인한다 — 그 행이 살아
    /// 있었다면 x0 는 행 하한 2 가 아니라 다른 값에 붙는다.
    #[test]
    fn a_row_with_no_finite_bound_is_dropped_from_the_model() {
        let unconstraining = LpRow {
            cols: vec![0, 1],
            vals: vec![1.0, 1.0],
            lower: f64::NEG_INFINITY,
            upper: f64::INFINITY,
        };
        let mut p = tiny_lp();
        p.objective = vec![1.0, 0.0];
        p.rows = vec![
            unconstraining,
            LpRow::greater_or_equal(vec![0], vec![1.0], 2.0),
        ];
        let sol = MicrolpBackend.solve(&p, budget());
        assert_eq!(sol.status, LpStatus::Optimal);
        let x0 = sol.x.first().copied().unwrap_or(f64::NAN);
        assert!((x0 - 2.0).abs() < 1.0e-6, "x0 {x0} != 2");
    }

    /// 자유 변수(양쪽 무한 경계)는 `add_var` 에 무한대를 그대로 넘긴다.
    /// **행에 등장하는** 자유 변수는 정상적으로 풀린다 — 여기까지가 보장 범위다.
    /// Unused zero-cost free columns are normalized independently (module docs).
    #[test]
    fn a_free_variable_that_appears_in_a_row_is_solved_normally() {
        let p = LpProblem {
            objective: vec![1.0],
            quadratic: 0.0,
            lower: vec![f64::NEG_INFINITY],
            upper: vec![f64::INFINITY],
            rows: vec![LpRow::greater_or_equal(vec![0], vec![1.0], -3.0)],
        };
        let sol = MicrolpBackend.solve(&p, budget());
        assert_eq!(sol.status, LpStatus::Optimal);
        let x0 = sol.x.first().copied().unwrap_or(f64::NAN);
        assert!((x0 + 3.0).abs() < 1.0e-6, "x0 {x0} != -3");
    }

    /// 2차항은 못 풀지만 **무시하고 LP 로 푼다** — 실패하거나 panic 하지 않는다.
    #[test]
    fn a_quadratic_objective_is_ignored_rather_than_rejected() {
        let mut qp = tiny_lp();
        qp.quadratic = 1.0;
        let sol = MicrolpBackend.solve(&qp, budget());
        assert_eq!(sol.status, LpStatus::Optimal);
        assert!((sol.objective - 2.0).abs() < 1.0e-6, "1차항만 최소화한다");
    }

    /// Unlimited/ample budgets retain optimal solutions; expired budgets return no candidate.
    #[test]
    fn budgets_distinguish_optimal_results_from_expired_attempts() {
        let problem = tiny_lp();

        // 1) 넉넉한 예산 — 정상 해.
        let sol = MicrolpBackend.solve(&problem, Duration::from_secs(60));
        assert_eq!(sol.status, LpStatus::Optimal);
        assert!((sol.objective - 2.0).abs() < 1.0e-6);

        // 2) 제한 없음(0) — 예산 판정 자체가 발동하지 않는다.
        let unlimited = MicrolpBackend.solve(&problem, Duration::ZERO);
        assert_eq!(unlimited.status, LpStatus::Optimal);
        assert!(!over_budget(Duration::from_secs(3600), Duration::ZERO));

        // A deadline may expire during assembly or solving. Neither commits a candidate.
        let tight = MicrolpBackend.solve(&problem, Duration::from_nanos(1));
        if tight.status == LpStatus::Limit {
            assert!(
                tight.x.is_empty(),
                "expired attempts must not expose a candidate"
            );
        }
    }

    #[test]
    fn over_budget_only_triggers_when_a_limit_was_actually_set() {
        assert!(!over_budget(Duration::from_secs(10), Duration::ZERO));
        assert!(over_budget(
            Duration::from_millis(2),
            Duration::from_millis(1)
        ));
        assert!(!over_budget(
            Duration::from_millis(1),
            Duration::from_millis(2)
        ));
    }
    #[test]
    fn cancellation_during_model_assembly_returns_no_candidate() {
        let count = std::cell::Cell::new(0usize);
        let mut problem = tiny_lp();
        problem
            .rows
            .extend((0..100).map(|_| LpRow::greater_or_equal(vec![0], vec![1.0], 0.0)));
        let sol = MicrolpBackend.solve_with_control(&problem, Duration::ZERO, &|| {
            count.set(count.get() + 1);
            count.get() >= 8
        });
        assert_eq!(sol.status, LpStatus::Limit);
        assert!(sol.x.is_empty());
        assert_eq!(count.get(), 8);
    }
}
