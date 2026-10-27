//! 내점법(interior point) 백엔드 — 순수 Rust, **QP 를 푼다**.
//!
//! 압축 모델에 2차 정규화 `(mu/2)·sum(d²)` 를 얹으려면 이 백엔드이거나 Xpress 여야
//! 한다. 단체법은 꼭짓점만 훑으므로 2차 목적을 표현할 방법이 없다.
//!
//! # 원뿔 형식으로의 변환
//!
//! clarabel 이 받는 표준형은
//!
//! ```text
//! minimize    (1/2) x' P x + q' x
//! s.t.        A x + s = b,   s in K
//! ```
//!
//! 이고, [`LpProblem`] 은 `lower <= a'x <= upper` 와 `lower <= x <= upper` 다.
//! 부등식 하나하나를 **비음 원뿔**(`s >= 0`) 한 행으로 바꾼다.
//!
//! ```text
//!  a'x <= u   ->    a'x + s =  u,  s >= 0
//!  a'x >= l   ->   -a'x + s = -l,  s >= 0     (부호를 뒤집어 <= 로 만든다)
//! ```
//!
//! 변수 경계도 `a = e_i` 인 단위 행으로 같게 처리한다. 등식 행(`lower == upper`)은
//! [`clarabel::solver::ZeroConeT`] 로 접을 수도 있지만, 그러면 원뿔 순서를 지키느라
//! 행 배열을 두 번 훑어야 한다. 두 개의 비음 행으로 넣어도 내점법이 같은 해를 내고
//! 코드가 한 겹 얇아지므로 **모든 행을 하나의 `NonnegativeConeT` 로 통일**했다.
//!
//! Only actual infinities mean absent bounds. Finite bounds outside Clarabel's
//! representable range are rejected; silently dropping them changes the LP.
//!
//! # 희소행렬은 열 우선이다
//!
//! [`CscMatrix`] 는 CSC(Compressed Sparse **Column**) 다. `colptr[j]..colptr[j+1]`
//! 구간이 `j` 번째 **열**의 원소이고, 그 구간 안의 `rowval` 은 **오름차순으로 정렬**되어
//! 있어야 한다. 그래서 행을 순서대로 만들면서 각 원소를 자기 열의 버킷에 밀어 넣는다 —
//! 행 인덱스가 단조 증가하므로 버킷은 저절로 정렬된 상태가 된다.
//!
//! # 시간 제한
//!
//! [`clarabel::solver::DefaultSettings::time_limit`] 은 초 단위 `f64` 다.
//! 예산이 0이거나 유한하지 않으면 무한대로 둔다 — 0을 그대로 넘기면 어떤 문제든
//! 첫 반복 전에 `MaxTime` 이 되어 항상 [`LpStatus::Limit`] 이 나온다.

use clarabel::algebra::CscMatrix;
use clarabel::solver::{
    DefaultSettings, DefaultSettingsBuilder, DefaultSolver, IPSolver, NonnegativeConeT,
    SolverStatus, SupportedConeT,
};
use std::time::Duration;

use super::{LpBackend, LpProblem, LpSolution, LpStatus};

/// 로그와 [`LpSolution::solved_by`] 에 그대로 나가는 이름.
const NAME: &str = "clarabel(내점법)";

/// 이 크기 이상의 경계는 "없음"으로 본다.
///
/// clarabel 자신이 `1e20` 을 무한대로 쓴다. 그보다 작은 값을 쓰면 진짜 유효한
/// 경계를 버리게 되고, 크게 쓰면 무의미한 행이 KKT 행렬에 남는다.
const INFINITY_THRESHOLD: f64 = 1.0e20;

/// clarabel 내점법 백엔드. 상태가 없어 필드도 없다 — 매 호출이 독립적이다.
pub struct ClarabelBackend;

/// 변환 중간 표현: `sum(coeffs) <= rhs` 한 행.
///
/// 열 인덱스와 계수를 짝으로 들고 있다가 마지막에 열 우선으로 재배열한다.
struct ConicRow {
    /// `(열 인덱스, 계수)`. 같은 열이 두 번 나오지 않도록 미리 합쳐 둔다.
    terms: Vec<(usize, f64)>,
    /// 우변 `b`.
    rhs: f64,
}

/// clarabel 에 넘길 `(A, b)`.
struct ConicForm {
    a: CscMatrix<f64>,
    b: Vec<f64>,
}

impl ClarabelBackend {
    /// [`LpProblem`] 의 모든 부등식을 `Ax + s = b, s >= 0` 형태로 모은다.
    ///
    /// `problem` 은 이미 [`LpProblem::validate`] 를 통과했다고 가정한다.
    /// 실패할 수 있는 경우는 CSC 조립뿐이며, 그때는 `None` 을 돌려 호출부가
    /// [`LpStatus::Failed`] 로 물러나게 한다.
    fn conic_form(problem: &LpProblem) -> Option<ConicForm> {
        let n = problem.n_vars();
        let mut rows: Vec<ConicRow> = Vec::new();

        // 1. 제약 행. 상한이 있으면 그대로, 하한이 있으면 부호를 뒤집어 <= 로.
        for row in problem.rows.iter() {
            let terms = coalesced_terms(&row.cols, &row.vals, n);
            if terms.is_empty() {
                continue;
            }
            if is_finite_bound(row.upper) {
                rows.push(ConicRow {
                    terms: terms.clone(),
                    rhs: row.upper,
                });
            }
            if is_finite_bound(row.lower) {
                rows.push(ConicRow {
                    terms: terms.iter().map(|&(c, v)| (c, -v)).collect(),
                    rhs: -row.lower,
                });
            }
        }

        // 2. 변수 경계. 단위 행 하나씩.
        for i in 0..n {
            if let Some(&hi) = problem.upper.get(i)
                && is_finite_bound(hi)
            {
                rows.push(ConicRow {
                    terms: vec![(i, 1.0)],
                    rhs: hi,
                });
            }
            if let Some(&lo) = problem.lower.get(i)
                && is_finite_bound(lo)
            {
                rows.push(ConicRow {
                    terms: vec![(i, -1.0)],
                    rhs: -lo,
                });
            }
        }

        let a = assemble_csc(&rows, n)?;
        let b = rows.iter().map(|r| r.rhs).collect();
        Some(ConicForm { a, b })
    }

    /// 목적의 2차항 `P = mu * I`. `mu == 0` 이면 빈 행렬(순수 LP)이다.
    ///
    /// clarabel 은 `P` 의 상삼각만 참조한다. 대각 행렬이라 그 구분이 의미가 없다.
    fn quadratic_matrix(mu: f64, n: usize) -> CscMatrix<f64> {
        if !(mu.is_finite() && mu > 0.0) {
            return CscMatrix::<f64>::zeros((n, n));
        }
        // 대각 원소가 열마다 정확히 하나 — colptr 은 0,1,2,...,n 이고 rowval 은 열 번호다.
        CscMatrix::new(n, n, (0..=n).collect(), (0..n).collect(), vec![mu; n])
    }
}

/// `(열, 계수)` 목록에서 중복 열을 합치고 범위 밖 열은 버린다.
///
/// CSC 는 같은 (행, 열) 좌표가 두 번 나오는 것을 허용하지 않는다. 중복이 남으면
/// clarabel 의 KKT 조립이 조용히 틀린 답을 내거나 내부에서 panic 한다.
fn coalesced_terms(cols: &[usize], vals: &[f64], n: usize) -> Vec<(usize, f64)> {
    let mut terms: Vec<(usize, f64)> = Vec::with_capacity(cols.len());
    for (idx, &col) in cols.iter().enumerate() {
        if col >= n {
            continue;
        }
        let Some(&val) = vals.get(idx) else { continue };
        match terms.iter_mut().find(|(c, _)| *c == col) {
            Some((_, acc)) => *acc += val,
            None => terms.push((col, val)),
        }
    }
    terms
}

/// 유한하고 clarabel 이 무한으로 보지 않을 만큼 작은 경계인가.
fn is_finite_bound(v: f64) -> bool {
    v.is_finite() && v.abs() < INFINITY_THRESHOLD
}

/// 행 목록을 **열 우선** CSC 로 조립한다.
///
/// 행을 인덱스 오름차순으로 훑으며 각 원소를 자기 열 버킷에 밀어 넣으므로,
/// 버킷 안의 행 인덱스는 저절로 오름차순이 된다 — CSC 가 요구하는 정렬 조건이다.
///
/// 조립 결과가 CSC 불변식을 깨면 `None`. [`CscMatrix::new`] 는 `assert!` 로
/// 검사하므로 넘기기 전에 우리가 먼저 확인해야 panic 을 피한다.
fn assemble_csc(rows: &[ConicRow], n: usize) -> Option<CscMatrix<f64>> {
    let m = rows.len();
    let mut buckets: Vec<Vec<(usize, f64)>> = vec![Vec::new(); n];
    for (r, row) in rows.iter().enumerate() {
        for &(col, val) in row.terms.iter() {
            buckets.get_mut(col)?.push((r, val));
        }
    }

    let nnz: usize = buckets.iter().map(|b| b.len()).sum();
    let mut colptr = Vec::with_capacity(n + 1);
    let mut rowval = Vec::with_capacity(nnz);
    let mut nzval = Vec::with_capacity(nnz);
    colptr.push(0usize);
    for bucket in buckets.iter() {
        for &(r, v) in bucket.iter() {
            rowval.push(r);
            nzval.push(v);
        }
        colptr.push(rowval.len());
    }

    // CscMatrix::new 의 assert 와 같은 조건을 먼저 확인한다.
    if colptr.len() != n + 1 || colptr.last() != Some(&rowval.len()) || rowval.len() != nzval.len()
    {
        log::warn!("[LP] {NAME}: CSC 조립이 불변식을 깼다 (m {m}, n {n}, nnz {nnz})");
        return None;
    }
    Some(CscMatrix::new(m, n, colptr, rowval, nzval))
}

/// 예산을 clarabel 설정으로 옮긴다. 0이거나 비정상이면 무한대(제한 없음).
fn settings_for(time_limit: Duration) -> Option<DefaultSettings<f64>> {
    let secs = time_limit.as_secs_f64();
    let limit = if secs.is_finite() && secs > 0.0 {
        secs
    } else {
        f64::INFINITY
    };
    match DefaultSettingsBuilder::<f64>::default()
        .verbose(false)
        .time_limit(limit)
        .build()
    {
        Ok(s) => Some(s),
        Err(e) => {
            log::warn!("[LP] {NAME}: 설정을 만들지 못했다 ({e})");
            None
        }
    }
}

/// clarabel 의 종료 상태를 계약상의 [`LpStatus`] 로.
///
/// `Almost*` 계열은 **정확도가 떨어진 해**다. [`LpStatus::is_usable`] 이 `Optimal`
/// 만 채택하는 이상 그걸 `Optimal` 로 올려 보내면 검증 비용만 늘어나므로
/// [`LpStatus::Failed`] 로 둔다 — 호출부는 압축 스텝을 그냥 건너뛴다.
fn map_status(status: SolverStatus) -> LpStatus {
    match status {
        SolverStatus::Solved => LpStatus::Optimal,
        SolverStatus::PrimalInfeasible => LpStatus::Infeasible,
        SolverStatus::DualInfeasible => LpStatus::Unbounded,
        // CallbackTerminated: a termination callback stopped the solve early.
        // None is registered today, but it is a limit, not a solver error.
        SolverStatus::MaxIterations
        | SolverStatus::MaxTime
        | SolverStatus::CallbackTerminated => LpStatus::Limit,
        _ => LpStatus::Failed,
    }
}

impl LpBackend for ClarabelBackend {
    fn name(&self) -> &'static str {
        NAME
    }

    /// 빌드에 항상 포함되므로 실패할 수 없다.
    fn availability(&self) -> Result<(), String> {
        Ok(())
    }

    /// 내점법은 2차 목적항을 그대로 푼다 — 에피그래프 재정식화가 필요 없다.
    fn supports_quadratic(&self) -> bool {
        true
    }

    fn solve(&self, problem: &LpProblem, time_limit: Duration) -> LpSolution {
        if let Err(reason) = problem.validate() {
            log::warn!("[LP] {NAME}: 모델이 유효하지 않다 ({reason})");
            return LpSolution::failed(NAME);
        }

        if problem
            .lower
            .iter()
            .chain(&problem.upper)
            .chain(problem.rows.iter().flat_map(|r| [&r.lower, &r.upper]))
            .any(|v| v.is_finite() && v.abs() >= INFINITY_THRESHOLD)
        {
            log::warn!("[LP] {NAME}: finite bound exceeds the backend numeric range");
            return LpSolution::failed(NAME);
        }

        let n = problem.n_vars();
        let Some(form) = Self::conic_form(problem) else {
            return LpSolution::failed(NAME);
        };
        let Some(settings) = settings_for(time_limit) else {
            return LpSolution::failed(NAME);
        };

        let p = Self::quadratic_matrix(problem.quadratic, n);
        let cones: [SupportedConeT<f64>; 1] = [NonnegativeConeT(form.b.len())];

        let mut solver =
            match DefaultSolver::new(&p, &problem.objective, &form.a, &form.b, &cones, settings) {
                Ok(s) => s,
                Err(e) => {
                    log::warn!("[LP] {NAME}: 솔버를 만들지 못했다 ({e:?})");
                    return LpSolution::failed(NAME);
                }
            };
        solver.solve();

        let status = map_status(solver.solution.status);
        // 실행 불가/무계에서 clarabel 이 돌려주는 x 는 해가 아니라 **증명서**다.
        // 그걸 해로 착각해 적용하는 일이 없도록 비워서 돌린다.
        let keep_x = matches!(status, LpStatus::Optimal | LpStatus::Limit);
        let mut x = if keep_x {
            solver.solution.x.clone()
        } else {
            Vec::new()
        };
        x.truncate(n);
        let objective = if keep_x {
            solver.solution.obj_val
        } else {
            f64::NAN
        };

        LpSolution {
            status,
            x,
            objective,
            solved_by: NAME,
        }
        .checked(problem)
    }
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
    fn the_interior_point_backend_is_always_available_and_solves_quadratics() {
        assert!(ClarabelBackend.availability().is_ok());
        assert!(ClarabelBackend.supports_quadratic());
        assert_eq!(ClarabelBackend.name(), NAME);
    }

    #[test]
    fn a_tiny_lp_reaches_the_known_optimum_of_two() {
        let sol = ClarabelBackend.solve(&tiny_lp(), budget());
        assert_eq!(sol.status, LpStatus::Optimal);
        assert!(
            (sol.objective - 2.0).abs() < 1.0e-4,
            "목적값 {} != 2",
            sol.objective
        );
        assert_eq!(sol.x.len(), 2);
    }

    /// 한 행에 `1 <= x0 <= 3` 을 통째로 넣었을 때, 목적의 방향에 따라 **양쪽 경계가
    /// 모두** 걸려야 한다. 부호를 뒤집는 하한 변환이 틀리면 여기서 잡힌다.
    #[test]
    fn a_row_with_both_bounds_pins_the_variable_from_each_side() {
        let two_sided = LpRow {
            cols: vec![0],
            vals: vec![1.0],
            lower: 1.0,
            upper: 3.0,
        };

        let mut down = tiny_lp();
        down.objective = vec![1.0, 0.0];
        down.rows = vec![two_sided.clone()];
        let sol = ClarabelBackend.solve(&down, budget());
        assert_eq!(sol.status, LpStatus::Optimal);
        assert!(
            (sol.x.first().copied().unwrap_or(f64::NAN) - 1.0).abs() < 1.0e-4,
            "x0 {:?} != 1",
            sol.x.first()
        );

        let mut up = tiny_lp();
        up.objective = vec![-1.0, 0.0];
        up.rows = vec![two_sided];
        let sol = ClarabelBackend.solve(&up, budget());
        assert_eq!(sol.status, LpStatus::Optimal);
        assert!(
            (sol.x.first().copied().unwrap_or(f64::NAN) - 3.0).abs() < 1.0e-4,
            "x0 {:?} != 3",
            sol.x.first()
        );
    }

    /// `min (mu/2)(x0² + x1²)  s.t.  x0 + x1 >= 2` 의 해는 대칭점 `(1, 1)` 이다.
    /// 단체법으로는 표현조차 안 되는 문제 — 내점법 경로가 실제로 살아 있다는 증거다.
    #[test]
    fn a_quadratic_objective_finds_the_symmetric_interior_optimum() {
        let mu = 1.0;
        let qp = LpProblem {
            objective: vec![0.0, 0.0],
            quadratic: mu,
            lower: vec![-10.0, -10.0],
            upper: vec![10.0, 10.0],
            rows: vec![LpRow::greater_or_equal(vec![0, 1], vec![1.0, 1.0], 2.0)],
        };
        let sol = ClarabelBackend.solve(&qp, budget());
        assert_eq!(sol.status, LpStatus::Optimal);
        let x0 = sol.x.first().copied().unwrap_or(f64::NAN);
        let x1 = sol.x.get(1).copied().unwrap_or(f64::NAN);
        assert!((x0 - 1.0).abs() < 1.0e-4, "x0 {x0} != 1");
        assert!((x1 - 1.0).abs() < 1.0e-4, "x1 {x1} != 1");
        // (mu/2)(1 + 1) = 1
        assert!(
            (sol.objective - 1.0).abs() < 1.0e-4,
            "목적값 {} != 1",
            sol.objective
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
        let sol = ClarabelBackend.solve(&p, budget());
        assert_eq!(sol.status, LpStatus::Infeasible);
        assert!(sol.x.is_empty(), "증명서를 해로 돌려주면 안 된다");
    }

    #[test]
    fn a_problem_that_fails_validation_returns_failed_without_panicking() {
        let mut broken = tiny_lp();
        broken.objective.clear();
        let sol = ClarabelBackend.solve(&broken, budget());
        assert_eq!(sol.status, LpStatus::Failed);
        assert!(sol.x.is_empty());

        let mut ragged = tiny_lp();
        ragged.rows[0].vals.pop();
        assert_eq!(
            ClarabelBackend.solve(&ragged, budget()).status,
            LpStatus::Failed
        );

        let mut out_of_range = tiny_lp();
        out_of_range.rows[0].cols = vec![0, 9];
        assert_eq!(
            ClarabelBackend.solve(&out_of_range, budget()).status,
            LpStatus::Failed
        );
    }

    /// 무한대 경계는 행을 만들지 않는다 — 유한한 쪽만 남아야 한다.
    #[test]
    fn infinite_bounds_produce_no_conic_rows() {
        let p = LpProblem {
            objective: vec![1.0],
            quadratic: 0.0,
            lower: vec![0.0],
            upper: vec![f64::INFINITY],
            rows: vec![LpRow::greater_or_equal(vec![0], vec![1.0], 1.0)],
        };
        let form = ClarabelBackend::conic_form(&p).expect(crate::dmsg!("조립이 되어야 한다"));
        // 행 하한 1 개 + 변수 하한 1 개 = 2. 무한 상한 두 개는 빠진다.
        assert_eq!(form.b.len(), 2, "b = {:?}", form.b);
        let sol = ClarabelBackend.solve(&p, budget());
        assert_eq!(sol.status, LpStatus::Optimal);
        assert!((sol.objective - 1.0).abs() < 1.0e-4);
    }

    /// CSC 는 열 우선이고 열 안의 행 인덱스가 오름차순이어야 한다.
    #[test]
    fn the_constraint_matrix_is_column_major_with_sorted_row_indices() {
        let p = tiny_lp();
        let form = ClarabelBackend::conic_form(&p).expect(crate::dmsg!("조립이 되어야 한다"));
        let a = &form.a;
        assert_eq!(a.n, 2);
        assert_eq!(a.m, form.b.len());
        assert_eq!(a.colptr.len(), a.n + 1);
        assert_eq!(a.colptr.first().copied(), Some(0));
        assert_eq!(a.colptr.last().copied(), Some(a.rowval.len()));
        assert_eq!(a.rowval.len(), a.nzval.len());
        for j in 0..a.n {
            let (Some(&s), Some(&e)) = (a.colptr.get(j), a.colptr.get(j + 1)) else {
                panic!("{}", crate::dmsg!("colptr 이 짧다"));
            };
            assert!(s <= e, "colptr 이 단조 증가해야 한다");
            let Some(slice) = a.rowval.get(s..e) else {
                panic!("{}", crate::dmsg!("rowval 범위가 어긋났다"));
            };
            assert!(
                slice.windows(2).all(|w| w[0] < w[1]),
                "열 {j} 의 행 인덱스가 정렬/중복 조건을 깼다: {slice:?}"
            );
        }
    }

    /// 중복 열은 CSC 좌표 중복이 되어 조용히 틀린 행렬을 만든다 — 합쳐야 한다.
    #[test]
    fn duplicate_columns_in_one_row_are_merged_into_a_single_entry() {
        let terms = coalesced_terms(&[0, 0, 1], &[1.0, 1.0, 3.0], 2);
        assert_eq!(terms, vec![(0, 2.0), (1, 3.0)]);
        // 범위 밖 열은 버린다 (validate 가 먼저 잡지만 방어적으로).
        assert_eq!(coalesced_terms(&[5], &[1.0], 2), Vec::new());
    }

    #[test]
    fn a_zero_budget_means_no_limit_rather_than_instant_timeout() {
        let s = settings_for(Duration::ZERO).expect(crate::dmsg!("설정은 만들어져야 한다"));
        assert!(s.time_limit.is_infinite());
        let s = settings_for(Duration::from_millis(250)).expect(crate::dmsg!("설정은 만들어져야 한다"));
        assert!((s.time_limit - 0.25).abs() < 1.0e-9);
    }

    #[test]
    fn solver_statuses_map_onto_the_contract() {
        assert_eq!(map_status(SolverStatus::Solved), LpStatus::Optimal);
        assert_eq!(
            map_status(SolverStatus::PrimalInfeasible),
            LpStatus::Infeasible
        );
        assert_eq!(
            map_status(SolverStatus::DualInfeasible),
            LpStatus::Unbounded
        );
        assert_eq!(map_status(SolverStatus::MaxIterations), LpStatus::Limit);
        assert_eq!(map_status(SolverStatus::MaxTime), LpStatus::Limit);
        assert_eq!(
            map_status(SolverStatus::CallbackTerminated),
            LpStatus::Limit
        );
        assert_eq!(map_status(SolverStatus::NumericalError), LpStatus::Failed);
        assert_eq!(map_status(SolverStatus::AlmostSolved), LpStatus::Failed);
        assert_eq!(map_status(SolverStatus::Unsolved), LpStatus::Failed);
    }

    /// `mu = 0` 이면 P 는 비어 있어야 한다 — 순수 LP 에 0 대각을 채워 넣으면
    /// KKT 인수분해만 무거워진다.
    #[test]
    fn a_zero_quadratic_coefficient_yields_an_empty_p_matrix() {
        let p = ClarabelBackend::quadratic_matrix(0.0, 3);
        assert_eq!(p.nzval.len(), 0);
        assert_eq!(p.colptr.len(), 4);

        let p = ClarabelBackend::quadratic_matrix(2.5, 3);
        assert_eq!(p.nzval, vec![2.5, 2.5, 2.5]);
        assert_eq!(p.rowval, vec![0, 1, 2]);
        assert_eq!(p.colptr, vec![0, 1, 2, 3]);
    }

    /// 무계 목적은 원뿔 형식에서 **쌍대 실행 불가**로 나타난다.
    /// `DualInfeasible -> Unbounded` 매핑이 실제 솔버에서도 성립하는지 본다
    /// (map_status 단위 테스트는 매핑 표만 볼 뿐 솔버가 그 상태를 내는지는 모른다).
    #[test]
    fn an_unbounded_objective_is_reported_as_unbounded() {
        let p = LpProblem {
            objective: vec![-1.0],
            quadratic: 0.0,
            lower: vec![0.0],
            upper: vec![f64::INFINITY],
            rows: vec![LpRow::greater_or_equal(vec![0], vec![1.0], 0.0)],
        };
        let sol = ClarabelBackend.solve(&p, budget());
        assert_eq!(sol.status, LpStatus::Unbounded);
        assert!(sol.x.is_empty(), "증명서를 해로 돌려주면 안 된다");
    }

    /// 유한 경계가 하나도 없으면 원뿔 차원이 **0** 이다. `validate()` 는 이걸
    /// 통과시키므로(경계가 뒤집히지 않았고 변수도 있다) 백엔드까지 그대로 내려온다.
    /// `NonnegativeConeT(0)` 과 `0 x n` 행렬을 clarabel 에 넘기는 경로가 실제로
    /// panic 하지 않는지 확인한다 — 폴백 대상이 아니어도 계약은 "어떤 입력에도 panic 금지"다.
    #[test]
    fn a_model_without_any_finite_bound_yields_an_empty_cone_without_panicking() {
        let p = LpProblem {
            objective: vec![1.0],
            quadratic: 0.0,
            lower: vec![f64::NEG_INFINITY],
            upper: vec![f64::INFINITY],
            rows: Vec::new(),
        };
        let form = ClarabelBackend::conic_form(&p).expect(crate::dmsg!("빈 원뿔도 조립되어야 한다"));
        assert!(form.b.is_empty(), "제약이 하나도 없어야 한다");
        assert_eq!(form.a.m, 0);
        // 제약 없는 min x0 는 무계다. 상태가 무엇이든 panic 하지 않는 것이 요점이다.
        let sol = ClarabelBackend.solve(&p, budget());
        assert_ne!(sol.status, LpStatus::Optimal, "무계 문제에 최적해는 없다");
    }

    /// 등식 행(`lower == upper`)을 `ZeroConeT` 없이 **비음 원뿔 두 행**으로 접었을 때
    /// 내점법이 실제로 수렴하는가. 등식은 실행가능영역의 내부를 그 방향으로 없애 버려서
    /// 중심 경로가 퇴화한다 — 모듈 문서가 "두 개의 비음 행으로 넣어도 같은 해를 낸다"고
    /// 주장하므로 그 주장을 그대로 시험한다. 여기서 깨지면 ZeroConeT 분리가 필요하다는 뜻이다.
    #[test]
    fn an_equality_row_still_converges_without_a_zero_cone() {
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
            let sol = ClarabelBackend.solve(&p, budget());
            assert_eq!(sol.status, LpStatus::Optimal, "목적 {objective:?}");
            let x0 = sol.x.first().copied().unwrap_or(f64::NAN);
            assert!(
                (x0 - 2.0).abs() < 1.0e-4,
                "목적 {objective:?} 에서 x0 {x0} != 2"
            );
        }
    }

    /// 어떤 행에도 나오지 않는 변수는 CSC 에서 **빈 열**이 된다.
    /// `colptr` 이 같은 값을 두 번 담는 경우라 조립 로직이 여기서 어긋나기 쉽다.
    #[test]
    fn a_variable_that_appears_in_no_row_leaves_an_empty_csc_column() {
        let p = LpProblem {
            objective: vec![1.0, 0.0],
            quadratic: 0.0,
            // x1 은 경계도 없고 어떤 행에도 없다 -> A 의 두 번째 열이 통째로 비었다.
            lower: vec![0.0, f64::NEG_INFINITY],
            upper: vec![f64::INFINITY, f64::INFINITY],
            rows: vec![LpRow::greater_or_equal(vec![0], vec![1.0], 1.0)],
        };
        let form = ClarabelBackend::conic_form(&p).expect(crate::dmsg!("조립이 되어야 한다"));
        assert_eq!(form.a.n, 2);
        let (Some(&s), Some(&e)) = (form.a.colptr.get(1), form.a.colptr.get(2)) else {
            panic!("{}", crate::dmsg!("colptr 이 짧다"));
        };
        assert_eq!(s, e, "x1 의 열은 비어 있어야 한다");

        let sol = ClarabelBackend.solve(&p, budget());
        assert_eq!(sol.status, LpStatus::Optimal);
        assert_eq!(sol.x.len(), 2, "빈 열이 있어도 해는 변수 개수만큼 나온다");
        assert!(
            (sol.objective - 1.0).abs() < 1.0e-4,
            "목적값 {}",
            sol.objective
        );
    }
}
