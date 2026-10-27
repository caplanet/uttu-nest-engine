//! LP·QP 솔버 백엔드 — 설정으로 고르고 분기 실행한다.
//!
//! 압축 단계는 "조각들을 조금씩 옮겨 strip 폭을 줄이는" 선형계획을 푼다. 모델은
//! 하나지만 **푸는 엔진은 갈아 끼울 수 있어야 한다** — 단체법과 내점법은 같은 문제에서
//! 전혀 다른 시간·정확도를 내고, 현장에 이미 상용 솔버가 있으면 그걸 쓰는 편이 낫다.
//!
//! ```text
//! LpProblem  (모델 — 백엔드와 무관)
//!     |
//!     +-- Microlp    단체법, 순수 Rust, 항상 사용 가능        LP
//!     +-- Clarabel   내점법(barrier), 순수 Rust               LP + QP
//!     +-- Clp        COIN-OR, 런타임 DLL 로딩                 LP
//!     +-- Xpress     FICO, 런타임 DLL 로딩                    LP + QP
//! ```
//!
//! # Clp / Xpress 를 링크하지 않는 이유
//!
//! 둘 다 빌드 시점에 벤더 SDK 를 요구한다. Xpress 는 라이선스도 필요하다. 그걸
//! `Cargo.toml` 에 박으면 **SDK 가 없는 모든 환경에서 빌드가 깨진다**.
//!
//! 그래서 두 백엔드는 C ABI 를 **런타임에 동적 로딩**한다([`libloading`]).
//! 라이브러리가 없으면 그 사실을 로그로 알리고 [`LpSolverConfig::fallback`]에 따라
//! 물러난다. 빌드는 어디서나 되고, DLL 을 놓아 두면 그때부터 동작한다.
//!
//! 필요한 파일은 실행 파일 옆이나 `PATH` 에 두면 된다.
//!
//! ```text
//! Clp     Windows: clp.dll        Linux: libClp.so
//! Xpress  Windows: xprs.dll       Linux: libxprs.so
//! ```
//!
//! # QP
//!
//! 순수 LP 는 목적이 `min W`(폭) 뿐이라 해가 퇴화하기 쉽다 — 폭이 같은 배치가 무수히
//! 많고, 그중 아무거나 고르면 조각이 불필요하게 멀리 튄다. 2차항
//! `(mu/2)·sum(dx² + dy²)` 을 얹으면 **같은 폭이면 덜 움직이는** 해가 유일하게 뽑히고,
//! 선형화가 유효한 신뢰 영역 안에 머물러 백트래킹이 줄어든다.
//!
//! `quadratic_regularization > 0` 이면 QP 다. LP 전용 백엔드에서는 무시하고 경고한다.

pub mod clarabel_backend;
pub mod clp;
pub mod microlp_backend;
pub mod xpress;

use std::time::Duration;

/// 어떤 엔진으로 풀지.
///
/// [`crate::config::SwarmConfig`]가 `Copy`라 이 열거형도 `Copy`다.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LpSolverKind {
    /// 단체법, 순수 Rust. 의존성이 이미 들어와 있어 **항상** 쓸 수 있다. LP 전용.
    Microlp,
    /// 내점법(barrier), 순수 Rust. **QP 를 푼다.** 제약이 많은 큰 문제에서 단체법보다
    /// 잘 버티고, 2차 정규화를 쓰려면 이쪽이거나 Xpress 여야 한다.
    Clarabel,
    /// COIN-OR Clp. 런타임에 `clp.dll` / `libClp.so` 를 찾는다. LP 전용.
    Clp,
    /// FICO Xpress. 런타임에 `xprs.dll` / `libxprs.so` 를 찾는다. LP + QP.
    /// 라이선스가 있어야 실제로 푼다.
    Xpress,
}

impl LpSolverKind {
    /// 사람이 읽을 이름. 로그에 그대로 나간다.
    pub fn label(self) -> &'static str {
        match self {
            Self::Microlp => "microlp(단체법)",
            Self::Clarabel => "clarabel(내점법)",
            Self::Clp => "COIN-OR Clp",
            Self::Xpress => "FICO Xpress",
        }
    }

    /// 이 엔진이 2차 목적항을 푸는가.
    pub fn supports_quadratic(self) -> bool {
        matches!(self, Self::Clarabel | Self::Xpress)
    }

    /// 빌드에 항상 포함되어 실패할 수 없는 엔진인가.
    ///
    /// `false` 면 런타임 로딩이 필요하고, 실패할 수 있다.
    pub fn is_always_available(self) -> bool {
        matches!(self, Self::Microlp | Self::Clarabel)
    }
}

/// 솔버 선택 설정.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LpSolverConfig {
    /// 어떤 엔진으로 풀지.
    pub backend: LpSolverKind,
    /// 2차 정규화 계수 `mu`. `0` 이면 순수 LP.
    ///
    /// 단위에 주의한다. 목적의 1차항은 폭 `W`(mm)이고 2차항은 `(mu/2)·sum(d²)`(mm²)이라,
    /// 조각이 `d` mm 움직일 때 드는 벌점이 `mu·d²/2` 다. 신뢰 영역이 `T` mm 라면
    /// `mu ~ 1/T` 정도가 "폭 1mm 를 얻자고 조각을 T mm 옮기는 것"과 균형이 맞는다.
    pub quadratic_regularization: f32,
    /// 선택한 백엔드를 쓸 수 없을 때 [`LpSolverKind::Microlp`]로 물러날지.
    ///
    /// `false` 면 압축 단계를 통째로 건너뛴다. 상용 솔버 결과를 재현해야 하는
    /// 검증 실행에서 조용한 폴백은 오히려 해롭기 때문에 선택지로 남긴다.
    pub fallback: bool,
    /// 한 번의 풀이에 허용할 시간(ms). `0` 이하면 [`DEFAULT_LP_TIME_LIMIT_MS`].
    pub time_limit_ms: u64,
}

/// 한 번의 LP 풀이 기본 제한 시간(ms).
pub const DEFAULT_LP_TIME_LIMIT_MS: u64 = 5_000;

/// 기존 동작 그대로 — 단체법, 순수 LP, 폴백 켜짐.
pub const DEFAULT_LP_SOLVER_CONFIG: LpSolverConfig = LpSolverConfig {
    backend: LpSolverKind::Microlp,
    quadratic_regularization: 0.0,
    fallback: true,
    time_limit_ms: DEFAULT_LP_TIME_LIMIT_MS,
};

impl Default for LpSolverConfig {
    fn default() -> Self {
        DEFAULT_LP_SOLVER_CONFIG
    }
}

impl LpSolverConfig {
    /// `0` 이하 필드를 기본값으로 올린 사본.
    pub fn normalized(mut self) -> Self {
        if self.time_limit_ms == 0 {
            self.time_limit_ms = DEFAULT_LP_TIME_LIMIT_MS;
        }
        if !(self.quadratic_regularization.is_finite() && self.quadratic_regularization > 0.0) {
            self.quadratic_regularization = 0.0;
        }
        self
    }

    /// 2차 목적항을 실제로 쓰는가. 켰더라도 LP 전용 백엔드면 `false`.
    pub fn uses_quadratic(&self) -> bool {
        self.quadratic_regularization > 0.0 && self.backend.supports_quadratic()
    }

    /// 제한 시간.
    pub fn time_limit(&self) -> Duration {
        Duration::from_millis(self.normalized().time_limit_ms)
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 문제 모델 — 백엔드와 무관한 표준형
// ────────────────────────────────────────────────────────────────────────────

/// 제약 한 행: `lower <= sum(vals[i] * x[cols[i]]) <= upper`.
///
/// 한쪽만 필요하면 반대쪽에 무한대를 넣는다. 등식은 양쪽을 같게 둔다.
#[derive(Debug, Clone, Default)]
pub struct LpRow {
    /// 계수가 0이 아닌 열 인덱스.
    pub cols: Vec<usize>,
    /// 같은 순서의 계수.
    pub vals: Vec<f64>,
    /// 행 하한. 없으면 `f64::NEG_INFINITY`.
    pub lower: f64,
    /// 행 상한. 없으면 `f64::INFINITY`.
    pub upper: f64,
}

impl LpRow {
    /// `sum(a_i x_i) >= rhs` 행.
    pub fn greater_or_equal(cols: Vec<usize>, vals: Vec<f64>, rhs: f64) -> Self {
        Self {
            cols,
            vals,
            lower: rhs,
            upper: f64::INFINITY,
        }
    }

    /// `sum(a_i x_i) <= rhs` 행.
    pub fn less_or_equal(cols: Vec<usize>, vals: Vec<f64>, rhs: f64) -> Self {
        Self {
            cols,
            vals,
            lower: f64::NEG_INFINITY,
            upper: rhs,
        }
    }

    /// 유효한 행인가. 길이가 어긋나거나 비유한 계수가 있으면 백엔드가 거절해야 한다.
    ///
    /// **같은 열이 두 번 나오는 것도 거절한다.** microlp 0.6 의 `Problem::add_constraint`는
    /// 내부에서 `CsVec::new_from_unsorted(...).unwrap()` 을 부르는데, 그 생성자는 인덱스가
    /// 중복되면 `Err` 를 내므로 그대로 **panic** 한다. CSC 로 조립하는 백엔드에서는
    /// panic 대신 좌표가 중복된 채 조용히 틀린 행렬이 되어 더 나쁘다.
    ///
    /// 백엔드마다 계수를 합쳐 방어할 수도 있지만, 그러면 새 백엔드를 추가하는 사람이
    /// 반드시 같은 함정을 다시 밟는다. 계약 층에서 한 번만 막는다.
    pub fn is_well_formed(&self) -> bool {
        if self.cols.len() != self.vals.len()
            || self.cols.is_empty()
            || !self.vals.iter().all(|v| v.is_finite())
            || self.lower.is_nan()
            || self.upper.is_nan()
            || self.lower > self.upper
            || self.lower == f64::INFINITY
            || self.upper == f64::NEG_INFINITY
        {
            return false;
        }
        // 행 하나의 열 수는 보통 한 자릿수라 정렬 사본이 해시셋보다 싸다.
        if self.cols.len() <= 8 {
            // Contact rows are short: avoid one heap allocation per row.
            self.cols
                .iter()
                .enumerate()
                .all(|(i, col)| !self.cols[..i].contains(col))
        } else {
            let mut sorted = self.cols.clone();
            sorted.sort_unstable();
            sorted.windows(2).all(|w| w[0] != w[1])
        }
    }
}

/// ```text
/// minimize    c^T x + (mu / 2) * x^T x
/// subject to  rows
///             lower <= x <= upper
/// ```
///
/// `mu = 0` 이면 순수 LP 다.
#[derive(Debug, Clone, Default)]
pub struct LpProblem {
    /// 1차 목적 계수. 길이가 변수 개수를 정한다.
    pub objective: Vec<f64>,
    /// 2차 정규화 계수 `mu`. `0` 이면 LP.
    ///
    /// 대각 형태(`mu * I`)만 지원한다. 압축 모델이 필요로 하는 것이 정확히 그것이고,
    /// 일반 행렬을 받으면 백엔드마다 표현이 갈려 계약이 흐려진다.
    pub quadratic: f64,
    /// 변수 하한.
    pub lower: Vec<f64>,
    /// 변수 상한.
    pub upper: Vec<f64>,
    /// 제약 행.
    pub rows: Vec<LpRow>,
}

impl LpProblem {
    /// 변수 개수.
    pub fn n_vars(&self) -> usize {
        self.objective.len()
    }

    /// 제약 개수.
    pub fn n_rows(&self) -> usize {
        self.rows.len()
    }

    /// 모든 백엔드가 넘기기 전에 확인해야 하는 최소 조건.
    ///
    /// 여기서 걸러 두면 벤더 라이브러리에 깨진 배열을 넘겨 프로세스를 죽이는 일이 없다.
    pub fn validate(&self) -> Result<(), String> {
        let n = self.n_vars();
        if n == 0 {
            return Err("변수가 없다".into());
        }
        if self.lower.len() != n || self.upper.len() != n {
            return Err(format!(
                "경계 길이 불일치: lower {}, upper {}, 변수 {n}",
                self.lower.len(),
                self.upper.len()
            ));
        }
        if !self.objective.iter().all(|v| v.is_finite()) {
            return Err("목적 계수에 비유한 값이 있다".into());
        }
        if !self.quadratic.is_finite() || self.quadratic < 0.0 {
            return Err(format!("2차 계수가 유효하지 않다: {}", self.quadratic));
        }
        for (i, (lo, hi)) in self.lower.iter().zip(self.upper.iter()).enumerate() {
            if lo.is_nan()
                || hi.is_nan()
                || lo > hi
                || *lo == f64::INFINITY
                || *hi == f64::NEG_INFINITY
            {
                return Err(format!("변수 {i}의 경계가 뒤집혔다: [{lo}, {hi}]"));
            }
        }
        for (i, row) in self.rows.iter().enumerate() {
            if !row.is_well_formed() {
                return Err(format!("제약 {i} 이 유효하지 않다"));
            }
            if let Some(&bad) = row.cols.iter().find(|&&c| c >= n) {
                return Err(format!("제약 {i} 이 범위 밖 열 {bad} 을 참조한다"));
            }
        }
        Ok(())
    }

    /// Check a primal vector against the original model, independently of the
    /// backend's status or its internal representation of infinity.
    pub fn is_primal_feasible(&self, x: &[f64]) -> bool {
        if x.len() != self.n_vars()
            || self.lower.len() != x.len()
            || self.upper.len() != x.len()
            || !x.iter().all(|v| v.is_finite())
        {
            return false;
        }
        let within = |v: f64, lo: f64, hi: f64| {
            // Relative tolerance with an absolute floor: interior-point solvers
            // (Clarabel) stop at ~1e-8 relative gaps, so tiny bounds would
            // otherwise reject valid solutions.
            let slack = |bound: f64| (1.0e-7 * bound.abs()).max(1.0e-6);
            !lo.is_nan()
                && !hi.is_nan()
                && lo <= hi
                && (lo == f64::NEG_INFINITY || lo.is_finite() && v >= lo - slack(lo))
                && (hi == f64::INFINITY || hi.is_finite() && v <= hi + slack(hi))
        };
        if !x
            .iter()
            .zip(&self.lower)
            .zip(&self.upper)
            .all(|((&v, &lo), &hi)| within(v, lo, hi))
        {
            return false;
        }
        self.rows.iter().all(|row| {
            if row.cols.len() != row.vals.len() {
                return false;
            }
            let activity = row
                .cols
                .iter()
                .zip(&row.vals)
                .try_fold(0.0, |sum, (&col, &value)| {
                    let next = sum + value * x.get(col)?;
                    next.is_finite().then_some(next)
                });
            activity.is_some_and(|v| within(v, row.lower, row.upper))
        })
    }
}

/// 풀이 결과 상태.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LpStatus {
    /// 최적해를 찾았다.
    Optimal,
    /// 실행 가능한 해가 없다.
    Infeasible,
    /// 목적이 무한히 좋아진다 — 모델이 잘못됐다는 뜻이다.
    Unbounded,
    /// 시간/반복 제한에 걸렸다. `x` 는 그때까지의 최선이며 신뢰할 수 없다.
    Limit,
    /// 백엔드를 쓸 수 없거나 내부 오류. `x` 는 비어 있다.
    Failed,
}

impl LpStatus {
    /// 해를 채택해도 되는가. `Limit` 는 채택하지 않는다 — 압축은 실행 가능성을
    /// 사후 검증하지만, 신뢰할 수 없는 해로 검증 비용을 쓰는 것 자체가 낭비다.
    pub fn is_usable(self) -> bool {
        matches!(self, Self::Optimal)
    }
}

/// 풀이 결과.
#[derive(Debug, Clone)]
pub struct LpSolution {
    pub status: LpStatus,
    /// 변수 값. `status` 가 [`LpStatus::Failed`] 면 비어 있다.
    pub x: Vec<f64>,
    /// 목적 함수값.
    pub objective: f64,
    /// 실제로 푼 백엔드 이름. 폴백이 일어났으면 요청과 다를 수 있다.
    pub solved_by: &'static str,
}

impl LpSolution {
    pub(crate) fn checked(self, model: &LpProblem) -> Self {
        if self.status.is_usable()
            && (!self.objective.is_finite() || !model.is_primal_feasible(&self.x))
        {
            log::warn!(
                "[LP] {} returned an invalid primal solution",
                self.solved_by
            );
            Self::failed(self.solved_by)
        } else {
            self
        }
    }

    /// 실패 결과.
    pub fn failed(solved_by: &'static str) -> Self {
        Self {
            status: LpStatus::Failed,
            x: Vec::new(),
            objective: f64::NAN,
            solved_by,
        }
    }
}

/// 갈아 끼울 수 있는 풀이 엔진.
pub trait LpBackend {
    /// 로그에 나갈 이름.
    fn name(&self) -> &'static str;

    /// 지금 이 환경에서 실제로 쓸 수 있는가.
    ///
    /// 동적 로딩 백엔드는 여기서 라이브러리를 찾는다. 실패 사유를 문자열로 돌려
    /// 호출부가 사용자에게 그대로 보여 줄 수 있게 한다.
    fn availability(&self) -> Result<(), String>;

    /// 2차 목적항을 푸는가.
    fn supports_quadratic(&self) -> bool;

    /// 푼다. 어떤 입력에도 panic 하지 않아야 한다.
    fn solve(&self, problem: &LpProblem, time_limit: Duration) -> LpSolution;

    /// Backends with resumable solves override this to poll between solver slices.
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
            solved_by: self.name(),
        };
        if should_stop() {
            return limited();
        }
        let started = jagua_rs::Instant::now();
        let solution = self.solve(problem, time_limit);
        // Native one-shot backends may finish after cancellation or their soft
        // deadline. Their result must not escape through the common contract.
        // A zero `time_limit` means "no limit", as it does for every backend.
        if should_stop() || (!time_limit.is_zero() && started.elapsed() >= time_limit) {
            limited()
        } else {
            solution
        }
    }
}

/// 설정이 가리키는 백엔드를 만든다.
///
/// 쓸 수 없고 `fallback` 이 켜져 있으면 [`LpSolverKind::Microlp`]로 물러난다.
/// 꺼져 있으면 `None` 이고, 호출부는 압축을 건너뛴다.
pub fn backend_for(cfg: &LpSolverConfig) -> Option<Box<dyn LpBackend>> {
    let requested: Box<dyn LpBackend> = match cfg.backend {
        LpSolverKind::Microlp => Box::new(microlp_backend::MicrolpBackend),
        LpSolverKind::Clarabel => Box::new(clarabel_backend::ClarabelBackend),
        LpSolverKind::Clp => Box::new(clp::ClpBackend),
        LpSolverKind::Xpress => Box::new(xpress::XpressBackend),
    };

    match requested.availability() {
        Ok(()) => {
            if cfg.quadratic_regularization > 0.0 && !requested.supports_quadratic() {
                log::warn!(
                    "[LP] {} 는 2차 목적항을 풀지 못한다 — 정규화 {}을 무시하고 순수 LP 로 푼다",
                    requested.name(),
                    cfg.quadratic_regularization,
                );
            }
            Some(requested)
        }
        Err(reason) => {
            if cfg.fallback {
                log::warn!(
                    "[LP] {} 를 쓸 수 없다 ({reason}) — microlp(단체법)로 물러난다",
                    cfg.backend.label(),
                );
                Some(Box::new(microlp_backend::MicrolpBackend))
            } else {
                log::warn!(
                    "[LP] {} 를 쓸 수 없고 폴백이 꺼져 있다 ({reason}) — 압축 단계를 건너뛴다",
                    cfg.backend.label(),
                );
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `min x0 + x1`, `x0 + x1 >= 2`, `0 <= x <= 10`  ->  목적값 2.
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
    fn default_keeps_the_existing_simplex_behaviour() {
        let cfg = LpSolverConfig::default();
        assert_eq!(cfg.backend, LpSolverKind::Microlp);
        assert_eq!(cfg.quadratic_regularization, 0.0);
        assert!(cfg.fallback);
        assert!(!cfg.uses_quadratic());
    }

    #[test]
    fn quadratic_is_only_active_on_capable_backends() {
        let qp = LpSolverConfig {
            backend: LpSolverKind::Clarabel,
            quadratic_regularization: 0.5,
            ..DEFAULT_LP_SOLVER_CONFIG
        };
        assert!(qp.uses_quadratic());

        // 단체법은 2차항을 못 푼다 — 켜도 무시된다.
        let lp = LpSolverConfig {
            backend: LpSolverKind::Microlp,
            quadratic_regularization: 0.5,
            ..DEFAULT_LP_SOLVER_CONFIG
        };
        assert!(!lp.uses_quadratic());
    }

    #[test]
    fn validate_rejects_malformed_problems() {
        assert!(tiny_lp().validate().is_ok());

        let mut empty = tiny_lp();
        empty.objective.clear();
        assert!(empty.validate().is_err(), "변수 0개는 거절해야 한다");

        let mut short_bounds = tiny_lp();
        short_bounds.lower.pop();
        assert!(short_bounds.validate().is_err(), "경계 길이 불일치");

        let mut flipped = tiny_lp();
        flipped.lower[0] = 5.0;
        flipped.upper[0] = 1.0;
        assert!(flipped.validate().is_err(), "뒤집힌 경계");

        let mut out_of_range = tiny_lp();
        out_of_range.rows[0].cols = vec![0, 9];
        assert!(out_of_range.validate().is_err(), "범위 밖 열 참조");

        let mut ragged = tiny_lp();
        ragged.rows[0].vals.pop();
        assert!(ragged.validate().is_err(), "cols/vals 길이 불일치");

        let mut nan_obj = tiny_lp();
        nan_obj.objective[0] = f64::NAN;
        assert!(nan_obj.validate().is_err(), "비유한 목적 계수");

        // 중복 열은 microlp 를 panic 시키고 CSC 를 조용히 망가뜨린다.
        let mut duplicate = tiny_lp();
        duplicate.rows[0] = LpRow::greater_or_equal(vec![0, 0], vec![1.0, 1.0], 2.0);
        assert!(
            duplicate.validate().is_err(),
            "같은 열이 두 번 나오는 행은 계약 층에서 막아야 한다"
        );

        // 열 순서가 뒤섞여 있어도 중복만 없으면 유효하다.
        let mut unsorted = tiny_lp();
        unsorted.rows[0] = LpRow::greater_or_equal(vec![1, 0], vec![1.0, 1.0], 2.0);
        assert!(unsorted.validate().is_ok(), "정렬은 요구하지 않는다");
    }

    /// 항상 쓸 수 있는 두 백엔드는 같은 LP 에서 같은 최적값을 내야 한다.
    #[test]
    fn builtin_backends_agree_on_a_tiny_lp() {
        let problem = tiny_lp();
        for kind in [LpSolverKind::Microlp, LpSolverKind::Clarabel] {
            let cfg = LpSolverConfig {
                backend: kind,
                ..DEFAULT_LP_SOLVER_CONFIG
            };
            let backend = backend_for(&cfg).expect(crate::dmsg!("내장 백엔드는 항상 있어야 한다"));
            assert!(backend.availability().is_ok(), "{}", kind.label());
            let sol = backend.solve(&problem, Duration::from_secs(5));
            assert_eq!(sol.status, LpStatus::Optimal, "{}", kind.label());
            assert!(
                (sol.objective - 2.0).abs() < 1.0e-4,
                "{} 목적값 {} != 2",
                kind.label(),
                sol.objective
            );
        }
    }

    /// 동적 로딩 백엔드는 라이브러리가 없어도 **panic 하지 않고** 폴백해야 한다.
    #[test]
    fn dynamic_backends_fall_back_instead_of_failing() {
        for kind in [LpSolverKind::Clp, LpSolverKind::Xpress] {
            let cfg = LpSolverConfig {
                backend: kind,
                ..DEFAULT_LP_SOLVER_CONFIG
            };
            // 라이브러리가 있든 없든 Some 이어야 한다 (없으면 microlp).
            let backend = backend_for(&cfg).expect(crate::dmsg!("폴백이 켜져 있으면 항상 Some"));
            let sol = backend.solve(&tiny_lp(), Duration::from_secs(5));
            assert!(
                sol.status == LpStatus::Optimal,
                "{} 경로가 최적해를 내야 한다 (직접 풀든 폴백이든)",
                kind.label()
            );

            // 폴백을 끄면 라이브러리가 없을 때 None 이다.
            let strict = LpSolverConfig {
                backend: kind,
                fallback: false,
                ..DEFAULT_LP_SOLVER_CONFIG
            };
            let available = match kind {
                LpSolverKind::Clp => clp::ClpBackend.availability().is_ok(),
                _ => xpress::XpressBackend.availability().is_ok(),
            };
            assert_eq!(backend_for(&strict).is_some(), available);
        }
    }
}
