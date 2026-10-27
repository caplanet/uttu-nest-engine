//! FICO Xpress 백엔드 — C ABI 를 **런타임에** 동적 로딩한다. LP + QP.
//!
//! # 왜 링크하지 않고 동적 로딩하는가
//!
//! Xpress 는 SDK 뿐 아니라 **라이선스**까지 요구한다. `Cargo.toml` 에 박으면 라이선스가 없는
//! 모든 머신에서 빌드가 깨지고, 그건 CI 와 개발자 PC 대부분이다. 그래서 [`super::clp`] 와
//! 같은 방식으로 `xprs.dll` / `libxprs.so` 를 실행 시점에 찾는다.
//!
//! 라이선스가 없으면 `XPRSinit` 이 0 이 아닌 값을 낸다. 그 경우를 "라이브러리가 없다"와
//! 똑같이 [`LpBackend::availability`] 실패로 다뤄서, [`super::backend_for`] 가 microlp 로
//! 물러날 수 있게 한다.
//!
//! # QP
//!
//! 압축 모델의 2차 정규화 `(mu/2)·sum(dx² + dy²)` 는 **대각** 행렬이다. Xpress 는
//! `XPRSloadqp` 로 목적의 2차부를 (col1, col2, coef) 삼중항으로 받으므로 대각 항만
//! `i == j` 로 넘긴다. `XPRSloadqp` 심볼이 없는 (LP 전용) 배포판이면 경고를 남기고
//! `XPRSloadlp` 로 강등한다 — 2차항은 해의 유일성을 위한 정규화라서, 없으면 해가
//! 퇴화할 뿐 모델 자체가 틀리지는 않는다.
//!
//! # 상수 검증
//!
//! 컨트롤/속성 번호는 SDK 헤더(`xprs.h`)에 있지만 이 저장소는 헤더를 두지 않는다. 아래
//! 상수는 공개된 `xprs.h` 의 값을 그대로 옮긴 것이다. 설치된 Xpress 의 `xprs.h` 와
//! 대조해 확인해야 하며, 컨트롤 번호가 틀리면 `XPRSsetintcontrol` 등이 0 이 아닌 코드를
//! 돌려줄 뿐이라 치명적이지 않지만 **속성** 번호가 틀리면 다른 값을 조용히 읽을 수 있다.

use std::ffi::{c_char, c_double, c_int, c_void};
use std::sync::OnceLock;
use std::time::Duration;

use libloading::Library;

use super::clp::{ColumnMajor, clamp_infinity, to_column_major};
use super::{LpBackend, LpProblem, LpSolution, LpStatus};

/// 로그·`solved_by` 에 나갈 이름.
const NAME: &str = "FICO Xpress";

/// 이 값을 먼저 시도한다.
const ENV_OVERRIDE: &str = "SWARM_XPRESS_LIB";

/// 플랫폼별 표준 파일명. 위에서부터 순서대로 시도한다.
const CANDIDATE_NAMES: &[&str] = &["xprs.dll", "libxprs.so", "libxprs.dylib"];

/// Xpress 가 무한대로 취급하는 경계값(`XPRS_PLUSINFINITY`).
///
/// COIN 의 `1e30` 과 다르다. 이보다 큰 값은 무한으로 읽힌다.
const XPRS_INFINITY: f64 = 1.0e20;

// 아래 컨트롤/속성 번호는 공개된 `xprs.h` 의 `#define` 값이다. 설치된 Xpress 배포판의
// `xprs.h` 와 대조해 확인할 것 — 메이저 버전이 바뀌어도 번호는 유지되는 것이 관례지만
// 보증은 아니다.

/// 정수 컨트롤 `XPRS_OUTPUTLOG` (`xprs.h`: 8035). 0 으로 두면 라이브러리가 stdout 에
/// 아무것도 안 쓴다. 틀리면 로그가 시끄러워질 뿐이다.
const XPRS_OUTPUTLOG: c_int = 8035;

/// 정수 컨트롤 `XPRS_MAXTIME` (`xprs.h`: 8020). 단위는 **초**(정수). 양수면 그 시간이
/// 지난 뒤 중단하고, 0 이면 제한 없음. [`XPRS_TIMELIMIT`] 을 걸 수 없을 때의 폴백이다.
const XPRS_MAXTIME: c_int = 8020;

/// 실수 컨트롤 `XPRS_TIMELIMIT` (`xprs.h`: 8360). 단위는 **초**(실수). Xpress 9.0 부터
/// 있다. 먼저 이걸 걸어 보고, 실패하면(구버전) 정수 [`XPRS_MAXTIME`] 으로 다시 건다 —
/// 둘 다 실패해도 제한 시간만 못 걸 뿐 풀이는 진행한다.
const XPRS_TIMELIMIT: c_int = 8360;

/// 정수 속성 `XPRS_LPSTATUS` (`xprs.h`: 1010).
///
/// # 모듈 머리말의 안전 논거가 여기에는 적용되지 않는다
///
/// "상수가 틀려도 0 이 아닌 오류 코드가 올 뿐이라 안전하다" 는 논거는 **컨트롤
/// 설정**(`XPRSsetintcontrol` 등)에 대한 것이고, 아래 두 개는 **속성 조회 번호**다.
/// 번호가 틀리면 조회가 실패하거나 — 더 나쁘게 — **다른 속성의 값을 조용히 돌려준다.**
/// 그러면 풀이 상태나 목적값을 잘못 읽고도 성공으로 보고한다.
///
/// 확인 방법: 설치된 `xprs.h` 의 `XPRS_LPSTATUS` / `XPRS_LPOBJVAL` 정의와 대조하거나,
/// 알려진 최적값을 갖는 작은 LP 를 풀어 목적값이 일치하는지 본다.
const XPRS_LPSTATUS: c_int = 1010;

/// 실수 속성 `XPRS_LPOBJVAL` (`xprs.h`: 2001).
/// 위 [`XPRS_LPSTATUS`] 의 경고가 그대로 적용된다.
const XPRS_LPOBJVAL: c_int = 2001;

// ────────────────────────────────────────────────────────────────────────────
// 행 표현 — 상하한 쌍을 Xpress 의 (rowtype, rhs, range) 삼중항으로
// ────────────────────────────────────────────────────────────────────────────

/// Xpress 가 요구하는 행 서술. 길이는 모두 행 개수와 같다.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct RowSpec {
    /// `'E' | 'L' | 'G' | 'R' | 'N'` 중 하나.
    pub rowtype: Vec<c_char>,
    /// 우변.
    pub rhs: Vec<f64>,
    /// 범위 행(`'R'`)의 폭. 그 밖은 0.
    pub range: Vec<f64>,
}

/// [`super::LpRow`]의 `lower <= a'x <= upper` 를 Xpress 의 행 타입으로 옮긴다.
///
/// 대응은 이렇다.
///
/// ```text
/// lower == upper                 'E'  rhs = lower
/// lower = -inf,  upper 유한      'L'  rhs = upper
/// lower 유한,    upper = +inf    'G'  rhs = lower
/// 둘 다 유한하고 다름            'R'  rhs = upper,  range = upper - lower
/// 둘 다 무한                     'N'  자유 행 — 제약이 아니다
/// ```
///
/// **검증 필요**: 범위 행 `'R'` 의 규약. Xpress 문서 기준 `rhs` 는 범위의 **상한**이고
/// `range` 는 폭 `|upper - lower|` 다. 반대(하한 기준)로 읽는 버전이 있다면 범위 제약이
/// 아래로 밀린다. 압축 모델은 대부분 단측 부등식(`'G'`/`'L'`)이라 실제 노출은 적다.
///
/// 자유 행(`'N'`)을 그냥 버리지 않고 남기는 이유는, 행을 지우면 뒤 행의 인덱스가 밀려
/// [`to_column_major`] 가 만든 CSC 의 행 인덱스와 어긋나기 때문이다.
pub(crate) fn row_specs(problem: &LpProblem) -> RowSpec {
    let n = problem.n_rows();
    let mut rowtype = Vec::with_capacity(n);
    let mut rhs = Vec::with_capacity(n);
    let mut range = Vec::with_capacity(n);

    for row in &problem.rows {
        let lo_finite = row.lower > -XPRS_INFINITY && row.lower.is_finite();
        let hi_finite = row.upper < XPRS_INFINITY && row.upper.is_finite();
        let (t, r, g) = match (lo_finite, hi_finite) {
            (true, true) if row.lower == row.upper => (b'E', row.lower, 0.0),
            (true, true) => (b'R', row.upper, row.upper - row.lower),
            (false, true) => (b'L', row.upper, 0.0),
            (true, false) => (b'G', row.lower, 0.0),
            (false, false) => (b'N', 0.0, 0.0),
        };
        rowtype.push(t as c_char);
        rhs.push(clamp_infinity(r, XPRS_INFINITY));
        // 범위 폭도 잘라 넘긴다. 상하한이 각각 ±XPRS_INFINITY 바로 안쪽이면 폭이 2e20 까지
        // 커져 Xpress 의 무한대 문턱을 넘고, 그러면 범위 행이 조용히 단측 행처럼 읽힌다.
        range.push(clamp_infinity(g, XPRS_INFINITY));
    }

    RowSpec {
        rowtype,
        rhs,
        range,
    }
}

/// 대각 2차 목적항을 `XPRSloadqp` 가 받는 (col1, col2, coef) 삼중항으로 편다.
///
/// 압축 모델의 정규화는 `(mu/2)·sum(x_i²)` 이라 비대각 항이 없다. 그래서 삼중항은
/// `col1 == col2 == i` 인 항목 `n_vars` 개가 전부이고, 두 인덱스 배열의 내용이 같으므로
/// 호출부는 같은 버퍼를 두 번 넘긴다.
///
/// `mu` 가 유한한 양수가 아니면 빈 삼중항이다 — 호출부는 그걸 보고 LP 로 푼다.
/// `solve_inner` 안에 인라인으로 두면 라이브러리 없이는 한 줄도 검증할 수 없어서 뺐다.
pub(crate) fn diagonal_quadratic_terms(problem: &LpProblem) -> (Vec<c_int>, Vec<f64>) {
    let mu = problem.quadratic;
    if !(mu.is_finite() && mu > 0.0) {
        return (Vec::new(), Vec::new());
    }
    // to_column_major 가 이미 c_int 범위를 확인하고 거절하므로 여기서 잘릴 일은 없다.
    // 그래도 as 캐스팅이 조용히 음수를 만드는 것보다는 잘라 두는 편이 낫다.
    let n = problem.n_vars().min(c_int::MAX as usize);
    ((0..n as c_int).collect(), vec![mu; n])
}

/// `XPRS_LPSTATUS` -> [`LpStatus`].
///
/// 1 OPTIMAL, 2 INFEASIBLE, 5 UNBOUNDED. 3(CUTOFF)·4(UNFINISHED)는 시간/반복 제한에
/// 걸린 것이라 [`LpStatus::Limit`], 나머지(0 UNSTARTED, 6·7 UNSOLVED, 8 NONCONVEX)는 실패다.
fn map_status(code: c_int) -> LpStatus {
    match code {
        1 => LpStatus::Optimal,
        2 => LpStatus::Infeasible,
        5 => LpStatus::Unbounded,
        3 | 4 => LpStatus::Limit,
        _ => LpStatus::Failed,
    }
}

// ────────────────────────────────────────────────────────────────────────────
// C ABI 심볼
// ────────────────────────────────────────────────────────────────────────────

/// `XPRSprob` 는 불투명 포인터다.
type XprsProb = *mut c_void;

type FnInit = unsafe extern "C" fn(*const c_char) -> c_int;
type FnCreateProb = unsafe extern "C" fn(*mut XprsProb) -> c_int;
type FnDestroyProb = unsafe extern "C" fn(XprsProb) -> c_int;
type FnSetIntControl = unsafe extern "C" fn(XprsProb, c_int, c_int) -> c_int;
type FnSetDblControl = unsafe extern "C" fn(XprsProb, c_int, c_double) -> c_int;
type FnLoadLp = unsafe extern "C" fn(
    XprsProb,
    *const c_char,
    c_int,
    c_int,
    *const c_char,
    *const c_double,
    *const c_double,
    *const c_double,
    *const c_int,
    *const c_int,
    *const c_int,
    *const c_double,
    *const c_double,
    *const c_double,
) -> c_int;
/// `XPRSloadlp` 뒤에 2차 목적항 삼중항 네 개(개수, col1, col2, coef)가 더 붙는다.
type FnLoadQp = unsafe extern "C" fn(
    XprsProb,
    *const c_char,
    c_int,
    c_int,
    *const c_char,
    *const c_double,
    *const c_double,
    *const c_double,
    *const c_int,
    *const c_int,
    *const c_int,
    *const c_double,
    *const c_double,
    *const c_double,
    c_int,
    *const c_int,
    *const c_int,
    *const c_double,
) -> c_int;
type FnLpOptimize = unsafe extern "C" fn(XprsProb, *const c_char) -> c_int;
type FnGetIntAttrib = unsafe extern "C" fn(XprsProb, c_int, *mut c_int) -> c_int;
type FnGetDblAttrib = unsafe extern "C" fn(XprsProb, c_int, *mut c_double) -> c_int;
type FnGetLpSol = unsafe extern "C" fn(
    XprsProb,
    *mut c_double,
    *mut c_double,
    *mut c_double,
    *mut c_double,
) -> c_int;

/// 로딩된 Xpress 진입점 묶음.
struct XpressApi {
    create_prob: FnCreateProb,
    destroy_prob: FnDestroyProb,
    load_lp: FnLoadLp,
    lp_optimize: FnLpOptimize,
    get_int_attrib: FnGetIntAttrib,
    get_dbl_attrib: FnGetDblAttrib,
    get_lp_sol: FnGetLpSol,
    /// 없으면 로그를 못 끄는 것뿐이다.
    set_int_control: Option<FnSetIntControl>,
    /// 없으면 제한 시간을 실수로 못 거는 것뿐이다.
    set_dbl_control: Option<FnSetDblControl>,
    /// LP 전용 배포판에는 없다. 없으면 [`XpressBackend::supports_quadratic`] 은 그대로
    /// `true` 지만 실제로는 LP 로 강등하고 경고를 남긴다.
    load_qp: Option<FnLoadQp>,
    _lib: Library,
}

/// 심볼 하나를 함수 포인터로 꺼낸다.
///
/// # 안전성
///
/// 호출부가 `T` 를 실제 심볼의 C 시그니처와 맞춰야 한다. 이 모듈 안에서만 쓴다.
fn load_symbol<T: Copy>(lib: &Library, name: &[u8]) -> Result<T, String> {
    // SAFETY: name 은 이 파일에 하드코딩된 NUL 종료 심볼명이고, T 는 위에서 원본 C 선언을
    // 그대로 옮긴 함수 포인터 타입이다.
    unsafe { lib.get::<T>(name) }.map(|sym| *sym).map_err(|e| {
        format!(
            "심볼 {} 없음 ({e})",
            String::from_utf8_lossy(name).trim_end_matches('\0')
        )
    })
}

impl XpressApi {
    /// 열린 라이브러리에서 심볼을 꺼내고 `XPRSinit` 으로 라이선스를 확인한다.
    ///
    /// `XPRSfree` 는 **일부러 호출하지 않는다**. 프로세스 안에서 한 번 초기화한 뒤 살아 있는
    /// 동안 계속 쓰고, 종료는 OS 에 맡긴다. 중간에 free 하면 아직 살아 있는 다른 `XPRSprob`
    /// 가 함께 죽는다.
    fn from_library(lib: Library) -> Result<Self, String> {
        // 필수 심볼을 전부 먼저 꺼낸다. XPRSinit 을 먼저 부르고 나서 심볼이 없어 실패하면
        // XPRSfree 없이 돌아가므로 라이선스 슬롯이 새고, 다음 후보 경로에서도 init 이
        // 실패할 수 있다. 심볼이 다 갖춰진 뒤에만 라이선스를 잡는다.
        let init: FnInit = load_symbol(&lib, b"XPRSinit\0")?;
        let create_prob = load_symbol(&lib, b"XPRScreateprob\0")?;
        let destroy_prob = load_symbol(&lib, b"XPRSdestroyprob\0")?;
        let load_lp = load_symbol(&lib, b"XPRSloadlp\0")?;
        let lp_optimize = load_symbol(&lib, b"XPRSlpoptimize\0")?;
        let get_int_attrib = load_symbol(&lib, b"XPRSgetintattrib\0")?;
        let get_dbl_attrib = load_symbol(&lib, b"XPRSgetdblattrib\0")?;
        let get_lp_sol = load_symbol(&lib, b"XPRSgetlpsol\0")?;
        let set_int_control = load_symbol(&lib, b"XPRSsetintcontrol\0").ok();
        let set_dbl_control = load_symbol(&lib, b"XPRSsetdblcontrol\0").ok();
        let load_qp = load_symbol(&lib, b"XPRSloadqp\0").ok();

        // SAFETY: 널 경로는 "기본 위치에서 라이선스를 찾아라"라는 뜻의 정식 인자다.
        let rc = unsafe { init(std::ptr::null()) };
        if rc != 0 {
            return Err(format!(
                "XPRSinit 이 {rc} 를 돌려줬다 — 라이선스가 없거나 만료됐을 가능성이 높다"
            ));
        }

        Ok(Self {
            create_prob,
            destroy_prob,
            load_lp,
            lp_optimize,
            get_int_attrib,
            get_dbl_attrib,
            get_lp_sol,
            set_int_control,
            set_dbl_control,
            load_qp,
            _lib: lib,
        })
    }
}

/// 시도할 경로 목록을 만든다. `override_path` 가 있으면 **가장 먼저** 온다.
///
/// 환경변수 읽기와 순서 정하기를 나눈 이유는 [`super::clp`] 와 같다 — 순서 규칙은
/// 라이브러리 없이 검증할 수 있는 정책인데, 환경변수를 테스트에서 건드리면
/// (2024 에디션에서 `set_var` 는 `unsafe` 다) 병렬로 도는 다른 테스트를 깨뜨린다.
fn candidate_paths_from(override_path: Option<&str>) -> Vec<String> {
    let mut out = Vec::with_capacity(CANDIDATE_NAMES.len() + 1);
    if let Some(p) = override_path {
        let p = p.trim();
        if !p.is_empty() {
            out.push(p.to_string());
        }
    }
    out.extend(CANDIDATE_NAMES.iter().map(|s| (*s).to_string()));
    out
}

/// 시도할 경로 목록. 환경변수가 있으면 **가장 먼저** 온다.
fn candidate_paths() -> Vec<String> {
    candidate_paths_from(std::env::var(ENV_OVERRIDE).ok().as_deref())
}

/// 후보를 순서대로 열어 본다. 전부 실패하면 무엇을 어디서 찾았는지 담은 사유를 돌린다.
fn load_api() -> Result<XpressApi, String> {
    let mut tried: Vec<String> = Vec::new();
    for cand in candidate_paths() {
        // SAFETY: 라이브러리 로딩은 대상 DLL 의 초기화 코드를 실행한다. 사용자가 지정하거나
        // 표준 이름으로 찾은 Xpress 배포판만 대상이다.
        match unsafe { Library::new(&cand) } {
            Ok(lib) => match XpressApi::from_library(lib) {
                Ok(api) => {
                    if api.load_qp.is_none() {
                        log::warn!(
                            "[LP] {NAME}: XPRSloadqp 심볼이 없다 — 2차 정규화를 요청하면 LP 로 강등한다"
                        );
                    }
                    return Ok(api);
                }
                Err(why) => tried.push(format!("{cand} ({why})")),
            },
            Err(e) => tried.push(format!("{cand} ({e})")),
        }
    }
    Err(format!(
        "{NAME} 동적 라이브러리를 열지 못했다. 시도한 파일: {}. \
         {ENV_OVERRIDE} 에 전체 경로를 주거나, 파일을 실행 파일 옆 또는 \
         PATH/LD_LIBRARY_PATH 에 두면 된다. 라이선스가 없으면 파일이 있어도 실패한다.",
        tried.join(", ")
    ))
}

/// 프로세스당 한 번만 로딩·초기화한다. 실패도 캐시한다 — 이유는 [`super::clp`] 와 같다.
static XPRESS_API: OnceLock<Result<XpressApi, String>> = OnceLock::new();

/// 캐시된 API 를 얻거나 실패 사유를 복제해 돌린다.
fn api() -> Result<&'static XpressApi, String> {
    match XPRESS_API.get_or_init(load_api) {
        Ok(api) => Ok(api),
        Err(why) => Err(why.clone()),
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 백엔드
// ────────────────────────────────────────────────────────────────────────────

/// FICO Xpress 백엔드. LP + QP, 라이선스 필요.
///
/// 상태가 없다 — 로딩된 라이브러리는 프로세스 전역([`XPRESS_API`])에 한 번만 산다.
#[derive(Debug, Clone, Copy, Default)]
pub struct XpressBackend;

impl XpressBackend {
    /// 모델을 만들어 풀고 해를 복사해 돌린다.
    fn solve_inner(
        api: &XpressApi,
        problem: &LpProblem,
        time_limit: Duration,
    ) -> Result<LpSolution, String> {
        let n_cols = problem.n_vars();
        let n_rows = problem.n_rows();
        let matrix: ColumnMajor = to_column_major(problem)?;
        let rows = row_specs(problem);

        let lb: Vec<f64> = problem
            .lower
            .iter()
            .map(|&v| clamp_infinity(v, XPRS_INFINITY))
            .collect();
        let ub: Vec<f64> = problem
            .upper
            .iter()
            .map(|&v| clamp_infinity(v, XPRS_INFINITY))
            .collect();
        let obj = problem.objective.clone();

        // 2차부: 대각만. Xpress 는 목적을 c'x + (1/2)·x'Qx 로 본다고 가정하므로 대각
        // 원소에 mu 를 그대로 넣으면 (mu/2)·sum(x²) 가 된다.
        //
        // **검증 필요**: 대각 항의 배수 규약(x² 계수를 그대로 주는지 2배로 주는지)은 실제
        // 라이브러리로 확인해야 한다. 틀려도 정규화 세기가 2배 어긋날 뿐이라 해가 깨지지는
        // 않는다 — 2차항의 역할이 "같은 폭이면 덜 움직이는 해"를 고르는 것이기 때문이다.
        let (qcol, qcoef): (Vec<c_int>, Vec<f64>) = match api.load_qp {
            Some(_) => diagonal_quadratic_terms(problem),
            None => (Vec::new(), Vec::new()),
        };
        let use_qp = !qcol.is_empty();
        if problem.quadratic > 0.0 && !use_qp {
            log::warn!(
                "[LP] {NAME}: XPRSloadqp 가 없어 2차 계수 {}을 버리고 LP 로 푼다",
                problem.quadratic
            );
        }

        // 모델 이름. Xpress 가 파일명으로도 쓸 수 있으니 공백 없는 ASCII 로 둔다.
        let prob_name = b"swarm_compaction\0";

        // SAFETY: 아래로 넘기는 포인터는 전부 이 함수 안에서 살아 있는 Vec 이고, 길이는
        // n_cols / n_rows / matrix / rows 로 서로 맞춰 두었다. validate() 가 길이 불일치와
        // 범위 밖 열 참조를 이미 걸렀다.
        let solution = unsafe {
            let mut prob: XprsProb = std::ptr::null_mut();
            let rc = (api.create_prob)(&mut prob);
            if rc != 0 || prob.is_null() {
                return Err(format!("XPRScreateprob 이 {rc} 를 돌려줬다"));
            }

            if let Some(set_int) = api.set_int_control {
                // 로그를 끈다. 실패해도 무해하다.
                let _ = set_int(prob, XPRS_OUTPUTLOG, 0);
            }
            // 제한 시간: 실수 컨트롤 TIMELIMIT(9.0+)로 먼저, 실패하면 정수 MAXTIME(초,
            // 올림)으로 다시. 0 이면 제한을 걸지 않는다(두 컨트롤 모두 0 = 무제한).
            if !time_limit.is_zero() {
                let secs = time_limit.as_secs_f64().max(0.001);
                let dbl_ok = match api.set_dbl_control {
                    Some(set_dbl) => set_dbl(prob, XPRS_TIMELIMIT, secs) == 0,
                    None => false,
                };
                if !dbl_ok {
                    if let Some(set_int) = api.set_int_control {
                        let _ = set_int(prob, XPRS_MAXTIME, secs.ceil().max(1.0) as c_int);
                    }
                }
            }

            let load_rc = match (use_qp, api.load_qp) {
                (true, Some(load_qp)) => load_qp(
                    prob,
                    prob_name.as_ptr() as *const c_char,
                    n_cols as c_int,
                    n_rows as c_int,
                    rows.rowtype.as_ptr(),
                    rows.rhs.as_ptr(),
                    rows.range.as_ptr(),
                    obj.as_ptr(),
                    matrix.start.as_ptr(),
                    matrix.length.as_ptr(),
                    matrix.index.as_ptr(),
                    matrix.value.as_ptr(),
                    lb.as_ptr(),
                    ub.as_ptr(),
                    qcol.len() as c_int,
                    qcol.as_ptr(),
                    qcol.as_ptr(),
                    qcoef.as_ptr(),
                ),
                _ => (api.load_lp)(
                    prob,
                    prob_name.as_ptr() as *const c_char,
                    n_cols as c_int,
                    n_rows as c_int,
                    rows.rowtype.as_ptr(),
                    rows.rhs.as_ptr(),
                    rows.range.as_ptr(),
                    obj.as_ptr(),
                    matrix.start.as_ptr(),
                    matrix.length.as_ptr(),
                    matrix.index.as_ptr(),
                    matrix.value.as_ptr(),
                    lb.as_ptr(),
                    ub.as_ptr(),
                ),
            };
            if load_rc != 0 {
                let _ = (api.destroy_prob)(prob);
                return Err(format!("모델 적재가 {load_rc} 를 돌려줬다"));
            }

            let opt_rc = (api.lp_optimize)(prob, b"\0".as_ptr() as *const c_char);

            let mut status_code: c_int = 0;
            let attr_rc = (api.get_int_attrib)(prob, XPRS_LPSTATUS, &mut status_code);
            let status = if attr_rc == 0 {
                map_status(status_code)
            } else {
                log::warn!(
                    "[LP] {NAME}: XPRSgetintattrib(XPRS_LPSTATUS={XPRS_LPSTATUS}) 가 {attr_rc} 를 \
                     돌려줬다 — 속성 번호를 설치된 xprs.h 와 대조할 것"
                );
                LpStatus::Failed
            };

            let mut objective: c_double = f64::NAN;
            let _ = (api.get_dbl_attrib)(prob, XPRS_LPOBJVAL, &mut objective);

            // Infeasible/Unbounded 에서 Xpress 가 주는 x 는 해가 아니라 증명서(ray)다.
            let mut x = vec![0.0f64; n_cols];
            let usable = matches!(status, LpStatus::Optimal | LpStatus::Limit) && n_cols > 0;
            if !usable
                || (api.get_lp_sol)(
                    prob,
                    x.as_mut_ptr(),
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                ) != 0
            {
                x.clear();
            }

            let _ = (api.destroy_prob)(prob);

            if opt_rc != 0 && status == LpStatus::Failed {
                return Err(format!("XPRSlpoptimize 가 {opt_rc} 를 돌려줬다"));
            }

            LpSolution {
                status,
                x,
                objective,
                solved_by: NAME,
            }
        };

        Ok(solution)
    }
}

impl LpBackend for XpressBackend {
    fn name(&self) -> &'static str {
        NAME
    }

    fn availability(&self) -> Result<(), String> {
        api().map(|_| ())
    }

    /// Xpress 는 QP 를 푼다. 실제로 2차항을 쓸 수 있는지는 `XPRSloadqp` 심볼 유무에
    /// 달렸지만, 그건 로딩된 뒤에야 알 수 있으므로 여기서는 계약대로 `true` 다.
    fn supports_quadratic(&self) -> bool {
        true
    }

    fn solve(&self, problem: &LpProblem, time_limit: Duration) -> LpSolution {
        if let Err(why) = problem.validate() {
            log::warn!("[LP] {NAME}: 모델이 유효하지 않다 — {why}");
            return LpSolution::failed(NAME);
        }

        let api = match api() {
            Ok(api) => api,
            Err(why) => {
                log::warn!("[LP] {NAME} 를 쓸 수 없다 — {why}");
                return LpSolution::failed(NAME);
            }
        };

        match Self::solve_inner(api, problem, time_limit) {
            Ok(sol) => sol.checked(problem),
            Err(why) => {
                log::warn!("[LP] {NAME} 풀이 실패 — {why}");
                LpSolution::failed(NAME)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lp_solver::LpRow;

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
    fn one_sided_and_equality_rows_map_to_the_expected_xpress_row_types() {
        let problem = LpProblem {
            objective: vec![0.0, 0.0],
            quadratic: 0.0,
            lower: vec![0.0; 2],
            upper: vec![1.0; 2],
            rows: vec![
                // >= 2  ->  'G'
                LpRow::greater_or_equal(vec![0], vec![1.0], 2.0),
                // <= 7  ->  'L'
                LpRow::less_or_equal(vec![1], vec![1.0], 7.0),
                // == 3  ->  'E'
                LpRow {
                    cols: vec![0],
                    vals: vec![1.0],
                    lower: 3.0,
                    upper: 3.0,
                },
                // 1 <= a'x <= 4  ->  'R', rhs = 4, range = 3
                LpRow {
                    cols: vec![1],
                    vals: vec![1.0],
                    lower: 1.0,
                    upper: 4.0,
                },
                // 자유 행  ->  'N'
                LpRow {
                    cols: vec![0],
                    vals: vec![1.0],
                    lower: f64::NEG_INFINITY,
                    upper: f64::INFINITY,
                },
            ],
        };

        let spec = row_specs(&problem);
        let types: Vec<u8> = spec.rowtype.iter().map(|&c| c as u8).collect();
        assert_eq!(types, b"GLERN".to_vec());
        assert_eq!(spec.rhs, vec![2.0, 7.0, 3.0, 4.0, 0.0]);
        assert_eq!(spec.range, vec![0.0, 0.0, 0.0, 3.0, 0.0]);
    }

    #[test]
    fn free_rows_are_kept_so_row_indices_stay_aligned_with_the_csc_matrix() {
        let problem = LpProblem {
            objective: vec![0.0],
            quadratic: 0.0,
            lower: vec![0.0],
            upper: vec![1.0],
            rows: vec![
                LpRow {
                    cols: vec![0],
                    vals: vec![1.0],
                    lower: f64::NEG_INFINITY,
                    upper: f64::INFINITY,
                },
                LpRow::greater_or_equal(vec![0], vec![1.0], 1.0),
            ],
        };
        let spec = row_specs(&problem);
        assert_eq!(spec.rowtype.len(), problem.n_rows(), "행을 버리면 안 된다");
        let csc = to_column_major(&problem).expect(crate::dmsg!("전치가 성공해야 한다"));
        // 두 번째 행(인덱스 1)이 CSC 에도 그대로 남아 있어야 한다.
        assert_eq!(csc.index, vec![0, 1]);
    }

    #[test]
    fn a_diagonal_regularization_becomes_one_triplet_per_variable() {
        let mut qp = tiny_lp();
        qp.quadratic = 0.25;
        let (cols, coefs) = diagonal_quadratic_terms(&qp);
        assert_eq!(cols, vec![0, 1], "대각이므로 col1 == col2 == i");
        assert_eq!(coefs, vec![0.25, 0.25]);
        assert_eq!(cols.len(), qp.n_vars(), "변수마다 정확히 하나");
    }

    #[test]
    fn a_zero_or_non_finite_regularization_produces_no_quadratic_terms_at_all() {
        // 빈 삼중항이 곧 "LP 로 풀어라"는 신호다. 0 짜리 항을 n_vars 개 넘기면
        // Xpress 가 구조적으로 QP 로 보고 불필요하게 내점법을 돌린다.
        for mu in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let mut qp = tiny_lp();
            qp.quadratic = mu;
            let (cols, coefs) = diagonal_quadratic_terms(&qp);
            assert!(cols.is_empty() && coefs.is_empty(), "mu = {mu}");
        }
    }

    #[test]
    fn a_range_row_that_would_overflow_the_infinity_threshold_is_clamped() {
        // 상하한이 각각 문턱 바로 안쪽이면 폭이 2e20 — 자르지 않으면 Xpress 가
        // 이 범위 행을 무한 범위로 읽어 하한이 사라진다.
        let problem = LpProblem {
            objective: vec![0.0],
            quadratic: 0.0,
            lower: vec![0.0],
            upper: vec![1.0],
            rows: vec![LpRow {
                cols: vec![0],
                vals: vec![1.0],
                lower: -9.9e19,
                upper: 9.9e19,
            }],
        };
        let spec = row_specs(&problem);
        assert_eq!(spec.rowtype.first().map(|&c| c as u8), Some(b'R'));
        assert!(
            spec.range.first().is_some_and(|&g| g <= XPRS_INFINITY),
            "범위 폭이 무한 문턱을 넘으면 안 된다: {:?}",
            spec.range
        );
    }

    #[test]
    fn the_environment_override_is_tried_before_the_standard_file_names() {
        let paths = candidate_paths_from(Some("/opt/xpress/lib/libxprs.so"));
        assert_eq!(
            paths.first().map(String::as_str),
            Some("/opt/xpress/lib/libxprs.so"),
            "환경변수 경로가 첫 후보여야 한다"
        );
        assert_eq!(paths.len(), CANDIDATE_NAMES.len() + 1);
        assert_eq!(
            paths.get(1).map(String::as_str),
            CANDIDATE_NAMES.first().copied()
        );
    }

    #[test]
    fn a_blank_environment_override_is_ignored_instead_of_being_opened_as_a_path() {
        for blank in [None, Some(""), Some("  \t ")] {
            assert_eq!(
                candidate_paths_from(blank).len(),
                CANDIDATE_NAMES.len(),
                "{blank:?} 는 후보가 되면 안 된다"
            );
        }
    }

    #[test]
    fn availability_names_every_file_it_looked_for_when_the_library_is_missing() {
        match XpressBackend.availability() {
            // 이 환경에 Xpress 가 깔려 있고 라이선스도 산다 — 검사할 실패 문자열이 없다.
            Ok(()) => {}
            Err(reason) => {
                for name in CANDIDATE_NAMES {
                    assert!(
                        reason.contains(name),
                        "사유에 시도한 파일명 {name} 이 없다: {reason}"
                    );
                }
                assert!(
                    reason.contains(ENV_OVERRIDE),
                    "사유에 환경변수 이름이 없다: {reason}"
                );
            }
        }
    }

    #[test]
    fn solving_without_the_library_reports_failed_instead_of_panicking() {
        let sol = XpressBackend.solve(&tiny_lp(), Duration::from_millis(200));
        if XpressBackend.availability().is_err() {
            assert_eq!(sol.status, LpStatus::Failed);
            assert!(sol.x.is_empty());
            assert_eq!(sol.solved_by, NAME);
        } else {
            assert_eq!(sol.status, LpStatus::Optimal);
        }
    }

    #[test]
    fn a_quadratic_request_without_the_library_still_fails_quietly() {
        let mut qp = tiny_lp();
        qp.quadratic = 0.25;
        let sol = XpressBackend.solve(&qp, Duration::from_millis(200));
        if XpressBackend.availability().is_err() {
            assert_eq!(sol.status, LpStatus::Failed);
        }
    }

    #[test]
    fn an_invalid_model_never_reaches_the_vendor_library() {
        let broken = LpProblem::default();
        let sol = XpressBackend.solve(&broken, Duration::from_millis(200));
        assert_eq!(sol.status, LpStatus::Failed);
        assert!(sol.x.is_empty());

        // 범위 밖 열 참조도 마찬가지다.
        let mut bad_col = tiny_lp();
        bad_col.rows[0].cols = vec![0, 42];
        assert_eq!(
            XpressBackend
                .solve(&bad_col, Duration::from_millis(200))
                .status,
            LpStatus::Failed
        );
    }

    #[test]
    fn xpress_lpstatus_codes_map_to_the_documented_outcomes() {
        assert_eq!(map_status(1), LpStatus::Optimal);
        assert_eq!(map_status(2), LpStatus::Infeasible);
        assert_eq!(map_status(5), LpStatus::Unbounded);
        assert_eq!(map_status(3), LpStatus::Limit);
        assert_eq!(map_status(4), LpStatus::Limit);
        assert_eq!(map_status(0), LpStatus::Failed);
        assert_eq!(map_status(8), LpStatus::Failed);
    }

    #[test]
    fn xpress_declares_itself_quadratic_capable() {
        assert!(XpressBackend.supports_quadratic());
        assert_eq!(XpressBackend.name(), NAME);
    }
}
