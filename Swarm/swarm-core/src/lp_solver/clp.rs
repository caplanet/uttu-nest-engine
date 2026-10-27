//! COIN-OR Clp 백엔드 — C ABI 를 **런타임에** 동적 로딩한다.
//!
//! # 왜 링크하지 않고 동적 로딩하는가
//!
//! Clp 를 `Cargo.toml` 에 박으려면 빌드 머신마다 COIN-OR SDK(헤더 + import 라이브러리)가
//! 있어야 한다. 이 저장소는 CI·개발자 PC·WASM 까지 같은 소스로 빌드하므로, SDK 하나 때문에
//! **모든 환경에서 빌드가 깨지는** 쪽이 훨씬 비싸다. 그래서 `libloading` 으로 실행 시점에
//! `clp.dll` / `libClp.so` 를 찾고, 없으면 [`super::backend_for`] 가 microlp 로 물러난다.
//!
//! 결과적으로 이 파일은 **SDK 가 없어도 컴파일되고 테스트가 돈다**. 라이브러리를 실행 파일
//! 옆이나 `PATH` 에 두는 순간부터 동작이 켜진다.
//!
//! # 이 파일에서 실제로 검증 가능한 로직
//!
//! FFI 는 라이브러리 없이는 실행할 수 없지만, [`to_column_major`] 는 순수 함수다.
//! [`super::LpProblem`] 은 **행 우선**이고 `Clp_loadProblem` 은 **열 우선 CSC** 를 받으므로
//! 전치가 필요한데, 여기서 틀리면 벤더 라이브러리 안에서 조용히 엉뚱한 답이 나오거나
//! 프로세스가 죽는다. 그래서 전치는 별도 함수로 빼고 손으로 계산한 예제로 직접 검증한다.
//!
//! # 안전성 방침
//!
//! * 넘기기 전에 [`super::LpProblem::validate`] 로 배열 길이·범위를 전부 확인한다.
//!   깨진 배열을 C 로 넘기면 복구할 수 없다.
//! * 어떤 경로에서도 panic 하지 않는다. 실패는 로그 + [`LpSolution::failed`] 다.

use std::ffi::{c_double, c_int, c_void};
use std::sync::OnceLock;
use std::time::Duration;

use libloading::Library;

use super::{LpBackend, LpProblem, LpSolution, LpStatus};

/// 로그·`solved_by` 에 나갈 이름.
const NAME: &str = "COIN-OR Clp";

/// 이 값을 먼저 시도한다. 벤더 배포판이 특이한 위치에 있을 때의 탈출구다.
const ENV_OVERRIDE: &str = "SWARM_CLP_LIB";

/// 플랫폼별 표준 파일명. 위에서부터 순서대로 시도한다.
///
/// Windows 배포판이 `clp.dll` 과 `libClp.dll` 두 이름을 모두 쓰기 때문에 둘 다 넣었다.
/// `libClp.so.1` 은 개발 심볼릭 링크(`libClp.so`)를 깔지 않은 런타임 전용 패키지용이다.
const CANDIDATE_NAMES: &[&str] = &[
    "clp.dll",
    "libClp.dll",
    "libClp.so",
    "libClp.so.1",
    "libClp.dylib",
];

/// COIN 계열이 무한대로 취급하는 경계값.
///
/// COIN 내부는 `1e30` 초과를 무한으로 본다. `f64::INFINITY` 를 그대로 넘겨도 대개 통하지만,
/// 일부 빌드가 무한대로 산술을 해 NaN 을 만드는 사례가 보고돼 있어 잘라서 넘긴다.
const COIN_INFINITY: f64 = 1.0e30;

// ────────────────────────────────────────────────────────────────────────────
// 행 우선 -> 열 우선(CSC) 전치
// ────────────────────────────────────────────────────────────────────────────

/// 열 우선(CSC) 희소 행렬. `Clp_loadProblem` 과 `XPRSloadlp` 가 둘 다 이 형태를 받는다.
///
/// 열 `j` 의 원소는 `index[start[j] .. start[j] + length[j]]` 이고, 그 안에서 행 인덱스는
/// **오름차순**이다. `start[j + 1] == start[j] + length[j]` 가 항상 성립하므로 두 배열이
/// 중복돼 보이지만, Clp 는 `start` 를, Xpress 는 `start` + `collen` 을 요구해서 둘 다 만든다.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ColumnMajor {
    /// 열 시작 오프셋. 길이는 `n_cols + 1`, 마지막 원소가 총 비영 개수다.
    pub start: Vec<c_int>,
    /// 열별 비영 개수. 길이는 `n_cols`.
    pub length: Vec<c_int>,
    /// 행 인덱스. 열 안에서 오름차순.
    pub index: Vec<c_int>,
    /// 같은 순서의 계수.
    pub value: Vec<f64>,
}

/// [`super::LpProblem`]의 행 우선 제약을 열 우선 CSC 로 전치한다.
///
/// 계수 정렬(counting sort)이라 비영 개수에 선형이다. 행을 0번부터 훑으므로 각 열 안의 행
/// 인덱스는 자연히 오름차순이 되고, 그래서 **같은 (행, 열)에 두 번 들어온 계수는 서로 붙어
/// 있다** — 그걸 한 번의 선형 훑기로 더해서 합친다. 벤더 라이브러리는 중복 항목을 만나면
/// 조용히 하나만 쓰거나 진단 없이 실패하므로, 넘기기 전에 여기서 정리한다.
///
/// 값이 0 인 항목은 **지우지 않는다**. 구조적 비영(structural nonzero)은 기저 선택에 영향을
/// 주고, 압축 모델은 계수가 0 인 열을 일부러 남기는 경우가 있다.
///
/// # 오류
///
/// 비영 개수나 차원이 `c_int` 범위를 넘으면 `Err` 다. C API 가 32비트 인덱스를 쓰므로
/// 넘겨 봐야 조용히 잘려서 메모리를 깨뜨린다.
pub(crate) fn to_column_major(problem: &LpProblem) -> Result<ColumnMajor, String> {
    let n_cols = problem.n_vars();
    let n_rows = problem.n_rows();
    let limit = c_int::MAX as usize;
    if n_cols > limit || n_rows > limit {
        return Err(format!(
            "문제가 32비트 인덱스를 넘는다: 열 {n_cols}, 행 {n_rows}"
        ));
    }

    // 1단계 — 열별 비영 개수를 센다.
    let mut counts = vec![0usize; n_cols];
    let mut nnz = 0usize;
    for row in &problem.rows {
        for (&col, _) in row.cols.iter().zip(row.vals.iter()) {
            match counts.get_mut(col) {
                Some(slot) => {
                    *slot += 1;
                    nnz += 1;
                }
                // validate() 가 이미 걸렀어야 하는 상황이다. 여기까지 왔다면 호출부가
                // validate() 를 건너뛴 것이므로, 죽지 말고 사유를 돌려준다.
                None => return Err(format!("제약이 범위 밖 열 {col} 을 참조한다")),
            }
        }
    }
    if nnz > limit {
        return Err(format!("비영 개수 {nnz} 가 32비트 인덱스를 넘는다"));
    }

    // 2단계 — 누적합으로 버킷 경계를 만든다.
    let mut bucket = Vec::with_capacity(n_cols + 1);
    let mut acc = 0usize;
    bucket.push(0usize);
    for &c in &counts {
        acc += c;
        bucket.push(acc);
    }

    // 3단계 — 행을 0번부터 훑으며 버킷에 채운다. 열 안의 행 인덱스가 오름차순이 된다.
    let mut cursor: Vec<usize> = bucket.iter().take(n_cols).copied().collect();
    let mut tmp_index = vec![0usize; nnz];
    let mut tmp_value = vec![0.0f64; nnz];
    for (r, row) in problem.rows.iter().enumerate() {
        for (&col, &val) in row.cols.iter().zip(row.vals.iter()) {
            let Some(slot) = cursor.get_mut(col) else {
                return Err(format!("제약이 범위 밖 열 {col} 을 참조한다"));
            };
            let at = *slot;
            match (tmp_index.get_mut(at), tmp_value.get_mut(at)) {
                (Some(i), Some(v)) => {
                    *i = r;
                    *v = val;
                }
                _ => return Err(format!("전치 버퍼 인덱스 {at} 가 범위를 벗어났다")),
            }
            *slot = at + 1;
        }
    }

    // 4단계 — 열 단위로 중복 (행, 열) 항목을 합치며 최종 배열을 만든다.
    let mut start = Vec::with_capacity(n_cols + 1);
    let mut length = Vec::with_capacity(n_cols);
    let mut index = Vec::with_capacity(nnz);
    let mut value = Vec::with_capacity(nnz);
    for col in 0..n_cols {
        let (Some(&lo), Some(&hi)) = (bucket.get(col), bucket.get(col + 1)) else {
            return Err(format!("열 {col} 의 버킷 경계가 없다"));
        };
        start.push(index.len() as c_int);

        let mut k = lo;
        while k < hi {
            let (Some(&row), Some(&first)) = (tmp_index.get(k), tmp_value.get(k)) else {
                return Err(format!("전치 버퍼 인덱스 {k} 가 범위를 벗어났다"));
            };
            let mut sum = first;
            let mut j = k + 1;
            // 같은 행이 연속으로 나오면 계수를 더해 하나로 만든다.
            while j < hi && tmp_index.get(j) == Some(&row) {
                sum += tmp_value.get(j).copied().unwrap_or(0.0);
                j += 1;
            }
            index.push(row as c_int);
            value.push(sum);
            k = j;
        }

        let written = index.len() as c_int - start.last().copied().unwrap_or(0);
        length.push(written);
    }
    start.push(index.len() as c_int);

    Ok(ColumnMajor {
        start,
        length,
        index,
        value,
    })
}

/// 무한대를 벤더가 이해하는 유한한 큰 수로 자른다.
///
/// NaN 은 [`super::LpProblem::validate`] 가 이미 걸러서 여기 도달할 수 없다. 그래도
/// 방어적으로 처리하는 이유는, NaN 하나가 벤더 라이브러리 안으로 들어가면 진단 없이
/// 전체 해를 오염시키기 때문이다. 부호 비트를 따라 ±`infinity` 로 눕힌다.
///
/// 부호가 "조이는" 쪽이면(예: 하한이 +`infinity` 가 되면) 경계가 뒤집혀 **실행 불가능**으로
/// 보고된다. 자유 변수가 되는 게 아니다 — 그게 의도다. 조용히 틀린 해를 받는 것보다
/// 눈에 띄게 실패하는 편이 낫다.
pub(crate) fn clamp_infinity(v: f64, infinity: f64) -> f64 {
    if v.is_nan() {
        return if v.is_sign_negative() {
            -infinity
        } else {
            infinity
        };
    }
    v.clamp(-infinity, infinity)
}

// ────────────────────────────────────────────────────────────────────────────
// C ABI 심볼
// ────────────────────────────────────────────────────────────────────────────

type FnNewModel = unsafe extern "C" fn() -> *mut c_void;
type FnDeleteModel = unsafe extern "C" fn(*mut c_void);
/// `start` 는 `CoinBigIndex*` 다.
///
/// **검증 필요**: `CoinBigIndex` 는 기본 빌드에서 `int` 지만, `--enable-coinbigindex`
/// 로 빌드한 배포판에서는 `long long` 이다. 그런 배포판을 물리면 `start` 배열이 어긋나
/// 엉뚱한 결과가 나온다. 대다수 공식 바이너리가 `int` 라 `c_int` 로 갔다.
type FnLoadProblem = unsafe extern "C" fn(
    *mut c_void,
    c_int,
    c_int,
    *const c_int,
    *const c_int,
    *const c_double,
    *const c_double,
    *const c_double,
    *const c_double,
    *const c_double,
    *const c_double,
);
type FnSetDirection = unsafe extern "C" fn(*mut c_void, c_double);
type FnSetLogLevel = unsafe extern "C" fn(*mut c_void, c_int);
type FnSetMaximumSeconds = unsafe extern "C" fn(*mut c_void, c_double);
type FnInitialSolve = unsafe extern "C" fn(*mut c_void) -> c_int;
type FnStatus = unsafe extern "C" fn(*mut c_void) -> c_int;
type FnObjectiveValue = unsafe extern "C" fn(*mut c_void) -> c_double;
type FnGetColSolution = unsafe extern "C" fn(*mut c_void) -> *mut c_double;
type FnNumberColumns = unsafe extern "C" fn(*mut c_void) -> c_int;

/// 로딩된 Clp 진입점 묶음.
///
/// `_lib` 를 마지막 필드로 둔다. 구조체 필드는 선언 순서로 drop 되므로, 함수 포인터가
/// 먼저 사라지고 라이브러리가 마지막에 닫히는 순서를 눈으로 보이게 하려는 것이다
/// (함수 포인터에는 drop 이 없어 실제 안전성에는 영향이 없다).
struct ClpApi {
    new_model: FnNewModel,
    delete_model: FnDeleteModel,
    load_problem: FnLoadProblem,
    set_direction: FnSetDirection,
    initial_solve: FnInitialSolve,
    status: FnStatus,
    objective_value: FnObjectiveValue,
    get_col_solution: FnGetColSolution,
    /// 오래된 배포판에 없을 수 있다. 없으면 로그가 시끄러워질 뿐이라 선택 사항이다.
    set_log_level: Option<FnSetLogLevel>,
    /// 없으면 제한 시간을 못 건다. 그것 때문에 백엔드를 통째로 포기할 이유는 없다.
    set_maximum_seconds: Option<FnSetMaximumSeconds>,
    /// 적재된 모델의 실제 열 개수. `Clp_loadProblem` 이 반환값 없이 실패했는지 알아내는
    /// **유일한** 수단이라 중요하지만, 없는 배포판에서 백엔드를 포기할 정도는 아니라
    /// 선택 사항으로 둔다. 없으면 이 검사를 건너뛴다.
    number_columns: Option<FnNumberColumns>,
    _lib: Library,
}

/// 심볼 하나를 함수 포인터로 꺼낸다.
///
/// # 안전성
///
/// 호출부가 `T` 를 실제 심볼의 C 시그니처와 정확히 맞춰야 한다. 이 모듈 안에서만 쓰고,
/// 타입 별칭은 위쪽 주석에 원본 선언을 그대로 적어 두었다.
fn load_symbol<T: Copy>(lib: &Library, name: &[u8]) -> Result<T, String> {
    // SAFETY: name 은 이 파일에 하드코딩된 NUL 종료 심볼명이고, T 는 바로 위에 원본
    // C 선언을 적어 둔 함수 포인터 타입이다.
    unsafe { lib.get::<T>(name) }.map(|sym| *sym).map_err(|e| {
        format!(
            "심볼 {} 없음 ({e})",
            String::from_utf8_lossy(name).trim_end_matches('\0')
        )
    })
}

impl ClpApi {
    /// 열린 라이브러리에서 필요한 심볼을 전부 꺼낸다.
    ///
    /// 필수 심볼이 하나라도 없으면 "Clp 가 아닌 다른 DLL 을 물었다"는 뜻이므로 실패다.
    fn from_library(lib: Library) -> Result<Self, String> {
        Ok(Self {
            new_model: load_symbol(&lib, b"Clp_newModel\0")?,
            delete_model: load_symbol(&lib, b"Clp_deleteModel\0")?,
            load_problem: load_symbol(&lib, b"Clp_loadProblem\0")?,
            set_direction: load_symbol(&lib, b"Clp_setOptimizationDirection\0")?,
            initial_solve: load_symbol(&lib, b"Clp_initialSolve\0")?,
            status: load_symbol(&lib, b"Clp_status\0")?,
            objective_value: load_symbol(&lib, b"Clp_objectiveValue\0")?,
            get_col_solution: load_symbol(&lib, b"Clp_getColSolution\0")?,
            set_log_level: load_symbol(&lib, b"Clp_setLogLevel\0").ok(),
            set_maximum_seconds: load_symbol(&lib, b"Clp_setMaximumSeconds\0").ok(),
            number_columns: load_symbol(&lib, b"Clp_numberColumns\0").ok(),
            _lib: lib,
        })
    }
}

/// 시도할 경로 목록을 만든다. `override_path` 가 있으면 **가장 먼저** 온다.
///
/// 환경변수를 읽는 일과 순서를 정하는 일을 나눈 이유는, "환경변수가 표준 파일명보다
/// 먼저"라는 규칙이 라이브러리 없이 검증할 수 있는 몇 안 되는 정책인데
/// `std::env::set_var` 는 2024 에디션에서 `unsafe` 고 프로세스 전역 상태라 병렬 테스트가
/// 서로를 깨뜨리기 때문이다. 순수 함수로 빼면 환경을 만지지 않고 그대로 확인된다.
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

/// 후보를 순서대로 열어 본다. 전부 실패하면 **무엇을 어디서 찾았는지** 담은 사유를 돌린다.
fn load_api() -> Result<ClpApi, String> {
    let mut tried: Vec<String> = Vec::new();
    for cand in candidate_paths() {
        // SAFETY: 라이브러리 로딩은 대상 DLL 의 초기화 코드를 실행한다. 사용자가 지정하거나
        // 표준 이름으로 찾은 Clp 배포판만 대상이므로 이 크레이트가 통제할 수 있는 위험이 아니다.
        match unsafe { Library::new(&cand) } {
            Ok(lib) => match ClpApi::from_library(lib) {
                Ok(api) => return Ok(api),
                Err(why) => tried.push(format!("{cand} ({why})")),
            },
            Err(e) => tried.push(format!("{cand} ({e})")),
        }
    }
    Err(format!(
        "{NAME} 동적 라이브러리를 열지 못했다. 시도한 파일: {}. \
         {ENV_OVERRIDE} 에 전체 경로를 주거나, 파일을 실행 파일 옆 또는 \
         PATH/LD_LIBRARY_PATH 에 두면 된다.",
        tried.join(", ")
    ))
}

/// 프로세스당 한 번만 로딩한다.
///
/// 실패도 캐시한다 — 없는 DLL 을 LP 를 풀 때마다 다시 찾으면 압축 단계가 눈에 띄게 느려진다.
/// 대신 실행 중에 DLL 을 놓아도 재시작 전에는 잡히지 않는다. 그 편이 낫다고 봤다.
static CLP_API: OnceLock<Result<ClpApi, String>> = OnceLock::new();

/// 캐시된 API 를 얻거나 실패 사유를 복제해 돌린다.
fn api() -> Result<&'static ClpApi, String> {
    match CLP_API.get_or_init(load_api) {
        Ok(api) => Ok(api),
        Err(why) => Err(why.clone()),
    }
}

/// `Clp_status` 반환값 -> [`LpStatus`].
///
/// 0 최적, 1 primal infeasible, 2 dual infeasible(= 목적이 무한히 좋아짐), 3 중단.
/// 그 밖(4 = 오류, -1 = 아직 안 풂)은 전부 실패로 본다.
fn map_status(code: c_int) -> LpStatus {
    match code {
        0 => LpStatus::Optimal,
        1 => LpStatus::Infeasible,
        2 => LpStatus::Unbounded,
        3 => LpStatus::Limit,
        _ => LpStatus::Failed,
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 백엔드
// ────────────────────────────────────────────────────────────────────────────

/// COIN-OR Clp 단체법/내점법 백엔드. LP 전용.
///
/// 상태가 없다 — 로딩된 라이브러리는 프로세스 전역([`CLP_API`])에 한 번만 산다.
#[derive(Debug, Clone, Copy, Default)]
pub struct ClpBackend;

impl ClpBackend {
    /// 실제 풀이. FFI 를 한 곳에 모아 두려고 [`LpBackend::solve`] 에서 분리했다.
    fn solve_inner(
        api: &ClpApi,
        problem: &LpProblem,
        time_limit: Duration,
    ) -> Result<LpSolution, String> {
        let n_cols = problem.n_vars();
        let n_rows = problem.n_rows();
        let matrix = to_column_major(problem)?;

        // 경계와 행 범위를 COIN 이 이해하는 유한 값으로 자른다.
        let collb: Vec<f64> = problem
            .lower
            .iter()
            .map(|&v| clamp_infinity(v, COIN_INFINITY))
            .collect();
        let colub: Vec<f64> = problem
            .upper
            .iter()
            .map(|&v| clamp_infinity(v, COIN_INFINITY))
            .collect();
        let rowlb: Vec<f64> = problem
            .rows
            .iter()
            .map(|r| clamp_infinity(r.lower, COIN_INFINITY))
            .collect();
        let rowub: Vec<f64> = problem
            .rows
            .iter()
            .map(|r| clamp_infinity(r.upper, COIN_INFINITY))
            .collect();
        let obj = problem.objective.clone();

        // SAFETY: 아래 블록이 넘기는 모든 포인터는 바로 위에서 만든 살아 있는 Vec 이고,
        // 길이는 n_cols / n_rows / matrix 로 서로 맞춰 두었다. validate() 가 길이 불일치와
        // 범위 밖 열 참조를 이미 걸렀다.
        let solution = unsafe {
            let model = (api.new_model)();
            if model.is_null() {
                return Err("Clp_newModel 이 널을 돌려줬다".into());
            }

            // 로그를 먼저 끈다 — loadProblem 이 stdout 을 어지럽히지 않도록.
            if let Some(set_log) = api.set_log_level {
                set_log(model, 0);
            }

            (api.load_problem)(
                model,
                n_cols as c_int,
                n_rows as c_int,
                matrix.start.as_ptr(),
                matrix.index.as_ptr(),
                matrix.value.as_ptr(),
                collb.as_ptr(),
                colub.as_ptr(),
                obj.as_ptr(),
                rowlb.as_ptr(),
                rowub.as_ptr(),
            );

            // 1.0 = 최소화. 압축 모델은 항상 최소화다.
            (api.set_direction)(model, 1.0);

            if let Some(set_secs) = api.set_maximum_seconds {
                // 0 이나 음수를 넘기면 배포판에 따라 "제한 없음"으로 읽힌다. 1ms 를 하한으로 둔다.
                let secs = time_limit.as_secs_f64().max(0.001);
                set_secs(model, secs);
            }

            let _rc = (api.initial_solve)(model);
            let status = map_status((api.status)(model));
            let objective = (api.objective_value)(model);

            // `Clp_loadProblem` 은 **반환값이 없다** — start 배열이 어긋나거나 메모리가
            // 모자라 적재가 실패해도 알려 주지 않는다. 그런 모델의 `Clp_getColSolution` 은
            // n_cols 보다 짧은 배열(또는 이전 모델의 잔해)을 가리키므로, 그걸 그대로
            // n_cols 개 읽으면 범위 밖 읽기다. 적재된 열 개수를 되물어 확인한다.
            let loaded_cols = match api.number_columns {
                Some(count) => count(model),
                // 심볼이 없으면 확인할 방법이 없다. 예전처럼 믿고 간다.
                None => n_cols as c_int,
            };
            let loaded_matches = i64::from(loaded_cols) == n_cols as i64;

            let x = if status == LpStatus::Failed || n_cols == 0 || !loaded_matches {
                Vec::new()
            } else {
                let ptr = (api.get_col_solution)(model);
                if ptr.is_null() {
                    Vec::new()
                } else {
                    // 여기까지 왔으면 Clp 가 n_cols 개의 double 을 보장한다.
                    // 모델을 지우기 전에 복사한다.
                    std::slice::from_raw_parts(ptr as *const f64, n_cols).to_vec()
                }
            };

            (api.delete_model)(model);

            // 열 개수가 어긋났다면 상태가 Optimal 이어도 그 해는 우리 모델의 해가 아니다.
            // 조용히 빈 해를 Optimal 로 넘기면 호출부가 그걸 채택해 버리므로 실패로 올린다.
            if !loaded_matches {
                return Err(format!(
                    "Clp 가 적재한 열 개수 {loaded_cols} 가 모델의 {n_cols} 과 다르다 \
                     — Clp_loadProblem 이 조용히 실패했다"
                ));
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

impl LpBackend for ClpBackend {
    fn name(&self) -> &'static str {
        NAME
    }

    fn availability(&self) -> Result<(), String> {
        api().map(|_| ())
    }

    /// Clp 는 순수 LP 솔버다. 2차 목적항은 못 푼다.
    fn supports_quadratic(&self) -> bool {
        false
    }

    fn solve(&self, problem: &LpProblem, time_limit: Duration) -> LpSolution {
        if let Err(why) = problem.validate() {
            log::warn!("[LP] {NAME}: 모델이 유효하지 않다 — {why}");
            return LpSolution::failed(NAME);
        }
        if problem.quadratic > 0.0 {
            log::warn!(
                "[LP] {NAME} 는 LP 전용이다 — 2차 계수 {}을 버리고 1차 목적만 푼다",
                problem.quadratic
            );
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

    /// ```text
    ///      c0   c1   c2
    /// r0    1    .    3
    /// r1    .    5    .
    /// r2    7    8    9
    /// ```
    fn hand_computed_3x3() -> LpProblem {
        LpProblem {
            objective: vec![0.0, 0.0, 0.0],
            quadratic: 0.0,
            lower: vec![0.0; 3],
            upper: vec![1.0; 3],
            rows: vec![
                LpRow::greater_or_equal(vec![0, 2], vec![1.0, 3.0], 0.0),
                LpRow::greater_or_equal(vec![1], vec![5.0], 0.0),
                LpRow::greater_or_equal(vec![0, 1, 2], vec![7.0, 8.0, 9.0], 0.0),
            ],
        }
    }

    #[test]
    fn transposing_a_hand_computed_matrix_gives_the_expected_csc_arrays() {
        let csc = to_column_major(&hand_computed_3x3()).expect(crate::dmsg!("전치가 성공해야 한다"));

        // 열0: (r0,1) (r2,7) / 열1: (r1,5) (r2,8) / 열2: (r0,3) (r2,9)
        assert_eq!(csc.start, vec![0, 2, 4, 6], "열 시작 오프셋");
        assert_eq!(csc.length, vec![2, 2, 2], "열별 비영 개수");
        assert_eq!(csc.index, vec![0, 2, 1, 2, 0, 2], "행 인덱스");
        assert_eq!(csc.value, vec![1.0, 7.0, 5.0, 8.0, 3.0, 9.0], "계수");
    }

    #[test]
    fn every_column_slice_is_sorted_by_row_and_starts_where_the_previous_one_ended() {
        let csc = to_column_major(&hand_computed_3x3()).expect(crate::dmsg!("전치가 성공해야 한다"));
        assert_eq!(csc.start.len(), 4, "열 개수 + 1");
        assert_eq!(
            csc.start.last().copied(),
            Some(csc.index.len() as c_int),
            "마지막 오프셋은 총 비영 개수다"
        );
        for col in 0..3usize {
            let lo = csc.start[col] as usize;
            let len = csc.length[col] as usize;
            assert_eq!(
                lo + len,
                csc.start[col + 1] as usize,
                "열 {col} 의 start + length 는 다음 열 start 와 같아야 한다"
            );
            let slice = &csc.index[lo..lo + len];
            assert!(
                slice.windows(2).all(|w| w[0] < w[1]),
                "열 {col} 의 행 인덱스가 오름차순이 아니다: {slice:?}"
            );
        }
    }

    #[test]
    fn duplicate_entries_in_one_row_are_summed_into_a_single_coefficient() {
        // 같은 행에서 열 1 이 두 번 나온다: 2 + 3 = 5 여야 한다.
        let problem = LpProblem {
            objective: vec![0.0, 0.0],
            quadratic: 0.0,
            lower: vec![0.0; 2],
            upper: vec![1.0; 2],
            rows: vec![LpRow::greater_or_equal(
                vec![1, 1, 0],
                vec![2.0, 3.0, 4.0],
                0.0,
            )],
        };
        let csc = to_column_major(&problem).expect(crate::dmsg!("전치가 성공해야 한다"));
        assert_eq!(csc.start, vec![0, 1, 2]);
        assert_eq!(csc.length, vec![1, 1]);
        assert_eq!(csc.index, vec![0, 0]);
        assert_eq!(csc.value, vec![4.0, 5.0], "중복 계수는 더해져야 한다");
    }

    #[test]
    fn a_column_that_no_row_touches_gets_a_zero_length_slice() {
        // 열 1 은 어떤 행에도 안 나온다.
        let problem = LpProblem {
            objective: vec![0.0, 0.0, 0.0],
            quadratic: 0.0,
            lower: vec![0.0; 3],
            upper: vec![1.0; 3],
            rows: vec![LpRow::less_or_equal(vec![0, 2], vec![1.0, 1.0], 1.0)],
        };
        let csc = to_column_major(&problem).expect(crate::dmsg!("전치가 성공해야 한다"));
        assert_eq!(csc.start, vec![0, 1, 1, 2]);
        assert_eq!(csc.length, vec![1, 0, 1], "빈 열은 길이 0");
        assert_eq!(csc.index, vec![0, 0]);
    }

    #[test]
    fn a_problem_with_no_rows_transposes_to_all_empty_columns() {
        let problem = LpProblem {
            objective: vec![1.0, 1.0],
            quadratic: 0.0,
            lower: vec![0.0; 2],
            upper: vec![1.0; 2],
            rows: Vec::new(),
        };
        let csc = to_column_major(&problem).expect(crate::dmsg!("행이 없어도 전치는 성공해야 한다"));
        assert_eq!(csc.start, vec![0, 0, 0]);
        assert_eq!(csc.length, vec![0, 0]);
        assert!(csc.index.is_empty() && csc.value.is_empty());
    }

    #[test]
    fn transposing_rejects_a_column_index_outside_the_variable_range() {
        let mut problem = hand_computed_3x3();
        problem.rows[0].cols = vec![0, 99];
        assert!(
            to_column_major(&problem).is_err(),
            "범위 밖 열은 C 로 넘기기 전에 걸러야 한다"
        );
    }

    #[test]
    fn clamping_replaces_infinities_with_the_coin_sentinel() {
        assert_eq!(clamp_infinity(f64::INFINITY, COIN_INFINITY), COIN_INFINITY);
        assert_eq!(
            clamp_infinity(f64::NEG_INFINITY, COIN_INFINITY),
            -COIN_INFINITY
        );
        assert_eq!(clamp_infinity(3.5, COIN_INFINITY), 3.5);
        assert!(clamp_infinity(f64::NAN, COIN_INFINITY).is_finite());
    }

    #[test]
    fn nan_bounds_collapse_to_a_sentinel_rather_than_reaching_the_vendor_library() {
        // validate() 가 먼저 걸러서 도달할 수 없는 경로지만, NaN 이 C 로 새어 나가면
        // 진단 없이 해가 오염된다. 부호를 따라 눕는지 확인한다.
        assert_eq!(clamp_infinity(f64::NAN, COIN_INFINITY), COIN_INFINITY);
        assert_eq!(clamp_infinity(-f64::NAN, COIN_INFINITY), -COIN_INFINITY);
    }

    #[test]
    fn the_environment_override_is_tried_before_the_standard_file_names() {
        let paths = candidate_paths_from(Some("D:/vendor/clp.dll"));
        assert_eq!(
            paths.first().map(String::as_str),
            Some("D:/vendor/clp.dll"),
            "환경변수 경로가 첫 후보여야 한다"
        );
        assert_eq!(
            paths.len(),
            CANDIDATE_NAMES.len() + 1,
            "표준 이름을 하나도 잃으면 안 된다"
        );
        assert_eq!(
            paths.get(1).map(String::as_str),
            CANDIDATE_NAMES.first().copied(),
            "그 뒤로는 선언된 순서 그대로다"
        );
    }

    #[test]
    fn a_blank_environment_override_is_ignored_instead_of_being_opened_as_a_path() {
        // 빈 문자열을 Library::new 에 넘기면 플랫폼마다 다른 엉뚱한 실패가 나고,
        // 그 문자열이 사유에 섞여 진단을 흐린다.
        for blank in [None, Some(""), Some("   ")] {
            assert_eq!(
                candidate_paths_from(blank).len(),
                CANDIDATE_NAMES.len(),
                "{blank:?} 는 후보가 되면 안 된다"
            );
        }
    }

    #[test]
    fn availability_names_every_file_it_looked_for_when_the_library_is_missing() {
        match ClpBackend.availability() {
            // 이 환경에 실제로 Clp 가 깔려 있다 — 검사할 실패 문자열이 없다.
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
        let problem = LpProblem {
            objective: vec![1.0, 1.0],
            quadratic: 0.0,
            lower: vec![0.0, 0.0],
            upper: vec![10.0, 10.0],
            rows: vec![LpRow::greater_or_equal(vec![0, 1], vec![1.0, 1.0], 2.0)],
        };
        let sol = ClpBackend.solve(&problem, Duration::from_millis(200));
        if ClpBackend.availability().is_err() {
            assert_eq!(sol.status, LpStatus::Failed);
            assert!(sol.x.is_empty());
            assert_eq!(sol.solved_by, NAME);
        } else {
            // 라이브러리가 있으면 진짜로 풀려야 한다.
            assert_eq!(sol.status, LpStatus::Optimal);
        }
    }

    #[test]
    fn an_invalid_model_never_reaches_the_vendor_library() {
        // 변수 0개 — validate() 가 거절한다. 라이브러리 유무와 무관하게 Failed 여야 한다.
        let broken = LpProblem::default();
        let sol = ClpBackend.solve(&broken, Duration::from_millis(200));
        assert_eq!(sol.status, LpStatus::Failed);
        assert!(sol.x.is_empty());
    }

    #[test]
    fn clp_status_codes_map_to_the_documented_outcomes() {
        assert_eq!(map_status(0), LpStatus::Optimal);
        assert_eq!(map_status(1), LpStatus::Infeasible);
        assert_eq!(map_status(2), LpStatus::Unbounded);
        assert_eq!(map_status(3), LpStatus::Limit);
        assert_eq!(map_status(4), LpStatus::Failed);
        assert_eq!(map_status(-1), LpStatus::Failed);
    }

    #[test]
    fn clp_declares_itself_linear_only() {
        assert!(!ClpBackend.supports_quadratic());
        assert_eq!(ClpBackend.name(), NAME);
    }
}
