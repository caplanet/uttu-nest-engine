//! §5 기본 파라미터 표 — **설계 문서의 값을 그대로** 상수로 고정한다.
//!
//! 이 파일의 모든 값은 `레퍼런스-초기배치-설계.md` §5 에서 왔다. 값을 바꾸려면
//! 문서를 먼저 고치고, 그 근거(실측 조건 포함)를 여기 주석에 남긴다.
//!
//! # 왜 상수를 한 파일에 모으나
//!
//! 이 저장소가 반복해 밟은 함정 하나가 "낡은 실측치를 근거로 상수를 고른다" 이다.
//! 근거가 문서 한 곳(설계 §5)에 있고 코드가 그것을 1:1 로 비추면, 값이 어디서 왔는지
//! 되짚을 수 있다. 값마다 문서의 어느 줄인지 적어 둔다.

use std::time::Duration;

// ────────────────────────────────────────────────────────────────────────────
// 개선 루프 (§5)
// ────────────────────────────────────────────────────────────────────────────

/// 반복 상한 (내부 개선). 개선 루프 최대 반복.
pub const IMPROVE_MAX_ITERS: usize = 20;

/// 개선 허용오차. 개선폭이 이 값 미만이면 수렴 (길이 정수 단위).
///
/// 문서가 "길이 정수 단위" 라고 못박는다 — 정규화된 값이 아니다. 이 엔진의 좌표는
/// mm 이므로 100 은 100mm 다.
pub const IMPROVE_TOLERANCE: f64 = 100.0;

/// 이동 임계값. 전 피스 이동이 이 값 미만이면 수렴.
pub const MOVE_THRESHOLD: f64 = 30.0;

// ────────────────────────────────────────────────────────────────────────────
// 회전 (§5)
// ────────────────────────────────────────────────────────────────────────────

/// cos/sin 사전계산 방향 수 = **24** (15도 간격).
///
/// # 이것은 배치 회전이 아니다
///
/// 설계 §모듈 1 이 명시한다 — 이 테이블은 **잔여 조각 LP 의 지지함수 제약**용이다.
/// 배치 단계의 회전은 조각별 [`RotationList`](super::record::RotationList) 를 순회한다.
/// 초판 문서를 읽고 "후보 = NFP 경계 x 24방향" 으로 이해하기 쉬운데, 갱신판이
/// 그 오해를 명시적으로 정정했다.
pub const SUPPORT_DIRECTIONS: usize = 24;

/// 배치 탐색의 **자유회전** 미세 스텝(라디안) = `0.01° = π/18000`.
///
/// 허용 회전이 연속일 때만 쓴다. 다른 단계는 1°·5° 스텝을 쓴다(§5).
pub const FREE_ROTATION_STEP_RAD: f64 = std::f64::consts::PI / 18_000.0;

// ────────────────────────────────────────────────────────────────────────────
// 탐색·버퍼 (§5)
// ────────────────────────────────────────────────────────────────────────────

/// 후보 평가 루프 상한.
///
/// 초과하면 **조용히 절단**한다 — 새 후보 평가만 멈추고 그때까지의 최선을 유지한다.
/// 재정렬도 부분 재개도 없다(§모듈 7 절단 규칙).
pub const PROBE_LIMIT: usize = 10_000;

/// 워커당 후보 배열 선할당. 초과 시 2배 재할당.
pub const CANDIDATE_PREALLOC: usize = 512;

/// 직렬/병렬 임계. 후보가 이 값 미만이면 직렬 처리.
pub const PARALLEL_THRESHOLD: usize = 8;

/// 피스별 평가배열 초기 용량.
pub const EVAL_ARRAY_PREALLOC: usize = 20_000;

// ────────────────────────────────────────────────────────────────────────────
// 점수 (§4-2, §5)
// ────────────────────────────────────────────────────────────────────────────

/// 균형 점수가 보는 표면 수 상한.
pub const MAX_SCORED_SURFACES: usize = 9;

/// 표면 하나가 받는 후보 기여 수 상한.
pub const MAX_CONTRIBUTIONS_PER_SURFACE: usize = 5;

/// 부족률 `d` 의 % 환산 배율.
pub const SHORTFALL_SCALE: f64 = 100.0;

/// `w(d)` 구간 경계 — 이하이면 "최고와 동등" 으로 보아 벌점 0.
pub const WEIGHT_BREAK_LOW: f64 = 0.2;

/// `w(d)` 구간 경계 — 이하이면 `1 - 0.5*d`, 초과 1.0 이하이면 `1.0`.
pub const WEIGHT_BREAK_HIGH: f64 = 0.5;

/// `score_s = v_max - SCORE_S_COEFF * avg_s`.
pub const SCORE_S_COEFF: f64 = 2.0;

/// 최종 점수 `= FINAL_MIN_COEFF * min_s(score_s) - FINAL_VMAX_COEFF * v_max`.
pub const FINAL_MIN_COEFF: f64 = 4.0;

/// 위 식의 감산 계수.
pub const FINAL_VMAX_COEFF: f64 = 3.0;

/// 가중합 하한. `Σw` 가 이 값 이하이면 그 표면은 센티널로 제외한다.
pub const WEIGHT_SUM_FLOOR: f64 = 0.001;

/// 제외된 표면을 나타내는 센티널 값.
///
/// 설계 §4-2 가 `-1.0` 을 명시한다. `f64::NAN` 이 아니라 유한한 음수인 것이 중요하다 —
/// `min_s` 를 취할 때 NaN 은 비교를 통과하지 못해 최솟값이 훑는 순서에 따라 달라진다.
pub const SURFACE_SENTINEL: f64 = -1.0;

/// NFP 에지 계수 스케일 (반평면 계수).
pub const NFP_EDGE_COEFF: f64 = 0.01;

// ────────────────────────────────────────────────────────────────────────────
// 단계별 예산 (§모듈 10)
// ────────────────────────────────────────────────────────────────────────────

/// Complete 재배치의 완료 대기 타임아웃.
pub const COMPLETE_JOIN_TIMEOUT: Duration = Duration::from_secs(5);

// ────────────────────────────────────────────────────────────────────────────
// 단순화 (§5)
// ────────────────────────────────────────────────────────────────────────────

/// Draft 다각형 단순화 허용오차 — **볼록/오목 각 3단계**.
///
/// 설계는 단계 수만 정하고 값은 정하지 않는다. 이 엔진으로 직접 재서 정했다.
///
/// # 오목 강도 실측 (wsjk156 156조각, `SPSolution::density`, 2026-08-31)
///
/// ```text
///   오목    밀도      시간
///   0.0    72.91%   32.3s
///   0.01   72.81%    3.2s   <- Draft 기본값. 10배 빠른데 손해는 0.10%p
///   0.02   72.61%    2.2s
///   0.05   72.46%    1.4s
///   0.10   70.75%    1.0s   <- 여기서 급락한다
/// ```
///
/// `0.01` 이 무릎이다. 그 아래로 내려가면 시간이 10배 뛰고, 위로 올리면 밀도가
/// 눈에 띄게 빠진다. `0.10` 은 −2.06%p 로 급락하므로 넘지 말 것.
///
/// # 왜 오목만인가
///
/// 볼록 분해 조각 수는 오목 정점 수를 따라가고 pair NFP 의 원시 조각 수는
/// `조각(A) x 조각(B)` 다 — 비용의 지배항이 오목 정점이다. 그리고 오목 정점을
/// 없애면 도형이 **커져** 원본의 상위집합이 되므로 NFP 가 금지영역을 과대평가한다.
/// 놓을 수 있다고 한 자리는 원본 기준으로도 안전하다.
///
/// 볼록은 반대 방향이라 훨씬 작게 잡는다 — 볼록 정점을 깎으면 도형이 **작아져**
/// 실제 겹침을 놓친다.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimplifyLevels {
    /// 볼록 정점 단순화 3단계 (Draft 초안 -> 중간 -> 정밀).
    pub convex: [f32; 3],
    /// 오목 정점 단순화 3단계.
    pub concave: [f32; 3],
}

impl Default for SimplifyLevels {
    fn default() -> Self {
        Self {
            convex: [0.01, 0.005, 0.001],
            concave: [0.01, 0.005, 0.0],
        }
    }
}

impl SimplifyLevels {
    /// Draft 단계에서 쓸 강도 (가장 거친 단계).
    pub fn draft(&self) -> (f32, f32) {
        (self.convex[0], self.concave[0])
    }

    /// Complete 단계에서 쓸 강도 (가장 정밀한 단계).
    ///
    /// 설계 §모듈 10: "Draft와 Complete는 알고리즘 차이가 아니라 다각형 정밀도 차이다."
    pub fn complete(&self) -> (f32, f32) {
        (self.convex[2], self.concave[2])
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 형상 등급 분류 (§5 클래스 파라미터)
// ────────────────────────────────────────────────────────────────────────────

/// 형상 등급 분류 파라미터. 정렬·평가 보조에 쓴다.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClassParams {
    /// 직경 계수 — 등급 경계를 조각 직경의 몇 배로 잡을지.
    pub diameter_factor: f64,
    /// 상위 백분위 (0..1). 이 위가 "큰" 등급.
    pub upper_percentile: f64,
    /// 중간 백분위 (0..1). 이 위가 "중간" 등급.
    pub middle_percentile: f64,
}

impl Default for ClassParams {
    fn default() -> Self {
        Self {
            diameter_factor: 1.0,
            upper_percentile: 0.80,
            middle_percentile: 0.50,
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 기울임 (§모듈 1 파라미터 그룹: att·step·max)
// ────────────────────────────────────────────────────────────────────────────

/// 기울임(tilt) 파라미터.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TiltParams {
    /// 시도 횟수.
    pub attempts: usize,
    /// 한 번에 기울일 각도(라디안).
    pub step_rad: f64,
    /// 최대 누적 기울임(라디안).
    pub max_rad: f64,
}

impl Default for TiltParams {
    fn default() -> Self {
        Self {
            attempts: 0,
            step_rad: 0.0,
            max_rad: 0.0,
        }
    }
}

impl TiltParams {
    /// 기울임을 쓰는가. 기본은 **끔** — 켜면 기존 결과가 바뀐다.
    pub fn enabled(&self) -> bool {
        self.attempts > 0 && self.step_rad > 0.0 && self.max_rad > 0.0
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 수리 (§모듈 1 파라미터 그룹: 초기 시간 비율·재시작 스텝)
// ────────────────────────────────────────────────────────────────────────────

/// 수리(repair) 파라미터.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RepairParams {
    /// 전체 예산 중 초기 배치에 줄 비율 (0..1).
    pub initial_time_fraction: f64,
    /// 재시작 시 물러날 스텝 수.
    pub restart_step: usize,
}

impl Default for RepairParams {
    fn default() -> Self {
        Self {
            initial_time_fraction: 0.5,
            restart_step: 1,
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 전체 묶음
// ────────────────────────────────────────────────────────────────────────────

/// 설계 §모듈 1 의 "전역 파라미터 테이블".
///
/// 그룹 이름은 문서의 등록명을 따른다: 클래스 / 마커 / NFP / 단순화 / 간격 / 기울임 / 수리.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RefPlaceParams {
    /// 클래스 — 형상 등급 분류.
    pub class: ClassParams,
    /// 단순화 — 볼록/오목 각 3단계.
    pub simplify: SimplifyLevels,
    /// 간격 — 조각 간 요구 간격(연성 제약).
    pub gap: super::record::GapSettings,
    /// 기울임.
    pub tilt: TiltParams,
    /// 수리.
    pub repair: RepairParams,
    /// NFP 구멍 생성 — 구멍 있는 형상의 NFP 에 구멍을 반영할지.
    ///
    /// 켜면 오목한 조각의 내부 구멍에 다른 조각을 넣을 수 있게 되지만 NFP 가 커진다.
    pub build_nfp_holes: bool,
    /// 워커 수. `0` 이면 사용 가능한 코어 수.
    pub workers: usize,
    /// 배치 가능 영역을 **증분으로 유지**할지. 기본 **꺼짐**.
    ///
    /// # 왜 기본이 꺼짐인가
    ///
    /// 증분 유지는 이론적으로 매력적이다 — 집합 차가 합집합에 분배되므로
    /// (`A - (B ∪ C) = (A - B) - C`, `swarm-nfp/tests/incremental_subtraction.rs`
    /// 가 그 동치를 못박는다) 매번 다시 합집합하는 대신 새로 놓인 것만 빼면 된다.
    ///
    /// 그런데 실측하니 **4.4배 느렸다** (wsjk156: 20.6초 -> 92초). 배치가 비트 단위로
    /// 같았으므로 동치는 확인됐고, 느린 이유는 비대칭에 있다:
    ///
    /// ```text
    ///   재계산  놓을 조각 1개의 영역 x 장애물 n개   -> 영역 정규화 1회
    ///   증분    미배치 자세 n개의 영역 x 장애물 1개 -> 영역 정규화 n회
    /// ```
    ///
    /// 같은 `O(n²)` 인데 증분 쪽이 영역 정규화를 n 배 더 한다. 그리고 **26개 절벽도
    /// 그대로였다** — 병목이 합집합 크기가 아니었다는 뜻이다(실측: 폴백 46회가 전부
    /// "영역 없음" = 커널 연산 실패이고, "영역 비었음" 은 0회다).
    ///
    /// 그래서 기본은 꺼짐이다. 코드는 남긴다 — 커널 실패 원인을 잡고 나면 증분이
    /// 다시 유리해질 수 있고, 그때 이 스위치로 바로 재 볼 수 있다.
    pub incremental_regions: bool,
    /// 배치 예산(ms). `0` 이면 무제한.
    ///
    /// # 왜 필요한가
    ///
    /// 이 엔진은 `swarm_init` 안에서 **블록**된다. 그 시점에는 진행 콜백이 아직
    /// 없으므로, 오래 걸리면 호스트(WPF)는 **빈 화면으로 계속 기다린다.** 실제로
    /// 그렇게 보고됐다 — wsjk156 에서 92초가 걸렸고 사용자에게는 아무것도 안 보였다.
    ///
    /// 예산을 넘기면 그때까지 놓은 것으로 끝낸다. 부분 배치가 무한 대기보다 낫다.
    pub budget_ms: u64,
}

impl Default for RefPlaceParams {
    fn default() -> Self {
        Self {
            class: ClassParams::default(),
            simplify: SimplifyLevels::default(),
            gap: super::record::GapSettings::default(),
            tilt: TiltParams::default(),
            repair: RepairParams::default(),
            build_nfp_holes: false,
            workers: 0,
            incremental_regions: false,
            budget_ms: 20_000,
        }
    }
}

impl RefPlaceParams {
    /// 실제로 쓸 워커 수. `workers == 0` 이면 코어 수에서 정한다.
    pub fn worker_count(&self) -> usize {
        if self.workers > 0 {
            return self.workers;
        }
        std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// §5 표의 값이 그대로 들어와 있는지. 문서와 코드가 갈라지면 여기서 깨진다.
    #[test]
    fn the_parameter_table_matches_the_design_document() {
        assert_eq!(IMPROVE_MAX_ITERS, 20);
        assert_eq!(IMPROVE_TOLERANCE, 100.0);
        assert_eq!(MOVE_THRESHOLD, 30.0);
        assert_eq!(SUPPORT_DIRECTIONS, 24);
        assert_eq!(PROBE_LIMIT, 10_000);
        assert_eq!(CANDIDATE_PREALLOC, 512);
        assert_eq!(PARALLEL_THRESHOLD, 8);
        assert_eq!(MAX_SCORED_SURFACES, 9);
        assert_eq!(MAX_CONTRIBUTIONS_PER_SURFACE, 5);
        assert_eq!(SHORTFALL_SCALE, 100.0);
        assert_eq!(WEIGHT_BREAK_LOW, 0.2);
        assert_eq!(WEIGHT_BREAK_HIGH, 0.5);
        assert_eq!(SCORE_S_COEFF, 2.0);
        assert_eq!(FINAL_MIN_COEFF, 4.0);
        assert_eq!(FINAL_VMAX_COEFF, 3.0);
        assert_eq!(WEIGHT_SUM_FLOOR, 0.001);
        assert_eq!(SURFACE_SENTINEL, -1.0);
        assert_eq!(NFP_EDGE_COEFF, 0.01);
        assert_eq!(EVAL_ARRAY_PREALLOC, 20_000);
    }

    /// 자유회전 스텝이 정확히 `0.01°` 인지.
    #[test]
    fn the_free_rotation_step_is_one_hundredth_of_a_degree() {
        let one_deg = std::f64::consts::PI / 180.0;
        assert!((FREE_ROTATION_STEP_RAD - one_deg / 100.0).abs() < 1.0e-18);
        // 18000 스텝이면 정확히 180도.
        assert!((FREE_ROTATION_STEP_RAD * 18_000.0 - std::f64::consts::PI).abs() < 1.0e-12);
    }

    /// 단순화 단계는 **거친 것에서 정밀한 것으로** 가야 한다.
    ///
    /// Draft 가 Complete 보다 정밀하면 2단계의 의미가 뒤집힌다.
    #[test]
    fn simplification_goes_from_coarse_to_fine() {
        let s = SimplifyLevels::default();
        assert!(s.convex[0] >= s.convex[1] && s.convex[1] >= s.convex[2]);
        assert!(s.concave[0] >= s.concave[1] && s.concave[1] >= s.concave[2]);
        let (dc, dn) = s.draft();
        let (cc, cn) = s.complete();
        assert!(dc >= cc && dn >= cn, "Draft 가 Complete 보다 정밀하면 안 된다");
    }

    /// 기울임은 기본 꺼짐이어야 한다 — 켜면 기존 결과가 바뀐다.
    #[test]
    fn tilt_is_off_by_default() {
        assert!(!TiltParams::default().enabled());
    }
}
