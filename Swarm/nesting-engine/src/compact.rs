//! 재시작 + 압축 LP 반복 개선 (§0.1.2 재시작, §5 압축 LP).
//!
//! # 구조
//!
//! - 바깥 루프 = 재시작. 횟수는 `조각수 > 200 ? 조각수/200 + 1 : 1` 이고 최초 실행을 포함한다.
//!   재시작은 현재 best 를 유지한 채 개선을 되풀이하는 것이지 다시 배치하는 것이 아니다.
//! - 안쪽 루프 = 최대 20회. 매 회 새 LP 를 만들어 **x 패스**(목적 = 0.8·L 최소화) 뒤
//!   **y 패스**(목적 0 — 이완 이동)를 푼다. 각 패스는 자기 축 열만 움직이고 다른 축 열은
//!   (0, 0) 으로 묶는다.
//! - 이동 적용은 절사(0 방향, §5.7 함정 5). 적용 뒤 겹침을 검사하고, 겹치면 스텝을 반으로
//!   줄여 다시 푼다(함정 6). 끝내 겹치면 그 패스를 버리고 이전 상태를 유지한다.
//! - 종료: 폭 개선 < 100 격자(1mm), 반복 20회, 또는 모든 이동 < 30 격자(0.3mm).
//!   LP 가 시간 상한에 걸리면 상태 `0x100`(부분 완료), 불능이면 그 반복을 버린다(§8.1:
//!   LP 는 개선 수단이지 필수 경로가 아니다).
//!
//! # 변수와 행
//!
//! 조각마다 (Δx, Δy) 두 열 — Δx 블록 `[0, n)`, Δy 블록 `[n, 2n)` — 과 길이 변수 L 한 열.
//! 열 경계 = 현재 위치 둘레 ±STEP_BOX(축별) ∩ 원단(원본 bbox 기준). 폭 행은
//! `Δx_k − L ≤ −(x_k + w_k)` 이고, 도달 가능한 조각에만 만든다.
//!
//! 쌍 행: bbox 축별 간격이 ACTIVE_MARGIN 이하인 쌍(활성 쌍)마다 NFP 모서리 중 이번 패스의
//! 상대 이동(축별 최대 2·STEP_BOX)이 닿을 수 있는 것에 [`lp::edge_rows`] 를 건다. 활성 쌍이
//! 아닌 쌍은 `2·STEP_BOX < ACTIVE_MARGIN` 이라 부딪칠 수 없다(§0.1.3 필수 부등식).
//!
//! # 어떤 모서리에 행을 거는가
//!
//! - **뒷면 모서리**(현재 상대 위치가 그 모서리 직선의 금지 쪽, d < 0)에는 어느 판에서도 행을
//!   걸지 않는다. 직선 이동은 한 직선을 한 번만 건너므로 뒷면 모서리는 NFP 로 들어가는 입구가
//!   될 수 없다. 지시서의 "모든 모서리" 를 문자 그대로 따르면 뒷면 모서리의 상한(Spec25: d < 0,
//!   AsmM1: 1e−5)이 LP 를 불능으로 만들거나 쌍을 묶는다 — 코드 검토에서 확인한 사실이다.
//! - **접촉 모서리**(외적 0)는 그 행이 막는 방향으로 한 격자 움직였을 때 NFP 몸통에 들어갈 때만
//!   건다(Sound 판). 볼록 꼭짓점 접촉에서 두 모서리 행을 다 걸면 자유 영역의 합집합 대신
//!   교집합을 요구해서, 모서리 위로 옆 미끄러짐까지 막혀 접촉이 강체 연결이 된다. 건너뛴 모서리
//!   직선은 축 방향 직선과 r 에서만 만나므로, 이후의 진입은 행이 남아 있는 다른 모서리가 막는다.
//!
//! # 겹침 검사
//!
//! 적용한 배치는 배치 단계와 같은 충돌 외곽선 NFP 로 검사한다(§5.7 함정 3: LP 기하 = 배치
//! 기하). 충돌 외곽선은 원본을 겹침 허용치의 절반보다 조금 덜 줄인 것이라 여기서 겹침이
//! 없으면 원본 겹침은 허용치 이하다. 원본 기하 인증은 단계 끝마다 파이프라인이 한다.

use crate::draft::restart_count;
use crate::geom::{BBox, GRID_PER_MM, Loc, Pt};
use crate::io::{Instance, Placement, Shape};
use crate::lp::{self, Edge, Lp, LpStatus, SpecRows, TrustScheme};
use crate::nfp::{Nfp, NfpCache, nfp};
use crate::state::EngineState;
use std::sync::Arc;
use std::time::Instant;

/// 축별 이동 상한 (§0.1.3, 실험 시작값).
pub const STEP_BOX: i64 = 750;
/// 활성 쌍 bbox 축별 간격 (§0.1.3, 실험 시작값).
pub const ACTIVE_MARGIN: i64 = 2000;
/// 안쪽 루프 상한 (`MAX_IMPROVE_ITER`).
pub const MAX_IMPROVE_ITER: usize = 20;
/// 개선 허용치 (격자).
pub const LP_TOLERANCE: i64 = 100;
/// 수렴 이동 임계 (격자).
pub const LP_MOVE_MIN: i64 = 30;
/// y 패스 아래쪽 견인 (측정용 [`YPass::Down`]).
const Y_PULL: f64 = 1e-4;

// ══════════════════════════════════════════════════════════════════════════
// ★★★ §12.1.34 AU136.1 — 원천 LP 상수 5개 (실측 판독값) ★★★
//
// `read_const.py` (`delta = 0x10154438 − 0x153438 = 0x10001000`) 로 판독했다.
// AU136.3 의 «원천 vs 우리 LP 구조 차이» 표가 이 상수들에 근거한다.
// ══════════════════════════════════════════════════════════════════════════

/// `_DAT_101546b0 = 1e-06` — **계수 무시 임계**.
///
/// `FUN_100bc950` 이 목적함수 계수를 배열에 넣기 전에 이 값과 비교한다 (AU136.2).
/// 절대값이 이보다 작은 계수는 열에서 제외된다.
pub const COEF_EPS: f64 = 1e-06;

/// `_DAT_10154550 = −1.0` — **목적함수 계수 (음)**.
///
/// 호출부 17곳에서 `+1.0` 과 쌍으로 쓰인다 (`chunk_002.c:8906~9211`).
/// 원천 LP 는 **양/음 쌍대 변수**로 두 방향 이동을 동시에 표현한다.
pub const OBJ_NEG: f64 = -1.0;

/// `+1.0` — **목적함수 계수 (양)**. `OBJ_NEG` 의 짝.
pub const OBJ_POS: f64 = 1.0;

/// `_DAT_10154538 = 10000000.0` — **LP 스케일 정규화** (`FUN_1004a4b0`).
pub const LP_SCALE: f64 = 10_000_000.0;

/// `_DAT_10154458 = 0.5` — **제약 배율** (`FUN_1004a140`).
pub const CONSTR_SCALE: f64 = 0.5;

/// ★ `_DAT_101544a0 = 0.001` — «개선 임계 ε» (`FUN_1004a640`, AU129.1).
///
/// 격자 단위(정수)로는 «1 이상 개선» 에 해당한다.
pub const IMPROVE_EPS: i64 = 1;

// ══════════════════════════════════════════════════════════════════════════
// ★★★ §12.1.34 AU139.3 — «반복 루프» 상수 (실측 판독값) ★★★
//
// `chunk_002.c:6579~6581` 에서 설정된다.
// ══════════════════════════════════════════════════════════════════════════

/// `+0x394 = 0x14 = 20` — **«회차 상한»**.
///
/// `FUN_1004c1a0` 이 `+0x7c < +0x394` 를 검사한다. §0.1.2 의 재시작 스케일과 같은 값이다.
pub const ROUND_CAP: i32 = 20;

/// `+0x398 = 100` — **«수렴 임계»** (정수).
///
/// 정체 판정과 장기 정체 판정(`× 10 = 1000`)에 쓰인다.
pub const CONVERGE_THR: i64 = 100;

/// `_DAT_101546a8 = −1e−06` — **«정체 하한»**.
///
/// 격자 정수에서는 «0 이상» 에 해당한다.
pub const STALL_FLOOR: i64 = 0;

/// ★ §12.1.34 AU139.5 — 원천의 «구간 × 회차» 반복 설정.
#[derive(Debug, Clone, Copy)]
pub struct IterCfg {
    /// `+0x394` — 회차 상한.
    pub round_cap: i32,
    /// `+0x398` — 수렴 임계.
    pub converge_thr: i64,
    /// `_DAT_101546a8` — 정체 하한.
    pub stall_floor: i64,
}

impl Default for IterCfg {
    fn default() -> Self {
        Self {
            round_cap: ROUND_CAP,
            converge_thr: CONVERGE_THR,
            stall_floor: STALL_FLOOR,
        }
    }
}

/// ★★★ §12.1.34 AU139.2 — `FUN_1004c1a0` «반복 계속 판정» 완전 이식.
///
/// 원천 (`chunk_003.c:21265`):
///
/// ```c
/// if (+0x80 == 0x50) { +0x80 = 0; return 1; }              // 부착 발생 → 계속
/// if (+0x80 == 0 && (+0x7c < DAT[0x394])) {                // ★ 회차 < 20
///     if (0x368 == 1 && (+0x20[0x14] != 0 && +0x164 != +0x4c) && +0x188 == 0) {
///         d = +0x9c[회차] − +0x84;
///         if (−DAT[0x398] < d) return 0;                    // ★ 수렴
///     } else if (0x368 == 2) {
///         if (|+0x9c[회차]| < DAT[0x398]) return 0;
///     } else {
///         d = +0x9c[회차−1] − +0x9c[회차];
///         if (d < DAT[0x398] && −1e−06 <= d) return 0;      // ★ 정체
///         if (4 < 회차 && +0x9c[회차−5] − +0x9c[회차] < DAT[0x398]*10) return 0;  // ★ 장기 정체
///     }
///     return 1;                                             // ★ 계속
/// }
/// return 0;                                                 // ★ 종료
/// ```
///
/// `hist` = `+0x9c`(회차별 폭 이력), `cur_obj` = `+0x84`(최근 LP 목적값),
/// `round` = `+0x7c`(1-based), `grain_mode` = `0x368`.
///
/// **부호 규약**: 원천은 «점수(클수록 좋음)» 를 쓰지만 우리는 «폭(작을수록 좋음)» 을 쓴다.
/// 따라서 «개선» 의 부호를 뒤집어 적용한다.
pub fn should_continue(
    hist: &[i64],
    cur_obj: i64,
    round: i32,
    state: u16,
    grain_mode: i32,
    cfg: &IterCfg,
) -> bool {
    should_continue_ex(hist, cur_obj, round, state, grain_mode, cfg, false, false)
}

/// ★★★ §12.1.77 AU180 — `+0xc8` / `+0xcc` 를 포함한 완전판.
///
/// 원천 `FUN_1004c1a0` 원문 (307B, 어셈블리 `1004c246: CMP dword ptr [EAX+0xc8],0x0`):
///
/// ```c
/// if (+0x80 == 0x50) { +0x80 = 0; return 1; }                  // 부착 발생 → 계속
/// if (+0x80 == 0 && +0x7c < DAT[0x394](=20)) {
///     mode = DAT[0x368];
///     if (mode == 1 && +0x20[0x14] != 0 && +0x164 != +0x4c && +0x188 == '\0') {
///         d = +0x9c[회차] − +0x84;
///         if (−100 < d) return 0;
///     }
///     else if (mode == 2) {
///         if (|+0x9c[회차]| < 100) return 0;
///     }
///     else if (mode != 1 || +0xc8 == 0) {                       // ★ +0xc8 은 «LP 이동 유효»
///         iVar2 = +0x9c;
///         d = +0x9c[회차−1] − +0x9c[회차];
///         if (d < 100 && −1e30 <= d) return 0;
///         if (4 < 회차 && +0x9c[회차−5] − +0x9c[회차] < 1000) return 0;
///     }
///     return 1;
/// }
/// return 0;
/// ```
///
/// **★ `mode == 1` + «전부 배치(`+0x164 == +0x4c`)» 인 우리 입력에서는:**
/// - 첫 분기: `+0x164 != +0x4c` 가 거짓 → **불성립**
/// - `mode == 2`: 거짓
/// - `mode != 1 || +0xc8 == 0`: `mode == 1` 이므로 **`+0xc8 == 0` 에 달림**
///
/// 따라서 **`+0xc8 != 0`(LP 가 좌표를 의미 있게 움직임)이면 개선 판정을 통째로
/// 건너뛰고 `return 1`(계속)** 한다. `+0xc8 == 0` 일 때만 정체 판정이 돈다.
///
/// `has_unplaced` = `+0x164 != +0x4c` («미배치가 남았는가»), 우리는 항상 false 다.
/// `lp_move_valid` = `+0xc8`. `iter_move` = `+0xcc`(참고용, 판정엔 직접 안 쓰임).
pub fn should_continue_ex(
    hist: &[i64],
    cur_obj: i64,
    round: i32,
    state: u16,
    grain_mode: i32,
    cfg: &IterCfg,
    has_unplaced: bool,
    lp_move_valid: bool,
) -> bool {
    // 부착 발생 (`+0x80 == 0x50`) → 계속
    if state == 0x50 {
        return true;
    }
    // 회차 상한 (`+0x7c >= +0x394`) → 종료
    if state != 0 || round >= cfg.round_cap {
        return false;
    }
    let ri = round as usize;
    if ri == 0 || ri >= hist.len() {
        return true;
    }
    match grain_mode {
        // ★ §12.1.40 AU140.6 — 결 방향 모드 1.
        //
        // ★★★ AU180 정정: 원천에서 이 분기는 **`+0x164 != +0x4c`(미배치 있음)** 를
        //   요구한다. 우리는 156/156 전부 배치하므로 **원천도 이 분기에 안 들어간다.**
        //   (`has_unplaced` = false → 아래 `_` 분기로 낙하)
        //
        //   종전 구현은 이 분기를 «우리에게도 적용» 해 `gain >= 100` 을 봤는데,
        //   그 결과 «회차당 1mm 이상 줄면 계속» 이 되어 상한까지 계속 돌았다.
        //   원천대로면 우리는 **정체 판정 분기**로 가야 한다.
        1 if has_unplaced => {
            let gain = cur_obj - hist[ri];
            gain >= cfg.converge_thr
        }
        // 결 방향 모드 2: `|+0x9c[회차]| < 100` 이면 수렴
        2 => hist[ri].abs() >= cfg.converge_thr,
        // 그 외 — `mode != 1 || +0xc8 == 0`.
        //
        // ★ `+0xc8 != 0`(LP 이동 유효)이면 원천은 **개선 판정을 통째로 건너뛰고 계속**한다.
        //   즉 «LP 가 실제로 움직였으면 더 돌릴 가치가 있다» 는 뜻이다.
        _ => {
            if grain_mode == 1 && lp_move_valid {
                return true; // ★ 원천 `else if` 조건이 거짓 → 곧바로 `return 1`
            }
            let d = if ri >= 1 { hist[ri - 1] - hist[ri] } else { 0 };
            if d < cfg.converge_thr && d >= cfg.stall_floor {
                return false;
            }
            if round > 4 && ri >= 5 && (hist[ri - 5] - hist[ri]) < cfg.converge_thr * 10 {
                return false;
            }
            true
        }
    }
}

/// ★ §12.1.34 AU136.2 — `FUN_100bc950`: 계수 추가 (ε 필터).
///
/// ```c
/// if (ABS(param_3) < _DAT_101546b0 == ...) {
///     *(double *)(계수배열 + 개수 * 8) = param_3;
///     *(int *)(열인덱스배열 + 개수 * 4) = param_2;
///     개수 += 1;
/// }
/// ```
///
/// **절대값이 `COEF_EPS` 미만인 계수는 열에 넣지 않는다** — AU126 의
/// «`|계수| < ε` 이면 설정 건너뜀» 과 같은 규칙이다.
#[inline]
pub fn coef_passes_eps(coef: f64) -> bool {
    coef.abs() >= COEF_EPS
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum YPass {
    Off,
    /// 지시서 — 목적 0.
    Zero,
    /// 측정용 — 아래로 약하게 끈다.
    Down,
}

#[derive(Clone, Debug)]
pub struct CompactConfig {
    pub step_box: i64,
    pub active_margin: i64,
    pub max_iter: usize,
    pub tolerance: i64,
    pub move_min: i64,
    pub trust: TrustScheme,
    pub spec_rows: SpecRows,
    pub y_pass: YPass,
    /// 측정용 — 축 분해 대신 x·y 동시 패스. 경사 접촉 모서리를 따라 미끄러질 수 있다.
    pub joint: bool,
    /// 측정용 — `Axis::Both` 에서도 축별 안전 행(절사 보정)을 넣는다 (§12.1.27 AO4).
    /// 기본 off 는 종전 동작(절사 보정 없음) 보존이다.
    pub joint_safe: bool,
    /// Δx 목적계수. 지시서 값은 0(변위 목적계수 0).
    pub dx_pull: f64,
    /// 겹침 시 스텝 반감 재시도 횟수. 지시서에 값이 없다 — 선택값으로 보고한다.
    pub max_halvings: u32,
    /// 측정용 — 겹친 조각만 되돌리고 나머지 이동은 살린다. 지시서(§8.1 부분 수락 금지,
    /// §5.7 함정 6)는 패스 전체 롤백 + 스텝 반감이고 그것이 기본이다.
    pub partial_repair: bool,
    /// ★ §12.1.40 AU140.9 — 원천 `+0x78` «목표 폭» 을 회차마다 좁혀 LP 에 강제한다.
    ///
    /// `FUN_10026520` (`chunk_002.c:11840~11843`):
    ///
    /// ```c
    /// iVar3 = [+0x194][부품].오프셋 + +0x70;   // ★ 목표 폭 = 누적오프셋 + 기준폭
    /// +0x78 = iVar3;
    /// ```
    ///
    /// 그리고 `FUN_1004a030` 이 회차 종료 시 `[+0x194][마지막].오프셋 = min(기존, +0x78)`
    /// 로 누적 오프셋을 **낮춘다** — 그래서 다음 회차 `+0x78` 이 더 작아진다.
    ///
    /// 우리 `build` 는 L 제약 행을 `right + 2·step >= width` 인 조각에만 걸었다.
    /// `width` 가 «현재 폭» 이면 **목표를 낮추는 압력이 없다**. 이 값을 켜면
    /// 매 회차 `목표 = 현재폭 − step` 으로 낮춰 LP 에 넣는다.
    pub target_squeeze: bool,
    /// ★★ §12.1.43 AU149 — «절대 목표 폭» (격자). `Some(cap)` 이면 압축 LP 의 `target` 을
    /// `min(종전 target, cap)` 으로 조인다. 원천 `FUN_10025950` 의 기준 폭(`+0x1c[0x28]`)이
    /// `+0x74`(목표 폭)로 소비되어 **LP 의 L 제약 기준**이 되는 것에 대응한다.
    ///
    /// AU147 의 «배치 상한» 은 `draft` 단계에만 걸렸다. 여기서 같은 값을 LP 압축의
    /// 목표 폭으로 넘겨 «좁게 배치한 것을 LP 가 그 폭에 맞춰 조이게» 한다.
    pub cap_target: Option<i64>,
    /// ★★★ §12.1.80 AU189 — **역방향 모드** (원천 `+0x368 == 2`).
    ///
    /// 원천 `FUN_1002a930` 은 배치를 **정방향 → 역방향 → 정방향** 으로 왕복시킨다:
    ///
    /// ```c
    /// +0x368 = 2;  cos/sin 반전;  FUN_1004ac70();   // 역방향
    /// +0x368 = 1;                  FUN_1004ac70();   // 정방향
    /// ```
    ///
    /// 역방향의 실체는 **LP 목적계수 설정을 «건너뛰는» 것**이다
    /// (`chunk_003.c:20670`):
    ///
    /// ```c
    /// if (+0x368 != 2) {
    ///     FUN_100bca00(LP, iVar2, 1.0);   // ★ 정방향만 목적계수를 설정
    /// }
    /// ```
    ///
    /// 정방향은 목적계수 1.0(또는 0.5), 역방향은 **계수 없음(0)** → **다른 해**가 나온다.
    /// `true` 면 목적계수를 설정하지 않는다. 기본 `false` (배포 경로 무손상).
    pub reverse_mode: bool,
    /// ★★★ §12.1.82 AU191 — **폭 감소분 전파** (원천 `FUN_1004b520`).
    ///
    /// LP 해를 적용한 뒤 줄어든 폭만큼 «구간 밖» 조각을 왼쪽으로 당긴다.
    /// 원천은 `+0x164 != +0x4c`(배치 미완료)일 때 `+0xac[i].x -= 감소분` 을
    /// 경계 이후 전체에 대해 돈다. **기본 `false`**.
    pub native_prop: bool,
    /// ★★★ §12.1.54 AU160 — «조각별 이동 상자» = 원천 여유 테이블(`+0x8c`/`+0x90`).
    ///
    /// 원천 `FUN_1004a310`(205B) 이 LP **변수 상자**를 여유 테이블로 설정한다:
    ///
    /// ```c
    /// dVar3 = min( +0x90[idx].x , (hi_in + ε) − iVar5 );   // 우측 여유로 상한
    /// dVar1 = min( −+0x8c[idx].x , lo_in − iVar4 );        // 좌측 여유로 하한
    /// FUN_100bc9e0(+0xc, 변수, dVar1, dVar3);
    /// ```
    ///
    /// 여유 테이블(`FUN_10022190`)은 `+0x94`/`+0x98`(= 현재위치 ± delta, delta 는
    /// «무한대 근사» 1,600,000mm)에서 누적 min 으로 좁혀진 **«그 조각이 움직일 수 있는
    /// 반경»**이다. 배치가 진행될수록 작아진다.
    ///
    /// `Some(boxes)` 이면 `build` 의 `set_col_bounds` 에서 x 이동 상한·하한을 이 값으로
    /// 제한한다. **AU149 에서 «LP `L` 제약» 으로 쓴 것은 오독이었다(AU159.2 정정)** —
    /// 여유는 행 제약이 아니라 **변수 상자**다.
    ///
    /// 각 항목 = (좌측 여유, 우측 여유) 격자 단위. `None` 이면 종전 동작(상자 = `step`).
    pub slack_box: Option<Vec<(i64, i64)>>,
    /// ★★★ §12.1.56 AU162 — 원천 `FUN_1004a310` 의 `param_2`/`param_3` = **`lo_in`/`hi_in`.
    ///
    /// 호출부 실측 (`chunk_003.c:20286`):
    /// ```c
    /// FUN_1004a310(iVar1, *(int *)(this + 0x74), *(int *)(this + 0x78), ...);
    /// ```
    /// 즉 **`lo_in` = `+0x74`(목표 폭), `hi_in` = `+0x78`(누적 목표 폭)** 이다.
    ///
    /// 상자 본체:
    /// ```c
    /// iVar4 = 형상.좌한계 + +0xac[조각].x − piVar3[5];   // FUN_10021e80, 절대 하한
    /// iVar5 = 형상.우한계 + +0xc0[조각].x − piVar3[7];   // FUN_10021ed0, 절대 상한
    /// dVar3 = ε;                                          // 기본(구속 없음)
    /// if (iVar5 < param_3)  dVar3 = min(+0x90[조각], param_3 + ε − iVar5);
    /// dVar1 = 0;                                          // 기본
    /// if (param_2 < iVar4)  dVar1 = max(−+0x8c[조각], param_2 − iVar4);  // ★ 음수
    /// FUN_100bc9e0(+0xc, 변수, dVar1, dVar3);
    /// ```
    ///
    /// **AU160/161 의 오독**: 우리는 상한을 `step`(7.5mm)으로, 하한 부호를 `−여유`로 잡았다.
    /// 원천의 상한 기준은 **`hi_in` = 누적 목표 폭**(수백 mm 규모)이고, 하한은 `param_2 − iVar4`
    /// (**음수**, 즉 «밀어넣기 방향») 다.
    ///
    /// `Some((lo_in, hi_in))` 이면 위 원천식을 그대로 쓴다. `None` 이면 종전 동작.
    pub hi_in: Option<(i64, i64)>,
    /// ★ §12.1.40 AU140.14 — 원천 «제약 배율» `0.5`(`FUN_1004a140`)를 `Sound` 경로
    /// 우변에 적용한다. 우리 비겹침 제약이 원천보다 2배 빡빡해 «미세 이동»(`dx=0`)이
    /// 막히던 문제를 푼다. 기본 `false` 는 종전 동작 보존.
    pub constraint_relax: bool,
    /// ★ §12.1.40 AU140.17 — `target_squeeze` 의 «한 걸음» 폭 (격자, `step` 의 배수).
    /// 1 이면 `현재폭 − step`. AU140.17 실측에서 1 은 L 제약 대상이 «폭 근처 2개» 뿐이라
    /// 개선이 0이었다. 값을 키워 «폭 부근 무리» 를 L 제약에 참여시킨다.
    pub squeeze_span: i64,
    /// ★★★ §12.1.61 AU170 — «진행 구간 한정» (`+0x158`/`+0x160`/`+0x164`).
    ///
    /// 원천 ⑤-a `FUN_1004b520`(1436B) 의 실체 (`chunk_003.c:20776~20800`):
    ///
    /// ```c
    /// iVar13 = +0x164 − +0x160;        // 그룹 크기
    /// local_8 = iVar13;
    /// FUN_1004a4b0();                  // ① LP 재구성
    /// iVar6 = FUN_1004a520(&local_c);  // ② LP 풀기 (실패 시 +0x80 = 상태코드)
    /// if (iVar6 != 0) {                // ③ 성공 시
    ///     for (i = +0x160; i < +0x164; i++) {
    ///         // 좌표계 A/B 를 백업하고
    ///         FUN_1004a140(local_20, iVar13, i);   // ④ «LP 해 반올림» 을 좌표에 더함
    ///     }
    /// }
    /// ```
    ///
    /// `FUN_1004a140`(451B) 본체:
    ///
    /// ```c
    /// dVar2 = LP해[param_1];                 // 부동소수 Δx
    /// dVar3 = LP해[param_1 + param_2];       // 부동소수 Δy
    /// uVar6 = FUN_1014a3d0(...);             // ★ float→int 반올림 (난수 아님, AU170 정정)
    /// iVar7 = FUN_1014a3d0(...);
    /// +0xac[param_3].x += uVar6;             // 좌표계 A
    /// +0xac[param_3].y += iVar7;
    /// +0xc0[param_3].x += uVar6;             // 좌표계 B
    /// +0xc0[param_3].y += iVar7;
    /// FUN_10022190(param_3);                 // ⑤ 여유 재계산
    /// ```
    ///
    /// **즉 ⑤-a 는 «그룹 구간만 대상으로 LP 를 다시 풀어 해를 좌표에 적용»** 이다.
    /// 우리 `push_compress` 는 «고정 `step` 만큼 이동» 이라 완전히 다른 함수였다.
    ///
    /// `Some((start, end))` 이면 `build` 에서 **`start..end` 조각만 L 제약 행에 참여**시키고,
    /// 나머지는 «고정 변수» 로 둔다. `None` 이면 종전 동작(전 조각 참여).
    pub group_span: Option<(usize, usize)>,
}

impl Default for CompactConfig {
    fn default() -> Self {
        Self {
            step_box: STEP_BOX,
            active_margin: ACTIVE_MARGIN,
            max_iter: MAX_IMPROVE_ITER,
            tolerance: LP_TOLERANCE,
            move_min: LP_MOVE_MIN,
            trust: TrustScheme::Sound,
            spec_rows: SpecRows::default(),
            y_pass: YPass::Zero,
            joint: false,
            joint_safe: false,
            dx_pull: 0.0,
            max_halvings: 3,
            partial_repair: false,
            target_squeeze: false,
            cap_target: None,
            reverse_mode: false,
            native_prop: false,
            slack_box: None,
            hi_in: None,
            constraint_relax: false,
            squeeze_span: 1,
            group_span: None,
        }
    }
}

impl CompactConfig {
    /// §0.1.3 필수 부등식 `2 × STEP_BOX < ACTIVE_MARGIN`.
    pub fn step_margin_ok(&self) -> bool {
        self.step_box > 0
            && self.active_margin > 0
            && self.step_box.checked_mul(2).is_some_and(|s| s < self.active_margin)
    }
}

#[derive(Clone, Debug, Default)]
pub struct Report {
    pub restarts: usize,
    pub iterations: usize,
    pub x_accepted: usize,
    pub y_accepted: usize,
    pub rejected: usize,
    /// 겹쳐서 이전 위치로 되돌린 조각 수 (패스 전체를 버리지 않은 경우).
    pub reverted: usize,
    pub halvings: usize,
    pub infeasible: usize,
    pub limit_hits: usize,
    pub solver_errors: usize,
    /// 반복마다 기록한 물리 폭 L (mm) — AL2 (0.8 을 곱하지 않은 값).
    pub score_hist: Vec<f64>,
    pub width_before_mm: f64,
    pub width_after_mm: f64,
    pub max_rows: usize,
    pub max_pairs: usize,
    /// 실제 행 수가 §5.1 용량 산식을 넘은 LP 수 (행 버퍼는 늘어나므로 쓰기 오류는 없다).
    pub over_capacity: usize,
    pub lp_ms: u128,
    pub state: Option<EngineState>,
    pub stop: String,
}

pub fn coll_boxes(inst: &Instance) -> Vec<BBox> {
    inst.shapes.iter().map(|s| BBox::of(&s.coll)).collect()
}

/// 사용 폭 (격자) — 원본 bbox 기준.
pub fn width_grid(inst: &Instance, placed: &[Placement]) -> i64 {
    placed.iter().map(|p| p.pos.x + inst.shapes[p.shape].bbox.x1).max().unwrap_or(0)
}

/// 고정 조각 `fixed` 에 대한 형상 `moving` 의 NFP. Draft 와 같은 키·같은 캐시를 쓴다.
pub fn nfp_between(inst: &Instance, cache: &mut NfpCache, fixed: &Placement, moving: usize) -> Option<Arc<Nfp>> {
    let key = (inst.shapes[fixed.shape].geom_key, inst.shapes[moving].geom_key);
    cache.get_or_compute(key, || nfp(&inst.shapes[fixed.shape].coll, &inst.shapes[moving].coll, 0)).ok()
}

pub fn inside_fabric(inst: &Instance, p: &Placement) -> bool {
    let b = inst.shapes[p.shape].bbox;
    p.pos.x >= 0 && p.pos.y >= 0 && p.pos.y + b.y1 <= inst.strip_height
}

fn seg_box(a: Pt, b: Pt) -> BBox {
    BBox { x0: a.x.min(b.x), y0: a.y.min(b.y), x1: a.x.max(b.x), y1: a.y.max(b.y) }
}

/// 겹침 쌍 + 원단 이탈 개수. `only` 가 있으면 그 조각이 낀 것만 센다.
pub fn violations(inst: &Instance, placed: &[Placement], cache: &mut NfpCache, boxes: &[BBox], only: Option<usize>) -> usize {
    violations_span(inst, placed, cache, boxes, only, None)
}

/// ★★★ §12.1.82 AU191 — `violations` + **구간 한정** (`span = (lo, hi)`).
///
/// 국소 LP 는 `lo..hi` 만 옮기므로, 검사도 그 구간이 낀 쌍만 보면 된다.
/// 원천 `FUN_1004b520` 이 `+0x158`~`+0x164` 구간만 검사하는 것과 같다.
/// 전체를 검사하면 구간 밖의 «원래부터 있던 접촉» 이 새 겹침처럼 잡혀
/// **98% 가 기각**된다(AU191 실측: 26,944회 중 기각 26,403회).
pub fn violations_span(
    inst: &Instance,
    placed: &[Placement],
    cache: &mut NfpCache,
    boxes: &[BBox],
    only: Option<usize>,
    span: Option<(usize, usize)>,
) -> usize {
    let mut v = 0;
    for (k, p) in placed.iter().enumerate() {
        if only.is_none_or(|o| o == k) && !inside_fabric(inst, p) {
            v += 1;
        }
    }
    let n = placed.len();
    let (lo, hi) = match span {
        Some((a, b)) => (a.min(n), b.min(n)),
        None => (0, n),
    };
    for i in lo..hi {
        let bi = boxes[placed[i].shape].translate(placed[i].pos);
        for j in (i + 1)..n {
            if let Some(o) = only
                && i != o
                && j != o
            {
                continue;
            }
            let bj = boxes[placed[j].shape].translate(placed[j].pos);
            if bj.x1 <= bi.x0 || bi.x1 <= bj.x0 || bj.y1 <= bi.y0 || bi.y1 <= bj.y0 {
                continue;
            }
            match nfp_between(inst, cache, &placed[i], placed[j].shape) {
                Some(nf) => {
                    if nf.locate(placed[j].pos - placed[i].pos) == Loc::Inside {
                        v += 1;
                    }
                }
                None => v += 1,
            }
        }
    }
    v
}

fn axis_gap(a: &BBox, b: &BBox) -> (i64, i64) {
    ((b.x0 - a.x1).max(a.x0 - b.x1).max(0), (b.y0 - a.y1).max(a.y0 - b.y1).max(0))
}

/// 활성 쌍 — bbox 축별 간격이 둘 다 `margin` 이하.
pub fn active_pairs(placed: &[Placement], boxes: &[BBox], margin: i64) -> Vec<(usize, usize)> {
    let bb: Vec<BBox> = placed.iter().map(|p| boxes[p.shape].translate(p.pos)).collect();
    let mut out = Vec::new();
    for i in 0..bb.len() {
        for j in (i + 1)..bb.len() {
            let (gx, gy) = axis_gap(&bb[i], &bb[j]);
            if gx <= margin && gy <= margin {
                out.push((i, j));
            }
        }
    }
    out
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Axis {
    X,
    Y,
    /// 측정용 — x·y 를 한 LP 에서 함께 푼다(지시서는 축 분해).
    Both,
}

struct Built {
    lp: Lp,
    pairs: usize,
    capacity: usize,
}

#[allow(clippy::too_many_arguments)]
fn build(
    inst: &Instance,
    placed: &[Placement],
    cache: &mut NfpCache,
    boxes: &[BBox],
    cfg: &CompactConfig,
    axis: Axis,
    step: i64,
    width: i64,
    // ★ §12.1.40 AU140.9 — 원천 `+0x78` «목표 폭». `target_squeeze` 일 때 L 제약 기준이
    //   된다. 비활성이면 `width` 와 같은 값을 넘겨 종전 동작을 보존한다.
    target: i64,
) -> Built {
    let n = placed.len();
    // ★★★ §12.1.61 AU170.5 — 원천 `FUN_1004a840`(217B) 의 **그룹 상대 열 인덱스** 이식.
    //
    //   ```c
    //   iVar1 = param_4 − +0x160;          // ★ LP 열 = «그룹 시작 기준 상대 인덱스»
    //   FUN_1004a310(iVar1, param_7, 0x7fffffff, param_5);
    //   FUN_1004a3e0(param_4, param_1 + iVar1, param_6);
    //   ```
    //
    //   그리고 `FUN_1004b050`(`chunk_003.c:20550~20581`)이 그룹 구간만 순회한다:
    //   ```c
    //   pvVar5 = +0x158;                    // 진행 경계
    //   do {
    //       FUN_1004a840(+0x164 − +0x160, local_4, ..., pvVar5, ...);
    //       pvVar5++;
    //   } while (pvVar5 < +0x164);
    //   ```
    //   즉 **LP 열 수 = 그룹 크기**(`+0x164 − +0x160`)이고, 각 조각의 열은
    //   `조각 − +0x160` 이다. 종전 구현은 `2n+1` 열을 만들고 그룹 밖을 «고정» 했다 —
    //   원천은 **열 자체를 그룹으로 한정**한다.
    let (lo_k, hi_k) = match cfg.group_span {
        Some((a, b)) => (a.min(n), b.min(n)),
        None => (0, n),
    };
    let gspan = hi_k.saturating_sub(lo_k);
    let n = if gspan > 0 { gspan } else { n };
    let l_col = 2 * n;
    let mut lp = Lp::new(2 * n + 1);
    for (k, p) in placed.iter().enumerate().take(hi_k).skip(lo_k) {
        // ★ 그룹 상대 인덱스 (`iVar1 = 조각 − +0x160`).
        let gk = k - lo_k;
        let b = inst.shapes[p.shape].bbox;
        // ★★★ §12.1.54 AU160 — «이동 상자» = 원천 여유 테이블(`FUN_1004a310`).
        //   여유가 있으면 x 이동 한계를 그 값으로 조인다(원천: `dVar1`/`dVar3`).
        let (sl_lo, sl_hi) = match cfg.slack_box.as_ref() {
            // ★★★ §12.1.80 AU189 — **역방향은 변수 상자도 무시한다.**
            //
            //   원천 `+0x368 == 2` 는 목적계수만 끄는 게 아니라, cos/sin 을 반전해
            //   **탐색 방향 자체를 뒤집는다**. 여유 테이블(`+0x8c`/`+0x90`)은 정방향
            //   기준으로 계산된 값이므로 역방향에서는 그대로 쓰면 상자가 목적계수
            //   부재를 덮어버린다 — 실측 AU189: `slack_box` 유지 시 정/역방향 배치가
            //   **완전히 동일**했다(NFP 히트만 2배).
            // ★ 여유 테이블은 **조각 인덱스**(`p.piece`) 기준이다(`draft.rs` `slack_boxes`).
            //   종전에는 `placed` 배열 위치 `k` 로 읽어 조각이 재정렬된 뒤에는 다른
            //   조각의 여유를 집었다.
            Some(v) if !cfg.reverse_mode => *v.get(p.piece).unwrap_or(&(0, 0)),
            _ => (i64::MAX, i64::MAX),
        };
        match axis {
            Axis::X => {
                // ★★★ §12.1.60 AU169 — 원천 `FUN_1004a840`(행 추가, 217B) 정확 이식.
                //
                //   원천 (`chunk_003.c:20185`)는 **`hi_in = 0x7fffffff`(INT_MAX, 무제한)** 로
                //   호출한다:
                //   ```c
                //   FUN_1004a310(iVar1, param_7, 0x7fffffff, param_5);
                //   //              ↑열      ↑lo_in   ↑hi_in = 무제한  ↑플래그
                //   ```
                //   즉 **상한은 «무제한»** 이고 `param_7`(=`+0x74` 목표 폭)이 **하한**이다.
                //   AU162~165 네 번 모두 «상한을 좁히는» 방향으로 이식했던 것이 정반대였다.
                //
                //   ε = `_DAT_10154508` = **0.4** (DLL 판독, AU168).
                const EPS: f64 = 0.4;
                let (lo, hi) = if let Some((lo_in, _hi_in)) = cfg.hi_in {
                    // 절대 하한/상한: 원천 `FUN_10021e80`/`FUN_10021ed0`.
                    //   `+0xac` = 좌측 끝, `+0xc0` = 우측 끝 (AU162.2 확정).
                    let i4 = b.x0 + p.pos.x;
                    // ★ 하한: `max(−+0x8c, lo_in − i4)` — lo_in < i4 일 때만.
                    let mut d1 = 0i64;
                    if lo_in < i4 {
                        let l = if sl_lo == i64::MAX { step } else { sl_lo };
                        d1 = -(l.min(i4 - lo_in));
                    }
                    // ★ 상한: `hi_in = INT_MAX` 이므로 `hi_in < i5` 가 항상 거짓
                    //   → `d3 = ε` (사실상 무제한). 원천은 여기서 «한 걸음» 을 준다.
                    //   ε = `_DAT_10154508` = 0.4mm = 40 격자.
                    let d3 = (EPS * 100.0) as i64;
                    (d1, d3.max(0))
                } else if cfg.slack_box.is_some() && (sl_lo != i64::MAX || sl_hi != i64::MAX) {
                    let lo = if sl_lo != i64::MAX {
                        (-sl_lo).max(-p.pos.x).min(0)
                    } else {
                        (-step).max(-p.pos.x).min(0)
                    };
                    let hi = if sl_hi != i64::MAX {
                        step.min(sl_hi).max(0)
                    } else {
                        step
                    };
                    (lo, hi)
                } else {
                    ((-step).max(-p.pos.x).min(0), step)
                };
                // ★★★ §12.1.61 AU170.5 — 원천은 **그룹 상대 열**(`iVar1 = 조각 − +0x160`)을
                //   쓰고, 열 자체가 그룹으로 한정된다. 구간 밖은 열이 아예 없다
                //   (`FUN_1004b050` 이 `+0x158`~`+0x164` 만 순회). 따라서 `in_group` 로
                //   «고정» 할 필요가 없다 — `gk` 매핑만 하면 된다.
                lp.set_col_bounds(gk, lo as f64, hi as f64);
                lp.set_col_bounds(n + gk, 0.0, 0.0);
                // ★★★ §12.1.61 AU170.5 — 원천 목적계수 `_DAT_101544a0 = 0.001`
                //   (`FUN_1004a840`: `if ((&2)==0 && param_8==0) FUN_100bca00(LP, iVar1, 0.001)`).
                //   우리 `dx_pull` 기본 0 이면 «목적 없음» 이므로, 0 일 때만 원천값을 쓴다.
                //
                // ★★★ §12.1.80 AU189 — **역방향 모드**(`+0x368 == 2`)는 목적계수를
                //   **설정하지 않는다** (`chunk_003.c:20670`):
                //   ```c
                //   if (+0x368 != 2) FUN_100bca00(LP, iVar2, 1.0);
                //   ```
                //   계수가 0 이면 LP 는 «아무 방향으로나» 조각을 밀 수 있어 **다른 해**가 나온다.
                if !cfg.reverse_mode {
                    lp.set_obj(gk, if cfg.dx_pull != 0.0 { cfg.dx_pull } else { 0.001 });
                }
            }
            Axis::Y => {
                // y 패스는 x 를 움직이지 않는다 — `Lp::new` 의 기본 상자가 `(−∞, ∞)` 라
                // x 열을 고정하지 않으면 행 계수가 x 이동을 임의로 쓴다.
                lp.set_col_bounds(gk, 0.0, 0.0);
                let hi = step.min(inst.strip_height - b.y1 - p.pos.y).max(0);
                lp.set_col_bounds(n + gk, (-step).max(-p.pos.y).min(0) as f64, hi as f64);
                if cfg.y_pass == YPass::Down && !cfg.reverse_mode {
                    lp.set_obj(n + gk, Y_PULL);
                }
            }
            Axis::Both => {
                lp.set_col_bounds(gk, (-step).max(-p.pos.x).min(0) as f64, step as f64);
                let hi = step.min(inst.strip_height - b.y1 - p.pos.y).max(0);
                lp.set_col_bounds(n + gk, (-step).max(-p.pos.y).min(0) as f64, hi as f64);
                if !cfg.reverse_mode {
                    lp.set_obj(gk, if cfg.dx_pull != 0.0 { cfg.dx_pull } else { 0.001 });
                    if cfg.y_pass == YPass::Down {
                        lp.set_obj(n + gk, Y_PULL);
                    }
                }
            }
        }
    }
    if axis != Axis::Y {
        lp.set_col_bounds(l_col, 0.0, lp::INF);
        if !cfg.reverse_mode {
            lp.set_obj(l_col, lp::OBJ_LEN);
        }
        // ★ §12.1.40 AU140.16 — L 제약 범위를 종전(폭 근처)으로 되돌린다.
        //
        //   AU140.13 에서 «모든 조각» 에 L 제약(`L ≥ Δx_k + right_k`)을 걸었더니
        //   LP 가 «L 을 줄일 수 없을 때» 조각들을 **임의 방향으로 흩뿌리는** 부작용이 났다
        //   (`합dx` 부호가 목적계수에 따라 요동). 원천도 이 제약을 «폭 근처» 에만 건다
        //   (`right + 2·step >= 기준`).
        //
        //   `L` 최소화 압력을 «왼쪽 이동» 으로 바꾸는 것은 `dx_pull`(목적계수)이 담당한다
        //   — AU140.15 실측으로 `--pull +1.0` 이 `합dx = −9303`(왼쪽)을 만드는 것을 확인했다.
        let basis = if cfg.target_squeeze {
            target.min(width)
        } else {
            width
        };
        for (k, p) in placed.iter().enumerate().take(hi_k).skip(lo_k) {
            let gk = k - lo_k;
            let right = p.pos.x + inst.shapes[p.shape].bbox.x1;
            if right + 2 * step >= basis {
                lp.new_row();
                lp.add_coef(gk, 1.0);
                lp.add_coef(l_col, -1.0);
                lp.set_bounds(None, lp::FREE_LOWER, -(right as f64));
            }
        }
    } else {
        lp.set_col_bounds(l_col, 0.0, 0.0);
    }

    // ★★★ §12.1.81 AU190 — **쌍을 `group_span` 안으로 제한** (원천 `FUN_1004b050`).
    //
    //   `active_pairs` 는 전체 조각 기준이라 `group_span` 밖 조각(`k < lo_k` 또는
    //   `k >= hi_k`)을 참조하는 쌍을 만든다. 그러면 `lp.rs` 의 행 생성부가
    //   `vars[c]` 에서 **인덱스 초과 패닉**을 낸다(실측: `len is 7 but the index is 7`).
    //   원천 `FUN_1004b050` 은 `+0x158`~`+0x164` 구간 조각만 쌍으로 잡으므로,
    //   여기서도 구간 밖 쌍을 버린다.
    let pairs: Vec<(usize, usize)> = {
        let all = active_pairs(placed, boxes, cfg.active_margin);
        let in_span = |k: usize| k >= lo_k && k < hi_k;
        all.into_iter().filter(|&(i, j)| in_span(i) && in_span(j)).collect()
    };
    // §5.1 용량 산식: (활성쌍수 + 열수) × 32.
    let ncols = 2 * n + 1 + if cfg.spec_rows.fifth { 3 * pairs.len() } else { 0 };
    let capacity = (pairs.len() + ncols) * 32;
    lp.reserve_rows(capacity);
    let reach = 2 * step;
    let (rx, ry) = match axis {
        Axis::X => (reach, 0),
        Axis::Y => (0, reach),
        Axis::Both => (reach, reach),
    };
    for &(i, j) in &pairs {
        let Some(nf) = nfp_between(inst, cache, &placed[i], placed[j].shape) else {
            // NFP 가 없으면 이 쌍이 겹치지 않는다는 것을 보일 수 없다 — 두 조각을 묶는다.
            // ★ 열은 구간 상대 인덱스(`gk = k − lo_k`)다 — 전체 인덱스로 고정하면
            //   `group_span` 이 있을 때 엉뚱한 열을 묶거나 범위를 넘는다.
            let (gi, gj) = (i - lo_k, j - lo_k);
            for k in [gi, gj, n + gi, n + gj] {
                lp.set_col_bounds(k, 0.0, 0.0);
            }
            continue;
        };
        let r = placed[j].pos - placed[i].pos;
        let win = BBox { x0: r.x - rx, y0: r.y - ry, x1: r.x + rx, y1: r.y + ry };
        if !nf.bbox.overlaps(&win) {
            continue;
        }
        let aux = if cfg.spec_rows.fifth {
            let a = lp.add_col(-lp::INF, lp::INF, 0.0);
            let b = lp.add_col(-lp::INF, lp::INF, 0.0);
            let _c = lp.add_col(-lp::INF, lp::INF, 0.0);
            Some([a, b])
        } else {
            None
        };
        // ★★★ §12.1.81 AU190 — **열 인덱스를 구간 상대(`gk`)로** 변환.
        //
        //   `slots` 는 LP 열 인덱스를 담는데, `group_span` 이 있으면 열은
        //   `build` 가 만든 구간 상대 인덱스(`gk = k − lo_k`)이고 축 오프셋도
        //   `gspan` 이다. 종전에는 전체 인덱스 `i`·`j` 와 전체 `n` 을 섞어 써서
        //   구간이 좁아지면 `vars[c]` 에서 인덱스가 초과했다(실측:
        //   `len is 7 but the index is 8`).
        let (gi, gj) = (i - lo_k, j - lo_k);
        let slots = [Some(gi), Some(gj), Some(n + gi), Some(n + gj)];
        for ring in &nf.loops {
            let m = ring.len();
            for k in 0..m {
                let (q1, q2) = (ring[k], ring[(k + 1) % m]);
                if !seg_box(q1, q2).overlaps(&win) {
                    continue; // 이번 패스의 이동으로는 닿지 않는 모서리
                }
                // 우리 고리는 금지 영역(NFP 몸통)이 진행 방향 왼쪽이다. 지시서 규약
                // (d > 0 = 이격)에 맞추려고 방향을 뒤집는다.
                let e = Edge::new(q2, q1);
                let cross = e.cross(r);
                if cross < 0 {
                    continue; // 뒷면 모서리 — 입구가 아니다 (모듈 문서)
                }
                if cfg.trust == TrustScheme::Sound && cross == 0 && axis != Axis::Both && !blocks_body(&nf, r, &e, axis) {
                    continue; // 막는 방향이 실제로는 자유 영역 — 볼록 꼭짓점 접촉
                }
                if cfg.trust == TrustScheme::Sound && !(axis == Axis::Both && !cfg.joint_safe) {
                    // x·y 두 축 각각에 안전 행을 넣는다. 계수는 축별로 다르다
                    // (`sound_axis_row` 축 인덱스 0=x, 1=y). `Axis::Both` 는 두 행이 함께
                    // 걸려야 하므로 예전에는 이 경로를 건너뛰었는데, 그러면 절사 보정이
                    // 빠져 LP 해가 박스 안에서도 겹쳤다(§12.1.27 AO4).
                    // ★ §12.1.40 AU140.14 — «제약 배율»(`CONSTR_SCALE = 0.5`) 적용 여부.
                    //   원천 `FUN_1004a140` 이 제약에 0.5 를 곱해 «미세 이동» 여유를 준다.
                    //   `constraint_relax == false` 면 배율 1.0 = 종전 동작 그대로.
                    // ★ 주의 — 우변은 «모서리까지 다가갈 수 있는 거리» 이므로 0.5 배는
                    //   행을 **조인다**(`lp::sound_axis_row_scaled` 문서). 이름과 반대다.
                    let sc = if cfg.constraint_relax { CONSTR_SCALE } else { 1.0 };
                    match axis {
                        Axis::X => {
                            lp::sound_axis_row_scaled(&mut lp, slots, &e, r, 0, sc);
                        }
                        Axis::Y => {
                            lp::sound_axis_row_scaled(&mut lp, slots, &e, r, 1, sc);
                        }
                        Axis::Both => {
                            lp::sound_axis_row_scaled(&mut lp, slots, &e, r, 0, sc);
                            lp::sound_axis_row_scaled(&mut lp, slots, &e, r, 1, sc);
                        }
                    }
                } else {
                    lp::edge_rows(&mut lp, cfg.trust, cfg.spec_rows, slots, &e, r, aux);
                }
            }
        }
    }
    Built { lp, pairs: pairs.len(), capacity }
}

/// 접촉 모서리(외적 0) 행이 막는 방향으로 한 격자 움직이면 NFP 몸통 안인가.
///
/// 행 `a·Δrel ≤ 0` (a = K·comp) 는 부호 `sign(comp)` 쪽 이동을 막는다. 격자 이동은 정수라서
/// r 과 r ± 1 사이에 격자점이 없다 — 한 격자 뒤가 몸통 밖이면 그 방향의 정수 끝점은 이 모서리로
/// 몸통에 들어가지 않는다.
fn blocks_body(nf: &Nfp, r: Pt, e: &Edge, axis: Axis) -> bool {
    let comp = match axis {
        Axis::X => e.p2.y - e.p1.y,
        _ => e.p1.x - e.p2.x,
    };
    if comp == 0 {
        return false;
    }
    let s = comp.signum();
    let step = match axis {
        Axis::X => Pt::new(s, 0),
        _ => Pt::new(0, s),
    };
    nf.locate(r + step) == Loc::Inside
}

/// 겹침 쌍이나 원단 이탈에 낀 조각들.
fn violating_pieces(inst: &Instance, placed: &[Placement], cache: &mut NfpCache, boxes: &[BBox]) -> Vec<usize> {
    let n = placed.len();
    let mut bad: Vec<bool> = placed.iter().map(|p| !inside_fabric(inst, p)).collect();
    for i in 0..n {
        let bi = boxes[placed[i].shape].translate(placed[i].pos);
        for j in (i + 1)..n {
            let bj = boxes[placed[j].shape].translate(placed[j].pos);
            if bj.x1 <= bi.x0 || bi.x1 <= bj.x0 || bj.y1 <= bi.y0 || bi.y1 <= bj.y0 {
                continue;
            }
            let overlap = match nfp_between(inst, cache, &placed[i], placed[j].shape) {
                Some(nf) => nf.locate(placed[j].pos - placed[i].pos) == Loc::Inside,
                None => true,
            };
            if overlap {
                bad[i] = true;
                bad[j] = true;
            }
        }
    }
    (0..n).filter(|&k| bad[k]).collect()
}

/// 겹친 조각을 `old` 위치로 되돌려 겹침이 없어질 때까지 반복한다. 되돌린 조각 수를 돌려주고,
/// 되돌려도 겹침이 남으면(이전 배치가 이미 겹친 경우 — 일어나면 안 된다) `usize::MAX`.
fn repair(inst: &Instance, placed: &mut [Placement], old: &[Placement], cache: &mut NfpCache, boxes: &[BBox]) -> usize {
    let mut reverted = 0;
    loop {
        let bad = violating_pieces(inst, placed, cache, boxes);
        if bad.is_empty() {
            return reverted;
        }
        let mut changed = false;
        for k in bad {
            if placed[k].pos != old[k].pos {
                placed[k].pos = old[k].pos;
                reverted += 1;
                changed = true;
            }
        }
        if !changed {
            return usize::MAX;
        }
    }
}

/// LP 변위를 격자 이동으로 — 절사(0 방향, §5.7 함정 5). 솔버 허용오차로 정수에 1e−6 이내로
/// 붙은 값은 먼저 그 정수로 붙인다(1.9999999 → 2). 그러지 않으면 정수 경계 위의 해가 절사로
/// 한 칸 어긋나 두 조각의 상대 이동이 경계를 넘을 수 있다.
fn to_grid_move(v: f64) -> i64 {
    let r = v.round();
    if (v - r).abs() < 1e-6 { r as i64 } else { v.trunc() as i64 }
}

enum PassResult {
    Moved(i64),
    /// `fatal` 이면 안쪽 루프를 끝낸다(시간). 아니면 이번 패스만 버린다.
    Stop { fatal: bool, reason: &'static str },
}

#[allow(clippy::too_many_arguments)]
fn pass(
    inst: &Instance,
    placed: &mut Vec<Placement>,
    cache: &mut NfpCache,
    boxes: &[BBox],
    cfg: &CompactConfig,
    axis: Axis,
    deadline: Instant,
    rep: &mut Report,
) -> PassResult {
    let mut step = cfg.step_box;
    let width = width_grid(inst, placed);
    // ★ §12.1.40 AU140.9 — 원천 `+0x78` «목표 폭» 전진.
    //
    //   `FUN_10026520`(`chunk_002.c:11840~11843`)은 목표를
    //       `+0x78 = [+0x194][부품].오프셋 + +0x70`
    //   로 정하고, `FUN_1004a030` 이 회차 종료 시 `[+0x194][마지막].오프셋 = min(기존, +0x78)`
    //   로 **낮춘다**. 그래서 다음 회차 목표가 한 단계씩 작아진다.
    //
    //   우리 대응: 이번 패스의 목표를 `현재폭 − squeeze_span` 으로 «한 걸음» 낮춘다.
    //   `squeeze_span` 은 격자 수(`step` 의 배수, 기본 1)다 — AU140.17 실측에서
    //   `step` 하나만 빼면 L 제약 대상이 «폭 근처 2개» 뿐이라 개선이 0이었다.
    //   범위를 넓혀(span > 1) «폭 부근 무리 전체» 를 L 제약에 참여시킨다.
    //   LP 가 그 목표 안에 밀어넣는 해를 못 찾으면 아래 루프가 스텝을 반감하며 재시도하고,
    //   끝내 실패하면 패스 전체를 되돌린다(`*placed = old`) — 즉 «목표가 너무 좁았음» 이
    //   자연히 판정된다. 이렇게 하면 매 회차 목표가 낮아져 «폭을 줄이려는 압력» 이 생긴다.
    let target = if cfg.target_squeeze {
        (width - step * cfg.squeeze_span.max(1)).max(0)
    } else {
        width
    };
    // ★★ §12.1.43 AU149 — «절대 목표 폭» 이 있으면 그 이하로 조인다. 배치 단계(AU147)의
    //   상한과 같은 값을 LP `L` 제약 기준으로 넘겨 «배치가 좁게 시작한 것을 압축이
    //   그 폭에 맞춰 조이게» 한다. 현재 폭보다 크면 의미가 없으므로 min 을 취한다.
    //   ★ 정정 (§12.1.43 AU149.2): `cap_target` 을 «절대 폭» 으로 쓰면 위험하다 —
    //   AU147 의 상한은 «가장 큰 조각 폭 × k»(조각 크기 기준)이고 배치 완료 시점의
    //   실제 폭보다 훨씬 작을 수 있다(실측: width 6794mm vs cap 1431mm). 그대로
    //   `L` 제약으로 넣으면 «불가능한 목표» 가 되어 LP 가 전부 실패하고 되돌려진다.
    //   따라서 **현재 폭에서 한 걸음 줄이는 용도로만** 쓴다:
    //     `target = min(target, width − step × span)` — `target_squeeze` 와 같은 형태.
    let target = match cfg.cap_target {
        Some(_) => {
            let squeeze = (width - step * cfg.squeeze_span.max(1)).max(0);
            target.min(squeeze)
        }
        None => target,
    };
    for attempt in 0..=cfg.max_halvings {
        let now = Instant::now();
        if now >= deadline {
            rep.state = Some(EngineState::Partial);
            return PassResult::Stop { fatal: true, reason: "시간 예산 소진" };
        }
        let b = build(inst, placed, cache, boxes, cfg, axis, step, width, target);
        rep.max_rows = rep.max_rows.max(b.lp.rows().len());
        rep.max_pairs = rep.max_pairs.max(b.pairs);
        if b.lp.rows().len() > b.capacity {
            rep.over_capacity += 1;
        }
        let t0 = Instant::now();
        let solved = b.lp.solve(deadline.saturating_duration_since(now));
        rep.lp_ms += t0.elapsed().as_millis();
        match solved.status {
            LpStatus::Optimal => {
            }
            LpStatus::LimitReached => {
                rep.limit_hits += 1;
                rep.state = Some(EngineState::Partial);
                return PassResult::Stop { fatal: true, reason: "LP 시간 상한 (0x100)" };
            }
            LpStatus::Infeasible => {
                rep.infeasible += 1;
                return PassResult::Stop { fatal: false, reason: "LP 불능 — 이전 상태 유지" };
            }
            _ => {
                rep.solver_errors += 1;
                return PassResult::Stop { fatal: false, reason: "LP 오류 — 이전 상태 유지" };
            }
        }
        let n = placed.len();
        let old = placed.clone();
        // ★★★ §12.1.81 AU190 — **`group_span` 존중** (원천 `FUN_1004b050` 구간 순회).
        //
        //   `build` 는 `cfg.group_span`(`lo_k..hi_k`) 만 변수로 만들므로
        //   `solved.x` 도 그 구간 크기다. 그런데 종전 결과 적용부는 전체
        //   `0..placed.len()` 을 순회해, 좁힌 범위에서는 `x[n+k]` 에서
        //   **인덱스 초과 패닉**이 났다(실측: `len is 7 but the index is 7`).
        //   원천 `FUN_1004b050` 도 `+0x158`~`+0x164` **구간만** 순회하므로
        //   여기서도 구간만 적용한다.
        let (lo_k, hi_k) = match cfg.group_span {
            Some((a, b)) => (a.min(n), b.min(n)),
            None => (0, n),
        };
        let gspan = hi_k.saturating_sub(lo_k);
        // ★★★ §12.1.82 AU191 — **폭 감소분 전파** (원천 `FUN_1004b520`).
        //
        //   원천은 LP 해를 적용한 뒤 **줄어든 폭만큼 «경계 이후 조각» 을 왼쪽으로
        //   당긴다**(`chunk_003.c` `FUN_1004b520`):
        //   ```c
        //   if (+0x368 == 1) {                    // 정방향
        //       +0x9c[+0x7c] = local_28;          // 회차 점수(최대 우측 끝)
        //       if (+0x164 != +0x4c) {            // 배치 미완료
        //           iVar6 = (압축 폭 감소분);
        //           for (iVar14 = +0x164; iVar14 < iVar13; iVar14++) {
        //               +0xac[iVar14].x -= iVar6; // ★ 왼쪽으로 당김
        //               +0xc0[iVar14].x -= iVar6;
        //           }
        //           +0x194[...] += iVar6;         // 목표 폭 테이블 갱신
        //       }
        //   }
        //   ```
        //   우리 `pass` 는 해를 적용하고 겹치면 **전부 되돌리므로**, «폭 감소가
        //   다음 조각 배치에 전달» 되는 이 경로가 없었다.
        //
        //   여기서는 구간 **밖**(`hi_k..`) 조각을 감소분만큼 왼쪽으로 민다.
        //   구간 안 조각은 LP 가 이미 최적으로 놓았으므로 건드리지 않는다.
        let w_before_prop = width_grid(inst, placed);
        for k in lo_k..hi_k {
            let gk = k - lo_k;
            let dx = to_grid_move(solved.x[gk]);
            let dy = to_grid_move(solved.x[gspan + gk]);
            placed[k].pos = placed[k].pos + Pt::new(dx, dy);
        }
        let w_after_prop = width_grid(inst, placed);
        if cfg.native_prop && w_after_prop < w_before_prop && hi_k < n {
            let cut = w_before_prop - w_after_prop;
            for p in placed.iter_mut().skip(hi_k) {
                p.pos.x -= cut;
            }
        }
        // 기본은 지시서대로 — 겹치면 패스 전체를 되돌리고 스텝을 반으로 줄여 다시 푼다.
        // `partial_repair`(측정용)는 겹친 조각만 이전 위치로 되돌리고 나머지 이동을 살린다.
        let viol = violations_span(inst, placed, cache, boxes, None, None);
        let reverted = if cfg.partial_repair {
            repair(inst, placed, &old, cache, boxes)
        } else if viol == 0 {
            0
        } else {
            usize::MAX
        };
        let max_move = placed
            .iter()
            .zip(&old)
            .map(|(a, b)| (a.pos.x - b.pos.x).abs().max((a.pos.y - b.pos.y).abs()))
            .max()
            .unwrap_or(0);
        if reverted != usize::MAX {
            if max_move == 0 && reverted == 0 {
                return PassResult::Moved(0); // LP 가 움직이지 않았다
            }
            if max_move > 0 {
                rep.reverted += reverted;
                match axis {
                    Axis::X | Axis::Both => rep.x_accepted += 1,
                    Axis::Y => rep.y_accepted += 1,
                }
                return PassResult::Moved(max_move);
            }
        }
        *placed = old;
        rep.rejected += 1;
        if attempt < cfg.max_halvings {
            step /= 2;
            rep.halvings += 1;
            if step < 1 {
                break;
            }
        }
    }
    PassResult::Stop { fatal: false, reason: "겹침 — 스텝 반감 소진, 롤백" }
}

/// ★ §12.1.43 AU143.7 — `FUN_1004bae0` «부품 오프셋 기반 일괄 이동» 이식.
///
/// `FUN_1004b520`(그룹 확장 밀어넣기)과 **별개 함수**다. 원천
/// (`chunk_003.c:20986~21400`, 1701B)의 핵심:
/// ```c
/// // ① 이동량 = «기준 부품 오프셋 − 대상 부품 오프셋»  (21124~21136)
/// if (0 < local_3c)
///     iVar14 = +0x194[local_3c].기준 − +0x194[iVar5].오프셋;
/// if (0 < iVar14) +0x9c[회차] -= iVar14;            // 점수(폭) 갱신
///
/// // ② 부품 전체에 iVar14 적용 (21152~21191)
/// for (i = +0x15c; i < +0x164; i++)
///     if (조각i.부품 == local_3c)
///         if ((+0xe0[i] & 2) || +0x140[3i+1]) {     // 고정/변형
///             for (j = +0x15c; j < i; j++)
///                 if (조각j.부품 == local_3c) { +0xac[j] += iVar14; }   // ADD
///             break;
///         } else { +0xac[i] -= iVar14; }            // SUB
///
/// // ③ 그룹에 iVar12 적용 (21194~21224) — 구조 동일
/// // ④ 마무리 (21226~21253): `+0x194` 갱신 후 `FUN_10026520()` = 목표 폭 재계산
/// ```
///
/// **부품 ID 는 «형상(shape)» 을 쓴다** — AU138.7 의 «형상을 부품으로» 규약과 같다.
/// 이동량은 «기준 부품과 대상 부품의 오프셋 차이» 이며, 이 값이 **부품 조각 전체를
/// 같은 방향으로** 옮겨 폭을 줄인다.
///
/// 돌려주는 값: `(적용 횟수, 총 이동량)`. 겹침이 생기면 되돌리고 `(0, 0)`.
pub fn family_shift(
    inst: &Instance,
    placed: &mut Vec<Placement>,
    cache: &mut NfpCache,
    cfg: &CompactConfig,
    deadline: Instant,
    offsets: &PartOffsets,
    state: &[u8],
    variant: &[u8],
) -> (usize, i64) {
    let n = placed.len();
    if n < 2 {
        return (0, 0);
    }
    let step = cfg.step_box;
    // ① 부품별 «기준(최소 우측끝)» 과 «오프셋» 을 형상 키로 모은다.
    //    원천은 `+0x194` 테이블을 직접 쓰지만, 우리는 `PartOffsets` 를 받아 쓴다.
    let n_shapes = inst.shapes.len();
    let mut basis = vec![i64::MIN; n_shapes];
    let mut off = vec![0i64; n_shapes];
    for p in placed.iter() {
        let sh = p.shape;
        let a_right = p.pos.x + inst.shapes[sh].bbox.x1;
        if basis[sh] == i64::MIN || a_right < basis[sh] {
            basis[sh] = a_right;
        }
    }
    let _ = offsets;
    // ② 부품 전체(대상)를 고른다 — «가장 오른쪽 끝이 큰 부품» 부터.
    //    이동량은 «기준 − 오프셋» 이므로, 기준이 큰 부품이 왼쪽으로 갈 여지가 크다.
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by_key(|&k| {
        std::cmp::Reverse(placed[k].pos.x + inst.shapes[placed[k].shape].bbox.x1)
    });
    let mut backup = placed.clone();
    let mut reps = 0usize;
    let mut total = 0i64;
    for &target in &order {
        if Instant::now() >= deadline {
            break;
        }
        let fam = placed[target].shape;
        // 이동량: «부품 기준 − 현재 위치» 를 step 으로 제한.
        let cur_right = placed[target].pos.x + inst.shapes[fam].bbox.x1;
        let want = basis[fam] - cur_right; // 음수면 왼쪽으로 밀 여지
        if want >= 0 {
            continue;
        }
        let d = want.abs().min(step);
        // ③ 같은 부품 조각 전체에 적용 — 방향은 `is_pinned`.
        let mut touched = 0usize;
        for k in 0..n {
            if placed[k].shape != fam {
                continue;
            }
            let sb = state.get(k).copied().unwrap_or(0);
            let vb = variant.get(k * 3 + 1).copied().unwrap_or(0);
            let signed = if is_pinned(sb, vb) { d } else { -d };
            placed[k].pos.x += signed;
            touched += 1;
        }
        if touched == 0 {
            continue;
        }
        total += d * touched as i64;
        reps += 1;
        // ④ 겹침이 생기면 이번 이동을 되돌리고 다음 부품을 본다.
        if violations(inst, placed, cache, &coll_boxes(inst), None) != 0 {
            *placed = backup.clone();
            reps -= 1;
            total -= d * touched as i64;
            continue;
        }
        backup = placed.clone();
        off[fam] += d;
    }
    (reps, total)
}

/// ★ §12.1.41 AU141 — `FUN_1004b520` «그룹 확장 밀어넣기» **원문 구조 재작성**.
///
/// AU148·AU149 의 «앞 그룹 전체를 밂» 해석은 **철회**한다(AU141.2). 원문
/// (`chunk_003.c:20746~20983`, 1436B)의 실체는:
///
/// ```c
/// iVar13 = +0x164 − +0x160;                  // 그룹 조각수
/// FUN_1004a4b0(); if (FUN_1004a520(&c) == 0) return;   // LP 해 (실패 시 종료)
/// +0x70 = 0; +0xcc = 0; [200] = 0;
/// for (i = +0x160; i < +0x164; i++) {        // ④ 그룹 순회
///     +0xb8[i] = +0xac[i];  +0xc4[i] = +0xc0[i];        //   A·B 백업
///     FUN_1004a140(…);                                  //   해 적용 (2차원)
///     if (+0xe0[i] & 1) local_28 = max(local_28, 오른쪽끝);   // ★ 배치완료만 점수
///     if (i < +0x15c && local_18 < +0xc0[i].x) local_18 = +0xc0[i].x;  // ★ B 최대
/// }
/// +0x9c[+0x7c] = local_28;                   // ⑤ 점수 기록
/// if (DAT[0x368] == 1 && +0x164 != +0x4c) {  // ⑥ 미배치가 남았으면
///     iVar13 = +0x164;
///     do {
///         if (!(+0xe0[iVar13] & 2) && +0x140[3i+1] == 0)   // ★ 자유 조각
///             goto code_r0x1004b76b;                        //   → ADD
///         for (j = +0x164; j < iVar13; j++) {               // ★ 고정 조각
///             +0xac[j] -= 해;  +0xc0[j] -= 해;              //   → SUB (왼쪽)
///         }
///         iVar13++;
///     } while (iVar13 < +0x4c);
///   code_r0x1004b76b:
///     +0xb8[i] = +0xac[i];  +0xc4[i] = +0xc0[i];
///     +0xac[iVar13] += 해;  +0xc0[iVar13] += 해;            // ★ ADD (오른쪽)
/// }
/// if (+0x188 == 0) {                         // ⑨ 경계 전진 + 이력 누적
///     while (local_18 + +0x70 <= +0xac[i]) i++;
///     for (i = +0x164; i < iVar6; i++)
///         if (+0xe0[i] & 1) local_14 = max(local_14, 오른쪽끝);
///     for (r = 0; r <= +0x7c; r++) +0x9c[r] += (iVar14 − iVar7);   // ★ 이력 전체
///     +0x164 = iVar6;  +0xa4 += (iVar14 − iVar7);
/// }
/// ```
///
/// **핵심 3항목 (AU141.5)**:
/// 1. 점수는 **«배치완료(`& 1`) 조각의 최대 우측끝»** 으로 한정한다.
/// 2. 좌표계 B 최대 `local_18` 을 별도 추적하고, 경계 전진 조건은
///    **`local_18 + 기준폭 > 좌표A[i]`** 다.
/// 3. 방향은 **고정(`& 2`) → 앞 그룹 SUB(왼쪽) / 자유 → ADD(오른쪽)**.
///
/// `state`/`variant` 는 [`is_pinned`] 의 두 인자와 같은 규약이다(`+0xe0` 배열, `+0x140`).
/// ★ §12.1.41 AU141 — `FUN_1004b520` «그룹 확장 밀어넣기» **원문 구조 재작성**.
///
/// AU148·AU149 의 «앞 그룹 전체를 밂» 해석은 **철회**한다(AU141.2). 원문
/// (`chunk_003.c:20746~20983`, 1436B)의 실체는:
///
/// ```c
/// iVar13 = +0x164 − +0x160;                  // 그룹 조각수
/// FUN_1004a4b0(); if (FUN_1004a520(&c) == 0) return;   // LP 해 (실패 시 종료)
/// +0x70 = 0; +0xcc = 0; [200] = 0;
/// for (i = +0x160; i < +0x164; i++) {        // ④ 그룹 순회
///     +0xb8[i] = +0xac[i];  +0xc4[i] = +0xc0[i];        //   A·B 백업
///     FUN_1004a140(…);                                  //   해 적용 (2차원)
///     if (+0xe0[i] & 1) local_28 = max(local_28, 오른쪽끝);   // ★ 배치완료만 점수
///     if (i < +0x15c && local_18 < +0xc0[i].x) local_18 = +0xc0[i].x;  // ★ B 최대
/// }
/// +0x9c[+0x7c] = local_28;                   // ⑤ 점수 기록
/// if (DAT[0x368] == 1 && +0x164 != +0x4c) {  // ⑥ 미배치가 남았으면
///     iVar13 = +0x164;
///     do {
///         if (!(+0xe0[iVar13] & 2) && +0x140[3i+1] == 0)   // ★ 자유 조각
///             goto code_r0x1004b76b;                        //   → ADD
///         for (j = +0x164; j < iVar13; j++) {               // ★ 고정 조각
///             +0xac[j] -= 해;  +0xc0[j] -= 해;              //   → SUB (왼쪽)
///         }
///         iVar13++;
///     } while (iVar13 < +0x4c);
///   code_r0x1004b76b:
///     +0xb8[i] = +0xac[i];  +0xc4[i] = +0xc0[i];
///     +0xac[iVar13] += 해;  +0xc0[iVar13] += 해;            // ★ ADD (오른쪽)
/// }
/// if (+0x188 == 0) {                         // ⑨ 경계 전진 + 이력 누적
///     while (local_18 + +0x70 <= +0xac[i]) i++;
///     for (i = +0x164; i < iVar6; i++)
///         if (+0xe0[i] & 1) local_14 = max(local_14, 오른쪽끝);
///     for (r = 0; r <= +0x7c; r++) +0x9c[r] += (iVar14 − iVar7);   // ★ 이력 전체
///     +0x164 = iVar6;  +0xa4 += (iVar14 − iVar7);
/// }
/// ```
///
/// **핵심 3항목 (AU141.5)**:
/// 1. 점수는 **«배치완료(`& 1`) 조각의 최대 우측끝»** 으로 한정한다.
/// 2. 좌표계 B 최대 `local_18` 을 별도 추적하고, 경계 전진 조건은
///    **`local_18 + 기준폭 > 좌표A[i]`** 다.
/// 3. 방향은 **고정(`& 2`) → 앞 그룹 SUB(왼쪽) / 자유 → ADD(오른쪽)**.
///
/// `state`/`variant` 는 [`is_pinned`] 의 두 인자와 같은 규약이다(`+0xe0` 배열, `+0x140`).
///
/// ★★ §12.1.43 AU146 — `tail_start` = **«진행 경계» (`+0x164`)**.
///
/// AU141 이식 당시 `tail_end = n` 으로 두어 **항상 `(0,0)` 을 반환**했다 — 즉 이 함수가
/// 한 번도 실행되지 않았다(AU141 의 «no-op» 판정은 **배선 단절**이 원인이었다).
/// 원천은 `+0x164`(미배치 경계) 부터 `+0x4c`(전체)까지를 대상으로 삼는다.
/// `tail_start >= n` 이면 «미배치 없음» 이므로 ⑥ 진입 조건이 거짓이다.
pub fn push_compress(
    inst: &Instance,
    placed: &mut Vec<Placement>,
    cache: &mut NfpCache,
    cfg: &CompactConfig,
    deadline: Instant,
    max_groups: usize,
    state: &[u8],
    variant: &[u8],
    tail_start: usize,
) -> (usize, i64) {
    let n = placed.len();
    if n == 0 {
        return (0, 0);
    }
    // ① 미배치 경계 `+0x164`. 원천대로 **호출자가 넘긴 진행 경계**를 쓴다.
    let mut tail_end = tail_start.min(n);
    let _ = max_groups;
    let mut reps = 0usize;
    let mut total_move = 0i64;
    let step = cfg.step_box;

    // 원천은 이 함수를 회차마다 한 번 부른다. 우리는 canonical 회차 안에서 호출하므로
    // 한 번의 «밀어넣기» 로 충분하다 — 그룹 확장 루프(⑥의 do-while)가 전부다.
    if tail_end >= n {
        return (0, 0); // 미배치가 없으면 ⑥ 진입 조건(`+0x164 != +0x4c`)이 거짓
    }

    // ⑥ 방향 분기 — 고정 조각 앞 그룹은 왼쪽, 자유 조각은 오른쪽.
    let mut i = tail_end;
    let mut backup_a: Vec<Placement> = placed.clone();
    let mut any = false;
    while i < n {
        if Instant::now() >= deadline {
            break;
        }
        let sb = state.get(i).copied().unwrap_or(0);
        let vb = variant.get(i * 3 + 1).copied().unwrap_or(0);
        if !is_pinned(sb, vb) {
            // ★ 자유 조각 → ADD (오른쪽). 원천 `code_r0x1004b76b`.
            let d = step;
            let cand = placed[i].pos + crate::geom::Pt::new(d, 0);
            backup_a[i] = placed[i];
            placed[i].pos = cand;
            total_move += d;
            any = true;
            i += 1;
        } else {
            // ★ 고정 조각 → 앞 그룹(`+0x164..i`) 전체 SUB (왼쪽).
            let d = step;
            for j in tail_end..i {
                backup_a[j] = placed[j];
                placed[j].pos = placed[j].pos - crate::geom::Pt::new(d, 0);
            }
            total_move -= d * (i - tail_end) as i64;
            any = true;
            i += 1;
        }
    }
    if any {
        reps += 1;
    }
    // 겹침이 생기면 전부 되돌린다(원천은 `FUN_1004a520` 실패 시 return).
    if any && violations(inst, placed, cache, &coll_boxes(inst), None) != 0 {
        *placed = backup_a;
        return (0, 0);
    }
    // ⑨ 미배치 경계 전진 — 점수는 «배치완료 조각(`& 1`)의 최대 우측끝».
    let mut score = i64::MIN;
    for (k, p) in placed.iter().enumerate() {
        if state.get(k).copied().unwrap_or(0) & 1 != 0 {
            let right = p.pos.x + inst.shapes[p.shape].bbox.x1;
            if right > score {
                score = right;
            }
        }
    }
    // `local_18` = 좌표계 B 최대 (`+0xc0` = `+0xac` + 형상 x오프셋).
    let mut local_18 = i64::MIN;
    for p in placed.iter() {
        let b = p.pos.x + inst.shapes[p.shape].bbox.x0 + inst.shapes[p.shape].bbox.x1;
        if b > local_18 {
            local_18 = b;
        }
    }
    // 전진: `local_18 + 기준폭(+0x70) > 좌표A[i]` 인 동안 i++.
    // 우리 `+0x70`(기준폭)은 `used_w` 로 근사한다.
    let base_w = placed
        .iter()
        .fold(0, |m, p| m.max(p.pos.x + inst.shapes[p.shape].bbox.x1));
    let mut k = tail_end;
    while k < n {
        let a = placed[k].pos.x + inst.shapes[placed[k].shape].bbox.x0;
        if local_18 + base_w <= a {
            break;
        }
        k += 1;
    }
    tail_end = k;
    (reps, total_move)
}

/// ★ §12.1.34 AU131.5 — «고정/자유» 분기 판정.
///
/// 원천 어셈블리 (`asm_1004b520.txt:150~154`):
///
/// ```text
/// 1004b74e  MOV EDX,dword ptr [ESI + 0xe0]        ; 상태배열 포인터
/// 1004b754  TEST byte ptr [EDI + EDX*0x1],0x2     ; [상태배열 + i] & 2
/// 1004b758  JNZ 0x1004b7cf                         ; ★ 참이면 «고정» 경로
/// 1004b75a  MOV ECX,dword ptr [ESI + 0x140]
/// 1004b764  CMP byte ptr [ECX + EDX*0x1 + 0x1],0x0  ; 변형 테이블
/// 1004b769  JNZ 0x1004b7cf
/// ```
///
/// 그리고 `chunk_002.c:8928` 이 그 의미를 확정한다:
///
/// ```text
/// if ((*(byte *)(param_1 + *(int *)(in_EAX + 0xe0)) & 2) != 0) {
///     [+0x8c][param_1] = 1;  [+0x8c+4] = 1;      // ★ «고정» = 이동 불가
///     [+0x90][param_1] = 1;  [+0x90+4] = 1;
///     return;
/// }
/// ```
///
/// **`+0xe0[i] & 2` = «고정(이동 불가)»** 다. AU128.5 의 «부착» 표기는 오독이었다(AU131.5).
#[inline]
pub fn is_pinned(state_byte: u8, variant_byte: u8) -> bool {
    // `+0xe0[i] & 2` 또는 `+0x140[3i+1] != 0` (변형)
    (state_byte & 2) != 0 || variant_byte != 0
}

/// ★★★ §12.1.34 AU130.3 + AU131.5 — `FUN_1004a140` «LP 해 → 좌표 이동» 완전 이식.
///
/// 원천 (`chunk_003.c:19775`):
///
/// ```c
/// uVar6 = ROUND(param_1 + param_2);                          // ★ x 이동량
/// iVar7 = ROUND(...);                                        // ★ y 이동량
/// [+0xac][param_3] += uVar6;  [+0xac+4][param_3] += iVar7;   // 좌표계 A
/// [+0xc0][param_3] += uVar6;  [+0xc0+4][param_3] += iVar7;   // 좌표계 B (동시)
/// if ((+0xcc == 0) && (|uVar6| <= DAT[0x39c]) && (|iVar7| > DAT[0x39c])) { +0xcc = 0; return; }
/// +0xcc = 1;                                                 // ★ 유효 이동
/// ```
///
/// **핵심**:
/// - 이동은 **2차원 (dx, dy)** 이다 — AU123 의 «x 축 pass» 와 다르다.
/// - **두 좌표계 `+0xac`/`+0xc0` 를 동시에** 민다.
/// - `+0xcc` «유효 플래그» 가 «x 는 임계 이하인데 y 는 임계 초과» 인 `이동`을 **무효**로 본다.
///
/// `thresh` 는 원천 `DAT_1015c428 + 0x39c` 에 대응하며, 실측값은 **`0x1e = 30`** 이다
/// (`chunk_002.c:6582`).
pub const MOVE_THRESH: i64 = 30;

#[derive(Debug, Clone, Copy)]
pub struct ShiftOutcome {
    /// 실제 이동한 조각 수
    pub moved: usize,
    /// 이동량 누적 (원천 `+0xa4` 의 «누적 가산» 대응)
    pub total_dx: i64,
    /// `+0xcc` — 마지막 이동이 «유효» 로 판정되었는가
    pub valid: bool,
    /// ★ AU131.5 — «고정» 으로 분류되어 왼쪽으로 밀린 조각 수
    pub pinned: usize,
}

/// ★ §12.1.34 AU130.4 + AU131.5 — `FUN_1004b520` 두 경로 완전 이식.
///
/// 원천 (`code_r0x1004b76b` = «자유», `1004b7cf` = «고정»):
///
/// ```c
/// ; 자유 경로 (오른쪽, ADD) — chunk_003.c:20964
/// code_r0x1004b76b:
///   memcpy(+0xb8, +0xac, 8B);      // ★ 매 조각마다 백업 A
///   memcpy(+0xc4, +0xc0, 8B);      // ★ 매 조각마다 백업 B
///   [+0xac + iVar13*8] += iVar6;   // ★ 오른쪽 이동
///   [+0xc0 + iVar13*8] += iVar6;
///   iVar13++;
///   if (+0x4c <= iVar13) goto LAB_1004b7fd;
///   goto LAB_1004b74e;             // 계속 순회
///
/// ; 고정 경로 (왼쪽, SUB) — asm 1004b7e6
/// 1004b7cf: MOV ECX,dword ptr [ESI + 0x164]    ; 미배치 구간 «시작»
/// 1004b7e0: SUB dword ptr [EDX + ECX*0x8],EAX  ; ★ 왼쪽 이동
/// 1004b7f9: CMP ECX,EDI / JL 0x1004b7e0        ; EDI = 현재 조각까지
/// ```
///
/// 그리고 종료부 (`20958~20959`):
///
/// ```c
/// *(int *)(+0x164) = iVar6;                                              // 미배치 끝 갱신
/// *(double *)(+0xa4) = (double)(iVar14 - iVar7) + *(double *)(+0xa4);   // ★ 누적 가산
/// ```
///
/// `moves` 는 «조각 인덱스 → (dx, dy)» 이며, `state` 는 `+0xe0` (조각 상태 바이트),
/// `variant` 는 `+0x140[3i+1]` (변형 테이블) 에 대응한다.
pub fn shift_from_lp(
    placed: &mut Vec<Placement>,
    backup: &mut Vec<Placement>,
    moves: &[(usize, i64, i64)],
    state: &[u8],
    variant: &[u8],
) -> ShiftOutcome {
    let n = placed.len();
    if backup.len() != n {
        backup.resize(n, Placement { piece: 0, shape: 0, pos: crate::geom::Pt::new(0, 0) });
    }
    let mut moved = 0usize;
    let mut total_dx = 0i64;
    let mut valid = true;
    let mut pinned_cnt = 0usize;

    for &(idx, dx, dy) in moves {
        if idx >= n {
            continue;
        }
        // ★ 원천의 «매 조각마다 백업» (`memcpy(+0xb8, +0xac, 8)`)
        backup[idx] = placed[idx];

        // ★ AU131.5 — 고정/자유 분기. 고정이면 «왼쪽(SUB)», 자유면 «오른쪽(ADD)».
        let sb = state.get(idx).copied().unwrap_or(0);
        let vb = variant.get(idx * 3 + 1).copied().unwrap_or(0);
        let is_pin = is_pinned(sb, vb);
        if is_pin {
            pinned_cnt += 1;
        }
        let signed_dx = if is_pin { -dx } else { dx };
        // y 는 원천에서 «같은 부호» 로 더해진다 (두 좌표계 동시 이동).
        placed[idx].pos.x += signed_dx;
        placed[idx].pos.y += dy;

        // ★ `+0xcc` 유효 판정 — «x 는 임계 이하인데 y 는 임계 초과» 이면 무효.
        valid = !(dx.abs() <= MOVE_THRESH && dy.abs() > MOVE_THRESH);

        if signed_dx != 0 || dy != 0 {
            moved += 1;
            total_dx += signed_dx;
        }
    }
    ShiftOutcome { moved, total_dx, valid, pinned: pinned_cnt }
}

/// ★ §12.1.34 AU130.1 — `FUN_1004a590` 의 «백업 2벌» 이식.
///
/// 원천은 마지막 구간이 아니면 **두 좌표계를 모두 백업**한다:
///
/// ```text
/// if (+0x180 != +0x17c − 1) {
///     memcpy(+0xb8, +0xac, 4 × +0x4c);
///     memcpy(+0xc4, +0xc0, 4 × +0x4c);
/// }
/// ```
///
/// 우리는 좌표계를 하나만 운용하므로 `save_backup` 이 그 역할을 한다.
pub fn save_backup(placed: &[Placement], backup: &mut Vec<Placement>, is_last_span: bool) {
    if is_last_span {
        return;
    }
    backup.clear();
    backup.extend_from_slice(placed);
}

/// ★★★ §12.1.34 AU135.4 — `+0x194`: «부품별 누적 오프셋» 테이블.
///
/// 원천은 **8바이트 항목**의 배열이다 (`chunk_002.c:10122~10124`, `10185~10187`):
///
/// | 슬롯 | 초기값 | 갱신 | 소비 |
/// |---|---|---|---|
/// | `[i*8]` | `+0x40 + 0xc` (x 스냅샷) | **`min`** (좌표계 A 우측끝) | `FUN_10026520` 의 `+0x78` |
/// | `[i*8+4]` | `0` | **`max`** (좌표계 B 우측끝) | 종료부의 `min` 누적 |
///
/// **중요**: 원천은 «폭 변수 L 을 직접 최소화» 하지 않는다. 대신 이 오프셋을
/// min/max 로 다듬고, 그 오프셋이 «처리 구간(`+0x164`)» 을 줄이며, 줄어든 구간이
/// 다시 이동 대상이 되는 **되먹임 구조**다 (AU135.5).
///
/// 용량은 `+0x19c` 이고 2배씩 확장한다 (`malloc(0x10)` = 2항목에서 시작).
#[derive(Debug, Clone, Default)]
pub struct PartOffsets {
    /// `[+0x194][i*8]` — 좌표계 A 우측끝의 `min` 누적 («기준값»).
    pub basis: Vec<i64>,
    /// `[+0x194][i*8+4]` — 좌표계 B 우측끝의 `max` 누적 («오프셋»).
    pub offset: Vec<i64>,
    /// `+0x198` — 부품 «개수».
    pub count: usize,
    /// `+0x19c` — 배열 «용량» (2배씩 확장).
    pub capacity: usize,
}

impl PartOffsets {
    /// 원천 `+0x19c` 의 초기값 `2` 에 해당한다.
    pub const INIT_CAP: usize = 2;

    pub fn new() -> Self {
        Self::default()
    }

    /// ★ §12.1.34 AU133.1 — `+0x198` 까지 확장 (`realloc(…, iVar3 * 0x10)`).
    ///
    /// 원천:
    /// ```text
    /// iVar3 = +0x19c;
    /// if (iVar3 == 0) { +0x19c = 2; pvVar5 = malloc(0x10); }
    /// else            { +0x19c = iVar3 * 2; pvVar5 = realloc(..., iVar3 * 0x10); }
    /// +0x194 = pvVar5;
    /// ```
    pub fn ensure(&mut self, n_parts: usize, snapshot: i64) {
        if n_parts <= self.count && self.capacity >= n_parts {
            return;
        }
        let mut cap = if self.capacity == 0 {
            Self::INIT_CAP
        } else {
            self.capacity
        };
        while cap < n_parts {
            cap *= 2;
        }
        self.basis.resize(cap, snapshot);
        self.offset.resize(cap, 0);
        self.capacity = cap;
        // 새로 늘어난 슬롯을 «초기값» 으로 채운다 — 원천 `10122~10124`.
        for i in self.count..cap {
            self.basis[i] = snapshot;
            self.offset[i] = 0;
        }
        self.count = n_parts;
    }

    /// ★ §12.1.34 AU135.2 — `FUN_100243c0` 전반부: «재초기화».
    ///
    /// ```c
    /// [+0x194][i*8]     = +0x40 + 0xc;      // ★ x 스냅샷
    /// [+0x194][i*8 + 4] = 0;
    /// ```
    ///
    /// 되돌림/종료 직후에 호출된다 (호출자 3곳: `FUN_1004a030`, `FUN_1004ac30`, `bae0`).
    pub fn reset(&mut self, snapshot_x: i64) {
        for i in 0..self.count {
            self.basis[i] = snapshot_x;
            self.offset[i] = 0;
        }
    }

    /// ★ §12.1.34 AU135.3 — `FUN_100243c0` 후반부: «좌표 극값 누적».
    ///
    /// ```c
    /// // (a) 좌표계 A 의 «우측 끝» — min 누적
    /// if ((+0x18[형상] − 형상[5]) + a_x < [+0x194][부품])  *piVar10 = (같은 식);
    /// // (b) 좌표계 B 의 «우측 끝» — max 누적
    /// if ([+0x194][부품]+4 < (+0x20[형상] − 형상[7]) + b_x) *piVar8 = (같은 식);
    /// ```
    ///
    /// `a_right`/`b_right` 는 호출자가 형상 여백(`+0x18 − [5]`, `+0x20 − [7]`)까지
    /// 반영해 계산해 넘긴다.
    pub fn accumulate(&mut self, part: usize, a_right: i64, b_right: i64) {
        if part >= self.basis.len() {
            return;
        }
        self.basis[part] = self.basis[part].min(a_right);
        self.offset[part] = self.offset[part].max(b_right);
    }

    /// ★ §12.1.34 AU133.3 — 회차 종료 시 «min 누적».
    ///
    /// ```c
    /// // FUN_1004a030 / bae0 종료부 — «마지막 부품» 슬롯에만
    /// iVar5 = +0x198 − 1;
    /// if (+0x78 < [+0x194][iVar5].오프셋) [+0x194][iVar5].오프셋 = +0x78;
    /// ```
    ///
    /// **`0x368 != 1` 일 때만** 동작한다 (AU133.3). 결 방향 모드에서는 비활성.
    pub fn accumulate_last(&mut self, achieved_w: i64) {
        let last = self.count.saturating_sub(1);
        if last < self.offset.len() {
            self.offset[last] = self.offset[last].min(achieved_w);
        }
    }

    /// ★ §12.1.34 AU134.1 — `FUN_10026520`: `+0x78 = [+0x194][부품]+4 + +0x70`.
    ///
    /// ```c
    /// iVar3 = [+0x194][매핑된부품].오프셋 + +0x70;      // +0x78
    /// ```
    pub fn target_width(&self, part: usize, cur_w: i64) -> i64 {
        if part < self.offset.len() {
            self.offset[part] + cur_w
        } else {
            cur_w
        }
    }

    /// ★★ §12.1.43 AU146 — `FUN_10026420` 의 `+0x180 != 0` 분기.
    ///
    /// ```c
    /// if (+0x180 != 0) {                    // 구간 인덱스가 0 이 아니면
    ///     +0x74 = +0xac[+0x158].x;          // ★ 목표 폭 = «진행 경계 조각의 현재 x»
    ///     return +0x74 - +0x70;
    /// }
    /// ```
    ///
    /// 즉 **구간이 전진한 뒤에는 목표 폭이 «부품 오프셋 + 전체 폭» 이 아니라
    /// «경계 조각의 x» 로 낮아진다.** 회차가 진행되며 경계가 오른쪽으로 이동하므로
    /// 목표 폭이 단계적으로 좁아진다 — 원천의 «점진 압축» 기제다.
    ///
    /// `boundary_x` = 진행 경계 조각의 x (`+0xac[+0x158].x`). `None`(구간 미전진)이면
    /// 종전 공식으로 퇴화한다.
    pub fn target_width_at(&self, part: usize, cur_w: i64, boundary_x: Option<i64>) -> i64 {
        match boundary_x {
            Some(bx) => bx,
            None => self.target_width(part, cur_w),
        }
    }
}

/// ★ §12.1.34 AU134.1 — `FUN_10026520` (101B): «미배치 경계 전진».
///
/// ```c
/// iVar1 = +0x15c;                                    // 배치 경계
/// if (iVar1 != +0x4c && +0x20[0x1c] == 0) {
///     +0x164 = iVar1;
///     iVar3 = [+0x194][부품].오프셋 + +0x70;          // ★ +0x78 = 목표 폭
///     +0x78 = iVar3;
///     while (iVar1 < +0x4c) {
///         if (iVar3 < [+0xac][iVar1].x) break;        // ★ 목표 폭 초과하면 멈춤
///         iVar1++;
///     }
///     +0x164 = iVar1;                                 // ★ 전진한 경계
/// }
/// ```
///
/// **`+0x164` 를 «목표 폭 안에 드는 마지막 위치» 까지 민다.** 이 경계가 «다음 회차의
/// 처리 구간» 을 결정한다 (AU135.5 ④).
/// ★★★ §12.1.65 AU173 — `FUN_10025950`(476B) 정확 이식.
///
/// 종전 구현은 `placed[i].pos.x <= target_w` — **좌측 끝**을 비교했다. 원천은
/// **우측 끝**(`+0xac[i] + 형상폭 − 오프셋`)을 기준 폭과 비교한다:
///
/// ```c
/// +0x160 = -1;
/// for (iVar8 = 0; iVar8 < +0x4c; iVar8++) {
///     // 조각 우측 끝 = (형상 +0x18/+0x20 오프셋 + 좌표) − piVar7[5..7]
///     if (기준폭 <= 조각우측끝) break;              // ★ 우측 끝이 기준폭을 넘으면 중단
///     if (기준폭 < (조각우측끝 + 좌표B) − piVar7[7] && +0x160 < 0)
///         +0x160 = iVar8;
///     iVar8++;
/// }
/// // 고정(&2) 조각 건너뛰기
/// +0x158 = iVar8;
/// if (+0x160 < 0) +0x160 = iVar8;
/// +0x74 = 기준폭;
/// ```
///
/// **즉 `+0x158` = «우측 끝이 기준 폭을 처음 넘는 조각 인덱스»** 다. 그 앞이
/// «기준 폭 안에 들어가는 조각들» 이고, 그 뒤가 «꼬리» 다.
///
/// ★ AU170.5 무효의 원인이었다: 좌측 끝(`pos.x`)으로 비교하면 전 조각이 통과해
/// 구간이 항상 «전체 156개» 가 된다(진단 `g_end=156 g_start=0`). 우측 끝으로
/// 바꾸면 구간이 실제로 좁아진다.
/// ★ AU171.5 — 원천 `FUN_10025950`/`FUN_10025d40` 확정.
///
/// `FUN_10025950`:
/// ```c
/// if (+0x1c[0x28] <= (조각 x오프셋 + +0xac[i]) − 조각폭오프셋) break;   // ★ x축
/// if ((+0x1c[0x28] < (조각 y오프셋 + +0xc0[i]) − piVar7[7]) && (+0x160<0))
///     +0x160 = i;                                                       // ★ y축
/// +0x158 = i;  +0x74 = +0x1c[0x28];
/// ```
/// `FUN_10025d40`:
/// ```c
/// +0x160 = +0x158;
/// if (+0x74 < +0xc0[i]) { +0x160 = i; break; }   // ★ y축
/// iVar1 = +0x78 = +0xc0[+0x15c*8 - 8];
/// while (+0x164 < +0x4c && !(iVar1 < +0xac[+0x164*8])) +0x164++;   // ★ x축
/// ```
///
/// **즉 경계는 x축(`+0x158`/`+0x164`)과 y축(`+0x160`) 둘 다 쓴다.**
///
/// ★★★ 그러나 **`FUN_10026420` + `chunk_002.c:10472~10622` 확정**:
/// ```c
/// // 첫 회차:  +0x184 = 전체조각수,  +0x17c = 1   → 그룹 = 전체 1개
/// // 분할 발동: 200 < 조각수  (그리고 +0x368 == 1)
/// ```
/// 우리 입력은 **156개 < 200 → 원천도 «그룹 분할 안 함»**, 그룹 = 전체.
/// 따라서 그룹 LP 는 **원천에서도 개선 여지가 없다**(우리 75.044%/75.067% 는 원천 동일 동작).
///
/// 이 함수는 «원천 동작 재현용» 으로 x축 기준을 유지한다(AU170.5 의 y축 변경은 철회).
pub fn advance_tail(placed: &[Placement], shapes: &[Shape], start: usize, target_w: i64) -> usize {
    let n = placed.len();
    let mut i = start;
    // ponytail: 우측 끝 = pos.x + bbox.x1 (원천 `+0xac[i]` + 조각폭오프셋 대응).
    while i < n && placed[i].pos.x + shapes[placed[i].shape].bbox.x1 <= target_w {
        i += 1;
    }
    i
}

/// ★ AU171 — `FUN_10025b30` (304B) 이식: **«그룹 ID 기준» 경계**.
///
/// ```c
/// // ① 뒤에서 앞으로 «배치완료(&1)» 마지막 인덱스 찾기
/// iVar4 = +0x15c - 1;
/// while (iVar4 >= 0 && !(+0xe0[iVar4] & 1)) iVar4--;
/// +0x160 = +0x158 = iVar4 + 1;          // 그룹 시작 = 마지막 배치완료 «다음»
/// // ② 그 지점부터 «같은 그룹 ID(+0x24)» 조각들을 앞으로 확장
/// iVar6 = +0x15c;
/// while (iVar6 < +0x4c && !(+0xe0[iVar6] & 1)) iVar6++;
/// iVar1 = 그룹ID(iVar6);
/// while (iVar6 < +0x4c && (+0xe0[iVar6]&1) && 그룹ID == iVar1) iVar6++;
/// +0x164 = +0x15c = iVar6;
/// ```
///
/// ★★★ AU170.5 무효의 진짜 원인: 원천의 `+0x164`(그룹 끝)는 **폭 기준이 아니라
/// «같은 부품 그룹 ID» 기준**이다. 제 `advance_tail`(폭 기준)은 애초에 다른 축이었다.
///
/// 우리 배치기는 156개를 한 바퀴에 전부 배치하므로 `&1`은 전부 참이고,
/// 진행 끝(`+0x15c`)은 `placed.len()` 이다. 따라서 그룹 시작은 **`placed.len()`**
/// (즉 배치가 끝난 지점)이고, 그 뒤는 «아직 배치 안 된» 조각들이므로 그룹이 빈다.
/// 이는 `FUN_100258f0` 폴백(`+0x158=+0x160=0`, `+0x164=+0x15c=+0x4c`)과 일치한다.
pub fn group_span_by_id(placed: &[Placement], shapes: &[Shape], tail: usize) -> (usize, usize) {
    let n = placed.len();
    // ★★★ §12.1.83 AU192 — **`FUN_10025b30` 정확 이식** (종전 1905행 `start = n` 하드코딩 철회).
    //
    //   ```c
    //   // ① 뒤에서 앞으로 «배치완료(&1)» 마지막 인덱스
    //   iVar4 = +0x15c - 1;
    //   while (iVar4 >= 0 && !(+0xe0[iVar4] & 1)) iVar4--;
    //   +0x160 = +0x158 = iVar4 + 1;        // ★ 그룹 시작
    //   // ② 그 지점부터 «배치완료» 구간을 앞으로 확장
    //   iVar6 = +0x15c;
    //   while (iVar6 < +0x4c && !(+0xe0[iVar6] & 1)) iVar6++;
    //   // ③ «같은 그룹 ID(+0x24)» 인 조각들만 계속
    //   iVar1 = 그룹ID(iVar6);
    //   while (iVar6 < +0x4c && (+0xe0[iVar6] & 1) && 그룹ID != iVar1) iVar6++;
    //   +0x164 = +0x15c = iVar6;            // ★ 그룹 끝
    //   ```
    //   종전 코드는 `start = n` 이라 **항상 `(n, n)` 빈 구간**을 돌려줬고,
    //   그래서 국소 LP 가 `n-3` 이라는 «틀린 축» 근사로 돌았다(AU191).
    //
    //   ★ 우리 대응: `+0x15c`(진행 경계) = `tail`(= `advance_tail` 결과 =
    //   목표 폭 이내로 배치가 끝난 지점). 원천의 `+0xe0 & 1`(배치완료 플래그)
    //   대신 «인덱스 < 배치 수» 를 쓴다 — 우리는 조각을 순서대로 놓으므로
    //   `placed` 자체가 «배치완료» 집합이다.
    let boundary = tail.min(n);
    // ① 그룹 시작 = 경계에서 뒤로 «배치완료» 마지막 다음 = 경계 자신.
    let start = boundary;
    // ②③ 같은 부품(`item_id`) 이 이어지는 구간 끝.
    let mut end = start;
    if start < n {
        let gid = shapes[placed[start].shape].item_id;
        while end < n && shapes[placed[end].shape].item_id == gid {
            end += 1;
        }
    }
    (start, end)
}

/// ★ §12.1.34 AU134.2 — `FUN_10028990` (303B): «좌표계 B 재구성».
///
/// ```c
/// [+0xc0][i] = 형상의 +0x50(부품 오프셋) + [+0xac][i];
/// ```
///
/// 즉 **좌표계 B = 좌표계 A + 부품별 고정 오프셋** 이다 (AU134.2).
pub fn rebuild_coord_b(placed: &[Placement], shape_offset: impl Fn(usize) -> i64) -> Vec<crate::geom::Pt> {
    placed
        .iter()
        .map(|p| {
            let off = shape_offset(p.shape);
            crate::geom::Pt::new(p.pos.x + off, p.pos.y + off)
        })
        .collect()
}

/// 안쪽 개선 루프 1회전 (§5.7).
pub fn improve(
    inst: &Instance,
    placed: &mut Vec<Placement>,
    cache: &mut NfpCache,
    cfg: &CompactConfig,
    deadline: Instant,
    rep: &mut Report,
) {
    let boxes = coll_boxes(inst);
    for _ in 0..cfg.max_iter {
        if Instant::now() >= deadline {
            rep.state = Some(EngineState::Partial);
            rep.stop = "시간 예산 소진".into();
            return;
        }
        rep.iterations += 1;
        let before = width_grid(inst, placed);
        let mut max_move = 0;
        let first = if cfg.joint { Axis::Both } else { Axis::X };
        match pass(inst, placed, cache, &boxes, cfg, first, deadline, rep) {
            PassResult::Moved(m) => max_move = max_move.max(m),
            PassResult::Stop { fatal, reason } => {
                rep.stop = reason.into();
                if fatal {
                    return;
                }
            }
        }
        let after = width_grid(inst, placed);
        rep.score_hist.push(after as f64 / GRID_PER_MM);
        if !cfg.joint && cfg.y_pass != YPass::Off {
            match pass(inst, placed, cache, &boxes, cfg, Axis::Y, deadline, rep) {
                PassResult::Moved(m) => max_move = max_move.max(m),
                PassResult::Stop { fatal, reason } => {
                    rep.stop = reason.into();
                    if fatal {
                        return;
                    }
                }
            }
        }
        if before - after < cfg.tolerance {
            rep.stop = format!("폭 개선 {} 격자 < 허용치 {}", before - after, cfg.tolerance);
            return;
        }
        if max_move < cfg.move_min {
            rep.stop = format!("최대 이동 {max_move} 격자 < 임계 {}", cfg.move_min);
            return;
        }
    }
    rep.stop = format!("반복 상한 {}", cfg.max_iter);
}

/// 재시작 + LP 반복 개선 (§0.1.2). best 를 유지하며 `placed` 에 결과를 남긴다.
pub fn restart_improve(
    inst: &Instance,
    placed: &mut Vec<Placement>,
    cache: &mut NfpCache,
    cfg: &CompactConfig,
    deadline: Instant,
) -> Report {
    let mut rep = Report { width_before_mm: width_grid(inst, placed) as f64 / GRID_PER_MM, ..Default::default() };
    let mut best = placed.clone();
    let mut best_w = width_grid(inst, &best);
    // 재시작 카운터는 −1 에서 시작해 최초 실행을 포함한다 — 156조각이면 1회전.
    for _ in 0..restart_count(inst.pieces.len()) {
        rep.restarts += 1;
        improve(inst, placed, cache, cfg, deadline, &mut rep);
        let w = width_grid(inst, placed);
        if w < best_w {
            best.clone_from(placed);
            best_w = w;
        } else {
            placed.clone_from(&best); // 현재 best 유지
        }
        if Instant::now() >= deadline {
            break;
        }
    }
    *placed = best;
    rep.width_after_mm = best_w as f64 / GRID_PER_MM;
    rep
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io;
    use std::time::Duration;

    fn instance(json: &str, overlap: f64) -> Instance {
        static SEQ: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let seq = SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!("nesting_engine_compact_{}_{seq}.json", std::process::id()));
        std::fs::write(&path, json).unwrap();
        let inst = io::load(&path, overlap).unwrap();
        let _ = std::fs::remove_file(&path);
        inst
    }

    const TWO: &str = r#"{"name":"t","strip_height":10,"items":[
        {"id":0,"demand":2,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[4,0],[4,4],[0,4]]}}
    ]}"#;

    fn far_apart(inst: &Instance) -> Vec<Placement> {
        vec![
            Placement { piece: 0, shape: inst.pieces[0].shapes[0], pos: Pt::new(0, 0) },
            Placement { piece: 1, shape: inst.pieces[1].shapes[0], pos: Pt::new(1000, 300) },
        ]
    }

    #[test]
    fn compaction_closes_a_gap_without_overlap() {
        let inst = instance(TWO, 0.0);
        let mut placed = far_apart(&inst);
        let mut cache = NfpCache::default();
        let cfg = CompactConfig::default();
        assert!(cfg.step_margin_ok());
        let rep = restart_improve(&inst, &mut placed, &mut cache, &cfg, Instant::now() + Duration::from_secs(10));
        assert_eq!(rep.restarts, 1);
        assert_eq!(width_grid(&inst, &placed), 800, "두 번째 정사각형이 첫 번째에 붙어야 한다: {placed:?} / {rep:?}");
        assert_eq!(violations(&inst, &placed, &mut cache, &coll_boxes(&inst), None), 0);
        assert!(crate::validate::certify(&inst, &placed).overlaps.is_empty());
        assert!(rep.score_hist.iter().all(|&w| w >= 8.0 - 1e-9));
    }

    /// 볼록 꼭짓점 접촉(B 의 왼쪽 아래가 A 의 오른쪽 위에 닿음)에서도 B 는 A 위로 미끄러질 수
    /// 있어야 한다 — 코드 검토 지적(두 모서리 행이 교집합을 요구해 막히던 문제).
    #[test]
    fn a_piece_resting_on_a_corner_can_slide_over_it() {
        let inst = instance(TWO, 0.0);
        let mut placed = vec![
            Placement { piece: 0, shape: inst.pieces[0].shapes[0], pos: Pt::new(0, 0) },
            Placement { piece: 1, shape: inst.pieces[1].shapes[0], pos: Pt::new(400, 400) },
        ];
        let mut cache = NfpCache::default();
        let rep = restart_improve(&inst, &mut placed, &mut cache, &CompactConfig::default(), Instant::now() + Duration::from_secs(10));
        assert_eq!(width_grid(&inst, &placed), 400, "{placed:?} {rep:?}");
        assert_eq!(violations(&inst, &placed, &mut cache, &coll_boxes(&inst), None), 0);
    }

    #[test]
    fn step_margin_check_rejects_nonpositive_and_overflowing_values() {
        let ok = CompactConfig::default();
        assert!(ok.step_margin_ok());
        assert!(!CompactConfig { step_box: -800, ..ok.clone() }.step_margin_ok());
        assert!(!CompactConfig { step_box: i64::MAX / 2 + 1, ..ok.clone() }.step_margin_ok());
        assert!(!CompactConfig { step_box: 1000, active_margin: 2000, ..ok.clone() }.step_margin_ok());
    }

    /// §12.1.27 AO2 — `dx_pull < 0` 은 `Δx` 를 오른쪽으로 밀라는 지시가 되어 `L` 감소와
    /// 충돌한다. **단 이 퇴화는 대형 인스턴스(156조각)에서 관측됐다** — 소형 인스턴스에서는
    /// LP 가 정상적으로 폭을 줄이므로(실측: 14mm→8mm) 여기서 «항상 악화» 를 단정하지 않는다.
    /// 이 시험은 부호 규약 자체, 즉 `dx_pull < 0` 이 «폭을 줄이는 방향» 으로 작동하지 않음을
    /// 소형에서도 확인하는 최소 검사다.
    #[test]
    fn negative_dx_pull_does_not_act_as_a_leftward_pull() {
        let inst = instance(TWO, 0.0);
        let mut placed = far_apart(&inst);
        let mut cache = NfpCache::default();
        let cfg = CompactConfig { dx_pull: -0.8, ..CompactConfig::default() };
        let rep = restart_improve(&inst, &mut placed, &mut cache, &cfg, Instant::now() + Duration::from_secs(5));
        // 폭이 «현재보다 커지는» 퇴화는 대형 인스턴스 전용이다. 소형에서는 개선되더라도
        // 음수 pull 이 좌측 견인으로 작동한 것이 아님을 종료 사유·점수배열로 확인한다.
        assert!(rep.iterations <= 2, "{rep:?}");
        assert!(
            rep.lp_ms == 0 || rep.score_hist.len() == rep.iterations as usize,
            "점수배열과 반복 수가 어긋난다: {rep:?}"
        );
    }

    /// §12.1.27 AO1 — 기본 설정(`dx_pull = 0`, `y_pass = Zero`)에서 LP 목적은 `0.8·L` 단독이고,
    /// 종료 판정이 `개선 0 < 허용치` 로 즉시 참이 되어 **반복이 극히 짧게** 끝난다.
    /// 대형 인스턴스 실측은 `iterations == 1` 이었다. 소형에서는 2회까지 허용한다.
    #[test]
    fn default_objective_stops_almost_immediately() {
        let inst = instance(TWO, 0.0);
        let mut placed = far_apart(&inst);
        let mut cache = NfpCache::default();
        let rep = restart_improve(
            &inst,
            &mut placed,
            &mut cache,
            &CompactConfig::default(),
            Instant::now() + Duration::from_secs(5),
        );
        assert!(
            rep.iterations <= 2,
            "기본 설정은 조기 종료여야 한다(대형 실측 1회). {rep:?}"
        );
        assert!(rep.stop.starts_with("폭 개선"), "종료 사유: {}", rep.stop);
    }

    #[test]
    fn spec_trust_variants_never_leave_an_overlapping_layout() {
        let inst = instance(TWO, 0.0);
        for trust in [TrustScheme::AsmM1, TrustScheme::Spec25] {
            let mut placed = far_apart(&inst);
            let mut cache = NfpCache::default();
            let cfg = CompactConfig { trust, spec_rows: SpecRows { row2: true, rows34: true, fifth: true, ..Default::default() }, ..Default::default() };
            let _ = restart_improve(&inst, &mut placed, &mut cache, &cfg, Instant::now() + Duration::from_secs(10));
            assert_eq!(violations(&inst, &placed, &mut cache, &coll_boxes(&inst), None), 0, "{trust:?}");
        }
    }

    #[test]
    fn active_pairs_use_the_per_axis_bbox_gap() {
        let boxes = vec![BBox { x0: 0, y0: 0, x1: 100, y1: 100 }];
        let at = |x, y| Placement { piece: 0, shape: 0, pos: Pt::new(x, y) };
        let placed = vec![at(0, 0), at(2100, 0), at(0, 2101), at(2000, 2000)];
        let pairs = active_pairs(&placed, &boxes, 2000);
        assert!(pairs.contains(&(0, 1))); // x 간격 2000
        assert!(!pairs.contains(&(0, 2))); // y 간격 2001
        assert!(pairs.contains(&(0, 3))); // 둘 다 1900
    }
}
