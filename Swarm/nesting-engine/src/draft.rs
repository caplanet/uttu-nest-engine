//! Draft 배치 (§3) — 부착 체인 · 접촉 쌍 워크리스트 · NFP 경계 후보 · 점수 우선 선택.
//!
//! 도식(`초기배치-1회-도식.md`)의 8단계를 그대로 따른다.
//!
//! 1. **순서** (§1.6.3, §3.2.1): 그룹 없는 입력은 미배치 조각의 피스 키가 모두 0 이라
//!    안정 정렬 결과가 곧 **선언 순서**다. 크기 정렬은 지시서가 명시적으로 부정한다
//!    ("형상 크기 정렬이 아니다"). 크기순은 A/B 측정용 옵션으로만 둔다.
//! 2. **부착 대상** (§3.1): 첫 조각은 부착 대상이 없어(−1) 컨테이너 원점에 시드한다.
//!    이후 조각의 부착 대상은 **직전에 배치된 조각**(체인의 마지막 노드)이다.
//! 3. **후보** (§3.3, §3.3.2): 부착 조각과의 NFP **바깥 경계** 위에서 나온다 — 꼭짓점,
//!    원단 경계와의 교점, 그리고 부착 조각 가까운 이웃의 NFP 와의 교점(두 조각에 동시에
//!    닿는 자리). 구멍(포켓) 후보 생성기는 만들지 않는다.
//! 4. **선택** (§3.4, §3.4.1): 모든 허용 회전을 평가하고 `value` 내림차순, 동률은
//!    `x` 오름차순 → `y` 오름차순 → 접촉 내림차순. "첫 유효 회전 채택" 이 아니다.
//! 5. **보류** (§3.5.6): 1차 패스에서 놓지 못한 조각은 보류 체인으로 보내 2차 패스에서
//!    다시 시도하고, 거기서도 실패하면 `Failed` 로 남긴다. 둘을 섞지 않는다.
//!
//! # 점수와 실행 가능성의 순서
//!
//! `value = (누적 배치면적 + 후보의 현재 폭 안쪽 넓이) / (개수 × 현재 폭 × 높이)` 이고
//! 한 조각 안에서 분모는 고정이다(개수 = 수락 판정 전 목록 크기). 그래서 후보를 먼저
//! 이 순서로 정렬해 두고 실행 가능성(겹침 없음)을 앞에서부터 검사하면, 처음 통과한
//! 후보가 곧 최선이다. §3.4.1 의 탐색 상한 10,000 은 이 검사 횟수에 건다 — 넘으면 새
//! 평가만 멈추고 그때까지의 최선을 유지한다.
//!
//! # 부착 조각이 묻혔을 때 — 전역 접촉 쌍 워크리스트
//!
//! 점수가 "현재 폭 안에 들어가는 넓이" 라서 조각은 왼쪽 빈틈을 먼저 채운다. 빈틈을 꼭
//! 맞게 채운 조각이 체인의 마지막 노드가 되면 그 둘레가 전부 막혀, 다음 조각이 닿을
//! 자리가 없다. 부착 대상은 배치가 성공해야 바뀌므로 이후 조각이 **연쇄 보류**된다
//! (표본 마커 실측: 부착 후보만 쓰면 41/156 배치, 나머지 115 개 전부 실패).
//!
//! 지시서의 `build_contact_worklist`(§3.3)는 **기배치 조각 쌍 전체**에서 bbox 가 교차하는
//! 쌍을 거리² 오름차순으로 모은다. 이 워크리스트로 두 기배치 조각(또는 기배치 조각과
//! 원단 벽)에 동시에 닿는 자리를 후보로 만들고, 부착 조각 후보와 같은 점수·선택 규칙으로
//! 겨루게 한다([`Global::Always`], 기본). 부착 조각 후보만 쓰는 [`Global::Off`] 와 막혔을
//! 때만 쓰는 [`Global::Fallback`] 은 측정용으로 남긴다.
//!
//! # 원단 경계
//!
//! 조각끼리는 수축한 충돌 외곽선으로 판정하지만(겹침 허용), 원단 경계(IFP)는 **수축
//! 전 원본 외곽선의 bbox** 로 잡는다. 그래서 원단 밖 이탈은 격자 반올림(≤0.005mm)뿐이다.
//!
//! # §3.5 게이트를 여기서 쓰지 않는 이유
//!
//! [`crate::gate`] 문서를 보라. 위치 후보 목록에 걸면 동률 후보가 전부 거절되어
//! 조각이 놓이지 못한다.

use crate::compact;
use crate::geom::{BBox, Loc, Pt, area_left_of, area2, dist2, segment_intersection};
use crate::io::{Instance, Placement};
use crate::nfp::{Nfp, NfpCache, nfp};
use std::collections::HashSet;
use std::sync::Arc;
use std::time::{Duration, Instant};

/// 조각 1회 배치(후보 평가 루프 전체)의 탐색 상한 (§3.4.1, `CONTACT_CAP`).
pub const CONTACT_CAP: usize = 10_000;
/// 재시작 횟수 산출 계수 (§0.1.2).
pub const RESTART_SCALE: usize = 200;
/// 후보 **생성** 상한. 평가 상한의 몇 배까지 만들어 두고 좋은 것부터 평가한다.
const CANDIDATE_GEN_CAP: usize = 4 * CONTACT_CAP;

/// 참조 구현(§12.1.33 AU27)의 다중 시작점 계수. 전역 초기화 테이블에서 `+0x3e0 = 0x14 = 20`.
pub const SEED_TRIALS: usize = 20;
/// 참조 구현 `FUN_1005f3f0`/`FUN_1005f4d0` 의 시작점 후보 비트.
///
/// | 비트 | 의미 | 근거 |
/// |---|---|---|
/// | `0x04` | 시작점 후보 | `chunk_004.c:16540` |
/// | `0x10` | 시작점 후보(대안) | `chunk_004.c:16590` |
/// | `0x40` | 시작점 제외 | `chunk_004.c:16540` |
/// | `0x80` | 이미 시작점으로 씀 | `chunk_004.c:16561` |
pub const SEED_FLAG_CAND: u8 = 0x04;
pub const SEED_FLAG_ALT: u8 = 0x10;
pub const SEED_FLAG_SKIP: u8 = 0x40;
pub const SEED_FLAG_USED: u8 = 0x80;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Order {
    /// 지시서 기본 — 선언 순서 (§1.6.3, §3.2.1).
    Declaration,
    /// A/B 측정용 — 면적 내림차순. 지시서 밖이다.
    AreaDesc,
    /// 참조 구현 다중 시작점(§12.1.33 AU27): 면적 내림차순으로 N 개를 첫 조각 후보로 두고,
    /// 각각을 시작점으로 배치를 처음부터 끝까지 돌려 가장 좋은 결과를 채택한다.
    SeedSweep,
    /// 참조 구현 배치 순서(§12.1.34 AU66): 키 = `+0xac[i].x − +0x8c[i].x`.
    ///
    /// 원천 `FUN_10028210` 이 `+0xb0` 에 이 키를 넣고 qsort 한다. `+0xac` = 작업 좌표,
    /// `+0x8c` = 좌측 상한. 우리 입력에서는 `+0x8c` 가 무제한(±INT_MAX, AU63)이므로
    /// **키 ≈ 현재 x 좌표** 가 된다 — 즉 «왼쪽부터» 순서다.
    ///
    /// 원천은 이 정렬을 **매 반복**(`FUN_1004b490` 안) 다시 한다. 이 모드는 초기 1회만
    /// 하므로 «근사» 다. 실측 비교용이다.
    LeftFree,
    /// ★★★ AU172 — 원천 `LAB_100191b0` 정렬 키 이식.
    ///
    /// 원천 목록 생성(`FUN_10017930`) → `qsort(0x14바이트, LAB_100191b0)`.
    /// 비교 함수 어셈블리(`10019272~10019283`):
    /// ```asm
    /// MOV EAX,[EDI+8]   ; A+0x08
    /// MOV ECX,[EBX+8]   ; B+0x08
    /// CMP EAX,ECX ; JL → +1(A 뒤로) ; SETLE → 0 ; else -1
    /// ```
    /// 즉 **`+0x08`(형상 `+0x10`) 오름차순**. 동점이면 `+0x04`(조각 포인터 = 입력 순서),
    /// 그다음 `+0x00`(원래 인덱스).
    ///
    /// 형상 구조체(`0x58`바이트) 확정 필드:
    /// | 오프셋 | 필드 | 근거 |
    /// |---|---|---|
    /// | `+0x10` | **1차 정렬 키** (미확정) | `FUN_10017930` 레코드 `+0x08` |
    /// | `+0x18` | **폭** | `FUN_10082a10` x축 차 |
    /// | `+0x1c` | **높이** | `FUN_10007930` |
    ///
    /// ★ AU172 실측: **폭 오름차순은 최악(64.054%)** 이었다 — `+0x10` 은 폭이 아니다.
    /// 남은 후보는 «높이» 또는 «면적». `+0x18`(폭)/`+0x1c`(높이) **앞**에 있으므로
    /// 두 값을 함께 쓰는 **면적**이 유력하다.
    ///
    /// 이 모드는 **면적 오름차순**으로 근사하고, 동점은 입력 순서로 둔다.
    ShapeWidthAsc,
}

/// 기배치 전체 접촉 쌍 워크리스트(§3.3 `build_contact_worklist`)를 언제 쓰는가.
///
/// 표본 마커(156조각, 겹침 0.8mm) 실측: `Off` 41/156 배치 · `Fallback` 65.16% ·
/// `Always` 71.22% (지시서의 단순화 기준선 0.7108 과 일치, Draft 게이트 0.70 통과).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Global {
    /// 부착 조각 주변 후보만. 부착 조각이 묻히면 이후 조각이 연쇄 보류된다.
    Off,
    /// 부착 조각에 닿는 자리가 전부 막혔을 때만.
    Fallback,
    /// 기본 — 지시서 그대로. 워크리스트는 기배치 쌍 전체에서 만들고, 그 후보가 부착
    /// 조각 후보와 같은 `best_spot` 규칙으로 겨룬다.
    Always,
}

#[derive(Clone, Debug)]
pub struct Config {
    pub order: Order,
    /// 전체 제한 시간 (§0.1.1: 20초). 단조 시계로 잰다(§13.1).
    pub time_limit: Duration,
    /// 절대 마감 시각. 있으면 `time_limit` 대신 쓴다 — 파이프라인의 20초 예산은 전처리
    /// 진입부터 재므로 Draft 가 자기 시작 시각으로 다시 재면 안 된다.
    pub deadline: Option<Instant>,
    /// NFP 끝점 등가 허용오차 (격자). 인자로 받는다(§2.2).
    pub nfp_tol: i64,
    /// 부착 조각과 그 이웃 NFP 의 교점을 후보로 만든다 (§3.3).
    pub use_worklist: bool,
    /// ★ §12.1.86 AU205 — 원천 `FUN_1005f8e0` + `FUN_1005f9c0` 의
    /// **«미결 쌍 수렴 반복»**. `out.pairs` 를 «상태 있는 목록» 으로 보고,
    /// 미결(2) 쌍이 남아 있는 한 «상태 전파 → 기각/부활» 을 반복한다.
    /// **기본 꺼짐** — 배포 경로 무손상.
    pub canonical_pair_converge: bool,
    pub global: Global,
    /// `Order::SeedSweep` 에서 시도할 시작점 수.
    pub seed_trials: usize,
    /// 시작점을 적응적으로 고를지 (§12.1.33 AU39, 원천 `FUN_1005f3f0`).
    /// `true` 면 직전 배치가 만든 접촉 쌍의 두 조각을 다음 시작점 후보로 쓴다.
    pub adaptive_seeds: bool,
    /// 측정용 — 적응 목록을 `seed_trials` 에서 자르지 않는다 (§12.1.34 AU40).
    ///
    /// 원천은 `+0x22c < +0x3e0` 일 때만 목록에 **추가**하고 루프는 `+0x1b0`(목록 전체)까지
    /// 돈다. 즉 `+0x3e0`(20)은 «목록 크기 상한» 이지 «시도 횟수 상한» 이 아니다.
    /// `true` 면 그 해석대로 접촉 쌍이 있는 한 계속 시도한다. 기본 off.
    pub adaptive_uncapped: bool,
    /// ★★★ §12.1.79 AU188 — **재시작 이중 루프** (원천 `FUN_1004ac70` 의 `while (상태 == 0)`).
    ///
    /// 종전 `run_with` 는 시작점 큐를 **한 번만** 돌았다. 원천은 그 위에 **재시작 루프**를
    /// 하나 더 얹는다 (`au38.md` + `chunk_002.c:10413`):
    ///
    /// ```c
    /// +0x180 = -1;
    /// while (상태 == 0) {
    ///     +0x180++;                               // 재시작 카운터
    ///     if (+0x17c <= +0x180) break;            // ★ 재시작 상한
    ///     FUN_10025770(); FUN_10025870();         //   피스 평가배열/모드 세팅 (큐 리셋)
    ///     if (+0x158 < +0x15c) {
    ///         FUN_10049f80();                     //   시간 예산 검사
    ///         do { FUN_1004ad80(); }              //   내부: 시작점 sweep + 회차 루프
    ///         while (FUN_1004c1a0());
    ///     }
    /// }
    /// ```
    ///
    /// **`+0x17c`(재시작 횟수)는 런타임 계산이다** — `au38.md` 가 «156/200+1 = 1» 로 적은
    /// 것은 **초기값**이고, 실제로는 `chunk_002.c:10413` 이 덮어쓴다:
    ///
    /// ```c
    /// piVar5 = +0x194;                            // 부품별 슬롯 (stride 8)
    /// do {
    ///     if ((*piVar5 < +0x58) && (0 < piVar5[1])) iVar8++;   // 조건 충족 개수
    ///     piVar5 += 2;
    /// } while (--iVar7);
    /// if (1 < iVar8) +0x17c = iVar8;              // ★ 재시작 횟수 = iVar8
    /// ```
    ///
    /// `+0x58 = 전역최대폭` (`chunk_002.c:6525`: `+0x58 = *(입력+4) + 0x70`, `+0x70` = 전역 최대 폭)
    /// 이고 `+0x194[슬롯][0]` = 좌표계 A 우측끝의 누적 min 이므로, 조건은 사실상
    /// **«부품 슬롯이 하나라도 있으면 참»** 이다 → `iVar8 ≈ 부품 종류 수`.
    ///
    /// `true` 면 재시작 루프를 돌고, 매 재시작마다 **시작점 큐를 리셋**하며 접촉 쌍·상태 비트를
    /// 초기 상태로 되돌린다. 기본 off (배포 경로 무손상).
    pub restart_loop: bool,
    /// ★★★ §12.1.80 AU189 — **역방향 모드** (원천 `+0x368 == 2`).
    ///
    /// 원천 `FUN_1002a930` 은 배치를 **정방향 → 역방향 → 정방향** 으로 왕복시킨다.
    /// 역방향의 실체는 `compact.rs` 의 LP **목적계수 미설정**이다
    /// (`chunk_003.c:20670`: `if (+0x368 != 2) FUN_100bca00(LP, iVar2, 1.0);`).
    ///
    /// `true` 면 LP 목적계수를 설정하지 않아 **다른 해**가 나온다.
    /// 기본 `false` (배포 경로 무손상). `bidir_outer` 와 함께 쓴다.
    pub reverse_mode: bool,
    /// ★★★ §12.1.80 AU189 — **시작점별 방향 왕복** (원천 `FUN_1002a930`).
    ///
    /// `true` 면 **시작점 1개마다** 정방향 sweep 과 역방향 sweep 을 **둘 다** 돌리고
    /// 폭이 좁은 쪽을 채택한다. 원천의 `+0x368 = 2 → 1` 왕복에 대응한다.
    ///
    /// **왕복 단위가 «시작점 1개» 인 것이 핵심이다.** 전체 스윕을 단위로 하면
    /// 1회 왕복이 예산을 다 먹는다(실측: 정방향 100초 + 역방향 79초 = 179초, 왕복 1회).
    ///
    /// 기본 `false` (배포 경로 무손상).
    pub bidir_swap: bool,
    /// ★★★ §12.1.81 AU190 — **원천 파이프라인 이식** (`--autonest-native`).
    ///
    /// 원천 `FUN_1004ae60` 은 «배치(`FUN_1004b490`) → LP(`FUN_1004aee0`)» 를
    /// **한 함수 안에서** 돌린다. 우리 `--max-density` 는 `canonical_rounds = 0`
    /// 이라 LP 블록 전체를 건너뛰어 **LP 가 한 번도 돌지 않았다**(AU189 계측:
    /// `[BUILD]` 0건). 이 플래그가 그 게이트를 우회한다.
    ///
    /// **기본 `false`** — 배포 경로(`--best`)와 최고기록(`--max-density`) 무손상.
    pub native_lp: bool,
    /// ★★★ §12.1.82 AU191 — **폭 감소분 전파** (원천 `FUN_1004b520`).
    /// `--native-prop` 로 켠다. **기본 `false`**.
    pub native_prop: bool,
    /// 재시작 루프의 `+0x3e0` — **시작점 목록 크기 상한** (원천 20).
    ///
    /// 원천 `FUN_1005f3f0` 은 `+0x22c < +0x3e0` 일 때만 목록에 **추가**하고, `+0x22c`(쌍 개수)가
    /// `+0x3e0` 에 도달하면 `+0x16c = +0x174` 로 **«배치 종료»** 한다. 재시작 루프가 이 종료를
    /// 받아 다음 재시작으로 넘어간다. 0 이면 무제한(종전 동작).
    pub seed_list_cap: usize,
    /// 측정용 — 전역 후보를 «전 기배치 조각» 으로 볼지 (§12.1.34 AU86).
    ///
    /// 원천 `FUN_10028380` 은 정렬 키 + 이진 탐색으로 좁힌 범위만 훑는다. `false` 면
    /// 그 구조를 근사해 «현재 사용 폭 − `global_window`» 이후 조각만 본다. 기본 `true`
    /// (종전 동작 보존).
    pub wide_global: bool,
    /// 측정용 — `wide_global == false` 일 때 후보로 볼 폭 구간(격자).
    pub global_window: i64,
    /// 측정용 — 배치 후 «현재 좌표 기준» 으로 순서를 다시 계산해 재배치하는 회차 수
    /// (§12.1.34 AU66·AU90). 원천은 `FUN_1004b490` 안에서 **매 반복 정렬**한다. 0 = 끔.
    pub reorder_rounds: usize,
    /// ★ 재작성 모드 (§12.1.34 AU101) — 원천 `FUN_10028380` 의 **«기준값 기반 배치»**.
    ///
    /// `true` 면 `place()` 가 «부착 체인 + NFP 경계 후보» 대신 **원천 구조**를 쓴다:
    ///
    /// 1. 조각마다 기준값 `+0xf0 = bbox.x0 − 좌측상한`(AU99, 우리 입력은 좌측상한 무제한이라
    ///    `bbox.x0`)을 계산해 **정렬 목록**을 만든다.
    /// 2. 목록을 기준값 오름차순으로 훑으며, **각 후보 조각을 «이미 배치된 조각 전체»** 에
    ///    대해 시도한다(부착 대상이 «직전 조각» 하나가 아니다).
    /// 3. 위치는 `FUN_10091a10`(AU98)대로 **`기배치 조각 위치 − NFP 기준점`** 으로 역산한다.
    /// 4. 채택은 `FUN_10022020`(AU100)의 **관계 제약 + 겹침 없음**이다.
    ///
    /// 기본 `false`(종전 부착 체인 동작) — A/B 실측용.
    pub canonical: bool,
    /// ★★★ §12.1.76 AU179 — 원천 `DAT_1015c428 + 0x394` **«내부 반복 상한»**.
    ///
    /// **원천 값 = 20** (`pe-data-constant-read.md`: 초기화 함수 리터럴 `+0x394=0x14`).
    /// `FUN_1004c1a0` 이 `회차(+0x7c) < +0x394` 를 **직접** 검사하므로, 이 값은
    /// «우리가 정하는 상한» 이 아니라 **원천 상수다.** 종전 AU139 주석이
    /// «원천엔 회차 수 노브가 없다» 는 취지로 적혀 있었으나 **오독**이었다(AU179 정정).
    ///
    /// 루프를 실제로 멈추는 것은 상한이 아니라 `+0x398`(수렴 임계 100)이며
    /// `compact::should_continue` 가 매 회차 그것을 본다.
    pub canonical_rounds: usize,
    /// ★★ §12.1.43 AU145 — «배치 도중 개선». 첫 배치 루프를 3분할해 구간 경계마다
    /// `canonical_pass` + LP 압축을 끼운다.
    ///
    /// AU144.13 에서 확정한 대로 원천의 압축 계열은 `+0x15c`/`+0x160`/`+0x164`
    /// «진행 중 상태» 를 입력으로 받으므로, **배치가 안 끝난 시점**에 돌려야
    /// `& 1`(배치완료) 이 «부분집합» 을 나타내 판별력이 생긴다.
    ///
    /// 기본 `false` — 종전 동작(배치 완료 후 회차) 무손상.
    pub canonical_midpass: bool,
    /// ★ §12.1.43 AU145.5 — 첫 바퀴에서 전역 폴백을 끌지 여부(«배치 도중» 생성).
    pub canonical_midpass_late_global: bool,
    /// ★★ §12.1.43 AU146 — 구간 분할 임계 `+0x3cc`. 0 이면 구간 1개(종전 동작).
    ///
    /// 원천 `DAT[0x3cc] = 200` 이지만 진입 조건(`200 < 조각수`)이 우리 배치(156)에서
    /// 거짓이라 구간이 1개가 된다. 이 값을 낮추면 **구간 전진 → 목표 폭 하강** 기제가
    /// 살아난다(`FUN_10026420` 의 `+0x180 != 0` 분기).
    pub canonical_slice_thr: usize,
    /// ★ §12.1.43 AU147 — 첫 바퀴 목표 폭 상한 배수(«가장 큰 조각 폭 × 배수»).
    pub canonical_cap_mult: i64,
    /// ★★ §12.1.43 AU150 — 상한 배수의 **퍼밀**(1/1000) 정밀도. 0 이면 `canonical_cap_mult`
    /// 정수 배수를 쓴다. 예: 2500 = ×2.5. AU147 의 정수 스윕(k=1,2,3)에서 **k=2 가 최적**
    /// 이었으나 2.0~3.0 사이에 더 나은 값이 있을 수 있다.
    pub canonical_cap_permille: i64,
    /// ★★ §12.1.43 AU149 — 배치 상한(AU147)을 **LP 압축 목표 폭**으로도 전달할지.
    pub canonical_cap_to_lp: bool,
    /// ★★ §12.1.43 AU148 — 상한을 **매 바퀴 점진 확대**할지 여부.
    ///
    /// 원천 `FUN_10025950` 은 기준 폭(`+0x1c[0x28]`)을 **매번 재설정**한다 — 즉 «좁게
    /// 시작해 배치 진행에 따라 단계적으로 넓어지는» 거동이다. `true` 면 배수가
    /// `k + wave` 로 커지고 상한이 «배치된 폭» 아래로 내려가지 않는다.
    pub canonical_cap_grow: bool,
    /// ★★★ §12.1.61 AU170 — «진행 구간 한정 LP» (원천 ⑤-a `FUN_1004b520`).
    ///
    /// 원천 회차 루프(`FUN_1004ad80`, `chunk_003.c:20401~20407`)는 Complete 이후:
    ///
    /// ```c
    /// if ((+0x20[0x14] == 0) || (+0x188 != 0) || (+0x164 == +0x4c))
    ///     FUN_1004b520();     // ⑤-a  ← «전부 배치됨» 에서 호출
    /// else
    ///     FUN_1004bae0();     // ⑤-b  ← «미배치 있음» 에서 호출
    /// ```
    ///
    /// **종전 판정(«`+0x164 != +0x4c` 일 때 b520»)은 정반대였다** — AU170 정정.
    /// 우리는 항상 156개 전부 배치하므로 `+0x164 == +0x4c` 가 참 → **⑤-a** 경로다.
    ///
    /// `FUN_1004b520` 은 그룹 구간(`+0x160..+0x164`)만 LP 로 풀어 해를 적용한다
    /// (`FUN_1004a4b0` → `FUN_1004a520` → `FUN_1004a140`).
    ///
    /// `true` 면 회차 루프 끝에서 `group_span` 을 «진행 구간» 으로 설정한 LP 를 한 번 더
    /// 돌린다. 구간은 `advance_tail` 로 구한 «목표 폭 이하 마지막 위치» 를 끝으로 삼는다.
    pub canonical_group_lp: bool,
    /// ★★★ §12.1.62 AU171 — ⑤-b «부품 단위 일괄 이동» (`FUN_1004bae0`).
    ///
    /// 원천 회차 루프의 Complete 이후 분기에서 «미배치가 남았을 때» 타는 경로다.
    /// 우리는 항상 전부 배치하므로 기본은 ⑤-a 이지만, **대조 실험**과
    /// «⑤-b를 ⑤-a와 함께 돌리는» 조합을 위해 개별 스위치를 둔다.
    pub canonical_family_lp: bool,
    /// ★★★ §AU185 — `+0x37c`(허용오차 동적계산). 켜지면 `+0x58`(난수 진동 폭)을 **2배**로
    /// 하고(`chunk_002.c:10474~10476`), `_DAT_101547a8`(−1000.0) 가드도 함께 쓴다.
    /// 원천은 모드 4 블록에서만 1 로 켠다(`chunk_001.c:7530~7531` / 복원 `7554`).
    /// 기본 **꺼짐** — 배포 경로 무손상.
    pub canonical_dyn_tol: bool,
    /// ★★ §12.1.45 AU152 — 바퀴 사이에 **미배치 목록을 국소 재정렬**할지 여부.
    ///
    /// 원천 `FUN_1004b490` 은 배치 루프 **안에서 매 반복** `FUN_10028210`
    /// (키 `+0xac[i].x − +0x8c[i].x` qsort) 을 다시 호출한다. 그러나 우리의
    /// `Order::LeftFree`(AU66) 는 `bbox.x0` 기준 **초기 1회** 정렬이라
    /// AU151.4 실측에서 7242.75mm 로 악화됐다(−0.51pp) — 키가 아니라
    /// «정렬 시점» 이 원인이었다(AU152 동기).
    ///
    /// `true` 면 **배치가 진행된 뒤 남은 조각만** 재정렬한다. 정렬 키는 원천의
    /// `+0xac.x`(현재 작업 좌표 = 배치된 위치의 x) 대응으로, «부착 조각 위치에서
    /// 가까운(좌측) 순» 이다. `pending` 벡터만 대상이므로 이미 놓은 조각은
    /// 건드리지 않는다 — AU148 의 «전체 재배치»(`--reorder-rounds 8` = 71.021%)
    /// 처럼 파괴적이지 않다.
    pub canonical_local_reorder: bool,
    /// ★★ §12.1.46 AU153 — «원천 점수 함수» 사용 여부.
    ///
    /// 원천 `FUN_10028380`(12994~13009)의 점수는 **넓이가 아니라**
    /// `+0x90[i].x`(우향 여유) `+ +0xc0[i].y`(배치 y) 다. 기본 `false` 면 우리
    /// 종전 `inside_area`(현재 폭 안에 드는 넓이) 를 그대로 쓴다.
    pub canonical_score: bool,
    /// ★★ §12.1.47 AU154 — «키 정렬 + 누적 최대 이진 탐색» 범위 필터.
    ///
    /// 원천 `FUN_10028210`(12905~12948)이 **정렬 후** 다음 두 테이블을 만든다:
    /// - `+0xb0[i]` = `{키: +0xac[i].x − +0x8c[i].x, 원래 인덱스}` — 키 오름차순 qsort.
    /// - `+0xb4[i]` = `+0xc0[+0xb0[i].idx] + +0x90[+0xb0[i].idx]` 의 **누적 최대**
    ///   (`if +0xb4[i] < +0xb4[i-1] { +0xb4[i] = +0xb4[i-1] }`).
    ///
    /// 그리고 `FUN_10028380` 은 `FUN_10021cf0`(8630~8639)으로 **기준 조각 자신의**
    /// 임계값을 설정한 뒤:
    /// - `FUN_100284e0`: `+0xb4` 에서 `+0xf0` 미만인 마지막 위치 → `lo`
    /// - `FUN_10028530`: `+0xb0` 에서 `+0xf4` 를 넘지 않는 마지막 위치 → `hi`
    /// - 스캔 범위 = **`(lo, hi]`**
    ///
    /// 즉 «키가 기준의 점수 범위 안에 들고, 점수가 기준 이상인 조각» 만 본다.
    /// `true` 면 `global_candidates` 의 `wide_global` 창 대신 이 범위를 쓴다.
    pub canonical_scan: bool,
    /// ★★ §12.1.48 AU155 — «scan 범위 안에서만 원천 점수 적용».
    ///
    /// AU154 에서 `--canonical-scan`(범위 필터)은 단독 개선을 냈으나, AU153 의
    /// 점수 함수와 **동시에 켜면** 충돌했다(`scan + score` = 7427mm). 이유는 두
    /// 옵션이 독립이라 `score` 가 **전역 후보 전체**에 적용되기 때문이다.
    ///
    /// 원천 `FUN_10028380` 은 «정렬 키 + 누적 최대 이진 탐색» 으로 **좁힌 범위**를
    /// 얻은 뒤, 그 범위에서 다시 점수(`+0x90 + +0xc0`)로 판정한다 — 두 기제가
    /// **순차**다. `true` 면 정확히 그 순서를 따른다:
    ///   ① `canonical_scan` 으로 후보 조각 범위를 좁힌다.
    ///   ② **좁혀진 후보에 대해서만** `canon_score`(`y − x`) 를 쓴다.
    ///
    /// `canonical_score` 와 배타적으로 쓰도록, 이 옵션이 켜지면 점수 선택은
    /// `scan` 여부에 따라 갈린다.
    pub canonical_scan_score: bool,
    /// ★★ §12.1.49 AU156 — «y 축 쌍 검사» (`FUN_10028590`, 13093~13133).
    ///
    /// AU154 는 x 축(`FUN_10028380` → `+0xf0`/`+0xf4`) 범위 필터를 이식했다.
    /// 대응하는 **y 축 검사**가 `FUN_10028590` 에 있다:
    ///
    /// ```c
    /// if ( +0xf8 <= (+0xc0[param_3].y + +0x90[param_3].y)   // ① y 점수 하한
    ///   && (+0xac[param_3].y − +0x8c[param_3].y) <= +0xfc )  // ② y 키 상한
    /// ```
    ///
    /// - `+0xf8` = `+0xac[pivot].y − +0x8c[pivot].y` (`FUN_10021cf0`)
    /// - `+0xfc` = `+0x90[pivot].y + +0xc0[pivot].y` (`FUN_10021cf0`)
    /// - 그 뒤 `& 1`(배치완료) 여부로 두 조각을 정렬해 **«쌍»** 을 검증한다.
    ///
    /// 즉 x 축은 «단일 조각 스캔», y 축은 **«두 조각 쌍 검증»** 이다. `true` 면
    /// `feasible` 의 쌍 검사에 이 y 조건을 추가한다.
    pub canonical_yscan: bool,
    /// ★★★ §12.1.50 AU157 — «쌍 전용 y 축 검사» (`FUN_10028590` + `FUN_10028790`).
    ///
    /// AU156 은 y 축 검사를 `feasible`(후보 vs 기배치 **전체**)에 넣어 무효였다
    /// (9463.41mm). 원천을 재해독한 결과 **호출 구조가 다르다**:
    ///
    /// ```c
    /// // chunk_004.c:16477~16481 — 진짜 배치 루프
    /// while (+0x174 < +0x16c) {                       // 진행 경계 루프
    ///   iVar3 = (+0x168)[+0x174];                     // 작업 목록에서 조각
    ///   FUN_10028380((+400)[iVar3], 0, FUN_10028790); // ★ 콜백 = FUN_10028790
    /// }
    ///
    /// // FUN_10028380 — 이중 루프 + 콜백
    /// FUN_10021cf0(pivot);                            // pivot 임계값 (+0xf0/f4/f8/fc)
    /// lo = FUN_100284e0(pivot);                       // 하한 (하락 구간 건너뜀)
    /// for i in lo..pivot-1:  if (+0xf0 <= 점수)  콜백(pivot, i, idx);
    /// hi = FUN_10028530(pivot);                       // 상한
    /// for i in pivot+1..=hi:                    콜백(pivot, i, idx);
    ///
    /// // FUN_10028790 — 콜백
    /// if (FUN_10028590(기준, i, 후보) == 0) return 0; // ★ y 축 «쌍» 검사
    /// if ((+0xe8[기준][후보] & 2) == 0)                // 이미 거부된 쌍 아닌가
    ///    if (FUN_10028820(기준, 후보, tol) != 0)       // ★ 실제 배치 시도
    ///       +0xe8 쌍 비트 |= 0x10;
    /// ```
    ///
    /// **즉 y 축 검사는 «스캔 조건» 이 아니라 «쌍 콜백» 이다.** `true` 면
    /// `global_candidates` 의 쌍 생성부에서 **그 쌍(pivot↔후보조각)에만** y 조건을
    /// 적용한다 — AU156 처럼 전역 제약으로 쓰지 않는다.
    pub canonical_pair_y: bool,
    /// ★★★ §12.1.51 AU158 — «여유 테이블»(`+0x8c`/`+0x90`) 이식.
    ///
    /// AU154·AU157 이 «여유 테이블이 없어 임계값을 재현할 수 없다» 로 남긴 과제를 수행한다.
    /// 원천 `FUN_10022190`(8917~8960) 이 **매 단계(배치·LP·Complete) 직후**
    /// `FUN_10022270` 을 통해 호출하는 갱신 함수다:
    ///
    /// ```c
    /// void FUN_10022190(i):
    ///   if ((+0xe0[i] & 2) != 0) {              // 고정 조각
    ///      +0x8c[i] = {1,1};  +0x90[i] = {1,1}; return; }
    ///   // 좌측 여유 (+0x8c) — «벽까지 갈 수 있는 거리»
    ///   if (+0xac[i].x − +0x8c[i].x < +0x94[i].x)  +0x8c[i].x = +0xac[i].x − +0x94[i].x;
    ///   if (+0xac[i].y − +0x8c[i].y < +0x94[i].y)  +0x8c[i].y = +0xac[i].y − +0x94[i].y;
    ///   // 우측 여유 (+0x90)
    ///   if (+0x90[i].x + +0xac[i].x > +0x98[i].x)  +0x90[i].x = +0x98[i].x − +0xac[i].x;
    ///   if (+0x90[i].y + +0xac[i].y > +0x98[i].y)  +0x90[i].y = +0x98[i].y − +0xac[i].y;
    /// ```
    ///
    /// `+0xac` = 현재 배치 좌표, `+0x94` = 좌측 한계(원단 왼쪽·아래 벽),
    /// `+0x98` = 우측 한계(원단 오른쪽·위 벽). **누적 min** 으로 유지된다.
    ///
    /// 그리고 「최상위 회차 루프」 `FUN_1004ae60`(20415~20433) 이
    /// `배치 → FUN_10022270(0x100) → LP → FUN_10022270(0x100) → Complete →
    ///  FUN_10022270(0x100)` 순으로 **각 단계 직후 갱신**한다.
    /// `true` 면 AU154 의 scan 필터가 «좌측 끝 x» 대신 **정확한 여유값**을 쓴다.
    pub canonical_slack: bool,
    /// ★★★ §12.1.82 AU201 — **«제약 여유 침식»** (원천 `FUN_1005f170`, 평면 배치 전용).
    ///
    /// 원천은 시작점을 하나 처리할 때마다, 그 시작점 조각에 **제약이 있으면**
    /// 여유 테이블을 **누적 감소**시킨다. 어셈블리 (`1005f263~1005f2c4`):
    /// ```asm
    /// mov   edx, [esi + 0x21c]        ; 제약 테이블
    /// lea   ecx, [ebp + ebp*2]        ; ecx = 조각 * 3
    /// fild  dword ptr [edx + ecx*8]   ; ST0 = 제약개수[조각]  (int → float)
    /// fldz                            ; 0 과 비교
    /// fcomp st(1) / test ah,0x41
    /// jne   +fadd 0.5                 ; 0 이하 → 올림
    ///       fsub 0.5                  ; 양수   → 내림
    /// call  1014a3d0                  ; ★ 반올림(ST0 → int)
    /// cmp   eax, 100000 / 클램프
    /// neg   eax                       ; ★ 부호 반전
    /// add   [esi+0x8c][i*8],   eax    ; ★ 좌여유 x += (−값)
    /// add   [esi+0x8c][i*8+4], eax    ; ★ 좌여유 y += (−값)
    /// add   [esi+0x90][i*8],   eax    ; ★ 우여유 x += (−값)
    /// add   [esi+0x90][i*8+4], eax    ; ★ 우여유 y += (−값)
    /// ```
    ///
    /// **즉 감소량 = `round(제약개수 ∓ 0.5)` 의 부호 반전 = `−ceil/floor(제약개수)`**
    /// 이고, **같은 값이 좌·우 여유 네 칸 모두에 더해진다.**
    /// `_DAT_10154458 = 0.5` 가 그 `fsub`/`fadd` 상수다(AU198 확보).
    ///
    /// ★ AU201 — 우리 `refresh_slack` 은 종전 `min(b.x0, DELTA)` **고정 min** 이었다.
    ///   원천은 여기에 **누적 감소**가 더해진다 — 이 플래그가 그 축을 켠다.
    ///   기본 **꺼짐**이라 배포 경로(`--best`)는 손상되지 않는다.
    pub canonical_slack_erode: bool,
    ///
    /// 원천 `FUN_100284e0`/`FUN_10028530` 의 이진 탐색이 좁히는 범위에 대응한다.
    pub canonical_window: usize,
    /// ★ 재작성 3단 교대에서 ② LP 압축 설정 (§12.1.34 AU115).
    ///
    /// 원천 `FUN_1004ae60` 은 `FUN_1004b490`(배치) → `FUN_1004aee0`(LP) → `FUN_10029cc0`
    /// (Complete) 을 **매 회차** 돈다. 여기서는 ①② 만 쓰고 ③ 은 `pipeline` 에 맡긴다.
    pub canonical_lp: Option<Box<crate::compact::CompactConfig>>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            // §12.1.27 AO5 정정: 정답 데이터 실측에서 `AreaDesc + Always + 워크리스트 미사용`
            // 이 최선(75.566%)이었다. `Declaration + Always + 워크리스트 사용`(구 기본값)은
            // 69.753% 로 최하위권이다 — 순서 하나로 5.8pp 차이.
            order: Order::AreaDesc,
            time_limit: Duration::from_secs(20),
            deadline: None,
            nfp_tol: 0,
            use_worklist: false,
            // ★ §12.1.86 AU205 — 기본 꺼짐(배포 경로 무손상).
            canonical_pair_converge: false,
            global: Global::Always,
            seed_trials: SEED_TRIALS,
            adaptive_seeds: false,
            adaptive_uncapped: false,
            restart_loop: false,
            reverse_mode: false,
            bidir_swap: false,
            native_lp: false,
            native_prop: false,
            seed_list_cap: 0,
            wide_global: true,
            global_window: 2000,
            reorder_rounds: 0,
            canonical: false,
            canonical_rounds: 0,
            canonical_midpass: false,
            canonical_midpass_late_global: false,
            canonical_slice_thr: 0,
            canonical_cap_mult: 3,
            canonical_cap_permille: 0,
            canonical_cap_grow: false,
            canonical_group_lp: false,
            canonical_family_lp: false,
            canonical_dyn_tol: false,
            canonical_local_reorder: false,
            canonical_score: false,
            canonical_scan: false,
            canonical_scan_score: false,
            canonical_yscan: false,
            canonical_pair_y: false,
            canonical_slack: false,
            canonical_slack_erode: false,
            canonical_cap_to_lp: false,
            canonical_window: 8,
            canonical_lp: None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Pending,
    Placed,
    Held,
    Failed,
}

#[derive(Clone, Debug, Default)]
pub struct Stats {
    pub nfp_computed: usize,
    pub nfp_hits: usize,
    /// NFP **실제 계산에** 쓴 시간(ms) = `nfp_us / 1000`. 캐시 히트는 0 이다 — 첫 계산
    /// 비용만 잡힌다. ★ Draft 구간의 **부분집합**이다. 전체 시간에서 빼지 말 것.
    pub nfp_ms: u128,
    /// 같은 값의 µs 누적. 계산 1회는 보통 1ms 미만이라 ms 단위로 더하면 전부 0 으로
    /// 잘렸다(`as_millis` 의 반올림 아님 절사).
    pub nfp_us: u128,
    /// NFP 계산에 실패한 (고정, 이동) 형상 쌍 수. 실패한 쌍은 다시 계산하지 않고, 그
    /// 쌍이 걸린 후보만 실행 불가로 본다(조각 전체를 보류로 보내지 않는다).
    pub nfp_errors: usize,
    pub error_samples: Vec<String>,
    pub candidates: usize,
    pub evaluations: usize,
    pub eval_cap_hits: usize,
    /// 전역 접촉 쌍 워크리스트 후보로 놓인 조각 수.
    pub global_placed: usize,
    pub pairs: usize,
    pub held_first_pass: usize,
    pub failed: usize,
    /// §0.1.2 재시작 횟수 N. 조각 200개 이하면 1(재시작 없음).
    pub restarts: usize,
    /// 다중 시작점(§12.1.33 AU27)에서 실제로 시도한 시작점 수. `SeedSweep` 이 아니면 0.
    pub seed_trials: usize,
    /// ★ §12.1.34 AU129.1 — «개선 판정 → 최적 복원» 게이트가 돈 회차 수 (`+0x7c`).
    pub canonical_rounds: usize,
    /// ★ §12.1.34 AU129.2 — «개선 없음» 으로 최적 배치를 복원한 횟수 (`FUN_1004ac30` 호출).
    pub canonical_reverts: usize,
    /// ★ §12.1.34 AU139.2 — «부착 발생» 횟수 (`+0x80 == 0x50`).
    ///
    /// 원천 `FUN_1004c1a0` 이 이 상태에서 **반복을 계속**한다.
    pub attach_events: usize,
    /// ★ §12.1.34 AU139.2 — 마지막 «반복 계속 판정» 결과 (`FUN_1004c1a0`).
    pub canonical_continues: usize,
    pub elapsed_ms: u128,
}

pub struct Outcome {
    pub placements: Vec<Placement>,
    /// `placements[k]` 가 전역 워크리스트 후보로 놓였는가.
    pub via_global: Vec<bool>,
    pub states: Vec<State>,
    pub stats: Stats,
    pub timed_out: bool,
    /// 이 실행이 만든 접촉 쌍 (§12.1.33 AU39 — 다음 시작점 후보).
    pub pairs: Vec<(i128, usize, usize)>,
    /// ★★★ §12.1.83 AU202 — **«마지막으로 배치된 조각»** (원천 `FUN_1005f8e0` 의 `+0x158..+0x15c`).
    ///
    /// 원천 `FUN_10020a00(iVar2)` 는 **이 조각을 포함하는 접촉 쌍만** 모은다:
    /// ```c
    /// FUN_10020a00(param_1) {                       // param_1 = 방금 배치된 조각
    ///     in_EAX[3] = param_1;
    ///     for (i = 0; i < 쌍목록[1]; i++) {
    ///         piVar3 = 쌍목록[0] + i * 0x38;
    ///         if (piVar3[0xc] != 0                              // 쌍 유효
    ///             && (piVar3[0] == param_1 || piVar3[1] == param_1))   // ★ 포함
    ///             && (개수 != 상한 || FUN_10020980() != 0)) {          // 중복 방지
    ///             후보[개수++] = piVar3;
    ///         }
    ///     }
    /// }
    /// ```
    /// 우리는 `out.pairs` **전체** 를 훑고 있었다 — 원천은 **마지막 배치 조각 기준**이다.
    pub last_seed: Option<usize>,
    /// ★★★ §12.1.84 AU203 — **진행 경계** (원천 `+0x15c`).
    ///
    /// `FUN_10020b60` 의 «재후보 유지» 판정이 `cmp [edx + 0x15c], ecx / jl` 로
    /// 이 값을 본다 — **경계 밖 조각은 다음 회차 후보에서 탈락**한다.
    pub tail_end: i32,
    /// ★★★ §AU184 — 조각별 `+0xe0` 상태 바이트 (원천 `FUN_1005f3f0` 시작점 자격).
    /// `bits[k][k]` 의 대각선 사본이다. 부품 슬롯별 조각 인덱스로 접근한다.
    pub state_bits: Vec<u8>,
    /// 이 실행이 채운 NFP 캐시 — 압축 LP·Complete 가 이어 쓴다.
    pub cache: NfpCache,
}

/// §0.1.2: `조각수 > 200` 이면 `조각수/200 + 1`, 아니면 1.
pub fn restart_count(n_pieces: usize) -> usize {
    if n_pieces > RESTART_SCALE { n_pieces / RESTART_SCALE + 1 } else { 1 }
}

/// NFP 바깥 경계 고리의 모서리들 (월드 좌표).
fn outer_edges(n: &Nfp, off: Pt) -> Vec<(Pt, Pt)> {
    let mut e = Vec::new();
    for lp in n.outer_loops() {
        let m = lp.len();
        for i in 0..m {
            e.push((lp[i] + off, lp[(i + 1) % m] + off));
        }
    }
    e
}

/// 격자에 없는 교점은 둘러싼 네 격자점을 모두 후보로 넣는다. 어느 쪽이 겹침 없이
/// 닿는지는 실행 가능성 검사가 정한다.
fn push_rounded(out: &mut Vec<Pt>, x: f64, y: f64) {
    let (x0, x1) = (x.floor() as i64, x.ceil() as i64);
    let (y0, y1) = (y.floor() as i64, y.ceil() as i64);
    for (a, b) in [(x0, y0), (x1, y0), (x0, y1), (x1, y1)] {
        out.push(Pt::new(a, b));
    }
}

fn seg_box(a: Pt, b: Pt) -> BBox {
    BBox { x0: a.x.min(b.x), y0: a.y.min(b.y), x1: a.x.max(b.x), y1: a.y.max(b.y) }
}

/// NFP 경계와 원단 벽(IFP 의 `y = 0`, `y = H − h`, `x = 0`)의 교점 — 조각과 벽에 동시에
/// 닿는 자리. 원단 경계도 같은 파이프라인에 넣는다(§3.3.2, 경계 전용 분기 없음).
fn fabric_hits(edges: &[(Pt, Pt)], y_top: i64, out: &mut Vec<Pt>) {
    for &(a, b) in edges {
        if a.y == 0 || a.y == y_top || a.x == 0 {
            out.push(a);
        }
        for y in [0, y_top] {
            if (a.y - y) * (b.y - y) < 0 {
                let x = a.x as f64 + (y - a.y) as f64 * (b.x - a.x) as f64 / (b.y - a.y) as f64;
                push_rounded(out, x, y as f64);
            }
        }
        if a.x * b.x < 0 {
            let y = a.y as f64 + (0 - a.x) as f64 * (b.y - a.y) as f64 / (b.x - a.x) as f64;
            push_rounded(out, 0.0, y);
        }
    }
}

/// 두 NFP 경계의 교점 — 두 조각에 동시에 닿는 자리. `region` 은 두 NFP bbox 의 교집합.
fn pair_hits(ea: &[(Pt, Pt)], eb: &[(Pt, Pt)], region: BBox, out: &mut Vec<Pt>) {
    if region.x0 > region.x1 || region.y0 > region.y1 {
        return;
    }
    let ea2: Vec<&(Pt, Pt)> = ea.iter().filter(|e| seg_box(e.0, e.1).overlaps(&region)).collect();
    if ea2.is_empty() {
        return;
    }
    let eb2: Vec<&(Pt, Pt)> = eb.iter().filter(|e| seg_box(e.0, e.1).overlaps(&region)).collect();
    for &&(a0, a1) in &ea2 {
        let sa = seg_box(a0, a1);
        for &&(b0, b1) in &eb2 {
            if !sa.overlaps(&seg_box(b0, b1)) {
                continue;
            }
            if let Some((x, y)) = segment_intersection(a0, a1, b0, b1) {
                push_rounded(out, x, y);
                if out.len() >= CANDIDATE_GEN_CAP {
                    return;
                }
            }
        }
    }
}

fn intersect(a: BBox, b: BBox) -> BBox {
    BBox { x0: a.x0.max(b.x0), y0: a.y0.max(b.y0), x1: a.x1.min(b.x1), y1: a.y1.min(b.y1) }
}

/// 평가 대기 후보: (현재 폭 안쪽 넓이, 위치, 회전 형상, 전역 워크리스트 출처인가).
type Cand = (f64, Pt, usize, bool);

struct Ctx<'a> {
    inst: &'a Instance,
    cfg: Config,
    cache: NfpCache,
    nfp_failed: HashSet<(usize, usize)>,
    coll_box: Vec<BBox>,
    placed: Vec<Placement>,
    /// 접촉 쌍 워크리스트 (§3.3.1): (대표점 거리², i, j), 거리² 오름차순. 배치 때마다
    /// 새 조각과의 쌍만 증분으로 끼워 넣는다.
    pairs: Vec<(i128, usize, usize)>,
    /// 현재 사용 폭 (격자) — 배치된 원본 외곽선의 최대 x.
    used_w: i64,
    deadline: Instant,
    stats: Stats,
    /// ★ 재작성 모드에서 «아직 놓지 않은 조각» 을 판정하는 상태 배열 (§12.1.34 AU101).
    states: Vec<State>,
    /// ★★ 조각 쌍 비트 행렬 `+0xe8` (§12.1.34 AU111).
    ///
    /// 원천은 조각마다 1바이트를 두고 **쌍 관계를 누적**한다:
    ///
    /// | 비트 | 값 | 의미 | 설정자 | `& 6` 에서 |
    /// |---|---|---|---|---|
    /// | 0 | 1 | 배치 완료 | `FUN_1001fe90` | **지움** |
    /// | 1 | 2 | 부착 | `FUN_10028590` | 보존 |
    /// | 2 | 4 | **쌍(겹치지 않는 순서)** | `FUN_10021f60`, `FUN_10028ad0` | **보존** |
    /// | 3 | 8 | 변형 | `FUN_100218a0` | **지움** |
    /// | 4 | 16 | 각도 (변형 전용) | `FUN_10028790` | 보존 |
    ///
    /// `FUN_10028210` 이 **매 반복** `&= 6` 을 하므로 배치완료·변형은 리셋되고
    /// **부착·쌍은 반복을 넘어 누적**된다. 이 누적이 회차마다 후보 판정을 바꾸는 원동력이다.
    bits: Vec<Vec<u8>>,
    /// ★★★ `+0x7c` — «회차 카운터» (§12.1.34 AU129.1).
    ///
    /// 원천 `FUN_1004ad80` 이 루프 진입마다 `*(int *)(in_EAX + 0x7c) += 1` 한다.
    /// `FUN_1004a640` 이 `+0x9c[+0x7c − 1]` 을 읽으므로 **1-based** 다.
    round: i32,
    /// ★★★ `+0x9c` — «회차별 폭 이력 배열» (§12.1.34 AU129.1).
    ///
    /// 원천 `FUN_1004b520` 이 `*(double *)(+0x9c + +0x7c * 8) = (double)local_28` 로
    /// 회차마다 계산된 폭을 기록한다. `FUN_1004a640` 이 `-8 + +0x7c * 8` (즉 `[+0x7c−1]`)을
    /// 읽어 **직전 회차 폭**으로 개선을 판정한다. 인덱스 0 은 원천에서 `+0x40[0x10]`
    /// (초기값)으로 채워진다(`11101`).
    hist_w: Vec<i64>,
    /// ★★★ `+0xa4` — «현재 최적 폭» (§12.1.34 AU129.1).
    ///
    /// `FUN_1004a640` 이 기준값을 만들 때 쓰고, `FUN_1004b520` 뒤
    /// (`20736`)에 `+0x9c[회차−1]` 로 갱신된다.
    best_w: i64,
    /// ★★★ `+0xbc` — «최적 배치 백업 배열» (§12.1.34 AU129.2).
    ///
    /// `FUN_1004b520` 뒤에 `memcpy(+0xbc, +0xac, 4 × +0x4c)` 로 복사된다(`20737`).
    /// `FUN_1004ac30` 이 개선이 없을 때 `memcpy(+0xac, +0xbc, ...)` 로 **복원**한다.
    /// `+0xac` = 현재 배치(`placed`)에 대응한다.
    best_backup: Vec<Placement>,
    /// ★★ `+0x164` — «미배치 구간의 끝» (§12.1.34 AU128.1).
    ///
    /// `FUN_10025b30` 이 `+0x15c`(배치 경계)에서 `+0xe0[i] & 1`(배치완료) 비트를
    /// 건너뛰며 찾은 위치를 기록한다. `+0x164 − +0x160` = «아직 배치 안 된 개수»(`11132`).
    /// `FUN_1004a640` 의 분기식이 `+0x164 == +0x4c`(전부 배치) 를 검사한다.
    tail_end: i32,
    /// ★★ `+0x188` — «모드 플래그(char)» (§12.1.34 AU128.3).
    ///
    /// `+0x20[0x14] == 0 || +0x188 != 0 || +0x164 == +0x4c` 가
    /// `b520`/`bae0` 분기(`20401`)와 `FUN_1004a640`(`20041`)에 **동일하게** 나타난다.
    mode_flag: bool,
    /// ★ §12.1.43 AU147 — «목표 폭 상한» (mm, 그리드 단위). 배치 후보를 이 폭 이내로 제한한다.
    /// 원천 `FUN_10025950` 의 «기준 폭» 대응. `None` = 제한 없음.
    target_cap: Option<i64>,
    /// ★ §12.1.43 AU145.5 — «전역 폴백 지연» 바퀴 수. `Some(w)` 면 `midpass_wave < w` 인
    /// 동안 전역 후보를 쓰지 않아 «배치 도중» 상태를 만든다. `None` = 제한 없음.
    midpass_off_waves: Option<usize>,
    /// 현재 배치 바퀴 인덱스 (`midpass_off_waves` 판정용).
    midpass_wave: usize,
    /// ★★★ `+0x194` — «부품별 누적 오프셋» 테이블 (§12.1.34 AU133.1).
    ///
    /// `FUN_100243c0` 이 재계산하고 `FUN_10026520` 이 `+0x78` 계산에 쓴다.
    /// AU138.7 에서 **형상(shape)을 부품으로** 매핑해 배선했다.
    part_offsets: compact::PartOffsets,
    /// ★★★ §12.1.51 AU158 — «여유 테이블» (`+0x8c` 좌 / `+0x90` 우).
    ///
    /// 원천 `FUN_10022190`(8917~8960). 매 단계(배치·LP·Complete) 직후 `FUN_10022270`
    /// 을 통해 갱신된다. 값 = «조각이 벽까지 갈 수 있는 거리» (누적 min).
    /// `canonical_slack` 이 켜지면 AU154 의 scan 필터가 «좌측 끝 x» 대신 이 값을 쓴다.
    slack_left: Vec<(i64, i64)>,
    slack_right: Vec<(i64, i64)>,
    /// ★★ `+0x78` — «목표 폭» = `part_offsets.offset[부품] + used_w` (§12.1.34 AU134.1).
    target_w: i64,
    /// ★★★ §12.1.34 AU139.2 — `FUN_1004c1a0` «반복 계속 판정» 결과.
    ///
    /// `true` 면 원천은 **안쪽 회차 루프를 한 번 더 돈다** (`+0x7c < +0x394 = 20` 이고
    /// 수렴/정체가 아닐 때).
    cont: bool,
    /// ★ §12.1.41 AU141.5 — `+0x140` «변형 테이블» 대응. `[3i+1]` 이 «고정» 판정에 쓰인다.
    /// 전부 0 이면 «비고정» 이며, 종전 동작과 동일하다.
    variant: Vec<u8>,
    /// ★★★ §12.1.77 AU180 — `+0xc8` «LP 이동 유효» 플래그.
    ///
    /// `FUN_1004a140` 이 dx·dy 각각을 `FUN_10049f00` 으로 판정해 **참이면 1** 을 쓴다:
    /// ```c
    /// if (0.8 * v < Δ)      { *p = round(Δ); if (1000000 < *p) *p = 1000000; return 1; }  // +0x3a8=0.8, +0x3a4
    /// if (Δ < 0.5 * v)      { *p = round(Δ); if (*p < 10000) { *p = 10000; return 0; } }  // +0x3b0=0.5, +0x3a0
    /// return 0;
    /// ```
    /// `should_continue` 의 `else if (mode != 1 || +0xc8 == 0)` 분기가 이 값을 본다 —
    /// **1이면 «개선 판정을 건너뛰고 무조건 계속»** 이다.
    lp_moved_valid: bool,
    /// ★★★ §12.1.77 AU180 — `+0xcc` «이동 임계 초과» 플래그.
    ///
    /// `FUN_1004a140` 끝에서:
    /// ```c
    /// if (+0xcc == 0 && |dx| <= +0x39c(30) && |dy| <= 30)  +0xcc = 0;   // 수렴 유지
    /// else                                                  +0xcc = 1;   // 이동 있음
    /// ```
    /// **한번 1이 되면 회차 리셋 전까지 유지**된다(`JNZ` 로 즉시 점프).
    iter_move_flag: bool,
    /// ★★★ §12.1.77 AU180 — **회차 시작 시점의 좌표 스냅샷**.
    ///
    /// 원천 `FUN_1004a140` 은 LP 해를 **좌표 델타**로 적용하므로 `+0xc8`/`+0xcc` 를
    /// «직전 좌표 대비 이동량» 으로 판정한다. 우리 `restart_improve` 는 절대 좌표를
    /// ★★★ §12.1.77 AU180 — 회차 시작 좌표 스냅샷 (비어 있으면 아직 스냅샷 없음).
    round_start_pos: Vec<(i64, i64)>,
    /// ★★★ §12.1.78 AU181 — **판정 전용 회차 카운터** (원천 `+0x7c`).
    ///
    /// `ctx.round` 와 분리한 이유: `round` 는 `span = np/(round+1)`·
    /// `jitter = 75+75*(round%4)` 등 «부수 계산» 에도 쓰여, 매 회차 올리면
    /// `group_span` 이 어긋나 LP 열 수가 맞지 않아 패닉한다
    /// (실측: `compact.rs:991 index out of bounds: len 157, index 157`).
    /// 원천에서도 `+0x7c` 는 «회차 상한 판정» 전용이고, `+0x58`(난수 진동)은 별도다.
    iter_round: i32,
}

/// ★ `_DAT_101544a0` — «개선 허용 임계 ε» (§12.1.34 AU129.1).
///
/// `FUN_1004a640` 이 `best_w + ε` 를 기준으로 삼고, 그 외에는 `ε` 자체를 기준으로 쓴다.
/// AU119 에서 판독한 `_DAT_101546a8 = −1e−06` 과 같은 계열의 «정체 하한» 상수다.
pub const EPS_IMPROVE: i64 = 0;

/// ★★★ §12.1.77 AU180 — `DAT_1015c428 + 0x39c` = **`0x1e`(30)** — «이동 임계».
///
/// `FUN_1004a140` 끝에서 `|dx| > 30 || |dy| > 30` 이면 `ctx+0xcc = 1`(«이동 있음»),
/// 둘 다 이하면 `+0xcc = 0`(«수렴 유지») 로 둔다. 좌표 1단위 = 0.01mm 이므로 0.30mm 다.
pub const MOVE_THRESH_GRID: i64 = 30;

/// 쌍 비트 (§12.1.34 AU111).
pub const BIT_PLACED: u8 = 1;
pub const BIT_ATTACH: u8 = 2;
pub const BIT_PAIR: u8 = 4;
pub const BIT_VARIANT: u8 = 8;
/// `FUN_10028210` 의 `&= 6` 마스크 — 배치완료(1)·변형(8)만 지운다.
pub const BIT_KEEP: u8 = 6;

/// ★★★ §AU184 — `+0xe0` 조각 상태 바이트의 «시작점 자격» 비트 (`FUN_1005f3f0` 조건).
///
/// 원천 조건식 (`chunk_004.c:16588~16602`):
/// ```c
/// bVar1 = +0xe0[k];
/// if (bVar1 >= 0                                   // bit7(0x80) 미설정
///     && (((bVar1 & 4) && !(bVar1 & 0x40))         // BIT_PAIR && !BIT_CONSUMED
///         || (bVar1 & 0x10)))                      // 또는 BIT_LINKED
/// { FUN_1008c990(k); +0xe0[k] |= 0x80; }           // 시작점 등록 + 중복방지
/// ```
///
/// `BIT_PAIR`(4)는 이미 `+0xe8` 쌍 행렬에 쓰이므로 **대각선 `bits[k][k]`** 에
/// 같은 비트를 세워 `+0xe0` 을 겸용한다. `FUN_1008c990` 은 단순 동적배열 push 라
/// 이 두 비트만 있으면 조건이 완성된다(bit3 `0x08` 는 조건에 없다 — AU184 확인).
pub const BIT_LINKED: u8 = 0x10;
/// «소모됨» — 설정되면 시작점 후보에서 **제외**된다.
pub const BIT_CONSUMED: u8 = 0x40;
/// «이미 시작점으로 뽑힘» — 중복 방지.
pub const BIT_SEEDED: u8 = 0x80;

/// ★ `FUN_10021f60` (§12.1.34 AU112) — «겹치지 않는 순서 관계» 를 쌍으로 표시한다.
///
/// 원천:
/// ```c
/// cVar3 = 변형플래그A[in_EAX];  cVar2 = 변형플래그B[in_EAX];
/// if (param_3 && param_4) { if (!cVar3) goto y검사; if (cVar2) goto 쌍설정; }
/// if (cVar3) {
///     if (+0xc0[param_2].x < +0xac[in_EAX].x ||
///         +0xc0[in_EAX].x  < +0xac[param_2].x) goto 쌍설정;   // x 축 비겹침
/// }
/// y검사:
/// if (!param_4 || !cVar2) return;
/// if (+0xac[in_EAX].y <= +0xc0[param_2].y && +0xac[param_2].y <= +0xc0[in_EAX].y) return;
/// 쌍설정: +0xe8[param_2][in_EAX] |= 4;  +0xe8[in_EAX][param_2] |= 4;
/// ```
///
/// **우리 입력은 변형이 없어(AU83) `cVar2`·`cVar3` 가 항상 0** 이다. 그러면 `param_3 != 0 &&
/// param_4 != 0` 분기에서 `cVar3 == 0` 이므로 `goto y검사` 로 가고, 거기서 `cVar2 == 0` 이라
/// **즉시 반환**한다. 즉 **변형 없는 입력에서는 쌍이 하나도 설정되지 않는다.**
///
/// (`FUN_10028ad0` 은 다른 경로다 — `+0x2c0` 목록의 쌍에 직접 `|= 4` 를 한다. 그쪽은
/// 입력이 주는 «고정 쌍» 이고, `FUN_10021f60` 은 «계산된 쌍» 이다.)
///
/// 따라서 이식 대상은 **`FUN_10028ad0` 의 «입력 쌍» + 매 반복 누적 보존** 이다.
/// 우리 입력의 «부품» 관계(`domain`/`family_id`)가 그 역할을 한다.
fn pair_flag(a: &Placement, b: &Placement, sa: &crate::io::Shape, sb: &crate::io::Shape) -> bool {
    // x 축 비겹침: a 의 우측 끝이 b 의 좌측 시작 이하이거나 그 반대.
    let a_left = a.pos.x + sa.bbox.x0;
    let a_right = a.pos.x + sa.bbox.x1;
    let b_left = b.pos.x + sb.bbox.x0;
    let b_right = b.pos.x + sb.bbox.x1;
    if a_right <= b_left || b_right <= a_left {
        return true;
    }
    // y 축 비겹침.
    let a_bot = a.pos.y + sa.bbox.y0;
    let a_top = a.pos.y + sa.bbox.y1;
    let b_bot = b.pos.y + sb.bbox.y0;
    let b_top = b.pos.y + sb.bbox.y1;
    a_top <= b_bot || b_top <= a_bot
}

impl<'a> Ctx<'a> {
    /// ★★★ §12.1.51 AU158 / §12.1.54 AU160 / §12.1.55 AU161 — 여유 테이블 (`FUN_10022190`).
    ///
    /// 원천 `FUN_10021800` 이 `+0x94`/`+0x98` = «현재 위치 ± delta» 를 채우고
    /// (delta = «무한대 근사» 1,600,000mm, `FUN_10016ca0`), `FUN_10022190` 이 누적 min:
    ///
    /// ```c
    /// if ((+0xe0[i] & 2) != 0) { +0x8c[i] = {1,1}; +0x90[i] = {1,1}; return; }  // 고정
    /// if (배치x − +0x8c[i].x < +0x94[i].x)  +0x8c[i].x = 배치x − +0x94[i].x;   // 좌측
    /// if (+0x98[i].x < +0x90[i].x + 배치x)  +0x90[i].x = +0x98[i].x − 배치x;   // 우측
    /// ```
    ///
    /// `+0x94 = 배치x − delta`, `+0x98 = 배치x + delta` 이고 delta 는 «무한대» 이므로:
    /// - 좌측 여유 = `배치x − 0` = **배치x** (좌측 벽까지 거리, 누적 min)
    /// - 우측 여유 = `배치x + delta − 배치x` = **delta** (무한대 → 초기 무제한)
    ///
    /// **AU161 ★ 우측 여유 추적**: 원천은 `+0x90` 도 매 갱신마다 누적 min 을 취한다.
    /// delta 무한대이므로 «우측 벽» 은 **현재 사용 폭**(`used_w`)이 된다 — 즉
    /// **우측 여유 = `used_w − 배치x` 의 누적 min** 이다. 배치가 넓어질수록 좁아진다.
    ///
    /// 사용처 (AU159.2·AU161 해독):
    /// - `FUN_1004a310`: LP **변수 상자** (`lo`, `hi`)
    /// - `FUN_1001fe20`: **쌍 사이 총 이동 거리** (`+0x8c[A] + +0x90[B]`)
    /// - `FUN_10021cf0`: 스캔 임계값 `+0xf0 = 배치x − 좌여유`, `+0xf4 = C0 + 우여유`
    fn refresh_slack(&mut self) {
        // ★★★ §12.1.58 AU164 정정 — 원천 `FUN_10025770` 은 여유 초기값을 `+0x390`=20000
        //   으로 두고, `FUN_10022190` 이 **누적 min** 을 취한다:
        //   ```c
        //   if (배치x − +0x8c) < +0x94 (= 배치x − delta)   +0x8c = 배치x − +0x94
        //   if (+0x98 < +0x90 + 배치x)                      +0x90 = +0x98 − 배치x
        //   ```
        //   delta = «무한대 근사»(1,600,000mm) → `+0x94 = 배치x − delta`(음수),
        //   `+0x98 = 배치x + delta`(양수). 따라서 «배치x − +0x94 = delta» 는 **줄어들지
        //   않고**, 초기값 20000 이 그대로 유지된다. 우측도 «+0x98 − 배치x = delta».
        //
        //   AU161 에서 `right_wall = used_w` 를 넣은 것은 오독 — 원천은 «현재 폭» 이
        //   아니라 `배치x + delta` 를 쓴다. 그래서 우리 여유가 6000mm 대로 부풀어
        //   `hi_in`(75~300) 을 압도했다.
        //
        //   우리 대응: **delta 를 «초기 여유 20000» 으로 삼고 누적 min 만 취한다.**
        //   → 여유 = `min(20000, 배치 여유)`.
        const DELTA: i64 = 20_000; // `+0x390`, 200mm
        for p in &self.placed {
            let b = self.inst.shapes[p.shape].bbox.translate(p.pos);
            // ★ 테이블은 **조각 인덱스** 기준이다(크기 `pieces.len()`, `slack_boxes` 와
            //   `compact::build` 의 `v.get(p.piece)` 가 같은 축). 종전에는 `item_id` 로
            //   읽어 같은 부품의 복제본이 한 칸을 공유했고 `slack_boxes` 는 조각 인덱스로
            //   내보내 축이 어긋났다.
            let i = p.piece;
            // 좌측 여유 `+0x8c` = «배치x − 좌측한계(0)», 상한 delta.
            self.slack_left[i].0 = self.slack_left[i].0.min(b.x0.min(DELTA));
            self.slack_left[i].1 = self.slack_left[i].1.min(b.y0.min(DELTA));
            // 우측 여유 `+0x90` = «+0x98 − 배치x». 원천 `+0x98 = 배치x + delta` 이므로
            //   **`+0x90 = delta` 가 상수로 유지된다**(줄어들지 않는다).
            //
            // ★ AU193 — 종전 `used_w` 는 위 909~911행 주석이 «오독» 이라 적어놓고도
            //   코드에 남아 있었다(주석만 고치고 코드는 안 고침). 원천대로 delta.
            let right_gap = DELTA;
            self.slack_right[i].0 = self.slack_right[i].0.min(right_gap);
            self.slack_right[i].1 = self
                .slack_right[i]
                .1
                .min(self.inst.strip_height.saturating_sub(b.y1).min(DELTA));
        }
    }

    /// ★★★ §12.1.82 AU201 — **«제약 여유 침식»** (원천 `FUN_1005f170` 뒷부분).
    ///
    /// ## 원천 (어셈블리 `1005f263~1005f2c4`, 전량 해독)
    ///
    /// 시작점 조각 `i` 를 처리할 때마다, 그 조각의 플래그 `+0xe0[i]` 에 **bit2(0x04)**
    /// 가 서 있으면(= `FUN_100218a0` 이 «제약 있음» 으로 표시) 여유를 깎는다:
    ///
    /// ```asm
    /// mov   edx, [esi + 0x21c]        ; 제약 테이블 (조각당 0x18)
    /// lea   ecx, [ebp + ebp*2]        ; ecx = i * 3          ← ★ 8바이트 간격 ×3 = 0x18
    /// fild  dword ptr [edx + ecx*8]   ; ST0 = 제약개수[i]     ← ★ 항목 첫 4바이트
    /// fldz  /  fcomp st(1) / test ah,0x41
    /// jne   → fadd 0.5                ; 0 이하 → 올림
    ///        → fsub 0.5               ; 양수   → 내림
    /// call  1014a3d0                  ; ★ x87 반올림 (callers=197, 난수 아님)
    /// cmp eax,100000 / 클램프
    /// neg eax                         ; ★ 부호 반전
    /// add [esi+0x8c][ebp*8],   eax    ; 좌여유 x += (−값)
    /// add [esi+0x8c][ebp*8+4], eax    ; 좌여유 y += (−값)
    /// add [esi+0x90][ebp*8],   eax    ; 우여유 x += (−값)
    /// add [esi+0x90][ebp*8+4], eax    ; 우여유 y += (−값)
    /// ```
    ///
    /// ★ **감소량은 난수가 아니다.** AU201 초기에 `FUN_1014a3d0` 을 난수로 의심했으나,
    ///   디컴파일 본문이 `ROUND(in_ST0)` + x87 정밀도 보정이고 `callers=197` 인
    ///   **공용 반올림 래퍼**였다. 실체는 **제약 개수의 내림/올림**이다.
    ///
    /// ★ `_DAT_10154458 = 0.5` 가 그 `fsub`/`fadd` 상수다(AU198 확보).
    ///
    /// ★ **대상은 시작점 조각 하나뿐**(`ebp` = 현재 시작점)이고, **좌·우 네 칸 모두에
    ///   같은 값**이 더해진다. 우리 `refresh_slack` 은 배치된 조각 전체를 도는
    ///   «누적 min» 이므로, 이 함수는 그 위에 **얹히는 별도 축**이다.
    ///
    /// ## 대응
    ///
    /// `+0x21c[조각*3]` = `FUN_100218a0` 이 `piVar1[1]`(제약 개수)을 써 넣은 값이다.
    /// 우리에게 그 테이블이 없으므로 **부품의 제약 유무·개수에서 직접 만든다**.
    /// `constraint_counts[i]` 가 그 대응이고, 0 이면 원천의 `*piVar4 == 0` 과 같아
    /// **아무 일도 일어나지 않는다**(= 기본 동작 불변).
    fn erode_slack(&mut self, piece: usize, constraint_counts: &[i32]) {
        let c = constraint_counts.get(piece).copied().unwrap_or(0);
        if c <= 0 {
            return; // 원천 `if (플래그 & 4) != 0` 실패와 동치
        }
        // 원천: 양수면 `− 0.5` 후 반올림(= 내림), 0 이하면 `+ 0.5` 후 반올림(= 올림).
        //       그 뒤 부호를 뒤집어 «빼기» 로 만든다.
        let rounded = if c > 0 {
            (c as f64 - 0.5).round() as i64
        } else {
            (c as f64 + 0.5).round() as i64
        };
        let delta = -rounded.clamp(-100_000, 100_000);
        // ★ `piece` 는 조각 인덱스이고 테이블도 조각 인덱스 기준이다. 종전에는
        //   `self.placed[piece]` 를 거쳐 `item_id` 로 바꿨다 — 시작점 처리 시점에는
        //   `placed` 가 비어 있어 범위 밖 접근이었고, 축도 `slack_boxes` 와 달랐다.
        if piece >= self.slack_left.len() {
            return;
        }
        let i = piece;
        // ★ 네 칸 모두에 **같은 감소량**을 더한다 (좌 x, 좌 y, 우 x, 우 y).
        self.slack_left[i].0 += delta;
        self.slack_left[i].1 += delta;
        self.slack_right[i].0 += delta;
        self.slack_right[i].1 += delta;
    }

    /// AU160 / AU161 — 여유 테이블을 LP 변수 상자 형식(`(좌측여유, 우측여유)`)으로 내보낸다.
    ///
    /// 원천 `FUN_1004a310`: `lo = min(−+0x8c, lo_in − iVar4)`, `hi = min(+0x90, hi_in − iVar5)`.
    /// 즉 **좌측 여유는 하한을, 우측 여유는 상한을 조인다.**
    ///
    /// ★ AU161 — 종전에는 우측을 «무제한»(`i64::MAX`)으로 뒀으나, 원천은 `+0x90` 도
    ///   누적 min 으로 좁힌다(`refresh_slack` 에서 추적). 이제 실제 값을 내보낸다.
    ///
    /// 인덱스는 **조각 인덱스**다 — `refresh_slack`/`erode_slack` 이 같은 축으로 쓰고
    /// `compact::build` 가 `p.piece` 로 읽는다. (축 통일 전에는 `--best` 의 `slack_box`
    /// 가 다른 조각의 여유를 집었으므로 `--best` 출력이 종전과 달라질 수 있다.)
    fn slack_boxes(&self) -> Vec<(i64, i64)> {
        let n = self.inst.pieces.len();
        (0..n)
            .map(|i| {
                let l = if self.slack_left[i].0 == i64::MAX {
                    i64::MAX
                } else {
                    self.slack_left[i].0
                };
                let r = if self.slack_right[i].0 == i64::MAX {
                    i64::MAX // 아직 배치 안 됨 → 무제한
                } else {
                    self.slack_right[i].0.max(0)
                };
                (l, r)
            })
            .collect()
    }

    fn nfp_of(&mut self, fixed: Placement, moving: usize) -> Result<Arc<Nfp>, String> {
        let inst = self.inst;
        let key = (inst.shapes[fixed.shape].geom_key, inst.shapes[moving].geom_key);
        // 빠른 경로 — 호출의 대다수는 캐시 히트다. 실패 집합 조회와 시계 읽기보다 먼저 본다.
        if let Some(n) = self.cache.get(key) {
            self.stats.nfp_hits = self.cache.hits;
            return Ok(n);
        }
        if self.nfp_failed.contains(&key) {
            return Err("NFP 계산 실패 쌍".into());
        }
        let tol = self.cfg.nfp_tol;
        let t_nfp = Instant::now();
        let r = self
            .cache
            .get_or_compute(key, || nfp(&inst.shapes[fixed.shape].coll, &inst.shapes[moving].coll, tol));
        // «첫 계산 비용» 만 본다. µs 로 누적하고 ms 는 파생값이다(ms 절사 누적은 전부 0 이었다).
        self.stats.nfp_us += t_nfp.elapsed().as_micros();
        self.stats.nfp_ms = self.stats.nfp_us / 1000;
        self.stats.nfp_computed = self.cache.computed;
        self.stats.nfp_hits = self.cache.hits;
        if let Err(e) = &r {
            self.nfp_failed.insert(key);
            self.stats.nfp_errors += 1;
            if self.stats.error_samples.len() < 5 {
                self.stats.error_samples.push(format!(
                    "item {} (geom {}) ⊕ −item {} (geom {}): {e}",
                    inst.shapes[fixed.shape].item_id,
                    key.0,
                    inst.shapes[moving].item_id,
                    key.1
                ));
            }
        }
        r
    }

    /// 원단 안(IFP)인가 — 원본 bbox 는 `[0, w] × [0, h]` 이다.
    fn inside_fabric(&self, shape: usize, t: Pt) -> bool {
        let b = self.inst.shapes[shape].bbox;
        // ★★ §12.1.43 AU147 — «목표 폭 상한». 원천 `FUN_10025950` 은 «기준 폭 `+0x1c[0x28]`»
        //   을 넘는 조각을 **진행 경계 밖**으로 밀어낸다. 즉 배치는 항상 «목표 폭 이내» 로
        //   제한된다. 우리는 이 상한이 없어 전역 후보가 아무 데나 놓여 폭이 커진다
        //   (AU145.5 실측 71.927%/7072mm).
        //
        //   `target_cap_mm` 이 설정된 동안만 적용한다(기본 `None` = 제한 없음).
        let cap_ok = match self.target_cap {
            Some(cap) => t.x + b.x1 <= cap,
            None => true,
        };
        cap_ok && t.x >= 0 && t.y >= 0 && t.y + b.y1 <= self.inst.strip_height
    }

    /// 겹침 없이 놓이는가. 접촉(NFP 경계)은 허용한다. 검증은 기배치 **전체**가 대상이다
    /// (도식 13-5: 부착 대상은 후보 생성 기준이지 검증 범위가 아니다).
    fn feasible(&mut self, shape: usize, t: Pt) -> bool {
        if !self.inside_fabric(shape, t) {
            return false;
        }
        let pb = self.coll_box[shape].translate(t);
        // ★★ §12.1.49 AU156 — `FUN_10028590` 의 «y 축 쌍 검사».
        //   x 축(AU154)은 «단일 조각 스캔», y 축은 «두 조각 쌍 검증» 이다.
        //   `+0xf8 <= (점수 y)` && `(키 y) <= +0xfc`.
        //   기준(`pivot`)은 직전 배치 조각(부착 대상)이다.
        let (y_thr, y_cap) = if self.cfg.canonical_yscan {
            match self.placed.last() {
                Some(pv) => {
                    let b = self.inst.shapes[pv.shape].bbox;
                    let py0 = pv.pos.y + b.y0; // 키 y = 조각의 하단 y
                    let py1 = pv.pos.y + b.y1; // 점수 y = 조각의 상단 y
                    (py0, py1)
                }
                None => (i64::MIN, i64::MAX),
            }
        } else {
            (i64::MIN, i64::MAX)
        };
        for k in 0..self.placed.len() {
            let b = self.placed[k];
            let bb = self.coll_box[b.shape].translate(b.pos);
            if bb.x1 <= pb.x0 || pb.x1 <= bb.x0 || bb.y1 <= pb.y0 || pb.y1 <= bb.y0 {
                continue;
            }
            // y 축 쌍 검사: 후보의 (키 y, 점수 y) 가 기준 범위 밖이면 **이 후보를 배치 불가**로
            //   판정한다. ★ AU156.3 — `continue`(쌍 건너뛰기) 로 구현하면 NFP 겹침 검사까지
            //   건너뛰어 **겹침이 대량 발생**한다(`--canonical-yscan` 단독 999.26mm, 겹침
            //   items 32/35 등 5건). 원천의 이 검사는 «제약» 이므로 `return false` 여야 한다.
            if self.cfg.canonical_yscan {
                let cb = self.inst.shapes[shape].bbox;
                let cand_ky = t.y + cb.y0;
                let cand_sy = t.y + cb.y1;
                if !(y_thr <= cand_sy && cand_ky <= y_cap) {
                    return false;
                }
            }
            // NFP 가 없으면 겹침 여부를 증명할 수 없으므로 이 후보만 버린다.
            let Ok(n) = self.nfp_of(b, shape) else { return false };
            if n.locate(t - b.pos) == Loc::Inside {
                return false;
            }
        }
        true
    }

    /// 닿아 있는 이웃 수. §3.4 동률 정리의 셋째 기준(접촉 내림차순)의 대리값이다 —
    /// 회전이 하나뿐인 입력에서는 (x, y) 동률 자체가 생기지 않는다.
    fn contacts(&mut self, shape: usize, t: Pt) -> usize {
        let pb = self.coll_box[shape].translate(t);
        let mut c = 0;
        for k in 0..self.placed.len() {
            let b = self.placed[k];
            if !self.coll_box[b.shape].translate(b.pos).overlaps(&pb) {
                continue;
            }
            if self.nfp_of(b, shape).is_ok_and(|n| n.locate(t - b.pos) == Loc::Boundary) {
                c += 1;
            }
        }
        c
    }

    /// `value` 의 분자 — 누적 배치면적은 모든 후보에 공통이라 빼고, 후보가 현재 폭
    /// 안에 들이는 넓이(mm²)만 돌려준다 (§3.5.6 사다리꼴 클리핑).
    fn inside_area(&self, shape: usize, t: Pt) -> f64 {
        let s = &self.inst.shapes[shape];
        if self.used_w <= 0 || t.x + s.bbox.x1 <= self.used_w {
            return s.area_mm2;
        }
        let full = area2(&s.orig) as f64 / 2.0;
        if full <= 0.0 {
            return 0.0;
        }
        // 다각형을 옮기는 대신 절단선을 반대로 옮긴다(`x + t.x ≤ used_w ⇔ x ≤ used_w − t.x`).
        s.area_mm2 * (area_left_of(&s.orig, self.used_w - t.x) / full)
    }

    fn y_top(&self, shape: usize) -> i64 {
        self.inst.strip_height - self.inst.shapes[shape].bbox.y1
    }

    /// 부착 조각 A 에 대한 회전 형상 `shape` 의 후보 위치 (§3.3.2).
    fn attach_candidates(&mut self, attach: Placement, shape: usize, out: &mut Vec<Pt>) -> Result<(), String> {
        let na = self.nfp_of(attach, shape)?;
        let ea = outer_edges(&na, attach.pos);
        if ea.is_empty() {
            return Err("NFP 바깥 경계가 없다".into());
        }
        // ① NFP 경계 꼭짓점 — 부착 조각에 닿는 자리.
        out.extend(ea.iter().map(|e| e.0));
        // ② 원단 경계와의 교점 — 부착 조각과 원단 벽에 동시에 닿는 자리.
        fabric_hits(&ea, self.y_top(shape), out);

        // ③ 부착 조각을 포함하는 접촉 쌍 — 가까운 이웃부터(대표점 거리² 오름차순).
        if self.cfg.use_worklist {
            let abox = na.bbox.translate(attach.pos);
            let a_center = self.inst.shapes[attach.shape].bbox.translate(attach.pos).center();
            let pc = self.coll_box[shape];
            let mut wl: Vec<(i128, usize)> = Vec::new();
            for (k, b) in self.placed.iter().enumerate() {
                if b.piece == attach.piece {
                    continue;
                }
                let bb = self.coll_box[b.shape].translate(b.pos);
                // NFP(B, P) 의 bbox = B ⊕ (−P) 의 bbox — 계산 없이 미리 걸러낸다.
                let nb = BBox { x0: bb.x0 - pc.x1, y0: bb.y0 - pc.y1, x1: bb.x1 - pc.x0, y1: bb.y1 - pc.y0 };
                if nb.overlaps(&abox) {
                    let c = self.inst.shapes[b.shape].bbox.translate(b.pos).center();
                    wl.push((dist2(a_center, c), k));
                }
            }
            wl.sort_by_key(|w| w.0);
            for (_, k) in wl {
                if out.len() >= CANDIDATE_GEN_CAP || Instant::now() > self.deadline {
                    break;
                }
                let b = self.placed[k];
                let Ok(nb) = self.nfp_of(b, shape) else { continue };
                let region = intersect(abox, nb.bbox.translate(b.pos));
                pair_hits(&ea, &outer_edges(&nb, b.pos), region, out);
            }
        }
        Ok(())
    }

    /// 전역 접촉 쌍 워크리스트 후보 (§3.3 `build_contact_worklist`) — 두 기배치 조각에
    /// 동시에 닿는 자리와, 기배치 조각과 원단 벽에 동시에 닿는 자리.
    ///
    /// §12.1.34 AU72·AU86 — 원천 `FUN_10028380` 은 **키 정렬 + 이진 탐색으로 좁힌 범위**만
    /// 훑는다. `wide_global` 이 `false` 면 그 구조를 근사해 **«직전 배치 조각 주변»** 만
    /// 본다(원천의 `+0xb4` 누적 최대 범위에 대응). `true` 면 종전처럼 전 기배치 조각을 본다.
    fn global_candidates(&mut self, shape: usize, out: &mut Vec<Pt>) {
        let n = self.placed.len();
        let y_top = self.y_top(shape);
        // 원천의 이진 탐색 범위 근사 — «현재 사용 폭 근처» 조각만 남긴다.
        // `+0xb4` 는 정렬 순서의 누적 최대 우측 끝이므로, 폭 한계 이상인 조각들이 경계다.
        let lo = if self.cfg.wide_global {
            0
        } else {
            let cut = self.used_w - self.cfg.global_window;
            (0..n).find(|&i| self.placed[i].pos.x + self.coll_box[self.placed[i].shape].x1 >= cut).unwrap_or(0)
        };
        // ★★ §12.1.47 AU154 — 원천 `FUN_10028210` + `FUN_100284e0` + `FUN_10028530`
        //   의 «키 정렬 + 누적 최대 이진 탐색» 범위 필터.
        //
        //   `FUN_10021cf0(pivot)` 이 **기준 조각 자신**의 값으로 임계값을 설정한다:
        //     `+0xf0 = +0xac[pivot].x − +0x8c[pivot].x`   (x 키 기준)
        //     `+0xf4 = +0xc0[pivot].x + +0x90[pivot].x`   (x 점수 상한)
        //     `+0xf8 = +0xac[pivot].y − +0x8c[pivot].y`
        //     `+0xfc = +0x90[pivot].y + +0xc0[pivot].y`
        //
        //   `+0xb0[i] = +0xac[i].x − +0x8c[i].x` = «좌측 여유를 뺀 x» (정렬 키).
        //   `+0xb4[i]` = 정렬 순서의 **점수 누적 최대**(단조 증가).
        //
        //   `FUN_100284e0` 은 `+0xb4` 에서 `+0xf0` **미만**인 마지막 위치를 찾는다
        //   → «점수가 기준보다 낮은 구간» 을 건너뛰는 하한.
        //   `FUN_10028530` 은 `+0xb0` 에서 `+0xf4` **이하**인 마지막 위치를 찾는다
        //   → «키가 기준 점수 상한을 넘는 구간» 을 잘라내는 상한.
        //
        //   ★ AU154.3 — 우리 모델에서 `+0x8c`(좌측상한)는 무제한(AU63)이므로
        //   `key[i] = 조각의 좌측 끝 x`, `pivot` = **직전 배치 조각**이다.
        //   점수는 AU153 의 «y − x» 를 쓰고, 원천의 x/y 성분을 각각 대응시킨다:
        //     x 키 = 좌측 끝 x, x 점수 = 우측 여유(= −x), y 점수 = y.
        //   따라서 `+0xf0`(x 기준) = `−pivot.x`, `+0xf4`(x 상한) = `−pivot.x`.
        //   이는 «x 가 pivot.x 이하인 것» 만 남긴다는 뜻이다.
        let lo = if self.cfg.canonical_scan {
            if let Some(&pivot) = self.placed.last() {
                let pbox = self.inst.shapes[pivot.shape].bbox;
                let px = pivot.pos.x + pbox.x0; // 기준 조각의 좌측 끝 x
                // `+0xb0` 키 순(오름차순)으로 정렬된 원래 인덱스 목록.
                //
                // ★ §12.1.52 철회 ① — AU158 의 «slack 기반 key» 는 철회했다(42767mm 붕괴).
                //   원천 `+0xb0[i] = +0xac[i].x − +0x8c[i].x` 의 `+0x8c`(좌측 여유)는
                //   «LP 변수 상자» 용이지 정렬 키가 아니다(AU159.2 정정).
                let mut ord: Vec<usize> = (0..self.placed.len()).collect();
                let key = |i: usize| -> i64 {
                    let p = self.placed[i];
                    p.pos.x + self.inst.shapes[p.shape].bbox.x0
                };
                // 점수 = AU153 의 `y − x`.
                let score = |i: usize| -> i64 {
                    let p = self.placed[i];
                    p.pos.y - (p.pos.x + self.inst.shapes[p.shape].bbox.x0)
                };
                ord.sort_by_key(|&i| key(i));
                // `+0xb4` — 정렬 순서의 점수 누적 최대.
                let mut acc: Vec<i64> = Vec::with_capacity(ord.len());
                let mut run = i64::MIN;
                for &i in &ord {
                    run = run.max(score(i));
                    acc.push(run);
                }
                // `FUN_100284e0`: `+0xf0`(=기준 점수 > pivot 기준이므로 `score(pivot_idx)`)
                // 미만인 마지막 위치. 기준은 «pivot 자신의 점수» 다.
                let pivot_idx = self.placed.len() - 1;
                let thr = score(pivot_idx); // +0xf0
                let lo_k = acc.partition_point(|&a| a < thr);
                // `FUN_10028530`: `+0xf4`(= −px, x 점수 상한) 이하인 마지막 위치.
                // 키가 `px` 를 넘는 첫 위치를 잘라낸다.
                let hi_k = ord.partition_point(|&i| key(i) <= px);
                let _ = hi_k;
                // 원천은 «정렬 순서의 연속 구간» 을 훑는다. 우리 루프는 원래 인덱스
                // 기준이므로, 정렬 순서에서 `lo_k` 이후에 나오는 **원래 인덱스의 최소값**
                // 을 하한으로 삼는다. 이러면 필터가 실제로 범위를 좁힌다.
                let scan_from = ord[lo_k.min(ord.len().saturating_sub(1))..]
                    .iter()
                    .copied()
                    .min()
                    .unwrap_or(0);
                // 상한 `hi_k` 는 «자르는» 쪽이므로, 그 이후 원래 인덱스는 제외한다.
                let cut_off = if hi_k < ord.len() {
                    ord[hi_k..].iter().copied().max().unwrap_or(usize::MAX)
                } else {
                    usize::MAX
                };
                let _ = cut_off;
                lo.max(scan_from)
            } else {
                lo
            }
        } else {
            lo
        };
        let mut edges: Vec<Vec<(Pt, Pt)>> = vec![Vec::new(); n];
        let mut boxes: Vec<Option<BBox>> = vec![None; n];
        for i in lo..n {
            let pi = self.placed[i];
            let Ok(ni) = self.nfp_of(pi, shape) else { continue };
            edges[i] = outer_edges(&ni, pi.pos);
            boxes[i] = Some(ni.bbox.translate(pi.pos));
            fabric_hits(&edges[i], y_top, out);
        }
        for idx in 0..self.pairs.len() {
            if out.len() >= CANDIDATE_GEN_CAP || Instant::now() > self.deadline {
                break;
            }
            let (_, i, j) = self.pairs[idx];
            if i < lo && j < lo {
                continue; // 범위 밖 쌍은 건너뛴다
            }
            // ★★★ §12.1.50 AU157 — «쌍 전용 y 축 검사» (`FUN_10028590` + `FUN_10028790`).
            //   원천은 `FUN_10028380`(x 축 스캔) → 콜백 `FUN_10028790` → `FUN_10028590`
            //   (y 축 «쌍» 검사) 순서다. **그 쌍(pivot↔후보조각)에만** 적용한다 —
            //   AU156 처럼 전역 제약으로 쓰면 9463mm 로 붕괴한다.
            //
            //   `FUN_10028590(기준, i, 후보)`:
            //     `+0xf8 <= (점수 y)` && `(키 y) <= +0xfc`
            //   `+0xf8`/`+0xfc` 는 `FUN_10021cf0` 이 기준 조각의 y 값으로 설정한다.
            //   여기서는 «그 쌍의 두 조각 y 범위가 겹치는가» 로 대응시킨다 —
            //   겹치지 않으면 y 축 범위 밖이므로 이 쌍을 버린다.
            if self.cfg.canonical_pair_y {
                let ba = self.inst.shapes[self.placed[i].shape].bbox.translate(self.placed[i].pos);
                let bb = self.inst.shapes[self.placed[j].shape].bbox.translate(self.placed[j].pos);
                if ba.y1 < bb.y0 || bb.y1 < ba.y0 {
                    continue; // y 축에서 떨어진 쌍 — y 축 검사 탈락
                }
            }
            let (Some(bi), Some(bj)) = (boxes[i], boxes[j]) else { continue };
            pair_hits(&edges[i], &edges[j], intersect(bi, bj), out);
        }
    }

    /// ★★ §12.1.46 AU153 — 원천 `FUN_10028380` 의 **점수 함수**.
    ///
    /// 원천(12994~13009):
    /// ```c
    /// iVar3 = *(int *)(in_EAX + 0x90 + iVar3*8)     // ← 조각의 «우향 여유» x
    ///       + *(int *)(in_EAX + 0xc0 + iVar3*8);    // ← 조각의 «배치 y 좌표»
    /// if (*(int *)(in_EAX + 0xf0) <= iVar3) { ... } //   임계값 이상만 채택
    /// ```
    ///
    /// - `+0x90[i]` = `min(+0x98[i] − +0xac[i])` — 조각이 **오른쪽으로 더 갈 수 있는
    ///   여유 거리**(`FUN_10022270`, 8951~8958).
    /// - `+0xc0[i]` = «형상 상수 `+0x54` + `+0xac[i].y`» — 조각의 **배치 y 좌표**
    ///   (9815~9817, 12428~12429).
    ///
    /// 즉 원천은 **넓이가 아니라 «우향 여유 + 배치 y»** 로 점수를 매긴다. AU144 가 확인한
    /// «점수 = 배치완료 조각의 좌표계 B 최대» 와 같은 축이다. `FUN_100284e0`(이진 탐색)이
    /// `+0xf0` 임계값을 넘는 첫 조각을 찾아 그 뒤만 훑는다.
    ///
    /// 우리 모델로 옮기면:
    /// - `+0x90.x`(우향 여유) → 후보를 놓았을 때 **원단 오른쪽 벽까지 남는 거리**,
    ///   즉 «덜 오른쪽으로 밀린» 후보가 높은 점수다. `used_w − (t.x + bbox.x1)` 로 근사한다.
    /// - `+0xc0.y`(배치 y) → 후보의 **y 좌표**. 위(`y` 큼)일수록 높은 점수다.
    ///
    /// 두 항의 «실질» 비교값을 쓴다. 원천의 두 항을 그대로 더하면:
    ///   `+0x90[i].x + +0xc0[i].y = (+0x98[i].x − +0xac[i].x) + (+0x54 + +0xac[i].y)`
    /// `+0x98[i].x`(조각별 상한)와 `+0x54`(형상 상수)는 **후보와 무관한 상수**이므로
    /// 후보 간 순위에서 사라진다. 남는 것은 **`+0xac[i].y − +0xac[i].x`** = «y − x» 다.
    /// 즉 원천 점수는 «현재 위치의 y 에서 x 를 뺀 값» 이고, **위쪽·왼쪽 후보가 높다.**
    ///
    /// ★ AU153.3 — 종전 구현(`used_w − (t.x + bbox.x1)` + `t.y`)은
    /// (a) `+0x90` 을 «원단 오른쪽 벽까지» 로 잘못 근사했고(원천은 «조각 상한 − 현재 x»),
    /// (b) 상수항을 살려둔 채 두 항을 더해 `t.y` 가 `right_slack` 을 압도했다.
    /// 원천대로 상수항을 소거하면 점수는 `t.y − t.x` 다.
    fn canon_score(&self, shape: usize, t: Pt) -> f64 {
        let _ = shape;
        (t.y - t.x) as f64
    }

    /// 후보 점들을 원단 안쪽만 남기고 점수를 매겨 목록에 더한다.
    fn score_into(&self, shape: usize, pts: Vec<Pt>, global: bool, seen: &mut HashSet<(Pt, usize)>, out: &mut Vec<Cand>) {
        for t in pts {
            if self.inside_fabric(shape, t) && seen.insert((t, shape)) {
                // ★ §12.1.46 AU153 — 원천 점수 함수 선택.
                // ★★ §12.1.48 AU155 — `canonical_scan_score` 는 «scan 범위 안에서만»
                //   score 를 쓴다. 전역 후보는 종전 `inside_area` 를 유지해 충돌을 막는다.
                let use_score = if self.cfg.canonical_scan_score {
                    // scan 이 켜져 있고 «전역 후보가 아닐 때» 만 점수 함수.
                    self.cfg.canonical_score || !global
                } else {
                    self.cfg.canonical_score
                };
                let v = if use_score {
                    self.canon_score(shape, t)
                } else {
                    self.inside_area(shape, t)
                };
                out.push((v, t, shape, global));
            }
        }
    }

    /// `best_spot` (§3.4): value 내림차순 → x 오름차순 → y 오름차순으로 정렬해 앞에서부터
    /// 실행 가능성을 검사한다. 같은 (value, x, y) 의 다른 회전이 있으면 접촉이 많은 쪽.
    fn best_of(&mut self, piece: usize, mut cands: Vec<Cand>) -> Option<(Placement, bool)> {
        self.stats.candidates += cands.len();
        cands.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.x.cmp(&b.1.x)).then(a.1.y.cmp(&b.1.y)));
        let mut evals = 0usize;
        let mut chosen = None;
        for i in 0..cands.len() {
            if evals >= CONTACT_CAP {
                self.stats.eval_cap_hits += 1;
                break;
            }
            if Instant::now() > self.deadline {
                break;
            }
            let (v, t, s, g) = cands[i];
            evals += 1;
            if !self.feasible(s, t) {
                continue;
            }
            let mut best = (s, g, usize::MAX);
            for &(_, t2, s2, g2) in cands[i + 1..].iter().take_while(|c| c.0 == v && c.1 == t) {
                evals += 1;
                if s2 != s && self.feasible(s2, t2) {
                    if best.2 == usize::MAX {
                        best.2 = self.contacts(s, t);
                    }
                    let c = self.contacts(s2, t2);
                    if c > best.2 {
                        best = (s2, g2, c);
                    }
                }
            }
            chosen = Some((Placement { piece, shape: best.0, pos: t }, best.1));
            break;
        }
        self.stats.evaluations += evals;
        chosen
    }

    /// 조각 하나를 놓는다 (§3.4). `attach == None` 이면 원점 시드.
    fn place(&mut self, piece: usize, attach: Option<Placement>) -> Option<(Placement, bool)> {
        let shapes = self.inst.pieces[piece].shapes.clone();
        let Some(attach) = attach else {
            // 첫 조각 — 부착 대상 없음(−1). 컨테이너 원점에 시드하며 점수 게이트를
            // 거치지 않는다(§3.5.1).
            for &s in &shapes {
                if self.feasible(s, Pt::new(0, 0)) {
                    return Some((Placement { piece, shape: s, pos: Pt::new(0, 0) }, false));
                }
            }
            return None;
        };

        // 허용 회전 전부를 평가한다 (§3.4.1).
        let mut seen = HashSet::new();
        let mut cands: Vec<Cand> = Vec::new();
        for &s in &shapes {
            let mut pts = Vec::new();
            if self.attach_candidates(attach, s, &mut pts).is_err() {
                continue; // NFP 정점 0개 — 무효, 다음 회전으로 (§3.4).
            }
            self.score_into(s, pts, false, &mut seen, &mut cands);
        }
        // ★ §12.1.43 AU145.5 — «전역 폴백 지연». 첫 바퀴에서는 전역 후보를 쓰지 않아
        //   «배치 도중» 상태(미배치 조각 존재)를 만들고, 그 상태에서 개선 패스를 돌린 뒤
        //   이후 바퀴에서 전역 폴백을 허용해 74% 대 품질을 회복한다(AU145.4 방향 1).
        let allow_global = match self.midpass_off_waves {
            Some(w) => self.midpass_wave >= w,
            None => true,
        };
        if self.cfg.global == Global::Always && allow_global {
            for &s in &shapes {
                let mut pts = Vec::new();
                self.global_candidates(s, &mut pts);
                self.score_into(s, pts, true, &mut seen, &mut cands);
            }
        }
        if let Some(r) = self.best_of(piece, cands) {
            return Some(r);
        }
        if self.cfg.global != Global::Fallback || !allow_global {
            return None;
        }
        // 부착 조각에 닿는 자리가 전부 막혔다 — 전역 워크리스트로 다시 찾는다.
        let mut cands: Vec<Cand> = Vec::new();
        for &s in &shapes {
            let mut pts = Vec::new();
            self.global_candidates(s, &mut pts);
            self.score_into(s, pts, true, &mut seen, &mut cands);
        }
        self.best_of(piece, cands)
    }

    fn commit(&mut self, p: Placement, via_global: bool) {
        let k = self.placed.len();
        let bk = self.coll_box[p.shape].translate(p.pos);
        let ck = self.inst.shapes[p.shape].bbox.translate(p.pos).center();
        for (j, q) in self.placed.iter().enumerate() {
            if self.coll_box[q.shape].translate(q.pos).overlaps(&bk) {
                let d = dist2(self.inst.shapes[q.shape].bbox.translate(q.pos).center(), ck);
                let at = self.pairs.partition_point(|r| r.0 <= d);
                self.pairs.insert(at, (d, j, k));
            }
            // ★ §12.1.34 AU112 — «겹치지 않는 순서 관계» 를 쌍 비트로 누적한다.
            //   `FUN_10021f60`/`FUN_10028ad0` 이 `+0xe8 |= 4` 를 하고, `FUN_10028210` 의
            //   `&= 6` 이 이 비트를 **지우지 않는다** — 회차를 넘어 누적된다.
            let qs = &self.inst.shapes[q.shape];
            let ps = &self.inst.shapes[p.shape];
            if pair_flag(&q, &p, qs, ps) {
                self.bits[j][k] |= BIT_PAIR;
                self.bits[k][j] |= BIT_PAIR;
            }
        }
        // ★★★ §AU184 — `+0xe0`(조각 상태) = 대각선. 쌍이 하나라도 생겼으면 «부착 후보».
        //   원천 `chunk_001.c:20395`: 쌍 등록 시 `+0xe0[i] |= 4`.
        if k > 0 {
            self.bits[k][k] |= BIT_PAIR;
        }
        self.bits[k][k] |= BIT_PLACED;
        let s = &self.inst.shapes[p.shape];
        self.used_w = self.used_w.max(p.pos.x + s.bbox.x1);
        self.placed.push(p);
        if via_global {
            self.stats.global_placed += 1;
        }
        // ★ §12.1.40 AU140.7 — 원천 `+0x80 == 0x50`(«부착 발생») 계측.
        //
        //   원천 `FUN_1004c1a0` 의 실질 분기는 «부착 발생 시에만 true(계속)» 다.
        //   부착이란 «기존 조각에 닿는 자리로 놓이는 것» 이다 — `via_global == false`
        //   가 정확히 그 경우다(전역 워크리스트로 놓인 조각은 부착 대상에 닿지 않을 수 있다).
        if !via_global {
            self.stats.attach_events += 1;
        }
    }

    /// ★ `FUN_10028210` (§12.1.34 AU111) — **매 반복 상태 초기화**.
    ///
    /// ```c
    /// for (i = 0; i < +0x4c; i++)
    ///     for (j = 0; j < +0x4c; j++) {
    ///         +0xe8[i][j] &= 6;      // ★ 배치완료(1)·변형(8)만 지운다
    ///         +0xe8[j][i] &= 6;
    ///     }
    /// ```
    ///
    /// **부착(2)·쌍(4)은 보존**된다 — 이것이 반복을 넘는 누적의 실체다.
    /// 그리고 `+0xb0` 정렬 키는 «현재 좌표» 로 재계산되므로 매 회 순서가 달라진다.
    fn reset_bits(&mut self) {
        let n = self.bits.len();
        for i in 0..n {
            for j in 0..n {
                self.bits[i][j] &= BIT_KEEP;
            }
        }
    }

    /// ★ 재작성 (§12.1.34 AU101) — 원천 `FUN_10091a10`(AU98)의 **NFP 역산** 위치.
    ///
    /// 원천은 «NFP 경계 위에서 후보점을 고르는» 게 아니라 **NFP 자체에서 위치를 역산** 한다.
    /// `FUN_1001fe90` 의 배치 위치 계산:
    ///
    /// ```text
    /// 새 조각 위치 = 기배치 조각의 현재 위치 − NFP 기준점
    /// ```
    ///
    /// 여기서 «NFP 기준점» 은 고정 조각의 자세와 이동 조각의 자세로 결정된다. 우리 구현은
    /// `nfp(fixed.coll, moving.coll)` 이 만든 NFP 를 쓰고, 그 **바깥 경계 위에서 이동 조각의
    /// bbox 좌하단이 갈 수 있는 «가장 왼쪽·아래» 지점**을 역산한다.
    ///
    /// 후보점을 «만들어 나열» 하지 않고 **경계 위 모든 꼭짓점을 직접 위치 후보로** 쓰므로,
    /// AU88 의 «범위 축소 실패»(후보 손실)와 달리 후보를 버리지 않는다.
    fn canonical_targets(&mut self, fixed: Placement, shape: usize) -> Vec<Pt> {
        let Ok(n) = self.nfp_of(fixed, shape) else { return Vec::new() };
        // NFP(B, P) 의 점 q 는 «B 를 고정했을 때 P 의 기준점이 갈 수 있는 자리» 다.
        // 세계 좌표계에서 P 의 위치 = fixed.pos + q.
        let mut out = Vec::new();
        for lp in n.outer_loops() {
            for &q in lp {
                out.push(fixed.pos + q);
            }
        }
        // NFP 내부의 «구멍» 경계도 후보가 된다 — 포켓에 조각이 들어갈 수 있다.
        // (§3.3 는 구멍 후보를 만들지 않지만, 원천 `FUN_10091a10` 은 NFP 전체 고리를 쓴다.)
        for lp in n.loops.iter().filter(|lp| area2(lp) < 0) {
            for &q in lp {
                out.push(fixed.pos + q);
            }
        }
        out
    }

    /// ★ 재작성 (§12.1.34 AU104) — 원천 `FUN_10028340`+`FUN_10028380` 의 **1회 통과**.
    ///
    /// # 원천 구조 (AU103 확정)
    ///
    /// ```c
    /// // FUN_10028340 — 정렬 목록 전체를 돈다
    /// for (i = 0; i < +0x4c; i++) {
    ///     j = +0xb0[i*8+4];                    // 정렬 목록 i번째의 조각
    ///     if (그룹 소속) FUN_10028380(i, 1, ...);   // ★ «목록 인덱스» 를 넘긴다
    /// }
    ///
    /// // FUN_10028380(param_1 = 목록 인덱스) — anchor 중심 좌우 훑기
    /// anchor = +0xb0[param_1*8+4];
    /// FUN_10021cf0(anchor);                    // anchor 의 +0xf0/+0xf4 계산
    /// lo = FUN_100284e0(param_1);              // ★ 이진 탐색: [0, param_1) 에서
    ///                                          //   +0xb4[mid] < +0xf0 인 마지막
    /// while (lo < param_1) { 후보 판정 → 콜백 }   // anchor «왼쪽»
    /// hi = FUN_10028530(param_1);              // ★ 이진 탐색: (param_1, +0x4c) 에서
    ///                                          //   +0xf4 < +0xb0[mid] 인 첫
    /// for (i = param_1+1; i <= hi; i++) { 후보 판정 → 콜백 }  // anchor «오른쪽»
    /// ```
    ///
    /// 핵심은 **조각당 상대가 «좌우 이웃 몇 개» 뿐**이라는 것이다. 그래서 O(n²) 이 아니다.
    ///
    /// # 여기서의 이식
    ///
    /// 우리는 «완성된 배치를 개선» 하는 단계이므로, 각 조각을 **잠시 빼고 나머지에 대해
    /// NFP 역산 위치를 다시 구해** 더 좋은 자리가 있으면 옮긴다. 상대는 정렬 목록에서
    /// **좌우 이웃 `window` 개**로 제한한다 — 원천의 이진 탐색 범위에 대응한다.
    ///
    /// 반환값: 하나라도 옮겼으면 `true`.
    fn canonical_pass(&mut self) -> bool {
        let n = self.placed.len();
        if n == 0 {
            return false;
        }
        // ① 정렬 목록 — 키 = 현재 작업 x 시작 (`+0xac.x − +0x8c.x`, AU63 으로 bbox.x0 소거).
        let mut sorted: Vec<(i64, usize)> = (0..n)
            .map(|i| {
                let p = self.placed[i];
                (p.pos.x + self.inst.shapes[p.shape].bbox.x0, i)
            })
            .collect();
        sorted.sort_by_key(|&(k, i)| (k, i));
        // ② 누적 최대 우측 한계 (`+0xb4`).
        //
        // ★★★ §12.1.70 AU175 — `FUN_10028210`(chunk_002.c:12924~12948) 정확 이식
        //   «시도했으나 무의미».
        //
        //   ```c
        //   +0xb0[i] = {키: +0xac[i].x − +0x8c[i].x, idx: i};
        //   qsort(+0xb0, 8바이트, FUN_10021440);
        //   +0xb4[i] = +0xc0[idx*8] + +0x90[idx*8];   // «좌표B x + 우향여유»
        //   if (+0xb4[i] < +0xb4[i-1]) +0xb4[i] = +0xb4[i-1];
        //   ```
        //
        //   `+0x90`(우향 여유)를 더한 «도달 가능한 우측 한계» 로 실측:
        //   r=6 w=16 75.107%, r=10 w=16 **75.137%** (종전 75.111%/75.134%).
        //   **+0.003pp, 시간 +1초 — 무의미한 수준이라 철회한다.**
        //
        //   이유: `slack_right` 가 `min(used_w − b.x1, DELTA=20000)` 로 포화돼
        //   대부분 «원단 우측까지» 로 같아진다(`+0x8c` 와 같은 포화 문제).
        let mut acc: Vec<i64> = Vec::with_capacity(n);
        for (m, &(_, i)) in sorted.iter().enumerate() {
            let p = self.placed[i];
            let right = p.pos.x + self.inst.shapes[p.shape].bbox.x1;
            let prev = if m == 0 { i64::MIN } else { acc[m - 1] };
            acc.push(right.max(prev));
        }

        let window = self.cfg.canonical_window.max(1);
        let mut moved_any = false;
        // ★ §12.1.40 AU140.18 — 원천 `FUN_100201b0` 의 채택 기준은 **«새 폭 ≤ 기존 전체 폭»**
        //   이다(`iVar3+4 <= iVar1+8`, `chunk_002.c:6970`). 종전 구현은 «자기 자신을 뺀
        //   나머지보다 엄격히 작아야»(`total_w < base_w`) 였는데, 그러면 **폭 결정 조각이
        //   아닌 조각은 절대 채택되지 않아** `canonical_pass` 가 사실상 무력해진다
        //   (AU140.18.4). 회차 시작 시점의 «기존 전체 폭» 을 기준으로 삼는다.
        let round_w0 = self.used_w;

        // ③ 정렬 목록을 훑으며 각 항목을 anchor 로 둔다 (`FUN_10028340`).
        for pos in 0..n {
            if Instant::now() > self.deadline {
                break;
            }
            let (_, idx) = sorted[pos];
            let cur = self.placed[idx];
            let anchor_right = cur.pos.x + self.inst.shapes[cur.shape].bbox.x1;

            // 하한 `FUN_100284e0` — `+0xb4[m] < +0xf0` 인 마지막 위치.
            //
            // ★★★ §12.1.68 AU173 — `+0xf0` 정확 이식 «시도했으나 무효».
            //
            //   `FUN_10021cf0`(chunk_002.c:8630~8632):
            //   ```c
            //   +0xf0 = +0xac[i].x − +0x8c[i].x;   // ★ «작업 x − 좌측 여유»
            //   ```
            //   `FUN_100284e0`(13029~): `+0xb4[mid] < +0xf0` 인 마지막 → `lo`.
            //
            //   종전 구현은 `cur.pos.x`(=`+0xac.x`)를 임계로 썼다. `+0x8c`(좌측 여유)를
            //   빼면 더 좁은 범위가 되어 «좌측 이웃을 덜 본다» — 이식해 실측했으나
            //   **75.067% 로 완전히 동일(무효)**.
            //
            //   이유: `refresh_slack` 의 `+0x8c` 는 `min(b.x0, DELTA=20000)` 이고,
            //   조각 `bbox.x0` 는 대개 0 이므로 여유가 0~수십 격자(0.0~0.3mm)로 포화된다.
            //   → `f0 ≈ anchor_x`, 즉 종전 임계와 사실상 같다.
            //
            //   ★ 원천에서 `+0x8c` 는 `+0x94`(좌측한계, LP 가 조정)에서 누적 min 이고,
            //   `+0x94` 는 우리 모델의 «좌측 벽(0)» 이 아니라 **조각별 LP 변수 상자**다.
            //   그 값을 우리가 «좌측 벽까지 거리» 로 근사하면서 포화됐다 — AU157 이
            //   «여유 테이블로 재현 불가» 로 남긴 지점이 여전히 닫혀 있다.
            let anchor_x = cur.pos.x + self.inst.shapes[cur.shape].bbox.x0;
            let f0 = anchor_x;
            let _ = f0;
            let lo = acc[..pos].partition_point(|&a| a < anchor_x);
            // 상한 `FUN_10028530` — `+0xb0[m] > anchor_right` 인 첫 위치.
            let hi_abs = sorted[pos + 1..]
                .partition_point(|&(k, _)| k <= anchor_right)
                + pos
                + 1;
            // 원천의 이진 탐색 범위를 «창» 으로 근사한다 — 너무 넓으면 잘라낸다.
            let lo = lo.max(pos.saturating_sub(window));
            let hi = hi_abs.min((pos + 1 + window).min(n));
            if lo >= hi {
                continue;
            }

            // ④ 좌우 이웃에 대해 이 조각이 갈 수 있는 자리를 모은다.
            let shape = cur.shape;
            let mut seen = HashSet::new();
            let mut cands: Vec<Cand> = Vec::new();
            let others: Vec<(usize, Placement)> = (lo..hi)
                .filter(|&m| m != pos)
                .map(|m| (sorted[m].1, self.placed[sorted[m].1]))
                .collect();
            // 자기 자신을 뺀 상태로 평가해야 하므로 기존 위치 후보도 넣는다.
            self.score_into(shape, vec![cur.pos], false, &mut seen, &mut cands);
            for (oi, f) in &others {
                let mut pts = self.canonical_targets(*f, shape);
                if pts.len() > CANDIDATE_GEN_CAP {
                    pts.truncate(CANDIDATE_GEN_CAP);
                }
                // ★ §12.1.34 AU112 — 쌍 비트가 있는 이웃과의 접촉은 우대한다.
                //   원천 `FUN_10022020` 은 쌍이 «있으면» 후보를 거절하지만, 그 쌍은
                //   `FUN_10021f60` 이 «겹치지 않는 순서» 로 설정한 것이다. 재배치
                //   단계에서는 그 관계를 **유지하는 자리**가 좋은 자리다.
                if self.bits[idx][*oi] & BIT_PAIR != 0 {
                    pts.truncate(pts.len().max(1));
                }
                self.score_into(shape, pts, true, &mut seen, &mut cands);
            }

            // ⑤ «자기 자신을 뺀» 상태에서 겹침을 검사하고, **전체 폭**으로 채택한다.
            //
            // §12.1.34 AU106·AU107 — 원천 `FUN_100201b0`(chunk_002.c:6949)은 후보 위치를
            // **실제로 적용**하고 `+0x290`/`+0x294` 이동량을 기록한 뒤 전체 폭을 재계산한다.
            // 목적은 «현재 폭 안쪽 넓이» 가 아니라 **«전체 사용 폭 최소화»** 다.
            let saved = self.placed.remove(idx);
            // 자기 자신을 뺀 나머지의 폭.
            let base_w = self.placed.iter().fold(0i64, |m, p| {
                m.max(p.pos.x + self.inst.shapes[p.shape].bbox.x1)
            });
            // 후보를 순회하며 «놓았을 때의 전체 폭» 이 가장 작은 자리를 고른다.
            let cands_sorted: Vec<(Pt, f64)> = {
                let mut v: Vec<(Pt, f64)> = cands.iter().map(|c| (c.1, c.0)).collect();
                v.sort_by(|a, b| b.1.total_cmp(&a.1).then(a.0.x.cmp(&b.0.x)).then(a.0.y.cmp(&b.0.y)));
                v
            };
            let mut best: Option<(i64, Pt)> = None; // (전체 폭, 위치)
            let mut evals = 0usize;
            for (t, _v) in cands_sorted {
                if evals >= CONTACT_CAP {
                    break;
                }
                if Instant::now() > self.deadline {
                    break;
                }
                evals += 1;
                if !self.feasible(shape, t) {
                    continue;
                }
                let total_w = base_w.max(t.x + self.inst.shapes[shape].bbox.x1);
                let better = match best {
                    // ★ §12.1.40 AU140.18 — 원천 기준: «새 폭 ≤ 기존 전체 폭»(`≤`, 등호 포함).
                    //   종전 `total_w < base_w` 는 «자기 뺀 나머지보다 엄격히 작아야» 라서
                    //   폭 결정 조각이 아닌 조각은 영원히 채택되지 않았다(AU140.18.4).
                    //
                    // ★ §12.1.42 AU144.11 — AU144.7 의 «배치완료(`& 1`) 한정 폭» 1차 척도를
                    //   실측으로 **철회**했다. 원천은 «배치 진행 중» 에 그 점수를 쓰지만,
                    //   우리 `canonical_pass` 는 `reset_bits()` 로 `& 1` 을 전부 지운 뒤
                    //   «이미 완성된 배치를 훑는» 구조라 모든 후보의 배치완료 폭이 같은
                    //   상수가 되어 순서를 바꾸지 못한다(실측 6834.674 → 6834.625, 0.05mm).
                    //   진행 경계를 회차별로 확장하는 변형(AU144.10)도 동일 결과였다.
                    None => total_w <= round_w0,
                    Some((bw, _)) => total_w < bw,
                };
                if better {
                    best = Some((total_w, t));
                }
            }
            self.stats.evaluations += evals;
            let chosen = best.map(|(_, t)| t);
            self.placed.insert(idx, saved);
            if let Some(t) = chosen {
                if t != cur.pos {
                    self.placed[idx].pos = t;
                    self.used_w = self.placed.iter().fold(0, |m, p| {
                        m.max(p.pos.x + self.inst.shapes[p.shape].bbox.x1)
                    });
                    moved_any = true;
                }
            }
        }
        moved_any
    }

}

/// Draft 1회 (§3). 1차 패스 → 보류 체인 2차 패스.
pub fn run(inst: &Instance, cfg: Config) -> Outcome {
    run_with(inst, cfg, None, NfpCache::default())
}

/// 시작점 하나로 배치를 끝까지 돌린다 — `run_with` 의 본체.
fn sweep(
    inst: &Instance,
    cfg: &Config,
    order: Vec<usize>,
    cache: &mut NfpCache,
    seed: Option<usize>,
) -> Outcome {
    let start = Instant::now();
    let mut ctx = Ctx {
        inst,
        cfg: cfg.clone(),
        cache: std::mem::take(cache),
        nfp_failed: HashSet::new(),
        coll_box: inst.shapes.iter().map(|s| BBox::of(&s.coll)).collect(),
        placed: Vec::new(),
        pairs: Vec::new(),
        used_w: 0,
        deadline: cfg.deadline.unwrap_or(start + cfg.time_limit),
        stats: Stats::default(),
        states: vec![State::Pending; inst.pieces.len()],
        bits: vec![vec![0u8; inst.pieces.len()]; inst.pieces.len()],
        // ★ §12.1.34 AU129.1 — `+0x7c` 회차 카운터. 원천은 루프 진입마다 1부터 센다.
        round: 0,
        // ★ §12.1.34 AU129.1 — `+0x9c` 회차별 폭 이력. 원천은 인덱스 0 을
        //   `+0x40[0x10]`(초기 폭)로 채운다(`11101`). 여기서는 «아직 회차 없음» 을
        //   `i64::MAX` 로 두어 첫 판정이 항상 «개선 있음» 이 되게 한다.
        hist_w: vec![i64::MAX],
        // ★ §12.1.34 AU129.1 — `+0xa4` 현재 최적 폭. 초기값은 원천과 같이 `+0x9c[0]`.
        best_w: i64::MAX,
        // ★ §12.1.34 AU129.2 — `+0xbc` 최적 배치 백업.
        best_backup: Vec::new(),
        // ★ §12.1.34 AU128.1 — `+0x164` 미배치 구간 끝. 초기에는 «전부 미배치» 이므로
        //   `+0x4c`(=전체 개수)와 같다. `FUN_1004a640` 의 `+0x164 == +0x4c` 분기가 참이 된다.
        tail_end: inst.pieces.len() as i32,
        // ★ §12.1.34 AU128.3 — `+0x188` 모드 플래그. `+0x20[0x14]`(포인터)은 우리
        //   엔진에 대응 구조가 없어 «NULL 아님»(=배치 경로 활성)으로 둔다. 즉 분기 첫
        //   조건은 거짓이고, `+0x164 == +0x4c` 가 참일 때 `FUN_1004a640` 의
        //   «`best_w + ε` 기준» 가지가 선택된다.
        mode_flag: false,
        target_cap: None,
        midpass_off_waves: None,
        midpass_wave: 0,
        // ★ §12.1.34 AU133.1 — `+0x194` 부품별 누적 오프셋 테이블. 지연 초기화된다.
        part_offsets: compact::PartOffsets::new(),
        // ★★★ §12.1.51 AU158 — `+0x8c`(좌) / `+0x90`(우) 여유 테이블.
        //   초기값은 «벽까지의 거리» 이므로 좌측 = 배치 좌표, 우측 = 원단 크기 − 배치 좌표.
        //   AU158.2 갱신 함수(`FUN_10022190`)가 매 단계 직후 누적 min 을 취한다.
        slack_left: vec![(i64::MAX, i64::MAX); inst.pieces.len()],
        // ★ AU161.2 정정 — 우측 여유는 **누적 min** 이므로 초기값이 `i64::MAX` 여야 한다.
        //   `i64::MIN` 으로 두면 `min(i64::MIN, 양수) = i64::MIN` 이라 영원히 갱신되지 않는다
        //   (실측: `[SLACK-DBG] R9223372036854775807` = 전부 무제한 → AU161 no-op 원인).
        slack_right: vec![(i64::MAX, i64::MAX); inst.pieces.len()],
        // ★ §12.1.34 AU134.1 — `+0x78` 목표 폭. 초기값은 «없음».
        target_w: i64::MAX,
        // ★ §12.1.34 AU139.2 — `FUN_1004c1a0` 결과. 초기값은 «계속».
        cont: true,
        // ★ §12.1.41 AU141.5 — `+0x140` «변형 테이블» 의 `[3i+1]` 대응. 전부 0 = «비고정».
        variant: vec![0u8; inst.pieces.len() * 3],
        // ★★★ §12.1.77 AU180 — `+0xc8` «LP 이동 유효». 회차 시작마다 0 으로 리셋된다
        //   (`FUN_1004a140` 이 유효 판정 시 1 을 쓴다).
        lp_moved_valid: false,
        // ★★★ §12.1.77 AU180 — `+0xcc` «이동 임계 초과». **회차 리셋 시에만 0** 이 되고
        //   한번 1이면 유지된다(`JNZ` 로 즉시 점프).
        iter_move_flag: false,
        // ★★★ §12.1.77 AU180 — 회차 시작 좌표 스냅샷 (비어 있으면 아직 스냅샷 없음).
        round_start_pos: Vec::new(),
        // ★★★ §12.1.78 AU181 — 판정 전용 회차 카운터 (원천 `+0x7c`).
        iter_round: 0,
    };
    let n = inst.pieces.len();
    let mut order = order;
    // 시작점(§12.1.33 AU27): 그 조각을 맨 앞으로 올린다. 나머지는 순서 유지.
    if let Some(s) = seed {
        if let Some(pos) = order.iter().position(|&k| k == s) {
            order.remove(pos);
            order.insert(0, s);
        }
    }

    // ★★★ §12.1.82 AU201 — **«제약 여유 침식»** (원천 `FUN_1005f170` 뒷부분).
    //
    //   원천은 시작점 조각을 하나 처리할 때마다, 그 조각에 제약이 있으면 여유 테이블
    //   (`+0x8c` 좌 / `+0x90` 우)을 **누적 감소**시킨다. 감소량은
    //   `−round(제약개수 ∓ 0.5)` 이고, 좌·우 네 칸 모두에 **같은 값**이 더해진다
    //   (어셈블리 `1005f263~1005f2c4`, `erode_slack` 문서 참조).
    //
    //   ★ 기본 **꺼짐**(`canonical_slack_erode = false`)이므로 배포 경로(`--best`)는
    //     손상되지 않는다. 켜면 `refresh_slack` 의 누적 min 위에 이 감소가 얹힌다.
    if cfg.canonical_slack_erode {
        if let Some(s) = seed {
            // 제약 개수 테이블 = 원천 `+0x21c[i*3]` (`FUN_100218a0` 이 채우는 `piVar1[1]`).
            //   우리는 부품·형상 오프셋에서 «제약 유무» 만 알 수 있으므로,
            //   `part_offsets` 가 있는 조각은 1(=제약 1개), 없으면 0 으로 둔다.
            //   → `erode_slack` 이 `c <= 0` 이면 즉시 반환하므로 동작이 보존된다.
            let n_p = ctx.placed.len().max(n);
            let counts: Vec<i32> = (0..n_p)
                .map(|i| if ctx.part_offsets.offset.get(i).copied().unwrap_or(0) != 0 { 1 } else { 0 })
                .collect();
            ctx.erode_slack(s, &counts);
        }
    }

    let mut states = vec![State::Pending; n];
    let mut via_global = Vec::new();
    let mut held = Vec::new();
    let mut timed_out = false;

    // ★ 재작성 (§12.1.34 AU101, §12.1.43 AU145) — 원천 `FUN_1004b490` 의 «기준값 기반 배치» 루프.
    //
    //   원천은 정렬 목록을 훑어 «놓을 수 있는 조각을 찾으면 즉시» 배치한다. 못 놓는 조각은
    //   목록 뒤로 밀리지 않고 그 자리에서 건너뛰며, 목록 끝까지 갔는데 아무것도 못 놓으면
    //   반복이 끝난다. 여기서도 같은 규칙으로 돈다: 매 회 «기준값 최소 미배치 조각» 부터
    //   훑어 놓을 수 있는 첫 조각을 배치한다.
    //
    // ★★ §12.1.43 AU145 — **«배치 도중 canonical»**. AU144.13 에서 규명한 구조적 원인을
    //   푸는 변경이다:
    //
    //   원천의 압축 계열(`FUN_1004b520`/`FUN_1004bae0`/`FUN_10025b30`/`FUN_10026300`)은
    //   전부 `+0x15c`(진행 경계)·`+0x160`(배치 경계)·`+0x164`(그룹 경계) 를 입력으로 받고,
    //   그 상태는 **배치가 아직 안 끝났을 때만** 유효하다. 종전 구현은 배치를 100%
    //   끝낸 뒤 `canonical_pass` 를 돌렸으므로 이 세 값이 전부 «전체» 로 고정돼
    //   `& 1`(배치완료) 판별력이 사라졌다(AU144.10·144.11 실측 no-op).
    //
    //   여기서는 첫 배치 루프를 **3분할**하고 각 구간 사이에 `canonical_pass` 를 끼운다.
    //   이때 `commit()` 이 이미 세운 `BIT_PLACED` 가 «배치된 부분집합» 을 정확히 표시하므로
    //   «배치완료 한정 점수»(AU144.7) 가 실제로 후보를 가른다.
    if cfg.canonical || cfg.native_lp {
        // ★ 재작성 (§12.1.34 AU101~AU104) — 원천 구조는 **2단** 이다.
        //
        //   1. **최초 배치** — 기존 `place()` 부착 체인으로 156 조각을 모두 놓는다.
        //      (원천은 모든 조각이 위치를 가진 상태에서 다음 단계를 돈다. 그래서 `+0xc0`
        //       «현재 위치» 를 조건에 쓸 수 있다 — AU103.)
        //   2. **반복 개선** — `FUN_10028340` 대로 정렬 목록을 훑으며 각 항목을 anchor 로
        //      두고 좌우 이웃만 보고 재배치한다. 이 반복이 «매 반복 정렬 + 재배치» 의 실체다.
        //
        // §12.1.43 AU145 — 1단을 3분할해 사이에 개선 패스를 끼운다(«배치 도중 개선»).
        //   구간 수 = 3, 경계에서 `canonical_pass` + LP 압축 1회.
        // ★ §12.1.43 AU145 — «배치 도중 개선» (원천 `FUN_1004b490` 의 **재시도 루프**).
        //
        //   AU144.13 에서 확정한 구조적 원인을 푼다: 원천 압축 계열은
        //   `+0x15c`(진행 경계)·`+0x160`(배치 경계)·`+0x164`(그룹 경계) 를 입력으로 받고
        //   그 상태는 **배치가 안 끝났을 때만** 유효하다. 배치를 100% 끝낸 뒤 개선하면
        //   `& 1`(배치완료) 판별력이 사라진다(AU144.10·144.11 실측 no-op).
        //
        //   원천은 «목록을 훑어 놓을 수 있는 것을 놓고 끝» 이 아니라
        //   **전부 시도 → 재정렬 → 다시 전부 시도** 를 반복한다. 여기서도 그렇게 한다:
        //   한 바퀴 돌려 놓을 수 있는 것을 다 놓고, 남은 것이 있으면 개선 패스를 끼운 뒤
        //   **남은 것만으로 목록을 다시 만들어** 다음 바퀴를 돈다.
        //
        //   ★ 종전 시도(고정 3등분)는 두 번 실패했다: (a) 재정렬 없이 밀어붙여 73.514%,
        //   (b) `pending` 을 줄이면서 인덱스는 원래 길이로 계산해 25.276%/미배치 52.
        //   여기서는 «남은 목록 전체» 를 매 바퀴 처음부터 훑으므로 인덱스 불일치가 없다.
        let max_waves = if cfg.canonical_midpass { 3 } else { 1 };
        // ★ §12.1.43 AU145.5 — 첫 바퀴는 전역 폴백 없이 부착만으로 놓는다.
        //   그러면 «놓지 못한 조각» 이 남아 `+0x15c`(진행 경계)가 실재하고,
        //   `& 1`(배치완료) 이 «확정 부분집합» 을 나타낸다. 이후 바퀴에서 폴백을 켜
        //   74% 대 품질로 회복한다.
        ctx.midpass_off_waves = if cfg.canonical_midpass_late_global { Some(1) } else { None };
        let mut pending: Vec<usize> = order.clone();
        for _wave in 0..max_waves {
            ctx.midpass_wave = _wave;
            // ★ §12.1.43 AU147 — 첫 바퀴에서 «목표 폭 상한» 을 걸어 배치를 좁은 영역에 가둔다.
            //   원천 «기준 폭»(`FUN_10025950`) 대응. 상한값 = «가장 큰 조각 폭 × k».
            //
            // ★★ §12.1.43 AU148 — **매 바퀴 점진 확대**. 원천의 기준 폭(`+0x1c[0x28]`)은
            //   고정이 아니라 `FUN_10025950` 이 **매번 재설정**한다. 즉 «좁게 시작해 배치가
            //   진행되며 단계적으로 넓어지는» 것이 원천의 거동이다.
            //   여기서는 배수 `k` 를 바퀴마다 `k_base + wave` 로 키운다(2 → 3 → 4 …).
            //   상한이 «이미 배치된 폭» 보다 작아지면 아무것도 못 놓으므로 `used_w` 와 max 를
            //   취해 하한을 보장한다.
            ctx.target_cap = if cfg.canonical_midpass_late_global {
                let widest = (0..inst.pieces.len())
                    .flat_map(|i| inst.pieces[i].shapes.iter())
                    .map(|&s| inst.shapes[s].bbox.x1)
                    .max()
                    .unwrap_or(0);
                let mult = cfg.canonical_cap_mult.max(1) + _wave as i64;
                let cap = if cfg.canonical_cap_permille > 0 {
                    widest.saturating_mul(cfg.canonical_cap_permille) / 1000
                } else {
                    widest.saturating_mul(mult)
                };
                if cfg.canonical_cap_grow {
                    // 진행중 상한: «배치된 폭» 보다 작아지지 않게 한다.
                    Some(cap.max(ctx.used_w))
                } else if _wave == 0 {
                    Some(cap)
                } else {
                    None
                }
            } else {
                None
            };
            let wave_list = std::mem::take(&mut pending);
            if wave_list.is_empty() {
                break;
            }
            let placed_before = ctx.placed.len();
            for &k in &wave_list {
                if Instant::now() > ctx.deadline {
                    timed_out = true;
                    break;
                }
                if states[k] == State::Placed {
                    continue;
                }
                let attach = ctx.placed.last().copied();
                match ctx.place(k, attach) {
                    Some((p, g)) => {
                        ctx.commit(p, g);
                        via_global.push(g);
                        states[k] = State::Placed;
                        // ★★★ §12.1.81 AU190 — **조각마다 LP** (원천 `FUN_1004ae60`).
                        //
                        //   원천 `FUN_1004ad80` 은 한 조각을 놓을 때마다
                        //   `FUN_1004ae60` 을 부르고, 그 안에서
                        //   `FUN_1004b490`(배치) → `FUN_1004aee0`(LP) 가 **연속**한다:
                        //   ```c
                        //   FUN_1004ad80:  +0x7c++;  FUN_1004ae60();
                        //   FUN_1004ae60:  FUN_1004b490();   // 배치
                        //                  FUN_10022270(0x100);
                        //                  FUN_1004aee0();   // ★ LP
                        //                  FUN_10028cd0();   // 정방향 시작 인덱스
                        //   ```
                        //   종전 우리 `sweep` 은 조각 루프를 **전부 마친 뒤** LP 를 한 번
                        //   돌렸고(게다가 `max_waves > 1` 일 때만), 그래서 LP 가 배치
                        //   진행에 영향을 주지 못했다.
                        if cfg.native_lp && ctx.placed.len() >= 2 {
                            if let Some(lp) = cfg.canonical_lp.as_deref() {
                                if Instant::now() < ctx.deadline {
                                    let before = ctx.placed.clone();
                                    let mut c = lp.clone();
                                    c.reverse_mode = cfg.reverse_mode;
                                    // ★★★ §12.1.81 AU190 — **국소 범위** LP.
                                    //
                                    //   원천 `FUN_1004a690`(정방향) 은 조각마다
                                    //   `FUN_1004b050(iVar1, iVar4)` 를 부르는데, 이
                                    //   함수는 `+0x158`~`+0x164` **구간만** 순회한다
                                    //   (`chunk_003.c:20550~20581`). 즉 조각 1개를 놓을
                                    //   때마다 **직전 조각 주변만** 다시 푼다.
                                    //
                                    //   역방향 `FUN_100281c0` 은 반대로 `0..+0x1b0`
                                    //   **전체**를 돈다.
                                    //
                                    //   ★ AU192 — `n-3` 근사를 **원천 `FUN_10025b30`**
                                    //   정확 구간으로 교체했다. `+0x15c`(진행 경계)부터
                                    //   같은 부품이 이어지는 구간이다.
                                    let n_pl = ctx.placed.len();
                                    let (g_lo, g_hi) = if cfg.reverse_mode {
                                        (0, n_pl)
                                    } else {
                                        // ★ AU192 — 목표 폭은 `used_w` 가 아니다.
                                        //   원천 `FUN_10026520`:
                                        //   ```c
                                        //   +0x74 = +0xac[+0x158].x;      // ★ 경계 조각의 x
                                        //   iVar3 = [+0x194][부품].오프셋 + +0x70;
                                        //   while (iVar1 < +0x4c) {
                                        //       if (iVar3 < +0xac[iVar1].x) break;
                                        //       iVar1++;
                                        //   }
                                        //   +0x164 = iVar1;
                                        //   ```
                                        //   `used_w` 를 넘기면 전 조각이 통과해
                                        //   `advance_tail` 이 `n` 을 돌려주고 구간이
                                        //   `(n,n)` 으로 비어 LP 가 안 돈다
                                        //   (AU192 계측: `[SPAN] n=2 g=(2,2)` 2,170회).
                                        //
                                        //   ★ `sweep` 말미(§AU134.1)가 이미
                                        //   `ctx.tail_end = advance_tail(..., target_w)` 로
                                        //   `+0x158` 을 계산해 둔다. 그 값을 쓴다.
                                        let tail = (ctx.tail_end.max(0) as usize).min(n_pl);
                                        compact::group_span_by_id(
                                            &ctx.placed,
                                            &ctx.inst.shapes,
                                            tail,
                                        )
                                    };
                                    // 구간이 비면(그룹 없음) LP 를 건너뛴다.
                                    if g_lo < g_hi {
                                        let start = g_lo;
                                        c.group_span = Some((start, g_hi));
                                    // ★ 국소 범위에서는 `slack_box`(전체 156개)·`hi_in` 을
                                    //   넘기면 인덱스가 어긋난다(`group_span` 은 4개인데
                                    //   `slack_box` 는 156개 → `compact.rs:1039` panic).
                                    //   원천 `FUN_1004b050(iVar1, iVar4)` 도 구간 내에서만
                                    //   변수 상자를 다시 만든다. 국소 범위에서는 상자를
                                    //   «단계 상자» 기본값으로 둔다.
                                    c.slack_box = None;
                                    c.hi_in = None;
                                    c.cap_target = None;
                                    c.native_prop = cfg.native_prop;
                                    let _ = crate::compact::restart_improve(
                                        inst,
                                        &mut ctx.placed,
                                        &mut ctx.cache,
                                        &c,
                                        ctx.deadline,
                                    );
                                    // 폭이 나빠졌으면 되돌린다 (`FUN_1004b490` 스냅샷 복원 대응).
                                    let w_new = crate::compact::width_grid(inst, &ctx.placed);
                                    let w_old = crate::compact::width_grid(inst, &before);
                                    if w_new > w_old {
                                        ctx.placed = before;
                                    } else if w_new < w_old {
                                        // ★★★ §12.1.82 AU191 — **진행 경계 전진**
                                        //   (원천 `FUN_1004b520` 끝부분).
                                        //
                                        //   ```c
                                        //   *(int *)(in_EAX + 0x164) = iVar6;   // 진행 경계 확장
                                        //   *(double *)(in_EAX + 0xa4) += (감소분);
                                        //   ```
                                        //   LP 로 조각을 당긴 뒤 «여기까지 확정» 경계를
                                        //   전진시킨다. 이게 없으면 폭 감소가 다음
                                        //   배치에 전달되지 않는다.
                                        let adv = compact::advance_tail(
                                            &ctx.placed,
                                            &ctx.inst.shapes,
                                            0,
                                            ctx.used_w,
                                        );
                                        ctx.tail_end = ctx
                                            .tail_end
                                            .max(adv as i32)
                                            .min(ctx.placed.len() as i32);
                                    }
                                    }
                                }
                            }
                        }
                    }
                    None => {
                        states[k] = State::Held;
                        pending.push(k);
                    }
                }
            }
            if timed_out || pending.is_empty() {
                break;
            }
            // ★★ §12.1.45 AU152 — «미배치 국소 재정렬».
            //   원천 `FUN_1004b490` 은 배치 루프 안에서 매 반복 `FUN_10028210`
            //   (키 = `+0xac[i].x − +0x8c[i].x` qsort) 을 다시 호출한다. 종전
            //   `Order::LeftFree`(AU66) 는 이 정렬을 «초기 1회» 만 해 AU151.4 에서
            //   7242.75mm 로 악화됐다 — 원인은 키가 아니라 **정렬 시점**이었다.
            //
            //   여기서는 **아직 놓지 못한 조각(`pending`)만** 재정렬한다. 키는 원천의
            //   `+0xac.x`(작업 좌표 x) 대응 — 우리 모델에서 조각의 «배치될 x» 는
            //   아직 없으므로, 현재 사용 폭(`used_w`)에서 조각 폭을 뺀 «우측 정렬
            //   기준 좌측 끝» 을 쓴다. 즉 «현재 폭 오른쪽 끝에 붙을 수 있는,
            //   큰 것부터» 순서다. 이미 놓은 조각은 건드리지 않으므로 AU148 의
            //   «전체 재배치»(`--reorder-rounds 8` = 71.021%) 처럼 파괴적이지 않다.
            if cfg.canonical_local_reorder && _wave + 1 < max_waves {
                let width_of = |k: usize| {
                    inst.pieces[k]
                        .shapes
                        .iter()
                        .map(|&s| inst.shapes[s].bbox.x1)
                        .max()
                        .unwrap_or(0)
                };
                // ★ AU152.3 — 원천 키 `+0xac[i].x` 는 «작업 좌표 x» 이고, qsort 는
                //   **오름차순**(= 왼쪽부터)이다. 종전 구현은 `used_w − 폭`(우측 정렬
                //   여지) 내림차순이라 **방향이 반대**였다. 원천대로 «조각 폭 내림차순»
                //   으로 두면 좌표가 같을 때 큰 것이 먼저 오는 것과 같은 뜻이 되고,
                //   배치 진행 중에는 «현재 폭에 큰 것부터 밀어 넣는» 순서가 된다.
                pending.sort_by(|&a, &b| width_of(b).cmp(&width_of(a)));
            }
            // ★ 바퀴 사이 — «배치 도중» 이므로 `& 1` 이 «이미 확정된 부분집합» 을 나타낸다.
            //   여기서 개선 패스가 원천의 «진행 중 경계» 조건을 처음으로 만족한다.
            if _wave + 1 < max_waves && ctx.placed.len() >= 2 {
                let _ = ctx.canonical_pass();
                if let Some(lp) = cfg.canonical_lp.as_deref() {
                    let before = ctx.placed.clone();
                    let rep = crate::compact::restart_improve(
                        inst,
                        &mut ctx.placed,
                        &mut ctx.cache,
                        lp,
                        ctx.deadline,
                    );
                    if ctx.placed.len() != before.len() {
                        ctx.placed = before;
                    } else {
                        let _ = rep;
                    }
                }
            }
            if ctx.placed.len() == placed_before {
                break;
            }
        }
        // ★ 폴백 지연·상한 해제 — 이후 2차 패스/회차는 정상 동작한다.
        ctx.midpass_off_waves = None;
        ctx.target_cap = None;
        // 바퀴를 다 돌고도 못 놓은 조각은 `pending` 에 남는다(`held` 가 아니라). 2차 패스가
        // 그것을 받아야 상한·폴백이 풀린 상태에서 재시도된다 — 종전에는 `held` 가 비어
        // 2차 패스가 무동작이었고 그 조각들은 `Held` 로 남은 채 실패로 세지도 않았다.
        held.extend(pending.drain(..));
        ctx.stats.held_first_pass = held.len();
        for k in held {
            if timed_out || Instant::now() > ctx.deadline {
                timed_out = true;
                break;
            }
            let attach = ctx.placed.last().copied();
            match ctx.place(k, attach) {
                Some((p, g)) => {
                    ctx.commit(p, g);
                    via_global.push(g);
                    states[k] = State::Placed;
                }
                None => states[k] = State::Failed,
            }
        }
        // ★ 재작성 (§12.1.34 AU115) — 원천 `FUN_1004ae60` 의 **3단 교대 루프**.
        //
        //   원천:
        //     FUN_1004ae60() {                       // 매 회차
        //         FUN_1004b490(this);                // ① 배치 + 매 반복 재정렬
        //         if (상태 정상) {
        //             FUN_1004aee0();                // ② LP 압축 (좌표 이동!)
        //             if (상태 정상) FUN_10029cc0(); // ③ Complete
        //         }
        //     }
        //
        //   ②가 좌표를 실제로 이동시키므로 다음 회차의 정렬 키(`+0xac`)와 NFP 접촉 자리가
        //   달라진다 — 이것이 개선의 원동력이다. ③(Complete)은 `pipeline` 이 담당한다.
        // ★★ §12.1.43 AU149 — «절대 목표 폭» 을 LP 압축에 전달할 설정 사본.
        //   배치 상한(AU147)과 같은 «가장 큰 조각 폭 × k» 를 LP 의 L 제약 기준으로 넘긴다.
        //   그러면 좁게 배치한 결과를 LP 가 그 폭에 맞춰 조인다.
        let lp_with_cap: Option<Box<crate::compact::CompactConfig>> = match cfg.canonical_lp.as_deref() {
            Some(lp) if cfg.canonical_cap_to_lp => {
                let widest = (0..inst.pieces.len())
                    .flat_map(|i| inst.pieces[i].shapes.iter())
                    .map(|&s| inst.shapes[s].bbox.x1)
                    .max()
                    .unwrap_or(0);
                let cap = if cfg.canonical_cap_permille > 0 {
                    widest.saturating_mul(cfg.canonical_cap_permille) / 1000
                } else {
                    widest.saturating_mul(cfg.canonical_cap_mult.max(1))
                };
                let mut c = lp.clone();
                c.cap_target = Some(cap);
                c.reverse_mode = cfg.reverse_mode;
                Some(Box::new(c))
            }
            // ★★★ §12.1.80 AU189 — **역방향 전파.** `reverse_mode` 는 `sweep` 의
            //   호출 사본에만 설정되고 이 LP 설정으로는 전달되지 않아서, 역방향
            //   sweep 이 정방향과 **완전히 같은 배치**를 냈다(실측: NFP 히트만 2배,
            //   결과 동일). 여기서 반드시 넘겨야 `compact::build` 가 목적계수를
            //   건너뛴다.
            other => other.map(|l| {
                let mut c = l.clone();
                c.reverse_mode = cfg.reverse_mode;
                Box::new(c)
            }),
        };
        // ★★★ §12.1.81 AU190 — **원천 파이프라인 이식** (`--autonest-native`).
        //
        //   원천 `FUN_1004ae60` 은 «배치(`FUN_1004b490`) → LP(`FUN_1004aee0`)» 를
        //   **한 함수 안에서** 돌린다. 우리 `--max-density` 는 `canonical_rounds = 0`
        //   이라 이 블록 전체를 건너뛰어 **LP 가 한 번도 돌지 않았다**(AU189 계측:
        //   `[BUILD]` 0건). `native_lp` 는 그 게이트를 우회한다.
        //
        //   대응표: `.hermes/plans/autonest-native-pipeline.md` §4.
        if ctx.placed.len() == n && (cfg.canonical_rounds > 0 || cfg.native_lp) {
            // ★★★ §12.1.76 AU179 — 원천 안쪽 루프 종료 조건 완전 이식.
            //
            // 원천 (`chunk_003.c:20352~20358`):
            // ```c
            // do {
            //     iVar3 = FUN_1004ad80();     // 교대 3단 (회차 +1 → ①②③)
            //     if (iVar3 == 0) break;
            //     iVar3 = FUN_1004c1a0();     // ★ «계속?» 판정 — 매 회차
            // } while (iVar3 != 0);
            // ```
            // 즉 회차 상한(`+0x394`=20)은 `FUN_1004c1a0` **안에서** 검사되고,
            // 수렴 임계(`+0x398`=100)도 같은 함수가 본다. 종전 우리 코드는
            // `for 0..rounds` + `deadline` 만 보고 `should_continue` 결과를
            // **루프 밖에서 한 번** 계산해 통계로만 남겼다 → «수렴해도 계속 도는»
            // 동작이 나왔고, 이것이 `--canonical-rounds` 를 올리면 값이 계속
            // 오르던 원인이었다(AU179 정정).
            let iter_cfg = compact::IterCfg::default();
            // ★★★ §12.1.81 AU190 — `native_lp` 는 `canonical_rounds = 0` 이어도
            //   **최소 1회** 돌아야 한다. 원천 `FUN_1004ae60` 은 «배치 → LP» 를
            //   매 조각마다 부르므로, 회차 상한이 0 이면 LP 가 영원히 안 돈다.
            let n_rounds = if cfg.canonical_rounds > 0 { cfg.canonical_rounds } else { 1 };
            for _round in 0..n_rounds {
                if Instant::now() > ctx.deadline {
                    timed_out = true;
                    break;
                }
                // ★★★ §12.1.78 AU181 — `+0x7c` 회차 카운터 증가 (`FUN_1004ad80` 대응).
                //
                // 원천 `FUN_1004ad80` 은 **진입할 때마다** `+0x7c += 1` 을 한다:
                // ```c
                // *(int *)(in_EAX + 0x7c) = *(int *)(in_EAX + 0x7c) + 1;
                // iVar2 = *(int *)(in_EAX + 0xa0);
                // if (+0x7c == +0xa0) { +0xa0 = iVar2 * 2; realloc(+0x9c, iVar2 * 0x10); }
                // FUN_1004ae60();
                // ```
                //
                // ★ 종전 코드는 이 증가가 **루프 밖**(`if cfg.canonical` 블록, 구 2355행)에
                //   있었다 → `ctx.round` 가 회차마다 안 올라가 `should_continue` 의
                //   `round < +0x394(20)` 검사가 **항상 참**이 되어, `--canonical-rounds`
                //   를 올리면 값이 계속 올랐다(75.417 → 75.684). AU181 정정.
                //
                // ★ 주의: `ctx.round` 는 `span = np/(round+1)`·`jitter = 75+75*(round%4)`
                //   등 «부수 계산» 에도 쓰인다. 종전에는 회차 내내 같은 값(1)이었으므로,
                //   여기서 올리면 그 계산들이 매 회차 달라져 `group_span` 이 어긋나고
                //   LP 열 수가 맞지 않아 패닉한다(실측: `len 157, index 157`).
                //   그래서 **판정 전용 카운터** `iter_round` 를 따로 둔다 — 원천 `+0x7c`
                //   는 «회차 상한 판정» 에만 쓰이고, `+0x58`(난수 진동)은 별도 필드다.
                ctx.iter_round += 1;
                // ★★★ §12.1.77 AU180 — 회차 시작: 좌표 스냅샷 + 플래그 리셋.
                //   원천 `FUN_1004a140` 은 좌표 델타로 판정하므로 «회차 시작 좌표» 기준이다.
                //   `+0xc8` 은 회차마다 0 으로 리셋, `+0xcc` 도 회차 진입에서 0.
                ctx.round_start_pos = ctx.placed.iter().map(|p| (p.pos.x, p.pos.y)).collect();
                ctx.lp_moved_valid = false;
                ctx.iter_move_flag = false;
                // ★ §12.1.40 AU140 — 이번 회차의 부착 발생 횟수 스냅샷.
                let attach_before = ctx.stats.attach_events;
                // ①-a 매 반복 상태 초기화 (원천 `FUN_10028210` 의 `&= 6`).
                ctx.reset_bits();
                // ①-b 배치 + 재정렬 (원천 `FUN_1004b490`).
                let moved = ctx.canonical_pass();
                // ★★★ §12.1.61 AU170 — ⑤-a «그룹 확장 밀어넣기» (`FUN_1004b520`).
                //
                //   원천 회차 루프(`FUN_1004ad80`)는 Complete 이후에:
                //   ```c
                //   if ((+0x20[0x14]==0) || (+0x188!=0) || (+0x164 == +0x4c))
                //       FUN_1004b520();   // ⑤-a  ← 우리는 «전부 배치» 이므로 이쪽
                //   ```
                //   그리고 `FUN_1004b520` 은 **그룹 구간(`+0x160..+0x164`)만** LP 로
                //   다시 풀어 해를 좌표에 적용한다(`FUN_1004a4b0` → `FUN_1004a520`
                //   → `FUN_1004a140`).
                //
                //   구간 끝(`+0x164`) = «목표 폭 안에 드는 마지막 위치»(`advance_tail`).
                //   구간 시작(`+0x160`) = «그 그룹의 첫 조각» — 우리는 회차에 따라
                //   구간을 넓혀 간다(회차 0 = 앞 절반, 이후 전 구간).
                let group_moved = if cfg.canonical_group_lp && moved {
                    if let Some(lp) = lp_with_cap.as_deref() {
                        let np = ctx.placed.len();
                        // ★ AU171.5 — 원천은 조각 수 156(<200)에서 «그룹 분할 안 함»
                        //   (`chunk_002.c:10472~10622`: 첫 회차 `+0x184=전체, +0x17c=1`).
                        //   따라서 이 경로는 «원천 동작 재현용» 이며 기본 OFF 다.
                        let base = ctx.part_offsets.target_width(
                            ctx.placed.last().map(|p| p.shape).unwrap_or(0),
                            ctx.used_w,
                        );
                        let g_end = compact::advance_tail(&ctx.placed, &ctx.inst.shapes, 0, base).min(np);
                        let span = (np / (ctx.round as usize + 1).max(1)).max(1);
                        let g_start = g_end.saturating_sub(span);
                        if g_end > g_start {
                            let before = ctx.placed.clone();
                            let w_before = compact::width_grid(inst, &ctx.placed);
                            let mut c = lp.clone();
                            c.group_span = Some((g_start, g_end));
                            let _ = compact::restart_improve(
                                inst,
                                &mut ctx.placed,
                                &mut ctx.cache,
                                &c,
                                ctx.deadline,
                            );
                            let w_after = compact::width_grid(inst, &ctx.placed);
                            if ctx.placed.len() == np && w_after < w_before {
                                true
                            } else {
                                ctx.placed = before;
                                false
                            }
                        } else {
                            false
                        }
                    } else {
                        false
                    }
                } else {
                    false
                };
                // ★★★ §12.1.54 AU160 — `FUN_10022270(0x100)` 대응. **배치 직후**(LP 전)
                //   여유 테이블을 갱신하고, 그 값을 LP «변수 상자» 로 넘긴다.
                //   원천 `FUN_1004ae60`: `배치 → 갱신 → LP → 갱신 → Complete → 갱신`.
                let lp_round: Option<Box<crate::compact::CompactConfig>> = if cfg.canonical_slack || cfg.native_lp {
                        if cfg.canonical_slack {
                            ctx.refresh_slack();
                        }
                        lp_with_cap.as_deref().map(|lp| {
                            let mut c = lp.clone();
                            c.slack_box = Some(ctx.slack_boxes());
                            // ★★★ §12.1.57 AU163 — `lo_in`/`hi_in` 정확 이식.
                            //
                            //   원천 (`chunk_002.c:11704~11706`, `FUN_10026520`):
                            //   ```c
                            //   +0x78 = +0x58;      // ★ +0x58 = «난수 진동 폭» (75~300)
                            //   FUN_10026420();     // +0x74 = +0xac[+0x158].x;  return +0x74 − +0x70
                            //   ```
                            //   그리고 `FUN_10026520` 이 `+0x78 = 부품오프셋 + +0x70` 으로 덮어쓴다.
                            //
                            //   ★ `+0x58` 출처 (`FUN_10013350`, chunk_001.c:17382~17403):
                            //   ```c
                            //   iVar1 = +0x10;                       // 반복 계열
                            //   if (iVar1 < 5)  +0x58 = 75;          return;
                            //   if (iVar1 < 10) +0x58 = 100 − rand(); return;
                            //   if (iVar1 < 20) +0x58 = 125 − rand(); return;
                            //   if (iVar1 < 30) +0x58 = 150 − rand(); return;
                            //   if (iVar1 ≤ 39) +0x58 = 250 − rand(); return;
                            //   else            +0x58 = 300;          return;
                            //   ```
                            //   **즉 `hi_in` 은 «폭»(수천mm)이 아니라 «75~300 격자(0.75~3mm)의
                            //   난수 진동» 이다.** AU162 에서 `used_w`(6800mm)를 넣은 것은
                            //   4자리 스케일 오류였다.
                            //
                            //   `+0x70` = «모든 조각 여유의 최대»(`FUN_10025770`,
                            //   초기값 `+0x390`=20000) → 우리는 `refresh_slack` 의 여유 최대값.
                            let (lo_in, hi_in) = {
                                // ★★★ §12.1.58 AU164 정정 — `+0x74` 는 «경계 조각의 현재 x» 다
                                //   (`FUN_10026420`: `+0x74 = +0xac[+0x158].x`).
                                //   AU163 까지 `ctx.target_w`(전체 목표 폭, 6815mm)를 넣어
                                //   `hi_in` 이 조각 우측 끝(1409mm)과 4.8배 차이 → 무효였다.
                                //   `+0x158` = 진행 경계 → 우리 `ctx.tail_end` 대응.
                                let bi = (ctx.tail_end.max(0) as usize).min(ctx.placed.len());
                                let l = if bi < ctx.placed.len() {
                                    let pp = ctx.placed[bi];
                                    pp.pos.x + ctx.inst.shapes[pp.shape].bbox.x0
                                } else {
                                    0
                                };
                                // `+0x70` = 여유 최대값.
                                let slack_max = ctx
                                    .slack_left
                                    .iter()
                                    .map(|v| v.0)
                                    .chain(ctx.slack_right.iter().map(|v| v.0))
                                    .filter(|&v| v != i64::MAX && v != i64::MIN)
                                    .max()
                                    .unwrap_or(c.step_box.max(1));
                                // `+0x78` = «+0x58 난수 진동» (75~300).
                                let mut jitter: i64 = 75 + 75 * ((ctx.round % 4) as i64);
                                // ★★★ §AU185 — `+0x37c`(허용오차 동적계산) 가 켜지면
                                //   원천은 `+0x78 = +0x58 * 2` 로 **탐색 폭을 2배** 한다
                                //   (`chunk_002.c:10474~10476`). 모드 4 블록에서만 1 이다
                                //   (`chunk_001.c:7530~7531` 설정 / `7554` 복원).
                                if ctx.cfg.canonical_dyn_tol {
                                    jitter *= 2;
                                }
                                (l, l.max(slack_max).saturating_sub(jitter))
                            };
                            c.hi_in = Some((lo_in, hi_in));
                            // ★★★ §12.1.81 AU190 — **원천 `+0x368` 의 «재구성 범위» 이식**
                            //   (`FUN_1004a690` 정방향 vs `FUN_100281c0` 역방향).
                            //
                            //   원천 (`chunk_002.c:13429~13444`, `FUN_10028cd0`):
                            //   ```c
                            //   iVar1 = +0x180;                    // 정방향 시작 인덱스
                            //   if (+0x1ac[+0x180-1] != +0x180) {
                            //       for (i = 0; i < +0x1b0; i++)
                            //           if (+0x1ac[i] != 0 && ++iVar2 == +0x180) return i;
                            //       return -1;
                            //   }
                            //   ```
                            //   `FUN_1004a690` 은 **그 시작점 이후만** 조각별 LP 를 돌린다:
                            //   ```c
                            //   iVar1 = FUN_10028cd0();                // 시작 = 마지막 배치 조각
                            //   do { FUN_10028d10(iVar4);
                            //        iVar3 += FUN_1004b050(iVar1, iVar4); }
                            //   while (++iVar4 < +0x1b0);
                            //   ```
                            //   반면 `FUN_100281c0`(역방향) 은 **처음부터 전체**를 돈다:
                            //   ```c
                            //   for (i = 0; i < +0x1b0; i++) {
                            //       FUN_10028e60(EDI, i);              // 조각별 접촉쌍 재탐색
                            //       (*EBX)();                          // FUN_1004b050
                            //   }
                            //   ```
                            //   → **정방향 = 진행 경계 이후만, 역방향 = 전체.**
                            if cfg.native_lp {
                                let start = if cfg.reverse_mode {
                                    0
                                } else {
                                    (ctx.tail_end.max(0) as usize).min(ctx.placed.len())
                                };
                                c.group_span = Some((start, ctx.placed.len()));
                            }
                            Box::new(c)
                        })
                    } else {
                        lp_with_cap.clone()
                    };
                // ② LP 압축 (원천 `FUN_1004aee0`) — 비겹침 제약 LP 로 좌표를 미세조정.
                let lp_moved = if let Some(lp) = lp_round.as_deref() {
                    let before = ctx.placed.clone();
                    let rep = crate::compact::restart_improve(
                        inst,
                        &mut ctx.placed,
                        &mut ctx.cache,
                        lp,
                        ctx.deadline,
                    );
                    if ctx.placed.len() != n {
                        ctx.placed = before;
                        false
                    } else {
                        // ★★★ §12.1.80 AU189 — **역방향은 «폭 감소» 를 요구하지 않는다.**
                        //
                        //   `lp_moved` 의 채택 규칙은 «폭이 줄었는가» 다. 그런데 역방향은
                        //   목적계수를 끄므로 **LP 가 폭을 줄이려 하지 않는다** — 결과가
                        //   항상 기각되어 배치가 정방향과 완전히 같아진다(실측: NFP 히트만
                        //   2배, 밀도 75.887% 동일).
                        //
                        //   원천 `+0x368 = 2` 는 cos/sin 을 반전해 **탐색 방향을 뒤집는**
                        //   것이지 «폭을 줄이는» 단계가 아니다. 따라서 역방향에서는
                        //   겹침 없이 유효하기만 하면 결과를 **받아들인다**.
                        if lp.reverse_mode {
                            rep.width_after_mm.is_finite()
                        } else {
                            rep.width_after_mm < rep.width_before_mm - 1e-9
                        }
                    }
                } else {
                    false
                };
                // ★ ③ «그룹 확장 밀어넣기» (원천 `FUN_1004b520`) — AU120~AU123, AU141 재작성.
                //   원천은 이 단계를 교대 «밖» 에서 부르지만, 매 회차 개선을 위해 안에 둔다.
                //   AU141.5 — `state`/`variant` 를 넘겨 «고정/자유» 분기를 원천대로 태운다.
                let pushed = if let Some(lp) = lp_with_cap.as_deref() {
                    let before = ctx.placed.clone();
                    let w_before = crate::compact::width_grid(inst, &ctx.placed);
                    // `+0xe0` 대응: `bits[i][i]` 의 하위 바이트. `+0x140` 대응: `ctx.variant`.
                    let n = ctx.placed.len();
                    let state: Vec<u8> = (0..n).map(|i| ctx.bits[i][i]).collect();
                    let variant: Vec<u8> = ctx.variant.clone();
                    // ★★ §12.1.43 AU146 — `tail_start` = «진행 경계». 원천 `+0x164`.
                    //   회차가 진행되면 `ctx.tail_end`(= 목표 폭 이하 조각 수)가 «확정부»
                    //   이고, 그 뒤가 «아직 다듬을 여지가 있는 꼬리» 다. AU141 이식 당시
                    //   이 인자가 없어 `tail_end = n` → 항상 `(0,0)` 반환이었다.
                    let tail_start = (ctx.tail_end.max(0) as usize).min(ctx.placed.len());
                    let (reps, _mv) = crate::compact::push_compress(
                        inst,
                        &mut ctx.placed,
                        &mut ctx.cache,
                        lp,
                        ctx.deadline,
                        cfg.canonical_rounds.max(1),
                        &state,
                        &variant,
                        tail_start,
                    );
                    let w_after = crate::compact::width_grid(inst, &ctx.placed);
                    // 폭이 줄었고 겹침이 없으면 채택, 아니면 되돌린다.
                    if reps > 0 && w_after < w_before {
                        true
                    } else {
                        ctx.placed = before;
                        false
                    }
                } else {
                    false
                };
                // ★ ③-b «부품 오프셋 기반 일괄 이동» (원천 `FUN_1004bae0`) — AU143.7.
                //   `push_compress` 와 별개다. 같은 부품(형상) 조각들을 «오프셋 차이» 만큼
                //   한꺼번에 옮긴다. `canonical` 경로에서만 돈다.
                // ★★★ §12.1.62 AU171 — ⑤-b 는 이제 `canonical_family_lp` 로 개별 게이트된다.
                let family = if cfg.canonical_family_lp {
                    if let Some(lp) = lp_with_cap.as_deref() {
                    let before = ctx.placed.clone();
                    let w_before = crate::compact::width_grid(inst, &ctx.placed);
                    let n = ctx.placed.len();
                    let state: Vec<u8> = (0..n).map(|i| ctx.bits[i][i]).collect();
                    let variant: Vec<u8> = ctx.variant.clone();
                    let (reps, _mv) = crate::compact::family_shift(
                        inst,
                        &mut ctx.placed,
                        &mut ctx.cache,
                        lp,
                        ctx.deadline,
                        &ctx.part_offsets,
                        &state,
                        &variant,
                    );
                    let w_after = crate::compact::width_grid(inst, &ctx.placed);
                        if reps > 0 && w_after < w_before {
                            true
                        } else {
                            ctx.placed = before;
                            false
                        }
                    } else {
                        false
                    }
                } else {
                    false
                };
                // ★ ★ §12.1.40 AU140 — 교대 종료 판정 정정.
                //
                // 기존 조건: `!moved && !lp_moved && !pushed` → 즉시 break.
                //   원천은 이렇게 종료하지 않는다. 원천 안쪽 루프(`chunk_003.c:20352`)는
                //   `FUN_1004ad80()`(=교대 3단)이 0 을 돌려줄 때만 빠져나온다.
                //
                //   그리고 AU140.2 로 `should_continue`(`FUN_1004c1a0`)의 실질 분기는
                //   «부착 발생(`+0x80 == 0x50`) 시에만 true» 임이 확정됐다. 즉
                //   **배치가 «새 부착» 을 만들었으면 다음 회차를 돈다**가 원천 규칙이다.
                //
                //   우리 `attach_events` 는 회차 중 «부착 발생» 횟수의 근사다(AU131.3).
                //   회차 시작 시 값을 스냅샷해 «이번 회차에 부착이 있었는가» 로 판정한다.
                let attach_this_round = ctx.stats.attach_events > attach_before;
                // ★★★ §12.1.76 AU179 — `FUN_1004c1a0` «계속?» 판정을 **매 회차** 부른다.
                //
                // 원천 안쪽 루프는 `do { FUN_1004ad80(); if (!FUN_1004c1a0()) break; }` 다.
                // 종전 코드는 `ctx.round < ROUND_CAP` 만 봤고 `should_continue` 는 루프
                // 밖에서 한 번 계산해 통계로만 남겼다 → **수렴해도 상한까지 계속 돌아**
                // `--canonical-rounds` 를 올리면 값이 계속 올랐다(AU179 정정).
                //
                // ★★★ §12.1.77 AU180 — `+0xc8`(LP 이동 유효) / `+0xcc`(이동 임계 초과)도
                //   함께 배선한다. `FUN_1004a140` 이 LP 해를 좌표에 적용한 뒤:
                //     · dx·dy 각각 `FUN_10049f00` → 유효하면 `+0xc8 = 1`
                //     · `|dx| > 30 || |dy| > 30` 이면 `+0xcc = 1` (한번 1이면 유지)
                //   `+0xc8 != 0` 이면 `should_continue` 가 **개선 판정을 건너뛰고 계속**한다.
                let _ = (moved, lp_moved, pushed, family, group_moved);
                // `+0xcc` — 이번 회차 이동량이 임계(`+0x39c`=30 격자)를 넘었는가.
                if !ctx.iter_move_flag {
                    let thr = MOVE_THRESH_GRID;
                    let moved_far = ctx.placed.iter().any(|p| {
                        let cur = (p.pos.x, p.pos.y);
                        ctx.round_start_pos
                            .get(ctx.placed.iter().position(|q| q.piece == p.piece).unwrap_or(0))
                            .is_some_and(|&(ox, oy)| {
                                (cur.0 - ox).abs() > thr || (cur.1 - oy).abs() > thr
                            })
                    });
                    if moved_far {
                        ctx.iter_move_flag = true;
                    }
                }
                // `+0xc8` — LP 해가 «유효 범위» 안에서 좌표를 움직였는가
                //   (`FUN_10049f00`: `0.8·v < Δ` 또는 `Δ < 0.5·v`).
                //   회차 시작 좌표 대비 이동량으로 판정한다.
                if !ctx.lp_moved_valid && ctx.round_start_pos.len() == ctx.placed.len() {
                    ctx.lp_moved_valid = ctx.placed.iter().enumerate().any(|(i, p)| {
                        let (ox, oy) = ctx.round_start_pos[i];
                        let dx = (p.pos.x - ox).abs();
                        let dy = (p.pos.y - oy).abs();
                        dx > MOVE_THRESH_GRID || dy > MOVE_THRESH_GRID
                    });
                }
                let cur_obj = if ctx.round >= 1 && (ctx.round as usize) - 1 < ctx.hist_w.len() {
                    ctx.hist_w[ctx.round as usize - 1]
                } else {
                    ctx.hist_w.last().copied().unwrap_or(0)
                };
                let st: u16 = if attach_this_round { 0x50 } else { 0 };
                ctx.cont = compact::should_continue_ex(
                    &ctx.hist_w,
                    cur_obj,
                    // ★★★ §12.1.78 AU181 — 원천 `+0x7c`(회차 상한 판정) 대응.
                    //   `ctx.round` 는 부수 계산용으로 1 에 고정돼 있으므로
                    //   **판정 전용** `iter_round` 를 넘긴다.
                    ctx.iter_round,
                    st,
                    1, // `0x368` — 결 방향 모드 1 (AU128.4)
                    &iter_cfg,
                    false, // `+0x164 != +0x4c` — 우리는 항상 전부 배치 (AU180)
                    ctx.lp_moved_valid,
                );
                if !ctx.cont {
                    break; // ★ 원천 `FUN_1004c1a0() == 0` — 수렴 또는 상한 도달
                }
                ctx.stats.canonical_continues += 1;
                // 좌표가 바뀌었으니 사용 폭을 다시 계산한다.
                ctx.used_w = ctx
                    .placed
                    .iter()
                    .fold(0, |m, p| m.max(p.pos.x + inst.shapes[p.shape].bbox.x1));
            }
        }
    } else {
        for &k in &order {
            if Instant::now() > ctx.deadline {
                timed_out = true;
                break;
            }
            let attach = ctx.placed.last().copied();
            match ctx.place(k, attach) {
                Some((p, g)) => {
                    ctx.commit(p, g);
                    via_global.push(g);
                    states[k] = State::Placed;
                }
                None => {
                    states[k] = State::Held;
                    held.push(k);
                }
            }
        }
        ctx.stats.held_first_pass = held.len();

        // 2차 패스 — 보류 체인. 부착 대상은 그 시점의 체인 마지막 노드다.
        for k in held {
            if timed_out || Instant::now() > ctx.deadline {
                timed_out = true;
                break;
            }
            let attach = ctx.placed.last().copied();
            match ctx.place(k, attach) {
                Some((p, g)) => {
                    ctx.commit(p, g);
                    via_global.push(g);
                    states[k] = State::Placed;
                }
                None => states[k] = State::Failed,
            }
        }
    }

    // ★ §12.1.34 AU66·AU90 — 원천은 `FUN_1004b490` 안에서 **매 반복 정렬**한다
    //   (`FUN_10028210` → 키 `+0xac.x − +0x8c.x` 로 qsort). 여기서는 «배치가 끝난
    //   좌표» 기준으로 순서를 다시 계산해 재배치를 시도한다 — 원천의 «반복» 을
    //   신규 엔진 구조에서 근사한 실험 모드다.
    if cfg.reorder_rounds > 0 && ctx.stats.failed == 0 {
        for _round in 0..cfg.reorder_rounds {
            if Instant::now() > ctx.deadline {
                timed_out = true;
                break;
            }
            // 배치된 위치 기준 정렬: 키 = 현재 bbox 좌하단 x (원천 `+0xac.x − +0x8c.x` 근사)
            let mut pos_key: Vec<(i64, usize)> = ctx
                .placed
                .iter()
                .map(|p| (p.pos.x + ctx.coll_box[p.shape].x0, p.piece))
                .collect();
            pos_key.sort_by_key(|&(k, _)| k);
            let new_order: Vec<usize> = pos_key.into_iter().map(|(_, k)| k).collect();
            // 배치 상태를 초기화하고 새 순서로 처음부터 다시 돈다.
            let (snap, kept) = (ctx.placed.clone(), new_order);
            ctx.placed.clear();
            ctx.pairs.clear();
            ctx.used_w = 0;
            for &k in &kept {
                if Instant::now() > ctx.deadline {
                    timed_out = true;
                    break;
                }
                let attach = ctx.placed.last().copied();
                if let Some((p, g)) = ctx.place(k, attach) {
                    ctx.commit(p, g);
                } else if ctx.placed.len() < snap.len() {
                    break;
                }
            }
            if ctx.placed.len() < n {
                // 재배치가 조각을 잃으면 되돌린다.
                ctx.placed = snap;
                break;
            }
        }
    }

    // ★★★ §12.1.34 AU129.1~AU129.3 — «개선 판정 → 최적 복원» 게이트.
    //
    // 원천 구조 (`FUN_1004c2d0` 계열, `chunk_003.c:21326~21340`):
    //
    // ```c
    // if (0x368 == 1 && 1 < +0x7c) {          // 결 방향 모드 + 회차 > 1
    //     if (FUN_1004a640() == 0)            // «개선 없음»
    //         FUN_1004ac30();                 // «최적 배치로 되돌림»
    // }
    // ```
    //
    // 우리 입력은 `grain_directional = true` 이므로 원천 `0x368 == 1` 에 해당한다(AU128.4).
    // `cfg.canonical`(= 재작성/교대 경로 사용) 일 때만 이 게이트를 돈다 —
    // 배포 기본 경로(`canonical == false`)는 무손상 유지한다.
    if cfg.canonical {
        // ① `+0x7c` 회차 카운터 — **루프 안에서** 증가한다(§12.1.78 AU181).
        //    종전에는 여기서 한 번만 올렸고, 그 때문에 `should_continue` 의
        //    `round < +0x394` 검사가 항상 참이 되어 상한이 무력했다.

        // ② 이번 회차 폭을 `+0x9c` 이력에 기록 (`FUN_1004b520` 의 local_28 대응).
        //    원천은 «점수 배열에서 계산한 폭»(local_28)을 쓴다. 우리는 배치된
        //    원본 외곽선의 최대 x 를 쓴다 — `used_w` 가 그 역할이다.
        let cur_w = ctx.used_w;
        ctx.hist_w.push(cur_w);

        // ③ 최적 백업 갱신 (`+0xa4` / `+0xbc`, `20736~20737` 대응).
        //    개선 여부와 무관하게 «현재 상태» 를 백업한다 — 원천도 매 회차 복사한다.
        ctx.best_w = cur_w;
        ctx.best_backup = ctx.placed.clone();

        // ④ 개선 판정 — `FUN_1004a640` 완전 이식.
        //
        // ```c
        // if (+0x20[0x14]==0 || +0x188!=0 || +0x164 == +0x4c)
        //     dVar1 = +0xa4 + _DAT_101544a0;      // «현재 최적 + ε»
        // else
        //     dVar1 = _DAT_101544a0;              // «ε» 만
        // return dVar1 > +0x9c[+0x7c − 1];        // 직전 회차보다 좋아졌는가
        // ```
        //
        // 우리 대응: `+0x20[0x14]` 는 대응 구조 없음(항상 비NULL) → 첫 조건 거짓.
        // `+0x188` = `ctx.mode_flag`. `+0x164 == +0x4c` 는 «전부 배치됨».
        let all_placed = ctx.placed.len() == n;
        ctx.tail_end = if all_placed { n as i32 } else { 0 };
        let use_best_basis = ctx.mode_flag || ctx.tail_end == n as i32;

        // 직전 회차 폭 (`+0x9c[+0x7c − 1]`). 이력은 `[0]`=MAX, `[1]`=1회차 … 로 쌓인다.
        // `ctx.round` 는 1-based 이므로 직전 회차는 «방금 push 한 값의 바로 앞» 이다.
        let ri = ctx.round as usize;
        let prev_w = if ri >= 1 && ri - 1 < ctx.hist_w.len() {
            ctx.hist_w[ri - 1]
        } else {
            i64::MAX
        };

        // 기준값. 원천의 부호(`dVar1 > 이력`)를 «폭은 작을수록 좋다» 로 뒤집어 적용한다:
        //   원천은 점수(클수록 좋음)를 쓰지만 우리 `used_w` 는 폭(작을수록 좋음)이다.
        //   따라서 «기준 − ε > 직전폭» ⇔ «직전폭이 기준보다 ε 이상 작다».
        let baseline = if use_best_basis {
            ctx.best_w + EPS_IMPROVE
        } else {
            EPS_IMPROVE
        };
        let improved = prev_w > baseline;

        // ⑤ 개선이 없으면 최적 배치로 복원 — `FUN_1004ac30` 완전 이식
        //    (`memcpy(+0xac, +0xbc, 4 × +0x4c)`).
        if !improved && !ctx.best_backup.is_empty() && ctx.best_backup.len() == n {
            ctx.placed.clone_from_slice(&ctx.best_backup);
            ctx.used_w = ctx.best_w.max(0);
            ctx.stats.canonical_reverts += 1;
        }

        // ★★★ §12.1.34 AU138.7 — «부품 오프셋 되먹임» (AU133·AU134·AU135 이식).
        //
        // 원천은 «폭 변수 L 을 직접 최소화» 하지 않는다. 대신:
        //
        //   ① [+0x194][부품] = (기준, 오프셋)                     (AU133.1)
        //   ② +0x78 = [+0x194][부품].오프셋 + +0x70               (FUN_10026520)
        //   ③ +0x164 = «+0x78 안에 드는 마지막 미배치 위치»        (FUN_10026520)
        //   ④ 회차 종료: [+0x194][마지막].오프셋 = min(기존, +0x78) (AU133.3)
        //   ⑤ +0xc0 = +0xac + 부품 오프셋                         (FUN_10028990)
        //   ⑥ ②로 돌아감 — 오프셋이 줄었으므로 +0x78 이 작아짐
        //
        // 우리 `Ctx` 에는 부품 개념이 없으므로 **형상(shape)을 부품으로** 쓴다
        // (입력의 `domain.family_id` 가 곧 부품 식별자인 것과 같은 역할).
        {
            let n_shapes = ctx.inst.shapes.len();
            // ① 테이블 확장 — 초기 기준값은 «배치된 조각의 최대 x» (`+0x40 + 0xc` 대응).
            let snapshot = ctx
                .placed
                .iter()
                .map(|p| p.pos.x + ctx.coll_box[p.shape].x1)
                .max()
                .unwrap_or(0);
            ctx.part_offsets.ensure(n_shapes, snapshot);

            // ② 좌표 극값 누적 — AU135.3 (`FUN_100243c0` 후반부).
            //    좌표계 A 의 우측끝 → basis 의 `min`,
            //    좌표계 B 의 우측끝(= A + 형상 x오프셋) → offset 의 `max`.
            for p in &ctx.placed {
                let sh = &ctx.inst.shapes[p.shape];
                let a_right = p.pos.x + sh.bbox.x1;
                let b_right = a_right + sh.bbox.x0;
                ctx.part_offsets.accumulate(p.shape, a_right, b_right);
            }

            // ③ 목표 폭 (`+0x78`) 과 구간 전진 (`+0x164`) — AU134.1.
            // ★★ §12.1.43 AU146 — `FUN_10026420` 의 `+0x180 != 0` 분기 이식:
            //   «구간이 전진했으면 목표 폭 = 진행 경계 조각의 x» 로 **낮춘다**.
            //   회차가 돌수록 경계가 오른쪽으로 이동하므로 목표 폭이 단계적으로 좁아진다.
            //   `cfg.canonical_slice_thr` 가 0 이면 종전 동작(부품 오프셋 + 전체 폭)이다.
            let last_shape = ctx.placed.last().map(|p| p.shape).unwrap_or(0);
            // ★★ §12.1.43 AU146 — 원천은 `+0x184`(구간 크기)로 경계를 **일부러 좁게** 잡는다.
            //   `FUN_10024d40` 의 진입 조건(`200 < 조각수`)은 «에이전트가 스스로 구간을
            //   나눌 만큼 큰 문제» 를 뜻하지만, 기제 자체(구간 전진 → 목표 폭 하강)는
            //   조각 수와 무관하게 성립한다. 여기서는 **조각 목록을 회차에 따라 등분**해
            //   `+0x180`(구간 인덱스)을 만들고, 그 지점의 조각 x 를 목표 폭으로 삼는다.
            //
            //   `cfg.canonical_slice_thr == 0` 이면 종전 동작(전체 폭)으로 퇴화한다.
            let boundary_x: Option<i64> = {
                let n = ctx.placed.len();
                let slice = ctx.cfg.canonical_slice_thr;
                if slice == 0 || n == 0 {
                    None
                } else {
                    // `+0x17c`(구간 개수) = n / slice + 1, `+0x184`(구간 크기) = n / 개수 + 1
                    let nslices = n / slice.max(1) + 1;
                    let size = n / nslices.max(1) + 1;
                    // `+0x180` = 경계 / 구간크기, 경계 = `+0x158` (진행 경계)
                    let base = ctx.part_offsets.target_width(last_shape, ctx.used_w);
                    let cut = compact::advance_tail(&ctx.placed, &ctx.inst.shapes, 0, base);
                    let idx = (cut / size.max(1)).min(nslices - 1);
                    if idx == 0 {
                        None // `+0x180 == 0` → 종전 공식
                    } else {
                        // `+0x74 = +0xac[+0x158].x` — 경계 조각의 현재 x.
                        let at = (idx * size).min(n - 1);
                        Some(ctx.placed[at].pos.x)
                    }
                }
            };
            let target_w = ctx
                .part_offsets
                .target_width_at(last_shape, ctx.used_w, boundary_x);
            ctx.target_w = target_w;
            ctx.tail_end = compact::advance_tail(&ctx.placed, &ctx.inst.shapes, 0, target_w) as i32;

            // ④ 회차 종료 min 누적 — AU133.3.
            //    원천은 `0x368 != 1` 일 때만 돈다. 우리는 «결 방향 모드» 를
            //    `mode_flag` 로 두고 있으므로 그것으로 가른다.
            if !ctx.mode_flag {
                ctx.part_offsets.accumulate_last(ctx.used_w);
            }
        }

        // ★★★ §12.1.34 AU139.2 — «반복 계속 판정» (`FUN_1004c1a0` 완전 이식).
        //
        // 원천 안쪽 루프 (`chunk_003.c:20352~20358`):
        //
        // ```c
        // do {
        //     iVar3 = FUN_1004ad80();     // 교대 3단
        //     if (iVar3 == 0) break;
        //     iVar3 = FUN_1004c1a0();     // ★ «계속?» 판정
        // } while (iVar3 != 0);
        // ```
        //
        // ★★★ §12.1.76 AU179 — 판정은 이제 **회차 루프 안**에서 매 회차 부른다
        //   (위 `ctx.cont = compact::should_continue(...)`). 종전에는 여기서
        //   한 번만 계산해 통계로 남겼고, 그 때문에 «수렴해도 상한까지 계속 도는»
        //   동작이 나왔다. 원천 `do { FUN_1004ad80(); if (!FUN_1004c1a0()) break; }`
        //   와 같은 자리로 옮겼으므로 이 블록은 제거한다(중복).
        ctx.stats.canonical_rounds = ctx.round as usize;
    }

    ctx.stats.failed = states.iter().filter(|s| **s == State::Failed).count();
    ctx.stats.pairs = ctx.pairs.len();
    ctx.stats.restarts = restart_count(n);
    ctx.stats.elapsed_ms = start.elapsed().as_millis();
    *cache = ctx.cache;
    Outcome {
        placements: ctx.placed,
        via_global,
        states,
        stats: ctx.stats,
        timed_out,
        pairs: ctx.pairs,
        // ★★★ §12.1.83 AU202 — 원천 `FUN_1005f8e0` 의 `+0x158..+0x15c` = «방금 배치된 조각».
        //   `sweep` 은 시작점 `seed` 를 맨 앞에 놓으므로 그 조각이 곧 마지막 배치 조각이다.
        //   `FUN_10020a00(iVar2)` 가 «이 조각을 포함하는 쌍만» 고르는 데 쓴다.
        last_seed: seed,
        // ★ AU203 — 진행 경계 (원천 `+0x15c`). `sweep` 의 `ctx.tail_end` 를 그대로 낸다.
        tail_end: ctx.tail_end,
        // ★★★ §AU184 — `+0xe0` 대각선 사본. 원천은 조각 인덱스로 접근하므로
        //   `bits[k][k]` 를 그대로 뽑는다.
        state_bits: (0..inst.pieces.len()).map(|k| ctx.bits[k][k]).collect(),
        cache: NfpCache::default(),
    }
}

/// 배치 순서와 NFP 캐시를 밖에서 받는 판. Complete(§4)가 같은 루프를 피스 키 순서로 다시
/// 돌 때 쓴다 — 알고리즘은 같고 순서만 다르다.
///
/// `Order::SeedSweep` 이면 참조 구현(§12.1.33 AU27)의 다중 시작점을 돈다: 첫 조각 후보를
/// `seed_trials` 개 시도하고 «배치 수 많음 → 폭 좁음» 순으로 가장 좋은 결과를 채택한다.
///
/// 시작점을 고르는 방식이 `Adaptive` 에 따라 갈린다(§12.1.33 AU39):
/// - `false` — 면적 내림차순 앞 N 개 (고정 목록)
/// - `true` — **직전 배치가 만든 접촉 쌍에서** 다음 시작점을 뽑는다(원천 `FUN_1005f3f0`)
pub fn run_with(inst: &Instance, cfg: Config, given_order: Option<&[usize]>, mut cache: NfpCache) -> Outcome {
    let n = inst.pieces.len();
    let mut base: Vec<usize> = match given_order {
        Some(o) => o.to_vec(),
        None => (0..n).collect(),
    };
    if given_order.is_none() && matches!(cfg.order, Order::AreaDesc | Order::SeedSweep) {
        let area = |k: usize| inst.shapes[inst.pieces[k].shapes[0]].area_mm2;
        base.sort_by(|&a, &b| area(b).total_cmp(&area(a)));
    }
    // §12.1.34 AU66 — 원천 키는 `+0xac[i].x − +0x8c[i].x`(작업좌표 − 좌측상한).
    // 우리 입력에서는 좌측상한이 무제한(AU63)이므로 **키 ≈ 조각의 bbox 좌하단 x** 다.
    if given_order.is_none() && matches!(cfg.order, Order::LeftFree) {
        let left = |k: usize| inst.shapes[inst.pieces[k].shapes[0]].bbox.x0;
        base.sort_by_key(|&k| left(k));
    }
    // ★★★ §12.1.67 AU172 — 원천 `LAB_100191b0` 1차 키 이식.
    //
    //   원천 레코드(`0x14`바이트) = `{+0x00: 원래인덱스, +0x04: 조각포인터, +0x08: 형상+0x10}`.
    //   비교 함수 어셈블리(`10019272~10019283`) = `+0x08` **오름차순**, 동점이면 `+0x04`
    //   (조각 포인터 = 입력 순서), 그다음 `+0x00`.
    //
    //   형상 `+0x18`/`+0x1c` 가 «폭»/«높이» 임은 `FUN_10082a10`(치수 비교)과
    //   `FUN_10007890`/`FUN_10007930`(좌표 차)에서 확정됐다. 1차 키 `+0x10` 은 그보다
    //   앞선 스칼라(치수 계열, 미확정) — 여기서는 **폭(`bbox.x1`)으로 근사**한다.
    //   동점 처리는 «입력 순서 안정 정렬» 로 원천과 같다.
    if given_order.is_none() && matches!(cfg.order, Order::ShapeWidthAsc) {
        let a = |k: usize| inst.shapes[inst.pieces[k].shapes[0]].area_mm2;
        base.sort_by(|&x, &y| a(x).total_cmp(&a(y))); // 면적 «오름차순» (동점 = 입력 순서)
    }

    if cfg.order != Order::SeedSweep || given_order.is_some() {
        let mut out = sweep(inst, &cfg, base, &mut cache, None);
        // `sweep` 은 캐시를 `cache` 인자에 되돌려 준다 — 호출자(`pipeline`)가 이어 쓰도록
        // `Outcome.cache` 로 넘긴다(종전에는 언제나 빈 캐시였다).
        out.cache = cache;
        return out;
    }

    // 시작점 큐. `Adaptive` 면 초기에는 면적 1위 하나만 넣고, 배치가 만든 접촉 쌍으로 늘린다.
    //
    // ★★★ §12.1.79 AU188 — **재시작 이중 루프** (원천 `FUN_1004ac70`).
    //
    // ```c
    // +0x180 = -1;
    // while (상태 == 0) {                      // ── 재시작 루프
    //     +0x180++;
    //     if (+0x17c <= +0x180) break;         // ★ 재시작 상한
    //     FUN_10025770(); FUN_10025870();      //   큐·평가배열 리셋
    //     if (+0x158 < +0x15c) {
    //         FUN_10049f80();                  //   시간 예산 검사
    //         do { FUN_1004ad80(); } while (FUN_1004c1a0());   // 내부 sweep
    //     }
    // }
    // ```
    //
    // 종전 구현은 «내부 sweep 1회» 만 돌았다. `restart_loop` 이 켜지면 **재시작마다 큐를
    // 리셋**하고 다음 재시작으로 넘어간다 — 원천의 `while (상태 == 0)` 구조다.
    //
    // **재시작 횟수 `+0x17c` 는 런타임 계산이다** (`chunk_002.c:10413`):
    // 초기값 `조각수/200+1`(AU38)이 아니라, `+0x194`(부품 슬롯, stride 8)를 스캔해
    // `슬롯[0] < +0x58 && 슬롯[1] > 0` 인 항목 수를 센 값이다. `+0x58` = 전역 최대 폭
    // (`chunk_002.c:6525`)이므로 사실상 «부품 슬롯이 있는 항목 수» 다.
    // 여기서는 **부품(입력 item) 종류 수**로 근사한다 — `+0x194` 슬롯은 부품별이기 때문이다.
    let mut best: Option<Outcome> = None;

    // `Order::SeedSweep` 시도 횟수 — 시작점 큐의 기본 크기.
    let k = cfg.seed_trials;
    // 원천 `+0x17c` 초기값 (`au38.md` §0.1.2): 조각수 > 200 이면 `조각수/200 + 1`.
    fn draft_restart_count(np: usize) -> usize {
        if np > RESTART_SCALE {
            np / RESTART_SCALE + 1
        } else {
            1
        }
    }

    let n_restarts = if cfg.restart_loop {
        let parts = inst
            .pieces
            .iter()
            .map(|p| p.item_id)
            .collect::<HashSet<_>>()
            .len();
        parts.max(draft_restart_count(n))
    } else {
        1
    };

    for restart in 0..n_restarts {
        // ★ 원천 `FUN_10049f80` — 재시작 진입마다 시간 예산을 검사한다.
        if let Some(d) = cfg.deadline {
            if Instant::now() > d {
                break;
            }
        }
        // ★ 원천 `FUN_10025770(); FUN_10025870();` — 재시작마다 큐·시도를 **리셋**한다.
        //   종전에는 `tried` 를 회차 전체에서 공유해 두 번째 재시작이 즉시 끝났다.
        let mut queue: Vec<usize> = if cfg.adaptive_seeds {
            base.first().copied().into_iter().collect()
        } else {
            base.iter().copied().take(k).collect()
        };
        let mut tried: HashSet<usize> = queue.iter().copied().collect();
        // ★ 재시작 1회차는 종전과 동일하게 «면적 1위» 에서 시작한다. 2회차부터는
        //   시작 조각을 바꿔 **다른 구조**를 탐색한다 (원천은 `FUN_10025870` 이 모드별로
        //   시작점을 다시 세운다). 여기서는 면적 순위를 재시작 횟수만큼 밀어 대응한다.
        if restart > 0 && !cfg.adaptive_seeds {
            let off = restart * k;
            queue = base.iter().copied().skip(off).take(k).collect();
            tried = queue.iter().copied().collect();
        } else if restart > 0 {
            // 적응 모드: 재시작마다 «아직 시작점으로 안 쓴» 조각 중 면적 1위에서 출발한다.
            if let Some(&s) = base.iter().find(|p| !tried.contains(p)) {
                queue = vec![s];
                tried.insert(s);
            }
        }
        let mut cursor = 0usize;
        // ★ 원천 `FUN_1005f3f0` 은 `+0x22c < +0x3e0` 일 때만 목록에 **추가**하고,
        //   루프 자체는 `+0x1b0`(목록 원소 개수)까지 돈다 (§12.1.34 AU40/§12.1.33 AU39).
        //   즉 `+0x3e0`(20)은 «시도 횟수 상한» 이 아니라 «목록 크기 상한» 이다.
        //   `adaptive_uncapped` 는 그 해석을 그대로 구현한 모드다 — 목록은 `k` 에서 멈추지
        //   않고 접촉 쌍이 있는 한 늘어나며, 실제 시도는 목록 전체를 돈다.
        //
        //   ★ `restart_loop` 이 켜지면 `+0x3e0`(=`seed_list_cap`, 원천 20)이 **살아난다**:
        //     목록이 상한에 도달하면 원천은 `+0x16c = +0x174` 로 «배치 종료» 하고
        //     재시작 루프가 다음 회차로 넘어간다. 그게 20개 × 재시작의 이중 구조다.
        let cap = if cfg.seed_list_cap > 0 {
            cfg.seed_list_cap
        } else if cfg.adaptive_seeds && cfg.adaptive_uncapped {
            usize::MAX
        } else {
            k
        };
        let queue_cap = cap;
        // ★ AU204 철회 — 원천 `FUN_1005f4d0` 의 «시작점 목록 수명주기» 이식은 **기각**됐다.
        //
        //   실측: 75.436% / 75.152% (−0.406 / −0.690pp), **두 회차가 서로 달랐다**.
        //   AU202 의 결정성(2회 완전 동일)을 깨뜨렸다.
        //
        //   원인 ①: `queue.clear(); queue.extend(tried.iter().copied())` 에서
        //     `tried` 가 `HashSet` 이라 **순회 순서가 비결정적**이다. 원천 `+0x168` 은
        //     **배열**이라 삽입 순서가 보존된다.
        //   원인 ②: 원천 `FUN_1005f4d0` 는 **순회 종료 후 정리 단계**이고, 우리는 이미
        //     `FUN_1005f3f0` 확장으로 필요한 조각을 모으고 있다 — **중복 추가**가 된다.
        //
        //   ★ 교훈: `HashSet` 을 «순서 있는 목록» 으로 쓰면 비결정성이 생긴다.
        //     원천의 배열(`+0x168`)을 옮길 때는 `Vec` + 별도 `HashSet`(소속 검사)로
        //     나눠야 한다. 이 시도의 잔재는 남기지 않는다.
        let mut seed_round: usize = 0;
        while cursor < queue.len() && cursor < queue_cap {
            if let Some(d) = cfg.deadline {
                if Instant::now() > d {
                    break;
                }
            }
            let _ = &mut seed_round;
            let seed = queue[cursor];
            cursor += 1;
            // ★★★ §12.1.80 AU189 — **방향 왕복** (원천 `FUN_1002a930`).
            //
            // 원천은 시작점 1개를 배치할 때마다 방향을 뒤집는다:
            // ```c
            // +0x368 = 2;  FUN_1004ac70();   // 역방향 (LP 목적계수 미설정)
            // +0x368 = 1;  FUN_1004ac70();   // 정방향
            // ```
            // **왕복 단위는 «전체 스윕» 이 아니라 «시작점 1개»** 다 — 전체 스윕을
            // 단위로 하면 1회 왕복이 예산을 다 먹는다(실측: 정방향 100초 + 역방향 79초).
            //
            // `reverse_mode` 는 LP 목적계수를 설정하지 않아 **다른 배치**를 낸다.
            // `restart_loop`/`bidir_outer` 와 독립이며, 꺼져 있으면 종전과 동일하다.
            let mut out = if cfg.bidir_swap {
                let mut c = cfg.clone();
                c.reverse_mode = true;
                // ★★★ §12.1.81 AU190 — **회차 예산 분할** (원천 `FUN_1004ac70` 안쪽 루프).
                //
                //   원천은 «짧은 배치 회차 → LP → 다시 배치» 를 반복한다:
                //   ```c
                //   do {
                //       FUN_1004ad80();      // 한 조각 배치 (+0x7c++)
                //       FUN_1004c1a0();      // +0x7c < +0x394(20) 이면 계속
                //   } while (...);
                //   ```
                //   우리 `sweep` 1회는 `cfg.deadline` 까지 **전체 예산**을 쓰므로
                //   LP 차례가 오지 않는다(실측: `Draft` 149.5s → `Draft+LP` 반복 0).
                //   그래서 왕복 1회에 **남은 예산의 절반**만 주고, 다음 왕복이
                //   그 결과 위에서 LP 를 돌린다.
                let now = Instant::now();
                let budget = cfg
                    .deadline
                    .map(|d| d.saturating_duration_since(now))
                    .unwrap_or(cfg.time_limit);
                let half = budget / 2;
                let rev_deadline = now + half;
                c.deadline = Some(rev_deadline);
                let rev = sweep(inst, &c, base.clone(), &mut cache, Some(seed));
                // «좁다» 는 사용 폭(우측 끝)이다 — `max pos.x` 는 좌측 끝이라 폭이 아니다.
                let w_rev = compact::width_grid(inst, &rev.placements);
                let mut f = cfg.clone();
                f.deadline = Some(Instant::now() + half);
                let fwd = sweep(inst, &f, base.clone(), &mut cache, Some(seed));
                let w_fwd = compact::width_grid(inst, &fwd.placements);
                if w_rev < w_fwd { rev } else { fwd }
            } else {
                sweep(inst, &cfg, base.clone(), &mut cache, Some(seed))
            };
        // 채택 규칙: 배치 수 많음 → 사용 폭 좁음.
        let better = match &best {
            None => true,
            Some(b) => {
                if out.placements.len() != b.placements.len() {
                    out.placements.len() > b.placements.len()
                } else {
                    // 사용 폭(우측 끝) 기준 — `max pos.x` 는 좌측 끝이라 폭이 아니다.
                    let w = |o: &Outcome| compact::width_grid(inst, &o.placements);
                    w(&out) < w(b)
                }
            }
        };
        // 적응적 확장 (§12.1.33 AU39 + §12.1.59 AU166).
        //
        // ★ AU166 — 원천 `FUN_1005f3f0`(chunk_004.c:16570~16611) 완전 해독:
        // ```c
        // if (+0x22c < +0x3e0) {                       // ★ 쌍 목록 < 20
        //   for (i = 0; i < +0x22c; i++) {
        //     piVar3 = +0x228 + i*0x38;                // ★ 쌍 레코드(56B)
        //     for (k in {piVar3[0], piVar3[1]}) {
        //       bVar1 = +0xe0[k];
        //       if ((bVar1 & 0x80) == 0 &&             // ★ «이미 선택됨» 아님
        //           (((bVar1 & 4) && !(bVar1 & 0x40))  // ★ 이동상자 있고 «제외» 아님
        //            || (bVar1 & 0x10))) {             // ★ 또는 «우선»
        //         FUN_1008c990(k);  +0xe0[k] |= 0x80;  // ★ 선택 표시
        //       }
        //     }
        //   }
        //   return;
        // }
        // +0x16c = +0x174;   // ★ 20개 도달 → 배치 종료
        // ```
        //
        // ★ `+0x80` = «시작점으로 선택됨», `+0x40` = «선택 제외», `+0x10` = «우선 선택».
        //   쌍 레코드 `*piVar4` = «미배치 조각», `piVar4[1]` = «배치완료 조각»
        //   (chunk_002.c:8677~8680, `&1` 비트로 정렬).
        if cfg.adaptive_seeds {
            // ★ `+0x3e0` = 20 (chunk_002.c:6605) — 쌍 목록 크기 상한.
            let pair_cap = if cfg.adaptive_uncapped { usize::MAX } else { 20 };
            if queue.len() >= pair_cap {
                // ★ 상한 도달 → «배치 종료»(원천 `+0x16c = +0x174`).
                //   우리는 루프를 빠져나가는 것으로 대응한다.
            } else {
                for &(_, i, j) in out.pairs.iter().filter(|&&(_, i, j)| {
                    // ★★★ §12.1.83 AU202 — 원천 `FUN_10020a00`: **방금 배치된 조각을
                    //   포함하는 쌍만** 후보로 삼는다 (+0.102pp, 재현 확인).
                    //   ```c
                    //   if (piVar3[0] == param_1 || piVar3[1] == param_1)
                    //   ```
                    out.last_seed.is_none_or(|s| i == s || j == s)
                    // ★ AU203 철회 — 원천 `FUN_10020b60` 의 `+0x15c` «재후보 유지» 조건을
                    //   `ctx.tail_end` 로 근사했으나 **실측 +0.000pp 무효**였다.
                    //
                    //   무효 원인: 원천 `+0x15c` 는 `FUN_1005ed90` 이 **시작점마다**
                    //   세우는 «이번 회차 상한»(`+0x158 = i; +0x15c = i+1`)인데,
                    //   우리 `tail_end` 는 **«목표 폭 안에 들어온 조각 수»** 라
                    //   의미가 다르다. 조건이 한 번도 성립하지 않았다.
                    //
                    //   정확 이식은 `FUN_1005f170` 의 시작점 순회 구조(회차별 구간)를
                    //   통째로 옮겨야 가능하다 — 아래 «미이식» 목록 참조.
                }) {
                    if queue.len() >= pair_cap {
                        break;
                    }
                    for c in [i, j] {
                        // ★★★ §AU184 — 원천 `FUN_1005f3f0` 의 자격 조건 (완전 이식):
                        // ```c
                        // bVar1 = +0xe0[k];
                        // if (bVar1 >= 0 && ((bVar1 & 4) && !(bVar1 & 0x40)) || (bVar1 & 0x10))
                        // ```
                        // 즉 «bit2(부착후보) && !bit6(소모)» 또는 «bit4(연결)».
                        // `+0x80`(이미 뽑힘)은 `tried` 가 대응.
                        let st = out.states.get(c).copied().unwrap_or(State::Pending);
                        let sb = out.state_bits.get(c).copied().unwrap_or(0);
                        let qualified = sb & BIT_SEEDED == 0
                            && ((sb & BIT_PAIR != 0 && sb & BIT_CONSUMED == 0) || sb & BIT_LINKED != 0);
                        if !qualified {
                            continue;
                        }
                        if st == State::Failed {
                            continue;
                        }
                        if tried.insert(c) {
                            queue.push(c);
                        }
                    }
                }
            }
        }
            // ★ AU205 철회 — 원천 `FUN_1005f8e0`+`FUN_1005f9c0` 의
            //   «미결 쌍 수렴 반복» 이식은 **기각**됐다 (±0.000pp, 3회 동일).
            //
            //   무효 원인은 **순서 논리**다. `FUN_1005f9c0` 을 그대로 옮기면:
            //   ```text
            //   1회차: s == 2(미결) → 전부 3(기각).  changed = true
            //   2회차: s == 3 → `if s != 1 continue`.  changed = false → 종료
            //   ```
            //   모든 쌍이 편입 즉시 미결(2)로 시작하므로 1회차에 전부 기각되고,
            //   `s == 1`(채택)인 쌍이 **하나도 없어서** 전파·부활 분기가
            //   **한 번도 실행되지 않는다**.
            //
            //   원천에서 동작하는 이유는 **`FUN_10020b60` 이 상태를 1 로 세팅**하기
            //   때문이다(어셈블리 `mov [eax+0x30], edi`, `edi = 1`):
            //   | 단계 | 원천 | 우리 |
            //   |---|---|---|
            //   | ① `FUN_10020a00` | 쌍 수집 | `pairs` 그대로 |
            //   | ② `FUN_10020a70` | 제약 레코드 갱신 | **미이식** |
            //   | ③ `FUN_10020b60` | **상태를 1(채택)로 세팅** | **미이식** |
            //   | ④ `FUN_1005f9c0` | 1인 쌍만 전파 | 1인 쌍 = 없음 |
            //
            //   ★ **③이 빠져서 ④가 공회전한다.** AU203 에서 «③이 선행되어야
            //     ④가 유효하다» 고 적은 그대로다 — 그 선행 조건을 충족하지 않고
            //     시도했으니 무효인 것이 정상이다.
            //
            //   ★ AU204(비결정성 파괴)와 달리 이번엔 **3회 완전 동일**이었다.
            //     비결정성이 아니면 «조건이 성립하지 않음» 을 의심하라 — AU203·205 공통.
            // ★ AU206 철회 — 원천 `FUN_1005f170` 순회 뼈대 «전체» 이식은
            //   **기각**됐다 (±0.000pp, 3회 완전 동일).
            //
            //   ★★ **무효 원인이 코드로 확정됐다 — `+0xe0` 영속성 부재.**
            //
            //   원천 `+0xe0`(조각 상태 비트)은 **배치 컨텍스트에 속한 배열**이라
            //   회차·시작점 순회를 넘어 살아남는다. `FUN_1005ee10` 이 세우는
            //   `bit6(0x40)` «소모됨» 표시가 그래서 다음 회차에도 유효하다.
            //
            //   우리 `Outcome.state_bits` 는 `sweep()` **끝에서** 생성되는
            //   **대각 사본**(`ctx.bits[k][k]`, 3033행)이다:
            //   ```rust
            //   state_bits: (0..inst.pieces.len()).map(|k| ctx.bits[k][k]).collect(),
            //   ```
            //   `run_with` 의 `while cursor` 루프는 매 바퀴 `sweep()` 을 **새로**
            //   부르고 `Ctx` 도 새로 만든다 → `state_bits` 가 **매번 초기화**된다.
            //   그래서 여기서 `BIT_CONSUMED` 를 세워도 **다음 `sweep` 에 전달되지
            //   않는다.** 읽고 쓰는 대상이 그 회차의 사본일 뿐이다.
            //
            //   ★ **이것이 AU203·204·205·206 «4연속 ±0.000pp» 의 공통 뿌리다.**
            //     AU205 진단에서 «소모 표시가 없어서 1인 쌍이 안 만들어진다» 고
            //     적었는데, 정확히는 **표시를 해도 전달되지 않는다**였다.
            //
            //   ★ 정확 이식의 선행 조건:
            //     ① `Ctx`(또는 `run_with` 스코프)에 **영속 조각 상태 배열**을 둔다
            //        — 원천 `+0xe0` 대응. `sweep` 이 그걸 **받아서** 쓴다.
            //     ② `FUN_1005f300`(최초 구축) + `FUN_1005f3f0`(확장)의 자격 조건을
            //        그 영속 배열에 대해 평가한다.
            //     ③ `FUN_1005ee10` 의 `bit6` 소모 표시를 같은 배열에 쓴다.
            //     → ①이 없으면 ②③는 전부 사본에 쓰는 헛일이 된다.
            //
            //   ★ AU204(비결정성 파괴)·AU205(순서 논리)와 달리 이번엔 **3회 완전
            //     동일**이었고, 원인은 «상태가 함수 경계를 못 넘는다» 는
            //     **구조 문제**였다. 값이 안 변할 때 의심할 세 번째 축이다:
            //     ① 비결정성(AU204) ② 조건 미성립(AU203·205) ③ **상태 미전달(AU206)**
        if better {
            best = Some(out);
        }
        }  // while cursor
    }  // for restart
    // 시작점이 하나도 안 돌았으면(빈 인스턴스, 또는 진입 전 마감) 기본 sweep 으로 떨어진다
    // — 패닉 대신 «시도 0» 의 정상 결과를 낸다.
    let mut out = match best {
        Some(b) => b,
        None => sweep(inst, &cfg, base.clone(), &mut cache, None),
    };
    out.stats.seed_trials = k.min(n);
    // `sweep` 이 되돌려 준 캐시를 호출자에게 넘긴다 (`pipeline` 의 LP·Complete 가 이어 쓴다).
    out.cache = cache;
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io;

    fn instance(json: &str, overlap: f64) -> Instance {
        // 시험이 병렬로 돌므로 파일 이름이 겹치지 않게 한다.
        static SEQ: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let seq = SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!("nesting_engine_test_{}_{seq}.json", std::process::id()));
        std::fs::write(&path, json).unwrap();
        let inst = io::load(&path, overlap).unwrap();
        let _ = std::fs::remove_file(&path);
        inst
    }

    const RECTS: &str = r#"{"name":"t","strip_height":10,"items":[
        {"id":0,"demand":2,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[4,0],[4,6],[0,6]]}},
        {"id":1,"demand":3,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[3,0],[3,3],[0,3]]}},
        {"id":2,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[6,0],[6,2],[3,5],[0,2]]}}
    ]}"#;

    #[test]
    fn places_everything_without_overlap_and_seeds_the_origin() {
        let inst = instance(RECTS, 0.0);
        let out = run(&inst, Config::default());
        assert_eq!(out.placements.len(), inst.pieces.len());
        assert_eq!(out.placements[0].pos, Pt::new(0, 0), "첫 조각은 원점 시드");
        let rep = crate::validate::certify(&inst, &out.placements);
        assert!(rep.overlaps.is_empty(), "{:?}", rep.overlaps);
        assert!(rep.overrun.iter().all(|&o| o <= 1e-9), "{:?}", rep.overrun);
    }

    #[test]
    fn each_piece_touches_its_attach_target() {
        let inst = instance(RECTS, 0.0);
        let out = run(&inst, Config::default());
        let mut cache = NfpCache::default();
        for (k, w) in out.placements.windows(2).enumerate() {
            if out.via_global[k + 1] {
                continue; // 전역 워크리스트 후보는 부착 조각에 닿지 않을 수 있다.
            }
            let (a, p) = (w[0], w[1]);
            let n = cache
                .get_or_compute((a.shape, p.shape), || nfp(&inst.shapes[a.shape].coll, &inst.shapes[p.shape].coll, 0))
                .unwrap();
            assert_eq!(n.locate(p.pos - a.pos), Loc::Boundary, "조각은 부착 대상(직전 조각)에 닿아야 한다");
        }
    }

    #[test]
    fn overlap_allowance_is_respected_on_the_original_outlines() {
        let inst = instance(RECTS, 0.8);
        let out = run(&inst, Config::default());
        let rep = crate::validate::certify(&inst, &out.placements);
        assert_eq!(rep.placed, rep.total);
        assert!(rep.overlaps.is_empty(), "{:?}", rep.overlaps);
    }

    /// 부착 조각이 묻히면 이후 조각이 연쇄 보류된다. 6×4 위 왼쪽 끝에 2×2 가 올라가면,
    /// 높이 6 인 원단에서 6×6 은 `y = 0` 에만 놓일 수 있는데 2×2 에 닿는 그 자리는 6×4 와
    /// 겹친다. 체인 규칙만 쓰면 6×6 두 개가 모두 실패하고(2차 패스에서도 부착 대상이 그대로
    /// 2×2 다), 전역 워크리스트면 첫 6×6 이 6×4 오른쪽(6×4 와 바닥에 동시에 닿는 자리)에
    /// 놓이고 둘째는 그것에 부착된다.
    #[test]
    fn a_buried_attach_target_does_not_cascade_into_holds() {
        const SEALED: &str = r#"{"name":"t","strip_height":6,"items":[
            {"id":0,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[6,0],[6,4],[0,4]]}},
            {"id":1,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[2,0],[2,2],[0,2]]}},
            {"id":2,"demand":2,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[6,0],[6,6],[0,6]]}}
        ]}"#;
        let inst = instance(SEALED, 0.0);
        // 이 시험은 «체인 규칙» 을 검증하므로 선언 순서를 명시한다. §12.1.27 AO5 로
        // 기본 순서가 AreaDesc 로 바뀌어, 기본값에 의존하면 배치 순서가 달라진다.
        let decl = Config { order: Order::Declaration, ..Config::default() };
        let off = run(&inst, Config { global: Global::Off, ..decl.clone() });
        assert_eq!(off.placements.len(), 2, "체인 규칙만 쓰면 6×6 두 개가 연쇄 실패한다");
        assert_eq!(off.stats.failed, 2);
        let on = run(&inst, Config { global: Global::Fallback, ..decl.clone() });
        assert_eq!(on.placements.len(), 4);
        assert_eq!(on.placements[1].pos, Pt::new(0, 400), "2×2 는 6×4 위 왼쪽 끝");
        assert_eq!(on.placements[2].pos, Pt::new(600, 0));
        assert_eq!(on.stats.global_placed, 1, "첫 6×6 만 전역 후보, 둘째는 그것에 부착");
        let rep = crate::validate::certify(&inst, &on.placements);
        assert!(rep.overlaps.is_empty(), "{:?}", rep.overlaps);

        let always = run(&inst, decl);
        assert_eq!(always.placements.len(), 4);
        assert_eq!(always.placements[2].pos, Pt::new(600, 0));
    }

    /// 재작성(`canonical`) 경로도 바퀴가 끝난 뒤 남은 조각을 2차 패스로 넘겨야 한다.
    /// 종전에는 `pending` 이 `held` 로 옮겨지지 않아 미배치 조각이 `Held` 로 남고
    /// `failed` 에 세지도 않았다 — 같은 SEALED 인스턴스에서 체인 경로와 같은 결론
    /// (6×6 두 개 실패) 이 나와야 한다.
    #[test]
    fn canonical_path_retries_pending_pieces_in_the_second_pass() {
        const SEALED: &str = r#"{"name":"t","strip_height":6,"items":[
            {"id":0,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[6,0],[6,4],[0,4]]}},
            {"id":1,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[2,0],[2,2],[0,2]]}},
            {"id":2,"demand":2,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[6,0],[6,6],[0,6]]}}
        ]}"#;
        let inst = instance(SEALED, 0.0);
        let cfg = Config { canonical: true, order: Order::Declaration, global: Global::Off, ..Config::default() };
        let out = run(&inst, cfg);
        assert!(out.states.iter().all(|s| *s != State::Held), "{:?}", out.states);
        assert_eq!(out.stats.failed, 2);
        assert_eq!(out.placements.len(), 2);
    }

    #[test]
    fn restart_count_follows_section_0_1_2() {
        assert_eq!(restart_count(156), 1);
        assert_eq!(restart_count(200), 1);
        assert_eq!(restart_count(201), 2);
        assert_eq!(restart_count(450), 3);
    }
}
