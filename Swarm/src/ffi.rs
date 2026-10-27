// -- swarm 라이브러리를 위한 C 호환 FFI(Foreign Function Interface)

// 이 모듈의 export 함수는 모두 C에서 호출되는 경계 shim이며, 진입 시 포인터 null 검사를 한다.
// 시그니처를 `unsafe fn`으로 바꾸면 C ABI는 그대로지만 Rust 쪽 호출부(테스트 등)만 번거로워지므로 허용한다.
#![allow(clippy::not_unsafe_ptr_arg_deref)]

use std::ffi::CString;
use std::os::raw::c_char;
use std::time::Duration;

use crate::config::{
    CompressionConfig, DEFAULT_SWARM_CONFIG, ExplorationConfig, HoleFillConfig,
    LpCompactionConfig, PolishConfig, RasterNfpConfig, ShrinkAdaptMode, ShrinkDecayStrategy,
    SlideConfig, normalize_shrink_range,
};

/// A single swap attempt performs several O(n) collision-tracker updates. Keep
/// malformed/untrusted ABI input from turning one hook into an effectively
/// unbounded call; normal tuning values are typically single digits.
const MAX_SWAP_ATTEMPTS_PER_PASS: usize = 1_024;

/// Each separator worker owns its own collision-tracker/solution copy and a
/// thread. Keep malformed ABI input from requesting thousands of them.
const MAX_SEPARATOR_WORKERS: usize = 256;

/// `Duration::MAX` panics in `Instant + Duration`, so "unlimited" time limits
/// are clamped to a large finite value (~136 years) instead.
const MAX_FINITE_DURATION: Duration = Duration::from_secs(u32::MAX as u64);

/// 분리기 설정 구조체
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CSeparatorConfig {
    pub iter_no_imprv_limit: i32,
    pub strike_limit: i32,
    pub n_workers: i32,
    pub n_container_samples: i32,
    pub n_focussed_samples: i32,
    pub n_coord_descents: i32,
    // ── capmods 전용 (capmods feature 없이 빌드된 라이브러리에서는 무시됨) ──
    /// 스왑(교환) 이웃 연산 주기 (내부 루프 iteration 단위). 0 = off, 음수 = 기본값(20)
    pub swap_period: i32,
    /// 스왑 패스 1회당 시도 수. 0 이하 = 기본값(8), 최대 1024
    pub swap_attempts: i32,
    /// 스왑 파트너 면적비 하한. 0 이하 = 기본값(0.4)
    pub swap_area_ratio_min: f32,
    /// 스왑 파트너 면적비 상한. min보다 작으면 기본값(2.5)
    pub swap_area_ratio_max: f32,
    /// 컨테이너 전역 샘플의 좌측 편향 최대 강도. 0 = 균등 분포, 음수 = 기본값(2.0)
    pub left_bias_gain: f32,
}

/// 탐색 단계 설정 구조체
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CExplorationConfig {
    pub shrink_step: f32,
    pub time_limit_secs: f32,
    pub max_conseq_failed_attempts: i32, // <= 0 = None
    pub solution_pool_distribution_stddev: f32,
    pub large_item_ch_area_cutoff_percentile: f32,
    pub separator: CSeparatorConfig,
    // ── adaptive 전용 (adaptive feature 없이 빌드된 라이브러리에서는 무시됨) ──
    /// 파괴-재생성(ruin & recreate) 주기 (disruption 횟수 단위). 0 = off, 음수 = 기본값(3)
    pub rnr_period: i32,
    /// 파괴할 세로 밴드 폭 (strip 폭 대비 비율, 0~1). 0 이하 = 기본값(0.18)
    pub rnr_band_fraction: f32,
    /// 한 번에 파괴할 아이템 수 상한. 0 이하 = 기본값(40)
    pub rnr_max_items: i32,
    /// 밀도 기반 적응형 shrink의 적응 강도. 0 = 고정 스텝, 음수 = 기본값(4.0)
    pub adaptive_shrink_gain: f32,
    /// 적응형 shrink의 최대 배율 상한. 1 미만 = 기본값(5.0)
    pub adaptive_shrink_max_mult: f32,
    /// 폭 진동: 같은 폭 연속 실패 임계값. 0 = 비활성, 음수 = 기본값(4)
    pub osc_fail_threshold: i32,
    /// 폭 진동: 되돌릴 폭 (base shrink_step 배수). 0 이하 = 기본값(3.0)
    pub osc_expand_steps: f32,
    /// ALNS 가중치 지수 감쇠 계수 (0~1). 범위 밖 = 기본값(0.85)
    pub alns_decay: f32,
    /// ALNS 가중치 하한. 음수 = 기본값(0.1)
    pub alns_min_weight: f32,
}

/// 압축 단계 설정 구조체
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CCompressionConfig {
    /// Legacy ABI names: two positive finite values are accepted in either order and
    /// normalized internally to `(start = max, stop = min)`.
    pub shrink_range_min: f32,
    pub shrink_range_max: f32,
    pub time_limit_secs: f32,
    pub shrink_decay_strategy: i32, // 0 = TimeBased, 1 = FailureBased
    pub shrink_decay_ratio: f32,    // only used for FailureBased
    pub separator: CSeparatorConfig,
}

/// 후처리(polish) 단계 설정 구조체.
///
/// 활성 LP/slide 패스를 시간 예산 만료 또는 정지 요청까지 반복한다.
///   1) LP compaction (lp)  2) 좌측/하단 슬라이딩 (slide)
/// Legacy hole-fill fields are retained for ABI compatibility and have no effect.
///
/// 두 패스 모두 이동 전에 충돌 검증을 거치므로 입력 해보다 나빠지지 않는다.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CPolishConfig {
    pub enabled: i32, /* 0 = off, 그 외 = on */
    pub time_limit_secs: f32,
    pub n_rounds: i32, /* Legacy ABI field; LP/slide retry until time_limit_secs or stop. */

    pub slide_enabled: i32,
    pub slide_n_passes: i32,
    pub slide_down: i32,         /* 0이면 좌측 방향만 */
    pub slide_safety_ratio: f32, /* 접촉 시 남길 간격 (아이템 최소 변 대비) */

    pub hole_fill_enabled: i32,
    pub hole_fill_cell_size_ratio: f32, /* 격자 셀 크기 (최소 아이템 변 대비) */
    pub hole_fill_max_candidates: i32,
    pub hole_fill_candidate_band: f32, /* strip 우측 몇 %를 후보로 볼지 (0.0~1.0) */
    pub hole_fill_n_samples: i32,      /* 포켓당 샘플 수 */

    pub lp_enabled: i32,
    pub lp_n_iterations: i32,
    pub lp_trust_region_ratio: f32, /* 신뢰 영역 기본 단계 (strip 폭 대비). 막히면 2배씩 확대 */
    pub lp_left_pull_weight: f32,   /* 좌측으로 당기는 목적항 가중치 */
}

/// LBF 초기 배치의 래스터 NFP 후보 생성 설정.
///
/// 레이아웃과 아이템을 같은 격자로 래스터화해 겹치지 않는 격자 위치를 직접 열거한다.
/// 열거된 후보는 전부 기존 CDE로 검증되므로 격자 해상도와 무관하게 안전하다.
/// 기본값은 켜짐이며, capmods 없이 빌드된 라이브러리에서는 무시된다.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CRasterNfpConfig {
    /// 0 = off, 그 외 = on(기본값)
    pub enabled: i32,
    /// 셀 크기 = (가장 작은 아이템 최소 변) × 이 비율. 0 이하 = 기본값(0.12)
    pub cell_ratio: f32,
    /// 격자 셀 총 개수 상한. 0 이하 = 기본값(250000)
    pub max_cells: i32,
    /// CDE로 검증할 래스터 후보 수. 0 이하 = 기본값(64)
    pub n_candidates: i32,
    /// 래스터 후보와 병행할 난수 샘플 수. 0 = 래스터만, 음수 = 기본값(1000).
    /// 난수 샘플을 줄여 래스터로 대체하면 오히려 나빠지므로 보강용으로만 쓸 것.
    pub n_random_samples: i32,
}

/// Experimental P0-P5 structure-first pipeline configuration.
///
/// This is kept as one contiguous ABI block in [`CSwarmConfig`] so all legacy
/// fields retain their offsets. Callback-only options follow it at the parent
/// tail. Already-compiled callers using the older, shorter struct must still be
/// rebuilt. Callers should initialize the parent with
/// `swarm_get_default_config` before overriding selected values.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CStructureFirstConfig {
    /// 0 = legacy pipeline (default), non-zero = allow `swarm_gp_phase`.
    pub enabled: i32,
    /// P0 reversible motif discovery.  These are not public rigid PlaceGroups.
    pub motif_enabled: i32,
    /// Maximum number of mutually compatible motif hints used by one start.
    pub max_active_motifs: i32,
    /// Number of independent P1 construction starts.
    pub construction_starts: i32,
    /// P1 profile rows relative to the smallest item dimension.
    pub profile_row_height_ratio: f32,
    /// Number of top profile candidates eligible for stochastic selection.
    pub profile_top_k: i32,

    /// P2 short compression budget and shrink schedule.
    pub p2_time_limit_secs: f32,
    /// Legacy ABI names; positive finite values are order-independent and normalized
    /// internally to `(start = max, stop = min)`.
    pub p2_shrink_range_min: f32,
    pub p2_shrink_range_max: f32,
    pub p2_failure_decay: f32,

    /// P3 warm-start container slack (0.05 = five percent).
    pub p3_slack_ratio: f32,
    /// Negative = use the normal exploration budget; otherwise explicit P3 budget.
    pub p3_time_limit_secs: f32,

    /// P4 gap-target repair controls.
    pub p4_cycles: i32,
    pub p4_cycle_time_limit_secs: f32,
    pub p4_ruin_fraction: f32,
    pub p4_max_items: i32,
    pub p4_stagnation_cycles: i32,

    /// Non-zero = snap P5 placements back to the input's allowed rotations.
    pub p5_snap_rotations: i32,

    /// Appended structure-first V2 tuning fields. Existing field offsets above
    /// remain stable; callers must validate [`swarm_get_abi_info`] before
    /// passing the caller-allocated parent record.
    pub p1_time_limit_secs: f32,
    pub profile_max_rows: i32,
    pub item_beam_width: i32,
    pub candidate_pool_cap: i32,
    pub slide_refine_steps: i32,
    pub p1_include_legacy_baseline: i32,
    pub pair_max_offset_candidates: i32,
    pub pair_max_orientation_work: i32,
    /// Legacy ABI field; ignored since structure diagnostics were removed.
    pub diagnostics_max_raster_cells: i32,
    /// Legacy ABI field; ignored since structure diagnostics were removed.
    pub diagnostics_max_pair_comparisons: i32,
    /// Preview interval for every phase in milliseconds (default 2000); values below 500 are clamped to 500.
    pub progress_callback_interval_ms: i32,
}

/// Progress callback payload diagnostics.
///
/// These switches only control the additional diagnostics attached to callback
/// JSON. Solver collision handling and exact feasibility validation are never
/// disabled by this block.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CCallbackOptionsConfig {
    /// Non-zero = include the callback layout loss. Default: enabled.
    pub collect_loss: i32,
    /// Non-zero = include per-item collision status. Default: enabled.
    pub collect_collision_status: i32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct CallbackOptions {
    pub collect_loss: bool,
    pub collect_collision_status: bool,
}

pub(crate) const DEFAULT_CALLBACK_OPTIONS: CallbackOptions = CallbackOptions {
    collect_loss: true,
    collect_collision_status: true,
};

/// 전체 Swarm 설정 구조체
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CSwarmConfig {
    pub rng_seed: u64,                     // 0 = random, 1이상 = fixed seed
    pub poly_simpl_tolerance: f32,         // negative = None
    pub min_item_separation: f32,          // negative = None
    pub narrow_concavity_cutoff_dist: f32, // negative = None
    pub narrow_concavity_cutoff_area: f32, // negative = None
    pub lbf_x_multiplier: f32,             // LBF X-axis bias (default 10.0)
    pub lbf_y_multiplier: f32,             // LBF Y-axis bias (default 1.0)
    pub exploration: CExplorationConfig,
    pub compression: CCompressionConfig,
    pub polish: CPolishConfig,
    pub lbf_raster: CRasterNfpConfig,
    /// 0 = off(기본). 그 외 = 각 아이템의 허용 회전에 180° 반전을 추가한다.
    /// 결방향이 없는(non-nap) 원단 전용.
    pub allow_180_rotation: i32,
    /// discrete 회전 아이템에 허용하는 소각도 틸트 (도 단위, ±값). <=0 = off(기본).
    /// 결방향 등으로 정확한 각도가 필요한 원단에서는 0으로 둘 것.
    pub tilt_range_deg: f32,
    /// ABI-tail experimental configuration. Disabled by default.
    pub structure_first: CStructureFirstConfig,
    /// ABI-tail callback payload controls. Solver correctness is unaffected.
    pub callback_options: CCallbackOptionsConfig,
    /// ABI-tail N06 initial-placement engine settings. Disabled by default.
    pub initial_placement: CInitialPlacementConfig,
    /// ABI-tail exploration width-relief settings. Disabled by default.
    pub width_relief: CWidthReliefConfig,
    /// ABI-tail exploration shrink-step feedback. Disabled by default.
    pub shrink_feedback: CShrinkFeedbackConfig,
    /// ABI-tail 정체 감지 기반 적응형 국소 최적해 탈출 설정. 기본값은 비활성.
    pub escape: CEscapeConfig,
    /// ABI-tail NFP 기반 초기 배치(NFP-LBF) 시드 설정. 기본값은 비활성.
    pub nfp_lbf: CNfpLbfConfig,
    /// ABI-tail 원단 허용오차 설정. 기본값은 비활성.
    pub fabric_tolerance: CFabricToleranceConfig,
    /// ABI-tail 탐색 단계 NFP 활용 설정. 기본값은 전부 비활성.
    pub nfp_integration: CNfpIntegrationConfig,
    /// ABI-tail NFP-LBF 탐침 예산 튜닝. 기본값은 네이티브 기본치.
    pub nfp_lbf_tuning: CNfpLbfTuningConfig,
    /// ABI-tail NFP 초기 배치 전략 7종. 기본값은 전부 비활성.
    pub nfp_strategy: CNfpStrategyConfig,
    /// ABI-tail LP·QP 솔버 선택. 기본값은 기존 동작(microlp 단체법, 순수 LP).
    pub lp_solver: CLpSolverConfig,
    /// ABI-tail 초기 배치 전략 ⑧⑨ (ABI v11). 기본값은 둘 다 비활성.
    ///
    /// `CNfpStrategyConfig` 를 늘리지 않고 **새 블록**으로 붙인다 — 중간 블록을 늘리면
    /// 뒤따르는 `lp_solver` 오프셋이 밀려 기존 호스트가 조용히 깨진다.
    pub nfp_strategy2: CNfpStrategy2Config,
    /// ABI-tail 레퍼런스 설계 초기배치 엔진 (ABI v12). **실험**, 기본 비활성.
    pub refplace: CRefPlaceConfig,
    /// ABI-tail LBF 배치 순서 (ABI v13). 기본 0 = 현행 동작(큰 것부터).
    pub lbf_order: CLbfOrderConfig,
    /// ABI-tail 홀 우선 부착 (ABI v14). 기본 0 = 꺼짐(현행 동작).
    pub hole_first: CHoleFirstConfig,
    /// ABI-tail 잔여 예산 회수 (ABI v15). **기본 켜짐.**
    ///
    /// zeroed 버퍼를 넘기는 구 호스트는 `enabled = 0` 이 되어 꺼진 동작을 받는다 —
    /// 즉 이 블록을 모르는 호출자는 예전 그대로다. `swarm_get_default_config`
    /// 를 부르는 호출자만 켜진 기본값을 받는다.
    pub reclaim: CReclaimConfig,
    /// ABI v16 tail: bounded Explore stagnation repair; all algorithms opt-in.
    pub repair: CSeparationRepairConfig,
    /// ABI v17 tail — 구간 통째 재배열. 앞 블록 오프셋은 전부 그대로다.
    pub band_rearrange: CBandRearrangeConfig,
    /// ABI v17 tail — 원단 경계 여유. 앞 블록 오프셋은 전부 그대로다.
    pub fabric_edge: CFabricEdgeConfig,
    /// ABI v17 tail — GLS 가중치 학습률. 앞 블록 오프셋은 전부 그대로다.
    pub gls: CGlsWeightConfig,
    /// ABI v17 tail — 구성기 배치 규칙. 앞 블록 오프셋은 전부 그대로다.
    pub constructor: CConstructorConfig,
    /// ABI v18 tail — 탐색 정체 대응(재출발 창·R&R 원위치·탈출 창). 전부 0 = v17 동작.
    pub search: CSearchTuningConfig,
    /// ABI v19 tail — 원본 외곽선 겹침 초과 처리. 0 = 실현 불가능(기존 동작).
    /// 직전 꼬리 끝(948)과 구조체 끝(952) 사이의 정렬 여유에 들어가 크기는 그대로다.
    pub original_overlap: COriginalOverlapConfig,
    /// ABI v20 tail — place group 처리 방식. 0 = 다중 폴리곤 아이템(기본), 1 = 강체 이동(1.0.2 방식).
    /// 구조체 크기가 952 → 960 으로 늘어난다(8바이트 정렬).
    pub place_group: CPlaceGroupConfig,
}

/// V2 place groups(패턴 그룹) 처리 방식 (ABI v20 tail block).
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CPlaceGroupConfig {
    /// `0` = 다중 폴리곤 아이템(기본): 그룹을 멤버 외곽선을 합친 아이템 하나로 넘긴다.
    /// `1` = 강체 이동: 멤버를 별도 조각으로 두고 함께 옮긴다(기존 동작). 그 밖의 값은 `0`.
    pub mode: i32,
}

pub const PLACE_GROUP_MULTIPOLYGON: i32 = 0;
pub const PLACE_GROUP_RIGID: i32 = 1;

/// [`CPlaceGroupConfig`]의 C 기본값 — 다중 폴리곤 아이템.
pub(crate) fn default_place_group_c() -> CPlaceGroupConfig {
    CPlaceGroupConfig {
        mode: PLACE_GROUP_MULTIPOLYGON,
    }
}

/// 원본 외곽선 겹침이 `piece_overlap_mm` 허용값을 넘는 배치의 처리 (ABI v19 tail block).
///
/// 겹침을 허용하면 탐색은 조각을 허용값의 절반만큼 줄인 도형으로 하고, 결과를 원본
/// 외곽선으로 다시 검증한다. 날카로운 꼭짓점은 줄이기에서 더 깎이므로, 줄인 도형이
/// 겹치지 않아도 원본 외곽선은 허용값보다 깊이 겹칠 수 있다
/// (`[ORIGINAL:VALIDATE] original overlap exceeds configured piece_overlap_mm`).
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct COriginalOverlapConfig {
    /// `0` = 실현 불가능(기본): 그런 배치를 버린다. `1` = 실현 가능: 원본 외곽선 검증을
    /// 끄고 그대로 채택한다. 그 밖의 값은 `0` 으로 본다.
    pub excess_policy: i32,
}

pub const ORIGINAL_OVERLAP_INFEASIBLE: i32 = 0;
pub const ORIGINAL_OVERLAP_FEASIBLE: i32 = 1;

/// [`COriginalOverlapConfig`]의 C 기본값 — 실현 불가능(= v18 과 동일 동작).
pub(crate) fn default_original_overlap_c() -> COriginalOverlapConfig {
    COriginalOverlapConfig {
        excess_policy: ORIGINAL_OVERLAP_INFEASIBLE,
    }
}

/// 초기 구성기의 배치 규칙 (ABI v17 tail block).
///
/// `CRasterNfpConfig` 는 오프셋 244 의 **중간** 블록이라 넓히면 뒤 블록이 전부
/// 밀린다. 그래서 `hole_first` 와 같은 방식으로 꼬리에 따로 붙인다.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CConstructorConfig {
    /// `0` = 꺼짐(기본, 기존 좌·하단 규칙). 그 외 = **양쪽 모서리 앵커링**.
    ///
    /// LBF 손실의 y 항은 y 가 작을수록 좋아서 모든 조각이 아래부터 쌓인다. 실측
    /// (여성WSjk 156-패널 초기 배치): 하단 접촉 16개 대 상단 6개. 참조 네스터의
    /// 77% 초기 배치는 위·아래에 번갈아 붙여 프론티어를 둘 만든다. 켜면 y 항이
    /// "가까운 가로 모서리까지의 거리" 가 된다. 좌측 우선은 그대로다.
    ///
    /// 전 구간에 적용하면 가운데가 비어 오히려 나빠진다(실측 73.350% -> 72.696%).
    /// 그래서 값은 **앞쪽 몇 조각에 적용할지**를 뜻한다.
    pub double_sided_first_n: i32,
    /// 구성 **도중** 몇 조각마다 hole fill + LP 압축으로 다질지. `<=0` = 하지 않음(기본).
    ///
    /// 이 셋은 지금까지 Polish 에서만 돌았다. 그때는 배치가 굳어 있고 Explore 가 이미
    /// 그 배치를 전제로 폭을 정한 뒤다. 구성 도중에 돌리면 다음 조각이 다져진
    /// 프론티어를 보고 자리를 고른다.
    pub settle_every: i32,
    /// 구성 도중 다지기 한 번의 예산(ms). `<=0` = 기본값 200.
    pub settle_budget_ms: i32,
    /// 구성 직후 한 번만 다질 예산(ms). `<=0` = 하지 않음(기본).
    pub post_init_compaction_ms: i32,
    /// **구간 완성 폭(mm).** `<=0` = 끔(기본).
    ///
    /// 현재 세로 구간이 더 받아들이지 못할 때까지 프론티어를 오른쪽으로 밀지 않는다.
    /// 후보 생성이 조각마다 독립적이라 이것은 손실 항으로 표현할 수 없고, 배치 루프의
    /// 제어 흐름으로만 강제할 수 있다.
    pub band_width_mm: f32,
    /// NFP 구성형 배치에서 **가중합 점수를 1순위**로 쓸지. `0` = 끔(기본).
    ///
    /// 기본 선택은 `strip length -> x -> y -> 접촉` 의 사전식 순서다. x 가 연속값이라
    /// 동률이 거의 없으므로 접촉 항과 낭비 항은 사실상 아무것도 결정하지 못한다 —
    /// 가중치가 있어도 배치는 순수 좌·하단 규칙이다. 켜면 가중합이 먼저 보이고
    /// 사전식 순서가 동률 처리로 내려간다.
    pub nfp_score_first: i32,
    /// 낭비 면적 가중치 `w2`. 음수/비유한 = 기본값 0.35.
    pub nfp_w2_waste: f32,
    /// 접촉 길이 가중치 `w3`(부호는 내부에서 `-`). 음수/비유한 = 기본값 0.25.
    pub nfp_w3_contact: f32,
    /// 세로 위치 가중치 `w4`. 음수/비유한 = 기본값 0.10.
    pub nfp_w4_y: f32,
    /// **길이 동률 허용치(mm).** `<=0` = 끔(기본).
    ///
    /// strip 길이 차이가 이 값 이하인 후보들만 동률로 보고, 그 안에서 맞물림 품질
    /// (`w2·낭비 − w3·접촉 + w4·y`)로 고른다. 길이 차이가 더 크면 언제나 짧은 쪽이
    /// 이기므로, 접촉이 길이를 이겨 strip 이 밀려나는 일이 구조적으로 없다.
    pub nfp_length_tie_mm: f32,
    /// 구간 경계에서 목록 뒤쪽의 **작은** 조각까지 훑어 구간을 채울 때의 면적 상한
    /// (가장 큰 조각 대비 비율). `<=0` = 훑지 않음(기본).
    ///
    /// 제한 없이 훑으면 작은 조각이 앞 구간으로 끌려와 면적 내림차순이 깨지고 큰
    /// 조각이 늦게 배치돼 스트립이 밀린다(실측 73.903% -> 67.7%).
    pub band_fill_ratio: f32,
}

/// [`CConstructorConfig`]의 C 기본값 — 꺼짐(= v16 과 동일 동작).
pub(crate) fn default_constructor_c() -> CConstructorConfig {
    CConstructorConfig {
        double_sided_first_n: 0,
        settle_every: 0,
        settle_budget_ms: 0,
        post_init_compaction_ms: 0,
        band_width_mm: 0.0,
        nfp_score_first: 0,
        nfp_w2_waste: -1.0,
        nfp_w3_contact: -1.0,
        nfp_w4_y: -1.0,
        nfp_length_tie_mm: 0.0,
        band_fill_ratio: 0.0,
    }
}

/// 탐색 단계 정체 대응 노브 (ABI v18 tail block).
///
/// 전부 0 = "네이티브 기본값(= v17 동작)". 앞 블록 오프셋은 그대로다.
///
/// # 근거 (여성WSjk 156, 겹침 0.8mm, Explore 240초 로그, 2026-09-30)
///
/// 벽 근처 폭에서 분리 한 번이 17~58초를 쓰고, 그 안의 개선은 strike 롤백 직후에만
/// 몰려 있다(`i:27 -> i:418`, `i:1608 -> i:2000`). 인컴번트를 떠난 궤적은 돌아오지
/// 못하므로, 390라운드가 아니라 짧은 창마다 인컴번트로 재출발하게 한다.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CSearchTuningConfig {
    /// Explore 분리기의 정체 재출발 창(라운드). `<=0` = 꺼짐(기본).
    /// 한 strike 안에서 이 라운드 수 동안 개선이 없으면 인컴번트로 되돌아가
    /// 다른 난수 궤적으로 계속한다. strike 판정과 포기 시점은 바뀌지 않는다.
    pub sep_restart_window: i32,
    /// Explore 분리기 `iter_no_imprv_limit` 의 capmods 하한(`아이템 수 x 2.5`)을
    /// 이 값으로 덮어쓴다. `<=0` = 종전 동작.
    pub sep_iter_floor: i32,
    /// Compress 분리기의 정체 재출발 창. `<=0` = 꺼짐(기본).
    pub cmpr_restart_window: i32,
    /// `0` = 꺼짐(기본). 그 외 = R&R 재삽입에서 원위치를 후보로 포함한다
    /// (`RuinRecreateConfig::original_candidate`).
    pub rnr_original_candidate: i32,
    /// `0` = 종전(기본). 그 외 = 탈출 파이프라인의 정체 창을 `window_floor` **절대값**으로
    /// 쓴다(조각 수 x 3 하한을 끈다). 종전에는 156조각에서 창이 468 분리 호출이라
    /// 예산 안에 한 번도 발동하지 않았다.
    pub escape_window_absolute: i32,
    /// Explore 루프의 분리 1회 벽시계 상한(ms). `0` = 제한 없음(기본).
    /// 상한이 오면 그 폭을 «못 푼다» 로 보고 실패 분기(풀·교란·진동)로 넘긴다.
    ///
    /// `-1`([`SEP_TIME_CAP_ADAPTIVE`]) = **적응형 상한**: 이 실행에서 가장 오래 걸린 성공
    /// 분리의 1.25배를 상한으로 쓴다(`ExplorationConfig::sep_cap_adaptive`). 그 밖의 음수는
    /// 제한 없음.
    pub sep_time_cap_ms: i32,
    /// 분리 실패 뒤 탐침할 교란 후보 수. `<=1` = 종전(후보 1개, 탐침 없음).
    pub portfolio_k: i32,
    /// 교란 후보 하나의 탐침 예산(ms). `<=0` = 기본 2000.
    pub probe_budget_ms: i32,
}

/// [`CSearchTuningConfig::sep_time_cap_ms`] 의 «적응형 상한» 표식.
pub(crate) const SEP_TIME_CAP_ADAPTIVE: i32 = -1;

/// [`CSearchTuningConfig`]의 C 기본값 — 전부 꺼짐(= v17 과 동일 동작).
pub(crate) fn default_search_tuning_c() -> CSearchTuningConfig {
    CSearchTuningConfig {
        sep_restart_window: 0,
        sep_iter_floor: 0,
        cmpr_restart_window: 0,
        rnr_original_candidate: 0,
        escape_window_absolute: 0,
        sep_time_cap_ms: 0,
        portfolio_k: 0,
        probe_budget_ms: 0,
    }
}

/// GLS(Guided Local Search) 쌍 가중치의 학습률과 상한 (ABI v17 tail block).
///
/// # 왜 노출하나
///
/// 기존 상수(증가 1.2~2.0, 상한 1e6, 감쇠 0.95)에서는 충돌이 지속되는 쌍의 가중치가
/// **20~76 라운드 만에 상한에 닿는다**. 벽에서의 separation 한 번은 천 라운드를
/// 넘기므로, 호출의 대부분 구간에서 어려운 쌍은 전부 정확히 1e6 이고 한 번도 충돌한
/// 적 없는 쌍은 1.0 이다. 그러면 가중 손실이 사실상 `1e6 × 비가중 손실` 이라 GLS 의
/// 변별력이 사라지고, **새 충돌을 만드는 후보가 기존 충돌을 유지하는 것보다 1e6 배
/// 싸진다** — 갈등을 푸는 대신 새 영역으로 밀어내도록 유도된다.
///
/// 전부 `<= 0` 이면 네이티브 기본값(= 기존 상수)이므로 v16 호출자는 동작이 같다.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CGlsWeightConfig {
    /// 충돌 중인 쌍의 최소 증가율. `<= 1` = 기본값 1.2.
    pub min_inc: f32,
    /// 충돌 중인 쌍의 최대 증가율. `< min_inc` = 기본값 2.0.
    pub max_inc: f32,
    /// 충돌하지 않는 쌍의 감쇠율. `<= 0` 또는 `> 1` = 기본값 0.95.
    pub decay: f32,
    /// 가중치 상한. `<= 1` = 기본값 1e6.
    pub max_weight: f32,
}

/// [`CGlsWeightConfig`]의 C 기본값 — 전부 0(= 네이티브 상수 사용).
pub(crate) fn default_gls_weight_c() -> CGlsWeightConfig {
    CGlsWeightConfig {
        min_inc: 0.0,
        max_inc: 0.0,
        decay: 0.0,
        max_weight: 0.0,
    }
}

impl From<CGlsWeightConfig> for crate::optimizer::separator::GlsWeightConfig {
    fn from(c: CGlsWeightConfig) -> Self {
        // `sanitized` 가 범위 밖 값을 기본값으로 되돌리므로, 0 으로 채운 호출자는
        // 네 항목 모두 네이티브 상수를 받는다.
        Self {
            min_inc: c.min_inc,
            max_inc: c.max_inc,
            decay: c.decay,
            max_weight: c.max_weight,
        }
        .sanitized()
    }
}

/// 원본 외곽선이 원단 밖으로 나가는 양을 제한한다 (ABI v17 tail block).
///
/// # 왜 필요한가
///
/// 겹침 `d` 를 허용하면 각 조각의 충돌 도형이 `d/2` 만큼 침식되고, 컨테이너도 같은
/// 양만큼 줄어든다(`swarm-core/src/util/io.rs`). 그래도 **원본** 외곽선은 밖으로
/// 나간다 — 침식은 변을 `r` 만큼 들이지만 볼록 꼭짓점은 `r/sin(θ/2)` 만큼 들이기
/// 때문이다. 침식된 도형이 벽에 닿는 순간 원본 꼭짓점은 `r·(1/sin(θ/2) − 1)` 만큼
/// 벽을 넘고, 이 값은 각이 날카로울수록 한없이 커진다. 원본 외곽선을 경계와 대조하는
/// 검사는 이 코드베이스 어디에도 없다.
///
/// 실측(여성WSjk 156-패널, 겹침 0.8mm): 최소 꼭짓각 31.7°, 이론상 최대 1.065mm,
/// 산출물에서 실제 0.352mm 관측.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CFabricEdgeConfig {
    /// 원본 외곽선이 원단 밖으로 넘어도 되는 최대 거리(mm). `<= 0` 또는 비유한
    /// = 제한 없음(기본, v16 과 동일 동작).
    ///
    /// 값을 주면 그 초과량을 흡수할 만큼 컨테이너를 **더** 줄인다. 대가는 원단
    /// 폭이고, 1524mm 기준 여유 1mm 당 약 0.066% 의 밀도다.
    pub margin_mm: f32,
}

/// [`CFabricEdgeConfig`]의 C 기본값 — 제한 없음(= v16 과 동일 동작).
pub(crate) fn default_fabric_edge_c() -> CFabricEdgeConfig {
    CFabricEdgeConfig { margin_mm: 0.0 }
}

/// 구간(밴드)을 통째로 다시 배열하는 연산자 (ABI v17 tail block).
///
/// # 왜 별도 블록인가
///
/// 기존 R&R 은 ALNS disruption 경로에 있고, 그 경로는 **분리 실패** 뒤에만 돈다.
/// 여성WSjk 156-패널 실측(Explore 480초, 3시드)에서 분리는 13~14회 성공하고 4회만
/// 실패하므로, R&R 전체가 한 번의 실행에서 1~6회밖에 돌지 않는다. 그 안에 무엇을
/// 넣든 결과에 닿지 않는다는 뜻이다. 이 블록은 같은 연산자를 **축소 직후**에도
/// 걸 수 있게 해서 실행 횟수를 성공 횟수만큼(13~14회) 끌어올린다.
///
/// 전부 기본 꺼짐/0 이며, 0 은 언제나 "네이티브 기본값을 쓴다"를 뜻한다. 따라서
/// 이 블록을 0 으로 채운 호출자의 동작은 v16 과 한 비트도 다르지 않다.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CBandRearrangeConfig {
    /// `0` = 꺼짐(기본). 그 외 = 축소에 성공할 때마다 우측 밴드를 통째로 다시 배열한다.
    pub post_shrink: i32,
    /// `0` = 꺼짐(기본). 그 외 = 밴드 안 이동 가능 조각을 **전부** 파괴한다
    /// (`rnr_max_items` 상한과 무작위 truncate 를 건너뛴다).
    pub full_band: i32,
    /// 재삽입 LBF 평가기의 x 편향. `<=0` = 기본값 10.0.
    pub recreate_x_mult: f32,
    /// 재삽입 LBF 평가기의 y 편향. `<=0` = 기본값 1.0.
    ///
    /// 둘 다 양수일 때만 반영된다. 기본값 `(10, 1)` 은 좌측 우선이라 재삽입이
    /// 언제나 같은 좌하단 규칙을 재현한다. 예를 들어 `(1, 10)` 은 아래부터 채운다.
    pub recreate_y_mult: f32,
    /// 재배열을 유지할 최소 "빈자리 착석률" (0~1). `<=0` = 검사 안 함(기본값).
    ///
    /// 충돌 없는 자리에 앉은 조각 비율이 이 값 미만이면 재배열 전체를 롤백한다.
    /// 배치가 성길 때는 통과하고 조밀해지면 저절로 꺼지는 자동 스위치다.
    pub min_clear_reinsert_ratio: f32,
    /// `0` = 꺼짐(기본). 그 외 = 밴드 시작점 후보에 **실제 빈틈(포켓) 위치**를
    /// 면적 내림차순으로 추가한다.
    ///
    /// 종전 후보는 8개 균등 간격뿐이라 1524mm 원단에서 간격이 약 190mm 였다.
    /// 조각 하나 크기의 빈틈은 두 후보 사이에 완전히 빠진다.
    pub pocket_band: i32,
    /// `pocket_band` 로 추가할 포켓 개수 상한. `<=0` = 기본값 24.
    pub pocket_band_max: i32,
    /// 빈틈 격자 셀 크기 배율. `<=0` = 1.0(종전 동작). 작을수록 촘촘해져
    /// 좁은 포켓이 잡힌다.
    pub pocket_grid_cell_scale: f32,
    /// 축소 직후 재배열의 loss 악화 허용 배율. `<=0` = 검사 안 함(기본값).
    /// 재배열 후 loss 가 `전 × 이 배율` 을 넘으면 롤백한다.
    pub max_loss_growth: f32,
    /// 축소 직후 재배열의 파괴 전략. 0=Rightmost(기본) 1=LowDensity
    /// 2=Fragmented 3=GapTarget 4=DensitySeam 5=Random.
    pub post_strategy: i32,
}

/// [`CBandRearrangeConfig`]의 C 기본값 — 전부 꺼짐(= v16 과 동일 동작).
pub(crate) fn default_band_rearrange_c() -> CBandRearrangeConfig {
    CBandRearrangeConfig {
        post_shrink: 0,
        full_band: 0,
        recreate_x_mult: 0.0,
        recreate_y_mult: 0.0,
        min_clear_reinsert_ratio: 0.0,
        pocket_band: 0,
        pocket_band_max: 0,
        pocket_grid_cell_scale: 0.0,
        max_loss_growth: 0.0,
        post_strategy: 0,
    }
}

/// Explore 정체 복구 (ABI v16 꼬리 블록). 불리언은 int32, 예산은 uint32 다. Explore 전용.
///
/// 각 항목의 의미는 `swarm_core::optimizer::separator::SeparationRepairConfig` 문서 참조.
/// 범위를 벗어난 값은 변환할 때 문서화된 범위로 잘린다.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CSeparationRepairConfig {
    /// 접촉 방향: 충돌 단위 하나를 접촉 후보 방향으로 옮겨 본다. 0 = 꺼짐(기본).
    pub direction_enabled: i32,
    /// 작은 집단 LP: 주변 단위들을 선형계획으로 동시에 옮긴다. 0 = 꺼짐(기본).
    pub cluster_lp_enabled: i32,
    /// 차단 연쇄: 새로 부딪히는 이웃까지 따라 옮긴다. 0 = 꺼짐(기본).
    pub chain_enabled: i32,
    /// 복구를 켜는 정체 반복 수. 1 ~ 10000, 기본 24.
    pub stagnation_iterations: u32,
    /// 분리 호출 한 번당 복구 패스 수 상한. 1 ~ 8, 기본 2.
    pub max_passes: u32,
    /// 이동 단위 수 상한(대상 수 · LP 집단 크기 · 연쇄 길이). 2 ~ 8, 기본 4.
    pub max_units: u32,
    /// 패스 하나의 후보 평가 수 상한. 1 ~ 256, 기본 64.
    pub max_evaluations: u32,
    /// 패스 하나의 벽시계 예산(ms). 1 ~ 200, 기본 20.
    pub time_budget_ms: u32,
    /// 단위당 최대 이동 거리(mm). 0 초과 100 이하, 기본 2. 겹침 허용량이 아니다.
    pub max_move_mm: f32,
}

impl From<CSeparationRepairConfig> for crate::optimizer::separator::SeparationRepairConfig {
    fn from(c: CSeparationRepairConfig) -> Self {
        Self {
            direction_enabled: c.direction_enabled != 0,
            cluster_lp_enabled: c.cluster_lp_enabled != 0,
            chain_enabled: c.chain_enabled != 0,
            stagnation_iterations: c.stagnation_iterations, max_passes: c.max_passes,
            max_units: c.max_units, max_evaluations: c.max_evaluations,
            time_budget_ms: c.time_budget_ms, max_move_mm: c.max_move_mm,
        }.sanitized()
    }
}

impl From<crate::optimizer::separator::SeparationRepairConfig> for CSeparationRepairConfig {
    fn from(c: crate::optimizer::separator::SeparationRepairConfig) -> Self {
        let c = c.sanitized();
        Self {
            direction_enabled: c.direction_enabled as i32,
            cluster_lp_enabled: c.cluster_lp_enabled as i32,
            chain_enabled: c.chain_enabled as i32,
            stagnation_iterations: c.stagnation_iterations, max_passes: c.max_passes,
            max_units: c.max_units, max_evaluations: c.max_evaluations,
            time_budget_ms: c.time_budget_ms, max_move_mm: c.max_move_mm,
        }
    }
}

/// 폴리시 잔여 예산을 압축 재진입으로 회수한다 (ABI v15 tail block).
///
/// # 무엇을 하나
///
/// 폴리시는 시계가 아니라 **고정점**에서 끝난다 — 실측(52조각 OPS2 마커)에서 60초
/// 예산 중 2.4~7.4초만 쓴다. 그 잔여는 지금 그냥 버려진다(단계마다 자기 timeout 만
/// 걸고 이월이 없다). 켜면 남은 시간 동안 압축↔폴리시를 반복해 더 좁은 폭을 노린다.
///
/// # 왜 폴리시를 더 돌리지 않고 압축으로 되돌리나
///
/// 정제 계열에 시간을 더 부어도 소득이 없다. 같은 압축 결과에서 출발한 실측:
///
/// ```text
/// 기본              2.4s ( 3.9%)  폭 3595.77
/// 라운드 100        8.6s (14.4%)  폭 3595.77   동일
/// LP 반복 400       8.8s (14.7%)  폭 3595.77   동일
/// NFP 홀채움 ON    60.0s (100 %)  폭 3595.77   동일 — 예산은 다 쓰지만 소득 없음
/// 신뢰영역 x4      60.0s (100 %)  폭 3599.09   나빠짐
/// ```
///
/// 압축은 다르다. 재진입하면 더 좁아진 폭에서 축소 스케줄을 새로 시작한다.
///
/// # 총 소요 시간은 늘지 않는다
///
/// 마감이 **폴리시가 이미 받은 예산의 끝**이다. 호출자가 준 적 없는 시간은 안 쓴다.
/// 폴리시가 예산을 다 쓰는 인스턴스(156조각 마커: 97~100%)에서는 잔여가 없어
/// 자동으로 no-op 이다.
///
/// # 기대치
///
/// 짝지은 within-run 실측 4회: -17.11 / +0.00 / +0.00 / -8.00 mm.
/// 적중 2/4, 평균 -6.28mm. **한 번도 나빠지지 않는다** — 폭이 실제로 줄었을 때만
/// 교체하기 때문이다.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CReclaimConfig {
    /// 0 = 꺼짐, 그 외 = 켬. `swarm_get_default_config` 는 **1** 을 채운다.
    pub enabled: i32,
    /// 한 사이클에서 압축이 받을 몫 (0.0~1.0). 0 이하면 기본값 0.75.
    pub compress_share: f32,
    /// 사이클 수 상한. 0 = 예산까지.
    ///
    /// **재현성이 필요하면 고정한다.** 사이클 수가 실측 잔여 시간의 함수라
    /// 기계 속도에 따라 결과가 달라진다.
    pub max_cycles: i32,
    /// 남은 시간이 이보다 적으면 시작하지 않는다(ms). 0 이하면 기본값 500.
    pub min_slice_ms: i32,
    /// 잔여를 무엇에 쓸지. **0 = 압축 재진입(기본)**, 1 = R&R 교란+재탐색,
    /// 2 = 적응형 탈출 탐색. 범위 밖은 0 으로 되돌린다.
    ///
    /// 0 은 **정제** 계열이고 1·2 는 **탈출** 계열이다. 폴리시가 멈춘 지점은
    /// 정제로는 못 벗어나는 고정점이라, 조합 자체를 바꾸는 탈출 계열이 더 크게
    /// 움직일 수 있다 — 대신 더 나빠질 수도 있다(폭 게이트가 막는다).
    pub mode: i32,
    /// 탈출 계열에서 한 사이클에 부술 밴드 폭 (strip 폭 대비). 0 이하면 기본값 0.20.
    pub ruin_band_fraction: f32,
    /// 탈출 계열에서 한 사이클에 부술 조각 수 상한. 0 이하면 기본값 48.
    pub ruin_max_items: i32,
    /// **탈출 계열에서** 개선 없는 사이클을 몇 번까지 참을지. 0 이하면 기본값 2.
    ///
    /// 압축 재진입은 결정론적이라 이 값과 무관하게 첫 실패에서 멈춘다.
    /// 탈출 계열은 확률적이라 한 번 못 찾았다고 수렴한 게 아니다 — 첫 실패에
    /// 멈추면 예산을 크게 남긴다(실측: R&R 이 60초 중 24초만 쓰고 36초를 남겼다).
    pub stagnation_cycles: i32,
}

/// 초기 배치의 **홀 우선 부착** (ABI v14 tail block).
///
/// # 무엇을 하나
///
/// 래스터 격자 전수 스캔은 이미 배치된 조각들에 둘러싸인 빈 주머니(포켓) 안의
/// 위치도 만들어 낸다. 그런데 기존 LBF 는 좌·하단 손실이 최소인 후보 하나만
/// 고르므로, x 가 큰 포켓 후보는 언제나 진다. 이 블록을 켜면 포켓 구간을 먼저
/// 따로 평가하고, CDE 를 통과하면 그것을 채택한다.
///
/// 폴리시 단계의 hole fill 이 사후에 같은 일을 하지만 그때는 배치가 굳어 있다.
///
/// # 실측 (LBF 단독, seed 0, share 0.5)
///
/// ```text
/// wsjk156   폭 -52.85mm (-0.73%)  밀도 +0.52%p   포켓 배치 18개
/// swim      폭  -6.54mm (-0.09%)  밀도 +0.06%p
/// shirts    폭  +0.55mm (+0.80%)  밀도 -0.62%p   <- 나빠진다
/// ```
///
/// 인스턴스에 따라 갈리므로 기본은 꺼짐이다. 조각이 크고 오목한 의류 마커에서
/// 켤 것.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CHoleFirstConfig {
    /// 0 = 꺼짐(기본, 현행 동작), 그 외 = 켬.
    pub enabled: i32,
    /// 포켓 후보에 내줄 후보 슬롯 비율 (0.0 ~ 1.0). 0 이하면 기본값(0.5).
    pub share: f32,
    /// 예약 — 구조체 크기를 16B 로 유지한다.
    pub _reserved0: i32,
    pub _reserved1: i32,
}

/// LBF 초기 배치의 순회 방향 (ABI v13 tail block).
///
/// # 무엇을 바꾸고 무엇을 안 바꾸나
///
/// LBF 는 조각을 `볼록껍질 면적 x 직경` 내림차순으로 넣는다. 이 블록은 그 **점수의
/// 정의는 그대로 두고 순회 방향만** 바꾼다. 그래야 A/B 에서 "순서 효과" 와
/// "크기 정의 효과" 가 섞이지 않는다.
///
/// 앞 블록들의 오프셋은 하나도 건드리지 않는다 — 새 블록으로 뒤에 붙였다.
/// 0 으로 채워 보내면(구 호스트) 현행 동작이므로, v12 호스트는 그대로 돈다.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CLbfOrderConfig {
    /// 0 = 큰 것부터(기본, 현행), 1 = 작은 것부터,
    /// 2 = 중간부터 큰 쪽 먼저, 3 = 중간부터 작은 쪽 먼저,
    /// 4 = 방향성 오목도부터(`LBF수정사항.md`), 5 = 크기 x (1 + 오목도).
    ///
    /// 범위 밖 값은 0 으로 되돌린다 — 경계에서 프로세스를 죽이지 않는다.
    pub mode: i32,
    /// 예약 — 구조체 크기를 16B(4의 배수)로 유지한다.
    pub _reserved0: i32,
    pub _reserved1: i32,
    pub _reserved2: i32,
}

/// 레퍼런스 설계 초기배치 엔진 스위치 (ABI v12 tail block).
///
/// # 실험이다
///
/// `레퍼런스-초기배치-설계.md` 를 그대로 구현한 별도 엔진이다. 구조는 완성됐고 겹침
/// 없는 유효한 배치를 내지만 **품질이 아직 기존 경로에 크게 못 미친다**
/// (wsjk156 기준 25.04% vs 기존 71.75%). 켜면 기존 초기 배치를 **대체**한다.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CRefPlaceConfig {
    /// 켜면 기존 초기 배치를 이 엔진으로 대체한다. 기본 0(꺼짐).
    pub enabled: i32,
    /// 그룹 정의가 없을 때 쓸 대체 규칙. 0 = 조각 하나가 한 그룹, 1 = item_id 별 묶음.
    pub group_fallback: i32,
    /// Mode in the existing ABI slot: 1 = Column Draft, 0 = legacy refplace.
    pub _reserved0: i32,
    /// Former reserved slot, same size/offset. Global importer setting,
    /// independent of refplace.enabled. Positive finite mm; 0 disables it.
    pub simplify_max_deviation_mm: f32,
}

/// NFP 초기 배치 전략 ⑧ 누적 배치 / ⑨ 꼬리 회수 (ABI v11 tail block).
///
/// 두 전략은 v10 에 없었다. 코어에는 구현돼 있었지만 설정 스위치도 호출부도 없어
/// 벤치에서만 돌았다 — 함정 1(모듈은 만들고 배선을 빠뜨림)의 사례였다.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CNfpStrategy2Config {
    /// (8) 순서를 정하지 않고 매 단계 가장 잘 맞물리는 조각을 고른다.
    ///
    /// 켜면 ordering 을 **1개만** 평가한다(순서를 쓰지 않으므로 더 돌아도 같은 결과).
    /// 실측(wsjk156, 탐침 8192): 71.75% -> 72.93%, +⑦ 슬라이딩 시 73.09%.
    pub accretion: i32,
    /// (9) 꼬리의 조각을 빼서 앞쪽 빈 곳으로 되돌린다.
    ///
    /// 실측(wsjk156, 5초 예산): 71.75% -> 73.27% (+1.52%p).
    pub tail_reflow: i32,
    /// (9) 꼬리 회수 라운드 수. `<=0` = 기본값(2).
    pub reflow_rounds: i32,
    /// 예약. 다음 노브가 생겨도 구조체 크기가 바뀌지 않도록 4의 배수로 맞춘다.
    pub _reserved: i32,
}

/// NFP 초기 배치 전략 7종 (ABI v10 tail block).
///
/// 전부 `0 = off`, 그 외 = on. 조절 노브는 `<=0` 이면 네이티브 기본값으로 떨어진다.
/// 자세한 설명은 `swarm_core::nfp_strategy::NfpStrategyConfig` 참조.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CNfpStrategyConfig {
    /// (1) 회전 후보를 `allow_180` 플래그가 아니라 인스턴스 선언에서 그대로 뽑는다.
    ///
    /// 실측(albano, `allowed_orientations = [0, 180]`): 끄면 LBF 대비 -1.51%p 로 지고,
    /// 켜면 +8.57%p 로 이긴다. 회전 선언이 `[0]` 뿐인 인스턴스는 영향이 없다.
    pub instance_rotations: i32,
    /// (1) 연속 회전을 몇 도 간격으로 샘플링할지. `<=0` = 기본값(90).
    pub continuous_rotation_step_deg: i32,
    /// (2) 잘 맞물리는 조각 쌍을 하나의 super-piece 로 묶어 배치한다.
    pub pairwise_clustering: i32,
    /// (2) 쌍을 채택할 최소 충전율 이득. `<=0` = 기본값(0.05).
    pub cluster_min_gain: f32,
    /// (3) 후보를 "가장 왼쪽" 이 아니라 "접촉 최대" 로 고른다.
    pub contact_maximization: i32,
    /// (4) 프론티어를 늘리기 전에 내부 구멍부터 채운다.
    pub pocket_first: i32,
    /// (5) 부분 배치를 1개가 아니라 `beam_width` 개 유지한다.
    pub beam_search: i32,
    /// (5) 빔 폭. `<=0` = 기본값(4). 시간이 이 값에 정비례한다.
    pub beam_width: i32,
    /// (6) 순서를 고정하지 않고 매 단계 가장 잘 맞는 조각을 고른다.
    pub best_fit_selection: i32,
    /// (6) 매 단계 평가할 조각 종류 수 상한. `<=0` = 기본값(12).
    pub best_fit_lookahead: i32,
    /// (7) 배치 후 각 조각을 막힐 때까지 좌하단으로 민다.
    pub sliding_compaction: i32,
    /// (7) 슬라이딩 라운드 수. `<=0` = 기본값(2).
    pub sliding_rounds: i32,
}

/// 압축 단계 LP·QP 솔버 선택 (ABI v10 tail block).
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CLpSolverConfig {
    /// `0` = microlp(단체법, 순수 Rust, 항상 사용 가능, LP 전용)
    /// `1` = clarabel(내점법, 순수 Rust, QP 지원)
    /// `2` = COIN-OR Clp (런타임에 `clp.dll` / `libClp.so` 를 찾는다, LP 전용)
    /// `3` = FICO Xpress (런타임에 `xprs.dll` / `libxprs.so`, LP + QP, 라이선스 필요)
    ///
    /// 범위 밖이면 `0`.
    pub backend: i32,
    /// 2차 정규화 계수 mu. `<=0` 이면 순수 LP.
    ///
    /// 순수 LP 는 목적이 폭 최소화 하나뿐이라 폭이 같은 해가 무수히 많고, 그중 아무거나
    /// 고르면 조각이 불필요하게 멀리 튄다. 2차항을 얹으면 같은 폭이면 덜 움직이는
    /// 해가 유일하게 뽑힌다. LP 전용 백엔드(0, 2)에서는 무시되고 경고가 남는다.
    pub quadratic_regularization: f32,
    /// 선택한 백엔드를 쓸 수 없을 때 microlp 로 물러날지. `0` 이면 압축을 건너뛴다.
    ///
    /// 상용 솔버 결과를 재현해야 하는 검증 실행에서는 조용한 폴백이 오히려 해로우므로
    /// 끌 수 있게 남긴다.
    pub fallback: i32,
    /// 한 번의 풀이에 허용할 시간(ms). `<=0` = 기본값(5000).
    pub time_limit_ms: i32,
}

/// [`CNfpStrategyConfig`]의 C 기본값 - 전부 꺼짐.
pub(crate) fn default_nfp_strategy_c() -> CNfpStrategyConfig {
    let def = swarm_core::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG;
    CNfpStrategyConfig {
        instance_rotations: 0,
        continuous_rotation_step_deg: def.continuous_rotation_step_deg as i32,
        pairwise_clustering: 0,
        cluster_min_gain: def.cluster_min_gain,
        contact_maximization: 0,
        pocket_first: 0,
        beam_search: 0,
        beam_width: def.beam_width as i32,
        best_fit_selection: 0,
        best_fit_lookahead: def.best_fit_lookahead as i32,
        sliding_compaction: 0,
        sliding_rounds: def.sliding_rounds as i32,
    }
}

/// [`CLpSolverConfig`]의 C 기본값 - 기존 동작 그대로.
pub(crate) fn default_lp_solver_c() -> CLpSolverConfig {
    let def = swarm_core::lp_solver::DEFAULT_LP_SOLVER_CONFIG;
    CLpSolverConfig {
        backend: 0,
        quadratic_regularization: def.quadratic_regularization,
        fallback: def.fallback as i32,
        time_limit_ms: def.time_limit_ms as i32,
    }
}

/// NFP-LBF 순차 배치의 탐침 예산 튜닝 (ABI v9 tail block).
///
/// [`CNfpLbfConfig`]를 늘리면 뒤따르는 tail block들의 오프셋이 전부 밀려 기존 호스트가
/// 깨진다. 그래서 새 노브는 구조체 끝에 블록으로 덧붙인다.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CNfpLbfTuningConfig {
    /// 아이템 하나당 feasible 판정을 시도할 raw 후보 수 상한. `<=0` = 기본값(4096).
    ///
    /// **초기 배치 품질을 좌우하는 값이다.** 후보 수는 배치가 진행될수록 폭증하는데
    /// (여성WSjk 156-패널: 20번째 1,607개 -> 155번째 13,514개) 예산이 고정이면 후기
    /// 아이템이 후보의 7.5%만 보고 자리를 정한다.
    ///
    /// 실측 (여성WSjk 156-패널, ordering 50개, **오목 단순화 0.02**,
    /// LBF = 7325.05 / 69.45% — 단, 이 LBF 기준선은 벤치가 좌측편향 계수를
    /// 프로덕션(10.0)이 아닌 1.0 으로 만든 값이라 실제보다 나쁘다. 프로덕션 계수로
    /// 재면 70.61% 이므로 아래 델타는 약 1.16%p 부풀려져 있다):
    ///
    /// ```text
    /// 1024 -> 7479.09 / 68.02%  (LBF에 진다 -> 시드 기각)
    /// 2048 -> 7277.71 / 69.90%
    /// 4096 -> 7082.27 / 71.83%  <- 기본값. 여기서 포화한다
    /// 8192 -> 7057.59 / 72.08%
    /// ```
    pub max_candidate_probes: i32,
    /// 예약. 반드시 0으로 채울 것.
    pub _reserved: [i32; 3],
}

/// 탐색(exploration) 단계의 NFP 활용 범위 (ABI v8 tail block).
///
/// 기존 Swarm 탐색은 "이 조각이 어디로 갈 수 있나"를 전부 **근사**로 답한다 —
/// 난수 샘플링, bbox MTV, 래스터 격자. NFP는 그 질문에 정확히 답하지만 질의가 더 비싸다.
///
/// 그래서 전부 기본 꺼짐이고, 항목마다 따로 켜서 A/B로 밀도와 시간을 재 봐야 한다.
/// 모든 필드는 `0 = off`, 그 외 = on 이다.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CNfpIntegrationConfig {
    /// 탐색 단계를 **순수 NFP 순서 탐색**으로 교체한다.
    ///
    /// 겹침을 한 번도 허용하지 않고 배치 순서를 탐색 공간으로 삼는다.
    /// `escape` 보다 우선한다. 연속 완화가 주는 마지막 몇 %를 잃을 수 있다.
    pub ordering_exploration: i32,
    /// ① separator의 위치 샘플러를 NFP 후보로 교체한다.
    pub nfp_sampler: i32,
    /// ② Conflict Burst의 분리 벡터를 정확한 NFP 기반으로 바꾼다 (기존은 bbox MTV 근사).
    pub exact_separation_vector: i32,
    /// ③ hole filling을 `feasible_region` 기반으로 바꾼다 (기존은 래스터 격자).
    pub nfp_hole_filling: i32,
    /// ④ Ruin & Recreate의 재삽입을 NFP 기반으로 바꾼다 (기존은 샘플링 + CDE).
    pub nfp_reinsertion: i32,
}

/// [`CNfpIntegrationConfig`]의 C 기본값 — 전부 꺼짐.
pub(crate) fn default_nfp_integration_c() -> CNfpIntegrationConfig {
    CNfpIntegrationConfig {
        ordering_exploration: 0,
        nfp_sampler: 0,
        exact_separation_vector: 0,
        nfp_hole_filling: 0,
        nfp_reinsertion: 0,
    }
}

/// [`CNfpLbfTuningConfig`]의 C 기본값 — 네이티브 기본 탐침 예산.
pub(crate) fn default_nfp_lbf_tuning_c() -> CNfpLbfTuningConfig {
    CNfpLbfTuningConfig {
        max_candidate_probes: swarm_core::nfp_lbf::DEFAULT_MAX_CANDIDATE_PROBES as i32,
        _reserved: [0; 3],
    }
}

/// 원단 허용오차 — 조각 사이 겹침/간격 (ABI v7 tail block).
///
/// `min_item_separation`은 음수가 곧 겹침 허용이지만, C ABI에서는 이 필드의 음수가
/// 이미 "설정 안 함(None)"을 뜻하도록 굳어져 있어 겹침에 **도달할 수 없다**.
/// 그래서 겹침만 별도 필드로 뺀다.
///
/// 두 값은 배타적이다. `piece_overlap_mm > 0`이면 그것이 이기고,
/// `min_item_separation`은 무시된다.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CFabricToleranceConfig {
    /// 조각 사이에 허용할 겹침(mm). `<= 0` 또는 비유한 = 비활성(기본).
    ///
    /// jagua-rs importer가 도형을 이 값의 절반씩 안쪽으로 줄이므로, CDE와 NFP가
    /// 같은 도형을 본다. 즉 겹침이 충돌 판정을 깨지 않고 실제 원단에서만 물린다.
    ///
    /// 실측(여성WSjk 156-패널): `0.4`에서 strip 길이 -2.9%, 밀도 +1.95%p.
    /// 너무 크게 주면(`1.0`) 도형이 과하게 줄어 오히려 손해다.
    pub piece_overlap_mm: f32,
}

/// [`CFabricToleranceConfig`]의 C 기본값 (`swarm_get_default_config`).
pub(crate) fn default_fabric_tolerance_c() -> CFabricToleranceConfig {
    CFabricToleranceConfig {
        piece_overlap_mm: 0.0,
    }
}

/// 정체 감지 기반 적응형 국소 최적해 탈출 (ABI v6 tail block).
///
/// 최근 `max(window_floor, window_item_factor × 패턴 수)` 회 동안 효율 개선이
/// `efficiency_eps` 미만이고 후보 배치의 다양성이 `diversity_threshold` 미만이면
/// 정체로 판정하고, 탐색 단계를
/// [`swarm_core::escape::escape_driven_exploration_phase`]로 교체해 탈출을 시도한다.
///
/// `enabled != 0`이어야 동작한다. 나머지 필드는 기존 tail block 관례를 따라
/// 0/음수/비유한 값이면 코어 기본값으로 대체된다.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CEscapeConfig {
    /// 0 = 비활성(기본값). 그 외 = 탈출 기반 탐색 단계를 사용한다.
    pub enabled: i32,
    /// 정체 관측 창의 하한. <=0 = 기본값(100).
    pub window_floor: i32,
    /// 정체 관측 창의 패턴 수 배수. <=0 = 기본값(3).
    pub window_item_factor: i32,
    /// 정체 판정 효율 개선 임계값. <=0 = 기본값(0.0001 = 0.01%p).
    pub efficiency_eps: f32,
    /// 후보 배치 다양성 임계값. <=0 = 기본값(0.05).
    pub diversity_threshold: f32,
    /// 0 = 비활성. 그 외 = Conflict Burst + Partial Rebuild 경로 사용(기본 켜짐).
    pub enable_conflict_burst: i32,
    /// 0 = 비활성. 그 외 = Hard 탈출에서 하위 후보해를 Elite 기준으로 재초기화(기본 켜짐).
    pub enable_swarm_diversification: i32,
    /// Conflict Burst Stage 1에서 강제 분리할 아이템 수 상한. <=0 = 기본값(16).
    pub max_push_items: i32,
    /// infeasible 해 풀의 크기 상한. <=0 = 기본값(32).
    pub max_infeasible_pool: i32,
}

/// NFP 기반 초기 배치 시드 (ABI v6 tail block).
///
/// ```text
/// NFP Cache -> Multiple Item Ordering -> NFP Sequential Placement
///           -> Best Initial Nest -> Swarm / Local Improvement
/// ```
///
/// `enabled != 0`이면 기존 LBF 생성기 대신 NFP-LBF가 만든 최선 초기해를
/// 탐색 단계의 seed로 넣는다. 나머지 필드는 기존 tail block 관례대로
/// 0/음수/비유한 값이면 코어 기본값으로 대체된다.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CNfpLbfConfig {
    /// 0 = 비활성(기본값, 기존 LBF 생성기 유지). 그 외 = NFP-LBF 시드 사용.
    pub enabled: i32,
    /// 평가할 ordering 수. <=0 = 기본값(50).
    pub n_orderings: i32,
    /// NFP 콜드 캐시 생성 예산(ms). <=0 = 기본값(5000).
    pub cache_budget_ms: i32,
    /// 초기 배치 탐색 전체 예산(ms). <=0 = 기본값(10000).
    pub total_budget_ms: i32,
    /// 0 = 0도 단일(기본값). 그 외 = 0/180도. 결방향 없는(non-nap) 원단 전용.
    pub allow_180: i32,
    /// 0 = 비활성. 그 외 = 앞부분이 같은 ordering의 배치를 재사용(기본 켜짐).
    pub enable_prefix_reuse: i32,
    /// NFP 라이브러리 전용 오목 단순화 강도(원본 면적 대비 허용 증가 비율).
    /// 음수/비유한 = 기본값(0.0 = 비활성). 결과 도형은 원본의 상위집합이라 안전하다.
    pub concave_simplify_tolerance: f32,
}

/// Default C values for [`CEscapeConfig`] (`swarm_get_default_config`).
pub(crate) fn default_escape_c() -> CEscapeConfig {
    let def = crate::config::DEFAULT_ADAPTIVE_ESCAPE_CONFIG;
    CEscapeConfig {
        // 코어에는 내부 기본값이 있지만 C/managed 경계에서는 opt-in으로 둔다.
        enabled: 0,
        window_floor: def.stagnation.window_floor as i32,
        window_item_factor: def.stagnation.window_item_factor as i32,
        efficiency_eps: def.stagnation.efficiency_eps,
        diversity_threshold: def.stagnation.diversity_threshold,
        enable_conflict_burst: def.enable_conflict_burst as i32,
        enable_swarm_diversification: def.enable_swarm_diversification as i32,
        max_push_items: def.conflict_burst.max_push_items as i32,
        max_infeasible_pool: def.max_infeasible_pool as i32,
    }
}

/// Default C values for [`CNfpLbfConfig`] (`swarm_get_default_config`).
pub(crate) fn default_nfp_lbf_c() -> CNfpLbfConfig {
    let def = crate::config::DEFAULT_NFP_LBF_SEED_CONFIG;
    CNfpLbfConfig {
        // 코어 기본값도 꺼짐이지만, 경계에서도 명시적으로 opt-in을 유지한다.
        enabled: 0,
        n_orderings: def.n_orderings as i32,
        cache_budget_ms: def.cache_budget_ms.min(i32::MAX as u64) as i32,
        total_budget_ms: def.total_budget_ms.min(i32::MAX as u64) as i32,
        allow_180: def.allow_180 as i32,
        enable_prefix_reuse: def.enable_prefix_reuse as i32,
        concave_simplify_tolerance: def.concave_simplify_tolerance,
    }
}

/// Closed-loop control of the exploration shrink step (ABI v5 tail block).
///
/// `adaptive_shrink_gain` scales the step by density alone, which is open loop:
/// density rises monotonically as the strip narrows, so the multiplier is a fixed
/// schedule that never learns that a step was too large. This block adds the same
/// feedback the compression phase already has (`ShrinkDecayStrategy::FailureBased`):
/// back the step off when a width cannot be separated, earn it back when a new
/// feasible width is reached.
///
/// Only the FIRST failure at a width counts, so repeated disruption retries at one
/// width cannot collapse the step to its floor.
///
/// `0 < fail_decay < 1` enables it. Adaptive-only.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CShrinkFeedbackConfig {
    /// Step-scale multiplier applied on the first separation failure at a width.
    /// Must be in (0, 1) to take effect; 1.0 (or out of range) = disabled (default).
    pub fail_decay: f32,
    /// Lower bound for the step scale. <=0 or >1 = default (0.05).
    pub scale_min: f32,
}

/// Default C values for [`CShrinkFeedbackConfig`] (`swarm_get_default_config`).
pub(crate) fn default_shrink_feedback_c() -> CShrinkFeedbackConfig {
    let def = DEFAULT_SWARM_CONFIG.expl_cfg;
    CShrinkFeedbackConfig {
        // The C/managed boundary keeps feedback opt-in even though the core
        // adaptive policy has an internal tuning default.
        fail_decay: 1.0,
        scale_min: def.shrink_scale_min,
    }
}

/// Width relief - escape a width the separator cannot solve by widening the
/// strip in small steps instead of retrying the same width forever.
///
/// When the exploration phase fails `fail_threshold` times in a row at one
/// width, it keeps the best colliding layout and re-runs separation at
/// `+step`, `+2*step`, ... of the current width (each expansion is inserted at
/// the collision hot spot) until the overlaps clear or `max_ratio` is reached.
/// Shrinking then resumes from the untangled layout.
///
/// Unlike `osc_fail_threshold`, this is NOT capped by the best feasible width:
/// the oscillation may only give back half of the last shrink step, while
/// relief may exceed the incumbent width. The incumbent solution itself is
/// still preserved, and a wider phase result is rejected at the phase boundary,
/// so the reported result can never get worse.
///
/// `fail_threshold > 0` enables relief and makes `osc_fail_threshold` inert.
/// Kept as one contiguous ABI-tail block in [`CSwarmConfig`].
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CWidthReliefConfig {
    /// Consecutive separation failures at one width before relief runs.
    /// 0 = disabled (default), which keeps the legacy width-oscillation path.
    pub fail_threshold: i32,
    /// Per-round expansion as a fraction of the current width. <=0 = default (0.002).
    pub step: f32,
    /// Cumulative expansion cap relative to the stuck width. <=0 = default (0.03).
    pub max_ratio: f32,
    /// Expansion rounds attempted per relief. <=0 = default (12).
    pub max_rounds: i32,
    /// Separator strike limit for one expansion round. <=0 = default (1).
    pub strike_limit: i32,
    /// Wall-clock budget for one expansion round, in milliseconds. <=0 = unlimited
    /// (default). The strike limit alone still allows one strike to burn the phase's
    /// whole `iter_no_imprv_limit`, which can eat a multi-second budget in a single
    /// relief; this caps each round instead.
    pub round_budget_ms: i32,
}

/// Default C values for [`CWidthReliefConfig`] (`swarm_get_default_config`).
pub(crate) fn default_width_relief_c() -> CWidthReliefConfig {
    let def = DEFAULT_SWARM_CONFIG.expl_cfg;
    CWidthReliefConfig {
        fail_threshold: def.relief_fail_threshold as i32,
        step: def.relief_step,
        max_ratio: def.relief_max_ratio,
        max_rounds: def.relief_max_rounds as i32,
        strike_limit: def.relief_strike_limit as i32,
        round_budget_ms: def.relief_round_budget.as_millis().min(i32::MAX as u128) as i32,
    }
}

/// N06 independent initial-placement engine (`swarm_init_phase`).
///
/// Pipeline: GRASP construction → slide recompaction → repeated
/// [width shrink → overlap separation (rescue / stall-based exit / fine CD) →
/// recompaction], with `kicks` re-expanding the shrink step until the time
/// budget is exhausted. Legacy (single-strip) inputs only.
///
/// Zero/negative fields fall back to the confirmed N06 defaults (see each
/// field). Kept as one contiguous ABI-tail block in [`CSwarmConfig`].
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CInitialPlacementConfig {
    /// 0 = disabled (default). Non-zero = allow `swarm_init_phase`.
    pub enabled: i32,
    /// Total time budget in seconds. <=0 = default (60).
    pub time_limit_secs: f32,
    /// GRASP construction starts (minimum-width one is kept). <=0 = default (1).
    pub grasp: i32,
    /// Kick (shrink-step re-expansion) retries. Negative = default (100000),
    /// 0 = no kicks.
    pub kicks: i32,
    /// Initial shrink step as a fraction of strip width. <=0 = default (0.005).
    pub step0_frac: f32,
    /// Minimum shrink step fraction. <=0 = default (0.0005).
    pub min_frac: f32,
    /// Separation iteration cap. <=0 = default (3000).
    pub iter_cap: i32,
    /// Separation sample base per move. <=0 = default (50).
    pub samples_base: i32,
    /// GLS pair-weight growth. <=1 = default (1.1).
    pub growth: f32,
    /// Rescue (exhaustive relocation) period in stalled iterations.
    /// Negative = default (300), 0 = off.
    pub rescue_after: i32,
    /// Stall-based termination: consecutive non-improving iteration cap.
    /// Negative = default (1500), 0 = plain iteration-cap exit.
    pub stall_cap: i32,
    /// Minimum coordinate-descent step in mm. <=0 = default (0.03).
    pub cd_min: f32,
    /// Slide-recompaction pass cap. Negative = default (6), 0 = off.
    pub compact_passes: i32,
    /// Reseat (rightmost piece relocation) count. Negative = default (4), 0 = off.
    pub reseat_top: i32,
    /// Construction candidate evaluation cap. <=0 = default (32).
    pub cand_eval_cap: i32,
    /// Construction bottom-left score X weight. <=0 = default (500).
    pub lbf_x_weight: f32,
    /// Construction bottom-left score Y weight. Negative = default (0.1).
    pub lbf_y_weight: f32,
}

/// Resolved (defaulted) N06 initial-placement settings carried by the step handle.
#[derive(Debug, Clone, Copy)]
pub(crate) struct InitialPlacementSettings {
    pub enabled: bool,
    pub time_limit_secs: f32,
    pub grasp: usize,
    pub kicks: usize,
    pub step0_frac: f32,
    pub min_frac: f32,
    pub iter_cap: usize,
    pub samples_base: usize,
    pub growth: f32,
    pub rescue_after: usize,
    pub stall_cap: usize,
    pub cd_min: f32,
    pub compact_passes: usize,
    pub reseat_top: usize,
    pub cand_eval_cap: usize,
    pub lbf_x_weight: f32,
    pub lbf_y_weight: f32,
}

impl Default for InitialPlacementSettings {
    fn default() -> Self {
        InitialPlacementSettings {
            enabled: false,
            time_limit_secs: 60.0,
            grasp: 1,
            kicks: 100_000,
            step0_frac: 0.005,
            min_frac: 0.0005,
            iter_cap: 3000,
            samples_base: 50,
            growth: 1.1,
            rescue_after: 300,
            stall_cap: 1500,
            cd_min: 0.03,
            compact_passes: 6,
            reseat_top: 4,
            cand_eval_cap: 32,
            lbf_x_weight: 500.0,
            lbf_y_weight: 0.1,
        }
    }
}

impl InitialPlacementSettings {
    /// C 입력을 기본값 보정 규칙(0/음수 = 기본)에 따라 해석한다.
    pub(crate) fn from_c(c: &CInitialPlacementConfig) -> Self {
        let d = InitialPlacementSettings::default();
        // Non-finite values (NaN, inf) fall back to the default too; an infinite
        // weight turns every candidate score into inf/NaN.
        let pos_f = |v: f32, d: f32| if v.is_finite() && v > 0.0 { v } else { d };
        let pos_i = |v: i32, d: usize| if v > 0 { v as usize } else { d };
        // 음수 = 기본, 0 = 명시적 off
        let off_i = |v: i32, d: usize| {
            if v < 0 {
                d
            } else {
                v as usize
            }
        };
        InitialPlacementSettings {
            enabled: c.enabled != 0,
            time_limit_secs: pos_f(c.time_limit_secs, d.time_limit_secs),
            grasp: pos_i(c.grasp, d.grasp),
            kicks: off_i(c.kicks, d.kicks),
            step0_frac: pos_f(c.step0_frac, d.step0_frac),
            min_frac: pos_f(c.min_frac, d.min_frac),
            iter_cap: pos_i(c.iter_cap, d.iter_cap),
            samples_base: pos_i(c.samples_base, d.samples_base),
            growth: if c.growth.is_finite() && c.growth > 1.0 { c.growth } else { d.growth },
            rescue_after: off_i(c.rescue_after, d.rescue_after),
            stall_cap: off_i(c.stall_cap, d.stall_cap),
            cd_min: pos_f(c.cd_min, d.cd_min),
            compact_passes: off_i(c.compact_passes, d.compact_passes),
            reseat_top: off_i(c.reseat_top, d.reseat_top),
            cand_eval_cap: pos_i(c.cand_eval_cap, d.cand_eval_cap),
            lbf_x_weight: pos_f(c.lbf_x_weight, d.lbf_x_weight),
            lbf_y_weight: if c.lbf_y_weight.is_finite() && c.lbf_y_weight >= 0.0 {
                c.lbf_y_weight
            } else {
                d.lbf_y_weight
            },
        }
    }

    /// 기본값을 C 구조체 형태로 내보낸다 (`swarm_get_default_config`).
    pub(crate) fn default_c() -> CInitialPlacementConfig {
        let d = InitialPlacementSettings::default();
        CInitialPlacementConfig {
            enabled: 0,
            time_limit_secs: d.time_limit_secs,
            grasp: d.grasp as i32,
            kicks: d.kicks as i32,
            step0_frac: d.step0_frac,
            min_frac: d.min_frac,
            iter_cap: d.iter_cap as i32,
            samples_base: d.samples_base as i32,
            growth: d.growth,
            rescue_after: d.rescue_after as i32,
            stall_cap: d.stall_cap as i32,
            cd_min: d.cd_min,
            compact_passes: d.compact_passes as i32,
            reseat_top: d.reseat_top as i32,
            cand_eval_cap: d.cand_eval_cap as i32,
            lbf_x_weight: d.lbf_x_weight,
            lbf_y_weight: d.lbf_y_weight,
        }
    }
}

/// Stable C ABI handshake returned before a caller passes [`CSwarmConfig`].
///
/// `CSwarmConfig` is caller-allocated, so a managed/native size mismatch can
/// otherwise overwrite the caller's buffer.  Keep this record fixed-width and
/// bump [`SWARM_ABI_VERSION`] whenever an incompatible C layout is introduced.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CSwarmAbiInfo {
    pub abi_version: u32,
    pub swarm_config_size: u32,
    pub structure_config_size: u32,
    pub feature_flags: u32,
}

pub const SWARM_ABI_VERSION: u32 = 20;
pub const SWARM_FEATURE_SIMD: u32 = 1 << 0;
pub const SWARM_FEATURE_CAPMODS: u32 = 1 << 1;
pub const SWARM_FEATURE_ADAPTIVE: u32 = 1 << 2;
pub const SWARM_FEATURE_STRUCTURE_FIRST: u32 = 1 << 3;
pub const SWARM_FEATURE_JNI: u32 = 1 << 4;

fn compiled_feature_flags() -> u32 {
    let mut flags = 0;
    if cfg!(feature = "simd") {
        flags |= SWARM_FEATURE_SIMD;
    }
    if cfg!(feature = "capmods") {
        flags |= SWARM_FEATURE_CAPMODS;
    }
    if cfg!(feature = "adaptive") {
        flags |= SWARM_FEATURE_ADAPTIVE;
    }
    if cfg!(feature = "structure-first") {
        flags |= SWARM_FEATURE_STRUCTURE_FIRST;
    }
    if cfg!(feature = "jni") {
        flags |= SWARM_FEATURE_JNI;
    }
    flags
}

/// Int32 값을 최소값과 비교하여 usize로 변환(음수 처리)
fn c_count(value: i32, minimum: usize) -> usize {
    (value.max(0) as usize).max(minimum)
}

/// f32 초 값을 Duration으로 변환. 음수/NaN/무한 = 0, 너무 큰 값 = `MAX_FINITE_DURATION`
/// (`Duration::MAX` 는 `Instant + Duration` 에서 overflow panic 을 일으킨다).
fn c_duration_secs(value: f32) -> Duration {
    if value.is_finite() && value >= 0.0 {
        Duration::try_from_secs_f32(value)
            .unwrap_or(MAX_FINITE_DURATION)
            .min(MAX_FINITE_DURATION)
    } else {
        Duration::ZERO
    }
}

/// C 분리기 설정을 Rust SeparatorConfig로 변환 (`def`는 잘못된 값에 대한 기본값 공급원)
fn build_separator_config(
    sep: &CSeparatorConfig,
    def: &crate::optimizer::separator::SeparatorConfig,
    gls: crate::optimizer::separator::GlsWeightConfig,
) -> crate::optimizer::separator::SeparatorConfig {
    crate::optimizer::separator::SeparatorConfig {
        gls: gls.sanitized(),
        iter_no_imprv_limit: c_count(sep.iter_no_imprv_limit, 0),
        strike_limit: c_count(sep.strike_limit, 0),
        n_workers: c_count(sep.n_workers, 1).min(MAX_SEPARATOR_WORKERS),
        sample_config: crate::sample::search::SampleConfig {
            n_container_samples: c_count(sep.n_container_samples, 1),
            n_focussed_samples: c_count(sep.n_focussed_samples, 1),
            n_coord_descents: c_count(sep.n_coord_descents, 1),
            left_bias_gain: if sep.left_bias_gain.is_finite() && sep.left_bias_gain >= 0.0 {
                sep.left_bias_gain
            } else {
                def.sample_config.left_bias_gain
            },
            // 아직 C ABI에 노출하지 않음 — 기본값(0.0 = 틸트 비활성) 유지
            tilt_range: def.sample_config.tilt_range,
        },
        // `nfp_integration.nfp_sampler` 에서 전파한다(아래 `build_config`). 여기서는
        // 기본값을 유지한다 — separator 블록에는 이 노브가 없다.
        nfp_sampler: def.nfp_sampler,
        // ABI v18 `search` 꼬리 블록에서 호출자가 덮어쓴다. 여기서는 기본값(꺼짐).
        restart_window: def.restart_window,
        iter_floor_override: def.iter_floor_override,
        swap_period: if sep.swap_period >= 0 {
            sep.swap_period as usize
        } else {
            def.swap_period
        },
        swap_attempts: if sep.swap_attempts > 0 {
            (sep.swap_attempts as usize).min(MAX_SWAP_ATTEMPTS_PER_PASS)
        } else {
            def.swap_attempts
        },
        swap_area_ratio: if sep.swap_area_ratio_min > 0.0
            && sep.swap_area_ratio_max >= sep.swap_area_ratio_min
        {
            (sep.swap_area_ratio_min, sep.swap_area_ratio_max)
        } else {
            def.swap_area_ratio
        },
    }
}

/// 설정을 CSwarmConfig에서 SwarmConfig로 변환
/// # cfg : 적용할 설정
/// # 반환값 : SwarmConfig
pub(crate) fn build_config(cfg: &CSwarmConfig) -> crate::config::SwarmConfig {
    let mut config = DEFAULT_SWARM_CONFIG;

    if cfg.rng_seed != 0 {
        config.rng_seed = Some(cfg.rng_seed as usize);
    }

    config.simplify_max_deviation_mm = (cfg.refplace.simplify_max_deviation_mm.is_finite() && cfg.refplace.simplify_max_deviation_mm > 0.0).then_some(cfg.refplace.simplify_max_deviation_mm);
    config.poly_simpl_tolerance = if config.simplify_max_deviation_mm.is_none() && cfg.poly_simpl_tolerance >= 0.0 {
        Some(cfg.poly_simpl_tolerance)
    } else {
        None
    };

    // 겹침 허용이 켜져 있으면 그것이 간격 설정을 이긴다.
    // importer가 offset = separation / 2 로 쓰므로 음수 separation = 조각 축소 = 겹침.
    config.nfp_integration = crate::config::NfpIntegrationConfig {
        ordering_exploration: cfg.nfp_integration.ordering_exploration != 0,
        nfp_sampler: cfg.nfp_integration.nfp_sampler != 0,
        exact_separation_vector: cfg.nfp_integration.exact_separation_vector != 0,
        nfp_hole_filling: cfg.nfp_integration.nfp_hole_filling != 0,
        nfp_reinsertion: cfg.nfp_integration.nfp_reinsertion != 0,
    };

    let overlap = cfg.fabric_tolerance.piece_overlap_mm;
    config.min_item_separation = if overlap.is_finite() && overlap > 0.0 {
        Some(-overlap)
    } else if cfg.min_item_separation >= 0.0 {
        Some(cfg.min_item_separation)
    } else {
        None
    };
    // ABI v19 tail. 0(그리고 알 수 없는 값)은 실현 불가능 = v18 동작.
    config.original_overlap_policy =
        if cfg.original_overlap.excess_policy == ORIGINAL_OVERLAP_FEASIBLE {
            crate::config::OriginalOverlapPolicy::Feasible
        } else {
            crate::config::OriginalOverlapPolicy::Infeasible
        };
    // ABI v20 tail. 0(그리고 알 수 없는 값)은 다중 폴리곤 아이템.
    config.place_group_mode = if cfg.place_group.mode == PLACE_GROUP_RIGID {
        crate::config::PlaceGroupMode::Rigid
    } else {
        crate::config::PlaceGroupMode::MultiPolygon
    };
    // ABI v17 tail. 0 이하는 "제한 없음" 이므로 v16 호출자는 동작이 같다.
    config.init_band_width_mm = {
        let w = cfg.constructor.band_width_mm;
        (w.is_finite() && w > 0.0).then_some(w)
    };
    config.init_band_fill_ratio = cfg.constructor.band_fill_ratio.max(0.0);
    config.init_settle_every = cfg.constructor.settle_every.max(0) as usize;
    config.init_settle_budget_ms = if cfg.constructor.settle_budget_ms > 0 {
        cfg.constructor.settle_budget_ms as u64
    } else {
        DEFAULT_SWARM_CONFIG.init_settle_budget_ms
    };
    config.init_compaction_ms = cfg.constructor.post_init_compaction_ms.max(0) as u64;
    config.fabric_edge_margin_mm = {
        let m = cfg.fabric_edge.margin_mm;
        (m.is_finite() && m >= 0.0 && cfg.fabric_edge.margin_mm != 0.0).then_some(m)
    };

    config.narrow_concavity_cutoff_ratio =
        if cfg.narrow_concavity_cutoff_dist >= 0.0 && cfg.narrow_concavity_cutoff_area >= 0.0 {
            Some((
                cfg.narrow_concavity_cutoff_dist,
                cfg.narrow_concavity_cutoff_area,
            ))
        } else {
            None
        };

    config.lbf_x_multiplier = if cfg.lbf_x_multiplier > 0.0 {
        cfg.lbf_x_multiplier
    } else {
        10.0
    };
    config.lbf_y_multiplier = if cfg.lbf_y_multiplier > 0.0 {
        cfg.lbf_y_multiplier
    } else {
        1.0
    };

    // LBF 래스터 NFP. 0/음수는 "기본값 사용"으로 해석한다.
    // (n_random_samples만 0이 유효한 값 — 난수 샘플 없이 래스터 후보만 쓴다는 뜻)
    let raster_def = DEFAULT_SWARM_CONFIG.lbf_raster;
    config.lbf_raster = RasterNfpConfig {
        enabled: cfg.lbf_raster.enabled != 0,
        cell_ratio: if cfg.lbf_raster.cell_ratio > 0.0 {
            cfg.lbf_raster.cell_ratio
        } else {
            raster_def.cell_ratio
        },
        max_cells: if cfg.lbf_raster.max_cells > 0 {
            cfg.lbf_raster.max_cells as usize
        } else {
            raster_def.max_cells
        },
        n_candidates: if cfg.lbf_raster.n_candidates > 0 {
            cfg.lbf_raster.n_candidates as usize
        } else {
            raster_def.n_candidates
        },
        n_random_samples: if cfg.lbf_raster.n_random_samples >= 0 {
            cfg.lbf_raster.n_random_samples as usize
        } else {
            raster_def.n_random_samples
        },
        // 양쪽 앵커링도 ABI v17 꼬리 블록에서 온다 — 같은 이유로 중간 블록을
        // 넓힐 수 없다. 0 이면 기존 좌·하단 규칙이다.
        double_sided_first_n: if cfg.constructor.double_sided_first_n > 0 {
            cfg.constructor.double_sided_first_n as usize
        } else {
            raster_def.double_sided_first_n
        },
        // 홀 우선 노브는 ABI v14 꼬리 블록에서 온다. `CRasterNfpConfig` 는
        // 오프셋 244의 중간 블록이라 넓히면 뒤 블록이 전부 밀린다.
        // 여기서는 기본값을 넣고, 아래에서 꼬리 블록 값으로 덮는다.
        hole_first: raster_def.hole_first,
        hole_share: raster_def.hole_share,
    };

    // 탐색 단계 Config
    config.expl_cfg = ExplorationConfig {
        repair: cfg.repair.into(),
        shrink_step: if cfg.exploration.shrink_step.is_finite() {
            cfg.exploration.shrink_step
        } else {
            DEFAULT_SWARM_CONFIG.expl_cfg.shrink_step
        },
        time_limit: c_duration_secs(cfg.exploration.time_limit_secs),
        // `Some(0)` would end exploration after the first failed attempt, so 0 and
        // negatives both mean "no limit" (header: `<= 0 = None`).
        max_conseq_failed_attempts: if cfg.exploration.max_conseq_failed_attempts > 0 {
            Some(cfg.exploration.max_conseq_failed_attempts as usize)
        } else {
            None
        },
        solution_pool_distribution_stddev: if cfg
            .exploration
            .solution_pool_distribution_stddev
            .is_finite()
            && cfg.exploration.solution_pool_distribution_stddev > 0.0
        {
            cfg.exploration.solution_pool_distribution_stddev
        } else {
            DEFAULT_SWARM_CONFIG
                .expl_cfg
                .solution_pool_distribution_stddev
        },
        large_item_ch_area_cutoff_percentile: cfg.exploration.large_item_ch_area_cutoff_percentile,
        separator_config: {
            let mut sc = build_separator_config(
                &cfg.exploration.separator,
                &DEFAULT_SWARM_CONFIG.expl_cfg.separator_config,
                // ABI v17 꼬리. 0 으로 채운 호출자는 네이티브 상수를 그대로 받는다.
                cfg.gls.into(),
            );
            // ABI v18 꼬리. 0 = 종전 동작.
            sc.restart_window = cfg.search.sep_restart_window.max(0) as usize;
            sc.iter_floor_override = cfg.search.sep_iter_floor.max(0) as usize;
            sc
        },
        rnr: {
            let def = DEFAULT_SWARM_CONFIG.expl_cfg.rnr;
            crate::config::RuinRecreateConfig {
                period: if cfg.exploration.rnr_period >= 0 {
                    cfg.exploration.rnr_period as usize
                } else {
                    def.period
                },
                band_fraction: if cfg.exploration.rnr_band_fraction > 0.0
                    && cfg.exploration.rnr_band_fraction <= 1.0
                {
                    cfg.exploration.rnr_band_fraction
                } else {
                    def.band_fraction
                },
                max_items: if cfg.exploration.rnr_max_items > 0 {
                    cfg.exploration.rnr_max_items as usize
                } else {
                    def.max_items
                },
                // C ABI 에 없는 Rust 전용 노브 — 기본값(끔)을 그대로 쓴다.
                prefer_original_position: def.prefer_original_position,
                // ABI v18 꼬리. 0 = 꺼짐(종전 동작).
                original_candidate: cfg.search.rnr_original_candidate != 0,
                // ABI v17 tail. 0 = 기본값이므로 v16 호출자는 동작이 같다.
                full_band: cfg.band_rearrange.full_band != 0,
                post_shrink_rearrange: cfg.band_rearrange.post_shrink != 0,
                recreate_bias: {
                    let (x, y) = (
                        cfg.band_rearrange.recreate_x_mult,
                        cfg.band_rearrange.recreate_y_mult,
                    );
                    if x.is_finite() && y.is_finite() && x > 0.0 && y > 0.0 {
                        (x, y)
                    } else {
                        def.recreate_bias
                    }
                },
                min_clear_reinsert_ratio: {
                    let r = cfg.band_rearrange.min_clear_reinsert_ratio;
                    if r.is_finite() && r > 0.0 {
                        r.min(1.0)
                    } else {
                        def.min_clear_reinsert_ratio
                    }
                },
                // ABI v18 tail — 빈틈 순위 / 격자 해상도. 0 = 기본값.
                pocket_band_candidates: cfg.band_rearrange.pocket_band != 0,
                pocket_band_max: if cfg.band_rearrange.pocket_band_max > 0 {
                    cfg.band_rearrange.pocket_band_max as usize
                } else {
                    def.pocket_band_max
                },
                pocket_grid_cell_scale: {
                    let s = cfg.band_rearrange.pocket_grid_cell_scale;
                    if s.is_finite() && s > 0.0 {
                        s.clamp(0.05, 1.0)
                    } else {
                        def.pocket_grid_cell_scale
                    }
                },
                max_loss_growth: {
                    let g = cfg.band_rearrange.max_loss_growth;
                    if g.is_finite() && g > 1.0 {
                        g
                    } else {
                        def.max_loss_growth
                    }
                },
                post_strategy: {
                    let v = cfg.band_rearrange.post_strategy;
                    if (0..=5).contains(&v) {
                        v as u8
                    } else {
                        def.post_strategy
                    }
                },
            }
        },
        // The C ABI has no explicit mode field, so the mode is derived from the
        // `shrink_feedback` block's own documented contract: `fail_decay` in (0, 1)
        // means "apply the step-scale multiplier", which only `uses_feedback()` modes
        // honour (swarm-core/src/optimizer/explore_mods.rs shrink_feedback_params).
        // Without this the block was inert — the host could set fail_decay and nothing
        // read it. The C boundary default is 1.0 (out of range), so the default policy
        // is unchanged and no ABI field was added.
        // ABI v18 꼬리 — 막힌 폭 건너뛰기 / 교란 포트폴리오. 0 = 종전 동작.
        sep_time_cap: std::time::Duration::from_millis(cfg.search.sep_time_cap_ms.max(0) as u64),
        // `-1` 은 «적응형 상한» 을 뜻한다. 그 밖의 음수는 종전대로 제한 없음.
        sep_cap_adaptive: cfg.search.sep_time_cap_ms == SEP_TIME_CAP_ADAPTIVE,
        portfolio_k: cfg.search.portfolio_k.max(0) as usize,
        probe_budget: std::time::Duration::from_millis(cfg.search.probe_budget_ms.max(0) as u64),
        shrink_adapt_mode: if cfg.shrink_feedback.fail_decay.is_finite()
            && cfg.shrink_feedback.fail_decay > 0.0
            && cfg.shrink_feedback.fail_decay < 1.0
        {
            // Feedback (not DensitySlackFeedback): the caller sets the trajectory speed
            // through `shrink_step` alone, so the step is base x step_scale with no
            // second, opaque density multiplier layered on top.
            ShrinkAdaptMode::Feedback
        } else {
            DEFAULT_SWARM_CONFIG.expl_cfg.shrink_adapt_mode
        },
        adaptive_shrink_gain: if cfg.exploration.adaptive_shrink_gain.is_finite()
            && cfg.exploration.adaptive_shrink_gain >= 0.0
        {
            cfg.exploration.adaptive_shrink_gain
        } else {
            DEFAULT_SWARM_CONFIG.expl_cfg.adaptive_shrink_gain
        },
        adaptive_shrink_max_mult: if cfg.exploration.adaptive_shrink_max_mult >= 1.0 {
            cfg.exploration.adaptive_shrink_max_mult
        } else {
            DEFAULT_SWARM_CONFIG.expl_cfg.adaptive_shrink_max_mult
        },
        osc_fail_threshold: if cfg.exploration.osc_fail_threshold >= 0 {
            cfg.exploration.osc_fail_threshold as usize
        } else {
            DEFAULT_SWARM_CONFIG.expl_cfg.osc_fail_threshold
        },
        osc_expand_steps: if cfg.exploration.osc_expand_steps > 0.0 {
            cfg.exploration.osc_expand_steps
        } else {
            DEFAULT_SWARM_CONFIG.expl_cfg.osc_expand_steps
        },
        alns_decay: if cfg.exploration.alns_decay > 0.0 && cfg.exploration.alns_decay < 1.0 {
            cfg.exploration.alns_decay
        } else {
            DEFAULT_SWARM_CONFIG.expl_cfg.alns_decay
        },
        alns_min_weight: if cfg.exploration.alns_min_weight >= 0.0 {
            cfg.exploration.alns_min_weight
        } else {
            DEFAULT_SWARM_CONFIG.expl_cfg.alns_min_weight
        },
        // 폭 완화 복구는 ABI-tail 블록에 있다 (기존 필드 오프셋 보존).
        relief_fail_threshold: if cfg.width_relief.fail_threshold > 0 {
            cfg.width_relief.fail_threshold as usize
        } else {
            DEFAULT_SWARM_CONFIG.expl_cfg.relief_fail_threshold
        },
        relief_step: if cfg.width_relief.step.is_finite() && cfg.width_relief.step > 0.0 {
            cfg.width_relief.step
        } else {
            DEFAULT_SWARM_CONFIG.expl_cfg.relief_step
        },
        relief_max_ratio: if cfg.width_relief.max_ratio.is_finite()
            && cfg.width_relief.max_ratio > 0.0
        {
            cfg.width_relief.max_ratio
        } else {
            DEFAULT_SWARM_CONFIG.expl_cfg.relief_max_ratio
        },
        relief_max_rounds: if cfg.width_relief.max_rounds > 0 {
            cfg.width_relief.max_rounds as usize
        } else {
            DEFAULT_SWARM_CONFIG.expl_cfg.relief_max_rounds
        },
        relief_strike_limit: if cfg.width_relief.strike_limit > 0 {
            cfg.width_relief.strike_limit as usize
        } else {
            DEFAULT_SWARM_CONFIG.expl_cfg.relief_strike_limit
        },
        relief_round_budget: if cfg.width_relief.round_budget_ms > 0 {
            Duration::from_millis(cfg.width_relief.round_budget_ms as u64)
        } else {
            DEFAULT_SWARM_CONFIG.expl_cfg.relief_round_budget
        },
        // 실패 피드백 shrink도 ABI-tail 블록에 있다.
        // 범위 밖 감쇠는 "비활성"(1.0)으로 정규화한다 — 0이나 음수를 스텝에 곱하면
        // 축소가 멈추거나 폭이 커져 버리므로 조용히 통과시키면 안 된다.
        shrink_fail_decay: if cfg.shrink_feedback.fail_decay.is_finite()
            && cfg.shrink_feedback.fail_decay > 0.0
            && cfg.shrink_feedback.fail_decay < 1.0
        {
            cfg.shrink_feedback.fail_decay
        } else {
            1.0
        },
        shrink_scale_min: if cfg.shrink_feedback.scale_min.is_finite()
            && cfg.shrink_feedback.scale_min > 0.0
            && cfg.shrink_feedback.scale_min <= 1.0
        {
            cfg.shrink_feedback.scale_min
        } else {
            DEFAULT_SWARM_CONFIG.expl_cfg.shrink_scale_min
        },
    };

    // 압축 단계 Config
    config.cmpr_cfg = CompressionConfig {
        shrink_range: normalize_shrink_range((
            cfg.compression.shrink_range_min,
            cfg.compression.shrink_range_max,
        ))
        .unwrap_or(DEFAULT_SWARM_CONFIG.cmpr_cfg.shrink_range),
        time_limit: c_duration_secs(cfg.compression.time_limit_secs),
        shrink_decay: match cfg.compression.shrink_decay_strategy {
            1 => ShrinkDecayStrategy::FailureBased(cfg.compression.shrink_decay_ratio),
            _ => ShrinkDecayStrategy::TimeBased,
        },
        separator_config: {
            let mut sc = build_separator_config(
            &cfg.compression.separator,
            &DEFAULT_SWARM_CONFIG.cmpr_cfg.separator_config,
            // 압축은 탐색과 다른 단계다. 탐색용 GLS 노브를 여기까지 끌고 오면 한
            // 팔이 두 단계를 동시에 바꿔 원인을 분리할 수 없다. 기본값을 쓴다.
            DEFAULT_SWARM_CONFIG.cmpr_cfg.separator_config.gls,
            );
            // ABI v18 꼬리. 0 = 종전 동작.
            sc.restart_window = cfg.search.cmpr_restart_window.max(0) as usize;
            sc
        },
    };

    // 후처리 단계 Config (지정하지 않은 세부 항목은 기본값 유지)
    {
        let def = DEFAULT_SWARM_CONFIG.polish_cfg;
        let p = &cfg.polish;
        config.polish_cfg = PolishConfig {
            // 회수 노브는 ABI v15 꼬리 블록에서 온다. 여기서는 기본값을 넣고
            // 아래에서 꼬리 블록 값으로 덮는다.
            reclaim: def.reclaim,
            enabled: p.enabled != 0,
            time_limit: c_duration_secs(p.time_limit_secs),
            n_rounds: c_count(p.n_rounds, 1),
            slide: SlideConfig {
                enabled: p.slide_enabled != 0,
                n_passes: c_count(p.slide_n_passes, 1),
                slide_down: p.slide_down != 0,
                safety_ratio: if p.slide_safety_ratio >= 0.0 {
                    p.slide_safety_ratio
                } else {
                    def.slide.safety_ratio
                },
                ..def.slide
            },
            hole_fill: HoleFillConfig {
                enabled: p.hole_fill_enabled != 0,
                cell_size_ratio: if p.hole_fill_cell_size_ratio > 0.0 {
                    p.hole_fill_cell_size_ratio
                } else {
                    def.hole_fill.cell_size_ratio
                },
                max_candidates: c_count(p.hole_fill_max_candidates, 1),
                candidate_band: if p.hole_fill_candidate_band > 0.0 {
                    p.hole_fill_candidate_band
                } else {
                    def.hole_fill.candidate_band
                },
                n_samples_per_pocket: c_count(p.hole_fill_n_samples, 1),
                ..def.hole_fill
            },
            lp: LpCompactionConfig {
                enabled: p.lp_enabled != 0,
                n_iterations: c_count(p.lp_n_iterations, 1),
                trust_region_ratio: if p.lp_trust_region_ratio > 0.0 {
                    p.lp_trust_region_ratio
                } else {
                    def.lp.trust_region_ratio
                },
                left_pull_weight: if p.lp_left_pull_weight >= 0.0 {
                    p.lp_left_pull_weight
                } else {
                    def.lp.left_pull_weight
                },
                ..def.lp
            },
            nfp_left: def.nfp_left,
        };
    }

    // 회전 자유도
    config.allow_180_rotation = cfg.allow_180_rotation != 0;
    let tilt_rad = if cfg.tilt_range_deg.is_finite() && cfg.tilt_range_deg > 0.0 {
        cfg.tilt_range_deg.to_radians()
    } else {
        0.0
    };
    config.expl_cfg.separator_config.sample_config.tilt_range = tilt_rad;
    config.cmpr_cfg.separator_config.sample_config.tilt_range = tilt_rad;

    // Experimental P0-P5 path. Keep every unexposed structure setting at its
    // Rust default and only replace the stable C ABI subset below.
    {
        let def = DEFAULT_SWARM_CONFIG.structure;
        let s = &cfg.structure_first;
        let mut structure = def;
        structure.enabled = s.enabled != 0;
        structure.motif.enabled = s.motif_enabled != 0;
        structure.max_active_motifs = if s.max_active_motifs > 0 {
            s.max_active_motifs as usize
        } else {
            def.max_active_motifs
        };
        structure.construction_starts = if s.construction_starts > 0 {
            s.construction_starts as usize
        } else {
            def.construction_starts
        };
        structure.profile_row_height_ratio =
            if s.profile_row_height_ratio.is_finite() && s.profile_row_height_ratio > 0.0 {
                s.profile_row_height_ratio
            } else {
                def.profile_row_height_ratio
            };
        structure.profile_top_k = if s.profile_top_k > 0 {
            s.profile_top_k as usize
        } else {
            def.profile_top_k
        };
        structure.p1_time_limit = if s.p1_time_limit_secs.is_finite() && s.p1_time_limit_secs >= 0.0
        {
            c_duration_secs(s.p1_time_limit_secs)
        } else {
            def.p1_time_limit
        };
        structure.profile_max_rows = if s.profile_max_rows > 0 {
            s.profile_max_rows as usize
        } else {
            def.profile_max_rows
        };
        structure.item_beam_width = if s.item_beam_width > 0 {
            s.item_beam_width as usize
        } else {
            def.item_beam_width
        };
        structure.candidate_pool_cap = if s.candidate_pool_cap > 0 {
            s.candidate_pool_cap as usize
        } else {
            def.candidate_pool_cap
        };
        structure.slide_refine_steps = if s.slide_refine_steps > 0 {
            s.slide_refine_steps as usize
        } else {
            def.slide_refine_steps
        };
        structure.p1_include_legacy_baseline = s.p1_include_legacy_baseline != 0;
        structure.motif.pair_max_offset_candidates = if s.pair_max_offset_candidates > 0 {
            s.pair_max_offset_candidates as usize
        } else {
            def.motif.pair_max_offset_candidates
        };
        structure.motif.pair_max_orientation_work = if s.pair_max_orientation_work > 0 {
            s.pair_max_orientation_work as usize
        } else {
            def.motif.pair_max_orientation_work
        };
        structure.progress_callback_interval = if s.progress_callback_interval_ms > 0 {
            Duration::from_millis(s.progress_callback_interval_ms as u64)
        } else {
            def.progress_callback_interval
        };
        structure.p2_time_limit = if s.p2_time_limit_secs.is_finite() && s.p2_time_limit_secs >= 0.0
        {
            c_duration_secs(s.p2_time_limit_secs)
        } else {
            def.p2_time_limit
        };
        structure.p2_shrink_range =
            normalize_shrink_range((s.p2_shrink_range_min, s.p2_shrink_range_max))
                .unwrap_or(def.p2_shrink_range);
        structure.p2_failure_decay =
            if s.p2_failure_decay.is_finite() && (0.0..1.0).contains(&s.p2_failure_decay) {
                s.p2_failure_decay
            } else {
                def.p2_failure_decay
            };
        structure.p3_slack_ratio = if s.p3_slack_ratio.is_finite() && s.p3_slack_ratio >= 0.0 {
            s.p3_slack_ratio
        } else {
            def.p3_slack_ratio
        };
        structure.p3_time_limit = if s.p3_time_limit_secs.is_finite() && s.p3_time_limit_secs >= 0.0
        {
            Some(c_duration_secs(s.p3_time_limit_secs))
        } else {
            None
        };
        structure.p4_cycles = if s.p4_cycles >= 0 {
            s.p4_cycles as usize
        } else {
            def.p4_cycles
        };
        structure.p4_cycle_time_limit =
            if s.p4_cycle_time_limit_secs.is_finite() && s.p4_cycle_time_limit_secs >= 0.0 {
                c_duration_secs(s.p4_cycle_time_limit_secs)
            } else {
                def.p4_cycle_time_limit
            };
        structure.p4_ruin_fraction = if s.p4_ruin_fraction.is_finite()
            && s.p4_ruin_fraction > 0.0
            && s.p4_ruin_fraction <= 1.0
        {
            s.p4_ruin_fraction
        } else {
            def.p4_ruin_fraction
        };
        structure.p4_max_items = if s.p4_max_items > 0 {
            s.p4_max_items as usize
        } else {
            def.p4_max_items
        };
        structure.p4_stagnation_cycles = if s.p4_stagnation_cycles > 0 {
            s.p4_stagnation_cycles as usize
        } else {
            def.p4_stagnation_cycles
        };
        structure.p5_snap_rotations = s.p5_snap_rotations != 0;
        config.structure = structure.sanitized();
    }

    // 정체 감지 기반 탈출 (ABI v6 tail block). 잘못된 값은 코어 기본값으로 대체한다.
    {
        let e = &cfg.escape;
        let def = crate::config::DEFAULT_ADAPTIVE_ESCAPE_CONFIG;
        config.escape_enabled = e.enabled != 0;

        let mut escape = def;
        escape.stagnation.window_floor = if e.window_floor > 0 {
            e.window_floor as usize
        } else {
            def.stagnation.window_floor
        };
        escape.stagnation.window_item_factor = if cfg.search.escape_window_absolute != 0 {
            // ABI v18 꼬리: 창 = window_floor 절대값. 조각 수 배수 하한을 끈다.
            0
        } else if e.window_item_factor > 0 {
            e.window_item_factor as usize
        } else {
            def.stagnation.window_item_factor
        };
        escape.stagnation.efficiency_eps =
            if e.efficiency_eps.is_finite() && e.efficiency_eps > 0.0 {
                e.efficiency_eps
            } else {
                def.stagnation.efficiency_eps
            };
        escape.stagnation.diversity_threshold =
            if e.diversity_threshold.is_finite() && e.diversity_threshold > 0.0 {
                e.diversity_threshold
            } else {
                def.stagnation.diversity_threshold
            };
        escape.enable_conflict_burst = e.enable_conflict_burst != 0;
        escape.enable_swarm_diversification = e.enable_swarm_diversification != 0;
        escape.conflict_burst.max_push_items = if e.max_push_items > 0 {
            e.max_push_items as usize
        } else {
            def.conflict_burst.max_push_items
        };
        escape.max_infeasible_pool = if e.max_infeasible_pool > 0 {
            e.max_infeasible_pool as usize
        } else {
            def.max_infeasible_pool
        };
        config.escape = escape;
    }

    // NFP 기반 초기 배치 시드 (ABI v6 tail block).
    {
        let n = &cfg.nfp_lbf;
        let def = crate::config::DEFAULT_NFP_LBF_SEED_CONFIG;
        config.nfp_lbf = crate::config::NfpLbfSeedConfig {
            enabled: n.enabled != 0,
            n_orderings: if n.n_orderings > 0 {
                n.n_orderings as usize
            } else {
                def.n_orderings
            },
            cache_budget_ms: if n.cache_budget_ms > 0 {
                n.cache_budget_ms as u64
            } else {
                def.cache_budget_ms
            },
            total_budget_ms: if n.total_budget_ms > 0 {
                n.total_budget_ms as u64
            } else {
                def.total_budget_ms
            },
            allow_180: n.allow_180 != 0,
            enable_prefix_reuse: n.enable_prefix_reuse != 0,
            // 0.0은 "단순화 비활성"이라는 유효한 값이므로 음수/비유한만 걸러낸다.
            score_weights: {
                let d = crate::nfp_lbf::PlacementScoreWeights::default();
                let pick = |v: f32, def: f32| if v.is_finite() && v >= 0.0 { v } else { def };
                crate::nfp_lbf::PlacementScoreWeights {
                    w1_strip_length: d.w1_strip_length,
                    w2_waste: pick(cfg.constructor.nfp_w2_waste, d.w2_waste),
                    w3_contact: pick(cfg.constructor.nfp_w3_contact, d.w3_contact),
                    w4_y: pick(cfg.constructor.nfp_w4_y, d.w4_y),
                    score_first: cfg.constructor.nfp_score_first != 0,
                    length_tie_mm: cfg.constructor.nfp_length_tie_mm.max(0.0),
                }
            },
            concave_simplify_tolerance: if n.concave_simplify_tolerance.is_finite()
                && n.concave_simplify_tolerance >= 0.0
            {
                n.concave_simplify_tolerance
            } else {
                def.concave_simplify_tolerance
            },
            // 탐침 예산은 별도 tail block(ABI v9)에 있다. 오래된 호스트가 이 블록을
            // 0으로 두고 넘겨도 기본값으로 떨어지므로 안전하다.
            max_candidate_probes: if cfg.nfp_lbf_tuning.max_candidate_probes > 0 {
                cfg.nfp_lbf_tuning.max_candidate_probes as usize
            } else {
                def.max_candidate_probes
            },
        };
    }

    // NFP 초기 배치 전략 7종 (ABI v10 tail block).
    //
    // `<=0` 필드는 `normalized()`가 코어 기본값으로 올린다 - 오래된 호스트가 이 블록을
    // 0으로 두고 넘겨도 안전하다.
    {
        let n = &cfg.nfp_strategy;
        config.nfp_strategy = swarm_core::nfp_strategy::NfpStrategyConfig {
            instance_rotations: n.instance_rotations != 0,
            continuous_rotation_step_deg: n.continuous_rotation_step_deg as i64,
            pairwise_clustering: n.pairwise_clustering != 0,
            cluster_min_gain: n.cluster_min_gain,
            contact_maximization: n.contact_maximization != 0,
            pocket_first: n.pocket_first != 0,
            beam_search: n.beam_search != 0,
            beam_width: n.beam_width.max(0) as usize,
            best_fit_selection: n.best_fit_selection != 0,
            best_fit_lookahead: n.best_fit_lookahead.max(0) as usize,
            sliding_compaction: n.sliding_compaction != 0,
            sliding_rounds: n.sliding_rounds.max(0) as usize,
            // ⑧⑨는 v11 tail block 에서 온다 — 중간 블록을 늘리지 않기 위해서다.
            accretion: cfg.nfp_strategy2.accretion != 0,
            tail_reflow: cfg.nfp_strategy2.tail_reflow != 0,
            reflow_rounds: cfg.nfp_strategy2.reflow_rounds.max(0) as usize,
        }
        .normalized();
    }

    // LP/QP 솔버 선택 (ABI v10 tail block).
    {
        let l = &cfg.lp_solver;
        let lp_def = swarm_core::lp_solver::DEFAULT_LP_SOLVER_CONFIG;
        config.lp_solver = swarm_core::lp_solver::LpSolverConfig {
            backend: match l.backend {
                1 => swarm_core::lp_solver::LpSolverKind::Clarabel,
                2 => swarm_core::lp_solver::LpSolverKind::Clp,
                3 => swarm_core::lp_solver::LpSolverKind::Xpress,
                // 범위 밖 값은 조용히 기본 엔진으로 - 경계에서 프로세스를 죽이지 않는다.
                _ => swarm_core::lp_solver::LpSolverKind::Microlp,
            },
            quadratic_regularization: l.quadratic_regularization,
            fallback: l.fallback != 0,
            time_limit_ms: if l.time_limit_ms > 0 {
                l.time_limit_ms as u64
            } else {
                lp_def.time_limit_ms
            },
        }
        .normalized();
    }

    // ① NFP 샘플러는 후보를 만드는 주체인 **separator** 가 읽는다.
    // `NfpIntegrationConfig` 에만 두면 아무도 읽지 않는다 — 그것이 이 플래그가 오래
    // 죽어 있던 이유다(WPF 체크박스가 Rust 설정까지 도달한 뒤 버려졌다).
    //
    // **반드시 `expl_cfg`/`cmpr_cfg` 대입이 전부 끝난 뒤에** 전파해야 한다. 위쪽에 두면
    // 뒤따르는 separator 설정 조립이 통째로 덮어쓴다(실제로 그렇게 넣었다가 테스트가
    // 잡았다). `nfp_sampler_flag_reaches_the_separator_config` 가 이 순서를 지킨다.
    config.expl_cfg.separator_config.nfp_sampler = config.nfp_integration.nfp_sampler;
    config.cmpr_cfg.separator_config.nfp_sampler = config.nfp_integration.nfp_sampler;

    // ABI v12 tail — 레퍼런스 설계 엔진(실험).
    config.refplace = swarm_core::config::RefPlaceSeedConfig {
        enabled: cfg.refplace.enabled != 0,
        column_draft: cfg.refplace._reserved0 == 1,
        group_fallback: cfg.refplace.group_fallback.clamp(0, 1) as u8,
    };

    // ABI v13 tail — LBF 배치 순서.
    config.lbf_order =
        swarm_core::optimizer::lbf::LbfOrderMode::from_raw(cfg.lbf_order.mode);

    // ABI v14 tail — 홀 우선 부착. `lbf_raster` 는 오프셋 244의 중간 블록이라
    // 넓힐 수 없으므로, 노브는 꼬리 블록에 두고 여기서 합류시킨다.
    config.lbf_raster.hole_first = cfg.hole_first.enabled != 0;
    config.lbf_raster.hole_share = if cfg.hole_first.share > 0.0 {
        cfg.hole_first.share
    } else {
        DEFAULT_SWARM_CONFIG.lbf_raster.hole_share
    };

    // ABI v15 tail — 잔여 예산 회수. `CPolishConfig` 는 중간 블록이라 넓힐 수 없다.
    let reclaim_def = DEFAULT_SWARM_CONFIG.polish_cfg.reclaim;
    config.polish_cfg.reclaim = swarm_core::config::ReclaimConfig {
        enabled: cfg.reclaim.enabled != 0,
        compress_share: if cfg.reclaim.compress_share > 0.0 {
            cfg.reclaim.compress_share
        } else {
            reclaim_def.compress_share
        },
        max_cycles: cfg.reclaim.max_cycles.max(0) as usize,
        min_slice_ms: if cfg.reclaim.min_slice_ms > 0 {
            cfg.reclaim.min_slice_ms as u64
        } else {
            reclaim_def.min_slice_ms
        },
        mode: swarm_core::config::ReclaimMode::from_raw(cfg.reclaim.mode),
        ruin_band_fraction: if cfg.reclaim.ruin_band_fraction > 0.0 {
            cfg.reclaim.ruin_band_fraction
        } else {
            reclaim_def.ruin_band_fraction
        },
        ruin_max_items: if cfg.reclaim.ruin_max_items > 0 {
            cfg.reclaim.ruin_max_items as usize
        } else {
            reclaim_def.ruin_max_items
        },
        stagnation_cycles: if cfg.reclaim.stagnation_cycles > 0 {
            cfg.reclaim.stagnation_cycles as usize
        } else {
            reclaim_def.stagnation_cycles
        },
    };

    config
}

/// Extract callback-only diagnostics without coupling them to the solver config.
pub(crate) fn build_callback_options(cfg: &CSwarmConfig) -> CallbackOptions {
    CallbackOptions {
        collect_loss: cfg.callback_options.collect_loss != 0,
        collect_collision_status: cfg.callback_options.collect_collision_status != 0,
    }
}

/// Rust SeparatorConfig를 C 구조체로 내보낸다 (swarm_get_default_config용)
fn export_separator_config(sep: &crate::optimizer::separator::SeparatorConfig) -> CSeparatorConfig {
    CSeparatorConfig {
        iter_no_imprv_limit: sep.iter_no_imprv_limit as i32,
        strike_limit: sep.strike_limit as i32,
        n_workers: sep.n_workers as i32,
        n_container_samples: sep.sample_config.n_container_samples as i32,
        n_focussed_samples: sep.sample_config.n_focussed_samples as i32,
        n_coord_descents: sep.sample_config.n_coord_descents as i32,
        swap_period: sep.swap_period as i32,
        swap_attempts: sep.swap_attempts as i32,
        swap_area_ratio_min: sep.swap_area_ratio.0,
        swap_area_ratio_max: sep.swap_area_ratio.1,
        left_bias_gain: sep.sample_config.left_bias_gain,
    }
}

// region: DLL Export 함수 정의

/// Return the fixed-size ABI and compiled-feature handshake.
///
/// Callers must invoke this before passing a caller-allocated configuration
/// record.  Returns `-1` for a null output pointer and `0` on success.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_get_abi_info(info: *mut CSwarmAbiInfo) -> i32 {
    if info.is_null() {
        return -1;
    }

    unsafe {
        *info = CSwarmAbiInfo {
            abi_version: SWARM_ABI_VERSION,
            swarm_config_size: std::mem::size_of::<CSwarmConfig>() as u32,
            structure_config_size: std::mem::size_of::<CStructureFirstConfig>() as u32,
            feature_flags: compiled_feature_flags(),
        };
    }
    0
}

/// 기본 Swarm 설정을 가져옴
/// # config : CSwarmConfig 포인터
#[unsafe(no_mangle)]
pub extern "C" fn swarm_get_default_config(config: *mut CSwarmConfig) {
    if config.is_null() {
        return;
    }
    let cfg = unsafe { &mut *config };
    let def = DEFAULT_SWARM_CONFIG;

    cfg.rng_seed = 0;
    cfg.poly_simpl_tolerance = def.poly_simpl_tolerance.unwrap_or(-1.0);
    cfg.refplace.simplify_max_deviation_mm = def.simplify_max_deviation_mm.unwrap_or(0.0);
    cfg.min_item_separation = def.min_item_separation.unwrap_or(-1.0);
    cfg.narrow_concavity_cutoff_dist = def
        .narrow_concavity_cutoff_ratio
        .map(|(d, _)| d)
        .unwrap_or(-1.0);
    cfg.narrow_concavity_cutoff_area = def
        .narrow_concavity_cutoff_ratio
        .map(|(_, a)| a)
        .unwrap_or(-1.0);
    cfg.lbf_x_multiplier = def.lbf_x_multiplier;
    cfg.lbf_y_multiplier = def.lbf_y_multiplier;

    cfg.exploration = CExplorationConfig {
        shrink_step: def.expl_cfg.shrink_step,
        time_limit_secs: def.expl_cfg.time_limit.as_secs_f32(),
        max_conseq_failed_attempts: def
            .expl_cfg
            .max_conseq_failed_attempts
            .map(|v| v as i32)
            .unwrap_or(-1),
        solution_pool_distribution_stddev: def.expl_cfg.solution_pool_distribution_stddev,
        large_item_ch_area_cutoff_percentile: def.expl_cfg.large_item_ch_area_cutoff_percentile,
        separator: export_separator_config(&def.expl_cfg.separator_config),
        rnr_period: def.expl_cfg.rnr.period as i32,
        rnr_band_fraction: def.expl_cfg.rnr.band_fraction,
        rnr_max_items: def.expl_cfg.rnr.max_items as i32,
        adaptive_shrink_gain: def.expl_cfg.adaptive_shrink_gain,
        adaptive_shrink_max_mult: def.expl_cfg.adaptive_shrink_max_mult,
        osc_fail_threshold: def.expl_cfg.osc_fail_threshold as i32,
        osc_expand_steps: def.expl_cfg.osc_expand_steps,
        alns_decay: def.expl_cfg.alns_decay,
        alns_min_weight: def.expl_cfg.alns_min_weight,
    };

    cfg.compression = CCompressionConfig {
        shrink_range_min: def.cmpr_cfg.shrink_range.0,
        shrink_range_max: def.cmpr_cfg.shrink_range.1,
        time_limit_secs: def.cmpr_cfg.time_limit.as_secs_f32(),
        shrink_decay_strategy: match def.cmpr_cfg.shrink_decay {
            ShrinkDecayStrategy::TimeBased => 0,
            ShrinkDecayStrategy::FailureBased(_) => 1,
        },
        shrink_decay_ratio: match def.cmpr_cfg.shrink_decay {
            ShrinkDecayStrategy::FailureBased(r) => r,
            _ => 0.9,
        },
        separator: export_separator_config(&def.cmpr_cfg.separator_config),
    };

    cfg.polish = CPolishConfig {
        enabled: def.polish_cfg.enabled as i32,
        time_limit_secs: def.polish_cfg.time_limit.as_secs_f32(),
        n_rounds: def.polish_cfg.n_rounds as i32,

        slide_enabled: def.polish_cfg.slide.enabled as i32,
        slide_n_passes: def.polish_cfg.slide.n_passes as i32,
        slide_down: def.polish_cfg.slide.slide_down as i32,
        slide_safety_ratio: def.polish_cfg.slide.safety_ratio,

        hole_fill_enabled: def.polish_cfg.hole_fill.enabled as i32,
        hole_fill_cell_size_ratio: def.polish_cfg.hole_fill.cell_size_ratio,
        hole_fill_max_candidates: def.polish_cfg.hole_fill.max_candidates as i32,
        hole_fill_candidate_band: def.polish_cfg.hole_fill.candidate_band,
        hole_fill_n_samples: def.polish_cfg.hole_fill.n_samples_per_pocket as i32,

        lp_enabled: def.polish_cfg.lp.enabled as i32,
        lp_n_iterations: def.polish_cfg.lp.n_iterations as i32,
        lp_trust_region_ratio: def.polish_cfg.lp.trust_region_ratio,
        lp_left_pull_weight: def.polish_cfg.lp.left_pull_weight,
    };

    cfg.lbf_raster = CRasterNfpConfig {
        enabled: def.lbf_raster.enabled as i32,
        cell_ratio: def.lbf_raster.cell_ratio,
        max_cells: def.lbf_raster.max_cells as i32,
        n_candidates: def.lbf_raster.n_candidates as i32,
        n_random_samples: def.lbf_raster.n_random_samples as i32,
    };
    cfg.hole_first = CHoleFirstConfig {
        enabled: def.lbf_raster.hole_first as i32,
        share: def.lbf_raster.hole_share,
        _reserved0: 0,
        _reserved1: 0,
    };
    cfg.reclaim = CReclaimConfig {
        enabled: def.polish_cfg.reclaim.enabled as i32,
        compress_share: def.polish_cfg.reclaim.compress_share,
        max_cycles: def.polish_cfg.reclaim.max_cycles as i32,
        min_slice_ms: def.polish_cfg.reclaim.min_slice_ms as i32,
        mode: 0,
        ruin_band_fraction: def.polish_cfg.reclaim.ruin_band_fraction,
        ruin_max_items: def.polish_cfg.reclaim.ruin_max_items as i32,
        stagnation_cycles: def.polish_cfg.reclaim.stagnation_cycles as i32,
    };

    cfg.allow_180_rotation = def.allow_180_rotation as i32;
    cfg.tilt_range_deg = def
        .expl_cfg
        .separator_config
        .sample_config
        .tilt_range
        .to_degrees();

    cfg.structure_first = CStructureFirstConfig {
        enabled: def.structure.enabled as i32,
        motif_enabled: def.structure.motif.enabled as i32,
        max_active_motifs: def.structure.max_active_motifs as i32,
        construction_starts: def.structure.construction_starts as i32,
        profile_row_height_ratio: def.structure.profile_row_height_ratio,
        profile_top_k: def.structure.profile_top_k as i32,
        p2_time_limit_secs: def.structure.p2_time_limit.as_secs_f32(),
        p2_shrink_range_min: def.structure.p2_shrink_range.0,
        p2_shrink_range_max: def.structure.p2_shrink_range.1,
        p2_failure_decay: def.structure.p2_failure_decay,
        p3_slack_ratio: def.structure.p3_slack_ratio,
        p3_time_limit_secs: def
            .structure
            .p3_time_limit
            .map(|duration| duration.as_secs_f32())
            .unwrap_or(-1.0),
        p4_cycles: def.structure.p4_cycles as i32,
        p4_cycle_time_limit_secs: def.structure.p4_cycle_time_limit.as_secs_f32(),
        p4_ruin_fraction: def.structure.p4_ruin_fraction,
        p4_max_items: def.structure.p4_max_items as i32,
        p4_stagnation_cycles: def.structure.p4_stagnation_cycles as i32,
        p5_snap_rotations: def.structure.p5_snap_rotations as i32,
        p1_time_limit_secs: def.structure.p1_time_limit.as_secs_f32(),
        profile_max_rows: def.structure.profile_max_rows as i32,
        item_beam_width: def.structure.item_beam_width as i32,
        candidate_pool_cap: def.structure.candidate_pool_cap as i32,
        slide_refine_steps: def.structure.slide_refine_steps as i32,
        p1_include_legacy_baseline: def.structure.p1_include_legacy_baseline as i32,
        pair_max_offset_candidates: def.structure.motif.pair_max_offset_candidates as i32,
        pair_max_orientation_work: def.structure.motif.pair_max_orientation_work as i32,
        // Legacy ABI fields; the values are ignored but keep their former defaults.
        diagnostics_max_raster_cells: 250_000,
        diagnostics_max_pair_comparisons: 250_000,
        progress_callback_interval_ms: def
            .structure
            .progress_callback_interval
            .as_millis()
            .min(i32::MAX as u128) as i32,
    };
    cfg.callback_options = CCallbackOptionsConfig {
        collect_loss: DEFAULT_CALLBACK_OPTIONS.collect_loss as i32,
        collect_collision_status: DEFAULT_CALLBACK_OPTIONS.collect_collision_status as i32,
    };
    cfg.initial_placement = InitialPlacementSettings::default_c();
    cfg.width_relief = default_width_relief_c();
    cfg.shrink_feedback = default_shrink_feedback_c();
    cfg.escape = default_escape_c();
    cfg.nfp_lbf = default_nfp_lbf_c();
    cfg.fabric_tolerance = default_fabric_tolerance_c();
    cfg.nfp_integration = default_nfp_integration_c();
    cfg.nfp_lbf_tuning = default_nfp_lbf_tuning_c();
    cfg.nfp_strategy = default_nfp_strategy_c();
    cfg.lp_solver = default_lp_solver_c();
    cfg.repair = def.expl_cfg.repair.into();
    // ABI v17 tail — 전부 꺼짐. 0 초기화한 호출자와 같은 값이므로 v16 동작과 동일하다.
    cfg.band_rearrange = default_band_rearrange_c();
    cfg.fabric_edge = default_fabric_edge_c();
    cfg.gls = default_gls_weight_c();
    cfg.constructor = default_constructor_c();
    cfg.search = default_search_tuning_c();
    // ABI v19 tail — 실현 불가능. 0 초기화한 호출자와 같은 값이다.
    cfg.original_overlap = default_original_overlap_c();
    // ABI v20 tail — 다중 폴리곤 아이템. 0 초기화한 호출자와 같은 값이다.
    cfg.place_group = default_place_group_c();
}

/// String 포인터를 해제
/// # ptr : 해제할 C 문자열 포인터
#[unsafe(no_mangle)]
pub extern "C" fn swarm_free_string(ptr: *mut c_char) {
    if ptr.is_null() {
        return;
    }
    unsafe {
        let _ = CString::from_raw(ptr);
    }
}

/// Swarm 버전 가져오기
/// # 반환값 : 읽기전용 C 문자열 포인터
#[unsafe(no_mangle)]
pub extern "C" fn swarm_version() -> *const c_char {
    concat!(env!("PRODUCT_VERSION"), "\0").as_ptr() as *const c_char
}

// endregion

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn repair_defaults_roundtrip_and_explicit_tail_maps_to_explore() {
        use crate::optimizer::separator::SeparationRepairConfig;
        let mut c: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c);
        assert_eq!(build_config(&c).expl_cfg.repair, SeparationRepairConfig::default());
        c.repair = CSeparationRepairConfig {
            direction_enabled: 1, cluster_lp_enabled: -1, chain_enabled: 1,
            stagnation_iterations: 31, max_passes: 3, max_units: 6,
            max_evaluations: 99, time_budget_ms: 40, max_move_mm: 3.5,
        };
        let r = build_config(&c).expl_cfg.repair;
        assert!(r.direction_enabled && r.cluster_lp_enabled && r.chain_enabled);
        assert_eq!((r.stagnation_iterations,r.max_passes,r.max_units,r.max_evaluations,r.time_budget_ms),(31,3,6,99,40));
        assert_eq!(r.max_move_mm,3.5);
        c.repair.max_units = u32::MAX;
        c.repair.max_move_mm = f32::NAN;
        assert_eq!(build_config(&c).expl_cfg.repair.max_units,8);
        assert_eq!(build_config(&c).expl_cfg.repair.max_move_mm,2.0);
    }

    #[test]
    fn place_group_mode_reaches_the_core_config() {
        let zeroed: CSwarmConfig = unsafe { std::mem::zeroed() };
        assert_eq!(build_config(&zeroed).place_group_mode, crate::config::PlaceGroupMode::MultiPolygon);
        let mut c: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c);
        assert_eq!(c.place_group.mode, PLACE_GROUP_MULTIPOLYGON);
        c.place_group.mode = PLACE_GROUP_RIGID;
        assert_eq!(build_config(&c).place_group_mode, crate::config::PlaceGroupMode::Rigid);
        c.place_group.mode = 7;
        assert_eq!(build_config(&c).place_group_mode, crate::config::PlaceGroupMode::MultiPolygon);
    }

    #[test]
    fn original_overlap_policy_reaches_the_core_config() {
        use crate::config::OriginalOverlapPolicy;
        // A zero-filled buffer (older host) and the published default both reject.
        let zeroed: CSwarmConfig = unsafe { std::mem::zeroed() };
        assert_eq!(build_config(&zeroed).original_overlap_policy, OriginalOverlapPolicy::Infeasible);
        let mut c: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c);
        assert_eq!(c.original_overlap.excess_policy, ORIGINAL_OVERLAP_INFEASIBLE);
        assert_eq!(build_config(&c).original_overlap_policy, OriginalOverlapPolicy::Infeasible);
        c.original_overlap.excess_policy = ORIGINAL_OVERLAP_FEASIBLE;
        assert_eq!(build_config(&c).original_overlap_policy, OriginalOverlapPolicy::Feasible);
        // Unknown values keep the safe default.
        for v in [2, -1, i32::MAX] {
            c.original_overlap.excess_policy = v;
            assert_eq!(build_config(&c).original_overlap_policy, OriginalOverlapPolicy::Infeasible);
        }
    }

    #[test]
    fn exploration_knobs_are_sanitized_at_the_abi_boundary() {
        let mut c: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c);

        // max_conseq_failed_attempts: `<= 0` = None, positive = Some(n)
        for v in [0, -1, i32::MIN] {
            c.exploration.max_conseq_failed_attempts = v;
            assert_eq!(build_config(&c).expl_cfg.max_conseq_failed_attempts, None);
        }
        c.exploration.max_conseq_failed_attempts = 7;
        assert_eq!(build_config(&c).expl_cfg.max_conseq_failed_attempts, Some(7));

        // shrink_step: non-finite falls back to the default
        let def = DEFAULT_SWARM_CONFIG.expl_cfg.shrink_step;
        for v in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            c.exploration.shrink_step = v;
            assert_eq!(build_config(&c).expl_cfg.shrink_step, def);
        }
        c.exploration.shrink_step = 0.0025;
        assert_eq!(build_config(&c).expl_cfg.shrink_step, 0.0025);

        // n_workers is capped
        c.exploration.separator.n_workers = i32::MAX;
        assert_eq!(
            build_config(&c).expl_cfg.separator_config.n_workers,
            MAX_SEPARATOR_WORKERS
        );
    }

    #[test]
    fn c_duration_secs_never_yields_an_instant_overflowing_duration() {
        for v in [f32::MAX, 1.0e15] {
            let d = c_duration_secs(v);
            assert_eq!(d, MAX_FINITE_DURATION);
            let _ = std::time::Instant::now() + d; // must not panic
        }
        assert_eq!(c_duration_secs(f32::NAN), Duration::ZERO);
        assert_eq!(c_duration_secs(f32::INFINITY), Duration::ZERO);
        assert_eq!(c_duration_secs(-1.0), Duration::ZERO);
        assert_eq!(c_duration_secs(1.5), Duration::from_secs_f32(1.5));
    }

    #[test]
    fn mm_simplification_is_global_exclusive_and_default_resettable() {
        let mut c: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c);
        c.refplace.enabled = 0;
        c.refplace.simplify_max_deviation_mm = 0.3;
        let cfg = build_config(&c);
        assert_eq!(cfg.simplify_max_deviation_mm, Some(0.3));
        assert_eq!(cfg.poly_simpl_tolerance, None);
        assert!(!cfg.refplace.enabled);
        swarm_get_default_config(&mut c);
        assert_eq!(c.refplace.simplify_max_deviation_mm, 0.0);
        assert_eq!(build_config(&c).simplify_max_deviation_mm, None);
        for invalid in [f32::NAN, f32::INFINITY, -1.0] {
            c.refplace.simplify_max_deviation_mm = invalid;
            assert_eq!(build_config(&c).simplify_max_deviation_mm, None);
        }
    }

    /// 분리 시간 상한: `0` 은 꺼짐, 양수는 고정 상한, `-1` 만 적응형이다.
    #[test]
    fn separation_time_cap_modes_cross_the_abi() {
        let mut c: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c);
        let off = build_config(&c).expl_cfg;
        assert!(off.sep_time_cap.is_zero() && !off.sep_cap_adaptive);

        c.search.sep_time_cap_ms = 12_000;
        let fixed = build_config(&c).expl_cfg;
        assert_eq!(fixed.sep_time_cap, std::time::Duration::from_secs(12));
        assert!(!fixed.sep_cap_adaptive);

        c.search.sep_time_cap_ms = SEP_TIME_CAP_ADAPTIVE;
        let adaptive = build_config(&c).expl_cfg;
        assert!(adaptive.sep_time_cap.is_zero() && adaptive.sep_cap_adaptive);

        // 그 밖의 음수는 예전 호출자가 «제한 없음» 으로 넘기던 값이다.
        c.search.sep_time_cap_ms = -5;
        let legacy = build_config(&c).expl_cfg;
        assert!(legacy.sep_time_cap.is_zero() && !legacy.sep_cap_adaptive);
    }

    /// ABI v17 꼬리: 원단 경계 여유와 GLS 학습률이 코어까지 **실제로** 도달하는지.
    ///
    /// 두 블록 모두 `CExplorationConfig` 바깥의 꼬리에서 오므로 합류 대입을 빠뜨리면
    /// 조용히 죽는다. `shrink_feedback` 이 정확히 그렇게 죽어 있었다.
    #[test]
    fn fabric_edge_and_gls_knobs_cross_the_abi() {
        let mut c: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c);
        let gls_def = crate::optimizer::separator::GlsWeightConfig::DEFAULT;

        // 기본값(전부 0)은 v16 과 같은 동작이어야 한다.
        let base = build_config(&c);
        assert_eq!(base.fabric_edge_margin_mm, None);
        assert_eq!(base.expl_cfg.separator_config.gls, gls_def);
        assert_eq!(base.cmpr_cfg.separator_config.gls, gls_def);

        // 원단 경계 여유는 양수일 때만 켜진다.
        c.fabric_edge.margin_mm = 0.1;
        assert_eq!(build_config(&c).fabric_edge_margin_mm, Some(0.1));
        for invalid in [f32::NAN, f32::INFINITY, -1.0, 0.0] {
            c.fabric_edge.margin_mm = invalid;
            assert_eq!(build_config(&c).fabric_edge_margin_mm, None);
        }

        // GLS 는 탐색 단계에만 얹히고 압축은 기본값을 유지한다.
        c.gls.min_inc = 1.02;
        c.gls.max_inc = 1.10;
        c.gls.decay = 0.99;
        c.gls.max_weight = 1000.0;
        let on = build_config(&c);
        assert_eq!(
            on.expl_cfg.separator_config.gls,
            crate::optimizer::separator::GlsWeightConfig {
                min_inc: 1.02,
                max_inc: 1.10,
                decay: 0.99,
                max_weight: 1000.0,
            }
        );
        assert_eq!(on.cmpr_cfg.separator_config.gls, gls_def);

        // 범위를 벗어난 항목만 각자 기본값으로 되돌아간다.
        c.gls.min_inc = 0.5; // <= 1 은 발산하지 않는 학습이 아니다
        c.gls.max_inc = 1.0; // min_inc 미만
        c.gls.decay = 2.0; // 1 초과
        c.gls.max_weight = 0.5; // 1 이하
        assert_eq!(build_config(&c).expl_cfg.separator_config.gls, gls_def);
    }

    /// ABI v17 꼬리: 구간 통째 재배열 노브가 코어 설정까지 **실제로** 도달하는지.
    ///
    /// 이 블록은 `CExplorationConfig` 가 아니라 구조체 끝의 별도 꼬리에서 오므로,
    /// 합류시키는 대입을 빠뜨리면 체크박스가 조용히 죽는다. `shrink_feedback` 이
    /// 정확히 그렇게 죽어 있었다. 값·경계·기본값 셋을 한 테스트에서 고정한다.
    #[test]
    fn band_rearrange_knobs_cross_the_abi() {
        let mut c: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c);
        let def = DEFAULT_SWARM_CONFIG.expl_cfg.rnr;

        // 기본값(전부 0)은 v16 과 같은 동작이어야 한다.
        let base = build_config(&c).expl_cfg.rnr;
        assert!(!base.post_shrink_rearrange);
        assert!(!base.full_band);
        assert_eq!(base.recreate_bias, def.recreate_bias);
        assert_eq!(base.min_clear_reinsert_ratio, def.min_clear_reinsert_ratio);

        // 켜면 코어까지 간다.
        c.band_rearrange.post_shrink = 1;
        c.band_rearrange.full_band = 1;
        c.band_rearrange.recreate_x_mult = 1.0;
        c.band_rearrange.recreate_y_mult = 10.0;
        c.band_rearrange.min_clear_reinsert_ratio = 0.85;
        let on = build_config(&c).expl_cfg.rnr;
        assert!(on.post_shrink_rearrange);
        assert!(on.full_band);
        assert_eq!(on.recreate_bias, (1.0, 10.0));
        assert_eq!(on.min_clear_reinsert_ratio, 0.85);

        // 한쪽만 양수인 편향은 무시하고 기본값으로 떨어진다.
        c.band_rearrange.recreate_y_mult = 0.0;
        assert_eq!(
            build_config(&c).expl_cfg.rnr.recreate_bias,
            def.recreate_bias
        );

        // 비정상 비율은 검사 없음(기본값)으로 떨어지고, 1 초과는 1 로 clamp 된다.
        for invalid in [f32::NAN, f32::INFINITY, -1.0, 0.0] {
            c.band_rearrange.min_clear_reinsert_ratio = invalid;
            assert_eq!(
                build_config(&c).expl_cfg.rnr.min_clear_reinsert_ratio,
                def.min_clear_reinsert_ratio
            );
        }
        c.band_rearrange.min_clear_reinsert_ratio = 2.0;
        assert_eq!(build_config(&c).expl_cfg.rnr.min_clear_reinsert_ratio, 1.0);
    }

    /// 기본값 내보내기(swarm_get_default_config) → 재변환(build_config) 왕복이
    /// Rust 기본 설정과 일치하는지 확인한다. 새 필드 배선 누락을 잡는 안전망.
    #[test]
    fn default_config_roundtrip() {
        let mut c_cfg: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c_cfg);
        let cfg = build_config(&c_cfg);
        let def = DEFAULT_SWARM_CONFIG;

        // capmods/adaptive 관련 신규 필드
        assert_eq!(cfg.expl_cfg.rnr.period, def.expl_cfg.rnr.period);
        assert_eq!(
            cfg.expl_cfg.rnr.band_fraction,
            def.expl_cfg.rnr.band_fraction
        );
        assert_eq!(cfg.expl_cfg.rnr.max_items, def.expl_cfg.rnr.max_items);
        assert_eq!(
            cfg.expl_cfg.adaptive_shrink_gain,
            def.expl_cfg.adaptive_shrink_gain
        );
        assert_eq!(
            cfg.expl_cfg.adaptive_shrink_max_mult,
            def.expl_cfg.adaptive_shrink_max_mult
        );
        assert_eq!(
            cfg.expl_cfg.osc_fail_threshold,
            def.expl_cfg.osc_fail_threshold
        );
        assert_eq!(cfg.expl_cfg.osc_expand_steps, def.expl_cfg.osc_expand_steps);
        assert_eq!(cfg.expl_cfg.alns_decay, def.expl_cfg.alns_decay);
        assert_eq!(cfg.expl_cfg.alns_min_weight, def.expl_cfg.alns_min_weight);
        for (got, want) in [
            (
                &cfg.expl_cfg.separator_config,
                &def.expl_cfg.separator_config,
            ),
            (
                &cfg.cmpr_cfg.separator_config,
                &def.cmpr_cfg.separator_config,
            ),
        ] {
            assert_eq!(got.swap_period, want.swap_period);
            assert_eq!(got.swap_attempts, want.swap_attempts);
            assert_eq!(got.swap_area_ratio, want.swap_area_ratio);
            assert_eq!(
                got.sample_config.left_bias_gain,
                want.sample_config.left_bias_gain
            );
            assert_eq!(
                got.sample_config.n_container_samples,
                want.sample_config.n_container_samples
            );
        }

        // 기존 필드 몇 개도 왕복 확인
        assert_eq!(cfg.expl_cfg.shrink_step, def.expl_cfg.shrink_step);
        assert_eq!(cfg.cmpr_cfg.shrink_range, def.cmpr_cfg.shrink_range);
        assert_eq!(cfg.structure.enabled, def.structure.enabled);
        assert_eq!(cfg.structure.motif.enabled, def.structure.motif.enabled);
        assert_eq!(
            cfg.structure.max_active_motifs,
            def.structure.max_active_motifs
        );
        assert_eq!(
            cfg.structure.construction_starts,
            def.structure.construction_starts
        );
        assert_eq!(cfg.structure.profile_top_k, def.structure.profile_top_k);
        assert_eq!(cfg.structure.p2_time_limit, def.structure.p2_time_limit);
        assert_eq!(cfg.structure.p2_shrink_range, def.structure.p2_shrink_range);
        assert_eq!(cfg.structure.p3_time_limit, def.structure.p3_time_limit);
        assert_eq!(cfg.structure.p4_cycles, def.structure.p4_cycles);
        assert_eq!(
            cfg.structure.p4_cycle_time_limit,
            def.structure.p4_cycle_time_limit
        );
        assert_eq!(
            cfg.structure.p5_snap_rotations,
            def.structure.p5_snap_rotations
        );
    }

    #[test]
    fn shrink_ranges_accept_natural_and_legacy_field_orders() {
        let mut natural: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut natural);
        natural.compression.shrink_range_min = 0.00001;
        natural.compression.shrink_range_max = 0.0005;
        natural.structure_first.p2_shrink_range_min = 0.00005;
        natural.structure_first.p2_shrink_range_max = 0.003;

        let mut legacy = natural;
        legacy.compression.shrink_range_min = 0.0005;
        legacy.compression.shrink_range_max = 0.00001;
        legacy.structure_first.p2_shrink_range_min = 0.003;
        legacy.structure_first.p2_shrink_range_max = 0.00005;

        let natural = build_config(&natural);
        let legacy = build_config(&legacy);
        assert_eq!(natural.cmpr_cfg.shrink_range, (0.0005, 0.00001));
        assert_eq!(natural.cmpr_cfg.shrink_range, legacy.cmpr_cfg.shrink_range);
        assert_eq!(natural.structure.p2_shrink_range, (0.003, 0.00005));
        assert_eq!(
            natural.structure.p2_shrink_range,
            legacy.structure.p2_shrink_range
        );
    }

    #[test]
    fn explicit_structure_config_maps_to_rust() {
        let mut c_cfg: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c_cfg);
        c_cfg.structure_first = CStructureFirstConfig {
            enabled: 1,
            motif_enabled: 0,
            max_active_motifs: 17,
            construction_starts: 4,
            profile_row_height_ratio: 0.2,
            profile_top_k: 5,
            p2_time_limit_secs: 1.5,
            p2_shrink_range_min: 0.004,
            p2_shrink_range_max: 0.0001,
            p2_failure_decay: 0.6,
            p3_slack_ratio: 0.08,
            p3_time_limit_secs: 2.5,
            p4_cycles: 2,
            p4_cycle_time_limit_secs: 3.5,
            p4_ruin_fraction: 0.3,
            p4_max_items: 23,
            p4_stagnation_cycles: 1,
            p5_snap_rotations: 0,
            p1_time_limit_secs: 1.25,
            profile_max_rows: 320,
            item_beam_width: 9,
            candidate_pool_cap: 72,
            slide_refine_steps: 18,
            p1_include_legacy_baseline: 1,
            pair_max_offset_candidates: 128,
            pair_max_orientation_work: 1024,
            diagnostics_max_raster_cells: 100_000,
            diagnostics_max_pair_comparisons: 80_000,
            progress_callback_interval_ms: 1_200,
        };

        let structure = build_config(&c_cfg).structure;
        assert!(structure.enabled);
        assert!(!structure.motif.enabled);
        assert_eq!(structure.max_active_motifs, 17);
        assert_eq!(structure.construction_starts, 4);
        assert_eq!(structure.profile_row_height_ratio, 0.2);
        assert_eq!(structure.profile_top_k, 5);
        assert_eq!(structure.p1_time_limit, Duration::from_secs_f32(1.25));
        assert_eq!(structure.profile_max_rows, 320);
        assert_eq!(structure.item_beam_width, 9);
        assert_eq!(structure.candidate_pool_cap, 72);
        assert_eq!(structure.slide_refine_steps, 18);
        assert!(structure.p1_include_legacy_baseline);
        assert_eq!(structure.motif.pair_max_offset_candidates, 128);
        assert_eq!(structure.motif.pair_max_orientation_work, 1024);
        assert_eq!(
            structure.progress_callback_interval,
            Duration::from_millis(1_200)
        );
        assert_eq!(structure.p2_time_limit, Duration::from_secs_f32(1.5));
        assert_eq!(structure.p2_shrink_range, (0.004, 0.0001));
        assert_eq!(structure.p2_failure_decay, 0.6);
        assert_eq!(structure.p3_slack_ratio, 0.08);
        assert_eq!(structure.p3_time_limit, Some(Duration::from_secs_f32(2.5)));
        assert_eq!(structure.p4_cycles, 2);
        assert_eq!(structure.p4_cycle_time_limit, Duration::from_secs_f32(3.5));
        assert_eq!(structure.p4_ruin_fraction, 0.3);
        assert_eq!(structure.p4_max_items, 23);
        assert_eq!(structure.p4_stagnation_cycles, 1);
        assert!(!structure.p5_snap_rotations);
    }

    #[test]
    fn structure_progress_callback_interval_has_a_500_ms_floor() {
        let mut c_cfg: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c_cfg);
        assert_eq!(c_cfg.structure_first.progress_callback_interval_ms, 2_000);
        assert_eq!(
            build_config(&c_cfg).structure.progress_callback_interval,
            Duration::from_secs(2)
        );

        c_cfg.structure_first.progress_callback_interval_ms = 100;
        assert_eq!(
            build_config(&c_cfg).structure.progress_callback_interval,
            Duration::from_millis(500)
        );

        c_cfg.structure_first.progress_callback_interval_ms = 500;
        assert_eq!(
            build_config(&c_cfg).structure.progress_callback_interval,
            Duration::from_millis(500)
        );

        c_cfg.structure_first.progress_callback_interval_ms = 1_250;
        assert_eq!(
            build_config(&c_cfg).structure.progress_callback_interval,
            Duration::from_millis(1_250)
        );
    }

    #[test]
    fn the_last_abi_block_is_flush_with_the_struct_end() {
        assert_eq!(std::mem::size_of::<CStructureFirstConfig>(), 29 * 4);
        assert_eq!(std::mem::size_of::<CCallbackOptionsConfig>(), 2 * 4);
        assert_eq!(std::mem::size_of::<CInitialPlacementConfig>(), 17 * 4);
        assert_eq!(std::mem::size_of::<CWidthReliefConfig>(), 6 * 4);
        assert_eq!(std::mem::size_of::<CShrinkFeedbackConfig>(), 2 * 4);
        assert_eq!(std::mem::size_of::<CEscapeConfig>(), 9 * 4);
        assert_eq!(std::mem::size_of::<CNfpLbfConfig>(), 7 * 4);
        assert_eq!(std::mem::size_of::<CFabricToleranceConfig>(), 4);
        assert_eq!(std::mem::size_of::<CNfpIntegrationConfig>(), 5 * 4);
        assert_eq!(std::mem::size_of::<CNfpLbfTuningConfig>(), 4 * 4);
        assert_eq!(std::mem::size_of::<CNfpStrategyConfig>(), 12 * 4);
        assert_eq!(std::mem::size_of::<CLpSolverConfig>(), 4 * 4);
        // 기존(ABI v2) 필드 오프셋은 그대로 보존된다.
        assert_eq!(std::mem::offset_of!(CSwarmConfig, structure_first), 288);
        assert_eq!(std::mem::offset_of!(CSwarmConfig, callback_options), 404);
        // ABI v3 tail — 오프셋 유지.
        assert_eq!(std::mem::offset_of!(CSwarmConfig, initial_placement), 412);
        // ABI v4 tail — 오프셋 유지.
        assert_eq!(std::mem::offset_of!(CSwarmConfig, width_relief), 480);
        // ABI v5 tail — 오프셋 유지.
        assert_eq!(std::mem::offset_of!(CSwarmConfig, shrink_feedback), 504);
        // ABI v6 tail.
        assert_eq!(std::mem::offset_of!(CSwarmConfig, escape), 512);
        assert_eq!(std::mem::offset_of!(CSwarmConfig, nfp_lbf), 548);
        // ABI v7 tail — 원단 허용오차.
        assert_eq!(std::mem::offset_of!(CSwarmConfig, fabric_tolerance), 576);
        // ABI v8 tail — 탐색 단계 NFP 활용. 오프셋 유지.
        assert_eq!(std::mem::offset_of!(CSwarmConfig, nfp_integration), 580);
        // ABI v9 tail — NFP-LBF 탐침 예산. 오프셋 유지.
        assert_eq!(std::mem::offset_of!(CSwarmConfig, nfp_lbf_tuning), 600);
        // ABI v10 tail — 초기 배치 전략 7종 + LP/QP 솔버 선택. **오프셋 유지.**
        assert_eq!(std::mem::offset_of!(CSwarmConfig, nfp_strategy), 616);
        assert_eq!(std::mem::offset_of!(CSwarmConfig, lp_solver), 664);
        // ABI v11 tail — 전략 ⑧ 누적 배치 / ⑨ 꼬리 회수.
        //
        // `CNfpStrategyConfig`(616, 48B)를 늘리지 않고 **새 블록**으로 붙였다.
        // 중간 블록을 늘렸다면 `lp_solver` 가 664 에서 밀려 v10 호스트가 조용히 깨진다.
        assert_eq!(std::mem::size_of::<CNfpStrategy2Config>(), 4 * 4);
        assert_eq!(std::mem::offset_of!(CSwarmConfig, nfp_strategy2), 680);
        // ABI v12 tail — 레퍼런스 설계 엔진. 앞 블록 오프셋은 전부 그대로다.
        assert_eq!(std::mem::size_of::<CRefPlaceConfig>(), 4 * 4);
        assert_eq!(std::mem::offset_of!(CSwarmConfig, refplace), 696);
        // ABI v13 tail — LBF 배치 순서. 앞 블록 오프셋은 전부 그대로다.
        assert_eq!(std::mem::size_of::<CLbfOrderConfig>(), 4 * 4);
        assert_eq!(std::mem::offset_of!(CSwarmConfig, lbf_order), 712);
        // ABI v14 tail — 홀 우선 부착. 앞 블록 오프셋은 전부 그대로다.
        assert_eq!(std::mem::size_of::<CHoleFirstConfig>(), 4 * 4);
        assert_eq!(std::mem::offset_of!(CSwarmConfig, hole_first), 728);
        // ABI v15 tail — 잔여 예산 회수. 앞 블록 오프셋은 전부 그대로다.
        assert_eq!(std::mem::size_of::<CReclaimConfig>(), 8 * 4);
        assert_eq!(std::mem::offset_of!(CSwarmConfig, reclaim), 744);
        assert_eq!(std::mem::size_of::<CSeparationRepairConfig>(), 36);
        assert_eq!(std::mem::offset_of!(CSwarmConfig, repair), 776);
        // ABI v17 tail — 구간 통째 재배열 (2026-09-14 에 5 -> 10 필드로 늘었다).
        // 앞 블록 오프셋은 전부 그대로다.
        assert_eq!(std::mem::size_of::<CBandRearrangeConfig>(), 10 * 4);
        assert_eq!(std::mem::offset_of!(CSwarmConfig, band_rearrange), 812);
        // ABI v17 tail — 원단 경계 여유. 앞 블록 오프셋은 전부 그대로다.
        assert_eq!(std::mem::size_of::<CFabricEdgeConfig>(), 4);
        assert_eq!(std::mem::offset_of!(CSwarmConfig, fabric_edge), 852);
        // ABI v17 tail — GLS 가중치 학습률. 앞 블록 오프셋은 전부 그대로다.
        assert_eq!(std::mem::size_of::<CGlsWeightConfig>(), 4 * 4);
        assert_eq!(std::mem::offset_of!(CSwarmConfig, gls), 856);
        // ABI v17 tail — 구성기 배치 규칙. 앞 블록 오프셋은 전부 그대로다.
        assert_eq!(std::mem::size_of::<CConstructorConfig>(), 11 * 4);
        assert_eq!(std::mem::offset_of!(CSwarmConfig, constructor), 872);
        // ABI v18 tail — 탐색 정체 대응. 앞 블록 오프셋은 전부 그대로다.
        assert_eq!(std::mem::size_of::<CSearchTuningConfig>(), 8 * 4);
        assert_eq!(std::mem::offset_of!(CSwarmConfig, search), 916);
        // ABI v19 tail — 원본 겹침 초과 처리. 정렬 여유에 들어가 크기는 그대로다.
        assert_eq!(std::mem::size_of::<COriginalOverlapConfig>(), 4);
        assert_eq!(std::mem::offset_of!(CSwarmConfig, original_overlap), 948);
        // ABI v20 tail — place group 처리 방식. 구조체가 952 → 960 으로 늘어난다(정렬 8).
        assert_eq!(std::mem::size_of::<CPlaceGroupConfig>(), 4);
        assert_eq!(std::mem::offset_of!(CSwarmConfig, place_group), 952);
        assert_eq!(std::mem::size_of::<CSwarmConfig>(), 960);
        let tail_end = std::mem::offset_of!(CSwarmConfig, place_group)
            + std::mem::size_of::<CPlaceGroupConfig>();
        assert!(tail_end <= std::mem::size_of::<CSwarmConfig>());
        assert!(
            std::mem::size_of::<CSwarmConfig>() - tail_end
                < std::mem::align_of::<CSwarmConfig>()
        );
    }

    /// ABI v14 tail: 홀 우선 부착이 코어 설정까지 **실제로** 도달하는지.
    ///
    /// 노브가 `lbf_raster`(중간 블록)가 아니라 꼬리 블록에서 오므로, 합류시키는
    /// 대입을 빠뜨리면 체크박스가 조용히 죽는다. 소스 문자열이 아니라
    /// [`build_config`] 의 결과를 본다.
    #[test]
    fn hole_first_reaches_the_raster_config() {
        let mut c_cfg: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c_cfg);
        // 기본은 꺼짐 — 켜지 않은 호스트의 결과가 바뀌면 안 된다.
        assert_eq!(c_cfg.hole_first.enabled, 0);
        let base = build_config(&c_cfg);
        assert!(!base.lbf_raster.hole_first);

        c_cfg.hole_first.enabled = 1;
        c_cfg.hole_first.share = 0.25;
        let on = build_config(&c_cfg);
        assert!(on.lbf_raster.hole_first, "켬이 코어까지 전달되지 않았다");
        assert_eq!(on.lbf_raster.hole_share, 0.25);
        // 중간 블록의 기존 값은 그대로여야 한다.
        assert_eq!(on.lbf_raster.n_candidates, base.lbf_raster.n_candidates);
        assert_eq!(on.lbf_raster.cell_ratio, base.lbf_raster.cell_ratio);

        // share 가 0 이하면 코어 기본값으로. zeroed 꼬리(구 호스트)도 현행 동작으로.
        c_cfg.hole_first.share = 0.0;
        let def_share = DEFAULT_SWARM_CONFIG.lbf_raster.hole_share;
        assert_eq!(build_config(&c_cfg).lbf_raster.hole_share, def_share);
        c_cfg.hole_first = unsafe { std::mem::zeroed() };
        assert!(!build_config(&c_cfg).lbf_raster.hole_first);
    }

    /// ABI v13 tail: LBF 순서 모드가 코어 설정까지 **실제로** 도달하는지.
    ///
    /// 소스 문자열이 아니라 [`build_config`] 의 결과를 본다 — 문자열 단정은
    /// "대입은 있는데 뒤에서 덮어쓴다" 를 못 잡는다(실제로 겪었다).
    #[test]
    fn lbf_order_mode_reaches_the_core_config() {
        use swarm_core::optimizer::lbf::LbfOrderMode;
        let mut c_cfg: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c_cfg);
        // 기본값은 현행 동작이어야 한다 — 켜지 않은 호스트의 결과가 바뀌면 안 된다.
        assert_eq!(c_cfg.lbf_order.mode, 0);
        assert_eq!(build_config(&c_cfg).lbf_order, LbfOrderMode::LargestFirst);

        for (raw, expected) in [
            (1, LbfOrderMode::SmallestFirst),
            (2, LbfOrderMode::MiddleOutLargeFirst),
            (3, LbfOrderMode::MiddleOutSmallFirst),
            (4, LbfOrderMode::ConcavityFirst),
            (5, LbfOrderMode::SizeTimesConcavity),
        ] {
            c_cfg.lbf_order.mode = raw;
            assert_eq!(
                build_config(&c_cfg).lbf_order,
                expected,
                "mode={raw} 가 코어까지 전달되지 않았다"
            );
        }

        // 범위 밖과 zeroed tail(구 호스트)은 둘 다 현행 동작으로.
        c_cfg.lbf_order.mode = 77;
        assert_eq!(build_config(&c_cfg).lbf_order, LbfOrderMode::LargestFirst);
        c_cfg.lbf_order = unsafe { std::mem::zeroed() };
        assert_eq!(build_config(&c_cfg).lbf_order, LbfOrderMode::LargestFirst);
    }

    /// ABI v6 tail: 두 블록 모두 경계에서는 opt-in이고, 0/음수 필드는 코어 기본값으로 돌아간다.
    #[test]
    fn escape_and_nfp_lbf_default_off_and_zero_fields_fall_back() {
        let mut c_cfg: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c_cfg);
        assert_eq!(c_cfg.escape.enabled, 0);
        assert_eq!(c_cfg.nfp_lbf.enabled, 0);

        // 전부 0인 tail(구버전 호출자가 zeroed 버퍼를 넘긴 경우)도 기본값으로 복원된다.
        c_cfg.escape = unsafe { std::mem::zeroed() };
        c_cfg.nfp_lbf = unsafe { std::mem::zeroed() };
        let config = build_config(&c_cfg);
        let escape_def = crate::config::DEFAULT_ADAPTIVE_ESCAPE_CONFIG;
        let nfp_def = crate::config::DEFAULT_NFP_LBF_SEED_CONFIG;
        assert!(!config.escape_enabled);
        assert_eq!(
            config.escape.stagnation.window_floor,
            escape_def.stagnation.window_floor
        );
        assert_eq!(
            config.escape.stagnation.efficiency_eps,
            escape_def.stagnation.efficiency_eps
        );
        assert_eq!(
            config.escape.conflict_burst.max_push_items,
            escape_def.conflict_burst.max_push_items
        );
        assert_eq!(config.escape.max_infeasible_pool, escape_def.max_infeasible_pool);
        assert!(!config.nfp_lbf.enabled);
        assert_eq!(config.nfp_lbf.n_orderings, nfp_def.n_orderings);
        assert_eq!(config.nfp_lbf.cache_budget_ms, nfp_def.cache_budget_ms);
        assert_eq!(config.nfp_lbf.total_budget_ms, nfp_def.total_budget_ms);

        // 명시값은 그대로 실린다.
        c_cfg.escape.enabled = 1;
        c_cfg.escape.window_floor = 42;
        c_cfg.escape.enable_conflict_burst = 1;
        c_cfg.nfp_lbf.enabled = 1;
        c_cfg.nfp_lbf.n_orderings = 12;
        c_cfg.nfp_lbf.allow_180 = 1;
        let config = build_config(&c_cfg);
        assert!(config.escape_enabled);
        assert_eq!(config.escape.stagnation.window_floor, 42);
        assert!(config.escape.enable_conflict_burst);
        assert!(!config.escape.enable_swarm_diversification);
        assert!(config.nfp_lbf.enabled);
        assert_eq!(config.nfp_lbf.n_orderings, 12);
        assert!(config.nfp_lbf.allow_180);

        // 비유한/음수 실수는 조용히 통과하지 않는다.
        c_cfg.escape.efficiency_eps = f32::NAN;
        c_cfg.nfp_lbf.concave_simplify_tolerance = -1.0;
        let config = build_config(&c_cfg);
        assert_eq!(
            config.escape.stagnation.efficiency_eps,
            escape_def.stagnation.efficiency_eps
        );
        assert_eq!(
            config.nfp_lbf.concave_simplify_tolerance,
            nfp_def.concave_simplify_tolerance
        );
    }

    #[test]
    fn width_relief_defaults_are_off_and_zero_input_stays_off() {
        let mut c_cfg: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c_cfg);
        // 기본은 비활성 — 기존 폭 진동 동작이 그대로 유지된다.
        assert_eq!(c_cfg.width_relief.fail_threshold, 0);
        let def = DEFAULT_SWARM_CONFIG;
        let cfg = build_config(&c_cfg);
        assert_eq!(cfg.expl_cfg.relief_fail_threshold, 0);
        assert_eq!(cfg.expl_cfg.relief_step, def.expl_cfg.relief_step);
        assert_eq!(cfg.expl_cfg.relief_max_ratio, def.expl_cfg.relief_max_ratio);
        assert_eq!(cfg.expl_cfg.relief_max_rounds, def.expl_cfg.relief_max_rounds);
        assert_eq!(
            cfg.expl_cfg.relief_strike_limit,
            def.expl_cfg.relief_strike_limit
        );

        // 0/음수는 전부 기본값으로 보정된다 (구형 호출자가 tail을 0으로 남겨도 안전).
        c_cfg.width_relief = unsafe { std::mem::zeroed() };
        let cfg = build_config(&c_cfg);
        assert_eq!(cfg.expl_cfg.relief_fail_threshold, 0);
        assert_eq!(cfg.expl_cfg.relief_max_rounds, def.expl_cfg.relief_max_rounds);
        assert_eq!(cfg.expl_cfg.relief_strike_limit, 1);
        assert_eq!(cfg.expl_cfg.relief_round_budget, std::time::Duration::ZERO);
    }

    #[test]
    fn shrink_feedback_defaults_off_and_rejects_out_of_range_decay() {
        let mut c_cfg: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c_cfg);
        let def = DEFAULT_SWARM_CONFIG;

        // 기본은 비활성(감쇠 1.0) — 밀도 기반 적응만 남는다.
        assert_eq!(c_cfg.shrink_feedback.fail_decay, 1.0);
        let cfg = build_config(&c_cfg);
        assert_eq!(cfg.expl_cfg.shrink_fail_decay, 1.0);
        assert_eq!(cfg.expl_cfg.shrink_scale_min, def.expl_cfg.shrink_scale_min);

        // 켜면 그대로 전달된다.
        c_cfg.shrink_feedback.fail_decay = 0.6;
        c_cfg.shrink_feedback.scale_min = 0.1;
        let cfg = build_config(&c_cfg);
        assert!((cfg.expl_cfg.shrink_fail_decay - 0.6).abs() < 1.0e-7);
        assert!((cfg.expl_cfg.shrink_scale_min - 0.1).abs() < 1.0e-7);

        // 범위 밖 감쇠는 전부 비활성(1.0)으로 정규화된다 — 0이나 음수를 스텝에
        // 곱하면 축소가 멈추거나 폭이 커져 버린다.
        for bad in [0.0, -0.5, 1.0, 1.5, f32::NAN, f32::INFINITY] {
            c_cfg.shrink_feedback.fail_decay = bad;
            assert_eq!(
                build_config(&c_cfg).expl_cfg.shrink_fail_decay,
                1.0,
                "decay = {bad}"
            );
        }

        // 하한도 마찬가지로 정규화된다.
        c_cfg.shrink_feedback.fail_decay = 0.5;
        for bad in [0.0, -0.1, 1.5, f32::NAN] {
            c_cfg.shrink_feedback.scale_min = bad;
            assert_eq!(
                build_config(&c_cfg).expl_cfg.shrink_scale_min,
                def.expl_cfg.shrink_scale_min,
                "scale_min = {bad}"
            );
        }
    }

    #[test]
    fn valid_fail_decay_selects_a_feedback_shrink_mode() {
        // `fail_decay` 는 "(0,1)이면 적용된다"고 문서가 약속하는데, 그 계수를 읽는 쪽은
        // `uses_feedback()` 모드뿐이다. 모드가 C ABI에 없으므로 이 블록의 유효성으로
        // 모드를 고른다. 이 테스트가 그 연결을 못 박는다.
        let mut c_cfg: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c_cfg);

        // 기본(1.0 = 비활성)은 기존 정책 그대로 — 피드백을 읽지 않는 모드다.
        let mode = build_config(&c_cfg).expl_cfg.shrink_adapt_mode;
        assert_eq!(mode, DEFAULT_SWARM_CONFIG.expl_cfg.shrink_adapt_mode);
        assert!(!mode.uses_feedback());

        // (0,1) 이면 피드백을 읽는 모드가 선택된다.
        for good in [0.05_f32, 0.5, 0.6, 0.95] {
            c_cfg.shrink_feedback.fail_decay = good;
            let cfg = build_config(&c_cfg);
            assert!(
                cfg.expl_cfg.shrink_adapt_mode.uses_feedback(),
                "decay = {good} should enable feedback"
            );
            assert!((cfg.expl_cfg.shrink_fail_decay - good).abs() < 1.0e-7);
        }

        // 범위 밖은 감쇠와 모드가 함께 기존 정책으로 돌아간다.
        for bad in [0.0_f32, -0.5, 1.0, 1.5, f32::NAN, f32::INFINITY] {
            c_cfg.shrink_feedback.fail_decay = bad;
            let cfg = build_config(&c_cfg);
            assert_eq!(
                cfg.expl_cfg.shrink_adapt_mode,
                DEFAULT_SWARM_CONFIG.expl_cfg.shrink_adapt_mode,
                "decay = {bad}"
            );
            assert_eq!(cfg.expl_cfg.shrink_fail_decay, 1.0, "decay = {bad}");
        }
    }

    #[test]
    fn width_relief_accepts_explicit_values() {
        let mut c_cfg: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c_cfg);
        c_cfg.width_relief.fail_threshold = 2;
        c_cfg.width_relief.step = 0.004;
        c_cfg.width_relief.max_ratio = 0.05;
        c_cfg.width_relief.max_rounds = 20;
        c_cfg.width_relief.strike_limit = 2;
        c_cfg.width_relief.round_budget_ms = 250;

        let cfg = build_config(&c_cfg);
        assert_eq!(cfg.expl_cfg.relief_fail_threshold, 2);
        assert_eq!(
            cfg.expl_cfg.relief_round_budget,
            std::time::Duration::from_millis(250)
        );
        assert!((cfg.expl_cfg.relief_step - 0.004).abs() < 1.0e-7);
        assert!((cfg.expl_cfg.relief_max_ratio - 0.05).abs() < 1.0e-7);
        assert_eq!(cfg.expl_cfg.relief_max_rounds, 20);
        assert_eq!(cfg.expl_cfg.relief_strike_limit, 2);

        // 비정상 실수는 기본값으로 되돌린다.
        c_cfg.width_relief.step = f32::NAN;
        c_cfg.width_relief.max_ratio = -1.0;
        let cfg = build_config(&c_cfg);
        assert_eq!(cfg.expl_cfg.relief_step, DEFAULT_SWARM_CONFIG.expl_cfg.relief_step);
        assert_eq!(
            cfg.expl_cfg.relief_max_ratio,
            DEFAULT_SWARM_CONFIG.expl_cfg.relief_max_ratio
        );
    }

    #[test]
    fn initial_placement_defaults_roundtrip() {
        let mut c_cfg: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c_cfg);
        // 기본 C 값은 비활성 + 확정 조합이어야 한다.
        assert_eq!(c_cfg.initial_placement.enabled, 0);
        let s = InitialPlacementSettings::from_c(&c_cfg.initial_placement);
        assert!(!s.enabled);
        assert_eq!(s.rescue_after, 300);
        assert_eq!(s.stall_cap, 1500);
        assert!((s.cd_min - 0.03).abs() < 1e-6);
        assert_eq!(s.compact_passes, 6);
        assert_eq!(s.reseat_top, 4);

        // zeroed(전부 0) 입력은 "명시적 off" 필드만 0이고 나머지는 기본값으로 보정된다.
        let zeroed: CInitialPlacementConfig = unsafe { std::mem::zeroed() };
        let z = InitialPlacementSettings::from_c(&zeroed);
        assert!(!z.enabled);
        assert_eq!(z.kicks, 0);
        assert_eq!(z.rescue_after, 0);
        assert_eq!(z.stall_cap, 0);
        assert_eq!(z.iter_cap, 3000);
        assert!((z.time_limit_secs - 60.0).abs() < 1e-6);

        // 음수 = 기본값 복원.
        let mut neg = zeroed;
        neg.kicks = -1;
        neg.rescue_after = -1;
        neg.stall_cap = -1;
        neg.compact_passes = -1;
        neg.reseat_top = -1;
        let n = InitialPlacementSettings::from_c(&neg);
        assert_eq!(n.kicks, 100_000);
        assert_eq!(n.rescue_after, 300);
        assert_eq!(n.stall_cap, 1500);
        assert_eq!(n.compact_passes, 6);
        assert_eq!(n.reseat_top, 4);
    }

    #[test]
    fn callback_options_default_on_and_toggle_independently() {
        let mut c_cfg: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c_cfg);
        assert_eq!(build_callback_options(&c_cfg), DEFAULT_CALLBACK_OPTIONS);

        c_cfg.callback_options.collect_loss = 0;
        let options = build_callback_options(&c_cfg);
        assert!(!options.collect_loss);
        assert!(options.collect_collision_status);

        c_cfg.callback_options.collect_collision_status = 0;
        let options = build_callback_options(&c_cfg);
        assert!(!options.collect_loss);
        assert!(!options.collect_collision_status);
    }

    #[test]
    fn abi_info_reports_layout_and_compiled_features() {
        assert_eq!(swarm_get_abi_info(std::ptr::null_mut()), -1);

        let mut info = CSwarmAbiInfo {
            abi_version: 0,
            swarm_config_size: 0,
            structure_config_size: 0,
            feature_flags: 0,
        };
        assert_eq!(swarm_get_abi_info(&mut info), 0);
        assert_eq!(info.abi_version, SWARM_ABI_VERSION);
        assert_eq!(
            info.swarm_config_size as usize,
            std::mem::size_of::<CSwarmConfig>()
        );
        assert_eq!(
            info.structure_config_size as usize,
            std::mem::size_of::<CStructureFirstConfig>()
        );
        assert_eq!(info.feature_flags, compiled_feature_flags());
        assert_eq!(
            info.feature_flags & SWARM_FEATURE_SIMD != 0,
            cfg!(feature = "simd")
        );
        assert_eq!(
            info.feature_flags & SWARM_FEATURE_CAPMODS != 0,
            cfg!(feature = "capmods")
        );
        assert_eq!(
            info.feature_flags & SWARM_FEATURE_ADAPTIVE != 0,
            cfg!(feature = "adaptive")
        );
        assert_eq!(
            info.feature_flags & SWARM_FEATURE_STRUCTURE_FIRST != 0,
            cfg!(feature = "structure-first")
        );
        assert_eq!(
            info.feature_flags & SWARM_FEATURE_JNI != 0,
            cfg!(feature = "jni")
        );
    }

    /// 0/음수 등 유효하지 않은 값이 기본값으로 보정되는지 확인한다.
    #[test]
    fn invalid_values_fall_back_to_defaults() {
        let mut c_cfg: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut c_cfg);

        c_cfg.exploration.separator.swap_period = -1; // 음수 → 기본값
        c_cfg.exploration.separator.swap_attempts = 0; // 0 이하 → 기본값
        c_cfg.exploration.separator.swap_area_ratio_min = -1.0;
        c_cfg.exploration.separator.left_bias_gain = -1.0;
        c_cfg.exploration.rnr_period = -1;
        c_cfg.exploration.rnr_band_fraction = 2.0; // 1 초과 → 기본값
        c_cfg.exploration.rnr_max_items = 0;
        c_cfg.exploration.adaptive_shrink_max_mult = 0.5; // 1 미만 → 기본값
        c_cfg.exploration.osc_fail_threshold = -1; // 음수 → 기본값
        c_cfg.exploration.osc_expand_steps = 0.0; // 0 이하 → 기본값
        c_cfg.exploration.alns_decay = 1.5; // 범위 밖 → 기본값
        c_cfg.exploration.alns_min_weight = -0.5; // 음수 → 기본값

        let cfg = build_config(&c_cfg);
        let def = DEFAULT_SWARM_CONFIG;
        assert_eq!(
            cfg.expl_cfg.separator_config.swap_period,
            def.expl_cfg.separator_config.swap_period
        );
        assert_eq!(
            cfg.expl_cfg.separator_config.swap_attempts,
            def.expl_cfg.separator_config.swap_attempts
        );
        assert_eq!(
            cfg.expl_cfg.separator_config.swap_area_ratio,
            def.expl_cfg.separator_config.swap_area_ratio
        );
        assert_eq!(
            cfg.expl_cfg.separator_config.sample_config.left_bias_gain,
            def.expl_cfg.separator_config.sample_config.left_bias_gain
        );
        assert_eq!(cfg.expl_cfg.rnr.period, def.expl_cfg.rnr.period);
        assert_eq!(
            cfg.expl_cfg.rnr.band_fraction,
            def.expl_cfg.rnr.band_fraction
        );
        assert_eq!(cfg.expl_cfg.rnr.max_items, def.expl_cfg.rnr.max_items);
        assert_eq!(
            cfg.expl_cfg.adaptive_shrink_max_mult,
            def.expl_cfg.adaptive_shrink_max_mult
        );
        assert_eq!(
            cfg.expl_cfg.osc_fail_threshold,
            def.expl_cfg.osc_fail_threshold
        );
        assert_eq!(cfg.expl_cfg.osc_expand_steps, def.expl_cfg.osc_expand_steps);
        assert_eq!(cfg.expl_cfg.alns_decay, def.expl_cfg.alns_decay);
        assert_eq!(cfg.expl_cfg.alns_min_weight, def.expl_cfg.alns_min_weight);

        // 0은 "비활성"으로 존중되어야 하는 필드
        c_cfg.exploration.separator.swap_period = 0;
        c_cfg.exploration.rnr_period = 0;
        c_cfg.exploration.separator.left_bias_gain = 0.0;
        c_cfg.exploration.adaptive_shrink_gain = 0.0;
        c_cfg.exploration.osc_fail_threshold = 0;
        c_cfg.exploration.alns_min_weight = 0.0;
        let cfg = build_config(&c_cfg);
        assert_eq!(cfg.expl_cfg.separator_config.swap_period, 0);
        assert_eq!(cfg.expl_cfg.rnr.period, 0);
        assert_eq!(
            cfg.expl_cfg.separator_config.sample_config.left_bias_gain,
            0.0
        );
        assert_eq!(cfg.expl_cfg.adaptive_shrink_gain, 0.0);
        assert_eq!(cfg.expl_cfg.osc_fail_threshold, 0);
        assert_eq!(cfg.expl_cfg.alns_min_weight, 0.0);

        c_cfg.exploration.separator.swap_attempts = i32::MAX;
        let cfg = build_config(&c_cfg);
        assert_eq!(
            cfg.expl_cfg.separator_config.swap_attempts,
            MAX_SWAP_ATTEMPTS_PER_PASS
        );
    }
}

