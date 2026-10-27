//! Swarm 최적화 파이프라인의 단계별 FFI API.
//!
//! 라이브러리가 노출하는 유일한 최적화 진입점이며, 단계별로 실행을 제어한다:
//!
//!   h = swarm_init(input_path) or swarm_init_from_json(input_json)    // 초기화 (LBF 초기해까지)
//!   swarm_alg_phase(h)              // 탐색 단계 실행 (선택)
//!   swarm_cp_phase(h)             // 압축 단계 실행
//!   swarm_refine_phase(h)               // 후처리 단계 실행 (선택)
//!   // 또는 init 직후 swarm_gp_phase(h)  // P0-P5 대체 파이프라인
//!
//! **탐색 단계는 건너뛸 수 있다.** `swarm_init` 직후 곧바로 `swarm_cp_phase`를
//! 호출하면 LBF 초기해에서 바로 압축이 시작된다 (LBF + Compress). 탐색이 시간 예산의
//! 대부분을 쓰므로, 빠른 결과가 필요하거나 초기 배치를 크게 흔들고 싶지 않을 때 쓴다.
//!
//!   json = swarm_get_solution_json(h)   // 최종 솔루션 JSON 가져오기
//!   report = swarm_get_structure_report_json(h) // structure 실행 감사 보고서
//!   swarm_free_string(json)             // ffi::swarm_free_string 재사용
//!   swarm_free(h)

// export 함수는 모두 C에서 호출되는 경계 shim이며 진입 시 포인터 null 검사를 한다. (ffi.rs와 동일)
#![allow(clippy::not_unsafe_ptr_arg_deref)]

// region: USING Module

// ffi 의 CStr/CString 사용 CStr 은 C++ 의 Const Char*, CString 은 C++ 의 Char* 혹은 std::string 에 대응한다.
use std::cell::{Cell, RefCell};
use std::ffi::{CStr, CString};
// C 의 Char 타입과 상호작용하기 위해 c_char 를 사용한다.
use std::os::raw::c_char;
use std::rc::Rc;

use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution};
use jagua_rs::collision_detection::hazards::filter::NoFilter;
use jagua_rs::probs::spp::io::ext_repr::{ExtSPInstance, ExtSPSolution};
use rand::rngs::Xoshiro256PlusPlus;
use rand::{Rng, SeedableRng};

use crate::config::*;
use crate::consts::*;
use crate::ffi::{CSwarmConfig, CallbackOptions, DEFAULT_CALLBACK_OPTIONS};
use crate::grouped_engine::GroupedStepState;
#[cfg(feature = "structure-first")]
use crate::grouped_engine::StructureProgressKind;
use crate::grouped_input;
use crate::optimizer::compress::compression_phase;
use crate::optimizer::lbf::LBFBuilder;
use crate::optimizer::polish::polish_phase_with_nfp;
use crate::optimizer::separator::Separator;
use crate::util::io;
use crate::util::listener::{ReportType, SolutionListener};
use crate::util::terminator::{BasicTerminator, Terminator};

use jagua_rs::Instant;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::time::Duration;

thread_local! {
    /// Initialization errors are thread-local because the ABI creates no
    /// handle on failure. Callers read this immediately after a NULL result.
    static LAST_ERROR: RefCell<(i32, String)> = const { RefCell::new((0, String::new())) };
}

#[cfg(all(test, feature = "structure-first"))]
thread_local! {
    // Test-only unwind injection stays inside the Rust catch boundary.
    static LEGACY_STRUCTURE_PANIC_AFTER_BASELINE: Cell<bool> = const { Cell::new(false) };
}

// Counts input reads and full input text-to-Value attempts only while a
// direct public Init regression enables this thread-local probe.
#[cfg(test)]
thread_local! {
    static ISSUE9_INPUT_COUNTS: Cell<Option<(usize, usize)>> = const { Cell::new(None) };
}

#[cfg(test)]
pub(crate) fn issue9_count_input_work(read: bool) {
    ISSUE9_INPUT_COUNTS.with(|slot| {
        if let Some((reads, values)) = slot.get() {
            slot.set(Some((reads + usize::from(read), values + usize::from(!read))));
        }
    });
}

pub(crate) fn clear_last_error() {
    LAST_ERROR.with(|slot| *slot.borrow_mut() = (0, String::new()));
}

/// A callback may re-enter the ABI while its outer phase still owns the
/// lifecycle slot. That nested call correctly reports `busy`, but the outer
/// phase must clear the thread-local diagnostic when it subsequently succeeds.
fn phase_success() -> i32 {
    clear_last_error();
    0
}

pub(crate) fn set_last_error(code: i32, message: impl Into<String>) {
    LAST_ERROR.with(|slot| *slot.borrow_mut() = (code, message.into()));
}

fn has_last_error() -> bool {
    LAST_ERROR.with(|slot| slot.borrow().0 != 0)
}

pub(crate) fn grouped_input_error_code(code: &grouped_input::InputErrorCode) -> i32 {
    use grouped_input::InputErrorCode::*;
    match code {
        NotV2 => 1000,
        UnsupportedSchema => 1001,
        InvalidJson => 1002,
        InvalidConfig => 1003,
        InvalidItem => 1004,
        InvalidHint => 1005,
        InvalidRegion => 1006,
        RegionItemDuplicated => 1007,
        RegionItemUnassigned => 1008,
        InvalidPlaceGroup => 1009,
        PlaceGroupCrossesRegions => 1010,
        PlaceGroupMemberPoseMissing => 1011,
    }
}

// endregion

/// 단계 상태 열거형
#[cfg_attr(not(feature = "structure-first"), allow(dead_code))]
#[derive(PartialEq)]
enum Phase {
    /// swarm_init 완료
    Init,
    /// swarm_alg_phase 완료
    Exploration,
    /// swarm_cp_phase 완료
    Compression,
    /// swarm_refine_phase 완료
    Polish,
    /// swarm_gp_phase 완료 (P0-P5)
    Structure,
}

/// 기존 단일 Solver의 단계별 최적화 상태.
///
/// `SwarmStepHandle` 자체에는 이 값을 직접 노출하지 않고 mutex 뒤에 둔다. 따라서
/// phase 실행과 결과 조회/설정 API가 동시에 같은 Rust 값에 `&mut`/`&`를 만드는 일을
/// 방지한다. 추후 grouped coordinator를 추가할 때도 handle 수명 관리는 그대로 재사용할
/// 수 있다.
#[cfg_attr(not(feature = "structure-first"), allow(dead_code))]
struct LegacyStepState {
    instance: SPInstance,
    prob: SPProblem,
    rng: Xoshiro256PlusPlus,
    expl_config: ExplorationConfig,
    cmpr_config: CompressionConfig,
    plsh_config: PolishConfig,
    structure_config: crate::structure::config::StructureFirstConfig,
    lbf_x_multiplier: f32,
    lbf_y_multiplier: f32,
    lbf_raster: RasterNfpConfig,
    /// LBF 배치 순서 모드. 기본은 현행 동작(큰 것부터).
    lbf_order: swarm_core::optimizer::lbf::LbfOrderMode,
    /// 정체 감지 기반 적응형 국소 최적해 탈출을 쓸지.
    escape_enabled: bool,
    /// 위 탈출 기법의 세부 설정.
    escape_config: swarm_core::escape::AdaptiveEscapeConfig,
    /// 탐색 단계 NFP 활용 범위 (순수 NFP 순서탐색 / 샘플러 / 분리벡터 / hole filling / 재삽입).
    nfp_integration: swarm_core::config::NfpIntegrationConfig,
    /// LP·QP 압축 백엔드 설정. 후처리(polish)의 LP 압축이 읽는다.
    ///
    /// 이 필드가 없던 동안 step API 는 기본값을 하드코딩하는 래퍼를 통해 언제나
    /// `DEFAULT_LP_SOLVER_CONFIG` 로 압축했다 — WPF 에서 Clarabel/Clp/Xpress 를 골라도
    /// microlp 순수 LP 가 돌았다. 함정 1(배선 누락)의 재발 사례였다.
    lp_solver: swarm_core::lp_solver::LpSolverConfig,
    /// NFP 기반 초기 배치(NFP-LBF) 시드 설정.
    nfp_lbf: swarm_core::config::NfpLbfSeedConfig,
    phase: Phase,
    solutions: Vec<SPSolution>,
    solution: Option<SPSolution>,
    solution_callback: Option<SolutionCallback>, // 결과 콜백 함수
    // Raw pointers are not Send. Store the opaque value as an integer while it is protected by
    // the mutex and cast it back only for the synchronous callback invocation.
    callback_user_data: usize,
    emit_intermediate_json: bool, // 중간 결과 JSON 보고 여부
    callback_options: CallbackOptions,
    frozen_item_ids: Vec<usize>, // 고정된 item id 목록
    /// 원단 밖으로 조각의 **원본** 외곽선이 넘어도 되는 최대 거리(mm). `None` = 검사 없음.
    ///
    /// 켜면 각 단계의 결과를 공개하기 직전에
    /// [`swarm_core::fabric_edge::repair`] 로 넘은 조각만 안쪽으로 민다.
    fabric_edge_margin_mm: Option<f32>,
    /// N06 초기배치 페이즈 설정 (기본값 보정 완료 상태).
    initial_config: crate::ffi::InitialPlacementSettings,
    /// Imported solutions or explicit placement hints are authoritative P1
    /// warm starts. A fresh legacy input leaves this false so structure-first
    /// can actually execute P0/P1 instead of reusing the legacy LBF preview.
    has_input_seed: bool,
    /// An explicitly selected constructor is also an exploration starting point,
    /// but is kept separate from authoritative imported placement hints.
    has_column_draft: bool,
    structure_report: Option<crate::structure::pipeline::StructureRunReport>,
}

/// V2 all-items/size-region coordinator state.  A single public FFI handle owns
/// every region lane and therefore also owns one callback registration and one
/// one-shot cancellation token.
struct GroupedHandleState {
    solver: GroupedStepState,
    solution_callback: Option<SolutionCallback>,
    callback_user_data: usize,
    emit_intermediate_json: bool,
    callback_options: CallbackOptions,
    progress_callback_interval: Duration,
}

/// 한 공개 handle이 보유하는 실행 엔진.
///
/// 현재는 legacy 엔진만 있지만 enum으로 두어 grouped coordinator를 같은 lifecycle 아래
/// 추가할 수 있게 한다.
enum HandleEngine {
    Legacy(LegacyStepState),
    Grouped(GroupedHandleState),
}

impl HandleEngine {
    fn legacy_mut(&mut self) -> &mut LegacyStepState {
        match self {
            Self::Legacy(state) => state,
            Self::Grouped(_) => panic!("{}", swarm_core::dmsg!("legacy state requested from a grouped handle")),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LifecycleStatus {
    Open,
    Closing,
}

#[derive(Debug)]
struct LifecycleState {
    status: LifecycleStatus,
    active_calls: usize,
    phase_running: bool,
    phase_thread: Option<std::thread::ThreadId>,
}

/// FFI handle의 실행/해제 수명을 조정한다.
///
/// Cancellation is one-shot for a handle: once requested it remains requested until the handle is
/// freed.
struct HandleLifecycle {
    run_control: Arc<swarm_core::config::runtime::RunControl>,
    stop_requested: Arc<AtomicBool>,
    state: Mutex<LifecycleState>,
    changed: Condvar,
}

impl HandleLifecycle {
    fn new() -> Self {
        Self {
            run_control: swarm_core::config::runtime::RunControl::from_defaults(),
            stop_requested: Arc::new(AtomicBool::new(false)),
            state: Mutex::new(LifecycleState {
                status: LifecycleStatus::Open,
                active_calls: 0,
                phase_running: false,
                phase_thread: None,
            }),
            changed: Condvar::new(),
        }
    }

    fn lock(&self) -> MutexGuard<'_, LifecycleState> {
        self.state
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    fn wait<'a>(&self, guard: MutexGuard<'a, LifecycleState>) -> MutexGuard<'a, LifecycleState> {
        self.changed
            .wait(guard)
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    fn begin_call(&self) -> Option<HandleCallGuard<'_>> {
        let mut state = self.lock();
        if state.status != LifecycleStatus::Open {
            return None;
        }
        state.active_calls += 1;
        Some(HandleCallGuard {
            lifecycle: self,
            phase: false,
        })
    }

    fn begin_idle_call(&self) -> Option<HandleCallGuard<'_>> {
        let mut state = self.lock();
        if state.status != LifecycleStatus::Open || state.phase_running {
            return None;
        }
        state.active_calls += 1;
        Some(HandleCallGuard {
            lifecycle: self,
            phase: false,
        })
    }

    fn begin_phase(&self) -> Option<HandleCallGuard<'_>> {
        let mut state = self.lock();
        if state.status != LifecycleStatus::Open
            || state.phase_running
            || self.stop_requested.load(Ordering::Acquire)
        {
            return None;
        }
        state.active_calls += 1;
        state.phase_running = true;
        state.phase_thread = Some(std::thread::current().id());
        Some(HandleCallGuard {
            lifecycle: self,
            phase: true,
        })
    }

    fn request_stop(&self) {
        self.stop_requested.store(true, Ordering::Release);
    }

    fn request_stop_and_wait(&self) {
        self.request_stop();
        let caller = std::thread::current().id();
        let mut state = self.lock();
        while state.phase_running {
            // A native callback runs on the phase thread. Waiting for that same thread here would
            // self-deadlock; the phase will observe the stop flag as soon as the callback returns.
            if state.phase_thread == Some(caller) {
                return;
            }
            state = self.wait(state);
        }
    }

    /// Reject new calls, request cancellation, and wait for all already-entered API calls.
    ///
    /// Returns `true` when every call has left the handle, so the caller may drop the containing
    /// `Box<SwarmStepHandle>`. Returns `false` when the caller is itself the thread running a
    /// phase (a native callback calling free): waiting for that thread here would self-deadlock,
    /// so the handle is marked closed and left to the caller to leak.
    fn close_and_wait(&self) -> bool {
        self.request_stop();
        let caller = std::thread::current().id();
        let mut state = self.lock();
        state.status = LifecycleStatus::Closing;
        while state.active_calls != 0 {
            if state.phase_running && state.phase_thread == Some(caller) {
                return false;
            }
            state = self.wait(state);
        }
        true
    }
}

struct HandleCallGuard<'a> {
    lifecycle: &'a HandleLifecycle,
    phase: bool,
}

impl Drop for HandleCallGuard<'_> {
    fn drop(&mut self) {
        let mut state = self.lifecycle.lock();
        if self.phase {
            state.phase_running = false;
            state.phase_thread = None;
        }
        state.active_calls = state.active_calls.saturating_sub(1);
        self.lifecycle.changed.notify_all();
    }
}

/// Swarm 단계별 최적화의 공개 opaque handle.
pub struct SwarmStepHandle {
    engine: Mutex<HandleEngine>,
    lifecycle: HandleLifecycle,
}

impl SwarmStepHandle {
    fn lock_engine(&self) -> MutexGuard<'_, HandleEngine> {
        self.engine
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }
}

/// ffi::SolutionCallback과 일치하는 콜백 타입.
///
/// report_type:
///   0 = 탐색 중 실행 가능한 해(ExplFeas)
///   1 = 탐색 중 실행 불가능(Infeasible) 보고(ExplInfeas)
///   2 = 탐색 중 개선된 해(ExplImproving)
///   3 = 압축(Cmpr) 단계의 실행 가능한 해(CmprFeas)
///   4 = 최종 결과(Final)
/// Structure-first 체크포인트는 P1/P3=0, P2=3, P4=2, P5=4로 매핑한다.
///
/// strip_width: 현재 솔루션의 스트립 폭(mm)
///
/// density: 현재 솔루션의 효율(0.0~1.0)
///
/// solution_json: 현재 솔루션의 JSON 문자열(CStr).  콜백이 실행되는 동안에만 유효합니다.  (NULL이면 JSON 해 정보 없이 스칼라 값만 보고됨)
///
/// user_data: swarm_set_callback에 넘겼던 그대로 돌려줌
///
/// solution_json:
///   콜백이 실행되는 동안에만 유효(NULL이면 JSON 정보 없이 스칼라 값만)
pub type SolutionCallback = extern "C" fn(i32, f32, f32, *const c_char, *mut std::ffi::c_void);

/// C 문자열 포인터를 Rust의 &str로 변환하며, 포인터가 NULL이거나 UTF-8이 유효하지 않은 경우에는 빈 문자열을 반환
unsafe fn cstr_to_str<'a>(ptr: *const c_char) -> &'a str {
    if ptr.is_null() {
        ""
    } else {
        unsafe { CStr::from_ptr(ptr) }.to_str().unwrap_or("")
    }
}

/// 현재 RNG를 Next 하고, 그 시점의 상태로 새 RNG를 하나 더 만든다. 각 phase가 독립된 랜덤 값을 유지하기 위한 방식.
fn next_rng(rng: &mut Xoshiro256PlusPlus) -> Xoshiro256PlusPlus {
    Xoshiro256PlusPlus::seed_from_u64(rng.next_u64())
}

/// Place 충돌 상태 구조체
/// # item_id: 아이템 ID (0-based)
/// # collision: 충돌 여부 (true = 충돌 있음, false = 충돌 없음)
#[repr(C)]
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize)]
pub struct CollisionStatus {
    pub item_id: usize,
    pub collision: bool,
}

/// C ABI용 충돌 상태 구조체.
#[repr(C)]
pub struct CCollisionStatus {
    pub item_id: usize,
    pub collision: u8,
}

#[derive(Default)]
struct CallbackDiagnostics {
    loss: Option<f32>,
    collision_status: Option<Vec<CollisionStatus>>,
}

/// Callback payload용 loss와 collision status를 한 번의 CDE 순회로 계산한다.
/// 둘 다 필요하지 않으면 호출자가 이 함수를 건너뛰어야 한다.
#[cfg(test)]
fn compute_callback_diagnostics(
    layout: &jagua_rs::entities::Layout,
    options: CallbackOptions,
) -> CallbackDiagnostics {
    compute_callback_diagnostics_with_frozen(layout, options, &[])
}

fn compute_callback_diagnostics_with_frozen(
    layout: &jagua_rs::entities::Layout,
    options: CallbackOptions,
    frozen_ids: &[usize],
) -> CallbackDiagnostics {
    compute_callback_diagnostics_with_groups(
        layout,
        options,
        frozen_ids,
        &crate::optimizer::place_groups::PlaceGroups::default(),
    )
}

fn compute_callback_diagnostics_with_groups(
    layout: &jagua_rs::entities::Layout,
    options: CallbackOptions,
    frozen_ids: &[usize],
    groups: &crate::optimizer::place_groups::PlaceGroups,
) -> CallbackDiagnostics {
    use crate::quantify::{quantify_collision_poly_container, quantify_collision_poly_poly};
    use jagua_rs::collision_detection::hazards::BasicHazardEntity;
    use jagua_rs::collision_detection::hazards::collector::{
        BasicHazardCollector, HazardCollector,
    };

    if !options.collect_loss && !options.collect_collision_status {
        return CallbackDiagnostics::default();
    }

    let mut total_loss = 0.0f32;
    let placed = layout.placed_items().iter().collect::<Vec<_>>();
    let mut collision_flags = vec![false; placed.len()];
    let mut collector = BasicHazardCollector::with_capacity(layout.placed_items().len() + 1);
    let container_bbox = layout.container().outer_cd().bbox();
    let frozen = frozen_ids
        .iter()
        .copied()
        .collect::<std::collections::HashSet<_>>();

    // 정적 hazard 만 아이템마다 질의한다. 조각쌍 hazard 는 여기서 일부러 버리고,
    // 아래의 **대칭** 직접 쌍 검사 한 번으로 대신한다.
    //
    // # 이 주석은 예전에 틀리게 적혀 있었다
    //
    // 원래 이유를 "Jagua 의 질의 방향이 한쪽 교차를 놓칠 수 있다(WSjk 59/6 회귀)"
    // 라고 적어 두었는데, 그 픽스처가 실제로 보여주는 것은 정반대다
    // (`lbf_evaluator.rs` 의 `wsjk_item_59_and_6_sub_tolerance_sliver_is_treated_as_contact`):
    //
    // ```text
    // CDE(item_6 를 hazard 로)          -> 충돌이라고 답한다
    // polygons_collide_bidirectionally  -> 양방향 모두 깨끗하다고 답한다
    // ```
    //
    // 그 쌍의 관통은 f32 반올림이 만든 약 0.000045mm 정점 슬라이버다. 즉 이 직접
    // 검사는 CDE 보다 **엄격한 게 아니라 관대**하다 — 공개 mm 계약의 접촉 허용오차
    // (`contact_tolerance`)를 적용하기 때문이다.
    //
    // 그래서 이 패스가 실제로 사는 이유는 두 가지다:
    //   1. **대칭성** — Jagua 의 다각형-hazard 질의는 방향에 따라 답이 갈리는데,
    //      직접 쌍 검사는 순서와 무관하게 같은 답을 낸다.
    //   2. **판정 일관성** — 인증기(`solution_validation_error`)와 같은 술어를 쓰므로,
    //      콜백이 "충돌"이라 말한 해는 인증에서도 반드시 거부된다.
    //
    // 대가는 반대 방향의 위험이다: CDE 가 잡은 미세 관통을 이 쪽이 놓칠 수 있다.
    // 실측(wsjk156, 탐색+압축+폴리시 전체, 실행가능 라벨 42건 / 탐색 단독 69건)에서
    // 두 오라클이 어긋난 사례는 0건이었다 — `swarm-core/tests/oracle_gap.rs`.
    for (index, (pk_a, pi_a)) in placed.iter().enumerate() {
        // Every part of a multipart item is queried; its own parts are dropped.
        collector.clear();
        for shape in pi_a.shapes() {
            layout
                .cde()
                .collect_poly_collisions(shape, &NoFilter, &mut collector);
        }
        collector.retain(|_, e| !matches!(e, BasicHazardEntity::PlacedItem { pk, .. } if *pk == *pk_a));

        let outside_strip = !crate::structure::pipeline::bbox_is_contained_mm(
            [
                container_bbox.x_min,
                container_bbox.x_max,
                container_bbox.y_min,
                container_bbox.y_max,
            ],
            [
                pi_a.shape().bbox().x_min,
                pi_a.shape().bbox().x_max,
                pi_a.shape().bbox().y_min,
                pi_a.shape().bbox().y_max,
            ],
        );

        for (_, entity) in collector.iter() {
            match entity {
                BasicHazardEntity::PlacedItem { .. } => {}
                BasicHazardEntity::Exterior => {
                    if outside_strip && !frozen.contains(&pi_a.item().idx()) {
                        collision_flags[index] = true;
                        if options.collect_loss {
                            total_loss += pi_a
                                .shapes()
                                .iter()
                                .map(|shape| quantify_collision_poly_container(shape, container_bbox))
                                .sum::<f32>();
                        }
                    }
                }
                BasicHazardEntity::Hole { .. } | BasicHazardEntity::InferiorQualityZone { .. } => {
                    collision_flags[index] = true;
                }
            }
        }
    }

    for first in 0..placed.len() {
        for second in first + 1..placed.len() {
            let first_item = placed[first].1;
            let second_item = placed[second].1;
            if groups.same_group(first_item.item().idx(), second_item.item().idx()) {
                continue;
            }
            if frozen.contains(&first_item.item().idx()) && frozen.contains(&second_item.item().idx()) {
                continue;
            }
            // Part by part: a multipart item's hull is only an envelope.
            let mut pair_collides = false;
            for shape_a in first_item.shapes() {
                for shape_b in second_item.shapes() {
                    if crate::eval::lbf_evaluator::polygons_collide_bidirectionally(shape_a, shape_b) {
                        pair_collides = true;
                        if options.collect_loss {
                            total_loss += quantify_collision_poly_poly(shape_a, shape_b);
                        }
                    }
                }
            }
            if pair_collides {
                collision_flags[first] = true;
                collision_flags[second] = true;
            }
        }
    }

    let statuses = options.collect_collision_status.then(|| {
        placed
            .iter()
            .zip(collision_flags)
            .map(|((_, item), collision)| CollisionStatus {
                item_id: item.item().idx(),
                collision,
            })
            .collect()
    });

    CallbackDiagnostics {
        loss: options.collect_loss.then_some(total_loss),
        collision_status: statuses,
    }
}

/// CallbackListener 구조체: SolutionCallback을 구현하며, 중간/최종 결과를 콜백함.
/// layout 필드는 raw pointer로 저장하여 borrow checker 충돌을 피함.
/// (explore/compress phase에서 sep의 불변 빌림과 가변 빌림이 동시에 필요하기 때문)
struct CallbackListener {
    frozen_ids: Vec<usize>,
    callback: SolutionCallback,
    user_data: *mut std::ffi::c_void,
    emit_json: bool,
    last_emit: Rc<Cell<Option<Instant>>>,
    min_interval: Duration,
    /// Structure-first exposes every feasible P1/P3 candidate as a complete JSON snapshot.
    /// Legacy Explore keeps its established contract where `ExplFeas` is scalar-only.
    emit_feasible_json: bool,
    /// Structure-first throttles the callback itself, not only JSON serialization. This avoids
    /// flooding consumers with scalar-only events between complete JSON checkpoints.
    throttle_all_reports: bool,
    callback_options: CallbackOptions,
    /// Structure-first never exposes an infeasible separator snapshot. Native
    /// infeasible/improving events merely drive a replay of the latest exact
    /// solution at the configured interval.
    latest_exact: Option<Rc<RefCell<ExactSolutionCache>>>,
}

#[derive(Default)]
struct ExactSolutionCache {
    groups: crate::optimizer::place_groups::PlaceGroups,
    frozen_ids: Vec<usize>,
    /// Multipolygon place groups of the owning grouped handle.
    expansion: Option<std::sync::Arc<crate::group_expansion::GroupExpansion>>,
    /// Latest native feasible/final report. One deep clone is shared through Rc.
    pending: Option<Rc<SPSolution>>,
    pending_version: u64,
    /// Version already checked, whether accepted or rejected.
    validated_version: u64,
    /// Last solution approved by the symmetric complete-layout validator.
    certified: Option<CertifiedExactSnapshot>,
    /// Opt-in Legacy Structure publication policy; other engines keep their contract.
    legacy_structure: Option<LegacyStructurePublicPolicy>,
    #[cfg(test)]
    processed_candidates: usize,
}

struct LegacyStructurePublicPolicy {
    margin_mm: Option<f32>,
    frozen_reference: Option<Rc<SPSolution>>,
    frozen_mask: Vec<bool>,
}

struct CertifiedExactSnapshot {
    solution: Rc<SPSolution>,
    /// Combined JSON is independent of report_type and can be replayed.
    payload: Option<CString>,
}

#[cfg(feature = "structure-first")]
enum LegacyStructureOutcome {
    Completed(SPSolution, crate::structure::pipeline::StructureRunReport),
    Failed(crate::structure::pipeline::StructurePipelineFailure),
    Cancelled,
}

/// Retain the already certified public checkpoint without changing phase or errors.
#[cfg(feature = "structure-first")]
fn retain_legacy_structure_error_checkpoint(
    state: &mut LegacyStepState,
    cache: &Rc<RefCell<ExactSolutionCache>>,
) -> Option<(f32, f32)> {
    let solution = cache.borrow().certified.as_ref()
        .map(|best| Rc::clone(&best.solution))?;
    let metrics = (solution.strip_width(), solution.density());
    crate::util::frozen::restore(&mut state.prob, &solution);
    state.solutions.clear();
    state.solutions.push(solution.as_ref().clone());
    state.solution = Some(solution.as_ref().clone());
    Some(metrics)
}

impl ExactSolutionCache {
    fn from_certified(solution: Option<SPSolution>) -> Self {
        let solution = solution.map(Rc::new);
        let version = u64::from(solution.is_some());
        Self {
            groups: crate::optimizer::place_groups::PlaceGroups::default(),
            frozen_ids: Vec::new(),
            expansion: None,
            legacy_structure: None,
            #[cfg(test)]
            processed_candidates: 0,
            pending: solution.clone(),
            pending_version: version,
            validated_version: version,
            certified: solution.map(|solution| CertifiedExactSnapshot {
                solution,
                payload: None,
            }),
        }
    }

    #[cfg(feature = "structure-first")]
    fn for_legacy_structure(
        instance: &SPInstance,
        reference: Option<&SPSolution>,
        frozen_ids: &[usize],
        margin_mm: Option<f32>,
    ) -> Self {
        let mut frozen_mask = vec![false; instance.items.len()];
        for &id in frozen_ids {
            if let Some(frozen) = frozen_mask.get_mut(id) { *frozen = true; }
        }
        Self {
            frozen_ids: frozen_ids.to_vec(),
            legacy_structure: Some(LegacyStructurePublicPolicy {
                margin_mm,
                frozen_reference: reference.filter(|_| !frozen_ids.is_empty()).map(|s| Rc::new(s.clone())),
                frozen_mask,
            }),
            ..Self::default()
        }
    }

    fn with_groups(mut self, groups: &crate::optimizer::place_groups::PlaceGroups) -> Self {
        if &self.groups != groups {
            self.groups = groups.clone();
            self.validated_version = 0;
            self.certified = None;
        }
        self
    }

    fn with_frozen_ids(mut self, ids: &[usize]) -> Self {
        self.frozen_ids = ids.to_vec();
        self
    }

    fn with_expansion(
        mut self,
        expansion: Option<std::sync::Arc<crate::group_expansion::GroupExpansion>>,
    ) -> Self {
        self.expansion = expansion;
        self
    }
}

#[derive(serde::Serialize)]
struct CallbackPayload<'a> {
    solution: &'a ExtSPSolution,
    #[serde(skip_serializing_if = "Option::is_none")]
    loss: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    collision_status: Option<&'a [CollisionStatus]>,
}

/// 인증된(검증을 통과한) 해의 진단값.
///
/// # 조작하지 않고 측정한다
///
/// 예전에는 `loss: Some(0.0)` 과 `collision: false` 를 **그냥 써 넣었다**. 검증을
/// 통과했으니 0이 맞다는 논리였고, 그 자체는 틀리지 않는다. 문제는 그 값이
/// 소비자에게 **아무 정보도 주지 않는다**는 것이다 — 검증기와 진단기가 어긋나도,
/// 배선이 끊겨도, 언제나 0이라 구분할 방법이 없다.
///
/// 이제 실제로 잰다. 인증된 해라면 결과는 0이어야 하고, 0이 아니면 그것이
/// **검증기와 진단기의 불일치**를 알리는 신호다. 비용은 조각쌍 O(n²) 검사인데,
/// 이 경로는 진행 인터벌로 조절되므로(기본 500ms 이상) 감당할 수 있다.
///
/// 진단 수집이 꺼져 있으면 예전처럼 아무것도 계산하지 않는다.
fn certified_callback_diagnostics(
    solution: &ExtSPSolution,
    instance: &SPInstance,
    options: CallbackOptions,
    frozen_ids: &[usize],
) -> CallbackDiagnostics {
    certified_callback_diagnostics_with_groups(
        solution,
        instance,
        options,
        frozen_ids,
        &crate::optimizer::place_groups::PlaceGroups::default(),
    )
}

fn certified_callback_diagnostics_with_groups(
    solution: &ExtSPSolution,
    instance: &SPInstance,
    options: CallbackOptions,
    frozen_ids: &[usize],
    groups: &crate::optimizer::place_groups::PlaceGroups,
) -> CallbackDiagnostics {
    if !options.collect_loss && !options.collect_collision_status {
        return CallbackDiagnostics::default();
    }
    let prob = crate::util::frozen::import_solution(instance, solution);
    compute_callback_diagnostics_with_groups(&prob.layout(), options, frozen_ids, groups)
}

fn serialize_callback_payload(
    solution: &ExtSPSolution,
    diagnostics: &CallbackDiagnostics,
) -> Option<CString> {
    let payload = CallbackPayload {
        solution,
        loss: diagnostics.loss,
        collision_status: diagnostics.collision_status.as_deref(),
    };
    serde_json::to_string(&payload)
        .ok()
        .and_then(|json| CString::new(json).ok())
}

fn callback_report_type(report: &ReportType) -> i32 {
    match report {
        ReportType::ExplFeas => 0,
        ReportType::ExplInfeas => 1,
        ReportType::ExplImproving => 2,
        ReportType::CmprFeas => 3,
        ReportType::Final => 4,
    }
}

fn report_legacy_solution(
    callback: SolutionCallback,
    user_data: *mut std::ffi::c_void,
    emit_json: bool,
    report_type: i32,
    solution: &SPSolution,
    instance: &SPInstance,
    callback_options: CallbackOptions,
    certified_exact: bool,
    frozen_ids: &[usize],
) {
    let strip_width = solution.strip_width();
    let density = solution.density();
    let emit_json = emit_json
        && (certified_exact
            || swarm_core::original_overlap::validate(
                instance,
                solution,
                &crate::util::frozen::mask(instance.items.len(), frozen_ids),
            )
            .is_ok());
    if emit_json {
        let ext_sol = jagua_rs::probs::spp::io::export(solution, *crate::EPOCH);
        let diagnostics = if certified_exact {
            certified_callback_diagnostics(&ext_sol, instance, callback_options, frozen_ids)
        } else if callback_options.collect_loss || callback_options.collect_collision_status {
            let mut reported_prob = SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
            crate::util::frozen::restore(&mut reported_prob, solution);
            compute_callback_diagnostics_with_frozen(
                &reported_prob.layout(),
                callback_options,
                frozen_ids,
            )
        } else {
            CallbackDiagnostics::default()
        };
        if let Some(cs) = serialize_callback_payload(&ext_sol, &diagnostics) {
            callback(report_type, strip_width, density, cs.as_ptr(), user_data);
            return;
        }
    }
    callback(
        report_type,
        strip_width,
        density,
        std::ptr::null(),
        user_data,
    );
}

#[cfg(test)]
fn certify_legacy_candidate(
    instance: &SPInstance,
    candidate: SPSolution,
    fallback: &SPSolution,
    phase_name: &str,
) -> Result<SPSolution, String> {
    certify_legacy_candidate_policy(instance, candidate, fallback, phase_name, &[])
}

fn certify_legacy_candidate_policy(
    instance: &SPInstance,
    candidate: SPSolution,
    fallback: &SPSolution,
    phase_name: &str,
    frozen_ids: &[usize],
) -> Result<SPSolution, String> {
    match crate::structure::pipeline::solution_validation_error_with_frozen_ids(
        instance, &candidate, frozen_ids,
    ) {
        None => Ok(candidate),
        Some(candidate_error) => {
            log::warn!(
                "[{phase_name}] rejected non-exact candidate and restored the certified fallback: {candidate_error:?}"
            );
            match crate::structure::pipeline::solution_validation_error_with_frozen_ids(
                instance, fallback, frozen_ids,
            ) {
                None => Ok(fallback.clone()),
                Some(fallback_error) => Err(format!(
                    "{phase_name} produced an invalid candidate ({candidate_error:?}) and its fallback is also invalid ({fallback_error:?})"
                )),
            }
        }
    }
}

fn certify_legacy_candidate_with_frozen(
    instance: &SPInstance,
    candidate: SPSolution,
    fallback: &SPSolution,
    phase_name: &str,
    frozen_ids: &[usize],
    fabric_edge_margin_mm: Option<f32>,
) -> Result<SPSolution, String> {
    // 원단 경계 보정은 **인증 앞**에 둔다. 보정이 만든 배치도 겹침·고정 계약을 똑같이
    // 통과해야 하고, 통과하지 못하면 아래 정책이 fallback 으로 되돌린다.
    let candidate = repair_fabric_edge(
        instance,
        candidate,
        phase_name,
        frozen_ids,
        fabric_edge_margin_mm,
    );
    let mut mask = vec![false; instance.items.len()];
    for &id in frozen_ids {
        if let Some(value) = mask.get_mut(id) {
            *value = true;
        }
    }
    if !crate::util::assertions::frozen_poses_preserved(fallback, &candidate, &mask) {
        log::warn!("[{phase_name}] rejected candidate that moved a frozen item");
        return certify_legacy_candidate_policy(
            instance,
            fallback.clone(),
            fallback,
            phase_name,
            frozen_ids,
        );
    }
    certify_legacy_candidate_policy(instance, candidate, fallback, phase_name, frozen_ids)
}

/// 공개 직전에 원단 밖으로 나간 조각만 안쪽으로 민다.
///
/// 균일 컨테이너 인셋과 달리 가장 날카로운 꼭짓점 하나가 원반 전체를 줄이지 않는다 —
/// 실측에서 그 방식은 최종 효율을 1.5%p 깎았다. 실패해도 원래 해를 그대로 돌려주므로
/// 이 단계가 결과를 나쁘게 만들 수는 없다([`keep_valid_repair`] 가 그 약속을 지킨다).
fn repair_fabric_edge(
    instance: &SPInstance,
    candidate: SPSolution,
    phase_name: &str,
    frozen_ids: &[usize],
    margin_mm: Option<f32>,
) -> SPSolution {
    repair_fabric_edge_with_certificate(instance, candidate, phase_name, frozen_ids, margin_mm).0
}

fn repair_fabric_edge_with_certificate(
    instance: &SPInstance,
    candidate: SPSolution,
    phase_name: &str,
    frozen_ids: &[usize],
    margin_mm: Option<f32>,
) -> (SPSolution, bool) {
    let Some(margin) = margin_mm else {
        return (candidate, false);
    };
    let mut prob = jagua_rs::probs::spp::entities::SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
    prob.restore(&candidate);
    let frozen: std::collections::HashSet<usize> = frozen_ids.iter().copied().collect();
    let movable = |item_id: usize| !frozen.contains(&item_id);
    let term = crate::util::terminator::BasicTerminator::new();
    match swarm_core::fabric_edge::repair(&mut prob, margin, &movable, &term) {
        Ok(outcome) if outcome.violations == 0 => (candidate, false),
        Ok(outcome) => {
            log::info!(
                "[{phase_name}] 원단 경계 보정: 위반 {} 조각 중 {} 개 복구, 길이 +{:.4}mm, 최악 {:.4}mm -> {:.4}mm (예산 {:.3}mm)",
                outcome.violations,
                outcome.repaired,
                outcome.widened,
                outcome.worst_before,
                outcome.worst_after,
                margin
            );
            // 잔여 위반은 조용히 넘기지 않는다. 인증기는 원본 외곽선의 경계 초과를 보지 않으므로,
            // 여기서 알리지 않으면 예산을 넘긴 해가 흔적 없이 공개된다.
            if outcome.worst_after > FABRIC_EDGE_RESIDUAL_TOLERANCE_MM {
                log::warn!(
                    "[{phase_name}] 원단 경계 보정 뒤에도 예산을 {:.4}mm 넘는 조각이 남았다 (예산 {:.3}mm)",
                    outcome.worst_after,
                    margin
                );
            }
            certify_fabric_edge_repair(instance, candidate, prob.save(), frozen_ids, phase_name)
        }
        Err(e) => {
            log::warn!("[{phase_name}] 원단 경계 보정 실패, 원래 해를 유지한다: {e}");
            (candidate, false)
        }
    }
}

/// 보정 뒤 잔여 초과를 경고로 올리는 문턱(mm). 좌표가 수천 mm 면 f32 한 눈금이 0.0005mm 라,
/// 그 크기의 잔여는 반올림이지 위반이 아니다(보정은 이동량에 그보다 큰 여유 0.002mm 를 더한다).
const FABRIC_EDGE_RESIDUAL_TOLERANCE_MM: f64 = 1.0e-3;

/// 보정본이 인증을 통과할 때만 그것을 쓰고, 아니면 **보정 전 후보**를 돌려준다.
///
/// 보정은 인증기와 같은 기준(원본 겹침, 충돌 도형의 충돌, 새로 생기는 컨테이너 이탈)을 보고
/// 이동을 채택하지만, 인증기 자체를 부르는 것은 아니다 — 예전 보정은 원본 겹침만 봤고 실제로
/// 보정본이 인증에 떨어졌다. 그때 보정 전 후보를 다시 보지 않고 곧장 직전 단계의 해로 물러나
/// **단계가 얻은 것을 전부 버렸다**(Polish 라면 +0.3 ~ 0.5%p). 이 함수는 그 마지막 안전망이다.
#[cfg(test)]
fn keep_valid_repair(
    instance: &SPInstance,
    candidate: SPSolution,
    repaired: SPSolution,
    frozen_ids: &[usize],
    phase_name: &str,
) -> SPSolution {
    certify_fabric_edge_repair(instance, candidate, repaired, frozen_ids, phase_name).0
}

fn certify_fabric_edge_repair(
    instance: &SPInstance,
    candidate: SPSolution,
    repaired: SPSolution,
    frozen_ids: &[usize],
    phase_name: &str,
) -> (SPSolution, bool) {
    match crate::structure::pipeline::solution_validation_error_with_frozen_ids(
        instance, &repaired, frozen_ids,
    ) {
        None => (repaired, true),
        Some(error) => {
            log::warn!(
                "[{phase_name}] 원단 경계 보정본이 인증에 떨어져 보정 전 해를 유지한다: {error:?}"
            );
            (candidate, false)
        }
    }
}

/// 시드 대결을 여는 초기 폭 차이의 상한(비율). 이보다 넓은 시드는 대결 없이 기각한다.
const SEED_DUEL_MAX_GAP: f32 = 0.03;

/// 시드 대결에서 후보 하나에 주는 탐색 시간: 조각 하나당 15ms, 1초 이상 3초 이하.
/// 조각이 적은 입력은 탐색이 몇 초 안에 수렴하므로 3초를 다 줄 이유가 없다.
const SEED_DUEL_PROBE_PER_ITEM: std::time::Duration = std::time::Duration::from_millis(15);
const SEED_DUEL_PROBE_MIN: std::time::Duration = std::time::Duration::from_secs(1);
const SEED_DUEL_PROBE_MAX: std::time::Duration = std::time::Duration::from_secs(3);

fn seed_duel_probe_time(n_items: usize) -> std::time::Duration {
    SEED_DUEL_PROBE_PER_ITEM
        .saturating_mul(n_items.min(u32::MAX as usize) as u32)
        .clamp(SEED_DUEL_PROBE_MIN, SEED_DUEL_PROBE_MAX)
}

/// NFP-LBF 시드와 기존 LBF 배치 중 어느 쪽이 **탐색의 더 나은 출발점인지** 짧은 탐색으로 가린다.
///
/// 시드 채택 기준은 «초기 폭이 더 좁은 쪽» 이었다. 그런데 초기 폭은 탐색이 끝난 뒤의 폭을
/// 잘 예측하지 못한다 — 양쪽 방향으로 다 틀린다.
///
/// 실측 1 — 더 넓게 시작한 시드가 더 좋았다 (babel 겉감 200조각, 한 방향, 겹침 0.4mm, 탐색 480초):
///
/// | 시드 | LBF 초기 | NFP-LBF 초기 | 채택 | 최종 |
/// |---|---|---|---|---|
/// | 1 | 13815.6 | 13742.5 (77.53%) | NFP-LBF | 86.09% |
/// | 2 | 13805.7 | 13742.5 (77.53%) | NFP-LBF | 85.73% |
/// | 3 | 13669.8 (77.94%) | 13742.5 (77.53%) | **LBF** | **83.76%** |
///
/// 시드 3 은 LBF 가 0.5% 짧게 시작했다는 이유로 NFP-LBF 를 버렸고, 탐색은 60초에 83.2% 벽에
/// 닿아 남은 420초를 실패로 보냈다. 시드 1 의 두 배치에 탐색을 3초씩만 돌려 보면 79.4% 대
/// 82.6% 로 이미 갈린다.
///
/// 실측 2 — 더 좁게 시작한 시드가 더 나빴다 (표준 인스턴스, 탐색 90초, 시드 1·2):
/// shirts 는 NFP-LBF 가 79.32% 로 LBF(78.34% / 76.12%)보다 조밀하게 시작해 채택됐지만 최종은
/// 87.80% / 88.01% 대 LBF 88.06% / 89.01% 였다. marques 도 89.83% / 90.18% 대 89.83% / 90.71%.
///
/// 그래서 후보가 유효하고 초기 폭이 [`SEED_DUEL_MAX_GAP`] 넘게 넓지 않으면, 두 배치에서 각각
/// [`seed_duel_probe_time`] 동안 탐색을 돌려 후보 쪽이 더 좁아졌을 때만 후보를 고른다. 탐색
/// 결과는 버린다 — Init 이 내놓는 것은 여전히 구성기가 만든 배치다. NFP-LBF 를 켠 Init 은
/// 그만큼(2 ~ 6초) 길어진다.
///
/// 대결은 벽시계로 끝나는 탐색이다. 두 후보의 차이가 작으면 같은 `rng_seed` 라도 CPU 부하에
/// 따라 선택이 달라질 수 있다 — 탐색 단계가 원래 그렇듯이 Init 도 더는 비트 단위로 재현되지
/// 않는다. 차이가 큰 경우(babel: 탐색 뒤 13082 대 13443)는 뒤집히지 않는다.
fn seed_duel_prefers_candidate(
    instance: &SPInstance,
    candidate: &SPSolution,
    incumbent: &SPSolution,
    config: &swarm_core::config::SwarmConfig,
) -> bool {
    let probe_time = seed_duel_probe_time(candidate.layout_snapshot().placed_items().len());
    seed_duel_with_probe(instance, candidate, incumbent, config, probe_time)
}

/// [`seed_duel_prefers_candidate`] 의 본체. 탐색 시간만 인자로 뺐다(테스트용).
fn seed_duel_with_probe(
    instance: &SPInstance,
    candidate: &SPSolution,
    incumbent: &SPSolution,
    config: &swarm_core::config::SwarmConfig,
    probe_time: std::time::Duration,
) -> bool {
    use rand::SeedableRng;
    let probe = |start: &SPSolution| -> Option<f32> {
        let mut prob = jagua_rs::probs::spp::entities::SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
        prob.restore(start);
        // 두 후보가 같은 난수 흐름에서 출발해야 비교가 된다.
        let rng = rand::rngs::Xoshiro256PlusPlus::seed_from_u64(
            swarm_core::protected_seed::seed_duel(config.rng_seed.unwrap_or(0) as u64),
        );
        // 짧은 탐색 두 번에 NFP 샘플러 라이브러리를 두 번 짓는 것은 탐색 예산 밖의
        // 비용이다. 결투는 두 후보를 같은 조건으로 비교하면 되므로 샘플러 없이 돈다.
        let mut sep_config = config.expl_cfg.separator_config;
        sep_config.nfp_sampler = false;
        let mut sep = swarm_core::optimizer::separator::Separator::new(
            instance.clone(),
            prob,
            rng,
            sep_config,
        );
        let mut term = crate::util::terminator::BasicTerminator::new();
        term.new_timeout(probe_time);
        let term = InitCancelTerminator::new(term);
        #[cfg(feature = "adaptive")]
        let solutions = swarm_core::optimizer::explore_mods::exploration_phase_adaptive(
            instance, &mut sep, &mut None, &term, &config.expl_cfg,
        );
        #[cfg(not(feature = "adaptive"))]
        let solutions = swarm_core::optimizer::explore::exploration_phase(
            instance, &mut sep, &mut None, &term, &config.expl_cfg,
        );
        solutions.iter().map(|solution| solution.strip_width()).min_by(f32::total_cmp)
    };
    let incumbent_width = probe(incumbent);
    if swarm_core::config::runtime::init_cancel_requested() {
        return false;
    }
    let candidate_width = probe(candidate);
    let prefers = match (candidate_width, incumbent_width) {
        (Some(candidate_width), Some(incumbent_width)) => {
            candidate_width + 1.0e-5 < incumbent_width
        }
        _ => false,
    } && !swarm_core::config::runtime::init_cancel_requested();
    prefers
}

/// 구성이 끝난 배치에 hole fill 과 LP 압축을 한 번씩 적용한다.
///
/// Polish 가 나중에 같은 일을 하지만 그때는 배치가 굳어 있고 Explore 가 이미 그
/// 배치를 전제로 폭을 정한 뒤다. 구성 직후에 하면 Explore 가 더 나은 출발점에서
/// 사다리를 내려간다.
fn compact_initial_layout(
    instance: &SPInstance,
    prob: &mut jagua_rs::probs::spp::entities::SPProblem,
    frozen_ids: &[usize],
    config: &swarm_core::config::SwarmConfig,
) {
    use rand::SeedableRng;
    if init_cancelled() {
        return;
    }
    let mut sep_config = config.expl_cfg.separator_config;
    // 다지기는 탐색이 아니다. 워커 복제와 NFP 준비 비용을 지운다.
    sep_config.n_workers = 1;
    sep_config.nfp_sampler = false;
    let mut sep = swarm_core::optimizer::separator::Separator::new(
        instance.clone(),
        prob.clone(),
        rand::rngs::Xoshiro256PlusPlus::seed_from_u64(config.rng_seed.unwrap_or(0) as u64),
        sep_config,
    );
    sep.freeze_items(frozen_ids);
    let mut term = crate::util::terminator::BasicTerminator::new();
    term.new_timeout(std::time::Duration::from_millis(config.init_compaction_ms));
    let term = InitCancelTerminator::new(term);

    swarm_core::optimizer::polish::holes::hole_fill_pass(
        &mut sep,
        &config.polish_cfg.hole_fill,
        &term,
    );
    // Polish 와 같은 솔버 선택을 따른다. 기본 솔버로 고정하면 사용자가 고른 LP 솔버가
    // 초기 다지기에서만 무시된다.
    swarm_core::optimizer::polish::lp::lp_compaction_pass_with_solver(
        &mut sep,
        &config.polish_cfg.lp,
        &config.lp_solver,
        &term,
    );

    // Cancellation discards auxiliary work and retains the certified incumbent.
    if term.stop_requested() {
        return;
    }
    let candidate = sep.prob.save();
    // 인증을 통과할 때만 채택한다. 실패하면 원래 배치를 그대로 둔다.
    if crate::structure::pipeline::solution_validation_error_with_frozen_ids(
        instance,
        &candidate,
        frozen_ids,
    )
    .is_none() && !term.stop_requested()
    {
        *prob = sep.prob;
    }
}

fn retain_exact_solution(
    latest: &Rc<RefCell<ExactSolutionCache>>,
    report: &ReportType,
    instance: &SPInstance,
    solution: &SPSolution,
    certify_now: bool,
) {
    let native_feasible = matches!(
        report,
        ReportType::ExplFeas | ReportType::CmprFeas | ReportType::Final
    );
    let candidate = {
        let mut latest = latest.borrow_mut();
        // Final and milestones may repeat the same native snapshot. Structure
        // compares complete poses, never just width (equal-width P5 moves matter).
        let repeated_structure_snapshot = latest.legacy_structure.is_some()
            && latest.pending.as_ref().is_some_and(|pending| {
                same_native_snapshot(pending, solution)
            });
        if native_feasible && !repeated_structure_snapshot {
            latest.pending_version = latest.pending_version.wrapping_add(1).max(1);
            latest.pending = Some(Rc::new(solution.clone()));
        }
        if !certify_now && latest.certified.is_some() {
            return;
        }
        if latest.validated_version == latest.pending_version {
            return;
        }
        latest
            .pending
            .clone()
            .map(|candidate| (latest.pending_version, candidate))
    };
    let Some((version, candidate)) = candidate else {
        return;
    };
    #[cfg(test)]
    { latest.borrow_mut().processed_candidates += 1; }
    let candidate = {
        let cache = latest.borrow();
        let canonical = if let Some(policy) = &cache.legacy_structure {
            let preserves_reference = |s: &SPSolution| policy.frozen_reference.as_ref()
                .is_some_and(|reference| crate::util::assertions::frozen_poses_preserved(
                    reference, s, &policy.frozen_mask))
                || !policy.frozen_mask.iter().any(|frozen| *frozen);
            if !preserves_reference(&candidate) {
                None
            } else {
                let (solution, exact_checked) = if policy.margin_mm.is_some() {
                    let (repaired, checked) = repair_fabric_edge_with_certificate(instance,
                        candidate.as_ref().clone(), "STRUCT", &cache.frozen_ids, policy.margin_mm);
                    (Rc::new(repaired), checked)
                } else {
                    (Rc::clone(&candidate), false)
                };
                preserves_reference(&solution).then_some((solution, exact_checked))
            }
        } else {
            Some((candidate, false))
        };
        canonical.and_then(|(solution, exact_checked)| {
            // The repair helper certified this exact geometry with the same
            // frozen IDs. Grouped policies must still use their explicit groups.
            (exact_checked && cache.groups.is_empty()
                || crate::structure::pipeline::solution_validation_error_with_frozen_ids_and_groups(
                    instance, &solution, &cache.frozen_ids, &cache.groups).is_none())
                .then_some(solution)
        })
    };
    if let Some(candidate) = candidate {
        let mut latest = latest.borrow_mut();
        latest.validated_version = version;
        // This cache belongs to one complete instance/run. A late worker or
        // relaxed target must never replace an already published shorter nest.
        // Equal-width progress follows the same compactness ordering as Polish.
        if latest
            .certified
            .as_ref()
            .is_none_or(|best| {
                crate::optimizer::polish::prefers_compact_solution(&candidate, &best.solution)
            })
        {
            latest.certified = Some(CertifiedExactSnapshot {
                solution: candidate,
                payload: None,
            });
        }
    } else {
        latest.borrow_mut().validated_version = version;
    }
}

/// Native snapshots belong to one immutable instance. Rekeying/reordering
/// may miss a reuse opportunity, but can never hide changed item poses or width.
fn same_native_snapshot(a: &SPSolution, b: &SPSolution) -> bool {
    a.strip_width().to_bits() == b.strip_width().to_bits()
        && a.layout_snapshot().placed_items().len() == b.layout_snapshot().placed_items().len()
        && a.layout_snapshot().placed_items().values().zip(b.layout_snapshot().placed_items().values())
            .all(|(a, b)| {
                let (ax, ay) = a.d_transf().translation();
                let (bx, by) = b.d_transf().translation();
                a.item().idx() == b.item().idx()
                    && a.d_transf().rotation().to_bits() == b.d_transf().rotation().to_bits()
                    && ax.to_bits() == bx.to_bits() && ay.to_bits() == by.to_bits()
            })
}

#[cfg(feature = "structure-first")]
struct LegacyStructureListener {
    cache: Rc<RefCell<ExactSolutionCache>>,
    callback: Option<SolutionCallback>,
    user_data: usize,
    emit_json: bool,
    callback_options: CallbackOptions,
    last_emit: Rc<Cell<Option<Instant>>>,
    min_interval: Duration,
}

#[cfg(feature = "structure-first")]
impl SolutionListener for LegacyStructureListener {
    fn report_certified_early(&mut self, solution: &SPSolution, instance: &SPInstance) {
        let previous = self.cache.borrow().certified.as_ref().map(|s| Rc::clone(&s.solution));
        retain_exact_solution(&self.cache, &ReportType::ExplFeas, instance, solution, true);
        let changed = self.cache.borrow().certified.as_ref().is_some_and(|s| {
            previous.as_ref().is_none_or(|old| !Rc::ptr_eq(old, &s.solution))
        });
        if changed {
            self.emit(&ReportType::ExplFeas, instance);
        }
    }

    fn report(&mut self, report: ReportType, solution: &SPSolution, instance: &SPInstance) {
        let due = self.last_emit.get().is_none_or(|last| last.elapsed() >= self.min_interval);
        retain_exact_solution(&self.cache, &report, instance, solution,
            due || matches!(report, ReportType::Final));
        // The FFI owner commits the chosen canonical snapshot before its sole Final.
        if due && !matches!(report, ReportType::Final) {
            self.emit(&report, instance);
        }
    }
}

#[cfg(feature = "structure-first")]
impl LegacyStructureListener {
    fn emit(&self, report: &ReportType, instance: &SPInstance) {
        if let Some(callback) = self.callback {
            let report_type = if matches!(report, ReportType::ExplFeas | ReportType::CmprFeas) {
                callback_report_type(report)
            } else { 2 };
            report_cached_exact_solution(callback, self.user_data as *mut std::ffi::c_void,
                self.emit_json, report_type, &self.cache, instance, self.callback_options);
        }
        // Callback registration affects only emission, not candidate selection.
        self.last_emit.set(Some(Instant::now()));
    }
}

fn report_cached_exact_solution(
    callback: SolutionCallback,
    user_data: *mut std::ffi::c_void,
    emit_json: bool,
    report_type: i32,
    cache: &Rc<RefCell<ExactSolutionCache>>,
    instance: &SPInstance,
    callback_options: CallbackOptions,
) -> bool {
    let mut cache = cache.borrow_mut();
    let frozen_ids = cache.frozen_ids.clone();
    let groups = cache.groups.clone();
    let expansion = cache.expansion.clone();
    let Some(certified) = cache.certified.as_mut() else {
        return false;
    };
    let strip_width = certified.solution.strip_width();
    let density = certified.solution.density();
    if emit_json && certified.payload.is_none() {
        let ext_sol =
            jagua_rs::probs::spp::io::export(certified.solution.as_ref(), *crate::EPOCH);
        let diagnostics = certified_callback_diagnostics_with_groups(
            &ext_sol,
            instance,
            callback_options,
            &frozen_ids,
            &groups,
        );
        // Multipolygon place groups: public ids and member poses.
        let diagnostics = expand_diagnostics(expansion.as_ref(), diagnostics);
        let ext_sol = match &expansion {
            Some(expansion) => expansion.expand_solution(ext_sol),
            None => ext_sol,
        };
        certified.payload = serialize_callback_payload(&ext_sol, &diagnostics);
    }
    let json_ptr = certified
        .payload
        .as_ref()
        .filter(|_| emit_json)
        .map_or(std::ptr::null(), |payload| payload.as_ptr());
    callback(report_type, strip_width, density, json_ptr, user_data);
    true
}

/// 탐색,압축 단계에서 솔루션이 발견될 때마다 콜백을 하기위한 SolutionListener 구현부
impl SolutionListener for CallbackListener {
    fn report_certified_early(&mut self, solution: &SPSolution, instance: &SPInstance) {
        let cache = self.latest_exact.get_or_insert_with(|| {
            Rc::new(RefCell::new(
                ExactSolutionCache::default().with_frozen_ids(&self.frozen_ids),
            ))
        });
        let old_width = cache
            .borrow()
            .certified
            .as_ref()
            .map(|best| best.solution.strip_width());
        if old_width.is_some_and(|width| width <= solution.strip_width()) {
            return;
        }
        // Certification is cached on this owner thread. Retain before invoking
        // external code; that callback may immediately request cancellation.
        retain_exact_solution(cache, &ReportType::ExplFeas, instance, solution, true);
        let new_width = cache
            .borrow()
            .certified
            .as_ref()
            .map(|best| best.solution.strip_width());
        if new_width == old_width {
            return;
        }
        if report_cached_exact_solution(
            self.callback,
            self.user_data,
            self.emit_json,
            callback_report_type(&ReportType::ExplFeas),
            cache,
            instance,
            self.callback_options,
        ) {
            self.last_emit.set(Some(Instant::now()));
        }
    }

    fn report(&mut self, report: ReportType, solution: &SPSolution, instance: &SPInstance) {
        let report_type = callback_report_type(&report);
        // JSON 페이로드를 실을지는 **레이아웃이 실행 가능한가**로 정한다.
        //
        // # 왜 ExplImproving 을 뺐나
        //
        // `ExplImproving` 은 분리 루프의 `else if loss < min_loss` 가지에서만 나온다
        // (`separator.rs`). 그 가지에 닿았다는 것은 바로 위의 `loss == 0.0` 가지가
        // 아니었다는 뜻이고, 그 가지는 보고 없이 `break 'outer` 한다. 따라서
        // **ExplImproving 이 실어 나르는 레이아웃에는 반드시 겹침이 남아 있다**
        // (`tracker.rs` 의 loss 는 검출된 충돌에만 값이 붙고 `assert!(loss > 0.0)` 다).
        //
        // 예전에는 이 종류가 **무조건** JSON 적격이었고, 정작 겹침이 없는 `ExplFeas`
        // 는 기본이 꺼진 `emit_feasible_json` 뒤에 막혀 있었다. 그래서 탐색 단계
        // 내내 소비자에게 가는 JSON 은 전부 겹친 배치였다 — WPF 샘플은 JSON 이 없는
        // 콜백을 렌더링에서 버리므로(`WorkspaceViewModel.OnSolutionFound`), 화면에는
        // 겹친 해만 흘렀다. 우선순위가 뒤집혀 있었다.
        //
        // `Final` 은 `certify_legacy_candidate` 를 통과한 해만 실리고, `CmprFeas` 는
        // 압축·폴리시의 `get_total_loss() != 0.0` 가드 뒤에서만 나온다. 둘 다 안전하다.
        let json_eligible = matches!(report, ReportType::CmprFeas | ReportType::Final)
            || (self.emit_feasible_json && matches!(report, ReportType::ExplFeas));
        let is_final = matches!(report, ReportType::Final);
        let throttled = match self.last_emit.get() {
            Some(t) => t.elapsed() < self.min_interval,
            None => false,
        };

        if self.throttle_all_reports {
            let current_is_exact = matches!(
                report,
                ReportType::ExplFeas | ReportType::CmprFeas | ReportType::Final
            );
            if let Some(latest_exact) = &self.latest_exact {
                retain_exact_solution(
                    latest_exact,
                    &report,
                    instance,
                    solution,
                    is_final || !throttled,
                );
            }
            if !is_final && throttled {
                return;
            }
            let Some(latest_exact) = self.latest_exact.as_ref() else {
                return;
            };
            let exact_report_type = if current_is_exact { report_type } else { 2 };
            if !report_cached_exact_solution(
                self.callback,
                self.user_data,
                self.emit_json,
                exact_report_type,
                latest_exact,
                instance,
                self.callback_options,
            ) {
                return;
            }
            self.last_emit.set(Some(Instant::now()));
            return;
        }

        // Preserve the ordinary legacy contract: every solver event may still
        // produce its scalar callback, while complete JSON is rate-limited.
        let emit_json_now = self.emit_json && json_eligible && (is_final || !throttled);
        if let Some(cache) = &self.latest_exact
            && matches!(
                report,
                ReportType::ExplFeas | ReportType::CmprFeas | ReportType::Final
            )
        {
            retain_exact_solution(cache, &report, instance, solution, true);
            report_cached_exact_solution(
                self.callback,
                self.user_data,
                emit_json_now,
                report_type,
                cache,
                instance,
                self.callback_options,
            );
            if emit_json_now {
                self.last_emit.set(Some(Instant::now()));
            }
            return;
        }
        report_legacy_solution(
            self.callback,
            self.user_data,
            emit_json_now,
            report_type,
            solution,
            instance,
            self.callback_options,
            false,
            &self.frozen_ids,
        );
        if emit_json_now {
            self.last_emit.set(Some(Instant::now()));
        }
    }
}

/// Emit one coherent, globally merged V2 solution.  Region lanes never expose
/// their local IDs or local strip coordinates through the public callback.
fn report_grouped_solution(state: &GroupedHandleState, report_type: i32) -> Result<(), String> {
    report_grouped_solver_solution(
        &state.solver,
        state.solution_callback,
        state.callback_user_data,
        state.emit_intermediate_json,
        report_type,
        state.callback_options,
        true,
    )
}

fn report_grouped_solver_solution(
    solver: &GroupedStepState,
    callback: Option<SolutionCallback>,
    callback_user_data: usize,
    emit_intermediate_json: bool,
    report_type: i32,
    callback_options: CallbackOptions,
    certified_exact: bool,
) -> Result<(), String> {
    let solution = if certified_exact {
        solver.export_certified_solution()?
    } else {
        solver.export_solution()
    };
    let Some(callback) = callback else {
        return Ok(());
    };
    let strip_width = solution.strip_width;
    let density = solution.density;
    if emit_intermediate_json {
        let diagnostics = if certified_exact {
            certified_callback_diagnostics_with_groups(
                &solution,
                solver.full_instance(),
                callback_options,
                solver.frozen_ids(),
                solver.place_groups(),
            )
        } else if callback_options.collect_loss || callback_options.collect_collision_status {
            let prob = solver.restored_problem_from_exported(&solution);
            compute_callback_diagnostics_with_groups(
                &prob.layout(),
                callback_options,
                solver.frozen_ids(),
                solver.place_groups(),
            )
        } else {
            CallbackDiagnostics::default()
        };
        // Multipolygon place groups: the payload carries public ids and member poses.
        let solution = solver.expand_public(solution);
        let diagnostics = expand_diagnostics(solver.expansion(), diagnostics);
        if let Some(json) = serialize_callback_payload(&solution, &diagnostics) {
            callback(
                report_type,
                strip_width,
                density,
                json.as_ptr(),
                callback_user_data as *mut std::ffi::c_void,
            );
            return Ok(());
        }
    }
    callback(
        report_type,
        strip_width,
        density,
        std::ptr::null(),
        callback_user_data as *mut std::ffi::c_void,
    );
    Ok(())
}

/// Collision statuses of compact (engine) ids in public ids.
fn expand_statuses(
    expansion: Option<&std::sync::Arc<crate::group_expansion::GroupExpansion>>,
    statuses: Vec<CollisionStatus>,
) -> Vec<CollisionStatus> {
    match expansion {
        Some(expansion) => expansion
            .expand_flags(statuses.into_iter().map(|s| (s.item_id, s.collision)).collect())
            .into_iter()
            .map(|(item_id, collision)| CollisionStatus { item_id, collision })
            .collect(),
        None => statuses,
    }
}

fn expand_diagnostics(
    expansion: Option<&std::sync::Arc<crate::group_expansion::GroupExpansion>>,
    mut diagnostics: CallbackDiagnostics,
) -> CallbackDiagnostics {
    if expansion.is_some() {
        if let Some(status) = diagnostics.collision_status.take() {
            diagnostics.collision_status = Some(expand_statuses(expansion, status));
        }
    }
    diagnostics
}

/// Place Hint 정보 구조체. 위치, 고정, 회전, 스트라이프 간격 정보.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct CPlacedHint {
    /// Swarm 에서 인식하는 Item ID (0-based). 범위를 벗어나면 힌트 전체가 무시된다.
    pub item_id: i32,
    /// 고정 여부(1 : 고정, 0 : 미고정).
    ///
    /// 좌표 사용 여부와는 **무관하다**. 힌트를 보내면 `tr_x`/`tr_y`/`rot_angle_in_rad`는
    /// 항상 시작 위치로 쓰이고, 이 값이 1이면 그 위에 "이동 금지"가 더해질 뿐이다.
    /// 0이면 그 좌표에서 출발하되 탐색/압축이 자유롭게 옮긴다(warm start).
    /// 잠금끼리의 겹침과 잠금의 strip 이탈은 Init/모든 단계/검증/진단에서 허용된다.
    /// 미잠금 Place는 잠금 Place를 장애물로 취급하며 strip 내부에 있어야 한다.
    ///
    /// 주의: 고정은 배치된 개체가 아니라 **아이템 종류(`item_id`)** 단위로 걸린다.
    /// 한 종류의 수량이 2개 이상인 인스턴스에서 그 종류를 고정하면 (힌트로 놓은 것뿐 아니라)
    /// 같은 종류 전체가 고정된다. 조각마다 별도 id를 부여하면 이 구분은 사라진다.
    pub is_freeze: i32,
    /// translation x (mm) : 아이템 bbox 좌하단이 놓일 x 위치. 힌트를 보내면 항상 유효해야 한다.
    pub tr_x: f32,
    /// translation y (mm) : 아이템 bbox 좌하단이 놓일 y 위치. 힌트를 보내면 항상 유효해야 한다.
    pub tr_y: f32,
    /// 회전 radians
    pub rot_angle_in_rad: f32,
    /// stripe interval y (0 : 미사용)
    pub stripe_interval_y: f32,
    /// stripe interval x (0 : 미사용)
    pub stripe_interval_x: f32,
}

/// Place 바운딩 박스 정보.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct CItemBounds {
    pub item_id: i32,
    pub min_x: f32,
    pub min_y: f32,
    pub max_x: f32,
    pub max_y: f32,
}

/// Swarm 단계별 최적화 핸들에 대한 콜백 등록. 탐색,압축 단계에서 솔루션이 발견될 때마다 호출됨.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_set_callback(
    handle: *mut SwarmStepHandle,
    callback: Option<SolutionCallback>,
    user_data: *mut std::ffi::c_void,
    emit_json: i32,
) {
    if handle.is_null() {
        return;
    }
    let handle = unsafe { &*handle };
    let Some(_call) = handle.lifecycle.begin_idle_call() else {
        return;
    };
    let mut engine = handle.lock_engine();
    match &mut *engine {
        HandleEngine::Legacy(h) => {
            h.solution_callback = callback;
            h.callback_user_data = user_data as usize;
            h.emit_intermediate_json = emit_json != 0;
        }
        HandleEngine::Grouped(h) => {
            h.solution_callback = callback;
            h.callback_user_data = user_data as usize;
            h.emit_intermediate_json = emit_json != 0;
        }
    }
}

/// Error code for the most recent init, phase, or solution-export call on this thread.
/// Zero means no error. The numeric values are stable ABI diagnostics; the
/// accompanying message contains the field/item/region details.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_get_last_error_code() -> i32 {
    LAST_ERROR.with(|slot| slot.borrow().0)
}

/// UTF-8 message for the most recent init, phase, or solution-export error on this thread.
/// The returned string must be released with `swarm_free_string`.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_get_last_error_message() -> *mut c_char {
    let message = LAST_ERROR.with(|slot| slot.borrow().1.clone());
    if message.is_empty() {
        return std::ptr::null_mut();
    }
    CString::new(message.replace('\0', "�"))
        .map(CString::into_raw)
        .unwrap_or(std::ptr::null_mut())
}

/// Json 파일로 Swarm 단계별 최적화 핸들 초기화
/// # input_path : 입력 파일 경로 (C 문자열 포인터)
/// # config : CSwarmConfig 포인터 (NULL이면 기본값)
/// # hints : CPlacedHint 배열 포인터 (NULL이면 없음)
/// # hint_count : hints 배열 길이
/// # 반환값 : SwarmStepHandle 포인터 (NULL이면 초기화 실패)
#[unsafe(no_mangle)]
pub extern "C" fn swarm_init(
    input_path: *const c_char,
    config: *const CSwarmConfig,
    hints: *const CPlacedHint,
    hint_count: i32,
) -> *mut SwarmStepHandle {
    clear_last_error();
    // 하드닝(선택 feature): run 당 1회 런타임 탬퍼 스캔(비핫). 의심 시 호스트 콜백 보고.
    #[cfg(feature = "antitamper")]
    swarm_core::tamper::run_scan(&[swarm_init as usize, swarm_init_phase as usize]);
    let _init_scope = swarm_core::config::runtime::begin_init();
    let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(
        || -> Option<*mut SwarmStepHandle> {
            let input_str = unsafe { cstr_to_str(input_path) };
            if input_str.is_empty() {
                return None;
            }
            let input_p = std::path::Path::new(input_str);
            let (config, callback_options, initial_config) = if config.is_null() {
                (
                    DEFAULT_SWARM_CONFIG,
                    DEFAULT_CALLBACK_OPTIONS,
                    crate::ffi::InitialPlacementSettings::default(),
                )
            } else {
                let c_config = unsafe { &*config };
                (
                    crate::ffi::build_config(c_config),
                    crate::ffi::build_callback_options(c_config),
                    crate::ffi::InitialPlacementSettings::from_c(&c_config.initial_placement),
                )
            };
            #[cfg(test)]
            issue9_count_input_work(true);
            let json = std::fs::read_to_string(input_p).ok()?;
            #[cfg(test)]
            issue9_count_input_work(false);
            if let Some(value) = serde_json::from_str::<serde_json::Value>(&json)
                .ok().filter(grouped_input::is_v2_value)
            {
                let input = match grouped_input::parse_and_validate_value(value) {
                    Ok(input) => input,
                    Err(error) => {
                        set_last_error(grouped_input_error_code(&error.code), error.to_string());
                        return None;
                    }
                };
                drop(json);
                return match build_grouped_handle(input, config, callback_options) {
                    Ok(handle) => Some(handle),
                    Err(error) => {
                        set_last_error(2001, error);
                        None
                    }
                };
            }
            let (ext_inst, ext_sol) = io::parse_spp_input(&json).ok()?;
            drop(json);
            build_handle(
                ext_inst,
                ext_sol,
                config,
                callback_options,
                initial_config,
                unsafe { parse_hints(hints, hint_count) },
            )
        },
    ));
    match run {
        Ok(Some(ptr)) => {
            if init_cancelled() {
                swarm_free(ptr);
                std::ptr::null_mut()
            } else {
                ptr
            }
        }
        Ok(None) => {
            if init_cancelled() {
                return std::ptr::null_mut();
            }
            if !has_last_error() {
                set_last_error(2000, "failed to read or initialize Swarm input");
            }
            std::ptr::null_mut()
        }
        Err(_) => {
            set_last_error(9000, "panic while initializing Swarm input");
            std::ptr::null_mut()
        }
    }
}

/// Json 문자열로 Swarm 단계별 최적화 핸들 초기화
/// # input_json : 입력 JSON 문자열 (C 문자열 포인터)
/// # config : CSwarmConfig 포인터 (NULL이면 기본값)
/// # hints : CPlacedHint 배열 포인터 (NULL이면 없음)
/// # hint_count : hints 배열 길이
/// # 반환값 : SwarmStepHandle 포인터 (NULL이면 초기화 실패)
#[unsafe(no_mangle)]
pub extern "C" fn swarm_init_from_json(
    input_json: *const c_char,
    config: *const CSwarmConfig,
    hints: *const CPlacedHint,
    hint_count: i32,
) -> *mut SwarmStepHandle {
    #[cfg(feature = "init-profile")]
    let _profile_session = swarm_core::init_profile::Session::begin();
    #[cfg(feature = "init-profile")]
    let mut _profile_phase = swarm_core::init_profile::Scope::new("phase.input");
    clear_last_error();
    // 하드닝(선택 feature): run 당 1회 런타임 탬퍼 스캔(비핫). 의심 시 호스트 콜백 보고.
    #[cfg(feature = "antitamper")]
    swarm_core::tamper::run_scan(&[swarm_init as usize, swarm_init_phase as usize]);
    let _init_scope = swarm_core::config::runtime::begin_init();
    let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(
        || -> Option<*mut SwarmStepHandle> {
            let json_str = unsafe { cstr_to_str(input_json) };
            if json_str.is_empty() {
                return None;
            }
            let (config, callback_options, initial_config) = if config.is_null() {
                (
                    DEFAULT_SWARM_CONFIG,
                    DEFAULT_CALLBACK_OPTIONS,
                    crate::ffi::InitialPlacementSettings::default(),
                )
            } else {
                let c_config = unsafe { &*config };
                (
                    crate::ffi::build_config(c_config),
                    crate::ffi::build_callback_options(c_config),
                    crate::ffi::InitialPlacementSettings::from_c(&c_config.initial_placement),
                )
            };
            #[cfg(test)]
            issue9_count_input_work(false);
            if let Some(value) = serde_json::from_str::<serde_json::Value>(json_str)
                .ok().filter(grouped_input::is_v2_value)
            {
                let input = match grouped_input::parse_and_validate_value(value) {
                    Ok(input) => input,
                    Err(error) => {
                        set_last_error(grouped_input_error_code(&error.code), error.to_string());
                        return None;
                    }
                };
                #[cfg(feature = "init-profile")]
                _profile_phase.finish();
                return match build_grouped_handle(input, config, callback_options) {
                    Ok(handle) => Some(handle),
                    Err(error) => {
                        set_last_error(2001, error);
                        None
                    }
                };
            }
            let (ext_inst, ext_sol) = io::parse_spp_input(json_str).ok()?;
            #[cfg(feature = "init-profile")]
            _profile_phase.finish();
            build_handle(
                ext_inst,
                ext_sol,
                config,
                callback_options,
                initial_config,
                unsafe { parse_hints(hints, hint_count) },
            )
        },
    ));
    match run {
        Ok(Some(ptr)) => {
            if init_cancelled() {
                swarm_free(ptr);
                std::ptr::null_mut()
            } else {
                ptr
            }
        }
        Ok(None) => {
            if init_cancelled() {
                return std::ptr::null_mut();
            }
            if !has_last_error() {
                set_last_error(
                    2000,
                    "invalid legacy Swarm JSON or initialization failure",
                );
            }
            std::ptr::null_mut()
        }
        Err(_) => {
            set_last_error(9000, "panic while initializing Swarm JSON");
            std::ptr::null_mut()
        }
    }
}

unsafe fn parse_hints(hints: *const CPlacedHint, count: i32) -> Vec<CPlacedHint> {
    if hints.is_null() || count <= 0 {
        return vec![];
    }
    unsafe { std::slice::from_raw_parts(hints, count as usize) }.to_vec()
}

#[derive(Clone, Copy, Debug)]
enum Axis {
    X,
    Y,
}

/// 고정되지 않은 배치들을 `interval` 격자에 맞춰 해당 축으로 스냅한다 (best-effort).
fn snap_to_stripe(
    prob: &mut SPProblem,
    interval: f32,
    axis: Axis,
    frozen: &std::collections::HashSet<usize>,
) -> bool {
    use jagua_rs::geometry::DTransformation;
    use jagua_rs::geometry::geo_traits::Transformable;

    if !interval.is_finite() || interval <= 0.0 {
        return false;
    }
    let before = prob.save();
    let container_bbox = prob.layout().container().outer_cd().bbox();
    let mut required_width = prob.strip_width();
    let mut moves = Vec::new();
    for (pk, placed) in prob
        .layout()
        .placed_items()
        .iter()
        .filter(|(_, pi)| !frozen.contains(&pi.item().idx()))
    {
        // Hint coordinates are defined by the placed bbox bottom-left, so the
        // stripe anchor must use that same public coordinate rather than the
        // internal centroid-normalized translation.
        let anchor = match axis {
            Axis::X => placed.shape().bbox().x_min,
            Axis::Y => placed.shape().bbox().y_min,
        };
        let snapped = (anchor / interval).round() * interval;
        let delta = snapped - anchor;
        if delta.abs() <= 1.0e-4 {
            continue;
        }
        let (x, y) = placed.d_transf().translation();
        let translation = match axis {
            Axis::X => (x + delta, y),
            Axis::Y => (x, y + delta),
        };
        let target = DTransformation::new(placed.d_transf().rotation(), translation);
        let item = prob.instance().item(placed.item().idx());
        let target_shape = item.shape_cd().transform_clone(&target.compose());
        let bbox = target_shape.bbox();
        let fixed_bounds = [
            container_bbox.x_min,
            container_bbox.x_max.max(bbox.x_max),
            container_bbox.y_min,
            container_bbox.y_max,
        ];
        if !crate::structure::pipeline::bbox_is_contained_mm(
            fixed_bounds,
            [bbox.x_min, bbox.x_max, bbox.y_min, bbox.y_max],
        ) {
            log::warn!(
                "[INIT] rolled back {:?} stripe snap outside the fixed strip bounds",
                axis
            );
            return false;
        }
        required_width = required_width
            .max(bbox.x_max * 1.00001 + prob.strip().shape_modify_config.offset.unwrap_or(0.0));
        moves.push((pk, placed.item().idx(), target));
    }

    if moves.is_empty() {
        return true;
    }
    if required_width > prob.strip_width() {
        crate::util::frozen::change_strip_width(prob, required_width);
    }
    for (old_key, item_id, target) in moves {
        prob.remove_item(old_key);
        prob.place_item(SPPlacement {
            item_idx: item_id,
            d_transf: target,
        });
    }
    let mask = crate::util::frozen::mask(
        prob.instance().items.len(),
        &frozen.iter().copied().collect::<Vec<_>>(),
    );
    crate::util::frozen::fit_strip_with_frozen(prob, &mask);
    let candidate = prob.save();
    if let Some(error) = crate::structure::pipeline::solution_validation_error_with_frozen_ids(
        &prob.instance(),
        &candidate,
        &frozen.iter().copied().collect::<Vec<_>>(),
    ) {
        log::warn!("[INIT] rolled back {:?} stripe snap: {error:?}", axis);
        crate::util::frozen::restore(prob, &before);
        return false;
    }
    true
}

#[derive(Clone, Copy)]
struct LegacySeedPlacement {
    placement: SPPlacement,
    frozen: bool,
}

/// 힌트 배치의 겹침을 그 자리에서 풀 때 주는 예산.
///
/// 양자화로 생긴 미세 관통(sub-mm)은 보통 즉시 풀린다. 이 예산은 "풀 수 없는 배치"에
/// 초기화가 오래 붙잡히지 않게 하는 상한이지 목표 시간이 아니다. 넘기면 힌트를 포기하고
/// 기존 경로(frozen 전용 재구성)로 떨어진다.
const SEED_SEPARATION_BUDGET: Duration = Duration::from_secs(10);

/// Init 취소 오류 코드 (`swarm_cancel_init`). 호스트는 이 코드를 «실패» 가 아니라
/// «사용자 중단» 으로 다뤄야 한다.
pub(crate) const INIT_CANCELLED_ERROR: i32 = 2002;

/// Init 이 취소됐으면 마지막 오류를 기록하고 `true` 를 돌려준다.
pub(crate) fn init_cancelled() -> bool {
    if swarm_core::config::runtime::init_cancel_requested() {
        set_last_error(
            INIT_CANCELLED_ERROR,
            "initialization cancelled by swarm_cancel_init",
        );
        true
    } else {
        false
    }
}

use swarm_core::util::terminator::InitTerminator as InitCancelTerminator;

/// 시드 배치를 좌표 그대로 놓기만 한다. 검증은 하지 않는다.
fn legacy_seed_problem_raw(
    instance: &SPInstance,
    placements: &[LegacySeedPlacement],
    preferred_width: f32,
    enforce_strip_bounds: bool,
) -> Result<SPProblem, String> {
    use jagua_rs::geometry::geo_traits::Transformable;

    let mut counts = vec![0usize; instance.items.len()];
    for seed in placements {
        let Some(count) = counts.get_mut(seed.placement.item_idx) else {
            return Err(format!("unknown seeded item {}", seed.placement.item_idx));
        };
        *count += 1;
        let expected = instance.item_qty(seed.placement.item_idx);
        if *count > expected {
            return Err(format!(
                "seeded item {} exceeds demand {expected}",
                seed.placement.item_idx
            ));
        }
    }

    let mut prob = SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
    let base_bbox = prob.layout().container().outer_cd().bbox();
    let mut placements = placements.to_vec();
    let mut rightmost_seed_x = None::<f32>;
    for seed in &mut placements {
        let item = instance.item(seed.placement.item_idx);
        let mut shape = item
            .shape_cd()
            .transform_clone(&seed.placement.d_transf.compose());
        if enforce_strip_bounds && !seed.frozen {
            if let Some(nudged) = nudge_seed_into_strip(instance, &prob, seed.placement, &shape) {
                seed.placement = nudged;
                shape = item.shape_cd().transform_clone(&nudged.d_transf.compose());
            }
        }
        let bbox = shape.bbox();
        let fixed_bounds = [
            base_bbox.x_min,
            base_bbox.x_max.max(bbox.x_max),
            base_bbox.y_min,
            base_bbox.y_max,
        ];
        if enforce_strip_bounds
            && !seed.frozen
            && !crate::structure::pipeline::bbox_is_contained_mm(
                fixed_bounds,
                [bbox.x_min, bbox.x_max, bbox.y_min, bbox.y_max],
            )
        {
            return Err(format!(
                "seeded item {} lies outside the fixed strip bounds",
                seed.placement.item_idx
            ));
        }
        rightmost_seed_x = Some(rightmost_seed_x.map_or(bbox.x_max, |x| x.max(bbox.x_max)));
    }
    let mut required_width = prob.strip_width();
    if let Some(rightmost_seed_x) = rightmost_seed_x {
        let strip_offset = prob.strip().shape_modify_config.offset.unwrap_or(0.0);
        required_width = required_width.max((rightmost_seed_x + strip_offset) * 1.00001);
    }
    if preferred_width.is_finite() && preferred_width > 0.0 {
        required_width = required_width.max(preferred_width);
    }
    if required_width > prob.strip_width() {
        prob.change_strip_width(required_width).expect(swarm_core::dmsg!("strip width must be positive"));
    }
    let boxes = placements
        .iter()
        .map(|seed| {
            instance
                .item(seed.placement.item_idx)
                .shape_cd()
                .transform_clone(&seed.placement.d_transf.compose())
                .bbox()
        })
        .collect::<Vec<_>>();
    crate::util::frozen::prepare_placements(&mut prob, boxes);
    for seed in &placements {
        prob.place_item(seed.placement);
    }
    Ok(prob)
}

/// The largest inward shift applied to a hinted piece whose collision shape,
/// but not its original outline, crosses the strip edge.
const SEED_EDGE_NUDGE_MAX_MM: f32 = 2.0;

/// A hint placed flush against the strip edge (e.g. `y = 0`) is inside the
/// strip by its original outline, but the collision shape is inflated by
/// polygon simplification and piece separation, so it crosses the edge by a
/// fraction of a millimetre. Rejecting the hint threw away the whole warm
/// start and rebuilt the layout from scratch. Shift such a piece inward by
/// the overshoot instead; any micro-overlap this creates is left to the local
/// seed repair. Pieces whose original outline is itself outside, whose shape
/// does not fit the strip height, or that need more than
/// [`SEED_EDGE_NUDGE_MAX_MM`] are left alone (and still rejected).
fn nudge_seed_into_strip(
    instance: &SPInstance,
    prob: &SPProblem,
    placement: SPPlacement,
    shape: &jagua_rs::geometry::primitives::SPolygon,
) -> Option<SPPlacement> {
    use jagua_rs::geometry::DTransformation;
    use jagua_rs::geometry::geo_traits::Transformable;
    use jagua_rs::io::export::int_to_ext_transformation;

    let container = &prob.layout().container();
    let cd = container.outer_cd().bbox();
    let bbox = shape.bbox();
    if bbox.height() > cd.height() {
        return None;
    }
    let dx = (cd.x_min - bbox.x_min).max(0.0);
    let dy = if bbox.y_min < cd.y_min {
        cd.y_min - bbox.y_min
    } else if bbox.y_max > cd.y_max {
        cd.y_max - bbox.y_max
    } else {
        0.0
    };
    if dx == 0.0 && dy == 0.0 {
        return None;
    }
    if dx > SEED_EDGE_NUDGE_MAX_MM || dy.abs() > SEED_EDGE_NUDGE_MAX_MM {
        return None;
    }
    // Only when the user's original outline really is inside the strip.
    let item = instance.item(placement.item_idx);
    let ext = int_to_ext_transformation(&placement.d_transf, &item.shape_orig().pre_transform);
    let original = item.shape_orig().shape.transform_clone(&ext.compose()).bbox();
    let strip_ext = int_to_ext_transformation(
        &DTransformation::empty(),
        &container.outer_orig().pre_transform,
    );
    let strip = container.outer_orig().shape.transform_clone(&strip_ext.compose()).bbox();
    if !crate::structure::pipeline::bbox_is_contained_mm(
        [strip.x_min, strip.x_max.max(original.x_max), strip.y_min, strip.y_max],
        [original.x_min, original.x_max, original.y_min, original.y_max],
    ) {
        return None;
    }
    // A hair past the overshoot so float rounding cannot leave it on the edge.
    let margin = |d: f32| if d == 0.0 { 0.0 } else { d + d.signum() * 1.0e-3 };
    let (x, y) = placement.d_transf.translation();
    Some(SPPlacement {
        item_idx: placement.item_idx,
        d_transf: DTransformation::new(
            placement.d_transf.rotation(),
            (x + margin(dx), y + margin(dy)),
        ),
    })
}

/// 시드 배치를 놓고 **exact 검증까지 통과한** 문제만 돌려준다.
fn legacy_seed_problem(
    instance: &SPInstance,
    placements: &[LegacySeedPlacement],
    preferred_width: f32,
) -> Result<SPProblem, String> {
    let prob = legacy_seed_problem_raw(instance, placements, preferred_width, true)?;
    if let Some(error) =
        crate::structure::pipeline::partial_solution_validation_error_with_frozen_ids(
            instance,
            &prob.save(),
            &placements
                .iter()
                .filter(|s| s.frozen)
                .map(|s| s.placement.item_idx)
                .collect::<Vec<_>>(),
        )
    {
        return Err(format!("seed layout is not exact-feasible: {error:?}"));
    }
    Ok(prob)
}

/// 겹침이 남은 시드를 **버리지 않고** 분리기로 푼다.
///
/// # 왜 필요한가
///
/// 상용 마커를 변환한 입력은 0.025mm 격자 양자화 때문에 접촉면에 미세 관통이 남는다
/// (wsjk156: 49쌍, 최대 0.0954mm). 게다가 CDE 가 충돌 도형을 부풀린다. 그래서 힌트가
/// 지정한 배치는 exact 검증을 통과하지 못한다.
///
/// 예전에는 그때 시드를 통째로 버리고 frozen 힌트만 남겨 다시 지었다. 그런데 힌트가
/// 전부 `is_freeze: 0` 이면 frozen 집합이 비어 있어 **156개가 전부 사라지고 LBF 가
/// 처음부터 다시 배치**했다. 사용자가 배치 좌표를 준 의미가 없어진다.
///
/// 겹침을 푸는 것은 분리기가 존재하는 이유 그 자체다. 그러니 좌표는 그대로 쓰고
/// 겹침만 그 자리에서 해소한 뒤 넘긴다 — 상용 배치의 **구조**가 보존된다.
///
/// frozen 힌트가 하나라도 있으면 쓰지 않는다. 분리기는 고정 여부를 모르므로
/// 고정 조각을 움직여 버린다.
fn legacy_seed_problem_separated(
    instance: &SPInstance,
    placements: &[LegacySeedPlacement],
    preferred_width: f32,
    separator_config: crate::optimizer::separator::SeparatorConfig,
    rng: Xoshiro256PlusPlus,
    budget: Duration,
) -> Result<SPProblem, String> {
    // 경계 검사는 그대로 지킨다. 조각이 원단 폭을 넘는 배치는 분리기가 고정 폭에서
    // 풀 수 없다 — 부풀린 충돌 도형이 원단보다 큰 것이라 밀어 넣을 자리 자체가 없다.
    // 그런 입력에 분리 예산을 쓰는 것은 낭비이므로 여기서 바로 실패시킨다.
    if init_cancelled() {
        return Err("warm seed separation cancelled".to_string());
    }
    let prob = legacy_seed_problem_raw(instance, placements, preferred_width, true)?;
    if init_cancelled() {
        return Err("warm seed separation cancelled".to_string());
    }
    let mut sep = Separator::new(instance.clone(), prob, rng, separator_config);
    let entry_loss = sep.ct.get_total_loss();
    let mut term = BasicTerminator::new();
    term.new_timeout(budget);
    let term = InitCancelTerminator::new(term);
    if legacy_seed_local_repair(&mut sep, &term)? {
        return Ok(sep.prob);
    }
    if init_cancelled() {
        return Err("warm seed separation cancelled".to_string());
    }
    let (solution, snapshot) = sep.separate(&term, &mut None);
    if init_cancelled() {
        return Err("warm seed separation cancelled".to_string());
    }
    let residual = snapshot.get_total_loss();
    if residual != 0.0 {
        return Err(format!(
            "seed layout still overlaps after separation (entry loss {entry_loss:.3} -> {residual:.3})"
        ));
    }
    let mut resolved = SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
    crate::util::frozen::restore(&mut resolved, &solution);
    if let Some(error) =
        crate::structure::pipeline::partial_solution_validation_error(instance, &resolved.save())
    {
        return Err(format!(
            "separated seed layout is still not exact-feasible: {error:?}"
        ));
    }
    Ok(resolved)
}

/// Correct only tiny hint penetrations before sampling the whole container.
/// Trials retain placement identity (including repeated item IDs), and consume
/// no RNG. Failure restores the original keys, geometry and tracker together.
/// Wall-clock budget of the local seed repair. Debug-assertion builds re-verify
/// the whole tracker and quadtree on every trial move, which makes a trial tens
/// of times slower; give them room so tests exercise the same outcome as the
/// release library instead of timing out into the full separation.
const LOCAL_REPAIR_BUDGET: Duration = if cfg!(debug_assertions) {
    Duration::from_secs(5)
} else {
    Duration::from_millis(100)
};

fn legacy_seed_local_repair(
    sep: &mut Separator,
    term: &impl Terminator,
) -> Result<bool, String> {
    if sep.has_place_groups() || sep.frozen_mask.iter().any(|&frozen| frozen) {
        return Ok(false);
    }
    let started = std::time::Instant::now();
    legacy_seed_local_repair_with_elapsed(sep, term, || started.elapsed())
}

/// Keep elapsed time separate from candidate evaluation so identity and
/// rollback tests can exercise the same body without scheduling-dependent time.
fn legacy_seed_local_repair_with_elapsed(
    sep: &mut Separator,
    term: &impl Terminator,
    mut elapsed: impl FnMut() -> Duration,
) -> Result<bool, String> {
    let entry_prob = sep.prob.clone();
    let entry_ct = sep.ct.clone();
    let mut anchors = sep.prob.layout().placed_items().iter()
        .map(|(key, placed)| (key, placed.d_transf())).collect::<Vec<_>>();
    let mut evaluations = 0usize;
    let mut interrupted = || {
        let timeout = term.kill() || elapsed() >= LOCAL_REPAIR_BUDGET;
        if init_cancelled() { Err("warm seed separation cancelled".to_string()) }
        else { Ok(timeout) }
    };
    let outcome = (|| {
        for _ in 0..2 {
            for (key, anchor) in &mut anchors {
                if interrupted()? { return Ok(false); }
                if sep.ct.get_loss(*key) == 0.0 { continue; }
                // Trials move one item and move it back. The tracker recomputes
                // that item's losses from geometry and the total is re-summed
                // from scratch, so reverting restores the exact same state
                // without cloning the whole problem and tracker per trial
                // (the clones dominated the 100 ms budget on large layouts).
                let pose = sep.prob.layout().placed_items()[*key].d_transf();
                let (x, y) = pose.translation();
                let (ax, ay) = anchor.translation();
                let mut best: Option<(f32, f32)> = None;
                let mut best_loss = sep.ct.get_total_loss();
                // Origin-relative axis proposals let a later sweep undo a
                // coarse translation without walking a floating-point grid.
                let mut candidates = Vec::with_capacity(64);
                for offset in [0.00001f32, 0.0001] {
                    candidates.extend([(ax+offset,y),(ax-offset,y),(x,ay+offset),(x,ay-offset)]);
                }
                for step in [0.00001f32, 0.0001, 0.0005, 0.005, 0.05, 0.1, 0.25] {
                    for (dx, dy) in [(1.,0.),(-1.,0.),(0.,1.),(0.,-1.),
                        (1.,1.),(1.,-1.),(-1.,1.),(-1.,-1.)] {
                        let candidate = (x+dx*step,y+dy*step);
                        if !candidates.contains(&candidate) { candidates.push(candidate); }
                    }
                }
                for (tx, ty) in candidates {
                        if interrupted()? || evaluations >= 512 { return Ok(false); }
                        if (tx - ax).abs() > 0.25 || (ty - ay).abs() > 0.25 { continue; }
                        evaluations += 1;
                        let moved_key = sep.move_item(*key,
                            jagua_rs::geometry::DTransformation::new(pose.rotation(), (tx, ty)));
                        let loss = sep.ct.get_total_loss();
                        if sep.ct.get_container_loss(moved_key) == 0.0 && loss < best_loss {
                            if loss == 0.0 {
                                let error = crate::structure::pipeline::partial_solution_validation_error(
                                    &sep.instance, &sep.prob.save());
                                let certified = error.is_none();
                                if interrupted()? { return Ok(false); }
                                if certified { return Ok(true); }
                            } else {
                                best_loss = loss;
                                best = Some((tx, ty));
                            }
                        }
                        *key = sep.move_item(moved_key, pose);
                }
                if let Some((tx, ty)) = best {
                    *key = sep.move_item(*key,
                        jagua_rs::geometry::DTransformation::new(pose.rotation(), (tx, ty)));
                }
            }
        }
        Ok(false)
    })();
    if !matches!(outcome, Ok(true)) {
        sep.prob = entry_prob;
        sep.ct = entry_ct;
    }
    outcome
}

fn legacy_ext_seed_placements(
    instance: &SPInstance,
    solution: &ExtSPSolution,
) -> Result<Vec<LegacySeedPlacement>, String> {
    use jagua_rs::geometry::DTransformation;
    use jagua_rs::io::import::ext_to_int_transformation;

    if !solution.strip_width.is_finite() || solution.strip_width <= 0.0 {
        return Err("seed solution has an invalid strip width".to_string());
    }
    solution
        .layout
        .placed_items
        .iter()
        .map(|placed| {
            let item_id = usize::try_from(placed.item_id)
                .map_err(|_| format!("unknown seeded item {}", placed.item_id))?;
            if item_id >= instance.items.len() {
                return Err(format!("unknown seeded item {}", placed.item_id));
            }
            let ext = &placed.transformation;
            if !ext.rotation.is_finite()
                || !ext.translation.0.is_finite()
                || !ext.translation.1.is_finite()
            {
                return Err(format!("seeded item {item_id} has a non-finite transform"));
            }
            let ext_transf = DTransformation::from(ext.clone());
            let d_transf = ext_to_int_transformation(
                &ext_transf,
                &instance.item(item_id).shape_orig().pre_transform,
            );
            Ok(LegacySeedPlacement {
                placement: SPPlacement { item_idx: item_id, d_transf },
                frozen: false,
            })
        })
        .collect()
}

/// Make room for one more seed of `item_id`.
///
/// Only when the kind is already saturated (as many seeds as the instance demands) is the
/// oldest seed of that kind dropped; a repeated kind with spare demand keeps every seed, so
/// several hints for the same `item_id` all survive instead of collapsing to the last one.
fn collapse_saturated_seed_kind(
    instance: &SPInstance,
    seed_placements: &mut Vec<LegacySeedPlacement>,
    item_id: usize,
) {
    let same_kind = seed_placements
        .iter()
        .filter(|seed| seed.placement.item_idx == item_id)
        .count();
    if same_kind < instance.item_qty(item_id) {
        return;
    }
    if let Some(index) = seed_placements
        .iter()
        .position(|seed| seed.placement.item_idx == item_id)
    {
        seed_placements.remove(index);
    }
}

fn legacy_hint_placement(instance: &SPInstance, hint: &CPlacedHint) -> Result<SPPlacement, String> {
    use jagua_rs::geometry::DTransformation;
    use jagua_rs::io::import::ext_to_int_transformation;

    if !hint.tr_x.is_finite()
        || !hint.tr_y.is_finite()
        || !hint.rot_angle_in_rad.is_finite()
        || !hint.stripe_interval_x.is_finite()
        || !hint.stripe_interval_y.is_finite()
    {
        return Err(format!(
            "hinted item {} has a non-finite value",
            hint.item_id
        ));
    }
    let item_id = hint.item_id as usize;
    let item = instance.item(item_id);
    let raw_bbox = item.shape_orig().shape.bbox();
    let (cos, sin) = (hint.rot_angle_in_rad.cos(), hint.rot_angle_in_rad.sin());
    let ext_transf = DTransformation::new(
        hint.rot_angle_in_rad,
        (
            hint.tr_x - (cos * raw_bbox.x_min - sin * raw_bbox.y_min),
            hint.tr_y - (sin * raw_bbox.x_min + cos * raw_bbox.y_min),
        ),
    );
    let d_transf = ext_to_int_transformation(&ext_transf, &item.shape_orig().pre_transform);
    Ok(SPPlacement { item_idx: item_id, d_transf })
}

pub(crate) fn build_grouped_handle(
    input: grouped_input::ValidatedInput,
    config: crate::config::SwarmConfig,
    callback_options: CallbackOptions,
) -> Result<*mut SwarmStepHandle, String> {
    let progress_callback_interval = config.structure.progress_callback_interval;
    let solver = GroupedStepState::new(input, config)?;
    let grouped = GroupedHandleState {
        solver,
        solution_callback: None,
        callback_user_data: 0,
        emit_intermediate_json: false,
        callback_options,
        progress_callback_interval,
    };
    let handle = Box::new(SwarmStepHandle {
        engine: Mutex::new(HandleEngine::Grouped(grouped)),
        lifecycle: HandleLifecycle::new(),
    });
    Ok(Box::into_raw(handle))
}

/// 공통 init 로직: ExtSPInstance + Option<ExtSPSolution> → SwarmStepHandle
fn build_handle(
    mut ext_inst: ExtSPInstance,
    ext_sol: Option<ExtSPSolution>,
    config: crate::config::SwarmConfig,
    callback_options: CallbackOptions,
    initial_config: crate::ffi::InitialPlacementSettings,
    hints: Vec<CPlacedHint>,
) -> Option<*mut SwarmStepHandle> {

    #[cfg(feature = "init-profile")]
    let mut _profile_phase = swarm_core::init_profile::Scope::new("phase.setup");
    // 결방향 없는 원단: 허용 회전에 180° 반전을 추가한다 (import 전에 적용해야 한다)
    if config.allow_180_rotation {
        crate::util::io::augment_orientations_180(&mut ext_inst);
    }
    let mut rng: Xoshiro256PlusPlus = match config.rng_seed {
        Some(s) => Xoshiro256PlusPlus::seed_from_u64(s as u64),
        None => Xoshiro256PlusPlus::seed_from_u64(rand::random()),
    };
    // Jagua simplifies item collision polygons by *inflating* them and the
    // strip polygon by *deflating* it.  The simplified search geometry is
    // therefore conservative: a feasible layout cannot become invalid only
    // because omitted source vertices are restored.  Structure can keep the
    // configured simplification instead of paying the full source-vertex cost
    // throughout P0-P5.
    // 겹침 허용(`min_item_separation < 0`)일 때 **원단 경계를 열지 않는** import.
    // jagua 의 `import_instance` 를 직접 쓰면 컨테이너가 함께 부풀어 조각의 실제
    // 재단 도형이 원단 밖으로 나간다 — 근거는 `util::io::import_spp_instance` 문서.

    #[cfg(feature = "init-profile")]
    _profile_phase.next("phase.geometry");
    let mut instance = crate::util::io::import_spp_instance_with_mm(
        &ext_inst,
        config.cde_config,
        config.poly_simpl_tolerance,
        config.min_item_separation,
        config.narrow_concavity_cutoff_ratio,
        config.simplify_max_deviation_mm,
    )
    .ok()?;
    swarm_core::original_overlap::apply_policy(&mut instance, config.original_overlap_policy);
    if init_cancelled() {
        return None;
    }


    #[cfg(feature = "init-profile")]
    _profile_phase.next("phase.seed_setup");
    // 힌트 분류.
    //
    // **좌표는 `is_freeze`와 무관하게 항상 쓰인다.** 힌트를 보냈다는 것 자체가
    // "이 아이템을 여기서 시작하라"는 뜻이다. `is_freeze`는 그 위에 얹히는 별개의 제약
    // ("여기서 움직이지 마라")일 뿐이다. 두 개념을 분리한 덕에:
    //   - 힌트 없음            → LBF가 전부 배치 (기존과 동일)
    //   - 힌트 있음 + freeze 0 → 그 좌표에서 출발하되 탐색이 자유롭게 옮긴다 (warm start)
    //   - 힌트 있음 + freeze 1 → 그 좌표에 고정
    //   - 일부만 힌트          → 나머지만 LBF가 채운다
    let n_items = instance.items.len();
    let mut freeze_ids = Vec::new();
    let mut pre_placed = Vec::new();

    for h in &hints {
        // 범위를 벗어난 item_id는 무시한다 (stripe interval 전달용 더미 힌트 등).
        // 아래 `instance.item()`은 잘못된 id에 패닉하므로 여기서 걸러야 한다.
        if h.item_id < 0 || h.item_id as usize >= n_items {
            continue;
        }
        if h.is_freeze != 0 {
            freeze_ids.push(h.item_id as usize);
        }
        pre_placed.push(*h);
    }
    let requested_input_seed = ext_sol.is_some() || !pre_placed.is_empty();
    let mut seed_placements = Vec::<LegacySeedPlacement>::new();
    let mut invalid_movable_seed = false;
    let preferred_width = ext_sol.as_ref().map_or(0.0, |sol| sol.strip_width);
    if let Some(solution) = &ext_sol {
        match legacy_ext_seed_placements(&instance, solution) {
            Ok(placements) => seed_placements = placements,
            Err(error) => {
                log::warn!("[INIT] discarding invalid movable solution seed: {error}");
                invalid_movable_seed = true;
            }
        }
    }

    // A hint overrides one placement of the same item kind only once that kind
    // is already saturated (`item_qty` copies seeded), matching the previous
    // Legacy behavior for demand 1 while letting several hints of a repeated
    // kind coexist. Conversion happens before any Layout mutation so an
    // invalid movable hint can be discarded and an invalid frozen hint can
    // fail cleanly rather than leaving a partial CDE behind.
    for hint in &pre_placed {
        match legacy_hint_placement(&instance, hint) {
            Ok(placement) => {
                collapse_saturated_seed_kind(&instance, &mut seed_placements, placement.item_idx);
                seed_placements.push(LegacySeedPlacement {
                    placement,
                    frozen: hint.is_freeze != 0,
                });
            }
            Err(error) if hint.is_freeze != 0 => {
                set_last_error(2001, format!("invalid frozen Legacy Init hint: {error}"));
                return None;
            }
            Err(error) => {
                log::warn!("[INIT] discarding invalid movable hint: {error}");
                invalid_movable_seed = true;
            }
        }
    }

    freeze_ids.sort_unstable();
    freeze_ids.dedup();
    let frozen: std::collections::HashSet<usize> = freeze_ids.iter().copied().collect();
    for seed in &mut seed_placements {
        seed.frozen = frozen.contains(&seed.placement.item_idx);
    }
    let frozen_placements = seed_placements
        .iter()
        .copied()
        .filter(|seed| seed.frozen)
        .collect::<Vec<_>>();

    // 복구된 시드는 좌표는 사용자 것이지만 **우리가 손댄** 배치다. 시작 레이아웃으로는
    // 쓰되, "사용자가 인증한 warm start" 자격은 주지 않는다 — Structure P1 을 건너뛰는
    // 지름길은 손대지 않은 시드에만 열려 있어야 한다.
    let mut seed_was_repaired = false;
    let (start_prob, mut started_from_frozen_only) = if invalid_movable_seed {
        match legacy_seed_problem(&instance, &frozen_placements, 0.0) {
            Ok(prob) => (prob, true),
            Err(error) => {
                set_last_error(
                    2001,
                    format!("invalid frozen Legacy Init baseline: {error}"),
                );
                return None;
            }
        }
    } else {
        match legacy_seed_problem(&instance, &seed_placements, preferred_width) {
            Ok(prob) => (prob, false),
            Err(seed_error) => {
                // 좌표는 사용자가 준 것이다. 미세 겹침 때문에 통째로 버리기 전에
                // 그 자리에서 겹침만 풀어 본다 (frozen 힌트가 없을 때만).
                let repaired = frozen_placements.is_empty().then(|| {
                    legacy_seed_problem_separated(
                        &instance,
                        &seed_placements,
                        preferred_width,
                        config.expl_cfg.separator_config,
                        next_rng(&mut rng),
                        SEED_SEPARATION_BUDGET,
                    )
                });
                match repaired {
                    Some(Ok(prob)) => {
                        seed_was_repaired = true;
                        (prob, false)
                    }
                    other => {
                        if let Some(Err(repair_error)) = other {
                            log::warn!(
                                "[INIT] could not separate the hinted layout: {repair_error}"
                            );
                        }
                        // 여기서 조용히 LBF 로 떨어지면 사용자는 "Use Hints 를 켰는데
                        // 왜 무시되지?" 만 보게 된다. 이유와 조치를 함께 남긴다.
                        log::warn!(
                            "[INIT] 힌트 배치를 쓸 수 없어 버립니다 ({} 개) — 초기 배치를                              처음부터 다시 만듭니다. 사유: {seed_error}",
                            seed_placements.len()
                        );
                        log::warn!(
                            "[INIT] 원단 경계 위반이 사유라면 PolySimpl 을 음수(끄기)로,                              조각 겹침이 사유라면 Piece Overlap 을 관통 깊이보다 크게                              (상용 마커 변환본은 0.1mm 면 충분) 설정하십시오."
                        );
                        match legacy_seed_problem(&instance, &frozen_placements, 0.0) {
                            Ok(prob) => (prob, true),
                            Err(error) => {
                                set_last_error(
                                    2001,
                                    format!("invalid frozen Legacy Init baseline: {error}"),
                                );
                                return None;
                            }
                        }
                    }
                }
            }
        }
    };

    if init_cancelled() {
        return None;
    }

    #[cfg(feature = "init-profile")]
    _profile_phase.next("phase.placement");
    let mut construct = |start_prob: SPProblem| {
        LBFBuilder::from_problem(
            instance.clone(),
            start_prob,
            next_rng(&mut rng),
            LBF_SAMPLE_CONFIG,
            config.lbf_x_multiplier,
            config.lbf_y_multiplier,
        )
        .with_order_mode(config.lbf_order)
        .with_raster_config(config.lbf_raster)
        .with_band_width(config.init_band_width_mm)
        .with_band_fill_ratio(config.init_band_fill_ratio)
        .with_frozen_items(&freeze_ids)
        // 배치 도중 다지기. `init_settle_every == 0` 이면 `None` 이라 기존 경로와
        // 한 비트도 다르지 않다.
        .with_settle((config.init_settle_every > 0).then(|| {
            swarm_core::optimizer::lbf::SettleDuringConstruction {
                every: config.init_settle_every,
                budget_ms: config.init_settle_budget_ms,
                lp: config.polish_cfg.lp,
                hole_fill: config.polish_cfg.hole_fill,
                separator: config.expl_cfg.separator_config,
            }
        }))
        .construct()
        .prob
    };
    let use_column_draft = config.refplace.enabled
        && config.refplace.column_draft
        && !requested_input_seed
        && frozen_placements.is_empty();
    let mut has_column_draft = false;
    let mut prob = if use_column_draft {
        match swarm_core::refplace::columns::build_column_draft(
            &instance,
            swarm_core::refplace::columns::ColumnDraftConfig {
                preserve_input_contour: config.simplify_max_deviation_mm.is_some(),
                ..Default::default()
            },
        ) {
            Ok((solution, _)) => {
                has_column_draft = true;
                let mut drafted = SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
                drafted.restore(&solution);
                drafted
            }
            Err(error) => {
                log::warn!("[INIT] Column Draft failed: {error}; falling back to LBF");
                construct(start_prob)
            }
        }
    } else {
        construct(start_prob)
    };
    // Stop 이 Init 도중에 들어오면 여기서 끝낸다. 구성 루프는 조각 경계에서 멈추므로
    // `prob` 는 부분 배치일 수 있다 — 검증으로 가지 않고 바로 취소로 돌려준다.
    if init_cancelled() {
        return None;
    }


    #[cfg(feature = "init-profile")]
    _profile_phase.next("phase.original_repair");
    // Establish the original-certified incumbent before comparing optional
    // constructors. Raw LBF width may grow during original-overlap repair.
    let mut original_term = crate::util::terminator::BasicTerminator::new();
    original_term.new_timeout(SEED_SEPARATION_BUDGET);
    let original_term = InitCancelTerminator::new(original_term);
    let _ = swarm_core::original_overlap::repair_problem(
        &mut prob,
        &freeze_ids,
        &crate::optimizer::place_groups::PlaceGroups::default(),
        &original_term,
    );

    #[cfg(feature = "init-profile")]
    _profile_phase.next("phase.validation_incumbent");
    let mut incumbent_certified =
        crate::structure::pipeline::solution_validation_error_with_frozen_ids(
            &instance,
            &prob.save(),
            &freeze_ids,
        )
        .is_none();


    #[cfg(feature = "init-profile")]
    _profile_phase.next("phase.optional_constructors");
    // ── NFP-LBF 초기 배치 시드 ────────────────────────────────────────────
    //
    // NFP Cache -> Multiple Item Ordering -> NFP Sequential Placement 로 만든
    // Optional constructors compete with the complete certified incumbent.
    // Explicit Column Draft remains the primary fresh constructor above.
    if config.refplace.enabled
        && !config.refplace.column_draft
        && !requested_input_seed
        && frozen_placements.is_empty()
    {
        let params = swarm_core::refplace::RefPlaceParams::default();
        match swarm_core::refplace::build_reference_nest(&instance, &params, None) {
            Some((solution, _)) => {
                let incumbent = prob.save();
                let accepted = if incumbent_certified {
                    crate::initial::incumbent::candidate_is_improvement(
                        &instance,
                        &solution,
                        &incumbent,
                        &freeze_ids,
                        &crate::optimizer::place_groups::PlaceGroups::default(),
                    )
                } else {
                    crate::structure::pipeline::solution_validation_error_with_frozen_ids(
                        &instance,
                        &solution,
                        &freeze_ids,
                    )
                    .is_none()
                };
                if accepted {
                    let mut seeded = SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
                    seeded.restore(&solution);
                    prob = seeded;
                    incumbent_certified = true;
                }
            }
            None => log::warn!("[INIT] refplace(실험)가 해를 내지 못했다 — 기존 배치를 유지한다"),
        }
    }

    // Best Initial Nest 가 기존 LBF 결과보다 좁으면 그것으로 갈아 끼운다.
    //
    // 이 경로(step API)는 WPF 샘플이 쓴다. 예전에는 `optimize_configured` 에만 배선되어
    // 있어서 WPF 에서 NFP 를 켜도 초기 배치가 전혀 바뀌지 않았다.
    //
    // 실패하거나 더 넓으면 조용히 기존 LBF 결과를 유지한다 — 시드는 어디까지나 보조다.
    if init_cancelled() {
        return None;
    }
    if config.nfp_lbf.enabled
        && !use_column_draft
        && !requested_input_seed
        && frozen_placements.is_empty()
    {
        let nfp_cfg = config.nfp_lbf.to_config(config.nfp_strategy);
        // `construct` 클로저가 `rng` 를 가변 대여 중이므로 여기서는 새 스트림을 판다.
        // 시드 생성은 결정론적이어야 하니 고정 파생 상수를 쓴다.
        let mut seed_rng = Xoshiro256PlusPlus::seed_from_u64(
            swarm_core::protected_seed::nfp_lbf(config.rng_seed.unwrap_or(0) as u64),
        );
        match swarm_core::nfp_lbf::build_best_initial_nest(&instance, &nfp_cfg, &mut seed_rng) {
            Some(nest) => {
                let lbf_width = prob.strip_width();
                let nfp_width = nest.solution.strip_width();
                let nfp_valid =
                    crate::structure::pipeline::solution_validation_error_with_frozen_ids(
                        &instance,
                        &nest.solution,
                        &freeze_ids,
                    )
                    .is_none();
                // 유효한 시드를 채택할지는 초기 폭이 아니라 **짧은 탐색 뒤의 폭**으로 정한다
                // (`seed_duel_prefers_candidate`). 초기 폭이 한참 넓은 시드만 대결 없이 버린다.
                let accepted = if !incumbent_certified {
                    nfp_valid
                } else {
                    nfp_valid
                        && nfp_width <= lbf_width * (1.0 + SEED_DUEL_MAX_GAP)
                        && seed_duel_prefers_candidate(
                            &instance,
                            &nest.solution,
                            &prob.save(),
                            &config,
                        )
                };
                if accepted {
                    log::info!(
                        "[INIT] NFP-LBF 시드 채택: 폭 {:.3} -> {:.3} (밀도 {:.2}%, ordering '{}', {} 평가, {} ms)",
                        lbf_width,
                        nfp_width,
                        nest.report.best_density * 100.0,
                        nest.report.best_ordering_label,
                        nest.report.orderings_evaluated,
                        nest.report.elapsed_ms,
                    );
                    let mut seeded = SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
                    seeded.restore(&nest.solution);
                    prob = seeded;
                } else {
                    log::info!(
                        "[INIT] NFP-LBF 시드 기각: 후보 폭 {:.3}, 기존 폭 {:.3} — {}",
                        nfp_width,
                        lbf_width,
                        if !nfp_valid {
                            "인증 실패"
                        } else if incumbent_certified
                            && nfp_width > lbf_width * (1.0 + SEED_DUEL_MAX_GAP)
                        {
                            "초기 폭이 3% 넘게 넓다"
                        } else {
                            "시드 대결에서 기존 배치가 더 좁아졌다"
                        },
                    );
                }
            }
            None => {
                log::warn!("[INIT] NFP-LBF 시드를 만들지 못했다 — 기존 LBF 배치를 유지한다");
            }
        }
    }


    #[cfg(feature = "init-profile")]
    _profile_phase.next("phase.validation_final_gate");
    // Optional candidates were certified before adoption; retain the final gate.
    let mut baseline_error = crate::structure::pipeline::solution_validation_error_with_frozen_ids(
        &instance,
        &prob.save(),
        &freeze_ids,
    );
    if baseline_error.is_some() && requested_input_seed && !started_from_frozen_only {
        log::warn!("[INIT] retrying invalid completed seed from the frozen-only baseline");
        let frozen_prob = match legacy_seed_problem(&instance, &frozen_placements, 0.0) {
            Ok(prob) => prob,
            Err(error) => {
                set_last_error(
                    2001,
                    format!("invalid frozen Legacy Init baseline: {error}"),
                );
                return None;
            }
        };
        prob = construct(frozen_prob);
        started_from_frozen_only = true;
        let _ = swarm_core::original_overlap::repair_problem(
            &mut prob,
            &freeze_ids,
            &crate::optimizer::place_groups::PlaceGroups::default(),
            &original_term,
        );
        baseline_error = crate::structure::pipeline::solution_validation_error_with_frozen_ids(
            &instance,
            &prob.save(),
            &freeze_ids,
        );
    }
    if init_cancelled() {
        return None;
    }
    if let Some(error) = baseline_error {
        set_last_error(
            2001,
            format!("Legacy Init baseline is not exact-feasible: {error:?}"),
        );
        return None;
    }


    #[cfg(feature = "init-profile")]
    _profile_phase.next("phase.postprocess");
    // Snap items to stripe grid (best-effort post-processing)
    let stripe_y = hints
        .iter()
        .map(|h| h.stripe_interval_y)
        .find(|&v| v.is_finite() && v > 0.0);
    let stripe_x = hints
        .iter()
        .map(|h| h.stripe_interval_x)
        .find(|&v| v.is_finite() && v > 0.0);
    for (interval, axis) in [(stripe_y, Axis::Y), (stripe_x, Axis::X)] {
        let Some(interval) = interval else { continue };
        let _ = snap_to_stripe(&mut prob, interval, axis, &frozen);
    }

    // ── 초기 구성 다지기 ────────────────────────────────────────────────
    //
    // Hole fill 과 LP 압축은 지금까지 Polish 에서만 돌았다. 참조 네스터의 77% 초기
    // 배치 화면을 보면 **구성 도중에** 작은 조각이 큰 조각의 빈 속에 들어가 있고
    // 전체가 다져져 있다 — 그쪽은 이 둘을 구성 단계에서 쓴다는 뜻이다.
    //
    // 여기서는 구성이 끝난 직후 같은 두 패스를 한 번씩 돌린다. 둘 다 실패해도
    // 배치는 유효한 상태로 남고(각 패스가 자체 검증을 한다), 아래 인증이 최종
    // 판정을 하므로 이 단계가 결과를 무효로 만들 수는 없다. 기본은 꺼짐이다.
    if config.init_compaction_ms > 0 {
        compact_initial_layout(&instance, &mut prob, &freeze_ids, &config);
    }
    if init_cancelled() {
        return None;
    }


    #[cfg(feature = "init-profile")]
    _profile_phase.next("phase.validation_export");
    let init_solution = prob.save();
    if let Some(error) = crate::structure::pipeline::solution_validation_error_with_frozen_ids(
        &instance,
        &init_solution,
        &freeze_ids,
    ) {
        set_last_error(
            2001,
            format!("Legacy Init baseline is not exact-feasible after stripe snap: {error:?}"),
        );
        return None;
    }

    #[cfg(feature = "init-profile")]
    _profile_phase.next("phase.handle");
    // Only an accepted movable/complete input seed may bypass Structure P1.
    // If Init discarded it and rebuilt from frozen-only (or fresh) state, the
    // resulting LBF preview is not an authoritative user warm start.
    let has_input_seed = requested_input_seed
        && !started_from_frozen_only
        && !seed_was_repaired
        && !seed_placements.is_empty();
    let legacy = LegacyStepState {
        instance,
        prob,
        rng,
        expl_config: config.expl_cfg,
        cmpr_config: config.cmpr_cfg,
        plsh_config: config.polish_cfg,
        structure_config: config.structure,
        lbf_x_multiplier: config.lbf_x_multiplier,
        lbf_y_multiplier: config.lbf_y_multiplier,
        lbf_raster: config.lbf_raster,
        lbf_order: config.lbf_order,
        escape_enabled: config.escape_enabled,
        escape_config: config.escape,
        nfp_integration: config.nfp_integration,
        lp_solver: config.lp_solver,
        nfp_lbf: config.nfp_lbf,
        phase: Phase::Init,
        solutions: vec![],
        solution: Some(init_solution),
        solution_callback: None,
        callback_user_data: 0,
        emit_intermediate_json: false,
        callback_options,
        frozen_item_ids: freeze_ids,
        fabric_edge_margin_mm: config.fabric_edge_margin_mm,
        initial_config,
        has_input_seed,
        has_column_draft,
        structure_report: None,
    };
    let handle = Box::new(SwarmStepHandle {
        engine: Mutex::new(HandleEngine::Legacy(legacy)),
        lifecycle: HandleLifecycle::new(),
    });
    Some(Box::into_raw(handle))
}

// -- Swarm N06 초기배치 단계

/// N06 초기배치 페이즈. 성공하면 0, 실패하면 0이 아닌 값을 반환.
///
/// `swarm_init(_from_json)` 직후(Phase=Init)에만 유효하다. LBF 초기해를
/// N06 엔진(GRASP 구성 → 슬라이드 재압축 → 점진 축소·분리·kick) 결과로 교체하며,
/// 종료 후에도 Phase=Init 을 유지해 `swarm_alg_phase` /
/// `swarm_cp_phase` 를 그대로 이어 붙일 수 있다.
/// Grouped(V2) 입력은 지원하지 않는다 (3004).
/// `swarm_request_stop` 으로 협조적으로 중단할 수 있고, 중단 시에도 그때까지의
/// 최선 해가 (정합 검증을 통과하면) 채택된다.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_init_phase(handle: *mut SwarmStepHandle) -> i32 {
    if handle.is_null() {
        return -1;
    }

    clear_last_error();
    let handle = unsafe { &*handle };
    let _run_scope = swarm_core::config::runtime::enter(handle.lifecycle.run_control.clone());
    let Some(_phase_call) = handle.lifecycle.begin_phase() else {
        set_last_error(
            3000,
            "Initial phase rejected: handle is busy, closing, or stopped",
        );
        return -1;
    };
    let mut engine = handle.lock_engine();
    if matches!(&*engine, HandleEngine::Grouped(_)) {
        set_last_error(
            3004,
            "Initial placement phase supports Legacy (single-strip) inputs only",
        );
        return -1;
    }
    let h = engine.legacy_mut();
    if h.phase != Phase::Init {
        set_last_error(3000, "Initial placement is only valid directly after Init");
        return -1;
    }
    if !h.initial_config.enabled {
        set_last_error(
            3000,
            "Initial placement disabled: set CSwarmConfig.initial_placement.enabled",
        );
        return -1;
    }

    let fallback = match h.solution.clone() {
        Some(solution)
            if crate::structure::pipeline::solution_validation_error_with_frozen_ids(
                &h.instance,
                &solution,
                &h.frozen_item_ids,
            )
            .is_none() =>
        {
            solution
        }
        _ => {
            set_last_error(
                3005,
                "Initial placement has no exact-certified Init fallback",
            );
            return -1;
        }
    };

    let latest_initial = Rc::new(RefCell::new(fallback.clone()));
    let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| -> Result<(), String> {
        let s = h.initial_config;

        // N02C 확정 구성 + FFI 재정의
        let mut acfg = crate::initial::AttachCfg::n02c();
        acfg.structure = crate::initial::Structure::Sorted;
        acfg.lbf_x_weight = s.lbf_x_weight;
        acfg.lbf_y_weight = s.lbf_y_weight;
        acfg.cand_eval_cap = s.cand_eval_cap;
        let esc = crate::initial::EscapeCfg {
            rescue_after: s.rescue_after,
            stall_cap: s.stall_cap,
            cd_min: s.cd_min,
            ..crate::initial::EscapeCfg::default()
        };
        let ccfg = crate::initial::SlideCompactCfg {
            passes_max: s.compact_passes,
            reseat_top: s.reseat_top,
            ..crate::initial::SlideCompactCfg::default()
        };
        let seed = h.rng.next_u64();
        let budget_ms = (f64::from(s.time_limit_secs.max(0.1)) * 1000.0) as u64;

        // 진행 보고: 최선 해 갱신 시 progress_callback_interval 스로틀로 중간 보고.
        // The seeded engine publishes only complete, original-certified gains.
        let cb = h.solution_callback;
        let user = h.callback_user_data as *mut std::ffi::c_void;
        let emit = h.emit_intermediate_json;
        let opts = h.callback_options;
        let frozen_ids = h.frozen_item_ids.clone();
        let min_interval = h.structure_config.progress_callback_interval;
        let instance = h.instance.clone();
        let mut last_emit: Option<Instant> = None;
        let frozen_mask = crate::util::frozen::mask(instance.items.len(), &frozen_ids);
        let mut on_improve = |sol: &SPSolution| {
            if !crate::util::assertions::frozen_poses_preserved(&fallback, sol, &frozen_mask) {
                return;
            }
            if !crate::initial::incumbent::candidate_is_improvement(
                &instance,
                sol,
                &latest_initial.borrow(),
                &frozen_ids,
                &crate::optimizer::place_groups::PlaceGroups::default(),
            ) {
                return;
            }
            *latest_initial.borrow_mut() = sol.clone();
            let Some(callback) = cb else { return };
            if last_emit.is_some_and(|t| t.elapsed() < min_interval) {
                return;
            }
            report_legacy_solution(
                callback,
                user,
                emit,
                2,
                sol,
                &instance,
                opts,
                true,
                &frozen_ids,
            );
            last_emit = Some(Instant::now());
        };

        let (best, _stats, _report, _compact) = crate::initial::shrink_compact_seeded_ctl(
            &instance,
            &acfg,
            &esc,
            &ccfg,
            s.grasp,
            s.kicks,
            s.step0_frac,
            s.min_frac,
            s.iter_cap,
            s.samples_base,
            s.growth,
            budget_ms,
            seed,
            Some(handle.lifecycle.stop_requested.as_ref()),
            Some(&mut on_improve),
            crate::initial::incumbent::InitialSeed {
                solution: &fallback, frozen_ids: &frozen_ids,
                groups: &crate::optimizer::place_groups::PlaceGroups::default(),
            },
        )?;

        let best = if crate::initial::incumbent::candidate_is_improvement(
            &instance,
            &best,
            &latest_initial.borrow(),
            &frozen_ids,
            &crate::optimizer::place_groups::PlaceGroups::default(),
        ) {
            best
        } else { latest_initial.borrow().clone() };

        let committed = certify_legacy_candidate_with_frozen(
            &h.instance,
            best,
            &fallback,
            "INIT-N06",
            &h.frozen_item_ids,
            h.fabric_edge_margin_mm,
        )?;
        crate::util::frozen::restore(&mut h.prob, &committed);
        if let Some(callback) = h.solution_callback {
            report_legacy_solution(
                callback,
                h.callback_user_data as *mut std::ffi::c_void,
                h.emit_intermediate_json,
                callback_report_type(&ReportType::Final),
                &committed,
                &h.instance,
                h.callback_options,
                true,
                &h.frozen_item_ids,
            );
        }
        h.solution = Some(committed);
        // Phase=Init 유지 — Explore(Init 전용)와 Compress(Init/Exploration 허용)를
        // 모두 이어 붙일 수 있다.
        Ok(())
    }));

    match run {
        Ok(Ok(())) => phase_success(),
        Ok(Err(error)) => {
            let latest = latest_initial.borrow().clone();
            crate::util::frozen::restore(&mut h.prob, &latest);
            h.solution = Some(latest);
            set_last_error(3005, error);
            -1
        }
        Err(_) => {
            let latest = latest_initial.borrow().clone();
            crate::util::frozen::restore(&mut h.prob, &latest);
            h.solution = Some(latest);
            set_last_error(9001, "panic while running Initial placement");
            1
        }
    }
}

// -- Swarm 탐색 단계

/// 탐색 단계 수행. 성공하면 0, 실패하면 0이 아닌 값을 반환.
/// 설정에 맞는 탐색 단계를 고른다.
///
/// step API(`swarm_init` -> `swarm_alg_phase` -> ...)도
/// [`optimize_configured`](swarm_core::optimizer::optimize_configured)와 **같은 선택 규칙**을
/// 따라야 한다. 예전에는 이 경로가 `exploration_phase` / `exploration_phase_adaptive` 만
/// 호출해서, WPF 에서 탈출 기법이나 NFP 옵션을 켜도 아무 일도 일어나지 않았다.
///
/// ```text
/// ordering_exploration  -> nfp_ordering_exploration_phase   (순수 NFP 순서 탐색)
/// escape_enabled        -> escape_driven_exploration_phase_with_nfp
/// feature "adaptive"    -> exploration_phase_adaptive
/// 그 외                 -> exploration_phase (upstream 원본)
/// ```
#[allow(clippy::too_many_arguments)]
pub(crate) fn select_exploration_phase(
    nfp_integration: swarm_core::config::NfpIntegrationConfig,
    escape_enabled: bool,
    escape_config: &swarm_core::escape::AdaptiveEscapeConfig,
    nfp_lbf: &swarm_core::config::NfpLbfSeedConfig,
    instance: &SPInstance,
    sep: &mut swarm_core::optimizer::separator::Separator,
    sol_listener: &mut Option<&mut dyn swarm_core::util::listener::SolutionListener>,
    term: &impl swarm_core::util::terminator::Terminator,
    expl_config: &ExplorationConfig,
) -> Vec<SPSolution> {
    sep.configure_reinsertion(&nfp_integration);
    if nfp_integration.ordering_exploration {
        let mut seed = *nfp_lbf;
        seed.enabled = true;
        return swarm_core::nfp_lbf::nfp_ordering_exploration_phase(
            instance,
            sep,
            sol_listener,
            term,
            expl_config,
            &seed,
        );
    }

    if escape_enabled {
        return swarm_core::escape::pipeline::escape_driven_exploration_phase_with_nfp(
            instance,
            sep,
            sol_listener,
            term,
            expl_config,
            escape_config,
            &nfp_integration,
        );
    }

    #[cfg(not(feature = "adaptive"))]
    {
        swarm_core::optimizer::explore::exploration_phase(
            instance,
            sep,
            sol_listener,
            term,
            expl_config,
        )
    }
    #[cfg(feature = "adaptive")]
    {
        swarm_core::optimizer::explore_mods::exploration_phase_adaptive(
            instance,
            sep,
            sol_listener,
            term,
            expl_config,
        )
    }
}

/// 탐색 단계 수행. 성공하면 0, 실패하면 0이 아닌 값을 반환.
///
/// 탐색·압축·후처리 세 단계는 **순서와 횟수가 자유**다 (예: Explore → Explore →
/// Compress → Polish → Explore → Compress). 어느 호출이든 직전 호출이 남긴 인증된
/// 현재 해에서 출발해 자기 `time_limit` 만큼 돈다. 호출 사이에 넘어가는 것은 그 해
/// 하나뿐이다 — 풀던 중이던 좁은 폭의 충돌 해, GLS 가중치, 교란 연산자 학습치는
/// 넘어가지 않는다. 결과가 직전보다 나빠지는 일은 없다. Structure 뒤에는 이을 수 없다.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_alg_phase(handle: *mut SwarmStepHandle) -> i32 {
    if handle.is_null() {
        return -1;
    }

    clear_last_error();
    let handle = unsafe { &*handle };
    let _run_scope = swarm_core::config::runtime::enter(handle.lifecycle.run_control.clone());
    let Some(_phase_call) = handle.lifecycle.begin_phase() else {
        set_last_error(
            3000,
            "Explore phase rejected: handle is busy, closing, or stopped",
        );
        return -1;
    };
    let mut engine = handle.lock_engine();
    if let HandleEngine::Grouped(h) = &mut *engine {
        let callback = h.solution_callback;
        let callback_user_data = h.callback_user_data;
        let emit_intermediate_json = h.emit_intermediate_json;
        let callback_options = h.callback_options;
        let progress_callback_interval = h.progress_callback_interval;
        let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let last_emit = Rc::new(Cell::new(Instant::now()));
            let round_clock = Rc::clone(&last_emit);
            let mut on_round = |solver: &GroupedStepState| {
                if callback.is_some() && round_clock.get().elapsed() >= progress_callback_interval {
                    if let Err(error) = report_grouped_solver_solution(
                        solver,
                        callback,
                        callback_user_data,
                        emit_intermediate_json,
                        2,
                        callback_options,
                        true,
                    ) {
                        log::warn!("[GROUPED:EXPL] skipped uncertified progress: {error}");
                    }
                    round_clock.set(Instant::now());
                }
            };
            let solution_clock = Rc::clone(&last_emit);
            let callback_frozen_ids = h.solver.frozen_ids().to_vec();
            let callback_groups = h.solver.place_groups().clone();
            let callback_expansion = h.solver.expansion().cloned();
            let latest_safe = Rc::new(RefCell::new(
                ExactSolutionCache::default()
                    .with_frozen_ids(&callback_frozen_ids)
                    .with_groups(&callback_groups)
                    .with_expansion(callback_expansion.clone()),
            ));
            let safe_solution = Rc::clone(&latest_safe);
            let first_preview = Cell::new(true);
            let mut on_all_items_solution =
                |report: ReportType, instance: &SPInstance, solution: &SPSolution| {
                    let interval_due = first_preview.get()
                        || solution_clock.get().elapsed() >= progress_callback_interval;
                    retain_exact_solution(
                        &safe_solution,
                        &report,
                        instance,
                        solution,
                        interval_due,
                    );
                    if callback.is_none() || !interval_due {
                        return;
                    }
                    if !report_cached_exact_solution(
                        callback.expect(swarm_core::dmsg!("checked grouped callback")),
                        callback_user_data as *mut std::ffi::c_void,
                        emit_intermediate_json,
                        2,
                        &safe_solution,
                        instance,
                        callback_options,
                    ) {
                        return;
                    }
                    first_preview.set(false);
                    solution_clock.set(Instant::now());
                };
            if callback.is_some() {
                h.solver.run_exploration_with_callbacks(
                    handle.lifecycle.stop_requested.as_ref(),
                    &mut on_round,
                    &mut on_all_items_solution,
                )
            } else {
                h.solver
                    .run_exploration(handle.lifecycle.stop_requested.as_ref())
            }
        }));
        return match run {
            Ok(Ok(())) => match report_grouped_solution(h, 4) {
                Ok(()) => phase_success(),
                Err(error) => {
                    set_last_error(3005, error);
                    -1
                }
            },
            Ok(Err(error)) => {
                set_last_error(3003, error);
                -1
            }
            Err(_) => {
                set_last_error(9001, "panic while running grouped Explore");
                1
            }
        };
    }
    let h = engine.legacy_mut();
    // 탐색·압축·후처리는 순서와 횟수가 자유다 (예: Explore → Explore → Compress →
    // Polish → Explore). 어느 호출이든 직전 호출이 확정한 인증 해(`h.solution`)가
    // 출발점이자 fallback 이다. Structure 는 별도 경로라 그 뒤에는 이을 수 없다.
    if h.phase == Phase::Structure {
        set_last_error(3000, "Legacy Explore is not valid after Structure");
        return -1;
    }

    let fallback = match h.solution.clone() {
        Some(solution)
            if crate::structure::pipeline::solution_validation_error_with_frozen_ids(
                &h.instance,
                &solution,
                &h.frozen_item_ids,
            )
            .is_none() =>
        {
            solution
        }
        _ => {
            set_last_error(
                3005,
                "Legacy Explore has no exact-certified current incumbent",
            );
            return -1;
        }
    };
    // 공개된 현재 해가 기준이다. 이전 단계가 남긴 작업 상태·후보 목록은 버린다.
    crate::util::frozen::restore(&mut h.prob, &fallback);
    h.solutions.clear();

    let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| -> Result<(), String> {
        let mut term = CancellableTerminator {
            inner: BasicTerminator::new(),
            stop: handle.lifecycle.stop_requested.clone(),
        };
        term.new_timeout(h.expl_config.time_limit);
        let mut sep = Separator::new(
            h.instance.clone(),
            h.prob.clone(),
            next_rng(&mut h.rng),
            h.expl_config.separator_config,
        );
        sep.freeze_items(&h.frozen_item_ids);

        // branch listener dispatch (generic fn, can't dyn)
        let latest_exact = Rc::new(RefCell::new(
            ExactSolutionCache::from_certified(h.solution_callback.map(|_| fallback.clone()))
                .with_frozen_ids(&h.frozen_item_ids),
        ));
        let solutions = if let Some(cb) = h.solution_callback {
            let mut cb_listener = CallbackListener {
                frozen_ids: h.frozen_item_ids.clone(),
                callback: cb,
                user_data: h.callback_user_data as *mut std::ffi::c_void,
                emit_json: h.emit_intermediate_json,
                last_emit: Rc::new(Cell::new(None)),
                min_interval: h.structure_config.progress_callback_interval,
                // 겹침 없는 해를 소비자에게 실제로 보낸다. 이것을 끄면 탐색 단계
                // 내내 JSON 이 하나도 안 나간다(ExplImproving 은 이제 부적격).
                emit_feasible_json: true,
                throttle_all_reports: false,
                callback_options: h.callback_options,
                latest_exact: Some(Rc::clone(&latest_exact)),
            };

            select_exploration_phase(
                h.nfp_integration,
                h.escape_enabled,
                &h.escape_config,
                &h.nfp_lbf,
                &h.instance,
                &mut sep,
                &mut Some(&mut cb_listener),
                &term,
                &h.expl_config,
            )
        } else {
            let solutions = select_exploration_phase(
                h.nfp_integration,
                h.escape_enabled,
                &h.escape_config,
                &h.nfp_lbf,
                &h.instance,
                &mut sep,
                &mut None,
                &term,
                &h.expl_config,
            );
            solutions
        };

        // Only the last exact solution is ever read back. It is also the candidate a following
        // Compress may start from, so it has to honour the same frozen-pose contract against the
        // public incumbent that certification below enforces; otherwise a candidate that moved a
        // frozen item would be rejected here yet still carried over as the Compress start.
        let frozen_mask = crate::util::frozen::mask(h.instance.items.len(), &h.frozen_item_ids);
        h.solutions = solutions
            .into_iter()
            .rev()
            .find(|solution| {
                crate::util::assertions::frozen_poses_preserved(&fallback, solution, &frozen_mask)
                    && crate::structure::pipeline::solution_validation_error_with_frozen_ids(
                        &h.instance,
                        solution,
                        &h.frozen_item_ids,
                    )
                    .is_none()
            })
            .into_iter()
            .collect();
        let candidate = h
            .solutions
            .last()
            .cloned()
            .unwrap_or_else(|| fallback.clone());
        let candidate = latest_exact
            .borrow()
            .certified
            .as_ref()
            .filter(|best| best.solution.strip_width() <= candidate.strip_width())
            .map(|best| best.solution.as_ref().clone())
            .unwrap_or(candidate);
        let committed = certify_legacy_candidate_with_frozen(
            &h.instance,
            candidate,
            &fallback,
            "EXPL",
            &h.frozen_item_ids,
            h.fabric_edge_margin_mm,
        )?;
        crate::util::frozen::restore(&mut sep.prob, &committed);
        if let Some(callback) = h.solution_callback {
            report_legacy_solution(
                callback,
                h.callback_user_data as *mut std::ffi::c_void,
                h.emit_intermediate_json,
                callback_report_type(&ReportType::Final),
                &committed,
                &h.instance,
                h.callback_options,
                true,
                &h.frozen_item_ids,
            );
        }
        h.solution = Some(committed);
        h.prob = sep.prob;
        h.phase = Phase::Exploration;
        Ok(())
    }));

    match run {
        Ok(Ok(())) => phase_success(),
        Ok(Err(error)) => {
            set_last_error(3005, error);
            -1
        }
        Err(_) => {
            set_last_error(9001, "panic while running Legacy Explore");
            1
        }
    }
}

/// 압축 단계 수행. 성공하면 0, 실패하면 0이 아닌 값을 반환.
///
/// 순서와 횟수는 자유다 — `swarm_init` 직후(탐색 생략, LBF + Compress), 탐색 뒤,
/// 또는 앞선 압축·후처리 뒤에 다시 호출할 수 있다. 공개된 현재 해에서 출발한다.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_cp_phase(handle: *mut SwarmStepHandle) -> i32 {
    if handle.is_null() {
        return -1;
    }

    clear_last_error();
    let handle = unsafe { &*handle };
    let _run_scope = swarm_core::config::runtime::enter(handle.lifecycle.run_control.clone());
    let Some(_phase_call) = handle.lifecycle.begin_phase() else {
        set_last_error(
            3000,
            "Compress phase rejected: handle is busy, closing, or stopped",
        );
        return -1;
    };
    let mut engine = handle.lock_engine();
    if let HandleEngine::Grouped(h) = &mut *engine {
        let callback = h.solution_callback;
        let callback_user_data = h.callback_user_data;
        let emit_intermediate_json = h.emit_intermediate_json;
        let callback_options = h.callback_options;
        let progress_callback_interval = h.progress_callback_interval;
        let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let last_emit = Rc::new(Cell::new(Instant::now()));
            let round_clock = Rc::clone(&last_emit);
            let mut on_round = |solver: &GroupedStepState| {
                if callback.is_some() && round_clock.get().elapsed() >= progress_callback_interval {
                    if let Err(error) = report_grouped_solver_solution(
                        solver,
                        callback,
                        callback_user_data,
                        emit_intermediate_json,
                        3,
                        callback_options,
                        true,
                    ) {
                        log::warn!("[GROUPED:CMPR] skipped uncertified progress: {error}");
                    }
                    round_clock.set(Instant::now());
                }
            };
            let solution_clock = Rc::clone(&last_emit);
            let callback_frozen_ids = h.solver.frozen_ids().to_vec();
            let callback_groups = h.solver.place_groups().clone();
            let callback_expansion = h.solver.expansion().cloned();
            let latest_safe = Rc::new(RefCell::new(
                ExactSolutionCache::default()
                    .with_frozen_ids(&callback_frozen_ids)
                    .with_groups(&callback_groups)
                    .with_expansion(callback_expansion.clone()),
            ));
            let safe_solution = Rc::clone(&latest_safe);
            let first_preview = Cell::new(true);
            let mut on_all_items_solution =
                |report: ReportType, instance: &SPInstance, solution: &SPSolution| {
                    let interval_due = first_preview.get()
                        || solution_clock.get().elapsed() >= progress_callback_interval;
                    retain_exact_solution(
                        &safe_solution,
                        &report,
                        instance,
                        solution,
                        interval_due,
                    );
                    if callback.is_none() || !interval_due {
                        return;
                    }
                    if !report_cached_exact_solution(
                        callback.expect(swarm_core::dmsg!("checked grouped callback")),
                        callback_user_data as *mut std::ffi::c_void,
                        emit_intermediate_json,
                        3,
                        &safe_solution,
                        instance,
                        callback_options,
                    ) {
                        return;
                    }
                    first_preview.set(false);
                    solution_clock.set(Instant::now());
                };
            if callback.is_some() {
                h.solver.run_compression_with_callbacks(
                    handle.lifecycle.stop_requested.as_ref(),
                    &mut on_round,
                    &mut on_all_items_solution,
                )
            } else {
                h.solver
                    .run_compression(handle.lifecycle.stop_requested.as_ref())
            }
        }));
        return match run {
            Ok(Ok(())) => {
                // Match the legacy phase contract: compression progress is 3,
                // while the mandatory phase boundary is Final (4).
                match report_grouped_solution(h, 4) {
                    Ok(()) => phase_success(),
                    Err(error) => {
                        set_last_error(3005, error);
                        -1
                    }
                }
            }
            Ok(Err(error)) => {
                set_last_error(3003, error);
                -1
            }
            Err(_) => {
                set_last_error(9001, "panic while running grouped Compress");
                1
            }
        };
    }
    let h = engine.legacy_mut();
    // 순서와 횟수는 자유다. `Init`(LBF 직후)에서 바로 호출하면 초기해에서 압축을 시작하고,
    // 압축·후처리 뒤에 다시 불러도 된다. Structure 뒤에만 이을 수 없다.
    if h.phase == Phase::Structure {
        set_last_error(3000, "Legacy Compress is not valid after Structure");
        return -1;
    }

    // 공개된 현재 해(`h.solution`)가 fallback 이자 기본 출발점이다. 다만 직전 탐색의
    // 마지막 후보가 그보다 좁고 고정 포즈 계약을 지키면 그쪽에서 출발한다 — 탐색
    // 경계의 인증이 fallback 으로 물러났어도 탐색 결과를 잃지 않게 하던 종전 동작을
    // 유지한다. 인증은 어느 출발점이든 공개 해(`fallback`)를 기준으로 한다. 후보 목록은
    // 여기서 비워, 뒤에 다시 부른 압축이 낡은(더 넓은) 탐색 해로 되돌아가지 않게 한다.
    let frozen_mask = crate::util::frozen::mask(h.instance.items.len(), &h.frozen_item_ids);
    let (start_sol, fallback) = match (h.solutions.last(), h.solution.as_ref()) {
        (Some(last), Some(current))
            if last.strip_width() < current.strip_width()
                && crate::util::assertions::frozen_poses_preserved(current, last, &frozen_mask) =>
        {
            (last.clone(), current.clone())
        }
        (_, Some(current)) => (current.clone(), current.clone()),
        (Some(last), None) => (last.clone(), last.clone()),
        (None, None) => return -1,
    };
    for (label, solution) in [("start", &start_sol), ("fallback", &fallback)] {
        if let Some(error) = crate::structure::pipeline::solution_validation_error_with_frozen_ids(
            &h.instance,
            solution,
            &h.frozen_item_ids,
        ) {
            set_last_error(
                3005,
                format!("Legacy Compress {label} is not exact-certified: {error:?}"),
            );
            return -1;
        }
    }
    crate::util::frozen::restore(&mut h.prob, &start_sol);
    h.solutions.clear();

    let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| -> Result<(), String> {
        let mut term = CancellableTerminator {
            inner: BasicTerminator::new(),
            stop: handle.lifecycle.stop_requested.clone(),
        };
        term.new_timeout(h.cmpr_config.time_limit);
        let mut sep = Separator::new(
            h.instance.clone(),
            h.prob.clone(),
            next_rng(&mut h.rng),
            h.cmpr_config.separator_config,
        );
        sep.freeze_items(&h.frozen_item_ids);

        let cmpr_sol = if let Some(cb) = h.solution_callback {
            let mut cb_listener = CallbackListener {
                frozen_ids: h.frozen_item_ids.clone(),
                callback: cb,
                user_data: h.callback_user_data as *mut std::ffi::c_void,
                emit_json: h.emit_intermediate_json,
                last_emit: Rc::new(Cell::new(None)),
                min_interval: h.structure_config.progress_callback_interval,
                // 겹침 없는 해를 소비자에게 실제로 보낸다. 이것을 끄면 탐색 단계
                // 내내 JSON 이 하나도 안 나간다(ExplImproving 은 이제 부적격).
                emit_feasible_json: true,
                throttle_all_reports: false,
                callback_options: h.callback_options,
                latest_exact: None,
            };
            let sol = compression_phase(
                &h.instance,
                &mut sep,
                &start_sol,
                &mut Some(&mut cb_listener),
                &term,
                &h.cmpr_config,
            );
            sol
        } else {
            compression_phase(
                &h.instance,
                &mut sep,
                &start_sol,
                &mut None,
                &term,
                &h.cmpr_config,
            )
        };
        let committed = certify_legacy_candidate_with_frozen(
            &h.instance,
            cmpr_sol,
            &fallback,
            "CMPR",
            &h.frozen_item_ids,
            h.fabric_edge_margin_mm,
        )?;
        sep.rollback(&committed, None);
        if let Some(callback) = h.solution_callback {
            report_legacy_solution(
                callback,
                h.callback_user_data as *mut std::ffi::c_void,
                h.emit_intermediate_json,
                callback_report_type(&ReportType::Final),
                &committed,
                &h.instance,
                h.callback_options,
                true,
                &h.frozen_item_ids,
            );
        }
        h.prob = sep.prob;
        h.solution = Some(committed);
        h.phase = Phase::Compression;
        Ok(())
    }));

    match run {
        Ok(Ok(())) => phase_success(),
        Ok(Err(error)) => {
            set_last_error(3005, error);
            -1
        }
        Err(_) => {
            set_last_error(9001, "panic while running Legacy Compress");
            1
        }
    }
}

/// 후처리(polish) 단계 수행. 성공하면 0, 실패하면 0이 아닌 값을 반환.
///
/// 압축 단계가 끝난 feasible 해에 대해 결정론적 정리 패스를 적용한다.
///   1) 좌측/하단 슬라이딩  2) 빈 포켓 타깃 채우기  3) LP compaction
///
/// 세 패스 모두 이동 전에 충돌 검증을 거치므로 결과가 이전보다 나빠지지 않는다.
/// 고정(freeze)된 아이템은 이동하지 않는다.
/// 순서와 횟수는 자유다 — 공개된 현재 해에서 출발하므로 압축을 건너뛰거나
/// 다른 단계 사이에 여러 번 넣어도 된다. Structure 뒤에는 이을 수 없다.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_refine_phase(handle: *mut SwarmStepHandle) -> i32 {
    if handle.is_null() {
        return -1;
    }

    clear_last_error();
    let handle = unsafe { &*handle };
    let _run_scope = swarm_core::config::runtime::enter(handle.lifecycle.run_control.clone());
    let Some(_phase_call) = handle.lifecycle.begin_phase() else {
        set_last_error(
            3000,
            "Polish phase rejected: handle is busy, closing, or stopped",
        );
        return -1;
    };
    let mut engine = handle.lock_engine();
    if let HandleEngine::Grouped(h) = &mut *engine {
        let callback = h.solution_callback;
        let callback_user_data = h.callback_user_data;
        let emit_intermediate_json = h.emit_intermediate_json;
        let callback_options = h.callback_options;
        let progress_callback_interval = h.progress_callback_interval;
        let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let last_emit = Rc::new(Cell::new(Instant::now()));
            let round_clock = Rc::clone(&last_emit);
            let mut on_round = |solver: &GroupedStepState| {
                if callback.is_some() && round_clock.get().elapsed() >= progress_callback_interval {
                    // Polish has no dedicated intermediate ABI report type;
                    // report its exact-feasible snapshots as compression-like
                    // progress and reserve Final for the mandatory boundary.
                    if let Err(error) = report_grouped_solver_solution(
                        solver,
                        callback,
                        callback_user_data,
                        emit_intermediate_json,
                        3,
                        callback_options,
                        true,
                    ) {
                        log::warn!("[GROUPED:PLSH] skipped uncertified progress: {error}");
                    }
                    round_clock.set(Instant::now());
                }
            };
            let solution_clock = Rc::clone(&last_emit);
            let callback_frozen_ids = h.solver.frozen_ids().to_vec();
            let callback_groups = h.solver.place_groups().clone();
            let callback_expansion = h.solver.expansion().cloned();
            let latest_safe = Rc::new(RefCell::new(
                ExactSolutionCache::default()
                    .with_frozen_ids(&callback_frozen_ids)
                    .with_groups(&callback_groups)
                    .with_expansion(callback_expansion.clone()),
            ));
            let safe_solution = Rc::clone(&latest_safe);
            let first_preview = Cell::new(true);
            let mut on_all_items_solution =
                |report: ReportType, instance: &SPInstance, solution: &SPSolution| {
                    let interval_due = first_preview.get()
                        || solution_clock.get().elapsed() >= progress_callback_interval;
                    retain_exact_solution(
                        &safe_solution,
                        &report,
                        instance,
                        solution,
                        interval_due,
                    );
                    if callback.is_none() || !interval_due {
                        return;
                    }
                    if !report_cached_exact_solution(
                        callback.expect(swarm_core::dmsg!("checked grouped callback")),
                        callback_user_data as *mut std::ffi::c_void,
                        emit_intermediate_json,
                        3,
                        &safe_solution,
                        instance,
                        callback_options,
                    ) {
                        return;
                    }
                    first_preview.set(false);
                    solution_clock.set(Instant::now());
                };
            if callback.is_some() {
                h.solver.run_polish_with_callbacks(
                    handle.lifecycle.stop_requested.as_ref(),
                    &mut on_round,
                    &mut on_all_items_solution,
                )
            } else {
                h.solver
                    .run_polish(handle.lifecycle.stop_requested.as_ref())
            }
        }));
        return match run {
            Ok(Ok(())) => match report_grouped_solution(h, 4) {
                Ok(()) => phase_success(),
                Err(error) => {
                    set_last_error(3005, error);
                    -1
                }
            },
            Ok(Err(error)) => {
                set_last_error(3003, error);
                -1
            }
            Err(_) => {
                set_last_error(9001, "panic while running grouped Polish");
                1
            }
        };
    }
    let h = engine.legacy_mut();
    // 순서와 횟수는 자유다 — 공개된 현재 해에서 출발한다. Structure 뒤에만 이을 수 없다.
    if h.phase == Phase::Structure {
        set_last_error(3000, "Legacy Polish is not valid after Structure");
        return -1;
    }

    let cmpr_sol = match h.solution.clone() {
        Some(s) => s,
        None => return -1,
    };
    if let Some(error) = crate::structure::pipeline::solution_validation_error_with_frozen_ids(
        &h.instance,
        &cmpr_sol,
        &h.frozen_item_ids,
    ) {
        set_last_error(
            3005,
            format!("Legacy Polish start is not exact-certified: {error:?}"),
        );
        return -1;
    }
    crate::util::frozen::restore(&mut h.prob, &cmpr_sol);
    h.solutions.clear();

    let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| -> Result<(), String> {
        let mut term = CancellableTerminator {
            inner: BasicTerminator::new(),
            stop: handle.lifecycle.stop_requested.clone(),
        };
        // The input was certified above. Keep Final/GetSolution/phase semantics
        // while avoiding a Separator whose workers would never run. Reclaim is
        // independent of the local Polish enable flag and must keep its budget.
        if !swarm_core::optimizer::polish::has_active_passes(&h.plsh_config, false)
            && !h.plsh_config.reclaim.enabled
        {
            let _ = next_rng(&mut h.rng); // preserve the phase RNG stream boundary
            crate::util::frozen::restore(&mut h.prob, &cmpr_sol);
            if let Some(callback) = h.solution_callback {
                report_legacy_solution(
                    callback,
                    h.callback_user_data as *mut std::ffi::c_void,
                    h.emit_intermediate_json,
                    callback_report_type(&ReportType::Final),
                    &cmpr_sol,
                    &h.instance,
                    h.callback_options,
                    true,
                    &h.frozen_item_ids,
                );
            }
            h.solution = Some(cmpr_sol);
            h.phase = Phase::Polish;
            return Ok(());
        }
        term.new_timeout(h.plsh_config.time_limit);
        // 폴리시 마감을 여기서 붙잡아 둔다. 폴리시가 일찍 수렴하면 이 시각까지가
        // 회수 루프의 예산이다.
        let polish_deadline = term.timeout_at();
        let mut sep = Separator::new(
            h.instance.clone(),
            h.prob.clone(),
            next_rng(&mut h.rng),
            h.cmpr_config.separator_config,
        );
        sep.freeze_items(&h.frozen_item_ids);

        let plsh_sol = if let Some(cb) = h.solution_callback {
            let mut cb_listener = CallbackListener {
                frozen_ids: h.frozen_item_ids.clone(),
                callback: cb,
                user_data: h.callback_user_data as *mut std::ffi::c_void,
                emit_json: h.emit_intermediate_json,
                last_emit: Rc::new(Cell::new(None)),
                min_interval: h.structure_config.progress_callback_interval,
                // 겹침 없는 해를 소비자에게 실제로 보낸다. 이것을 끄면 탐색 단계
                // 내내 JSON 이 하나도 안 나간다(ExplImproving 은 이제 부적격).
                emit_feasible_json: true,
                throttle_all_reports: false,
                callback_options: h.callback_options,
                latest_exact: None,
            };
            let sol = polish_phase_with_nfp(
                &h.instance,
                &mut sep,
                &cmpr_sol,
                &mut Some(&mut cb_listener),
                &term,
                &h.plsh_config,
                &h.nfp_integration,
                &h.lp_solver,
            );
            sol
        } else {
            polish_phase_with_nfp(
                &h.instance,
                &mut sep,
                &cmpr_sol,
                &mut None,
                &term,
                &h.plsh_config,
                &h.nfp_integration,
                &h.lp_solver,
            )
        };
        let committed = certify_legacy_candidate_with_frozen(
            &h.instance,
            plsh_sol,
            &cmpr_sol,
            "PLSH",
            &h.frozen_item_ids,
            h.fabric_edge_margin_mm,
        )?;

        // ── 잔여 예산 회수 ──
        //
        // 폴리시는 시계가 아니라 고정점에서 끝난다(실측: 60s 예산 중 2.4~7.4s).
        // 그 잔여를 압축 재진입에 쓴다. 마감은 **폴리시가 이미 받은 예산의 끝**이라
        // 이 단계의 총 소요 시간은 늘지 않는다.
        //
        // 인증 **뒤에** 두는 이유: 회수는 이미 인증된 해에서 출발해야 하고,
        // 그 자신도 폭이 줄었을 때만 교체하므로 결과가 나빠질 수 없다.
        let committed = match polish_deadline {
            Some(deadline) if h.plsh_config.reclaim.enabled => {
                // 회수가 실패했을 때 돌아갈 곳은 **방금 인증된 폴리시 해**다. 압축 해로
                // 돌아가면 회수의 늦은 실패 하나가 폴리시가 얻은 것까지 버린다.
                let polished = committed.clone();
                let (reclaimed, _) = swarm_core::optimizer::reclaim::reclaim_leftover(
                    &h.instance,
                    &mut sep,
                    committed,
                    deadline,
                    &h.plsh_config.reclaim,
                    &h.cmpr_config,
                    &h.expl_config,
                    &h.plsh_config,
                    &h.nfp_integration,
                    &h.escape_config,
                    &h.lp_solver,
                    &term,
                );
                // 회수 결과도 인증을 통과해야 한다. 통과 못 하면 인증된 직전 해로
                // 되돌아간다 — 회수가 배치를 깨뜨릴 수는 없다.
                let reclaimed = certify_legacy_candidate_with_frozen(
                    &h.instance,
                    reclaimed,
                    &polished,
                    "RECLAIM",
                    &h.frozen_item_ids,
                    h.fabric_edge_margin_mm,
                )?;
                // 회수는 «폭이 줄었을 때만 교체» 를 약속한다. 그 판정은 원단 경계 보정
                // **앞**에서 났고, 보정은 우측 끝 초과를 길이로 메우므로 보정 뒤에 다시 본다.
                // 회수가 줄인 폭보다 보정이 늘린 길이가 크면 폴리시 해가 더 짧다.
                if reclaimed.strip_width() < polished.strip_width() {
                    reclaimed
                } else {
                    polished
                }
            }
            _ => committed,
        };

        sep.rollback(&committed, None);
        if let Some(callback) = h.solution_callback {
            report_legacy_solution(
                callback,
                h.callback_user_data as *mut std::ffi::c_void,
                h.emit_intermediate_json,
                callback_report_type(&ReportType::Final),
                &committed,
                &h.instance,
                h.callback_options,
                true,
                &h.frozen_item_ids,
            );
        }
        h.prob = sep.prob;
        h.solution = Some(committed);
        h.phase = Phase::Polish;
        Ok(())
    }));

    match run {
        Ok(Ok(())) => phase_success(),
        Ok(Err(error)) => {
            set_last_error(3005, error);
            -1
        }
        Err(_) => {
            set_last_error(9001, "panic while running Legacy Polish");
            1
        }
    }
}

/// Run the complete experimental P0-P5 structure-first pipeline.
///
/// This is an alternative to the legacy Explore -> Compress -> Polish chain
/// and is therefore only valid directly after `swarm_init`.  A successful
/// call updates the handle's current problem/solution and leaves an audit JSON
/// retrievable through [`swarm_get_structure_report_json`].  Cancellation is
/// one-shot and follows the same lifecycle contract as the legacy phases.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_gp_phase(handle: *mut SwarmStepHandle) -> i32 {
    if handle.is_null() {
        return -1;
    }

    clear_last_error();
    let handle = unsafe { &*handle };
    let _run_scope = swarm_core::config::runtime::enter(handle.lifecycle.run_control.clone());
    let Some(_phase_call) = handle.lifecycle.begin_phase() else {
        set_last_error(
            3000,
            "structure phase rejected: handle is busy, closing, or stopped",
        );
        return -1;
    };
    let mut engine = handle.lock_engine();

    if let HandleEngine::Grouped(h) = &mut *engine {
        #[cfg(not(feature = "structure-first"))]
        {
            let _ = h;
            set_last_error(
                3004,
                "library was built without the structure-first feature",
            );
            return -1;
        }
        #[cfg(feature = "structure-first")]
        {
            let callback = h.solution_callback;
            let callback_user_data = h.callback_user_data;
            let emit_intermediate_json = h.emit_intermediate_json;
            let callback_options = h.callback_options;
            let progress_callback_interval = h.progress_callback_interval;
            let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                let last_intermediate_emit = Rc::new(Cell::<Option<Instant>>::new(None));
                let milestone_clock = Rc::clone(&last_intermediate_emit);
                let mut on_progress =
                    |stage, kind: StructureProgressKind, solver: &GroupedStepState| {
                        let stage_completed = kind == StructureProgressKind::StageCompleted;
                        let is_final = stage_completed
                            && matches!(
                                stage,
                                crate::structure::pipeline::StructureStage::P5Finish
                            );
                        let force_emit = matches!(
                            kind,
                            StructureProgressKind::InitialBaseline
                                | StructureProgressKind::LaneCompleted
                                | StructureProgressKind::StageCompleted
                        );
                        let should_emit = force_emit
                            || milestone_clock
                                .get()
                                .is_none_or(|last| last.elapsed() >= progress_callback_interval);
                        if !should_emit {
                            return;
                        }
                        let report_type = match (stage, is_final) {
                            (
                                crate::structure::pipeline::StructureStage::P1Construct
                                | crate::structure::pipeline::StructureStage::P3Explore,
                                _,
                            ) => 0,
                            (crate::structure::pipeline::StructureStage::P5Finish, true) => 4,
                            (crate::structure::pipeline::StructureStage::P2Compress, _)
                            | (crate::structure::pipeline::StructureStage::P5Finish, false) => 3,
                            (crate::structure::pipeline::StructureStage::P4Refine, _) => 2,
                            (crate::structure::pipeline::StructureStage::P0Preprocess, _) => return,
                        };
                        if let Err(error) = report_grouped_solver_solution(
                            solver,
                            callback,
                            callback_user_data,
                            emit_intermediate_json,
                            report_type,
                            callback_options,
                            true,
                        ) {
                            log::warn!("[GROUPED:STRUCT] skipped uncertified milestone: {error}");
                        }
                        milestone_clock.set(Some(Instant::now()));
                    };
                let solution_clock = Rc::clone(&last_intermediate_emit);
                let callback_frozen_ids = h.solver.frozen_ids().to_vec();
                let callback_groups = h.solver.place_groups().clone();
            let callback_expansion = h.solver.expansion().cloned();
                let latest_safe = Rc::new(RefCell::new(
                    ExactSolutionCache::default()
                        .with_frozen_ids(&callback_frozen_ids)
                        .with_groups(&callback_groups)
                    .with_expansion(callback_expansion.clone()),
                ));
                let safe_solution = Rc::clone(&latest_safe);
                let last_solution_stage = Cell::new(None);
                let mut on_all_items_solution =
                    |stage, report: ReportType, instance: &SPInstance, solution: &SPSolution| {
                        let new_stage = last_solution_stage.get() != Some(stage);
                        if new_stage {
                            last_solution_stage.set(Some(stage));
                            *safe_solution.borrow_mut() = ExactSolutionCache::default()
                                .with_frozen_ids(&callback_frozen_ids)
                                .with_groups(&callback_groups)
                    .with_expansion(callback_expansion.clone());
                        }
                        let interval_due = solution_clock
                            .get()
                            .is_none_or(|last| last.elapsed() >= progress_callback_interval);
                        retain_exact_solution(
                            &safe_solution,
                            &report,
                            instance,
                            solution,
                            interval_due || new_stage,
                        );
                        // A new stage must publish its certified baseline even if the
                        // previous stage emitted less than one throttle interval ago.
                        if callback.is_none() || !(interval_due || new_stage) {
                            return;
                        }
                        let report_type = match stage {
                            crate::structure::pipeline::StructureStage::P2Compress
                            | crate::structure::pipeline::StructureStage::P5Finish => 3,
                            crate::structure::pipeline::StructureStage::P3Explore => 0,
                            crate::structure::pipeline::StructureStage::P4Refine => 2,
                            crate::structure::pipeline::StructureStage::P0Preprocess
                            | crate::structure::pipeline::StructureStage::P1Construct => return,
                        };
                        if !report_cached_exact_solution(
                            callback.expect(swarm_core::dmsg!("checked grouped callback")),
                            callback_user_data as *mut std::ffi::c_void,
                            emit_intermediate_json,
                            report_type,
                            &safe_solution,
                            instance,
                            callback_options,
                        ) {
                            return;
                        }
                        solution_clock.set(Some(Instant::now()));
                    };
                if callback.is_some() {
                    h.solver.run_structure_with_callbacks(
                        handle.lifecycle.stop_requested.as_ref(),
                        &mut on_progress,
                        &mut on_all_items_solution,
                    )
                } else {
                    h.solver
                        .run_structure(handle.lifecycle.stop_requested.as_ref())
                }
            }));
            return match run {
                Ok(Ok(())) => phase_success(),
                Ok(Err(error)) => {
                    set_last_error(3003, error);
                    -1
                }
                Err(_) => {
                    set_last_error(9001, "panic while running grouped structure-first pipeline");
                    1
                }
            };
        }
    }

    let h = engine.legacy_mut();
    if h.phase != Phase::Init {
        set_last_error(3001, "structure-first is only valid directly after init");
        return -1;
    }
    if !h.structure_config.enabled {
        set_last_error(3002, "structure-first is disabled in CSwarmConfig");
        return -1;
    }

    #[cfg(not(feature = "structure-first"))]
    {
        set_last_error(
            3004,
            "library was built without the structure-first feature",
        );
        return -1;
    }

    #[cfg(feature = "structure-first")]
    {
        let instance = h.instance.clone();
        let initial_baseline = h.solution.clone();
        let initial_solution =
            if h.has_input_seed || h.has_column_draft || !h.frozen_item_ids.is_empty() {
                h.solution.clone()
            } else {
                None
            };
        let frozen_item_ids = h.frozen_item_ids.clone();
        let expl_config = h.expl_config;
        let cmpr_config = h.cmpr_config;
        let plsh_config = h.plsh_config;
        let nfp_integration = h.nfp_integration;
        let lp_solver = h.lp_solver;
        let structure_config = h.structure_config;
        let lbf_x_multiplier = h.lbf_x_multiplier;
        let lbf_y_multiplier = h.lbf_y_multiplier;
        let lbf_raster = h.lbf_raster;
        let lbf_order = h.lbf_order;
        let callback = h.solution_callback;
        let callback_user_data = h.callback_user_data;
        let emit_intermediate_json = h.emit_intermediate_json;
        let callback_options = h.callback_options;
        let run_rng = next_rng(&mut h.rng);

        // The checkpoint outlives both the pipeline and its cancellation/error
        // boundary. All callbacks here contain the same complete root instance.
        let latest_exact = Rc::new(RefCell::new(
            ExactSolutionCache::for_legacy_structure(&instance, initial_baseline.as_ref(),
                &frozen_item_ids, h.fabric_edge_margin_mm),
        ));

        let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let mut terminator = CancellableTerminator {
                inner: BasicTerminator::new(),
                stop: handle.lifecycle.stop_requested.clone(),
            };
            if let Some(baseline) = &initial_baseline {
                retain_exact_solution(&latest_exact, &ReportType::ExplFeas, &instance, baseline, true);
            }
            let last_emit = Rc::new(Cell::new(None));
            if let Some(callback) = callback {
                if report_cached_exact_solution(callback,
                    callback_user_data as *mut std::ffi::c_void, emit_intermediate_json,
                    0, &latest_exact, &instance, callback_options) {
                    last_emit.set(Some(Instant::now()));
                }
            } else if latest_exact.borrow().certified.is_some() {
                last_emit.set(Some(Instant::now()));
            }
            #[cfg(test)]
            if LEGACY_STRUCTURE_PANIC_AFTER_BASELINE.with(|flag| flag.replace(false)) {
                panic!("{}", swarm_core::dmsg!("injected Legacy Structure panic after canonical baseline"));
            }
            if terminator.kill() {
                return LegacyStructureOutcome::Cancelled;
            }
            let mut listener = LegacyStructureListener {
                cache: Rc::clone(&latest_exact), callback, user_data: callback_user_data,
                emit_json: emit_intermediate_json, callback_options,
                last_emit: Rc::clone(&last_emit), min_interval: structure_config.progress_callback_interval,
            };
            let milestone_exact = Rc::clone(&latest_exact);
            let mut milestone = |_stage, report: ReportType, solution: &SPSolution,
                milestone_instance: &SPInstance| {
                retain_exact_solution(&milestone_exact, &report, milestone_instance, solution, true);
                if matches!(report, ReportType::Final) { return; }
                if let Some(callback) = callback {
                    report_cached_exact_solution(callback,
                        callback_user_data as *mut std::ffi::c_void, emit_intermediate_json,
                        callback_report_type(&report), &milestone_exact, milestone_instance, callback_options);
                }
                last_emit.set(Some(Instant::now()));
            };
            let result = crate::structure::pipeline::try_optimize_structure_first_with_milestones(
                instance.clone(), run_rng, Some(&mut listener), &mut terminator,
                &expl_config, &cmpr_config, &plsh_config, &nfp_integration, &lp_solver,
                initial_solution.as_ref(), &frozen_item_ids, lbf_x_multiplier, lbf_y_multiplier,
                &lbf_raster, lbf_order, &structure_config, Some(&mut milestone),
            );
            match result {
                Ok((solution, report)) => LegacyStructureOutcome::Completed(solution, report),
                Err(failure) if failure.report.cancelled => LegacyStructureOutcome::Cancelled,
                Err(failure) => LegacyStructureOutcome::Failed(failure),
            }
        }));

        match run {
            Ok(LegacyStructureOutcome::Completed(solution, mut report)) => {
                if !report.cancelled {
                    retain_exact_solution(&latest_exact, &ReportType::Final, &h.instance, &solution, true);
                }
                let committed = latest_exact.borrow().certified.as_ref()
                    .map(|best| best.solution.as_ref().clone());
                let Some(solution) = committed else {
                    set_last_error(3005, "Structure has no certified public solution");
                    return -1;
                };
                report.final_width = solution.strip_width();
                report.final_density = solution.density();
                crate::util::frozen::restore(&mut h.prob, &solution);
                h.solutions.clear();
                h.solutions.push(solution.clone());
                h.solution = Some(solution);
                let completed = !report.cancelled;
                h.structure_report = Some(report);
                h.phase = Phase::Structure;
                if completed {
                    if let Some(callback) = callback {
                        report_cached_exact_solution(callback, callback_user_data as *mut std::ffi::c_void,
                            emit_intermediate_json, 4, &latest_exact, &h.instance, callback_options);
                    }
                }
                phase_success()
            }
            Ok(LegacyStructureOutcome::Failed(mut failure)) => {
                let message = failure.to_string();
                if let Some((width, density)) =
                    retain_legacy_structure_error_checkpoint(h, &latest_exact)
                {
                    failure.report.final_width = width;
                    failure.report.final_density = density;
                }
                h.structure_report = Some(failure.report);
                set_last_error(3005, message);
                -1
            }
            Ok(LegacyStructureOutcome::Cancelled) => {
                // The Init baseline was certified before it was exposed. A
                // cancellation therefore returns that last certified public
                // solution instead of whatever transient separator state was
                // active when stop arrived.
                let retained = latest_exact
                    .borrow()
                    .certified
                    .as_ref()
                    .map(|best| best.solution.as_ref().clone());
                if let Some(solution) = retained {
                    crate::util::frozen::restore(&mut h.prob, &solution);
                    h.structure_report = Some(
                        crate::structure::pipeline::StructureRunReport::cancelled_baseline(
                            &h.instance,
                            &solution,
                        ),
                    );
                    h.solutions.clear();
                    h.solutions.push(solution.clone());
                    h.solution = Some(solution);
                    h.phase = Phase::Structure;
                    phase_success()
                } else {
                    set_last_error(3005, "Structure cancellation has no exact baseline");
                    -1
                }
            }
            Err(_) => {
                retain_legacy_structure_error_checkpoint(h, &latest_exact);
                // A retry can retain an earlier failure report. This panic has
                // no fresh audit, so do not expose stale metrics or diagnostics.
                h.structure_report = None;
                set_last_error(9001, "panic while running legacy structure-first pipeline");
                1
            }
        }
    }
}

/// Return the most recent P0-P5 audit report as UTF-8 JSON.
///
/// The caller owns the returned allocation and must release it with
/// `swarm_free_string`. NULL means that structure-first has not completed or
/// that the handle/report is unavailable.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_get_structure_report_json(
    handle: *const SwarmStepHandle,
) -> *mut c_char {
    if handle.is_null() {
        return std::ptr::null_mut();
    }
    let handle = unsafe { &*handle };
    let Some(_call) = handle.lifecycle.begin_idle_call() else {
        return std::ptr::null_mut();
    };
    let engine = handle.lock_engine();
    let json = match &*engine {
        HandleEngine::Legacy(h) => h
            .structure_report
            .as_ref()
            .and_then(|report| serde_json::to_string(report).ok()),
        HandleEngine::Grouped(h) => h.solver.structure_report_json(),
    };
    json.and_then(|value| CString::new(value).ok())
        .map(CString::into_raw)
        .unwrap_or(std::ptr::null_mut())
}

// 최종 결과를 JSON 문자열로 반환.
// 호출자는 반환된 문자열을 swarm_free_string으로 해제해야 함.
// 결과가 없거나 오류가 발생한 경우 null 반환.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_get_solution_json(handle: *mut SwarmStepHandle) -> *mut c_char {
    if handle.is_null() {
        return std::ptr::null_mut();
    }
    clear_last_error();
    let handle = unsafe { &*handle };
    let Some(_call) = handle.lifecycle.begin_idle_call() else {
        set_last_error(3000, "solution export rejected: handle is busy or closing");
        return std::ptr::null_mut();
    };
    let engine = handle.lock_engine();
    let json_output = match &*engine {
        HandleEngine::Legacy(h) => {
            let sol = match &h.solution {
                Some(s) => s,
                None => return std::ptr::null_mut(),
            };
            if let Some(error) =
                crate::structure::pipeline::solution_validation_error_with_frozen_ids(
                    &h.instance,
                    sol,
                    &h.frozen_item_ids,
                )
            {
                set_last_error(
                    3005,
                    format!("stored Legacy solution is not exact-feasible: {error:?}"),
                );
                return std::ptr::null_mut();
            }
            jagua_rs::probs::spp::io::export(sol, *crate::EPOCH)
        }
        HandleEngine::Grouped(h) => match h.solver.export_certified_public_solution() {
            Ok(solution) => solution,
            Err(error) => {
                set_last_error(3005, error);
                return std::ptr::null_mut();
            }
        },
    };

    let json_str = match serde_json::to_string(&json_output) {
        Ok(s) => s,
        Err(_) => return std::ptr::null_mut(),
    };

    match CString::new(json_str) {
        Ok(cs) => cs.into_raw(),
        Err(_) => std::ptr::null_mut(),
    }
}

/// 현재 모든 Place의 충돌 상태를 JSON 형식으로 반환.
/// 반환된 문자열은 호출자가 해제해야 함.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_get_collision_status_json(
    handle: *const SwarmStepHandle,
) -> *mut c_char {
    if handle.is_null() {
        return std::ptr::null_mut();
    }
    let handle = unsafe { &*handle };
    let Some(_call) = handle.lifecycle.begin_idle_call() else {
        return std::ptr::null_mut();
    };
    let engine = handle.lock_engine();
    let prob = match &*engine {
        HandleEngine::Legacy(h) => {
            let solution = match &h.solution {
                Some(solution) => solution,
                None => return std::ptr::null_mut(),
            };
            let mut prob = h.prob.clone();
            crate::util::frozen::restore(&mut prob, solution);
            prob
        }
        HandleEngine::Grouped(h) => h.solver.restored_problem(),
    };
    let frozen_ids = match &*engine {
        HandleEngine::Legacy(h) => h.frozen_item_ids.clone(),
        HandleEngine::Grouped(h) => h.solver.frozen_ids().to_vec(),
    };
    let empty_groups = crate::optimizer::place_groups::PlaceGroups::default();
    let groups = match &*engine {
        HandleEngine::Legacy(_) => &empty_groups,
        HandleEngine::Grouped(h) => h.solver.place_groups(),
    };
    let status = compute_callback_diagnostics_with_groups(
        &prob.layout(),
        CallbackOptions {
            collect_loss: false,
            collect_collision_status: true,
        },
        &frozen_ids,
        groups,
    )
    .collision_status
    .unwrap_or_default();
    let status = match &*engine {
        HandleEngine::Grouped(h) => expand_statuses(h.solver.expansion(), status),
        HandleEngine::Legacy(_) => status,
    };
    let json = match serde_json::to_string(&status) {
        Ok(json) => json,
        Err(_) => return std::ptr::null_mut(),
    };

    CString::new(json)
        .map(CString::into_raw)
        .unwrap_or(std::ptr::null_mut())
}

/// 현재 모든 Place의 충돌 상태 배열을 반환한다.
/// 반환된 배열은 swarm_free_collision_status로 해제해야 한다.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_get_collision_status(
    handle: *const SwarmStepHandle,
    count: *mut usize,
) -> *mut CCollisionStatus {
    if count.is_null() {
        return std::ptr::null_mut();
    }
    unsafe {
        *count = 0;
    }

    if handle.is_null() {
        return std::ptr::null_mut();
    }
    let handle = unsafe { &*handle };
    let Some(_call) = handle.lifecycle.begin_idle_call() else {
        return std::ptr::null_mut();
    };
    let engine = handle.lock_engine();
    let prob = match &*engine {
        HandleEngine::Legacy(h) => {
            let solution = match &h.solution {
                Some(solution) => solution,
                None => return std::ptr::null_mut(),
            };
            let mut prob = h.prob.clone();
            crate::util::frozen::restore(&mut prob, solution);
            prob
        }
        HandleEngine::Grouped(h) => h.solver.restored_problem(),
    };
    let frozen_ids = match &*engine {
        HandleEngine::Legacy(h) => h.frozen_item_ids.clone(),
        HandleEngine::Grouped(h) => h.solver.frozen_ids().to_vec(),
    };
    let empty_groups = crate::optimizer::place_groups::PlaceGroups::default();
    let groups = match &*engine {
        HandleEngine::Legacy(_) => &empty_groups,
        HandleEngine::Grouped(h) => h.solver.place_groups(),
    };
    let statuses = compute_callback_diagnostics_with_groups(
        &prob.layout(),
        CallbackOptions {
            collect_loss: false,
            collect_collision_status: true,
        },
        &frozen_ids,
        groups,
    )
    .collision_status
    .unwrap_or_default();
    let statuses = match &*engine {
        HandleEngine::Grouped(h) => expand_statuses(h.solver.expansion(), statuses),
        HandleEngine::Legacy(_) => statuses,
    };
    let c_statuses: Vec<CCollisionStatus> = statuses
        .into_iter()
        .map(|status| CCollisionStatus {
            item_id: status.item_id,
            collision: status.collision as u8,
        })
        .collect();

    if c_statuses.is_empty() {
        return std::ptr::null_mut();
    }

    unsafe {
        *count = c_statuses.len();
    }
    Box::into_raw(c_statuses.into_boxed_slice()) as *mut CCollisionStatus
}

/// swarm_get_collision_status가 반환한 충돌 상태 배열을 해제한다.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_free_collision_status(statuses: *mut CCollisionStatus, count: usize) {
    if statuses.is_null() || count == 0 {
        return;
    }
    unsafe {
        drop(Box::from_raw(std::ptr::slice_from_raw_parts_mut(
            statuses, count,
        )));
    }
}

/// C#에서 계산한 바운딩 박스 정보를 받아 AABB 1차 필터 → CDE 정밀 검사로
/// 실제 충돌 중인 아이템 ID만 반환한다.
///
/// `handle`: SwarmStepHandle (swarm_init 직후 유효).
///           null이면 AABB-only 모드로 동작 (정밀 검사 없음).
/// `bounds`: CItemBounds 배열 (item_id, min/max 좌표)
/// `count`: bounds 배열 길이
///
/// 반환값: 실제 충돌 중인 item_id를 정렬된 JSON 배열로 반환 ([id1, id2, ...]).
///          충돌이 없으면 null. 호출 후 swarm_free_string으로 해제해야 함.
///
/// 동작 방식:
///   1. bounds로 AABB O(N²) 전수 비교 → 후보 쌍 추출
///   2. handle이 있으면: 후보 쌍에 대해 layout.cde() 정밀 폴리곤 충돌 검사
///   3. handle이 없으면: AABB 결과 그대로 반환
#[unsafe(no_mangle)]
pub extern "C" fn swarm_find_bound_collisions(
    handle: *const SwarmStepHandle,
    bounds: *const CItemBounds,
    count: i32,
) -> *mut c_char {
    if bounds.is_null() || count <= 1 {
        return std::ptr::null_mut();
    }
    let items = unsafe { std::slice::from_raw_parts(bounds, count as usize) };

    // - 1: AABB 전수 비교 → 후보 쌍 수집
    let mut aabb_colliding: std::collections::HashSet<i32> = std::collections::HashSet::new();
    for (i, a) in items.iter().enumerate() {
        for b in &items[i + 1..] {
            if a.max_x >= b.min_x && a.min_x <= b.max_x && a.max_y >= b.min_y && a.min_y <= b.max_y
            {
                aabb_colliding.insert(a.item_id);
                aabb_colliding.insert(b.item_id);
            }
        }
    }

    // - 2: CDE 정밀 검사 (handle이 있을 때만)
    let final_ids = if !handle.is_null() {
        let handle = unsafe { &*handle };
        let Some(_call) = handle.lifecycle.begin_idle_call() else {
            return std::ptr::null_mut();
        };
        let engine = handle.lock_engine();
        // `prob` may lag behind `solution` after compression/polish. Always
        // restore the currently exported incumbent before exact collision checks.
        let prob = match &*engine {
            HandleEngine::Legacy(h) => {
                let Some(solution) = &h.solution else {
                    return std::ptr::null_mut();
                };
                let mut prob = h.prob.clone();
                crate::util::frozen::restore(&mut prob, solution);
                prob
            }
            HandleEngine::Grouped(h) => h.solver.restored_problem(),
        };
        let frozen_ids = match &*engine {
            HandleEngine::Legacy(h) => h.frozen_item_ids.as_slice(),
            HandleEngine::Grouped(h) => h.solver.frozen_ids(),
        };
        let empty_groups = crate::optimizer::place_groups::PlaceGroups::default();
        let groups = match &*engine {
            HandleEngine::Legacy(_) => &empty_groups,
            HandleEngine::Grouped(h) => h.solver.place_groups(),
        };
        let layout = &prob.layout();

        // item_id → 그 종류의 모든 PlacedItemKey. 수량이 2 이상인 종류는 복사본이 여럿이므로
        // 키 하나만 남기면 나머지 복사본의 충돌을 놓친다.
        let mut id_to_pks: std::collections::HashMap<usize, Vec<_>> =
            std::collections::HashMap::new();
        for (pk, pi) in layout.placed_items().iter() {
            id_to_pks.entry(pi.item().idx()).or_default().push(pk);
        }

        // AABB로 걸러진 ID들만 정밀 CDE 검사
        let mut ids: Vec<i32> = aabb_colliding.into_iter().collect();
        ids.sort();

        use jagua_rs::collision_detection::hazards::BasicHazardEntity;
        use jagua_rs::collision_detection::hazards::collector::BasicHazardCollector;
        let mut precise_colliding: std::collections::HashSet<i32> =
            std::collections::HashSet::new();
        let mut collector = BasicHazardCollector::new();

        for &id in &ids {
            let Some(pks) = id_to_pks.get(&(id as usize)) else {
                continue;
            };
            for &pk_a in pks {
                let pi_a = &layout.placed_items()[pk_a];

                collector.clear();
                layout
                    .cde()
                    .collect_poly_collisions(&pi_a.shape(), &NoFilter, &mut collector);

                for (_, he) in collector.iter() {
                    if let BasicHazardEntity::PlacedItem { pk, .. } = he
                        && *pk > pk_a
                    {
                        let coll_id = layout.placed_items()[*pk].item().idx() as i32;
                        if groups.same_group(pi_a.item().idx(), coll_id as usize) {
                            continue;
                        }
                        if frozen_ids.contains(&pi_a.item().idx())
                            && frozen_ids.contains(&(coll_id as usize))
                        {
                            continue;
                        }
                        // AABB 후보에 있는 쌍만 인정 (false positive 제거)
                        if ids.binary_search(&coll_id).is_ok() {
                            precise_colliding.insert(id);
                            precise_colliding.insert(coll_id);
                        }
                    }
                }
            }
        }

        let mut v: Vec<i32> = precise_colliding.into_iter().collect();
        v.sort();
        v
    } else {
        // handle 없음 → AABB 결과 그대로
        let mut v: Vec<i32> = aabb_colliding.into_iter().collect();
        v.sort();
        v
    };

    if final_ids.is_empty() {
        return std::ptr::null_mut();
    }
    let json = serde_json::to_string(&final_ids).unwrap_or_default();
    CString::new(json)
        .ok()
        .map(|cs| cs.into_raw())
        .unwrap_or(std::ptr::null_mut())
}

// -- Terminator

struct CancellableTerminator {
    inner: BasicTerminator,
    stop: Arc<AtomicBool>,
}

impl Terminator for CancellableTerminator {
    fn kill(&self) -> bool {
        self.stop.load(Ordering::Acquire) || self.inner.kill()
    }
    fn stop_requested(&self) -> bool {
        self.stop.load(Ordering::Acquire)
    }
    fn new_timeout(&mut self, timeout: Duration) {
        self.inner.new_timeout(timeout)
    }
    fn timeout_at(&self) -> Option<Instant> {
        self.inner.timeout_at()
    }
}

/// 고정할 item_id 목록을 설정한다. 실행할 첫 단계(탐색 또는 압축) 전에 호출해야 한다.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_freeze_items(
    handle: *mut SwarmStepHandle,
    ids: *const i32,
    count: i32,
) {
    if handle.is_null() || ids.is_null() || count <= 0 {
        return;
    }
    let handle = unsafe { &*handle };
    let Some(_call) = handle.lifecycle.begin_idle_call() else {
        return;
    };
    let mut engine = handle.lock_engine();
    let slice = unsafe { std::slice::from_raw_parts(ids, count as usize) };
    if let HandleEngine::Legacy(h) = &mut *engine {
        h.frozen_item_ids = slice.iter().map(|&id| id as usize).collect();
    }
}

/// Cancel an in-flight `swarm_init_from_json`. Safe to call from any thread.
///
/// Init has no handle yet, so `swarm_request_stop` cannot reach it. The flag is
/// process-wide: it is cleared when the next `swarm_init_from_json` begins and is
/// checked at every constructor item boundary and between Init stages. A cancelled
/// Init returns NULL with error code 2002.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_cancel_init() {
    swarm_core::config::runtime::request_init_cancel();
}

/// Signal the running phase to stop as soon as possible.
/// Safe to call from any thread while a phase is running.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_request_stop(handle: *mut SwarmStepHandle) {
    if !handle.is_null() {
        let handle = unsafe { &*handle };
        let Some(_call) = handle.lifecycle.begin_call() else {
            return;
        };
        handle.lifecycle.request_stop();
    }
}

/// Legacy process default for subsequently created handles. Does not change
/// active runs. Use swarm_set_handle_shrink_step for live control.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_set_shrink_step(step: f32) -> i32 {
    i32::from(swarm_core::config::runtime::set_shrink_step(step))
}
#[unsafe(no_mangle)]
pub extern "C" fn swarm_clear_shrink_step() {
    swarm_core::config::runtime::clear_shrink_step();
}
#[unsafe(no_mangle)]
pub extern "C" fn swarm_get_shrink_step() -> f32 {
    swarm_core::config::runtime::shrink_step_override().unwrap_or(-1.0)
}

/// Updates only this handle; callable while its phase owns the engine lock.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_set_handle_shrink_step(
    handle: *const SwarmStepHandle,
    step: f32,
) -> i32 {
    let Some(handle) = (unsafe { handle.as_ref() }) else {
        return 0;
    };
    let Some(_call) = handle.lifecycle.begin_call() else {
        return 0;
    };
    i32::from(handle.lifecycle.run_control.set_shrink_step(step))
}
#[unsafe(no_mangle)]
pub extern "C" fn swarm_clear_handle_shrink_step(handle: *const SwarmStepHandle) {
    let Some(handle) = (unsafe { handle.as_ref() }) else {
        return;
    };
    let Some(_call) = handle.lifecycle.begin_call() else {
        return;
    };
    handle.lifecycle.run_control.clear_shrink_step();
}
#[unsafe(no_mangle)]
pub extern "C" fn swarm_get_handle_shrink_step(handle: *const SwarmStepHandle) -> f32 {
    let Some(handle) = (unsafe { handle.as_ref() }) else {
        return -1.0;
    };
    let Some(_call) = handle.lifecycle.begin_call() else {
        return -1.0;
    };
    handle.lifecycle.run_control.shrink_step().unwrap_or(-1.0)
}

/// Read the current stop flag state (non-zero = stop requested).
#[unsafe(no_mangle)]
pub extern "C" fn swarm_is_stop_requested(handle: *const SwarmStepHandle) -> i32 {
    if handle.is_null() {
        return -1;
    }
    let handle = unsafe { &*handle };
    let Some(_call) = handle.lifecycle.begin_call() else {
        return -1;
    };
    handle.lifecycle.stop_requested.load(Ordering::Acquire) as i32
}

/// Signal the running phase to stop and block until it has actually stopped.
///
/// If no phase is running (including immediately after init), this returns immediately. The stop
/// request is permanent for this handle, so subsequent phase calls are rejected. When called from
/// a phase callback, it records cancellation and returns so the callback thread cannot wait on
/// itself; the phase exits after the callback returns.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_request_stop_and_wait(handle: *mut SwarmStepHandle) {
    if handle.is_null() {
        return;
    }
    let handle = unsafe { &*handle };
    let Some(_call) = handle.lifecycle.begin_call() else {
        return;
    };
    handle.lifecycle.request_stop_and_wait();
}

/// Free the handle and all associated resources.
///
/// If a phase or another API call is already active, cancellation is requested and this function
/// waits until those calls leave the handle before dropping it. As with ordinary C handles, callers
/// must not start a new API call after `swarm_free` has begun and must call free exactly once.
///
/// Calling `swarm_free` from inside a solution callback (i.e. on the thread that is running the
/// phase) cannot wait for that phase to finish; the handle is then closed but intentionally leaked
/// rather than freed underneath the running phase.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_free(handle: *mut SwarmStepHandle) {
    if !handle.is_null() {
        let handle_ref = unsafe { &*handle };
        if handle_ref.lifecycle.close_and_wait() {
            let _ = unsafe { Box::from_raw(handle) };
        }
    }
}

#[cfg(test)]
mod lifecycle_tests {
    use super::*;
    use std::sync::mpsc;

    #[test]
    fn public_handle_is_send_and_sync() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<SwarmStepHandle>();
    }

    #[test]
    fn stop_and_wait_returns_immediately_when_idle() {
        let lifecycle = HandleLifecycle::new();

        lifecycle.request_stop_and_wait();

        assert!(lifecycle.stop_requested.load(Ordering::Acquire));
        assert!(lifecycle.begin_phase().is_none());
    }

    #[test]
    fn stop_and_wait_from_phase_thread_does_not_self_deadlock() {
        let lifecycle = HandleLifecycle::new();
        let phase = lifecycle.begin_phase().expect(swarm_core::dmsg!("phase should start"));

        assert!(lifecycle.begin_idle_call().is_none());
        lifecycle.request_stop_and_wait();

        assert!(lifecycle.stop_requested.load(Ordering::Acquire));
        drop(phase);
    }

    #[test]
    fn stop_and_wait_blocks_until_running_phase_exits() {
        let lifecycle = Arc::new(HandleLifecycle::new());
        let (started_tx, started_rx) = mpsc::channel();
        let (release_tx, release_rx) = mpsc::channel();

        let worker_lifecycle = lifecycle.clone();
        let worker = std::thread::spawn(move || {
            let _phase = worker_lifecycle.begin_phase().expect(swarm_core::dmsg!("phase should start"));
            started_tx.send(()).unwrap();
            release_rx.recv().unwrap();
        });
        started_rx.recv().unwrap();

        let (done_tx, done_rx) = mpsc::channel();
        let waiter_lifecycle = lifecycle.clone();
        let waiter = std::thread::spawn(move || {
            waiter_lifecycle.request_stop_and_wait();
            done_tx.send(()).unwrap();
        });

        let deadline = std::time::Instant::now() + Duration::from_secs(2);
        while !lifecycle.stop_requested.load(Ordering::Acquire)
            && std::time::Instant::now() < deadline
        {
            std::thread::yield_now();
        }
        assert!(lifecycle.stop_requested.load(Ordering::Acquire));
        assert!(done_rx.recv_timeout(Duration::from_millis(50)).is_err());
        release_tx.send(()).unwrap();
        done_rx.recv_timeout(Duration::from_secs(2)).unwrap();

        worker.join().unwrap();
        waiter.join().unwrap();
        assert!(lifecycle.stop_requested.load(Ordering::Acquire));
    }

    #[test]
    fn close_waits_for_active_phase_and_rejects_new_calls() {
        let lifecycle = Arc::new(HandleLifecycle::new());
        let (started_tx, started_rx) = mpsc::channel();
        let (release_tx, release_rx) = mpsc::channel();

        let worker_lifecycle = lifecycle.clone();
        let worker = std::thread::spawn(move || {
            let _phase = worker_lifecycle.begin_phase().expect(swarm_core::dmsg!("phase should start"));
            started_tx.send(()).unwrap();
            release_rx.recv().unwrap();
        });
        started_rx.recv().unwrap();

        let (closed_tx, closed_rx) = mpsc::channel();
        let closer_lifecycle = lifecycle.clone();
        let closer = std::thread::spawn(move || {
            assert!(closer_lifecycle.close_and_wait());
            closed_tx.send(()).unwrap();
        });

        let deadline = std::time::Instant::now() + Duration::from_secs(2);
        while !lifecycle.stop_requested.load(Ordering::Acquire)
            && std::time::Instant::now() < deadline
        {
            std::thread::yield_now();
        }
        assert!(lifecycle.stop_requested.load(Ordering::Acquire));
        assert!(closed_rx.recv_timeout(Duration::from_millis(50)).is_err());
        release_tx.send(()).unwrap();
        closed_rx.recv_timeout(Duration::from_secs(2)).unwrap();

        worker.join().unwrap();
        closer.join().unwrap();
        assert!(lifecycle.begin_call().is_none());
        assert!(lifecycle.begin_phase().is_none());
    }

    #[test]
    fn close_from_phase_thread_does_not_self_deadlock() {
        let lifecycle = HandleLifecycle::new();
        let phase = lifecycle.begin_phase().expect(swarm_core::dmsg!("phase should start"));

        // A callback running on the phase thread frees the handle: the close must give up
        // instead of waiting for its own thread, and the handle stays closed afterwards.
        assert!(!lifecycle.close_and_wait());
        assert!(lifecycle.stop_requested.load(Ordering::Acquire));
        assert!(lifecycle.begin_call().is_none());
        assert!(lifecycle.begin_phase().is_none());

        drop(phase);
        assert!(lifecycle.close_and_wait());
    }
}

#[cfg(test)]
mod callback_diagnostics_tests {
    use super::*;
    use jagua_rs::geometry::DTransformation;
    use jagua_rs::probs::spp::entities::SPPlacement;

    fn exact_callback_problem() -> (SPInstance, SPProblem) {
        let (instance, _) = problem_with_second_x(13.0);
        let mut problem = SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
        problem.change_strip_width(30.0).expect(swarm_core::dmsg!("strip width must be positive"));
        for (item_id, x) in [(0, 7.0), (1, 18.0)] {
            problem.place_item(SPPlacement {
                item_idx: item_id,
                d_transf: DTransformation::new(0.0, (x, 7.0)),
            });
        }
        assert!(
            crate::structure::pipeline::solution_validation_error(&instance, &problem.save())
                .is_none()
        );
        (instance, problem)
    }

    #[test]
    fn issue3_exact_cache_keeps_compact_ties_and_rejects_wider_candidates() {
        let (instance, problem) = exact_callback_problem();
        let cache = Rc::new(RefCell::new(ExactSolutionCache::from_certified(Some(
            problem.save(),
        ))));
        let mut compact = SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
        compact.change_strip_width(30.0).expect(swarm_core::dmsg!("strip width must be positive"));
        for (item_id, x) in [(0, 6.0), (1, 18.0)] {
            compact.place_item(SPPlacement {
                item_idx: item_id,
                d_transf: DTransformation::new(0.0, (x, 7.0)),
            });
        }
        assert!(
            crate::structure::pipeline::solution_validation_error(&instance, &compact.save()).is_none()
        );
        retain_exact_solution(&cache, &ReportType::Final, &instance, &compact.save(), true);
        let retained_x = || {
            cache
                .borrow()
                .certified
                .as_ref()
                .unwrap()
                .solution
                .layout_snapshot()
                .placed_items()
                .values()
                .find(|p| p.item().idx() == 0)
                .unwrap()
                .d_transf()
                .translation()
                .0
        };
        assert_eq!(
            retained_x(),
            6.0,
            "equal-width compact final must replace the callback baseline"
        );
        let stale = problem.save();
        retain_exact_solution(&cache, &ReportType::Final, &instance, &stale, true);
        assert_eq!(
            retained_x(),
            6.0,
            "later stale tie must not undo compactness"
        );
        compact.change_strip_width(30.000002).expect(swarm_core::dmsg!("strip width must be positive"));
        let key = compact
            .layout()
            .placed_items()
            .iter()
            .find_map(|(key, p)| (p.item().idx() == 0).then_some(key))
            .unwrap();
        compact.remove_item(key);
        compact.place_item(SPPlacement {
            item_idx: 0,
            d_transf: DTransformation::new(0.0, (5.5, 7.0)),
        });
        assert!(
            crate::structure::pipeline::solution_validation_error(&instance, &compact.save()).is_none()
        );
        retain_exact_solution(&cache, &ReportType::Final, &instance, &compact.save(), true);
        assert_eq!(
            retained_x(),
            6.0,
            "even one ULP wider must be rejected despite better compactness"
        );
        assert_eq!(
            cache
                .borrow()
                .certified
                .as_ref()
                .unwrap()
                .solution
                .strip_width(),
            30.0
        );
    }

    #[test]
    fn certified_cache_keeps_shortest_when_late_final_is_wider() {
        let (instance, problem) = exact_callback_problem();
        let best = problem.save();
        let cache = Rc::new(RefCell::new(ExactSolutionCache::from_certified(Some(best))));
        let mut later = problem;
        later.change_strip_width(35.0).expect(swarm_core::dmsg!("strip width must be positive"));
        retain_exact_solution(&cache, &ReportType::Final, &instance, &later.save(), true);
        assert_eq!(
            cache
                .borrow()
                .certified
                .as_ref()
                .unwrap()
                .solution
                .strip_width(),
            30.0
        );
    }

    #[test]
    fn early_certified_callback_bypasses_interval_and_carries_json() {
        extern "C" fn capture(
            _: i32,
            width: f32,
            _: f32,
            json: *const std::os::raw::c_char,
            state: *mut std::ffi::c_void,
        ) {
            unsafe { &mut *(state as *mut Vec<(f32, bool)>) }.push((width, !json.is_null()));
        }
        let (instance, mut problem) = exact_callback_problem();
        let mut events = Vec::<(f32, bool)>::new();
        let mut listener = CallbackListener {
            frozen_ids: vec![],
            callback: capture,
            user_data: &mut events as *mut _ as *mut std::ffi::c_void,
            emit_json: true,
            last_emit: Rc::new(Cell::new(Some(Instant::now()))),
            min_interval: std::time::Duration::from_secs(60),
            emit_feasible_json: false,
            throttle_all_reports: false,
            callback_options: DEFAULT_CALLBACK_OPTIONS,
            latest_exact: Some(Rc::new(RefCell::new(ExactSolutionCache::from_certified(
                Some(problem.save()),
            )))),
        };
        problem.change_strip_width(29.0).expect(swarm_core::dmsg!("strip width must be positive"));
        listener.report_certified_early(&problem.save(), &instance);
        assert_eq!(events, vec![(29.0, true)]);
        listener.report_certified_early(&problem.save(), &instance);
        assert_eq!(
            events.len(),
            1,
            "equal-width peers must not duplicate public updates"
        );
        problem.change_strip_width(32.0).expect(swarm_core::dmsg!("strip width must be positive"));
        listener.report(ReportType::Final, &problem.save(), &instance);
        assert_eq!(
            events.last(),
            Some(&(29.0, true)),
            "late final cannot replace early public best"
        );
    }

    #[cfg(feature = "structure-first")]
    extern "C" fn no_op_callback(
        _report_type: i32,
        _strip_width: f32,
        _density: f32,
        _json: *const std::os::raw::c_char,
        _user_data: *mut std::ffi::c_void,
    ) {
    }

    fn problem_with_second_x(second_x: f32) -> (SPInstance, SPProblem) {
        let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
            "name":"callback-diagnostics",
            "strip_height":20.0,
            "items":[
                {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                    "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":10.0,"height":8.0}
                }},
                {"id":1,"demand":1,"allowed_orientations":[0.0],"shape":{
                    "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":10.0,"height":8.0}
                }}
            ]
        }))
        .unwrap();
        let config = DEFAULT_SWARM_CONFIG;
        let importer = swarm_core::util::io::new_importer(config.cde_config, config.poly_simpl_tolerance, config.min_item_separation, config.narrow_concavity_cutoff_ratio);
        let instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext).unwrap();
        let mut problem = SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
        problem.change_strip_width(30.0).expect(swarm_core::dmsg!("strip width must be positive"));
        for (item_id, x) in [(0, 2.0), (1, second_x)] {
            problem.place_item(SPPlacement {
                item_idx: item_id,
                d_transf: DTransformation::new(0.0, (x, 2.0)),
            });
        }
        (instance, problem)
    }

    #[test]
    fn combined_diagnostics_preserve_independent_options_and_collision_loss() {
        let (_, problem) = problem_with_second_x(2.0);
        let both = compute_callback_diagnostics(
            &problem.layout(),
            CallbackOptions {
                collect_loss: true,
                collect_collision_status: true,
            },
        );
        assert!(both.loss.is_some_and(|loss| loss > 0.0));
        let statuses = both.collision_status.unwrap();
        assert_eq!(statuses.len(), 2);
        assert!(statuses.iter().all(|status| status.collision));

        let loss_only = compute_callback_diagnostics(
            &problem.layout(),
            CallbackOptions {
                collect_loss: true,
                collect_collision_status: false,
            },
        );
        assert!(loss_only.loss.is_some_and(|loss| loss > 0.0));
        assert!(loss_only.collision_status.is_none());

        let status_only = compute_callback_diagnostics(
            &problem.layout(),
            CallbackOptions {
                collect_loss: false,
                collect_collision_status: true,
            },
        );
        assert!(status_only.loss.is_none());
        assert!(
            status_only
                .collision_status
                .is_some_and(|statuses| statuses.iter().all(|status| status.collision))
        );
    }

    #[test]
    #[cfg(feature = "structure-first")]
    fn exact_cache_validates_only_dirty_pending_versions() {
        let (instance, problem) = problem_with_second_x(13.0);
        let solution = problem.save();
        let cache = Rc::new(RefCell::new(ExactSolutionCache::from_certified(Some(
            solution.clone(),
        ))));

        retain_exact_solution(
            &cache,
            &ReportType::ExplImproving,
            &instance,
            &solution,
            true,
        );
        assert_eq!(cache.borrow().pending_version, 1);
        assert_eq!(cache.borrow().validated_version, 1);

        retain_exact_solution(&cache, &ReportType::ExplFeas, &instance, &solution, false);
        assert_eq!(cache.borrow().pending_version, 2);
        assert_eq!(cache.borrow().validated_version, 1);

        retain_exact_solution(
            &cache,
            &ReportType::ExplImproving,
            &instance,
            &solution,
            true,
        );
        assert_eq!(cache.borrow().pending_version, 2);
        assert_eq!(cache.borrow().validated_version, 2);
        assert!(cache.borrow().certified.is_some());

        assert!(report_cached_exact_solution(
            no_op_callback,
            std::ptr::null_mut(),
            true,
            0,
            &cache,
            &instance,
            DEFAULT_CALLBACK_OPTIONS,
        ));
        let first_payload = cache
            .borrow()
            .certified
            .as_ref()
            .unwrap()
            .payload
            .as_ref()
            .unwrap()
            .as_ptr();
        assert!(report_cached_exact_solution(
            no_op_callback,
            std::ptr::null_mut(),
            true,
            3,
            &cache,
            &instance,
            DEFAULT_CALLBACK_OPTIONS,
        ));
        let second_payload = cache
            .borrow()
            .certified
            .as_ref()
            .unwrap()
            .payload
            .as_ref()
            .unwrap()
            .as_ptr();
        assert_eq!(first_payload, second_payload);
    }
}

#[cfg(test)]
mod legacy_exact_boundary_tests {

    /// `ffi_step.rs` 의 **프로덕션 부분만** 돌려준다 (첫 `#[cfg(test)]` 앞까지).
    ///
    /// # 왜 잘라내는가
    ///
    /// 아래 배선 회귀 테스트들은 원래 `include_str!("ffi_step.rs")` **전체**에
    /// `contains` 를 걸었다. 그런데 찾는 문자열 리터럴이 바로 그 단정문 안에 들어 있으므로
    /// `contains` 는 **언제나 참**이었다 — 프로덕션 호출을 한 줄도 남김없이 지워도
    /// 테스트가 통과한다. 배선 누락을 막으라고 만든 방어가 자기 자신 때문에 무력했다.
    ///
    /// 실측: `grep -c "build_best_initial_nest" src/ffi_step.rs` = 2
    /// (프로덕션 1598 + 테스트 리터럴 1). 프로덕션을 지우면 1로 줄 뿐 `contains` 는 참이다.
    ///
    /// 테스트 모듈을 잘라내면 그 자기참조가 끊어진다.
    /// [`the_wiring_probe_must_not_read_its_own_assertions`] 가 그 성질을 지킨다.
    fn production_source() -> &'static str {
        let src = include_str!("ffi_step.rs");
        let cut = src
            .find("mod legacy_exact_boundary_tests {")
            .expect(swarm_core::dmsg!("테스트 모듈 경계(`#[cfg(test)]`)를 찾지 못했다"));
        &src[..cut]
    }

    /// 배선 탐침이 **자기 단정문을 읽지 않는지** 확인한다.
    ///
    /// 이 테스트가 깨지면 [`production_source`] 의 절단이 무너진 것이고,
    /// 그 순간 아래 배선 테스트 전부가 조용히 무의미해진다.
    #[test]
    fn the_wiring_probe_must_not_read_its_own_assertions() {
        let prod = production_source();
        assert!(
            !prod.contains("the_wiring_probe_must_not_read_its_own_assertions"),
            "프로덕션 절단이 테스트 모듈까지 포함하고 있다 — 배선 단정이 자기참조로 항상 참이 된다"
        );
        // 절단 후에도 프로덕션 본문은 남아 있어야 한다(경계를 너무 앞에서 자르지 않았는지).
        assert!(
            prod.contains("pub extern \"C\" fn swarm_init"),
            "절단이 너무 앞에서 일어나 프로덕션 본문이 사라졌다"
        );
    }

    /// 전략 ⑧ 누적 배치 / ⑨ 꼬리 회수가 **C ABI 를 건너 Rust 설정까지 도달하는지**.
    ///
    /// 두 전략은 코어에 구현돼 있었지만 `NfpStrategyConfig` 에 스위치가 없고
    /// `build_best_initial_nest` 도 부르지 않아 **벤치에서만 돌았다** — 함정 1 그 자체다.
    /// 값을 직접 확인하므로 전달이 끊기면 즉시 깨진다.
    #[test]
    fn accretion_and_tail_reflow_cross_the_abi() {
        let mut c_cfg = DEFAULT_C_SWARM_CONFIG_FOR_TEST.with(|c| *c);
        c_cfg.nfp_strategy2.accretion = 1;
        c_cfg.nfp_strategy2.tail_reflow = 1;
        c_cfg.nfp_strategy2.reflow_rounds = 5;
        let cfg = crate::ffi::build_config(&c_cfg);
        assert!(cfg.nfp_strategy.accretion, "⑧ 누적 배치가 전달되지 않았다");
        assert!(
            cfg.nfp_strategy.tail_reflow,
            "⑨ 꼬리 회수가 전달되지 않았다"
        );
        assert_eq!(cfg.nfp_strategy.reflow_rounds, 5);
        assert!(
            cfg.nfp_strategy.any_enabled(),
            "⑧⑨만 켜도 전략 경로로 들어가야 한다 — any_enabled 가 세지 않으면 배치가 기존 경로로 샌다"
        );
        assert!(
            cfg.nfp_strategy.ordering_is_only_a_multiset(),
            "⑧이 켜지면 호출부가 ordering 을 1개만 평가해야 한다"
        );

        // 0 이하 라운드는 코어 기본값으로 올라간다.
        c_cfg.nfp_strategy2.reflow_rounds = 0;
        let cfg = crate::ffi::build_config(&c_cfg);
        assert_eq!(
            cfg.nfp_strategy.reflow_rounds,
            swarm_core::nfp_strategy::DEFAULT_REFLOW_ROUNDS
        );

        // 기본값은 둘 다 꺼짐이어야 한다. 켜지면 기존 실측치가 전부 무효가 된다.
        let default_cfg = crate::ffi::build_config(&DEFAULT_C_SWARM_CONFIG_FOR_TEST.with(|c| *c));
        assert!(!default_cfg.nfp_strategy.accretion);
        assert!(!default_cfg.nfp_strategy.tail_reflow);
        assert!(!default_cfg.nfp_strategy.any_enabled());
    }

    /// 실험 엔진을 켜고 **WPF 와 같은 경로**(step API init)로 돌렸을 때
    /// 조각이 실제로 해에 들어가는지.
    ///
    /// 스위치가 도달하는 것과 배치가 나오는 것은 다른 문제다 — 사용자가 켰는데
    /// 화면에 아무것도 안 보이면 그건 여기서 잡혀야 한다.
    #[test]
    fn the_experimental_engine_actually_produces_a_visible_layout() {
        let mut config = DEFAULT_SWARM_CONFIG;
        config.refplace.enabled = true;
        let handle = build_handle(
            rectangle_instance(),
            None,
            config,
            CallbackOptions {
                collect_loss: false,
                collect_collision_status: false,
            },
            crate::ffi::InitialPlacementSettings::default(),
            Vec::new(),
        )
        .expect(swarm_core::dmsg!("핸들이 만들어져야 한다"));

        let (instance, solution) = unsafe { legacy_solution(handle) };
        let expected: usize = instance.items.iter().map(|(_, q)| *q).sum();
        let placed = solution.layout_snapshot().placed_items().len();
        assert_eq!(
            placed, expected,
            "실험 엔진을 켰더니 {placed}/{expected} 만 해에 들어갔다 — 화면이 빈다"
        );
        assert!(
            solution.strip_width().is_finite() && solution.strip_width() > 0.0,
            "폭이 유효해야 한다: {}",
            solution.strip_width()
        );
        assert!(
            crate::structure::pipeline::solution_validation_error(&instance, &solution).is_none(),
            "실험 엔진이 만든 해가 검증을 통과해야 한다"
        );
        swarm_free(handle);
    }

    #[test]
    fn column_draft_is_the_selected_init_and_survives_as_an_exploration_seed() {
        let mut config = DEFAULT_SWARM_CONFIG;
        config.refplace.enabled = true;
        config.refplace.column_draft = true;
        // The optional NFP-LBF seed must not overwrite an explicit Draft choice.
        config.nfp_lbf.enabled = true;
        let handle = build_handle(
            rectangle_instance(),
            None,
            config,
            CallbackOptions {
                collect_loss: false,
                collect_collision_status: false,
            },
            crate::ffi::InitialPlacementSettings::default(),
            Vec::new(),
        )
        .unwrap();
        {
            let state = unsafe { &*handle }.lock_engine();
            let HandleEngine::Legacy(h) = &*state else {
                panic!("{}", swarm_core::dmsg!("legacy handle"))
            };
            assert!(
                h.has_column_draft,
                "Draft failed or silently fell back to LBF"
            );
            assert!(
                !h.has_input_seed,
                "generated Draft must not be labelled a user hint"
            );
            let (expected, _) = swarm_core::refplace::columns::build_column_draft(
                &h.instance,
                Default::default(),
            )
            .unwrap();
            assert_eq!(
                h.solution.as_ref().unwrap().strip_width(),
                expected.strip_width()
            );
        }
        swarm_free(handle);
    }

    /// 실험 엔진 스위치가 C ABI 를 건너 Rust 설정까지 도달하는지.
    ///
    /// WPF 체크박스가 실제로 무언가 하는지를 값으로 확인한다 — 소스 문자열이 아니라.
    #[test]
    fn the_experimental_engine_switch_crosses_the_abi() {
        let mut c_cfg = DEFAULT_C_SWARM_CONFIG_FOR_TEST.with(|c| *c);
        c_cfg.refplace.enabled = 1;
        c_cfg.refplace.group_fallback = 0;
        let cfg = crate::ffi::build_config(&c_cfg);
        assert!(cfg.refplace.enabled, "실험 엔진 스위치가 전달되지 않았다");
        assert!(!cfg.refplace.column_draft);
        c_cfg.refplace._reserved0 = 1;
        assert!(crate::ffi::build_config(&c_cfg).refplace.column_draft);
        assert_eq!(
            cfg.refplace.fallback(),
            swarm_core::refplace::GroupFallback::EachPieceAlone
        );

        // 범위 밖 값은 조용히 잘린다 — 경계에서 프로세스를 죽이지 않는다.
        c_cfg.refplace.group_fallback = 99;
        let cfg = crate::ffi::build_config(&c_cfg);
        assert_eq!(
            cfg.refplace.fallback(),
            swarm_core::refplace::GroupFallback::ByItemId
        );

        // **기본은 꺼짐이어야 한다.** 켜지면 기존 초기 배치가 통째로 갈리고
        // 지금까지의 모든 실측치가 무효가 된다.
        let default_cfg = crate::ffi::build_config(&DEFAULT_C_SWARM_CONFIG_FOR_TEST.with(|c| *c));
        assert!(
            !default_cfg.refplace.enabled,
            "실험 엔진이 기본으로 켜지면 안 된다"
        );
    }

    /// ① NFP 샘플러 플래그가 **separator 설정까지 도달하는지** 확인한다.
    ///
    /// 이 플래그는 오래도록 WPF 체크박스 -> C# -> C ABI -> `SwarmConfig::nfp_integration`
    /// 까지 도달한 뒤 **읽는 코드가 0건**이라 아무 일도 하지 않았다. 후보를 만드는 주체는
    /// separator 이므로 그쪽 설정까지 전파되어야 실제로 동작한다.
    ///
    /// 소스 문자열이 아니라 **값**을 본다 — 전파 줄을 지우면 즉시 깨진다.
    #[test]
    fn nfp_sampler_flag_reaches_the_separator_config() {
        let mut c_cfg = DEFAULT_C_SWARM_CONFIG_FOR_TEST.with(|c| *c);
        c_cfg.nfp_integration.nfp_sampler = 1;
        let cfg = crate::ffi::build_config(&c_cfg);
        assert!(cfg.nfp_integration.nfp_sampler);
        assert!(
            cfg.expl_cfg.separator_config.nfp_sampler,
            "탐색 separator 가 NFP 샘플러 설정을 못 받았다 — 체크박스가 아무 일도 하지 않는다"
        );
        assert!(
            cfg.cmpr_cfg.separator_config.nfp_sampler,
            "압축 separator 가 NFP 샘플러 설정을 못 받았다"
        );

        // 기본값은 꺼짐이어야 한다. 켜지면 기존 실측치가 전부 무효가 된다.
        let default_cfg = crate::ffi::build_config(&DEFAULT_C_SWARM_CONFIG_FOR_TEST.with(|c| *c));
        assert!(!default_cfg.nfp_integration.nfp_sampler);
        assert!(!default_cfg.expl_cfg.separator_config.nfp_sampler);
        assert!(!default_cfg.cmpr_cfg.separator_config.nfp_sampler);
    }

    /// 기본값을 채워 주는 `polish_phase` 래퍼가 **되살아나지 않았는지** 확인한다.
    ///
    /// 그 래퍼가 있던 동안 step API·structure-first·grouped 엔진이 전부 그리로 흘러
    /// `lp_solver` 와 `nfp_integration` 이 조용히 버려졌다. 지금은 래퍼가 없어서
    /// 후처리를 부르려면 두 설정을 반드시 손에 쥐어야 하고, 빠뜨리면 컴파일이 안 된다.
    /// 편의를 이유로 래퍼를 다시 만들면 같은 결함이 즉시 되돌아온다.
    #[test]
    fn no_defaulting_polish_wrapper_may_come_back() {
        let polish = include_str!("../swarm-core/src/optimizer/polish/mod.rs");
        assert!(
            !polish.contains("pub fn polish_phase("),
            "기본값을 하드코딩하는 polish 래퍼가 되살아났다 — 배선 누락이 다시 컴파일러에 보이지 않게 된다"
        );
    }
    use super::*;
    use jagua_rs::geometry::DTransformation;
    use jagua_rs::geometry::geo_traits::Transformable;

    #[derive(Default)]
    struct CallbackCapture {
        final_count: usize,
        final_had_json: bool,
        final_json: Vec<String>,
    }

    struct ReentrantGroupedCapture {
        handle: *mut SwarmStepHandle,
        final_count: usize,
        nested_result: i32,
        nested_error: i32,
        attempted: bool,
    }

    extern "C" fn capture_callback(
        report_type: i32,
        _strip_width: f32,
        _density: f32,
        json: *const c_char,
        user_data: *mut std::ffi::c_void,
    ) {
        if report_type != callback_report_type(&ReportType::Final) || user_data.is_null() {
            return;
        }
        let capture = unsafe { &mut *(user_data as *mut CallbackCapture) };
        capture.final_count += 1;
        capture.final_had_json = !json.is_null();
        if !json.is_null() {
            capture.final_json.push(
                unsafe { CStr::from_ptr(json) }
                    .to_str()
                    .expect(swarm_core::dmsg!("callback JSON must be UTF-8"))
                    .to_owned(),
            );
        }
    }

    extern "C" fn reentrant_grouped_callback(
        report_type: i32,
        _strip_width: f32,
        _density: f32,
        _json: *const c_char,
        user_data: *mut std::ffi::c_void,
    ) {
        if report_type != callback_report_type(&ReportType::Final) || user_data.is_null() {
            return;
        }
        let capture = unsafe { &mut *(user_data as *mut ReentrantGroupedCapture) };
        capture.final_count += 1;
        if capture.attempted {
            return;
        }
        capture.attempted = true;
        capture.nested_result = swarm_alg_phase(capture.handle);
        capture.nested_error = swarm_get_last_error_code();
    }

    struct Issue9TempFile(std::path::PathBuf);

    impl Issue9TempFile {
        fn new(bytes: &[u8]) -> Self {
            static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
            let path = std::env::temp_dir().join(format!("swarm-issue9-{}-{}.json",
                std::process::id(), NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)));
            std::fs::write(&path, bytes).unwrap();
            Self(path)
        }
    }

    impl Drop for Issue9TempFile {
        fn drop(&mut self) { let _ = std::fs::remove_file(&self.0); }
    }

    fn issue9_init(json: &str, file: bool) -> (*mut SwarmStepHandle, (usize, usize)) {
        struct ProbeScope;
        impl Drop for ProbeScope {
            fn drop(&mut self) { ISSUE9_INPUT_COUNTS.with(|slot| slot.set(None)); }
        }
        let mut config = DEFAULT_C_SWARM_CONFIG_FOR_TEST.with(|c| *c);
        config.rng_seed = 41;
        config.nfp_lbf.enabled = 0;
        config.refplace.enabled = 0;
        let temp = file.then(|| Issue9TempFile::new(json.as_bytes()));
        let input = std::ffi::CString::new(temp.as_ref()
            .map_or_else(|| json.to_owned(), |f| f.0.to_string_lossy().into_owned())).unwrap();
        let _scope = ProbeScope;
        ISSUE9_INPUT_COUNTS.with(|slot| slot.set(Some((0, 0))));
        let handle = if file {
            swarm_init(input.as_ptr(), &config, std::ptr::null(), 0)
        } else {
            swarm_init_from_json(input.as_ptr(), &config, std::ptr::null(), 0)
        };
        let counts = ISSUE9_INPUT_COUNTS.with(|slot| slot.get().unwrap());
        (handle, counts)
    }

    fn issue9_v2_json() -> String {
        serde_json::json!({"schema_version":2,
            "nesting":{"mode":"all_items","group_workers":1,"quantum_ms":1000},
            "instance":rectangle_instance(),"regions":[],"place_groups":[]}).to_string()
    }

    fn issue9_certified_export(handle: *mut SwarmStepHandle) -> ExtSPSolution {
        assert!(!handle.is_null(), "Init failed: {}", swarm_get_last_error_code());
        let engine = unsafe { &*handle }.lock_engine();
        match &*engine {
            HandleEngine::Legacy(state) => {
                assert!(crate::structure::pipeline::solution_validation_error(
                    &state.instance, state.solution.as_ref().unwrap()).is_none());
            }
            HandleEngine::Grouped(state) => { state.solver.export_certified_solution().unwrap(); }
        }
        drop(engine);
        let ptr = swarm_get_solution_json(handle);
        assert!(!ptr.is_null());
        let output = serde_json::from_str(unsafe { CStr::from_ptr(ptr) }.to_str().unwrap()).unwrap();
        crate::ffi::swarm_free_string(ptr);
        output
    }

    #[test]
    fn issue9_legacy_file_reads_input_once() {
        let (handle, counts) = issue9_init(&serde_json::to_string(&rectangle_instance()).unwrap(), true);
        issue9_certified_export(handle);
        swarm_free(handle);
        assert_eq!(counts, (1, 1), "actual Legacy file Init read/Value attempts");
    }

    #[test]
    fn multipolygon_place_groups_export_members_with_exact_relative_poses() {
        use crate::ffi::{PLACE_GROUP_MULTIPOLYGON, PLACE_GROUP_RIGID};
        let mut v2: serde_json::Value = serde_json::from_str(&issue9_v2_json()).unwrap();
        v2["place_groups"] = serde_json::json!([{"id": 7, "member_item_ids": [0, 1]}]);
        // Members 4 wide at x = 0 and x = 12 leave an 8 wide gap; a third 4x4 item
        // is hinted into that gap. Its hull would overlap the group's envelope.
        v2["instance"]["items"].as_array_mut().unwrap().push(serde_json::json!({
            "id": 2, "demand": 1, "allowed_orientations": [0.0],
            "shape": {"type": "rectangle", "data": {"x_min": 0.0, "y_min": 0.0, "width": 4.0, "height": 4.0}},
            "placed_hint": {"item_id": 2, "is_freeze": 0, "tr_x": 6.0, "tr_y": 2.0, "rot_angle_in_rad": 0.0}
        }));
        for (id, x) in [(0usize, 0.0f32), (1, 12.0)] {
            v2["instance"]["items"][id]["placed_hint"] = serde_json::json!({
                "item_id": id, "is_freeze": 0, "tr_x": x, "tr_y": 2.0, "rot_angle_in_rad": 0.0});
        }
        for mode in [PLACE_GROUP_MULTIPOLYGON, PLACE_GROUP_RIGID] {
            let mut config = DEFAULT_C_SWARM_CONFIG_FOR_TEST.with(|c| *c);
            config.rng_seed = 41;
            config.nfp_lbf.enabled = 0;
            config.refplace.enabled = 0;
            config.place_group.mode = mode;
            let json = std::ffi::CString::new(v2.to_string()).unwrap();
            let handle = swarm_init_from_json(json.as_ptr(), &config, std::ptr::null(), 0);
            assert!(!handle.is_null(), "mode {mode}: init failed: {}", swarm_get_last_error_code());
            let output = issue9_certified_export(handle);
            let mut placed = output.layout.placed_items.clone();
            placed.sort_by_key(|p| p.item_id);
            assert_eq!(placed.iter().map(|p| p.item_id).collect::<Vec<_>>(), vec![0, 1, 2], "mode {mode}");
            // The members keep their hint-relative pose: item 1 is 12 to the right of item 0.
            let (a, b) = (&placed[0].transformation, &placed[1].transformation);
            assert!((a.rotation - b.rotation).abs() < 1e-3, "mode {mode}");
            let (sin, cos) = a.rotation.to_radians().sin_cos();
            let expected = (a.translation.0 + cos * 12.0, a.translation.1 + sin * 12.0);
            assert!(
                (b.translation.0 - expected.0).abs() < 1e-3 && (b.translation.1 - expected.1).abs() < 1e-3,
                "mode {mode}: {:?} vs {:?}", b.translation, expected
            );
            // Public collision status uses public ids.
            let status_ptr = swarm_get_collision_status_json(handle);
            assert!(!status_ptr.is_null());
            let status: serde_json::Value =
                serde_json::from_str(unsafe { CStr::from_ptr(status_ptr) }.to_str().unwrap()).unwrap();
            crate::ffi::swarm_free_string(status_ptr);
            let ids: Vec<u64> = status.as_array().unwrap().iter().map(|s| s["item_id"].as_u64().unwrap()).collect();
            assert_eq!(ids, vec![0, 1, 2], "mode {mode}");
            assert!(status.as_array().unwrap().iter().all(|s| s["collision"] == false), "mode {mode}: {status}");
            swarm_free(handle);
        }
    }

    #[test]
    fn issue9_v2_file_parses_value_once() {
        let (handle, counts) = issue9_init(&issue9_v2_json(), true);
        issue9_certified_export(handle);
        swarm_free(handle);
        assert_eq!(counts, (1, 1), "actual V2 file Init read/Value attempts");
    }

    #[test]
    fn issue9_v2_json_parses_value_once() {
        let (handle, counts) = issue9_init(&issue9_v2_json(), false);
        issue9_certified_export(handle);
        swarm_free(handle);
        assert_eq!(counts, (0, 1), "actual V2 JSON Init read/Value attempts");
    }

    #[test]
    fn issue9_file_json_success_error_and_group_policy_parity() {
        let base = serde_json::to_value(rectangle_instance()).unwrap();
        let (seed, _) = issue9_init(&base.to_string(), false);
        let solution = issue9_certified_export(seed); swarm_free(seed);
        let mut warm = base.clone(); warm["solution"] = serde_json::to_value(solution).unwrap();
        let mut cases = vec![("legacy",base.to_string(),0,false), ("flattened warm",warm.to_string(),0,true)];
        for (label, value) in [("null solution",serde_json::Value::Null), ("malformed solution",serde_json::json!({"strip_width":"bad"}))] {
            let mut v=base.clone(); v["solution"]=value; cases.push((label,v.to_string(),0,false));
        }
        let mut metadata=base.clone(); metadata["unknown"]=serde_json::json!(true);
        metadata["items"][0]["OrgID"]=serde_json::json!(9_007_199_254_740_993_i64);
        metadata["items"][1]["OrgID"]=serde_json::json!(i64::MAX);
        cases.push(("unknown metadata",metadata.to_string(),0,false));
        cases.push(("duplicate typed field",base.to_string().replacen("\"strip_height\":",
            "\"strip_height\":20.0,\"strip_height\":",1),2000,false));
        cases.push(("truncated","{\"items\":".into(),2000,false));
        cases.push(("empty","".into(),2000,false));
        let v2:serde_json::Value=serde_json::from_str(&issue9_v2_json()).unwrap();
        cases.push(("v2 all items",v2.to_string(),0,false));
        for version in [0_u64,1,3,u64::MAX] {
            let mut v=v2.clone(); v["schema_version"]=serde_json::json!(version);
            cases.push(("unsupported unsigned",v.to_string(),1001,false));
        }
        for version in [serde_json::Value::Null,serde_json::json!(2.0),serde_json::json!(2.5),serde_json::json!(-1),serde_json::json!("2")] {
            let mut legacy=base.clone(); legacy["schema_version"]=version.clone();
            cases.push(("legacy discriminator fallback",legacy.to_string(),0,false));
            let mut grouped=v2.clone(); grouped["schema_version"]=version;
            cases.push(("pure v2 falls back and fails",grouped.to_string(),2000,false));
        }
        let mut unknown=v2.clone(); unknown["unknown"]=serde_json::json!(true);
        cases.push(("v2 unknown field",unknown.to_string(),1002,false));
        let mut invalid=v2.clone(); invalid["nesting"]["group_workers"]=serde_json::json!(0);
        cases.push(("v2 invalid config",invalid.to_string(),1003,false));
        for size_regions in [false,true] {
            let mut grouped=v2.clone();
            grouped["place_groups"]=serde_json::json!([{"id":7,"member_item_ids":[0,1]}]);
            for id in 0..2 { grouped["instance"]["items"][id]["placed_hint"]=serde_json::json!({
                "item_id":id,"is_freeze":if id==0 {1} else {0},"tr_x":5.0,"tr_y":5.0,"rot_angle_in_rad":0.0}); }
            if size_regions {
                grouped["nesting"]["mode"]=serde_json::json!("size_regions");
                grouped["regions"]=serde_json::json!([{"id":"locked","kind":"size_group","item_ids":[0,1]}]);
            }
            cases.push(("overlapping partial frozen rigid group",grouped.to_string(),0,false));
        }
        for (label,text,expected,warm) in cases {
            let mut results=Vec::new();
            for file in [true,false] {
                let (handle,counts)=issue9_init(&text,file);
                let code=swarm_get_last_error_code();
                let message=LAST_ERROR.with(|slot| slot.borrow().1.clone());
                assert_eq!(code,expected,"{label} file={file}: {message}");
                assert_eq!(counts.0,usize::from(file),"{label}");
                assert_eq!(counts.1,usize::from(file || !text.is_empty()),"{label}");
                if expected!=0 {
                    assert!(handle.is_null(),"{label}"); results.push(serde_json::json!({"code":code,"message":message}));
                } else {
                    let output=issue9_certified_export(handle);
                    if serde_json::from_str::<serde_json::Value>(&text).unwrap().get("schema_version")
                        .and_then(serde_json::Value::as_u64)!=Some(2) {
                        assert_eq!(unsafe{legacy_has_input_seed(handle)},warm,"{label}");
                    }
                    let mut placed=serde_json::to_value(output.layout.placed_items).unwrap().as_array().unwrap().clone();
                    placed.sort_by_key(|p|p["item_id"].as_u64().unwrap());
                    assert_eq!(placed.len(),2,"{label}");
                    assert_eq!(placed[0]["item_id"],0); assert_eq!(placed[1]["item_id"],1);
                    if label=="overlapping partial frozen rigid group" {
                        for p in &placed { assert_eq!(p["transformation"],serde_json::json!({"rotation":0.0,"translation":[5.0,5.0]})); }
                    }
                    results.push(serde_json::json!({"width_bits":output.strip_width.to_bits(),"placed":placed}));
                    swarm_free(handle);
                }
            }
            if expected==2000 {
                assert_eq!(results[0]["message"],"failed to read or initialize Swarm input");
                assert_eq!(results[1]["message"],"invalid legacy Swarm JSON or initialization failure");
            } else { assert_eq!(results[0],results[1],"{label}"); }
        }
    }

    #[test]
    fn issue9_missing_and_non_utf8_file_keep_generic_error() {
        let config=DEFAULT_C_SWARM_CONFIG_FOR_TEST.with(|c|*c);
        let temp=Issue9TempFile::new(&[0xff,0xfe]);
        for path in [&temp.0, &temp.0.with_extension("absent")] {
            let c_path=std::ffi::CString::new(path.to_string_lossy().as_bytes()).unwrap();
            assert!(swarm_init(c_path.as_ptr(),&config,std::ptr::null(),0).is_null());
            assert_eq!(swarm_get_last_error_code(),2000);
            assert_eq!(LAST_ERROR.with(|slot|slot.borrow().1.clone()),"failed to read or initialize Swarm input");
        }
        assert!(swarm_init(std::ptr::null(),&config,std::ptr::null(),0).is_null());
        assert_eq!(swarm_get_last_error_code(),2000);
    }
    fn rectangle_instance() -> ExtSPInstance {
        serde_json::from_value(serde_json::json!({
            "name":"legacy-exact-boundary",
            "strip_height":20.0,
            "items":[
                {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                    "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
                }},
                {"id":1,"demand":1,"allowed_orientations":[0.0],"shape":{
                    "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
                }}
            ]
        }))
        .unwrap()
    }

    fn curved_instance(vertex_count: usize) -> ExtSPInstance {
        let radius = 8.0f32;
        let center = 10.0f32;
        let points = (0..vertex_count)
            .map(|index| {
                let angle = std::f32::consts::TAU * index as f32 / vertex_count as f32;
                [center + radius * angle.cos(), center + radius * angle.sin()]
            })
            .collect::<Vec<_>>();
        serde_json::from_value(serde_json::json!({
            "name":"legacy-structure-simplification",
            "strip_height":24.0,
            "items":[
                {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                    "type":"simple_polygon","data":points
                }}
            ]
        }))
        .unwrap()
    }

    fn hint(item_id: i32, frozen: bool, x: f32, y: f32) -> CPlacedHint {
        CPlacedHint {
            item_id,
            is_freeze: i32::from(frozen),
            tr_x: x,
            tr_y: y,
            rot_angle_in_rad: 0.0,
            stripe_interval_y: 0.0,
            stripe_interval_x: 0.0,
        }
    }

    fn fast_config() -> crate::config::SwarmConfig {
        let mut config = DEFAULT_SWARM_CONFIG;
        config.rng_seed = Some(7);
        config
    }

    fn zero_time_phase_config() -> crate::config::SwarmConfig {
        let mut config = fast_config();
        config.expl_cfg.time_limit = Duration::ZERO;
        config.cmpr_cfg.time_limit = Duration::ZERO;
        config.polish_cfg.time_limit = Duration::ZERO;
        config
    }

    /// **WPF 의 겹침 설정이 코어까지 도달하고, 원단 경계를 열지 않는가.**
    /// (`scale-collapse` fb4ca00 에서 이식.)
    ///
    /// 이 코드베이스는 "설정을 만들고 배선을 잊는" 사고를 반복해서 겪었다. 그래서
    /// 하네스가 아니라 **step API 경로로** 둘을 단정한다 —
    /// C ABI 의 `piece_overlap_mm` 이 코어 `min_item_separation` 으로 도달하는가,
    /// 그 결과 충돌 컨테이너가 **줄어드는가**(자라면 조각이 원단 밖으로 나간다).
    #[test]
    fn wpf_overlap_setting_reaches_the_core_and_keeps_the_fabric_edge_closed() {
        const OVERLAP: f32 = 2.0;

        let mut c_cfg = DEFAULT_C_SWARM_CONFIG_FOR_TEST.with(|c| *c);
        c_cfg.fabric_tolerance.piece_overlap_mm = OVERLAP;
        let built = crate::ffi::build_config(&c_cfg);
        assert_eq!(
            built.min_item_separation,
            Some(-OVERLAP),
            "WPF 의 겹침 값이 코어 설정에 도달하지 않는다 — 켜도 아무 일도 안 일어난다"
        );

        let mut config = zero_time_phase_config();
        config.min_item_separation = built.min_item_separation;
        // 픽스처는 겹침보다 충분히 큰 원단이어야 한다. `rectangle_instance` 는 기준 폭이
        // 1.6 이라 2mm 를 줄이면 다각형이 비어 상한에 걸린다(그 경로는 swarm-core 의
        // `overlap_semantics` 가 잰다).
        let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
            "name":"overlap-wiring",
            "strip_height":200.0,
            "items":[
                {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                    "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":40.0,"height":40.0}
                }},
                {"id":1,"demand":1,"allowed_orientations":[0.0],"shape":{
                    "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":40.0,"height":40.0}
                }}
            ]
        }))
        .unwrap();
        let handle = build_handle(
            ext,
            None,
            config,
            DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(),
            vec![],
        )
        .expect(swarm_core::dmsg!("겹침 설정으로 핸들을 못 만든다"));
        let (instance, solution) = unsafe { legacy_solution(handle) };

        let nominal = instance.base_strip.fixed_height;
        assert!(
            (nominal - 200.0).abs() < 1e-6,
            "픽스처의 공칭 높이가 바뀌었다 ({nominal})"
        );

        let bbox = solution.layout_snapshot().container().outer_cd().bbox();
        let shrank = nominal - (bbox.y_max - bbox.y_min);
        assert!(
            (shrank - OVERLAP).abs() < 0.05,
            "컨테이너가 {shrank}mm 줄었다 — {OVERLAP}mm 여야 한다. \
자라고 있으면(음수) `import_spp_instance` 를 안 거치는 경로가 남아 있다는 뜻이다"
        );
        assert!(
            bbox.y_min > 0.0,
            "컨테이너 아래 경계가 {} 다 — 0 위여야 한다",
            bbox.y_min
        );

        swarm_free(handle);
    }

    #[test]
    fn configured_overlap_is_shared_by_init_phases_callbacks_and_export() {
        for overlap in [1.0_f32, 3.0, 5.0] {
            let mut native = DEFAULT_C_SWARM_CONFIG_FOR_TEST.with(|c| *c);
            native.fabric_tolerance.piece_overlap_mm = overlap;
            let mut config = zero_time_phase_config();
            config.min_item_separation = crate::ffi::build_config(&native).min_item_separation;
            let ext:ExtSPInstance=serde_json::from_value(serde_json::json!({"name":"shared-original-overlap","strip_height":100.0,
                "items":(0..3).map(|id|serde_json::json!({"id":id,"demand":1,"allowed_orientations":[0.0],
                    "shape":{"type":"rectangle","data":{"x_min":0,"y_min":0,"width":40,"height":40}}})).collect::<Vec<_>>()
            })).unwrap();
            let handle = build_handle(
                ext,
                None,
                config,
                DEFAULT_CALLBACK_OPTIONS,
                crate::ffi::InitialPlacementSettings::default(),
                vec![],
            )
            .unwrap();
            let mut capture = CallbackCapture::default();
            swarm_set_callback(
                handle,
                Some(capture_callback),
                (&mut capture as *mut CallbackCapture).cast(),
                1,
            );
            for phase in [
                swarm_alg_phase as extern "C" fn(*mut SwarmStepHandle) -> i32,
                swarm_cp_phase,
                swarm_refine_phase,
            ] {
                assert_eq!(phase(handle), 0);
                let (instance, solution) = unsafe { legacy_solution(handle) };
                assert_eq!(
                    instance.items[0].0.shape_orig().modify_config.offset,
                    Some(-overlap / 2.)
                );
                assert!(
                    swarm_core::original_overlap::validate(&instance, &solution, &[]).is_ok()
                );
                assert_parseable_certified_export(handle, &instance);
            }
            assert!(capture.final_count > 0);
            assert!(capture.final_had_json);
            swarm_free(handle);
        }
    }

    #[test]
    fn structure_enabled_keeps_conservative_polygon_simplification() {
        let ext = curved_instance(96);
        let mut config = fast_config();
        config.structure.enabled = true;
        config.poly_simpl_tolerance = Some(0.01);

        let simplified_importer = swarm_core::util::io::new_importer(config.cde_config, config.poly_simpl_tolerance, config.min_item_separation, config.narrow_concavity_cutoff_ratio);
        let source_importer = swarm_core::util::io::new_importer(config.cde_config, None, config.min_item_separation, config.narrow_concavity_cutoff_ratio);
        let simplified = jagua_rs::probs::spp::io::import_instance(&simplified_importer, &ext)
            .expect(swarm_core::dmsg!("curved fixture must import with conservative simplification"));
        let source = jagua_rs::probs::spp::io::import_instance(&source_importer, &ext)
            .expect(swarm_core::dmsg!("curved fixture must import without simplification"));
        assert!(
            simplified.item(0).shape_cd().n_vertices() < source.item(0).shape_cd().n_vertices(),
            "fixture must prove that simplification actually removed vertices"
        );

        let handle = build_handle(
            ext,
            None,
            config,
            DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(),
            vec![],
        )
        .unwrap();
        let (actual, _) = unsafe { legacy_solution(handle) };
        assert_eq!(
            actual.item(0).shape_cd().n_vertices(),
            simplified.item(0).shape_cd().n_vertices(),
            "Structure must not silently switch the entire search to source vertices"
        );
        swarm_free(handle);
    }

    fn internal_instance() -> SPInstance {
        let config = fast_config();
        let importer = swarm_core::util::io::new_importer(config.cde_config, config.poly_simpl_tolerance, config.min_item_separation, config.narrow_concavity_cutoff_ratio);
        jagua_rs::probs::spp::io::import_instance(&importer, &rectangle_instance()).unwrap()
    }

    #[test]
    fn place_group_diagnostics_and_progress_cache_skip_only_explicit_internal_pairs() {
        use crate::optimizer::place_groups::{PlaceGroupMember, PlaceGroupSpec, PlaceGroups};
        let instance = internal_instance();
        let mut p = SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
        p.change_strip_width(40.0).expect(swarm_core::dmsg!("strip width must be positive"));
        for hint in [hint(0, false, 5.0, 5.0), hint(1, false, 6.0, 5.0)] {
            p.place_item(legacy_hint_placement(&instance, &hint).unwrap());
        }
        let groups = PlaceGroups::new(
            &p.instance(),
            vec![PlaceGroupSpec {
                id: 42,
                members: p
                    .layout()
                    .placed_items()
                    .values()
                    .map(|pi| PlaceGroupMember {
                        item_id: pi.item().idx(),
                        initial_transform: pi.d_transf(),
                    })
                    .collect(),
            }],
        )
        .unwrap();
        for collect_loss in [false, true] {
            for collect_collision_status in [false, true] {
                let options = CallbackOptions {
                    collect_loss,
                    collect_collision_status,
                };
                let d = compute_callback_diagnostics_with_groups(&p.layout(), options, &[], &groups);
                assert_eq!(d.loss, collect_loss.then_some(0.0));
                if let Some(status) = d.collision_status {
                    assert!(status.iter().all(|s| !s.collision));
                }
                let strict = compute_callback_diagnostics_with_frozen(&p.layout(), options, &[]);
                if let Some(loss) = strict.loss {
                    assert!(loss > 0.0);
                }
                if let Some(status) = strict.collision_status {
                    assert!(status.iter().all(|s| s.collision));
                }
            }
        }
        let s = p.save();
        let strict = Rc::new(RefCell::new(ExactSolutionCache::default()));
        retain_exact_solution(&strict, &ReportType::Final, &p.instance(), &s, true);
        assert!(strict.borrow().certified.is_none());
        let cache = Rc::new(RefCell::new(
            ExactSolutionCache::default().with_groups(&groups),
        ));
        retain_exact_solution(&cache, &ReportType::Final, &p.instance(), &s, true);
        assert!(cache.borrow().certified.is_some());
        let mut cleared =
            std::mem::take(&mut *cache.borrow_mut()).with_groups(&PlaceGroups::default());
        assert!(cleared.certified.is_none());
        cleared.pending_version = cleared.pending_version.wrapping_add(1);
        *cache.borrow_mut() = cleared;
        retain_exact_solution(&cache, &ReportType::Final, &p.instance(), &s, true);
        assert!(cache.borrow().certified.is_none());
    }

    #[test]
    fn frozen_diagnostics_keep_movable_lock_collisions() {
        let prob = hinted_problem(&[hint(0, true, -2.0, 5.0), hint(1, true, -2.0, 5.0)]);
        let options = CallbackOptions {
            collect_loss: true,
            collect_collision_status: true,
        };
        let exempt = compute_callback_diagnostics_with_frozen(&prob.layout(), options, &[0, 1]);
        assert_eq!(exempt.loss, Some(0.0));
        assert!(
            exempt
                .collision_status
                .unwrap()
                .iter()
                .all(|s| !s.collision)
        );
        let mixed = compute_callback_diagnostics_with_frozen(&prob.layout(), options, &[0]);
        assert!(mixed.loss.unwrap() > 0.0);
        assert!(mixed.collision_status.unwrap().iter().all(|s| s.collision));
    }

    fn hinted_problem(hints: &[CPlacedHint]) -> SPProblem {
        let instance = internal_instance();
        let placements = hints
            .iter()
            .map(|hint| LegacySeedPlacement {
                placement: legacy_hint_placement(&instance, hint).unwrap(),
                frozen: hint.is_freeze != 0,
            })
            .collect::<Vec<_>>();
        legacy_seed_problem(&instance, &placements, 0.0).unwrap()
    }

    unsafe fn legacy_solution(handle: *mut SwarmStepHandle) -> (SPInstance, SPSolution) {
        let handle = unsafe { &*handle };
        let engine = handle.lock_engine();
        match &*engine {
            HandleEngine::Legacy(state) => {
                (state.instance.clone(), state.solution.clone().unwrap())
            }
            HandleEngine::Grouped(_) => panic!("{}", swarm_core::dmsg!("expected a Legacy handle")),
        }
    }

    unsafe fn legacy_has_input_seed(handle: *mut SwarmStepHandle) -> bool {
        let handle = unsafe { &*handle };
        let engine = handle.lock_engine();
        match &*engine {
            HandleEngine::Legacy(state) => state.has_input_seed,
            HandleEngine::Grouped(_) => panic!("{}", swarm_core::dmsg!("expected a Legacy handle")),
        }
    }

    fn assert_parseable_certified_export(
        handle: *mut SwarmStepHandle,
        instance: &SPInstance,
    ) -> ExtSPSolution {
        let json = swarm_get_solution_json(handle);
        assert!(!json.is_null(), "certified phase output must be exportable");
        let text = unsafe { CStr::from_ptr(json) }
            .to_str()
            .expect(swarm_core::dmsg!("solution JSON must be UTF-8"))
            .to_owned();
        crate::ffi::swarm_free_string(json);
        let exported: ExtSPSolution =
            serde_json::from_str(&text).expect(swarm_core::dmsg!("solution JSON must match ExtSPSolution"));

        let placements = legacy_ext_seed_placements(instance, &exported)
            .expect(swarm_core::dmsg!("exported public transformations must import cleanly"));
        let rebuilt = legacy_seed_problem(instance, &placements, exported.strip_width)
            .expect(swarm_core::dmsg!("exported layout must rebuild through exact CDE"))
            .save();
        assert_eq!(
            rebuilt.layout_snapshot().placed_items().len(),
            instance.total_item_qty()
        );
        assert!(
            crate::structure::pipeline::solution_validation_error(instance, &rebuilt).is_none()
        );
        exported
    }

    #[test]
    #[cfg(feature = "adaptive")]
    fn early_explore_callback_stop_preserves_final_and_export() {
        struct Capture {
            handle: *mut SwarmStepHandle,
            stopped_width: Option<f32>,
            final_width: Option<f32>,
            stopped_json: Option<String>,
        }
        extern "C" fn callback(
            report: i32,
            width: f32,
            _: f32,
            json: *const c_char,
            user_data: *mut std::ffi::c_void,
        ) {
            let state = unsafe { &mut *user_data.cast::<Capture>() };
            if report == 0 && width < 30.0 && !json.is_null() && state.stopped_width.is_none() {
                state.stopped_width = Some(width);
                state.stopped_json = Some(
                    unsafe { CStr::from_ptr(json) }
                        .to_string_lossy()
                        .into_owned(),
                );
                swarm_request_stop(state.handle);
            }
            if report == 4 {
                state.final_width = Some(width);
            }
        }
        let mut config = zero_time_phase_config();
        config.expl_cfg.time_limit = Duration::from_secs(3);
        config.expl_cfg.separator_config.n_workers = 2;
        let handle = build_handle(
            rectangle_instance(),
            None,
            config,
            DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(),
            vec![],
        )
        .unwrap();
        {
            let mut engine = unsafe { &*handle }.lock_engine();
            let state = engine.legacy_mut();
            state.prob = SPProblem::new(state.instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
            state.prob.change_strip_width(30.0).expect(swarm_core::dmsg!("strip width must be positive"));
            for (item_id, x) in [(0, 7.0), (1, 18.0)] {
                state.prob.place_item(SPPlacement {
                    item_idx: item_id,
                    d_transf: DTransformation::new(0.0, (x, 7.0)),
                });
            }
            state.solution = Some(state.prob.save());
            state.solutions = vec![state.prob.save()];
        }
        let mut capture = Capture {
            handle,
            stopped_width: None,
            final_width: None,
            stopped_json: None,
        };
        swarm_set_callback(
            handle,
            Some(callback),
            (&mut capture as *mut Capture).cast(),
            1,
        );
        assert_eq!(swarm_alg_phase(handle), 0);
        assert!(
            capture.stopped_width.is_some(),
            "certified improvement must reach callback before deadline"
        );
        assert_eq!(capture.final_width, capture.stopped_width);
        let (instance, solution) = unsafe { legacy_solution(handle) };
        assert_eq!(Some(solution.strip_width()), capture.stopped_width);
        let exported = assert_parseable_certified_export(handle, &instance);
        let callback_json: serde_json::Value =
            serde_json::from_str(capture.stopped_json.as_ref().unwrap()).unwrap();
        let callback_solution: ExtSPSolution =
            serde_json::from_value(callback_json["solution"].clone()).unwrap();
        assert_eq!(
            serde_json::to_value(&exported).unwrap()["layout"],
            serde_json::to_value(&callback_solution).unwrap()["layout"]
        );
        swarm_free(handle);
    }

    #[test]
    fn legacy_init_explore_compress_polish_chain_exports_only_certified_json() {
        let handle = build_handle(
            rectangle_instance(),
            None,
            zero_time_phase_config(),
            DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(),
            vec![],
        )
        .unwrap();
        let mut capture = CallbackCapture::default();
        swarm_set_callback(
            handle,
            Some(capture_callback),
            (&mut capture as *mut CallbackCapture).cast(),
            1,
        );

        assert_eq!(swarm_alg_phase(handle), 0);
        assert_eq!(swarm_get_last_error_code(), 0);
        assert_eq!(
            capture.final_count, 1,
            "a successful Explore boundary must expose one certified Final callback"
        );
        assert_eq!(swarm_cp_phase(handle), 0);
        assert_eq!(capture.final_count, 2);
        assert_eq!(swarm_refine_phase(handle), 0);
        assert_eq!(capture.final_count, 3);
        assert!(capture.final_had_json);
        let (instance, solution) = unsafe { legacy_solution(handle) };
        assert!(
            crate::structure::pipeline::solution_validation_error(&instance, &solution).is_none()
        );
        for json in &capture.final_json {
            let payload: serde_json::Value =
                serde_json::from_str(json).expect(swarm_core::dmsg!("Final callback payload must be JSON"));
            let exported: ExtSPSolution = serde_json::from_value(
                payload
                    .get("solution")
                    .expect(swarm_core::dmsg!("callback payload must contain solution"))
                    .clone(),
            )
            .expect(swarm_core::dmsg!("callback solution must match ExtSPSolution"));
            let placements = legacy_ext_seed_placements(&instance, &exported).unwrap();
            let rebuilt = legacy_seed_problem(&instance, &placements, exported.strip_width)
                .unwrap()
                .save();
            assert!(
                crate::structure::pipeline::solution_validation_error(&instance, &rebuilt)
                    .is_none()
            );
        }
        assert_parseable_certified_export(handle, &instance);
        swarm_free(handle);
    }

    #[test]
    fn grouped_explore_reports_final_and_clears_reentrant_busy_error_on_success() {
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items","group_workers":1,"quantum_ms":1,"locked_prefix_gap_mm":0.0},
            "instance":{"name":"grouped-reentrant-callback","strip_height":20.0,"items":[
                {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                    "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
                }},
                {"id":1,"demand":1,"allowed_orientations":[0.0],"shape":{
                    "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
                }}
            ]},
            "regions":[],
            "place_groups":[]
        })
        .to_string();
        let input = grouped_input::parse_and_validate(&json).unwrap();
        let mut config = fast_config();
        config.expl_cfg.time_limit = Duration::ZERO;
        let handle = build_grouped_handle(input, config, DEFAULT_CALLBACK_OPTIONS).unwrap();
        let mut capture = ReentrantGroupedCapture {
            handle,
            final_count: 0,
            nested_result: 0,
            nested_error: 0,
            attempted: false,
        };
        swarm_set_callback(
            handle,
            Some(reentrant_grouped_callback),
            (&mut capture as *mut ReentrantGroupedCapture).cast(),
            0,
        );

        assert_eq!(swarm_alg_phase(handle), 0);
        assert_eq!(capture.final_count, 1);
        assert_eq!(capture.nested_result, -1);
        assert_eq!(capture.nested_error, 3000);
        assert_eq!(
            swarm_get_last_error_code(),
            0,
            "the successful outer phase must clear its callback's reentrant error"
        );
        swarm_free(handle);
    }

    #[test]
    fn grouped_legacy_panic_is_restored_and_mapped_to_ffi_9001() {
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items","group_workers":1,"quantum_ms":1,"locked_prefix_gap_mm":0.0},
            "instance":{"name":"grouped-panic-boundary","strip_height":20.0,"items":[
                {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                    "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
                }},
                {"id":1,"demand":1,"allowed_orientations":[0.0],"shape":{
                    "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
                }}
            ]},
            "regions":[],
            "place_groups":[]
        })
        .to_string();
        let input = grouped_input::parse_and_validate(&json).unwrap();
        let mut config = fast_config();
        config.expl_cfg.time_limit = Duration::ZERO;
        let handle = build_grouped_handle(input, config, DEFAULT_CALLBACK_OPTIONS).unwrap();
        {
            let handle_ref = unsafe { &*handle };
            let mut engine = handle_ref.lock_engine();
            match &mut *engine {
                HandleEngine::Grouped(grouped) => grouped.solver.panic_next_legacy_phase_for_test(),
                HandleEngine::Legacy(_) => panic!("{}", swarm_core::dmsg!("expected a grouped handle")),
            }
        }

        assert_eq!(swarm_alg_phase(handle), 1);
        assert_eq!(swarm_get_last_error_code(), 9001);
        {
            let handle_ref = unsafe { &*handle };
            let engine = handle_ref.lock_engine();
            match &*engine {
                HandleEngine::Grouped(grouped) => {
                    assert_eq!(
                        grouped.solver.phase(),
                        crate::grouped_engine::GroupPhase::Init
                    )
                }
                HandleEngine::Legacy(_) => panic!("{}", swarm_core::dmsg!("expected a grouped handle")),
            }
        }

        assert_eq!(swarm_alg_phase(handle), 0);
        assert_eq!(swarm_get_last_error_code(), 0);
        swarm_free(handle);
    }

    #[test]
    fn legacy_init_compress_polish_chain_skips_explore_and_still_exports_certified_json() {
        let handle = build_handle(
            rectangle_instance(),
            None,
            zero_time_phase_config(),
            DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(),
            vec![],
        )
        .unwrap();

        assert_eq!(swarm_cp_phase(handle), 0);
        assert_eq!(swarm_refine_phase(handle), 0);
        let (instance, solution) = unsafe { legacy_solution(handle) };
        assert!(
            crate::structure::pipeline::solution_validation_error(&instance, &solution).is_none()
        );
        assert_parseable_certified_export(handle, &instance);
        swarm_free(handle);
    }

    /// step API(WPF 경로)에서 **NFP-LBF 시드와 탐색 단계 선택 설정이 실제로 전달되는지** 본다.
    ///
    /// 예전에는 이 설정들이 `optimize_configured` 에만 배선되어 있었고 `ffi_step.rs` 는
    /// 읽지도 않았다. 그래서 WPF 에서 NFP 나 탈출 기법을 켜도 결과가 한 비트도 바뀌지
    /// 않았다(`ffi_step.rs` 안에 `nfp_lbf` 참조가 0건이었다). 이 테스트가 그 회귀를 막는다.
    #[test]
    fn step_api_config_carries_nfp_and_escape_flags() {
        let mut c_cfg = DEFAULT_C_SWARM_CONFIG_FOR_TEST.with(|c| *c);
        c_cfg.nfp_lbf.enabled = 1;
        c_cfg.nfp_lbf.n_orderings = 4;
        c_cfg.nfp_lbf.concave_simplify_tolerance = 0.02;
        c_cfg.escape.enabled = 1;
        c_cfg.nfp_integration.exact_separation_vector = 1;
        c_cfg.nfp_integration.ordering_exploration = 1;

        let cfg = crate::ffi::build_config(&c_cfg);

        assert!(cfg.nfp_lbf.enabled, "NFP-LBF 시드 플래그가 전달되어야 한다");
        assert_eq!(cfg.nfp_lbf.n_orderings, 4);
        assert!((cfg.nfp_lbf.concave_simplify_tolerance - 0.02).abs() < 1e-6);
        assert!(cfg.escape_enabled, "탈출 플래그가 전달되어야 한다");
        assert!(cfg.nfp_integration.exact_separation_vector);
        assert!(cfg.nfp_integration.ordering_exploration);

        // 그리고 `ffi_step.rs` 가 그 값들을 실제로 참조해야 한다.
        // 소스에 참조가 하나도 없으면(예전 상태) 이 단정이 깨진다.
        let src = production_source();
        assert!(
            src.contains("build_best_initial_nest"),
            "step API 가 NFP-LBF 시드를 호출하지 않는다"
        );
        assert!(
            src.contains("select_exploration_phase"),
            "step API 가 탐색 단계를 선택하지 않는다"
        );
        assert!(
            src.contains("escape_driven_exploration_phase_with_nfp"),
            "step API 가 탈출 탐색 단계를 연결하지 않았다"
        );
        assert!(
            src.contains("nfp_ordering_exploration_phase"),
            "step API 가 순수 NFP 순서 탐색을 연결하지 않았다"
        );
        // 후처리도 핸들의 설정을 그대로 받아야 한다. 예전에는 기본값 래퍼를 타서
        // `lp_solver`(백엔드 4종·QP)와 `nfp_hole_filling` 이 통째로 버려졌다.
        assert!(
            src.contains("&h.lp_solver"),
            "step API 의 후처리가 LP·QP 백엔드 설정을 전달하지 않는다 — WPF 에서 골라도 microlp 가 돈다"
        );
        assert!(
            src.contains("&h.nfp_integration"),
            "step API 의 후처리가 NFP 활용 설정을 전달하지 않는다 — hole filling 체크박스가 죽는다"
        );
        // structure-first 경로도 같은 두 설정을 받아야 한다.
        assert!(
            src.contains("&lp_solver,"),
            "structure-first 경로가 LP·QP 백엔드 설정을 받지 않는다"
        );
        // 실험 엔진도 실제로 불려야 한다. 스위치만 만들고 호출을 빠뜨리는 것이
        // 이 코드베이스가 네 번 겪은 결함이다.
        assert!(
            src.contains("refplace::build_reference_nest"),
            "레퍼런스 설계 엔진이 step API 에서 호출되지 않는다 — 체크박스가 죽는다"
        );
        // LBF 순서 모드도 마찬가지다. WPF Init 경로(`LBFBuilder::from_problem`)와
        // structure-first 경로 **양쪽** 에 배선돼야 한다 — 한쪽만 걸면 그 경로만
        // 조용히 기본 순서로 돈다.
        assert!(
            src.contains(".with_order_mode(config.lbf_order)"),
            "Init 경로의 LBF 가 순서 모드를 받지 않는다 — 콤보박스가 죽는다"
        );
        // Issue4 unified callback-enabled/disabled Structure execution through
        // the common collector. Its actual paths are exercised by the Issue4
        // and Issue6 phase/callback regressions; source call counts are not a
        // forwarding contract.
    }

    /// 초기 배치 전략 7종과 LP·QP 솔버 선택이 C 경계에서 배치까지 도달하는가.
    ///
    /// 이 코드베이스는 같은 결함을 두 번 겪었다 — 설정과 UI 와 ABI 를 다 만들어 놓고
    /// **실어 보내는 줄 하나를 빠뜨려서** 기능이 통째로 죽어 있었다. 빌드도 테스트도
    /// 통과했고, 아무도 "이 값이 실제로 흘러가는가" 를 묻지 않았다.
    ///
    /// 그래서 값이 실제로 변환되는지와, `to_config` 가 그 값을 **넘겨받는지**를 함께 본다.
    #[test]
    fn step_api_carries_the_initial_placement_strategies_and_lp_backend() {
        let mut c_cfg = DEFAULT_C_SWARM_CONFIG_FOR_TEST.with(|c| *c);
        c_cfg.nfp_strategy.instance_rotations = 1;
        c_cfg.nfp_strategy.continuous_rotation_step_deg = 45;
        c_cfg.nfp_strategy.pairwise_clustering = 1;
        c_cfg.nfp_strategy.cluster_min_gain = 0.12;
        c_cfg.nfp_strategy.contact_maximization = 1;
        c_cfg.nfp_strategy.pocket_first = 1;
        c_cfg.nfp_strategy.beam_search = 1;
        c_cfg.nfp_strategy.beam_width = 6;
        c_cfg.nfp_strategy.best_fit_selection = 1;
        c_cfg.nfp_strategy.best_fit_lookahead = 20;
        c_cfg.nfp_strategy.sliding_compaction = 1;
        c_cfg.nfp_strategy.sliding_rounds = 3;
        c_cfg.lp_solver.backend = 1; // clarabel
        c_cfg.lp_solver.quadratic_regularization = 0.25;
        c_cfg.lp_solver.fallback = 0;
        c_cfg.lp_solver.time_limit_ms = 1234;

        let cfg = crate::ffi::build_config(&c_cfg);

        let s = cfg.nfp_strategy;
        assert!(s.instance_rotations, "① 회전 플래그");
        assert_eq!(s.continuous_rotation_step_deg, 45);
        assert!(s.pairwise_clustering, "② 클러스터링 플래그");
        assert!((s.cluster_min_gain - 0.12).abs() < 1e-6);
        assert!(s.contact_maximization, "③ 접촉 최대화 플래그");
        assert!(s.pocket_first, "④ 포켓 우선 플래그");
        assert!(s.beam_search, "⑤ 빔 서치 플래그");
        assert_eq!(s.beam_width, 6);
        assert!(s.best_fit_selection, "⑥ best-fit 플래그");
        assert_eq!(s.best_fit_lookahead, 20);
        assert!(s.sliding_compaction, "⑦ 슬라이딩 플래그");
        assert_eq!(s.sliding_rounds, 3);
        assert!(s.any_enabled());

        assert_eq!(
            cfg.lp_solver.backend,
            swarm_core::lp_solver::LpSolverKind::Clarabel
        );
        assert!(cfg.lp_solver.uses_quadratic(), "clarabel 은 QP 를 푼다");
        assert!(!cfg.lp_solver.fallback);
        assert_eq!(cfg.lp_solver.time_limit_ms, 1234);

        // **그리고 그 설정이 실제로 시드 생성에 실려야 한다.**
        // `to_config` 가 전략을 인자로 받으므로, 여기서 빠뜨리면 컴파일이 안 된다 —
        // 그 계약이 유지되고 있는지를 소스로 확인한다.
        let src = production_source();
        assert!(
            src.contains("to_config(config.nfp_strategy)"),
            "step API 가 초기 배치 전략을 시드 설정에 싣지 않는다 —              켜도 아무 일도 일어나지 않는다"
        );

        // 기본값은 전부 꺼짐이어야 한다. 기존 동작이 조용히 바뀌면 안 된다.
        let default_cfg = crate::ffi::build_config(&DEFAULT_C_SWARM_CONFIG_FOR_TEST.with(|c| *c));
        assert!(
            !default_cfg.nfp_strategy.any_enabled(),
            "기본값에서 전략이 켜져 있으면 기존 측정치가 전부 무효가 된다"
        );
        assert_eq!(
            default_cfg.lp_solver.backend,
            swarm_core::lp_solver::LpSolverKind::Microlp,
            "기본 솔버는 기존 동작(단체법)이어야 한다"
        );
        assert!(!default_cfg.lp_solver.uses_quadratic());
    }

    thread_local! {
        /// `swarm_get_default_config` 가 채우는 값과 같은 C 기본 설정.
        static DEFAULT_C_SWARM_CONFIG_FOR_TEST: CSwarmConfig = {
            let mut cfg = std::mem::MaybeUninit::<CSwarmConfig>::zeroed();
            crate::ffi::swarm_get_default_config(cfg.as_mut_ptr());
            unsafe { cfg.assume_init() }
        };
    }

    /// N06 초기배치 페이즈가 시드를 교체하고 `Phase::Init` 을 유지하는지.
    ///
    /// 이 함수는 `#[test]` 를 잃어버려 **한 번도 실행된 적이 없었다**. 바로 위 테스트에
    /// 속성이 두 개 붙어 있었던 것이 원인이다(중복 하나가 이 함수의 것이었다).
    /// `cargo test` 는 조용히 통과했고 dead_code 경고만 남았다.
    #[test]
    fn initial_phase_replaces_the_seed_and_keeps_the_init_phase() {
        let initial = crate::ffi::InitialPlacementSettings {
            enabled: true,
            time_limit_secs: 2.0,
            grasp: 1,
            kicks: 2,
            ..crate::ffi::InitialPlacementSettings::default()
        };
        let handle = build_handle(
            rectangle_instance(),
            None,
            zero_time_phase_config(),
            DEFAULT_CALLBACK_OPTIONS,
            initial,
            vec![],
        )
        .unwrap();

        assert_eq!(swarm_init_phase(handle), 0);
        assert_eq!(swarm_get_last_error_code(), 0);
        let (instance, solution) = unsafe { legacy_solution(handle) };
        assert!(
            crate::structure::pipeline::solution_validation_error(&instance, &solution).is_none(),
            "initial phase must commit an exact-certified solution"
        );
        // Phase=Init 이 유지되어 Explore→Compress 를 그대로 이어 붙일 수 있다.
        assert_eq!(swarm_alg_phase(handle), 0);
        assert_eq!(swarm_cp_phase(handle), 0);
        let (instance, solution) = unsafe { legacy_solution(handle) };
        assert!(
            crate::structure::pipeline::solution_validation_error(&instance, &solution).is_none()
        );
        assert_parseable_certified_export(handle, &instance);
        swarm_free(handle);
    }

    #[test]
    fn initial_phase_disabled_or_out_of_order_is_rejected() {
        let handle = build_handle(
            rectangle_instance(),
            None,
            zero_time_phase_config(),
            DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(),
            vec![],
        )
        .unwrap();

        // 비활성 설정에서는 명시적 오류를 낸다 (조용한 no-op 금지).
        assert_eq!(swarm_init_phase(handle), -1);
        assert_eq!(swarm_get_last_error_code(), 3000);
        // Explore 이후에는 순서 위반이다.
        assert_eq!(swarm_alg_phase(handle), 0);
        assert_eq!(swarm_init_phase(handle), -1);
        assert_eq!(swarm_get_last_error_code(), 3000);
        swarm_free(handle);
    }

    #[test]
    fn legacy_phases_run_in_any_order_and_repetition() {
        let handle = build_handle(
            rectangle_instance(),
            None,
            zero_time_phase_config(),
            DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(),
            vec![],
        )
        .unwrap();

        // 탐색·압축·후처리는 순서와 횟수가 자유다. 어느 호출이든 공개된 현재 해에서
        // 출발하고, 호출마다 인증된 해를 남긴다.
        let calls: [(&str, extern "C" fn(*mut SwarmStepHandle) -> i32); 9] = [
            ("polish straight after init", swarm_refine_phase),
            ("explore", swarm_alg_phase),
            ("explore again", swarm_alg_phase),
            ("compress", swarm_cp_phase),
            ("compress again", swarm_cp_phase),
            ("explore after compress", swarm_alg_phase),
            ("polish", swarm_refine_phase),
            ("polish again", swarm_refine_phase),
            ("compress after polish", swarm_cp_phase),
        ];
        for (label, call) in calls {
            assert_eq!(call(handle), 0, "{label}");
            assert_eq!(swarm_get_last_error_code(), 0, "{label}");
            let (instance, _) = unsafe { legacy_solution(handle) };
            assert_parseable_certified_export(handle, &instance);
        }
        swarm_free(handle);
    }

    #[test]
    fn legacy_busy_and_stopped_boundaries_set_an_actionable_error_code() {
        let handle = build_handle(
            rectangle_instance(),
            None,
            zero_time_phase_config(),
            DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(),
            vec![],
        )
        .unwrap();
        let handle_ref = unsafe { &*handle };
        let phase_guard = handle_ref
            .lifecycle
            .begin_phase()
            .expect(swarm_core::dmsg!("test must reserve the phase slot"));

        let phase_entries = [
            (
                "Explore",
                swarm_alg_phase as extern "C" fn(*mut SwarmStepHandle) -> i32,
            ),
            ("Compress", swarm_cp_phase),
            ("Polish", swarm_refine_phase),
        ];
        for (name, phase) in phase_entries {
            assert_eq!(phase(handle), -1, "busy {name} must be rejected");
            assert_eq!(
                swarm_get_last_error_code(),
                3000,
                "a busy {name} call must not fail with last_error=0"
            );
        }
        let json = swarm_get_solution_json(handle);
        assert!(json.is_null());
        assert_eq!(
            swarm_get_last_error_code(),
            3000,
            "a busy export must not return NULL with last_error=0"
        );
        drop(phase_guard);

        let json = swarm_get_solution_json(handle);
        assert!(!json.is_null());
        assert_eq!(swarm_get_last_error_code(), 0);
        crate::ffi::swarm_free_string(json);

        swarm_request_stop(handle);
        for (name, phase) in phase_entries {
            assert_eq!(phase(handle), -1, "stopped {name} must be rejected");
            assert_eq!(
                swarm_get_last_error_code(),
                3000,
                "a stopped {name} call must report why it was rejected"
            );
        }
        swarm_free(handle);
    }

    #[test]
    fn legacy_init_repairs_overlapping_movable_hints() {
        clear_last_error();
        let hints = [hint(0, false, 5.0, 5.0), hint(1, false, 5.0, 5.0)];
        let handle = build_handle(
            rectangle_instance(),
            None,
            fast_config(),
            DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(),
            hints.to_vec(),
        )
        .expect(swarm_core::dmsg!("movable hints should fall back to a fresh LBF baseline"));
        let (instance, solution) = unsafe { legacy_solution(handle) };

        assert!(
            crate::structure::pipeline::solution_validation_error(&instance, &solution).is_none()
        );
        assert!(
            !unsafe { legacy_has_input_seed(handle) },
            "discarded movable hints must not bypass Structure P1"
        );
        assert_eq!(swarm_get_last_error_code(), 0);
        swarm_free(handle);
    }


    #[cfg(feature = "structure-first")]
    fn issue4_edge_handle() -> (*mut SwarmStepHandle, SPSolution, f32) {
        issue4_edge_handle_with_height(20.0)
    }

    #[cfg(feature = "structure-first")]
    fn issue4_edge_handle_with_height(height: f32) -> (*mut SwarmStepHandle, SPSolution, f32) {
        let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
            "name": "issue4-sharp-tip", "strip_height": height,
            "items": [
                {"id": 0, "demand": 1, "allowed_orientations": [0.0],
                 "shape": {"type": "simple_polygon", "data": [[0,0],[40,4],[0,8]]}},
                {"id": 1, "demand": 1, "allowed_orientations": [0.0],
                 "shape": {"type": "rectangle", "data": {
                     "x_min": 0, "y_min": 0, "width": 4, "height": 8}}}
            ]
        })).unwrap();
        let mut config = zero_time_phase_config();
        config.min_item_separation = Some(-1.0);
        config.init_settle_every = 0;
        config.init_compaction_ms = 0;
        config.structure.enabled = true;
        config.structure.p2_time_limit = Duration::ZERO;
        config.structure.p3_time_limit = Some(Duration::ZERO);
        config.structure.p4_cycles = 0;
        config.structure.p5_snap_rotations = false;
        let handle = build_handle(ext, None, config, DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(), vec![]).unwrap();
        let mut engine = unsafe { &*handle }.lock_engine();
        let state = engine.legacy_mut();
        let mut prob = SPProblem::new(state.instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
        prob.change_strip_width(40.0).expect(swarm_core::dmsg!("strip width must be positive"));
        for (item_id, x) in [(0, 4.3), (1, 0.5)] {
            prob.place_item(legacy_hint_placement(&state.instance,
                &hint(item_id, item_id == 1, x, 6.0)).unwrap());
        }
        let raw = prob.save();
        assert!(crate::structure::pipeline::solution_validation_error_with_frozen_ids(
            &state.instance, &raw, &[1]).is_none(), "fixture must be raw but exact");
        assert!(swarm_core::fabric_edge::inspect(&prob).unwrap().iter()
            .any(|(_, o)| o.item_id == 0 && o.right > 4.0));
        let outcome = swarm_core::fabric_edge::repair(&mut prob, 0.05,
            &|id| id != 1, &BasicTerminator::new()).unwrap();
        assert!(outcome.widened > 0.01, "fixture must force real widening: {outcome:?}");
        assert!(crate::structure::pipeline::solution_validation_error_with_frozen_ids(
            &state.instance, &prob.save(), &[1]).is_none(), "real repaired layout must certify");
        let expected_width = prob.strip_width();
        state.prob = SPProblem::new(state.instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
        crate::util::frozen::restore(&mut state.prob, &raw);
        state.solution = Some(raw.clone());
        state.solutions = vec![raw.clone()];
        state.frozen_item_ids = vec![1];
        state.has_input_seed = true;
        // Fault-inject a raw native phase boundary after Init: real repair,
        // exact geometry and frozen reference are exercised without optimizer timing.
        state.fabric_edge_margin_mm = Some(0.05);
        drop(engine);
        (handle, raw, expected_width)
    }

    #[cfg(feature = "structure-first")]
    fn issue4_run_edge_boundary(with_callback: bool, stop_at: Option<i32>, skip_matching: usize) {
        struct Capture {
            handle: *mut SwarmStepHandle,
            stop_at: Option<i32>,
            skip_matching: usize,
            events: Vec<(i32, f32, f32, String)>,
        }
        extern "C" fn callback(kind: i32, width: f32, density: f32,
            json: *const c_char, data: *mut std::ffi::c_void) {
            let c = unsafe { &mut *data.cast::<Capture>() };
            if !json.is_null() {
                c.events.push((kind, width, density,
                    unsafe { CStr::from_ptr(json) }.to_str().unwrap().to_owned()));
            }
            if c.stop_at == Some(kind) {
                if c.skip_matching > 0 {
                    c.skip_matching -= 1;
                } else {
                    c.stop_at = None;
                    swarm_request_stop(c.handle);
                }
            }
        }
        let (handle, reference, expected_width) = issue4_edge_handle();
        let mut capture = Capture { handle, stop_at, skip_matching, events: vec![] };
        if with_callback {
            swarm_set_callback(handle, Some(callback), (&mut capture as *mut Capture).cast(), 1);
        }
        assert_eq!(swarm_gp_phase(handle), 0);
        let (instance, final_solution) = unsafe { legacy_solution(handle) };
        let final_ext = assert_parseable_certified_export(handle, &instance);
        assert_eq!(final_solution.strip_width(), expected_width,
            "committed snapshot must include real edge widening, also on cancellation");
        assert!(crate::util::assertions::frozen_poses_preserved(
            &reference, &final_solution, &[false, true]));
        let report_ptr = swarm_get_structure_report_json(handle);
        assert!(!report_ptr.is_null());
        let report: serde_json::Value = serde_json::from_str(
            unsafe { CStr::from_ptr(report_ptr) }.to_str().unwrap()).unwrap();
        crate::ffi::swarm_free_string(report_ptr);
        assert_eq!(report["final_width"].as_f64().unwrap() as f32, final_solution.strip_width(),
            "report final width and getter must describe the same canonical snapshot");
        assert_eq!(report["final_density"].as_f64().unwrap() as f32, final_solution.density());
        let cancelled = stop_at == Some(0);
        assert_eq!(report["cancelled"], cancelled);
        if with_callback {
            assert_eq!(capture.events.iter().filter(|e| e.0 == 4).count(), usize::from(!cancelled),
                "cancelled runs must not fabricate a completion Final");
            if cancelled && skip_matching == 0 {
                assert_eq!(capture.events.iter().map(|event| event.0).collect::<Vec<_>>(), vec![0]);
            } else if cancelled {
                assert!(capture.events.len() >= 2, "must stop after entering P1, not at baseline");
            }
            for (kind, width, density, json) in &capture.events {
                assert!(*width > reference.strip_width(), "public {kind} used raw pre-repair width");
                let payload: serde_json::Value = serde_json::from_str(json).unwrap();
                let ext: ExtSPSolution = serde_json::from_value(payload["solution"].clone()).unwrap();
                assert_eq!(*width, ext.strip_width);
                assert_eq!(*density, ext.density);
                let prob = crate::util::frozen::import_solution(&instance, &ext);
                assert!(swarm_core::fabric_edge::inspect(&prob).unwrap().iter()
                    .filter(|(_, over)| over.item_id != 1)
                    .all(|(_, over)| over.worst() <= 0.051));
            }
            let published = if cancelled {
                capture.events.last().unwrap()
            } else {
                capture.events.iter().find(|e| e.0 == 4).unwrap()
            };
            let published_json: serde_json::Value = serde_json::from_str(&published.3).unwrap();
            let callback_ext: ExtSPSolution = serde_json::from_value(published_json["solution"].clone()).unwrap();
            assert_eq!(serde_json::to_value(callback_ext).unwrap()["layout"],
                serde_json::to_value(final_ext).unwrap()["layout"]);
        }
        swarm_free(handle);
    }

    #[cfg(feature = "structure-first")]
    #[test]
    fn issue4_edge_callbacks_and_final_share_canonical_snapshot() {
        issue4_run_edge_boundary(true, None, 0);
    }

    #[cfg(feature = "structure-first")]
    #[test]
    fn issue4_edge_without_callback_updates_report_from_getter_snapshot() {
        issue4_run_edge_boundary(false, None, 0);
    }

    #[cfg(feature = "structure-first")]
    #[test]
    fn issue4_edge_stop_at_baseline_keeps_canonical_snapshot() {
        issue4_run_edge_boundary(true, Some(0), 0);
    }

    #[cfg(feature = "structure-first")]
    #[test]
    fn issue4_edge_stop_in_p1_keeps_canonical_snapshot_without_final() {
        issue4_run_edge_boundary(true, Some(0), 1);
    }

    #[cfg(feature = "structure-first")]
    #[test]
    fn issue4_edge_stop_at_final_keeps_canonical_snapshot() {
        issue4_run_edge_boundary(true, Some(4), 0);
    }


    #[cfg(feature = "structure-first")]
    #[test]
    fn issue4_cache_orders_after_real_edge_repair_and_reuses_dirty_version() {
        let (handle, raw, widened_width) = issue4_edge_handle_with_height(30.0);
        let (instance, _) = unsafe { legacy_solution(handle) };
        let cache = Rc::new(RefCell::new(ExactSolutionCache::for_legacy_structure(
            &instance, Some(&raw), &[1], Some(0.05))));
        retain_exact_solution(&cache, &ReportType::ExplFeas, &instance, &raw, true);
        assert_eq!(cache.borrow().certified.as_ref().unwrap().solution.strip_width(), widened_width);
        assert_eq!(cache.borrow().processed_candidates, 1);
        report_cached_exact_solution(capture_callback, std::ptr::null_mut(), true, 0,
            &cache, &instance, DEFAULT_CALLBACK_OPTIONS);
        let payload = cache.borrow().certified.as_ref().unwrap().payload.as_ref().unwrap().as_ptr();
        for report in [ReportType::ExplImproving, ReportType::ExplFeas, ReportType::Final] {
            retain_exact_solution(&cache, &report, &instance, &raw, true);
            report_cached_exact_solution(capture_callback, std::ptr::null_mut(), true,
                callback_report_type(&report), &cache, &instance, DEFAULT_CALLBACK_OPTIONS);
        }
        assert_eq!(cache.borrow().processed_candidates, 1, "replay/duplicate Final must not repeat geometry work");
        assert_eq!(cache.borrow().certified.as_ref().unwrap().payload.as_ref().unwrap().as_ptr(), payload);

        // A clean candidate of width 43 is wider than RAW 40 but narrower than
        // canonical 43.4537. Independent vertical space avoids the fixed blocker.
        let mut clean = SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
        clean.change_strip_width(43.0).expect(swarm_core::dmsg!("strip width must be positive"));
        clean.place_item(legacy_hint_placement(&instance, &hint(0, false, 1.0, 16.0)).unwrap());
        let fixed = raw.layout_snapshot().placed_items().values().find(|p| p.item().idx() == 1).unwrap();
        clean.place_item(SPPlacement { item_idx: 1, d_transf: fixed.d_transf() });
        assert!(crate::structure::pipeline::solution_validation_error_with_frozen_ids(
            &instance, &clean.save(), &[1]).is_none());
        assert!(swarm_core::fabric_edge::inspect(&clean).unwrap().iter().all(|(_, o)| o.worst() <= 0.05));
        retain_exact_solution(&cache, &ReportType::CmprFeas, &instance, &clean.save(), true);
        assert_eq!(cache.borrow().certified.as_ref().unwrap().solution.strip_width(), 43.0);
        // Returning raw 40 may look like improvement, but its repair is wider.
        retain_exact_solution(&cache, &ReportType::Final, &instance, &raw, true);
        assert_eq!(cache.borrow().certified.as_ref().unwrap().solution.strip_width(), 43.0);
        assert!(cache.borrow().certified.as_ref().unwrap().payload.is_none());
        swarm_free(handle);
    }

    #[cfg(feature = "structure-first")]
    #[test]
    fn issue4_cache_rejects_exact_frozen_drift_and_remembers_rejection() {
        let (handle, raw, _) = issue4_edge_handle();
        let (instance, _) = unsafe { legacy_solution(handle) };
        let cache = Rc::new(RefCell::new(ExactSolutionCache::for_legacy_structure(
            &instance, Some(&raw), &[1], None)));
        retain_exact_solution(&cache, &ReportType::ExplFeas, &instance, &raw, true);
        let mut moved = SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
        crate::util::frozen::restore(&mut moved, &raw);
        let key = moved.layout().placed_items().iter().find_map(|(k, p)| (p.item().idx() == 1).then_some(k)).unwrap();
        moved.remove_item(key);
        // Drift the frozen piece to a more compact pose inside the strip.
        // (A pose entirely left of the strip trips jagua-rs's quadtree debug check.)
        moved.place_item(legacy_hint_placement(&instance, &hint(1, true, 0.0, 0.0)).unwrap());
        let drifted = moved.save();
        assert!(crate::structure::pipeline::solution_validation_error_with_frozen_ids(
            &instance, &drifted, &[1]).is_none(), "exact certificate alone does not protect reference poses");
        assert!(crate::optimizer::polish::prefers_compact_solution(&drifted, &raw),
            "drift would otherwise win the equal-width comparison");
        retain_exact_solution(&cache, &ReportType::Final, &instance, &drifted, true);
        let kept = cache.borrow().certified.as_ref().unwrap().solution.clone();
        assert!(crate::util::assertions::frozen_poses_preserved(&raw, &kept, &[false, true]));
        let work = cache.borrow().processed_candidates;
        retain_exact_solution(&cache, &ReportType::ExplImproving, &instance, &drifted, true);
        retain_exact_solution(&cache, &ReportType::Final, &instance, &drifted, true);
        assert_eq!(cache.borrow().processed_candidates, work, "rejected version is cached too");
        swarm_free(handle);
    }


    #[cfg(feature = "structure-first")]
    fn issue4_error_run_boundary(with_callback: bool, panic_after_baseline: bool) {
        struct Capture { events: Vec<(i32, String)> }
        extern "C" fn callback(kind: i32, _: f32, _: f32, json: *const c_char,
            user_data: *mut std::ffi::c_void) {
            let capture = unsafe { &mut *user_data.cast::<Capture>() };
            if !json.is_null() {
                capture.events.push((kind, unsafe { CStr::from_ptr(json) }.to_str().unwrap().to_owned()));
            }
        }
        let (handle, raw, _) = issue4_edge_handle();
        let (instance, _) = unsafe { legacy_solution(handle) };
        let frozen_ids = if panic_after_baseline { vec![1] } else { vec![] };
        let expected = repair_fabric_edge(&instance, raw.clone(), "ISSUE4:ERROR",
            &frozen_ids, Some(0.05));
        assert!(expected.strip_width() > raw.strip_width() + 0.01,
            "error fixture must canonicalize real sharp-tip geometry");
        assert!(crate::structure::pipeline::solution_validation_error_with_frozen_ids(
            &instance, &raw, &frozen_ids).is_none());
        assert!(crate::structure::pipeline::solution_validation_error_with_frozen_ids(
            &instance, &expected, &frozen_ids).is_none());
        {
            let mut engine = unsafe { &*handle }.lock_engine();
            let state = engine.legacy_mut();
            state.frozen_item_ids = frozen_ids.clone();
            state.has_input_seed = false;
            state.has_column_draft = false;
            state.structure_config.p1_time_limit = Duration::ZERO;
            if panic_after_baseline {
                // A retry can carry an older failed report while phase remains Init.
                state.structure_report = Some(crate::structure::pipeline::StructureRunReport::cancelled_baseline(
                    &instance, &raw));
            }
        }
        let mut capture = Capture { events: vec![] };
        if with_callback {
            swarm_set_callback(handle, Some(callback), (&mut capture as *mut Capture).cast(), 1);
        }
        if panic_after_baseline {
            LEGACY_STRUCTURE_PANIC_AFTER_BASELINE.with(|flag| flag.set(true));
        }
        let result = swarm_gp_phase(handle);
        assert_eq!(result, if panic_after_baseline { 1 } else { -1 }, "error must remain an error");
        assert_eq!(swarm_get_last_error_code(), if panic_after_baseline { 9001 } else { 3005 });
        let error_ptr = swarm_get_last_error_message();
        let message = unsafe { CStr::from_ptr(error_ptr) }.to_str().unwrap().to_owned();
        crate::ffi::swarm_free_string(error_ptr);
        assert!(message.contains(if panic_after_baseline { "panic" } else { "no complete exact-feasible P1 candidate" }));
        let (_, retained) = unsafe { legacy_solution(handle) };
        assert_eq!(retained.strip_width(), expected.strip_width(),
            "error must retain latest canonical certificate, not raw entry");
        let exported = assert_parseable_certified_export(handle, &instance);
        let expected_ext = jagua_rs::probs::spp::io::export(&expected, *crate::EPOCH);
        assert_eq!(serde_json::to_value(&exported).unwrap()["layout"],
            serde_json::to_value(&expected_ext).unwrap()["layout"]);
        assert!(crate::util::assertions::frozen_poses_preserved(&raw, &retained,
            &[false, panic_after_baseline]));
        let mut prob = SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
        crate::util::frozen::restore(&mut prob, &retained);
        assert!(swarm_core::fabric_edge::inspect(&prob).unwrap().iter()
            .filter(|(_, o)| !frozen_ids.contains(&o.item_id)).all(|(_, o)| o.worst() <= 0.051));
        {
            let engine = unsafe { &*handle }.lock_engine();
            let HandleEngine::Legacy(state) = &*engine else { panic!("{}", swarm_core::dmsg!("Legacy handle")) };
            assert!(state.phase == Phase::Init, "failure must preserve phase semantics");
            assert_eq!(state.solutions.len(), 1);
            assert!(same_native_snapshot(&state.prob.save(), &retained));
            assert!(same_native_snapshot(&state.solutions[0], &retained));
            if panic_after_baseline {
                assert!(state.structure_report.is_none(), "panic must not expose stale earlier failure report");
            } else {
                let report = state.structure_report.as_ref().unwrap();
                assert!(report.p1_winner.is_none());
                assert!(report.p1_deadline_exhausted);
                assert!(!report.cancelled);
                assert_eq!(report.final_width, retained.strip_width());
                assert_eq!(report.final_density, retained.density());
            }
        }
        if with_callback {
            assert!(!capture.events.is_empty());
            assert!(capture.events.iter().all(|event| event.0 != 4), "errors must not gain a new Final policy");
            let value: serde_json::Value = serde_json::from_str(&capture.events.last().unwrap().1).unwrap();
            let published: ExtSPSolution = serde_json::from_value(value["solution"].clone()).unwrap();
            assert_eq!(serde_json::to_value(published).unwrap()["layout"], serde_json::to_value(exported).unwrap()["layout"]);
        }
        swarm_free(handle);
    }

    #[cfg(feature = "structure-first")]
    #[test]
    fn issue4_error_failed_on_callback_keeps_canonical_certificate() {
        issue4_error_run_boundary(true, false);
    }
    #[cfg(feature = "structure-first")]
    #[test]
    fn issue4_error_failed_without_callback_keeps_canonical_certificate() {
        issue4_error_run_boundary(false, false);
    }
    #[cfg(feature = "structure-first")]
    #[test]
    fn issue4_error_panic_on_callback_keeps_frozen_certificate() {
        issue4_error_run_boundary(true, true);
    }
    #[cfg(feature = "structure-first")]
    #[test]
    fn issue4_error_panic_without_callback_keeps_frozen_certificate() {
        issue4_error_run_boundary(false, true);
    }

    #[cfg(feature = "structure-first")]
    fn issue3_run_structure_compact_tie(with_callback: bool) {
        let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
            "name": "issue3-frozen-barrier", "strip_height": 20.0,
            "items": (0..3).map(|id| serde_json::json!({
                "id": id, "demand": 1, "allowed_orientations": [0.0],
                "shape": {"type": "rectangle", "data": {
                    "x_min": 0.0, "y_min": 0.0, "width": 10.0,
                    "height": if id == 2 { 4.0 } else { 18.0 }
                }}
            })).collect::<Vec<_>>()
        }))
        .unwrap();
        let mut config = fast_config();
        config.init_settle_every = 0;
        config.init_compaction_ms = 0;
        config.structure.enabled = true;
        config.structure.p1_time_limit = Duration::from_secs(1);
        config.structure.p2_time_limit = Duration::ZERO;
        config.structure.p3_time_limit = Some(Duration::ZERO);
        config.structure.p4_cycles = 0;
        config.polish_cfg.time_limit = Duration::from_millis(200);
        config.polish_cfg.lp.enabled = false;
        config.polish_cfg.nfp_left.enabled = false;
        config.polish_cfg.reclaim.enabled = false;
        let hints = vec![
            hint(0, true, 90.0, 1.0),
            hint(1, false, 100.001175, 1.0),
            hint(2, false, 50.0, 3.0),
        ];
        let direct = build_handle(
            ext.clone(),
            None,
            config,
            DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(),
            hints.clone(),
        )
        .unwrap();
        assert_eq!(swarm_refine_phase(direct), 0);
        let (_, direct_solution) = unsafe { legacy_solution(direct) };
        swarm_free(direct);
        let interior_x = |s: &SPSolution| {
            s.layout_snapshot()
                .placed_items()
                .values()
                .find(|p| p.item().idx() == 2)
                .unwrap()
                .shape()
                .bbox()
                .x_min
        };
        assert!(
            interior_x(&direct_solution) < 1.0,
            "fixture must allow direct Polish to left pack"
        );
        let handle = build_handle(
            ext,
            None,
            config,
            DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(),
            hints,
        )
        .unwrap();
        let (instance, initial) = unsafe { legacy_solution(handle) };
        let mut capture = CallbackCapture::default();
        if with_callback {
            swarm_set_callback(
                handle,
                Some(capture_callback),
                (&mut capture as *mut CallbackCapture).cast(),
                1,
            );
        }
        assert_eq!(swarm_gp_phase(handle), 0);
        let (_, solution) = unsafe { legacy_solution(handle) };
        assert_eq!(solution.strip_width(), initial.strip_width());
        assert!(
            interior_x(&solution) < 1.0,
            "Structure final must retain equal-width P5 progress: {}",
            interior_x(&solution)
        );
        let frozen = |s: &SPSolution| {
            s.layout_snapshot()
                .placed_items()
                .values()
                .find(|p| p.item().idx() == 0)
                .unwrap()
                .d_transf()
        };
        assert_eq!(frozen(&solution), frozen(&initial));
        assert!(
            crate::structure::pipeline::solution_validation_error_with_frozen_ids(
                &instance,
                &solution,
                &[0]
            )
            .is_none()
        );
        let exported = assert_parseable_certified_export(handle, &instance);
        if with_callback {
            assert!(capture.final_count > 0);
            let value: serde_json::Value =
                serde_json::from_str(capture.final_json.last().unwrap()).unwrap();
            let callback_solution: ExtSPSolution =
                serde_json::from_value(value["solution"].clone()).unwrap();
            assert_eq!(callback_solution.strip_width, exported.strip_width);
            assert_eq!(
                serde_json::to_value(&callback_solution.layout).unwrap(),
                serde_json::to_value(&exported.layout).unwrap()
            );
        }
        let engine = unsafe { &*handle }.lock_engine();
        let HandleEngine::Legacy(state) = &*engine else {
            panic!("{}", swarm_core::dmsg!("legacy handle"))
        };
        assert_eq!(
            state.structure_report.as_ref().unwrap().best_stage,
            crate::structure::pipeline::StructureStage::P5Finish
        );
        drop(engine);
        swarm_free(handle);
    }

    #[cfg(feature = "structure-first")]
    #[test]
    fn issue3_structure_compact_tie_without_callback() {
        issue3_run_structure_compact_tie(false);
    }

    #[cfg(feature = "structure-first")]
    #[test]
    fn issue3_structure_compact_tie_with_callback() {
        issue3_run_structure_compact_tie(true);
    }

    #[cfg(feature = "structure-first")]
    #[test]
    fn discarded_movable_seed_runs_structure_p1_instead_of_warm_start_shortcut() {
        clear_last_error();
        let hints = [hint(0, false, 5.0, 5.0), hint(1, false, 5.0, 5.0)];
        let mut config = fast_config();
        config.structure.enabled = true;
        config.structure.motif.enabled = false;
        config.structure.construction_starts = 1;
        config.structure.p1_time_limit = Duration::from_millis(200);
        config.structure.p2_time_limit = Duration::ZERO;
        config.structure.p3_time_limit = Some(Duration::ZERO);
        config.structure.p4_cycles = 0;
        config.polish_cfg.time_limit = Duration::ZERO;
        let handle = build_handle(
            rectangle_instance(),
            None,
            config,
            DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(),
            hints.to_vec(),
        )
        .expect(swarm_core::dmsg!("invalid movable seed should rebuild to a fresh exact baseline"));

        assert_eq!(swarm_gp_phase(handle), 0);
        let handle_ref = unsafe { &*handle };
        let engine = handle_ref.lock_engine();
        let report = match &*engine {
            HandleEngine::Legacy(state) => state.structure_report.as_ref().unwrap(),
            HandleEngine::Grouped(_) => panic!("{}", swarm_core::dmsg!("expected a Legacy handle")),
        };
        assert_ne!(
            report.p1_winner.as_ref().unwrap().source,
            crate::structure::pipeline::P1CandidateSource::WarmStart
        );
        assert_eq!(report.construction_starts_requested, 1);
        drop(engine);
        swarm_free(handle);
    }

    #[test]
    fn grouped_phase_and_non_structure_export_reject_uncertified_state() {
        let json = serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items","group_workers":1,"quantum_ms":1,"locked_prefix_gap_mm":0.0},
            "instance":{"name":"grouped-boundary-certification","strip_height":10.0,"items":[
                {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                    "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
                },"placed_hint":{"item_id":0,"is_freeze":0,"tr_x":0.01,"tr_y":1.0,"rot_angle_in_rad":0.0,"stripe_interval_y":0.0,"stripe_interval_x":0.0}},
                {"id":1,"demand":1,"allowed_orientations":[0.0],"shape":{
                    "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
                },"placed_hint":{"item_id":1,"is_freeze":0,"tr_x":5.01,"tr_y":1.0,"rot_angle_in_rad":0.0,"stripe_interval_y":0.0,"stripe_interval_x":0.0}}
            ]},
            "regions":[],
            "place_groups":[]
        })
        .to_string();
        let input = grouped_input::parse_and_validate(&json).unwrap();
        let mut config = fast_config();
        config.expl_cfg.time_limit = Duration::ZERO;
        let handle = build_grouped_handle(input, config, DEFAULT_CALLBACK_OPTIONS).unwrap();
        {
            let handle_ref = unsafe { &*handle };
            let mut engine = handle_ref.lock_engine();
            let grouped = match &mut *engine {
                HandleEngine::Grouped(grouped) => grouped,
                HandleEngine::Legacy(_) => panic!("{}", swarm_core::dmsg!("expected a grouped handle")),
            };
            assert!(
                grouped
                    .solver
                    .corrupt_first_lane_public_transform_for_test()
            );
            assert!(report_grouped_solution(grouped, 4).is_err());
        }

        clear_last_error();
        assert_eq!(swarm_alg_phase(handle), -1);
        assert_eq!(swarm_get_last_error_code(), 3003);
        let json_ptr = swarm_get_solution_json(handle);
        assert!(json_ptr.is_null());
        assert_eq!(swarm_get_last_error_code(), 3005);
        swarm_free(handle);
    }

    fn issue10_warm_rectangles() -> ExtSPInstance {
        serde_json::from_value(serde_json::json!({
            "name":"issue10-warm-local", "strip_height":100.0,
            "items":(0..4).map(|id| serde_json::json!({
                "id":id,"demand":1,"allowed_orientations":[0.0],
                "shape":{"type":"simple_polygon","data":[[0,0],[40,0],[40,40],[0,40]]}
            })).collect::<Vec<_>>()
        })).unwrap()
    }

    fn issue10_check_warm_seed(seed: usize) {
        let hints = vec![hint(0,false,0.0,0.0),hint(1,false,39.95,0.0),
            hint(2,false,0.0,40.0),hint(3,false,40.0,40.0)];
        let mut config = fast_config();
        config.rng_seed = Some(seed);
        let handle = build_handle(issue10_warm_rectangles(), None, config,
            DEFAULT_CALLBACK_OPTIONS, crate::ffi::InitialPlacementSettings::default(),
            hints.clone()).expect(swarm_core::dmsg!("micro-overlap hints must produce an exact baseline"));
        let (instance, solution) = unsafe { legacy_solution(handle) };
        assert!(crate::structure::pipeline::solution_validation_error(&instance,&solution).is_none());
        let ptr = swarm_get_solution_json(handle);
        assert!(!ptr.is_null());
        let exported: ExtSPSolution = serde_json::from_str(unsafe { CStr::from_ptr(ptr) }.to_str().unwrap()).unwrap();
        crate::ffi::swarm_free_string(ptr);
        swarm_free(handle);
        assert_eq!(exported.layout.placed_items.len(),4);
        for placed in exported.layout.placed_items {
            let h = &hints[placed.item_id as usize];
            let (x,y) = placed.transformation.translation;
            assert!((x-h.tr_x).abs() <= 0.25 && (y-h.tr_y).abs() <= 0.25,
                "seed {seed}: item {} moved beyond local bound: ({x},{y}) from ({},{})",
                h.item_id,h.tr_x,h.tr_y);
            assert_eq!(placed.transformation.rotation,0.0);
        }
    }

    /// Two 40 x 30 rectangles with a 0.4 mm piece overlap, hinted so that the
    /// shrunk shapes are disjoint but the original outlines overlap by 0.3995 mm
    /// (beyond the allowance). Returns the exported translations, or None when
    /// Init rejected the input.
    fn excess_original_overlap_init(
        policy: swarm_core::config::OriginalOverlapPolicy,
    ) -> Option<Vec<(f32, f32)>> {
        let mut ext = issue10_warm_rectangles();
        ext.items.truncate(2);
        for item in &mut ext.items {
            item.base.shape = serde_json::from_value(serde_json::json!(
                {"type":"simple_polygon","data":[[0,0],[40,0],[40,30],[0,30]]})).unwrap();
        }
        let mut config = fast_config();
        config.min_item_separation = Some(-0.4);
        config.original_overlap_policy = policy;
        let hints = vec![hint(0,false,10.0,10.0),hint(1,false,49.6005,10.0)];
        let handle = build_handle(ext, None, config, DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(), hints)?;
        let ptr = swarm_get_solution_json(handle);
        assert!(!ptr.is_null());
        let exported: ExtSPSolution = serde_json::from_str(unsafe { CStr::from_ptr(ptr) }.to_str().unwrap()).unwrap();
        crate::ffi::swarm_free_string(ptr);
        swarm_free(handle);
        let mut poses: Vec<_> = exported.layout.placed_items.iter()
            .map(|p| (p.item_id, p.transformation.translation)).collect();
        poses.sort_by_key(|(id, _)| *id);
        Some(poses.into_iter().map(|(_, t)| t).collect())
    }

    #[test]
    fn feasible_original_overlap_policy_keeps_excess_overlap_hints() {
        use swarm_core::config::OriginalOverlapPolicy;
        let poses = excess_original_overlap_init(OriginalOverlapPolicy::Feasible)
            .expect(swarm_core::dmsg!("accepted layout must initialize"));
        let expected = [(10.0f32, 10.0f32), (49.6005, 10.0)];
        for (got, want) in poses.iter().zip(expected) {
            assert!((got.0 - want.0).abs() < 1e-3 && (got.1 - want.1).abs() < 1e-3,
                "feasible policy must keep the hinted pose: {got:?} vs {want:?}");
        }
        // The default policy does not accept the same input unchanged.
        let rejected = excess_original_overlap_init(OriginalOverlapPolicy::Infeasible);
        assert!(rejected.is_none_or(|poses| poses.iter().zip(expected).any(|(got, want)|
            (got.0 - want.0).abs() >= 1e-4 || (got.1 - want.1).abs() >= 1e-4)),
            "infeasible policy must repair or reject the excess overlap");
    }

    #[test]
    fn hints_flush_with_the_strip_edge_are_nudged_not_discarded() {
        // Piece separation inflates the collision shapes, so a hint whose
        // original outline touches the top edge crosses it by a fraction of a
        // millimetre. The warm start must survive with a small inward shift
        // instead of being thrown away and rebuilt by LBF (which would put
        // these pieces at the bottom).
        let mut ext = issue10_warm_rectangles();
        ext.items.truncate(2);
        let hints = vec![hint(0,false,10.0,60.0),hint(1,false,60.0,60.0)];
        let mut config = fast_config();
        config.min_item_separation = Some(1.0);
        let handle = build_handle(ext, None, config, DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(), hints.clone())
            .expect(swarm_core::dmsg!("edge-flush hints must produce a baseline"));
        let ptr = swarm_get_solution_json(handle);
        assert!(!ptr.is_null());
        let exported: ExtSPSolution = serde_json::from_str(unsafe { CStr::from_ptr(ptr) }.to_str().unwrap()).unwrap();
        crate::ffi::swarm_free_string(ptr);
        swarm_free(handle);
        for placed in exported.layout.placed_items {
            let h = &hints[placed.item_id as usize];
            let (x,y) = placed.transformation.translation;
            assert!((x-h.tr_x).abs() <= SEED_EDGE_NUDGE_MAX_MM && (y-h.tr_y).abs() <= SEED_EDGE_NUDGE_MAX_MM,
                "item {} was rebuilt instead of nudged: ({x},{y}) from ({},{})", h.item_id, h.tr_x, h.tr_y);
            assert!(y <= h.tr_y, "the nudge must point inward");
        }
    }

    #[test]
    fn issue10_warm_micro_overlap_seed1_preserves_pose() { issue10_check_warm_seed(1); }

    #[test]
    fn issue10_warm_micro_overlap_seed3_preserves_pose() { issue10_check_warm_seed(3); }

    fn issue10_local_separator(repeated: bool, deep: bool, overlap: Option<f32>) -> Separator {
        let mut ext = issue10_warm_rectangles();
        if repeated {
            ext.items.truncate(1);
            ext.items[0].demand = if deep { 2 } else { 4 };
        }
        let mut config = fast_config();
        config.min_item_separation = overlap;
        config.expl_cfg.separator_config.n_workers = 1;
        let importer = swarm_core::util::io::new_importer(config.cde_config, config.poly_simpl_tolerance, config.min_item_separation, config.narrow_concavity_cutoff_ratio);
        let instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext).unwrap();
        let positions = if deep { vec![(10.,10.),(20.,10.)] }
            else { vec![(0.,0.),(39.95,0.),(0.,40.),(40.,40.)] };
        let placements = positions.into_iter().enumerate().map(|(index,(x,y))| {
            LegacySeedPlacement { placement:legacy_hint_placement(&instance,
                &hint(if repeated {0} else {index as i32},false,x,y)).unwrap(),frozen:false }
        }).collect::<Vec<_>>();
        let prob = legacy_seed_problem_raw(&instance,&placements,0.0,true).unwrap();
        Separator::new(instance,prob,Xoshiro256PlusPlus::seed_from_u64(3),config.expl_cfg.separator_config)
    }

    fn issue10_assert_local_state_unchanged(sep: &Separator, before: &SPProblem,
        ct: &swarm_core::quantify::tracker::CollisionTracker) {
        assert_eq!(sep.prob.strip_width().to_bits(),before.strip_width().to_bits());
        assert_eq!(sep.prob.layout().placed_items().len(),before.layout().placed_items().len());
        assert_eq!(sep.ct.get_total_loss().to_bits(),ct.get_total_loss().to_bits());
        assert_eq!(sep.ct.get_total_weighted_loss().to_bits(),ct.get_total_weighted_loss().to_bits());
        for (key, placed) in before.layout().placed_items() {
            assert_eq!(sep.prob.layout().placed_items()[key].d_transf(),placed.d_transf());
            assert_eq!(sep.prob.layout().placed_items()[key].item().idx(),placed.item().idx());
            assert_eq!(sep.ct.pk_idx_map[key],ct.pk_idx_map[key]);
            assert_eq!(sep.ct.get_container_loss(key).to_bits(),ct.get_container_loss(key).to_bits());
            assert_eq!(sep.ct.get_container_weight(key).to_bits(),ct.get_container_weight(key).to_bits());
            for other in before.layout().placed_items().keys() {
                assert_eq!(sep.ct.get_pair_loss(key,other).to_bits(),ct.get_pair_loss(key,other).to_bits());
                assert_eq!(sep.ct.get_pair_weight(key,other).to_bits(),ct.get_pair_weight(key,other).to_bits());
            }
        }
        assert!(swarm_core::util::assertions::tracker_matches_layout(&sep.ct,&sep.prob.layout()));
    }

    #[test]
    fn issue10_local_deep_overlap_restores_keys_tracker_and_rng() {
        let mut sep = issue10_local_separator(true,true,None);
        for entry in sep.ct.container_collisions_mut() { entry.weight = 3.25; }
        let before = sep.prob.clone();
        let ct = sep.ct.clone();
        let mut expected_rng = sep.rng.clone();
        let mut worker_rngs = sep.workers.iter().map(|worker| worker.rng.clone()).collect::<Vec<_>>();
        assert!(!legacy_seed_local_repair(&mut sep,&BasicTerminator::new()).unwrap());
        issue10_assert_local_state_unchanged(&sep,&before,&ct);
        for _ in 0..4 {
            assert_eq!(rand::RngExt::random::<u64>(&mut sep.rng),rand::RngExt::random::<u64>(&mut expected_rng));
            for (worker,expected) in sep.workers.iter_mut().zip(&mut worker_rngs) {
                assert_eq!(rand::RngExt::random::<u64>(&mut worker.rng),rand::RngExt::random::<u64>(expected));
            }
        }
    }

    #[test]
    fn hint_collapse_only_replaces_seed_once_item_kind_is_saturated() {
        let mut ext = issue10_warm_rectangles();
        ext.items.truncate(2);
        ext.items[0].demand = 2;
        ext.items[1].demand = 1;
        let config = fast_config();
        let importer = swarm_core::util::io::new_importer(config.cde_config, config.poly_simpl_tolerance, config.min_item_separation, config.narrow_concavity_cutoff_ratio);
        let instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext).unwrap();
        let seed = |item_id: i32, x: f32| LegacySeedPlacement {
            placement: legacy_hint_placement(&instance, &hint(item_id, false, x, 0.0)).unwrap(),
            frozen: false,
        };
        let x_of = |s: &LegacySeedPlacement| s.placement.d_transf.translation().0;
        let translations = |seeds: &[LegacySeedPlacement], item_id: usize| seeds.iter()
            .filter(|s| s.placement.item_idx == item_id).map(x_of).collect::<Vec<_>>();

        // Demand 2: a second hint of the same kind keeps the first seed.
        let mut seeds = vec![seed(0, 0.0)];
        collapse_saturated_seed_kind(&instance, &mut seeds, 0);
        seeds.push(seed(0, 50.0));
        assert_eq!(translations(&seeds, 0), vec![x_of(&seed(0, 0.0)), x_of(&seed(0, 50.0))]);
        // A third hint of kind 0 is over demand: the oldest seed of that kind gives way.
        collapse_saturated_seed_kind(&instance, &mut seeds, 0);
        seeds.push(seed(0, 100.0));
        assert_eq!(translations(&seeds, 0), vec![x_of(&seed(0, 50.0)), x_of(&seed(0, 100.0))]);
        // Demand 1 keeps the historical override-one-placement behavior.
        seeds.push(seed(1, 0.0));
        collapse_saturated_seed_kind(&instance, &mut seeds, 1);
        seeds.push(seed(1, 70.0));
        assert_eq!(translations(&seeds, 1), vec![x_of(&seed(1, 70.0))]);
        assert_eq!(translations(&seeds, 0), vec![x_of(&seed(0, 50.0)), x_of(&seed(0, 100.0))]);
    }

    #[test]
    fn issue10_local_micro_overlap_keeps_repeated_item_instances_distinct() {
        let mut sep = issue10_local_separator(true,false,None);
        let before = sep.prob.clone();
        assert!(legacy_seed_local_repair_with_elapsed(
            &mut sep,&BasicTerminator::new(),|| Duration::ZERO).unwrap());
        assert_eq!(sep.prob.strip_width(),before.strip_width());
        assert_eq!(sep.prob.n_placed_items(),4);
        for anchor in before.layout().placed_items().values() {
            let (ax,ay) = anchor.d_transf().translation();
            assert_eq!(sep.prob.layout().placed_items().values().filter(|placed| {
                let (x,y) = placed.d_transf().translation();
                placed.item().idx() == 0 && placed.d_transf().rotation() == anchor.d_transf().rotation()
                    && (x-ax).abs() <= 0.25 && (y-ay).abs() <= 0.25
            }).count(),1,"each repeated instance must retain its own hint anchor");
        }
        assert!(crate::structure::pipeline::solution_validation_error(&sep.instance,&sep.prob.save()).is_none());
    }

    #[test]
    fn issue10_local_micro_overlap_passes_original_outline_allowance() {
        let mut sep = issue10_local_separator(false,false,Some(-0.01));
        assert!(swarm_core::original_overlap::OriginalOverlapGuard::is_active(&sep.instance));
        assert!(legacy_seed_local_repair_with_elapsed(
            &mut sep,&BasicTerminator::new(),|| Duration::ZERO).unwrap());
        assert!(swarm_core::original_overlap::validate(&sep.instance,&sep.prob.save(),&[]).is_ok());
        assert!(crate::structure::pipeline::solution_validation_error(&sep.instance,&sep.prob.save()).is_none());
    }

    #[test]
    fn issue10_local_repair_does_not_move_frozen_hints() {
        let mut sep = issue10_local_separator(false,false,None);
        sep.freeze_items(&[0]);
        let before = sep.prob.clone();
        let ct = sep.ct.clone();
        assert!(!legacy_seed_local_repair(&mut sep,&BasicTerminator::new()).unwrap());
        issue10_assert_local_state_unchanged(&sep,&before,&ct);
    }

    #[test]
    fn issue10_local_expired_budget_preserves_entry() {
        let mut sep = issue10_local_separator(false,false,None);
        let before = sep.prob.clone();
        let ct = sep.ct.clone();
        let term = BasicTerminator { timeout:Some(Instant::now()-Duration::from_secs(1)) };
        assert!(!legacy_seed_local_repair(&mut sep,&term).unwrap());
        issue10_assert_local_state_unchanged(&sep,&before,&ct);
    }

    #[test]
    fn issue10_local_elapsed_deadline_mid_trial_restores_entry_and_rng() {
        let mut sep = issue10_local_separator(true,false,None);
        let before = sep.prob.clone();
        let ct = sep.ct.clone();
        let mut expected_rng = sep.rng.clone();
        let mut worker_rngs = sep.workers.iter().map(|worker| worker.rng.clone()).collect::<Vec<_>>();
        let mut polls = 0usize;
        let elapsed = || {
            polls += 1;
            // The first placement has retained its best strict improvement by
            // this point; expiry must restore the entry, not just a trial.
            if polls >= 64 { LOCAL_REPAIR_BUDGET } else { Duration::ZERO }
        };
        assert!(!legacy_seed_local_repair_with_elapsed(
            &mut sep,&BasicTerminator::new(),elapsed).unwrap());
        assert_eq!(polls,64,"the supplied local deadline must stop an in-progress trial loop");
        issue10_assert_local_state_unchanged(&sep,&before,&ct);
        for _ in 0..4 {
            assert_eq!(rand::RngExt::random::<u64>(&mut sep.rng),rand::RngExt::random::<u64>(&mut expected_rng));
            for (worker,expected) in sep.workers.iter_mut().zip(&mut worker_rngs) {
                assert_eq!(rand::RngExt::random::<u64>(&mut worker.rng),rand::RngExt::random::<u64>(expected));
            }
        }
    }

    #[test]
    #[ignore = "broadcast Init cancellation; run issue10_local_cancel --include-ignored --test-threads=1"]
    fn issue10_local_cancel_during_trials_restores_entry_and_propagates() {
        struct CancelDuringTrials(std::sync::atomic::AtomicUsize);
        impl Terminator for CancelDuringTrials {
            fn kill(&self) -> bool {
                if self.0.fetch_add(1,Ordering::SeqCst) >= 4 { swarm_cancel_init(); true }
                else { false }
            }
            fn new_timeout(&mut self,_:Duration) {}
            fn timeout_at(&self) -> Option<Instant> { None }
        }
        let mut sep = issue10_local_separator(false,false,None);
        let before = sep.prob.clone();
        let ct = sep.ct.clone();
        let _scope = swarm_core::config::runtime::begin_init();
        let term = CancelDuringTrials(std::sync::atomic::AtomicUsize::new(0));
        assert!(legacy_seed_local_repair(&mut sep,&term).is_err());
        assert!(init_cancelled());
        issue10_assert_local_state_unchanged(&sep,&before,&ct);
    }

    #[test]
    fn empty_seed_keeps_the_fresh_lbf_base_width_unchanged() {
        let instance = internal_instance();
        let base_width = SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container")).strip_width();
        let seed_problem = legacy_seed_problem(&instance, &[], 0.0).unwrap();

        assert_eq!(seed_problem.strip_width(), base_width);
        assert_eq!(seed_problem.n_placed_items(), 0);
    }

    #[test]
    #[ignore = "broadcast Init cancellation; run issue2_ --include-ignored --test-threads=1"]
    fn issue2_mid_cancel_warm_overlap_returns_promptly_with_code_2002() {
        let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
            "name": "issue2-warm-overlap-800", "strip_height": 100.0,
            "items": (0..800).map(|id| serde_json::json!({
                "id": id, "demand": 1, "allowed_orientations": [0.0],
                "shape": {"type": "rectangle", "data": {
                    "x_min": 0.0, "y_min": 0.0, "width": 10.0, "height": 10.0
                }}
            })).collect::<Vec<_>>()
        })).unwrap();
        let hints = (0..800).map(|id| hint(id, false, 1.0, 1.0)).collect::<Vec<_>>();
        let json = std::ffi::CString::new(serde_json::to_string(&ext).unwrap()).unwrap();
        let config = DEFAULT_C_SWARM_CONFIG_FOR_TEST.with(|c| *c);
        let cancel = std::thread::spawn(|| {
            std::thread::sleep(Duration::from_secs(1));
            let at = std::time::Instant::now();
            swarm_cancel_init();
            at
        });
        let handle = swarm_init_from_json(json.as_ptr(), &config, hints.as_ptr(), hints.len() as i32);
        let finished = std::time::Instant::now();
        let requested = cancel.join().unwrap();
        let latency = finished.saturating_duration_since(requested);
        if !handle.is_null() { swarm_free(handle); }
        assert!(handle.is_null(), "the dense warm overlap must cancel during Init");
        assert_eq!(swarm_get_last_error_code(), 2002);
        println!("issue2 warm overlap cancel latency: {:.3}s", latency.as_secs_f64());
        assert!(latency < Duration::from_secs(2), "warm separation ignored Init stop for {latency:?}");
    }
    #[test]
    #[ignore = "broadcast Init cancellation; run issue2_ --include-ignored --test-threads=1"]
    fn issue2_cancelled_warm_separation_rejects_before_work() {
        let instance = internal_instance();
        let placements = [hint(0, false, 10.0, 5.0), hint(1, false, 30.0, 5.0)]
            .iter().map(|h| LegacySeedPlacement {
                placement: legacy_hint_placement(&instance, h).unwrap(), frozen: false,
            }).collect::<Vec<_>>();
        let _scope = swarm_core::config::runtime::begin_init();
        swarm_cancel_init();
        let result = legacy_seed_problem_separated(
            &instance, &placements, 60.0, fast_config().expl_cfg.separator_config,
            Xoshiro256PlusPlus::seed_from_u64(7), Duration::from_millis(300),
        );
        assert!(result.is_err(), "cancelled warm separation must not return a usable seed");
        assert!(init_cancelled());
        assert_eq!(swarm_get_last_error_code(), 2002);
    }

    #[test]
    #[ignore = "broadcast Init cancellation; run issue2_ --include-ignored --test-threads=1"]
    fn issue2_cancelled_compaction_preserves_exact_incumbent() {
        let instance = internal_instance();
        let placements = [hint(0, true, 10.0, 5.0), hint(1, false, 30.0, 5.0)]
            .iter().map(|h| LegacySeedPlacement {
                placement: legacy_hint_placement(&instance, h).unwrap(), frozen: h.is_freeze != 0,
            }).collect::<Vec<_>>();
        let mut prob = legacy_seed_problem(&instance, &placements, 60.0).unwrap();
        let before = prob.save();
        let mut config = fast_config();
        config.init_compaction_ms = 300;
        let _scope = swarm_core::config::runtime::begin_init();
        swarm_cancel_init();
        compact_initial_layout(&instance, &mut prob, &[0], &config);
        assert_eq!(prob.strip_width(), before.strip_width(), "cancelled compaction must retain width");
        for (key, item) in before.layout_snapshot().placed_items() {
            assert_eq!(prob.layout().placed_items()[key].d_transf(), item.d_transf());
        }
        drop(_scope); // Certification must run outside the cancelled Init request.
        assert!(crate::structure::pipeline::solution_validation_error_with_frozen_ids(
            &instance, &prob.save(), &[0]).is_none());
    }

    #[test]
    #[ignore = "broadcast Init cancellation; run issue2_ --include-ignored --test-threads=1"]
    fn issue2_cancelled_init_can_restart_from_json_and_file() {
        {
            let _scope = swarm_core::config::runtime::begin_init();
            swarm_cancel_init();
            assert!(build_handle(rectangle_instance(), None, fast_config(),
                DEFAULT_CALLBACK_OPTIONS, crate::ffi::InitialPlacementSettings::default(),
                vec![]).is_none());
            assert!(init_cancelled()); // The public Init boundary normalizes cancellation.
            assert_eq!(swarm_get_last_error_code(), 2002);
        }
        let json = serde_json::to_string(&rectangle_instance()).unwrap();
        let c_json = std::ffi::CString::new(json.clone()).unwrap();
        let config = DEFAULT_C_SWARM_CONFIG_FOR_TEST.with(|c| *c);
        let json_handle = swarm_init_from_json(c_json.as_ptr(), &config, std::ptr::null(), 0);
        assert!(!json_handle.is_null(), "JSON restart failed: {}", swarm_get_last_error_code());
        let (instance, _) = unsafe { legacy_solution(json_handle) };
        assert_parseable_certified_export(json_handle, &instance);
        swarm_free(json_handle);
        let path = std::env::temp_dir().join(format!("swarm-issue2-restart-{}.json", std::process::id()));
        std::fs::write(&path, json).unwrap();
        let c_path = std::ffi::CString::new(path.to_string_lossy().as_bytes()).unwrap();
        let file_handle = swarm_init(c_path.as_ptr(), &config, std::ptr::null(), 0);
        std::fs::remove_file(path).unwrap();
        assert!(!file_handle.is_null(), "file restart failed: {}", swarm_get_last_error_code());
        let (instance, _) = unsafe { legacy_solution(file_handle) };
        assert_parseable_certified_export(file_handle, &instance);
        assert_eq!(swarm_get_last_error_code(), 0);
        swarm_free(file_handle);
    }
    #[test]
    fn legacy_init_settle_preserves_frozen_hint_transform() {
        let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
            "name": "frozen-settle-regression", "strip_height": 20.0,
            "items": (0..4).map(|id| serde_json::json!({
                "id": id, "demand": 1, "allowed_orientations": [0.0],
                "shape": {"type": "rectangle", "data": {
                    "x_min": 0.0, "y_min": 0.0, "width": 10.0, "height": 10.0
                }}
            })).collect::<Vec<_>>()
        })).unwrap();
        let mut fixed_pose = None;
        for every in [0, 1] {
            let mut config = fast_config();
            config.rng_seed = Some(123);
            config.init_settle_every = every;
            config.init_settle_budget_ms = 300;
            config.init_compaction_ms = 0;
            config.nfp_lbf.enabled = false;
            config.refplace.enabled = false;
            let handle = build_handle(
                ext.clone(), None, config, DEFAULT_CALLBACK_OPTIONS,
                crate::ffi::InitialPlacementSettings::default(),
                vec![hint(0, true, 100.0, 0.1)],
            ).expect(swarm_core::dmsg!("partial frozen hint must construct an exact layout"));
            let (instance, solution) = unsafe { legacy_solution(handle) };
            swarm_free(handle);
            let pose = solution.layout_snapshot().placed_items().values()
                .find(|p| p.item().idx() == 0).unwrap().d_transf();
            assert!(crate::structure::pipeline::solution_validation_error_with_frozen_ids(
                &instance, &solution, &[0],
            ).is_none());
            if let Some(reference) = fixed_pose {
                assert_eq!(pose, reference, "settle must preserve the original frozen hint");
            } else {
                fixed_pose = Some(pose);
            }
        }
    }

    #[test]
    fn legacy_init_accepts_overlapping_frozen_hints() {
        clear_last_error();
        let hints = [hint(0, true, 5.0, 5.0), hint(1, true, 5.0, 5.0)];
        let handle = build_handle(
            rectangle_instance(),
            None,
            fast_config(),
            DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(),
            hints.to_vec(),
        );

        let handle = handle.expect(swarm_core::dmsg!("frozen overlap must be accepted"));
        assert_eq!(swarm_get_last_error_code(), 0);
        swarm_free(handle);
    }

    #[test]
    fn candidate_certification_restores_the_exact_fallback() {
        let handle = build_handle(
            rectangle_instance(),
            None,
            fast_config(),
            DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(),
            vec![],
        )
        .unwrap();
        let (instance, fallback) = unsafe { legacy_solution(handle) };
        let mut candidate = fallback.clone();
        let mut placements = candidate.layout_snapshot().placed_items().iter();
        let (_, first) = placements.next().unwrap();
        let first_transform = first.d_transf();
        let second_key = placements.next().map(|(key, _)| key).unwrap();
        candidate.layout_snapshot_mut().placed_items_mut()[second_key].set_d_transf(first_transform);

        let committed = certify_legacy_candidate(&instance, candidate, &fallback, "TEST").unwrap();
        assert_eq!(committed.strip_width(), fallback.strip_width());
        assert!(
            crate::structure::pipeline::solution_validation_error(&instance, &committed).is_none()
        );
        swarm_free(handle);
    }

    /// 원단 경계 보정본이 인증에 떨어지면 **보정 전 후보**가 살아남아야 한다.
    /// 예전에는 후보를 보정본으로 덮어쓴 뒤 인증해서, 실패하면 직전 단계의 해로 물러났다.
    #[test]
    fn an_invalid_fabric_edge_repair_keeps_the_unrepaired_candidate() {
        let handle = build_handle(
            rectangle_instance(),
            None,
            fast_config(),
            DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(),
            vec![],
        )
        .unwrap();
        let (instance, candidate) = unsafe { legacy_solution(handle) };
        // 두 조각을 같은 자리에 겹쳐 놓은 «보정본».
        let mut broken = candidate.clone();
        let mut placements = broken.layout_snapshot().placed_items().iter();
        let first_transform = placements.next().unwrap().1.d_transf();
        let second_key = placements.next().map(|(key, _)| key).unwrap();
        broken.layout_snapshot_mut().placed_items_mut()[second_key].set_d_transf(first_transform);
        assert!(
            crate::structure::pipeline::solution_validation_error(&instance, &broken).is_some()
        );

        let pose = |sol: &SPSolution| {
            sol.layout_snapshot()
                .placed_items()
                .values()
                .map(|p| p.d_transf().translation())
                .collect::<Vec<_>>()
        };
        let kept = keep_valid_repair(&instance, candidate.clone(), broken, &[], "TEST");
        assert_eq!(pose(&kept), pose(&candidate), "보정 전 후보가 아닌 것이 돌아왔다");
        // 유효한 보정본은 그대로 채택된다.
        let adopted =
            keep_valid_repair(&instance, candidate.clone(), candidate.clone(), &[], "TEST");
        assert_eq!(pose(&adopted), pose(&candidate));
        swarm_free(handle);
    }

    #[test]
    fn seed_duel_probe_time_scales_with_piece_count_within_one_to_three_seconds() {
        let ms = |n| seed_duel_probe_time(n).as_millis();
        assert_eq!(ms(0), 1000);
        assert_eq!(ms(24), 1000); // marques
        assert_eq!(ms(99), 1485); // shirts
        assert_eq!(ms(156), 2340); // 여성WSjk
        assert_eq!(ms(200), 3000); // babel 겉감
        assert_eq!(ms(usize::MAX), 3000);
    }

    /// 시드 대결은 후보가 **더 좁아졌을 때만** 후보를 고른다. 탐색 시간이 0 이면 두 탐색 모두
    /// 출발 배치를 그대로 돌려주므로 같은 배치끼리는 동률이고, 동률에서는 기존 배치가 남아야
    /// 한다 — 여기서 시드를 갈아 끼우면 초기 폭이 더 넓은 쪽이 아무 근거 없이 이긴다.
    #[test]
    fn a_seed_duel_between_identical_layouts_keeps_the_incumbent() {
        let handle = build_handle(
            rectangle_instance(),
            None,
            fast_config(),
            DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(),
            vec![],
        )
        .unwrap();
        let (instance, solution) = unsafe { legacy_solution(handle) };
        let config = fast_config();
        assert!(!seed_duel_with_probe(
            &instance,
            &solution,
            &solution,
            &config,
            std::time::Duration::ZERO,
        ));
        swarm_free(handle);
    }

    #[test]
    fn handle_shrink_updates_are_isolated_and_do_not_require_the_engine_lock() {
        let make = || {
            build_handle(
                rectangle_instance(),
                None,
                zero_time_phase_config(),
                DEFAULT_CALLBACK_OPTIONS,
                crate::ffi::InitialPlacementSettings::default(),
                vec![],
            )
            .unwrap()
        };
        let a = make();
        let b = make();
        assert_eq!(swarm_set_handle_shrink_step(a, 0.02), 1);
        assert_eq!(swarm_set_handle_shrink_step(b, 0.04), 1);
        {
            let handle = unsafe { &*a };
            let _engine = handle.lock_engine();
            let _scope = swarm_core::config::runtime::enter(handle.lifecycle.run_control.clone());
            assert_eq!(swarm_set_handle_shrink_step(a, 0.03), 1);
            assert_eq!(
                swarm_core::config::runtime::effective_shrink_step(0.001),
                0.03
            );
            assert_eq!(swarm_get_handle_shrink_step(b), 0.04);
            assert_eq!(swarm_set_handle_shrink_step(a, f32::NAN), 0);
            assert_eq!(swarm_get_handle_shrink_step(a), 0.03);
            swarm_clear_handle_shrink_step(a);
            assert_eq!(
                swarm_core::config::runtime::effective_shrink_step(0.001),
                0.001
            );
        }
        assert_eq!(swarm_set_handle_shrink_step(std::ptr::null(), 0.1), 0);
        assert_eq!(swarm_get_handle_shrink_step(std::ptr::null()), -1.0);
        swarm_free(a);
        swarm_free(b);
    }

    #[test]
    fn frozen_certification_rejects_an_exact_but_moved_lock() {
        let instance = internal_instance();
        let mut problem = hinted_problem(&[hint(0, true, 5.0, 5.0), hint(1, false, 20.0, 5.0)]);
        let fallback = problem.save();
        let key = problem
            .layout()
            .placed_items()
            .iter()
            .find(|(_, p)| p.item().idx() == 0)
            .unwrap()
            .0;
        problem.remove_item(key);
        problem.place_item(SPPlacement {
            item_idx: 0,
            d_transf: DTransformation::new(0.0, (8.0, 5.0)),
        });
        let candidate = problem.save();
        assert!(
            crate::structure::pipeline::solution_validation_error(&instance, &candidate).is_none()
        );
        let accepted = certify_legacy_candidate_with_frozen(
            &instance,
            candidate,
            &fallback,
            "TEST",
            &[0],
            None,
        )
        .unwrap();
        assert!(crate::util::assertions::frozen_poses_preserved(
            &fallback,
            &accepted,
            &[true, false]
        ));
    }

    #[test]
    fn legacy_export_rejects_a_stale_invalid_transform() {
        let handle = build_handle(
            rectangle_instance(),
            None,
            fast_config(),
            DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(),
            vec![],
        )
        .unwrap();
        {
            let handle_ref = unsafe { &*handle };
            let mut engine = handle_ref.lock_engine();
            let state = engine.legacy_mut();
            let solution = state.solution.as_mut().unwrap();
            let mut placements = solution.layout_snapshot().placed_items().iter();
            let (_, first) = placements.next().unwrap();
            let first_transform = first.d_transf();
            let second_key = placements.next().map(|(key, _)| key).unwrap();
            solution.layout_snapshot_mut().placed_items_mut()[second_key].set_d_transf(DTransformation::new(first_transform.rotation(), first_transform.translation()));
        }

        let json = swarm_get_solution_json(handle);
        assert!(json.is_null());
        assert_eq!(swarm_get_last_error_code(), 3005);
        swarm_free(handle);
    }

    #[test]
    fn stripe_snap_rebuilds_shape_and_cde_from_the_new_transform() {
        let mut problem = hinted_problem(&[hint(0, false, 6.0, 5.0), hint(1, false, 20.0, 5.0)]);

        assert!(snap_to_stripe(
            &mut problem,
            5.0,
            Axis::X,
            &std::collections::HashSet::new(),
        ));
        let placed = problem
            .layout()
            .placed_items()
            .values()
            .find(|placed| placed.item().idx() == 0)
            .unwrap();
        assert!((placed.shape().bbox().x_min - 5.0).abs() <= 1.0e-4);
        let expected = problem
            .instance()
            .item(placed.item().idx())
            .shape_cd()
            .transform_clone(&placed.d_transf().compose());
        assert!((placed.shape().bbox().x_min - expected.bbox().x_min).abs() <= 1.0e-5);
        assert!((placed.shape().bbox().x_max - expected.bbox().x_max).abs() <= 1.0e-5);
        assert!(jagua_rs::util::assertions::layout_qt_matches_fresh_qt(
            &problem.layout()
        ));
        assert!(
            crate::structure::pipeline::solution_validation_error(
                &problem.instance(),
                &problem.save(),
            )
            .is_none()
        );
    }

    #[test]
    fn stripe_collision_rolls_back_transform_width_and_density() {
        let mut problem = hinted_problem(&[hint(0, false, 6.0, 5.0), hint(1, false, 14.0, 5.0)]);
        let before = problem.save();
        let before_density = problem.density();

        assert!(!snap_to_stripe(
            &mut problem,
            10.0,
            Axis::X,
            &std::collections::HashSet::new(),
        ));
        assert_eq!(problem.strip_width(), before.strip_width());
        assert_eq!(problem.density(), before_density);
        for before_item in before.layout_snapshot().placed_items().values() {
            let restored = problem
                .layout()
                .placed_items()
                .values()
                .find(|placed| placed.item().idx() == before_item.item().idx())
                .unwrap();
            assert_eq!(restored.d_transf(), before_item.d_transf());
        }
        assert!(jagua_rs::util::assertions::layout_qt_matches_fresh_qt(
            &problem.layout()
        ));
        assert!(
            crate::structure::pipeline::solution_validation_error(
                &problem.instance(),
                &problem.save(),
            )
            .is_none()
        );
    }

    #[test]
    fn stripe_snap_never_moves_frozen_item_kinds() {
        let mut problem = hinted_problem(&[hint(0, true, 6.0, 5.0), hint(1, false, 14.0, 5.0)]);
        let frozen = std::collections::HashSet::from([0usize]);

        assert!(snap_to_stripe(&mut problem, 10.0, Axis::X, &frozen));
        let frozen_item = problem
            .layout()
            .placed_items()
            .values()
            .find(|placed| placed.item().idx() == 0)
            .unwrap();
        let movable_item = problem
            .layout()
            .placed_items()
            .values()
            .find(|placed| placed.item().idx() == 1)
            .unwrap();
        assert!((frozen_item.shape().bbox().x_min - 6.0).abs() <= 1.0e-4);
        assert!((movable_item.shape().bbox().x_min - 10.0).abs() <= 1.0e-4);
        assert!(
            crate::structure::pipeline::solution_validation_error(
                &problem.instance(),
                &problem.save(),
            )
            .is_none()
        );
    }

    #[test]
    fn stripe_snap_expands_x_before_registering_a_rightward_hazard() {
        let mut problem = hinted_problem(&[hint(0, false, 6.0, 5.0), hint(1, false, 16.0, 5.0)]);
        let width_before = problem.strip_width();

        assert!(snap_to_stripe(
            &mut problem,
            10.0,
            Axis::X,
            &std::collections::HashSet::new(),
        ));
        assert!(problem.strip_width() > width_before);
        let rightmost = problem
            .layout()
            .placed_items()
            .values()
            .map(|placed| placed.shape().bbox().x_max)
            .max_by(f32::total_cmp)
            .unwrap();
        assert!(problem.strip_width() >= rightmost);
        assert!(jagua_rs::util::assertions::layout_qt_matches_fresh_qt(
            &problem.layout()
        ));
        assert!(
            crate::structure::pipeline::solution_validation_error(
                &problem.instance(),
                &problem.save(),
            )
            .is_none()
        );
    }

    #[test]
    fn compression_commits_and_callbacks_only_an_exact_solution() {
        let mut config = fast_config();
        config.cmpr_cfg.time_limit = Duration::ZERO;
        let handle = build_handle(
            rectangle_instance(),
            None,
            config,
            DEFAULT_CALLBACK_OPTIONS,
            crate::ffi::InitialPlacementSettings::default(),
            vec![],
        )
        .unwrap();
        let mut capture = CallbackCapture::default();
        swarm_set_callback(
            handle,
            Some(capture_callback),
            (&mut capture as *mut CallbackCapture).cast(),
            1,
        );

        assert_eq!(swarm_cp_phase(handle), 0);
        assert_eq!(capture.final_count, 1);
        assert!(capture.final_had_json);
        let (instance, solution) = unsafe { legacy_solution(handle) };
        assert!(
            crate::structure::pipeline::solution_validation_error(&instance, &solution).is_none()
        );
        let json = swarm_get_solution_json(handle);
        assert!(!json.is_null());
        crate::ffi::swarm_free_string(json);
        swarm_free(handle);
    }

    #[test]
    fn issue6_ordering_final_callback_and_getter_keep_no_worse_incumbent() {
        // Exercise actual Legacy and V2 step exports, including the V2 safe
        // callback cache. Intermediate exact search reports keep their policy.
        for grouped in [false,true] {
            for ordering in [false,true] {
                let mut config=zero_time_phase_config();
                config.expl_cfg.time_limit=Duration::from_millis(30);
                config.nfp_integration.ordering_exploration=ordering;
                config.nfp_lbf.cache_budget_ms=0;
                config.expl_cfg.separator_config.n_workers=1;
                let handle=if grouped {
                    let json=serde_json::json!({"schema_version":2,
                        "nesting":{"mode":"all_items","group_workers":1,"quantum_ms":1000,"locked_prefix_gap_mm":0.0},
                        "instance":{"name":"issue6-final-incumbent","strip_height":30.0,"items":[
                            {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{"type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":10.0,"height":10.0}},
                             "placed_hint":{"item_id":0,"is_freeze":0,"tr_x":0.0,"tr_y":0.0,"rot_angle_in_rad":0.0,"stripe_interval_y":0.0,"stripe_interval_x":0.0}}
                        ]},"regions":[],"place_groups":[]}).to_string();
                    build_grouped_handle(grouped_input::parse_and_validate(&json).unwrap(),config,DEFAULT_CALLBACK_OPTIONS).unwrap()
                } else {
                    build_handle(rectangle_instance(),None,config,DEFAULT_CALLBACK_OPTIONS,crate::ffi::InitialPlacementSettings::default(),vec![]).unwrap()
                };
                let before_json=swarm_get_solution_json(handle);
                let before:serde_json::Value=serde_json::from_str(unsafe{CStr::from_ptr(before_json)}.to_str().unwrap()).unwrap();
                crate::ffi::swarm_free_string(before_json);
                let mut capture=CallbackCapture::default();
                swarm_set_callback(handle,Some(capture_callback),(&mut capture as *mut CallbackCapture).cast(),1);
                assert_eq!(swarm_alg_phase(handle),0,"grouped={grouped}, ordering={ordering}");
                assert_eq!(capture.final_count,1);
                let json=swarm_get_solution_json(handle);
                let getter:serde_json::Value=serde_json::from_str(unsafe{CStr::from_ptr(json)}.to_str().unwrap()).unwrap();
                crate::ffi::swarm_free_string(json);
                let final_payload:serde_json::Value=serde_json::from_str(&capture.final_json[0]).unwrap();
                assert_eq!(getter["layout"],final_payload["solution"]["layout"]);
                assert_eq!(getter["strip_width"],final_payload["solution"]["strip_width"]);
                assert!(getter["strip_width"].as_f64().unwrap()<=before["strip_width"].as_f64().unwrap());
                if grouped && ordering { assert_eq!(getter["layout"],before["layout"]); }
                swarm_free(handle);
            }
        }
    }

}

#[cfg(test)]
#[path = "ffi_step_push_bench.rs"]
mod push_bench;
