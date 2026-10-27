//! End-to-end checks for the public structure-first C ABI.

#![cfg(feature = "structure-first")]

use cpswcore::ffi::{CSwarmConfig, swarm_free_string, swarm_get_default_config};
use cpswcore::ffi_step::*;
use std::ffi::{CStr, CString};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::thread;
use std::time::{Duration, Instant};

static CALLBACK_REPORT_TYPES: Mutex<Vec<i32>> = Mutex::new(Vec::new());
static GROUPED_CALLBACKS: Mutex<Vec<(i32, usize)>> = Mutex::new(Vec::new());
static LEGACY_JSON_CALLBACKS: Mutex<Vec<(i32, Option<String>)>> = Mutex::new(Vec::new());
static LEGACY_PHASE_CALLBACKS: Mutex<Vec<(i32, bool)>> = Mutex::new(Vec::new());
static LEGACY_TIMED_CALLBACKS: Mutex<Vec<(Instant, i32, bool)>> = Mutex::new(Vec::new());
/// (report_type, JSON 페이로드). 페이로드가 없으면 `None`.
static LEGACY_PAYLOAD_CALLBACKS: Mutex<Vec<(i32, Option<String>)>> = Mutex::new(Vec::new());
static STRUCTURE_FFI_SERIAL: Mutex<()> = Mutex::new(());

struct LegacyStructureStopCapture {
    handle: usize,
    callback_count: AtomicUsize,
    report_types: Mutex<Vec<i32>>,
    payloads: Mutex<Vec<String>>,
}

extern "C" fn count_callback(
    report_type: i32,
    _strip_width: f32,
    _density: f32,
    _solution_json: *const std::os::raw::c_char,
    _user_data: *mut std::ffi::c_void,
) {
    CALLBACK_REPORT_TYPES.lock().unwrap().push(report_type);
}

extern "C" fn grouped_callback(
    report_type: i32,
    _strip_width: f32,
    _density: f32,
    solution_json: *const std::os::raw::c_char,
    _user_data: *mut std::ffi::c_void,
) {
    let placed_count = if solution_json.is_null() {
        0
    } else {
        let json = unsafe { CStr::from_ptr(solution_json) }.to_str().unwrap();
        let value: serde_json::Value = serde_json::from_str(json).unwrap();
        value["solution"]["layout"]["placed_items"]
            .as_array()
            .unwrap()
            .len()
    };
    GROUPED_CALLBACKS
        .lock()
        .unwrap()
        .push((report_type, placed_count));
}

extern "C" fn legacy_json_callback(
    report_type: i32,
    _strip_width: f32,
    _density: f32,
    solution_json: *const std::os::raw::c_char,
    _user_data: *mut std::ffi::c_void,
) {
    let json = (!solution_json.is_null()).then(|| {
        unsafe { CStr::from_ptr(solution_json) }
            .to_str()
            .unwrap()
            .to_owned()
    });
    LEGACY_JSON_CALLBACKS
        .lock()
        .unwrap()
        .push((report_type, json));
}

extern "C" fn legacy_phase_callback(
    report_type: i32,
    _strip_width: f32,
    _density: f32,
    solution_json: *const std::os::raw::c_char,
    _user_data: *mut std::ffi::c_void,
) {
    LEGACY_PHASE_CALLBACKS
        .lock()
        .unwrap()
        .push((report_type, !solution_json.is_null()));
}

extern "C" fn legacy_payload_callback(
    report_type: i32,
    _strip_width: f32,
    _density: f32,
    solution_json: *const std::os::raw::c_char,
    _user_data: *mut std::ffi::c_void,
) {
    let payload = if solution_json.is_null() {
        None
    } else {
        Some(
            unsafe { std::ffi::CStr::from_ptr(solution_json) }
                .to_string_lossy()
                .into_owned(),
        )
    };
    LEGACY_PAYLOAD_CALLBACKS
        .lock()
        .unwrap()
        .push((report_type, payload));
}

extern "C" fn legacy_timed_callback(
    report_type: i32,
    _strip_width: f32,
    _density: f32,
    solution_json: *const std::os::raw::c_char,
    _user_data: *mut std::ffi::c_void,
) {
    LEGACY_TIMED_CALLBACKS.lock().unwrap().push((
        Instant::now(),
        report_type,
        !solution_json.is_null(),
    ));
}

extern "C" fn legacy_structure_stop_callback(
    report_type: i32,
    _strip_width: f32,
    _density: f32,
    solution_json: *const std::os::raw::c_char,
    user_data: *mut std::ffi::c_void,
) {
    let capture = unsafe { &*(user_data as *const LegacyStructureStopCapture) };
    if let Ok(mut report_types) = capture.report_types.lock() {
        report_types.push(report_type);
    }
    if !solution_json.is_null() {
        let payload = unsafe { CStr::from_ptr(solution_json) }
            .to_string_lossy()
            .into_owned();
        if let Ok(mut payloads) = capture.payloads.lock() {
            payloads.push(payload);
        }
    }

    if capture.callback_count.fetch_add(1, Ordering::AcqRel) == 0 {
        swarm_request_stop_and_wait(capture.handle as *mut SwarmStepHandle);
    }
}

fn quick_structure_config() -> CSwarmConfig {
    let mut config: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut config);
    config.rng_seed = 11;
    config.exploration.time_limit_secs = 0.02;
    config.compression.time_limit_secs = 0.02;
    config.polish.time_limit_secs = 0.02;
    config.structure_first.enabled = 1;
    config.structure_first.construction_starts = 1;
    config.structure_first.max_active_motifs = 8;
    config.structure_first.profile_top_k = 1;
    config.structure_first.p2_time_limit_secs = 0.02;
    config.structure_first.p3_time_limit_secs = 0.02;
    config.structure_first.p4_cycles = 0;
    config
}

fn rectangle(id: u64) -> serde_json::Value {
    serde_json::json!({
        "id": id,
        "demand": 1,
        "allowed_orientations": [0.0],
        "shape": {"type":"rectangle", "data": {
            "x_min":0.0, "y_min":0.0, "width":10.0, "height":8.0
        }}
    })
}

fn init(value: serde_json::Value, config: &CSwarmConfig) -> *mut SwarmStepHandle {
    let json = CString::new(value.to_string()).unwrap();
    let handle = swarm_init_from_json(json.as_ptr(), config, std::ptr::null(), 0);
    assert!(!handle.is_null());
    handle
}

fn take_json(ptr: *mut std::os::raw::c_char) -> serde_json::Value {
    assert!(!ptr.is_null());
    let json = unsafe { CStr::from_ptr(ptr) }.to_str().unwrap().to_owned();
    swarm_free_string(ptr);
    serde_json::from_str(&json).unwrap()
}

fn assert_same_layout(expected: &serde_json::Value, actual: &serde_json::Value) {
    let expected_items = expected["layout"]["placed_items"].as_array().unwrap();
    let actual_items = actual["layout"]["placed_items"].as_array().unwrap();
    assert_eq!(actual_items.len(), expected_items.len());
    for expected_item in expected_items {
        let item_id = expected_item["item_id"].as_u64().unwrap();
        let actual_item = actual_items
            .iter()
            .find(|item| item["item_id"].as_u64() == Some(item_id))
            .unwrap_or_else(|| panic!("item {item_id} is missing from the layout"));
        let expected_transform = &expected_item["transformation"];
        let actual_transform = &actual_item["transformation"];
        assert!(
            (actual_transform["rotation"].as_f64().unwrap()
                - expected_transform["rotation"].as_f64().unwrap())
            .abs()
                <= 1.0e-6
        );
        for axis in 0..2 {
            assert!(
                (actual_transform["translation"][axis].as_f64().unwrap()
                    - expected_transform["translation"][axis].as_f64().unwrap())
                .abs()
                    <= 1.0e-4,
                "item {item_id} changed its Init translation"
            );
        }
    }
}

#[test]
fn legacy_structure_phase_streams_and_exports_audit() {
    let _serial = STRUCTURE_FFI_SERIAL.lock().unwrap();
    CALLBACK_REPORT_TYPES.lock().unwrap().clear();
    let config = quick_structure_config();
    let handle = init(
        serde_json::json!({
            "name":"structure-ffi",
            "strip_height":24.0,
            "items":[rectangle(0), rectangle(1), rectangle(2)]
        }),
        &config,
    );
    swarm_set_callback(handle, Some(count_callback), std::ptr::null_mut(), 0);

    assert_eq!(swarm_gp_phase(handle), 0);
    let callback_types = CALLBACK_REPORT_TYPES.lock().unwrap().clone();
    assert!(callback_types.iter().filter(|&&kind| kind == 0).count() >= 2);
    assert!(callback_types.contains(&2));
    assert!(callback_types.contains(&3));
    assert!(callback_types.contains(&4));
    let solution = take_json(swarm_get_solution_json(handle));
    assert_eq!(
        solution["layout"]["placed_items"].as_array().unwrap().len(),
        3
    );

    let report = take_json(swarm_get_structure_report_json(handle));
    assert_eq!(report["schema_version"], 3);
    assert!(report.get("motif_pairs").is_some());
    assert!(report["motif_generation_stats"].is_object());
    assert!(report["motif_selection_stats"].is_object());
    assert!(report["p1_start_stats"].is_array());
    assert_eq!(
        report["construction_starts_attempted"],
        report["p1_start_stats"].as_array().unwrap().len()
    );
    assert!(report["p1_winner"].is_object());
    let stages = report["checkpoints"]
        .as_array()
        .unwrap()
        .iter()
        .map(|checkpoint| checkpoint["stage"].as_str().unwrap())
        .collect::<Vec<_>>();
    assert_eq!(
        stages,
        [
            "p1_construct",
            "p2_compress",
            "p3_explore",
            "p4_refine",
            "p5_finish"
        ]
    );
    assert_eq!(swarm_gp_phase(handle), -1);
    assert_eq!(swarm_alg_phase(handle), -1);
    swarm_free(handle);
}

#[test]
fn legacy_structure_json_starts_with_init_baseline_and_carries_complete_stage_snapshots() {
    let _serial = STRUCTURE_FFI_SERIAL.lock().unwrap();
    LEGACY_JSON_CALLBACKS.lock().unwrap().clear();
    let mut config = quick_structure_config();
    // The public config sanitizer clamps this to the documented 500 ms floor. The deliberately
    // short stages below therefore expose only forced baseline/stage milestones in this test.
    config.structure_first.progress_callback_interval_ms = 100;
    let handle = init(
        serde_json::json!({
            "name":"structure-legacy-json",
            "strip_height":24.0,
            "items":[rectangle(0), rectangle(1), rectangle(2)]
        }),
        &config,
    );
    let init_solution = take_json(swarm_get_solution_json(handle));
    swarm_set_callback(handle, Some(legacy_json_callback), std::ptr::null_mut(), 1);

    assert_eq!(swarm_gp_phase(handle), 0);
    let callbacks = LEGACY_JSON_CALLBACKS.lock().unwrap().clone();
    assert!(callbacks.len() >= 6, "callbacks: {callbacks:?}");
    assert_eq!(callbacks.first().map(|callback| callback.0), Some(0));
    assert_eq!(callbacks.last().map(|callback| callback.0), Some(4));

    let first: serde_json::Value = serde_json::from_str(
        callbacks
            .first()
            .and_then(|callback| callback.1.as_deref())
            .expect("the immediate baseline callback must include JSON"),
    )
    .unwrap();
    let first_items = first["solution"]["layout"]["placed_items"]
        .as_array()
        .unwrap();
    let init_items = init_solution["layout"]["placed_items"].as_array().unwrap();
    assert_eq!(first_items.len(), init_items.len());
    for (first_item, init_item) in first_items.iter().zip(init_items) {
        assert_eq!(first_item["item_id"], init_item["item_id"]);
        assert_eq!(
            first_item["transformation"]["rotation"],
            init_item["transformation"]["rotation"]
        );
        for axis in 0..2 {
            let first_coord = first_item["transformation"]["translation"][axis]
                .as_f64()
                .unwrap();
            let init_coord = init_item["transformation"]["translation"][axis]
                .as_f64()
                .unwrap();
            assert!((first_coord - init_coord).abs() < 1.0e-5);
        }
    }

    let complete_json_callbacks = callbacks
        .iter()
        .filter(|(report_type, _)| matches!(report_type, 0 | 2 | 3 | 4))
        .map(|(report_type, json)| {
            let value: serde_json::Value = serde_json::from_str(
                json.as_deref()
                    .unwrap_or_else(|| panic!("report type {report_type} was scalar-only")),
            )
            .unwrap();
            assert_eq!(
                value["solution"]["layout"]["placed_items"]
                    .as_array()
                    .unwrap()
                    .len(),
                3
            );
            assert!(
                value
                    .get("loss")
                    .and_then(serde_json::Value::as_f64)
                    .unwrap_or_else(|| panic!("Structure callback omitted loss: {value}"))
                    .abs()
                    <= 1.0e-6,
                "Structure callbacks must replay an exact-feasible snapshot: {value}"
            );
            assert!(
                value["collision_status"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .all(|status| status["collision"] == false),
                "exact Structure callbacks must not report contradictory collision flags: {value}"
            );
            *report_type
        })
        .collect::<Vec<_>>();

    // Baseline, P1, P2, P3, P4 and P5 are all forced checkpoints. Intermediate callbacks may be
    // interleaved on slower machines, so assert the milestone sequence as an ordered subsequence.
    let expected = [0, 0, 3, 0, 2, 4];
    let mut next = 0;
    for report_type in complete_json_callbacks {
        if next < expected.len() && report_type == expected[next] {
            next += 1;
        }
    }
    assert_eq!(next, expected.len(), "missing Structure milestone callback");
    swarm_free(handle);
}

#[test]
fn callback_options_toggle_loss_and_collision_status_independently() {
    let _serial = STRUCTURE_FFI_SERIAL.lock().unwrap();
    for (collect_loss, collect_collision_status) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
        LEGACY_JSON_CALLBACKS.lock().unwrap().clear();
        let mut config = quick_structure_config();
        config.callback_options.collect_loss = collect_loss;
        config.callback_options.collect_collision_status = collect_collision_status;
        let handle = init(
            serde_json::json!({
                "name":"callback-options",
                "strip_height":24.0,
                "items":[rectangle(0), rectangle(1), rectangle(2)]
            }),
            &config,
        );
        swarm_set_callback(handle, Some(legacy_json_callback), std::ptr::null_mut(), 1);

        assert_eq!(swarm_gp_phase(handle), 0);
        let callbacks = LEGACY_JSON_CALLBACKS.lock().unwrap().clone();
        let payload: serde_json::Value = serde_json::from_str(
            callbacks
                .first()
                .and_then(|callback| callback.1.as_deref())
                .expect("baseline callback must carry solution JSON"),
        )
        .unwrap();
        assert!(payload["solution"].is_object());
        assert_eq!(payload.get("loss").is_some(), collect_loss != 0);
        assert_eq!(
            payload.get("collision_status").is_some(),
            collect_collision_status != 0
        );
        if collect_loss != 0 {
            assert_eq!(payload["loss"], 0.0);
        }
        if collect_collision_status != 0 {
            assert!(
                payload["collision_status"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .all(|status| status["collision"] == false)
            );
        }
        swarm_free(handle);
    }
}

#[test]
fn legacy_structure_init_callback_can_stop_and_wait_without_fake_milestones() {
    let _serial = STRUCTURE_FFI_SERIAL.lock().unwrap();
    let mut config = quick_structure_config();
    config.structure_first.motif_enabled = 0;
    config.structure_first.construction_starts = 100_000;
    config.structure_first.p1_time_limit_secs = 30.0;
    config.structure_first.p2_time_limit_secs = 30.0;
    config.structure_first.p3_time_limit_secs = 30.0;
    config.structure_first.p4_cycles = 100_000;
    let items = (0..18).map(rectangle).collect::<Vec<_>>();
    let handle = init(
        serde_json::json!({
            "name":"legacy-structure-callback-stop",
            "strip_height":48.0,
            "items":items
        }),
        &config,
    );
    let init_solution = take_json(swarm_get_solution_json(handle));
    let capture = Arc::new(LegacyStructureStopCapture {
        handle: handle as usize,
        callback_count: AtomicUsize::new(0),
        report_types: Mutex::new(Vec::new()),
        payloads: Mutex::new(Vec::new()),
    });
    swarm_set_callback(
        handle,
        Some(legacy_structure_stop_callback),
        Arc::as_ptr(&capture) as *mut std::ffi::c_void,
        1,
    );

    let (tx, rx) = mpsc::channel();
    let handle_address = handle as usize;
    let phase_thread = thread::spawn(move || {
        let result = swarm_gp_phase(handle_address as *mut SwarmStepHandle);
        let _ = tx.send(result);
    });
    let result = match rx.recv_timeout(Duration::from_secs(8)) {
        Ok(result) => result,
        Err(error) => {
            // If this regresses into a callback self-deadlock, keep both callback userdata and
            // the native handle alive instead of letting the detached phase access freed memory.
            let _ = Arc::into_raw(capture);
            drop(phase_thread);
            panic!("legacy Structure callback stop-and-wait did not finish: {error}");
        }
    };
    phase_thread.join().unwrap();

    assert_eq!(result, 0);
    assert_eq!(swarm_is_stop_requested(handle), 1);
    assert_eq!(capture.callback_count.load(Ordering::Acquire), 1);
    assert_eq!(
        *capture.report_types.lock().unwrap(),
        [0],
        "cancelled legacy Structure must not emit fake P1-P5/final milestones"
    );
    let payloads = capture.payloads.lock().unwrap().clone();
    assert_eq!(payloads.len(), 1);
    let init_callback: serde_json::Value = serde_json::from_str(&payloads[0]).unwrap();
    let callback_items = init_callback["solution"]["layout"]["placed_items"]
        .as_array()
        .unwrap();
    assert_eq!(callback_items.len(), 18);
    assert_same_layout(&init_solution, &init_callback["solution"]);

    let final_solution = take_json(swarm_get_solution_json(handle));
    assert_same_layout(&init_solution, &final_solution);
    let cancelled_report = take_json(swarm_get_structure_report_json(handle));
    assert_eq!(cancelled_report["schema_version"], 3);
    assert_eq!(cancelled_report["cancelled"], true);
    swarm_free(handle);
}

/// 레거시 Explore 의 JSON 발행은 **실행 가능한 해에만** 붙는다.
///
/// # 이 계약은 뒤집힌 것이다
///
/// 예전에는 정반대였다: `ExplFeas`(겹침 없음)가 scalar-only 였고
/// `ExplImproving`(겹침 남음)이 무조건 JSON 을 실었다. 그래서 탐색 단계 내내
/// 소비자에게 가는 배치는 전부 겹친 것이었고, WPF 샘플은 JSON 없는 콜백을
/// 렌더링에서 버리므로 화면에 겹친 해만 흘렀다.
///
/// 그때의 동작을 잠그던 테스트가 바로 이 자리에 있었다
/// (`legacy ExplFeas must remain scalar-only`). 그 계약이 결함이었으므로
/// 테스트를 뒤집는다.
#[test]
fn legacy_explore_attaches_json_to_feasible_reports_and_withholds_it_from_overlapping_ones() {
    let _serial = STRUCTURE_FFI_SERIAL.lock().unwrap();
    LEGACY_PHASE_CALLBACKS.lock().unwrap().clear();
    let config = quick_structure_config();
    let handle = init(
        serde_json::json!({
            "name":"legacy-callback-contract",
            "strip_height":24.0,
            "items":[rectangle(0), rectangle(1), rectangle(2)]
        }),
        &config,
    );
    swarm_set_callback(handle, Some(legacy_phase_callback), std::ptr::null_mut(), 1);

    assert_eq!(swarm_alg_phase(handle), 0);
    let callbacks = LEGACY_PHASE_CALLBACKS.lock().unwrap().clone();
    // 0 = ExplFeas (겹침 없음), 1 = ExplInfeas, 2 = ExplImproving (겹침 남음),
    // 3 = CmprFeas, 4 = Final.
    assert!(
        callbacks
            .iter()
            .any(|(report_type, has_json)| *report_type == 0 && *has_json),
        "겹침 없는 ExplFeas 에 JSON 이 실려야 한다: {callbacks:?}"
    );
    assert!(
        callbacks
            .iter()
            .all(|(report_type, has_json)| *report_type != 2 || !has_json),
        "겹침이 남은 ExplImproving 에 JSON 이 실리면 안 된다: {callbacks:?}"
    );
    swarm_free(handle);
}

/// **핵심 회귀 방어.** JSON 이 실린 콜백은 예외 없이 충돌이 없어야 한다.
///
/// 리포트 종류가 아니라 **페이로드에 실제로 실린 진단값**을 본다. 종류만 보면
/// "그 종류가 정말 실행 가능한가" 라는 원래 질문을 되풀이하게 된다.
#[test]
fn every_json_bearing_callback_carries_a_collision_free_layout() {
    let _serial = STRUCTURE_FFI_SERIAL.lock().unwrap();
    LEGACY_PAYLOAD_CALLBACKS.lock().unwrap().clear();
    let mut config = quick_structure_config();
    config.exploration.time_limit_secs = 1.5;
    // 진단 수집을 명시적으로 켠다 — 꺼져 있으면 필드가 아예 직렬화되지 않아
    // 이 테스트가 조용히 무의미해진다.
    config.callback_options.collect_loss = 1;
    config.callback_options.collect_collision_status = 1;
    // 분리가 즉시 성공하면 ExplImproving 이 아예 안 나서 이 테스트가 무의미해진다.
    // 조각을 늘리고 스트립을 낮춰 분리 루프가 실제로 돌게 한다.
    let items = (0..48).map(rectangle).collect::<Vec<_>>();
    let handle = init(
        serde_json::json!({
            "name":"legacy-callback-feasibility",
            "strip_height":24.0,
            "items":items
        }),
        &config,
    );
    swarm_set_callback(handle, Some(legacy_payload_callback), std::ptr::null_mut(), 1);

    assert_eq!(swarm_alg_phase(handle), 0);
    let events = LEGACY_PAYLOAD_CALLBACKS.lock().unwrap().clone();
    let with_json = events
        .iter()
        .filter_map(|(t, p)| p.as_ref().map(|p| (*t, p)))
        .collect::<Vec<_>>();
    assert!(
        !with_json.is_empty(),
        "JSON 페이로드가 하나도 안 나왔다 — 소비자가 배치를 전혀 못 받는다: {:?}",
        events.iter().map(|(t, p)| (*t, p.is_some())).collect::<Vec<_>>()
    );
    // 진단 필드가 아예 없으면 아래 루프가 아무것도 단정하지 않는다 — 조용히
    // 무의미해지는 것을 막는다.
    let with_diagnostics = with_json
        .iter()
        .filter(|(_, p)| p.contains("\"collision_status\"") || p.contains("\"loss\""))
        .count();
    assert!(
        with_diagnostics > 0,
        "진단 필드(loss / collision_status)를 실은 페이로드가 하나도 없다 —          collect_loss / collect_collision_status 배선이 끊겼다"
    );
    for (report_type, payload) in &with_json {
        let value: serde_json::Value =
            serde_json::from_str(payload).expect("페이로드는 유효한 JSON 이어야 한다");
        if let Some(loss) = value.get("loss").and_then(|v| v.as_f64()) {
            assert_eq!(
                loss, 0.0,
                "report_type {report_type} 의 페이로드가 loss {loss} 로 겹침을 싣고 있다"
            );
        }
        if let Some(statuses) = value.get("collision_status").and_then(|v| v.as_array()) {
            let colliding = statuses
                .iter()
                .filter(|s| s.get("collision").and_then(|c| c.as_bool()) == Some(true))
                .count();
            assert_eq!(
                colliding, 0,
                "report_type {report_type} 의 페이로드에 충돌 조각이 {colliding}개 있다"
            );
        }
    }
    swarm_free(handle);
}

#[test]
fn legacy_explore_json_respects_the_configured_minimum_interval() {
    let _serial = STRUCTURE_FFI_SERIAL.lock().unwrap();
    LEGACY_TIMED_CALLBACKS.lock().unwrap().clear();
    let mut config = quick_structure_config();
    config.exploration.time_limit_secs = 1.5;
    config.structure_first.progress_callback_interval_ms = 500;
    let items = (0..36).map(rectangle).collect::<Vec<_>>();
    let handle = init(
        serde_json::json!({
            "name":"legacy-callback-throttle",
            "strip_height":48.0,
            "items":items
        }),
        &config,
    );
    swarm_set_callback(handle, Some(legacy_timed_callback), std::ptr::null_mut(), 1);

    assert_eq!(swarm_alg_phase(handle), 0);
    let json_events = LEGACY_TIMED_CALLBACKS
        .lock()
        .unwrap()
        .iter()
        .filter(|(_, _, has_json)| *has_json)
        .cloned()
        .collect::<Vec<_>>();
    assert!(
        !json_events.is_empty(),
        "legacy Explore must still attach at least one JSON snapshot: {json_events:?}"
    );
    // The interval is an event-driven minimum throttle, not a heartbeat. A
    // run with no second qualifying native event may legitimately emit only
    // one JSON snapshot; when multiple snapshots exist they must respect the
    // configured floor.
    for pair in json_events.windows(2) {
        assert!(
            pair[1].0.duration_since(pair[0].0) >= Duration::from_millis(475),
            "JSON callbacks violated the configured 500 ms floor: {json_events:?}"
        );
    }
    swarm_free(handle);
}

#[test]
fn grouped_structure_phase_returns_merged_report() {
    let _serial = STRUCTURE_FFI_SERIAL.lock().unwrap();
    GROUPED_CALLBACKS.lock().unwrap().clear();
    let mut config = quick_structure_config();
    // Values below 500 ms are clamped to 500 ms. Keep P2 alive beyond that
    // floor so this proves callbacks are streamed within a stage, not only at borders.
    config.structure_first.progress_callback_interval_ms = 100;
    config.structure_first.p2_time_limit_secs = 0.8;
    let handle = init(
        serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"size_regions", "group_workers":2, "quantum_ms":5,
                       "locked_prefix_gap_mm":0.0},
            "instance":{"name":"structure-grouped", "strip_height":24.0,
                        "items":[rectangle(0), rectangle(1), rectangle(2)]},
            "regions":[
                {"id":"S+M", "kind":"size_group", "item_ids":[0,1], "gap_after_mm":3.0},
                {"id":"XL", "kind":"remainder", "item_ids":[2], "gap_after_mm":0.0}
            ],
            "place_groups":[]
        }),
        &config,
    );
    swarm_set_callback(handle, Some(grouped_callback), std::ptr::null_mut(), 1);

    assert_eq!(swarm_gp_phase(handle), 0);
    let callbacks = GROUPED_CALLBACKS.lock().unwrap().clone();
    let callback_types = callbacks.iter().map(|(kind, _)| *kind).collect::<Vec<_>>();
    assert_eq!(callback_types.first(), Some(&0));
    assert_eq!(callback_types.last(), Some(&4));
    assert!(callback_types.iter().filter(|&&kind| kind == 3).count() >= 2);
    assert!(callback_types.contains(&0));
    assert!(callback_types.contains(&2));
    assert!(callbacks.iter().all(|(_, placed_count)| *placed_count == 3));
    let report = take_json(swarm_get_structure_report_json(handle));
    assert_eq!(report["schema_version"], 3);
    assert_eq!(report["nesting_mode"], "size_regions");
    assert_eq!(report["lanes"].as_array().unwrap().len(), 2);
    assert!(report["required_region_gap_mm"].as_f64().unwrap() >= 3.0);
    for lane in report["lanes"].as_array().unwrap() {
        assert_eq!(lane["report"]["schema_version"], 3);
        assert!(lane["report"]["motif_generation_stats"].is_object());
        assert!(lane["report"]["motif_selection_stats"].is_object());
        assert!(lane["report"]["p1_start_stats"].is_array());
    }
    swarm_free(handle);
}

#[test]
fn disabled_structure_request_is_explicit_and_does_not_break_legacy_phase() {
    let _serial = STRUCTURE_FFI_SERIAL.lock().unwrap();
    let mut config = quick_structure_config();
    config.structure_first.enabled = 0;
    let handle = init(
        serde_json::json!({
            "name":"structure-disabled",
            "strip_height":24.0,
            "items":[rectangle(0), rectangle(1)]
        }),
        &config,
    );

    assert_eq!(swarm_gp_phase(handle), -1);
    assert_eq!(swarm_get_last_error_code(), 3002);
    let error = take_json_string(swarm_get_last_error_message());
    assert!(error.contains("disabled"));
    assert_eq!(swarm_cp_phase(handle), 0);
    swarm_free(handle);
}

fn take_json_string(ptr: *mut std::os::raw::c_char) -> String {
    assert!(!ptr.is_null());
    let value = unsafe { CStr::from_ptr(ptr) }.to_str().unwrap().to_owned();
    swarm_free_string(ptr);
    value
}
