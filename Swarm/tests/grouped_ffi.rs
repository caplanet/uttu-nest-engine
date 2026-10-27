//! End-to-end checks for the versioned one-handle region contract.

use cpswcore::ffi::{CSwarmConfig, swarm_free_string, swarm_get_default_config};
use cpswcore::ffi_step::*;
use std::collections::{BTreeMap, BTreeSet};
use std::ffi::{CStr, CString};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::thread;
use std::time::{Duration, Instant};

struct CallbackCapture {
    started: Instant,
    events: Mutex<Vec<(Duration, i32, usize)>>,
}

/// Owns callback payloads until the phase returns.  The callback deliberately
/// performs no parsing or assertions: unwinding through an `extern "C"`
/// boundary would abort the test process instead of producing a useful test
/// failure.
struct JsonCallbackCapture {
    handle: usize,
    stop_mode: StopMode,
    callback_count: AtomicUsize,
    payloads: Mutex<Vec<(i32, String)>>,
}

#[derive(Clone, Copy)]
enum StopMode {
    None,
    Request,
    RequestAndWait,
}

extern "C" fn capture_grouped_progress(
    report_type: i32,
    _strip_width: f32,
    _density: f32,
    solution_json: *const std::os::raw::c_char,
    user_data: *mut std::ffi::c_void,
) {
    let capture = unsafe { &*(user_data as *const CallbackCapture) };
    let item_count = if solution_json.is_null() {
        0
    } else {
        let json = unsafe { CStr::from_ptr(solution_json) }.to_str().unwrap();
        let value: serde_json::Value = serde_json::from_str(json).unwrap();
        value["solution"]["layout"]["placed_items"]
            .as_array()
            .unwrap()
            .len()
    };
    capture
        .events
        .lock()
        .unwrap()
        .push((capture.started.elapsed(), report_type, item_count));
}

extern "C" fn capture_json_and_optionally_stop(
    report_type: i32,
    _strip_width: f32,
    _density: f32,
    solution_json: *const std::os::raw::c_char,
    user_data: *mut std::ffi::c_void,
) {
    let capture = unsafe { &*(user_data as *const JsonCallbackCapture) };
    if !solution_json.is_null() {
        let json = unsafe { CStr::from_ptr(solution_json) }
            .to_string_lossy()
            .into_owned();
        capture.payloads.lock().unwrap().push((report_type, json));
    }

    if capture.callback_count.fetch_add(1, Ordering::AcqRel) == 0 {
        let handle = capture.handle as *mut SwarmStepHandle;
        match capture.stop_mode {
            StopMode::None => {}
            StopMode::Request => swarm_request_stop(handle),
            StopMode::RequestAndWait => swarm_request_stop_and_wait(handle),
        }
    }
}

fn item(id: u64, frozen: bool, x: f32) -> serde_json::Value {
    let mut item = serde_json::json!({
        "id": id,
        "demand": 1,
        "allowed_orientations": [0.0],
        "shape": {"type":"rectangle", "data":{
            "x_min":0.0, "y_min":0.0, "width":10.0, "height":10.0
        }}
    });
    if frozen {
        item["placed_hint"] = serde_json::json!({
            "item_id":id, "is_freeze":1, "tr_x":x, "tr_y":0.0,
            "rot_angle_in_rad":0.0, "stripe_interval_y":0.0, "stripe_interval_x":0.0
        });
    }
    item
}

fn hinted_item(id: u64, frozen: bool, x: f32, y: f32) -> serde_json::Value {
    serde_json::json!({
        "id": id,
        "demand": 1,
        "allowed_orientations": [0.0, 180.0],
        "shape": {"type":"rectangle", "data":{
            "x_min":0.0, "y_min":0.0, "width":10.0, "height":10.0
        }},
        "placed_hint": {
            "item_id":id, "is_freeze":if frozen { 1 } else { 0 },
            "tr_x":x, "tr_y":y, "rot_angle_in_rad":0.0,
            "stripe_interval_y":0.0, "stripe_interval_x":0.0
        }
    })
}

fn init(json: serde_json::Value) -> *mut SwarmStepHandle {
    let json = CString::new(json.to_string()).unwrap();
    swarm_init_from_json(json.as_ptr(), std::ptr::null(), std::ptr::null(), 0)
}

fn init_with_config(json: serde_json::Value, config: &CSwarmConfig) -> *mut SwarmStepHandle {
    let json = CString::new(json.to_string()).unwrap();
    swarm_init_from_json(json.as_ptr(), config, std::ptr::null(), 0)
}

fn solution(handle: *mut SwarmStepHandle) -> serde_json::Value {
    let ptr = swarm_get_solution_json(handle);
    assert!(!ptr.is_null());
    let json = unsafe { CStr::from_ptr(ptr) }.to_str().unwrap().to_owned();
    swarm_free_string(ptr);
    serde_json::from_str(&json).unwrap()
}

fn callback_solutions(capture: &JsonCallbackCapture) -> Vec<(i32, serde_json::Value)> {
    capture
        .payloads
        .lock()
        .unwrap()
        .iter()
        .map(|(kind, json)| (*kind, serde_json::from_str(json).unwrap()))
        .collect()
}

fn placements(value: &serde_json::Value) -> BTreeMap<u64, &serde_json::Value> {
    let solution = value.get("solution").unwrap_or(value);
    solution["layout"]["placed_items"]
        .as_array()
        .expect("solution must contain placed_items")
        .iter()
        .map(|placement| (placement["item_id"].as_u64().unwrap(), placement))
        .collect()
}

fn translation(placement: &serde_json::Value) -> (f64, f64) {
    let translation = placement["transformation"]["translation"]
        .as_array()
        .unwrap();
    (
        translation[0].as_f64().unwrap(),
        translation[1].as_f64().unwrap(),
    )
}

#[test]
#[cfg(feature = "structure-first")]
fn fixed_only_size_regions_still_require_structure_to_be_enabled() {
    let mut config: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut config);
    assert_eq!(config.structure_first.enabled, 0);
    let handle = init_with_config(
        serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"size_regions", "group_workers":1, "quantum_ms":5,
                       "locked_prefix_gap_mm":0.0},
            "instance":{"name":"fixed-only-disabled", "strip_height":20.0,
                        "items":[item(0, true, 0.0)]},
            "regions":[{"id":"fixed", "kind":"size_bundle", "item_ids":[0],
                        "gap_after_mm":0.0}],
            "place_groups":[]
        }),
        &config,
    );
    assert!(!handle.is_null());

    assert_eq!(swarm_gp_phase(handle), -1);
    assert_eq!(swarm_get_last_error_code(), 3003);
    let error_ptr = swarm_get_last_error_message();
    assert!(!error_ptr.is_null());
    let error = unsafe { CStr::from_ptr(error_ptr) }
        .to_string_lossy()
        .into_owned();
    swarm_free_string(error_ptr);
    assert!(error.contains("structure-first is disabled"), "{error}");
    assert_eq!(placements(&solution(handle)).len(), 1);
    swarm_free(handle);
}

#[test]
#[cfg(feature = "structure-first")]
fn fixed_boundary_contact_structure_callback_has_zero_loss_and_no_collision() {
    let mut config: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut config);
    config.structure_first.enabled = 1;
    let handle = init_with_config(
        serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"size_regions", "group_workers":1, "quantum_ms":5,
                       "locked_prefix_gap_mm":0.0},
            "instance":{"name":"fixed-boundary-callback", "strip_height":10.0,
                        "items":[item(0, true, 0.0)]},
            "regions":[{"id":"fixed", "kind":"size_bundle", "item_ids":[0],
                        "gap_after_mm":0.0}],
            "place_groups":[]
        }),
        &config,
    );
    assert!(!handle.is_null());
    let capture = Arc::new(JsonCallbackCapture {
        handle: handle as usize,
        stop_mode: StopMode::RequestAndWait,
        callback_count: AtomicUsize::new(0),
        payloads: Mutex::new(vec![]),
    });
    swarm_set_callback(
        handle,
        Some(capture_json_and_optionally_stop),
        Arc::as_ptr(&capture) as *mut std::ffi::c_void,
        1,
    );

    assert_eq!(swarm_gp_phase(handle), 0);
    let callbacks = callback_solutions(&capture);
    assert_eq!(callbacks.len(), 1);
    let callback = &callbacks[0].1;
    assert_eq!(callback["loss"].as_f64(), Some(0.0));
    assert!(
        callback["collision_status"]
            .as_array()
            .unwrap()
            .iter()
            .all(|status| status["collision"].as_bool() == Some(false))
    );
    swarm_free(handle);
}

fn assert_complete_global_layout(
    value: &serde_json::Value,
    expected_ids: impl IntoIterator<Item = u64>,
) {
    let expected = expected_ids.into_iter().collect::<BTreeSet<_>>();
    let solution = value.get("solution").unwrap_or(value);
    let placed = solution["layout"]["placed_items"].as_array().unwrap();
    let actual = placed
        .iter()
        .map(|placement| placement["item_id"].as_u64().unwrap())
        .collect::<BTreeSet<_>>();
    assert_eq!(actual, expected, "callback must expose global item IDs");
    assert_eq!(
        placed.len(),
        actual.len(),
        "callback must not duplicate a global item ID"
    );
}

fn assert_place_group_pose(value: &serde_json::Value, expected_local_delta: (f64, f64)) {
    let placed = placements(value);
    let first = placed.get(&0).unwrap();
    let second = placed.get(&1).unwrap();
    let (ax, ay) = translation(first);
    let (bx, by) = translation(second);
    let anchor_radians = first["transformation"]["rotation"]
        .as_f64()
        .unwrap()
        .to_radians();
    let member_rotation = second["transformation"]["rotation"].as_f64().unwrap();
    let anchor_rotation = first["transformation"]["rotation"].as_f64().unwrap();
    let rotation_delta = ((member_rotation - anchor_rotation + 180.0).rem_euclid(360.0)) - 180.0;
    assert!(
        rotation_delta.abs() <= 1.0e-3,
        "PlaceGroup member rotations diverged: {anchor_rotation} vs {member_rotation}"
    );

    // Convert the world-space origin delta back into the anchor's local frame.
    // A rigid common rotation must leave this vector unchanged.
    let dx = bx - ax;
    let dy = by - ay;
    let local_dx = anchor_radians.cos() * dx + anchor_radians.sin() * dy;
    let local_dy = -anchor_radians.sin() * dx + anchor_radians.cos() * dy;
    assert!(
        (local_dx - expected_local_delta.0).abs() <= 1.0e-3
            && (local_dy - expected_local_delta.1).abs() <= 1.0e-3,
        "PlaceGroup local delta changed: expected {expected_local_delta:?}, got ({local_dx}, {local_dy})"
    );
}

#[test]
fn size_regions_use_one_handle_and_merge_locked_prefix_and_gaps() {
    let handle = init(serde_json::json!({
        "schema_version":2,
        "nesting":{"mode":"size_regions", "group_workers":1, "quantum_ms":5,
                   "locked_prefix_gap_mm":7.0},
        "instance":{"name":"ffi-regions", "strip_height":25.0, "items":[
            item(0, true, 0.0), item(1, false, 0.0),
            item(2, false, 0.0), item(3, false, 0.0)
        ]},
        "regions":[
            {"id":"S+M", "kind":"size_group", "item_ids":[0,1], "gap_after_mm":5.0},
            {"id":"remainder", "kind":"remainder", "item_ids":[2,3], "gap_after_mm":0.0}
        ],
        "place_groups":[]
    }));
    assert!(
        !handle.is_null(),
        "valid V2 input must create one public handle"
    );

    let output = solution(handle);
    let placed = output["layout"]["placed_items"].as_array().unwrap();
    assert_eq!(placed.len(), 4);
    let ids = placed
        .iter()
        .map(|p| p["item_id"].as_u64().unwrap())
        .collect::<std::collections::BTreeSet<_>>();
    assert_eq!(ids, std::collections::BTreeSet::from([0, 1, 2, 3]));

    let x = |id: u64| {
        placed
            .iter()
            .find(|p| p["item_id"].as_u64() == Some(id))
            .unwrap()["transformation"]["translation"][0]
            .as_f64()
            .unwrap()
    };
    assert!(
        x(1) >= 17.0 - 1e-3,
        "movable lanes start after lock right+gap"
    );
    assert!(x(2).min(x(3)) >= x(1) + 10.0 + 5.0 - 1e-3);

    swarm_free(handle);
}

#[test]
fn place_group_crossing_regions_is_rejected_at_the_ffi_boundary() {
    let mut a = item(0, false, 0.0);
    let mut b = item(1, false, 15.0);
    for (value, id, x) in [(&mut a, 0, 0.0), (&mut b, 1, 15.0)] {
        value["placed_hint"] = serde_json::json!({
            "item_id":id, "is_freeze":0, "tr_x":x, "tr_y":0.0,
            "rot_angle_in_rad":0.0, "stripe_interval_y":0.0, "stripe_interval_x":0.0
        });
    }
    let handle = init(serde_json::json!({
        "schema_version":2,
        "nesting":{"mode":"size_regions", "group_workers":2, "quantum_ms":5,
                   "locked_prefix_gap_mm":0.0},
        "instance":{"name":"ffi-invalid", "strip_height":25.0, "items":[a,b]},
        "regions":[
            {"id":"a", "kind":"size_bundle", "item_ids":[0], "gap_after_mm":1.0},
            {"id":"b", "kind":"size_bundle", "item_ids":[1], "gap_after_mm":0.0}
        ],
        "place_groups":[{"id":9, "member_item_ids":[0,1]}]
    }));
    assert!(handle.is_null());
    assert_eq!(swarm_get_last_error_code(), 1010);
    let error_ptr = swarm_get_last_error_message();
    assert!(!error_ptr.is_null());
    let error = unsafe { CStr::from_ptr(error_ptr) }
        .to_str()
        .unwrap()
        .to_owned();
    swarm_free_string(error_ptr);
    assert!(error.contains("spans multiple size regions"));
}

#[test]
fn grouped_handle_runs_all_three_public_phases() {
    let mut config: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut config);
    config.rng_seed = 17;
    config.exploration.time_limit_secs = 0.02;
    config.compression.time_limit_secs = 0.02;
    config.polish.time_limit_secs = 0.02;

    let handle = init_with_config(
        serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"size_regions", "group_workers":2, "quantum_ms":5,
                       "locked_prefix_gap_mm":0.0},
            "instance":{"name":"ffi-phases", "strip_height":25.0, "items":[
                item(0, false, 0.0), item(1, false, 0.0), item(2, false, 0.0)
            ]},
            "regions":[
                {"id":"g", "kind":"size_group", "item_ids":[0,1], "gap_after_mm":2.0},
                {"id":"rest", "kind":"remainder", "item_ids":[2], "gap_after_mm":0.0}
            ],
            "place_groups":[]
        }),
        &config,
    );
    assert!(!handle.is_null());
    assert_eq!(swarm_alg_phase(handle), 0);
    assert_eq!(swarm_cp_phase(handle), 0);
    assert_eq!(swarm_refine_phase(handle), 0);
    assert_eq!(
        solution(handle)["layout"]["placed_items"]
            .as_array()
            .unwrap()
            .len(),
        3
    );
    swarm_free(handle);
}

#[test]
fn size_region_legacy_phases_stream_merged_progress_at_the_configured_interval() {
    let mut config: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut config);
    config.rng_seed = 31;
    config.exploration.time_limit_secs = 1.25;
    config.compression.time_limit_secs = 1.25;
    config.polish.time_limit_secs = 1.25;
    config.structure_first.progress_callback_interval_ms = 500;

    let handle = init_with_config(
        serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"size_regions", "group_workers":1, "quantum_ms":25,
                       "locked_prefix_gap_mm":0.0},
            "instance":{"name":"ffi-progress", "strip_height":25.0, "items":[
                item(0, false, 0.0), item(1, false, 0.0),
                item(2, false, 0.0), item(3, false, 0.0)
            ]},
            "regions":[
                {"id":"a", "kind":"size_bundle", "item_ids":[0,1], "gap_after_mm":2.0},
                {"id":"b", "kind":"size_bundle", "item_ids":[2,3], "gap_after_mm":0.0}
            ],
            "place_groups":[]
        }),
        &config,
    );
    assert!(!handle.is_null());

    let capture = CallbackCapture {
        started: Instant::now(),
        events: Mutex::new(vec![]),
    };
    swarm_set_callback(
        handle,
        Some(capture_grouped_progress),
        &capture as *const CallbackCapture as *mut std::ffi::c_void,
        1,
    );

    assert_eq!(swarm_alg_phase(handle), 0);
    let events = capture.events.lock().unwrap();
    // Explore 의 lane slice 는 라운드마다 2배로 자란다. 미리보기는 라운드 경계에서만
    // 나오므로 '구간당 1개'가 보장되지 않는다 — 타이밍이 아니라 구조적 성질이다.
    // 그래서 개수가 아니라 (a) 첫 미리보기가 스로틀 하한을 지켰는지,
    // (b) 미리보기 종류가 ExplImproving(2) 인지, (c) 마지막이 경계(4)인지를 본다.
    assert!(
        events.len() >= 2,
        "expected at least one interval preview plus the phase boundary, got {events:?}"
    );
    assert!(events[0].0 >= Duration::from_millis(450));
    let (boundary, previews) = events.split_last().expect("checked non-empty above");
    assert_eq!(boundary.1, 4, "phase boundary must be Final(4): {events:?}");
    assert!(
        previews.iter().all(|(_, report_type, _)| *report_type == 2),
        "in-phase previews must be ExplImproving(2): {events:?}"
    );
    assert!(events.iter().all(|(_, _, item_count)| *item_count == 4));
    drop(events);

    let compression_capture = CallbackCapture {
        started: Instant::now(),
        events: Mutex::new(vec![]),
    };
    swarm_set_callback(
        handle,
        Some(capture_grouped_progress),
        &compression_capture as *const CallbackCapture as *mut std::ffi::c_void,
        1,
    );
    assert_eq!(swarm_cp_phase(handle), 0);
    let compression_events = compression_capture.events.lock().unwrap();
    // 미리보기는 슬라이스 라운드 경계에서만 나온다. 개수가 아니라
    // 스로틀 하한과 종류로 검증한다 (Explore 블록과 동일한 이유).
    assert!(compression_events.len() >= 2, "{compression_events:?}");
    assert!(compression_events[0].0 >= Duration::from_millis(450));
    assert!(
        compression_events[..compression_events.len() - 1]
            .iter()
            .all(|(_, report_type, item_count)| *report_type == 3 && *item_count == 4)
    );
    assert_eq!(compression_events.last().unwrap().1, 4);
    assert_eq!(compression_events.last().unwrap().2, 4);
    drop(compression_events);

    let polish_capture = CallbackCapture {
        started: Instant::now(),
        events: Mutex::new(vec![]),
    };
    swarm_set_callback(
        handle,
        Some(capture_grouped_progress),
        &polish_capture as *const CallbackCapture as *mut std::ffi::c_void,
        1,
    );
    assert_eq!(swarm_refine_phase(handle), 0);
    let polish_events = polish_capture.events.lock().unwrap();
    // 미리보기는 슬라이스 라운드 경계에서만 나온다. 개수가 아니라
    // 스로틀 하한과 종류로 검증한다 (Explore 블록과 동일한 이유).
    assert!(polish_events.len() >= 2, "{polish_events:?}");
    assert!(polish_events[0].0 >= Duration::from_millis(450));
    assert_eq!(polish_events.last().unwrap().1, 4);
    assert!(
        polish_events[..polish_events.len() - 1]
            .iter()
            .all(|(_, report_type, item_count)| *report_type == 3 && *item_count == 4)
    );
    assert_eq!(polish_events.last().unwrap().2, 4);
    drop(polish_events);

    swarm_free(handle);
}

fn assert_locked_prefix_and_region_gap(value: &serde_json::Value) {
    assert_complete_global_layout(value, 0..6);
    let placed = placements(value);
    let (locked_x, locked_y) = translation(placed.get(&0).unwrap());
    assert!((locked_x - 3.0).abs() <= 1.0e-3);
    assert!(locked_y.abs() <= 1.0e-3);

    let first_lane_right = [1, 2]
        .into_iter()
        .map(|id| translation(placed.get(&id).unwrap()).0 + 10.0)
        .fold(f64::NEG_INFINITY, f64::max);
    let second_lane_left = [3, 4, 5]
        .into_iter()
        .map(|id| translation(placed.get(&id).unwrap()).0)
        .fold(f64::INFINITY, f64::min);
    assert!(
        second_lane_left + 1.0e-3 >= first_lane_right + 7.0,
        "required 7 mm region gap was lost: first right={first_lane_right}, second left={second_lane_left}"
    );
}

fn stop_from_grouped_callback(mode: StopMode) {
    let mut config: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut config);
    config.rng_seed = 73;
    // The test timeout is deliberately much shorter than this phase budget.
    // Completion therefore demonstrates callback-driven cancellation rather
    // than an ordinary phase timeout.
    config.exploration.time_limit_secs = 30.0;
    config.structure_first.progress_callback_interval_ms = 500;

    let handle = init_with_config(
        serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"size_regions", "group_workers":2, "quantum_ms":25,
                       "locked_prefix_gap_mm":9.0},
            "instance":{"name":"ffi-stop", "strip_height":25.0, "items":[
                item(0, true, 3.0), item(1, false, 0.0), item(2, false, 0.0),
                item(3, false, 0.0), item(4, false, 0.0), item(5, false, 0.0)
            ]},
            "regions":[
                {"id":"first", "kind":"size_group", "item_ids":[0,1,2], "gap_after_mm":7.0},
                {"id":"second", "kind":"remainder", "item_ids":[3,4,5], "gap_after_mm":0.0}
            ],
            "place_groups":[]
        }),
        &config,
    );
    assert!(!handle.is_null());

    let capture = Arc::new(JsonCallbackCapture {
        handle: handle as usize,
        stop_mode: mode,
        callback_count: AtomicUsize::new(0),
        payloads: Mutex::new(vec![]),
    });
    swarm_set_callback(
        handle,
        Some(capture_json_and_optionally_stop),
        Arc::as_ptr(&capture) as *mut std::ffi::c_void,
        1,
    );

    let (tx, rx) = mpsc::channel();
    let handle_address = handle as usize;
    let phase_thread = thread::spawn(move || {
        let result = swarm_alg_phase(handle_address as *mut SwarmStepHandle);
        let _ = tx.send(result);
    });
    let result = match rx.recv_timeout(Duration::from_secs(8)) {
        Ok(result) => result,
        Err(error) => {
            // Keep callback userdata alive if a regression leaves the phase
            // thread blocked.  This failure path intentionally leaks only the
            // test handle/capture so that it cannot become a use-after-free.
            let _ = Arc::into_raw(capture);
            drop(phase_thread);
            panic!("callback-driven grouped stop did not finish: {error}");
        }
    };
    phase_thread.join().unwrap();
    assert_eq!(result, 0);
    assert_eq!(swarm_is_stop_requested(handle), 1);
    assert!(capture.callback_count.load(Ordering::Acquire) >= 1);

    let callbacks = callback_solutions(&capture);
    assert!(
        callbacks.len() >= 2,
        "expected a progress callback followed by the phase boundary: {callbacks:?}"
    );
    for (_, value) in &callbacks {
        assert_locked_prefix_and_region_gap(value);
    }
    assert_locked_prefix_and_region_gap(&solution(handle));
    swarm_free(handle);
}

#[test]
fn grouped_callback_can_request_stop_with_two_lane_workers() {
    stop_from_grouped_callback(StopMode::Request);
}

#[test]
fn grouped_callback_can_request_stop_and_wait_without_self_deadlock() {
    stop_from_grouped_callback(StopMode::RequestAndWait);
}

#[test]
fn partially_locked_place_group_is_fully_frozen_through_all_public_phases() {
    let mut config: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut config);
    config.rng_seed = 91;
    config.exploration.time_limit_secs = 0.03;
    config.compression.time_limit_secs = 0.03;
    config.polish.time_limit_secs = 0.03;

    let handle = init_with_config(
        serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items", "group_workers":1, "quantum_ms":5,
                       "locked_prefix_gap_mm":0.0},
            "instance":{"name":"ffi-partial-group-lock", "strip_height":30.0, "items":[
                hinted_item(0, true, 4.0, 2.0),
                hinted_item(1, false, 16.0, 2.0),
                item(2, false, 0.0), item(3, false, 0.0)
            ]},
            "regions":[],
            "place_groups":[{"id":41, "member_item_ids":[0,1]}]
        }),
        &config,
    );
    assert!(!handle.is_null());

    let assert_group_stayed_locked = |value: &serde_json::Value| {
        assert_complete_global_layout(value, 0..4);
        let placed = placements(value);
        let (first_x, first_y) = translation(placed.get(&0).unwrap());
        let (second_x, second_y) = translation(placed.get(&1).unwrap());
        assert!((first_x - 4.0).abs() <= 1.0e-3 && (first_y - 2.0).abs() <= 1.0e-3);
        assert!((second_x - 16.0).abs() <= 1.0e-3 && (second_y - 2.0).abs() <= 1.0e-3);
    };

    assert_group_stayed_locked(&solution(handle));
    assert_eq!(swarm_alg_phase(handle), 0);
    assert_group_stayed_locked(&solution(handle));
    assert_eq!(swarm_cp_phase(handle), 0);
    assert_group_stayed_locked(&solution(handle));
    assert_eq!(swarm_refine_phase(handle), 0);
    assert_group_stayed_locked(&solution(handle));
    swarm_free(handle);
}

#[test]
fn all_items_place_group_stays_rigid_in_callbacks_and_finals() {
    let mut config: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut config);
    config.rng_seed = 113;
    config.exploration.time_limit_secs = 0.05;
    config.compression.time_limit_secs = 0.05;
    config.polish.time_limit_secs = 0.05;
    config.structure_first.progress_callback_interval_ms = 500;

    let mut input_items = vec![
        hinted_item(0, false, 0.01, 0.01),
        hinted_item(1, false, 12.01, 2.01),
    ];
    input_items.extend((2..14).map(|id| item(id, false, 0.0)));

    let handle = init_with_config(
        serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items", "group_workers":1, "quantum_ms":5,
                       "locked_prefix_gap_mm":0.0},
            "instance":{"name":"ffi-rigid-group", "strip_height":40.0, "items":input_items},
            "regions":[],
            "place_groups":[{"id":42, "member_item_ids":[0,1]}]
        }),
        &config,
    );
    assert!(!handle.is_null());

    let mut snapshots = vec![solution(handle)];
    let capture = Arc::new(JsonCallbackCapture {
        handle: handle as usize,
        stop_mode: StopMode::None,
        callback_count: AtomicUsize::new(0),
        payloads: Mutex::new(vec![]),
    });
    swarm_set_callback(
        handle,
        Some(capture_json_and_optionally_stop),
        Arc::as_ptr(&capture) as *mut std::ffi::c_void,
        1,
    );

    // 단계 경계는 종류를 가리지 않고 '인증된 Final(4)' 스냅샷 하나를 발행한다.
    // 구간 미리보기만 단계별 종류(Explore=2, Compress=3)를 쓴다.
    for (phase, boundary_type, minimum_payloads) in [
        (
            swarm_alg_phase as extern "C" fn(*mut SwarmStepHandle) -> i32,
            4,
            1,
        ),
        (swarm_cp_phase, 4, 1),
        // PlaceGroup-aware Polish intentionally skips unsafe per-item passes,
        // so only its mandatory final boundary is required.
        (swarm_refine_phase, 4, 1),
    ] {
        let before = capture.payloads.lock().unwrap().len();
        assert_eq!(phase(handle), 0);
        let phase_payloads = capture.payloads.lock().unwrap()[before..].to_vec();
        assert!(
            phase_payloads.len() >= minimum_payloads,
            "phase must emit interval preview(s) plus its boundary: {phase_payloads:?}"
        );
        assert_eq!(phase_payloads.last().unwrap().0, boundary_type);
        snapshots.push(solution(handle));
    }
    snapshots.extend(
        callback_solutions(&capture)
            .into_iter()
            .map(|(_, value)| value),
    );
    assert!(snapshots.len() >= 7);
    for snapshot in &snapshots {
        assert_complete_global_layout(snapshot, 0..14);
        assert_place_group_pose(snapshot, (12.0, 2.0));
    }
    swarm_free(handle);
}

#[test]
#[cfg(feature = "structure-first")]
fn all_items_place_group_structure_stop_and_wait_returns_without_fake_final_stages() {
    let mut config: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut config);
    // The P1 fallback this test exercises is specific to rigid place groups.
    config.place_group.mode = cpswcore::ffi::PLACE_GROUP_RIGID;
    config.rng_seed = 117;
    config.structure_first.enabled = 1;
    config.structure_first.motif_enabled = 0;
    config.structure_first.construction_starts = 100_000;
    config.structure_first.p1_time_limit_secs = 30.0;
    config.structure_first.p2_time_limit_secs = 30.0;
    config.structure_first.p3_time_limit_secs = 30.0;
    config.structure_first.progress_callback_interval_ms = 500;

    let mut input_items = vec![
        hinted_item(0, false, 0.01, 0.01),
        hinted_item(1, false, 12.01, 2.01),
    ];
    input_items.extend((2..18).map(|id| item(id, false, 0.0)));
    let handle = init_with_config(
        serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items", "group_workers":2, "quantum_ms":5,
                       "locked_prefix_gap_mm":0.0},
            "instance":{"name":"ffi-rigid-structure-stop", "strip_height":40.0,
                        "items":input_items},
            "regions":[],
            "place_groups":[{"id":52, "member_item_ids":[0,1]}]
        }),
        &config,
    );
    assert!(!handle.is_null());

    let capture = Arc::new(JsonCallbackCapture {
        handle: handle as usize,
        stop_mode: StopMode::RequestAndWait,
        callback_count: AtomicUsize::new(0),
        payloads: Mutex::new(vec![]),
    });
    swarm_set_callback(
        handle,
        Some(capture_json_and_optionally_stop),
        Arc::as_ptr(&capture) as *mut std::ffi::c_void,
        1,
    );

    let (tx, rx) = mpsc::channel();
    let handle_address = handle as usize;
    let phase_thread = thread::spawn(move || {
        let result = swarm_gp_phase(handle_address as *mut SwarmStepHandle);
        let _ = tx.send(result);
    });
    let result = rx
        .recv_timeout(Duration::from_secs(8))
        .expect("Structure callback stop-and-wait must not self-deadlock");
    phase_thread.join().unwrap();
    assert_eq!(result, 0);
    assert_eq!(swarm_is_stop_requested(handle), 1);

    let callbacks = callback_solutions(&capture);
    assert_eq!(
        callbacks.len(),
        1,
        "cancelled Structure must not fake P2-P5 completion"
    );
    assert_eq!(callbacks[0].0, 0, "the exact Init baseline must be first");
    assert_complete_global_layout(&callbacks[0].1, 0..18);
    assert_place_group_pose(&callbacks[0].1, (12.0, 2.0));
    assert_eq!(callbacks[0].1["loss"], 0.0);
    assert!(
        callbacks[0].1["collision_status"]
            .as_array()
            .unwrap()
            .iter()
            .all(|item| item["collision"] == false)
    );
    let cancelled_solution = solution(handle);
    assert_complete_global_layout(&cancelled_solution, 0..18);
    assert_place_group_pose(&cancelled_solution, (12.0, 2.0));
    let report_ptr = swarm_get_structure_report_json(handle);
    assert!(!report_ptr.is_null());
    let report_json = unsafe { CStr::from_ptr(report_ptr) }
        .to_str()
        .unwrap()
        .to_owned();
    swarm_free_string(report_ptr);
    let report: serde_json::Value = serde_json::from_str(&report_json).unwrap();
    assert!(
        report["lanes"]
            .as_array()
            .unwrap()
            .iter()
            .all(|lane| lane["report"]["cancelled"] == true)
    );
    swarm_free(handle);
}

#[test]
#[cfg(feature = "adaptive")]
fn all_items_native_listener_can_stop_and_wait_with_a_rigid_place_group() {
    let mut config: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut config);
    config.rng_seed = 118;
    config.exploration.time_limit_secs = 5.0;
    config.structure_first.progress_callback_interval_ms = 500;

    let mut input_items = vec![
        hinted_item(0, false, 0.01, 0.01),
        hinted_item(1, false, 12.01, 2.01),
    ];
    input_items.extend((2..30).map(|id| item(id, false, 0.0)));
    let handle = init_with_config(
        serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items", "group_workers":2, "quantum_ms":5,
                       "locked_prefix_gap_mm":0.0},
            "instance":{"name":"ffi-rigid-native-stop", "strip_height":48.0,
                        "items":input_items},
            "regions":[],
            "place_groups":[{"id":53, "member_item_ids":[0,1]}]
        }),
        &config,
    );
    assert!(!handle.is_null());

    let capture = Arc::new(JsonCallbackCapture {
        handle: handle as usize,
        stop_mode: StopMode::RequestAndWait,
        callback_count: AtomicUsize::new(0),
        payloads: Mutex::new(vec![]),
    });
    swarm_set_callback(
        handle,
        Some(capture_json_and_optionally_stop),
        Arc::as_ptr(&capture) as *mut std::ffi::c_void,
        1,
    );

    let started = Instant::now();
    let (tx, rx) = mpsc::channel();
    let handle_address = handle as usize;
    let phase_thread = thread::spawn(move || {
        let result = swarm_alg_phase(handle_address as *mut SwarmStepHandle);
        let _ = tx.send(result);
    });
    let result = rx
        .recv_timeout(Duration::from_secs(8))
        .expect("native all_items callback stop-and-wait must not self-deadlock");
    phase_thread.join().unwrap();
    assert_eq!(result, 0);
    assert!(
        started.elapsed() < Duration::from_secs(4),
        "callback stop must terminate the five-second Explore budget promptly"
    );
    assert_eq!(swarm_is_stop_requested(handle), 1);

    let callbacks = callback_solutions(&capture);
    assert!(!callbacks.is_empty());
    for (_, snapshot) in callbacks {
        assert_complete_global_layout(&snapshot, 0..30);
        assert_place_group_pose(&snapshot, (12.0, 2.0));
    }
    swarm_free(handle);
}

#[test]
fn all_items_explore_and_compress_stream_internal_progress() {
    let mut config: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut config);
    config.rng_seed = 119;
    config.exploration.time_limit_secs = 1.25;
    config.compression.time_limit_secs = 1.25;
    // Small shrink attempts keep producing feasible native solver events over
    // the phase instead of exhausting all easy compactions before the 500 ms
    // callback floor.
    config.compression.shrink_range_min = 0.0005;
    config.compression.shrink_range_max = 0.00005;
    config.compression.shrink_decay_strategy = 0;
    config.structure_first.progress_callback_interval_ms = 500;

    let input_items = (0..24).map(|id| item(id, false, 0.0)).collect::<Vec<_>>();
    let handle = init_with_config(
        serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items", "group_workers":1, "quantum_ms":5,
                       "locked_prefix_gap_mm":0.0},
            "instance":{"name":"ffi-all-items-progress", "strip_height":43.0,
                        "items":input_items},
            "regions":[],
            "place_groups":[]
        }),
        &config,
    );
    assert!(!handle.is_null());

    let capture = Arc::new(JsonCallbackCapture {
        handle: handle as usize,
        stop_mode: StopMode::None,
        callback_count: AtomicUsize::new(0),
        payloads: Mutex::new(vec![]),
    });
    swarm_set_callback(
        handle,
        Some(capture_json_and_optionally_stop),
        Arc::as_ptr(&capture) as *mut std::ffi::c_void,
        1,
    );

    assert_eq!(swarm_alg_phase(handle), 0);
    let explore_callbacks = callback_solutions(&capture);
    assert!(
        explore_callbacks.len() >= 2,
        "Explore must emit a native in-phase preview plus boundary: {explore_callbacks:?}"
    );
    assert!(
        explore_callbacks[..explore_callbacks.len() - 1]
            .iter()
            .all(|(kind, _)| *kind == 2),
        "in-phase Explore previews must report ExplImproving(2): {explore_callbacks:?}"
    );
    assert_eq!(explore_callbacks.last().unwrap().0, 4);
    for (_, value) in &explore_callbacks {
        assert_complete_global_layout(value, 0..24);
    }

    capture.payloads.lock().unwrap().clear();
    assert_eq!(swarm_cp_phase(handle), 0);
    let compress_callbacks = callback_solutions(&capture);
    assert!(
        compress_callbacks.len() >= 2,
        "Compress must emit a native in-phase preview plus boundary: {compress_callbacks:?}"
    );
    assert!(
        compress_callbacks[..compress_callbacks.len() - 1]
            .iter()
            .all(|(kind, _)| *kind == 3)
    );
    assert_eq!(compress_callbacks.last().unwrap().0, 4);
    for (_, value) in &compress_callbacks {
        assert_complete_global_layout(value, 0..24);
    }
    swarm_free(handle);
}

#[test]
#[cfg(feature = "structure-first")]
fn all_items_structure_streams_p2_p3_previews_and_p5_boundary() {
    let mut config: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut config);
    config.rng_seed = 127;
    config.structure_first.enabled = 1;
    config.structure_first.motif_enabled = 0;
    config.structure_first.construction_starts = 1;
    config.structure_first.p1_time_limit_secs = 0.02;
    config.structure_first.p2_time_limit_secs = 1.25;
    config.structure_first.p2_shrink_range_min = 0.0005;
    config.structure_first.p2_shrink_range_max = 0.00005;
    config.structure_first.p2_failure_decay = 0.98;
    config.structure_first.p3_time_limit_secs = 1.25;
    config.structure_first.p4_cycles = 0;
    config.polish.time_limit_secs = 1.25;
    config.polish.n_rounds = 100;
    config.structure_first.progress_callback_interval_ms = 500;

    let input_items = (0..18).map(|id| item(id, false, 0.0)).collect::<Vec<_>>();
    let handle = init_with_config(
        serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"all_items", "group_workers":1, "quantum_ms":5,
                       "locked_prefix_gap_mm":0.0},
            "instance":{"name":"ffi-all-items-structure-progress", "strip_height":40.0,
                        "items":input_items},
            "regions":[],
            "place_groups":[]
        }),
        &config,
    );
    assert!(!handle.is_null());

    let capture = Arc::new(JsonCallbackCapture {
        handle: handle as usize,
        stop_mode: StopMode::None,
        callback_count: AtomicUsize::new(0),
        payloads: Mutex::new(vec![]),
    });
    swarm_set_callback(
        handle,
        Some(capture_json_and_optionally_stop),
        Arc::as_ptr(&capture) as *mut std::ffi::c_void,
        1,
    );
    assert_eq!(swarm_gp_phase(handle), 0);

    let callbacks = callback_solutions(&capture);
    assert!(!callbacks.is_empty());
    let types = callbacks.iter().map(|(kind, _)| *kind).collect::<Vec<_>>();
    assert_eq!(types.first(), Some(&0), "Init baseline must be first");
    assert_eq!(types.last(), Some(&4), "P5 boundary must be final");
    for (_, value) in &callbacks {
        assert_complete_global_layout(value, 0..18);
    }

    let p2_begin = types
        .iter()
        .position(|kind| *kind == 3)
        .expect("P2 callback missing");
    let p3_begin = types[p2_begin..]
        .iter()
        .position(|kind| *kind == 0)
        .map(|offset| p2_begin + offset)
        .expect("P3 callback missing");
    assert!(
        types[p2_begin..p3_begin]
            .iter()
            .filter(|kind| **kind == 3)
            .count()
            >= 2,
        "long P2 must emit an internal preview and its stage boundary: {types:?}"
    );

    let p4_begin = types[p3_begin..]
        .iter()
        .position(|kind| *kind == 2)
        .map(|offset| p3_begin + offset)
        .expect("P4 callback missing");
    assert!(
        types[p3_begin..p4_begin]
            .iter()
            .filter(|kind| **kind == 0)
            .count()
            >= 2,
        "long P3 must emit an internal preview and its stage boundary: {types:?}"
    );

    // P5 remains event-driven: it may emit type-3 previews when Polish finds
    // an improvement, but a layout that is already compact may legitimately
    // have no CmprFeas event.  Its unthrottled Final boundary is mandatory.
    assert!(
        types[p4_begin + 1..types.len() - 1]
            .iter()
            .all(|kind| *kind == 3)
    );
    swarm_free(handle);
}

#[test]
fn invalid_frozen_prefix_is_rejected_during_ffi_initialization() {
    let mut off_angle = hinted_item(0, true, 10.0, 10.0);
    off_angle["allowed_orientations"] = serde_json::json!([0.0]);
    off_angle["placed_hint"]["rot_angle_in_rad"] = serde_json::json!(std::f32::consts::FRAC_PI_2);
    let cases = vec![(
        "frozen item outside allowed rotations",
        vec![off_angle, item(1, false, 0.0)],
    )];

    for (case, items) in cases {
        let handle = init(serde_json::json!({
            "schema_version":2,
            "nesting":{"mode":"size_regions", "group_workers":1, "quantum_ms":5,
                       "locked_prefix_gap_mm":3.0},
            "instance":{"name":case, "strip_height":25.0, "items":items},
            "regions":[
                {"id":"all", "kind":"size_group", "item_ids":[0,1], "gap_after_mm":0.0}
            ],
            "place_groups":[]
        }));
        assert!(
            handle.is_null(),
            "{case} must fail before a handle is published"
        );
        assert_eq!(swarm_get_last_error_code(), 2001, "{case}");
        let error_ptr = swarm_get_last_error_message();
        assert!(!error_ptr.is_null(), "{case}");
        let error = unsafe { CStr::from_ptr(error_ptr) }
            .to_string_lossy()
            .into_owned();
        swarm_free_string(error_ptr);
        assert!(
            error.contains("invalid frozen prefix"),
            "{case} should identify the invalid frozen prefix: {error}"
        );
    }
}
