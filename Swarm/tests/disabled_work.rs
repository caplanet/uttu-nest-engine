//! Public phase-boundary regressions for runtime-disabled work.
use cpswcore::{ffi::*, ffi_step::*};
use std::{
    ffi::{CStr, CString},
    sync::Mutex,
};

fn config(enabled: i32) -> CSwarmConfig {
    let mut c: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut c);
    c.rng_seed = 71;
    c.compression.time_limit_secs = 0.;
    c.polish.time_limit_secs = 0.1;
    c.polish.enabled = enabled;
    c.polish.slide_enabled = 0;
    c.polish.hole_fill_enabled = 0;
    c.polish.lp_enabled = 0;
    c.reclaim.enabled = 0;
    c
}
fn fixture(mode: u8) -> serde_json::Value {
    let items: Vec<_> = (0..4)
        .map(|id| {
            serde_json::json!({"id":id,"OrgID":100+id,"demand":1,"allowed_orientations":[0],
        "shape":{"type":"rectangle","data":{"x_min":0,"y_min":0,"width":4,"height":4}},
        "placed_hint":{"item_id":id,"is_freeze":if id==0 {1} else {0},"tr_x":2+id*7,"tr_y":2,
            "rot_angle_in_rad":0,"stripe_interval_x":0,"stripe_interval_y":0}})
        })
        .collect();
    let instance =
        serde_json::json!({"name":"empty-polish-boundary","strip_height":12,"items":items});
    match mode {
        0 => instance,
        1 => serde_json::json!({"schema_version":2,"instance":instance,
            "nesting":{"mode":"all_items","group_workers":1,"quantum_ms":5,"locked_prefix_gap_mm":0},
            "regions":[],"place_groups":[{"id":7,"member_item_ids":[0,1]}]}),
        _ => serde_json::json!({"schema_version":2,"instance":instance,
            "nesting":{"mode":"size_regions","group_workers":2,"quantum_ms":5,"locked_prefix_gap_mm":0},
            "regions":[{"id":"a","kind":"size_bundle","item_ids":[0,1],"gap_after_mm":0},
                       {"id":"b","kind":"size_bundle","item_ids":[2,3],"gap_after_mm":0}],"place_groups":[]}),
    }
}
fn read(h: *mut SwarmStepHandle) -> serde_json::Value {
    let p = swarm_get_solution_json(h);
    assert!(!p.is_null());
    let text = unsafe { CStr::from_ptr(p) }.to_string_lossy().into_owned();
    swarm_free_string(p);
    serde_json::from_str(&text).unwrap()
}
fn state(v: &serde_json::Value) -> (serde_json::Value, serde_json::Value) {
    let s = v.get("solution").unwrap_or(v);
    assert_eq!(s["layout"]["placed_items"].as_array().unwrap().len(), 4);
    (
        s["strip_width"].clone(),
        s["layout"]["placed_items"].clone(),
    )
}
struct Capture {
    handle: usize,
    stop: bool,
    events: Mutex<Vec<(i32, String)>>,
}
extern "C" fn capture(
    kind: i32,
    _width: f32,
    _density: f32,
    json: *const std::os::raw::c_char,
    data: *mut std::ffi::c_void,
) {
    let c = unsafe { &*(data as *const Capture) };
    if !json.is_null() {
        c.events.lock().unwrap().push((
            kind,
            unsafe { CStr::from_ptr(json) }
                .to_string_lossy()
                .into_owned(),
        ));
    }
    if kind == 4 && c.stop {
        swarm_request_stop(c.handle as *mut SwarmStepHandle);
    }
}

#[test]
fn disabled_and_empty_polish_preserve_final_json_constraints_and_phase() {
    for mode in 0..=2 {
        for enabled in [0, 1] {
            for stop in [false, true] {
                // Rigid groups support LP now; keep every pass explicitly off
                // to exercise the empty-phase contract in all three modes.
                let cfg = config(enabled);
                let input = CString::new(fixture(mode).to_string()).unwrap();
                let h = swarm_init_from_json(input.as_ptr(), &cfg, std::ptr::null(), 0);
                assert!(!h.is_null());
                // Public phases can run directly from the current certified layout.
                let initial = state(&read(h));
                assert_eq!(swarm_refine_phase(h), 0);
                assert_eq!(state(&read(h)), initial);
                assert_eq!(swarm_cp_phase(h), 0);
                let before = state(&read(h));
                let c = Capture {
                    handle: h as usize,
                    stop,
                    events: Mutex::new(Vec::new()),
                };
                swarm_set_callback(h, Some(capture), &c as *const Capture as *mut _, 1);
                assert_eq!(swarm_refine_phase(h), 0);
                assert_eq!(
                    state(&read(h)),
                    before,
                    "mode={mode}, enabled={enabled}, stop={stop}"
                );
                let events = c.events.lock().unwrap();
                let (kind, json) = events.last().expect("mandatory Final JSON missing");
                assert_eq!(*kind, 4);
                assert_eq!(state(&serde_json::from_str(json).unwrap()), before);
                drop(events);
                // A Final callback may latch stop; otherwise Polish is repeatable.
                assert_eq!(swarm_refine_phase(h), if stop { -1 } else { 0 });
                assert_eq!(state(&read(h)), before);
                swarm_free(h);
            }
        }
    }
}

#[test]
fn disabling_local_polish_does_not_disable_reclaim() {
    let mut cfg = config(0);
    cfg.reclaim.enabled = 1;
    cfg.reclaim.max_cycles = 1;
    cfg.reclaim.min_slice_ms = 1;
    // One Reclaim cycle (max_cycles = 1) bounds the work; the budget only has
    // to be long enough for that cycle on a loaded test host (0.1 s was not).
    cfg.polish.time_limit_secs = 1.0;
    let input = CString::new(fixture(0).to_string()).unwrap();
    let h = swarm_init_from_json(input.as_ptr(), &cfg, std::ptr::null(), 0);
    assert!(!h.is_null());
    assert_eq!(swarm_cp_phase(h), 0);
    let before = state(&read(h));
    assert_eq!(swarm_refine_phase(h), 0);
    let after = state(&read(h));
    assert!(
        after.0.as_f64().unwrap() < before.0.as_f64().unwrap(),
        "Reclaim must still compact this spaced fixture"
    );
    swarm_free(h);
}

#[test]
fn active_polish_ffi_stays_within_budget_on_unchanged_frozen_placements() {
    use std::time::{Duration, Instant};
    for mode in 0..=1 {
        let mut cfg = config(1);
        cfg.polish.time_limit_secs = 0.08;
        cfg.polish.n_rounds = 1;
        cfg.polish.slide_enabled = 1;
        let mut value = fixture(mode);
        let items = if mode == 0 {
            &mut value["items"]
        } else {
            &mut value["instance"]["items"]
        };
        for item in items.as_array_mut().unwrap() {
            item["placed_hint"]["is_freeze"] = serde_json::json!(1);
        }
        let input = CString::new(value.to_string()).unwrap();
        // Legacy input reads hints through its C ABI; schema v2 reads JSON hints.
        let hints: Vec<_> = (0..4)
            .map(|item_id| CPlacedHint {
                item_id,
                is_freeze: 1,
                tr_x: (2 + item_id * 7) as f32,
                tr_y: 2.,
                rot_angle_in_rad: 0.,
                stripe_interval_x: 0.,
                stripe_interval_y: 0.,
            })
            .collect();
        let h = swarm_init_from_json(
            input.as_ptr(),
            &cfg,
            if mode == 0 {
                hints.as_ptr()
            } else {
                std::ptr::null()
            },
            if mode == 0 { hints.len() as i32 } else { 0 },
        );
        assert!(!h.is_null());
        let before = state(&read(h));
        let started = Instant::now();
        assert_eq!(swarm_refine_phase(h), 0);
        let elapsed = started.elapsed();
        // Polish 는 고정점이 증명되면 예산을 기다리지 않고 끝난다. 남은 시간은
        // Reclaim 이 쓰거나 그대로 반환되므로, 상한만 확인한다.
        assert!(
            elapsed < Duration::from_secs(1),
            "mode={mode} did not bound its budget: {elapsed:?}"
        );
        assert_eq!(
            state(&read(h)).1,
            before.1,
            "frozen placements changed in mode={mode}"
        );
        swarm_free(h);
    }
}
