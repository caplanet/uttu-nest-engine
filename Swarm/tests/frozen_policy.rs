//! Public lock exceptions must agree at Init, every phase, diagnostics and export.
use cpswcore::ffi::{CSwarmConfig, swarm_free_string, swarm_get_default_config};
use cpswcore::ffi_step::*;
use std::ffi::{CStr, CString, c_char, c_void};
use std::sync::Mutex;

extern "C" fn callback(_: i32, _: f32, _: f32, json: *const c_char, user: *mut c_void) {
    if !json.is_null() {
        let output = unsafe { &*(user as *const Mutex<Vec<String>>) };
        if let Ok(mut output) = output.lock() {
            output.push(
                unsafe { CStr::from_ptr(json) }
                    .to_string_lossy()
                    .into_owned(),
            );
        }
    }
}

fn read_json(ptr: *mut c_char) -> serde_json::Value {
    assert!(!ptr.is_null(), "{}", last_error());
    let text = unsafe { CStr::from_ptr(ptr) }
        .to_string_lossy()
        .into_owned();
    swarm_free_string(ptr);
    serde_json::from_str(&text).unwrap()
}

fn last_error() -> String {
    let ptr = swarm_get_last_error_message();
    if ptr.is_null() {
        return String::new();
    }
    let s = unsafe { CStr::from_ptr(ptr) }
        .to_string_lossy()
        .into_owned();
    swarm_free_string(ptr);
    s
}

fn assert_locks(solution: &serde_json::Value, xy: (f64, f64), lock_count: usize) {
    let placed = solution["layout"]["placed_items"].as_array().unwrap();
    assert_eq!(placed.len(), 3);
    for id in 0..lock_count as u64 {
        let p = placed
            .iter()
            .find(|p| p["item_id"].as_u64() == Some(id))
            .unwrap();
        let t = &p["transformation"]["translation"];
        assert!((t[0].as_f64().unwrap() - xy.0).abs() < 0.001, "{p}");
        assert!((t[1].as_f64().unwrap() - xy.1).abs() < 0.001, "{p}");
        assert_eq!(p["transformation"]["rotation"].as_f64().unwrap(), 0.0);
    }
}

fn run(mode: &str, xy: (f64, f64), structure: bool, grouped_locks: bool) {
    run_frozen_count(mode, xy, structure, grouped_locks, 2)
}

fn run_frozen_count(
    mode: &str,
    xy: (f64, f64),
    structure: bool,
    grouped_locks: bool,
    lock_count: usize,
) {
    eprintln!("case {mode} {xy:?} structure={structure}");
    let items=(0..3).map(|id| {
        let mut p=serde_json::json!({"id":id,"demand":1,"allowed_orientations":[0.0],
            "shape":{"type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}}});
        if id<lock_count {p["placed_hint"]=serde_json::json!({"item_id":id,"is_freeze":1,
            "tr_x":xy.0,"tr_y":xy.1,"rot_angle_in_rad":0.0,"stripe_interval_y":0.0,"stripe_interval_x":0.0});}
        p
    }).collect::<Vec<_>>();
    let instance = serde_json::json!({"name":"frozen-policy","strip_height":20.0,"items":items});
    let json = if mode == "legacy" {
        instance
    } else {
        serde_json::json!({"schema_version":2,
        "nesting":{"mode":mode,"group_workers":2,"quantum_ms":5,"locked_prefix_gap_mm":0.0},
        "instance":instance,"regions":if mode=="size_regions" {serde_json::json!([
            {"id":"all","kind":"size_group","item_ids":[0,1,2],"gap_after_mm":0.0}])} else {serde_json::json!([])},
        "place_groups":if grouped_locks {serde_json::json!([{"id":7,"member_item_ids":[0,1]}])} else {serde_json::json!([])}})
    };
    let mut cfg: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut cfg);
    // Test-only budgets; production configuration/defaults remain unchanged.
    cfg.rng_seed = 17;
    cfg.exploration.time_limit_secs = 0.03;
    cfg.compression.time_limit_secs = 0.03;
    cfg.polish.time_limit_secs = 0.03;
    cfg.structure_first.enabled = 1;
    cfg.structure_first.p1_time_limit_secs = 0.05;
    cfg.structure_first.p2_time_limit_secs = 0.03;
    cfg.structure_first.p3_time_limit_secs = 0.03;
    cfg.structure_first.p4_cycles = 0;
    cfg.initial_placement.enabled = 1;
    cfg.initial_placement.time_limit_secs = 0.1;
    let text = CString::new(json.to_string()).unwrap();
    let hints = (0..lock_count as i32)
        .map(|item_id| CPlacedHint {
            item_id,
            is_freeze: 1,
            tr_x: xy.0 as f32,
            tr_y: xy.1 as f32,
            rot_angle_in_rad: 0.0,
            stripe_interval_x: 0.0,
            stripe_interval_y: 0.0,
        })
        .collect::<Vec<_>>();
    let handle = swarm_init_from_json(
        text.as_ptr(),
        &cfg,
        if mode == "legacy" {
            hints.as_ptr()
        } else {
            std::ptr::null()
        },
        if mode == "legacy" {
            lock_count as i32
        } else {
            0
        },
    );
    assert!(!handle.is_null(), "{mode} {xy:?}: {}", last_error());
    assert_locks(
        &read_json(swarm_get_solution_json(handle)),
        xy,
        lock_count,
    );
    let capture = Mutex::new(Vec::<String>::new());
    swarm_set_callback(
        handle,
        Some(callback),
        &capture as *const _ as *mut c_void,
        1,
    );
    if mode == "legacy" && xy == (5.0, 5.0) && !structure {
        assert_eq!(swarm_init_phase(handle), 0, "{}", last_error());
        assert_locks(
            &read_json(swarm_get_solution_json(handle)),
            xy,
            lock_count,
        );
    }
    if structure {
        assert_eq!(swarm_gp_phase(handle), 0, "{}", last_error());
    } else {
        for phase in [
            swarm_alg_phase,
            swarm_cp_phase,
            swarm_refine_phase,
        ] {
            assert_eq!(phase(handle), 0, "{mode} {xy:?}: {}", last_error());
            assert_locks(
                &read_json(swarm_get_solution_json(handle)),
                xy,
                lock_count,
            );
            let status = read_json(swarm_get_collision_status_json(handle));
            assert!(
                status
                    .as_array()
                    .unwrap()
                    .iter()
                    .all(|s| s["collision"] == false),
                "{status}"
            );
        }
    }
    assert_locks(
        &read_json(swarm_get_solution_json(handle)),
        xy,
        lock_count,
    );
    let mut count = 0;
    let statuses = swarm_get_collision_status(handle, &mut count);
    assert!(!statuses.is_null());
    assert_eq!(count, 3);
    assert!(
        unsafe { std::slice::from_raw_parts(statuses, count) }
            .iter()
            .all(|s| s.collision == 0)
    );
    swarm_free_collision_status(statuses, count);
    let bounds = [0, 1].map(|item_id| CItemBounds {
        item_id,
        min_x: xy.0 as f32,
        min_y: xy.1 as f32,
        max_x: xy.0 as f32 + 4.0,
        max_y: xy.1 as f32 + 4.0,
    });
    let conflicts = swarm_find_bound_collisions(handle, bounds.as_ptr(), 2);
    assert!(conflicts.is_null());
    let payloads = capture.lock().unwrap();
    assert!(!payloads.is_empty());
    for text in payloads.iter() {
        let value: serde_json::Value = serde_json::from_str(text).unwrap();
        assert_locks(&value["solution"], xy, lock_count);
        assert_eq!(value["loss"].as_f64(), Some(0.0), "{value}");
        assert!(
            value["collision_status"]
                .as_array()
                .unwrap()
                .iter()
                .all(|s| s["collision"] == false)
        );
    }
    swarm_free(handle);
}

#[test]
fn frozen_policy_all_public_phases() {
    for mode in ["legacy", "all_items", "size_regions"] {
        for xy in [(5.0, 5.0), (-1000.0, -1000.0), (5.0, 1000.0), (1000.0, 5.0)] {
            run(mode, xy, false, false);
        }
    }
}

#[test]
fn frozen_policy_overlapping_locked_place_group() {
    run("all_items", (-2.0, 18.0), false, true);
}

#[cfg(feature = "structure-first")]
#[test]
fn frozen_policy_structure_phases() {
    for mode in ["legacy", "all_items", "size_regions"] {
        run(mode, (-1000.0, -1000.0), true, false);
        run(mode, (1000.0, 5.0), true, false);
    }
}

#[test]
fn frozen_policy_all_items_locked() {
    for mode in ["legacy", "all_items", "size_regions"] {
        run_frozen_count(mode, (-1000.0, -1000.0), false, false, 3);
        run_frozen_count(mode, (1000.0, 5.0), false, false, 3);
    }
}
