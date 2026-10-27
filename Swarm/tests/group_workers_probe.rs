//! Opt-in probe: exploration progress with 1 vs 8 separator workers, with and
//! without the input's place groups (docs/GROUP_WORKERS_912_2026-10-15.md).
//!
//! ```text
//! set PROBE_INPUT=C:\path\to\grouped_input.json
//! cargo test --release --features "simd capmods adaptive structure-first" ^
//!     --test group_workers_probe -- --ignored --nocapture
//! ```
//!
//! `PROBE_SECS` (default 30), `PROBE_SEEDS` (default `1,2,3`) and
//! `PROBE_ONLY=groups|plain` narrow the run; `PROBE_GROUP_MODE=0|1` picks the
//! place-group mode (0 = multipolygon item, 1 = rigid member moves) and
//! `PROBE_WORKERS` (default `1,8`) the worker counts and `PROBE_OVERLAP_MM` the
//! piece overlap allowance. Wall-clock results are only meaningful on an
//! otherwise idle machine.
use cpswcore::ffi::{CSwarmConfig, swarm_free_string, swarm_get_default_config};
use cpswcore::ffi_step::*;
use std::ffi::{CStr, CString, c_char, c_void};
use std::sync::Mutex;
use std::time::Instant;

struct Trace {
    start: Instant,
    points: Mutex<Vec<(f64, f32)>>,
}

extern "C" fn on_solution(kind: i32, width: f32, _: f32, _: *const c_char, user: *mut c_void) {
    // 1 = infeasible, 4 = final; neither is a usable layout width.
    if kind == 1 || kind == 4 {
        return;
    }
    let trace = unsafe { &*(user as *const Trace) };
    trace
        .points
        .lock()
        .unwrap()
        .push((trace.start.elapsed().as_secs_f64(), width));
}

fn solution_width(handle: *mut SwarmStepHandle) -> f32 {
    let json = swarm_get_solution_json(handle);
    let value: serde_json::Value =
        serde_json::from_str(unsafe { CStr::from_ptr(json) }.to_str().unwrap()).unwrap();
    swarm_free_string(json);
    value["strip_width"].as_f64().unwrap() as f32
}

/// Returns the initial width, the improvement trace and the final width.
fn run(json: &str, workers: i32, seed: u64, secs: f32) -> (f32, Vec<(f64, f32)>, f32) {
    let mut config: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut config);
    config.rng_seed = seed;
    config.exploration.time_limit_secs = secs;
    config.exploration.separator.n_workers = workers;
    config.compression.separator.n_workers = workers;
    if let Ok(mode) = std::env::var("PROBE_GROUP_MODE") {
        config.place_group.mode = mode.trim().parse().unwrap();
    }
    if let Ok(overlap) = std::env::var("PROBE_OVERLAP_MM") {
        config.fabric_tolerance.piece_overlap_mm = overlap.trim().parse().unwrap();
    }
    let json = CString::new(json).unwrap();
    let handle = swarm_init_from_json(json.as_ptr(), &config, std::ptr::null(), 0);
    assert!(!handle.is_null(), "init failed: {}", swarm_get_last_error_code());
    let trace = Trace {
        start: Instant::now(),
        points: Mutex::new(Vec::new()),
    };
    swarm_set_callback(
        handle,
        Some(on_solution),
        &trace as *const Trace as *mut c_void,
        0,
    );
    let initial = solution_width(handle);
    assert_eq!(swarm_alg_phase(handle), 0);
    let last = solution_width(handle);
    // Public collision status of the exported result; every entry must be false.
    let status_ptr = swarm_get_collision_status_json(handle);
    if !status_ptr.is_null() {
        let status: serde_json::Value =
            serde_json::from_str(unsafe { CStr::from_ptr(status_ptr) }.to_str().unwrap()).unwrap();
        swarm_free_string(status_ptr);
        let entries = status.as_array().unwrap();
        let colliding = entries.iter().filter(|s| s["collision"] == true).count();
        println!("  STATUS entries={} colliding={colliding}", entries.len());
    }
    swarm_free(handle);
    (initial, trace.points.into_inner().unwrap(), last)
}

fn width_at(points: &[(f64, f32)], t: f64, initial: f32) -> f32 {
    points
        .iter()
        .filter(|p| p.0 <= t)
        .map(|p| p.1)
        .fold(initial, f32::min)
}

#[test]
#[ignore]
fn group_workers_probe() {
    let Ok(path) = std::env::var("PROBE_INPUT") else {
        eprintln!("PROBE_INPUT is not set; nothing to probe");
        return;
    };
    let secs: f32 = std::env::var("PROBE_SECS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(30.0);
    let seeds: Vec<u64> = std::env::var("PROBE_SEEDS")
        .unwrap_or_else(|_| "1,2,3".into())
        .split(',')
        .map(|s| s.trim().parse().unwrap())
        .collect();
    let only = std::env::var("PROBE_ONLY").ok();
    let worker_counts: Vec<i32> = std::env::var("PROBE_WORKERS")
        .unwrap_or_else(|_| "1,8".into())
        .split(',')
        .map(|s| s.trim().parse().unwrap())
        .collect();
    let mode_label = std::env::var("PROBE_GROUP_MODE").unwrap_or_else(|_| "default".into());
    let raw = std::fs::read_to_string(&path).unwrap();
    let grouped: serde_json::Value =
        serde_json::from_str(raw.trim_start_matches('\u{feff}')).unwrap();
    let mut plain = grouped.clone();
    plain["place_groups"] = serde_json::json!([]);
    let marks = [5.0, 10.0, 20.0];
    for (label, doc) in [("groups", &grouped), ("plain", &plain)] {
        if only.as_deref().is_some_and(|o| o != label) {
            continue;
        }
        for &workers in &worker_counts {
            for &seed in &seeds {
                let (initial, points, last) = run(&doc.to_string(), workers, seed, secs);
                let at: Vec<String> = marks
                    .iter()
                    .filter(|&&t| t < secs as f64)
                    .map(|&t| format!("{t}s {:.0}", width_at(&points, t, initial)))
                    .collect();
                println!(
                    "PROBE {label:<6} mode={mode_label} workers={workers} seed={seed} initial={initial:.0} {} end {last:.0}",
                    at.join(" / ")
                );
            }
        }
    }
}
