//! Real phase boundaries under the requested 0.08 mm / 0.2 mm tolerances.
use cpswcore::{ffi::*, ffi_step::*};
use std::ffi::{CStr, CString, c_char};

fn read(ptr: *mut c_char) -> serde_json::Value {
    assert!(!ptr.is_null());
    let value = serde_json::from_str(unsafe { CStr::from_ptr(ptr) }.to_str().unwrap()).unwrap();
    swarm_free_string(ptr);
    value
}

#[test]
fn explore_and_compress_with_few_workers_keep_certified_complete_results() {
    let input = CString::new(include_str!("../data/input/fu.json")).unwrap();
    for workers in [1, 2, 4] {
        for seed in [17, 42] {
            let mut config: CSwarmConfig = unsafe { std::mem::zeroed() };
            swarm_get_default_config(&mut config);
            config.rng_seed = seed;
            config.exploration.time_limit_secs = 0.3;
            config.compression.time_limit_secs = 0.3;
            config.exploration.separator.n_workers = workers;
            config.compression.separator.n_workers = workers;
            config.fabric_tolerance.piece_overlap_mm = 0.08;
            config.fabric_edge.margin_mm = 0.2;
            // Old presets must not resurrect the removed operators.
            config.exploration.rnr_period = 1;
            config.exploration.separator.swap_period = 1;
            config.compression.separator.swap_period = 1;
            config.repair.direction_enabled = 1;
            config.repair.cluster_lp_enabled = 1;
            config.repair.chain_enabled = 1;
            let handle = swarm_init_from_json(input.as_ptr(), &config, std::ptr::null(), 0);
            assert!(!handle.is_null());
            let initial = read(swarm_get_solution_json(handle));
            let count = initial["layout"]["placed_items"].as_array().unwrap().len();
            let mut width = initial["strip_width"].as_f64().unwrap();
            for phase in [swarm_alg_phase, swarm_cp_phase] {
                assert_eq!(phase(handle), 0, "workers={workers}, seed={seed}");
                let solution = read(swarm_get_solution_json(handle));
                assert_eq!(
                    solution["layout"]["placed_items"].as_array().unwrap().len(),
                    count
                );
                let next_width = solution["strip_width"].as_f64().unwrap();
                assert!(next_width <= width + 0.001);
                width = next_width;
                let status = read(swarm_get_collision_status_json(handle));
                assert!(
                    status
                        .as_array()
                        .unwrap()
                        .iter()
                        .all(|s| s["collision"] == false),
                    "workers={workers}, seed={seed}: {status}"
                );
            }
            swarm_free(handle);
        }
    }
}
