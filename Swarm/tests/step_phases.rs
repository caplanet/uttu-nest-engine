//! 단계별 FFI API의 단계 전이 검증.
//!
//! 실행:  cargo test --release --features capmods --test step_phases -- --nocapture
//!
//! 특히 **탐색 단계를 건너뛰고 LBF 초기해에서 바로 압축**할 수 있는지를 확인한다
//! (LBF + Compress). 탐색이 시간 예산의 대부분을 쓰므로 이 경로가 필요하다.

use cpswcore::ffi::{CSwarmConfig, swarm_free_string, swarm_get_default_config};
use cpswcore::ffi_step::*;
use std::ffi::{CStr, CString};

/// 각 단계를 짧게 제한한 config (테스트가 몇 분씩 걸리지 않도록).
fn quick_config() -> CSwarmConfig {
    let mut cfg: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut cfg);
    cfg.rng_seed = 1; // 재현 가능하게
    cfg.exploration.time_limit_secs = 3.0;
    cfg.compression.time_limit_secs = 3.0;
    cfg.polish.time_limit_secs = 2.0;
    cfg
}

fn init_handle(name: &str, cfg: &CSwarmConfig) -> *mut SwarmStepHandle {
    let path =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(format!("data/input/{name}.json"));
    let json = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", path.display()));
    let c_json = CString::new(json).expect("json contains NUL");
    let h = swarm_init_from_json(c_json.as_ptr(), cfg, std::ptr::null(), 0);
    assert!(
        !h.is_null(),
        "swarm_init_from_json returned NULL for {name}"
    );
    h
}

/// 현재 해의 strip 폭을 해 JSON에서 읽는다.
fn solution_width(h: *mut SwarmStepHandle) -> f32 {
    let ptr = swarm_get_solution_json(h);
    assert!(!ptr.is_null(), "no solution available");
    let json = unsafe { CStr::from_ptr(ptr) }.to_str().unwrap().to_owned();
    swarm_free_string(ptr);

    let v: serde_json::Value = serde_json::from_str(&json).expect("solution json must parse");
    // 해 JSON 스키마가 버전에 따라 다를 수 있어 폭 필드를 유연하게 찾는다.
    fn find_width(v: &serde_json::Value) -> Option<f32> {
        match v {
            serde_json::Value::Object(m) => {
                for key in ["strip_width", "width", "container_width"] {
                    if let Some(w) = m.get(key).and_then(|x| x.as_f64()) {
                        return Some(w as f32);
                    }
                }
                m.values().find_map(find_width)
            }
            serde_json::Value::Array(a) => a.iter().find_map(find_width),
            _ => None,
        }
    }
    find_width(&v).expect("solution json must contain a strip width")
}

/// 탐색을 건너뛰고 초기화 직후 곧바로 압축할 수 있어야 한다 (LBF + Compress).
#[test]
fn compress_runs_without_exploration() {
    let cfg = quick_config();
    let h = init_handle("shirts", &cfg);

    let width_after_lbf = solution_width(h);
    // 탐색 없이 바로 압축
    assert_eq!(
        swarm_cp_phase(h),
        0,
        "compress phase must succeed right after init (LBF + Compress)"
    );

    let width_after_cmpr = solution_width(h);
    println!("LBF {width_after_lbf:.3} -> Compress {width_after_cmpr:.3}");
    assert!(
        width_after_cmpr <= width_after_lbf + 1e-3,
        "compression must not widen the strip: {width_after_lbf} -> {width_after_cmpr}"
    );

    // 압축을 마쳤으므로 후처리도 이어서 가능해야 한다
    assert_eq!(
        swarm_refine_phase(h),
        0,
        "polish must be callable after a compress that skipped exploration"
    );

    swarm_free(h);
}

/// 기존 경로(LBF + Exploration + Compress)도 그대로 동작해야 한다.
#[test]
fn explore_then_compress_still_works() {
    let cfg = quick_config();
    let h = init_handle("shirts", &cfg);

    assert_eq!(swarm_alg_phase(h), 0, "explore phase must succeed");
    let width_after_expl = solution_width(h);
    assert_eq!(
        swarm_cp_phase(h),
        0,
        "compress phase must succeed after explore"
    );
    let width_after_cmpr = solution_width(h);

    println!("Explore {width_after_expl:.3} -> Compress {width_after_cmpr:.3}");
    assert!(
        width_after_cmpr <= width_after_expl + 1e-3,
        "compression must not widen the strip: {width_after_expl} -> {width_after_cmpr}"
    );

    swarm_free(h);
}

/// 탐색은 이어서 여러 번 호출할 수 있어야 한다 (Explore → Explore → Explore → Compress → Polish).
/// 이어서 돈 탐색은 직전 해에서 출발하므로 폭이 늘어나면 안 된다.
#[test]
fn explore_can_be_repeated_before_compress() {
    let cfg = quick_config();
    let h = init_handle("shirts", &cfg);

    let mut width = solution_width(h);
    for round in 1..=3 {
        assert_eq!(
            swarm_alg_phase(h),
            0,
            "explore call {round} must succeed"
        );
        let next = solution_width(h);
        println!("Explore #{round}: {width:.3} -> {next:.3}");
        assert!(
            next <= width + 1e-3,
            "explore call {round} widened the strip: {width} -> {next}"
        );
        width = next;
    }

    assert_eq!(
        swarm_cp_phase(h),
        0,
        "compress must succeed after repeated explores"
    );
    assert!(solution_width(h) <= width + 1e-3);
    assert_eq!(
        swarm_refine_phase(h),
        0,
        "polish must succeed after compress"
    );

    swarm_free(h);
}

/// 단계 순서와 횟수는 자유다 (Explore → Compress → Polish → Explore → Compress → Polish,
/// 압축 없는 Polish 포함). 어느 호출도 폭을 늘리지 않는다.
#[test]
fn phases_run_in_any_order_and_never_widen() {
    let cfg = quick_config();
    let h = init_handle("shirts", &cfg);

    // 압축을 건너뛴 후처리도 현재 해(LBF 초기해)에서 그대로 돈다.
    let mut width = solution_width(h);
    let sequence: [(&str, extern "C" fn(*mut SwarmStepHandle) -> i32); 7] = [
        ("polish", swarm_refine_phase),
        ("explore", swarm_alg_phase),
        ("compress", swarm_cp_phase),
        ("polish", swarm_refine_phase),
        ("explore", swarm_alg_phase),
        ("compress", swarm_cp_phase),
        ("compress", swarm_cp_phase),
    ];
    for (i, (name, call)) in sequence.into_iter().enumerate() {
        assert_eq!(call(h), 0, "step {i} ({name}) must succeed");
        let next = solution_width(h);
        println!("step {i} {name}: {width:.3} -> {next:.3}");
        assert!(
            next <= width + 1e-3,
            "step {i} ({name}) widened the strip: {width} -> {next}"
        );
        width = next;
    }

    swarm_free(h);
}
