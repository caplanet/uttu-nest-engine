//! WPF 가 실제로 타는 **step API 경로**에서 LBF 배치 순서 모드를 검증한다.
//!
//! # 왜 별도 하네스인가
//!
//! `swarm-core/tests/lbf_order_feasibility.rs` 는 `LBFBuilder` 를 직접 부른다.
//! WPF 는 그러지 않는다 — `swarm_init_from_json` -> `swarm_init_phase`
//! -> `swarm_alg_phase` ... 로 가고, 그 경로에는 초기 해를 **인증**하는
//! 단계(`certify_legacy_candidate` / `solution_validation_error`)와 NFP-LBF
//! 시드·전략 스위치가 끼어 있다. 순서 모드가 그 조합에서만 깨질 수도 있다.
//!
//! 여기서는 각 모드로 Init 을 돌리고, 반환 코드와
//! `swarm_get_collision_status_json` 을 함께 본다.
//!
//! ```powershell
//! $env:SWARM_INPUT='C:\Caplanet\AiWorks\InputData\여성WSjk_비교용-G_New_SwarmV1.json'
//! cargo test --release --features "simd,capmods,adaptive,structure-first" `
//!   --test lbf_order_ffi -- --ignored --nocapture --test-threads=1
//! ```

use cpswcore::ffi::{CSwarmConfig, swarm_free_string, swarm_get_default_config};
use cpswcore::ffi_step::{
    SwarmStepHandle, swarm_free, swarm_get_collision_status_json,
    swarm_get_last_error_code, swarm_get_last_error_message, swarm_get_solution_json,
    swarm_alg_phase, swarm_init_from_json, swarm_init_phase,
};
use std::ffi::{CStr, CString};

fn take_json(ptr: *mut std::os::raw::c_char) -> serde_json::Value {
    assert!(!ptr.is_null(), "네이티브가 null 을 돌려줬다");
    let json = unsafe { CStr::from_ptr(ptr) }.to_str().unwrap().to_owned();
    swarm_free_string(ptr);
    serde_json::from_str(&json).unwrap()
}

fn last_error() -> String {
    let code = swarm_get_last_error_code();
    let ptr = swarm_get_last_error_message();
    let msg = if ptr.is_null() {
        String::new()
    } else {
        let s = unsafe { CStr::from_ptr(ptr) }.to_str().unwrap_or("").to_owned();
        swarm_free_string(ptr);
        s
    };
    format!("code={code} {msg}")
}

#[test]
#[ignore = "실측 하네스 — 지정 데이터로 step API 를 실제로 돌린다"]
fn every_lbf_order_mode_survives_the_step_api_init() {
    let path = std::env::var("SWARM_INPUT").unwrap_or_else(|_| {
        concat!(env!("CARGO_MANIFEST_DIR"), "/data/input/wsjk156.json").to_string()
    });
    let raw = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let value: serde_json::Value = serde_json::from_str(&raw).expect("입력 JSON 파싱");
    let json = CString::new(value.to_string()).unwrap();

    println!("\n══════ step API Init: LBF 배치 순서별 ══════");
    println!("  입력  {path}\n");

    let mut bad = vec![];
    for (mode, label) in [
        (0, "큰것부터 (기본)"),
        (1, "작은것부터"),
        (2, "중간부터(큰쪽먼저)"),
        (3, "중간부터(작은쪽먼저)"),
    ] {
        let mut cfg: CSwarmConfig = unsafe { std::mem::zeroed() };
        swarm_get_default_config(&mut cfg);
        cfg.lbf_order.mode = mode;
        // 탐색 기본 예산이 9분이라 4모드면 36분이다. 하네스는 짧게 돌린다.
        cfg.exploration.time_limit_secs = std::env::var("ORDER_EXPLORE_SECS")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(20.0);

        let handle: *mut SwarmStepHandle =
            swarm_init_from_json(json.as_ptr(), &cfg, std::ptr::null(), 0);
        if handle.is_null() {
            println!("  {label:22}  Init 핸들 생성 실패: {}", last_error());
            bad.push((label, "handle null".to_string()));
            continue;
        }

        // `swarm_init_phase` 는 별도 GRASP 초기배치이고 기본 꺼짐이다.
        // WPF 기본 경로는 `swarm_init_from_json` 안에서 LBF 를 돌리고 끝난다 —
        // 여기서 보는 해가 곧 사용자가 보는 "초기 해" 다.
        let _ = swarm_init_phase;

        let sol = take_json(swarm_get_solution_json(handle));
        let placed = sol["layout"]["placed_items"].as_array().map(|a| a.len()).unwrap_or(0);
        let width = sol["strip_width"].as_f64().unwrap_or(f64::NAN);

        // 게터는 콜백 설정과 무관하게 **항상 실측**한다.
        let statuses = take_json(swarm_get_collision_status_json(handle));
        let arr = statuses.as_array().cloned().unwrap_or_default();
        let colliding = arr
            .iter()
            .filter(|s| s.get("collision").and_then(|c| c.as_bool()) == Some(true))
            .count();

        println!(
            "  {label:22}  rc=0  폭 {width:9.2}  배치 {placed}  충돌보고 {colliding}/{}{}",
            arr.len(),
            if colliding > 0 { "  ◀ 문제" } else { "" }
        );
        if colliding > 0 {
            let ids: Vec<i64> = arr
                .iter()
                .filter(|s| s.get("collision").and_then(|c| c.as_bool()) == Some(true))
                .filter_map(|s| s.get("item_id").and_then(|v| v.as_i64()))
                .collect();
            println!("  {:22}   충돌 item_id: {ids:?}", "");
            bad.push((label, format!("{colliding} colliding")));
        }

        // 이어서 탐색을 짧게 돌려, 그 뒤의 해도 인증을 통과하는지 본다.
        // (WPF 는 Init 다음에 Explore 를 부른다.)
        let rc = swarm_alg_phase(handle);
        if rc != 0 {
            println!("  {:22}   explore rc={rc}  {}", "", last_error());
            bad.push((label, format!("explore rc={rc}")));
        } else {
            let after = take_json(swarm_get_collision_status_json(handle));
            let arr2 = after.as_array().cloned().unwrap_or_default();
            let col2 = arr2
                .iter()
                .filter(|s| s.get("collision").and_then(|c| c.as_bool()) == Some(true))
                .count();
            let sol2 = take_json(swarm_get_solution_json(handle));
            println!(
                "  {:22}   탐색 후: 폭 {:9.2}  충돌보고 {col2}/{}{}",
                "",
                sol2["strip_width"].as_f64().unwrap_or(f64::NAN),
                arr2.len(),
                if col2 > 0 { "  ◀ 문제" } else { "" }
            );
            if col2 > 0 {
                bad.push((label, format!("explore 후 {col2} colliding")));
            }
        }
        swarm_free(handle);
    }

    println!();
    assert!(
        bad.is_empty(),
        "step API Init 에서 문제가 난 모드가 있다: {bad:?}"
    );
}
