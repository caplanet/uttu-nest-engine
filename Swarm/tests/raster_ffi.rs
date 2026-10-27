//! 래스터 NFP가 FFI(ffi_step) 경로로 실제 LBF까지 도달하는지 검증.
//!
//! 실행:  cargo test --release --features "capmods simd" --test raster_ffi -- --nocapture
//!
//! WPF 등 외부 앱에서 "래스터를 켜도 초기해가 안 변한다"면 원인은 둘 중 하나다.
//!
//! 1. 네이티브 DLL이 capmods 없이 빌드됐거나 재빌드되지 않았다 (구 DLL은 확장된
//!    CSwarmConfig의 lbf_raster 꾬리를 읽지 않고 조용히 무시한다).
//! 2. **배치 힌트가 전 아이템을 선배치하고 있다** — 힌트가 있으면 그 좌표가 초기
//!    배치가 되고 LBF는 "남은 아이템"만 채우므로, 전 아이템 힌트에서는 LBF가
//!    아무것도 배치하지 않는다. 래스터는 LBF 후보 생성기라 효과가 정확히 0이다.
//!
//! 이 파일은 두 경우를 분리해 재현한다.

// Raster LBF candidates only exist with the `capmods` feature (optimizer/lbf.rs).
#![cfg(feature = "capmods")]

use cpswcore::ffi::{CSwarmConfig, swarm_free_string, swarm_get_default_config};
use cpswcore::ffi_step::*;
use std::ffi::{CStr, CString};

fn quick_config(raster_on: bool) -> CSwarmConfig {
    let mut c: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut c);
    c.rng_seed = 7;
    c.lbf_raster.enabled = raster_on as i32;
    c
}

fn init_with_hints(
    json: &str,
    cfg: &CSwarmConfig,
    hints: &[CPlacedHint],
) -> *mut SwarmStepHandle {
    let cs = CString::new(json).expect("json contains NUL");
    let ptr = if hints.is_empty() {
        std::ptr::null()
    } else {
        hints.as_ptr()
    };
    let h = swarm_init_from_json(cs.as_ptr(), cfg, ptr, hints.len() as i32);
    assert!(!h.is_null(), "swarm_init_from_json returned NULL");
    h
}

fn init_width(json: &str, cfg: &CSwarmConfig, hints: &[CPlacedHint]) -> f32 {
    let h = init_with_hints(json, cfg, hints);
    let p = swarm_get_solution_json(h);
    assert!(!p.is_null());
    let s = unsafe { CStr::from_ptr(p) }.to_str().unwrap().to_owned();
    swarm_free_string(p);
    swarm_free(h);

    let v: serde_json::Value = serde_json::from_str(&s).unwrap();
    v.get("strip_width")
        .and_then(|x| x.as_f64())
        .expect("strip_width missing") as f32
}

fn instance_json(name: &str) -> String {
    std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(format!("data/input/{name}.json")),
    )
    .unwrap_or_else(|e| panic!("failed to read {name}.json: {e}"))
}

/// 힌트가 없으면 래스터 on/off가 FFI 경로에서도 실제로 다른 초기해를 만든다.
#[test]
fn raster_reaches_lbf_through_ffi() {
    // 저장소 인스턴스 + (있으면) 실물 마커도 함께 확인
    let mut cases: Vec<(String, String)> = vec![("shirts".into(), instance_json("shirts"))];
    if let Ok(p) = std::env::var("SWARM_EXTRA_INSTANCE")
        && let Ok(j) = std::fs::read_to_string(&p)
    {
        cases.push((p, j));
    }

    for (name, json) in &cases {
        // 시드별 폭이 같을 수도 있으므로 여러 시드에서 한 번이라도 달라지는지 본다.
        let mut any_diff = false;
        for seed in [7u64, 8, 9] {
            let mut off = quick_config(false);
            off.rng_seed = seed;
            let mut on = quick_config(true);
            on.rng_seed = seed;
            let w_off = init_width(json, &off, &[]);
            let w_on = init_width(json, &on, &[]);
            println!(
                "{name} seed {seed}: raster off {w_off:.2} -> on {w_on:.2} ({:+.2}%)",
                (w_on / w_off - 1.0) * 100.0
            );
            if (w_on - w_off).abs() > 1e-3 {
                any_diff = true;
            }
        }
        assert!(
            any_diff,
            "{name}: 래스터가 FFI 경로에서 LBF에 전혀 영향을 주지 않는다 (게이트/배선 문제)"
        );
    }
}

/// 전 아이템 힌트(isFreeze=0)가 있으면 LBF가 아무것도 배치하지 않으므로
/// 래스터 on/off가 완전히 동일한 초기해를 낸다. 이것은 버그가 아니라
/// "래스터는 LBF 후보 생성기"라는 설계의 직접적 귀결이다.
#[test]
fn full_hints_bypass_lbf_so_raster_is_noop() {
    let json = instance_json("fu");
    let v: serde_json::Value = serde_json::from_str(&json).unwrap();
    let n_items = v["items"].as_array().unwrap().len();

    // 아이템을 일렬로 늘어놓는 전 아이템 힌트 (freeze 아님 → warm start)
    let hints: Vec<CPlacedHint> = (0..n_items)
        .map(|i| CPlacedHint {
            item_id: i as i32,
            is_freeze: 0,
            tr_x: 40.0 * i as f32,
            tr_y: 0.0,
            rot_angle_in_rad: 0.0,
            stripe_interval_y: 0.0,
            stripe_interval_x: 0.0,
        })
        .collect();

    let w_off = init_width(&json, &quick_config(false), &hints);
    let w_on = init_width(&json, &quick_config(true), &hints);
    println!("full hints: raster off {w_off:.2} / on {w_on:.2}");

    assert!(
        (w_on - w_off).abs() < 1e-3,
        "전 아이템 힌트에서는 LBF가 배치할 게 없으므로 래스터는 무영향이어야 한다: {w_off} vs {w_on}"
    );
}
