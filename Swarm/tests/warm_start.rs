//! LBF를 건너뛰고 **입력 배치를 그대로** 시작 레이아웃으로 쓰는 두 경로 검증.
//!
//! 실행:  cargo test --release --features capmods --test warm_start -- --nocapture
//!
//! LBF는 on/off 스위치가 아니라 "아직 배치되지 않은 아이템만" 채우는 단계다.
//! 따라서 사전 배치를 어디까지 주느냐가 곧 LBF가 얼마나 관여하느냐를 결정한다.
//!
//! 1. **해 JSON** — `swarm_init*`에 인스턴스와 해가 함께 담긴 JSON(`ExtSPOutput`:
//!    인스턴스 필드 + `"solution"`)을 주면 전 아이템이 복원되고 LBF는 `fit_strip()`만 한다.
//! 2. **배치 힌트** — `CPlacedHint`를 주면 그 좌표에서 출발한다. `is_freeze`는 좌표 사용
//!    여부와 무관한 별개의 제약(이동 금지)이므로, `is_freeze = 0`이면 그 자리에서 시작하되
//!    탐색이 자유롭게 옮긴다. 힌트를 준 아이템만 고정되고 나머지는 LBF가 채운다.

use cpswcore::ffi::{CSwarmConfig, swarm_free_string, swarm_get_default_config};
use cpswcore::ffi_step::*;
use std::ffi::{CStr, CString};

fn quick_config() -> CSwarmConfig {
    let mut c: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut c);
    c.rng_seed = 1;
    c.exploration.time_limit_secs = 2.0;
    c.compression.time_limit_secs = 1.0;
    c.polish.time_limit_secs = 1.0;
    c
}

fn init(json: &str, cfg: &CSwarmConfig) -> *mut SwarmStepHandle {
    init_with_hints(json, cfg, &[])
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

fn hint(item_id: i32, is_freeze: i32, tr_x: f32, tr_y: f32) -> CPlacedHint {
    CPlacedHint {
        item_id,
        is_freeze,
        tr_x,
        tr_y,
        rot_angle_in_rad: 0.0,
        stripe_interval_y: 0.0,
        stripe_interval_x: 0.0,
    }
}

fn instance_json(name: &str) -> String {
    std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(format!("data/input/{name}.json")),
    )
    .unwrap_or_else(|e| panic!("failed to read {name}.json: {e}"))
}

fn shirts_json() -> String {
    instance_json("shirts")
}

fn solution_json(h: *mut SwarmStepHandle) -> String {
    let p = swarm_get_solution_json(h);
    assert!(!p.is_null(), "no solution available");
    let s = unsafe { CStr::from_ptr(p) }.to_str().unwrap().to_owned();
    swarm_free_string(p);
    s
}

fn item_poses(json: &str, id: u64) -> Vec<String> {
    let value: serde_json::Value = serde_json::from_str(json).unwrap();
    let mut poses = value["layout"]["placed_items"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|p| p["item_id"].as_u64() == Some(id))
        .map(|p| p["transformation"].to_string())
        .collect::<Vec<_>>();
    assert!(!poses.is_empty());
    poses.sort();
    poses
}

fn strip_width(json: &str) -> f32 {
    let v: serde_json::Value = serde_json::from_str(json).expect("json must parse");
    fn find(v: &serde_json::Value) -> Option<f32> {
        match v {
            serde_json::Value::Object(m) => {
                for k in ["strip_width", "width", "container_width"] {
                    if let Some(w) = m.get(k).and_then(|x| x.as_f64()) {
                        return Some(w as f32);
                    }
                }
                m.values().find_map(find)
            }
            serde_json::Value::Array(a) => a.iter().find_map(find),
            _ => None,
        }
    }
    find(&v).expect("json must contain a strip width")
}

/// 인스턴스 + 해를 함께 주면 LBF가 재배치하지 않고 그 배치를 그대로 쓴다.
#[test]
fn init_with_solution_skips_lbf_placement() {
    let cfg = quick_config();
    let instance_json = std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("data/input/shirts.json"),
    )
    .expect("failed to read shirts.json");

    // 1) 인스턴스만 주면 LBF가 전부 배치한다 (기준값).
    let h1 = init(&instance_json, &cfg);
    let sol_json = solution_json(h1);
    let w_lbf = strip_width(&sol_json);
    println!("instance only  (LBF 실행): {w_lbf:.4}");

    // 2) 그 해를 인스턴스에 얹어 ExtSPOutput 형식으로 다시 넣는다.
    //    (`swarm_get_solution_json`은 해만 내보내므로 인스턴스와 합쳐야 한다.)
    let merged = {
        let mut v: serde_json::Value = serde_json::from_str(&instance_json).unwrap();
        let sol: serde_json::Value = serde_json::from_str(&sol_json).unwrap();
        v.as_object_mut().unwrap().insert("solution".into(), sol);
        serde_json::to_string(&v).unwrap()
    };
    let h2 = init(&merged, &cfg);
    let w_restored = strip_width(&solution_json(h2));
    println!("instance+해     (LBF 무동작): {w_restored:.4}");

    // 3) 시드를 바꿔도 동일해야 한다 — 달라지면 LBF가 재배치했다는 뜻이다.
    let mut cfg_other_seed = cfg;
    cfg_other_seed.rng_seed = 999;
    let h3 = init(&merged, &cfg_other_seed);
    let w_other_seed = strip_width(&solution_json(h3));
    println!("instance+해     (다른 시드): {w_other_seed:.4}");

    assert!(
        (w_restored - w_lbf).abs() < 1e-3,
        "입력 배치가 그대로 반영되지 않았다: {w_lbf} -> {w_restored}"
    );
    assert!(
        (w_other_seed - w_lbf).abs() < 1e-3,
        "시드에 따라 결과가 달라졌다 = LBF가 재배치했다: {w_lbf} -> {w_other_seed}"
    );

    // 입력 배치를 유지한 채 탐색까지 이어진다.
    assert_eq!(
        swarm_alg_phase(h2),
        0,
        "warm start 이후 탐색이 가능해야 한다"
    );

    swarm_free(h1);
    swarm_free(h2);
    swarm_free(h3);
}

/// `is_freeze = 0` 힌트도 좌표가 반영되어야 하고(=시작 위치), 그럼에도 이동은 가능해야 한다.
/// `is_freeze = 1`과의 대비로 "좌표 사용"과 "고정"이 분리되었음을 확인한다.
#[test]
fn unfrozen_hint_seeds_position_but_stays_movable() {
    let mut cfg = quick_config();
    cfg.exploration.time_limit_secs = 5.0;
    let inst = shirts_json();

    // 기준: 힌트 없이 LBF만
    let h0 = init(&inst, &cfg);
    let w_base = strip_width(&solution_json(h0));

    // 아이템 0을 기준 폭의 3배쯤 되는 먼 오른쪽에 두는 힌트
    let far_x = w_base * 3.0;
    println!("기준 폭 {w_base:.2}, 힌트 위치 x={far_x:.2}");

    // (1) freeze = 0 : 좌표는 쓰되 이동 허용
    let h1 = init_with_hints(&inst, &cfg, &[hint(0, 0, far_x, 0.0)]);
    let free_before = item_poses(&solution_json(h1), 0);
    let w1_init = strip_width(&solution_json(h1));
    assert!(
        w1_init > w_base * 2.0,
        "freeze=0 힌트의 좌표가 무시됐다: 기준 {w_base:.2} vs 힌트 적용 {w1_init:.2}"
    );
    assert_eq!(swarm_alg_phase(h1), 0);
    let w1_expl = strip_width(&solution_json(h1));

    // (2) freeze = 1 : strip 폭과 무관하게 원래 좌표/회전을 유지한다
    let h2 = init_with_hints(&inst, &cfg, &[hint(0, 1, far_x, 0.0)]);
    let frozen_before = item_poses(&solution_json(h2), 0);
    let w2_init = strip_width(&solution_json(h2));
    assert_eq!(swarm_alg_phase(h2), 0);
    let w2_expl = strip_width(&solution_json(h2));

    println!("freeze=0: init {w1_init:.2} -> explore {w1_expl:.2}");
    println!("freeze=1: init {w2_init:.2} -> explore {w2_expl:.2}");

    assert!(w1_expl < w1_init, "freeze=0인데 탐색이 전혀 줄이지 못했다");
    assert_ne!(free_before, item_poses(&solution_json(h1), 0));
    assert_eq!(frozen_before, item_poses(&solution_json(h2), 0));

    swarm_free(h0);
    swarm_free(h1);
    swarm_free(h2);
}

/// 범위를 벗어난 item_id 힌트는 패닉 없이 무시되어야 한다.
#[test]
fn out_of_range_hint_is_ignored() {
    let cfg = quick_config();
    let inst = shirts_json();
    let h = init_with_hints(
        &inst,
        &cfg,
        &[hint(99_999, 0, 10.0, 10.0), hint(-1, 1, 5.0, 5.0)],
    );
    let w = strip_width(&solution_json(h));
    assert!(w.is_finite() && w > 0.0, "잘못된 힌트가 초기화를 깨뜨렸다");
    swarm_free(h);
}

/// 전 아이템에 힌트를 주되 **일부만** `is_freeze = 1`인 혼합 상태.
/// 고정된 것은 제자리를 지키고, 고정되지 않은 것은 탐색이 끌어와야 한다.
///
/// 수량이 모두 1개인 `fu`를 쓴다 — 고정은 배치 인스턴스가 아니라 **아이템 종류(item_id)**
/// 단위로 걸리므로, 한 종류가 여러 개인 인스턴스에서는 같은 종류 전체가 함께 고정된다.
#[test]
fn mixed_freeze_holds_only_the_frozen_items() {
    let mut cfg = quick_config();
    cfg.exploration.time_limit_secs = 8.0;
    let inst = instance_json("fu");

    let baseline = init(&inst, &cfg);
    let w_base = strip_width(&solution_json(baseline));
    swarm_free(baseline);
    let (near_x, far_x) = (w_base * 2.0, w_base * 5.0);
    println!("fu 기준 폭 {w_base:.2} / 고정 위치 {near_x:.2} / 이동 대상 위치 {far_x:.2}");

    // (1) 혼합: item 1은 near_x에 고정, item 0은 far_x에서 출발하되 이동 허용
    let h_mixed = init_with_hints(
        &inst,
        &cfg,
        &[hint(0, 0, far_x, 0.0), hint(1, 1, near_x, 0.0)],
    );
    let mixed_before = solution_json(h_mixed);
    let w_mixed_init = strip_width(&mixed_before);
    assert_eq!(swarm_alg_phase(h_mixed), 0);
    let w_mixed = strip_width(&solution_json(h_mixed));

    // (2) 대조: 둘 다 고정 → strip 폭과 무관하게 두 좌표를 모두 유지한다
    let h_both = init_with_hints(
        &inst,
        &cfg,
        &[hint(0, 1, far_x, 0.0), hint(1, 1, near_x, 0.0)],
    );
    let both_before = solution_json(h_both);
    assert_eq!(swarm_alg_phase(h_both), 0);
    let w_both = strip_width(&solution_json(h_both));

    println!("혼합(0 자유/1 고정): init {w_mixed_init:.2} -> explore {w_mixed:.2}");
    println!("둘 다 고정         : explore {w_both:.2}");

    let mixed_after = solution_json(h_mixed);
    let both_after = solution_json(h_both);
    assert_eq!(item_poses(&mixed_before, 1), item_poses(&mixed_after, 1));
    assert_ne!(item_poses(&mixed_before, 0), item_poses(&mixed_after, 0));
    for id in [0, 1] {
        assert_eq!(item_poses(&both_before, id), item_poses(&both_after, id));
    }
    assert!(w_mixed < w_mixed_init);

    swarm_free(h_mixed);
    swarm_free(h_both);
}
