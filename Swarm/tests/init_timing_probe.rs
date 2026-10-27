//! Init 시간이 어디에 쓰이는지 단계별로 잰다 (JSON 파싱 vs import vs 시드 처리).
//!
//! 실행:  cargo test --release --test init_timing_probe -- --ignored --nocapture
//!
//! 질문: 배치 조각이 많을 때 Init 이 느린 것이 JSON 전달 때문인가?
//! JSON 파싱만 따로 재고, 같은 배치를 (a) JSON solution 으로, (b) 메모리 힌트 배열로
//! 넘겨 Init 시간을 비교한다.

use cpswcore::ffi::{CSwarmConfig, swarm_get_default_config};
use cpswcore::ffi_step::*;
use std::ffi::CString;
use std::time::Instant;

fn read(name: &str) -> String {
    std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(format!("data/input/{name}.json")),
    )
    .unwrap()
}

fn config() -> CSwarmConfig {
    let mut c: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut c);
    c.rng_seed = 1;
    c
}

extern "C" fn print_native_log(level: i32, message: *const std::ffi::c_char, _: usize) {
    // 경고 이상만: 힌트가 버려지거나 분리 단계로 넘어갈 때 그 이유가 찍힌다.
    if level <= 2 && !message.is_null() {
        let text = unsafe { std::ffi::CStr::from_ptr(message) }.to_string_lossy();
        println!("    [native] {text}");
    }
}

fn ms(t: Instant) -> f64 {
    t.elapsed().as_secs_f64() * 1e3
}

fn timed_init(label: &str, json: &str, cfg: &CSwarmConfig, hints: &[CPlacedHint]) {
    let cs = CString::new(json).unwrap();
    let ptr = if hints.is_empty() { std::ptr::null() } else { hints.as_ptr() };
    let t = Instant::now();
    let h = swarm_init_from_json(cs.as_ptr(), cfg, ptr, hints.len() as i32);
    let init_ms = ms(t);
    let ok = !h.is_null();
    let code = cpswcore::ffi_step::swarm_get_last_error_code();
    if ok {
        swarm_free(h);
    }
    println!("{label:<44} swarm_init_from_json {init_ms:>9.1} ms (ok={ok}, last_error={code})");
}

/// warmstart 의 solution 배치를 메모리 힌트로 바꾼다 (원래 외곽 기준 좌표).
fn hints_from_solution(doc: &serde_json::Value, frozen: bool) -> Vec<CPlacedHint> {
    let items = doc["items"].as_array().unwrap();
    doc["solution"]["layout"]["placed_items"]
        .as_array()
        .unwrap()
        .iter()
        .map(|p| {
            let id = p["item_id"].as_u64().unwrap() as usize;
            let rot = p["transformation"]["rotation"].as_f64().unwrap().to_radians() as f32;
            let tx = p["transformation"]["translation"][0].as_f64().unwrap() as f32;
            let ty = p["transformation"]["translation"][1].as_f64().unwrap() as f32;
            let pts = items[id]["shape"]["data"].as_array().unwrap();
            let xmin = pts.iter().map(|v| v[0].as_f64().unwrap()).fold(f64::INFINITY, f64::min) as f32;
            let ymin = pts.iter().map(|v| v[1].as_f64().unwrap()).fold(f64::INFINITY, f64::min) as f32;
            let (c, s) = (rot.cos(), rot.sin());
            CPlacedHint {
                item_id: id as i32,
                is_freeze: frozen as i32,
                tr_x: tx + (c * xmin - s * ymin),
                tr_y: ty + (s * xmin + c * ymin),
                rot_angle_in_rad: rot,
                stripe_interval_y: 0.0,
                stripe_interval_x: 0.0,
            }
        })
        .collect()
}

#[test]
#[ignore]
fn init_time_breakdown_wsjk156() {
    let plain = read("wsjk156");
    let warm = read("wsjk156.warmstart");
    println!("입력 크기: plain {} KB, warmstart {} KB", plain.len() / 1024, warm.len() / 1024);

    // jagua-rs 0.8 의 solution 은 strip_width/density/run_time_sec 가 있어야 읽힌다.
    // data/ 의 warmstart 파일에는 그 필드가 없어 배치가 조용히 버려진다 — 채워서 쓴다.
    let mut doc: serde_json::Value = serde_json::from_str(&warm).unwrap();
    let max_x = {
        let items = doc["items"].as_array().unwrap().clone();
        doc["solution"]["layout"]["placed_items"]
            .as_array()
            .unwrap()
            .iter()
            .map(|p| {
                let id = p["item_id"].as_u64().unwrap() as usize;
                let tx = p["transformation"]["translation"][0].as_f64().unwrap();
                let pts = items[id]["shape"]["data"].as_array().unwrap();
                tx + pts.iter().map(|v| v[0].as_f64().unwrap()).fold(f64::NEG_INFINITY, f64::max)
            })
            .fold(0.0, f64::max)
    };
    doc["solution"]["strip_width"] = serde_json::json!(max_x + 1.0);
    doc["solution"]["density"] = serde_json::json!(0.0);
    doc["solution"]["run_time_sec"] = serde_json::json!(0);
    doc["solution"]["layout"]["density"] = serde_json::json!(0.0);
    let warm_fixed = doc.to_string();

    for (label, json) in [("plain", &plain), ("warm(solution 보완)", &warm_fixed)] {
        for _ in 0..3 {
            let t = Instant::now();
            let v: serde_json::Value = serde_json::from_str(json).unwrap();
            let value_ms = ms(t);
            drop(v);
            let t = Instant::now();
            let (inst, sol) = cpswcore::swarm_core::util::io::parse_spp_input(json).unwrap();
            let typed_ms = ms(t);
            let cfg = cpswcore::swarm_core::config::DEFAULT_SWARM_CONFIG;
            let t = Instant::now();
            let _instance = cpswcore::swarm_core::util::io::import_spp_instance_with_mm(
                &inst,
                cfg.cde_config,
                cfg.poly_simpl_tolerance,
                cfg.min_item_separation,
                cfg.narrow_concavity_cutoff_ratio,
                cfg.simplify_max_deviation_mm,
            )
            .unwrap();
            let import_ms = ms(t);
            println!(
                "{label:<22} JSON→Value {value_ms:>7.1} ms | JSON→typed {typed_ms:>7.1} ms | import(형상 준비) {import_ms:>8.1} ms | placed={}",
                sol.map_or(0, |s| s.layout.placed_items.len())
            );
        }
    }

    cpswcore::ffi_log::swarm_set_log_callback(Some(print_native_log), 0, 2);
    let cfg = config();
    let movable = hints_from_solution(&doc, false);
    let frozen = hints_from_solution(&doc, true);
    timed_init("plain, 배치 없음 (LBF 가 전부 배치)", &plain, &cfg, &[]);
    // WPF 기본값처럼 NFP-LBF 시드를 켜면 시드 생성과 시드 대결(짧은 탐색 2회)이 더해진다.
    let mut wpf_like = cfg;
    wpf_like.nfp_lbf.enabled = 1;
    timed_init("plain, NFP-LBF 시드 켬 (WPF 기본)", &plain, &wpf_like, &[]);
    timed_init("warm JSON solution 156개 (이동 가능)", &warm_fixed, &cfg, &[]);
    timed_init("plain + 메모리 힌트 156개 (이동 가능)", &plain, &cfg, &movable);
    timed_init("plain + 메모리 힌트 156개 (고정)", &plain, &cfg, &frozen);

    // 실제 입력처럼 힌트끼리 미세하게 겹치는 경우: x 좌표를 0.2% 좁힌다.
    for squeeze in [0.9995f32, 0.998] {
        let tight: Vec<CPlacedHint> = movable
            .iter()
            .map(|h| CPlacedHint { tr_x: h.tr_x * squeeze, ..*h })
            .collect();
        timed_init(&format!("plain + 메모리 힌트 156개 (x*{squeeze}, 미세 겹침)"), &plain, &cfg, &tight);
    }

    // V2 envelope: 조각마다 placed_hint 를 JSON 안에 담는다 (WPF 의 Place/PlaceGroup 경로).
    for (label, hints) in [("V2 JSON placed_hint 156개 (이동 가능)", &movable), ("V2 JSON placed_hint 156개 (고정)", &frozen)] {
        let mut inst: serde_json::Value = serde_json::from_str(&plain).unwrap();
        for h in hints.iter() {
            inst["items"][h.item_id as usize]["placed_hint"] = serde_json::json!({
                "item_id": h.item_id, "is_freeze": h.is_freeze, "tr_x": h.tr_x, "tr_y": h.tr_y,
                "rot_angle_in_rad": h.rot_angle_in_rad, "stripe_interval_y": 0.0, "stripe_interval_x": 0.0
            });
        }
        let v2 = serde_json::json!({
            "schema_version": 2,
            "nesting": {"mode": "all_items", "group_workers": 1, "quantum_ms": 250, "locked_prefix_gap_mm": 0.0},
            "instance": inst, "regions": [], "place_groups": []
        })
        .to_string();
        let t = Instant::now();
        let parsed = cpswcore::grouped_input::parse_and_validate(&v2);
        println!("{label:<44} V2 파싱+검증 {:>7.1} ms (ok={})", ms(t), parsed.is_ok());
        timed_init(label, &v2, &cfg, &[]);
    }
}

/// NFP-LBF 시드 탐색만 따로 잰다: 같은 예산 안에서 몇 개의 ordering 을 평가했는지.
#[test]
#[ignore]
fn nfp_lbf_seed_orderings_within_budget_wsjk156() {
    use cpswcore::swarm_core::config::DEFAULT_SWARM_CONFIG;
    use rand::SeedableRng;
    let plain = read("wsjk156");
    let (ext, _) = cpswcore::swarm_core::util::io::parse_spp_input(&plain).unwrap();
    let cfg = DEFAULT_SWARM_CONFIG;
    let instance = cpswcore::swarm_core::util::io::import_spp_instance_with_mm(
        &ext,
        cfg.cde_config,
        cfg.poly_simpl_tolerance,
        cfg.min_item_separation,
        cfg.narrow_concavity_cutoff_ratio,
        cfg.simplify_max_deviation_mm,
    )
    .unwrap();
    let nfp_cfg = cfg.nfp_lbf.to_config(cfg.nfp_strategy);
    for seed in [1u64, 2, 3] {
        let mut rng = rand::rngs::Xoshiro256PlusPlus::seed_from_u64(seed);
        let t = Instant::now();
        let nest = cpswcore::swarm_core::nfp_lbf::build_best_initial_nest(&instance, &nfp_cfg, &mut rng)
            .expect("seed");
        let r = &nest.report;
        println!(
            "seed {seed}: {:>7.1} ms, orderings {:>3}, prefix hits {:>3}, best length {:.1}",
            ms(t),
            r.orderings_evaluated,
            r.prefix_reuse_hits,
            r.best_strip_length
        );
    }
}
