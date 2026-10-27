use cpswcore::ffi::{CSwarmConfig, swarm_free_string, swarm_get_default_config};
use cpswcore::ffi_step::{swarm_free, swarm_get_solution_json, swarm_init_from_json};
use std::ffi::{CStr, CString};
use std::time::Instant;

fn init_without_hints(text: String) -> (serde_json::Value, f64) {
    init_with_overlap(text, 0.0)
}

fn init_with_overlap(text: String, overlap: f32) -> (serde_json::Value, f64) {
    init_with_geometry(text, overlap, None)
}

fn init_with_geometry(text: String, overlap: f32, mm: Option<f32>) -> (serde_json::Value, f64) {
    let mut cfg: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut cfg);
    cfg.refplace.enabled = 1;
    cfg.refplace._reserved0 = 1;
    cfg.fabric_tolerance.piece_overlap_mm = overlap;
    cfg.refplace.simplify_max_deviation_mm = mm.unwrap_or(0.0);
    let json = CString::new(text).unwrap();
    let start = Instant::now();
    let handle = swarm_init_from_json(json.as_ptr(), &cfg, std::ptr::null(), 0);
    assert!(!handle.is_null(), "Draft Init failed");
    let p = swarm_get_solution_json(handle);
    assert!(!p.is_null());
    let text = unsafe { CStr::from_ptr(p) }.to_str().unwrap().to_owned();
    swarm_free_string(p);
    swarm_free(handle);
    (
        serde_json::from_str(&text).unwrap(),
        start.elapsed().as_secs_f64(),
    )
}

#[test]
fn configured_overlap_reaches_draft_contacts_through_public_init() {
    let input = serde_json::json!({"name":"column-overlap-ffi","strip_height":100,
        "items":[{"id":0,"demand":2,"allowed_orientations":[0],"shape":{
            "type":"simple_polygon","data":[[0,0],[40,0],[40,50.05],[0,50.05]]}}]})
    .to_string();
    let (zero, _) = init_with_overlap(input.clone(), 0.0);
    let (allowed, _) = init_with_overlap(input, 0.2);
    assert!(zero["strip_width"].as_f64().unwrap() > 79.0);
    assert!(allowed["strip_width"].as_f64().unwrap() < 41.0);
    assert_eq!(
        allowed["layout"]["placed_items"].as_array().unwrap().len(),
        2
    );
}

#[test]
fn selected_column_draft_works_through_the_public_init_api() {
    let (sol, _) = init_without_hints(
        serde_json::json!({"name":"column-ffi","strip_height":100,
        "items":[{"id":0,"demand":6,"allowed_orientations":[0],"shape":{
            "type":"simple_polygon","data":[[0,0],[40,0],[40,45],[0,45]]}}]})
        .to_string(),
    );
    assert_eq!(sol["layout"]["placed_items"].as_array().unwrap().len(), 6);
    assert!(sol["strip_width"].as_f64().unwrap() < 122.0);
}

#[test]
#[ignore = "Set DRAFT_INPUT: default UI geometry settings, no hints"]
fn real_marker_column_draft_init_without_hints() {
    let _ = fern::Dispatch::new()
        .level(log::LevelFilter::Info)
        .filter(|m| m.target().contains("ffi_step") || m.target().contains("util::io"))
        .chain(std::io::stdout())
        .apply();
    let text = std::fs::read_to_string(std::env::var("DRAFT_INPUT").expect("DRAFT_INPUT")).unwrap();
    let mut input: serde_json::Value = serde_json::from_str(&text).unwrap();
    let expected: usize = input["items"]
        .as_array()
        .unwrap()
        .iter()
        .map(|it| it["demand"].as_u64().unwrap() as usize)
        .sum();
    for it in input["items"].as_array_mut().unwrap() {
        it.as_object_mut().unwrap().remove("placed_hint");
    }
    let overlap: f32 = std::env::var("DRAFT_OVERLAP_MM")
        .ok()
        .map(|s| s.parse().unwrap())
        .unwrap_or(0.0);
    assert!(overlap >= 0.0);
    let mm: Option<f32> = std::env::var("DRAFT_MM").ok().map(|s| s.parse().unwrap());
    let (sol, secs) = init_with_geometry(input.to_string(), overlap, mm);
    let actual = sol["layout"]["placed_items"].as_array().unwrap().len();
    assert_eq!(actual, expected);
    // Audit the exported transforms against the source outline, independently
    // of the internal collision-shape bbox (acute offset corners differ).
    for placed in sol["layout"]["placed_items"].as_array().unwrap() {
        let id = placed["item_id"].as_u64().unwrap();
        let item = input["items"]
            .as_array()
            .unwrap()
            .iter()
            .find(|it| it["id"].as_u64() == Some(id))
            .unwrap();
        let t = &placed["transformation"];
        let (sin, cos) = t["rotation"].as_f64().unwrap().sin_cos();
        for v in item["shape"]["data"].as_array().unwrap() {
            let (x, y) = (v[0].as_f64().unwrap(), v[1].as_f64().unwrap());
            let px = cos * x - sin * y + t["translation"][0].as_f64().unwrap();
            let py = sin * x + cos * y + t["translation"][1].as_f64().unwrap();
            assert!(
                px >= -0.002
                    && py >= -0.002
                    && px <= sol["strip_width"].as_f64().unwrap() + 0.002
                    && py <= input["strip_height"].as_f64().unwrap() + 0.002,
                "source corner outside fabric: item {id}, {px}, {py}"
            );
        }
    }
    println!(
        "Column Draft public Init: {actual} pieces, overlap {overlap}mm, simplification {mm:?}mm, width {}, elapsed {secs:.3}s",
        sol["strip_width"]
    );
    if let Ok(path) = std::env::var("DRAFT_SOLUTION") {
        std::fs::write(path, serde_json::to_string_pretty(&sol).unwrap()).unwrap();
    }
}
