//! **Use Hints 를 켜면 힌트 좌표가 실제로 쓰이는가** — WPF 가 타는 경로 그대로 검증한다.
//!
//! # 배경
//!
//! `placed_hint` 를 담은 입력을 Use Hints 로 실행했는데 힌트가 무시되고 LBF 가 처음부터
//! 배치하는 일이 있었다. 원인은 힌트가 아니라 **검증**이었다.
//!
//! `legacy_seed_problem` 이 힌트대로 전 조각을 놓은 뒤 exact 검증을 돌리는데, 상용 마커를
//! 변환한 입력은 0.025mm 격자 양자화 때문에 접촉면에 미세 관통이 남는다. 검증이 실패하면
//! 예전 코드는 frozen 힌트만 남겨 다시 지었고, 힌트가 전부 `is_freeze: 0` 이면 frozen
//! 집합이 비어 **156개가 통째로 사라졌다**.
//!
//! 지금은 좌표를 그대로 쓰고 겹침만 그 자리에서 분리기로 푼다. 이 테스트가 그 계약을
//! 고정한다 — 힌트 배치의 구조가 초기 해에 남아 있어야 한다.
//!
//! 합성 인스턴스를 쓰므로 외부 파일이 필요 없다.

use cpswcore::ffi::{CSwarmConfig, swarm_free_string, swarm_get_default_config};
use cpswcore::ffi_step::{
    CPlacedHint, SwarmStepHandle, swarm_free, swarm_get_solution_json,
    swarm_init_from_json,
};
use std::ffi::{CStr, CString};

/// 폭 `w`, 높이 `h` 사각형 아이템 하나.
fn item(id: u64, w: f32, h: f32) -> serde_json::Value {
    serde_json::json!({
        "id": id,
        "demand": 1,
        "allowed_orientations": [0],
        "shape": { "type": "simple_polygon", "data": [[0,0],[w,0],[w,h],[0,h]] }
    })
}

/// 조각 4개를 폭 100 스트립에 담는 인스턴스.
fn instance_json() -> serde_json::Value {
    serde_json::json!({
        "name": "hint-warm-start",
        "items": [item(0, 40.0, 40.0), item(1, 40.0, 40.0), item(2, 40.0, 40.0), item(3, 40.0, 40.0)],
        "strip_height": 100.0
    })
}

fn hint(item_id: i32, tr_x: f32, tr_y: f32) -> CPlacedHint {
    CPlacedHint {
        item_id,
        is_freeze: 0,
        tr_x,
        tr_y,
        rot_angle_in_rad: 0.0,
        stripe_interval_y: 0.0,
        stripe_interval_x: 0.0,
    }
}

fn init(json: &serde_json::Value, hints: &[CPlacedHint]) -> *mut SwarmStepHandle {
    init_with(json, hints, None)
}

/// 상용 마커 변환본을 힌트로 쓰려면 두 가지가 필요하다.
///
/// * `poly_simpl_tolerance` 를 끈다(음수). 단순화는 충돌 도형을 최대 0.9mm 부풀리는데,
///   상용 마커는 조각이 원단 폭에 정확히 꽉 차 있어 그대로면 경계를 넘는다.
///   이건 분리기로 못 고친다 — 부풀린 도형이 원단보다 커서 밀어 넣을 자리가 없다.
/// * `piece_overlap_mm` 을 0.1 로 준다. 0.025mm 격자 양자화가 남긴 미세 관통을 흡수한다.
///   **더 크게 주면 오히려 실패한다** — 겹침 허용은 컨테이너도 같이 줄이므로
///   가장자리 조각이 밖으로 나간다(실측: 0.2mm 에서 경계 위반 1건).
fn marker_config() -> CSwarmConfig {
    let mut cfg: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut cfg);
    cfg.poly_simpl_tolerance = -1.0;
    cfg.fabric_tolerance.piece_overlap_mm = 0.1;
    cfg
}

fn init_with(
    json: &serde_json::Value,
    hints: &[CPlacedHint],
    config: Option<&CSwarmConfig>,
) -> *mut SwarmStepHandle {
    let text = CString::new(json.to_string()).unwrap();
    let handle = swarm_init_from_json(
        text.as_ptr(),
        config.map_or(std::ptr::null(), |c| c as *const CSwarmConfig),
        if hints.is_empty() { std::ptr::null() } else { hints.as_ptr() },
        hints.len() as i32,
    );
    assert!(!handle.is_null(), "init 이 실패했다");
    handle
}

/// 초기 해의 (조각별 좌하단 x, y) 목록을 id 순으로 돌려준다.
fn placements(handle: *mut SwarmStepHandle) -> Vec<(u64, f32, f32)> {
    let ptr = swarm_get_solution_json(handle);
    assert!(!ptr.is_null());
    let text = unsafe { CStr::from_ptr(ptr) }.to_str().unwrap().to_owned();
    swarm_free_string(ptr);
    let doc: serde_json::Value = serde_json::from_str(&text).unwrap();
    let mut out: Vec<(u64, f32, f32)> = doc["layout"]["placed_items"]
        .as_array()
        .expect("placed_items")
        .iter()
        .map(|p| {
            let t = &p["transformation"]["translation"];
            (
                p["item_id"].as_u64().unwrap(),
                t[0].as_f64().unwrap() as f32,
                t[1].as_f64().unwrap() as f32,
            )
        })
        .collect();
    out.sort_by_key(|(id, _, _)| *id);
    out
}

/// 겹치지 않는 힌트는 예전에도 통과했다 — 회귀 방지용 기준선.
#[test]
fn clean_hints_are_used_as_the_initial_layout() {
    let hints = [
        hint(0, 0.0, 0.0),
        hint(1, 40.0, 0.0),
        hint(2, 0.0, 40.0),
        hint(3, 40.0, 40.0),
    ];
    let handle = init(&instance_json(), &hints);
    let placed = placements(handle);
    swarm_free(handle);

    assert_eq!(placed.len(), 4);
    for (i, (_, x, y)) in placed.iter().enumerate() {
        let h = &hints[i];
        assert!(
            (x - h.tr_x).abs() < 0.5 && (y - h.tr_y).abs() < 0.5,
            "힌트 좌표가 그대로 쓰이지 않았다: item {i} 힌트 ({}, {}) vs 배치 ({x}, {y})",
            h.tr_x,
            h.tr_y
        );
    }
}

/// **핵심 회귀 테스트.** 미세하게 겹친 힌트도 버리지 않고, 좌표를 그대로 쓰되
/// 겹침만 풀어야 한다. 예전에는 4개가 전부 버려지고 LBF 가 처음부터 배치했다.
#[test]
fn slightly_overlapping_hints_are_repaired_not_discarded() {
    // item 1 을 0.05mm 왼쪽으로 밀어 item 0 과 관통시킨다.
    // 상용 마커의 양자화 겹침(최대 0.0954mm)과 같은 규모다.
    let hints = [
        hint(0, 0.0, 0.0),
        hint(1, 39.95, 0.0),
        hint(2, 0.0, 40.0),
        hint(3, 40.0, 40.0),
    ];
    let handle = init(&instance_json(), &hints);
    let placed = placements(handle);
    swarm_free(handle);

    assert_eq!(placed.len(), 4);
    // 겹침을 풀었으므로 좌표가 조금 움직일 수 있다. 그러나 **구조**는 남아야 한다 —
    // LBF 로 다시 지었다면 네 조각이 전부 왼쪽 아래로 몰려 힌트와 크게 어긋난다.
    for (i, (_, x, y)) in placed.iter().enumerate() {
        let h = &hints[i];
        assert!(
            (x - h.tr_x).abs() < 2.0 && (y - h.tr_y).abs() < 2.0,
            "힌트 배치가 버려지고 다시 지어진 것으로 보인다: \
             item {i} 힌트 ({}, {}) vs 배치 ({x}, {y})",
            h.tr_x,
            h.tr_y
        );
    }
}

/// 힌트가 없으면 예전대로 LBF 가 전부 배치한다.
#[test]
fn without_hints_the_constructor_still_runs() {
    let handle = init(&instance_json(), &[]);
    let placed = placements(handle);
    swarm_free(handle);
    assert_eq!(placed.len(), 4, "힌트가 없어도 네 조각이 모두 배치되어야 한다");
}

/// 실제 상용 마커 변환 파일로 확인한다. `HINT_JSON` 이 있을 때만 돈다.
///
/// 힌트가 살아 있으면 초기 해 밀도가 상용 배치(약 78.8%)에 가깝고,
/// 버려졌다면 LBF 배치(약 72%)가 나온다. 그 둘은 멀리 떨어져 있어 오판할 여지가 없다.
#[test]
#[ignore = "HINT_JSON 환경변수 필요"]
fn real_marker_hints_survive_init() {
    let Ok(path) = std::env::var("HINT_JSON") else {
        println!("HINT_JSON 없음 — 건너뜀");
        return;
    };
    let raw = std::fs::read_to_string(&path).expect("read json");
    let doc: serde_json::Value = serde_json::from_str(&raw).expect("parse json");
    let items = doc["items"].as_array().expect("items");

    let hints: Vec<CPlacedHint> = items
        .iter()
        .filter_map(|it| {
            let h = it.get("placed_hint")?.as_object()?;
            Some(hint(
                it["id"].as_i64()? as i32,
                h["tr_x"].as_f64()? as f32,
                h["tr_y"].as_f64()? as f32,
            ))
        })
        .collect();
    println!("힌트 {}개 / 아이템 {}개", hints.len(), items.len());
    assert!(!hints.is_empty(), "이 파일에는 placed_hint 가 없다");

    let handle = init_with(&doc, &hints, Some(&marker_config()));
    let ptr = swarm_get_solution_json(handle);
    let text = unsafe { CStr::from_ptr(ptr) }.to_str().unwrap().to_owned();
    swarm_free_string(ptr);
    swarm_free(handle);
    let sol: serde_json::Value = serde_json::from_str(&text).unwrap();

    let width = sol["strip_width"].as_f64().unwrap() as f32;
    let placed = sol["layout"]["placed_items"].as_array().unwrap().len();
    let strip_height = doc["strip_height"].as_f64().unwrap() as f32;
    // 조각 면적 합은 입력에서 직접 잰다.
    let area: f32 = items
        .iter()
        .map(|it| {
            let p: Vec<(f32, f32)> = it["shape"]["data"]
                .as_array()
                .unwrap()
                .iter()
                .map(|q| (q[0].as_f64().unwrap() as f32, q[1].as_f64().unwrap() as f32))
                .collect();
            let mut s = 0.0;
            for i in 0..p.len() {
                let j = (i + 1) % p.len();
                s += p[i].0 * p[j].1 - p[j].0 * p[i].1;
            }
            s.abs() / 2.0
        })
        .sum();
    let density = 100.0 * area / (width * strip_height);
    println!("초기 해: {placed}개 배치, 폭 {width:.1}, 밀도 {density:.2}%");

    assert_eq!(placed, items.len(), "모든 조각이 배치되어야 한다");
    assert!(
        density > 76.0,
        "힌트가 버려지고 LBF 가 다시 배치한 것으로 보인다 (밀도 {density:.2}%, 상용 배치는 약 78.8%)"
    );
}
