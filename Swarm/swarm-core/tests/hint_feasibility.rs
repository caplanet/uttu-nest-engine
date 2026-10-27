//! **힌트 배치가 어느 겹침 허용에서 통과하는가** — 진단 하네스.
//!
//! # 왜 필요한가
//!
//! `placed_hint` 를 담은 입력을 Use Hints 켜고 실행해도 힌트가 무시되고 LBF 가 도는
//! 일이 있다. 원인은 힌트 자체가 아니라 **검증**이다. `src/ffi_step.rs` 의
//! `legacy_seed_problem` 이 156개를 전부 놓아 본 뒤
//! `partial_solution_validation_error` 로 레이아웃 전체를 본다. 한 쌍이라도 겹치면
//! `"seed layout is not exact-feasible"` 로 실패하고, 호출부는 frozen 힌트만 남겨
//! 다시 짓는다. **모든 힌트가 `is_freeze: 0` 이면 frozen 집합이 비어 있으므로
//! warm start 가 부분적으로 살아남는 게 아니라 통째로 버려진다** — 그래서 LBF 가 돈다.
//!
//! 상용 마커를 변환한 입력은 0.025mm 격자 양자화 때문에 접촉면에 미세 관통이 남는다
//! (wsjk156: 49쌍, 최대 0.0954mm). 게다가 CDE 가 도형을 부풀리므로 실제로 필요한
//! 허용값은 그 관통 깊이보다 크다. 이 하네스는 **어느 값부터 통과하는지**를 찾는다.
//!
//! ```powershell
//! $env:HINT_JSON='E:/CaPlanit/여성WSjk_비교용-G_Swarm_PLT변환.json'
//! $env:HINT_OVERLAPS='0,0.1,0.2,0.3,0.5,1.0'
//! cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" `
//!   --test hint_feasibility -- --ignored --nocapture --test-threads=1
//! ```
//!
//! `HINT_SIMPL=off` 로 주면 `poly_simpl_tolerance` 를 꺼서 그 영향도 분리해 볼 수 있다.
#![cfg(feature = "adaptive")]

use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::geo_traits::Transformable;
use jagua_rs::io::import::ext_to_int_transformation;
use jagua_rs::probs::spp::entities::{SPPlacement, SPProblem};
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::util::io::{import_spp_instance, read_spp_input};
use std::path::PathBuf;

/// 입력 JSON 에서 (item_id, tr_x, tr_y) 를 읽는다. 힌트가 없는 아이템은 건너뛴다.
fn read_hints(path: &PathBuf) -> Vec<(usize, f32, f32)> {
    let raw = std::fs::read_to_string(path).expect("read json");
    let doc: serde_json::Value = serde_json::from_str(&raw).expect("parse json");
    let instance = doc.get("instance").unwrap_or(&doc);
    let items = instance["items"].as_array().expect("items array");
    let mut out = Vec::new();
    for item in items {
        let Some(hint) = item.get("placed_hint").filter(|h| h.is_object()) else {
            continue;
        };
        let id = item["id"].as_u64().expect("item id") as usize;
        let rot = hint["rot_angle_in_rad"].as_f64().unwrap_or(0.0);
        assert!(rot.abs() < 1e-12, "이 하네스는 회전 없는 힌트만 다룬다");
        out.push((
            id,
            hint["tr_x"].as_f64().unwrap_or(0.0) as f32,
            hint["tr_y"].as_f64().unwrap_or(0.0) as f32,
        ));
    }
    out
}

#[test]
#[ignore = "진단 하네스 — HINT_JSON 필요"]
fn which_overlap_tolerance_lets_the_hinted_layout_through() {
    let Ok(path) = std::env::var("HINT_JSON") else {
        println!("HINT_JSON 이 없어 건너뜁니다");
        return;
    };
    let path = PathBuf::from(path);
    let overlaps: Vec<f32> = std::env::var("HINT_OVERLAPS")
        .unwrap_or_else(|_| "0,0.1,0.2,0.3,0.5,1.0".to_string())
        .split(',')
        .filter_map(|v| v.trim().parse().ok())
        .collect();
    let simpl_off = std::env::var("HINT_SIMPL").as_deref() == Ok("off");

    let hints = read_hints(&path);
    let (ext, _) = read_spp_input(&path).expect("read instance");
    let c = DEFAULT_SWARM_CONFIG;

    println!("\n══════ 힌트 배치 실현가능성 ══════");
    println!("  입력      {}", path.display());
    println!("  힌트      {}개", hints.len());
    println!(
        "  단순화    poly_simpl_tolerance = {}",
        if simpl_off { "None (꺼짐)".to_string() } else { format!("{:?}", c.poly_simpl_tolerance) }
    );
    println!("  판정      legacy_seed_problem 과 같은 검사(전 조각 배치 후 exact 검증)\n");

    for &overlap in &overlaps {
        // WPF 의 piece_overlap_mm 은 코어에서 min_item_separation = -overlap 이 된다.
        let sep = (overlap > 0.0).then_some(-overlap);
        let instance = import_spp_instance(
            &ext,
            c.cde_config,
            if simpl_off { None } else { c.poly_simpl_tolerance },
            sep,
            c.narrow_concavity_cutoff_ratio,
        )
        .expect("import");

        let mut prob = SPProblem::new(instance.clone()).expect("strip forms a valid container");
        // 힌트가 요구하는 폭까지 스트립을 넓힌다 (legacy_seed_problem 과 같은 처리).
        let mut rightmost = 0.0f32;
        let mut placements = Vec::with_capacity(hints.len());
        for &(id, tr_x, tr_y) in &hints {
            let item = instance.item(id);
            let raw = item.shape_orig().shape.bbox();
            let ext_transf =
                DTransformation::new(0.0, (tr_x - raw.x_min, tr_y - raw.y_min));
            let d_transf = ext_to_int_transformation(&ext_transf, &item.shape_orig().pre_transform);
            let shape = item.shape_cd().transform_clone(&d_transf.compose());
            rightmost = rightmost.max(shape.bbox().x_max);
            placements.push(SPPlacement { item_idx: id, d_transf });
        }
        if rightmost * 1.00001 > prob.strip_width() {
            prob.change_strip_width(rightmost * 1.00001).expect("strip width must be positive");
        }
        for p in &placements {
            prob.place_item(*p);
        }

        let sol = prob.save();
        let verdict = swarm_core::structure::pipeline::partial_solution_validation_error(
            &instance, &sol,
        );
        let density = sol.density() * 100.0;
        match verdict {
            None => println!(
                "  겹침 {overlap:>4} mm  → 통과 ✓   폭 {:.1}  밀도 {density:.2}%",
                sol.strip_width()
            ),
            Some(err) => {
                let text = format!("{err:?}");
                let short = text.chars().take(150).collect::<String>();
                println!("  겹침 {overlap:>4} mm  → 거부     {short}");
            }
        }
    }
    println!(
        "\n  통과한 값 중 가장 작은 것을 WPF 의 Piece Overlap (mm) 에 넣으면\n  \
         Use Hints 가 실제로 먹는다. 거부되면 156개가 통째로 버려지고 LBF 가 돈다.\n"
    );
}
