//! **선택 구조(Global, 창 K) vs 정렬 구조(Sorted) 시드 밀도** — `initial/attach.rs` 엔진 재측정.
//!
//! `Structure::Global` 은 "미배치 전체 x 배치 전체 후보에서 전역 최선 1개" 다. 순수 전역
//! 선택은 위치 항이 작은 조각을 항상 고르므로 64% 로 무너진다(실측). `pool_window` 로
//! 정렬 상위 K 개 안에서만 고르면 "큰 것부터" 구조를 유지하며 창 안에서 최적 맞춤을 한다.
//!
//! ```powershell
//! $env:BF_SEEDS='0,1,2'; $env:BF_WINDOWS='1,3,6,12,24'; $env:BF_STAGE='S2C'   # 또는 D / F
//! cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" `
//!   --test bestfit_probe -- --ignored --nocapture --test-threads=1
//! ```
use jagua_rs::probs::spp::entities::SPInstance;
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::initial::{AttachCfg, CompactCfg, Stage, Structure, build_initial_staged_cc};
use swarm_core::util::io::read_spp_input;
use std::path::Path;

fn load() -> SPInstance {
    let path = std::env::var("SWARM_INPUT").map(std::path::PathBuf::from)
        .unwrap_or_else(|_| Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input/wsjk156.json"));
    let (ext, _) = read_spp_input(&path).expect("read");
    let c = DEFAULT_SWARM_CONFIG;
    let imp = swarm_core::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&imp, &ext).expect("import")
}

fn env_list(key: &str, default: &str) -> Vec<usize> {
    std::env::var(key).unwrap_or(default.into()).split(',').filter_map(|v| v.trim().parse().ok()).collect()
}

fn stage() -> Stage {
    match std::env::var("BF_STAGE").as_deref() { Ok("D") => Stage::D, Ok("F") => Stage::F, _ => Stage::S2C }
}

#[test]
#[ignore = "실측 프로브"]
fn global_selection_vs_sorted_order() {
    let inst = load();
    let seeds: Vec<u64> = env_list("BF_SEEDS", "0,1,2").into_iter().map(|v| v as u64).collect();
    // 팔: "구조:창:크기편향:좌하단배율" 목록 (구조 S=Sorted, G=Global). 창 0 = 전체, 편향 0 = 끔.
    let arms_env = std::env::var("BF_ARMS").unwrap_or("S:0:0:1,G:0:1:0,G:12:1:0".into());
    let cap = env_list("BF_CAP", "32")[0];
    let raster_cell: f32 = std::env::var("BF_RASTER").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0);
    let shadow_cap = env_list("BF_SHADOW", "64")[0];
    let st = stage();
    println!("\n══════ 구조/창/편향/배율 팔 · stage {st:?} · cap {cap} · 래스터 {raster_cell}mm (0=bbox 낭비) · 그림자 상한 {shadow_cap} ══════");
    let mut arms: Vec<(String, Structure, usize, f32, f32)> = Vec::new();
    for a in arms_env.split(',') {
        let parts: Vec<&str> = a.split(':').map(|v| v.trim()).collect();
        if parts.len() != 4 { continue; }
        let structure = if parts[0] == "S" { Structure::Sorted } else { Structure::Global };
        let f: Vec<f32> = parts[1..].iter().filter_map(|v| v.parse().ok()).collect();
        if f.len() == 3 { arms.push((format!("{} 창{:>3} 편향{:>4} 배율{:>3}", parts[0], f[0], f[1], f[2]), structure, f[0] as usize, f[1], f[2])); }
    }
    for (name, structure, window, bias, scale) in arms {
        let mut ds = Vec::new();
        for &seed in &seeds {
            let mut cfg = AttachCfg::n02c();
            cfg.structure = structure;
            cfg.pool_window = window;
            cfg.select_size_bias = bias;
            cfg.select_lbf_scale = scale;
            cfg.raster_fit_cell = raster_cell;
            cfg.raster_shadow_cap = shadow_cap;
            cfg.lbf_x_weight = 500.0;
            cfg.lbf_y_weight = 0.1;
            cfg.cand_eval_cap = cap;
            let mut rng = Xoshiro256PlusPlus::seed_from_u64(seed);
            let (_sol, s) = build_initial_staged_cc(&inst, &cfg, &CompactCfg::default(), st, &mut rng);
            println!("  {name:22} seed {seed}  placed {}/{}  dens {:6.2}%  width {:8.1}  {:6.2}s  evals {}", s.placed, s.total, s.density, s.width, s.secs, s.cands_evaluated);
            ds.push(s.density);
        }
        println!("  {name:22} 평균 {:6.2}%", ds.iter().sum::<f32>() / ds.len() as f32);
    }
}
