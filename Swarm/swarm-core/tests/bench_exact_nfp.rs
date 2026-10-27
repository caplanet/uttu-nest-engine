//! exact NFP 벤치마크: 실물 마커 여성WSjk_비교용-G로 LBF 초기 폭을 측정한다.
//!
//! 실행: cargo test --release -p swarm-core --features capmods --test bench_exact_nfp -- --nocapture

#![cfg(feature = "capmods")]

use jagua_rs::probs::spp::entities::SPInstance;
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::{DEFAULT_RASTER_NFP_CONFIG, DEFAULT_SWARM_CONFIG, RasterNfpConfig};
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::optimizer::lbf::LBFBuilder;
use swarm_core::optimizer::separator::Separator;
use swarm_core::util::io::read_spp_input;
use std::path::Path;

fn load(path: &str) -> SPInstance {
    let (ext_inst, _) =
        read_spp_input(Path::new(path)).unwrap_or_else(|e| panic!("failed to read {}: {e}", path));
    let cfg = DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(cfg.cde_config, cfg.poly_simpl_tolerance, cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&importer, &ext_inst)
        .expect("failed to import instance")
}

fn build_lbf(
    instance: &SPInstance,
    seed: u64,
    raster: RasterNfpConfig,
) -> (f32, usize, f32, std::time::Duration) {
    let t0 = std::time::Instant::now();
    let builder = LBFBuilder::new(
        instance.clone(),
        Xoshiro256PlusPlus::seed_from_u64(seed),
        LBF_SAMPLE_CONFIG,
        10.0,
        1.0,
    )
    .with_raster_config(raster)
    .construct();
    let elapsed = t0.elapsed();

    let width = builder.prob.strip_width();
    let n_placed = builder.prob.layout().placed_items().len();

    let sep = Separator::new(
        instance.clone(),
        builder.prob,
        Xoshiro256PlusPlus::seed_from_u64(seed + 1),
        DEFAULT_SWARM_CONFIG.expl_cfg.separator_config,
    );
    (width, n_placed, sep.ct.get_total_loss(), elapsed)
}

/// 장시간 벤치. **머신 고정 절대경로**를 읽으므로 기본 실행에서 빠진다.
///
/// 예전에는 `#[ignore]` 가 없어 `capmods` 를 켠 일반 테스트 실행에서 그대로 돌았고,
/// 그 경로가 없는 머신에서는 **panic** 했다. 지표도 다른 하네스와 달라(밀도 없음,
/// 5-seed 평균) 나란히 읽을 수 없다.
///
/// `SWARM_INPUT` 으로 경로를 바꿀 수 있다.
#[test]
#[ignore = "장시간 + 머신 고정 경로 — SWARM_INPUT 을 주고 --ignored 로 명시 실행"]
fn bench_exact_nfp_wsjk() {
    let instance_path = std::env::var("SWARM_INPUT").unwrap_or_else(|_| {
        r"C:\Caplanet\TestFiles\여성WSjk_비교용-G_Swarm.json".to_string()
    });
    let input_path: &str = &instance_path;
    const N_SEEDS: u64 = 5;

    let instance = load(input_path);
    let n_items = instance.total_item_qty();
    println!("instance: {input_path}, items: {n_items}");

    // OFF: raster NFP off (baseline — 난수 샘플만 사용)
    let cfg_off = RasterNfpConfig {
        enabled: false,
        ..DEFAULT_RASTER_NFP_CONFIG
    };

    // ON: raster + exact NFP (현재 코드)
    let cfg_on = RasterNfpConfig {
        enabled: true,
        n_candidates: 64,
        n_random_samples: 1000, // baseline과 동일 예산
        ..DEFAULT_RASTER_NFP_CONFIG
    };

    // baseline (off)
    let mut w_off = 0.0;
    let mut t_off = std::time::Duration::ZERO;
    for seed in 0..N_SEEDS {
        let (w, placed, loss, t) = build_lbf(&instance, seed, cfg_off);
        assert_eq!(loss, 0.0, "seed {seed}: baseline must be feasible");
        assert_eq!(placed, n_items, "seed {seed}: all items must be placed");
        w_off += w;
        t_off += t;
    }
    w_off /= N_SEEDS as f32;
    let ms_off = t_off.as_secs_f32() * 1000.0 / N_SEEDS as f32;

    // raster + exact NFP (on)
    let mut w_on = 0.0;
    let mut t_on = std::time::Duration::ZERO;
    for seed in 0..N_SEEDS {
        let (w, placed, loss, t) = build_lbf(&instance, seed, cfg_on);
        assert_eq!(loss, 0.0, "seed {seed}: raster+exact must be feasible");
        assert_eq!(placed, n_items, "seed {seed}: all items must be placed");
        w_on += w;
        t_on += t;
    }
    w_on /= N_SEEDS as f32;
    let ms_on = t_on.as_secs_f32() * 1000.0 / N_SEEDS as f32;

    println!();
    println!("═══════════════════════════════════════");
    println!("  instance: 여성WSjk_비교용-G ({n_items} items)");
    println!("  seeds: {N_SEEDS}");
    println!("───────────────────────────────────────");
    println!(
        "  baseline (raster OFF):   {:>10.1} mm  ({:>6.0} ms)",
        w_off, ms_off
    );
    println!(
        "  raster + exact NFP:      {:>10.1} mm  ({:>6.0} ms)",
        w_on, ms_on
    );
    println!("───────────────────────────────────────");
    let delta = w_on - w_off;
    let pct = (w_on / w_off - 1.0) * 100.0;
    println!(
        "  Δ width:                 {:+10.1} mm  ({:+.2}%)",
        delta, pct
    );
    let dt = ms_on - ms_off;
    println!("  Δ time:                  {:+10.0} ms", dt);
    println!("═══════════════════════════");
}
