#![cfg(feature = "adaptive")]
//! 폭 완화(width relief)를 켠 채 축소 스텝을 바꿔 가며 탐색 결과 폭을 재는 진단.
//!
//! ```powershell
//! $env:SWARM_INPUT='C:\Caplanet\AiWorks\InputData\여성WSjk_비교용-G_New_SwarmV1.json'
//! cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" `
//!   --test width_relief_probe -- --ignored --nocapture --test-threads=1
//! ```

use jagua_rs::probs::spp::entities::SPInstance;
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::optimizer::lbf::LBFBuilder;
use swarm_core::optimizer::separator::Separator;
use swarm_core::util::io::read_spp_input;
use swarm_core::util::terminator::{BasicTerminator, Terminator};
use std::path::{Path, PathBuf};
use std::time::Duration;

fn input_path() -> PathBuf {
    match std::env::var("SWARM_INPUT") {
        Ok(p) => PathBuf::from(p),
        Err(_) => Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../data/input")
            .join("wsjk156.json"),
    }
}

fn load(path: &Path) -> SPInstance {
    let (ext, _) = read_spp_input(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let c = DEFAULT_SWARM_CONFIG;
    let imp = swarm_core::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&imp, &ext).unwrap()
}

fn run_once(inst: &SPInstance, relief: usize, secs: u64, shrink: f32) {
    let mut cfg = DEFAULT_SWARM_CONFIG.expl_cfg;
    cfg.relief_fail_threshold = relief;
    cfg.time_limit = Duration::from_secs(secs);
    if shrink > 0.0 {
        cfg.shrink_step = shrink;
    }

    let rng = Xoshiro256PlusPlus::seed_from_u64(0);
    let built = LBFBuilder::new(
        inst.clone(),
        rng,
        LBF_SAMPLE_CONFIG,
        DEFAULT_SWARM_CONFIG.lbf_x_multiplier,
        DEFAULT_SWARM_CONFIG.lbf_y_multiplier,
    )
    .construct();
    let mut sep = Separator::new(
        inst.clone(),
        built.prob,
        Xoshiro256PlusPlus::seed_from_u64(1),
        cfg.separator_config,
    );
    let mut term = BasicTerminator::new();
    term.new_timeout(cfg.time_limit);
    let sols = swarm_core::optimizer::explore_mods::exploration_phase_adaptive(
        inst, &mut sep, &mut None, &term, &cfg,
    );
    let width = sols.last().map(|s| s.strip_width()).unwrap_or(f32::NAN);
    println!("  shrink={shrink:<7} ReliefFail={relief:<3} -> 폭 {width:9.2}");
}

#[test]
#[ignore = "분 단위 — --ignored 로 명시 실행"]
fn does_width_relief_actually_fire() {
    let path = input_path();
    let inst = load(&path);
    println!("\n══ 폭 완화 발동 진단 ══");
    println!("  입력 {}", path.display());
    println!(
        "  조각 {} / strip 높이 {}",
        inst.items.iter().map(|(_, q)| *q).sum::<usize>(),
        inst.base_strip.fixed_height
    );
    let secs: u64 = std::env::var("EXPLORE_S")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(20);
    // ── 축소 스텝 스윕 ──
    //
    // 앞선 진단에서 49번 축소하는 동안 실패가 1번뿐이었다. 거의 항상 성공한다는 뜻이고,
    // 그러면 (a) 폭 완화는 발동할 이유가 없고 (b) 더 크게 줄여도 성공할 여지가 있다.
    // 병목이 "막힘" 이 아니라 "축소 속도" 라는 가설을 여기서 판정한다.
    println!("
  ── 축소 스텝 스윕 (완화 4 고정) ──");
    for shrink in [0.001f32, 0.003, 0.006, 0.012] {
        run_once(&inst, 4, secs, shrink);
    }
}
