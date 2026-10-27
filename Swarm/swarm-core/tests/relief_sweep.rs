//! **적응 탐색 노브 A/B** — 큰 걸음(`shrink_step`)과 폭 완화 복구(`relief_*`)를 환경변수로
//! 켜서, 실제 파이프라인(적응 탐색 → 압축)의 최종 밀도를 잰다.
//!
//! # 왜 이 하네스인가
//!
//! 폭 완화 복구는 "충돌을 풀며 넓히는" 기구인데 **기본 꺼짐**이고 켠 실측이 없다.
//! `lbf_concavity_order` 는 기본 설정만 돌리므로 노브를 바꿀 수 없다. 여기서는
//! 한 번에 팔 하나만 돌리고(노브 하나 = 팔 하나), 쉘에서 팔을 이어 붙인다.
//!
//! ```powershell
//! # 기준
//! $env:SWEEP_SECS='480'; $env:SWEEP_CMPR_SECS='120'; $env:SWEEP_SEEDS='0,1,2'
//! cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" `
//!   --test relief_sweep -- --ignored --nocapture --test-threads=1
//! # 큰 걸음 + 완화
//! $env:SWEEP_SHRINK='400'; $env:RELIEF_FAILS='2'; $env:RELIEF_ROUND_MS='2000'
//! cargo test ... --test relief_sweep -- --ignored --nocapture --test-threads=1
//! ```
//!
//! 노브: `SWEEP_SHRINK`(x100000, 기본 설정값) · `RELIEF_FAILS`(0=끔) · `RELIEF_STEP`(x1000) ·
//! `RELIEF_MAX`(x1000) · `RELIEF_ROUNDS` · `RELIEF_STRIKES` · `RELIEF_ROUND_MS`.
#![cfg(feature = "adaptive")]

use jagua_rs::probs::spp::entities::{SPInstance, SPProblem};
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::optimizer::lbf::LBFBuilder;
use swarm_core::optimizer::separator::Separator;
use swarm_core::util::io::read_spp_input;
use swarm_core::util::terminator::{BasicTerminator, Terminator};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

fn env_u64(key: &str, default: u64) -> u64 {
    std::env::var(key).ok().and_then(|v| v.trim().parse().ok()).unwrap_or(default)
}

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
    jagua_rs::probs::spp::io::import_instance(&imp, &ext).expect("import")
}

#[test]
#[ignore = "실측 하네스 — 시드 x (탐색 + 압축) 예산만큼 돈다"]
fn adaptive_exploration_knob_arm() {
    let path = input_path();
    let inst = load(&path);
    let c = DEFAULT_SWARM_CONFIG;
    let expl_secs = env_u64("SWEEP_SECS", 45);
    let cmpr_secs = env_u64("SWEEP_CMPR_SECS", 15);
    let seeds: Vec<u64> = std::env::var("SWEEP_SEEDS")
        .unwrap_or_else(|_| "0,1,2".into())
        .split(',')
        .filter_map(|v| v.trim().parse().ok())
        .collect();

    // ── 노브 ──
    let mut expl = c.expl_cfg;
    expl.time_limit = Duration::from_secs(expl_secs);
    if env_u64("SWEEP_SHRINK", 0) > 0 {
        expl.shrink_step = env_u64("SWEEP_SHRINK", 0) as f32 / 100_000.0;
    }
    expl.relief_fail_threshold = env_u64("RELIEF_FAILS", 0) as usize;
    if env_u64("RELIEF_STEP", 0) > 0 {
        expl.relief_step = env_u64("RELIEF_STEP", 0) as f32 / 1000.0;
    }
    if env_u64("RELIEF_MAX", 0) > 0 {
        expl.relief_max_ratio = env_u64("RELIEF_MAX", 0) as f32 / 1000.0;
    }
    if env_u64("RELIEF_ROUNDS", 0) > 0 {
        expl.relief_max_rounds = env_u64("RELIEF_ROUNDS", 0) as usize;
    }
    if env_u64("RELIEF_STRIKES", 0) > 0 {
        expl.relief_strike_limit = env_u64("RELIEF_STRIKES", 0) as usize;
    }
    expl.relief_round_budget = Duration::from_millis(env_u64("RELIEF_ROUND_MS", 0));
    let mut cmpr = c.cmpr_cfg;
    cmpr.time_limit = Duration::from_secs(cmpr_secs);

    println!("\n══════ 적응 탐색 노브 팔 ══════");
    println!("  입력     {}", path.display());
    println!(
        "  노브     shrink_step {:.5} · relief fails={} step={:.4} max={:.3} rounds={} strikes={} round_budget={:?}",
        expl.shrink_step,
        expl.relief_fail_threshold,
        expl.relief_step,
        expl.relief_max_ratio,
        expl.relief_max_rounds,
        expl.relief_strike_limit,
        expl.relief_round_budget
    );
    println!("  예산     탐색 {expl_secs}s + 압축 {cmpr_secs}s · seed {seeds:?}");
    println!("  밀도     fit_strip 후 SPSolution::density · 실행 간 분산 ±0.4%p\n");

    let mut finals = Vec::new();
    for &seed in &seeds {
        let lbf = LBFBuilder::new(
            inst.clone(),
            Xoshiro256PlusPlus::seed_from_u64(seed),
            LBF_SAMPLE_CONFIG,
            c.lbf_x_multiplier,
            c.lbf_y_multiplier,
        )
        .construct()
        .prob
        .save();

        let mut prob = SPProblem::new(inst.clone()).expect("strip forms a valid container");
        prob.restore(&lbf);
        let mut sep = Separator::new(
            inst.clone(),
            prob,
            Xoshiro256PlusPlus::seed_from_u64(seed + 1),
            expl.separator_config,
        );
        let t = Instant::now();
        let mut term = BasicTerminator::new();
        term.new_timeout(expl.time_limit);
        let sols = swarm_core::optimizer::explore_mods::exploration_phase_adaptive(
            &inst, &mut sep, &mut None, &term, &expl,
        );
        let expl_sol = sols.last().cloned().unwrap_or_else(|| lbf.clone());
        let expl_secs_used = t.elapsed().as_secs_f32();

        sep.change_strip_width(expl_sol.strip_width(), None);
        sep.rollback(&expl_sol, None);
        let mut term = BasicTerminator::new();
        term.new_timeout(cmpr.time_limit);
        let fin = swarm_core::optimizer::compress::compression_phase(
            &inst, &mut sep, &expl_sol, &mut None, &term, &cmpr,
        );
        let fd = fin.density() * 100.0;
        println!(
            "  seed {seed}  시드 {:8.1}  탐색 {:8.1} ({:.2}%, {}개 개선, {expl_secs_used:.0}s)  최종 {:8.1}  {fd:6.2}%",
            lbf.strip_width(),
            expl_sol.strip_width(),
            expl_sol.density() * 100.0,
            sols.len(),
            fin.strip_width()
        );
        finals.push(fd);
    }
    let mean = finals.iter().sum::<f32>() / finals.len().max(1) as f32;
    let min = finals.iter().cloned().fold(f32::INFINITY, f32::min);
    let max = finals.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
    println!("\n  최종 평균 {mean:.2}%  (최소 {min:.2} / 최대 {max:.2})\n");
}
