//! **fail-fast 폴 커버리지 A/B** — `ff_pole_area_ratio` 를 실측한다.
//!
//! # 왜 이 하네스인가
//!
//! jagua-rs 0.8 은 fail-fast 폴을 개수가 아니라 **면적 비율**로 고른다. 0.7.2 승급 시
//! 동작 보존을 위해 `0.001`(= 기존 `n_ff_poles: 1` 과 동치)로 두었지만, 상류 swarm 는
//! `0.5` 를 쓴다. 흥미롭게도 우리 분리기 핫루프(`specialized_jaguars_pipeline.rs`)는
//! 이미 "면적 50% 까지 폴을 훑는다" 는 같은 규칙을 하드코딩하고 있다 — 즉 0.5 는
//! 신설 값이 아니라 서로게이트 설정을 우리 핫루프와 일치시키는 값이다.
//!
//! 폴은 조각 내부에 완전히 포함된 원이라 폴 충돌은 항상 참 충돌이고, 뒤따르는 정확
//! 검사가 같은 답을 낸다. 따라서 이 노브는 **결과를 바꾸지 않는 속도 노브**이며
//! 1차 효과는 밀도가 아니라 `detect_surrogate_collision` 의 fast-reject 강도다.
//! 그 함수를 부르는 곳은 LBF·폴리시·구조 평가기 7곳이므로 **LBF 시간**을 먼저 본다.
//!
//! ```powershell
//! $env:FF_RATIOS='0.001,0.5'; $env:QT_SECS='120'; $env:QT_CMPR_SECS='30'; $env:QT_SEEDS='0,1,2'
//! cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" `
//!   --test ff_pole_sweep -- --ignored --nocapture --test-threads=1
//! ```
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

fn env_list(key: &str, default: &str) -> Vec<u64> {
    std::env::var(key)
        .unwrap_or_else(|_| default.to_string())
        .split(',')
        .filter_map(|v| v.trim().parse().ok())
        .collect()
}

fn input_path() -> PathBuf {
    match std::env::var("SWARM_INPUT") {
        Ok(p) => PathBuf::from(p),
        Err(_) => Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../data/input")
            .join("wsjk156.json"),
    }
}

/// 주어진 쿼드트리 설정으로 인스턴스를 임포트한다.
/// `cde_config` 는 임포트 시점에 굳으므로 팔마다 다시 임포트해야 한다.
fn load_with(path: &Path, ff_ratio: f32) -> SPInstance {
    let (ext, _) = read_spp_input(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let c = DEFAULT_SWARM_CONFIG;
    let mut cde = c.cde_config;
    cde.item_surrogate_config.ff_pole_area_ratio = ff_ratio;
    let imp = swarm_core::util::io::new_importer(cde, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&imp, &ext).expect("import")
}

#[test]
#[ignore = "실측 하네스 — 깊이 x 시드 x (탐색 + 압축) 예산만큼 돈다"]
fn ff_pole_area_ratio_sweep() {
    let path = input_path();
    let c = DEFAULT_SWARM_CONFIG;
    let ratios: Vec<f32> = std::env::var("FF_RATIOS")
        .unwrap_or_else(|_| "0.001,0.5".to_string())
        .split(',')
        .filter_map(|v| v.trim().parse().ok())
        .collect();
    let expl_secs = env_u64("QT_SECS", 120);
    let cmpr_secs = env_u64("QT_CMPR_SECS", 30);
    let seeds: Vec<u64> = env_list("QT_SEEDS", "0,1,2");

    println!("\n══════ ff_pole_area_ratio 스윕 ══════");
    println!("  입력     {}", path.display());
    println!("  기준     현재 기본값 ff_pole_area_ratio={} (상류 swarm 는 0.5)",
             c.cde_config.item_surrogate_config.ff_pole_area_ratio);
    println!("  예산     탐색 {expl_secs}s + 압축 {cmpr_secs}s · seed {seeds:?}");
    println!("  판정     최종 밀도(±0.4%p 밴드). 개선 횟수는 처리량 대리 지표다.\n");

    for &ratio in &ratios {
        let inst = load_with(&path, ratio);
        let mut finals = Vec::new();
        let mut improvements = Vec::new();
        let mut lbf_secs_all = Vec::new();

        for &seed in &seeds {
            let t_lbf = Instant::now();
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
            let lbf_secs = t_lbf.elapsed().as_secs_f32();

            let mut prob = SPProblem::new(inst.clone()).expect("strip forms a valid container");
            prob.restore(&lbf);
            let mut sep = Separator::new(
                inst.clone(),
                prob,
                Xoshiro256PlusPlus::seed_from_u64(seed + 1),
                c.expl_cfg.separator_config,
            );

            let mut expl = c.expl_cfg;
            expl.time_limit = Duration::from_secs(expl_secs);
            let mut term = BasicTerminator::new();
            term.new_timeout(expl.time_limit);
            let sols = swarm_core::optimizer::explore_mods::exploration_phase_adaptive(
                &inst, &mut sep, &mut None, &term, &expl,
            );
            let expl_sol = sols.last().cloned().unwrap_or_else(|| lbf.clone());

            let mut cmpr = c.cmpr_cfg;
            cmpr.time_limit = Duration::from_secs(cmpr_secs);
            sep.change_strip_width(expl_sol.strip_width(), None);
            sep.rollback(&expl_sol, None);
            let mut term = BasicTerminator::new();
            term.new_timeout(cmpr.time_limit);
            let fin = swarm_core::optimizer::compress::compression_phase(
                &inst, &mut sep, &expl_sol, &mut None, &term, &cmpr,
            );

            let fd = fin.density() * 100.0;
            println!(
                "  ratio {ratio:<5}  seed {seed}  LBF {lbf_secs:5.2}s  탐색 {:6.2}% ({:3}개 개선)  최종 {fd:6.2}%",
                expl_sol.density() * 100.0,
                sols.len()
            );
            finals.push(fd);
            improvements.push(sols.len() as f32);
            lbf_secs_all.push(lbf_secs);
        }

        let mean = |v: &[f32]| v.iter().sum::<f32>() / v.len().max(1) as f32;
        let min = finals.iter().cloned().fold(f32::INFINITY, f32::min);
        let max = finals.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        println!(
            "  ratio {ratio:<5}  평균 최종 {:.2}%  (최소 {min:.2} / 최대 {max:.2})  개선 평균 {:.1}회  LBF 평균 {:.2}s\n",
            mean(&finals),
            mean(&improvements),
            mean(&lbf_secs_all)
        );
    }

    println!("  읽는 법: 이 노브는 분리기를 건드리지 않는다 — 1차 지표는 LBF 시간이다.");
    println!("          밀도가 움직이면 그것은 fast-reject 강도가 배치 선택을 바꾼 2차 효과다.\n");
}
