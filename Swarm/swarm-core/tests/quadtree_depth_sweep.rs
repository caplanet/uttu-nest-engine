//! **쿼드트리 해상도 재보정** — `quadtree_depth` 와 `cd_threshold` 를 실측한다.
//!
//! # 왜 이 하네스인가
//!
//! 업스트림이 `cd_threshold` 를 16 → 64 로 올렸는데, 우리 인스턴스에서는 그 값이
//! 아예 걸리지 않는다. wsjk156 은 리프당 약 523 간선으로 34개 인스턴스 중 가장
//! 조밀해서, 임계값이 16이든 64이든 항상 리프까지 내려가 무차별 검사한다.
//!
//! 즉 우리 문제는 임계값이 아니라 **해상도**다. 깊이 4에서 리프 하나가 523 간선을
//! 떠안는다는 것은 쿼드트리가 가지치기를 거의 못 하고 있다는 뜻이다. 깊이를 올리면
//! 리프당 간선이 4배씩 줄어든다(추정: 5→131, 6→35, 7→9).
//!
//! 이 값은 탐지 결과를 바꾸지 않는 **순수 성능 노브**다. 따라서 1차 효과는 밀도가
//! 아니라 처리량이고, 밀도는 "같은 시간 예산에 반복을 더 많이 돌아서" 따라오는
//! 2차 효과다. 그래서 최종 밀도와 함께 탐색 개선 횟수도 같이 찍는다.
//!
//! ```powershell
//! $env:QT_DEPTHS='4,5,6'; $env:QT_SECS='120'; $env:QT_CMPR_SECS='30'; $env:QT_SEEDS='0,1,2'
//! cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" `
//!   --test quadtree_depth_sweep -- --ignored --nocapture --test-threads=1
//! ```
//!
//! 노브: `QT_DEPTHS`(쉼표 구분) · `QT_CD`(cd_threshold, 기본 설정값) ·
//! `QT_SECS` · `QT_CMPR_SECS` · `QT_SEEDS` · `SWARM_INPUT`.
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
fn load_with(path: &Path, depth: u8, cd_threshold: u8) -> SPInstance {
    let (ext, _) = read_spp_input(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let c = DEFAULT_SWARM_CONFIG;
    let mut cde = c.cde_config;
    cde.quadtree_depth = depth;
    cde.cd_threshold = cd_threshold;
    let imp = swarm_core::util::io::new_importer(cde, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&imp, &ext).expect("import")
}

#[test]
#[ignore = "실측 하네스 — 깊이 x 시드 x (탐색 + 압축) 예산만큼 돈다"]
fn quadtree_resolution_sweep() {
    let path = input_path();
    let c = DEFAULT_SWARM_CONFIG;
    let depths: Vec<u8> = env_list("QT_DEPTHS", "4,5,6").into_iter().map(|v| v as u8).collect();
    let cd = env_u64("QT_CD", c.cde_config.cd_threshold as u64) as u8;
    let expl_secs = env_u64("QT_SECS", 120);
    let cmpr_secs = env_u64("QT_CMPR_SECS", 30);
    let seeds: Vec<u64> = env_list("QT_SEEDS", "0,1,2");

    println!("\n══════ 쿼드트리 해상도 스윕 ══════");
    println!("  입력     {}", path.display());
    println!("  기준     현재 기본값 quadtree_depth={} cd_threshold={}",
             c.cde_config.quadtree_depth, c.cde_config.cd_threshold);
    println!("  고정     cd_threshold={cd}");
    println!("  예산     탐색 {expl_secs}s + 압축 {cmpr_secs}s · seed {seeds:?}");
    println!("  판정     최종 밀도(±0.4%p 밴드). 개선 횟수는 처리량 대리 지표다.\n");

    for &depth in &depths {
        let inst = load_with(&path, depth, cd);
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
                "  depth {depth}  seed {seed}  LBF {lbf_secs:5.2}s  탐색 {:6.2}% ({:3}개 개선)  최종 {fd:6.2}%",
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
            "  depth {depth}  평균 최종 {:.2}%  (최소 {min:.2} / 최대 {max:.2})  개선 평균 {:.1}회  LBF 평균 {:.2}s\n",
            mean(&finals),
            mean(&improvements),
            mean(&lbf_secs_all)
        );
    }

    println!("  읽는 법: 깊이를 올려 처리량이 오르면 '개선 횟수'가 먼저 늘고,");
    println!("          최종 밀도는 그 뒤를 따라온다. 개선 횟수가 그대로면 병목이 충돌 검출이 아니다.\n");
}
