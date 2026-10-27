//! **"목표 폭으로 점프해 놓고 충돌을 풀며 넓히기" 는 되는가** — 안→바깥 궤적의 실현 가능성 프로브.
//!
//! # 무엇을 재나
//!
//! 현재 탐색은 feasible 시드에서 0.1%씩 조이고 실패하면 롤백하는 **바깥→안** 궤적이다.
//! 제안된 방향은 반대다 — 목표 폭으로 **한 번에** 조여 전 조각이 겹친 상태를 만들고,
//! 분리기가 풀지 못하면 폭을 조금씩 **넓혀 가며** 재분리한다(Egeblad/Umetani 계열의
//! 고정 길이 겹침 최소화 + 길이 조정).
//!
//! 엔진(`Separator::separate`)은 두 궤적이 같다. 다른 것은 바깥 루프뿐이므로, 같은
//! 시드·같은 예산에서 **최종 feasible 폭**을 나란히 놓으면 방향의 우열이 갈린다.
//!
//! 두 가지를 찍는다.
//!
//! 1. 점프 직후 한 번의 분리가 겹침을 **얼마나** 푸는가 (진입 손실 → 잔여 손실).
//!    큰 점프가 규모에서 막힌다는 기존 관찰(shirts −20% 미해소)을 이 인스턴스에서 확인한다.
//! 2. 넓히며 재분리해 feasible 에 닿은 폭 vs 같은 예산의 바깥→안 탐색이 닿은 폭.
//!
//! ```powershell
//! $env:JUMP_TARGETS='0.95,0.90,0.84'; $env:JUMP_SECS='60'; $env:JUMP_SEEDS='0,1,2'
//! cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" `
//!   --test target_width_jump -- --ignored --nocapture --test-threads=1
//! ```
//!
//! `JUMP_TARGETS` 는 LBF 시드 폭 대비 비율이다. wsjk156 에서 0.84 가 상용 밀도(82.66%) 폭이다.
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

fn env_list_f32(key: &str, default: &str) -> Vec<f32> {
    std::env::var(key)
        .unwrap_or_else(|_| default.to_string())
        .split(',')
        .filter_map(|v| v.trim().parse().ok())
        .collect()
}

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

fn separator_from(inst: &SPInstance, seed: &jagua_rs::probs::spp::entities::SPSolution, rng: u64) -> Separator {
    let mut prob = SPProblem::new(inst.clone()).expect("strip forms a valid container");
    prob.restore(seed);
    Separator::new(
        inst.clone(),
        prob,
        Xoshiro256PlusPlus::seed_from_u64(rng),
        DEFAULT_SWARM_CONFIG.expl_cfg.separator_config,
    )
}

fn term_for(remaining: Duration) -> BasicTerminator {
    let mut t = BasicTerminator::new();
    t.new_timeout(remaining);
    t
}

#[test]
#[ignore = "실측 프로브 — 목표 x 시드 x 예산만큼 돈다"]
fn can_the_separator_dig_out_of_a_target_width_jump() {
    let path = input_path();
    let inst = load(&path);
    let c = DEFAULT_SWARM_CONFIG;
    let targets = env_list_f32("JUMP_TARGETS", "0.95,0.90,0.84");
    let secs = env_u64("JUMP_SECS", 60);
    let widen = env_list_f32("JUMP_WIDEN", "0.01")[0];
    let seeds: Vec<u64> = env_list_f32("JUMP_SEEDS", "0,1,2").into_iter().map(|v| v as u64).collect();
    let budget = Duration::from_secs(secs);
    let height = inst.base_strip.fixed_height;
    let area: f32 = inst.items.iter().map(|(it, q)| it.shape_orig().area() * *q as f32).sum();
    let density_at = |w: f32| area / (w * height) * 100.0;

    println!("\n══════ 목표 폭 점프 프로브 ══════");
    println!("  입력    {}", path.display());
    println!("  예산    팔마다 {secs}s · 넓히기 걸음 {:.1}% · seed {seeds:?}", widen * 100.0);
    println!("  이론하한 폭 {:.1} (밀도 100%)\n", area / height);

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
        let w0 = lbf.strip_width();
        println!("── seed {seed}: LBF 폭 {w0:.1} ({:.2}%)", density_at(w0));

        // 기준: 바깥→안 (현재 탐색), 같은 예산.
        {
            let mut sep = separator_from(&inst, &lbf, seed + 1);
            let term = term_for(budget);
            let t = Instant::now();
            let sols = swarm_core::optimizer::explore_mods::exploration_phase_adaptive(
                &inst, &mut sep, &mut None, &term, &c.expl_cfg,
            );
            let best = sols.last().map(|s| s.strip_width()).unwrap_or(w0);
            println!(
                "   바깥→안 (기준)          최종 feasible 폭 {best:9.1} ({:.2}%)   {}개 개선   {:.0}s",
                density_at(best),
                sols.len(),
                t.elapsed().as_secs_f32()
            );
        }

        // 제안: 목표 폭으로 점프 → 분리 → 안 풀리면 넓히며 재분리.
        for &f in &targets {
            let target = w0 * f;
            let mut sep = separator_from(&inst, &lbf, seed + 1);
            let start = Instant::now();
            sep.change_strip_width(target, None);
            let entry = sep.ct.get_total_loss();
            let (_, snap) = sep.separate(&term_for(budget), &mut None);
            let first_residual = snap.get_total_loss();
            let first_secs = start.elapsed().as_secs_f32();
            let resolved_frac = if entry > 0.0 { 1.0 - first_residual / entry } else { 1.0 };

            let mut w = target;
            let mut rounds = 0usize;
            let mut feasible = first_residual == 0.0;
            while !feasible {
                let Some(remaining) = budget.checked_sub(start.elapsed()) else { break };
                if remaining < Duration::from_millis(200) {
                    break;
                }
                w *= 1.0 + widen;
                rounds += 1;
                // 분리기는 실패 시 최선(최소 손실) 배치로 되돌아가 있다 — 거기서 넓힌다.
                sep.change_strip_width(w, None);
                let (_, snap) = sep.separate(&term_for(remaining), &mut None);
                feasible = snap.get_total_loss() == 0.0;
            }
            let verdict = if feasible {
                format!("feasible 폭 {w:9.1} ({:.2}%)  넓힘 {rounds}회", density_at(w))
            } else {
                format!("**미해소** (마지막 폭 {w:9.1}, 넓힘 {rounds}회)")
            };
            println!(
                "   점프 x{f:.2} → {target:8.1} ({:.2}%)  진입손실 {entry:9.1} → 첫 분리 잔여 {first_residual:8.1} ({:.1}% 해소, {first_secs:.0}s)   {verdict}   {:.0}s",
                density_at(target),
                resolved_frac * 100.0,
                start.elapsed().as_secs_f32()
            );
        }
        println!();
    }
    println!("  읽는 법: 점프 팔의 feasible 폭이 기준(바깥→안)보다 좁으면 방향이 유리한 것이다.");
    println!("          '첫 분리 해소율' 이 낮으면 분리기가 대량 겹침에서 기울기를 잃는다는 뜻이다.\n");
}
