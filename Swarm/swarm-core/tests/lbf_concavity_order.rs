//! **오목도 정렬이 시드와 최종 밀도에 무엇을 하는가** — `LBF수정사항.md` 의 판정 하네스.
//!
//! # 왜 시드만 재면 안 되는가
//!
//! `scale-collapse` 브랜치의 실측(`DENSITY_GAP_ANALYSIS.md` §2-4)은 **초기 배치 개선이
//! 최종 밀도로 전달되지 않는다**는 것을 보였다 — 시드를 +3.5%p 올려도 60초 예산에서
//! 전부 씻기고, 가장 나쁜 시드(LBF 69.45%)가 최종은 가장 좋았다. 그러므로 오목도
//! 정렬의 판정은 **시드 밀도가 아니라 탐색·압축을 거친 밀도**로 해야 한다.
//!
//! 이 하네스는 모드마다 둘 다 찍는다.
//!
//! ```text
//! 모드            시드 폭 / 밀도        →  탐색 + 압축 뒤 폭 / 밀도
//! ```
//!
//! # 판정 기준
//!
//! * 실행 간 분산이 **±0.4%p** 다. 시드 3개 평균 차이가 그 안이면 잡음이다.
//! * 최종 밀도의 세 실행이 기준선 세 실행보다 **전부** 위면 평균 차이보다 강한 증거다.
//!
//! ```powershell
//! $env:SWARM_INPUT='C:\...\여성WSjk_비교용-G_New_SwarmV1.json'
//! $env:ORDER_EXPLORE_SECS='45'; $env:ORDER_COMPRESS_SECS='15'; $env:ORDER_SEEDS='0,1,2'
//! cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" `
//!   --test lbf_concavity_order -- --ignored --nocapture --test-threads=1
//! ```
//!
//! `ORDER_ALL=1` 이면 여섯 모드 전부, `ORDER_MODES=0,4` 처럼 raw 값을 주면 그것만,
//! 아니면 기준선·오목도부터·크기×오목도 셋을 돈다.

use jagua_rs::probs::spp::entities::{SPInstance, SPProblem, SPSolution};
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::optimizer::concavity::analyze_spolygon;
use swarm_core::optimizer::lbf::{LBFBuilder, LbfOrderMode};
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

/// 시드 -> 탐색 -> 압축. `budget_usage.rs` 와 같은 순서(step API 와 같다).
fn explore_and_compress(inst: &SPInstance, seed: &SPSolution, rng_seed: u64, expl: Duration, cmpr: Duration) -> SPSolution {
    let c = DEFAULT_SWARM_CONFIG;
    let mut prob = SPProblem::new(inst.clone()).expect("strip forms a valid container");
    prob.restore(seed);
    let mut sep = Separator::new(
        inst.clone(),
        prob,
        Xoshiro256PlusPlus::seed_from_u64(rng_seed + 1),
        c.expl_cfg.separator_config,
    );
    let mut term = BasicTerminator::new();
    term.new_timeout(expl);
    let feasible = swarm_core::optimizer::explore::exploration_phase(inst, &mut sep, &mut None, &term, &c.expl_cfg);
    let expl_sol = feasible.last().cloned().unwrap_or_else(|| seed.clone());

    sep.change_strip_width(expl_sol.strip_width(), None);
    sep.rollback(&expl_sol, None);
    let mut term = BasicTerminator::new();
    term.new_timeout(cmpr);
    swarm_core::optimizer::compress::compression_phase(inst, &mut sep, &expl_sol, &mut None, &term, &c.cmpr_cfg)
}

#[test]
#[ignore = "실측 하네스 — 모드 x 시드만큼 탐색 예산을 쓴다"]
fn does_concavity_ordering_survive_exploration() {
    let path = input_path();
    let inst = load(&path);
    let c = DEFAULT_SWARM_CONFIG;
    let expl = Duration::from_secs(env_u64("ORDER_EXPLORE_SECS", 45));
    let cmpr = Duration::from_secs(env_u64("ORDER_COMPRESS_SECS", 15));
    let seeds: Vec<u64> = std::env::var("ORDER_SEEDS")
        .unwrap_or_else(|_| "0,1,2".into())
        .split(',')
        .filter_map(|v| v.trim().parse().ok())
        .collect();
    let modes: Vec<LbfOrderMode> = if let Ok(list) = std::env::var("ORDER_MODES") {
        // raw ABI 값 목록, 예: "0,4". 10분 예산에서는 팔 수가 곧 시간이다.
        list.split(',').filter_map(|v| v.trim().parse::<i32>().ok()).map(LbfOrderMode::from_raw).collect()
    } else if env_u64("ORDER_ALL", 0) != 0 {
        vec![
            LbfOrderMode::LargestFirst,
            LbfOrderMode::SmallestFirst,
            LbfOrderMode::MiddleOutLargeFirst,
            LbfOrderMode::MiddleOutSmallFirst,
            LbfOrderMode::ConcavityFirst,
            LbfOrderMode::SizeTimesConcavity,
        ]
    } else {
        vec![LbfOrderMode::LargestFirst, LbfOrderMode::ConcavityFirst, LbfOrderMode::SizeTimesConcavity]
    };

    // ── 이 인스턴스의 오목도 분포. 점수가 전부 0 이면 정렬이 바뀔 것이 없다. ──
    let mut scores: Vec<(usize, f64, f64, f64)> = inst
        .items
        .iter()
        .map(|(item, _)| {
            let s = analyze_spolygon(&item.shape_orig().shape);
            (item.idx(), s.final_score, s.left_score, s.bottom_score)
        })
        .collect();
    scores.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    let positive = scores.iter().filter(|s| s.1 > 0.0).count();
    let mean = scores.iter().map(|s| s.1).sum::<f64>() / scores.len().max(1) as f64;

    println!("\n══════ 오목도 정렬 — 시드와 최종 밀도 ══════");
    println!("  입력    {}", path.display());
    println!(
        "  조각    {}종 / {}개 · 오목도 > 0 인 종류 {positive} · 평균 점수 {mean:.3}",
        inst.items.len(),
        inst.total_item_qty()
    );
    println!("  상위 5  {}", scores.iter().take(5).map(|(id, f, l, b)| format!("#{id}:{f:.2}(L{l:.2}/B{b:.2})")).collect::<Vec<_>>().join("  "));
    println!("  예산    탐색 {}s + 압축 {}s · seed {seeds:?} · 워커 {}", expl.as_secs(), cmpr.as_secs(), c.expl_cfg.separator_config.n_workers);
    println!("  밀도    fit_strip 후 SPSolution::density. 실행 간 분산 ±0.4%p\n");
    println!("  {:14} {:>4}  {:>9} {:>7}   {:>9} {:>7}", "모드", "seed", "시드폭", "밀도", "최종폭", "밀도");

    let mut summary: Vec<(LbfOrderMode, Vec<f32>, Vec<f32>)> = Vec::new();
    for mode in modes {
        let (mut seed_d, mut final_d) = (Vec::new(), Vec::new());
        for &seed in &seeds {
            let t = Instant::now();
            let mut built = LBFBuilder::new(
                inst.clone(),
                Xoshiro256PlusPlus::seed_from_u64(seed),
                LBF_SAMPLE_CONFIG,
                c.lbf_x_multiplier,
                c.lbf_y_multiplier,
            )
            .with_order_mode(mode)
            .with_raster_config(c.lbf_raster)
            .construct();
            built.prob.fit_strip().expect("strip width must be positive");
            let seed_sol = built.prob.save();
            let lbf_ms = t.elapsed().as_millis();
            let placed = seed_sol.layout_snapshot().placed_items().len();
            if placed != inst.total_item_qty() {
                println!("  ⚠ {} seed {seed}: 미배치 {} — 이 행의 밀도는 비교 불가", mode.label(), inst.total_item_qty() - placed);
            }
            let sd = seed_sol.density() * 100.0;
            let fin = explore_and_compress(&inst, &seed_sol, seed, expl, cmpr);
            let fd = fin.density() * 100.0;
            println!(
                "  {:14} {seed:>4}  {:9.2} {sd:6.2}%   {:9.2} {fd:6.2}%   (LBF {lbf_ms}ms)",
                mode.label(),
                seed_sol.strip_width(),
                fin.strip_width()
            );
            seed_d.push(sd);
            final_d.push(fd);
        }
        summary.push((mode, seed_d, final_d));
    }

    println!();
    let mean = |v: &[f32]| v.iter().sum::<f32>() / v.len().max(1) as f32;
    let base = summary.iter().find(|(m, _, _)| *m == LbfOrderMode::LargestFirst);
    println!("  {:14} {:>9} {:>9}   {:>9} {:>9}   {}", "모드", "시드평균", "vs기준", "최종평균", "vs기준", "판정");
    for (mode, sd, fd) in &summary {
        let (bs, bf) = base.map(|(_, s, f)| (mean(s), mean(f))).unwrap_or((mean(sd), mean(fd)));
        let d_final = mean(fd) - bf;
        let separated = base.map_or(false, |(_, _, f)| {
            let base_max = f.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
            fd.iter().all(|&x| x > base_max)
        });
        let verdict = if *mode == LbfOrderMode::LargestFirst {
            "기준선"
        } else if separated && d_final > 0.4 {
            "◀ 이긴다 (분포 분리)"
        } else if d_final.abs() <= 0.4 {
            "잡음 안"
        } else if d_final > 0.0 {
            "평균만 우세 (분포 겹침)"
        } else {
            "진다"
        };
        println!(
            "  {:14} {:8.2}% {:+8.2}   {:8.2}% {:+8.2}   {verdict}",
            mode.label(),
            mean(sd),
            mean(sd) - bs,
            mean(fd),
            d_final
        );
    }
    println!("\n  읽는 법: 시드 열이 올라도 최종 열이 안 오르면 초기 배치는 전달되지 않은 것이다");
    println!("          (scale-collapse §2-4 의 반증과 같은 결과). 판정은 **최종 열**로만 한다.\n");
}
