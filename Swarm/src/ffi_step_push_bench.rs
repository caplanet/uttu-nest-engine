//! Polish 왼쪽 밀기 변형(A: NFP 재배치, B: 비스듬한 slide, C: 원본 외곽 인지 slide) 비교 벤치.
//!
//! 앱(Uttu.Nest)과 같은 설정으로 Init → Explore → Compress 를 돌려 압축 결과를 시드마다
//! 저장해 두고, 모든 변형을 **같은 압축 결과**에서 Polish 만 다시 돌려 비교한다.
//!
//! ```powershell
//! $env:PUSH_INPUT='C:\...\marker.json'   # 생략하면 아래 기본 경로
//! $env:PUSH_MAX_SECS='60'                # 앱의 MaxSeconds. Polish 는 그 1/10
//! $env:PUSH_SEEDS='1,2,3,4'
//! cargo test --release --features "simd,capmods,adaptive,structure-first" --lib `
//!   ffi_step::push_bench -- --ignored --nocapture --test-threads=1
//! ```
//! 압축 결과는 `target/push_bench/` 에 남아 다음 실행이 재사용한다(입력·예산·시드가 같을 때).

use super::*;
use crate::ffi::{CSwarmConfig, swarm_get_default_config};
use jagua_rs::probs::spp::entities::SPInstance;
use jagua_rs::geometry::DTransformation;
use jagua_rs::probs::spp::entities::SPPlacement;
use swarm_core::config::PolishConfig;
use swarm_core::optimizer::polish::{PolishStats, polish_phase_measured};
use std::path::PathBuf;

const DEFAULT_INPUT: &str =
    r"C:\Caplanet\TestFiles\Swarm Test Files\output\여성WSjk_비교용-G_NEW13_Swarm밀기.json";

fn env_or(key: &str, default: &str) -> String {
    std::env::var(key).unwrap_or_else(|_| default.to_string())
}

/// Uttu.Nest `CreateSwarmConfig` 와 같은 값.
fn app_config(max_secs: f32, seed: u64) -> CSwarmConfig {
    let hds = max_secs / 10.0;
    let mut c: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut c);
    c.rng_seed = seed;
    c.poly_simpl_tolerance = 0.001;
    c.min_item_separation = 0.0;
    c.narrow_concavity_cutoff_dist = 0.001;
    c.narrow_concavity_cutoff_area = 0.001;
    c.lbf_x_multiplier = 10.0;
    c.lbf_y_multiplier = 1.0;
    c.allow_180_rotation = 0;
    c.tilt_range_deg = 0.0;
    c.lbf_order.mode = 0;
    c.width_relief.fail_threshold = 0;
    c.width_relief.step = 0.0;
    c.width_relief.max_ratio = 0.03;
    c.width_relief.max_rounds = 12;
    c.width_relief.strike_limit = 1;
    c.width_relief.round_budget_ms = 0;
    c.fabric_tolerance.piece_overlap_mm = 0.08;
    c.fabric_edge.margin_mm = 0.2;

    let e = &mut c.exploration;
    e.shrink_step = 0.001;
    e.time_limit_secs = hds * 6.0;
    e.max_conseq_failed_attempts = -1;
    e.solution_pool_distribution_stddev = 0.5;
    e.large_item_ch_area_cutoff_percentile = 0.5;
    e.rnr_period = 0;
    e.rnr_band_fraction = 0.2;
    e.rnr_max_items = 100;
    e.adaptive_shrink_gain = 4.0;
    e.adaptive_shrink_max_mult = 5.0;
    e.osc_fail_threshold = 0;
    e.osc_expand_steps = 3.0;
    e.alns_decay = 0.85;
    e.alns_min_weight = 0.1;
    let s = &mut e.separator;
    s.iter_no_imprv_limit = 1;
    s.strike_limit = 1;
    s.n_workers = 4;
    s.n_container_samples = 50;
    s.n_focussed_samples = 25;
    s.n_coord_descents = 1;
    s.swap_period = 0;
    s.swap_attempts = 8;
    s.swap_area_ratio_min = 0.4;
    s.swap_area_ratio_max = 2.5;
    s.left_bias_gain = 2.0;

    let k = &mut c.compression;
    k.shrink_range_min = 0.001;
    k.shrink_range_max = 0.00001;
    k.time_limit_secs = hds * 3.0;
    k.shrink_decay_strategy = 0;
    k.shrink_decay_ratio = 0.9;
    let s = &mut k.separator;
    s.iter_no_imprv_limit = 200;
    s.strike_limit = 3;
    s.n_workers = 4;
    s.n_container_samples = 100;
    s.n_focussed_samples = 50;
    s.n_coord_descents = 5;
    s.swap_period = 0;
    s.swap_attempts = 8;
    s.swap_area_ratio_min = 0.4;
    s.swap_area_ratio_max = 2.5;
    s.left_bias_gain = 2.0;

    let p = &mut c.polish;
    p.enabled = 1;
    p.time_limit_secs = hds;
    p.n_rounds = 3;
    p.slide_enabled = 1;
    p.slide_n_passes = 3;
    p.slide_down = 1;
    p.slide_safety_ratio = 0.0;
    p.hole_fill_enabled = 0;
    p.lp_enabled = 1;
    p.lp_n_iterations = 40;
    p.lp_trust_region_ratio = 0.005;
    p.lp_left_pull_weight = 0.02;

    c.lbf_raster.enabled = 0;
    c
}

/// 압축 결과 한 벌: 폭과 조각별 자세.
#[derive(serde::Serialize, serde::Deserialize)]
struct Snapshot {
    width: f32,
    poses: Vec<(usize, f32, f32, f32)>,
}

impl Snapshot {
    fn of(sol: &SPSolution) -> Self {
        let mut poses: Vec<_> = sol
            .layout_snapshot()
            .placed_items()
            .values()
            .map(|p| {
                let (x, y) = p.d_transf().translation();
                (p.item().idx(), p.d_transf().rotation(), x, y)
            })
            .collect();
        poses.sort_by(|a, b| a.0.cmp(&b.0).then(a.2.total_cmp(&b.2)));
        Self {
            width: sol.strip_width(),
            poses,
        }
    }

    fn restore(&self, instance: &SPInstance) -> SPProblem {
        let mut prob = SPProblem::new(instance.clone()).expect(swarm_core::dmsg!("strip forms a valid container"));
        prob.change_strip_width(self.width).expect(swarm_core::dmsg!("strip width must be positive"));
        for &(item_id, r, x, y) in &self.poses {
            prob.place_item(SPPlacement {
                item_idx: item_id,
                d_transf: DTransformation::new(r, (x, y)),
            });
        }
        prob
    }
}

struct Prepared {
    instance: SPInstance,
    plsh_config: PolishConfig,
    cmpr_config: CompressionConfig,
    nfp_integration: swarm_core::config::NfpIntegrationConfig,
    lp_solver: swarm_core::lp_solver::LpSolverConfig,
    frozen: Vec<usize>,
    snapshot: Snapshot,
}

/// 시드 하나의 압축 결과. 저장본이 있으면 Init 만 다시 해서 인스턴스·설정을 얻는다.
fn prepare(json: &CString, cfg: &CSwarmConfig, cache: &PathBuf) -> Prepared {
    let cached = std::fs::read_to_string(cache)
        .ok()
        .and_then(|text| serde_json::from_str::<Snapshot>(&text).ok());
    let started = std::time::Instant::now();
    let handle = swarm_init_from_json(json.as_ptr(), cfg, std::ptr::null(), 0);
    assert!(!handle.is_null(), "init failed: {}", swarm_get_last_error_code());
    if cached.is_none() {
        assert_eq!(swarm_alg_phase(handle), 0, "explore failed");
        assert_eq!(swarm_cp_phase(handle), 0, "compress failed");
    }
    let prepared = {
        let h = unsafe { &*handle };
        let mut engine = h.lock_engine();
        let state = engine.legacy_mut();
        let snapshot = cached.unwrap_or_else(|| {
            let snap = Snapshot::of(state.solution.as_ref().expect(swarm_core::dmsg!("compressed solution")));
            std::fs::create_dir_all(cache.parent().unwrap()).unwrap();
            std::fs::write(cache, serde_json::to_string(&snap).unwrap()).unwrap();
            println!(
                "  압축 결과 저장: 폭 {:.3}mm ({:.0}s) -> {}",
                snap.width,
                started.elapsed().as_secs_f32(),
                cache.display()
            );
            snap
        });
        Prepared {
            instance: state.instance.clone(),
            plsh_config: state.plsh_config,
            cmpr_config: state.cmpr_config,
            nfp_integration: state.nfp_integration,
            lp_solver: state.lp_solver,
            frozen: state.frozen_item_ids.clone(),
            snapshot,
        }
    };
    swarm_free(handle);
    prepared
}

struct Outcome {
    width: f32,
    mean_x_max: f64,
    moved: usize,
    stats: PolishStats,
    secs: f32,
    solution: SPSolution,
}

fn mean_x_max(sol: &SPSolution) -> f64 {
    let items = &sol.layout_snapshot().placed_items();
    items.values().map(|p| p.shape().bbox().x_max as f64).sum::<f64>() / items.len().max(1) as f64
}

fn run_variant(p: &Prepared, config: &PolishConfig, seed: u64) -> Outcome {
    let prob = p.snapshot.restore(&p.instance);
    let start = prob.save();
    assert!(
        crate::structure::pipeline::solution_validation_error_with_frozen_ids(
            &p.instance,
            &start,
            &p.frozen
        )
        .is_none(),
        "restored compression result must certify"
    );
    let mut sep = Separator::new(
        p.instance.clone(),
        prob,
        Xoshiro256PlusPlus::seed_from_u64(seed),
        p.cmpr_config.separator_config,
    );
    sep.freeze_items(&p.frozen);
    let mut term = BasicTerminator::new();
    term.new_timeout(config.time_limit);
    let started = std::time::Instant::now();
    let (solution, stats) = polish_phase_measured(
        &p.instance,
        &mut sep,
        &start,
        &mut None,
        &term,
        config,
        &p.nfp_integration,
        &p.lp_solver,
    );
    let secs = started.elapsed().as_secs_f32();
    assert!(
        crate::structure::pipeline::solution_validation_error_with_frozen_ids(
            &p.instance,
            &solution,
            &p.frozen
        )
        .is_none(),
        "polish result must certify"
    );
    let before: std::collections::HashMap<usize, (f32, f32)> = start
        .layout_snapshot()
        .placed_items()
        .values()
        .map(|p| (p.item().idx(), p.d_transf().translation()))
        .collect();
    let moved = solution
        .layout_snapshot()
        .placed_items()
        .values()
        .filter(|p| {
            let (x, y) = p.d_transf().translation();
            let (bx, by) = before[&p.item().idx()];
            (x - bx).abs() > 1.0e-3 || (y - by).abs() > 1.0e-3
        })
        .count();
    Outcome {
        width: solution.strip_width(),
        mean_x_max: mean_x_max(&solution),
        moved,
        stats,
        secs,
        solution,
    }
}

fn write_svg(_p: &Prepared, sol: &SPSolution, path: &PathBuf, title: &str) {
    let doc = jagua_rs::io::svg::s_layout_to_svg(
        sol.layout_snapshot(),
        0,
        jagua_rs::io::svg::SvgDrawOptions::default(),
        title,
    );
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    svg::save(path, &doc).unwrap();
}

type Variant = (&'static str, fn(&mut PolishConfig));

const VARIANTS: [Variant; 7] = [
    ("base", |c| c.nfp_left.enabled = false),
    ("C", |c| {
        c.nfp_left.enabled = false;
        c.slide.original_aware = true;
    }),
    ("B", |c| {
        c.nfp_left.enabled = false;
        c.slide.diagonal = true;
    }),
    ("B+C", |c| {
        c.nfp_left.enabled = false;
        c.slide.diagonal = true;
        c.slide.original_aware = true;
    }),
    ("A", |c| c.nfp_left.enabled = true),
    ("A+C", |c| {
        c.nfp_left.enabled = true;
        c.slide.original_aware = true;
    }),
    ("A+B+C", |c| {
        c.nfp_left.enabled = true;
        c.slide.diagonal = true;
        c.slide.original_aware = true;
    }),
];

#[test]
#[ignore]
fn polish_push_variants() {
    let input = env_or("PUSH_INPUT", DEFAULT_INPUT);
    let max_secs: f32 = env_or("PUSH_MAX_SECS", "60").parse().unwrap();
    let seeds: Vec<u64> = env_or("PUSH_SEEDS", "1,2,3,4")
        .split(',')
        .map(|s| s.trim().parse().unwrap())
        .collect();
    let only: Option<Vec<String>> = std::env::var("PUSH_VARIANTS")
        .ok()
        .map(|v| v.split(',').map(|s| s.trim().to_string()).collect());
    let no_lp = std::env::var("PUSH_NO_LP").is_ok_and(|v| v == "1");
    let svg_seed: Option<u64> = std::env::var("PUSH_SVG_SEED").ok().map(|s| s.parse().unwrap());
    let text = std::fs::read_to_string(&input).expect(swarm_core::dmsg!("input must be readable"));
    let json = CString::new(text.trim_start_matches('\u{feff}')).unwrap();
    let stem = PathBuf::from(&input)
        .file_stem()
        .unwrap()
        .to_string_lossy()
        .into_owned();
    let out_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("target/push_bench");

    println!("입력 {input}, MaxSeconds {max_secs}, 시드 {seeds:?}, LP {}", if no_lp { "끔" } else { "켬" });
    let mut totals: Vec<(f64, f64, f64, usize)> = vec![(0.0, 0.0, 0.0, 0); VARIANTS.len()];
    for &seed in &seeds {
        let cfg = app_config(max_secs, seed);
        let cache = out_dir.join(format!("{stem}_{max_secs}s_seed{seed}.json"));
        let prepared = prepare(&json, &cfg, &cache);
        let start_width = prepared.snapshot.width;
        let start_mean = mean_x_max(&prepared.snapshot.restore(&prepared.instance).save());
        println!("\n시드 {seed}: 압축 결과 폭 {start_width:.3}mm, 평균 x_max {start_mean:.2}mm");
        println!(
            "  {:<6} {:>10} {:>9} {:>11} {:>6} {:>7} {:>6} {:>7} {:>6}",
            "변형", "폭(mm)", "Δ폭", "Δ평균x_max", "이동", "라운드", "실패", "변경", "초"
        );
        if svg_seed == Some(seed) {
            let start = prepared.snapshot.restore(&prepared.instance).save();
            write_svg(&prepared, &start, &out_dir.join(format!("seed{seed}_compress.svg")), "compress");
        }
        for (index, (name, apply)) in VARIANTS.iter().enumerate() {
            if only.as_ref().is_some_and(|names| !names.iter().any(|n| n == name)) {
                continue;
            }
            let mut config = prepared.plsh_config;
            if no_lp {
                config.lp.enabled = false;
            }
            apply(&mut config);
            let o = run_variant(&prepared, &config, seed);
            let dw = (o.width - start_width) as f64;
            let dm = o.mean_x_max - start_mean;
            println!(
                "  {:<6} {:>10.3} {:>+9.3} {:>+11.2} {:>6} {:>7} {:>6} {:>7} {:>6.1}",
                name, o.width, dw, dm, o.moved, o.stats.rounds, o.stats.failed_rounds, o.stats.changes, o.secs
            );
            let t = &mut totals[index];
            t.0 += dw;
            t.1 += dm;
            t.2 += o.moved as f64;
            t.3 += 1;
            if svg_seed == Some(seed) {
                write_svg(
                    &prepared,
                    &o.solution,
                    &out_dir.join(format!("seed{seed}_{}.svg", name.replace('+', ""))),
                    name,
                );
            }
        }
    }
    println!("\n평균 (시드 {}개)", seeds.len());
    println!("  {:<6} {:>9} {:>11} {:>7}", "변형", "Δ폭", "Δ평균x_max", "이동");
    for ((name, _), t) in VARIANTS.iter().zip(&totals) {
        if t.3 == 0 {
            continue;
        }
        let n = t.3 as f64;
        println!("  {:<6} {:>+9.3} {:>+11.2} {:>7.1}", name, t.0 / n, t.1 / n, t.2 / n);
    }
}

/// 압축 결과에서 각 방법이 **한 번 훑을 때** 조각을 얼마나 왼쪽으로 옮기는지 잰다 (LP 없음).
#[test]
#[ignore]
fn push_potential() {
    use swarm_core::config::SlideConfig;
    use swarm_core::optimizer::polish::{nfp_left::nfp_left_pass, slide::slide_pass};

    let input = env_or("PUSH_INPUT", DEFAULT_INPUT);
    let max_secs: f32 = env_or("PUSH_MAX_SECS", "60").parse().unwrap();
    let seeds: Vec<u64> = env_or("PUSH_SEEDS", "1,2,3,4")
        .split(',')
        .map(|s| s.trim().parse().unwrap())
        .collect();
    let text = std::fs::read_to_string(&input).expect(swarm_core::dmsg!("input must be readable"));
    let json = CString::new(text.trim_start_matches('\u{feff}')).unwrap();
    let stem = PathBuf::from(&input)
        .file_stem()
        .unwrap()
        .to_string_lossy()
        .into_owned();
    let out_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("target/push_bench");

    for &seed in &seeds {
        let cfg = app_config(max_secs, seed);
        let cache = out_dir.join(format!("{stem}_{max_secs}s_seed{seed}.json"));
        let p = prepare(&json, &cfg, &cache);
        {
            // 출발 배치에서 원본 외곽 검사가 이미 막는 조각이 있는지.
            let prob = p.snapshot.restore(&p.instance);
            let sep = Separator::new(
                p.instance.clone(),
                prob,
                Xoshiro256PlusPlus::seed_from_u64(seed),
                p.cmpr_config.separator_config,
            );
            let blocked = sep
                .prob
                .layout()
                .placed_items()
                .iter()
                .filter(|(pk, pi)| {
                    let item = sep.instance.item(pi.item().idx());
                    let probe = swarm_core::optimizer::polish::probe::CollisionProbe::new(
                        &sep.prob.layout(),
                        item,
                        *pk,
                    )
                    .with_original(sep.original_guard());
                    !probe.original_ok(pi.d_transf())
                })
                .count();
            println!("\n시드 {seed}: 출발 자리에서 원본 검사에 걸리는 조각 {blocked}개");
        }
        println!("시드 {seed}: 한 번 훑기 (왼쪽만)");
        println!(
            "  {:<8} {:>6} {:>8} {:>8} {:>10} {:>8} {:>6}",
            "방법", "≥1mm", "≥5mm", "최대", "합계(mm)", "인증", "초"
        );
        let base = SlideConfig {
            n_passes: 1,
            slide_down: false,
            ..p.plsh_config.slide
        };
        let methods: [(&str, Option<SlideConfig>); 5] = [
            ("slide", Some(base)),
            ("C", Some(SlideConfig { original_aware: true, ..base })),
            ("B", Some(SlideConfig { diagonal: true, ..base })),
            ("B+C", Some(SlideConfig { diagonal: true, original_aware: true, ..base })),
            ("A", None),
        ];
        for (name, slide) in methods {
            let prob = p.snapshot.restore(&p.instance);
            let start = prob.save();
            let mut sep = Separator::new(
                p.instance.clone(),
                prob,
                Xoshiro256PlusPlus::seed_from_u64(seed),
                p.cmpr_config.separator_config,
            );
            sep.freeze_items(&p.frozen);
            let term = BasicTerminator::new();
            let started = std::time::Instant::now();
            match slide {
                Some(cfg) => {
                    slide_pass(&mut sep, &cfg, &term);
                }
                None => {
                    let mut nfp = swarm_core::optimizer::separator::build_nfp_sampler_cache(
                        &p.instance,
                    )
                    .expect(swarm_core::dmsg!("NFP library"));
                    nfp_left_pass(&mut sep, &p.plsh_config.nfp_left, &mut nfp, &term);
                }
            }
            let secs = started.elapsed().as_secs_f32();
            let certified = sep.ct.get_total_loss() == 0.0
                && sep.repair_original_overlap(&term)
                && sep.solution_is_certified(&sep.prob.save(), &start);
            let before: std::collections::HashMap<usize, f32> = start
                .layout_snapshot()
                .placed_items()
                .values()
                .map(|p| (p.item().idx(), p.d_transf().translation().0))
                .collect();
            let gains: Vec<f32> = sep
                .prob
                .layout()
                .placed_items()
                .values()
                .map(|p| before[&p.item().idx()] - p.d_transf().translation().0)
                .collect();
            println!(
                "  {:<8} {:>6} {:>8} {:>8.2} {:>10.1} {:>8} {:>6.2}",
                name,
                gains.iter().filter(|&&g| g >= 1.0).count(),
                gains.iter().filter(|&&g| g >= 5.0).count(),
                gains.iter().cloned().fold(0.0f32, f32::max),
                gains.iter().filter(|&&g| g > 0.0).sum::<f32>(),
                if certified { "통과" } else { "실패" },
                secs
            );
        }
    }
}
