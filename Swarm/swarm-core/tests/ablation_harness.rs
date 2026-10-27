//! Opt-in, source-controlled algorithm ablation harness.
//!
//! The test is ignored in normal CI because it is deliberately a long-running
//! experiment over a user-supplied marker. Every post-LBF arm for a seed starts
//! from the exact same warm-start incumbent and uses one Separator worker.
//!
//! PowerShell example:
//!
//! ```text
//! $env:SWARM_ABLATION_INPUT='C:\markers\input.json'
//! $env:SWARM_ABLATION_SEEDS='11,23,37'
//! $env:SWARM_ABLATION_ARMS='garment,swap_on,hole_bay_on,rnr_on'
//! $env:SWARM_ABLATION_EXPLORE_SECS='540'
//! $env:SWARM_ABLATION_COMPRESS_SECS='60'
//! $env:SWARM_ABLATION_POLISH_SECS='20'
//! $env:SWARM_ABLATION_BUILD_REV=(git rev-parse HEAD)
//! $env:SWARM_ABLATION_OUTPUT='C:\markers\ablation.json'
//! cargo test --release -p swarm-core --features "simd,capmods,adaptive,structure-first" --test ablation_harness -- --ignored --nocapture
//! ```

#![cfg(all(feature = "capmods", feature = "adaptive"))]

use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use serde::Serialize;
use swarm_core::config::{DEFAULT_SWARM_CONFIG, SwarmConfig};
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::optimizer::lbf::LBFBuilder;
use swarm_core::optimizer::optimize_configured;
use swarm_core::structure::pipeline::solution_validation_error;
use swarm_core::util::io::read_spp_input;
use swarm_core::util::terminator::BasicTerminator;
use std::env;
use std::path::Path;
use std::time::{Duration, Instant};

#[derive(Debug, Serialize)]
struct AblationReport {
    schema_version: u32,
    package_version: String,
    build_revision: Option<String>,
    input: String,
    input_hash_fnv1a64: String,
    item_count: usize,
    workers_per_arm: usize,
    compiled_features: CompiledFeatures,
    phase_budgets_secs: PhaseBudgets,
    seeds: Vec<u64>,
    arms: Vec<String>,
    treatments: Vec<TreatmentConfig>,
    seed_run_order: Vec<SeedRunOrder>,
    records: Vec<AblationRecord>,
}

#[derive(Debug, Serialize)]
struct CompiledFeatures {
    simd: bool,
    capmods: bool,
    adaptive: bool,
    structure_first: bool,
}

#[derive(Debug, Serialize)]
struct PhaseBudgets {
    explore: f32,
    compress: f32,
    polish: f32,
}

#[derive(Debug, Serialize)]
struct TreatmentConfig {
    arm: String,
    explore_swap_period: usize,
    compress_swap_period: usize,
    rnr_period: usize,
    lp_enabled: bool,
    slide_enabled: bool,
    hole_bay_enabled: bool,
    explore_left_bias_gain: f32,
    compress_left_bias_gain: f32,
}

#[derive(Debug, Serialize)]
struct SeedRunOrder {
    seed: u64,
    lbf_arms: Vec<&'static str>,
    post_lbf_arms: Vec<String>,
}

#[derive(Debug, Serialize)]
struct AblationRecord {
    kind: &'static str,
    arm: String,
    seed: u64,
    initial_width: f32,
    final_width: f32,
    /// **0~1 분수**다. 다른 하네스는 전부 `* 100.0` 한 퍼센트를 같은 이름으로 쓴다 —
    /// 이름만 보고 섞으면 100배 틀린다. 여기서 이름에 단위를 박아 그 혼동을 막는다.
    final_density_fraction: f32,
    width_reduction_percent: f32,
    elapsed_ms: u128,
    exact_valid: bool,
}

fn load_instance(path: &Path) -> SPInstance {
    let (external, _) = read_spp_input(path)
        .unwrap_or_else(|error| panic!("failed to read {}: {error}", path.display()));
    let config = DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(config.cde_config, config.poly_simpl_tolerance, config.min_item_separation, config.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&importer, &external)
        .expect("failed to import ablation instance")
}

fn env_seconds(name: &str, fallback: f32) -> Duration {
    let seconds = env::var(name)
        .ok()
        .and_then(|value| value.parse::<f32>().ok())
        .filter(|value| value.is_finite() && *value >= 0.0)
        .unwrap_or(fallback);
    Duration::try_from_secs_f32(seconds).unwrap_or(Duration::MAX)
}

fn parse_seeds() -> Vec<u64> {
    env::var("SWARM_ABLATION_SEEDS")
        .unwrap_or_else(|_| "11,23,37".to_string())
        .split(',')
        .filter_map(|value| value.trim().parse().ok())
        .collect::<Vec<_>>()
}

fn parse_arms() -> Vec<String> {
    env::var("SWARM_ABLATION_ARMS")
        .unwrap_or_else(|_| "garment,swap_on,hole_bay_on,rnr_on".to_string())
        .split(',')
        .map(str::trim)
        .filter(|value| !value.is_empty())
        .map(str::to_string)
        .collect()
}

fn fnv1a64(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325, |hash, byte| {
        (hash ^ u64::from(*byte)).wrapping_mul(0x0000_0100_0000_01b3)
    })
}

fn garment_config() -> SwarmConfig {
    let mut config = DEFAULT_SWARM_CONFIG;
    config.structure.enabled = false;
    config.expl_cfg.time_limit = env_seconds("SWARM_ABLATION_EXPLORE_SECS", 30.0);
    config.cmpr_cfg.time_limit = env_seconds("SWARM_ABLATION_COMPRESS_SECS", 10.0);
    config.polish_cfg.time_limit = env_seconds("SWARM_ABLATION_POLISH_SECS", 10.0);
    config.expl_cfg.separator_config.n_workers = 1;
    config.cmpr_cfg.separator_config.n_workers = 1;
    config.expl_cfg.separator_config.swap_period = 0;
    config.cmpr_cfg.separator_config.swap_period = 0;
    config.expl_cfg.rnr.period = 0;
    config.polish_cfg.lp.enabled = true;
    config.polish_cfg.slide.enabled = true;
    config.polish_cfg.hole_fill.enabled = false;
    config
}

fn arm_config(arm: &str) -> SwarmConfig {
    let mut config = garment_config();
    match arm {
        "garment" => {}
        "native_bundle" => {
            let budgets = (
                config.expl_cfg.time_limit,
                config.cmpr_cfg.time_limit,
                config.polish_cfg.time_limit,
            );
            config = DEFAULT_SWARM_CONFIG;
            config.structure.enabled = false;
            config.expl_cfg.time_limit = budgets.0;
            config.cmpr_cfg.time_limit = budgets.1;
            config.polish_cfg.time_limit = budgets.2;
            config.expl_cfg.separator_config.n_workers = 1;
            config.cmpr_cfg.separator_config.n_workers = 1;
        }
        "swap_on" => {
            config.expl_cfg.separator_config.swap_period = 20;
            config.cmpr_cfg.separator_config.swap_period = 20;
        }
        "hole_bay_on" => config.polish_cfg.hole_fill.enabled = true,
        "rnr_on" => config.expl_cfg.rnr.period = 3,
        "lp_only" => config.polish_cfg.slide.enabled = false,
        "left_bias_off" => {
            config
                .expl_cfg
                .separator_config
                .sample_config
                .left_bias_gain = 0.0;
            config
                .cmpr_cfg
                .separator_config
                .sample_config
                .left_bias_gain = 0.0;
        }
        unknown => panic!(
            "unknown ablation arm `{unknown}`; supported: garment,native_bundle,swap_on,hole_bay_on,rnr_on,lp_only,left_bias_off"
        ),
    }
    config
}

fn treatment_config(arm: &str) -> TreatmentConfig {
    let config = arm_config(arm);
    assert_eq!(config.expl_cfg.separator_config.n_workers, 1);
    assert_eq!(config.cmpr_cfg.separator_config.n_workers, 1);
    TreatmentConfig {
        arm: arm.to_string(),
        explore_swap_period: config.expl_cfg.separator_config.swap_period,
        compress_swap_period: config.cmpr_cfg.separator_config.swap_period,
        rnr_period: config.expl_cfg.rnr.period,
        lp_enabled: config.polish_cfg.lp.enabled,
        slide_enabled: config.polish_cfg.slide.enabled,
        hole_bay_enabled: config.polish_cfg.hole_fill.enabled,
        explore_left_bias_gain: config
            .expl_cfg
            .separator_config
            .sample_config
            .left_bias_gain,
        compress_left_bias_gain: config
            .cmpr_cfg
            .separator_config
            .sample_config
            .left_bias_gain,
    }
}

fn lbf_initial(instance: &SPInstance, seed: u64, raster_enabled: bool) -> SPSolution {
    let mut config = garment_config();
    config.lbf_raster.enabled = raster_enabled;
    LBFBuilder::new(
        instance.clone(),
        Xoshiro256PlusPlus::seed_from_u64(seed),
        LBF_SAMPLE_CONFIG,
        config.lbf_x_multiplier,
        config.lbf_y_multiplier,
    )
    .with_raster_config(config.lbf_raster)
    .construct()
    .prob
    .save()
}

fn run_lbf_arm(
    instance: &SPInstance,
    seed: u64,
    raster_enabled: bool,
) -> (SPSolution, AblationRecord) {
    let started = Instant::now();
    let solution = lbf_initial(instance, seed, raster_enabled);
    let exact_valid = solution_validation_error(instance, &solution).is_none();
    let arm = if raster_enabled {
        "raster_on"
    } else {
        "raster_off"
    };
    assert!(exact_valid, "seed {seed}: {arm} LBF solution is not exact");
    let record = AblationRecord {
        kind: "lbf_arm",
        arm: arm.to_string(),
        seed,
        initial_width: solution.strip_width(),
        final_width: solution.strip_width(),
        final_density_fraction: solution.density(),
        width_reduction_percent: 0.0,
        elapsed_ms: started.elapsed().as_millis(),
        exact_valid,
    };
    (solution, record)
}

#[test]
#[ignore = "long-running experiment; set SWARM_ABLATION_INPUT and run explicitly"]
fn marker_algorithm_ablation() {
    let input = env::var("SWARM_ABLATION_INPUT")
        .expect("SWARM_ABLATION_INPUT must point to a Swarm input JSON file");
    let seeds = parse_seeds();
    let arms = parse_arms();
    assert!(!seeds.is_empty(), "at least one ablation seed is required");
    assert!(!arms.is_empty(), "at least one ablation arm is required");
    let unique_arms = arms.iter().collect::<std::collections::HashSet<_>>();
    assert_eq!(
        unique_arms.len(),
        arms.len(),
        "ablation arms must be unique"
    );

    let instance = load_instance(Path::new(&input));
    let input_bytes = std::fs::read(&input)
        .unwrap_or_else(|error| panic!("failed to hash ablation input {input}: {error}"));
    let treatments = arms
        .iter()
        .map(|arm| treatment_config(arm))
        .collect::<Vec<_>>();
    let base_config = garment_config();
    let phase_budgets_secs = PhaseBudgets {
        explore: base_config.expl_cfg.time_limit.as_secs_f32(),
        compress: base_config.cmpr_cfg.time_limit.as_secs_f32(),
        polish: base_config.polish_cfg.time_limit.as_secs_f32(),
    };
    let mut records = Vec::with_capacity(seeds.len() * (arms.len() + 2));
    let mut seed_run_order = Vec::with_capacity(seeds.len());

    for (seed_index, &seed) in seeds.iter().enumerate() {
        // Initial-construction arms cannot share an incumbent because Raster NFP
        // is itself the constructor treatment. They still use the same seed and
        // report initial quality separately. Alternate OFF/ON execution order
        // by seed to counterbalance cache/thermal timing bias.
        let (initial, lbf_order) = if seed_index.is_multiple_of(2) {
            let (_, raster_off_record) = run_lbf_arm(&instance, seed, false);
            let (initial, raster_on_record) = run_lbf_arm(&instance, seed, true);
            records.extend([raster_off_record, raster_on_record]);
            (initial, vec!["raster_off", "raster_on"])
        } else {
            let (initial, raster_on_record) = run_lbf_arm(&instance, seed, true);
            let (_, raster_off_record) = run_lbf_arm(&instance, seed, false);
            records.extend([raster_on_record, raster_off_record]);
            (initial, vec!["raster_on", "raster_off"])
        };

        // Rotate the post-LBF order for each seed. Every arm still gets the
        // identical exact incumbent and optimizer RNG seed for that seed.
        let mut post_lbf_order = arms.clone();
        let order_len = post_lbf_order.len();
        post_lbf_order.rotate_left(seed_index % order_len);
        seed_run_order.push(SeedRunOrder {
            seed,
            lbf_arms: lbf_order,
            post_lbf_arms: post_lbf_order.clone(),
        });

        for arm in &post_lbf_order {
            let config = arm_config(arm);
            assert_eq!(config.expl_cfg.separator_config.n_workers, 1);
            assert_eq!(config.cmpr_cfg.separator_config.n_workers, 1);
            let mut terminator = BasicTerminator::new();
            let started = Instant::now();
            let (solution, _) = optimize_configured(
                instance.clone(),
                Xoshiro256PlusPlus::seed_from_u64(seed ^ 0xA5A5_5A5A_D3C4_B2E1),
                None,
                &mut terminator,
                &config,
                Some(&initial),
            );
            let exact_valid = solution_validation_error(&instance, &solution).is_none();
            let record = AblationRecord {
                kind: "post_lbf_arm",
                arm: arm.clone(),
                seed,
                initial_width: initial.strip_width(),
                final_width: solution.strip_width(),
                final_density_fraction: solution.density(),
                width_reduction_percent: (1.0 - solution.strip_width() / initial.strip_width())
                    * 100.0,
                elapsed_ms: started.elapsed().as_millis(),
                exact_valid,
            };
            println!(
                "ABLATION_JSON {}",
                serde_json::to_string(&record).expect("record serialization failed")
            );
            assert!(
                exact_valid,
                "seed {seed}, arm {arm}: final solution is not exact"
            );
            records.push(record);
        }
    }

    let report = AblationReport {
        schema_version: 2,
        package_version: env!("CARGO_PKG_VERSION").to_string(),
        build_revision: env::var("SWARM_ABLATION_BUILD_REV").ok(),
        input: input.clone(),
        input_hash_fnv1a64: format!("{:016x}", fnv1a64(&input_bytes)),
        item_count: instance.total_item_qty(),
        workers_per_arm: 1,
        compiled_features: CompiledFeatures {
            simd: cfg!(feature = "simd"),
            capmods: cfg!(feature = "capmods"),
            adaptive: cfg!(feature = "adaptive"),
            structure_first: cfg!(feature = "structure-first"),
        },
        phase_budgets_secs,
        seeds,
        arms,
        treatments,
        seed_run_order,
        records,
    };
    let json = serde_json::to_string_pretty(&report).expect("report serialization failed");
    println!("ABLATION_REPORT\n{json}");
    if let Ok(output) = env::var("SWARM_ABLATION_OUTPUT") {
        std::fs::write(&output, json)
            .unwrap_or_else(|error| panic!("failed to write {output}: {error}"));
    }
}

#[test]
fn harness_metadata_and_treatment_snapshots_are_stable() {
    assert_eq!(fnv1a64(b"hello"), 0xa430_d846_80aa_bd0b);

    let garment = treatment_config("garment");
    assert_eq!(garment.explore_swap_period, 0);
    assert_eq!(garment.compress_swap_period, 0);
    assert_eq!(garment.rnr_period, 0);
    assert!(garment.lp_enabled);
    assert!(garment.slide_enabled);
    assert!(!garment.hole_bay_enabled);

    // `native_bundle` is the product default configuration. Swap, R&R and hole
    // fill used to be on by default and have since been turned off, so check
    // the arm against the defaults rather than against frozen values.
    let native = treatment_config("native_bundle");
    let defaults = DEFAULT_SWARM_CONFIG;
    assert_eq!(native.explore_swap_period, defaults.expl_cfg.separator_config.swap_period);
    assert_eq!(native.compress_swap_period, defaults.cmpr_cfg.separator_config.swap_period);
    assert_eq!(native.rnr_period, defaults.expl_cfg.rnr.period);
    assert_eq!(native.lp_enabled, defaults.polish_cfg.lp.enabled);
    assert_eq!(native.slide_enabled, defaults.polish_cfg.slide.enabled);
    assert_eq!(native.hole_bay_enabled, defaults.polish_cfg.hole_fill.enabled);
}
