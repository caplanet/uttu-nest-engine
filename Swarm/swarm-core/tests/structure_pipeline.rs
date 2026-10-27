#![cfg(feature = "structure-first")]

use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::geo_enums::RotationRange;
use jagua_rs::probs::spp::entities::{SPInstance, SPProblem};
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::optimizer::lbf::LBFBuilder;
use swarm_core::optimizer::optimize_configured;
use swarm_core::optimizer::separator::Separator;
use swarm_core::structure::pipeline::{StructureStage, optimize_structure_first};
use swarm_core::util::io::read_spp_input;
use swarm_core::util::terminator::BasicTerminator;
use std::path::Path;
use std::time::Duration;

fn load_fu_instance() -> SPInstance {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input/fu.json");
    let (ext_inst, _) = read_spp_input(&path).expect("failed to read fu.json");
    let cfg = DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(cfg.cde_config, cfg.poly_simpl_tolerance, cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&importer, &ext_inst)
        .expect("failed to import instance")
}

fn fast_structure_config() -> swarm_core::config::SwarmConfig {
    let mut cfg = DEFAULT_SWARM_CONFIG;
    cfg.structure.enabled = true;
    cfg.structure.construction_starts = 2;
    cfg.structure.max_active_motifs = 12;
    cfg.structure.p1_time_limit = Duration::from_secs(2);
    cfg.structure.motif.pair_pool_size = 10;
    cfg.structure.motif.top_k_pairs = 16;
    cfg.structure.motif.top_k_bands = 8;
    cfg.structure.p2_time_limit = Duration::from_millis(60);
    cfg.structure.p3_time_limit = Some(Duration::from_millis(100));
    cfg.structure.p4_cycles = 1;
    cfg.structure.p4_cycle_time_limit = Duration::from_millis(100);
    cfg.expl_cfg.time_limit = Duration::from_millis(100);
    cfg.expl_cfg.separator_config.n_workers = 1;
    cfg.expl_cfg.separator_config.iter_no_imprv_limit = 5;
    cfg.expl_cfg.separator_config.strike_limit = 1;
    cfg.cmpr_cfg.separator_config.n_workers = 1;
    cfg.cmpr_cfg.separator_config.iter_no_imprv_limit = 5;
    cfg.cmpr_cfg.separator_config.strike_limit = 1;
    cfg.polish_cfg.time_limit = Duration::from_millis(100);
    cfg.polish_cfg.n_rounds = 1;
    cfg.polish_cfg.hole_fill.enabled = false;
    cfg.polish_cfg.lp.enabled = false;
    cfg
}

#[test_case::test_case(false; "repair_off")]
#[test_case::test_case(true; "repair_on")]
fn p0_to_p5_returns_exact_feasible_monotone_incumbent_and_report(repair_enabled: bool) {
    let instance = load_fu_instance();
    let mut cfg = fast_structure_config();
    cfg.expl_cfg.repair.direction_enabled = repair_enabled;
    cfg.expl_cfg.repair.cluster_lp_enabled = repair_enabled;
    cfg.expl_cfg.repair.chain_enabled = repair_enabled;
    cfg.expl_cfg.repair.stagnation_iterations = 1;
    // The structure path must not leak relaxed separator tilt into a discrete
    // or fixed-angle incumbent.
    cfg.expl_cfg.separator_config.sample_config.tilt_range = 0.15;
    cfg.cmpr_cfg.separator_config.sample_config.tilt_range = 0.15;
    let mut terminator = BasicTerminator::new();
    let (solution, report) = optimize_configured(
        instance.clone(),
        Xoshiro256PlusPlus::seed_from_u64(17),
        None,
        &mut terminator,
        &cfg,
        None,
    );
    let report = report.expect("structure report");

    assert_eq!(
        solution.layout_snapshot().placed_items().len(),
        instance.total_item_qty()
    );
    let mut problem = SPProblem::new(instance.clone()).expect("strip forms a valid container");
    problem.restore(&solution);
    let separator = Separator::new(
        instance.clone(),
        problem,
        Xoshiro256PlusPlus::seed_from_u64(18),
        cfg.expl_cfg.separator_config,
    );
    assert_eq!(separator.ct.get_total_loss(), 0.0);
    assert_allowed_rotations(&instance, &solution);

    for stage in [
        StructureStage::P1Construct,
        StructureStage::P2Compress,
        StructureStage::P3Explore,
        StructureStage::P4Refine,
        StructureStage::P5Finish,
    ] {
        assert!(report.checkpoints.iter().any(|point| point.stage == stage));
    }
    let p1_width = report
        .checkpoints
        .iter()
        .find(|point| point.stage == StructureStage::P1Construct)
        .unwrap()
        .width;
    assert!(solution.strip_width() <= p1_width + 1.0e-5);
    assert_eq!(report.final_width, solution.strip_width());
    assert!(report.construction_starts_completed >= 1);
    assert_eq!(report.schema_version, 3);
    assert_eq!(
        report.motif_generation_stats.singles.emitted,
        report.motif_singles
    );
    assert_eq!(
        report.motif_generation_stats.pairs.emitted,
        report.motif_pairs
    );
    assert_eq!(
        report.motif_generation_stats.rectangle.emitted,
        report.motif_blocks
    );
    assert_eq!(
        report.motif_generation_stats.bands.emitted,
        report.motif_bands
    );
    assert_eq!(
        report.motif_selection_stats.selected_pairs,
        report.pair_matching_selected_pairs
    );
    assert_eq!(
        report.construction_starts_attempted,
        report.p1_start_stats.len()
    );
    assert_eq!(
        report.construction_starts_completed,
        report
            .p1_start_stats
            .iter()
            .filter(|start| start.complete && start.exact_valid)
            .count()
    );
    assert!(report.p1_winner.is_some());
    let selected_candidates = report
        .p1_control_candidate
        .iter()
        .chain(report.p1_start_stats.iter())
        .filter(|candidate| candidate.selected_as_p1_incumbent)
        .count();
    assert_eq!(selected_candidates, 1);
    assert!(
        report
            .checkpoints
            .windows(2)
            .all(|pair| pair[1].incumbent_width <= pair[0].incumbent_width + 1.0e-5)
    );
}

#[test]
fn invalid_partial_warm_start_is_not_retained_as_incumbent() {
    let instance = load_fu_instance();
    let mut cfg = fast_structure_config();
    cfg.structure.construction_starts = 1;
    let empty = SPProblem::new(instance.clone()).expect("strip forms a valid container").save();
    let mut terminator = BasicTerminator::new();
    let (solution, report) = optimize_configured(
        instance.clone(),
        Xoshiro256PlusPlus::seed_from_u64(21),
        None,
        &mut terminator,
        &cfg,
        Some(&empty),
    );

    assert_eq!(
        solution.layout_snapshot().placed_items().len(),
        instance.total_item_qty()
    );
    assert!(report.unwrap().construction_starts_completed >= 1);
    assert_allowed_rotations(&instance, &solution);
}

#[test]
fn direct_pipeline_preserves_frozen_item_transform() {
    let instance = load_fu_instance();
    let cfg = fast_structure_config();
    let initial = LBFBuilder::new(
        instance.clone(),
        Xoshiro256PlusPlus::seed_from_u64(30),
        LBF_SAMPLE_CONFIG,
        cfg.lbf_x_multiplier,
        cfg.lbf_y_multiplier,
    )
    .with_raster_config(cfg.lbf_raster)
    .construct()
    .prob
    .save();
    let frozen_item_id = (0..instance.items.len())
        .find(|item_id| instance.item_qty(*item_id) == 1)
        .expect("fu must contain one single-demand item type");
    let before = initial
        .layout_snapshot()
        .placed_items()
        .values()
        .find(|placed| placed.item().idx() == frozen_item_id)
        .unwrap()
        .d_transf();
    let mut terminator = BasicTerminator::new();
    let (solution, _) = optimize_structure_first(
        instance.clone(),
        Xoshiro256PlusPlus::seed_from_u64(31),
        None,
        &mut terminator,
        &cfg.expl_cfg,
        &cfg.cmpr_cfg,
        &cfg.polish_cfg,
        &cfg.nfp_integration,
        &cfg.lp_solver,
        Some(&initial),
        &[frozen_item_id],
        cfg.lbf_x_multiplier,
        cfg.lbf_y_multiplier,
        &cfg.lbf_raster,
        cfg.lbf_order,
        &cfg.structure,
    );
    let after = solution
        .layout_snapshot()
        .placed_items()
        .values()
        .find(|placed| placed.item().idx() == frozen_item_id)
        .unwrap()
        .d_transf();

    assert_transform_close(before, after);
}

#[test]
fn runtime_switch_off_keeps_legacy_path_and_no_structure_report() {
    let instance = load_fu_instance();
    let mut cfg = fast_structure_config();
    cfg.structure.enabled = false;
    cfg.expl_cfg.time_limit = Duration::from_millis(20);
    cfg.cmpr_cfg.time_limit = Duration::from_millis(20);
    cfg.polish_cfg.time_limit = Duration::from_millis(20);
    let mut terminator = BasicTerminator::new();
    let (solution, report) = optimize_configured(
        instance.clone(),
        Xoshiro256PlusPlus::seed_from_u64(19),
        None,
        &mut terminator,
        &cfg,
        None,
    );
    assert!(report.is_none());
    assert_eq!(
        solution.layout_snapshot().placed_items().len(),
        instance.total_item_qty()
    );
}

fn assert_allowed_rotations(
    instance: &SPInstance,
    solution: &jagua_rs::probs::spp::entities::SPSolution,
) {
    for placed in solution.layout_snapshot().placed_items().values() {
        let allowed = match instance.item(placed.item().idx()).allowed_orientations().rotations() {
            RotationRange::None => angular_distance(placed.d_transf().rotation(), 0.0) <= 1.0e-5,
            RotationRange::Continuous => true,
            RotationRange::Discrete(angles) => angles
                .iter()
                .any(|angle| angular_distance(placed.d_transf().rotation(), *angle) <= 1.0e-5),
        };
        assert!(allowed, "item {} has off-grid angle", placed.item().idx());
    }
}

fn assert_transform_close(a: DTransformation, b: DTransformation) {
    assert!(angular_distance(a.rotation(), b.rotation()) <= 1.0e-5);
    assert!((a.translation().0 - b.translation().0).abs() <= 1.0e-5);
    assert!((a.translation().1 - b.translation().1).abs() <= 1.0e-5);
}

fn angular_distance(a: f32, b: f32) -> f32 {
    let delta = a - b;
    delta.sin().atan2(delta.cos()).abs()
}
