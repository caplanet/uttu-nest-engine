//! separation 핵심 경로 회귀 테스트 (capmods 여부와 무관하게 실행 가능).
//!
//! 좁힌 strip에서 여러 시드로 separate를 완주시켜 디버그 불변식
//! (트래커-레이아웃 일치, CDE 파이프라인 superset 검증 등)이 깨지지 않는지 확인한다.

use jagua_rs::probs::spp::entities::SPInstance;
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::optimizer::lbf::LBFBuilder;
use swarm_core::optimizer::separator::Separator;
use swarm_core::util::io::read_spp_input;
use swarm_core::util::terminator::{BasicTerminator, Terminator};
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

#[test]
fn vanilla_separate_multi_seed() {
    for seed in [1234u64, 42, 2026] {
        let instance = load_fu_instance();
        let rng = Xoshiro256PlusPlus::seed_from_u64(seed);
        let builder =
            LBFBuilder::new(instance.clone(), rng, LBF_SAMPLE_CONFIG, 10.0, 1.0).construct();

        let sep_cfg = DEFAULT_SWARM_CONFIG.expl_cfg.separator_config;
        let mut sep = Separator::new(
            instance,
            builder.prob,
            Xoshiro256PlusPlus::seed_from_u64(seed + 1),
            sep_cfg,
        );

        let new_width = sep.prob.strip_width() * 0.85;
        sep.change_strip_width(new_width, None);

        let mut term = BasicTerminator::new();
        term.new_timeout(Duration::from_secs(6));
        let (_sol, ct) = sep.separate(&term, &mut None);
        println!(
            "seed {seed}: separate finished, loss: {}",
            ct.get_total_loss()
        );
    }
}
