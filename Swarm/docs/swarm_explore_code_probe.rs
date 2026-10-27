//! Fixed-work Release comparison. Uses DEFAULT_SWARM_CONFIG unchanged.
use jagua_rs::io::import::Importer;
use rand::rngs::Xoshiro256PlusPlus;
use rand::{RngExt, SeedableRng};
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::optimizer::{lbf::LBFBuilder, separator::Separator};
use swarm_core::util::{
    io::read_spp_input,
    terminator::{BasicTerminator, Terminator},
};
use std::{
    path::Path,
    time::{Duration, Instant},
};

fn main() {
    for name in std::env::args().skip(1) {
        for seed in [17, 42, 83] {
            let cfg = DEFAULT_SWARM_CONFIG;
            let (ext, _) = read_spp_input(Path::new(&name)).unwrap();
            let importer = Importer::new(
                cfg.cde_config,
                cfg.poly_simpl_tolerance,
                cfg.min_item_separation,
                cfg.narrow_concavity_cutoff_ratio,
            );
            let instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext).unwrap();
            let builder = LBFBuilder::new(
                instance.clone(),
                Xoshiro256PlusPlus::seed_from_u64(seed),
                LBF_SAMPLE_CONFIG,
                cfg.lbf_x_multiplier,
                cfg.lbf_y_multiplier,
            )
            .construct();
            let mut sep = Separator::new(
                instance.clone(),
                builder.prob,
                Xoshiro256PlusPlus::seed_from_u64(seed + 1),
                cfg.expl_cfg.separator_config,
            );
            let initial = sep.prob.strip_width();
            let mut term = BasicTerminator::new();
            term.new_timeout(Duration::from_secs(120));
            let start = Instant::now();
            let mut trace = Vec::new();
            for _ in 0..3 {
                sep.change_strip_width(
                    sep.prob.strip_width() * (1.0 - cfg.expl_cfg.shrink_step),
                    None,
                );
                let (sol, ct) = sep.separate(&term, &mut None);
                assert!(!term.kill(), "probe exceeded its external guard");
                sep.rollback(&sol, Some(&ct));
                let mut poses: Vec<_> = sep
                    .prob
                    .layout
                    .placed_items
                    .values()
                    .map(|p| {
                        let (x, y) = p.d_transf.translation();
                        (
                            p.item_id,
                            p.d_transf.rotation().to_bits(),
                            x.to_bits(),
                            y.to_bits(),
                        )
                    })
                    .collect();
                poses.sort_unstable();
                trace.push((
                    sol.strip_width().to_bits(),
                    ct.get_total_loss().to_bits(),
                    poses,
                ));
            }
            let ms = start.elapsed().as_secs_f64() * 1000.0;
            let feasible = sep.prob.layout.is_feasible();
            let validation_error = swarm_core::structure::pipeline::solution_validation_error(
                &instance,
                &sep.prob.save(),
            )
            .map(|error| format!("{error:?}"));
            let rng_tail: Vec<u64> = sep.workers.iter_mut().map(|w| w.rng.random()).collect();
            println!(
                "{}",
                serde_json::json!({"file":name,"seed":seed,"initial_width":initial,
                "ms":ms,"trace":trace,"rng_tail":rng_tail,"exact_feasible":feasible,
                "validation_error":validation_error})
            );
        }
    }
}
