//! Identical harness compiled before and after the phase improvements.
use rand::{SeedableRng, rngs::Xoshiro256PlusPlus};
use swarm_core::{
    config::DEFAULT_SWARM_CONFIG as C,
    consts::LBF_SAMPLE_CONFIG,
    optimizer::{lbf::LBFBuilder, separator::Separator},
    structure::pipeline::solution_validation_error,
    util::{
        io::import_spp_instance_with_mm,
        terminator::{BasicTerminator, Terminator},
    },
};
use std::time::{Duration, Instant};
fn main() {
    let args: Vec<_> = std::env::args().collect();
    let seed: u64 = args[2].parse().unwrap();
    let ext = serde_json::from_slice(&std::fs::read(&args[1]).unwrap()).unwrap();
    let inst =
        import_spp_instance_with_mm(&ext, C.cde_config, Some(0.01), Some(-5.), None, None).unwrap();
    let started = Instant::now();
    let built = LBFBuilder::new(
        inst.clone(),
        Xoshiro256PlusPlus::seed_from_u64(seed),
        LBF_SAMPLE_CONFIG,
        C.lbf_x_multiplier,
        C.lbf_y_multiplier,
    )
    .construct();
    let mut sep = Separator::new(
        inst.clone(),
        built.prob,
        Xoshiro256PlusPlus::seed_from_u64(seed + 1),
        C.expl_cfg.separator_config,
    );
    assert!(sep.repair_original_overlap(&BasicTerminator::new()));
    let initial = sep.prob.save();
    assert!(solution_validation_error(&inst, &initial).is_none());
    let init_seconds = started.elapsed().as_secs_f64();
    let mut current = initial.clone();
    let mut rows = Vec::new();
    for phase in ["Explore", "Compress", "Polish"] {
        let seconds = match phase {
            "Explore" => 4,
            "Compress" => 3,
            _ => 2,
        };
        let mut term = BasicTerminator::new();
        let budget = Duration::from_secs(seconds);
        term.new_timeout(budget);
        let clock = Instant::now();
        current = match phase {
            "Explore" => {
                let mut cfg = C.expl_cfg;
                cfg.time_limit = budget;
                swarm_core::optimizer::explore_mods::exploration_phase_adaptive(
                    &inst, &mut sep, &mut None, &term, &cfg,
                )
                .last()
                .cloned()
                .unwrap_or(current)
            }
            "Compress" => {
                let mut cfg = C.cmpr_cfg;
                cfg.time_limit = budget;
                swarm_core::optimizer::compress::compression_phase(
                    &inst, &mut sep, &current, &mut None, &term, &cfg,
                )
            }
            _ => {
                let mut cfg = C.polish_cfg;
                cfg.enabled = true;
                cfg.time_limit = budget;
                swarm_core::optimizer::polish::polish_phase_with_nfp(
                    &inst,
                    &mut sep,
                    &current,
                    &mut None,
                    &term,
                    &cfg,
                    &C.nfp_integration,
                    &C.lp_solver,
                )
            }
        };
        let elapsed = clock.elapsed().as_secs_f64();
        // Public validation plus an independent cold original guard.
        assert!(solution_validation_error(&inst, &current).is_none());
        let mut original = swarm_core::original_overlap::OriginalOverlapGuard::new(&inst);
        assert!(
            original
                .inspect(&current, &[], &BasicTerminator::new())
                .unwrap()
                .is_empty()
        );
        rows.push(serde_json::json!({"phase":phase,"seconds":elapsed,"width":current.strip_width(),"density":current.density()}));
    }
    let clock = Instant::now();
    for _ in 0..100 {
        assert!(solution_validation_error(&inst, &current).is_none());
    }
    let validation_ms = clock.elapsed().as_secs_f64() * 10.;
    std::fs::write(&args[3], serde_json::to_vec_pretty(&serde_json::json!({
        "input":args[1],"seed":seed,"init_seconds":init_seconds,"initial_width":initial.strip_width(),
        "rows":rows,"validation_ms_per_call":validation_ms,
        "poses":current.layout_snapshot().placed_items().values().map(|p| (p.item().idx(),p.d_transf().translation(),p.d_transf().rotation())).collect::<Vec<_>>()
    })).unwrap()).unwrap();
}
