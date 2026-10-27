//! Bounded pre-integration gate: real phase results, then exact original audit
//! and transactional repair. Production callbacks are not changed by this probe.
use jagua_rs::probs::spp::entities::SPSolution;
use rand::{SeedableRng, rngs::Xoshiro256PlusPlus};
use swarm_core::{
    config::DEFAULT_SWARM_CONFIG as C,
    consts::LBF_SAMPLE_CONFIG,
    optimizer::{lbf::LBFBuilder, separator::Separator},
    original_overlap::OriginalOverlapGuard,
    util::{
        io::import_spp_instance_with_mm,
        terminator::{BasicTerminator, Terminator},
    },
};
use std::{
    path::PathBuf,
    time::{Duration, Instant},
};
fn main() {
    let args: Vec<_> = std::env::args().collect();
    let input = PathBuf::from(&args[1]);
    let seed: u64 = args[2].parse().unwrap();
    let overlap: f32 = args[3].parse().unwrap();
    let output = PathBuf::from(&args[4]);
    let growth = args.iter().any(|a| a == "--growth");
    let ext = serde_json::from_slice(&std::fs::read(&input).unwrap()).unwrap();
    let start = Instant::now();
    let inst =
        import_spp_instance_with_mm(&ext, C.cde_config, Some(0.01), Some(-overlap), None, None)
            .unwrap();
    let imported = start.elapsed().as_secs_f64();
    let builder = LBFBuilder::new(
        inst.clone(),
        Xoshiro256PlusPlus::seed_from_u64(seed),
        LBF_SAMPLE_CONFIG,
        C.lbf_x_multiplier,
        C.lbf_y_multiplier,
    )
    .construct();
    let built = start.elapsed().as_secs_f64();
    let mut config = C.expl_cfg.separator_config;
    config.n_workers = 1;
    let mut sep = Separator::new(
        inst.clone(),
        builder.prob,
        Xoshiro256PlusPlus::seed_from_u64(seed + 1),
        config,
    );
    let mut stages = vec![("Init", sep.prob.save(), built - imported)];
    let mut term = BasicTerminator::new();
    term.new_timeout(Duration::from_secs(4));
    let phase = Instant::now();
    let sols = swarm_core::optimizer::explore_mods::exploration_phase_adaptive(
        &inst,
        &mut sep,
        &mut None,
        &term,
        &C.expl_cfg,
    );
    let explored = sols.last().cloned().unwrap_or_else(|| stages[0].1.clone());
    stages.push(("Explore", explored.clone(), phase.elapsed().as_secs_f64()));
    term.new_timeout(Duration::from_secs(3));
    let phase = Instant::now();
    let compressed = swarm_core::optimizer::compress::compression_phase(
        &inst,
        &mut sep,
        &explored,
        &mut None,
        &term,
        &C.cmpr_cfg,
    );
    stages.push((
        "Compress",
        compressed.clone(),
        phase.elapsed().as_secs_f64(),
    ));
    term.new_timeout(Duration::from_secs(2));
    let phase = Instant::now();
    let mut polish = C.polish_cfg;
    polish.enabled = true;
    let polished = swarm_core::optimizer::polish::polish_phase_with_nfp(
        &inst,
        &mut sep,
        &compressed,
        &mut None,
        &term,
        &polish,
        &C.nfp_integration,
        &C.lp_solver,
    );
    stages.push(("Polish", polished, phase.elapsed().as_secs_f64()));
    let mut rows = Vec::new();
    for (name, sol, phase_seconds) in stages {
        sep.rollback(&sol, None);
        let mut guard = OriginalOverlapGuard::new(&inst);
        let term = BasicTerminator::new();
        let check_start = Instant::now();
        let violations = guard.inspect(&sol, &sep.frozen_mask, &term).unwrap();
        let cold_check = check_start.elapsed().as_secs_f64();
        let check_start = Instant::now();
        for _ in 0..20 {
            guard.inspect(&sol, &sep.frozen_mask, &term).unwrap();
        }
        let warm_check = check_start.elapsed().as_secs_f64() / 20.;
        let mut repair_term = BasicTerminator::new();
        repair_term.new_timeout(Duration::from_secs(5));
        let repair_start = Instant::now();
        let repaired = guard.repair(&mut sep, &repair_term, growth);
        let repair_seconds = repair_start.elapsed().as_secs_f64();
        let fixed = sep.prob.save();
        let final_violations = guard
            .inspect(&fixed, &sep.frozen_mask, &BasicTerminator::new())
            .unwrap()
            .len();
        let status = format!("{repaired:?}");
        println!(
            "{name} seed={seed} overlap={overlap}: phase={phase_seconds:.4}s check={cold_check:.4}s cached={warm_check:.6}s conflicts={} repair={status} {repair_seconds:.4}s remaining={final_violations}",
            violations.len()
        );
        let export = |s: &SPSolution| serde_json::json!({"width":s.strip_width(),"poses":s.layout_snapshot().placed_items().values().map(|p|{let e=jagua_rs::io::export::int_to_ext_transformation(&p.d_transf(),&inst.items[p.item().idx()].0.shape_orig().pre_transform);serde_json::json!({"id":p.item().idx(),"rotation":e.rotation(),"translation":e.translation()})}).collect::<Vec<_>>()});
        rows.push(serde_json::json!({"stage":name,"phase_seconds":phase_seconds,"cold_check_seconds":cold_check,"warm_check_seconds":warm_check,"initial_violating_pairs":violations.len(),"repair_status":status,"repair_seconds":repair_seconds,"remaining_pairs":final_violations,"before":export(&sol),"after":export(&fixed),"transforms":guard.stats.transformed,"pair_checks":guard.stats.exact_pairs}));
    }
    std::fs::write(output,serde_json::to_vec_pretty(&serde_json::json!({"input":input,"seed":seed,"overlap":overlap,"allow_width_growth":growth,"import_seconds":imported,"init_seconds":built-imported,"rows":rows})).unwrap()).unwrap();
}
