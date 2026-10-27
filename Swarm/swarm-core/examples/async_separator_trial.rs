//! Isolated experiment; no production scheduler or configuration is changed.
//! Compare the existing 3-worker barrier with 3 independent 1-worker Explore
//! streams at the SAME total geometry-worker budget and shared wall deadline.
//! The latter shares certified output, not live search state or cancellation.
use anyhow::{Result, ensure};
use jagua_rs::{probs::spp::entities::{SPInstance, SPProblem, SPSolution}};
use rand::{SeedableRng, rngs::Xoshiro256PlusPlus};
use rayon::prelude::*;
use serde_json::json;
use swarm_core::{
    config::DEFAULT_SWARM_CONFIG as C,
    consts::LBF_SAMPLE_CONFIG,
    optimizer::{lbf::LBFBuilder, separator::Separator},
    original_overlap::OriginalOverlapGuard,
    structure::pipeline::solution_validation_error,
    util::{frozen, io::import_spp_instance_with_mm,
        listener::{ReportType, SolutionListener},
        terminator::{BasicTerminator, Terminator}},
};
use std::{path::Path, sync::mpsc, time::{Duration, Instant}};

fn certify(i: &SPInstance, s: &SPSolution) -> Result<()> {
    ensure!(s.layout_snapshot().placed_items().len() == i.total_item_qty(), "missing placements");
    ensure!(solution_validation_error(i, s).is_none(), "native certification failed");
    let mut guard = OriginalOverlapGuard::new(i);
    ensure!(guard.inspect(s, &[], &BasicTerminator::new()).map_err(anyhow::Error::msg)?.is_empty(),
        "cold original certification failed");
    Ok(())
}
fn write(path: &str, v: &impl serde::Serialize) -> Result<()> {
    if let Some(p) = Path::new(path).parent() { std::fs::create_dir_all(p)?; }
    std::fs::write(path, serde_json::to_vec_pretty(v)?)?;
    Ok(())
}
fn export(_i: &SPInstance, s: &SPSolution) -> serde_json::Value {
    serde_json::to_value(jagua_rs::probs::spp::io::export(s, *swarm_core::EPOCH)).unwrap()
}
struct Event { lane: usize, at: f64, solution: SPSolution }
struct Listener {
    lane: usize, start: Instant, tx: mpsc::Sender<Event>, best: f32,
    feasible: usize, infeasible: usize, improving: usize,
}
impl SolutionListener for Listener {
    fn report(&mut self, kind: ReportType, s: &SPSolution, _: &SPInstance) {
        match kind {
            ReportType::ExplFeas => {
                self.feasible += 1;
                if s.strip_width() < self.best {
                    self.best = s.strip_width();
                    let _ = self.tx.send(Event { lane: self.lane,
                        at: self.start.elapsed().as_secs_f64(), solution: s.clone() });
                }
            }
            ReportType::ExplInfeas => self.infeasible += 1,
            ReportType::ExplImproving => self.improving += 1,
            _ => {}
        }
    }
}
fn make_sep(i: &SPInstance, p: SPProblem, seed: u64, workers: usize) -> Separator {
    let mut cfg = C.expl_cfg.separator_config;
    cfg.n_workers = workers; // Resource partition inside this experiment only.
    let mut sep = Separator::new(i.clone(), p, Xoshiro256PlusPlus::seed_from_u64(seed), cfg);
    sep.configure_reinsertion(&C.nfp_integration);
    sep
}
fn run(i: &SPInstance, p: SPProblem, mode: &str, seconds: f64, workers: usize, seed: u64) -> Result<serde_json::Value> {
    let initial = p.save();
    certify(i, &initial)?;
    let lanes = if mode == "async" { workers } else { 1 };
    let inner_workers = if mode == "async" { 1 } else { workers };
    let setup = Instant::now();
    let separators: Vec<_> = (0..lanes).map(|lane|
        make_sep(i, p.clone(), seed.wrapping_add(lane as u64 * 1_000_003), inner_workers)).collect();
    let setup_seconds = setup.elapsed().as_secs_f64();
    let (tx, rx) = mpsc::channel();
    let start = Instant::now();
    let deadline = start + Duration::from_secs_f64(seconds);
    let mut best = initial.clone();
    let mut trace = vec![json!({"at":0.,"received_at":0.,"lane":null,"width":best.strip_width(),"density":best.density()})];
    let lane_rows = std::thread::scope(|scope| {
        let mut handles = Vec::new();
        for (lane, mut sep) in separators.into_iter().enumerate() {
            let tx = tx.clone();
            handles.push(scope.spawn(move || {
                let term = BasicTerminator { timeout: Some(deadline) };
                let mut listener = Listener { lane, start, tx, best: sep.prob.strip_width(),
                    feasible:0, infeasible:0, improving:0 };
                let local_initial = sep.prob.save();
                let mut local_best = local_initial.clone();
                let mut calls = 0;
                while !term.kill() {
                    calls += 1;
                    let mut callback: Option<&mut dyn SolutionListener> = Some(&mut listener);
                    let sols = swarm_core::optimizer::explore_mods::exploration_phase_adaptive(
                        &sep.instance.clone(), &mut sep, &mut callback, &term, &C.expl_cfg);
                    if let Some(s) = sols.last() {
                        if s.strip_width() < local_best.strip_width() { local_best = s.clone(); }
                        listener.report(ReportType::ExplFeas, s, &sep.instance);
                    }
                    // No peer-success cancellation. A naturally ended stream can
                    // continue with its own best until the SAME global deadline.
                    if !term.kill() { sep.rollback(&local_best, None); }
                }
                let elapsed = start.elapsed().as_secs_f64();
                (json!({"lane":lane,"elapsed":elapsed,"explore_calls":calls,
                    "feasible_reports":listener.feasible,"failed_attempts":listener.infeasible,
                    "loss_improvements":listener.improving,
                    "final_width":local_best.strip_width()}), local_best)
            }));
        }
        drop(tx);
        // Receive certified improvements without waiting for the other streams.
        // Existing Explore certifies ExplFeas before invoking the listener.
        for event in rx {
            if event.solution.strip_width() < best.strip_width() {
                best = event.solution;
                trace.push(json!({"at":event.at,"received_at":start.elapsed().as_secs_f64(),
                    "lane":event.lane,"width":best.strip_width(),"density":best.density()}));
            }
        }
        handles.into_iter().map(|h| h.join().expect("trial worker panic")).collect::<Vec<_>>()
    });
    let elapsed = start.elapsed().as_secs_f64();
    let cold = Instant::now();
    for (_, solution) in &lane_rows { certify(i, solution)?; }
    certify(i, &best)?;
    Ok(json!({"mode":mode,"seed":seed,"budget_seconds":seconds,
        "geometry_workers":workers,"streams":lanes,"workers_per_stream":inner_workers,
        "setup_seconds":setup_seconds,"elapsed_seconds":elapsed,
        "cold_validation_seconds":cold.elapsed().as_secs_f64(),"certified":true,
        "initial_width":initial.strip_width(),"width":best.strip_width(),"density":best.density(),
        "trace":trace,"lanes":lane_rows.into_iter().map(|(r,_)|r).collect::<Vec<_>>(),
        "solution":export(i,&best)}))
}
fn probe(i: &SPInstance, p: SPProblem, workers: usize, seed: u64) -> serde_json::Value {
    let mut sep = make_sep(i, p, seed, workers);
    sep.change_strip_width(sep.prob.strip_width() * 0.99, None);
    let term = BasicTerminator::new();
    let mut rounds = Vec::new();
    for iteration in 0..80 {
        if sep.ct.get_total_loss() == 0. { sep.change_strip_width(sep.prob.strip_width() * 0.995, None); }
        let master = sep.prob.save();
        let tracker = &sep.ct;
        let start = Instant::now();
        let pool = sep.thread_pool.as_ref().unwrap();
        let rows = pool.install(|| sep.workers.par_iter_mut().map(|w| {
            let begun = start.elapsed().as_secs_f64();
            w.load(&master, tracker);
            let loaded = start.elapsed().as_secs_f64();
            w.move_items(&term);
            json!({"begun":begun,"loaded":loaded,"done":start.elapsed().as_secs_f64(),
                "loss":w.ct.get_total_loss(),"weighted_loss":w.ct.get_total_weighted_loss()})
        }).collect::<Vec<_>>());
        let seconds = start.elapsed().as_secs_f64();
        let winner = sep.workers.iter().enumerate().min_by(|(_,a),(_,b)|
            a.ct.get_total_weighted_loss().total_cmp(&b.ct.get_total_weighted_loss())).unwrap().0;
        std::mem::swap(&mut sep.prob, &mut sep.workers[winner].prob);
        std::mem::swap(&mut sep.ct, &mut sep.workers[winner].ct);
        rounds.push(json!({"iteration":iteration,"seconds":seconds,"winner":winner,"workers":rows}));
        sep.ct.update_weights();
    }
    json!({"mode":"probe","rounds":rounds,"workers":workers,
        "note":"Isolated move_items_multi replica; no swap hook or full Explore lifecycle"})
}
fn main() -> Result<()> {
    let a: Vec<_> = std::env::args().collect();
    ensure!(a.len()==9, "INPUT INITIAL OUTPUT MODE SECONDS WORKERS SEED OVERLAP_MM");
    let seconds: f64 = a[5].parse()?;
    let workers: usize = a[6].parse()?;
    let seed: u64 = a[7].parse()?;
    let overlap: f32 = a[8].parse()?;
    ensure!(workers>0 && seconds.is_finite() && seconds>=0. && overlap.is_finite() && overlap>=0., "bad args");
    let ext = serde_json::from_slice(&std::fs::read(&a[1])?)?;
    let i = import_spp_instance_with_mm(&ext,C.cde_config,Some(0.01),Some(-overlap),C.narrow_concavity_cutoff_ratio,None)?;
    if a[4]=="prepare" {
        let built = LBFBuilder::new(i.clone(),Xoshiro256PlusPlus::seed_from_u64(17),
            LBF_SAMPLE_CONFIG,C.lbf_x_multiplier,C.lbf_y_multiplier).construct();
        let mut sep = make_sep(&i,built.prob,seed,workers);
        ensure!(sep.repair_original_overlap(&BasicTerminator::new()),"initial repair failed");
        if seconds>0. {
            let mut term=BasicTerminator::new(); term.new_timeout(Duration::from_secs_f64(seconds));
            let sols=swarm_core::optimizer::explore_mods::exploration_phase_adaptive(&i,&mut sep,&mut None,&term,&C.expl_cfg);
            if let Some(s)=sols.last(){ sep.rollback(s,None); }
        }
        let s=sep.prob.save(); certify(&i,&s)?;
        write(&a[2],&export(&i,&s))?;
        return write(&a[3],&json!({"initial_width":s.strip_width(),"density":s.density(),"items":i.total_item_qty(),"certified":true}));
    }
    let initial=serde_json::from_slice(&std::fs::read(&a[2])?)?;
    let p=frozen::import_solution(&i,&initial);
    let mut report=match a[4].as_str(){
        "sync"|"async"=>run(&i,p,&a[4],seconds,workers,seed)?,
        "probe"=>probe(&i,p,workers,seed),
        _=>anyhow::bail!("unknown mode")
    };
    report["input"]=json!(a[1]); report["overlap_mm"]=json!(overlap);
    report["poly_simpl"]=json!(0.01);
    write(&a[3],&report)
}
