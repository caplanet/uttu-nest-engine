"""Freeze the current core in an isolated workspace and add experimental variants.
No production Rust source is modified. The output is intentionally not overwritten.
"""
import hashlib
import json
import shutil
from pathlib import Path

repo = Path(__file__).resolve().parents[1]
out = repo / 'target/separator-variants-trial'
src = out / 'source'
if src.exists():
    raise SystemExit(f'Refusing to overwrite existing experiment: {src}')
src.mkdir(parents=True)
hashes = {}
for folder in ('swarm-core', 'swarm-nfp', 'data'):
    shutil.copytree(repo / folder, src / folder, ignore=shutil.ignore_patterns('target', '.git'))
for f in ('Cargo.lock', 'rust-toolchain.toml'):
    shutil.copy2(repo / f, src / f)
for path in (repo / 'swarm-core/src').rglob('*.rs'):
    hashes[str(path.relative_to(repo))] = hashlib.sha256(path.read_bytes()).hexdigest()
(out / 'production-source-hashes.json').write_text(json.dumps(hashes, indent=2), encoding='utf-8')
profiles = (repo / 'Cargo.toml').read_text(encoding='utf-8').split('[profile.dev]', 1)[1]
(src / 'Cargo.toml').write_text('[workspace]\nmembers = ["swarm-core", "swarm-nfp"]\nresolver = "3"\n\n[profile.dev]'+profiles, encoding='utf-8')

def edit(path, before, after, count=1):
    path = src / path
    text = path.read_text(encoding='utf-8')
    assert text.count(before) == count, (str(path), before[:80], text.count(before), count)
    path.write_text(text.replace(before, after), encoding='utf-8')

support = r'''
//! Experiment-only controls, instrumentation and certified migration mailbox.
use std::{cell::Cell, sync::{Mutex, OnceLock, atomic::{AtomicU64, Ordering}}, time::Instant};
use jagua_rs::{geometry::{DTransformation, primitives::Rect}, probs::spp::entities::SPSolution};
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Mode { Baseline, NoopEmpty, Preparation, Shared, Kernel, Profile }
static MODE: OnceLock<Mode> = OnceLock::new();
pub fn set_mode(s: &str) { let _ = MODE.set(match s {
    "noop_empty" => Mode::NoopEmpty, "preparation" => Mode::Preparation,
    "shared" => Mode::Shared, "kernel" => Mode::Kernel, "profile" => Mode::Profile,
    _ => Mode::Baseline }); }
pub fn mode() -> Mode { *MODE.get_or_init(|| Mode::Baseline) }
static BEST: Mutex<Option<SPSolution>> = Mutex::new(None);
pub static NOOPS: AtomicU64 = AtomicU64::new(0);
pub static EMPTY: AtomicU64 = AtomicU64::new(0);
pub static PREPARED: AtomicU64 = AtomicU64::new(0);
pub static MIGRATIONS: AtomicU64 = AtomicU64::new(0);
pub static PUBLICATIONS: AtomicU64 = AtomicU64::new(0);
pub static EARLY_REJECTS: AtomicU64 = AtomicU64::new(0);
pub fn count(c: &AtomicU64) { c.fetch_add(1, Ordering::Relaxed); }
pub fn publish(s: &SPSolution) {
    if mode()!=Mode::Shared { return; }
    let mut best=BEST.lock().unwrap();
    if best.as_ref().is_none_or(|b| s.strip_width()<b.strip_width()) {
        *best=Some(s.clone()); count(&PUBLICATIONS);
    }
}
pub fn newer(width: f32) -> Option<SPSolution> {
    if mode()!=Mode::Shared { return None; }
    BEST.lock().unwrap().as_ref().filter(|s|s.strip_width()<width).cloned()
}
// A zero bound rejects every real exterior collision. Only use exact zero
// rotation, for which translated shape extrema are directly available.
// The small strict margin makes this an intentionally incomplete reject filter;
// everything else continues through the unchanged precise evaluator.
pub fn definitely_outside(dt:DTransformation,b:Rect,c:Rect)->bool {
    if dt.rotation()!=0.0 { return false; }
    let (x,y)=dt.translation();
    let margin=0.001;
    b.x_min+x < c.x_min-margin || b.y_min+y < c.y_min-margin ||
    b.x_max+x > c.x_max+margin || b.y_max+y > c.y_max+margin
}
thread_local! { static TICK: Cell<u64> = const { Cell::new(0) }; static SAMPLED: Cell<bool> = const { Cell::new(false) }; }
static EVAL_NS:AtomicU64=AtomicU64::new(0);
static TRANSFORM_NS:AtomicU64=AtomicU64::new(0);
static COLLECT_NS:AtomicU64=AtomicU64::new(0);
static SAMPLES:AtomicU64=AtomicU64::new(0);
static ZERO_BOUNDS:AtomicU64=AtomicU64::new(0);
static OUTSIDE_POTENTIAL:AtomicU64=AtomicU64::new(0);
pub struct Timer { start:Option<Instant>, kind:u8 }
impl Timer {
    pub fn eval(zero:bool,outside:bool)->Self {
        let sampled=mode()==Mode::Profile && TICK.with(|t|{ let n=t.get()+1;t.set(n);n%256==0 });
        SAMPLED.with(|s|s.set(sampled));
        if sampled {count(&SAMPLES);if zero{count(&ZERO_BOUNDS);}if zero&&outside{count(&OUTSIDE_POTENTIAL);}}
        Self{start:sampled.then(Instant::now),kind:0}
    }
    pub fn part(kind:u8)->Self {Self{start:SAMPLED.with(Cell::get).then(Instant::now),kind}}
}
impl Drop for Timer {fn drop(&mut self){if let Some(s)=self.start {
    let v=s.elapsed().as_nanos() as u64;
    match self.kind {0=>{EVAL_NS.fetch_add(v,Ordering::Relaxed);SAMPLED.with(|s|s.set(false));},
        1=>{TRANSFORM_NS.fetch_add(v,Ordering::Relaxed);},_=>{COLLECT_NS.fetch_add(v,Ordering::Relaxed);}}
}}}
pub fn snapshot()->serde_json::Value {use Ordering::Relaxed as R;serde_json::json!({
    "skipped_noops":NOOPS.load(R),"skipped_empty_rounds":EMPTY.load(R),"prepared_rounds":PREPARED.load(R),
    "migrations":MIGRATIONS.load(R),"publications":PUBLICATIONS.load(R),"early_rejects":EARLY_REJECTS.load(R),
    "profile_samples":SAMPLES.load(R),"profile_eval_ns":EVAL_NS.load(R),"profile_transform_ns":TRANSFORM_NS.load(R),
    "profile_collect_ns":COLLECT_NS.load(R),"profile_zero_bounds":ZERO_BOUNDS.load(R),"profile_outside_potential":OUTSIDE_POTENTIAL.load(R)})}
'''
(src / 'swarm-core/src/trial_support.rs').write_text(support, encoding='utf-8')
edit('swarm-core/src/lib.rs', 'pub mod config;', 'pub mod trial_support;\npub mod config;')

# A: exact no-op elimination and an already-clear separation fast path.
edit('swarm-core/src/optimizer/worker.rs', '                self.move_item(pk, new_dt);', '''                if crate::trial_support::mode()==crate::trial_support::Mode::NoopEmpty
                    && new_dt==self.prob.layout.placed_items[pk].d_transf {
                    crate::trial_support::count(&crate::trial_support::NOOPS); continue;
                }
                self.move_item(pk, new_dt);''')
edit('swarm-core/src/optimizer/worker.rs', '            self.move_item(pk, new_dt);\n            total_moves += 1;', '''            if crate::trial_support::mode()==crate::trial_support::Mode::NoopEmpty
                && new_dt==self.prob.layout.placed_items[pk].d_transf {
                crate::trial_support::count(&crate::trial_support::NOOPS); continue;
            }
            self.move_item(pk, new_dt);
            total_moves += 1;''')
edit('swarm-core/src/optimizer/separator.rs', '        let mut min_loss = self.ct.get_total_loss();', '''        let mut min_loss = self.ct.get_total_loss();
        if crate::trial_support::mode()==crate::trial_support::Mode::NoopEmpty && min_loss==0.0 {
            crate::trial_support::count(&crate::trial_support::EMPTY);
            if self.original_guard.is_some() && !term.kill() && self.repair_original_overlap(term) {
                min_loss_sol=(self.prob.save(),self.ct.save());
            }
            return min_loss_sol;
        }''')

# B: collect the initial movable/conflicting keys once. The shuffle order and
# subsequent current-loss checks are unchanged; rigid units use the old path.
worker = src / 'swarm-core/src/optimizer/worker.rs'
w = worker.read_text(encoding='utf-8')
start = w.index('    fn move_items_legacy(')
end = w.index('        let mut total_moves = 0;', start)
old = w[start:end]
collect = old[old.index('        let mask ='):]
collect = collect.replace('        let candidates = self','        let candidates = self').replace('.tap_mut(|v| v.shuffle(&mut self.rng));','.tap_mut(|v| v.shuffle(&mut self.rng));')
new = '''    pub fn move_items_prepared(&mut self, term: &impl Terminator, keys: Option<&[PItemKey]>) -> SepStats {
        if self.place_groups.is_empty() { self.move_items_legacy_with_keys(term,keys) }
        else { self.move_items(term) }
    }
    fn move_items_legacy(&mut self, term: &impl Terminator) -> SepStats {
        self.move_items_legacy_with_keys(term,None)
    }
    fn move_items_legacy_with_keys(&mut self, term: &impl Terminator, keys:Option<&[PItemKey]>) -> SepStats {
        let candidates = match keys {
            Some(keys)=>keys.to_vec().tap_mut(|v|v.shuffle(&mut self.rng)),
            None=>{
'''+collect+'''                candidates
            }
        };

'''
worker.write_text(w[:start]+new+w[end:], encoding='utf-8')
edit('swarm-core/src/optimizer/separator.rs', '        let main_metrics_before = crate::util::phase_metrics::snapshot();', '''        let main_metrics_before = crate::util::phase_metrics::snapshot();
        let prepared = if crate::trial_support::mode()==crate::trial_support::Mode::Preparation && self.place_groups.is_empty() {
            crate::trial_support::count(&crate::trial_support::PREPARED);
            Some(self.prob.layout.placed_items.keys().filter(|pk| {
                let id=self.prob.layout.placed_items[*pk].item_id;
                !self.frozen_mask.get(id).copied().unwrap_or(false) && self.ct.get_loss(*pk)>0.0
            }).collect::<Vec<_>>())
        } else {None};''')
edit('swarm-core/src/optimizer/separator.rs', '                    let report = worker.move_items(term);', '                    let report = worker.move_items_prepared(term, prepared.as_deref());')

# C: migration occurs ONLY at a completed separation boundary, never inside a
# slow worker's current attempt. Certification and all width bookkeeping follow.
edit('swarm-core/src/optimizer/explore_mods.rs', '''    while !term.kill() {
        // Attempt to separate the current layout''', '''    while !term.kill() {
        if let Some(incoming)=crate::trial_support::newer(best_width) {
            if sep.solution_is_certified(&incoming,&initial_solution) {
                sep.rollback(&incoming,None);
                current_width=incoming.strip_width(); best_width=current_width;
                feasible_sols.push(incoming.clone()); infeas_sol_pool.clear();
                pending_op=None; n_conseq_fails=0; n_fails_at_width=0;
                next_relief_attempt=config.relief_fail_threshold; step_scale=1.0;
                tabu.clear(); ops.reset_noops();
                crate::trial_support::count(&crate::trial_support::MIGRATIONS);
                if let Some(l)=sol_listener.as_deref_mut() { l.report(ReportType::ExplFeas,&incoming,instance); }
            }
        }
        // Attempt to separate the current layout''')

# D/profile: conservative exterior rejection for a zero collision upper bound.
edit('swarm-core/src/eval/sep_evaluator.rs', '        // Reload the hazard collector to prepare for a new query', '''        let trial_mode=crate::trial_support::mode();
        let outside = (trial_mode==crate::trial_support::Mode::Kernel || trial_mode==crate::trial_support::Mode::Profile)
            && loss_bound==0.0 && crate::trial_support::definitely_outside(dt,self.item.shape_cd.bbox,self.layout.container.outer_cd.bbox);
        let _trial_timer=crate::trial_support::Timer::eval(loss_bound==0.0,outside);
        if trial_mode==crate::trial_support::Mode::Kernel && outside {
            crate::trial_support::count(&crate::trial_support::EARLY_REJECTS);
            return SampleEval::Invalid;
        }
        // Reload the hazard collector to prepare for a new query''')
edit('swarm-core/src/eval/specialized_jaguars_pipeline.rs', '    let t = dt.compose();', '    let trial_transform=crate::trial_support::Timer::part(1);\n    let t = dt.compose();')
edit('swarm-core/src/eval/specialized_jaguars_pipeline.rs', '''    {
        // We start off by checking a few poles''', '''    drop(trial_transform);
    let _trial_collect=crate::trial_support::Timer::part(2);
    {
        // We start off by checking a few poles''')

# Extend only the copied harness; the previous experiment stays reproducible.
e = src / 'swarm-core/examples/async_separator_trial.rs'
text = e.read_text(encoding='utf-8')
text = text.replace('entities::Instance, ', '')
text = text.replace('if mode == "async"', 'if mode == "async" || mode == "shared"')
text = text.replace('                self.feasible += 1;', '                self.feasible += 1;\n                swarm_core::trial_support::publish(s);')
text = text.replace('    let lanes =', '    swarm_core::trial_support::publish(&initial);\n    let lanes =', 1)
text = text.replace('"solution":export(i,&best)', '"trial_counters":swarm_core::trial_support::snapshot(),"solution":export(i,&best)')
text = text.replace('    let seconds: f64 =', '    swarm_core::trial_support::set_mode(&a[4]);\n    let seconds: f64 =', 1)
text = text.replace('"sync"|"async"=>run', '"sync"|"async"|"noop_empty"|"preparation"|"shared"|"kernel"|"profile"=>run')
e.write_text(text, encoding='utf-8')
print(src)
