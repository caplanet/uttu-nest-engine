//! Diagnostic only: measure exact candidate repetitions without caching results.
//! Usage: explore_candidate_probe INPUT_JSON SAVED_INITIAL_JSON OUTPUT_JSON [OVERLAP_MM]
//! Import matches adaptive_control_after_trial (simplification 0.01, default CDE).
//! Each shrink starts from the same saved initial. Searches inspect a fixed
//! shrunken layout: candidate placements are NOT committed between searches.
use anyhow::{Result, ensure};
use jagua_rs::{
    geometry::DTransformation,
};
use rand::{SeedableRng, prelude::SliceRandom, rngs::Xoshiro256PlusPlus};
use serde_json::json;
use swarm_core::{
    config::DEFAULT_SWARM_CONFIG as C,
    eval::{
        sample_eval::{SampleEval, SampleEvaluator},
        sep_evaluator::{PooledSeparationEvaluator, SeparationEvaluatorPool},
    },
    optimizer::separator::Separator,
    original_overlap::OriginalOverlapGuard,
    prepared_geometry::PreparedGeometry,
    sample::search::search_placement_prepared_with_control,
    structure::pipeline::solution_validation_error,
    util::{frozen, io::import_spp_instance_with_mm, terminator::BasicTerminator},
};
use std::{collections::{HashSet, VecDeque}, path::Path};

type Key = [u32; 3];

fn key(dt: DTransformation) -> Key {
    let (x, y) = dt.translation();
    [dt.rotation().to_bits(), x.to_bits(), y.to_bits()]
}

// This reproduces SeparationEvaluator's loss-bound interpretation, including
// None/Invalid => infinity, Clear => zero, and strictly-greater rejection.
fn loss_bound(bound: Option<SampleEval>) -> f32 {
    match bound {
        Some(SampleEval::Collision { loss }) => loss,
        Some(SampleEval::Clear { .. }) => 0.0,
        _ => f32::INFINITY,
    }
}

#[derive(Clone, Copy)]
struct Record {
    key: Key,
    bound: f32,
    result: SampleEval,
}

impl Record {
    fn reusable(self, new_bound: f32) -> Option<SampleEval> {
        match self.result {
            // Invalid contains no exact loss, only evidence it exceeded bound.
            SampleEval::Invalid => (new_bound <= self.bound).then_some(SampleEval::Invalid),
            SampleEval::Clear { loss } | SampleEval::Collision { loss } => {
                Some(if loss > new_bound { SampleEval::Invalid } else { self.result })
            }
        }
    }
}

fn same_result(a: SampleEval, b: SampleEval) -> bool {
    match (a, b) {
        (SampleEval::Invalid, SampleEval::Invalid) => true,
        (SampleEval::Clear { loss: a }, SampleEval::Clear { loss: b })
        | (SampleEval::Collision { loss: a }, SampleEval::Collision { loss: b }) => {
            a.to_bits() == b.to_bits()
        }
        _ => false,
    }
}

#[derive(Default, serde::Serialize)]
struct Counts {
    total_calls: usize,
    all_history_repetitions: usize,
    last8_key_hits: usize,
    last8_usable_hits: usize,
    last8_invalid_reuses: usize,
    last8_exact_reuses: usize,
    last8_bound_rejections: usize,
    reuse_prediction_mismatches: usize,
}

impl Counts {
    fn add(&mut self, other: &Self) {
        self.total_calls += other.total_calls;
        self.all_history_repetitions += other.all_history_repetitions;
        self.last8_key_hits += other.last8_key_hits;
        self.last8_usable_hits += other.last8_usable_hits;
        self.last8_invalid_reuses += other.last8_invalid_reuses;
        self.last8_exact_reuses += other.last8_exact_reuses;
        self.last8_bound_rejections += other.last8_bound_rejections;
        self.reuse_prediction_mismatches += other.reuse_prediction_mismatches;
    }
}

struct Probe<'a, E> {
    inner: E,
    counts: &'a mut Counts,
    history: HashSet<Key>,
    // Exactly the previous eight actual evaluator calls, newest searched first.
    // This is a rolling window, not a simulated LRU or insertion-on-miss cache.
    recent: VecDeque<Record>,
}

impl<E: SampleEvaluator> SampleEvaluator for Probe<'_, E> {
    fn evaluate_sample(&mut self, dt: DTransformation, bound: Option<SampleEval>) -> SampleEval {
        let key = key(dt);
        let new_bound = loss_bound(bound);
        self.counts.total_calls += 1;
        if !self.history.insert(key) {
            self.counts.all_history_repetitions += 1;
        }
        let key_hit = self.recent.iter().any(|record| record.key == key);
        self.counts.last8_key_hits += usize::from(key_hit);
        let predicted = self.recent.iter().rev().filter(|record| record.key == key)
            .find_map(|record| record.reusable(new_bound).map(|result| (record.result, result)));
        if let Some((old, _)) = predicted {
            self.counts.last8_usable_hits += 1;
            if matches!(old, SampleEval::Invalid) {
                self.counts.last8_invalid_reuses += 1;
            } else {
                self.counts.last8_exact_reuses += 1;
            }
        } else if key_hit {
            self.counts.last8_bound_rejections += 1;
        }
        // Always evaluate. Neither predicted values nor counters affect search,
        // RNG consumption, logical n_evals, layout, tracker, or bound arguments.
        let result = self.inner.evaluate_sample(dt, bound);
        if let Some((_, prediction)) = predicted {
            self.counts.reuse_prediction_mismatches += usize::from(!same_result(prediction, result));
        }
        if self.recent.len() == 8 { self.recent.pop_front(); }
        self.recent.push_back(Record { key, bound: new_bound, result });
        result
    }

    fn n_evals(&self) -> usize { self.inner.n_evals() }
}

// Stable file provenance without adding a dependency. Explicitly noncryptographic.
fn file_provenance(path: &str, bytes: &[u8]) -> Result<serde_json::Value> {
    let fingerprint = bytes.iter().fold(0xcbf29ce484222325u64,
        |hash, byte| (hash ^ u64::from(*byte)).wrapping_mul(0x100000001b3));
    Ok(json!({"path":std::fs::canonicalize(path)?, "bytes":bytes.len(),
        "fnv1a64_noncryptographic":format!("{fingerprint:016x}")}))
}

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().collect();
    ensure!(args.len() == 4 || args.len() == 5,
        "INPUT_JSON SAVED_INITIAL_JSON OUTPUT_JSON [OVERLAP_MM=2.2]");
    let overlap: f32 = args.get(4).map(|s| s.parse()).transpose()?.unwrap_or(2.2);
    ensure!(overlap.is_finite() && overlap >= 0.0, "invalid overlap");
    let input_bytes = std::fs::read(&args[1])?;
    let initial_bytes = std::fs::read(&args[2])?;
    let ext = serde_json::from_slice(&input_bytes)?;
    let instance = import_spp_instance_with_mm(&ext, C.cde_config, Some(0.01),
        Some(-overlap), C.narrow_concavity_cutoff_ratio, None)?;
    let initial = serde_json::from_slice(&initial_bytes)?;
    let problem = frozen::import_solution(&instance, &initial);
    let saved = problem.save();
    ensure!(saved.layout_snapshot().placed_items().len() == instance.total_item_qty(), "missing initial placements");
    ensure!(solution_validation_error(&instance, &saved).is_none(), "initial native certification failed");
    let mut guard = OriginalOverlapGuard::new(&instance);
    ensure!(guard.inspect(&saved, &[], &BasicTerminator::new())
        .map_err(anyhow::Error::msg)?.is_empty(), "initial cold original certification failed");
    let prepared = PreparedGeometry::new(&instance);
    let pool = SeparationEvaluatorPool::default();
    let seeds = [41u64, 42, 43];
    let shrinks = [0.995f32, 0.99, 0.98];
    let mut totals = Counts::default();
    let mut searches = Vec::new();
    let mut fixtures = Vec::new();
    for shrink in shrinks {
        let mut config = C.expl_cfg.separator_config;
        config.n_workers = 3;
        let mut separator = Separator::new(instance.clone(), problem.clone(),
            Xoshiro256PlusPlus::seed_from_u64(17), config);
        separator.change_strip_width(saved.strip_width() * shrink, None);
        let layout = &separator.prob.layout();
        let tracker = &separator.ct;
        let colliding: Vec<_> = layout.placed_items().keys()
            .filter(|&pk| tracker.get_loss(pk) > 0.0).collect();
        fixtures.push(json!({"shrink":shrink,"width":separator.prob.strip_width(),
            "colliding_placements":colliding.len(),"total_loss":tracker.get_total_loss()}));
        for seed in seeds {
            let mut rng = Xoshiro256PlusPlus::seed_from_u64(seed);
            let mut candidates = colliding.clone();
            candidates.shuffle(&mut rng);
            for (search_index, pk) in candidates.into_iter().enumerate() {
                let item_id = layout.placed_items()[pk].item().idx();
                let item = instance.item(item_id);
                let mut counts = Counts::default();
                let evaluator = Probe {
                    inner: PooledSeparationEvaluator::new(layout, item, pk, tracker, &pool),
                    counts: &mut counts, history: HashSet::new(), recent: VecDeque::with_capacity(8),
                };
                let (best, n_evals) = search_placement_prepared_with_control(layout,
                    item, Some(pk), evaluator, config.sample_config, &mut rng,
                    &mut || false, &prepared);
                ensure!(n_evals == counts.total_calls, "logical evaluator count changed");
                totals.add(&counts);
                searches.push(json!({"shrink":shrink,"seed":seed,"search_index":search_index,
                    "item_id":item_id,"reference_transform_bits":key(layout.placed_items()[pk].d_transf()),
                    "logical_n_evals":n_evals,"counts":counts,
                    "best_transform_bits":best.map(|(dt,_)|key(dt)),
                    "best_eval_debug":best.map(|(_,eval)|format!("{eval:?}"))}));
            }
        }
    }
    let report = json!({"diagnostic_only":true,"input":file_provenance(&args[1],&input_bytes)?,
        "saved_initial":file_provenance(&args[2],&initial_bytes)?,"initial_width":saved.strip_width(),
        "initial_certified":true,"placements":instance.total_item_qty(),"poly_simpl":0.01,
        "overlap_mm":overlap,"seeds":seeds,"shrinks":shrinks,
        "sample_config_debug":format!("{:?}",C.expl_cfg.separator_config.sample_config),
        "features":{"simd":cfg!(feature="simd"),"capmods":cfg!(feature="capmods"),
            "adaptive":cfg!(feature="adaptive"),"structure_first":cfg!(feature="structure-first")},
        "search_count":searches.len(),"totals":totals,"fixtures":fixtures,"searches":searches,
        "notes":["Every evaluation delegates to PooledSeparationEvaluator; no memoization is applied.",
            "Keys are unmodified f32 rotation/x/y bits; no quantization or cross-search reuse.",
            "last8 means previous eight actual evaluation records; history resets per search.",
            "Reusable Invalid requires new loss bound <= recorded bound; exact results are limited by the new bound.",
            "Prediction mismatch counts must be zero before considering caching; this probe does not prove overall speedup.",
            "Searches inspect fixed shrunken snapshots and do not commit placements; no NFP sampler, GLS weight updates, groups, frozen hints, or full Explore lifecycle.",
            "Seed17 constructs the separator only; each search sequence RNG is seeded 41/42/43 then shuffled.",
            "Instrumentation overhead makes wall time unsuitable for production performance comparisons."]});
    if let Some(parent) = Path::new(&args[3]).parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(&args[3], serde_json::to_vec_pretty(&report)?)?;
    Ok(())
}
