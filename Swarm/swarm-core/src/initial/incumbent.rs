//! Certified initial candidates and bounded improvement of an existing layout.
use crate::optimizer::{place_groups::PlaceGroups, separator::Separator};
use crate::util::terminator::Terminator;
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};

pub struct InitialSeed<'a> {
    pub solution: &'a SPSolution,
    pub frozen_ids: &'a [usize],
    pub groups: &'a PlaceGroups,
}

pub(super) struct SeedTerm<'a> {
    deadline: jagua_rs::Instant,
    stop: Option<&'a std::sync::atomic::AtomicBool>,
}
impl<'a> SeedTerm<'a> {
    pub(super) fn new(ms: u64, stop: Option<&'a std::sync::atomic::AtomicBool>) -> Self {
        Self {
            deadline: jagua_rs::Instant::now() + std::time::Duration::from_millis(ms),
            stop,
        }
    }
}
impl Terminator for SeedTerm<'_> {
    fn kill(&self) -> bool {
        self.stop_requested() || jagua_rs::Instant::now() >= self.deadline
    }
    fn stop_requested(&self) -> bool {
        self.stop
            .is_some_and(|s| s.load(std::sync::atomic::Ordering::Relaxed))
    }
    fn new_timeout(&mut self, timeout: std::time::Duration) {
        self.deadline = self.deadline.min(jagua_rs::Instant::now() + timeout);
    }
    fn timeout_at(&self) -> Option<jagua_rs::Instant> {
        Some(self.deadline)
    }
}

/// A partial layout is never compared to a complete incumbent by width alone.
/// Equal-width candidates are useful only when their right-edge profile improves.
pub fn candidate_is_improvement(
    instance: &SPInstance,
    candidate: &SPSolution,
    incumbent: &SPSolution,
    frozen_ids: &[usize],
    groups: &PlaceGroups,
) -> bool {
    if !candidate.strip_width().is_finite() || candidate.strip_width() > incumbent.strip_width() {
        return false;
    }
    let mask = crate::util::frozen::mask(instance.items.len(), frozen_ids);
    if !crate::util::assertions::frozen_poses_preserved(incumbent, candidate, &mask)
        || !groups.solution_preserves_relative_poses(candidate)
        || crate::structure::pipeline::solution_validation_error_with_frozen_ids_and_groups(
            instance, candidate, frozen_ids, groups,
        )
        .is_some()
    {
        return false;
    }
    candidate.strip_width() < incumbent.strip_width()
        || right_edges(candidate) < right_edges(incumbent)
}

fn right_edges(sol: &SPSolution) -> Vec<ordered_float::OrderedFloat<f32>> {
    let mut edges: Vec<_> = sol
        .layout_snapshot()
        .placed_items()
        .values()
        .map(|p| ordered_float::OrderedFloat(p.shape().bbox().x_max))
        .collect();
    edges.sort_unstable_by(|a, b| b.cmp(a));
    edges
}

/// Rigid/frozen or arbitrary-angle seeds cannot safely be reconstructed by the
/// discrete N06 engine. Improve only movable units using existing group-aware
/// operators; every trial is a transaction against the certified incumbent.
pub fn improve_seeded_units(
    sep: &mut Separator,
    initial: &SPSolution,
    compact: &super::SlideCompactCfg,
    attempts: usize,
    term: &impl Terminator,
    mut on_improve: Option<&mut dyn FnMut(&SPSolution)>,
) -> Result<SPSolution, String> {
    if !sep.solution_is_certified(initial, initial) {
        return Err("Initial seed is not a complete certified layout".into());
    }
    let frozen: Vec<_> = sep
        .frozen_mask
        .iter()
        .enumerate()
        .filter_map(|(id, &f)| f.then_some(id))
        .collect();
    let mut best = initial.clone();
    sep.rollback(&best, None);
    let slide = crate::config::SlideConfig {
        n_passes: compact.passes_max,
        ..crate::config::DEFAULT_POLISH_CONFIG.slide
    };
    let ruin = crate::config::RuinRecreateConfig {
        period: 1,
        band_fraction: 0.18,
        max_items: compact.reseat_top,
        ..crate::config::DEFAULT_SWARM_CONFIG.expl_cfg.rnr
    };
    let mut tabu = crate::optimizer::lns::BandTabu::new();
    for attempt in 0..attempts.max(1) {
        if term.kill() {
            break;
        }
        if attempt > 0 && compact.reseat_top > 0 {
            crate::optimizer::lns::ruin_and_recreate(
                sep,
                &ruin,
                crate::optimizer::lns::RuinStrategy::Rightmost,
                &mut tabu,
                term,
            );
            if !term.kill() && sep.ct.get_total_loss() != 0.0 {
                let (separated, _) = sep.separate(term, &mut None);
                sep.rollback(&separated, None);
            }
        }
        if compact.passes_max > 0 && !term.kill() && sep.ct.get_total_loss() == 0.0 {
            crate::optimizer::polish::slide::slide_pass(sep, &slide, term);
        }
        if term.kill() {
            break;
        }
        sep.refit_strip();
        if sep.ct.get_total_loss() == 0.0 && sep.repair_original_overlap(term) && !term.kill() {
            let candidate = sep.prob.save();
            if candidate_is_improvement(
                &sep.instance,
                &candidate,
                &best,
                &frozen,
                &sep.place_groups,
            ) {
                best = candidate;
                if let Some(cb) = on_improve.as_deref_mut() {
                    cb(&best);
                }
            }
        }
        sep.rollback(&best, None);
    }
    sep.rollback(&best, None);
    Ok(best)
}
