//! Trial original-outline certification and bounded transactional correction.
//!
//! The acceptance metric matches Uttu's intersection followed by half-depth
//! erosion. Broad-phase uses ORIGINAL bounds, never the shrunken search bounds.
//! A local guard owns all pose/pair caches; independent runs share no mutable state.
use crate::optimizer::separator::Separator;
use crate::util::terminator::Terminator;
use clipper2::{EndType, FillRule, JoinType, Paths, PointScaler};
use jagua_rs::entities::{PItemKey};
use jagua_rs::geometry::DTransformation;
use jagua_rs::io::export::int_to_ext_transformation;
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
mod kernels;

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ExactGrid;
impl PointScaler for ExactGrid {
    const MULTIPLIER: f64 = 10_000.0;
}
type Outline = Paths<ExactGrid>;
// Half of the existing piece-overlap setting, with a sub-mm grid guard.
// No second Config value or independently hardcoded overlap budget.
const GRID_GUARD_MM: f64 = 0.0005;

/// Apply [`OriginalOverlapPolicy`] to a freshly imported instance.
///
/// The guard is active exactly when an item's original shape carries the
/// negative offset that piece overlap introduces ([`OriginalOverlapGuard::is_active`]),
/// and every user of the guard (LBF construction, separator, compression and
/// Polish entry repair, certification) checks that one predicate. With
/// [`OriginalOverlapPolicy::Feasible`] the offset marker is cleared on the
/// original shapes, so the guard is off everywhere — including grouped lanes,
/// which clone these items — and layouts whose original outlines overlap
/// deeper than the allowance are accepted. The collision shapes are untouched:
/// the search still uses pieces shrunk by half the allowance. Fabric-edge
/// checks do not depend on the guard and keep working.
///
/// Returns whether any item was changed.
pub fn apply_policy(
    instance: &mut SPInstance,
    policy: crate::config::OriginalOverlapPolicy,
) -> bool {
    if policy == crate::config::OriginalOverlapPolicy::Infeasible
        || !OriginalOverlapGuard::is_active(instance)
    {
        return false;
    }
    for (item, _) in &mut instance.items {
        if item.shape_orig().modify_config.offset.is_some_and(|v| v < 0.) {
            let mut original = item.shape_orig().as_ref().clone();
            original.modify_config.offset = None;
            *item = std::sync::Arc::new(item.as_ref().clone().with_shape_orig(std::sync::Arc::new(original)));
        }
    }
    log::info!(
        "[ORIGINAL] original-outline overlap beyond piece_overlap_mm is accepted (policy = feasible)"
    );
    true
}

#[derive(Clone)]
pub(crate) struct PlacedOutline {
    id: usize,
    pose: DTransformation,
    revision: u64,
    paths: Outline,
    bounds: [f64; 4],
}

impl PlacedOutline {
    /// `[min_x, min_y, max_x, max_y]` of the transformed **original** outline.
    /// `fabric_edge` compares this against the strip rectangle; no pairwise check
    /// in this module looks at the container at all.
    pub(crate) fn bounds(&self) -> [f64; 4] {
        self.bounds
    }
}
#[derive(Debug, Clone)]
pub struct Violation {
    pub a: PItemKey,
    pub b: PItemKey,
    pub area: f64,
}
#[derive(Debug, Default, Clone, Copy)]
pub struct GuardStats {
    pub transformed: usize,
    pub exact_pairs: usize,
    pub cached_pairs: usize,
    pub parallel_pairs: usize,
    pub checks: usize,
}
pub struct OriginalOverlapGuard {
    instance: SPInstance,
    groups: crate::optimizer::place_groups::PlaceGroups,
    half_depth: f64,
    rows: HashMap<PItemKey, PlacedOutline>,
    pairs: HashMap<(PItemKey, PItemKey), (u64, u64, f64)>,
    revision: u64,
    pub stats: GuardStats,
}
fn bounds(paths: &Outline) -> [f64; 4] {
    let b = paths.bounds();
    [b.min.x(), b.min.y(), b.max.x(), b.max.y()]
}
fn intersection_area(a: &PlacedOutline, b: &PlacedOutline, half_depth: f64) -> Result<f64, String> {
    let x = a.bounds[2].min(b.bounds[2]) - a.bounds[0].max(b.bounds[0]);
    let y = a.bounds[3].min(b.bounds[3]) - a.bounds[1].max(b.bounds[1]);
    if x <= 2.0 * half_depth || y <= 2.0 * half_depth {
        return Ok(0.0);
    }
    let common = clipper2::intersect(a.paths.clone(), b.paths.clone(), FillRule::NonZero)
        .map_err(|e| e.to_string())?;
    if common.is_empty() {
        return Ok(0.0);
    }
    Ok(
        clipper2::inflate(common, -half_depth, JoinType::Bevel, EndType::Polygon, 2.0)
            .signed_area()
            .abs(),
    )
}
impl OriginalOverlapGuard {
    pub fn new(instance: &SPInstance) -> Self {
        let half_depth = instance
            .items
            .iter()
            .map(|(i, _)| (-i.shape_orig().modify_config.offset.unwrap_or(0.) as f64).max(0.))
            .fold(f64::INFINITY, f64::min);
        Self {
            instance: instance.clone(),
            groups: crate::optimizer::place_groups::PlaceGroups::default(),
            half_depth: (half_depth - GRID_GUARD_MM).max(0.),
            rows: HashMap::new(),
            pairs: HashMap::new(),
            revision: 0,
            stats: GuardStats::default(),
        }
    }
    pub fn set_place_groups(&mut self, groups: &crate::optimizer::place_groups::PlaceGroups) {
        self.groups.clone_from(groups);
    }
    pub fn is_active(instance: &SPInstance) -> bool {
        instance
            .items
            .iter()
            .any(|(i, _)| i.shape_orig().modify_config.offset.is_some_and(|v| v < 0.))
    }
    pub(crate) fn outline(
        &self,
        id: usize,
        pose: DTransformation,
    ) -> Result<PlacedOutline, String> {
        // Reject before composing NotNan matrices: infinity * zero would panic
        // inside Jagua instead of returning a failed certification.
        let (x, y) = pose.translation();
        if !x.is_finite() || !y.is_finite() || !pose.rotation().is_finite() {
            return Err("non-finite original transform".into());
        }
        let item = &self
            .instance
            .items
            .get(id)
            .ok_or_else(|| format!("unknown original item {id}"))?
            .0;
        let ext = int_to_ext_transformation(&pose, &item.shape_orig().pre_transform);
        let (tx, ty) = ext.translation();
        let r = ext.rotation();
        if !tx.is_finite() || !ty.is_finite() || !r.is_finite() {
            return Err("non-finite original transform".into());
        }
        let (sin, cos) = (r as f64).sin_cos();
        // One path per part; an ordinary item has a single part.
        let mut polygons = Vec::with_capacity(item.n_parts());
        for part in item.parts_orig() {
            polygons.push(kernels::transform(
                part.shape.vertices(),
                [sin, cos, tx as f64, ty as f64],
            )?);
        }
        let paths: Outline = polygons.into();
        Ok(PlacedOutline {
            id,
            pose,
            revision: self.revision,
            bounds: bounds(&paths),
            paths,
        })
    }
    /// Use the same original-outline predicate when proposing a Draft position
    /// as when certifying the final public result. Callers retain placed outlines.
    pub(crate) fn outlines_allowed(
        &self,
        a: &PlacedOutline,
        b: &PlacedOutline,
    ) -> Result<bool, String> {
        Ok(self.groups.same_group(a.id, b.id) || intersection_area(a, b, self.half_depth)? <= 0.0)
    }

    /// 충돌 도형 bbox 로 사전 거르기할 때 쓰는 여유(mm). 충돌 도형 bbox 는 원본 bbox 보다
    /// 허용치만큼 안쪽이므로 `2 x 허용치` 에 5mm 를 더한다. 넉넉한 사전 거르기일 뿐이고
    /// 판정은 [`Self::outlines_allowed`] 가 한다.
    pub(crate) fn reach(&self) -> f64 {
        2.0 * self.half_depth + 5.0
    }

    /// 후보 외곽 하나가 `layout` 에 이미 놓인 모든 조각과 원본 겹침 계약을 지키는지.
    ///
    /// 구성기가 조각을 **놓기 전에** 묻는 용도다. 충돌 도형(침식된 외곽)끼리 정확히 맞닿으면
    /// 원본 외곽은 `2 x 허용치` 만큼 겹치는데, 인증은 격자 여유([`GRID_GUARD_MM`])만큼 더
    /// 엄격해서 그 자리를 기각한다. «접촉까지 밀기» 를 하는 구성기는 이 술어로 자리를 고르지
    /// 않으면 놓는 족족 계약을 어긴다.
    pub(crate) fn outline_allowed_in_layout(
        &self,
        layout: &jagua_rs::entities::Layout,
        candidate: &PlacedOutline,
    ) -> Result<bool, String> {
        self.outline_allowed_in_layout_excluding(layout, candidate, None)
    }

    /// [`Self::outline_allowed_in_layout`] 와 같되, 이미 놓인 조각 `skip` 을 옮기는 중이라
    /// 그 조각의 현재 자리는 비교하지 않는다.
    pub(crate) fn outline_allowed_in_layout_excluding(
        &self,
        layout: &jagua_rs::entities::Layout,
        candidate: &PlacedOutline,
        skip: Option<jagua_rs::entities::PItemKey>,
    ) -> Result<bool, String> {
        // 충돌 도형 bbox 는 원본 bbox 보다 허용치만큼 안쪽이다. 넉넉한 사전 거르기일 뿐이고
        // 판정은 `outlines_allowed` 가 한다.
        let reach = self.reach();
        for (pk, placed) in layout.placed_items().iter() {
            if Some(pk) == skip {
                continue;
            }
            let b = placed.shape().bbox();
            if b.x_min as f64 - reach > candidate.bounds[2]
                || (b.x_max as f64) + reach < candidate.bounds[0]
                || b.y_min as f64 - reach > candidate.bounds[3]
                || (b.y_max as f64) + reach < candidate.bounds[1]
            {
                continue;
            }
            let other = self.outline(placed.item().idx(), placed.d_transf())?;
            if !self.outlines_allowed(candidate, &other)? {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Synchronize construction's original-outline cache, including warm starts,
    /// rekeyed layouts and poses changed by settle. Original bounds are authoritative;
    /// an eroded collision bbox is not a safe broad phase for sharp source tips.
    pub(crate) fn sync_construction_layout(
        &mut self,
        layout: &jagua_rs::entities::Layout,
    ) -> Result<(), String> {
        self.rows
            .retain(|key, _| layout.placed_items().contains_key(*key));
        for (key, placed) in layout.placed_items() {
            if crate::config::runtime::init_cancel_requested() { return Err("initialization cancelled".into()); }
            if self
                .rows
                .get(&key)
                .is_some_and(|row| row.id == placed.item().idx() && row.pose == placed.d_transf())
            {
                continue;
            }
            self.revision += 1;
            self.rows
                .insert(key, self.outline(placed.item().idx(), placed.d_transf())?);
            self.stats.transformed += 1;
        }
        Ok(())
    }

    /// Only used after sync_construction_layout, while the layout stays unchanged.
    pub(crate) fn construction_allows(
        &self,
        members: &[jagua_rs::probs::spp::entities::SPPlacement],
    ) -> Result<bool, String> {
        let mut candidates = Vec::with_capacity(members.len());
        for member in members {
            let candidate = self.outline(member.item_idx, member.d_transf)?;
            for other in self.rows.values().chain(candidates.iter()) {
                if crate::config::runtime::init_cancel_requested() { return Err("initialization cancelled".into()); }
                if !self.outlines_allowed(&candidate, other)? {
                    return Ok(false);
                }
            }
            candidates.push(candidate);
        }
        Ok(true)
    }

    pub(crate) fn construction_right_edge(&self) -> f64 {
        self.rows
            .values()
            .map(|row| row.bounds[2])
            .fold(0.0, f64::max)
    }

    pub fn inspect(
        &mut self,
        sol: &SPSolution,
        frozen: &[bool],
        term: &impl Terminator,
    ) -> Result<Vec<Violation>, String> {
        self.inspect_mode(sol, frozen, term, false)
    }

    /// Certify until the first forbidden pair. Repair and diagnostics use
    /// `inspect` instead because they need the complete set of violations.
    pub fn first_violation(
        &mut self,
        sol: &SPSolution,
        frozen: &[bool],
        term: &impl Terminator,
    ) -> Result<Option<Violation>, String> {
        Ok(self.inspect_mode(sol, frozen, term, true)?.into_iter().next())
    }

    fn inspect_mode(
        &mut self,
        sol: &SPSolution,
        frozen: &[bool],
        term: &impl Terminator,
        first_only: bool,
    ) -> Result<Vec<Violation>, String> {
        self.stats.checks += 1;
        if term.kill() {
            return Err("original validation cancelled".into());
        }
        let active: HashSet<_> = sol.layout_snapshot().placed_items().keys().collect();
        self.rows.retain(|k, _| active.contains(k));
        if self.pairs.len()
            > active
                .len()
                .saturating_mul(active.len())
                .saturating_mul(2)
                .max(16)
        {
            self.pairs.clear();
        }
        for (key, pi) in sol.layout_snapshot().placed_items() {
            if term.kill() {
                return Err("original validation cancelled".into());
            }
            if self
                .rows
                .get(&key)
                .is_some_and(|r| r.id == pi.item().idx() && r.pose == pi.d_transf())
            {
                continue;
            }
            self.revision += 1;
            self.rows
                .insert(key, self.outline(pi.item().idx(), pi.d_transf())?);
            self.stats.transformed += 1;
        }
        let mut order: Vec<_> = self.rows.keys().copied().collect();
        order.sort_by(|a, b| {
            self.rows[a].bounds[0]
                .total_cmp(&self.rows[b].bounds[0])
                .then_with(|| a.cmp(b))
        });
        let parallel = order.len() >= 64 && rayon::current_num_threads() > 1;
        let mut prewarmed = !first_only;
        if parallel && prewarmed {
            self.precompute_expensive_pairs(&order, frozen, term)?;
        }
        let mut checked_pairs = 0;
        let mut violations = Vec::new();
        for (i, &a) in order.iter().enumerate() {
            if term.kill() {
                return Err("original validation cancelled".into());
            }
            for chunk in order[i + 1..].chunks(4) {
                // Bound cancellation latency to one SIMD batch of exact pairs.
                // An individual Clipper operation cannot be interrupted here.
                if term.kill() {
                    return Err("original validation cancelled".into());
                }
                // Early failures need no speculative Clipper work. Valid cold
                // layouts still benefit from the existing bounded parallel batch.
                if parallel && !prewarmed && checked_pairs >= 8 {
                    self.precompute_expensive_pairs(&order, frozen, term)?;
                    prewarmed = true;
                }
                let ra = &self.rows[&a];
                if self.rows[&chunk[0]].bounds[0] >= ra.bounds[2] - 2.0 * self.half_depth {
                    break;
                }
                let boxes = std::array::from_fn::<_, 4, _>(|j| {
                    chunk.get(j).map_or([0.; 4], |k| self.rows[k].bounds)
                });
                let mask = kernels::candidate_mask(
                    ra.bounds,
                    &boxes[..chunk.len()],
                    2.0 * self.half_depth,
                );
                for (lane, &b) in chunk.iter().enumerate() {
                    if mask & (1 << lane) == 0 {
                        continue;
                    }
                    let rb = &self.rows[&b];
                    if self.groups.same_group(ra.id, rb.id) {
                        continue;
                    }
                    if frozen.get(ra.id).copied().unwrap_or(false)
                        && frozen.get(rb.id).copied().unwrap_or(false)
                    {
                        continue;
                    }
                    checked_pairs += 1;
                    let area = if let Some(&(va, vb, area)) = self
                        .pairs
                        .get(&(a, b))
                        .filter(|(va, vb, _)| *va == ra.revision && *vb == rb.revision)
                    {
                        let _ = (va, vb);
                        self.stats.cached_pairs += 1;
                        area
                    } else {
                        self.stats.exact_pairs += 1;
                        let area = intersection_area(ra, rb, self.half_depth)?;
                        self.pairs.insert((a, b), (ra.revision, rb.revision, area));
                        area
                    };
                    if area > 0.0 {
                        violations.push(Violation { a, b, area });
                        if first_only {
                            if term.kill() {
                                return Err("original validation cancelled".into());
                            }
                            return Ok(violations);
                        }
                    }
                }
            }
        }
        Ok(violations)
    }

    /// Bound both queued memory and speculative work. Each worker owns its
    /// Clipper input/output; only the caller mutates the cache after joining.
    fn precompute_expensive_pairs(
        &mut self,
        order: &[PItemKey],
        frozen: &[bool],
        term: &impl Terminator,
    ) -> Result<(), String> {
        use rayon::prelude::*;
        let mut jobs = Vec::new();
        let mut vertices = 0usize;
        'outer: for (i, &a) in order.iter().enumerate() {
            if term.kill() {
                return Err("original validation cancelled".into());
            }
            let ra = &self.rows[&a];
            for &b in &order[i + 1..] {
                let rb = &self.rows[&b];
                if rb.bounds[0] >= ra.bounds[2] - 2.0 * self.half_depth {
                    break;
                }
                if self.groups.same_group(ra.id, rb.id) {
                    continue;
                }
                if frozen.get(ra.id).copied().unwrap_or(false)
                    && frozen.get(rb.id).copied().unwrap_or(false)
                {
                    continue;
                }
                if kernels::candidate_mask(ra.bounds, &[rb.bounds], 2.0 * self.half_depth) == 0 {
                    continue;
                }
                if self
                    .pairs
                    .get(&(a, b))
                    .is_some_and(|(va, vb, _)| *va == ra.revision && *vb == rb.revision)
                {
                    continue;
                }
                vertices += self.instance.items[ra.id].0.shape_orig().shape.vertices().len()
                    + self.instance.items[rb.id].0.shape_orig().shape.vertices().len();
                jobs.push((a, b));
                if jobs.len() == 128 {
                    break 'outer;
                }
            }
        }
        if jobs.len() < 16 || vertices < 8192 {
            return Ok(());
        }
        let rows = &self.rows;
        let depth = self.half_depth;
        let results: Vec<_> = jobs
            .par_iter()
            .map(|&(a, b)| {
                if term.kill() {
                    return Err("original validation cancelled".into());
                }
                intersection_area(&rows[&a], &rows[&b], depth)
            })
            .collect();
        for ((a, b), result) in jobs.into_iter().zip(results) {
            let area = result?;
            self.stats.exact_pairs += 1;
            self.stats.parallel_pairs += 1;
            self.pairs.insert(
                (a, b),
                (self.rows[&a].revision, self.rows[&b].revision, area),
            );
        }
        Ok(())
    }

    /// Correct only colliding placement units. Each trial changes the unit as a
    /// whole, checks affected original pairs and retains the search CDE contract.
    /// Failure/cancellation restores the exact incoming snapshot, including width.
    pub fn repair(
        &mut self,
        sep: &mut Separator,
        term: &impl Terminator,
        allow_width_growth: bool,
    ) -> Result<bool, String> {
        self.set_place_groups(&sep.place_groups);
        let incoming = sep.prob.save();
        let result = self.repair_inner(sep, term, allow_width_growth);
        if !matches!(result, Ok(true)) {
            sep.rollback(&incoming, None);
        } else if allow_width_growth {
            sep.refit_strip();
        }
        result
    }
    fn repair_inner(
        &mut self,
        sep: &mut Separator,
        term: &impl Terminator,
        allow_width_growth: bool,
    ) -> Result<bool, String> {
        // Two sweeps allow local correction without unbounded polishing.
        for _ in 0..sep.prob.layout().placed_items().len().saturating_mul(2).max(1) {
            let current = sep.prob.save();
            let conflicts = self.inspect(&current, &sep.frozen_mask, term)?;
            if conflicts.is_empty() {
                return Ok(sep.ct.get_total_loss() == 0.0);
            }
            let worst = conflicts
                .iter()
                .max_by(|a, b| a.area.total_cmp(&b.area))
                .unwrap();
            let keys = [worst.a, worst.b];
            let total: f64 = conflicts.iter().map(|v| v.area).sum();
            let mut best: Option<(SPSolution, f64)> = None;
            for key in keys {
                let pi = &current.layout_snapshot().placed_items()[key];
                if sep.frozen_mask.get(pi.item().idx()).copied().unwrap_or(false) {
                    continue;
                }
                let representative = sep.place_groups.representative(pi.item().idx());
                let rep_key = if sep.place_groups.group_id_of(pi.item().idx()).is_none() {
                    key
                } else {
                    let Some(k) = current
                        .layout_snapshot()
                        .placed_items()
                        .iter()
                        .find(|(_, p)| p.item().idx() == representative)
                        .map(|(k, _)| k)
                    else {
                        continue;
                    };
                    k
                };
                let rep_pose = current.layout_snapshot().placed_items()[rep_key].d_transf();
                let member_ids = sep.place_groups.member_item_ids(representative);
                let unit_keys: HashSet<_> = if sep.place_groups.group_id_of(pi.item().idx()).is_none() {
                    HashSet::from([key])
                } else {
                    current
                        .layout_snapshot()
                        .placed_items()
                        .iter()
                        .filter(|(_, p)| member_ids.contains(&p.item().idx()))
                        .map(|(k, _)| k)
                        .collect()
                };
                let old_local: f64 = conflicts
                    .iter()
                    .filter(|v| unit_keys.contains(&v.a) || unit_keys.contains(&v.b))
                    .map(|v| v.area)
                    .sum();
                let requested = self
                    .instance
                    .item(pi.item().idx())
                    .shape_orig()
                    .modify_config
                    .offset
                    .unwrap_or(0.)
                    .abs()
                    * 2.;
                let mut steps = vec![
                    0.005f32,
                    0.02,
                    0.1,
                    0.5,
                    1.0,
                    2.0,
                    4.0,
                    (requested - 2.0 * self.half_depth as f32 + 0.005).max(0.005),
                ];
                steps.sort_by(f32::total_cmp);
                steps.dedup();
                for step in steps {
                    for (dx, dy) in [
                        (1., 0.),
                        (-1., 0.),
                        (0., 1.),
                        (0., -1.),
                        (0.707, 0.707),
                        (0.707, -0.707),
                        (-0.707, 0.707),
                        (-0.707, -0.707),
                    ] {
                        if term.kill() {
                            return Err("original correction cancelled".into());
                        }
                        // Score only changed-to-unchanged pairs before allocating a
                        // problem snapshot or rebuilding the search tracker.
                        let mut local = 0.0;
                        for &uk in &unit_keys {
                            let p = &current.layout_snapshot().placed_items()[uk];
                            let (ux, uy) = p.d_transf().translation();
                            let trial = self.outline(
                                p.item().idx(),
                                DTransformation::new(
                                    p.d_transf().rotation(),
                                    (ux + dx * step, uy + dy * step),
                                ),
                            )?;
                            for (&other, row) in &self.rows {
                                if unit_keys.contains(&other) {
                                    continue;
                                }
                                local += intersection_area(&trial, row, self.half_depth)?;
                            }
                        }
                        let estimate = total - old_local + local;
                        if estimate >= best.as_ref().map_or(total, |(_, cost)| *cost) - 1e-10 {
                            continue;
                        }
                        let Some(pk) = sep
                            .prob
                            .layout()
                            .placed_items()
                            .iter()
                            .find(|(_, p)| p.item().idx() == representative && p.d_transf() == rep_pose)
                            .map(|(key, _)| key)
                        else {
                            return Err("correction unit disappeared".into());
                        };
                        let (x, y) = rep_pose.translation();
                        sep.move_item(
                            pk,
                            DTransformation::new(
                                rep_pose.rotation(),
                                (x + dx * step, y + dy * step),
                            ),
                        );
                        if sep.ct.get_total_loss() != 0.0 {
                            sep.rollback(&current, None);
                            continue;
                        }
                        let candidate = sep.prob.save();
                        let cost: f64 = self
                            .inspect(&candidate, &sep.frozen_mask, term)?
                            .iter()
                            .map(|v| v.area)
                            .sum();
                        if cost < total - 1e-10 && best.as_ref().is_none_or(|(_, old)| cost < *old)
                        {
                            best = Some((candidate, cost));
                        }
                        if cost == 0.0 {
                            return Ok(true);
                        }
                        sep.rollback(&current, None);
                        self.inspect(&current, &sep.frozen_mask, term)?;
                    }
                }
            }
            sep.rollback(&current, None);
            if let Some((candidate, _)) = best {
                sep.rollback(&candidate, None);
                continue;
            }
            if allow_width_growth {
                // Deterministic completion fallback only during initial repair.
                // Put one movable unit beyond all existing ORIGINAL and CD bounds.
                let movable = keys.into_iter().find(|key| {
                    !sep.frozen_mask
                        .get(current.layout_snapshot().placed_items()[*key].item().idx())
                        .copied()
                        .unwrap_or(false)
                });
                if let Some(key) = movable {
                    let id = sep
                        .place_groups
                        .representative(current.layout_snapshot().placed_items()[key].item().idx());
                    let pk = sep.placement_key(id).unwrap();
                    let pose = sep.prob.layout().placed_items()[pk].d_transf();
                    let unit = sep.place_groups.member_item_ids(id);
                    let right = self
                        .rows
                        .values()
                        .map(|r| r.bounds[2])
                        .fold(0.0f64, f64::max)
                        .max(
                            sep.prob
                                .layout()
                                .placed_items()
                                .values()
                                .map(|p| p.shape().bbox().x_max as f64)
                                .fold(0.0, f64::max),
                        );
                    let left = self
                        .rows
                        .values()
                        .filter(|r| unit.contains(&r.id))
                        .map(|r| r.bounds[0])
                        .fold(f64::INFINITY, f64::min);
                    let width = self
                        .rows
                        .values()
                        .filter(|r| unit.contains(&r.id))
                        .map(|r| r.bounds[2])
                        .fold(f64::NEG_INFINITY, f64::max)
                        - left;
                    let shift = (right - left + 0.01) as f32;
                    sep.change_strip_width(
                        current.strip_width().max((right + width + 10.0) as f32),
                        Some(f32::INFINITY),
                    );
                    // Changing width restores/rekeys the layout as well. Never
                    // retain a SlotMap key across that transactional operation.
                    let pk = sep
                        .prob
                        .layout()
                        .placed_items()
                        .iter()
                        .find(|(_, p)| p.item().idx() == id && p.d_transf() == pose)
                        .map(|(key, _)| key)
                        .ok_or_else(|| {
                            "correction unit disappeared after width change".to_string()
                        })?;
                    let (x, y) = pose.translation();
                    sep.move_item(pk, DTransformation::new(pose.rotation(), (x + shift, y)));
                    if sep.ct.get_total_loss() == 0.0 {
                        continue;
                    }
                    sep.rollback(&current, None);
                }
            }
            return Ok(false);
        }
        Ok(false)
    }
}

thread_local! {
    // Geometry Arcs owned by each guard keep pointer identities live. Bounded
    // per-thread LRU avoids a global lock and releases old solver instances.
    static CERTIFICATION_CACHE: RefCell<Vec<(Vec<usize>,OriginalOverlapGuard)>> = const {RefCell::new(Vec::new())};
}
pub fn validate(
    instance: &SPInstance,
    solution: &SPSolution,
    frozen: &[bool],
) -> Result<(), String> {
    validate_with_groups(
        instance,
        solution,
        frozen,
        &crate::optimizer::place_groups::PlaceGroups::default(),
    )
}

pub fn validate_with_groups(
    instance: &SPInstance,
    solution: &SPSolution,
    frozen: &[bool],
    groups: &crate::optimizer::place_groups::PlaceGroups,
) -> Result<(), String> {
    if !OriginalOverlapGuard::is_active(instance) {
        return Ok(());
    }
    let identity: Vec<_> = instance
        .items
        .iter()
        .map(|(i, _)| std::sync::Arc::as_ptr(&i.shape_orig()) as usize)
        .collect();
    CERTIFICATION_CACHE.with(|cell| {
        let mut cache = cell.borrow_mut();
        let entry = cache
            .iter()
            .position(|(key, _)| *key == identity)
            .map(|index| cache.remove(index))
            .unwrap_or_else(|| (identity, OriginalOverlapGuard::new(instance)));
        cache.push(entry);
        if cache.len() > 4 {
            cache.remove(0);
        }
        let guard = &mut cache.last_mut().unwrap().1;
        guard.set_place_groups(groups);
        let violation = guard.first_violation(
            solution,
            frozen,
            &crate::util::terminator::InitTerminator::new(
                crate::util::terminator::BasicTerminator::new(),
            ),
        )?;
        if let Some(pair) = violation {
            let a = solution.layout_snapshot().placed_items()[pair.a].item().idx();
            let b = solution.layout_snapshot().placed_items()[pair.b].item().idx();
            Err(format!(
                "original overlap exceeds configured piece_overlap_mm: items {a}/{b}"
            ))
        } else {
            Ok(())
        }
    })
}

/// Used at initialization boundaries where a Separator does not exist yet.
/// The supplied deadline includes correction; failure leaves the input intact.
pub fn repair_problem(
    prob: &mut jagua_rs::probs::spp::entities::SPProblem,
    frozen_ids: &[usize],
    groups: &crate::optimizer::place_groups::PlaceGroups,
    term: &impl Terminator,
) -> Result<bool, String> {
    #[cfg(feature = "init-profile")]
    let _profile = crate::init_profile::Scope::new("detail.original_repair");

    let frozen = crate::util::frozen::mask(prob.instance().items.len(), frozen_ids);
    if validate_with_groups(&prob.instance(), &prob.save(), &frozen, groups).is_ok() {
        return Ok(true);
    }
    if term.kill() { return Ok(false); }
    use rand::SeedableRng;
    let mut config = crate::config::DEFAULT_SWARM_CONFIG
        .expl_cfg
        .separator_config;
    // Correction never runs sampler workers or NFP preparation.
    config.n_workers = 1;
    config.nfp_sampler = false;
    let mut sep = Separator::new(
        prob.instance().clone(),
        prob.clone(),
        rand::rngs::Xoshiro256PlusPlus::seed_from_u64(0),
        config,
    );
    sep.set_place_groups(groups.clone())
        .map_err(|e| format!("original repair groups: {e:?}"))?;
    sep.freeze_items(frozen_ids);
    if sep.repair_original_overlap(term) {
        *prob = sep.prob;
        return Ok(true);
    }
    if !term.kill() && OriginalOverlapGuard::new(&sep.instance).repair(&mut sep, term, true)? {
        *prob = sep.prob;
        return Ok(true);
    }
    Ok(false)
}

#[cfg(test)]
mod policy_tests {
    use super::*;
    use crate::config::{DEFAULT_SWARM_CONFIG, OriginalOverlapPolicy};
    use jagua_rs::probs::spp::entities::{SPPlacement, SPProblem};
    use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;

    /// Two 40 x 30 rectangles imported with a 0.4 mm piece overlap allowance.
    fn overlap_allowed_pair() -> SPInstance {
        let items: Vec<_> = (0..2)
            .map(|id| {
                serde_json::json!({
                    "id": id, "demand": 1, "allowed_orientations": [0.0],
                    "shape": {"type": "simple_polygon",
                              "data": [[0.0, 0.0], [40.0, 0.0], [40.0, 30.0], [0.0, 30.0]]}
                })
            })
            .collect();
        let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
            "name": "original-overlap-policy", "strip_height": 100.0, "items": items
        }))
        .unwrap();
        let c = DEFAULT_SWARM_CONFIG;
        crate::util::io::import_spp_instance(
            &ext,
            c.cde_config,
            c.poly_simpl_tolerance,
            Some(-0.4),
            c.narrow_concavity_cutoff_ratio,
        )
        .unwrap()
    }

    /// The shrunk collision shapes are disjoint, but the original outlines
    /// overlap by 0.3995 mm, more than the 0.4 mm allowance permits (half each).
    fn excess_overlap_layout(instance: &SPInstance) -> SPSolution {
        let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
        prob.change_strip_width(200.0).expect(crate::dmsg!("strip width must be positive"));
        for (item_id, x) in [(0, 30.0), (1, 69.6005)] {
            prob.place_item(SPPlacement {
                item_idx: item_id,
                d_transf: DTransformation::new(0.0, (x, 30.0)),
            });
        }
        prob.save()
    }

    #[test]
    fn infeasible_policy_keeps_rejecting_excess_original_overlap() {
        let mut instance = overlap_allowed_pair();
        assert!(!apply_policy(&mut instance, OriginalOverlapPolicy::Infeasible));
        assert!(OriginalOverlapGuard::is_active(&instance));
        let layout = excess_overlap_layout(&instance);
        assert!(validate(&instance, &layout, &[]).is_err());
        assert!(
            crate::structure::pipeline::solution_validation_error(&instance, &layout).is_some()
        );
    }

    #[test]
    fn feasible_policy_accepts_excess_original_overlap() {
        let mut instance = overlap_allowed_pair();
        let shrunk = instance.items[0].0.shape_cd().clone();
        assert!(apply_policy(&mut instance, OriginalOverlapPolicy::Feasible));
        assert!(!OriginalOverlapGuard::is_active(&instance));
        // The search geometry is unchanged: pieces stay shrunk by half the allowance.
        assert!(std::sync::Arc::ptr_eq(&shrunk, &instance.items[0].0.shape_cd()));
        let layout = excess_overlap_layout(&instance);
        assert!(validate(&instance, &layout, &[]).is_ok());
        assert!(
            crate::structure::pipeline::solution_validation_error(&instance, &layout).is_none(),
            "an accepted layout must pass the shared exact validation"
        );
        // Applying it again is a no-op.
        assert!(!apply_policy(&mut instance, OriginalOverlapPolicy::Feasible));
    }

    #[test]
    fn policy_without_piece_overlap_changes_nothing() {
        let mut instance = overlap_allowed_pair();
        for (item, _) in &mut instance.items {
            let mut original = item.shape_orig().as_ref().clone();
            original.modify_config.offset = None;
            *item = std::sync::Arc::new(item.as_ref().clone().with_shape_orig(std::sync::Arc::new(original)));
        }
        assert!(!apply_policy(&mut instance, OriginalOverlapPolicy::Feasible));
    }
}
