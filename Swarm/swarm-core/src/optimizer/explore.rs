use crate::config::ExplorationConfig;
use crate::optimizer::separator::Separator;
use crate::sample::uniform_sampler::convert_sample_to_closest_feasible;
use crate::util::listener::{ReportType, SolutionListener};
use crate::util::terminator::Terminator;
use float_cmp::approx_eq;
use itertools::Itertools;
use jagua_rs::collision_detection::hazards::BasicHazardEntity;
use jagua_rs::collision_detection::hazards::filter::NoFilter;
use jagua_rs::collision_detection::hazards::collector::BasicHazardCollector;
use jagua_rs::entities::{Layout, PItemKey};
use jagua_rs::geometry::geo_traits::CollidesWith;
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use log::warn;
use ordered_float::OrderedFloat;
use rand::prelude::{Distribution, IteratorRandom};
use rand_distr::Normal;
use std::cmp::Reverse;

/// I-12: infeasible 해 풀 상한. adaptive 경로(explore_mods)의 값과 동일하게 유지한다.
const INFEAS_POOL_CAP: usize = 32;

/// Algorithm 12 from https://doi.org/10.48550/arXiv.2509.13329
pub fn exploration_phase(
    instance: &SPInstance,
    sep: &mut Separator,
    sol_listener: &mut Option<&mut dyn SolutionListener>,
    term: &impl Terminator,
    config: &ExplorationConfig,
) -> Vec<SPSolution> {
    let _run_scope = crate::config::runtime::enter_if_unset(sep.run_control.clone());
    let mut current_width = sep.prob.strip_width();
    let mut best_width = current_width;

    sep.repair_original_overlap(term);
    let initial_solution = sep.prob.save();
    let initial_certified = sep.solution_is_certified(&initial_solution, &initial_solution);
    let mut feasible_sols = if initial_certified {
        vec![initial_solution.clone()]
    } else {
        vec![]
    };

    // 진입 레이아웃을 **검사 없이** ExplFeas 로 보고하면 안 된다. ExplFeas 는
    // "겹침 없음" 을 뜻하고, 소비자는 그 라벨을 믿고 배치를 그린다.
    if let Some(l) = sol_listener.as_deref_mut()
        && initial_certified
    {
        l.report(ReportType::ExplFeas, &feasible_sols[0], instance);
    }

    let mut infeas_sol_pool: Vec<(SPSolution, f32)> = vec![];
    // I-12: pool 은 상한(32)으로 자르므로, 연속 실패 종료 조건은 별도 카운터로 센다.
    // (기존에는 pool 이 성공 시 비워져 pool.len() == 연속 실패 수였다 — 의미 동일.)
    let mut n_conseq_fails = 0usize;

    // I-10: rnr 설정이 켜져 있어도 비-adaptive 빌드에서는 R&R 이 전혀 돌지 않는다.
    // 조용히 무시하지 말고 시작 시 한 번 경고한다.
    #[cfg(not(feature = "adaptive"))]
    if config.rnr.period > 0 {
        log::warn!(
            "[EXPL] rnr.period = {} 이 설정됐지만 이 빌드에는 'adaptive' feature 가 없어 \
             ruin&recreate 가 비활성 상태다 (설정 무시됨)",
            config.rnr.period
        );
    }

    while !term.kill() {
        // Attempt to separate the current layout
        let local_best = sep.separate_with_repair(term, sol_listener, &config.repair, false);
        let total_loss = local_best.1.get_total_loss();
        // 손실 0 인데 정확 인증을 통과하지 못한 후보는 이 폭에서의 실패한 시도로 다룬다
        // (`explore_mods.rs` 의 같은 자리 참조). 예전의 `break` 는 남은 예산을 버렸다.
        let total_loss = if total_loss == 0.0
            && !sep.solution_is_certified(&local_best.0, &initial_solution)
        {
            log::warn!(
                "[EXPL] zero-loss candidate failed exact certification; treating it as a failed attempt"
            );
            f32::MIN_POSITIVE
        } else {
            total_loss
        };
        // Separation returns its best snapshot; subsequent shrinking must use that state.
        if total_loss == 0.0 {
            sep.rollback(&local_best.0, Some(&local_best.1));
        }

        if total_loss == 0.0 {
            // If successfully separated
            if current_width < best_width || feasible_sols.is_empty() {
                best_width = current_width;
                feasible_sols.push(local_best.0.clone());
                if let Some(l) = sol_listener.as_deref_mut() {
                    l.report(ReportType::ExplFeas, &local_best.0, instance);
                }
            }
            // Shrink the strip width and clear the infeasible solution pool
            let step = crate::config::runtime::effective_shrink_step(config.shrink_step);
            let mut next_width = current_width * (1.0 - step);
            // 고정 아이템이 경계를 침범하지 않도록 축소 폭의 하한을 clamp
            if let Some(min_w) = sep.min_feasible_width()
                && next_width < min_w
            {
                next_width = min_w;
            }
            if next_width >= current_width {
                break;
            }
            sep.change_strip_width(next_width, None);
            current_width = next_width;
            infeas_sol_pool.clear();
            n_conseq_fails = 0;
        } else {
            if let Some(l) = sol_listener.as_deref_mut() {
                l.report(ReportType::ExplInfeas, &local_best.0, instance);
            }

            // Separation was not successful add it to the pool of infeasible solutions
            // (I-17: NaN 방어 — partial_cmp().unwrap() 대신 total_cmp)
            match infeas_sol_pool.binary_search_by(|(_, o)| o.total_cmp(&total_loss)) {
                Ok(idx) | Err(idx) => {
                    infeas_sol_pool.insert(idx, (local_best.0.clone(), total_loss))
                }
            }
            // I-12: adaptive 경로(explore_mods)와 동일한 상한 — 무제한 성장 방지.
            infeas_sol_pool.truncate(INFEAS_POOL_CAP);

            n_conseq_fails += 1;
            if n_conseq_fails >= config.max_conseq_failed_attempts.unwrap_or(usize::MAX) {
                break;
            }

            // 아래는 다음 반복을 위한 준비(해 선택 + disruption)다. 중단 요청이 들어왔다면
            // 그 결과를 쓸 일이 없으므로 시작하지 않는다.
            if term.kill() {
                break;
            }

            // Restore to a random solution from the pool, with better solutions having more chance to be selected
            let selected_sol = {
                // Sample a value in range [0.0, 1.0[ from a normal distribution
                let distribution =
                    Normal::new(0.0, config.solution_pool_distribution_stddev).unwrap();
                let sample = distribution.sample(&mut sep.rng).abs().min(0.999);
                // Map it to an index in the infeasible solution pool (better solutions are at the start of the pool)
                let selected_idx = (sample * infeas_sol_pool.len() as f32) as usize;

                &infeas_sol_pool[selected_idx].0
            };

            // Rollback to this solution and disrupt it.
            sep.rollback(selected_sol, None);
            disrupt_solution(sep, config);
        }
    }

    // Never leave a caller looking at a last, unrepaired perturbation.
    sep.rollback(feasible_sols.last().unwrap_or(&initial_solution), None);
    feasible_sols
}

// fork: visibility opened to `pub(crate)` so `optimizer::explore_mods` can reuse it.
/// Returns `true` only when a disruption was actually applied.  Adaptive
/// feedback uses this to exclude frozen/small-layout no-ops from learning.
pub(crate) fn disrupt_solution(sep: &mut Separator, config: &ExplorationConfig) -> bool {
    if sep.has_place_groups() {
        return disrupt_place_units(sep);
    }
    if sep.prob.layout().placed_items().len() < 2 {
        warn!("[DSRP] cannot disrupt solution with less than 2 items");
        return false;
    }

    // The general idea is to disrupt a solution by swapping two 'large' items in the layout.
    // 'Large' items are those whose convex hull area falls within a certain top percentile
    // of the total convex hull area of all items in the layout.

    // Step 1: Define what constitutes a 'large' item.

    // Calculate the total convex hull area of all items, considering quantities.
    let total_convex_hull_area: f32 = sep
        .prob
        .instance()
        .items
        .iter()
        .map(|(item, quantity)| item.shape_cd().surrogate().convex_hull_area * (*quantity as f32))
        .sum();

    let cutoff_threshold_area =
        total_convex_hull_area * config.large_item_ch_area_cutoff_percentile;

    // Sort items by convex hull area in descending order.
    let sorted_items_by_ch_area = sep
        .prob
        .instance()
        .items
        .iter()
        .sorted_by_key(|(item, _)| {
            Reverse(OrderedFloat(item.shape_cd().surrogate().convex_hull_area))
        })
        .peekable();

    let mut cumulative_ch_area = 0.0;
    let mut ch_area_cutoff = 0.0;

    // Iterate through items, accumulating their convex hull areas until the cumulative sum
    // exceeds the cutoff_threshold_area. The convex hull area of the item that causes
    // this excess becomes the ch_area_cutoff.
    for (item, quantity) in sorted_items_by_ch_area {
        let item_ch_area = item.shape_cd().surrogate().convex_hull_area;
        cumulative_ch_area += item_ch_area * (*quantity as f32);
        if cumulative_ch_area > cutoff_threshold_area {
            ch_area_cutoff = item_ch_area;
            break;
        }
    }

    // Step 2: Select two 'large' items and 'swap' them.

    // I-01: 고정(frozen) 아이템은 교환 대상에서 제외한다. 필터가 없으면 move_item 이
    // 이동을 거부한 채, 일어나지 않은 이동을 전제로 주변 포함 아이템들이 흩뿌려진다.
    let frozen_mask = sep.frozen_mask.clone();
    let movable = |item_id: usize| !frozen_mask.get(item_id).copied().unwrap_or(false);
    if sep
        .prob
        .layout()
        .placed_items()
        .iter()
        .filter(|(_, pi)| movable(pi.item().idx()))
        .count()
        < 2
    {
        warn!("[DSRP] cannot disrupt: fewer than 2 movable (non-frozen) items");
        return false;
    }

    let large_items = sep
        .prob
        .layout()
        .placed_items()
        .iter()
        .filter(|(_, pi)| movable(pi.item().idx()))
        .filter(|(_, pi)| pi.shape().surrogate().convex_hull_area >= ch_area_cutoff);

    //Choose a first item with a large enough convex hull.
    //If every large item is frozen, fall back to any movable item.
    let (pk1, pi1) = large_items
        .clone()
        .choose(&mut sep.rng)
        .or_else(|| {
            sep.prob
                .layout()
                .placed_items()
                .iter()
                .filter(|(_, pi)| movable(pi.item().idx()))
                .choose(&mut sep.rng)
        })
        .expect(crate::dmsg!("[DSRP] failed to choose first item"));

    //Choose a second item with a large enough convex hull and different enough from the first.
    //If no such item is found, choose a random (movable) one.
    let (pk2, pi2) = large_items.clone()
        .filter(|(pk, pi)|
            *pk != pk1 &&
            // Ensure the second item is different from the first
            !approx_eq!(f32, pi.shape().area(),pi1.shape().area(), epsilon = pi1.shape().area() * 0.01) &&
                !approx_eq!(f32, pi.shape().diameter(), pi1.shape().diameter(), epsilon = pi1.shape().diameter() * 0.01)
        )
        .choose(&mut sep.rng)
        .or_else(|| {
            sep.prob.layout().placed_items().iter()
                .filter(|(pk, pi)| *pk != pk1 && movable(pi.item().idx()))
                .choose(&mut sep.rng)
        }) // As a fallback, choose any movable item
        .expect(crate::dmsg!("[EXPL] failed to choose second item for disruption"));

    // Step 3: Swap the two items' positions in the layout.

    let dt1_old = pi1.d_transf();
    let dt2_old = pi2.d_transf();

    // Make sure the swaps do not violate feasibility (rotation).
    let dt1_new = convert_sample_to_closest_feasible(dt2_old, sep.prob.instance().item(pi1.item().idx()));
    let dt2_new = convert_sample_to_closest_feasible(dt1_old, sep.prob.instance().item(pi2.item().idx()));

    let pk1 = sep.move_item(pk1, dt1_new);
    let pk2 = sep.move_item(pk2, dt2_new);

    // Step 4: Move all items that are practically contained by one of the swapped items to the "empty space" created by the moved item.
    //         This is particularly important when huge items are swapped with smaller items.
    //         The huge item will create a large empty space and many of the items which previously
    //         surrounded the smaller one will be contained by the huge one.
    {
        // transformation to convert the contained items' position (relative to the old and new positions of the swapped items)
        let converting_transformation = dt1_new.compose().inverse().transform(&dt1_old.compose());

        for c1_pk in practically_contained_items(&sep.prob.layout(), pk1)
            .into_iter()
            .filter(|c1_pk| *c1_pk != pk2)
        {
            let c1_pi = &sep.prob.layout().placed_items()[c1_pk];

            let new_dt = c1_pi
                .d_transf()
                .compose()
                .transform(&converting_transformation)
                .decompose();

            //Ensure the sure the new position is feasible
            let new_feasible_dt =
                convert_sample_to_closest_feasible(new_dt, sep.prob.instance().item(c1_pi.item().idx()));
            sep.move_item(c1_pk, new_feasible_dt);
        }
    }

    // Do the same for the second item, but using the second transformation
    {
        let converting_transformation = dt2_new.compose().inverse().transform(&dt2_old.compose());

        for c2_pk in practically_contained_items(&sep.prob.layout(), pk2)
            .into_iter()
            .filter(|c2_pk| *c2_pk != pk1)
        {
            let c2_pi = &sep.prob.layout().placed_items()[c2_pk];
            let new_dt = c2_pi
                .d_transf()
                .compose()
                .transform(&converting_transformation)
                .decompose();

            //make sure the new position is feasible
            let new_feasible_dt =
                convert_sample_to_closest_feasible(new_dt, sep.prob.instance().item(c2_pi.item().idx()));
            sep.move_item(c2_pk, new_feasible_dt);
        }
    }
    true
}

/// Conservative disruption for mixed single items and rigid groups. It swaps
/// two placement-unit origins by translation only; rotations stay unchanged,
/// so the allowed-orientation intersection established by the caller remains
/// valid. The normal contained-item follow-up is intentionally omitted because
/// it reasons about a single polygon rather than a rigid unit envelope.
fn disrupt_place_units(sep: &mut Separator) -> bool {
    let candidates = sep
        .place_groups
        .placed_unit_representatives(&sep.prob.layout())
        .into_iter()
        .filter(|&item_id| !sep.frozen_mask.get(item_id).copied().unwrap_or(false))
        .collect_vec();
    if candidates.len() < 2 {
        warn!("[DSRP] cannot disrupt fewer than two movable placement units");
        return false;
    }

    let Some(&item1) = candidates.iter().choose(&mut sep.rng) else {
        return false;
    };
    let Some(&item2) = candidates
        .iter()
        .filter(|&&id| id != item1)
        .choose(&mut sep.rng)
    else {
        return false;
    };
    let Some(pk1) = sep.placement_key(item1) else {
        return false;
    };
    let Some(pk2) = sep.placement_key(item2) else {
        return false;
    };
    let dt1 = sep.prob.layout().placed_items()[pk1].d_transf();
    let dt2 = sep.prob.layout().placed_items()[pk2].d_transf();
    let target1 = jagua_rs::geometry::DTransformation::new(dt1.rotation(), dt2.translation());
    let target2 = jagua_rs::geometry::DTransformation::new(dt2.rotation(), dt1.translation());

    sep.move_item(pk1, target1);
    // `pk2` belongs to a different unit and remains valid after the first move.
    sep.move_item(pk2, target2);
    true
}

/// Collects all items which point of inaccessibility (POI) is contained by pk_c's shape.
fn practically_contained_items(layout: &Layout, pk_c: PItemKey) -> Vec<PItemKey> {
    let pi_c = &layout.placed_items()[pk_c];
    // Detect all collisions with the item pk_c's shape.
    let mut collector = BasicHazardCollector::new();
    layout
        .cde()
        .collect_poly_collisions(&pi_c.shape(), &NoFilter, &mut collector);

    // Filter out the items that have their POI contained by pk_c's shape.
    collector
        .iter()
        .filter_map(|(_, he)| match he {
            BasicHazardEntity::PlacedItem { pk, .. } => Some(*pk),
            _ => None,
        })
        .filter(|pk| *pk != pk_c) // Ensure we don't include the item itself
        .filter(|pk| {
            // Check if the POI of the item is contained by pk_c's shape
            let poi = layout.placed_items()[*pk].shape().poi();
            pi_c.shape().collides_with(&poi.center)
        })
        .collect_vec()
}
