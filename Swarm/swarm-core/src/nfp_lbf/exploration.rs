//! 순수 NFP 순서 탐색 단계 — 탐색(exploration) 단계의 네 번째 선택지.
//!
//! ```text
//! ordering 하나  ->  NFP 순차 배치(결정론적)  ->  strip 길이
//!       ^                                            |
//!       +---- 이웃 연산(swap / move / block reorder) --+
//!             + 수용 기준(simulated annealing)
//!             + 주기적 LP 압축으로 미세 조정
//! ```
//!
//! # 기존 탐색과 무엇이 다른가
//!
//! 기존 Swarm 탐색은 **겹침을 허용한 연속 완화**다. 폭을 줄여 일부러 충돌을 만들고,
//! 그 충돌을 분리(separate)해 가며 내려간다. 탐색 공간은 "각 조각의 위치"라는 연속
//! 공간이고, 위치 후보는 난수 샘플링으로 뽑는다.
//!
//! 이 단계는 정반대다.
//!
//! * **겹침을 한 번도 만들지 않는다.** 모든 중간 상태가 실행 가능한 배치다.
//! * **탐색 공간이 위치가 아니라 배치 순서다.** 순서 하나를 정하면 NFP 순차 배치가
//!   결정론적으로 위치를 정하므로, 순서 -> strip 길이는 순수 함수다.
//! * 따라서 이웃 연산은 좌표 섭동이 아니라 **순열 연산**(swap / move / block reorder)이다.
//!
//! 이 교환에는 대가가 있다. 연속 완화가 짜내는 마지막 몇 %는 "조금 겹쳤다가 풀면서
//! 자리를 맞바꾸는" 움직임에서 나오는데, 순서 공간에는 그 움직임이 없다. 그래서
//! 이 단계는 **기존 탐색보다 나쁠 수도 있고**, 그것이 기본 꺼짐인 이유다
//! ([`crate::config::NfpIntegrationConfig::ordering_exploration`]).
//!
//! # LP 압축을 왜 섞는가
//!
//! 순서 탐색이 도달할 수 없는 배치가 있다. 순차 배치는 조각을 놓는 순간
//! "그 시점에서 가장 좋은 자리"에 확정하고 다시 건드리지 않으므로, 전부 놓은 뒤에야
//! 보이는 몇 mm짜리 여유는 회수하지 못한다. [`crate::initial::compact`]의 LP 압축은
//! 순서를 바꾸지 않고 **좌표만** 동시에 당긴다 — 정확히 순서 탐색의 사각지대다.
//!
//! 다만 압축 결과는 순서로 되돌릴 수 없다(같은 순서를 다시 배치해도 그 좌표가 나오지
//! 않는다). 그래서 압축은 **해 수준 인컴번트에만** 적용하고, 순서 탐색 상태는 압축
//! 이전 배치를 그대로 이어간다.
//!
//! # 절대 규칙
//!
//! * 반환하는 모든 해는 [`crate::structure::pipeline::solution_validation_error`]를
//!   통과해야 한다. NFP가 "괜찮다"고 해도 jagua-rs CDE가 최종 판정자다.
//! * NFP 질의는 근사보다 비싸다. [`PreparedLibrary`]/[`NfpLbfCache`]와
//!   [`SequentialPlacer`]의 SoA 버퍼를 **한 번 만들어 끝까지 재사용**한다.
//!   `SPProblem`도 마찬가지로 하나만 만들어 `restore`로 되감는다.

use std::time::Duration;

use jagua_rs::Instant;
use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::geo_enums::RotationRange;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution};
use log::warn;
use rand::RngExt;
use rand::rngs::Xoshiro256PlusPlus;

use crate::config::{ExplorationConfig, NfpLbfSeedConfig};
use crate::initial::compact::{CompactCfg, compact_with_control};
use crate::nfp_lbf::cache::{NfpLbfCache, PreparedLibrary};
use crate::nfp_lbf::ordering::build_orderings;
use crate::nfp_lbf::placement::{SequentialPlacement, SequentialPlacer};
use crate::nfp_lbf::score::PlacementScoreWeights;
use crate::optimizer::separator::Separator;
use crate::util::listener::{ReportType, SolutionListener};
use crate::util::terminator::Terminator;

// ────────────────────────────────────────────────────────────────────────────
// 상수 — 근거를 함께 남긴다
// ────────────────────────────────────────────────────────────────────────────

/// 초기에 평가할 결정론적 ordering 수.
///
/// [`build_orderings`]는 앞쪽 6종(area / maxdim / hull / concavity / difficulty-tiered
/// / random-shuffle)을 결정론적으로 만들고 7번째부터 난수 파생으로 넘어간다. 파생은
/// 어차피 SA 루프가 직접 하므로 여기서는 앞 6종만 받는다.
const INITIAL_ORDERING_COUNT: usize = 6;

/// 시드 해를 만들 때 strip 폭에 얹는 상대 여유. `fit_strip()`이 곧바로 되돌린다.
/// 경계 위에 놓인 조각이 컨테이너 밖으로 판정되는 것만 막으면 된다.
const SEED_STRIP_RELATIVE_SLACK: f32 = 1.0e-3;

/// 시드 해를 만들 때 strip 폭에 얹는 절대 여유(mm).
const SEED_STRIP_ABSOLUTE_SLACK: f32 = 1.0e-3;

/// SA 초기 온도 = 기준 strip 길이 × 이 비율.
///
/// 온도의 단위는 목적함수와 같은 mm다. 초기에는 기준 길이의 2% 정도 악화까지
/// 어렵지 않게 받아들이고(= exp(-1) 근방), 끝에서는 사실상 hill climbing이 된다.
const SA_T0_RATIO: f32 = 0.02;

/// SA 종료 온도 = 기준 strip 길이 × 이 비율. `SA_T0_RATIO`의 1/100.
const SA_T_END_RATIO: f32 = 0.0002;

/// 이웃 연산이 건드릴 수 있는 꼬리 구간의 최소 비율.
///
/// 변형 지점이 뒤쪽일수록 공통 prefix가 길어져 재배치 비용이 싸다. 반대로 앞쪽을
/// 전혀 건드리지 않으면 큰 조각의 배치 순서를 영원히 바꾸지 못한다. 그래서 꼬리
/// 폭을 매번 다시 뽑되, 제곱 편향으로 대부분은 짧게(=싸게) 유지한다.
const MUTATION_SPAN_MIN: f32 = 0.05;

/// 이웃 연산이 건드릴 수 있는 꼬리 구간의 최대 비율. [`MUTATION_SPAN_MIN`] 참조.
const MUTATION_SPAN_MAX: f32 = 0.75;

/// LP 압축을 현재 SA 상태에도 걸어 보는 주기(반복 수).
///
/// 전역 최선이 갱신될 때는 항상 압축한다. 그와 별개로, 최선은 아니지만 압축 여지가
/// 큰 배치가 있을 수 있어 주기적으로 현재 상태에도 한 번 걸어 본다.
const LP_PROBE_PERIOD: usize = 64;

/// 한 번의 LP 압축에서 돌릴 반복 수. 탐색 루프 안이라 짧게 잡는다.
const LP_MAX_ITERS: u32 = 6;

/// LP 한 번의 시간 상한(ms). `microlp` 호출 하나에 걸리는 상한이고, 압축 반복
/// 전체 비용은 이보다 크다(조각별 볼록 분해 + 제약 생성 + CDE 재검증).
const LP_TIME_LIMIT_MS: u64 = 100;

/// LP 압축이 먹어도 되는 잔여 시간의 역수. `1/이 값` 이상 걸릴 것 같으면 건너뛴다.
///
/// LP 한 번은 156조각 마커에서 초 단위가 나올 수 있어, 개선마다 무조건 돌리면
/// 순서 탐색이 아니라 압축이 예산을 전부 먹는다. 직전 실측 비용을 그대로 다음 번
/// 예측으로 쓰는 자기조정 규칙이다 — 인스턴스 크기를 미리 알 필요가 없다.
const LP_BUDGET_DIVISOR: u32 = 10;

/// [`CompactCfg::default`]의 절대값(mm)이 맞춰진 기준 strip 높이.
///
/// 기본값(`neighbor_slack` 40mm, `move_max` 15mm, `improve_tol` 0.05mm)은 실물 의류
/// 마커 척도에서 잡힌 값이다. 척도가 다른 인스턴스(표준 벤치마크는 높이가 수십~수백)에
/// 그대로 쓰면 이동 상한이 도형보다 커져 압축이 폭주하거나, 반대로 개선 판정 문턱이
/// 전체 폭보다 커져 1회 만에 멈춘다. 그래서 strip 높이에 비례해 다시 잡는다.
const COMPACT_REFERENCE_HEIGHT: f32 = 1_500.0;

/// 폭 개선으로 인정할 최소 차이(mm). 부동소수 잡음으로 "개선"이 무한히 보고되는 것을 막는다.
const WIDTH_IMPROVE_EPS: f32 = 1.0e-5;

// ────────────────────────────────────────────────────────────────────────────
// 진입점
// ────────────────────────────────────────────────────────────────────────────

/// 순수 NFP 순서 탐색 단계 — 탐색 단계의 네 번째 선택지.
///
/// [`crate::config::NfpIntegrationConfig::ordering_exploration`]가 켜졌을 때에만
/// 호출된다. 꺼져 있으면 이 함수는 아예 호출되지 않으므로 기존 경로는 한 비트도
/// 바뀌지 않는다 — A/B 측정의 전제다.
///
/// 반환은 다른 탐색 단계와 동일하게 **feasible 해 목록**이며 **마지막이 최선**이다.
/// 목록은 절대 비지 않는다 — 하나도 만들지 못하면 진입 시점의 `sep.prob.save()`를 넣는다.
///
/// 이 단계는 `sep`이 들고 있던 배치를 **대체**한다. 마지막에
/// [`Separator::rollback`]으로 컨테이너와 최선 배치를 함께 복원해
/// 다음 단계(compression)가 그대로 이어받게 한다.
///
/// # 주의
///
/// 순서 공간 탐색은 연속 완화가 짜내는 마지막 몇 %를 잃을 수 있다. 즉 **기존 탐색보다
/// 나쁜 결과가 나올 수 있고 그것이 정상이다.** 상위 호출자
/// ([`crate::optimizer::optimize`])는 `retain_exact_no_worse`로 인컴번트를 보호하므로
/// 전체 실행이 퇴보하지는 않는다.
pub fn nfp_ordering_exploration_phase(
    instance: &SPInstance,
    sep: &mut Separator,
    sol_listener: &mut Option<&mut dyn SolutionListener>,
    term: &impl Terminator,
    expl_config: &ExplorationConfig,
    nfp_cfg: &NfpLbfSeedConfig,
) -> Vec<SPSolution> {
    let _run_scope = crate::config::runtime::enter_if_unset(sep.run_control.clone());
    let started = Instant::now();
    let local_deadline = started + expl_config.time_limit;
    let deadline = term.timeout_at().map_or(local_deadline, |owner| owner.min(local_deadline));
    let should_stop = || term.kill() || Instant::now() >= deadline
        || crate::config::runtime::init_cancel_requested();
    // The entry is the caller's certified incumbent. Stop does not authorize
    // reconstruction, ordering RNG, or a fresh feasibility callback.
    let entry_solution = sep.prob.save();
    if should_stop() { return vec![entry_solution]; }

    // Sequence reconstruction does not model fixed obstacles or rigid units.
    // Keep optimizing through the constraint-aware separator instead of dropping
    // those contracts when installing an otherwise collision-free new layout.
    if sep.has_place_groups() || sep.frozen_mask.iter().any(|&frozen| frozen) {
        #[cfg(feature = "adaptive")]
        return crate::optimizer::explore_mods::exploration_phase_adaptive(
            instance,
            sep,
            sol_listener,
            term,
            expl_config,
        );
        #[cfg(not(feature = "adaptive"))]
        return crate::optimizer::explore::exploration_phase(
            instance,
            sep,
            sol_listener,
            term,
            expl_config,
        );
    }
    let entry_width = entry_solution.strip_width();

    let mut out: Vec<SPSolution> = Vec::new();

    // ── 1) 도형 라이브러리 + NFP 콜드 캐시 ────────────────────────────────
    let rotations: Vec<i64> = if nfp_cfg.allow_180 {
        vec![0, 180]
    } else {
        vec![0]
    };
    let Some(library) =
        PreparedLibrary::build_with_control(instance, &rotations, nfp_cfg.concave_simplify_tolerance, &should_stop)
    else {
        warn!("[NFP-EXPL] prepared library build failed; falling back to the entry solution");
        return vec![entry_solution];
    };

    let mut cache = NfpLbfCache::new(library);
    // prewarm 예산은 이 단계의 남은 시간을 넘을 수 없다. 넘기면 SA가 아예 못 돈다.
    let prewarm_budget = Duration::from_millis(nfp_cfg.cache_budget_ms).min(remaining(deadline));
    cache.prewarm_with_control(prewarm_budget, &should_stop);
    if should_stop() { return vec![entry_solution]; }

    let strip_height = instance.base_strip.fixed_height;
    if !strip_height.is_finite() || strip_height <= 0.0 {
        warn!("[NFP-EXPL] invalid strip height {strip_height}; falling back to the entry solution");
        return vec![entry_solution];
    }
    let strip_width_cap = worst_case_strip_width(instance, strip_height);

    // 미배치 1개에 물리는 벌점(mm). 1열 최악 폭이라 사실상 사전식 비교
    // (미배치 수 우선, 동수면 strip 길이)가 된다. SA 수용 확률을 쓰려면 스칼라가
    // 필요해서 사전식 비교 대신 이 형태를 쓴다.
    let unplaced_penalty = strip_width_cap.max(strip_height);

    // ── 2) 초기 ordering — 결정론적 전략들 ────────────────────────────────
    if should_stop() { return vec![entry_solution]; }
    let orderings = build_orderings(instance, INITIAL_ORDERING_COUNT, &mut sep.rng, None);
    if orderings.is_empty() {
        warn!("[NFP-EXPL] no item ordering produced; falling back to the entry solution");
        return vec![entry_solution];
    }

    // SoA 버퍼를 들고 있는 배치기. **한 번만** 만들어 끝까지 재사용한다.
    let item_rotations = item_allowed_rotations(instance, &rotations);
    let mut placer = SequentialPlacer::with_item_rotations(
        &mut cache,
        strip_height,
        strip_width_cap,
        PlacementScoreWeights::default(),
        item_rotations,
    );

    // `SPProblem`도 하나만 만들어 `restore`로 되감는다. 아이템마다 새로 만들면
    // CDE 재구축 비용이 그대로 반복된다.
    // Finalizer is created only after a complete, uninterrupted ordering exists.

    // ── 3) 결정론적 ordering 평가 ─────────────────────────────────────────
    let mut cur_seq: Vec<usize> = Vec::new();
    let mut cur_placement = SequentialPlacement::default();
    let mut cur_cost = f32::INFINITY;
    let mut best_placement = SequentialPlacement::default();
    let mut best_cost = f32::INFINITY;
    let mut have_state = false;

    // 직전에 평가한 (순서, 배치) — prefix 재사용의 원본.
    let mut prev: Option<(Vec<usize>, SequentialPlacement)> = None;

    for ordering in orderings.iter() {
        if should_stop() {
            break;
        }
        let (prefix, prefix_len) = prefix_view(
            nfp_cfg.enable_prefix_reuse,
            prev.as_ref(),
            &ordering.sequence,
        );
        let placement =
            placer.place_sequence_with_control(&ordering.sequence, prefix, prefix_len, Some(deadline), &should_stop);
        if should_stop() { break; }
        let cost = placement_cost(&placement, unplaced_penalty);

        if cost < best_cost {
            best_cost = cost;
            best_placement = placement.clone();
        }
        if cost < cur_cost {
            cur_cost = cost;
            cur_seq.clear();
            cur_seq.extend_from_slice(&ordering.sequence);
            cur_placement = placement.clone();
        }
        have_state = true;
        prev = Some((ordering.sequence.clone(), placement));
    }
    drop(prev);

    if should_stop() || !have_state || !best_cost.is_finite() {
        warn!("[NFP-EXPL] no ordering could be evaluated; falling back to the entry solution");
        return vec![entry_solution];
    }

    // 최선 배치를 곧바로 해로 굳혀 둔다. 여기서 실패하면 이 단계는 아무것도 못 돌려준다.
    let mut finalizer = Finalizer::new(instance, strip_height);
    let mut best_width = f32::INFINITY;
    if let Some(sol) = finalizer.finalize(&best_placement, deadline, &should_stop)
        && !should_stop()
    {
        best_width = sol.strip_width();
        report_and_push(&mut out, sol, instance, sol_listener);
    }

    // ── 4) SA 순서 탐색 ───────────────────────────────────────────────────
    let reference_len = best_placement.strip_length.max(strip_height);
    let t0 = (SA_T0_RATIO * reference_len).max(f32::MIN_POSITIVE);
    let t_end = (SA_T_END_RATIO * reference_len).max(f32::MIN_POSITIVE);
    // 냉각 스케줄은 실제 마감(호출자 타임아웃과 time_limit 중 이른 쪽) 기준이어야
    // `progress` 가 1.0 에 도달한다 — time_limit 만 보면 외부 마감이 짧을 때
    // 고온 구간에서 끝나 버린다.
    let total_secs = deadline
        .saturating_duration_since(started)
        .as_secs_f32()
        .max(f32::MIN_POSITIVE);

    let mut cand_seq: Vec<usize> = Vec::with_capacity(cur_seq.len());
    let mut block_scratch: Vec<usize> = Vec::new();
    let mut n_iters = 0usize;

    // Equal IDs cannot produce a different ordering either (demand > 1).
    let can_mutate = cur_seq.windows(2).any(|pair| pair[0] != pair[1]);
    while can_mutate && !should_stop() {
        n_iters += 1;

        // a) 이웃 연산 — 현재 상태에서 파생한다(최선이 아니라 현재다. 그래야 SA가
        //    받아들인 악화 상태에서 계속 걸어 나갈 수 있다).
        cand_seq.clear();
        cand_seq.extend_from_slice(&cur_seq);
        let lo = mutation_lower_bound(cand_seq.len(), &mut sep.rng);
        let op = match n_iters % 3 {
            0 => op_swap(&mut cand_seq, lo, &mut sep.rng),
            1 => op_move(&mut cand_seq, lo, &mut sep.rng),
            _ => op_block(&mut cand_seq, lo, &mut block_scratch, &mut sep.rng),
        };
        if op.is_none() {
            // 이 길이에서는 성립하지 않는 연산이다. 다음 연산으로 넘어간다.
            continue;
        }

        // b) 공통 prefix 재사용 — suffix만 재배치한다.
        let requested = if nfp_cfg.enable_prefix_reuse {
            common_prefix_len(&cur_seq, &cand_seq)
        } else {
            0
        };
        let prefix_len = usable_prefix_len(&cand_seq, &cur_placement, requested);
        let prefix = if prefix_len > 0 {
            Some(&cur_placement)
        } else {
            None
        };
        let placement = placer.place_sequence_with_control(&cand_seq, prefix, prefix_len, Some(deadline), &should_stop);

        // 예산이 배치 도중에 끊겼다면 그 결과는 잘린 배치다 — 채택하면 안 된다.
        if should_stop() {
            break;
        }

        let cost = placement_cost(&placement, unplaced_penalty);

        // c) 수용 판정 — 줄면 채택, 늘면 SA 확률.
        let delta = cost - cur_cost;
        let progress = (started.elapsed().as_secs_f32() / total_secs).clamp(0.0, 1.0);
        let temp = temperature(t0, t_end, progress);
        let accept = if delta <= 0.0 {
            true
        } else {
            let p = (-delta / temp).exp();
            let roll: f32 = sep.rng.random::<f32>();
            roll < p
        };

        if accept {
            cur_cost = cost;
            cur_seq.clear();
            cur_seq.extend_from_slice(&cand_seq);
            cur_placement = placement;
        } else {
            // 기각한 배치는 버린다. 다음 반복은 다시 `cur_placement`에서 파생한다.
            drop(placement);
        }

        // d) 전역 최선 갱신 -> 해로 굳혀 보고
        let improved_ordering = cur_cost < best_cost - f32::EPSILON;
        if improved_ordering {
            best_cost = cur_cost;
            best_placement = cur_placement.clone();

            if let Some(sol) = finalizer.finalize(&best_placement, deadline, &should_stop)
                && !should_stop()
                && sol.strip_width() < best_width - WIDTH_IMPROVE_EPS
            {
                best_width = sol.strip_width();
                report_and_push(&mut out, sol, instance, sol_listener);
            }
        }

        // e) 주기적 LP 압축 — 순서 탐색이 못 하는 미세 위치 조정.
        //
        //   최선이 아닌 현재 상태에도 걸어 본다. 순서로는 조금 나쁜 배치가 압축 뒤에
        //   더 짧아지는 경우가 있고, 그건 순서 공간에서 절대 보이지 않는다.
        //   압축 결과는 순서로 되돌릴 수 없으므로 탐색 상태는 건드리지 않는다.
        if !improved_ordering && n_iters % LP_PROBE_PERIOD == 0 {
            if let Some(sol) = finalizer.finalize(&cur_placement, deadline, &should_stop)
                && !should_stop()
                && sol.strip_width() < best_width - WIDTH_IMPROVE_EPS
            {
                best_width = sol.strip_width();
                report_and_push(&mut out, sol, instance, sol_listener);
            }
        }
    }

    drop(placer);

    // ── 5) 최소 1개 보장 ──────────────────────────────────────────────────
    if out.is_empty() {
        warn!(
            "[NFP-EXPL] produced no valid solution; returning the entry solution (width {:.3})",
            entry_width
        );
        out.push(entry_solution);
    }

    // ── 6) separator를 최선 해로 맞춰 다음 단계에 넘긴다 ──────────────────
    //
    // 이 단계는 `sep`이 들고 있던 배치를 대체한다. 폭을 먼저 맞추지 않으면
    // `rollback`의 debug_assert(폭 일치)에 걸린다.
    if let Some(best) = out.last() {
        sep.rollback(best, None);
    }

    out
}

// ────────────────────────────────────────────────────────────────────────────
// 해 확정 — materialize -> 검증 -> LP 압축 -> 재검증
// ────────────────────────────────────────────────────────────────────────────

/// 순차 배치 결과를 검증된 [`SPSolution`]으로 굳히는 재사용 가능한 작업대.
///
/// `SPProblem`을 하나만 들고 `restore(empty)`로 되감는다. 개선마다 새로 만들면
/// `Layout::from_snapshot`의 CDE 재구축이 그대로 반복된다.
struct Finalizer<'a> {
    instance: &'a SPInstance,
    prob: SPProblem,
    /// 아이템이 하나도 없는 초기 스냅샷. 매번 여기로 되감는다.
    empty: SPSolution,
    /// 남은 수요 카운터. 매번 새로 할당하지 않고 덮어쓴다.
    remaining: Vec<usize>,
    compact_cfg: CompactCfg,
    /// 직전 LP 압축 1회의 실측 비용. 다음 번 예산 판단에 그대로 쓴다.
    /// [`LP_BUDGET_DIVISOR`] 참조.
    last_lp_cost: Option<Duration>,
}

impl<'a> Finalizer<'a> {
    fn new(instance: &'a SPInstance, strip_height: f32) -> Self {
        let prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
        let empty = prob.save();
        let remaining = vec![0usize; instance.items.len()];
        Self {
            instance,
            prob,
            empty,
            remaining,
            compact_cfg: scaled_compact_cfg(strip_height),
            last_lp_cost: None,
        }
    }

    /// 이번에 LP 압축을 돌려도 되는가.
    ///
    /// 첫 번째는 비용을 재기 위해 반드시 한 번 돌린다(잔여 시간이 남아 있는 한).
    /// 그 뒤로는 직전 실측 비용이 잔여 시간의 `1/LP_BUDGET_DIVISOR` 안에 들어올 때만 돈다.
    fn lp_affordable(&self, deadline: Instant) -> bool {
        let left = remaining(deadline);
        if left.is_zero() {
            return false;
        }
        match self.last_lp_cost {
            None => true,
            Some(cost) => cost * LP_BUDGET_DIVISOR <= left,
        }
    }

    /// 배치 -> 검증된 해. 실패하면 `None`.
    ///
    /// LP 압축은 **검증을 통과한 해에만** 적용하고, 압축 결과도 다시 검증한다.
    /// 압축이 실패하거나 폭이 줄지 않으면 압축 이전 해를 그대로 쓴다.
    fn finalize(
        &mut self,
        placement: &SequentialPlacement,
        deadline: Instant,
        should_stop: &(impl Fn() -> bool + Sync),
    ) -> Option<SPSolution> {
        if should_stop() { return None; }
        let base = self.materialize(placement, should_stop)?;
        if should_stop() { return None; }
        if let Some(err) =
            crate::structure::pipeline::solution_validation_error(self.instance, &base)
        {
            warn!("[NFP-EXPL] rejected an invalid placement: {err:?}");
            return None;
        }

        // Exact certification is an atomic scan here (Init-aware only).
        // Stop during that scan invalidates this attempt for publication.
        if should_stop() { return None; }
        // LP 압축 — 순서를 바꾸지 않고 좌표만 동시에 당긴다.
        // 남은 예산이 감당 못 할 것 같으면 압축 없이 그대로 돌려준다.
        if !self.lp_affordable(deadline) {
            return Some(base);
        }
        let lp_started = Instant::now();
        let (compacted, _stats) = compact_with_control(
            self.instance, &base, &self.compact_cfg, LP_MAX_ITERS, Some(deadline), should_stop,
        );
        self.last_lp_cost = Some(lp_started.elapsed());
        if should_stop() { return None; }
        if compacted.strip_width() < base.strip_width() - WIDTH_IMPROVE_EPS {
            // 압축은 자체적으로 CDE feasibility를 확인하지만, 최종 판정자는 언제나
            // 전체 검증이다. 통과하지 못하면 압축 이전 해로 되돌린다.
            match crate::structure::pipeline::solution_validation_error(self.instance, &compacted) {
                None if !should_stop() => return Some(compacted),
                None => return None,
                Some(err) => {
                    warn!("[NFP-EXPL] LP compaction produced an invalid solution: {err:?}")
                }
            }
        }
        Some(base)
    }

    /// [`SequentialPlacement`] -> [`SPSolution`]. 검증은 하지 않는다.
    fn materialize(&mut self, placement: &SequentialPlacement,
        should_stop: &(impl Fn() -> bool + Sync),
    ) -> Option<SPSolution> {
        if should_stop() { return None; }
        if placement.pieces.is_empty() {
            return None;
        }
        // 미배치가 남은 배치는 어차피 검증에서 거부된다 — 레이아웃을 통째로 다시
        // 만들고 경고까지 찍는 비용을 아낀다.
        if !placement.unplaced.is_empty() {
            return None;
        }
        if !placement.strip_length.is_finite() || placement.strip_length <= 0.0 {
            warn!(
                "[NFP-EXPL] invalid strip length {}; placement skipped",
                placement.strip_length
            );
            return None;
        }

        self.prob.restore(&self.empty);
        // `search::materialize` 와 같은 이유로 오른쪽 물리 여백을 미리 확보한다.
        let right_inset = self
            .instance
            .base_strip
            .shape_modify_config
            .offset
            .unwrap_or(0.0)
            .max(0.0);
        self.prob
            .change_strip_width(seed_strip_width(placement.strip_length + right_inset)).expect(crate::dmsg!("strip width must be positive"));

        // `place_item`은 수요 카운터를 감소시킨다. 수요를 넘겨 넣으면 underflow로
        // panic하므로 남은 수요를 직접 센다.
        for (slot, (_, qty)) in self.remaining.iter_mut().zip(self.instance.items.iter()) {
            *slot = *qty;
        }

        for piece in placement.pieces.iter() {
            if should_stop() { return None; }
            let Some(left) = self.remaining.get_mut(piece.item_id) else {
                warn!(
                    "[NFP-EXPL] unknown item id {}; piece skipped",
                    piece.item_id
                );
                continue;
            };
            if *left == 0 {
                continue;
            }
            let (x, y) = piece.translation;
            if !x.is_finite() || !y.is_finite() {
                warn!(
                    "[NFP-EXPL] item {} has a non-finite translation; piece skipped",
                    piece.item_id
                );
                continue;
            }
            *left -= 1;
            self.prob.place_item(SPPlacement {
                d_transf: DTransformation::new(rotation_to_radians(piece.rotation), (x, y)),
                item_idx: piece.item_id,
            });
        }

        if self.prob.n_placed_items() == 0 {
            return None;
        }
        if should_stop() { return None; }
        self.prob.fit_strip().expect(crate::dmsg!("strip width must be positive"));
        if should_stop() { return None; }
        Some(self.prob.save())
    }
}

/// strip 높이에 맞춰 다시 잡은 LP 압축 설정. [`COMPACT_REFERENCE_HEIGHT`] 참조.
fn scaled_compact_cfg(strip_height: f32) -> CompactCfg {
    let base = CompactCfg::default();
    let scale = if strip_height.is_finite() && strip_height > 0.0 {
        (strip_height / COMPACT_REFERENCE_HEIGHT).clamp(1.0e-3, 1.0e3)
    } else {
        1.0
    };
    CompactCfg {
        max_iters: LP_MAX_ITERS,
        improve_tol: base.improve_tol * scale,
        move_tol: base.move_tol * scale,
        neighbor_slack: base.neighbor_slack * scale,
        move_max: base.move_max * scale,
        lp_time_limit_ms: LP_TIME_LIMIT_MS,
        // 탐색 루프 안에서는 최선 해 보관(rollback)과 에지 반평면을 모두 켠다.
        // 되돌림이 있어야 압축이 해를 나쁘게 만들 수 없다.
        edge_halfplanes: true,
        rollback: true,
        ..base
    }
}

/// 해를 목록에 넣고 [`ReportType::ExplFeas`]로 보고한다.
fn report_and_push(
    out: &mut Vec<SPSolution>,
    sol: SPSolution,
    instance: &SPInstance,
    sol_listener: &mut Option<&mut dyn SolutionListener>,
) {
    if let Some(l) = sol_listener.as_deref_mut() {
        l.report(ReportType::ExplFeas, &sol, instance);
    }
    out.push(sol);
}

// ────────────────────────────────────────────────────────────────────────────
// 목적함수 / SA
// ────────────────────────────────────────────────────────────────────────────

/// 순서 하나의 비용(mm). `strip 길이 + 미배치 수 × 벌점`.
///
/// 미배치 벌점은 1열 최악 폭이라 사실상 사전식(미배치 수 우선)이다. SA 수용 확률은
/// 스칼라 차이를 요구하므로 튜플 비교 대신 이 형태를 쓴다.
#[inline]
fn placement_cost(placement: &SequentialPlacement, unplaced_penalty: f32) -> f32 {
    if !placement.strip_length.is_finite() {
        return f32::MAX;
    }
    placement.strip_length + placement.unplaced.len() as f32 * unplaced_penalty
}

/// 기하 감쇠 온도. `progress`는 `[0, 1]`의 시간 진행률.
#[inline]
fn temperature(t0: f32, t_end: f32, progress: f32) -> f32 {
    let p = progress.clamp(0.0, 1.0);
    let t = t0 * (t_end / t0).powf(p);
    // 0 이나 NaN 이 되면 수용 확률 계산이 깨진다.
    if t.is_finite() && t > 0.0 { t } else { t_end }
}

// ────────────────────────────────────────────────────────────────────────────
// prefix 재사용
// ────────────────────────────────────────────────────────────────────────────

/// 두 나열이 앞에서부터 몇 개나 일치하는지.
#[inline]
fn common_prefix_len(a: &[usize], b: &[usize]) -> usize {
    a.iter().zip(b.iter()).take_while(|(x, y)| x == y).count()
}

/// 실제로 재사용해도 되는 prefix 길이.
///
/// [`SequentialPlacer::place_sequence`]의 prefix 재사용은 `placement.pieces[k]`와
/// `sequence[k]`의 `item_id`가 같을 때만 그 조각을 그대로 가져간다. 어긋나면 그
/// 아이템을 **미배치로 밀어 넣는다** — 재사용이 해를 망가뜨리는 방향이다.
///
/// 순차 배치는 제약 조각과 부유조각을 나눠 놓으므로 `pieces`의 순서가 `sequence`의
/// 순서와 다를 수 있다. 그래서 요청 길이를 그대로 믿지 않고, **실제로 1:1 일치하는
/// 구간까지만** 잘라서 넘긴다.
#[inline]
fn usable_prefix_len(
    sequence: &[usize],
    placement: &SequentialPlacement,
    requested: usize,
) -> usize {
    let cap = requested.min(placement.pieces.len()).min(sequence.len());
    let mut k = 0usize;
    while k < cap && placement.pieces[k].item_id == sequence[k] {
        k += 1;
    }
    k
}

/// 직전 평가 결과에서 재사용 가능한 prefix를 뽑는다.
#[inline]
fn prefix_view<'p>(
    enabled: bool,
    prev: Option<&'p (Vec<usize>, SequentialPlacement)>,
    sequence: &[usize],
) -> (Option<&'p SequentialPlacement>, usize) {
    let (Some((prev_seq, prev_placement)), true) = (prev, enabled) else {
        return (None, 0);
    };
    let requested = common_prefix_len(prev_seq, sequence);
    let usable = usable_prefix_len(sequence, prev_placement, requested);
    if usable > 0 {
        (Some(prev_placement), usable)
    } else {
        (None, 0)
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 이웃 연산 — swap 2 / move 1 / block reorder
// ────────────────────────────────────────────────────────────────────────────

/// 변형이 일어날 수 있는 최소 인덱스.
///
/// 꼬리 폭을 `MUTATION_SPAN_MIN..MUTATION_SPAN_MAX` 사이에서 **제곱 편향**으로 뽑는다.
/// 대부분은 짧은 꼬리(=긴 공통 prefix = 싼 재배치)를 고르고, 가끔 앞쪽까지 열어
/// 큰 조각의 배치 순서도 바꿀 수 있게 한다.
#[inline]
fn mutation_lower_bound(len: usize, rng: &mut Xoshiro256PlusPlus) -> usize {
    if len < 4 {
        return 0;
    }
    let f: f32 = rng.random::<f32>();
    let ratio = MUTATION_SPAN_MIN + (MUTATION_SPAN_MAX - MUTATION_SPAN_MIN) * f * f;
    let span = ((len as f32) * ratio).ceil() as usize;
    len.saturating_sub(span.max(2))
}

/// `Swap 2 items`. 반환값은 변경 시작 지점. 성립하지 않으면 `None`.
fn op_swap(seq: &mut [usize], lo: usize, rng: &mut Xoshiro256PlusPlus) -> Option<usize> {
    let len = seq.len();
    if lo + 1 >= len {
        return None;
    }
    let i = rng.random_range(lo..len);
    let mut j = rng.random_range(lo..len - 1);
    if j >= i {
        j += 1;
    }
    seq.swap(i, j);
    Some(i.min(j))
}

/// `Move 1 item`. 반환값은 변경 시작 지점.
fn op_move(seq: &mut Vec<usize>, lo: usize, rng: &mut Xoshiro256PlusPlus) -> Option<usize> {
    let len = seq.len();
    if lo + 1 >= len {
        return None;
    }
    let from = rng.random_range(lo..len);
    let mut to = rng.random_range(lo..len - 1);
    if to >= from {
        to += 1;
    }
    let v = seq.remove(from);
    let to = to.min(seq.len());
    seq.insert(to, v);
    Some(from.min(to))
}

/// `Block reorder` — 연속 블록을 통째로 옮긴다. 반환값은 변경 시작 지점.
///
/// `scratch`는 호출자가 들고 있는 재사용 버퍼다. 반복마다 새로 할당하지 않는다.
fn op_block(
    seq: &mut Vec<usize>,
    lo: usize,
    scratch: &mut Vec<usize>,
    rng: &mut Xoshiro256PlusPlus,
) -> Option<usize> {
    let len = seq.len();
    let max_block = (len.saturating_sub(lo)).min(len / 4);
    if max_block < 2 {
        return None;
    }
    let block_len = rng.random_range(2..=max_block);
    let max_start = len - block_len;
    if lo >= max_start {
        return None;
    }
    let start = rng.random_range(lo..=max_start);
    let mut dest = rng.random_range(lo..max_start);
    if dest >= start {
        dest += 1;
    }

    scratch.clear();
    scratch.extend(seq.drain(start..start + block_len));
    let dest = dest.min(seq.len());
    for (k, v) in scratch.iter().copied().enumerate() {
        seq.insert(dest + k, v);
    }
    Some(start.min(dest))
}

// ────────────────────────────────────────────────────────────────────────────
// 소소한 보조
// ────────────────────────────────────────────────────────────────────────────

/// 마감까지 남은 시간. 이미 지났으면 0.
#[inline]
fn remaining(deadline: Instant) -> Duration {
    let now = Instant::now();
    if now >= deadline {
        Duration::ZERO
    } else {
        deadline - now
    }
}

/// 시드 해를 만들 때 쓸 strip 폭. `fit_strip()`이 바로 줄여준다.
#[inline]
fn seed_strip_width(strip_length: f32) -> f32 {
    strip_length * (1.0 + SEED_STRIP_RELATIVE_SLACK) + SEED_STRIP_ABSOLUTE_SLACK
}

/// 도(i64) -> 라디안(f32). `[0, 2π)`로 정규화한다.
#[inline]
fn rotation_to_radians(degrees: i64) -> f32 {
    let normalized = degrees.rem_euclid(360);
    (normalized as f64).to_radians() as f32
}

/// 최악의 1열 배치 폭. `SequentialPlacer`의 후보 x 상한으로 쓴다.
fn worst_case_strip_width(instance: &SPInstance, strip_height: f32) -> f32 {
    let mut sum = 0.0f32;
    for (item, qty) in instance.items.iter() {
        let w = item.shape_cd().bbox().width();
        if w.is_finite() && w > 0.0 {
            sum += w * (*qty as f32);
        }
    }
    if sum.is_finite() && sum > 0.0 {
        sum
    } else {
        strip_height
    }
}

/// `item_id`별 허용 회전(도) 목록. 인스턴스 선언과 설정을 교집합한다.
///
/// 길이가 2 이상인 아이템이 **부유조각**(결·위치 제약 없음)이고, `SequentialPlacer`는
/// 그런 조각을 뒤로 미뤄 앞선 배치가 만든 공백을 채우게 한다.
fn item_allowed_rotations(instance: &SPInstance, rotations: &[i64]) -> Vec<Vec<i64>> {
    let lib: Vec<i64> = if rotations.is_empty() {
        vec![0]
    } else {
        rotations.to_vec()
    };
    let max_id = instance.items.iter().map(|(i, _)| i.idx()).max().unwrap_or(0);
    let mut out = vec![Vec::new(); max_id + 1];

    for (item, _) in instance.items.iter() {
        let allowed: Vec<i64> = match item.allowed_orientations().rotations() {
            RotationRange::None => vec![0],
            RotationRange::Continuous => lib.clone(),
            RotationRange::Discrete(set) => {
                let mut degs: Vec<i64> = set
                    .iter()
                    .map(|r| (r.to_degrees().round() as i64).rem_euclid(360))
                    .collect();
                degs.sort_unstable();
                degs.dedup();
                let inter: Vec<i64> = lib.iter().copied().filter(|d| degs.contains(d)).collect();
                if inter.is_empty() { vec![0] } else { inter }
            }
        };
        out[item.idx()] = allowed;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nfp_lbf::placement::PlacedPiece;
    use rand::SeedableRng;

    fn rng() -> Xoshiro256PlusPlus {
        Xoshiro256PlusPlus::seed_from_u64(0x5EED_1234)
    }

    fn placement(ids: &[usize]) -> SequentialPlacement {
        SequentialPlacement {
            pieces: ids
                .iter()
                .map(|&item_id| PlacedPiece {
                    item_id,
                    rotation: 0,
                    translation: (0.0, 0.0),
                })
                .collect(),
            strip_length: 100.0,
            placed: ids.len(),
            unplaced: Vec::new(),
        }
    }

    /// 이웃 연산은 배치 대상 멀티셋을 절대 바꾸지 않는다.
    /// 조각이 늘거나 줄면 그 순간 해가 무효가 된다.
    #[test]
    fn neighbourhood_ops_preserve_the_multiset() {
        let mut r = rng();
        let base: Vec<usize> = (0..40).map(|i| i % 7).collect();
        let mut sorted_base = base.clone();
        sorted_base.sort_unstable();
        let mut scratch = Vec::new();

        for _ in 0..500 {
            let mut seq = base.clone();
            let lo = mutation_lower_bound(seq.len(), &mut r);
            assert!(lo < seq.len());
            op_swap(&mut seq, lo, &mut r);
            op_move(&mut seq, lo, &mut r);
            op_block(&mut seq, lo, &mut scratch, &mut r);
            let mut sorted = seq.clone();
            sorted.sort_unstable();
            assert_eq!(sorted, sorted_base);
            assert_eq!(seq.len(), base.len());
        }
    }

    /// 연산이 보고한 변경 시작 지점은 실제 공통 prefix 길이 이하여야 한다.
    /// 이 관계가 깨지면 prefix 재사용이 바뀐 구간을 재사용해 버린다.
    #[test]
    fn reported_change_point_never_exceeds_the_common_prefix() {
        let mut r = rng();
        let base: Vec<usize> = (0..60).collect();
        let mut scratch = Vec::new();

        for i in 0..600 {
            let mut seq = base.clone();
            let lo = mutation_lower_bound(seq.len(), &mut r);
            let at = match i % 3 {
                0 => op_swap(&mut seq, lo, &mut r),
                1 => op_move(&mut seq, lo, &mut r),
                _ => op_block(&mut seq, lo, &mut scratch, &mut r),
            };
            let Some(at) = at else { continue };
            assert!(at <= common_prefix_len(&base, &seq), "op {}", i % 3);
        }
    }

    /// 너무 짧은 나열에서는 연산이 성립하지 않고, panic 대신 `None`이 나와야 한다.
    #[test]
    fn ops_reject_degenerate_lengths() {
        let mut r = rng();
        let mut scratch = Vec::new();
        assert!(op_swap(&mut [], 0, &mut r).is_none());
        assert!(op_swap(&mut [1], 0, &mut r).is_none());
        assert!(op_move(&mut vec![1], 0, &mut r).is_none());
        // block_len >= 2 이려면 len/4 >= 2, 즉 len >= 8이 필요하다.
        assert!(op_block(&mut vec![1, 2, 3, 4], 0, &mut scratch, &mut r).is_none());
        assert!(op_block(&mut (0..16).collect::<Vec<_>>(), 0, &mut scratch, &mut r).is_some());
    }

    /// prefix 재사용은 `pieces`와 `sequence`가 실제로 1:1 일치하는 구간까지만 열린다.
    #[test]
    fn usable_prefix_stops_at_the_first_mismatch() {
        let p = placement(&[5, 3, 9, 1]);
        assert_eq!(usable_prefix_len(&[5, 3, 9, 1], &p, 4), 4);
        // 세 번째에서 어긋나면 2까지만.
        assert_eq!(usable_prefix_len(&[5, 3, 7, 1], &p, 4), 2);
        // 요청 길이가 상한이다.
        assert_eq!(usable_prefix_len(&[5, 3, 9, 1], &p, 2), 2);
        // 배치된 조각 수보다 길게 요청할 수 없다.
        assert_eq!(usable_prefix_len(&[5, 3, 9, 1, 8], &p, 5), 4);
        // 처음부터 다르면 재사용 없음.
        assert_eq!(usable_prefix_len(&[4, 3, 9, 1], &p, 4), 0);
    }

    /// 배치 순서가 뒤섞인 결과(부유조각 후치 배치)에는 재사용이 열리면 안 된다.
    #[test]
    fn reordered_pieces_close_the_prefix_window() {
        // 순차 배치가 부유조각 9를 뒤로 미뤄 pieces 순서가 sequence와 달라진 경우.
        let p = placement(&[5, 1, 9]);
        let seq = [5, 9, 1];
        assert_eq!(usable_prefix_len(&seq, &p, 3), 1);
    }

    /// 미배치는 strip 길이보다 언제나 무겁다.
    ///
    /// 벌점은 실제로 1열 최악 폭(`worst_case_strip_width`)이라 어떤 배치의 strip
    /// 길이보다도 크다. 그 전제에서 미배치 1개는 사전식 비교와 동일하게 동작한다.
    #[test]
    fn unplaced_dominates_strip_length() {
        // 1열 최악 폭 = 도달 가능한 어떤 strip 길이보다 크다.
        let penalty = 20_000.0;
        let mut a = placement(&[0, 1]);
        a.strip_length = 10_000.0;
        let mut b = placement(&[0, 1]);
        b.strip_length = 1.0;
        b.unplaced.push(3);
        assert!(placement_cost(&a, penalty) < placement_cost(&b, penalty));

        // 미배치 수가 같으면 strip 길이로 갈린다.
        let mut c = placement(&[0, 1]);
        c.strip_length = 9_000.0;
        c.unplaced.push(3);
        assert!(placement_cost(&c, penalty) < placement_cost(&a, penalty) + penalty);
        assert!(placement_cost(&b, penalty) < placement_cost(&c, penalty));

        // 비유한 strip 길이는 최악값으로 취급한다(NaN 이 비교를 오염시키지 않게).
        let mut nan = placement(&[0, 1]);
        nan.strip_length = f32::NAN;
        assert_eq!(placement_cost(&nan, penalty), f32::MAX);
    }

    /// 온도는 단조 감소하고 언제나 양수여야 한다(수용 확률의 분모).
    #[test]
    fn temperature_decays_monotonically_and_stays_positive() {
        let (t0, t_end) = (20.0f32, 0.2f32);
        let mut prev = f32::INFINITY;
        for k in 0..=10 {
            let t = temperature(t0, t_end, k as f32 / 10.0);
            assert!(t > 0.0 && t.is_finite());
            assert!(t <= prev + 1.0e-6);
            prev = t;
        }
        assert!((temperature(t0, t_end, 0.0) - t0).abs() < 1.0e-4);
        assert!((temperature(t0, t_end, 1.0) - t_end).abs() < 1.0e-4);
        // 범위 밖 진행률도 clamp된다.
        assert!(temperature(t0, t_end, -1.0).is_finite());
        assert!(temperature(t0, t_end, 7.0).is_finite());
    }

    /// LP 압축 설정은 strip 높이에 비례해야 한다 — 절대 mm 값을 그대로 쓰면
    /// 척도가 다른 인스턴스에서 이동 상한이 도형보다 커진다.
    #[test]
    fn compact_cfg_scales_with_strip_height() {
        let big = scaled_compact_cfg(COMPACT_REFERENCE_HEIGHT);
        let small = scaled_compact_cfg(COMPACT_REFERENCE_HEIGHT / 100.0);
        assert!(small.move_max < big.move_max);
        assert!(small.neighbor_slack < big.neighbor_slack);
        assert!((big.move_max - CompactCfg::default().move_max).abs() < 1.0e-3);
        // 비정상 높이에서도 유한한 값이 나와야 한다.
        for h in [0.0f32, -1.0, f32::NAN, f32::INFINITY] {
            let c = scaled_compact_cfg(h);
            assert!(c.move_max.is_finite() && c.move_max > 0.0);
        }
    }

    /// 시드 폭 여유는 `fit_strip`이 곧바로 회수할 만큼만 얹는다.
    #[test]
    fn seed_strip_width_leaves_a_small_slack() {
        let w = seed_strip_width(1_000.0);
        assert!(w > 1_000.0 && w < 1_002.0);
    }

    /// 회전각은 항상 `[0, 2π)`로 정규화된다.
    #[test]
    fn rotations_normalize_into_one_turn() {
        for deg in [-720, -181, -1, 0, 1, 179, 181, 359, 361, 1_080] {
            let r = rotation_to_radians(deg);
            assert!(r >= 0.0 && r < std::f32::consts::TAU, "deg = {deg}");
        }
    }

    // These tests call the real NFP phase. Their literal contracts catch forced
    // prestop work, ignored absolute deadlines and mutation-free SA spinning.
    struct Issue6Term {
        polls: std::sync::atomic::AtomicUsize,
        stop_at: usize,
        deadline: Option<Instant>,
    }
    impl Issue6Term {
        fn new(stop_at: usize, deadline: Option<Instant>) -> Self {
            Self { polls: std::sync::atomic::AtomicUsize::new(0), stop_at, deadline }
        }
    }
    impl Terminator for Issue6Term {
        fn kill(&self) -> bool {
            self.polls.fetch_add(1, std::sync::atomic::Ordering::Relaxed) >= self.stop_at
        }
        fn stop_requested(&self) -> bool { self.polls.load(std::sync::atomic::Ordering::Relaxed) > self.stop_at }
        fn new_timeout(&mut self, _: Duration) { panic!("{}", crate::dmsg!("phase must not reset owner deadline")) }
        fn timeout_at(&self) -> Option<Instant> { self.deadline }
    }
    struct Issue6Reports(usize);
    impl SolutionListener for Issue6Reports {
        fn report(&mut self, _: ReportType, _: &SPSolution, _: &SPInstance) { self.0 += 1; }
    }
    fn issue6_fixture(count: usize) -> (SPInstance, Separator) {
        use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
        let items: Vec<_> = (0..count).map(|id| serde_json::json!({
            "id":id,"demand":1,"allowed_orientations":[0.0],
            "shape":{"type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":8.0 + id as f32,"height":8.0}}
        })).collect();
        let ext: ExtSPInstance=serde_json::from_value(serde_json::json!({"name":"issue6", "strip_height":30.0,"items":items})).unwrap();
        let cfg=crate::config::DEFAULT_SWARM_CONFIG;
        let importer=crate::util::io::new_importer(cfg.cde_config, cfg.poly_simpl_tolerance, cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
        let instance=jagua_rs::probs::spp::io::import_instance(&importer,&ext).unwrap();
        let mut prob=SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
        prob.change_strip_width(30.0+count as f32*12.0).expect(crate::dmsg!("strip width must be positive"));
        for id in 0..count { prob.place_item(SPPlacement{item_idx:id,d_transf:DTransformation::new(0.0,(20.0+id as f32*12.0,10.0))}); }
        prob.fit_strip().expect(crate::dmsg!("strip width must be positive"));
        let mut sep_cfg=cfg.expl_cfg.separator_config;
        sep_cfg.n_workers=1;
        let sep=Separator::new(instance.clone(),prob,rng(),sep_cfg);
        assert!(crate::structure::pipeline::solution_validation_error(&instance,&sep.prob.save()).is_none());
        (instance,sep)
    }
    fn issue6_poses(sol: &SPSolution) -> Vec<(usize,u32,u32,u32)> {
        let mut v: Vec<_>=sol.layout_snapshot().placed_items().values().map(|p|(p.item().idx(),p.d_transf().rotation().to_bits(),p.d_transf().translation().0.to_bits(),p.d_transf().translation().1.to_bits())).collect();
        v.sort_unstable(); v
    }
    fn issue6_run_stopped(term: Issue6Term, duration: Duration) {
        let (instance,mut sep)=issue6_fixture(2);
        let entry=sep.prob.save();
        let mut expected_rng=sep.rng.clone();
        let mut reports=Issue6Reports(0);
        let mut cfg=crate::config::DEFAULT_SWARM_CONFIG.expl_cfg; cfg.time_limit=duration;
        let mut nfp=crate::config::DEFAULT_NFP_LBF_SEED_CONFIG; nfp.cache_budget_ms=0;
        let out=nfp_ordering_exploration_phase(&instance,&mut sep,&mut Some(&mut reports),&term,&cfg,&nfp);
        assert_eq!(reports.0,0,"stopped phase must not materialize/report a new candidate");
        assert_eq!(sep.rng.random::<u64>(),expected_rng.random::<u64>(),"stopped phase consumed ordering RNG");
        assert_eq!(issue6_poses(&out[0]),issue6_poses(&entry));
        assert_eq!(issue6_poses(&sep.prob.save()),issue6_poses(&entry));
        assert_eq!(out[0].strip_width(),entry.strip_width());
    }
    #[test]
    fn issue6_nfp_prestop_preserves_geometry_rng_and_reports() {
        issue6_run_stopped(Issue6Term::new(0,None),Duration::from_secs(1));
    }
    #[test]
    fn issue6_nfp_absolute_owner_deadline_prevents_forced_first_order() {
        issue6_run_stopped(Issue6Term::new(usize::MAX,Some(Instant::now()-Duration::from_secs(1))),Duration::from_secs(1));
    }
    #[test]
    fn issue6_nfp_zero_budget_preserves_geometry_rng_and_reports() {
        issue6_run_stopped(Issue6Term::new(usize::MAX,None),Duration::ZERO);
    }
    #[test]
    fn issue6_nfp_single_item_exits_without_impossible_mutation_loop() {
        let (instance,mut sep)=issue6_fixture(1);
        let term=Issue6Term::new(512,None);
        let mut cfg=crate::config::DEFAULT_SWARM_CONFIG.expl_cfg; cfg.time_limit=Duration::from_secs(5);
        let mut nfp=crate::config::DEFAULT_NFP_LBF_SEED_CONFIG; nfp.cache_budget_ms=0;
        let out=nfp_ordering_exploration_phase(&instance,&mut sep,&mut None,&term,&cfg,&nfp);
        assert!(term.polls.load(std::sync::atomic::Ordering::Relaxed)<512,"single item burned the entire cooperative operation budget");
        assert!(out.iter().all(|s|crate::structure::pipeline::solution_validation_error(&instance,s).is_none()));
    }


    #[test]
    fn issue6_controlled_library_and_parallel_prewarm_obey_phase_stop() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        let (instance,_)=issue6_fixture(3);
        assert!(PreparedLibrary::build_with_control(&instance,&[0,180],0.0,&||true).is_none());
        let polls=AtomicUsize::new(0);
        assert!(PreparedLibrary::build_with_control(&instance,&[0,180],0.0,&||polls.fetch_add(1,Ordering::Relaxed)>=2).is_none(),
            "mid-build cancellation must not return a partial library");
        let library=PreparedLibrary::build_with_control(&instance,&[0,180],0.0,&||false).unwrap();
        let mut cache=NfpLbfCache::new(library);
        let stats=cache.prewarm_with_control(Duration::from_secs(10),&||true);
        assert_eq!(stats.entries,0); assert_eq!(stats.prewarm_pairs,0);
        let polls=AtomicUsize::new(0);
        let pool=rayon::ThreadPoolBuilder::new().num_threads(2).build().unwrap();
        pool.install(||cache.prewarm_with_control(Duration::from_secs(10),&||polls.fetch_add(1,Ordering::Relaxed)>=9));
        let stats=cache.stats();
        assert!(stats.entries<cache.library().pair_count(),"prewarm ignored mid-phase cancellation");
        assert_eq!(stats.entries,stats.prewarm_pairs);
        // A cancelled partial cache remains fully usable by legacy lazy lookup.
        let mut placer=SequentialPlacer::new(&mut cache,30.0,80.0,PlacementScoreWeights::default());
        let result=placer.place_sequence(&[0,1,2],None,0,None);
        assert_eq!(result.placed,3); assert!(result.unplaced.is_empty());
    }
    #[test]
    fn issue6_controlled_sequence_preserves_prefix_and_discards_interrupted_item() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        let (instance,_)=issue6_fixture(3);
        let library=PreparedLibrary::build(&instance,&[0],0.0).unwrap();
        let mut cache=NfpLbfCache::new(library);
        let mut placer=SequentialPlacer::new(&mut cache,30.0,80.0,PlacementScoreWeights::default());
        let prefix=SequentialPlacement {
            pieces:vec![PlacedPiece{item_id:0,rotation:0,translation:(20.0,10.0)},PlacedPiece{item_id:1,rotation:0,translation:(32.0,10.0)}],
            strip_length:37.0,placed:2,unplaced:vec![],
        };
        for cutoff in [0,5,8,12] {
            let polls=AtomicUsize::new(0);
            let out=placer.place_sequence_with_control(&[0,1,2],Some(&prefix),2,None,
                &||polls.fetch_add(1,Ordering::Relaxed)>=cutoff);
            assert_eq!(out.placed,2,"interrupted item must not be committed: cutoff={cutoff}");
            assert_eq!(out.unplaced,vec![2]);
            for (actual,expected) in out.pieces.iter().zip(&prefix.pieces) {
                assert_eq!(actual.item_id,expected.item_id);
                assert_eq!(actual.translation,expected.translation);
                assert_eq!(actual.rotation,expected.rotation);
            }
        }
        let legacy=placer.place_sequence(&[0,1,2],None,0,None);
        let controlled=placer.place_sequence_with_control(&[0,1,2],None,0,None,&||false);
        assert_eq!(legacy.strip_length,controlled.strip_length);
        assert_eq!(legacy.unplaced,controlled.unplaced);
        assert_eq!(legacy.pieces.len(),controlled.pieces.len());
        for (a,b) in legacy.pieces.iter().zip(&controlled.pieces) { assert_eq!(a.translation,b.translation); assert_eq!(a.rotation,b.rotation); assert_eq!(a.item_id,b.item_id); }
    }
    #[test]
    fn issue6_finalizer_rejects_interrupted_materialization_and_reuses_workspace() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        let (instance,sep)=issue6_fixture(2);
        let placement=SequentialPlacement {
            pieces:sep.prob.save().layout_snapshot().placed_items().values().map(|p|PlacedPiece{
                item_id:p.item().idx(),rotation:0,translation:p.d_transf().translation(),
            }).collect(),strip_length:40.0,placed:2,unplaced:vec![],
        };
        let mut finalizer=Finalizer::new(&instance,30.0);
        let deadline=Instant::now()+Duration::from_secs(10);
        assert!(finalizer.finalize(&placement,deadline,&||true).is_none());
        let polls=AtomicUsize::new(0);
        assert!(finalizer.finalize(&placement,deadline,&||polls.fetch_add(1,Ordering::Relaxed)>=3).is_none());
        assert!(finalizer.prob.n_placed_items()<2,"fixture must cancel inside materialization");
        // Reject an attempt whose validator finished but phase Stop arrived at
        // its postcheck. No LP can run and a valid cached certificate is not a bypass.
        let polls=AtomicUsize::new(0);
        assert!(finalizer.finalize(&placement,deadline,&||polls.fetch_add(1,Ordering::Relaxed)>=7).is_none());
        assert!(finalizer.last_lp_cost.is_none());
        let certified=finalizer.finalize(&placement,deadline,&||false).unwrap();
        assert!(crate::structure::pipeline::solution_validation_error(&instance,&certified).is_none());
        assert_eq!(certified.layout_snapshot().placed_items().len(),2);
    }
    #[test]
    fn issue6_compact_stop_keeps_complete_input_and_zero_work() {
        use crate::initial::compact::{compact,compact_with_control};
        use std::sync::atomic::{AtomicUsize, Ordering};
        let (instance,sep)=issue6_fixture(3); let entry=sep.prob.save();
        let cfg=scaled_compact_cfg(30.0);
        let (stopped,stats)=compact_with_control(&instance,&entry,&cfg,5,None,&||true);
        assert_eq!(issue6_poses(&stopped),issue6_poses(&entry));
        assert_eq!(stats.lp_iters_used,0);assert_eq!(stats.rows,0);assert_eq!(stats.applied,0);
        let polls=AtomicUsize::new(0);
        let (stopped,stats)=compact_with_control(&instance,&entry,&cfg,5,None,&||polls.fetch_add(1,Ordering::Relaxed)>=4);
        assert_eq!(issue6_poses(&stopped),issue6_poses(&entry));assert_eq!(stats.lp_iters_used,0);assert_eq!(stats.applied,0);
        let (expired,stats)=compact_with_control(&instance,&entry,&cfg,5,Some(Instant::now()-Duration::from_secs(1)),&||false);
        assert_eq!(issue6_poses(&expired),issue6_poses(&entry));assert_eq!(stats.lp_iters_used,0);
        let (legacy,_)=compact(&instance,&entry,&cfg,1);
        let (controlled,_)=compact_with_control(&instance,&entry,&cfg,1,Some(Instant::now()+Duration::from_secs(5)),&||false);
        assert_eq!(issue6_poses(&legacy),issue6_poses(&controlled));assert_eq!(legacy.strip_width(),controlled.strip_width());
    }
    #[test]
    fn issue6_ordering_preserves_frozen_and_rigid_constraint_fallback() {
        use crate::optimizer::place_groups::{PlaceGroups,PlaceGroupSpec,PlaceGroupMember};
        for rigid in [false,true] {
            let (instance,mut sep)=issue6_fixture(2);let entry=sep.prob.save();
            if rigid {
                let members=entry.layout_snapshot().placed_items().values().map(|p|PlaceGroupMember{item_id:p.item().idx(),initial_transform:p.d_transf()}).collect();
                sep.set_place_groups(PlaceGroups::new(&instance,vec![PlaceGroupSpec{id:7,members}]).unwrap()).unwrap();
            }
            sep.freeze_items(&[0]);
            let term=Issue6Term::new(8,None);
            let mut cfg=crate::config::DEFAULT_SWARM_CONFIG.expl_cfg;cfg.time_limit=Duration::from_secs(5);
            let out=nfp_ordering_exploration_phase(&instance,&mut sep,&mut None,&term,&cfg,&crate::config::DEFAULT_NFP_LBF_SEED_CONFIG);
            let frozen=entry.layout_snapshot().placed_items().values().find(|p|p.item().idx()==0).unwrap();
            for sol in &out {
                let actual=sol.layout_snapshot().placed_items().values().find(|p|p.item().idx()==0).unwrap();
                assert_eq!(actual.d_transf(),frozen.d_transf());
                assert!(sep.solution_is_certified(sol,&entry));
                if rigid { assert_eq!(issue6_poses(sol),issue6_poses(&entry)); }
            }
        }
    }

}
