//! 정체 감지 기반 적응형 탈출을 얹은 탐색(exploration) 단계.
//!
//! 기존 [`crate::optimizer::explore_mods::exploration_phase_adaptive`]와 동일한
//! 시그니처 계열의 대체 탐색 단계다. 폭 축소 루프·infeasible 풀·`separate` 호출
//! 규약은 그대로 유지하고, 그 위에 문서의 의사코드를 그대로 얹는다.
//!
//! ```text
//! while not timeout:
//!
//!     swarm_step()
//!     local_compaction()
//!
//!     if best_efficiency improved:
//!         stagnation = 0
//!         escape_level = 0
//!         decay_guided_weights()
//!     else:
//!         stagnation += 1
//!
//!     if stagnation >= threshold
//!        and population_diversity < diversity_threshold:
//!
//!         targets = select_bottleneck_items(
//!             right_boundary,
//!             large_void_neighbors,
//!             blocking_degree,
//!             move_failure_count
//!         )
//!
//!         if escape_level == 0:
//!             relocate_or_swap(targets[0:2])
//!
//!         else if escape_level == 1:
//!             destroy_and_repair(
//!                 targets,
//!                 ratio = 0.05 ~ 0.10
//!             )
//!
//!         else:
//!             preserve_dense_left_core()
//!             rebuild_right_region(
//!                 ratio = 0.15 ~ 0.30
//!             )
//!             reinitialize_low_quality_particles()
//!
//!         increase_guided_weights()
//!         temporarily_increase_exploration()
//!
//!         escape_level = min(escape_level + 1, 2)
//!         stagnation = 0
//! ```
//!
//! 각 단계의 대응 관계:
//!
//! * `swarm_step()`        → [`Separator::separate`]
//! * `local_compaction()`  → [`Separator::refit_strip`]
//! * `select_bottleneck_items(..)` → [`select_bottleneck_items`]
//! * `escape_level 0/1/2`  → [`EscapeLevel::Soft`] / [`EscapeLevel::Medium`] / [`EscapeLevel::Hard`]
//! * `reinitialize_low_quality_particles()` → [`SwarmDiversification`]의 `Reinit` 계층 교체
//! * `increase_guided_weights()` → [`GuidedWeights::increase_guided_weights`]
//! * `temporarily_increase_exploration()` → [`SwarmDiversification::exploration_boost`]를
//!   infeasible 풀 선택 분포의 표준편차에 반영

use std::collections::HashMap;

use crate::config::ExplorationConfig;
use crate::escape::bottleneck::select_bottleneck_items;
use crate::escape::conflict_burst::{ConflictBurstConfig, ConflictGraph};
use crate::escape::guided::GuidedWeights;
use crate::escape::levels::{EscapeLevel, apply_escape};
use crate::escape::stagnation::{StagnationConfig, StagnationDetector};
use crate::escape::swarm::{ParticleTier, SwarmDiversification};
use crate::escape::{COLLISION_RATIO_LOW, ESCAPE_LEVEL_MAX, POST_ESCAPE_BOOST_ITERS};
use crate::optimizer::explore::disrupt_solution;
use crate::optimizer::lns::{BandTabu, instance_rotation_degrees};
use crate::optimizer::separator::Separator;
use crate::util::listener::{ReportType, SolutionListener};
use crate::util::terminator::Terminator;
use jagua_rs::entities::PItemKey;
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use rand::prelude::Distribution;
use rand_distr::Normal;

/// 충돌 쌍 스캔의 상한. 충돌 아이템이 많을 때 `O(k²)` 스캔이 폭주하지 않게 자른다.
const MAX_CONFLICT_SCAN: usize = 128;

/// infeasible 풀의 기본 상한. 원본 탐색 단계와 같은 값이다.
const DEFAULT_MAX_INFEASIBLE_POOL: usize = 32;

/// 풀 선택 분포 표준편차의 안전 범위. 0 이하나 NaN이면 분포 생성이 실패한다.
const POOL_STDDEV_RANGE: (f32, f32) = (0.01, 4.0);

/// 이 파이프라인에서 정체 관측 창의 상한 (바깥 루프 반복 = `separate` 호출 단위).
///
/// 문서의 `max(100, 3 × 패턴 수)` 는 **separator 내부 이동 반복** 단위로 쓴 창이다.
/// 이 파이프라인의 `record_iteration` 은 `separate_with_repair` 한 번(= 수백~수천 번의
/// 내부 이동)마다 한 번 호출되므로, 그 창을 그대로 쓰면 한 폭에서 100회 이상의 완전한
/// separation 이 지나야 비로소 탈출이 가능해져 실행 시간 예산 안에서는 사실상 한 번도
/// 발동하지 않는다. 그래서 창을 바깥 반복 단위로 다시 재고 패턴 수 배수를 끈다.
pub const STAGNATION_WINDOW_OUTER_CAP: usize = 8;

// ────────────────────────────────────────────────────────────────────────────
// 설정 / 상태
// ────────────────────────────────────────────────────────────────────────────

/// 적응형 탈출 파이프라인 설정.
// SwarmConfig가 Copy라서 이 구조체도 Copy여야 한다 (config.rs의 const 기본값).
#[derive(Clone, Copy, Debug)]
pub struct AdaptiveEscapeConfig {
    /// 정체 감지 임계값 묶음.
    pub stagnation: StagnationConfig,
    /// Conflict Burst + Partial Rebuild 설정.
    pub conflict_burst: ConflictBurstConfig,
    /// 광범위한 충돌이 감지되면 Conflict Burst 경로를 쓸지.
    pub enable_conflict_burst: bool,
    /// Hard 탈출에서 하위 후보해를 Elite 기준으로 재초기화할지.
    pub enable_swarm_diversification: bool,
    /// infeasible 해 풀의 크기 상한 (loss 오름차순 유지).
    pub max_infeasible_pool: usize,
}

impl Default for AdaptiveEscapeConfig {
    fn default() -> Self {
        Self {
            stagnation: StagnationConfig::default(),
            conflict_burst: ConflictBurstConfig::default(),
            enable_conflict_burst: true,
            enable_swarm_diversification: true,
            max_infeasible_pool: DEFAULT_MAX_INFEASIBLE_POOL,
        }
    }
}

/// 파이프라인의 탈출 상태. 의사코드의 `stagnation` / `escape_level`이
/// 그대로 필드로 살아 있다.
#[derive(Debug, Default, Clone)]
pub struct AdaptiveEscapeState {
    /// 의사코드의 `stagnation` 카운터.
    pub stagnation: usize,
    /// 의사코드의 `escape_level` (0 = Level 1, 1 = Level 2, 2 = Level 3).
    pub escape_level: u8,
    /// 마지막 탈출 이후 경과한 반복 수. `temporarily_increase_exploration()`의 수명이다.
    pub iters_since_escape: usize,
}

// ────────────────────────────────────────────────────────────────────────────
// 순수 헬퍼 — 파일 하단 단위 테스트가 검증하는 수식들
// ────────────────────────────────────────────────────────────────────────────

/// 문서의 `효율 개선량 < 0.01%p`를 뒤집은 판정. `eps`를 **넘어야만** 개선이다.
#[inline]
fn efficiency_improved(efficiency: f32, best: f32, eps: f32) -> bool {
    efficiency.is_finite() && efficiency > best + eps.max(0.0)
}

/// Conflict Burst 진입 임계값 — `detector.window() / 4` (최소 1).
#[inline]
fn conflict_burst_threshold(window: usize) -> usize {
    (window / 4).max(1)
}

/// 설정의 정체 창을 바깥 루프 단위로 다시 잰 감지기 설정.
/// [`STAGNATION_WINDOW_OUTER_CAP`] 참조 — 나머지 임계값은 그대로 둔다.
#[inline]
fn outer_loop_stagnation_config(cfg: StagnationConfig) -> StagnationConfig {
    StagnationConfig {
        window_floor: cfg.window_floor.min(STAGNATION_WINDOW_OUTER_CAP).max(1),
        window_item_factor: 0,
        ..cfg
    }
}

/// 탈출 강도. 감지기의 신호 기반 강도와 **현재 탈출 단계** 중 큰 쪽이다.
///
/// 감지기는 탈출 직후 `reset()` 되어 다음 탈출 시점에는 강도가 다시 0 근처에서 출발한다.
/// 단계가 Level 2/3 로 올라갔는데 강도가 바닥이면 `5~10%`, `15~30%` 같은 범위의 최솟값만
/// 쓰게 되어 단계 상승이 무의미해지므로, 단계 자체를 강도의 하한으로 삼는다.
#[inline]
fn escape_intensity(detector_intensity: f32, escape_level: u8) -> f32 {
    let level_floor = escape_level.min(ESCAPE_LEVEL_MAX) as f32 / ESCAPE_LEVEL_MAX.max(1) as f32;
    let d = if detector_intensity.is_nan() {
        0.0
    } else {
        detector_intensity.clamp(0.0, 1.0)
    };
    d.max(level_floor)
}

/// `temporarily_increase_exploration()`이 실제로 만지는 값.
///
/// Global Best 추종 강도(`pull`)가 낮을수록, 랜덤 탐색 강도(`boost`)가 높을수록
/// infeasible 풀에서 **더 넓게** 고르도록 표준편차를 키운다. 기본 상수
/// (`pull = 0.5`, `boost = 2.0`)에서 탈출 직후 배율은 4배, 부스트가 끝나면
/// (`pull = 1.0`, `boost = 1.0`) 정확히 설정값으로 되돌아온다.
#[inline]
fn pool_selection_stddev(base: f32, global_best_pull: f32, exploration_boost: f32) -> f32 {
    let base = if base.is_finite() && base > 0.0 {
        base
    } else {
        POOL_STDDEV_RANGE.0
    };
    let pull = if global_best_pull.is_finite() && global_best_pull > 0.0 {
        global_best_pull
    } else {
        1.0
    };
    let boost = if exploration_boost.is_finite() && exploration_boost > 0.0 {
        exploration_boost
    } else {
        1.0
    };
    (base * boost / pull).clamp(POOL_STDDEV_RANGE.0, POOL_STDDEV_RANGE.1)
}

/// loss 오름차순 풀에 `loss`가 들어갈 자리.
#[inline]
fn pool_insert_index(losses: &[f32], loss: f32) -> usize {
    match losses.binary_search_by(|probe| probe.total_cmp(&loss)) {
        Ok(idx) | Err(idx) => idx,
    }
}

/// 정규분포 샘플을 풀 인덱스로 접는다. 좋은(앞쪽) 해가 더 자주 뽑힌다.
#[inline]
fn pool_index_from_sample(sample: f32, len: usize) -> usize {
    if len == 0 {
        return 0;
    }
    let s = if sample.is_finite() {
        sample.abs().min(0.999)
    } else {
        0.0
    };
    ((s * len as f32) as usize).min(len - 1)
}

// ────────────────────────────────────────────────────────────────────────────
// 레이아웃 관찰 헬퍼
// ────────────────────────────────────────────────────────────────────────────

/// 현재 충돌 중인 아이템 쌍을 `(item_id, item_id)`로 모은다.
///
/// MultiPolygon 제약상 이동 단위는 place group이지만, Guided 가중치와 정체 통계는
/// 문서대로 **패턴(아이템) 쌍** 단위로 학습한다.
fn colliding_pairs(sep: &Separator) -> Vec<(usize, usize)> {
    let mut conflicted: Vec<(PItemKey, usize)> = sep
        .prob
        .layout()
        .placed_items()
        .iter()
        .filter(|(pk, _)| sep.ct.get_loss(*pk) > 0.0)
        .map(|(pk, pi)| (pk, pi.item().idx()))
        .collect();
    conflicted.truncate(MAX_CONFLICT_SCAN);

    let mut pairs = Vec::new();
    for i in 0..conflicted.len() {
        for j in (i + 1)..conflicted.len() {
            let (pk_a, id_a) = conflicted[i];
            let (pk_b, id_b) = conflicted[j];
            if sep.ct.get_pair_loss(pk_a, pk_b) > 0.0 {
                pairs.push((id_a, id_b));
            }
        }
    }
    pairs
}

/// 충돌을 그대로 안고 남은 아이템의 "이동 실패" 횟수를 누적한다.
/// [`select_bottleneck_items`]의 `MoveFailureFrequency` 항에 그대로 들어간다.
fn accumulate_move_failures(sep: &Separator, move_failures: &mut HashMap<usize, u32>) {
    for (pk, pi) in sep.prob.layout().placed_items().iter() {
        if sep.ct.get_loss(pk) > 0.0 {
            *move_failures.entry(pi.item().idx()).or_insert(0) += 1;
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 탐색 단계 본체
// ────────────────────────────────────────────────────────────────────────────

/// 정체 감지 기반 적응형 탈출을 얹은 탐색 단계.
///
/// 반환값은 원본과 동일하게 `feasible_sols`이며 **마지막 원소가 최종 해**다.
/// 최소 한 개(시작 해)는 항상 들어 있다.
pub fn escape_driven_exploration_phase(
    instance: &SPInstance,
    sep: &mut Separator,
    sol_listener: &mut Option<&mut dyn SolutionListener>,
    term: &impl Terminator,
    expl_config: &ExplorationConfig,
    escape_config: &AdaptiveEscapeConfig,
) -> Vec<SPSolution> {
    escape_driven_exploration_phase_with_nfp(
        instance,
        sep,
        sol_listener,
        term,
        expl_config,
        escape_config,
        &crate::config::DEFAULT_NFP_INTEGRATION_CONFIG,
    )
}

/// [`escape_driven_exploration_phase`]에 탐색 단계 NFP 활용 설정을 얹은 진입점.
///
/// * `exact_separation_vector` — Conflict Burst 의 `EscapeVector`가 쓰는 분리 벡터를
///   bbox MTV 근사 대신 정확한 NFP 기반으로 바꾼다.
/// * `nfp_reinsertion` — Level 2/3 와 Conflict Burst Stage 3 의 R&R 재삽입을 NFP 기반으로 바꾼다.
///
/// 꺼져 있으면 기존 경로가 한 비트도 바뀌지 않는다.
#[allow(clippy::too_many_arguments)]
pub fn escape_driven_exploration_phase_with_nfp(
    instance: &SPInstance,
    sep: &mut Separator,
    sol_listener: &mut Option<&mut dyn SolutionListener>,
    term: &impl Terminator,
    expl_config: &ExplorationConfig,
    escape_config: &AdaptiveEscapeConfig,
    nfp_cfg: &crate::config::NfpIntegrationConfig,
) -> Vec<SPSolution> {
    sep.configure_reinsertion(nfp_cfg);
    let _run_scope = crate::config::runtime::enter_if_unset(sep.run_control.clone());
    // 분리 벡터용 NFP 캐시. 플래그가 꺼져 있으면 만들지 않는다.
    // 회전 집합은 인스턴스가 허용하는 각도 전부여야 한다 — `&[0]` 만 준비하면 회전된
    // 배치의 pair NFP 가 캐시 miss 로 bbox 근사로 떨어진다.
    let mut nfp_sep_cache = nfp_cfg
        .exact_separation_vector
        .then(|| {
            instance_rotation_degrees(instance).and_then(|rotations| {
                crate::optimizer::lns::NfpReinsertionCache::new(
                    instance,
                    &rotations,
                    crate::nfp_lbf::DEFAULT_CONCAVE_SIMPLIFY_TOLERANCE,
                )
            })
        })
        .flatten();
    if nfp_cfg.exact_separation_vector && nfp_sep_cache.is_none() {
        log::warn!(
            "[ESCAPE] 정확한 분리 벡터를 켰지만 NFP 라이브러리를 만들지 못했다 — bbox 근사를 쓴다"
        );
    }

    let n_patterns = instance.total_item_qty().max(1);
    let max_pool = escape_config.max_infeasible_pool.max(1);

    let mut detector = StagnationDetector::new(
        outer_loop_stagnation_config(escape_config.stagnation),
        n_patterns,
    );
    let mut guided = GuidedWeights::new();
    let mut swarm = SwarmDiversification::from_stagnation_intensity(0.0);
    let mut tabu = BandTabu::new();

    let mut state = AdaptiveEscapeState {
        // 시작 시점은 "탈출 직후"가 아니다 — 부스트를 꺼 둔 채 출발한다.
        iters_since_escape: POST_ESCAPE_BOOST_ITERS,
        ..AdaptiveEscapeState::default()
    };

    let mut current_width = sep.prob.strip_width();
    let mut best_width = current_width;
    let mut best_efficiency = sep.prob.density();

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

    // infeasible 해 풀 — loss 오름차순 유지. 두 벡터를 같은 인덱스로 동기화해서
    // `population_diversity`에 복사 없이 슬라이스를 넘긴다.
    let mut pool_sols: Vec<SPSolution> = Vec::new();
    let mut pool_losses: Vec<f32> = Vec::new();

    let mut move_failures: HashMap<usize, u32> = HashMap::new();
    let mut n_conseq_fails: usize = 0;
    let mut n_fails_at_width: usize = 0;
    let mut prev_loss = f32::INFINITY;

    while !term.kill() {
        // ── ① swarm_step() ───────────────────────────────────────────────
        let (local_sol, local_ct) =
            sep.separate_with_repair(term, sol_listener, &expl_config.repair, false);
        let total_loss = local_ct.get_total_loss();
        let feasible =
            total_loss == 0.0 && sep.solution_is_certified(&local_sol, &initial_solution);
        // 손실 0 인데 정확 인증을 통과하지 못한 후보는 이 폭에서의 실패한 시도로 다룬다
        // (`explore_mods.rs` 의 같은 자리 참조). 예전의 `break` 는 인증 실패 한 번에 남은 탐색
        // 예산을 전부 버렸다.
        let total_loss = if total_loss == 0.0 && !feasible {
            log::warn!(
                "[ESCAPE] zero-loss candidate failed exact certification; treating it as a failed attempt"
            );
            f32::MIN_POSITIVE
        } else {
            total_loss
        };

        // `separate`는 최선 해를 반환할 뿐 separator를 그 상태로 되돌리지 않는다.
        // 뒤따르는 압축/관찰이 실제 최선 해를 보도록 여기서 명시적으로 맞춘다.
        sep.rollback(&local_sol, Some(&local_ct));

        // ── ② local_compaction() ─────────────────────────────────────────
        // 아이템을 움직이지 않고 남는 폭만 회수한다. 충돌이 남아 있으면 의미가 없다.
        if feasible && sep.refit_strip() {
            current_width = sep.prob.strip_width();
        }

        // 밀도는 `배치 면적 / (폭 × 높이)`라 배치가 아니라 **폭**으로 결정된다.
        // 즉 여기서의 efficiency 개선은 곧 폭 개선이다.
        let efficiency = sep.prob.density();

        // ── ③ 정체 신호 기록 ────────────────────────────────────────────
        detector.record_iteration(
            if feasible {
                efficiency
            } else {
                best_efficiency
            },
            StagnationDetector::right_boundary_signature(sep),
        );
        detector.record_move(feasible || total_loss < prev_loss);
        detector.record_population(StagnationDetector::population_diversity(&pool_sols));
        prev_loss = total_loss;

        // ── ④ if best_efficiency improved ───────────────────────────────
        if feasible
            && efficiency_improved(
                efficiency,
                best_efficiency,
                escape_config.stagnation.efficiency_eps,
            )
        {
            best_efficiency = efficiency;
            state.stagnation = 0;
            state.escape_level = 0;
            guided.decay_guided_weights();
        } else {
            state.stagnation += 1;
        }

        if feasible {
            // ── feasible: 인컴번트 갱신 후 폭을 줄여 다시 도전한다 ────────
            if current_width < best_width || feasible_sols.is_empty() {
                best_width = current_width;
                let sol = sep.prob.save();
                feasible_sols.push(sol.clone());
                if let Some(l) = sol_listener.as_deref_mut() {
                    l.report(ReportType::ExplFeas, &sol, instance);
                }
            }

            let step = crate::config::runtime::effective_shrink_step(expl_config.shrink_step);
            let mut next_width = current_width * (1.0 - step);
            // 고정 아이템이 경계를 침범하지 않도록 축소 폭의 하한을 clamp
            if let Some(min_w) = sep.min_feasible_width()
                && next_width < min_w
            {
                next_width = min_w;
            }
            if !next_width.is_finite() || next_width >= current_width {
                break;
            }

            sep.change_strip_width(next_width, None);
            current_width = next_width;

            pool_sols.clear();
            pool_losses.clear();
            move_failures.clear();
            n_conseq_fails = 0;
            n_fails_at_width = 0;
            prev_loss = f32::INFINITY;
            tabu.clear(); // 폭이 바뀌면 밴드 좌표계가 이동한다
            state.iters_since_escape = state.iters_since_escape.saturating_add(1);
            continue;
        }

        // ── infeasible: 풀에 담고 다음 반복을 준비한다 ────────────────────
        if let Some(l) = sol_listener.as_deref_mut() {
            l.report(ReportType::ExplInfeas, &local_sol, instance);
        }

        let idx = pool_insert_index(&pool_losses, total_loss);
        // 이 폭에서 지금까지의 최선 충돌 해라면 "개선 중"으로 보고한다.
        if idx == 0 && !pool_losses.is_empty() {
            if let Some(l) = sol_listener.as_deref_mut() {
                l.report(ReportType::ExplImproving, &local_sol, instance);
            }
        }
        pool_sols.insert(idx, local_sol);
        pool_losses.insert(idx, total_loss);
        pool_sols.truncate(max_pool);
        pool_losses.truncate(max_pool);

        accumulate_move_failures(sep, &mut move_failures);
        for (a, b) in colliding_pairs(sep) {
            detector.record_pair_failure(a, b);
        }

        n_conseq_fails += 1;
        n_fails_at_width += 1;

        if n_conseq_fails >= expl_config.max_conseq_failed_attempts.unwrap_or(usize::MAX) {
            break;
        }

        // separation이 중단 요청으로 빠져나왔다면 아래의 비싼 교란은 전부 버려진다.
        if term.kill() {
            break;
        }

        // ── ⑤ Conflict Burst — 같은 폭에서 광범위한 충돌이 굳었을 때 ──────
        if escape_config.enable_conflict_burst
            && n_fails_at_width >= conflict_burst_threshold(detector.window())
        {
            let ratio = ConflictGraph::build(sep).collision_ratio();
            if ratio >= COLLISION_RATIO_LOW {
                // 문서: particle 계층별로 탈출 강도를 달리한다.
                //
                // ```text
                // Best 20%   -> 현재 배치 유지 + 약한 local search
                // Middle 50% -> Collision Push + 부분 재삽입
                // Worst 30%  -> Strip Relaxation + 큰 Destroy & Repair
                // ```
                //
                // 이 파이프라인에서 "particle"은 infeasible 풀의 해다. 풀은 loss 오름차순이라
                // 현재 separator 가 들고 있는 해가 풀에서 몇 번째인지가 곧 그 계층이다.
                // Best 계층이면 강한 교란을 걸지 않고 그대로 둔다 — 좋은 해를 부수지 않는다.
                let loss_before_burst = sep.ct.get_total_loss();
                let width_before_burst = current_width;
                let tier = current_particle_tier(&pool_losses, loss_before_burst);
                let burst_cfg = tier_burst_config(&escape_config.conflict_burst, tier);
                let outcome =
                    crate::escape::conflict_burst::conflict_burst_partial_rebuild_with_nfp(
                        sep,
                        &burst_cfg,
                        &expl_config.rnr,
                        &mut tabu,
                        term,
                        nfp_sep_cache.as_mut().map(|c| c.cache_mut()),
                    );
                current_width = sep.prob.strip_width();

                if outcome.resolved {
                    // 폭이 완화되었을 수 있으므로 풀·밴드 좌표계를 모두 버린다.
                    pool_sols.clear();
                    pool_losses.clear();
                    n_conseq_fails = 0;
                    n_fails_at_width = 0;
                    prev_loss = f32::INFINITY;
                    tabu.clear();
                    state.iters_since_escape = state.iters_since_escape.saturating_add(1);
                    continue;
                }

                // 해소되지 않았다. 카운터를 되돌리지 않으면 다음 반복부터 매번 burst 가
                // 다시 불려 separator 가 전혀 돌지 못한다.
                n_fails_at_width = 0;

                // burst 가 loss 를 낮춘 상태는 이 폭에서의 새 후보다. 바로 아래의 풀 롤백이
                // 이 상태를 버리기 전에 풀에 담아 둔다 (unresolved burst 는 폭을 되돌리므로
                // 풀의 해와 같은 폭일 때만).
                let loss_after_burst = sep.ct.get_total_loss();
                if sep.prob.strip_width() == width_before_burst
                    && loss_after_burst.is_finite()
                    && loss_after_burst < loss_before_burst
                {
                    let idx = pool_insert_index(&pool_losses, loss_after_burst);
                    pool_sols.insert(idx, sep.prob.save());
                    pool_losses.insert(idx, loss_after_burst);
                    pool_sols.truncate(max_pool);
                    pool_losses.truncate(max_pool);
                }
            }
        }

        // 폭이 바뀌지 않았어야 풀의 해로 되돌릴 수 있다 (rollback은 동일 폭 전제).
        if pool_sols.is_empty() || sep.prob.strip_width() != current_width {
            state.iters_since_escape = state.iters_since_escape.saturating_add(1);
            continue;
        }

        // ── ⑥ temporarily_increase_exploration()이 반영된 풀 선택 ─────────
        let stddev = pool_selection_stddev(
            expl_config.solution_pool_distribution_stddev,
            swarm.global_best_pull(state.iters_since_escape),
            swarm.exploration_boost(state.iters_since_escape),
        );
        let sample = match Normal::new(0.0f32, stddev) {
            Ok(d) => d.sample(&mut sep.rng),
            Err(_) => 0.0,
        };
        let selected = pool_index_from_sample(sample, pool_sols.len());
        sep.rollback(&pool_sols[selected], None);
        // `rollback(.., None)` 은 트래커를 새로 만들어 쌍 가중치를 중립으로 되돌린다.
        // 학습된 Guided 가중치는 풀을 바꿔 탈 때마다 다시 얹어야 살아남는다.
        guided.apply_to_tracker(sep);

        // ── ⑦ 탈출 조건: stagnation >= threshold AND 정체 신호 성립 ───────
        let stagnant = state.stagnation >= detector.window() && detector.is_stagnant();
        if !stagnant {
            let _ = disrupt_solution(sep, expl_config);
            state.iters_since_escape = state.iters_since_escape.saturating_add(1);
            continue;
        }

        let intensity = escape_intensity(detector.stagnation_intensity(), state.escape_level);
        let analysis = select_bottleneck_items(sep, &move_failures);
        let level = EscapeLevel::from_code(state.escape_level);

        let outcome = apply_escape(
            sep,
            level,
            &analysis,
            intensity,
            &expl_config.rnr,
            &mut tabu,
            term,
        );
        detector.record_move(outcome.applied);
        current_width = sep.prob.strip_width();

        // ── ⑧ reinitialize_low_quality_particles() (Level 3 전용) ────────
        if level == EscapeLevel::Hard
            && escape_config.enable_swarm_diversification
            && pool_sols.len() >= 2
        {
            swarm = SwarmDiversification::from_stagnation_intensity(intensity);
            let tiers = swarm.classify(pool_sols.len());
            let elites: Vec<usize> = tiers
                .iter()
                .enumerate()
                .filter(|(_, tier)| **tier == ParticleTier::Elite)
                .map(|(i, _)| i)
                .collect();

            if !elites.is_empty() {
                let mut n_reinit = 0usize;
                let mut next_sols = Vec::with_capacity(pool_sols.len());
                let mut next_losses = Vec::with_capacity(pool_losses.len());
                for i in 0..pool_sols.len() {
                    match tiers.get(i) {
                        Some(&ParticleTier::Reinit) => {
                            // 하위 계층 해는 버리고 Elite 해를 **기준으로** 재초기화한다.
                            // Elite 를 그대로 복제하면 풀에 같은 해가 여러 장 생겨 다양성이
                            // 오히려 떨어지므로, 최대 강도의 Soft 교란을 얹어 흩뜨린다.
                            // 교란이 성립하지 않으면 그대로 복제한다.
                            let src = elites[n_reinit % elites.len()];
                            n_reinit += 1;
                            match relocate_middle_particle(
                                sep,
                                &pool_sols[src],
                                1.0,
                                &move_failures,
                                &expl_config.rnr,
                                &mut tabu,
                                term,
                            ) {
                                Some((sol, loss)) => {
                                    next_sols.push(sol);
                                    next_losses.push(loss);
                                }
                                None => {
                                    next_sols.push(pool_sols[src].clone());
                                    next_losses.push(pool_losses[src]);
                                }
                            }
                        }
                        Some(&ParticleTier::Middle) => {
                            // 문서 §5: `중간 후보 -> 병목 패턴 부분 재배치`.
                            //
                            // Elite 처럼 보존하지도, Reinit 처럼 버리지도 않는다. 그 해로
                            // 되돌아가 병목만 골라 소규모로 흔든 뒤(Level 1 Soft) 다시 담는다.
                            // 개선되지 않으면 `apply_escape` 가 트랜잭션으로 되돌리므로
                            // 이 계층이 해를 나쁘게 만들 수는 없다.
                            match relocate_middle_particle(
                                sep,
                                &pool_sols[i],
                                intensity,
                                &move_failures,
                                &expl_config.rnr,
                                &mut tabu,
                                term,
                            ) {
                                Some((sol, loss)) => {
                                    next_sols.push(sol);
                                    next_losses.push(loss);
                                }
                                None => {
                                    next_sols.push(pool_sols[i].clone());
                                    next_losses.push(pool_losses[i]);
                                }
                            }
                        }
                        _ => {
                            next_sols.push(pool_sols[i].clone());
                            next_losses.push(pool_losses[i]);
                        }
                    }
                }
                // loss 오름차순 불변식을 복구한다.
                let mut order: Vec<usize> = (0..next_losses.len()).collect();
                order.sort_by(|&a, &b| next_losses[a].total_cmp(&next_losses[b]));
                pool_sols = order.iter().map(|&i| next_sols[i].clone()).collect();
                pool_losses = order.iter().map(|&i| next_losses[i]).collect();
            }
        }

        // ── ⑨ increase_guided_weights() ─────────────────────────────────
        // `observe_collisions` 가 충돌 쌍을 모아 한 번씩 증가시킨다 — 같은 쌍을 다시
        // 증가시키면 한 번의 탈출에 1.3² 배가 되어 가중치가 폭주한다.
        guided.observe_collisions(sep);
        guided.apply_to_tracker(sep);

        // ── ⑩ temporarily_increase_exploration() + 강도 상승 ─────────────
        state.iters_since_escape = 0;
        state.escape_level = level.next().code();
        state.stagnation = 0;
        detector.reset();
        tabu.clear();
    }

    // Never leave a caller looking at a last, unrepaired perturbation.
    sep.rollback(feasible_sols.last().unwrap_or(&initial_solution), None);
    feasible_sols
}

// ────────────────────────────────────────────────────────────────────────────
// 단위 테스트 — 외부 데이터 없이 수식·임계값만 검증한다.
// ────────────────────────────────────────────────────────────────────────────

/// 현재 해가 풀에서 어느 particle 계층에 속하는지.
///
/// 풀은 loss 오름차순이므로 현재 loss 의 순위가 곧 계층이다.
/// 비율은 문서가 정한 `Best 20% / Middle 50% / Worst 30%`
/// ([`SwarmDiversification::conflict_tiers`])를 그대로 쓴다.
fn current_particle_tier(pool_losses: &[f32], current_loss: f32) -> ParticleTier {
    if pool_losses.is_empty() {
        return ParticleTier::Middle;
    }
    let tiers = SwarmDiversification::conflict_tiers(pool_losses.len());
    let rank = pool_losses
        .iter()
        .filter(|&&l| l < current_loss)
        .count()
        .min(tiers.len().saturating_sub(1));
    tiers.get(rank).copied().unwrap_or(ParticleTier::Middle)
}

/// particle 계층에 맞춰 Conflict Burst 강도를 조절한 설정.
///
/// * `Elite`  — 좋은 해다. 제거 비율을 최소로 낮춰 배치를 최대한 보존한다.
/// * `Middle` — 설정 그대로.
/// * `Reinit` — 나쁜 해다. 제거 비율을 상한까지 올려 크게 해체한다.
fn tier_burst_config(base: &ConflictBurstConfig, tier: ParticleTier) -> ConflictBurstConfig {
    let mut cfg = *base;
    match tier {
        ParticleTier::Elite => {
            cfg.remove_ratio_max = cfg.remove_ratio_min;
        }
        ParticleTier::Middle => {}
        ParticleTier::Reinit => {
            cfg.remove_ratio_min = cfg.remove_ratio_max;
        }
    }
    cfg
}

/// 중간 계층 후보 하나를 병목 기준으로 소규모 재배치한다.
///
/// 문서 §5의 `중간 후보 -> 병목 패턴 부분 재배치`에 대응한다. 해당 해로 롤백해
/// [`EscapeLevel::Soft`]를 적용하고, 실제로 pose가 바뀐 경우에만 새 해와 그 loss를 돌려준다.
///
/// 반환이 `None`이면 호출자는 원래 해를 그대로 둔다. separator 상태는 어느 경로에서도
/// 호출 전으로 되돌려 놓는다 — 이 함수는 풀만 건드리고 진행 중인 탐색 상태를 바꾸지 않는다.
fn relocate_middle_particle(
    sep: &mut Separator,
    particle: &SPSolution,
    intensity: f32,
    move_failures: &std::collections::HashMap<usize, u32>,
    rnr_cfg: &crate::config::RuinRecreateConfig,
    tabu: &mut crate::optimizer::lns::BandTabu,
    term: &impl Terminator,
) -> Option<(SPSolution, f32)> {
    if term.kill() || particle.strip_width() != sep.prob.strip_width() {
        return None;
    }
    let restore = sep.prob.save();
    let restore_ct = sep.ct.save();

    sep.rollback(particle, None);
    let analysis = select_bottleneck_items(sep, move_failures);
    let outcome = apply_escape(
        sep,
        EscapeLevel::Soft,
        &analysis,
        intensity,
        rnr_cfg,
        tabu,
        term,
    );

    let result = if outcome.applied && outcome.n_relocated > 0 {
        Some((sep.prob.save(), sep.ct.get_total_loss()))
    } else {
        None
    };

    // 풀만 갱신한다 — separator는 호출 전 상태로 되돌린다.
    sep.rollback(&restore, Some(&restore_ct));
    result
}

#[cfg(test)]
mod tests {
    use super::{
        AdaptiveEscapeConfig, AdaptiveEscapeState, DEFAULT_MAX_INFEASIBLE_POOL, POOL_STDDEV_RANGE,
        STAGNATION_WINDOW_OUTER_CAP, conflict_burst_threshold, efficiency_improved,
        escape_intensity, outer_loop_stagnation_config, pool_index_from_sample,
        pool_insert_index, pool_selection_stddev,
    };
    use crate::escape::stagnation::StagnationConfig;
    use crate::escape::{
        ESCAPE_LEVEL_MAX, POST_ESCAPE_EXPLORATION_BOOST, POST_ESCAPE_GLOBAL_BEST_PULL,
        STAGNATION_EFFICIENCY_EPS,
    };

    #[test]
    fn outer_loop_window_is_capped_and_ignores_pattern_count() {
        let cfg = outer_loop_stagnation_config(StagnationConfig::default());
        assert_eq!(cfg.window_floor, STAGNATION_WINDOW_OUTER_CAP);
        assert_eq!(cfg.window_item_factor, 0);
        // 패턴 수와 무관하게 창은 상한에 고정된다 (separate 호출 단위).
        assert_eq!(cfg.window(0), STAGNATION_WINDOW_OUTER_CAP);
        assert_eq!(cfg.window(10_000), STAGNATION_WINDOW_OUTER_CAP);
        // 나머지 임계값은 그대로.
        assert_eq!(cfg.efficiency_eps, STAGNATION_EFFICIENCY_EPS);
        // 사용자가 더 작은 창을 줬다면 존중하되 0은 아니다.
        let small = outer_loop_stagnation_config(StagnationConfig {
            window_floor: 3,
            ..StagnationConfig::default()
        });
        assert_eq!(small.window(500), 3);
        let zero = outer_loop_stagnation_config(StagnationConfig {
            window_floor: 0,
            ..StagnationConfig::default()
        });
        assert_eq!(zero.window(500), 1);
    }

    #[test]
    fn escape_intensity_never_drops_below_the_escape_level() {
        // Level 1 (code 0): 감지기 강도 그대로.
        assert_eq!(escape_intensity(0.0, 0), 0.0);
        assert!((escape_intensity(0.3, 0) - 0.3).abs() < 1.0e-6);
        // Level 2 (code 1): 하한 0.5.
        assert!((escape_intensity(0.0, 1) - 0.5).abs() < 1.0e-6);
        assert!((escape_intensity(0.8, 1) - 0.8).abs() < 1.0e-6);
        // Level 3 (code ESCAPE_LEVEL_MAX): 하한 1.0.
        assert_eq!(escape_intensity(0.0, ESCAPE_LEVEL_MAX), 1.0);
        assert_eq!(escape_intensity(0.2, 7), 1.0);
        // 비정상 입력은 단계 하한으로 떨어진다.
        assert_eq!(escape_intensity(f32::NAN, 0), 0.0);
        assert_eq!(escape_intensity(f32::INFINITY, 0), 1.0);
    }

    #[test]
    fn improvement_needs_to_exceed_the_documented_epsilon() {
        let eps = STAGNATION_EFFICIENCY_EPS;
        // 정확히 임계값만큼은 "개선 없음"이다 (문서: 개선량 < 0.01%p → 정체).
        assert!(!efficiency_improved(0.90 + eps, 0.90, eps));
        assert!(efficiency_improved(0.90 + eps * 2.0, 0.90, eps));
        assert!(!efficiency_improved(0.89, 0.90, eps));
        assert!(!efficiency_improved(f32::NAN, 0.90, eps));
    }

    #[test]
    fn conflict_burst_threshold_is_a_quarter_window_but_never_zero() {
        assert_eq!(conflict_burst_threshold(100), 25);
        assert_eq!(conflict_burst_threshold(468), 117);
        // 창이 아무리 작아도 0이 되어 매 반복 burst를 부르면 안 된다.
        assert_eq!(conflict_burst_threshold(3), 1);
        assert_eq!(conflict_burst_threshold(0), 1);
    }

    #[test]
    fn post_escape_boost_widens_the_pool_distribution_and_then_returns() {
        let base = 0.25;
        // 탈출 직후: pull ↓, boost ↑ → stddev 4배
        let boosted = pool_selection_stddev(
            base,
            POST_ESCAPE_GLOBAL_BEST_PULL,
            POST_ESCAPE_EXPLORATION_BOOST,
        );
        let expected = base * POST_ESCAPE_EXPLORATION_BOOST / POST_ESCAPE_GLOBAL_BEST_PULL;
        assert!(
            (boosted - expected).abs() < 1.0e-6,
            "{boosted} != {expected}"
        );
        assert!(boosted > base);

        // 부스트가 끝나면 설정값으로 정확히 복귀한다.
        let normal = pool_selection_stddev(base, 1.0, 1.0);
        assert!((normal - base).abs() < 1.0e-6);
    }

    #[test]
    fn pool_stddev_stays_inside_the_safe_range() {
        let (lo, hi) = POOL_STDDEV_RANGE;
        for (base, pull, boost) in [
            (0.0, 1.0, 1.0),
            (f32::NAN, 0.5, 2.0),
            (0.25, 0.0, 2.0),
            (0.25, 0.5, f32::INFINITY),
            (10.0, 0.001, 100.0),
            (-1.0, -1.0, -1.0),
        ] {
            let s = pool_selection_stddev(base, pull, boost);
            assert!(
                s.is_finite() && s >= lo && s <= hi,
                "stddev out of range: {s}"
            );
        }
    }

    #[test]
    fn pool_stays_sorted_by_loss() {
        let mut losses: Vec<f32> = Vec::new();
        for loss in [3.0f32, 1.0, 7.0, 0.5, 2.0] {
            let idx = pool_insert_index(&losses, loss);
            losses.insert(idx, loss);
        }
        assert_eq!(losses, vec![0.5, 1.0, 2.0, 3.0, 7.0]);
        // 새 최선 해는 언제나 앞자리를 차지한다 (ExplImproving 보고 조건).
        assert_eq!(pool_insert_index(&losses, 0.1), 0);
        assert_eq!(pool_insert_index(&losses, 100.0), losses.len());
    }

    #[test]
    fn pool_index_never_escapes_the_pool() {
        assert_eq!(pool_index_from_sample(0.0, 8), 0);
        assert_eq!(pool_index_from_sample(0.999, 8), 7);
        assert_eq!(pool_index_from_sample(5.0, 8), 7);
        assert_eq!(pool_index_from_sample(-0.5, 8), 4);
        assert_eq!(pool_index_from_sample(f32::NAN, 8), 0);
        assert_eq!(pool_index_from_sample(0.5, 0), 0);
    }

    #[test]
    fn defaults_match_the_documented_pipeline() {
        let cfg = AdaptiveEscapeConfig::default();
        assert!(cfg.enable_conflict_burst);
        assert!(cfg.enable_swarm_diversification);
        assert_eq!(cfg.max_infeasible_pool, DEFAULT_MAX_INFEASIBLE_POOL);
        assert_eq!(cfg.max_infeasible_pool, 32);
        assert_eq!(cfg.stagnation.efficiency_eps, STAGNATION_EFFICIENCY_EPS);

        let state = AdaptiveEscapeState::default();
        assert_eq!(state.stagnation, 0);
        assert_eq!(state.escape_level, 0);
    }
}
