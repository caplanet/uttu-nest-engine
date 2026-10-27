//! NFP-LBF 파이프라인의 최상위 진입점 — Best Initial Nest 탐색.
//!
//! ```text
//! NFP Cache
//!    ↓
//! Multiple Item Ordering
//!    ↓
//! NFP 기반 Sequential Placement
//!    ↓
//! Strip Length 최소 후보 우선
//!    ↓
//! LB + Contact + Void 평가
//!    ↓
//! Best Initial Nest
//!    ↓
//! Swarm / Local Improvement
//! ```
//!
//! NFP-LBF는 **최종 optimizer가 아니라 강력한 constructive heuristic**이다.
//!
//! ```text
//! NFP-LBF를 최종 optimizer라기보다 강력한 constructive heuristic으로 사용한다.
//! 현재 Swarm 결과가 초기 배치에 민감하다면, 랜덤 초기 배치 대신 이 NFP-LBF 결과를
//! seed로 넣는 것만으로도 꽤 큰 차이가 날 가능성이 있다.
//! ```
//!
//! # 예산
//!
//! ```text
//! | 단계                          | 예상 시간   |
//! |-------------------------------|------------|
//! | NFP Cache 생성, 12,090 pair   | 1~5초      |
//! | Multiple Item Ordering 생성   | <0.1초     |
//! | NFP Sequential Placement 1회  | 0.05~0.5초 |
//! | LB + Contact + Void 평가 포함 | 0.1~1초    |
//! | 50개 ordering 평가            | 5~30초     |
//! | Best Initial Nest 선택        | 무시 가능   |
//! ```
//!
//! ```text
//! NFP cold cache          2.0 sec
//! LBF ordering 1회       100~200 ms
//! 20 orderings            2~4 sec
//! 50 orderings            5~10 sec
//! 100 orderings          10~20 sec
//! ```
//!
//! > **Cold start 10초 이내에 NFP 캐시 + 30~50개 LBF ordering 평가 완료**
//!
//! # Prefix 재사용
//!
//! ```text
//! 156개를 매 ordering마다 처음부터 전부 재배치할 필요는 없다.
//!
//! Base ordering → NFP-LBF → 좋은 결과 확보
//! 이후 Swap 2 items / Move 1 item / Block reorder
//!     ↓
//! 부분적으로 달라진 suffix만 재배치
//! ```

use std::time::Duration;

use jagua_rs::Instant;
use jagua_rs::geometry::DTransformation;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution};
use log::warn;
use rand::rngs::Xoshiro256PlusPlus;

use crate::nfp_lbf::cache::{NfpCacheStats, NfpLbfCache, PreparedLibrary};
use crate::nfp_lbf::ordering::build_orderings;
use crate::nfp_lbf::placement::{SequentialPlacement, SequentialPlacer};
use crate::nfp_lbf::score::PlacementScoreWeights;
use crate::nfp_lbf::{DEFAULT_CACHE_BUDGET_MS, DEFAULT_N_ORDERINGS, DEFAULT_TOTAL_BUDGET_MS};

/// 시드 해를 만들 때 strip 폭에 얹는 상대 여유. `fit_strip()`이 곧바로 되돌리므로
/// 경계 위 조각이 컨테이너 밖으로 판정되는 것만 막으면 된다.
const SEED_STRIP_RELATIVE_SLACK: f32 = 1.0e-3;

/// 시드 해를 만들 때 strip 폭에 얹는 절대 여유(mm).
const SEED_STRIP_ABSOLUTE_SLACK: f32 = 1.0e-3;

/// NFP-LBF 초기 배치 탐색 설정.
#[derive(Clone, Debug)]
pub struct NfpLbfConfig {
    /// 평가할 ordering 수.
    pub n_orderings: usize,
    /// NFP 콜드 캐시 pre-warm 예산.
    pub cache_budget: std::time::Duration,
    /// 캐시 생성을 포함한 전체 예산.
    pub total_budget: std::time::Duration,
    /// `Score(p) = w1·L(p) + w2·Waste(p) − w3·Contact(p) + w4·Y(p)`의 가중치.
    pub weights: PlacementScoreWeights,
    /// 시도할 회전 상태(도). 문서 기본 조건은 "일단 회전 필요없고 쌍별 페어로만".
    pub rotations: Vec<i64>,
    /// 외부에서 지정한 배치 순서. `Some`이면 [`OrderingStrategy::CustomGarment`]로 쓰인다.
    ///
    /// [`OrderingStrategy::CustomGarment`]: crate::nfp_lbf::OrderingStrategy::CustomGarment
    pub custom_order: Option<Vec<usize>>,
    /// 순서가 뒷부분만 달라졌을 때 앞의 배치를 그대로 재사용한다.
    pub enable_prefix_reuse: bool,
    /// NFP 라이브러리 전용 오목 단순화 강도. `0.0`이면 비활성.
    /// [`crate::nfp_lbf::DEFAULT_CONCAVE_SIMPLIFY_TOLERANCE`] 참조.
    pub concave_simplify_tolerance: f32,
    /// 아이템 하나당 feasible 판정을 시도할 raw 후보 수 상한.
    ///
    /// **이 값이 곧 초기 배치 품질이다.** 후보 수가 배치 진행에 따라 폭증하는데 예산이
    /// 고정이면 후기 아이템이 후보의 극히 일부만 보고 자리를 정한다. 근거와 실측표는
    /// [`crate::nfp_lbf::DEFAULT_MAX_CANDIDATE_PROBES`] 참조.
    pub max_candidate_probes: usize,
    /// NFP 초기 배치 전략 7종. 전부 꺼져 있으면 아래 경로가 한 비트도 바뀌지 않는다.
    ///
    /// [`crate::nfp_strategy`] 참조.
    pub strategy: crate::nfp_strategy::NfpStrategyConfig,
    /// 탐침 예산을 남은 시간에 맞춰 ordering 사이에서 **줄인다**. 기본 **꺼짐**.
    ///
    /// # 왜 기본이 꺼짐인가
    ///
    /// **결과가 벽시계에 의존하게 된다.** 조절 판정이 `Instant::now()` 실측을 쓰므로
    /// 같은 입력이 머신·부하에 따라 다른 배치를 낸다. 이 코드베이스는 회귀를 비트
    /// 단위 비교로 잡으므로 그 순간 모든 실측치가 비교 불가능해진다.
    ///
    /// # 그리고 올리지는 못한다
    ///
    /// 호출부가 placer 를 [`max_candidate_probes`](Self::max_candidate_probes)로 시작시키고
    /// 같은 값을 상한으로 넘기므로, [`adjust_probe_budget`]의 "2배" 분기는 처음부터
    /// 도달할 수 없다. 실제 동작은 **시간이 모자랄 때 정밀도를 깎는 안전밸브** 하나뿐이다.
    ///
    /// 그 안전밸브가 실제로 이득인지는 **재지 않았다** — 12 ordering 을 낮은 정밀도로
    /// 도는 것과 3 ordering 을 높은 정밀도로 도는 것 중 무엇이 나은지 모른다.
    /// 켜기 전에 A/B 로 확인할 것.
    pub adaptive_probe_budget: bool,
}

impl Default for NfpLbfConfig {
    fn default() -> Self {
        Self {
            n_orderings: DEFAULT_N_ORDERINGS,
            cache_budget: Duration::from_millis(DEFAULT_CACHE_BUDGET_MS),
            total_budget: Duration::from_millis(DEFAULT_TOTAL_BUDGET_MS),
            weights: PlacementScoreWeights::default(),
            // 문서: "일단 회전 필요없고 쌍별 페어로만".
            rotations: vec![0],
            custom_order: None,
            enable_prefix_reuse: true,
            concave_simplify_tolerance: crate::nfp_lbf::DEFAULT_CONCAVE_SIMPLIFY_TOLERANCE,
            max_candidate_probes: crate::nfp_lbf::DEFAULT_MAX_CANDIDATE_PROBES,
            strategy: crate::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG,
            adaptive_probe_budget: false,
        }
    }
}

/// 남은 시간에 맞춰 다음 ordering 의 탐침 예산을 정한다.
///
/// # 경고 — 결과가 벽시계에 의존하게 된다
///
/// 판정에 `Instant::now()` 실측이 들어가므로, 이 함수를 쓰는 순간 같은 입력이
/// 머신·부하에 따라 **다른 배치**를 낸다. 그래서
/// [`NfpLbfConfig::adaptive_probe_budget`]은 기본 꺼짐이다.
///
/// # 경고 — 실제로는 내리기만 한다
///
/// 호출부가 placer 를 `cfg.max_candidate_probes` 로 시작시키고 같은 값을 `ceiling`
/// 으로 넘긴다. 따라서 아래 "2배" 분기는 **처음부터 도달할 수 없다**. 올리려면
/// 시작값을 상한보다 낮게 잡아야 하는데, 그건 기본 동작을 바꾸는 변경이라
/// 측정 없이는 하지 않았다.
///
/// # 왜 필요한가
///
/// 탐침 예산은 곧 배치 품질이다(실측: 1024 -> 68.02%, 4096 -> 71.83%, 8192 -> 72.08%).
/// 그런데 4096 은 **여성WSjk 156-패널에서 잰 포화점**이지 보편 상수가 아니다.
///
/// * 조각이 적은 인스턴스는 후보 자체가 4096 에 못 미쳐 예산이 무의미하다.
/// * 조각이 많거나 오목한 인스턴스는 4096 으로도 커버리지가 모자란다.
/// * 예산 시간이 넉넉하면 더 정밀하게 풀 수 있는데 그냥 남긴다.
///
/// 그래서 **"주어진 시간을 쓴다"** 로 바꾼다. 한 ordering 을 돌 때마다 걸린 시간을
/// 남은 예산·남은 ordering 수와 견줘 다음 예산을 올리거나 내린다.
///
/// # 규칙
///
/// ```text
/// 목표 = 남은 시간 / 남은 ordering 수
/// 걸린 시간 < 목표 x 0.5   ->  예산 2배 (상한까지)
/// 걸린 시간 > 목표 x 1.5   ->  예산 1/2 (하한까지)
/// 그 사이                  ->  그대로
/// ```
///
/// 2배/절반으로 움직이는 이유는 품질이 예산에 **로그에 가깝게** 반응하기 때문이다
/// (1024 -> 2048 이 +1.88%p, 2048 -> 4096 이 +1.93%p, 4096 -> 8192 가 +0.25%p).
/// 선형으로 조금씩 움직이면 수렴 전에 ordering 이 끝난다.
///
/// 히스테리시스(0.5 / 1.5)를 둔 이유는 목표 근처에서 매번 뒤집히면 ordering 마다
/// 다른 정밀도로 풀려 결과 비교가 무의미해지기 때문이다.
fn adjust_probe_budget(
    current: usize,
    last_ordering: Duration,
    remaining: Duration,
    remaining_orderings: usize,
    ceiling: usize,
) -> usize {
    if remaining_orderings == 0 || last_ordering.is_zero() {
        return current;
    }
    let target = remaining / (remaining_orderings as u32).max(1);
    if target.is_zero() {
        // 시간이 없다 — 남은 ordering 은 어차피 못 돈다. 그대로 둔다.
        return current;
    }

    if last_ordering.mul_f32(2.0) < target {
        // 시간이 남아돈다 — 정밀도를 올린다.
        (current * 2).min(ceiling)
    } else if last_ordering.mul_f32(2.0) > target.mul_f32(3.0) {
        // 예산을 넘기고 있다 — 남은 ordering 을 아예 못 도는 것보다 낫다.
        (current / 2).max(MIN_ADAPTIVE_PROBES)
    } else {
        current
    }
}

/// 자동 조절이 내려갈 수 있는 하한.
///
/// 이보다 낮추면 후기 아이템이 후보를 거의 못 봐서 배치가 무너진다
/// (512 에서 strip 7971.75, 1024 에서 7686.44).
///
/// # 이 수치의 측정 조건
///
/// **모른다.** 인스턴스·ordering 수·feature·리비전이 어디에도 적혀 있지 않고, 같은
/// 예산에 대한 저장소의 주 실측표(`DEFAULT_MAX_CANDIDATE_PROBES` 문서)는 1024 에서
/// strip **7479.09** 라고 적는다 — 여기 적힌 7686.44 와 맞지 않는다. 둘 중 하나(또는
/// 둘 다)가 지금 코드의 값이 아니다.
///
/// 그래도 이 상수를 지금 바꾸지 않는다. **도달할 수 없는 코드**이기 때문이다 —
/// `adaptive_probe_budget` 이 기본 꺼짐이고, 켜더라도 호출부가 placer 를
/// `max_candidate_probes` 로 시작시키고 같은 값을 상한으로 넘기므로 예산은 내려가기만
/// 한다(그 두 사실은 `adaptive_probe_budget_is_off_by_default...` 와
/// `in_the_real_call_shape_the_budget_can_only_go_down` 이 못박고 있다).
///
/// 되살릴 때는 **측정 조건과 함께** 새로 재고 이 문단을 지울 것.
pub const MIN_ADAPTIVE_PROBES: usize = 1024;

/// 탐색 결과 요약. 로그/벤치 리포트에 그대로 실린다.
#[derive(Clone, Debug, Default)]
pub struct NfpLbfReport {
    /// 마지막 시점의 NFP 캐시 통계(pre-warm + 배치 중 miss 계산 누적).
    pub cache: NfpCacheStats,
    /// 실제로 평가한 ordering 수. 예산 초과로 중단되면 요청 수보다 작다.
    pub orderings_evaluated: usize,
    /// 최선 해의 strip 길이(mm).
    pub best_strip_length: f32,
    /// 최선 해를 만든 ordering의 라벨.
    pub best_ordering_label: String,
    /// 최선 해의 밀도(0.0~1.0).
    pub best_density: f32,
    /// 전체 소요 시간(ms).
    pub elapsed_ms: u128,
    /// prefix 재사용이 실제로 일어난 횟수.
    pub prefix_reuse_hits: usize,
    /// 켜진 전략 라벨. 비어 있으면 기존 경로로 돌았다는 뜻이다.
    pub strategies: Vec<&'static str>,
    /// ②가 실제로 붙인 쌍의 수(최선 해 기준).
    pub clusters_applied: usize,
    /// ⑦이 실제로 옮긴 조각 수(최선 해 기준).
    pub pieces_slid: usize,
}

/// Best Initial Nest — Swarm / Local Improvement에 넘길 시드.
pub struct BestInitialNest {
    /// 검증을 통과한 시드 해.
    pub solution: SPSolution,
    /// 그 해를 만든 순차 배치 결과.
    pub placement: SequentialPlacement,
    /// 탐색 요약.
    pub report: NfpLbfReport,
}

/// 호스트의 Init 취소 요청. 시드 탐색의 모든 예산 검사가 이것을 함께 본다.
#[inline]
fn init_cancelled() -> bool {
    crate::config::runtime::init_cancel_requested()
}

/// 여러 ordering을 NFP 순차 배치로 평가해 가장 좋은 초기 배치를 만든다.
///
/// 실패는 전부 `None` + [`log::warn!`]이다. 어떤 경로에서도 panic하지 않는다.
pub fn build_best_initial_nest(
    instance: &SPInstance,
    cfg: &NfpLbfConfig,
    rng: &mut Xoshiro256PlusPlus,
) -> Option<BestInitialNest> {
    // 호스트가 Init 을 취소했으면 시드를 만들지 않는다 — 호출자가 취소를 처리한다.
    if init_cancelled() {
        return None;
    }
    let started = Instant::now();
    let deadline = started + cfg.total_budget;

    let strategy = cfg.strategy.normalized();

    // ── 1) 도형 라이브러리 ──────────────────────────────────────────────
    //
    // ① 인스턴스 회전은 **라이브러리를 만들기 전에** 정해야 한다. 라이브러리에 없는
    // 회전은 배치에 쓸 수 없기 때문이다.
    let rotations = crate::nfp_strategy::pipeline::resolve_rotations(
        instance,
        &strategy,
        &cfg.rotations,
    );
    let Some(library) =
        PreparedLibrary::build(instance, &rotations, cfg.concave_simplify_tolerance)
    else {
        warn!("[NFP-LBF] prepared library build failed; skipping the NFP seed");
        return None;
    };

    // ── 2) NFP 콜드 캐시 pre-warm ──────────────────────────────────────
    let mut cache = NfpLbfCache::new(library);
    // pre-warm 이 전체 예산을 넘기면 ordering 평가 시간이 남지 않는다.
    let prewarm = cache.prewarm(cfg.cache_budget.min(cfg.total_budget));
    let mut report = NfpLbfReport {
        cache: prewarm,
        ..Default::default()
    };

    // ── 3) Multiple Item Ordering ──────────────────────────────────────
    //
    // 켠 전략에 따라 배치 1회 비용이 몇 배가 되므로 ordering 수를 그만큼 줄인다.
    // 빔 폭 4 + best-fit 12 면 48배다 — 50개를 그대로 돌리면 예산이 폭발한다.
    let n_orderings = crate::nfp_strategy::pipeline::scale_orderings(cfg.n_orderings, &strategy);
    let orderings = build_orderings(instance, n_orderings, rng, cfg.custom_order.as_deref());
    if orderings.is_empty() {
        warn!("[NFP-LBF] no item ordering produced; skipping the NFP seed");
        return None;
    }

    let strip_height = instance.base_strip.fixed_height;
    if !strip_height.is_finite() || strip_height <= 0.0 {
        warn!("[NFP-LBF] invalid strip height {strip_height}; skipping the NFP seed");
        return None;
    }
    let strip_width_cap = worst_case_strip_width(instance, strip_height);

    // ── 4~6) ordering 평가 + prefix 재사용 + 예산 관리 ──────────────────
    // 아이템별 허용 회전 — 결(grain) 제약이 있는 조각은 자세가 고정되고,
    // 없는 조각만 부유조각으로 분리된다.
    let item_rotations = if strategy.instance_rotations {
        crate::nfp_strategy::rotations::item_rotation_table(instance, &rotations)
    } else {
        item_allowed_rotations(instance, &cfg.rotations)
    };
    let mut placer = SequentialPlacer::with_item_rotations(
        &mut cache,
        strip_height,
        strip_width_cap,
        cfg.weights,
        item_rotations,
    )
    .with_probe_budget(cfg.max_candidate_probes);

    // (직전에 평가한 ordering의 인덱스, 그 배치 결과)
    let mut prev: Option<(usize, SequentialPlacement)> = None;
    // 파생 ordering 은 모두 ordering 0 을 한 번 변형한 것이라, 직전 ordering 보다
    // ordering 0 과의 공통 접두가 대체로 길다(기댓값 ≈0.75·n vs ≈0.67·n).
    let mut first: Option<(usize, SequentialPlacement)> = None;
    let mut best: Option<(usize, SequentialPlacement)> = None;

    // ── 전략 경로 ───────────────────────────────────────────────────────
    //
    // 하나라도 켜져 있으면 배치 방식 자체가 달라지므로 prefix 재사용을 쓸 수 없다
    // (빔·best-fit 은 "앞 k개를 그대로 둔다" 가 성립하지 않는다).
    if strategy.any_enabled() {
        return strategy_path(
            instance,
            cfg,
            &strategy,
            &orderings,
            &mut placer,
            deadline,
            report,
            started,
        );
    }

    for (idx, ordering) in orderings.iter().enumerate() {
        // 최소 1개는 반드시 평가한다. 호스트의 Init 취소는 예산과 같게 취급한다.
        if idx > 0 && (Instant::now() >= deadline || init_cancelled()) {
            break;
        }

        // prefix 재사용: 직전 ordering과 공통 접두가 있고, 그 구간이 전부 배치에
        // 성공했을 때에만 suffix만 다시 배치한다.
        // 직전 ordering 과 ordering 0 중 실제로 재사용할 수 있는 접두가 긴 쪽을 쓴다.
        let (prefix, prefix_len) = if cfg.enable_prefix_reuse {
            [prev.as_ref(), first.as_ref()]
                .into_iter()
                .flatten()
                .map(|(i, p)| {
                    let requested = orderings[*i].common_prefix_len(ordering);
                    (p, valid_prefix_len(&ordering.sequence, p, requested))
                })
                .max_by_key(|&(_, usable)| usable)
                .filter(|&(_, usable)| usable > 0)
                .map_or((None, 0), |(p, usable)| (Some(p), usable))
        } else {
            (None, 0)
        };
        if prefix_len > 0 {
            report.prefix_reuse_hits += 1;
        }

        let ordering_started = Instant::now();
        let placement =
            placer.place_sequence(&ordering.sequence, prefix, prefix_len, Some(deadline));
        report.orderings_evaluated += 1;

        // 남은 시간에 맞춰 다음 ordering 의 정밀도를 조절한다.
        if cfg.adaptive_probe_budget {
            let now = Instant::now();
            let remaining = deadline.saturating_duration_since(now);
            let next = adjust_probe_budget(
                placer.probe_budget(),
                now.saturating_duration_since(ordering_started),
                remaining,
                orderings.len().saturating_sub(idx + 1),
                cfg.max_candidate_probes,
            );
            if next != placer.probe_budget() {
                placer.set_probe_budget(next);
            }
        }

        // 5) 미배치가 적은 쪽이 무조건 우선, 동수면 strip length가 짧은 쪽.
        let replace = match best.as_ref() {
            None => true,
            Some((_, incumbent)) => is_better(
                (placement.unplaced.len(), placement.strip_length),
                (incumbent.unplaced.len(), incumbent.strip_length),
            ),
        };
        if replace {
            best = Some((idx, placement.clone()));
        }
        if idx == 0 && cfg.enable_prefix_reuse {
            first = Some((idx, placement.clone()));
        }
        prev = Some((idx, placement));
    }

    // 캐시 통계는 배치 중 발생한 miss 계산까지 반영한 최종값으로 갱신한다.
    report.cache = placer.cache().stats();
    drop(placer);

    let Some((best_idx, best_placement)) = best else {
        warn!("[NFP-LBF] no ordering could be evaluated within the budget");
        return None;
    };

    // ── 7) SPSolution 변환 ─────────────────────────────────────────────
    let Some(solution) = materialize(instance, &best_placement) else {
        return None;
    };

    // ── 8) 검증 — 잘못된 seed를 optimizer에 넘기지 않는다 ───────────────
    if let Some(err) = crate::structure::pipeline::solution_validation_error(instance, &solution) {
        warn!("[NFP-LBF] rejected an invalid initial nest: {err:?}");
        return None;
    }

    // ── 9) 리포트 마무리 ───────────────────────────────────────────────
    report.best_strip_length = solution.strip_width();
    report.best_ordering_label = orderings[best_idx].label.clone();
    report.best_density = solution.density();
    report.elapsed_ms = started.elapsed().as_millis();

    Some(BestInitialNest {
        solution,
        placement: best_placement,
        report,
    })
}

/// 전략이 켜졌을 때의 배치 경로.
///
/// 기존 경로와 갈리는 점은 셋이다.
///
/// 1. **prefix 재사용을 쓰지 않는다.** 빔과 best-fit 은 "앞 k개를 그대로 둔다" 가
///    성립하지 않는다 — 뒤가 달라지면 앞의 선택도 달라져야 의미가 있다.
/// 2. **클러스터를 한 번만 만든다.** ordering 과 무관한 전처리라 ordering 마다
///    다시 만들면 그 비용만 `ordering 수`배가 된다.
/// 3. 결과를 [`SequentialPlacement`]로 합류시켜 **검증 경로를 똑같이 통과**시킨다.
#[allow(clippy::too_many_arguments)]
fn strategy_path(
    instance: &SPInstance,
    cfg: &NfpLbfConfig,
    strategy: &crate::nfp_strategy::NfpStrategyConfig,
    orderings: &[crate::nfp_lbf::ItemOrdering],
    placer: &mut SequentialPlacer<'_>,
    deadline: Instant,
    mut report: NfpLbfReport,
    started: Instant,
) -> Option<BestInitialNest> {
    use crate::nfp_strategy::{StrategyContext, cluster, pipeline};

    report.strategies = strategy.enabled_labels();

    let item_rotations = placer_item_rotations(placer, instance);
    let mut ctx = StrategyContext::new(placer, *strategy, item_rotations, Some(deadline));

    // ② 클러스터링 — ordering 과 무관한 전처리다. 한 번만 만든다.
    let clusters = if strategy.pairwise_clustering {
        cluster::build_clusters(&mut ctx, instance, Some(deadline))
    } else {
        Vec::new()
    };

    let mut best: Option<(usize, crate::nfp_strategy::pipeline::StrategyPlacement)> = None;

    for (idx, ordering) in orderings.iter().enumerate() {
        // 최소 1개는 반드시 평가한다 — 하나도 못 돌면 시드 자체가 없다.
        if idx > 0 && (Instant::now() >= deadline || init_cancelled()) {
            break;
        }
        let ordering_started = Instant::now();
        let placement = pipeline::place_with_strategies(&mut ctx, &ordering.sequence, &clusters);
        report.orderings_evaluated += 1;

        // 기본 경로와 같은 되먹임. 전략 경로는 배치 1회가 훨씬 비싸므로 여기가 더 중요하다.
        if cfg.adaptive_probe_budget {
            let now = Instant::now();
            let remaining = deadline.saturating_duration_since(now);
            let next = adjust_probe_budget(
                ctx.placer.probe_budget(),
                now.saturating_duration_since(ordering_started),
                remaining,
                orderings.len().saturating_sub(idx + 1),
                cfg.max_candidate_probes,
            );
            if next != ctx.placer.probe_budget() {
                ctx.placer.set_probe_budget(next);
            }
        }

        let replace = match best.as_ref() {
            None => true,
            Some((_, incumbent)) => is_better(
                (placement.unplaced.len(), placement.strip_length as f32),
                (incumbent.unplaced.len(), incumbent.strip_length as f32),
            ),
        };
        if replace {
            best = Some((idx, placement));
        }

        // ⑧ 누적 배치는 ordering 의 **순서**를 쓰지 않는다 — 같은 다중집합이면 결과가
        // 같으므로 나머지를 도는 것은 같은 계산의 반복이다. 예산을 아껴 ⑦/⑨에 준다.
        if strategy.ordering_is_only_a_multiset() {
            break;
        }
    }

    report.cache = ctx.placer.cache().stats();

    let Some((best_idx, best_placement)) = best else {
        warn!("[NFP-LBF] 전략 경로에서 평가된 ordering 이 하나도 없다");
        return None;
    };
    report.clusters_applied = best_placement.clusters_applied;
    report.pieces_slid = best_placement.pieces_slid;

    let sequential = best_placement.into_sequential();
    let solution = materialize(instance, &sequential)?;
    if let Some(err) = crate::structure::pipeline::solution_validation_error(instance, &solution) {
        warn!("[NFP-LBF] 전략 경로가 만든 초기 배치가 검증에 실패했다: {err:?}");
        return None;
    }

    report.best_strip_length = solution.strip_width();
    report.best_ordering_label = orderings
        .get(best_idx)
        .map(|o| o.label.clone())
        .unwrap_or_default();
    report.best_density = solution.density();
    report.elapsed_ms = started.elapsed().as_millis();
    let _ = cfg;

    Some(BestInitialNest {
        solution,
        placement: sequential,
        report,
    })
}

/// 오라클이 들고 있는 아이템별 회전표를 그대로 복사한다.
///
/// [`StrategyContext`]가 자기 표를 갖는 이유는 오라클을 가변 대여 중에도 회전을
/// 읽어야 하기 때문이다. 오라클의 표를 그대로 쓰므로 두 쪽이 어긋날 일은 없다.
///
/// [`StrategyContext`]: crate::nfp_strategy::StrategyContext
fn placer_item_rotations(placer: &SequentialPlacer<'_>, instance: &SPInstance) -> Vec<Vec<i64>> {
    let max_id = instance.items.iter().map(|(i, _)| i.idx()).max().unwrap_or(0);
    (0..=max_id)
        .map(|id| placer.allowed_rotations(id))
        .collect()
}

/// `item_id`별 허용 회전(도) 목록을 만든다.
///
/// 인스턴스가 선언한 `allowed_rotation`과 설정의 `rotations`를 교집합한다.
/// 결과 길이가 2 이상인 아이템이 **부유조각**(결·위치 제약 없음)이다.
///
/// * `RotationRange::None` -> `[0]` (자세 고정)
/// * `RotationRange::Discrete(set)` -> set을 도 단위로 반올림해 `rotations`와 교집합
/// * `RotationRange::Continuous` -> `rotations` 전체 (자유 회전)
fn item_allowed_rotations(instance: &SPInstance, rotations: &[i64]) -> Vec<Vec<i64>> {
    use jagua_rs::geometry::geo_enums::RotationRange;

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
        // 방어적 하한. 최소한 strip 높이만큼은 확보한다.
        strip_height
    }
}

/// `(미배치 수, strip length)` 사전식 비교. 미배치가 적은 쪽이 무조건 우선이다.
#[inline]
fn is_better(candidate: (usize, f32), incumbent: (usize, f32)) -> bool {
    if candidate.0 != incumbent.0 {
        return candidate.0 < incumbent.0;
    }
    candidate.1 < incumbent.1
}

/// 재사용해도 되는 prefix 길이.
///
/// `placement.pieces`는 배치 순서 그대로지만 미배치 아이템은 건너뛰므로, 이전
/// ordering 과의 공통 접두 `requested` 가 그대로 `pieces` 의 접두라는 보장이 없다.
/// (`unplaced` 는 아이템 id 이지 시퀀스 인덱스가 아니다.) 그래서 요청 길이를 그대로
/// 믿지 않고 `pieces[k].item_id == sequence[k]` 가 **실제로 1:1 일치하는 구간까지만**
/// 잘라서 넘긴다 — `exploration::usable_prefix_len` 과 같은 의미다.
#[inline]
fn valid_prefix_len(
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

/// 도(i64) → 라디안(f32). `[0, 2π)`로 정규화한다.
#[inline]
fn rotation_to_radians(degrees: i64) -> f32 {
    let normalized = degrees.rem_euclid(360);
    (normalized as f64).to_radians() as f32
}

/// 시드 해를 만들 때 쓸 strip 폭. `fit_strip()`이 바로 줄여준다.
#[inline]
fn seed_strip_width(strip_length: f32) -> f32 {
    strip_length * (1.0 + SEED_STRIP_RELATIVE_SLACK) + SEED_STRIP_ABSOLUTE_SLACK
}

/// 순차 배치 결과를 [`SPSolution`]으로 굳힌다.
fn materialize(instance: &SPInstance, placement: &SequentialPlacement) -> Option<SPSolution> {
    if placement.pieces.is_empty() {
        warn!("[NFP-LBF] best placement has no piece; skipping the NFP seed");
        return None;
    }
    if !placement.unplaced.is_empty() {
        warn!(
            "[NFP-LBF] {} piece(s) could not be placed; the seed will be incomplete",
            placement.unplaced.len()
        );
    }
    if !placement.strip_length.is_finite() || placement.strip_length <= 0.0 {
        warn!(
            "[NFP-LBF] invalid strip length {}; skipping the NFP seed",
            placement.strip_length
        );
        return None;
    }

    let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
    // 임시 문제의 컨테이너도 Deflate 되므로 실제 오른쪽 벽은 `seed_width - inset` 이다.
    // 수치 여유만으로는 인셋보다 작아서, 짧은 prefix 에서는 맨 오른쪽 조각이 삽입
    // 시점에 컨테이너 밖에 놓이고 뒤이은 `fit_strip` 이 그 중간 상태를 굳혀 버린다.
    let right_inset = instance
        .base_strip
        .shape_modify_config
        .offset
        .unwrap_or(0.0)
        .max(0.0);
    prob.change_strip_width(seed_strip_width(placement.strip_length + right_inset)).expect(crate::dmsg!("strip width must be positive"));

    // `place_item`은 수요 카운터를 감소시킨다. 수요를 넘겨 넣으면 underflow로
    // panic하므로 남은 수요를 직접 센다.
    let mut remaining: Vec<usize> = instance.items.iter().map(|(_, qty)| *qty).collect();

    for piece in placement.pieces.iter() {
        let Some(left) = remaining.get_mut(piece.item_id) else {
            warn!("[NFP-LBF] unknown item id {}; piece skipped", piece.item_id);
            continue;
        };
        if *left == 0 {
            warn!(
                "[NFP-LBF] item {} exceeds its demand; piece skipped",
                piece.item_id
            );
            continue;
        }
        let (x, y) = piece.translation;
        if !x.is_finite() || !y.is_finite() {
            warn!(
                "[NFP-LBF] item {} has a non-finite translation; piece skipped",
                piece.item_id
            );
            continue;
        }
        *left -= 1;
        prob.place_item(SPPlacement {
            d_transf: DTransformation::new(rotation_to_radians(piece.rotation), (x, y)),
            item_idx: piece.item_id,
        });
    }

    if prob.n_placed_items() == 0 {
        warn!("[NFP-LBF] nothing was placed into the seed problem");
        return None;
    }

    prob.fit_strip().expect(crate::dmsg!("strip width must be positive"));
    Some(prob.save())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// **ordering 마다 «짧은 탐색 뒤의 폭» 을 잰다.** 시드는 초기 폭이 가장 짧은 ordering 으로
    /// 고르는데, 그 기준이 탐색 뒤의 폭을 얼마나 예측하는지 보는 조사용 하네스다.
    ///
    /// 노브: `AB_INPUT`(필수), `AB_OVERLAP_MM`(기본 0.4), `AB_PROBE_SECS`(기본 3),
    /// `AB_ORDERINGS`(기본 50), `AB_SEED`(기본 1), `AB_WORKERS`(기본 2),
    /// `AB_OUT_DIR`(각 초기 배치를 저장), `AB_PROBE_ORDERINGS`/`AB_STRATEGIES`/`AB_WEIGHTS`
    /// (`0` 이면 그 묶음을 건너뛴다), `AB_CUSTOM_ORDERS`(순서 파일: 한 줄에 `이름:id,id,...`).
    #[test]
    #[ignore]
    #[cfg(feature = "adaptive")]
    fn probe_width_per_ordering_on_an_input() {
        use crate::util::terminator::{BasicTerminator, Terminator};
        use rand::SeedableRng;

        let Ok(input) = std::env::var("AB_INPUT") else {
            println!("[A/B] AB_INPUT missing; skipped");
            return;
        };
        let env = |key: &str, default: &str| std::env::var(key).unwrap_or_else(|_| default.into());
        let overlap: f32 = env("AB_OVERLAP_MM", "0.4").parse().unwrap();
        let probe_secs: f32 = env("AB_PROBE_SECS", "3").parse().unwrap();
        let n_orderings: usize = env("AB_ORDERINGS", "50").parse().unwrap();
        let seed: u64 = env("AB_SEED", "1").parse().unwrap();
        let workers: usize = env("AB_WORKERS", "2").parse().unwrap();

        let c = crate::config::DEFAULT_SWARM_CONFIG;
        let (ext, _) = crate::util::io::read_spp_input(std::path::Path::new(&input)).unwrap();
        let instance = crate::util::io::import_spp_instance(
            &ext,
            c.cde_config,
            c.poly_simpl_tolerance,
            (overlap > 0.0).then_some(-overlap),
            c.narrow_concavity_cutoff_ratio,
        )
        .unwrap();

        let cfg = NfpLbfConfig { n_orderings, ..NfpLbfConfig::default() };
        let library =
            PreparedLibrary::build(&instance, &cfg.rotations, cfg.concave_simplify_tolerance)
                .unwrap();
        let mut cache = NfpLbfCache::new(library);
        cache.prewarm(cfg.cache_budget);
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(crate::protected_seed::nfp_lbf(seed));
        let orderings = build_orderings(&instance, n_orderings, &mut rng, None);
        let strip_height = instance.base_strip.fixed_height;
        let mut placer = SequentialPlacer::with_item_rotations(
            &mut cache,
            strip_height,
            worst_case_strip_width(&instance, strip_height),
            cfg.weights,
            item_allowed_rotations(&instance, &cfg.rotations),
        )
        .with_probe_budget(cfg.max_candidate_probes);

        let mut sep_config = c.expl_cfg.separator_config;
        sep_config.n_workers = workers;
        let out_dir = std::env::var("AB_OUT_DIR").ok();
        if let Some(dir) = &out_dir {
            std::fs::create_dir_all(dir).unwrap();
        }
        // 초기 배치 하나를 저장하고(전체 실행의 warm start 로 쓴다) 짧은 탐색 뒤의 폭을 잰다.
        let probe = |name: &str, solution: &SPSolution| -> f32 {
            if let Some(dir) = &out_dir {
                let ext = jagua_rs::probs::spp::io::export(solution, *crate::EPOCH);
                let path = std::path::Path::new(dir).join(format!("{name}.json"));
                std::fs::write(path, serde_json::to_string(&ext).unwrap()).unwrap();
            }
            let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
            prob.restore(solution);
            let mut sep = crate::optimizer::separator::Separator::new(
                instance.clone(),
                prob,
                Xoshiro256PlusPlus::seed_from_u64(seed),
                sep_config,
            );
            let mut term = BasicTerminator::new();
            term.new_timeout(Duration::from_secs_f32(probe_secs));
            crate::optimizer::explore_mods::exploration_phase_adaptive(
                &instance, &mut sep, &mut None, &term, &c.expl_cfg,
            )
            .iter()
            .map(|s| s.strip_width())
            .min_by(f32::total_cmp)
            .unwrap_or(f32::NAN)
        };
        let report = |name: &str, solution: &SPSolution, probed: f32| {
            println!(
                "[A/B] {name:<22} init {:10.3} ({:.3}%)  probe {:10.3} ({:.3}%)",
                solution.strip_width(),
                solution.density() * 100.0,
                probed,
                solution.density() * solution.strip_width() / probed * 100.0,
            );
        };

        let mut rows = vec![];
        for (idx, ordering) in orderings.iter().enumerate() {
            if env("AB_PROBE_ORDERINGS", "1") == "0" {
                break;
            }
            let name = format!("ord{idx:02}_{}", ordering.label.replace('@', "-"));
            let placement = placer.place_sequence(&ordering.sequence, None, 0, None);
            if !placement.unplaced.is_empty() {
                println!("[A/B] {name:<22} unplaced {}", placement.unplaced.len());
                continue;
            }
            let Some(solution) = materialize(&instance, &placement) else { continue };
            if let Some(err) =
                crate::structure::pipeline::solution_validation_error(&instance, &solution)
            {
                println!("[A/B] {name:<22} invalid: {err:?}");
                continue;
            }
            let probed = probe(&name, &solution);
            report(&name, &solution, probed);
            rows.push((name, solution.strip_width(), probed));
        }
        // 사용자 지정 순서 — `AB_CUSTOM_ORDERS` 파일, 한 줄에 `이름:id,id,...`.
        // (예: 사이즈별 구역을 만드는 순서.)
        if let Ok(path) = std::env::var("AB_CUSTOM_ORDERS") {
            for line in std::fs::read_to_string(path).unwrap().lines() {
                let Some((name, ids)) = line.trim().split_once(':') else { continue };
                let sequence: Vec<usize> =
                    ids.split(',').map(|id| id.trim().parse().unwrap()).collect();
                let placement = placer.place_sequence(&sequence, None, 0, None);
                if !placement.unplaced.is_empty() {
                    println!("[A/B] {name:<22} unplaced {}", placement.unplaced.len());
                    continue;
                }
                let Some(solution) = materialize(&instance, &placement) else { continue };
                if let Some(err) =
                    crate::structure::pipeline::solution_validation_error(&instance, &solution)
                {
                    println!("[A/B] {name:<22} invalid: {err:?}");
                    continue;
                }
                let probed = probe(name, &solution);
                report(name, &solution, probed);
                rows.push((name.to_string(), solution.strip_width(), probed));
            }
        }
        drop(placer);

        // 초기 배치 전략 변형 — 각 전략이 고른 «최선 초기 배치» 하나씩.
        let base = crate::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG;
        let variants: Vec<(&str, crate::nfp_strategy::NfpStrategyConfig)> = vec![
            ("strat_contact", crate::nfp_strategy::NfpStrategyConfig { contact_maximization: true, ..base }),
            ("strat_pocket", crate::nfp_strategy::NfpStrategyConfig { pocket_first: true, ..base }),
            ("strat_contact_pocket", crate::nfp_strategy::NfpStrategyConfig { contact_maximization: true, pocket_first: true, ..base }),
            ("strat_bestfit", crate::nfp_strategy::NfpStrategyConfig { best_fit_selection: true, ..base }),
            ("strat_slide", crate::nfp_strategy::NfpStrategyConfig { sliding_compaction: true, ..base }),
            ("strat_accretion", crate::nfp_strategy::NfpStrategyConfig { accretion: true, ..base }),
            ("strat_reflow", crate::nfp_strategy::NfpStrategyConfig { tail_reflow: true, ..base }),
            ("strat_accr_reflow", crate::nfp_strategy::NfpStrategyConfig { accretion: true, tail_reflow: true, ..base }),
        ];
        if env("AB_STRATEGIES", "1") != "0" {
            for (name, strategy) in variants {
                let cfg_s = NfpLbfConfig { strategy, ..NfpLbfConfig::default() };
                let mut rng_s = Xoshiro256PlusPlus::seed_from_u64(crate::protected_seed::nfp_lbf(seed));
                match build_best_initial_nest(&instance, &cfg_s, &mut rng_s) {
                    Some(nest) => {
                        let probed = probe(name, &nest.solution);
                        report(name, &nest.solution, probed);
                        rows.push((name.to_string(), nest.solution.strip_width(), probed));
                    }
                    None => println!("[A/B] {name:<22} no seed produced"),
                }
            }
        }

        // 자리 고르기 규칙 변형 — 길이가 비슷한 후보 안에서 맞물림(접촉·낭비)으로 고른다.
        // 작은 조각이 «가장 왼쪽» 대신 «가장 잘 맞물리는 틈» 으로 간다.
        if env("AB_WEIGHTS", "1") != "0" {
            let w = cfg.weights;
            let weight_variants = [
                ("tie_5mm", PlacementScoreWeights { length_tie_mm: 5.0, ..w }),
                ("tie_50mm", PlacementScoreWeights { length_tie_mm: 50.0, ..w }),
                ("tie_200mm", PlacementScoreWeights { length_tie_mm: 200.0, ..w }),
                ("score_first", PlacementScoreWeights { score_first: true, ..w }),
            ];
            for (name, weights) in weight_variants {
                let cfg_w = NfpLbfConfig { weights, ..NfpLbfConfig::default() };
                let mut rng_w = Xoshiro256PlusPlus::seed_from_u64(crate::protected_seed::nfp_lbf(seed));
                match build_best_initial_nest(&instance, &cfg_w, &mut rng_w) {
                    Some(nest) => {
                        let probed = probe(name, &nest.solution);
                        report(name, &nest.solution, probed);
                        rows.push((name.to_string(), nest.solution.strip_width(), probed));
                    }
                    None => println!("[A/B] {name:<22} no seed produced"),
                }
            }
        }

        rows.sort_by(|a, b| a.2.total_cmp(&b.2));
        println!("[A/B] --- ranked by probe width ---");
        for (rank, (name, init, probed)) in rows.iter().enumerate() {
            println!("[A/B] #{:02} {name:<22} init {init:10.3}  probe {probed:10.3}", rank + 1);
        }
        let by_init = rows.iter().min_by(|a, b| a.1.total_cmp(&b.1)).unwrap();
        println!("[A/B] best by init : {} (init {:.3}, probe {:.3})", by_init.0, by_init.1, by_init.2);
    }

    #[test]
    fn default_config_matches_the_documented_targets() {
        let cfg = NfpLbfConfig::default();
        assert_eq!(cfg.n_orderings, DEFAULT_N_ORDERINGS);
        assert_eq!(
            cfg.cache_budget,
            Duration::from_millis(DEFAULT_CACHE_BUDGET_MS)
        );
        assert_eq!(
            cfg.total_budget,
            Duration::from_millis(DEFAULT_TOTAL_BUDGET_MS)
        );
        // 문서: "일단 회전 필요없고 쌍별 페어로만".
        assert_eq!(cfg.rotations, vec![0]);
        assert!(cfg.custom_order.is_none());
        assert!(cfg.enable_prefix_reuse);
        // 탐침 예산이 곧 초기 배치 품질이다. 1024 로 되돌리면 NFP-LBF 가 기존 LBF 에
        // 져서 시드가 항상 기각된다 (여성WSjk 156-패널: 1024 -> 68.02%, LBF 69.45%).
        assert_eq!(
            cfg.max_candidate_probes,
            crate::nfp_lbf::DEFAULT_MAX_CANDIDATE_PROBES
        );
        assert!(
            cfg.max_candidate_probes >= 4096,
            "4096 미만이면 후기 아이템의 후보 커버리지가 무너져 LBF 에 진다"
        );
    }

    /// 0 은 "지정 안 함"이므로 코어 기본값으로 떨어져야 한다.
    /// C ABI 가 `<=0 = 기본값` 규약이라 이 경로가 실제로 쓰인다.
    #[test]
    fn zero_probe_budget_falls_back_to_the_default() {
        let seed = crate::config::NfpLbfSeedConfig {
            max_candidate_probes: 0,
            ..crate::config::DEFAULT_NFP_LBF_SEED_CONFIG
        };
        assert_eq!(
            seed.to_config(crate::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG)
                .max_candidate_probes,
            crate::nfp_lbf::DEFAULT_MAX_CANDIDATE_PROBES
        );

        let explicit = crate::config::NfpLbfSeedConfig {
            max_candidate_probes: 8192,
            ..crate::config::DEFAULT_NFP_LBF_SEED_CONFIG
        };
        assert_eq!(
            explicit
                .to_config(crate::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG)
                .max_candidate_probes,
            8192
        );
    }

    /// 일곱 전략이 **실제로 호출되는지**를 소스 수준에서 고정한다.
    ///
    /// 이 코드베이스는 같은 결함을 두 번 겪었다. 설정과 UI 와 ABI 를 다 만들어 놓고
    /// **호출하는 줄 하나를 빠뜨려서** 기능이 통째로 죽어 있었다. 그때도 빌드는
    /// 통과했고 테스트도 통과했다 — 아무도 "이게 불리는가" 를 묻지 않았기 때문이다.
    ///
    /// 동작을 검증하는 테스트가 아니라 **배선이 끊기면 즉시 깨지는** 테스트다.
    /// 실제 품질은 벤치가, 각 전략의 정확성은 그 모듈의 단위 테스트가 본다.
    /// `search.rs` 의 **프로덕션 부분만** 돌려준다 (첫 `#[cfg(test)]` 앞까지).
    ///
    /// 아래 배선 단정은 원래 `include_str!("search.rs")` 전체를 읽었는데, 찾는 문자열
    /// 리터럴이 바로 그 단정문 안에 있으므로 **언제나 참**이었다 — 프로덕션 호출을
    /// 전부 지워도 통과한다. `pipeline.rs` 를 읽는 뒷부분 4개만 실효가 있었다.
    /// 테스트 모듈을 잘라내 자기참조를 끊는다.
    fn production_source() -> &'static str {
        let src = include_str!("search.rs");
        let cut = src
            .find("#[cfg(test)]")
            .expect(crate::dmsg!("테스트 모듈 경계(`#[cfg(test)]`)를 찾지 못했다"));
        &src[..cut]
    }

    /// 배선 탐침이 자기 단정문을 읽지 않는지 확인한다.
    #[test]
    fn the_strategy_wiring_probe_must_not_read_its_own_assertions() {
        let prod = production_source();
        assert!(
            !prod.contains("the_strategy_wiring_probe_must_not_read_its_own_assertions"),
            "프로덕션 절단이 테스트 모듈까지 포함하고 있다 — 배선 단정이 항상 참이 된다"
        );
        assert!(
            prod.contains("pub fn build_best_initial_nest"),
            "절단이 너무 앞에서 일어나 프로덕션 본문이 사라졌다"
        );
    }

    #[test]
    fn every_strategy_is_actually_reachable_from_this_file() {
        let src = production_source();
        for (entry, why) in [
            ("resolve_rotations", "① 인스턴스 회전 — 라이브러리 생성 전에 불려야 한다"),
            ("item_rotation_table", "① 아이템별 회전표"),
            ("build_clusters", "② 쌍별 클러스터링 (전처리)"),
            ("place_with_strategies", "③④⑤⑥ 배치 — 파이프라인 진입점"),
            ("scale_orderings", "전략 비용만큼 ordering 수를 줄인다"),
            ("strategy_path", "전략 경로 분기 자체"),
            ("any_enabled", "전부 꺼지면 기존 경로로 빠진다"),
        ] {
            assert!(
                src.contains(entry),
                "`{entry}` 가 search.rs 에서 사라졌다 — {why}.                  배선이 끊기면 WPF 에서 옵션을 켜도 아무 일도 일어나지 않는다."
            );
        }
        // ⑦ 슬라이딩은 파이프라인 안에서 불린다. 그 배선은 pipeline.rs 가 지킨다.
        let pipeline = include_str!("../nfp_strategy/pipeline.rs");
        assert!(
            pipeline.contains("sliding::compact"),
            "⑦ 슬라이딩 압축이 파이프라인에서 사라졌다"
        );
        assert!(
            pipeline.contains("beam::place_sequence_beam"),
            "⑤ 빔 서치가 파이프라인에서 사라졌다"
        );
        assert!(
            pipeline.contains("select::pick_next_piece"),
            "⑥ best-fit 선택이 파이프라인에서 사라졌다"
        );
        assert!(
            pipeline.contains("expand_cluster"),
            "② 클러스터 확장이 파이프라인에서 사라졌다"
        );
        assert!(
            pipeline.contains("accrete::accrete"),
            "⑧ 누적 배치가 파이프라인에서 사라졌다"
        );
        assert!(
            pipeline.contains("reflow::reflow"),
            "⑨ 꼬리 회수가 파이프라인에서 사라졌다"
        );
        // ⑧은 ordering 을 안 쓰므로 호출부가 1개만 돌아야 한다. 그 분기가 살아 있는지.
        assert!(
            src.contains("ordering_is_only_a_multiset"),
            "⑧이 켜져도 ordering 을 전부 도는 상태로 되돌아갔다 — 같은 계산의 반복이다"
        );
    }

    /// `NfpStrategyConfig` 에 새 전략이 늘면 이 테스트가 **먼저 깨진다.**
    ///
    /// 스위치만 만들고 배선을 빠뜨리는 것이 이 코드베이스가 세 번 겪은 결함이다.
    /// 라벨 수와 배선 검사 항목 수를 묶어 두면, 전략을 추가하면서 배선 테스트를
    /// 갱신하지 않는 경로가 막힌다.
    #[test]
    fn every_declared_strategy_has_a_label() {
        let all_on = crate::nfp_strategy::NfpStrategyConfig {
            instance_rotations: true,
            pairwise_clustering: true,
            contact_maximization: true,
            pocket_first: true,
            beam_search: true,
            best_fit_selection: true,
            sliding_compaction: true,
            accretion: true,
            tail_reflow: true,
            ..crate::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG
        };
        assert_eq!(
            all_on.enabled_labels().len(),
            9,
            "전략을 추가했는데 `enabled_labels` 를 갱신하지 않았다 — 로그·리포트에서 사라진다"
        );
        assert!(all_on.any_enabled());
    }

    /// 전략이 전부 꺼져 있으면 기존 경로로 빠져야 한다.
    ///
    /// 새 경로가 기본 동작을 조용히 갈아치우면, 지금까지 쌓은 실측치가 전부 무의미해진다.
    #[test]
    fn all_strategies_off_keeps_the_existing_rotation_set() {
        let cfg = NfpLbfConfig::default();
        assert!(!cfg.strategy.any_enabled(), "기본값은 전부 꺼짐이어야 한다");
        // 꺼져 있으면 회전 집합은 설정값 그대로다 (인스턴스를 보지 않는다).
        assert_eq!(cfg.rotations, vec![0]);
    }

    #[test]
    fn the_probe_budget_follows_the_time_that_is_actually_left() {
        let ceiling = 8192;
        let sec = Duration::from_secs(1);

        // 목표(10초/10개 = 1초)의 절반도 안 걸렸다 -> 정밀도를 올린다.
        assert_eq!(
            adjust_probe_budget(4096, sec / 4, Duration::from_secs(10), 10, ceiling),
            8192
        );
        // 상한을 넘기지 않는다.
        assert_eq!(
            adjust_probe_budget(8192, sec / 100, Duration::from_secs(10), 10, ceiling),
            8192
        );
        // 목표의 1.5배를 넘겼다 -> 내린다.
        assert_eq!(
            adjust_probe_budget(4096, sec * 2, Duration::from_secs(10), 10, ceiling),
            2048
        );
        // 하한 아래로는 내려가지 않는다 — 그 아래는 배치가 무너진다.
        assert_eq!(
            adjust_probe_budget(MIN_ADAPTIVE_PROBES, sec * 100, Duration::from_secs(1), 10, ceiling),
            MIN_ADAPTIVE_PROBES
        );
        // 목표 근처에서는 그대로 둔다 — 매번 뒤집히면 ordering 마다 정밀도가 달라진다.
        assert_eq!(
            adjust_probe_budget(4096, sec, Duration::from_secs(10), 10, ceiling),
            4096
        );
    }

    #[test]
    fn the_budget_feedback_is_inert_when_there_is_nothing_to_decide() {
        let ceiling = 8192;
        // 남은 ordering 이 없다.
        assert_eq!(
            adjust_probe_budget(4096, Duration::from_secs(1), Duration::from_secs(10), 0, ceiling),
            4096
        );
        // 걸린 시간이 0 이면 판단 근거가 없다 (시계 해상도 아래).
        assert_eq!(
            adjust_probe_budget(4096, Duration::ZERO, Duration::from_secs(10), 5, ceiling),
            4096
        );
        // 시간이 다 됐다 — 어차피 남은 ordering 을 못 돈다.
        assert_eq!(
            adjust_probe_budget(4096, Duration::from_secs(1), Duration::ZERO, 5, ceiling),
            4096
        );
    }

    /// 기본값은 **꺼짐**이어야 한다 — 켜면 결과가 벽시계에 의존한다.
    ///
    /// 이 코드베이스는 회귀를 비트 단위로 잡는다. 기본 경로가 머신·부하에 따라
    /// 달라지면 지금까지 쌓은 실측치가 전부 비교 불가능해진다.
    #[test]
    fn adaptive_probe_budget_is_off_by_default_so_results_do_not_depend_on_the_clock() {
        let cfg = NfpLbfConfig::default();
        assert!(
            !cfg.adaptive_probe_budget,
            "기본으로 켜면 같은 입력이 머신마다 다른 배치를 낸다"
        );
    }

    /// 호출부 구성에서는 **올라갈 수 없다**는 사실을 고정한다.
    ///
    /// placer 가 `max_candidate_probes` 로 시작하고 상한도 같은 값이라, 시간이
    /// 아무리 남아돌아도 "2배" 분기는 같은 값을 돌려준다. 실제 동작은 안전밸브
    /// 하나뿐이라는 것을 여기서 못 박는다 — 누가 문서만 보고 "정밀도를 올려 준다"
    /// 고 믿으면 안 된다.
    #[test]
    fn in_the_real_call_shape_the_budget_can_only_go_down() {
        let cfg = NfpLbfConfig::default();
        let start = cfg.max_candidate_probes;

        // 시간이 남아돌아도 그대로다 (시작값 == 상한).
        assert_eq!(
            adjust_probe_budget(
                start,
                Duration::from_millis(1),
                Duration::from_secs(600),
                50,
                start
            ),
            start,
            "시작값이 상한과 같으므로 올라갈 수 없다"
        );

        // 예산을 넘기면 내려간다 — 이것이 유일하게 실제로 일어나는 동작이다.
        assert_eq!(
            adjust_probe_budget(
                start,
                Duration::from_secs(10),
                Duration::from_secs(10),
                10,
                start
            ),
            start / 2
        );
    }

    #[test]
    fn fewer_unplaced_always_wins() {
        // 미배치가 적으면 strip length가 아무리 길어도 이긴다.
        assert!(is_better((0, 9_999.0), (1, 1.0)));
        assert!(!is_better((2, 1.0), (1, 9_999.0)));
    }

    #[test]
    fn equal_unplaced_falls_back_to_strip_length() {
        assert!(is_better((3, 100.0), (3, 100.5)));
        assert!(!is_better((3, 100.5), (3, 100.0)));
        // 동률이면 먼저 평가한 쪽을 유지한다.
        assert!(!is_better((3, 100.0), (3, 100.0)));
    }

    fn placement_of(ids: &[usize], unplaced: &[usize]) -> SequentialPlacement {
        SequentialPlacement {
            pieces: ids
                .iter()
                .map(|&item_id| crate::nfp_lbf::placement::PlacedPiece {
                    item_id,
                    rotation: 0,
                    translation: (0.0, 0.0),
                })
                .collect(),
            strip_length: 100.0,
            placed: ids.len(),
            unplaced: unplaced.to_vec(),
        }
    }

    #[test]
    fn prefix_is_cut_at_the_first_unplaced_index() {
        // 앞 4개까지는 pieces 와 시퀀스가 1:1 로 일치하므로 전부 재사용한다.
        let p = placement_of(&[5, 3, 9, 1], &[]);
        assert_eq!(valid_prefix_len(&[5, 3, 9, 1], &p, 4), 4);
        // 아이템 7 이 미배치라 pieces 가 한 칸 당겨졌다 — 7 직전까지만 유효하다.
        let p = placement_of(&[5, 3, 1, 8], &[7]);
        assert_eq!(valid_prefix_len(&[5, 3, 7, 1, 8], &p, 5), 2);
        // 요청 길이보다 길게 쓰지 않는다.
        let p = placement_of(&[5, 3, 9, 1], &[]);
        assert_eq!(valid_prefix_len(&[5, 3, 9, 1], &p, 2), 2);
        // 배치된 조각 수 / 시퀀스 길이보다 길게 요청할 수 없다.
        assert_eq!(valid_prefix_len(&[5, 3, 9, 1, 8], &p, 30), 4);
        assert_eq!(valid_prefix_len(&[5, 3], &p, 30), 2);
        // 순서가 처음부터 다르면 재사용 없음.
        assert_eq!(valid_prefix_len(&[4, 3, 9, 1], &p, 4), 0);
        assert_eq!(valid_prefix_len(&[5, 3, 9, 1], &p, 0), 0);
        // 미배치 id 가 시퀀스 인덱스로 오해되면 안 된다: id 0 이 미배치여도
        // 앞 구간이 일치하면 그대로 재사용한다.
        let p = placement_of(&[5, 3, 9], &[0]);
        assert_eq!(valid_prefix_len(&[5, 3, 9, 0], &p, 4), 3);
    }

    #[test]
    fn degrees_convert_to_normalized_radians() {
        assert_eq!(rotation_to_radians(0), 0.0);
        assert!((rotation_to_radians(180) - std::f32::consts::PI).abs() < 1.0e-6);
        assert_eq!(rotation_to_radians(360), 0.0);
        assert!(
            (rotation_to_radians(-90) - 1.5 * std::f32::consts::PI).abs() < 1.0e-5,
            "음수 각도는 [0, 2pi)로 정규화되어야 한다"
        );
        // 모든 결과는 [0, 2pi) 안에 있어야 검증기의 회전 허용 판정을 통과한다.
        for deg in [-720, -181, -1, 0, 1, 179, 181, 359, 361, 1_080] {
            let r = rotation_to_radians(deg);
            assert!(r >= 0.0 && r < 2.0 * std::f32::consts::PI, "deg = {deg}");
        }
    }

    #[test]
    fn seed_strip_width_leaves_a_small_slack() {
        let w = seed_strip_width(1_000.0);
        assert!(w > 1_000.0);
        assert!(w < 1_002.0, "여유는 fit_strip이 곧바로 회수할 만큼 작아야 한다");
        assert!(seed_strip_width(0.5).is_finite());
    }
}
