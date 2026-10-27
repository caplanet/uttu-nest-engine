//! 커스텀 알고리즘 변형 모음 (upstream 원본에는 없는 fork 전용 파일).
//!
//! 설계 원칙 — **원본 파일(`explore.rs`)을 직접 뜯어고치지 않는다.**
//!   1. 원본과 "동일한 시그니처"의 새 함수를 여기(새 파일)에 만든다.
//!   2. `optimizer/mod.rs::optimize()` 에서 `#[cfg(feature = "adaptive")]`로
//!      원본 함수와 이 변형 함수를 컴파일 타임에 갈아끼운다.
//!
//! 이렇게 하면:
//!   - upstream(`git merge upstream/main`) 시 충돌이 거의 없다.
//!   - `--features adaptive` 를 켜고 끄며 원본 vs 내 알고리즘을 A/B 비교할 수 있다.
//!
//! 탐색 단계 교체는 `adaptive`, 그 밖의 커스텀 변형(래스터 NFP,
//! 좌측 편향 샘플링 등)은 `capmods`가 담당한다. 서로 독립적으로 켜고 끌 수 있다.
//!
//! 사용:  cargo run --release --features adaptive --bin bench -- -i data/... -t 30

use crate::config::{ExplorationConfig, ShrinkAdaptMode};
use crate::optimizer::explore::disrupt_solution; // explore.rs에서 pub(crate)로 연 헬퍼 재사용
use crate::optimizer::separator::Separator;
use crate::util::listener::{ReportType, SolutionListener};
use crate::util::terminator::Terminator;
use jagua_rs::Instant;
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use log::warn;
use rand::prelude::Distribution;
use rand_distr::Normal;
use std::time::Duration;

// (폭 진동 파라미터는 ExplorationConfig::osc_fail_threshold / osc_expand_steps 로 노출됨)

/// 다음 폭 진동에서 사용할 완화 폭을 계산한다.
///
/// 설정된 step만큼 넓히되, 현재 폭과 최선 feasible 폭 사이에 남은 여유의 절반만
/// 한 번에 사용한다. 따라서 반환값은 항상 `current_width < target < best_width`이고,
/// 같은 feasible incumbent에서도 여러 실패 구간에 걸쳐 반복 완화할 수 있다.
fn oscillation_relax_width(
    current_width: f32,
    best_width: f32,
    shrink_step: f32,
    expand_steps: f32,
) -> Option<f32> {
    if !current_width.is_finite()
        || !best_width.is_finite()
        || !shrink_step.is_finite()
        || !expand_steps.is_finite()
        || current_width <= 0.0
        || best_width <= current_width
        || shrink_step <= 0.0
        || expand_steps <= 0.0
    {
        return None;
    }

    let headroom = best_width - current_width;
    let requested_delta = current_width * shrink_step * expand_steps;
    let max_delta = headroom * 0.5;
    let delta = if requested_delta.is_finite() {
        requested_delta.min(max_delta)
    } else {
        max_delta
    };
    let target = current_width + delta;

    (target.is_finite() && target > current_width && target < best_width).then_some(target)
}

/// 축소 스텝의 **밀도 기반** 배율.
///
/// - [`ShrinkAdaptMode::DensityLegacy`]: 밀도의 절대값을 쓰는 기존 선형식.
/// - [`ShrinkAdaptMode::DensitySlack`]: 런 자신의 시작 밀도로 정규화한 진행도를 쓴다.
///   기준이 자기 자신이라 인스턴스의 절대 밀도에 의존하지 않고, 시작에서 `max_mult`,
///   이론 한계에서 1로 수렴하므로 `max_mult`가 실제로 살아 있는 값이 된다.
/// - 나머지 모드: 1.0 (밀도 적응 없음).
fn density_step_mult(
    mode: ShrinkAdaptMode,
    density: f32,
    density_start: f32,
    gain: f32,
    max_mult: f32,
) -> f32 {
    if !mode.uses_density() || !density.is_finite() {
        return 1.0;
    }

    match mode {
        ShrinkAdaptMode::DensityLegacy => (1.0 + gain * (1.0 - density)).clamp(1.0, max_mult),
        _ => {
            let span = 1.0 - density_start;
            if !span.is_finite() || span <= 1.0e-6 {
                return 1.0;
            }
            let progress = ((density - density_start) / span).clamp(0.0, 1.0);
            let head = (max_mult - 1.0).max(0.0);
            (1.0 + head * (1.0 - progress).powf(gain.max(0.0))).clamp(1.0, max_mult.max(1.0))
        }
    }
}

/// 실패 피드백 shrink 설정을 검증해 `(감쇠 계수, 배율 하한)`으로 돌려준다.
///
/// 모드가 피드백을 쓰지 않으면 `None` — 스텝 배율이 1.0으로 고정된다.
/// 감쇠나 하한이 범위 밖이면 기본값으로 정규화한다. 스텝에 0이나 음수를 곱하면 축소가
/// 멈추거나 폭이 커져 버리므로 조용히 통과시키지 않는다.
#[cfg_attr(not(feature = "cpguard-policy-off"), cpguard::obfuscate)]
fn shrink_feedback_params(config: &ExplorationConfig) -> Option<(f32, f32)> {
    if !config.shrink_adapt_mode.uses_feedback() {
        return None;
    }

    let defaults = crate::config::DEFAULT_SWARM_CONFIG.expl_cfg;
    let decay = config.shrink_fail_decay;
    let decay = if decay.is_finite() && decay > 0.0 && decay < 1.0 {
        decay
    } else {
        warn!(
            "[adaptive] shrink_fail_decay {} is out of (0,1); using the default {}",
            decay, defaults.shrink_fail_decay
        );
        defaults.shrink_fail_decay
    };

    let min_scale = config.shrink_scale_min;
    let min_scale = if min_scale.is_finite() && min_scale > 0.0 && min_scale <= 1.0 {
        min_scale
    } else {
        warn!(
            "[adaptive] shrink_scale_min {} is out of (0,1]; using the default {}",
            min_scale, defaults.shrink_scale_min
        );
        defaults.shrink_scale_min
    };

    Some((decay, min_scale))
}

/// 복구 라운드 하나에만 걸리는 보조 데드라인.
///
/// 바깥 terminator의 중단은 그대로 전파하고, 라운드 예산이 끝나면 추가로 kill 한다.
/// strike 한도만으로는 한 strike 안에서 `iter_no_imprv_limit`를 전부 태울 수 있어
/// 초 단위 예산에서 복구 한 번이 예산을 다 먹는 것을 막지 못한다.
struct RoundDeadline<'a, T: Terminator> {
    inner: &'a T,
    until: Option<Instant>,
}

impl<T: Terminator> Terminator for RoundDeadline<'_, T> {
    fn kill(&self) -> bool {
        self.inner.kill() || self.until.is_some_and(|until| Instant::now() > until)
    }

    /// 라운드 예산 소진은 "외부 취소"가 아니다 — 바깥 의미를 그대로 전달한다.
    fn stop_requested(&self) -> bool {
        self.inner.stop_requested()
    }

    fn new_timeout(&mut self, _timeout: std::time::Duration) {
        // 라운드 데드라인은 생성 시점에 고정된다.
    }

    fn timeout_at(&self) -> Option<Instant> {
        match (self.inner.timeout_at(), self.until) {
            (Some(outer), Some(round)) => Some(if round < outer { round } else { outer }),
            (outer, round) => outer.or(round),
        }
    }
}

/// 분리 1회 벽시계 상한을 실행 중 관측한 성공 시간에서 정한다 (`sep_cap_adaptive`).
///
/// 벽 근처에서 탐색 시간의 70 ~ 90% 는 **실패하는 분리**가 쓴다. 실패는 성공보다 오래 걸린다 —
/// 성공은 충돌이 사라지는 즉시 끝나지만, 실패는 strike 한도를 다 태워야 끝난다. 그래서
/// «성공이 걸리는 시간» 을 조금 넘긴 분리는 대부분 실패로 끝날 분리이고, 거기서 끊고 다음
/// 교란으로 넘어가면 같은 예산으로 더 많은 시도를 한다.
///
/// 규칙:
/// - 성공 표본([`Self::MIN_SAMPLE`] 이상 걸린 성공)이 [`Self::MIN_OK`] 개 모이기 전에는 상한이
///   없다. 벽에서 먼 폭의 분리는 1초 안에 끝나므로 표본에서 뺀다.
/// - 상한 = max([`Self::FLOOR`], 가장 오래 걸린 성공 × [`Self::KAPPA`]).
/// - 상한에 걸린 실패가 [`Self::ESCALATE_EVERY`] 번 이어질 때마다 상한을
///   [`Self::ESCALATE_BY`] 배로 늘린다. 조여 갈수록 성공에 필요한 시간도 늘기 때문에, 고정된
///   상한은 어느 밀도부터 모든 시도를 끊는다. 성공하면 배율은 1로 돌아간다.
#[derive(Debug, Clone, Copy, Default)]
struct SepCapTuner {
    /// 표본에 든 성공 중 가장 오래 걸린 것.
    longest_ok: Duration,
    /// 표본에 든 성공의 수.
    n_ok: usize,
    /// 마지막 성공 뒤로 상한에 걸려 끝난 실패의 수.
    capped_fails: usize,
}

impl SepCapTuner {
    const MIN_SAMPLE: Duration = Duration::from_secs(1);
    const MIN_OK: usize = 3;
    const FLOOR: Duration = Duration::from_secs(5);
    const KAPPA: f32 = 1.25;
    const ESCALATE_EVERY: usize = 4;
    const ESCALATE_BY: f32 = 1.5;

    /// 다음 분리에 걸 상한. 표본이 모자라면 `None`.
    fn cap(&self) -> Option<Duration> {
        if self.n_ok < Self::MIN_OK {
            return None;
        }
        let base = self.longest_ok.mul_f32(Self::KAPPA).max(Self::FLOOR);
        let steps = (self.capped_fails / Self::ESCALATE_EVERY).min(16) as i32;
        Some(base.mul_f32(Self::ESCALATE_BY.powi(steps)))
    }

    /// 분리 하나의 결과를 반영한다. `cap` 은 그 분리에 걸려 있던 상한이다.
    fn record(&mut self, elapsed: Duration, cap: Option<Duration>, feasible: bool) {
        if feasible {
            if elapsed >= Self::MIN_SAMPLE {
                self.n_ok += 1;
                self.longest_ok = self.longest_ok.max(elapsed);
            }
            self.capped_fails = 0;
        } else if cap.is_some_and(|cap| elapsed >= cap) {
            self.capped_fails += 1;
        }
    }
}

/// **과도한 축소**로 보는 문턱 — 최선 feasible 폭에서 이 비율 넘게 좁힌 폭.
///
/// 기본 설정의 한 걸음은 0.1% × 밀도 배율(최대 약 2) = 0.2% 안팎이라 여기에 닿지 않는다.
/// `shrink_step` 을 크게(예: 0.1) 주면 밀도 배율까지 곱해져 한 걸음이 19 ~ 22% 가 되고,
/// 그 폭은 조각 면적만으로도 들어가지 않는다(밀도 90 ~ 96%).
///
/// 그런 폭에서의 실패는 «풀 수 있는 매듭» 이 아니라 «불가능한 폭» 이다. 그런데 종전 흐름은
/// 둘을 구분하지 않아서, 같은 폭에서 교란을 바꿔 가며 네 번을 다 실패한 뒤에야 진동으로 폭을
/// 절반 되돌렸다. 실측(2026-10-02, `shrink_step` 0.1, 탐색 480초):
///
/// * 여성WSjk 156 — 7226 → 5648mm(밀도 90%)에서 106 + 54 + 69 + 75초 실패, 진동으로 6437mm,
///   다음 목표 6042mm(84%)에서 다시 네 번 실패하다 시간이 끝났다. 탐색 결과 79.03%(평소 80.3 ~ 80.5%).
/// * babel 겉감 — 13743 → 11133mm(밀도 95.7%)에서 67 + 43 + 57 + 42초 실패. 탐색 결과 81.64%(평소 85.3 ~ 86.0%).
///
/// 이 문턱을 넘는 폭에서 실패하면 재시도 없이 **최선 feasible 해로 돌아가 걸음을 반으로 줄인다**
/// ([`OVERSHOOT_BACKOFF`]). 그 시도 자체에도 시간 상한을 건다([`overshoot_attempt_cap`]).
const OVERSHOOT_GAP: f32 = 0.005;

/// 과도한 축소가 실패할 때마다 걸음에 곱하는 값. 성공해도 되돌리지 않는다 — 배치가 조밀해질수록
/// 통하는 걸음은 작아지기만 한다.
const OVERSHOOT_BACKOFF: f32 = 0.5;

/// 과도한 축소 시도 하나의 시간 상한: 조각 하나당 100ms, 최소 10초.
///
/// 불가능한 폭에서의 분리는 손실이 조금씩이라도 줄어드는 동안 포기하지 않아 한 번에 1분 안팎을
/// 쓴다. 상한에서 끊고 걸음을 줄이는 쪽이 싸다 — 통하는 걸음이라면 이 시간 안에 풀린다.
fn overshoot_attempt_cap(n_items: usize) -> Duration {
    Duration::from_millis(100)
        .saturating_mul(n_items.min(u32::MAX as usize) as u32)
        .max(Duration::from_secs(10))
}

/// 최선 feasible 폭 대비 `width` 가 얼마나 좁은가(비율). 좁지 않으면 0.
fn shrink_gap(best_width: f32, width: f32) -> f32 {
    if best_width.is_finite() && width.is_finite() && best_width > 0.0 && width < best_width {
        (best_width - width) / best_width
    } else {
        0.0
    }
}

/// 폭 진동이 더 넓힐 여유가 남지 않았는가 — 지금 폭이 최선 feasible 폭과 사실상 같은가.
///
/// 폭 진동은 겹친 배치(실패한 계보)를 그대로 둔 채 최선 폭까지 남은 여유의 절반만 넓힌다. 거기서도
/// 풀리지 않으면 또 절반 — 여유 21mm 가 f32 눈금 하나가 될 때까지 14번이고, 그 뒤로는 **최선 폭과
/// 같은 폭에서** 겹친 배치를 교란하며 실패만 반복한다. 그 폭에는 이미 겹침 없는 해가 있다.
/// 실측(2026-10-02, babel 겉감 480초 탐색): 최선 폭의 0.01% 안에서 실패하며 쓴 시간이 기준 실행
/// 13개 중 3개에서 4 ~ 139초, 「축소 직후 재배열」 을 켠 실행 8개 중 7개에서 39 ~ 339초(연속 실패 최대 69회).
///
/// 이 상태가 되면 겹친 계보를 버리고 최선 해로 돌아가 다시 줄인다.
///
/// 진동 자체를 «최선 해로 복귀 + 절반 걸음» 으로 바꾸는 것은 **더 나빴다** — babel 시드 1 ~ 4 가
/// 85.79 / 85.68 / 85.40 / 84.70 (기준 86.1 / 85.7 / 86.05 / 84.9), wsjk 는 81.16 / 81.09 (기준 81.0 / 81.04).
/// 진동이 푸는 것은 다섯에 하나뿐이지만 그렇게 풀린 배치가 다음 축소의 출발점을 바꿔 준다. 그래서
/// 여유가 남아 있는 동안은 진동을 그대로 둔다.
fn stalled_at_best(best_width: f32, current_width: f32) -> bool {
    best_width.is_finite()
        && current_width.is_finite()
        && current_width > 0.0
        && current_width <= best_width
        && shrink_gap(best_width, current_width) <= AT_BEST_GAP
}

/// 이 간격(비율) 안이면 최선 폭과 같은 폭으로 본다. 12m 마커에서 1.2mm.
const AT_BEST_GAP: f32 = 1.0e-4;

/// 폭 완화 복구 1회의 결과.
#[derive(Debug, Clone, Copy)]
struct ReliefOutcome {
    /// 복구 후 strip 폭. 해소에 실패하면 막힌 폭 그대로다.
    width: f32,
    /// 충돌을 전부 해소했는가.
    resolved: bool,
}

/// 폭 완화 복구의 다음 목표 폭.
///
/// `stuck * (1 + max_ratio)`를 천장으로 두고 현재 폭을 `step` 비율만큼 넓힌다.
/// 폭 진동과 달리 **최선 feasible 폭을 참조하지 않는다** — 천장은 오직 이 비율이다.
/// 이미 천장에 닿았거나 입력이 비정상이면 `None`.
fn relief_next_width(current: f32, stuck: f32, step: f32, max_ratio: f32) -> Option<f32> {
    if !current.is_finite()
        || !stuck.is_finite()
        || !step.is_finite()
        || !max_ratio.is_finite()
        || current <= 0.0
        || stuck <= 0.0
        || current < stuck
        || step <= 0.0
        || max_ratio <= 0.0
    {
        return None;
    }

    let ceiling = stuck * (1.0 + max_ratio);
    let target = (current * (1.0 + step)).min(ceiling);

    (target.is_finite() && target > current).then_some(target)
}

/// 충돌 손실로 가중한 아이템 중심 x — 매듭이 몰려 있는 위치다.
///
/// [`Separator::change_strip_width`]의 split position으로 넘기면 늘어난 폭이 정확히
/// 그 자리에 삽입된다(그 x보다 오른쪽 아이템만 delta만큼 밀린다). 기본값(폭의 중앙)은
/// 매듭이 좌측이나 우측에 몰려 있을 때 엉뚱한 곳을 벌린다. 충돌이 없으면 `None`.
fn collision_hotspot_x(sep: &Separator) -> Option<f32> {
    let (weighted, total) = sep.prob.layout().placed_items().iter().fold(
        (0.0f64, 0.0f64),
        |(weighted, total), (pk, pi)| {
            let loss = sep.ct.get_loss(pk);
            let x = pi.shape().centroid().0;
            if loss.is_finite() && loss > 0.0 && x.is_finite() {
                (
                    weighted + f64::from(x) * f64::from(loss),
                    total + f64::from(loss),
                )
            } else {
                (weighted, total)
            }
        },
    );

    (total > 0.0)
        .then(|| (weighted / total) as f32)
        .filter(|x| x.is_finite())
}

/// 폭 완화 복구 — 막힌 폭의 최선 충돌 해를 **그대로 둔 채** 폭을 조금씩 넓히며
/// 재분리해서 매듭을 푼다.
///
/// 폭 진동([`oscillation_relax_width`])과의 차이는 상한이다. 진동은 최선 feasible
/// 폭까지 남은 여유의 절반으로 잘리고 그 여유는 언제나 "직전 shrink 한 스텝"이라,
/// 누적해도 그 폭을 넘지 못한다. 복구의 상한은 `relief_max_ratio`뿐이라 최선
/// feasible 폭을 넘어설 수 있다. 인컴번트는 호출자의 `feasible_sols`에 따로 보존되고
/// 단계 경계의 `retain_exact_no_worse`가 더 넓은 결과를 거부하므로, 넘어서더라도
/// 최종 결과가 나빠지지는 않는다.
///
/// 확장은 [`collision_hotspot_x`] 위치에 삽입하므로 늘어난 폭이 매듭에 직접 들어간다.
///
/// 성공하면 넓어진 폭과 무충돌 배치를 separator에 남긴 채 반환한다. 실패하면 막힌
/// 폭과 출발 해를 정확히 되돌린다. 설정이 비정상이거나 이미 중단 요청이 들어왔으면
/// 확장을 한 번도 시도하지 않고 `None`을 반환한다 (호출자는 기존 disruption 경로로
/// 계속 진행한다). 중단으로 끝난 경우 separator는 `start_sol` 상태로 남는데, 호출자가
/// 곧바로 풀에서 고른 해로 다시 rollback 하므로 문제되지 않는다.
fn width_relief_escape(
    sep: &mut Separator,
    sol_listener: &mut Option<&mut dyn SolutionListener>,
    term: &impl Terminator,
    config: &ExplorationConfig,
    start_sol: &SPSolution,
    stuck_width: f32,
) -> Option<ReliefOutcome> {
    if config.relief_max_rounds == 0
        || term.kill()
        || relief_next_width(
            stuck_width,
            stuck_width,
            config.relief_step,
            config.relief_max_ratio,
        )
        .is_none()
    {
        return None;
    }

    // 막힌 폭에서 가장 좋은 충돌 해로 되돌린 뒤 시작한다.
    sep.rollback(start_sol, None);

    // 확장 라운드는 "이 폭이면 풀리는가"만 본다 — 탐색용 긴 예산은 낭비다.
    // strike 한도만 줄인다 (sample_config는 워커 재동기화가 필요하므로 건드리지 않는다).
    let saved_config = sep.config;
    sep.config.strike_limit = config.relief_strike_limit.max(1);

    let mut width = stuck_width;
    let mut rounds = 0;
    let mut resolved = false;

    while rounds < config.relief_max_rounds && !term.kill() {
        let Some(next_width) = relief_next_width(
            width,
            stuck_width,
            config.relief_step,
            config.relief_max_ratio,
        ) else {
            break;
        };

        let split = collision_hotspot_x(sep);
        sep.change_strip_width(next_width, split);
        width = next_width;
        rounds += 1;

        // 라운드 예산이 있으면 이 분리에만 적용한다. 바깥 중단은 그대로 전파된다.
        let round_term = RoundDeadline {
            inner: term,
            until: (!config.relief_round_budget.is_zero())
                .then(|| Instant::now() + config.relief_round_budget),
        };
        let (sol, ct) = sep.separate(&round_term, sol_listener);
        if ct.get_total_loss() == 0.0 {
            // GLS 가중치는 유지한다 — 어느 쌍이 문제였는지는 이후 축소에서도 유효하다.
            sep.rollback(&sol, Some(&ct));
            resolved = true;
            break;
        }
    }

    sep.config = saved_config;

    if rounds == 0 {
        return None;
    }
    if !resolved {
        // 막힌 폭과 출발 해를 정확히 되돌린다 — 호출자의 풀과 카운터가 그대로 유효해진다.
        // `rollback` 이 strip 도 복원하지만, 폭 변경의 resync(작업자 RNG 재설정 포함)를
        // 빼면 이후 탐색 궤적이 달라져 width_relief 테스트가 가끔 시간 제한까지 돌았다.
        sep.change_strip_width(stuck_width, None);
        sep.rollback(start_sol, None);
        return Some(ReliefOutcome {
            width: stuck_width,
            resolved: false,
        });
    }

    Some(ReliefOutcome {
        width,
        resolved: true,
    })
}

/// 원본 `exploration_phase`(Algorithm 12)의 커스텀 변형.
///
/// 원본과의 차이점:
///   1. strip 폭을 줄이는 `shrink_step` 을 고정값이 아니라
///      현재 밀도(density)에 따라 동적으로 조절한다("밀도 기반 적응형 shrink").
///   2. 기본 perturbation으로 정체한 배치를 흔든다.
///   3. 같은 폭에서 연속 실패하면 폭을 넓혔다가 다시 조여 들어간다 — 좁은 폭에서 풀 수
///      없는 매듭을 넓은 폭에서 푸는 것이다. 두 전략 중 하나가 쓰인다:
///        - `config.relief_fail_threshold > 0`: **폭 완화 복구**. 충돌이 몰린 x에 폭을
///          삽입하며 `relief_max_ratio`까지 조금씩 넓혀 충돌이 풀릴 때까지 재분리한다.
///          최선 feasible 폭을 넘어설 수 있다 ([`width_relief_escape`]).
///        - 그 외 `config.osc_fail_threshold > 0`: 기존 **폭 진동**. 최선 feasible 폭까지
///          남은 여유의 절반만 되돌린다 ([`oscillation_relax_width`]).
///   4. infeasible 솔루션 풀 크기를 상한으로 막아 메모리를 일정하게 유지한다.
///
/// 직관:
///   - 밀도가 낮다 = 아직 빈 공간이 많다 → **과감히 크게** 줄여 빠르게 수렴
///   - 밀도가 높다 = 거의 꽉 찼다(목표 근접) → **신중히 작게** 줄여 infeasible 폭주 방지
///
/// 공식:  `adaptive_step = base_step * clamp(1 + GAIN*(1 - density), 1, MAX_MULT)`
///   - density 0.50 → mult≈3.0 (3배 빠르게)
///   - density 0.90 → mult≈1.4
///   - density 0.97 → mult≈1.12 (원본과 거의 동일)
///
/// 나머지 로직(separate / infeasible 풀 관리 / rollback / disrupt)은 원본과 100% 동일.
pub fn exploration_phase_adaptive(
    instance: &SPInstance,
    sep: &mut Separator,
    sol_listener: &mut Option<&mut dyn SolutionListener>,
    term: &impl Terminator,
    config: &ExplorationConfig,
) -> Vec<SPSolution> {
    let _run_scope = crate::config::runtime::enter_if_unset(sep.run_control.clone());
    // infeasible 솔루션 풀 크기 상한. 원본은 무제한이라 실패가 누적되면 해 스냅샷이
    // 계속 쌓여 메모리를 잠식한다. 선택은 어차피 좋은 해에 쏠리므로 상위만 유지한다.
    const MAX_INFEAS_POOL_SIZE: usize = 32;

    // 적응형 shrink 파라미터
    let mode = config.shrink_adapt_mode;
    let gain = config.adaptive_shrink_gain.max(0.0);
    let max_mult = config.adaptive_shrink_max_mult.max(1.0);

    // 실패 피드백 shrink: 스텝 배율(1.0 = 설정 그대로). None이면 비활성.
    let shrink_feedback = shrink_feedback_params(config);
    let mut step_scale = 1.0f32;
    // 과도한 축소가 실패할 때마다 반으로 줄어드는 걸음 배율 (`OVERSHOOT_GAP`). 1.0 = 설정 그대로.
    let mut overshoot_scale = 1.0f32;

    // 폭 탈출 전략은 둘 중 하나만 쓴다. 복구가 켜져 있으면 진동은 무시된다.
    let relief_enabled = config.relief_fail_threshold > 0;

    let mut current_width = sep.prob.strip_width();
    let mut best_width = current_width;
    // DensitySlack 모드의 기준점. 런마다 자기 자신의 출발 밀도를 쓴다.
    let density_start = sep.prob.density();

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
    // 풀 크기가 상한에 걸려도 종료 조건이 유지되도록 실패 횟수는 별도로 센다
    let mut n_conseq_fails: usize = 0;

    // 폭 진동용: 현재 폭에서의 연속 실패 횟수
    let mut n_fails_at_width: usize = 0;
    // 이 실행에서 분리에 실패한 폭 중 가장 넓은 값. 축소 목표의 하한으로 쓴다.
    let mut max_infeasible_width: Option<f32> = None;
    // Width-relief 실패 뒤 같은 폭의 정체 카운터는 보존하되, 매 실패마다 비싼
    // 복구를 재시도하지 않도록 다음 시도 지점만 별도로 미룬다.
    let mut next_relief_attempt = config.relief_fail_threshold;

    let mut cap_tuner = SepCapTuner::default();
    while !term.kill() {
        // Attempt to separate the current layout.
        //
        // 상한(`sep_time_cap` 또는 적응형)이 있으면 이 분리에만 보조 데드라인을 건다. 상한이
        // 오면 분리는 최선 충돌 해를 돌려주고 아래 실패 분기로 간다 — 막힌 폭을 건너뛴다.
        // 바깥 중단(`term`)은 그대로 전파되고, 상한 소진은 중단으로 취급되지 않는다.
        let sep_cap = if config.sep_cap_adaptive {
            cap_tuner.cap()
        } else {
            (!config.sep_time_cap.is_zero()).then_some(config.sep_time_cap)
        };
        // 과도한 축소 시도에는 따로 상한을 건다 (`OVERSHOOT_GAP`).
        let overshoot_attempt =
            !feasible_sols.is_empty() && shrink_gap(best_width, current_width) > OVERSHOOT_GAP;
        let sep_cap = if overshoot_attempt {
            let cap = overshoot_attempt_cap(sep.prob.layout().placed_items().len());
            Some(sep_cap.map_or(cap, |configured| configured.min(cap)))
        } else {
            sep_cap
        };
        let sep_started = Instant::now();
        let cap_term = RoundDeadline {
            inner: term,
            until: sep_cap.map(|cap| sep_started + cap),
        };
        let local_best = sep.separate_with_early_reports(&cap_term, sol_listener);
        let sep_elapsed = sep_started.elapsed();
        let total_loss = local_best.1.get_total_loss();
        // 충돌 손실이 0 인데 정확 인증(원본 겹침·회전·고정 계약)을 통과하지 못한 후보는
        // **이 폭에서의 실패한 시도**로 다룬다. 예전에는 여기서 `break` 해 탐색을 통째로
        // 끝냈다 — 인증 실패 한 번에 남은 탐색 예산이 전부 버려졌다(실측: 8초 예산의
        // 탐색이 0.04초에 끝났다). 압축 단계는 같은 상황을 실패한 시도로만 처리한다.
        // 아주 작은 양수 손실로 바꿔 아래의 실패 분기(풀·교란·재분리)로 보낸다.
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
        // 적응형 상한의 표본. 바깥 중단으로 끊긴 분리는 성공도 실패도 아니므로 넣지 않는다.
        if config.sep_cap_adaptive && !term.kill() {
            cap_tuner.record(sep_elapsed, sep_cap, total_loss == 0.0);
        }
        // Separation returns its best snapshot; subsequent shrinking must use that state.
        if total_loss == 0.0 {
            sep.rollback(&local_best.0, Some(&local_best.1));
            current_width = local_best.0.strip_width();
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

            // Preserve the certified early report when its callback requests Stop.
            if term.kill() {
                break;
            }

            // 실패 피드백: 성공 한 번이 실패 한 번을 되돌린다
            // (compress.rs의 AIMD 회복과 같은 관례).
            if let Some((decay, _)) = shrink_feedback {
                step_scale = (step_scale / decay).min(1.0);
            }

            // 교란이 벽을 뚫어 이 폭에서 성공했다면, 그 이상을 '실패 폭'으로 남겨 둘
            // 이유가 없다. 기록을 지워 다시 전체 스텝으로 내려갈 수 있게 한다.
            if let Some(bad) = max_infeasible_width
                && bad >= current_width
            {
                max_infeasible_width = None;
            }

            // ★★★ 밀도 기반 배율 × 실패 피드백 배율 ★★★
            let density = sep.prob.density(); // 0.0 ~ 1.0
            let mult = density_step_mult(mode, density, density_start, gain, max_mult);
            let base_step = crate::config::runtime::effective_shrink_step(config.shrink_step);
            let adaptive_step = base_step * mult * step_scale * overshoot_scale;
            let mut next_width = current_width * (1.0 - adaptive_step);
            // 이미 실패한 폭 아래로 다시 뛰어내리지 않는다.
            //
            // 축소 스텝은 고정 비율이라, 어떤 폭에서 실패한 뒤 완화/교란으로 그 위에서
            // 다시 성공하면 다음 목표가 **또 그 실패한 폭 아래**로 내려간다. 그래서 같은
            // 벽을 반복해서 들이받고 그 사이 구간(스텝 하나 폭)은 한 번도 시도되지 않는다.
            // 실측: shrink_step 0.004 에서 성공 14회 뒤 480초 중 345초가 이 반복에 쓰였다.
            //
            // 대신 실행 중 관측한 **가장 넓은 실패 폭**을 하한으로 두고 그 위를 이분한다.
            // 가능/불가능 경계로 수렴하는 표준적인 접근이다. 실패한 폭 자체는 계속
            // 재시도되므로(실패 분기는 current_width 를 그대로 둔다) 교란이 벽을 뚫으면
            // 하한도 함께 내려간다. 즉 탐색 가능 영역을 영구히 잘라내지 않는다.
            if let Some(bad) = max_infeasible_width
                && next_width <= bad
                && bad < current_width
            {
                let midpoint = 0.5 * (current_width + bad);
                if midpoint < current_width {
                    next_width = midpoint;
                }
            }
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
            n_fails_at_width = 0;
            next_relief_attempt = config.relief_fail_threshold;
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
            // 상한 초과분(가장 나쁜 해)은 버려 메모리를 일정하게 유지한다
            infeas_sol_pool.truncate(MAX_INFEAS_POOL_SIZE);

            // 이 폭은 (적어도 지금의 배치·예산에서는) 풀리지 않았다. 다음 축소 목표의
            // 하한으로 기록한다. 현재 폭 자체는 계속 재시도되므로 영구 배제가 아니다.
            //
            // 과도한 축소의 실패는 기록하지 않는다. 그것은 «한 번에 너무 많이 줄였다» 는
            // 뜻이지 그 폭이 벽이라는 증거가 아니고(시간 상한에서 끊은 시도다), 기록하면
            // 더 작은 걸음으로 충분히 지나갈 수 있는 폭이 이분 탐색의 바닥으로 굳는다.
            let overshoot_failure =
                !feasible_sols.is_empty() && shrink_gap(best_width, current_width) > OVERSHOOT_GAP;
            if !overshoot_failure {
                max_infeasible_width = Some(match max_infeasible_width {
                    Some(prev) if prev >= current_width => prev,
                    _ => current_width,
                });
            }

            n_conseq_fails += 1;
            if n_conseq_fails >= config.max_conseq_failed_attempts.unwrap_or(usize::MAX) {
                break;
            }

            // 여기서부터는 전부 "다음 반복을 위한 준비"(폭 진동 / 해 선택 / disruption)다.
            // separation이 중단 요청으로 빠져나온 경우 그 작업은 버려지므로,
            // 비싼 R&R을 시작하기 전에 여기서 끝낸다.
            if term.kill() {
                break;
            }

            // ── 과도한 축소의 되돌림 (`OVERSHOOT_GAP`) ───────────────────────────
            //
            // 최선 feasible 폭에서 한참 좁힌 폭의 실패는 교란으로 풀 매듭이 아니다. 재시도하지
            // 않고, 겹친 배치도 버리고, 최선 feasible 해에서 **절반 걸음**으로 다시 줄인다.
            // 줄어든 걸음은 이후의 축소에도 그대로 쓰인다(`overshoot_scale`).
            let gap = shrink_gap(best_width, current_width);
            if overshoot_failure && let Some(best_sol) = feasible_sols.last() {
                overshoot_scale *= OVERSHOOT_BACKOFF;
                sep.rollback(best_sol, None);
                let mut next_width = best_width * (1.0 - gap * OVERSHOOT_BACKOFF);
                if let Some(min_w) = sep.min_feasible_width()
                    && next_width < min_w
                {
                    next_width = min_w;
                }
                infeas_sol_pool.clear();
                n_fails_at_width = 0;
                next_relief_attempt = config.relief_fail_threshold;
                if next_width < best_width {
                    sep.change_strip_width(next_width, None);
                    current_width = next_width;
                } else {
                    // 고정 조각이 더 줄이지 못하게 막고 있다 — 최선 폭에서 계속한다.
                    current_width = best_width;
                }
                continue;
            }

            // 같은 폭에서 계속 실패하면 폭을 넓혔다가 다시 조여 들어간다.
            // 좁은 폭에서 풀 수 없는 매듭을 넓은 폭에서 푼 뒤 재접근하는 효과.
            n_fails_at_width += 1;
            // 실패 피드백: 이 폭에서의 **첫** 실패에만 반응한다. `n_fails_at_width`는 폭이
            // 바뀔 때마다 0으로 돌아가므로 결과적으로 "폭 하나당 한 번"이 된다. 같은 폭에서의
            // 교란 재시도까지 세면 스텝이 몇 초 만에 하한으로 무너진다.
            if let Some((decay, min_scale)) = shrink_feedback
                && n_fails_at_width == 1
            {
                step_scale = (step_scale * decay).max(min_scale);
            }
            if relief_enabled && n_fails_at_width >= next_relief_attempt {
                // 폭 완화 복구: 최선 충돌 해를 유지한 채 충돌이 풀릴 때까지 미세 확장.
                let stuck_width = current_width;
                // 실패 분기 진입 시 방금 채웠으므로 풀은 비어 있을 수 없다.
                let start_sol = infeas_sol_pool[0].0.clone();
                match width_relief_escape(sep, sol_listener, term, config, &start_sol, stuck_width)
                {
                    Some(outcome) if outcome.resolved => {
                        n_fails_at_width = 0;
                        next_relief_attempt = config.relief_fail_threshold;
                        current_width = outcome.width;
                        infeas_sol_pool.clear();
                        continue;
                    }
                    Some(_) => {
                        // 폭과 해는 복구 함수가 이미 되돌렸다. 풀은 그대로 두고 아래
                        // disruption 경로로 계속한다. 정체 신호는 selector/shrink
                        // feedback을 위해 보존하고, 복구 재시도 지점만 미룬다.
                        next_relief_attempt =
                            n_fails_at_width.saturating_add(config.relief_fail_threshold.max(1));
                    }
                    None => {
                        // 설정이 비정상이어도 같은 호출을 매 실패마다 반복하지 않는다.
                        next_relief_attempt =
                            n_fails_at_width.saturating_add(config.relief_fail_threshold.max(1));
                    }
                }
            } else if !relief_enabled
                && config.osc_fail_threshold > 0
                && n_fails_at_width >= config.osc_fail_threshold
            {
                n_fails_at_width = 0;
                // 더 넓힐 여유가 없으면 겹친 계보를 버리고 최선 해로 돌아간다
                // ([`stalled_at_best`]). 다음 반복의 분리는 바로 성공하고, 거기서 다시 줄인다.
                if let Some(best_sol) = feasible_sols.last()
                    && stalled_at_best(best_width, current_width)
                {
                    sep.rollback(best_sol, None);
                    current_width = best_width;
                    // 여기까지의 실패는 겹친 계보의 것이다 — 폭의 하한으로 남기지 않는다.
                    max_infeasible_width = None;
                    infeas_sol_pool.clear();
                    continue;
                }
                if let Some(w_osc) = oscillation_relax_width(
                    current_width,
                    best_width,
                    config.shrink_step,
                    config.osc_expand_steps,
                ) {
                    // 가장 좋은 infeasible 해에서 폭을 넓혀 매듭을 푼다.
                    //
                    // 넓힌 폭은 스트립 한가운데에 들어간다(분할 위치 `None`). 매듭이 있는
                    // 자리(`collision_hotspot_x`)에 넣는 쪽이 그럴듯해 보이지만 **이득이
                    // 없었다** — 2026-10-01 실측(480초, 시드 1·2): babel 겉감 86.09 / 85.73 →
                    // 86.01 / 85.66, 여성WSjk 156 80.98 / 81.04 → 80.99 / 80.13. 진동 임계를
                    // 1 로 낮춰 진동을 자주 일으키면 양쪽 모두 더 나빠졌다(babel 85.72 / 84.98).
                    // 진동 직후의 첫 분리는 매듭 위치에 넣어도 36회 중 5회만 성공했다.
                    sep.rollback(&infeas_sol_pool.first().unwrap().0, None);
                    sep.change_strip_width(w_osc, None);
                    current_width = w_osc;
                    infeas_sol_pool.clear();
                    continue;
                }
            }

            // Restore to a random solution from the pool (better solutions favoured)
            let selected_sol = {
                let distribution =
                    Normal::new(0.0, config.solution_pool_distribution_stddev).unwrap();
                let sample = distribution.sample(&mut sep.rng).abs().min(0.999);
                let selected_idx = (sample * infeas_sol_pool.len() as f32) as usize;

                &infeas_sol_pool[selected_idx].0
            };

            // Rollback to this solution and disrupt it.
            sep.rollback(selected_sol, None);

            // Keep the base search perturbation; no auxiliary selector,
            // trial portfolio, R&R preparation or performance bookkeeping.
            disrupt_solution(sep, config);
        }
    }

    // Never leave a caller looking at a last, unrepaired perturbation.
    sep.rollback(feasible_sols.last().unwrap_or(&initial_solution), None);
    feasible_sols
}

#[cfg(test)]
mod tests {
    use super::{
        collision_hotspot_x, density_step_mult, oscillation_relax_width, relief_next_width,
        shrink_feedback_params, width_relief_escape,
    };
    use crate::config::ShrinkAdaptMode;
    use crate::config::{DEFAULT_SWARM_CONFIG, ExplorationConfig};
    use crate::consts::LBF_SAMPLE_CONFIG;
    use crate::optimizer::lbf::LBFBuilder;
    use crate::optimizer::separator::Separator;
    use crate::util::assertions::tracker_matches_layout;
    use crate::util::io::read_spp_input;
    use crate::util::terminator::{BasicTerminator, Terminator};
    use jagua_rs::probs::spp::entities::SPSolution;
    use rand::SeedableRng;
    use rand::rngs::Xoshiro256PlusPlus;
    use std::path::Path;

    /// fu.json LBF 해에서 strip을 `shrink`만큼 좁혀 충돌 상태의 separator를 만든다.
    fn stuck_separator(seed: u64, shrink: f32) -> (Separator, SPSolution, f32) {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input/fu.json");
        let (external, _) = read_spp_input(&path).expect(crate::dmsg!("failed to read fu.json"));
        let config = DEFAULT_SWARM_CONFIG;
        let importer = crate::util::io::new_importer(config.cde_config, config.poly_simpl_tolerance, config.min_item_separation, config.narrow_concavity_cutoff_ratio);
        let instance = jagua_rs::probs::spp::io::import_instance(&importer, &external)
            .expect(crate::dmsg!("failed to import fu.json"));
        let builder = LBFBuilder::new(
            instance.clone(),
            Xoshiro256PlusPlus::seed_from_u64(seed),
            LBF_SAMPLE_CONFIG,
            10.0,
            1.0,
        )
        .construct();

        let mut sep = Separator::new(
            instance,
            builder.prob,
            Xoshiro256PlusPlus::seed_from_u64(seed + 1),
            config.expl_cfg.separator_config,
        );

        let stuck_width = sep.prob.strip_width() * (1.0 - shrink);
        sep.change_strip_width(stuck_width, None);
        assert!(
            sep.ct.get_total_loss() > 0.0,
            "좁히면 충돌이 생겨야 테스트가 의미 있다"
        );

        let start_sol = sep.prob.save();
        (sep, start_sol, stuck_width)
    }

    /// 진동은 남은 여유의 절반씩만 넓힌다. 한 걸음(0.165%) 줄인 폭에서 시작하면 다섯 번의 진동 뒤에
    /// 최선 폭의 0.01% 안에 들어오고, 그때부터는 넓힐 여유가 없다고 본다.
    #[test]
    fn oscillation_runs_out_of_headroom_after_a_few_relaxations() {
        use super::{oscillation_relax_width, stalled_at_best};

        // 실측에서 가져온 폭: 최선 12730.539, 한 걸음 줄인 12709.502 에서 정체.
        let best = 12730.539f32;
        let mut width = 12709.502f32;
        let mut relaxations = 0;
        while !stalled_at_best(best, width) {
            width = oscillation_relax_width(width, best, 0.001, 3.0).unwrap();
            relaxations += 1;
        }
        // 0.165% -> 0.083 -> 0.041 -> 0.021 -> 0.0103 -> 0.0052%
        assert_eq!(relaxations, 5);
        assert!(best - width < 0.7, "{width}");

        // 한 걸음 줄인 폭은 정체가 아니고, 최선 폭 자체와 그 바로 아래는 정체다.
        assert!(!stalled_at_best(best, 12709.502));
        assert!(stalled_at_best(best, best));
        assert!(stalled_at_best(best, 12730.538));
        // 최선보다 넓은 폭(완화 복구가 넓힌 폭)과 비정상 값은 대상이 아니다.
        assert!(!stalled_at_best(best, best + 5.0));
        assert!(!stalled_at_best(f32::NAN, 1.0));
        assert!(!stalled_at_best(100.0, f32::NAN));
    }

    /// 기본 설정의 한 걸음은 «과도한 축소» 가 아니어야 한다 — 문턱에 걸리면 기본 탐색의 재시도·
    /// 진동 흐름이 통째로 바뀐다. 0.1 처럼 크게 준 걸음은 걸려야 한다.
    #[test]
    fn overshoot_threshold_separates_default_steps_from_oversized_ones() {
        use super::{OVERSHOOT_GAP, density_step_mult, overshoot_attempt_cap, shrink_gap};
        use std::time::Duration;

        let cfg = DEFAULT_SWARM_CONFIG.expl_cfg;
        // 밀도 50% 에서도(배율이 가장 클 때) 기본 걸음은 문턱 아래다.
        let widest_default = cfg.shrink_step
            * density_step_mult(
                cfg.shrink_adapt_mode,
                0.5,
                0.5,
                cfg.adaptive_shrink_gain,
                cfg.adaptive_shrink_max_mult,
            );
        assert!(widest_default < OVERSHOOT_GAP, "{widest_default}");
        // WPF 기본 걸음(0.001)도 같은 배율에서 문턱 아래다.
        assert!(0.001 * widest_default / cfg.shrink_step < OVERSHOOT_GAP);
        // 0.1 은 배율이 1 이어도 넘는다.
        assert!(0.1 > OVERSHOOT_GAP);

        assert_eq!(shrink_gap(100.0, 100.0), 0.0);
        assert_eq!(shrink_gap(100.0, 101.0), 0.0);
        assert!((shrink_gap(100.0, 90.0) - 0.1).abs() < 1.0e-6);
        assert_eq!(shrink_gap(f32::NAN, 90.0), 0.0);
        assert_eq!(shrink_gap(0.0, -1.0), 0.0);

        assert_eq!(overshoot_attempt_cap(0), Duration::from_secs(10));
        assert_eq!(overshoot_attempt_cap(156), Duration::from_millis(15_600));
        assert_eq!(overshoot_attempt_cap(200), Duration::from_secs(20));
        assert!(overshoot_attempt_cap(usize::MAX) > Duration::from_secs(10));
    }

    #[test]
    fn adaptive_separation_cap_waits_for_samples_and_follows_the_longest_success() {
        use super::SepCapTuner;
        use std::time::Duration;
        let secs = Duration::from_secs_f32;

        let mut tuner = SepCapTuner::default();
        assert_eq!(tuner.cap(), None);
        // 1초 미만의 성공은 표본이 아니다 — 벽에서 먼 폭의 분리다.
        for _ in 0..10 {
            tuner.record(secs(0.2), None, true);
        }
        assert_eq!(tuner.cap(), None);
        // 표본이 두 개일 때까지는 상한이 없다.
        tuner.record(secs(2.0), None, true);
        tuner.record(secs(3.0), None, true);
        assert_eq!(tuner.cap(), None);
        // 세 번째 표본부터 상한이 생긴다. 가장 긴 성공(3초)의 1.25배는 하한(5초)보다 짧다.
        tuner.record(secs(1.5), None, true);
        assert_eq!(tuner.cap(), Some(SepCapTuner::FLOOR));
        // 더 긴 성공이 상한을 끌어올린다.
        tuner.record(secs(10.0), tuner.cap(), true);
        assert_eq!(tuner.cap(), Some(secs(12.5)));
        // 상한보다 일찍 끝난 실패는 상한을 건드리지 않는다.
        for _ in 0..10 {
            tuner.record(secs(8.0), tuner.cap(), false);
        }
        assert_eq!(tuner.cap(), Some(secs(12.5)));
    }

    #[test]
    fn adaptive_separation_cap_grows_while_capped_failures_pile_up() {
        use super::SepCapTuner;
        use std::time::Duration;
        let secs = Duration::from_secs_f32;

        let mut tuner = SepCapTuner::default();
        for _ in 0..3 {
            tuner.record(secs(8.0), None, true);
        }
        let base = tuner.cap().unwrap();
        assert_eq!(base, secs(10.0));
        // 상한에 걸린 실패 세 번까지는 그대로, 네 번째에서 1.5배.
        for _ in 0..3 {
            tuner.record(base, Some(base), false);
            assert_eq!(tuner.cap(), Some(base));
        }
        tuner.record(base, Some(base), false);
        let grown = tuner.cap().unwrap();
        assert!((grown.as_secs_f32() - 15.0).abs() < 1.0e-3, "{grown:?}");
        // 늘어난 상한 안에서 성공하면 배율은 풀리고, 그 성공이 새 기준이 된다.
        tuner.record(secs(14.0), Some(grown), true);
        assert_eq!(tuner.cap(), Some(secs(17.5)));
        // 실패가 아무리 쌓여도 넘치지 않는다.
        for _ in 0..10_000 {
            tuner.record(secs(1.0e6), Some(secs(1.0)), false);
        }
        assert!(tuner.cap().unwrap() > grown);
    }

    fn relief_config(threshold: usize, step: f32, max_ratio: f32) -> ExplorationConfig {
        ExplorationConfig {
            relief_fail_threshold: threshold,
            relief_step: step,
            relief_max_ratio: max_ratio,
            relief_max_rounds: 10,
            relief_strike_limit: 3,
            ..DEFAULT_SWARM_CONFIG.expl_cfg
        }
    }

    fn translations(sol: &SPSolution) -> Vec<(usize, [f32; 3])> {
        let mut out = sol
            .layout_snapshot()
            .placed_items()
            .values()
            .map(|pi| {
                (
                    pi.item().idx(),
                    [
                        pi.d_transf().rotation(),
                        pi.d_transf().translation().0,
                        pi.d_transf().translation().1,
                    ],
                )
            })
            .collect::<Vec<_>>();
        out.sort_by(|a, b| {
            a.0.cmp(&b.0)
                .then(a.1[1].total_cmp(&b.1[1]))
                .then(a.1[2].total_cmp(&b.1[2]))
        });
        out
    }

    #[test]
    fn hotspot_is_none_without_collisions_and_inside_the_strip_with_them() {
        let (sep, _start, stuck_width) = stuck_separator(11, 0.03);
        let hotspot = collision_hotspot_x(&sep).expect(crate::dmsg!("충돌이 있으면 hot spot이 나와야 한다"));
        assert!(
            hotspot > 0.0 && hotspot < stuck_width,
            "hotspot={hotspot}, width={stuck_width}"
        );
    }

    /// 재출발 창(`restart_window`)은 strike 판정을 바꾸지 않고 인컴번트로 되돌아가는
    /// 지점만 더한다. 따라서 (1) 분리가 정상 종료하고, (2) 반환된 해가 트래커와
    /// 일치하며, (3) 창을 켠 쪽의 최종 loss 가 출발 loss 보다 나빠지지 않아야 한다.
    #[test]
    fn restart_window_keeps_the_separation_contract() {
        for window in [1usize, 7, 50] {
            let (mut sep, start_sol, _) = stuck_separator(7, 0.15);
            let start_loss = sep.ct.get_total_loss();
            sep.config.restart_window = window;
            sep.config.strike_limit = 2;
            sep.config.iter_no_imprv_limit = 40;
            // capmods 하한(아이템 수 × 2.5)을 작게 덮어써 테스트를 짧게 만든다.
            sep.config.iter_floor_override = 40;
            let mut term = BasicTerminator::new();
            term.new_timeout(std::time::Duration::from_secs(20));
            let (sol, ct) = sep.separate(&term, &mut None);
            assert!(
                ct.get_total_loss() <= start_loss,
                "window {window}: loss {} > start {start_loss}",
                ct.get_total_loss()
            );
            sep.rollback(&sol, Some(&ct));
            assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));
            assert_eq!(
                sol.layout_snapshot().placed_items().len(),
                start_sol.layout_snapshot().placed_items().len()
            );
        }
    }

    #[test]
    fn relief_widens_until_the_overlaps_clear() {
        let (mut sep, start_sol, stuck_width) = stuck_separator(11, 0.03);
        let config = relief_config(1, 0.01, 0.05);
        let mut term = BasicTerminator::new();
        term.new_timeout(std::time::Duration::from_secs(60));

        let outcome =
            width_relief_escape(&mut sep, &mut None, &term, &config, &start_sol, stuck_width)
                .expect(crate::dmsg!("설정이 유효하면 시도해야 한다"));

        assert!(outcome.resolved, "복구가 충돌을 해소해야 한다: {outcome:?}");
        assert!(
            outcome.width > stuck_width,
            "폭이 넓어져야 한다: {} -> {}",
            stuck_width,
            outcome.width
        );
        assert!(
            outcome.width <= stuck_width * 1.05 + 1.0e-3,
            "상한(+5%)을 넘으면 안 된다: {}",
            outcome.width
        );
        assert!((sep.prob.strip_width() - outcome.width).abs() < 1.0e-3);
        assert_eq!(sep.ct.get_total_loss(), 0.0);
        assert!(sep.prob.layout().is_collision_free());
        assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));
        // strike 한도는 복구 전으로 복원된다.
        assert_eq!(
            sep.config.strike_limit,
            DEFAULT_SWARM_CONFIG
                .expl_cfg
                .separator_config
                .strike_limit
        );
    }

    #[test]
    fn relief_restores_the_stuck_width_and_solution_when_it_cannot_resolve() {
        // 20% 좁힌 뒤 +0.05%만 허용하면 절대 못 푼다.
        // 라운드/strike는 최소로 줄인다 — 여기서 보려는 건 복구 성능이 아니라 원복이다.
        let (mut sep, start_sol, stuck_width) = stuck_separator(23, 0.20);
        let config = ExplorationConfig {
            relief_max_rounds: 2,
            relief_strike_limit: 1,
            ..relief_config(1, 0.0001, 0.0005)
        };
        // 20% 겹침 상태의 분리는 [profile.test]의 debug-assertions(O(n^2) 트래커 검증)
        // 때문에 매우 비싸다. 여기서 볼 것은 원복이므로 예산을 짧게 준다.
        let mut term = BasicTerminator::new();
        term.new_timeout(std::time::Duration::from_secs(5));

        let outcome =
            width_relief_escape(&mut sep, &mut None, &term, &config, &start_sol, stuck_width)
                .expect(crate::dmsg!("설정이 유효하면 시도해야 한다"));

        assert!(!outcome.resolved);
        assert_eq!(outcome.width, stuck_width);
        assert_eq!(sep.prob.strip_width(), stuck_width);
        assert_eq!(translations(&sep.prob.save()), translations(&start_sol));
        assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));
        assert_eq!(
            sep.config.strike_limit,
            DEFAULT_SWARM_CONFIG
                .expl_cfg
                .separator_config
                .strike_limit
        );
    }

    #[test]
    fn relief_round_budget_bounds_each_expansion_round() {
        // 절대 못 푸는 상태에서 라운드마다 예산만큼만 쓰고 넘어가야 한다.
        let (mut sep, start_sol, stuck_width) = stuck_separator(23, 0.20);
        let rounds = 3;
        let budget = std::time::Duration::from_millis(300);
        let config = ExplorationConfig {
            relief_max_rounds: rounds,
            relief_strike_limit: 5,
            relief_round_budget: budget,
            ..relief_config(1, 0.0001, 0.001)
        };
        // 바깥 예산은 넉넉히 준다 — 라운드 예산이 실제로 끊는지 보려는 것이다.
        let mut term = BasicTerminator::new();
        term.new_timeout(std::time::Duration::from_secs(120));

        let started = std::time::SystemTime::now();
        let outcome =
            width_relief_escape(&mut sep, &mut None, &term, &config, &start_sol, stuck_width)
                .expect(crate::dmsg!("설정이 유효하면 시도해야 한다"));
        let elapsed = started.elapsed().expect(crate::dmsg!("monotonic clock"));

        assert!(!outcome.resolved);
        // 라운드당 예산 + 분리 한 번의 마무리 여유. 예산이 없으면 이보다 훨씬 오래 걸린다.
        assert!(
            elapsed < budget * (rounds as u32) + std::time::Duration::from_secs(10),
            "라운드 예산이 적용되지 않았다: {elapsed:?}"
        );
        assert_eq!(sep.prob.strip_width(), stuck_width);
    }

    #[test]
    fn relief_is_a_no_op_when_disabled_or_misconfigured() {
        let mut term = BasicTerminator::new();
        term.new_timeout(std::time::Duration::from_secs(60));

        for config in [
            // 확장 라운드 0
            ExplorationConfig {
                relief_max_rounds: 0,
                ..relief_config(1, 0.01, 0.05)
            },
            // 확장 비율 0
            relief_config(1, 0.0, 0.05),
            // 상한 0
            relief_config(1, 0.01, 0.0),
        ] {
            let (mut sep, start_sol, stuck_width) = stuck_separator(37, 0.05);
            let before = sep.prob.save();
            assert!(
                width_relief_escape(&mut sep, &mut None, &term, &config, &start_sol, stuck_width)
                    .is_none()
            );
            assert_eq!(sep.prob.strip_width(), stuck_width);
            assert_eq!(translations(&sep.prob.save()), translations(&before));
        }
    }

    #[test]
    fn oscillation_relaxes_repeatedly_without_reaching_best_width() {
        let best = 100.0;
        let mut current = 99.0;

        for _ in 0..6 {
            let next = oscillation_relax_width(current, best, 0.01, 3.0)
                .expect(crate::dmsg!("each failure interval should retain usable headroom"));
            assert!(next > current);
            assert!(next < best);
            current = next;
        }
    }

    #[test]
    fn oscillation_uses_configured_expansion_when_headroom_is_sufficient() {
        let target = oscillation_relax_width(80.0, 100.0, 0.01, 2.0).unwrap();
        assert!((target - 81.6).abs() < 1e-5);
    }

    #[test]
    fn oscillation_rejects_degenerate_and_non_finite_inputs() {
        for args in [
            (100.0, 100.0, 0.01, 3.0),
            (101.0, 100.0, 0.01, 3.0),
            (99.0, 100.0, 0.0, 3.0),
            (99.0, 100.0, 0.01, 0.0),
            (f32::NAN, 100.0, 0.01, 3.0),
            (99.0, f32::INFINITY, 0.01, 3.0),
            (99.0, 100.0, f32::NAN, 3.0),
            (99.0, 100.0, 0.01, f32::INFINITY),
        ] {
            assert_eq!(
                oscillation_relax_width(args.0, args.1, args.2, args.3),
                None
            );
        }
    }

    #[test]
    fn feedback_is_gated_by_the_mode_not_by_the_decay_value() {
        let base = DEFAULT_SWARM_CONFIG.expl_cfg;
        // 기본 모드는 밀도 기반이므로 피드백은 꺼져 있다.
        assert_eq!(base.shrink_adapt_mode, ShrinkAdaptMode::DensityLegacy);
        assert_eq!(shrink_feedback_params(&base), None);

        for mode in [
            ShrinkAdaptMode::Fixed,
            ShrinkAdaptMode::DensityLegacy,
            ShrinkAdaptMode::DensitySlack,
        ] {
            let cfg = ExplorationConfig {
                shrink_adapt_mode: mode,
                ..base
            };
            assert_eq!(shrink_feedback_params(&cfg), None, "{mode:?}");
        }

        for mode in [
            ShrinkAdaptMode::Feedback,
            ShrinkAdaptMode::DensitySlackFeedback,
        ] {
            let cfg = ExplorationConfig {
                shrink_adapt_mode: mode,
                shrink_fail_decay: 0.5,
                shrink_scale_min: 0.02,
                ..base
            };
            assert_eq!(shrink_feedback_params(&cfg), Some((0.5, 0.02)), "{mode:?}");
        }
    }

    #[test]
    fn out_of_range_feedback_values_fall_back_to_the_defaults() {
        let base = ExplorationConfig {
            shrink_adapt_mode: ShrinkAdaptMode::Feedback,
            ..DEFAULT_SWARM_CONFIG.expl_cfg
        };
        let (def_decay, def_min) = (base.shrink_fail_decay, base.shrink_scale_min);

        // 0이나 음수를 스텝에 곱하면 축소가 멈추거나 폭이 커진다 — 통과시키면 안 된다.
        for decay in [0.0, -0.5, 1.0, 1.5, f32::NAN, f32::INFINITY] {
            let cfg = ExplorationConfig {
                shrink_fail_decay: decay,
                ..base
            };
            assert_eq!(
                shrink_feedback_params(&cfg),
                Some((def_decay, def_min)),
                "decay = {decay}"
            );
        }

        for min_scale in [0.0, -0.1, 1.5, f32::NAN] {
            let cfg = ExplorationConfig {
                shrink_scale_min: min_scale,
                ..base
            };
            assert_eq!(
                shrink_feedback_params(&cfg),
                Some((def_decay, def_min)),
                "min_scale = {min_scale}"
            );
        }
    }

    #[test]
    fn density_legacy_multiplier_is_bit_for_bit_the_previous_formula() {
        let (gain, max_mult) = (4.0f32, 5.0f32);
        for density in [0.0f32, 0.5, 0.7, 0.84, 0.97, 1.0] {
            let expected = (1.0 + gain * (1.0 - density)).clamp(1.0, max_mult);
            let actual = density_step_mult(
                ShrinkAdaptMode::DensityLegacy,
                density,
                0.60,
                gain,
                max_mult,
            );
            assert!((actual - expected).abs() < 1.0e-6, "density = {density}");
        }
    }

    #[test]
    fn density_slack_spans_the_full_max_mult_range_from_its_own_start() {
        let (gain, max_mult, start) = (1.0f32, 5.0f32, 0.60f32);
        let at =
            |d: f32| density_step_mult(ShrinkAdaptMode::DensitySlack, d, start, gain, max_mult);

        // 출발 시점에는 max_mult, 이론 한계(밀도 1.0)에서는 1.0.
        assert!((at(start) - max_mult).abs() < 1.0e-5);
        assert!((at(1.0) - 1.0).abs() < 1.0e-5);
        // gain=1이면 진행도에 대해 선형이다.
        assert!((at(0.80) - 3.0).abs() < 1.0e-5, "at(0.80) = {}", at(0.80));

        // 단조 감소이며 항상 [1, max_mult] 안에 있다.
        let mut previous = f32::INFINITY;
        for step in 0..=40 {
            let d = start + (1.0 - start) * (step as f32 / 40.0);
            let m = at(d);
            assert!((1.0..=max_mult).contains(&m), "d={d}, m={m}");
            assert!(m <= previous + 1.0e-6, "d={d}, m={m}, prev={previous}");
            previous = m;
        }

        // 서로 다른 절대 밀도에서 출발해도 같은 진행도면 같은 배율이다
        // (기존 Legacy 식이 못 하던 것 — 인스턴스 간 비교 가능성).
        let half_a = density_step_mult(ShrinkAdaptMode::DensitySlack, 0.80, 0.60, gain, max_mult);
        let half_b = density_step_mult(ShrinkAdaptMode::DensitySlack, 0.60, 0.20, gain, max_mult);
        assert!((half_a - half_b).abs() < 1.0e-5);
    }

    #[test]
    fn modes_without_density_always_report_a_neutral_multiplier() {
        for mode in [ShrinkAdaptMode::Fixed, ShrinkAdaptMode::Feedback] {
            for density in [0.0f32, 0.5, 0.9] {
                assert_eq!(
                    density_step_mult(mode, density, 0.6, 4.0, 5.0),
                    1.0,
                    "{mode:?} @ {density}"
                );
            }
        }
        // 시작 밀도가 이미 이론 한계면 정규화가 불가능하므로 중립값으로 물러난다.
        assert_eq!(
            density_step_mult(ShrinkAdaptMode::DensitySlack, 1.0, 1.0, 1.0, 5.0),
            1.0
        );
        assert_eq!(
            density_step_mult(ShrinkAdaptMode::DensitySlack, f32::NAN, 0.6, 1.0, 5.0),
            1.0
        );
    }

    #[test]
    fn mode_codes_round_trip_through_the_c_abi_encoding() {
        for mode in [
            ShrinkAdaptMode::Fixed,
            ShrinkAdaptMode::DensityLegacy,
            ShrinkAdaptMode::DensitySlack,
            ShrinkAdaptMode::Feedback,
            ShrinkAdaptMode::DensitySlackFeedback,
        ] {
            assert_eq!(ShrinkAdaptMode::from_code(mode.code()), Some(mode));
        }
        for unknown in [-1, 5, 99] {
            assert_eq!(ShrinkAdaptMode::from_code(unknown), None);
        }
    }

    /// 탐색 루프가 배율에 적용하는 규칙(실패 시 ×decay, 성공 시 ÷decay)이
    /// 하한과 상한 안에서 대칭적으로 움직이는지 확인한다.
    #[test]
    fn shrink_feedback_backs_off_geometrically_and_recovers_to_one() {
        let (decay, min_scale) = (0.5f32, 0.05f32);
        let mut scale = 1.0f32;

        // 서로 다른 폭에서 연속으로 막히면 기하급수적으로 물러난다.
        for expected in [0.5, 0.25, 0.125] {
            scale = (scale * decay).max(min_scale);
            assert!((scale - expected).abs() < 1.0e-6, "scale = {scale}");
        }

        // 하한 아래로는 내려가지 않는다.
        for _ in 0..20 {
            scale = (scale * decay).max(min_scale);
        }
        assert!((scale - min_scale).abs() < 1.0e-6, "scale = {scale}");

        // 성공은 한 번에 한 단계씩 되돌리고 1.0을 넘지 않는다.
        let mut recoveries = 0;
        while scale < 1.0 {
            scale = (scale / decay).min(1.0);
            recoveries += 1;
            assert!(recoveries < 100, "회복이 수렴하지 않는다");
        }
        assert_eq!(scale, 1.0);
    }

    #[test]
    fn relief_expands_step_by_step_up_to_the_ratio_ceiling() {
        let stuck = 100.0;
        let mut w = stuck;
        let mut rounds = 0;

        while let Some(next) = relief_next_width(w, stuck, 0.002, 0.03) {
            assert!(next > w);
            assert!(next <= stuck * 1.03 + 1.0e-4);
            w = next;
            rounds += 1;
            assert!(rounds < 1000, "relief must terminate at its ceiling");
        }

        // 0.2%씩 늘리면 3% 천장까지 15라운드 안팎이다.
        assert!((w - stuck * 1.03).abs() < 1.0e-3, "w = {w}");
        assert!((14..=16).contains(&rounds), "rounds = {rounds}");
    }

    #[test]
    fn relief_ignores_the_best_feasible_width() {
        // 진동과 달리 best_width를 인자로 받지 않는다 — 막힌 폭 기준 비율만이 상한이다.
        let target = relief_next_width(100.0, 100.0, 0.01, 0.03).unwrap();
        assert!((target - 101.0).abs() < 1.0e-4);
    }

    #[test]
    fn relief_clamps_the_last_round_to_the_ceiling() {
        // 남은 여유(0.5%)보다 스텝(2%)이 크면 천장으로 잘린다.
        let target = relief_next_width(100.5, 100.0, 0.02, 0.01).unwrap();
        assert!((target - 101.0).abs() < 1.0e-4);
    }

    #[test]
    fn relief_rejects_degenerate_and_non_finite_inputs() {
        for args in [
            (103.0, 100.0, 0.002, 0.03), // 이미 천장
            (99.0, 100.0, 0.002, 0.03),  // 막힌 폭보다 좁다
            (100.0, 100.0, 0.0, 0.03),   // 스텝 없음
            (100.0, 100.0, 0.002, 0.0),  // 여유 없음
            (f32::NAN, 100.0, 0.002, 0.03),
            (100.0, f32::INFINITY, 0.002, 0.03),
            (100.0, 100.0, f32::NAN, 0.03),
            (100.0, 100.0, 0.002, f32::INFINITY),
            (0.0, 0.0, 0.002, 0.03),
        ] {
            assert_eq!(
                relief_next_width(args.0, args.1, args.2, args.3),
                None,
                "args = {args:?}"
            );
        }
    }

    #[test]
    fn relief_contains_finite_multiplication_overflow() {
        assert_eq!(
            relief_next_width(1.0e38, 1.0e38, f32::MAX, f32::MAX),
            None,
            "천장이 f32 범위를 넘으면 확장하지 않는다"
        );
    }

    #[test]
    fn oscillation_contains_finite_multiplication_overflow() {
        let target = oscillation_relax_width(1.0e38, 2.0e38, f32::MAX, f32::MAX).unwrap();
        assert!(target > 1.0e38);
        assert!(target < 2.0e38);
        assert!(target.is_finite());
    }
}
