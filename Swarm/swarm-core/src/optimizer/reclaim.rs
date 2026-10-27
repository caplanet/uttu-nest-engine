//! 폴리시가 남긴 예산을 **압축 재진입**으로 회수한다.
//!
//! # 문제
//!
//! 단계마다 자기 `time_limit` 으로 timeout 을 걸고, 안 쓴 시간은 버려진다. 그런데
//! 탐색과 압축은 언제나 예산을 100% 쓴다 — 압축의 `step >= shrink_stop` 가드는
//! 수렴 판정이 아니라 같은 길이의 두 번째 타이머다(`compress.rs`). **잔여는 오직
//! 폴리시에서만 생기고, 폴리시는 마지막 단계라 넘겨줄 다음이 없다.**
//!
//! # 왜 압축으로 되돌리나
//!
//! 폴리시를 더 돌리는 것은 소득이 없다. 같은 압축 결과에서 출발한 실측:
//! 라운드 100 도, LP 반복 400 도, 정확 NFP 홀채움(예산 100% 소진)도 폭이
//! **완전히 동일**했고, 신뢰 영역을 키우면 오히려 나빠졌다. 폴리시가 도달한
//! 지점은 해상도 문제도 반복 부족도 아닌 진짜 고정점이다.
//!
//! 압축은 다르다. 재진입하면 **더 좁아진 폭에서 축소를 다시 건다.**
//!
//! 다만 폴리시의 LP 가 강해질수록 그 뒤의 배치는 접촉이 전부 맞물려 압축이 통하지 않는다
//! (실측: wsjk156·babel 에서 채택 0). 그래서 압축 재진입은 시도마다 시간 상한을 두고 연속
//! 실패에서 물러난다([`compress_reentry`]) — 통하지 않는 배치에서 남은 예산을 다 태우지 않는다.
//!
//! # 안전 성질
//!
//! * **총 소요 시간이 늘지 않는다.** 마감은 호출자가 폴리시에 이미 준 예산의 끝이다.
//! * **결과가 나빠지지 않는다.** 폭이 실제로 줄었을 때만 교체한다. 인증기
//!   (`solution_validation_error`)는 실행 가능성만 보고 **폭은 보지 않으므로**,
//!   폭 게이트는 여기서 직접 건다.
//! * **끄면 한 비트도 안 바뀐다.** 비활성일 때 `sep` 의 난수를 건드리지 않고
//!   즉시 반환한다 — `attempt_to_compress` 가 시도마다 `sep.rng` 를 뽑으므로
//!   한 번만 새어도 이후 스트림이 어긋난다.

use crate::config::{
    CompressionConfig, ExplorationConfig, NfpIntegrationConfig, PolishConfig, ReclaimConfig,
    ReclaimMode, ReclaimStats,
};
use crate::lp_solver::LpSolverConfig;
use crate::optimizer::separator::Separator;
use crate::util::terminator::Terminator;
use jagua_rs::Instant;
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use std::time::Duration;

/// 사이클 하나의 예산을 압축/폴리시로 쪼갤 때 폴리시가 받는 최소 몫.
///
/// `compress_share` 를 1.0 으로 줘도 폴리시가 0을 받으면 압축 결과를 다듬지 못해
/// 재진입의 절반이 죽는다.
const MIN_POLISH_SHARE: f32 = 0.05;

/// 압축 몫을 쓸 수 있는 값으로 만든다.
///
/// **NaN 을 따로 막아야 한다.** `f32::clamp` 는 NaN 을 그대로 통과시키고,
/// `Duration::mul_f32(NaN)` 은 패닉한다. C 경계에서 온 값이므로 신뢰할 수 없다.
fn sanitize_share(share: f32) -> f32 {
    if share.is_finite() {
        share.clamp(0.0, 1.0 - MIN_POLISH_SHARE)
    } else {
        DEFAULT_COMPRESS_SHARE
    }
}

/// `compress_share` 가 유한하지 않을 때 쓸 값.
const DEFAULT_COMPRESS_SHARE: f32 = 0.75;

/// 해 목록에서 가장 좁은 것.
fn narrowest(sols: &[SPSolution]) -> Option<&SPSolution> {
    sols.iter()
        .min_by(|a, b| a.strip_width().total_cmp(&b.strip_width()))
}

/// 회수 사이클 한 조각의 터미네이터 — **조각 마감과 바깥 정지 요청을 함께 본다.**
///
/// # 왜 `BasicTerminator` 를 쓰면 안 되는가 (실제로 겪은 결함)
///
/// 예전에는 조각마다 `BasicTerminator::new()` 에 시간만 걸어 안쪽 압축·폴리시·R&R·
/// 탐색에 넘겼다. 그 터미네이터는 **바깥 정지 플래그를 모른다.** 바깥 플래그는
/// 사이클 **시작** 시점에만 확인됐으므로, 사용자가 폴리시 도중 정지(`StopWait`)를
/// 누르면 현재 사이클의 조각 시간(남은 폴리시 예산 전부)이 다할 때까지 멈추지
/// 않았다. 폴리시 예산을 크게 준 실행에서 "정지가 굉장히 오래 걸린다" 가 그것이다.
///
/// `kill()` 이 바깥의 `stop_requested()` 를 보는 이유는 문서대로 **`kill()` 이 아니라**
/// `stop_requested()` 여야 하기 때문이다 — 바깥 폴리시 터미네이터의 시계는 회수가
/// 시작될 때 이미 만료돼 있어서, `outer.kill()` 을 보면 조각이 시작조차 못 한다.
struct SliceTerminator<'a, T: Terminator + ?Sized> {
    outer: &'a T,
    deadline: Instant,
}

impl<'a, T: Terminator + ?Sized> SliceTerminator<'a, T> {
    fn new(outer: &'a T, slice: Duration) -> Self {
        Self {
            outer,
            deadline: Instant::now() + slice,
        }
    }
}

impl<T: Terminator + ?Sized> Terminator for SliceTerminator<'_, T> {
    fn kill(&self) -> bool {
        self.outer.stop_requested() || Instant::now() >= self.deadline
    }
    fn stop_requested(&self) -> bool {
        self.outer.stop_requested()
    }
    fn new_timeout(&mut self, timeout: Duration) {
        self.deadline = Instant::now() + timeout;
    }
    fn timeout_at(&self) -> Option<Instant> {
        Some(self.deadline)
    }
}

/// 압축 재진입에서 축소 시도 하나가 쓸 수 있는 시간.
///
/// # 왜 상한이 필요한가
///
/// 분리는 «개선 없는 반복» 으로만 포기하는데, capmods 의 반복 하한이 `조각 수 x 2.5` 라
/// (156조각이면 390라운드 x strike 5) 벽에 닿은 배치에서는 실패하는 시도 하나가 조각 예산
/// 전부를 태운다. 실측(wsjk156, Polish 60s): 폴리시가 6 ~ 12초에 끝난 뒤 0.05% 축소 한 번이
/// 36 ~ 40초를 돌고 실패했다 — 회수 예산의 전부였고 채택은 0 이었다.
///
/// # 왜 10초인가
///
/// 압축 단계 로그 7개(wsjk156 5 · babel 200조각 2)의 시도 126회를 보면, 성공 105회 중 103회가
/// 10초 안에 끝났고(대부분 1초 미만, 나머지 2회는 16초·20초) 실패 21회는 중앙값 23초였다.
/// 10초를 넘긴 시도는 거의 전부 실패다.
const ATTEMPT_CAP: Duration = Duration::from_secs(10);

/// 압축 재진입이 물러나는 연속 실패 수. 실패하면 축소 폭을 반으로 줄여 한 번 더 본다.
///
/// 폴리시 뒤의 배치는 모든 접촉이 LP 마진으로 맞물려 있어 아주 작은 축소도 풀리지 않는
/// 경우가 많다(wsjk156: 0.05% ~ 0.002% 축소 15회가 8초 안에 전부 실패). 그런 배치에서는
/// 더 시도해도 같은 결과이므로 두 번에서 끊는다 — 회수가 최대 `2 x ATTEMPT_CAP` 에 끝난다.
const MAX_FAILED_ATTEMPTS: usize = 2;

/// 압축 재진입이 처음 시도하는 축소 폭의 상한(비율). 네이티브 기본 `shrink_range` 의 시작값이다.
///
/// 축소 폭은 `shrink_range` 의 시작값에서 출발하는데, 호스트가 그 값을 크게 잡으면(예: 0.1 = 10%)
/// 폴리시가 끝난 배치에서 10% → 5% 두 번을 시도하다 실패하고 [`MAX_FAILED_ATTEMPTS`] 에 걸려
/// 회수가 아무것도 못 하고 끝난다. 폴리시 뒤에 통하는 축소는 실측에서 0.05% 이하였으므로
/// 시작값을 거기서 자른다. 더 작게 설정한 호출자는 그대로다.
const START_STEP_CAP: f32 = 0.0005;

/// 압축 재진입의 (연속 실패 한도, 시도 시간 상한). 생산 빌드는 위의 두 상수 그대로다.
#[cfg(not(test))]
fn policy() -> (usize, Duration) {
    (MAX_FAILED_ATTEMPTS, ATTEMPT_CAP)
}

/// 테스트 빌드 전용 — A/B 하네스가 두 값을 바꿔 가며 잰다. 0 이면 상수를 쓴다.
///
/// 속성을 `cfg(any(test))` 로 쓴 데는 이유가 있다. 아래 테스트 하나가 이 파일의 «생산 부분» 을
/// 첫 `cfg(test)` 속성 앞까지로 잘라 읽으므로, 그 글자 그대로를 여기에 쓰면 절단이 당겨진다.
#[cfg(any(test))]
static POLICY_OVERRIDE: (std::sync::atomic::AtomicUsize, std::sync::atomic::AtomicU64) = (
    std::sync::atomic::AtomicUsize::new(0),
    std::sync::atomic::AtomicU64::new(0),
);

#[cfg(any(test))]
fn policy() -> (usize, Duration) {
    use std::sync::atomic::Ordering::Relaxed;
    let failed = POLICY_OVERRIDE.0.load(Relaxed);
    let cap_ms = POLICY_OVERRIDE.1.load(Relaxed);
    (
        if failed == 0 {
            MAX_FAILED_ATTEMPTS
        } else {
            failed
        },
        if cap_ms == 0 {
            ATTEMPT_CAP
        } else {
            Duration::from_millis(cap_ms)
        },
    )
}

/// 폴리시 결과 위에서 압축을 다시 건다. 한 번이라도 줄였으면 그 해를, 아니면 `None` 을 준다.
///
/// [`compression_phase`] 를 그대로 부르지 않는 이유는 위의 두 상수다. 그쪽은 시도에 시간
/// 상한이 없고 실패해도 예산이 남는 한 계속 시도한다 — 회수에서는 그것이 낭비의 전부였다.
/// 축소 폭은 `shrink_range` 의 시작값([`START_STEP_CAP`] 이하로 자른다)에서 출발해 실패할 때마다
/// 반으로 줄인다.
///
/// 반환할 때 `sep` 은 돌려준 해(없으면 `best`) 위에 있다.
///
/// [`compression_phase`]: crate::optimizer::compress::compression_phase
fn compress_reentry(
    sep: &mut Separator,
    best: &SPSolution,
    cmpr_config: &CompressionConfig,
    slice: Duration,
    outer: &impl Terminator,
) -> Option<SPSolution> {
    let (shrink_start, shrink_stop) =
        crate::config::normalize_shrink_range(cmpr_config.shrink_range)?;
    let deadline = Instant::now() + slice;
    let mut narrower: Option<SPSolution> = None;
    // 종료값이 상한보다 크게 설정돼 있으면 종료값에서 한 번은 시도한다.
    let mut step = shrink_start.min(START_STEP_CAP).max(shrink_stop);
    let mut failed = 0usize;
    let (max_failed, attempt_cap) = policy();
    while failed < max_failed && step >= shrink_stop && !outer.stop_requested() {
        let Some(left) = deadline.checked_duration_since(Instant::now()) else {
            break;
        };
        if left.is_zero() {
            break;
        }
        let term = SliceTerminator::new(outer, left.min(attempt_cap));
        let from = narrower.as_ref().unwrap_or(best);
        match crate::optimizer::compress::attempt_to_compress(sep, from, step, &term, &mut None) {
            Some(sol) => {
                narrower = Some(sol);
                failed = 0;
            }
            None => {
                failed += 1;
                step *= 0.5;
            }
        }
    }
    sep.rollback(narrower.as_ref().unwrap_or(best), None);
    narrower
}

/// 남은 예산이 다할 때까지 압축↔폴리시를 반복한다.
///
/// `deadline` 은 **폴리시 단계의 마감**이다(폴리시 시작 + `polish_config.time_limit`).
/// 이 함수는 그 시각을 넘기지 않는다.
///
/// `outer` 는 외부 취소(사용자 중단)를 보는 터미네이터다. 시계는 여기서 따로 재므로
/// `outer.stop_requested()` 만 본다 — `kill()` 을 보면 이미 만료된 폴리시
/// 터미네이터 때문에 루프가 시작조차 못 한다.
#[allow(clippy::too_many_arguments)]
pub fn reclaim_leftover(
    instance: &SPInstance,
    sep: &mut Separator,
    certified: SPSolution,
    deadline: Instant,
    cfg: &ReclaimConfig,
    cmpr_config: &CompressionConfig,
    expl_config: &ExplorationConfig,
    polish_config: &PolishConfig,
    nfp_cfg: &NfpIntegrationConfig,
    escape_config: &crate::escape::AdaptiveEscapeConfig,
    lp_solver: &LpSolverConfig,
    outer: &impl Terminator,
) -> (SPSolution, ReclaimStats) {
    let mut stats = ReclaimStats {
        width_before: certified.strip_width(),
        width_after: certified.strip_width(),
        ..ReclaimStats::default()
    };
    // 끄면 아무것도 하지 않는다 — 특히 `sep.rng` 를 건드리지 않는다.
    if !cfg.enabled {
        return (certified, stats);
    }
    sep.configure_reinsertion(nfp_cfg);
    // 무개선을 몇 번까지 참을지. 모드의 성격이 정한다.
    let stagnation_limit = match cfg.mode {
        // Conservative retry policy for compression reentry; its split selection
        // is stochastic, so one failed cycle is not a proof of convergence.
        ReclaimMode::CompressReentry | ReclaimMode::RuinRecreate => 1,
        // 확률적이다 — 한 번 못 찾았다고 수렴한 게 아니다. 첫 실패에 멈추면
        // 예산을 크게 남긴다(실측: R&R 이 60초 중 24초만 쓰고 멈췄다).
        _ => cfg.stagnation_cycles.max(1),
    };
    let mut stagnant = 0usize;
    let min_slice = Duration::from_millis(cfg.min_slice_ms.max(1));
    let mut best = certified;
    let mut cycle = 0usize;

    loop {
        if cfg.max_cycles != 0 && cycle >= cfg.max_cycles {
            break;
        }
        if outer.stop_requested() {
            break;
        }
        let Some(remaining) = deadline.checked_duration_since(Instant::now()) else {
            break;
        };
        if remaining < min_slice {
            break;
        }

        // 사이클 예산을 쪼갠다. 압축이 폭을 줄이고 폴리시가 그 결과를 다듬으므로
        // 압축 쪽이 크다. 다만 폴리시 몫이 0이 되지는 않게 한다.
        let share = sanitize_share(cfg.compress_share);
        let cmpr_slice = remaining.mul_f32(share);
        let plsh_slice = remaining - cmpr_slice;
        cycle += 1;

        // ── 사이클 전반부: 모드가 정하는 연산자 ──
        //
        // 어느 모드든 출발점은 현재 최선이고, 끝나면 `compressed` 에 후보가 담긴다.
        // 실행 불가능한 상태로 나올 수 있으므로 뒤의 폴리시 전에 반드시 검증한다.
        sep.rollback(&best, None);
        // 정지 요청을 보는 조각 터미네이터 — `SliceTerminator` 문서 참조.
        let term = SliceTerminator::new(outer, cmpr_slice);

        let compressed = match cfg.mode {
            // 정제 계열 — 더 좁아진 폭에서 축소를 다시 건다. 시도마다 시간 상한을 걸고
            // 연속 실패에서 물러난다([`compress_reentry`]).
            ReclaimMode::CompressReentry | ReclaimMode::RuinRecreate => {
                match compress_reentry(sep, &best, cmpr_config, cmpr_slice, outer) {
                    Some(narrower) => narrower,
                    None => {
                        // 한 번도 못 줄였다. 다듬을 것이 없다 — 폴리시는 같은 해에서 이미
                        // 고정점까지 갔으므로 다시 돌려도 같은 자리다. 남은 예산은 쓰지 않고 돌려준다.
                        stagnant += 1;
                        stats.cycles = cycle;
                        stats.stagnant = stagnant;
                        if stagnant >= stagnation_limit || outer.stop_requested() {
                            break;
                        }
                        continue;
                    }
                }
            }
            // 정체를 감지해 단계적으로 강한 교란을 거는 탐색에 잔여를 돌린다.
            ReclaimMode::Escape => {
                let mut expl = *expl_config;
                expl.time_limit = cmpr_slice;
                let resumed = crate::escape::pipeline::escape_driven_exploration_phase_with_nfp(
                    instance,
                    sep,
                    &mut None,
                    &term,
                    &expl,
                    escape_config,
                    nfp_cfg,
                );
                narrowest(&resumed).cloned().unwrap_or_else(|| best.clone())
            }
        };

        if outer.stop_requested() {
            // 전반부가 정지로 잘렸다. 다듬을 것도, 채택할 것도 없다 — 최선을 유지한다.
            // 시작한 사이클은 통계에 남긴다 — 정지 지연 테스트가 "사이클이 돌던 중에
            // 멈췄는가" 를 이 값으로 확인한다.
            stats.cycles = cycle;
            stats.stagnant = stagnant;
            break;
        }

        // ── 압축 결과를 다듬는다 ──
        let mut plsh = polish_config.clone();
        plsh.time_limit = plsh_slice;
        // 중첩 회수는 금지한다. 여기서 또 회수를 켜면 재귀가 된다.
        plsh.reclaim.enabled = false;
        sep.rollback(&compressed, None);
        let term = SliceTerminator::new(outer, plsh_slice);
        let polished = crate::optimizer::polish::polish_phase_with_nfp(
            instance,
            sep,
            &compressed,
            &mut None,
            &term,
            &plsh,
            nfp_cfg,
            lp_solver,
        );

        // ── 폭 게이트 ──
        //
        // 인증기는 실행 가능성만 보고 폭은 보지 않는다. 좁아졌을 때만 받는다.
        let improved = polished.strip_width() < best.strip_width()
            && crate::util::assertions::frozen_poses_preserved(&best, &polished, &sep.frozen_mask)
            && crate::structure::pipeline::solution_validation_error_with_frozen_and_groups(
                instance,
                &polished,
                &sep.frozen_mask,
                &sep.place_groups,
            )
            .is_none();
        if improved {
            best = polished;
            stats.accepted += 1;
            stagnant = 0;
        } else {
            stagnant += 1;
        }
        stats.cycles = cycle;
        stats.stagnant = stagnant;
        if stagnant >= stagnation_limit {
            break;
        }
    }

    // 마지막에 반드시 최선으로 맞춰 둔다. 루프가 개선 없는 사이클로 끝났다면
    // separator 는 그 버려진 해 위에 있다.
    sep.rollback(&best, None);

    stats.width_after = best.strip_width();
    (best, stats)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::DEFAULT_SWARM_CONFIG;

    fn cfg() -> ReclaimConfig {
        DEFAULT_SWARM_CONFIG.polish_cfg.reclaim
    }

    /// **기본은 켜짐이다.** 폴리시가 남긴 예산이 그냥 버려지는 것이 기본값이면
    /// 안 된다는 판단이다.
    ///
    /// 끄는 쪽이 안전판이므로, 껐을 때 `sep` 을 건드리지 않고 즉시 반환하는지도
    /// 함께 지킨다 — `attempt_to_compress` 가 시도마다 `sep.rng` 를 뽑으므로
    /// 한 번만 새어도 이후 난수 스트림이 어긋난다.
    #[test]
    fn reclaim_is_on_by_default() {
        assert!(cfg().enabled);
        // Actual no-op behavior (poses, width, and RNG) is covered by the
        // parallel_optimization integration test, without matching source text.
    }

    /// 폴리시 몫이 0이 되면 압축 결과를 다듬지 못해 재진입의 절반이 죽는다.
    /// `compress_share` 를 1.0 으로 줘도 폴리시가 최소 몫을 받아야 한다.
    #[test]
    fn the_compress_share_always_leaves_something_for_polish() {
        for share in [
            0.0f32,
            0.5,
            0.75,
            1.0,
            2.0,
            -1.0,
            f32::NAN,
            f32::INFINITY,
            f32::NEG_INFINITY,
        ] {
            let clamped = sanitize_share(share);
            assert!(
                clamped.is_finite(),
                "share {share} 가 유한하지 않은 값 {clamped} 로 나왔다 —                  Duration::mul_f32 이 패닉한다"
            );
            assert!(
                (0.0..=1.0 - MIN_POLISH_SHARE).contains(&clamped),
                "share {share} 가 {clamped} 로 정규화됐다 — 폴리시 몫이 사라진다"
            );
        }
    }

    /// 상한 조합이 유한하게 끝나는지. `max_cycles` 가 0이면 예산이,
    /// 0이 아니면 사이클 수가 상한이다 — 둘 다 없으면 무한 루프다.
    /// 모드가 **압축 재진입**이 기본이어야 한다. 사용자가 명시한 기본값이다.
    #[test]
    fn the_default_mode_is_compress_reentry() {
        assert_eq!(cfg().mode, ReclaimMode::CompressReentry);
        assert_eq!(ReclaimMode::default(), ReclaimMode::CompressReentry);
    }

    /// C ABI 정수 대응. 범위 밖은 기본값으로 — 경계에서 프로세스를 죽이지 않는다.
    #[test]
    fn mode_from_raw_maps_abi_values_and_clamps_out_of_range() {
        assert_eq!(ReclaimMode::from_raw(0), ReclaimMode::CompressReentry);
        assert_eq!(ReclaimMode::from_raw(1), ReclaimMode::RuinRecreate);
        assert_eq!(ReclaimMode::from_raw(2), ReclaimMode::Escape);
        for v in [-1, 3, 99, i32::MIN, i32::MAX] {
            assert_eq!(ReclaimMode::from_raw(v), ReclaimMode::CompressReentry);
        }
    }

    /// **정체 허용은 모드의 성격을 따라야 한다.**
    ///
    /// 압축 재진입은 보수적으로 한 번 실패하면 끝낸다. 탈출 계열은
    /// 첫 실패에 멈추면 예산을 크게 남긴다 — 실측에서 R&R 이 60초 중 24초만 쓰고
    /// 36초를 남겼고, 허용을 넣자 -0.01mm 에서 -22.97mm 로 뛰었다.
    #[test]
    fn escape_modes_allow_more_stagnant_cycles_than_compression_reentry() {
        let src = include_str!("reclaim.rs");
        let production = &src[..src.find("#[cfg(test)]").expect(crate::dmsg!("테스트 모듈 경계"))];
        assert!(
            !production.contains("fn stochastic_modes_tolerate"),
            "생산 코드 절단이 잘못됐다"
        );
        // 압축 재진입만 1로 고정되고, 나머지는 설정값을 쓴다.
        assert!(
            production.contains("ReclaimMode::CompressReentry | ReclaimMode::RuinRecreate => 1,"),
            "결정론적 모드가 첫 실패에서 멈추지 않는다"
        );
        assert!(
            production.contains("_ => cfg.stagnation_cycles.max(1),"),
            "확률적 모드가 정체 허용을 쓰지 않는다"
        );
        assert!(
            cfg().stagnation_cycles >= 2,
            "탈출 계열 기본 허용이 1이면 첫 실패에 멈춰 예산을 남긴다"
        );
    }

    /// **저장된 해 위에서 회수 모드 A/B.**
    ///
    /// 탐색·압축은 시간 종료라 같은 시드로도 배치가 재현되지 않는다. 그래서 FullBench 가 남긴
    /// `*.Compress.json` 을 읽어 폴리시를 한 번 돌리고, 그 결과를 모든 팔의 시작 배치로 쓴다.
    ///
    /// 노브: `AB_SOLUTION`(필수, 해 JSON), `AB_INPUT`(기본 wsjk156), `AB_OVERLAP_MM`(기본 0.4),
    /// `AB_RECLAIM_SECS`(기본 40), `AB_RECLAIM_MODES`(기본 `0,1,2` = [`ReclaimMode::from_raw`]),
    /// `AB_SEEDS`(기본 `1`; 쉼표로 여러 개).
    ///
    /// ```powershell
    /// $env:AB_SOLUTION='...\seed1.Compress.json'
    /// cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" `
    ///   --lib reclaim_modes_ab_on_a_saved_layout -- --ignored --nocapture --test-threads=1
    /// ```
    #[test]
    #[ignore]
    fn reclaim_modes_ab_on_a_saved_layout() {
        use crate::util::terminator::BasicTerminator;
        use rand::SeedableRng;
        use rand::rngs::Xoshiro256PlusPlus;

        let Ok(solution_path) = std::env::var("AB_SOLUTION") else {
            println!("[A/B] AB_SOLUTION 이 없어 건너뛴다");
            return;
        };
        let env = |key: &str, default: &str| std::env::var(key).unwrap_or_else(|_| default.into());
        let list = |key: &str, default: &str| -> Vec<u64> {
            env(key, default)
                .split(',')
                .filter_map(|v| v.trim().parse().ok())
                .collect()
        };
        let input = std::env::var("AB_INPUT")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|_| {
                std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input/wsjk156.json")
            });
        let overlap: f32 = env("AB_OVERLAP_MM", "0.4").parse().expect(crate::dmsg!("AB_OVERLAP_MM"));
        let secs: u64 = env("AB_RECLAIM_SECS", "40")
            .parse()
            .expect(crate::dmsg!("AB_RECLAIM_SECS"));

        let (ext, _) = crate::util::io::read_spp_input(&input).expect(crate::dmsg!("입력 읽기"));
        let c = DEFAULT_SWARM_CONFIG;
        let instance = crate::util::io::import_spp_instance(
            &ext,
            c.cde_config,
            c.poly_simpl_tolerance,
            (overlap > 0.0).then_some(-overlap),
            c.narrow_concavity_cutoff_ratio,
        )
        .expect(crate::dmsg!("import"));
        let ext_sol: jagua_rs::probs::spp::io::ext_repr::ExtSPSolution =
            serde_json::from_str(&std::fs::read_to_string(&solution_path).expect(crate::dmsg!("해 읽기")))
                .expect(crate::dmsg!("해 JSON"));
        let start = crate::util::frozen::import_solution(&instance, &ext_sol);
        let start_sol = start.save();
        let new_sep = |seed: u64| {
            let mut sep_cfg = c.cmpr_cfg.separator_config;
            sep_cfg.n_workers = 3;
            Separator::new(
                instance.clone(),
                start.clone(),
                Xoshiro256PlusPlus::seed_from_u64(seed),
                sep_cfg,
            )
        };

        // 폴리시 한 번 — 모든 팔이 이 결과에서 출발한다.
        let mut plsh = c.polish_cfg.clone();
        plsh.reclaim.enabled = false;
        let polished = {
            let mut sep = new_sep(1);
            let mut term = BasicTerminator::new();
            term.new_timeout(plsh.time_limit);
            crate::optimizer::polish::polish_phase_with_nfp(
                &instance,
                &mut sep,
                &start_sol,
                &mut None,
                &term,
                &plsh,
                &c.nfp_integration,
                &crate::lp_solver::DEFAULT_LP_SOLVER_CONFIG,
            )
        };
        println!(
            "\n[A/B] 시작 {:.4} -> 폴리시 {:.4} ({:.3}%)  ({solution_path})",
            start_sol.strip_width(),
            polished.strip_width(),
            polished.density() * 100.0
        );

        // `AB_RECLAIM_POLICIES`: "연속실패한도:시도상한ms" 를 쉼표로. `0:0` = 생산 상수 그대로.
        let policies: Vec<(usize, u64)> = env("AB_RECLAIM_POLICIES", "0:0")
            .split(',')
            .filter_map(|p| {
                let (a, b) = p.trim().split_once(':')?;
                Some((a.parse().ok()?, b.parse().ok()?))
            })
            .collect();
        for (mode, (max_failed, cap_ms)) in list("AB_RECLAIM_MODES", "0,1,2")
            .into_iter()
            .flat_map(|mode| policies.iter().map(move |p| (mode, *p)))
        {
            POLICY_OVERRIDE
                .0
                .store(max_failed, std::sync::atomic::Ordering::Relaxed);
            POLICY_OVERRIDE
                .1
                .store(cap_ms, std::sync::atomic::Ordering::Relaxed);
            println!(
                "[A/B] policy: max consecutive failures {max_failed}, attempt cap {cap_ms}ms (0 = default)"
            );
            for seed in list("AB_SEEDS", "1") {
                let mut sep = new_sep(seed);
                sep.rollback(&polished, None);
                let mut cfg = c.polish_cfg.reclaim;
                cfg.mode = ReclaimMode::from_raw(mode as i32);
                let t = std::time::Instant::now();
                let (best, stats) = reclaim_leftover(
                    &instance,
                    &mut sep,
                    polished.clone(),
                    Instant::now() + Duration::from_secs(secs),
                    &cfg,
                    &c.cmpr_cfg,
                    &c.expl_cfg,
                    &c.polish_cfg,
                    &c.nfp_integration,
                    &c.escape,
                    &crate::lp_solver::DEFAULT_LP_SOLVER_CONFIG,
                    &BasicTerminator::new(),
                );
                println!(
                    "[A/B] {} seed {seed}  {secs}s  폭 {:.4} -> {:.4} ({:+.3}mm, {:+.4}%p)  사이클 {} / 채택 {}  실제 {:.1}s",
                    cfg.mode.label(),
                    polished.strip_width(),
                    best.strip_width(),
                    best.strip_width() - polished.strip_width(),
                    (best.density() - polished.density()) * 100.0,
                    stats.cycles,
                    stats.accepted,
                    t.elapsed().as_secs_f32()
                );
                assert!(
                    best.strip_width() <= polished.strip_width(),
                    "회수가 폭을 늘렸다"
                );
                assert!(
                    sep.solution_is_certified(&best, &polished),
                    "회수 결과 인증 실패"
                );
            }
        }
        POLICY_OVERRIDE
            .0
            .store(0, std::sync::atomic::Ordering::Relaxed);
        POLICY_OVERRIDE
            .1
            .store(0, std::sync::atomic::Ordering::Relaxed);
    }

    #[test]
    fn either_the_cycle_cap_or_the_deadline_bounds_the_loop() {
        let c = cfg();
        assert!(
            c.max_cycles != 0 || c.min_slice_ms > 0,
            "사이클 상한이 없으면 최소 조각이라도 있어야 마감에 수렴한다"
        );
    }
}
