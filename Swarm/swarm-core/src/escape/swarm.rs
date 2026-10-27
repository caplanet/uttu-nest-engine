//! §5. Swarm에 적용할 때의 추가 처리 — **Adaptive Swarm Diversification**
//!
//! 현재 Swarm이 여러 후보 배치를 동시에 탐색하는 구조라면 **모든 후보가 하나의
//! Global Best 주변으로 몰리는 현상**도 감지해야 한다.
//!
//! 후보 다양성이 낮아지면 다음과 같이 처리한다.
//!
//! ```text
//! 상위 10~20% 후보
//! → Elite로 보존
//!
//! 중간 후보
//! → 병목 패턴 부분 재배치
//!
//! 하위 20~30% 후보
//! → 다른 Elite를 기준으로 재초기화
//! ```
//!
//! 그리고 탈출 직후 일정 반복 동안은:
//!
//! ```text
//! Global Best 추종 강도 ↓
//! 랜덤 탐색 및 개별 탐색 강도 ↑
//! ```
//!
//! 로 조정한 뒤, 새로운 개선이 발생하면 정상 파라미터로 복귀한다.
//!
//! 또한 Conflict Burst 문서의 particle 계층:
//!
//! ```text
//! Best 20%
//! → 현재 배치 유지 + 약한 local search
//!
//! Middle 50%
//! → Collision Push + 부분 재삽입
//!
//! Worst 30%
//! → Strip Relaxation + 큰 Destroy & Repair
//! ```
//!
//! 이 모듈은 **분류와 강도 스케줄만** 담당한다. 실제 교란은
//! [`crate::escape::levels::apply_escape`] 와
//! [`crate::escape::conflict_burst::conflict_burst_partial_rebuild`] 가 수행한다.

use crate::escape::{
    PARTICLE_BEST_RATIO, PARTICLE_MIDDLE_RATIO, PARTICLE_WORST_RATIO, POST_ESCAPE_BOOST_ITERS,
    POST_ESCAPE_EXPLORATION_BOOST, POST_ESCAPE_GLOBAL_BEST_PULL, SWARM_ELITE_RATIO_MAX,
    SWARM_ELITE_RATIO_MIN, SWARM_REINIT_RATIO_MAX, SWARM_REINIT_RATIO_MIN, lerp_ratio,
};

/// 후보(particle) 하나에 배정되는 처리 계층.
///
/// ```text
/// 상위 10~20% 후보 → Elite로 보존
/// 중간 후보        → 병목 패턴 부분 재배치
/// 하위 20~30% 후보 → 다른 Elite를 기준으로 재초기화
/// ```
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ParticleTier {
    /// 상위 후보 — 현재 배치를 그대로 보존하고 약한 local search만 돌린다.
    Elite,
    /// 중간 후보 — 병목 패턴을 부분 재배치(Collision Push + 부분 재삽입)한다.
    Middle,
    /// 하위 후보 — 다른 Elite를 기준으로 재초기화(Strip Relaxation + 큰 D&R)한다.
    Reinit,
}

/// 정체 강도에 따라 Elite/Reinit 비율을 조절하는 다양화 정책.
///
/// 정체가 심할수록(t → 1) Elite 구간은 좁아지고(20% → 10%) 재초기화 구간은
/// 넓어진다(20% → 30%).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SwarmDiversification {
    /// 상위 보존 비율. `SWARM_ELITE_RATIO_MIN..=SWARM_ELITE_RATIO_MAX`로 clamp된다.
    pub elite_ratio: f32,
    /// 하위 재초기화 비율. `SWARM_REINIT_RATIO_MIN..=SWARM_REINIT_RATIO_MAX`로 clamp된다.
    pub reinit_ratio: f32,
}

impl Default for SwarmDiversification {
    /// 정체가 없는 상태(t = 0)의 기본 정책.
    fn default() -> Self {
        Self::from_stagnation_intensity(0.0)
    }
}

impl SwarmDiversification {
    /// 두 비율을 문서가 정한 범위로 clamp해서 정책을 만든다.
    ///
    /// NaN이 들어오면 `clamp`가 패닉하므로 먼저 각 범위의 하한으로 치환한다.
    pub fn new(elite_ratio: f32, reinit_ratio: f32) -> Self {
        let elite = if elite_ratio.is_finite() {
            elite_ratio
        } else {
            SWARM_ELITE_RATIO_MIN
        };
        let reinit = if reinit_ratio.is_finite() {
            reinit_ratio
        } else {
            SWARM_REINIT_RATIO_MIN
        };
        Self {
            elite_ratio: elite.clamp(SWARM_ELITE_RATIO_MIN, SWARM_ELITE_RATIO_MAX),
            reinit_ratio: reinit.clamp(SWARM_REINIT_RATIO_MIN, SWARM_REINIT_RATIO_MAX),
        }
    }

    /// 정체 강도 `t ∈ [0, 1]`로 두 비율을 선형 보간한다.
    ///
    /// ```text
    /// elite_ratio  = lerp((MAX, MIN), t)   20% → 10%
    /// reinit_ratio = lerp((MIN, MAX), t)   20% → 30%
    /// ```
    pub fn from_stagnation_intensity(t: f32) -> Self {
        let t = if t.is_finite() { t.clamp(0.0, 1.0) } else { 0.0 };
        Self::new(
            lerp_ratio((SWARM_ELITE_RATIO_MAX, SWARM_ELITE_RATIO_MIN), t),
            lerp_ratio((SWARM_REINIT_RATIO_MIN, SWARM_REINIT_RATIO_MAX), t),
        )
    }

    /// 성능 **내림차순**으로 정렬된 particle 인덱스 순서를 가정하고 계층을 배정한다.
    ///
    /// 상위 `round(n × elite_ratio)`개(최소 1개)가 Elite, 하위
    /// `round(n × reinit_ratio)`개가 Reinit, 나머지가 Middle이다. 두 구간이 겹치면
    /// **Elite를 우선**한다.
    pub fn classify(&self, n_particles: usize) -> Vec<ParticleTier> {
        if n_particles == 0 {
            return Vec::new();
        }
        let n = n_particles;
        let n_elite = ((n as f32 * self.elite_ratio).round() as usize).clamp(1, n);
        // Elite가 이미 가져간 몫은 Reinit이 침범할 수 없다.
        let n_reinit = ((n as f32 * self.reinit_ratio).round() as usize).min(n - n_elite);
        let n_middle = n - n_elite - n_reinit;

        let mut tiers = Vec::with_capacity(n);
        tiers.extend(std::iter::repeat_n(ParticleTier::Elite, n_elite));
        tiers.extend(std::iter::repeat_n(ParticleTier::Middle, n_middle));
        tiers.extend(std::iter::repeat_n(ParticleTier::Reinit, n_reinit));
        debug_assert_eq!(tiers.len(), n);
        tiers
    }

    /// Conflict Burst 문서의 고정 particle 계층.
    ///
    /// ```text
    /// Best 20%   → 현재 배치 유지 + 약한 local search
    /// Middle 50% → Collision Push + 부분 재삽입
    /// Worst 30%  → Strip Relaxation + 큰 Destroy & Repair
    /// ```
    ///
    /// [`Self::classify`]와 달리 정체 강도와 무관한 **고정 비율**이며, 반올림 오차로
    /// 남는 몫은 Worst(= [`ParticleTier::Reinit`])가 흡수한다.
    pub fn conflict_tiers(n_particles: usize) -> Vec<ParticleTier> {
        if n_particles == 0 {
            return Vec::new();
        }
        let n = n_particles;
        let n_best = ((n as f32 * PARTICLE_BEST_RATIO).round() as usize).clamp(1, n);
        let n_worst_target = ((n as f32 * PARTICLE_WORST_RATIO).round() as usize).min(n - n_best);
        let remaining = n - n_best - n_worst_target;
        let n_middle_target = (n as f32 * PARTICLE_MIDDLE_RATIO).round() as usize;
        // 남은 자리가 Middle 목표치보다 많으면 초과분은 Worst로 내린다.
        let n_middle = n_middle_target.min(remaining);
        let n_worst = n - n_best - n_middle;

        let mut tiers = Vec::with_capacity(n);
        tiers.extend(std::iter::repeat_n(ParticleTier::Elite, n_best));
        tiers.extend(std::iter::repeat_n(ParticleTier::Middle, n_middle));
        tiers.extend(std::iter::repeat_n(ParticleTier::Reinit, n_worst));
        debug_assert_eq!(tiers.len(), n);
        tiers
    }

    /// 탈출 직후 Global Best 추종 강도 배율.
    ///
    /// ```text
    /// Global Best 추종 강도 ↓
    /// ```
    ///
    /// 탈출 직후(`iters = 0`)에는 [`POST_ESCAPE_GLOBAL_BEST_PULL`], 이후
    /// [`POST_ESCAPE_BOOST_ITERS`]회에 걸쳐 1.0으로 선형 복귀한다.
    pub fn global_best_pull(&self, iters_since_escape: usize) -> f32 {
        Self::post_escape_ramp(POST_ESCAPE_GLOBAL_BEST_PULL, iters_since_escape)
    }

    /// 탈출 직후 랜덤·개별 탐색 강도 배율.
    ///
    /// ```text
    /// 랜덤 탐색 및 개별 탐색 강도 ↑
    /// ```
    ///
    /// 탈출 직후(`iters = 0`)에는 [`POST_ESCAPE_EXPLORATION_BOOST`], 이후
    /// [`POST_ESCAPE_BOOST_ITERS`]회에 걸쳐 1.0으로 선형 복귀한다.
    pub fn exploration_boost(&self, iters_since_escape: usize) -> f32 {
        Self::post_escape_ramp(POST_ESCAPE_EXPLORATION_BOOST, iters_since_escape)
    }

    /// `start`에서 시작해 [`POST_ESCAPE_BOOST_ITERS`]회 동안 1.0으로 선형 복귀하는 램프.
    #[inline]
    fn post_escape_ramp(start: f32, iters_since_escape: usize) -> f32 {
        if POST_ESCAPE_BOOST_ITERS == 0 || iters_since_escape >= POST_ESCAPE_BOOST_ITERS {
            return 1.0;
        }
        let t = iters_since_escape as f32 / POST_ESCAPE_BOOST_ITERS as f32;
        lerp_ratio((start, 1.0), t)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f32 = 1.0e-6;

    fn approx(a: f32, b: f32) -> bool {
        (a - b).abs() < EPS
    }

    #[test]
    fn new_clamps_into_document_ranges() {
        let d = SwarmDiversification::new(0.0, 0.0);
        assert!(approx(d.elite_ratio, SWARM_ELITE_RATIO_MIN));
        assert!(approx(d.reinit_ratio, SWARM_REINIT_RATIO_MIN));

        let d = SwarmDiversification::new(1.0, 1.0);
        assert!(approx(d.elite_ratio, SWARM_ELITE_RATIO_MAX));
        assert!(approx(d.reinit_ratio, SWARM_REINIT_RATIO_MAX));

        // NaN은 하한으로 치환된다(clamp 패닉 방지).
        let d = SwarmDiversification::new(f32::NAN, f32::NAN);
        assert!(approx(d.elite_ratio, SWARM_ELITE_RATIO_MIN));
        assert!(approx(d.reinit_ratio, SWARM_REINIT_RATIO_MIN));
    }

    #[test]
    fn stagnation_intensity_interpolates_both_ends() {
        // t = 0 : 정체 없음 → Elite 넓게(20%), Reinit 좁게(20%)
        let calm = SwarmDiversification::from_stagnation_intensity(0.0);
        assert!(approx(calm.elite_ratio, SWARM_ELITE_RATIO_MAX));
        assert!(approx(calm.reinit_ratio, SWARM_REINIT_RATIO_MIN));

        // t = 1 : 정체 극심 → Elite 좁게(10%), Reinit 넓게(30%)
        let stuck = SwarmDiversification::from_stagnation_intensity(1.0);
        assert!(approx(stuck.elite_ratio, SWARM_ELITE_RATIO_MIN));
        assert!(approx(stuck.reinit_ratio, SWARM_REINIT_RATIO_MAX));

        // 중간값은 정확히 두 끝의 평균
        let mid = SwarmDiversification::from_stagnation_intensity(0.5);
        assert!(approx(
            mid.elite_ratio,
            (SWARM_ELITE_RATIO_MIN + SWARM_ELITE_RATIO_MAX) * 0.5
        ));
        assert!(approx(
            mid.reinit_ratio,
            (SWARM_REINIT_RATIO_MIN + SWARM_REINIT_RATIO_MAX) * 0.5
        ));

        // 범위 밖 t 는 clamp된다.
        assert_eq!(
            SwarmDiversification::from_stagnation_intensity(-3.0),
            calm
        );
        assert_eq!(SwarmDiversification::from_stagnation_intensity(9.0), stuck);
    }

    #[test]
    fn classify_partitions_and_orders_tiers() {
        let d = SwarmDiversification::from_stagnation_intensity(1.0); // 10% / 30%
        let tiers = d.classify(20);
        assert_eq!(tiers.len(), 20);

        let n_elite = tiers.iter().filter(|t| **t == ParticleTier::Elite).count();
        let n_middle = tiers.iter().filter(|t| **t == ParticleTier::Middle).count();
        let n_reinit = tiers.iter().filter(|t| **t == ParticleTier::Reinit).count();
        assert_eq!(n_elite, 2); // round(20 × 0.10)
        assert_eq!(n_reinit, 6); // round(20 × 0.30)
        assert_eq!(n_middle, 12);
        assert_eq!(n_elite + n_middle + n_reinit, 20);

        // Elite → Middle → Reinit 순서(성능 내림차순 인덱스 가정)
        assert!(tiers[..n_elite].iter().all(|t| *t == ParticleTier::Elite));
        assert!(tiers[20 - n_reinit..]
            .iter()
            .all(|t| *t == ParticleTier::Reinit));
    }

    #[test]
    fn classify_edge_cases() {
        let d = SwarmDiversification::from_stagnation_intensity(1.0);
        assert!(d.classify(0).is_empty());

        // 후보가 1개면 Elite 최소 1개 보장 → 전부 Elite
        assert_eq!(d.classify(1), vec![ParticleTier::Elite]);

        // 후보가 2개면 Elite 1개가 우선, 나머지 1개만 Reinit이 가져갈 수 있다.
        let t2 = d.classify(2);
        assert_eq!(t2.len(), 2);
        assert_eq!(t2[0], ParticleTier::Elite);
        assert!(t2.iter().filter(|t| **t == ParticleTier::Elite).count() >= 1);

        // 어떤 n에서도 Elite ≥ 1 이고 총합이 보존된다.
        for n in 1..64usize {
            let tiers = d.classify(n);
            assert_eq!(tiers.len(), n);
            assert!(tiers.iter().filter(|t| **t == ParticleTier::Elite).count() >= 1);
        }
    }

    #[test]
    fn conflict_tiers_follow_20_50_30() {
        let tiers = SwarmDiversification::conflict_tiers(10);
        let n_best = tiers.iter().filter(|t| **t == ParticleTier::Elite).count();
        let n_middle = tiers.iter().filter(|t| **t == ParticleTier::Middle).count();
        let n_worst = tiers.iter().filter(|t| **t == ParticleTier::Reinit).count();
        assert_eq!((n_best, n_middle, n_worst), (2, 5, 3));

        assert!(SwarmDiversification::conflict_tiers(0).is_empty());
        assert_eq!(
            SwarmDiversification::conflict_tiers(1),
            vec![ParticleTier::Elite]
        );

        // 총합 보존 + 순서 유지
        for n in 1..64usize {
            let tiers = SwarmDiversification::conflict_tiers(n);
            assert_eq!(tiers.len(), n);
            let first_middle = tiers.iter().position(|t| *t == ParticleTier::Middle);
            let first_reinit = tiers.iter().position(|t| *t == ParticleTier::Reinit);
            if let (Some(m), Some(r)) = (first_middle, first_reinit) {
                assert!(m < r, "n = {n}: Middle이 Reinit보다 앞서야 한다");
            }
        }

        // 세 비율의 합은 1.00 이어야 한다(문서 명세).
        assert!(approx(
            PARTICLE_BEST_RATIO + PARTICLE_MIDDLE_RATIO + PARTICLE_WORST_RATIO,
            1.0
        ));
    }

    #[test]
    fn post_escape_ramps_return_to_normal() {
        let d = SwarmDiversification::default();

        // 탈출 직후 = 문서 배율
        assert!(approx(d.global_best_pull(0), POST_ESCAPE_GLOBAL_BEST_PULL));
        assert!(approx(d.exploration_boost(0), POST_ESCAPE_EXPLORATION_BOOST));

        // 창을 벗어나면 정상 파라미터(1.0)
        assert!(approx(d.global_best_pull(POST_ESCAPE_BOOST_ITERS), 1.0));
        assert!(approx(d.exploration_boost(POST_ESCAPE_BOOST_ITERS), 1.0));
        assert!(approx(d.global_best_pull(POST_ESCAPE_BOOST_ITERS * 5), 1.0));
        assert!(approx(d.exploration_boost(POST_ESCAPE_BOOST_ITERS * 5), 1.0));

        // 중간 지점 = 두 끝의 평균
        let half = POST_ESCAPE_BOOST_ITERS / 2;
        assert!(approx(
            d.global_best_pull(half),
            (POST_ESCAPE_GLOBAL_BEST_PULL + 1.0) * 0.5
        ));
        assert!(approx(
            d.exploration_boost(half),
            (POST_ESCAPE_EXPLORATION_BOOST + 1.0) * 0.5
        ));

        // 단조성: pull은 증가, boost는 감소
        for i in 1..POST_ESCAPE_BOOST_ITERS {
            assert!(d.global_best_pull(i) >= d.global_best_pull(i - 1));
            assert!(d.exploration_boost(i) <= d.exploration_boost(i - 1));
        }
    }
}
