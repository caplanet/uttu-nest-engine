//! §4. Guided Local Search 방식의 동적 가중치
//!
//! 같은 패턴 쌍이 계속 서로 가로막는다면 해당 관계에 가중치를 부여한다.
//!
//! ```text
//! 기본 평가값
//! CollisionCost(a, b)
//!
//! 동적 평가값
//! Weight(a, b) × CollisionCost(a, b)
//! ```
//!
//! 동일한 충돌이나 방해 관계가 반복되면:
//!
//! ```text
//! Weight(a, b) 증가
//! ```
//!
//! 문제가 해결되고 일정 시간이 지나면:
//!
//! ```text
//! Weight(a, b) → 1로 점진적 감소
//! ```
//!
//! 그러면 탐색 알고리즘은 같은 배치로 다시 돌아가는 것을 피하고, 다른 위치 관계를
//! 찾게 된다.
//!
//! # 키를 `item_id`로 잡는 이유
//!
//! [`PItemKey`]는 아이템이 제거·재삽입되면 새 키로 바뀐다. 탈출 연산(Destroy &
//! Repair, Collision Push)은 아이템을 빈번하게 뽑았다 다시 넣으므로 `PItemKey`를
//! 키로 쓰면 학습이 매번 끊긴다. 반면 `item_id`는 인스턴스 수명 내내 불변이라
//! "같은 패턴 쌍이 계속 서로 가로막는다"는 관계를 누적할 수 있다.

use crate::consts::GLS_WEIGHT_MAX;
use crate::escape::{
    GUIDED_WEIGHT_DECAY, GUIDED_WEIGHT_INCREASE, GUIDED_WEIGHT_MAX, GUIDED_WEIGHT_NEUTRAL,
};
use crate::optimizer::separator::Separator;
use jagua_rs::entities::PItemKey;
use std::collections::{HashMap, HashSet};

/// 가중치를 학습할 패턴 쌍의 키. 항상 `(min, max)`로 정규화되어 `(a, b)`와 `(b, a)`가
/// 같은 항목을 가리킨다. 원소는 **item_id**다.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct GuidedPairKey {
    pub a: usize,
    pub b: usize,
}

impl GuidedPairKey {
    /// `(min(a, b), max(a, b))`로 정규화해 생성한다.
    #[inline]
    pub fn new(a: usize, b: usize) -> Self {
        if a <= b {
            Self { a, b }
        } else {
            Self { a: b, b: a }
        }
    }
}

/// 패턴 쌍별 동적 가중치 테이블.
///
/// 중립값(1.0)인 쌍은 저장하지 않는다. 즉 맵에 들어 있는 항목은 항상
/// `> GUIDED_WEIGHT_NEUTRAL`이며, 감쇠 결과가 1.0 이하로 내려가는 순간 제거되어
/// 문서가 말하는 `Weight(a, b) → 1`이 정확히 1에서 멈춘다.
#[derive(Default, Debug, Clone)]
pub struct GuidedWeights {
    weights: HashMap<GuidedPairKey, f32>,
}

impl GuidedWeights {
    pub fn new() -> Self {
        Self {
            weights: HashMap::new(),
        }
    }

    /// `Weight(a, b)`. 학습된 기록이 없는 쌍은 중립값 [`GUIDED_WEIGHT_NEUTRAL`]이다.
    #[inline]
    pub fn weight(&self, a: usize, b: usize) -> f32 {
        match self.weights.get(&GuidedPairKey::new(a, b)) {
            Some(&w) if w.is_finite() && w > GUIDED_WEIGHT_NEUTRAL => w,
            _ => GUIDED_WEIGHT_NEUTRAL,
        }
    }

    /// ```text
    /// 동적 평가값
    /// Weight(a, b) × CollisionCost(a, b)
    /// ```
    #[inline]
    pub fn guided_cost(&self, a: usize, b: usize, collision_cost: f32) -> f32 {
        self.weight(a, b) * collision_cost
    }

    /// ```text
    /// 동일한 충돌이나 방해 관계가 반복되면
    /// Weight(a, b) 증가
    /// ```
    ///
    /// 각 쌍의 가중치에 [`GUIDED_WEIGHT_INCREASE`]를 곱하고 [`GUIDED_WEIGHT_MAX`]로
    /// 상한을 건다.
    pub fn increase_guided_weights(&mut self, pairs: impl IntoIterator<Item = GuidedPairKey>) {
        for key in pairs {
            let entry = self.weights.entry(key).or_insert(GUIDED_WEIGHT_NEUTRAL);
            // 과거에 비정상 값이 들어갔더라도 학습을 중립값에서 다시 시작한다.
            let base = if entry.is_finite() && *entry > GUIDED_WEIGHT_NEUTRAL {
                *entry
            } else {
                GUIDED_WEIGHT_NEUTRAL
            };
            let updated = base * GUIDED_WEIGHT_INCREASE;
            *entry = if updated.is_finite() {
                updated.clamp(GUIDED_WEIGHT_NEUTRAL, GUIDED_WEIGHT_MAX)
            } else {
                GUIDED_WEIGHT_MAX
            };
        }
    }

    /// ```text
    /// 문제가 해결되고 일정 시간이 지나면
    /// Weight(a, b) → 1로 점진적 감소
    /// ```
    ///
    /// 등록된 **모든** 쌍에 [`GUIDED_WEIGHT_DECAY`]를 곱한다. 결과가 중립값 이하로
    /// 내려가면 맵에서 제거해 정확히 1.0에 고정한다.
    pub fn decay_guided_weights(&mut self) {
        self.weights.retain(|_, w| {
            let decayed = *w * GUIDED_WEIGHT_DECAY;
            if !decayed.is_finite() || decayed <= GUIDED_WEIGHT_NEUTRAL {
                // 1.0으로 고정 = 맵에서 제거(미등록 쌍은 weight()가 1.0을 돌려준다)
                false
            } else {
                *w = decayed.min(GUIDED_WEIGHT_MAX);
                true
            }
        });
    }

    /// 현재 레이아웃에서 실제로 충돌 중인(= `pair loss > 0`) 아이템 쌍을 모두 찾아
    /// 그 쌍들의 가중치를 증가시킨다.
    ///
    /// 반환값은 **증가시킨 서로 다른 `item_id` 쌍의 수**다. 같은 `item_id` 쌍이
    /// 여러 배치 인스턴스에서 동시에 충돌하더라도 한 번의 관측에서는 한 번만
    /// 증가시켜, 수량이 많은 패턴의 가중치가 폭주하는 것을 막는다.
    pub fn observe_collisions(&mut self, sep: &Separator) -> usize {
        // 트래커에 등록되어 있는 배치만 대상으로 한다(get_pair_loss는 미등록 키에서
        // 패닉한다).
        let placed: Vec<(PItemKey, usize)> = sep
            .prob
            .layout()
            .placed_items()
            .iter()
            .filter(|(pk, _)| sep.ct.pk_idx_map.contains_key(*pk))
            .map(|(pk, pi)| (pk, pi.item().idx()))
            .collect();

        let mut colliding: HashSet<GuidedPairKey> = HashSet::new();
        for (i, &(pk1, id1)) in placed.iter().enumerate() {
            for &(pk2, id2) in placed[i + 1..].iter() {
                let loss = sep.ct.get_pair_loss(pk1, pk2);
                if loss.is_finite() && loss > 0.0 {
                    colliding.insert(GuidedPairKey::new(id1, id2));
                }
            }
        }

        let n = colliding.len();
        self.increase_guided_weights(colliding);
        n
    }

    /// 학습된 쌍 가중치를 [`CollisionTracker`](crate::quantify::tracker::CollisionTracker)의
    /// 쌍 가중치에 반영한다.
    ///
    /// 기존 GLS 가중치를 덮어쓰지 않고 **곱한** 뒤 [`GLS_WEIGHT_MAX`]로 clamp한다.
    /// 즉 트래커가 자체적으로 학습한 `weight`와 이 모듈이 학습한 `Weight(a, b)`가
    /// 함께 작용한다.
    pub fn apply_to_tracker(&self, sep: &mut Separator) {
        if self.weights.is_empty() {
            return;
        }

        // 트래커 인덱스 → item_id 대응표를 먼저 만든다(이후 ct를 가변 대여해야 하므로).
        let size = sep.ct.size;
        let mut idx_to_item: Vec<Option<usize>> = vec![None; size];
        for (pk, &idx) in sep.ct.pk_idx_map.iter() {
            if idx >= size {
                continue;
            }
            if let Some(pi) = sep.prob.layout().placed_items().get(pk) {
                idx_to_item[idx] = Some(pi.item().idx());
            }
        }

        // Invalidate cached sums once for the whole weight update.
        let pair_collisions = sep.ct.pair_collisions_mut();
        for i in 0..size {
            let Some(id_i) = idx_to_item[i] else { continue };
            for j in (i + 1)..size {
                let Some(id_j) = idx_to_item[j] else { continue };
                let gw = self.weight(id_i, id_j);
                if gw == GUIDED_WEIGHT_NEUTRAL {
                    continue;
                }
                let entry = &mut pair_collisions[(i, j)];
                let base = if entry.weight.is_finite() && entry.weight > 0.0 {
                    entry.weight
                } else {
                    1.0
                };
                let scaled = base * gw;
                entry.weight = if scaled.is_finite() {
                    scaled.clamp(1.0, GLS_WEIGHT_MAX)
                } else {
                    1.0
                };
            }
        }
    }

    /// 중립값이 아닌(= 학습된) 쌍의 수.
    #[inline]
    pub fn len(&self) -> usize {
        self.weights.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.weights.is_empty()
    }

    /// 학습 기록을 모두 버린다(모든 쌍이 다시 중립값 1.0).
    #[inline]
    pub fn clear(&mut self) {
        self.weights.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pair_key_is_normalised() {
        assert_eq!(GuidedPairKey::new(7, 3), GuidedPairKey::new(3, 7));
        let k = GuidedPairKey::new(9, 2);
        assert_eq!((k.a, k.b), (2, 9));
        let same = GuidedPairKey::new(4, 4);
        assert_eq!((same.a, same.b), (4, 4));
    }

    #[test]
    fn unregistered_pair_is_neutral() {
        let gw = GuidedWeights::new();
        assert_eq!(gw.weight(0, 1), GUIDED_WEIGHT_NEUTRAL);
        assert!(gw.is_empty());
        // 동적 평가값 = 1.0 × CollisionCost
        assert_eq!(gw.guided_cost(0, 1, 12.5), 12.5);
    }

    #[test]
    fn increase_multiplies_by_the_documented_ratio() {
        let mut gw = GuidedWeights::new();
        gw.increase_guided_weights([GuidedPairKey::new(1, 0)]);
        assert!((gw.weight(0, 1) - GUIDED_WEIGHT_INCREASE).abs() < 1e-6);
        assert_eq!(gw.len(), 1);

        gw.increase_guided_weights([GuidedPairKey::new(0, 1)]);
        let expected = GUIDED_WEIGHT_INCREASE * GUIDED_WEIGHT_INCREASE;
        assert!((gw.weight(1, 0) - expected).abs() < 1e-5);
    }

    #[test]
    fn guided_cost_is_weight_times_collision_cost() {
        let mut gw = GuidedWeights::new();
        gw.increase_guided_weights([GuidedPairKey::new(2, 5)]);
        let w = gw.weight(2, 5);
        assert!((gw.guided_cost(5, 2, 4.0) - w * 4.0).abs() < 1e-6);
    }

    #[test]
    fn increase_is_clamped_at_the_upper_bound() {
        let mut gw = GuidedWeights::new();
        for _ in 0..1_000 {
            gw.increase_guided_weights([GuidedPairKey::new(3, 8)]);
        }
        assert_eq!(gw.weight(3, 8), GUIDED_WEIGHT_MAX);
        assert!(gw.weight(3, 8).is_finite());
    }

    #[test]
    fn decay_converges_exactly_to_one() {
        let mut gw = GuidedWeights::new();
        gw.increase_guided_weights([GuidedPairKey::new(1, 2), GuidedPairKey::new(3, 4)]);
        assert_eq!(gw.len(), 2);

        // 1.30 → 1.17 → 1.053 → 0.9477(≤ 1.0) → 제거
        gw.decay_guided_weights();
        assert!((gw.weight(1, 2) - GUIDED_WEIGHT_INCREASE * GUIDED_WEIGHT_DECAY).abs() < 1e-6);
        assert_eq!(gw.len(), 2);

        for _ in 0..64 {
            gw.decay_guided_weights();
        }
        // 정확히 1.0에서 멈추고, 맵에서도 사라진다
        assert_eq!(gw.weight(1, 2), GUIDED_WEIGHT_NEUTRAL);
        assert_eq!(gw.weight(3, 4), GUIDED_WEIGHT_NEUTRAL);
        assert!(gw.is_empty());
    }

    #[test]
    fn decay_never_undershoots_below_neutral() {
        let mut gw = GuidedWeights::new();
        gw.increase_guided_weights([GuidedPairKey::new(0, 0)]);
        for _ in 0..200 {
            gw.decay_guided_weights();
            assert!(
                gw.weight(0, 0) >= GUIDED_WEIGHT_NEUTRAL,
                "가중치는 1.0 미만으로 내려가면 안 된다"
            );
        }
    }

    #[test]
    fn clear_resets_every_pair() {
        let mut gw = GuidedWeights::new();
        gw.increase_guided_weights([GuidedPairKey::new(1, 2), GuidedPairKey::new(2, 3)]);
        assert_eq!(gw.len(), 2);
        gw.clear();
        assert!(gw.is_empty());
        assert_eq!(gw.weight(1, 2), GUIDED_WEIGHT_NEUTRAL);
    }

    #[test]
    fn guided_weight_constants_are_consistent() {
        assert!(GUIDED_WEIGHT_INCREASE > 1.0);
        assert!(GUIDED_WEIGHT_DECAY < 1.0 && GUIDED_WEIGHT_DECAY > 0.0);
        assert!(GUIDED_WEIGHT_MAX > GUIDED_WEIGHT_NEUTRAL);
        // 트래커에 곱해도 GLS 상한 안에서 유효한 범위여야 한다
        assert!(GUIDED_WEIGHT_MAX <= GLS_WEIGHT_MAX);
    }
}
