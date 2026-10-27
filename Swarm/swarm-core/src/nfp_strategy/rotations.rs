//! ① 인스턴스 회전 — 회전 후보를 인스턴스 선언에서 그대로 뽑는다.
//!
//! # 왜 필요한가
//!
//! NFP-LBF 는 회전 후보를 `NfpLbfSeedConfig::allow_180` 플래그로 정한다. 즉 실제
//! 인스턴스가 무엇을 허용하든 `[0]` 아니면 `[0, 180]` 둘 중 하나다. 반면 기존 LBF
//! 생성기는 `item.allowed_rotation` 을 그대로 쓴다. 그래서 회전이 허용된 인스턴스에서
//! NFP-LBF 만 **구조적으로** 불리했다 — 알고리즘이 나빠서가 아니라 탐색 공간이 좁아서다.
//!
//! 실측(albano, `allowed_orientations = [0, 180]`): 이 전략을 끄면 LBF 대비 −1.51%p 로
//! **지고**, 켜면 +8.57%p 로 이긴다. 선언이 `[0]` 뿐인 인스턴스(여성WSjk)에서는 회전
//! 집합이 그대로라 영향이 없다.
//!
//! # 왜 `nfp_lbf::search::item_allowed_rotations` 를 쓰지 않았나
//!
//! 그쪽은 **설정으로 준 회전 집합과의 교집합**을 구한다 — 설정이 주인이고 인스턴스가
//! 종이다. 이 모듈은 반대다: **인스턴스가 주인**이고, 라이브러리 집합은 인스턴스에서
//! 유도한다. 규칙이 반대이므로 함수를 공유하지 않고 새로 썼다.
//!
//! ```text
//! instance_rotation_set(instance, step)   →  라이브러리 회전 집합 (모든 아이템의 합집합)
//!            │                                 이 집합으로 pair NFP 를 미리 굽는다
//!            v
//! item_rotation_table(instance, library)  →  아이템별 허용 회전 (선언 ∩ 라이브러리)
//! ```

use jagua_rs::geometry::geo_enums::RotationRange;
use jagua_rs::probs::spp::entities::SPInstance;

use super::config::DEFAULT_CONTINUOUS_ROTATION_STEP_DEG;

/// 라이브러리 회전 집합의 상한.
///
/// pair NFP 의 개수는 회전 집합 크기의 **제곱**에 비례한다(자세 a × 자세 b). 16이면
/// 최대 256 조합이고, 그 위로는 캐시 준비 시간이 배치 시간을 넘어선다. 상한을 넘는
/// 집합은 균등하게 솎아낸다 — 잘라내는 게 아니라 솎아내는 이유는, 뒤쪽을 통째로
/// 버리면 180도 이상이 전부 사라져 좌우 대칭 배치를 못 하기 때문이다.
pub const MAX_ROTATION_LIBRARY: usize = 16;

/// 인스턴스 전체가 필요로 하는 회전 집합(도, 오름차순 중복 제거).
///
/// 모든 아이템이 선언한 회전의 **합집합**이다. 이 집합으로 NFP 라이브러리를 굽는다.
///
/// * [`RotationRange::None`] → `[0]`
/// * [`RotationRange::Discrete`] → 라디안을 도로 반올림, `rem_euclid(360)`, 정렬·중복제거
/// * [`RotationRange::Continuous`] → `0, step, 2*step, ...` (`< 360`)
///
/// `continuous_step_deg` 가 `0` 이하이거나 `180` 을 넘으면
/// [`DEFAULT_CONTINUOUS_ROTATION_STEP_DEG`] 로 되돌린다 — 180을 넘기면 샘플이 사실상
/// `0` 하나라 회전을 켠 의미가 없다.
///
/// 집합이 [`MAX_ROTATION_LIBRARY`] 를 넘으면 균등 간격으로 솎아내고 경고를 남긴다.
/// 결과가 빌 일은 없다 — 최악이라도 `[0]` 을 돌려준다.
///
/// # 선언된 이산 회전은 솎아내지 않는다
///
/// `Discrete`/`None` 으로 **선언된** 각은 라이브러리에 그대로 남긴다. 상한에 걸려 선언된
/// 각이 빠지면 그 아이템은 [`item_rotation_table`] 에서 교집합이 비어 `[0]` 으로
/// 떨어지는데, `RotationRange` 가 `0` 을 허용하지 않는 아이템이면 검증이 그 시드를
/// **통째로 거부**한다. 그래서 솎아내기는 `Continuous` 샘플에만 건다 — 이산 각을 넣고
/// **남는 자리**(최소 1)에 연속 샘플을 균등하게 채운다. 연속 아이템은 어떤 각이든
/// 허용하므로 샘플이 줄어도 틀린 자세가 생기지 않는다.
///
/// 이산 각만으로 상한을 넘는 인스턴스는 그래도 솎아낸다(경고). 그 경우 어떤 아이템의
/// 선언이 빠질 수 있다 — 그런 인스턴스는 선언 자체를 줄여야 한다.
pub fn instance_rotation_set(instance: &SPInstance, continuous_step_deg: i64) -> Vec<i64> {
    let step = normalized_step(continuous_step_deg);

    let mut union: Vec<i64> = Vec::new();
    let mut continuous: Vec<i64> = Vec::new();
    for (item, _) in instance.items.iter() {
        match item.allowed_orientations().rotations() {
            RotationRange::Continuous => {
                if continuous.is_empty() {
                    continuous = declared_degrees(&item.allowed_orientations().rotations(), step);
                }
            }
            other => union.extend(declared_degrees(other, step)),
        }
    }
    union.sort_unstable();
    union.dedup();

    if !continuous.is_empty() {
        // 이산 각을 넣고 남는 자리에만 연속 샘플을 채운다. 자리가 없어도 최소 하나
        // (첫 샘플 = 0) 는 남긴다 — 연속 아이템이 라이브러리 전체를 받으므로 그걸로 충분하다.
        let room = MAX_ROTATION_LIBRARY.saturating_sub(union.len()).max(1);
        union.extend(thin_evenly(continuous, room));
        union.sort_unstable();
        union.dedup();
    }

    if union.is_empty() {
        // 아이템이 없거나 전부 유한하지 않은 각을 선언했다. 회전 없이 도는 편이
        // 배치 불가보다 낫다.
        return vec![0];
    }

    if union.len() > MAX_ROTATION_LIBRARY {
        let before = union.len();
        union = thin_evenly(union, MAX_ROTATION_LIBRARY);
        log::warn!(
            "[nfp-strategy] 회전 집합이 {before}개라 상한 {MAX_ROTATION_LIBRARY}개로 솎아냈다 \
             (pair NFP 는 집합 크기의 제곱으로 는다); 남은 집합 {union:?}"
        );
    }
    union
}

/// `item_id` 로 인덱싱한 아이템별 허용 회전(도).
///
/// 각 아이템의 **선언 ∩ 라이브러리** 다. 라이브러리에 없는 회전은 pair NFP 가 구워져
/// 있지 않아 쓸 수 없으므로 뺀다.
///
/// 교집합이 비면 `[0]` 으로 폴백한다. **빈 회전 목록은 "이 조각은 어떤 자세로도 놓을 수
/// 없다" 를 뜻하므로 절대 내보내면 안 된다** — 그 조각은 통째로 미배치가 된다.
///
/// [`RotationRange::Continuous`] 아이템은 어떤 각이든 허용하므로 라이브러리 전체를
/// 받는다(여기서 다시 이산화하지 않는다 — 라이브러리가 이미 이산 집합이다).
///
/// 반환 길이는 `max(item.id) + 1` 이다. 아이템이 없는 인덱스는 빈 벡터로 남는다 —
/// [`super::StrategyContext::rotations_for`] 와 `SequentialPlacer` 는 빈 항목을
/// "오라클 기본값을 쓰라" 로 읽는다.
pub fn item_rotation_table(instance: &SPInstance, library_rotations: &[i64]) -> Vec<Vec<i64>> {
    let library: Vec<i64> = if library_rotations.is_empty() {
        vec![0]
    } else {
        let mut l = library_rotations.to_vec();
        l.sort_unstable();
        l.dedup();
        l
    };

    let max_id = instance.items.iter().map(|(i, _)| i.idx()).max().unwrap_or(0);
    let mut table: Vec<Vec<i64>> = vec![Vec::new(); max_id.saturating_add(1)];

    for (item, _) in instance.items.iter() {
        // 연속 회전은 라이브러리 전체가 그대로 교집합이다. 여기서 step 으로 다시
        // 이산화하면 라이브러리에 없는 각이 섞여 들어온다.
        let declared: Vec<i64> = match item.allowed_orientations().rotations() {
            RotationRange::Continuous => library.clone(),
            other => declared_degrees(other, DEFAULT_CONTINUOUS_ROTATION_STEP_DEG),
        };

        let mut allowed: Vec<i64> = library
            .iter()
            .copied()
            .filter(|d| declared.contains(d))
            .collect();
        if allowed.is_empty() {
            allowed = vec![0];
        }

        match table.get_mut(item.idx()) {
            Some(slot) => *slot = allowed,
            // `SPInstance::new` 가 id 연속성을 보장하므로 도달하지 않는다.
            None => log::warn!("[nfp-strategy] item id {} 가 표 범위를 벗어났다", item.idx()),
        }
    }
    table
}

// ────────────────────────────────────────────────────────────────────────────
// 내부 헬퍼
// ────────────────────────────────────────────────────────────────────────────

/// 연속 회전 샘플 간격을 쓸 수 있는 값으로 정리한다.
fn normalized_step(step: i64) -> i64 {
    if step <= 0 || step > 180 {
        DEFAULT_CONTINUOUS_ROTATION_STEP_DEG
    } else {
        step
    }
}

/// 한 아이템이 선언한 회전을 도 단위 정규 집합으로 바꾼다(오름차순, 중복 제거).
///
/// 라디안 → 도 변환에서 반올림하는 이유는 캐시가 **정수 도**를 키로 쓰기 때문이다.
/// `rem_euclid(360)` 으로 음수각과 한 바퀴 넘는 각을 같은 자리로 접는다.
fn declared_degrees(range: &RotationRange, continuous_step_deg: i64) -> Vec<i64> {
    let mut out: Vec<i64> = match range {
        RotationRange::None => vec![0],
        RotationRange::Discrete(set) => set
            .iter()
            .filter(|r| r.is_finite())
            .map(|r| (r.to_degrees().round() as i64).rem_euclid(360))
            .collect(),
        RotationRange::Continuous => {
            let step = normalized_step(continuous_step_deg);
            let mut v = Vec::new();
            let mut d = 0i64;
            while d < 360 {
                v.push(d);
                d += step;
            }
            v
        }
    };
    out.sort_unstable();
    out.dedup();
    if out.is_empty() { vec![0] } else { out }
}

/// 정렬된 집합을 `cap` 개 이하로 **균등하게** 솎아낸다.
///
/// 인덱스를 `i * (len - 1) / (cap - 1)` 로 잡아 **양 끝을 포함해 정확히 `cap` 개**를
/// 집는다. 고정 `stride` 로 건너뛰면 상한을 한참 밑돌게 잘린다는 것이 이유다 —
/// `stride = ceil(len / cap)` 방식은 17개를 16개로 줄이라는 요구에 stride 2 를 골라
/// **9개**까지 떨어뜨린다. 회전 하나가 곧 배치 자유도라 그 손실은 그냥 손해다.
///
/// 첫 원소는 반드시 남는다. 입력이 오름차순이고 집합에 `0` 이 있으면 `0` 이 첫 원소이므로
/// **회전 없음은 절대 사라지지 않는다**. 이게 중요한 이유는, `0` 이 빠지면 회전을 선언하지
/// 않은 아이템이 [`item_rotation_table`] 에서 교집합이 비어 통째로 폴백을 타기 때문이다.
///
/// `cluster` 도 같은 규칙으로 회전을 솎아내야 해서 크레이트에 열어 둔다 — 앞에서부터
/// 자르면 촘촘한 라이브러리에서 `180` 근처가 통째로 사라진다.
pub(crate) fn thin_evenly(rots: Vec<i64>, cap: usize) -> Vec<i64> {
    if cap == 0 {
        return vec![0];
    }
    if rots.len() <= cap {
        return rots;
    }
    if cap == 1 {
        return match rots.first() {
            Some(&d) => vec![d],
            None => vec![0],
        };
    }

    // `len > cap >= 2` 이므로 `last >= 2`, `cap - 1 >= 1` 이다 — 0 나눗셈이 없다.
    let last = rots.len().saturating_sub(1);
    let mut kept: Vec<i64> = Vec::with_capacity(cap);
    for i in 0..cap {
        let idx = i.saturating_mul(last) / (cap - 1);
        if let Some(&d) = rots.get(idx) {
            kept.push(d);
        }
    }
    // idx 는 단조 증가라 중복이 나올 일이 없지만, 값이 같은 원소가 섞여 들어오면
    // 그대로 남는다. 오름차순이므로 인접 dedup 으로 충분하다.
    kept.dedup();
    if kept.is_empty() { vec![0] } else { kept }
}

#[cfg(test)]
mod tests {
    use super::*;
    use jagua_rs::collision_detection::CDEConfig;
    use jagua_rs::entities::Item;
    use jagua_rs::geometry::DTransformation;
    use jagua_rs::geometry::OriginalShape;
    use jagua_rs::geometry::fail_fast::SPSurrogateConfig;
    use jagua_rs::geometry::primitives::{Point, SPolygon};
    use jagua_rs::geometry::shape_modification::{ShapeModifyConfig, ShapeModifyMode};
    use jagua_rs::probs::spp::entities::Strip;
    use std::f32::consts::PI;

    const SURROGATE: SPSurrogateConfig = SPSurrogateConfig {
        n_pole_limits: [(64, 0.0), (16, 0.8), (8, 0.9)],
        ff_pole_area_ratio: 0.5,
        n_ff_piers: 0,
    };

    const CDE: CDEConfig = CDEConfig {
        quadtree_depth: 4,
        cd_threshold: 64,
        item_surrogate_config: SURROGATE,
    };

    /// 좌하단이 원점인 `w × h` 직사각형 아이템.
    fn rect_item(id: usize, w: f32, h: f32, allowed: RotationRange) -> std::sync::Arc<Item> {
        let shape = SPolygon::new(vec![
            Point(0.0, 0.0),
            Point(w, 0.0),
            Point(w, h),
            Point(0.0, h),
        ])
        .expect(crate::dmsg!("valid rectangle"));
        Item::new(id, (id) as u64, OriginalShape {
                shape,
                pre_transform: DTransformation::empty(),
                modify_mode: ShapeModifyMode::Inflate,
                modify_config: ShapeModifyConfig::default(),
            }, jagua_rs::geometry::AllowedOrientations::new(allowed, vec![]).unwrap(), None, SURROGATE).map(std::sync::Arc::new)
        .expect(crate::dmsg!("valid item"))
    }

    fn instance_of(items: Vec<(std::sync::Arc<Item>, usize)>) -> SPInstance {
        let strip = Strip::new(200.0, CDE, ShapeModifyConfig::default(), 2000.0).expect(crate::dmsg!("strip"));
        SPInstance::new(items, strip)
    }

    /// 도 단위를 라디안으로 — `Discrete` 는 라디안을 담는다.
    fn deg(d: f32) -> f32 {
        d * PI / 180.0
    }

    #[test]
    fn a_no_rotation_item_contributes_only_zero() {
        let inst = instance_of(vec![(rect_item(0, 10.0, 20.0, RotationRange::None), 3)]);
        assert_eq!(instance_rotation_set(&inst, 90), vec![0]);
    }

    #[test]
    fn a_discrete_declaration_is_rounded_to_degrees_and_deduplicated() {
        // 라디안 −90도와 270도는 같은 자세다. 중복 제거로 하나만 남아야 한다.
        let inst = instance_of(vec![(
            rect_item(
                0,
                10.0,
                20.0,
                RotationRange::Discrete(vec![deg(180.0), deg(-90.0), deg(270.0), 0.0]),
            ),
            1,
        )]);
        assert_eq!(instance_rotation_set(&inst, 90), vec![0, 180, 270]);
    }

    #[test]
    fn continuous_with_a_ninety_degree_step_yields_the_four_quadrants() {
        let inst = instance_of(vec![(
            rect_item(0, 10.0, 20.0, RotationRange::Continuous),
            1,
        )]);
        assert_eq!(instance_rotation_set(&inst, 90), vec![0, 90, 180, 270]);
    }

    #[test]
    fn an_unusable_continuous_step_falls_back_to_the_default() {
        let inst = instance_of(vec![(
            rect_item(0, 10.0, 20.0, RotationRange::Continuous),
            1,
        )]);
        // 0 이하와 180 초과는 둘 다 기본값(90도)으로 되돌아간다.
        assert_eq!(instance_rotation_set(&inst, 0), vec![0, 90, 180, 270]);
        assert_eq!(instance_rotation_set(&inst, 361), vec![0, 90, 180, 270]);
    }

    #[test]
    fn the_library_set_is_the_union_over_every_item() {
        let inst = instance_of(vec![
            (rect_item(0, 10.0, 20.0, RotationRange::None), 1),
            (
                rect_item(1, 30.0, 5.0, RotationRange::Discrete(vec![deg(90.0)])),
                1,
            ),
            (
                rect_item(2, 7.0, 7.0, RotationRange::Discrete(vec![deg(180.0)])),
                1,
            ),
        ]);
        assert_eq!(instance_rotation_set(&inst, 90), vec![0, 90, 180]);
    }

    #[test]
    fn a_set_above_the_cap_is_thinned_but_never_loses_zero() {
        // 1도 간격이면 360개다 — 상한 16개를 크게 넘는다.
        let inst = instance_of(vec![(
            rect_item(0, 10.0, 20.0, RotationRange::Continuous),
            1,
        )]);
        let set = instance_rotation_set(&inst, 1);
        assert!(
            set.len() <= MAX_ROTATION_LIBRARY,
            "상한을 넘으면 안 된다: {}",
            set.len()
        );
        assert_eq!(set.first().copied(), Some(0), "0 은 반드시 남아야 한다");
        assert!(set.contains(&0));
        // 균등 간격이므로 뒤쪽 큰 각도 살아남아야 한다 — 앞을 잘라 쓰면 안 된다.
        assert!(
            set.iter().any(|&d| d >= 180),
            "180도 이상이 통째로 사라지면 좌우 대칭 배치를 못 한다: {set:?}"
        );
    }

    /// 선언된 이산 각은 상한에 밀려도 **절대** 라이브러리에서 빠지면 안 된다.
    ///
    /// 빠지면 그 아이템은 `[0]` 으로 떨어지고, `0` 을 허용하지 않는 선언이면 검증이
    /// 시드를 통째로 거부한다. 솎아내기는 연속 샘플에만 걸어야 한다.
    #[test]
    fn declared_discrete_angles_survive_even_when_continuous_samples_overflow_the_cap() {
        // 이산 선언 12개(0 없음) + 연속 1도 샘플 360개 → 선언된 각이 16개를 훨씬 넘는다.
        let declared: Vec<i64> = (0..12).map(|i| 15 + i * 27).collect();
        let mut items: Vec<(std::sync::Arc<Item>, usize)> = declared
            .iter()
            .enumerate()
            .map(|(id, &d)| {
                (
                    rect_item(id, 10.0, 20.0, RotationRange::Discrete(vec![deg(d as f32)])),
                    1,
                )
            })
            .collect();
        items.push((rect_item(12, 5.0, 5.0, RotationRange::Continuous), 1));
        let inst = instance_of(items);

        let set = instance_rotation_set(&inst, 1);
        assert!(set.len() <= MAX_ROTATION_LIBRARY, "상한 초과: {}", set.len());
        for d in &declared {
            assert!(set.contains(d), "선언된 {d}도가 라이브러리에서 빠졌다: {set:?}");
        }
        // 남는 자리는 연속 샘플이 채운다 — 이산 각만으로 끝나면 안 된다.
        assert!(set.len() > declared.len(), "연속 샘플이 하나도 안 들어갔다: {set:?}");

        let table = item_rotation_table(&inst, &set);
        for (id, &d) in declared.iter().enumerate() {
            assert_eq!(table[id], vec![d], "아이템 {id} 의 선언이 바뀌었다");
        }
        assert_eq!(table[12], set, "연속 아이템은 라이브러리 전체를 받는다");
    }

    #[test]
    fn thinning_keeps_the_set_sorted_and_evenly_spread() {
        let dense: Vec<i64> = (0..64).collect();
        let thin = thin_evenly(dense, 8);
        assert!(thin.len() <= 8);
        assert_eq!(thin.first().copied(), Some(0));
        assert!(thin.windows(2).all(|w| w[0] < w[1]));
    }

    #[test]
    fn thinning_uses_the_whole_budget_instead_of_undershooting_it() {
        // 고정 stride 방식은 여기서 stride 2 를 골라 9개까지 떨어졌다. 회전 하나가
        // 곧 배치 자유도이므로 상한을 밑도는 만큼이 그대로 손해다.
        let seventeen: Vec<i64> = (0..17).collect();
        let thin = thin_evenly(seventeen, 16);
        assert_eq!(thin.len(), 16, "상한을 다 쓰지 않으면 회전을 공짜로 버리는 것이다");
        assert_eq!(thin.first().copied(), Some(0));
        assert_eq!(thin.last().copied(), Some(16), "양 끝을 모두 잡아야 균등하다");
    }

    #[test]
    fn thinning_to_a_single_slot_keeps_the_first_rotation() {
        assert_eq!(thin_evenly(vec![0, 90, 180, 270], 1), vec![0]);
        // 0 이 없는 집합이라도 인덱스 밖을 짚지 않는다.
        assert_eq!(thin_evenly(vec![90, 180, 270], 1), vec![90]);
    }

    #[test]
    fn the_item_table_drops_rotations_the_library_does_not_have() {
        let inst = instance_of(vec![(
            rect_item(
                0,
                10.0,
                20.0,
                RotationRange::Discrete(vec![0.0, deg(90.0), deg(180.0)]),
            ),
            1,
        )]);
        // 라이브러리에 90도가 없다 → 아이템도 90도를 못 쓴다.
        let table = item_rotation_table(&inst, &[0, 180]);
        assert_eq!(table.len(), 1);
        assert_eq!(table[0], vec![0, 180]);
    }

    #[test]
    fn an_item_with_no_overlap_falls_back_to_zero_instead_of_becoming_unplaceable() {
        let inst = instance_of(vec![(
            rect_item(0, 10.0, 20.0, RotationRange::Discrete(vec![deg(45.0)])),
            1,
        )]);
        // 라이브러리에 45도가 없다 → 교집합이 비지만 빈 목록을 내보내면 안 된다.
        let table = item_rotation_table(&inst, &[0, 180]);
        assert_eq!(table[0], vec![0]);
    }

    #[test]
    fn a_continuous_item_receives_the_whole_library() {
        let inst = instance_of(vec![(
            rect_item(0, 10.0, 20.0, RotationRange::Continuous),
            1,
        )]);
        let table = item_rotation_table(&inst, &[0, 37, 211]);
        assert_eq!(table[0], vec![0, 37, 211]);
    }

    #[test]
    fn the_table_is_indexed_by_item_id() {
        let inst = instance_of(vec![
            (rect_item(0, 10.0, 20.0, RotationRange::None), 1),
            (
                rect_item(1, 30.0, 5.0, RotationRange::Discrete(vec![0.0, deg(180.0)])),
                2,
            ),
        ]);
        let library = instance_rotation_set(&inst, 90);
        let table = item_rotation_table(&inst, &library);
        assert_eq!(table.len(), 2);
        assert_eq!(table[0], vec![0]);
        assert_eq!(table[1], vec![0, 180]);
    }

    #[test]
    fn an_empty_library_is_treated_as_zero_only() {
        let inst = instance_of(vec![(
            rect_item(0, 10.0, 20.0, RotationRange::Discrete(vec![0.0, deg(180.0)])),
            1,
        )]);
        assert_eq!(item_rotation_table(&inst, &[])[0], vec![0]);
    }
}
