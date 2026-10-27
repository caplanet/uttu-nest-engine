//! §4 점수 산식 — **설계 문서의 수식을 그대로** 옮긴다.
//!
//! ```text
//! 4-1  표면 점수 = 효율 = 배치 면적 / 기준 면적
//!      기준 면적 = 현재 길이(폭) x 높이
//!
//! 4-2  v_max = 최고 효율 (효율 내림차순 정렬 후 첫 후보)
//!      부족률 d = (1 - v / v_max) x 100                     (%)
//!
//!      w(d):  d <= 0.2        -> 0
//!             0.2 < d <= 0.5  -> 1 - 0.5*d
//!             0.5 < d <= 1.0  -> 1.0
//!             d > 1.0         -> 0.5
//!
//!      표면 s별 (최대 9개 표면, 표면당 기여 최대 5개):
//!          avg_s   = Σ w*(효율 - 기준값) / Σ w
//!          (Σ w <= 0.001 이면 그 표면 = 센티널 -1.0, 제외)
//!          score_s = v_max - 2.0*avg_s
//!
//!      후보 최종 점수 = 4.0*min_s(score_s) - 3.0*v_max
//!                     = v_max - 8.0*max_s(avg_s)
//! ```
//!
//! # 두 표현이 같은지 확인했다
//!
//! 문서가 최종 점수를 두 형태로 적는다. 대수적으로 같다:
//!
//! ```text
//! 4*min_s(v_max - 2*avg_s) - 3*v_max
//!   = 4*v_max - 8*max_s(avg_s) - 3*v_max      (min(v - 2a) = v - 2*max(a))
//!   = v_max - 8*max_s(avg_s)
//! ```
//!
//! [`candidate_score`] 는 **첫 번째 형태**로 계산하고, 테스트가 두 형태의 일치를
//! 고정한다. 첫 형태를 쓰는 이유는 `score_s` 자체가 진단에 쓸모 있기 때문이다.

use crate::refplace::params::{
    FINAL_MIN_COEFF, FINAL_VMAX_COEFF, MAX_CONTRIBUTIONS_PER_SURFACE, MAX_SCORED_SURFACES,
    SCORE_S_COEFF, SHORTFALL_SCALE, SURFACE_SENTINEL, WEIGHT_BREAK_HIGH, WEIGHT_BREAK_LOW,
    WEIGHT_SUM_FLOOR,
};

// ────────────────────────────────────────────────────────────────────────────
// 4-1 표면 점수 = 효율
// ────────────────────────────────────────────────────────────────────────────

/// 스트립 경계를 넘어간 부분을 **잘라서** 포함한 배치 면적(mm²).
///
/// 설계 §4-1: "현재 스트립 경계(폭)를 넘어간 피스는 **넘은 만큼을 잘라서 포함**한다".
///
/// 여기서는 bbox 를 기준으로 잘라 낸다 — 정확한 폴리곤 클리핑은 후보마다 하기엔 비싸고,
/// 이 값의 쓰임(효율의 분자)은 후보 사이의 **순서**를 정하는 것이라 근사로 충분하다.
/// 다만 **분모(기준 면적)도 같은 근사 수준**이어야 한다는 점에 주의할 것 — 분자만
/// 정확하게 만들면 비율이 어긋난다(이 저장소가 실측으로 확인한 함정이다).
pub fn clipped_area(bbox: (f64, f64, f64, f64), strip_width: f64, strip_height: f64) -> f64 {
    let (x_min, y_min, x_max, y_max) = bbox;
    let w = (x_max.min(strip_width) - x_min.max(0.0)).max(0.0);
    let h = (y_max.min(strip_height) - y_min.max(0.0)).max(0.0);
    w * h
}

/// 표면 점수 = 효율 = 배치 면적 / 기준 면적 (§4-1).
///
/// 기준 면적이 0 이하이면 `0.0` 이다 — 폭이 아직 안 정해진 표면은 효율을 말할 수 없다.
pub fn efficiency(placed_area: f64, reference_area: f64) -> f64 {
    if !(reference_area > 0.0) || !placed_area.is_finite() {
        return 0.0;
    }
    placed_area / reference_area
}

// ────────────────────────────────────────────────────────────────────────────
// 4-2 후보 점수 = 균형 가중식
// ────────────────────────────────────────────────────────────────────────────

/// 부족률 `d` = `(1 - v / v_max) x 100` (%).
///
/// `v_max` 가 0 이하이면 비교 기준이 없으므로 `0.0`(= 최고와 동등)으로 본다.
pub fn shortfall(v: f64, v_max: f64) -> f64 {
    if !(v_max > 0.0) {
        return 0.0;
    }
    (1.0 - v / v_max) * SHORTFALL_SCALE
}

/// 가중치 `w(d)` (§4-2).
///
/// # 구간이 단조가 아니다
///
/// `d > 1.0` 에서 `0.5` 로 **내려간다**. 즉 "최고와 아주 많이 차이 나는 후보" 는 벌점
/// 계산에서 오히려 덜 세게 반영된다. 직관에는 어긋나지만 문서가 정한 값이고, 뜻은
/// 이렇게 읽힌다 — 크게 뒤진 후보는 이미 탈락이라 형평 판단에 끌어들일 필요가 없다.
/// **바꾸지 말 것.** 바꾸려면 문서를 먼저 고친다.
pub fn weight(d: f64) -> f64 {
    if !d.is_finite() {
        return 0.0;
    }
    if d <= WEIGHT_BREAK_LOW {
        0.0
    } else if d <= WEIGHT_BREAK_HIGH {
        1.0 - 0.5 * d
    } else if d <= 1.0 {
        1.0
    } else {
        0.5
    }
}

/// 표면 하나가 낸 후보 기여 하나.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Contribution {
    /// 이 기여의 효율 `v`.
    pub efficiency: f64,
    /// §4-2 의 **기준값** — 후보 레코드가 배치 후 컨테이너 기준으로 기록해 둔 값.
    pub baseline: f64,
}

/// 표면 하나의 `avg_s`.
///
/// `Σ w <= WEIGHT_SUM_FLOOR` 이면 [`SURFACE_SENTINEL`] 을 돌려준다 — 그 표면은
/// `min_s` 에서 제외된다.
///
/// 기여는 앞에서부터 [`MAX_CONTRIBUTIONS_PER_SURFACE`] 개만 본다(§4-2).
pub fn surface_avg(contributions: &[Contribution], v_max: f64) -> f64 {
    let mut num = 0.0;
    let mut den = 0.0;
    for c in contributions.iter().take(MAX_CONTRIBUTIONS_PER_SURFACE) {
        if !c.efficiency.is_finite() || !c.baseline.is_finite() {
            continue;
        }
        let w = weight(shortfall(c.efficiency, v_max));
        num += w * (c.efficiency - c.baseline);
        den += w;
    }
    if den <= WEIGHT_SUM_FLOOR {
        return SURFACE_SENTINEL;
    }
    num / den
}

/// 한 표면의 `score_s = v_max - 2.0 * avg_s`.
///
/// `avg_s` 가 센티널이면 이 표면도 센티널이다 — 그래야 `min_s` 에서 제외된다.
pub fn surface_score(avg_s: f64, v_max: f64) -> f64 {
    if avg_s == SURFACE_SENTINEL {
        return SURFACE_SENTINEL;
    }
    v_max - SCORE_S_COEFF * avg_s
}

/// 후보 최종 점수 (§4-2).
///
/// `surfaces` 는 표면별 기여 목록이다. 앞에서부터 [`MAX_SCORED_SURFACES`] 개만 본다.
///
/// 살아남은 표면이 하나도 없으면(전부 센티널) 균형 항이 없으므로 `v_max` 를 그대로
/// 돌려준다 — 표면이 하나뿐인 단일 strip 에서 자연히 그렇게 된다.
pub fn candidate_score(surfaces: &[Vec<Contribution>], v_max: f64) -> f64 {
    let mut min_score: Option<f64> = None;
    for contributions in surfaces.iter().take(MAX_SCORED_SURFACES) {
        let avg = surface_avg(contributions, v_max);
        let s = surface_score(avg, v_max);
        if s == SURFACE_SENTINEL {
            continue;
        }
        min_score = Some(match min_score {
            None => s,
            Some(m) => m.min(s),
        });
    }
    match min_score {
        None => v_max,
        Some(m) => FINAL_MIN_COEFF * m - FINAL_VMAX_COEFF * v_max,
    }
}

/// 문서가 적은 **두 번째 형태** — `v_max - 8.0 * max_s(avg_s)`.
///
/// 진단·검증용이다. [`candidate_score`] 와 항상 같아야 하고, 테스트가 그것을 고정한다.
pub fn candidate_score_alt(surfaces: &[Vec<Contribution>], v_max: f64) -> f64 {
    let mut max_avg: Option<f64> = None;
    for contributions in surfaces.iter().take(MAX_SCORED_SURFACES) {
        let avg = surface_avg(contributions, v_max);
        if avg == SURFACE_SENTINEL {
            continue;
        }
        max_avg = Some(match max_avg {
            None => avg,
            Some(m) => m.max(avg),
        });
    }
    match max_avg {
        None => v_max,
        Some(a) => v_max - FINAL_MIN_COEFF * SCORE_S_COEFF * a,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn c(efficiency: f64, baseline: f64) -> Contribution {
        Contribution {
            efficiency,
            baseline,
        }
    }

    /// `w(d)` 의 네 구간이 문서와 정확히 같은지. 경계값 포함.
    #[test]
    fn the_weight_curve_matches_the_document_at_every_breakpoint() {
        // d <= 0.2 -> 0 (최고와 동등, 벌점 없음)
        assert_eq!(weight(0.0), 0.0);
        assert_eq!(weight(0.2), 0.0);
        // 0.2 < d <= 0.5 -> 1 - 0.5d
        assert!((weight(0.3) - 0.85).abs() < 1.0e-12);
        assert!((weight(0.5) - 0.75).abs() < 1.0e-12);
        // 0.5 < d <= 1.0 -> 1.0
        assert_eq!(weight(0.51), 1.0);
        assert_eq!(weight(1.0), 1.0);
        // d > 1.0 -> 0.5
        assert_eq!(weight(1.01), 0.5);
        assert_eq!(weight(100.0), 0.5);
    }

    /// `w(d)` 는 `d > 1` 에서 **내려간다**. 단조라고 착각하면 안 된다.
    #[test]
    fn the_weight_curve_is_not_monotone_and_that_is_intentional() {
        assert!(weight(1.0) > weight(1.5), "d > 1 에서 0.5 로 내려가는 것이 설계다");
    }

    /// 부족률이 문서 식 그대로인지.
    #[test]
    fn shortfall_is_a_percentage_of_the_best() {
        // v 가 v_max 의 99% 면 d = 1(%)
        assert!((shortfall(0.99, 1.0) - 1.0).abs() < 1.0e-12);
        assert_eq!(shortfall(1.0, 1.0), 0.0);
        // 기준이 없으면 동등으로 본다.
        assert_eq!(shortfall(0.5, 0.0), 0.0);
    }

    /// 가중합이 하한 이하이면 그 표면은 **센티널**이다.
    ///
    /// 센티널이 `-1.0`(유한한 음수)인 것이 중요하다. `NaN` 이면 `min` 비교가 훑는
    /// 순서에 따라 달라져 결정론이 깨진다.
    #[test]
    fn a_surface_with_no_weight_becomes_the_sentinel() {
        // 전부 최고와 동등(d <= 0.2)이면 w = 0 이라 Σw = 0.
        let contributions = vec![c(1.0, 0.5), c(0.999, 0.5)];
        assert_eq!(surface_avg(&contributions, 1.0), SURFACE_SENTINEL);
        assert_eq!(
            surface_score(SURFACE_SENTINEL, 1.0),
            SURFACE_SENTINEL,
            "센티널 표면의 score_s 도 센티널이라야 min 에서 빠진다"
        );
        assert!(SURFACE_SENTINEL.is_finite(), "센티널은 NaN 이면 안 된다");
    }

    /// 표면당 기여는 **5개까지만** 본다.
    #[test]
    fn only_the_first_five_contributions_of_a_surface_count() {
        let mut many = vec![c(0.99, 0.0); MAX_CONTRIBUTIONS_PER_SURFACE];
        let five = surface_avg(&many, 1.0);
        // 6번째부터는 무시돼야 하므로, 아주 다른 값을 붙여도 결과가 같다.
        many.push(c(0.10, 0.0));
        many.push(c(0.10, 0.0));
        assert!((surface_avg(&many, 1.0) - five).abs() < 1.0e-12);
    }

    /// 표면은 **9개까지만** 본다.
    #[test]
    fn only_the_first_nine_surfaces_count() {
        let surface = vec![c(0.99, 0.0)];
        let nine: Vec<Vec<Contribution>> = vec![surface.clone(); MAX_SCORED_SURFACES];
        let s9 = candidate_score(&nine, 1.0);
        let mut ten = nine.clone();
        // 10번째에 아주 나쁜 표면을 붙여도 결과가 같아야 한다.
        ten.push(vec![c(0.01, 0.9)]);
        assert!((candidate_score(&ten, 1.0) - s9).abs() < 1.0e-12);
    }

    /// 문서가 적은 두 형태가 대수적으로 같은지 — 무작위 아닌 여러 형태로 확인.
    #[test]
    fn the_two_forms_of_the_final_score_agree() {
        let cases: Vec<(Vec<Vec<Contribution>>, f64)> = vec![
            (vec![vec![c(0.9, 0.2), c(0.8, 0.3)]], 1.0),
            (
                vec![vec![c(0.9, 0.2)], vec![c(0.7, 0.1), c(0.6, 0.4)]],
                1.0,
            ),
            (
                vec![
                    vec![c(0.95, 0.5)],
                    vec![c(0.5, 0.5)],
                    vec![c(0.99, 0.99)],
                ],
                1.0,
            ),
            // 전부 센티널인 경우 — 둘 다 v_max 를 돌려준다.
            (vec![vec![c(1.0, 0.0)]], 1.0),
        ];
        for (surfaces, v_max) in cases {
            let a = candidate_score(&surfaces, v_max);
            let b = candidate_score_alt(&surfaces, v_max);
            assert!(
                (a - b).abs() < 1.0e-9,
                "두 형태가 갈렸다: {a} vs {b} (v_max={v_max})"
            );
        }
    }

    /// 표면이 하나뿐이면 균형 항이 무의미해도 점수가 정의돼야 한다.
    #[test]
    fn a_single_surface_still_produces_a_score() {
        let s = candidate_score(&[vec![c(0.9, 0.2)]], 1.0);
        assert!(s.is_finite());
    }

    /// **한 표면만 크게 뒤처지면 8배로 깎인다** — 이 식의 존재 이유다.
    #[test]
    fn one_lagging_surface_is_penalised_eightfold() {
        // 두 표면이 고르게 좋은 경우.
        let even = vec![vec![c(0.9, 0.85)], vec![c(0.9, 0.85)]];
        // 한 표면만 기준값 대비 크게 앞서 avg_s 가 큰 경우 = 형평이 깨진 배치.
        let uneven = vec![vec![c(0.9, 0.85)], vec![c(0.9, 0.10)]];
        let s_even = candidate_score(&even, 1.0);
        let s_uneven = candidate_score(&uneven, 1.0);
        assert!(
            s_uneven < s_even,
            "형평이 깨진 배치가 더 낮은 점수를 받아야 한다: {s_uneven} vs {s_even}"
        );
        // 차이가 avg_s 차이의 8배여야 한다.
        let d_avg = surface_avg(&uneven[1], 1.0) - surface_avg(&even[1], 1.0);
        assert!(
            ((s_even - s_uneven) - 8.0 * d_avg).abs() < 1.0e-9,
            "감점 배율이 8이 아니다"
        );
    }

    /// 4-1: 스트립을 넘어간 부분은 **잘라서** 포함한다.
    #[test]
    fn area_outside_the_strip_is_clipped_not_dropped() {
        // 폭 100 스트립에 x 90..120 인 조각 — 90..100 만 센다.
        let a = clipped_area((90.0, 0.0, 120.0, 10.0), 100.0, 50.0);
        assert!((a - 100.0).abs() < 1.0e-12, "잘린 만큼만 포함해야 한다");
        // 통째로 밖이면 0.
        assert_eq!(clipped_area((200.0, 0.0, 210.0, 10.0), 100.0, 50.0), 0.0);
        // 통째로 안이면 전부.
        assert!((clipped_area((0.0, 0.0, 10.0, 10.0), 100.0, 50.0) - 100.0).abs() < 1.0e-12);
    }

    /// 기준 면적이 없으면 효율은 0 이다 — 0 나눗셈으로 `inf` 를 만들면 정렬이 무너진다.
    #[test]
    fn efficiency_is_zero_when_there_is_no_reference_area() {
        assert_eq!(efficiency(100.0, 0.0), 0.0);
        assert_eq!(efficiency(100.0, -1.0), 0.0);
        assert_eq!(efficiency(f64::NAN, 10.0), 0.0);
    }
}
