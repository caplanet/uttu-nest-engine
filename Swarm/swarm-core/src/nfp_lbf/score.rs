//! LB + Contact + Void 평가.
//!
//! LBF라면 평가를 대략 `min(x, y)` 또는 strip이 x 방향으로 늘어난다면
//!
//! ```text
//! Score = α · NewStripLength + β · x + γ · y
//! ```
//!
//! 처럼 둘 수 있다. 하지만 추천하는 건 **순수 LB보다 `Strip Length 우선 +
//! Bottom/Left 보조`**다.
//!
//! ```text
//! 1순위 : 배치 후 Strip Length 최소
//! 2순위 : x 최소
//! 3순위 : y 최소
//! 4순위 : 주변 패턴과의 접촉도 최대
//! ```
//!
//! 의류 nesting에서는 다음 후보가 더 좋다.
//!
//! ```text
//! Score(p) = w1 L(p) + w2 Waste(p) - w3 Contact(p) + w4 Y(p)
//! ```
//!
//! * `L(p)` = 배치 후 strip length
//! * `Waste(p)` = 새로 발생하는 unusable void
//! * `Contact(p)` = 기존 패턴/strip boundary와 맞닿는 길이
//! * `Y(p)` = 세로 위치
//!
//! 즉 같은 strip length라면 **주변 형상에 더 밀착되는 위치**를 선택한다.
//!
//! ```text
//! Candidate A          Candidate B
//! AAAAA  PPPP          AAAAAPPPP
//! AAAAA  PPPP          AAAAAPPPP
//! 공백 큼               밀착
//! ```
//!
//! `B`를 선택한다. 이게 후속 아이템의 공간을 확보하는 데 상당히 중요하다.
//!
//! 또한 초기 배치 단계에서는 **근사 점수**를 쓰는 편이 좋다:
//!
//! ```text
//! StripLength
//! Contact
//! Bounding-box / local-gap 기반 Void approximation
//! ```
//!
//! 매 candidate마다 실제 polygon union이나 남은 빈 공간을 정확하게 계산하면 엄청
//! 느려진다. 그래서 이 모듈의 [`approx_waste`] / [`approx_contact`]는 전부 bbox와
//! local gap만으로 계산하며 polygon union을 절대 쓰지 않는다.
//!
//! # 부호 규약
//!
//! **점수는 낮을수록 좋은 후보다(minimize).** [`PlacementScoreWeights::score`]가
//! 돌려주는 [`CandidateScore::value`]를 오름차순으로 정렬하면 좋은 후보가 앞에 온다.
//! 네 항 중 `Contact`만 **감점(-)**이고 나머지 세 항(`L`, `Waste`, `Y`)은 가점(+)이다.
//! 접촉도가 클수록 점수가 내려가야 밀착 후보가 선택되기 때문이다.

use std::cmp::Ordering;

use crate::nfp_lbf::{
    DEFAULT_SCORE_W1_STRIP_LENGTH, DEFAULT_SCORE_W2_WASTE,
    DEFAULT_SCORE_W3_CONTACT, DEFAULT_SCORE_W4_Y, STRIP_LENGTH_TIE_EPS,
};
use jagua_rs::geometry::primitives::Rect;

// ── 내부 구현 상수 (문서 지정 가중치·허용 오차는 crate::nfp_lbf에 있다) ──

/// 0 나눗셈 방지용 하한. 정규화 분모가 이 값보다 작으면 이 값으로 대체한다.
const MIN_DIVISOR: f32 = 1.0e-6;

/// 정사각형 가정으로 bbox 둘레를 면적에서 역산할 때 쓰는 계수.
/// `perimeter ≈ 4 · √area`.
const PERIMETER_FROM_AREA_FACTOR: f32 = 4.0;

// ────────────────────────────────────────────────────────────────────────────
// 가중치 · 항목
// ────────────────────────────────────────────────────────────────────────────

/// `Score(p) = w1·L(p) + w2·Waste(p) − w3·Contact(p) + w4·Y(p)`의 가중치 묶음.
///
/// 기본값은 [`crate::nfp_lbf`]의 확정 상수를 그대로 쓴다.
#[derive(Clone, Copy, Debug)]
pub struct PlacementScoreWeights {
    /// `w1` — 배치 후 strip length `L(p)`에 곱한다. (+)
    pub w1_strip_length: f32,
    /// `w2` — 새로 발생하는 unusable void `Waste(p)`에 곱한다. (+)
    pub w2_waste: f32,
    /// `w3` — 기존 패턴/strip boundary와 맞닿는 길이 `Contact(p)`에 곱한다.
    /// **적용 부호는 `-`다.** 이 부호는 절대 바뀌면 안 된다.
    pub w3_contact: f32,
    /// `w4` — 세로 위치 `Y(p)`에 곱한다. (+)
    pub w4_y: f32,
    /// 가중합 점수를 **1순위**로 쓸지. 기본값 false.
    ///
    /// # 왜 노브인가
    ///
    /// 기본 선택은 [`compare_by_priority`] 의 사전식 순서다 —
    /// `strip length → x → y → 접촉`. x 가 연속값이라 동률이 거의 없으므로 접촉 항은
    /// **사실상 아무것도 결정하지 못하고**, 가중합은 4순위까지 동률일 때만 쓰인다.
    /// 즉 `w2`~`w4` 가 있어도 배치는 순수 좌·하단 규칙이다.
    ///
    /// `true` 면 가중합을 먼저 보고 사전식 순서를 동률 처리로 내린다. 그러면 접촉
    /// 길이와 낭비 면적이 실제로 자리를 고른다.
    ///
    /// 단, 이것만 켜면 접촉이 strip 길이를 **이겨 버린다** — 정규화된 항에서 `w1=1.0`
    /// 대비 `w3=0.25` 라 접촉이 큰 후보가 길이를 밀어낸다. 156개를 그렇게 쌓으면
    /// 길이가 계속 밀려 기본 LBF 보다 나빠진다(실측 72.549% -> 71.362%).
    /// 그래서 보통은 [`length_tie_mm`](Self::length_tie_mm) 쪽을 쓴다.
    pub score_first: bool,
    /// **길이 동률 허용치(mm).** `0` 이면 끔.
    ///
    /// strip 길이 차이가 이 값 이하인 후보들만 동률로 보고, 그 안에서 맞물림 품질
    /// (`w2·낭비 − w3·접촉 + w4·y`)로 고른다. 길이 차이가 이보다 크면 언제나 짧은
    /// 쪽이 이긴다 — 접촉이 길이를 이기는 일이 구조적으로 불가능해진다.
    ///
    /// 기본 사전식 순서는 길이 동률일 때 `x → y` 로 갈라서, x 가 연속값이라 접촉 항이
    /// 사실상 아무것도 결정하지 못했다. 이 값은 그 자리를 맞물림 품질로 바꾼다.
    pub length_tie_mm: f32,
}

impl Default for PlacementScoreWeights {
    fn default() -> Self {
        PlacementScoreWeights {
            w1_strip_length: DEFAULT_SCORE_W1_STRIP_LENGTH,
            w2_waste: DEFAULT_SCORE_W2_WASTE,
            w3_contact: DEFAULT_SCORE_W3_CONTACT,
            w4_y: DEFAULT_SCORE_W4_Y,
            score_first: false,
            length_tie_mm: 0.0,
        }
    }
}

/// 한 후보 위치에서 측정한 원시(또는 정규화된) 평가 항목.
///
/// `x`는 점수식에는 들어가지 않지만 [`compare_by_priority`]의 2순위 비교에 쓰인다.
#[derive(Clone, Copy, Debug, Default)]
pub struct ScoreTerms {
    /// `L(p)` — 이 후보를 배치한 뒤의 strip length.
    pub strip_length: f32,
    /// `Waste(p)` — 새로 발생하는 unusable void의 근사 면적.
    pub waste: f32,
    /// `Contact(p)` — 기존 패턴 및 strip boundary와 맞닿는 길이의 합.
    pub contact: f32,
    /// 후보 bbox의 좌측 좌표(2순위 비교용).
    pub x: f32,
    /// `Y(p)` — 후보 bbox의 하단 좌표.
    pub y: f32,
}

/// 가중합 점수와 그 산출에 쓰인 항목. **값이 작을수록 좋은 후보다.**
#[derive(Clone, Copy, Debug)]
pub struct CandidateScore {
    /// `w1·L + w2·Waste − w3·Contact + w4·Y`. 낮을수록 좋다.
    pub value: f32,
    /// 점수 산출에 쓰인 항목 원본. 동률 처리(4단 우선순위)에 다시 쓴다.
    pub terms: ScoreTerms,
}

impl PlacementScoreWeights {
    /// 문서 수식 그대로 계산한다.
    ///
    /// ```text
    /// Score(p) = w1 L(p) + w2 Waste(p) - w3 Contact(p) + w4 Y(p)
    /// ```
    ///
    /// `contact` 항만 부호가 `-`다. 접촉도가 클수록 점수가 낮아져(= 더 좋아져)
    /// 같은 strip length라면 주변 형상에 더 밀착되는 위치가 선택된다.
    /// 길이를 뺀 **맞물림 품질**만의 값. 낮을수록 좋다.
    ///
    /// `w2·Waste − w3·Contact + w4·Y`. 길이 동률 구간 안에서 자리를 고를 때 쓴다 —
    /// 이미 길이가 같다고 본 구간이므로 길이 항을 다시 더하면 노이즈만 섞인다.
    #[inline]
    pub fn fit_value(&self, terms: ScoreTerms) -> f32 {
        self.w2_waste * terms.waste - self.w3_contact * terms.contact + self.w4_y * terms.y
    }

    #[inline]
    pub fn score(&self, terms: ScoreTerms) -> CandidateScore {
        let value = self.w1_strip_length * terms.strip_length + self.w2_waste * terms.waste
            - self.w3_contact * terms.contact
            + self.w4_y * terms.y;
        CandidateScore { value, terms }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 4단 우선순위 비교
// ────────────────────────────────────────────────────────────────────────────

/// 문서의 4단 우선순위를 그 순서 그대로 비교한다.
///
/// ```text
/// 1순위 : 배치 후 Strip Length 최소
/// 2순위 : x 최소
/// 3순위 : y 최소
/// 4순위 : 주변 패턴과의 접촉도 최대
/// ```
///
/// [`Ordering::Less`]는 **`a`가 더 좋은 후보**라는 뜻이다(정렬하면 좋은 후보가 앞에
/// 온다). 1순위는 [`STRIP_LENGTH_TIE_EPS`], 2·3순위는 [`BBOX_TOUCH_EPS_MM`] 이내면 동률로
/// 보고 다음 순위로 내려간다. 마지막 순위까지 동률이면 [`Ordering::Equal`]이다.
///
/// f32 비교는 전부 [`f32::total_cmp`]로 하므로 `NaN`이 섞여도 전순서가 깨지지 않는다
/// (`NaN`은 동률 판정에서 항상 탈락하고 `total_cmp`의 결정적 순서를 따른다).
pub fn compare_by_priority(a: &ScoreTerms, b: &ScoreTerms) -> Ordering {
    // 1순위 — strip length 오름차순
    let ord = cmp_ascending(a.strip_length, b.strip_length, STRIP_LENGTH_TIE_EPS);
    if ord != Ordering::Equal {
        return ord;
    }
    // 2순위 — x 오름차순
    let ord = cmp_ascending(a.x, b.x, BBOX_TOUCH_EPS_MM);
    if ord != Ordering::Equal {
        return ord;
    }
    // 3순위 — y 오름차순
    let ord = cmp_ascending(a.y, b.y, BBOX_TOUCH_EPS_MM);
    if ord != Ordering::Equal {
        return ord;
    }
    // 4순위 — contact 내림차순(접촉도 최대). 인자를 뒤집어 오름차순 비교를 재사용한다.
    cmp_ascending(b.contact, a.contact, BBOX_TOUCH_EPS_MM)
}

/// `eps` 이내면 동률, 아니면 오름차순. `NaN`은 `<= eps`가 항상 거짓이므로
/// [`f32::total_cmp`]의 결정적 순서로 넘어간다.
#[inline]
fn cmp_ascending(a: f32, b: f32, eps: f32) -> Ordering {
    if (a - b).abs() <= eps {
        Ordering::Equal
    } else {
        a.total_cmp(&b)
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 근사 계산 헬퍼 — polygon union을 쓰지 않는다
// ────────────────────────────────────────────────────────────────────────────

/// 새로 발생하는 unusable void의 **근사** 면적.
///
/// ```text
/// Bounding-box / local-gap 기반 Void approximation
/// ```
///
/// 두 성분을 더한다.
///
/// 1. **bbox 여유분** — 배치된 아이템 bbox 안에서 실제 도형이 채우지 못한 면적
///    (`placed_bbox.area() - item_area`). 오목한 의류 패턴일수록 커진다.
/// 2. **local gap** — 좌(`-x`)/하(`-y`) 방향으로 가장 가까운 이웃(없으면 strip의
///    `x = 0` / `y = 0` 경계)까지 남은 띠 모양 공백. 이 방향은 이미 지나간 방향이라
///    후속 아이템이 채우기 어려우므로 낭비로 계산한다.
///
/// polygon union도, 남은 빈 공간의 정확한 계산도 하지 않는다. 매 candidate마다
/// 정확히 계산하면 엄청 느려지기 때문이다.
pub fn approx_waste(placed_bbox: Rect, neighbour_bboxes: &[Rect], item_area: f32) -> f32 {
    let width = placed_bbox.width().max(0.0);
    let height = placed_bbox.height().max(0.0);

    // ① bbox 안에서 도형이 채우지 못한 면적
    let bbox_slack = (placed_bbox.area() - item_area).max(0.0);

    // ② 좌측 local gap — y축이 겹치는 이웃 중 가장 오른쪽 끝(없으면 x = 0)
    let mut left_ref = 0.0_f32;
    // ③ 하단 local gap — x축이 겹치는 이웃 중 가장 위쪽 끝(없으면 y = 0)
    let mut bottom_ref = 0.0_f32;

    for nb in neighbour_bboxes {
        let y_overlap = overlap_len(placed_bbox.y_min, placed_bbox.y_max, nb.y_min, nb.y_max);
        if y_overlap > 0.0 && nb.x_max <= placed_bbox.x_min && nb.x_max > left_ref {
            left_ref = nb.x_max;
        }
        let x_overlap = overlap_len(placed_bbox.x_min, placed_bbox.x_max, nb.x_min, nb.x_max);
        if x_overlap > 0.0 && nb.y_max <= placed_bbox.y_min && nb.y_max > bottom_ref {
            bottom_ref = nb.y_max;
        }
    }

    let left_gap = (placed_bbox.x_min - left_ref).max(0.0) * height;
    let bottom_gap = (placed_bbox.y_min - bottom_ref).max(0.0) * width;

    bbox_slack + left_gap + bottom_gap
}

/// bbox 두 개가 "맞닿았다" 고 볼 간극 폭(mm).
///
/// # 왜 `BBOX_TOUCH_EPS_MM`(1e-4mm)를 쓰지 않는가
///
/// 이 판정의 피연산자는 **f32** 좌표다. f32 의 표현 간격은 값의 크기에 비례하므로,
/// 의류 마커 오른쪽 끝(x ~ 7000mm)에서 1 ulp 는
///
/// ```text
/// 7000 x 2^-23 ~= 8.3e-4 mm
/// ```
///
/// 즉 `1e-4` 허용오차는 **표현 한계보다 좁다.** 정확히 맞닿은 두 조각이라도 그 좌표를
/// f32 로 표현하는 순간 간극이 8e-4mm 로 벌어질 수 있고, 그러면 접촉이 0으로 계산된다.
/// 왼쪽(x ~ 0)에서는 1 ulp 가 1e-7mm 라 같은 임계가 잘 동작한다 — **같은 코드가 마커의
/// 어느 쪽이냐에 따라 다르게 판정한다.** 정규화되지 않은 절대 허용오차를 스케일이
/// 변하는 값에 쓴 것이고, 이 코드베이스가 반복해 밟은 함정이다.
///
/// 10 µm 는 marker 전 구간에서 f32 표현 간격보다 10배 이상 크고, 재단 공차(1mm 급)
/// 보다는 두 자리 작다. 즉 **표현 잡음은 흡수하고 기하는 보존한다.**
///
/// [`crate::nfp_lbf::placement`] 의 `CONTACT_BAND_MM`(1.0mm)과 값이 다른 것은 의도적이다.
/// 저쪽은 `1 - g/band` 선형 감쇠라 밴드가 넓어도 먼 조각의 기여가 0에 수렴하지만,
/// 이쪽은 **경성 임계**라 밴드를 넓히면 안 닿은 조각까지 전량 접촉으로 센다.
pub const BBOX_TOUCH_EPS_MM: f32 = 1.0e-2;

/// 기존 패턴 및 strip boundary와 맞닿는 **길이**의 합.
///
/// 두 bbox가 한 축에서 서로 접하고(간격 `<= BBOX_TOUCH_EPS_MM`) 다른 축에서 겹치면 그
/// 겹침 길이를 더한다. strip 경계는 `x = 0`, `y = 0`, `y = strip_height` 세 곳을 센다.
/// strip이 자라나는 방향인 `x = strip_width`(오른쪽 끝)는 접촉으로 세지 않는다 —
/// 그쪽에 붙는 것은 밀착이 아니라 strip을 늘리는 행위이기 때문이다.
///
/// `strip_width` / `strip_height`는 접촉 길이를 strip 내부로 잘라내는 데에도 쓴다.
/// 값이 `0.0` 이하이면 해당 축의 상한 제한이 없는 것으로 본다.
pub fn approx_contact(
    placed_bbox: Rect,
    neighbour_bboxes: &[Rect],
    strip_width: f32,
    strip_height: f32,
) -> f32 {
    let sw = if strip_width > 0.0 {
        strip_width
    } else {
        f32::INFINITY
    };
    let sh = if strip_height > 0.0 {
        strip_height
    } else {
        f32::INFINITY
    };

    let placed = clip_to_strip(placed_bbox, sw, sh);
    let mut contact = 0.0_f32;

    // ① 이웃 패턴과의 접촉
    for nb in neighbour_bboxes {
        contact += touch_length(placed, clip_to_strip(*nb, sw, sh));
    }

    // ② strip 경계와의 접촉 — x = 0, y = 0, y = strip_height
    let width = (placed.x_max - placed.x_min).max(0.0);
    let height = (placed.y_max - placed.y_min).max(0.0);
    if placed.x_min.abs() <= BBOX_TOUCH_EPS_MM {
        contact += height;
    }
    if placed.y_min.abs() <= BBOX_TOUCH_EPS_MM {
        contact += width;
    }
    if sh.is_finite() && (sh - placed.y_max).abs() <= BBOX_TOUCH_EPS_MM {
        contact += width;
    }

    contact
}

/// 세 항의 스케일 차이를 없앤다.
///
/// * `strip_length`, `x`, `y` — 길이 차원이므로 `strip_height`로 나눈다.
///   `x`는 점수식에 들어가지 않지만, 세 항이 같은 배율이라야 이 구조체를 로그나
///   진단에 찍을 때 서로 비교 가능한 값이 된다.
///
/// # 우선순위 비교에는 쓰지 마라
///
/// 4단 우선순위([`compare_by_priority`])는 **정규화하지 않은 실제 mm 값**으로 비교해야
/// 한다. 정규화한 값을 넣으면 축마다 나누는 기준이 달라져 같은 1mm 차이가 축에 따라
/// 동률이 되기도 하고 아니기도 한다 — 실제로 그렇게 만들었다가 한 줄 배치가 나왔고
/// (strip 126.9 -> 고친 뒤 42.0), 그 이후 raw 비교로 되돌렸다.
///
/// 정규화는 **차원이 다른 항을 하나의 가중합으로 더하는** `Score(p)` 에만 쓴다.
///
/// > 같은 이름의 함수가 [`crate::nfp_lbf::placement`] 에도 있고 **분모가 다르다**
/// > (그쪽은 `strip_length`·`x` 를 `strip_width_cap` 으로 나눈다). 둘은 서로 대체할 수
/// > 없다. 두 함수 모두 우선순위 비교에는 쓰이지 않으므로 분모 차이가 순서를 바꾸지는
/// > 않지만, 로그의 수치를 서로 비교하면 안 된다.
/// * `waste` — 면적 차원이므로 `item_area`로 나눈다.
/// * `contact` — 길이 차원이지만 스케일 기준이 아이템 자신이므로 bbox 둘레로 나눈다.
///   bbox를 직접 받지 않으므로 정사각형을 가정해 `perimeter ≈ 4·√area`로 역산한다.
///
/// 모든 분모는 `MIN_DIVISOR`(1.0e-6)로 하한을 잡아 0으로 나누지 않는다.
pub fn normalize_terms(raw: ScoreTerms, strip_height: f32, item_area: f32) -> ScoreTerms {
    let len_div = strip_height.max(MIN_DIVISOR);
    let area_div = item_area.max(MIN_DIVISOR);
    let perimeter_div =
        (PERIMETER_FROM_AREA_FACTOR * item_area.max(0.0).sqrt()).max(MIN_DIVISOR);

    ScoreTerms {
        strip_length: raw.strip_length / len_div,
        waste: raw.waste / area_div,
        contact: raw.contact / perimeter_div,
        x: raw.x / len_div,
        y: raw.y / len_div,
    }
}

// ── 내부 기하 헬퍼 ──

/// 1차원 구간 `[a_min, a_max]`와 `[b_min, b_max]`의 겹침 길이(음수면 0).
#[inline]
fn overlap_len(a_min: f32, a_max: f32, b_min: f32, b_max: f32) -> f32 {
    (a_max.min(b_max) - a_min.max(b_min)).max(0.0)
}

/// bbox를 strip 영역 `[0, sw] × [0, sh]`으로 잘라낸다. 완전히 밖이면 폭·높이가 0인
/// 축퇴 사각형이 되어 접촉 길이에 기여하지 않는다.
#[inline]
fn clip_to_strip(r: Rect, sw: f32, sh: f32) -> Rect {
    let x_min = r.x_min.clamp(0.0, sw);
    let x_max = r.x_max.clamp(0.0, sw).max(x_min);
    let y_min = r.y_min.clamp(0.0, sh);
    let y_max = r.y_max.clamp(0.0, sh).max(y_min);
    Rect {
        x_min,
        y_min,
        x_max,
        y_max,
    }
}

/// 두 bbox가 맞닿는 길이. 한 축에서 간격이 [`BBOX_TOUCH_EPS_MM`] 이내로 접하고 다른 축에서
/// 겹치면 그 겹침 길이를 더한다. 모서리만 스치는 경우는 겹침이 0이라 기여하지 않는다.
#[inline]
fn touch_length(a: Rect, b: Rect) -> f32 {
    let mut len = 0.0_f32;

    // 세로 변끼리 접촉 → 겹침은 y축에서 측정
    let x_touch =
        (a.x_max - b.x_min).abs() <= BBOX_TOUCH_EPS_MM || (b.x_max - a.x_min).abs() <= BBOX_TOUCH_EPS_MM;
    if x_touch {
        len += overlap_len(a.y_min, a.y_max, b.y_min, b.y_max);
    }

    // 가로 변끼리 접촉 → 겹침은 x축에서 측정
    let y_touch =
        (a.y_max - b.y_min).abs() <= BBOX_TOUCH_EPS_MM || (b.y_max - a.y_min).abs() <= BBOX_TOUCH_EPS_MM;
    if y_touch {
        len += overlap_len(a.x_min, a.x_max, b.x_min, b.x_max);
    }

    len
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(x_min: f32, y_min: f32, x_max: f32, y_max: f32) -> Rect {
        Rect {
            x_min,
            y_min,
            x_max,
            y_max,
        }
    }

    fn terms(strip_length: f32, x: f32, y: f32, contact: f32) -> ScoreTerms {
        ScoreTerms {
            strip_length,
            waste: 0.0,
            contact,
            x,
            y,
        }
    }

    // ── (a) 부호 규약 ──

    #[test]
    fn default_weights_match_the_document_constants() {
        let w = PlacementScoreWeights::default();
        assert_eq!(w.w1_strip_length, DEFAULT_SCORE_W1_STRIP_LENGTH);
        assert_eq!(w.w2_waste, DEFAULT_SCORE_W2_WASTE);
        assert_eq!(w.w3_contact, DEFAULT_SCORE_W3_CONTACT);
        assert_eq!(w.w4_y, DEFAULT_SCORE_W4_Y);
    }

    #[test]
    fn score_follows_the_document_formula_exactly() {
        let w = PlacementScoreWeights {
            w1_strip_length: 1.0,
            w2_waste: 2.0,
            w3_contact: 3.0,
            w4_y: 4.0,
            // 이 시험은 «문서 수식» 을 검증한다 — 사전식 동률 규칙은 끄고
            // 가중합만 1순위로 쓴다(종전 동작과 같다).
            score_first: true,
            length_tie_mm: 0.0,
        };
        let t = ScoreTerms {
            strip_length: 10.0,
            waste: 5.0,
            contact: 7.0,
            x: 99.0, // 점수식에는 들어가지 않는다
            y: 2.0,
        };
        // 1*10 + 2*5 - 3*7 + 4*2 = 10 + 10 - 21 + 8 = 7
        let s = w.score(t);
        assert!((s.value - 7.0).abs() < 1e-5, "value = {}", s.value);
        assert_eq!(s.terms.x, 99.0);
    }

    #[test]
    fn contact_is_the_only_negative_term() {
        let w = PlacementScoreWeights::default();
        let base = ScoreTerms {
            strip_length: 100.0,
            waste: 10.0,
            contact: 0.0,
            x: 0.0,
            y: 5.0,
        };

        // contact가 커지면 점수는 반드시 내려간다(= 좋아진다).
        let more_contact = ScoreTerms {
            contact: 20.0,
            ..base
        };
        assert!(w.score(more_contact).value < w.score(base).value);

        // 나머지 세 항은 커지면 점수가 올라간다(= 나빠진다).
        assert!(
            w.score(ScoreTerms {
                strip_length: 200.0,
                ..base
            })
            .value
                > w.score(base).value
        );
        assert!(w.score(ScoreTerms { waste: 20.0, ..base }).value > w.score(base).value);
        assert!(w.score(ScoreTerms { y: 50.0, ..base }).value > w.score(base).value);

        // 감점량은 정확히 w3 × contact다.
        let delta = w.score(base).value - w.score(more_contact).value;
        assert!((delta - DEFAULT_SCORE_W3_CONTACT * 20.0).abs() < 1e-4);
    }

    // ── (b) 4단 우선순위 ──

    #[test]
    fn priority_1_strip_length_ascending() {
        let short = terms(100.0, 50.0, 50.0, 0.0);
        let long = terms(120.0, 0.0, 0.0, 999.0);
        // strip length가 이기면 x·y·contact가 아무리 나빠도 뒤집히지 않는다.
        assert_eq!(compare_by_priority(&short, &long), Ordering::Less);
        assert_eq!(compare_by_priority(&long, &short), Ordering::Greater);
    }

    #[test]
    fn priority_2_x_ascending_when_strip_length_ties() {
        // 차이가 STRIP_LENGTH_TIE_EPS 이내면 1순위는 동률
        let a = terms(100.0, 10.0, 90.0, 0.0);
        let b = terms(100.0 + STRIP_LENGTH_TIE_EPS * 0.5, 20.0, 0.0, 999.0);
        assert_eq!(compare_by_priority(&a, &b), Ordering::Less);
        assert_eq!(compare_by_priority(&b, &a), Ordering::Greater);
    }

    #[test]
    fn priority_3_y_ascending_when_strip_length_and_x_tie() {
        let a = terms(100.0, 10.0, 3.0, 0.0);
        let b = terms(100.0, 10.0 + BBOX_TOUCH_EPS_MM * 0.5, 8.0, 999.0);
        assert_eq!(compare_by_priority(&a, &b), Ordering::Less);
        assert_eq!(compare_by_priority(&b, &a), Ordering::Greater);
    }

    #[test]
    fn priority_4_contact_descending_and_full_tie_is_equal() {
        let tight = terms(100.0, 10.0, 3.0, 42.0);
        let loose = terms(100.0, 10.0, 3.0, 7.0);
        // 접촉도는 클수록 좋다 → 내림차순
        assert_eq!(compare_by_priority(&tight, &loose), Ordering::Less);
        assert_eq!(compare_by_priority(&loose, &tight), Ordering::Greater);
        // 마지막까지 동률이면 Equal
        assert_eq!(compare_by_priority(&tight, &tight), Ordering::Equal);
    }

    #[test]
    fn nan_does_not_break_the_total_order() {
        let sane = terms(100.0, 10.0, 3.0, 5.0);
        let nasty = terms(f32::NAN, f32::NAN, f32::NAN, f32::NAN);
        let ord = compare_by_priority(&sane, &nasty);
        assert_ne!(ord, Ordering::Equal);
        assert_eq!(compare_by_priority(&nasty, &sane), ord.reverse());
        // 정렬해도 패닉하지 않는다.
        let mut v = vec![nasty, sane, terms(100.0, 10.0, 3.0, 5.0)];
        v.sort_by(compare_by_priority);
        assert_eq!(v.len(), 3);
    }

    // ── 근사 헬퍼 ──

    #[test]
    fn approx_contact_counts_neighbour_and_strip_boundaries() {
        // 4×2 아이템을 좌하단 구석에 붙인다. 이웃은 없다.
        let placed = rect(0.0, 0.0, 4.0, 2.0);
        // x = 0 (길이 2) + y = 0 (길이 4) = 6
        let c = approx_contact(placed, &[], 100.0, 10.0);
        assert!((c - 6.0).abs() < 1e-4, "c = {c}");

        // y = strip_height(=2)에도 붙으면 위쪽 4가 더해진다.
        let c = approx_contact(placed, &[], 100.0, 2.0);
        assert!((c - 10.0).abs() < 1e-4, "c = {c}");

        // 오른쪽 이웃과 변으로 맞닿으면 겹침 길이만큼 더해진다.
        let nb = rect(4.0, 0.0, 8.0, 1.5);
        let c = approx_contact(placed, &[nb], 100.0, 10.0);
        assert!((c - (6.0 + 1.5)).abs() < 1e-4, "c = {c}");

        // 모서리만 스치면 겹침이 0이라 기여하지 않는다.
        let corner = rect(4.0, 2.0, 8.0, 4.0);
        let c = approx_contact(placed, &[corner], 100.0, 10.0);
        assert!((c - 6.0).abs() < 1e-4, "c = {c}");
    }

    #[test]
    fn approx_waste_adds_bbox_slack_and_left_bottom_gaps() {
        let placed = rect(5.0, 0.0, 9.0, 2.0); // bbox 면적 8

        // 도형이 bbox를 절반만 채우면 여유분 4, 이웃 없음 → 좌측 gap = 5*2 = 10
        let w = approx_waste(placed, &[], 4.0);
        assert!((w - (4.0 + 10.0)).abs() < 1e-4, "w = {w}");

        // 좌측에 딱 붙은 이웃이 있으면 좌측 gap이 사라진다.
        let hug = rect(0.0, 0.0, 5.0, 2.0);
        let w = approx_waste(placed, &[hug], 8.0);
        assert!(w.abs() < 1e-4, "w = {w}");

        // 이웃이 2mm 떨어져 있으면 그 띠(2 × 높이 2 = 4)가 낭비로 잡힌다.
        let far = rect(0.0, 0.0, 3.0, 2.0);
        let w = approx_waste(placed, &[far], 8.0);
        assert!((w - 4.0).abs() < 1e-4, "w = {w}");
    }

    #[test]
    fn normalize_terms_is_scale_safe() {
        let raw = ScoreTerms {
            strip_length: 200.0,
            waste: 50.0,
            contact: 40.0,
            x: 100.0,
            y: 20.0,
        };
        let n = normalize_terms(raw, 100.0, 25.0);
        assert!((n.strip_length - 2.0).abs() < 1e-5);
        assert!((n.y - 0.2).abs() < 1e-5);
        assert!((n.x - 1.0).abs() < 1e-5);
        assert!((n.waste - 2.0).abs() < 1e-5);
        // perimeter ≈ 4·√25 = 20 → 40 / 20 = 2
        assert!((n.contact - 2.0).abs() < 1e-5);

        // 0 분모여도 폭발하지 않는다.
        let z = normalize_terms(raw, 0.0, 0.0);
        assert!(z.strip_length.is_finite());
        assert!(z.waste.is_finite());
        assert!(z.contact.is_finite());
        assert!(z.y.is_finite());
    }

    // ── (c) 문서의 Candidate A vs B ──

    /// ```text
    /// Candidate A          Candidate B
    /// AAAAA  PPPP          AAAAAPPPP
    /// AAAAA  PPPP          AAAAAPPPP
    /// 공백 큼               밀착
    /// ```
    ///
    /// 같은 아이템 `P`를 이미 배치된 `A` 옆에 놓는다. `B`는 밀착, `A`는 공백을 남긴다.
    /// strip length가 달라지므로 1순위에서 이미 `B`가 이긴다.
    #[test]
    fn doc_example_prefers_the_tight_candidate_b() {
        let strip_height = 2.0;
        let strip_width_cap = 100.0;
        let neighbour = rect(0.0, 0.0, 5.0, 2.0); // AAAAA
        let item_area = 8.0; // 4 × 2 직사각 패턴

        // B — 밀착: x = 5에서 시작
        let b_box = rect(5.0, 0.0, 9.0, 2.0);
        let b = ScoreTerms {
            strip_length: b_box.x_max,
            waste: approx_waste(b_box, &[neighbour], item_area),
            contact: approx_contact(b_box, &[neighbour], strip_width_cap, strip_height),
            x: b_box.x_min,
            y: b_box.y_min,
        };

        // A — 공백 큼: x = 7에서 시작(2mm 벌어짐)
        let a_box = rect(7.0, 0.0, 11.0, 2.0);
        let a = ScoreTerms {
            strip_length: a_box.x_max,
            waste: approx_waste(a_box, &[neighbour], item_area),
            contact: approx_contact(a_box, &[neighbour], strip_width_cap, strip_height),
            x: a_box.x_min,
            y: a_box.y_min,
        };

        // 밀착한 B가 접촉도는 크고 낭비는 작다.
        assert!(b.contact > a.contact, "b={} a={}", b.contact, a.contact);
        assert!(b.waste < a.waste, "b={} a={}", b.waste, a.waste);

        // 1순위(strip length)에서 이미 B가 이긴다.
        assert_eq!(compare_by_priority(&b, &a), Ordering::Less);

        // 가중합 점수도 B가 낮다(= 좋다).
        let w = PlacementScoreWeights::default();
        assert!(w.score(b).value < w.score(a).value);
    }

    /// 같은 strip length·x·y라면 **접촉도가 큰 쪽(밀착)** 이 선택된다.
    ///
    /// `P`의 위치는 동일하고 이미 배치된 `A`의 길이만 다른 두 상황을 비교한다.
    #[test]
    fn same_strip_length_picks_the_higher_contact_candidate() {
        let strip_height = 10.0;
        let strip_width_cap = 100.0;
        let item_area = 8.0;
        let p_box = rect(5.0, 0.0, 9.0, 2.0);

        let measure = |neighbour: Rect| ScoreTerms {
            strip_length: p_box.x_max,
            waste: approx_waste(p_box, &[neighbour], item_area),
            contact: approx_contact(p_box, &[neighbour], strip_width_cap, strip_height),
            x: p_box.x_min,
            y: p_box.y_min,
        };

        let b = measure(rect(0.0, 0.0, 5.0, 2.0)); // 밀착
        let a = measure(rect(0.0, 0.0, 3.0, 2.0)); // 공백 큼

        // 1·2·3순위는 전부 동률이다.
        assert_eq!(b.strip_length, a.strip_length);
        assert_eq!(b.x, a.x);
        assert_eq!(b.y, a.y);

        // 4순위에서 접촉도가 큰 B가 선택된다.
        assert!(b.contact > a.contact, "b={} a={}", b.contact, a.contact);
        assert_eq!(compare_by_priority(&b, &a), Ordering::Less);
        assert_eq!(compare_by_priority(&a, &b), Ordering::Greater);

        // 가중합 점수도 B가 낮다.
        let w = PlacementScoreWeights::default();
        assert!(w.score(b).value < w.score(a).value);
    }
}
