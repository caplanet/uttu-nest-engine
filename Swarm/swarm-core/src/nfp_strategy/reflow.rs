//! 꼬리 회수 + 저밀도 띠 재구성 — 배치가 끝난 뒤 **폭을 정하는 조각들만** 다시 넣는다.
//!
//! # 왜 이 패스가 있어야 하나 — 손실이 어디에 있는지 재 봤다
//!
//! `fit_strip` 은 가장 오른쪽 조각의 `x_max` 에서 strip 을 자른다. 그러면 마지막 몇
//! 조각이 프론티어를 크게 밀었을 때 **그 뒤가 통째로 빈 채 폭에 포함**된다.
//! x축을 5% 구간으로 잘라 채움률을 재면 그게 그대로 보인다(마지막 10구간, %):
//!
//! ```text
//! wsjk156   60 64 75 79 76 69 76 79 59 35     <- 마지막 2구간만 떨어진다
//! albano    90 90 85 71 84 78 76 44 28 13     <- 마지막 3구간이 무너진다
//! fu        61 67 61 70 76 69 63 49 23  5     <- 마지막 3구간이 무너진다
//! ```
//!
//! albano 는 폭을 90% 지점에서 자르면 73.57% -> 76.50% 다. **조각 5개가 2.9%p 를
//! 먹고 있다**는 뜻이다. 다만 그 조각들을 버릴 수는 없으므로, 여기서는 **빼서 안쪽
//! 포켓에 다시 넣는다**.
//!
//! wsjk156 은 꼬리보다 **중간 띠**가 문제다(x 50~60% 부근이 60~64%). 그래서 꼬리와
//! "채움률 최저 띠" 둘 다를 후보로 본다.
//!
//! # 밀도를 어떻게 재야 하는가 — 먼저 이것부터
//!
//! 이 파일의 첫 실측표는 밀도를 **NFP 라이브러리 도형 면적**으로 재고 있었다. 그 도형은
//! `concave_simplify_tolerance` 로 오목을 메운 원본의 **상위집합**이라 실제보다 크다.
//! 실측: wsjk156 은 인스턴스 면적 7,760,261 에 대해 라이브러리 면적 7,880,393 —
//! **1.55% 부풀어 있다**(albano 는 0.98%). 밀도의 분자가 그만큼 커지니 결과가
//! 통째로 1.1~1.2%p 높게 나온다. 폭은 정확했는데 밀도만 틀렸던 이유가 이것이다.
//!
//! 확인: 폭 6943.1mm, strip 높이 1524 에서
//! `7,880,393 / (6943.1 × 1524) = 74.48%` (옛 표의 값),
//! `7,760,261 / (6943.1 × 1524) = 73.27%` (실제 값).
//!
//! **밀도는 반드시 `SPSolution::density(instance)` 로 잰다.** 아래 표는 전부 그것이다.
//!
//! # 실측 — 5초 예산을 ordering 과 이 패스에 나눠 준 결과
//!
//! 대조군은 "이 패스 없이 ordering 만 예산이 다 탈 때까지 돌리기" 다. 같은 시간을 쓰는
//! 다른 쓸 곳과 나란히 놓지 않으면 이득인지 알 수 없다.
//!
//! **예산은 NFP 콜드 캐시를 포함한 벽시계 5초다.** 캐시를 예산 밖으로 빼면 안 된다 —
//! wsjk156 의 콜드 캐시는 `concave_simplify_tolerance = 0.02` 에서 0.7초,
//! `0.0` 에서 **3.8초**다. 그걸 빼고 "5초" 라고 하면 실제로는 8.7초짜리 실험이 된다.
//!
//! ```text
//! wsjk156 (156조각, strip 1524) — 벽시계 5초, 탐침 4096, 단순화 0.02
//!   대조: ordering 만 (예산 소진, ord 11)   71.75%  폭 7090.0
//!   ordering  1 + reflow                  73.27%  폭 6943.1   <- +1.52%p
//!   ordering  2 + reflow                  73.27%  폭 6943.1
//!   ordering  4 + reflow                  72.68%  폭 6999.4
//!   ordering  8 + reflow                  72.09%  폭 7056.5
//!
//! albano (24조각)
//!   대조: ordering 만 (ord 148)            72.06%  폭 12080.7
//!   + reflow                              78.87%  폭 11038.0  <- +6.81%p
//! ```
//!
//! 대조군 71.75% / 폭 7090.0 은 문서가 말하는 기준선과 소수점까지 같다 — 하네스가
//! 프로덕션 경로를 재현하고 있다는 확인이다.
//!
//! 읽는 법: **시드를 키우는 것보다 이 패스에 시간을 주는 편이 낫다.** ordering 을
//! 8개까지 돌리면 이 패스가 도는 시간이 없어 72.09% 에 그친다.
//!
//! 예산 배분 스윕(같은 5초, 시드에 준 비율):
//!
//! | 인스턴스 | reflow 없음 | 시드 10% | 시드 25% | 시드 50% | 시드 75% |
//! |---|---:|---:|---:|---:|---:|
//! | wsjk156 | 71.75% | **73.27%** | 72.68% | 72.18% | 72.09% |
//! | albano | 72.06% | **78.87%** | 78.87% | 78.87% | 78.87% |
//! | swim | 61.28% | 60.44% | 61.29% | **62.81%** | 61.28% |
//! | trousers | 76.46% | **77.44%** | 77.44% | 77.44% | 77.44% |
//! | fu / shirts / dagli | 74.95 / 79.32 / 74.93% | 동일 | 동일 | 동일 | 동일 |
//!
//! **배분을 잘못하면 진다.** swim 을 시드 10% 로 굶기면 60.44% 로 대조군(61.28%)보다
//! 나쁘다 — 이 패스가 해를 나쁘게 만든 것이 아니라 **시드가 그만큼 나빠진** 것이다.
//! 이 패스 자체는 입력보다 나쁜 출력을 낼 수 없다(아래). 인스턴스마다 최적 배분이
//! 다르므로 호출부가 그걸 정해야 한다.
//!
//! fu / shirts / dagli 는 채택 라운드가 0 이고, 예산을 다 쓰기 전에 **일정과 띠를 다
//! 훑고 스스로 멈춘다**(fu 0.03초, dagli 0.6초). 남은 시간은 호출부가 다른 데 써야 한다.
//!
//! # 이것은 탐색이 아니다
//!
//! 난수도, 겹침 허용도, 온도도 쓰지 않는다. 자리는 전부
//! [`SequentialPlacer::candidates_for`](crate::nfp_lbf::SequentialPlacer::candidates_for)
//! 가 준 feasible 앵커이므로 중간 상태를 포함해 **항상 겹침이 없다**. 라운드는
//! `strip 길이가 줄었을 때만` 채택하고 아니면 원복하므로, 이 패스는 정의상
//! **결과를 나쁘게 만들 수 없다**.
//!
//! # ⑦ 슬라이딩([`super::sliding`])과 무엇이 다른가
//!
//! 슬라이딩은 조각 하나를 빼고 **같은 순간에** 다시 넣는다. 그래서 "A 가 비켜 주면
//! B 가 들어갈 수 있다" 는 **2조각 이상의 동시 이동**을 원리적으로 못 한다.
//! 이쪽은 한 라운드에서 여러 조각을 **동시에 빼고** 면적 큰 것부터 다시 넣으므로
//! 그 교환이 가능하다. 대신 되돌리기 단위가 라운드 전체다.
//!
//! # 왜 결과가 나빠질 수 없나
//!
//! 라운드는 (1) 뺀 조각을 **전부** 다시 넣었고 (2) strip 길이가 [`MIN_GAIN_MM`] 이상
//! 줄었을 때에만 채택된다. 둘 중 하나라도 어긋나면 라운드 시작 시점의 배치로 통째로
//! 되돌린다. 그래서 출력의 strip 길이는 입력 이하이고 조각 수는 항상 같다.

use std::collections::BTreeSet;

use super::select::{SelectionRule, pick_best};
use super::{AnchorMetrics, AnchorOption, StrategyContext};
use crate::nfp_lbf::PlacedPiece;
use crate::nfp_lbf::placement::Aabb;

/// 라운드를 채택할 최소 strip 감소폭(mm).
///
/// 앵커가 `f32` 로 굳으므로 같은 자리를 다시 고른 라운드도 1e-7mm 쯤 "줄어" 보인다.
/// 그걸 개선으로 세면 라운드가 영원히 끝나지 않는다. 0.01mm 는 재단 공차보다 훨씬
/// 작아 품질에는 영향이 없다.
const MIN_GAIN_MM: f64 = 0.01;

/// 평지 이동을 채택할 최소 **퍼짐** 감소폭(mm). [`ReflowParams::plateau_moves`] 참조.
///
/// 퍼짐은 조각 156개의 `x_max` 합이라 1mm 는 조각 하나가 1mm 왼쪽으로 간 것과 같다.
/// 이보다 작은 변화는 f32 앵커가 다시 굳으면서 생기는 잡음과 구별되지 않는다.
const MIN_SPREAD_GAIN_MM: f64 = 1.0;

/// 구간(band) 폭 = strip 높이의 몇 배인가. 기본값 근거는 [`ReflowParams`] 문서의 스윕표.
pub const DEFAULT_BAND_WIDTH_RATIO: f64 = 0.5;

/// 꼬리 구간 폭 = strip 높이의 몇 배인가.
pub const DEFAULT_TAIL_RATIO: f64 = 0.75;

/// 한 라운드에 뺄 조각 수의 시작값.
pub const DEFAULT_MIN_REMOVE: usize = 4;

/// 한 라운드에 뺄 조각 수 상한.
///
/// 너무 크면 그냥 재배치라 느리고(조각 하나 재삽입 = 후보 열거 1회), 너무 작으면
/// "A 가 비켜야 B 가 들어간다" 는 교환이 안 일어난다.
pub const DEFAULT_MAX_REMOVE: usize = 64;

/// 기본 라운드 수.
pub const DEFAULT_REFLOW_ROUNDS: usize = 64;

/// 띠 폭 배수 주기. 뺄 개수와 띠를 다 훑고도 못 줄이면 이 순서로 띠 폭을 바꾼다.
///
/// 띠 폭이 바뀌면 "채움률 최저 띠" 가 통째로 다른 조각 묶음을 가리키므로, 같은
/// 배치에서도 **다른 이웃**이 생긴다. 난수를 쓰지 않고 이웃을 넓히는 방법이다.
const BAND_WIDTH_CYCLE: [f64; 4] = [1.0, 2.0, 0.5, 4.0];

/// 재삽입 동안 쓸 기본 탐침 예산. `0`이면 오라클의 현재 값을 그대로 둔다.
/// 근거는 [`ReflowParams::probe_budget`] 의 실측표.
pub const DEFAULT_REFLOW_PROBE_BUDGET: usize = 8_192;

/// 재구성이 실제로 무엇을 하고 있는지 세는 계측 카운터.
///
/// 켜지 않으면 라운드마다 relaxed load 몇 번이다 — 라운드 하나가 조각 수십 개의
/// NFP 후보 열거이므로 계측에 잡히지 않는다. 켜는 것은 `#[ignore]` 계측 테스트뿐이다.
///
/// # 이 카운터가 답한 것
///
/// [`BEST_ATTEMPT_UM`] 이 핵심이다. 라운드가 원복될 때 "얼마나 아깝게 졌나" 를
/// µm 단위로 남긴다. 이 값이 0 근처면 규칙을 조금 손보면 되는 것이고, 크게 양수면
/// 재삽입이 원래 자리를 그대로 되찾고 있다는 뜻이라 **접근 자체가 틀린 것**이다.
#[doc(hidden)]
pub mod meas {
    use std::sync::atomic::{AtomicBool, AtomicI64, AtomicUsize, Ordering as AO};

    pub static ON: AtomicBool = AtomicBool::new(false);
    /// `reflow_with` 호출 수.
    pub static CALLS: AtomicUsize = AtomicUsize::new(0);
    /// 실제로 시도한 라운드 수.
    pub static ROUNDS: AtomicUsize = AtomicUsize::new(0);
    /// 뺀 조각 수의 합(원복된 라운드 포함).
    pub static VICTIMS: AtomicUsize = AtomicUsize::new(0);
    /// 재삽입 자리를 못 찾은 조각 수.
    pub static REINSERT_FAIL: AtomicUsize = AtomicUsize::new(0);
    /// 원복된 라운드 수.
    pub static REVERTS: AtomicUsize = AtomicUsize::new(0);
    /// 채택된 라운드 수.
    pub static ACCEPTS: AtomicUsize = AtomicUsize::new(0);
    /// 그중 **평지 이동**(폭은 그대로, 퍼짐만 줄었다)으로 채택된 라운드 수.
    pub static PLATEAUS: AtomicUsize = AtomicUsize::new(0);
    /// 원복된 라운드 중 **가장 아까웠던** `after - before`(µm). 0 이하면 채택됐을 것.
    pub static BEST_ATTEMPT_UM: AtomicI64 = AtomicI64::new(i64::MAX);
    /// 재삽입 후 **실제로 자리가 바뀐** 조각 수(원복된 라운드 포함).
    ///
    /// 이 값이 0 이면 오라클이 뺀 조각을 전부 제자리에 되돌린 것이다 — 그건 규칙이
    /// 아니라 **접근**이 틀렸다는 뜻이라 노브를 아무리 돌려도 소용이 없다.
    pub static MOVED: AtomicUsize = AtomicUsize::new(0);
    /// 프론티어가 이미 원래 값을 넘어 조기에 끊은 라운드 수.
    pub static EARLY_ABORTS: AtomicUsize = AtomicUsize::new(0);

    pub fn set(on: bool) {
        ON.store(on, AO::Relaxed);
    }
    pub fn is_on() -> bool {
        ON.load(AO::Relaxed)
    }
    pub fn reset() {
        for c in [
            &CALLS,
            &ROUNDS,
            &VICTIMS,
            &REINSERT_FAIL,
            &REVERTS,
            &ACCEPTS,
            &PLATEAUS,
            &MOVED,
            &EARLY_ABORTS,
        ] {
            c.store(0, AO::Relaxed);
        }
        BEST_ATTEMPT_UM.store(i64::MAX, AO::Relaxed);
    }
    /// 원복된 라운드의 `after - before`(mm)를 기록한다.
    pub fn note_attempt(delta_mm: f64) {
        if !is_on() || !delta_mm.is_finite() {
            return;
        }
        let um = (delta_mm * 1_000.0).round().clamp(i64::MIN as f64, i64::MAX as f64) as i64;
        BEST_ATTEMPT_UM.fetch_min(um, AO::Relaxed);
    }
    fn g(c: &AtomicUsize) -> usize {
        c.load(AO::Relaxed)
    }
    pub fn report() -> String {
        let best = BEST_ATTEMPT_UM.load(AO::Relaxed);
        let best = if best == i64::MAX {
            "없음".to_string()
        } else {
            format!("{:+.3}mm", best as f64 / 1_000.0)
        };
        format!(
            "호출 {} / 라운드 {} (채택 {} — 그중 평지 {} / 원복 {}) / 뺀 조각 {} (자리 바뀜 {}) / \
             재삽입 실패 {} / 조기중단 {} / 최선 시도 {}",
            g(&CALLS),
            g(&ROUNDS),
            g(&ACCEPTS),
            g(&PLATEAUS),
            g(&REVERTS),
            g(&VICTIMS),
            g(&MOVED),
            g(&REINSERT_FAIL),
            g(&EARLY_ABORTS),
            best,
        )
    }
}

/// 어느 구간의 조각을 뺄 것인가.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReflowScope {
    /// 꼬리만. 폭을 정하는 것은 꼬리이므로 이것만으로도 이득이 날 수 있다.
    TailOnly,
    /// 채움률 최저 띠만. 꼬리를 건드리지 않으므로 폭이 줄려면 다른 조각이
    /// 자발적으로 그 자리로 이주해야 한다 — 대조군용이다.
    LowBandOnly,
    /// 둘 다. 꼬리 조각이 갈 곳(= 비워진 저밀도 띠)을 같은 라운드에서 만들어 준다.
    Both,
}

/// 뺀 조각을 어떤 순서로 다시 넣을 것인가.
///
/// # 왜 이게 이 패스에서 가장 중요한 노브인가 — 재 봤다
///
/// 오른쪽 끝 `k` 개를 빼서 **오른쪽부터** 다시 넣어 봤다(폭 mm):
///
/// | k | wsjk156 | albano |
/// |---:|---:|---:|
/// | 1 | 7089.9 -> 7089.9 | 12080.6 -> 12080.6 |
/// | 3 | 7089.9 -> 7089.9 | 12080.6 -> 12080.6 |
/// | 6 | 7089.9 -> **7124.8** | 12080.6 -> **13189.9** |
/// | 10 | 7089.9 -> **7202.0** | 12080.6 -> 11571.1 |
///
/// 조각 하나하나는 잘 들어간다. wsjk156 k=10 에서 앞의 9개는 전부 7066 아래로
/// 들어갔고 **마지막 하나만 7202 로 튀어나왔다**. 즉 실패는 자리가 없어서가 아니라
/// **순서** 때문이다 — 먼저 넣은 조각이 좋은 포켓을 먹고 마지막 조각이 밖으로 밀린다.
///
/// 이건 minimax 문제다(목적이 `max(x_max)` 의 최소화). 그 그리디의 정석은
/// **가장 제약이 심한 것부터**다: 장애물은 넣을수록 늘기만 하므로, 지금 이미
/// 가장 오른쪽까지 밀리는 조각은 나중에 넣으면 반드시 더 나빠진다.
///
/// # 그런데 그 정석이 졌다
///
/// 그렇게 구현해서([`InsertOrder::MostConstrainedFirst`]) 재 보니 벽시계 5초에서
/// wsjk156 72.06% 로, 면적 순서(73.27%)에 **졌다**. 이유는 품질이 아니라 **비용**이다:
/// 매 단계마다 남은 조각 전부를 다시 열거하므로 라운드 하나가 `O(k²/2)` 열거가 되고,
/// 같은 5초에 도는 라운드 수가 4~5배로 줄었다(라운드 69 -> 5~20). 이 패스에서는
/// **라운드 수가 곧 품질**이라 그 손해를 못 메운다.
///
/// 그래서 기본값은 면적 순서다. 제약 순서는 예산이 넉넉한 호출부를 위해 남겨 둔다.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InsertOrder {
    /// 면적 큰 것부터. 열거를 조각당 한 번만 하므로 `O(k)` 로 싸다.
    AreaDesc,
    /// 매 단계에서 **남은 조각 전부**의 최선 `x_max` 를 재 보고, 그 값이 가장 큰
    /// (= 가장 제약이 심한) 조각을 먼저 넣는다. 열거가 `O(k²/2)` 로 늘어난다.
    MostConstrainedFirst,
}

/// 재구성 노브. 기본값은 [`ReflowParams::default`].
///
/// # 기본값을 어떻게 골랐나
///
/// wsjk156, **벽시계 5초**(캐시 포함), 시드 ordering 1개(탐침 4096), 밀도는
/// `SPSolution::density(instance)`. 한 축만 기본값에서 바꾼 값이다.
///
/// ```text
/// 재삽입 탐침   4096 72.22 | 8192 **73.27** | 16384 72.91
/// 뺄 개수 상한    24 72.38 |   64 **73.27**
/// 규칙        LeftBottom 72.87 | MaxContact **73.27**
/// 범위        꼬리만 72.09 | 최저 띠만 71.89 | 둘 다 **73.27**
/// 재삽입 순서  면적 큰 것부터 **73.27** | 제약 심한 것부터 72.06 (그리고 훨씬 느리다)
/// 꼬리 폭      0.25 -> 72.43 | 0.75 **73.27** | 3.0 -> 73.27
/// 평지 이동    끔 **73.27** | 켬 73.27 (다른 인스턴스에서 지므로 끔이 기본)
/// ```
///
/// 범위 세 줄이 이 패스의 핵심이다: 꼬리만(72.09) 도 최저 띠만(71.89) 도 아니고
/// **둘 다**(73.27) 여야 한다. 꼬리만은 갈 곳을 안 만들고, 띠만은 폭을 정하는 조각을
/// 안 건드린다. 그리고 꼬리만/띠만은 일정을 금방 다 훑고 **1.2초에 스스로 멈춘다**.
///
/// 어느 축도 단조가 아니다 — 탐침을 올릴수록 좋아지지도, 많이 뺄수록 좋아지지도
/// 않는다. 그래서 "이 값이 최적" 이 아니라 "이 인스턴스에서 이 값이 이겼다" 로 읽을 것.
#[derive(Clone, Copy, Debug)]
pub struct ReflowParams {
    /// 라운드 상한. 한 라운드가 실패하면(= 폭이 안 줄면) 그 라운드는 원복된다.
    pub rounds: usize,
    /// 재삽입 자리를 고르는 규칙.
    pub rule: SelectionRule,
    /// 구간 폭 = `strip_height * band_width_ratio`.
    ///
    /// strip 높이에 비례시키는 이유: 구간의 "정상" 채움률은 조각 크기 대비 구간
    /// 면적으로 정해지는데, 조각 크기는 strip 높이에 묶여 있다. 절대 mm 로 잡으면
    /// 인스턴스가 바뀔 때마다 다시 튜닝해야 한다.
    pub band_width_ratio: f64,
    /// 꼬리 구간 폭 = `strip_height * tail_ratio`.
    ///
    /// # "무력한 노브" 가 아니다 — 앞선 기록은 틀렸다
    ///
    /// 이 필드에는 "0.5 / 0.75 / 1.5 가 소수점까지 같으니 사실상 죽은 노브" 라고 적혀
    /// 있었다. 절반만 맞다. 다시 재 보면(wsjk156, 벽시계 5초):
    ///
    /// | `tail_ratio` | 꼬리 절단선 | 밀도 |
    /// |---:|---:|---:|
    /// | 0.25 | strip − 381 | 72.43% |
    /// | 0.75 | strip − 1143 | **73.27%** |
    /// | 3.0 | strip − 4572 | 73.27% |
    ///
    /// 0.84%p 차이는 죽은 노브가 아니다. 원리는 이렇다: 꼬리 조각은 `x_max` 내림차순으로
    /// 뽑고 **개수 상한**(`max_remove` 의 절반)이 먼저 걸리므로, 구간이 그 상한을 채울
    /// 만큼 넓기만 하면 뽑히는 조각이 같다. 그래서 **임계값 위에서는 평평하고 아래에서는
    /// 준다**. 0.75 는 이미 그 평평한 구간 안이라 더 키워도 얻을 것이 없고, 줄이면
    /// 꼬리 후보가 상한에 못 미쳐 실제로 손해다.
    ///
    /// 즉 "최소한 이만큼은 꼬리로 본다" 는 **하한**이 맞고, 그 하한이 실제로 구속력을
    /// 가진다. 필드를 없애면 안 된다.
    pub tail_ratio: f64,
    /// 한 라운드에 뺄 조각 수의 **시작값**. [`ReflowParams::max_remove`] 참조.
    pub min_remove: usize,
    /// 한 라운드에 뺄 조각 수 상한.
    ///
    /// # 왜 고정값이 아니라 `min_remove` 에서 자라나
    ///
    /// 고정하면 라운드가 사실상 1회로 끝난다. 같은 배치·같은 규칙·같은 뺄 조각 수면
    /// 다음 라운드는 **같은 조각을 골라 같은 결과**를 내기 때문이다. 실측에서도
    /// 라운드를 4/8/16 로 올려도 채택은 1~2회에서 멈췄다.
    ///
    /// 그래서 실패한 라운드마다 뺄 개수를 두 배로 키우고, 성공하면 다시 시작값으로
    /// 돌아간다. 뺄 개수를 하나 고정해 놓고 훑은 결과가 **단조가 아니었다**는 것도
    /// 같은 이야기다. 아래 세 값은 오래된 하네스(탐침 16384, 꼬리만)에서 잰 것이라
    /// **다른 표와 절대값을 비교하면 안 된다** — 여기서 볼 것은 순서뿐이다.
    ///
    /// | 뺌 | 밀도(상대 비교용) |
    /// |---:|---:|
    /// | 24 | 73.34% |
    /// | 32 | **73.71%** |
    /// | 40 | 73.45% |
    ///
    /// 어느 하나가 옳은 게 아니라 그때그때 다르다. 훑는 편이 낫다.
    pub max_remove: usize,
    /// 어디를 뺄 것인가.
    pub scope: ReflowScope,
    /// 뺀 조각을 다시 넣는 순서. [`InsertOrder`] 문서의 실측표 참조.
    pub order: InsertOrder,
    /// 재삽입 동안만 쓸 탐침 예산. `0`이면 오라클의 현재 값을 그대로 둔다.
    ///
    /// # 왜 여기서만 올리나
    ///
    /// 탐침 예산은 곧 "얼마나 깊은 포켓까지 찾아보나" 다. wsjk156 의 프론티어 조각
    /// item 82(현재 `x_max` 7090)를 혼자 빼서 다시 넣어 보면:
    ///
    /// | 예산 | 후보 수 | 도달 가능한 최선 `x_max` | 시간 |
    /// |---:|---:|---:|---:|
    /// | 4096 | 111 | 4840 | 5ms |
    /// | 16384 | 116 | **4120** | 6ms |
    /// | 65536 | 116 | 4120 | 6ms |
    ///
    /// 예산을 4배로 올리는 데 1ms 밖에 안 드는데 도달 깊이가 720mm 늘어난다. 초기
    /// 배치는 조각 156개에 이 값을 **전부** 곱해야 해서 못 올리지만, 이 패스는 조각
    /// 수십 개만 다시 넣으므로 올릴 수 있다. 예산을 **폭을 정하는 조각에만** 쓰는 것이다.
    ///
    /// 끝나면 원래 값으로 되돌린다 — 같은 오라클을 이어 쓰는 호출부가 있다.
    ///
    /// 예산 262144 에서는 후보가 116 -> 7 로 **줄어든다**. 이건 오라클
    /// (`nfp_lbf::placement`) 쪽 문제이고 이 파일에서 고칠 수 없다. 그래서 여기서는
    /// 그 구간을 쓰지 않는다.
    pub probe_budget: usize,
    /// 폭이 줄지 **않아도** 안쪽이 더 뭉쳤으면 그 라운드를 채택한다.
    ///
    /// # 왜 필요한가 — 라운드가 이어지지 않는다는 문제
    ///
    /// 원래 규칙은 "폭이 줄었을 때만 채택" 이다. 그런데 계측을 켜 보면 wsjk156 5초에서
    /// 라운드 45회 중 채택이 **2회**뿐이고 43회가 원복이다. 폭을 줄이는 라운드는
    /// 대부분 "먼저 안쪽을 비워 두고 그 다음 라운드에서 꼬리를 끌어오는" 2단 동작인데,
    /// 1단계(안쪽 정리)는 폭을 안 줄이므로 통째로 버려진다. 그래서 그 연쇄가 원리적으로
    /// 일어날 수 없다.
    ///
    /// 평지 이동은 그 1단계를 살린다. 채택 조건은 **폭이 늘지 않고**(`after <= strip`)
    /// **퍼짐**(모든 조각 `x_max` 의 합)이 [`MIN_SPREAD_GAIN_MM`] 이상 줄었을 때다.
    /// 퍼짐은 단조 감소만 하므로 라운드가 무한히 왔다 갔다 하지 않는다.
    ///
    /// # 실측 — 해 봤고, **이득이 없어서 기본값이 꺼짐이다**
    ///
    /// 벽시계 5초 전체(캐시 prewarm 포함), 탐침 4096, 시드 10% / reflow 90% 배분,
    /// 밀도는 **인스턴스 실제 도형 면적** 기준:
    ///
    /// | 인스턴스 | reflow 없음 | 평지 끔 | 평지 켬 |
    /// |---|---:|---:|---:|
    /// | wsjk156 | 71.75% | **73.27%** | 73.27% |
    /// | albano | 72.06% | **78.87%** | 78.87% |
    /// | swim | 61.28% | **62.81%** | 62.81% |
    /// | trousers | 76.46% | **77.44%** | 76.46% |
    /// | fu / shirts / dagli | — | 변화 없음 | 변화 없음 |
    ///
    /// 여섯 곳에서 동률이고 trousers 에서만 **0.98%p 진다**. 가설 자체는 맞았다 —
    /// 계측을 켜면 평지 이동이 실제로 채택된다(wsjk156 5초에서 채택 6회 중 1회가 평지).
    /// 그런데 그 1회를 살리려고 재삽입 중단선을 `strip - 0.01` 에서 `strip` 으로 늦추면
    /// 조기 중단이 42회에서 33회로 줄고, **그만큼 도는 라운드가 줄어든다**. 이 패스에서는
    /// 라운드 수가 곧 품질이라 평지 1회로 그 손해를 못 메운다.
    ///
    /// 지우지 않고 남기는 이유: "폭이 안 줄어도 안쪽을 뭉치면 다음 라운드가 이어질
    /// 것" 은 이 패스를 보면 누구나 떠올리는 생각이다. 다시 구현하지 않도록 결과를
    /// 남겨 둔다. 되살리려면 **가지치기를 잃지 않는 방법**을 먼저 찾아야 한다.
    pub plateau_moves: bool,
}

impl Default for ReflowParams {
    fn default() -> Self {
        Self {
            rounds: DEFAULT_REFLOW_ROUNDS,
            rule: SelectionRule::MaxContact,
            band_width_ratio: DEFAULT_BAND_WIDTH_RATIO,
            tail_ratio: DEFAULT_TAIL_RATIO,
            min_remove: DEFAULT_MIN_REMOVE,
            max_remove: DEFAULT_MAX_REMOVE,
            scope: ReflowScope::Both,
            order: InsertOrder::AreaDesc,
            probe_budget: DEFAULT_REFLOW_PROBE_BUDGET,
            // 실측에서 이득이 없었다. 근거는 `plateau_moves` 문서의 표.
            plateau_moves: false,
        }
    }
}

/// 재구성 결과.
#[derive(Clone, Debug, Default)]
pub struct ReflowOutcome {
    /// 재구성 후 배치. 채택된 라운드가 없으면 입력과 같다.
    pub pieces: Vec<PlacedPiece>,
    /// 자리를 못 찾은 아이템 id.
    ///
    /// **정상 동작에서는 항상 빈다.** 뺀 조각을 전부 다시 넣지 못한 라운드는 통째로
    /// 원복하기 때문이다(미배치를 남기는 것은 폭을 줄인 것보다 훨씬 나쁘다).
    /// 비어 있지 않다면 그건 원복 경로가 깨졌다는 신호다.
    pub unplaced: Vec<usize>,
    /// 재구성 후 strip 길이(mm). 정의상 입력의 strip 길이 이하다.
    pub strip_length: f64,
    /// 빼서 다시 넣은 조각 수(채택된 라운드만 센다).
    pub relocated: usize,
    /// 라운드별 strip 길이. `history[0]` 은 **입력** 값이고, 그 뒤로 라운드마다 한 개씩
    /// 붙는다(원복된 라운드는 직전 값이 그대로 다시 붙는다).
    pub history: Vec<f64>,
}

impl ReflowOutcome {
    /// 줄인 strip 길이(mm). 음수가 될 수 없다.
    pub fn gain(&self) -> f64 {
        match self.history.first() {
            Some(first) => (first - self.strip_length).max(0.0),
            None => 0.0,
        }
    }
}

/// 저밀도 구간의 조각을 빼내 남은 자리에 다시 넣는다.
///
/// 탐색이 아니다 — 난수도 겹침 허용도 쓰지 않는다. 항상 feasible 하다.
pub fn reflow(
    ctx: &mut StrategyContext<'_, '_>,
    pieces: &[PlacedPiece],
    rounds: usize,
    rule: SelectionRule,
) -> ReflowOutcome {
    reflow_with(
        ctx,
        pieces,
        ReflowParams {
            rounds,
            rule,
            ..ReflowParams::default()
        },
    )
}

/// [`reflow`] 의 노브 전체를 여는 형태. 스윕 측정이 이걸 쓴다.
pub fn reflow_with(
    ctx: &mut StrategyContext<'_, '_>,
    pieces: &[PlacedPiece],
    params: ReflowParams,
) -> ReflowOutcome {
    let mut current: Vec<PlacedPiece> = pieces.to_vec();
    let (mut strip, mut spread) = frontier_and_spread(ctx, &current);
    let mut history = vec![strip];

    if current.is_empty() || params.rounds == 0 || params.max_remove == 0 {
        return ReflowOutcome {
            pieces: current,
            unplaced: Vec::new(),
            strip_length: strip,
            relocated: 0,
            history,
        };
    }

    let strip_h = f64::from(ctx.placer.strip_height());
    if !strip_h.is_finite() || strip_h <= 0.0 {
        return ReflowOutcome {
            pieces: current,
            unplaced: Vec::new(),
            strip_length: strip,
            relocated: 0,
            history,
        };
    }

    if meas::is_on() {
        meas::CALLS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    }

    // 재삽입 동안만 탐침 예산을 올린다. 아래 라운드 루프는 `break` 로만 빠져나오고
    // 조기 `return` 이 없으므로, 루프 뒤 한 곳에서 복원하면 모든 경로가 덮인다.
    // (여기보다 앞의 조기 반환들은 예산을 아직 바꾸기 전이다.)
    let saved_budget = ctx.placer.probe_budget();
    if params.probe_budget > 0 {
        ctx.placer.set_probe_budget(params.probe_budget);
    }

    let mut relocated = 0usize;
    // 실패한 라운드에서 고른 띠. 같은 띠를 또 고르면 입력이 같으니 결과도 같다 —
    // 순수한 낭비이므로 다음 라운드에서 제외한다.
    let mut exhausted_bands: BTreeSet<usize> = BTreeSet::new();

    // 뺄 개수 일정. 실패할 때마다 다음 칸으로 넘어가고, 성공하면 처음으로 돌아간다.
    let schedule = removal_schedule(params.min_remove, params.max_remove);
    let mut step = 0usize;
    // 띠 폭 배수. 뺄 개수와 띠를 다 훑고도 못 줄이면 **띠 폭 자체**를 바꿔 본다.
    let mut width_idx = 0usize;

    for _ in 0..params.rounds {
        if !ctx.has_budget() {
            break;
        }
        let Some(&take) = schedule.get(step) else {
            break;
        };

        let boxes = ctx.placer.world_bounds(&current);
        let areas = shape_areas(ctx, &current);
        let width_ratio =
            params.band_width_ratio * BAND_WIDTH_CYCLE.get(width_idx).copied().unwrap_or(1.0);
        let profile = BandProfile::build(&boxes, &areas, strip, strip_h, width_ratio);
        let choice = select_victims(
            &boxes,
            &profile,
            strip,
            strip_h,
            &params,
            take,
            &exhausted_bands,
        );

        if choice.victims.is_empty() {
            break;
        }

        let backup = current.clone();
        let removed = extract(&mut current, &choice.victims);
        if meas::is_on() {
            use std::sync::atomic::Ordering as AO;
            meas::ROUNDS.fetch_add(1, AO::Relaxed);
            meas::VICTIMS.fetch_add(removed.len(), AO::Relaxed);
        }

        // 재삽입 도중 프론티어가 이 값을 넘으면 그 라운드는 이미 진 것이다.
        // 프론티어는 조각을 넣을수록 **늘기만** 하므로 되돌릴 방법이 없다.
        //
        // 평지 이동을 허용하면 "폭은 그대로지만 안쪽이 더 뭉친" 라운드도 살려야 하므로
        // 끊는 선이 `strip` 자체가 된다. 가지치기가 그만큼 약해지는 대신 라운드가
        // **이어질** 수 있다 — 어느 쪽이 이기는지는 인스턴스마다 다르다(아래 실측표).
        let limit = if params.plateau_moves {
            strip
        } else {
            strip - MIN_GAIN_MM
        };
        let base_frontier = ctx.frontier(&current);
        let all_back = match params.order {
            InsertOrder::AreaDesc => {
                insert_area_desc(ctx, &mut current, &removed, params.rule, base_frontier, limit)
            }
            InsertOrder::MostConstrainedFirst => insert_most_constrained(
                ctx,
                &mut current,
                &removed,
                params.rule,
                base_frontier,
                limit,
            ),
        };

        let (after, spread_after) = frontier_and_spread(ctx, &current);
        if meas::is_on() && all_back {
            meas::MOVED.fetch_add(
                count_moved(&backup, &current),
                std::sync::atomic::Ordering::Relaxed,
            );
        }
        // 폭이 실제로 줄었나 / 폭은 그대로지만 안쪽이 뭉쳤나.
        let shrank = after <= strip - MIN_GAIN_MM;
        // `after <= strip` 에 여유(epsilon)를 주지 않는다. 조금이라도 봐주면 그 오차가
        // 라운드마다 누적되어 이 패스가 폭을 **늘릴** 수 있고, 그러면 "출력 strip 은
        // 입력 이하" 라는 유일한 안전 성질이 깨진다. f32 잡음으로 같은 배치가 1e-7mm
        // 커 보여 평지 이동 하나를 놓치는 쪽이 훨씬 싸다.
        let plateau = params.plateau_moves
            && after <= strip
            && spread_after <= spread - MIN_SPREAD_GAIN_MM;
        if !all_back || !(shrank || plateau) {
            if all_back {
                meas::note_attempt(after - strip);
            }
            if meas::is_on() {
                meas::REVERTS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            }
            current = backup;
            history.push(strip);

            // 다음 라운드가 **다른** 입력을 보게 만든다. 그러지 않으면 같은 조각을
            // 골라 같은 결과를 내므로 라운드를 더 도는 것이 순수한 낭비다.
            step += 1;
            if step >= schedule.len() {
                step = 0;
                match (params.scope, choice.band) {
                    // 뺄 개수를 다 훑었으면 다른 띠로 넘어간다.
                    (ReflowScope::Both | ReflowScope::LowBandOnly, Some(b)) => {
                        exhausted_bands.insert(b);
                    }
                    // 띠를 다 훑었으면 띠 폭을 바꿔 **다른 조각 묶음**을 만든다.
                    // 꼬리만 보는 모드에는 띠가 없으므로 여기서 끝난다.
                    _ => {
                        width_idx += 1;
                        exhausted_bands.clear();
                        if params.scope == ReflowScope::TailOnly
                            || width_idx >= BAND_WIDTH_CYCLE.len()
                        {
                            break;
                        }
                    }
                }
            }
            continue;
        }

        if meas::is_on() {
            use std::sync::atomic::Ordering as AO;
            meas::ACCEPTS.fetch_add(1, AO::Relaxed);
            if !shrank {
                meas::PLATEAUS.fetch_add(1, AO::Relaxed);
            }
        }
        relocated += removed.len();
        strip = after;
        spread = spread_after;
        history.push(strip);
        // 폭이 바뀌면 띠 경계가 통째로 이동한다. 이전 라운드의 "막다른 띠" 판정도,
        // 뺄 개수 일정의 진행도 더 이상 같은 상황을 가리키지 않으므로 되감는다.
        exhausted_bands.clear();
        step = 0;
        width_idx = 0;
    }

    // 예산을 되돌린다 — 같은 오라클을 이어 쓰는 호출부가 이 값을 물려받으면
    // 다음 ordering 이 조용히 다른 정밀도로 돌아 재현이 깨진다.
    ctx.placer.set_probe_budget(saved_budget);

    ReflowOutcome {
        pieces: current,
        unplaced: Vec::new(),
        strip_length: strip,
        relocated,
        history,
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 구간(band) 채움률
// ────────────────────────────────────────────────────────────────────────────

/// x축 구간별 채움률과 조각의 소속 구간.
///
/// 순수 계산만 한다(오라클을 만지지 않는다) — 그래야 단위 테스트가 가능하다.
#[derive(Clone, Debug, Default)]
pub struct BandProfile {
    /// 구간 폭(mm).
    pub width: f64,
    /// 구간별 채움률 `0..`. `조각 면적 / 구간 면적` 이라 1을 넘지 않는 것이 정상이다.
    pub fill: Vec<f64>,
    /// 조각 인덱스 -> 가장 많이 걸친 구간 인덱스.
    pub owner: Vec<usize>,
}

impl BandProfile {
    /// `boxes`/`areas` 는 조각 인덱스가 1:1 이어야 한다.
    ///
    /// 조각 면적은 bbox 가 아니라 **실제 도형 면적**을 쓴다. bbox 충전율이 62.6% 인
    /// 인스턴스에서 bbox 로 재면 모든 구간이 일제히 부풀어 순서가 흐려진다.
    /// 구간에 걸친 조각은 bbox 의 x 겹침 비율로 면적을 나눠 준다 — 근사지만
    /// 필요한 것은 "어느 구간이 제일 비었나" 라는 **순서**뿐이다.
    pub fn build(
        boxes: &[Aabb],
        areas: &[f64],
        strip: f64,
        strip_height: f64,
        band_width_ratio: f64,
    ) -> Self {
        let width = (strip_height * band_width_ratio).max(1.0);
        if !strip.is_finite() || strip <= 0.0 || !width.is_finite() {
            return Self::default();
        }
        let n = ((strip / width).ceil() as usize).max(1);
        let mut filled = vec![0.0_f64; n];
        let mut owner = vec![0_usize; boxes.len()];

        for (i, b) in boxes.iter().enumerate() {
            let area = areas.get(i).copied().unwrap_or(0.0);
            let bw = (b.max_x - b.min_x).max(0.0);
            let mut best = (0.0_f64, 0_usize);
            for band in 0..n {
                let lo = width * band as f64;
                let hi = (lo + width).min(strip);
                let overlap = (b.max_x.min(hi) - b.min_x.max(lo)).max(0.0);
                if overlap <= 0.0 {
                    continue;
                }
                if overlap > best.0 {
                    best = (overlap, band);
                }
                if let Some(slot) = filled.get_mut(band) {
                    // 폭이 0 인 조각(퇴화 도형)은 겹침 비율이 0/0 이다. 통째로 그
                    // 구간에 넣는다 — 어차피 면적도 0 에 가깝다.
                    *slot += if bw > 0.0 { area * overlap / bw } else { area };
                }
            }
            if let Some(slot) = owner.get_mut(i) {
                *slot = best.1;
            }
        }

        let fill = (0..n)
            .map(|band| {
                let lo = width * band as f64;
                let hi = (lo + width).min(strip);
                let cap = (hi - lo).max(0.0) * strip_height;
                if cap > 0.0 {
                    filled.get(band).copied().unwrap_or(0.0) / cap
                } else {
                    // 폭 0 인 마지막 구간은 "꽉 찼다" 로 둔다. 0 으로 두면 항상
                    // 최저 구간으로 뽑혀 라운드가 전부 헛돈다.
                    1.0
                }
            })
            .collect();

        Self { width, fill, owner }
    }

    /// 채움률이 가장 낮은 구간. `skip` 에 든 구간과 조각이 없는 구간은 건너뛴다.
    pub fn lowest(&self, skip: &BTreeSet<usize>) -> Option<usize> {
        let mut best: Option<(f64, usize)> = None;
        for (band, ratio) in self.fill.iter().enumerate() {
            if skip.contains(&band) {
                continue;
            }
            if !self.owner.iter().any(|o| *o == band) {
                continue;
            }
            let r = if ratio.is_finite() { *ratio } else { f64::MAX };
            match best {
                Some((cur, _)) if cur <= r => {}
                _ => best = Some((r, band)),
            }
        }
        best.map(|(_, b)| b)
    }
}

/// 이번 라운드에 뺄 조각들과, 그 근거가 된 띠.
#[derive(Clone, Debug, Default)]
struct VictimChoice {
    /// `pieces` 인덱스. 오름차순 중복 없음.
    victims: Vec<usize>,
    /// 채움률 최저 띠(골랐다면).
    band: Option<usize>,
}

/// 꼬리 + 채움률 최저 띠에서 뺄 조각을 고른다.
///
/// 상한(`max_remove`)을 넘으면 **꼬리를 우선**한다. 폭을 정하는 것은 꼬리이므로
/// 꼬리를 못 빼면 그 라운드는 원리적으로 폭을 줄일 수 없다. 다만 `Both` 모드에서
/// 꼬리가 상한을 전부 먹으면 띠가 비지 않아 꼬리 조각이 갈 곳이 없으므로,
/// 꼬리 몫을 절반으로 제한한다.
fn select_victims(
    boxes: &[Aabb],
    profile: &BandProfile,
    strip: f64,
    strip_height: f64,
    params: &ReflowParams,
    take: usize,
    exhausted: &BTreeSet<usize>,
) -> VictimChoice {
    let mut out: BTreeSet<usize> = BTreeSet::new();

    let (tail_cap, band_cap) = match params.scope {
        ReflowScope::TailOnly => (take, 0),
        ReflowScope::LowBandOnly => (0, take),
        ReflowScope::Both => {
            let tail = take.div_ceil(2);
            (tail, take.saturating_sub(tail))
        }
    };

    // ── 꼬리: 오른쪽 끝에서 `tail_ratio * strip_height` 안에 걸친 조각 ──
    if tail_cap > 0 {
        let cut = strip - (strip_height * params.tail_ratio).max(0.0);
        let mut tail: Vec<(f64, usize)> = boxes
            .iter()
            .enumerate()
            .filter(|(_, b)| b.max_x > cut)
            .map(|(i, b)| (b.max_x, i))
            .collect();
        // 가장 오른쪽부터. 폭에 대한 기여가 큰 순서다.
        tail.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
        for (_, i) in tail.into_iter().take(tail_cap) {
            out.insert(i);
        }
    }

    // ── 채움률 최저 띠 ──
    let mut chosen_band = None;
    if band_cap > 0 {
        if let Some(band) = profile.lowest(exhausted) {
            chosen_band = Some(band);
            let mut owned: Vec<(f64, usize)> = profile
                .owner
                .iter()
                .enumerate()
                .filter(|(_, o)| **o == band)
                // **bbox 면적**으로 뺄 조각을 고른다. 정확한 도형 면적이 아니다 — 의도적이다.
                //
                // `BandProfile::build` 는 정확한 면적을 쓰고(`bbox_area_would_lie_...`
                // 테스트가 그것을 고정한다), 여기는 bbox 를 쓴다. 불일치처럼 보여
                // 정확 면적으로 바꿔 재 봤더니 **나빠졌다**: ⑨ 단독 73.53% -> 73.14%
                // (wsjk156, 2026-08-31).
                //
                // 두 자리가 묻는 질문이 다르기 때문이다. 프로파일은 "이 띠가 얼마나
                // 채워졌나" 를 묻고, 그건 실제 면적의 질문이다. 여기는 "이 조각을 빼면
                // strip 폭이 얼마나 열리나" 를 묻는다 — **점유 폭**의 질문이고, 넓게
                // 퍼진 조각이 답이다. bbox 면적이 그 점유를 더 잘 나타낸다.
                .filter_map(|(i, _)| boxes.get(i).map(|b| (b.area(), i)))
                .collect();
            // 큰 조각부터. 작은 조각을 빼 봐야 생기는 자리가 아무것도 못 받는다.
            owned.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
            for (_, i) in owned.into_iter() {
                if out.len() >= tail_cap + band_cap {
                    break;
                }
                out.insert(i);
            }
        }
    }

    VictimChoice {
        victims: out.into_iter().collect(),
        band: chosen_band,
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 배치 조작
// ────────────────────────────────────────────────────────────────────────────

/// 뺄 개수 일정 — `min` 에서 두 배씩 키워 `max` 까지. 중복 없이 오름차순.
///
/// 두 배씩인 이유: 실패한 라운드는 통째로 버려지므로, 실패 횟수가 곧 낭비다.
/// 선형으로 늘리면 `max` 에 닿기까지 실패를 너무 많이 산다.
fn removal_schedule(min: usize, max: usize) -> Vec<usize> {
    let max = max.max(1);
    let mut k = min.max(1).min(max);
    let mut out = Vec::new();
    loop {
        out.push(k);
        if k >= max {
            break;
        }
        k = k.saturating_mul(2).min(max);
    }
    out
}

/// `victims`(오름차순 인덱스)를 배치에서 빼내 돌려준다.
///
/// 뒤에서부터 지운다 — 앞에서 지우면 뒤쪽 인덱스가 전부 밀려 엉뚱한 조각이 빠진다.
fn extract(pieces: &mut Vec<PlacedPiece>, victims: &[usize]) -> Vec<PlacedPiece> {
    let mut out = Vec::with_capacity(victims.len());
    for idx in victims.iter().rev() {
        if *idx < pieces.len() {
            out.push(pieces.remove(*idx));
        }
    }
    out
}

/// 조각 하나를 **원래 회전 그대로** 놓을 수 있는 모든 자리.
///
/// 회전을 바꾸지 않는 이유: 이 인스턴스는 회전이 `[0]` 하나뿐이고, 결(grain) 제약이
/// 걸린 조각의 자세를 바꾸면 그건 배치가 아니라 불량이다.
fn options_for_piece(
    ctx: &mut StrategyContext<'_, '_>,
    placed: &[PlacedPiece],
    piece: PlacedPiece,
) -> Vec<AnchorOption> {
    let Some((bounds, _)) = ctx.placer.shape_bounds_area(piece.item_id, piece.rotation) else {
        return Vec::new();
    };
    let frontier = ctx.frontier(placed);

    ctx.placer.set_probe_frontier(frontier);
    // ④ 설정과 무관하게 **항상** 포켓 우선이다. 이 패스의 목적 자체가 "프론티어를
    // 늘리지 않는 자리" 를 찾는 것이고, 기본 탐침 순서는 안쪽에 예산의 1/N 만 쓴다.
    ctx.placer.set_pocket_first(true);
    let candidates = ctx.placer.candidates_for(piece.item_id, piece.rotation, placed);
    ctx.placer.set_pocket_first(false);

    candidates
        .into_iter()
        .map(|c| {
            let (x, y) = (f64::from(c.translation.0), f64::from(c.translation.1));
            AnchorOption {
                item_id: piece.item_id,
                rotation: piece.rotation,
                x,
                y,
                world: bounds.translated(x, y),
                touches: c.touch_count,
            }
        })
        .collect()
}

/// 선택지 중 하나를 규칙으로 고른다.
///
/// `frontier` 를 그대로 넘기면 [`AnchorMetrics::strip_length`] 가
/// `max(frontier, x_max)` 가 되고, 1순위가 그 값의 최소다. 즉 **프론티어를 늘리지
/// 않는 자리가 전부 동률로 1순위**가 되고 그 안에서 규칙이 결정한다 — 명세의
/// "프론티어를 늘리지 않는 자리를 우선" 이 별도 필터 없이 그대로 나온다.
fn choose(
    ctx: &StrategyContext<'_, '_>,
    placed: &[PlacedPiece],
    options: &[AnchorOption],
    rule: SelectionRule,
) -> Option<AnchorOption> {
    if options.is_empty() {
        return None;
    }
    let frontier = ctx.frontier(placed);
    let boxes = ctx.placer.world_bounds(placed);
    let metrics: Vec<AnchorMetrics> = options
        .iter()
        .map(|o| ctx.metrics(o, &boxes, frontier))
        .collect();
    let best = pick_best(options, &metrics, rule)?;
    options.get(best).copied()
}

/// 재삽입 실패를 계측에 남기고 `false` 를 돌려준다.
fn note_fail() -> bool {
    if meas::is_on() {
        meas::REINSERT_FAIL.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    }
    false
}

/// 이미 진 라운드를 조기에 끊는다.
///
/// # 왜 이게 큰 이득인가
///
/// 프론티어는 조각을 넣을수록 **늘기만** 한다. 그러니 재삽입 도중에 이미 원래
/// strip 길이를 넘어섰다면 남은 조각을 다 넣어 봐야 결과는 정해져 있다 — 원복이다.
/// 실패한 라운드가 전체의 8할이고 라운드 하나가 조각 수십 개의 후보 열거이므로,
/// 여기서 끊는 만큼 그대로 라운드 수로 돌아온다.
fn abort_hopeless() -> bool {
    if meas::is_on() {
        meas::EARLY_ABORTS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    }
    false
}

/// 면적 큰 것부터 다시 넣는다. 열거는 조각당 한 번뿐이다.
fn insert_area_desc(
    ctx: &mut StrategyContext<'_, '_>,
    current: &mut Vec<PlacedPiece>,
    removed: &[PlacedPiece],
    rule: SelectionRule,
    base_frontier: f64,
    limit: f64,
) -> bool {
    // 빼고 남은 것만으로 이미 한계를 넘었다면 무엇을 넣어도 못 이긴다 — 열거 전에 끊는다.
    if base_frontier > limit {
        return abort_hopeless();
    }

    let mut order: Vec<(f64, PlacedPiece)> =
        removed.iter().map(|p| (piece_area(ctx, p), *p)).collect();
    order.sort_by(|a, b| b.0.total_cmp(&a.0));

    let mut running = base_frontier;
    for (_, piece) in order {
        if !ctx.has_budget() {
            return false;
        }
        let options = options_for_piece(ctx, current, piece);
        match choose(ctx, current, &options, rule) {
            Some(option) => {
                running = running.max(option.world.max_x);
                current.push(option.to_piece());
                if running > limit {
                    return abort_hopeless();
                }
            }
            None => return note_fail(),
        }
    }
    true
}

/// 매 단계에서 남은 조각 전부를 다시 열거해, **최선의 `x_max` 가 가장 큰** 조각을
/// 먼저 넣는다.
///
/// # 왜 이 순서가 맞나
///
/// 목적은 `max(x_max)` 의 최소화(minimax)다. 재삽입이 진행될수록 장애물은 **늘기만**
/// 하므로 한 조각의 최선 `x_max` 는 절대 줄지 않는다. 그러면 "지금 이미 가장 오른쪽까지
/// 밀리는 조각" 을 뒤로 미루는 것은 순손해다 — 그 조각이 결국 폭을 정하기 때문이다.
///
/// 비용은 열거 `k(k+1)/2` 회다. wsjk156 에서 열거 1회가 약 2.3ms 이므로 `k=10` 이면
/// 약 130ms — 5초 예산 안에서 충분히 감당된다.
fn insert_most_constrained(
    ctx: &mut StrategyContext<'_, '_>,
    current: &mut Vec<PlacedPiece>,
    removed: &[PlacedPiece],
    rule: SelectionRule,
    base_frontier: f64,
    limit: f64,
) -> bool {
    // 빼고 남은 것만으로 이미 한계를 넘었다면 무엇을 넣어도 못 이긴다 — 열거 전에 끊는다.
    if base_frontier > limit {
        return abort_hopeless();
    }

    let mut pending: Vec<PlacedPiece> = removed.to_vec();
    let mut running = base_frontier;

    while !pending.is_empty() {
        if !ctx.has_budget() {
            return false;
        }

        // (최선 x_max, pending 인덱스, 선택지)
        let mut evaluated: Vec<(f64, usize, Vec<AnchorOption>)> = Vec::with_capacity(pending.len());
        for (i, piece) in pending.clone().into_iter().enumerate() {
            let options = options_for_piece(ctx, current, piece);
            if options.is_empty() {
                return note_fail();
            }
            let best_max_x = options
                .iter()
                .map(|o| o.world.max_x)
                .fold(f64::INFINITY, f64::min);
            evaluated.push((best_max_x, i, options));
        }

        // 가장 제약이 심한 조각. 동률이면 더 작은 인덱스가 이긴다(결정론).
        let mut winner: Option<usize> = None;
        for (slot, (best_max_x, _, _)) in evaluated.iter().enumerate() {
            let better = match winner.and_then(|w| evaluated.get(w)) {
                None => true,
                Some((cur, _, _)) => *best_max_x > *cur,
            };
            if better {
                winner = Some(slot);
            }
        }
        let Some(slot) = winner else {
            return false;
        };
        let Some((_, idx, options)) = evaluated.get(slot) else {
            return false;
        };
        if pending.get(*idx).is_none() {
            return false;
        }
        match choose(ctx, current, options, rule) {
            Some(option) => {
                running = running.max(option.world.max_x);
                current.push(option.to_piece());
            }
            None => return note_fail(),
        }
        if *idx < pending.len() {
            pending.remove(*idx);
        }
        if running > limit {
            return abort_hopeless();
        }
    }
    true
}

/// `after` 중 `before` 에 같은 자세·같은 자리가 없는 조각 수 (계측 전용).
///
/// 재삽입은 조각 순서를 바꾸므로 인덱스로 비교할 수 없다. 수요가 2 이상인 아이템이
/// 있으므로 `item_id` 로도 갈리지 않아, **자리까지 같은 짝이 남아 있는지**를 본다.
/// `O(n²)` 이지만 계측을 켰을 때만 돈다.
fn count_moved(before: &[PlacedPiece], after: &[PlacedPiece]) -> usize {
    /// 같은 자리로 볼 허용 오차(mm). 앵커가 `f32` 라 재계산만으로도 1e-4 쯤 흔들린다.
    const SAME_SPOT_MM: f32 = 1.0e-3;

    let mut used = vec![false; before.len()];
    let mut moved = 0usize;
    for a in after {
        let mut matched = false;
        for (i, b) in before.iter().enumerate() {
            if used.get(i).copied().unwrap_or(true) {
                continue;
            }
            if b.item_id == a.item_id
                && b.rotation == a.rotation
                && (b.translation.0 - a.translation.0).abs() <= SAME_SPOT_MM
                && (b.translation.1 - a.translation.1).abs() <= SAME_SPOT_MM
            {
                if let Some(slot) = used.get_mut(i) {
                    *slot = true;
                }
                matched = true;
                break;
            }
        }
        if !matched {
            moved += 1;
        }
    }
    moved
}

/// `(프론티어, 퍼짐)` 을 `world_bounds` **한 번**으로 함께 잰다.
///
/// 프론티어는 `max(x_max)`(= strip 길이), 퍼짐은 `sum(x_max)` 다. 퍼짐은 평지 이동의
/// 판정 기준이며, 안쪽 조각 하나가 왼쪽으로 갈 때만 줄고 폭에는 영향을 주지 않는다.
///
/// 둘을 따로 재면 `world_bounds`(조각당 bbox 변환)가 라운드마다 두 번 돈다. 라운드가
/// 곧 품질인 패스라 그 정도도 아깝다.
///
/// `NaN` 좌표는 프론티어 비교에서 조용히 빠지지만 합에는 `NaN` 으로 번져 퍼짐 비교가
/// 전부 거짓이 된다 — 그러면 평지 이동이 채택되지 않을 뿐이라 안전한 쪽으로 실패한다.
fn frontier_and_spread(ctx: &StrategyContext<'_, '_>, pieces: &[PlacedPiece]) -> (f64, f64) {
    let mut frontier = 0.0_f64;
    let mut spread = 0.0_f64;
    for b in ctx.placer.world_bounds(pieces) {
        if b.max_x > frontier {
            frontier = b.max_x;
        }
        spread += b.max_x;
    }
    (frontier, spread)
}

/// 배치된 조각들의 **실제 도형 면적**(mm²), 인덱스 1:1.
fn shape_areas(ctx: &StrategyContext<'_, '_>, pieces: &[PlacedPiece]) -> Vec<f64> {
    pieces.iter().map(|p| piece_area(ctx, p)).collect()
}

/// 조각 하나의 실제 도형 면적(mm²). 못 구하면 `0.0` — panic 하지 않는다.
fn piece_area(ctx: &StrategyContext<'_, '_>, piece: &PlacedPiece) -> f64 {
    ctx.placer
        .shape_bounds_area(piece.item_id, piece.rotation)
        .map(|(_, area)| area)
        .unwrap_or(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bbox(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Aabb {
        Aabb {
            min_x,
            min_y,
            max_x,
            max_y,
        }
    }

    #[test]
    fn an_empty_band_reads_as_the_lowest_fill() {
        // strip 300, 높이 100, 구간 폭 100 -> 구간 3개.
        // 0~100 은 꽉, 100~200 은 비었고, 200~300 은 절반.
        let boxes = [bbox(0.0, 0.0, 100.0, 100.0), bbox(200.0, 0.0, 300.0, 50.0)];
        let areas = [10_000.0, 5_000.0];
        let p = BandProfile::build(&boxes, &areas, 300.0, 100.0, 1.0);

        assert_eq!(p.fill.len(), 3);
        assert!((p.fill[0] - 1.0).abs() < 1e-9, "{:?}", p.fill);
        assert_eq!(p.fill[1], 0.0);
        assert!((p.fill[2] - 0.5).abs() < 1e-9, "{:?}", p.fill);
        // 빈 구간은 소유 조각이 없으므로 후보에서 빠진다 — 뺄 게 없는 구간을
        // 고르면 라운드가 통째로 헛돈다.
        assert_eq!(p.lowest(&BTreeSet::new()), Some(2));
    }

    #[test]
    fn a_piece_straddling_two_bands_splits_its_area_by_x_overlap() {
        // 조각 하나가 두 구간에 절반씩 걸친다.
        let boxes = [bbox(50.0, 0.0, 150.0, 100.0)];
        let areas = [10_000.0];
        let p = BandProfile::build(&boxes, &areas, 200.0, 100.0, 1.0);
        assert!((p.fill[0] - 0.5).abs() < 1e-9, "{:?}", p.fill);
        assert!((p.fill[1] - 0.5).abs() < 1e-9, "{:?}", p.fill);
        // 겹침이 동률이면 앞 구간이 주인이다(결정론).
        assert_eq!(p.owner[0], 0);
    }

    #[test]
    fn bbox_area_would_lie_so_the_profile_uses_the_real_shape_area() {
        // bbox 는 꽉 찼지만 실제 도형은 그 60% 뿐인 오목 조각. 이 인스턴스의
        // bbox 충전율이 62.6% 라 이 차이가 곧 순위 차이다.
        let boxes = [bbox(0.0, 0.0, 100.0, 100.0)];
        let p = BandProfile::build(&boxes, &[6_000.0], 100.0, 100.0, 1.0);
        assert!((p.fill[0] - 0.6).abs() < 1e-9, "{:?}", p.fill);
    }

    #[test]
    fn the_tail_scope_takes_the_rightmost_pieces_up_to_the_cap() {
        let boxes = [
            bbox(0.0, 0.0, 100.0, 100.0),
            bbox(700.0, 0.0, 800.0, 100.0),
            bbox(850.0, 0.0, 900.0, 100.0),
            bbox(900.0, 0.0, 1000.0, 100.0),
        ];
        let areas = [10_000.0; 4];
        let strip = 1000.0;
        let profile = BandProfile::build(&boxes, &areas, strip, 100.0, 1.0);
        let params = ReflowParams {
            scope: ReflowScope::TailOnly,
            min_remove: 2,
            max_remove: 2,
            tail_ratio: 2.0, // 꼬리 폭 200 -> cut = 800
            ..ReflowParams::default()
        };
        let choice = select_victims(&boxes, &profile, strip, 100.0, &params, params.max_remove, &BTreeSet::new());
        // max_x 가 1000, 900 인 두 개. 800 짜리는 cut 과 같아 걸리지 않는다.
        assert_eq!(choice.victims, vec![2, 3]);
        assert_eq!(choice.band, None);
    }

    #[test]
    fn both_scope_reserves_half_the_budget_for_the_low_band() {
        // 꼬리에 조각이 넘쳐도 띠 몫을 다 먹으면 안 된다 — 꼬리 조각이 갈 곳이
        // 없어져 라운드가 항상 실패한다.
        let mut boxes: Vec<Aabb> = (0..6)
            .map(|i| {
                let x = 800.0 + i as f64 * 30.0;
                bbox(x, 0.0, x + 30.0, 100.0)
            })
            .collect();
        // 저밀도 띠(0~100)에 작은 조각 하나.
        boxes.insert(0, bbox(0.0, 0.0, 20.0, 20.0));
        let areas: Vec<f64> = boxes.iter().map(|b| b.area()).collect();
        let strip = 1000.0;
        let profile = BandProfile::build(&boxes, &areas, strip, 100.0, 1.0);
        let params = ReflowParams {
            scope: ReflowScope::Both,
            min_remove: 4,
            max_remove: 4,
            tail_ratio: 2.0,
            ..ReflowParams::default()
        };
        let choice = select_victims(&boxes, &profile, strip, 100.0, &params, params.max_remove, &BTreeSet::new());
        assert!(choice.victims.len() <= 4);
        assert!(
            choice.victims.contains(&0),
            "저밀도 띠의 조각이 빠져야 한다: {:?}",
            choice.victims
        );
        assert_eq!(choice.band, Some(0));
    }

    #[test]
    fn an_exhausted_band_is_skipped_next_round() {
        let boxes = [bbox(0.0, 0.0, 20.0, 20.0), bbox(100.0, 0.0, 150.0, 100.0)];
        let areas: Vec<f64> = boxes.iter().map(|b| b.area()).collect();
        let profile = BandProfile::build(&boxes, &areas, 200.0, 100.0, 1.0);
        assert_eq!(profile.lowest(&BTreeSet::new()), Some(0));
        let mut skip = BTreeSet::new();
        skip.insert(0usize);
        assert_eq!(profile.lowest(&skip), Some(1));
        skip.insert(1usize);
        assert_eq!(profile.lowest(&skip), None);
    }

    #[test]
    fn extraction_removes_exactly_the_chosen_pieces() {
        let mk = |id: usize| PlacedPiece {
            item_id: id,
            rotation: 0,
            translation: (id as f32, 0.0),
        };
        let mut pieces: Vec<PlacedPiece> = (0..5).map(mk).collect();
        let removed = extract(&mut pieces, &[1, 3]);
        assert_eq!(pieces.len(), 3);
        assert_eq!(
            pieces.iter().map(|p| p.item_id).collect::<Vec<_>>(),
            vec![0, 2, 4]
        );
        let mut ids: Vec<usize> = removed.iter().map(|p| p.item_id).collect();
        ids.sort_unstable();
        assert_eq!(ids, vec![1, 3]);
    }

    #[test]
    fn extraction_ignores_out_of_range_indices_instead_of_panicking() {
        let mut pieces = vec![PlacedPiece {
            item_id: 0,
            rotation: 0,
            translation: (0.0, 0.0),
        }];
        let removed = extract(&mut pieces, &[0, 7]);
        assert_eq!(removed.len(), 1);
        assert!(pieces.is_empty());
    }

    #[test]
    fn a_degenerate_strip_produces_no_bands_instead_of_dividing_by_zero() {
        let p = BandProfile::build(&[bbox(0.0, 0.0, 1.0, 1.0)], &[1.0], 0.0, 100.0, 1.0);
        assert!(p.fill.is_empty());
        assert_eq!(p.lowest(&BTreeSet::new()), None);
    }

    #[test]
    fn the_removal_schedule_doubles_and_stops_at_the_cap() {
        assert_eq!(removal_schedule(4, 64), vec![4, 8, 16, 32, 64]);
        // 상한이 두 배 경계에 안 맞아도 마지막은 정확히 상한이다.
        assert_eq!(removal_schedule(8, 20), vec![8, 16, 20]);
        // 시작값이 상한보다 크면 상한 하나뿐 — 빈 일정이 나오면 라운드가 통째로 죽는다.
        assert_eq!(removal_schedule(50, 10), vec![10]);
        // 0 은 무한 루프가 되기 쉬운 입력이다.
        assert_eq!(removal_schedule(0, 0), vec![1]);
        assert_eq!(removal_schedule(0, 4), vec![1, 2, 4]);
    }

    #[test]
    fn moved_counting_matches_pieces_by_pose_not_by_index() {
        let at = |id: usize, x: f32, y: f32| PlacedPiece {
            item_id: id,
            rotation: 0,
            translation: (x, y),
        };
        // 순서만 바뀐 배치는 "안 움직인 것" 이다 — 재삽입이 조각 순서를 바꾸므로
        // 인덱스로 비교하면 전부 움직인 것으로 잘못 센다.
        let before = vec![at(1, 0.0, 0.0), at(2, 5.0, 0.0)];
        let shuffled = vec![at(2, 5.0, 0.0), at(1, 0.0, 0.0)];
        assert_eq!(count_moved(&before, &shuffled), 0);

        // 수요가 2 이상인 아이템은 `item_id` 로 갈리지 않으므로 자리까지 봐야 한다.
        let twins_before = vec![at(7, 0.0, 0.0), at(7, 9.0, 0.0)];
        let twins_after = vec![at(7, 0.0, 0.0), at(7, 9.5, 0.0)];
        assert_eq!(count_moved(&twins_before, &twins_after), 1);
    }

    #[test]
    fn plateau_moves_stay_off_by_default() {
        // 켜면 trousers 에서 0.98%p 지고 나머지는 동률이었다(`plateau_moves` 문서의 표).
        // 이 단언은 "재 보지 않고 켜는 것" 을 막는 자물쇠다 — 켜려면 표부터 갱신할 것.
        assert!(!ReflowParams::default().plateau_moves);
    }

    #[test]
    fn gain_is_never_negative() {
        let out = ReflowOutcome {
            pieces: Vec::new(),
            unplaced: Vec::new(),
            strip_length: 120.0,
            relocated: 0,
            history: vec![100.0, 120.0],
        };
        assert_eq!(out.gain(), 0.0);
    }
}
