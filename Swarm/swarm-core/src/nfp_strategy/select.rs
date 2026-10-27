//! ③ 접촉 최대화 후보 선택 규칙과 ⑥ best-fit 조각 선택.
//!
//! # ③ 왜 기존 규칙을 바꿔야 하나 — 사전식 우선순위가 붕괴한다
//!
//! 기존 규칙은 `strip 길이 -> x 최소 -> y 최소 -> 접촉 최대` 다. 그런데 **한 조각·한
//! 회전** 안에서 폭 `W` 는 상수이므로
//!
//! ```text
//! strip_length(후보) = max(frontier, x + W)
//! ```
//!
//! 는 `x` 의 **단조 증가(비감소) 함수**다. 즉 `x_a < x_b` 이면 `strip(a) <= strip(b)`
//! 라서 1순위가 2순위를 **절대 뒤집지 못한다**. 사전식 `(strip, x, y)` 는 전순서로
//! `(x, y)` 와 같아지고, 4순위 접촉은 도달 불가능한 죽은 코드가 된다.
//!
//! 여러 회전을 한 풀에 섞으면 `W` 가 달라지므로 완전한 붕괴는 아니지만, 같은 회전
//! 안에서는 여전히 `argmin(x)` 다. 실제 후보의 대부분이 같은 회전에서 나오므로
//! 결과적으로 좌하단 규칙과 구분되지 않는다.
//!
//! [`SelectionRule::MaxContact`] 는 2순위와 4순위를 맞바꾼다(Gomes & Oliveira).
//! `frontier` 가 이미 `x + W` 보다 큰 구간 — 즉 프론티어 **안쪽** — 에서는 여러 후보의
//! strip 길이가 진짜로 동률이 되고, 그때 비로소 접촉이 결정권을 갖는다.
//!
//! # ③이 이 인스턴스에서 지는 이유 — 규칙이 아니라 **지표**가 눈이 멀었다
//!
//! wsjk156(탐침 4096, 유효 ordering 12개)에서 ③은 기준선 71.75% -> 71.07% 로 진다.
//! [`meas`] 카운터로 그 안을 들여다본 결과다(호출 1,872회 / 후보 73,325개):
//!
//! | 지표 | 값 |
//! |---|---:|
//! | 접촉이 **정확히 0**인 후보 | 56,063 / 73,325 = **76.5%** |
//! | strip 1순위가 동률이라 접촉이 결정권을 갖는 후보 | 14,182 = 19.3% |
//! | 그 동률 그룹이 2개 이상이었던 호출 | 887 = 47.4% |
//! | **고른 후보의 접촉이 0** 이었던 호출 | 1,111 = **59.3%** |
//! | 두 규칙이 서로 다른 후보를 고른 호출 | 437 = 23.3% |
//!
//! 읽는 법:
//!
//! * 접촉은 bbox 근사([`crate::nfp_strategy`]의 `contact_length`)라 오목한 의류 패널이
//!   실제로 맞물려도 bbox 가 0.5mm 밴드 안에 없으면 **0**이 된다. 후보 넷 중 셋이 그렇다.
//! * 그래서 절반 넘는 호출(59.3%)에서 **접촉 최대값 자체가 0**이다. 그 호출에서 ③은
//!   접촉 동률로 내려가 `x -> y`, 즉 좌하단 규칙과 완전히 같아진다.
//! * 실제로 갈리는 23.3%가 승부를 정하는데, 그 23.3%의 신호가 잡음이다. 허용오차를
//!   바꿔도([`contact_eps_for`]), 접촉 동률을 낭비로 한 번 더 갈라도(71.10%), 접촉이
//!   x 를 뒤집을 수 있는 x 밴드를 20/100/400mm 로 둬도(71.45/70.75/71.16%) 전부
//!   기준선 아래에서 **추세 없이** 흔들린다.
//!
//! 탐침을 8192로 올리면 부호가 뒤집힌다: 기준 71.94% -> ③ 72.06%. 접촉 위치를 충분히
//! 열거해야 신호가 생긴다는 [`crate::nfp_strategy::NfpStrategyConfig::contact_maximization`]
//! 의 서술 그대로다. **규칙 쪽에 고칠 결함은 없다.** 고쳐야 할 것은 접촉 지표(bbox 근사)
//! 이거나, ③을 탐침 예산에 따라 켜는 판단이며 둘 다 이 파일 바깥에 있다.
//!
//! # 허용오차를 왜 mm 실제 값에 거나
//!
//! [`AnchorMetrics`] 는 정규화하지 않은 mm 값이다. 축마다 나누는 기준을 다르게 잡으면
//! 같은 1mm 차이가 축에 따라 동률이 되기도 하고 아니기도 한다. 그래서 길이·좌표는
//! 절대 허용오차, 접촉은 크기가 조각마다 자릿수로 다르므로 **상대** 허용오차를 쓴다.
//!
//! # 결정론
//!
//! 모든 비교가 동률이면 **더 작은 인덱스**가 이긴다. 배치 결과가 실행마다 흔들리면
//! 회귀 측정 자체가 불가능해지기 때문이다.

use std::cmp::Ordering;
use std::collections::HashSet;

use crate::nfp_lbf::PlacedPiece;

use super::{AnchorMetrics, AnchorOption, StrategyContext};

/// ③ 선택 규칙이 실제로 무엇을 보고 있는지 세는 계측 카운터.
///
/// 켜지 않으면 [`pick_best`] 호출마다 relaxed load 한 번이다(호출 1,872회/팔 —
/// 후보 열거의 NFP 탐침 옆에서는 계측에 잡히지 않는다). 켜는 것은 `#[ignore]`
/// 계측 테스트뿐이고, 켜면 호출마다 후보를 한 번 더 훑는다.
///
/// 이 카운터가 답한 것은 모듈 문서의 "③이 이 인스턴스에서 지는 이유" 표에 있다.
#[doc(hidden)]
pub mod meas {
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering as AO};

    pub static ON: AtomicBool = AtomicBool::new(false);
    /// `pick_best` 호출 수 = 조각을 한 번 놓으려 한 횟수.
    pub static CALLS: AtomicUsize = AtomicUsize::new(0);
    /// 본 후보 총수.
    pub static CANDS: AtomicUsize = AtomicUsize::new(0);
    /// 접촉이 0 인 후보 수.
    pub static ZERO_CONTACT: AtomicUsize = AtomicUsize::new(0);
    /// strip 1순위가 최소값과 동률인 후보 수의 합 (= 접촉이 결정권을 갖는 후보 수).
    pub static STRIP_TIE: AtomicUsize = AtomicUsize::new(0);
    /// strip 동률 그룹이 2개 이상이었던 호출 수.
    pub static TIE_CALLS: AtomicUsize = AtomicUsize::new(0);
    /// 고른 후보의 접촉이 0 이었던 횟수.
    pub static CHOSEN_ZERO: AtomicUsize = AtomicUsize::new(0);
    /// 고른 후보가 strip 바닥(min_y <= 0.5mm)에 붙어 있던 횟수.
    pub static CHOSEN_FLOOR: AtomicUsize = AtomicUsize::new(0);
    /// 두 규칙이 서로 다른 후보를 골랐던 호출 수.
    pub static DIVERGED: AtomicUsize = AtomicUsize::new(0);

    pub fn set(on: bool) {
        ON.store(on, AO::Relaxed);
    }
    pub fn is_on() -> bool {
        ON.load(AO::Relaxed)
    }
    pub fn reset() {
        for c in [
            &CALLS,
            &CANDS,
            &ZERO_CONTACT,
            &STRIP_TIE,
            &TIE_CALLS,
            &CHOSEN_ZERO,
            &CHOSEN_FLOOR,
            &DIVERGED,
        ] {
            c.store(0, AO::Relaxed);
        }
    }
    pub fn get(c: &AtomicUsize) -> usize {
        c.load(AO::Relaxed)
    }
    pub fn report() -> String {
        let calls = get(&CALLS).max(1);
        let cands = get(&CANDS).max(1);
        format!(
            "호출 {} / 후보 {} (평균 {:.1}개)\n\
             접촉 0 후보 {} ({:.1}%)\n\
             strip 동률 후보 {} ({:.1}%) — 동률 그룹>1 호출 {} ({:.1}%)\n\
             고른 후보가 접촉 0 {} ({:.1}%) / 바닥 밀착 {} ({:.1}%)\n\
             두 규칙이 갈린 호출 {} ({:.1}%)",
            get(&CALLS),
            get(&CANDS),
            get(&CANDS) as f64 / calls as f64,
            get(&ZERO_CONTACT),
            100.0 * get(&ZERO_CONTACT) as f64 / cands as f64,
            get(&STRIP_TIE),
            100.0 * get(&STRIP_TIE) as f64 / cands as f64,
            get(&TIE_CALLS),
            100.0 * get(&TIE_CALLS) as f64 / calls as f64,
            get(&CHOSEN_ZERO),
            100.0 * get(&CHOSEN_ZERO) as f64 / calls as f64,
            get(&CHOSEN_FLOOR),
            100.0 * get(&CHOSEN_FLOOR) as f64 / calls as f64,
            get(&DIVERGED),
            100.0 * get(&DIVERGED) as f64 / calls as f64,
        )
    }
}

/// 두 선택지의 접촉을 같다고 볼 절대 허용오차(mm).
///
/// 접촉은 조각 크기에 비례해 자릿수가 통째로 달라지므로 `max(1, |a|, |b|)` 로 스케일을
/// 잡는다. [`CONTACT_REL_EPS`] 문서 참조.
///
/// # 왜 "조각 둘레 대비" 로 바꾸지 않았나 — 재 보니 차이가 없다
///
/// 접촉값이 bbox 근사라 오차가 float 잡음이 아니라 mm 단위이므로, 허용오차를 후보
/// bbox 둘레의 비율로 잡는 편이 원리상 맞아 보인다. wsjk156·탐침 4096·유효 ordering
/// 12개에서 그 비율 `k` 를 훑은 결과다(밀도, 기준선 71.75%):
///
/// | 방식 | 밀도 |
/// |---|---:|
/// | 기존(접촉값 상대 1e-3) | 71.07% |
/// | 둘레 대비 k=0.002 | 71.07% |
/// | 둘레 대비 k=0.01  | 71.12% |
/// | 둘레 대비 k=0.05  | 70.91% |
///
/// 폭이 0.2% 안에서 흔들릴 뿐 추세가 없다. 허용오차는 ③이 지는 원인이 **아니다**.
/// 그래서 기존 방식을 그대로 둔다 — 이득 없는 복잡도를 사지 않는다.
///
/// **호출부는 [`worst_if_not_finite_max`] 로 정규화한 값을 넘겨야 한다.** 날값을 넘기면
/// `+inf` 하나가 `eps == inf` 를 만들어 접촉 축이 전부 동률로 무너진다.
fn contact_eps_for(a: f64, b: f64) -> f64 {
    CONTACT_REL_EPS * a.abs().max(b.abs()).max(1.0)
}

/// [`meas`] 가 켜져 있을 때 `pick_best` 한 번의 후보 분포를 센다.
fn meas_record(
    options: &[AnchorOption],
    metrics: &[AnchorMetrics],
    rule: SelectionRule,
    best: Option<usize>,
) {
    use std::sync::atomic::Ordering as AO;
    let n = options.len().min(metrics.len());
    meas::CALLS.fetch_add(1, AO::Relaxed);
    meas::CANDS.fetch_add(n, AO::Relaxed);

    let mut min_strip = f64::MAX;
    for m in metrics.iter().take(n) {
        if m.contact <= 0.0 {
            meas::ZERO_CONTACT.fetch_add(1, AO::Relaxed);
        }
        if m.strip_length < min_strip {
            min_strip = m.strip_length;
        }
    }
    let tie = metrics
        .iter()
        .take(n)
        .filter(|m| (m.strip_length - min_strip).abs() <= STRIP_LENGTH_EPS)
        .count();
    meas::STRIP_TIE.fetch_add(tie, AO::Relaxed);
    if tie > 1 {
        meas::TIE_CALLS.fetch_add(1, AO::Relaxed);
    }
    if let Some(i) = best {
        if metrics.get(i).is_some_and(|m| m.contact <= 0.0) {
            meas::CHOSEN_ZERO.fetch_add(1, AO::Relaxed);
        }
        if options.get(i).is_some_and(|o| o.world.min_y <= 0.5) {
            meas::CHOSEN_FLOOR.fetch_add(1, AO::Relaxed);
        }
    }
    // 반대 규칙이면 다른 후보를 골랐을까 — ③이 실제로 무언가 바꾸고 있는지의 척도다.
    let other = match rule {
        SelectionRule::LeftBottom => SelectionRule::MaxContact,
        SelectionRule::MaxContact => SelectionRule::LeftBottom,
    };
    let mut alt: Option<usize> = None;
    for i in 0..n {
        let Some(cand) = metrics.get(i) else { continue };
        match alt {
            None => alt = Some(i),
            Some(b) => {
                let Some(cur) = metrics.get(b) else { continue };
                if cmp_by_rule(other, cand, cur) == Ordering::Less {
                    alt = Some(i);
                }
            }
        }
    }
    if alt != best {
        meas::DIVERGED.fetch_add(1, AO::Relaxed);
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 허용오차
// ────────────────────────────────────────────────────────────────────────────

/// 두 strip 길이를 같다고 볼 절대 허용오차(mm).
///
/// 후보 좌표는 `f32` 로 왕복하므로 mm 단위에서 1e-4 아래 차이는 반올림 잡음이다.
pub const STRIP_LENGTH_EPS: f64 = 1e-4;

/// 두 좌표(x, y)를 같다고 볼 절대 허용오차(mm). [`STRIP_LENGTH_EPS`] 와 같은 근거다.
pub const COORD_EPS: f64 = 1e-4;

/// 두 접촉 길이를 같다고 볼 **상대** 허용오차.
///
/// 접촉은 조각 크기에 비례해 자릿수가 통째로 달라진다(작은 조각 수 mm, 큰 패널 수백 mm).
/// 절대 허용오차를 쓰면 큰 조각에서는 사실상 항상 다르다고 판정되고 작은 조각에서는
/// 사실상 항상 동률이 된다. 그래서 `max(1, |a|, |b|)` 로 스케일을 잡는다.
pub const CONTACT_REL_EPS: f64 = 1e-3;

/// 프론티어를 "전혀 밀지 않았다" 고 볼 절대 허용오차(mm).
///
/// [`STRIP_LENGTH_EPS`] 와 같은 근거(f32 왕복 잡음)이지만 쓰는 곳이 다르다 — 이쪽은
/// ⑥ best-fit 의 조기 종료 판정이라 이름을 따로 둔다.
pub const GROWTH_FREE_EPS: f64 = 1e-4;

/// 두 [`FitCost::ratio`] 를 같다고 볼 절대 허용오차.
///
/// `ratio` 는 무차원(소비한 strip 면적 / 조각 면적)이고 실측 범위가 0~수 정도라
/// 상대 오차를 쓸 이유가 없다. 값이 이보다 가까우면 면적으로 내려간다.
const FIT_RATIO_EPS: f64 = 1.0e-9;

/// ⑥ best-fit 이 조각을 바꿔치기할 수 있는 **면적 티어의 폭**.
///
/// `0.0` 이면 매 단계 남은 것 중 가장 큰 조각만 후보다(= 면적 내림차순 고정 순서),
/// `1.0` 이면 남은 조각 전부가 후보다(= 제한 없는 best-fit).
/// 실제로는 "가장 큰 배치 가능 조각의 면적 x (1 - BAND)" 이상인 조각만 겨룬다.
///
/// # 왜 티어가 필요한가 — 제한 없는 best-fit 은 이 인스턴스에서 진다
///
/// 유효 ordering 을 1개로 고정해(= first-fit 과 완전히 같은 조건) wsjk156 을 재보면
/// **티어 없이는**(`BAND = 1.00`) 창을 넓힐수록 **단조롭게 나빠진다**:
///
/// | 팔 | 폭 | 밀도 | 시간 |
/// |---|---:|---:|---:|
/// | first-fit (면적 내림차순 1개) | 7112.90 | 71.52% | 2.9s |
/// | best-fit lookahead 4   | 7493.80 | 67.88% | 4.2s |
/// | best-fit lookahead 12  | 8096.52 | 62.83% | 11.8s |
/// | best-fit lookahead 32  | 8000.04 | 63.59% | 18.1s |
/// | best-fit lookahead 200(전수) | 8019.95 | 63.43% | 34.7s |
///
/// **읽을 때 주의** — 이 표는 `BAND = 1.00`, 즉 티어를 끈 조건이다. 티어가 들어간
/// 지금 코드로 `measure_best_fit_lookahead` 를 돌리면 당연히 다른 숫자가 나온다
/// (아래 "티어를 넣은 뒤" 표). 두 조건을 한 번에 볼 수 있도록 그 계측 테스트가
/// `BAND` 별로 창을 훑게 해 두었으므로, 위 표는 그 출력의 `BAND 1.00` 행이다.
///
/// # 티어를 넣은 뒤 — 함정이 사라졌다
///
/// 같은 조건(ordering 1개, wsjk156)에서 `BAND = 0.20` 으로 다시 훑은 결과다.
/// 이제 창을 넓혀도 first-fit 아래로 떨어지지 않는다:
///
/// | 팔 | 폭 | 밀도 | 시간 |
/// |---|---:|---:|---:|
/// | first-fit (면적 내림차순 1개) | 7112.90 | 71.52% | 1.1s |
/// | best-fit lookahead 4   | 7107.42 | 71.57% | 2.1s |
/// | best-fit lookahead 12  | 7077.75 | 71.87% | 3.1s |
/// | best-fit lookahead 32  | 7080.21 | 71.85% | 3.7s |
/// | best-fit lookahead 200(전수) | 7080.21 | 71.85% | 3.7s |
///
/// `lookahead` 는 12 부근에서 이미 포화한다 — 32 이상은 시간만 더 쓰고 답이 같다.
///
/// 원인은 **한계비용 그리디의 구조적 함정**이다. 모든 조각은 언젠가 반드시 놓인다.
/// 매 단계 "지금 가장 싼 조각" 을 고르면 비싼 조각이 계속 뒤로 밀리는데, 뒤로 갈수록
/// 맞물릴 이웃이 줄어 **미룬 비용이 더 비싸진다**. 그래서 단계별 최적이 총합 최적이
/// 아니고, 미룬 조각들이 마지막에 몰려 각자 새 띠를 연다. `lookahead` 를 키울수록
/// 미룰 자유가 커지므로 더 나빠진다 — 위 표가 정확히 그 모양이다.
///
/// 그래서 고전적 해법 그대로 **큰 조각 우선을 1순위로 되돌리고, best-fit 은 그 티어
/// 안에서만** 쓴다.
///
/// # 티어 폭을 0.20 으로 고른 근거
///
/// 6개 인스턴스에서 first-fit(= 면적 내림차순 고정 순서) **대비 밀도 차이**(%p).
/// 전부 유효 ordering 1개·탐침 4096, `lookahead = 12` (`measure_best_fit_area_band`):
///
/// | BAND | albano | shirts | trousers | swim | dagli | wsjk156 | 평균 |
/// |---:|---:|---:|---:|---:|---:|---:|---:|
/// | 0.05 | +0.00 | +0.00 | +0.00 | +1.56 | +0.00 | -1.36 | +0.03 |
/// | 0.10 | +5.89 | +0.00 | +0.00 | +1.66 | -0.65 | +0.19 | +1.18 |
/// | 0.15 | +4.96 | +0.00 | +0.00 | -0.61 | -1.35 | -1.14 | +0.31 |
/// | **0.20** | **+4.96** | **+0.00** | **+0.00** | **+3.45** | **-1.35** | **+0.35** | **+1.24** |
/// | 0.30 | +1.25 | -0.39 | +0.00 | +2.38 | -1.35 | -0.98 | +0.15 |
/// | 1.00 | +1.12 | -6.95 | -5.76 | -5.75 | -12.38 | -8.69 | -6.40 |
///
/// 읽는 법:
///
/// * **`1.00`(제한 없는 best-fit)은 6개 중 5개에서 크게 진다.** 위의 함정이 특정
///   인스턴스의 사고가 아니라는 근거다.
/// * 티어가 좁으면(`0.05`) 대부분 first-fit 과 **완전히 같은 답**이 된다(±0.00). 즉
///   이 노브는 "면적 내림차순에서 얼마나 벗어나도 되는가" 를 그대로 재는 값이고,
///   안전한 하한이 명확하다.
/// * `0.10`~`0.30` 구간의 곡선은 매끄럽지 않다(그리디 구성법이 흔히 그렇다).
///   `0.20` 은 평균이 가장 좋고 목표 인스턴스(wsjk156)와 swim 에서 최고다. 대신
///   dagli 에서 -1.35%p 를 잃는다 — 무료 점심은 아니다.
///
/// # 이 6개는 `0.20` 을 **고른** 집합이다 — 그래서 따로 검증했다
///
/// 위 표는 6개 인스턴스 x 6개 폭에서 평균 최대를 집은 것이라, 그 6개에서 잘 나오는
/// 것은 당연하고 근거가 되지 못한다. 그래서 **폭 선택에 전혀 쓰지 않은** 8개에서
/// `BAND = 0.20` 을 다시 쟀다(같은 조건: 유효 ordering 1개, 탐침 4096,
/// `lookahead = 12`). first-fit 대비 밀도차(%p):
///
/// | 인스턴스 | BAND 0.20 | BAND 1.00 |
/// |---|---:|---:|
/// | fu       | +0.03 | +7.17 |
/// | jakobs1  | +0.00 | -6.61 |
/// | jakobs2  | **-2.08** | -4.03 |
/// | mao      | +8.81 | -2.18 |
/// | marques  | +3.52 | -6.26 |
/// | shapes0  | +0.00 | -1.54 |
/// | shapes1  | +0.00 | -1.54 |
/// | blaz1    | +0.03 | -2.62 |
/// | **평균** | **+1.29** | **-2.20** |
///
/// 튜닝 집합의 평균 `+1.24` 와 홀드아웃 평균 `+1.29` 가 사실상 같다 — `0.20` 은 6개에
/// 맞춰 깎은 값이 아니다. 지는 인스턴스도 여전히 있다(jakobs2 -2.08, dagli -1.35).
///
/// 제한 없는 best-fit 은 홀드아웃 8개 중 7개에서도 진다. 튜닝 집합 5/6 과 합쳐
/// **14개 중 12개** — 이 함정은 인스턴스 사고가 아니다.
///
/// 재현: `BF_INSTANCES="fu.json,jakobs1.json,jakobs2.json,mao.json,marques.json,\
/// shapes0.json,shapes1.json,blaz1.json" BF_BANDS="0.20,1.00"` 로
/// `measure_best_fit_area_band`.
///
/// # 시간
///
/// 티어 밖은 오라클을 부르지 않으므로 단계당 평가 수가 `lookahead`(12)에서 2~3 으로
/// 떨어진다. wsjk156, 유효 ordering 1개 기준으로 `BAND 1.00` 10.5s -> `BAND 0.20`
/// 3.6s 다(= 위 두 표의 `lookahead 12` 행, 11.8s -> 3.1s 와 같은 이야기).
pub const BEST_FIT_AREA_BAND: f64 = 0.20;

// ────────────────────────────────────────────────────────────────────────────
// ③ 후보 선택 규칙
// ────────────────────────────────────────────────────────────────────────────

/// 후보 선택 규칙.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SelectionRule {
    /// strip 길이 -> x 최소 -> y 최소 -> 접촉 최대 (기존 규칙).
    ///
    /// 같은 회전 안에서는 사실상 `argmin(x)` 로 붕괴한다. 모듈 문서 참조.
    LeftBottom,
    /// strip 길이 -> 접촉 최대 -> x 최소 -> y 최소 (Gomes & Oliveira).
    MaxContact,
}

impl Default for SelectionRule {
    /// 기존 경로를 한 비트도 바꾸지 않는 쪽이 기본이다.
    fn default() -> Self {
        SelectionRule::LeftBottom
    }
}

impl SelectionRule {
    /// `NfpStrategyConfig::contact_maximization` 플래그를 규칙으로 옮긴다.
    ///
    /// 호출부가 `if cfg.contact_maximization { .. } else { .. }` 를 여기저기 반복하지
    /// 않도록 한 곳에 모아 둔다.
    pub fn from_contact_maximization(on: bool) -> Self {
        if on {
            SelectionRule::MaxContact
        } else {
            SelectionRule::LeftBottom
        }
    }
}

/// 두 선택지를 규칙에 따라 비교한다. [`Ordering::Less`] 면 `a` 가 **더 좋다**.
///
/// 빔 서치(⑤)처럼 자기 풀을 따로 관리하는 전략도 같은 비교를 써야 규칙이 한 곳에서만
/// 정의된다. 그래서 공개한다.
///
/// 비유한 값(NaN/±inf)은 panic 대신 "가장 나쁨" 으로 밀어낸다 — 후보 하나가 망가졌다고
/// 배치 전체를 죽이는 것보다 그 후보를 지는 쪽에 두는 편이 낫다.
pub fn cmp_by_rule(rule: SelectionRule, a: &AnchorMetrics, b: &AnchorMetrics) -> Ordering {
    // 1순위는 두 규칙 공통 — strip 길이.
    let strip = cmp_smaller_is_better(
        worst_if_not_finite_min(a.strip_length),
        worst_if_not_finite_min(b.strip_length),
        STRIP_LENGTH_EPS,
    );
    if strip != Ordering::Equal {
        return strip;
    }

    let x = || {
        cmp_smaller_is_better(
            worst_if_not_finite_min(a.x),
            worst_if_not_finite_min(b.x),
            COORD_EPS,
        )
    };
    let y = || {
        cmp_smaller_is_better(
            worst_if_not_finite_min(a.y),
            worst_if_not_finite_min(b.y),
            COORD_EPS,
        )
    };
    let contact = || {
        // 허용오차는 **정규화한 뒤** 값으로 잰다. 날값으로 재면 `contact == +inf` 인
        // 후보에서 `eps == inf` 가 되어 `|차이| <= eps` 가 항상 참이 되고, 접촉 축이
        // 통째로 "전부 동률" 로 무너진다 — "비유한 값은 가장 나쁨" 이라는 이 함수의
        // 계약과 정반대다. 정규화한 `f64::MIN` 으로 재면 eps 가 유한하고, 차이(inf)가
        // eps 를 넘어 원래대로 "가장 나쁨" 이 된다.
        let (a_c, b_c) = (
            worst_if_not_finite_max(a.contact),
            worst_if_not_finite_max(b.contact),
        );
        // 인자를 뒤집어 "큰 쪽이 Less(=더 좋음)" 로 만든다.
        cmp_smaller_is_better(b_c, a_c, contact_eps_for(a_c, b_c))
    };

    // **정확한** 접촉 조각 수. 오라클이 feasible 판정에서 obstacle 마다
    // `Location::Boundary` 를 세면서 이미 구한 값이라 공짜이고 근사가 아니다.
    //
    // # 왜 접촉 **길이** 뒤에 오는가
    //
    // 처음에는 "정확한 값이니 앞에 두자" 로 1순위에 놓았다. **나빠졌다**
    // (wsjk156, ordering 12, 탐침 4096, `SPSolution::density`, 2026-08-31):
    //
    // ```text
    //   기준선(전부 끔)                 71.75%   폭 7089.96
    //   ③ 접촉 최대화 — 수를 앞에       70.37%   폭 7228.60   <- 기준선보다 나쁘다
    //   ③ 접촉 최대화 — 길이를 앞에     72.00%   폭 7065.60   <- 문서 값 재현
    // ```
    //
    // 이유: 접촉 조각 **수**는 0,1,2,3 같은 거친 정수라 후보 다수가 한 계단에 뭉친다.
    // 게다가 "몇 조각과 닿았나" 와 "얼마나 길게 닿았나" 는 다른 목적이다 — 작은 조각
    // 셋에 살짝 닿는 배치가 큰 조각 하나에 길게 맞물리는 배치를 이겨 버린다.
    //
    // 정확값이라고 해서 더 앞에 두어야 하는 것은 아니다. **무엇을 재는 값인지**가
    // 순위를 정한다. 여기서는 길이가 목적이고, 수는 길이가 동률일 때 가르는 신호다.
    let touches = || b.touches.cmp(&a.touches);

    match rule {
        SelectionRule::LeftBottom => x().then_with(y).then_with(contact).then_with(touches),
        SelectionRule::MaxContact => contact().then_with(touches).then_with(x).then_with(y),
    }
}

/// 규칙에 따라 가장 좋은 선택지의 인덱스.
///
/// 완전 동률이면 **더 작은 인덱스**를 돌려준다 — 뒤 후보로 갱신할 때
/// [`Ordering::Less`] (엄격히 더 좋음)만 통과시키기 때문이다.
///
/// `options` 와 `metrics` 는 인덱스가 1:1 이어야 한다. 길이가 어긋나면 **짧은 쪽까지만**
/// 본다 — 호출부 실수로 인덱스가 밖으로 나가 panic 하는 것보다 조용히 덜 보는 편이 낫다
/// (덜 본 결과는 여전히 유효한 배치다).
pub fn pick_best(
    options: &[AnchorOption],
    metrics: &[AnchorMetrics],
    rule: SelectionRule,
) -> Option<usize> {
    let n = options.len().min(metrics.len());
    let mut best: Option<usize> = None;
    for i in 0..n {
        let Some(cand) = metrics.get(i) else { continue };
        match best {
            None => best = Some(i),
            Some(b) => {
                let Some(cur) = metrics.get(b) else { continue };
                if cmp_by_rule(rule, cand, cur) == Ordering::Less {
                    best = Some(i);
                }
            }
        }
    }
    if meas::is_on() {
        meas_record(options, metrics, rule, best);
    }
    best
}

/// 규칙에 따라 좋은 순서로 상위 `k` 개의 인덱스 (빔 서치가 쓴다).
///
/// * 중복 없이 정확히 `min(k, 후보수)` 개.
/// * 앞자리일수록 좋다. 첫 원소는 [`pick_best`] 와 항상 일치한다.
/// * 동률은 더 작은 인덱스가 앞선다.
///
/// # 왜 `sort_by` 가 아니라 선택 정렬인가
///
/// 허용오차 비교는 **추이적이 아니다**: `a≈b`, `b≈c` 인데 `a<c` 일 수 있다. 그런 비교
/// 함수를 `sort_by` 에 넘기면 std 가 "comparison function does not implement a total
/// order" 로 **panic** 한다. `k` 는 빔 폭(기본 4) 정도라 `O(k·n)` 이 문제 되지 않으므로
/// 선택 정렬로 그 위험을 통째로 없앴다.
pub fn pick_top_k(
    options: &[AnchorOption],
    metrics: &[AnchorMetrics],
    rule: SelectionRule,
    k: usize,
) -> Vec<usize> {
    let n = options.len().min(metrics.len());
    let want = k.min(n);
    let mut chosen: Vec<usize> = Vec::with_capacity(want);
    let mut taken = vec![false; n];

    for _ in 0..want {
        let mut best: Option<usize> = None;
        for i in 0..n {
            if taken.get(i).copied().unwrap_or(true) {
                continue;
            }
            let Some(cand) = metrics.get(i) else { continue };
            match best {
                None => best = Some(i),
                Some(b) => {
                    let Some(cur) = metrics.get(b) else { continue };
                    if cmp_by_rule(rule, cand, cur) == Ordering::Less {
                        best = Some(i);
                    }
                }
            }
        }
        match best {
            Some(i) => {
                if let Some(slot) = taken.get_mut(i) {
                    *slot = true;
                }
                chosen.push(i);
            }
            // 남은 후보가 없다 — `want <= n` 이므로 정상 경로에서는 도달하지 않는다.
            None => break,
        }
    }
    chosen
}

// ────────────────────────────────────────────────────────────────────────────
// ⑥ best-fit 조각 선택
// ────────────────────────────────────────────────────────────────────────────

/// 이번 단계에 놓기로 정한 조각과 그 자리.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BestFitPick {
    /// 놓을 조각·자세·좌표.
    pub option: AnchorOption,
    /// 그 자리의 평가 지표.
    pub metrics: AnchorMetrics,
    /// `remaining` 안에서의 인덱스 — 호출부가 수요를 깎는 데 쓴다.
    pub remaining_index: usize,
}

/// 조각 하나를 "지금 놓을 값어치" 로 환산한 값.
///
/// # 왜 strip 길이 증가량만으로는 안 되나 — 작은 조각이 무조건 이긴다
///
/// 원래 구현은 [`cmp_by_rule`] 을 조각 비교에 그대로 썼다. 1순위가 `strip_length =
/// max(frontier, max_x)` 이므로 "프론티어를 가장 덜 미는 조각" 이 이긴다. 그런데
/// **프론티어를 미는 양은 조각 크기에 비례**한다. 작은 조각은 언제나 덜 밀고, 프론티어
/// 안쪽 빈틈에도 더 자주 들어가므로 `strip_length` 동률 뒤의 `x 최소` 까지 가져간다.
/// 결과는 매 단계 "가장 작은 조각" 이 이기는 것이고, 큰 조각은 끝까지 밀려 마지막에
/// 오른쪽에 통째로 쌓인다.
///
/// wsjk156(156종, 수요 각 1) 실측이 정확히 그 모양이었다:
///
/// | | 폭 | 밀도 |
/// |---|---:|---:|
/// | 기준(전략 전부 끔) | 7112.90 | 71.52% |
/// | ⑥ 옛 기준(증가량만) | 8629.86 | 58.95% |
///
/// `lookahead = 12` 가 이 붕괴를 더 키웠다. 창이 "면적 상위 12종" 이므로 매 단계
/// **12종 중 가장 작은 것**이 뽑히고, 가장 큰 조각은 12번째보다 항상 져서 사실상
/// 영원히 미뤄진다. ordering 을 6개로 돌리든 12개로 돌리든 폭이 8629.86 으로 **똑같이**
/// 나온 것이 그 증거다 — 조각 순서를 전혀 쓰지 않는 결정론적 경로 하나가 반복됐다.
///
/// # 대신 쓰는 값 — 소비한 strip 면적 / 조각 면적
///
/// ```text
/// growth = max(0, strip_length_after - frontier)      // 프론티어를 민 거리(mm)
/// ratio  = growth * strip_height / area               // 무차원
/// ```
///
/// 분자는 이 배치가 **새로 사 들인 strip 면적**(mm²)이고 분모는 그 대가로 담은 조각
/// 면적이다. 즉 `ratio` 는 "이 조각을 지금 놓으면 원단을 얼마나 비효율적으로 쓰는가"
/// 이고, `0` 은 프론티어 안쪽 빈틈을 공짜로 메웠다는 뜻, `1` 은 새로 연 띠를 조각이
/// 정확히 꽉 채웠다는 뜻이다. 크기로 정규화됐으므로 큰 조각이 작은 조각에게 크다는
/// 이유만으로 지지 않는다.
///
/// 동률이면 **면적이 큰 조각**이 이긴다. 고전적 해법(큰 조각 우선 + 그 안에서 best-fit)
/// 의 "큰 조각 우선" 이 여기 들어간다. 프론티어 안쪽에 여러 조각이 공짜로 들어가는
/// 국면(=`ratio` 가 다 0)에서 순서를 정하는 것은 사실상 이 규칙이다.
#[derive(Clone, Copy, Debug, PartialEq)]
struct FitCost {
    /// 소비한 strip 면적 / 조각 면적. 작을수록 좋다. 계산 불가면 [`f64::MAX`].
    ratio: f64,
    /// 조각의 실제 면적(mm²). 동률 해소용 — 클수록 좋다.
    area: f64,
    /// 프론티어를 민 거리(mm). 조기 종료 판정에만 쓴다.
    growth: f64,
}

impl FitCost {
    /// `frontier` 는 이번 단계 모든 조각에게 같은 값이어야 한다.
    fn new(metrics: &AnchorMetrics, frontier: f64, area: f64, strip_height: f64) -> Self {
        let growth = (metrics.strip_length - frontier).max(0.0);
        // 면적·높이가 비정상이면 나눗셈이 ±inf/NaN 이 된다. panic 대신 "가장 나쁨" 으로
        // 밀어내고 나머지 후보로 진행한다 — 조각 하나 때문에 배치를 죽이지 않는다.
        let ratio = if area > 0.0 && strip_height.is_finite() && growth.is_finite() {
            growth * strip_height / area
        } else {
            f64::MAX
        };
        Self {
            ratio: worst_if_not_finite_min(ratio),
            area: if area.is_finite() { area } else { 0.0 },
            growth: if growth.is_finite() { growth } else { f64::MAX },
        }
    }

    /// 프론티어를 사실상 전혀 밀지 않았는가 (= 공짜 배치).
    ///
    /// # 이 판정으로 뒤를 끊는 것은 **ε-최적**이지 완전 최적이 아니다
    ///
    /// `growth` 가 정확히 0 이 아니라 `0 < growth <= GROWTH_FREE_EPS` 면 `ratio` 도
    /// 정확히 0 이 아니라 `growth x strip_height / area` 만큼 남는다. 뒤에 `growth`
    /// 가 진짜 0 인 후보가 있으면 그쪽이 `ratio` 로는 **엄밀히 더 좋다**. 즉 조기
    /// 종료가 최적해를 하나 건너뛸 수 있다.
    ///
    /// 그래도 끊는 이유는 건너뛰는 양이 유계이고 무의미하기 때문이다. 놓치는 `ratio`
    /// 차이는 `GROWTH_FREE_EPS(1e-4mm) x strip_height / area` 이하이고, wsjk156
    /// (strip 높이 ~1.5m, 패널 면적 ~1e4mm² 이상)에서 그 상한은 약 `1.5e-5` 다.
    /// 겨루는 후보들의 실제 `ratio` 는 O(1) 이라 자릿수가 다섯 개 넘게 차이 난다.
    /// 게다가 건너뛰는 쪽은 면적 내림차순에서 뒤 — 즉 **더 작은** 조각이므로,
    /// 놓치더라도 이 파일이 채택한 "큰 조각 우선" 과 같은 방향이다.
    fn is_free(&self) -> bool {
        self.growth <= GROWTH_FREE_EPS
    }
}

/// 남은 조각을 평가해 **지금 놓기 가장 좋은 조각**을 고른다.
///
/// # 판단 기준
///
/// 1순위는 `ratio = (프론티어를 민 거리 x strip 높이) / 조각 면적` — 이 배치가 새로
/// 사 들인 strip 면적을 그 대가로 담은 조각 면적으로 나눈 무차원 값이다. 동률이면
/// 면적이 큰 조각, 그다음이 `rule` 이 정한 자리 비교([`cmp_by_rule`]), 마지막이
/// **더 작은 `remaining` 인덱스**(결정론)다.
///
/// 왜 "strip 길이 증가량 최소" 를 버렸는지 — 증가량은 조각 크기에 비례하므로 그 기준은
/// 매 단계 **가장 작은 조각**을 뽑고 큰 조각을 끝까지 미룬다. 실측 근거는 이 파일의
/// 비공개 타입 `FitCost` 문서에 적어 두었다(wsjk156: 71.52% -> 58.95%).
///
/// # 왜 순서를 미리 정하지 않나
///
/// 고정 ordering 은 앞에서 한 번 잘못 고르면 끝까지 되돌리지 못한다. best-fit 은 매
/// 단계 현재 프론티어 모양을 보고 고르므로, 방금 생긴 구멍에 맞는 조각이 있으면 그것을
/// 먼저 집는다. 대가는 비용이다 — 한 단계가 `평가 조각 수` 배로 늘어난다.
///
/// # 비용을 깎는 네 장치
///
/// * `remaining` 에 같은 `item_id` 가 여러 번 나오면 **한 번만** 평가한다. 같은 조각은
///   같은 후보 집합을 만들므로 결과가 반드시 같다.
/// * `lookahead` 로 평가할 **종류 수**를 자른다. 면적 내림차순 상위만 본다 — 큰 조각이
///   자리를 잃으면 되돌릴 수 없고, 작은 조각은 나중에도 들어갈 자리가 있다.
///   `0` 이면 자르지 않고 전수 평가한다.
/// * **공짜 배치를 만나면 즉시 끝낸다.** 후보를 면적 내림차순으로 보므로, `growth ≈ 0`
///   인 첫 조각은 `ratio` 가 사실상 최솟값이고 면적도 뒤의 어느 후보보다 크거나 같다.
///   엄밀히는 ε-최적이다 — 어디까지 건너뛰는지는 [`FitCost::is_free`] 문서 참조.
///   **다만 이 경로는 실측에서 거의 발동하지 않는다**: 탐침 예산(4096)이 프론티어
///   안쪽까지 충분히 못 가서 `growth > 0` 이 거의 항상이다. wsjk156 의 시간 단축
///   (ordering 1개 기준 10.5s -> 3.6s)은 사실상 전부 아래 티어 이탈 `break` 의 몫이다.
/// * `ctx.has_budget()` 을 조각마다 확인하고, 예산이 끝나면 **그때까지의 최선**을
///   돌려준다. 예산 초과가 곧 실패가 되면 안 된다.
///
/// # 창(`lookahead`) 안이 전멸하면 창 밖까지 본다
///
/// 창 안 12종이 전부 `options` 가 비면 예전 구현은 `None` 을 돌려줬고, 호출부
/// (`pipeline::place_best_fit`)는 그 즉시 루프를 끊어 **남은 조각 전부를 미배치**로
/// 보고했다. 작은 조각은 아직 들어갈 자리가 있는데도 그렇다. 그래서 창 안에서 아무도
/// 못 놓을 때만 나머지를 마저 훑는다. strip 이 거의 찬 국면에서만 발생하므로 평상시
/// 비용은 0이다.
///
/// 하나도 놓을 수 없으면 `None`.
pub fn pick_next_piece(
    ctx: &mut StrategyContext<'_, '_>,
    remaining: &[usize],
    placed: &[PlacedPiece],
    rule: SelectionRule,
    lookahead: usize,
) -> Option<BestFitPick> {
    #[cfg(test)]
    let band = band_override::get().unwrap_or(BEST_FIT_AREA_BAND);
    #[cfg(not(test))]
    let band = BEST_FIT_AREA_BAND;
    pick_next_piece_in_band(ctx, remaining, placed, rule, lookahead, band)
}

/// [`pick_next_piece`] 의 본체. 면적 티어 폭을 인자로 받는다.
///
/// 상수 [`BEST_FIT_AREA_BAND`] 를 실측으로 고르려면 한 번의 실행에서 여러 폭을 돌려
/// 봐야 하는데, 상수를 직접 읽으면 그럴 수가 없다. 그래서 본체를 분리했다.
fn pick_next_piece_in_band(
    ctx: &mut StrategyContext<'_, '_>,
    remaining: &[usize],
    placed: &[PlacedPiece],
    rule: SelectionRule,
    lookahead: usize,
    area_band: f64,
) -> Option<BestFitPick> {
    if remaining.is_empty() {
        return None;
    }

    // 후보마다 다시 만들면 그것만으로 O(조각 x 후보 x 배치) 가 된다. 한 번만 만든다.
    //
    // `ctx.frontier(placed)` 를 부르지 않고 여기서 직접 접는 이유: 그 함수가 내부에서
    // `world_bounds` 를 **또** 만든다. 두 번 부르면 배치 목록을 두 벌 할당하게 되므로
    // 이미 만든 bbox 를 그대로 접는다. 접는 식은 `StrategyContext::frontier` 와 같다
    // (배치된 조각들의 max_x 최대값, 빈 배치면 0).
    let placed_boxes = ctx.placer.world_bounds(placed);
    let frontier = placed_boxes
        .iter()
        .map(|b| b.max_x)
        .fold(0.0_f64, f64::max);
    let strip_height = ctx.placer.strip_height() as f64;

    // ── 평가 대상 추리기: 종류별 1회, 면적 내림차순 ──
    let mut seen: HashSet<usize> = HashSet::with_capacity(remaining.len());
    // (remaining 인덱스, item_id, 면적)
    let mut candidates: Vec<(usize, usize, f64)> = Vec::new();
    for (idx, &item_id) in remaining.iter().enumerate() {
        if !seen.insert(item_id) {
            continue;
        }
        // 면적은 회전에 불변이므로(`shape_bounds_area` 의 두 번째 값은 bbox 가 아니라
        // 폴리곤 면적이다) 첫 허용 회전에서 한 번만 읽는다.
        let rotations = ctx.rotations_for(item_id);
        let area = sane_area(
            rotations
                .first()
                .and_then(|&rot| ctx.placer.shape_bounds_area(item_id, rot))
                .map(|(_, area)| area),
        );
        candidates.push((idx, item_id, area));
    }
    // `total_cmp` + 인덱스 동률 해소는 진짜 전순서라 `sort_by` 가 panic 하지 않는다
    // (허용오차 비교와 달리 추이적이다).
    candidates.sort_by(|a, b| b.2.total_cmp(&a.2).then(a.0.cmp(&b.0)));
    // 자르지 않고 **창 경계만** 들고 간다 — 창 안이 전멸했을 때 나머지를 마저 보려면
    // 잘라 버린 뒤에는 되돌릴 방법이 없다.
    let window = if lookahead > 0 {
        lookahead.min(candidates.len())
    } else {
        candidates.len()
    };

    // 면적 티어의 하한. 첫 **배치 성공** 후보의 면적으로 고정한다 — 가장 큰 조각이
    // 놓을 자리가 없을 수도 있으므로 `candidates[0]` 로 잡으면 티어가 통째로 빈다.
    let band = area_band.clamp(0.0, 1.0);
    let mut tier_floor: Option<f64> = None;

    let mut best: Option<(BestFitPick, FitCost)> = None;
    for (rank, &(remaining_index, item_id, area)) in candidates.iter().enumerate() {
        // 창 밖은 창 안이 하나도 못 놓았을 때만 본다.
        if rank >= window && best.is_some() {
            break;
        }
        // 면적 티어 밖은 볼 필요가 없다 — 후보가 면적 내림차순이라 뒤는 전부 밖이다.
        // 이 break 가 곧 "큰 조각 우선" 이고, 평가 수를 줄여 시간도 같이 깎는다.
        if tier_floor.is_some_and(|floor| area < floor) {
            break;
        }
        if !ctx.has_budget() {
            break;
        }

        // 프론티어는 위에서 이미 접었다 — `options_for` 가 `world_bounds` 를 또 만들지 않게 넘긴다.
        let options = ctx.options_for_at(item_id, placed, frontier);
        if options.is_empty() {
            continue;
        }
        let metrics: Vec<AnchorMetrics> = options
            .iter()
            .map(|opt| ctx.metrics(opt, &placed_boxes, frontier))
            .collect();

        // 조각 **안에서** 자리를 고르는 규칙은 그대로다 — 여기서는 frontier 가 상수라
        // `cmp_by_rule` 의 1순위(strip 길이 최소)가 곧 "이 조각의 최선의 자리" 다.
        let Some(pos) = pick_best(&options, &metrics, rule) else {
            continue;
        };
        let (Some(&option), Some(&picked)) = (options.get(pos), metrics.get(pos)) else {
            continue;
        };
        let cost = FitCost::new(&picked, frontier, area, strip_height);
        // 놓을 수 있는 가장 큰 조각을 만난 순간 티어 하한이 정해진다.
        if tier_floor.is_none() {
            tier_floor = Some(area * (1.0 - band));
        }

        let wins = match &best {
            None => true,
            Some((current, current_cost)) => {
                piece_beats(rule, (&cost, &picked), (current_cost, &current.metrics))
            }
        };
        if wins {
            best = Some((
                BestFitPick {
                    option,
                    metrics: picked,
                    remaining_index,
                },
                cost,
            ));
        }

        // 공짜 배치는 뒤의 어느 후보에게도 지지 않는다(위 문서 참조). 즉시 확정한다.
        if cost.is_free() {
            break;
        }
    }
    best.map(|(pick, _)| pick)
}

/// 후보 조각이 현재 최선을 이기는가.
///
/// 순서는 `ratio` 오름차순 -> 면적 내림차순 -> [`cmp_by_rule`] 이다. 마지막까지 동률이면
/// `false` — 먼저 잡은 최선(= 면적 내림차순에서 더 앞, 동면적이면 더 작은 `remaining`
/// 인덱스)을 유지해 결정론을 지킨다.
fn piece_beats(
    rule: SelectionRule,
    cand: (&FitCost, &AnchorMetrics),
    current: (&FitCost, &AnchorMetrics),
) -> bool {
    let (cand_cost, cand_metrics) = cand;
    let (cur_cost, cur_metrics) = current;

    let ratio = cmp_smaller_is_better(cand_cost.ratio, cur_cost.ratio, FIT_RATIO_EPS);
    if ratio != Ordering::Equal {
        return ratio == Ordering::Less;
    }
    // 큰 조각 우선 — 자리를 잃으면 되돌릴 수 없는 쪽을 먼저 놓는다.
    let area = cur_cost.area.total_cmp(&cand_cost.area);
    if area != Ordering::Equal {
        return area == Ordering::Less;
    }
    cmp_by_rule(rule, cand_metrics, cur_metrics) == Ordering::Less
}

// ────────────────────────────────────────────────────────────────────────────
// 비교 원시 연산
// ────────────────────────────────────────────────────────────────────────────

/// 작을수록 좋은 값의 비교. 차이가 `eps` 이하면 [`Ordering::Equal`].
fn cmp_smaller_is_better(a: f64, b: f64, eps: f64) -> Ordering {
    if (a - b).abs() <= eps {
        Ordering::Equal
    } else if a < b {
        Ordering::Less
    } else {
        Ordering::Greater
    }
}

/// 조각 면적을 쓰기 전에 눕힌다. 읽을 수 없거나 비유한·비양수면 `0.0`.
///
/// # 왜 `unwrap_or(0.0)` 만으로는 부족한가 — 티어가 조용히 꺼진다
///
/// 이 값은 [`pick_next_piece_in_band`] 에서 **두 곳**에 쓰이는데 둘 다 비유한 값에
/// 조용히 무너진다.
///
/// 1. 정렬 키. `total_cmp` 는 `NaN` 을 `+inf` 보다 **크게** 보므로 `NaN` 면적 후보가
///    면적 내림차순의 맨 앞을 차지한다.
/// 2. 티어 하한. 그 후보가 배치에 성공하면 `tier_floor = NaN` 이 되고 `area < NaN` 은
///    언제나 `false` 라 티어 이탈 `break` 가 **영원히 발동하지 않는다**.
///
/// 즉 면적 하나가 망가지면 [`BEST_FIT_AREA_BAND`] 가 통째로 무력화되어 제한 없는
/// best-fit(그 문서의 `BAND 1.00` 행 — wsjk156 71.52% -> 62.83%)으로 되돌아간다.
/// 면적 하나 때문에 밀도 8.7%p 를 조용히 잃는 대신 그 후보만 "가장 작음" 으로 밀어낸다.
///
/// [`FitCost::new`] 는 이미 같은 정책으로 막혀 있었다. 입구를 안 막아 두면 그 방어가
/// 반쪽이라 여기서 함께 막는다.
fn sane_area(area: Option<f64>) -> f64 {
    match area {
        Some(a) if a.is_finite() && a > 0.0 => a,
        _ => 0.0,
    }
}

/// 작을수록 좋은 축에서 비유한 값을 "가장 나쁨" 으로 바꾼다.
fn worst_if_not_finite_min(v: f64) -> f64 {
    if v.is_finite() { v } else { f64::MAX }
}

/// 클수록 좋은 축에서 비유한 값을 "가장 나쁨" 으로 바꾼다.
fn worst_if_not_finite_max(v: f64) -> f64 {
    if v.is_finite() { v } else { f64::MIN }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nfp_lbf::placement::Aabb;

    /// 좌표만 다른 더미 선택지. 선택 규칙은 `metrics` 만 보므로 형상은 무관하다.
    fn option_at(x: f64, y: f64) -> AnchorOption {
        AnchorOption {
            item_id: 0,
            rotation: 0,
            x,
            y,
            world: Aabb {
                min_x: x,
                min_y: y,
                max_x: x + 10.0,
                max_y: y + 10.0,
            },
            touches: 0,
        }
    }

    fn metrics_of(strip_length: f64, contact: f64, x: f64, y: f64) -> AnchorMetrics {
        AnchorMetrics {
            strip_length,
            contact,
            waste: 0.0,
            touches: 0,
            x,
            y,
        }
    }

    /// 후보 수만큼의 더미 선택지 (길이 짝을 맞추기 위한 것).
    fn options_for(metrics: &[AnchorMetrics]) -> Vec<AnchorOption> {
        metrics.iter().map(|m| option_at(m.x, m.y)).collect()
    }

    #[test]
    fn the_two_rules_pick_different_candidates_when_contact_and_x_conflict() {
        // 프론티어가 이미 200mm 라 두 후보 모두 strip 을 늘리지 않는다 — 1순위 동률.
        // 0번: 맨 왼쪽이지만 아무 것과도 맞닿지 않는다.
        // 1번: 50mm 오른쪽이지만 이웃에 120mm 밀착한다.
        let metrics = vec![
            metrics_of(200.0, 0.0, 0.0, 0.0),
            metrics_of(200.0, 120.0, 50.0, 0.0),
        ];
        let options = options_for(&metrics);

        assert_eq!(
            pick_best(&options, &metrics, SelectionRule::LeftBottom),
            Some(0),
            "좌하단 규칙은 x 가 작은 쪽을 고른다"
        );
        assert_eq!(
            pick_best(&options, &metrics, SelectionRule::MaxContact),
            Some(1),
            "접촉 최대 규칙은 밀착한 쪽을 고른다"
        );
    }

    #[test]
    fn both_rules_agree_on_the_shorter_strip_because_it_is_the_shared_first_key() {
        // 0번: strip 이 길지만 접촉이 크고 x 도 작다 — 그래도 져야 한다.
        // 1번: strip 이 짧다.
        let metrics = vec![
            metrics_of(300.0, 500.0, 0.0, 0.0),
            metrics_of(200.0, 0.0, 190.0, 90.0),
        ];
        let options = options_for(&metrics);

        for rule in [SelectionRule::LeftBottom, SelectionRule::MaxContact] {
            assert_eq!(
                pick_best(&options, &metrics, rule),
                Some(1),
                "{rule:?}: 1순위인 strip 길이가 나머지를 이겨야 한다"
            );
        }
    }

    #[test]
    fn a_perfect_tie_resolves_to_the_smaller_index() {
        let metrics = vec![
            metrics_of(100.0, 5.0, 10.0, 10.0),
            metrics_of(100.0, 5.0, 10.0, 10.0),
            metrics_of(100.0, 5.0, 10.0, 10.0),
        ];
        let options = options_for(&metrics);

        for rule in [SelectionRule::LeftBottom, SelectionRule::MaxContact] {
            assert_eq!(pick_best(&options, &metrics, rule), Some(0));
            assert_eq!(pick_top_k(&options, &metrics, rule, 3), vec![0, 1, 2]);
        }
    }

    #[test]
    fn differences_below_the_tolerance_count_as_a_tie() {
        // strip 은 1e-5 차이(허용오차 1e-4 이하) 라 동률이고, 그다음 키로 내려가야 한다.
        let metrics = vec![
            metrics_of(100.000_01, 0.0, 50.0, 0.0),
            metrics_of(100.0, 0.0, 5.0, 0.0),
        ];
        let options = options_for(&metrics);
        assert_eq!(
            pick_best(&options, &metrics, SelectionRule::LeftBottom),
            Some(1),
            "strip 차이가 잡음 수준이면 x 가 결정해야 한다"
        );

        // 접촉은 상대 오차 — 1000mm 기준 1e-3 이면 1mm 아래 차이가 동률이다.
        let contact = vec![
            metrics_of(100.0, 1000.0, 5.0, 0.0),
            metrics_of(100.0, 1000.5, 50.0, 0.0),
        ];
        let contact_options = options_for(&contact);
        assert_eq!(
            pick_best(&contact_options, &contact, SelectionRule::MaxContact),
            Some(0),
            "0.5mm 접촉 차이는 1000mm 규모에서 동률이라 x 가 결정한다"
        );
    }

    #[test]
    fn pick_top_k_returns_exactly_k_distinct_indices_in_ranked_order() {
        let metrics = vec![
            metrics_of(100.0, 0.0, 40.0, 0.0), // 0
            metrics_of(100.0, 0.0, 10.0, 0.0), // 1  <- 2등
            metrics_of(90.0, 0.0, 90.0, 0.0),  // 2  <- 1등 (strip 최소)
            metrics_of(100.0, 0.0, 70.0, 0.0), // 3
            metrics_of(100.0, 0.0, 30.0, 0.0), // 4  <- 3등
        ];
        let options = options_for(&metrics);

        let top = pick_top_k(&options, &metrics, SelectionRule::LeftBottom, 3);
        assert_eq!(top, vec![2, 1, 4]);

        // 중복이 없다.
        let unique: HashSet<usize> = top.iter().copied().collect();
        assert_eq!(unique.len(), top.len());

        // 첫 원소는 언제나 pick_best 와 같다.
        assert_eq!(
            top.first().copied(),
            pick_best(&options, &metrics, SelectionRule::LeftBottom)
        );

        // 상위 k 는 전체 순위의 접두사다.
        let all = pick_top_k(&options, &metrics, SelectionRule::LeftBottom, metrics.len());
        assert_eq!(all.len(), metrics.len());
        assert_eq!(all.get(..top.len()), Some(&top[..]));
    }

    #[test]
    fn pick_top_k_ranks_by_contact_first_under_the_max_contact_rule() {
        // strip 이 전부 동률이라 2순위가 순위를 정한다. LeftBottom 과 MaxContact 가
        // **서로 뒤집힌 순위**를 내야 규칙이 진짜로 갈린 것이다.
        let metrics = vec![
            metrics_of(100.0, 10.0, 0.0, 0.0),  // 0: 가장 왼쪽, 접촉 최소
            metrics_of(100.0, 50.0, 20.0, 0.0), // 1
            metrics_of(100.0, 90.0, 40.0, 0.0), // 2: 접촉 최대, 가장 오른쪽
        ];
        let options = options_for(&metrics);

        assert_eq!(
            pick_top_k(&options, &metrics, SelectionRule::MaxContact, 3),
            vec![2, 1, 0],
            "접촉 내림차순이어야 한다"
        );
        assert_eq!(
            pick_top_k(&options, &metrics, SelectionRule::LeftBottom, 3),
            vec![0, 1, 2],
            "좌하단은 x 오름차순이어야 한다"
        );
    }

    #[test]
    fn pick_top_k_returns_everything_when_k_exceeds_the_candidate_count() {
        let metrics = vec![
            metrics_of(100.0, 0.0, 20.0, 0.0),
            metrics_of(100.0, 0.0, 10.0, 0.0),
        ];
        let options = options_for(&metrics);

        let top = pick_top_k(&options, &metrics, SelectionRule::LeftBottom, 99);
        assert_eq!(top, vec![1, 0]);
        assert_eq!(
            pick_top_k(&options, &metrics, SelectionRule::LeftBottom, 0),
            Vec::<usize>::new(),
            "k = 0 이면 빈 벡터다"
        );
    }

    #[test]
    fn empty_input_yields_none_and_an_empty_ranking() {
        let options: Vec<AnchorOption> = Vec::new();
        let metrics: Vec<AnchorMetrics> = Vec::new();
        for rule in [SelectionRule::LeftBottom, SelectionRule::MaxContact] {
            assert_eq!(pick_best(&options, &metrics, rule), None);
            assert!(pick_top_k(&options, &metrics, rule, 5).is_empty());
        }
    }

    #[test]
    fn mismatched_option_and_metric_lengths_stop_at_the_shorter_one() {
        // metrics 가 더 길다 — 인덱스 2는 보이지 않아야 한다(더 좋은 값이어도).
        let metrics = vec![
            metrics_of(100.0, 0.0, 20.0, 0.0),
            metrics_of(100.0, 0.0, 10.0, 0.0),
            metrics_of(10.0, 0.0, 0.0, 0.0),
        ];
        let options = vec![option_at(20.0, 0.0), option_at(10.0, 0.0)];

        assert_eq!(
            pick_best(&options, &metrics, SelectionRule::LeftBottom),
            Some(1)
        );
        assert_eq!(
            pick_top_k(&options, &metrics, SelectionRule::LeftBottom, 5),
            vec![1, 0]
        );
    }

    #[test]
    fn non_finite_metrics_lose_instead_of_panicking() {
        let metrics = vec![
            metrics_of(f64::NAN, f64::NAN, f64::NAN, f64::NAN),
            metrics_of(100.0, 0.0, 10.0, 0.0),
            metrics_of(f64::INFINITY, 0.0, 0.0, 0.0),
        ];
        let options = options_for(&metrics);

        for rule in [SelectionRule::LeftBottom, SelectionRule::MaxContact] {
            assert_eq!(
                pick_best(&options, &metrics, rule),
                Some(1),
                "{rule:?}: 유한한 후보가 이겨야 한다"
            );
            // 순위가 전부 나오고 중복이 없다.
            let all = pick_top_k(&options, &metrics, rule, 3);
            assert_eq!(all.len(), 3);
            assert_eq!(all.first().copied(), Some(1));
            assert_eq!(all.iter().copied().collect::<HashSet<_>>().len(), 3);
        }
    }

    #[test]
    fn an_infinite_contact_loses_instead_of_tying_with_everything() {
        // strip·x·y 를 전부 같게 두어 **접촉만** 승부를 가르게 한다.
        // 0번의 접촉이 `+inf` 다 — "가장 나쁨" 으로 밀려 1번이 이겨야 한다.
        //
        // 허용오차를 날값으로 재면(`CONTACT_REL_EPS * inf == inf`) `|차이| <= eps` 가
        // 항상 참이 되어 접촉 축이 통째로 동률로 무너지고, 동률이면 앞 인덱스가 남으므로
        // **망가진 후보가 이긴다**. 정규화한 값으로 재야 eps 가 유한하다.
        let metrics = vec![
            metrics_of(200.0, f64::INFINITY, 0.0, 0.0),
            metrics_of(200.0, 120.0, 0.0, 0.0),
        ];
        let options = options_for(&metrics);

        assert_eq!(
            pick_best(&options, &metrics, SelectionRule::MaxContact),
            Some(1),
            "접촉이 +inf 인 후보가 유한한 후보를 이기면 안 된다"
        );
        assert_eq!(
            pick_top_k(&options, &metrics, SelectionRule::MaxContact, 2),
            vec![1, 0]
        );
        // 좌하단 규칙에서도 접촉은 3순위로 쓰인다 — 같은 계약이 성립해야 한다.
        assert_eq!(
            pick_best(&options, &metrics, SelectionRule::LeftBottom),
            Some(1)
        );
    }

    /// 조각 비교용 (FitCost, AnchorMetrics) 짝을 만든다.
    ///
    /// `frontier` 는 이번 단계 모든 조각에게 같으므로 200mm 로 고정하고, strip 높이는
    /// 계산이 눈으로 보이도록 1.0 으로 둔다 — 그러면 `ratio = growth / area` 다.
    fn piece(strip_length: f64, area: f64, contact: f64, x: f64) -> (FitCost, AnchorMetrics) {
        let m = metrics_of(strip_length, contact, x, 0.0);
        (FitCost::new(&m, 200.0, area, 1.0), m)
    }

    #[test]
    fn a_bigger_piece_wins_even_though_it_grows_the_strip_more() {
        // 이것이 -12.80%p 회귀의 핵심이다. 옛 기준(증가량 최소)이면 작은 조각이 이긴다.
        //   큰 조각 : growth 50mm, 면적 5000mm²  -> ratio 0.010
        //   작은 조각: growth 10mm, 면적  200mm² -> ratio 0.050
        let big = piece(250.0, 5000.0, 0.0, 200.0);
        let small = piece(210.0, 200.0, 0.0, 200.0);

        for rule in [SelectionRule::LeftBottom, SelectionRule::MaxContact] {
            assert!(
                piece_beats(rule, (&big.0, &big.1), (&small.0, &small.1)),
                "{rule:?}: 면적당 소비가 적은 큰 조각이 이겨야 한다"
            );
            assert!(!piece_beats(rule, (&small.0, &small.1), (&big.0, &big.1)));
        }
    }

    #[test]
    fn the_cheaper_strip_area_per_piece_area_wins() {
        // 같은 면적이면 여전히 "적게 미는 쪽" 이 이긴다 — 정규화가 그 순서를 깨지 않는다.
        let cheap = piece(210.0, 1000.0, 0.0, 200.0);
        let costly = piece(260.0, 1000.0, 900.0, 200.0);
        for rule in [SelectionRule::LeftBottom, SelectionRule::MaxContact] {
            assert!(piece_beats(rule, (&cheap.0, &cheap.1), (&costly.0, &costly.1)));
            assert!(!piece_beats(rule, (&costly.0, &costly.1), (&cheap.0, &cheap.1)));
        }
    }

    #[test]
    fn a_ratio_tie_is_broken_by_the_larger_area_then_by_the_placement_rule() {
        // ratio 가 같으면(둘 다 0.02) 큰 조각이 먼저다.
        let large = piece(300.0, 5000.0, 0.0, 200.0); // growth 100 / 5000
        let smallish = piece(220.0, 1000.0, 900.0, 200.0); // growth  20 / 1000
        for rule in [SelectionRule::LeftBottom, SelectionRule::MaxContact] {
            assert!(
                piece_beats(rule, (&large.0, &large.1), (&smallish.0, &smallish.1)),
                "{rule:?}: ratio 동률이면 큰 조각 우선이다"
            );
        }

        // ratio 도 면적도 같으면 그때서야 자리 비교 규칙이 갈린다.
        let left = piece(210.0, 1000.0, 0.0, 5.0);
        let snug = piece(210.0, 1000.0, 300.0, 80.0);
        assert!(piece_beats(
            SelectionRule::LeftBottom,
            (&left.0, &left.1),
            (&snug.0, &snug.1)
        ));
        assert!(piece_beats(
            SelectionRule::MaxContact,
            (&snug.0, &snug.1),
            (&left.0, &left.1)
        ));
        // 완전 동률이면 먼저 잡은 최선을 유지한다(= 면적 내림차순에서 더 앞).
        assert!(!piece_beats(
            SelectionRule::LeftBottom,
            (&left.0, &left.1),
            (&left.0, &left.1)
        ));
    }

    #[test]
    fn a_placement_inside_the_frontier_is_free_and_costs_nothing() {
        // 프론티어(200mm) 안쪽에 통째로 들어가면 strip 을 새로 사지 않는다.
        let inside = piece(200.0, 1000.0, 0.0, 100.0);
        assert!(inside.0.is_free());
        assert_eq!(inside.0.ratio, 0.0);
        // 조금이라도 밀면 공짜가 아니다.
        let outside = piece(200.5, 1000.0, 0.0, 100.0);
        assert!(!outside.0.is_free());
        assert!(outside.0.ratio > 0.0);
    }

    #[test]
    fn a_degenerate_area_loses_instead_of_producing_a_nan_ratio() {
        // 면적 0 이면 나눗셈이 inf/NaN 이 된다 — 비교가 무너지지 않고 지기만 해야 한다.
        let broken = piece(201.0, 0.0, 0.0, 200.0);
        let sane = piece(300.0, 10.0, 0.0, 200.0);
        assert!(broken.0.ratio.is_finite());
        assert!(piece_beats(
            SelectionRule::LeftBottom,
            (&sane.0, &sane.1),
            (&broken.0, &broken.1)
        ));
        assert!(!piece_beats(
            SelectionRule::LeftBottom,
            (&broken.0, &broken.1),
            (&sane.0, &sane.1)
        ));
    }

    // ────────────────────────────────────────────────────────────────────
    // ⑥ pick_next_piece — 진짜 오라클 위에서
    // ────────────────────────────────────────────────────────────────────
    //
    // 순수 함수 `piece_beats` 만 테스트하면 **비교 로직**은 덮이지만 그 앞의
    // 세 장치(중복 item_id 스킵, 면적 내림차순 lookahead 절단, 예산 확인)는 통째로
    // 안 덮인다. 그 셋이 이 함수의 비용을 결정하는 부분이라 진짜 `SequentialPlacer`
    // 를 세워 확인한다. 외부 데이터 파일에는 의존하지 않는다 — 직사각형이면 충분하다.

    use jagua_rs::collision_detection::CDEConfig;
    use jagua_rs::entities::Item;
    use jagua_rs::geometry::DTransformation;
    use jagua_rs::geometry::OriginalShape;
    use jagua_rs::geometry::fail_fast::SPSurrogateConfig;
    use jagua_rs::geometry::geo_enums::RotationRange;
    use jagua_rs::geometry::primitives::{Point, SPolygon};
    use jagua_rs::geometry::shape_modification::{ShapeModifyConfig, ShapeModifyMode};
    use jagua_rs::probs::spp::entities::{SPInstance, Strip};

    use crate::nfp_lbf::cache::{NfpLbfCache, PreparedLibrary};
    use crate::nfp_lbf::{PlacementScoreWeights, SequentialPlacer};
    use crate::nfp_strategy::config::DEFAULT_NFP_STRATEGY_CONFIG;

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
    /// strip 고정 변(mm).
    const STRIP_H: f32 = 20.0;
    /// IFP 계산에 쓸 x 상한(mm).
    const STRIP_CAP: f32 = 120.0;

    fn rect_item(id: usize, w: f32, h: f32) -> std::sync::Arc<Item> {
        let shape = SPolygon::new(vec![
            Point(0.0, 0.0),
            Point(w, 0.0),
            Point(w, h),
            Point(0.0, h),
        ])
        .expect(crate::dmsg!("직사각형은 유효한 폴리곤이다"));
        Item::new(id, (id) as u64, OriginalShape {
                shape,
                pre_transform: DTransformation::empty(),
                modify_mode: ShapeModifyMode::Inflate,
                modify_config: ShapeModifyConfig::default(),
            }, jagua_rs::geometry::AllowedOrientations::new(RotationRange::None, vec![]).unwrap(), None, SURROGATE).map(std::sync::Arc::new)
        .expect(crate::dmsg!("유효한 아이템"))
    }

    /// 다섯 종류의 직사각형. **면적 순위와 `ratio` 순위가 어긋나도록** 골랐다 —
    /// 그래야 "면적으로 자르고(lookahead) ratio 로 고른다" 가 구분된다.
    ///
    /// 빈 strip(`frontier = 0`)에서 `ratio = 폭 x STRIP_H / 면적` 이다.
    ///
    /// | item | 크기(mm) | 면적(mm²) | 폭(mm) | ratio |
    /// |------|----------|-----------|--------|-------|
    /// | 0    | 10 x 4   | 40        | 10     | 5.00  |
    /// | 1    | 6 x 9    | 54        | 6      | 2.22  |
    /// | 2    | 4 x 4    | 16        | 4      | 5.00  |
    /// | 3    | 20 x 5   | 100       | 20     | 4.00  |
    /// | 4    | 200 x 5  | 1000      | 200    | 배치 불가 (폭 상한 120mm 초과) |
    ///
    /// item 4 는 IFP 가 비어 후보가 하나도 안 나온다. `lookahead` 창이 통째로 막혔을 때
    /// 창 밖까지 보는지 확인하는 데 쓴다 — 면적이 가장 커서 항상 창의 맨 앞을 차지한다.
    fn fixture_instance() -> SPInstance {
        let strip =
            Strip::new(STRIP_H, CDE, ShapeModifyConfig::default(), STRIP_CAP).expect(crate::dmsg!("유효한 strip"));
        SPInstance::new(
            vec![
                (rect_item(0, 10.0, 4.0), 4),
                (rect_item(1, 6.0, 9.0), 4),
                (rect_item(2, 4.0, 4.0), 4),
                (rect_item(3, 20.0, 5.0), 4),
                (rect_item(4, 200.0, 5.0), 4),
            ],
            strip,
        )
    }

    /// 빈 strip 위에서 조각 선택을 한 번 돌린다.
    ///
    /// 빈 배치라 `frontier = 0` 이고 조각마다 `strip_length = 폭` 이므로
    /// `ratio = 폭 x STRIP_H / 면적` 이다 (위 표 참조).
    ///
    /// `area_band` 를 명시로 받는다. 프로덕션 기본값([`BEST_FIT_AREA_BAND`])은 티어를
    /// 좁게 잡아 사실상 면적 내림차순이 되므로, `ratio` 비교나 `lookahead` 절단 같은
    /// **다른 장치**를 확인하려면 티어를 열어(`1.0`) 놓아야 한다.
    fn pick_in_band_on_empty_strip(
        remaining: &[usize],
        lookahead: usize,
        expired_budget: bool,
        area_band: f64,
    ) -> Option<BestFitPick> {
        let instance = fixture_instance();
        let library = PreparedLibrary::build(&instance, &[0], 0.0).expect(crate::dmsg!("라이브러리가 준비된다"));
        let mut cache = NfpLbfCache::new(library);
        let mut placer = SequentialPlacer::new(
            &mut cache,
            STRIP_H,
            STRIP_CAP,
            PlacementScoreWeights::default(),
        );
        // 이미 지난 시각을 마감으로 주면 첫 조각에서 바로 예산이 없다.
        let deadline = expired_budget.then(jagua_rs::Instant::now);
        let mut ctx = StrategyContext::new(
            &mut placer,
            DEFAULT_NFP_STRATEGY_CONFIG,
            Vec::new(),
            deadline,
        );
        pick_next_piece_in_band(
            &mut ctx,
            remaining,
            &[],
            SelectionRule::LeftBottom,
            lookahead,
            area_band,
        )
    }

    /// 티어를 열어 둔([`pick_in_band_on_empty_strip`] `area_band = 1.0`) 단축 호출.
    fn pick_on_empty_strip(
        remaining: &[usize],
        lookahead: usize,
        expired_budget: bool,
    ) -> Option<BestFitPick> {
        pick_in_band_on_empty_strip(remaining, lookahead, expired_budget, 1.0)
    }

    #[test]
    fn best_fit_picks_the_cheapest_strip_area_per_piece_area_on_a_real_placer() {
        // ratio: item0 5.00 / item1 2.22 / item2 5.00 -> item 1 이 이긴다.
        // 옛 기준(증가량 최소)이면 가장 좁은(=가장 작은) item 2 가 이겼다.
        let pick = pick_on_empty_strip(&[0, 1, 2], 0, false).expect(crate::dmsg!("셋 다 놓을 수 있다"));
        assert_eq!(
            pick.option.item_id, 1,
            "면적당 소비가 가장 적은 item 1 이 이겨야 한다 (가장 좁은 item 2 가 아니다)"
        );
        assert_eq!(pick.remaining_index, 1, "remaining 안의 자리를 그대로 돌려준다");
        assert!(
            (pick.metrics.strip_length - 6.0).abs() < 0.5,
            "빈 strip 위 item 1 의 strip 길이는 폭 6mm 여야 한다: {}",
            pick.metrics.strip_length
        );
    }

    #[test]
    fn a_repeated_item_id_is_evaluated_once_and_reports_its_first_slot() {
        // remaining = [3, 3, 1], lookahead = 2.
        // 종류별 1회로 접으면 평가 대상은 {item 3(100mm²), item 1(54mm²)} 두 종류이고
        // 둘 다 창에 남아 ratio 가 작은 item 1(2.22 < 4.00)이 이긴다.
        // 접지 않으면 [item3, item3, item1] 중 면적 상위 2개가 전부 item 3 이라
        // item 1 은 평가조차 되지 않는다. 두 경로가 **다른 답**을 내므로 구분된다.
        let pick = pick_on_empty_strip(&[3, 3, 1], 2, false).expect(crate::dmsg!("놓을 수 있다"));
        assert_eq!(
            pick.option.item_id, 1,
            "같은 item_id 를 두 번 평가하면 lookahead 를 낭비해 item 1 을 놓친다"
        );
        assert_eq!(pick.remaining_index, 2);

        // 중복 종류의 remaining_index 는 **첫 번째** 자리다 (호출부가 하나만 깎는다).
        let repeated = pick_on_empty_strip(&[1, 1, 1], 0, false).expect(crate::dmsg!("놓을 수 있다"));
        assert_eq!(repeated.option.item_id, 1);
        assert_eq!(repeated.remaining_index, 0);
    }

    #[test]
    fn the_lookahead_keeps_the_largest_pieces_by_area_and_drops_the_rest() {
        // 전수 평가면 ratio 가 작은 item 1(2.22)이 item 3(4.00)을 이긴다.
        let all = pick_on_empty_strip(&[1, 3], 0, false).expect(crate::dmsg!("놓을 수 있다"));
        assert_eq!(all.option.item_id, 1);

        // lookahead = 1 이면 면적 최대(100mm²)인 item 3 하나만 평가한다.
        // ratio 기준으로는 item 1 이 더 좋으므로, 답이 바뀌는 것 자체가 절단의 증거다.
        let cut = pick_on_empty_strip(&[1, 3], 1, false).expect(crate::dmsg!("놓을 수 있다"));
        assert_eq!(
            cut.option.item_id, 3,
            "면적 내림차순 상위 1개만 봐야 한다 (item 3 = 100mm²)"
        );

        // lookahead = 2 면 둘 다 보이므로 다시 item 1 이다.
        let two = pick_on_empty_strip(&[1, 3], 2, false).expect(crate::dmsg!("놓을 수 있다"));
        assert_eq!(two.option.item_id, 1);
    }

    #[test]
    fn a_fully_infeasible_lookahead_window_falls_through_to_the_rest() {
        // item 4(1000mm²)는 폭 200mm 로 strip 폭 상한 120mm 를 넘어 후보가 하나도 없다.
        // 면적이 가장 커서 lookahead = 1 창을 혼자 차지한다.
        //
        // 창만 보고 포기하면 `None` 이고, 호출부(place_best_fit)는 그 즉시 루프를 끊어
        // **남은 조각 전부를 미배치**로 보고한다. 창 밖까지 봐야 item 2 가 놓인다.
        let pick = pick_on_empty_strip(&[4, 2], 1, false)
            .expect(crate::dmsg!("창이 막혀도 창 밖의 item 2 를 찾아야 한다"));
        assert_eq!(pick.option.item_id, 2);
        assert_eq!(pick.remaining_index, 1);

        // 놓을 수 있는 것이 정말 하나도 없으면 여전히 None 이다.
        assert!(pick_on_empty_strip(&[4], 1, false).is_none());
    }

    #[test]
    fn the_area_tier_puts_the_big_piece_first_and_best_fit_only_breaks_ties_inside_it() {
        // item 3(100mm², ratio 4.00) vs item 1(54mm², ratio 2.22).
        // 티어를 열면 ratio 가 좋은 item 1 이, 좁히면 면적이 큰 item 3 이 이겨야 한다.
        let open = pick_in_band_on_empty_strip(&[1, 3], 0, false, 1.0).expect(crate::dmsg!("놓을 수 있다"));
        assert_eq!(open.option.item_id, 1, "티어를 열면 ratio 가 결정한다");

        let tiered = pick_in_band_on_empty_strip(&[1, 3], 0, false, 0.05).expect(crate::dmsg!("놓을 수 있다"));
        assert_eq!(
            tiered.option.item_id, 3,
            "면적이 46% 작은 item 1 은 5% 티어 밖이라 겨루지 못한다"
        );

        // 티어 안(면적 차 0%)이면 다시 ratio 가 결정한다 — item 0(40mm², 5.00) 과
        // item 1(54mm², 2.22) 은 27% 차이라 0.30 티어에 함께 들어온다.
        let inside = pick_in_band_on_empty_strip(&[0, 1], 0, false, 0.30).expect(crate::dmsg!("놓을 수 있다"));
        assert_eq!(inside.option.item_id, 1);

        // 프로덕션 기본값은 좁은 티어 쪽이다.
        assert!(
            (0.0..=0.25).contains(&BEST_FIT_AREA_BAND),
            "티어를 넓히면 한계비용 그리디의 함정으로 되돌아간다: {BEST_FIT_AREA_BAND}"
        );
    }

    #[test]
    fn the_area_tier_is_anchored_to_the_largest_placeable_piece_not_the_largest_one() {
        // item 4(1000mm²)는 배치 불가다. 그것을 기준으로 티어를 잡으면 하한이 950mm² 라
        // 남은 조각이 전부 밖으로 밀려 아무 것도 못 놓는다.
        let pick = pick_in_band_on_empty_strip(&[4, 3, 1], 0, false, 0.05)
            .expect(crate::dmsg!("배치 불가 조각은 티어 기준이 되어선 안 된다"));
        assert_eq!(pick.option.item_id, 3, "놓을 수 있는 가장 큰 조각이 기준이다");
    }

    #[test]
    fn a_broken_area_is_flattened_before_it_can_poison_the_sort_or_the_tier() {
        // 정상값은 그대로 통과한다.
        assert_eq!(sane_area(Some(1234.5)), 1234.5);

        // `NaN` 이 가장 위험하다: `total_cmp` 가 `+inf` 보다 크게 보므로 면적 내림차순의
        // 맨 앞을 차지하고, 그 후보가 놓이면 `tier_floor = NaN` 이라 `area < floor` 가
        // 영원히 false — 면적 티어가 통째로 꺼진다(= BAND 1.00 거동).
        assert_eq!(sane_area(Some(f64::NAN)), 0.0);
        assert_eq!(sane_area(Some(f64::INFINITY)), 0.0);
        assert_eq!(sane_area(Some(f64::NEG_INFINITY)), 0.0);
        // 0·음수 면적은 `ratio` 의 분모로 못 쓴다.
        assert_eq!(sane_area(Some(0.0)), 0.0);
        assert_eq!(sane_area(Some(-1.0)), 0.0);
        // 형상을 못 읽은 경우.
        assert_eq!(sane_area(None), 0.0);

        // 눕힌 값은 정렬에서 맨 뒤로 간다 — `NaN` 이었다면 맨 앞이었을 자리다.
        let mut v = vec![(0usize, 0usize, sane_area(Some(f64::NAN))), (1, 1, 10.0)];
        v.sort_by(|a, b| b.2.total_cmp(&a.2).then(a.0.cmp(&b.0)));
        assert_eq!(
            v.first().map(|c| c.1),
            Some(1),
            "망가진 면적이 면적 내림차순의 맨 앞을 차지하면 안 된다"
        );
    }

    #[test]
    fn an_expired_budget_yields_no_pick_instead_of_panicking() {
        assert!(
            pick_on_empty_strip(&[0, 1, 2], 0, true).is_none(),
            "예산이 처음부터 없으면 최선이 없으므로 None 이다"
        );
    }

    #[test]
    fn an_empty_remaining_list_yields_no_pick() {
        assert!(pick_on_empty_strip(&[], 0, false).is_none());
        assert!(pick_on_empty_strip(&[], 4, false).is_none());
    }

    #[test]
    fn the_rule_follows_the_contact_maximization_flag() {
        assert_eq!(
            SelectionRule::from_contact_maximization(true),
            SelectionRule::MaxContact
        );
        assert_eq!(
            SelectionRule::from_contact_maximization(false),
            SelectionRule::LeftBottom
        );
        assert_eq!(SelectionRule::default(), SelectionRule::LeftBottom);
    }

    // ────────────────────────────────────────────────────────────────────
    // 실 인스턴스 계측 하네스 (여성WSjk 156-패널). 자동 회귀가 아니라 **측정**이라
    // `#[ignore]` 다 — 한 번 도는 데 수십 초가 걸린다. 모듈 문서의 실측표를 다시
    // 뽑는 방법이기도 하다.
    //
    //   cargo test -p swarm-core --release \
    //     --features "simd,capmods,adaptive,structure-first" \
    //     --lib nfp_strategy::select::tests::measure -- --ignored --nocapture
    // ────────────────────────────────────────────────────────────────────

    use crate::config::DEFAULT_SWARM_CONFIG;
    use crate::nfp_lbf::{NfpLbfConfig, build_best_initial_nest};
    use rand::SeedableRng;
    use rand::rngs::Xoshiro256PlusPlus;

    /// 이름을 받아 `data/input/<name>` 을 읽는다 (⑥ 계측이 여러 인스턴스를 훑는다).
    pub(crate) fn load_named(name: &str) -> SPInstance {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../data/input")
            .join(name);
        let (ext, _) = crate::util::io::read_spp_input(&path).expect(crate::dmsg!("인스턴스 읽기"));
        let c = DEFAULT_SWARM_CONFIG;
        let imp = crate::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
        jagua_rs::probs::spp::io::import_instance(&imp, &ext).expect(crate::dmsg!("import"))
    }

    pub(crate) fn load_marker() -> SPInstance {
        let path =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input/wsjk156.json");
        let (ext, _) = crate::util::io::read_spp_input(&path).expect(crate::dmsg!("wsjk156.json 읽기"));
        let c = DEFAULT_SWARM_CONFIG;
        let imp = crate::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
        jagua_rs::probs::spp::io::import_instance(&imp, &ext).expect(crate::dmsg!("import"))
    }

    /// 벤치와 같은 조건(유효 ordering 12개, 탐침 4096)으로 한 팔을 돌린다.
    pub(crate) fn run_arm(
        inst: &SPInstance,
        strategy: crate::nfp_strategy::NfpStrategyConfig,
    ) -> (f32, f32, u128, usize) {
        let factor = strategy.estimated_cost_factor().max(1.0);
        let cfg = NfpLbfConfig {
            n_orderings: ((12.0 * factor).ceil()) as usize,
            cache_budget: std::time::Duration::from_secs(10),
            total_budget: std::time::Duration::from_secs(900),
            concave_simplify_tolerance: 0.02,
            max_candidate_probes: std::env::var("MEASURE_PROBES")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(4096),
            strategy,
            ..NfpLbfConfig::default()
        };
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(0);
        let t = std::time::Instant::now();
        match build_best_initial_nest(inst, &cfg, &mut rng) {
            Some(n) => (
                n.solution.strip_width(),
                n.solution.density() * 100.0,
                t.elapsed().as_millis(),
                n.report.pieces_slid,
            ),
            None => (f32::NAN, f32::NAN, t.elapsed().as_millis(), 0),
        }
    }

    #[test]
    #[ignore = "실 인스턴스 필요 — 계측용"]
    fn measure_contact_maximization() {
        let inst = load_marker();
        let off = crate::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG;

        meas::set(true);
        meas::reset();
        let base = run_arm(&inst, off);
        let base_report = meas::report();

        meas::reset();
        let on = run_arm(
            &inst,
            crate::nfp_strategy::NfpStrategyConfig {
                contact_maximization: true,
                ..off
            },
        );
        let on_report = meas::report();
        meas::set(false);

        println!("\n[③ 계측] 기준(LeftBottom): 폭 {:.2} 밀도 {:.2}% {}ms\n{base_report}", base.0, base.1, base.2);
        println!("\n[③ 계측] 접촉최대(MaxContact): 폭 {:.2} 밀도 {:.2}% {}ms\n{on_report}", on.0, on.1, on.2);
    }

    /// ⑥ 전용: **유효 ordering 을 1개로 고정**해 first-fit 과 best-fit 을 같은 조건에서
    /// 비교한다.
    ///
    /// 벤치(ordering 6~12)로는 두 요인이 섞인다 — 그리디는 서로 다른 ordering 6개 중
    /// 최선을 고르지만 best-fit 은 순서를 multiset 으로만 쓰므로 6개가 전부 같은 답이다.
    /// "best-fit 이 first-fit 보다 나쁠 수 없다" 는 명제는 **ordering 1개**에서만 뜻이
    /// 있으므로 여기서 그 조건을 만든다. `lookahead` 도 함께 훑어 창 크기가 진짜 원인인지
    /// 가린다.
    fn run_one_ordering(
        inst: &SPInstance,
        strategy: crate::nfp_strategy::NfpStrategyConfig,
    ) -> (f32, f32, u128) {
        let factor = strategy.estimated_cost_factor().max(1.0);
        let cfg = NfpLbfConfig {
            // scale_orderings 가 factor 로 다시 나누므로 유효 ordering 은 1개다.
            n_orderings: (factor.ceil()) as usize,
            cache_budget: std::time::Duration::from_secs(10),
            total_budget: std::time::Duration::from_secs(900),
            concave_simplify_tolerance: 0.02,
            max_candidate_probes: 4096,
            strategy,
            ..NfpLbfConfig::default()
        };
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(0);
        let t = std::time::Instant::now();
        match build_best_initial_nest(inst, &cfg, &mut rng) {
            Some(n) => (
                n.solution.strip_width(),
                n.solution.density() * 100.0,
                t.elapsed().as_millis(),
            ),
            None => (f32::NAN, f32::NAN, t.elapsed().as_millis()),
        }
    }

    #[test]
    #[ignore = "실 인스턴스 필요 — 계측용"]
    fn measure_best_fit_lookahead() {
        let inst = load_marker();
        let off = crate::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG;

        let (w0, d0, ms0) = run_one_ordering(&inst, off);
        println!("\n[⑥ 계측] first-fit (ordering 1개)   폭 {w0:9.2} 밀도 {d0:5.2}% {ms0:>6}ms");

        // 폭을 함께 훑는다. [`BEST_FIT_AREA_BAND`] 문서의 두 표 — 티어 없는 옛 거동
        // (`1.00`)과 지금 거동(`0.20`) — 이 **한 번의 실행**에서 다 나와야 서로
        // 비교되고 재현도 된다. 폭을 고정해 두면 문서의 옛 표를 다시 만들 방법이
        // 없어져, 읽는 사람이 어느 조건의 숫자인지 확인할 수 없다.
        for band in [1.00_f64, BEST_FIT_AREA_BAND] {
            println!("[⑥ 계측] ── BAND {band:.2} ──");
            // 0 은 `normalized()` 가 기본값 12 로 되돌리므로 "전수" 는 종류 수(156)보다
            // 큰 값으로 표현한다.
            for lookahead in [4usize, 12, 32, 200] {
                band_override::set(Some(band));
                let (w, d, ms) = run_one_ordering(
                    &inst,
                    crate::nfp_strategy::NfpStrategyConfig {
                        best_fit_selection: true,
                        best_fit_lookahead: lookahead,
                        ..off
                    },
                );
                band_override::set(None);
                println!(
                    "[⑥ 계측] best-fit lookahead {lookahead:<4}     폭 {w:9.2} 밀도 {d:5.2}% {ms:>6}ms  ({:+.2}%p)",
                    d - d0
                );
            }
        }
    }

    /// 면적 티어 폭([`BEST_FIT_AREA_BAND`])을 고르기 위한 계측.
    ///
    /// 상수를 직접 바꿔 가며 다시 빌드하지 않으려고 `pick_next_piece_in_band` 를
    /// 스레드 로컬 오버라이드로 갈아 끼운다 — 계측 코드에서만 쓴다.
    #[test]
    #[ignore = "실 인스턴스 필요 — 계측용"]
    fn measure_best_fit_area_band() {
        // BF_INSTANCES / BF_BANDS 로 훑을 대상을 바꿀 수 있다.
        let names = std::env::var("BF_INSTANCES").unwrap_or_else(|_| "wsjk156.json".into());
        let bands: Vec<f64> = std::env::var("BF_BANDS")
            .unwrap_or_else(|_| "0.10,0.20,0.30,1.00".into())
            .split(',')
            .filter_map(|s| s.trim().parse().ok())
            .collect();
        let off = crate::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG;

        for name in names.split(',').map(str::trim).filter(|s| !s.is_empty()) {
            let inst = load_named(name);
            let (w0, d0, ms0) = run_one_ordering(&inst, off);
            println!("\n[⑥ 티어] {name} first-fit(면적 내림차순) 폭 {w0:9.2} 밀도 {d0:5.2}% {ms0:>6}ms");

            for &band in &bands {
                band_override::set(Some(band));
                let (w, d, ms) = run_one_ordering(
                    &inst,
                    crate::nfp_strategy::NfpStrategyConfig {
                        best_fit_selection: true,
                        best_fit_lookahead: 12,
                        ..off
                    },
                );
                band_override::set(None);
                println!(
                    "[⑥ 티어] {name} BAND {band:<4}            폭 {w:9.2} 밀도 {d:5.2}% {ms:>6}ms  ({:+.2}%p)",
                    d - d0
                );
            }
        }
    }
}

/// 계측 전용 티어 폭 오버라이드.
///
/// 상수를 바꿔 가며 재빌드하면 한 번의 측정에 수 분씩 더 든다. `cfg(test)` 에서만
/// 존재하고 프로덕션 경로에는 남지 않는다.
#[cfg(test)]
mod band_override {
    use std::cell::Cell;

    thread_local! {
        static BAND: Cell<Option<f64>> = const { Cell::new(None) };
    }

    pub(super) fn set(v: Option<f64>) {
        BAND.with(|b| b.set(v));
    }

    pub(super) fn get() -> Option<f64> {
        BAND.with(|b| b.get())
    }
}
