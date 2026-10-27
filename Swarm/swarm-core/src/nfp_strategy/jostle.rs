//! Jostling — 배치 결과를 순서로 되먹여 다시 배치한다 (Gomes & Oliveira).
//!
//! # 왜 이걸 하는가
//!
//! 순차 배치는 **순서가 결과를 결정**하는데, 좋은 순서를 미리 알 방법이 없다.
//! Jostling 은 그 순환을 끊는다: 아무 순서로 한 번 놓고, **놓인 결과를 x 좌표로 읽어**
//! 다음 순서를 만들고, 그 순서로 다시 놓는다. 방향을 번갈아 뒤집으면 앞 패스에서
//! 오른쪽 끝으로 밀려난 조각이 다음 패스에서 **먼저** 놓일 기회를 얻는다.
//!
//! 이 마커(wsjk156)에서 탐색의 이득은 **전역 재배열**에서 온다는 것이 실측으로
//! 확인됐다(45초 탐색 시점: 조각 이동거리 중앙값 1407mm = strip 높이의 92%,
//! 94%가 100mm 이상 이동). jostling 은 매 패스가 완전히 새로운 배치라 그 전역
//! 재배열을 **구성적으로** 만든다 — 비용은 그리디 1패스뿐이다.
//!
//! # 결론 — 이 인스턴스에서 jostling 은 **이득이 없다**
//!
//! 조건: wsjk156 / 탐침 4096 / 회전 `[0]` / 초기 순서 면적 내림차순 /
//! 전략 전부 끔(그리디 + 좌하단 규칙) / 라이브러리·캐시는 한 번 만들어 공유
//! (prewarm 0.64~0.73s, 아래 시간에 **포함되지 않음**).
//!
//! | 패스 | strip(mm) | 밀도 | 누적 시간 |
//! |---:|---:|---:|---:|
//! | 1 (= 면적↓ 그리디 1패스) | 7145.4 | **71.19%** | 0.38s |
//! | 2 | 7440.3 | 68.37% | 0.77s |
//! | 4 | 7492.9 | 67.89% | 1.50s |
//! | 8 | 7857.5 | 64.74% | 3.02s |
//! | 15 | 7964.0 | 63.88% | 5.90s |
//!
//! (표는 [`JostleOrder::XMinAscending`]. 다른 규칙도 패스 1이 최선인 것은 같다.)
//!
//! **5초 예산 실측** (prewarm 0.64s 를 빼고 4.36s 를 jostling 에 준 것):
//!
//! | 규칙 | 밀도 | 돈 패스 | 최선 패스 |
//! |---|---:|---:|---:|
//! | [`JostleOrder::XMinAscending`] | 71.19% | 11 | 1 |
//! | [`JostleOrder::XMaxDescending`] | 71.19% | 10 | 1 |
//! | [`JostleOrder::Alternating`] | 71.19% | 10 | 1 |
//!
//! 기준선(NFP-LBF, 무작위 ordering 12개) **71.75% / 4.4초** 에 **-0.56%p** 로 진다.
//! 목표였던 74~76% 와는 3%p 이상 떨어져 있다. 되먹인 순서가 초기 순서를 이긴 적이
//! **한 번도 없다** — 최선은 세 규칙 모두, 모든 실행에서 패스 1이었다.
//!
//! # 왜 지는가 — 되먹임이 순서 하나로 수렴시키는데, 그 순서가 초기 순서보다 나쁘다
//!
//! 원인 가설("되먹인 x 순서는 '큰 것 먼저' 라는 사전지식을 버린다")을
//! [`JostleOrder::BandedAreaDesc`] 로 검증했다. 띠를 넓힐수록 면적 순서가 되살아나고,
//! 패스 2 이후 밀도가 실제로 올라간다(64~68% -> 70%대). 그런데 **패스 1을 넘지는
//! 못한다** — 띠를 무한대로 키우면 그건 초기 순서 자체이므로 정의상 넘을 수 없다.
//! 숫자는 [`JostleOrder::BandedAreaDesc`] 문서에 있다.
//!
//! 더 근본적인 이유는 인스턴스 성질이다. **회전이 `[0]` 하나뿐**이라 조각의 자세
//! 자유도가 없고, 156종 x 수요 1 이라 같은 조각을 여러 자리에 나눠 놓을 여지도 없다.
//! 순서를 바꿔도 조각이 들어갈 수 있는 자리 집합 자체가 거의 변하지 않는다. 반면
//! 같은 예산을 **무작위 ordering 12개**에 쓰면 서로 무관한 12개의 해를 얻어 최선을
//! 고를 수 있다 — 되먹임은 그 다양성을 스스로 없앤다.
//!
//! # 그래도 남은 쓸모 있는 사실
//!
//! 패스 1, 즉 **면적 내림차순 그리디 한 번**이 71.19% / 0.38초(+prewarm 0.64초)다.
//! 기준선은 그 위에 3.3초를 더 써서 **+0.56%p** 를 산다. 5초 예산에서 그 3.3초를
//! 무엇에 쓸지가 진짜 질문이고, 이 모듈은 "jostling 에 쓰면 안 된다" 를 답한다.
//!
//! # 위 숫자를 다시 재는 법
//!
//! 측정은 임시 통합 테스트로 했고 끝나고 지웠다. 재현하려면 `swarm-core/tests/` 에
//! 아래를 만든다(이 모듈이 아직 등록 전이면 `#[path]` 로 끌어온다).
//!
//! ```text
//! pub use swarm_core::{nfp_lbf, nfp_strategy};   // jostle.rs 의 `crate::` 경로를 푼다
//! #[path = "../src/nfp_strategy/jostle.rs"] mod jostle;
//! ```
//!
//! 그리고 `PreparedLibrary::build(&inst, &[0], 0.02)` -> `NfpLbfCache::new` ->
//! `cache.prewarm(10s)` -> `SequentialPlacer::with_item_rotations(..).with_probe_budget(4096)`
//! -> `StrategyContext::new(placer, DEFAULT_NFP_STRATEGY_CONFIG, .., Some(deadline))` 로
//! 문맥을 만든 뒤 [`jostle_with_order`] 를 규칙마다 부른다. 초기 순서는 면적 내림차순
//! (`item.shape_orig.area()` 내림차순, 동률은 id 오름차순)이고,
//! 밀도(%)는 **jagua 정의**로 잰다: 조각을 `SPProblem` 에 넣고 `fit_strip()` 한 뒤
//! `SPSolution::density`. 여기서 `SequentialPlacer::world_bounds` 의 프론티어
//! (`max_x` 최댓값)를 strip 폭으로 써서 `면적합 / (프론티어 x 높이)` 를 계산하면
//! 이 마커에서 **약 1.2%p 높게** 나온다(같은 배치가 72.37% vs 71.19%).
//! `fit_strip` 이 쓰는 폭이 `min_item_separation` 만큼 넓기 때문이다 —
//! 과제가 준 기준선 71.75% 와 비교하려면 반드시 jagua 쪽이어야 한다.
//! 시간을 규칙끼리 비교하려면 **캐시를 공유**해야 한다 — 규칙마다 새로 prewarm 하면
//! 첫 규칙만 콜드 캐시를 물고 시간이 거짓말을 한다.
//!
//! # 재현 확인 (2026-08, 독립 하네스)
//!
//! 위 표를 별도 하네스로 다시 쟀고 **strip 길이가 소수점 첫째 자리까지 전부 일치**했다
//! (규칙 3종 x 15패스 = 45개 값, 띠 2000/4000mm 포함). 사이클 지점(띠2000 7패스,
//! 띠4000 4패스)도 같다. 최선 해 3개 모두 `structure::pipeline::solution_validation_error`
//! 를 통과했고 조각 156개 / 미배치 0 / 수요 다중집합 보존을 확인했다.
//!
//! 같은 실행에서 **기준선도 직접 쟀다**(무작위 ordering 12개, 같은 캐시):
//!
//! | k (누적 best-of-k) | 밀도 | 누적 시간 |
//! |---:|---:|---:|
//! | 1 | **71.52%** | 0.37s |
//! | 6 | 71.52% | 2.32s |
//! | 7 | **71.75%** | 2.70s |
//! | 12 | 71.75% | 4.52s |
//!
//! 이게 이 모듈의 결론을 한 번 더 굳힌다: `build_orderings` 의 **무작위 ordering 하나**가
//! 0.37초에 71.52% 로, 같은 시간을 쓴 면적↓ 그리디(71.19%)보다 이미 낫다. jostling 의
//! 패스 1은 5초 예산의 출발점으로도 최선이 아니다.
//!
//! ## 출발점 탓인가, 되먹임 탓인가 — 되먹임 탓이다
//!
//! 위 표의 숫자는 전부 면적↓ 에서 출발한 것이라 "출발점이 나빠서 못 넘은 것" 이라는
//! 반론이 가능하다. 그래서 **12개 중 최선 ordering**(strip 7089.9mm = 71.75%)에서
//! 다시 출발시켰다. 10패스 strip 이력(mm):
//!
//! | 규칙 | 이력 |
//! |---|---|
//! | x_min↑ | 7090, 7339, 7397, 7454, 7455, 7478, 7726, 7627, 7518, 7617 |
//! | 번갈아 | 7090, 7339, 8238, 8451, 7423, 7388, 7989, 8130, 7275, 7450 |
//!
//! 최선은 또 패스 1이고, 되먹인 18개 패스 중 출발점(7090)을 넘은 것이 **하나도 없다**.
//! 출발점을 71.19% 에서 71.75% 로 올려도 결과가 같으므로, 지는 원인은 출발점이 아니라
//! **되먹임 자체**다. 누적 63개 패스(면적↓ 45 + 띠 9 + 최선 ordering 18 중 되먹임분)에서
//! 되먹인 순서가 출발 순서를 이긴 횟수 = 0.

use std::collections::HashSet;
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use crate::nfp_lbf::PlacedPiece;
use crate::nfp_lbf::placement::Aabb;
use crate::nfp_strategy::StrategyContext;
use crate::nfp_strategy::pipeline;
use crate::nfp_strategy::select::SelectionRule;

/// 다음 패스의 순서를 배치 결과에서 뽑아내는 규칙.
///
/// # 실측 — 세 규칙 모두 패스 1을 넘지 못한다
///
/// 조건은 모듈 문서와 같다. 패스별 밀도(%):
///
/// | 패스 | [`XMinAscending`] | [`XMaxDescending`] | [`Alternating`] |
/// |---:|---:|---:|---:|
/// | 1 | **71.19** | **71.19** | **71.19** |
/// | 2 | 68.37 | 64.56 | 68.37 |
/// | 3 | 69.00 | 69.63 | 63.98 |
/// | 4 | 67.89 | 61.42 | 63.77 |
/// | 5 | 67.46 | 69.16 | 69.99 |
/// | 8 | 64.74 | 64.38 | 61.00 |
/// | 15 | 63.88 | 69.56 | 64.44 |
///
/// 읽는 법:
///
/// * 세 규칙의 **최선값이 소수점까지 같다**(71.19%). 셋 다 패스 1이 최선이었기 때문이다.
/// * [`XMinAscending`] 은 패스가 갈수록 **단조롭게 나빠진다**(71.2 -> 63.9). 방향을
///   안 바꾸면 되먹임이 한쪽으로 밀려 계속 나빠진다.
/// * [`XMaxDescending`] 은 홀/짝이 크게 진동한다(69.6 / 61.4 / 69.2 / 62.0 …).
///   좌우 반전이 두 번이면 원래대로 돌아오는 성질이 그대로 드러난다.
/// * [`Alternating`] 은 그 중간이고 진폭이 가장 크다. 셋 중 어느 것도 추세로 개선하지
///   않는다 — 되먹임이 만드는 순서는 초기 순서 주변의 **잡음**이지 개선 방향이 아니다.
///
/// 15패스 상한에서 사이클 감지는 걸리지 않았다(세 규칙 모두 15패스를 다 돌았다).
/// 순수 x 규칙은 고정점에 안 빠진다 — 그래서 예산이 있는 한 계속 헛돈다.
///
/// [`XMinAscending`]: JostleOrder::XMinAscending
/// [`XMaxDescending`]: JostleOrder::XMaxDescending
/// [`Alternating`]: JostleOrder::Alternating
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum JostleOrder {
    /// `x_min` 오름차순 — 왼쪽에 놓인 조각부터 다시 놓는다.
    XMinAscending,
    /// `x_max` 내림차순 — 오른쪽 끝에 밀려난 조각부터 다시 놓는다(= 좌우 반전).
    XMaxDescending,
    /// 패스마다 위 둘을 번갈아 쓴다. Gomes & Oliveira 의 원래 형태다.
    #[default]
    Alternating,
    /// x 를 **띠(band) 단위로만** 보고, 같은 띠 안에서는 **면적 내림차순**으로 정렬한다.
    /// 값은 띠 폭(mm)이며 `0` 이면 [`JostleOrder::XMinAscending`] 과 같다.
    ///
    /// 위 세 규칙이 지는 이유에 대한 가설을 검증하려고 넣었다: 되먹인 x 순서는
    /// "큰 것 먼저" 라는 초기 순서의 사전지식을 통째로 버린다. 띠를 넓게 잡을수록
    /// 면적 순서에 가까워지고, 좁게 잡을수록 순수 x 순서에 가까워진다 —
    /// 두 극단 사이를 하나의 노브로 훑을 수 있다.
    ///
    /// 방향은 왼쪽부터 고정이다(반전은 이 가설과 무관하다).
    ///
    /// # 실측 — 가설은 맞았지만 그래도 패스 1을 못 넘는다
    ///
    /// 조건은 모듈 문서와 같고 상한 8패스. 패스별 밀도(%):
    ///
    /// | 패스 | 띠 없음(x_min↑) | 200mm | 500mm | 1000mm | 2000mm | 4000mm |
    /// |---:|---:|---:|---:|---:|---:|---:|
    /// | 1 | **71.19** | **71.19** | **71.19** | **71.19** | **71.19** | **71.19** |
    /// | 2 | 68.37 | 69.99 | 68.89 | 69.73 | 70.09 | 69.25 |
    /// | 3 | 69.00 | 67.67 | 69.10 | 68.28 | 69.68 | 68.95 |
    /// | 4 | 67.89 | 68.03 | 67.83 | 67.96 | 69.47 | 69.90 |
    /// | 8 | 64.74 | 67.05 | 67.96 | 70.03 | (7패스에서 사이클) | (4패스에서 사이클) |
    /// | 최저 | 64.74 | 66.80 | 67.83 | 67.96 | 69.47 | 68.95 |
    ///
    /// 띠를 넓힐수록 **패스 2 이후가 확실히 좋아진다** — 최저값이 64.74 -> 69.47% 로
    /// 5%p 가까이 올라간다. 원인 가설("x 되먹임이 면적 사전지식을 버린다")은 맞다.
    ///
    /// 그런데 **패스 1을 넘지는 못한다.** 띠를 무한대로 키우면 그건 초기 순서 자체이므로
    /// 정의상 넘을 수 없고, 실제로 띠 2000mm 는 7패스, 4000mm 는 4패스에서 사이클 감지에
    /// 걸린다(= 고정점에 도달했다). 즉 이 노브가 하는 일은 "되먹임을 껐다 켰다" 이지
    /// 초기 순서보다 나은 순서를 찾는 것이 아니다.
    ///
    /// 이 변형은 **원인 진단용**이다. 프로덕션에서 쓸 이유가 없다.
    BandedAreaDesc(u32),
}

impl JostleOrder {
    /// `derivation` 번째 순서 도출(0-기준)에서 쓸 방향. `true` 면 왼쪽부터.
    fn left_first(self, derivation: usize) -> bool {
        match self {
            JostleOrder::XMinAscending => true,
            JostleOrder::XMaxDescending => false,
            JostleOrder::Alternating => derivation % 2 == 0,
            JostleOrder::BandedAreaDesc(_) => true,
        }
    }

    /// 이 규칙이 쓸 띠 폭(mm). `0` 이면 띠를 쓰지 않는다(순수 x 순서).
    fn band_mm(self) -> f64 {
        match self {
            JostleOrder::BandedAreaDesc(mm) => mm as f64,
            _ => 0.0,
        }
    }

    /// 로그·표에 쓸 짧은 이름.
    pub fn label(self) -> String {
        match self {
            JostleOrder::XMinAscending => "x_min↑".to_string(),
            JostleOrder::XMaxDescending => "x_max↓".to_string(),
            JostleOrder::Alternating => "번갈아".to_string(),
            JostleOrder::BandedAreaDesc(mm) => format!("띠{mm}mm+면적↓"),
        }
    }
}

/// 한 번의 jostling 결과.
#[derive(Clone, Debug, Default)]
pub struct JostleOutcome {
    /// **최선** 패스의 배치. 마지막 패스가 아니다 — [`jostle`] 문서 참조.
    pub pieces: Vec<PlacedPiece>,
    /// 최선 패스에서 놓지 못한 `sequence` 인덱스.
    pub unplaced: Vec<usize>,
    /// 최선 패스의 strip 길이(mm).
    pub strip_length: f64,
    /// 실제로 돈 패스 수.
    pub passes_run: usize,
    /// 패스별 strip 길이(mm) — 수렴을 보려면 이게 필요하다.
    pub history: Vec<f64>,
    /// 패스가 끝난 시점의 **누적** 경과 시간(ms). `history` 와 인덱스가 1:1 이다.
    ///
    /// 스펙에 없는 항목이지만 이게 없으면 "패스 N까지 몇 초" 를 재려고 같은 실행을
    /// N번 반복해야 하고, 그러면 캐시가 데워져 시간이 거짓말을 한다.
    pub elapsed_ms: Vec<u128>,
    /// 패스별 **미배치 조각 수**. `history` 와 인덱스가 1:1 이다.
    ///
    /// 없으면 패스별 밀도를 못 잰다. 밀도는 `놓인 조각 면적 합 / (strip x 높이)` 인데
    /// `history` 의 strip 길이만 보면 "조각을 덜 놓아서 짧은" 패스와 "잘 놓아서 짧은"
    /// 패스가 구별되지 않고, 전자를 밀도로 환산하면 **거짓으로 높게** 나온다.
    /// 예산이 패스 도중에 끊기면 실제로 그 상황이 생긴다.
    pub unplaced_history: Vec<usize>,
}

impl JostleOutcome {
    /// 최선이 나온 패스 번호(1-기준). 패스를 하나도 못 돌았으면 `None`.
    ///
    /// `(미배치 수, strip 길이)` 사전식으로 **가장 좋은 첫 패스**를 찾는다.
    /// strip 길이만 비교하면 미배치가 남아 짧아진 패스가 최선으로 잡힐 수 있는데,
    /// 그건 본 루프의 비교 규칙(미배치 우선)과 어긋나 진단이 거짓말을 한다.
    pub fn best_pass(&self) -> Option<usize> {
        let key = (self.unplaced.len(), self.strip_length);
        self.history
            .iter()
            .enumerate()
            .position(|(i, l)| {
                let u = self.unplaced_history.get(i).copied().unwrap_or(0);
                u == key.0 && (*l - key.1).abs() <= 1.0e-6
            })
            .map(|i| i + 1)
    }
}

/// 순서를 되먹여 반복 배치한다. 사이클이 잡히거나 예산이 끊기면 조기 종료한다.
///
/// (명세 초안은 "개선이 없으면 조기 종료" 라고 썼지만 그건 구현하지 않았다 —
/// jostling 은 단조 개선이 아니라서 한 번 나빠졌다고 멈추면 그 뒤의 좋은 패스를
/// 통째로 버린다. 실제로 [`JostleOrder::XMaxDescending`] 은 홀/짝으로 진동해
/// 나쁜 패스 바로 다음이 그 실행의 두 번째로 좋은 패스인 경우가 있다.
/// 종료 조건은 사이클 감지와 예산 둘뿐이고, 최선 해는 따로 보관하므로 안전하다.)
///
/// 순서 도출은 [`JostleOrder::Alternating`] 이다. 다른 규칙은 [`jostle_with_order`].
///
/// # 쓰기 전에
///
/// **wsjk156 에서는 이 함수를 부르지 마라.** 패스 1(= 초기 순서 그리디)이 항상 최선이라
/// 2패스 이후는 전부 낭비다(모듈 문서의 실측표). `max_passes = 1` 로 부르는 것은
/// `place_with_strategies` 를 직접 부르는 것과 같으므로 그럴 바엔 그쪽을 부르는 게 낫다.
/// 회전 자유도가 있거나 수요가 큰 다른 인스턴스에서는 결론이 다를 수 있고, 그때
/// 다시 재라고 남겨 둔 코드다.
///
/// # 반드시 알아야 할 것
///
/// * **jostling 은 단조 개선이 아니다.** 매 패스의 결과를 비교해 가장 좋은 것을 따로
///   보관하고 그것을 돌려준다. 마지막 패스가 최선이라고 가정하면 안 된다.
/// * 같은 (방향, 순서) 가 다시 나오면 그 뒤는 결정론적 반복이므로 **즉시 멈춘다**.
///   그게 없으면 남은 예산을 같은 배치를 다시 만드는 데 태운다.
/// * 예산([`StrategyContext::has_budget`])은 **패스 시작 전에** 본다. 패스 도중에
///   예산이 끊기면 미배치가 잔뜩 생기는데, 그런 패스는 비교 규칙(미배치 우선)에서
///   자동으로 진다.
pub fn jostle(
    ctx: &mut StrategyContext<'_, '_>,
    initial_sequence: &[usize],
    max_passes: usize,
    rule: SelectionRule,
) -> JostleOutcome {
    jostle_with_order(
        ctx,
        initial_sequence,
        max_passes,
        rule,
        JostleOrder::default(),
    )
}

/// 순서 도출 규칙을 지정하는 [`jostle`].
pub fn jostle_with_order(
    ctx: &mut StrategyContext<'_, '_>,
    initial_sequence: &[usize],
    max_passes: usize,
    rule: SelectionRule,
    order: JostleOrder,
) -> JostleOutcome {
    let mut outcome = JostleOutcome::default();
    if initial_sequence.is_empty() || max_passes == 0 {
        return outcome;
    }

    // ⑥ best-fit 은 **순서를 쓰지 않는다**(그게 best-fit 의 정의다). 켜진 채로 돌면
    // 모든 패스가 같은 배치를 내놓아 jostling 이 통째로 무의미해진다 — 예산만 태운다.
    // 그래서 이 함수가 도는 동안만 끄고 원래대로 되돌린다.
    let saved_best_fit = ctx.cfg.best_fit_selection;
    if saved_best_fit {
        ctx.cfg.best_fit_selection = false;
    }
    // ⑧ 누적 배치도 같은 이유로 끈다 — `sequence` 를 다중집합으로만 보므로 순서를
    // 되먹여도 매 패스 같은 배치가 나온다. 게다가 `placement_strategy` 에서 ⑧이
    // ⑥보다 먼저 잡히므로, ⑥만 꺼서는 jostling 이 여전히 순서를 쓰지 않는다.
    let saved_accretion = ctx.cfg.accretion;
    if saved_accretion {
        ctx.cfg.accretion = false;
    }
    // `place_with_strategies` 는 선택 규칙을 `cfg.contact_maximization` 에서 다시
    // 유도한다. 호출부가 넘긴 `rule` 이 조용히 무시되면 측정이 거짓말을 하므로 맞춘다.
    let saved_contact = ctx.cfg.contact_maximization;
    ctx.cfg.contact_maximization = matches!(rule, SelectionRule::MaxContact);

    let mut sequence: Vec<usize> = initial_sequence.to_vec();
    // 조각 면적표. [`JostleOrder::BandedAreaDesc`] 만 쓰지만, 매 패스 다시 물으면
    // 오라클을 조각 수만큼 더 두드리므로 한 번만 만든다. 회전 0 의 면적을 쓴다 —
    // 이 마커는 회전이 [0] 하나뿐이고, 여러 회전이 있어도 면적은 회전 불변이다.
    let areas: Vec<f64> = {
        let max_id = initial_sequence.iter().copied().max().unwrap_or(0);
        (0..=max_id)
            .map(|id| {
                let rot = ctx.rotations_for(id).first().copied().unwrap_or(0);
                ctx.placer
                    .shape_bounds_area(id, rot)
                    .map(|(_, a)| a)
                    .unwrap_or(0.0)
            })
            .collect()
    };
    // 이미 본 (방향, 순서) 지문. 다시 나오면 그 뒤는 결정론적 반복이다.
    //
    // **초기 상태도 넣는다.** 안 넣으면 되먹임이 초기 순서로 되돌아왔을 때 그것을
    // 사이클로 못 알아보고 패스를 통째로 하나 더 태운다(패스당 0.35~0.45초 —
    // 5초 예산의 8%다). 넣어 두면 그 자리에서 멈춘다.
    let mut seen: HashSet<u64> = HashSet::new();
    seen.insert(fingerprint(&sequence, order.left_first(0)));

    // 다중집합 검사용 기준. 되먹인 순서가 조각을 잃거나 중복시키면 수요가 어긋난
    // 배치가 만들어지므로 매 패스 확인한다(156개 정렬 = 마이크로초, 패스 0.4초 대비 무시).
    let mut want_multiset: Vec<usize> = initial_sequence.to_vec();
    want_multiset.sort_unstable();

    // 최선 = (미배치 수, strip 길이, 조각, 미배치 자리).
    let mut best: Option<(usize, f64, Vec<PlacedPiece>, Vec<usize>)> = None;
    let started = jagua_rs::Instant::now();

    for pass in 0..max_passes {
        if !ctx.has_budget() {
            break;
        }

        let placement = pipeline::place_with_strategies(ctx, &sequence, &[]);
        outcome.passes_run = pass + 1;
        outcome.history.push(placement.strip_length);
        outcome.elapsed_ms.push(started.elapsed().as_millis());
        outcome.unplaced_history.push(placement.unplaced.len());

        // 미배치가 적은 쪽이 **먼저**. 미배치가 있으면 그건 애초에 온전한 해가 아니므로
        // strip 길이보다 우선한다.
        let is_better = match &best {
            None => true,
            Some((u, l, _, _)) => {
                placement.unplaced.len() < *u
                    || (placement.unplaced.len() == *u && placement.strip_length < *l)
            }
        };

        // 다음 순서는 **최선이 아니라 방금 만든 배치**에서 뽑는다. 최선에서만 뽑으면
        // 같은 순서에 갇혀 jostling 이 한 걸음도 못 나간다.
        let derived = if pass + 1 < max_passes {
            let boxes = ctx.placer.world_bounds(&placement.pieces);
            Some(derive_sequence(
                &placement.pieces,
                &boxes,
                &placement.unplaced,
                &sequence,
                order.left_first(pass),
                order.band_mm(),
                &areas,
            ))
        } else {
            None
        };

        if is_better {
            best = Some((
                placement.unplaced.len(),
                placement.strip_length,
                placement.pieces,
                placement.unplaced,
            ));
        }

        let Some(next) = derived else { break };
        // 순서가 조각을 잃거나 늘렸으면 되먹임이 깨진 것이다 — 그 순서로 더 가면
        // 수요가 어긋난 배치를 만든다. 지금까지의 최선을 들고 멈춘다.
        //
        // 길이만 보면 **부족하다**: 한 조각이 중복되고 다른 하나가 빠져도 길이는 같은데,
        // 그 순서로 배치하면 수요를 넘긴 조각이 생겨 `materialize` 단계에서 조용히
        // 버려진다(= 조각이 사라진 해). 다중집합으로 봐야 그게 잡힌다.
        let mut got = next.clone();
        got.sort_unstable();
        if got != want_multiset {
            log::warn!(
                "[JOSTLE] 되먹인 순서의 다중집합이 어긋났다 (길이 {} -> {}) — 중단한다",
                sequence.len(),
                next.len()
            );
            break;
        }
        // 사이클 감지. 다음 패스가 쓸 (방향, 순서) 를 지문으로 남긴다.
        if !seen.insert(fingerprint(&next, order.left_first(pass + 1))) {
            break;
        }
        sequence = next;
    }

    ctx.cfg.best_fit_selection = saved_best_fit;
    ctx.cfg.accretion = saved_accretion;
    ctx.cfg.contact_maximization = saved_contact;

    if let Some((_, length, pieces, unplaced)) = best {
        outcome.strip_length = length;
        outcome.pieces = pieces;
        outcome.unplaced = unplaced;
    }
    outcome
}

/// 배치 결과에서 다음 순서를 뽑는다.
///
/// * `left_first` 면 `x_min` 오름차순, 아니면 `x_max` 내림차순.
/// * 놓지 못한 조각은 **맨 앞**에 둔다 — 밀려난 조각에 먼저 고를 기회를 주는 것이
///   jostling 의 목적이다.
/// * 완전 동률은 배치 순서(= 인덱스)로 깬다. 배치가 실행마다 흔들리면 회귀 측정이
///   불가능해지기 때문이다.
///
/// 반환 순서의 다중집합은 `pieces + unplaced` 의 item_id 다. 호출부가 길이로 검사한다.
fn derive_sequence(
    pieces: &[PlacedPiece],
    boxes: &[Aabb],
    unplaced: &[usize],
    prev_sequence: &[usize],
    left_first: bool,
    band_mm: f64,
    areas: &[f64],
) -> Vec<usize> {
    let n = pieces.len().min(boxes.len());
    // 띠 번호. 띠가 없으면 전부 0이라 정렬이 순수 x 순서로 내려간다.
    let banded = band_mm.is_finite() && band_mm > 0.0;
    let band_of = |b: &Aabb| -> i64 {
        if !banded {
            return 0;
        }
        let x = if left_first { b.min_x } else { -b.max_x };
        if x.is_finite() {
            (x / band_mm).floor() as i64
        } else {
            i64::MAX
        }
    };
    let area_of = |i: usize| -> f64 {
        pieces
            .get(i)
            .and_then(|p| areas.get(p.item_id))
            .copied()
            .unwrap_or(0.0)
    };

    let mut idx: Vec<usize> = (0..n).collect();
    idx.sort_by(|&a, &b| {
        let (Some(ba), Some(bb)) = (boxes.get(a), boxes.get(b)) else {
            return a.cmp(&b);
        };
        // 띠가 켜져 있으면 x 는 띠 단위로만 보고, 같은 띠 안은 면적 내림차순이다.
        if banded {
            return band_of(ba)
                .cmp(&band_of(bb))
                .then(area_of(b).total_cmp(&area_of(a)))
                .then(a.cmp(&b));
        }
        // `total_cmp` 를 쓴다 — `partial_cmp` 는 NaN 에서 `None` 이라 정렬이 전순서가
        // 아니게 되고, 그러면 std 가 "does not implement a total order" 로 panic 한다.
        let primary = if left_first {
            ba.min_x
                .total_cmp(&bb.min_x)
                .then(ba.min_y.total_cmp(&bb.min_y))
        } else {
            // 내림차순 — 인자를 뒤집는다.
            bb.max_x
                .total_cmp(&ba.max_x)
                .then(bb.max_y.total_cmp(&ba.max_y))
        };
        primary.then(a.cmp(&b))
    });

    let mut out: Vec<usize> = Vec::with_capacity(prev_sequence.len());
    // 미배치 자리의 item_id 를 먼저. `get` 으로만 읽어 인덱스가 어긋나도 죽지 않는다.
    for &slot in unplaced {
        if let Some(&id) = prev_sequence.get(slot) {
            out.push(id);
        }
    }
    for i in idx {
        if let Some(p) = pieces.get(i) {
            out.push(p.item_id);
        }
    }
    out
}

/// 순서와 방향을 하나의 지문으로. 사이클 감지에 쓴다.
///
/// 방향을 함께 넣는 이유: [`JostleOrder::Alternating`] 에서는 **같은 순서라도 다음에
/// 적용할 방향이 다르면 다음 패스의 결과가 다르다**. 순서만 해시하면 아직 탐색하지
/// 않은 가지를 사이클로 오인해 예산을 버린다.
fn fingerprint(sequence: &[usize], left_first: bool) -> u64 {
    let mut h = DefaultHasher::new();
    left_first.hash(&mut h);
    sequence.hash(&mut h);
    h.finish()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn piece(item_id: usize, x: f32) -> PlacedPiece {
        PlacedPiece {
            item_id,
            rotation: 0,
            translation: (x, 0.0),
        }
    }

    fn bbox(min_x: f64, max_x: f64) -> Aabb {
        Aabb {
            min_x,
            min_y: 0.0,
            max_x,
            max_y: 10.0,
        }
    }

    #[test]
    fn left_first_reads_the_layout_from_the_left() {
        // 배치 순서는 2,0,1 인데 x 로는 0,1,2 다. 되먹인 순서가 x 를 따라야 한다.
        let pieces = vec![piece(2, 20.0), piece(0, 0.0), piece(1, 10.0)];
        let boxes = vec![bbox(20.0, 25.0), bbox(0.0, 5.0), bbox(10.0, 15.0)];
        let out = derive_sequence(&pieces, &boxes, &[], &[2, 0, 1], true, 0.0, &[]);
        assert_eq!(out, vec![0, 1, 2]);
    }

    #[test]
    fn right_first_is_the_mirror_image() {
        let pieces = vec![piece(2, 20.0), piece(0, 0.0), piece(1, 10.0)];
        let boxes = vec![bbox(20.0, 25.0), bbox(0.0, 5.0), bbox(10.0, 15.0)];
        let out = derive_sequence(&pieces, &boxes, &[], &[2, 0, 1], false, 0.0, &[]);
        assert_eq!(out, vec![2, 1, 0], "오른쪽 끝 조각이 먼저 와야 한다");
    }

    #[test]
    fn unplaced_pieces_get_the_first_pick_and_the_multiset_is_preserved() {
        // sequence = [7, 3, 9]. 3만 놓였고 자리 0(=7)과 2(=9)가 미배치다.
        let prev = vec![7usize, 3, 9];
        let pieces = vec![piece(3, 0.0)];
        let boxes = vec![bbox(0.0, 5.0)];
        let out = derive_sequence(&pieces, &boxes, &[0, 2], &prev, true, 0.0, &[]);
        assert_eq!(out, vec![7, 9, 3], "미배치가 앞");
        let (mut a, mut b) = (out.clone(), prev.clone());
        a.sort_unstable();
        b.sort_unstable();
        assert_eq!(a, b, "다중집합이 보존돼야 수요가 어긋나지 않는다");
    }

    #[test]
    fn ties_are_broken_deterministically_by_placement_index() {
        // x 가 완전히 같은 세 조각. 실행마다 순서가 흔들리면 회귀 측정이 불가능하다.
        let pieces = vec![piece(5, 0.0), piece(1, 0.0), piece(3, 0.0)];
        let boxes = vec![bbox(0.0, 5.0), bbox(0.0, 5.0), bbox(0.0, 5.0)];
        for _ in 0..8 {
            assert_eq!(
                derive_sequence(&pieces, &boxes, &[], &[5, 1, 3], true, 0.0, &[]),
                vec![5, 1, 3]
            );
        }
    }

    #[test]
    fn a_non_finite_coordinate_does_not_panic_the_sort() {
        // `partial_cmp` 를 쓰면 여기서 std 가 전순서 위반으로 panic 한다.
        let pieces = vec![piece(0, 0.0), piece(1, 0.0), piece(2, 0.0)];
        let boxes = vec![
            bbox(f64::NAN, 5.0),
            bbox(0.0, f64::NAN),
            bbox(10.0, 15.0),
        ];
        let out = derive_sequence(&pieces, &boxes, &[], &[0, 1, 2], true, 0.0, &[]);
        assert_eq!(out.len(), 3);
    }

    #[test]
    fn alternating_flips_direction_every_derivation() {
        let a = JostleOrder::Alternating;
        assert!(a.left_first(0));
        assert!(!a.left_first(1));
        assert!(a.left_first(2));
        assert!(JostleOrder::XMinAscending.left_first(1));
        assert!(!JostleOrder::XMaxDescending.left_first(0));
    }

    #[test]
    fn the_fingerprint_separates_the_same_order_taken_in_two_directions() {
        // 방향을 빼면 아직 안 가 본 가지를 사이클로 오인한다.
        let s = [1usize, 2, 3];
        assert_ne!(fingerprint(&s, true), fingerprint(&s, false));
        assert_eq!(fingerprint(&s, true), fingerprint(&s, true));
        assert_ne!(fingerprint(&s, true), fingerprint(&[1, 3, 2], true));
    }

    /// [`jostle_with_order`] 의 최선 갱신 조건을 그대로 떼어 낸 것.
    ///
    /// 본 함수는 오라클(`SequentialPlacer`)이 있어야 돌아가서 단위 테스트로 못 부른다.
    /// 그런데 이 조건이 뒤집히면 "미배치가 많은데 짧은" 배치가 이겨 **해가 아닌 것을
    /// 최선으로 들고 나온다** — 조용히 틀리는 종류라 따로 못 박아 둔다.
    fn is_better(cand: (usize, f64), best: Option<(usize, f64)>) -> bool {
        match best {
            None => true,
            Some((u, l)) => cand.0 < u || (cand.0 == u && cand.1 < l),
        }
    }

    #[test]
    fn fewer_unplaced_beats_a_shorter_strip() {
        // 미배치 3개짜리 3000mm 배치는 미배치 0개짜리 9000mm 배치를 이기면 안 된다.
        assert!(!is_better((3, 3000.0), Some((0, 9000.0))));
        assert!(is_better((0, 9000.0), Some((3, 3000.0))));
        // 미배치가 같으면 짧은 쪽.
        assert!(is_better((0, 8999.0), Some((0, 9000.0))));
        assert!(!is_better((0, 9000.0), Some((0, 9000.0))), "동률은 앞 패스 유지");
        assert!(is_better((0, 9000.0), None));
    }

    #[test]
    fn best_pass_reports_the_first_pass_that_hit_the_best_length() {
        let mut o = JostleOutcome {
            history: vec![100.0, 90.0, 95.0, 90.0],
            unplaced_history: vec![0, 0, 0, 0],
            strip_length: 90.0,
            ..Default::default()
        };
        assert_eq!(o.best_pass(), Some(2));
        o.history.clear();
        assert_eq!(o.best_pass(), None);
    }

    #[test]
    fn best_pass_does_not_point_at_a_pass_that_was_short_because_it_dropped_pieces() {
        // 패스 1은 미배치 5개로 3000mm, 패스 2는 온전한 해로 9000mm.
        // strip 길이만 보면 패스 1이 최선으로 잡히는데 그건 해가 아니다.
        let o = JostleOutcome {
            history: vec![3000.0, 9000.0],
            unplaced_history: vec![5, 0],
            strip_length: 9000.0,
            unplaced: Vec::new(),
            ..Default::default()
        };
        assert_eq!(o.best_pass(), Some(2));
    }
}
