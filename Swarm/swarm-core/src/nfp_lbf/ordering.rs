//! Multiple Item Ordering — 배치 순서 후보 생성.
//!
//! > **배치 순서(order)가 엄청 중요하다.** 같은 NFP-LBF라도:
//!
//! ```text
//! Area descending
//! Max dimension descending
//! Convex hull area descending
//! Concavity descending
//! Random
//! Custom garment order
//! ```
//!
//! 에 따라 결과가 크게 달라진다. 한 번만 LBF를 돌리지 않고:
//!
//! ```text
//! Order #1 → NFP-LBF → 82.1%
//! Order #2 → NFP-LBF → 83.4%
//! Order #3 → NFP-LBF → 81.8%
//! ```
//!
//! 처럼 **여러 ordering을 매우 빠르게 평가**하고 최상의 것을 초기해로 쓴다.
//!
//! 또한 처음부터 100개의 완전 랜덤 ordering을 만드는 대신:
//!
//! ```text
//! Base ordering
//!     ↓
//! NFP-LBF
//!     ↓
//! 좋은 결과 확보
//!
//! 이후
//! Swap 2 items
//! Move 1 item
//! Block reorder
//!     ↓
//! 부분적으로 달라진 suffix만 재배치
//! ```
//!
//! 하도록 만들면 훨씬 빨라진다. 가령 156개 중 80번째 이후 순서만 바뀌었다면 앞의 79개
//! 배치를 그대로 재사용할 수 있다.
//!
//! 그래서 이 모듈의 이웃 연산은 변경 지점을 기본적으로 `len/2 ..= len-1` 구간에서 뽑는다.
//! 공통 prefix가 길수록 [`ItemOrdering::common_prefix_len`]이 크게 나오고, 그만큼
//! `SequentialPlacer`가 재사용할 수 있는 배치가 늘어난다.
//!
//! # 다만 그건 prefix 재사용을 쓰는 경로에서만 이득이다
//!
//! base ordering이 면적 내림차순이므로 `len/2` 뒤쪽은 **가장 작은 조각들**이다. 즉
//! strip 길이를 지배하는 큰 조각의 순서가 파생 ordering 내내 동일하다 — 다양성이
//! 사실상 없다. 반면 전략 경로(`nfp_strategy`: 빔·best-fit·슬라이딩)는 "앞 k개를
//! 그대로 둔다"가 성립하지 않아 prefix 재사용을 **아예 쓰지 않는다**. 거기서는
//! 하한을 0으로 내려도 잃는 게 없다.
//!
//! 그래서 하한과 연산 혼합비를 [`OrderingDiversity`]로 뽑아 두고,
//! [`build_orderings`]는 기존 동작([`OrderingDiversity::PREFIX_REUSE`])을 그대로 유지하는
//! 얇은 래퍼로 남겼다. 하한을 바꾸려면 [`build_orderings_with`]를 쓴다.
//!
//! ## 실측 (여성WSjk 156-패널, 탐침 4096, ordering 50개 = 결정 6 + 파생 44, seed 0~8)
//!
//! `파생best평균` 은 seed 마다 파생 44개 중 최고를 고른 뒤 9 seed 평균이다. 결정론
//! 6종의 best 는 어떤 arm 에서도 **7112.8mm (밀도 71.52%, area-desc)** 로 같다.
//!
//! `표준편차`·`최대-최소`는 **seed 안에서** 파생 44개의 흩어짐을 재서 9 seed 평균낸
//! 값이다(seed 를 섞어 재면 seed 간 편차까지 섞여 다양성을 못 본다).
//!
//! ```text
//! 하한   혼합[swap,move,block]  앞절반diff  파생best평균   최고(밀도)      파생평균  표준편차  최대-최소
//! len/2      [1,1,1] (기존)       0 / 396      7085.8   7062.7 (72.03%)   7153.0     47.2    233.7
//! 0          [1,1,1]            303 / 396      7079.6   7061.3 (72.04%)   7201.3     99.1    516.9
//! 0          [1,0,0] swap 만    302 / 396      7085.2   7066.5 (71.99%)   7201.2     83.5    396.3
//! 0          [0,1,0] move 만    302 / 396      7075.4   7018.9 (72.48%)   7175.6     66.6    335.4
//! 0          [0,0,1] block 만   332 / 396      7089.7   7042.4 (72.24%)   7230.9    106.6    557.3
//! 0          [1,2,1]  <- 채택   303 / 396      7074.6   7018.9 (72.48%)   7190.8     77.7    396.9
//! 0.25       [1,1,1]            238 / 396      7087.1   7069.9 (71.95%)   7186.1     77.9    384.0
//! ```
//!
//! 읽는 법 — **정직하게 말하면 앞부분 순서는 품질에 거의 영향이 없다.**
//!
//! * 다양성은 확실히 늘었다. 앞쪽 절반이 base와 다른 파생이 `0/396 → 300/396`.
//!   기존 하한에서는 파생 44개가 **큰 조각 순서 기준으로 전부 같은 해**였다.
//! * 그런데 품질은 거의 안 움직인다. 채택안이 기존 대비 11.2mm(0.16%) 나을 뿐이고
//!   `[1,0,0]`·`[0,0,1]`·`0.25` 는 기존과 동률이거나 더 나쁘다.
//! * seed 를 나눠 보면 순위가 뒤집힌다. seed 0~5 에서는 `[1,2,1]` 7066.7 / 기존
//!   7085.3 이었는데, seed 6~8 만 보면 `[1,2,1]` 7090.4 / 기존 7086.8 로 역전된다.
//!   즉 arm 사이 차이(약 10mm)는 **seed 분산(약 20mm)보다 작다.** 9 seed 를 모아야
//!   겨우 방향이 보이는 크기다.
//! * 파생 **평균**은 나빠지고(7153 → 7176~7231) 흩어짐은 커진다(표준편차 47 → 67~107).
//!   큰 조각을 흔들면 대부분 더 나쁜 해가 되고 가끔 더 좋은 해가 나온다. 우리는 best
//!   하나만 쓰므로 흩어짐 자체는 손해가 아니지만, **덜 흩어질수록 best 가 좋다**는
//!   방향이 보인다: move 계열이 하한 0 중 표준편차가 가장 낮고(66.6/77.7) 파생best
//!   평균도 가장 좋다. block 만 쓰는 쪽은 가장 흩어지고(106.6) 가장 나쁘다(7089.7).
//!   `[1,2,1]`(move 두 배)의 근거가 이 열이다 — 한 조각만 옮기는 move 가 가장 얕은
//!   교란이다.
//! * 그러니 이 변경의 값어치는 "품질이 오른다"가 아니라 **"prefix 재사용을 어차피 못
//!   쓰는 경로에서 공짜로 분산을 산다"** 다. 유일하게 눈에 띄는 숫자는 단일 최고해로,
//!   기존 하한이 9 seed 를 통틀어 한 번도 못 닿은 7018.9mm(72.48%)에 하한 0의
//!   move 계열이 닿았다는 것뿐이다(기존 최고 7062.7 = 72.03%).
//!
//! ## 정작 쓸 경로에서 재면 — 전략 경로 A/B (②+③+④+⑦, ordering 12개, seed 0~8)
//!
//! 위 표는 전략을 다 끈 그리디다. [`OrderingDiversity::FULL_RANGE`]가 실제로 쓰일 곳은
//! 빔·best-fit·슬라이딩이 도는 경로이므로 거기서 다시 쟀다(`strategy_path_diversity_ab`).
//! `search.rs` 배선 없이도 잴 수 있다 — 전략 파이프라인을 계측 테스트가 직접 부른다.
//!
//! ```text
//! 하한                     seed별 best 평균   9 seed 단일 최고
//! len/2 [1,1,1] (기존)     7062.4 (71.98%)   7030.1 (72.36%)
//! 0     [1,2,1] (채택)     7052.3 (72.08%)   6985.5 (72.82%)
//! ```
//!
//! * 방향은 그리디와 같다 — 채택안이 **10.1mm(0.10%p)** 앞선다. 크기까지 비슷하다
//!   (그리디 11.2mm). 두 경로가 독립적으로 같은 답을 낸 셈이다.
//! * 그런데 여전히 **seed 분산 안이다.** seed 0~2 만 보면 기존 7046.4 / 채택 7061.4 로
//!   기존이 이기고, seed 3~8 에서는 기존 7070.4 / 채택 7047.7 로 뒤집힌다. 3 seed 로
//!   결론을 내면 안 된다는 증거가 여기도 있다.
//! * 단일 최고해만 또렷하다. 채택안이 **6985.5mm = 72.82%** 에 닿았다. 이 프로젝트가
//!   지금까지 기록한 어떤 조합보다 좋은 값이고(최고 조합 기준선 72.18%), 기존 하한은
//!   9 seed 를 통틀어 72.36% 가 최고였다. ordering 50개 중 best 하나만 쓰는 우리 용법
//!   에서는 이 열이 실제 이득에 가장 가깝다.
//!
//! 숫자는 `diversity_measurement` / `strategy_path_diversity_ab` 계측 테스트가 찍는다.

use std::collections::HashSet;

use jagua_rs::geometry::convex_hull::convex_hull_from_points;
use jagua_rs::geometry::primitives::Point;
use jagua_rs::probs::spp::entities::SPInstance;
use rand::RngExt;
use rand::rngs::Xoshiro256PlusPlus;
use rand::seq::SliceRandom;

/// 문서가 나열한 ordering 전략.
///
/// ```text
/// Area descending
/// Max dimension descending
/// Convex hull area descending
/// Concavity descending
/// Random
/// Custom garment order
/// ```
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum OrderingStrategy {
    /// `item.shape_cd.area` 내림차순.
    AreaDescending,
    /// `max(bbox.width(), bbox.height())` 내림차순.
    MaxDimensionDescending,
    /// convex hull 면적 내림차순.
    ConvexHullAreaDescending,
    /// `convex hull 면적 / 실제 면적` 내림차순 (클수록 오목).
    ConcavityDescending,
    /// 셔플. base ordering에서 파생된 이웃 해도 이 전략으로 표기한다.
    Random,
    /// 호출자가 지정한 의류 재단 순서.
    CustomGarment,
    /// 조각 난이도 사전분류 — hard -> medium -> easy.
    ///
    /// 직경 계수(`diam_factor`)로 난이도 점수를 매겨 세 등급으로 나누고, 어려운
    /// 조각을 먼저 배치한다. 배치가 실패할 조각이라면 앞쪽에서 드러나야 실패 비용이
    /// 싸다 — 뒤쪽에서 드러나면 이미 배치한 조각들이 전부 헛수고가 된다.
    DifficultyTiered,
}

/// [`OrderingStrategy::DifficultyTiered`]의 등급 경계.
///
/// ```text
/// cp->diam_factor      난이도 점수에서 직경이 갖는 비중
/// cp->hard_percent     상위 몇 %를 hard 로 볼지
/// cp->medium_percent   그 다음 몇 %를 medium 으로 볼지
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DifficultyTiers {
    /// 난이도 점수에서 직경(최대 지름)이 갖는 가중치. 나머지는 오목도가 갖는다.
    pub diam_factor: f32,
    /// hard 등급 비율 (0.0~1.0).
    pub hard_percent: f32,
    /// medium 등급 비율 (0.0~1.0). 나머지가 easy 다.
    pub medium_percent: f32,
}

impl Default for DifficultyTiers {
    fn default() -> Self {
        Self {
            diam_factor: DEFAULT_DIAM_FACTOR,
            hard_percent: DEFAULT_HARD_PERCENT,
            medium_percent: DEFAULT_MEDIUM_PERCENT,
        }
    }
}

/// 난이도 점수의 직경 비중 기본값.
pub const DEFAULT_DIAM_FACTOR: f32 = 0.6;
/// hard 등급 기본 비율.
pub const DEFAULT_HARD_PERCENT: f32 = 0.25;
/// medium 등급 기본 비율.
pub const DEFAULT_MEDIUM_PERCENT: f32 = 0.45;

/// 조각 난이도 등급.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum DifficultyTier {
    /// 가장 배치하기 어려운 조각. 가장 먼저 놓는다.
    Hard,
    Medium,
    Easy,
}

impl OrderingStrategy {
    /// 리포트/로그에 그대로 찍히는 짧은 이름.
    pub fn label(self) -> &'static str {
        match self {
            OrderingStrategy::AreaDescending => "area-desc",
            OrderingStrategy::MaxDimensionDescending => "maxdim-desc",
            OrderingStrategy::ConvexHullAreaDescending => "hull-area-desc",
            OrderingStrategy::ConcavityDescending => "concavity-desc",
            OrderingStrategy::Random => "random",
            OrderingStrategy::CustomGarment => "custom-garment",
            OrderingStrategy::DifficultyTiered => "difficulty-tiered",
        }
    }
}

/// 하나의 배치 순서 후보.
///
/// `sequence`는 **배치 대상 폴리곤의 나열**이다. 아이템 종류마다 수요(qty)가 여러 개일
/// 수 있으므로 `instance.items`의 `(Item, qty)`를 펼쳐 `item_id`를 qty만큼 반복해 넣는다.
/// 따라서 `sequence.len() == instance.total_item_qty()`이며 같은 `item_id`가 여러 번
/// 나타날 수 있다.
#[derive(Clone, Debug)]
pub struct ItemOrdering {
    /// 이 순서를 만든 전략. 파생 이웃 해는 [`OrderingStrategy::Random`]이다.
    pub strategy: OrderingStrategy,
    /// 배치할 `item_id`의 나열 (수요만큼 반복됨).
    pub sequence: Vec<usize>,
    /// 사람이 읽는 이름. 결정론적 6종은 `strategy.label()`을 그대로 쓰고,
    /// 파생 이웃 해는 `"swap@{i}"`, `"move@{i}"`, `"block@{i}"` 형태다.
    pub label: String,
}

impl ItemOrdering {
    /// 두 sequence가 앞에서부터 몇 개나 일치하는지 돌려준다.
    ///
    /// suffix만 재배치하는 prefix 재사용의 핵심이다. 156개 중 80번째 이후 순서만
    /// 바뀌었다면 이 값이 79가 되고, 앞의 79개 배치를 그대로 재사용할 수 있다.
    pub fn common_prefix_len(&self, other: &ItemOrdering) -> usize {
        self.sequence
            .iter()
            .zip(other.sequence.iter())
            .take_while(|(a, b)| a == b)
            .count()
    }
}

/// 파생 ordering의 다양성 노브 — 변형 위치의 하한과 연산 혼합비.
///
/// prefix 재사용을 쓰는 경로와 쓰지 않는 경로가 원하는 값이 정반대라서 분리했다.
/// 자세한 배경은 모듈 문서 참고.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OrderingDiversity {
    /// 변형 시작 지점의 하한 비율. 하한 = `floor(len * ratio)`.
    ///
    /// * `0.5` — 뒤쪽 절반만 건드린다. 공통 prefix가 최소 `len/2` 보장되어
    ///   `SequentialPlacer`의 prefix 재사용이 매번 걸린다.
    /// * `0.0` — 전 구간. 큰 조각의 순서까지 바뀌므로 다양성이 실제로 생긴다.
    pub mutation_lower_ratio: f32,
    /// `[swap, move, block]` 연산의 정수 가중치. 이 비율대로 순환하며 뽑는다.
    ///
    /// 전부 0이면 `[1, 1, 1]`로 되돌린다(파생을 아예 못 만드는 상태를 막는다).
    pub op_weights: [u8; 3],
}

impl OrderingDiversity {
    /// 기존 동작 — 하한 `len/2`, 세 연산 균등 순환.
    ///
    /// `SequentialPlacer`의 prefix 재사용을 쓰는 경로(`search.rs`의 비-전략 경로,
    /// `exploration.rs`)는 **반드시** 이 값을 써야 한다. 한 비트도 달라지면 안 된다.
    pub const PREFIX_REUSE: Self = Self {
        mutation_lower_ratio: 0.5,
        op_weights: [1, 1, 1],
    };

    /// 전 구간 변형 — prefix 재사용을 쓰지 않는 전략 경로용.
    ///
    /// 혼합비 `[1, 2, 1]`(move 를 두 배)은 실측으로 고른 값이다. 156-패널 9 seed 평균
    /// 파생 best 7074.6mm 로, 기존 하한(7085.8)·균등 `[1,1,1]`(7079.6)·swap 만
    /// `[1,0,0]`(7085.2)·block 만 `[0,0,1]`(7089.7) 중 제일 낫다. 하한 0에서 큰 조각을
    /// **한 개만 옮기는** move 가 가장 안전한 교란이고, swap·block 은 두 자리를 동시에
    /// 흔들어 평균을 더 망친다. 그래도 둘을 0으로 죽이지는 않는다 — seed 에 따라
    /// 최고 해가 block 에서 나오기도 한다.
    ///
    /// **다만 arm 사이 차이(약 10mm = 0.15%)는 seed 분산(약 20mm)보다 작다.** 이건
    /// "이 혼합비가 옳다"는 강한 근거가 아니라 "9 seed 를 모아 보니 이쪽이 조금 앞선다"
    /// 정도다.
    ///
    /// 정작 쓸 곳인 **전략 경로(②+③+④+⑦)에서 다시 재도 결론은 같다** — 9 seed
    /// 평균 7052.3 대 기존 7062.4 로 10.1mm(0.10%p) 앞서고, 그 차이는 여전히 seed
    /// 분산 안이다. 다만 단일 최고해는 또렷하게 좋다(6985.5mm = 72.82%, 기존 하한의
    /// 9 seed 최고는 72.36%). ordering 여러 개 중 best 하나만 쓰는 용법이라 이 쪽이
    /// 실제 이득에 가깝다. 자세한 표는 모듈 문서 참고.
    pub const FULL_RANGE: Self = Self {
        mutation_lower_ratio: 0.0,
        op_weights: [1, 2, 1],
    };

    /// 경로에 맞는 값 하나를 고른다 — 호출부가 한 줄로 끝나게 하려는 헬퍼다.
    ///
    /// `strategy_path`가 참이면 (= 빔·best-fit·슬라이딩 중 하나라도 켜졌으면)
    /// prefix 재사용을 못 쓰므로 하한을 낮춰 다양성을 공짜로 가져간다.
    pub fn for_strategy_path(strategy_path: bool) -> Self {
        if strategy_path {
            Self::FULL_RANGE
        } else {
            Self::PREFIX_REUSE
        }
    }

    /// 하한 인덱스. `ratio`는 `0.0..=1.0`으로 clamp 한다.
    ///
    /// `f64`로 계산하는 이유: `ratio = 0.5`일 때 결과가 정확히 `len / 2`여야
    /// 기존 경로의 동작이 보존된다(`f32`는 큰 `len`에서 반올림이 생길 수 있다).
    #[inline]
    fn lower_bound(self, len: usize) -> usize {
        let ratio = self.mutation_lower_ratio.clamp(0.0, 1.0) as f64;
        let lo = (len as f64 * ratio).floor();
        (lo as usize).min(len)
    }

    /// 가중치를 펼친 연산 순환표. `[1,1,1]`이면 `[0,1,2]` — 기존 순환과 동일하다.
    fn op_plan(self) -> Vec<usize> {
        let mut plan = Vec::new();
        for (op, &w) in self.op_weights.iter().enumerate() {
            for _ in 0..w {
                plan.push(op);
            }
        }
        if plan.is_empty() {
            plan = vec![0, 1, 2];
        }
        plan
    }
}

impl Default for OrderingDiversity {
    fn default() -> Self {
        Self::PREFIX_REUSE
    }
}

/// 정확히 `n`개(또는 가능한 만큼)의 [`ItemOrdering`]을 만든다.
///
/// 기존 시그니처 유지용 래퍼다. 파생 변형은 [`OrderingDiversity::PREFIX_REUSE`]
/// (= 하한 `len/2`)로, 즉 **이전과 완전히 동일하게** 만들어진다.
///
/// 동일함의 근거 — 결과가 달라질 수 있는 지점은 셋뿐이고 셋 다 테스트로 고정했다.
///
/// 1. 하한: `lower_bound(len)` 가 모든 `len`에서 정확히 `len / 2`
///    (`lower_bound_ratio_matches_integer_halving`).
/// 2. 연산 순서: `op_plan()` 이 `[0, 1, 2]` — 예전 `op_cycle % 3` 과 같다
///    (`op_plan_expands_weights`).
/// 3. rng 소비 순서: `pick_*` 는 `lo` 를 인자로 받게만 바뀌었고 난수 호출 순서는
///    그대로다(특히 `pick_block` 은 예전처럼 `block_len` 을 `lo` 판정보다 먼저 뽑는다).
///
/// 셋이 같으면 같은 seed 에서 같은 sequence 가 같은 순서로 나온다.
pub fn build_orderings(
    instance: &SPInstance,
    n: usize,
    rng: &mut Xoshiro256PlusPlus,
    custom: Option<&[usize]>,
) -> Vec<ItemOrdering> {
    build_orderings_with(instance, n, rng, custom, OrderingDiversity::PREFIX_REUSE)
}

/// 정확히 `n`개(또는 가능한 만큼)의 [`ItemOrdering`]을 만든다.
///
/// 앞쪽에 **결정론적 6종**을 넣고, 7번째부터는 base ordering
/// (= `AreaDescending`)에서 문서가 지정한 이웃 연산으로 파생한다.
///
/// ```text
/// Swap 2 items
/// Move 1 item
/// Block reorder
/// ```
///
/// * `custom`이 `None`이면 `CustomGarment` 항목은 건너뛴다.
/// * 중복 sequence는 제거한다(같은 순서를 두 번 평가하지 않는다).
/// * `diversity`가 파생 변형의 위치 하한과 연산 혼합비를 정한다.
///   결정론적 6종은 `diversity`와 무관하게 항상 같다.
pub fn build_orderings_with(
    instance: &SPInstance,
    n: usize,
    rng: &mut Xoshiro256PlusPlus,
    custom: Option<&[usize]>,
    diversity: OrderingDiversity,
) -> Vec<ItemOrdering> {
    let mut out: Vec<ItemOrdering> = Vec::new();
    if n == 0 {
        return out;
    }

    let metrics = collect_metrics(instance);
    if metrics.is_empty() {
        return out;
    }

    // ── 결정론적 6종 ──────────────────────────────────────────────────────
    let by_area = expand_sorted(metrics.iter().map(|m| (m.id, m.qty, m.area)).collect());
    if by_area.is_empty() {
        return out;
    }
    let by_max_dim = expand_sorted(metrics.iter().map(|m| (m.id, m.qty, m.max_dim)).collect());
    let by_hull = expand_sorted(metrics.iter().map(|m| (m.id, m.qty, m.hull_area)).collect());
    let by_concavity = expand_sorted(metrics.iter().map(|m| (m.id, m.qty, m.concavity)).collect());

    let mut seen: HashSet<Vec<usize>> = HashSet::new();
    let push = |out: &mut Vec<ItemOrdering>,
                seen: &mut HashSet<Vec<usize>>,
                strategy: OrderingStrategy,
                label: String,
                sequence: Vec<usize>| {
        if out.len() >= n || sequence.is_empty() || !seen.insert(sequence.clone()) {
            return;
        }
        out.push(ItemOrdering {
            strategy,
            sequence,
            label,
        });
    };

    let s = OrderingStrategy::AreaDescending;
    push(
        &mut out,
        &mut seen,
        s,
        s.label().to_string(),
        by_area.clone(),
    );
    let s = OrderingStrategy::MaxDimensionDescending;
    push(&mut out, &mut seen, s, s.label().to_string(), by_max_dim);
    let s = OrderingStrategy::ConvexHullAreaDescending;
    push(&mut out, &mut seen, s, s.label().to_string(), by_hull);
    let s = OrderingStrategy::ConcavityDescending;
    push(&mut out, &mut seen, s, s.label().to_string(), by_concavity);
    let s = OrderingStrategy::DifficultyTiered;
    push(
        &mut out,
        &mut seen,
        s,
        s.label().to_string(),
        difficulty_tiered_sequence(&metrics, DifficultyTiers::default()),
    );

    // Custom garment order — `custom`이 `None`이면 이 항목은 건너뛴다.
    if let Some(custom) = custom {
        let quantities: Vec<usize> = {
            let mut q = vec![0usize; metrics.iter().map(|m| m.id + 1).max().unwrap_or(0)];
            for m in &metrics {
                q[m.id] = m.qty;
            }
            q
        };
        let seq = sanitize_custom(custom, &quantities, &by_area);
        let s = OrderingStrategy::CustomGarment;
        push(&mut out, &mut seen, s, s.label().to_string(), seq);
    }

    // Random — 셔플.
    {
        let mut seq = by_area.clone();
        seq.shuffle(rng);
        let s = OrderingStrategy::Random;
        push(&mut out, &mut seen, s, s.label().to_string(), seq);
    }

    // ── 7번째 이후: base ordering에서 이웃 연산으로 파생 ────────────────────
    //
    // 변형 위치의 하한은 `diversity`가 정한다. 하한 `len/2`면 앞쪽 prefix가 그대로
    // 남아 `SequentialPlacer`가 suffix만 다시 배치할 수 있다. 하한 0이면 그 이득을
    // 포기하는 대신 큰 조각의 순서까지 흔든다.
    let base = &by_area;
    let mut attempts = 0usize;
    let max_attempts = n.saturating_sub(out.len()).saturating_mul(12) + 64;
    let mut op_cycle = 0usize;
    let op_plan = diversity.op_plan();

    while out.len() < n && attempts < max_attempts {
        attempts += 1;
        let mut seq = base.clone();
        let op = op_plan.get(op_cycle % op_plan.len()).copied().unwrap_or(0);
        op_cycle += 1;
        let lo = diversity.lower_bound(seq.len());

        let tagged = match op {
            0 => pick_swap(seq.len(), lo, rng).map(|(i, j)| {
                let at = op_swap(&mut seq, i, j);
                format!("swap@{at}")
            }),
            1 => pick_move(seq.len(), lo, rng).map(|(from, to)| {
                let at = op_move(&mut seq, from, to);
                format!("move@{at}")
            }),
            _ => pick_block(seq.len(), lo, rng).map(|(start, block_len, dest)| {
                let at = op_block(&mut seq, start, block_len, dest);
                format!("block@{at}")
            }),
        };

        let Some(label) = tagged else {
            // 이 연산은 이 길이에서 아예 불가능하다. 다음 연산으로 넘어간다.
            continue;
        };
        push(&mut out, &mut seen, OrderingStrategy::Random, label, seq);
    }

    out
}

// ────────────────────────────────────────────────────────────────────────────
// 아이템 지표
// ────────────────────────────────────────────────────────────────────────────

/// 정렬 키로 쓰는 아이템별 지표.
#[derive(Clone, Copy, Debug)]
pub struct ItemMetrics {
    pub id: usize,
    pub qty: usize,
    /// `item.shape_cd.area`
    pub area: f32,
    /// `max(bbox.width(), bbox.height())`
    pub max_dim: f32,
    /// convex hull 면적
    pub hull_area: f32,
    /// `convex hull 면적 / 실제 면적` — 클수록 오목하다.
    pub concavity: f32,
}

/// 인스턴스의 모든 아이템 종류에 대해 정렬 키를 한 번씩만 계산한다.
pub fn collect_metrics(instance: &SPInstance) -> Vec<ItemMetrics> {
    instance
        .items
        .iter()
        .map(|(item, qty)| {
            let shape = item.shape_cd().as_ref();
            let bbox = shape.bbox();
            let area = shape.area();
            let hull_area = shoelace_area(&convex_hull_from_points(shape.vertices().to_vec()));
            ItemMetrics {
                id: item.idx(),
                qty: *qty,
                area,
                max_dim: bbox.width().max(bbox.height()),
                hull_area,
                // 볼록하면 1.0, 오목할수록 커진다. 면적이 0이면 1.0으로 둔다.
                concavity: if area > f32::EPSILON {
                    (hull_area / area).max(1.0)
                } else {
                    1.0
                },
            }
        })
        .collect()
}

/// 난이도 등급별로 묶은 배치 순서 (hard -> medium -> easy).
///
/// 난이도 점수는 정규화한 직경과 오목도를 `diam_factor`로 섞는다.
///
/// ```text
/// score = diam_factor x norm(max_dim) + (1 - diam_factor) x norm(concavity)
/// ```
///
/// 큰 조각은 놓을 자리가 적어서, 오목한 조각은 이웃과 맞물리기 까다로워서 어렵다.
/// 등급 안에서는 점수 내림차순, 동점은 `item_id` 오름차순으로 안정화한다.
pub fn difficulty_tiered_sequence(metrics: &[ItemMetrics], tiers: DifficultyTiers) -> Vec<usize> {
    if metrics.is_empty() {
        return Vec::new();
    }
    let scored = difficulty_scores(metrics, tiers);
    // 점수 내림차순 = 어려운 순. expand_sorted 가 그대로 처리한다.
    expand_sorted(
        scored
            .iter()
            .map(|&(id, qty, score, _)| (id, qty, score))
            .collect(),
    )
}

/// 각 아이템의 `(id, qty, 난이도 점수, 등급)`. 점수 내림차순으로 정렬되어 있다.
pub fn difficulty_scores(
    metrics: &[ItemMetrics],
    tiers: DifficultyTiers,
) -> Vec<(usize, usize, f32, DifficultyTier)> {
    let factor = tiers.diam_factor.clamp(0.0, 1.0);
    let max_dim = metrics
        .iter()
        .map(|m| m.max_dim)
        .fold(0.0f32, f32::max)
        .max(f32::EPSILON);
    let max_concavity = metrics
        .iter()
        .map(|m| m.concavity)
        .fold(0.0f32, f32::max)
        .max(f32::EPSILON);

    let mut scored: Vec<(usize, usize, f32)> = metrics
        .iter()
        .map(|m| {
            let score =
                factor * (m.max_dim / max_dim) + (1.0 - factor) * (m.concavity / max_concavity);
            (m.id, m.qty, score)
        })
        .collect();
    // 점수 내림차순, 동점은 id 오름차순 (결정론).
    scored.sort_by(|a, b| b.2.total_cmp(&a.2).then(a.0.cmp(&b.0)));

    let n = scored.len();
    let hard_end = ((n as f32) * tiers.hard_percent.clamp(0.0, 1.0)).round() as usize;
    let medium_end =
        hard_end + (((n as f32) * tiers.medium_percent.clamp(0.0, 1.0)).round() as usize);

    scored
        .into_iter()
        .enumerate()
        .map(|(rank, (id, qty, score))| {
            let tier = if rank < hard_end {
                DifficultyTier::Hard
            } else if rank < medium_end.min(n) {
                DifficultyTier::Medium
            } else {
                DifficultyTier::Easy
            };
            (id, qty, score, tier)
        })
        .collect()
}

/// 신발끈 공식으로 링의 면적을 구한다 (방향 무관, 항상 양수).
fn shoelace_area(pts: &[Point]) -> f32 {
    if pts.len() < 3 {
        return 0.0;
    }
    let mut acc = 0.0f32;
    let mut j = pts.len() - 1;
    for i in 0..pts.len() {
        acc += (pts[j].0 + pts[i].0) * (pts[j].1 - pts[i].1);
        j = i;
    }
    (acc * 0.5).abs()
}

/// `(item_id, qty, metric)`을 metric 내림차순으로 정렬한 뒤 qty만큼 펼친다.
///
/// 동점은 `item_id` 오름차순으로 안정화한다(결정론성). 같은 종류의 폴리곤은
/// 지표가 동일하므로 항상 연속으로 붙는다.
fn expand_sorted(mut keyed: Vec<(usize, usize, f32)>) -> Vec<usize> {
    // `total_cmp` 는 NaN 도 전순서로 다루므로 `sort_by` 의 전순서 계약을 지킨다
    // (`partial_cmp().unwrap_or(Equal)` 은 NaN 이 섞이면 비추이적이다).
    keyed.sort_by(|a, b| b.2.total_cmp(&a.2).then_with(|| a.0.cmp(&b.0)));
    let mut out = Vec::with_capacity(keyed.iter().map(|(_, q, _)| *q).sum());
    for (id, qty, _) in keyed {
        for _ in 0..qty {
            out.push(id);
        }
    }
    out
}

/// 호출자가 준 custom 순서를 실제 수요와 맞춘다.
///
/// * 존재하지 않는 `item_id`나 수요를 초과하는 반복은 버린다.
/// * 그러고도 모자란 폴리곤은 `fallback`(= area 내림차순 전개열) 순서로 뒤에 채운다.
///
/// 즉 호출자가 준 순서는 그대로 지키되, 항상 완전한 배치 대상 나열이 되도록 보정한다.
fn sanitize_custom(custom: &[usize], quantities: &[usize], fallback: &[usize]) -> Vec<usize> {
    let mut remaining = quantities.to_vec();
    let mut out: Vec<usize> = Vec::with_capacity(fallback.len());

    for &id in custom {
        if let Some(left) = remaining.get_mut(id) {
            if *left > 0 {
                *left -= 1;
                out.push(id);
            }
        }
    }

    for &id in fallback {
        if let Some(left) = remaining.get_mut(id) {
            if *left > 0 {
                *left -= 1;
                out.push(id);
            }
        }
    }

    out
}

// ────────────────────────────────────────────────────────────────────────────
// 이웃 연산 — Swap 2 items / Move 1 item / Block reorder
// ────────────────────────────────────────────────────────────────────────────

/// `Swap 2 items` — 두 위치를 맞바꾼다. 반환값은 변경 시작 지점.
fn op_swap(seq: &mut [usize], i: usize, j: usize) -> usize {
    seq.swap(i, j);
    i.min(j)
}

/// `Move 1 item` — 한 원소를 다른 위치로 옮긴다. 반환값은 변경 시작 지점.
fn op_move(seq: &mut Vec<usize>, from: usize, to: usize) -> usize {
    let v = seq.remove(from);
    let to = to.min(seq.len());
    seq.insert(to, v);
    from.min(to)
}

/// `Block reorder` — 연속 블록을 다른 위치로 옮긴다. 반환값은 변경 시작 지점.
fn op_block(seq: &mut Vec<usize>, start: usize, block_len: usize, dest: usize) -> usize {
    let block: Vec<usize> = seq.drain(start..start + block_len).collect();
    let dest = dest.min(seq.len());
    for (k, v) in block.into_iter().enumerate() {
        seq.insert(dest + k, v);
    }
    start.min(dest)
}

/// 서로 다른 두 위치를 `lo ..= len-1`에서 뽑는다.
fn pick_swap(len: usize, lo: usize, rng: &mut Xoshiro256PlusPlus) -> Option<(usize, usize)> {
    if len < 2 || lo + 1 >= len {
        return None;
    }
    let i = rng.random_range(lo..len);
    let mut j = rng.random_range(lo..len - 1);
    if j >= i {
        j += 1;
    }
    Some((i, j))
}

/// 서로 다른 출발/도착 위치를 `lo ..= len-1`에서 뽑는다.
fn pick_move(len: usize, lo: usize, rng: &mut Xoshiro256PlusPlus) -> Option<(usize, usize)> {
    pick_swap(len, lo, rng)
}

/// 길이 `2..=len/4`의 연속 블록과 그 이동 목적지를 `lo` 이상에서 뽑는다.
fn pick_block(
    len: usize,
    lo: usize,
    rng: &mut Xoshiro256PlusPlus,
) -> Option<(usize, usize, usize)> {
    let max_block = len / 4;
    if max_block < 2 {
        return None;
    }
    let block_len = rng.random_range(2..=max_block);
    let max_start = len - block_len;
    if lo >= max_start {
        return None;
    }
    let start = rng.random_range(lo..=max_start);
    let mut dest = rng.random_range(lo..max_start);
    if dest >= start {
        dest += 1;
    }
    Some((start, block_len, dest))
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::SeedableRng;

    fn rng() -> Xoshiro256PlusPlus {
        Xoshiro256PlusPlus::seed_from_u64(0xC0FFEE)
    }

    fn ordering(sequence: Vec<usize>) -> ItemOrdering {
        ItemOrdering {
            strategy: OrderingStrategy::Random,
            sequence,
            label: "t".to_string(),
        }
    }

    /// 전략 라벨은 문서에 적힌 철자 그대로여야 한다.
    #[test]
    fn strategy_labels_match_document() {
        assert_eq!(OrderingStrategy::AreaDescending.label(), "area-desc");
        assert_eq!(
            OrderingStrategy::MaxDimensionDescending.label(),
            "maxdim-desc"
        );
        assert_eq!(
            OrderingStrategy::ConvexHullAreaDescending.label(),
            "hull-area-desc"
        );
        assert_eq!(
            OrderingStrategy::ConcavityDescending.label(),
            "concavity-desc"
        );
        assert_eq!(OrderingStrategy::Random.label(), "random");
        assert_eq!(OrderingStrategy::CustomGarment.label(), "custom-garment");
    }

    /// metric 내림차순 + 동점은 `item_id` 오름차순, 그리고 qty만큼 전개.
    #[test]
    fn expand_sorted_is_descending_and_deterministic() {
        // id 0: 면적 5(×2), id 1: 면적 9(×1), id 2: 면적 5(×3), id 3: 면적 1(×1)
        let seq = expand_sorted(vec![(0, 2, 5.0), (1, 1, 9.0), (2, 3, 5.0), (3, 1, 1.0)]);
        assert_eq!(seq, vec![1, 0, 0, 2, 2, 2, 3]);
        assert_eq!(seq.len(), 2 + 1 + 3 + 1);
    }

    /// 지표가 NaN이어도 패닉 없이 `item_id` 오름차순으로 안정화된다.
    #[test]
    fn expand_sorted_tolerates_nan() {
        let seq = expand_sorted(vec![(0, 1, f32::NAN), (1, 1, f32::NAN)]);
        assert_eq!(seq, vec![0, 1]);
    }

    /// 앞에서부터 몇 개가 일치하는지 — prefix 재사용의 핵심.
    #[test]
    fn common_prefix_len_counts_matching_head() {
        let a = ordering(vec![1, 2, 3, 4, 5]);
        let b = ordering(vec![1, 2, 3, 5, 4]);
        assert_eq!(a.common_prefix_len(&b), 3);
        assert_eq!(b.common_prefix_len(&a), 3);
        assert_eq!(a.common_prefix_len(&a), 5);
        assert_eq!(a.common_prefix_len(&ordering(vec![9, 2, 3])), 0);
        assert_eq!(a.common_prefix_len(&ordering(vec![1, 2])), 2);
        assert_eq!(a.common_prefix_len(&ordering(vec![])), 0);
    }

    /// 156개 중 80번째 이후만 바뀌면 앞의 79개를 재사용할 수 있다(문서 예시).
    #[test]
    fn document_example_reuses_seventy_nine_prefix() {
        let base: Vec<usize> = (0..156).collect();
        let mut swapped = base.clone();
        // 0-based 79번 인덱스 = 80번째 원소부터 달라진다.
        op_swap(&mut swapped, 79, 120);
        assert_eq!(ordering(base).common_prefix_len(&ordering(swapped)), 79);
    }

    /// Swap 2 items — 멀티셋 보존 + 변경 시작 지점 보고.
    #[test]
    fn op_swap_preserves_multiset() {
        let mut seq = vec![0, 1, 2, 3, 4, 5];
        let at = op_swap(&mut seq, 4, 2);
        assert_eq!(at, 2);
        assert_eq!(seq, vec![0, 1, 4, 3, 2, 5]);
    }

    /// Move 1 item — 멀티셋 보존 + 변경 시작 지점 보고.
    #[test]
    fn op_move_preserves_multiset() {
        let mut seq = vec![0, 1, 2, 3, 4, 5];
        let at = op_move(&mut seq, 5, 2);
        assert_eq!(at, 2);
        assert_eq!(seq, vec![0, 1, 5, 2, 3, 4]);

        let mut seq = vec![0, 1, 2, 3, 4, 5];
        let at = op_move(&mut seq, 2, 5);
        assert_eq!(at, 2);
        assert_eq!(seq, vec![0, 1, 3, 4, 5, 2]);
    }

    /// Block reorder — 연속 블록이 통째로 옮겨진다.
    #[test]
    fn op_block_moves_contiguous_run() {
        let mut seq = vec![0, 1, 2, 3, 4, 5, 6, 7];
        let at = op_block(&mut seq, 5, 3, 1);
        assert_eq!(at, 1);
        assert_eq!(seq, vec![0, 5, 6, 7, 1, 2, 3, 4]);

        let mut seq = vec![0, 1, 2, 3, 4, 5, 6, 7];
        let at = op_block(&mut seq, 1, 2, 5);
        assert_eq!(at, 1);
        assert_eq!(seq, vec![0, 3, 4, 5, 6, 1, 2, 7]);
    }

    /// 하한 `len/2`(= `PREFIX_REUSE`)에서 변형 지점은 항상 뒤쪽 절반에 있다.
    #[test]
    fn mutation_points_stay_in_second_half() {
        let mut r = rng();
        let len = 156;
        let lo = OrderingDiversity::PREFIX_REUSE.lower_bound(len);
        assert_eq!(lo, 78);
        for _ in 0..500 {
            let (i, j) = pick_swap(len, lo, &mut r).expect(crate::dmsg!("가능한 길이"));
            assert!(i >= lo && i < len);
            assert!(j >= lo && j < len);
            assert_ne!(i, j);

            let (from, to) = pick_move(len, lo, &mut r).expect(crate::dmsg!("가능한 길이"));
            assert!(from >= lo && from < len);
            assert!(to >= lo && to < len);
            assert_ne!(from, to);

            let (start, block_len, dest) = pick_block(len, lo, &mut r).expect(crate::dmsg!("가능한 길이"));
            assert!((2..=len / 4).contains(&block_len));
            assert!(start >= lo && start + block_len <= len);
            assert!(dest >= lo && dest + block_len <= len);
            assert_ne!(start, dest);
        }
    }

    /// 하한 비율은 `floor(len * ratio)`이고, 0.5는 정확히 `len / 2`다.
    ///
    /// 기존 경로가 이 등식에 의존한다 — 홀수 길이에서 반올림이 한 칸이라도 달라지면
    /// 파생 ordering이 통째로 바뀐다.
    #[test]
    fn lower_bound_ratio_matches_integer_halving() {
        for len in 0..1000usize {
            assert_eq!(
                OrderingDiversity::PREFIX_REUSE.lower_bound(len),
                len / 2,
                "len {len}"
            );
            assert_eq!(OrderingDiversity::FULL_RANGE.lower_bound(len), 0);
        }
        // clamp: 음수·1 초과 비율도 유효 범위로 접힌다.
        let over = OrderingDiversity {
            mutation_lower_ratio: 4.0,
            ..OrderingDiversity::PREFIX_REUSE
        };
        assert_eq!(over.lower_bound(100), 100);
        let under = OrderingDiversity {
            mutation_lower_ratio: -1.0,
            ..OrderingDiversity::PREFIX_REUSE
        };
        assert_eq!(under.lower_bound(100), 0);
    }

    /// 연산 순환표: `[1,1,1]`은 예전 `op_cycle % 3`과 같은 `[0,1,2]`여야 한다.
    #[test]
    fn op_plan_expands_weights() {
        assert_eq!(OrderingDiversity::PREFIX_REUSE.op_plan(), vec![0, 1, 2]);
        assert_eq!(OrderingDiversity::FULL_RANGE.op_plan(), vec![0, 1, 1, 2]);
        // 전부 0이면 파생을 못 만드니 균등으로 되돌린다.
        let zero = OrderingDiversity {
            op_weights: [0, 0, 0],
            ..OrderingDiversity::FULL_RANGE
        };
        assert_eq!(zero.op_plan(), vec![0, 1, 2]);
        // 한 연산만 쓰는 것도 유효하다(계측용).
        let swap_only = OrderingDiversity {
            op_weights: [1, 0, 0],
            ..OrderingDiversity::FULL_RANGE
        };
        assert_eq!(swap_only.op_plan(), vec![0]);
    }

    /// 하한 0이면 변형이 앞쪽(= 큰 조각)까지 닿는다 — 다양성의 전제.
    #[test]
    fn full_range_reaches_the_front_half() {
        let mut r = rng();
        let len = 156;
        let mut front_hits = 0usize;
        for _ in 0..500 {
            if let Some((i, j)) = pick_swap(len, 0, &mut r) {
                if i < len / 2 || j < len / 2 {
                    front_hits += 1;
                }
            }
        }
        // 뒤쪽 절반만 뽑힐 확률은 1/4 이므로 500회 중 대부분이 앞쪽을 건드린다.
        assert!(front_hits > 300, "front_hits = {front_hits}");
    }

    /// 너무 짧은 나열에서는 이웃 연산이 성립하지 않는다.
    #[test]
    fn pickers_reject_degenerate_lengths() {
        let mut r = rng();
        let lo = |len: usize| OrderingDiversity::PREFIX_REUSE.lower_bound(len);
        assert!(pick_swap(0, lo(0), &mut r).is_none());
        assert!(pick_swap(1, lo(1), &mut r).is_none());
        assert!(pick_swap(2, lo(2), &mut r).is_none());
        assert!(pick_swap(3, lo(3), &mut r).is_some());
        // block_len >= 2 이려면 len/4 >= 2, 즉 len >= 8이 필요하다.
        assert!(pick_block(4, lo(4), &mut r).is_none());
        assert!(pick_block(7, lo(7), &mut r).is_none());
        // len = 8 : block_len = 2, lo = 4 < max_start = 6 이므로 성립한다.
        assert!(pick_block(8, lo(8), &mut r).is_some());
        assert!(pick_block(16, lo(16), &mut r).is_some());
        // 하한 0에서는 더 짧은 나열에서도 성립한다.
        assert!(pick_swap(2, 0, &mut r).is_some());
    }

    /// 이웃 연산은 멀티셋을 바꾸지 않는다 — 배치 대상 수가 늘거나 줄면 안 된다.
    #[test]
    fn neighbourhood_ops_never_change_the_multiset() {
        let mut r = rng();
        let base: Vec<usize> = (0..40).map(|i| i % 7).collect();
        let mut sorted_base = base.clone();
        sorted_base.sort_unstable();

        for _ in 0..200 {
            let mut seq = base.clone();
            // 하한 둘 다 확인한다 — 하한 0에서도 멀티셋은 보존되어야 한다.
            for lo in [seq.len() / 2, 0] {
                if let Some((i, j)) = pick_swap(seq.len(), lo, &mut r) {
                    op_swap(&mut seq, i, j);
                }
                if let Some((from, to)) = pick_move(seq.len(), lo, &mut r) {
                    op_move(&mut seq, from, to);
                }
                if let Some((s, b, d)) = pick_block(seq.len(), lo, &mut r) {
                    op_block(&mut seq, s, b, d);
                }
            }
            let mut sorted = seq.clone();
            sorted.sort_unstable();
            assert_eq!(sorted, sorted_base);
        }
    }

    /// custom 순서는 그대로 지키되 수요 초과·미지 id는 버리고 모자란 만큼 채운다.
    #[test]
    fn sanitize_custom_respects_demand() {
        let quantities = vec![2, 1, 3];
        let fallback = vec![2, 2, 2, 0, 0, 1];

        // 정상 입력은 그대로.
        let ok = sanitize_custom(&[1, 0, 0, 2, 2, 2], &quantities, &fallback);
        assert_eq!(ok, vec![1, 0, 0, 2, 2, 2]);

        // 수요 초과(0이 3번) + 미지 id(9)는 버리고, 남은 것은 fallback 순서로 채운다.
        let fixed = sanitize_custom(&[0, 0, 0, 9, 2], &quantities, &fallback);
        assert_eq!(fixed, vec![0, 0, 2, 2, 2, 1]);
        assert_eq!(fixed.len(), fallback.len());

        // 빈 custom이면 fallback 그대로.
        assert_eq!(sanitize_custom(&[], &quantities, &fallback), fallback);
    }

    /// 신발끈 면적은 방향과 무관하게 항상 양수다.
    #[test]
    fn shoelace_area_is_orientation_free() {
        let ccw = [
            Point(0.0, 0.0),
            Point(2.0, 0.0),
            Point(2.0, 3.0),
            Point(0.0, 3.0),
        ];
        let cw = [
            Point(0.0, 0.0),
            Point(0.0, 3.0),
            Point(2.0, 3.0),
            Point(2.0, 0.0),
        ];
        assert!((shoelace_area(&ccw) - 6.0).abs() < 1e-5);
        assert!((shoelace_area(&cw) - 6.0).abs() < 1e-5);
        // 점/선분은 면적 0.
        assert_eq!(shoelace_area(&[]), 0.0);
        assert_eq!(shoelace_area(&[Point(0.0, 0.0), Point(1.0, 1.0)]), 0.0);
    }

    // ────────────────────────────────────────────────────────────────────
    // 다양성 계측 — 실제 마커를 배치해 본다
    // ────────────────────────────────────────────────────────────────────

    /// 여성WSjk 156-패널. 없으면 계측을 건너뛴다(CI 에는 데이터가 없을 수 있다).
    fn garment_marker() -> Option<SPInstance> {
        let path =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input/wsjk156.json");
        let (ext, _) = crate::util::io::read_spp_input(&path)
            .map_err(|e| eprintln!("wsjk156.json 을 읽을 수 없어 계측을 건너뛴다: {e}"))
            .ok()?;
        let c = crate::config::DEFAULT_SWARM_CONFIG;
        let importer = crate::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
        jagua_rs::probs::spp::io::import_instance(&importer, &ext)
            .map_err(|e| eprintln!("wsjk156.json import 실패로 계측을 건너뛴다: {e:?}"))
            .ok()
    }

    fn env_usize(key: &str, default: usize) -> usize {
        std::env::var(key)
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(default)
    }

    /// 파생 ordering인가 — 라벨이 연산 이름으로 시작한다.
    fn is_derived(o: &ItemOrdering) -> bool {
        o.label.starts_with("swap@")
            || o.label.starts_with("move@")
            || o.label.starts_with("block@")
    }

    /// 기존 경로(`build_orderings`)의 계약: 파생 ordering은 base(area-desc)와
    /// **앞 `len/2`가 반드시 같다**. 이게 깨지면 prefix 재사용이 통째로 무력화된다.
    #[test]
    fn prefix_reuse_path_keeps_the_front_half_intact() {
        let Some(instance) = garment_marker() else {
            return;
        };
        let mut r = Xoshiro256PlusPlus::seed_from_u64(0);
        let orderings = build_orderings(&instance, 30, &mut r, None);
        let Some(base) = orderings.first() else {
            panic!("{}", crate::dmsg!("ordering 이 하나도 없다"));
        };
        assert_eq!(base.strategy, OrderingStrategy::AreaDescending);
        let half = base.sequence.len() / 2;
        let derived = orderings.iter().filter(|o| is_derived(o)).count();
        assert!(derived > 0, "파생 ordering 이 없다");
        for o in orderings.iter().filter(|o| is_derived(o)) {
            assert!(
                base.common_prefix_len(o) >= half,
                "{} 의 공통 prefix {} < {half}",
                o.label,
                base.common_prefix_len(o),
            );
        }

        // 하한 0이면 정반대여야 한다 — 앞쪽 절반이 실제로 흔들린다.
        let mut r = Xoshiro256PlusPlus::seed_from_u64(0);
        let full = build_orderings_with(&instance, 30, &mut r, None, OrderingDiversity::FULL_RANGE);
        let shaken = full
            .iter()
            .filter(|o| is_derived(o) && base.common_prefix_len(o) < half)
            .count();
        let derived_full = full.iter().filter(|o| is_derived(o)).count();
        assert!(
            shaken * 2 > derived_full,
            "하한 0인데 앞쪽이 바뀐 파생이 {shaken}/{derived_full} 뿐이다"
        );
    }

    /// 다양성 노브가 실제 strip 길이에 무엇을 하는가 — 배치를 끝까지 돌려 잰다.
    ///
    /// 모듈 문서 상단의 표가 이 테스트의 출력이다.
    ///
    /// ```text
    /// cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first"     ///     --lib nfp_lbf::ordering::tests::diversity_measurement -- --ignored --nocapture
    ///
    /// ORD_N        ordering 수 (기본 50)
    /// ORD_PROBES   탐침 예산  (기본 4096)
    /// ORD_SEEDS    쉼표 구분 seed 목록 (기본 0~8)
    /// ```
    ///
    /// seed 를 아홉 개나 도는 이유: arm 사이 차이가 10mm(0.15%) 수준인데 seed 분산이
    /// 그보다 커서 적은 seed 로는 순위가 그냥 뒤집힌다. 실제로 3 seed 에서는 block 만
    /// 쓰는 쪽이 1등이었는데(7064.4) 9 seed 로 늘리자 꼴찌가 됐다(7089.7).
    /// 9 seed 한 바퀴가 약 18분이다.
    #[test]
    #[ignore = "실측용 — 156패널 배치를 수백 번 돌려 분 단위가 걸린다"]
    fn diversity_measurement() {
        use crate::nfp_lbf::cache::{NfpLbfCache, PreparedLibrary};
        use crate::nfp_lbf::placement::SequentialPlacer;
        use crate::nfp_lbf::score::PlacementScoreWeights;
        use crate::nfp_strategy::rotations::item_rotation_table;

        let Some(instance) = garment_marker() else {
            return;
        };
        let n = env_usize("ORD_N", 50);
        let probes = env_usize("ORD_PROBES", 4096);
        let seeds: Vec<u64> = match std::env::var("ORD_SEEDS") {
            Ok(v) => v.split(',').filter_map(|s| s.trim().parse().ok()).collect(),
            Err(_) => vec![0, 1, 2, 3, 4, 5, 6, 7, 8],
        };

        let strip_height = instance.base_strip.fixed_height;
        let strip_width_cap: f32 = instance
            .items
            .iter()
            .map(|(item, qty)| item.shape_cd().bbox().width() * (*qty as f32))
            .filter(|w| w.is_finite() && *w > 0.0)
            .sum();
        let item_area = instance.item_area() as f64;
        let rotations = vec![0i64];
        let Some(library) = PreparedLibrary::build(&instance, &rotations, 0.02) else {
            eprintln!("NFP 라이브러리 build 실패 — 계측을 건너뛴다");
            return;
        };
        let mut cache = NfpLbfCache::new(library);
        let _ = cache.prewarm(std::time::Duration::from_secs(10));
        let table = item_rotation_table(&instance, &rotations);
        let mut placer = SequentialPlacer::with_item_rotations(
            &mut cache,
            strip_height,
            strip_width_cap,
            PlacementScoreWeights::default(),
            table,
        )
        .with_probe_budget(probes);

        let at = |ratio: f32, w: [u8; 3]| OrderingDiversity {
            mutation_lower_ratio: ratio,
            op_weights: w,
        };
        let arms: Vec<(&str, OrderingDiversity)> = vec![
            ("하한 len/2 [1,1,1] 기존", OrderingDiversity::PREFIX_REUSE),
            ("하한 0     [1,1,1]     ", at(0.0, [1, 1, 1])),
            ("하한 0     [1,0,0] swap", at(0.0, [1, 0, 0])),
            ("하한 0     [0,1,0] move", at(0.0, [0, 1, 0])),
            ("하한 0     [0,0,1] blk ", at(0.0, [0, 0, 1])),
            ("하한 0     [1,2,1] 채택", OrderingDiversity::FULL_RANGE),
            ("하한 0.25  [1,1,1]     ", at(0.25, [1, 1, 1])),
        ];

        let dens = |len: f32| item_area / (len as f64 * strip_height as f64) * 100.0;
        println!(
            "\n=== ordering 다양성 계측 — 156 패널 / ordering {n}개 / 탐침 {probes} / seed {seeds:?} ==="
        );

        for (label, diversity) in arms {
            let t0 = std::time::Instant::now();
            let mut det_best = f32::INFINITY;
            let mut best_per_seed: Vec<f32> = Vec::new();
            let mut mean_per_seed: Vec<f32> = Vec::new();
            // 스프레드(표준편차·최대-최소)를 seed 마다 따로 재서 평균낸다. seed 를 섞어
            // 한꺼번에 재면 seed 간 편차까지 섞여 "다양성이 늘었나"를 못 본다.
            let mut sd_per_seed: Vec<f32> = Vec::new();
            let mut range_per_seed: Vec<f32> = Vec::new();
            let mut front_diff = 0usize;
            let mut derived_total = 0usize;

            for &seed in seeds.iter() {
                let mut rng = Xoshiro256PlusPlus::seed_from_u64(seed);
                let orderings = build_orderings_with(&instance, n, &mut rng, None, diversity);
                let Some(base_seq) = orderings.first().map(|o| o.sequence.clone()) else {
                    continue;
                };
                let half = base_seq.len() / 2;
                let mut derived: Vec<f32> = Vec::new();

                for o in orderings.iter() {
                    let p = placer.place_sequence(&o.sequence, None, 0, None);
                    // 미배치가 있으면 strip 길이 비교가 무의미하다 — 벌점을 준다.
                    let len = p.strip_length + p.unplaced.len() as f32 * strip_width_cap;
                    if is_derived(o) {
                        derived_total += 1;
                        if o.sequence.get(..half) != base_seq.get(..half) {
                            front_diff += 1;
                        }
                        derived.push(len);
                    } else if len < det_best {
                        det_best = len;
                    }
                }
                if derived.is_empty() {
                    continue;
                }
                let lo_len = derived.iter().copied().fold(f32::INFINITY, f32::min);
                let hi_len = derived.iter().copied().fold(f32::NEG_INFINITY, f32::max);
                let mean = derived.iter().sum::<f32>() / derived.len() as f32;
                // 표본 표준편차. 파생이 하나뿐이면 정의되지 않으므로 0으로 둔다.
                let sd = if derived.len() < 2 {
                    0.0
                } else {
                    let var = derived
                        .iter()
                        .map(|v| {
                            let d = (*v - mean) as f64;
                            d * d
                        })
                        .sum::<f64>()
                        / (derived.len() - 1) as f64;
                    var.max(0.0).sqrt() as f32
                };
                best_per_seed.push(lo_len);
                mean_per_seed.push(mean);
                sd_per_seed.push(sd);
                range_per_seed.push(hi_len - lo_len);
            }

            let avg = |v: &[f32]| {
                if v.is_empty() {
                    f32::NAN
                } else {
                    v.iter().sum::<f32>() / v.len() as f32
                }
            };
            let best_min = best_per_seed.iter().copied().fold(f32::INFINITY, f32::min);
            println!(
                "{label}  앞절반diff {front_diff:>4}/{derived_total:<4}  결정best {det_best:>8.1}  \
                 파생best평균 {:>8.1}  최고 {best_min:>8.1} ({:>5.2}%)  파생평균 {:>8.1}  \
                 표준편차 {:>6.1}  최대-최소 {:>6.1}  {:>5.1}s",
                avg(&best_per_seed),
                dens(best_min),
                avg(&mean_per_seed),
                avg(&sd_per_seed),
                avg(&range_per_seed),
                t0.elapsed().as_secs_f32(),
            );
        }
    }

    /// 정작 하한 0을 쓸 곳 — **전략 경로**에서 A/B 를 돌린다.
    ///
    /// # 왜 따로 필요한가
    ///
    /// `diversity_measurement` 는 전략을 다 끈 그리디로 잰 값이다. 그런데
    /// [`OrderingDiversity::FULL_RANGE`] 가 실제로 쓰일 곳은 빔·best-fit·슬라이딩이
    /// 도는 경로다. 그쪽은 배치 1회 비용이 몇 배라 파생 ordering 하나의 값어치도
    /// 다르고, 무엇보다 그리디에서 잰 이득(11mm)이 seed 분산(20mm)보다 작았다.
    /// **쓸 경로에서 재지 않으면 채택 근거가 없다.**
    ///
    /// `search.rs` 배선 없이도 잴 수 있다 — 전략 파이프라인을 여기서 직접 부르면 된다
    /// (`search.rs::strategy_path` 가 하는 일과 같다: 클러스터 한 번 만들고 ordering
    /// 마다 `place_with_strategies`).
    ///
    /// ```text
    /// ORD_ST_N       ordering 수  (기본 12 — 벤치 기준선과 같다)
    /// ORD_ST_PROBES  탐침 예산    (기본 4096)
    /// ORD_ST_SEEDS   쉼표 구분 seed 목록 (기본 0,1,2)
    /// ```
    #[test]
    #[ignore = "실측용 — 전략 경로를 seed 마다 돌려 분 단위가 걸린다"]
    fn strategy_path_diversity_ab() {
        use crate::nfp_lbf::cache::{NfpLbfCache, PreparedLibrary};
        use crate::nfp_lbf::placement::SequentialPlacer;
        use crate::nfp_lbf::score::PlacementScoreWeights;
        use crate::nfp_strategy::rotations::item_rotation_table;
        use crate::nfp_strategy::{
            DEFAULT_NFP_STRATEGY_CONFIG, NfpStrategyConfig, StrategyContext, cluster, pipeline,
        };

        let Some(instance) = garment_marker() else {
            return;
        };
        let n = env_usize("ORD_ST_N", 12);
        let probes = env_usize("ORD_ST_PROBES", 4096);
        let seeds: Vec<u64> = match std::env::var("ORD_ST_SEEDS") {
            Ok(v) => v.split(',').filter_map(|s| s.trim().parse().ok()).collect(),
            Err(_) => vec![0, 1, 2],
        };

        let strip_height = instance.base_strip.fixed_height;
        let strip_width_cap: f32 = instance
            .items
            .iter()
            .map(|(item, qty)| item.shape_cd().bbox().width() * (*qty as f32))
            .filter(|w| w.is_finite() && *w > 0.0)
            .sum();
        let item_area = instance.item_area() as f64;
        let rotations = vec![0i64];
        let Some(library) = PreparedLibrary::build(&instance, &rotations, 0.02) else {
            eprintln!("NFP 라이브러리 build 실패 — 계측을 건너뛴다");
            return;
        };
        let mut cache = NfpLbfCache::new(library);
        let _ = cache.prewarm(std::time::Duration::from_secs(10));
        let table = item_rotation_table(&instance, &rotations);

        // 벤치의 "최고 조합"(②+③+④+⑦, 72.18%)과 같은 설정으로 잰다.
        let strategy = NfpStrategyConfig {
            pairwise_clustering: true,
            contact_maximization: true,
            pocket_first: true,
            sliding_compaction: true,
            ..DEFAULT_NFP_STRATEGY_CONFIG
        };
        let dens = |len: f64| item_area / (len * strip_height as f64) * 100.0;

        println!(
            "\n=== 전략 경로(②+③+④+⑦) ordering 다양성 A/B — ordering {n}개 / 탐침 {probes} / seed {seeds:?} ==="
        );
        for (label, diversity) in [
            ("하한 len/2 [1,1,1] 기존", OrderingDiversity::PREFIX_REUSE),
            ("하한 0     [1,2,1] 채택", OrderingDiversity::FULL_RANGE),
        ] {
            let t0 = std::time::Instant::now();
            let mut best_per_seed: Vec<f64> = Vec::new();
            for &seed in seeds.iter() {
                let mut rng = Xoshiro256PlusPlus::seed_from_u64(seed);
                let orderings = build_orderings_with(&instance, n, &mut rng, None, diversity);
                let mut placer = SequentialPlacer::with_item_rotations(
                    &mut cache,
                    strip_height,
                    strip_width_cap,
                    PlacementScoreWeights::default(),
                    table.clone(),
                )
                .with_probe_budget(probes);
                // `search.rs::placer_item_rotations` 와 같은 방식으로 만든다.
                let max_id = instance.items.iter().map(|(i, _)| i.idx()).max().unwrap_or(0);
                let item_rotations: Vec<Vec<i64>> = (0..=max_id)
                    .map(|id| placer.allowed_rotations(id))
                    .collect();
                let mut ctx = StrategyContext::new(&mut placer, strategy, item_rotations, None);
                let clusters = cluster::build_clusters(&mut ctx, &instance, None);
                let mut best = f64::INFINITY;
                for o in orderings.iter() {
                    let p = pipeline::place_with_strategies(&mut ctx, &o.sequence, &clusters);
                    // 미배치는 비교 불가 — 그리디 계측과 같은 벌점을 준다.
                    let len = p.strip_length + p.unplaced.len() as f64 * strip_width_cap as f64;
                    if len < best {
                        best = len;
                    }
                }
                if best.is_finite() {
                    best_per_seed.push(best);
                }
            }
            let mean = if best_per_seed.is_empty() {
                f64::NAN
            } else {
                best_per_seed.iter().sum::<f64>() / best_per_seed.len() as f64
            };
            let min = best_per_seed.iter().copied().fold(f64::INFINITY, f64::min);
            println!(
                "{label}  seed별best평균 {mean:>8.1} ({:>5.2}%)  최고 {min:>8.1} ({:>5.2}%)  \
                 seed별 {:?}  {:>5.1}s",
                dens(mean),
                dens(min),
                best_per_seed
                    .iter()
                    .map(|v| format!("{v:.1}"))
                    .collect::<Vec<_>>(),
                t0.elapsed().as_secs_f32(),
            );
        }
    }

    /// L 자 형상의 오목도 = hull 면적 / 실제 면적 > 1.
    #[test]
    fn concavity_of_l_shape_exceeds_one() {
        // (0,0)-(2,0)-(2,1)-(1,1)-(1,2)-(0,2) : 면적 3, hull 면적 3.5
        let pts = vec![
            Point(0.0, 0.0),
            Point(2.0, 0.0),
            Point(2.0, 1.0),
            Point(1.0, 1.0),
            Point(1.0, 2.0),
            Point(0.0, 2.0),
        ];
        let area = shoelace_area(&pts);
        let hull_area = shoelace_area(&convex_hull_from_points(pts));
        assert!((area - 3.0).abs() < 1e-5);
        assert!((hull_area - 3.5).abs() < 1e-5);
        assert!(hull_area / area > 1.0);
    }
}
