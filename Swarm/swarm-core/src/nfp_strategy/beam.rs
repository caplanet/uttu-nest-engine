//! ⑤ 빔 서치 — 부분 배치를 1개가 아니라 `B`개 유지한다.
//!
//! # 왜 필요한가
//!
//! 순차 배치는 되돌아갈 수 없다. 3번째 조각을 어디에 놓을지 정하는 순간 그 선택은
//! 156번째 조각까지 그대로 간다. 그리디가 "지금 가장 좋은" 자리를 골라도 그 자리가
//! 뒤에 올 조각에게 못 쓰는 공백을 남기면 손해는 끝까지 누적된다.
//!
//! 빔은 그 결정을 **유예**한다. 각 깊이에서 유망한 부분 배치를 `B`개 남겨 두고,
//! 어느 쪽이 옳았는지는 더 깊이 내려가 strip 길이로 판정한다.
//!
//! ```text
//! 깊이 d      노드 B개
//!   |  각 노드에서 상위 branch개 후보로 확장  ->  B x branch 개 자식
//!   |  같은 배치에 도달한 자식 제거(0.01mm 격자 해시)
//!   v  미배치 개수 -> strip 길이 -> 총 낭비 순으로 상위 B개만 남긴다
//! 깊이 d+1    노드 B개
//! ```
//!
//! # 그리디 앵커 — 빔은 그리디보다 나빠질 수 없다
//!
//! 빔 서치는 일반적으로 폭에 대해 단조가 아니다. 폭을 넓히면 자식 풀이 커지고,
//! 좁은 빔이 살렸던 노드가 그 안에서 밀려나 사라질 수 있기 때문이다. 그래서 이
//! 구현은 **빔의 0번 슬롯을 그리디 체인에 고정 배정**한다. 0번 노드의 `rule` 1순위
//! 자식은 점수와 무관하게 반드시 살아남고, 나머지 `B-1`칸을 남은 자식들이 겨룬다.
//!
//! 그 결과 어떤 폭에서도 그리디 경로가 빔 안에 통째로 들어 있으므로, 최종 최선은
//! **정의상 그리디 이하**다. `width == 1`이면 0번 슬롯 하나만 남으므로 결과가
//! 순수 그리디와 **완전히 같다**(테스트로 고정).
//!
//! 여기서 "이하"는 [`node_order`]의 사전식 순서 — `(미배치 개수, strip 길이, 낭비)` —
//! 기준이다. 풀어 쓰면 **빔은 그리디보다 조각을 덜 놓지 않고, 같은 수를 놓았다면
//! strip이 더 길지 않다**. 이 둘을 "strip이 더 짧다" 한 줄로 줄일 수 없는 이유는
//! 조각을 포기하기만 하면 strip은 언제든 짧아지기 때문이다 — 그래서 미배치 개수가
//! 먼저다.
//!
//! # 이 파일이 오라클에서 가져오는 것
//!
//! `ctx.options_for`(feasible 앵커 열거)와 `ctx.metrics`(mm 단위 지표)뿐이다.
//! 어느 후보를 고르고 어느 부분 배치를 살릴지는 전부 여기서 정한다.

use std::cmp::Ordering;
use std::collections::HashSet;

use crate::nfp_lbf::PlacedPiece;
use crate::nfp_lbf::placement::Aabb;
use crate::nfp_strategy::cluster::Cluster;
use crate::nfp_strategy::pipeline;
use crate::nfp_strategy::select::{self, SelectionRule};
use crate::nfp_strategy::{AnchorMetrics, AnchorOption, StrategyContext};

/// 중복 노드 판정에 쓰는 좌표 격자(mm).
///
/// 후보 좌표는 NFP 경계 교점에서 나오고 `f32`를 거치므로, 기하학적으로 같은 자리가
/// 마지막 자리에서 미세하게 갈린다. 0.01mm는 이 코드베이스가 쓰는 좌표 허용오차
/// (`COORD_TIE_EPS`, 0.001mm)보다 한 자리 크게 잡아 그 흔들림을 흡수하면서도
/// 실제로 다른 배치를 같다고 뭉개지 않는 폭이다.
const DEDUP_GRID_MM: f64 = 0.01;

/// 노드당 확장 폭. `width >= 2`면 폭과 무관하게 이 값이다.
///
/// 확장 폭은 **비싼 축이 아니다**. 깊이당 NFP 비용은 `ctx.options_for` 호출 횟수,
/// 즉 `width`에만 비례한다(한 노드의 후보 목록은 한 번 만들어 재사용한다).
/// `branch`는 그 목록에서 상위 몇 개를 자식으로 삼을지일 뿐이라 정렬·복제 비용만 는다.
/// 그래서 `NfpStrategyConfig::estimated_cost_factor`도 `beam_width`만 곱한다.
///
/// # 왜 `width.clamp(2, MAX_BRANCH)` 가 아니라 상수인가 — 예전 식은 상한이 안 걸렸다
///
/// 예전 식은 `branch = width.clamp(2, MAX_BRANCH)` 였다. `width <= 4`면 상한이 한 번도
/// 걸리지 않으므로 이 상수는 **죽은 값**이었고 branch 가 그대로 width 였다. 즉 폭 2
/// 빔은 노드당 후보를 2개만 보고 자식 4개만 만들었다 — 위 근거("branch 는 싸다")와
/// 정반대로 동작한 셈이다.
///
/// 실측(wsjk156, 탐침 4096, 유효 ordering 12개, seed 0). 기준선(전략 전부 끔) 71.75%:
///
/// | 폭 | branch | 밀도 | 시간 |
/// |---:|---:|---:|---:|
/// | 2 | 2 (예전 식) | 71.75% | 10.5s |
/// | **2** | **4 (지금)** | **72.10%** | **11.5s** |
/// | 2 | 6 | 72.10% | 10.7s |
/// | 3 | 3 (예전 식) | 71.82%\* | 15.0s |
/// | 4 | 4 (두 식 같음) | 71.91% | 22.8s |
/// | 4 | 6 | 71.97% | 22.0s |
/// | 4 | 10 | 71.75% | 25.2s |
/// | 4 | 20 | 71.89% | 25.5s |
/// | 8 | 4 (두 식 같음) | 71.80% | 36.3s |
///
/// \* 폭 3은 예전 식에서도 branch 3이라 지금 식(4)과 다르다 — 표의 값은 branch 4다.
///
/// 읽는 법: **branch 를 폭에서 떼어내면 폭 2가 폭 4보다 좋아진다** (72.10% / 11.5s vs
/// 71.91% / 22.8s). 다른 seed(1, 2)에서도 순서가 같다: 폭 2 branch 2 → 71.52%,
/// 폭 2 branch 4 → 71.96%, 폭 4 → 71.89%.
///
/// 4를 넘겨도 이득이 없는 것은 예전 근거 그대로다 — 자식 풀이 `width x branch`로
/// 커져 봐야 어차피 `width`개로 다시 잘린다. 6~20 구간은 ±0.15%p 안에서 흔들릴 뿐
/// 추세가 없다.
const MAX_BRANCH: usize = 4;

/// 빔이 실제로 갈라지고 있는지 세는 계측 카운터.
///
/// 켜지 않으면 깊이마다 relaxed load 한 번이다(깊이 = 조각 수 x ordering 수라
/// NFP 열거 옆에서는 계측에 잡히지 않는다). 켜는 것은 `#[ignore]` 계측 테스트뿐이다.
///
/// # 이 카운터가 실제로 답한 것
///
/// wsjk156·폭 4·탐침 4096·유효 ordering 12개에서 깊이당 평균:
/// 살아 있는 노드 3.98, 자식 15.92 -> 중복 제거 후 **15.92**, 생존 4.00,
/// **생존 노드가 나온 서로 다른 부모 3.26개**.
///
/// 즉 (1) 중복 제거가 지우는 노드가 사실상 없고, (2) 살아남는 노드가 3개 넘는 서로
/// 다른 부모에서 나온다. "중복 제거가 과해 빔이 한 상태로 수렴한다" 는 가설은
/// **거짓**이다 — 빔은 갈라지고 있고, 이득이 작은 이유는 다른 데 있다.
#[doc(hidden)]
pub mod meas {
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering as AO};

    pub static ON: AtomicBool = AtomicBool::new(false);
    /// 깊이 반복 횟수(= 조각 수 x ordering 수).
    pub static DEPTHS: AtomicUsize = AtomicUsize::new(0);
    /// 각 깊이 진입 시 살아 있던 노드 수의 합.
    pub static BEAM_SIZE: AtomicUsize = AtomicUsize::new(0);
    /// 중복 제거 전 자식 수의 합.
    pub static CHILD_RAW: AtomicUsize = AtomicUsize::new(0);
    /// 중복 제거 후 자식 수의 합.
    pub static CHILD_DEDUP: AtomicUsize = AtomicUsize::new(0);
    /// 잘라내고 남은 노드 수의 합.
    pub static SURVIVORS: AtomicUsize = AtomicUsize::new(0);
    /// 남은 노드가 나온 **서로 다른 부모** 수의 합. 깊이당 1이면 다양성이 죽은 것이다.
    pub static SURV_PARENTS: AtomicUsize = AtomicUsize::new(0);

    pub fn set(on: bool) {
        ON.store(on, AO::Relaxed);
    }
    pub fn is_on() -> bool {
        ON.load(AO::Relaxed)
    }
    pub fn reset() {
        for c in [
            &DEPTHS,
            &BEAM_SIZE,
            &CHILD_RAW,
            &CHILD_DEDUP,
            &SURVIVORS,
            &SURV_PARENTS,
        ] {
            c.store(0, AO::Relaxed);
        }
    }
    fn g(c: &AtomicUsize) -> usize {
        c.load(AO::Relaxed)
    }
    pub fn report() -> String {
        let d = g(&DEPTHS).max(1) as f64;
        format!(
            "깊이 {} / 깊이당 평균: 살아있는 노드 {:.2}, 자식(중복전) {:.2} -> (중복후) {:.2}, \
             생존 {:.2}, 생존 부모 종류 {:.2}",
            g(&DEPTHS),
            g(&BEAM_SIZE) as f64 / d,
            g(&CHILD_RAW) as f64 / d,
            g(&CHILD_DEDUP) as f64 / d,
            g(&SURVIVORS) as f64 / d,
            g(&SURV_PARENTS) as f64 / d,
        )
    }
}

/// 빔 서치 배치 결과.
#[derive(Clone, Debug, Default)]
pub struct BeamPlacement {
    /// 확정 배치된 조각들(배치 순서 그대로).
    pub pieces: Vec<PlacedPiece>,
    /// 놓지 못한 sequence 인덱스.
    ///
    /// **item id가 아니라 `sequence` 안의 위치**다. 같은 item id가 여러 번 나오는
    /// 것이 정상이므로 id로는 어느 것이 빠졌는지 지목할 수 없다.
    pub unplaced: Vec<usize>,
    /// 최종 strip 길이(mm).
    pub strip_length: f64,
    /// 실제로 확장한 노드 수 — 비용 계측용.
    pub nodes_expanded: usize,
    /// **이긴 노드**가 ②로 붙인 쌍의 수.
    ///
    /// 빔 전체의 합이 아니다. 살아남지 못한 노드가 붙인 쌍은 결과에 남지 않으므로
    /// 그것까지 세면 보고서의 "쌍 N개" 가 배치에 없는 쌍을 가리키게 된다.
    pub clusters_applied: usize,
}

/// 빔 안의 부분 배치 하나.
///
/// # 왜 `Vec`을 통째로 복제해도 괜찮은가
///
/// 접두를 `Rc`로 공유하면 복사는 사라지지만, `ctx.options_for`가 `&[PlacedPiece]`
/// **연속 슬라이스**를 요구하므로 확장할 때마다 어차피 평탄화해야 한다. 즉 복사가
/// 옮겨질 뿐 없어지지 않는다.
///
/// 그리고 실제 양이 작다. `PlacedPiece`는 24바이트, 조각 156개면 노드 하나가
/// 3.7KB다. 깊이마다 살아 있는 노드는 `width x branch <= 4 x 4 = 16`개뿐이고
/// 다음 깊이로 넘어가면서 `width`개로 잘리므로 최대 상주량이 60KB 남짓이다.
/// 깊이에 비례해 쌓이지 않는다.
///
/// 반면 같은 깊이 한 번의 NFP 후보 열거는 탐침 수천 회다. 60KB memcpy는 그 옆에서
/// 계측에 잡히지 않는다. 복잡도를 사서 얻을 것이 없어 그냥 복제한다.
#[derive(Clone, Debug)]
struct BeamNode {
    /// 배치된 조각들.
    pieces: Vec<PlacedPiece>,
    /// `pieces`와 1:1인 월드 bbox. `ctx.metrics`가 매번 요구하므로 증분으로 들고 간다
    /// — 자식마다 `world_bounds`를 다시 부르면 그것만으로 O(자식 x 배치)가 된다.
    boxes: Vec<Aabb>,
    /// 놓지 못한 sequence 인덱스(오름차순).
    unplaced: Vec<usize>,
    /// 현재까지의 strip 길이(mm).
    strip_length: f64,
    /// 지금까지 배치가 만든 낭비의 합(mm²).
    waste: f64,
    /// 이 노드를 만든 부모의 빔 안 순위. [`meas`] 의 다양성 계측에만 쓴다 —
    /// 중복 판정(`node_key`)에도 순서(`node_order`)에도 들어가지 않는다.
    parent_rank: usize,
    /// ②로 **미리 놓여 소비된** 앞으로의 sequence 자리.
    ///
    /// 노드마다 따로 든다. 한 노드가 짝으로 미리 쓴 자리를 다른 노드는 아직 안 썼을 수
    /// 있으므로 전역 하나로는 표현이 안 된다.
    consumed: SlotSet,
    /// 이 노드가 지금까지 붙인 쌍의 수.
    clusters_applied: usize,
}

/// 노드별 "짝으로 소비된 자리" 집합 — `u64` 비트셋.
///
/// # 왜 `Vec<bool>` 도 `Rc` 도 아닌가 (요구받은 판단)
///
/// 조각 156개 기준 한 노드가 드는 양:
///
/// | 표현 | 바이트 | 복제 비용 |
/// |---|---:|---|
/// | `Vec<bool>` | 156 | 힙 할당 1회 + 156B memcpy |
/// | **`Vec<u64>` 비트셋** | **24** | 힙 할당 1회 + 24B memcpy |
/// | `Rc<RefCell<..>>` 공유 | 8 | 쓰기마다 copy-on-write 분기 |
///
/// 비트셋을 고른 이유는 크기가 아니라 **할당 횟수**다. 셋 다 노드당 할당 1회거나
/// 그 이하라 세 배 차이는 의미가 없고, 어차피 같은 노드가 이미 `pieces`(3.7KB) +
/// `boxes`(5.0KB)를 복제한다 — 24B든 156B든 그 옆에서 계측에 안 잡힌다. `Rc` 는
/// 공유 상태를 쓰기 시점에 갈라야 해서 "노드마다 다르다" 는 이 자료구조의 유일한
/// 요구를 가장 복잡하게 만드는 선택이라 뺐다.
///
/// **②가 꺼져 있으면 `words` 는 끝까지 비어 있어 할당이 아예 없다.** `set` 이 필요할
/// 때만 늘리기 때문이다. 그래서 ⑤ 단독 경로는 이 필드가 생기기 전과 바이트 단위로 같다.
#[derive(Clone, Debug, Default, PartialEq, Eq, Hash)]
struct SlotSet {
    words: Vec<u64>,
}

impl SlotSet {
    /// 이 자리가 짝으로 소비되었는가. 범위 밖은 `false` — 아직 아무도 안 썼다는 뜻이다.
    fn get(&self, slot: usize) -> bool {
        self.words
            .get(slot / 64)
            .is_some_and(|w| (w >> (slot % 64)) & 1 == 1)
    }

    /// 이 자리를 소비 표시한다. 필요한 만큼만 늘린다.
    fn set(&mut self, slot: usize) {
        let word = slot / 64;
        if self.words.len() <= word {
            self.words.resize(word + 1, 0);
        }
        if let Some(w) = self.words.get_mut(word) {
            *w |= 1u64 << (slot % 64);
        }
    }
}

impl BeamNode {
    /// 아무것도 놓지 않은 시작 노드.
    fn root() -> Self {
        Self {
            pieces: Vec::new(),
            boxes: Vec::new(),
            unplaced: Vec::new(),
            strip_length: 0.0,
            waste: 0.0,
            parent_rank: 0,
            consumed: SlotSet::default(),
            clusters_applied: 0,
        }
    }

    /// 이 조각을 놓지 못한 채 깊이만 넘긴 자식.
    fn skipping(&self, depth: usize) -> Self {
        let mut child = self.clone();
        child.unplaced.push(depth);
        child
    }

    /// 이 선택지를 확정한 자식.
    fn extended(&self, option: AnchorOption, metrics: AnchorMetrics) -> Self {
        let mut child = self.clone();
        child.pieces.push(option.to_piece());
        child.boxes.push(option.world);
        // `metrics.strip_length`는 `frontier.max(world.max_x)`라 이미 갱신된 값이다.
        child.strip_length = child.strip_length.max(metrics.strip_length);
        child.waste += metrics.waste.max(0.0);
        child
    }

    /// `from` 부터 끝까지를 미배치로 확정한다 — **②로 이미 놓아 둔 자리는 뺀다.**
    ///
    /// 예산이 끝났거나 자식이 하나도 안 나오면 여기서 배치를 접는데, 그때 남은 구간을
    /// 통째로 `unplaced` 에 넣으면 이 노드가 짝으로 **미리 놓아 버린 앞자리**까지
    /// 들어간다. 그 순간 같은 자리가 배치이자 미배치가 되어
    /// `pieces.len() + unplaced.len() > sequence.len()` 이 되고,
    /// 호출부([`crate::nfp_lbf::search`])가 미배치 개수로 ordering 을 비교하므로
    /// 그 노드를 실제보다 나쁘게 보고 버린다 — 짝을 붙일수록 손해가 되는 뒤집힌 압력이다.
    ///
    /// wsjk156 벤치는 `total_budget` 이 600초라 이 경로에 안 들어가 실측에는 안 잡혔다.
    /// 프로덕션 예산(수 초)에서는 정상적으로 도달한다.
    /// ②가 꺼져 있으면 `consumed` 가 통째로 비어 있어 예전 동작과 바이트 단위로 같다.
    fn give_up_rest(&mut self, from: usize, total: usize) {
        for slot in from..total {
            if !self.consumed.get(slot) {
                self.unplaced.push(slot);
            }
        }
    }

    /// 결과 구조체로 굳힌다.
    fn finish(self, nodes_expanded: usize) -> BeamPlacement {
        BeamPlacement {
            pieces: self.pieces,
            unplaced: self.unplaced,
            strip_length: self.strip_length,
            nodes_expanded,
            clusters_applied: self.clusters_applied,
        }
    }
}

/// 중복 판정 키. `(item_id, 회전, 양자화 x, 양자화 y)`를 정렬한 것 + 미배치 목록
/// + ②로 미리 소비한 자리.
///
/// 해시 대신 키 자체를 들고 다닌다. 깊이당 항목이 16개뿐이라 메모리는 문제가
/// 아니고, 64비트 해시 충돌로 **서로 다른 배치가 조용히 지워지는** 쪽이 훨씬 비싸다.
///
/// 소비 자리가 키에 들어가는 이유: 그림이 같아도 앞으로 **어느 자리를 건너뛸지**가
/// 다르면 다른 상태다. ②가 꺼져 있으면 모든 노드가 빈 벡터라 예전 키와 같다.
type NodeKey = (Vec<(usize, i64, i64, i64)>, Vec<usize>, Vec<u64>);

/// 좌표를 [`DEDUP_GRID_MM`] 격자로 양자화한다.
///
/// 비유한 값은 하나의 통에 몰아넣는다 — 여기서 panic 하지 않는 것이 규칙이고,
/// 애초에 비유한 좌표가 나왔다면 그 노드는 어차피 살아남지 못한다.
fn quantize(v: f64) -> i64 {
    if !v.is_finite() {
        return i64::MIN;
    }
    (v / DEDUP_GRID_MM).round() as i64
}

/// 노드의 중복 판정 키를 만든다.
///
/// 조각 목록을 **정렬**하는 이유: 같은 배치에 서로 다른 순서로 도달할 수 있다.
/// 순서를 그대로 두면 같은 그림이 다른 키가 되어 빔이 사실상 한 상태로 가득 찬다.
fn node_key(node: &BeamNode) -> NodeKey {
    let mut pieces: Vec<(usize, i64, i64, i64)> = node
        .pieces
        .iter()
        .map(|p| {
            (
                p.item_id,
                p.rotation,
                quantize(p.translation.0 as f64),
                quantize(p.translation.1 as f64),
            )
        })
        .collect();
    pieces.sort_unstable();

    let mut unplaced = node.unplaced.clone();
    unplaced.sort_unstable();

    (pieces, unplaced, node.consumed.words.clone())
}

/// 부분 배치의 우열. **1순위 미배치 개수, 2순위 strip 길이, 3순위 총 낭비.**
/// 뒤 둘은 mm 단위 실제 값이다.
///
/// # 왜 미배치 개수가 strip 길이보다 앞인가
///
/// 처음에는 사양 1번 그대로 strip 길이를 1순위로 뒀다가 뒤집었다. 사양 6번이
/// "놓을 수 없으면 `unplaced`로 기록하고 계속 간다"이므로 **같은 깊이에 배치 개수가
/// 서로 다른 노드가 공존한다**. 그 상태에서 strip 길이가 1순위면 **조각을 포기한
/// 노드가 이긴다** — 조각을 안 놓으면 프론티어가 안 늘어나니까. 극단적으로 아무것도
/// 놓지 않은 노드는 `strip_length == 0.0`이라 모든 노드를 이긴다.
///
/// 미배치 조각은 strip 길이로 환산되지 않는 손실이다(그 조각은 다음 스트립을
/// 통째로 열어야 한다). 그러니 먼저 봐야 한다.
///
/// 같은 깊이에서는 `pieces.len() + unplaced.len()`이 항상 그 깊이 값이므로 이 키는
/// "많이 놓은 쪽이 낫다"와 **정확히 같은 순서**다. 빔은 레벨 동기라 깊이가 다른
/// 노드를 섞어 비교하지 않는다.
///
/// 나머지 두 키는 사양대로다. 정규화하지 않는 이유는 [`AnchorMetrics`] 문서와 같고,
/// 길이가 같을 때 낭비로 가르는 이유는 같은 길이라면 안쪽이 촘촘한 쪽이 뒤에 올
/// 조각을 더 받기 때문이다.
fn node_order(a: &BeamNode, b: &BeamNode) -> Ordering {
    a.unplaced
        .len()
        .cmp(&b.unplaced.len())
        .then_with(|| a.strip_length.total_cmp(&b.strip_length))
        .then_with(|| a.waste.total_cmp(&b.waste))
        // 깊이가 다른 노드를 섞어 비교하는 호출자가 훗날 생기더라도 결정론을 잃지
        // 않도록 남겨 둔다. 레벨 동기인 지금은 1순위에 이미 포함된 정보다.
        .then_with(|| b.pieces.len().cmp(&a.pieces.len()))
}

// # 낭비를 더 크게 반영해 보았다 — 더 나빴다
//
// 부분 배치에서 strip 길이는 "조각을 적게 놓은 쪽이 항상 짧다" 는 편향이 있어,
// 낭비를 길이로 환산해(`strip_length + waste / strip_height`, 단위가 맞는 유일한
// 환산이다) 가지치기 2순위로 쓰는 편이 나은지 실험했다. 최종 선택은 목적함수인
// `node_order` 로 남겨 그리디 앵커 보장은 유지한 채, **살릴 노드를 고르는 순서만**
// 바꾼 형태다.
//
// wsjk156·탐침 4096·유효 ordering 12개: 폭 4에서 71.91% -> **71.75%**,
// branch 8 에서도 71.75%. 두 경우 모두 폭이 7089.96mm 로 그리디와 **정확히 같아졌다**
// — 낭비 기준이 빔이 살려 두던 노드를 전부 밀어내 앵커 체인만 남았다는 뜻이다.
// 이득이 없으므로 넣지 않는다.

/// 같은 키를 가진 자식을 **앞엣것만 남기고** 지운다.
///
/// 앞을 남기는 것이 중요하다. 그리디 앵커의 자식을 맨 앞에 넣어 두므로, 다른 부모가
/// 우연히 같은 배치에 도달해도 앵커 쪽이 살아남아 0번 슬롯 보장이 깨지지 않는다.
fn dedup_in_place(children: &mut Vec<BeamNode>) {
    let mut seen: HashSet<NodeKey> = HashSet::with_capacity(children.len());
    children.retain(|node| seen.insert(node_key(node)));
}

/// 후보를 `rule`로 평가해 상위 `k`개를 지표와 함께, **좋은 순서로** 돌려준다.
///
/// 규칙 비교는 [`select::pick_top_k`] 한 곳에만 있다. 빔이 자기 비교를 따로 쓰면
/// ③ 접촉 최대화를 켰을 때 그리디와 빔이 서로 다른 후보를 1순위로 보게 되고,
/// 그 순간 "폭 1은 그리디와 같다"가 조용히 깨진다.
///
/// 인덱스는 `get`으로만 되돌린다 — 오라클과 규칙이 서로 다른 길이를 보게 되는
/// 사고가 나더라도 배치가 죽지 않고 그 후보만 빠진다.
fn rank_options(
    ctx: &StrategyContext<'_, '_>,
    options: &[AnchorOption],
    boxes: &[Aabb],
    frontier: f64,
    rule: SelectionRule,
    k: usize,
) -> Vec<(AnchorOption, AnchorMetrics)> {
    if options.is_empty() || k == 0 {
        return Vec::new();
    }
    let metrics: Vec<AnchorMetrics> = options
        .iter()
        .map(|opt| ctx.metrics(opt, boxes, frontier))
        .collect();

    select::pick_top_k(options, &metrics, rule, k)
        .into_iter()
        .filter_map(|i| Some((*options.get(i)?, *metrics.get(i)?)))
        .collect()
}

/// 주어진 순서를 빔 서치로 배치한다.
///
/// `width`는 각 깊이에서 유지할 부분 배치 수다. `0`은 `1`로 올린다 — 빔이 비면
/// 아무것도 배치할 수 없고, 호출부가 그걸 의도했을 리 없다.
///
/// 예산(`ctx.has_budget()`)이 끝나면 **현재 최선 노드를 그리디로 마저 채우지 않고**
/// 남은 조각을 전부 `unplaced`로 돌려준다. 반쯤 채운 배치를 조용히 완성해 두면
/// 호출부가 "예산 안에서 나온 결과"와 구분하지 못한다.
///
/// # ② 클러스터
///
/// `clusters` 가 비어 있지 않으면, 앵커를 놓은 **그 자식 안에서** 짝까지 놓는다.
/// 짝이 쓴 sequence 자리는 그 노드의 [`SlotSet`] 에 표시되어 그 노드만 건너뛴다 —
/// 형제 노드는 영향을 받지 않는다. 비어 있으면 이 경로는 통째로 꺼져 결과가
/// ② 배선 전과 바이트 단위로 같다.
pub fn place_sequence_beam(
    ctx: &mut StrategyContext<'_, '_>,
    sequence: &[usize],
    width: usize,
    rule: SelectionRule,
    clusters: &[Cluster],
) -> BeamPlacement {
    let width = width.max(1);
    // width == 1 이면 유예할 여지 자체가 없다. branch 를 1로 묶어 0번 슬롯의
    // rule 1순위만 따라가게 하면 결과가 순수 그리디와 비트 단위로 같아진다.
    let branch = branch_for(width);

    let mut beam: Vec<BeamNode> = vec![BeamNode::root()];
    let mut nodes_expanded = 0usize;

    for (depth, &item_id) in sequence.iter().enumerate() {
        if !ctx.has_budget() {
            let mut best = take_best(beam);
            best.give_up_rest(depth, sequence.len());
            return best.finish(nodes_expanded);
        }

        // 0번 노드(그리디 앵커)의 1순위 자식. 점수와 무관하게 반드시 살린다.
        let mut anchor: Option<BeamNode> = None;
        let mut rest: Vec<BeamNode> = Vec::with_capacity(beam.len() * branch);

        // 짝을 붙인 노드와 안 붙인 노드를 둘 다 살릴지. 깊이마다 relaxed load 한 번이라
        // NFP 열거 옆에서는 계측에 잡히지 않는다. 근거는 `pipeline::search_pairs` 문서.
        let keep_both =
            pipeline::search_pairs::get() == pipeline::search_pairs::SearchPairs::KeepBoth;

        for (rank, node) in beam.iter().enumerate() {
            // 이 자리는 **이 노드에서만** 짝으로 이미 놓였다. 깊이만 넘긴다 —
            // 미배치가 아니므로 `unplaced` 에 넣지 않는다.
            if node.consumed.get(depth) {
                let mut passed = node.clone();
                passed.parent_rank = rank;
                if rank == 0 && anchor.is_none() {
                    anchor = Some(passed);
                } else {
                    rest.push(passed);
                }
                continue;
            }

            nodes_expanded += 1;
            // `node.strip_length` 는 놓인 조각들의 `max_x` 최대값이라 프론티어와 같다.
            // 넘겨 주면 `options_for` 가 `world_bounds` 를 다시 만들지 않는다.
            let options = ctx.options_for_at(item_id, &node.pieces, node.strip_length);
            let picks = rank_options(ctx, &options, &node.boxes, node.strip_length, rule, branch);

            if picks.is_empty() {
                // 이 노드에서는 이 조각을 놓을 자리가 없다. 노드를 버리지 않고
                // 미배치로 기록한 채 계속 간다 — 뒤 조각은 더 작을 수 있다.
                let mut skipped = node.skipping(depth);
                skipped.parent_rank = rank;
                if rank == 0 && anchor.is_none() {
                    anchor = Some(skipped);
                } else {
                    rest.push(skipped);
                }
                continue;
            }

            for (idx, (option, metrics)) in picks.into_iter().enumerate() {
                let mut child = node.extended(option, metrics);
                child.parent_rank = rank;

                // ② 짝을 **같은 자식 안에서** 붙인다. 앵커를 놓은 직후가 유일하게
                // 옳은 시점이다 — 그때만 앵커의 실제 자리를 알고 그 옆이 아직 비어 있다.
                let paired = if clusters.is_empty() {
                    None
                } else {
                    attach_pair(ctx, clusters, &option, &child, sequence, depth)
                };

                // 앵커 슬롯(rank 0 의 1순위 자식)은 **그리디와 같은 선택**이어야 한다.
                // 그리디는 붙일 수 있으면 붙이므로, 짝이 붙은 쪽이 앵커다.
                let anchor_slot = rank == 0 && idx == 0 && anchor.is_none();
                match paired {
                    Some(paired_child) => {
                        // 둘 다 살리는 것은 계측 노브다 — 근거는 `search_pairs` 문서.
                        if keep_both {
                            rest.push(child);
                        }
                        if anchor_slot {
                            anchor = Some(paired_child);
                        } else {
                            rest.push(paired_child);
                        }
                    }
                    None if anchor_slot => anchor = Some(child),
                    None => rest.push(child),
                }
            }
        }

        // 앵커를 맨 앞에 두고 중복을 지운다 — 앞엣것이 남으므로 앵커는 무사하다.
        let mut children: Vec<BeamNode> = Vec::with_capacity(rest.len() + 1);
        let had_anchor = anchor.is_some();
        if let Some(a) = anchor {
            children.push(a);
        }
        children.append(&mut rest);
        let raw_children = children.len();
        dedup_in_place(&mut children);
        let dedup_children = children.len();

        if children.is_empty() {
            // 어떤 노드도 자식을 만들지 못했다 — 이론상 도달하지 않지만
            // (자리가 없으면 skip 자식이 나온다) 빔을 비운 채 진행하지는 않는다.
            let mut best = take_best(beam);
            best.give_up_rest(depth, sequence.len());
            return best.finish(nodes_expanded);
        }

        // 0번은 앵커라 정렬에서 제외하고, 나머지에서 상위 width-1 개를 고른다.
        let keep_from = usize::from(had_anchor);
        // 범위 인덱싱 대신 `get_mut` — 앵커 유무가 바뀌어도 여기서 panic 하지 않는다.
        if let Some(tail) = children.get_mut(keep_from..) {
            tail.sort_by(node_order);
        }
        children.truncate(width.max(keep_from));

        if meas::is_on() {
            use std::sync::atomic::Ordering as AO;
            let parents: HashSet<usize> = children.iter().map(|c| c.parent_rank).collect();
            meas::DEPTHS.fetch_add(1, AO::Relaxed);
            meas::BEAM_SIZE.fetch_add(beam.len(), AO::Relaxed);
            meas::CHILD_RAW.fetch_add(raw_children, AO::Relaxed);
            meas::CHILD_DEDUP.fetch_add(dedup_children, AO::Relaxed);
            meas::SURVIVORS.fetch_add(children.len(), AO::Relaxed);
            meas::SURV_PARENTS.fetch_add(parents.len(), AO::Relaxed);
        }

        beam = children;
    }

    take_best(beam).finish(nodes_expanded)
}

/// 방금 앵커를 놓은 자식에 ② 짝까지 붙인 **새 자식**을 만든다.
///
/// 짝을 못 붙이면 `None` 이고, 그 조각은 제 차례(`slot`)에 평범한 조각으로 배치된다 —
/// 아무것도 잃지 않는다.
///
/// 짝이 쓸 자리를 `depth + 1` 부터만 찾는 이유: 그 앞의 자리는 이 노드에서 이미
/// 결말이 났다(놓였거나 미배치이거나 짝으로 소비됐다). 앞을 다시 뒤지면 이미 미배치로
/// 기록한 자리를 짝으로 또 집어 **같은 자리가 미배치이자 배치**가 된다.
fn attach_pair(
    ctx: &mut StrategyContext<'_, '_>,
    clusters: &[Cluster],
    anchor: &AnchorOption,
    child: &BeamNode,
    sequence: &[usize],
    depth: usize,
) -> Option<BeamNode> {
    let (slot, partner) =
        pipeline::attach_partner(ctx, clusters, anchor, &child.pieces, |partner_item| {
            sequence
                .iter()
                .enumerate()
                .skip(depth + 1)
                .find(|&(i, &id)| id == partner_item && !child.consumed.get(i))
                .map(|(i, _)| i)
        })?;

    // 짝도 지표를 거쳐야 strip 길이와 낭비가 노드에 누적된다. 그냥 `pieces` 에만
    // 밀어 넣으면 이 노드가 실제보다 짧은 strip 을 들고 다니며 순위를 훔친다.
    let metrics = ctx.metrics(&partner, &child.boxes, child.strip_length);
    let mut paired = child.extended(partner, metrics);
    paired.consumed.set(slot);
    paired.clusters_applied += 1;
    Some(paired)
}

/// 이 폭에서 노드당 몇 개의 후보로 뻗을지.
///
/// `width == 1` 이면 1이다 — 유예할 여지가 없으므로 0번 슬롯의 `rule` 1순위만 따라가야
/// 결과가 순수 그리디와 비트 단위로 같아진다(테스트로 고정).
///
/// `width >= 2` 면 **폭과 무관하게** [`MAX_BRANCH`] 다. 예전 식
/// `width.clamp(2, MAX_BRANCH)` 는 `width <= 4` 에서 상한이 한 번도 걸리지 않아
/// branch 가 그대로 width 였다 — 폭 2 빔이 노드당 후보를 2개만 보고 자식 4개만 만들었다.
/// 실측표는 [`MAX_BRANCH`] 문서에 있다.
fn branch_for(width: usize) -> usize {
    if width <= 1 { 1 } else { MAX_BRANCH.max(2) }
}

/// 빔에서 가장 좋은 노드를 꺼낸다. 빔이 비어 있으면 빈 노드.
fn take_best(beam: Vec<BeamNode>) -> BeamNode {
    beam.into_iter()
        .min_by(node_order)
        .unwrap_or_else(BeamNode::root)
}

#[cfg(test)]
mod tests {
    use super::*;

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

    // ── 픽스처 ──────────────────────────────────────────────────────────
    //
    // 외부 데이터 파일에 의존하지 않는다. 직사각형만 쓰는 이유는 "빔이 그리디보다
    // 나쁘지 않다"가 기하가 아니라 **탐색 구조**의 성질이기 때문이다.

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

    /// 세 종류의 직사각형이 있는 인스턴스.
    fn fixture_instance() -> SPInstance {
        let strip = Strip::new(STRIP_H, CDE, ShapeModifyConfig::default(), STRIP_CAP)
            .expect(crate::dmsg!("유효한 strip"));
        SPInstance::new(
            vec![
                (rect_item(0, 10.0, 4.0), 3),
                (rect_item(1, 6.0, 9.0), 3),
                (rect_item(2, 4.0, 4.0), 3),
            ],
            strip,
        )
    }

    fn fixture_cache(instance: &SPInstance) -> NfpLbfCache {
        let library = PreparedLibrary::build(instance, &[0], 0.0).expect(crate::dmsg!("라이브러리가 준비된다"));
        NfpLbfCache::new(library)
    }

    /// 배치 순서. 큰 것부터 섞어 그리디가 틈을 남기도록 유도한다.
    const SEQUENCE: [usize; 8] = [1, 0, 2, 1, 0, 2, 1, 0];

    /// 이 테스트가 쓰는 선택 규칙. `rule`이 무엇이든 빔의 성질은 같아야 한다.
    fn fixture_rule() -> SelectionRule {
        SelectionRule::default()
    }

    /// 픽스처 위에서 주어진 폭으로 배치한다.
    fn run_beam(width: usize) -> BeamPlacement {
        let instance = fixture_instance();
        let mut cache = fixture_cache(&instance);
        let mut placer = SequentialPlacer::new(
            &mut cache,
            STRIP_H,
            STRIP_CAP,
            PlacementScoreWeights::default(),
        )
        .with_probe_budget(512);
        let mut ctx =
            StrategyContext::new(&mut placer, DEFAULT_NFP_STRATEGY_CONFIG, Vec::new(), None);
        place_sequence_beam(&mut ctx, &SEQUENCE, width, fixture_rule(), &[])
    }

    /// 빔을 거치지 않은 **순수 그리디** — 매 단계 `rule` 1순위를 그냥 확정한다.
    ///
    /// 후보 선택에 빔의 [`rank_options`]가 아니라 [`select::pick_best`]를 **직접**
    /// 쓴다. 빔이 그리디와 같다는 명제를 빔 자신의 헬퍼로 증명하면, 그 헬퍼가
    /// 틀렸을 때 양쪽이 똑같이 틀려 테스트가 통과해 버린다. 여기서 빔과 공유하는
    /// 것은 상태를 담는 [`BeamNode`]뿐이고, **어느 후보를 고르는지**는 독립이다.
    fn run_greedy() -> BeamPlacement {
        let instance = fixture_instance();
        let mut cache = fixture_cache(&instance);
        let mut placer = SequentialPlacer::new(
            &mut cache,
            STRIP_H,
            STRIP_CAP,
            PlacementScoreWeights::default(),
        )
        .with_probe_budget(512);
        let mut ctx =
            StrategyContext::new(&mut placer, DEFAULT_NFP_STRATEGY_CONFIG, Vec::new(), None);

        let rule = fixture_rule();
        let mut node = BeamNode::root();
        let mut expanded = 0usize;
        for (depth, &item_id) in SEQUENCE.iter().enumerate() {
            expanded += 1;
            let options = ctx.options_for(item_id, &node.pieces);
            let metrics: Vec<AnchorMetrics> = options
                .iter()
                .map(|opt| ctx.metrics(opt, &node.boxes, node.strip_length))
                .collect();
            let picked = select::pick_best(&options, &metrics, rule)
                .and_then(|i| Some((*options.get(i)?, *metrics.get(i)?)));
            match picked {
                Some((option, m)) => node = node.extended(option, m),
                None => node = node.skipping(depth),
            }
        }
        node.finish(expanded)
    }

    fn bbox(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Aabb {
        Aabb {
            min_x,
            min_y,
            max_x,
            max_y,
        }
    }

    fn node_of(pieces: &[(usize, i64, f32, f32)]) -> BeamNode {
        let mut node = BeamNode::root();
        for &(item_id, rotation, x, y) in pieces {
            node.pieces.push(PlacedPiece {
                item_id,
                rotation,
                translation: (x, y),
            });
            node.boxes
                .push(bbox(x as f64, y as f64, x as f64 + 1.0, y as f64 + 1.0));
        }
        node
    }

    // ── 테스트 ──────────────────────────────────────────────────────────

    #[test]
    fn a_beam_of_width_one_is_exactly_the_greedy_placement() {
        let beam = run_beam(1);
        let greedy = run_greedy();

        assert_eq!(
            beam.pieces.len(),
            greedy.pieces.len(),
            "배치 개수가 같아야 한다"
        );
        assert_eq!(beam.unplaced, greedy.unplaced, "미배치 목록이 같아야 한다");
        assert_eq!(
            beam.strip_length, greedy.strip_length,
            "strip 길이가 같아야 한다"
        );
        for (b, g) in beam.pieces.iter().zip(greedy.pieces.iter()) {
            assert_eq!(b.item_id, g.item_id);
            assert_eq!(b.rotation, g.rotation);
            assert_eq!(b.translation, g.translation, "같은 자리에 놓여야 한다");
        }
        // 폭 1이면 깊이마다 노드 하나만 편다.
        assert_eq!(beam.nodes_expanded, SEQUENCE.len());
    }

    #[test]
    fn nodes_reaching_the_same_layout_in_a_different_order_are_collapsed() {
        // 같은 두 조각을 순서만 바꿔 놓은 두 노드 — 그림은 하나다.
        let a = node_of(&[(0, 0, 0.0, 0.0), (1, 0, 10.0, 0.0)]);
        let b = node_of(&[(1, 0, 10.0, 0.0), (0, 0, 0.0, 0.0)]);
        // 0.01mm 격자 안에서 흔들린 사본도 같은 그림이다.
        let c = node_of(&[(0, 0, 0.0, 0.0), (1, 0, 10.002, 0.0)]);
        // 0.05mm 떨어지면 다른 그림이다.
        let d = node_of(&[(0, 0, 0.0, 0.0), (1, 0, 10.05, 0.0)]);

        let mut children = vec![a, b, c, d];
        dedup_in_place(&mut children);
        assert_eq!(children.len(), 2, "같은 배치는 하나로 접혀야 한다");
    }

    #[test]
    fn nodes_with_different_unplaced_sets_are_not_collapsed() {
        // 조각 그림은 같지만 무엇을 포기했는지가 다르면 다른 상태다.
        let mut a = node_of(&[(0, 0, 0.0, 0.0)]);
        let mut b = node_of(&[(0, 0, 0.0, 0.0)]);
        a.unplaced.push(1);
        b.unplaced.push(2);

        let mut children = vec![a, b];
        dedup_in_place(&mut children);
        assert_eq!(children.len(), 2);
    }

    #[test]
    fn a_node_that_gave_up_pieces_never_outranks_one_that_placed_them() {
        // 사양 6번이 "놓을 수 없으면 unplaced 로 기록하고 계속 간다"이므로 같은 깊이에
        // 배치 개수가 다른 노드가 공존한다. strip 길이만 보면 **조각을 포기한 쪽이
        // 이긴다** — 프론티어를 안 늘렸으니까. 극단적으로 아무것도 놓지 않은 노드는
        // strip 0mm 라 무적이 된다. 이 테스트는 그 붕괴를 막는다.
        let mut placed_all = node_of(&[(0, 0, 0.0, 0.0), (1, 0, 10.0, 0.0), (2, 0, 20.0, 0.0)]);
        placed_all.strip_length = 30.0;

        // 하나만 놓고 둘을 포기했다 — strip 은 더 짧다.
        let mut quitter = node_of(&[(0, 0, 0.0, 0.0)]);
        quitter.strip_length = 25.0;
        quitter.unplaced = vec![1, 2];

        // 전부 포기 — strip 0mm.
        let mut gave_up = BeamNode::root();
        gave_up.unplaced = vec![0, 1, 2];

        assert_eq!(
            node_order(&placed_all, &quitter),
            Ordering::Less,
            "strip 이 5mm 길어도 두 조각을 더 놓은 쪽이 이겨야 한다"
        );
        assert_eq!(
            node_order(&placed_all, &gave_up),
            Ordering::Less,
            "strip 0mm 인 빈 노드가 이기면 안 된다"
        );

        let best = take_best(vec![gave_up, quitter, placed_all.clone()]);
        assert_eq!(best.pieces.len(), 3, "빔의 최선은 전부 놓은 노드다");
        assert!(best.unplaced.is_empty());
    }

    #[test]
    fn among_nodes_that_placed_the_same_count_the_shorter_strip_still_wins() {
        // 위 수정이 과해서 strip 길이가 순위에서 밀려나면 빔이 통째로 무의미해진다.
        // 미배치 개수가 같을 때는 사양 1번대로 strip -> 낭비 순이어야 한다.
        let mut short = node_of(&[(0, 0, 0.0, 0.0)]);
        short.strip_length = 20.0;
        short.waste = 100.0;

        let mut long = node_of(&[(0, 0, 5.0, 0.0)]);
        long.strip_length = 25.0;
        long.waste = 0.0;

        assert_eq!(
            node_order(&short, &long),
            Ordering::Less,
            "미배치가 같으면 strip 이 1순위다 — 낭비가 커도 짧은 쪽이 이긴다"
        );

        // strip 까지 같으면 낭비가 2순위다.
        let mut tie = short.clone();
        tie.strip_length = 20.0;
        tie.waste = 5.0;
        assert_eq!(
            node_order(&tie, &short),
            Ordering::Less,
            "strip 이 같으면 낭비가 적은 쪽이 이긴다"
        );
    }

    #[test]
    fn an_empty_sequence_yields_an_empty_placement_without_panicking() {
        let instance = fixture_instance();
        let mut cache = fixture_cache(&instance);
        let mut placer = SequentialPlacer::new(
            &mut cache,
            STRIP_H,
            STRIP_CAP,
            PlacementScoreWeights::default(),
        );
        let mut ctx =
            StrategyContext::new(&mut placer, DEFAULT_NFP_STRATEGY_CONFIG, Vec::new(), None);

        let out = place_sequence_beam(&mut ctx, &[], 4, fixture_rule(), &[]);
        assert!(out.pieces.is_empty());
        assert!(out.unplaced.is_empty());
        assert_eq!(out.strip_length, 0.0);
        assert_eq!(out.nodes_expanded, 0);
    }

    #[test]
    fn an_expired_budget_leaves_every_piece_unplaced() {
        use jagua_rs::Instant;

        let instance = fixture_instance();
        let mut cache = fixture_cache(&instance);
        let mut placer = SequentialPlacer::new(
            &mut cache,
            STRIP_H,
            STRIP_CAP,
            PlacementScoreWeights::default(),
        );
        // 이미 지난 시각을 마감으로 준다. `Instant::now()`를 그대로 쓰면 `has_budget`의
        // `Instant::now() < deadline`이 **같은 틱 안에서 거짓이 되는 데 의존**하게 되어
        // 타이머 해상도가 거친 환경에서 흔들린다. 1분을 빼서 여유를 둔다.
        // (부팅 직후라 뺄 수 없으면 그때는 now() 로 되돌아간다 — panic 하지 않는다.)
        let past = Instant::now()
            .checked_sub(std::time::Duration::from_secs(60))
            .unwrap_or_else(Instant::now);
        let mut ctx = StrategyContext::new(
            &mut placer,
            DEFAULT_NFP_STRATEGY_CONFIG,
            Vec::new(),
            Some(past),
        );

        let out = place_sequence_beam(&mut ctx, &SEQUENCE, 4, fixture_rule(), &[]);
        assert!(out.pieces.is_empty(), "예산이 없으면 아무것도 놓지 않는다");
        assert_eq!(
            out.unplaced,
            (0..SEQUENCE.len()).collect::<Vec<_>>(),
            "남은 조각이 전부 미배치로 나와야 한다"
        );
        assert_eq!(out.nodes_expanded, 0, "확장한 노드가 없다");
    }

    #[test]
    fn a_zero_width_beam_falls_back_to_a_single_node() {
        // 폭 0은 빔을 비운다 — 그건 호출부의 의도일 수 없으므로 1로 올린다.
        let zero = run_beam(0);
        let one = run_beam(1);
        assert_eq!(zero.strip_length, one.strip_length);
        assert_eq!(zero.pieces.len(), one.pieces.len());
    }

    /// 임의의 strip 높이·조각 치수·순서로 한 번 배치한다.
    fn run_case(h: f32, dims: &[(f32, f32)], seq: &[usize], width: usize) -> BeamPlacement {
        let strip =
            Strip::new(h, CDE, ShapeModifyConfig::default(), STRIP_CAP).expect(crate::dmsg!("유효한 strip"));
        let items: Vec<(std::sync::Arc<Item>, usize)> = dims
            .iter()
            .enumerate()
            .map(|(id, &(w, item_h))| (rect_item(id, w, item_h), seq.len()))
            .collect();
        let instance = SPInstance::new(items, strip);
        let library = PreparedLibrary::build(&instance, &[0], 0.0).expect(crate::dmsg!("라이브러리가 준비된다"));
        let mut cache = NfpLbfCache::new(library);
        let mut placer =
            SequentialPlacer::new(&mut cache, h, STRIP_CAP, PlacementScoreWeights::default())
                .with_probe_budget(2048);
        let mut ctx =
            StrategyContext::new(&mut placer, DEFAULT_NFP_STRATEGY_CONFIG, Vec::new(), None);
        place_sequence_beam(&mut ctx, seq, width, fixture_rule(), &[])
    }

    /// 서로 다른 여섯 개의 (strip 높이, 조각 치수, 순서) 조합.
    fn guarantee_cases() -> Vec<(f32, Vec<(f32, f32)>, Vec<usize>)> {
        vec![
            (
                20.0,
                vec![(10.0, 4.0), (6.0, 9.0), (4.0, 4.0)],
                vec![1, 0, 2, 1, 0, 2, 1, 0],
            ),
            (
                10.0,
                vec![(6.0, 7.0), (5.0, 4.0), (3.0, 9.0)],
                vec![0, 1, 2, 1, 0, 2, 1, 0],
            ),
            (
                12.0,
                vec![(7.0, 5.0), (4.0, 11.0), (9.0, 3.0)],
                vec![0, 1, 2, 0, 1, 2, 0, 1, 2],
            ),
            (
                9.0,
                vec![(5.0, 5.0), (3.0, 8.0), (8.0, 2.0)],
                vec![1, 2, 0, 1, 2, 0, 1, 2, 0],
            ),
            (
                15.0,
                vec![(11.0, 6.0), (5.0, 13.0), (6.0, 4.0), (3.0, 3.0)],
                vec![0, 1, 2, 3, 0, 1, 2, 3, 0, 1],
            ),
            (
                7.0,
                vec![(4.0, 6.0), (6.0, 3.0), (2.0, 5.0)],
                vec![0, 1, 2, 0, 1, 2, 0, 1, 2, 0],
            ),
        ]
    }

    #[test]
    fn widening_the_beam_never_lengthens_the_strip_on_any_fixture() {
        // 단일 픽스처 하나로는 "우연히 같았다"와 구분되지 않으므로 여섯 조합을 돈다.
        //
        // 실측(직사각형 + `SelectionRule::LeftBottom`): 여섯 조합 모두 폭 1/4/8의
        // strip 길이가 **소수점까지 동일**했다 (24.021 / 34.061 / 37.051 / 33.051 /
        // 37.041 / 34.081 mm). 직사각형에서는 좌하단 그리디가 이미 최선에 가까워
        // 빔이 이길 여지가 거의 없다는 뜻이다. 그래도 이 테스트가 지키는 명제는
        // "이겨야 한다"가 아니라 **"지면 안 된다"**이고, 그게 그리디 앵커 슬롯이
        // 보장하는 것이다. 앵커가 깨지면 여기서 바로 드러난다.
        // 보장은 `node_order` 의 사전식 순서 그대로 검사한다: **미배치 개수가 먼저,
        // 그다음이 strip 길이**. strip 길이만 보면 조각을 포기해 짧아진 결과가 통과해
        // 버리기 때문이다.
        for (ci, (h, dims, seq)) in guarantee_cases().iter().enumerate() {
            let greedy = run_case(*h, dims, seq, 1);
            // 여섯 조합 모두 그리디가 전부 놓는다. 이걸 고정해 두지 않으면, 언젠가
            // 픽스처가 배치 불가능해졌을 때 아래 비교가 "둘 다 아무것도 못 놓았다"로
            // **공허하게** 통과한다.
            assert!(
                greedy.unplaced.is_empty() && greedy.pieces.len() == seq.len(),
                "case{ci}: 픽스처가 그리디로 전부 배치되지 않는다 — 비교가 공허해진다"
            );

            for width in [2usize, 3, 4, 8] {
                let wide = run_case(*h, dims, seq, width);
                assert!(
                    wide.unplaced.len() <= greedy.unplaced.len(),
                    "case{ci}: 폭 {width}가 그리디보다 많이 포기했다 ({} > {})",
                    wide.unplaced.len(),
                    greedy.unplaced.len()
                );
                if wide.unplaced.len() == greedy.unplaced.len() {
                    assert!(
                        wide.strip_length <= greedy.strip_length + 1e-9,
                        "case{ci}: 폭 {width}의 strip {} 가 그리디 {} 보다 길다",
                        wide.strip_length,
                        greedy.strip_length
                    );
                }
                assert!(
                    wide.pieces.len() >= greedy.pieces.len(),
                    "case{ci}: 폭 {width}가 그리디보다 적게 놓았다"
                );
            }
        }
    }

    #[test]
    fn a_wider_beam_actually_keeps_more_than_one_distinct_node() {
        // 중복 제거가 과하면 빔이 매 깊이 한 노드로 접혀 위 보장이 **공허하게** 참이 된다.
        // 확장한 노드 수가 깊이 수보다 많다는 것은 살아남은 서로 다른 상태가 있었다는 뜻이다.
        //
        // 실측(기본 픽스처, 깊이 8): 폭 1 -> 8, 폭 2 -> 15, 폭 4 -> 29, 폭 8 -> 53.
        let single = run_beam(1);
        assert_eq!(single.nodes_expanded, SEQUENCE.len());

        let wide = run_beam(4);
        assert!(
            wide.nodes_expanded > single.nodes_expanded,
            "폭 4가 폭 1과 같은 수({})만 확장했다 — 빔이 한 상태로 접혔다",
            wide.nodes_expanded
        );
    }

    // ────────────────────────────────────────────────────────────────────
    // 실 인스턴스 계측 하네스 (여성WSjk 156-패널). 자동 회귀가 아니라 **측정**이라
    // `#[ignore]` 다 — 한 번 도는 데 수십 초가 걸린다. 위 문서의 실측표를 다시 뽑는
    // 방법이기도 하다.
    // ────────────────────────────────────────────────────────────────────

    /// 벤치와 같은 인스턴스(여성WSjk 156-패널).
    fn load_marker() -> SPInstance {
        let path =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input/wsjk156.json");
        let (ext, _) = crate::util::io::read_spp_input(&path).expect(crate::dmsg!("wsjk156.json 읽기"));
        let c = crate::config::DEFAULT_SWARM_CONFIG;
        let imp = crate::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
        jagua_rs::probs::spp::io::import_instance(&imp, &ext).expect(crate::dmsg!("import"))
    }

    /// 벤치와 같은 조건(유효 ordering 12개, 탐침 4096)으로 한 팔을 돌린다.
    fn run_arm(
        inst: &SPInstance,
        strategy: crate::nfp_strategy::NfpStrategyConfig,
    ) -> (f32, f32, u128, usize) {
        use crate::nfp_lbf::{NfpLbfConfig, build_best_initial_nest};
        use rand::SeedableRng;
        use rand::rngs::Xoshiro256PlusPlus;

        let factor = strategy.estimated_cost_factor().max(1.0);
        let cfg = NfpLbfConfig {
            n_orderings: (12.0 * factor).ceil() as usize,
            cache_budget: std::time::Duration::from_secs(10),
            total_budget: std::time::Duration::from_secs(900),
            concave_simplify_tolerance: 0.02,
            // 탐침 예산은 이 모듈이 재는 모든 값의 **전제**다(③은 4096에서 지고
            // 8192에서 이긴다). 상수로 박아 두면 "다른 예산에서도 그런가" 를 코드를
            // 고쳐야만 물을 수 있다 — `select::tests::run_arm` 과 같은 이름의 노브를 둔다.
            max_candidate_probes: std::env::var("MEASURE_PROBES")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(4096),
            strategy,
            ..NfpLbfConfig::default()
        };
        let seed = std::env::var("MEASURE_SEED")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(0u64);
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(seed);
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

    // ── ② 클러스터 배선 ─────────────────────────────────────────────────

    /// 픽스처 위에서 클러스터까지 주고 배치한다.
    fn run_beam_with_clusters(width: usize, clusters: &[Cluster]) -> BeamPlacement {
        let instance = fixture_instance();
        let mut cache = fixture_cache(&instance);
        let mut placer = SequentialPlacer::new(
            &mut cache,
            STRIP_H,
            STRIP_CAP,
            PlacementScoreWeights::default(),
        )
        .with_probe_budget(512);
        let mut ctx =
            StrategyContext::new(&mut placer, DEFAULT_NFP_STRATEGY_CONFIG, Vec::new(), None);
        place_sequence_beam(&mut ctx, &SEQUENCE, width, fixture_rule(), clusters)
    }

    /// item 0(10x4)을 자기 자신 바로 위에 4mm 올려 쌓는 손수 만든 클러스터.
    ///
    /// `cluster::build_clusters` 를 부르지 않는 이유: 이 테스트가 묻는 것은 "직사각형에서
    /// 쌍이 발견되는가" 가 아니라 **"쌍이 있으면 빔이 그것을 쓰는가"** 다. 발견기를 끼우면
    /// 발견기가 0쌍을 내는 순간 이 테스트가 공허하게 통과한다.
    fn stacking_cluster() -> Cluster {
        Cluster {
            anchor_item: 0,
            anchor_rotation: 0,
            partner_item: 0,
            partner_rotation: 0,
            offset: (0.0, 4.0),
            fill_ratio: 1.0,
            partner_bounds: bbox(0.0, 0.0, 10.0, 4.0),
        }
    }

    #[test]
    fn a_cluster_partner_is_placed_inside_the_same_beam_node() {
        let out = run_beam_with_clusters(2, &[stacking_cluster()]);
        assert!(
            out.clusters_applied > 0,
            "짝이 하나도 안 붙었다 — ②가 ⑤에 배선되지 않았다는 뜻이다"
        );
        // 한 자리는 한 번만 결말을 갖는다. 짝이 미리 놓인 자리를 본 루프가 다시
        // 처리하면 조각이 수요보다 많아지고 이 합이 깨진다.
        assert_eq!(
            out.pieces.len() + out.unplaced.len(),
            SEQUENCE.len(),
            "자리 수가 안 맞는다 — 어떤 자리가 두 번 소비됐다"
        );
    }

    #[test]
    fn an_empty_cluster_list_leaves_the_beam_exactly_as_it_was() {
        // ②를 끄면 소비 비트가 통째로 비어 있어야 한다 — 그래야 ⑤ 단독 실측값
        // (72.10%)이 이 배선 때문에 흔들리지 않는다.
        let with_none = run_beam_with_clusters(2, &[]);
        let plain = run_beam(2);
        assert_eq!(with_none.strip_length, plain.strip_length);
        assert_eq!(with_none.pieces.len(), plain.pieces.len());
        assert_eq!(with_none.clusters_applied, 0);
        assert_eq!(with_none.nodes_expanded, plain.nodes_expanded);
    }

    #[test]
    fn a_consumed_slot_set_only_allocates_when_something_is_consumed() {
        let mut set = SlotSet::default();
        assert!(set.words.is_empty(), "빈 집합은 할당이 없어야 한다");
        assert!(!set.get(0));
        // 범위를 한참 넘겨 물어봐도 panic 하지 않고 false 다.
        assert!(!set.get(10_000));

        set.set(3);
        set.set(155);
        assert!(set.get(3) && set.get(155));
        assert!(!set.get(4) && !set.get(154) && !set.get(156));
        // 156자리를 담는 데 u64 세 개면 충분하다 — `Vec<bool>` 156바이트 대비 24바이트.
        assert_eq!(set.words.len(), 3);
    }

    #[test]
    fn giving_up_the_rest_does_not_report_an_already_paired_slot_as_unplaced() {
        // 예산이 끝나 깊이 2에서 접는 상황. 이 노드는 ②로 4번 자리를 **이미 놓았다**.
        // 남은 구간을 통째로 미배치에 넣으면 4번이 배치이자 미배치가 되어
        // `pieces + unplaced` 가 sequence 길이를 넘고, 호출부가 미배치 개수로
        // ordering 을 비교하므로 짝을 붙인 노드가 오히려 손해를 본다.
        let mut node = node_of(&[(0, 0, 0.0, 0.0), (1, 0, 10.0, 0.0), (2, 0, 20.0, 0.0)]);
        node.consumed.set(4);
        node.give_up_rest(2, 6);
        assert_eq!(
            node.unplaced,
            vec![2, 3, 5],
            "짝으로 놓은 4번이 빠져야 한다"
        );
        assert_eq!(
            node.pieces.len() + node.unplaced.len(),
            6,
            "자리 수가 안 맞는다 — 어떤 자리가 두 번 결말을 가졌다"
        );

        // ②가 꺼져 있으면(소비 비트가 비어 있으면) 예전과 똑같이 전 구간이 들어간다.
        let mut plain = node_of(&[(0, 0, 0.0, 0.0)]);
        plain.give_up_rest(1, 4);
        assert_eq!(plain.unplaced, vec![1, 2, 3]);
    }

    #[test]
    fn two_nodes_that_consumed_different_future_slots_are_not_collapsed() {
        // 그림도 미배치도 같지만 앞으로 건너뛸 자리가 다르면 다른 상태다.
        let mut a = node_of(&[(0, 0, 0.0, 0.0)]);
        let mut b = node_of(&[(0, 0, 0.0, 0.0)]);
        a.consumed.set(5);
        b.consumed.set(6);

        let mut children = vec![a.clone(), b, a];
        dedup_in_place(&mut children);
        assert_eq!(children.len(), 2, "같은 자리를 소비한 둘만 접혀야 한다");
    }

    #[test]
    fn the_branch_factor_is_not_tied_to_the_beam_width() {
        // 폭 1만 특별하다(그리디와 비트 단위로 같아야 하므로).
        assert_eq!(branch_for(0), 1);
        assert_eq!(branch_for(1), 1);
        // 나머지는 폭과 무관한 상수다. 예전 식 `width.clamp(2, MAX_BRANCH)` 는
        // 폭 2에서 2, 폭 3에서 3을 줘서 좁은 빔이 후보를 거의 못 봤다.
        for width in [2usize, 3, 4, 8, 64] {
            assert_eq!(
                branch_for(width),
                MAX_BRANCH,
                "폭 {width}의 확장 폭이 폭에 끌려 다닌다"
            );
        }
    }

    #[test]
    #[ignore = "실 인스턴스 필요 — 계측용"]
    fn measure_beam_on_the_real_marker() {
        use crate::nfp_strategy::NfpStrategyConfig;

        let inst = load_marker();
        let off = crate::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG;
        let width: usize = std::env::var("NFP_BEAM_WIDTH")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(4);

        meas::set(true);
        meas::reset();
        let r = run_arm(
            &inst,
            NfpStrategyConfig {
                beam_search: true,
                beam_width: width,
                ..off
            },
        );
        let rep = meas::report();
        meas::set(false);
        println!(
            // `MAX_BRANCH` 가 아니라 **실제로 쓴** branch 를 찍는다. 이 표의 행 이름이
            // 곧 측정 조건인데, 상수를 찍으면 폭 1(branch 1)이 "branch 4" 로 기록된다
            // — 조건을 잘못 적은 측정치는 없는 것만 못하다.
            "\n[⑤ 계측] 폭 {width} branch {} -> 폭 {:.2} 밀도 {:.2}% {}ms\n{rep}",
            branch_for(width),
            r.0,
            r.1,
            r.2
        );
    }
}
