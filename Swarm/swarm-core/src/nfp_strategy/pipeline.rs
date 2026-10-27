//! 켜진 전략들을 하나의 배치 파이프라인으로 조합한다.
//!
//! ```text
//! ② 클러스터링 (전처리, 호출부가 미리 만들어 넘긴다)
//!      v
//! ⑥ best-fit 선택  또는  ⑤ 빔 서치  또는  그리디 1패스
//!      |                      |                  |
//!      +----------------------+------------------+
//!                             v
//!               ④ 포켓 우선 (오라클의 탐침 순서)
//!               ③ 접촉 최대화 (후보 선택 규칙)
//!                             v
//!                      ⑦ 슬라이딩 압축 (후처리)
//! ```
//!
//! ①(인스턴스 회전)은 여기 나타나지 않는다. NFP 라이브러리를 **만들기 전에** 회전
//! 집합을 정해야 하므로 호출부([`crate::nfp_lbf::build_best_initial_nest`])가 처리한다.
//!
//! # 배치 방식이 셋으로 갈리는 이유
//!
//! ⑤와 ⑥은 둘 다 "다음에 무엇을 어디에" 를 정한다. 함께 켜면 빔의 각 노드에서
//! best-fit 을 돌려야 하는데, 비용이 `빔 폭 x 남은 조각 수` 로 곱해지는 데다
//! 빔 노드마다 남은 조각 집합이 달라져 상태가 급격히 커진다. 실익이 불분명한 조합에
//! 그 비용을 쓰는 대신 **⑥을 우선**하고 그 사실을 로그로 알린다.
//!
//! # ②는 세 경로 모두에 **배선되어 있지만, ⑤/⑥ 에서는 기본으로 꺼져 있다**
//!
//! 셋 다 [`attach_partner`] 를 부를 수 있다 — 붙이는 시점과 자리를 빼는 방법만 다르다.
//!
//! | 경로 | 짝을 붙이는 시점 | 짝이 쓸 자리를 무엇에서 빼는가 |
//! |---|---|---|
//! | 그리디 | 앵커를 확정한 직후 | 파이프라인 전역 `consumed` 비트 |
//! | ⑤ 빔 | 자식 노드를 만든 직후 | **노드마다 따로 든** 소비 비트(`BeamNode::consumed`) |
//! | ⑥ best-fit | 조각을 고른 직후 | `remaining`/`origin` 두 벡터에서 같이 제거 |
//!
//! 빔만 상태를 노드별로 드는 이유는 노드마다 남은 조각 집합이 다르기 때문이다.
//! 한 노드가 짝으로 미리 쓴 자리를 다른 노드가 아직 안 썼을 수 있다.
//!
//! ⑤/⑥ 쪽을 기본으로 끈 이유는 **재 봤더니 졌기 때문**이다. 숫자는 [`search_pairs`]
//! 문서에 있다.

use jagua_rs::Instant;
use jagua_rs::probs::spp::entities::SPInstance;

use crate::nfp_lbf::{PlacedPiece, SequentialPlacement};
use crate::nfp_strategy::cluster::{self, Cluster};
use crate::nfp_strategy::config::PlacementStrategy;
use crate::nfp_strategy::select::{self, SelectionRule};
use crate::nfp_strategy::{AnchorOption, StrategyContext, accrete, beam, reflow, sliding};

/// 클러스터 파트너를 실제 후보에 맞춰 붙일 때 허용할 어긋남(mm).
///
/// [`cluster::expand_cluster`]가 주는 위치는 **앵커 하나만** 보고 계산한 상대 좌표라,
/// 다른 조각이 이미 그 근처를 차지했으면 그 좌표가 후보 목록에 없을 수 있다.
/// 그래서 정확히 일치하는 후보를 찾는 대신 이 반경 안의 가장 가까운 후보로 스냅한다.
///
/// 반경을 크게 잡으면 "쌍을 유지했다" 는 말이 무의미해지므로 조각 크기에 비해 아주
/// 작게 잡는다. 스냅에 실패하면 파트너는 평범한 조각으로 나중에 배치된다.
///
/// [`cluster::expand_cluster`]: crate::nfp_strategy::cluster::expand_cluster
const CLUSTER_SNAP_MM: f64 = 0.5;

/// ⑤/⑥ 경로에서 ② 쌍을 쓸지 정하는 노브.
///
/// # 왜 설정(`NfpStrategyConfig`)이 아니라 여기인가
///
/// 이건 사용자가 고를 전략이 아니라 **한 번 재고 끝낸 질문**이다. 설정 플래그로 두면
/// "②와 ⑤를 같이 켜면 어떻게 되나" 를 사용자가 매번 다시 물어야 하고, 아래 표가
/// 코드에서 멀어진다. 재현하려는 사람만 이 노브를 만진다(벤치가 그렇게 쓴다).
///
/// # 실측 — ⑤/⑥ 에서 ②는 진다 (wsjk156, 탐침 4096, 유효 ordering 12개, seed 0)
///
/// | 팔 | [`SearchPairs::Off`] | [`SearchPairs::Replace`] | [`SearchPairs::KeepBoth`] |
/// |---|---:|---:|---:|
/// | ②+⑤ (폭 2) | **72.10%** / 10.0s / 쌍 0 | 71.27% / 19.2s / 쌍 13 | 71.27% / 21.5s / 쌍 13 |
/// | ②+⑥ (12) | **71.87%** / 30.0s / 쌍 0 | 70.04% / 27.6s / 쌍 12 | — |
/// | ②+③+④+⑤+⑦ | 72.18% / 36.7s / 쌍 0 | 72.18% / 39.0s / 쌍 13 | 72.18% / 37.7s / 쌍 13 |
///
/// 시간은 참고값이다 — 이 워크스페이스는 여러 작업이 같은 target 을 공유해 같은 팔이
/// 실행마다 ±40% 흔들렸다(②+③+④+⑤+⑦ Replace 가 56.4s 와 39.0s). 밀도와 폭은
/// seed 0 고정이라 소수점까지 재현된다.
///
/// `Off` 열은 쌍이 0개라 각각 ⑤ 단독·⑥ 단독과 같은 값이다. 배선이 실제로 동작한다는
/// 것은 나머지 두 열의 "쌍 13개 / 12개" 로 확인했다 — 0이면 배선이 안 된 것이다.
/// 벤치에서 세 열을 나란히 재는 팔이 `nfp_strategy_bench` 에 있다
/// (`BENCH_ARMS='쌍 안 넘김,쌍 넘김,둘 다'`).
///
/// 읽는 법: ②를 ⑤에 넣으면 **-0.83%p**, ⑥에 넣으면 **-1.83%p** 다. 두 경우 모두
/// 시간은 늘었다. `②+⑤` 결과가 `② 단독`(71.27%)과 소수점까지 같은 것이 원인을
/// 말해 준다 — 짝을 강제하면 빔의 자식들이 서로 같아져 **그리디 앵커 체인 말고는
/// 살아남는 노드가 없다**. 빔의 존재 이유(선택을 유예한다)가 사라진다.
///
/// [`SearchPairs::KeepBoth`] 는 그 유예를 되살리려는 시도였다 — 짝을 붙인 노드와 안
/// 붙인 노드를 둘 다 자식으로 넣는다. 결과는 소수점까지 같았다. [`beam`] 의
/// `node_order` 는 같은 깊이를 `(미배치 수, strip 길이, 낭비)` 로 비교하는데, 짝을
/// 붙인 노드는 조각을 하나 더 놓아 strip 이 같거나 길다. 그래서 안 붙인 노드가 매
/// 깊이 살아남지만, 그 노드도 결국 그 조각을 놓아야 하므로 마지막에는 도로 만난다.
///
/// # 덧붙여 — ②는 "최고 조합" 에서도 공이 없다
///
/// 같은 조건에서 `③+④+⑦` (②를 뺀 것) 이 **72.18% / 폭 7047.66mm**, `②+③+④+⑦` 이
/// 72.18% / 폭 7047.29mm 다. 차이는 0.37mm — 밀도로는 소수점 둘째 자리 아래다.
/// ②가 "최고 조합" 의 일원인 것은 **해를 끼치지 않아서**이지 보태서가 아니다.
/// ②는 그리디 단독에서도 -0.48%p(71.27% vs 71.75%)이므로, 이 마커에서 ②가 이긴
/// 국면은 아직 하나도 없다.
#[doc(hidden)]
pub mod search_pairs {
    use std::sync::atomic::{AtomicU8, Ordering as AO};

    /// ⑤/⑥ 이 ② 쌍을 다루는 방식.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    pub enum SearchPairs {
        /// 쌍을 넘기지 않는다. ⑤/⑥ 은 ② 없이 돌던 그대로다. **기본값.**
        #[default]
        Off,
        /// 짝을 붙일 수 있으면 붙인다. 그리디 경로와 같은 선택이다.
        Replace,
        /// ⑤ 전용 — 짝을 붙인 노드와 안 붙인 노드를 **둘 다** 자식으로 남긴다.
        /// ⑥에서는 [`SearchPairs::Replace`] 와 같게 동작한다(고를 자식이 없다).
        KeepBoth,
    }

    static MODE: AtomicU8 = AtomicU8::new(0);

    pub fn set(mode: SearchPairs) {
        let v = match mode {
            SearchPairs::Off => 0,
            SearchPairs::Replace => 1,
            SearchPairs::KeepBoth => 2,
        };
        MODE.store(v, AO::Relaxed);
    }

    pub fn get() -> SearchPairs {
        // 모르는 값은 기본값으로 눕힌다 — 여기서 panic 할 이유가 없다.
        match MODE.load(AO::Relaxed) {
            1 => SearchPairs::Replace,
            2 => SearchPairs::KeepBoth,
            _ => SearchPairs::Off,
        }
    }

    /// 쌍을 ⑤/⑥ 에 넘기는가.
    pub fn enabled() -> bool {
        get() != SearchPairs::Off
    }
}

/// 전략 파이프라인의 배치 결과.
#[derive(Clone, Debug, Default)]
pub struct StrategyPlacement {
    /// 놓인 조각들.
    pub pieces: Vec<PlacedPiece>,
    /// 놓지 못한 `sequence` 인덱스.
    pub unplaced: Vec<usize>,
    /// 최종 strip 길이(mm).
    pub strip_length: f64,
    /// ②가 실제로 붙인 쌍의 수. 0이면 클러스터가 하나도 성립하지 않았다는 뜻이다.
    pub clusters_applied: usize,
    /// ⑦이 실제로 옮긴 조각 수.
    pub pieces_slid: usize,
}

impl StrategyPlacement {
    /// 기존 파이프라인이 쓰는 표현으로 옮긴다.
    ///
    /// `materialize` 와 ordering 평가가 전부 이 타입을 보므로, 전략 경로도 여기로
    /// 합류시켜야 검증(`solution_validation_error`)을 똑같이 통과한다.
    pub fn into_sequential(self) -> SequentialPlacement {
        SequentialPlacement {
            placed: self.pieces.len(),
            pieces: self.pieces,
            strip_length: self.strip_length as f32,
            unplaced: self.unplaced,
        }
    }
}

/// 켜진 전략들로 하나의 순서를 배치한다.
///
/// `sequence` 는 수요만큼 반복된 `item_id` 목록이다. ⑥이 켜지면 순서는 무시되고
/// **남은 조각의 다중집합**으로만 쓰인다 — 그게 best-fit 의 정의다.
pub fn place_with_strategies(
    ctx: &mut StrategyContext<'_, '_>,
    sequence: &[usize],
    clusters: &[Cluster],
) -> StrategyPlacement {
    let rule = SelectionRule::from_contact_maximization(ctx.cfg.contact_maximization);
    let cfg = ctx.cfg.normalized();
    let strategy = cfg.placement_strategy();

    // ⑤/⑥ 이 볼 쌍. 노브가 꺼져 있으면 빈 목록이라 그 경로는 ② 없이 돌던 그대로다.
    // 왜 기본이 꺼짐인지는 `search_pairs` 문서의 실측표에 있다 (⑤ -0.83%p, ⑥ -1.83%p).
    let search_clusters: &[Cluster] = if search_pairs::enabled() {
        clusters
    } else {
        &[]
    };

    let mut out = if strategy == PlacementStrategy::Accretion {
        // ⑧ 누적 배치 — `sequence` 를 순서가 아니라 **다중집합**으로 쓴다.
        // 오라클을 거치지 않고 캐시를 직접 읽지만, strip 높이·폭 상한·점수 가중치를
        // 오라클에서 그대로 받아 오므로 두 경로의 점수는 같은 눈금이다.
        let outcome = accrete::accrete(
            ctx.placer.cache(),
            sequence,
            ctx.placer.strip_height(),
            ctx.placer.strip_width_cap(),
            ctx.placer.weights(),
            &ctx.item_rotations,
            ctx.placer.probe_budget(),
            rule,
            ctx.deadline,
        );
        StrategyPlacement {
            pieces: outcome.pieces,
            unplaced: outcome.unplaced,
            strip_length: outcome.strip_length,
            ..Default::default()
        }
    } else if strategy == PlacementStrategy::BestFit {
        place_best_fit(ctx, sequence, rule, cfg.best_fit_lookahead, search_clusters)
    } else if strategy == PlacementStrategy::Beam {
        let beamed =
            beam::place_sequence_beam(ctx, sequence, cfg.beam_width, rule, search_clusters);
        StrategyPlacement {
            pieces: beamed.pieces,
            unplaced: beamed.unplaced,
            strip_length: beamed.strip_length,
            clusters_applied: beamed.clusters_applied,
            ..Default::default()
        }
    } else {
        place_greedy(ctx, sequence, rule, clusters)
    };

    // ⑦ 슬라이딩 압축 (후처리)
    if cfg.sliding_compaction && !out.pieces.is_empty() {
        let outcome = sliding::compact(ctx, &out.pieces, cfg.sliding_rounds);
        out.pieces_slid = outcome.moved;
        out.strip_length = outcome.after;
        out.pieces = outcome.pieces;
    }

    // ⑨ 꼬리 회수 (후처리) — ⑦ **뒤**에 온다.
    //
    // 순서가 중요하다. ⑦이 먼저 조각을 좌하단으로 밀어 붙여야 앞쪽에 회수 가능한
    // 포켓이 또렷해지고, 그 상태에서 꼬리를 빼야 갈 자리가 생긴다. 뒤집으면 ⑨가
    // 옮긴 조각을 ⑦이 다시 흩는다.
    if cfg.tail_reflow && !out.pieces.is_empty() {
        let outcome = reflow::reflow(ctx, &out.pieces, cfg.reflow_rounds, rule);
        // 미배치가 남았다면 원복 경로가 깨진 것이다(모듈 문서의 계약).
        // 조각을 잃는 것은 폭을 못 줄인 것보다 훨씬 나쁘므로 결과를 통째로 버린다.
        if outcome.unplaced.is_empty() {
            out.pieces_slid += outcome.relocated;
            out.strip_length = outcome.strip_length;
            out.pieces = outcome.pieces;
        } else {
            log::warn!(
                "[STRATEGY] ⑨ 꼬리 회수가 {}개를 미배치로 남겼다 — 결과를 버리고 ⑦까지의 배치를 쓴다",
                outcome.unplaced.len()
            );
        }
    }

    out
}

/// 순서대로 하나씩 놓는 그리디 1패스. ③과 ④는 여기에 자동으로 반영된다.
fn place_greedy(
    ctx: &mut StrategyContext<'_, '_>,
    sequence: &[usize],
    rule: SelectionRule,
    clusters: &[Cluster],
) -> StrategyPlacement {
    let mut placed: Vec<PlacedPiece> = Vec::with_capacity(sequence.len());
    let mut unplaced = Vec::new();
    // **결말이 정해진** sequence 자리 — 놓았거나, 미배치로 확정했거나, 짝으로 먼저
    // 놓였거나. 본 루프가 건너뛰고 [`attach_partner`] 도 여기서 짝을 고르지 않는다.
    //
    // 예전에는 "짝으로 소비된" 자리만 표시했고 미배치 자리는 `false` 로 남겼다.
    // 그러면 이미 `unplaced` 에 넣은 자리를 뒤이은 `attach_partner` 가 짝으로 집어
    // **같은 자리가 미배치이자 배치**가 된다 — 수요보다 조각이 하나 더 놓이고
    // `placed.len() + unplaced.len() > sequence.len()` 이 된다. wsjk156 은 156개가
    // 전부 놓여 미배치가 없으므로 실측에는 안 잡혔지만, 조각이 남는 인스턴스에서는
    // 검증(`solution_validation_error`)이 깨진다.
    let mut consumed = vec![false; sequence.len()];
    let mut clusters_applied = 0usize;

    for idx in 0..sequence.len() {
        if consumed[idx] {
            continue;
        }
        if !ctx.has_budget() {
            // 예산이 끝나면 남은 조각은 전부 미배치다. 여기서 억지로 채우면
            // 그 배치가 최선인 것처럼 보이지만 실제로는 아무 근거가 없다.
            unplaced.extend((idx..sequence.len()).filter(|&i| !consumed[i]));
            break;
        }
        let Some(&item_id) = sequence.get(idx) else {
            continue;
        };
        let Some(anchor) = best_option(ctx, item_id, &placed, rule) else {
            unplaced.push(idx);
            // 미배치도 결말이다 — 표시하지 않으면 뒤의 `attach_partner` 가 이 자리를
            // 짝으로 다시 집어 같은 자리가 두 번 결말을 갖는다.
            consumed[idx] = true;
            continue;
        };
        placed.push(anchor.to_piece());
        consumed[idx] = true;

        // ② 짝을 **지금 바로** 붙인다.
        //
        // 예전에는 배치가 다 끝난 뒤 "놓지 못한 조각" 에만 붙였다. 그런데 이 마커는
        // 156개가 전부 놓이므로 그 진입 조건이 영영 참이 되지 않았다 — 쌍을 43개
        // 만들어 놓고 적용은 0개였다(감사 실측).
        //
        // 앵커를 놓은 **직후**가 유일하게 옳은 시점이다. 그때만 앵커의 실제 자리를
        // 알고, 그 자리 기준 상대 위치가 아직 비어 있다.
        if !clusters.is_empty()
            && let Some((slot, partner)) =
                attach_partner(ctx, clusters, &anchor, &placed, |partner_item| {
                    sequence
                        .iter()
                        .enumerate()
                        // `get` 으로만 읽는다 — 두 벡터의 길이가 어긋나는 사고가
                        // 나더라도 배치가 죽지 않고 그 자리만 후보에서 빠진다.
                        .position(|(i, &id)| {
                            id == partner_item && matches!(consumed.get(i), Some(false))
                        })
                })
        {
            placed.push(partner.to_piece());
            consumed[slot] = true;
            clusters_applied += 1;
        }
    }

    let strip_length = ctx.frontier(&placed);
    StrategyPlacement {
        pieces: placed,
        unplaced,
        strip_length,
        clusters_applied,
        ..Default::default()
    }
}

/// 방금 놓은 앵커의 짝을 찾아 그 옆에 붙인다.
///
/// 반환값은 `(소비한 자리, 붙인 위치)`. 붙일 수 없으면 `None` 이고, 그 조각은 나중에
/// 평범한 조각으로 제 차례에 배치된다 — **아무것도 잃지 않는다**.
///
/// # 왜 "남은 자리 찾기" 를 클로저로 받는가
///
/// 세 경로가 남은 조각을 서로 다른 자료구조로 든다. 그리디는 `sequence` + 전역
/// `consumed` 비트, ⑤ 빔은 **노드마다 따로 든** 소비 비트, ⑥ best-fit 은 `remaining`
/// 벡터다. 이 함수가 그중 하나를 직접 알면 나머지 둘은 함수를 복제해야 하고, 그러면
/// 스냅 반경([`CLUSTER_SNAP_MM`])과 회전 일치 조건이 세 벌로 갈려 한쪽만 고쳐진다.
/// 그래서 "이 `item_id` 를 쓸 수 있는 자리 하나" 만 밖에서 받는다.
///
/// `free_slot` 이 돌려준 인덱스가 곧 반환값의 첫 항이다 — 그 인덱스가 무엇을 가리키는지
/// (sequence 자리인지 `remaining` 인덱스인지)는 **호출부의 약속**이다.
///
/// 붙이지 못하는 경우는 셋이다.
/// 1. 이 앵커를 쓰는 클러스터가 없다.
/// 2. 짝이 될 조각이 남아 있지 않다(이미 다 놓였다).
/// 3. 유도한 자리가 실제로는 못 놓는 자리다. [`cluster::expand_cluster`]가 주는 위치는
///    **앵커 하나만** 보고 계산한 상대 좌표라, 다른 조각이 이미 그 근처를 차지했으면
///    성립하지 않는다. 그래서 오라클이 주는 feasible 후보로 스냅해 확인한다.
pub(super) fn attach_partner(
    ctx: &mut StrategyContext<'_, '_>,
    clusters: &[Cluster],
    anchor: &AnchorOption,
    placed: &[PlacedPiece],
    mut free_slot: impl FnMut(usize) -> Option<usize>,
) -> Option<(usize, AnchorOption)> {
    // 같은 앵커에 걸린 클러스터는 같은 짝 조각·같은 자세를 되풀이해 묻는 일이 잦다
    // (앵커 하나에 짝 후보가 여럿). `options_for` 는 회전 전체의 후보 열거라 한 번이
    // 수 ms 다. 이 호출 안에서는 `placed` 가 변하지 않으므로 **(짝 조각, 자세)** 별로
    // 열거 결과를 한 번만 만들고 재사용한다.
    let mut memo: Vec<((usize, i64), Vec<AnchorOption>)> = Vec::new();

    for cluster in clusters
        .iter()
        .filter(|c| c.anchor_item == anchor.item_id && c.anchor_rotation == anchor.rotation)
    {
        // 짝으로 쓸 조각이 아직 남아 있어야 한다.
        let Some(slot) = free_slot(cluster.partner_item) else {
            continue;
        };

        let Some(target) = cluster::expand_cluster(cluster, anchor) else {
            continue;
        };

        let key = (cluster.partner_item, target.rotation);
        if !memo.iter().any(|(k, _)| *k == key) {
            let options: Vec<AnchorOption> = ctx
                .options_for(cluster.partner_item, placed)
                .into_iter()
                .filter(|o| o.rotation == target.rotation)
                .collect();
            memo.push((key, options));
        }
        let Some((_, options)) = memo.iter().find(|(k, _)| *k == key) else {
            continue;
        };

        // 유도한 자리를 **실제 feasible 후보**로 스냅한다. 반경 안에 없으면 포기한다 —
        // 억지로 놓으면 그건 쌍이 아니라 그냥 다른 자리다.
        let snapped = options
            .iter()
            .map(|o| ((o.x - target.x).hypot(o.y - target.y), o))
            .filter(|(d, _)| *d <= CLUSTER_SNAP_MM)
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, o)| *o);

        if let Some(option) = snapped {
            return Some((slot, option));
        }
    }
    None
}

/// ⑥ best-fit — 순서를 쓰지 않고 매 단계 가장 잘 맞는 조각을 고른다.
///
/// # ②를 여기에 어떻게 얹는가
///
/// best-fit 은 순서를 쓰지 않으므로 "이 조각 다음엔 저 조각" 이 성립하지 않는다.
/// 대신 **조각을 고른 직후** 그 조각이 어떤 클러스터의 앵커면 짝을 바로 붙이고
/// `remaining`/`origin` 에서 함께 뺀다. 짝은 다음 `pick_next_piece` 의 후보에서
/// 사라지므로 두 번 놓이지 않는다.
fn place_best_fit(
    ctx: &mut StrategyContext<'_, '_>,
    sequence: &[usize],
    rule: SelectionRule,
    lookahead: usize,
    clusters: &[Cluster],
) -> StrategyPlacement {
    let mut remaining: Vec<usize> = sequence.to_vec();
    // 미배치는 **원래 sequence 인덱스**로 보고해야 호출부의 prefix 재사용 판정과
    // 맞는다. 조각을 꺼낼 때마다 인덱스가 밀리므로 따로 들고 다닌다.
    let mut origin: Vec<usize> = (0..sequence.len()).collect();
    let mut placed: Vec<PlacedPiece> = Vec::with_capacity(sequence.len());
    let mut clusters_applied = 0usize;

    while !remaining.is_empty() {
        if !ctx.has_budget() {
            break;
        }
        let Some(pick) = select::pick_next_piece(ctx, &remaining, &placed, rule, lookahead) else {
            // 남은 조각 중 어느 것도 놓을 수 없다. 남은 전부가 미배치다.
            break;
        };
        let Some(&_gone) = remaining.get(pick.remaining_index) else {
            break;
        };
        placed.push(pick.option.to_piece());

        // ② 짝을 지금 붙인다. `placed` 에 앵커를 이미 넣었으므로 스냅 후보 열거가
        // 앵커를 장애물로 본다 — 그래야 겹치는 자리가 후보에 안 들어온다.
        let attached = if clusters.is_empty() {
            None
        } else {
            attach_partner(ctx, clusters, &pick.option, &placed, |partner_item| {
                remaining
                    .iter()
                    .enumerate()
                    // 앵커 자신의 자리는 제외한다. 같은 종류끼리 묶인 클러스터
                    // (`partner_item == anchor.item_id`)에서 이걸 빼먹으면 앵커 자리를
                    // 짝으로도 소비해 **조각 하나가 사라진다**.
                    .position(|(i, &id)| id == partner_item && i != pick.remaining_index)
            })
        };

        match attached {
            Some((slot, partner)) => {
                placed.push(partner.to_piece());
                clusters_applied += 1;
                // 두 자리를 뺀다. **큰 인덱스부터** 빼야 남은 하나가 안 밀린다.
                let (hi, lo) = if slot > pick.remaining_index {
                    (slot, pick.remaining_index)
                } else {
                    (pick.remaining_index, slot)
                };
                remove_slot(&mut remaining, &mut origin, hi);
                remove_slot(&mut remaining, &mut origin, lo);
            }
            None => remove_slot(&mut remaining, &mut origin, pick.remaining_index),
        }
    }

    let strip_length = ctx.frontier(&placed);
    StrategyPlacement {
        pieces: placed,
        unplaced: origin,
        strip_length,
        clusters_applied,
        ..Default::default()
    }
}

/// `remaining` 과 `origin` 에서 같은 자리를 뺀다. 범위를 벗어나면 아무것도 하지 않는다.
///
/// 두 벡터는 같은 인덱스로 짝지어 있으므로 **반드시 함께** 빼야 한다. 한쪽만 빼면
/// 그 뒤의 미배치 보고가 통째로 어긋난다 — 따로 쓰면 언젠가 한쪽을 빠뜨린다.
fn remove_slot(remaining: &mut Vec<usize>, origin: &mut Vec<usize>, idx: usize) {
    if idx < remaining.len() && idx < origin.len() {
        remaining.remove(idx);
        origin.remove(idx);
    }
}

/// 한 조각의 최선 자리. ③(규칙)과 ④(탐침 순서)가 여기에 반영된다.
fn best_option(
    ctx: &mut StrategyContext<'_, '_>,
    item_id: usize,
    placed: &[PlacedPiece],
    rule: SelectionRule,
) -> Option<AnchorOption> {
    let options = ctx.options_for(item_id, placed);
    if options.is_empty() {
        return None;
    }
    // 지표는 후보마다 같은 배치 상태를 보므로 bbox 목록을 한 번만 만든다.
    let boxes = ctx.placer.world_bounds(placed);
    let frontier = boxes.iter().map(|b| b.max_x).fold(0.0_f64, f64::max);
    let metrics: Vec<_> = options
        .iter()
        .map(|o| ctx.metrics(o, &boxes, frontier))
        .collect();
    let best = select::pick_best(&options, &metrics, rule)?;
    options.get(best).copied()
}

/// ① 회전 집합과 아이템별 회전표를 정한다.
///
/// NFP 라이브러리를 **만들기 전에** 불려야 한다 — 라이브러리에 없는 회전은 배치에
/// 쓸 수 없기 때문이다.
///
/// 전략이 꺼져 있으면 `fallback_rotations` 를 그대로 쓴다(기존 동작).
pub fn resolve_rotations(
    instance: &SPInstance,
    cfg: &crate::nfp_strategy::NfpStrategyConfig,
    fallback_rotations: &[i64],
) -> Vec<i64> {
    if !cfg.instance_rotations {
        return fallback_rotations.to_vec();
    }
    let set = crate::nfp_strategy::rotations::instance_rotation_set(
        instance,
        cfg.normalized().continuous_rotation_step_deg,
    );
    if set.is_empty() {
        log::warn!("[STRATEGY] ① 인스턴스 회전 집합이 비었다 — 설정값으로 되돌린다");
        return fallback_rotations.to_vec();
    }
    set
}

/// 전략 경로가 쓸 예산 배분.
///
/// 켠 전략에 따라 배치 1회 비용이 몇 배가 되므로, ordering 수를 그만큼 줄여야 전체
/// 예산이 지켜진다. 빔 폭 4 + best-fit 12 면 48배다 — 50 ordering 을 그대로 돌리면
/// 예산이 폭발한다.
///
/// 최소 1개는 보장한다. ordering 을 하나도 못 돌면 시드 자체가 없다.
pub fn scale_orderings(requested: usize, cfg: &crate::nfp_strategy::NfpStrategyConfig) -> usize {
    let factor = cfg.estimated_cost_factor();
    if !(factor.is_finite() && factor > 1.0) {
        return requested.max(1);
    }
    let scaled = (requested as f32 / factor).floor() as usize;
    scaled.max(1)
}

/// 두 마감 중 이른 쪽.
#[allow(dead_code)]
fn earliest(a: Option<Instant>, b: Option<Instant>) -> Option<Instant> {
    match (a, b) {
        (Some(x), Some(y)) => Some(if x < y { x } else { y }),
        (some, None) | (None, some) => some,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nfp_strategy::{DEFAULT_NFP_STRATEGY_CONFIG, NfpStrategyConfig};

    #[test]
    fn ordering_count_shrinks_by_the_cost_of_the_executed_strategy() {
        // 전부 꺼짐 — 그대로.
        assert_eq!(scale_orderings(50, &DEFAULT_NFP_STRATEGY_CONFIG), 50);

        // 빔 폭 4 -> 4배 비싸다 -> 50 / 4 = 12
        let beam = NfpStrategyConfig {
            beam_search: true,
            beam_width: 4,
            ..DEFAULT_NFP_STRATEGY_CONFIG
        };
        assert_eq!(scale_orderings(50, &beam), 12);

        // Best-fit wins over Beam: 50 / 12 = 4, not 50 / (4 * 12).
        let both = NfpStrategyConfig {
            beam_search: true,
            beam_width: 4,
            best_fit_selection: true,
            best_fit_lookahead: 12,
            ..DEFAULT_NFP_STRATEGY_CONFIG
        };
        assert_eq!(scale_orderings(50, &both), 4);
        let accretion = NfpStrategyConfig { accretion: true, ..both };
        assert_eq!(scale_orderings(50, &accretion), 50);

        // 아무리 비싸도 최소 1개는 돈다 — 0이면 시드가 아예 없다.
        assert_eq!(scale_orderings(1, &both), 1);
    }

    #[test]
    fn a_cost_free_strategy_does_not_shrink_the_ordering_budget() {
        // ①②③ 은 전처리이거나 비교 규칙만 바꾼다 — 배치 1회 비용이 그대로다.
        let cheap = NfpStrategyConfig {
            instance_rotations: true,
            pairwise_clustering: true,
            contact_maximization: true,
            ..DEFAULT_NFP_STRATEGY_CONFIG
        };
        assert_eq!(scale_orderings(50, &cheap), 50);
    }

    #[test]
    fn a_strategy_placement_converts_without_losing_pieces_or_unplaced_indices() {
        let placement = StrategyPlacement {
            pieces: vec![
                PlacedPiece {
                    item_id: 0,
                    rotation: 0,
                    translation: (1.0, 2.0),
                },
                PlacedPiece {
                    item_id: 1,
                    rotation: 180,
                    translation: (3.0, 4.0),
                },
            ],
            unplaced: vec![7, 9],
            strip_length: 42.5,
            clusters_applied: 1,
            pieces_slid: 2,
        };
        let seq = placement.into_sequential();
        assert_eq!(seq.pieces.len(), 2);
        assert_eq!(seq.placed, 2, "placed 는 조각 수와 같아야 한다");
        assert_eq!(seq.unplaced, vec![7, 9]);
        assert!((seq.strip_length - 42.5).abs() < 1.0e-6);
    }

    #[test]
    fn rotations_fall_back_to_the_configured_set_while_the_strategy_is_off() {
        // 인스턴스를 만들지 않고도 확인할 수 있는 경로다.
        let off = DEFAULT_NFP_STRATEGY_CONFIG;
        assert!(!off.instance_rotations);
        // `resolve_rotations` 는 꺼져 있으면 인스턴스를 보지 않는다는 것이 계약이다.
        // 그 계약을 지키는지는 호출부 테스트(`search.rs`)가 실제 인스턴스로 확인한다.
        assert_eq!(off.normalized().continuous_rotation_step_deg, 90);
    }

    #[test]
    fn removing_two_slots_from_the_back_keeps_the_two_vectors_aligned() {
        // ⑥에서 앵커와 짝을 함께 뺄 때 **작은 인덱스를 먼저** 빼면 뒤가 밀려 엉뚱한
        // 자리가 사라진다. 이 테스트는 `place_best_fit` 이 큰 쪽부터 빼는 계약을 고정한다.
        let mut remaining = vec![10, 11, 12, 13, 14];
        let mut origin = vec![0, 1, 2, 3, 4];
        // 앵커 1번, 짝 3번을 뺀다.
        remove_slot(&mut remaining, &mut origin, 3);
        remove_slot(&mut remaining, &mut origin, 1);
        assert_eq!(remaining, vec![10, 12, 14]);
        assert_eq!(origin, vec![0, 2, 4]);

        // 범위 밖은 아무것도 하지 않는다 — panic 하지 않는 것이 계약이다.
        remove_slot(&mut remaining, &mut origin, 99);
        assert_eq!(remaining.len(), 3);
        assert_eq!(origin.len(), 3);
    }

    #[test]
    fn the_earlier_deadline_wins_and_a_missing_one_is_ignored() {
        let now = Instant::now();
        let soon = now + std::time::Duration::from_secs(1);
        let later = now + std::time::Duration::from_secs(10);
        assert_eq!(earliest(Some(soon), Some(later)), Some(soon));
        assert_eq!(earliest(Some(later), Some(soon)), Some(soon));
        assert_eq!(earliest(Some(soon), None), Some(soon));
        assert_eq!(earliest(None, Some(soon)), Some(soon));
        assert_eq!(earliest(None, None), None);
    }
}
