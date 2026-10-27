//! ② 쌍별 클러스터링 — 잘 맞물리는 두 조각을 하나의 super-piece 로 묶는다.
//!
//! # 왜 필요한가
//!
//! 의류 마커에서 좌/우 대칭 패널과 소매는 **짝지어 놓을 때** 낭비가 급감한다. 볼록하지
//! 않은 두 조각이 서로의 오목한 부분에 들어가면 묶음 bbox 가 두 bbox 의 합보다 훨씬
//! 작아진다. 그런데 순차 배치는 조각을 하나씩 보므로 그 짝을 스스로 찾지 못한다 —
//! 앵커를 놓는 순간 짝이 들어갈 자리는 이미 다른 조각에게 넘어간 뒤다.
//!
//! # 알고리즘
//!
//! ```text
//! 1) 종류 쌍 (a, b) × 회전 조합마다, b 를 a 의 NFP 경계 위에 놓는 후보를 구한다
//!    → NFP 경계 위이므로 정의상 겹치지 않고 맞닿는다
//! 2) 각 후보가 아끼는 면적 = (bbox_a + bbox_b) − 묶음_hull_면적
//! 3) 쌍마다 최선 후보만 남기고, 상대 이득 = 아낀 면적 / min(bbox_a, bbox_b) 가
//!    cluster_min_gain 을 넘는 쌍만 후보로 둔다
//! 4) **아낀 면적** 내림차순 탐욕 매칭 — 각 인스턴스는 최대 한 번, 수요(qty)는 존중
//! ```
//!
//! 후보 위치는 [`super::StrategyContext`] 의 오라클(`candidates_for`)에서 그대로 받는다.
//! `a` 하나만 장애물로 둔 상태의 feasible 앵커 집합이 곧 "a 에 대해 b 가 놓일 수 있는
//! 자리" 이기 때문이다. 여기서 새로 쓰는 것은 **무엇이 좋은 짝인가** 라는 판단뿐이다.
//!
//! # 비용
//!
//! 쌍 수가 `O(종류² × 회전²)` 이라 상한 없이는 전처리가 배치보다 오래 걸린다.
//! [`MAX_CLUSTER_ITEM_KINDS`], [`MAX_CLUSTER_ROTATIONS`], [`MAX_PAIR_PROBES`] 세 상한과
//! [`CLUSTER_BUILD_BUDGET`] 으로 묶는다.
//!
//! # 이 전략을 켜지 말아야 하는 경우 (여성WSjk 156-패널 실측)
//!
//! **이 마커에서는 ②를 켜지 마라.** 쌍 탐색 자체는 고쳤고 잘 맞물리는 짝을 실제로
//! 찾아낸다(확정 43쌍). 그런데 그 쌍을 배치에 반영하면 **밀도가 오히려 떨어진다**.
//!
//! ```text
//! 붙이는 방식              폭        밀도      기준 대비
//! 쌍 미적용 (기준)      7089.89    71.75%      —
//! 사후 부착             7137.85    71.27%    -0.48%p   (43쌍 중 13쌍만 붙음)
//! 쌍 우선 (super-piece) 7119.06    71.46%    -0.29%p   (43쌍 중 26쌍 붙음)
//! ```
//!
//! 이유는 이렇다. 순차 배치의 후보 좌표는 **이미 NFP 경계 위**라 놓는 조각마다 기존
//! 조각에 맞닿는다 — 즉 배치기는 클러스터링이 노리는 맞물림을 이미 하고 있다.
//! 쌍을 강제하면 거기에 "이 두 조각은 이 상대 자세여야 한다" 는 제약만 더해지고,
//! 그 제약이 아끼는 면적보다 잃는 자유도가 크다. 임계를 0.80 까지 올려 강한 짝만
//! 남겨도 결과는 나빠지기만 한다(위 표는 계측 테스트
//! `attaching_pairs_during_placement_is_what_actually_moves_the_density` 참조).
//!
//! **"쌍이 아끼는 면적" 을 남은 여유로 읽지 마라.** 확정 43쌍의 면적 합은 strip 면적의
//! 10.97% 지만, 같은 쌍의 폭 합은 strip 길이의 99% 다. 둘 다 배치기가 이미 회수한
//! 겹침을 중복해서 세고 있어서, 어느 쪽도 달성 가능한 개선의 상한이 아니다
//! ([`ClusterDiagnostics::matched_saved_area`] 참조).
//!
//! 이 전략이 값어치를 하려면 배치기가 맞물림을 **놓치는** 인스턴스여야 한다 —
//! 후보 탐침 예산이 아주 낮거나, 조각이 크고 수요가 많아 같은 짝이 반복되는 마커다.
//! 그런 근거 없이 켜지 마라.

use jagua_rs::Instant;
use jagua_rs::probs::spp::entities::SPInstance;

use super::{AnchorOption, StrategyContext};
use crate::nfp_lbf::PlacedPiece;
use crate::nfp_lbf::placement::Aabb;

/// 짝을 찾아볼 아이템 종류 수 상한 (면적 내림차순 상위 N).
///
/// 쌍은 자기 자신과의 짝을 포함해 `N(N+1)/2` 개다. 여기에 회전 조합을 곱한 만큼
/// `candidates_for` 를 부른다. 장애물이 하나뿐인 가벼운 호출이지만 상한이 없으면
/// 전처리가 배치보다 오래 걸린다.
///
/// 큰 조각을 먼저 보는 이유는 **큰 조각이 남기는 공백이 크기 때문**이다. 작은 조각끼리
/// 짝지어 봐야 아끼는 면적이 작다.
///
/// # 왜 8이 아니라 160인가 (실측)
///
/// 의류 마커(여성WSjk 156-패널)는 **156 종류가 전부 수요 1개**다. 상한 8은 전체의
/// 5%만 보고, 수요가 1이라 같은 종류끼리 묶을 수도 없어 후보가 28쌍에 그친다.
/// 그리고 그 28쌍 안에는 채택할 만한 짝이 실제로 없다.
///
/// 종류 상한을 훑은 실측(탐침 4096, 회전 1종, 계측 테스트
/// `pair_search_on_the_garment_marker_is_measured_not_guessed`):
///
/// ```text
/// 상한    평가 쌍   채택   확정 클러스터   전처리   최대 이득   (옛 정의로 채택)
///    8         28      0        0            1ms      +0.011           0
///   16        120      0        0            4ms      +0.022           0
///   24        276     12        2            7ms      +0.531           0
///   32        496     46        7           11ms      +0.557           0
///   48       1128    162        8           23ms      +0.564           0
///   64       2016    416        9           42ms      +1.000           1
///   96       4560   1280       21           95ms      +1.000          12
///  156      12090   3193       43          184ms      +1.000         189
/// ```
///
/// 상한 8·16 에서는 **정의를 고쳐도 채택이 0** 이다. 이 마커에서 잘 맞물리는 짝은
/// 큰 패널끼리가 아니라 소매·요크 같은 중간 크기 조각 사이에 있어서, 면적 상위 16
/// 안에는 아예 들어오지 않는다.
///
/// 맨 오른쪽 열이 중요하다 — **정의를 그대로 두고 상한만 올려도 쌍은 나온다**(189쌍).
/// 즉 "쌍이 0개" 의 주범은 이 상한이었고, 이득 정의는 그 위에서 채택 폭을 17배 넓히는
/// 두 번째 인자다. 둘을 뒤집어 읽지 마라.
///
/// 비용은 쌍 수(`O(N²)`)에 비례한다. 156 종류를 전부 봐도 0.2초(실측 184~222ms, 4회
/// 반복)로, ordering 12회 배치(4.8s)의 4%다. 전처리는 ordering 과 무관하게 **한 번만** 도므로 이 정도는
/// 그대로 쓸 값어치가 있다. 160은 이 마커를 통째로 덮는 값이고, 그보다 큰
/// 인스턴스는 [`CLUSTER_BUILD_BUDGET`] 이 시간으로 잘라 준다.
pub const MAX_CLUSTER_ITEM_KINDS: usize = 160;

/// 쌍 전처리에 쓸 시간 상한.
///
/// `build_clusters` 는 호출부에서 **전체 예산의 deadline** 을 그대로 받는다. 그대로
/// 쓰면 종류 상한을 올리는 순간 전처리가 배치 예산을 통째로 먹을 수 있다. 전처리는
/// 실패해도 "묶지 않는다" 로 안전하게 퇴화하므로, 자기 몫을 따로 잘라 쓰는 편이 맞다.
///
/// 2초는 위 실측표의 최댓값(156 종류 = 184~222ms)에 9배 여유를 둔 값이다. 이 마커에서는
/// 절대 걸리지 않고, 종류가 수백 개인 인스턴스에서만 발동한다. 잘려도 그때까지 찾은
/// 쌍은 남는다 — 면적 내림차순으로 돌기 때문에 잘리는 쪽이 늘 작은 조각이다.
pub const CLUSTER_BUILD_BUDGET: std::time::Duration = std::time::Duration::from_millis(2000);

/// 한 아이템에서 시험할 회전 수 상한.
///
/// 조합이 이 값의 제곱으로 늘어난다. 4를 넘기면 `MAX_CLUSTER_ITEM_KINDS` 를 같이
/// 줄여야 총 호출 수가 유지된다.
pub const MAX_CLUSTER_ROTATIONS: usize = 4;

/// 한 (쌍 × 회전 조합) 에서 평가할 후보 위치 수 상한.
///
/// # 이 상한은 오늘 한 번도 발동하지 않는다
///
/// 오라클이 이미 [`MAX_EVALUATED_CANDIDATES`](crate::nfp_lbf::MAX_EVALUATED_CANDIDATES)
/// `= 256` 개에서 잘라 돌려주므로, 여기 512 는 그보다 **위**에 있다. 즉 이 값은
/// "쌍 탐색이 후보를 얼마나 보는가" 를 정하지 않는다 — 실제 상한은 오라클 쪽이다.
/// (여성WSjk 156-패널 실측: 쌍당 평균 51개, 총 617,752 / 12,090쌍.)
///
/// 그래도 남겨 두는 이유는 오라클의 상한이 나중에 올라갈 수 있기 때문이다. 그때
/// 전처리가 조용히 그만큼 느려지지 않도록 여기서 한 번 더 막는다. 두 값의 관계는
/// `the_pair_probe_cap_sits_above_what_the_oracle_can_return` 이 못 박는다.
///
/// **이 상한을 근거로 "후보를 다 못 봤다/다 봤다" 를 말하지 마라.** 그 답은 오라클의
/// 상한과 탐침 예산이 정한다.
pub const MAX_PAIR_PROBES: usize = 512;

/// 묶인 조각 쌍 하나. 두 번째 조각의 위치는 첫 번째 앵커 기준 상대 좌표다.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cluster {
    /// 앵커 조각의 `item_id`.
    pub anchor_item: usize,
    /// 앵커에 적용할 회전(도).
    pub anchor_rotation: i64,
    /// 파트너 조각의 `item_id`.
    pub partner_item: usize,
    /// 파트너에 적용할 회전(도).
    pub partner_rotation: i64,
    /// 앵커 기준 상대 평행이동(mm).
    pub offset: (f64, f64),
    /// 묶음 충전율(0~1) = `(실면적 합) / 묶음 hull 면적`.
    ///
    /// **채택 기준이 아니라 보고용 값**이다. 채택은 [`pair_gain`] 이 정한다 — 충전율은
    /// 두 조각의 크기가 다르면 짝의 좋고 나쁨과 무관하게 움직인다.
    pub fill_ratio: f64,
    /// 파트너의 **앵커 기준** bbox(회전 적용 후).
    ///
    /// [`expand_cluster`] 가 placer 없이 파트너의 월드 bbox 를 만들려면 이 값이
    /// 필요하다. 계약에 적힌 여섯 필드에 더해 두는 이유는, 이게 없으면
    /// `expand_cluster` 가 `&mut StrategyContext` 를 받아야 해서 배치 루프 한가운데서
    /// 오라클을 다시 빌려야 하기 때문이다.
    pub partner_bounds: Aabb,
}

/// 짝을 찾아볼 아이템 종류 하나.
#[derive(Clone, Copy, Debug)]
struct Kind {
    item_id: usize,
    /// 남은 수요. 같은 종류끼리 묶으려면 2 이상이어야 한다.
    qty: usize,
    /// 실제 면적(mm²). 상위 N 선정 기준.
    area: f64,
}

/// 쌍 하나의 평가 결과 — 채택 여부를 정하는 이득을 들고 다닌다.
#[derive(Clone, Copy, Debug)]
struct PairCandidate {
    cluster: Cluster,
    /// **상대 이득** = 아낀 면적 / min(개별 bbox 면적). `cluster_min_gain` 과 비교하는 값.
    gain: f64,
    /// **절대 이득**(mm²) = `bbox_a + bbox_b − 묶음 hull`. 탐욕 매칭 정렬 기준.
    saved_area: f64,
    /// 이 쌍이 strip 길이 방향으로 아낀 폭(mm). 계측용 — 근거는 [`saved_width`].
    saved_width: f64,
}

/// 한 쌍을 훑고 나온 것 전부.
///
/// 최선 후보만 돌려주면 안 되는 이유는 [`ClusterDiagnostics::best_fill_gain`] 때문이다.
/// 옛 정의의 최댓값은 **옛 정의가 고르는 자리**에서 재야 하는데, 최선 후보는 새 정의가
/// 고른 자리라 거기서 읽은 옛 이득은 옛 정의의 최댓값이 아니라 그 아래 어딘가다.
/// 두 값은 서로 다른 argmax 에서 나오므로 따로 들고 나온다.
struct PairScan {
    /// 새 정의로 고른 최선 후보.
    best: Option<PairCandidate>,
    /// 이 쌍에서 평가한 후보 위치 수.
    probes: usize,
    /// 이 쌍의 **모든** 후보에서 관측한 옛 정의 이득의 최댓값.
    best_fill_gain: f64,
}

/// 탐욕 매칭이 확정한 것과 그 이론상 값어치.
struct MatchOutcome {
    clusters: Vec<Cluster>,
    /// 확정한 쌍이 아끼는 bbox 면적의 합(mm²).
    saved_area: f64,
    /// 확정한 쌍이 아끼는 **폭**의 합(mm). 근거는 [`saved_width`].
    saved_width: f64,
}

/// 쌍 탐색이 실제로 무엇을 보았고 무엇에서 걸렀는지 세는 계측판.
///
/// "쌍이 0개" 라는 증상은 원인이 최소 넷이다 — 종류 상한, 이득 **정의**, 임계값, 예산.
/// 넷을 구분하지 못하면 추측으로 고치게 된다. 그래서 세어서 남긴다.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ClusterDiagnostics {
    /// 인스턴스의 전체 아이템 종류 수(수요 > 0).
    pub kinds_total: usize,
    /// 상한을 적용한 뒤 실제로 짝을 찾아본 종류 수.
    pub kinds_considered: usize,
    /// 평가한 (a, b) 쌍의 수.
    pub pairs_evaluated: usize,
    /// 후보 위치가 하나라도 나온 쌍의 수.
    pub pairs_with_candidate: usize,
    /// 후보는 나왔지만 이득이 `cluster_min_gain` 에 못 미쳐 버린 쌍의 수.
    pub pairs_below_min_gain: usize,
    /// 임계를 통과해 매칭 후보가 된 쌍의 수.
    pub pairs_accepted: usize,
    /// **옛 정의**였다면 같은 임계를 통과했을 쌍의 수.
    ///
    /// "쌍이 0개" 의 원인이 정의인지 종류 상한인지를 가르는 값이다. 상한을 올린
    /// 상태에서 이 값이 0 이면 정의가 원인이고, 0 이 아니면 상한이 원인이었다는
    /// 뜻이다 — 실측은 후자였다([`MAX_CLUSTER_ITEM_KINDS`] 의 표).
    pub pairs_accepted_by_old_gain: usize,
    /// 탐욕 매칭이 실제로 확정한 클러스터 수.
    pub clusters_matched: usize,
    /// 확정한 클러스터들이 **이론상** 아끼는 면적의 합(mm²).
    ///
    /// # 이 값을 "아직 남은 여유" 로 읽지 마라
    ///
    /// bbox 산술의 합계일 뿐 달성 가능한 개선의 상한이 **아니다**. 좋은 배치에서는
    /// 조각들의 bbox 가 서로 깊이 겹치고 배치기가 그 겹침을 이미 쓰고 있어서, 여기서
    /// 세는 "아낀 면적" 은 대부분 배치기가 이미 회수한 몫이다.
    ///
    /// 여성WSjk 156-패널 실측이 그것을 보여 준다. 확정 43쌍의 면적 합은
    /// 1,185,108mm² = strip 면적의 **10.97%** 인데, 같은 쌍의 폭 합
    /// ([`Self::matched_saved_width`])은 7,017mm 로 strip 길이 7,090mm 의 **99%** 다.
    /// strip 을 99% 줄일 수 있을 리 없다 — 두 합계 모두 상한이 아니라는 증거다.
    /// 실제 A/B 에서 ②는 어느 임계에서도 기준선을 이기지 못했다(모듈 문서의 표).
    pub matched_saved_area: f64,
    /// 확정한 클러스터들이 이론상 아끼는 **폭**의 합(mm). 근거는 [`saved_width`].
    ///
    /// 면적과 함께 재는 이유는 목적함수가 strip **길이**이기 때문이다. 위아래로 쌓아
    /// 폭만 아끼는 쌍은 면적 이득이 0이어도 값어치가 있고, 좌우로 붙여 면적만 아끼는
    /// 쌍은 면적 이득이 커도 strip 을 1mm 도 줄이지 못한다.
    ///
    /// 실측에서 이 값은 strip 길이의 99% 로 나온다 — [`Self::matched_saved_area`] 참조.
    /// 그 자리에서 "폭이 목적함수에 더 맞으니 이걸로 정렬하자" 로 넘어가지 마라.
    /// 두 합계가 똑같이 겹침을 중복해서 세고 있다는 뜻이지, 폭이 더 나은 눈금이라는
    /// 뜻이 아니다.
    pub matched_saved_width: f64,
    /// 평가한 후보 위치의 총합(`MAX_PAIR_PROBES` 적용 후).
    pub probes_evaluated: usize,
    /// 예산이 끊겨 쌍 탐색을 조기 종료했는가.
    pub budget_hit: bool,
    /// 전 쌍에서 관측한 상대 이득의 최댓값.
    pub best_gain: f64,
    /// 전 쌍에서 관측한 **옛 정의** 이득의 최댓값.
    ///
    /// 옛 정의가 고를 자리에서 재야 뜻이 있으므로 [`PairScan`] 이 모든 후보를 훑어
    /// 따로 센다. 새 정의가 고른 최선 후보에서 읽으면 그건 옛 정의의 최댓값이 아니라
    /// 그 아래 어딘가라, "옛 정의로는 못 넘는다" 의 근거로 쓸 수 없다.
    pub best_fill_gain: f64,
    /// 쌍 탐색에 든 시간(ms).
    pub elapsed_ms: u128,
}

/// 인스턴스에서 묶을 만한 조각 쌍을 찾는다.
///
/// 반환은 **배치에 바로 쓸 수 있는 확정 쌍 목록**이다. 같은 종류가 여러 개면 같은 쌍이
/// 여러 번 들어갈 수 있다(수요만큼). 이득이 `ctx.cfg.cluster_min_gain` 에 못 미치는
/// 쌍은 들어가지 않는다 — 이득 없이 묶으면 배치 자유도만 잃는다.
///
/// `deadline`, `ctx.deadline`, [`CLUSTER_BUILD_BUDGET`] 중 **가장 이른 것**을 지킨다.
/// 예산을 넘기면 그때까지 찾은 쌍으로 매칭해 돌려준다 (빈 목록은 "묶지 않는다" 라는
/// 안전한 폴백이다).
pub fn build_clusters(
    ctx: &mut StrategyContext<'_, '_>,
    instance: &SPInstance,
    deadline: Option<Instant>,
) -> Vec<Cluster> {
    // 전처리는 자기 몫만 쓴다 — 근거는 [`CLUSTER_BUILD_BUDGET`].
    let limit = earliest(deadline, Some(Instant::now() + CLUSTER_BUILD_BUDGET));
    build_clusters_measured(ctx, instance, limit, MAX_CLUSTER_ITEM_KINDS).0
}

/// [`build_clusters`] 와 같은 일을 하되 **왜 그 결과가 나왔는지**를 함께 돌려준다.
///
/// `max_kinds` 를 인자로 뺀 이유는 상한이 결과를 좌우하는 노브인데 상수로 박혀 있으면
/// 그 영향을 잴 수 없기 때문이다 — 계측 테스트가 값을 훑는다.
pub fn build_clusters_measured(
    ctx: &mut StrategyContext<'_, '_>,
    instance: &SPInstance,
    deadline: Option<Instant>,
    max_kinds: usize,
) -> (Vec<Cluster>, ClusterDiagnostics) {
    // 이득은 음수일 수 있다. 0으로 시작하면 "전부 음수" 라는 사실이 가려지므로
    // −∞ 에서 시작한다 — 그 사실이 바로 이번에 확인해야 할 것이다.
    let mut diag = ClusterDiagnostics {
        best_gain: f64::NEG_INFINITY,
        best_fill_gain: f64::NEG_INFINITY,
        ..ClusterDiagnostics::default()
    };
    if !ctx.cfg.pairwise_clustering {
        return (Vec::new(), diag);
    }
    let started = Instant::now();
    let limit = earliest(ctx.deadline, deadline);
    let min_gain = ctx.cfg.normalized().cluster_min_gain as f64;

    // ── 후보 종류 선정: 면적 내림차순 상위 N ────────────────────────────
    //
    // 면적은 회전과 무관하므로 한 자세에서 한 번만 재고 그 값을 들고 정렬한다
    // (`sort_by` 안에서 재면 비교마다 오라클을 다시 부른다).
    let mut kinds: Vec<Kind> = Vec::new();
    for (item, qty) in instance.items.iter() {
        if *qty == 0 {
            continue;
        }
        let area = kind_area(ctx, item.idx());
        if area > 0.0 {
            kinds.push(Kind {
                item_id: item.idx(),
                qty: *qty,
                area,
            });
        }
    }
    kinds.sort_by(|a, b| {
        b.area
            .partial_cmp(&a.area)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    diag.kinds_total = kinds.len();
    kinds.truncate(max_kinds.max(1));
    diag.kinds_considered = kinds.len();
    if kinds.is_empty() {
        return (Vec::new(), diag);
    }

    // 오라클의 탐침 순서를 중립으로 — 클러스터 탐색은 프론티어 개념이 없다.
    //
    // 끝나고 되돌리지 않는다. 되돌릴 원래 값이라는 게 없기 때문이다 — 이 둘을 읽는
    // 쪽(`StrategyContext::options_for`, `sliding`)은 호출 때마다 자기 값을 다시
    // 써 넣는다. 그 규약이 깨지면 여기서 남긴 `frontier = 0` 이 그대로 새어 나가
    // ④ 포켓 우선이 조용히 죽으므로, 새 호출자를 만들 때 확인해야 할 지점이다.
    ctx.placer.set_probe_frontier(0.0);
    ctx.placer.set_pocket_first(false);

    // ── 쌍 평가 ────────────────────────────────────────────────────────
    let mut candidates: Vec<PairCandidate> = Vec::new();

    'outer: for ai in 0..kinds.len() {
        for bi in ai..kinds.len() {
            if expired(limit) {
                diag.budget_hit = true;
                break 'outer;
            }
            let (Some(a), Some(b)) = (kinds.get(ai).copied(), kinds.get(bi).copied()) else {
                continue;
            };
            let (a_id, b_id) = (a.item_id, b.item_id);
            // 같은 종류끼리 묶으려면 최소 2개가 있어야 한다.
            if a_id == b_id && a.qty < 2 {
                continue;
            }

            diag.pairs_evaluated += 1;
            let scan = best_pair(ctx, a_id, b_id, limit);
            diag.probes_evaluated += scan.probes;
            diag.best_fill_gain = diag.best_fill_gain.max(scan.best_fill_gain);
            if scan.best_fill_gain > min_gain {
                diag.pairs_accepted_by_old_gain += 1;
            }
            if let Some(best) = scan.best {
                diag.pairs_with_candidate += 1;
                diag.best_gain = diag.best_gain.max(best.gain);
                if best.gain > min_gain {
                    diag.pairs_accepted += 1;
                    candidates.push(best);
                } else {
                    diag.pairs_below_min_gain += 1;
                }
            }
        }
    }

    // ── 탐욕 매칭 ──────────────────────────────────────────────────────
    let max_id = instance.items.iter().map(|(i, _)| i.idx()).max().unwrap_or(0);
    let mut remaining = vec![0usize; max_id.saturating_add(1)];
    for (item, qty) in instance.items.iter() {
        if let Some(slot) = remaining.get_mut(item.idx()) {
            *slot = *qty;
        }
    }

    let matched = greedy_match(candidates, &mut remaining);
    let clusters = matched.clusters;
    diag.clusters_matched = clusters.len();
    diag.matched_saved_area = matched.saved_area;
    diag.matched_saved_width = matched.saved_width;
    diag.elapsed_ms = started.elapsed().as_millis();
    (clusters, diag)
}

/// 클러스터를 배치 순서로 펼친다 — 앵커를 놓은 뒤 파트너를 상대 위치에 놓는다.
///
/// `anchor` 는 **이 클러스터의 앵커를 실제로 놓은 선택지**여야 한다. 조각이나 자세가
/// 다르면 상대 오프셋이 성립하지 않으므로 `None` 을 돌려준다 — 그 경우 호출부는
/// 파트너를 평범한 조각으로 따로 배치하면 된다.
///
/// 반환된 위치는 앵커가 놓인 자리에서 유도한 것이라 **strip 안이라는 보장이 없다**.
/// 호출부는 그대로 확정하지 말고 오라클로 feasible 여부를 다시 확인해야 한다.
pub fn expand_cluster(cluster: &Cluster, anchor: &AnchorOption) -> Option<AnchorOption> {
    if anchor.item_id != cluster.anchor_item || anchor.rotation != cluster.anchor_rotation {
        return None;
    }
    let x = anchor.x + cluster.offset.0;
    let y = anchor.y + cluster.offset.1;
    if !(x.is_finite() && y.is_finite()) {
        return None;
    }
    Some(AnchorOption {
        item_id: cluster.partner_item,
        rotation: cluster.partner_rotation,
        x,
        y,
        world: cluster.partner_bounds.translated(x, y),
        // 파트너는 오라클 후보가 아니라 앵커에 **고정 상대 위치**로 붙는다. 그래서
        // 오라클이 세어 주는 정확 접촉 수가 없다. 0 은 "모른다" 가 아니라
        // "이 선택지에는 그 신호가 없다" 는 뜻이고, 그러면 비교가 근사 접촉 길이로
        // 내려간다 — 쌍을 붙이는 판단은 이미 `expand_cluster` 가 이득으로 했으므로
        // 여기서 접촉 수로 다시 겨룰 일이 없다.
        touches: 0,
    })
}

// ────────────────────────────────────────────────────────────────────────────
// 내부 — 쌍 평가
// ────────────────────────────────────────────────────────────────────────────

/// 두 종류의 모든 회전 조합에서 가장 잘 맞물리는 배치를 찾는다.
///
/// 앵커는 원점이 아니라 **자기 폭만큼 오른쪽**에, 세로로는 **바닥 줄과 천장 줄** 두
/// 곳에 둔다. 원점에 두면 IFP(= strip 안에 들어가는 앵커 영역)가 파트너를 왼쪽·아래로
/// 놓지 못하게 막아 짝의 절반을 못 보게 되고, 세로 중앙 한 곳만 보면 키 큰 앵커에
/// 위아래로 쌓이는 짝을 못 본다.
///
/// 반환은 [`PairScan`] 이다. 후보 수와 옛 정의 최댓값을 같이 내는 것은 계측용이다 —
/// 후보가 0개라서 쌍이 안 나온 것인지, 후보는 많은데 이득이 없어서 안 나온 것인지
/// 구분해야 하고, 옛 정의가 정말 못 넘는지도 옛 정의의 자기 최댓값에서 봐야 한다.
fn best_pair(
    ctx: &mut StrategyContext<'_, '_>,
    a_id: usize,
    b_id: usize,
    limit: Option<Instant>,
) -> PairScan {
    let strip_h = ctx.placer.strip_height() as f64;

    let a_rots = capped_rotations(ctx, a_id);
    let b_rots = capped_rotations(ctx, b_id);

    let mut best: Option<PairCandidate> = None;
    let mut probes_seen = 0usize;
    let mut best_fill_gain = f64::NEG_INFINITY;

    for &rot_a in &a_rots {
        let Some((bounds_a, area_a)) = ctx.placer.shape_bounds_area(a_id, rot_a) else {
            continue;
        };
        if !(area_a.is_finite() && area_a > 0.0) {
            continue;
        }

        for &rot_b in &b_rots {
            if expired(limit) {
                return PairScan {
                    best,
                    probes: probes_seen,
                    best_fill_gain,
                };
            }
            // 같은 종류끼리는 `(rot_a, rot_b)` 와 `(rot_b, rot_a)` 가 역할만 바뀐 같은
            // 상대 배치다 — 오프셋 부호만 뒤집힌다. 한쪽만 본다.
            if a_id == b_id && rot_b < rot_a {
                continue;
            }
            let Some((bounds_b, area_b)) = ctx.placer.shape_bounds_area(b_id, rot_b) else {
                continue;
            };
            if !(area_b.is_finite() && area_b > 0.0) {
                continue;
            }

            // 앵커를 놓을 자리. 좌우 양쪽에 파트너가 들어갈 여유를 준다.
            let pad = bounds_a.width().max(bounds_b.width());
            let ta_x = -bounds_a.min_x + pad;

            // 옛 정의가 쓰던 기준 — 지금은 계측으로만 남긴다. 근거는 [`pair_gain`].
            let solo = solo_fill_ratio(bounds_a, area_a).max(solo_fill_ratio(bounds_b, area_b));
            // 따로 놓았을 때 각자 차지하는 면적의 합. 이것이 올바른 비교 대상이다.
            let separate = bounds_a.area() + bounds_b.area();

            // 앵커를 **바닥 줄과 천장 줄** 두 곳에 놓고 각각 훑는다. 세로 중앙 한 곳만
            // 보면 앵커 높이가 strip 의 절반을 넘는 순간 파트너가 위로도 아래로도
            // 들어갈 자리가 없어 **위아래로 쌓이는 짝**(폭을 아끼는 바로 그 짝)이 통째로
            // 사라진다. 바닥 줄은 파트너가 위로 쌓이는 자리를, 천장 줄은 아래로 쌓이는
            // 자리를 열어 준다. `best` 는 두 줄의 최댓값을 그대로 이어받는다.
            for ta_y in [-bounds_a.min_y, strip_h - bounds_a.max_y] {
                if !(ta_x.is_finite() && ta_y.is_finite()) {
                    continue;
                }
                // 오라클이 실제로 보는 평행이동은 `f32` 다. 앵커를 f64 원본으로 들고
                // 있으면 `offset = tb - ta` 가 f32 반올림만큼 어긋난 상대 좌표가 되고,
                // 그 오차가 `expand_cluster` 를 지나 배치 좌표까지 그대로 실려 간다.
                // 오차 자체는 mm 단위로 1e-4 수준이지만, 이 쌍의 존재 이유가 "NFP 경계
                // 위라서 정확히 맞닿는다" 인 이상 맞닿음을 흐리는 오차는 두지 않는다.
                // 그래서 앵커를 f32 로 한 번 접은 뒤 그 값을 기준으로 삼는다.
                let (ta_x32, ta_y32) = (ta_x as f32, ta_y as f32);
                let (ta_x, ta_y) = (ta_x32 as f64, ta_y32 as f64);
                let world_a = bounds_a.translated(ta_x, ta_y);

                let anchor_piece = PlacedPiece {
                    item_id: a_id,
                    rotation: rot_a,
                    translation: (ta_x32, ta_y32),
                };

                let probes = ctx.placer.candidates_for(b_id, rot_b, &[anchor_piece]);
                for cand in probes.iter().take(MAX_PAIR_PROBES) {
                    probes_seen += 1;
                    let (tb_x, tb_y) = (cand.translation.0 as f64, cand.translation.1 as f64);
                    if !(tb_x.is_finite() && tb_y.is_finite()) {
                        continue;
                    }
                    let world_b = bounds_b.translated(tb_x, tb_y);
                    let hull = hull_area(world_a, world_b);
                    let Some((gain, saved_area)) = pair_gain(hull, separate, bounds_a, bounds_b)
                    else {
                        continue;
                    };
                    let fill = pair_fill_ratio(world_a, world_b, area_a, area_b);
                    // 옛 정의의 최댓값은 **모든** 후보에서 따로 센다. 아래 `best` 는 새
                    // 정의가 고른 자리라, 거기서 읽은 옛 이득은 옛 정의의 최댓값이 아니다.
                    best_fill_gain = best_fill_gain.max(fill - solo);
                    if best.map(|c| gain > c.gain).unwrap_or(true) {
                        best = Some(PairCandidate {
                            cluster: Cluster {
                                anchor_item: a_id,
                                anchor_rotation: rot_a,
                                partner_item: b_id,
                                partner_rotation: rot_b,
                                offset: (tb_x - ta_x, tb_y - ta_y),
                                fill_ratio: fill,
                                partner_bounds: bounds_b,
                            },
                            gain,
                            saved_area,
                            saved_width: saved_width(world_a, world_b, bounds_a, bounds_b),
                        });
                    }
                }
            }
        }
    }
    PairScan {
        best,
        probes: probes_seen,
        best_fill_gain,
    }
}

/// 이 종류에서 시험할 회전 — [`MAX_CLUSTER_ROTATIONS`] 개까지 **균등하게** 솎아낸다.
///
/// 앞에서부터 `truncate` 하면 안 된다. `[0, 180]` 같은 성긴 목록에서는 결과가 같지만,
/// 연속 회전을 이산화한 촘촘한 라이브러리(예: `[0, 23, 47, 71, ..., 359]`)에서는 앞
/// 네 개가 전부 0도 근처라 **대칭 배치에서 값어치가 가장 큰 180도 부근이 통째로
/// 사라진다**. 좌/우 대칭 패널을 묶는 것이 이 전략의 존재 이유이므로 그 손실은 치명적이다.
/// [`super::rotations::thin_evenly`] 는 양 끝을 포함해 고르게 집으므로 `0` 과 뒤쪽 큰 각이
/// 함께 살아남는다.
fn capped_rotations(ctx: &StrategyContext<'_, '_>, item_id: usize) -> Vec<i64> {
    let mut rots = ctx.rotations_for(item_id);
    if rots.is_empty() {
        return vec![0];
    }
    // `thin_evenly` 는 오름차순을 전제로 "첫 원소 = 0" 을 보장한다. 오라클 폴백
    // (`allowed_rotations`)이 정렬을 약속하지 않으므로 여기서 직접 세운다.
    rots.sort_unstable();
    rots.dedup();
    super::rotations::thin_evenly(rots, MAX_CLUSTER_ROTATIONS)
}

/// 회전 0 기준 실제 면적(mm²). 못 구하면 `0`.
fn kind_area(ctx: &StrategyContext<'_, '_>, item_id: usize) -> f64 {
    let rot = ctx.rotations_for(item_id).first().copied().unwrap_or(0);
    ctx.placer
        .shape_bounds_area(item_id, rot)
        .map(|(_, area)| area)
        .filter(|a| a.is_finite() && *a > 0.0)
        .unwrap_or(0.0)
}

/// 조각 하나를 따로 놓았을 때의 bbox 충전율.
fn solo_fill_ratio(bounds: Aabb, area: f64) -> f64 {
    let b = bounds.area();
    if b > 0.0 { (area / b).min(1.0) } else { 0.0 }
}

/// 두 월드 bbox 를 감싸는 bbox.
fn hull_box(world_a: Aabb, world_b: Aabb) -> Aabb {
    Aabb {
        min_x: world_a.min_x.min(world_b.min_x),
        min_y: world_a.min_y.min(world_b.min_y),
        max_x: world_a.max_x.max(world_b.max_x),
        max_y: world_a.max_y.max(world_b.max_y),
    }
}

/// 두 월드 bbox 를 감싸는 bbox 의 면적(mm²).
fn hull_area(world_a: Aabb, world_b: Aabb) -> f64 {
    hull_box(world_a, world_b).area()
}

/// 이 쌍이 **strip 길이 방향으로** 아낀 폭(mm) = `w_a + w_b − 묶음 hull 폭`.
///
/// # 왜 면적과 따로 재는가
///
/// 목적함수는 strip **길이**(= `max_x`)지 면적이 아니다. 두 조각을 위아래로 쌓으면
/// 면적은 하나도 안 줄어도 폭은 `min(w_a, w_b)` 만큼 준다. 반대로 좌우로 나란히 붙여
/// 면적을 크게 아껴도 폭이 그대로면 strip 은 1mm 도 짧아지지 않는다.
///
/// 즉 "아낀 면적" 은 클러스터링의 값어치를 **과대평가**할 수 있다. 확정한 쌍이 실제로
/// 폭을 얼마나 아끼는지는 [`ClusterDiagnostics::matched_saved_width`] 로 따로 센다.
fn saved_width(world_a: Aabb, world_b: Aabb, bounds_a: Aabb, bounds_b: Aabb) -> f64 {
    let saved = bounds_a.width() + bounds_b.width() - hull_box(world_a, world_b).width();
    if saved.is_finite() { saved } else { 0.0 }
}

/// 이 쌍을 묶어서 **아낀 면적**과 그 상대 크기.
///
/// 반환은 `(상대 이득, 절대로 아낀 면적 mm²)`. 잴 수 없으면 `None`.
///
/// # 왜 정의를 바꿨나 (실측)
///
/// 옛 정의는 `묶음 충전율 − max(개별 충전율)` 이었다. 이 값은 **짝의 좋고 나쁨이 아니라
/// 두 조각의 크기 차를 잰다**. 나란히 놓으면 묶음 bbox 의 높이가 `max(h_a, h_b)` 로
/// 늘어 낮은 쪽 위에 빈 띠가 생기고, 높이가 정확히 같아 빈 띠가 없는 이상적인 경우조차
/// 묶음 충전율은 두 개별 충전율의 **가중평균**이라 `max` 를 넘지 못한다. 즉 옛 이득이
/// 0을 넘으려면 맞물림이 그 구조적 손해까지 메워야 해서, 눈금이 짝의 품질과 어긋난다.
///
/// 여성WSjk 156-패널 실측(`ClusterDiagnostics`, 임계 0.05):
///
/// ```text
/// 종류 상한   옛 정의 최댓값   옛 정의로 채택   새 정의로 채택
///        8         +0.0021              0                0
///       48         +0.0109              0              162
///      156         +0.1051            189            3,193
/// ```
///
/// 여기서 읽어야 할 것 둘.
///
/// 1. **"쌍이 0개" 의 주범은 정의가 아니라 종류 상한이었다.** 상한 8에서는 두 정의 모두
///    채택 0이고, 상한을 156까지 올리면 **옛 정의도 189쌍을 채택한다**. 정의를 그대로
///    두고 상한만 올렸어도 쌍은 나왔다.
/// 2. 그럼에도 정의를 바꾼 이유는 눈금이 짝의 품질과 어긋나기 때문이다. 같은 상한·같은
///    임계에서 채택 폭이 189 → 3,193 으로 17배 넓어진다. 옛 정의가 걸러 낸 3,000여 쌍은
///    "맞물림이 나빠서" 가 아니라 "두 조각의 크기가 달라서" 떨어진 것들이다.
///
/// 올바른 비교 대상은 "두 조각을 **각각 따로** 배치했을 때 차지하는 면적"
/// (`bbox_a + bbox_b`) 이다. 묶음 hull 이 그보다 작으면 그 차이가 실제로 아낀 면적이다.
///
/// 상대화는 `min(bbox_a, bbox_b)` 로 나눈다. `separate` 로 나누면 큰 조각과 작은 조각의
/// 짝에서 분모가 큰 쪽에 눌려, 작은 조각이 큰 조각의 오목한 곳에 **통째로** 들어가는
/// (= 사실상 공짜로 얹히는) 최고의 짝조차 이득이 몇 %로 찍힌다. `min` 으로 나누면 그
/// 경우가 1.0 에 가까워져 "작은 쪽의 자리를 얼마나 벌었나" 라는 뜻이 된다.
///
/// 이 눈금은 위로 막혀 있다. `hull ≥ max(bbox_a, bbox_b)` 이므로
/// `saved = bbox_a + bbox_b − hull ≤ min(bbox_a, bbox_b)` 이고, 따라서 **이득은 1.0 을
/// 넘을 수 없다**. 1.0 은 "작은 쪽이 완전히 공짜" 라는 뜻이라 임계를 정할 때 기준이
/// 분명하다 — 아래로는 음수까지 열려 있다(묶으면 손해인 짝).
fn pair_gain(hull: f64, separate: f64, bounds_a: Aabb, bounds_b: Aabb) -> Option<(f64, f64)> {
    if !(hull.is_finite() && separate.is_finite() && separate > 0.0) {
        return None;
    }
    let saved = separate - hull;
    let smaller = bounds_a.area().min(bounds_b.area());
    if !(smaller.is_finite() && smaller > 0.0) {
        return None;
    }
    let gain = saved / smaller;
    if !gain.is_finite() {
        return None;
    }
    Some((gain, saved))
}

/// 묶음 충전율 = `(area_a + area_b) / 두 월드 bbox 를 감싸는 bbox 의 면적`.
///
/// 1에 가까울수록 묶음이 직사각형에 가깝다는 뜻이고, 그런 묶음은 strip 어디에 놓아도
/// 주변에 못 쓰는 틈을 덜 남긴다.
fn pair_fill_ratio(world_a: Aabb, world_b: Aabb, area_a: f64, area_b: f64) -> f64 {
    let h = hull_area(world_a, world_b);
    if h > 0.0 {
        ((area_a + area_b) / h).min(1.0)
    } else {
        0.0
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 내부 — 탐욕 매칭
// ────────────────────────────────────────────────────────────────────────────

/// **아낀 면적** 내림차순으로 쌍을 확정한다. 각 인스턴스는 최대 한 번만 쓰인다.
///
/// 같은 종류가 여러 개면 같은 쌍을 수요만큼 반복해 만든다. 이득이 큰 쌍부터 수요를
/// 다 쓰는 것은 순수 탐욕이며 최적이 아니다 — 최적 매칭은 가중 일반 매칭 문제라
/// 전처리에 쓰기엔 비싸고, 여기서 필요한 것은 "명백히 좋은 짝을 놓치지 않는 것" 이다.
///
/// 반환은 [`MatchOutcome`] — 확정한 쌍과 그 쌍들이 이론상 아끼는 면적·폭이다.
///
/// 정렬 기준이 상대 이득(`gain`)이 아니라 절대 면적(`saved_area`)인 이유: 상대 이득은
/// `min(bbox)` 로 나눈 값이라 **작은 조각이 큰 조각에 얹히는 짝이 1.0 근처로 몰린다**.
/// 그 짝들이 먼저 큰 조각의 수요를 다 쓰면, strip 길이를 실제로 줄이는 큰 조각끼리의
/// 짝이 밀려난다. 채택 여부(임계)는 상대 이득으로, 순서는 절대 면적으로 나눈다.
fn greedy_match(mut candidates: Vec<PairCandidate>, remaining: &mut [usize]) -> MatchOutcome {
    candidates.sort_by(|x, y| {
        y.saved_area
            .partial_cmp(&x.saved_area)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut out = Vec::new();
    let mut saved_area = 0.0f64;
    let mut saved_width = 0.0f64;
    for cand in candidates {
        let a = cand.cluster.anchor_item;
        let b = cand.cluster.partner_item;

        loop {
            let pairs_left = if a == b {
                remaining.get(a).copied().unwrap_or(0) / 2
            } else {
                remaining
                    .get(a)
                    .copied()
                    .unwrap_or(0)
                    .min(remaining.get(b).copied().unwrap_or(0))
            };
            if pairs_left == 0 {
                break;
            }
            if a == b {
                if let Some(slot) = remaining.get_mut(a) {
                    *slot = slot.saturating_sub(2);
                }
            } else {
                if let Some(slot) = remaining.get_mut(a) {
                    *slot = slot.saturating_sub(1);
                }
                if let Some(slot) = remaining.get_mut(b) {
                    *slot = slot.saturating_sub(1);
                }
            }
            out.push(cand.cluster);
            saved_area += cand.saved_area;
            saved_width += cand.saved_width;
        }
    }
    MatchOutcome {
        clusters: out,
        saved_area,
        saved_width,
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 내부 — 예산
// ────────────────────────────────────────────────────────────────────────────

/// 두 예산 중 이른 쪽. 둘 다 없으면 `None`.
fn earliest(a: Option<Instant>, b: Option<Instant>) -> Option<Instant> {
    match (a, b) {
        (Some(x), Some(y)) => Some(x.min(y)),
        (Some(x), None) => Some(x),
        (None, Some(y)) => Some(y),
        (None, None) => None,
    }
}

/// 예산을 넘겼는가. Init 취소 요청도 예산 소진으로 본다.
fn expired(limit: Option<Instant>) -> bool {
    if crate::config::runtime::init_cancel_requested() {
        return true;
    }
    match limit {
        Some(d) => Instant::now() >= d,
        None => false,
    }
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

    fn cluster_at(anchor: usize, partner: usize, offset: (f64, f64)) -> Cluster {
        Cluster {
            anchor_item: anchor,
            anchor_rotation: 0,
            partner_item: partner,
            partner_rotation: 180,
            offset,
            fill_ratio: 0.9,
            partner_bounds: bbox(0.0, 0.0, 10.0, 10.0),
        }
    }

    // ── 충전율 ──────────────────────────────────────────────────────────

    #[test]
    fn two_squares_side_by_side_fill_their_bounding_box_completely() {
        // 10×10 두 개를 나란히: 묶음 bbox 20×10 = 200, 면적 합 200 → 1.0
        let a = bbox(0.0, 0.0, 10.0, 10.0);
        let b = bbox(10.0, 0.0, 20.0, 10.0);
        assert_eq!(pair_fill_ratio(a, b, 100.0, 100.0), 1.0);
    }

    #[test]
    fn two_squares_placed_diagonally_fill_only_half_their_bounding_box() {
        // 대각으로 놓으면 묶음 bbox 20×20 = 400, 면적 합 200 → 0.5
        let a = bbox(0.0, 0.0, 10.0, 10.0);
        let b = bbox(10.0, 10.0, 20.0, 20.0);
        assert_eq!(pair_fill_ratio(a, b, 100.0, 100.0), 0.5);
    }

    #[test]
    fn a_degenerate_bounding_box_yields_zero_fill_instead_of_a_division_by_zero() {
        let flat = bbox(5.0, 5.0, 5.0, 5.0);
        assert_eq!(pair_fill_ratio(flat, flat, 0.0, 0.0), 0.0);
    }

    // ── 이득 정의 ───────────────────────────────────────────────────────

    #[test]
    fn the_old_fill_ratio_gain_can_never_be_positive_for_pieces_merely_set_side_by_side() {
        // 왜 정의를 바꿨는지 못 박는 회귀 검사.
        //
        // 10×10 두 조각(실면적 75)을 나란히 놓으면 묶음 bbox 는 20×10 이다.
        // 옛 정의 = 묶음 충전율(150/200 = 0.75) − 개별 충전율 max(0.75) = 0.
        //
        // **높이가 같은 이상적인 경우조차 0** 이다. 즉 옛 이득이 양수가 되려면 실제
        // 맞물림이 있어야 하는데, 그 양이 두 조각의 크기 차에 눌린다 — 실제 마커에서
        // 옛 정의의 최댓값이 +0.105 에 그친 이유다(`pair_gain` 의 표).
        let a = bbox(0.0, 0.0, 10.0, 10.0);
        let b = bbox(10.0, 0.0, 20.0, 10.0);
        let solo = solo_fill_ratio(a, 75.0);
        assert_eq!(pair_fill_ratio(a, b, 75.0, 75.0) - solo, 0.0);

        // 높이가 다르면 빈 띠가 생겨 **음수**가 된다.
        let low = bbox(10.0, 0.0, 20.0, 5.0); // 10×5, 실면적 40
        let mixed = pair_fill_ratio(a, low, 75.0, 40.0);
        let solo_mixed = solo_fill_ratio(a, 75.0).max(solo_fill_ratio(low, 40.0));
        assert!(
            mixed - solo_mixed < 0.0,
            "옛 정의는 높이가 다른 짝에서 구조적으로 음수다: {mixed} vs {solo_mixed}"
        );
    }

    #[test]
    fn the_gain_is_the_area_saved_against_placing_both_pieces_on_their_own() {
        let a = bbox(0.0, 0.0, 10.0, 10.0);
        let separate = 100.0 + 100.0;

        // 나란히 — 아낀 것이 없다.
        let side = bbox(10.0, 0.0, 20.0, 10.0);
        let (gain, saved) = pair_gain(hull_area(a, side), separate, a, side).expect(crate::dmsg!("잴 수 있다"));
        assert_eq!(saved, 0.0);
        assert_eq!(gain, 0.0);

        // 반씩 맞물리면 hull 이 15×10 = 150 으로 줄어 50 을 아낀다 → 작은 쪽 bbox 의 50%.
        let tight = bbox(5.0, 0.0, 15.0, 10.0);
        let (gain, saved) = pair_gain(hull_area(a, tight), separate, a, tight).expect(crate::dmsg!("잴 수 있다"));
        assert_eq!(saved, 50.0);
        assert_eq!(gain, 0.5);

        // 대각으로 벌어지면 hull 이 커져 **음수**다 — 묶으면 손해라는 뜻이고 맞다.
        let far = bbox(10.0, 10.0, 20.0, 20.0);
        let (gain, saved) = pair_gain(hull_area(a, far), separate, a, far).expect(crate::dmsg!("잴 수 있다"));
        assert_eq!(saved, -200.0);
        assert!(gain < 0.0);
    }

    #[test]
    fn a_small_piece_tucked_inside_a_big_ones_bounding_box_scores_near_one() {
        // 큰 조각 100×100, 작은 조각 10×10 이 그 오목한 곳에 통째로 들어간다.
        // hull 은 큰 조각의 bbox 그대로 → 작은 조각은 공짜로 얹힌 셈이다.
        let big = bbox(0.0, 0.0, 100.0, 100.0);
        let small = bbox(10.0, 10.0, 20.0, 20.0);
        let separate = big.area() + small.area();
        let (gain, saved) = pair_gain(hull_area(big, small), separate, big, small).expect(crate::dmsg!("잴 수 있다"));
        assert_eq!(saved, 100.0);
        // `separate`(10100)로 나눴다면 0.0099 라 임계 0.05 를 못 넘는다. `min(bbox)`
        // 로 나누므로 1.0 이 된다 — 크기 차가 큰 좋은 짝을 놓치지 않는 이유다.
        assert_eq!(gain, 1.0);
    }

    #[test]
    fn a_degenerate_piece_yields_no_gain_instead_of_a_division_by_zero() {
        let flat = bbox(5.0, 5.0, 5.0, 5.0);
        let a = bbox(0.0, 0.0, 10.0, 10.0);
        assert!(pair_gain(hull_area(a, flat), 100.0, a, flat).is_none());
        assert!(pair_gain(f64::NAN, 100.0, a, a).is_none());
        assert!(pair_gain(100.0, 0.0, a, a).is_none());
    }

    #[test]
    fn the_pair_probe_cap_sits_above_what_the_oracle_can_return() {
        // `MAX_PAIR_PROBES` 는 오라클의 상한보다 커야 **의미가 있다**. 아래로 내려가는
        // 순간 쌍 탐색이 후보를 잘라 보기 시작하는데, 자르는 순서가 x 오름차순이라
        // 앵커 오른쪽에서 맞물리는 자리가 통째로 사라진다 — 그러면 "짝이 없다" 가
        // 사실이 아니라 절단의 결과가 된다. 두 값이 엇갈리면 여기서 잡는다.
        assert!(
            MAX_PAIR_PROBES >= crate::nfp_lbf::MAX_EVALUATED_CANDIDATES,
            "쌍 탐색 상한({MAX_PAIR_PROBES})이 오라클 상한({})보다 작으면 후보가 잘린다",
            crate::nfp_lbf::MAX_EVALUATED_CANDIDATES,
        );
    }

    // ── 펼치기 ──────────────────────────────────────────────────────────

    #[test]
    fn expanding_a_cluster_places_the_partner_at_the_anchor_plus_the_offset() {
        let cluster = cluster_at(3, 7, (12.5, -4.0));
        let anchor = AnchorOption {
            item_id: 3,
            rotation: 0,
            x: 100.0,
            y: 50.0,
            world: bbox(100.0, 50.0, 130.0, 80.0),
            touches: 0,
        };

        let partner = expand_cluster(&cluster, &anchor).expect(crate::dmsg!("앵커가 맞으면 펼쳐져야 한다"));
        assert_eq!(partner.item_id, 7);
        assert_eq!(partner.rotation, 180);
        assert_eq!(partner.x, 112.5);
        assert_eq!(partner.y, 46.0);
        // 월드 bbox 는 파트너의 앵커 기준 bbox 를 그 자리로 옮긴 것이다.
        assert_eq!(partner.world, bbox(112.5, 46.0, 122.5, 56.0));
    }

    #[test]
    fn expanding_refuses_an_anchor_whose_piece_or_pose_does_not_match() {
        let cluster = cluster_at(3, 7, (1.0, 1.0));
        let wrong_item = AnchorOption {
            item_id: 4,
            rotation: 0,
            x: 0.0,
            y: 0.0,
            world: bbox(0.0, 0.0, 1.0, 1.0),
            touches: 0,
        };
        assert!(expand_cluster(&cluster, &wrong_item).is_none());

        let wrong_pose = AnchorOption {
            item_id: 3,
            rotation: 90,
            x: 0.0,
            y: 0.0,
            world: bbox(0.0, 0.0, 1.0, 1.0),
            touches: 0,
        };
        assert!(expand_cluster(&cluster, &wrong_pose).is_none());
    }

    // ── 탐욕 매칭 ───────────────────────────────────────────────────────

    /// 탐욕 매칭은 정렬에 `saved_area` 만 본다. `saved_width` 는 합계만 따라가므로
    /// 여기서는 면적의 1/10 로 두어 합산이 제대로 도는지 함께 확인한다.
    fn candidate(cluster: Cluster, saved_area: f64) -> PairCandidate {
        PairCandidate {
            cluster,
            gain: saved_area,
            saved_area,
            saved_width: saved_area / 10.0,
        }
    }

    #[test]
    fn greedy_matching_never_uses_an_instance_twice() {
        let strong = candidate(cluster_at(0, 1, (1.0, 0.0)), 300.0);
        let weak = candidate(cluster_at(0, 2, (2.0, 0.0)), 100.0);
        // 0 이 2개, 1 이 2개, 2 가 5개.
        let mut remaining = vec![2, 2, 5];
        let outcome = greedy_match(vec![weak, strong], &mut remaining);
        let out = outcome.clusters;

        // 이득이 큰 (0,1) 이 0 의 수요를 전부 먹는다 → (0,2) 는 만들 수 없다.
        assert_eq!(out.len(), 2);
        // 아낀 면적·폭은 확정한 쌍의 것만 더한다 (300 × 2쌍, 폭은 그 1/10).
        assert_eq!(outcome.saved_area, 600.0);
        assert_eq!(outcome.saved_width, 60.0);
        assert!(out.iter().all(|c| c.partner_item == 1));
        assert_eq!(remaining, vec![0, 0, 5]);
    }

    #[test]
    fn a_kind_paired_with_itself_consumes_two_of_its_demand_per_cluster() {
        let same = candidate(cluster_at(0, 0, (5.0, 0.0)), 50.0);
        // 5개면 쌍 2개까지만 만들어지고 1개가 남는다.
        let mut remaining = vec![5];
        let out = greedy_match(vec![same], &mut remaining).clusters;
        assert_eq!(out.len(), 2);
        assert_eq!(remaining, vec![1]);
    }

    #[test]
    fn a_kind_with_a_single_instance_cannot_pair_with_itself() {
        let same = candidate(cluster_at(0, 0, (5.0, 0.0)), 50.0);
        let mut remaining = vec![1];
        assert!(greedy_match(vec![same], &mut remaining).clusters.is_empty());
        assert_eq!(remaining, vec![1]);
    }

    #[test]
    fn a_pair_referring_to_a_missing_kind_is_skipped_instead_of_panicking() {
        // 표 밖의 id 는 수요 0으로 읽혀야 한다 — 인덱싱 panic 이 나면 안 된다.
        let bogus = candidate(cluster_at(0, 99, (1.0, 0.0)), 90.0);
        let mut remaining = vec![3];
        assert!(greedy_match(vec![bogus], &mut remaining).clusters.is_empty());
    }

    // ── 오라클을 실제로 태우는 통합 검사 ────────────────────────────────

    use jagua_rs::collision_detection::CDEConfig;
    use jagua_rs::entities::Item;
    use jagua_rs::geometry::DTransformation;
    use jagua_rs::geometry::OriginalShape;
    use jagua_rs::geometry::fail_fast::SPSurrogateConfig;
    use jagua_rs::geometry::geo_enums::RotationRange;
    use jagua_rs::geometry::primitives::{Point, SPolygon};
    use jagua_rs::geometry::shape_modification::{ShapeModifyConfig, ShapeModifyMode};
    use jagua_rs::probs::spp::entities::Strip;

    use super::super::config::DEFAULT_NFP_STRATEGY_CONFIG;
    use super::super::rotations::item_rotation_table;
    use crate::nfp_lbf::cache::{NfpLbfCache, PreparedLibrary};
    use crate::nfp_lbf::{PlacementScoreWeights, SequentialPlacer};

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

    const STRIP_HEIGHT: f32 = 60.0;
    const STRIP_WIDTH_CAP: f32 = 400.0;

    /// L 자 두 개짜리 인스턴스와 그에 맞춘 NFP 캐시·회전표.
    ///
    /// `PreparedLibrary::build` 는 인스턴스와 회전 목록만 보는 **결정적** 함수다.
    /// 예전 버전은 실패를 `return` 으로 조용히 넘겼는데, 그러면 라이브러리가 깨진
    /// 순간부터 이 통합 검사가 아무것도 검증하지 않으면서 초록으로 남는다. 실패가
    /// 곧 결함이므로 여기서는 터뜨린다.
    fn l_shape_fixture() -> (SPInstance, NfpLbfCache, Vec<Vec<i64>>) {
        // 20×20 bbox 에서 10×10 을 도려낸 L. 실면적 300, bbox 400 → 개별 충전율 0.75.
        let shape = SPolygon::new(vec![
            Point(0.0, 0.0),
            Point(20.0, 0.0),
            Point(20.0, 10.0),
            Point(10.0, 10.0),
            Point(10.0, 20.0),
            Point(0.0, 20.0),
        ])
        .expect(crate::dmsg!("valid L"));
        let item = Item::new(0, (0) as u64, OriginalShape {
                shape,
                pre_transform: DTransformation::empty(),
                modify_mode: ShapeModifyMode::Inflate,
                modify_config: ShapeModifyConfig::default(),
            }, jagua_rs::geometry::AllowedOrientations::new(RotationRange::Discrete(vec![0.0, std::f32::consts::PI]), vec![]).unwrap(), None, SURROGATE).map(std::sync::Arc::new)
        .expect(crate::dmsg!("valid item"));

        let strip = Strip::new(
            STRIP_HEIGHT,
            CDE,
            ShapeModifyConfig::default(),
            STRIP_WIDTH_CAP,
        )
        .expect(crate::dmsg!("strip"));
        let instance = SPInstance::new(vec![(item, 2)], strip);

        let library_rots = vec![0i64, 180];
        let library = PreparedLibrary::build(&instance, &library_rots, 0.0)
            .expect(crate::dmsg!("L 두 개짜리 인스턴스의 NFP 라이브러리는 결정적으로 만들어져야 한다"));
        let table = item_rotation_table(&instance, &library_rots);
        (instance, NfpLbfCache::new(library), table)
    }

    /// L 자 두 개는 180도 돌려 맞물리면 직사각형이 된다 — 그 짝을 찾아내는지 본다.
    #[test]
    fn two_interlocking_l_shapes_are_clustered_because_the_pair_saves_a_measurable_area() {
        let (instance, mut cache, table) = l_shape_fixture();
        let mut placer = SequentialPlacer::with_item_rotations(
            &mut cache,
            STRIP_HEIGHT,
            STRIP_WIDTH_CAP,
            PlacementScoreWeights::default(),
            table.clone(),
        );

        let cfg = super::super::NfpStrategyConfig {
            pairwise_clustering: true,
            ..DEFAULT_NFP_STRATEGY_CONFIG
        };
        let mut ctx = StrategyContext::new(&mut placer, cfg, table, None);

        let clusters = build_clusters(&mut ctx, &instance, None);
        assert_eq!(clusters.len(), 1, "수요 2개면 쌍은 하나만 나온다");

        let Some(&c) = clusters.first() else {
            unreachable!("{}", crate::dmsg!("바로 위에서 길이 1을 확인했다"));
        };
        assert_eq!(c.anchor_item, 0);
        assert_eq!(c.partner_item, 0);

        // **채택 기준을 그대로 다시 계산해 확인한다.**
        //
        // `fill_ratio` 로 확인하면 안 된다 — 그건 보고용 값이고, 이 쌍이 통과한 관문은
        // [`pair_gain`] 이다. 실제로 걸러 낸 값이 아닌 다른 값을 재는 검사는 정의를
        // 되돌려도 초록으로 남는다.
        let (bounds_a, _) = ctx
            .placer
            .shape_bounds_area(c.anchor_item, c.anchor_rotation)
            .expect(crate::dmsg!("앵커 형상 bbox"));
        let world_a = bounds_a.translated(0.0, 0.0);
        let world_b = c.partner_bounds.translated(c.offset.0, c.offset.1);
        let separate = bounds_a.area() + c.partner_bounds.area();
        let (gain, saved) =
            pair_gain(hull_area(world_a, world_b), separate, bounds_a, c.partner_bounds)
                .expect(crate::dmsg!("확정된 쌍은 이득을 잴 수 있어야 한다"));
        assert!(
            gain > DEFAULT_NFP_STRATEGY_CONFIG.cluster_min_gain as f64,
            "확정된 쌍은 임계를 넘은 이득을 갖고 있어야 한다: {gain}"
        );
        // 아낄 수 있는 면적의 **상한**은 기하가 정한다. L 하나의 실면적이 300 이라
        // 묶음 hull 은 600 아래로 내려갈 수 없고, 따로 놓으면 400+400 = 800 이다.
        // 즉 최대 200 을 아낄 수 있고, 그때가 두 L 이 빈틈없이 맞물린 경우다.
        // 실측 199.50 = 상한의 99.7% (hull 600.5).
        const PERFECT_SAVING: f64 = 800.0 - 600.0;
        assert!(
            saved > 0.95 * PERFECT_SAVING && saved <= PERFECT_SAVING,
            "180도 돌린 L 두 개는 거의 빈틈없이 맞물려야 한다: 아낀 면적 {saved} \
             (기하 상한 {PERFECT_SAVING})"
        );
        // 보고용 충전율도 개별(0.75)보다 커야 앞뒤가 맞는다.
        assert!(
            c.fill_ratio > 0.75,
            "묶음 충전율이 개별(0.75)보다 커야 한다: {}",
            c.fill_ratio
        );

        // 펼치면 앵커에서 offset 만큼 떨어진 자리가 나온다.
        let anchor = AnchorOption {
            item_id: 0,
            rotation: c.anchor_rotation,
            x: 5.0,
            y: 7.0,
            world: bbox(5.0, 7.0, 25.0, 27.0),
            touches: 0,
        };
        let partner = expand_cluster(&c, &anchor).expect(crate::dmsg!("앵커 자세가 맞으면 펼쳐진다"));
        assert_eq!(partner.x, 5.0 + c.offset.0);
        assert_eq!(partner.y, 7.0 + c.offset.1);
    }

    /// 묶인 두 조각이 **실제로 겹치지 않는지** 본다.
    ///
    /// 쌍의 상대 위치는 NFP 경계 위에서 골랐으니 정의상 겹치지 않아야 한다. 그런데
    /// 앵커를 f32 로 접는 지점이 있어서, 좌표가 한 번 어긋나면 이 성질이 조용히
    /// 깨진다. 오라클에 앵커를 장애물로 주고 파트너 좌표가 여전히 feasible 후보
    /// 집합 안에 있는지 확인해 그 성질을 못 박는다.
    #[test]
    fn the_two_pieces_of_a_cluster_are_still_a_feasible_pair_for_the_oracle() {
        let (instance, mut cache, table) = l_shape_fixture();
        let mut placer = SequentialPlacer::with_item_rotations(
            &mut cache,
            STRIP_HEIGHT,
            STRIP_WIDTH_CAP,
            PlacementScoreWeights::default(),
            table.clone(),
        );
        let cfg = super::super::NfpStrategyConfig {
            pairwise_clustering: true,
            ..DEFAULT_NFP_STRATEGY_CONFIG
        };
        let mut ctx = StrategyContext::new(&mut placer, cfg, table, None);

        let clusters = build_clusters(&mut ctx, &instance, None);
        let Some(&c) = clusters.first() else {
            panic!("{}", crate::dmsg!("맞물리는 L 쌍은 반드시 나와야 한다"));
        };

        // `best_pair` 가 쓴 것과 같은 앵커 자리를 다시 만든다 — 바닥 줄과 천장 줄.
        // 쌍은 둘 중 한 줄에서 나왔으므로 그 줄에서는 반드시 feasible 집합 안에 있다.
        let (bounds_a, _) = ctx
            .placer
            .shape_bounds_area(c.anchor_item, c.anchor_rotation)
            .expect(crate::dmsg!("앵커 형상 bbox"));
        let (bounds_b, _) = ctx
            .placer
            .shape_bounds_area(c.partner_item, c.partner_rotation)
            .expect(crate::dmsg!("파트너 형상 bbox"));
        let pad = bounds_a.width().max(bounds_b.width());
        let ta_x = (-bounds_a.min_x + pad) as f32;
        let rows = [
            (-bounds_a.min_y) as f32,
            (STRIP_HEIGHT as f64 - bounds_a.max_y) as f32,
        ];

        let found_on_some_row = rows.iter().any(|&ta_y| {
            let anchor_piece = PlacedPiece {
                item_id: c.anchor_item,
                rotation: c.anchor_rotation,
                translation: (ta_x, ta_y),
            };
            let anchor = AnchorOption {
                item_id: c.anchor_item,
                rotation: c.anchor_rotation,
                x: ta_x as f64,
                y: ta_y as f64,
                world: bounds_a.translated(ta_x as f64, ta_y as f64),
                touches: 0,
            };
            let partner = expand_cluster(&c, &anchor).expect(crate::dmsg!("앵커 자세가 맞으면 펼쳐진다"));

            let feasible = ctx
                .placer
                .candidates_for(c.partner_item, c.partner_rotation, &[anchor_piece]);
            feasible.iter().any(|cand| {
                (cand.translation.0 as f64 - partner.x).abs() < 1e-6
                    && (cand.translation.1 as f64 - partner.y).abs() < 1e-6
            })
        });
        assert!(
            found_on_some_row,
            "펼친 파트너 좌표가 어느 앵커 줄의 feasible 집합에도 없다 — 상대 오프셋이 어긋났다"
        );
    }

    #[test]
    fn clustering_returns_nothing_at_all_while_the_strategy_is_off() {
        // 기본 설정에서 ②는 꺼져 있다.
        assert!(!DEFAULT_NFP_STRATEGY_CONFIG.pairwise_clustering);

        let (instance, mut cache, table) = l_shape_fixture();
        let mut placer = SequentialPlacer::with_item_rotations(
            &mut cache,
            STRIP_HEIGHT,
            STRIP_WIDTH_CAP,
            PlacementScoreWeights::default(),
            table.clone(),
        );
        let mut ctx = StrategyContext::new(&mut placer, DEFAULT_NFP_STRATEGY_CONFIG, table, None);

        // 같은 인스턴스가 켜져 있을 때는 쌍을 하나 내놓는다(위 검사). 즉 이 빈 결과는
        // "짝이 없어서" 가 아니라 **플래그가 꺼져서** 나온 것이다.
        assert!(
            build_clusters(&mut ctx, &instance, None).is_empty(),
            "꺼진 전략이 일을 하면 안 된다"
        );
    }

    #[test]
    fn an_expired_budget_yields_no_clusters_instead_of_running_the_pair_search() {
        let (instance, mut cache, table) = l_shape_fixture();
        let mut placer = SequentialPlacer::with_item_rotations(
            &mut cache,
            STRIP_HEIGHT,
            STRIP_WIDTH_CAP,
            PlacementScoreWeights::default(),
            table.clone(),
        );
        let cfg = super::super::NfpStrategyConfig {
            pairwise_clustering: true,
            ..DEFAULT_NFP_STRATEGY_CONFIG
        };
        let mut ctx = StrategyContext::new(&mut placer, cfg, table, None);

        // 이미 지난 시각. `expired` 가 `>=` 라 같은 시각이어도 만료로 읽힌다.
        let past = Instant::now();
        assert!(
            build_clusters(&mut ctx, &instance, Some(past)).is_empty(),
            "예산이 없으면 빈 목록이 안전한 폴백이다"
        );
    }

    // ── 실측: 의류 마커에서 쌍 탐색이 어디서 막히는가 ───────────────────

    /// 실제 마커(`data/input/wsjk156.json`)에서 쌍 탐색의 각 관문을 **세어서** 남긴다.
    ///
    /// "쌍이 0개" 의 원인 후보는 넷이었다 — 종류 상한 / 이득 정의 / 임계값 / 예산.
    /// 이 테스트가 넷을 분리한다. 상한을 훑으므로 [`MAX_CLUSTER_ITEM_KINDS`] 와
    /// [`CLUSTER_BUILD_BUDGET`] 의 근거표도 여기서 나온다.
    ///
    /// ```text
    /// cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" \
    ///     --lib nfp_strategy::cluster::tests::pair_search_on_the_garment_marker -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "실측용 — 인스턴스 파일을 읽고 수십 초 걸린다"]
    fn pair_search_on_the_garment_marker_is_measured_not_guessed() {
        // 인스턴스가 없는 체크아웃에서는 잴 것이 없다. 없는 것을 실패로 만들면
        // "측정을 못 했다" 가 "코드가 깨졌다" 로 잘못 읽힌다.
        let Some(instance) = garment_marker() else {
            return;
        };

        let strip_height = instance.base_strip.fixed_height;
        // 최악의 경우 폭 — 모든 조각을 한 줄로 늘어놓은 길이.
        let strip_width_cap: f32 = instance
            .items
            .iter()
            .map(|(item, qty)| item.shape_cd().bbox().width() * (*qty as f32))
            .filter(|w| w.is_finite() && *w > 0.0)
            .sum();

        // 이 인스턴스는 모든 아이템이 `allowed_orientations = [0]` 이므로 회전은 1종이다.
        let rotations = vec![0i64];
        let n_kinds = instance.items.len();
        let n_pieces: usize = instance.items.iter().map(|(_, q)| *q).sum();
        println!(
            "\n== wsjk156: 종류 {n_kinds}개 / 조각 {n_pieces}개 / strip 높이 {strip_height:.1} =="
        );

        for &max_kinds in &[8usize, 16, 24, 32, 48, 64, 96, 156] {
            // 캡마다 캐시를 새로 만든다. 같은 캐시를 물려주면 앞 라운드가 데워 둔
            // NFP 덕분에 뒤 라운드가 부당하게 빨라져 시간 비교가 무의미해진다.
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
                table.clone(),
            )
            .with_probe_budget(4096);
            let cfg = super::super::NfpStrategyConfig {
                pairwise_clustering: true,
                ..DEFAULT_NFP_STRATEGY_CONFIG
            };
            let mut ctx = StrategyContext::new(&mut placer, cfg, table, None);

            let (clusters, d) = build_clusters_measured(&mut ctx, &instance, None, max_kinds);
            println!(
                "[상한 {max_kinds:>2}] 종류 {:>3}/{:>3} · 쌍 {:>4} 평가 · 후보 있음 {:>4} · \
                 임계미달 {:>4} · 채택 {:>4} · 확정 {:>3} · 탐침 {:>7} · {:>6}ms{} \
                 | 최대이득 {:+.4} · 옛정의 최대 {:+.4} / 옛정의로 채택될 쌍 {:>4}",
                d.kinds_considered,
                d.kinds_total,
                d.pairs_evaluated,
                d.pairs_with_candidate,
                d.pairs_below_min_gain,
                d.pairs_accepted,
                clusters.len(),
                d.probes_evaluated,
                d.elapsed_ms,
                if d.budget_hit { " (예산소진)" } else { "" },
                d.best_gain,
                d.best_fill_gain,
                d.pairs_accepted_by_old_gain,
            );
            if !clusters.is_empty() {
                println!(
                    "            └ 확정 쌍이 이론상 아끼는 면적 {:.0}mm² / 폭 {:.0}mm",
                    d.matched_saved_area, d.matched_saved_width,
                );
            }
            assert_eq!(d.clusters_matched, clusters.len());
        }
    }

    /// 마커 인스턴스를 읽어 준다. 없으면 `None` — 계측 테스트는 조용히 건너뛴다.
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

    /// ②를 켠 배치가 **실제 마커에서 무엇을 바꾸는지** 끝까지 돌려 본다.
    ///
    /// `build_clusters` 가 쌍을 만드는 것과 그 쌍이 배치에 반영되는 것은 별개다.
    /// 앞의 계측 테스트는 전자만 본다. 이 테스트가 후자를 본다.
    ///
    /// ```text
    /// cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" \
    ///     --lib nfp_strategy::cluster::tests::clustering_end_to_end -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "실측용 — ordering 을 실제로 돌려 분 단위까지 걸린다"]
    fn clustering_end_to_end_on_the_garment_marker() {
        use crate::nfp_lbf::{NfpLbfConfig, build_best_initial_nest};
        use rand::SeedableRng;

        let Some(instance) = garment_marker() else {
            return;
        };

        let cfg_for = |strategy: super::super::NfpStrategyConfig| NfpLbfConfig {
            n_orderings: 12,
            cache_budget: std::time::Duration::from_secs(10),
            total_budget: std::time::Duration::from_secs(600),
            concave_simplify_tolerance: 0.02,
            max_candidate_probes: 4096,
            strategy,
            ..NfpLbfConfig::default()
        };

        for (label, strategy) in [
            ("기준 (전부 끔)", DEFAULT_NFP_STRATEGY_CONFIG),
            (
                "② 쌍별 클러스터링",
                super::super::NfpStrategyConfig {
                    pairwise_clustering: true,
                    ..DEFAULT_NFP_STRATEGY_CONFIG
                },
            ),
        ] {
            let mut rng = rand::rngs::Xoshiro256PlusPlus::seed_from_u64(0);
            let t = Instant::now();
            match build_best_initial_nest(&instance, &cfg_for(strategy), &mut rng) {
                Some(nest) => println!(
                    "[{label:<18}] 폭 {:9.2} · 밀도 {:5.2}% · 붙인 쌍 {:>3} · ordering {} · {}ms",
                    nest.solution.strip_width(),
                    nest.solution.density() * 100.0,
                    nest.report.clusters_applied,
                    nest.report.orderings_evaluated,
                    t.elapsed().as_millis(),
                ),
                None => println!("[{label:<18}] 시드 생성 실패"),
            }
        }
    }

    /// 쌍을 **배치 도중에** 붙이는 그리디 한 패스 — 프로토타입.
    ///
    /// 지금의 [`super::super::pipeline`] 은 쌍을 배치가 **끝난 뒤** 미배치 조각에만
    /// 붙인다. 이 마커는 156개가 전부 배치되므로 그 경로는 영영 돌지 않는다(실측:
    /// 붙인 쌍 0). 파이프라인을 고칠 값어치가 있는지 판단하려면 "고쳤을 때 무엇이
    /// 달라지는가" 를 먼저 재야 한다. 그 재기 위한 최소 구현이다.
    ///
    /// 반환은 `(strip 길이, 놓은 조각 수, 붙인 쌍 수)`.
    fn greedy_pass(
        ctx: &mut StrategyContext<'_, '_>,
        sequence: &[usize],
        clusters: &[Cluster],
        pair_aware: bool,
    ) -> (f64, usize, usize) {
        use super::super::select::{self, SelectionRule};

        /// 유도한 파트너 자리를 실제 후보에 붙일 때 허용할 어긋남(mm).
        /// `pipeline::CLUSTER_SNAP_MM` 과 같은 값이다.
        const SNAP_MM: f64 = 0.5;

        fn best_option(
            ctx: &mut StrategyContext<'_, '_>,
            item_id: usize,
            placed: &[PlacedPiece],
        ) -> Option<AnchorOption> {
            let options = ctx.options_for(item_id, placed);
            if options.is_empty() {
                return None;
            }
            let boxes = ctx.placer.world_bounds(placed);
            let frontier = boxes.iter().map(|b| b.max_x).fold(0.0_f64, f64::max);
            let metrics: Vec<_> = options
                .iter()
                .map(|o| ctx.metrics(o, &boxes, frontier))
                .collect();
            let idx = select::pick_best(&options, &metrics, SelectionRule::LeftBottom)?;
            options.get(idx).copied()
        }

        /// 쌍을 **한 덩어리로** 놓아 본다.
        ///
        /// 앵커를 평범하게 놓고 파트너를 나중에 붙이면, 앵커 자리는 자기만 보고 고른
        /// 것이라 파트너가 들어갈 자리가 남아 있을 이유가 없다(실측: 확정 43쌍 중
        /// 13쌍만 붙었다). 여기서는 **쌍 전체의 strip 길이**가 가장 짧은 앵커 자리부터
        /// 시험한다 — 그게 super-piece 로 놓는다는 말의 뜻이다.
        ///
        /// 반환은 `(앵커 자리, 파트너 자리)`.
        fn try_place_pair(
            ctx: &mut StrategyContext<'_, '_>,
            placed: &[PlacedPiece],
            cluster: Cluster,
        ) -> Option<(AnchorOption, AnchorOption)> {
            /// 쌍으로 시험해 볼 앵커 자리 수. 자리마다 파트너 후보를 다시 열거하므로
            /// 이 값이 곧 비용 배수다.
            const PAIR_ANCHOR_TRIALS: usize = 8;

            let strip_h = ctx.placer.strip_height() as f64;
            let options = ctx.options_for(cluster.anchor_item, placed);
            let mut ranked: Vec<(f64, AnchorOption, AnchorOption)> = options
                .iter()
                .filter(|o| o.rotation == cluster.anchor_rotation)
                .filter_map(|o| {
                    let t = expand_cluster(&cluster, o)?;
                    // 유도한 파트너 자리가 strip 밖이면 애초에 쌍이 성립하지 않는다.
                    if t.world.min_x < 0.0 || t.world.min_y < 0.0 || t.world.max_y > strip_h {
                        return None;
                    }
                    Some((o.world.max_x.max(t.world.max_x), *o, t))
                })
                .collect();
            ranked.sort_by(|a, b| a.0.total_cmp(&b.0));

            for (_, anchor, target) in ranked.into_iter().take(PAIR_ANCHOR_TRIALS) {
                let mut trial = placed.to_vec();
                trial.push(anchor.to_piece());
                let snapped = ctx
                    .options_for(cluster.partner_item, &trial)
                    .into_iter()
                    .filter(|o| o.rotation == target.rotation)
                    .map(|o| ((o.x - target.x).hypot(o.y - target.y), o))
                    .filter(|(d, _)| *d <= SNAP_MM)
                    .min_by(|a, b| a.0.total_cmp(&b.0))
                    .map(|(_, o)| o);
                if let Some(partner) = snapped {
                    return Some((anchor, partner));
                }
            }
            None
        }

        let mut placed: Vec<PlacedPiece> = Vec::with_capacity(sequence.len());
        let mut used = vec![false; sequence.len()];
        let mut attached = 0usize;

        for i in 0..sequence.len() {
            if used.get(i).copied().unwrap_or(true) {
                continue;
            }
            let Some(&item_id) = sequence.get(i) else {
                continue;
            };

            // 이 조각이 앵커인 클러스터 중 파트너가 아직 남아 있는 것.
            let picked = clusters
                .iter()
                .find(|c| {
                    c.anchor_item == item_id
                        && sequence
                            .iter()
                            .zip(used.iter())
                            .any(|(&it, &u)| !u && it == c.partner_item)
                })
                .copied();

            if let Some(cluster) = picked {
                let slot = sequence
                    .iter()
                    .zip(used.iter())
                    .position(|(&it, &u)| !u && it == cluster.partner_item);
                let pair = if pair_aware {
                    try_place_pair(ctx, &placed, cluster)
                } else {
                    // 옛 방식: 앵커를 자기만 보고 놓고, 그 자리에서 파트너를 붙여 본다.
                    best_option(ctx, item_id, &placed).and_then(|anchor| {
                        if anchor.rotation != cluster.anchor_rotation {
                            return None;
                        }
                        let target = expand_cluster(&cluster, &anchor)?;
                        let mut trial = placed.clone();
                        trial.push(anchor.to_piece());
                        let partner = ctx
                            .options_for(cluster.partner_item, &trial)
                            .into_iter()
                            .filter(|o| o.rotation == target.rotation)
                            .map(|o| ((o.x - target.x).hypot(o.y - target.y), o))
                            .filter(|(d, _)| *d <= SNAP_MM)
                            .min_by(|a, b| a.0.total_cmp(&b.0))
                            .map(|(_, o)| o)?;
                        Some((anchor, partner))
                    })
                };
                if let (Some((anchor, partner)), Some(slot)) = (pair, slot) {
                    if let Some(u) = used.get_mut(i) {
                        *u = true;
                    }
                    if let Some(u) = used.get_mut(slot) {
                        *u = true;
                    }
                    placed.push(anchor.to_piece());
                    placed.push(partner.to_piece());
                    attached += 1;
                    continue;
                }
            }

            // 평범한 배치.
            let Some(option) = best_option(ctx, item_id, &placed) else {
                continue; // 미배치
            };
            if let Some(slot) = used.get_mut(i) {
                *slot = true;
            }
            placed.push(option.to_piece());
        }

        let strip_length = ctx.frontier(&placed);
        (strip_length, placed.len(), attached)
    }

    /// 쌍을 **배치 도중에** 붙이면 실제로 밀도가 오르는가.
    ///
    /// ```text
    /// cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" \
    ///     --lib nfp_strategy::cluster::tests::attaching_pairs_during_placement -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "실측용 — ordering 을 실제로 돌려 분 단위까지 걸린다"]
    fn attaching_pairs_during_placement_is_what_actually_moves_the_density() {
        use crate::nfp_lbf::ordering::build_orderings;
        use rand::SeedableRng;

        let Some(instance) = garment_marker() else {
            return;
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
            table.clone(),
        )
        .with_probe_budget(4096);

        let n_pieces: usize = instance.items.iter().map(|(_, q)| *q).sum();
        let density = |len: f64| item_area / (len * strip_height as f64) * 100.0;

        let mut rng = rand::rngs::Xoshiro256PlusPlus::seed_from_u64(0);
        let orderings = build_orderings(&instance, 12, &mut rng, None);

        // ── 기준선: 쌍을 쓰지 않은 같은 그리디 ──────────────────────────
        let mut ctx = StrategyContext::new(
            &mut placer,
            super::super::NfpStrategyConfig {
                pairwise_clustering: true,
                ..DEFAULT_NFP_STRATEGY_CONFIG
            },
            table.clone(),
            None,
        );
        let mut best_plain = (f64::INFINITY, 0usize);
        for ordering in orderings.iter() {
            let (len, n, _) = greedy_pass(&mut ctx, &ordering.sequence, &[], false);
            if n > best_plain.1 || (n == best_plain.1 && len < best_plain.0) {
                best_plain = (len, n);
            }
        }
        println!(
            "\n쌍 미적용        : 폭 {:9.2} · 밀도 {:5.2}% · 배치 {}/{n_pieces}",
            best_plain.0,
            density(best_plain.0),
            best_plain.1,
        );
        let strip_area = best_plain.0 * strip_height as f64;

        // ── 임계값을 훑는다 — 아주 강한 짝만 남겨도 이기는 구간이 있는가 ──
        for &min_gain in &[0.05f32, 0.15, 0.30, 0.50, 0.80] {
            let mut ctx = StrategyContext::new(
                &mut placer,
                super::super::NfpStrategyConfig {
                    pairwise_clustering: true,
                    cluster_min_gain: min_gain,
                    ..DEFAULT_NFP_STRATEGY_CONFIG
                },
                table.clone(),
                None,
            );
            let (clusters, diag) =
                build_clusters_measured(&mut ctx, &instance, None, MAX_CLUSTER_ITEM_KINDS);

            // 면적과 **폭**을 나란히 찍는다. 목적함수가 strip 길이이므로 클러스터링의
            // 진짜 상한은 폭 쪽이다 — 면적만 보면 값어치를 과대평가한다.
            println!(
                "\nmin_gain {min_gain:.2} — 확정 쌍 {:>2} · 이론상 아끼는 면적 {:.0}mm² \
                 = strip 의 {:.2}% · 아끼는 폭 {:.0}mm = strip 길이의 {:.2}%",
                clusters.len(),
                diag.matched_saved_area,
                diag.matched_saved_area / strip_area * 100.0,
                diag.matched_saved_width,
                diag.matched_saved_width / best_plain.0 * 100.0,
            );
            // 붙이는 방식 둘을 같은 쌍 집합으로 비교한다.
            //   사후부착: 앵커를 자기만 보고 놓고 그 자리에서 파트너를 붙인다(현 파이프라인).
            //   쌍우선  : 쌍 전체의 strip 길이가 짧은 앵커 자리부터 시험한다(super-piece).
            for (mode, pair_aware) in [("사후부착", false), ("쌍우선  ", true)] {
                let mut best = (f64::INFINITY, 0usize, 0usize);
                for ordering in orderings.iter() {
                    let (len, n, attached) =
                        greedy_pass(&mut ctx, &ordering.sequence, &clusters, pair_aware);
                    if n > best.1 || (n == best.1 && len < best.0) {
                        best = (len, n, attached);
                    }
                }
                println!(
                    "  {mode} : 폭 {:9.2} · 밀도 {:5.2}% ({:+.2}%p) · 배치 {}/{n_pieces} \
                     · 붙인 쌍 {:>2}",
                    best.0,
                    density(best.0),
                    density(best.0) - density(best_plain.0),
                    best.1,
                    best.2,
                );
            }
        }
    }

    // ── 예산 ────────────────────────────────────────────────────────────

    #[test]
    fn the_earlier_of_two_deadlines_wins() {
        let now = Instant::now();
        let soon = now + std::time::Duration::from_millis(10);
        let later = now + std::time::Duration::from_millis(500);
        assert_eq!(earliest(Some(soon), Some(later)), Some(soon));
        assert_eq!(earliest(Some(later), Some(soon)), Some(soon));
        assert_eq!(earliest(Some(soon), None), Some(soon));
        assert_eq!(earliest(None, None), None);
        // 예산이 없으면 절대 만료되지 않는다.
        assert!(!expired(None));
    }
}
