//! NFP 초기 배치 전략 선택 설정.
//!
//! 아홉 전략은 **서로 독립적으로** 켜고 끈다. 전부 끄면 기존 NFP-LBF 경로가
//! 한 비트도 바뀌지 않는다.
//!
//! ```text
//! ① instance_rotations  회전 집합을 인스턴스 선언에서 그대로 뽑는다
//! ② pairwise_clustering 잘 맞물리는 조각 쌍을 하나로 묶어 배치한다
//! ③ contact_maximization 위치를 "가장 왼쪽"이 아니라 "접촉 최대"로 고른다
//! ④ pocket_first        프론티어를 늘리기 전에 내부 구멍부터 채운다
//! ⑤ beam_search         부분 배치를 1개가 아니라 B개 유지한다
//! ⑥ best_fit_selection  순서를 고정하지 않고 매 단계 최선의 조각을 고른다
//! ⑦ sliding_compaction  배치 후 각 조각을 막힐 때까지 좌하단으로 민다
//! ⑧ accretion           순서를 정하지 않고 매번 가장 잘 맞물리는 조각을 고른다
//! ⑨ tail_reflow         꼬리의 조각을 빼서 앞쪽 빈 곳으로 되돌린다
//! ```
//!
//! # 조합
//!
//! 전략은 파이프라인의 서로 다른 지점에 붙으므로 대부분 함께 켤 수 있다.
//!
//! ```text
//! ① 회전 집합 결정
//!      v
//! ② 클러스터링 (전처리)
//!      v
//! ⑥ 조각 선택  <-- ⑤ 빔 서치가 감싼다
//!      v
//! ④ 후보 필터  ->  ③ 후보 점수
//!      v
//! ⑦ 슬라이딩 압축 (후처리)
//! ```
//!
//! 예외는 ⑤와 ⑥이다. 둘 다 "다음에 무엇을 어디에" 를 정하므로, 함께 켜면
//! 빔의 각 노드에서 best-fit 으로 조각을 고른다 — 비용이 `B x 남은 조각 수`로
//! 곱해진다. 허용하되 [`NfpStrategyConfig::estimated_cost_factor`]가 그 배수를
//! 알려 준다.

/// 일곱 전략의 on/off 와 각 전략의 조절 노브.
///
/// [`crate::config::SwarmConfig`]가 `Copy`라 이 구조체도 `Copy`다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NfpStrategyConfig {
    // ── ① 인스턴스 회전 ────────────────────────────────────────────────
    /// 회전 후보를 `NfpLbfSeedConfig::allow_180` 플래그가 아니라 **인스턴스가 선언한
    /// `allowed_rotation`** 에서 그대로 뽑는다.
    ///
    /// 기존 LBF 생성기는 이미 그렇게 한다. NFP-LBF 만 0도(또는 0/180도)로 묶여 있어
    /// 회전이 허용된 인스턴스에서 구조적으로 불리했다.
    ///
    /// 실측(albano, `allowed_orientations = [0, 180]`): 끄면 LBF 대비 −1.51%p 로 **지고**,
    /// 켜면 +8.57%p 로 이긴다. 회전 선언이 `[0]` 뿐인 인스턴스(여성WSjk)에서는 영향이 없다.
    pub instance_rotations: bool,

    /// `RotationRange::Continuous` 를 몇 도 간격으로 샘플링할지.
    ///
    /// 연속 회전은 후보가 무한하므로 유한 집합으로 이산화해야 한다. 이 값이 작을수록
    /// 촘촘하지만 NFP 라이브러리가 `360 / step` 배로 커진다 — pair NFP 수는 그 **제곱**에
    /// 비례하므로 90도(4개) 아래로 내릴 때는 캐시 예산을 함께 늘려야 한다.
    ///
    /// `0` 이하면 [`DEFAULT_CONTINUOUS_ROTATION_STEP_DEG`].
    pub continuous_rotation_step_deg: i64,

    // ── ② 쌍별 클러스터링 ──────────────────────────────────────────────
    /// 서로 잘 맞물리는 조각 **쌍**을 찾아 하나의 super-piece 로 묶어 배치한다.
    ///
    /// 의류 마커에서 좌/우 대칭 패널과 소매는 짝을 이룰 때 낭비가 급감한다. 순차 배치는
    /// 조각을 하나씩 보므로 그 짝을 스스로 찾지 못한다.
    ///
    /// 쌍의 상대 위치는 NFP 경계 위에서 고르므로 **정의상 겹치지 않는다**.
    pub pairwise_clustering: bool,

    /// 쌍을 채택할 최소 이득. `1 - (면적합 / 묶음 bbox 면적)` 이 이 값보다 좋아야 한다.
    ///
    /// 너무 낮게 잡으면 이득 없는 쌍까지 묶여 배치 자유도만 잃는다.
    /// `0` 이하면 [`DEFAULT_CLUSTER_MIN_GAIN`].
    pub cluster_min_gain: f32,

    // ── ③ 접촉 최대화 ──────────────────────────────────────────────────
    /// 후보 위치를 "strip 길이 → 가장 왼쪽 → 가장 아래" 가 아니라
    /// **"strip 길이 → 접촉 최대 → 왼쪽 → 아래"** 로 고른다 (Gomes & Oliveira).
    ///
    /// 기존 우선순위는 사실상 `argmin(x)` 로 붕괴한다: 한 조각·한 회전에서 폭 `W`는
    /// 상수라 `strip_length = max(frontier, x + W)` 가 `x` 의 단조 증가 함수이고,
    /// 따라서 1순위가 2순위를 절대 뒤집지 못한다. 그 결과 기존 LBF 와 같은 규칙이 된다.
    ///
    /// 실측(여성WSjk, 탐침 8192): 72.08% -> 72.48%. 탐침 예산이 낮으면 이득이 없거나
    /// 오히려 손해다 — 접촉 위치를 충분히 열거해야 의미가 생긴다.
    pub contact_maximization: bool,

    // ── ④ 포켓 우선 ────────────────────────────────────────────────────
    /// 프론티어를 늘리기 전에 **이미 배치된 영역 안쪽의 구멍**부터 채운다.
    ///
    /// 순차 배치는 오른쪽으로 자라면서 뒤에 쓸 수 없는 공백을 남긴다. 탐침 예산은
    /// 프론티어 근처에 몰려 있어(그쪽이 feasible 하므로) 안쪽 구멍은 예산이 닿기 전에
    /// 끝난다. 이 전략은 순서를 뒤집어, 내부 후보를 **먼저 전량** 훑고 거기서 자리를
    /// 찾지 못했을 때만 프론티어로 나간다.
    pub pocket_first: bool,

    // ── ⑤ 빔 서치 ──────────────────────────────────────────────────────
    /// 각 깊이에서 부분 배치를 1개가 아니라 `beam_width` 개 유지한다.
    ///
    /// 순차 배치는 되돌아갈 수 없어서, 초반 한 번의 나쁜 선택이 끝까지 간다.
    /// 빔은 그 선택을 유예한다.
    pub beam_search: bool,

    /// 유지할 부분 배치 수. `0` 이하면 [`DEFAULT_BEAM_WIDTH`].
    ///
    /// 비용과 시간이 이 값에 정비례한다.
    pub beam_width: usize,

    // ── ⑥ best-fit 동적 선택 ───────────────────────────────────────────
    /// 배치 순서를 미리 정하지 않고, **매 단계 남은 조각 전체를 평가해** 가장 잘 맞는
    /// 조각을 고른다 (first-fit 이 아니라 best-fit).
    ///
    /// 켜면 ordering 목록은 "평가 시작점" 으로만 쓰인다 — 50개 ordering 을 도는 대신
    /// 적응형 1패스가 된다.
    pub best_fit_selection: bool,

    /// 매 단계 평가할 남은 조각 종류 수 상한. `0` 이하면 [`DEFAULT_BEST_FIT_LOOKAHEAD`].
    ///
    /// 전수 평가는 `O(남은 조각 x 후보)` 라 조각이 많으면 감당이 안 된다.
    /// 면적 내림차순 상위 이만큼만 본다.
    pub best_fit_lookahead: usize,

    // ── ⑦ 슬라이딩 압축 ────────────────────────────────────────────────
    /// 배치가 끝난 뒤 각 조각을 **막힐 때까지 좌하단으로 민다**.
    ///
    /// 그리디 배치는 나중 조각이 앞 조각을 밀어낼 수 없어 틈을 남긴다. 이 패스는
    /// 조각을 하나씩 빼서 그 자리의 feasible 영역을 다시 구하고, 원래 위치보다
    /// 좌하단인 자리가 있으면 옮긴다. NFP 기반이라 겹침이 생길 수 없다.
    ///
    /// LP 압축([`crate::config::LpCompactionConfig`])과 목적이 겹치지만 성격이 다르다:
    /// LP 는 모든 조각을 **동시에** 조금씩 옮기고, 이쪽은 한 조각을 **크게** 옮긴다.
    pub sliding_compaction: bool,

    /// 슬라이딩을 몇 라운드 돌릴지. `0` 이하면 [`DEFAULT_SLIDING_ROUNDS`].
    ///
    /// 한 라운드는 모든 조각을 한 번씩 훑는다. 개선이 없으면 남은 라운드는 건너뛴다.
    pub sliding_rounds: usize,

    // ── ⑧ 누적 배치 ────────────────────────────────────────────────────
    /// 배치 **순서를 아예 정하지 않고**, 매 단계 남은 조각 전체 x 가능한 자리 전체에서
    /// 가장 잘 맞물리는 (조각, 자리) 한 쌍을 고른다.
    ///
    /// ⑥ best-fit 과 다르다. ⑥은 ordering 을 평가 시작점으로 남겨 두고 조각만 고르지만,
    /// ⑧은 ordering 자체를 쓰지 않는다 — `sequence` 는 **남은 조각의 다중집합**으로만
    /// 쓰인다. 그래서 ⑧이 켜지면 ordering 을 여러 개 도는 것이 무의미하다
    /// (같은 다중집합이면 같은 결과다). 호출부가 1개만 평가한다.
    ///
    /// 실측(wsjk156, 탐침 8192, jagua 밀도): 기준 71.75% -> **72.93%**, 2.5초.
    /// ⑦ 슬라이딩 2회를 얹으면 **73.09%**. 26초 걸리던 ②+③+④+⑦(72.18%)보다
    /// 좋으면서 훨씬 싸다.
    ///
    /// 병렬 결정론이 통과 조건이다 — `tests/nfp_parallel_determinism.rs` 참조.
    pub accretion: bool,

    // ── ⑨ 꼬리 회수 ────────────────────────────────────────────────────
    /// 오른쪽 끝(꼬리)의 조각을 빼서 앞쪽에 남은 빈 곳으로 되돌린다.
    ///
    /// 그리디 배치는 앞을 조이고 뒤를 굶긴다. 경쟁사 마커와 200mm 구간별 충전율을
    /// 대조하면 그 편중이 그대로 보인다 (2026-08-31, wsjk156):
    ///
    /// ```text
    ///            앞 1/3   중 1/3   뒤 1/3   최저구간
    /// 우리        76.9     77.5     67.3      57
    /// 경쟁사      78.6     82.9     82.7      73
    /// ```
    ///
    /// 앞부분은 거의 대등한데 뒤에서 −15.4%p 벌어진다. ⑨는 정확히 그 자리를 친다.
    ///
    /// 실측(wsjk156, 5초 예산): 71.75% -> **73.27%** (+1.52%p).
    /// ⑦ 슬라이딩과 성격이 다르다 — ⑦은 조각을 **제자리 근처**로 밀고,
    /// ⑨는 조각을 **멀리 옮긴다**(꼬리에서 앞쪽 포켓으로).
    pub tail_reflow: bool,

    /// 꼬리 회수를 몇 라운드 돌릴지. `0` 이하면 [`DEFAULT_REFLOW_ROUNDS`].
    pub reflow_rounds: usize,
}

/// `RotationRange::Continuous` 기본 샘플 간격(도).
///
/// 90도는 pair NFP 를 16배로 늘린다(4 x 4). 그 이상 촘촘하게 하려면 캐시 예산을
/// 함께 올려야 한다.
pub const DEFAULT_CONTINUOUS_ROTATION_STEP_DEG: i64 = 90;

/// 쌍 채택 기본 임계. 묶음 bbox 대비 면적 충전율이 이만큼은 올라야 한다.
pub const DEFAULT_CLUSTER_MIN_GAIN: f32 = 0.05;

/// 기본 빔 폭.
///
/// **2다.** 직관은 "넓을수록 낫다" 지만 실측은 반대다 (wsjk156, 탐침 4096, 12 ordering,
/// 전략 기준선 71.75%):
///
/// ```text
/// 폭 2   72.10%    9.6 s   <- 기본값
/// 폭 3   71.82%   15.0 s
/// 폭 4   71.91%   18.5 s
/// 폭 8   71.80%   36.3 s
/// ```
///
/// 폭을 넓히면 노드는 늘지만 **각 노드가 뻗는 가지 수**([`beam::branch_for`])는 그만큼
/// 늘지 않는다. 그래서 넓은 빔은 서로 비슷한 상태로 채워지고, 다양성 대신 비용만 는다.
/// 2가 품질도 시간도 가장 좋다.
///
/// > **두 표의 시간이 다르다.** [`beam`] 모듈 문서의 표는 같은 조건이라고 적으면서
/// > 폭 2를 11.5초, 폭 4를 22.8초로 적는다(여기는 9.6초 / 18.5초). 최대 23% 차이다.
/// > 어느 리비전에서 잰 것인지 양쪽 다 표기가 없어 지금은 조정할 수 없다.
/// > **밀도(72.10% / 71.91%)는 두 표가 일치하므로 결론은 같다** — 시간만 신뢰하지 말 것.
/// > 다시 잴 때는 리비전과 머신을 함께 적을 것.
///
/// [`beam`]: crate::nfp_strategy::beam
///
/// [`beam::branch_for`]: crate::nfp_strategy::beam
pub const DEFAULT_BEAM_WIDTH: usize = 2;

/// best-fit 이 매 단계 평가할 조각 종류 수 기본 상한.
pub const DEFAULT_BEST_FIT_LOOKAHEAD: usize = 12;

/// 기본 슬라이딩 라운드 수.
pub const DEFAULT_SLIDING_ROUNDS: usize = 2;

/// 기본 꼬리 회수 라운드 수.
///
/// **100이다.** 처음에 ⑦ 슬라이딩과 같은 2로 두었는데, 그건 서로 다른 두 루프를
/// 같은 이름이라는 이유로 같은 값으로 맞춘 것이었다(함정 4: 근거 없는 상수).
///
/// ⑨의 라운드 루프는 **스스로 멈춘다** — 뺄 개수 일정(`removal_schedule`, 4→8→16→32→64)과
/// 띠 폭 주기(`BAND_WIDTH_CYCLE`, 4종)를 다 훑고도 못 줄이면 `break` 한다. 성공하면
/// 일정이 처음으로 되돌아가므로, 라운드 수는 "몇 번 시도할까" 가 아니라 **"자기종료를
/// 허용할 상한"** 이다. 값을 낮게 잡으면 개선이 남았는데 잘라 버린다.
///
/// 실측 (wsjk156, `SPSolution::density`, 2026-08-31):
///
/// ```text
/// rounds     ⑨ 단독(ord1, 탐침4096)      ⑧+⑦+⑨(탐침8192)
///     2      72.06%  폭 7059.2   0.9s     73.09%  폭 6959.9   3.7s
///     8      72.27%  폭 7038.8   1.3s
///    30      72.62%  폭 7004.6   2.2s
///   100      73.53%  폭 6918.3   5.6s     73.72%  폭 6900.4   8.7s   <- 기본값
///   200      73.53%  폭 6918.3  11.8s     73.84%  폭 6888.8  16.1s
///   400      73.53%  폭 6918.3  14.9s     73.92%  폭 6881.7  26.0s
/// ```
///
/// ⑨ 단독은 **100에서 자기종료**한다(200·400이 비트 단위로 같다). ⑧과 함께 쓰면 더
/// 올라가지만 시간이 정비례로 늘어 이득이 급격히 나빠진다. 실제 시간 제어는 이 상한이
/// 아니라 `StrategyContext::deadline` 이 한다 — 예산이 없으면 라운드 루프가 즉시 빠진다.
///
/// > 이 표는 `reflow` 모듈 문서의 스윕표와 함께 읽을 것. 그쪽은 라운드를 축으로 잡지
/// > 않았고(벽시계 5초로 돌렸다), 그래서 문서의 73.27% 를 라운드 2로는 재현할 수 없다.
pub const DEFAULT_REFLOW_ROUNDS: usize = 100;

/// 전부 꺼짐. 기존 NFP-LBF 경로가 그대로 돈다.
pub const DEFAULT_NFP_STRATEGY_CONFIG: NfpStrategyConfig = NfpStrategyConfig {
    instance_rotations: false,
    continuous_rotation_step_deg: DEFAULT_CONTINUOUS_ROTATION_STEP_DEG,
    pairwise_clustering: false,
    cluster_min_gain: DEFAULT_CLUSTER_MIN_GAIN,
    contact_maximization: false,
    pocket_first: false,
    beam_search: false,
    beam_width: DEFAULT_BEAM_WIDTH,
    best_fit_selection: false,
    best_fit_lookahead: DEFAULT_BEST_FIT_LOOKAHEAD,
    sliding_compaction: false,
    sliding_rounds: DEFAULT_SLIDING_ROUNDS,
    accretion: false,
    tail_reflow: false,
    reflow_rounds: DEFAULT_REFLOW_ROUNDS,
};

impl Default for NfpStrategyConfig {
    fn default() -> Self {
        DEFAULT_NFP_STRATEGY_CONFIG
    }
}

/// The placement branches are mutually exclusive; postpasses are independent.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum PlacementStrategy { Accretion, BestFit, Beam, Greedy }

impl NfpStrategyConfig {
    pub(crate) fn placement_strategy(&self) -> PlacementStrategy {
        if self.accretion { PlacementStrategy::Accretion }
        else if self.best_fit_selection { PlacementStrategy::BestFit }
        else if self.beam_search { PlacementStrategy::Beam }
        else { PlacementStrategy::Greedy }
    }
    /// 하나라도 켜져 있는가. 전부 꺼져 있으면 호출부가 기존 경로로 빠진다.
    pub fn any_enabled(&self) -> bool {
        self.instance_rotations
            || self.pairwise_clustering
            || self.contact_maximization
            || self.pocket_first
            || self.beam_search
            || self.best_fit_selection
            || self.sliding_compaction
            || self.accretion
            || self.tail_reflow
    }

    /// `0` 이하 필드를 기본값으로 올린 사본. 경계(C ABI)에서 넘어온 값을 정리한다.
    pub fn normalized(mut self) -> Self {
        if self.continuous_rotation_step_deg <= 0 || self.continuous_rotation_step_deg > 180 {
            self.continuous_rotation_step_deg = DEFAULT_CONTINUOUS_ROTATION_STEP_DEG;
        }
        if !(self.cluster_min_gain.is_finite() && self.cluster_min_gain > 0.0) {
            self.cluster_min_gain = DEFAULT_CLUSTER_MIN_GAIN;
        }
        if self.beam_width == 0 {
            self.beam_width = DEFAULT_BEAM_WIDTH;
        }
        if self.best_fit_lookahead == 0 {
            self.best_fit_lookahead = DEFAULT_BEST_FIT_LOOKAHEAD;
        }
        if self.sliding_rounds == 0 {
            self.sliding_rounds = DEFAULT_SLIDING_ROUNDS;
        }
        if self.reflow_rounds == 0 {
            self.reflow_rounds = DEFAULT_REFLOW_ROUNDS;
        }
        self
    }

    /// 실제 선택된 배치 전략과 후처리가 1회 비용을 대략 몇 배로 만드는지.
    ///
    /// 정확한 예측이 아니라 **예산 배분용 눈금**이다. 호출부가 이 값으로
    /// ordering 수를 줄여 전체 시간을 맞춘다.
    ///
    /// ```text
    /// ⑤ 빔 서치        x beam_width
    /// ⑥ best-fit       x best_fit_lookahead
    /// ④ 포켓 우선      x 1.5  (내부 후보를 추가로 훑는다)
    /// ⑦ 슬라이딩       x (1 + 0.5 x rounds)
    /// ①②③            비용 영향 미미 (전처리이거나 비교 규칙만 바뀐다)
    /// ```
    pub fn estimated_cost_factor(&self) -> f32 {
        let cfg = self.normalized();
        let strategy = cfg.placement_strategy();
        let mut factor = match strategy {
            PlacementStrategy::Accretion | PlacementStrategy::Greedy => 1.0,
            PlacementStrategy::BestFit => cfg.best_fit_lookahead as f32,
            PlacementStrategy::Beam => cfg.beam_width as f32,
        };
        if cfg.pocket_first && strategy != PlacementStrategy::Accretion {
            factor *= 1.5;
        }
        if cfg.sliding_compaction {
            factor *= 1.0 + 0.5 * cfg.sliding_rounds as f32;
        }
        if cfg.tail_reflow {
            factor *= 1.0 + 0.5 * cfg.reflow_rounds as f32;
        }
        // ⑧은 배치 1회를 비싸게 만들지 않는다. 대신 **ordering 을 1개만** 평가하므로
        // 전체 예산은 오히려 준다. 그 절약은 호출부가 ordering 수를 줄여 반영한다.
        factor
    }

    /// 켜진 전략을 사람이 읽을 라벨로. 로그와 리포트에 그대로 실린다.
    pub fn enabled_labels(&self) -> Vec<&'static str> {
        let mut out = Vec::new();
        if self.instance_rotations {
            out.push("①인스턴스회전");
        }
        if self.pairwise_clustering {
            out.push("②쌍별클러스터링");
        }
        if self.contact_maximization {
            out.push("③접촉최대화");
        }
        if self.pocket_first {
            out.push("④포켓우선");
        }
        if self.beam_search {
            out.push("⑤빔서치");
        }
        if self.best_fit_selection {
            out.push("⑥best-fit선택");
        }
        if self.sliding_compaction {
            out.push("⑦슬라이딩압축");
        }
        if self.accretion {
            out.push("⑧누적배치");
        }
        if self.tail_reflow {
            out.push("⑨꼬리회수");
        }
        out
    }

    /// ⑧이 켜졌는가. 켜지면 ordering 목록은 **다중집합**으로만 쓰이므로 호출부가
    /// 여러 개를 도는 것이 낭비다.
    ///
    /// 별도 메서드로 둔 이유: 호출부가 `cfg.accretion` 을 직접 읽으면 "왜 1개만 도는가"
    /// 라는 근거가 호출부에 흩어진다. 여기 한 곳에 남긴다.
    pub fn ordering_is_only_a_multiset(&self) -> bool {
        self.accretion
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_all_off_so_the_existing_path_is_untouched() {
        let cfg = NfpStrategyConfig::default();
        assert!(!cfg.any_enabled());
        assert!(cfg.enabled_labels().is_empty());
        // 전부 꺼져 있으면 비용도 그대로다.
        assert_eq!(cfg.estimated_cost_factor(), 1.0);
    }

    #[test]
    fn zero_knobs_fall_back_to_defaults() {
        let raw = NfpStrategyConfig {
            continuous_rotation_step_deg: 0,
            cluster_min_gain: -1.0,
            beam_width: 0,
            best_fit_lookahead: 0,
            sliding_rounds: 0,
            ..DEFAULT_NFP_STRATEGY_CONFIG
        };
        let cfg = raw.normalized();
        assert_eq!(
            cfg.continuous_rotation_step_deg,
            DEFAULT_CONTINUOUS_ROTATION_STEP_DEG
        );
        assert_eq!(cfg.cluster_min_gain, DEFAULT_CLUSTER_MIN_GAIN);
        assert_eq!(cfg.beam_width, DEFAULT_BEAM_WIDTH);
        assert_eq!(cfg.best_fit_lookahead, DEFAULT_BEST_FIT_LOOKAHEAD);
        assert_eq!(cfg.sliding_rounds, DEFAULT_SLIDING_ROUNDS);

        // 180도를 넘는 간격은 무의미하다 (NFP 는 반바퀴면 대칭이 아니므로 360 이 상한이지만,
        // 180 을 넘기면 샘플이 1개뿐이라 회전을 켠 의미가 없다).
        let coarse = NfpStrategyConfig {
            continuous_rotation_step_deg: 200,
            ..DEFAULT_NFP_STRATEGY_CONFIG
        }
        .normalized();
        assert_eq!(
            coarse.continuous_rotation_step_deg,
            DEFAULT_CONTINUOUS_ROTATION_STEP_DEG
        );
    }

    #[test]
    fn cost_factor_uses_the_executed_strategy_and_active_postprocessing() {
        let beam = NfpStrategyConfig {
            beam_search: true,
            beam_width: 4,
            ..DEFAULT_NFP_STRATEGY_CONFIG
        };
        assert_eq!(beam.estimated_cost_factor(), 4.0);

        let both = NfpStrategyConfig {
            beam_search: true,
            beam_width: 4,
            best_fit_selection: true,
            best_fit_lookahead: 8,
            ..DEFAULT_NFP_STRATEGY_CONFIG
        };
        // Best-fit is the selected branch; Beam does not execute inside it.
        assert_eq!(both.estimated_cost_factor(), 8.0);

        let accretion = NfpStrategyConfig {
            accretion: true,
            pocket_first: true,
            sliding_compaction: true,
            sliding_rounds: 2,
            tail_reflow: true,
            reflow_rounds: 1,
            ..both
        };
        // Accretion ignores best-fit, Beam and pocket-first; both postpasses run.
        assert_eq!(accretion.estimated_cost_factor(), 3.0);

        // 비교 규칙만 바꾸는 전략은 비용에 영향이 없다.
        let contact = NfpStrategyConfig {
            contact_maximization: true,
            instance_rotations: true,
            pairwise_clustering: true,
            ..DEFAULT_NFP_STRATEGY_CONFIG
        };
        assert_eq!(contact.estimated_cost_factor(), 1.0);
    }

    #[test]
    fn placement_precedence_covers_every_selection_combination() {
        for flags in 0..8 {
            let cfg = NfpStrategyConfig { accretion:flags & 1 != 0,
                best_fit_selection:flags & 2 != 0, beam_search:flags & 4 != 0,
                best_fit_lookahead:8, beam_width:4, ..DEFAULT_NFP_STRATEGY_CONFIG };
            let expected = if flags & 1 != 0 { (PlacementStrategy::Accretion,1.0) }
                else if flags & 2 != 0 { (PlacementStrategy::BestFit,8.0) }
                else if flags & 4 != 0 { (PlacementStrategy::Beam,4.0) }
                else { (PlacementStrategy::Greedy,1.0) };
            assert_eq!(cfg.placement_strategy(),expected.0);
            assert_eq!(cfg.estimated_cost_factor(),expected.1);
        }
    }

    #[test]
    fn labels_list_every_enabled_strategy_in_order() {
        let all = NfpStrategyConfig {
            instance_rotations: true,
            pairwise_clustering: true,
            contact_maximization: true,
            pocket_first: true,
            beam_search: true,
            best_fit_selection: true,
            sliding_compaction: true,
            ..DEFAULT_NFP_STRATEGY_CONFIG
        };
        assert_eq!(all.enabled_labels().len(), 7);
        assert!(all.any_enabled());
    }
}
