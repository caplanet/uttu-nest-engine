//! NFP 기반 초기 배치 파이프라인.
//!
//! ```text
//! NFP Cache
//!    ↓
//! Multiple Item Ordering
//!    ↓
//! NFP 기반 Sequential Placement
//!    ↓
//! Strip Length 최소 후보 우선
//!    ↓
//! LB + Contact + Void 평가
//!    ↓
//! Best Initial Nest
//!    ↓
//! Swarm / Local Improvement
//! ```
//!
//! # 설계 원칙
//!
//! * **캐시 Key는 Geometry Hash로 잡는다.** 패턴 instance 번호를 키에 넣으면
//!   동일한 형상을 계속 다시 계산하게 된다.
//!
//!   ```text
//!   잘못된 방식              올바른 방식
//!   Pattern_001 × Pattern_020   GeometryHash(A) × GeometryHash(B)
//!   Pattern_001 × Pattern_021
//!   ```
//!
//! * **Cache Miss는 즉시 계산하고 저장한다.** 동시에 동일 실행 내 병렬 pre-warm으로
//!   나머지 고확률 pair를 채운다.
//!
//!   ```text
//!   Nesting 시작 → 필요한 NFP 요청 → Cache Hit ─→ 즉시 사용
//!                                     │
//!                                   Cache Miss → NFP 계산 → Cache 저장
//!   ```
//!
//! * **캐시 저장을 연산 중 mutex로 매번 잠그지 않는다.** thread-local 결과를 모아서
//!   마지막에 한 번 합친다.
//!
//! * **매 ordering마다 156개를 처음부터 전부 재배치하지 않는다.** 순서가 `k`번째부터
//!   달라졌다면 앞의 `k-1`개 배치를 그대로 재사용하고 suffix만 다시 배치한다.
//!
//! 기하 연산은 전부 [`swarm_nfp`] 커널(Clipper2 i64 정수 격자)에 위임한다.

pub mod cache;
/// 순수 NFP 순서 탐색 단계 (`NfpIntegrationConfig::ordering_exploration`).
pub mod exploration;
pub mod ordering;
pub mod placement;
pub mod score;
pub mod search;

pub use exploration::nfp_ordering_exploration_phase;
pub use cache::{
    GeometryHash, MaskCell, NfpCacheStats, NfpKey, NfpLbfCache, NfpMask, PreparedLibrary,
};
pub use ordering::{
    DEFAULT_DIAM_FACTOR, DEFAULT_HARD_PERCENT, DEFAULT_MEDIUM_PERCENT, DifficultyTier,
    DifficultyTiers, ItemMetrics, ItemOrdering, OrderingStrategy, build_orderings,
    collect_metrics, difficulty_scores, difficulty_tiered_sequence,
};
pub use placement::{
    CandidateSource, DEFAULT_MAX_CANDIDATE_PROBES, LEFT_REGION_PROBE_DIVISOR,
    MAX_EVALUATED_CANDIDATES, PlacedPiece, PlacementCandidate, SequentialPlacer,
    SequentialPlacement,
};
pub use score::{CandidateScore, PlacementScoreWeights, ScoreTerms};
pub use search::{BestInitialNest, NfpLbfConfig, NfpLbfReport, build_best_initial_nest};

/// `Score(p) = w1·L(p) + w2·Waste(p) − w3·Contact(p) + w4·Y(p)`
///
/// * `L(p)` = 배치 후 strip length
/// * `Waste(p)` = 새로 발생하는 unusable void
/// * `Contact(p)` = 기존 패턴/strip boundary와 맞닿는 길이
/// * `Y(p)` = 세로 위치
///
/// 부호에 주의한다. `Contact`만 **감점(-)**이며, 나머지 세 항은 가점(+)이다.
/// 즉 같은 strip length라면 주변 형상에 더 밀착되는 위치가 선택된다.
pub const DEFAULT_SCORE_W1_STRIP_LENGTH: f32 = 1.0;
/// [`DEFAULT_SCORE_W1_STRIP_LENGTH`] 참조.
pub const DEFAULT_SCORE_W2_WASTE: f32 = 0.35;
/// [`DEFAULT_SCORE_W1_STRIP_LENGTH`] 참조. 계산 시 부호는 `-`로 적용된다.
pub const DEFAULT_SCORE_W3_CONTACT: f32 = 0.25;
/// [`DEFAULT_SCORE_W1_STRIP_LENGTH`] 참조.
pub const DEFAULT_SCORE_W4_Y: f32 = 0.10;

/// 후보 평가 우선순위.
///
/// ```text
/// 1순위 : 배치 후 Strip Length 최소
/// 2순위 : x 최소
/// 3순위 : y 최소
/// 4순위 : 주변 패턴과의 접촉도 최대
/// ```
///
/// 순수 LB(`min(x, y)`)가 아니라 **Strip Length 우선 + Bottom/Left 보조**다.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CandidatePriority {
    /// 1순위 — 배치 후 Strip Length 최소
    StripLength,
    /// 2순위 — x 최소
    MinX,
    /// 3순위 — y 최소
    MinY,
    /// 4순위 — 주변 패턴과의 접촉도 최대
    MaxContact,
}

/// 우선순위를 문서에 적힌 순서 그대로 나열한 표.
pub const CANDIDATE_PRIORITY_ORDER: [CandidatePriority; 4] = [
    CandidatePriority::StripLength,
    CandidatePriority::MinX,
    CandidatePriority::MinY,
    CandidatePriority::MaxContact,
];

/// 두 strip length가 같다고 볼 허용 오차(mm). 1순위 비교에서 동률을 판정한 뒤
/// 2·3·4순위로 내려간다.
pub const STRIP_LENGTH_TIE_EPS: f32 = 1.0e-4;

/// 좌표 동률 판정 허용 오차(mm).
pub const COORD_TIE_EPS: f32 = 1.0e-4;

/// 문서가 정한 기본 목표치.
///
/// ```text
/// NFP cold cache          2.0 sec
/// LBF ordering 1회       100~200 ms
/// 20 orderings            2~4 sec
/// 50 orderings            5~10 sec
/// 100 orderings          10~20 sec
/// ```
///
/// > **Cold start 10초 이내에 NFP 캐시 + 30~50개 LBF ordering 평가 완료**
pub const DEFAULT_N_ORDERINGS: usize = 50;

/// NFP 라이브러리 전용 **오목(concave) 단순화** 기본 강도 (원본 면적 대비 허용 증가 비율).
///
/// 볼록 분해 조각 수는 `오목 정점 수 + 1`을 따라가고, pair NFP의 원시 조각 수는
/// `조각(A) x 조각(B)`다. 즉 NFP 비용을 줄이는 레버는 오목 정점뿐이며 볼록 정점은
/// 기여하지 않는다. 전역 `poly_simpl_tolerance`를 올리면 볼록까지 함께 뭉개지므로,
/// 여기서 오목만 따로 더 줄인다.
///
/// 오목 정점을 없애면 도형이 커져 원본의 상위집합이 되고, NFP는 실제 금지영역을
/// 과대평가한다 - 배치 가능 판정이 CDE 기준으로도 항상 안전하다.
///
/// `0.0`이면 비활성(= 솔버의 `poly_simpl_tolerance` 결과를 그대로 사용).
///
/// # 기본값이 왜 `0.02` 인가
///
/// 오래도록 기본값이 `0.0`(비활성)이었다. 그런데 이 저장소의 **모든 실측표가 예외 없이
/// `0.02` 조건**에서 잰 것이었다 — 즉 출하되는 regime 에서 잰 수치가 하나도 없었다.
/// 그것을 재 봤다 (wsjk156, 탐침 8192, `SPSolution::density`, 2026-08-31):
///
/// ```text
///                        0.0 (옛 기본값)      0.02 (실측 regime)
///   NFP 라이브러리 prewarm    4,055 ms            593 ms      <- 6.8배
///   ③ 접촉 최대화            시드 생성 실패        72.00%
///   ③ 기준선(전략 전부 끔)     71.46%             71.75%
///   ⑧ 누적 배치              73.08%             72.93%
/// ```
///
/// 밀도는 사실상 무승부인데(±0.15%p, 방향도 엇갈린다) **준비 시간이 7배** 차이 나고,
/// 심지어 전략 하나는 예산 안에 시드를 만들지 못했다. 기본값이 아무도 재지 않은
/// 나쁜 regime 이었던 것이다.
///
/// 안전성은 방향으로 보장된다 — 오목 정점을 없애면 도형이 **커져** 원본의 상위집합이
/// 되고, 그 NFP 는 실제 금지영역을 과대평가한다. 그래서 "배치 가능" 판정은 jagua-rs CDE
/// 기준으로도 항상 안전하다. 반대 방향(볼록 정점을 깎아 도형을 줄이는 것)은 하지 않는다.
///
/// 강도를 더 올리면 손해로 돌아선다 — `0.05` 에서 albano 밀도가 72.06% -> 71.11% 로
/// 떨어진다(문서 §6-4).
pub const DEFAULT_CONCAVE_SIMPLIFY_TOLERANCE: f32 = 0.02;

/// NFP 콜드 캐시 생성의 기본 예산(ms). 문서 목표 `< 2 sec`, 최악 `< 5 sec`.
pub const DEFAULT_CACHE_BUDGET_MS: u64 = 5_000;

/// 전체 초기 배치 탐색의 기본 예산(ms). 문서 목표 `Cold start 10초 이내`.
pub const DEFAULT_TOTAL_BUDGET_MS: u64 = 10_000;
