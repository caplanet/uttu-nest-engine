//! 정체 감지 기반 적응형 국소 최적해 탈출 기법
//! **Stagnation-Triggered Adaptive Local-Optimum Escape**
//!
//! > 네스팅 효율 개선이 일정 시간 멈추면, 현재 배치의 공백 구조와 후보해 다양성을
//! > 분석하여 이동 대상·교란 범위·탐색 강도를 동적으로 조절하고 새로운 배치 영역으로
//! > 탐색을 전환하는 기법
//!
//! # 권장 알고리즘 구조
//!
//! ```text
//! Swarm / Local Search 실행
//!           ↓
//! 탐색 정체 여부 감지
//!           ↓
//! 병목 패턴 및 공백 구조 분석
//!           ↓
//! 탈출 강도 결정
//!    Level 1 → 소규모 재배치
//!    Level 2 → 부분 배치 해체
//!    Level 3 → 부분 재시작
//!           ↓
//! 재삽입 및 압축
//!           ↓
//! 효율 개선 여부 확인
//!      ├─ 개선됨 → 정상 탐색 복귀
//!      └─ 개선 없음 → 탈출 강도 증가
//! ```
//!
//! # 현재 Swarm에 가장 적합한 조합
//!
//! ```text
//! ① Guided Local Search
//!    반복되는 패턴 관계에 동적 가중치 부여
//!
//! ② Geometry-Aware Destroy & Repair
//!    병목 패턴 집합을 선택적으로 해체하고 재삽입
//!
//! ③ Adaptive Swarm Diversification
//!    후보군이 한 배치로 수렴하면 하위 후보를 부분 재초기화
//! ```
//!
//! `jagua-rs`는 충돌 검사를 담당하는 Collision Detection Engine이고, 국소 최적해
//! 탈출은 그 위의 최적화 계층 — 즉 이 모듈 — 에서 구현한다.
//!
//! MultiPolygon 제약: 하나의 의류 아이템을 구성하는 여러 폴리곤은 각각 독립적으로
//! 교란하면 안 되고 **동일한 Transform을 공유하는 Linked Group 단위**로 이동·회전·
//! 재삽입해야 한다. 이 모듈의 모든 교란 연산은 [`Separator::place_groups`]가 정의한
//! 배치 단위(unit)를 원자 단위로 취급한다.

pub mod bottleneck;
pub mod conflict_burst;
pub mod guided;
pub mod levels;
pub mod pipeline;
pub mod stagnation;
pub mod swarm;

pub use bottleneck::{BottleneckAnalysis, BottleneckTarget, EscapeScoreTerms, select_bottleneck_items};
pub use conflict_burst::{
    CollisionRatioBand, ConflictBurstConfig, ConflictBurstOutcome, ConflictComponent,
    ConflictEscapeScoreTerms, ConflictGraph, StripRelaxationLadder, conflict_burst_partial_rebuild,
};
pub use guided::{GuidedPairKey, GuidedWeights};
pub use levels::{EscapeLevel, EscapeOutcome, apply_escape};
pub use pipeline::{AdaptiveEscapeConfig, AdaptiveEscapeState, escape_driven_exploration_phase};
pub use stagnation::{StagnationConfig, StagnationDetector, StagnationSignals};
pub use swarm::{ParticleTier, SwarmDiversification};

// ────────────────────────────────────────────────────────────────────────────
// §1. 탐색 정체 감지 — 문서 지정 임계값
// ────────────────────────────────────────────────────────────────────────────

/// 정체 판정 관측 창의 하한.
///
/// ```text
/// 최근 max(100, 3 × 패턴 수) 회 동안
/// ```
pub const STAGNATION_WINDOW_FLOOR: usize = 100;

/// 정체 판정 관측 창의 패턴 수 배수.
///
/// ```text
/// 최근 max(100, 3 × 패턴 수) 회 동안
/// ```
pub const STAGNATION_WINDOW_ITEM_FACTOR: usize = 3;

/// 효율 개선량 임계값. 문서의 `0.01%p`를 비율(0.0~1.0) 단위로 표현한 값이다.
///
/// ```text
/// 효율 개선량 < 0.01%p
/// ```
pub const STAGNATION_EFFICIENCY_EPS: f32 = 0.000_1;

/// 후보 배치 간 위치·순서 다양성의 기준값. 이 값 미만이면 "다양성이 기준 이하"다.
///
/// ```text
/// 후보 배치 간 위치·순서 다양성이 기준 이하
/// ```
pub const STAGNATION_DIVERSITY_THRESHOLD: f32 = 0.05;

// ────────────────────────────────────────────────────────────────────────────
// §2. EscapeScore — 문서 지정 가중치 (합 = 1.00)
// ────────────────────────────────────────────────────────────────────────────

/// ```text
/// EscapeScore(i)
///  = 0.35 × RightBoundaryContribution
///  + 0.30 × VoidAdjacency
///  + 0.20 × BlockingDegree
///  + 0.15 × MoveFailureFrequency
/// ```
pub const W_RIGHT_BOUNDARY_CONTRIBUTION: f32 = 0.35;
/// [`W_RIGHT_BOUNDARY_CONTRIBUTION`] 참조.
pub const W_VOID_ADJACENCY: f32 = 0.30;
/// [`W_RIGHT_BOUNDARY_CONTRIBUTION`] 참조.
pub const W_BLOCKING_DEGREE: f32 = 0.20;
/// [`W_RIGHT_BOUNDARY_CONTRIBUTION`] 참조.
pub const W_MOVE_FAILURE_FREQUENCY: f32 = 0.15;

// ────────────────────────────────────────────────────────────────────────────
// §3. 정체 수준에 따른 동적 교란 — 대상 규모
// ────────────────────────────────────────────────────────────────────────────

/// Level 1 — Soft Escape. `대상: 1~2개 패턴`
pub const L1_TARGET_MIN: usize = 1;
/// Level 1 — Soft Escape. `대상: 1~2개 패턴`
pub const L1_TARGET_MAX: usize = 2;

/// Level 1의 미세 Tilt 사다리. `±1° → ±3° → ±5°`
///
/// 의류 패턴의 회전 허용값이 `0°/180°`라면 그 범위 안에서만 탐색한다. 이 사다리는
/// 인스턴스가 미세 Tilt를 허용할 때에만(= 연속 회전 범위) 탈출 단계에 따라 확대된다.
pub const L1_TILT_LADDER_DEG: [f32; 3] = [1.0, 3.0, 5.0];

/// Level 2 — Medium Escape. `대상: 전체 패턴의 약 5~10%`
pub const L2_RATIO_MIN: f32 = 0.05;
/// Level 2 — Medium Escape. `대상: 전체 패턴의 약 5~10%`
pub const L2_RATIO_MAX: f32 = 0.10;

/// Level 3 — Hard Escape. `재구성: 오른쪽 15~30% 영역`
pub const L3_REBUILD_RATIO_MIN: f32 = 0.15;
/// Level 3 — Hard Escape. `재구성: 오른쪽 15~30% 영역`
pub const L3_REBUILD_RATIO_MAX: f32 = 0.30;

/// 탈출 강도 상한. 의사코드의 `escape_level = min(escape_level + 1, 2)`에 대응한다
/// (0 = Level 1, 1 = Level 2, 2 = Level 3).
pub const ESCAPE_LEVEL_MAX: u8 = 2;

// ────────────────────────────────────────────────────────────────────────────
// §5. Swarm 후보군 계층 — 문서 지정 비율
// ────────────────────────────────────────────────────────────────────────────

/// ```text
/// 상위 10~20% 후보 → Elite로 보존
/// ```
pub const SWARM_ELITE_RATIO_MIN: f32 = 0.10;
/// [`SWARM_ELITE_RATIO_MIN`] 참조.
pub const SWARM_ELITE_RATIO_MAX: f32 = 0.20;

/// ```text
/// 하위 20~30% 후보 → 다른 Elite를 기준으로 재초기화
/// ```
pub const SWARM_REINIT_RATIO_MIN: f32 = 0.20;
/// [`SWARM_REINIT_RATIO_MIN`] 참조.
pub const SWARM_REINIT_RATIO_MAX: f32 = 0.30;

/// 탈출 직후 일정 반복 동안 `Global Best 추종 강도 ↓`에 적용할 배율.
pub const POST_ESCAPE_GLOBAL_BEST_PULL: f32 = 0.5;

/// 탈출 직후 일정 반복 동안 `랜덤 탐색 및 개별 탐색 강도 ↑`에 적용할 배율.
pub const POST_ESCAPE_EXPLORATION_BOOST: f32 = 2.0;

/// 위 두 배율을 유지할 반복 수. 새로운 개선이 발생하면 그 전에 정상 파라미터로 복귀한다.
pub const POST_ESCAPE_BOOST_ITERS: usize = 20;

// ────────────────────────────────────────────────────────────────────────────
// Conflict Burst + Partial Rebuild — ConflictEscapeScore 가중치 (합 = 1.00)
// ────────────────────────────────────────────────────────────────────────────

/// ```text
/// ConflictEscapeScore(i)
/// =
/// 0.35 × CollisionDegree
/// + 0.25 × StripBoundaryContribution
/// + 0.20 × BlockingScore
/// + 0.10 × Area
/// + 0.10 × Concavity
/// ```
pub const W_COLLISION_DEGREE: f32 = 0.35;
/// [`W_COLLISION_DEGREE`] 참조.
pub const W_STRIP_BOUNDARY_CONTRIBUTION: f32 = 0.25;
/// [`W_COLLISION_DEGREE`] 참조.
pub const W_BLOCKING_SCORE: f32 = 0.20;
/// [`W_COLLISION_DEGREE`] 참조.
pub const W_CONFLICT_AREA: f32 = 0.10;
/// [`W_COLLISION_DEGREE`] 참조.
pub const W_CONCAVITY: f32 = 0.10;

/// `상위 5~15% 정도를 제거한 다음 다시 삽입합니다.`
pub const CONFLICT_REMOVE_RATIO_MIN: f32 = 0.05;
/// [`CONFLICT_REMOVE_RATIO_MIN`] 참조.
pub const CONFLICT_REMOVE_RATIO_MAX: f32 = 0.15;

// ── Adaptive Strip Relaxation — collision ratio에 따른 ΔL ──
//
// ```text
// collision ratio < 10%   → ΔL = 1~2%
// collision ratio 10~30%  → ΔL = 3~5%
// collision ratio > 30%   → ΔL = 5~10%
// ```

/// `collision ratio < 10%` 구간 경계.
pub const COLLISION_RATIO_LOW: f32 = 0.10;
/// `collision ratio 10~30%` 구간 상단 경계.
pub const COLLISION_RATIO_HIGH: f32 = 0.30;

/// `collision ratio < 10% → ΔL = 1~2%`
pub const DELTA_L_LOW: (f32, f32) = (0.01, 0.02);
/// `collision ratio 10~30% → ΔL = 3~5%`
pub const DELTA_L_MID: (f32, f32) = (0.03, 0.05);
/// `collision ratio > 30% → ΔL = 5~10%`
pub const DELTA_L_HIGH: (f32, f32) = (0.05, 0.10);

/// 탈출 직후 원래 Strip 길이로 점진 복귀하는 사다리.
///
/// ```text
/// L + 6%
///  ↓
/// L + 4%
///  ↓
/// L + 2%
///  ↓
/// L
///  ↓
/// L - ε 시도
/// ```
pub const STRIP_RELAXATION_LADDER: [f32; 4] = [0.06, 0.04, 0.02, 0.00];

/// 사다리 마지막 단계의 `L - ε` 시도량(현재 폭 대비 비율).
pub const STRIP_RELAXATION_EPSILON: f32 = 0.005;

// ── 충돌 방향 기반 강제 이동 (Collision Push) ──
//
// ```text
// 평소 Local Move
// 1~3 mm
//
// Escape Mode
// 10~50 mm 또는 bbox 크기의 20~50%
// ```

/// `평소 Local Move  1~3 mm`
pub const LOCAL_MOVE_MM: (f32, f32) = (1.0, 3.0);
/// `Escape Mode  10~50 mm`
pub const ESCAPE_MOVE_MM: (f32, f32) = (10.0, 50.0);
/// `Escape Mode  ... 또는 bbox 크기의 20~50%`
pub const ESCAPE_MOVE_BBOX_RATIO: (f32, f32) = (0.20, 0.50);

// ── Swarm particle 계층별 탈출 강도 ──
//
// ```text
// Best 20%   → 현재 배치 유지 + 약한 local search
// Middle 50% → Collision Push + 부분 재삽입
// Worst 30%  → Strip Relaxation + 큰 Destroy & Repair
// ```

/// `Best 20% → 현재 배치 유지 + 약한 local search`
pub const PARTICLE_BEST_RATIO: f32 = 0.20;
/// `Middle 50% → Collision Push + 부분 재삽입`
pub const PARTICLE_MIDDLE_RATIO: f32 = 0.50;
/// `Worst 30% → Strip Relaxation + 큰 Destroy & Repair`
pub const PARTICLE_WORST_RATIO: f32 = 0.30;

// ────────────────────────────────────────────────────────────────────────────
// §4. Guided Local Search 동적 가중치
// ────────────────────────────────────────────────────────────────────────────

/// 중립 가중치. 문서의 `Weight(a, b) → 1로 점진적 감소`가 수렴하는 값이다.
pub const GUIDED_WEIGHT_NEUTRAL: f32 = 1.0;

/// `동일한 충돌이나 방해 관계가 반복되면 Weight(a, b) 증가` — 한 번의 증가 배율.
pub const GUIDED_WEIGHT_INCREASE: f32 = 1.30;

/// `문제가 해결되고 일정 시간이 지나면 Weight(a, b) → 1로 점진적 감소` — 한 번의 감쇠 배율.
pub const GUIDED_WEIGHT_DECAY: f32 = 0.90;

/// 학습된 가중치의 상한. 수치 폭주를 막는다.
pub const GUIDED_WEIGHT_MAX: f32 = 100.0;

/// 문서에서 정한 두 구간 비율 `(min, max)`을 정체 강도 `t ∈ [0, 1]`로 선형 보간한다.
///
/// 문서가 `5~10%`, `15~30%`처럼 범위로 지정한 모든 항목이 이 함수를 거친다.
#[inline]
pub fn lerp_ratio(range: (f32, f32), t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    range.0 + (range.1 - range.0) * t
}
