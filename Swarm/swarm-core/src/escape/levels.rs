//! 정체 수준에 따른 동적 교란 — Level 1 / 2 / 3.
//!
//! ```text
//! Level 1 → 소규모 재배치
//! Level 2 → 부분 배치 해체
//! Level 3 → 부분 재시작
//! ```
//!
//! # Level 1 — Soft Escape
//!
//! 초기 정체에서는 좋은 배치를 최대한 보존한다.
//!
//! ```text
//! 대상: 1~2개 패턴
//!
//! 동작:
//! - 패턴 순서 교환
//! - 허용 각도 내 회전
//! - 큰 공백 주변으로 재삽입
//! - 오른쪽 경계 패턴의 위치 교환
//! ```
//!
//! 의류 패턴의 회전 허용값이 `0°/180°`라면 그 범위 안에서만 탐색해야 한다. 미세 Tilt를
//! 허용하는 프로젝트라면 `±1° → ±3° → ±5°`처럼 탈출 단계에 따라 범위를 확대할 수 있다
//! ([`L1_TILT_LADDER_DEG`]).
//!
//! # Level 2 — Medium Escape
//!
//! Level 1을 여러 차례 수행해도 개선되지 않을 때 실행한다.
//!
//! ```text
//! 대상: 전체 패턴의 약 5~10%
//!
//! 동작:
//! - 병목 패턴 제거
//! - 병목 주변 패턴도 함께 제거
//! - 패턴 순서 재구성
//! - 빈 공간 적합도 기준으로 재삽입
//! - 최종 Left/Bottom Compaction
//! ```
//!
//! 핵심은 **한 개 패턴이 아니라 서로 막고 있는 작은 패턴 집합을 동시에 해체하는 것**이다.
//!
//! # Level 3 — Hard Escape
//!
//! 장시간 정체된 경우 부분 재시작을 수행한다.
//!
//! ```text
//! 유지:
//! - 왼쪽의 밀집도가 높은 안정 영역
//! - 현재 최고 배치의 핵심 패턴군
//!
//! 재구성:
//! - 오른쪽 15~30% 영역
//! - 가장 큰 내부 공백 주변
//! - 성능이 낮은 Swarm 후보군
//! ```
//!
//! 전체 랜덤 재시작보다 **좋은 왼쪽 배치를 보존하고 오른쪽 영역만 재구성하는 방식**이
//! 의류 네스팅에 더 적합하다.
//!
//! # 재삽입 경로에 관한 구현 원칙
//!
//! "제거 → 순서 재구성 → 빈 공간 적합도 기준 재삽입"은 이미 검증된
//! [`ruin_and_recreate`] 경로가 그대로 수행한다. 이 모듈에서 직접
//! `remove_item` / `place_item` 쌍을 돌리면 재삽입 실패 시 아이템이 레이아웃에서
//! 유실되어 `SPProblem`의 demand 불변식이 깨지므로, **해체 이후의 재삽입은 전적으로
//! R&R에 위임한다**. 이 모듈이 직접 하는 배치 변경은 아이템을 절대 잃지 않는
//! [`Separator::move_item`](crate::optimizer::separator::Separator::move_item)뿐이며,
//! 이는 MultiPolygon Linked Group을 place_group 단위로 원자 이동시킨다.

use crate::escape::bottleneck::BottleneckAnalysis;
use crate::escape::{
    ESCAPE_LEVEL_MAX, ESCAPE_MOVE_BBOX_RATIO, L1_TARGET_MAX, L1_TARGET_MIN, L1_TILT_LADDER_DEG,
    L2_RATIO_MAX, L2_RATIO_MIN, L3_REBUILD_RATIO_MAX, L3_REBUILD_RATIO_MIN, lerp_ratio,
};
use crate::optimizer::lns::{BandTabu, RuinStrategy, ruin_and_recreate};
use crate::optimizer::separator::Separator;
use crate::quantify::tracker::{CTSnapshot, CollisionTracker};
use jagua_rs::entities::{PItemKey};
use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::geo_enums::RotationRange;
use jagua_rs::geometry::primitives::Rect;
use jagua_rs::probs::spp::entities::SPSolution;
use rand::rngs::Xoshiro256PlusPlus;
use std::collections::HashSet;

// ── 내부 구현 상수 ───────────────────────────────────────────────────────────
// 문서가 지정한 임계값·비율은 전부 `crate::escape`의 확정 상수를 쓴다.
// 아래는 수치 판정용 허용 오차와 반복 상한뿐이다.

/// 이동/회전이 "실제로 일어났다"고 볼 최소 변화량(mm).
const POSE_EPS: f32 = 1.0e-4;
/// 회전 동일성 판정 허용 오차(rad).
const ROTATION_EPS: f32 = 1.0e-4;
/// 총 loss 비교 허용 오차.
const LOSS_EPS: f32 = 1.0e-5;
/// Level 2 — 병목 bbox를 대각선의 이 비율만큼 팽창시켜 주변 패턴을 모은다.
const NEIGHBOR_EXPANSION_DIAG_RATIO: f32 = 0.25;
/// Level 2 — 재삽입을 위임할 R&R 반복 상한.
const L2_MAX_RNR_ROUNDS: usize = 2;
/// Level 3 — 부분 재시작에서 R&R을 반복 호출할 상한.
const L3_MAX_RNR_ROUNDS: usize = 3;

// ────────────────────────────────────────────────────────────────────────────
// 탈출 강도
// ────────────────────────────────────────────────────────────────────────────

/// 탈출 강도. 의사코드의 `escape_level`(0/1/2)에 1:1 대응한다.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub enum EscapeLevel {
    /// Level 1 — 소규모 재배치. `대상: 1~2개 패턴`
    Soft,
    /// Level 2 — 부분 배치 해체. `대상: 전체 패턴의 약 5~10%`
    Medium,
    /// Level 3 — 부분 재시작. `재구성: 오른쪽 15~30% 영역`
    Hard,
}

impl EscapeLevel {
    /// `0 → Soft`, `1 → Medium`, `2 이상 → Hard`.
    pub fn from_code(code: u8) -> Self {
        match code {
            0 => EscapeLevel::Soft,
            1 => EscapeLevel::Medium,
            _ => EscapeLevel::Hard,
        }
    }

    /// [`Self::from_code`]의 역함수.
    pub fn code(self) -> u8 {
        match self {
            EscapeLevel::Soft => 0,
            EscapeLevel::Medium => 1,
            EscapeLevel::Hard => 2,
        }
    }

    /// `escape_level = min(escape_level + 1, 2)` — [`ESCAPE_LEVEL_MAX`]에서 포화한다.
    pub fn next(self) -> Self {
        Self::from_code(self.code().saturating_add(1).min(ESCAPE_LEVEL_MAX))
    }
}

/// 한 번의 [`apply_escape`] 결과 요약.
#[derive(Clone, Copy, Debug)]
pub struct EscapeOutcome {
    /// 위치/각도가 실제로 바뀐 패턴 수.
    pub n_relocated: usize,
    /// 해체(파괴) 단계에서 자리를 잃은 패턴 수.
    pub n_destroyed: usize,
    /// 재삽입(repair) 단계에서 다시 배치된 패턴 수.
    pub n_repaired: usize,
    /// 교란이 레이아웃에 반영되었는지. `false`면 호출 전 상태로 완전히 되돌아갔다.
    pub applied: bool,
}

impl EscapeOutcome {
    fn empty() -> Self {
        Self {
            n_relocated: 0,
            n_destroyed: 0,
            n_repaired: 0,
            applied: false,
        }
    }

    /// 레이아웃을 실제로 건드렸는가.
    fn touched(&self) -> bool {
        self.n_relocated + self.n_destroyed + self.n_repaired > 0
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 진입점
// ────────────────────────────────────────────────────────────────────────────

/// 정체 수준(`level`)과 정체 강도(`intensity ∈ [0, 1]`)에 맞춰 현재 배치를 교란한다.
///
/// 이 함수는 **트랜잭션**이다. 시작 시 문제/트래커/RNG 스냅샷을 잡고,
/// `term.kill()`이거나 아무 것도 적용하지 못했으면 완전히 되돌린 뒤
/// `applied: false`를 반환한다.
pub fn apply_escape(
    sep: &mut Separator,
    level: EscapeLevel,
    analysis: &BottleneckAnalysis,
    intensity: f32,
    rnr_cfg: &crate::config::RuinRecreateConfig,
    tabu: &mut crate::optimizer::lns::BandTabu,
    term: &impl crate::util::terminator::Terminator,
) -> EscapeOutcome {
    let mut outcome = EscapeOutcome::empty();
    if term.kill() || sep.prob.layout().placed_items().is_empty() {
        return outcome;
    }

    let intensity = if intensity.is_finite() {
        intensity.clamp(0.0, 1.0)
    } else {
        0.0
    };

    // ── 트랜잭션 시작 ──
    let before_sol = sep.prob.save();
    let before_ct = sep.ct.save();
    let before_rng = sep.rng.clone();

    match level {
        EscapeLevel::Soft => apply_soft_escape(sep, analysis, intensity, &mut outcome),
        EscapeLevel::Medium => {
            apply_medium_escape(sep, analysis, intensity, rnr_cfg, tabu, term, &mut outcome)
        }
        EscapeLevel::Hard => {
            apply_hard_escape(sep, analysis, intensity, rnr_cfg, tabu, term, &mut outcome)
        }
    }

    // ── 트랜잭션 종료: 중단되었거나 아무 것도 못 했으면 통째로 롤백 ──
    if term.kill() || !outcome.touched() {
        rollback_escape(sep, &before_sol, &before_ct, &before_rng);
        return EscapeOutcome::empty();
    }

    outcome.applied = true;
    outcome
}

/// 트랜잭션 롤백. strip 폭이 바뀌었으면 폭부터 되돌려 worker/tracker를 resync한다.
fn rollback_escape(
    sep: &mut Separator,
    sol: &SPSolution,
    ct: &CTSnapshot,
    rng: &Xoshiro256PlusPlus,
) {
    if sep.prob.strip_width() != sol.strip_width() {
        // split_position을 +∞로 두면 어떤 아이템도 시프트되지 않고
        // (centroid.x > +∞ 는 항상 거짓) 폭 변경 + resync만 일어난다.
        sep.change_strip_width(sol.strip_width(), Some(f32::INFINITY));
    }
    sep.prob.restore(sol);
    sep.ct.copy_from(ct);
    sep.rng = rng.clone();
}

// ────────────────────────────────────────────────────────────────────────────
// Level 1 — Soft Escape
// ────────────────────────────────────────────────────────────────────────────

/// ```text
/// 대상: 1~2개 패턴
///
/// 동작:
/// - 패턴 순서 교환
/// - 허용 각도 내 회전
/// - 큰 공백 주변으로 재삽입
/// - 오른쪽 경계 패턴의 위치 교환
/// ```
fn apply_soft_escape(
    sep: &mut Separator,
    analysis: &BottleneckAnalysis,
    intensity: f32,
    outcome: &mut EscapeOutcome,
) {
    let n = soft_target_count(intensity);
    let mut keys: Vec<PItemKey> = analysis
        .top(n)
        .iter()
        .filter(|t| sep.prob.layout().placed_items().contains_key(t.pk))
        .filter(|t| !is_frozen(sep, t.item_id))
        .map(|t| t.pk)
        .collect();
    if keys.is_empty() {
        return;
    }

    // ① 패턴 순서 교환 / 오른쪽 경계 패턴의 위치 교환.
    //    EscapeScore 상위 두 패턴은 대개 우측 경계 기여도가 가장 큰 패턴들이므로,
    //    둘의 평행이동을 맞바꾸는 것이 곧 "오른쪽 경계 패턴의 위치 교환"이다.
    if keys.len() >= 2 {
        if let Some((a, b)) = swap_translations(sep, keys[0], keys[1]) {
            keys[0] = a;
            keys[1] = b;
            outcome.n_relocated += 2;
        }
    }

    // ② 허용 각도 내 회전.
    for slot in keys.iter_mut() {
        if let Some(new_pk) = rotate_within_allowed(sep, *slot, intensity) {
            *slot = new_pk;
            outcome.n_relocated += 1;
        }
    }

    // ③ 큰 공백 주변으로 재삽입.
    if let Some(void) = analysis.largest_void {
        if let Some(&pk) = keys.first() {
            if shift_towards_void(sep, pk, void, 1.0).is_some() {
                outcome.n_relocated += 1;
            }
        }
    }
}

/// 두 패턴의 평행이동을 맞바꾼다. 각자의 회전은 허용 각도를 지키도록 그대로 둔다.
/// 교환 후 총 loss가 나빠지면 되돌리고 `None`을 돌려준다.
fn swap_translations(
    sep: &mut Separator,
    a: PItemKey,
    b: PItemKey,
) -> Option<(PItemKey, PItemKey)> {
    if a == b {
        return None;
    }
    let (dt_a, dt_b) = {
        let pa = sep.prob.layout().placed_items().get(a)?;
        let pb = sep.prob.layout().placed_items().get(b)?;
        (pa.d_transf(), pb.d_transf())
    };
    let (ta, tb) = (dt_a.translation(), dt_b.translation());
    if (ta.0 - tb.0).abs() < POSE_EPS && (ta.1 - tb.1).abs() < POSE_EPS {
        return None;
    }

    let before_sol = sep.prob.save();
    let before_ct = sep.ct.save();
    let loss_before = sep.ct.get_total_loss();

    let new_a = sep.move_item(a, DTransformation::new(dt_a.rotation(), tb));
    // place_group 단위 이동은 여러 키를 한 번에 교체하므로 상대 키가 사라질 수 있다.
    if !sep.prob.layout().placed_items().contains_key(b) {
        sep.prob.restore(&before_sol);
        sep.ct.copy_from(&before_ct);
        return None;
    }
    let new_b = sep.move_item(b, DTransformation::new(dt_b.rotation(), ta));

    let loss_after = sep.ct.get_total_loss();
    if !loss_after.is_finite() || loss_after > loss_before + LOSS_EPS {
        sep.prob.restore(&before_sol);
        sep.ct.copy_from(&before_ct);
        return None;
    }
    Some((new_a, new_b))
}

/// 허용 각도 내 회전. `RotationRange::Discrete`면 그 집합 안에서만 고르고
/// (의류 패턴의 `0°/180°`), `Continuous`면 [`L1_TILT_LADDER_DEG`]의 미세 Tilt를 쓴다.
fn rotate_within_allowed(sep: &mut Separator, pk: PItemKey, intensity: f32) -> Option<PItemKey> {
    let (dt, item_id) = {
        let pi = sep.prob.layout().placed_items().get(pk)?;
        (pi.d_transf(), pi.item().idx())
    };
    if is_frozen(sep, item_id) {
        return None;
    }
    let current = dt.rotation();

    let new_rot = {
        let item = sep.instance.item(item_id);
        match item.allowed_orientations().rotations() {
            // 회전이 금지된 패턴은 각도를 건드리지 않는다.
            RotationRange::None => None,
            RotationRange::Discrete(set) => pick_discrete_rotation(set, current),
            RotationRange::Continuous => {
                // ±deg — 부호는 item_id 패리티로 결정해 재현 가능하게 둔다.
                let deg = tilt_for_intensity(intensity);
                let signed = if item_id % 2 == 0 { deg } else { -deg };
                Some(current + signed.to_radians())
            }
        }
    }?;

    if (new_rot - current).abs() < ROTATION_EPS {
        return None;
    }
    Some(sep.move_item(pk, DTransformation::new(new_rot, dt.translation())))
}

/// 패턴을 최대 공백 중심 쪽으로 `ratio` 비율만큼 끌어당긴다. 이동 후 bbox가 컨테이너를
/// 벗어나지 않도록 변위를 클램프한다.
fn shift_towards_void(
    sep: &mut Separator,
    pk: PItemKey,
    void: Rect,
    ratio: f32,
) -> Option<PItemKey> {
    let (dt, bbox, item_id) = {
        let pi = sep.prob.layout().placed_items().get(pk)?;
        (pi.d_transf(), pi.shape().bbox(), pi.item().idx())
    };
    if is_frozen(sep, item_id) {
        return None;
    }
    let from = centre_of(bbox);
    let to = centre_of(void);
    let full = (to.0 - from.0, to.1 - from.1);
    let raw = (full.0 * ratio, full.1 * ratio);
    let container = sep.prob.layout().container().outer_cd().bbox();
    let (dx, dy) = clamp_step(bbox, container, raw);
    if dx.abs() < POSE_EPS && dy.abs() < POSE_EPS {
        return None;
    }
    let t = dt.translation();
    Some(sep.move_item(
        pk,
        DTransformation::new(dt.rotation(), (t.0 + dx, t.1 + dy)),
    ))
}

// ────────────────────────────────────────────────────────────────────────────
// Level 2 — Medium Escape
// ────────────────────────────────────────────────────────────────────────────

/// ```text
/// 대상: 전체 패턴의 약 5~10%
///
/// 동작:
/// - 병목 패턴 제거
/// - 병목 주변 패턴도 함께 제거
/// - 패턴 순서 재구성
/// - 빈 공간 적합도 기준으로 재삽입
/// - 최종 Left/Bottom Compaction
/// ```
///
/// 해체는 병목 집합을 통째로 흩는 것으로 수행하고(아이템 유실 없음), 재삽입은
/// [`ruin_and_recreate`]`(Fragmented)`에 위임한다.
fn apply_medium_escape(
    sep: &mut Separator,
    analysis: &BottleneckAnalysis,
    intensity: f32,
    rnr_cfg: &crate::config::RuinRecreateConfig,
    tabu: &mut BandTabu,
    term: &impl crate::util::terminator::Terminator,
    outcome: &mut EscapeOutcome,
) {
    if !sep
        .prob
        .layout()
        .placed_items()
        .values()
        .any(|pi| sep.can_move_individually(pi.item().idx()))
    {
        apply_soft_escape(sep, analysis, intensity, outcome);
        return;
    }
    let ratio = lerp_ratio((L2_RATIO_MIN, L2_RATIO_MAX), intensity);
    let bottlenecks: Vec<(PItemKey, Rect)> = analysis
        .top_ratio(ratio)
        .iter()
        .filter_map(|t| {
            let pi = sep.prob.layout().placed_items().get(t.pk)?;
            (sep.can_move_individually(pi.item().idx())).then_some((t.pk, pi.shape().bbox()))
        })
        .collect();
    if bottlenecks.is_empty() {
        return;
    }

    // ── 병목 + 주변 패턴 수집 ──
    // "서로 막고 있는 작은 패턴 집합을 동시에 해체" — 병목 bbox를 대각선의 25%만큼
    // 팽창시켜 겹치는 비고정 패턴을 같은 집합으로 묶는다.
    let frac = lerp_ratio(ESCAPE_MOVE_BBOX_RATIO, intensity);
    let void_centre = analysis.largest_void.map(centre_of);
    let cap = usize::max(4, sep.prob.layout().placed_items().len() / 3);

    let mut seen: HashSet<PItemKey> = HashSet::new();
    let mut plan: Vec<(PItemKey, (f32, f32))> = Vec::new();

    for &(pk, bbox) in &bottlenecks {
        if plan.len() >= cap {
            break;
        }
        let centre = centre_of(bbox);
        let diag = diagonal_of(bbox);

        // 병목 본체: 가장 큰 공백 쪽으로 (공백을 모르면 우측 frontier 쪽으로)
        if seen.insert(pk) {
            let step = match void_centre {
                Some(vc) => step_towards(centre, vc, diag * frac),
                None => (diag * frac, 0.0),
            };
            plan.push((pk, step));
        }

        // 주변 패턴: 병목 중심에서 바깥으로 밀어 매듭을 푼다.
        let halo = expanded_bbox(bbox);
        let neighbours: Vec<(PItemKey, Rect)> = sep
            .prob
            .layout()
            .placed_items()
            .iter()
            .filter(|(other_pk, pi)| {
                *other_pk != pk && !seen.contains(other_pk) && sep.can_move_individually(pi.item().idx())
            })
            .filter(|(_, pi)| rects_overlap(halo, pi.shape().bbox()))
            .map(|(other_pk, pi)| (other_pk, pi.shape().bbox()))
            .collect();

        for (other_pk, other_bbox) in neighbours {
            if plan.len() >= cap {
                break;
            }
            if !seen.insert(other_pk) {
                continue;
            }
            let step = step_away(
                centre,
                centre_of(other_bbox),
                diagonal_of(other_bbox) * frac,
            );
            plan.push((other_pk, step));
        }
    }

    // ── 해체: 집합을 동시에 흩는다 ──
    let container = sep.prob.layout().container().outer_cd().bbox();
    for (pk, raw) in plan {
        if term.kill() {
            return;
        }
        let Some(pi) = sep.prob.layout().placed_items().get(pk) else {
            continue;
        };
        let (dt, bbox) = (pi.d_transf(), pi.shape().bbox());
        let (dx, dy) = clamp_step(bbox, container, raw);
        if dx.abs() < POSE_EPS && dy.abs() < POSE_EPS {
            continue;
        }
        let t = dt.translation();
        sep.move_item(
            pk,
            DTransformation::new(dt.rotation(), (t.0 + dx, t.1 + dy)),
        );
        outcome.n_destroyed += 1;
    }

    // ── 재삽입: 순서 재구성 + 빈 공간 적합도 평가는 R&R(Fragmented)에 위임 ──
    for _ in 0..L2_MAX_RNR_ROUNDS {
        if term.kill() {
            return;
        }
        let n = ruin_and_recreate(sep, rnr_cfg, RuinStrategy::Fragmented, tabu, term);
        outcome.n_repaired += n;
        if n == 0 {
            break;
        }
    }

    // ── 최종 Left/Bottom Compaction ──
    compact_strip(sep);
}

// ────────────────────────────────────────────────────────────────────────────
// Level 3 — Hard Escape
// ────────────────────────────────────────────────────────────────────────────

/// ```text
/// 유지:
/// - 왼쪽의 밀집도가 높은 안정 영역
/// - 현재 최고 배치의 핵심 패턴군
///
/// 재구성:
/// - 오른쪽 15~30% 영역
/// - 가장 큰 내부 공백 주변
/// ```
fn apply_hard_escape(
    sep: &mut Separator,
    analysis: &BottleneckAnalysis,
    intensity: f32,
    rnr_cfg: &crate::config::RuinRecreateConfig,
    tabu: &mut BandTabu,
    term: &impl crate::util::terminator::Terminator,
    outcome: &mut EscapeOutcome,
) {
    if !sep
        .prob
        .layout()
        .placed_items()
        .values()
        .any(|pi| sep.can_move_individually(pi.item().idx()))
    {
        apply_soft_escape(sep, analysis, intensity, outcome);
        return;
    }
    // ① 유지 영역 경계. strip 폭을 x축 길이로 보고 왼쪽부터 보존한다.
    let strip_length = if analysis.strip_length.is_finite() && analysis.strip_length > 0.0 {
        analysis.strip_length
    } else {
        sep.prob.strip_width()
    };
    let x_keep = hard_keep_boundary(strip_length, intensity);
    let void_halo = analysis.largest_void.map(expanded_bbox);

    // ② 재구성 대상 = x_keep 오른쪽에 bbox 중심이 있는 비고정 아이템
    //                 + 가장 큰 내부 공백 주변 아이템
    let mut rebuild_ids: HashSet<usize> = HashSet::new();
    for (_, pi) in sep.prob.layout().placed_items().iter() {
        if !sep.can_move_individually(pi.item().idx()) {
            continue;
        }
        let bbox = pi.shape().bbox();
        let right_of_keep = centre_of(bbox).0 > x_keep;
        let near_void = void_halo.is_some_and(|halo| rects_overlap(halo, bbox));
        if right_of_keep || near_void {
            rebuild_ids.insert(pi.item().idx());
        }
    }
    if rebuild_ids.is_empty() {
        return;
    }

    // ③ 왼쪽 핵심 패턴군을 일시 고정한다.
    //    `freeze_items`는 item_id 단위이므로, 같은 item_id의 다른 사본이 재구성
    //    대상이면 그 id 전체를 고정하지 않는다(위 rebuild_ids 우선 규칙).
    let keep_ids: Vec<usize> = {
        let mut ids: Vec<usize> = sep
            .prob
            .layout()
            .placed_items()
            .iter()
            .map(|(_, pi)| pi.item().idx())
            .filter(|id| !rebuild_ids.contains(id))
            .collect();
        ids.sort_unstable();
        ids.dedup();
        ids
    };
    let saved_mask = sep.frozen_mask.clone();
    sep.freeze_items(&keep_ids);

    // ④ 재구성은 R&R(Rightmost)에 위임한다. 고정된 왼쪽 패턴은 victim이 되지 않으므로
    //    오른쪽 영역만 파괴-재생성된다.
    for _ in 0..L3_MAX_RNR_ROUNDS {
        if term.kill() {
            break;
        }
        let n = ruin_and_recreate(sep, rnr_cfg, RuinStrategy::Rightmost, tabu, term);
        outcome.n_destroyed += n;
        outcome.n_repaired += n;
        if n == 0 {
            break;
        }
    }

    // ⑤ 원래 frozen_mask 복원 — 학습된 GLS 가중치는 이월한다.
    restore_frozen_mask(sep, saved_mask);

    if term.kill() {
        return;
    }
    // R&R 경로가 아무 것도 건드리지 못했다면(현재 `ruin_and_recreate`는 inert stub)
    // Hard 탈출은 no-op 이 되어 `apply_escape` 가 통째로 롤백한다 — 정체가 영원히
    // 풀리지 않는다. 그 경우 Level 2 (직접 move_item 으로 해체) 로 내려가 실제 교란을
    // 보장한다.
    if !outcome.touched() {
        apply_medium_escape(sep, analysis, intensity, rnr_cfg, tabu, term, outcome);
        return;
    }
    compact_strip(sep);
}

/// 일시 고정을 해제하고 collision tracker를 재구축한다.
fn restore_frozen_mask(sep: &mut Separator, saved_mask: Vec<bool>) {
    sep.frozen_mask = saved_mask;
    let old = std::mem::replace(
        &mut sep.ct,
        CollisionTracker::new_with_frozen_prepared_and_groups(
            &sep.prob.layout(),
            &sep.frozen_mask,
            &sep.prepared,
            &sep.place_groups,
        ),
    );
    sep.ct.inherit_weights_from(&old);
}

// ────────────────────────────────────────────────────────────────────────────
// 공용 헬퍼 (순수 계산)
// ────────────────────────────────────────────────────────────────────────────

fn is_frozen(sep: &Separator, item_id: usize) -> bool {
    sep.frozen_mask.get(item_id).copied().unwrap_or(false)
}

/// Left/Bottom Compaction. `refit_strip`은 폭을 늘릴 수도 있으므로 축소일 때만 부른다.
fn compact_strip(sep: &mut Separator) -> bool {
    if sep.prob.layout().placed_items().is_empty() {
        return false;
    }
    // 충돌이 남아 있는 배치를 좁히면 겹침만 더 깊어진다 — 파이프라인의
    // `local_compaction()` 과 같은 이유로 손실 0 일 때만 압축한다.
    if sep.ct.get_total_loss() > 0.0 {
        return false;
    }
    let x_max = sep
        .prob
        .layout()
        .placed_items()
        .iter()
        .map(|(_, pi)| pi.shape().bbox().x_max)
        .fold(f32::NEG_INFINITY, f32::max);
    if !x_max.is_finite() || x_max * 1.00001 >= sep.prob.strip_width() {
        return false;
    }
    sep.refit_strip()
}

/// `대상: 1~2개 패턴` — 정체 강도로 [`L1_TARGET_MIN`]..[`L1_TARGET_MAX`]를 보간한다.
fn soft_target_count(intensity: f32) -> usize {
    let t = intensity.clamp(0.0, 1.0);
    let span = L1_TARGET_MAX.saturating_sub(L1_TARGET_MIN) as f32;
    L1_TARGET_MIN + (span * t).round() as usize
}

/// `±1° → ±3° → ±5°` — 탈출 단계(정체 강도)에 따라 미세 Tilt 범위를 확대한다.
fn tilt_for_intensity(intensity: f32) -> f32 {
    let t = intensity.clamp(0.0, 1.0);
    let n = L1_TILT_LADDER_DEG.len();
    let idx = ((t * n as f32) as usize).min(n - 1);
    L1_TILT_LADDER_DEG[idx]
}

/// 이산 회전 집합(`0°/180°` 등)에서 현재 각도와 가장 먼 다른 각도를 고른다.
fn pick_discrete_rotation(set: &[f32], current: f32) -> Option<f32> {
    set.iter()
        .copied()
        .filter(|r| r.is_finite() && (r - current).abs() > ROTATION_EPS)
        .max_by(|a, b| (a - current).abs().total_cmp(&(b - current).abs()))
}

/// `보존 경계 x_keep = strip_length * (1 - r)`, `r = lerp(15%, 30%)`.
fn hard_keep_boundary(strip_length: f32, intensity: f32) -> f32 {
    let r = lerp_ratio((L3_REBUILD_RATIO_MIN, L3_REBUILD_RATIO_MAX), intensity);
    strip_length * (1.0 - r)
}

fn centre_of(r: Rect) -> (f32, f32) {
    ((r.x_min + r.x_max) * 0.5, (r.y_min + r.y_max) * 0.5)
}

fn diagonal_of(r: Rect) -> f32 {
    let (w, h) = (r.x_max - r.x_min, r.y_max - r.y_min);
    (w * w + h * h).sqrt()
}

/// 병목 bbox를 대각선의 [`NEIGHBOR_EXPANSION_DIAG_RATIO`]만큼 사방으로 팽창시킨다.
fn expanded_bbox(r: Rect) -> Rect {
    let e = diagonal_of(r) * NEIGHBOR_EXPANSION_DIAG_RATIO;
    Rect {
        x_min: r.x_min - e,
        y_min: r.y_min - e,
        x_max: r.x_max + e,
        y_max: r.y_max + e,
    }
}

fn rects_overlap(a: Rect, b: Rect) -> bool {
    a.x_min < b.x_max && b.x_min < a.x_max && a.y_min < b.y_max && b.y_min < a.y_max
}

/// `from`에서 `to` 방향으로 최대 `max_len`만큼 이동하는 변위.
fn step_towards(from: (f32, f32), to: (f32, f32), max_len: f32) -> (f32, f32) {
    let (dx, dy) = (to.0 - from.0, to.1 - from.1);
    let len = (dx * dx + dy * dy).sqrt();
    if !len.is_finite() || len < POSE_EPS || !max_len.is_finite() || max_len <= 0.0 {
        return (0.0, 0.0);
    }
    let s = max_len.min(len) / len;
    (dx * s, dy * s)
}

/// `anchor`에서 `from`을 바깥으로 `len`만큼 밀어내는 변위. 두 점이 겹치면 +x로 민다.
fn step_away(anchor: (f32, f32), from: (f32, f32), len: f32) -> (f32, f32) {
    if !len.is_finite() || len <= 0.0 {
        return (0.0, 0.0);
    }
    let (dx, dy) = (from.0 - anchor.0, from.1 - anchor.1);
    let d = (dx * dx + dy * dy).sqrt();
    if !d.is_finite() || d < POSE_EPS {
        return (len, 0.0);
    }
    (dx / d * len, dy / d * len)
}

/// 이동 후 bbox가 컨테이너를 벗어나지 않도록 변위를 클램프한다.
/// bbox가 컨테이너보다 큰 축은 이동시키지 않는다.
fn clamp_step(bbox: Rect, container: Rect, step: (f32, f32)) -> (f32, f32) {
    let clamp_axis = |v: f32, lo: f32, hi: f32| -> f32 {
        if lo <= hi && v.is_finite() {
            v.clamp(lo, hi)
        } else {
            0.0
        }
    };
    (
        clamp_axis(
            step.0,
            container.x_min - bbox.x_min,
            container.x_max - bbox.x_max,
        ),
        clamp_axis(
            step.1,
            container.y_min - bbox.y_min,
            container.y_max - bbox.y_max,
        ),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    fn rect(x_min: f32, y_min: f32, x_max: f32, y_max: f32) -> Rect {
        Rect {
            x_min,
            y_min,
            x_max,
            y_max,
        }
    }

    #[test]
    fn escape_level_codes_round_trip() {
        assert_eq!(EscapeLevel::from_code(0), EscapeLevel::Soft);
        assert_eq!(EscapeLevel::from_code(1), EscapeLevel::Medium);
        assert_eq!(EscapeLevel::from_code(2), EscapeLevel::Hard);
        // 상한을 넘는 코드도 Hard로 포화한다
        assert_eq!(EscapeLevel::from_code(7), EscapeLevel::Hard);
        for code in 0..=ESCAPE_LEVEL_MAX {
            assert_eq!(EscapeLevel::from_code(code).code(), code);
        }
    }

    #[test]
    fn escape_level_next_saturates_at_documented_max() {
        // escape_level = min(escape_level + 1, 2)
        assert_eq!(EscapeLevel::Soft.next(), EscapeLevel::Medium);
        assert_eq!(EscapeLevel::Medium.next(), EscapeLevel::Hard);
        assert_eq!(EscapeLevel::Hard.next(), EscapeLevel::Hard);
        assert_eq!(EscapeLevel::Hard.code(), ESCAPE_LEVEL_MAX);
    }

    #[test]
    fn soft_target_count_spans_document_range() {
        // 대상: 1~2개 패턴
        assert_eq!(soft_target_count(0.0), L1_TARGET_MIN);
        assert_eq!(soft_target_count(1.0), L1_TARGET_MAX);
        assert_eq!(soft_target_count(-5.0), L1_TARGET_MIN);
        assert_eq!(soft_target_count(5.0), L1_TARGET_MAX);
        let mid = soft_target_count(0.5);
        assert!((L1_TARGET_MIN..=L1_TARGET_MAX).contains(&mid));
    }

    #[test]
    fn tilt_ladder_widens_with_escape_stage() {
        // ±1° → ±3° → ±5°
        assert_eq!(tilt_for_intensity(0.0), 1.0);
        assert_eq!(tilt_for_intensity(0.5), 3.0);
        assert_eq!(tilt_for_intensity(1.0), 5.0);
        assert_eq!(tilt_for_intensity(2.0), 5.0);
        assert!(tilt_for_intensity(0.0) <= tilt_for_intensity(0.99));
    }

    #[test]
    fn discrete_rotation_stays_inside_allowed_set() {
        // 의류 패턴의 0°/180°: 현재 0이면 180°(π)만 후보다
        let set = [0.0, PI];
        assert_eq!(pick_discrete_rotation(&set, 0.0), Some(PI));
        assert_eq!(pick_discrete_rotation(&set, PI), Some(0.0));
        // 후보가 하나뿐이면 회전할 곳이 없다
        assert_eq!(pick_discrete_rotation(&[0.0], 0.0), None);
        assert_eq!(pick_discrete_rotation(&[], 0.0), None);
    }

    #[test]
    fn hard_keep_boundary_matches_document_ratio() {
        // 재구성: 오른쪽 15~30% 영역 → 보존 경계 = L * (1 - r)
        let l = 100.0;
        assert!((hard_keep_boundary(l, 0.0) - 85.0).abs() < 1.0e-4);
        assert!((hard_keep_boundary(l, 1.0) - 70.0).abs() < 1.0e-4);
        let mid = hard_keep_boundary(l, 0.5);
        assert!((70.0..=85.0).contains(&mid), "{mid}");
    }

    #[test]
    fn medium_target_ratio_matches_document_range() {
        // 대상: 전체 패턴의 약 5~10%
        assert!((lerp_ratio((L2_RATIO_MIN, L2_RATIO_MAX), 0.0) - 0.05).abs() < 1.0e-6);
        assert!((lerp_ratio((L2_RATIO_MIN, L2_RATIO_MAX), 1.0) - 0.10).abs() < 1.0e-6);
    }

    #[test]
    fn neighbour_halo_uses_quarter_of_the_diagonal() {
        // 3 × 4 bbox → 대각선 5 → 팽창량 1.25
        let halo = expanded_bbox(rect(0.0, 0.0, 3.0, 4.0));
        assert!((halo.x_min + 1.25).abs() < 1.0e-5, "{halo:?}");
        assert!((halo.x_max - 4.25).abs() < 1.0e-5, "{halo:?}");
        assert!((halo.y_min + 1.25).abs() < 1.0e-5, "{halo:?}");
        assert!((halo.y_max - 5.25).abs() < 1.0e-5, "{halo:?}");
    }

    #[test]
    fn halo_catches_only_genuinely_adjacent_patterns() {
        let bottleneck = rect(0.0, 0.0, 3.0, 4.0);
        let halo = expanded_bbox(bottleneck);
        // 팽창 영역 안(오른쪽 1.0mm 떨어짐)
        assert!(rects_overlap(halo, rect(4.0, 0.0, 6.0, 4.0)));
        // 팽창 영역 밖(오른쪽 2.0mm 떨어짐 > 1.25)
        assert!(!rects_overlap(halo, rect(5.0, 0.0, 7.0, 4.0)));
        // 자기 자신은 당연히 겹친다
        assert!(rects_overlap(halo, bottleneck));
    }

    #[test]
    fn step_towards_is_capped_by_max_length() {
        let full = step_towards((0.0, 0.0), (3.0, 4.0), 5.0);
        assert!((full.0 - 3.0).abs() < 1.0e-5 && (full.1 - 4.0).abs() < 1.0e-5);
        let capped = step_towards((0.0, 0.0), (3.0, 4.0), 2.5);
        assert!((capped.0 - 1.5).abs() < 1.0e-5 && (capped.1 - 2.0).abs() < 1.0e-5);
        // 이미 같은 자리면 이동하지 않는다
        assert_eq!(step_towards((1.0, 1.0), (1.0, 1.0), 5.0), (0.0, 0.0));
        assert_eq!(step_towards((0.0, 0.0), (3.0, 4.0), 0.0), (0.0, 0.0));
    }

    #[test]
    fn step_away_pushes_outwards_from_the_bottleneck() {
        let away = step_away((0.0, 0.0), (3.0, 4.0), 10.0);
        assert!((away.0 - 6.0).abs() < 1.0e-5 && (away.1 - 8.0).abs() < 1.0e-5);
        // 중심이 겹치면 +x로 민다
        assert_eq!(step_away((2.0, 2.0), (2.0, 2.0), 4.0), (4.0, 0.0));
        assert_eq!(step_away((0.0, 0.0), (1.0, 0.0), 0.0), (0.0, 0.0));
    }

    #[test]
    fn clamp_step_keeps_the_pattern_inside_the_container() {
        let container = rect(0.0, 0.0, 100.0, 50.0);
        let bbox = rect(10.0, 10.0, 20.0, 20.0);
        // 오른쪽으로 200mm 요구 → x_max가 컨테이너에 닿는 80mm까지만
        let (dx, dy) = clamp_step(bbox, container, (200.0, -30.0));
        assert!((dx - 80.0).abs() < 1.0e-5, "{dx}");
        assert!((dy + 10.0).abs() < 1.0e-5, "{dy}");
        // 컨테이너보다 큰 축은 아예 움직이지 않는다
        let oversized = rect(-5.0, 0.0, 105.0, 50.0);
        assert_eq!(clamp_step(oversized, container, (5.0, 5.0)).0, 0.0);
    }

    #[test]
    fn centre_and_diagonal_are_plain_geometry() {
        let r = rect(2.0, 4.0, 8.0, 12.0);
        let c = centre_of(r);
        assert!((c.0 - 5.0).abs() < 1.0e-5 && (c.1 - 8.0).abs() < 1.0e-5);
        assert!((diagonal_of(r) - 10.0).abs() < 1.0e-5);
    }

    #[test]
    fn empty_outcome_is_not_applied_and_untouched() {
        let outcome = EscapeOutcome::empty();
        assert!(!outcome.applied);
        assert!(!outcome.touched());
        let touched = EscapeOutcome {
            n_relocated: 1,
            ..EscapeOutcome::empty()
        };
        assert!(touched.touched());
    }
}
