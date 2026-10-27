//! 원단 경계를 **원본** 외곽선 기준으로 검사하고, 넘은 조각만 안쪽으로 민다.
//!
//! # 왜 필요한가
//!
//! 겹침 `d` 를 허용하면 각 조각의 충돌 도형이 `r = d/2` 만큼 침식되고, 컨테이너도
//! [`crate::util::io`] 가 같은 양만큼 줄인다. 그래도 **원본** 외곽선은 밖으로 나간다 —
//! 침식은 변을 `r` 만큼 들이지만 볼록 꼭짓점은 각의 이등분선을 따라 `r/sin(θ/2)` 만큼
//! 들이기 때문이다. 침식된 도형이 벽에 닿는 순간 원본 꼭짓점은
//!
//! ```text
//! r · (1/sin(θ/2) − 1)
//! ```
//!
//! 만큼 벽을 넘고, 이 값은 각이 날카로울수록 한없이 커진다. [`crate::original_overlap`]
//! 의 가드는 **쌍(pair) 전용**이라 이 초과를 전혀 보지 않는다.
//!
//! # 왜 컨테이너를 더 줄이지 않는가
//!
//! 그 방법도 있고 [`crate::util::io`] 에 구현되어 있다. 하지만 인셋은 **가장 날카로운
//! 꼭짓점 하나**가 원반 전체를 줄인다. 여성WSjk 156-패널 실측: 최소 꼭짓각 31.7° 라
//! 예산 0.1mm 를 지키려면 사방 0.965mm 를 더 줄여야 하는데, 그렇게 날카로운 꼭짓점은
//! 볼록 꼭짓점 17,048개 중 592개뿐이고 실제로 벽에 닿는 것은 그보다 훨씬 적다.
//! 균일 인셋의 측정 비용은 최종 효율 **−1.5%p** 였다(면적 손실만 보면 0.18% 인데도).
//!
//! 실제 위반은 작고 드물다 — 같은 실측에서 최악 0.115mm, 조각 몇 개다. 그래서 여기서는
//! 넘은 조각만 필요한 만큼 민다.
//!
//! # 계약
//!
//! 보정은 **겹침 계약을 절대 깨지 않는다.** 옮긴 자리에서 이웃과의 원본 겹침이 허용치를
//! 넘으면 그 이동은 채택하지 않는다. 채택하는 이동은 인증기와 같은 기준(충돌 도형의 충돌·
//! 컨테이너 포함)도 통과해야 한다. 조각은 안쪽으로만 민다. 밀어 넣지 못한 **우측 끝** 초과만
//! 스트립 길이를 그만큼 늘려 없앤다 — 우측 끝은 원단 가장자리가 아니라 마커 길이의 끝이다.
//! 고정(frozen)·그룹 조각은 건드리지 않고 잔여 위반으로 보고한다.

use crate::original_overlap::OriginalOverlapGuard;
use crate::util::terminator::Terminator;
use jagua_rs::collision_detection::hazards::BasicHazardEntity;
use jagua_rs::collision_detection::hazards::filter::NoFilter;
use jagua_rs::collision_detection::hazards::collector::BasicHazardCollector;
use jagua_rs::entities::{PItemKey, PlacedItem};
use jagua_rs::geometry::DTransformation;
use jagua_rs::probs::spp::entities::{SPPlacement, SPProblem};

/// 한 조각이 원단 밖으로 나간 정도. 부호 없는 거리(mm)다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EdgeOverrun {
    pub item_id: usize,
    pub left: f64,
    pub right: f64,
    pub bottom: f64,
    pub top: f64,
}

impl EdgeOverrun {
    pub fn worst(&self) -> f64 {
        self.left.max(self.right).max(self.bottom).max(self.top)
    }
    /// 예산 안으로 들어가기 위한 최소 평행이동. 한 조각이 마주보는 두 벽을 동시에
    /// 넘는 경우(= 조각이 원단보다 큼)는 이동으로 풀 수 없으므로 0 을 돌려준다.
    fn shift(&self, budget: f64) -> (f64, f64) {
        let need_right = (self.left - budget).max(0.0);
        let need_left = (self.right - budget).max(0.0);
        let need_up = (self.bottom - budget).max(0.0);
        let need_down = (self.top - budget).max(0.0);
        let dx = if need_right > 0.0 && need_left > 0.0 {
            0.0
        } else {
            need_right - need_left
        };
        let dy = if need_up > 0.0 && need_down > 0.0 {
            0.0
        } else {
            need_up - need_down
        };
        (dx, dy)
    }
}

/// 보정 결과 요약. 호출자가 로그·인증에 쓴다.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RepairOutcome {
    /// 예산을 넘은 조각 수 (보정 전).
    pub violations: usize,
    /// 채택한 이동의 수. 조각 수가 아니다 — 한 번의 이동이 이웃 여럿을 함께 들이기도 하고,
    /// 일부만 들어간 조각은 새 자세에서 다시 밀리기도 한다.
    pub repaired: usize,
    /// 보정 전 최악 초과량(mm).
    pub worst_before: f64,
    /// 보정 후 최악 초과량(mm). 0 이면 계약 충족.
    pub worst_after: f64,
    /// 우측 끝의 잔여 초과를 없애려고 늘린 스트립 길이(mm). 0 이면 늘리지 않았다.
    pub widened: f64,
}

impl RepairOutcome {
    pub fn is_clean(&self) -> bool {
        self.worst_after <= 0.0
    }
}

/// 이 조각을 옮겨도 되는가. 고정·그룹 조각은 옮기면 다른 계약이 깨진다.
pub type Movable<'a> = &'a dyn Fn(usize) -> bool;

/// 현재 배치에서 각 조각이 원단 밖으로 나간 정도.
///
/// `budget` 과 무관하게 **모든** 초과를 돌려준다. 판정은 호출자가 한다.
pub fn inspect(prob: &SPProblem) -> Result<Vec<(PItemKey, EdgeOverrun)>, String> {
    overruns(prob, &OriginalOverlapGuard::new(&prob.instance()))
}

fn overruns(
    prob: &SPProblem,
    guard: &OriginalOverlapGuard,
) -> Result<Vec<(PItemKey, EdgeOverrun)>, String> {
    let (w, h) = (
        f64::from(prob.strip_width()),
        f64::from(prob.instance().base_strip.fixed_height),
    );
    let mut out = Vec::new();
    for (pk, placed) in prob.layout().placed_items().iter() {
        let o = guard.outline(placed.item().idx(), placed.d_transf())?;
        let b = o.bounds();
        let over = EdgeOverrun {
            item_id: placed.item().idx(),
            left: (-b[0]).max(0.0),
            bottom: (-b[1]).max(0.0),
            right: (b[2] - w).max(0.0),
            top: (b[3] - h).max(0.0),
        };
        if over.worst() > 0.0 {
            out.push((pk, over));
        }
    }
    Ok(out)
}

/// 원단 밖으로 `budget` 을 넘은 조각만 안쪽으로 민다.
///
/// 이동 뒤 이웃과의 **원본** 겹침이 허용치를 넘으면 그 이동은 채택하지 않고, 대신
/// 채택 가능한 최대 비율까지 이분해서 넣는다. 그래도 남으면 잔여 위반으로 보고한다.
/// 실패해도 배치는 언제나 유효한 상태로 남는다 — 채택한 이동만 반영되기 때문이다.
pub fn repair(
    prob: &mut SPProblem,
    budget: f32,
    movable: Movable<'_>,
    term: &impl Terminator,
) -> Result<RepairOutcome, String> {
    let mut outcome = RepairOutcome::default();
    if !budget.is_finite() || budget < 0.0 {
        return Ok(outcome);
    }
    let budget = f64::from(budget);
    let guard = OriginalOverlapGuard::new(&prob.instance());
    let initial: Vec<EdgeOverrun> = overruns(prob, &guard)?
        .into_iter()
        .map(|(_, o)| o)
        .filter(|o| o.worst() > budget)
        .collect();
    outcome.violations = initial.len();
    outcome.worst_before = initial.iter().map(|o| o.worst()).fold(0.0, f64::max);
    if initial.is_empty() {
        return Ok(outcome);
    }

    // **매 반복 위반 목록을 다시 만든다.** 조각을 옮기면(`remove_item` + `place_item`) 그
    // 조각의 `PItemKey` 가 바뀐다. 처음에 한 번 모은 목록을 돌면, 앞선 조각의 연쇄 이동에
    // 딸려 간 이웃이 자기 차례에 낡은 키로 조회돼 `placed_items[key]` 가 패닉했다 — 인접한
    // 두 조각이 함께 경계를 넘으면 매번 재현된다. 낡은 초과량으로 이미 들어간 조각을 한 번
    // 더 미는 문제도 같은 원인이었다.
    //
    // 같은 자세에서는 한 번만 시도한다. 옮겨지면 자세가 달라지므로, 일부만 들어간 조각은
    // 새 자세에서 다시 시도된다. 채택된 이동은 초과량을 엄격히 줄이지만 이분 탐색이 아주
    // 작은 걸음을 반복할 수 있어 반복 수에도 상한을 둔다.
    let mut tried: Vec<(usize, [u32; 3])> = Vec::new();
    for _ in 0..initial.len() * 2 + MAX_CASCADE {
        if term.kill() {
            break;
        }
        // 크게 넘은 것부터. 큰 이동이 먼저 자리를 잡아야 작은 이동이 그 결과 위에서
        // 판정된다 — 반대 순서면 큰 이동이 이미 옮긴 이웃에 막힌다.
        let next = overruns(prob, &guard)?
            .into_iter()
            .filter(|(pk, o)| {
                o.worst() > budget
                    && movable(o.item_id)
                    && o.shift(budget) != (0.0, 0.0)
                    && !tried.contains(&pose_id(prob, *pk))
            })
            .max_by(|a, b| a.1.worst().total_cmp(&b.1.worst()));
        let Some((pk, over)) = next else {
            break;
        };
        tried.push(pose_id(prob, pk));
        let (dx, dy) = over.shift(budget);
        // 부동소수 반올림으로 예산에 간신히 걸치는 일을 막는다.
        let (dx, dy) = (nudge(dx), nudge(dy));
        if let Some(moves) =
            largest_accepted_shift(prob, &guard, pk, dx, dy, budget, movable, term)?
        {
            // `moves` 의 키는 같은 스냅숏에서 나온 서로 다른 조각이라, 하나를 옮겨도 나머지 키는 유효하다.
            for (mk, dt) in moves {
                let item_id = prob.layout().placed_items()[mk].item().idx();
                prob.remove_item(mk);
                prob.place_item(SPPlacement { item_idx: item_id, d_transf: dt });
            }
            outcome.repaired += 1;
        }
    }
    // **우측 끝의 잔여 초과는 길이를 늘려서 없앤다.** 우측 끝은 원단의 물리적 가장자리가
    // 아니라 마커 길이의 끝이다 — 조각이 그 선을 넘었다는 것은 보고한 길이가 그만큼 짧다는
    // 뜻이므로, 안으로 밀 수 없으면 길이를 바로잡는 쪽이 맞다.
    //
    // 밀어 넣기는 조밀한 배치에서 자주 실패한다. 무리는 같은 벡터로만 움직이므로 이웃과의
    // 간격이 이동량보다 좁으면 이웃이 줄줄이 무리에 들어오고, `MAX_CASCADE` 를 넘으면 포기한다.
    // 실측(wsjk156, 폴리시 뒤): 0.34mm 넘은 조각 하나를 0.14mm 들이지 못했다. 늘어나는
    // 길이는 초과량에서 예산을 뺀 만큼이라 효율로는 0.003%p 아래다.
    // 좌·하·상단은 실제 원단 가장자리라 늘릴 수 없다 — 잔여 위반으로 보고한다.
    //
    // 옮길 수 없는 조각(고정·그룹)은 여기서도 뺀다. 고정 조각은 스트립 밖에 놓여도 되고
    // 폭의 하한이 아니다(`fit_strip_with_frozen` 이 같은 이유로 건너뛴다). 그것까지 세면
    // 스트립 밖의 고정 조각 하나가 매 단계 마커 길이를 그 조각의 우측 끝까지 늘린다.
    let right = overruns(prob, &guard)?
        .into_iter()
        .filter(|(_, o)| movable(o.item_id))
        .map(|(_, o)| o.right - budget)
        .fold(0.0, f64::max);
    if right > 0.0 && !term.kill() {
        let before = prob.strip_width();
        let grown = (f64::from(before) + nudge(right)) as f32;
        if grown.is_finite() && grown > before {
            crate::util::frozen::change_strip_width(prob, grown);
            outcome.widened = f64::from(grown - before);
        }
    }
    outcome.worst_after = overruns(prob, &guard)?
        .into_iter()
        .map(|(_, o)| (o.worst() - budget).max(0.0))
        .fold(0.0, f64::max);
    Ok(outcome)
}

/// 배치된 조각을 키가 아니라 «어느 조각이 어느 자세에 있는가» 로 식별한다.
/// 키는 조각을 옮길 때마다 바뀌고, 같은 `item_id` 가 여러 장 놓일 수도 있다.
fn pose_id(prob: &SPProblem, pk: PItemKey) -> (usize, [u32; 3]) {
    let placed = &prob.layout().placed_items()[pk];
    let (x, y) = placed.d_transf().translation();
    (
        placed.item().idx(),
        [placed.d_transf().rotation().to_bits(), x.to_bits(), y.to_bits()],
    )
}

/// 예산 밖으로 되돌아가지 않도록 이동량에 더하는 여유.
///
/// f32 좌표 한 눈금보다 커야 한다. 좌표가 수천 mm 면 한 눈금이 0.0005mm 라서 예전의 1e-4 는
/// 반올림에 묻혔고, 보정된 조각이 예산 선 위에 정확히 걸쳐 독립 검증기(예산에서 격자 여유
/// 0.0005mm 를 뺀 값을 쓴다)가 위반으로 읽었다.
const NUDGE_MM: f64 = 2.0e-3;

fn nudge(d: f64) -> f64 {
    if d > 0.0 {
        d + NUDGE_MM
    } else if d < 0.0 {
        d - NUDGE_MM
    } else {
        0.0
    }
}

fn shifted(dt: DTransformation, dx: f64, dy: f64) -> DTransformation {
    let (x, y) = dt.translation();
    DTransformation::new(dt.rotation(), (x + dx as f32, y + dy as f32))
}

/// 한 번에 밀 수 있는 조각 수의 상한. 경계 위반은 조각 몇 개짜리 국소 현상이므로
/// 이 이상으로 번지면 국소 보정이 아니라 재배치다 — 그때는 포기하는 편이 낫다.
const MAX_CASCADE: usize = 8;

/// `(dx, dy)` 를 넣는다. 이웃이 막으면 **그 이웃까지 같은 방향으로 함께 민다.**
///
/// 조각 하나만 밀면 옆 조각이 곧바로 벽 역할을 해서 마지막 수십 마이크로미터가
/// 남는다(실측: 0.1142mm 위반이 0.0142mm 까지만 줄었다). 같은 방향으로 함께 밀면
/// 그 사이 간격은 변하지 않으므로 계약이 깨지지 않고, 무리 전체가 안쪽으로 들어간다.
///
/// 전부 넣을 수 없으면 넣을 수 있는 최대 비율까지 이분한다. 이동은 안쪽 한 방향이라
/// 비율이 커질수록 벽에서는 안전해지고 바깥 이웃과는 위험해지므로 단조롭다.
#[allow(clippy::too_many_arguments)]
fn largest_accepted_shift(
    prob: &SPProblem,
    guard: &OriginalOverlapGuard,
    pk: PItemKey,
    dx: f64,
    dy: f64,
    budget: f64,
    movable: Movable<'_>,
    term: &impl Terminator,
) -> Result<Option<Vec<(PItemKey, DTransformation)>>, String> {
    if let Some(moves) = group_shift(prob, guard, pk, dx, dy, budget, movable)? {
        return Ok(Some(moves));
    }
    let (mut lo, mut hi) = (0.0f64, 1.0f64);
    let mut best = None;
    for _ in 0..12 {
        if term.kill() {
            break;
        }
        let mid = 0.5 * (lo + hi);
        match group_shift(prob, guard, pk, dx * mid, dy * mid, budget, movable)? {
            Some(moves) => {
                lo = mid;
                best = Some(moves);
            }
            None => hi = mid,
        }
    }
    Ok(best)
}

/// 위반 조각과, 그것을 막는 이웃들을 함께 `(dx, dy)` 만큼 민 결과. 계약을 지키면
/// 옮길 목록을, 아니면 `None` 을 돌려준다.
fn group_shift(
    prob: &SPProblem,
    guard: &OriginalOverlapGuard,
    seed: PItemKey,
    dx: f64,
    dy: f64,
    budget: f64,
    movable: Movable<'_>,
) -> Result<Option<Vec<(PItemKey, DTransformation)>>, String> {
    let (w, h) = (
        f64::from(prob.strip_width()),
        f64::from(prob.instance().base_strip.fixed_height),
    );
    let mut group = vec![seed];
    loop {
        // 무리 전체를 옮긴 뒤의 외곽선.
        let mut moved = Vec::with_capacity(group.len());
        for &mk in &group {
            // 낡은 키로 들어오면 패닉하지 않고 «옮길 수 없다» 로 답한다.
            let Some(placed) = prob.layout().placed_items().get(mk) else {
                return Ok(None);
            };
            let dt = shifted(placed.d_transf(), dx, dy);
            let outline = guard.outline(placed.item().idx(), dt)?;
            // 같이 밀린 조각이 반대편 벽을 넘으면 보정이 아니라 이동이다.
            //
            // 판정은 «이 이동이 어느 벽의 초과를 **새로 만들거나 키우는가**» 다. 예전에는
            // 이동 뒤에 예산을 넘는 벽이 하나라도 있으면 기각했는데, 그러면 부분 이동은 언제나
            // 기각된다 — 절반만 들어간 조각은 고치려던 바로 그 벽을 아직 넘고 있기 때문이다.
            // 그래서 `largest_accepted_shift` 의 이분 탐색은 한 번도 채택하지 못하고 12번을
            // 헛돌았다. 줄어드는 초과는 허용해야 부분 이동이 성립한다.
            let before = guard.outline(placed.item().idx(), placed.d_transf())?.bounds();
            let after = outline.bounds();
            let over = |b: &[f64; 4]| [-b[0], -b[1], b[2] - w, b[3] - h];
            let worsened = over(&after)
                .iter()
                .zip(over(&before))
                .any(|(after, before)| *after > budget && *after > before + 1.0e-9);
            if worsened {
                return Ok(None);
            }
            moved.push((mk, dt, outline));
        }
        // 무리 밖 조각과의 계약 검사. 무리 안끼리는 같은 벡터로 움직였으므로
        // 상대 위치가 그대로다 — 검사할 필요가 없다.
        let mut blockers = Vec::new();
        let reach = guard.reach();
        for (mk, dt, outline) in &moved {
            let ob = outline.bounds();
            for (other_pk, other) in prob.layout().placed_items().iter() {
                if group.contains(&other_pk) || other_pk == *mk {
                    continue;
                }
                // 충돌 도형 bbox 로 멀리 있는 조각을 먼저 거른다 — 원본 외곽선 변환과
                // 클리핑은 비싸고, `original_overlap` 의 사전 거르기와 같은 기준이다.
                let b = other.shape().bbox();
                if b.x_min as f64 - reach > ob[2]
                    || (b.x_max as f64) + reach < ob[0]
                    || b.y_min as f64 - reach > ob[3]
                    || (b.y_max as f64) + reach < ob[1]
                {
                    continue;
                }
                let o = guard.outline(other.item().idx(), other.d_transf())?;
                if !guard.outlines_allowed(outline, &o)? {
                    if !movable(other.item().idx()) {
                        return Ok(None);
                    }
                    if !blockers.contains(&other_pk) {
                        blockers.push(other_pk);
                    }
                }
            }
            // **인증기와 같은 기준도 본다.** 인증은 원본 외곽선의 겹침 계약에 더해 충돌
            // 도형(`shape_cd`)끼리의 충돌과 컨테이너 포함을 요구한다. 충돌 도형은 단순화로
            // 원본보다 엄격한 구간이 있어, 원본 계약만 보고 옮기면 보정본이 인증에 떨어진다
            // (실측 wsjk156: 우측 벽을 0.34mm 넘은 조각을 0.14mm 들였더니 옆 조각과 충돌
            // 도형이 겹쳐 Compress·Polish·Reclaim 세 단계의 보정이 전부 기각됐다).
            // 걸리는 이웃은 같은 방향으로 함께 민다.
            let placed = &prob.layout().placed_items()[*mk];
            let item = prob.instance().item(placed.item().idx());
            let (hits, outside) = shape_collisions(prob, &PlacedItem::new(item, *dt).shape());
            // 옮겨서 **새로** 컨테이너를 벗어나면 인증에서 떨어진다.
            if outside && !shape_collisions(prob, &placed.shape()).1 {
                return Ok(None);
            }
            for other_pk in hits {
                if group.contains(&other_pk) || other_pk == *mk {
                    continue;
                }
                if !movable(prob.layout().placed_items()[other_pk].item().idx()) {
                    return Ok(None);
                }
                if !blockers.contains(&other_pk) {
                    blockers.push(other_pk);
                }
            }
        }
        if blockers.is_empty() {
            return Ok(Some(moved.into_iter().map(|(mk, dt, _)| (mk, dt)).collect()));
        }
        if group.len() + blockers.len() > MAX_CASCADE {
            return Ok(None);
        }
        group.extend(blockers);
    }
}

/// 충돌 도형 `shape` 가 현재 배치에서 부딪히는 조각들과, 컨테이너(또는 품질 구역)를
/// 벗어나는지 여부.
fn shape_collisions(
    prob: &SPProblem,
    shape: &jagua_rs::geometry::primitives::SPolygon,
) -> (Vec<PItemKey>, bool) {
    let mut collector = BasicHazardCollector::with_capacity(8);
    prob.layout().cde().collect_poly_collisions(shape, &NoFilter, &mut collector);
    let mut items = Vec::new();
    let mut outside = false;
    for (_, hazard) in collector.iter() {
        match hazard {
            BasicHazardEntity::PlacedItem { pk, .. } => items.push(*pk),
            _ => outside = true,
        }
    }
    (items, outside)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::DEFAULT_SWARM_CONFIG;
    use crate::util::terminator::BasicTerminator;

    /// 4x4 정사각형 `n` 개, 원단 40 x 20.
    fn problem(n: usize) -> SPProblem {
        let items: Vec<_> = (0..n).map(|i| serde_json::json!({
            "id": i, "demand": 1, "allowed_orientations": [0.0],
            "shape": {"type": "rectangle", "data": {"x_min": 0.0, "y_min": 0.0, "width": 4.0, "height": 4.0}}
        })).collect();
        let ext = serde_json::from_value(
            serde_json::json!({"name": "edge", "strip_height": 20.0, "items": items}),
        )
        .unwrap();
        let importer = crate::util::io::new_importer(DEFAULT_SWARM_CONFIG.cde_config, None, None, None);
        let inst = jagua_rs::probs::spp::io::import_instance(&importer, &ext).unwrap();
        let mut prob = SPProblem::new(inst).expect(crate::dmsg!("strip forms a valid container"));
        prob.change_strip_width(40.0).expect(crate::dmsg!("strip width must be positive"));
        prob
    }

    /// 원본 외곽선의 좌하단이 `(left, bottom)` 에 오도록 놓는다.
    fn place(prob: &mut SPProblem, item_id: usize, left: f32, bottom: f32) {
        let guard = OriginalOverlapGuard::new(&prob.instance());
        let b = guard
            .outline(item_id, DTransformation::new(0.0, (0.0, 0.0)))
            .unwrap()
            .bounds();
        prob.place_item(SPPlacement {
            item_idx: item_id,
            d_transf: DTransformation::new(0.0, (left - b[0] as f32, bottom - b[1] as f32)),
        });
    }

    /// **낡은 키 회귀.** 모서리의 A 가 왼쪽·아래 벽을 넘고, 바로 오른쪽의 B 가 아래 벽을 넘는다.
    /// A 를 안으로 밀면 B 가 막아서 함께 밀리고(키가 바뀐다), 그 이동으로 B 도 예산 안에
    /// 들어온다. 예전에는 처음에 모아 둔 목록에서 B 차례가 오면 낡은 키로 조회해 패닉했다.
    #[test]
    fn a_neighbour_dragged_along_by_an_earlier_repair_is_not_looked_up_by_its_old_key() {
        let mut prob = problem(2);
        place(&mut prob, 0, -0.11, -0.09);
        place(&mut prob, 1, 3.891, -0.08);
        let budget = 0.05f32;
        let before = inspect(&prob).unwrap();
        assert_eq!(before.iter().filter(|(_, o)| o.worst() > 0.05).count(), 2);

        let outcome = repair(&mut prob, budget, &|_| true, &BasicTerminator::new())
            .expect(crate::dmsg!("보정이 오류 없이 끝나야 한다"));
        assert_eq!(outcome.violations, 2);
        // 이동은 한 번뿐이다 — B 는 A 에 딸려 가서 들어왔지, 따로 보정된 것이 아니다.
        // (이 값이 2 면 연쇄가 일어나지 않은 것이고, 이 테스트는 회귀를 지키지 못한다.)
        assert_eq!(outcome.repaired, 1, "{outcome:?}");
        assert!(outcome.is_clean(), "{outcome:?}");
        assert_eq!(prob.layout().placed_items().len(), 2);
        for (_, o) in inspect(&prob).unwrap() {
            assert!(o.worst() <= f64::from(budget), "{o:?}");
        }
    }

    /// 옮길 수 없는(고정) 조각의 위반은 그대로 두고 잔여로 보고한다.
    #[test]
    fn an_immovable_violation_is_reported_as_a_residual() {
        let mut prob = problem(1);
        place(&mut prob, 0, -0.2, 5.0);
        let outcome = repair(&mut prob, 0.05, &|_| false, &BasicTerminator::new()).unwrap();
        assert_eq!((outcome.violations, outcome.repaired), (1, 0));
        assert!(!outcome.is_clean());
        assert!((outcome.worst_after - 0.15).abs() < 1e-3, "{outcome:?}");
        assert_eq!(outcome.widened, 0.0, "왼쪽 벽은 실제 원단 가장자리라 늘릴 수 없다");
        assert_eq!(prob.strip_width(), 40.0);
    }

    /// 우측 끝을 넘은 조각을 밀어 넣을 수 없으면 길이를 초과량에서 예산을 뺀 만큼 늘린다.
    /// 조각 0 은 옮길 수 있지만 바로 왼쪽의 고정 조각 1 에 막혀 들어가지 못한다.
    #[test]
    fn a_right_end_overrun_that_cannot_be_pushed_in_lengthens_the_strip() {
        let mut prob = problem(2);
        place(&mut prob, 0, 36.3, 5.0);
        place(&mut prob, 1, 32.3, 5.0);
        let outcome = repair(&mut prob, 0.05, &|id| id == 0, &BasicTerminator::new()).unwrap();
        assert_eq!((outcome.violations, outcome.repaired), (1, 0), "{outcome:?}");
        assert!(outcome.is_clean(), "{outcome:?}");
        assert!((outcome.widened - 0.25).abs() < 0.01, "{outcome:?}");
        assert!((prob.strip_width() - 40.25).abs() < 0.01);
        assert_eq!(prob.layout().placed_items().len(), 2);
    }

    /// 스트립 밖에 놓인 **고정** 조각은 마커 길이를 늘리지 않는다. 고정 조각은 폭의 하한이
    /// 아니다 — 그것까지 세면 매 단계의 공개 직전에 길이가 그 조각의 우측 끝까지 늘어난다.
    #[test]
    fn a_frozen_piece_past_the_right_end_does_not_lengthen_the_strip() {
        let mut prob = problem(1);
        place(&mut prob, 0, 36.3, 5.0);
        let outcome = repair(&mut prob, 0.05, &|_| false, &BasicTerminator::new()).unwrap();
        assert_eq!((outcome.violations, outcome.repaired), (1, 0));
        assert_eq!(outcome.widened, 0.0, "{outcome:?}");
        assert_eq!(prob.strip_width(), 40.0);
        assert!((outcome.worst_after - 0.25).abs() < 1e-3, "{outcome:?}");
    }

    /// 전부 넣을 수 없으면 **넣을 수 있는 만큼** 넣는다. 조각 0 은 왼쪽 벽을 0.3 넘었고,
    /// 오른쪽 0.1 떨어진 곳에 고정 조각 1 이 있다. 0.25 를 다 들이면 겹치지만 0.1 까지는
    /// 들일 수 있다. 예전에는 «이동 뒤에도 예산을 넘는 벽이 있으면 기각» 이라 부분 이동이
    /// 언제나 기각됐고, 이분 탐색은 한 번도 채택하지 못했다.
    #[test]
    fn a_blocked_repair_still_moves_the_piece_as_far_as_it_can() {
        let mut prob = problem(2);
        place(&mut prob, 0, -0.3, 5.0);
        place(&mut prob, 1, 3.8, 5.0);
        let outcome = repair(&mut prob, 0.05, &|id| id == 0, &BasicTerminator::new()).unwrap();
        assert_eq!(outcome.violations, 1);
        assert!(outcome.repaired >= 1, "부분 이동이 채택되지 않았다: {outcome:?}");
        // 0.3 을 넘었다가 0.1 쯤 들어갔으므로 예산(0.05) 초과분은 0.15 근처다.
        assert!(
            outcome.worst_after > 0.13 && outcome.worst_after < 0.17,
            "{outcome:?}"
        );
        assert_eq!(outcome.widened, 0.0, "왼쪽 벽은 늘릴 수 없다");
        // 고정 조각은 그대로다.
        let fixed = prob.layout().placed_items().values().find(|p| p.item().idx() == 1).unwrap();
        let guard = OriginalOverlapGuard::new(&prob.instance());
        assert!((guard.outline(1, fixed.d_transf()).unwrap().bounds()[0] - 3.8).abs() < 1e-4);
    }

    /// **저장된 해 위에서 보정 → 인증.** 실제 실행에서 보정본이 인증에 떨어진 해를 넣어 확인한다.
    ///
    /// 노브: `AB_SOLUTION`(필수), `AB_INPUT`(기본 wsjk156), `AB_OVERLAP_MM`(기본 0.4),
    /// `AB_EDGE_MARGIN_MM`(기본 0.2).
    #[test]
    #[ignore]
    fn repair_then_certify_on_a_saved_layout() {
        let Ok(solution_path) = std::env::var("AB_SOLUTION") else {
            println!("[EDGE] AB_SOLUTION missing; skipped");
            return;
        };
        let env = |key: &str, default: &str| std::env::var(key).unwrap_or_else(|_| default.into());
        let input = std::env::var("AB_INPUT")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|_| {
                std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input/wsjk156.json")
            });
        let overlap: f32 = env("AB_OVERLAP_MM", "0.4").parse().unwrap();
        let margin: f32 = env("AB_EDGE_MARGIN_MM", "0.2").parse().unwrap();
        let (ext, _) = crate::util::io::read_spp_input(&input).unwrap();
        let c = DEFAULT_SWARM_CONFIG;
        let instance = crate::util::io::import_spp_instance(
            &ext,
            c.cde_config,
            c.poly_simpl_tolerance,
            (overlap > 0.0).then_some(-overlap),
            c.narrow_concavity_cutoff_ratio,
        )
        .unwrap();
        let ext_sol: jagua_rs::probs::spp::io::ext_repr::ExtSPSolution =
            serde_json::from_str(&std::fs::read_to_string(&solution_path).unwrap()).unwrap();
        let mut prob = crate::util::frozen::import_solution(&instance, &ext_sol);
        let before = prob.save();
        assert!(
            crate::structure::pipeline::solution_validation_error(&instance, &before).is_none(),
            "입력 배치가 인증되지 않는다"
        );
        let outcome = repair(&mut prob, margin, &|_| true, &BasicTerminator::new()).unwrap();
        let error = crate::structure::pipeline::solution_validation_error(&instance, &prob.save());
        println!("[EDGE] {outcome:?}  보정 뒤 인증: {}  ({error:?})", error.is_none());
        assert!(error.is_none(), "보정본이 인증에 떨어진다");
        assert!(outcome.worst_after <= 1.0e-3, "{outcome:?}");
    }
}