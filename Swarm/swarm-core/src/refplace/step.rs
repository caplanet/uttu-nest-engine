//! 모듈 6 — 배치 스텝.
//!
//! 설계 §모듈 6 을 그대로 옮긴다.
//!
//! ```text
//!   진행 가능 검사
//!   피스 순회:
//!     부착 대상 있음 -> NFP 이진탐색 조회
//!       무효(정점 0 또는 전체겹침) -> 부착 해제
//!       대상 미배치 + 보류 사유 있음 -> 보류 체인 등록
//!         (NFP 판독 상태 코드 분기: 1/2/3 = 좌표 배가 처리, 4 = 특수 처리)
//!       정상 -> NFP 경계 배치 (모듈 7)
//!     부착 대상 없음 -> 다음 피스
//! ```
//!
//! # 이 스텝은 **계산하지 않는다**
//!
//! 설계가 "배치 시점에는 NFP 를 이진탐색으로 **조회만** 한다" 고 못박는다. 여기서
//! NFP 를 만들기 시작하면 체인 선계산의 의미가 사라지고, 배치 한 번의 비용이 예측
//! 불가능해진다. 조회에 실패하면 계산하는 것이 아니라 **부착을 해제**한다.

use crate::refplace::chain::{NfpLookup, NfpPairIndex, NfpPairKey, PoseKey};
use crate::refplace::collect::defer;
use crate::refplace::record::{PieceRecord, PlaceState, RotationEntry};

// ────────────────────────────────────────────────────────────────────────────
// NFP 판독 상태 코드 (§모듈 6)
// ────────────────────────────────────────────────────────────────────────────

/// NFP 판독 상태 코드.
///
/// 설계 §모듈 6 이 "1/2/3 = 좌표 배가 처리, 4 = 특수 처리" 라고만 적는다. 원 엔진의
/// 내부 코드값이라 의미가 문서에 없으므로, **관측된 동작만** 타입으로 남긴다 —
/// 추측한 의미를 이름에 박으면 나중에 그 추측이 사실처럼 굳는다.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReadCode {
    /// 코드 1·2·3 — 좌표 배가 처리가 필요한 판독.
    ///
    /// "좌표 배가" 는 NFP 경계 좌표를 그대로 쓰지 않고 배율을 적용해야 한다는 뜻으로
    /// 읽힌다. 세 코드가 서로 무엇이 다른지는 문서에 없다.
    ScaledCoords(u8),
    /// 코드 4 — 특수 처리.
    Special,
    /// 그 밖(0 포함) — 배율 없이 그대로 쓴다.
    Plain,
}

impl ReadCode {
    /// 원 엔진의 정수 코드에서.
    pub fn from_raw(code: u8) -> Self {
        match code {
            1..=3 => Self::ScaledCoords(code),
            4 => Self::Special,
            _ => Self::Plain,
        }
    }

    /// 좌표에 배가 처리가 필요한가.
    pub fn needs_coord_scaling(&self) -> bool {
        matches!(self, Self::ScaledCoords(_))
    }
}

/// 선계산된 NFP 하나에 대해 이 스텝이 알아야 할 것.
///
/// 실제 기하는 들고 오지 않는다 — 모듈 6 의 일은 **분기**이고, 분기에 필요한 것은
/// 정점 수와 겹침 여부와 판독 코드뿐이다. 그래야 이 모듈이 기하 커널 없이 테스트된다.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NfpSummary {
    /// 경계 정점 수. `0` 이면 붙을 자리가 없다.
    pub vertex_count: usize,
    /// 전체 겹침 — 어느 자리에도 못 놓는다.
    pub full_overlap: bool,
    /// 판독 상태 코드.
    pub code: ReadCode,
}

impl NfpSummary {
    /// 정상 판독.
    pub fn ready(vertex_count: usize) -> Self {
        Self {
            vertex_count,
            full_overlap: false,
            code: ReadCode::Plain,
        }
    }

    /// 이 판독이 배치로 갈 수 있는가.
    pub fn lookup(&self) -> NfpLookup {
        if self.full_overlap {
            NfpLookup::FullOverlap
        } else if self.vertex_count == 0 {
            NfpLookup::Empty
        } else {
            NfpLookup::Ready
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 스텝 결과
// ────────────────────────────────────────────────────────────────────────────

/// 피스 하나에 대한 스텝 판정.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepOutcome {
    /// 부착 대상이 없다 — 독립 배치로 간다. 루트가 여기 해당한다.
    Independent,
    /// 정상 — 모듈 7(NFP 경계 배치)로 간다.
    Place {
        /// 조회에 성공한 쌍.
        key: NfpPairKey,
        /// 판독 코드. 모듈 7 이 좌표를 어떻게 읽을지 정한다.
        code: ReadCode,
    },
    /// 부착 해제 — 무효(정점 0 / 전체 겹침 / 색인에 없음)였다.
    ///
    /// 해제된 피스는 **독립 배치 후보로 남는다.** 버리지 않는다.
    Detached(NfpLookup),
    /// 보류 체인 등록 — 대상이 아직 안 놓였다.
    Deferred,
    /// 이미 놓였다 — 아무 일도 하지 않는다.
    AlreadyPlaced,
}

/// 진행 가능 검사 (§모듈 6 첫 줄).
///
/// 설계는 "라이선스 포함" 이라고 적지만 이 구현에는 라이선스 개념이 없다. 남는 것은
/// **취소 요청**이다 — 그것을 확인하지 않으면 긴 배치가 멈추지 않는다.
pub trait StepGuard {
    /// 계속 진행해도 되는가.
    fn may_proceed(&self) -> bool;
}

/// 언제나 진행. 테스트와 취소가 없는 호출부용.
#[derive(Clone, Copy, Debug, Default)]
pub struct AlwaysProceed;

impl StepGuard for AlwaysProceed {
    fn may_proceed(&self) -> bool {
        true
    }
}

/// 피스 하나의 배치 스텝을 판정한다 (§모듈 6).
///
/// `index` 는 [`crate::refplace::chain::NfpPairPlan::precompute`] 가 만든 색인이다.
/// **여기서 새로 계산하지 않는다** — 조회만 한다.
///
/// `fixed_pose` 는 부착 대상이 실제로 놓인 자세다. 그것이 없으면(= 대상이 미배치면)
/// 보류로 간다.
pub fn step_piece(
    pieces: &mut [PieceRecord],
    piece: usize,
    moving_pose: &RotationEntry,
    index: &NfpPairIndex<NfpSummary>,
) -> StepOutcome {
    if pieces[piece].is_placed() {
        return StepOutcome::AlreadyPlaced;
    }

    // 부착 대상 없음 -> 다음 피스(독립 배치).
    let Some(target) = pieces[piece].attach_to else {
        return StepOutcome::Independent;
    };

    // 대상이 아직 안 놓였다 -> 보류 체인.
    //
    // 설계의 "대상 미배치 + 보류 사유 있음" 에서 보류 사유는 **대상이 미배치라는
    // 사실 자체**다. 별도 사유 플래그를 두면 그 플래그를 세우는 곳을 빠뜨렸을 때
    // 조각이 조용히 사라진다.
    let Some(target_pose) = pieces
        .get(target)
        .filter(|t| t.is_placed())
        .and_then(|t| t.pose)
    else {
        defer(&mut pieces[piece]);
        return StepOutcome::Deferred;
    };

    let key = NfpPairKey {
        fixed_item: pieces[target].item_id,
        fixed_pose: PoseKey::of(&target_pose.rotation),
        moving_item: pieces[piece].item_id,
        moving_pose: PoseKey::of(moving_pose),
    };

    // **이진탐색 조회만** 한다.
    let Some(summary) = index.get(&key) else {
        detach(&mut pieces[piece]);
        return StepOutcome::Detached(NfpLookup::Missing);
    };

    match summary.lookup() {
        NfpLookup::Ready => StepOutcome::Place {
            key,
            code: summary.code,
        },
        other => {
            detach(&mut pieces[piece]);
            StepOutcome::Detached(other)
        }
    }
}

/// 부착을 해제한다.
///
/// 해제된 피스는 **독립 배치 후보로 남는다** — `attach_to` 만 지우고 상태는
/// `Pending` 으로 되돌린다. 버리면(=`Failed`) 잔여 조각 처리가 다시 시도할 대상이
/// 사라진다.
pub fn detach(piece: &mut PieceRecord) {
    piece.attach_to = None;
    piece.predecessors.clear();
    if piece.state == PlaceState::Deferred {
        piece.state = PlaceState::Pending;
    }
}

/// 후보 순회 한 바퀴 (§모듈 6 "피스 순회").
///
/// `guard` 가 진행 불가를 알리면 **즉시 멈춘다** — 그때까지의 판정은 유효하다.
/// 부분 결과를 버리지 않는 것이 이 엔진 전체의 규칙이다.
pub fn step_all(
    pieces: &mut [PieceRecord],
    order: &[usize],
    moving_pose: &RotationEntry,
    index: &NfpPairIndex<NfpSummary>,
    guard: &impl StepGuard,
) -> Vec<(usize, StepOutcome)> {
    let mut out = Vec::with_capacity(order.len());
    for &p in order {
        if !guard.may_proceed() {
            break;
        }
        if p >= pieces.len() {
            continue;
        }
        let outcome = step_piece(pieces, p, moving_pose, index);
        out.push((p, outcome));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::refplace::chain::NfpPairPlan;
    use crate::refplace::record::{PlacedPose, RotationList};

    fn piece(item_id: usize) -> PieceRecord {
        PieceRecord::new(item_id, 0, RotationList::fixed())
    }

    fn placed(item_id: usize) -> PieceRecord {
        let mut p = piece(item_id);
        p.state = PlaceState::Placed;
        p.pose = Some(PlacedPose {
            rotation: RotationEntry::IDENTITY,
            extra_rad: 0.0,
            x: 0.0,
            y: 0.0,
        });
        p
    }

    fn index_with(entries: Vec<(NfpPairKey, NfpSummary)>) -> NfpPairIndex<NfpSummary> {
        let mut plan = NfpPairPlan::default();
        let mut sorted = entries;
        sorted.sort_by_key(|(k, _)| *k);
        plan.pairs = sorted.iter().map(|(k, _)| *k).collect();
        plan.center_distance = vec![0.0; plan.pairs.len()];
        let mut it = sorted.into_iter();
        plan.precompute(move |_| it.next().map(|(_, v)| v))
    }

    fn key(fixed: usize, moving: usize) -> NfpPairKey {
        let pose = PoseKey::of(&RotationEntry::IDENTITY);
        NfpPairKey {
            fixed_item: fixed,
            fixed_pose: pose,
            moving_item: moving,
            moving_pose: pose,
        }
    }

    /// 부착 대상이 없으면 독립 배치다.
    #[test]
    fn a_piece_with_no_target_is_placed_independently() {
        let mut pieces = vec![piece(0)];
        let index = index_with(Vec::new());
        assert_eq!(
            step_piece(&mut pieces, 0, &RotationEntry::IDENTITY, &index),
            StepOutcome::Independent
        );
    }

    /// 대상이 아직 안 놓였으면 **보류 체인**에 들어간다.
    #[test]
    fn an_unplaced_target_sends_the_piece_to_the_deferred_chain() {
        let mut pieces = vec![piece(0), piece(1)];
        pieces[1].attach_to = Some(0);
        let index = index_with(vec![(key(0, 1), NfpSummary::ready(8))]);
        assert_eq!(
            step_piece(&mut pieces, 1, &RotationEntry::IDENTITY, &index),
            StepOutcome::Deferred
        );
        assert_eq!(pieces[1].state, PlaceState::Deferred);
        // 부착은 유지된다 — 2차 패스가 같은 대상으로 재시도한다.
        assert_eq!(pieces[1].attach_to, Some(0));
    }

    /// 정점 0 이면 **부착 해제**다.
    #[test]
    fn an_empty_nfp_detaches_the_piece() {
        let mut pieces = vec![placed(0), piece(1)];
        pieces[1].attach_to = Some(0);
        pieces[1].predecessors = vec![0];
        let index = index_with(vec![(key(0, 1), NfpSummary::ready(0))]);
        assert_eq!(
            step_piece(&mut pieces, 1, &RotationEntry::IDENTITY, &index),
            StepOutcome::Detached(NfpLookup::Empty)
        );
        assert_eq!(pieces[1].attach_to, None);
        assert!(pieces[1].predecessors.is_empty());
    }

    /// 전체 겹침도 부착 해제다.
    #[test]
    fn a_fully_overlapping_nfp_detaches_the_piece() {
        let mut pieces = vec![placed(0), piece(1)];
        pieces[1].attach_to = Some(0);
        let index = index_with(vec![(
            key(0, 1),
            NfpSummary {
                vertex_count: 12,
                full_overlap: true,
                code: ReadCode::Plain,
            },
        )]);
        assert_eq!(
            step_piece(&mut pieces, 1, &RotationEntry::IDENTITY, &index),
            StepOutcome::Detached(NfpLookup::FullOverlap)
        );
    }

    /// **해제된 피스는 버려지지 않는다** — 독립 배치 후보로 남는다.
    #[test]
    fn detaching_leaves_the_piece_placeable_rather_than_failed() {
        let mut pieces = vec![placed(0), piece(1)];
        pieces[1].attach_to = Some(0);
        let index = index_with(vec![(key(0, 1), NfpSummary::ready(0))]);
        step_piece(&mut pieces, 1, &RotationEntry::IDENTITY, &index);
        assert_eq!(pieces[1].state, PlaceState::Pending);
        assert_ne!(pieces[1].state, PlaceState::Failed, "해제는 실패가 아니다");
        // 이제 독립 배치로 간다.
        assert_eq!(
            step_piece(&mut pieces, 1, &RotationEntry::IDENTITY, &index),
            StepOutcome::Independent
        );
    }

    /// 색인에 없는 쌍은 **계산하지 않고** 해제한다 — 조회만 한다는 계약이다.
    #[test]
    fn a_missing_pair_is_never_computed_on_the_fly() {
        let mut pieces = vec![placed(0), piece(1)];
        pieces[1].attach_to = Some(0);
        let index = index_with(Vec::new());
        assert_eq!(
            step_piece(&mut pieces, 1, &RotationEntry::IDENTITY, &index),
            StepOutcome::Detached(NfpLookup::Missing)
        );
    }

    /// 정상이면 판독 코드를 실어 모듈 7 로 넘긴다.
    #[test]
    fn a_valid_lookup_carries_the_read_code_to_the_boundary_step() {
        let mut pieces = vec![placed(0), piece(1)];
        pieces[1].attach_to = Some(0);
        let index = index_with(vec![(
            key(0, 1),
            NfpSummary {
                vertex_count: 32,
                full_overlap: false,
                code: ReadCode::from_raw(2),
            },
        )]);
        match step_piece(&mut pieces, 1, &RotationEntry::IDENTITY, &index) {
            StepOutcome::Place { key: k, code } => {
                assert_eq!(k, key(0, 1));
                assert_eq!(code, ReadCode::ScaledCoords(2));
                assert!(code.needs_coord_scaling());
            }
            other => panic!("정상 판독이어야 한다: {other:?}"),
        }
    }

    /// 판독 코드 분기가 문서대로인지 — 1/2/3 은 좌표 배가, 4 는 특수.
    #[test]
    fn the_read_codes_split_the_way_the_document_says() {
        assert_eq!(ReadCode::from_raw(1), ReadCode::ScaledCoords(1));
        assert_eq!(ReadCode::from_raw(3), ReadCode::ScaledCoords(3));
        assert_eq!(ReadCode::from_raw(4), ReadCode::Special);
        assert_eq!(ReadCode::from_raw(0), ReadCode::Plain);
        assert_eq!(ReadCode::from_raw(9), ReadCode::Plain);
        assert!(!ReadCode::Special.needs_coord_scaling());
    }

    /// 이미 놓인 조각은 건드리지 않는다.
    #[test]
    fn an_already_placed_piece_is_left_alone() {
        let mut pieces = vec![placed(0)];
        let index = index_with(Vec::new());
        assert_eq!(
            step_piece(&mut pieces, 0, &RotationEntry::IDENTITY, &index),
            StepOutcome::AlreadyPlaced
        );
    }

    /// 진행 불가면 **즉시 멈추되 그때까지의 판정은 유지**한다.
    #[test]
    fn the_guard_stops_the_sweep_without_discarding_what_was_done() {
        struct StopAfter(std::cell::Cell<usize>);
        impl StepGuard for StopAfter {
            fn may_proceed(&self) -> bool {
                let n = self.0.get();
                self.0.set(n + 1);
                n < 2
            }
        }
        let mut pieces = vec![piece(0), piece(1), piece(2), piece(3)];
        let index = index_with(Vec::new());
        let out = step_all(
            &mut pieces,
            &[0, 1, 2, 3],
            &RotationEntry::IDENTITY,
            &index,
            &StopAfter(std::cell::Cell::new(0)),
        );
        assert_eq!(out.len(), 2, "두 개까지만 처리하고 멈춘다");
        assert!(out.iter().all(|(_, o)| *o == StepOutcome::Independent));
    }
}
