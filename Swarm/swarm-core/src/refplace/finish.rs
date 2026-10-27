//! 모듈 9 — 검증·마무리.
//!
//! 설계 §모듈 9:
//!
//! ```text
//!   배치된 피스를 표면에 통합 (표면 타입별)
//!   표면 검증 — 실패 시 오류 표식
//!   2차 패스: 보류 체인 재시도
//!   전체 검증 후 효율 재계산 (배치·롤백·잔여조각 처리 후에도 동일하게 재계산)
//! ```
//!
//! # 효율은 **매 단계 끝마다** 다시 잰다
//!
//! 설계 §4-1 이 "각 단계(배치 완료·롤백·잔여조각 처리) 종료 시점마다 재계산한다" 고
//! 못박는다. 한 번 재고 들고 다니면 롤백 뒤에 낡은 값이 남고, 그 값으로 다음 판단을
//! 하게 된다. 재계산이 싸지 않다면 그건 재계산을 안 할 이유가 아니라 **효율 계산이
//! 비싸다는 결함**이다.

use crate::refplace::params::IMPROVE_TOLERANCE;

/// 검증에서 쓸 **기하** 허용오차(mm).
///
/// # 수렴 허용오차를 여기에 쓰면 안 된다
///
/// 처음에 [`IMPROVE_TOLERANCE`](crate::refplace::params::IMPROVE_TOLERANCE)(100)을
/// 그대로 썼다가 테스트가 잡았다. 그 값은 **개선 루프의 수렴 판정**용이고 단위가 길이
/// 정수다 — 기하 허용오차로 쓰면 100mm 겹침을 "겹치지 않았다" 로 보게 된다.
///
/// 이름이 비슷하다는 이유로 상수를 돌려 쓰는 것이 이 저장소가 반복해 밟은 함정이다.
/// 쓰임이 다르면 상수도 다르다.
///
/// 0.01mm 는 재단 공차(1mm 급)보다 두 자리 작고, f64 좌표 잡음보다는 훨씬 크다.
const VALIDATION_SLACK_MM: f64 = 0.01;
use crate::refplace::record::{PieceRecord, PlaceState, SurfaceRecord};
use crate::refplace::score::{clipped_area, efficiency};

// ────────────────────────────────────────────────────────────────────────────
// 표면 통합·검증
// ────────────────────────────────────────────────────────────────────────────

/// 표면 검증 실패 사유.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SurfaceError {
    /// 이 표면의 피스가 컨테이너 밖으로 나갔다.
    OutsideContainer,
    /// 두 피스가 겹친다.
    Overlap,
    /// 놓였다고 표시됐는데 자세가 없다 — 상태와 데이터가 어긋났다.
    MissingPose,
}

/// 표면 하나의 검증 결과.
#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceReport {
    /// 표면 인덱스.
    pub surface: usize,
    /// 통합된 피스 수.
    pub integrated: usize,
    /// 오류 표식 — 비어 있으면 통과.
    ///
    /// 실패해도 **배치를 버리지 않는다.** 설계가 "실패 시 오류 표식" 이라고만 적는다 —
    /// 표식을 남기고 진행한다. 부분 결과가 무결과보다 낫기 때문이다.
    pub errors: Vec<(usize, SurfaceError)>,
    /// 이 표면의 효율.
    pub efficiency: f64,
}

impl SurfaceReport {
    pub fn ok(&self) -> bool {
        self.errors.is_empty()
    }
}

/// 배치된 피스를 표면에 통합하고 검증한다 (§모듈 9).
///
/// `bbox_of` 는 피스의 월드 bbox `(x_min, y_min, x_max, y_max)` 를 준다.
/// `None` 이면 아직 안 놓인 것이다.
///
/// `current_width` 는 지금까지의 strip 폭 — 효율의 분모를 만든다(§4-1).
pub fn integrate_and_validate(
    pieces: &[PieceRecord],
    surface: &SurfaceRecord,
    surface_index: usize,
    current_width: f64,
    bbox_of: impl Fn(usize) -> Option<(f64, f64, f64, f64)>,
) -> SurfaceReport {
    let mut errors = Vec::new();
    let mut integrated = 0usize;
    let mut placed_area = 0.0f64;
    let (cw, ch) = surface.extent;

    let mut boxes: Vec<(usize, (f64, f64, f64, f64))> = Vec::new();
    for &p in &surface.pieces {
        let Some(piece) = pieces.get(p) else { continue };
        if !piece.is_placed() {
            continue;
        }
        if piece.pose.is_none() {
            errors.push((p, SurfaceError::MissingPose));
            continue;
        }
        let Some(bbox) = bbox_of(p) else {
            errors.push((p, SurfaceError::MissingPose));
            continue;
        };
        // 컨테이너 밖으로 나갔는가. **폭은 자라는 방향이라 보지 않는다** — strip packing
        // 에서 오른쪽으로 나가는 것은 위반이 아니라 폭이 늘어난 것이다. 높이와 좌·하단만 본다.
        if bbox.0 < -VALIDATION_SLACK_MM
            || bbox.1 < -VALIDATION_SLACK_MM
            || bbox.3 > ch + VALIDATION_SLACK_MM
        {
            errors.push((p, SurfaceError::OutsideContainer));
        }
        // §4-1: 스트립 경계를 넘어간 피스는 **넘은 만큼을 잘라서** 포함한다.
        placed_area += clipped_area(bbox, current_width.max(cw.min(current_width)), ch);
        boxes.push((p, bbox));
        integrated += 1;
    }

    // 겹침 — bbox 로 먼저 거른다. bbox 가 안 겹치면 도형도 안 겹친다(bbox 는 상위집합).
    // bbox 가 겹치는 것만으로는 겹침이 아니므로, 여기서는 **bbox 가 실질적으로 파고든**
    // 경우만 표식한다. 정확한 판정은 호출부의 CDE 가 한다.
    for i in 0..boxes.len() {
        for j in (i + 1)..boxes.len() {
            let (pi, a) = boxes[i];
            let (_, b) = boxes[j];
            let ox = (a.2.min(b.2) - a.0.max(b.0)).max(0.0);
            let oy = (a.3.min(b.3) - a.1.max(b.1)).max(0.0);
            if ox > VALIDATION_SLACK_MM && oy > VALIDATION_SLACK_MM {
                errors.push((pi, SurfaceError::Overlap));
            }
        }
    }

    SurfaceReport {
        surface: surface_index,
        integrated,
        errors,
        efficiency: efficiency(placed_area, surface.reference_area(current_width)),
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 2차 패스 (§모듈 9)
// ────────────────────────────────────────────────────────────────────────────

/// 2차 패스 결과.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct SecondPass {
    /// 재시도 대상이 된 피스들 (보류 체인).
    pub retried: Vec<usize>,
    /// 재시도 후에도 못 놓은 피스들 — 잔여 조각 처리(§모듈 10)로 간다.
    pub still_unplaced: Vec<usize>,
}

/// 보류 체인을 모은다 — 2차 패스가 재시도할 대상.
///
/// **한 번만 재시도한다.** 설계가 "2차 패스" 라고 단수로 적는다. 무한 재시도하면
/// 서로를 기다리는 두 조각에서 영원히 돌 수 있다.
pub fn collect_deferred(pieces: &[PieceRecord]) -> Vec<usize> {
    pieces
        .iter()
        .enumerate()
        .filter(|(_, p)| p.state == PlaceState::Deferred)
        .map(|(i, _)| i)
        .collect()
}

/// 2차 패스를 준비한다 — 보류를 `Pending` 으로 되돌려 다시 후보가 되게 한다.
///
/// 되돌리지 않으면 후보 수집(§모듈 4)이 `Deferred` 를 미배치로 보긴 하지만, 상태가
/// 남아 있어 "왜 안 놓였나" 를 구분할 수 없다.
pub fn arm_second_pass(pieces: &mut [PieceRecord]) -> SecondPass {
    let retried = collect_deferred(pieces);
    for &i in &retried {
        pieces[i].state = PlaceState::Pending;
    }
    SecondPass {
        retried,
        still_unplaced: Vec::new(),
    }
}

/// 2차 패스가 끝난 뒤 남은 미배치를 확정한다.
///
/// 여기서 `Failed` 로 바꾸는 것이 **처음이자 유일한 지점**이다. 그 전에는 어떤 경로도
/// 조각을 실패로 만들지 않는다 — 실패는 잔여 조각 처리로 넘긴다는 뜻이지 버린다는
/// 뜻이 아니기 때문이다.
pub fn close_second_pass(pieces: &mut [PieceRecord], pass: &mut SecondPass) {
    for (i, p) in pieces.iter_mut().enumerate() {
        if !p.is_placed() && p.state != PlaceState::Failed {
            p.state = PlaceState::Failed;
            pass.still_unplaced.push(i);
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 효율 재계산 (§4-1, §모듈 9)
// ────────────────────────────────────────────────────────────────────────────

/// 어느 단계가 끝났는가. 로그와 진단에 그대로 실린다.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    /// 배치 완료.
    Placement,
    /// 롤백.
    Rollback,
    /// 잔여 조각 처리.
    Residual,
    /// Complete 재배치.
    Complete,
    /// Pack 압축.
    Pack,
}

/// 단계 종료 시점의 효율 스냅샷.
#[derive(Clone, Debug, PartialEq)]
pub struct EfficiencySnapshot {
    pub stage: Stage,
    /// 표면별 효율.
    pub per_surface: Vec<f64>,
    /// 전체 효율 — 배치 면적 합 / 기준 면적 합.
    pub overall: f64,
}

/// 효율을 다시 잰다 (§4-1 "각 단계 종료 시점마다 재계산").
pub fn recompute_efficiency(
    stage: Stage,
    reports: &[SurfaceReport],
    surfaces: &[SurfaceRecord],
    current_width: f64,
) -> EfficiencySnapshot {
    let per_surface: Vec<f64> = reports.iter().map(|r| r.efficiency).collect();
    // 전체는 표면 효율의 평균이 아니라 **면적 비의 합**이다. 평균을 쓰면 작은 표면이
    // 큰 표면과 같은 무게를 갖는다.
    let mut num = 0.0;
    let mut den = 0.0;
    for r in reports {
        let Some(s) = surfaces.get(r.surface) else {
            continue;
        };
        let area = s.reference_area(current_width);
        num += r.efficiency * area;
        den += area;
    }
    EfficiencySnapshot {
        stage,
        per_surface,
        overall: if den > 0.0 { num / den } else { 0.0 },
    }
}

/// 개선이 수렴했는가 (§5 개선 허용오차 100, 이동 임계값 30).
///
/// 두 조건 중 **하나라도** 맞으면 수렴이다. 설계가 둘을 따로 적어 두었고, 둘 다
/// 만족해야 한다고는 적지 않았다.
pub fn converged(improvement: f64, total_movement: f64) -> bool {
    improvement.abs() < IMPROVE_TOLERANCE
        || total_movement.abs() < crate::refplace::params::MOVE_THRESHOLD
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::refplace::record::{PlacedPose, RotationEntry, RotationList};

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

    fn surface(pieces: Vec<usize>) -> SurfaceRecord {
        let mut s = SurfaceRecord::new(1000.0, 100.0);
        s.pieces = pieces;
        s
    }

    /// 통합·검증은 놓인 조각만 세고, 효율을 함께 낸다.
    #[test]
    fn integration_counts_placed_pieces_and_reports_efficiency() {
        let pieces = vec![placed(0), piece(1)];
        let s = surface(vec![0, 1]);
        // 조각 0 은 10x10 = 100mm², 기준 면적 = 200 x 100 = 20,000.
        let r = integrate_and_validate(&pieces, &s, 0, 200.0, |p| {
            (p == 0).then_some((0.0, 0.0, 10.0, 10.0))
        });
        assert_eq!(r.integrated, 1);
        assert!(r.ok());
        assert!((r.efficiency - 100.0 / 20_000.0).abs() < 1.0e-12);
    }

    /// 검증 실패는 **표식일 뿐** — 배치를 버리지 않는다.
    #[test]
    fn a_validation_failure_is_only_a_mark_not_a_discard() {
        let pieces = vec![placed(0)];
        let s = surface(vec![0]);
        // y 가 컨테이너(높이 100) 밖으로 크게 나간다.
        let r = integrate_and_validate(&pieces, &s, 0, 200.0, |_| {
            Some((0.0, 0.0, 10.0, 500.0))
        });
        assert!(!r.ok());
        assert_eq!(r.errors[0].1, SurfaceError::OutsideContainer);
        // 그래도 통합은 됐다.
        assert_eq!(r.integrated, 1);
    }

    /// **오른쪽으로 나가는 것은 위반이 아니다** — strip 은 그 방향으로 자란다.
    #[test]
    fn growing_past_the_current_width_is_not_a_violation() {
        let pieces = vec![placed(0)];
        let s = surface(vec![0]);
        let r = integrate_and_validate(&pieces, &s, 0, 50.0, |_| {
            Some((40.0, 0.0, 900.0, 10.0))
        });
        assert!(r.ok(), "폭이 자라는 것은 컨테이너 이탈이 아니다");
    }

    /// 상태와 데이터가 어긋나면(놓였는데 자세 없음) 표식한다.
    #[test]
    fn a_placed_piece_without_a_pose_is_flagged() {
        let mut pieces = vec![placed(0)];
        pieces[0].pose = None;
        let s = surface(vec![0]);
        let r = integrate_and_validate(&pieces, &s, 0, 200.0, |_| Some((0.0, 0.0, 1.0, 1.0)));
        assert_eq!(r.errors[0].1, SurfaceError::MissingPose);
    }

    /// 겹침은 bbox 가 **실질적으로 파고든** 경우만 표식한다.
    #[test]
    fn only_a_real_bbox_intrusion_is_flagged_as_overlap() {
        let pieces = vec![placed(0), placed(1)];
        let s = surface(vec![0, 1]);
        // 맞닿기만 한 경우 — 겹침이 아니다.
        let touching = integrate_and_validate(&pieces, &s, 0, 500.0, |p| {
            Some(if p == 0 {
                (0.0, 0.0, 100.0, 100.0)
            } else {
                (100.0, 0.0, 200.0, 100.0)
            })
        });
        assert!(touching.ok(), "맞닿음은 겹침이 아니다");
        // 크게 파고든 경우.
        let overlapping = integrate_and_validate(&pieces, &s, 0, 500.0, |p| {
            Some(if p == 0 {
                (0.0, 0.0, 100.0, 100.0)
            } else {
                (0.0, 0.0, 100.0, 100.0)
            })
        });
        assert!(overlapping.errors.iter().any(|(_, e)| *e == SurfaceError::Overlap));
    }

    /// 2차 패스는 보류를 되돌려 **다시 후보가 되게** 한다.
    #[test]
    fn the_second_pass_rearms_the_deferred_chain() {
        let mut pieces = vec![piece(0), piece(1), placed(2)];
        pieces[0].state = PlaceState::Deferred;
        pieces[1].state = PlaceState::Deferred;
        let pass = arm_second_pass(&mut pieces);
        assert_eq!(pass.retried, vec![0, 1]);
        assert_eq!(pieces[0].state, PlaceState::Pending);
        assert_eq!(pieces[1].state, PlaceState::Pending);
        assert_eq!(pieces[2].state, PlaceState::Placed, "놓인 것은 건드리지 않는다");
    }

    /// 실패로 바꾸는 것은 **2차 패스가 끝난 뒤 한 곳에서만** 일어난다.
    #[test]
    fn failure_is_only_stamped_after_the_second_pass_closes() {
        let mut pieces = vec![piece(0), placed(1)];
        // 2차 패스 준비 시점에는 아무것도 실패가 아니다.
        let mut pass = arm_second_pass(&mut pieces);
        assert_ne!(pieces[0].state, PlaceState::Failed);
        // 닫을 때 비로소 실패가 된다.
        close_second_pass(&mut pieces, &mut pass);
        assert_eq!(pieces[0].state, PlaceState::Failed);
        assert_eq!(pass.still_unplaced, vec![0]);
        assert_eq!(pieces[1].state, PlaceState::Placed);
    }

    /// 전체 효율은 **면적 가중**이다 — 평균이면 작은 표면이 큰 표면과 같은 무게를 갖는다.
    #[test]
    fn the_overall_efficiency_is_area_weighted_not_averaged() {
        let mut big = SurfaceRecord::new(1000.0, 100.0);
        big.pieces = vec![0];
        let mut small = SurfaceRecord::new(10.0, 10.0);
        small.pieces = vec![1];
        let surfaces = vec![big, small];
        let reports = vec![
            SurfaceReport {
                surface: 0,
                integrated: 1,
                errors: Vec::new(),
                efficiency: 0.5,
            },
            SurfaceReport {
                surface: 1,
                integrated: 1,
                errors: Vec::new(),
                efficiency: 1.0,
            },
        ];
        let snap = recompute_efficiency(Stage::Placement, &reports, &surfaces, 100.0);
        // 큰 표면(100x100=10,000)과 작은 표면(100x10=1,000) 가중.
        let expected = (0.5 * 10_000.0 + 1.0 * 1_000.0) / 11_000.0;
        assert!((snap.overall - expected).abs() < 1.0e-12);
        // 산술 평균(0.75)과는 다르다.
        assert!((snap.overall - 0.75).abs() > 0.01);
    }

    /// 수렴은 두 조건 **중 하나**로 판정한다 (§5).
    #[test]
    fn convergence_needs_either_criterion_not_both() {
        // 개선은 작지만 이동은 크다 -> 수렴.
        assert!(converged(50.0, 1000.0));
        // 개선은 크지만 이동은 작다 -> 수렴.
        assert!(converged(1000.0, 10.0));
        // 둘 다 크다 -> 아직.
        assert!(!converged(1000.0, 1000.0));
    }

    /// 효율 스냅샷은 어느 단계의 것인지 함께 남긴다 — 낡은 값과 구분하기 위해서다.
    #[test]
    fn a_snapshot_records_which_stage_produced_it() {
        let snap = recompute_efficiency(Stage::Rollback, &[], &[], 100.0);
        assert_eq!(snap.stage, Stage::Rollback);
        assert_eq!(snap.overall, 0.0);
    }
}
