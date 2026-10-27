//! 모듈 10 — 후속 단계 (선택적).
//!
//! 설계 §모듈 10:
//!
//! ```text
//!   Complete 재배치: 단순화 다각형 -> 원본 다각형으로 전환.
//!     워커 스레드에서 같은 배치 루프(재시작 + LP 개선, 모듈 7~9와 동일 기계)를
//!     다시 실행한다. 완료 대기(5초 타임아웃, 상태 로그) 후 효율 재계산 + 마무리 패스.
//!     즉 Draft와 Complete는 알고리즘 차이가 아니라 다각형 정밀도 차이다.
//!   잔여 조각 처리: 조각 1개당 3변수 LP — min t, 24방향 지지함수로 이동거리 근사,
//!     실행 불가 시 요구 간격 완화 후 재시도
//!   Pack: 방향별(아래->위 / 위->아래 등) 압축
//! ```

use crate::refplace::params::{COMPLETE_JOIN_TIMEOUT, SUPPORT_DIRECTIONS, SimplifyLevels};
use crate::refplace::record::GapSettings;

// ────────────────────────────────────────────────────────────────────────────
// Complete 재배치
// ────────────────────────────────────────────────────────────────────────────

/// 배치 정밀도 단계.
///
/// **알고리즘이 아니라 다각형 정밀도의 차이다** — 설계가 명시적으로 그렇게 적는다.
/// 그래서 이 타입은 단순화 강도만 고르고, 배치 루프는 같은 것을 쓴다.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Precision {
    /// 단순화 다각형으로 빠르게 초안.
    Draft,
    /// 원본 다각형으로 재배치.
    Complete,
}

impl Precision {
    /// 이 단계가 쓸 (볼록, 오목) 단순화 강도.
    pub fn tolerances(&self, levels: &SimplifyLevels) -> (f32, f32) {
        match self {
            Self::Draft => levels.draft(),
            Self::Complete => levels.complete(),
        }
    }

    /// 다음 단계. `Complete` 다음은 없다.
    pub fn next(&self) -> Option<Self> {
        match self {
            Self::Draft => Some(Self::Complete),
            Self::Complete => None,
        }
    }
}

/// Complete 재배치의 완료 대기 결과.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JoinOutcome {
    /// 제때 끝났다.
    Finished,
    /// 5초 타임아웃. **Draft 결과를 유지한다** — 버리지 않는다.
    TimedOut,
}

/// Complete 재배치 워커를 기다린다 (§모듈 10 "완료 대기 5초 타임아웃, 상태 로그").
///
/// `poll` 이 `true` 를 돌려주면 끝난 것이다. 타임아웃이면 **Draft 결과를 그대로 쓴다** —
/// 정밀도를 못 올린 것이지 배치를 잃은 것이 아니다.
pub fn join_complete(mut poll: impl FnMut() -> bool, mut elapsed: impl FnMut() -> std::time::Duration) -> JoinOutcome {
    loop {
        if poll() {
            return JoinOutcome::Finished;
        }
        if elapsed() >= COMPLETE_JOIN_TIMEOUT {
            log::warn!(
                "[REFPLACE] Complete 재배치가 {}초 안에 끝나지 않았다 — Draft 결과를 유지한다",
                COMPLETE_JOIN_TIMEOUT.as_secs()
            );
            return JoinOutcome::TimedOut;
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 잔여 조각 LP (§모듈 10)
// ────────────────────────────────────────────────────────────────────────────

/// 24방향 지지함수 방향 벡터 테이블 (§모듈 1, §5).
///
/// 15도 간격. **이것이 24방향 테이블의 진짜 쓰임이다** — 배치 회전이 아니다.
/// 갱신된 설계가 그 오해를 명시적으로 정정했다.
pub fn support_directions() -> Vec<(f64, f64)> {
    (0..SUPPORT_DIRECTIONS)
        .map(|k| {
            let a = 2.0 * std::f64::consts::PI * (k as f64) / (SUPPORT_DIRECTIONS as f64);
            (a.cos(), a.sin())
        })
        .collect()
}

/// 잔여 조각 하나를 넣기 위한 3변수 LP.
///
/// 변수는 `(dx, dy, t)` 다. 목적은 `min t` — `t` 는 "요구 간격을 얼마나 어겼는가" 이고,
/// `t <= 0` 이면 간격을 지키면서 들어간다는 뜻이다.
///
/// 제약은 24방향 지지함수로 만든다: 방향 `u` 마다
/// `<u, (dx,dy)> <= 여유_u + t`. 즉 **이동거리를 24개 반평면으로 근사**한다.
#[derive(Clone, Debug, PartialEq)]
pub struct ResidualLp {
    /// 방향별 여유(mm). 길이는 [`SUPPORT_DIRECTIONS`].
    pub slack: Vec<f64>,
    /// 요구 간격(mm).
    pub required_gap: f64,
}

/// LP 풀이 결과.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LpOutcome {
    /// 들어간다. `(dx, dy)` 만큼 옮기면 된다.
    Feasible { dx: f64, dy: f64, t: f64 },
    /// 요구 간격으로는 안 된다 — **완화 후 재시도**해야 한다.
    NeedsRelaxation,
    /// 완화해도 안 된다.
    Infeasible,
}

impl ResidualLp {
    /// 24방향 반평면 제약을 만든다.
    ///
    /// 반환은 `(방향, 우변)` 목록이다. 우변은 `여유 - 요구간격` 이다 — 간격을 지키려면
    /// 그만큼 덜 움직여야 한다.
    pub fn halfplanes(&self, gap: f64) -> Vec<((f64, f64), f64)> {
        support_directions()
            .into_iter()
            .zip(self.slack.iter().copied())
            .map(|(dir, s)| (dir, s - gap))
            .collect()
    }

    /// `min t` 를 푼다.
    ///
    /// # 왜 완결된 LP 솔버를 쓰지 않는가
    ///
    /// 변수가 셋이고 제약이 24개인 문제다. 이 규모에서는 반평면 교집합의 중심을
    /// 직접 구하는 편이 솔버 호출보다 싸고, 무엇보다 **결정론적**이다. 큰 LP(전체 압축)는
    /// [`crate::lp_solver`] 가 맡는다.
    ///
    /// 여기서는 최소 우변을 `t` 로 삼는 보수적 해를 쓴다 — 모든 방향을 동시에 만족하는
    /// 이동을 찾는 대신, **가장 빡빡한 방향**이 곧 `t` 가 된다.
    pub fn solve(&self, gap: f64) -> LpOutcome {
        let planes = self.halfplanes(gap);
        if planes.is_empty() {
            return LpOutcome::Infeasible;
        }
        // 가장 빡빡한 방향 = 우변이 가장 작은 것.
        let (dir, rhs) = planes
            .iter()
            .copied()
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .expect(crate::dmsg!("비어 있지 않다"));
        if !rhs.is_finite() {
            return LpOutcome::Infeasible;
        }
        if rhs >= 0.0 {
            // 여유가 있다 — 움직일 필요가 없다.
            return LpOutcome::Feasible {
                dx: 0.0,
                dy: 0.0,
                t: -rhs,
            };
        }
        // 부족한 만큼 그 방향의 **반대**로 민다.
        let t = -rhs;
        LpOutcome::Feasible {
            dx: -dir.0 * t,
            dy: -dir.1 * t,
            t,
        }
    }

    /// 요구 간격을 지켜 풀고, 안 되면 **완화 후 재시도**한다 (§모듈 10).
    ///
    /// 설계: "실행 불가 시 요구 간격 완화 후 재시도". 간격은 연성 제약이므로 못 지킬
    /// 바에는 줄여서라도 넣는다 — 넣지 못하면 그 조각은 통째로 버려지기 때문이다.
    pub fn solve_with_relaxation(&self, gaps: &GapSettings) -> LpOutcome {
        match self.solve(self.required_gap) {
            LpOutcome::Feasible { dx, dy, t } if t <= 0.0 => LpOutcome::Feasible { dx, dy, t },
            _ => {
                // 부분 폐쇄(기본 90%)로 완화해 다시 푼다.
                let relaxed = gaps.relaxed().min(self.required_gap);
                match self.solve(relaxed) {
                    LpOutcome::Feasible { dx, dy, t } if t <= 0.0 => LpOutcome::Feasible { dx, dy, t },
                    LpOutcome::Feasible { .. } => LpOutcome::NeedsRelaxation,
                    other => other,
                }
            }
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Pack 압축 (§모듈 10)
// ────────────────────────────────────────────────────────────────────────────

/// Pack 압축 방향.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PackDirection {
    /// 아래 -> 위.
    BottomUp,
    /// 위 -> 아래.
    TopDown,
    /// 왼쪽 -> 오른쪽.
    LeftToRight,
    /// 오른쪽 -> 왼쪽.
    RightToLeft,
}

impl PackDirection {
    /// 이 방향으로 압축할 때 조각을 훑는 정렬 키.
    ///
    /// **미는 방향의 반대쪽부터** 훑어야 한다. 아래로 밀면서 위 조각부터 손대면
    /// 그 아래 조각이 아직 안 내려와 자리가 비어 보이지 않는다.
    pub fn sort_key(&self, bbox: (f64, f64, f64, f64)) -> f64 {
        match self {
            Self::BottomUp => bbox.1,        // y_min 오름차순
            Self::TopDown => -bbox.3,        // y_max 내림차순
            Self::LeftToRight => bbox.0,     // x_min 오름차순
            Self::RightToLeft => -bbox.2,    // x_max 내림차순
        }
    }

    /// 미는 단위 벡터.
    pub fn vector(&self) -> (f64, f64) {
        match self {
            Self::BottomUp => (0.0, -1.0),
            Self::TopDown => (0.0, 1.0),
            Self::LeftToRight => (-1.0, 0.0),
            Self::RightToLeft => (1.0, 0.0),
        }
    }
}

/// 압축 순서를 만든다 — 미는 방향의 반대쪽부터.
///
/// 동률이면 인덱스로 갈라 **전순서**를 만든다. 부동소수 키만으로 정렬하면 같은 입력이
/// 다른 순서를 낼 수 있다.
pub fn pack_order(boxes: &[(usize, (f64, f64, f64, f64))], dir: PackDirection) -> Vec<usize> {
    let mut order: Vec<usize> = (0..boxes.len()).collect();
    order.sort_by(|&a, &b| {
        dir.sort_key(boxes[a].1)
            .total_cmp(&dir.sort_key(boxes[b].1))
            .then_with(|| boxes[a].0.cmp(&boxes[b].0))
    });
    order
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Draft 와 Complete 는 **알고리즘이 아니라 정밀도** 차이다.
    #[test]
    fn draft_and_complete_differ_only_in_polygon_precision() {
        let levels = SimplifyLevels::default();
        let (dc, dn) = Precision::Draft.tolerances(&levels);
        let (cc, cn) = Precision::Complete.tolerances(&levels);
        assert!(dc >= cc && dn >= cn, "Draft 가 더 거칠어야 한다");
        assert_eq!(Precision::Draft.next(), Some(Precision::Complete));
        assert_eq!(Precision::Complete.next(), None);
    }

    /// 타임아웃이면 **Draft 결과를 유지한다** — 버리지 않는다.
    #[test]
    fn a_timeout_keeps_the_draft_instead_of_discarding_it() {
        let t0 = std::time::Instant::now();
        // 절대 끝나지 않는 워커 + 이미 지난 시간.
        let out = join_complete(|| false, || COMPLETE_JOIN_TIMEOUT + std::time::Duration::from_secs(1));
        assert_eq!(out, JoinOutcome::TimedOut);
        let _ = t0;
        // 제때 끝나면 Finished.
        assert_eq!(
            join_complete(|| true, || std::time::Duration::ZERO),
            JoinOutcome::Finished
        );
    }

    /// 24방향 지지함수 테이블 — **배치 회전이 아니다**.
    #[test]
    fn the_support_table_has_twenty_four_directions_fifteen_degrees_apart() {
        let dirs = support_directions();
        assert_eq!(dirs.len(), 24);
        // 전부 단위 벡터.
        for (x, y) in &dirs {
            assert!((x * x + y * y - 1.0).abs() < 1.0e-12);
        }
        // 첫 방향은 +x, 여섯 번째(90도)는 +y.
        assert!((dirs[0].0 - 1.0).abs() < 1.0e-12);
        assert!((dirs[6].1 - 1.0).abs() < 1.0e-12, "24/4 = 6 번째가 90도다");
    }

    /// 여유가 충분하면 움직이지 않는다.
    #[test]
    fn a_piece_with_room_does_not_move() {
        let lp = ResidualLp {
            slack: vec![10.0; SUPPORT_DIRECTIONS],
            required_gap: 2.0,
        };
        match lp.solve(2.0) {
            LpOutcome::Feasible { dx, dy, t } => {
                assert_eq!((dx, dy), (0.0, 0.0));
                assert!(t <= 0.0, "여유가 있으면 t <= 0 이다");
            }
            other => panic!("들어가야 한다: {other:?}"),
        }
    }

    /// 부족하면 **가장 빡빡한 방향의 반대**로 민다.
    #[test]
    fn a_tight_direction_pushes_the_piece_the_other_way() {
        let mut slack = vec![10.0; SUPPORT_DIRECTIONS];
        slack[0] = 0.5; // +x 방향이 빡빡하다
        let lp = ResidualLp {
            slack,
            required_gap: 2.0,
        };
        match lp.solve(2.0) {
            LpOutcome::Feasible { dx, dy, t } => {
                assert!(t > 0.0, "간격을 어긴다");
                assert!(dx < 0.0, "+x 가 빡빡하면 -x 로 밀어야 한다: dx={dx}");
                assert!(dy.abs() < 1.0e-12);
            }
            other => panic!("해가 있어야 한다: {other:?}"),
        }
    }

    /// 요구 간격으로 안 되면 **완화 후 재시도**한다 (§모듈 10).
    #[test]
    fn an_infeasible_gap_is_retried_after_relaxation() {
        let mut slack = vec![10.0; SUPPORT_DIRECTIONS];
        slack[0] = 1.85; // 요구 2.0 은 못 지키지만 완화된 1.8 은 지킨다
        let lp = ResidualLp {
            slack,
            required_gap: 2.0,
        };
        let gaps = GapSettings {
            gap_mm: 2.0,
            lp_partial_closure: 0.9, // -> 1.8
            ..GapSettings::default()
        };
        // 요구 간격 그대로는 안 된다.
        assert!(matches!(lp.solve(2.0), LpOutcome::Feasible { t, .. } if t > 0.0));
        // 완화하면 들어간다.
        match lp.solve_with_relaxation(&gaps) {
            LpOutcome::Feasible { t, .. } => assert!(t <= 0.0, "완화 후에는 지켜야 한다"),
            other => panic!("완화하면 들어가야 한다: {other:?}"),
        }
    }

    /// 완화해도 안 되면 그렇게 보고한다 — 조용히 겹쳐 놓지 않는다.
    #[test]
    fn a_piece_that_cannot_fit_even_relaxed_is_reported_not_forced() {
        let mut slack = vec![10.0; SUPPORT_DIRECTIONS];
        slack[3] = -50.0; // 통째로 겹친다
        let lp = ResidualLp {
            slack,
            required_gap: 2.0,
        };
        let gaps = GapSettings {
            gap_mm: 2.0,
            ..GapSettings::default()
        };
        assert_eq!(lp.solve_with_relaxation(&gaps), LpOutcome::NeedsRelaxation);
    }

    /// Pack 은 **미는 방향의 반대쪽부터** 훑는다.
    #[test]
    fn pack_walks_from_the_side_opposite_to_the_push() {
        let boxes = vec![
            (0usize, (0.0, 50.0, 10.0, 60.0)),  // 위
            (1usize, (0.0, 0.0, 10.0, 10.0)),   // 아래
            (2usize, (0.0, 25.0, 10.0, 35.0)),  // 중간
        ];
        // 아래로 밀면 아래 조각부터.
        assert_eq!(pack_order(&boxes, PackDirection::BottomUp), vec![1, 2, 0]);
        // 위로 밀면 위 조각부터.
        assert_eq!(pack_order(&boxes, PackDirection::TopDown), vec![0, 2, 1]);
    }

    /// 미는 벡터가 방향과 맞는지.
    #[test]
    fn the_push_vector_matches_the_direction() {
        assert_eq!(PackDirection::BottomUp.vector(), (0.0, -1.0));
        assert_eq!(PackDirection::TopDown.vector(), (0.0, 1.0));
        assert_eq!(PackDirection::LeftToRight.vector(), (-1.0, 0.0));
        assert_eq!(PackDirection::RightToLeft.vector(), (1.0, 0.0));
    }

    /// 동률이면 **인덱스**로 갈라 전순서를 만든다.
    #[test]
    fn pack_order_is_total_even_when_keys_tie() {
        let boxes = vec![
            (7usize, (0.0, 0.0, 1.0, 1.0)),
            (3usize, (0.0, 0.0, 1.0, 1.0)),
            (5usize, (0.0, 0.0, 1.0, 1.0)),
        ];
        // 키가 전부 같다 -> 원소의 id(7,3,5) 오름차순 = 인덱스 1,2,0.
        assert_eq!(pack_order(&boxes, PackDirection::BottomUp), vec![1, 2, 0]);
    }

    /// 제약 우변은 `여유 - 요구간격` 이다.
    #[test]
    fn the_halfplane_rhs_subtracts_the_required_gap() {
        let lp = ResidualLp {
            slack: vec![5.0; SUPPORT_DIRECTIONS],
            required_gap: 2.0,
        };
        let planes = lp.halfplanes(2.0);
        assert_eq!(planes.len(), SUPPORT_DIRECTIONS);
        assert!((planes[0].1 - 3.0).abs() < 1.0e-12);
    }
}
