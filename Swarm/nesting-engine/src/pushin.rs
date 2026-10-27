//! 밀어내기 압축 (§5.11) — 정정 반영: 제4차 B2, 제5차 N1·N2, 제6차 P3, 제11차 V1~V4, 제24차 AL2.
//!
//! # 두 경로
//!
//! - **루프 안 경로**: LP 풀이 뒤처리. `배치수 < 전체수` 일 때만 돈다. 점 배열 A·B 의
//!   `[배치수, 상한)` 구간에서 x·y 를 **같은 k 로 네 번** 뺀다. 상한은 배치수부터 앞으로
//!   훑다가 `x[상한] ≥ 최대경계 + 지역경계값` 에서 멈춘다.
//! - **최종 경로**: 상태가 `0x200`(압축 단계 완료)으로 넘어갈 때 점수 배열·최고점·스냅샷을
//!   확정한다.
//!
//! 156/156 처럼 전부 놓인 입력에서는 루프 안 경로가 **아무것도 하지 않는다** — 지시서가
//! 그렇게 정의한다. 밀도 향상은 §5 압축 LP 와 Complete 에서 나온다.
//!
//! # k
//!
//! `k = round(점수배열[직전])`, 셔플 플래그(`+0x188`)가 서 있으면 `k = round(폭 − 점수배열[직전])`.
//! 점수배열에는 LP 해의 **물리 길이 L** 이 들어간다(AL2) — 목적값 `0.8·L` 이 아니다. k 에
//! 0.8 을 다시 곱하지 않는다. 반올림 방식은 전역 모드로 고정한다(0 = 반올림, 그 밖 = 절사).
//!
//! # 옮기지 않은 것
//!
//! 점 배열의 바이트 배치(`+0xac/+0xc0`, 스냅샷 `+0xb8/+0xc4`)와 x87 명령열. `지역경계값` 은
//! 지시서 어디에도 정의가 없어 호출자가 넘기는 인자로 둔다.

use crate::state::EngineState;

/// k 반올림 방식 — 원천의 전역 `ROUND_MODE` (0 = 반올림, 그 밖 = 절사).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RoundMode {
    /// C 런타임 `round` — 절반은 0 에서 멀어지는 쪽. Rust `f64::round` 와 같다.
    Round,
    Trunc,
}

impl RoundMode {
    pub fn apply(self, v: f64) -> i64 {
        match self {
            RoundMode::Round => v.round() as i64,
            RoundMode::Trunc => v.trunc() as i64,
        }
    }
}

/// k 산출. `index` 는 읽을 점수 배열 칸(제11차: 반복 카운터 `+0x7c` 가 가리키는 칸 = 가장
/// 최근에 기록된 L). `shuffle_width` 가 있으면 셔플 경로.
pub fn k_value(score_hist: &[f64], index: usize, shuffle_width: Option<f64>, mode: RoundMode) -> Option<i64> {
    let s = *score_hist.get(index)?;
    Some(match shuffle_width {
        None => mode.apply(s),
        Some(w) => mode.apply(w - s),
    })
}

/// 두 병렬 점 배열과 그 스냅샷 (§5.11 A·B, P3: 각각 `{x, y}` 쌍의 배열).
#[derive(Clone, Debug, Default)]
pub struct PointArrays {
    pub a: Vec<(i64, i64)>,
    pub b: Vec<(i64, i64)>,
    snap_a: Vec<(i64, i64)>,
    snap_b: Vec<(i64, i64)>,
}

impl PointArrays {
    pub fn new(a: Vec<(i64, i64)>, b: Vec<(i64, i64)>) -> Self {
        let (snap_a, snap_b) = (a.clone(), b.clone());
        Self { a, b, snap_a, snap_b }
    }

    pub fn snapshot(&mut self) {
        self.snap_a.clone_from(&self.a);
        self.snap_b.clone_from(&self.b);
    }

    /// 재시작 카운터가 마지막 값에 닿거나 상태가 `0x20` 으로 넘어갈 때 (§5.11).
    pub fn rollback(&mut self) {
        self.a.clone_from(&self.snap_a);
        self.b.clone_from(&self.snap_b);
    }
}

/// 루프 안 경로. 움직인 점 개수를 돌려준다. 루프 비교 상한은 전체수(`+0x4c`, 제5차 N2).
pub fn push_in(arr: &mut PointArrays, placed: usize, total: usize, max_bound: i64, local_bound: i64, k: i64) -> usize {
    if placed >= total {
        return 0;
    }
    let total = total.min(arr.a.len()).min(arr.b.len());
    let mut hi = placed;
    while hi < total {
        if arr.a[hi].0 >= max_bound + local_bound {
            break;
        }
        hi += 1;
    }
    for i in placed..hi {
        arr.a[i].0 -= k;
        arr.a[i].1 -= k;
        arr.b[i].0 -= k;
        arr.b[i].1 -= k;
    }
    hi.saturating_sub(placed)
}

/// 최종 경로 (상태 `0x200`) 의 확정 결과.
#[derive(Clone, Debug)]
pub struct FinalCommit {
    pub state: EngineState,
    /// 점수 배열의 최소 폭(= 최고점). 비어 있으면 `None`.
    pub best_score: Option<f64>,
    pub score_hist: Vec<f64>,
}

/// 압축 단계 완료로 넘어가며 점수 배열·최고점·스냅샷을 확정한다.
pub fn final_commit(score_hist: &[f64], arr: &mut PointArrays) -> FinalCommit {
    arr.snapshot();
    let best_score = score_hist.iter().copied().fold(None, |b: Option<f64>, s| Some(b.map_or(s, |b| b.min(s))));
    FinalCommit { state: EngineState::CompactDone, best_score, score_hist: score_hist.to_vec() }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// §5.9 예제 2.
    #[test]
    fn k_follows_example_two() {
        let hist = [7178.0, 7102.0, 7060.0];
        assert_eq!(k_value(&hist, 1, None, RoundMode::Round), Some(7102));
        assert_eq!(k_value(&hist, 2, Some(7178.0), RoundMode::Round), Some(118));
        assert_eq!(k_value(&hist, 2, None, RoundMode::Round), Some(7060));
        assert_eq!(k_value(&hist, 3, None, RoundMode::Round), None);
    }

    /// 회귀 ④ — 118.6 에서 반올림과 절사가 갈린다.
    #[test]
    fn round_mode_splits_at_118_6() {
        assert_eq!(RoundMode::Round.apply(118.4), 118);
        assert_eq!(RoundMode::Trunc.apply(118.4), 118);
        assert_eq!(RoundMode::Round.apply(118.6), 119);
        assert_eq!(RoundMode::Trunc.apply(118.6), 118);
    }

    /// AL2: 점수배열에는 L 이 들어가고 k 에 0.8 을 다시 곱하지 않는다.
    #[test]
    fn k_is_not_rescaled_by_the_objective_coefficient() {
        assert_eq!(k_value(&[100.0], 0, None, RoundMode::Round), Some(100));
        assert_eq!(k_value(&[7102.0], 0, None, RoundMode::Round), Some(7102));
    }

    #[test]
    fn push_in_moves_both_axes_of_both_arrays_by_the_same_k() {
        let pts = vec![(10, 20), (100, 200), (300, 400), (5000, 1)];
        let mut arr = PointArrays::new(pts.clone(), pts.clone());
        // 배치수 1, 전체 4, 경계 1000 → 칸 1·2 만 대상 (칸 3 은 x=5000 ≥ 1000).
        let moved = push_in(&mut arr, 1, 4, 900, 100, 118);
        assert_eq!(moved, 2);
        assert_eq!(arr.a[0], (10, 20));
        assert_eq!(arr.a[1], (100 - 118, 200 - 118));
        assert_eq!(arr.b[2], (300 - 118, 400 - 118));
        assert_eq!(arr.a[3], (5000, 1));
        arr.rollback();
        assert_eq!(arr.a, pts);
    }

    #[test]
    fn push_in_is_inert_when_everything_is_placed() {
        let pts = vec![(1, 1); 5];
        let mut arr = PointArrays::new(pts.clone(), pts.clone());
        assert_eq!(push_in(&mut arr, 5, 5, 0, 0, 7), 0);
        assert_eq!(arr.a, pts);
    }

    #[test]
    fn final_commit_marks_compaction_done_with_the_smallest_width() {
        let mut arr = PointArrays::default();
        let c = final_commit(&[7178.0, 7102.0, 7110.0], &mut arr);
        assert_eq!(c.state, EngineState::CompactDone);
        assert_eq!(c.best_score, Some(7102.0));
        assert!(final_commit(&[], &mut arr).best_score.is_none());
    }
}
