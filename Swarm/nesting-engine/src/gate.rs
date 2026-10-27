//! 점수·수락 게이트 (§3.5, §3.5.4, §3.5.6, §5.9 예제 3).
//!
//! # 이 모듈을 위치 선택의 거부권으로 쓰지 않는 이유
//!
//! 지시서의 게이트를 한 조각의 **위치 후보 목록**에 그대로 걸면, 현재 스트립 폭 안에
//! 들어가는 후보는 `value` 가 정확히 같다(분자 = 누적 배치면적, 분모 고정). 그러면 상위
//! 후보의 `d3 = 0` → `w = 0` → `Σw < 0.001` → 클래스 제외(`−1.0`) → `min_s = 100` 유지
//! → 첫 후보에서 `value < min_s` 로 break 해 **아무것도 수락되지 않는다.** 지시서 §5.9 도
//! "경로A 에서 동률이 전부 거절되는 것은 설계" 라고 적는다. 그대로 쓰면 조각이 놓이지
//! 못하고 §11 의 156/156 게이트와 모순이 된다.
//!
//! 그래서 위치는 §3.4 `best_spot`(value 내림차순, 동률은 x → y → 접촉)으로 고르고,
//! 이 게이트는 §5.9 정답표로 검증한 독립 함수로 두어 후보 목록마다 수락 수를 진단값으로
//! 남긴다. 지시서의 수식과 상수는 한 글자도 바꾸지 않았다.

/// 클래스 슬롯 수 (§3.5.4: 유효 인덱스 0..8).
pub const N_CLASSES: usize = 9;
/// 클래스당 누적 상한 (`CLASS_CAP`).
pub const CLASS_CAP: f64 = 5.0;
/// 부족률 스케일 (`DEFICIT_SCALE`).
pub const DEFICIT_SCALE: f64 = 100.0;
/// 가중치 기본값이자 구간3 곱셈계수 (`GATE_HALF`, 두 역할 겸용).
pub const GATE_HALF: f64 = 0.5;
/// 가중치 0 이 되는 하한 임계 (`GATE_FLOOR`).
pub const GATE_FLOOR: f64 = 0.2;
/// `Σw` 센티널 임계.
pub const SUMW_EPS: f64 = 0.001;
/// 클래스 제외 센티널.
pub const EXCLUDED: f64 = -1.0;
/// `min_s` 초기값.
pub const MIN_S_INIT: f64 = 100.0;
/// 기준값 모드 임계 (`REF_MATCH_EPS`).
pub const REF_MATCH_EPS: f64 = 0.001;

/// 후보 통계 레코드 (§1.3 — 필드 의미만 옮기고 바이트 배치는 강제하지 않는다, §13.1).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Record {
    pub base: f64,
    pub value: f64,
    /// 클래스(표면) 인덱스. 수락 판정에서 `0` 은 클래스 8 로 읽는다.
    pub class: usize,
}

/// `w(d3)` — §3.5.4 의 중첩 분기를 그대로 옮긴다. **비단조**다(0.5 → 0.75, 0.51 → 1.0).
pub fn weight(d3: f64) -> f64 {
    let mut w = GATE_HALF;
    if d3 <= 1.0 {
        w = 1.0;
        if d3 <= GATE_HALF {
            w = 0.0;
            if GATE_FLOOR < d3 {
                w = 1.0 - d3 * GATE_HALF;
            }
        }
    }
    w
}

#[derive(Clone, Debug, PartialEq)]
pub struct GateResult {
    pub v_max: f64,
    pub acc: [f64; N_CLASSES],
    pub sumw: [f64; N_CLASSES],
    pub scores: [f64; N_CLASSES],
    pub min_s: f64,
    pub final_score: f64,
    /// 수락된 레코드의 원래 인덱스 (value 내림차순 순회 순서).
    pub accepted: Vec<usize>,
    pub rejected: Vec<usize>,
}

/// value 내림차순 안정 정렬한 인덱스 (§1.3: 동점이면 순서 유지).
fn order_desc(records: &[Record]) -> Vec<usize> {
    let mut idx: Vec<usize> = (0..records.len()).collect();
    idx.sort_by(|&a, &b| records[b].value.total_cmp(&records[a].value));
    idx
}

/// 클래스 점수 게이트 (`ref_mode_flag == 0`, §3.5).
///
/// `v_max == 0` 이거나 레코드가 없으면 `None` — §3.5.6 "후보 없음으로 종료". 임의 값을
/// 대입하지 않는다.
pub fn run(records: &[Record]) -> Option<GateResult> {
    if records.is_empty() {
        return None;
    }
    let order = order_desc(records);
    let v_max = records[order[0]].value;
    if v_max == 0.0 || !v_max.is_finite() {
        return None;
    }

    let mut acc = [0.0; N_CLASSES];
    let mut sumw = [0.0; N_CLASSES];
    let mut cap = [0.0; N_CLASSES];
    for &i in &order {
        let r = records[i];
        let cls = r.class.min(N_CLASSES - 1);
        if cap[cls] < CLASS_CAP {
            let d3 = (1.0 - r.value / v_max) * DEFICIT_SCALE;
            let w = weight(d3);
            acc[cls] += (r.value - r.base) * w;
            sumw[cls] += w;
            cap[cls] += 1.0;
        }
    }

    let mut scores = [0.0; N_CLASSES];
    let mut min_s = MIN_S_INIT;
    for c in 0..N_CLASSES {
        if sumw[c] < SUMW_EPS {
            scores[c] = EXCLUDED;
        } else {
            let avg = acc[c] / sumw[c];
            scores[c] = v_max - avg * 2.0;
            min_s = min_s.min(scores[c]);
        }
    }
    let final_score = min_s * 4.0 - v_max * 3.0;
    for s in scores.iter_mut() {
        if *s < 0.0 {
            *s = final_score;
        }
    }

    let mut accepted = Vec::new();
    let mut rejected = Vec::new();
    for (k, &i) in order.iter().enumerate() {
        let r = records[i];
        let cls = if r.class == 0 { 8 } else { r.class.min(N_CLASSES - 1) };
        if scores[cls] <= r.value {
            accepted.push(i);
        } else if r.value < min_s {
            // 정렬돼 있으므로 이후 전부 거절이다.
            rejected.extend(order[k..].iter().copied());
            break;
        } else {
            rejected.push(i);
        }
    }

    Some(GateResult { v_max, acc, sumw, scores, min_s, final_score, accepted, rejected })
}

/// 기준값 모드 (§3.5.5). **의도적 비활성** — 지시서가 `ref_mode_flag` 를 0 으로 고정하라고
/// 한다. 비교값 산출은 확정됐지만 밀도 영향이 미측정이라, 게이트 미달 시 재검토 1순위로
/// 남긴다. 함수는 회귀 시험을 위해 둔다.
pub fn run_ref_mode(records: &[Record], ref_value: f64) -> Vec<usize> {
    let mut accepted = Vec::new();
    for i in order_desc(records) {
        let v = records[i].value;
        if (ref_value - v).abs() < REF_MATCH_EPS && v < ref_value {
            break;
        }
        accepted.push(i);
    }
    accepted
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    /// §5.9 예제 3 `w(d3)` 기대값 전수 + 회귀 ⑥(비단조).
    #[test]
    fn weight_matches_the_example_table_including_the_non_monotone_jump() {
        for (d3, w) in [
            (0.0, 0.0),
            (0.2, 0.0),
            (0.21, 0.895),
            (0.5, 0.75),
            (0.51, 1.0),
            (1.0, 1.0),
            (1.01, 0.5),
            (100.0, 0.5),
        ] {
            assert!(close(weight(d3), w), "d3={d3}: got {}, want {w}", weight(d3));
        }
    }

    fn accumulate(values: [f64; 2], base: Option<f64>) -> (f64, f64) {
        let recs: Vec<Record> = values
            .iter()
            .map(|&v| Record { base: base.unwrap_or(v), value: v, class: 1 })
            .collect();
        let g = run(&recs).unwrap();
        (g.sumw[1], g.acc[1])
    }

    /// §5.9 예제 3 경로A — `base = value` 이므로 누적은 **항상 0**.
    ///
    /// `w합` 두 행은 지시서 표와 다르다. 표는 십진 산술을 가정하지만 `0.0998 / 0.1` 과
    /// `0.0995 / 0.1` 은 이진수로 정확하지 않아, 배정밀도에서 `d3` 가 각각
    /// `0.20000000000000018`, `0.5000000000000004` 로 경계 바로 **위**에 떨어진다. 그래서
    /// 가중치가 `0.0 → 0.9`, `0.75 → 1.0` 으로 바뀐다(비단조 구간의 경계 효과).
    /// 게이트 수식에는 엡실론을 넣지 않았다 — §13 "명세의 값을 그대로". 이 두 행은 입력이
    /// 정확히 경계에 걸리는 인위적 사례이고, 누적(이 시험의 요점)은 모든 행에서 0 이다.
    #[test]
    fn path_a_accumulation_is_always_zero() {
        for (vals, sumw) in [
            ([0.1, 0.1], 0.0),
            ([0.1, 0.0999], 0.0),
            ([0.1, 0.0998], 0.8999999999999999),
            ([0.1, 0.0995], 1.0),
            ([0.1, 0.099], 0.5),
            ([0.1, 0.05], 0.5),
        ] {
            let (sw, acc) = accumulate(vals, None);
            assert!(close(sw, sumw), "{vals:?}: w합 {sw}");
            assert!(close(acc, 0.0), "{vals:?}: 누적 {acc}");
        }
    }

    /// §5.9 예제 3 경로B — `base = 전역 최고점수` 일 때만 누적이 0 이 아니다. 회귀 ②.
    #[test]
    fn path_b_accumulation_matches_the_example_table() {
        for (vals, base, acc_want) in [
            ([0.1, 0.1], 0.09, 0.0),
            ([0.1, 0.1], 0.10, 0.0),
            ([0.1, 0.099], 0.10, -0.0005),
            ([0.1, 0.05], 0.10, -0.025),
            ([0.1, 0.05], 0.09, -0.020),
        ] {
            let (_, acc) = accumulate(vals, Some(base));
            assert!(close(acc, acc_want), "{vals:?} base {base}: 누적 {acc}, 기대 {acc_want}");
        }
    }

    /// 클래스당 5개까지만 누적한다 — 6번째는 건너뛴다(탈출 아님).
    #[test]
    fn class_cap_skips_the_sixth_record() {
        let recs: Vec<Record> = (0..7)
            .map(|k| Record { base: 0.1, value: 0.1 - 0.02 * k as f64, class: 2 })
            .collect();
        let g = run(&recs).unwrap();
        // 누적된 것은 value 상위 5개: d3 = 0,20,40,60,80% → w = 0,.5,.5,.5,.5
        assert!(close(g.sumw[2], 2.0));
    }

    /// 경로A 동률은 전부 거절된다 — §5.9 "버그가 아니라 설계".
    #[test]
    fn path_a_ties_reject_everything() {
        let recs = vec![
            Record { base: 0.1, value: 0.1, class: 8 },
            Record { base: 0.1, value: 0.1, class: 8 },
        ];
        let g = run(&recs).unwrap();
        assert!(g.accepted.is_empty());
        assert_eq!(g.min_s, MIN_S_INIT);
    }

    #[test]
    fn zero_v_max_ends_without_candidates() {
        assert!(run(&[Record { base: 0.0, value: 0.0, class: 1 }]).is_none());
        assert!(run(&[]).is_none());
    }

    #[test]
    fn ref_mode_breaks_just_below_the_reference() {
        let recs = vec![
            Record { base: 0.0, value: 0.9, class: 1 },
            Record { base: 0.0, value: 0.7995, class: 1 },
            Record { base: 0.0, value: 0.5, class: 1 },
        ];
        assert_eq!(run_ref_mode(&recs, 0.8), vec![0]);
    }
}
