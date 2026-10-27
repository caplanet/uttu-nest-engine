//! 모듈 7 — NFP 경계 배치 (후보 평가 루프).
//!
//! 설계 §모듈 7 을 그대로 옮긴다.
//!
//! ```text
//!   1. 경계 데이터 준비
//!   2. 기준 피스(부착 대상) 위치 설정
//!   3. 후보 루프 (탐색 상한 10000):
//!      - 후보 위치 생성: 평가 레코드 쌍 기반 — 레코드 쌍의 bbox 교차 검사 ->
//!        접촉/근접 쌍을 골라 거리²을 가중치로 워크 리스트에 등록, 워크 리스트를
//!        순회하며 방향 플래그(4비트)로 상태를 관리하면서 표면별 배치를 적용.
//!        별도의 서브샘플링 개수 상수는 없다 (후보 = 접촉 쌍 전부).
//!      - 접촉 윤곽 유지: 배치 상태의 점 리스트들을 재귀 병합(방향 플래그로
//!        순/역방향 연결)한 접촉 경계 폴리라인으로 조립
//!      - 표면별 점수 기여 누적
//!      - 기하 변환·교차 검사
//!      - 회전: 조각별 회전 레코드 순회 (+ 자유회전 0.01° 미세 스텝)
//!      - 절단 규칙: 상한 초과 시 조용히 절단 — 최선 유지, 재정렬·부분 재개 없음
//!      - 현재 최선과 비교, 열등 후보 제거
//!   4. 확정: 좌표를 피스 레코드에 기록
//! ```
//!
//! # "N개를 고른다" 가 아니다
//!
//! 문서가 이 점을 명시적으로 정정한다 — 후보는 경계에서 **정점을 몇 개 샘플링**하는
//! 것이 아니라 **접촉 쌍 전부**다. 서브샘플링 상수가 없다. 그래서 후보 수를 제한하는
//! 유일한 장치가 탐색 상한(10000)이고, 그 상한은 **조용히 절단**한다.

use crate::refplace::params::{CANDIDATE_PREALLOC, EVAL_ARRAY_PREALLOC, PROBE_LIMIT};
use crate::refplace::record::{ContactPoint, RotationEntry, RotationList};

// ────────────────────────────────────────────────────────────────────────────
// 평가 레코드 (§모듈 7-3 후보 위치 생성)
// ────────────────────────────────────────────────────────────────────────────

/// 평가 레코드 — 표면 또는 NFP 경계의 점 리스트 하나.
///
/// 후보는 이 레코드들의 **쌍**에서 나온다. 레코드 하나가 곧 하나의 폴리라인이고,
/// bbox 는 쌍 사전 필터용이다.
#[derive(Clone, Debug, PartialEq)]
pub struct EvalRecord {
    /// 점 목록.
    pub points: Vec<ContactPoint>,
    /// 사전 필터용 bbox `(x_min, y_min, x_max, y_max)`.
    pub bbox: (f64, f64, f64, f64),
}

impl EvalRecord {
    /// 점 목록에서 bbox 를 계산해 만든다.
    pub fn new(points: Vec<ContactPoint>) -> Self {
        let mut bbox = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for p in &points {
            bbox.0 = bbox.0.min(p.x);
            bbox.1 = bbox.1.min(p.y);
            bbox.2 = bbox.2.max(p.x);
            bbox.3 = bbox.3.max(p.y);
        }
        if points.is_empty() {
            bbox = (0.0, 0.0, 0.0, 0.0);
        }
        Self { points, bbox }
    }

    /// bbox 가 겹치는가 — 쌍 사전 필터.
    ///
    /// 이건 **의도적 근사**다. 여기서 통과한 쌍만 선분 거리 계산으로 넘어간다.
    /// bbox 겹침을 "접촉" 으로 착각하면 안 되고, bbox 가 안 겹치는데 접촉하는 경우도
    /// 없으므로(bbox 는 상위집합) 이 필터는 안전하다.
    pub fn bbox_overlaps(&self, other: &Self, slack: f64) -> bool {
        self.bbox.0 <= other.bbox.2 + slack
            && other.bbox.0 <= self.bbox.2 + slack
            && self.bbox.1 <= other.bbox.3 + slack
            && other.bbox.1 <= self.bbox.3 + slack
    }
}

/// 워크 리스트 항목 — 접촉/근접 쌍 하나.
///
/// 설계: "접촉/근접 쌍을 골라 **거리²을 가중치로** 워크 리스트에 등록".
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WorkItem {
    /// 레코드 A 인덱스.
    pub a: usize,
    /// 레코드 B 인덱스.
    pub b: usize,
    /// 거리 제곱 — 가중치. **제곱근을 쓰지 않는다**(설계 그대로, 그리고 더 싸다).
    pub dist2: f64,
}

/// 두 레코드의 최소 거리 제곱.
///
/// 점 대 점으로 잰다. 선분 대 선분이 더 정확하지만, 이 값의 쓰임은 **워크 리스트의
/// 순서를 정하는 가중치**이지 기하 판정이 아니다 — 판정은 뒤의 교차 검사가 한다.
pub fn min_distance_squared(a: &EvalRecord, b: &EvalRecord) -> f64 {
    let mut best = f64::MAX;
    for p in &a.points {
        for q in &b.points {
            let (dx, dy) = (p.x - q.x, p.y - q.y);
            let d2 = dx * dx + dy * dy;
            if d2 < best {
                best = d2;
            }
        }
    }
    if best == f64::MAX { 0.0 } else { best }
}

/// 워크 리스트를 만든다 — bbox 사전 필터를 통과한 쌍만, 거리² 가중치와 함께.
///
/// 순서는 **거리² 오름차순**이다(가까운 쌍이 먼저). 동률이면 인덱스 쌍으로 갈라
/// 전순서를 만든다 — 그래야 같은 입력이 같은 배치를 낸다.
pub fn build_work_list(records: &[EvalRecord], slack: f64) -> Vec<WorkItem> {
    let mut work = Vec::with_capacity(CANDIDATE_PREALLOC);
    // 쌍이 최대 하나뿐이면 가중치는 순서를 정할 일이 없다 — 점 대 점 O(n·m) 거리
    // 계산을 건너뛴다. 결과(워크 리스트의 순서)는 같다.
    let single_pair = records.len() <= 2;
    for i in 0..records.len() {
        for j in (i + 1)..records.len() {
            if !records[i].bbox_overlaps(&records[j], slack) {
                continue;
            }
            work.push(WorkItem {
                a: i,
                b: j,
                dist2: if single_pair {
                    0.0
                } else {
                    min_distance_squared(&records[i], &records[j])
                },
            });
        }
    }
    work.sort_by(|x, y| {
        x.dist2
            .total_cmp(&y.dist2)
            .then_with(|| (x.a, x.b).cmp(&(y.a, y.b)))
    });
    work
}

// ────────────────────────────────────────────────────────────────────────────
// 접촉 윤곽 재귀 병합 (§모듈 7-3)
// ────────────────────────────────────────────────────────────────────────────

/// 방향 플래그를 따라 점 리스트들을 하나의 접촉 경계 폴리라인으로 병합한다.
///
/// 설계: "배치 상태의 점 리스트들을 **재귀 병합**(방향 플래그로 순방향/역방향 연결)한
/// 접촉 경계 폴리라인으로 조립해 두고".
///
/// # 재귀가 아니라 반복으로 쓴다
///
/// 의미는 재귀 병합이지만 구현은 명시적 스택이다. 조각 156개 규모에서 폴리라인이
/// 길어지면 재귀 깊이가 그대로 스택 깊이가 되고, 배치 도중 스택 오버플로가 나면
/// 그때까지의 배치를 통째로 잃는다. 결과는 같다.
pub fn merge_contours(mut segments: Vec<Vec<ContactPoint>>) -> Vec<ContactPoint> {
    if segments.is_empty() {
        return Vec::new();
    }
    // 첫 조각으로 시작한다. `CONSUMED` 로 표시해 두 번 쓰지 않는다.
    let mut out = segments.remove(0);
    let mut used = vec![false; segments.len()];
    let mut progress = true;
    while progress {
        progress = false;
        for (i, seg) in segments.iter().enumerate() {
            if used[i] || seg.is_empty() {
                continue;
            }
            let (Some(tail), Some(head)) = (out.last(), seg.first()) else {
                continue;
            };
            let (Some(out_head), Some(seg_tail)) = (out.first(), seg.last()) else {
                continue;
            };
            // 순방향 연결 — out 의 끝이 seg 의 앞과 만난다.
            if same_point(tail, head) {
                out.extend(seg.iter().skip(1).copied());
                used[i] = true;
                progress = true;
            // 역방향 연결 — out 의 끝이 seg 의 **끝**과 만나면 뒤집어 붙인다.
            } else if same_point(tail, seg_tail) {
                out.extend(seg.iter().rev().skip(1).copied());
                used[i] = true;
                progress = true;
            // out 의 앞에 붙는 경우도 본다 — 조각이 어느 방향으로 들어올지 모른다.
            } else if same_point(out_head, seg_tail) {
                let mut merged = seg.clone();
                merged.pop();
                merged.extend(out.iter().copied());
                out = merged;
                used[i] = true;
                progress = true;
            } else if same_point(out_head, head) {
                let mut merged: Vec<ContactPoint> = seg.iter().rev().copied().collect();
                merged.pop();
                merged.extend(out.iter().copied());
                out = merged;
                used[i] = true;
                progress = true;
            }
        }
    }
    // 병합에 쓰인 점에 표시를 남긴다 — 다음 라운드가 같은 점을 다시 잇지 않도록.
    for p in out.iter_mut() {
        p.flags |= ContactPoint::CONSUMED;
    }
    if let Some(first) = out.first_mut() {
        first.flags |= ContactPoint::TERMINAL;
    }
    if let Some(last) = out.last_mut() {
        last.flags |= ContactPoint::TERMINAL;
    }
    out
}

/// 두 접촉점이 같은 자리인가.
///
/// 허용오차는 **mm 절대값**이다. 이 좌표들은 NFP 경계에서 나온 f64 라 f32 잡음이 없고,
/// 폴리라인을 잇는 판정이므로 기하 해상도(재단 공차 1mm 급)보다 훨씬 작으면 된다.
fn same_point(a: &ContactPoint, b: &ContactPoint) -> bool {
    const JOIN_EPS: f64 = 1.0e-6;
    (a.x - b.x).abs() <= JOIN_EPS && (a.y - b.y).abs() <= JOIN_EPS
}

// ────────────────────────────────────────────────────────────────────────────
// 후보 평가 루프 (§모듈 7-3)
// ────────────────────────────────────────────────────────────────────────────

/// 평가된 후보 하나.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoundaryCandidate {
    /// 앵커 좌표(mm).
    pub x: f64,
    pub y: f64,
    /// 적용한 자세.
    pub rotation: RotationEntry,
    /// 자유회전으로 더한 각도(라디안).
    pub extra_rad: f64,
    /// 이 후보의 점수. 클수록 좋다 (§4-2 최종 점수).
    pub score: f64,
}

/// 후보 루프의 결과.
#[derive(Clone, Debug, Default)]
pub struct BoundaryOutcome {
    /// 최선 후보. 하나도 못 만들었으면 `None`.
    pub best: Option<BoundaryCandidate>,
    /// 실제로 평가한 후보 수.
    pub evaluated: usize,
    /// 탐색 상한에 걸려 **절단**되었는가.
    ///
    /// 절단은 오류가 아니다 — 설계가 정한 정상 동작이다. 다만 절단됐다는 사실은
    /// 알려야 한다. 그러지 않으면 "후보를 다 봤다" 와 구분할 수 없다.
    pub truncated: bool,
}

/// 후보 평가 루프 (§모듈 7-3).
///
/// `poses` 는 조각의 회전 목록이다. 자유회전이 켜져 있으면 엔트리마다
/// [`RotationList::micro_steps`] 로 미세 스텝을 더 뻗는다.
///
/// `evaluate` 는 `(x, y, 자세, 추가각)` 에 대해 점수를 돌려준다. `None` 이면
/// 그 후보는 기하 검사(교차)에서 탈락한 것이다.
///
/// # 절단 규칙
///
/// 평가 수가 [`PROBE_LIMIT`] 에 닿으면 **조용히 멈춘다**. 재정렬도 부분 재개도 없고,
/// 그때까지의 최선을 그대로 돌려준다. 설계 §모듈 7 이 정한 그대로다.
pub fn evaluate_candidates(
    work: &[WorkItem],
    records: &[EvalRecord],
    poses: &RotationList,
    free_span_rad: f64,
    mut evaluate: impl FnMut(f64, f64, &RotationEntry, f64) -> Option<f64>,
) -> BoundaryOutcome {
    let mut out = BoundaryOutcome::default();
    let mut budget = PROBE_LIMIT;

    'work: for item in work {
        // 쌍의 **두 레코드 모두**에서 접촉 위치 후보를 만든다 — 각 점이 곧 앵커 후보다.
        //
        // "정점을 N개 고른다" 가 아니라 **접촉 쌍 전부**다(설계가 명시적으로 정정한
        // 부분이고, 서브샘플링 상수가 없는 이유다). 한쪽 레코드만 보면 상대 쪽 경계에만
        // 있는 접촉 자세를 통째로 놓친다.
        let (Some(ra), Some(rb)) = (records.get(item.a), records.get(item.b)) else {
            continue;
        };
        // 자기 자신과의 쌍(`a == b`)이면 같은 점 목록을 두 번 돌 이유가 없다 —
        // 두 번째 순회는 첫 번째와 같은 후보를 내면서 예산만 깎는다.
        let rb_points: &[ContactPoint] = if item.a == item.b { &[] } else { &rb.points };
        for anchor in ra.points.iter().chain(rb_points.iter()) {
            for pose in poses.entries() {
                for extra in poses.micro_steps(free_span_rad) {
                    if budget == 0 {
                        out.truncated = true;
                        break 'work;
                    }
                    budget -= 1;
                    out.evaluated += 1;
                    let Some(score) = evaluate(anchor.x, anchor.y, pose, extra) else {
                        continue;
                    };
                    // 현재 최선과 비교, 열등 후보 제거.
                    //
                    // **엄격히 더 좋을 때만** 갈아탄다. 동률에 갈아타면 훑는 순서가
                    // 결과를 바꾸고, 그러면 같은 입력이 다른 배치를 낸다.
                    let better = match out.best {
                        None => true,
                        Some(b) => score > b.score,
                    };
                    if better {
                        out.best = Some(BoundaryCandidate {
                            x: anchor.x,
                            y: anchor.y,
                            rotation: *pose,
                            extra_rad: extra,
                            score,
                        });
                    }
                }
            }
        }
    }
    out
}

/// 평가 배열의 초기 용량 (§5 평가배열 초기 20000).
pub fn eval_array_capacity() -> usize {
    EVAL_ARRAY_PREALLOC
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: f64, y: f64) -> ContactPoint {
        ContactPoint::new(x, y)
    }

    fn rec(points: Vec<(f64, f64)>) -> EvalRecord {
        EvalRecord::new(points.into_iter().map(|(x, y)| pt(x, y)).collect())
    }

    /// bbox 사전 필터는 **상위집합**이라 접촉 쌍을 놓치지 않는다.
    #[test]
    fn the_bbox_prefilter_never_drops_a_touching_pair() {
        let a = rec(vec![(0.0, 0.0), (10.0, 10.0)]);
        let b = rec(vec![(10.0, 10.0), (20.0, 20.0)]); // 한 점을 공유한다
        assert!(a.bbox_overlaps(&b, 0.0));
        let far = rec(vec![(100.0, 100.0), (110.0, 110.0)]);
        assert!(!a.bbox_overlaps(&far, 0.0));
        // slack 을 주면 근접 쌍도 통과한다.
        let near = rec(vec![(10.5, 10.5), (12.0, 12.0)]);
        assert!(!a.bbox_overlaps(&near, 0.0));
        assert!(a.bbox_overlaps(&near, 1.0));
    }

    /// 워크 리스트는 거리² 오름차순이고, 동률은 인덱스로 갈라 **전순서**다.
    #[test]
    fn the_work_list_is_ordered_by_squared_distance_with_a_total_tiebreak() {
        let records = vec![
            rec(vec![(0.0, 0.0)]),
            rec(vec![(1.0, 0.0)]),
            rec(vec![(2.0, 0.0)]),
        ];
        let work = build_work_list(&records, 10.0);
        assert_eq!(work.len(), 3);
        // (0,1)=1, (1,2)=1, (0,2)=4
        assert_eq!((work[0].a, work[0].b), (0, 1));
        assert_eq!((work[1].a, work[1].b), (1, 2));
        assert_eq!((work[2].a, work[2].b), (0, 2));
        assert!(work[0].dist2 <= work[1].dist2 && work[1].dist2 <= work[2].dist2);
    }

    /// 거리 가중치는 **제곱근을 쓰지 않는다** — 설계 그대로이고 더 싸다.
    #[test]
    fn the_weight_is_squared_distance_not_distance() {
        let a = rec(vec![(0.0, 0.0)]);
        let b = rec(vec![(3.0, 4.0)]);
        assert_eq!(min_distance_squared(&a, &b), 25.0, "5²=25 이지 5 가 아니다");
    }

    /// 순방향·역방향 연결이 모두 병합된다.
    #[test]
    fn contours_merge_in_both_directions() {
        // A: (0,0)->(1,0),  B 는 뒤집혀 들어온다: (2,0)->(1,0)
        let a = vec![pt(0.0, 0.0), pt(1.0, 0.0)];
        let b = vec![pt(2.0, 0.0), pt(1.0, 0.0)];
        let merged = merge_contours(vec![a, b]);
        let xs: Vec<f64> = merged.iter().map(|p| p.x).collect();
        assert_eq!(xs, vec![0.0, 1.0, 2.0], "역방향 조각도 이어져야 한다");
    }

    /// 앞쪽으로 붙는 조각도 병합된다.
    #[test]
    fn a_segment_that_joins_at_the_head_is_merged_too() {
        let a = vec![pt(1.0, 0.0), pt(2.0, 0.0)];
        let b = vec![pt(0.0, 0.0), pt(1.0, 0.0)];
        let merged = merge_contours(vec![a, b]);
        let xs: Vec<f64> = merged.iter().map(|p| p.x).collect();
        assert_eq!(xs, vec![0.0, 1.0, 2.0]);
    }

    /// 병합된 점에는 `CONSUMED` 표시가 남고, 양 끝은 `TERMINAL` 이다.
    #[test]
    fn merged_points_are_marked_so_they_are_not_reused() {
        let merged = merge_contours(vec![vec![pt(0.0, 0.0), pt(1.0, 0.0)]]);
        assert!(merged.iter().all(|p| p.has(ContactPoint::CONSUMED)));
        assert!(merged.first().unwrap().has(ContactPoint::TERMINAL));
        assert!(merged.last().unwrap().has(ContactPoint::TERMINAL));
    }

    /// 이어지지 않는 조각은 **버려지지 않고** 그냥 안 붙는다.
    #[test]
    fn a_disjoint_segment_simply_does_not_join() {
        let a = vec![pt(0.0, 0.0), pt(1.0, 0.0)];
        let far = vec![pt(50.0, 50.0), pt(51.0, 50.0)];
        let merged = merge_contours(vec![a, far]);
        assert_eq!(merged.len(), 2, "붙지 않는 조각은 이 폴리라인에 들어오지 않는다");
    }

    /// **탐색 상한 10000 에서 조용히 절단**하고, 그때까지의 최선을 유지한다.
    #[test]
    fn the_loop_truncates_silently_at_the_probe_limit_and_keeps_the_best() {
        // 레코드 30개 x 점 30개 -> 워크 쌍 435개 x 앵커 60개 = 26,100 후보.
        // 상한(10,000)을 확실히 넘긴다.
        let records: Vec<EvalRecord> = (0..30)
            .map(|k| rec((0..30).map(|i| (i as f64, k as f64)).collect()))
            .collect();
        let work = build_work_list(&records, 1000.0);
        let poses = RotationList::fixed();
        let mut n = 0;
        let out = evaluate_candidates(&work, &records, &poses, 0.0, |x, _, _, _| {
            n += 1;
            Some(x) // x 가 클수록 좋다
        });
        assert!(out.truncated, "상한에 걸려야 한다");
        assert_eq!(out.evaluated, PROBE_LIMIT);
        assert_eq!(n, PROBE_LIMIT, "상한을 넘겨 평가하면 안 된다");
        assert!(out.best.is_some(), "절단돼도 그때까지의 최선은 남는다");
    }

    /// 기하 검사에서 탈락한 후보(`None`)는 최선을 갈아치우지 못한다.
    #[test]
    fn a_candidate_rejected_by_geometry_never_becomes_the_best() {
        let records = vec![rec(vec![(5.0, 0.0)]), rec(vec![(6.0, 0.0)])];
        let work = build_work_list(&records, 10.0);
        let out = evaluate_candidates(&work, &records, &RotationList::fixed(), 0.0, |_, _, _, _| {
            None
        });
        assert!(out.best.is_none());
        assert!(out.evaluated > 0, "평가는 했다");
        assert!(!out.truncated);
    }

    /// **동률에서는 갈아타지 않는다** — 훑는 순서가 결과를 바꾸면 안 된다.
    #[test]
    fn ties_do_not_replace_the_incumbent() {
        let records = vec![
            rec(vec![(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]),
            rec(vec![(1.5, 0.0)]),
        ];
        let work = build_work_list(&records, 10.0);
        let out = evaluate_candidates(&work, &records, &RotationList::fixed(), 0.0, |_, _, _, _| {
            Some(1.0) // 전부 동점
        });
        let best = out.best.expect(crate::dmsg!("후보가 있어야 한다"));
        assert_eq!(best.x, 1.0, "첫 후보(레코드 A 의 첫 점)를 유지해야 한다");
    }

    /// 자유회전이 켜지면 엔트리마다 **0.01° 미세 스텝**으로 더 뻗는다.
    #[test]
    fn free_rotation_fans_out_in_hundredth_degree_steps() {
        let records = vec![rec(vec![(0.0, 0.0)]), rec(vec![(1.0, 0.0)])];
        let work = build_work_list(&records, 10.0);
        // 0.05도 구간 -> 0.01도 스텝이 6개(0 포함).
        let span = 5.0 * crate::refplace::params::FREE_ROTATION_STEP_RAD;
        let free = RotationList::fixed().with_free_rotation();
        let mut seen: Vec<f64> = Vec::new();
        evaluate_candidates(&work, &records, &free, span, |_, _, _, extra| {
            seen.push(extra);
            Some(0.0)
        });
        // 앵커 2개(양쪽 레코드의 점) x 6 스텝.
        assert_eq!(seen.len(), 12, "0 을 포함해 앵커마다 6 스텝: {seen:?}");
        // 스텝 간격이 정확히 0.01도.
        let step = crate::refplace::params::FREE_ROTATION_STEP_RAD;
        assert!((seen[1] - seen[0] - step).abs() < 1.0e-15);
    }

    /// 자유회전이 꺼져 있으면 엔트리당 한 번만 평가한다.
    #[test]
    fn a_fixed_pose_list_evaluates_each_entry_once() {
        let records = vec![rec(vec![(0.0, 0.0)]), rec(vec![(1.0, 0.0)])];
        let work = build_work_list(&records, 10.0);
        let poses = RotationList::new(vec![
            RotationEntry::IDENTITY,
            RotationEntry::turn(90_000),
            RotationEntry {
                flip_y: true,
                ..RotationEntry::IDENTITY
            },
        ]);
        let mut n = 0;
        evaluate_candidates(&work, &records, &poses, 1.0, |_, _, _, _| {
            n += 1;
            Some(0.0)
        });
        // 두 레코드의 점 1개씩 = 앵커 2개 x 자세 3개.
        assert_eq!(n, 6);
    }

    /// 평가배열 초기 용량은 §5 표대로 20000.
    #[test]
    fn the_eval_array_capacity_matches_the_table() {
        assert_eq!(eval_array_capacity(), 20_000);
    }
}
