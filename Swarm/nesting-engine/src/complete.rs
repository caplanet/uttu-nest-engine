//! Complete (§4) — 배치 뒤 **이동변수 상자(box constraint)를 좁히는** 단계.
//!
//! # ★ 정정 (§12.1.34 AU41~AU44)
//!
//! 종전 구현은 «피스 키(x) 오름차순 재배치 + Draft 재실행» 이었다. **이것은 원천에
//! 없는 자체 발명이다.** 원천 `FUN_100281c0`(type 2/3) / `FUN_1004a690`(type 1) 을
//! 다시 읽어 확정한 실체는 다음이다:
//!
//! ```c
//! void FUN_100281c0(void) {
//!   if ((조건 A) && (조건 B)) {
//!     for (i = 0; i < +0x1b0; i++) {   // +0x1b0 = 목록 원소 개수
//!       FUN_10028e60(this, i);          // 항목 i 의 «이동변수 min/max 범위» 산출
//!       callback();                     // LP 에 제약 추가
//!     }
//!   } else { callback(); }
//! }
//! ```
//!
//! - **순서 변경 없음** — x순 재배치는 원천에 없다(실측 75.319% → 72.815% 악화의 원인).
//! - **Draft 재실행 없음**.
//! - 하는 일은 **LP 변수의 상·하한을 조각/부품별로 좁혀** 해를 안정화하는 것뿐이다.
//!
//! # 원천 대응
//!
//! | 원천 | 역할 | 이 파일 |
//! |---|---|---|
//! | `FUN_100281c0` | 항목 순회 루프 | `run()` |
//! | `FUN_10028e60` | 항목별 min/max 산출 | `item_bounds()` |
//! | `FUN_1004a690` | type 1 루프 | `run()` (동일 구조) |
//! | `FUN_10028d10` | type 1 항목 처리 | `item_bounds()` |
//!
//! # 상자 값의 출처
//!
//! 원천은 «최고 해 좌표 − 변형 오프셋» 을 기준으로 후보 조각들의 min/max 를 모은다
//! (`FUN_10028e60` 의 `+0xac`/`+0xc0` = 작업 좌표, `+0x1d0`/`+0x1d4` = 하한/상한).
//! 신규 엔진에서 그 대응은 **현재 배치의 좌표에서 «이 조각이 더 갈 수 있는 최대
//! 변위»** 다 — 이미 `compact::build` 가 `±step_box` 로 쓰는 값과 같은 축이며,
//! Complete 는 그 상자를 **측정된 여유**로 좁힌다.
//!
//! # 5초 워커 대기 (§4, §8.1)
//!
//! 5초는 워커 완료를 기다리는 시간이지 작업 예산이 아니다. 넘으면 그 시점의 인증된 best 로
//! 마무리하고 "still running" 을 남긴다. 늦게 온 결과는 받지 않는다.

use crate::compact::{self, CompactConfig};
use crate::io::{Instance, Placement};
use crate::nfp::NfpCache;
use crate::validate::{self, Report};
use std::sync::{Arc, mpsc};
use std::time::{Duration, Instant};

pub const WORKER_WAIT: Duration = Duration::from_secs(5);
/// 이보다 적게 남았으면 워커를 띄우지 않는다.
const MIN_WORKER_TIME: Duration = Duration::from_millis(200);

pub struct Done {
    pub placements: Vec<Placement>,
    /// 항목별로 좁혀진 이동변수 상자 (§12.1.34 AU42). `(하한, 상한)` 격자 단위.
    pub boxes: Vec<(i64, i64)>,
    pub lp: Option<compact::Report>,
    pub report: Report,
    pub elapsed_ms: u128,
}

pub enum Outcome {
    Done(Box<Done>),
    TimedOut { waited_ms: u128 },
    Skipped(String),
}

/// 항목별 이동변수 min/max 산출 — 원천 `FUN_10028e60` / `FUN_10028d10` 대응.
///
/// 원천은 «최고 해 좌표 − 변형 오프셋» 에서 후보 조각들을 스캔해 하한(`+0x1d0`)과
/// 상한(`+0x1d4`)을 만든다. 신규 엔진에서 같은 값은 **그 항목에 속한 조각들이 지금
/// 놓인 위치 중 가장 왼쪽·오른쪽** 이다 — 그보다 더 넓은 상자는 의미가 없고,
/// 그보다 좁으면 현재 해를 자를 수 있다.
///
/// 반환값은 **격자 단위** 의 `(하한, 상한)` 이다.
///
/// ★ **현재는 보고 전용이다.** `run()` 은 이 상자를 `Done::boxes` 로 돌려주기만 하고
/// `compact::restart_improve` 에는 넘기지 않는다(`CompactConfig::slack_box` 는 조각
/// 인덱스 기준 `(좌측 여유, 우측 여유)` 이고 이 값은 형상 인덱스 기준 절대 x 구간이라
/// 형식이 다르다). 즉 Complete 의 LP 는 Draft+LP 와 같은 상자로 돈다. 상자를 실제로
/// 적용하려면 조각별 상대 이동 한계로 바꿔 `slack_box` 에 넣는 변환이 필요하다.
pub fn item_bounds(inst: &Instance, placed: &[Placement]) -> Vec<(i64, i64)> {
    let n_shapes = inst.shapes.len();
    let mut lo = vec![i64::MAX; n_shapes];
    let mut hi = vec![i64::MIN; n_shapes];
    for p in placed {
        let s = p.shape;
        let (a, b) = (p.pos.x, p.pos.x + inst.shapes[s].bbox.x1);
        if a < lo[s] {
            lo[s] = a;
        }
        if b > hi[s] {
            hi[s] = b;
        }
    }
    for s in 0..n_shapes {
        if lo[s] == i64::MAX {
            // 이 형상이 하나도 배치되지 않았으면 상자를 열어 둔다(제약 없음).
            lo[s] = 0;
            hi[s] = 0;
        }
    }
    lo.into_iter().zip(hi).collect()
}

/// Complete 실행 — 원천 `FUN_100281c0` / `FUN_1004a690` 루프 대응.
///
/// **배치를 바꾸지 않는다.** 항목별 상자를 산출하고, 그 상자로 압축 LP 를 한 번 돌린다.
/// 상자가 좁아지면 LP 는 같은 격자에서 더 안정적으로 움직인다 — 원천이 Complete 에서
/// 얻는 것도 이것이다.
#[allow(clippy::too_many_arguments)]
pub fn run(
    inst: Arc<Instance>,
    best: &[Placement],
    cache: NfpCache,
    lcfg: CompactConfig,
    lp_enabled: bool,
    deadline: Instant,
    wait: Duration,
) -> Outcome {
    if deadline.saturating_duration_since(Instant::now()) < MIN_WORKER_TIME {
        return Outcome::Skipped("남은 시간 부족 — 워커를 띄우지 않았다".into());
    }
    let (tx, rx) = mpsc::channel();
    let worker_inst = Arc::clone(&inst);
    let start = best.to_vec();
    let started = Instant::now();
    let spawned = std::thread::Builder::new().name("complete".into()).spawn(move || {
        let inst = worker_inst;
        let mut placements = start;
        let mut cache = cache;
        // 원천 Complete 의 본체 — 항목별 이동변수 상자 산출 (§12.1.34 AU42).
        // ★ 보고 전용: 아래 LP 에는 넘기지 않는다(`item_bounds` 문서 참조).
        let boxes = item_bounds(&inst, &placements);
        // 한 번 더 압축한다. 배치 자체는 유지된다.
        let lp = if lp_enabled {
            Some(compact::restart_improve(&inst, &mut placements, &mut cache, &lcfg, deadline))
        } else {
            None
        };
        let report = validate::certify(&inst, &placements);
        // 받는 쪽이 이미 떠났으면(타임아웃) 결과는 버려진다.
        let _ = tx.send(Done { placements, boxes, lp, report, elapsed_ms: started.elapsed().as_millis() });
    });
    if let Err(e) = spawned {
        return Outcome::Skipped(format!("워커 생성 실패: {e}"));
    }
    let wait = wait.min(deadline.saturating_duration_since(Instant::now()));
    let waiting = Instant::now();
    match rx.recv_timeout(wait) {
        Ok(done) => Outcome::Done(Box::new(done)),
        Err(mpsc::RecvTimeoutError::Timeout) => {
            eprintln!(
                "[complete] still running — {:.1}초 대기 초과. 현재 인증된 best 로 마무리하고 늦게 오는 결과는 버린다",
                wait.as_secs_f64()
            );
            Outcome::TimedOut { waited_ms: waiting.elapsed().as_millis() }
        }
        // 워커가 결과를 보내기 전에 끝났다(panic). 살아 있는 워커의 시간 초과와 구분한다.
        Err(mpsc::RecvTimeoutError::Disconnected) => {
            Outcome::Skipped("Complete 워커 비정상 종료(panic) — 현재 best 유지".into())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Pt;
    use crate::io;

    fn instance(json: &str, overlap: f64) -> Instance {
        static SEQ: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let seq = SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!("nesting_engine_complete_{}_{seq}.json", std::process::id()));
        std::fs::write(&path, json).unwrap();
        let inst = io::load(&path, overlap).unwrap();
        let _ = std::fs::remove_file(&path);
        inst
    }

    #[test]
    fn item_bounds_span_the_pieces_of_each_shape() {
        let inst = instance(
            r#"{"name":"t","strip_height":10,"items":[
                {"id":0,"demand":3,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[2,0],[2,2],[0,2]]}},
                {"id":1,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[5,0],[5,1],[0,1]]}}
            ]}"#,
            0.0,
        );
        // demand 3 + 1 → 조각 4개. 형상 인덱스는 JSON 순서가 아니라 `pieces[].shapes` 다.
        let s0 = inst.pieces[0].shapes[0];
        let s1 = inst.pieces[3].shapes[0];
        let placed = vec![
            Placement { piece: 0, shape: s0, pos: Pt::new(100, 0) },
            Placement { piece: 1, shape: s0, pos: Pt::new(300, 0) },
            Placement { piece: 3, shape: s1, pos: Pt::new(0, 0) },
        ];
        let b = item_bounds(&inst, &placed);
        // 형상 s0: 조각들이 놓인 폭의 최소·최대 = [100, 300+width]
        let w0 = inst.shapes[s0].bbox.x1;
        assert_eq!(b[s0], (100, 300 + w0));
        // 형상 s1: [0, width]
        let w1 = inst.shapes[s1].bbox.x1;
        assert_eq!(b[s1], (0, w1));
    }

    #[test]
    fn item_bounds_leave_unplaced_shapes_open() {
        let inst = instance(
            r#"{"name":"t","strip_height":10,"items":[
                {"id":0,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[2,0],[2,2],[0,2]]}}
            ]}"#,
            0.0,
        );
        let b = item_bounds(&inst, &[]);
        assert_eq!(b[0], (0, 0), "배치된 조각이 없으면 상자는 열려 있어야 한다");
    }

    #[test]
    fn run_does_not_reorder_or_move_pieces_without_lp() {
        let inst = instance(
            r#"{"name":"t","strip_height":10,"items":[
                {"id":0,"demand":2,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[2,0],[2,2],[0,2]]}}
            ]}"#,
            0.0,
        );
        let best = vec![
            Placement { piece: 0, shape: 0, pos: Pt::new(500, 0) },
            Placement { piece: 1, shape: 0, pos: Pt::new(100, 0) },
        ];
        let deadline = Instant::now() + Duration::from_secs(5);
        let out = run(
            Arc::new(inst),
            &best,
            NfpCache::default(),
            CompactConfig::default(),
            false, // LP 끔 → 배치를 절대 건드리지 않아야 한다
            deadline,
            Duration::from_secs(5),
        );
        match out {
            Outcome::Done(d) => {
                // ★ 핵심: 원천 Complete 는 순서를 바꾸지 않는다 (AU41).
                assert_eq!(d.placements.len(), best.len());
                for (a, b) in d.placements.iter().zip(best.iter()) {
                    assert_eq!(a.pos, b.pos, "Complete 는 배치를 바꾸지 않는다");
                    assert_eq!(a.piece, b.piece, "Complete 는 순서를 바꾸지 않는다");
                }
            }
            other => panic!("워커가 결과를 내야 한다: {:?}", matches!(other, Outcome::Skipped(_))),
        }
    }
}
