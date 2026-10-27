//! ⑦ 슬라이딩 압축 — 배치가 끝난 뒤 각 조각을 좌하단으로 다시 민다.
//!
//! # 왜 필요한가
//!
//! 순차 배치는 **되돌아갈 수 없다**. k번째 조각을 놓을 때 그 자리가 최선이었더라도,
//! k+1번째 이후가 놓이면서 왼쪽에 새 공백이 생길 수 있다. 그리디는 이미 굳은 조각을
//! 밀어낼 수 없으므로 그 공백은 끝까지 남는다.
//!
//! 이 패스는 그 비대칭을 푼다. 조각을 하나씩 **빼내고**, 나머지 전부를 장애물로 놓은
//! 상태에서 그 조각의 feasible 앵커를 다시 구한 뒤, 원래 자리보다 좌하단인 자리가
//! 있으면 옮긴다.
//!
//! # 왜 겹칠 수 없는가
//!
//! 후보는 [`SequentialPlacer::candidates_for`](crate::nfp_lbf::SequentialPlacer::candidates_for)가
//! 준 것만 쓴다. 그 함수는 "어떤 NFP의 내부에도 있지 않고 IFP 안" 인 앵커만 돌려준다.
//! 즉 겹침 여부를 이쪽에서 다시 판정하지 않는다 — 판정은 NFP 커널이 이미 했다.
//!
//! # 왜 LP 압축과 따로 두는가
//!
//! [`LpCompactionConfig`](crate::config::LpCompactionConfig)는 모든 조각을 **동시에
//! 조금씩** 민다(선형 근사라 큰 이동을 못 한다). 이쪽은 한 조각을 **크게** 옮긴다 —
//! strip 오른쪽 끝에 남은 조각을 왼쪽 구멍으로 통째로 이주시키는 것은 LP가 못 한다.
//! 두 패스는 서로를 대체하지 않는다.

use super::{AnchorOption, StrategyContext};
use crate::nfp_lbf::PlacedPiece;

/// 이동을 채택할 최소 개선폭(mm).
///
/// 앵커 좌표는 `f32`로 굳으므로 후보와 현재 위치가 1e-7mm 어긋나는 일이 흔하다.
/// 그 차이까지 "개선" 으로 세면 조각이 제자리에서 미세하게 떨리며 `moved`가 영원히
/// 0이 되지 않고, 수렴 판정(한 라운드 이동 0개)이 절대 걸리지 않는다.
/// 0.01mm는 재단 공차보다 훨씬 작아 품질에는 영향이 없다.
const MIN_GAIN_MM: f64 = 0.01;

/// "x가 같다" 로 볼 허용 오차(mm).
///
/// [`MIN_GAIN_MM`]과 목적이 다르다. 이쪽은 **동률 판정**이라 최대한 작아야 한다 —
/// 크게 잡으면 x가 실제로 늘어나는 이동까지 "같은 열에서 아래로" 로 통과한다.
const COORD_EPS_MM: f64 = 1e-6;

/// strip 길이가 "늘지 않았다" 로 볼 허용 오차(mm). f64 누산 오차만 흡수한다.
const LENGTH_EPS_MM: f64 = 1e-6;

/// 압축이 실제로 무엇을 하고 있는지 세는 계측 카운터.
///
/// 켜지 않으면 조각마다 relaxed load 한 번이다 — 조각 하나당 NFP 후보 열거가 수천 회
/// 탐침이므로 계측에 잡히지 않는다. 켜는 것은 `#[ignore]` 계측 테스트뿐이다.
///
/// # 이 카운터가 실제로 답한 것 (wsjk156, 탐침 4096, 유효 ordering 12개)
///
/// * **되돌림 0회.** 사양 5번("strip 길이가 늘면 안 된다")을 어기는 이동이 한 번도
///   나오지 않았다. 되돌리기 경로는 방어이지 상시 동작이 아니다.
/// * **이동의 90%가 x 를 줄인다**(x감소 409 / y만 43). "좌하단 사전식이라 위아래로만
///   움직이고 폭은 안 준다" 는 가설은 거짓이다 — 채택 조건이 `x < cur.x - 이득`
///   이거나 (같은 열에서) `y < cur.y - 이득` 이고, 고를 때도 `min(x)` 가 먼저다.
/// * **라운드가 2에서 끊기지 않는다.** 포켓 우선 탐침으로 바꾸기 전에는 R1=257,
///   R2=195 로 2라운드에서도 이동이 거의 줄지 않았다. 바꾼 뒤에는
///   R1=229, R2=83, R3=22, R4=9 로 떨어지고 **밀도는 2라운드에서 이미 최종값**이다
///   (라운드 2/4/8 모두 72.11%). 즉 기본 2라운드로 충분하다.
#[doc(hidden)]
pub mod meas {
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering as AO};

    /// 라운드별 이동 수를 따로 세기 위한 슬롯 수.
    pub const MAX_TRACKED_ROUNDS: usize = 16;

    pub static ON: AtomicBool = AtomicBool::new(false);
    /// `compact` 호출 수(= ordering 수).
    pub static CALLS: AtomicUsize = AtomicUsize::new(0);
    /// 조각을 하나 재배치하려고 후보를 구한 횟수.
    pub static PROBES: AtomicUsize = AtomicUsize::new(0);
    /// 그때 받은 후보 수의 합.
    pub static CANDS: AtomicUsize = AtomicUsize::new(0);
    /// 좌하단 필터를 통과한 후보 수의 합.
    pub static LOWER_LEFT: AtomicUsize = AtomicUsize::new(0);
    /// x 를 실제로 줄인 이동 수.
    pub static MOVED_X: AtomicUsize = AtomicUsize::new(0);
    /// 같은 열에서 아래로만 내려간 이동 수.
    pub static MOVED_Y: AtomicUsize = AtomicUsize::new(0);
    /// strip 이 늘어 되돌린 횟수.
    pub static REVERTS: AtomicUsize = AtomicUsize::new(0);
    /// 라운드 인덱스별 이동 수.
    pub static ROUND_MOVED: [AtomicUsize; MAX_TRACKED_ROUNDS] =
        [const { AtomicUsize::new(0) }; MAX_TRACKED_ROUNDS];

    pub fn set(on: bool) {
        ON.store(on, AO::Relaxed);
    }
    pub fn is_on() -> bool {
        ON.load(AO::Relaxed)
    }
    pub fn reset() {
        for c in [
            &CALLS,
            &PROBES,
            &CANDS,
            &LOWER_LEFT,
            &MOVED_X,
            &MOVED_Y,
            &REVERTS,
        ] {
            c.store(0, AO::Relaxed);
        }
        for c in ROUND_MOVED.iter() {
            c.store(0, AO::Relaxed);
        }
    }
    fn g(c: &AtomicUsize) -> usize {
        c.load(AO::Relaxed)
    }
    pub fn report() -> String {
        let probes = g(&PROBES).max(1);
        let rounds: Vec<String> = ROUND_MOVED
            .iter()
            .enumerate()
            .filter(|(_, c)| g(c) > 0)
            .map(|(i, c)| format!("R{}={}", i + 1, g(c)))
            .collect();
        format!(
            "compact {}회 / 조각 질의 {} (후보 평균 {:.1}개, 좌하단 후보 평균 {:.2}개)\n\
             이동: x감소 {} + y만 {} = {} / 되돌림 {}\n\
             라운드별 이동: {}",
            g(&CALLS),
            g(&PROBES),
            g(&CANDS) as f64 / probes as f64,
            g(&LOWER_LEFT) as f64 / probes as f64,
            g(&MOVED_X),
            g(&MOVED_Y),
            g(&MOVED_X) + g(&MOVED_Y),
            g(&REVERTS),
            rounds.join(" "),
        )
    }
}

/// 슬라이딩 압축 한 번의 결과.
///
/// `PartialEq`를 달지 않는 이유는 [`PlacedPiece`]가 `PartialEq`가 아니기 때문이다.
/// 배치 두 개의 동치 판정은 좌표 반올림 때문에 어차피 허용 오차가 필요하다.
#[derive(Clone, Debug, Default)]
pub struct SlidingOutcome {
    /// 압축 후 배치. 입력과 조각 수·`item_id`·`rotation`이 모두 같고 위치만 바뀐다.
    pub pieces: Vec<PlacedPiece>,
    /// 압축 전 strip 길이(mm).
    pub before: f64,
    /// 압축 후 strip 길이(mm).
    ///
    /// 정의상 `before` 이하다 — 다만 채택 판정이 [`LENGTH_EPS_MM`] 허용 오차를 쓰므로
    /// 엄밀하게는 `before + LENGTH_EPS_MM` 이하다. 라운드를 아무리 돌아도 이 오차가
    /// 누적되지는 않는다([`compact`]의 `strip` 갱신이 단조 비증가로 잠겨 있다).
    pub after: f64,
    /// 실제로 옮긴 조각 수(같은 조각을 두 라운드에서 옮기면 2로 센다).
    pub moved: usize,
    /// 실제로 돈 라운드 수. 수렴하거나 예산이 끝나면 `rounds`보다 작다.
    pub rounds_run: usize,
}

impl SlidingOutcome {
    /// 압축으로 줄어든 strip 길이(mm). 음수가 될 수 없다.
    pub fn gain(&self) -> f64 {
        (self.before - self.after).max(0.0)
    }
}

/// 배치를 좌하단으로 압축한다.
///
/// `rounds`는 상한이다. 한 라운드에서 옮긴 조각이 0개면 더 돌아도 결과가 같으므로
/// 즉시 끝낸다(같은 입력 → 같은 후보 → 같은 판단이라 두 번째 라운드는 순수한 낭비다).
///
/// 예산([`StrategyContext::has_budget`])은 **조각마다** 확인한다. 라운드마다 확인하면
/// 조각 수백 개짜리 배치에서 한 라운드가 통째로 예산을 넘긴다.
///
/// 입력이 비었거나 `rounds == 0`이면 입력을 그대로 돌려준다.
pub fn compact(
    ctx: &mut StrategyContext<'_, '_>,
    pieces: &[PlacedPiece],
    rounds: usize,
) -> SlidingOutcome {
    let before = ctx.frontier(pieces);
    let mut current: Vec<PlacedPiece> = pieces.to_vec();

    if current.is_empty() || rounds == 0 {
        return SlidingOutcome {
            pieces: current,
            before,
            after: before,
            moved: 0,
            rounds_run: 0,
        };
    }

    let mut moved = 0usize;
    let mut rounds_run = 0usize;
    // 지금까지의 strip 길이. 이동이 이 값을 넘기면 되돌린다.
    let mut strip = before;

    if meas::is_on() {
        meas::CALLS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    }

    for _ in 0..rounds {
        if !ctx.has_budget() {
            break;
        }
        rounds_run += 1;

        let mut moved_this_round = 0usize;
        for idx in processing_order(&current) {
            if !ctx.has_budget() {
                break;
            }
            let Some(piece) = current.get(idx).copied() else {
                continue;
            };
            let cur = (piece.translation.0 as f64, piece.translation.1 as f64);

            // 나머지 **전부**가 장애물이다. 자기 자신을 빼지 않으면 NFP가 자기
            // 자리를 금지해 후보가 통째로 사라진다.
            let others: Vec<PlacedPiece> = current
                .iter()
                .enumerate()
                .filter(|(i, _)| *i != idx)
                .map(|(_, p)| *p)
                .collect();

            let options = options_at_rotation(ctx, piece.item_id, piece.rotation, &others);
            if meas::is_on() {
                use std::sync::atomic::Ordering as AO;
                let lower_left = options
                    .iter()
                    .filter(|o| is_lower_left((o.x, o.y), cur))
                    .count();
                meas::PROBES.fetch_add(1, AO::Relaxed);
                meas::CANDS.fetch_add(options.len(), AO::Relaxed);
                meas::LOWER_LEFT.fetch_add(lower_left, AO::Relaxed);
            }
            let Some(best) = options
                .into_iter()
                .filter(|o| is_lower_left((o.x, o.y), cur))
                .min_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)))
            else {
                continue;
            };
            let reduced_x = best.x < cur.0 - MIN_GAIN_MM;

            let Some(slot) = current.get_mut(idx) else {
                continue;
            };
            let restore = *slot;
            *slot = best.to_piece();

            // strip 길이는 이 이동으로 늘어날 수 없다(회전이 같으니 폭이 같고 x는
            // 줄거나 그대로다). 그래도 확인하는 이유는 오라클이 예상 밖의 앵커를
            // 돌려줬을 때 **품질이 조용히 나빠지는 것보다 되돌리는 편이 낫기**
            // 때문이다. 비용은 배치당 bbox 한 번 훑기다.
            let after_move = ctx.frontier(&current);
            if after_move > strip + LENGTH_EPS_MM {
                if let Some(slot) = current.get_mut(idx) {
                    *slot = restore;
                }
                if meas::is_on() {
                    meas::REVERTS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                }
                continue;
            }
            if meas::is_on() {
                use std::sync::atomic::Ordering as AO;
                if reduced_x {
                    meas::MOVED_X.fetch_add(1, AO::Relaxed);
                } else {
                    meas::MOVED_Y.fetch_add(1, AO::Relaxed);
                }
                if let Some(slot) = meas::ROUND_MOVED.get(rounds_run.saturating_sub(1)) {
                    slot.fetch_add(1, AO::Relaxed);
                }
            }

            // `after_move` 를 그대로 넣지 않고 `min` 으로 잠그는 이유: 채택 조건이
            // `strip + LENGTH_EPS_MM` 이므로 매 이동이 기준선을 1e-6mm 씩 **올릴** 수
            // 있다. 조각 156개 x 라운드 몇 번이면 수백 번이라 기준선이 조용히 늘어난다.
            // 단조 비증가로 잠그면 "strip 은 절대 늘지 않는다" 가 라운드 수와 무관하게
            // 성립한다.
            strip = after_move.min(strip);
            moved += 1;
            moved_this_round += 1;
        }

        if moved_this_round == 0 {
            break;
        }
    }

    let after = ctx.frontier(&current);
    SlidingOutcome {
        pieces: current,
        before,
        after,
        moved,
        rounds_run,
    }
}

/// 이 라운드에서 조각을 훑을 순서(=`pieces` 인덱스), **x가 작은 것부터**.
///
/// # 왜 왼쪽부터인가
///
/// 압축은 연쇄다. 조각 A가 왼쪽으로 빠지면 A가 있던 자리가 비고, 그 오른쪽의 B가
/// 그 자리로 따라 들어갈 수 있다. 순서가 왼쪽 → 오른쪽이면 이 연쇄가 **한 라운드 안에서**
/// 전부 일어난다.
///
/// 오른쪽부터 돌면 정반대가 된다. B를 먼저 볼 때 A는 아직 원래 자리에 있어 B가 갈 곳이
/// 없고, 그 뒤에 A가 빠져도 B는 이미 지나간 뒤라 그 자리를 못 쓴다. 같은 결과를 얻으려면
/// 라운드를 조각 수만큼 돌려야 한다 — 라운드당 비용이 `O(조각 x 후보)`이므로 그건
/// 사실상 불가능한 예산이다.
///
/// x가 같으면 y가 작은 것을 먼저 본다. 같은 열에서도 아래쪽이 먼저 자리를 잡아야
/// 위쪽이 내려올 수 있다는 같은 논리다.
fn processing_order(pieces: &[PlacedPiece]) -> Vec<usize> {
    let mut order: Vec<(f64, f64, usize)> = pieces
        .iter()
        .enumerate()
        .map(|(i, p)| (p.translation.0 as f64, p.translation.1 as f64, i))
        .collect();
    // `total_cmp`를 쓰는 이유는 NaN이 섞여도 순서가 정해져 정렬이 패닉하지 않기 때문이다.
    order.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
    order.into_iter().map(|(_, _, i)| i).collect()
}

/// `(x, y)`가 `cur`보다 **엄격히 좌하단**인가 (사전식).
///
/// x 비교에만 [`COORD_EPS_MM`]이 아니라 [`MIN_GAIN_MM`]을 쓰는 쪽과 아닌 쪽이 갈린다:
/// "왼쪽으로 갔다" 는 판정은 이득이 있어야 하므로 `MIN_GAIN_MM`이고,
/// "같은 열이다" 는 판정은 동률이므로 `COORD_EPS_MM`이다. 둘을 같은 값으로 묶으면
/// x가 0.009mm **늘어나는** 이동이 "같은 열에서 아래로" 로 통과한다.
fn is_lower_left(candidate: (f64, f64), cur: (f64, f64)) -> bool {
    if !(candidate.0.is_finite() && candidate.1.is_finite()) {
        return false;
    }
    if candidate.0 < cur.0 - MIN_GAIN_MM {
        return true;
    }
    if candidate.0 > cur.0 + COORD_EPS_MM {
        return false;
    }
    candidate.1 < cur.1 - MIN_GAIN_MM
}

/// 이 조각을 **원래 회전 그대로** 놓을 수 있는 선택지들.
///
/// # 왜 회전을 고정하는가
///
/// 회전을 바꾸면 그건 압축이 아니라 재배치다. 두 가지가 깨진다.
///
/// * **결(grain)** — 원단·목재는 방향이 물성이다. 배치기가 이 조각에 0°를 준 것이
///   결 제약 때문이었다면, 후처리가 90°로 돌리는 순간 못 쓰는 결과가 된다.
/// * **평가의 의미** — 이 패스는 "같은 해를 더 조밀하게" 만드는 것으로 정의돼 있다.
///   회전까지 열면 상위 전략(⑤ 빔, ⑥ best-fit)이 이미 평가한 선택을 후처리가
///   뒤집는 셈이라, 어느 전략의 기여인지 측정할 수 없게 된다.
///
/// # 왜 `StrategyContext::options_for`를 쓰지 않는가
///
/// `options_for`는 **허용 회전 전부**를 열거한다. 우리는 그중 하나만 쓸 것이므로
/// 회전이 4개면 NFP 열거의 3/4이 버려진다 — 이 패스의 지배적 비용이 바로 그
/// 열거라서 그냥 넘길 수 있는 낭비가 아니다.
///
/// 처음에는 `ctx.item_rotations[item_id]`를 잠시 `vec![rotation]`으로 좁혀 재사용했지만
/// 두 가지가 걸렸다.
///
/// * `ctx`를 **몰래 변형**한다. 중간에 조기 반환이 하나라도 생기면 좁혀진 채로 새고,
///   그 증상은 이 패스가 아니라 뒤에 도는 전략에서 터진다.
/// * `item_rotations`가 비어 있으면(호출부가 `Vec::new()`를 주는 흔한 경우)
///   `get_mut`이 `None`이라 **좁히기가 조용히 무산된다** — 낭비는 그대로 남는데
///   코드만 안전해 보인다.
///
/// 오라클을 직접 부르면 둘 다 사라진다. `candidates_for`는 회전을 인자로 받으므로
/// 반환된 후보의 회전은 **구성상** 이 값이고, 사후 필터링도 필요 없다.
/// 프론티어는 `others` 기준이다 — 옮길 조각을 뺀 상태가 이 질의의 진짜 배치이기 때문이다.
///
/// # 왜 탐침을 **항상** 포켓 우선으로 두는가 — ④ 설정을 따르면 안 된다
///
/// 예전에는 `ctx.cfg.pocket_first`(④ 전략 플래그)를 그대로 넘겼다. ④가 꺼진 기본
/// 경로에서 오라클이 안쪽을 보는 방식은 이렇다 — `placement.rs::candidates_for` 의
/// 구간 분리를 그대로 읽은 것이다.
///
/// * 구간 1(프론티어 안쪽)이 **아예 열리지 않는다**. 기본 경로의 경계식은
///   `프론티어 - ifp_bbox.width()` 인데 `ifp_bbox` 는 조각이 아니라 **앵커 가용
///   영역**이라 폭이 프론티어보다 한 자릿수 크다(wsjk156: 51141mm 대 최대 7.3e3mm).
///   그래서 `left_end` 가 항상 0이고 `LEFT_REGION_PROBE_DIVISOR` 가지는 실행되지
///   않는다 — placement.rs 가 "기본 경로의 구간 1은 지금 죽은 코드" 라고 주석으로
///   못박아 둔 그대로다(기준선 측정이 그 상태에서 나온 숫자라 손대지 않는다).
/// * 그래서 안쪽을 보는 것은 구간 2뿐인데, 그쪽은 x **오름차순으로 앞에서부터**
///   예산의 절반을 태운다. x가 작은 쪽은 가장 오래 전에 채워져 남은 자리가 거의 없는
///   영역이라 탐침이 거기서 소진되고, 남는 것은 구간 3의 전역 성긴 스캔뿐이다.
///
/// 순차 배치에는 그게 맞다 — 새 조각이 갈 자리는 프론티어 근처에 있다. 그런데 이
/// 패스의 질문은 정확히 그 반대다. "이 조각을 **더 왼쪽·아래**로 옮길 수 있나" 는
/// **안쪽만 묻는 질문**이고, 찾는 자리는 방금 이 조각을 뺀 자리 주변, 즉 프론티어
/// **바로 안쪽**이다. 포켓 우선은 그 구간을 프론티어에서 왼쪽으로 **역방향**으로 훑고,
/// 구간을 끝까지 보고 후보를 찾았으면 바깥 구간을 아예 보지 않는다. 목적과 탐침 순서가
/// 그제서야 맞는다.
///
/// 실측(wsjk156, 탐침 4096, 유효 ordering 12개, 라운드 2). 기준선 71.75%:
///
/// | 탐침 순서 | 질의당 후보 | 밀도 | 시간 | 라운드별 이동 |
/// |---|---:|---:|---:|---|
/// | ④를 따름(예전) | 22.6개 | 72.02% | 29.5s | R1=257 R2=195 |
/// | **항상 포켓 우선(지금)** | **78.9개** | **72.11%** | **28.5s** | R1=229 R2=83 |
///
/// 후보가 3.5배로 늘었는데 **시간은 오히려 조금 줄었다** — 포켓 우선은 안쪽에서
/// 자리를 찾으면 바깥 구간을 아예 보지 않고 반환하기 때문이다. 이동 수는 452 -> 312로
/// 줄었는데 결과는 더 좋다(같은 이득을 더 적은 이동으로 얻는다). 라운드도 실제로
/// 수렴한다: 라운드 2/4/8 의 밀도가 전부 72.11% 로 같다.
///
/// 두 탐침 순서의 후보를 **합집합**으로 쓰면 더 좋아지지만(라운드 2에서 72.17%,
/// 라운드 4에서 72.34%) 오라클 호출이 조각마다 2회가 되어 이 패스의 비용이 2배가 된다.
/// `NfpStrategyConfig::estimated_cost_factor`가 그 배수를 모르므로(⑦은 `1 + 0.5 x 라운드`
/// 로만 잡혀 있다) 예산 배분이 어긋난다. 그 상수를 함께 고치기 전에는 넣지 않는다.
fn options_at_rotation(
    ctx: &mut StrategyContext<'_, '_>,
    item_id: usize,
    rotation: i64,
    others: &[PlacedPiece],
) -> Vec<AnchorOption> {
    let Some((bounds, _)) = ctx.placer.shape_bounds_area(item_id, rotation) else {
        return Vec::new();
    };

    let frontier = ctx.frontier(others);
    ctx.placer.set_probe_frontier(frontier);

    // ④ 설정과 무관하게 **항상** 포켓 우선으로 훑는다. 근거는 함수 문서 참조.
    ctx.placer.set_pocket_first(true);

    let options: Vec<AnchorOption> = ctx
        .placer
        .candidates_for(item_id, rotation, others)
        .into_iter()
        .map(|cand| {
            let (x, y) = (cand.translation.0 as f64, cand.translation.1 as f64);
            AnchorOption {
                item_id,
                rotation,
                x,
                y,
                world: bounds.translated(x, y),
                touches: cand.touch_count,
            }
        })
        .collect();

    // 탐침 순서를 기본값으로 되돌린다 — 같은 `placer`를 이어 쓰는 호출부가 있다.
    ctx.placer.set_pocket_first(false);
    options
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::DEFAULT_SWARM_CONFIG;
    use crate::nfp_lbf::placement::Aabb;
    use crate::nfp_lbf::{NfpLbfCache, PlacementScoreWeights, PreparedLibrary, SequentialPlacer};
    use crate::nfp_strategy::config::{DEFAULT_NFP_STRATEGY_CONFIG, NfpStrategyConfig};
    use jagua_rs::probs::spp::entities::SPInstance;
    use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;

    /// strip 높이(mm). 모든 픽스처가 공유한다.
    const STRIP_H: f32 = 20.0;
    /// IFP x 상한(mm). 픽스처가 쓰는 폭보다 넉넉하다.
    const STRIP_W_CAP: f32 = 200.0;

    fn import_instance(value: serde_json::Value) -> SPInstance {
        let external: ExtSPInstance = serde_json::from_value(value).expect(crate::dmsg!("픽스처 JSON 파싱"));
        let config = DEFAULT_SWARM_CONFIG;
        let importer = crate::util::io::new_importer(config.cde_config, config.poly_simpl_tolerance, config.min_item_separation, config.narrow_concavity_cutoff_ratio);
        jagua_rs::probs::spp::io::import_instance(&importer, &external).expect(crate::dmsg!("픽스처 import"))
    }

    /// 직사각형 아이템 목록으로 인스턴스 JSON을 만든다.
    fn rect_instance(rects: &[(f64, f64)], orientations: &[f64]) -> SPInstance {
        let items: Vec<serde_json::Value> = rects
            .iter()
            .enumerate()
            .map(|(id, (w, h))| {
                serde_json::json!({
                    "id": id,
                    "demand": 1,
                    "allowed_orientations": orientations,
                    "shape": {"type":"rectangle","data":{
                        "x_min":0.0,"y_min":0.0,"width":w,"height":h
                    }}
                })
            })
            .collect();
        import_instance(serde_json::json!({
            "name": "sliding-fixture",
            "strip_height": STRIP_H,
            "items": items,
        }))
    }

    /// 테스트가 쓰는 배치 준비 일체. 대여 수명 때문에 호출부에서 풀어서 써야 한다.
    struct Fixture {
        cache: NfpLbfCache,
        item_rotations: Vec<Vec<i64>>,
    }

    fn fixture(rects: &[(f64, f64)], rotations: &[i64]) -> Fixture {
        let orientations: Vec<f64> = rotations.iter().map(|d| (*d as f64).to_radians()).collect();
        let instance = rect_instance(rects, &orientations);
        let library =
            PreparedLibrary::build(&instance, rotations, 0.0).expect(crate::dmsg!("NFP 라이브러리 준비"));
        Fixture {
            cache: NfpLbfCache::new(library),
            item_rotations: vec![rotations.to_vec(); rects.len()],
        }
    }

    fn placer(fx: &mut Fixture) -> SequentialPlacer<'_> {
        SequentialPlacer::with_item_rotations(
            &mut fx.cache,
            STRIP_H,
            STRIP_W_CAP,
            PlacementScoreWeights::default(),
            fx.item_rotations.clone(),
        )
    }

    fn config() -> NfpStrategyConfig {
        NfpStrategyConfig {
            sliding_compaction: true,
            ..DEFAULT_NFP_STRATEGY_CONFIG
        }
    }

    /// 월드 좌하단 `(min_x, min_y)`에 놓이도록 앵커 평행이동을 역산한다.
    ///
    /// jagua-rs가 형상을 무게중심 기준으로 정규화하므로 앵커는 좌하단이 아니다.
    /// 테스트가 좌표를 직접 쓰면 정규화 방식이 바뀔 때 통째로 깨진다.
    fn piece_at(
        placer: &SequentialPlacer<'_>,
        item_id: usize,
        rotation: i64,
        min_x: f64,
        min_y: f64,
    ) -> PlacedPiece {
        let (bounds, _) = placer
            .shape_bounds_area(item_id, rotation)
            .expect(crate::dmsg!("픽스처 아이템의 bbox"));
        PlacedPiece {
            item_id,
            rotation,
            translation: (
                (min_x - bounds.min_x) as f32,
                (min_y - bounds.min_y) as f32,
            ),
        }
    }

    #[test]
    fn an_artificial_gap_is_squeezed_out_of_the_strip() {
        // 10x10 세 개. 세 번째만 x=40에 떨어뜨려 인위적인 틈을 만든다.
        let mut fx = fixture(&[(10.0, 10.0), (10.0, 10.0), (10.0, 10.0)], &[0]);
        let mut p = placer(&mut fx);
        let pieces = vec![
            piece_at(&p, 0, 0, 0.0, 0.0),
            piece_at(&p, 1, 0, 10.0, 0.0),
            piece_at(&p, 2, 0, 40.0, 0.0),
        ];
        let mut ctx = StrategyContext::new(&mut p, config(), Vec::new(), None);

        let out = compact(&mut ctx, &pieces, 3);

        assert_eq!(out.pieces.len(), 3, "조각이 사라지면 안 된다");
        assert!(
            out.after < out.before - MIN_GAIN_MM,
            "틈이 있으면 strip이 줄어야 한다: {} -> {}",
            out.before,
            out.after
        );
        assert!(out.moved > 0, "줄었다면 무언가는 옮겼어야 한다");
        assert!(out.gain() > 0.0);
    }

    #[test]
    fn a_perfectly_tight_layout_moves_nothing_and_stops_after_one_round() {
        // 높이 20 strip 을 10x10 네 장이 2x2 로 꽉 채운다 — 어느 조각도 더 갈 데가 없다.
        let mut fx = fixture(
            &[(10.0, 10.0), (10.0, 10.0), (10.0, 10.0), (10.0, 10.0)],
            &[0],
        );
        let mut p = placer(&mut fx);
        let pieces = vec![
            piece_at(&p, 0, 0, 0.0, 0.0),
            piece_at(&p, 1, 0, 0.0, 10.0),
            piece_at(&p, 2, 0, 10.0, 0.0),
            piece_at(&p, 3, 0, 10.0, 10.0),
        ];
        let mut ctx = StrategyContext::new(&mut p, config(), Vec::new(), None);

        let out = compact(&mut ctx, &pieces, 5);

        assert_eq!(out.moved, 0, "이미 최적이면 아무것도 옮기지 않는다");
        assert_eq!(out.rounds_run, 1, "이동이 없으면 즉시 수렴 종료해야 한다");
        assert_eq!(out.before, out.after);
    }

    #[test]
    fn rotations_survive_compaction_untouched() {
        // 0°/90° 가 모두 허용된 12x6 두 장. 90° 후보가 더 좌하단이더라도 쓰면 안 된다.
        let mut fx = fixture(&[(12.0, 6.0), (12.0, 6.0)], &[0, 90]);
        let mut p = placer(&mut fx);
        let pieces = vec![
            piece_at(&p, 0, 0, 0.0, 0.0),
            piece_at(&p, 1, 0, 30.0, 0.0),
        ];
        let rots = fx_rotations(&[0, 90], 2);
        let mut ctx = StrategyContext::new(&mut p, config(), rots, None);

        let out = compact(&mut ctx, &pieces, 3);

        assert_eq!(out.pieces.len(), pieces.len());
        // 이 단언이 없으면 오라클이 후보를 하나도 못 줘도 테스트가 통과한다 —
        // "회전이 안 바뀌었다" 가 "아무것도 안 했다" 로 위장되는 것을 막는다.
        assert!(
            out.moved > 0,
            "x=30 의 조각은 x=12 로 갈 수 있어야 한다 (테스트가 공회전했다)"
        );
        for (before, after) in pieces.iter().zip(out.pieces.iter()) {
            assert_eq!(
                before.rotation, after.rotation,
                "item {}의 회전이 바뀌었다",
                before.item_id
            );
            assert_eq!(before.item_id, after.item_id, "조각 순서가 뒤바뀌었다");
        }
    }

    #[test]
    fn an_expired_budget_leaves_the_layout_exactly_as_it_was() {
        // 예산이 이미 끝났으면 첫 라운드에 들어가기도 전에 나와야 한다.
        // 부분 결과라도 **입력과 같은 배치**여야 호출부가 그대로 쓸 수 있다.
        let mut fx = fixture(&[(10.0, 10.0), (10.0, 10.0)], &[0]);
        let mut p = placer(&mut fx);
        let pieces = vec![
            piece_at(&p, 0, 0, 0.0, 0.0),
            piece_at(&p, 1, 0, 40.0, 0.0),
        ];
        let past = jagua_rs::Instant::now();
        let mut ctx = StrategyContext::new(&mut p, config(), Vec::new(), Some(past));

        let out = compact(&mut ctx, &pieces, 8);

        assert_eq!(out.rounds_run, 0, "예산이 없으면 라운드를 돌지 않는다");
        assert_eq!(out.moved, 0);
        assert_eq!(out.before, out.after);
        assert_eq!(out.gain(), 0.0);
        for (before, after) in pieces.iter().zip(out.pieces.iter()) {
            assert_eq!(before.translation, after.translation, "배치가 바뀌었다");
        }
    }

    #[test]
    fn the_context_rotation_list_is_untouched_by_compaction() {
        // `ctx` 는 여러 전략이 이어 쓰는 공유 상태다. 압축이 회전 목록을 건드리면
        // **뒤에 도는 전략이 조용히 회전을 잃는다** — 증상이 sliding 에서 안 나오므로
        // 여기서 못 잡으면 못 잡는다. (예전 구현은 실제로 여기를 좁혔다 되돌렸다.)
        let allowed = vec![0, 90];
        let mut fx = fixture(&[(12.0, 6.0), (12.0, 6.0)], &allowed);
        let mut p = placer(&mut fx);
        let pieces = vec![
            piece_at(&p, 0, 0, 0.0, 0.0),
            piece_at(&p, 1, 0, 30.0, 0.0),
        ];
        let rots = fx_rotations(&allowed, 2);
        let mut ctx = StrategyContext::new(&mut p, config(), rots.clone(), None);

        let out = compact(&mut ctx, &pieces, 3);
        assert!(out.moved > 0, "좁히기 경로를 실제로 지나야 의미가 있다");

        assert_eq!(
            ctx.item_rotations, rots,
            "압축이 ctx 의 허용 회전 목록을 바꿔 놓았다"
        );
        for id in 0..pieces.len() {
            assert_eq!(ctx.rotations_for(id), allowed, "item {id}의 회전이 좁혀진 채 남았다");
        }
    }

    /// 두 bbox 가 실제로 겹치는가. 접촉(간극 0)은 겹침이 아니다.
    ///
    /// 픽스처가 전부 축 정렬 직사각형이라 bbox 겹침 = 실제 겹침이다. 허용 오차
    /// 0.01mm 는 f32 앵커 반올림만 흡수한다 — 진짜 겹침은 mm 단위로 난다.
    fn boxes_overlap(a: &Aabb, b: &Aabb) -> bool {
        const TOUCH_EPS_MM: f64 = 0.01;
        let dx = a.max_x.min(b.max_x) - a.min_x.max(b.min_x);
        let dy = a.max_y.min(b.max_y) - a.min_y.max(b.min_y);
        dx > TOUCH_EPS_MM && dy > TOUCH_EPS_MM
    }

    #[test]
    fn compaction_never_makes_two_pieces_overlap() {
        // 이 패스의 안전성 주장 전체가 "후보가 NFP 에서 왔으니 겹칠 수 없다" 다.
        // 그 주장을 결과에서 직접 확인하지 않으면 아무도 확인하지 않는다.
        let rects = [(8.0, 9.0), (6.0, 12.0), (11.0, 7.0), (5.0, 5.0), (9.0, 10.0)];
        let mut fx = fixture(&rects, &[0]);
        let mut p = placer(&mut fx);
        let spots = [
            (0.0, 0.0),
            (14.0, 0.0),
            (25.0, 3.0),
            (40.0, 8.0),
            (50.0, 0.0),
        ];
        let pieces: Vec<PlacedPiece> = spots
            .iter()
            .enumerate()
            .map(|(id, (x, y))| piece_at(&p, id, 0, *x, *y))
            .collect();
        let mut ctx = StrategyContext::new(&mut p, config(), Vec::new(), None);

        // 입력이 이미 겹쳐 있으면 이 테스트는 아무것도 증명하지 못한다. 먼저 확인한다.
        let start = ctx.placer.world_bounds(&pieces);
        for i in 0..start.len() {
            for j in (i + 1)..start.len() {
                if let (Some(a), Some(b)) = (start.get(i), start.get(j)) {
                    assert!(!boxes_overlap(a, b), "픽스처 입력이 이미 겹쳐 있다: {i} vs {j}");
                }
            }
        }

        let out = compact(&mut ctx, &pieces, 4);
        assert!(out.moved > 0, "아무것도 안 옮기면 검사가 공회전이다");

        let boxes = ctx.placer.world_bounds(&out.pieces);
        assert_eq!(boxes.len(), out.pieces.len(), "bbox 를 못 구한 조각이 있다");
        for i in 0..boxes.len() {
            for j in (i + 1)..boxes.len() {
                let (Some(a), Some(b)) = (boxes.get(i), boxes.get(j)) else {
                    continue;
                };
                assert!(
                    !boxes_overlap(a, b),
                    "압축 후 조각 {i}{a:?} 와 {j}{b:?} 가 겹친다"
                );
            }
        }
    }

    /// `item_rotations` 벡터를 만든다(테스트 가독성용).
    fn fx_rotations(rotations: &[i64], count: usize) -> Vec<Vec<i64>> {
        vec![rotations.to_vec(); count]
    }

    #[test]
    fn the_strip_never_gets_longer_than_it_started() {
        // 흩어진 다섯 장. 어느 조각이 어디로 가든 strip 이 늘어나면 실패다.
        let rects = [
            (8.0, 9.0),
            (6.0, 12.0),
            (11.0, 7.0),
            (5.0, 5.0),
            (9.0, 10.0),
        ];
        let mut fx = fixture(&rects, &[0]);
        let mut p = placer(&mut fx);
        let spots = [
            (0.0, 0.0),
            (14.0, 0.0),
            (25.0, 3.0),
            (40.0, 8.0),
            (50.0, 0.0),
        ];
        let pieces: Vec<PlacedPiece> = spots
            .iter()
            .enumerate()
            .map(|(id, (x, y))| piece_at(&p, id, 0, *x, *y))
            .collect();
        let mut ctx = StrategyContext::new(&mut p, config(), Vec::new(), None);

        let out = compact(&mut ctx, &pieces, 4);

        assert!(
            out.after <= out.before + LENGTH_EPS_MM,
            "strip 이 늘었다: {} -> {}",
            out.before,
            out.after
        );
        assert_eq!(out.pieces.len(), rects.len());
        assert!(out.rounds_run <= 4);
    }

    #[test]
    fn an_empty_layout_comes_back_empty_without_panicking() {
        let mut fx = fixture(&[(10.0, 10.0)], &[0]);
        let mut p = placer(&mut fx);
        let mut ctx = StrategyContext::new(&mut p, config(), Vec::new(), None);

        let out = compact(&mut ctx, &[], 3);

        assert!(out.pieces.is_empty());
        assert_eq!(out.before, 0.0);
        assert_eq!(out.after, 0.0);
        assert_eq!(out.moved, 0);
        assert_eq!(out.rounds_run, 0);
    }

    #[test]
    fn zero_rounds_returns_the_input_untouched() {
        let mut fx = fixture(&[(10.0, 10.0), (10.0, 10.0)], &[0]);
        let mut p = placer(&mut fx);
        let pieces = vec![
            piece_at(&p, 0, 0, 0.0, 0.0),
            piece_at(&p, 1, 0, 40.0, 0.0),
        ];
        let mut ctx = StrategyContext::new(&mut p, config(), Vec::new(), None);

        let out = compact(&mut ctx, &pieces, 0);

        assert_eq!(out.rounds_run, 0);
        assert_eq!(out.moved, 0);
        assert_eq!(out.before, out.after);
        for (before, after) in pieces.iter().zip(out.pieces.iter()) {
            assert_eq!(before.translation, after.translation);
        }
    }

    // ── 순수 함수 단위 테스트 (오라클 없이 도는 것들) ────────────────────

    #[test]
    fn pieces_are_processed_from_the_left_so_gaps_cascade_in_one_round() {
        let piece = |x: f32, y: f32| PlacedPiece {
            item_id: 0,
            rotation: 0,
            translation: (x, y),
        };
        // 입력은 오른쪽부터인데, 처리 순서는 왼쪽부터여야 한다.
        let pieces = [piece(30.0, 0.0), piece(10.0, 5.0), piece(10.0, 0.0)];
        assert_eq!(processing_order(&pieces), vec![2, 1, 0]);
    }

    #[test]
    fn a_move_shorter_than_the_minimum_gain_is_not_worth_taking() {
        let cur = (10.0, 5.0);
        assert!(!is_lower_left((10.0 - MIN_GAIN_MM * 0.5, 5.0), cur));
        assert!(!is_lower_left((10.0, 5.0 - MIN_GAIN_MM * 0.5), cur));
        assert!(!is_lower_left(cur, cur), "제자리는 개선이 아니다");
    }

    #[test]
    fn only_strictly_lower_left_candidates_are_accepted() {
        let cur = (10.0, 5.0);
        assert!(is_lower_left((9.0, 5.0), cur), "왼쪽이면 y는 상관없다");
        assert!(is_lower_left((9.0, 19.0), cur));
        assert!(is_lower_left((10.0, 1.0), cur), "같은 열이면 아래여야 한다");
        assert!(!is_lower_left((11.0, 1.0), cur), "오른쪽은 아무리 낮아도 안 된다");
        assert!(!is_lower_left((10.0, 9.0), cur));
        assert!(!is_lower_left((f64::NAN, 0.0), cur), "NaN 후보는 버린다");
    }

    // ────────────────────────────────────────────────────────────────────
    // 실 인스턴스 계측 하네스 (여성WSjk 156-패널). 자동 회귀가 아니라 **측정**이라
    // `#[ignore]` 다 — 한 번 도는 데 수십 초가 걸린다. 위 문서의 실측표를 다시 뽑는
    // 방법이기도 하다.
    // ────────────────────────────────────────────────────────────────────

    fn load_marker() -> SPInstance {
        let path =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input/wsjk156.json");
        let (ext, _) = crate::util::io::read_spp_input(&path).expect(crate::dmsg!("wsjk156.json 읽기"));
        let c = DEFAULT_SWARM_CONFIG;
        let imp = crate::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
        jagua_rs::probs::spp::io::import_instance(&imp, &ext).expect(crate::dmsg!("import"))
    }

    fn run_arm(inst: &SPInstance, strategy: NfpStrategyConfig) -> (f32, f32, u128, usize) {
        use crate::nfp_lbf::{NfpLbfConfig, build_best_initial_nest};
        use rand::SeedableRng;
        use rand::rngs::Xoshiro256PlusPlus;

        let factor = strategy.estimated_cost_factor().max(1.0);
        let cfg = NfpLbfConfig {
            n_orderings: (12.0 * factor).ceil() as usize,
            cache_budget: std::time::Duration::from_secs(10),
            total_budget: std::time::Duration::from_secs(900),
            concave_simplify_tolerance: 0.02,
            // 탐침 예산·시드는 이 모듈이 재는 모든 값의 **전제**다. 상수로 박아 두면
            // "다른 예산·시드에서도 그런가" 를 코드를 고쳐야만 물을 수 있다 —
            // `select::tests::run_arm`/`beam::tests::run_arm` 과 같은 이름의 노브를 둔다.
            max_candidate_probes: std::env::var("MEASURE_PROBES")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(4096),
            strategy,
            ..NfpLbfConfig::default()
        };
        let seed = std::env::var("MEASURE_SEED")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(0u64);
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(seed);
        let t = std::time::Instant::now();
        match build_best_initial_nest(inst, &cfg, &mut rng) {
            Some(n) => (
                n.solution.strip_width(),
                n.solution.density() * 100.0,
                t.elapsed().as_millis(),
                n.report.pieces_slid,
            ),
            None => (f32::NAN, f32::NAN, t.elapsed().as_millis(), 0),
        }
    }

    #[test]
    #[ignore = "실 인스턴스 필요 — 계측용"]
    fn measure_sliding_on_the_real_marker() {
        let inst = load_marker();
        let off = DEFAULT_NFP_STRATEGY_CONFIG;
        let rounds_list: Vec<usize> = std::env::var("NFP_SLIDE_ROUNDS")
            .unwrap_or_else(|_| "2".into())
            .split(',')
            .filter_map(|s| s.trim().parse().ok())
            .collect();

        for rounds in rounds_list {
            meas::set(true);
            meas::reset();
            let r = run_arm(
                &inst,
                NfpStrategyConfig {
                    sliding_compaction: true,
                    sliding_rounds: rounds,
                    ..off
                },
            );
            let rep = meas::report();
            meas::set(false);
            println!(
                "\n[⑦ 계측] 라운드 {rounds} -> 폭 {:.2} 밀도 {:.2}% {}ms (최선해 이동 {})\n{rep}",
                r.0, r.1, r.2, r.3
            );
        }
    }

    #[test]
    fn the_gain_of_an_outcome_never_reports_a_negative_number() {
        let grew = SlidingOutcome {
            before: 10.0,
            after: 12.0,
            ..SlidingOutcome::default()
        };
        assert_eq!(grew.gain(), 0.0);
        let shrank = SlidingOutcome {
            before: 10.0,
            after: 7.5,
            ..SlidingOutcome::default()
        };
        assert_eq!(shrank.gain(), 2.5);
    }
}
