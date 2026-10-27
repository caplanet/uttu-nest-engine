//! 누적 배치(accretion) — 순서를 정하지 않고 **가장 잘 맞물리는 조각**을 매번 고른다.
//!
//! ```text
//! 1. 가장 큰 조각 하나를 좌하단에 놓는다
//! 2. 반복:
//!      남은 조각 전부 x 그 조각의 feasible 후보 전부 를 훑어
//!      "이미 놓인 덩어리에 가장 잘 맞물리는 (조각, 자리)" 하나를 고른다
//! 3. 남은 조각이 없을 때까지
//! ```
//!
//! # 기존 방식과 무엇이 다른가
//!
//! | | 무엇을 고르나 | 기준 |
//! |---|---|---|
//! | LBF / NFP-LBF | 순서는 **고정**, 자리만 고른다 | 가장 왼쪽·아래 |
//! | ⑥ best-fit | 조각도 고른다 | **strip 을 덜 늘리는** 조각 |
//! | 여기(누적 배치) | 조각도 고른다 | **가장 잘 맞물리는** 조각 |
//!
//! ⑥과의 차이가 핵심이다. "strip 을 덜 늘린다" 는 기준은 조각 크기에 끌려간다 —
//! 작은 조각이 언제나 프론티어를 덜 미니까(그래서 ⑥은 면적 정규화와 큰 조각 우선
//! 티어가 필요했다). "잘 맞물린다" 는 기준은 **덩어리의 모양**을 본다.
//!
//! # 왜 이제야 가능한가
//!
//! "맞물림" 을 재려면 접촉을 알아야 하는데, 전략 계층의 접촉 지표는 bbox 근사라
//! 오목한 의류 패널에서 실제 맞물림을 자주 놓쳤다(실측: 후보의 76.2%가 접촉 0).
//!
//! 지금은 오라클의 feasible 판정이 **정확한 접촉 조각 수**를 함께 돌려준다
//! ([`crate::nfp_lbf::PlacementCandidate::touch_count`]). 그 판정이 obstacle 마다
//! 이미 계산하던 `Location::Boundary` 를 세기만 한 값이라 공짜다.
//!
//! # 비용과 병렬화
//!
//! 매 단계 `남은 조각 수 x 후보 수` 를 본다. 156조각이면 최악 156단계 x 156조각이라
//! 단일 스레드로는 분 단위다(⑥이 27초 걸린 이유).
//!
//! 그런데 **한 단계 안에서 조각끼리는 완전히 독립**이다. 16코어에 흩으면 그대로
//! 16배다. 캐시는 [`NfpLbfCache::fork`]로 스레드마다 사본을 준다 — 무거운 것은 전부
//! 이미 `Arc` 뒤에 있어 사본이 싸다.
//!
//! [`NfpLbfCache::fork`]: crate::nfp_lbf::NfpLbfCache::fork

use std::sync::Mutex;

use jagua_rs::Instant;
use rayon::prelude::*;

use crate::nfp_lbf::placement::Aabb;
use crate::nfp_lbf::{NfpLbfCache, PlacedPiece, PlacementScoreWeights, SequentialPlacer};
use crate::nfp_strategy::select::SelectionRule;
use crate::nfp_strategy::{AnchorOption, StrategyContext};

/// 큰 조각 우선 티어의 폭. 이 비율 안쪽 면적끼리만 맞물림으로 겨룬다.
///
/// 이 장치가 없으면 "잘 맞물린다" 는 기준도 결국 작은 조각으로 쏠린다 — 작은 조각은
/// 어디든 비집고 들어가 접촉을 많이 만든다. 그러면 큰 조각이 끝까지 밀려 마지막에
/// 몰리고, ⑥ best-fit 이 처음에 -12.8%p 로 무너졌던 것과 같은 붕괴가 난다.
///
/// ⑥에서 6개 인스턴스로 고른 `BEST_FIT_AREA_BAND = 0.20` 과 같은 값을 쓴다.
/// 같은 함정을 막는 같은 장치이므로 따로 튜닝할 근거가 없다.
pub const ACCRETE_AREA_BAND: f64 = 0.20;

/// 프론티어를 "늘리지 않았다" 고 볼 여유(mm).
///
/// f32 로 내려간 좌표를 f64 로 비교하므로 정확히 0 을 요구하면 거의 아무것도
/// 통과하지 못한다.
const GROWTH_FREE_EPS: f64 = 1.0e-4;

/// 한 조각을 한 자리에 놓는 후보의 **맞물림 품질**.
///
/// 비교는 사전식이고, 모든 항이 조각 크기로 정규화되어 있어 큰 조각과 작은 조각이
/// 같은 눈금에서 겨룬다.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Fit {
    /// 프론티어를 밀지 **않는가**. 이게 1순위다 — 폭을 늘리지 않는 자리가 있으면
    /// 그것들끼리만 겨루는 것이 이 기법의 정의다.
    free: bool,
    /// 정확히 맞닿는 조각 수. 근사가 아니다.
    touches: u16,
    /// 둘레로 정규화한 접촉 길이. 접촉 조각 수가 같을 때 가른다.
    contact_ratio: f64,
    /// 면적으로 정규화한 새 낭비. 작을수록 좋다.
    waste_ratio: f64,
    /// 면적으로 정규화한 프론티어 확장. `free` 가 거짓일 때만 의미가 있다.
    growth_ratio: f64,
    /// 좌하단 — 마지막 결정론 타이브레이크.
    x: f64,
    y: f64,
}

/// 비교 키를 **양자화**하는 격자.
///
/// # 왜 epsilon 이 아니라 양자화인가
///
/// `|a - b| <= eps` 로 동률을 판정하면 **추이성이 깨진다**. `a~b`, `b~c` 인데 `a<c` 인
/// 삼각형이 생기고, 그러면 "최댓값" 이 **훑는 순서에 따라 달라진다**.
///
/// 순차로 돌 때는 늘 같은 순서라 드러나지 않는다. 그런데 이 평가는 rayon 으로
/// 청크를 나눠 돌리고, **청크 경계는 스레드 수에 따라 달라진다**. 즉 같은 입력이
/// 머신마다·실행마다 다른 배치를 낼 수 있다.
///
/// 양자화는 값 하나만 보고 정수로 떨어뜨리므로 **정의상 추이적**이다. "가까우면
/// 같게 본다" 는 의도는 그대로 지키면서 전순서가 보장된다.
///
/// # 격자를 값의 스케일에 맞춘다
///
/// 처음에는 격자가 하나(`1.0e-6`)였고 주석은 "mm 단위 좌표에서 1 nm" 라고 적었다.
/// 그런데 같은 격자가 **무차원 비율**(`contact_ratio`, `waste_ratio`, `growth_ratio`)
/// 에도 걸렸다. 두 값군은 스케일이 3~4자리 다르다:
///
/// ```text
///                     범위        f32 기하에서 오는 잡음      1e-6 격자
/// 무차원 비율        0 .. ~1      ~1e-7 (상대정밀도)         잡음의 10배 — 적절
/// mm 좌표         0 .. ~7000     ~6e-4 (7000mm 의 1 ulp)    잡음의 1/600 — 너무 곱다
/// ```
///
/// 좌표 쪽이 잡음보다 600배 고우면 "가까우면 같게 본다" 가 성립하지 않는다. 실제로는
/// f32 재계산 잡음이 마지막 타이브레이크를 가르게 되고, 그 자리는 원래 **전역 유일한
/// 인덱스**(`slot`)가 정하도록 설계된 곳이다.
///
/// 결정론에는 어느 쪽이든 문제가 없다 — 같은 입력이면 f32 값도 같기 때문이다.
/// 문제는 **의도한 동작이 아니라는 것**이고, 그것이 이 코드베이스가 반복해 밟은
/// "정규화한 값에 mm 단위 허용오차를 쓴다" 는 함정의 형태다.
const QUANT_RATIO: f64 = 1.0e-6;

/// mm 좌표용 격자 — 1 µm.
///
/// f32 좌표의 1 ulp(7000mm 에서 ~0.6 µm)보다 크고, 재단 공차(0.1mm 급)보다는 훨씬
/// 작다. 즉 **잡음은 뭉개고 기하는 보존한다.** 여기서 동률이 나면 마지막 타이브레이크인
/// 전역 유일 인덱스가 결정한다.
const QUANT_MM: f64 = 1.0e-3;

/// 작을수록 좋은 항을 격자에 올린다. 비유한 값은 "가장 나쁨" 으로 눕힌다.
///
/// `NaN` 은 어느 비교에도 걸리지 않아 정렬을 통째로 망가뜨린다. `i64` 로 떨어뜨리면
/// 그 위험이 사라진다.
#[inline]
fn q_asc(v: f64, quant: f64) -> i64 {
    if !v.is_finite() {
        return i64::MAX;
    }
    (v / quant).round() as i64
}

/// **클수록 좋은** 항을 격자에 올린다.
///
/// # 왜 별도 함수인가
///
/// 예전에는 `-q(v)` 로 부호만 뒤집어 썼다. 그런데 `q` 의 비유한 감시자는 `i64::MAX`
/// (= 가장 나쁨)이라, 부호를 뒤집으면 `-i64::MAX` = **가장 좋음**이 된다.
/// 즉 `contact_ratio` 가 `NaN` 인 후보가 "접촉이 가장 많은 후보" 로 정렬돼 이겼다.
/// 계약이 정확히 반대로 뒤집혀 있었다.
///
/// 부호를 뒤집는 것은 **유한한 값에만** 하고, 비유한 값은 뒤집기 전에 걸러 낸다.
#[inline]
fn q_desc(v: f64, quant: f64) -> i64 {
    if !v.is_finite() {
        return i64::MAX;
    }
    -((v / quant).round() as i64)
}

impl Fit {
    /// 비교 키. **정수 튜플**이라 사전식 비교가 곧 전순서다.
    ///
    /// 부호를 뒤집어 "작을수록 좋음" 으로 통일한다 — 그래야 튜플 하나로 비교된다.
    fn key(&self) -> (u8, i64, i64, i64, i64, i64, i64) {
        (
            // 1순위 — 폭을 늘리지 않는 쪽 (0 이 좋다).
            u8::from(!self.free),
            // 2순위 — 폭을 늘려야 한다면 면적당 확장이 작은 쪽.
            //         `free` 면 이 항이 0 이라 순위에 영향이 없다.
            if self.free {
                0
            } else {
                q_asc(self.growth_ratio, QUANT_RATIO)
            },
            // 3순위 — 정확한 접촉 조각 수 (많을수록 좋으므로 음수로).
            //         `u16` 이라 비유한 값이 없다.
            -(self.touches as i64),
            // 4순위 — 접촉 길이(정규화). 많을수록 좋다.
            q_desc(self.contact_ratio, QUANT_RATIO),
            // 5순위 — 새 낭비. 적을수록 좋다.
            q_asc(self.waste_ratio, QUANT_RATIO),
            // 6·7순위 — 좌하단. **mm 격자**를 쓴다 (§`QUANT_MM`).
            q_asc(self.x, QUANT_MM),
            q_asc(self.y, QUANT_MM),
        )
    }
}

impl Best {
    /// `a` 가 `b` 보다 좋은가.
    ///
    /// 완전 동률이면 **`slot` 이 작은 쪽**이 이긴다. `slot` 은 남은 조각 목록의
    /// 인덱스라 전역에서 유일하므로, 이 비교는 **강전순서**다 — 어떤 순서로 훑어도
    /// 최댓값이 하나로 정해진다. 그것이 병렬 축약이 결정론적인 이유다.
    fn better(a: &Best, b: &Best) -> bool {
        (a.fit.key(), a.slot) < (b.fit.key(), b.slot)
    }
}

/// 누적 배치 결과.
#[derive(Clone, Debug, Default)]
pub struct AccretionOutcome {
    pub pieces: Vec<PlacedPiece>,
    /// 놓지 못한 `remaining` 원소의 원래 인덱스.
    pub unplaced: Vec<usize>,
    pub strip_length: f64,
}

/// 한 조각 종류를 평가한 결과.
struct Best {
    slot: usize,
    option: AnchorOption,
    fit: Fit,
}

/// 누적 배치로 전부 놓는다.
///
/// `remaining` 은 수요만큼 반복된 `item_id` 목록이다. 순서는 **쓰지 않는다** —
/// 다중집합으로만 본다.
///
/// `cache` 는 pre-warm 이 끝난 상태여야 한다. 스레드마다 [`NfpLbfCache::fork`] 사본을
/// 만들어 쓰므로, warm 하지 않으면 스레드마다 같은 NFP 를 따로 계산하게 된다.
///
/// [`NfpLbfCache::fork`]: crate::nfp_lbf::NfpLbfCache::fork
pub fn accrete(
    cache: &NfpLbfCache,
    remaining: &[usize],
    strip_height: f32,
    strip_width_cap: f32,
    weights: PlacementScoreWeights,
    item_rotations: &[Vec<i64>],
    probe_budget: usize,
    rule: SelectionRule,
    deadline: Option<Instant>,
) -> AccretionOutcome {
    let mut left: Vec<usize> = remaining.to_vec();
    let mut origin: Vec<usize> = (0..remaining.len()).collect();
    // 놓을 수 없어 후보에서 뺀 조각의 원래 인덱스. `origin` 은 `left` 와 자리를 맞춰야
    // 하므로 거기서 빼면 사라진다 — 미배치 보고에는 여기에 모아 둔 것도 합친다.
    let mut dropped: Vec<usize> = Vec::new();
    let mut placed: Vec<PlacedPiece> = Vec::with_capacity(remaining.len());

    // 캐시 사본은 **스레드마다 하나**, 호출당 한 번만 만든다.
    //
    // 예전에는 `map_init(|| cache.fork())` 로 만들었는데, rayon 은 init 을 작업
    // 분할(split)마다 다시 부르므로 청크 1 에서는 사실상 **조각마다** 사본이 생겼다.
    // 사본은 `Arc` 복제라 싸지만 단계 x 조각 수만큼 쌓이면 무시할 수 없다.
    // `current_thread_index` 로 고르면 한 스레드가 늘 같은 사본을 쓰고, Mutex 는
    // 풀 밖(인덱스 None)에서 불렸을 때의 안전망이다 — 경합은 정상 경로에 없다.
    let n_threads = rayon::current_num_threads().max(1);
    let forks: Vec<Mutex<NfpLbfCache>> = (0..n_threads).map(|_| Mutex::new(cache.fork())).collect();

    // 청크 크기를 **스레드 수와 무관하게** 고정한다.
    //
    // `tier.len() / 스레드수` 로 쪼개면 같은 입력이 머신마다 다르게 분할된다.
    // 비교가 전순서라 답 자체는 같지만, 분할이 입력의 함수여야 재현이 명확하다
    // (테스트가 스레드 1/3/16 에서 같은 결과를 요구한다).
    //
    // **1이다.** 조각 하나가 곧 한 작업 단위다.
    //
    // 큰 조각 우선 티어 때문에 한 단계에서 겨루는 조각이 보통 10여 개다. 청크를 4로
    // 묶으면 3작업밖에 안 나와 16코어 중 13개가 논다 — 실측으로 배치가
    // 1,607ms -> 2,391ms 로 **느려졌다**. 1이면 조각마다 한 작업이라 병렬도가 최대다.
    //
    // 대가는 조각마다 `SequentialPlacer` 를 새로 만드는 비용이다(SoA 버퍼 재할당).
    // 캐시 사본은 스레드마다 하나를 재사용하므로 남는 비용은 그 버퍼뿐이고,
    // 실측에서 병렬도 손실이 그보다 훨씬 컸다.
    const CHUNK: usize = 1;

    while !left.is_empty() {
        // 예산 소진이든 Init 취소 요청이든 즉시 멈춘다. 취소를 안 보면 호출부가
        // 취소를 눌러도 이 루프가 남은 예산을 다 태운 뒤에야 돌아온다.
        if deadline.is_some_and(|d| Instant::now() >= d)
            || crate::config::runtime::init_cancel_requested()
        {
            break;
        }

        // ── 큰 조각 우선 티어 ──
        //
        // 남은 조각 중 가장 큰 것의 면적 x (1 - BAND) 이상만 겨룬다.
        // 근거는 `ACCRETE_AREA_BAND` 문서 참조.
        let areas: Vec<f64> = left
            .iter()
            .map(|&id| item_area(cache, id, item_rotations))
            .collect();
        let max_area = areas.iter().copied().fold(0.0_f64, f64::max);
        let floor = max_area * (1.0 - ACCRETE_AREA_BAND);
        let tier: Vec<usize> = (0..left.len())
            .filter(|&i| areas.get(i).copied().unwrap_or(0.0) >= floor)
            .collect();
        let tier = if tier.is_empty() {
            (0..left.len()).collect::<Vec<_>>()
        } else {
            tier
        };

        let chunk = CHUNK;

        // ── 티어 안의 조각들을 병렬로 평가한다 ──
        //
        // 조각끼리 서로를 보지 않으므로 완전히 독립이다. 청크마다 placer 를 하나만
        // 만들어 SoA 버퍼를 그 안에서 재사용한다.
        let found: Vec<Best> = tier
            .par_chunks(chunk)
            .map(|slots| {
                // 이 스레드의 캐시 사본. 인덱스가 풀 크기를 넘는 일은 없지만, 다른
                // 풀에서 불려도 Mutex 가 정확성을 지키므로 나머지 연산으로 접는다.
                let idx = rayon::current_thread_index().unwrap_or(0) % forks.len();
                let mut local = match forks[idx].lock() {
                    Ok(guard) => guard,
                    Err(poisoned) => poisoned.into_inner(),
                };
                let mut placer = SequentialPlacer::with_item_rotations(
                    &mut local,
                    strip_height,
                    strip_width_cap,
                    weights,
                    item_rotations.to_vec(),
                )
                .with_probe_budget(probe_budget);
                // 회전표는 placer 에 이미 들어 있다. 빈 표를 주면 `rotations_for` 가
                // 오라클(placer)의 표로 폴백하므로 같은 표를 두 번 복사하지 않는다.
                let mut ctx = StrategyContext::new(
                    &mut placer,
                    crate::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG,
                    Vec::new(),
                    deadline,
                );

                let mut best: Option<Best> = None;
                let boxes = ctx.placer.world_bounds(&placed);
                let frontier = boxes.iter().map(|b| b.max_x).fold(0.0_f64, f64::max);

                for &slot in slots {
                    let Some(&item_id) = left.get(slot) else {
                        continue;
                    };
                    let Some(cand) =
                        best_for_item(&mut ctx, item_id, &placed, &boxes, frontier, rule)
                    else {
                        continue;
                    };
                    let cand = Best {
                        slot,
                        option: cand.0,
                        fit: cand.1,
                    };
                    if best.as_ref().is_none_or(|b| Best::better(&cand, b)) {
                        best = Some(cand);
                    }
                }
                best
            })
            .flatten()
            .collect();

        // ── 청크별 최선 중 전역 최선 ──
        let Some(winner) = found.into_iter().reduce(|a, b| {
            if Best::better(&b, &a) { b } else { a }
        }) else {
            // 티어의 어느 조각도 놓을 수 없다. 티어를 넓혀 한 번 더 시도하고,
            // 그래도 안 되면 남은 전부를 미배치로 보고한다.
            if tier.len() == left.len() {
                break;
            }
            // 티어의 조각은 **전부** 놓을 수 없다고 방금 확인했다. 하나씩 빼면 같은
            // 티어(의 나머지)를 단계마다 다시 평가해 같은 결론만 반복하므로, 티어를
            // 통째로 미배치로 뺀다. `tier` 는 오름차순 인덱스라 뒤에서부터 지워야
            // 앞의 인덱스가 밀리지 않는다.
            for &slot in tier.iter().rev() {
                if slot < origin.len() {
                    unplaced_push(&mut origin, slot, &mut left, &mut dropped);
                }
            }
            continue;
        };

        placed.push(winner.option.to_piece());
        if winner.slot < left.len() {
            left.remove(winner.slot);
            origin.remove(winner.slot);
        }
    }

    let strip_length = placed
        .iter()
        .filter_map(|p| {
            let (b, _) = shape_of(cache, p.item_id, p.rotation)?;
            Some(b.max_x + p.translation.0 as f64)
        })
        .fold(0.0_f64, f64::max);

    // 미배치 = 후보에서 뺀 것 + 예산 소진 등으로 끝까지 남은 것. 둘을 합쳐야
    // `pieces + unplaced == remaining` 이 맞는다.
    let mut unplaced = dropped;
    unplaced.extend(origin);
    unplaced.sort_unstable();

    AccretionOutcome {
        pieces: placed,
        unplaced,
        strip_length,
    }
}

/// 한 조각의 최선 자리와 그 맞물림 품질.
fn best_for_item(
    ctx: &mut StrategyContext<'_, '_>,
    item_id: usize,
    placed: &[PlacedPiece],
    boxes: &[Aabb],
    frontier: f64,
    _rule: SelectionRule,
) -> Option<(AnchorOption, Fit)> {
    // **오라클에서 직접** 후보를 받는다.
    //
    // `ctx.options_for` 를 쓰지 않는 이유: 그쪽은 `AnchorOption` 만 돌려주는데,
    // 이 기법에 꼭 필요한 **정확한 접촉 조각 수**(`PlacementCandidate::touch_count`)가
    // 거기서 떨어져 나간다. 접촉을 bbox 로 다시 추정하면 이 기법의 전제가 무너진다
    // (오목한 의류 패널에서 후보의 76.2%가 접촉 0 으로 잡힌다).
    ctx.placer.set_probe_frontier(frontier);
    let mut options: Vec<(AnchorOption, u16, f64)> = Vec::new();
    for rotation in ctx.rotations_for(item_id) {
        let Some((bounds, shape_area)) = ctx.placer.shape_bounds_area(item_id, rotation) else {
            continue;
        };
        for cand in ctx.placer.candidates_for(item_id, rotation, placed) {
            let (x, y) = (cand.translation.0 as f64, cand.translation.1 as f64);
            options.push((
                AnchorOption {
                    item_id,
                    rotation,
                    x,
                    y,
                    world: bounds.translated(x, y),
                    touches: cand.touch_count,
                },
                cand.touch_count,
                shape_area,
            ));
        }
    }
    if options.is_empty() {
        return None;
    }
    let mut best: Option<(AnchorOption, Fit)> = None;
    for (option, touch_count, shape_area) in options {
        let m = ctx.metrics(&option, boxes, frontier);
        let w = (option.world.max_x - option.world.min_x).max(0.0);
        let h = (option.world.max_y - option.world.min_y).max(0.0);
        // **bbox 면적**으로 정규화한다. 정확한 도형 면적이 아니다 — 의도적이다.
        //
        // # "정확한 값이 있으면 그걸 써라" 가 여기서는 틀린다
        //
        // 오라클의 `shape_bounds_area` 가 정확한 폴리곤 면적을 함께 돌려주므로, 그것으로
        // 바꿔 재 봤다. **전부 나빠졌다** (wsjk156, 탐침 8192, 2026-08-31):
        //
        // ```text
        //                    bbox 면적    정확 면적
        //   ⑧ 누적배치        72.93%      72.04%   -0.89%p
        //   ⑧+⑦             73.09%      72.04%   -1.05%p
        //   ⑧+⑦+⑨           73.72%      72.59%   -1.13%p
        // ```
        //
        // 이유는 **분자**에 있다. `m.waste` 는 이웃 bbox 사이의 간극을 직사각형으로 잡은
        // bbox 근사이고, `growth` 도 bbox 로 정한 프론티어의 확장이다. 그 분자를 정확한
        // 면적으로 나누면 근사 수준이 어긋난 비율이 나온다 — 의류 패널은 자기 bbox 의
        // 62.6%만 채우므로 오목할수록 분모가 작아져 낭비·확장이 실제보다 크게 보이고,
        // 그래서 **맞물릴 수 있는 오목한 조각이 뒤로 밀린다.**
        //
        // 함정 3("bbox 근사를 실제 기하로 착각")의 교훈은 "언제나 정확한 값을 써라" 가
        // 아니다. **분자와 분모의 근사 수준을 맞춰라** 이다. 정확한 값을 쓰려면
        // `waste` 와 `growth` 도 함께 정확해져야 하고, 그건 별개의 작업이다.
        let area = (w * h).max(1.0e-9);
        let perimeter = (2.0 * (w + h)).max(1.0e-9);
        // 정확 면적은 지금 쓰지 않는다. 위 표를 뒤집는 측정이 나오면 그때 쓴다.
        let _ = shape_area;
        let growth = (m.strip_length - frontier).max(0.0);
        let fit = Fit {
            free: growth <= GROWTH_FREE_EPS,
            touches: touch_count,
            contact_ratio: m.contact / perimeter,
            waste_ratio: m.waste / area,
            growth_ratio: growth * ctx.placer.strip_height() as f64 / area,
            x: m.x,
            y: m.y,
        };
        if best.as_ref().is_none_or(|(_, b)| fit.key() < b.key()) {
            best = Some((option, fit));
        }
    }
    best
}

/// `(item_id, rotation)` 의 앵커 기준 bbox 와 면적.
fn shape_of(cache: &NfpLbfCache, item_id: usize, rotation: i64) -> Option<(Aabb, f64)> {
    let lib = cache.library();
    let hash = lib.hash_of_item(item_id)?;
    let shape = lib.shape(hash, rotation)?;
    let b = shape.bounds();
    Some((
        Aabb {
            min_x: b.min.x,
            min_y: b.min.y,
            max_x: b.max.x,
            max_y: b.max.y,
        },
        shape.area_mm2(),
    ))
}

/// 조각의 실제 면적. 티어 판정에 쓴다. 읽지 못하면 `0.0` — 티어 맨 뒤로 밀린다.
fn item_area(cache: &NfpLbfCache, item_id: usize, item_rotations: &[Vec<i64>]) -> f64 {
    let rot = item_rotations
        .get(item_id)
        .and_then(|r| r.first().copied())
        .unwrap_or(0);
    match shape_of(cache, item_id, rot) {
        Some((_, area)) if area.is_finite() && area > 0.0 => area,
        _ => 0.0,
    }
}

/// 놓을 수 없는 조각을 후보 집합에서 빼고 원래 인덱스를 `dropped` 에 미배치로 남긴다.
fn unplaced_push(
    origin: &mut Vec<usize>,
    slot: usize,
    left: &mut Vec<usize>,
    dropped: &mut Vec<usize>,
) {
    if slot < left.len() {
        left.remove(slot);
    }
    // `origin` 은 `left` 와 자리를 맞춰야 하므로 여기서 빼되, 원래 인덱스는 잃지 않는다.
    if slot < origin.len() {
        dropped.push(origin.remove(slot));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 비유한 값은 **어느 방향의 항에서도** "가장 나쁨" 이어야 한다.
    ///
    /// 예전에는 클수록 좋은 항을 `-q(v)` 로 썼는데, `q` 의 비유한 감시자가
    /// `i64::MAX`(가장 나쁨) 이라 부호를 뒤집으면 **가장 좋음**이 됐다.
    /// `contact_ratio` 가 `NaN` 인 후보가 "접촉이 가장 많은 후보" 로 이긴 것이다.
    #[test]
    fn a_non_finite_term_never_wins_in_either_direction() {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(
                q_asc(bad, QUANT_RATIO),
                i64::MAX,
                "작을수록 좋은 항에서 비유한 값이 최악이 아니다"
            );
            assert_eq!(
                q_desc(bad, QUANT_RATIO),
                i64::MAX,
                "클수록 좋은 항에서 비유한 값이 최악이 아니다 — 부호를 뒤집으면서 계약이 깨졌다"
            );
            // 어떤 유한한 값보다도 나빠야 한다(정렬은 오름차순 = 작을수록 좋음).
            assert!(q_desc(bad, QUANT_RATIO) > q_desc(0.0, QUANT_RATIO));
            assert!(q_desc(bad, QUANT_RATIO) > q_desc(1.0e9, QUANT_RATIO));
        }
    }

    /// 격자는 **값의 스케일에 맞아야** 한다.
    ///
    /// 무차원 비율(0..1)과 mm 좌표(0..7000)는 잡음 스케일이 3~4자리 다르다.
    /// 하나의 격자를 둘 다에 쓰면 한쪽에서는 반드시 어긋난다 — 이 코드베이스가
    /// 반복해 밟은 함정이다.
    #[test]
    fn the_grid_matches_the_scale_of_what_it_quantizes() {
        // mm 격자는 f32 좌표의 1 ulp(7000mm 에서 약 6e-4mm)보다 커야 잡음을 뭉갠다.
        let ulp_at_7000 = (7000.0f32.next_up() - 7000.0f32) as f64;
        assert!(
            QUANT_MM > ulp_at_7000,
            "mm 격자 {QUANT_MM} 가 f32 잡음 {ulp_at_7000} 보다 곱다 — 잡음이 타이브레이크를 가른다"
        );
        // 그러면서 재단 공차(0.1mm 급)보다는 훨씬 작아야 기하를 보존한다.
        assert!(QUANT_MM < 0.01, "mm 격자가 재단 공차에 가까우면 실제 기하가 뭉개진다");
        // 비율 격자는 0..1 구간을 충분히 가른다.
        assert!(QUANT_RATIO <= 1.0e-6);
        // 두 격자는 서로 달라야 한다 — 같으면 한쪽 스케일에는 반드시 안 맞는다.
        assert_ne!(QUANT_MM, QUANT_RATIO);
    }

    /// 테스트용 — 키 비교를 그대로 쓴다. `Best::better` 의 `slot` 타이브레이크만 뺀 것이다.
    fn fit_better(a: &Fit, b: &Fit) -> bool {
        a.key() < b.key()
    }

    fn fit(free: bool, touches: u16, contact: f64, waste: f64, growth: f64) -> Fit {
        Fit {
            free,
            touches,
            contact_ratio: contact,
            waste_ratio: waste,
            growth_ratio: growth,
            x: 0.0,
            y: 0.0,
        }
    }

    #[test]
    fn a_placement_that_does_not_widen_the_strip_always_wins() {
        // 폭을 안 늘리는 자리는 접촉이 적어도 이긴다 — 그게 이 기법의 1순위다.
        let free_weak = fit(true, 1, 0.0, 1.0, 0.0);
        let grow_strong = fit(false, 9, 1.0, 0.0, 0.1);
        assert!(fit_better(&free_weak, &grow_strong));
        assert!(!fit_better(&grow_strong, &free_weak));
    }

    #[test]
    fn among_free_placements_the_one_touching_more_pieces_wins() {
        let a = fit(true, 3, 0.1, 0.5, 0.0);
        let b = fit(true, 1, 0.9, 0.0, 0.0);
        assert!(
            fit_better(&a, &b),
            "정확한 접촉 조각 수가 접촉 길이 추정보다 우선이어야 한다"
        );
    }

    #[test]
    fn equal_touch_counts_fall_through_to_the_contact_length_then_waste() {
        let more_contact = fit(true, 2, 0.9, 0.5, 0.0);
        let less_contact = fit(true, 2, 0.1, 0.0, 0.0);
        assert!(fit_better(&more_contact, &less_contact));

        let less_waste = fit(true, 2, 0.5, 0.1, 0.0);
        let more_waste = fit(true, 2, 0.5, 0.9, 0.0);
        assert!(fit_better(&less_waste, &more_waste));
    }

    #[test]
    fn when_every_option_widens_the_strip_the_cheapest_growth_wins() {
        // 둘 다 폭을 늘려야 한다면 **면적당** 확장이 작은 쪽이다.
        // 정규화가 없으면 작은 조각이 언제나 이겨 큰 조각이 끝까지 밀린다.
        let cheap = fit(false, 0, 0.0, 0.0, 0.2);
        let dear = fit(false, 9, 1.0, 0.0, 0.9);
        assert!(
            fit_better(&cheap, &dear),
            "폭을 늘려야 하는 상황에서는 확장 비용이 접촉보다 우선이다"
        );
    }

    #[test]
    fn the_comparison_is_deterministic_and_irreflexive() {
        let a = fit(true, 2, 0.5, 0.5, 0.0);
        assert!(!fit_better(&a, &a), "자기 자신보다 좋을 수는 없다");

        let mut b = a;
        b.x = 1.0;
        assert!(fit_better(&a, &b), "완전 동률이면 좌하단이 이긴다");
        assert!(!fit_better(&b, &a));
    }

    /// **전순서인가.** 병렬 축약이 결정론적이려면 이것이 성립해야 한다.
    ///
    /// epsilon 비교(`|a-b| <= eps`)를 쓰면 여기서 깨진다 — `a~b`, `b~c` 인데 `a<c` 인
    /// 삼각형이 생기고, 그러면 청크를 어떻게 나누느냐에 따라 최댓값이 달라진다.
    /// 양자화한 정수 키는 정의상 이 성질을 만족한다.
    #[test]
    fn the_comparison_is_a_total_order_so_any_evaluation_sequence_agrees() {
        // 격자(1e-6) 근처에서 서로 조금씩 다른 값들을 만든다 — epsilon 비교였다면
        // 여기서 추이성이 깨질 조합이다.
        let mut samples = Vec::new();
        for i in 0..40 {
            let d = i as f64 * 4.0e-7; // 격자보다 작은 간격
            samples.push(fit(true, 2, 0.5 + d, 0.5 - d, 0.0));
        }

        // 삼분법: 정확히 하나만 참이어야 한다.
        for a in samples.iter() {
            for b in samples.iter() {
                let ab = fit_better(a, b);
                let ba = fit_better(b, a);
                let eq = a.key() == b.key();
                assert_eq!(
                    [ab, ba, eq].iter().filter(|x| **x).count(),
                    1,
                    "삼분법 위반: a<b={ab}, b<a={ba}, a==b={eq}"
                );
            }
        }

        // 추이성.
        for a in samples.iter() {
            for b in samples.iter() {
                for c in samples.iter() {
                    if fit_better(a, b) && fit_better(b, c) {
                        assert!(fit_better(a, c), "추이성 위반 — 병렬 축약이 순서에 의존하게 된다");
                    }
                }
            }
        }
    }

    /// 비유한 값이 순서를 망가뜨리지 않는가.
    ///
    /// `NaN` 은 어느 비교에도 걸리지 않아 정렬을 통째로 무너뜨린다. 키가 정수라
    /// 그 위험이 없어야 한다.
    #[test]
    fn a_broken_metric_sinks_to_the_bottom_instead_of_poisoning_the_order() {
        let sane = fit(true, 2, 0.5, 0.5, 0.0);
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let broken = fit(true, 2, bad, 0.5, 0.0);
            // 어느 쪽이 이기든 **정확히 한 방향으로만** 정해져야 한다.
            let ab = fit_better(&sane, &broken);
            let ba = fit_better(&broken, &sane);
            assert!(!(ab && ba), "양방향으로 더 좋을 수는 없다 ({bad})");
            // 그리고 자기 자신과는 동률이다 (NaN != NaN 이 새어 나오면 안 된다).
            assert!(!fit_better(&broken, &broken), "{bad} 가 자기 자신보다 좋다");
        }
    }

    #[test]
    fn the_area_band_matches_the_one_that_best_fit_needed() {
        // 같은 함정(작은 조각 쏠림)을 막는 같은 장치다. 값이 갈라지면 한쪽만
        // 튜닝되어 조용히 어긋난다.
        assert_eq!(ACCRETE_AREA_BAND, 0.20);
    }
}
