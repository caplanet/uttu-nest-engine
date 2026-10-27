//! §1. 탐색 정체 감지 (Stagnation Detection)
//!
//! ```text
//! 단순히 반복 횟수만 보는 것보다 다음 지표를 같이 사용한다.
//!
//! ① Best Efficiency 개선량
//! ② 후보 배치 간 다양성
//! ③ 이동 성공률
//! ④ 오른쪽 경계 패턴의 고정 시간
//! ⑤ 동일한 패턴 쌍의 반복 충돌 또는 이동 실패 횟수
//! ```
//!
//! 예시 조건:
//!
//! ```text
//! 최근 max(100, 3 × 패턴 수) 회 동안
//! 효율 개선량 < 0.01%p
//!
//! AND
//!
//! 후보 배치 간 위치·순서 다양성이 기준 이하
//! ```
//!
//! 두 조건이 **동시에** 발생했을 때 탈출을 실행하면, 단순히 계산이 오래 걸리는 구간을
//! 국소 최적해로 오판하는 문제를 줄일 수 있다.
//!
//! 따라서 [`StagnationDetector::is_stagnant`]는 ①과 ②의 **AND**로만 판정한다.
//! ③④⑤는 [`StagnationSignals`]로 노출되며 [`StagnationDetector::stagnation_intensity`]
//! (= 탈출 강도 보간 계수)에만 반영된다.

use crate::escape::{
    STAGNATION_DIVERSITY_THRESHOLD, STAGNATION_EFFICIENCY_EPS, STAGNATION_WINDOW_FLOOR,
    STAGNATION_WINDOW_ITEM_FACTOR,
};
use crate::optimizer::separator::Separator;
use jagua_rs::probs::spp::entities::SPSolution;
use std::cmp::Ordering;
use std::collections::hash_map::DefaultHasher;
use std::collections::{HashMap, VecDeque};
use std::hash::{Hash, Hasher};

// ────────────────────────────────────────────────────────────────────────────
// 내부 튜닝 상수 (문서가 수치를 지정하지 않은 보조 항목들)
// ────────────────────────────────────────────────────────────────────────────

/// 우측 경계 시그니처를 계산할 때 사용하는 밴드 폭(strip 폭 대비 비율).
/// strip 우측 끝에서 이 비율만큼 안쪽까지 걸친 아이템이 "오른쪽 경계 패턴"이다.
const RIGHT_BOUNDARY_BAND_RATIO: f32 = 0.02;

/// 같은 쌍이 몇 번 실패해야 "반복 실패"로 셀지의 하한. 문서의 ⑤ 지표.
const PAIR_FAILURE_REPEAT_MIN: u32 = 2;

/// 다양성 결합 가중치 — 위치(정규화 거리) 항.
const W_DIVERSITY_POSITION: f32 = 0.6;

/// 다양성 결합 가중치 — 좌우 정렬 순서(Spearman 불일치) 항.
const W_DIVERSITY_ORDER: f32 = 0.4;

/// `stagnation_intensity()` 결합 가중치 (합 = 1.00).
const W_INTENSITY_PERSISTENCE: f32 = 0.45;
/// [`W_INTENSITY_PERSISTENCE`] 참조 — ④ 우측 경계 고정 시간.
const W_INTENSITY_FREEZE: f32 = 0.20;
/// [`W_INTENSITY_PERSISTENCE`] 참조 — ③ 이동 성공률(의 여집합).
const W_INTENSITY_MOVE_FAILURE: f32 = 0.20;
/// [`W_INTENSITY_PERSISTENCE`] 참조 — ⑤ 반복 실패 쌍.
const W_INTENSITY_PAIR: f32 = 0.15;

/// ⑤ 항이 포화되는 반복 실패 쌍의 수 = `max(4, window × 0.05)`.
const PAIR_SATURATION_WINDOW_RATIO: f32 = 0.05;
/// [`PAIR_SATURATION_WINDOW_RATIO`]의 하한.
const PAIR_SATURATION_FLOOR: f32 = 4.0;

// ────────────────────────────────────────────────────────────────────────────
// 설정
// ────────────────────────────────────────────────────────────────────────────

/// 정체 감지 파라미터.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StagnationConfig {
    /// 관측 창의 하한. `max(100, 3 × 패턴 수)`의 `100`.
    pub window_floor: usize,
    /// 관측 창의 패턴 수 배수. `max(100, 3 × 패턴 수)`의 `3`.
    pub window_item_factor: usize,
    /// 효율 개선량 임계값. `0.01%p = 0.0001`.
    pub efficiency_eps: f32,
    /// 후보 배치 간 위치·순서 다양성의 기준값.
    pub diversity_threshold: f32,
}

impl Default for StagnationConfig {
    fn default() -> Self {
        Self {
            window_floor: STAGNATION_WINDOW_FLOOR,
            window_item_factor: STAGNATION_WINDOW_ITEM_FACTOR,
            efficiency_eps: STAGNATION_EFFICIENCY_EPS,
            diversity_threshold: STAGNATION_DIVERSITY_THRESHOLD,
        }
    }
}

impl StagnationConfig {
    /// ```text
    /// 최근 max(100, 3 × 패턴 수) 회 동안
    /// ```
    #[inline]
    pub fn window(&self, n_patterns: usize) -> usize {
        self.window_floor
            .max(self.window_item_factor.saturating_mul(n_patterns))
            .max(1)
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 관측 신호
// ────────────────────────────────────────────────────────────────────────────

/// 문서가 나열한 5개 지표의 현재 관측값 스냅샷.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StagnationSignals {
    /// ① 관측 창 안의 최고 효율 − 창 시작 시점 최고 효율 (0.0~1.0 밀도 단위).
    pub best_efficiency_gain: f32,
    /// ② 후보 배치 간 위치·순서 다양성 (0.0~1.0).
    pub population_diversity: f32,
    /// ③ 이동 성공률 (0.0~1.0). 관측이 없으면 0.0.
    pub move_success_rate: f32,
    /// ④ 우측 경계 패턴 시그니처가 바뀌지 않은 연속 반복 수.
    pub right_boundary_freeze_iters: usize,
    /// ⑤ 2회 이상 실패한 패턴 쌍의 수.
    pub repeated_pair_failures: usize,
    /// 현재 관측 창 크기.
    pub window: usize,
    /// 지금까지 기록된 반복 수.
    pub observations: usize,
}

// ────────────────────────────────────────────────────────────────────────────
// 감지기
// ────────────────────────────────────────────────────────────────────────────

/// 5개 지표를 링버퍼로 추적하는 정체 감지기.
///
/// `is_stagnant()`는 문서가 명시한 두 조건의 AND로만 판정한다.
#[derive(Clone, Debug)]
pub struct StagnationDetector {
    config: StagnationConfig,
    /// `config.window(n_patterns)`로 고정된 관측 창 크기.
    window: usize,
    /// ① 최근 `window` 회의 효율값 링버퍼.
    efficiency: VecDeque<f32>,
    /// 링버퍼에서 밀려난(= 창 시작 이전) 효율값들의 최댓값.
    best_before_window: f32,
    /// ② 가장 최근에 보고된 후보군 다양성.
    diversity: f32,
    /// ③ 최근 `window` 회의 이동 성공/실패 링버퍼.
    moves: VecDeque<bool>,
    /// [`Self::moves`] 안의 성공 수 (증분 유지).
    move_successes: usize,
    /// ④ 직전 우측 경계 시그니처.
    last_boundary_sig: Option<u64>,
    /// ④ 시그니처가 유지된 연속 반복 수.
    boundary_freeze_iters: usize,
    /// ⑤ 정규화된 (min, max) 쌍별 실패 카운터.
    pair_failures: HashMap<(usize, usize), u32>,
    /// 정체 조건이 연속으로 성립한 반복 수.
    stagnant_iters: usize,
    /// 총 관측 반복 수.
    observations: usize,
}

impl StagnationDetector {
    /// `n_patterns`는 인스턴스의 총 패턴(배치 단위) 수다.
    pub fn new(config: StagnationConfig, n_patterns: usize) -> Self {
        let window = config.window(n_patterns);
        Self {
            config,
            window,
            efficiency: VecDeque::with_capacity(window),
            best_before_window: f32::NEG_INFINITY,
            diversity: 0.0,
            moves: VecDeque::with_capacity(window),
            move_successes: 0,
            last_boundary_sig: None,
            boundary_freeze_iters: 0,
            pair_failures: HashMap::new(),
            stagnant_iters: 0,
            observations: 0,
        }
    }

    /// 고정된 관측 창 크기.
    #[inline]
    pub fn window(&self) -> usize {
        self.window
    }

    /// 반복 1회를 기록한다.
    ///
    /// * `efficiency` — 0.0~1.0 밀도값.
    /// * `right_boundary_signature` — [`Self::right_boundary_signature`]의 결과.
    pub fn record_iteration(&mut self, efficiency: f32, right_boundary_signature: u64) {
        // ① 효율 링버퍼 갱신 — 창 밖으로 밀려난 값은 "창 시작 시점 최고 효율"에 흡수된다.
        if self.efficiency.len() == self.window {
            if let Some(evicted) = self.efficiency.pop_front() {
                if evicted > self.best_before_window {
                    self.best_before_window = evicted;
                }
            }
        }
        self.efficiency.push_back(efficiency);

        // ④ 우측 경계 시그니처 고정 시간.
        match self.last_boundary_sig {
            Some(prev) if prev == right_boundary_signature => {
                self.boundary_freeze_iters = self.boundary_freeze_iters.saturating_add(1);
            }
            _ => {
                self.last_boundary_sig = Some(right_boundary_signature);
                self.boundary_freeze_iters = 0;
            }
        }

        self.observations = self.observations.saturating_add(1);

        // 정체 지속 길이.
        if self.is_stagnant() {
            self.stagnant_iters = self.stagnant_iters.saturating_add(1);
        } else {
            self.stagnant_iters = 0;
        }
    }

    /// ③ 이동 시도 1회의 성공 여부를 기록한다.
    pub fn record_move(&mut self, succeeded: bool) {
        if self.moves.len() == self.window {
            if let Some(old) = self.moves.pop_front() {
                if old {
                    self.move_successes -= 1;
                }
            }
        }
        self.moves.push_back(succeeded);
        if succeeded {
            self.move_successes += 1;
        }
    }

    /// ⑤ 동일 패턴 쌍의 충돌/이동 실패를 기록한다. 키는 `(min, max)`로 정규화된다.
    pub fn record_pair_failure(&mut self, a: usize, b: usize) {
        let key = if a <= b { (a, b) } else { (b, a) };
        *self.pair_failures.entry(key).or_insert(0) += 1;
    }

    /// ② 후보군 다양성을 보고한다. [`Self::population_diversity`]의 결과를 넣는다.
    ///
    /// 이 값을 한 번도 보고하지 않으면 다양성은 `0.0`(= 기준 이하)으로 간주되어
    /// 판정이 사실상 ① 조건만으로 이루어진다.
    pub fn record_population(&mut self, diversity: f32) {
        self.diversity = diversity.clamp(0.0, 1.0);
    }

    /// 현재 5개 지표 스냅샷.
    pub fn signals(&self) -> StagnationSignals {
        StagnationSignals {
            best_efficiency_gain: self.best_efficiency_gain(),
            population_diversity: self.diversity,
            move_success_rate: self.move_success_rate(),
            right_boundary_freeze_iters: self.boundary_freeze_iters,
            repeated_pair_failures: self.repeated_pair_failures(),
            window: self.window,
            observations: self.observations,
        }
    }

    /// 문서가 명시한 두 조건의 **AND**.
    ///
    /// ```text
    /// 최근 max(100, 3 × 패턴 수) 회 동안
    /// 효율 개선량 < 0.01%p
    /// AND
    /// 후보 배치 간 위치·순서 다양성이 기준 이하
    /// ```
    ///
    /// 관측 창이 아직 차지 않았다면(= 관측 부족) 정체로 보지 않는다.
    pub fn is_stagnant(&self) -> bool {
        self.observations >= self.window
            && self.best_efficiency_gain() < self.config.efficiency_eps
            && self.diversity < self.config.diversity_threshold
    }

    /// 탈출 강도 보간 계수 `t ∈ [0, 1]`.
    ///
    /// 정체 지속 비율(창 대비)에 ③④⑤ 보조 신호를 결합한다. 문서의 `5~10%`,
    /// `15~30%` 같은 범위는 이 값으로 [`crate::escape::lerp_ratio`] 보간된다.
    pub fn stagnation_intensity(&self) -> f32 {
        if self.observations == 0 {
            return 0.0;
        }
        let w = self.window.max(1) as f32;

        // 정체가 창 하나만큼 지속되면 포화.
        let persistence = (self.stagnant_iters as f32 / w).clamp(0.0, 1.0);
        // ④ 우측 경계가 창 하나만큼 굳어 있으면 포화.
        let freeze = (self.boundary_freeze_iters as f32 / w).clamp(0.0, 1.0);
        // ③ 이동 성공률의 여집합. 관측이 없으면 0으로 둔다(무정보 ≠ 실패).
        let move_failure = if self.moves.is_empty() {
            0.0
        } else {
            (1.0 - self.move_success_rate()).clamp(0.0, 1.0)
        };
        // ⑤ 반복 실패 쌍.
        let saturation = (w * PAIR_SATURATION_WINDOW_RATIO).max(PAIR_SATURATION_FLOOR);
        let pair = (self.repeated_pair_failures() as f32 / saturation).clamp(0.0, 1.0);

        (W_INTENSITY_PERSISTENCE * persistence
            + W_INTENSITY_FREEZE * freeze
            + W_INTENSITY_MOVE_FAILURE * move_failure
            + W_INTENSITY_PAIR * pair)
            .clamp(0.0, 1.0)
    }

    /// 탈출을 실행한 뒤 관측을 초기화한다. 설정과 창 크기는 유지한다.
    pub fn reset(&mut self) {
        self.efficiency.clear();
        self.best_before_window = f32::NEG_INFINITY;
        self.diversity = 0.0;
        self.moves.clear();
        self.move_successes = 0;
        self.last_boundary_sig = None;
        self.boundary_freeze_iters = 0;
        self.pair_failures.clear();
        self.stagnant_iters = 0;
        self.observations = 0;
    }

    // ── 내부 계산 ────────────────────────────────────────────────────────────

    /// ① `관측 창 안의 최고 효율 − 창 시작 시점 최고 효율`.
    fn best_efficiency_gain(&self) -> f32 {
        let Some(&first) = self.efficiency.front() else {
            return 0.0;
        };
        let best_in_window = self
            .efficiency
            .iter()
            .copied()
            .fold(f32::NEG_INFINITY, f32::max);
        // 아직 아무것도 밀려나지 않았다면 창의 첫 관측값이 기준선이다.
        let baseline = if self.best_before_window.is_finite() {
            self.best_before_window
        } else {
            first
        };
        (best_in_window - baseline).max(0.0)
    }

    /// ③ 이동 성공률.
    fn move_success_rate(&self) -> f32 {
        if self.moves.is_empty() {
            0.0
        } else {
            self.move_successes as f32 / self.moves.len() as f32
        }
    }

    /// ⑤ 2회 이상 실패한 쌍의 수.
    fn repeated_pair_failures(&self) -> usize {
        self.pair_failures
            .values()
            .filter(|&&c| c >= PAIR_FAILURE_REPEAT_MIN)
            .count()
    }

    // ── 정적 헬퍼 ────────────────────────────────────────────────────────────

    /// ② 후보 배치 간 **위치·순서** 다양성 (0.0~1.0).
    ///
    /// 각 해에서 아이템별 (x, y) 중심 좌표 벡터를 만들고,
    /// * 해들 사이의 평균 정규화 거리
    /// * 좌우 정렬 순서(rank)의 평균 Spearman 불일치
    ///
    /// 를 결합한다. 해가 1개 이하면 `0.0`.
    pub fn population_diversity(solutions: &[SPSolution]) -> f32 {
        if solutions.len() < 2 {
            return 0.0;
        }
        // 해마다 좌표를 **한 번만** SoA로 뽑고 x-순위도 미리 계산한다.
        // (예전 구현은 쌍마다 좌표를 다시 훑고 `ranks_by_x`를 2회씩 다시 정렬했다 —
        //  32개 해면 정렬 992회 → 32회로 줄어든다.)
        let centers: Vec<CentersSoA> = solutions.iter().map(CentersSoA::from_solution).collect();
        let diagonals: Vec<f32> = solutions
            .iter()
            .map(|s| {
                let w = s.strip().width;
                let h = s.strip().fixed_height;
                (w * w + h * h).sqrt()
            })
            .collect();

        let mut acc = 0.0f32;
        let mut n_pairs = 0usize;
        for i in 0..centers.len() {
            for j in (i + 1)..centers.len() {
                let diag = diagonals[i].max(diagonals[j]).max(f32::EPSILON);
                let pos = positional_divergence_soa(&centers[i], &centers[j], diag);
                let ord = order_disagreement_soa(&centers[i], &centers[j]);
                acc += W_DIVERSITY_POSITION * pos + W_DIVERSITY_ORDER * ord;
                n_pairs += 1;
            }
        }
        if n_pairs == 0 {
            0.0
        } else {
            (acc / n_pairs as f32).clamp(0.0, 1.0)
        }
    }

    /// ④ 우측 경계 패턴 시그니처.
    ///
    /// strip 우측 끝(`x_max` 최대)에서 폭의 2% 밴드 안에 걸친 아이템들의 `item_id`를
    /// 정렬해 해시한다. 이 값이 바뀌지 않는 동안 오른쪽 경계 패턴은 고정된 것이다.
    pub fn right_boundary_signature(sep: &Separator) -> u64 {
        let layout = &sep.prob.layout();
        let right_edge = layout
            .placed_items()
            .values()
            .map(|pi| pi.shape().bbox().x_max)
            .fold(f32::NEG_INFINITY, f32::max);
        if !right_edge.is_finite() {
            // 배치된 아이템이 없다.
            return 0;
        }
        let band = (sep.prob.strip_width() * RIGHT_BOUNDARY_BAND_RATIO).max(0.0);
        let cutoff = right_edge - band;

        let mut ids: Vec<usize> = layout
            .placed_items()
            .values()
            .filter(|pi| pi.shape().bbox().x_max >= cutoff)
            .map(|pi| pi.item().idx())
            .collect();
        ids.sort_unstable();

        let mut hasher = DefaultHasher::new();
        ids.len().hash(&mut hasher);
        for id in ids {
            id.hash(&mut hasher);
        }
        hasher.finish()
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 다양성 계산 보조 함수
// ────────────────────────────────────────────────────────────────────────────

/// 해 하나의 아이템 중심 좌표를 SoA(Structure of Arrays)로 담는다.
///
/// 해마다 `PItemKey`가 다르므로 `(item_id, x, y)` 순으로 정렬해 **정준 순서**를 만들고,
/// 그 인덱스를 해 사이의 대응 관계로 쓴다.
///
/// `ranks`는 x 오름차순 순위를 미리 계산해 둔 것이다. 쌍 루프에서 재정렬을 없애기 위해
/// 해 하나당 한 번만 만든다. 순위는 정수지만 SIMD 누산에 바로 쓰려고 `f32`로 보관한다
/// (아이템 수가 `2^24`를 넘지 않는 한 정확히 표현된다).
#[derive(Debug, Clone, Default)]
#[repr(align(32))]
struct CentersSoA {
    xs: Vec<f32>,
    ys: Vec<f32>,
    ranks: Vec<f32>,
}

impl CentersSoA {
    #[inline]
    fn len(&self) -> usize {
        self.xs.len()
    }

    /// 해 하나를 정준 순서의 SoA로 변환한다.
    fn from_solution(sol: &SPSolution) -> Self {
        let items = &sol.layout_snapshot().placed_items();
        let mut rows: Vec<(usize, f32, f32)> = Vec::with_capacity(items.len());
        rows.extend(items.values().map(|pi| {
            let bb = pi.shape().bbox();
            (
                pi.item().idx(),
                (bb.x_min + bb.x_max) * 0.5,
                (bb.y_min + bb.y_max) * 0.5,
            )
        }));
        rows.sort_by(|a, b| {
            a.0.cmp(&b.0)
                .then(a.1.partial_cmp(&b.1).unwrap_or(Ordering::Equal))
                .then(a.2.partial_cmp(&b.2).unwrap_or(Ordering::Equal))
        });

        let mut xs = Vec::with_capacity(rows.len());
        let mut ys = Vec::with_capacity(rows.len());
        for &(_, x, y) in &rows {
            xs.push(x);
            ys.push(y);
        }
        let ranks = ranks_by_x_soa(&xs);
        Self { xs, ys, ranks }
    }
}

/// x 좌표 기준 좌우 정렬 순위. 동률은 인덱스 순으로 깨진다(원본 `ranks_by_x`와 동일 정의).
fn ranks_by_x_soa(xs: &[f32]) -> Vec<f32> {
    let n = xs.len();
    let mut order: Vec<u32> = (0..n as u32).collect();
    order.sort_unstable_by(|&i, &j| {
        xs[i as usize]
            .partial_cmp(&xs[j as usize])
            .unwrap_or(Ordering::Equal)
            .then(i.cmp(&j))
    });
    let mut ranks = vec![0.0f32; n];
    for (rank, &idx) in order.iter().enumerate() {
        ranks[idx as usize] = rank as f32;
    }
    ranks
}

/// 두 배치 사이의 평균 정규화 거리 (0.0~1.0). `diag`는 strip 대각선 길이.
fn positional_divergence_soa(a: &CentersSoA, b: &CentersSoA, diag: f32) -> f32 {
    let n = a.len().min(b.len());
    if n == 0 {
        return 0.0;
    }
    let sum = distance_sum(&a.xs[..n], &a.ys[..n], &b.xs[..n], &b.ys[..n]);
    ((sum / n as f32) / diag.max(f32::EPSILON)).clamp(0.0, 1.0)
}

/// 좌우 정렬 순서의 Spearman 불일치 (footrule, 0.0~1.0).
///
/// 무작위 순열의 기대 평균 변위가 `n/3`이므로 그것으로 정규화한다.
fn order_disagreement_soa(a: &CentersSoA, b: &CentersSoA) -> f32 {
    let n = a.len().min(b.len());
    if n < 2 {
        return 0.0;
    }
    // 길이가 같으면 미리 계산해 둔 순위를 그대로 쓴다(= 거의 모든 경우).
    // 한쪽이 더 길면 접두부의 순위는 달라지므로 그때만 다시 계산한다.
    let tmp_a = (a.len() != n).then(|| ranks_by_x_soa(&a.xs[..n]));
    let tmp_b = (b.len() != n).then(|| ranks_by_x_soa(&b.xs[..n]));
    let ra: &[f32] = tmp_a.as_deref().unwrap_or(&a.ranks);
    let rb: &[f32] = tmp_b.as_deref().unwrap_or(&b.ranks);

    let sum = abs_diff_sum(&ra[..n], &rb[..n]);
    let mean_shift = sum / n as f32;
    let expected_random = (n as f32 / 3.0).max(1.0);
    (mean_shift / expected_random).clamp(0.0, 1.0)
}

// ────────────────────────────────────────────────────────────────────────────
// 누산 커널 — SIMD / 스칼라
//
// `simd` feature가 꺼지면 아래 `*_scalar`만 남고 동작·반환값은 완전히 동일하다.
// ────────────────────────────────────────────────────────────────────────────

/// `Σ sqrt(dx² + dy²)`.
#[inline]
fn distance_sum(ax: &[f32], ay: &[f32], bx: &[f32], by: &[f32]) -> f32 {
    #[cfg(feature = "simd")]
    {
        #[cfg(target_arch = "x86_64")]
        {
            if std::arch::is_x86_feature_detected!("avx512f") {
                return unsafe { distance_sum_16(ax, ay, bx, by) };
            }
            if std::arch::is_x86_feature_detected!("avx2") {
                return unsafe { distance_sum_8(ax, ay, bx, by) };
            }
        }
        distance_sum_kernel::<4>(ax, ay, bx, by)
    }
    #[cfg(not(feature = "simd"))]
    {
        distance_sum_scalar(ax, ay, bx, by)
    }
}

/// `Σ |a − b|`.
#[inline]
fn abs_diff_sum(a: &[f32], b: &[f32]) -> f32 {
    #[cfg(feature = "simd")]
    {
        #[cfg(target_arch = "x86_64")]
        {
            if std::arch::is_x86_feature_detected!("avx512f") {
                return unsafe { abs_diff_sum_16(a, b) };
            }
            if std::arch::is_x86_feature_detected!("avx2") {
                return unsafe { abs_diff_sum_8(a, b) };
            }
        }
        abs_diff_sum_kernel::<4>(a, b)
    }
    #[cfg(not(feature = "simd"))]
    {
        abs_diff_sum_scalar(a, b)
    }
}

/// 스칼라 기준 구현 — `simd` feature가 꺼졌을 때의 경로이자 SIMD 검증용 레퍼런스.
#[cfg_attr(feature = "simd", allow(dead_code))]
fn distance_sum_scalar(ax: &[f32], ay: &[f32], bx: &[f32], by: &[f32]) -> f32 {
    (0..ax.len())
        .map(|k| {
            let dx = ax[k] - bx[k];
            let dy = ay[k] - by[k];
            (dx * dx + dy * dy).sqrt()
        })
        .sum()
}

/// [`distance_sum_scalar`] 참조.
#[cfg_attr(feature = "simd", allow(dead_code))]
fn abs_diff_sum_scalar(a: &[f32], b: &[f32]) -> f32 {
    (0..a.len()).map(|k| (a[k] - b[k]).abs()).sum()
}

// region: SIMD 커널 (폭에 제네릭한 한 벌)

#[cfg(feature = "simd")]
use std::simd::Simd;

#[cfg(feature = "simd")]
type F32xN<const W: usize> = Simd<f32, W>;

#[cfg(feature = "simd")]
#[inline(always)]
fn distance_sum_kernel<const W: usize>(ax: &[f32], ay: &[f32], bx: &[f32], by: &[f32]) -> f32 {
    use std::simd::StdFloat;
    use std::simd::prelude::SimdFloat;

    let n = ax.len();
    let chunks = n / W;
    let mut acc = F32xN::<W>::splat(0.0);
    for chunk in 0..chunks {
        let i = chunk * W;
        let dx = F32xN::<W>::from_slice(&ax[i..i + W]) - F32xN::<W>::from_slice(&bx[i..i + W]);
        let dy = F32xN::<W>::from_slice(&ay[i..i + W]) - F32xN::<W>::from_slice(&by[i..i + W]);
        acc += (dx * dx + dy * dy).sqrt();
    }
    let mut sum = acc.reduce_sum();
    // 꼬리(remainder)는 스칼라로 처리한다.
    for k in (chunks * W)..n {
        let dx = ax[k] - bx[k];
        let dy = ay[k] - by[k];
        sum += (dx * dx + dy * dy).sqrt();
    }
    sum
}

#[cfg(feature = "simd")]
#[inline(always)]
fn abs_diff_sum_kernel<const W: usize>(a: &[f32], b: &[f32]) -> f32 {
    use std::simd::prelude::SimdFloat;

    let n = a.len();
    let chunks = n / W;
    let mut acc = F32xN::<W>::splat(0.0);
    for chunk in 0..chunks {
        let i = chunk * W;
        let d = F32xN::<W>::from_slice(&a[i..i + W]) - F32xN::<W>::from_slice(&b[i..i + W]);
        acc += d.abs();
    }
    let mut sum = acc.reduce_sum();
    for k in (chunks * W)..n {
        sum += (a[k] - b[k]).abs();
    }
    sum
}

// endregion

// region: Target-feature 래퍼 (x86_64 전용 — wasm32에서는 W=4 경로만 쓰인다)

#[cfg(all(feature = "simd", target_arch = "x86_64"))]
#[target_feature(enable = "avx2")]
unsafe fn distance_sum_8(ax: &[f32], ay: &[f32], bx: &[f32], by: &[f32]) -> f32 {
    distance_sum_kernel::<8>(ax, ay, bx, by)
}

#[cfg(all(feature = "simd", target_arch = "x86_64"))]
#[target_feature(enable = "avx512f")]
unsafe fn distance_sum_16(ax: &[f32], ay: &[f32], bx: &[f32], by: &[f32]) -> f32 {
    distance_sum_kernel::<16>(ax, ay, bx, by)
}

#[cfg(all(feature = "simd", target_arch = "x86_64"))]
#[target_feature(enable = "avx2")]
unsafe fn abs_diff_sum_8(a: &[f32], b: &[f32]) -> f32 {
    abs_diff_sum_kernel::<8>(a, b)
}

#[cfg(all(feature = "simd", target_arch = "x86_64"))]
#[target_feature(enable = "avx512f")]
unsafe fn abs_diff_sum_16(a: &[f32], b: &[f32]) -> f32 {
    abs_diff_sum_kernel::<16>(a, b)
}

// endregion

// ────────────────────────────────────────────────────────────────────────────
// 단위 테스트 — 외부 데이터 없이 순수 계산만 검증한다.
// ────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── AoS 레퍼런스 구현 ────────────────────────────────────────────────────
    // 최적화 이전의 원본 수식. SoA/SIMD 경로는 반드시 이 결과와 실질적으로 같아야 한다.

    /// 두 배치 사이의 평균 정규화 거리 (0.0~1.0).
    fn positional_divergence(a: &[(f32, f32)], b: &[(f32, f32)], diag: f32) -> f32 {
        let n = a.len().min(b.len());
        if n == 0 {
            return 0.0;
        }
        let sum: f32 = (0..n)
            .map(|k| {
                let dx = a[k].0 - b[k].0;
                let dy = a[k].1 - b[k].1;
                (dx * dx + dy * dy).sqrt()
            })
            .sum();
        ((sum / n as f32) / diag.max(f32::EPSILON)).clamp(0.0, 1.0)
    }

    /// x 좌표 기준 좌우 정렬 순위를 계산한다.
    fn ranks_by_x(centers: &[(f32, f32)]) -> Vec<usize> {
        let mut order: Vec<usize> = (0..centers.len()).collect();
        order.sort_by(|&i, &j| {
            centers[i]
                .0
                .partial_cmp(&centers[j].0)
                .unwrap_or(Ordering::Equal)
                .then(i.cmp(&j))
        });
        let mut ranks = vec![0usize; centers.len()];
        for (rank, &idx) in order.iter().enumerate() {
            ranks[idx] = rank;
        }
        ranks
    }

    /// 좌우 정렬 순서의 Spearman 불일치 (footrule, 0.0~1.0).
    fn order_disagreement(a: &[(f32, f32)], b: &[(f32, f32)]) -> f32 {
        let n = a.len().min(b.len());
        if n < 2 {
            return 0.0;
        }
        let ra = ranks_by_x(&a[..n]);
        let rb = ranks_by_x(&b[..n]);
        let sum: f32 = (0..n)
            .map(|k| (ra[k] as f32 - rb[k] as f32).abs())
            .sum::<f32>();
        let mean_shift = sum / n as f32;
        let expected_random = (n as f32 / 3.0).max(1.0);
        (mean_shift / expected_random).clamp(0.0, 1.0)
    }

    /// AoS 좌표 목록을 `CentersSoA`로 변환한다(테스트 전용 생성자).
    fn soa(centers: &[(f32, f32)]) -> CentersSoA {
        let xs: Vec<f32> = centers.iter().map(|c| c.0).collect();
        let ys: Vec<f32> = centers.iter().map(|c| c.1).collect();
        let ranks = ranks_by_x_soa(&xs);
        CentersSoA { xs, ys, ranks }
    }

    /// 결정적 LCG — 외부 의존 없이 재현 가능한 좌표를 만든다.
    fn lcg_centers(seed: &mut u32, n: usize, span: f32) -> Vec<(f32, f32)> {
        let mut next = || {
            *seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (*seed >> 8) as f32 / (1u32 << 24) as f32
        };
        (0..n).map(|_| (next() * span, next() * span)).collect()
    }

    /// `window() = max(window_floor, window_item_factor × n_patterns)`
    #[test]
    fn window_follows_document_formula() {
        let cfg = StagnationConfig::default();
        assert_eq!(cfg.window_floor, 100);
        assert_eq!(cfg.window_item_factor, 3);
        // 패턴이 적으면 하한 100.
        assert_eq!(cfg.window(0), 100);
        assert_eq!(cfg.window(10), 100);
        assert_eq!(cfg.window(33), 100);
        // 3 × 패턴 수가 100을 넘으면 그쪽이 채택된다.
        assert_eq!(cfg.window(34), 102);
        assert_eq!(cfg.window(156), 468);
    }

    /// 문서 임계값이 그대로 기본값으로 들어가야 한다. `0.01%p = 0.0001`
    #[test]
    fn default_config_matches_document_thresholds() {
        let cfg = StagnationConfig::default();
        assert_eq!(cfg.efficiency_eps, STAGNATION_EFFICIENCY_EPS);
        assert_eq!(cfg.efficiency_eps, 0.000_1);
        assert_eq!(cfg.diversity_threshold, STAGNATION_DIVERSITY_THRESHOLD);
    }

    fn detector(n_patterns: usize) -> StagnationDetector {
        StagnationDetector::new(StagnationConfig::default(), n_patterns)
    }

    /// 창이 차기 전에는 정체로 판정하지 않는다.
    #[test]
    fn not_stagnant_before_window_is_filled() {
        let mut d = detector(0);
        d.record_population(0.0);
        for _ in 0..(d.window() - 1) {
            d.record_iteration(0.5, 7);
        }
        assert!(!d.is_stagnant());
        assert_eq!(d.signals().observations, 99);
        d.record_iteration(0.5, 7);
        assert!(d.is_stagnant());
    }

    /// ① 개선량 0 + ② 다양성 기준 이하 → 정체.
    #[test]
    fn and_condition_requires_both_signals() {
        let window = detector(0).window();

        // 개선 없음 + 다양성 낮음 → 정체.
        let mut d = detector(0);
        d.record_population(0.0);
        for _ in 0..window {
            d.record_iteration(0.5, 1);
        }
        assert!(d.best_efficiency_gain() < d.config.efficiency_eps);
        assert!(d.is_stagnant());

        // 다양성만 높아져도 정체가 아니다 (AND).
        d.record_population(0.5);
        assert!(!d.is_stagnant());
    }

    /// ① 효율이 임계값 이상 개선되면 정체가 아니다.
    #[test]
    fn efficiency_gain_breaks_stagnation() {
        let mut d = detector(0);
        d.record_population(0.0);
        let window = d.window();
        for i in 0..window {
            // 총 개선량 = 0.099 ≫ 0.0001
            d.record_iteration(0.5 + i as f32 * 0.001, 1);
        }
        assert!(d.best_efficiency_gain() > d.config.efficiency_eps);
        assert!(!d.is_stagnant());

        // 0.01%p 미만의 개선은 정체로 본다.
        let mut d2 = detector(0);
        d2.record_population(0.0);
        for i in 0..window {
            d2.record_iteration(0.5 + i as f32 * 1.0e-7, 1);
        }
        assert!(d2.best_efficiency_gain() < d2.config.efficiency_eps);
        assert!(d2.is_stagnant());
    }

    /// ④ 우측 경계 시그니처가 바뀌면 고정 시간이 리셋된다.
    #[test]
    fn boundary_freeze_counter() {
        let mut d = detector(0);
        for _ in 0..5 {
            d.record_iteration(0.5, 0xABCD);
        }
        // 첫 관측이 기준선이므로 5회 기록 → 4회 연속 유지.
        assert_eq!(d.signals().right_boundary_freeze_iters, 4);

        d.record_iteration(0.5, 0x1234);
        assert_eq!(d.signals().right_boundary_freeze_iters, 0);
    }

    /// ⑤ (min, max) 정규화 키 + 2회 이상만 집계.
    #[test]
    fn pair_failures_are_normalised_and_thresholded() {
        let mut d = detector(0);
        d.record_pair_failure(3, 1);
        assert_eq!(d.signals().repeated_pair_failures, 0);
        // (1,3) 과 (3,1) 은 같은 키다.
        d.record_pair_failure(1, 3);
        assert_eq!(d.signals().repeated_pair_failures, 1);

        d.record_pair_failure(7, 9);
        assert_eq!(d.signals().repeated_pair_failures, 1);
        d.record_pair_failure(9, 7);
        assert_eq!(d.signals().repeated_pair_failures, 2);
    }

    /// ③ 이동 성공률은 창 크기 링버퍼로 유지된다.
    #[test]
    fn move_success_rate_uses_ring_buffer() {
        let mut d = detector(0);
        assert_eq!(d.signals().move_success_rate, 0.0);

        for _ in 0..10 {
            d.record_move(true);
        }
        for _ in 0..10 {
            d.record_move(false);
        }
        assert!((d.signals().move_success_rate - 0.5).abs() < 1e-6);

        // 창(100)을 넘겨 실패로만 채우면 성공률이 0으로 수렴한다.
        for _ in 0..d.window() {
            d.record_move(false);
        }
        assert_eq!(d.signals().move_success_rate, 0.0);
    }

    /// 강도는 항상 [0, 1]이고, 모든 보조 신호가 포화하면 1.0에 도달한다.
    #[test]
    fn intensity_is_bounded_and_saturates() {
        let mut d = detector(0);
        assert_eq!(d.stagnation_intensity(), 0.0);

        d.record_population(0.0);
        let window = d.window();
        // 두 배의 창을 정체 상태로 기록 → persistence, freeze 포화.
        for _ in 0..(window * 2) {
            d.record_iteration(0.5, 42);
            d.record_move(false);
        }
        // ⑤ 포화: max(4, 100 × 0.05) = 5 쌍.
        for p in 0..8 {
            d.record_pair_failure(p, p + 100);
            d.record_pair_failure(p, p + 100);
        }
        let t = d.stagnation_intensity();
        assert!((0.0..=1.0).contains(&t));
        assert!(t > 0.99, "모든 신호 포화 시 1.0에 근접해야 한다: {t}");
    }

    /// 강도 가중치 합은 1.00이어야 포화 시 1.0이 된다.
    #[test]
    fn intensity_weights_sum_to_one() {
        let sum = W_INTENSITY_PERSISTENCE
            + W_INTENSITY_FREEZE
            + W_INTENSITY_MOVE_FAILURE
            + W_INTENSITY_PAIR;
        assert!((sum - 1.0).abs() < 1e-6);

        let dsum = W_DIVERSITY_POSITION + W_DIVERSITY_ORDER;
        assert!((dsum - 1.0).abs() < 1e-6);
    }

    /// `reset()`은 관측만 지우고 창 크기는 유지한다.
    #[test]
    fn reset_clears_observations_only() {
        let mut d = detector(200);
        let w = d.window();
        assert_eq!(w, 600);
        d.record_population(0.0);
        for _ in 0..w {
            d.record_iteration(0.5, 1);
            d.record_move(false);
        }
        d.record_pair_failure(1, 2);
        d.record_pair_failure(1, 2);
        assert!(d.is_stagnant());

        d.reset();
        assert_eq!(d.window(), w);
        assert_eq!(d.signals().observations, 0);
        assert_eq!(d.signals().repeated_pair_failures, 0);
        assert_eq!(d.signals().right_boundary_freeze_iters, 0);
        assert_eq!(d.stagnation_intensity(), 0.0);
        assert!(!d.is_stagnant());
    }

    /// 해가 1개 이하면 다양성은 0.0.
    #[test]
    fn population_diversity_needs_two_solutions() {
        assert_eq!(StagnationDetector::population_diversity(&[]), 0.0);
    }

    /// 동일한 배치는 위치 divergence 0.
    #[test]
    fn positional_divergence_of_identical_layouts_is_zero() {
        let a = vec![(0.0, 0.0), (10.0, 5.0), (20.0, 1.0)];
        assert_eq!(positional_divergence(&a, &a, 100.0), 0.0);
    }

    /// 평균 거리 / 대각선으로 정규화된다.
    #[test]
    fn positional_divergence_is_normalised_by_diagonal() {
        let a = vec![(0.0, 0.0), (0.0, 0.0)];
        let b = vec![(3.0, 4.0), (6.0, 8.0)]; // 거리 5, 10 → 평균 7.5
        let d = positional_divergence(&a, &b, 100.0);
        assert!((d - 0.075).abs() < 1e-6, "got {d}");
        // 대각선보다 큰 변위는 1.0으로 잘린다.
        assert_eq!(positional_divergence(&a, &b, 1.0), 1.0);
    }

    /// 순서가 같으면 불일치 0, 완전 역순이면 큰 값.
    #[test]
    fn order_disagreement_detects_reordering() {
        let a = vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)];
        assert_eq!(order_disagreement(&a, &a), 0.0);

        // 정준 인덱스는 그대로지만 x 순서가 완전히 뒤집힌 경우.
        let b = vec![(3.0, 0.0), (2.0, 0.0), (1.0, 0.0), (0.0, 0.0)];
        let d = order_disagreement(&a, &b);
        // 평균 변위 = (3+1+1+3)/4 = 2, 기대 무작위 변위 = 4/3 → clamp(1.5) = 1.0
        assert_eq!(d, 1.0);

        // 인접한 두 개만 뒤바뀐 경우는 작은 값.
        let c = vec![(1.0, 0.0), (0.0, 0.0), (2.0, 0.0), (3.0, 0.0)];
        let d2 = order_disagreement(&a, &c);
        assert!(d2 > 0.0 && d2 < 0.5, "got {d2}");
    }

    /// 순위 계산이 x 오름차순인지 확인.
    #[test]
    fn ranks_by_x_is_ascending() {
        let c = vec![(5.0, 0.0), (1.0, 0.0), (3.0, 0.0)];
        assert_eq!(ranks_by_x(&c), vec![2, 0, 1]);
        // 동률은 인덱스 순으로 안정 정렬된다.
        let t = vec![(1.0, 0.0), (1.0, 9.0)];
        assert_eq!(ranks_by_x(&t), vec![0, 1]);
    }

    /// 빈 입력에 대한 방어.
    #[test]
    fn helpers_handle_empty_input() {
        assert_eq!(positional_divergence(&[], &[], 10.0), 0.0);
        assert_eq!(order_disagreement(&[], &[]), 0.0);
        assert_eq!(order_disagreement(&[(0.0, 0.0)], &[(1.0, 1.0)]), 0.0);
        assert!(ranks_by_x(&[]).is_empty());

        // SoA 경로도 동일하게 방어된다.
        let empty = soa(&[]);
        assert_eq!(positional_divergence_soa(&empty, &empty, 10.0), 0.0);
        assert_eq!(order_disagreement_soa(&empty, &empty), 0.0);
        assert_eq!(
            order_disagreement_soa(&soa(&[(0.0, 0.0)]), &soa(&[(1.0, 1.0)])),
            0.0
        );
    }

    /// 미리 계산한 SoA 순위는 AoS 레퍼런스와 완전히 같아야 한다.
    #[test]
    fn ranks_by_x_soa_matches_reference() {
        let mut seed = 0x5EED_1234;
        for n in [0usize, 1, 2, 3, 7, 8, 15, 16, 17, 156] {
            let c = lcg_centers(&mut seed, n, 1000.0);
            let xs: Vec<f32> = c.iter().map(|p| p.0).collect();
            let expect: Vec<f32> = ranks_by_x(&c).into_iter().map(|r| r as f32).collect();
            assert_eq!(ranks_by_x_soa(&xs), expect, "n={n}");
        }
        // 동률은 인덱스 순.
        assert_eq!(ranks_by_x_soa(&[1.0, 1.0, 0.0]), vec![1.0, 2.0, 0.0]);
    }

    /// SoA(+SIMD) 경로가 AoS 레퍼런스와 실질적으로 같은 값을 낸다.
    ///
    /// 위치 항은 f32 누산 순서 차이만큼의 미세 오차를 허용하고, 순서 항은 정수 합이라
    /// 완전히 일치해야 한다. 길이가 다른 쌍(접두부 순위 재계산 경로)도 함께 본다.
    #[test]
    fn soa_path_matches_aos_reference() {
        let mut seed = 0xC0FF_EE01;
        for (na, nb) in [
            (0usize, 0usize),
            (1, 1),
            (2, 2),
            (5, 5),
            (8, 8),
            (17, 17),
            (156, 156),
            (156, 100),
            (33, 40),
        ] {
            let a = lcg_centers(&mut seed, na, 2000.0);
            let b = lcg_centers(&mut seed, nb, 2000.0);
            let (sa, sb) = (soa(&a), soa(&b));

            let diag = 3000.0f32;
            let want_pos = positional_divergence(&a, &b, diag);
            let got_pos = positional_divergence_soa(&sa, &sb, diag);
            let tol = want_pos.abs().max(1.0) * 1e-5;
            assert!(
                (want_pos - got_pos).abs() <= tol,
                "pos 불일치 na={na} nb={nb}: {want_pos} vs {got_pos}"
            );

            let want_ord = order_disagreement(&a, &b);
            let got_ord = order_disagreement_soa(&sa, &sb);
            assert!(
                (want_ord - got_ord).abs() <= 1e-6,
                "ord 불일치 na={na} nb={nb}: {want_ord} vs {got_ord}"
            );

            // 결합값은 항상 0.0~1.0 안에 있다.
            let combined = W_DIVERSITY_POSITION * got_pos + W_DIVERSITY_ORDER * got_ord;
            assert!((0.0..=1.0).contains(&combined), "combined={combined}");
        }
    }

    /// 동일 배치는 위치·순서 divergence가 모두 0 (SoA 경로).
    #[test]
    fn soa_identical_layouts_are_zero() {
        let mut seed = 0x1357_9BDF;
        let a = lcg_centers(&mut seed, 156, 1000.0);
        let sa = soa(&a);
        assert_eq!(positional_divergence_soa(&sa, &sa, 1000.0), 0.0);
        assert_eq!(order_disagreement_soa(&sa, &sa), 0.0);
    }
}

// ────────────────────────────────────────────────────────────────────────────
// SIMD ↔ 스칼라 동치성 검증
// ────────────────────────────────────────────────────────────────────────────

#[cfg(all(test, feature = "simd"))]
mod simd_parity_tests {
    use super::*;

    /// 결정적 LCG 좌표.
    fn lcg(seed: &mut u32, n: usize, span: f32) -> Vec<f32> {
        (0..n)
            .map(|_| {
                *seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                ((*seed >> 8) as f32 / (1u32 << 24) as f32) * span
            })
            .collect()
    }

    /// 모든 폭(4/8/16)의 거리 누산 커널이 스칼라와 상대오차 1e-5 안에서 일치한다.
    ///
    /// 폭은 `portable_simd`가 소프트웨어로도 에뮬레이션하므로 CPU와 무관하게 검증된다.
    #[test]
    fn distance_sum_kernels_match_scalar() {
        let mut seed = 0xABCD_0001;
        // 꼬리(remainder) 경로를 모두 밟도록 여러 길이를 본다.
        for n in [0usize, 1, 3, 4, 5, 7, 8, 9, 15, 16, 17, 31, 33, 156, 512] {
            let ax = lcg(&mut seed, n, 2000.0);
            let ay = lcg(&mut seed, n, 1000.0);
            let bx = lcg(&mut seed, n, 2000.0);
            let by = lcg(&mut seed, n, 1000.0);

            let want = distance_sum_scalar(&ax, &ay, &bx, &by);
            let tol = want.abs().max(1.0) * 1e-5;
            for (w, got) in [
                (4, distance_sum_kernel::<4>(&ax, &ay, &bx, &by)),
                (8, distance_sum_kernel::<8>(&ax, &ay, &bx, &by)),
                (16, distance_sum_kernel::<16>(&ax, &ay, &bx, &by)),
            ] {
                assert!(
                    (want - got).abs() <= tol,
                    "distance_sum W={w} n={n}: {want} vs {got}"
                );
            }
            // 런타임 디스패치 경로도 같은 값을 낸다.
            let got = distance_sum(&ax, &ay, &bx, &by);
            assert!(
                (want - got).abs() <= tol,
                "distance_sum dispatch n={n}: {want} vs {got}"
            );
        }
    }

    /// 순위 불일치 누산은 정수 합이라 폭과 무관하게 **완전히** 일치해야 한다.
    #[test]
    fn abs_diff_sum_kernels_match_scalar() {
        let mut seed = 0xABCD_0002;
        for n in [0usize, 1, 3, 4, 7, 8, 9, 16, 17, 33, 156, 512] {
            // 순위와 같은 성질(0..n의 정수)을 가진 입력.
            let a: Vec<f32> = (0..n).map(|k| k as f32).collect();
            let mut b: Vec<f32> = lcg(&mut seed, n, n as f32);
            for v in b.iter_mut() {
                *v = v.floor();
            }

            let want = abs_diff_sum_scalar(&a, &b);
            for (w, got) in [
                (4, abs_diff_sum_kernel::<4>(&a, &b)),
                (8, abs_diff_sum_kernel::<8>(&a, &b)),
                (16, abs_diff_sum_kernel::<16>(&a, &b)),
            ] {
                assert_eq!(want, got, "abs_diff_sum W={w} n={n}");
            }
            assert_eq!(want, abs_diff_sum(&a, &b), "abs_diff_sum dispatch n={n}");
        }
    }

    /// 실수 입력에서도 폭별 누산 오차가 판정을 바꿀 만큼 커지지 않는다.
    #[test]
    fn abs_diff_sum_kernels_match_scalar_on_reals() {
        let mut seed = 0xABCD_0003;
        for n in [5usize, 13, 100, 156] {
            let a = lcg(&mut seed, n, 500.0);
            let b = lcg(&mut seed, n, 500.0);
            let want = abs_diff_sum_scalar(&a, &b);
            let tol = want.abs().max(1.0) * 1e-5;
            for (w, got) in [
                (4, abs_diff_sum_kernel::<4>(&a, &b)),
                (8, abs_diff_sum_kernel::<8>(&a, &b)),
                (16, abs_diff_sum_kernel::<16>(&a, &b)),
            ] {
                assert!(
                    (want - got).abs() <= tol,
                    "abs_diff_sum W={w} n={n}: {want} vs {got}"
                );
            }
        }
    }
}
