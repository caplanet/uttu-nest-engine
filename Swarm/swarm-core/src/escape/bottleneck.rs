//! §2. 무작위 패턴이 아니라 **병목 패턴** 선택.
//!
//! ```text
//! 아무 패턴이나 랜덤하게 이동하면 이미 잘 배치된 영역까지 무너질 수 있다. 다음 패턴을
//! 우선 대상으로 선택해야 한다.
//!
//! - 현재 사용 길이의 오른쪽 끝을 결정하는 패턴
//! - 가장 큰 내부 공백과 접해 있는 패턴
//! - 주변 접촉 패턴 수가 많아 움직이지 못하는 패턴
//! - 이동 또는 삽입에 반복적으로 실패한 패턴
//! - 오목도가 높거나 형상이 커서 다른 패턴의 이동을 방해하는 패턴
//! ```
//!
//! ```text
//! EscapeScore(i)
//!  = 0.35 × RightBoundaryContribution
//!  + 0.30 × VoidAdjacency
//!  + 0.20 × BlockingDegree
//!  + 0.15 × MoveFailureFrequency
//! ```
//!
//! ```text
//! 점수가 높은 패턴부터 탈출 대상에 포함한다.
//! ```
//!
//! 네 항은 모두 **인스턴스 내 최대값 기준으로 0.0~1.0으로 정규화**해서 합산한다.
//! 가중치는 [`crate::escape`]에 확정 상수로 선언되어 있으며 이 파일은 그 값을
//! 그대로 곱해 쓴다(합 = 1.00).
//!
//! MultiPolygon 제약: [`Separator::has_place_groups`]가 참이면 하나의 의류 아이템을
//! 구성하는 여러 폴리곤은 동일한 배치 단위(unit)이므로 **대표 하나만** 탈출 대상
//! 목록에 올린다. 단위 점수는 멤버별 점수의 최대값을 취한다 — 멤버 하나라도 병목이면
//! 그 단위 전체가 병목이기 때문이다.

use std::collections::HashMap;

use crate::escape::{
    W_BLOCKING_DEGREE, W_MOVE_FAILURE_FREQUENCY, W_RIGHT_BOUNDARY_CONTRIBUTION, W_VOID_ADJACENCY,
};
use crate::optimizer::separator::Separator;
use jagua_rs::entities::{Layout, PItemKey};
use jagua_rs::geometry::convex_hull::convex_hull_from_points;
use jagua_rs::geometry::primitives::{Point, Rect, SPolygon};

// ── 내부 구현 상수 (문서 지정 가중치는 crate::escape에 있다) ──

/// 공백 래스터화 격자의 한 축 최대 분할 수. 문서 지침의 `최대 64x64`.
const MAX_VOID_GRID: usize = 64;

/// `BlockingDegree` 산출 시 아이템 bbox를 팽창시키는 비율(대각선 대비). 지침의 `2%`.
const BLOCKING_INFLATE_RATIO: f32 = 0.02;

/// 사용 길이를 "실제로 결정한다"고 볼 임계 비율. `x_max ≥ 0.99 × strip_length`이면 포화.
const RIGHT_BOUNDARY_SATURATION: f32 = 0.99;

/// 오목도(convex hull 면적 / 실제 면적) 상한. 극단적으로 가느다란 형상이 점수를
/// 독식하지 않도록 잘라낸다.
const MAX_CONCAVITY: f32 = 3.0;

/// 0 나눗셈 방지용 하한.
const EPS: f32 = 1.0e-6;

// ────────────────────────────────────────────────────────────────────────────
// EscapeScore
// ────────────────────────────────────────────────────────────────────────────

/// `EscapeScore(i)`의 네 항. 모두 0.0~1.0으로 정규화된 값이다.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EscapeScoreTerms {
    /// `현재 사용 길이의 오른쪽 끝을 결정하는 패턴` — `x_max / strip_length`.
    pub right_boundary_contribution: f32,
    /// `가장 큰 내부 공백과 접해 있는 패턴` — largest void bbox와의 접촉 길이/근접도.
    pub void_adjacency: f32,
    /// `주변 접촉 패턴 수가 많아 움직이지 못하는 패턴` +
    /// `오목도가 높거나 형상이 커서 다른 패턴의 이동을 방해하는 패턴`.
    pub blocking_degree: f32,
    /// `이동 또는 삽입에 반복적으로 실패한 패턴`.
    pub move_failure_frequency: f32,
}

impl EscapeScoreTerms {
    /// ```text
    /// EscapeScore(i)
    ///  = 0.35 × RightBoundaryContribution
    ///  + 0.30 × VoidAdjacency
    ///  + 0.20 × BlockingDegree
    ///  + 0.15 × MoveFailureFrequency
    /// ```
    #[inline]
    pub fn score(&self) -> f32 {
        W_RIGHT_BOUNDARY_CONTRIBUTION * self.right_boundary_contribution
            + W_VOID_ADJACENCY * self.void_adjacency
            + W_BLOCKING_DEGREE * self.blocking_degree
            + W_MOVE_FAILURE_FREQUENCY * self.move_failure_frequency
    }
}

/// 탈출 대상 후보 하나.
#[derive(Clone, Copy, Debug)]
pub struct BottleneckTarget {
    /// 레이아웃 내 배치 키. place group 모드에서는 **대표 멤버**의 키다.
    pub pk: PItemKey,
    /// 배치 단위 식별자. place group 모드에서는 대표 item id.
    pub item_id: usize,
    /// [`EscapeScoreTerms::score`] 결과. 정렬 기준.
    pub score: f32,
    /// 점수 분해 — 로깅/디버깅과 단계별 전략 선택에 쓴다.
    pub terms: EscapeScoreTerms,
}

/// [`select_bottleneck_items`]의 결과.
#[derive(Clone, Debug)]
pub struct BottleneckAnalysis {
    /// `점수가 높은 패턴부터` — score 내림차순으로 정렬되어 있다.
    pub targets: Vec<BottleneckTarget>,
    /// 가장 큰 내부 공백의 bounding box. 공백이 없으면 `None`.
    pub largest_void: Option<Rect>,
    /// 현재 사용 길이 (배치된 모든 아이템 `x_max`의 최대값).
    pub strip_length: f32,
}

impl BottleneckAnalysis {
    /// 상위 `n`개. 보유 개수보다 크면 전부 돌려준다.
    pub fn top(&self, n: usize) -> &[BottleneckTarget] {
        &self.targets[..n.min(self.targets.len())]
    }

    /// 상위 `ceil(len × ratio)`개 (최소 1개). 비어 있으면 빈 슬라이스.
    pub fn top_ratio(&self, ratio: f32) -> &[BottleneckTarget] {
        if self.targets.is_empty() {
            return &self.targets[..0];
        }
        let ratio = ratio.clamp(0.0, 1.0);
        let n = (self.targets.len() as f32 * ratio).ceil() as usize;
        self.top(n.clamp(1, self.targets.len()))
    }

    pub fn is_empty(&self) -> bool {
        self.targets.is_empty()
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 선택 진입점
// ────────────────────────────────────────────────────────────────────────────

/// 현재 배치에서 병목 패턴을 골라 `EscapeScore` 내림차순으로 돌려준다.
///
/// `move_failures`는 `item_id → 이동/삽입 실패 횟수` 누적 카운터다. 호출자가
/// 탐색 루프에서 계속 갱신해 넘긴다.
pub fn select_bottleneck_items(
    sep: &Separator,
    move_failures: &HashMap<usize, u32>,
) -> BottleneckAnalysis {
    let layout = &sep.prob.layout();

    // ── 0. 배치된 아이템 수집 ──
    let placed: Vec<(PItemKey, usize, Rect)> = layout
        .placed_items()
        .iter()
        .map(|(pk, pi)| (pk, pi.item().idx(), pi.shape().bbox()))
        .collect();

    if placed.is_empty() {
        return BottleneckAnalysis {
            targets: Vec::new(),
            largest_void: None,
            strip_length: 0.0,
        };
    }

    // ── 1. 현재 사용 길이 = 전체 아이템 x_max의 최대값 ──
    let strip_length = placed
        .iter()
        .map(|(_, _, bb)| bb.x_max)
        .fold(f32::NEG_INFINITY, f32::max)
        .max(EPS);

    // ── 2. 가장 큰 내부 공백 ──
    let largest_void = largest_internal_void(layout);

    // ── 3. 항별 원시값 산출 ──
    let n = placed.len();
    let mut raw_right = vec![0.0f32; n];
    let mut raw_void = vec![0.0f32; n];
    let mut raw_block = vec![0.0f32; n];
    let mut raw_fail = vec![0.0f32; n];

    // 3-a. RightBoundaryContribution — 이미 0~1이므로 별도 정규화가 필요 없다.
    for (i, (_, _, bb)) in placed.iter().enumerate() {
        raw_right[i] = right_boundary_term(bb.x_max, strip_length);
    }

    // 3-b. VoidAdjacency
    if let Some(void) = largest_void {
        for (i, (_, _, bb)) in placed.iter().enumerate() {
            raw_void[i] = void_adjacency_raw(*bb, void);
        }
    }

    // 3-c. BlockingDegree — 접촉 이웃 수 × 오목도 × 상대 면적
    let mut touch_counts = vec![0.0f32; n];
    let inflated: Vec<Rect> = placed.iter().map(|(_, _, bb)| inflate(*bb)).collect();
    for i in 0..n {
        let same_unit_i: Option<usize> = sep
            .has_place_groups()
            .then(|| sep.place_groups.representative(placed[i].1));
        let mut count = 0.0f32;
        for j in 0..n {
            if i == j {
                continue;
            }
            // 같은 배치 단위의 멤버는 서로를 막지 않는다 (강체로 함께 움직인다).
            match same_unit_i {
                Some(rep_i) if rep_i == sep.place_groups.representative(placed[j].1) => continue,
                _ => {}
            }
            if rects_overlap(inflated[i], placed[j].2) {
                count += 1.0;
            }
        }
        touch_counts[i] = count;
    }
    let max_touch = touch_counts.iter().copied().fold(0.0f32, f32::max);
    let max_area = layout
        .placed_items()
        .values()
        .map(|pi| pi.shape().area())
        .fold(0.0f32, f32::max)
        .max(EPS);

    // 오목도는 회전 불변이므로 item_id 단위로 캐시한다.
    let mut concavity_cache: HashMap<usize, f32> = HashMap::new();
    for (i, (pk, item_id, _)) in placed.iter().enumerate() {
        let shape = &layout.placed_items()[*pk].shape();
        let concavity = *concavity_cache
            .entry(*item_id)
            .or_insert_with(|| concavity_of(shape));
        let rel_area = (shape.area() / max_area).clamp(0.0, 1.0);
        let touch_norm = if max_touch > 0.0 {
            touch_counts[i] / max_touch
        } else {
            0.0
        };
        raw_block[i] = touch_norm * concavity * rel_area;
    }

    // 3-d. MoveFailureFrequency
    for (i, (_, item_id, _)) in placed.iter().enumerate() {
        raw_fail[i] = move_failures.get(item_id).copied().unwrap_or(0) as f32;
    }

    // ── 4. 최대값 기준 정규화 ──
    normalize_by_max(&mut raw_void);
    normalize_by_max(&mut raw_block);
    normalize_by_max(&mut raw_fail);

    // ── 5. 배치 단위(unit)로 축약 ──
    let mut targets: Vec<BottleneckTarget> = Vec::with_capacity(n);
    if sep.has_place_groups() {
        // 대표 item id → targets 내 인덱스
        let mut unit_slot: HashMap<usize, usize> = HashMap::new();
        for (i, (pk, item_id, _)) in placed.iter().enumerate() {
            let rep = sep.place_groups.representative(*item_id);
            let terms = EscapeScoreTerms {
                right_boundary_contribution: raw_right[i],
                void_adjacency: raw_void[i],
                blocking_degree: raw_block[i],
                move_failure_frequency: raw_fail[i],
            };
            match unit_slot.get(&rep).copied() {
                Some(slot) => {
                    let t = &mut targets[slot];
                    t.terms = merge_max(t.terms, terms);
                    t.score = t.terms.score();
                }
                None => {
                    unit_slot.insert(rep, targets.len());
                    targets.push(BottleneckTarget {
                        pk: sep.placement_key(rep).unwrap_or(*pk),
                        item_id: rep,
                        score: terms.score(),
                        terms,
                    });
                }
            }
        }
    } else {
        for (i, (pk, item_id, _)) in placed.iter().enumerate() {
            let terms = EscapeScoreTerms {
                right_boundary_contribution: raw_right[i],
                void_adjacency: raw_void[i],
                blocking_degree: raw_block[i],
                move_failure_frequency: raw_fail[i],
            };
            targets.push(BottleneckTarget {
                pk: *pk,
                item_id: *item_id,
                score: terms.score(),
                terms,
            });
        }
    }

    // ── 6. `점수가 높은 패턴부터` ──
    targets.sort_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.item_id.cmp(&b.item_id))
    });

    BottleneckAnalysis {
        targets,
        largest_void,
        strip_length,
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 항별 순수 계산
// ────────────────────────────────────────────────────────────────────────────

/// `x_max / strip_length`. 단, 사용 길이를 실제로 **결정하는** 아이템
/// (`x_max ≥ 0.99 × strip_length`)은 1.0으로 포화시킨다.
#[inline]
fn right_boundary_term(x_max: f32, strip_length: f32) -> f32 {
    if strip_length <= EPS {
        return 0.0;
    }
    let r = x_max / strip_length;
    if r >= RIGHT_BOUNDARY_SATURATION {
        1.0
    } else {
        r.clamp(0.0, 1.0)
    }
}

/// 아이템 bbox와 largest void bbox의 `경계 접촉 길이 + 근접도`를 0~1 원시값으로.
///
/// * `근접도` — 두 bbox의 축별 간격에서 얻은 유클리드 거리 `d`에 대해 `s / (s + d)`.
///   `s`는 공백 bbox의 반둘레이므로 공백 크기에 비례해 "가깝다"의 기준이 달라진다.
/// * `접촉 길이` — 두 bbox가 축별로 공유하는 투영 길이의 합을 공백 반둘레로 나눈 값.
fn void_adjacency_raw(item: Rect, void: Rect) -> f32 {
    let sep_x = (void.x_min - item.x_max)
        .max(item.x_min - void.x_max)
        .max(0.0);
    let sep_y = (void.y_min - item.y_max)
        .max(item.y_min - void.y_max)
        .max(0.0);
    let dist = (sep_x * sep_x + sep_y * sep_y).sqrt();

    let span_x = (item.x_max.min(void.x_max) - item.x_min.max(void.x_min)).max(0.0);
    let span_y = (item.y_max.min(void.y_max) - item.y_min.max(void.y_min)).max(0.0);

    let half_perimeter = (void.width() + void.height()).max(EPS);
    let contact = ((span_x + span_y) / half_perimeter).clamp(0.0, 1.0);
    let proximity = half_perimeter / (half_perimeter + dist);

    proximity * (0.5 + 0.5 * contact)
}

/// bbox를 대각선의 [`BLOCKING_INFLATE_RATIO`]만큼 사방으로 팽창시킨다.
#[inline]
fn inflate(bb: Rect) -> Rect {
    let w = bb.width();
    let h = bb.height();
    let pad = (w * w + h * h).sqrt() * BLOCKING_INFLATE_RATIO;
    Rect {
        x_min: bb.x_min - pad,
        y_min: bb.y_min - pad,
        x_max: bb.x_max + pad,
        y_max: bb.y_max + pad,
    }
}

#[inline]
fn rects_overlap(a: Rect, b: Rect) -> bool {
    !(a.x_max < b.x_min || b.x_max < a.x_min || a.y_max < b.y_min || b.y_max < a.y_min)
}

/// 오목도 = convex hull 면적 / 실제 면적. 볼록하면 1.0, 오목할수록 커진다.
fn concavity_of(shape: &SPolygon) -> f32 {
    let area = shape.area().max(EPS);
    let hull = convex_hull_from_points(shape.vertices().to_vec());
    let hull_area = ring_area(&hull);
    (hull_area / area).clamp(1.0, MAX_CONCAVITY)
}

/// 신발끈 공식으로 링의 면적을 구한다 (방향 무관, 항상 양수).
fn ring_area(pts: &[Point]) -> f32 {
    if pts.len() < 3 {
        return 0.0;
    }
    let mut acc = 0.0f32;
    let mut j = pts.len() - 1;
    for i in 0..pts.len() {
        acc += (pts[j].0 + pts[i].0) * (pts[j].1 - pts[i].1);
        j = i;
    }
    (acc * 0.5).abs()
}

/// 최대값으로 나눠 0.0~1.0 구간으로 옮긴다. 최대값이 0이면 전부 0으로 둔다.
fn normalize_by_max(values: &mut [f32]) {
    let max = values.iter().copied().fold(0.0f32, f32::max);
    if max <= EPS {
        for v in values.iter_mut() {
            *v = 0.0;
        }
        return;
    }
    for v in values.iter_mut() {
        *v = (*v / max).clamp(0.0, 1.0);
    }
}

/// 배치 단위 축약 — 멤버별 항의 최대값을 취한다.
#[inline]
fn merge_max(a: EscapeScoreTerms, b: EscapeScoreTerms) -> EscapeScoreTerms {
    EscapeScoreTerms {
        right_boundary_contribution: a
            .right_boundary_contribution
            .max(b.right_boundary_contribution),
        void_adjacency: a.void_adjacency.max(b.void_adjacency),
        blocking_degree: a.blocking_degree.max(b.blocking_degree),
        move_failure_frequency: a.move_failure_frequency.max(b.move_failure_frequency),
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 공백 구조 분석 — 균등 격자 래스터화 + 연결 성분
// ────────────────────────────────────────────────────────────────────────────

/// 레이아웃 bbox를 최대 `64 × 64` 균등 격자로 래스터화해 빈 셀의 연결 성분 중
/// 가장 큰 **내부** 공백의 bounding box를 돌려준다.
///
/// 격자 경계에 닿지 않는 성분(= 사방이 아이템으로 둘러싸인 진짜 내부 공백)을
/// 우선하고, 그런 성분이 하나도 없으면 전체 최대 성분으로 대체한다.
fn largest_internal_void(layout: &Layout) -> Option<Rect> {
    let mut region: Option<Rect> = None;
    for pi in layout.placed_items().values() {
        let bb = pi.shape().bbox();
        region = Some(match region {
            None => bb,
            Some(r) => Rect {
                x_min: r.x_min.min(bb.x_min),
                y_min: r.y_min.min(bb.y_min),
                x_max: r.x_max.max(bb.x_max),
                y_max: r.y_max.max(bb.y_max),
            },
        });
    }
    let region = region?;

    let w = region.width().max(EPS);
    let h = region.height().max(EPS);
    let (nx, ny) = grid_dims(w, h);
    let cw = w / nx as f32;
    let ch = h / ny as f32;

    // 셀 중심이 어떤 아이템 폴리곤 내부에 있으면 점유 셀로 본다.
    let mut occ = vec![false; nx * ny];
    for pi in layout.placed_items().values() {
        let bb = pi.shape().bbox();
        let ix0 = cell_index(bb.x_min - region.x_min, cw, nx);
        let ix1 = cell_index(bb.x_max - region.x_min, cw, nx);
        let iy0 = cell_index(bb.y_min - region.y_min, ch, ny);
        let iy1 = cell_index(bb.y_max - region.y_min, ch, ny);
        for iy in iy0..=iy1 {
            let cy = region.y_min + (iy as f32 + 0.5) * ch;
            for ix in ix0..=ix1 {
                let idx = iy * nx + ix;
                if occ[idx] {
                    continue;
                }
                let cx = region.x_min + (ix as f32 + 0.5) * cw;
                if point_in_polygon(&pi.shape(), Point(cx, cy)) {
                    occ[idx] = true;
                }
            }
        }
    }

    let (ix0, iy0, ix1, iy1) = largest_void_cells(&occ, nx, ny)?;
    Some(Rect {
        x_min: region.x_min + ix0 as f32 * cw,
        y_min: region.y_min + iy0 as f32 * ch,
        x_max: region.x_min + (ix1 + 1) as f32 * cw,
        y_max: region.y_min + (iy1 + 1) as f32 * ch,
    })
}

/// 종횡비를 유지하면서 긴 축을 [`MAX_VOID_GRID`]로 맞춘다.
fn grid_dims(w: f32, h: f32) -> (usize, usize) {
    let m = MAX_VOID_GRID;
    if w >= h {
        let ny = ((m as f32 * h / w).round() as usize).clamp(1, m);
        (m, ny)
    } else {
        let nx = ((m as f32 * w / h).round() as usize).clamp(1, m);
        (nx, m)
    }
}

/// 구간 오프셋을 셀 인덱스로. 범위를 벗어나면 잘라낸다.
#[inline]
fn cell_index(offset: f32, cell_size: f32, n: usize) -> usize {
    if cell_size <= EPS {
        return 0;
    }
    let raw = (offset / cell_size).floor();
    if raw <= 0.0 {
        0
    } else {
        (raw as usize).min(n - 1)
    }
}

/// 빈 셀의 4-연결 성분 중 최대 공백의 셀 인덱스 bbox `(ix0, iy0, ix1, iy1)`.
fn largest_void_cells(occ: &[bool], nx: usize, ny: usize) -> Option<(usize, usize, usize, usize)> {
    if nx == 0 || ny == 0 || occ.len() != nx * ny {
        return None;
    }
    let mut visited = vec![false; occ.len()];
    let mut best: Option<(usize, (usize, usize, usize, usize))> = None;
    let mut best_internal: Option<(usize, (usize, usize, usize, usize))> = None;
    let mut stack: Vec<usize> = Vec::new();

    for start in 0..occ.len() {
        if occ[start] || visited[start] {
            continue;
        }
        visited[start] = true;
        stack.push(start);

        let (mut ix0, mut iy0) = (nx - 1, ny - 1);
        let (mut ix1, mut iy1) = (0usize, 0usize);
        let mut count = 0usize;
        let mut touches_border = false;

        while let Some(idx) = stack.pop() {
            let ix = idx % nx;
            let iy = idx / nx;
            count += 1;
            ix0 = ix0.min(ix);
            ix1 = ix1.max(ix);
            iy0 = iy0.min(iy);
            iy1 = iy1.max(iy);
            if ix == 0 || iy == 0 || ix + 1 == nx || iy + 1 == ny {
                touches_border = true;
            }

            if ix > 0 && !occ[idx - 1] && !visited[idx - 1] {
                visited[idx - 1] = true;
                stack.push(idx - 1);
            }
            if ix + 1 < nx && !occ[idx + 1] && !visited[idx + 1] {
                visited[idx + 1] = true;
                stack.push(idx + 1);
            }
            if iy > 0 && !occ[idx - nx] && !visited[idx - nx] {
                visited[idx - nx] = true;
                stack.push(idx - nx);
            }
            if iy + 1 < ny && !occ[idx + nx] && !visited[idx + nx] {
                visited[idx + nx] = true;
                stack.push(idx + nx);
            }
        }

        let bbox = (ix0, iy0, ix1, iy1);
        if best.is_none_or(|(c, _)| count > c) {
            best = Some((count, bbox));
        }
        if !touches_border && best_internal.is_none_or(|(c, _)| count > c) {
            best_internal = Some((count, bbox));
        }
    }

    best_internal.or(best).map(|(_, bbox)| bbox)
}

/// 광선 투사(ray casting) 내부 판정.
fn point_in_polygon(poly: &SPolygon, p: Point) -> bool {
    let v = &poly.vertices();
    let n = v.len();
    if n < 3 {
        return false;
    }
    let (px, py) = (p.0, p.1);
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = (v[i].0, v[i].1);
        let (xj, yj) = (v[j].0, v[j].1);
        if (yi > py) != (yj > py) {
            let t = (py - yi) / (yj - yi);
            if px < xi + t * (xj - xi) {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

// ────────────────────────────────────────────────────────────────────────────
// 단위 테스트 — 외부 데이터 없이 수식/임계값만 검증한다.
// ────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(x_min: f32, y_min: f32, x_max: f32, y_max: f32) -> Rect {
        Rect {
            x_min,
            y_min,
            x_max,
            y_max,
        }
    }

    #[test]
    fn score_matches_document_weights() {
        // 문서 원문: 0.35 / 0.30 / 0.20 / 0.15 (합 = 1.00)
        let all_one = EscapeScoreTerms {
            right_boundary_contribution: 1.0,
            void_adjacency: 1.0,
            blocking_degree: 1.0,
            move_failure_frequency: 1.0,
        };
        assert!((all_one.score() - 1.0).abs() < 1e-6);

        let only_right = EscapeScoreTerms {
            right_boundary_contribution: 1.0,
            ..Default::default()
        };
        assert!((only_right.score() - 0.35).abs() < 1e-6);

        let mixed = EscapeScoreTerms {
            right_boundary_contribution: 0.5,
            void_adjacency: 1.0,
            blocking_degree: 0.0,
            move_failure_frequency: 0.2,
        };
        let expected = 0.35 * 0.5 + 0.30 * 1.0 + 0.20 * 0.0 + 0.15 * 0.2;
        assert!((mixed.score() - expected).abs() < 1e-6);
    }

    #[test]
    fn right_boundary_saturates_for_the_deciding_item() {
        // 사용 길이를 실제로 결정하는 아이템 (99% 이상) → 1.0
        assert_eq!(right_boundary_term(100.0, 100.0), 1.0);
        assert_eq!(right_boundary_term(99.5, 100.0), 1.0);
        // 그 아래는 비율 그대로
        assert!((right_boundary_term(50.0, 100.0) - 0.5).abs() < 1e-6);
        assert_eq!(right_boundary_term(10.0, 0.0), 0.0);
    }

    #[test]
    fn void_adjacency_prefers_touching_over_distant() {
        let void = rect(10.0, 0.0, 20.0, 10.0);
        // 왼쪽 변 전체가 공백과 맞닿은 아이템
        let touching = rect(0.0, 0.0, 10.0, 10.0);
        // 멀리 떨어진 아이템
        let far = rect(100.0, 0.0, 110.0, 10.0);
        let t = void_adjacency_raw(touching, void);
        let f = void_adjacency_raw(far, void);
        assert!(t > f, "touching {t} should beat far {f}");
        // 접촉 시 거리 = 0 → proximity = 1
        assert!(t > 0.5 && t <= 1.0);
        assert!(f < 0.3);
    }

    #[test]
    fn void_adjacency_is_bounded() {
        let void = rect(0.0, 0.0, 10.0, 10.0);
        let inside = rect(1.0, 1.0, 9.0, 9.0);
        let v = void_adjacency_raw(inside, void);
        assert!((0.0..=1.0).contains(&v), "out of range: {v}");
    }

    #[test]
    fn normalize_scales_to_unit_max() {
        let mut v = vec![0.0, 2.0, 4.0, 8.0];
        normalize_by_max(&mut v);
        assert_eq!(v, vec![0.0, 0.25, 0.5, 1.0]);

        let mut zeros = vec![0.0, 0.0];
        normalize_by_max(&mut zeros);
        assert_eq!(zeros, vec![0.0, 0.0]);
    }

    #[test]
    fn inflation_is_two_percent_of_the_diagonal() {
        let bb = rect(0.0, 0.0, 3.0, 4.0); // 대각선 5.0 → pad 0.1
        let inf = inflate(bb);
        assert!((inf.x_min - (-0.1)).abs() < 1e-5);
        assert!((inf.x_max - 3.1).abs() < 1e-5);
        assert!((inf.y_max - 4.1).abs() < 1e-5);
    }

    #[test]
    fn inflated_bbox_catches_near_touching_neighbour() {
        let a = rect(0.0, 0.0, 10.0, 10.0); // 대각선 ≈ 14.14 → pad ≈ 0.283
        let near = rect(10.2, 0.0, 20.0, 10.0);
        let far = rect(11.0, 0.0, 20.0, 10.0);
        assert!(rects_overlap(inflate(a), near));
        assert!(!rects_overlap(inflate(a), far));
    }

    #[test]
    fn ring_area_is_orientation_independent() {
        let ccw = vec![
            Point(0.0, 0.0),
            Point(4.0, 0.0),
            Point(4.0, 3.0),
            Point(0.0, 3.0),
        ];
        let mut cw = ccw.clone();
        cw.reverse();
        assert!((ring_area(&ccw) - 12.0).abs() < 1e-4);
        assert!((ring_area(&cw) - 12.0).abs() < 1e-4);
        assert_eq!(ring_area(&ccw[..2]), 0.0);
    }

    #[test]
    fn concavity_of_l_shape_exceeds_one() {
        // L 형상: 4x4 정사각형에서 우상단 2x2를 뺀 면적 12
        let l_shape = SPolygon::new(vec![
            Point(0.0, 0.0),
            Point(4.0, 0.0),
            Point(4.0, 2.0),
            Point(2.0, 2.0),
            Point(2.0, 4.0),
            Point(0.0, 4.0),
        ])
        .expect(crate::dmsg!("valid simple polygon"));
        assert!((l_shape.area() - 12.0).abs() < 1e-3);
        let c = concavity_of(&l_shape);
        // hull 면적 14 / 실제 면적 12 ≈ 1.1667
        assert!(c > 1.1 && c < 1.25, "concavity was {c}");

        let square = SPolygon::new(vec![
            Point(0.0, 0.0),
            Point(4.0, 0.0),
            Point(4.0, 4.0),
            Point(0.0, 4.0),
        ])
        .expect(crate::dmsg!("valid simple polygon"));
        assert!((concavity_of(&square) - 1.0).abs() < 1e-3);
    }

    #[test]
    fn point_in_polygon_handles_square() {
        let square = SPolygon::new(vec![
            Point(0.0, 0.0),
            Point(4.0, 0.0),
            Point(4.0, 4.0),
            Point(0.0, 4.0),
        ])
        .expect(crate::dmsg!("valid simple polygon"));
        assert!(point_in_polygon(&square, Point(2.0, 2.0)));
        assert!(!point_in_polygon(&square, Point(5.0, 2.0)));
        assert!(!point_in_polygon(&square, Point(2.0, -1.0)));
    }

    #[test]
    fn grid_dims_respect_the_64_cap() {
        assert_eq!(grid_dims(100.0, 100.0), (MAX_VOID_GRID, MAX_VOID_GRID));
        let (nx, ny) = grid_dims(200.0, 50.0);
        assert_eq!(nx, MAX_VOID_GRID);
        assert_eq!(ny, 16);
        let (nx, ny) = grid_dims(1.0, 1000.0);
        assert_eq!(ny, MAX_VOID_GRID);
        assert_eq!(nx, 1);
    }

    #[test]
    fn largest_void_prefers_the_internal_component() {
        // 5x5 격자. 중앙 (2,2) 한 칸만 내부 공백이고, 왼쪽 열 전체는 경계에 닿는다.
        let (nx, ny) = (5usize, 5usize);
        let mut occ = vec![true; nx * ny];
        // 왼쪽 두 열은 비어 있음(경계 접촉, 셀 10개)
        for iy in 0..ny {
            occ[iy * nx] = false;
            occ[iy * nx + 1] = false;
        }
        // 중앙 한 칸은 내부 공백(셀 1개)
        occ[2 * nx + 3] = true; // 확실히 막아둔다
        occ[2 * nx + 2] = false;
        // (2,2)가 (2,1)과 이어지지 않도록 (1,2) 열을 막는다
        occ[2 * nx + 1] = true;

        let bbox = largest_void_cells(&occ, nx, ny).expect(crate::dmsg!("some void"));
        // 셀 수는 더 적지만 내부 성분이 우선한다
        assert_eq!(bbox, (2, 2, 2, 2));
    }

    #[test]
    fn largest_void_falls_back_to_border_component() {
        let (nx, ny) = (4usize, 4usize);
        let mut occ = vec![true; nx * ny];
        occ[0] = false; // 모서리 한 칸만 비어 있다 → 내부 성분 없음
        let bbox = largest_void_cells(&occ, nx, ny).expect(crate::dmsg!("some void"));
        assert_eq!(bbox, (0, 0, 0, 0));

        let full = vec![true; nx * ny];
        assert_eq!(largest_void_cells(&full, nx, ny), None);
    }

    #[test]
    fn top_ratio_returns_at_least_one() {
        let terms = EscapeScoreTerms::default();
        let analysis = BottleneckAnalysis {
            targets: Vec::new(),
            largest_void: None,
            strip_length: 0.0,
        };
        assert!(analysis.is_empty());
        assert_eq!(analysis.top_ratio(0.5).len(), 0);
        assert_eq!(analysis.top(3).len(), 0);

        // 10개 중 5% → ceil(0.5) = 1개, 50% → ceil(5.0) = 5개
        let mut sm = slotmap::SlotMap::<PItemKey, ()>::with_key();
        let keys: Vec<PItemKey> = (0..10).map(|_| sm.insert(())).collect();
        let targets = keys
            .iter()
            .enumerate()
            .map(|(i, pk)| BottleneckTarget {
                pk: *pk,
                item_id: i,
                score: 1.0 - i as f32 * 0.01,
                terms,
            })
            .collect();
        let analysis = BottleneckAnalysis {
            targets,
            largest_void: None,
            strip_length: 100.0,
        };
        assert_eq!(analysis.top_ratio(0.05).len(), 1);
        assert_eq!(analysis.top_ratio(0.50).len(), 5);
        assert_eq!(analysis.top_ratio(0.0).len(), 1);
        assert_eq!(analysis.top_ratio(1.0).len(), 10);
        assert_eq!(analysis.top(100).len(), 10);
    }

    #[test]
    fn merge_max_keeps_the_worst_member_per_term() {
        let a = EscapeScoreTerms {
            right_boundary_contribution: 1.0,
            void_adjacency: 0.1,
            blocking_degree: 0.4,
            move_failure_frequency: 0.0,
        };
        let b = EscapeScoreTerms {
            right_boundary_contribution: 0.2,
            void_adjacency: 0.9,
            blocking_degree: 0.3,
            move_failure_frequency: 0.7,
        };
        let m = merge_max(a, b);
        assert_eq!(m.right_boundary_contribution, 1.0);
        assert_eq!(m.void_adjacency, 0.9);
        assert_eq!(m.blocking_degree, 0.4);
        assert_eq!(m.move_failure_frequency, 0.7);
    }
}
