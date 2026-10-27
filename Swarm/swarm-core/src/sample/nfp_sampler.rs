//! ① separator 샘플러를 **정확한 NFP 후보**로 교체한다
//! (`NfpIntegrationConfig::nfp_sampler`로 켠다).
//!
//! # 왜 필요한가
//!
//! [`crate::sample::search::search_placement`]는 컨테이너에 균등 난수를
//! `n_container_samples`개(기본 1000) 뿌리고 CDE로 거른다. 그 한계는
//! [`crate::sample::raster_nfp`] 주석이 이미 적어 두었다.
//!
//! > 배치가 진행돼 빈 공간이 좁아질수록 유효한 자리가 차지하는 면적 비율이 급감하므로,
//! > 난수 샘플링은 좁은 포켓을 확률적으로 통째로 놓친다.
//!
//! 래스터 NFP는 그 답을 **격자 근사**로 냈다. 여기서는 같은 질문에 정확히 답한다.
//! 후보를 뿌리는 게 아니라, `NFP(placed, item)` 경계에서 **직접 뽑는다**. 좁은 포켓의
//! 유일한 자리도 NFP 정점이나 IFP 교점으로 반드시 열거된다 — 면적이 0이어도 상관없다.
//!
//! # 후보 출처
//!
//! ```text
//! 1) NFP vertex            — 기존 조각과 접촉하는 자세
//! 2) IFP vertex            — strip 경계(좌하단 등)에 접촉하는 자세
//! 3) IFP edge ∩ NFP edge   — 조각과 strip 경계에 동시에 접촉하는 자세
//! ```
//!
//! [`crate::nfp_lbf::placement`]는 여기에 `NFP edge ∩ NFP edge`까지 넣는다. 그건
//! `O(n²·e²)`라 한 조각을 배치할 때 수천 번의 교차 판정이 필요하고, 문서가 정한
//! 후보 예산을 통째로 먹는다. separator는 **한 스텝에 조각 하나**를 옮기는 게 아니라
//! 레이아웃의 모든 겹친 조각을 차례로 옮기므로, 조각당 비용이 그대로 스텝 비용이 된다.
//! 그래서 여기서는 `O(n·e)`인 세 출처만 쓰고, 남는 예산을 실제 `evaluator` 평가에
//! 돌린다. 접촉 자세를 놓치는 경우는 "두 조각에 동시에 끼는 자리"뿐이고 그 자리는
//! 뒤이은 좌표 하강이 메운다.
//!
//! # 판정 주체
//!
//! NFP는 **제안**만 한다. 최종 판정은 언제나 호출자가 넘긴 `evaluator`(= jagua-rs CDE)다.
//! 그래서 이 파일의 기하가 낙관적이면 후보 하나를 낭비할 뿐이고, 보수적이면 그 자리를
//! 놓칠 뿐 잘못된 배치는 나올 수 없다.
//!
//! # 겹친 상태에서의 계약
//!
//! separator는 **이미 겹친 상태**에서 돈다. NFP 후보는 정의상 "겹치지 않는 위치"만
//! 주므로, 빽빽한 레이아웃에서는 후보가 0개일 수 있다. 그때는 `None`을 돌려준다.
//! **이 파일은 절대 스스로 폴백하지 않는다** — 기존 난수 샘플러로 되돌릴지는 호출자가
//! 정한다. 그래야 A/B 측정에서 "NFP가 답을 못 낸 횟수"가 그대로 보인다.

use std::collections::HashSet;
use std::sync::Arc;

use jagua_rs::entities::{Item, Layout, PItemKey};
use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::geo_enums::RotationRange;
use rand::Rng;
use swarm_nfp::{
    GRID_SCALE, Location, NfpEngine, Point as NfpPoint, PolygonSet, PreparedRegion, Ring as NfpRing,
};

use crate::eval::sample_eval::{SampleEval, SampleEvaluator};
use crate::nfp_lbf::COORD_TIE_EPS;
use crate::nfp_lbf::cache::{GeometryHash, MaskCell, NfpKey, NfpLbfCache, NfpMask};
use crate::sample::search::SampleConfig;

// ────────────────────────────────────────────────────────────────────────────
// 예산 상수
// ────────────────────────────────────────────────────────────────────────────

/// feasible 판정(= 마스크 조회 + `locate_prepared`) 예산을 평가 예산의 몇 배까지 줄지.
///
/// 판정 한 번은 CDE 평가 한 번보다 한참 싸다(마스크가 대부분을 `locate` 없이 끝낸다).
/// 그래도 무제한이면 빽빽한 레이아웃에서 조각 하나에 수만 번이 나가므로 상한을 둔다.
/// 4배면 유효 후보 비율이 25%까지 떨어져도 평가 예산을 다 채울 수 있다.
const PROBE_BUDGET_SCALE: usize = 4;

/// 한 회전당 모을 raw 후보 점 상한.
///
/// 정점 후보는 `Σ(obstacle 정점 수)`라 156-패널이면 수천 개가 정상이다. 이 상한은
/// 병리적인 입력(정점 수만 개짜리 형상)에서 메모리와 정렬 비용이 터지는 것만 막는다.
const MAX_RAW_CANDIDATES: usize = 16_384;

/// 선언 회전을 정수 도(degree)로 반올림할 때 허용하는 오차(도).
///
/// NFP 라이브러리는 회전을 정수 도로만 들고 있다. 실제 자세가 그 정수 도에서 이만큼
/// 넘게 벗어나 있으면 **다른 형상의 NFP**를 쓰는 셈이므로 후보를 만들지 않는다.
const ROTATION_DEG_TOLERANCE: f32 = 1.0e-3;

// ────────────────────────────────────────────────────────────────────────────
// 공개 API
// ────────────────────────────────────────────────────────────────────────────

/// [`crate::sample::search::search_placement`]의 **NFP 후보 기반 대체 함수**.
///
/// 반환 형태는 원본과 같다: `(최선 배치와 그 평가, 총 평가 횟수)`.
///
/// * 난수 대신 `NFP(placed, item)` 경계에서 후보를 뽑아 `evaluator`에 넣는다.
/// * 평가 횟수 상한은 `sample_config.n_container_samples`다 — 기존 경로와 비용을 맞춘다.
/// * 후보가 하나도 없으면 `(None, n_evals)`를 돌려준다. 호출자가 기존 샘플러로
///   폴백해야 한다 (모듈 주석의 "겹친 상태에서의 계약" 참조).
///
/// `ref_pk`가 가리키는 조각은 **지금 옮기는 대상**이므로 장애물에서 제외한다.
/// (그러지 않으면 자기 자신과의 NFP가 모든 자리를 막아 후보가 0개가 된다.)
///
/// `cache`는 호출자가 소유한 [`NfpLbfCache`]다. pair NFP·[`PreparedRegion`]·마스크가
/// 전부 그 안에서 `Arc`로 공유되므로, 이 함수는 아이템마다 기하를 새로 만들지 않는다.
pub fn search_placement_nfp(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    evaluator: impl SampleEvaluator,
    sample_config: SampleConfig,
    cache: &mut NfpLbfCache,
    rng: &mut impl Rng,
) -> (Option<(DTransformation, SampleEval)>, usize) {
    search_placement_nfp_with_control(
        l,
        item,
        ref_pk,
        evaluator,
        sample_config,
        cache,
        rng,
        &mut || false,
    )
}

pub fn search_placement_nfp_with_control(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    mut evaluator: impl SampleEvaluator,
    sample_config: SampleConfig,
    cache: &mut NfpLbfCache,
    rng: &mut impl Rng,
    should_stop: &mut dyn FnMut() -> bool,
) -> (Option<(DTransformation, SampleEval)>, usize) {
    // NFP 후보 열거는 완전히 결정적이다. 난수는 쓰지 않지만, 호출부가 기존 샘플러와
    // 이 함수를 한 자리에서 교체할 수 있도록 시그니처는 그대로 맞춰 둔다.
    let _ = rng;

    let eval_budget = sample_config.n_container_samples.max(1);
    let probe_budget = eval_budget.saturating_mul(PROBE_BUDGET_SCALE);

    let candidates = collect_candidates(l, item, ref_pk, cache, probe_budget, should_stop);
    if candidates.is_empty() {
        return (None, evaluator.n_evals());
    }

    // 평가 규약은 기존 경로([`crate::sample::best_samples::BestSamples`], size 1)와 같다.
    //   * 상한을 넘긴(= `Invalid`) 표본은 채택하지 않는다.
    //   * 동률이면 **먼저 본 후보**를 유지한다 — 후보가 이미 우선순위대로 정렬돼 있으므로
    //     이게 곧 "가장 덜 움직이는 자리"(separator) 또는 "가장 좌하단"(그 외)이다.
    //   * 참조 조각이 있으면 **지금 자세**를 먼저 평가해 기준으로 삼는다 (기존 샘플러와
    //     같다). 그래야 후보가 전부 지금보다 나쁠 때 더 나쁜 자리로 옮기지 않는다.
    let mut best: Option<(DTransformation, SampleEval)> = ref_pk
        .and_then(|pk| l.placed_items().get(pk))
        .map(|pi| (pi.d_transf(), evaluator.evaluate_sample(pi.d_transf(), None)))
        .filter(|(_, eval)| *eval != SampleEval::Invalid);
    for cand in candidates.iter().take(eval_budget) {
        if should_stop() {
            break;
        }
        let bound = best.map_or(SampleEval::Invalid, |(_, eval)| eval);
        let eval = evaluator.evaluate_sample(cand.dt, Some(bound));
        if eval < bound {
            best = Some((cand.dt, eval));
        }
    }

    (best, evaluator.n_evals())
}

/// Polish 의 NFP 왼쪽 재배치([`crate::optimizer::polish::nfp_left`])가 살펴보는 창.
#[derive(Clone, Copy, Debug)]
pub(crate) struct LeftwardWindow {
    /// 왼쪽으로 찾아볼 최대 거리(mm).
    pub reach: f64,
    /// 현재 높이에서 위·아래로 살펴볼 거리(mm).
    pub y_band: f64,
    /// 현재 높이 위·아래 각각에 둘 수평선 수.
    pub n_lines_each_side: usize,
    /// 이보다 덜 왼쪽인 자리는 내지 않는다(mm).
    pub min_gain: f64,
}

/// 한 수평선에서 남길 후보 수. 맨 왼쪽 자리가 검증에서 떨어질 때를 대비한다.
const LEFTWARD_CANDIDATES_PER_LINE: usize = 2;

/// NFP 경계(= 접촉 자세)에서 빈 구간 쪽으로 이만큼 들어간 점을 후보로 쓴다(mm).
const LEFTWARD_CONTACT_NUDGE: f64 = 0.002;

/// 배치된 조각 `pk` 를 지금보다 왼쪽으로 옮길 수 있는 자리를 왼쪽 것부터 돌려준다.
///
/// 현재 높이와 그 위·아래 수평선마다 IFP 구간에서 `NFP(장애물, 조각)` 내부 구간을 빼고,
/// 남은 구간의 왼쪽 끝(벽 또는 장애물 NFP 경계 = 접촉 자세)을 후보로 낸다. 그래서 똑바로
/// 밀면 막히는 조각도, 막힌 구간 너머의 빈자리로 건너뛸 수 있다. 창 밖 조각은 창 안의
/// 어떤 후보와도 닿을 수 없으므로 장애물에서 뺀다.
///
/// 후보는 제안일 뿐이다. 판정은 호출자가 CDE 와 원본 외곽 계약으로 한다.
pub(crate) fn leftward_line_candidates(
    l: &Layout,
    item: &Item,
    pk: PItemKey,
    cache: &mut NfpLbfCache,
    window: LeftwardWindow,
    should_stop: &mut dyn FnMut() -> bool,
) -> Vec<DTransformation> {
    let Some(pi) = l.placed_items().get(pk) else {
        return Vec::new();
    };
    let cont = l.container().outer_cd().bbox();
    let (ox, oy) = (cont.x_min as f64, cont.y_min as f64);
    let (cw, ch) = (cont.width(), cont.height());
    if !cw.is_finite() || !ch.is_finite() || cw <= 0.0 || ch <= 0.0 {
        return Vec::new();
    }
    let Some(hash_item) = cache.library().hash_of_item(item.idx()) else {
        return Vec::new();
    };
    let rotation = pi.d_transf().rotation();
    let Some(deg) = normalized_degrees(rotation) else {
        return Vec::new();
    };
    if cache.library().shape(hash_item, deg).is_none() {
        return Vec::new();
    }
    let (tx0, ty0) = {
        let (x, y) = pi.d_transf().translation();
        (x as f64 - ox, y as f64 - oy)
    };
    let (x_lo, x_hi) = (tx0 - window.reach, tx0 - window.min_gain);
    if x_hi <= x_lo {
        return Vec::new();
    }
    // 창 안의 어느 후보에서든 조각이 덮을 수 있는 월드 영역 (1mm 여유).
    let b = pi.shape().bbox();
    let sweep = Aabb {
        min_x: b.x_min as f64 - window.reach - 1.0,
        min_y: b.y_min as f64 - window.y_band - 1.0,
        max_x: b.x_max as f64 + 1.0,
        max_y: b.y_max as f64 + window.y_band + 1.0,
    };

    let Some((ifp, _, _)) = cache.inner_fit_view_prepared(hash_item, deg, cw, ch) else {
        return Vec::new();
    };
    let ifp_edges = edges_of(&ifp, 0.0, 0.0);

    // 장애물마다 NFP edge 를 컨테이너 로컬로 옮겨 두고, 수평선이 닿는 y 범위를 기억한다.
    let mut obstacles: Vec<(Vec<[f64; 4]>, f64, f64)> = Vec::new();
    for (other_pk, other) in l.placed_items().iter() {
        if should_stop() {
            return Vec::new();
        }
        let ob = other.shape().bbox();
        if other_pk == pk
            || (ob.x_max as f64) < sweep.min_x
            || (ob.x_min as f64) > sweep.max_x
            || (ob.y_max as f64) < sweep.min_y
            || (ob.y_min as f64) > sweep.max_y
        {
            continue;
        }
        // 창 안의 장애물 하나라도 모델에 못 넣으면 후보가 틀린 말을 하게 된다. 내지 않는다.
        let Some(hash) = cache.library().hash_of_item(other.item().idx()) else {
            return Vec::new();
        };
        let Some(other_deg) = normalized_degrees(other.d_transf().rotation()) else {
            return Vec::new();
        };
        let Some((region, _, _)) =
            cache.view_prepared_or_compute(NfpKey::new(hash, other_deg, hash_item, deg))
        else {
            return Vec::new();
        };
        let (otx, oty) = other.d_transf().translation();
        let edges = edges_of(&region, otx as f64 - ox, oty as f64 - oy);
        let (lo, hi) = edges.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), e| {
            (lo.min(e[1].min(e[3])), hi.max(e[1].max(e[3])))
        });
        obstacles.push((edges, lo, hi));
    }

    let eps = COORD_TIE_EPS as f64;
    let n = window.n_lines_each_side;
    let step = if n > 0 { window.y_band / n as f64 } else { 0.0 };
    let mut out: Vec<(f64, f64)> = Vec::new();
    let mut xs: Vec<f64> = Vec::new();
    let mut allowed: Vec<(f64, f64)> = Vec::new();
    let mut forbidden: Vec<(f64, f64)> = Vec::new();
    let mut line: Vec<f64> = Vec::new();
    for k in 0..=2 * n {
        if should_stop() {
            break;
        }
        // 0, +1, -1, +2, -2, … 순서: 덜 움직이는 높이를 먼저 본다.
        let j = (k + 1) / 2;
        let sign = if k % 2 == 1 { 1.0 } else { -1.0 };
        let y = ty0 + sign * j as f64 * step;

        horizontal_crossings(&ifp_edges, y, &mut xs);
        allowed.clear();
        allowed.extend(xs.chunks_exact(2).map(|c| (c[0], c[1])));
        if allowed.is_empty() {
            continue;
        }
        forbidden.clear();
        for (edges, lo, hi) in &obstacles {
            if y < *lo || y > *hi {
                continue;
            }
            horizontal_crossings(edges, y, &mut xs);
            forbidden.extend(xs.chunks_exact(2).map(|c| (c[0], c[1])));
        }

        // 빈 구간의 왼쪽 끝 후보: 창의 왼쪽 끝, 벽, 그리고 각 금지 구간의 오른쪽 끝.
        line.clear();
        let starts = std::iter::once(x_lo)
            .chain(allowed.iter().map(|&(a, _)| a))
            .chain(forbidden.iter().map(|&(_, b)| b + LEFTWARD_CONTACT_NUDGE));
        for x in starts {
            if x < x_lo - eps || x > x_hi {
                continue;
            }
            if !allowed.iter().any(|&(a, b)| x >= a - eps && x <= b + eps) {
                continue;
            }
            if forbidden.iter().any(|&(a, b)| x > a + eps && x < b - eps) {
                continue;
            }
            line.push(x);
        }
        line.sort_by(f64::total_cmp);
        line.dedup_by(|a, b| (*a - *b).abs() <= eps);
        out.extend(line.iter().take(LEFTWARD_CANDIDATES_PER_LINE).map(|&x| (x, y)));
    }

    out.sort_by(|a, b| {
        a.0.total_cmp(&b.0)
            .then_with(|| (a.1 - ty0).abs().total_cmp(&(b.1 - ty0).abs()))
    });
    out.into_iter()
        .map(|(x, y)| DTransformation::new(rotation, ((x + ox) as f32, (y + oy) as f32)))
        .collect()
}

/// 닫힌 다각형 edge 들과 수평선 `y` 의 교차 x 를 정렬해 `out` 에 담는다.
///
/// 반열린 규칙(아래 끝 포함, 위 끝 제외)이라 정점을 지나는 선도 두 번 세지 않는다.
/// 짝을 지은 `[x0, x1], [x2, x3], …` 이 짝홀 규칙의 내부 구간이다.
fn horizontal_crossings(edges: &[[f64; 4]], y: f64, out: &mut Vec<f64>) {
    out.clear();
    for e in edges {
        let (x1, y1, x2, y2) = (e[0], e[1], e[2], e[3]);
        if (y1 <= y && y < y2) || (y2 <= y && y < y1) {
            out.push(x1 + (y - y1) * (x2 - x1) / (y2 - y1));
        }
    }
    out.sort_by(f64::total_cmp);
    if out.len() % 2 == 1 {
        out.pop();
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 내부 타입
// ────────────────────────────────────────────────────────────────────────────

/// 평가 대기 중인 후보 하나.
#[derive(Clone, Copy, Debug)]
struct Candidate {
    dt: DTransformation,
    /// 사전식 정렬 키. 작을수록 먼저 평가한다.
    key: (f64, f64),
}

/// 축 정렬 경계 상자(mm, `f64`).
#[derive(Clone, Copy, Debug, PartialEq)]
struct Aabb {
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
}

impl Aabb {
    fn translated(self, dx: f64, dy: f64) -> Self {
        Self {
            min_x: self.min_x + dx,
            min_y: self.min_y + dy,
            max_x: self.max_x + dx,
            max_y: self.max_y + dy,
        }
    }

    fn contains(self, x: f64, y: f64, eps: f64) -> bool {
        x >= self.min_x - eps
            && x <= self.max_x + eps
            && y >= self.min_y - eps
            && y <= self.max_y + eps
    }

    fn overlaps(self, other: Aabb, eps: f64) -> bool {
        self.min_x <= other.max_x + eps
            && other.min_x <= self.max_x + eps
            && self.min_y <= other.max_y + eps
            && other.min_y <= self.max_y + eps
    }
}

/// 레이아웃에 이미 놓여 있는 조각 하나를 NFP 질의 좌표로 옮긴 서술.
///
/// 실제 NFP(그리고 마스크·정규화 영역)는 회전마다 다르므로 여기 담지 않는다.
/// 아이템 회전을 하나 고른 뒤 [`Obstacle`]로 승격시킨다.
#[derive(Clone, Copy, Debug)]
struct ObstacleRef {
    hash: GeometryHash,
    rotation: i64,
    /// 컨테이너 로컬 프레임의 평행이동(mm).
    tx: f64,
    ty: f64,
}

/// 특정 아이템 회전에 대해 준비가 끝난 금지 영역 하나.
///
/// NFP는 **복사하지 않는다.** 캐시가 소유한 로컬 좌표 `Arc`를 그대로 들고, 질의할 때
/// 점에서 평행이동을 뺀다 ([`crate::nfp_lbf::placement`]의 `Obstacle`과 같은 규약).
struct Obstacle {
    region: Arc<PolygonSet>,
    mask: Arc<NfpMask>,
    prepared: Option<Arc<PreparedRegion>>,
    tx: f64,
    ty: f64,
    /// 평행이동의 0.001mm 격자 정수값.
    tx_q: i64,
    ty_q: i64,
    /// 컨테이너 로컬 프레임의 월드 bbox.
    bbox: Aabb,
}

// ────────────────────────────────────────────────────────────────────────────
// 후보 생성
// ────────────────────────────────────────────────────────────────────────────

/// 레이아웃 전체에 대해 feasible한 NFP 후보를 우선순위대로 모은다.
///
/// 빈 벡터는 "NFP가 답을 못 냈다"는 뜻이고, 그 사유는 셋 중 하나다.
///
/// 1. 라이브러리에 없는 형상/회전이 레이아웃에 들어 있다 (모델이 불완전하다).
/// 2. 아이템이 컨테이너에 아예 들어가지 않는다 (IFP가 비었다).
/// 3. 겹치지 않는 자리가 실제로 없다 (separator가 겹친 상태에서 부른 경우).
///
/// 셋 다 호출자의 폴백으로 처리해야 하므로 구분해서 돌려주지 않는다 (로그로만 남긴다).
fn collect_candidates(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    cache: &mut NfpLbfCache,
    probe_budget: usize,
    should_stop: &mut dyn FnMut() -> bool,
) -> Vec<Candidate> {
    let empty = Vec::new();

    // ── 컨테이너 로컬 프레임 ──────────────────────────────────────────────
    //
    // `NfpLbfCache::inner_fit`은 컨테이너를 항상 `(0,0)-(w,h)`로 만든다(그래야 IFP가
    // 폭·높이만으로 캐싱된다). 그래서 좌표를 컨테이너 좌하단 기준으로 내렸다가
    // 마지막에 다시 올린다. strip은 좌하단이 원점이라 실제로는 항등이지만, 그렇지
    // 않은 컨테이너에서도 기하가 어긋나지 않게 한다.
    //
    // bbox는 `outer_cd`(CDE가 실제로 검사하는 형상)에서 딴다. 원본보다 조금 줄어든
    // 형상이므로 IFP가 그만큼 보수적이 되고, 그 방향의 오차는 안전하다.
    let cont = l.container().outer_cd().bbox();
    let (ox, oy) = (cont.x_min as f64, cont.y_min as f64);
    let (cw, ch) = (cont.width(), cont.height());
    if !cw.is_finite() || !ch.is_finite() || cw <= 0.0 || ch <= 0.0 {
        return empty;
    }

    // ── 아이템 형상과 회전 ────────────────────────────────────────────────
    let Some(hash_item) = cache.library().hash_of_item(item.idx()) else {
        return empty;
    };
    let rotations = library_rotations_for(item, cache.library().rotations());
    let rotations: Vec<(i64, f32)> = rotations
        .into_iter()
        .filter(|(deg, _)| cache.library().shape(hash_item, *deg).is_some())
        .collect();
    if rotations.is_empty() {
        return empty;
    }

    // ── 장애물 서술 ──────────────────────────────────────────────────────
    //
    // 하나라도 라이브러리로 옮길 수 없으면 **후보를 만들지 않는다**. 그 조각을 빼고
    // 계산한 NFP는 "겹치지 않는 위치"를 잘못 말하게 되고, 그러면 예산만 태우고
    // evaluator에서 전부 기각된다. 차라리 즉시 폴백하는 편이 싸다.
    let mut refs: Vec<ObstacleRef> = Vec::with_capacity(l.placed_items().len());
    for (pk, pi) in l.placed_items().iter() {
        if should_stop() {
            return empty;
        }
        // 지금 옮기는 조각 자신은 장애물이 아니다.
        if Some(pk) == ref_pk {
            continue;
        }
        let Some(hash) = cache.library().hash_of_item(pi.item().idx()) else {
            return empty;
        };
        let Some(rotation) = normalized_degrees(pi.d_transf().rotation()) else {
            return empty;
        };
        if cache.library().shape(hash, rotation).is_none() {
            return empty;
        }
        let (tx, ty) = pi.d_transf().translation();
        refs.push(ObstacleRef {
            hash,
            rotation,
            tx: tx as f64 - ox,
            ty: ty as f64 - oy,
        });
    }

    // ── 정렬 기준점 ──────────────────────────────────────────────────────
    //
    // separator는 겹침을 푸는 중이다. 같은 손실이면 **가장 덜 움직이는** 자리가
    // 나머지 레이아웃을 가장 덜 흔든다. 그래서 참조 배치가 있으면 그 자리로부터의
    // 거리순, 없으면(초기 배치 문맥) LBF의 본래 의도대로 좌하단순으로 평가한다.
    let anchor = ref_pk.and_then(|pk| l.placed_items().get(pk)).map(|pi| {
        let (x, y) = pi.d_transf().translation();
        (x as f64 - ox, y as f64 - oy)
    });

    // ── 회전별 후보 ──────────────────────────────────────────────────────
    let engine = NfpEngine::new();
    let mut probes_left = probe_budget;
    let mut out: Vec<Candidate> = Vec::new();
    let mut points: Vec<(f64, f64)> = Vec::new();

    for (deg, rot_rad) in rotations {
        if probes_left == 0 || should_stop() {
            break;
        }
        points.clear();
        feasible_points_for_rotation(
            cache,
            &engine,
            hash_item,
            deg,
            cw,
            ch,
            &refs,
            &mut probes_left,
            &mut points,
            should_stop,
        );
        for &(x, y) in points.iter() {
            let dt = DTransformation::new(rot_rad, ((x + ox) as f32, (y + oy) as f32));
            let key = match anchor {
                Some((ax, ay)) => {
                    let (dx, dy) = (x - ax, y - ay);
                    (dx * dx + dy * dy, 0.0)
                }
                None => (x, y),
            };
            out.push(Candidate { dt, key });
        }
    }

    // 안정 정렬이라 동률 후보는 생성 순서(= 회전 선언 순서 → 출처 순서)를 유지한다.
    out.sort_by(|a, b| {
        a.key
            .0
            .total_cmp(&b.key.0)
            .then_with(|| a.key.1.total_cmp(&b.key.1))
    });
    out
}

/// 한 회전에 대한 feasible 후보 점(컨테이너 로컬 좌표)을 `out`에 채운다.
#[allow(clippy::too_many_arguments)]
fn feasible_points_for_rotation(
    cache: &mut NfpLbfCache,
    engine: &NfpEngine,
    hash_item: GeometryHash,
    deg: i64,
    container_w: f32,
    container_h: f32,
    refs: &[ObstacleRef],
    probes_left: &mut usize,
    out: &mut Vec<(f64, f64)>,
    should_stop: &mut dyn FnMut() -> bool,
) {
    let eps = COORD_TIE_EPS as f64;

    // ── IFP: 아이템이 컨테이너 안에 온전히 들어가는 앵커 영역 ──
    let Some((ifp, ifp_mask, ifp_prepared)) =
        cache.inner_fit_view_prepared(hash_item, deg, container_w, container_h)
    else {
        return;
    };
    if ifp.is_empty() {
        return;
    }
    let Some(ifp_bbox) = aabb_of_mask(&ifp_mask) else {
        return;
    };

    // ── 장애물별 금지 영역 (캐시의 Arc만 빌린다 — 기하 복사 0회) ──
    let mut obstacles: Vec<Obstacle> = Vec::with_capacity(refs.len());
    for r in refs {
        if should_stop() {
            return;
        }
        let key = NfpKey::new(r.hash, r.rotation, hash_item, deg);
        let Some((region, mask, prepared)) = cache.view_prepared_or_compute(key) else {
            // 이 회전의 금지 영역을 다 못 만들었다 = 모델이 불완전하다. 회전 하나를
            // 통째로 버린다. 반쯤 만든 모델로 후보를 내면 안 된다.
            return;
        };
        let Some((lx, ly, hx, hy)) = mask.bounds() else {
            continue;
        };
        let bbox = Aabb {
            min_x: lx,
            min_y: ly,
            max_x: hx,
            max_y: hy,
        }
        .translated(r.tx, r.ty);
        obstacles.push(Obstacle {
            region,
            mask,
            prepared,
            tx: r.tx,
            ty: r.ty,
            tx_q: grid_round(r.tx).unwrap_or(0),
            ty_q: grid_round(r.ty).unwrap_or(0),
            bbox,
        });
    }

    // ── raw 후보 수집 ────────────────────────────────────────────────────
    //
    // 네 번째 성분은 이 점을 만든 obstacle 인덱스다. NFP 정점에서 나온 점은 그
    // obstacle에 대해 **반드시 경계**(= 접촉 배치, 허용)이므로 feasible 판정에서
    // 질의 한 번을 확실히 건너뛸 수 있다.
    let mut raw: Vec<(f64, f64, u32)> = Vec::new();

    // 1) NFP vertex
    'vertices: for (oi, o) in obstacles.iter().enumerate() {
        if should_stop() {
            return;
        }
        for ring in rings(&o.region) {
            for v in &ring.vertices {
                if raw.len() >= MAX_RAW_CANDIDATES {
                    break 'vertices;
                }
                raw.push((v.x + o.tx, v.y + o.ty, oi as u32));
            }
        }
    }

    // 2) IFP vertex — strip 경계에 붙는 자세 (빈 레이아웃의 유일한 후보이기도 하다)
    'ifp_vertices: for ring in rings(&ifp) {
        for v in &ring.vertices {
            if raw.len() >= MAX_RAW_CANDIDATES {
                break 'ifp_vertices;
            }
            raw.push((v.x, v.y, NO_GENERATOR));
        }
    }

    // 3) IFP edge ∩ NFP edge — 조각과 strip 경계에 동시에 접촉하는 자세
    //
    // IFP는 직사각형 컨테이너에서 보통 edge 4개짜리다. 그래서 이 항의 비용은
    // `O(IFP edge × NFP edge)`로 사실상 선형이고, 얻는 것은 "왼쪽 벽에 붙인 채
    // 앞 조각에 밀착"처럼 난수가 절대 못 맞히는 자세다.
    let ifp_edges = edges_of(&ifp, 0.0, 0.0);
    'boundary: for o in obstacles.iter() {
        if should_stop() {
            return;
        }
        if !o.bbox.overlaps(ifp_bbox, eps) {
            continue;
        }
        for eb in edges_of(&o.region, o.tx, o.ty) {
            for ea in ifp_edges.iter() {
                if !segment_bbox_overlaps(ea, &eb, eps) {
                    continue;
                }
                if let Some((x, y)) = segment_intersection(ea, &eb) {
                    if raw.len() >= MAX_RAW_CANDIDATES {
                        break 'boundary;
                    }
                    raw.push((x, y, NO_GENERATOR));
                }
            }
        }
    }

    // ── 0.001mm 격자 중복 제거 ───────────────────────────────────────────
    let mut seen: HashSet<(i64, i64)> = HashSet::with_capacity(raw.len());
    let mut unique: Vec<(f64, f64, u32)> = Vec::with_capacity(raw.len());
    for (x, y, origin) in raw {
        if !x.is_finite() || !y.is_finite() {
            continue;
        }
        if seen.insert(grid_key(x, y)) {
            unique.push((x, y, origin));
        }
    }

    // ── feasible 판정 ────────────────────────────────────────────────────
    for (x, y, origin) in unique {
        if *probes_left == 0 || should_stop() {
            break;
        }
        *probes_left -= 1;
        if is_feasible(
            engine,
            x,
            y,
            origin,
            &ifp,
            &ifp_mask,
            ifp_prepared.as_deref(),
            ifp_bbox,
            &obstacles,
        ) {
            out.push((x, y));
        }
    }
}

/// [`feasible_points_for_rotation`]의 후보에 생성원이 없음을 뜻하는 표식.
const NO_GENERATOR: u32 = u32::MAX;

/// 후보 점 `(x, y)`(컨테이너 로컬)의 feasible 판정.
///
/// * (a) 어떤 `NFP(o, item)`의 **내부**에도 있으면 안 된다. 경계는 접촉 배치라 허용한다.
///   (pair NFP는 캐시가 이미 `PLACEMENT_CLEARANCE_MM`만큼 부풀려 두므로, 경계 위 앵커도
///    실제 도형끼리는 그만큼 떨어져 있다 — f32로 내려간 뒤에도 CDE 겹침이 되지 않는다.)
/// * (b) IFP의 내부 또는 경계에 있어야 한다.
///
/// 마스크가 `Inside`/`Outside`로 확정해 주는 셀은 질의 없이 끝내고, `Boundary`만
/// [`NfpEngine::locate_prepared`]로 정확히 판정한다. 캐시가 방향마다
/// [`PreparedRegion`]을 한 번만 만들어 두므로 호출당 재정규화가 없다.
#[allow(clippy::too_many_arguments)]
fn is_feasible(
    engine: &NfpEngine,
    x: f64,
    y: f64,
    generator: u32,
    ifp: &PolygonSet,
    ifp_mask: &NfpMask,
    ifp_prepared: Option<&PreparedRegion>,
    ifp_bbox: Aabb,
    obstacles: &[Obstacle],
) -> bool {
    let eps = COORD_TIE_EPS as f64;

    // (b) IFP 판정 — 탐침마다 반드시 일어나므로 bbox로 먼저 싸게 거른다.
    if !ifp_bbox.contains(x, y, eps) {
        return false;
    }
    match ifp_mask.classify(x, y) {
        MaskCell::Inside => {}
        MaskCell::Outside => return false,
        MaskCell::Boundary => {
            let located = match ifp_prepared {
                Some(prepared) => engine.locate_prepared(prepared, NfpPoint::new(x, y)),
                None => engine.locate(ifp, NfpPoint::new(x, y)),
            };
            match located {
                Ok(Location::Inside) | Ok(Location::Boundary) => {}
                _ => return false,
            }
        }
    }

    // 질의점을 격자 정수로 한 번만 올려 둔다. obstacle마다 정수 뺄셈만 하면 된다.
    let (Some(px_q), Some(py_q)) = (grid_round(x), grid_round(y)) else {
        // 격자에 못 올리는 좌표는 `locate`도 어차피 Err를 준다 = 기각.
        return false;
    };

    // (a) 어떤 NFP 내부에도 있으면 안 된다.
    for (k, o) in obstacles.iter().enumerate() {
        if !o.bbox.contains(x, y, eps) {
            continue;
        }
        // 이 후보를 만든 obstacle이면 점이 그 NFP의 정점 그 자체다 = 경계 = 허용.
        // 질의를 통째로 건너뛴다.
        if k as u32 == generator {
            continue;
        }
        let lx = (px_q - o.tx_q) as f64 / GRID_SCALE as f64;
        let ly = (py_q - o.ty_q) as f64 / GRID_SCALE as f64;
        match o.mask.classify(lx, ly) {
            MaskCell::Outside => {}
            MaskCell::Inside => return false,
            MaskCell::Boundary => {
                let located = match o.prepared.as_deref() {
                    Some(prepared) => engine.locate_prepared(prepared, NfpPoint::new(lx, ly)),
                    None => engine.locate(&o.region, NfpPoint::new(lx, ly)),
                };
                match located {
                    Ok(Location::Inside) => return false,
                    Ok(_) => {}
                    Err(_) => return false,
                }
            }
        }
    }
    true
}

// ────────────────────────────────────────────────────────────────────────────
// 회전 매핑
// ────────────────────────────────────────────────────────────────────────────

/// 아이템의 허용 회전을 **NFP 라이브러리가 들고 있는 정수 도**와 교집합한다.
///
/// 반환값은 `(라이브러리 조회용 정수 도, 실제로 내보낼 라디안)` 쌍이다. 둘을 나누는
/// 이유는 [`RotationRange::Discrete`] 때문이다 — 그 경우 인스턴스가 선언한 `f32` 값을
/// **그대로** 돌려줘야 한다. 도 → 라디안으로 다시 환산하면 마지막 비트가 달라져
/// "허용된 회전"과 부동소수 동치가 깨질 수 있다.
///
/// 라이브러리에 없는 회전은 NFP를 만들 수 없으므로 조용히 버린다. 전부 버려지면
/// 호출자가 폴백한다.
fn library_rotations_for(item: &Item, library: &[i64]) -> Vec<(i64, f32)> {
    let mut out: Vec<(i64, f32)> = Vec::new();
    let mut push_unique = |deg: i64, rad: f32| {
        if library.contains(&deg) && !out.iter().any(|(d, _)| *d == deg) {
            out.push((deg, rad));
        }
    };

    match item.allowed_orientations().rotations() {
        RotationRange::None => push_unique(0, 0.0),
        RotationRange::Discrete(set) => {
            for &r in set.iter() {
                let Some(deg) = normalized_degrees(r) else {
                    continue;
                };
                push_unique(deg, r);
            }
        }
        // 자유 회전이면 라이브러리가 준비해 둔 자세를 전부 쓸 수 있다.
        RotationRange::Continuous => {
            for &deg in library.iter() {
                push_unique(deg, degrees_to_radians(deg));
            }
        }
    }
    out
}

/// 라디안 자세를 `[0, 360)` 정수 도로 옮긴다.
///
/// 정수 도에서 [`ROTATION_DEG_TOLERANCE`]보다 멀면 `None`이다 — 그 자세의 형상은
/// 라이브러리에 없으므로 NFP로 답할 수 없다.
fn normalized_degrees(rotation: f32) -> Option<i64> {
    if !rotation.is_finite() {
        return None;
    }
    let deg = rotation.to_degrees();
    let rounded = deg.round();
    if (deg - rounded).abs() > ROTATION_DEG_TOLERANCE {
        return None;
    }
    Some((rounded as i64).rem_euclid(360))
}

/// 정수 도 → 라디안. `[0, 2π)`로 정규화한다
/// ([`crate::nfp_lbf::search`]의 `rotation_to_radians`와 같은 식이다).
fn degrees_to_radians(degrees: i64) -> f32 {
    let normalized = degrees.rem_euclid(360);
    (normalized as f64).to_radians() as f32
}

// ────────────────────────────────────────────────────────────────────────────
// 기하 보조
// ────────────────────────────────────────────────────────────────────────────

/// `PolygonSet`의 모든 ring(외곽 + 구멍).
fn rings(set: &PolygonSet) -> impl Iterator<Item = &NfpRing> {
    set.components
        .iter()
        .flat_map(|c| std::iter::once(&c.outer).chain(c.holes.iter()))
}

/// 모든 ring의 닫힌 edge를 `[x1, y1, x2, y2]`로 펼치며 `(dx, dy)`만큼 옮긴다.
fn edges_of(set: &PolygonSet, dx: f64, dy: f64) -> Vec<[f64; 4]> {
    let mut out = Vec::new();
    for ring in rings(set) {
        let n = ring.vertices.len();
        if n < 2 {
            continue;
        }
        for i in 0..n {
            let a = ring.vertices[i];
            let b = ring.vertices[(i + 1) % n];
            out.push([a.x + dx, a.y + dy, b.x + dx, b.y + dy]);
        }
    }
    out
}

/// 마스크가 들고 있는 로컬 bbox를 [`Aabb`]로.
fn aabb_of_mask(mask: &NfpMask) -> Option<Aabb> {
    mask.bounds().map(|(min_x, min_y, max_x, max_y)| Aabb {
        min_x,
        min_y,
        max_x,
        max_y,
    })
}

/// `swarm_nfp`의 `quantize_mm`과 **같은 식**으로 mm를 0.001mm 격자 정수로 올린다.
fn grid_round(v: f64) -> Option<i64> {
    let scaled = v * GRID_SCALE as f64;
    if !scaled.is_finite() || scaled.abs() > 9.0e15 {
        return None;
    }
    Some(scaled.round() as i64)
}

/// 0.001mm 격자 키. 중복 후보 제거용.
fn grid_key(x: f64, y: f64) -> (i64, i64) {
    let s = GRID_SCALE as f64;
    ((x * s).round() as i64, (y * s).round() as i64)
}

fn min_max(a: f64, b: f64) -> (f64, f64) {
    if a <= b { (a, b) } else { (b, a) }
}

/// 두 선분의 bbox가 겹치는지 (교차 판정 전 사전 필터).
fn segment_bbox_overlaps(a: &[f64; 4], b: &[f64; 4], eps: f64) -> bool {
    let (a_min_x, a_max_x) = min_max(a[0], a[2]);
    let (a_min_y, a_max_y) = min_max(a[1], a[3]);
    let (b_min_x, b_max_x) = min_max(b[0], b[2]);
    let (b_min_y, b_max_y) = min_max(b[1], b[3]);
    a_min_x <= b_max_x + eps
        && b_min_x <= a_max_x + eps
        && a_min_y <= b_max_y + eps
        && b_min_y <= a_max_y + eps
}

/// 두 선분의 교점. 평행/공선은 `None`이다.
///
/// 공선 겹침에서 나오는 점은 이미 정점 후보로 들어와 있으므로 따로 처리할 필요가 없다.
fn segment_intersection(a: &[f64; 4], b: &[f64; 4]) -> Option<(f64, f64)> {
    const DENOM_EPS: f64 = 1.0e-12;
    const PARAM_EPS: f64 = 1.0e-9;

    let rx = a[2] - a[0];
    let ry = a[3] - a[1];
    let sx = b[2] - b[0];
    let sy = b[3] - b[1];

    let denom = rx * sy - ry * sx;
    if denom.abs() <= DENOM_EPS {
        return None;
    }

    let qpx = b[0] - a[0];
    let qpy = b[1] - a[1];
    let t = (qpx * sy - qpy * sx) / denom;
    let u = (qpx * ry - qpy * rx) / denom;

    if t < -PARAM_EPS || t > 1.0 + PARAM_EPS || u < -PARAM_EPS || u > 1.0 + PARAM_EPS {
        return None;
    }
    let x = a[0] + t * rx;
    let y = a[1] + t * ry;
    if x.is_finite() && y.is_finite() {
        Some((x, y))
    } else {
        None
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 단위 테스트 — 외부 데이터 파일에 의존하지 않는다.
// ────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use jagua_rs::collision_detection::CDEConfig;
    use jagua_rs::entities::Container;
    use jagua_rs::geometry::OriginalShape;
    use jagua_rs::geometry::fail_fast::SPSurrogateConfig;
    use jagua_rs::geometry::primitives::{Point, SPolygon};
    use jagua_rs::geometry::shape_modification::{ShapeModifyConfig, ShapeModifyMode};
    use jagua_rs::probs::spp::entities::{SPInstance, Strip};
    use rand::SeedableRng;
    use rand::rngs::Xoshiro256PlusPlus;
    use std::f32::consts::PI;

    use crate::nfp_lbf::cache::PreparedLibrary;

    // ── 픽스처 ──────────────────────────────────────────────────────────

    const SURROGATE: SPSurrogateConfig = SPSurrogateConfig {
        n_pole_limits: [(64, 0.0), (16, 0.8), (8, 0.9)],
        ff_pole_area_ratio: 0.5,
        n_ff_piers: 0,
    };

    const CDE: CDEConfig = CDEConfig {
        quadtree_depth: 4,
        cd_threshold: 64,
        item_surrogate_config: SURROGATE,
    };

    /// 좌하단이 원점인 `w × h` 직사각형 아이템.
    fn rect_item(id: usize, w: f32, h: f32, allowed: RotationRange) -> std::sync::Arc<Item> {
        let shape = SPolygon::new(vec![
            Point(0.0, 0.0),
            Point(w, 0.0),
            Point(w, h),
            Point(0.0, h),
        ])
        .expect(crate::dmsg!("valid rectangle"));
        Item::new(id, (id) as u64, OriginalShape {
                shape,
                pre_transform: DTransformation::empty(),
                modify_mode: ShapeModifyMode::Inflate,
                modify_config: ShapeModifyConfig::default(),
            }, jagua_rs::geometry::AllowedOrientations::new(allowed, vec![]).unwrap(), None, SURROGATE).map(std::sync::Arc::new)
        .expect(crate::dmsg!("valid item"))
    }

    fn strip(w: f32, h: f32) -> Strip {
        Strip::new(h, CDE, ShapeModifyConfig::default(), w).expect(crate::dmsg!("valid strip"))
    }

    fn cache_for(instance: &SPInstance, rotations: &[i64]) -> NfpLbfCache {
        let library =
            PreparedLibrary::build(instance, rotations, 0.0).expect(crate::dmsg!("prepared library builds"));
        NfpLbfCache::new(library)
    }

    fn sample_config(n: usize) -> SampleConfig {
        SampleConfig {
            n_container_samples: n,
            n_focussed_samples: 0,
            n_coord_descents: 1,
            left_bias_gain: 0.0,
            tilt_range: 0.0,
        }
    }

    /// 기하만 검증하기 위한 평가기 — CDE를 타지 않고 `x + y`를 손실로 준다.
    /// 이렇게 하면 "무엇이 후보로 들어왔는가"만 시험대에 오른다.
    #[derive(Default)]
    struct LowerLeftEval {
        n_evals: usize,
    }

    impl SampleEvaluator for LowerLeftEval {
        fn evaluate_sample(
            &mut self,
            dt: DTransformation,
            _upper_bound: Option<SampleEval>,
        ) -> SampleEval {
            self.n_evals += 1;
            let (x, y) = dt.translation();
            SampleEval::Clear { loss: x + y }
        }

        fn n_evals(&self) -> usize {
            self.n_evals
        }
    }

    /// 무엇을 넣어도 거부하는 평가기 — 후보가 있어도 `None`이 나오는지 확인한다.
    #[derive(Default)]
    struct RejectAll {
        n_evals: usize,
    }

    impl SampleEvaluator for RejectAll {
        fn evaluate_sample(
            &mut self,
            _dt: DTransformation,
            _upper_bound: Option<SampleEval>,
        ) -> SampleEval {
            self.n_evals += 1;
            SampleEval::Invalid
        }

        fn n_evals(&self) -> usize {
            self.n_evals
        }
    }

    // ── 회전 매핑 ────────────────────────────────────────────────────────

    #[test]
    fn degrees_round_trip_through_the_library_grid() {
        assert_eq!(normalized_degrees(0.0), Some(0));
        assert_eq!(normalized_degrees(PI), Some(180));
        assert_eq!(normalized_degrees(2.0 * PI), Some(0));
        assert_eq!(normalized_degrees(-PI / 2.0), Some(270));
        assert_eq!(degrees_to_radians(0), 0.0);
        assert!((degrees_to_radians(180) - PI).abs() < 1.0e-6);
        assert_eq!(degrees_to_radians(360), 0.0);
    }

    #[test]
    fn non_integer_degrees_have_no_library_shape() {
        // 0.5도는 라이브러리가 준비해 둔 자세가 아니다 — NFP로 답할 수 없다.
        let half_degree = 0.5_f32.to_radians();
        assert_eq!(normalized_degrees(half_degree), None);
        assert_eq!(normalized_degrees(f32::NAN), None);
        assert_eq!(normalized_degrees(f32::INFINITY), None);
    }

    #[test]
    fn discrete_rotations_keep_the_declared_float_value() {
        // 선언값을 그대로 돌려줘야 "허용된 회전" 판정이 부동소수로도 성립한다.
        let declared = PI;
        let item = rect_item(0, 10.0, 4.0, RotationRange::Discrete(vec![0.0, declared]));
        let mapped = library_rotations_for(&item, &[0, 180]);
        assert_eq!(mapped.len(), 2);
        assert_eq!(mapped[0], (0, 0.0));
        assert_eq!(mapped[1].0, 180);
        assert_eq!(
            mapped[1].1.to_bits(),
            declared.to_bits(),
            "선언된 f32 값이 재환산으로 바뀌면 안 된다"
        );
    }

    #[test]
    fn rotations_absent_from_the_library_are_dropped() {
        let item = rect_item(0, 10.0, 4.0, RotationRange::Discrete(vec![0.0, PI]));
        // 라이브러리가 0°만 준비했다면 180°는 쓸 수 없다.
        assert_eq!(library_rotations_for(&item, &[0]), vec![(0, 0.0)]);
        // 0°조차 없으면 이 아이템으로는 NFP 후보를 만들 수 없다.
        assert!(library_rotations_for(&item, &[90]).is_empty());
    }

    #[test]
    fn fixed_and_continuous_rotations_map_as_declared() {
        let fixed = rect_item(0, 10.0, 4.0, RotationRange::None);
        assert_eq!(library_rotations_for(&fixed, &[0, 90]), vec![(0, 0.0)]);

        let free = rect_item(0, 10.0, 4.0, RotationRange::Continuous);
        let mapped = library_rotations_for(&free, &[0, 90]);
        assert_eq!(mapped.len(), 2);
        assert_eq!(mapped[0].0, 0);
        assert_eq!(mapped[1].0, 90);
    }

    // ── 기하 보조 ────────────────────────────────────────────────────────

    #[test]
    fn segment_intersection_finds_the_crossing_and_rejects_parallels() {
        let a = [0.0, 0.0, 10.0, 0.0];
        let b = [5.0, -5.0, 5.0, 5.0];
        let (x, y) = segment_intersection(&a, &b).expect(crate::dmsg!("crossing"));
        assert!((x - 5.0).abs() < 1.0e-9 && y.abs() < 1.0e-9);

        // 평행선은 교점이 없다.
        assert!(segment_intersection(&a, &[0.0, 1.0, 10.0, 1.0]).is_none());
        // 선분 밖에서 만나는 직선도 기각한다.
        assert!(segment_intersection(&a, &[20.0, -5.0, 20.0, 5.0]).is_none());
    }

    #[test]
    fn segment_bbox_prefilter_matches_the_intersection_outcome() {
        let a = [0.0, 0.0, 10.0, 0.0];
        assert!(segment_bbox_overlaps(&a, &[5.0, -5.0, 5.0, 5.0], 0.0));
        assert!(!segment_bbox_overlaps(&a, &[20.0, -5.0, 20.0, 5.0], 0.0));
    }

    #[test]
    fn grid_key_quantizes_to_the_micron_grid() {
        assert_eq!(grid_key(1.0, -2.5), (1_000, -2_500));
        // 0.0005mm 안쪽으로 붙은 두 점은 같은 후보다.
        assert_eq!(grid_key(1.0, 1.0), grid_key(1.0002, 1.0002));
        assert_eq!(grid_round(1.234), Some(1_234));
        assert_eq!(grid_round(f64::INFINITY), None);
    }

    // ── 통합: 실제 NFP 캐시로 후보를 뽑는다 ───────────────────────────────

    #[test]
    fn empty_layout_yields_the_inner_fit_corner() {
        let item = rect_item(0, 10.0, 4.0, RotationRange::None);
        let s = strip(40.0, 20.0);
        let instance = SPInstance::new(vec![(item.clone(), 2)], s);
        let mut cache = cache_for(&instance, &[0]);
        let layout = Layout::new(Container::try_from(s).expect(crate::dmsg!("strip forms a valid container")));
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(1);

        let (best, n_evals) = search_placement_nfp(
            &layout,
            &item,
            None,
            LowerLeftEval::default(),
            sample_config(64),
            &mut cache,
            &mut rng,
        );

        let (dt, _) = best.expect(crate::dmsg!("빈 컨테이너에는 반드시 자리가 있다"));
        assert!(n_evals > 0);
        let (x, y) = dt.translation();
        // IFP 정점 = strip의 네 모서리. 손실 x+y가 최소인 좌하단이 뽑혀야 한다.
        assert!(x.abs() < 0.01, "x = {x}");
        assert!(y.abs() < 0.01, "y = {y}");
    }

    #[test]
    fn a_placed_piece_pushes_the_candidate_onto_the_nfp_boundary() {
        let item = rect_item(0, 10.0, 4.0, RotationRange::None);
        let s = strip(40.0, 20.0);
        let instance = SPInstance::new(vec![(item.clone(), 2)], s);
        let mut cache = cache_for(&instance, &[0]);
        let mut layout = Layout::new(Container::try_from(s).expect(crate::dmsg!("strip forms a valid container")));
        layout.place_item(&item, DTransformation::new(0.0, (0.0, 0.0)));
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(2);

        let (best, _) = search_placement_nfp(
            &layout,
            &item,
            None,
            LowerLeftEval::default(),
            sample_config(256),
            &mut cache,
            &mut rng,
        );

        let (dt, _) = best.expect(crate::dmsg!("좌하단이 막혔어도 접촉 자리는 있다"));
        let (x, y) = dt.translation();
        // (0,0)-(10,4)가 막혀 있으므로 좌하단 그 자체는 후보에서 빠져야 한다.
        assert!(
            x >= 10.0 || y >= 4.0,
            "배치된 조각과 겹치는 자리가 나왔다: ({x}, {y})"
        );
        // x+y가 최소인 접촉 자세는 "왼쪽 벽에 붙인 채 위로 올린" 자리다.
        // 이건 IFP edge ∩ NFP edge에서만 나오는 점이고, 난수로는 맞힐 수 없다.
        assert!(x < 1.0, "왼쪽 벽에 붙지 않았다: x = {x}");
        assert!(
            (4.0..4.1).contains(&y),
            "NFP 경계(= 접촉 + 배치 간극) 위가 아니다: y = {y}"
        );
    }

    #[test]
    fn the_reference_piece_is_not_its_own_obstacle() {
        let item = rect_item(0, 10.0, 4.0, RotationRange::None);
        let s = strip(40.0, 20.0);
        let instance = SPInstance::new(vec![(item.clone(), 2)], s);
        let mut cache = cache_for(&instance, &[0]);
        let mut layout = Layout::new(Container::try_from(s).expect(crate::dmsg!("strip forms a valid container")));
        let pk = layout.place_item(&item, DTransformation::new(0.0, (5.0, 5.0)));
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(3);

        let (best, _) = search_placement_nfp(
            &layout,
            &item,
            Some(pk),
            LowerLeftEval::default(),
            sample_config(256),
            &mut cache,
            &mut rng,
        );

        // 유일한 배치 조각이 참조 조각이므로 장애물은 0개다 = 빈 컨테이너와 같다.
        let (dt, _) = best.expect(crate::dmsg!("참조 조각을 제외하면 컨테이너가 비어 있다"));
        let (x, y) = dt.translation();
        assert!(x.abs() < 0.01 && y.abs() < 0.01, "({x}, {y})");
    }

    #[test]
    fn candidates_are_ordered_by_displacement_when_a_reference_exists() {
        // 참조 배치가 있으면 동률 후보 중 **가장 덜 움직이는** 자리를 남긴다.
        // 평가기가 모든 후보를 Clear{0}으로 보므로 순서가 곧 결과다.
        #[derive(Default)]
        struct AllClear {
            n_evals: usize,
        }
        impl SampleEvaluator for AllClear {
            fn evaluate_sample(
                &mut self,
                _dt: DTransformation,
                _upper_bound: Option<SampleEval>,
            ) -> SampleEval {
                self.n_evals += 1;
                SampleEval::Clear { loss: 0.0 }
            }
            fn n_evals(&self) -> usize {
                self.n_evals
            }
        }

        let item = rect_item(0, 10.0, 4.0, RotationRange::None);
        let s = strip(40.0, 20.0);
        let instance = SPInstance::new(vec![(item.clone(), 3)], s);
        let mut cache = cache_for(&instance, &[0]);
        let mut layout = Layout::new(Container::try_from(s).expect(crate::dmsg!("strip forms a valid container")));
        layout.place_item(&item, DTransformation::new(0.0, (0.0, 0.0)));
        // 참조 조각은 오른쪽 끝 근처에 있다.
        let pk = layout.place_item(&item, DTransformation::new(0.0, (28.0, 15.0)));
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(4);

        let (best, _) = search_placement_nfp(
            &layout,
            &item,
            Some(pk),
            AllClear::default(),
            sample_config(256),
            &mut cache,
            &mut rng,
        );

        let (dt, _) = best.expect(crate::dmsg!("후보가 있어야 한다"));
        let (x, y) = dt.translation();
        // 좌하단이 아니라 참조 자리(28, 15) 근처가 나와야 한다.
        assert!(
            x > 14.0 && y > 7.0,
            "참조 배치에서 가장 가까운 자리가 아니다: ({x}, {y})"
        );
    }

    #[test]
    fn the_evaluation_budget_is_the_container_sample_count() {
        let item = rect_item(0, 10.0, 4.0, RotationRange::None);
        let s = strip(40.0, 20.0);
        let instance = SPInstance::new(vec![(item.clone(), 2)], s);
        let mut cache = cache_for(&instance, &[0]);
        let mut layout = Layout::new(Container::try_from(s).expect(crate::dmsg!("strip forms a valid container")));
        layout.place_item(&item, DTransformation::new(0.0, (0.0, 0.0)));
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(5);

        for budget in [1usize, 3, 7] {
            let (_, n_evals) = search_placement_nfp(
                &layout,
                &item,
                None,
                LowerLeftEval::default(),
                sample_config(budget),
                &mut cache,
                &mut rng,
            );
            assert!(
                n_evals <= budget,
                "예산 {budget}을 넘겨 {n_evals}번 평가했다"
            );
            assert!(n_evals > 0, "후보가 있는데 한 번도 평가하지 않았다");
        }
    }

    #[test]
    fn an_item_that_cannot_fit_produces_no_candidate() {
        // 아이템이 strip보다 크면 IFP가 비고, 후보는 0개다 = 호출자가 폴백한다.
        let big = rect_item(0, 100.0, 4.0, RotationRange::None);
        let s = strip(40.0, 20.0);
        let instance = SPInstance::new(vec![(big.clone(), 1)], s);
        let mut cache = cache_for(&instance, &[0]);
        let layout = Layout::new(Container::try_from(s).expect(crate::dmsg!("strip forms a valid container")));
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(6);

        let (best, n_evals) = search_placement_nfp(
            &layout,
            &big,
            None,
            LowerLeftEval::default(),
            sample_config(64),
            &mut cache,
            &mut rng,
        );
        assert!(best.is_none());
        assert_eq!(n_evals, 0, "후보가 없으면 평가기를 부르지 않는다");
    }

    #[test]
    fn every_candidate_rejected_by_the_evaluator_yields_none() {
        // 기하가 후보를 내도 최종 판정자는 evaluator다. 전부 기각하면 None이다.
        let item = rect_item(0, 10.0, 4.0, RotationRange::None);
        let s = strip(40.0, 20.0);
        let instance = SPInstance::new(vec![(item.clone(), 2)], s);
        let mut cache = cache_for(&instance, &[0]);
        let layout = Layout::new(Container::try_from(s).expect(crate::dmsg!("strip forms a valid container")));
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(7);

        let (best, n_evals) = search_placement_nfp(
            &layout,
            &item,
            None,
            RejectAll::default(),
            sample_config(8),
            &mut cache,
            &mut rng,
        );
        assert!(best.is_none());
        assert!(n_evals > 0, "후보는 있었으므로 평가는 일어나야 한다");
    }

    #[test]
    fn results_are_deterministic_across_rng_seeds() {
        // NFP 후보 열거는 난수를 쓰지 않는다. 시드가 달라도 결과가 같아야 한다.
        let item = rect_item(0, 10.0, 4.0, RotationRange::None);
        let s = strip(40.0, 20.0);
        let instance = SPInstance::new(vec![(item.clone(), 2)], s);
        let mut cache = cache_for(&instance, &[0]);
        let mut layout = Layout::new(Container::try_from(s).expect(crate::dmsg!("strip forms a valid container")));
        layout.place_item(&item, DTransformation::new(0.0, (12.0, 3.0)));

        let mut results = Vec::new();
        for seed in [0u64, 42, 12_345] {
            let mut rng = Xoshiro256PlusPlus::seed_from_u64(seed);
            let (best, _) = search_placement_nfp(
                &layout,
                &item,
                None,
                LowerLeftEval::default(),
                sample_config(128),
                &mut cache,
                &mut rng,
            );
            let (dt, _) = best.expect(crate::dmsg!("후보가 있어야 한다"));
            results.push(dt.translation());
        }
        assert!(
            results.windows(2).all(|w| w[0] == w[1]),
            "시드에 따라 결과가 달라졌다: {results:?}"
        );
    }
}
