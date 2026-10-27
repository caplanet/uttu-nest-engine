//! capmods 전용: LBF 초기 배치용 **래스터 NFP 후보 생성기** (fork 전용 파일).
//!
//! 원본 LBF는 컨테이너 전역에 균등 난수 샘플을 1000개 뿌리고 그중 충돌하지 않는
//! 최선을 고른다. 배치가 진행돼 빈 공간이 좁아질수록 "유효한 자리"가 차지하는
//! 면적 비율이 급감하므로, 난수 샘플링은 좁은 포켓을 확률적으로 통째로 놓친다.
//!
//! 여기서는 레이아웃과 아이템을 **같은 격자로 래스터화**해 겹치지 않는 격자 위치를
//! 직접 열거한다. 아이템 마스크를 행별 런(run)으로 압축하고 점유 격자의 행별 누적합을
//! 두면, 한 후보 위치의 충돌 여부를 런 개수(≈ 아이템 행 수)만큼의 O(1) 조회로 판정할 수
//! 있다. 덕분에 수만 개 위치를 전수 검사해도 비용이 난수 샘플링 수준에 머문다.
//!
//! ## 정확성 — 래스터는 제안만, 판정은 CDE가 한다
//!
//! 격자는 근사이므로 이 모듈의 판정을 그대로 믿지 않는다. 여기서 나온 후보는 전부
//! 호출자가 기존 [`crate::eval::lbf_evaluator::LBFEvaluator`](CDE 정밀 검사)로 검증한
//! 뒤에만 채택된다. 따라서:
//!   - 격자가 낙관적이면(실제로는 겹치는데 통과) → CDE가 걸러낸다. 후보 하나를 낭비할 뿐.
//!   - 격자가 보수적이면(실제로는 되는데 막힘) → 그 자리를 놓칠 뿐 잘못된 배치는 없다.
//!
//! 래스터화는 "셀 중심이 도형 내부" + "모서리가 지나가는 셀"을 모두 표시해 실제
//! 도형보다 약간 크게(보수적으로) 잡는다. 그래서 후보 대부분이 CDE를 통과하고,
//! 남은 한 셀 남짓의 여유는 뒤이은 좌표 하강이 메운다.

use crate::config::RasterNfpConfig;
use jagua_rs::entities::{Item, Layout};
use jagua_rs::geometry::geo_enums::RotationRange;
use jagua_rs::geometry::geo_traits::{CollidesWith, TransformableFrom};
use jagua_rs::geometry::primitives::{Point, Rect, SPolygon};
use jagua_rs::geometry::{DTransformation, Transformation};
use ordered_float::OrderedFloat;
use std::collections::HashMap;

/// 홀 우선 모드가 **실제로 무엇을 했는지** 세는 진단 카운터.
///
/// # 왜 필요한가
///
/// 이 저장소는 "모듈·설정·UI 를 다 만들고 호출하는 줄을 빠뜨리는" 결함을 반복해서
/// 겪었다. 홀 우선 모드는 켜도 결과가 안 바뀔 수 있는데(포켓이 없으면 정상 동작이다),
/// 그때 "포켓이 없었다" 와 "배선이 끊겼다" 를 **구분할 방법이 없으면** 진단이 멈춘다.
/// 이 카운터가 그 둘을 가른다.
///
/// 비용은 후보당 relaxed add 한 번이다.
pub mod hole_stats {
    use std::sync::atomic::{AtomicUsize, Ordering};

    /// `candidates()` 호출 횟수 (홀 우선 모드일 때만).
    pub static CALLS: AtomicUsize = AtomicUsize::new(0);
    /// 격자 스캔에서 포켓 **안**으로 판정된 후보 위치 수.
    pub static POCKET_FOUND: AtomicUsize = AtomicUsize::new(0);
    /// 그중 실제로 반환 목록 앞자리에 실린 수 (억제·상한 통과분).
    pub static POCKET_EMITTED: AtomicUsize = AtomicUsize::new(0);
    /// 포켓 후보가 CDE 를 통과해 **실제 배치로 채택된** 횟수.
    ///
    /// 이 값이 0이면 홀 우선 모드는 아무것도 바꾸지 않는다 — 그 사실을
    /// `POCKET_FOUND` 와 나란히 봐야 원인을 가릴 수 있다.
    pub static POCKET_ACCEPTED: AtomicUsize = AtomicUsize::new(0);

    pub fn reset() {
        CALLS.store(0, Ordering::Relaxed);
        POCKET_FOUND.store(0, Ordering::Relaxed);
        POCKET_EMITTED.store(0, Ordering::Relaxed);
        POCKET_ACCEPTED.store(0, Ordering::Relaxed);
    }

    /// `(호출, 발견, 앞자리 채택, 실제 배치 채택)`
    pub fn snapshot() -> (usize, usize, usize, usize) {
        (
            CALLS.load(Ordering::Relaxed),
            POCKET_FOUND.load(Ordering::Relaxed),
            POCKET_EMITTED.load(Ordering::Relaxed),
            POCKET_ACCEPTED.load(Ordering::Relaxed),
        )
    }
}

/// 후보 풀을 최종 개수의 몇 배까지 모을지 (억제(suppression) 후 개수를 확보하기 위함).
const POOL_OVERSAMPLE: usize = 8;
/// 모서리를 따라 걸으며 셀을 표시할 때의 보폭 (셀 크기 대비).
const EDGE_WALK_STEP_RATIO: f32 = 0.5;
/// 채택된 후보끼리 요구하는 최소 간격 (아이템 최소 변 길이 대비).
const SUPPRESS_RATIO: f32 = 0.5;

/// 레이아웃 점유 격자 + 아이템 마스크 캐시.
///
/// 아이템을 하나 배치할 때마다 [`Self::mark_placed`]로 점유를 갱신한다(증분).
/// strip 폭이 바뀌면 격자 좌표계가 달라지므로 [`Self::rebuild`]로 다시 만든다.
pub struct RasterNfp {
    origin: Point,
    cell: f32,
    nx: usize,
    ny: usize,
    occupied: Vec<bool>,
    /// 행별 누적 점유 수. `prefix[iy * (nx+1) + ix]` = 행 `iy`의 `[0, ix)` 구간 점유 셀 수.
    prefix: Vec<u32>,
    prefix_valid: bool,
    /// 행별 누적 **열린 빈 셀** 수. 열린 빈 셀 = 비어 있으면서 포켓이 아닌 셀,
    /// 즉 packing frontier 바깥(우측 개방)과 4-연결로 이어진 자리다.
    ///
    /// 이걸 세면 "이 후보가 포켓 **안**에 온전히 들어가는가" 를 [`Self::fits`] 와
    /// 똑같은 비용(런 개수만큼의 O(1) 조회)으로 판정할 수 있다. `fits` 가 이미
    /// 점유 셀이 없음을 보장하므로, 열린 셀도 0이면 남은 건 전부 포켓 셀이다.
    open_prefix: Vec<u32>,
    open_valid: bool,
    /// (item.id, rotation.to_bits()) → 마스크
    masks: HashMap<(usize, u32), ItemMask>,
    /// 격자가 커버하는 컨테이너 bbox. 폭이 바뀌면 재생성 필요 여부 판단에 쓴다.
    container: Rect,
}

/// 특정 회전 상태의 아이템을 격자에 래스터화한 마스크.
struct ItemMask {
    /// 회전만 적용한 도형 bbox의 좌하단. 격자 위치 → 평행이동량 환산에 쓴다.
    bbox_min: Point,
    /// 회전만 적용한 도형 bbox의 실제 폭·높이. 격자 셀 수(`mw`, `mh`)는 끝점을 담는
    /// 셀까지 포함해 실제 길이보다 최대 한 셀 크므로, 컨테이너 경계 판정은 이 값으로 한다.
    w: f32,
    h: f32,
    mw: usize,
    mh: usize,
    /// 행별 점유 구간 `(my, x0, x1)` (반열린 구간). 오목한 아이템은 한 행에 여러 런이 생긴다.
    runs: Vec<(usize, usize, usize)>,
}

impl RasterNfp {
    /// 컨테이너 bbox를 덮는 격자를 만든다. `min_item_dim`은 인스턴스 전체에서 가장 작은
    /// 아이템의 최소 변 길이로, 셀 크기 기준이 된다.
    pub fn new(container: Rect, min_item_dim: f32, cfg: &RasterNfpConfig) -> Option<Self> {
        if !cfg.enabled || min_item_dim.is_nan() || min_item_dim <= 0.0 {
            return None;
        }
        let area = container.width() * container.height();
        if area.is_nan() || area <= 0.0 {
            return None;
        }
        // 셀 수 상한을 넘지 않는 선에서 요청한 해상도를 쓴다.
        let min_cell = (area / cfg.max_cells.max(1) as f32).sqrt();
        let cell = (min_item_dim * cfg.cell_ratio.max(0.01)).max(min_cell);
        if cell.is_nan() || cell <= 0.0 {
            return None;
        }

        let nx = grid_dim(container.width(), cell);
        let ny = grid_dim(container.height(), cell);

        Some(Self {
            origin: Point(container.x_min, container.y_min),
            cell,
            nx,
            ny,
            occupied: vec![false; nx * ny],
            prefix: vec![0; ny * (nx + 1)],
            prefix_valid: false,
            open_prefix: vec![0; ny * (nx + 1)],
            open_valid: false,
            masks: HashMap::new(),
            container,
        })
    }

    /// 격자가 현재 컨테이너와 맞는지. strip 폭이 바뀌면 false.
    pub fn matches(&self, container: Rect) -> bool {
        self.container.x_min == container.x_min
            && self.container.y_min == container.y_min
            && self.container.x_max == container.x_max
            && self.container.y_max == container.y_max
    }

    /// 레이아웃 전체로 점유 격자를 다시 만든다.
    pub fn rebuild(&mut self, layout: &Layout) {
        self.occupied.iter_mut().for_each(|c| *c = false);
        for (_, pi) in layout.placed_items().iter() {
            self.rasterize_world(&pi.shape());
        }
        self.prefix_valid = false;
        self.open_valid = false;
    }

    /// 새로 배치된 도형 하나를 점유로 추가한다.
    pub fn mark_placed(&mut self, shape: &SPolygon) {
        self.rasterize_world(shape);
        self.prefix_valid = false;
        self.open_valid = false;
    }

    /// 겹치지 않는 격자 위치들을 LBF 손실(좌·하단 우선) 순으로 최대 `cfg.n_candidates`개 낸다.
    ///
    /// 반환된 변환은 전부 호출자가 CDE로 검증해야 한다 (모듈 주석 참조).
    /// 회전이 연속(`RotationRange::Continuous`)인 아이템은 회전마다 마스크를 만드는 비용이
    /// 커서 대상에서 제외하고 빈 벡터를 반환한다 — 호출자는 기존 난수 경로로 처리한다.
    pub fn candidates(
        &mut self,
        item: &Item,
        x_mult: f32,
        y_mult: f32,
        cfg: &RasterNfpConfig,
    ) -> Vec<DTransformation> {
        self.candidates_split(item, x_mult, y_mult, cfg).0
    }

    /// [`Self::candidates`] 와 같되, 앞쪽 **포켓 후보의 개수**를 함께 낸다.
    ///
    /// # 왜 개수가 필요한가
    ///
    /// 포켓 후보를 목록 앞에 놓는 것만으로는 아무 일도 일어나지 않는다. 호출부는
    /// 시드 전체를 `LBFEvaluator`(좌·하단 손실 최소)로 채점해 **하나**를 고르는데,
    /// 포켓은 정의상 이미 배치된 조각들 사이에 있어 x 가 크고 따라서 손실이 크다.
    /// 순서가 어떻든 argmin 에서 항상 진다.
    ///
    /// 그래서 호출부가 "앞 N 개는 포켓" 을 알아야 그 구간만 따로 먼저 시도할 수 있다.
    /// 실측으로 확인한 사실이다: 순서만 바꿨을 때 폭·밀도가 한 비트도 안 변했고,
    /// 진단 카운터(`hole_stats`)는 포켓 후보 203개가 실제로 만들어졌음을 보여줬다.
    pub fn candidates_split(
        &mut self,
        item: &Item,
        x_mult: f32,
        y_mult: f32,
        cfg: &RasterNfpConfig,
    ) -> (Vec<DTransformation>, usize) {
    #[cfg(feature = "init-profile")]
    let _profile = crate::init_profile::Scope::new("detail.raster_candidates");

        let rotations: Vec<f32> = match item.allowed_orientations().rotations() {
            RotationRange::None => vec![0.0],
            RotationRange::Discrete(v) => v.clone(),
            RotationRange::Continuous => return (vec![], 0),
        };

        self.ensure_prefix();
        let hole_first = cfg.hole_first;
        if hole_first {
            self.ensure_open();
            hole_stats::CALLS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }

        let pool_cap = cfg.n_candidates.max(1) * POOL_OVERSAMPLE;
        // (손실, 회전, ix, iy)
        let mut pool: Vec<(OrderedFloat<f32>, f32, usize, usize)> = vec![];
        // 홀 우선 모드에서만 채운다. 포켓 후보는 정의상 x 가 크므로 손실도 커서,
        // 같은 풀에 넣으면 `pool_cap` 절단에 먼저 잘려 나간다 — 그래서 분리한다.
        let mut hole_pool: Vec<(OrderedFloat<f32>, f32, usize, usize)> = vec![];

        let (cw, ch) = (self.container.width(), self.container.height());

        for r in rotations {
            self.ensure_mask(item, r);
            let Some(mask) = self.masks.get(&(item.idx(), r.to_bits())) else {
                continue;
            };
            if mask.mw > self.nx
                || mask.mh > self.ny
                || mask.runs.is_empty()
                || mask.w > cw
                || mask.h > ch
            {
                continue;
            }
            // 격자 셀 수(`nx`, `ny`)는 컨테이너 끝점을 담는 셀까지 포함하므로 셀 수로만
            // 상한을 잡으면 마지막 열·행의 후보가 오른쪽·위 모서리를 한 셀 미만 넘어간다.
            // 실제 길이로도 묶어 후보의 bbox 가 컨테이너 안에 들어가게 한다.
            let max_ix = (((cw - mask.w) / self.cell).floor() as usize).min(self.nx - mask.mw);
            let max_iy = (((ch - mask.h) / self.cell).floor() as usize).min(self.ny - mask.mh);
            // 컨테이너 위쪽 경계. 양쪽 앵커링의 "위 모서리까지의 거리" 기준이다.
            let top_y = self.container.y_max;

            for ix in 0..=max_ix {
                if crate::config::runtime::init_cancel_requested() { return (vec![], 0); }
                // 이 열에서 나올 수 있는 최소 손실. 이미 모은 후보보다 나쁘면 이후 열은 볼 필요 없다
                // (손실이 x에 대해 단조 증가하므로).
                let col_min =
                    x_mult * (self.origin.0 + ix as f32 * self.cell) + y_mult * self.origin.1;
                if pool.len() >= pool_cap {
                    // 상위 `pool_cap` 개만 남기면 된다 — 전체 정렬은 마지막에 한 번 한다.
                    pool.select_nth_unstable_by_key(pool_cap - 1, |e| e.0);
                    pool.truncate(pool_cap);
                    // 홀 우선 모드에서는 조기 종료하지 않는다. 포켓은 배치가 진행된
                    // **왼쪽 안쪽**이 아니라 조각들 사이 어디에나 생기고, 손실 기준으로는
                    // 이미 모은 후보보다 나쁘다. 여기서 끊으면 정확히 그 자리를 놓친다.
                    // 대가는 격자 전수 스캔이다 — 이 모드의 알려진 비용이다.
                    if !hole_first && x_mult > 0.0 && col_min > pool[pool_cap - 1].0.0 {
                        break;
                    }
                }
                for iy in 0..=max_iy {
                    if !self.fits(ix, iy, mask) {
                        continue;
                    }
                    let wx = self.origin.0 + ix as f32 * self.cell;
                    let wy = self.origin.1 + iy as f32 * self.cell;
                    // 실제 LBF 손실은 (poi + bbox 모서리)의 가중합인데, 두 항 모두 평행이동에
                    // 대해 선형이므로 순위는 이 값과 정확히 일치한다.
                    //
                    // 양쪽 앵커링에서는 y 항을 "가까운 가로 모서리까지의 거리" 로 바꾼다.
                    // 위 모서리까지의 거리는 컨테이너 상단에서 조각 윗변을 뺀 값이다.
                    let y_term = if cfg.double_sided_first_n > 0 {
                        let from_top = top_y - (wy + mask.h);
                        (wy - self.origin.1).min(from_top)
                    } else {
                        wy
                    };
                    let entry = (OrderedFloat(x_mult * wx + y_mult * y_term), r, ix, iy);
                    if hole_first && self.inside_pocket(ix, iy, mask) {
                        hole_stats::POCKET_FOUND.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                        hole_pool.push(entry);
                    } else {
                        pool.push(entry);
                    }
                }
            }
        }

        pool.sort_unstable_by_key(|e| e.0);
        hole_pool.sort_unstable_by_key(|e| e.0);

        // 서로 붙어 있는 격자 위치는 사실상 같은 후보다. 최소 간격을 두고 골라
        // CDE 검증 예산을 다양한 자리에 배분한다.
        let item_min_dim = f32::min(item.shape_cd().bbox().width(), item.shape_cd().bbox().height());
        let suppress = ((item_min_dim * SUPPRESS_RATIO / self.cell) as usize).max(1);

        // 포켓 후보를 **앞쪽**에 놓되 전부 내주지는 않는다. 포켓 후보가 CDE 에서
        // 전멸하면 그 아이템은 시드 후보를 통째로 날리기 때문이다. `hole_share` 가
        // 앞자리 상한이고, 나머지는 기존 LBF 순서가 채운다.
        let hole_cap = if hole_first {
            let share = cfg.hole_share.clamp(0.0, 1.0);
            ((cfg.n_candidates as f32 * share).ceil() as usize).min(cfg.n_candidates)
        } else {
            0
        };
        let mut chosen: Vec<(f32, usize, usize)> = vec![];
        let mut out = vec![];
        let mut n_hole = 0usize;
        for (_, r, ix, iy) in hole_pool {
            if n_hole >= hole_cap {
                break;
            }
            let too_close = chosen.iter().any(|&(cr, cx, cy)| {
                cr == r && cx.abs_diff(ix) < suppress && cy.abs_diff(iy) < suppress
            });
            if too_close {
                continue;
            }
            let Some(mask) = self.masks.get(&(item.idx(), r.to_bits())) else {
                continue;
            };
            let t = (
                self.origin.0 + ix as f32 * self.cell - mask.bbox_min.0,
                self.origin.1 + iy as f32 * self.cell - mask.bbox_min.1,
            );
            chosen.push((r, ix, iy));
            out.push(DTransformation::new(r, t));
            n_hole += 1;
            hole_stats::POCKET_EMITTED.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }
        for (_, r, ix, iy) in pool {
            if out.len() >= cfg.n_candidates {
                break;
            }
            let too_close = chosen.iter().any(|&(cr, cx, cy)| {
                cr == r && cx.abs_diff(ix) < suppress && cy.abs_diff(iy) < suppress
            });
            if too_close {
                continue;
            }
            let Some(mask) = self.masks.get(&(item.idx(), r.to_bits())) else {
                continue;
            };
            // 마스크 원점이 격자점에 놓이도록 하는 평행이동 (마스크와 점유 격자가 정렬된다)
            let t = (
                self.origin.0 + ix as f32 * self.cell - mask.bbox_min.0,
                self.origin.1 + iy as f32 * self.cell - mask.bbox_min.1,
            );
            chosen.push((r, ix, iy));
            out.push(DTransformation::new(r, t));
        }
        (out, n_hole)
    }

    /// 마스크를 격자 위치 `(ix, iy)`에 놓았을 때 점유 셀과 겹치지 않는지.
    fn fits(&self, ix: usize, iy: usize, mask: &ItemMask) -> bool {
        let stride = self.nx + 1;
        for &(my, x0, x1) in &mask.runs {
            let base = (iy + my) * stride;
            if self.prefix[base + ix + x1] != self.prefix[base + ix + x0] {
                return false;
            }
        }
        true
    }

    fn ensure_prefix(&mut self) {
        if self.prefix_valid {
            return;
        }
        let stride = self.nx + 1;
        for iy in 0..self.ny {
            let mut acc = 0u32;
            let base = iy * stride;
            let row = iy * self.nx;
            self.prefix[base] = 0;
            for ix in 0..self.nx {
                if self.occupied[row + ix] {
                    acc += 1;
                }
                self.prefix[base + ix + 1] = acc;
            }
        }
        self.prefix_valid = true;
    }

    /// 빈 셀을 **포켓**과 **열린 만**으로 나눠 `open_prefix` 를 채운다.
    ///
    /// # 포켓의 정의
    ///
    /// 폴리시 단계([`crate::optimizer::polish::holes`])와 **같은 정의**를 쓴다:
    /// strip 우측 경계에 닿지 않는 4-연결 빈 영역. 우측에 닿는 영역은 아직
    /// 아무것도 배치되지 않은 packing frontier 바깥이므로 포켓이 아니다.
    /// 두 곳이 다른 정의를 쓰면 "초기 배치가 채운 홀" 과 "폴리시가 채우는 홀" 이
    /// 서로 다른 것을 가리키게 되고, 실측을 비교할 수 없다.
    ///
    /// 구현은 우측 열에서 시작하는 한 번의 flood fill 이다. 성분마다 따로 라벨을
    /// 붙이는 대신 "우측에서 도달 가능한가" 만 보면 되므로 스택 하나로 끝난다.
    fn ensure_open(&mut self) {
        if self.open_valid {
            return;
        }
        let n = self.nx * self.ny;
        // reachable[i] = 빈 셀이면서 우측 경계에서 4-연결로 도달 가능 (= 열린 만)
        let mut reachable = vec![false; n];
        let mut stack: Vec<usize> = Vec::new();
        let last_col = self.nx - 1;
        for iy in 0..self.ny {
            let idx = iy * self.nx + last_col;
            if !self.occupied[idx] && !reachable[idx] {
                reachable[idx] = true;
                stack.push(idx);
            }
        }
        while let Some(idx) = stack.pop() {
            let (ix, iy) = (idx % self.nx, idx / self.nx);
            let visit = |nidx: usize, reachable: &mut Vec<bool>, stack: &mut Vec<usize>| {
                if !self.occupied[nidx] && !reachable[nidx] {
                    reachable[nidx] = true;
                    stack.push(nidx);
                }
            };
            if ix > 0 {
                visit(idx - 1, &mut reachable, &mut stack);
            }
            if ix + 1 < self.nx {
                visit(idx + 1, &mut reachable, &mut stack);
            }
            if iy > 0 {
                visit(idx - self.nx, &mut reachable, &mut stack);
            }
            if iy + 1 < self.ny {
                visit(idx + self.nx, &mut reachable, &mut stack);
            }
        }

        let stride = self.nx + 1;
        for iy in 0..self.ny {
            let mut acc = 0u32;
            let base = iy * stride;
            let row = iy * self.nx;
            self.open_prefix[base] = 0;
            for ix in 0..self.nx {
                if reachable[row + ix] {
                    acc += 1;
                }
                self.open_prefix[base + ix + 1] = acc;
            }
        }
        self.open_valid = true;
    }

    /// 마스크를 `(ix, iy)` 에 놓았을 때 **열린 만 셀을 하나도 쓰지 않는가**.
    ///
    /// [`Self::fits`] 가 참인 위치에서만 의미가 있다. 그 경우 마스크가 덮는 셀은
    /// 전부 비어 있으므로, 열린 셀이 0이면 남은 건 전부 포켓 셀이다 — 즉 이
    /// 후보는 둘러싸인 주머니 **안**에 온전히 들어간다.
    fn inside_pocket(&self, ix: usize, iy: usize, mask: &ItemMask) -> bool {
        let stride = self.nx + 1;
        for &(my, x0, x1) in &mask.runs {
            let base = (iy + my) * stride;
            if self.open_prefix[base + ix + x1] != self.open_prefix[base + ix + x0] {
                return false;
            }
        }
        true
    }

    fn ensure_mask(&mut self, item: &Item, r: f32) {
        let key = (item.idx(), r.to_bits());
        if self.masks.contains_key(&key) {
            return;
        }
        let mut buf = item.shape_cd().as_ref().clone();
        buf.transform_from(item.shape_cd().as_ref(), &Transformation::from_rotation(r));

        let bbox = buf.bbox();
        let mw = grid_dim(bbox.width(), self.cell);
        let mh = grid_dim(bbox.height(), self.cell);
        let origin = Point(bbox.x_min, bbox.y_min);

        let mut cells = vec![false; mw * mh];
        rasterize(&buf, origin, self.cell, mw, mh, &mut cells);

        // 행별로 연속 구간(런)을 뽑아 둔다 — 후보 검사 비용을 행 수 수준으로 줄인다.
        let mut runs = vec![];
        for my in 0..mh {
            let row = my * mw;
            let mut mx = 0;
            while mx < mw {
                if cells[row + mx] {
                    let start = mx;
                    while mx < mw && cells[row + mx] {
                        mx += 1;
                    }
                    runs.push((my, start, mx));
                } else {
                    mx += 1;
                }
            }
        }

        self.masks.insert(
            key,
            ItemMask {
                bbox_min: origin,
                w: bbox.width(),
                h: bbox.height(),
                mw,
                mh,
                runs,
            },
        );
    }

    /// 도형이 덮는 셀들을 월드 점유 격자에 표시한다.
    fn rasterize_world(&mut self, shape: &SPolygon) {
        let (origin, cell, nx, ny) = (self.origin, self.cell, self.nx, self.ny);
        rasterize(shape, origin, cell, nx, ny, &mut self.occupied);
    }
}

/// `extent` 길이를 덮는 데 필요한 셀 개수.
/// 셀 `i`는 `[origin + i*cell, origin + (i+1)*cell)`를 담당하므로 끝점을 담는 셀까지 포함한다.
fn grid_dim(extent: f32, cell: f32) -> usize {
    ((extent / cell).floor() as usize) + 1
}

/// 도형을 격자에 표시한다.
///
/// 셀 중심이 도형 내부인 경우와 **모서리가 지나가는 셀**을 모두 표시하므로, 결과는 실제
/// 도형을 조금 크게 감싼다(보수적). 얇은 조각이 셀 중심을 하나도 품지 않아 통째로
/// 누락되는 경우를 모서리 표시가 막아 준다.
fn rasterize(shape: &SPolygon, origin: Point, cell: f32, nx: usize, ny: usize, out: &mut [bool]) {
    let idx_of = |p: Point| -> Option<usize> {
        let fx = (p.0 - origin.0) / cell;
        let fy = (p.1 - origin.1) / cell;
        if fx < 0.0 || fy < 0.0 {
            return None;
        }
        let (ix, iy) = (fx as usize, fy as usize);
        if ix >= nx || iy >= ny {
            return None;
        }
        Some(iy * nx + ix)
    };

    // 1. 내부: bbox 범위의 셀 중심을 검사한다.
    let bbox = shape.bbox();
    let ix0 = (((bbox.x_min - origin.0) / cell).floor().max(0.0)) as usize;
    let iy0 = (((bbox.y_min - origin.1) / cell).floor().max(0.0)) as usize;
    let ix1 = ((((bbox.x_max - origin.0) / cell).floor() + 1.0).max(0.0) as usize).min(nx);
    let iy1 = ((((bbox.y_max - origin.1) / cell).floor() + 1.0).max(0.0) as usize).min(ny);
    let poles = shape.surrogate().poles.as_slice();

    for iy in iy0..iy1 {
        for ix in ix0..ix1 {
            let idx = iy * nx + ix;
            if out[idx] {
                continue;
            }
            let c = Point(
                origin.0 + (ix as f32 + 0.5) * cell,
                origin.1 + (iy as f32 + 0.5) * cell,
            );
            // pole(도형 내부에 완전히 들어가는 원) 안이면 정밀 검사를 생략한다.
            let in_pole = poles.iter().any(|p| {
                let (dx, dy) = (c.0 - p.center.0, c.1 - p.center.1);
                dx * dx + dy * dy <= p.radius * p.radius
            });
            if in_pole || shape.collides_with(&c) {
                out[idx] = true;
            }
        }
    }

    // 2. 경계: 각 모서리를 따라 걸으며 지나는 셀을 표시한다.
    let step = cell * EDGE_WALK_STEP_RATIO;
    for e in shape.edge_iter() {
        let (dx, dy) = (e.end.0 - e.start.0, e.end.1 - e.start.1);
        let len = (dx * dx + dy * dy).sqrt();
        let n = ((len / step).ceil() as usize).max(1);
        for i in 0..=n {
            let t = i as f32 / n as f32;
            let p = Point(e.start.0 + dx * t, e.start.1 + dy * t);
            if let Some(idx) = idx_of(p) {
                out[idx] = true;
            }
        }
    }
}

#[cfg(test)]
mod pocket_tests {
    use super::*;

    /// `#` = 점유. 문자열의 **첫 줄이 맨 위 행**이라 읽기 쉽게 뒤집어 넣는다.
    fn grid(rows: &[&str]) -> RasterNfp {
        let ny = rows.len();
        let nx = rows[0].len();
        assert!(rows.iter().all(|r| r.len() == nx), "행 길이가 다르다");
        let mut occupied = vec![false; nx * ny];
        for (top_down, row) in rows.iter().enumerate() {
            let iy = ny - 1 - top_down;
            for (ix, ch) in row.chars().enumerate() {
                occupied[iy * nx + ix] = ch == '#';
            }
        }
        RasterNfp {
            origin: Point(0.0, 0.0),
            cell: 1.0,
            nx,
            ny,
            occupied,
            prefix: vec![0; ny * (nx + 1)],
            prefix_valid: false,
            open_prefix: vec![0; ny * (nx + 1)],
            open_valid: false,
            masks: HashMap::new(),
            container: Rect::try_new(0.0, 0.0, nx as f32, ny as f32).expect(crate::dmsg!("rect")),
        }
    }

    /// 1×1 마스크. 격자 한 칸이 포켓인지 직접 묻는 데 쓴다.
    fn unit_mask() -> ItemMask {
        ItemMask {
            bbox_min: Point(0.0, 0.0),
            w: 1.0,
            h: 1.0,
            mw: 1,
            mh: 1,
            runs: vec![(0, 0, 1)],
        }
    }

    /// 조각들에 둘러싸인 한 칸은 포켓, 우측으로 트인 공간은 포켓이 아니다.
    #[test]
    fn an_enclosed_cell_is_a_pocket_and_open_space_is_not() {
        //  . . . . . . .
        //  . # # # . . .
        //  . # . # . . .   <- (2,2) 가 둘러싸인 주머니
        //  . # # # . . .
        //  . . . . . . .
        let mut g = grid(&[
            ".......", //
            ".###...", //
            ".#.#...", //
            ".###...", //
            ".......",
        ]);
        g.ensure_prefix();
        g.ensure_open();
        let m = unit_mask();

        assert!(g.fits(2, 2, &m), "전제: 그 칸은 비어 있다");
        assert!(g.inside_pocket(2, 2, &m), "둘러싸인 칸은 포켓이어야 한다");

        for (ix, iy) in [(0usize, 0usize), (6, 4), (5, 2), (0, 2)] {
            assert!(g.fits(ix, iy, &m), "전제: ({ix},{iy}) 는 비어 있다");
            assert!(
                !g.inside_pocket(ix, iy, &m),
                "({ix},{iy}) 는 우측으로 트여 있으므로 포켓이 아니다"
            );
        }
    }

    /// **폴리시와 같은 정의여야 한다.** 우측 경계에 닿는 만(bay)은 포켓이 아니다.
    /// 좌·상·하 경계에 닿는 것은 상관없다 — 그쪽은 컨테이너 벽이다.
    #[test]
    fn a_right_open_bay_is_not_a_pocket_but_a_left_closed_one_is() {
        // 우측으로 뚫린 만: 포켓이 아니다.
        let mut bay = grid(&[
            "#####", //
            "#...."
                , //
            "#####",
        ]);
        bay.ensure_prefix();
        bay.ensure_open();
        let m = unit_mask();
        assert!(!bay.inside_pocket(2, 1, &m), "우측으로 뚫린 만은 포켓이 아니다");

        // 우측이 막힌 같은 모양: 포켓이다.
        let mut pocket = grid(&[
            "#####", //
            "#...#", //
            "#####",
        ]);
        pocket.ensure_prefix();
        pocket.ensure_open();
        assert!(pocket.inside_pocket(2, 1, &m), "우측이 막히면 포켓이다");
    }

    /// 포켓에 **일부만** 걸치는 후보는 포켓 후보가 아니다.
    /// 한 칸이라도 열린 만을 쓰면 그건 프론티어 배치다.
    #[test]
    fn a_candidate_straddling_the_pocket_mouth_is_not_counted_as_inside() {
        //  # # # # # #
        //  # . . . . .    <- 우측으로 트인 만
        //  # # # # # #
        let mut g = grid(&[
            "######", //
            "#.....", //
            "######",
        ]);
        g.ensure_prefix();
        g.ensure_open();
        // 가로 2칸 마스크
        let wide = ItemMask {
            bbox_min: Point(0.0, 0.0),
            w: 2.0,
            h: 1.0,
            mw: 2,
            mh: 1,
            runs: vec![(0, 0, 2)],
        };
        assert!(g.fits(1, 1, &wide));
        assert!(!g.inside_pocket(1, 1, &wide));
    }

    /// 점유가 바뀌면 포켓 판정도 다시 계산돼야 한다.
    /// 캐시 무효화를 빠뜨리면 배치가 진행돼도 첫 격자의 포켓만 계속 보게 된다 —
    /// 이 저장소가 반복해서 겪은 "만들고 배선을 빠뜨림" 의 변종이다.
    #[test]
    fn marking_new_occupancy_invalidates_the_pocket_classification() {
        let mut g = grid(&[
            "#####", //
            "#....", //
            "#####",
        ]);
        g.ensure_prefix();
        g.ensure_open();
        let m = unit_mask();
        assert!(!g.inside_pocket(2, 1, &m), "처음에는 우측으로 트여 있다");

        // 입구를 막는다. `mark_placed` 는 도형을 받으므로 여기서는 점유를 직접 바꾸고
        // 같은 무효화를 흉내낸다 — 무효화가 없으면 아래 단정이 깨진다.
        g.occupied[1 * g.nx + 4] = true;
        g.prefix_valid = false;
        g.open_valid = false;
        g.ensure_prefix();
        g.ensure_open();
        assert!(g.inside_pocket(2, 1, &m), "입구가 막히면 포켓이 된다");
    }

    /// 빈 격자에는 포켓이 없다. 초기 배치 첫 아이템에서 홀 후보가 나오면 버그다.
    #[test]
    fn an_empty_grid_has_no_pockets() {
        let mut g = grid(&["....", "....", "...."]);
        g.ensure_prefix();
        g.ensure_open();
        let m = unit_mask();
        for iy in 0..g.ny {
            for ix in 0..g.nx {
                assert!(!g.inside_pocket(ix, iy, &m), "({ix},{iy}) 가 포켓으로 나왔다");
            }
        }
    }
}

#[cfg(test)]
mod bounds_tests {
    use super::*;
    use jagua_rs::geometry::OriginalShape;
    use jagua_rs::geometry::fail_fast::SPSurrogateConfig;
    use jagua_rs::geometry::shape_modification::{ShapeModifyConfig, ShapeModifyMode};

    const SURROGATE: SPSurrogateConfig = SPSurrogateConfig {
        n_pole_limits: [(64, 0.0), (16, 0.8), (8, 0.9)],
        ff_pole_area_ratio: 0.5,
        n_ff_piers: 0,
    };

    /// 좌하단이 원점인 `w × h` 직사각형 아이템 (회전 없음).
    fn rect_item(id: usize, w: f32, h: f32) -> std::sync::Arc<Item> {
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
            }, jagua_rs::geometry::AllowedOrientations::new(RotationRange::None, vec![]).unwrap(), None, SURROGATE).map(std::sync::Arc::new)
        .expect(crate::dmsg!("valid item"))
    }

    fn cfg(n_candidates: usize) -> RasterNfpConfig {
        RasterNfpConfig {
            enabled: true,
            cell_ratio: 1.0,
            max_cells: 1_000_000,
            n_candidates,
            double_sided_first_n: 0,
            n_random_samples: 0,
            hole_first: false,
            hole_share: 0.0,
        }
    }

    /// 격자 셀 수는 컨테이너 끝점을 담는 셀까지 포함한다(10mm 를 1mm 셀로 덮으면 11칸).
    /// 셀 수로만 상한을 잡으면 마지막 열·행의 후보가 컨테이너 오른쪽·위 모서리를
    /// 한 셀 미만 넘어간다 — 그런 후보는 CDE 에서 전부 떨어져 예산만 낭비한다.
    #[test]
    fn candidates_never_protrude_past_the_container_right_or_top_edge() {
        let container = Rect::try_new(0.0, 0.0, 10.0, 6.0).expect(crate::dmsg!("rect"));
        let cfg = cfg(1000);
        let mut g = RasterNfp::new(container, 1.0, &cfg).expect(crate::dmsg!("grid"));
        assert_eq!((g.nx, g.ny), (11, 7), "전제: 끝점을 담는 셀까지 포함한다");

        let (w, h) = (2.5f32, 1.5f32);
        let item = rect_item(0, w, h);
        let out = g.candidates(&item, 1.0, 1.0, &cfg);
        assert!(!out.is_empty(), "빈 컨테이너에는 후보가 있어야 한다");

        let mut max_right = f32::NEG_INFINITY;
        let mut max_top = f32::NEG_INFINITY;
        for dt in &out {
            let (x, y) = dt.translation();
            let (right, top) = (x + w, y + h);
            assert!(right <= container.x_max, "오른쪽 모서리를 넘었다: {right}");
            assert!(top <= container.y_max, "위 모서리를 넘었다: {top}");
            max_right = max_right.max(right);
            max_top = max_top.max(top);
        }
        // 경계 **안쪽** 마지막 격자 열·행까지는 그대로 후보로 낸다.
        assert!(max_right >= 9.5, "오른쪽 경계 근처 후보가 사라졌다: {max_right}");
        assert!(max_top >= 5.5, "위 경계 근처 후보가 사라졌다: {max_top}");
    }

    /// 컨테이너보다 큰 아이템은 후보를 내지 않는다.
    #[test]
    fn an_item_larger_than_the_container_yields_no_candidate() {
        let container = Rect::try_new(0.0, 0.0, 10.0, 6.0).expect(crate::dmsg!("rect"));
        let cfg = cfg(64);
        let mut g = RasterNfp::new(container, 1.0, &cfg).expect(crate::dmsg!("grid"));
        let item = rect_item(0, 10.5, 1.0);
        assert!(g.candidates(&item, 1.0, 1.0, &cfg).is_empty());
    }
}
