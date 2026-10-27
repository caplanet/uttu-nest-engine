//! 초기배치 엔진 본체.
//!
//! 알고리즘
//! 1) 정렬:   아이템을 면적 내림차순 (큰 것 먼저)
//! 2) 첫 배치: 최상위 아이템 1개를 좌하단(0,0)에 놓는다
//! 3) 반복:   후보 = {미배치 i} x {배치된 j} x {허용 회전 r}
//!            NFP 경계 정점에서 후보 위치를 만들고, CDE 정확 검사 -> 슬라이딩 -> 점수.
//!            최선 1개 배치. 새 배치로 새로 가능해진 후보만 증분 추가.
//! 4) 실패 아이템: 부착 대상을 바꿔 2차 패스 1회 재시도
//! 5) fit_strip 후 save
//!
//! 부착 대상은 **배치된 조각 전체**다. 최근 N개로 제한하지 않는다.

use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering as AtomicOrdering};
use std::time::Instant as StdInstant;

use jagua_rs::collision_detection::hazards::BasicHazardEntity;
use jagua_rs::collision_detection::hazards::collector::BasicHazardCollector;
use jagua_rs::collision_detection::hazards::filter::NoFilter;
use jagua_rs::entities::{PItemKey};
use jagua_rs::geometry::convex_hull::convex_hull_from_points;
use jagua_rs::geometry::geo_enums::RotationRange;
use jagua_rs::geometry::geo_traits::{Transformable, TransformableFrom};
use jagua_rs::geometry::primitives::{Point, Rect, SPolygon};
use jagua_rs::geometry::{DTransformation, Transformation};
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution};
use rand::RngExt;
use rand::rngs::Xoshiro256PlusPlus;

use super::compact::{self, CompactCfg};
use super::nfp::{self, Nfp, NfpCache, NfpKey};
use super::raster::OccRaster;
use super::{AttachCfg, Order, Stage, Structure, clamp_opt};

/// 정렬 키에 주는 곱셈 지터. seed 별로 순서를 조금씩 흔들어
/// `dens_stddev`(순서 민감도)와 G4 재시작이 의미를 갖게 한다.
// ponytail: 고정 3%. 순서 민감도가 크게 나오면 그때 조정.
const ORDER_JITTER: f32 = 0.03;

/// 작업 스트립 폭 = 100% 밀도 폭 x 이 배수. 배치 후 `fit_strip`으로 조인다.
const WORK_WIDTH_MULT: f32 = 2.2;

/// 배치 간격 epsilon (모델 단위 = mm).
/// 조각이 서로/경계와 "정확히" 맞닿으면 CDE 경계 판정이 축퇴해
/// 삽입 시점엔 통과했다가 이후 겹침으로 보고된다. 전 좌표를 이만큼 띄운다.
/// 5087mm 스트립에서 0.05mm 는 밀도에 1e-5 미만 영향이다.
pub const PLACE_EPS: f32 = 0.05;

/// 최종 감사에서 겹친 조각을 다시 놓는 최대 라운드 수.
const REPAIR_ROUNDS: usize = 4;

/// 쌍-회전 하나에서 허용하는 최대 CDE 검사 수. 비볼록 NFP는 조각 배열이라
/// 겹치는(=거부되는) 후보가 많아, 예산이 없으면 검사 수가 폭발한다.
/// 예산에 걸려 버린 후보 수는 `budget_dropped` 로 로그에 남긴다.
// ponytail: 400. 밀도가 이 값에 민감하면 그때 올린다.
const CDE_BUDGET: usize = 4000;

/// 하한 정렬을 청크 단위로 하는 크기. 전체 정렬을 피한다.
const LB_CHUNK: usize = 256;

/// 2C-D 에서 한 조각당 훑는 후보 위치 상한. 초과분은 하한 순으로 잘린다.
// ponytail: 40000 = 사실상 전수. 3000(하한 순 상위)에서는 좌측 밀집부만 봐서 0/25 였다.
const INSERT_SCAN_CAP: usize = 40000;

/// 후보 위치 중복 제거 허용오차(mm). 이보다 가까운 같은 회전의 위치는 한 번만 검사한다.
const DEDUP_TOL: f32 = 0.5;

/// 이분 탐색 슬라이딩 종료 허용오차(mm). 이보다 좁아지면 남은 반복을 생략한다.
const SLIDE_TOL: f32 = 0.01;

#[derive(Clone, Debug, Default)]
pub struct AttachStats {
    pub placed: usize,
    pub total: usize,
    pub density: f32,
    pub width: f32,
    pub secs: f64,
    /// NFP 캐시 적중/실패
    pub nfp_cache_hits: u64,
    pub nfp_cache_misses: u64,
    /// NFP 정점 수 합 / NFP 개수 (평균 산출용)
    pub nfp_verts_total: u64,
    pub nfp_count: u64,
    /// NFP 볼록조각 수 합 (F단계)
    pub nfp_pieces_total: u64,
    pub hull_fallbacks: u64,
    /// 후보/검사 카운터
    pub cand_generated: u64,
    pub cde_checks: u64,
    /// 2차 패스로 살린 개수, 꼬리 강제 배치 개수
    pub second_pass: usize,
    pub tail_appends: usize,
    /// G단계
    pub lp_fail: usize,
    pub lp_iters_used: usize,
    pub lp_interrupted: usize,
    pub compact_gain_pp: f32,
    /// 진단
    pub self_collisions: usize,
    pub repaired: usize,
    pub unrepairable: usize,
    pub slide_gain_total: f64,
    /// G단계 진단
    pub lp_rows: usize,
    pub lp_pairs: usize,
    pub lp_skipped_pairs: usize,
    pub lp_applied: usize,
    pub lp_reverted: usize,
    pub blocking_item: Option<usize>,
    pub blocking_travel: f32,
    /// CDE 예산에 걸린 (쌍,회전) 수와 그때 버린 후보 수
    pub budget_hits: u64,
    pub budget_dropped: u64,
    pub lp_frozen_pairs: usize,

    // ---- N02C ----
    /// 실제로 수행한 슬라이딩 횟수
    pub slides: u64,
    /// 슬라이딩 후 좌표가 겹쳐서 슬라이딩을 취소한 횟수
    pub slide_cancelled: u64,
    /// 슬라이딩 후 좌표가 이미 나온 위치라 건너뛴 횟수 (평가 횟수에 세지 않는다)
    pub dedup_skipped: u64,
    /// 실제로 채점한 후보 수
    pub cands_evaluated: u64,
    /// 배치 직후 등록 도형 재검사에서 겹쳐 되돌린 횟수
    pub place_reverted: u64,
    // ---- 2C-D ----
    pub inserted: usize,
    pub insert_attempts: usize,
    pub ruined_total: usize,
    pub insert_reverted: usize,
    pub insert_fail_ids: Vec<usize>,
    /// 끝까지 배치하지 못한 아이템 id
    pub unplaced_ids: Vec<usize>,
}

/// 아이템별 (회전 인덱스 -> 회전 적용된 기준 도형 + 볼록 껍질).
struct ItemGeom {
    angles: Vec<f32>,
    base: Vec<Arc<SPolygon>>,
    /// 회전별 CCW 볼록 껍질
    hull: Vec<Vec<Point>>,
    /// 회전별 −껍질 (CCW 복원)
    hull_neg: Vec<Vec<Point>>,
    /// F단계: 회전별 볼록 조각 배열 (분해 실패 시 껍질 1조각)
    pieces: Vec<Vec<Vec<Point>>>,
    /// F단계: 회전별 −조각 배열 (CCW 복원)
    pieces_neg: Vec<Vec<Vec<Point>>>,
}

/// 배치 후보 하나 (기하만 보관, 점수는 선택 시점 ctx로 다시 계산).
#[derive(Clone, Copy, Debug)]
struct Cand {
    rot: u16,
    tx: f32,
    ty: f32,
}

/// 이미 배치된 조각의 요약(후보 생성용).
#[derive(Clone, Copy)]
struct Placed {
    pkey: PItemKey,
    item_id: usize,
    rot: usize,
    bbox: Rect,
    tx: f32,
    ty: f32,
}

pub struct Engine<'a> {
    inst: &'a SPInstance,
    cfg: &'a AttachCfg,
    stage: Stage,
    geom: Vec<ItemGeom>,
    /// 도형 등가 클래스 (같은 도형이면 NFP도 같다)
    class: Vec<u32>,
    /// 아이템별 실면적 (`shape_orig.area()`)
    area: Vec<f32>,
    /// 최대 실면적 (선택 점수의 크기 편향 정규화)
    area_max: f32,
    strip_h: f32,
    /// 충돌 검사용 스크래치 버퍼 (아이템마다 정점 수가 달라 아이템별로 하나씩)
    buff: Vec<SPolygon>,
    cache: NfpCache,
    /// 래스터 맞춤 평가용 점유 격자 (`cfg.raster_fit_cell > 0` 일 때만).
    /// 작업 폭이 정해진 뒤 `init_raster` 로 만들고, `try_push_placed` 성공 시 표시한다.
    /// 조각 제거 경로(강제 삽입·롤백)는 반영하지 않는다 — S2C 프로브 용도.
    raster: Option<OccRaster>,
    /// `collect_overlaps` 용 재사용 수집기 (호출마다 할당하지 않는다).
    collector: BasicHazardCollector,
    pub stats: AttachStats,
}

/// 두 회전각이 같은가 (2π 주기, `rot_index_for` 와 같은 허용오차).
fn same_angle(a: f32, b: f32) -> bool {
    let norm = |x: f32| x.rem_euclid(std::f32::consts::TAU);
    let d = (norm(a) - norm(b)).abs();
    d < 1e-4 || (std::f32::consts::TAU - d) < 1e-4
}

impl<'a> Engine<'a> {
    pub fn new(inst: &'a SPInstance, cfg: &'a AttachCfg, stage: Stage) -> Self {
        Self::with_seed(inst, cfg, stage, None)
    }

    /// `new` 와 같되, `seed` 배치에 실제로 쓰인 회전각을 각 아이템의 회전 테이블에
    /// 더한다. S2C 처럼 회전을 끈 단계에서 외부 해(회전 포함)를 복원하면
    /// `rot_index_for` 가 각을 찾지 못해 0 번 기하로 평가하던 문제를 막는다.
    pub fn with_seed(
        inst: &'a SPInstance,
        cfg: &'a AttachCfg,
        stage: Stage,
        seed: Option<&SPSolution>,
    ) -> Self {
        let n_types = inst.items.len();
        let mut geom = Vec::with_capacity(n_types);
        let mut area = Vec::with_capacity(n_types);
        let mut buff = Vec::with_capacity(n_types);
        let mut class = Vec::with_capacity(n_types);
        let mut class_map: HashMap<u64, u32> = HashMap::new();
        let mut n_hull_fallback: u64 = 0;

        // 회전 테이블은 시작 시 한 번만 만든다 (후보 평가에서 삼각함수 호출 제거)
        let table: Vec<f32> = (0..cfg.rotations)
            .map(|i| std::f32::consts::TAU * i as f32 / cfg.rotations as f32)
            .collect();

        for id in 0..n_types {
            let item = inst.item(id);
            let mut angles: Vec<f32> = if !stage.use_rotations() {
                // E단계 전: 회전 없음. 허용 집합에 0이 없으면 첫 허용각을 쓴다.
                match item.allowed_orientations().rotations() {
                    RotationRange::None | RotationRange::Continuous => vec![0.0],
                    RotationRange::Discrete(v) => {
                        if v.iter().any(|a| a.abs() < 1e-6) {
                            vec![0.0]
                        } else {
                            vec![v[0]]
                        }
                    }
                }
            } else {
                // 허용 회전과 교집합
                match item.allowed_orientations().rotations() {
                    RotationRange::None => vec![0.0],
                    RotationRange::Continuous => table.clone(),
                    RotationRange::Discrete(v) => v.clone(),
                }
            };
            // 시드 배치에 쓰인 각을 테이블에 더한다 (중복은 허용오차로 제거).
            if let Some(sol) = seed {
                for pi in sol.layout_snapshot().placed_items().values() {
                    if pi.item().idx() != id {
                        continue;
                    }
                    let a = pi.d_transf().rotation();
                    if !angles.iter().any(|&t| same_angle(t, a)) {
                        angles.push(a);
                    }
                }
            }
            let base: Vec<Arc<SPolygon>> = angles
                .iter()
                .map(|&a| {
                    if a == 0.0 {
                        item.shape_cd().clone()
                    } else {
                        Arc::new(
                            item.shape_cd()
                                .transform_clone(&Transformation::from_rotation(a)),
                        )
                    }
                })
                .collect();
            let hull: Vec<Vec<Point>> = base.iter().map(|s| nfp::hull_ccw(s)).collect();
            let hull_neg: Vec<Vec<Point>> = hull.iter().map(|h| nfp::negate_ccw(h)).collect();

            // F단계: 비볼록 NFP용 볼록 조각 배열. 상한 초과 시 껍질 근사로 폴백.
            let mut pieces: Vec<Vec<Vec<Point>>> = Vec::new();
            let mut pieces_neg: Vec<Vec<Vec<Point>>> = Vec::new();
            if stage.use_nonconvex_nfp() {
                for (r, b) in base.iter().enumerate() {
                    let pcs = match nfp::decompose_convex(&b.vertices(), cfg.nfp_max_pieces) {
                        Some(v) => v,
                        None => {
                            n_hull_fallback += 1;
                            vec![hull[r].clone()]
                        }
                    };
                    let neg: Vec<Vec<Point>> = pcs.iter().map(|q| nfp::negate_ccw(q)).collect();
                    pieces.push(pcs);
                    pieces_neg.push(neg);
                }
            }

            // 등가 클래스는 (도형, 회전 테이블) 로 정한다 — NFP 캐시 키의 rot 인덱스가
            // 같은 클래스 안에서 같은 각을 뜻해야 한다.
            let h = class_hash(&item.shape_cd(), &angles);
            let next = class_map.len() as u32;
            let c = *class_map.entry(h).or_insert(next);
            class.push(c);

            buff.push(item.shape_cd().as_ref().clone());
            area.push(item.area());
            geom.push(ItemGeom {
                angles,
                base,
                hull,
                hull_neg,
                pieces,
                pieces_neg,
            });
        }

        Engine {
            inst,
            cfg,
            stage,
            geom,
            class,
            area_max: area.iter().cloned().fold(1.0f32, f32::max),
            area,
            strip_h: inst.base_strip.fixed_height,
            buff,
            cache: NfpCache::new(),
            raster: None,
            collector: BasicHazardCollector::with_capacity(16),
            stats: AttachStats {
                hull_fallbacks: n_hull_fallback,
                ..Default::default()
            },
        }
    }

    /// 정렬 키 (내림차순으로 정렬한다).
    fn order_key(&self, item_id: usize) -> f32 {
        let item = self.inst.item(item_id);
        match self.cfg.order {
            Order::Area => self.area[item_id],
            Order::Diameter => item.shape_cd().diameter(),
            Order::Concavity => {
                let ha = nfp::signed_area(&self.geom[item_id].hull[0]).abs();
                if item.shape_cd().area() > 0.0 {
                    ha / item.shape_cd().area()
                } else {
                    0.0
                }
            }
        }
    }

    /// 충돌 검사. true = 충돌.
    fn collides(&mut self, prob: &SPProblem, item_id: usize, rot: usize, tx: f32, ty: f32) -> bool {
        self.stats.cde_checks += 1;
        let base = self.geom[item_id].base[rot].clone();
        let t = Transformation::from_translation((tx, ty));
        let buff = &mut self.buff[item_id];
        buff.transform_from(&base, &t);
        prob.layout().cde().detect_poly_collision(buff, &NoFilter)
    }

    /// 2C-D: 겹침을 버리지 않고 **수집한다**.
    /// 스트립 밖(Exterior)에 걸리면 그 후보는 무효라 `false` 를 돌려준다.
    fn collect_overlaps(
        &mut self,
        prob: &SPProblem,
        item_id: usize,
        rot: usize,
        tx: f32,
        ty: f32,
        exclude: Option<PItemKey>,
        out: &mut Vec<PItemKey>,
    ) -> bool {
        self.stats.cde_checks += 1;
        let base = self.geom[item_id].base[rot].clone();
        let t = Transformation::from_translation((tx, ty));
        let buff = &mut self.buff[item_id];
        buff.transform_from(&base, &t);
        let c = &mut self.collector;
        c.clear();
        prob.layout().cde().collect_poly_collisions(buff, &NoFilter, c);
        out.clear();
        let mut inside = true;
        for (_, e) in c.iter() {
            match e {
                // 이미 배치된 조각을 그 자리에서 재평가할 때 자기 자신이 잡힌다.
                BasicHazardEntity::PlacedItem { pk, .. } if Some(*pk) != exclude => out.push(*pk),
                BasicHazardEntity::PlacedItem { .. } => {}
                _ => inside = false,
            }
        }
        inside
    }

    /// 스트립 범위 밖이면 즉시 버린다(CDE 호출 전 값싼 컷).
    #[inline(always)]
    fn inside_strip(&self, bb: &Rect, tx: f32, ty: f32, strip_w: f32) -> bool {
        let y0 = bb.y_min + ty;
        let y1 = bb.y_max + ty;
        let x0 = bb.x_min + tx;
        let x1 = bb.x_max + tx;
        y0 >= PLACE_EPS
            && y1 <= self.strip_h - PLACE_EPS
            && x0 >= PLACE_EPS
            && x1 <= strip_w - PLACE_EPS
    }

    /// 점수식. D단계 미만에서는 좌하단 항만 쓴다.
    fn score(&self, item_id: usize, bb: &Rect, tx: f32, ty: f32, ctx: &ScoreCtx) -> f32 {
        let (fit, lbf) = self.score_terms(item_id, bb, tx, ty, ctx);
        fit + lbf
    }

    /// 전역 선택(조각 간 비교)용 점수. 좌하단 항을 `select_lbf_scale` 배로 줄이고
    /// 면적 비율만큼 빼서, 위치 항이 작은 조각을 항상 고르는 편향을 상쇄한다.
    fn select_score(&self, item_id: usize, bb: &Rect, tx: f32, ty: f32, ctx: &ScoreCtx) -> f32 {
        let (fit, lbf) = self.score_terms(item_id, bb, tx, ty, ctx);
        fit + lbf * self.cfg.select_lbf_scale
            - self.cfg.select_size_bias * self.area[item_id] / self.area_max
    }

    fn init_raster(&mut self, work_w: f32) {
        self.raster = (self.cfg.raster_fit_cell > 0.0)
            .then(|| OccRaster::new(work_w, self.strip_h, self.cfg.raster_fit_cell));
    }

    /// 래스터 낭비 면적. 후보 자리에서 bbox 안쪽 빈 칸 + 왼쪽/아래 그림자 빈 칸.
    fn raster_waste(&self, item_id: usize, rot: usize, tx: f32, ty: f32) -> Option<f32> {
        let r = self.raster.as_ref()?;
        let f = r.fit(&self.geom[item_id].base[rot], tx, ty, self.cfg.raster_shadow_cap);
        Some(f.waste_cells() as f32 * r.cell() * r.cell())
    }

    /// (맞춤 항 = waste + len_gain, 좌하단 항). D단계 미만에서는 맞춤 항이 0 이다.
    fn score_terms(&self, item_id: usize, bb: &Rect, tx: f32, ty: f32, ctx: &ScoreCtx) -> (f32, f32) {
        let x0 = bb.x_min + tx;
        let y0 = bb.y_min + ty;
        let lbf_term = x0 * self.cfg.lbf_x_weight + y0 * self.cfg.lbf_y_weight;
        if !self.stage.use_full_score() {
            return (0.0, lbf_term);
        }
        let x1 = bb.x_max + tx;
        let y1 = bb.y_max + ty;
        // len_gain: 이 배치로 늘어나는 스트립 길이
        let len_gain = (x1 - ctx.cur_len).max(0.0);
        // waste: 배치 점유 bbox 확장분에서 아이템 실면적을 뺀 값(>=0)
        let nx0 = ctx.bx0.min(x0);
        let ny0 = ctx.by0.min(y0);
        let nx1 = ctx.bx1.max(x1);
        let ny1 = ctx.by1.max(y1);
        let old_a = (ctx.bx1 - ctx.bx0).max(0.0) * (ctx.by1 - ctx.by0).max(0.0);
        let new_a = (nx1 - nx0) * (ny1 - ny0);
        let waste = (new_a - old_a - self.area[item_id]).max(0.0);
        // 래스터 맞춤이 켜져 있으면 bbox 낭비 대신 실제 빈 면적을 쓴다.
        // 회전은 bbox 로 역추적한다 (S2C 는 회전 1개).
        let waste = match self.raster {
            Some(_) => {
                let rot = self.geom[item_id]
                    .base
                    .iter()
                    .position(|b| b.bbox() == *bb)
                    .unwrap_or(0);
                self.raster_waste(item_id, rot, tx, ty).unwrap_or(waste)
            }
            None => waste,
        };
        if self.cfg.normalize_score {
            // 2C-A 무차원화 — 세 항을 같은 크기 범위로 맞춘다.
            // 나누는 값은 후보별로 변하지 않아야 후보 간 비교가 성립한다.
            let waste_n = waste / self.area[item_id].max(1.0);
            let len_gain_n = len_gain / self.strip_h;
            let lbf_n = lbf_term / self.strip_h;
            return (
                clamp_opt(waste_n * self.cfg.eff_area_factor, self.cfg.eff_area_factor_max)
                    + clamp_opt(len_gain_n * self.cfg.score_factor, self.cfg.score_factor_max),
                lbf_n,
            );
        }
        (
            clamp_opt(waste * self.cfg.eff_area_factor, self.cfg.eff_area_factor_max)
                + clamp_opt(len_gain * self.cfg.score_factor, self.cfg.score_factor_max),
            lbf_term,
        )
    }

    /// 점수 하한(가지치기용). waste >= 0 이므로 좌하단항 + len_gain항은 유효한 하한이다.
    #[inline(always)]
    fn score_lb(&self, bb: &Rect, tx: f32, ty: f32, ctx: &ScoreCtx) -> f32 {
        let lbf = (bb.x_min + tx) * self.cfg.lbf_x_weight
            + (bb.y_min + ty) * self.cfg.lbf_y_weight;
        if !self.stage.use_full_score() {
            return lbf;
        }
        let len_gain = (bb.x_max + tx - ctx.cur_len).max(0.0);
        if self.cfg.normalize_score {
            return lbf / self.strip_h
                + clamp_opt(
                    (len_gain / self.strip_h) * self.cfg.score_factor,
                    self.cfg.score_factor_max,
                );
        }
        lbf + clamp_opt(len_gain * self.cfg.score_factor, self.cfg.score_factor_max)
    }

    /// (고정 도형 = p, 이동 도형 = item_id@rot) 의 NFP.
    /// 결과는 **고정 도형의 로컬 프레임** 기준이다. 쓸 때 p 의 평행이동을 더한다.
    fn nfp_for(&mut self, p: &Placed, item_id: usize, rot: usize) -> Arc<Nfp> {
        let splits = self.cfg.nfp_edge_splits.max(1);
        let key = NfpKey {
            class_b: self.class[p.item_id],
            rot_b: p.rot as u16,
            class_a: self.class[item_id],
            rot_a: rot as u16,
        };
        // geom(공유) 과 cache(가변) 는 서로 다른 필드라 복제 없이 함께 빌릴 수 있다.
        let geom = &self.geom;
        let cache = &mut self.cache;
        if self.stage.use_nonconvex_nfp() {
            return cache.get_or_build(key, || {
                nfp::nfp_piece_array(
                    &geom[p.item_id].pieces[p.rot],
                    &geom[item_id].pieces_neg[rot],
                    PLACE_EPS,
                    splits,
                )
            });
        }
        cache.get_or_build(key, || {
            nfp::nfp_hull_pair(
                &geom[p.item_id].hull[p.rot],
                &geom[item_id].hull_neg[rot],
                PLACE_EPS,
                splits,
            )
        })
    }

    fn sync_cache_stats(&mut self) {
        self.stats.nfp_cache_hits = self.cache.hits;
        self.stats.nfp_cache_misses = self.cache.misses;
        self.stats.nfp_verts_total = self.cache.verts_total;
        self.stats.nfp_count = self.cache.count;
        self.stats.nfp_pieces_total = self.cache.pieces_total;
    }
}

#[derive(Clone, Copy, Debug)]
struct ScoreCtx {
    cur_len: f32,
    bx0: f32,
    by0: f32,
    bx1: f32,
    by1: f32,
}

/// (도형, 회전 테이블) 등가 클래스 해시. 각은 비트 단위로 비교한다 — 같은 테이블/시드에서
/// 나온 각만 클래스를 공유하면 충분하고, 갈라져도 NFP 를 한 번 더 계산할 뿐이다.
fn class_hash(s: &SPolygon, angles: &[f32]) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    s.vertices().len().hash(&mut h);
    for p in s.vertices().iter() {
        ((p.0 * 1024.0).round() as i64).hash(&mut h);
        ((p.1 * 1024.0).round() as i64).hash(&mut h);
    }
    angles.len().hash(&mut h);
    for a in angles {
        a.to_bits().hash(&mut h);
    }
    h.finish()
}

/// 키 오름차순 상위 `cap` 개만 남기고 그 안에서 정렬한다.
/// 전체 정렬 대신 `select_nth` 로 먼저 잘라 O(n) + O(cap log cap) 이다.
fn keep_smallest(cands: &mut Vec<(f32, f32, f32)>, cap: usize) {
    let cmp = |a: &(f32, f32, f32), b: &(f32, f32, f32)| {
        a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal)
    };
    if cap == 0 {
        cands.clear();
        return;
    }
    if cands.len() > cap {
        cands.select_nth_unstable_by(cap - 1, cmp);
        cands.truncate(cap);
    }
    cands.sort_unstable_by(cmp);
}

/// 사양에 명시된 진입점. 전체 단계를 수행한다.
pub fn build_initial(
    instance: &SPInstance,
    cfg: &AttachCfg,
    rng: &mut Xoshiro256PlusPlus,
) -> SPSolution {
    let cc = CompactCfg::default();
    build_initial_restarts(instance, cfg, &cc, Stage::G4, rng, cc.restart_count).0
}

/// 단계 지정 진입점. 하네스가 단계별 기여도를 재기 위해 쓴다.
pub fn build_initial_staged(
    instance: &SPInstance,
    cfg: &AttachCfg,
    stage: Stage,
    rng: &mut Xoshiro256PlusPlus,
) -> (SPSolution, AttachStats) {
    build_initial_staged_cc(instance, cfg, &CompactCfg::default(), stage, rng)
}

/// G4: 서로 다른 시드로 `restarts` 회 돌려 최선을 고른다.
pub fn build_initial_restarts(
    instance: &SPInstance,
    cfg: &AttachCfg,
    ccfg: &CompactCfg,
    stage: Stage,
    rng: &mut Xoshiro256PlusPlus,
    restarts: u32,
) -> (SPSolution, AttachStats) {
    let t0 = StdInstant::now();
    let mut best: Option<(SPSolution, AttachStats)> = None;
    for _ in 0..restarts.max(1) {
        let r = build_initial_staged_cc(instance, cfg, ccfg, stage, rng);
        let better = best
            .as_ref()
            .map_or(true, |(_, bs)| r.1.density > bs.density);
        if better {
            best = Some(r);
        }
    }
    let (sol, mut st) = best.expect(crate::dmsg!("restarts >= 1"));
    st.secs = t0.elapsed().as_secs_f64();
    (sol, st)
}

/// 단계 + 압축 설정 지정 진입점.
pub fn build_initial_staged_cc(
    instance: &SPInstance,
    cfg: &AttachCfg,
    ccfg: &CompactCfg,
    stage: Stage,
    rng: &mut Xoshiro256PlusPlus,
) -> (SPSolution, AttachStats) {
    let t0 = StdInstant::now();
    let mut eng = Engine::new(instance, cfg, stage);

    // ---- 1) 정렬: 정렬 키 내림차순 -------------------------------------
    let mut slots: Vec<usize> = Vec::with_capacity(instance.total_item_qty());
    for (id, (_item, qty)) in instance.items.iter().enumerate() {
        for _ in 0..*qty {
            slots.push(id);
        }
    }
    let keys: Vec<f32> = slots
        .iter()
        .map(|&id| {
            let k = eng.order_key(id);
            let j: f32 = rng.random::<f32>() - 0.5;
            k * (1.0 + ORDER_JITTER * j)
        })
        .collect();
    let mut idx: Vec<usize> = (0..slots.len()).collect();
    idx.sort_by(|&a, &b| {
        keys[b]
            .partial_cmp(&keys[a])
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(slots[a].cmp(&slots[b]))
    });
    let order: Vec<usize> = idx.iter().map(|&i| slots[i]).collect();

    // ---- 스트립 준비 ---------------------------------------------------
    let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
    let full_w = instance.item_area() / instance.base_strip.fixed_height;
    // 조각 수가 아주 적은 축퇴 인스턴스에서는 이론 폭×배수가 가장 넓은 조각보다
    // 좁을 수 있다 — 작업 폭에 최대 조각 폭 기반 하한을 둔다.
    let max_item_w = (0..instance.items.len())
        .map(|id| {
            let bb = eng.geom[id].base[0].bbox();
            bb.x_max - bb.x_min
        })
        .fold(0.0f32, f32::max);
    // 2C-C: 폭을 제약으로 걸 수 있다. None 이면 자유 폭.
    let work_w = cfg
        .fixed_width
        .unwrap_or((full_w * WORK_WIDTH_MULT).max(max_item_w * 2.0 + 8.0 * PLACE_EPS));
    prob.change_strip_width(work_w).expect(crate::dmsg!("strip width must be positive"));
    eng.init_raster(work_w);

    // ---- 2) 첫 배치: 좌하단 -------------------------------------------
    let mut placed: Vec<Placed> = Vec::with_capacity(order.len());
    let first_id = order[0];
    {
        let bb = eng.geom[first_id].base[0].bbox();
        let (tx, ty) = (-bb.x_min + PLACE_EPS, -bb.y_min + PLACE_EPS);
        let pk = prob.place_item(SPPlacement {
            item_idx: first_id,
            d_transf: DTransformation::new(eng.geom[first_id].angles[0], (tx, ty)),
        });
        placed.push(Placed {
            pkey: pk,
            item_id: first_id,
            rot: 0,
            bbox: shifted(&bb, tx, ty),
            tx,
            ty,
        });
        let base = eng.geom[first_id].base[0].clone();
        if let Some(r) = eng.raster.as_mut() {
            r.mark(&base, tx, ty);
        }
    }

    // ---- 3) 반복 -------------------------------------------------------
    let remaining: Vec<usize> = order[1..].to_vec();
    let n_slots = remaining.len();
    let cap = order.len();
    // (미배치 슬롯 s) x (배치된 조각 j) 별 최선 후보. 증분으로 채운다.
    let mut cand: Vec<Option<Cand>> = vec![None; n_slots * cap];
    let mut evaluated: Vec<usize> = vec![0; n_slots];
    let mut alive: Vec<bool> = vec![true; n_slots];
    let mut failed: Vec<usize> = Vec::new();
    let mut n_alive = n_slots;
    let mut positions: Vec<(f32, f32)> = Vec::with_capacity(cfg.cand_cap);

    // 2C-B: 정렬 순서대로 한 조각씩 배치하는 구조. 후보 쌍이 O(n^2) 로 줄어든다.
    if cfg.structure == Structure::Sorted {
        for s in 0..n_slots {
            let item_id = remaining[s];
            let ctx = score_ctx(&placed);
            let mut best: Option<(Cand, f32)> = None;
            for j in 0..placed.len() {
                if let Some(c) = eval_pair(
                    &mut eng,
                    &prob,
                    &placed,
                    j,
                    item_id,
                    &ctx,
                    work_w,
                    &mut positions,
                ) {
                    let bb = eng.geom[item_id].base[c.rot as usize].bbox();
                    let sc = eng.score(item_id, &bb, c.tx, c.ty, &ctx);
                    if best.map_or(true, |(_, cur)| sc < cur) {
                        best = Some((c, sc));
                    }
                }
            }
            let ok = match best {
                Some((c, _)) => try_push_placed(&mut eng, &mut prob, &mut placed, item_id, c),
                None => false,
            };
            if !ok {
                failed.push(s);
            }
            alive[s] = false;
        }
        n_alive = 0;
    }

    while n_alive > 0 {
        let ctx = score_ctx(&placed);

        // 선택 창: 정렬 순서 상위 `pool_window` 개의 살아 있는 슬롯까지만 본다.
        let window_end = if cfg.pool_window == 0 {
            n_slots
        } else {
            let mut seen = 0usize;
            let mut end = n_slots;
            for s in 0..n_slots {
                if alive[s] {
                    seen += 1;
                    if seen == cfg.pool_window {
                        end = s + 1;
                        break;
                    }
                }
            }
            end
        };

        // 3a) 새 배치 대상에 대한 후보만 증분 계산
        for s in 0..window_end {
            if !alive[s] {
                continue;
            }
            let item_id = remaining[s];
            for j in evaluated[s]..placed.len() {
                cand[s * cap + j] = eval_pair(
                    &mut eng,
                    &prob,
                    &placed,
                    j,
                    item_id,
                    &ctx,
                    work_w,
                    &mut positions,
                );
            }
            evaluated[s] = placed.len();
        }

        // 3b) 현재 ctx 로 다시 채점하고 전역 최선 하나 고르기.
        //     자리(j)는 배치 점수로, 조각(s)은 선택 점수로 고른다 — 두 결정을 분리한다.
        let mut pick: Option<(usize, usize, f32)> = None; // (s, j, select score)
        for s in 0..window_end {
            if !alive[s] {
                continue;
            }
            let item_id = remaining[s];
            let mut best_j: Option<(usize, f32)> = None;
            for j in 0..placed.len() {
                let Some(c) = cand[s * cap + j] else { continue };
                let bb = eng.geom[item_id].base[c.rot as usize].bbox();
                let sc = eng.score(item_id, &bb, c.tx, c.ty, &ctx);
                if best_j.map_or(true, |(_, best)| sc < best) {
                    best_j = Some((j, sc));
                }
            }
            let Some((j, _)) = best_j else { continue };
            let c = cand[s * cap + j].unwrap();
            let bb = eng.geom[item_id].base[c.rot as usize].bbox();
            let sel = eng.select_score(item_id, &bb, c.tx, c.ty, &ctx);
            if pick.map_or(true, |(_, _, best)| sel < best) {
                pick = Some((s, j, sel));
            }
        }

        let Some((s, j, _)) = pick else {
            for s in 0..n_slots {
                if alive[s] {
                    alive[s] = false;
                    failed.push(s);
                }
            }
            break;
        };

        let c = cand[s * cap + j].unwrap();
        let item_id = remaining[s];
        // 캐시된 후보가 그 사이 막혔을 수 있다 -> 재검증. 막혔으면 그 쌍만 다시 계산.
        if eng.collides(&prob, item_id, c.rot as usize, c.tx, c.ty) {
            cand[s * cap + j] = eval_pair(
                &mut eng,
                &prob,
                &placed,
                j,
                item_id,
                &ctx,
                work_w,
                &mut positions,
            );
            continue;
        }
        if !try_push_placed(&mut eng, &mut prob, &mut placed, item_id, c) {
            // 등록 도형 재검사 실패. eval_pair 는 결정적이라 다시 계산하면 같은 후보가
            // 나와 무한 루프가 된다 -> 이 (슬롯, 자리) 후보를 버리고 다른 자리로 간다.
            cand[s * cap + j] = None;
            continue;
        }
        alive[s] = false;
        n_alive -= 1;
    }

    // ---- 4) 실패 아이템 2차 패스 (부착 대상 전체 재평가) ---------------
    if !failed.is_empty() {
        let still: Vec<usize> = failed.clone();
        failed.clear();
        for s in still {
            let item_id = remaining[s];
            let ctx = score_ctx(&placed);
            let mut b: Option<(Cand, f32)> = None;
            for j in 0..placed.len() {
                if let Some(c) = eval_pair(
                    &mut eng,
                    &prob,
                    &placed,
                    j,
                    item_id,
                    &ctx,
                    work_w,
                    &mut positions,
                ) {
                    let bb = eng.geom[item_id].base[c.rot as usize].bbox();
                    let sc = eng.score(item_id, &bb, c.tx, c.ty, &ctx);
                    if b.map_or(true, |(_, cur)| sc < cur) {
                        b = Some((c, sc));
                    }
                }
            }
            // 등록 도형 재검사에 실패하면 꼬리 강제 배치로 넘긴다 (조용히 버리지 않는다).
            let ok = match b {
                Some((c, _)) if !eng.collides(&prob, item_id, c.rot as usize, c.tx, c.ty) => {
                    try_push_placed(&mut eng, &mut prob, &mut placed, item_id, c)
                }
                _ => false,
            };
            if ok {
                eng.stats.second_pass += 1;
            } else {
                failed.push(s);
            }
        }
    }

    // ---- 4b) 그래도 실패하면 꼬리에 강제로 붙인다 ----------------------
    for s in failed {
        let item_id = remaining[s];
        if let Some((rot, tx, ty)) = tail_append(&mut eng, &prob, &placed, item_id, work_w) {
            let c = Cand {
                rot: rot as u16,
                tx,
                ty,
            };
            if try_push_placed(&mut eng, &mut prob, &mut placed, item_id, c) {
                eng.stats.tail_appends += 1;
            } else {
                eng.stats.unrepairable += 1;
            }
        }
    }

    // ---- 4d) 2C-D: 미배치 조각 강제 삽입 -------------------------------
    if cfg.force_insert {
        let mut unplaced: Vec<usize> = prob
            .item_demand_qtys()
            .iter()
            .enumerate()
            .filter(|(_, q)| **q > 0)
            .flat_map(|(id, q)| std::iter::repeat(id).take(*q))
            .collect();
        // 면적 내림차순
        unplaced.sort_by(|a, b| {
            eng.area[*b]
                .partial_cmp(&eng.area[*a])
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let deadline =
            StdInstant::now() + std::time::Duration::from_millis(cfg.insert_time_budget_ms);
        for u in unplaced {
            if force_insert_one(
                &mut eng,
                &mut prob,
                &mut placed,
                u,
                work_w,
                cfg.insert_depth,
                deadline,
                &mut positions,
            ) {
                eng.stats.inserted += 1;
            } else {
                eng.stats.insert_fail_ids.push(u);
            }
        }
    }

    // ---- 4c) 최종 감사 & 복구 -----------------------------------------
    // 삽입 시점 CDE 질의가 통과했더라도, 이후 quadtree 재분할로 경계 축퇴
    // 케이스의 판정이 뒤집힐 수 있다. 검증식과 같은 술어로 다시 훑는다.
    repair(&mut eng, &mut prob, &mut placed);

    // ---- 5) 폭 정리 후 저장 --------------------------------------------
    eng.stats.self_collisions = count_collisions(&prob);
    if prob.n_placed_items() > 0 {
        prob.fit_strip().expect(crate::dmsg!("strip width must be positive"));
    }
    let mut sol = prob.save();
    eng.sync_cache_stats();
    eng.stats.unplaced_ids = prob
        .item_demand_qtys()
        .iter()
        .enumerate()
        .filter(|(_, q)| **q > 0)
        .flat_map(|(id, q)| std::iter::repeat(id).take(*q))
        .collect();
    let mut stats = eng.stats.clone();
    stats.placed = prob.n_placed_items();
    stats.total = instance.total_item_qty();
    stats.density = sol.density() * 100.0;
    stats.width = sol.strip_width();

    // ---- G) 반복 압축 -------------------------------------------------
    if stage.use_compaction() {
        let dens_before = stats.density;
        let mut cc = *ccfg;
        match stage {
            Stage::G1 => {
                cc.edge_halfplanes = false;
                cc.rollback = false;
            }
            Stage::G2 => {
                cc.edge_halfplanes = true;
                cc.rollback = false;
            }
            _ => {
                cc.edge_halfplanes = true;
                cc.rollback = true;
            }
        }
        let iters = if stage == Stage::G1 { 1 } else { cc.max_iters };
        let (nsol, cst) = compact::compact(instance, &sol, &cc, iters);
        let nd = nsol.density() * 100.0;
        if nd >= stats.density {
            sol = nsol;
            stats.density = nd;
            stats.width = sol.strip_width();
        }
        stats.lp_fail = cst.lp_fail;
        stats.lp_iters_used = cst.lp_iters_used;
        stats.lp_interrupted = cst.lp_interrupted;
        stats.compact_gain_pp = stats.density - dens_before;
        stats.lp_rows = cst.rows;
        stats.lp_pairs = cst.pairs;
        stats.lp_skipped_pairs = cst.skipped_pairs;
        stats.lp_applied = cst.applied;
        stats.lp_reverted = cst.reverted;
        stats.lp_frozen_pairs = cst.frozen_pairs;
        stats.blocking_item = cst.blocking_item;
        stats.blocking_travel = cst.blocking_travel;
    }

    stats.secs = t0.elapsed().as_secs_f64();
    (sol, stats)
}

/// 배치하고, **실제 등록 도형**으로 겹침을 재확인한다.
/// 후보 검사에 쓰는 변환 버퍼와 등록 도형은 연산 순서가 달라 마지막 비트가 어긋난다.
/// 겹치면 되돌리고 false 를 돌려준다.
fn try_push_placed(
    eng: &mut Engine,
    prob: &mut SPProblem,
    placed: &mut Vec<Placed>,
    item_id: usize,
    c: Cand,
) -> bool {
    let rot = c.rot as usize;
    let bb = eng.geom[item_id].base[rot].bbox();
    let pk = prob.place_item(SPPlacement {
        item_idx: item_id,
        d_transf: DTransformation::new(eng.geom[item_id].angles[rot], (c.tx, c.ty)),
    });
    if crate::util::parts::placed_collides(
        prob.layout().cde(),
        &prob.layout().placed_items()[pk],
        &crate::util::parts::self_filter(pk),
    ) {
        prob.remove_item(pk);
        eng.stats.place_reverted += 1;
        return false;
    }
    placed.push(Placed {
        pkey: pk,
        item_id,
        rot,
        bbox: shifted(&bb, c.tx, c.ty),
        tx: c.tx,
        ty: c.ty,
    });
    let base = eng.geom[item_id].base[rot].clone();
    if let Some(r) = eng.raster.as_mut() {
        r.mark(&base, c.tx, c.ty);
    }
    true
}

fn repair(eng: &mut Engine, prob: &mut SPProblem, placed: &mut Vec<Placed>) {
    for _ in 0..REPAIR_ROUNDS {
        let bad: Vec<PItemKey> = prob
            .layout()
            .placed_items()
            .iter()
            .filter(|(pk, pi)| {
                crate::util::parts::placed_collides(
                    prob.layout().cde(),
                    pi,
                    &crate::util::parts::self_filter(*pk),
                )
            })
            .map(|(pk, _)| pk)
            .collect();
        if bad.is_empty() {
            break;
        }
        eng.stats.repaired += bad.len();
        for pk in bad {
            let pl = prob.remove_item(pk);
            placed.retain(|p| p.pkey != pk);
            let item_id = pl.item_idx;
            let work = prob.strip_width();
            let pushed = match tail_append(eng, prob, placed, item_id, work) {
                Some((rot, tx, ty)) => {
                    let c = Cand {
                        rot: rot as u16,
                        tx,
                        ty,
                    };
                    try_push_placed(eng, prob, placed, item_id, c)
                }
                None => false,
            };
            if !pushed {
                eng.stats.unrepairable += 1;
            }
        }
    }
}

/// 진단용: 현재 layout 에서 서로 겹치는 조각 수.
fn count_collisions(prob: &SPProblem) -> usize {
    let layout = &prob.layout();
    let mut n = 0usize;
    for (pk, pi) in layout.placed_items().iter() {
        if crate::util::parts::placed_collides(layout.cde(), pi, &crate::util::parts::self_filter(pk)) {
            n += 1;
        }
    }
    n
}

fn shifted(bb: &Rect, tx: f32, ty: f32) -> Rect {
    Rect {
        x_min: bb.x_min + tx,
        y_min: bb.y_min + ty,
        x_max: bb.x_max + tx,
        y_max: bb.y_max + ty,
    }
}

fn score_ctx(placed: &[Placed]) -> ScoreCtx {
    if placed.is_empty() {
        return ScoreCtx {
            cur_len: 0.0,
            bx0: 0.0,
            by0: 0.0,
            bx1: 0.0,
            by1: 0.0,
        };
    }
    let mut cur_len = 0.0f32;
    let (mut bx0, mut by0) = (f32::MAX, f32::MAX);
    let (mut bx1, mut by1) = (f32::MIN, f32::MIN);
    for p in placed {
        cur_len = cur_len.max(p.bbox.x_max);
        bx0 = bx0.min(p.bbox.x_min);
        by0 = by0.min(p.bbox.y_min);
        bx1 = bx1.max(p.bbox.x_max);
        by1 = by1.max(p.bbox.y_max);
    }
    ScoreCtx {
        cur_len,
        bx0,
        by0,
        bx1,
        by1,
    }
}

/// (미배치 아이템, 배치된 조각 j) 쌍의 최선 후보.
#[allow(clippy::too_many_arguments)]
fn eval_pair(
    eng: &mut Engine,
    prob: &SPProblem,
    placed: &[Placed],
    j: usize,
    item_id: usize,
    ctx: &ScoreCtx,
    work_w: f32,
    positions: &mut Vec<(f32, f32)>,
) -> Option<Cand> {
    let n_rot = eng.geom[item_id].base.len();
    // (점수 하한, tx, ty, rot). 하한 오름차순으로 훑는다.
    // 슬라이딩 후 채점을 쓰면 하한은 더 이상 유효한 가지치기 경계가 아니다
    // (슬라이딩이 좌표를 왼쪽/아래로 옮겨 점수를 더 낮춘다).
    // 순서만 하한으로 잡고, 종료는 `cand_eval_cap` 이 담당한다.
    let mut cands: Vec<(f32, f32, f32, u16)> = Vec::with_capacity(eng.cfg.cand_cap);
    for rot in 0..n_rot {
        positions.clear();
        gen_positions(eng, placed[j], item_id, rot, positions);
        eng.stats.cand_generated += positions.len() as u64;
        let bb = eng.geom[item_id].base[rot].bbox();
        for k in 0..positions.len() {
            let (tx, ty) = positions[k];
            if !eng.inside_strip(&bb, tx, ty, work_w) {
                continue;
            }
            cands.push((eng.score_lb(&bb, tx, ty, ctx), tx, ty, rot as u16));
        }
    }
    if cands.is_empty() {
        return None;
    }

    let after_slide = eng.stage.use_slide() && eng.cfg.score_after_slide;
    let eval_cap = eng.cfg.cand_eval_cap.max(1);
    let mut best: Option<(Cand, f32)> = None;
    // 슬라이딩을 쓰지 않을 때만 쓰는 상위 K 버퍼
    let mut top: Vec<(Cand, f32)> = Vec::with_capacity(eval_cap + 1);
    // 이미 CDE로 검사한 슬라이딩 **전** 위치 (좌표 반올림 키로 중복 제거)
    let mut tested: Vec<(f32, f32, u16)> = Vec::with_capacity(CDE_BUDGET.min(256) + 8);
    // 이미 채점한 슬라이딩 **후** 위치
    let mut final_pos: Vec<(f32, f32, u16)> = Vec::with_capacity(eval_cap + 8);
    let mut evaluated = 0usize;

    // 전체 정렬 대신 청크 단위 부분선택. 하한 오름차순 순서는 그대로 유지된다.
    let cmp = |a: &(f32, f32, f32, u16), b: &(f32, f32, f32, u16)| {
        a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal)
    };
    let mut start = 0usize;
    let mut budget = CDE_BUDGET;
    let total = cands.len();
    'chunks: while start < total {
        let end = (start + LB_CHUNK).min(total);
        if end < total {
            cands[start..].select_nth_unstable_by(end - start - 1, cmp);
        }
        cands[start..end].sort_unstable_by(cmp);
        for k in start..end {
            let (lb, tx, ty, rot) = cands[k];
            if evaluated >= eval_cap {
                break 'chunks;
            }
            if !after_slide {
                // 슬라이딩 전 점수로 가지치기 (기존 C단계 경로)
                if let Some((_, cur)) = top.first() {
                    if lb >= *cur {
                        break 'chunks;
                    }
                }
            }
            if budget == 0 {
                eng.stats.budget_hits += 1;
                eng.stats.budget_dropped += (total - k) as u64;
                break 'chunks;
            }
            if tested.iter().any(|&(x, y, r)| {
                r == rot && (x - tx).abs() < DEDUP_TOL && (y - ty).abs() < DEDUP_TOL
            }) {
                continue;
            }
            tested.push((tx, ty, rot));
            budget -= 1;
            if eng.collides(prob, item_id, rot as usize, tx, ty) {
                continue;
            }
            let bb = eng.geom[item_id].base[rot as usize].bbox();

            if !after_slide {
                let sc = eng.score(item_id, &bb, tx, ty, ctx);
                let c = Cand { rot, tx, ty };
                let pos = top.partition_point(|(_, s)| *s < sc);
                top.insert(pos, (c, sc));
                top.truncate(eval_cap);
                evaluated += 1;
                continue;
            }

            // 2C-A: 이 후보를 슬라이딩하고, 슬라이딩 **후** 좌표로 채점한다.
            let (nx, ny) = slide(eng, prob, item_id, rot as usize, tx, ty);
            eng.stats.slides += 1;
            let (nx, ny) = if (nx - tx).abs() < 1e-6 && (ny - ty).abs() < 1e-6 {
                (tx, ty)
            } else if eng.collides(prob, item_id, rot as usize, nx, ny) {
                // 이분 탐색이 접촉 극한까지 민 위치가 실제로는 겹친다 -> 슬라이딩 취소
                eng.stats.slide_cancelled += 1;
                (tx, ty)
            } else {
                (nx, ny)
            };
            // 슬라이딩 후 좌표로 중복 제거. 중복은 평가 횟수에 세지 않는다.
            if final_pos.iter().any(|&(x, y, r)| {
                r == rot && (x - nx).abs() < DEDUP_TOL && (y - ny).abs() < DEDUP_TOL
            }) {
                eng.stats.dedup_skipped += 1;
                continue;
            }
            final_pos.push((nx, ny, rot));
            evaluated += 1;
            eng.stats.cands_evaluated += 1;
            let sc = eng.score(item_id, &bb, nx, ny, ctx);
            if best.map_or(true, |(_, cur)| sc < cur) {
                best = Some((
                    Cand {
                        rot,
                        tx: nx,
                        ty: ny,
                    },
                    sc,
                ));
            }
        }
        start = end;
    }

    if after_slide {
        return best.map(|(c, _)| c);
    }
    if !eng.stage.use_slide() {
        return top.first().map(|(c, _)| *c);
    }
    // C단계(기존 경로): 상위 후보를 밀착시키고 다시 채점한다.
    let mut b2: Option<(Cand, f32)> = None;
    for (c, _) in top.iter() {
        let (nx, ny) = slide(eng, prob, item_id, c.rot as usize, c.tx, c.ty);
        eng.stats.slides += 1;
        let bb = eng.geom[item_id].base[c.rot as usize].bbox();
        eng.stats.slide_gain_total += ((c.tx - nx) + (c.ty - ny)) as f64;
        let sc = eng.score(item_id, &bb, nx, ny, ctx);
        let nc = Cand {
            rot: c.rot,
            tx: nx,
            ty: ny,
        };
        if b2.map_or(true, |(_, cur)| sc < cur) {
            b2 = Some((nc, sc));
        }
    }
    b2.map(|(c, _)| c)
}

/// C단계: 한 축으로 이분 탐색해 밀착시킨 이동량을 돌려준다.
/// `lo` 는 항상 실현 가능(충돌 없음)이라는 불변식을 유지하므로 결과도 실현 가능하다.
#[allow(clippy::too_many_arguments)]
fn slide_axis(
    eng: &mut Engine,
    prob: &SPProblem,
    item_id: usize,
    rot: usize,
    tx: f32,
    ty: f32,
    axis_x: bool,
    max_travel: f32,
    iters: u32,
) -> f32 {
    if max_travel <= SLIDE_TOL {
        return 0.0;
    }
    let probe = |eng: &mut Engine, d: f32| -> bool {
        let (nx, ny) = if axis_x { (tx - d, ty) } else { (tx, ty - d) };
        !eng.collides(prob, item_id, rot, nx, ny)
    };
    if probe(eng, max_travel) {
        return max_travel;
    }
    let (mut lo, mut hi) = (0.0f32, max_travel);
    for _ in 0..iters {
        if hi - lo < SLIDE_TOL {
            break;
        }
        let mid = 0.5 * (lo + hi);
        if probe(eng, mid) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo
}

/// -x, -y 순서로 밀착시킨다.
fn slide(
    eng: &mut Engine,
    prob: &SPProblem,
    item_id: usize,
    rot: usize,
    tx: f32,
    ty: f32,
) -> (f32, f32) {
    let iters = eng.cfg.slide_iters;
    let bb = eng.geom[item_id].base[rot].bbox();
    let max_x = (bb.x_min + tx) - PLACE_EPS;
    let dx = slide_axis(eng, prob, item_id, rot, tx, ty, true, max_x, iters);
    let tx2 = tx - dx;
    let max_y = (bb.y_min + ty) - PLACE_EPS;
    let dy = slide_axis(eng, prob, item_id, rot, tx2, ty, false, max_y, iters);
    (tx2, ty - dy)
}

/// 후보 위치 생성. B단계 이상은 NFP 경계 정점(+에지 등분점)을 쓴다.
fn gen_positions(
    eng: &mut Engine,
    tgt: Placed,
    item_id: usize,
    rot: usize,
    out: &mut Vec<(f32, f32)>,
) {
    if !eng.stage.use_nfp() {
        // A단계 임시 코드: bbox 앵커.
        let ib = eng.geom[item_id].base[rot].bbox();
        let w = ib.x_max - ib.x_min;
        let h = ib.y_max - ib.y_min;
        let tb = tgt.bbox;
        let e = PLACE_EPS;
        let anchors = [
            (tb.x_max + e, tb.y_min),
            (tb.x_max + e, tb.y_max - h),
            (tb.x_min - w - e, tb.y_min),
            (tb.x_min - w - e, tb.y_max - h),
            (tb.x_min, tb.y_max + e),
            (tb.x_max - w, tb.y_max + e),
            (tb.x_min, tb.y_min - h - e),
            (tb.x_max - w, tb.y_min - h - e),
        ];
        for (x, y) in anchors {
            out.push((x - ib.x_min, y - ib.y_min));
        }
        return;
    }

    // 고정 도형 로컬 프레임의 NFP -> 세계 좌표로 옮긴다.
    // 후보 점은 캐시에 이미 (부풀림 + 합집합 내부 제거 + 에지 등분) 상태로 들어있다.
    let (ox, oy) = (tgt.tx, tgt.ty);
    let nfp_arc = eng.nfp_for(&tgt, item_id, rot);
    out.reserve(nfp_arc.cands.len());
    for p in nfp_arc.cands.iter() {
        out.push((p.0 + ox, p.1 + oy));
    }
}

/// 배치된 조각 전체를 대상으로 이 아이템의 최선 위치를 찾는다 (정렬 순서 구조의 내부 루프).
fn best_position(
    eng: &mut Engine,
    prob: &SPProblem,
    placed: &[Placed],
    item_id: usize,
    work_w: f32,
    positions: &mut Vec<(f32, f32)>,
) -> Option<Cand> {
    let ctx = score_ctx(placed);
    let mut best: Option<(Cand, f32)> = None;
    for j in 0..placed.len() {
        if let Some(c) = eval_pair(eng, prob, placed, j, item_id, &ctx, work_w, positions) {
            let bb = eng.geom[item_id].base[c.rot as usize].bbox();
            let sc = eng.score(item_id, &bb, c.tx, c.ty, &ctx);
            if best.map_or(true, |(_, cur)| sc < cur) {
                best = Some((c, sc));
            }
        }
    }
    best.map(|(c, _)| c)
}

/// 2C-D 후보 하나: 위치 + 그 자리를 막고 있는 조각들 + 비용.
struct RuinCand {
    cand: Cand,
    blockers: Vec<PItemKey>,
    n: usize,
    area: f32,
}

/// 이 아이템을 놓을 수 있는 후보들을, 막고 있는 조각 비용이 작은 순으로 뽑는다.
/// 비용 = 겹치는 조각 수, 동률이면 겹치는 bbox 교차 면적 합.
fn ruin_candidates(
    eng: &mut Engine,
    prob: &SPProblem,
    placed: &[Placed],
    item_id: usize,
    work_w: f32,
    try_cap: usize,
    scan_cap: usize,
    positions: &mut Vec<(f32, f32)>,
) -> Vec<RuinCand> {
    let n_rot = eng.geom[item_id].base.len();
    let ctx = score_ctx(placed);
    let mut cands: Vec<(f32, f32, f32, u16)> = Vec::new();
    for j in 0..placed.len() {
        for rot in 0..n_rot {
            positions.clear();
            gen_positions(eng, placed[j], item_id, rot, positions);
            let bb = eng.geom[item_id].base[rot].bbox();
            for k in 0..positions.len() {
                let (tx, ty) = positions[k];
                if !eng.inside_strip(&bb, tx, ty, work_w) {
                    continue;
                }
                cands.push((eng.score_lb(&bb, tx, ty, &ctx), tx, ty, rot as u16));
            }
        }
    }
    if cands.is_empty() {
        return Vec::new();
    }
    let cmp = |a: &(f32, f32, f32, u16), b: &(f32, f32, f32, u16)| {
        a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal)
    };
    let take = scan_cap.min(cands.len());
    if take < cands.len() {
        cands.select_nth_unstable_by(take - 1, cmp);
    }
    cands.truncate(take);
    cands.sort_unstable_by(cmp);

    let mut out: Vec<RuinCand> = Vec::new();
    let mut blockers: Vec<PItemKey> = Vec::new();
    let mut tested: Vec<(f32, f32, u16)> = Vec::new();
    for &(_, tx, ty, rot) in cands.iter() {
        if tested.iter().any(|&(x, y, r)| {
            r == rot && (x - tx).abs() < DEDUP_TOL && (y - ty).abs() < DEDUP_TOL
        }) {
            continue;
        }
        tested.push((tx, ty, rot));
        if !eng.collect_overlaps(prob, item_id, rot as usize, tx, ty, None, &mut blockers) {
            continue; // 스트립 밖
        }
        let bb = shifted(&eng.geom[item_id].base[rot as usize].bbox(), tx, ty);
        let mut area = 0.0f32;
        for pk in blockers.iter() {
            if let Some(p) = placed.iter().find(|p| p.pkey == *pk) {
                let ix = (bb.x_max.min(p.bbox.x_max) - bb.x_min.max(p.bbox.x_min)).max(0.0);
                let iy = (bb.y_max.min(p.bbox.y_max) - bb.y_min.max(p.bbox.y_min)).max(0.0);
                area += ix * iy;
            }
        }
        out.push(RuinCand {
            cand: Cand { rot, tx, ty },
            blockers: blockers.clone(),
            n: blockers.len(),
            area,
        });
    }
    out.sort_by(|a, b| {
        a.n.cmp(&b.n)
            .then(a.area.partial_cmp(&b.area).unwrap_or(std::cmp::Ordering::Equal))
    });
    out.truncate(try_cap);
    out
}

/// 2C-D: 막고 있는 조각을 들어내고 u 를 **먼저** 놓은 뒤 다시 채운다.
/// 성공하면 true. 실패하면 스냅샷으로 원자적으로 되돌린다.
#[allow(clippy::too_many_arguments)]
fn force_insert_one(
    eng: &mut Engine,
    prob: &mut SPProblem,
    placed: &mut Vec<Placed>,
    item_id: usize,
    work_w: f32,
    depth: u32,
    deadline: StdInstant,
    positions: &mut Vec<(f32, f32)>,
) -> bool {
    if StdInstant::now() >= deadline {
        return false;
    }
    let try_cap = eng.cfg.insert_try_cap.max(1);
    let cands = ruin_candidates(
        eng, prob, placed, item_id, work_w, try_cap, INSERT_SCAN_CAP, positions,
    );
    for rc in cands.iter() {
        if StdInstant::now() >= deadline {
            return false;
        }
        eng.stats.insert_attempts += 1;
        // 스냅샷: 문제 상태 + 엔진이 들고 있는 배치 기록을 함께 뜬다.
        let snap_sol = prob.save();
        let snap_placed = placed.clone();

        // ruin — 막고 있는 조각을 들어낸다
        let mut ruined: Vec<usize> = Vec::with_capacity(rc.blockers.len());
        for pk in rc.blockers.iter() {
            if placed.iter().any(|p| p.pkey == *pk) {
                let pl = prob.remove_item(*pk);
                placed.retain(|p| p.pkey != *pk);
                ruined.push(pl.item_idx);
            }
        }
        eng.stats.ruined_total += ruined.len();

        // u 를 **먼저** 놓는다 — 순서 변화가 유일한 지렛대다
        if !try_push_placed(eng, prob, placed, item_id, rc.cand) {
            prob.restore(&snap_sol);
            *placed = snap_placed;
            eng.stats.insert_reverted += 1;
            continue;
        }

        // recreate — 들어낸 조각을 면적 내림차순으로 다시 놓는다
        ruined.sort_by(|a, b| {
            eng.area[*b]
                .partial_cmp(&eng.area[*a])
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let mut leftover: Vec<usize> = Vec::new();
        for rid in ruined.iter() {
            let ok = match best_position(eng, prob, placed, *rid, work_w, positions) {
                Some(c) => try_push_placed(eng, prob, placed, *rid, c),
                None => false,
            };
            if !ok {
                leftover.push(*rid);
            }
        }

        // recreate 에서 또 미배치가 생기면 같은 절차를 재귀 적용한다
        if !leftover.is_empty() && depth > 0 {
            let mut still: Vec<usize> = Vec::new();
            for lid in leftover.iter() {
                if !force_insert_one(
                    eng, prob, placed, *lid, work_w, depth - 1, deadline, positions,
                ) {
                    still.push(*lid);
                }
            }
            leftover = still;
        }

        if leftover.is_empty() {
            return true;
        }
        prob.restore(&snap_sol);
        *placed = snap_placed;
        eng.stats.insert_reverted += 1;
    }
    false
}

// ===========================================================================
// 2C-E — 겹침 허용 분리
// ===========================================================================

/// 검증 없이 그대로 놓는다. 2C-E 는 **겹친 상태를 중간 단계로 허용한다.**
fn place_raw(
    eng: &Engine,
    prob: &mut SPProblem,
    placed: &mut Vec<Placed>,
    item_id: usize,
    c: Cand,
) {
    let rot = c.rot as usize;
    let bb = eng.geom[item_id].base[rot].bbox();
    let pk = prob.place_item(SPPlacement {
        item_idx: item_id,
        d_transf: DTransformation::new(eng.geom[item_id].angles[rot], (c.tx, c.ty)),
    });
    placed.push(Placed {
        pkey: pk,
        item_id,
        rot,
        bbox: shifted(&bb, c.tx, c.ty),
        tx: c.tx,
        ty: c.ty,
    });
}

#[inline(always)]
fn pair_key(a: usize, b: usize) -> (usize, usize) {
    if a < b { (a, b) } else { (b, a) }
}

fn bbox_overlap_area(a: &Rect, b: &Rect) -> f32 {
    let ix = (a.x_max.min(b.x_max) - a.x_min.max(b.x_min)).max(0.0);
    let iy = (a.y_max.min(b.y_max) - a.y_min.max(b.y_min)).max(0.0);
    ix * iy
}

/// 한 조각이 지금 자리에서 물고 있는 벌점.
/// 실제로 겹치는 쌍만 세고, 크기는 bbox 교차 면적으로 잰다.
/// `(가중, 원시)` 를 함께 돌려준다 — 가중치는 **탐색 유도에만** 쓰고
/// 개선 판정과 종료 판정은 원시값으로 한다(가중치가 커지면 총합이 항상 늘어난다).
/// 겹치는 쌍이 없으면 둘 다 정확히 0 이다.
#[allow(clippy::too_many_arguments)]
fn piece_penalty(
    eng: &mut Engine,
    prob: &SPProblem,
    placed: &[Placed],
    item_id: usize,
    rot: usize,
    tx: f32,
    ty: f32,
    weights: &HashMap<(usize, usize), f32>,
    exclude: Option<PItemKey>,
    scratch: &mut Vec<PItemKey>,
) -> (f64, f64) {
    if !eng.collect_overlaps(prob, item_id, rot, tx, ty, exclude, scratch) {
        // 스트립 밖 — 아주 큰 벌점으로 막는다
        return (1.0e12, 1.0e12);
    }
    let bb = shifted(&eng.geom[item_id].base[rot].bbox(), tx, ty);
    let (mut w_sum, mut raw) = (0.0f64, 0.0f64);
    for pk in scratch.iter() {
        let Some(o) = placed.iter().find(|q| q.pkey == *pk) else {
            continue;
        };
        let a = bbox_overlap_area(&bb, &o.bbox).max(1.0) as f64;
        let w = *weights.get(&pair_key(item_id, o.item_id)).unwrap_or(&1.0) as f64;
        w_sum += a * w;
        raw += a;
    }
    (w_sum, raw)
}

pub struct SeparateReport {
    pub penalty_start: f64,
    pub penalty_end: f64,
    pub iters: usize,
    pub resolved: bool,
    pub placed_all: bool,
}

/// 후보 위치를 만들 때 대상으로 삼는 이웃 조각 수. 전부 훑으면 반복당 15000+ 위치라 느리다.
const SEP_NEIGHBORS: usize = 24;

/// 폭을 고정한 채 전 조각을 겹친 상태로 놓고, 겹침을 반복적으로 푼다.
#[allow(clippy::too_many_arguments)]
fn separate(
    eng: &mut Engine,
    prob: &mut SPProblem,
    placed: &mut Vec<Placed>,
    work_w: f32,
    samples: usize,
    no_improve_limit: usize,
    growth: f32,
    rng: &mut Xoshiro256PlusPlus,
    deadline: StdInstant,
    positions: &mut Vec<(f32, f32)>,
) -> SeparateReport {
    let mut weights: HashMap<(usize, usize), f32> = HashMap::new();
    let mut scratch: Vec<PItemKey> = Vec::new();

    // ---- 1) 미배치 조각을 겹침 허용으로 전부 놓는다 (비용 최소 위치) ----
    let unplaced: Vec<usize> = prob
        .item_demand_qtys()
        .iter()
        .enumerate()
        .filter(|(_, q)| **q > 0)
        .flat_map(|(id, q)| std::iter::repeat(id).take(*q))
        .collect();
    for u in unplaced {
        let rc = ruin_candidates(eng, prob, placed, u, work_w, 1, INSERT_SCAN_CAP, positions);
        match rc.first() {
            Some(r) => place_raw(eng, prob, placed, u, r.cand),
            None => {
                let bb = eng.geom[u].base[0].bbox();
                place_raw(
                    eng,
                    prob,
                    placed,
                    u,
                    Cand {
                        rot: 0,
                        tx: -bb.x_min + PLACE_EPS,
                        ty: -bb.y_min + PLACE_EPS,
                    },
                );
            }
        }
    }
    let placed_all = prob.n_placed_items() == prob.instance().total_item_qty();

    // ---- 2) 벌점 반복 감소 (가중치는 유도, 판정은 원시값) ---------------
    let mut pw: Vec<f64> = Vec::with_capacity(placed.len());
    let mut praw: Vec<f64> = Vec::with_capacity(placed.len());
    macro_rules! recompute {
        () => {{
            pw.clear();
            praw.clear();
            for k in 0..placed.len() {
                let p = placed[k];
                let (a, b) = piece_penalty(
                    eng,
                    prob,
                    placed,
                    p.item_id,
                    p.rot,
                    p.tx,
                    p.ty,
                    &weights,
                    Some(p.pkey),
                    &mut scratch,
                );
                pw.push(a);
                praw.push(b);
            }
        }};
    }
    recompute!();
    let penalty_start = praw.iter().sum::<f64>() * 0.5;
    let mut best_raw = penalty_start;
    let mut no_improve = 0usize;
    let mut iters = 0usize;

    while StdInstant::now() < deadline {
        iters += 1;
        let raw_total = praw.iter().sum::<f64>() * 0.5;
        if raw_total <= 0.0 {
            return SeparateReport {
                penalty_start,
                penalty_end: 0.0,
                iters,
                resolved: true,
                placed_all,
            };
        }
        // 가중 벌점이 가장 큰 조각 (동률은 무작위)
        let mut k_worst = 0usize;
        let mut ties = 1usize;
        for k in 1..pw.len() {
            if pw[k] > pw[k_worst] {
                k_worst = k;
                ties = 1;
            } else if pw[k] == pw[k_worst] {
                ties += 1;
                if rng.random_range(0..ties) == 0 {
                    k_worst = k;
                }
            }
        }
        let target = placed[k_worst];

        // 그 조각을 빼고 후보 위치를 만든다 (조각이 서로를 통과할 수 있어야 한다)
        prob.remove_item(target.pkey);
        placed.remove(k_worst);

        let mut cands: Vec<(f32, f32, u16)> = Vec::new();
        let n_rot = eng.geom[target.item_id].base.len();
        let np = placed.len();
        let step = (np / SEP_NEIGHBORS).max(1);
        let off = if np > 0 { rng.random_range(0..np) } else { 0 };
        let mut t = 0usize;
        while t < np.min(SEP_NEIGHBORS * step) {
            let j = (off + t) % np.max(1);
            for rot in 0..n_rot {
                positions.clear();
                gen_positions(eng, placed[j], target.item_id, rot, positions);
                let bb = eng.geom[target.item_id].base[rot].bbox();
                for k in 0..positions.len() {
                    let (tx, ty) = positions[k];
                    if eng.inside_strip(&bb, tx, ty, work_w) {
                        cands.push((tx, ty, rot as u16));
                    }
                }
            }
            t += step;
        }
        // 현재 위치도 후보에 넣는다
        let cur = piece_penalty(
            eng,
            prob,
            placed,
            target.item_id,
            target.rot,
            target.tx,
            target.ty,
            &weights,
            None,
            &mut scratch,
        );
        let mut best = (
            cur.0,
            Cand {
                rot: target.rot as u16,
                tx: target.tx,
                ty: target.ty,
            },
        );
        let n = cands.len();
        for _ in 0..samples {
            if n == 0 {
                break;
            }
            let idx = rng.random_range(0..n);
            let (tx, ty, rot) = cands[idx];
            let p = piece_penalty(
                eng,
                prob,
                placed,
                target.item_id,
                rot as usize,
                tx,
                ty,
                &weights,
                None,
                &mut scratch,
            );
            if p.0 < best.0 {
                best = (p.0, Cand { rot, tx, ty });
            }
        }
        place_raw(eng, prob, placed, target.item_id, best.1);

        // 아직 겹쳐 있는 쌍의 가중치를 키운다 — 없으면 같은 국소 최적에서 맴돈다
        if best.0 > 0.0 {
            let last = placed.len() - 1;
            let p = placed[last];
            if eng.collect_overlaps(prob, p.item_id, p.rot, p.tx, p.ty, Some(p.pkey), &mut scratch)
            {
                let keys: Vec<PItemKey> = scratch.clone();
                for pk in keys {
                    if let Some(o) = placed.iter().find(|q| q.pkey == pk) {
                        let e = weights.entry(pair_key(p.item_id, o.item_id)).or_insert(1.0);
                        *e *= growth;
                    }
                }
            }
        }

        recompute!();
        let new_raw = praw.iter().sum::<f64>() * 0.5;
        if new_raw < best_raw - 1e-6 {
            best_raw = new_raw;
            no_improve = 0;
        } else {
            no_improve += 1;
            if no_improve >= no_improve_limit {
                return SeparateReport {
                    penalty_start,
                    penalty_end: new_raw,
                    iters,
                    resolved: false,
                    placed_all,
                };
            }
        }
    }
    let end = praw.iter().sum::<f64>() * 0.5;
    SeparateReport {
        penalty_start,
        penalty_end: end,
        iters,
        resolved: end <= 0.0,
        placed_all,
    }
}

/// 2C-E 진입점. 고정 폭에서 156개를 전부 놓고 겹침을 푼다.
/// 최종 해는 **반드시 겹침 0** 이어야 한다. 아니면 실패로 보고한다.
pub fn separate_at_width(
    instance: &SPInstance,
    cfg: &AttachCfg,
    width: f32,
    samples: usize,
    no_improve_limit: usize,
    growth: f32,
    time_budget_ms: u64,
    seed: u64,
) -> (SPSolution, AttachStats, SeparateReport) {
    use rand::SeedableRng;
    let t0 = StdInstant::now();
    let mut rng = Xoshiro256PlusPlus::seed_from_u64(seed);
    // 1) 고정 폭 구성 (겹침 0, 일부 미배치)
    let mut c = *cfg;
    c.fixed_width = Some(width);
    c.force_insert = false;
    let (_sol, base_stats) =
        build_initial_staged_cc(instance, &c, &CompactCfg::default(), Stage::S2C, &mut rng);

    // 2) 같은 설정으로 다시 구성한 뒤 그 상태에서 분리 루프를 돈다
    let mut eng = Engine::new(instance, &c, Stage::S2C);
    let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
    prob.change_strip_width(width).expect(crate::dmsg!("strip width must be positive"));
    let mut placed: Vec<Placed> = Vec::new();
    let mut positions: Vec<(f32, f32)> = Vec::with_capacity(c.cand_cap);

    let mut order: Vec<usize> = Vec::new();
    for (id, (_it, qty)) in instance.items.iter().enumerate() {
        for _ in 0..*qty {
            order.push(id);
        }
    }
    order.sort_by(|a, b| {
        eng.area[*b]
            .partial_cmp(&eng.area[*a])
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    for &id in order.iter() {
        match best_position(&mut eng, &prob, &placed, id, width, &mut positions) {
            Some(cd) => {
                try_push_placed(&mut eng, &mut prob, &mut placed, id, cd);
            }
            None => {}
        }
    }

    let deadline = StdInstant::now() + std::time::Duration::from_millis(time_budget_ms);
    let rep = separate(
        &mut eng,
        &mut prob,
        &mut placed,
        width,
        samples,
        no_improve_limit,
        growth,
        &mut rng,
        deadline,
        &mut positions,
    );

    if prob.n_placed_items() > 0 {
        prob.fit_strip().expect(crate::dmsg!("strip width must be positive"));
    }
    let sol = prob.save();
    let mut st = eng.stats.clone();
    st.placed = prob.n_placed_items();
    st.total = instance.total_item_qty();
    st.density = sol.density() * 100.0;
    st.width = sol.strip_width();
    st.secs = t0.elapsed().as_secs_f64();
    st.self_collisions = count_collisions(&prob);
    st.unplaced_ids = base_stats.unplaced_ids;
    (sol, st, rep)
}

// ===========================================================================
// 3C-A/3C-B — 동일 형상 클래스와 맞물림 오프셋
// ===========================================================================

/// 3C-B 결과: 같은 형상 두 개를 붙이는 평행이동 오프셋 하나.
#[derive(Clone, Copy, Debug)]
pub struct Interlock {
    pub d: (f32, f32),
    /// 합집합 bbox 면적 / (2 x 조각 면적). 1.0 에 가까울수록 조밀.
    pub ratio: f32,
}

/// 동일 형상 등가 클래스.
pub struct ShapeClass {
    pub ids: Vec<usize>,
    pub area: f32,
    pub n_verts: usize,
    pub bbox_w: f32,
    pub bbox_h: f32,
    pub concavity: f32,
    /// 전체 최선 / x 우세 / y 우세 오프셋
    pub best: Option<Interlock>,
    pub best_x: Option<Interlock>,
    pub best_y: Option<Interlock>,
}

/// 3C-A 형상 등가 클래스 키.
// ponytail: 위 4개 값이 모두 같으면 사실상 같은 형상이다. 어긋나면 그때 좌표 비교.
fn class_key(sh: &SPolygon) -> (usize, i64, i64, i64, i64) {
    let b = sh.bbox();
    let hull = nfp::hull_ccw(sh);
    let conc = nfp::signed_area(&hull).abs() / sh.area();
    (
        sh.n_vertices(),
        sh.area().round() as i64,
        ((b.x_max - b.x_min) * 10.0).round() as i64,
        ((b.y_max - b.y_min) * 10.0).round() as i64,
        (conc * 1000.0).round() as i64,
    )
}

/// 3C-A + 3C-B. 크기 2 이상 클래스에 대해 맞물림 오프셋까지 구한다.
pub fn analyze_classes(instance: &SPInstance, cfg: &AttachCfg) -> Vec<ShapeClass> {
    let mut eng = Engine::new(instance, cfg, Stage::S2C);
    let mut map: HashMap<(usize, i64, i64, i64, i64), Vec<usize>> = HashMap::new();
    for id in 0..instance.items.len() {
        map.entry(class_key(&instance.item(id).shape_cd()))
            .or_default()
            .push(id);
    }
    let mut classes: Vec<ShapeClass> = map
        .into_iter()
        .map(|(k, ids)| {
            let sh = &instance.item(ids[0]).shape_cd();
            let b = sh.bbox();
            ShapeClass {
                area: instance.item(ids[0]).area(),
                n_verts: k.0,
                bbox_w: b.x_max - b.x_min,
                bbox_h: b.y_max - b.y_min,
                concavity: k.4 as f32 / 1000.0,
                ids,
                best: None,
                best_x: None,
                best_y: None,
            }
        })
        .collect();
    classes.sort_by(|a, b| b.ids.len().cmp(&a.ids.len()).then(a.ids[0].cmp(&b.ids[0])));

    for c in classes.iter_mut() {
        if c.ids.len() < 2 {
            continue;
        }
        let (a, x, y) = interlock_for(&mut eng, instance, c.ids[0], c.ids[1]);
        c.best = a;
        c.best_x = x;
        c.best_y = y;
    }
    classes
}

/// 같은 형상 두 개(id_a, id_b)를 가장 조밀하게 붙이는 오프셋.
/// 회전은 쓰지 않는다 — 평행이동만이다.
/// NFP 경계 후보마다 합집합 bbox 비율을 재고, **CDE 정확 검사를 통과한 것만** 채택한다.
fn interlock_for(
    eng: &mut Engine,
    instance: &SPInstance,
    id_a: usize,
    id_b: usize,
) -> (Option<Interlock>, Option<Interlock>, Option<Interlock>) {
    let bb = eng.geom[id_a].base[0].bbox();
    let w = bb.x_max - bb.x_min;
    let h = bb.y_max - bb.y_min;
    let area = eng.area[id_a];

    // 검증용 스크래치: 폭은 넉넉히, 높이는 실제 원단 높이
    let scratch_w = (w * 6.0).max(3000.0);
    let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
    prob.change_strip_width(scratch_w).expect(crate::dmsg!("strip width must be positive"));
    // 사본 1을 가운데에 둔다 (음수 오프셋도 검증할 수 있게)
    let ax = scratch_w * 0.5 - w * 0.5 - bb.x_min;
    let ay = eng.strip_h * 0.5 - h * 0.5 - bb.y_min;
    prob.place_item(SPPlacement {
        item_idx: id_a,
        d_transf: DTransformation::new(eng.geom[id_a].angles[0], (ax, ay)),
    });

    // NFP(A, A) 경계 후보 오프셋
    let splits = eng.cfg.nfp_edge_splits.max(1);
    let hull = eng.geom[id_a].hull[0].clone();
    let neg = eng.geom[id_b].hull_neg[0].clone();
    let nfp = nfp::nfp_hull_pair(&hull, &neg, PLACE_EPS, splits);

    let mut best: Option<Interlock> = None;
    let mut best_x: Option<Interlock> = None;
    let mut best_y: Option<Interlock> = None;
    for p in nfp.cands.iter() {
        let d = (p.0, p.1);
        // 합집합 bbox
        let ux0 = bb.x_min.min(bb.x_min + d.0);
        let uy0 = bb.y_min.min(bb.y_min + d.1);
        let ux1 = bb.x_max.max(bb.x_max + d.0);
        let uy1 = bb.y_max.max(bb.y_max + d.1);
        let ratio = ((ux1 - ux0) * (uy1 - uy0)) / (2.0 * area);
        // 스크래치 안에 들어가는 후보만 검증할 수 있다
        let (tx, ty) = (ax + d.0, ay + d.1);
        if !eng.inside_strip(&bb, tx, ty, scratch_w) {
            continue;
        }
        if eng.collides(&prob, id_b, 0, tx, ty) {
            continue;
        }
        let il = Interlock { d, ratio };
        if best.map_or(true, |b| ratio < b.ratio) {
            best = Some(il);
        }
        if d.0.abs() >= d.1.abs() {
            if best_x.map_or(true, |b| ratio < b.ratio) {
                best_x = Some(il);
            }
        } else if best_y.map_or(true, |b| ratio < b.ratio) {
            best_y = Some(il);
        }
    }
    (best, best_x, best_y)
}

// ===========================================================================
// 3C-C — 동일 형상 묶음(복합 조각) 배치
// ===========================================================================

#[derive(Clone, Debug, Default)]
pub struct GroupStats {
    /// 묶음 시도 수 / 요청 크기 그대로 들어간 수
    pub groups_tried: usize,
    pub groups_full: usize,
    /// k -> k-1 로 줄인 횟수
    pub group_fallback: usize,
    /// 묶음으로 들어간 조각 수 / 개별로 돌아간 조각 수
    pub members_grouped: usize,
    pub members_single: usize,
    pub classes_grouped: usize,
}

impl<'a> Engine<'a> {
    /// 복합 조각을 가상 아이템으로 등록한다.
    /// **합집합 폴리곤을 만들지 않는다** — 후보 생성/슬라이딩/점수용 외곽으로
    /// 멤버 정점들의 볼록 껍질만 쓴다. 겹침 판정은 배치 때 멤버별로 한다.
    fn add_composite(&mut self, member_id: usize, offsets: &[(f32, f32)]) -> Option<usize> {
        let base = self.geom[member_id].base[0].clone();
        let mut pts: Vec<Point> = Vec::with_capacity(base.n_vertices() * offsets.len());
        for (dx, dy) in offsets.iter() {
            for v in base.vertices().iter() {
                pts.push(Point(v.0 + dx, v.1 + dy));
            }
        }
        let hull = nfp::to_ccw(nfp::dedup_pub(convex_hull_from_points(pts)));
        if hull.len() < 3 {
            return None;
        }
        let poly = SPolygon::new(hull.clone()).ok()?;
        let vid = self.geom.len();
        let hull_neg = nfp::negate_ccw(&hull);
        self.buff.push(poly.clone());
        self.geom.push(ItemGeom {
            angles: vec![0.0],
            base: vec![Arc::new(poly)],
            hull: vec![hull],
            hull_neg: vec![hull_neg],
            pieces: Vec::new(),
            pieces_neg: Vec::new(),
        });
        // NFP 캐시 키가 실제 도형과 충돌하지 않도록 새 클래스 번호를 준다
        self.class.push(u32::MAX - vid as u32);
        self.area.push(self.area[member_id] * offsets.len() as f32);
        Some(vid)
    }
}

/// 배치 단위: 개별 조각 하나이거나, 같은 형상 k개 묶음.
struct Unit {
    /// 묶음 멤버 (개별이면 1개)
    ids: Vec<usize>,
    /// 멤버별 상대 오프셋 (ids 와 같은 길이)
    offsets: Vec<(f32, f32)>,
    area: f32,
}

/// 3C-C. `group_k` 크기로 묶어 배치한다. `ratio_max` 미만인 클래스만 묶는다.
pub fn build_with_groups(
    instance: &SPInstance,
    cfg: &AttachCfg,
    group_k: usize,
    ratio_max: f32,
    fixed_width: Option<f32>,
) -> (SPSolution, AttachStats, GroupStats) {
    let t0 = StdInstant::now();
    let mut gs = GroupStats::default();
    let classes = analyze_classes(instance, cfg);
    let mut eng = Engine::new(instance, cfg, Stage::S2C);

    // ---- 배치 단위 구성 ------------------------------------------------
    let mut units: Vec<Unit> = Vec::new();
    let mut grouped: std::collections::HashSet<usize> = std::collections::HashSet::new();
    for c in classes.iter() {
        let Some(il) = c.best else { continue };
        if c.ids.len() < 2 || il.ratio >= ratio_max {
            continue;
        }
        let k = group_k.min(c.ids.len());
        if k < 2 {
            continue;
        }
        gs.classes_grouped += 1;
        for chunk in c.ids.chunks(k) {
            if chunk.len() < 2 {
                continue;
            }
            let offsets: Vec<(f32, f32)> = (0..chunk.len())
                .map(|i| (il.d.0 * i as f32, il.d.1 * i as f32))
                .collect();
            units.push(Unit {
                ids: chunk.to_vec(),
                offsets,
                area: eng.area[chunk[0]] * chunk.len() as f32,
            });
            for id in chunk {
                grouped.insert(*id);
            }
        }
    }
    for id in 0..instance.items.len() {
        if grouped.contains(&id) {
            continue;
        }
        units.push(Unit {
            ids: vec![id],
            offsets: vec![(0.0, 0.0)],
            area: eng.area[id],
        });
    }
    // 면적 내림차순 (복합 조각의 면적 = k x 조각 면적)
    units.sort_by(|a, b| {
        b.area
            .partial_cmp(&a.area)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.ids[0].cmp(&b.ids[0]))
    });

    // ---- 스트립 ---------------------------------------------------------
    let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
    let full_w = instance.item_area() / instance.base_strip.fixed_height;
    let work_w = fixed_width.unwrap_or(full_w * WORK_WIDTH_MULT);
    prob.change_strip_width(work_w).expect(crate::dmsg!("strip width must be positive"));
    let mut placed: Vec<Placed> = Vec::with_capacity(instance.total_item_qty());
    let mut positions: Vec<(f32, f32)> = Vec::with_capacity(cfg.cand_cap);
    let mut pending: Vec<usize> = Vec::new();

    // 첫 조각은 좌하단에 둔다 (기존 경로와 동일)
    let first = units[0].ids[0];
    {
        let bb = eng.geom[first].base[0].bbox();
        let (tx, ty) = (-bb.x_min + PLACE_EPS, -bb.y_min + PLACE_EPS);
        let pk = prob.place_item(SPPlacement {
            item_idx: first,
            d_transf: DTransformation::new(eng.geom[first].angles[0], (tx, ty)),
        });
        placed.push(Placed {
            pkey: pk,
            item_id: first,
            rot: 0,
            bbox: shifted(&bb, tx, ty),
            tx,
            ty,
        });
    }
    // 첫 단위가 묶음이면 나머지 멤버는 개별로 돌린다
    if units[0].ids.len() > 1 {
        for id in units[0].ids[1..].iter() {
            pending.push(*id);
        }
        gs.members_single += units[0].ids.len() - 1;
    }

    for u in units.iter().skip(1) {
        if u.ids.len() == 1 {
            let ok = match best_position(&mut eng, &prob, &placed, u.ids[0], work_w, &mut positions)
            {
                Some(c) => try_push_placed(&mut eng, &mut prob, &mut placed, u.ids[0], c),
                None => false,
            };
            if !ok {
                pending.push(u.ids[0]);
            }
            gs.members_single += 1;
            continue;
        }
        gs.groups_tried += 1;
        // k -> k-1 -> ... -> 1 로 줄이며 시도한다
        let mut m = u.ids.len();
        let mut done = false;
        while m >= 2 {
            let Some(vid) = eng.add_composite(u.ids[0], &u.offsets[..m]) else {
                break;
            };
            let cand = best_position(&mut eng, &prob, &placed, vid, work_w, &mut positions);
            if let Some(c) = cand {
                let snap_sol = prob.save();
                let snap_placed = placed.clone();
                let mut ok = true;
                for i in 0..m {
                    let (dx, dy) = u.offsets[i];
                    let mc = Cand {
                        rot: 0,
                        tx: c.tx + dx,
                        ty: c.ty + dy,
                    };
                    if !try_push_placed(&mut eng, &mut prob, &mut placed, u.ids[i], mc) {
                        ok = false;
                        break;
                    }
                }
                if ok {
                    if m == u.ids.len() {
                        gs.groups_full += 1;
                    }
                    gs.members_grouped += m;
                    for id in u.ids[m..].iter() {
                        pending.push(*id);
                        gs.members_single += 1;
                    }
                    done = true;
                    break;
                }
                prob.restore(&snap_sol);
                placed = snap_placed;
            }
            gs.group_fallback += 1;
            m -= 1;
        }
        if !done {
            for id in u.ids.iter() {
                pending.push(*id);
                gs.members_single += 1;
            }
        }
    }

    // ---- 남은 개별 조각 (면적 내림차순) --------------------------------
    pending.sort_by(|a, b| {
        eng.area[*b]
            .partial_cmp(&eng.area[*a])
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let mut failed: Vec<usize> = Vec::new();
    for id in pending {
        let ok = match best_position(&mut eng, &prob, &placed, id, work_w, &mut positions) {
            Some(c) => try_push_placed(&mut eng, &mut prob, &mut placed, id, c),
            None => false,
        };
        if !ok {
            failed.push(id);
        }
    }
    for id in failed {
        if let Some((rot, tx, ty)) = tail_append(&mut eng, &prob, &placed, id, work_w) {
            let c = Cand {
                rot: rot as u16,
                tx,
                ty,
            };
            if try_push_placed(&mut eng, &mut prob, &mut placed, id, c) {
                eng.stats.tail_appends += 1;
            } else {
                eng.stats.unrepairable += 1;
            }
        }
    }

    repair(&mut eng, &mut prob, &mut placed);
    if prob.n_placed_items() > 0 {
        prob.fit_strip().expect(crate::dmsg!("strip width must be positive"));
    }
    let sol = prob.save();
    eng.sync_cache_stats();
    let mut st = eng.stats.clone();
    st.unplaced_ids = prob
        .item_demand_qtys()
        .iter()
        .enumerate()
        .filter(|(_, q)| **q > 0)
        .flat_map(|(id, q)| std::iter::repeat(id).take(*q))
        .collect();
    st.placed = prob.n_placed_items();
    st.total = instance.total_item_qty();
    st.density = sol.density() * 100.0;
    st.width = sol.strip_width();
    st.secs = t0.elapsed().as_secs_f64();
    st.self_collisions = count_collisions(&prob);
    (sol, st, gs)
}

// ===========================================================================
// 3C-D — 점진 축소 + 좌표 하강 분리 (GLS)
// ===========================================================================

/// 스트립 밖으로 나간 면적. 겹침 면적과 같은 단위라 그대로 더할 수 있다.
fn overhang_area(bb: &Rect, strip_w: f32, strip_h: f32) -> f32 {
    let w = bb.x_max - bb.x_min;
    let h = bb.y_max - bb.y_min;
    let ox = (bb.x_max - (strip_w - PLACE_EPS)).max(0.0) + (PLACE_EPS - bb.x_min).max(0.0);
    let oy = (bb.y_max - (strip_h - PLACE_EPS)).max(0.0) + (PLACE_EPS - bb.y_min).max(0.0);
    (ox.min(w) * h + oy.min(h) * w).max(0.0)
}

/// 한 조각의 비용 `(가중, 원시)`.
/// 원시 = 실제로 겹치는 쌍의 bbox 교차 면적 합 + 스트립 밖으로 나간 면적.
/// 가중치는 **이동 선택에만** 쓰고 개선/종료 판정은 원시값으로 한다.
#[allow(clippy::too_many_arguments)]
fn piece_cost(
    eng: &mut Engine,
    prob: &SPProblem,
    placed: &[Placed],
    idx: &HashMap<PItemKey, usize>,
    item_id: usize,
    rot: usize,
    tx: f32,
    ty: f32,
    strip_w: f32,
    weights: &HashMap<(usize, usize), f32>,
    exclude: Option<PItemKey>,
    scratch: &mut Vec<PItemKey>,
) -> (f64, f64) {
    let inside = eng.collect_overlaps(prob, item_id, rot, tx, ty, exclude, scratch);
    let bb = shifted(&eng.geom[item_id].base[rot].bbox(), tx, ty);
    let (mut w_sum, mut raw) = (0.0f64, 0.0f64);
    if !inside {
        let o = overhang_area(&bb, strip_w, eng.strip_h).max(1.0) as f64;
        w_sum += o * OUT_WEIGHT;
        raw += o;
    }
    for pk in scratch.iter() {
        let Some(&k) = idx.get(pk) else { continue };
        let o = &placed[k];
        let a = bbox_overlap_area(&bb, &o.bbox).max(1.0) as f64;
        let w = *weights.get(&pair_key(item_id, o.item_id)).unwrap_or(&1.0) as f64;
        w_sum += a * w;
        raw += a;
    }
    (w_sum, raw)
}

/// 스트립 밖 면적에 주는 가중치. 겹침보다 우선해서 밀어넣게 한다.
const OUT_WEIGHT: f64 = 4.0;

/// 좌표 하강 스텝 상한(mm)과 하한.
const CD_STEP0: f32 = 32.0;
const CD_STEP_MIN: f32 = 0.25;

pub struct GlsReport {
    pub pen_start: f64,
    pub pen_end: f64,
    pub pairs_start: usize,
    pub pairs_end: usize,
    pub iters: usize,
    pub resolved: bool,
    // ---- N04C ----
    pub moves: usize,
    pub worse_accepted: usize,
    pub disrupts: usize,
    pub ruins_tried: usize,
    pub ruins_ok: usize,
    pub ruin_m_total: usize,
    pub chains_tried: usize,
    pub chains_ok: usize,
    // ---- N06 ----
    pub rescues_tried: usize,
    pub rescues_ok: usize,
    pub rescue_m_total: usize,
}

/// N04C 국소 최적 탈출 설정. 전부 끄면 N03C 동작과 동일하다.
#[derive(Clone, Copy, Debug)]
pub struct EscapeCfg {
    /// 4C-A: 담금질 초기 온도. 0.0 = 현재 동작(개선만 채택)
    pub t0: f32,
    /// 4C-B: 이 횟수만큼 연속 무개선이면 교란. 0 = 끔
    pub disrupt_after: usize,
    /// 4C-B: a=0(무작위 재배치) / b=1(군집 평행이동) / c=2(위치 교환)
    pub disrupt_kind: u8,
    /// 4C-B: kind=b 의 반경 (겹치는 조각 지름 배수)
    pub disrupt_radius: f32,
    /// 4C-C: 부분 파괴 반경 (겹치는 조각 지름 배수). 0 = 끔
    pub ruin_radius: f32,
    /// 4C-C: 정체 감지 임계
    pub ruin_after: usize,
    /// 4C-D: 연쇄 이동 깊이. 0 = 끔
    pub chain_depth: u32,
    /// 4C-D: 정체 감지 임계
    pub chain_after: usize,
    /// N06: 정체 시 겹친 조각을 **전수 후보**에서 무겹침 위치로 재배치. 0 = 끔.
    /// 기존 표본 탐색(이웃 24개 x 표본 50~250)과 달리 전 이웃의 NFP 후보를
    /// 좌측 우선 순으로 훑어 정확 CDE 무겹침 자리를 찾는다.
    pub rescue_after: usize,
    /// N06: 0 이 아니면 종료를 반복 수가 아니라 **무개선 연속 횟수**로 판정한다
    /// (하드 상한 = iter_cap x 10). 벌점이 계속 내려가는 중에 상한에 잘리는
    /// 사례(pen 2428→80 에서 3000반복 종료)를 막는다. 완전 정체는 오히려
    /// 더 일찍(stall_cap 에서) 끝난다. 0 = 기존 동작.
    pub stall_cap: usize,
    /// N06: 좌표 하강 최소 스텝 재정의(mm). 0.0 = 기존 CD_STEP_MIN(0.25).
    /// 벌점 수십 규모의 서브밀리미터 겹침은 0.25mm 격자로는 풀리지 않는다.
    pub cd_min: f32,
    /// N06: 일반 rescue 가 0건일 때(자유 자리가 전무한 벽) 단일 차단 조각과의
    /// 원자적 교환을 시도한다. 기존 연쇄 이동(표본 탐색)과 달리 양쪽 다
    /// 전수 스캔 + 무겹침 보장이다.
    pub swap_rescue: bool,
}

impl Default for EscapeCfg {
    fn default() -> Self {
        EscapeCfg {
            t0: 0.0,
            disrupt_after: 0,
            disrupt_kind: 0,
            disrupt_radius: 2.5,
            ruin_radius: 0.0,
            ruin_after: 200,
            chain_depth: 0,
            chain_after: 200,
            rescue_after: 0,
            stall_cap: 0,
            cd_min: 0.0,
            swap_rescue: false,
        }
    }
}

impl EscapeCfg {
    /// N06 확정 조합: 전수 재배치(300) + 정체 기반 종료(1500) + 미세 CD(0.03mm).
    /// 셋 다 켜야 유효하다 — 단독으로는 각각 기준선 이하였다 (6F/6G 측정).
    /// swap_rescue 는 벽시계 기준 강한 음의 기여로 확정 폐기 (6Z-swap 측정).
    pub fn n06() -> Self {
        EscapeCfg {
            rescue_after: 300,
            stall_cap: 1500,
            cd_min: 0.03,
            ..EscapeCfg::default()
        }
    }
}

/// 겹침을 허용한 상태에서 벌점을 반복적으로 줄인다 (guided local search).
/// N03C 기능: 좌표 하강 / 상대 이동 / 벌점 비례 표본 / 원시 벌점 최소 해 보관.
/// N04C 기능: 확률적 악화 채택 / 강제 교란 / 부분 파괴 재구성 / 연쇄 이동.
#[allow(clippy::too_many_arguments)]
fn separate_gls(
    eng: &mut Engine,
    prob: &mut SPProblem,
    placed: &mut Vec<Placed>,
    strip_w: f32,
    iter_cap: usize,
    samples_base: usize,
    growth: f32,
    esc: &EscapeCfg,
    rng: &mut Xoshiro256PlusPlus,
    deadline: StdInstant,
    positions: &mut Vec<(f32, f32)>,
    stop: Option<&AtomicBool>,
) -> GlsReport {
    let mut weights: HashMap<(usize, usize), f32> = HashMap::new();
    let mut scratch: Vec<PItemKey> = Vec::new();
    let n = placed.len();
    let mut idx: HashMap<PItemKey, usize> = HashMap::with_capacity(n);
    let mut pw = vec![0.0f64; n];
    let mut praw = vec![0.0f64; n];

    macro_rules! reindex {
        () => {{
            idx.clear();
            for (i, p) in placed.iter().enumerate() {
                idx.insert(p.pkey, i);
            }
        }};
    }
    macro_rules! recompute_all {
        () => {{
            reindex!();
            for k in 0..placed.len() {
                let p = placed[k];
                let (a, b) = piece_cost(
                    eng, prob, placed, &idx, p.item_id, p.rot, p.tx, p.ty, strip_w, &weights,
                    Some(p.pkey), &mut scratch,
                );
                pw[k] = a;
                praw[k] = b;
            }
        }};
    }
    macro_rules! recompute_some {
        ($ks:expr) => {{
            for &k in $ks.iter() {
                let p = placed[k];
                let (a, b) = piece_cost(
                    eng, prob, placed, &idx, p.item_id, p.rot, p.tx, p.ty, strip_w, &weights,
                    Some(p.pkey), &mut scratch,
                );
                pw[k] = a;
                praw[k] = b;
            }
        }};
    }

    recompute_all!();
    let mut raw_total: f64 = praw.iter().sum::<f64>() * 0.5;
    let pen_start = raw_total;
    let pairs_start = praw.iter().filter(|v| **v > 0.0).count();

    let mut r = GlsReport {
        pen_start,
        pen_end: raw_total,
        pairs_start,
        pairs_end: pairs_start,
        iters: 0,
        resolved: raw_total <= 0.0,
        moves: 0,
        worse_accepted: 0,
        disrupts: 0,
        ruins_tried: 0,
        ruins_ok: 0,
        ruin_m_total: 0,
        chains_tried: 0,
        chains_ok: 0,
        rescues_tried: 0,
        rescues_ok: 0,
        rescue_m_total: 0,
    };

    let mut best_raw = raw_total;
    let mut best_snap = if raw_total > 0.0 {
        Some((prob.save(), placed.clone()))
    } else {
        None
    };
    let mut stall = 0usize;

    // N06: stall_cap 모드에서는 무개선 연속 횟수로 끝낸다 (하드 상한 10배).
    let hard_cap = if esc.stall_cap > 0 {
        iter_cap.saturating_mul(10)
    } else {
        iter_cap
    };
    while r.iters < hard_cap
        && (esc.stall_cap == 0 || stall < esc.stall_cap)
        && StdInstant::now() < deadline
        && stop.map_or(true, |s| !s.load(AtomicOrdering::Relaxed))
    {
        r.iters += 1;
        if raw_total <= 0.0 {
            break;
        }

        // ---- 정체 시 탈출 연산자 (N06 rescue -> 4C-C -> 4C-D -> 4C-B 순) ----
        if esc.rescue_after > 0 && stall > 0 && stall % esc.rescue_after == 0 {
            r.rescues_tried += 1;
            let m =
                rescue_relocate(eng, prob, placed, &praw, strip_w, positions, esc.swap_rescue);
            // 재배치 실패(0개)여도 재삽입으로 pkey 가 전부 바뀌었으므로 항상 재계산한다.
            recompute_all!();
            raw_total = praw.iter().sum::<f64>() * 0.5;
            if m > 0 {
                r.rescues_ok += 1;
                r.rescue_m_total += m;
                if raw_total < best_raw - 1e-6 {
                    best_raw = raw_total;
                    best_snap = Some((prob.save(), placed.clone()));
                    stall = 0;
                } else {
                    // I-13: 무개선이면 stall 을 한 칸 밀어 배수에서 벗어나게 한다 —
                    // 즉시 재발화(전수 스캔 반복 지불)는 막되, 정체 기반 종료(stall_cap)
                    // 카운트는 계속 진행시킨다 (0/1 로 리셋하면 종료 창이 초기화되어
                    // 무개선 rescue 반복이 분리를 하드캡까지 끌고 간다 — 실측 확인).
                    stall += 1;
                }
                continue;
            }
        }
        if esc.disrupt_after > 0 && stall > 0 && stall % esc.disrupt_after == 0 {
            r.disrupts += 1;
            disrupt(
                eng, prob, placed, &praw, strip_w, esc.disrupt_kind, esc.disrupt_radius, rng,
            );
            recompute_all!();
            raw_total = praw.iter().sum::<f64>() * 0.5;
            stall = 1;
            continue;
        }

        // ---- 이동 대상 선택 ------------------------------------------
        let mut k_worst = 0usize;
        for k in 1..n {
            if pw[k] > pw[k_worst] {
                k_worst = k;
            }
        }
        let mut k_move = k_worst;
        {
            let p = placed[k_worst];
            eng.collect_overlaps(prob, p.item_id, p.rot, p.tx, p.ty, Some(p.pkey), &mut scratch);
            if !scratch.is_empty() && rng.random::<f32>() < 0.5 {
                let pick = scratch[rng.random_range(0..scratch.len())];
                if let Some(&kk) = idx.get(&pick) {
                    k_move = kk;
                }
            }
        }
        let target = placed[k_move];
        let total_w: f64 = pw.iter().sum::<f64>().max(1.0);
        let share = (pw[k_move] / total_w).clamp(0.0, 1.0);
        let n_samp = ((samples_base as f64) * (1.0 + 4.0 * share)).round() as usize;

        // 이동으로 비용이 바뀌는 조각 = 이동 조각 + 이동 전 겹침 상대 + 이동 후 겹침 상대.
        // 이동 전 상대는 제거 전에 모아 둔다 (k_move == k_worst 면 scratch 가 그 목록).
        let mut touched: Vec<usize> = Vec::with_capacity(scratch.len() + 8);
        touched.push(k_move);
        if k_move != k_worst {
            eng.collect_overlaps(
                prob, target.item_id, target.rot, target.tx, target.ty, Some(target.pkey),
                &mut scratch,
            );
        }
        for p in scratch.iter() {
            if let Some(&kk) = idx.get(p) {
                touched.push(kk);
            }
        }

        prob.remove_item(target.pkey);
        idx.remove(&target.pkey);

        let mut cands: Vec<(f32, f32)> = Vec::new();
        let np = placed.len();
        let step = (np / SEP_NEIGHBORS).max(1);
        let off = rng.random_range(0..np.max(1));
        let mut t = 0usize;
        while t < np.min(SEP_NEIGHBORS * step) {
            let j = (off + t) % np;
            if j != k_move {
                positions.clear();
                gen_positions(eng, placed[j], target.item_id, target.rot, positions);
                let bb = eng.geom[target.item_id].base[target.rot].bbox();
                for q in 0..positions.len() {
                    let (tx, ty) = positions[q];
                    if eng.inside_strip(&bb, tx, ty, strip_w) {
                        cands.push((tx, ty));
                    }
                }
            }
            t += step;
        }

        // 좌표 하강은 현재 위치와 최선 후보 **양쪽**에 적용한다.
        // 현재 위치를 다듬는 경로가 빠지면 T0=0 이 N03C 동작과 달라진다.
        let cd_min = if esc.cd_min > 0.0 { esc.cd_min } else { CD_STEP_MIN };
        let cd = |eng: &mut Engine,
                      prob: &SPProblem,
                      placed: &[Placed],
                      idx: &HashMap<PItemKey, usize>,
                      weights: &HashMap<(usize, usize), f32>,
                      scratch: &mut Vec<PItemKey>,
                      mut c: (f64, f32, f32)|
         -> (f64, f32, f32) {
            let mut st = CD_STEP0;
            while st >= cd_min {
                let mut improved = false;
                for (ddx, ddy) in [(-st, 0.0), (st, 0.0), (0.0, -st), (0.0, st)] {
                    let (nx, ny) = (c.1 + ddx, c.2 + ddy);
                    let bb = eng.geom[target.item_id].base[target.rot].bbox();
                    if !eng.inside_strip(&bb, nx, ny, strip_w) {
                        continue;
                    }
                    let v = piece_cost(
                        eng, prob, placed, idx, target.item_id, target.rot, nx, ny, strip_w,
                        weights, None, scratch,
                    );
                    if v.0 < c.0 - 1e-9 {
                        c = (v.0, nx, ny);
                        improved = true;
                    }
                }
                if !improved {
                    st *= 0.5;
                }
            }
            c
        };

        let cur0 = piece_cost(
            eng, prob, placed, &idx, target.item_id, target.rot, target.tx, target.ty, strip_w,
            &weights, None, &mut scratch,
        );
        let cur = cd(
            eng,
            prob,
            placed,
            &idx,
            &weights,
            &mut scratch,
            (cur0.0, target.tx, target.ty),
        );
        // 현재 위치를 제외한 최선 후보 (4C-A 가 Δ 를 재려면 분리해야 한다)
        let mut cand_best: Option<(f64, f32, f32)> = None;
        let m = cands.len();
        for _ in 0..n_samp {
            if m == 0 {
                break;
            }
            let (tx, ty) = cands[rng.random_range(0..m)];
            let c = piece_cost(
                eng, prob, placed, &idx, target.item_id, target.rot, tx, ty, strip_w, &weights,
                None, &mut scratch,
            );
            if cand_best.map_or(true, |b| c.0 < b.0) {
                cand_best = Some((c.0, tx, ty));
            }
        }
        if let Some(cb) = cand_best {
            cand_best = Some(cd(eng, prob, placed, &idx, &weights, &mut scratch, cb));
        }

        // ---- 4C-A 채택 판정 -------------------------------------------
        let (nx, ny) = match cand_best {
            None => (cur.1, cur.2),
            Some((cb, cx, cy)) => {
                let delta = cb - cur.0;
                if delta < 0.0 {
                    r.moves += 1;
                    (cx, cy)
                } else if esc.t0 > 0.0 {
                    let temp = esc.t0 as f64 * (1.0 - r.iters as f64 / iter_cap as f64).max(0.0);
                    let scale = (raw_total / n as f64).max(1.0);
                    let pacc = if temp > 0.0 {
                        (-delta / (temp * scale)).exp()
                    } else {
                        0.0
                    };
                    if rng.random::<f32>() < pacc as f32 {
                        r.moves += 1;
                        r.worse_accepted += 1;
                        (cx, cy)
                    } else {
                        (cur.1, cur.2)
                    }
                } else {
                    (cur.1, cur.2)
                }
            }
        };

        // I-15: 회전각·기하를 조각의 rot 인덱스와 일치시킨다 (0.0 리터럴 제거).
        let bb = eng.geom[target.item_id].base[target.rot].bbox();
        let pk = prob.place_item(SPPlacement {
            item_idx: target.item_id,
            d_transf: DTransformation::new(eng.geom[target.item_id].angles[target.rot], (nx, ny)),
        });
        placed[k_move] = Placed {
            pkey: pk,
            item_id: target.item_id,
            rot: target.rot,
            bbox: shifted(&bb, nx, ny),
            tx: nx,
            ty: ny,
        };
        idx.insert(pk, k_move);

        // 가중치 증가 (선택에만 사용)
        let mut bumped: Vec<(usize, usize)> = Vec::new();
        eng.collect_overlaps(prob, target.item_id, target.rot, nx, ny, Some(pk), &mut scratch);
        for p in scratch.iter() {
            if let Some(&kk) = idx.get(p) {
                touched.push(kk);
                let key = pair_key(target.item_id, placed[kk].item_id);
                let e = weights.entry(key).or_insert(1.0);
                *e *= growth;
                bumped.push(key);
            }
        }
        // 가중치는 (아이템 타입, 아이템 타입) 단위라, 오른 쌍의 타입에 속하면서 겹침이
        // 남아 있는 조각은 pw 가 달라질 수 있다 — 그 조각만 더 재계산한다 (praw 는 불변).
        if !bumped.is_empty() {
            for k in 0..n {
                if praw[k] > 0.0 {
                    let t = placed[k].item_id;
                    if bumped.iter().any(|&(x, y)| x == t || y == t) {
                        touched.push(k);
                    }
                }
            }
        }
        touched.sort_unstable();
        touched.dedup();
        recompute_some!(touched);
        raw_total = praw.iter().sum::<f64>() * 0.5;

        if raw_total < best_raw - 1e-6 {
            best_raw = raw_total;
            best_snap = Some((prob.save(), placed.clone()));
            stall = 0;
        } else {
            stall += 1;
        }
    }

    // 실제 벌점 최소 해를 채택한다 — 종료 시점 상태를 쓰지 않는다
    if raw_total > best_raw {
        if let Some((sol, pl)) = best_snap.take() {
            prob.restore(&sol);
            *placed = pl;
        }
    }
    recompute_all!();
    r.pen_end = praw.iter().sum::<f64>() * 0.5;
    r.pairs_end = praw.iter().filter(|v| **v > 0.0).count();
    r.resolved = r.pen_end <= 0.0;
    r
}

/// 겹침이 남은 조각 인덱스 목록.
fn overlapping_idx(praw: &[f64]) -> Vec<usize> {
    (0..praw.len()).filter(|&k| praw[k] > 0.0).collect()
}

/// 4C-B 강제 교란. 벌점을 무시하고 실행한다 — 그게 목적이다.
#[allow(clippy::too_many_arguments)]
fn disrupt(
    eng: &mut Engine,
    prob: &mut SPProblem,
    placed: &mut Vec<Placed>,
    praw: &[f64],
    strip_w: f32,
    kind: u8,
    radius_mult: f32,
    rng: &mut Xoshiro256PlusPlus,
) {
    let bad = overlapping_idx(praw);
    if bad.is_empty() {
        return;
    }
    let k = bad[rng.random_range(0..bad.len())];
    let a = placed[k];
    // I-15: 회전각·기하를 조각의 rot 인덱스와 일치시킨다 (0.0 리터럴 제거).
    let move_to = |eng: &Engine, prob: &mut SPProblem, placed: &mut Vec<Placed>, k: usize, tx: f32, ty: f32| {
        let p = placed[k];
        prob.remove_item(p.pkey);
        let bb = eng.geom[p.item_id].base[p.rot].bbox();
        let pk = prob.place_item(SPPlacement {
            item_idx: p.item_id,
            d_transf: DTransformation::new(eng.geom[p.item_id].angles[p.rot], (tx, ty)),
        });
        placed[k] = Placed {
            pkey: pk,
            item_id: p.item_id,
            rot: p.rot,
            bbox: shifted(&bb, tx, ty),
            tx,
            ty,
        };
    };
    match kind {
        // (a) 컨테이너 내 무작위 위치로 강제 이동
        0 => {
            let bb = eng.geom[a.item_id].base[a.rot].bbox();
            let w = bb.x_max - bb.x_min;
            let h = bb.y_max - bb.y_min;
            let x = rng.random::<f32>() * (strip_w - w - 2.0 * PLACE_EPS) + PLACE_EPS;
            let y = rng.random::<f32>() * (eng.strip_h - h - 2.0 * PLACE_EPS) + PLACE_EPS;
            move_to(eng, prob, placed, k, x - bb.x_min, y - bb.y_min);
        }
        // (b) 주변 반경 내 조각들을 함께 같은 방향으로 평행이동
        1 => {
            let diam = eng.geom[a.item_id].base[a.rot].diameter();
            let rad = diam * radius_mult;
            let (cx, cy) = (
                0.5 * (a.bbox.x_min + a.bbox.x_max),
                0.5 * (a.bbox.y_min + a.bbox.y_max),
            );
            let ang = rng.random::<f32>() * std::f32::consts::TAU;
            let dist = diam * 0.25 * (0.5 + rng.random::<f32>());
            let (dx, dy) = (ang.cos() * dist, ang.sin() * dist);
            let group: Vec<usize> = (0..placed.len())
                .filter(|&j| {
                    let p = placed[j];
                    let px = 0.5 * (p.bbox.x_min + p.bbox.x_max);
                    let py = 0.5 * (p.bbox.y_min + p.bbox.y_max);
                    ((px - cx).powi(2) + (py - cy).powi(2)).sqrt() <= rad
                })
                .collect();
            for j in group {
                let p = placed[j];
                let bb = eng.geom[p.item_id].base[p.rot].bbox();
                let (nx, ny) = (p.tx + dx, p.ty + dy);
                if eng.inside_strip(&bb, nx, ny, strip_w) {
                    move_to(eng, prob, placed, j, nx, ny);
                }
            }
        }
        // (c) 겹치는 두 조각의 위치를 서로 맞바꾼다
        _ => {
            let mut scratch: Vec<PItemKey> = Vec::new();
            eng.collect_overlaps(prob, a.item_id, a.rot, a.tx, a.ty, Some(a.pkey), &mut scratch);
            if scratch.is_empty() {
                return;
            }
            let pick = scratch[rng.random_range(0..scratch.len())];
            let Some(j) = placed.iter().position(|q| q.pkey == pick) else {
                return;
            };
            let b = placed[j];
            // bbox 좌하단 기준으로 자리를 맞바꾼다
            let abb = eng.geom[a.item_id].base[a.rot].bbox();
            let bbb = eng.geom[b.item_id].base[b.rot].bbox();
            let a_new = (
                b.bbox.x_min - abb.x_min,
                b.bbox.y_min - abb.y_min,
            );
            let b_new = (
                a.bbox.x_min - bbb.x_min,
                a.bbox.y_min - bbb.y_min,
            );
            if eng.inside_strip(&abb, a_new.0, a_new.1, strip_w)
                && eng.inside_strip(&bbb, b_new.0, b_new.1, strip_w)
            {
                move_to(eng, prob, placed, k, a_new.0, a_new.1);
                move_to(eng, prob, placed, j, b_new.0, b_new.1);
            }
        }
    }
}

#[derive(Clone, Debug)]
pub struct ShrinkStep {
    pub w_try: f32,
    pub ok: bool,
    pub pairs_start: usize,
    pub pairs_end: usize,
    pub iters: usize,
    pub pen_start: f64,
    pub pen_end: f64,
    pub secs: f64,
    pub moves: usize,
    pub worse_accepted: usize,
    pub disrupts: usize,
    pub ruins_ok: usize,
    pub chains_ok: usize,
    pub rescues_ok: usize,
}

pub struct ShrinkReport {
    pub w_start: f32,
    pub w_end: f32,
    pub ok: usize,
    pub fail: usize,
    pub steps: Vec<ShrinkStep>,
}

/// 3C-D. 기준선 해에서 시작해 폭만 조금씩 좁히고, 그때마다 분리를 돌린다.
/// **폭을 줄일 때 다시 배치하지 않는다** — 컨테이너만 좁힌다.
#[allow(clippy::too_many_arguments)]
pub fn shrink_and_separate(
    instance: &SPInstance,
    cfg: &AttachCfg,
    step0_frac: f32,
    min_frac: f32,
    iter_cap: usize,
    samples_base: usize,
    growth: f32,
    total_budget_ms: u64,
    seed: u64,
) -> (SPSolution, AttachStats, ShrinkReport) {
    shrink_and_separate_esc(
        instance,
        cfg,
        &EscapeCfg::default(),
        step0_frac,
        min_frac,
        iter_cap,
        samples_base,
        growth,
        total_budget_ms,
        seed,
    )
}

/// N04C 4C-A 진단: 기준선 축소가 막힌 폭에서 직접 분리를 시험한다.
/// 먼저 탈출 연산자를 끈 상태로 축소해 마지막 성공 해(6801.3mm)를 얻고,
/// 그 해에서 컨테이너만 목표 폭으로 좁혀 각 설정을 돌린다.
#[allow(clippy::too_many_arguments)]
pub fn stuck_probe(
    instance: &SPInstance,
    cfg: &AttachCfg,
    esc: &EscapeCfg,
    widths: &[f32],
    iter_cap: usize,
    samples_base: usize,
    growth: f32,
    budget_ms: u64,
    seed: u64,
) -> Vec<(f32, GlsReport, usize, bool, f64)> {
    use rand::SeedableRng;
    let mut rng = Xoshiro256PlusPlus::seed_from_u64(seed);
    // 기준선(탈출 끔)으로 마지막 성공 해를 얻는다
    let (base, _st, _rep) = shrink_and_separate_esc(
        instance,
        cfg,
        &EscapeCfg::default(),
        0.005,
        0.0005,
        iter_cap,
        samples_base,
        growth,
        180_000,
        seed,
    );
    let mut eng = Engine::new(instance, cfg, Stage::S2C);
    let mut positions: Vec<(f32, f32)> = Vec::with_capacity(cfg.cand_cap);
    let mut out = Vec::new();
    for &w in widths.iter() {
        let ts = StdInstant::now();
        let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
        prob.restore(&base);
        prob.change_strip_width(w).expect(crate::dmsg!("strip width must be positive"));
        // I-15: rot 를 실제 배치 회전각에서 복원한다
        let mut placed: Vec<Placed> = rebuild_placed(&eng, &prob);
        let dl = StdInstant::now() + std::time::Duration::from_millis(budget_ms);
        let g = separate_gls(
            &mut eng,
            &mut prob,
            &mut placed,
            w,
            iter_cap,
            samples_base,
            growth,
            esc,
            &mut rng,
            dl,
            &mut positions,
            None,
        );
        let coll = count_collisions(&prob);
        let full = prob.n_placed_items() == instance.total_item_qty();
        out.push((w, g, coll, full, ts.elapsed().as_secs_f64()));
    }
    out
}

/// N04C: 탈출 연산자를 켠 점진 축소.
#[allow(clippy::too_many_arguments)]
pub fn shrink_and_separate_esc(
    instance: &SPInstance,
    cfg: &AttachCfg,
    esc: &EscapeCfg,
    step0_frac: f32,
    min_frac: f32,
    iter_cap: usize,
    samples_base: usize,
    growth: f32,
    total_budget_ms: u64,
    seed: u64,
) -> (SPSolution, AttachStats, ShrinkReport) {
    use rand::SeedableRng;
    let t0 = StdInstant::now();
    let mut rng = Xoshiro256PlusPlus::seed_from_u64(seed);
    // 기준선 해
    let (base_sol, base_st) = build_initial_staged_cc(
        instance,
        cfg,
        &CompactCfg::default(),
        Stage::S2C,
        &mut rng,
    );
    let mut best_sol = base_sol.clone();
    let mut w = base_sol.strip_width();
    let w_start = w;
    let mut rep = ShrinkReport {
        w_start,
        w_end: w,
        ok: 0,
        fail: 0,
        steps: Vec::new(),
    };
    let mut eng = Engine::new(instance, cfg, Stage::S2C);
    let mut positions: Vec<(f32, f32)> = Vec::with_capacity(cfg.cand_cap);
    let deadline = StdInstant::now() + std::time::Duration::from_millis(total_budget_ms);

    let mut step = w * step0_frac;
    let min_step = w_start * min_frac;
    while step >= min_step && StdInstant::now() < deadline {
        let w_try = w - step;
        let ts = StdInstant::now();
        // 현재 해를 그대로 두고 컨테이너만 좁힌다
        let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
        prob.restore(&best_sol);
        prob.change_strip_width(w_try).expect(crate::dmsg!("strip width must be positive"));
        // I-15: rot 를 실제 배치 회전각에서 복원한다
        let mut placed: Vec<Placed> = rebuild_placed(&eng, &prob);
        let sub_deadline = deadline.min(ts + std::time::Duration::from_millis(60_000));
        let g = separate_gls(
            &mut eng,
            &mut prob,
            &mut placed,
            w_try,
            iter_cap,
            samples_base,
            growth,
            esc,
            &mut rng,
            sub_deadline,
            &mut positions,
            None,
        );
        let coll = count_collisions(&prob);
        let full = prob.n_placed_items() == instance.total_item_qty();
        let ok = g.resolved && coll == 0 && full;
        rep.steps.push(ShrinkStep {
            w_try,
            ok,
            pairs_start: g.pairs_start,
            pairs_end: g.pairs_end,
            iters: g.iters,
            pen_start: g.pen_start,
            pen_end: g.pen_end,
            secs: ts.elapsed().as_secs_f64(),
            moves: g.moves,
            worse_accepted: g.worse_accepted,
            disrupts: g.disrupts,
            ruins_ok: g.ruins_ok,
            chains_ok: g.chains_ok,
            rescues_ok: g.rescues_ok,
        });
        if ok {
            prob.fit_strip().expect(crate::dmsg!("strip width must be positive"));
            best_sol = prob.save();
            w = best_sol.strip_width();
            rep.ok += 1;
        } else {
            rep.fail += 1;
            step *= 0.5;
        }
    }
    rep.w_end = w;

    let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
    prob.restore(&best_sol);
    let mut st = base_st;
    st.placed = prob.n_placed_items();
    st.total = instance.total_item_qty();
    st.density = best_sol.density() * 100.0;
    st.width = best_sol.strip_width();
    st.secs = t0.elapsed().as_secs_f64();
    st.self_collisions = count_collisions(&prob);
    (best_sol, st, rep)
}

// ====================================================================
// N06 — 슬라이드 재압축 + 전수 구조 재배치 (이번 세션 신규 기능)
//
// N04C 까지의 벽: 축소가 6699mm 근처에서 3,000반복 내내 벌점 정체.
// 원인 분해:
//   1) 분리(separate_gls)가 조각을 옮긴 뒤 아무도 다시 밀착시키지 않는다.
//      LP 압축은 **구성 직후** 레이아웃에서만 측정됐고(slack 0.12mm/조각),
//      분리 후 레이아웃은 전혀 다른 상태다. → 접촉 슬라이딩 재압축.
//   2) 분리의 후보 탐색은 이웃 24개 표본 x 표본 50~250 으로 희소하다.
//      막힌 조각이 갈 수 있는 먼 빈자리를 절대 보지 못한다. → 전수 재배치.
//   3) 폭 결정(최우측) 조각은 아무 연산자도 직접 공략하지 않는다. → reseat.
// 셋 다 기존 저장소 경로(optimizer/*, structure/*, sample/*)를 쓰지 않고
// 이 모듈의 슬라이딩/NFP/CDE 프리미티브 위에만 세운다.
// ====================================================================

/// N06: 전수 재배치에서 조각당 CDE 검사 상한 (좌측 우선 순).
const RESCUE_SCAN_CAP: usize = 8000;

/// N06: reseat 이 의미 있다고 보는 최소 폭 이득(mm).
const RESEAT_MIN_GAIN: f32 = 0.5;

/// N06 슬라이드 재압축 설정.
#[derive(Clone, Copy, Debug)]
pub struct SlideCompactCfg {
    /// 전 조각 슬라이드 패스 최대 횟수 (홀수 패스는 -y 먼저)
    pub passes_max: usize,
    /// 한 패스의 총 이동량이 이보다 작으면 종료(mm)
    pub travel_tol: f32,
    /// 패스 후 재배치를 시도할 최우측 조각 수. 0 = reseat 끔
    pub reseat_top: usize,
    /// (슬라이드 패스 -> reseat) 라운드 수
    pub reseat_rounds: usize,
    /// reseat 전수 스캔에서 조각당 CDE 검사 상한
    pub scan_cap: usize,
}

impl Default for SlideCompactCfg {
    fn default() -> Self {
        SlideCompactCfg {
            passes_max: 6,
            travel_tol: 0.5,
            reseat_top: 4,
            reseat_rounds: 2,
            scan_cap: 6000,
        }
    }
}

/// N06 재압축 1회 결과.
#[derive(Clone, Copy, Debug, Default)]
pub struct CompactInfo {
    pub w_before: f32,
    pub w_after: f32,
    pub passes: usize,
    pub travel: f64,
    pub reseats: usize,
}

/// I-15: 배치 회전각을 `geom.angles` 인덱스로 역매핑한다 (허용 오차 1e-4 rad).
/// 매핑 실패(엔진이 모르는 각) 시 0 으로 폴백하되 debug 빌드에서는 단언한다.
fn rot_index_for(eng: &Engine, item_id: usize, angle: f32) -> usize {
    let angles = &eng.geom[item_id].angles;
    match angles.iter().position(|&a| same_angle(a, angle)) {
        Some(i) => i,
        None => {
            debug_assert!(
                false,
                "rot_index_for: item {item_id} 각 {angle} 가 허용 회전 집합에 없음"
            );
            0
        }
    }
}

/// prob 의 현재 배치를 Placed 배열로 재구성한다.
/// I-15: rot 를 0 으로 하드코딩하지 않고 실제 배치 회전각에서 복원한다.
fn rebuild_placed(eng: &Engine, prob: &SPProblem) -> Vec<Placed> {
    prob.layout()
        .placed_items()
        .iter()
        .map(|(pk, pi)| Placed {
            pkey: pk,
            item_id: pi.item().idx(),
            rot: rot_index_for(eng, pi.item().idx(), pi.d_transf().rotation()),
            bbox: pi.shape().bbox(),
            tx: pi.d_transf().translation().0,
            ty: pi.d_transf().translation().1,
        })
        .collect()
}

/// 배치 내용의 실제 우측 끝 (fit_strip 이 조일 폭).
fn content_width(placed: &[Placed]) -> f32 {
    placed.iter().map(|p| p.bbox.x_max).fold(0.0f32, f32::max)
}

/// -x/-y 밀착. `y_first` 면 -y 먼저(홀수 패스가 수직 슬랙을 먼저 회수한다).
fn slide_dir(
    eng: &mut Engine,
    prob: &SPProblem,
    item_id: usize,
    rot: usize,
    tx: f32,
    ty: f32,
    y_first: bool,
) -> (f32, f32) {
    if !y_first {
        return slide(eng, prob, item_id, rot, tx, ty);
    }
    let iters = eng.cfg.slide_iters;
    let bb = eng.geom[item_id].base[rot].bbox();
    let max_y = (bb.y_min + ty) - PLACE_EPS;
    let dy = slide_axis(eng, prob, item_id, rot, tx, ty, false, max_y, iters);
    let ty2 = ty - dy;
    let max_x = (bb.x_min + tx) - PLACE_EPS;
    let dx = slide_axis(eng, prob, item_id, rot, tx, ty2, true, max_x, iters);
    (tx - dx, ty2)
}

/// 등록 재검사를 포함한 배치. 겹치면 되돌리고 None.
fn place_checked(
    eng: &Engine,
    prob: &mut SPProblem,
    item_id: usize,
    rot: usize,
    tx: f32,
    ty: f32,
) -> Option<PItemKey> {
    let pk = prob.place_item(SPPlacement {
        item_idx: item_id,
        d_transf: DTransformation::new(eng.geom[item_id].angles[rot], (tx, ty)),
    });
    if crate::util::parts::placed_collides(
        prob.layout().cde(),
        &prob.layout().placed_items()[pk],
        &crate::util::parts::self_filter(pk),
    ) {
        prob.remove_item(pk);
        return None;
    }
    Some(pk)
}

/// 검사 없는 배치 (직전 상태 복원 전용).
fn place_unchecked(
    eng: &Engine,
    prob: &mut SPProblem,
    item_id: usize,
    rot: usize,
    tx: f32,
    ty: f32,
) -> PItemKey {
    prob.place_item(SPPlacement {
        item_idx: item_id,
        d_transf: DTransformation::new(eng.geom[item_id].angles[rot], (tx, ty)),
    })
}

/// placed[k] 를 (nx, ny) 로 옮긴다 (이미 prob 에서 제거된 상태로 호출).
/// 등록 재검사 실패 시 원위치로 복원한다. 반환 = 실제로 이동했는가.
fn reattach_at(
    eng: &Engine,
    prob: &mut SPProblem,
    placed: &mut [Placed],
    k: usize,
    nx: f32,
    ny: f32,
) -> bool {
    let p = placed[k];
    let bb = eng.geom[p.item_id].base[p.rot].bbox();
    let significant = (nx - p.tx).abs() > 1e-4 || (ny - p.ty).abs() > 1e-4;
    if significant {
        if let Some(pk) = place_checked(eng, prob, p.item_id, p.rot, nx, ny) {
            placed[k] = Placed {
                pkey: pk,
                item_id: p.item_id,
                rot: p.rot,
                bbox: shifted(&bb, nx, ny),
                tx: nx,
                ty: ny,
            };
            return true;
        }
    }
    let pk = match place_checked(eng, prob, p.item_id, p.rot, p.tx, p.ty) {
        Some(pk) => pk,
        // 원위치 재등록이 재검사에 걸리는 축퇴 케이스: 검사 없이 되돌린다
        // (제거 전과 동일한 상태이므로 실현성은 나빠지지 않는다).
        None => place_unchecked(eng, prob, p.item_id, p.rot, p.tx, p.ty),
    };
    placed[k] = Placed { pkey: pk, ..p };
    false
}

/// N06: 최우측 조각 전수 재배치. 폭 결정 조각을 들어내 전 이웃의 NFP 후보 중
/// x_max 가 확실히 줄어드는 위치로 옮긴다. 옮긴 수를 돌려준다.
fn reseat_pass(
    eng: &mut Engine,
    prob: &mut SPProblem,
    placed: &mut [Placed],
    strip_w: f32,
    top_k: usize,
    scan_cap: usize,
    positions: &mut Vec<(f32, f32)>,
    should_stop: &impl Fn() -> bool,
) -> usize {
    let n = placed.len();
    if n < 2 || top_k == 0 {
        return 0;
    }
    let mut ord: Vec<usize> = (0..n).collect();
    ord.sort_by(|&a, &b| {
        placed[b]
            .bbox
            .x_max
            .partial_cmp(&placed[a].bbox.x_max)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    ord.truncate(top_k);

    let mut moved = 0usize;
    for &k in ord.iter() {
        if should_stop() { break; }
        let p = placed[k];
        let old_x1 = p.bbox.x_max;
        prob.remove_item(p.pkey);
        let bb = eng.geom[p.item_id].base[p.rot].bbox();

        // (새 x_max, tx, ty) — x_max 오름차순으로 CDE 검사한다
        let mut cands: Vec<(f32, f32, f32)> = Vec::new();
        for j in 0..n {
            if should_stop() { break; }
            if j == k {
                continue;
            }
            positions.clear();
            gen_positions(eng, placed[j], p.item_id, p.rot, positions);
            for q in 0..positions.len() {
                let (tx, ty) = positions[q];
                if !eng.inside_strip(&bb, tx, ty, strip_w) {
                    continue;
                }
                let x1 = bb.x_max + tx;
                if x1 >= old_x1 - RESEAT_MIN_GAIN {
                    continue;
                }
                cands.push((x1, tx, ty));
            }
        }
        keep_smallest(&mut cands, scan_cap);

        let mut found: Option<(f32, f32)> = None;
        for &(_, tx, ty) in cands.iter() {
            if should_stop() { break; }
            if !eng.collides(prob, p.item_id, p.rot, tx, ty) {
                // 접촉 슬라이딩으로 다듬는다 (x_max 를 늘리지 않는 방향만)
                let (sx, sy) = slide(eng, prob, p.item_id, p.rot, tx, ty);
                let pick = if sx <= tx && !eng.collides(prob, p.item_id, p.rot, sx, sy) {
                    (sx, sy)
                } else {
                    (tx, ty)
                };
                found = Some(pick);
                break;
            }
        }
        match found {
            Some((tx, ty)) => {
                if reattach_at(eng, prob, placed, k, tx, ty) {
                    moved += 1;
                }
            }
            None => {
                reattach_at(eng, prob, placed, k, p.tx, p.ty);
            }
        }
    }
    moved
}

/// N06: 실현 가능(겹침 0) 레이아웃을 접촉 슬라이딩으로 재압축한다.
/// 조각별 원자적 이동만 하므로 겹침 0 이 유지된다.
fn compact_slide(
    eng: &mut Engine,
    prob: &mut SPProblem,
    placed: &mut Vec<Placed>,
    strip_w: f32,
    ccfg: &SlideCompactCfg,
    positions: &mut Vec<(f32, f32)>,
) -> CompactInfo {
    compact_slide_ctl(eng, prob, placed, strip_w, ccfg, positions, &|| false)
}

fn compact_slide_ctl(
    eng: &mut Engine, prob: &mut SPProblem, placed: &mut Vec<Placed>, strip_w: f32,
    ccfg: &SlideCompactCfg, positions: &mut Vec<(f32,f32)>, should_stop: &impl Fn() -> bool,
) -> CompactInfo {
    let mut info = CompactInfo {
        w_before: content_width(placed),
        ..CompactInfo::default()
    };
    let n = placed.len();
    if n < 2 {
        info.w_after = info.w_before;
        return info;
    }
    for _round in 0..ccfg.reseat_rounds.max(1) {
        if should_stop() { break; }
        for pass in 0..ccfg.passes_max {
            let y_first = pass % 2 == 1;
            let mut ord: Vec<usize> = (0..n).collect();
            ord.sort_by(|&a, &b| {
                let (ka, kb) = if y_first {
                    (placed[a].bbox.y_min, placed[b].bbox.y_min)
                } else {
                    (placed[a].bbox.x_min, placed[b].bbox.x_min)
                };
                ka.partial_cmp(&kb).unwrap_or(std::cmp::Ordering::Equal)
            });
            let mut travel = 0.0f64;
            for &k in ord.iter() {
                if should_stop() { break; }
                let p = placed[k];
                prob.remove_item(p.pkey);
                let (nx, ny) = slide_dir(eng, prob, p.item_id, p.rot, p.tx, p.ty, y_first);
                if reattach_at(eng, prob, placed, k, nx, ny) {
                    travel += ((p.tx - placed[k].tx).abs() + (p.ty - placed[k].ty).abs()) as f64;
                }
            }
            info.passes += 1;
            info.travel += travel;
            if travel < ccfg.travel_tol as f64 {
                break;
            }
        }
        if ccfg.reseat_top == 0 {
            break;
        }
        let m = reseat_pass(
            eng, prob, placed, strip_w, ccfg.reseat_top, ccfg.scan_cap, positions, should_stop,
        );
        info.reseats += m;
        if m == 0 {
            break;
        }
    }
    info.w_after = content_width(placed);
    info
}

/// N06: 이 조각의 무겹침 자리를 **전 이웃**의 NFP 후보에서 좌측 우선 순으로
/// 찾는다 (호출 전에 prob 에서 제거된 상태여야 한다).
#[allow(clippy::too_many_arguments)]
fn find_free_spot(
    eng: &mut Engine,
    prob: &SPProblem,
    placed: &[Placed],
    skip: usize,
    item_id: usize,
    rot: usize,
    strip_w: f32,
    positions: &mut Vec<(f32, f32)>,
    scan_cap: usize,
) -> Option<(f32, f32)> {
    let (wx, wy) = (eng.cfg.lbf_x_weight.max(1.0), eng.cfg.lbf_y_weight.max(0.0));
    let bb = eng.geom[item_id].base[rot].bbox();
    let mut cands: Vec<(f32, f32, f32)> = Vec::new();
    for j in 0..placed.len() {
        if j == skip {
            continue;
        }
        positions.clear();
        gen_positions(eng, placed[j], item_id, rot, positions);
        for q in 0..positions.len() {
            let (tx, ty) = positions[q];
            if !eng.inside_strip(&bb, tx, ty, strip_w) {
                continue;
            }
            cands.push(((bb.x_min + tx) * wx + (bb.y_min + ty) * wy, tx, ty));
        }
    }
    keep_smallest(&mut cands, scan_cap);
    for &(_, tx, ty) in cands.iter() {
        if !eng.collides(prob, item_id, rot, tx, ty) {
            let (sx, sy) = slide(eng, prob, item_id, rot, tx, ty);
            let pick = if !eng.collides(prob, item_id, rot, sx, sy) {
                (sx, sy)
            } else {
                (tx, ty)
            };
            return Some(pick);
        }
    }
    None
}

/// N06: 자유 자리가 전무한 조각 A 를, 후보 자리를 홀로 막는 조각 B 와
/// 원자적으로 교환한다. A 는 B 자리 근처의 후보로, B 는 전수 스캔 자유
/// 자리로 간다. 성공 시 placed[k]/placed[kb] 가 갱신된다.
/// 호출 시 A 는 prob 에서 제거된 상태다. 실패 시 A 는 제거된 채로 남는다
/// (호출자가 원위치 복원을 담당한다).
fn swap_try(
    eng: &mut Engine,
    prob: &mut SPProblem,
    placed: &mut [Placed],
    k: usize,
    strip_w: f32,
    positions: &mut Vec<(f32, f32)>,
) -> bool {
    let p = placed[k];
    let bb = eng.geom[p.item_id].base[p.rot].bbox();
    let (wx, wy) = (eng.cfg.lbf_x_weight.max(1.0), eng.cfg.lbf_y_weight.max(0.0));
    let mut cands: Vec<(f32, f32, f32)> = Vec::new();
    for j in 0..placed.len() {
        if j == k {
            continue;
        }
        positions.clear();
        gen_positions(eng, placed[j], p.item_id, p.rot, positions);
        for q in 0..positions.len() {
            let (tx, ty) = positions[q];
            if !eng.inside_strip(&bb, tx, ty, strip_w) {
                continue;
            }
            cands.push(((bb.x_min + tx) * wx + (bb.y_min + ty) * wy, tx, ty));
        }
    }
    keep_smallest(&mut cands, SWAP_SCAN_CAP);

    let mut scratch: Vec<PItemKey> = Vec::new();
    let mut tries = 0usize;
    for &(_, tx, ty) in cands.iter() {
        if tries >= SWAP_TRY_CAP {
            break;
        }
        // I-16: 값싼 bbox 사전 필터 — 이동 bbox 와 교차하는 배치 bbox 가 2개 이상이면
        // 단일 차단일 수 없으므로 비싼 CDE 전수 수집을 건너뛴다.
        let mbb = shifted(&bb, tx, ty);
        let mut n_bb = 0usize;
        for (j, q) in placed.iter().enumerate() {
            if j == k {
                continue;
            }
            if bbox_overlap_area(&mbb, &q.bbox) > 0.0 {
                n_bb += 1;
                if n_bb > 1 {
                    break;
                }
            }
        }
        if n_bb > 1 {
            continue;
        }
        // 이 후보를 막는 조각이 정확히 하나인가
        let inside = eng.collect_overlaps(prob, p.item_id, p.rot, tx, ty, None, &mut scratch);
        if !inside {
            continue;
        }
        // I-16: 차단이 아예 없으면 (find_free_spot 캡 밖의) 실제 자유 자리다 — 즉시 사용.
        if scratch.is_empty() {
            if let Some(apk) = place_checked(eng, prob, p.item_id, p.rot, tx, ty) {
                placed[k] = Placed {
                    pkey: apk,
                    item_id: p.item_id,
                    rot: p.rot,
                    bbox: mbb,
                    tx,
                    ty,
                };
                return true;
            }
            continue;
        }
        if scratch.len() != 1 {
            continue;
        }
        let bpk = scratch[0];
        let Some(kb) = placed.iter().position(|q| q.pkey == bpk) else {
            continue;
        };
        tries += 1;
        let b = placed[kb];
        // B 를 들어내고 A 를 넣는다
        prob.remove_item(b.pkey);
        let apk = if eng.collides(prob, p.item_id, p.rot, tx, ty) {
            None
        } else {
            place_checked(eng, prob, p.item_id, p.rot, tx, ty)
        };
        let Some(apk) = apk else {
            let pk = place_unchecked(eng, prob, b.item_id, b.rot, b.tx, b.ty);
            placed[kb] = Placed { pkey: pk, ..b };
            continue;
        };
        placed[k] = Placed {
            pkey: apk,
            item_id: p.item_id,
            rot: p.rot,
            bbox: shifted(&bb, tx, ty),
            tx,
            ty,
        };
        // B 의 자유 자리 스캔 (자기 스테일 엔트리는 앵커에서 제외, I-16 소캡)
        if let Some((bx, by)) = find_free_spot(
            eng, prob, placed, kb, b.item_id, b.rot, strip_w, positions, SWAP_B_SCAN_CAP,
        ) {
            if let Some(bpk2) = place_checked(eng, prob, b.item_id, b.rot, bx, by) {
                let bbb = eng.geom[b.item_id].base[b.rot].bbox();
                placed[kb] = Placed {
                    pkey: bpk2,
                    item_id: b.item_id,
                    rot: b.rot,
                    bbox: shifted(&bbb, bx, by),
                    tx: bx,
                    ty: by,
                };
                return true;
            }
        }
        // 실패 → 원상 복구 (A 는 호출자가 복원)
        prob.remove_item(apk);
        placed[k] = p;
        let pk = place_unchecked(eng, prob, b.item_id, b.rot, b.tx, b.ty);
        placed[kb] = Placed { pkey: pk, ..b };
    }
    false
}

/// N06: 정체 시 겹친 조각의 전수 재배치. 겹친 조각을 들어내 **전 이웃**의
/// NFP 후보를 좌측 우선 순으로 훑어 정확 CDE 무겹침 자리를 찾는다.
/// 찾으면 그 조각의 벌점 기여가 통째로 사라진다. 옮긴 수를 돌려준다.
/// `swap` 이면 자유 자리가 전무할 때 단일 차단 조각과의 교환까지 시도한다.
fn rescue_relocate(
    eng: &mut Engine,
    prob: &mut SPProblem,
    placed: &mut [Placed],
    praw: &[f64],
    strip_w: f32,
    positions: &mut Vec<(f32, f32)>,
    swap: bool,
) -> usize {
    let bad = overlapping_idx(praw);
    if bad.is_empty() {
        return 0;
    }
    let mut scratch: Vec<PItemKey> = Vec::new();
    let mut moved = 0usize;
    for &k in bad.iter() {
        let p = placed[k];
        // 앞선 재배치로 이미 풀렸을 수 있다 — 실제 겹침을 다시 확인한다
        let inside =
            eng.collect_overlaps(prob, p.item_id, p.rot, p.tx, p.ty, Some(p.pkey), &mut scratch);
        if inside && scratch.is_empty() {
            continue;
        }
        prob.remove_item(p.pkey);
        let bb = eng.geom[p.item_id].base[p.rot].bbox();

        if let Some((tx, ty)) = find_free_spot(
            eng, prob, placed, k, p.item_id, p.rot, strip_w, positions, RESCUE_SCAN_CAP,
        ) {
            if let Some(pk) = place_checked(eng, prob, p.item_id, p.rot, tx, ty) {
                placed[k] = Placed {
                    pkey: pk,
                    item_id: p.item_id,
                    rot: p.rot,
                    bbox: shifted(&bb, tx, ty),
                    tx,
                    ty,
                };
                moved += 1;
                continue;
            }
        } else if swap && swap_try(eng, prob, placed, k, strip_w, positions) {
            moved += 1;
            continue;
        }
        // 자유 자리 없음 / 등록 재검사 실패 → 원위치 복원 (겹친 상태라 검사 없이)
        let pk = place_unchecked(eng, prob, p.item_id, p.rot, p.tx, p.ty);
        placed[k] = Placed { pkey: pk, ..p };
    }
    moved
}

/// N06 swap-rescue 상한: 단일 차단 후보의 실제 교환 시도 수 / 스캔 후보 수.
const SWAP_TRY_CAP: usize = 60;
const SWAP_SCAN_CAP: usize = 4000;
/// I-16: 교환 시도당 B 의 자유 자리 스캔 상한 (rescue 본체의 8000 대비 소캡).
/// swap 이 필요한 국면은 자유 자리가 희소해 실패 스캔이 캡을 전액 소진하기 쉽다.
const SWAP_B_SCAN_CAP: usize = 500;

/// N06 진단: 구성 해 하나를 재압축만 해 본다 (축소 없음).
pub fn compact_probe(
    instance: &SPInstance,
    cfg: &AttachCfg,
    ccfg: &SlideCompactCfg,
    seed: u64,
) -> (SPSolution, AttachStats, CompactInfo) {
    use rand::SeedableRng;
    let t0 = StdInstant::now();
    let mut rng = Xoshiro256PlusPlus::seed_from_u64(seed);
    let (sol, st) = build_initial_staged_cc(instance, cfg, &CompactCfg::default(), Stage::S2C, &mut rng);
    let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
    prob.restore(&sol);
    let mut eng = Engine::new(instance, cfg, Stage::S2C);
    let mut placed = rebuild_placed(&eng, &prob);
    let w = prob.strip_width();
    let mut positions: Vec<(f32, f32)> = Vec::with_capacity(cfg.cand_cap);
    let info = compact_slide(&mut eng, &mut prob, &mut placed, w, ccfg, &mut positions);
    prob.fit_strip().expect(crate::dmsg!("strip width must be positive"));
    let out = prob.save();
    let mut st2 = st;
    st2.placed = prob.n_placed_items();
    st2.total = instance.total_item_qty();
    st2.density = out.density() * 100.0;
    st2.width = out.strip_width();
    st2.secs = t0.elapsed().as_secs_f64();
    st2.self_collisions = count_collisions(&prob);
    (out, st2, info)
}

/// N06 본선: 구성(GRASP) -> 재압축 -> [축소 -> 분리(+rescue) -> 재압축] 반복.
///
/// `shrink_and_separate_esc` 와의 차이
///   * 구성을 `grasp` 회 돌려 (완전·무겹침 중) 최소 폭 해로 시작한다.
///   * 축소 성공 때마다 슬라이드 재압축 + reseat 로 슬랙을 회수하고 폭을 더 조인다.
///   * 분리에는 esc.rescue_after 로 전수 재배치 연산자를 켤 수 있다.
#[allow(clippy::too_many_arguments)]
pub fn shrink_compact(
    instance: &SPInstance,
    cfg: &AttachCfg,
    esc: &EscapeCfg,
    ccfg: &SlideCompactCfg,
    grasp: usize,
    kicks: usize,
    step0_frac: f32,
    min_frac: f32,
    iter_cap: usize,
    samples_base: usize,
    growth: f32,
    total_budget_ms: u64,
    seed: u64,
) -> (SPSolution, AttachStats, ShrinkReport, Vec<CompactInfo>) {
    shrink_compact_ctl(
        instance,
        cfg,
        esc,
        ccfg,
        grasp,
        kicks,
        step0_frac,
        min_frac,
        iter_cap,
        samples_base,
        growth,
        total_budget_ms,
        seed,
        None,
        None,
    )
}

/// N06 본선 + 외부 제어: `stop` 플래그(협조적 중단)와 `on_improve`(최선 해 갱신
/// 콜백)를 받는다. FFI 단계 API(`swarm_init_phase`)가 사용한다.
#[allow(clippy::too_many_arguments)]
pub fn shrink_compact_ctl(
    instance: &SPInstance,
    cfg: &AttachCfg,
    esc: &EscapeCfg,
    ccfg: &SlideCompactCfg,
    grasp: usize,
    kicks: usize,
    step0_frac: f32,
    min_frac: f32,
    iter_cap: usize,
    samples_base: usize,
    growth: f32,
    total_budget_ms: u64,
    seed: u64,
    stop: Option<&AtomicBool>,
    on_improve: Option<&mut dyn FnMut(&SPSolution)>,
) -> (SPSolution, AttachStats, ShrinkReport, Vec<CompactInfo>) {
    shrink_compact_impl(instance, cfg, esc, ccfg, grasp, kicks, step0_frac, min_frac,
        iter_cap, samples_base, growth, total_budget_ms, seed, stop, on_improve, None)
}

/// Reuse an exact Init result. The discrete N06 engine starts from that result;
/// frozen/rigid/arbitrary-angle inputs use constraint-aware partial unit moves.
#[allow(clippy::too_many_arguments)]
pub fn shrink_compact_seeded_ctl(
    instance: &SPInstance, cfg: &AttachCfg, esc: &EscapeCfg, ccfg: &SlideCompactCfg,
    grasp: usize, kicks: usize, step0_frac: f32, min_frac: f32, iter_cap: usize,
    samples_base: usize, growth: f32, total_budget_ms: u64, seed: u64,
    stop: Option<&AtomicBool>, on_improve: Option<&mut dyn FnMut(&SPSolution)>,
    initial: super::incumbent::InitialSeed<'_>,
) -> Result<(SPSolution, AttachStats, ShrinkReport, Vec<CompactInfo>), String> {
    let started = StdInstant::now();
    let mut mask = crate::util::frozen::mask(instance.items.len(), initial.frozen_ids);
    initial.groups.expand_frozen_mask(&mut mask);
    if !initial.groups.solution_preserves_relative_poses(initial.solution)
        || crate::structure::pipeline::solution_validation_error_with_frozen_and_groups(
            instance, initial.solution, &mask, initial.groups).is_some() {
        return Err("Initial seed is not a complete certified layout".into());
    }
    let total_budget_ms = total_budget_ms.saturating_sub(started.elapsed().as_millis().min(u64::MAX as u128) as u64);
    if !initial.frozen_ids.is_empty() || !initial.groups.is_empty()
        || instance.items.iter().any(|(item,_)| matches!(item.allowed_orientations().rotations(), RotationRange::Continuous)) {
        use crate::util::terminator::Terminator;
        use rand::SeedableRng;
        let t0 = started;
        let term = super::incumbent::SeedTerm::new(total_budget_ms, stop);
        // Cancellation can return the already certified seed without preparing
        // a Separator or importing any geometry again.
        let all_frozen = initial.solution.layout_snapshot().placed_items().values()
            .all(|p| mask.get(p.item().idx()).copied().unwrap_or(false));
        let best = if term.kill() || all_frozen { initial.solution.clone() } else {
            let mut sc = crate::config::DEFAULT_SWARM_CONFIG.expl_cfg.separator_config;
            sc.n_workers = 1;
            sc.iter_no_imprv_limit = iter_cap;
            sc.sample_config.n_container_samples = samples_base;
            let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
            crate::util::frozen::restore(&mut prob, initial.solution);
            let mut sep = crate::optimizer::separator::Separator::new(instance.clone(), prob,
                Xoshiro256PlusPlus::seed_from_u64(seed), sc);
            sep.freeze_items(initial.frozen_ids);
            sep.set_place_groups(initial.groups.clone()).map_err(|e| e.to_string())?;
            super::incumbent::improve_seeded_units(&mut sep, initial.solution, ccfg,
                grasp.saturating_add(kicks).max(1), &term, on_improve)?
        };
        let st = AttachStats { placed: instance.total_item_qty(), total: instance.total_item_qty(),
            width: best.strip_width(), density: best.density()*100., secs: t0.elapsed().as_secs_f64(),
            ..Default::default() };
        let rep = ShrinkReport { w_start: initial.solution.strip_width(), w_end: best.strip_width(),
            ok: usize::from(best.strip_width()<initial.solution.strip_width()), fail: 0, steps: vec![] };
        return Ok((best,st,rep,vec![]));
    }
    Ok(shrink_compact_impl(instance, cfg, esc, ccfg, grasp, kicks, step0_frac, min_frac,
        iter_cap, samples_base, growth, total_budget_ms, seed, stop, on_improve, Some(initial.solution)))
}

#[allow(clippy::too_many_arguments)]
fn shrink_compact_impl(
    instance: &SPInstance, cfg: &AttachCfg, esc: &EscapeCfg, ccfg: &SlideCompactCfg,
    grasp: usize, kicks: usize, step0_frac: f32, min_frac: f32, iter_cap: usize,
    samples_base: usize, growth: f32, total_budget_ms: u64, seed: u64,
    stop: Option<&AtomicBool>, mut on_improve: Option<&mut dyn FnMut(&SPSolution)>,
    initial: Option<&SPSolution>,
) -> (SPSolution, AttachStats, ShrinkReport, Vec<CompactInfo>) {
    use rand::SeedableRng;
    let t0 = StdInstant::now();
    let mut rng = Xoshiro256PlusPlus::seed_from_u64(seed);
    let deadline = t0 + std::time::Duration::from_millis(total_budget_ms);
    let stopped = || stop.is_some_and(|s| s.load(AtomicOrdering::Relaxed)) || StdInstant::now() >= deadline;

    // ---- 구성: grasp 회 중 (완전·무겹침) 최소 폭 ----------------------
    let mut base: Option<(SPSolution, AttachStats)> = initial.map(|sol| (sol.clone(), AttachStats {
        placed: instance.total_item_qty(), total: instance.total_item_qty(), width: sol.strip_width(),
        density: sol.density()*100., ..Default::default()
    }));
    let mut last: Option<(SPSolution, AttachStats)> = None;
    for g in 0..if initial.is_some() { 0 } else { grasp.max(1) } {
        // 최소 1회는 구성해야 돌려줄 해가 있다
        if g > 0 && stopped() {
            break;
        }
        let (sol, st) =
            build_initial_staged_cc(instance, cfg, &CompactCfg::default(), Stage::S2C, &mut rng);
        let valid = st.placed == st.total && st.self_collisions == 0;
        let better = base
            .as_ref()
            .map_or(true, |(bs, _)| sol.strip_width() < bs.strip_width());
        if valid && better {
            base = Some((sol.clone(), st.clone()));
        }
        last = Some((sol, st));
    }
    // I-14: 구성 전량 무효(병리적 입력)여도 패닉하지 않고 마지막 구성 결과를
    // 그대로 반환한다 — stats 의 placed/self_collisions 가 실패를 드러내고,
    // FFI 경로에서는 최종 인증이 이를 거부해 LBF 폴백으로 이어진다.
    let Some((base_sol, base_st)) = base else {
        let (sol, mut st) = last.expect(crate::dmsg!("grasp >= 1 이므로 최소 1회 구성"));
        st.secs = t0.elapsed().as_secs_f64();
        let w = sol.strip_width();
        return (
            sol,
            st,
            ShrinkReport {
                w_start: w,
                w_end: w,
                ok: 0,
                fail: 0,
                steps: Vec::new(),
            },
            Vec::new(),
        );
    };

    if stopped() {
        let w = base_sol.strip_width();
        return (base_sol, base_st, ShrinkReport { w_start: w, w_end: w, ok: 0, fail: 0, steps: vec![] }, vec![]);
    }

    // 시드 해(외부 `initial`)는 S2C 테이블에 없는 회전을 포함할 수 있다 -> 각을 등록한다.
    let mut eng = Engine::with_seed(instance, cfg, Stage::S2C, Some(&base_sol));
    let mut positions: Vec<(f32, f32)> = Vec::with_capacity(cfg.cand_cap);
    let mut infos: Vec<CompactInfo> = Vec::new();

    // ---- 시작 재압축 ---------------------------------------------------
    let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
    prob.restore(&base_sol);
    let mut placed = rebuild_placed(&eng, &prob);
    {
        let w0 = prob.strip_width();
        let snap = prob.save();
        let snap_placed = placed.clone();
        let info = compact_slide_ctl(&mut eng, &mut prob, &mut placed, w0, ccfg, &mut positions, &stopped);
        if count_collisions(&prob) == 0 && prob.n_placed_items() == instance.total_item_qty() {
            infos.push(info);
        } else {
            // 방어선 — 재압축은 정의상 실현성을 깨지 않지만, 깨지면 통째로 버린다.
            // placed 는 이후 쓰지 않는다 (축소 루프가 매 반복 재구성).
            prob.restore(&snap);
            drop(snap_placed);
        }
    }
    prob.fit_strip().expect(crate::dmsg!("strip width must be positive"));
    if stopped() || prob.strip_width() > base_sol.strip_width()
        || crate::structure::pipeline::solution_validation_error(instance, &prob.save()).is_some() {
        prob.restore(&base_sol);
    }
    let mut best_sol = prob.save();
    if best_sol.strip_width() < base_sol.strip_width() {
        if let Some(cb) = on_improve.as_deref_mut() { cb(&best_sol); }
    }
    let mut w = best_sol.strip_width();
    let w_start = w;
    let mut rep = ShrinkReport {
        w_start,
        w_end: w,
        ok: 0,
        fail: 0,
        steps: Vec::new(),
    };

    // ---- 점진 축소 -----------------------------------------------------
    // kick: step 이 최소치 아래로 떨어져도 예산이 남았으면 step 을 재확장해
    // 다시 시도한다. 분리는 확률적이라(rng 상태가 진행) 매 시도 궤적이 다르고,
    // 성공한 해만 채택하므로 최선 해는 단조 개선이다.
    let mut kicks_left = kicks;
    let mut step = w * step0_frac;
    let min_step = w_start * min_frac;
    loop {
        if stopped() {
            break;
        }
        if step < min_step {
            if kicks_left == 0 || StdInstant::now() >= deadline {
                break;
            }
            kicks_left -= 1;
            step = w * step0_frac;
        }
        if StdInstant::now() >= deadline {
            break;
        }
        let w_try = w - step;
        let ts = StdInstant::now();
        let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
        prob.restore(&best_sol);
        prob.change_strip_width(w_try).expect(crate::dmsg!("strip width must be positive"));
        let mut placed = rebuild_placed(&eng, &prob);
        let sub_deadline = deadline.min(ts + std::time::Duration::from_millis(60_000));
        let g = separate_gls(
            &mut eng,
            &mut prob,
            &mut placed,
            w_try,
            iter_cap,
            samples_base,
            growth,
            esc,
            &mut rng,
            sub_deadline,
            &mut positions,
            stop,
        );
        if stopped() { break; }
        let coll = count_collisions(&prob);
        let full = prob.n_placed_items() == instance.total_item_qty();
        let ok = g.resolved && coll == 0 && full
            && crate::structure::pipeline::solution_validation_error(instance, &prob.save()).is_none();
        rep.steps.push(ShrinkStep {
            w_try,
            ok,
            pairs_start: g.pairs_start,
            pairs_end: g.pairs_end,
            iters: g.iters,
            pen_start: g.pen_start,
            pen_end: g.pen_end,
            secs: ts.elapsed().as_secs_f64(),
            moves: g.moves,
            worse_accepted: g.worse_accepted,
            disrupts: g.disrupts,
            ruins_ok: g.ruins_ok,
            chains_ok: g.chains_ok,
            rescues_ok: g.rescues_ok,
        });
        if ok {
            // ---- 성공: 재압축으로 슬랙 회수 후 폭을 조인다 ------------
            let snap = prob.save();
            // GLS has already passed full original certification. Keep and
            // publish it before optional compaction can time out or fail.
            if snap.strip_width() < best_sol.strip_width() {
                best_sol = snap.clone();
                w = best_sol.strip_width();
                rep.ok += 1;
                if let Some(cb) = on_improve.as_deref_mut() { cb(&best_sol); }
            }
            if stopped() { break; }
            let snap_placed = placed.clone();
            let info =
                compact_slide_ctl(&mut eng, &mut prob, &mut placed, w_try, ccfg, &mut positions, &stopped);
            if stopped() { break; }
            if count_collisions(&prob) == 0 && prob.n_placed_items() == instance.total_item_qty()
            {
                infos.push(info);
            } else {
                prob.restore(&snap);
                placed = snap_placed;
            }
            prob.fit_strip().expect(crate::dmsg!("strip width must be positive"));
            let candidate = prob.save();
            if !super::incumbent::candidate_is_improvement(instance, &candidate, &best_sol, &[],
                &crate::optimizer::place_groups::PlaceGroups::default()) {
                continue;
            }
            best_sol = candidate;
            w = best_sol.strip_width();
            if let Some(cb) = on_improve.as_deref_mut() {
                cb(&best_sol);
            }
        } else {
            rep.fail += 1;
            step *= 0.5;
        }
    }
    rep.w_end = w;

    let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
    prob.restore(&best_sol);
    let mut st = base_st;
    st.placed = prob.n_placed_items();
    st.total = instance.total_item_qty();
    st.density = best_sol.density() * 100.0;
    st.width = best_sol.strip_width();
    st.secs = t0.elapsed().as_secs_f64();
    st.self_collisions = count_collisions(&prob);
    (best_sol, st, rep, infos)
}

/// 후보가 전혀 없을 때 스트립 꼬리에 강제로 붙인다.
fn tail_append(
    eng: &mut Engine,
    prob: &SPProblem,
    placed: &[Placed],
    item_id: usize,
    work_w: f32,
) -> Option<(usize, f32, f32)> {
    let cur_len = placed.iter().map(|p| p.bbox.x_max).fold(0.0f32, f32::max);
    let n_rot = eng.geom[item_id].base.len();
    let step = eng.strip_h / 64.0;
    for rot in 0..n_rot {
        let bb = eng.geom[item_id].base[rot].bbox();
        let h = bb.y_max - bb.y_min;
        if h > eng.strip_h {
            continue;
        }
        let mut x = cur_len + PLACE_EPS;
        for _ in 0..8 {
            let mut y = PLACE_EPS;
            while y + h <= eng.strip_h - PLACE_EPS {
                let tx = x - bb.x_min;
                let ty = y - bb.y_min;
                if eng.inside_strip(&bb, tx, ty, work_w)
                    && !eng.collides(prob, item_id, rot, tx, ty)
                {
                    return Some((rot, tx, ty));
                }
                y += step;
            }
            x += (bb.x_max - bb.x_min) * 0.5;
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use jagua_rs::collision_detection::CDEConfig;
    use jagua_rs::entities::Item;
    use jagua_rs::geometry::OriginalShape;
    use jagua_rs::geometry::fail_fast::SPSurrogateConfig;
    use jagua_rs::geometry::shape_modification::{ShapeModifyConfig, ShapeModifyMode};
    use jagua_rs::probs::spp::entities::Strip;
    use std::f32::consts::PI;

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

    /// 좌하단이 원점인 `w × h` 직사각형 1종 × `qty` 개 인스턴스.
    fn rect_instance(w: f32, h: f32, qty: usize, allowed: RotationRange) -> SPInstance {
        let shape = SPolygon::new(vec![
            Point(0.0, 0.0),
            Point(w, 0.0),
            Point(w, h),
            Point(0.0, h),
        ])
        .expect(crate::dmsg!("valid rectangle"));
        let item = Item::new(0, (0) as u64, OriginalShape {
                shape,
                pre_transform: DTransformation::empty(),
                modify_mode: ShapeModifyMode::Inflate,
                modify_config: ShapeModifyConfig::default(),
            }, jagua_rs::geometry::AllowedOrientations::new(allowed, vec![]).unwrap(), None, SURROGATE).map(std::sync::Arc::new)
        .expect(crate::dmsg!("valid item"));
        let strip = Strip::new(200.0, CDE, ShapeModifyConfig::default(), 2000.0).expect(crate::dmsg!("strip"));
        SPInstance::new(vec![(item, qty)], strip)
    }

    /// S2C 는 회전을 끄므로 `new` 의 테이블은 각 1개다. 시드 해에 90° 배치가 있으면
    /// `with_seed` 가 그 각을 (중복 없이) 더하고, `rot_index_for` 가 회전된 기하를 가리킨다.
    #[test]
    fn with_seed_registers_seed_rotation_once() {
        let inst = rect_instance(40.0, 10.0, 2, RotationRange::Continuous);
        let cfg = AttachCfg::default();
        let mut prob = SPProblem::new(inst.clone()).expect(crate::dmsg!("strip forms a valid container"));
        prob.change_strip_width(500.0).expect(crate::dmsg!("strip width must be positive"));
        for x in [20.0f32, 120.0] {
            prob.place_item(SPPlacement {
                item_idx: 0,
                d_transf: DTransformation::new(PI / 2.0, (x, 20.0)),
            });
        }
        let seed = prob.save();

        let plain = Engine::new(&inst, &cfg, Stage::S2C);
        assert_eq!(plain.geom[0].angles.len(), 1, "S2C 기본 테이블은 각 1개");

        let eng = Engine::with_seed(&inst, &cfg, Stage::S2C, Some(&seed));
        assert_eq!(eng.geom[0].angles.len(), 2, "같은 시드 각은 한 번만 들어간다");
        assert_eq!(eng.geom[0].base.len(), 2);
        let r = rot_index_for(&eng, 0, PI / 2.0);
        assert_eq!(r, 1);
        let bb = eng.geom[0].base[r].bbox();
        let (w, h) = (bb.x_max - bb.x_min, bb.y_max - bb.y_min);
        assert!(h > w, "90° 기하는 세로가 길어야 한다: {w} x {h}");
        // 회전 테이블이 다르면 등가 클래스도 갈라져 NFP 캐시 키가 섞이지 않는다.
        assert_eq!(rot_index_for(&eng, 0, 0.0), 0);
    }

    /// `keep_smallest` 는 키 오름차순 상위 cap 개를 정렬된 상태로 남긴다.
    #[test]
    fn keep_smallest_keeps_sorted_prefix() {
        let mut v: Vec<(f32, f32, f32)> =
            [5.0, 1.0, 4.0, 2.0, 3.0, 0.0].iter().map(|&k| (k, k * 10.0, 0.0)).collect();
        keep_smallest(&mut v, 3);
        assert_eq!(v.iter().map(|c| c.0).collect::<Vec<_>>(), vec![0.0, 1.0, 2.0]);
        let mut w = v.clone();
        keep_smallest(&mut w, 10);
        assert_eq!(w.len(), 3);
        keep_smallest(&mut w, 0);
        assert!(w.is_empty());
    }
}
