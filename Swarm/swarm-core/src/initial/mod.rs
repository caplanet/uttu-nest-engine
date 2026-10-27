//! TASK N01 — 신규 초기배치 엔진.
//!
//! 기존 초기배치 경로(`optimizer/lbf.rs`, `structure/*`, `sample/exact_nfp.rs`,
//! `sample/raster_nfp.rs`)와 완전히 독립이다. 의존은 `jagua_rs` + `rand`,
//! 그리고 G단계의 `microlp` 뿐이다.

pub mod attach;
pub mod compact;
pub mod incumbent;
pub mod nfp;
pub mod raster;

pub use attach::{
    AttachStats, EscapeCfg, GlsReport, GroupStats, Interlock, SeparateReport, ShapeClass, ShrinkReport,
    ShrinkStep, analyze_classes,
    build_initial, build_with_groups, shrink_and_separate, shrink_and_separate_esc, stuck_probe,
    build_initial_restarts, build_initial_staged, build_initial_staged_cc, separate_at_width,
    CompactInfo, SlideCompactCfg, compact_probe, shrink_compact, shrink_compact_ctl, shrink_compact_seeded_ctl,
};
pub use compact::{CompactCfg, CompactStats};

/// 배치 순서 정렬 키.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Order {
    /// 면적 내림차순
    Area,
    /// 최대 지름(diameter) 내림차순
    Diameter,
    /// 오목도(볼록껍질면적 / 실면적) 내림차순
    Concavity,
}

/// 구현 단계. 하네스가 단계별 누적 기여도를 측정할 수 있도록 노출한다.
/// 값이 클수록 앞 단계의 기능을 모두 포함한다(누적).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Stage {
    /// bbox 앵커 (임시 배선 검증)
    A,
    /// NFP 경계 정점 후보
    B,
    /// 이분 탐색 접촉 슬라이딩
    C,
    /// 점수식 전체
    D,
    /// 회전 24방향
    E,
    /// 비볼록 NFP (볼록조각 배열)
    F,
    /// 압축 LP 1회
    G1,
    /// 반복 압축
    G2,
    /// 롤백
    G3,
    /// 재시작
    G4,
    /// TASK N02C — C 위에서 점수식 무차원화 + 슬라이딩 후 채점.
    /// 비볼록 NFP / 회전 / LP 압축은 전부 끈 상태다(N02C 금지 항목).
    S2C,
}

impl Stage {
    /// NFP 경계 정점을 후보로 쓰는가 (B 이상)
    pub fn use_nfp(self) -> bool {
        self >= Stage::B
    }
    /// 접촉 슬라이딩을 하는가 (C 이상)
    pub fn use_slide(self) -> bool {
        self >= Stage::C
    }
    /// 점수식 전체를 쓰는가 (D 이상)
    pub fn use_full_score(self) -> bool {
        self >= Stage::D
    }
    /// 회전 테이블을 쓰는가 (E 이상, S2C 제외)
    pub fn use_rotations(self) -> bool {
        self >= Stage::E && self != Stage::S2C
    }
    /// 비볼록 NFP(볼록조각 배열)를 쓰는가 (F 이상, S2C 제외)
    pub fn use_nonconvex_nfp(self) -> bool {
        self >= Stage::F && self != Stage::S2C
    }
    /// 압축을 하는가 (G1 이상, S2C 제외)
    pub fn use_compaction(self) -> bool {
        self >= Stage::G1 && self != Stage::S2C
    }
}

/// 배치 구조 (N02C 2C-B 비교 항목).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Structure {
    /// 매 반복 { 미배치 전체 } x { 배치 전체 } 후보에서 전역 최선 1개 배치. O(n^3)
    Global,
    /// 정렬 순서대로 한 조각씩, 각 조각마다 최선 위치 배치. O(n^2)
    Sorted,
}

#[derive(Clone, Copy, Debug)]
pub struct AttachCfg {
    pub order: Order,
    pub cand_cap: usize,
    pub slide_iters: u32,
    pub rotations: u32,
    pub nfp_edge_splits: u32,
    pub nfp_max_pieces: usize,
    pub eff_area_factor: f32,
    pub eff_area_factor_max: f32,
    pub score_factor: f32,
    pub score_factor_max: f32,
    pub lbf_x_weight: f32,
    pub lbf_y_weight: f32,

    // ---- TASK N02C ----------------------------------------------------
    /// 2C-A: 점수 세 항을 무차원화한다 (waste/item_area, len_gain/H, lbf/H).
    pub normalize_score: bool,
    /// 2C-A: 슬라이딩 후 좌표로 점수를 매긴다.
    pub score_after_slide: bool,
    /// 2C-B: 한 쌍에서 실제로 채점하는 후보 수 상한 (중복 제거 후 기준).
    pub cand_eval_cap: usize,
    /// 2C-B: 배치 구조.
    pub structure: Structure,
    /// 2C-C: 작업 스트립 폭을 고정한다 (None = 자유 폭).
    pub fixed_width: Option<f32>,
    /// 2C-D: 미배치 조각 강제 삽입을 수행한다.
    pub force_insert: bool,
    pub insert_try_cap: usize,
    pub insert_depth: u32,
    pub insert_time_budget_ms: u64,
    /// 선택 창. `Structure::Global` 에서 정렬 순서 상위 `pool_window` 개의 미배치 조각만
    /// 후보로 본다. 0 = 전체(순수 전역 선택), 1 = 사실상 `Sorted`.
    /// 전역 선택은 위치 항이 작은 조각을 항상 고르므로(64% 실측), 큰 것부터라는
    /// 구조를 창으로 유지하면서 창 안에서만 "가장 잘 맞는 조각" 을 고른다.
    pub pool_window: usize,
    /// 선택 점수의 크기 편향. 전역 선택(3b)에서만 쓴다: `score - bias * area/area_max`.
    /// 위치 항이 작은 조각을 고르는 편향을 상쇄한다. 0 = 끔.
    pub select_size_bias: f32,
    /// 선택 점수에서 좌하단 항의 배율 (1 = 배치 점수와 같음, 0 = 맞춤 항만 본다).
    pub select_lbf_scale: f32,
    /// 래스터 맞춤 평가의 칸 크기(mm). 0 = 끔(bbox 증가량 낭비). 켜면 `waste` 가
    /// "bbox 안쪽 빈 칸 + 왼쪽/아래 그림자 빈 칸" 의 면적으로 바뀐다 (`raster` 모듈).
    pub raster_fit_cell: f32,
    /// 그림자 탐색 상한(칸). 열린 공간을 무한 벌점으로 세지 않기 위한 값.
    pub raster_shadow_cap: usize,
}

impl Default for AttachCfg {
    fn default() -> Self {
        AttachCfg {
            order: Order::Area,
            cand_cap: 512,
            slide_iters: 20,
            rotations: 24,
            nfp_edge_splits: 4,
            nfp_max_pieces: 64,
            eff_area_factor: 1.0,
            eff_area_factor_max: -1.0,
            score_factor: 1.0,
            score_factor_max: -1.0,
            lbf_x_weight: 10.0,
            lbf_y_weight: 1.0,

            normalize_score: false,
            score_after_slide: false,
            cand_eval_cap: 3,
            structure: Structure::Global,
            fixed_width: None,
            force_insert: false,
            insert_try_cap: 8,
            insert_depth: 2,
            insert_time_budget_ms: 3000,
            pool_window: 0,
            select_size_bias: 0.0,
            select_lbf_scale: 1.0,
            raster_fit_cell: 0.0,
            raster_shadow_cap: 64,
        }
    }
}

impl AttachCfg {
    /// N02C 2C-A 기본 설정 (C단계 기능 + 점수식 수리).
    pub fn n02c() -> Self {
        AttachCfg {
            normalize_score: true,
            score_after_slide: true,
            cand_eval_cap: 8,
            ..AttachCfg::default()
        }
    }
}

/// `*_max < 0.0` 이면 클램프 비활성. 기본값은 클램프를 끈 상태다.
#[inline(always)]
pub fn clamp_opt(v: f32, max: f32) -> f32 {
    if max < 0.0 { v } else { v.min(max) }
}
