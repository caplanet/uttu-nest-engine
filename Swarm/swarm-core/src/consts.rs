use crate::sample::search::SampleConfig;
use jagua_rs::io::svg::{SvgDrawOptions, SvgLayoutTheme};

// GLS 가중치 최대 증가율
pub const GLS_WEIGHT_MAX_INC_RATIO: f32 = 2.0;
// GLS 가중치 최소 증가율
pub const GLS_WEIGHT_MIN_INC_RATIO: f32 = 1.2;
// GLS 가중치 감소율
pub const GLS_WEIGHT_DECAY: f32 = 0.95;
// GLS 가중치 상한. 장시간 탐색에서 반복 곱셈이 f32 범위를 넘지 않게 한다.
pub const GLS_WEIGHT_MAX: f32 = 1.0e6;

// 겹침 허용 오차(0이면 딱 겹쳐도됨) 0.01 = 100mm 기준 1mm 이내 겹침만 허용
pub const OVERLAP_PROXY_EPSILON_DIAM_RATIO: f32 = 0.01;

// 적응형 스템으로 충돌 후 효율 개선 성공시 스템 가속.
pub const CD_STEP_SUCCESS: f32 = 1.1;

// 적응형 스템으로 충돌 후 효율 개선 실패시 스템 감속.
pub const CD_STEP_FAIL: f32 = 0.5;

// 빠른 좌표 하강법 초기/최소 스텝 크기 비율(초기 : 아이템의 25%프로 스텝 점프, 최소 : 2%).
pub const PRE_REFINE_CD_TL_RATIOS: (f32, f32) = (0.25, 0.02);

// 빠른 좌표 하강법 초기/최소 회전 스텝(초기 : 5도, 최소 : 1도).
pub const PRE_REFINE_CD_R_STEPS: (f32, f32) = (f32::to_radians(5.0), f32::to_radians(1.0));

// 정밀 좌표 하강법 초기/최소 스텝 크기 비율(초기 : 아이템의 1%프로 스텝 점프, 최소 : 0.1%).
pub const SND_REFINE_CD_TL_RATIOS: (f32, f32) = (0.01, 0.001);

// 정밀 좌표 하강법 초기/최소 회전 스텝(초기 : 0.5도, 최소 : 0.05도).
pub const SND_REFINE_CD_R_STEPS: (f32, f32) = (f32::to_radians(0.5), f32::to_radians(0.05));

// 유일한(중복 방지) 샘플 Threshold(중심점 5%)
pub const UNIQUE_SAMPLE_THRESHOLD: f32 = 0.05;

// 기본 탐색 시간 비율
pub const DEFAULT_EXPLORE_TIME_RATIO: f32 = 0.8;

// 기본 압축 시간 비율
pub const DEFAULT_COMPRESS_TIME_RATIO: f32 = 0.2;

// 탐색 단계 충돌 해소 실패 최대치
pub const DEFAULT_MAX_CONSEQ_FAILS_EXPL: usize = 10;

// 압축 단계 충돌 실패 비율
pub const DEFAULT_FAIL_DECAY_RATIO_CMPR: f32 = 0.9;

// 삭제하기
pub const DRAW_OPTIONS: SvgDrawOptions = SvgDrawOptions {
    theme: SvgLayoutTheme::GRAY,
    quadtree: false,
    surrogate: false,
    highlight_collisions: true,
    draw_cd_shapes: false,
    highlight_cd_shapes: true,
};

// LBF 휴리스틱 샘플 설정
pub const LBF_SAMPLE_CONFIG: SampleConfig = SampleConfig {
    n_container_samples: 1000,
    n_focussed_samples: 25, //0, //0이 제일 빠르다.
    n_coord_descents: 3,
    left_bias_gain: 2.0,
    tilt_range: 0.0,
};
