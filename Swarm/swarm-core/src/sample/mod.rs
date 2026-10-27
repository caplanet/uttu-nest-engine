pub(crate) mod best_samples;
pub(crate) mod coord_descent;
pub mod search;
pub mod uniform_sampler;

// CA Planet: LBF 초기 배치용 래스터 NFP 후보 생성기 (capmods 전용).
#[cfg(feature = "capmods")]
pub mod raster_nfp;
// CA Planet: exact NFP — 래스터 후보를 접촉 경계까지 슬라이딩 (capmods 전용).
#[cfg(feature = "capmods")]
pub mod exact_nfp;
/// 탐색 단계용 NFP 후보 샘플러 (`NfpIntegrationConfig::nfp_sampler`).
pub mod nfp_sampler;
