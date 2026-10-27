//! Source compatibility for removed R&R and periodic swap operators.
//! Calls are inert even when an older host enables them. The NFP cache below
//! remains available to exact conflict-vector generation, which is independent.
use crate::config::{NfpIntegrationConfig, RuinRecreateConfig};
use crate::nfp_lbf::cache::{NfpCacheStats, NfpLbfCache, PreparedLibrary};
use crate::nfp_lbf::score::PlacementScoreWeights;
use crate::optimizer::separator::Separator;
use crate::structure::evaluator::ProfilePlacementConfig;
use crate::util::terminator::Terminator;
use jagua_rs::probs::spp::entities::SPInstance;
use std::time::Duration;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuinStrategy {
    Rightmost,
    LowDensity,
    Fragmented,
    GapTarget,
    DensitySeam,
    Random,
}
#[derive(Debug, Clone, Default)]
pub struct BandTabu;
impl BandTabu {
    pub fn new() -> Self {
        Self
    }
    pub fn clear(&mut self) {}
}
pub fn ruin_and_recreate(
    _sep: &mut Separator,
    _cfg: &RuinRecreateConfig,
    _strategy: RuinStrategy,
    _tabu: &mut BandTabu,
    _term: &impl Terminator,
) -> usize {
    0
}

pub fn ruin_and_recreate_with_nfp(
    _sep: &mut Separator,
    _cfg: &RuinRecreateConfig,
    _strategy: RuinStrategy,
    _tabu: &mut BandTabu,
    _term: &impl Terminator,
    _nfp: Option<&mut NfpReinsertionCache>,
) -> usize {
    0
}

pub fn ruin_and_recreate_profile(
    _sep: &mut Separator,
    _cfg: &RuinRecreateConfig,
    _strategy: RuinStrategy,
    _tabu: &mut BandTabu,
    _profile_cfg: ProfilePlacementConfig,
) {
}

pub fn ruin_and_recreate_profile_cancellable(
    _sep: &mut Separator,
    _cfg: &RuinRecreateConfig,
    _strategy: RuinStrategy,
    _tabu: &mut BandTabu,
    _profile_cfg: ProfilePlacementConfig,
    _term: &impl Terminator,
) {
}

pub fn swap_pass(_sep: &mut Separator, _n_attempts: usize, _term: &impl Terminator) -> usize {
    0
}

pub struct NfpReinsertionCache {
    cache: NfpLbfCache,
    weights: PlacementScoreWeights,
}

impl NfpReinsertionCache {
    /// 인스턴스에서 캐시를 만든다. 준비에 성공한 형상이 하나도 없으면 `None`.
    pub fn new(
        instance: &SPInstance,
        rotations: &[i64],
        concave_simplify_tolerance: f32,
    ) -> Option<Self> {
        let library = PreparedLibrary::build(instance, rotations, concave_simplify_tolerance)?;
        Some(Self {
            cache: NfpLbfCache::new(library),
            weights: PlacementScoreWeights::default(),
        })
    }

    /// Removed reinsertion never prepares geometry.
    pub fn from_integration_config(
        _instance: &SPInstance,
        _cfg: &NfpIntegrationConfig,
    ) -> Option<Self> {
        None
    }

    /// 내부 NFP 캐시에 대한 가변 접근.
    ///
    /// 재삽입 말고 다른 NFP 소비자(예: Conflict Burst 의 정확한 분리 벡터)가 **같은 캐시를
    /// 공유**하기 위한 통로다. pair NFP 는 소비자가 달라도 동일하므로 따로 만들 이유가 없다.
    pub fn cache_mut(&mut self) -> &mut NfpLbfCache {
        &mut self.cache
    }

    /// 점수 가중치 교체(기본값은 [`PlacementScoreWeights::default`]).
    pub fn with_weights(mut self, weights: PlacementScoreWeights) -> Self {
        self.weights = weights;
        self
    }

    /// 콜드 캐시를 미리 채운다. 넣지 않아도 동작은 같다(miss 시 즉시 계산).
    pub fn prewarm(&mut self, budget: Duration) -> NfpCacheStats {
        self.cache.prewarm(budget)
    }

    /// 현재 캐시 계측치.
    pub fn stats(&self) -> NfpCacheStats {
        self.cache.stats()
    }
}

const NFP_ROTATION_DEG_EPS: f64 = 1.0e-3;
const NFP_MAX_ROTATIONS: usize = 8;
fn integer_degrees(rotation: f32) -> Option<i64> {
    let degrees = (rotation as f64).to_degrees();
    if !degrees.is_finite() {
        return None;
    }
    let rounded = degrees.round();
    ((degrees - rounded).abs() <= NFP_ROTATION_DEG_EPS).then(|| (rounded as i64).rem_euclid(360))
}
pub(crate) fn instance_rotation_degrees(instance: &SPInstance) -> Option<Vec<i64>> {
    use jagua_rs::geometry::geo_enums::RotationRange;

    let mut degrees = vec![0i64];
    for (item, _) in instance.items.iter() {
        match item.allowed_orientations().rotations() {
            RotationRange::None => {}
            // 자유 회전은 열거할 수 없다. 직교 4자세만 준비한다.
            RotationRange::Continuous => degrees.extend_from_slice(&[90, 180, 270]),
            RotationRange::Discrete(set) => {
                for rotation in set.iter() {
                    degrees.push(integer_degrees(*rotation)?);
                }
            }
        }
    }
    degrees.sort_unstable();
    degrees.dedup();
    (degrees.len() <= NFP_MAX_ROTATIONS).then_some(degrees)
}
