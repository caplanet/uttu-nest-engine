use crate::consts::{
    PRE_REFINE_CD_R_STEPS, PRE_REFINE_CD_TL_RATIOS, SND_REFINE_CD_R_STEPS, SND_REFINE_CD_TL_RATIOS,
    UNIQUE_SAMPLE_THRESHOLD,
};
use crate::eval::sample_eval::{SampleEval, SampleEvaluator};
use crate::prepared_geometry::{PreparedGeometry, PreparedItemGeometry};
use crate::sample::best_samples::{BestSamples, dtransfs_are_similar};
use crate::sample::coord_descent::{
    CDConfig, refine_coord_desc_around_rotation_with_control, refine_coord_desc_with_control,
};
use crate::sample::uniform_sampler::UniformBBoxSampler;
use jagua_rs::entities::{Item, Layout, PItemKey};
use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::geo_enums::RotationRange;
use rand::{Rng, RngExt};

#[derive(Debug, Clone, Copy)]
pub struct SampleConfig {
    pub n_container_samples: usize,
    pub n_focussed_samples: usize,
    pub n_coord_descents: usize,
    /// 컨테이너 전역 샘플의 좌측 편향 최대 강도 (capmods 빌드에서만 반영).
    /// 0.0 = 균등 분포. 값이 클수록, 그리고 아이템이 작을수록 x가 작은 쪽(좌측)에
    /// 샘플이 몰린다. 기본값 2.0 (작은 아이템 기준 u³ 분포, 질량 중심 25% 지점).
    pub left_bias_gain: f32,
    /// discrete 회전 아이템에 허용하는 소각도 틸트 범위 (rad, 허용 회전 기준 ±값).
    /// 0.0 = 비활성 (기본). 켜면 좌표 하강의 회전(wiggle) 축이 discrete 아이템에도
    /// 활성화되고, 결과 회전은 가장 가까운 선언 회전 ± 이 값 안으로 클램프된다.
    /// 결방향 등으로 정확한 각도가 필요한 원단에서는 0으로 두어야 한다.
    pub tilt_range: f32,
}

/// Algorithm 6 and Figure 7 from https://doi.org/10.48550/arXiv.2509.13329
pub fn search_placement(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    evaluator: impl SampleEvaluator,
    sample_config: SampleConfig,
    rng: &mut impl Rng,
) -> (Option<(DTransformation, SampleEval)>, usize) {
    let mut never_stop = || false;
    let (best, _second, n_evals) = search_impl(
        l,
        item,
        ref_pk,
        &[],
        evaluator,
        sample_config,
        1,
        false,
        rng,
        &mut never_stop,
    );
    (best, n_evals)
}

pub fn search_placement_with_control(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    evaluator: impl SampleEvaluator,
    sample_config: SampleConfig,
    rng: &mut impl Rng,
    should_stop: &mut dyn FnMut() -> bool,
) -> (Option<(DTransformation, SampleEval)>, usize) {
    let (best, _second, n_evals) = search_impl(
        l,
        item,
        ref_pk,
        &[],
        evaluator,
        sample_config,
        1,
        false,
        rng,
        should_stop,
    );
    (best, n_evals)
}

/// Profile-construction variant of [`search_placement`] which deliberately
/// selects one of the ranked top-k basins before the final fine coordinate
/// descent.  With `top_k == 1` this is identical to the normal deterministic
/// best-sample path.  The supplied evaluator remains the authority for both
/// feasibility and ranking; callers use a profile-aware evaluator whose
/// invalid candidates have already failed exact CDE.
///
/// This small stochastic choice is intended for P1 multi-start construction:
/// different starts can retain different useful local structures without
/// weakening the exact feasibility boundary.
pub fn search_placement_stochastic_top_k(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    evaluator: impl SampleEvaluator,
    sample_config: SampleConfig,
    top_k: usize,
    rng: &mut impl Rng,
) -> (Option<(DTransformation, SampleEval)>, usize) {
    let mut never_stop = || false;
    let (selected, _second, n_evals) = search_impl(
        l,
        item,
        ref_pk,
        &[],
        evaluator,
        sample_config,
        top_k.max(1),
        false,
        rng,
        &mut never_stop,
    );
    (selected, n_evals)
}

pub fn search_placement_stochastic_top_k_with_control(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    evaluator: impl SampleEvaluator,
    sample_config: SampleConfig,
    top_k: usize,
    rng: &mut impl Rng,
    should_stop: &mut dyn FnMut() -> bool,
) -> (Option<(DTransformation, SampleEval)>, usize) {
    let (selected, _second, n_evals) = search_impl(
        l,
        item,
        ref_pk,
        &[],
        evaluator,
        sample_config,
        top_k.max(1),
        false,
        rng,
        should_stop,
    );
    (selected, n_evals)
}

/// Seeded counterpart of [`search_placement_stochastic_top_k`].  P0 motif
/// poses are merely extra proposals: they compete with the normal samples and
/// still pass through the caller's exact-authorized evaluator.
pub fn search_placement_seeded_stochastic_top_k(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    seeds: &[DTransformation],
    evaluator: impl SampleEvaluator,
    sample_config: SampleConfig,
    top_k: usize,
    rng: &mut impl Rng,
) -> (Option<(DTransformation, SampleEval)>, usize) {
    let mut never_stop = || false;
    let (selected, _second, n_evals) = search_impl(
        l,
        item,
        ref_pk,
        seeds,
        evaluator,
        sample_config,
        top_k.max(1),
        false,
        rng,
        &mut never_stop,
    );
    (selected, n_evals)
}

pub fn search_placement_seeded_stochastic_top_k_with_control(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    seeds: &[DTransformation],
    evaluator: impl SampleEvaluator,
    sample_config: SampleConfig,
    top_k: usize,
    rng: &mut impl Rng,
    should_stop: &mut dyn FnMut() -> bool,
) -> (Option<(DTransformation, SampleEval)>, usize) {
    let (selected, _second, n_evals) = search_impl(
        l,
        item,
        ref_pk,
        seeds,
        evaluator,
        sample_config,
        top_k.max(1),
        false,
        rng,
        should_stop,
    );
    (selected, n_evals)
}

/// [`search_placement`]과 동일하지만, 최선 배치와 함께 **위치가 뚜렷이 다른 2순위 후보**의
/// 평가값도 반환한다 (fork 전용). regret 기반 재삽입에서 "최선 자리를 놓쳤을 때의 손해"를
/// 추정하는 데 쓰인다. 정상 완료 시 두 후보 모두 같은 final 좌표 하강을 거친다.
pub fn search_placement_top2(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    evaluator: impl SampleEvaluator,
    sample_config: SampleConfig,
    rng: &mut impl Rng,
) -> (
    Option<(DTransformation, SampleEval)>,
    Option<SampleEval>,
    usize,
) {
    let mut never_stop = || false;
    search_impl(
        l,
        item,
        ref_pk,
        &[],
        evaluator,
        sample_config,
        1,
        true,
        rng,
        &mut never_stop,
    )
}

pub fn search_placement_top2_with_control(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    evaluator: impl SampleEvaluator,
    sample_config: SampleConfig,
    rng: &mut impl Rng,
    should_stop: &mut dyn FnMut() -> bool,
) -> (
    Option<(DTransformation, SampleEval)>,
    Option<SampleEval>,
    usize,
) {
    search_impl(
        l,
        item,
        ref_pk,
        &[],
        evaluator,
        sample_config,
        1,
        true,
        rng,
        should_stop,
    )
}

/// [`search_placement`]에 **외부에서 계산한 후보(seed)** 를 먼저 투입하는 변형 (fork 전용).
///
/// 시드도 다른 샘플과 똑같이 `evaluator`로 평가되므로, 시드가 실제로 유효한지는 여기서
/// 검증된다. 즉 시드 생성기(예: 래스터 NFP)가 근사적이어도 잘못된 배치가 통과할 수 없다.
pub fn search_placement_seeded(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    seeds: &[DTransformation],
    evaluator: impl SampleEvaluator,
    sample_config: SampleConfig,
    rng: &mut impl Rng,
) -> (Option<(DTransformation, SampleEval)>, usize) {
    let mut never_stop = || false;
    let (best, _second, n_evals) = search_impl(
        l,
        item,
        ref_pk,
        seeds,
        evaluator,
        sample_config,
        1,
        false,
        rng,
        &mut never_stop,
    );
    (best, n_evals)
}

pub fn search_placement_seeded_with_control(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    seeds: &[DTransformation],
    evaluator: impl SampleEvaluator,
    sample_config: SampleConfig,
    rng: &mut impl Rng,
    should_stop: &mut dyn FnMut() -> bool,
) -> (Option<(DTransformation, SampleEval)>, usize) {
    let (best, _second, n_evals) = search_impl(
        l,
        item,
        ref_pk,
        seeds,
        evaluator,
        sample_config,
        1,
        false,
        rng,
        should_stop,
    );
    (best, n_evals)
}

#[allow(clippy::too_many_arguments)]
pub fn search_placement_prepared_with_control(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    evaluator: impl SampleEvaluator,
    sample_config: SampleConfig,
    rng: &mut impl Rng,
    should_stop: &mut dyn FnMut() -> bool,
    prepared: &PreparedGeometry,
) -> (Option<(DTransformation, SampleEval)>, usize) {
    let (best, _second, n_evals) = search_impl_prepared(
        l,
        item,
        ref_pk,
        &[],
        evaluator,
        sample_config,
        1,
        false,
        rng,
        should_stop,
        Some(prepared),
    );
    (best, n_evals)
}

#[allow(clippy::too_many_arguments)]
pub fn search_placement_top2_prepared_with_control(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    evaluator: impl SampleEvaluator,
    sample_config: SampleConfig,
    rng: &mut impl Rng,
    should_stop: &mut dyn FnMut() -> bool,
    prepared: &PreparedGeometry,
) -> (
    Option<(DTransformation, SampleEval)>,
    Option<SampleEval>,
    usize,
) {
    search_impl_prepared(
        l,
        item,
        ref_pk,
        &[],
        evaluator,
        sample_config,
        1,
        true,
        rng,
        should_stop,
        Some(prepared),
    )
}

#[allow(clippy::too_many_arguments)]
pub fn search_placement_stochastic_top_k_prepared_with_control(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    evaluator: impl SampleEvaluator,
    sample_config: SampleConfig,
    top_k: usize,
    rng: &mut impl Rng,
    should_stop: &mut dyn FnMut() -> bool,
    prepared: &PreparedGeometry,
) -> (Option<(DTransformation, SampleEval)>, usize) {
    let (selected, _second, n_evals) = search_impl_prepared(
        l,
        item,
        ref_pk,
        &[],
        evaluator,
        sample_config,
        top_k.max(1),
        false,
        rng,
        should_stop,
        Some(prepared),
    );
    (selected, n_evals)
}

#[allow(clippy::too_many_arguments)]
pub fn search_placement_seeded_prepared_with_control(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    seeds: &[DTransformation],
    evaluator: impl SampleEvaluator,
    sample_config: SampleConfig,
    rng: &mut impl Rng,
    should_stop: &mut dyn FnMut() -> bool,
    prepared: &PreparedGeometry,
) -> (Option<(DTransformation, SampleEval)>, usize) {
    let (best, _second, n_evals) = search_impl_prepared(
        l,
        item,
        ref_pk,
        seeds,
        evaluator,
        sample_config,
        1,
        false,
        rng,
        should_stop,
        Some(prepared),
    );
    (best, n_evals)
}

#[allow(clippy::too_many_arguments)]
pub fn search_placement_seeded_stochastic_top_k_prepared_with_control(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    seeds: &[DTransformation],
    evaluator: impl SampleEvaluator,
    sample_config: SampleConfig,
    top_k: usize,
    rng: &mut impl Rng,
    should_stop: &mut dyn FnMut() -> bool,
    prepared: &PreparedGeometry,
) -> (Option<(DTransformation, SampleEval)>, usize) {
    let (selected, _second, n_evals) = search_impl_prepared(
        l,
        item,
        ref_pk,
        seeds,
        evaluator,
        sample_config,
        top_k.max(1),
        false,
        rng,
        should_stop,
        Some(prepared),
    );
    (selected, n_evals)
}

fn search_impl(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    seeds: &[DTransformation],
    evaluator: impl SampleEvaluator,
    sample_config: SampleConfig,
    selection_top_k: usize,
    refine_second: bool,
    rng: &mut impl Rng,
    should_stop: &mut dyn FnMut() -> bool,
) -> (
    Option<(DTransformation, SampleEval)>,
    Option<SampleEval>,
    usize,
) {
    search_impl_prepared(
        l,
        item,
        ref_pk,
        seeds,
        evaluator,
        sample_config,
        selection_top_k,
        refine_second,
        rng,
        should_stop,
        None,
    )
}

fn search_impl_prepared(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    seeds: &[DTransformation],
    mut evaluator: impl SampleEvaluator,
    sample_config: SampleConfig,
    selection_top_k: usize,
    refine_second: bool,
    rng: &mut impl Rng,
    should_stop: &mut dyn FnMut() -> bool,
    prepared: Option<&PreparedGeometry>,
) -> (
    Option<(DTransformation, SampleEval)>,
    Option<SampleEval>,
    usize,
) {
    #[cfg(feature = "init-profile")]
    let _profile = crate::init_profile::Scope::new("detail.search_placement");

    let item_min_dim = f32::min(item.shape_cd().bbox().width(), item.shape_cd().bbox().height());

    let mut best_samples = BestSamples::new(
        sample_config.n_coord_descents.max(selection_top_k).max(1),
        item_min_dim * UNIQUE_SAMPLE_THRESHOLD,
    );

    // Validate once per search, then intersect both ranges without transforming
    // any geometry. Public callers with replaced items use a fresh local entry.
    let local_prepared;
    let item_prepared = match prepared.and_then(|p| p.get(item)) {
        Some(entry) => entry,
        None => {
            local_prepared = PreparedItemGeometry::new(item);
            &local_prepared
        }
    };

    //Create the two uniform samplers, one focussed around the reference placement, one for the whole container
    let focussed_sampler = match ref_pk {
        Some(ref_pk) => {
            //Add the current placement (and evaluation) as a candidate
            let dt = l.placed_items()[ref_pk].d_transf();
            let eval = evaluator.evaluate_sample(dt, Some(best_samples.upper_bound()));
            best_samples.report(dt, eval);

            //Create a uniform sampler focussed around the current placement
            let pi_bbox = l.placed_items()[ref_pk].shape().bbox();
            UniformBBoxSampler::from_prepared(pi_bbox, l.container().outer_cd().bbox(), item_prepared)
        }
        None => None,
    };
    let container_sampler = UniformBBoxSampler::from_prepared(
        l.container().outer_cd().bbox(),
        l.container().outer_cd().bbox(),
        item_prepared,
    );


    #[cfg(feature = "init-profile")]
    let mut _profile_search = crate::init_profile::Scope::new("detail.search_sampling");
    //Evaluate the externally supplied seeds first (fork: raster NFP candidates)
    for &dt in seeds {
        if should_stop() {
            break;
        }
        let eval = evaluator.evaluate_sample(dt, Some(best_samples.upper_bound()));
        best_samples.report(dt, eval);
    }

    //Perform the focussed sampling
    if let Some(focussed_sampler) = focussed_sampler {
        for _ in 0..sample_config.n_focussed_samples {
            if should_stop() {
                break;
            }
            let dt = focussed_sampler.sample(rng);
            let eval = evaluator.evaluate_sample(dt, Some(best_samples.upper_bound()));
            best_samples.report(dt, eval);
        }
    }

    //Perform the container-wide sampling
    if let Some(container_sampler) = container_sampler {
        // capmods: 컨테이너 전역 샘플에 좌측 편향을 건다. 작은 아이템일수록 편향이
        // 강해져, 좌측의 빈 포켓에 작은 조각이 들어갈 확률이 높아진다.
        #[cfg(feature = "capmods")]
        let left_bias_k = left_bias_exponent(l, item, sample_config.left_bias_gain);

        for _ in 0..sample_config.n_container_samples {
            if should_stop() {
                break;
            }
            #[cfg(not(feature = "capmods"))]
            let dt = container_sampler.sample(rng);
            #[cfg(feature = "capmods")]
            let dt = if left_bias_k > 1.0 {
                container_sampler.sample_x_biased(rng, left_bias_k)
            } else {
                container_sampler.sample(rng)
            };

            let eval = evaluator.evaluate_sample(dt, Some(best_samples.upper_bound()));
            best_samples.report(dt, eval);
        }
    }

    //Refine some of the best random samples to a local minimum in two steps:

    #[cfg(feature = "init-profile")]
    _profile_search.next("detail.search_coordinate_descent");
    //1. Do a first refinement of all 'best samples' using coordinate descent
    for start in best_samples.samples.clone() {
        if should_stop() {
            break;
        }
        let descended = refine_coord_desc_for_allowed_rotation_with_control(
            start,
            &mut evaluator,
            prerefine_cd_config(item, sample_config.tilt_range),
            &item.allowed_orientations().rotations(),
            rng,
            should_stop,
        );
        best_samples.report(descended.0, descended.1);
    }

    // 2순위 후보(위치가 뚜렷이 다른 차선책) — top-2 경로에서는 최선 후보와
    // 동일한 final coordinate-descent 정제를 적용한 뒤 regret 계산에 사용한다.
    let second_candidate = best_samples.samples.get(1).cloned();

    //2. Take the best one and do an even finer coordinate descent refinement
    let selected = if selection_top_k <= 1 || best_samples.samples.len() <= 1 {
        best_samples.best()
    } else {
        let n = selection_top_k.min(best_samples.samples.len());
        Some(best_samples.samples[rng.random_range(0..n)].clone())
    };
    let (final_sample, second_best) = final_refine_candidates(
        selected,
        second_candidate,
        refine_second,
        &mut evaluator,
        final_refine_cd_config(item, sample_config.tilt_range),
        &item.allowed_orientations().rotations(),
        best_samples.unique_thresh,
        rng,
        should_stop,
    );

    #[cfg(feature = "init-profile")]
    crate::init_profile::count("search.evaluations", evaluator.n_evals());
    (final_sample, second_best, evaluator.n_evals())
}

#[allow(clippy::too_many_arguments)]
fn final_refine_candidates(
    selected: Option<(DTransformation, SampleEval)>,
    second_candidate: Option<(DTransformation, SampleEval)>,
    refine_second: bool,
    evaluator: &mut impl SampleEvaluator,
    cd_config: CDConfig,
    allowed_rotation: &RotationRange,
    unique_thresh: f32,
    rng: &mut impl Rng,
    should_stop: &mut dyn FnMut() -> bool,
) -> (Option<(DTransformation, SampleEval)>, Option<SampleEval>) {
    let selected = match selected {
        None => return (None, None),
        Some(selected) if should_stop() => {
            // A stopped top-2 search must not compare candidates refined to
            // different levels. Omit the second rather than exaggerating regret.
            let second = if refine_second {
                None
            } else {
                second_candidate.map(|(_, eval)| eval)
            };
            return (Some(selected), second);
        }
        Some(selected) => refine_coord_desc_for_allowed_rotation_with_control(
            selected,
            evaluator,
            cd_config,
            allowed_rotation,
            rng,
            should_stop,
        ),
    };

    if !refine_second {
        return (Some(selected), second_candidate.map(|(_, eval)| eval));
    }

    let Some(second_candidate) = second_candidate else {
        return (Some(selected), None);
    };
    if should_stop() {
        // The best candidate is final-refined, but the second is not. Returning
        // no second preserves refinement parity and lets the caller honour stop.
        return (Some(selected), None);
    }

    let second = refine_coord_desc_for_allowed_rotation_with_control(
        second_candidate,
        evaluator,
        cd_config,
        allowed_rotation,
        rng,
        should_stop,
    );
    if should_stop() {
        return (Some(selected), None);
    }
    let (best, second_eval) = rank_refined_top_two(selected, second, unique_thresh);
    (Some(best), Some(second_eval))
}

fn rank_refined_top_two(
    first: (DTransformation, SampleEval),
    second: (DTransformation, SampleEval),
    unique_thresh: f32,
) -> ((DTransformation, SampleEval), SampleEval) {
    let (best, runner_up) = if second.1 < first.1 {
        (second, first)
    } else {
        (first, second)
    };

    // Two distinct pre-refinement basins can coalesce under final CD. They no
    // longer represent two placement choices, so their regret must be zero.
    let runner_up_eval = if dtransfs_are_similar(best.0, runner_up.0, unique_thresh, unique_thresh)
    {
        best.1
    } else {
        runner_up.1
    };
    (best, runner_up_eval)
}

/// capmods 전용: 아이템 크기에 따른 좌측 편향 지수를 계산한다.
/// 컨테이너 폭 대비 아이템 최대 변이 작을수록 지수가 커져(최대 `1 + max_gain`)
/// 좌측 편향이 강해진다. 큰 아이템은 지수가 1에 가까워 사실상 균등 분포를 유지한다.
/// `max_gain <= 0`이면 1.0(균등 분포)을 반환한다.
#[cfg(feature = "capmods")]
fn left_bias_exponent(l: &Layout, item: &Item, max_gain: f32) -> f32 {
    if max_gain.is_nan() || max_gain <= 0.0 {
        return 1.0;
    }
    let cont_w = l.container().outer_cd().bbox().width().max(f32::EPSILON);
    let item_dim = f32::max(item.shape_cd().bbox().width(), item.shape_cd().bbox().height());
    let size_ratio = (item_dim / cont_w).clamp(0.0, 1.0);
    1.0 + max_gain * (1.0 - size_ratio)
}

/// 회전 축(wiggle) 활성 여부와 클램프 범위를 결정한다.
/// 연속 회전: wiggle 무제한. discrete/None 회전 + 틸트 허용: 가장 가까운 선언 회전
/// ± `tilt_range`로 클램프.
/// 그 외(틸트 0): wiggle 비활성 — 기존 동작 그대로.
fn wiggle_mode(item: &Item, tilt_range: f32) -> (bool, Option<f32>) {
    match item.allowed_orientations().rotations() {
        RotationRange::Continuous => (true, None),
        _ if tilt_range > 0.0 => (true, Some(tilt_range)),
        _ => (false, None),
    }
}

#[cfg(test)]
fn refine_coord_desc_for_allowed_rotation(
    start: (DTransformation, SampleEval),
    evaluator: &mut impl SampleEvaluator,
    cd_config: CDConfig,
    allowed_rotation: &RotationRange,
    rng: &mut impl Rng,
) -> (DTransformation, SampleEval) {
    refine_coord_desc_for_allowed_rotation_with_control(
        start,
        evaluator,
        cd_config,
        allowed_rotation,
        rng,
        &mut || false,
    )
}

fn refine_coord_desc_for_allowed_rotation_with_control(
    start: (DTransformation, SampleEval),
    evaluator: &mut impl SampleEvaluator,
    cd_config: CDConfig,
    allowed_rotation: &RotationRange,
    rng: &mut impl Rng,
    should_stop: &mut dyn FnMut() -> bool,
) -> (DTransformation, SampleEval) {
    match nearest_rotation_anchor(allowed_rotation, start.0.rotation()) {
        Some(anchor) => refine_coord_desc_around_rotation_with_control(
            start,
            evaluator,
            cd_config,
            anchor,
            rng,
            should_stop,
        ),
        None => refine_coord_desc_with_control(start, evaluator, cd_config, rng, should_stop),
    }
}

/// Returns the immutable declared orientation nearest to `rotation` on the
/// circle. `None` means the item is continuously rotatable and needs no clamp.
fn nearest_rotation_anchor(allowed_rotation: &RotationRange, rotation: f32) -> Option<f32> {
    match allowed_rotation {
        RotationRange::None => Some(0.0),
        RotationRange::Continuous => None,
        RotationRange::Discrete(anchors) => Some(
            anchors
                .iter()
                .copied()
                .filter(|anchor| anchor.is_finite())
                .min_by(|a, b| {
                    circular_angle_distance(rotation, *a)
                        .total_cmp(&circular_angle_distance(rotation, *b))
                })
                .unwrap_or(0.0),
        ),
    }
}

fn circular_angle_distance(a: f32, b: f32) -> f32 {
    use std::f32::consts::PI;

    ((a - b + PI).rem_euclid(2.0 * PI) - PI).abs()
}

// hole filling 패스가 동일한 좌표 하강 설정을 재사용할 수 있도록 공개.
pub(crate) fn prerefine_cd_config(item: &Item, tilt_range: f32) -> CDConfig {
    let item_min_dim = f32::min(item.shape_cd().bbox().width(), item.shape_cd().bbox().height());
    let (wiggle, r_range) = wiggle_mode(item, tilt_range);
    CDConfig {
        t_step_init: item_min_dim * PRE_REFINE_CD_TL_RATIOS.0,
        t_step_limit: item_min_dim * PRE_REFINE_CD_TL_RATIOS.1,
        // 틸트 범위가 기본 스텝보다 좁으면 스텝도 그에 맞춰 줄인다
        r_step_init: r_range.map_or(PRE_REFINE_CD_R_STEPS.0, |rr| {
            rr.min(PRE_REFINE_CD_R_STEPS.0)
        }),
        r_step_limit: PRE_REFINE_CD_R_STEPS.1,
        wiggle,
        r_range,
    }
}

pub(crate) fn final_refine_cd_config(item: &Item, tilt_range: f32) -> CDConfig {
    let item_min_dim = f32::min(item.shape_cd().bbox().width(), item.shape_cd().bbox().height());
    let (wiggle, r_range) = wiggle_mode(item, tilt_range);
    CDConfig {
        t_step_init: item_min_dim * SND_REFINE_CD_TL_RATIOS.0,
        t_step_limit: item_min_dim * SND_REFINE_CD_TL_RATIOS.1,
        r_step_init: r_range.map_or(SND_REFINE_CD_R_STEPS.0, |rr| {
            rr.min(SND_REFINE_CD_R_STEPS.0)
        }),
        r_step_limit: SND_REFINE_CD_R_STEPS.1,
        wiggle,
        r_range,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sample::coord_descent::clamp_rotation_around_anchor;
    use rand::SeedableRng;
    use rand::rngs::Xoshiro256PlusPlus;
    use std::f32::consts::PI;

    #[derive(Default)]
    struct PreferPositiveRotation {
        n_evals: usize,
    }

    impl SampleEvaluator for PreferPositiveRotation {
        fn evaluate_sample(
            &mut self,
            dt: DTransformation,
            _upper_bound: Option<SampleEval>,
        ) -> SampleEval {
            self.n_evals += 1;
            SampleEval::Clear {
                loss: -dt.rotation(),
            }
        }

        fn n_evals(&self) -> usize {
            self.n_evals
        }
    }

    #[derive(Default)]
    struct PreferOrigin {
        n_evals: usize,
    }

    impl SampleEvaluator for PreferOrigin {
        fn evaluate_sample(
            &mut self,
            dt: DTransformation,
            _upper_bound: Option<SampleEval>,
        ) -> SampleEval {
            self.n_evals += 1;
            let (x, y) = dt.translation();
            SampleEval::Clear {
                loss: x * x + y * y,
            }
        }

        fn n_evals(&self) -> usize {
            self.n_evals
        }
    }

    fn rotation_only_config(tilt_range: f32) -> CDConfig {
        CDConfig {
            t_step_init: 0.0,
            t_step_limit: 1.0,
            r_step_init: tilt_range * 0.4,
            r_step_limit: tilt_range * 0.01,
            wiggle: true,
            r_range: Some(tilt_range),
        }
    }

    fn translation_config() -> CDConfig {
        CDConfig {
            t_step_init: 1.0,
            t_step_limit: 0.25,
            r_step_init: 0.0,
            r_step_limit: 1.0,
            wiggle: false,
            r_range: None,
        }
    }

    fn clear(loss: f32) -> SampleEval {
        SampleEval::Clear { loss }
    }

    #[test]
    fn final_top_two_refines_both_and_counts_every_evaluation() {
        let selected = (DTransformation::new(0.0, (3.0, 1.0)), clear(10.0));
        let second = (DTransformation::new(0.0, (-3.0, -1.0)), clear(10.0));
        let config = translation_config();
        let allowed = RotationRange::Continuous;

        let mut expected_evaluator = PreferOrigin::default();
        let mut expected_rng = Xoshiro256PlusPlus::seed_from_u64(91);
        let expected_first = refine_coord_desc_for_allowed_rotation(
            selected,
            &mut expected_evaluator,
            config,
            &allowed,
            &mut expected_rng,
        );
        let expected_second = refine_coord_desc_for_allowed_rotation(
            second,
            &mut expected_evaluator,
            config,
            &allowed,
            &mut expected_rng,
        );
        let expected = rank_refined_top_two(expected_first, expected_second, 0.25);

        let mut evaluator = PreferOrigin::default();
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(91);
        let mut polls = 0;
        let mut never_stop = || {
            polls += 1;
            false
        };
        let actual = final_refine_candidates(
            Some(selected),
            Some(second),
            true,
            &mut evaluator,
            config,
            &allowed,
            0.25,
            &mut rng,
            &mut never_stop,
        );

        assert_eq!(actual, (Some(expected.0), Some(expected.1)));
        assert_eq!(evaluator.n_evals(), expected_evaluator.n_evals());
        assert!(
            polls > 2,
            "cancellation must also be polled inside each refinement"
        );
    }

    #[test]
    fn coalesced_final_candidates_have_zero_regret() {
        let first = (DTransformation::new(0.0, (4.0, 2.0)), clear(1.25));
        let second = (
            DTransformation::new(0.0, (4.0 + 1.0e-4, 2.0 - 1.0e-4)),
            clear(8.0),
        );

        let (best, runner_up_eval) = rank_refined_top_two(first, second, 0.01);
        let (clear_best, clear_runner_up) = match (best.1, runner_up_eval) {
            (SampleEval::Clear { loss: best }, SampleEval::Clear { loss: runner_up }) => {
                (best, runner_up)
            }
            _ => panic!("{}", crate::dmsg!("expected two clear samples")),
        };
        assert_eq!(clear_runner_up - clear_best, 0.0);
    }

    #[test]
    fn stop_before_second_refinement_omits_mixed_level_regret() {
        let selected = (DTransformation::new(0.0, (3.0, 1.0)), clear(10.0));
        let second = (DTransformation::new(0.0, (-3.0, -1.0)), clear(10.0));
        let config = translation_config();
        let allowed = RotationRange::Continuous;

        let mut expected_evaluator = PreferOrigin::default();
        let mut expected_rng = Xoshiro256PlusPlus::seed_from_u64(17);
        let mut first_polls = 0;
        let expected_best = refine_coord_desc_for_allowed_rotation_with_control(
            selected,
            &mut expected_evaluator,
            config,
            &allowed,
            &mut expected_rng,
            &mut || {
                first_polls += 1;
                false
            },
        );

        let mut evaluator = PreferOrigin::default();
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(17);
        let mut polls = 0;
        let mut stop_before_second = || {
            polls += 1;
            polls >= first_polls + 2
        };
        let (best, second_eval) = final_refine_candidates(
            Some(selected),
            Some(second),
            true,
            &mut evaluator,
            config,
            &allowed,
            0.25,
            &mut rng,
            &mut stop_before_second,
        );

        assert_eq!(best, Some(expected_best));
        assert_eq!(second_eval, None);
        assert_eq!(evaluator.n_evals(), expected_evaluator.n_evals());
        assert_eq!(polls, first_polls + 2);
    }

    #[test]
    fn ordinary_search_final_refinement_keeps_legacy_rng_and_eval_count() {
        let selected = (DTransformation::new(0.0, (3.0, 1.0)), clear(10.0));
        let second = (DTransformation::new(0.0, (-3.0, -1.0)), clear(10.0));
        let config = translation_config();
        let allowed = RotationRange::Continuous;

        let mut expected_evaluator = PreferOrigin::default();
        let mut expected_rng = Xoshiro256PlusPlus::seed_from_u64(23);
        let expected_best = refine_coord_desc_for_allowed_rotation(
            selected,
            &mut expected_evaluator,
            config,
            &allowed,
            &mut expected_rng,
        );

        let mut evaluator = PreferOrigin::default();
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(23);
        let mut polls = 0;
        let mut never_stop = || {
            polls += 1;
            false
        };
        let (best, second_eval) = final_refine_candidates(
            Some(selected),
            Some(second),
            false,
            &mut evaluator,
            config,
            &allowed,
            0.25,
            &mut rng,
            &mut never_stop,
        );

        assert_eq!(best, Some(expected_best));
        assert_eq!(second_eval, Some(second.1));
        assert_eq!(evaluator.n_evals(), expected_evaluator.n_evals());
        assert!(polls > 1, "ordinary refinement must poll inside CD");
    }

    #[test]
    fn stop_inside_first_refinement_returns_its_best_without_a_mixed_regret() {
        let mut evaluator = PreferOrigin::default();
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(17);
        let mut polls = 0;
        let (best, second) = final_refine_candidates(
            Some((DTransformation::new(0.0, (3.0, 1.0)), clear(10.0))),
            Some((DTransformation::new(0.0, (-3.0, -1.0)), clear(10.0))),
            true,
            &mut evaluator,
            translation_config(),
            &RotationRange::Continuous,
            0.25,
            &mut rng,
            &mut || {
                polls += 1;
                polls >= 4
            },
        );
        assert_eq!(evaluator.n_evals(), 4);
        assert!(best.unwrap().1 <= clear(10.0));
        assert_eq!(second, None);
    }

    #[test]
    fn repeated_refinement_cannot_drift_outside_declared_tilt_range() {
        let allowed = RotationRange::None;
        let tilt_range = 0.1;
        let config = rotation_only_config(tilt_range);
        let mut evaluator = PreferPositiveRotation::default();
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(7);
        let mut dt = DTransformation::new(0.0, (0.0, 0.0));

        for _ in 0..16 {
            let eval = evaluator.evaluate_sample(dt, None);
            (dt, _) = refine_coord_desc_for_allowed_rotation(
                (dt, eval),
                &mut evaluator,
                config,
                &allowed,
                &mut rng,
            );
            assert!(
                circular_angle_distance(dt.rotation(), 0.0) <= tilt_range + 1.0e-6,
                "rotation {} drifted outside ±{} around the declared 0° anchor",
                dt.rotation(),
                tilt_range
            );
        }

        assert!(dt.rotation() > tilt_range * 0.9);
    }

    #[test]
    fn multiple_allowed_rotations_choose_the_nearest_anchor() {
        let allowed = RotationRange::Discrete(vec![-1.2, 0.25, 2.2]);
        let anchor = nearest_rotation_anchor(&allowed, 2.0).unwrap();
        assert!((anchor - 2.2).abs() <= f32::EPSILON);
    }

    #[test]
    fn nearest_anchor_and_clamp_handle_pi_wrap() {
        let anchor = -PI + 0.04;
        let allowed = RotationRange::Discrete(vec![0.0, anchor]);
        let rotation = PI - 0.02;

        let selected = nearest_rotation_anchor(&allowed, rotation).unwrap();
        assert!((selected - anchor).abs() <= f32::EPSILON);

        let unchanged = clamp_rotation_around_anchor(rotation, selected, 0.08);
        assert!(circular_angle_distance(unchanged, rotation) <= 1.0e-6);

        let clamped = clamp_rotation_around_anchor(PI - 0.20, selected, 0.08);
        assert!((circular_angle_distance(clamped, selected) - 0.08).abs() <= 1.0e-6);
    }

    #[test]
    fn no_rotation_uses_zero_anchor_and_continuous_remains_unbounded() {
        let near_two_pi = 2.0 * PI - 0.01;
        assert_eq!(
            nearest_rotation_anchor(&RotationRange::None, near_two_pi),
            Some(0.0)
        );
        assert_eq!(
            nearest_rotation_anchor(&RotationRange::Continuous, near_two_pi),
            None
        );
    }
}
