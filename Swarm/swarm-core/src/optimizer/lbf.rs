use crate::config::RasterNfpConfig;
use crate::eval::lbf_evaluator::{LBFEvaluator, flat_exact_cde, polygons_collide_bidirectionally};
use crate::eval::sample_eval::{SampleEval, SampleEvaluator};
use crate::optimizer::place_groups::{PlaceGroupError, PlaceGroups};
use crate::sample::search::{SampleConfig, search_placement_prepared_with_control};
use crate::structure::evaluator::{
    ProfilePlacementConfig, ProfilePlacementEvaluator, build_profile_frontier,
    profile_alignment_candidates, refine_profile_candidate_with_control, tag_spolygon,
};
use crate::structure::motif::{MotifCatalog, MotifItemRef, MotifKind, PlacementMotif};
use crate::structure::profile::{
    ProfileFrontier, TranslationRefinementConfig, refine_translation_exact,
};
use crate::util::assertions;
use itertools::Itertools;
use jagua_rs::Instant;
use jagua_rs::collision_detection::CDEngine;
use jagua_rs::collision_detection::hazards::filter::NoFilter;
use jagua_rs::entities::{Layout};
use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::geo_traits::TransformableFrom;
use jagua_rs::geometry::primitives::SPolygon;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem};
use ordered_float::OrderedFloat;
use rand::RngExt;
use rand::rngs::Xoshiro256PlusPlus;
use serde::{Deserialize, Serialize};
use std::cmp::Reverse;
use std::collections::{HashMap, HashSet};

/// A structure-first placement step may revisit the remaining anchors after a
/// wider strip is installed, but it must never grow the strip without bound.
/// Four rounds allow a little over 2x width recovery while keeping a hard,
/// deterministic upper bound on sampling work for an impossible remainder.
const MAX_STRUCTURE_RECOVERY_EXPANSIONS: usize = 4;

/// Fixed-size candidate counters for one P1 construction start.
///
/// `attempted` counts complete placement-unit hypotheses entering exact-authorized
/// evaluation. `exact_feasible` counts hypotheses which produced a candidate after
/// all exact checks and slide refinement. The committed counters describe the
/// selected candidates only.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct StructureCandidateStats {
    pub attempted: usize,
    pub exact_feasible: usize,
    /// One-way CDE authorization succeeded, but the selected-candidate audit
    /// detected a reverse-direction edge/containment collision.
    pub bidirectional_rejected: usize,
    pub committed_units: usize,
    pub committed_members: usize,
}

/// Candidate-unit channels whose feasibility provenance remains observable at
/// the LBF boundary. A single candidate is a composite search over motif/profile,
/// Raster/exact-NFP, and random inputs; the search API intentionally does not
/// attribute its post-coordinate-descent result to one of those inputs.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct StructureSourceStats {
    pub single: StructureCandidateStats,
    pub motif: StructureCandidateStats,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StructureMotifKind {
    Single,
    Pair,
    RectangleBlock,
    RepeatedBand,
}

/// P1 attempt/feasibility/commit counters split by the originating P0 motif kind.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct StructureMotifKindStats {
    pub single: StructureCandidateStats,
    pub pair: StructureCandidateStats,
    pub rectangle_block: StructureCandidateStats,
    pub repeated_band: StructureCandidateStats,
}

/// Input-channel audit values for the composite single-item search.
///
/// These values deliberately stop at generation/deduplication. The existing
/// search routine mixes seeds and random samples before coordinate descent and
/// does not expose source provenance, so claiming per-source exact-feasible or
/// committed counts here would be misleading.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct StructureProposalStats {
    pub motif_seed_generated: usize,
    pub profile_generated: usize,
    pub profile_retained_after_dedup: usize,
    pub raster_generated: usize,
    pub exact_nfp_generated: usize,
    pub raster_exact_retained_after_dedup: usize,
    pub random_sample_budget: usize,
    pub legacy_fallback_attempted: usize,
    pub legacy_fallback_exact_feasible: usize,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StructureSelectionSource {
    #[default]
    Single,
    Motif,
}

/// Compact, allocation-bounded summary of one committed unit. The anchor identity
/// and member count preserve construction order without cloning member vectors.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct StructureSelectionStats {
    pub selection_index: usize,
    pub source: StructureSelectionSource,
    pub motif_kind: Option<StructureMotifKind>,
    pub anchor_item_id: usize,
    pub anchor_copy_index: usize,
    pub member_count: usize,
}

/// Outcome of one structure-first constructor start.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct StartStats {
    pub requested_members: usize,
    pub initially_placed_members: usize,
    pub committed_units: usize,
    pub committed_members: usize,
    pub strip_expansions: usize,
    /// Number of beam batches which produced no bidirectionally valid commit.
    pub recovery_rounds: usize,
    /// True when every remaining anchor was tried for the bounded number of
    /// width-recovery rounds. The returned problem is intentionally partial.
    pub recovery_exhausted: bool,
    pub completed: bool,
    pub stopped: bool,
    pub elapsed_secs: f64,
    pub width: f32,
    pub density: f32,
    pub selection_order: Vec<StructureSelectionStats>,
}

/// Auditable P1 statistics attached to the returned [`LBFBuilder`].
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct StructureHintStats {
    pub start: StartStats,
    pub sources: StructureSourceStats,
    pub motif_kinds: StructureMotifKindStats,
    pub proposals: StructureProposalStats,
}

impl StructureMotifKindStats {
    fn for_kind_mut(&mut self, kind: StructureMotifKind) -> &mut StructureCandidateStats {
        match kind {
            StructureMotifKind::Single => &mut self.single,
            StructureMotifKind::Pair => &mut self.pair,
            StructureMotifKind::RectangleBlock => &mut self.rectangle_block,
            StructureMotifKind::RepeatedBand => &mut self.repeated_band,
        }
    }

    fn add_assign(&mut self, other: Self) {
        for (kind, source) in [
            (StructureMotifKind::Single, other.single),
            (StructureMotifKind::Pair, other.pair),
            (StructureMotifKind::RectangleBlock, other.rectangle_block),
            (StructureMotifKind::RepeatedBand, other.repeated_band),
        ] {
            let destination = self.for_kind_mut(kind);
            destination.attempted += source.attempted;
            destination.exact_feasible += source.exact_feasible;
            destination.bidirectional_rejected += source.bidirectional_rejected;
            destination.committed_units += source.committed_units;
            destination.committed_members += source.committed_members;
        }
    }
}

#[cfg(feature = "capmods")]
use crate::sample::exact_nfp;
#[cfg(feature = "capmods")]
use crate::sample::raster_nfp::RasterNfp;
#[cfg(feature = "capmods")]
use crate::sample::search::search_placement_seeded_prepared_with_control;
use crate::sample::search::search_placement_seeded_stochastic_top_k_prepared_with_control;

/// LBF 배치 순서 모드.
///
/// # 기준 크기가 "면적" 이 아니다
///
/// 정렬 점수는 `볼록껍질 면적 x 직경` 이다([`LBFBuilder::construct_with_control`]).
/// 실제 도형 면적이 아니라 **오목한 부분이 메워진** 면적이고, 거기에 직경이 곱해져
/// 길쭉한 조각이 앞으로 온다. 차원도 면적이 아니라 `면적 x 길이` 다.
///
/// 이 enum 은 그 점수의 **정의는 건드리지 않고 순회 방향만** 바꾼다. 그래야 A/B 에서
/// "순서를 바꾼 효과" 와 "크기 정의를 바꾼 효과" 가 섞이지 않는다.
///
/// # 왜 중간부터인가
///
/// 큰 것부터 넣으면 큰 조각이 좋은 자리를 먼저 차지하고 작은 조각이 남은 틈을 메운다.
/// 작은 것부터 넣으면 그 반대다. 중간부터는 **양쪽을 번갈아** 내보내, 큰 조각의 골격과
/// 작은 조각의 채움이 배치 내내 섞이게 한다. 어느 쪽이 좋은지는 인스턴스마다 다르므로
/// 재 보는 수밖에 없다 — 그래서 설정으로 뺐다.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LbfOrderMode {
    /// 큰 것부터 (현행 동작). 점수 내림차순.
    #[default]
    LargestFirst,
    /// 작은 것부터. 위 순서를 그대로 뒤집는다.
    SmallestFirst,
    /// **중간부터**, 갈림길에서 **큰 쪽**을 먼저.
    ///
    /// 정렬된 목록 `[가장 큼 ... 가장 작음]` 의 가운데에서 시작해 바깥으로 번갈아
    /// 나간다: `m, m-1, m+1, m-2, m+2, ...` (`m-1` 이 더 큰 쪽이다).
    MiddleOutLargeFirst,
    /// **중간부터**, 갈림길에서 **작은 쪽**을 먼저.
    ///
    /// `m, m+1, m-1, m+2, m-2, ...`
    MiddleOutSmallFirst,
    /// **방향성 오목도**가 큰 것부터 (`LBF수정사항.md`).
    ///
    /// 정렬 키가 크기가 아니라 [`crate::optimizer::concavity`] 의 점수
    /// `10·S_L + S_B + 2·S_C + 0.5·S_D` 다 — 왼쪽·아래로 열린 오목부를 가진 조각을
    /// 먼저 놓아 뒤 조각이 그 안에 들어가게 한다. 점수는 조각 면적으로 정규화돼
    /// 있어 크기와 무관하다(설계 §15). 동점은 크기 내림차순.
    ///
    /// 위 네 모드와 달리 **크기 점수의 정의 자체를 바꾼다.** A/B 에서 그 차이를
    /// 분리하려면 [`Self::SizeTimesConcavity`] 와 함께 재라.
    ConcavityFirst,
    /// 크기 점수에 오목도를 곱한 것부터 — `크기 × (1 + 오목도)`.
    ///
    /// [`Self::ConcavityFirst`] 는 정규화된 점수라 작은 조각도 오목하면 맨 앞에
    /// 온다. 이 코드베이스는 "작은 조각이 무조건 이긴다" 로 한 번 졌으므로
    /// (⑥ best-fit 감사), 큰 것부터의 골격을 유지하면서 오목한 조각을 앞당기는
    /// 절충안을 함께 둔다.
    SizeTimesConcavity,
}

impl LbfOrderMode {
    /// C ABI 정수에서. 범위 밖은 기본값(현행 동작)이다 — 경계에서 프로세스를 죽이지 않는다.
    pub fn from_raw(v: i32) -> Self {
        match v {
            1 => Self::SmallestFirst,
            2 => Self::MiddleOutLargeFirst,
            3 => Self::MiddleOutSmallFirst,
            4 => Self::ConcavityFirst,
            5 => Self::SizeTimesConcavity,
            _ => Self::LargestFirst,
        }
    }

    /// 로그·리포트용 라벨.
    pub fn label(&self) -> &'static str {
        match self {
            Self::LargestFirst => "큰것부터",
            Self::SmallestFirst => "작은것부터",
            Self::MiddleOutLargeFirst => "중간부터(큰쪽먼저)",
            Self::MiddleOutSmallFirst => "중간부터(작은쪽먼저)",
            Self::ConcavityFirst => "오목도부터",
            Self::SizeTimesConcavity => "크기x오목도",
        }
    }

    /// 크기 내림차순으로 정렬된 목록을 이 모드의 순서로 재배열한다.
    ///
    /// 입력은 **반드시 점수 내림차순**이어야 한다(가장 큰 것이 앞). 그 전제가 깨지면
    /// "중간" 과 "큰 쪽" 의 의미가 사라진다.
    #[cfg_attr(not(feature = "cpguard-policy-off"), cpguard::obfuscate)]
    pub fn arrange<T>(&self, sorted_desc: Vec<T>) -> Vec<T> {
        let n = sorted_desc.len();
        if n <= 1 {
            return sorted_desc;
        }
        match self {
            // 오목도 모드는 정렬 **키**가 다를 뿐 순회는 내림차순 그대로다.
            Self::LargestFirst | Self::ConcavityFirst | Self::SizeTimesConcavity => sorted_desc,
            Self::SmallestFirst => sorted_desc.into_iter().rev().collect(),
            Self::MiddleOutLargeFirst | Self::MiddleOutSmallFirst => {
                let large_first = matches!(self, Self::MiddleOutLargeFirst);
                // 원소를 옮기지 않고 **인덱스 순서**만 만든 뒤 한 번에 재배치한다.
                // `remove` 로 꺼내면 O(n²) 이고, 조각이 수천 개인 마커에서 그게 보인다.
                let m = n / 2;
                let mut order = Vec::with_capacity(n);
                order.push(m);
                let (mut lo, mut hi) = (m, m); // 이미 쓴 구간 [lo, hi]
                while order.len() < n {
                    let can_low = lo > 0; // 더 큰 쪽
                    let can_high = hi + 1 < n; // 더 작은 쪽
                    let take_low = match (can_low, can_high) {
                        (true, false) => true,
                        (false, true) => false,
                        // 둘 다 가능하면 모드가 정한 쪽을 먼저.
                        (true, true) => large_first == (order.len() % 2 == 1),
                        (false, false) => break,
                    };
                    if take_low {
                        lo -= 1;
                        order.push(lo);
                    } else {
                        hi += 1;
                        order.push(hi);
                    }
                }
                let mut slots: Vec<Option<T>> = sorted_desc.into_iter().map(Some).collect();
                order.into_iter().filter_map(|i| slots[i].take()).collect()
            }
        }
    }
}

pub struct LBFBuilder {
    prepared: std::sync::Arc<crate::prepared_geometry::PreparedGeometry>,
    pub instance: SPInstance,
    pub prob: SPProblem,
    pub rng: Xoshiro256PlusPlus,
    pub sample_config: SampleConfig,
    pub lbf_x_multiplier: f32,
    pub lbf_y_multiplier: f32,
    /// 래스터 NFP 후보 생성 설정. capmods 빌드에서만 쓰인다.
    raster_cfg: RasterNfpConfig,
    /// 배치 도중 다지기 설정. `None`(기본)이면 기존 구성 경로와 한 비트도 다르지 않다.
    settle: Option<SettleDuringConstruction>,
    /// User locks for the auxiliary settle separator (item-kind ids).
    frozen_ids: Vec<usize>,
    /// 구간 완성 폭(mm). `None`(기본)이면 구간 제한 없이 기존대로 배치한다.
    band_width_mm: Option<f32>,
    /// 현재 구간의 오른쪽 끝. `find_placement` 가 이 값을 후보 상한으로 넘긴다.
    band_x_max: Option<f32>,
    /// 구간 경계에서 목록 뒤쪽까지 훑을 때, 끌어올려도 되는 조각의 최대 면적
    /// (가장 큰 조각 대비 비율). `0`(기본)이면 훑지 않는다.
    ///
    /// 제한 없이 훑으면 작은 조각이 앞 구간으로 끌려와 **면적 내림차순이 깨지고**
    /// 큰 조각이 늦게 배치돼 스트립이 밀린다(실측 73.903% -> 67.7%). 진짜 오목부
    /// 채우기용 조각만 허용해야 한다.
    band_fill_max_area_ratio: f32,
    /// 위 비율을 실제 면적으로 환산한 값. 구성 시작 시 한 번 계산한다.
    band_fill_area_cap: f32,
    /// 배치가 진행되며 갱신되는 점유 격자. `None`이면 기존 난수 샘플링만 쓴다.
    #[cfg(feature = "capmods")]
    raster: Option<RasterNfp>,
    /// Rigid placement units. Empty preserves the legacy item-by-item LBF.
    place_groups: PlaceGroups,
    /// Optional structure-first profile ranker. `None` is the byte-for-byte
    /// legacy LBF evaluation path.
    profile_cfg: Option<ProfilePlacementConfig>,
    /// Incremental P1 x(y) state. Rebuilt only when strip bounds change.
    profile_frontier: Option<ProfileFrontier>,
    /// Reversible P0 suggestions selected for this constructor run.
    motifs: Vec<PlacementMotif>,
    /// Fresh-construction occurrence poses used to translate relative motif
    /// suggestions.  This is deliberately solver-local and is discarded as
    /// soon as construction finishes.
    placed_copies: HashMap<MotifItemRef, DTransformation>,
    /// Present only for the actual structure-first branch. Legacy and rigid
    /// PlaceGroup construction intentionally leave this as `None`.
    structure_stats: Option<StructureHintStats>,
    /// Original-outline contract for Structure candidates and Legacy winners.
    /// Only allocated when the input permits piece overlap.
    original_guard: Option<crate::original_overlap::OriginalOverlapGuard>,
    /// 배치 순서 모드. 기본은 현행 동작(큰 것부터).
    order_mode: LbfOrderMode,
}

/// 접촉 자리가 원본 겹침 계약을 어길 때 물러나 볼 거리(mm).
///
/// 구조 힌트 경로는 조각을 충돌 도형 기준 «정확 접촉» 까지 민다(허용 오차 1e-4mm). 그 자리의
/// 원본 외곽은 인증 기준보다 격자 여유(0.001mm)만큼 더 겹쳐서, 놓는 족족 계약을 어겼다
/// (실측: babel 겉감 200조각, 허용 0.4mm 에서 후보 하나에 위반 68쌍 — 줄지어 맞닿은 조각은
/// 사후 보정으로도 풀리지 않아 후보가 통째로 기각됐다). 첫 값은 사후 보정의 최소 이동량과
/// 같고, 스트립 끝(좌표 ~14000mm)의 f32 눈금 0.001mm 보다 충분히 크다.
pub(crate) const CONTRACT_BACKOFF_MM: [f32; 4] = [0.005, 0.02, 0.1, 0.5];

#[derive(Clone, Debug)]
struct StructurePlacementCandidate {
    members: Vec<(MotifItemRef, SPPlacement)>,
    loss: f32,
    tag: StructureCandidateTag,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum StructureCandidateTag {
    Single,
    Motif(StructureMotifKind),
}

#[derive(Clone, Copy, Debug, Default)]
struct MotifCandidateBatchStats {
    motif_kinds: StructureMotifKindStats,
}

/// 구간 완성에서 한 구간당 시도해 볼 조각 수 상한.
///
/// 배치 순서는 이미 면적 등으로 정렬되어 있으므로 앞쪽 몇 개만 보면 된다. 전수
/// 탐색은 배치 탐색을 조각 수의 제곱만큼 돌려 구성 예산을 통째로 삼킨다.
const BAND_LOOKAHEAD: usize = 16;

/// Optional construction settle at the configured placement intervals.
///
/// LP currently certifies full demand, so incomplete intervals are skipped.
/// Complete intervals retain LP compaction with frozen and rigid constraints.
#[derive(Debug, Clone, Copy)]
pub struct SettleDuringConstruction {
    /// 몇 조각마다 다질지. `1` 이면 매 조각.
    pub every: usize,
    /// 한 번의 다지기에 줄 예산(ms).
    pub budget_ms: u64,
    /// LP 압축 설정.
    pub lp: crate::config::LpCompactionConfig,
    /// Hole fill 설정.
    pub hole_fill: crate::config::HoleFillConfig,
    /// 분리기 설정(좌표 하강 횟수·샘플 예산이 여기서 온다).
    pub separator: crate::optimizer::separator::SeparatorConfig,
}

impl LBFBuilder {
    /// `pending[from..to]` 에서 현재 구간에 들어가는 첫 조각을 배치하고 그 인덱스를
    /// 돌려준다. 수요가 이미 0 인 항목은 배치 없이 소비한다.
    fn try_place_from(
        &mut self,
        pending: &std::collections::VecDeque<MotifItemRef>,
        from: usize,
        to: usize,
        should_stop: &mut dyn FnMut() -> bool,
    ) -> Option<usize> {
        for i in from..to.min(pending.len()) {
            let item_ref = pending[i];
            if self.prob.item_demand_qtys()[item_ref.item_id] == 0 {
                return Some(i);
            }
            // 목록 뒤쪽(창 밖)은 오목부 채우기용 작은 조각만 끌어올린다.
            if i >= BAND_LOOKAHEAD
                && self.instance.item(item_ref.item_id).shape_cd().area() > self.band_fill_area_cap
            {
                continue;
            }
            let before = self.prob.layout().placed_items().len();
            self.place_item(item_ref, should_stop);
            if self.prob.layout().placed_items().len() > before {
                return Some(i);
            }
            if should_stop() {
                return None;
            }
        }
        None
    }

    /// Settle only a complete layout: LP requires exact full-demand certification.
    /// Incomplete attempts consume their historical seed draw, but do no polish
    /// work, so skipping guaranteed rollbacks preserves later construction RNG.
    fn settle_partial_layout(
        &mut self,
        settle: SettleDuringConstruction,
        should_stop: &mut dyn FnMut() -> bool,
    ) {
        if should_stop() {
            return;
        }
        let seed = self.rng.random_range(0..u64::MAX)
            ^ self.prob.layout().placed_items().len() as u64;
        if self.prob.item_demand_qtys().iter().any(|&qty| qty > 0) {
            return;
        }
        use crate::util::terminator::Terminator as _;
        use rand::SeedableRng as _;
        let mut sep_config = settle.separator;
        // 다지기는 탐색이 아니다. 워커 복제와 NFP 준비 비용을 지운다.
        sep_config.n_workers = 1;
        sep_config.nfp_sampler = false;
        let mut sep = crate::optimizer::separator::Separator::new(
            self.instance.clone(),
            self.prob.clone(),
            Xoshiro256PlusPlus::seed_from_u64(seed),
            sep_config,
        );
        if !self.place_groups.is_empty()
            && let Err(error) = sep.set_place_groups(self.place_groups.clone())
        {
            log::warn!("[LBF settle] invalid rigid layout: {error}");
            return;
        }
        if !self.frozen_ids.is_empty() {
            sep.freeze_items(&self.frozen_ids);
        }
        let reference = self.prob.save();
        let mut term = crate::util::terminator::BasicTerminator::new();
        term.new_timeout(std::time::Duration::from_millis(settle.budget_ms.max(1)));
        let term = crate::util::terminator::InitTerminator::new(term);

        crate::optimizer::polish::holes::hole_fill_pass(&mut sep, &settle.hole_fill, &term);
        if should_stop() {
            return;
        }
        crate::optimizer::polish::lp::lp_compaction_pass(&mut sep, &settle.lp, &term);
        if term.stop_requested() {
            return;
        }

        // Collision freedom alone cannot certify locks or rigid relative poses.
        if sep.ct.get_total_loss() == 0.0
            && sep.solution_is_certified(&sep.prob.save(), &reference)
            && !term.stop_requested()
        {
            self.prob = sep.prob;
        }
    }
}

impl LBFBuilder {
    pub fn new(
        instance: SPInstance,
        rng: Xoshiro256PlusPlus,
        sample_config: SampleConfig,
        lbf_x_multiplier: f32,
        lbf_y_multiplier: f32,
    ) -> Self {
        let prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
        Self::from_problem(
            instance,
            prob,
            rng,
            sample_config,
            lbf_x_multiplier,
            lbf_y_multiplier,
        )
    }

    /// 이미 일부 아이템이 놓인 문제에서 이어서 배치할 때 쓴다 (warm start / 배치 힌트 경로).
    pub fn from_problem(
        instance: SPInstance,
        prob: SPProblem,
        rng: Xoshiro256PlusPlus,
        sample_config: SampleConfig,
        lbf_x_multiplier: f32,
        lbf_y_multiplier: f32,
    ) -> Self {
    #[cfg(feature = "init-profile")]
    let _profile = crate::init_profile::Scope::new("detail.lbf_builder_prepare");

        Self {
            prepared: std::sync::Arc::new(crate::prepared_geometry::PreparedGeometry::new(
                &instance,
            )),
            instance,
            prob,
            rng,
            sample_config,
            lbf_x_multiplier,
            lbf_y_multiplier,
            raster_cfg: crate::config::DEFAULT_RASTER_NFP_CONFIG,
            settle: None,
            frozen_ids: vec![],
            band_width_mm: None,
            band_x_max: None,
            band_fill_max_area_ratio: 0.0,
            band_fill_area_cap: 0.0,
            #[cfg(feature = "capmods")]
            raster: None,
            place_groups: PlaceGroups::default(),
            profile_cfg: None,
            profile_frontier: None,
            motifs: vec![],
            placed_copies: HashMap::new(),
            structure_stats: None,
            original_guard: None,
            order_mode: LbfOrderMode::default(),
        }
    }

    /// 배치 순서 모드를 지정한다 (생략 시 [`LbfOrderMode::LargestFirst`] = 현행 동작).
    ///
    /// **빌더 메서드로 둔 이유**: `new`/`from_problem` 에 인자를 더하면 저장소의
    /// 호출부 11곳이 전부 깨지고, 그중 하나를 빠뜨리면 그 경로만 조용히 다른 순서로
    /// 돈다. 기본값이 현행 동작이므로 부르지 않은 경로는 한 비트도 바뀌지 않는다.
    pub fn with_order_mode(mut self, mode: LbfOrderMode) -> Self {
        self.order_mode = mode;
        self
    }

    /// 래스터 NFP 설정을 지정한다 (생략 시 [`crate::config::DEFAULT_RASTER_NFP_CONFIG`]).
    /// 구간 완성을 켠다. 현재 구간이 더 받아들이지 못할 때까지 프론티어를 오른쪽으로
    /// 밀지 않는다. `None`(기본)이면 기존 경로와 한 비트도 다르지 않다.
    ///
    /// # 왜
    ///
    /// 후보 생성은 조각마다 독립적이라 "이 구간을 다 채우고 나아간다" 를 손실 항으로
    /// 표현할 수 없다. 참조 네스터의 77% 초기 배치는 세로 구간을 위아래로 완전히
    /// 채운 뒤에야 오른쪽으로 나아간다. 그래서 제어 흐름으로 강제한다.
    /// 구간 경계에서 목록 뒤쪽의 **작은** 조각까지 훑어 구간을 채운다.
    /// 비율은 가장 큰 조각 면적 대비다. `0`(기본)이면 훑지 않는다.
    pub fn with_band_fill_ratio(mut self, ratio: f32) -> Self {
        self.band_fill_max_area_ratio = if ratio.is_finite() && ratio > 0.0 {
            ratio
        } else {
            0.0
        };
        self
    }

    pub fn with_band_width(mut self, mm: Option<f32>) -> Self {
        self.band_width_mm = mm.filter(|v| v.is_finite() && *v > 0.0);
        self
    }

    /// 배치 도중 다지기를 켠다. [`SettleDuringConstruction`] 참고.
    pub fn with_settle(mut self, settle: Option<SettleDuringConstruction>) -> Self {
        self.settle = settle;
        self
    }

    /// Preserve user-locked item transforms during optional construction settle.
    /// Locking one rigid member also locks the whole group in the Separator.
    pub fn with_frozen_items(mut self, ids: &[usize]) -> Self {
        self.frozen_ids = ids.to_vec();
        self
    }

    pub fn with_raster_config(mut self, cfg: RasterNfpConfig) -> Self {
        self.raster_cfg = cfg;
        self
    }

    /// Returns the P1 audit record after an actual structure-first construction.
    /// Legacy and rigid PlaceGroup paths return `None` and retain their historical
    /// behavior and cost.
    pub fn structure_hint_stats(&self) -> Option<&StructureHintStats> {
        self.structure_stats.as_ref()
    }

    /// Configures fresh/warm LBF to place each rigid group as one unit.
    pub fn with_place_groups(mut self, groups: PlaceGroups) -> Result<Self, PlaceGroupError> {
        groups.validate_layout(&self.prob.layout())?;
        self.place_groups = groups;
        Ok(self)
    }

    /// Enables P0 soft motifs and P1 profile ranking for a fresh constructor.
    ///
    /// Motifs affect only item order and seed poses. They are never installed
    /// as rigid [`PlaceGroups`], and every seed is checked by exact CDE. Warm
    /// starts and user PlaceGroups remain supported; soft motifs are simply
    /// ignored when rigid groups are active.
    pub fn with_structure_hints(
        self,
        profile_cfg: ProfilePlacementConfig,
        catalog: MotifCatalog,
        max_motifs: usize,
    ) -> Self {
        let motifs = catalog
            .disjoint_ranked(max_motifs)
            .into_iter()
            .cloned()
            .collect();
        self.with_preselected_structure_hints(profile_cfg, motifs)
    }

    /// Enables the structure constructor with an already audited disjoint P0
    /// selection. Pipeline multi-starts use this entrypoint so matching and
    /// set-packing run exactly once and do not consume the shared P1 deadline
    /// again for every start.
    pub fn with_preselected_structure_hints(
        mut self,
        profile_cfg: ProfilePlacementConfig,
        motifs: Vec<PlacementMotif>,
    ) -> Self {
        self.profile_cfg = Some(profile_cfg);
        self.profile_frontier = build_profile_frontier(&self.prob.layout(), profile_cfg).ok();
        if self.place_groups.is_empty() {
            self.motifs = motifs;
        }
        self.original_guard =
            crate::original_overlap::OriginalOverlapGuard::is_active(&self.instance)
                .then(|| crate::original_overlap::OriginalOverlapGuard::new(&self.instance));
        self
    }

    /// 접촉 자리 `refined` 와, 계약을 어길 때 물러나 볼 자리들. 밀기는 왼쪽 → 아래 순서라
    /// 되돌아가는 방향은 오른쪽·위·둘 다이다. 술어가 없으면 `refined` 하나뿐이다(기존 동작).
    fn contract_backoff_poses(&self, refined: DTransformation) -> Vec<DTransformation> {
        let mut poses = vec![refined];
        if self.original_guard.is_some() {
            let (x, y) = refined.translation();
            for step in CONTRACT_BACKOFF_MM {
                for (dx, dy) in [(1.0, 0.0), (0.0, 1.0), (1.0, 1.0)] {
                    poses.push(DTransformation::new(
                        refined.rotation(),
                        (x + dx * step, y + dy * step),
                    ));
                }
            }
        }
        poses
    }

    /// 후보 조각들이 이미 놓인 조각, 그리고 서로와 원본 겹침 계약을 지키는지.
    /// 술어가 없으면 언제나 참이다.
    fn original_contract_allows(&self, members: &[SPPlacement]) -> bool {
        let Some(guard) = self.original_guard.as_ref() else {
            return true;
        };
        let mut outlines = Vec::with_capacity(members.len());
        for member in members {
            let Ok(candidate) = guard.outline(member.item_idx, member.d_transf) else {
                return false;
            };
            if !matches!(
                guard.outline_allowed_in_layout(&self.prob.layout(), &candidate),
                Ok(true)
            ) || outlines
                .iter()
                .any(|other| !matches!(guard.outlines_allowed(&candidate, other), Ok(true)))
            {
                return false;
            }
            outlines.push(candidate);
        }
        true
    }

    /// 호스트의 Init 취소 요청([`crate::config::runtime::request_init_cancel`])을
    /// 조각 경계마다 확인한다. 취소되면 부분 배치를 돌려주며, 호출자는 수요를 검증해야 한다.
    pub fn construct(self) -> Self {
        let mut stop_on_init_cancel = crate::config::runtime::init_cancel_requested;
        self.construct_with_control(&mut stop_on_init_cancel)
    }

    /// Cooperative counterpart used by bounded P1 multi-start construction.
    /// A stop may return a partial problem; callers must validate item demand
    /// before treating it as a solution candidate.
    // Protect the construction entry once, outside per-candidate hot paths.
    #[cfg_attr(not(feature = "cpguard-policy-off"), cpguard::obfuscate)]
    pub fn construct_with_control(mut self, should_stop: &mut dyn FnMut() -> bool) -> Self {
    #[cfg(feature = "init-profile")]
    let _profile = crate::init_profile::Scope::new("detail.lbf_construct");

        let start = Instant::now();
        let n_items = self.instance.items.len();
        let order_mode = self.order_mode;
        let sorted_item_ids = (0..n_items)
            // Stable sorting preserves representative ties in ascending item-id order.
            .filter(|id| self.place_groups.is_representative(*id))
            .sorted_by_cached_key(|id| {
                let members = self.place_groups.member_item_ids(*id);
                // 현행 크기 점수: 볼록껍질 면적 x 직경 (그룹은 합).
                let size = members
                    .iter()
                    .map(|&member_id| {
                        let shape = self.instance.item(member_id).shape_cd().as_ref();
                        shape.surrogate().convex_hull_area * shape.diameter()
                    })
                    .sum::<f32>();
                // 방향성 오목도 (그룹은 면적 가중 평균). **원본 도형**으로 계산한다 —
                // 충돌용 도형은 좁은 오목부가 메워져 있다(`concavity` 모듈 문서).
                // `sorted_by_cached_key` 가 조각 종류마다 한 번만 부르므로 이것이 캐시다.
                let concavity = || -> f32 {
                    let (mut acc, mut w) = (0.0f64, 0.0f64);
                    for &member_id in &members {
                        let orig = &self.instance.item(member_id).shape_orig().shape;
                        let sc = crate::optimizer::concavity::analyze_spolygon(orig);
                        let a = sc.polygon_area.max(1e-9);
                        acc += sc.final_score * a;
                        w += a;
                    }
                    if w > 0.0 { (acc / w) as f32 } else { 0.0 }
                };
                let (primary, secondary) = match order_mode {
                    LbfOrderMode::ConcavityFirst => (concavity(), size),
                    LbfOrderMode::SizeTimesConcavity => (size * (1.0 + concavity()), 0.0),
                    _ => (size, 0.0),
                };
                (
                    Reverse(OrderedFloat(primary)),
                    Reverse(OrderedFloat(secondary)),
                )
            })
            .collect_vec();
        // 크기 정의는 그대로 두고 **순회 방향만** 바꾼다(§`LbfOrderMode`).
        let sorted_item_ids = self.order_mode.arrange(sorted_item_ids);

        let mut sorted_item_indices = vec![];
        for id in sorted_item_ids {
            let members = self.place_groups.member_item_ids(id);
            if members.len() == 1 {
                let first_missing_copy =
                    self.instance.item_qty(id) - self.prob.item_demand_qtys()[id];
                sorted_item_indices.extend((first_missing_copy..self.instance.item_qty(id)).map(
                    |copy_index| MotifItemRef {
                        item_id: id,
                        copy_index,
                    },
                ));
                continue;
            }
            let n_missing = members
                .iter()
                .filter(|&&member_id| self.prob.item_demand_qtys()[member_id] > 0)
                .count();
            assert!(
                n_missing == 0 || n_missing == members.len(),
                "place group {} is only partially placed before LBF",
                self.place_groups.group_id_of(id).unwrap()
            );
            if n_missing == members.len() {
                sorted_item_indices.push(MotifItemRef {
                    item_id: id,
                    copy_index: 0,
                });
            }
        }

        // P1 has its own bounded all-remaining constructor. Keeping this
        // branch before every legacy ordering/sampling statement is
        // intentional: when profile_cfg is None (or a rigid PlaceGroup is
        // active), the historical path below executes unchanged.
        if self.profile_cfg.is_some() && self.place_groups.is_empty() {
            self.structure_stats = Some(StructureHintStats {
                start: StartStats {
                    requested_members: sorted_item_indices.len(),
                    initially_placed_members: self.prob.layout().placed_items().len(),
                    selection_order: Vec::with_capacity(sorted_item_indices.len()),
                    ..StartStats::default()
                },
                ..StructureHintStats::default()
            });
            #[cfg(feature = "capmods")]
            self.init_raster();
            let mut stopped = false;
            {
                // Observe existing polls without adding a poll or latching the
                // callback. This preserves one-shot callback behavior exactly.
                let mut observed_stop = || {
                    let requested = should_stop();
                    stopped |= requested;
                    requested
                };
                self.construct_structure_first(sorted_item_indices, &mut observed_stop);
            }
            if !self.prob.layout().placed_items().is_empty() {
                crate::util::frozen::fit_strip(&mut self.prob);
            }
            let completed = self.prob.item_demand_qtys().iter().all(|qty| *qty == 0);
            let density = self.prob.density();
            if let Some(stats) = self.structure_stats.as_mut() {
                stats.start.completed = completed;
                stats.start.stopped = stopped;
                stats.start.elapsed_secs = start.elapsed().as_secs_f64();
                stats.start.width = self.prob.strip_width();
                stats.start.density = if density.is_finite() { density } else { 0.0 };
            }
            return self;
        }

        // Keep every demanded copy exactly once, but bring members of the
        // highest-ranked disjoint motifs next to one another. A failed motif
        // seed still falls through to ordinary profile/random sampling.
        if self.place_groups.is_empty() && !self.motifs.is_empty() {
            let mut prioritized = Vec::with_capacity(sorted_item_indices.len());
            for motif in &self.motifs {
                for member in &motif.members {
                    if let Some(index) = sorted_item_indices
                        .iter()
                        .position(|item| *item == member.item)
                    {
                        prioritized.push(sorted_item_indices.remove(index));
                    }
                }
            }
            prioritized.append(&mut sorted_item_indices);
            sorted_item_indices = prioritized;
        }

        // Certify original external pairs before committing; explicit rigid
        // placement units retain their user-defined internal intersections.
        self.original_guard =
            crate::original_overlap::OriginalOverlapGuard::is_active(&self.instance).then(|| {
                let mut guard = crate::original_overlap::OriginalOverlapGuard::new(&self.instance);
                guard.set_place_groups(&self.place_groups);
                guard
            });

        #[cfg(feature = "capmods")]
        self.init_raster();

        let mut since_settle = 0usize;
        let mut pending: std::collections::VecDeque<MotifItemRef> =
            sorted_item_indices.into_iter().collect();
        self.band_x_max = self.band_width_mm;
        self.band_fill_area_cap = if self.band_fill_max_area_ratio > 0.0 {
            let largest = self
                .instance
                .items
                .iter()
                .map(|(item, _)| item.shape_cd().area())
                .fold(0.0f32, f32::max);
            largest * self.band_fill_max_area_ratio
        } else {
            0.0
        };
        let container_x_max = self.prob.layout().container().outer_cd().bbox().x_max;
        while !pending.is_empty() {
            #[cfg(feature = "init-profile")]
            let _profile_block = crate::init_profile::Scope::new(match self.prob.layout().placed_items().len() / 1000 { 0 => "detail.lbf_items_0000_0999", 1 => "detail.lbf_items_1000_1999", 2 => "detail.lbf_items_2000_2999", _ => "detail.lbf_items_3000_plus" });
            if should_stop() {
                break;
            }
            // 구간 안에 들어가는 조각을 찾는다.
            //
            // 1단계는 앞쪽 몇 개만 본다 — 순서는 이미 면적 등으로 정렬되어 있고,
            // 매번 전수 탐색하면 배치 탐색을 조각 수의 제곱만큼 돌린다.
            //
            // 1단계가 실패하면 **전진하기 전에** 남은 전체를 훑는다. 이것이
            // 오목부 채우기다 — 큰 조각 스무 개를 놓고 나면 작은 조각은 목록 한참
            // 뒤에 있어서, 창만 보면 구간 안에서 시도조차 되지 않는다. 그 결과
            // 큰 조각이 만든 빈 속이 그대로 남은 채 프론티어가 전진한다.
            // 전수 훑기는 구간 경계에서만 돌므로 비용이 배치 수가 아니라 구간 수다.
            let mut chosen = self.try_place_from(&pending, 0, BAND_LOOKAHEAD, should_stop);
            if chosen.is_none()
                && self.band_fill_area_cap > 0.0
                && pending.len() > BAND_LOOKAHEAD
                && !should_stop()
            {
                chosen = self.try_place_from(&pending, BAND_LOOKAHEAD, pending.len(), should_stop);
            }
            match chosen {
                Some(i) => {
                    pending.remove(i);
                    since_settle += 1;
                    // Also settle after the last placement so the final layout
                    // is compacted even when the interval was not reached.
                    if let Some(settle) = self.settle
                        && (since_settle >= settle.every.max(1) || pending.is_empty())
                        && self.prob.layout().placed_items().len() >= 2
                    {
                        since_settle = 0;
                        self.settle_partial_layout(settle, should_stop);
                    }
                }
                None => {
                    // 이 구간은 더 받아들이지 못한다. 오른쪽으로 한 칸 나아간다.
                    // 상한이 컨테이너를 넘으면 제한을 아예 풀어 종료를 보장한다.
                    match (self.band_width_mm, self.band_x_max) {
                        (Some(width), Some(x)) if x < container_x_max => {
                            self.band_x_max = Some((x + width).min(container_x_max));
                        }
                        (Some(_), Some(_)) => self.band_x_max = None,
                        // 제한이 없는데도 아무것도 안 들어간다 = 남은 조각은 못 넣는다.
                        _ => break,
                    }
                }
            }
        }
        self.band_x_max = None;

        // A cooperative stop may be observed before the first item is
        // placed. Returning that empty partial problem is part of the
        // cancellable API contract; `fit_strip` requires a non-empty layout.
        if !self.prob.layout().placed_items().is_empty() {
            crate::util::frozen::fit_strip(&mut self.prob);
        }
        self
    }

    fn construct_structure_first(
        &mut self,
        mut remaining: Vec<MotifItemRef>,
        should_stop: &mut dyn FnMut() -> bool,
    ) {
        let config = self
            .profile_cfg
            .expect(crate::dmsg!("structure branch requires profile config"));
        // A failed beam anchor is skipped until either another unit commits or
        // every remaining anchor has been tried. This is essential for beam=1:
        // regenerating the same one-way-clear but reverse-colliding best sample
        // cannot be repaired by repeatedly increasing only the strip width.
        let mut stalled = HashSet::with_capacity(remaining.len());
        let mut recovery_expansions = 0usize;
        while !remaining.is_empty() && !should_stop() {
            let beam = self.structure_item_beam(&remaining, config.item_beam_width, &stalled);
            if beam.is_empty() {
                // Defensive: all remaining anchors should normally be handled
                // by `recover_structure_stall` immediately after their batch.
                if !self.recover_structure_stall(
                    &[],
                    remaining.len(),
                    &mut stalled,
                    &mut recovery_expansions,
                ) {
                    return;
                }
                continue;
            }
            let mut candidates = Vec::with_capacity(beam.len() * 2);

            for &item_ref in &beam {
                if should_stop() {
                    return;
                }
                if self.prob.item_demand_qtys()[item_ref.item_id] == 0 {
                    continue;
                }
                if let Some(stats) = self.structure_stats.as_mut() {
                    stats.sources.single.attempted += 1;
                }
                if let Some(single) = self.find_structure_single(item_ref, should_stop) {
                    if let Some(stats) = self.structure_stats.as_mut() {
                        stats.sources.single.exact_feasible += 1;
                    }
                    candidates.push(single);
                }
                let (motif_candidates, batch_stats) = self.structure_motif_candidates(
                    item_ref,
                    &remaining,
                    config.candidate_pool_cap,
                    should_stop,
                );
                if let Some(stats) = self.structure_stats.as_mut() {
                    stats.motif_kinds.add_assign(batch_stats.motif_kinds);
                    stats.sources.motif.attempted += motif_kind_attempted(batch_stats.motif_kinds);
                    stats.sources.motif.exact_feasible +=
                        motif_kind_exact_feasible(batch_stats.motif_kinds);
                }
                candidates.extend(motif_candidates);
            }

            if should_stop() {
                return;
            }
            candidates.sort_by(|a, b| {
                a.loss
                    .total_cmp(&b.loss)
                    .then_with(|| b.members.len().cmp(&a.members.len()))
                    .then_with(|| a.members[0].0.item_id.cmp(&b.members[0].0.item_id))
            });
            if candidates.is_empty() {
                if !self.recover_structure_stall(
                    &beam,
                    remaining.len(),
                    &mut stalled,
                    &mut recovery_expansions,
                ) {
                    return;
                }
                continue;
            }

            let eligible_cap = config.top_k.max(1).min(candidates.len());
            let mut eligible_indices = Vec::with_capacity(eligible_cap);
            let mut rejected_tags = vec![];
            for (index, candidate) in candidates.iter().enumerate() {
                if self.structure_candidate_is_bidirectionally_clear(candidate) {
                    eligible_indices.push(index);
                    if eligible_indices.len() == eligible_cap {
                        break;
                    }
                } else {
                    rejected_tags.push(candidate.tag);
                }
            }
            for tag in rejected_tags {
                self.record_structure_bidirectional_rejection(tag);
            }
            if eligible_indices.is_empty() {
                if !self.recover_structure_stall(
                    &beam,
                    remaining.len(),
                    &mut stalled,
                    &mut recovery_expansions,
                ) {
                    return;
                }
                continue;
            }
            let selected_index = if eligible_indices.len() == 1 {
                eligible_indices[0]
            } else {
                eligible_indices[self.rng.random_range(0..eligible_indices.len())]
            };
            let selected = candidates.remove(selected_index);
            let placed_refs: Vec<_> = selected.members.iter().map(|(item, _)| *item).collect();
            let tag = selected.tag;
            let anchor = selected.members[0].0;
            let member_count = selected.members.len();
            self.commit_structure_candidate(selected);
            self.record_structure_commit(tag, anchor, member_count);
            remaining.retain(|item| !placed_refs.contains(item));
            stalled.clear();
            recovery_expansions = 0;
        }
    }

    /// Selects a bounded, copy-aware cross-section of all remaining choices.
    /// Motif anchors are considered first, then one representative per item
    /// family, then additional copies. This avoids spending the whole beam on
    /// identical copies while retaining copy-specific motif hypotheses.
    fn structure_item_beam(
        &self,
        remaining: &[MotifItemRef],
        max_width: usize,
        stalled: &HashSet<MotifItemRef>,
    ) -> Vec<MotifItemRef> {
        let n_available = remaining
            .iter()
            .filter(|item| !stalled.contains(item))
            .count();
        let cap = max_width.max(1).min(n_available);
        let mut beam = Vec::with_capacity(cap);
        for motif in &self.motifs {
            if let Some(anchor) = motif.members.first().map(|member| member.item)
                && remaining.contains(&anchor)
                && !stalled.contains(&anchor)
                && !beam.contains(&anchor)
            {
                beam.push(anchor);
                if beam.len() == cap {
                    return beam;
                }
            }
        }
        for item in remaining {
            if stalled.contains(item) {
                continue;
            }
            if beam.iter().any(|selected| selected.item_id == item.item_id) {
                continue;
            }
            beam.push(*item);
            if beam.len() == cap {
                return beam;
            }
        }
        for item in remaining {
            if !stalled.contains(item) && !beam.contains(item) {
                beam.push(*item);
                if beam.len() == cap {
                    break;
                }
            }
        }
        beam
    }

    fn find_structure_single(
        &mut self,
        item_ref: MotifItemRef,
        should_stop: &mut dyn FnMut() -> bool,
    ) -> Option<StructurePlacementCandidate> {
        let config = self.profile_cfg?;
        let candidate_cap = config.candidate_pool_cap.max(1);
        let item_id = item_ref.item_id;
        let item = self.instance.item(item_id);
        let mut seeds = self.motif_seeds(item_ref);
        if let Some(stats) = self.structure_stats.as_mut() {
            stats.proposals.motif_seed_generated += seeds.len();
        }
        if let Some(frontier) = &self.profile_frontier {
            let proposals =
                profile_alignment_candidates(frontier, item, (candidate_cap / 2).max(1));
            let generated = proposals.len();
            let before = seeds.len();
            append_unique_transforms(&mut seeds, proposals);
            if let Some(stats) = self.structure_stats.as_mut() {
                stats.proposals.profile_generated += generated;
                stats.proposals.profile_retained_after_dedup += seeds.len() - before;
            }
        }

        #[cfg(feature = "capmods")]
        {
            if should_stop() {
                return None;
            }
            let raster_candidates = self
                .raster
                .as_mut()
                .map(|raster| {
                    raster.candidates(
                        item,
                        self.lbf_x_multiplier,
                        self.lbf_y_multiplier,
                        &self.raster_cfg,
                    )
                })
                .unwrap_or_default();
            let raster_generated = raster_candidates.len();
            let raster_refine_cap = (candidate_cap / 8).clamp(1, 16);
            let raster_candidates = raster_candidates
                .into_iter()
                .take(raster_refine_cap)
                .collect_vec();
            // In structure mode Raster and exact-NFP are candidate sources in
            // the same pool at every construction step, not a late fallback.
            let refined = exact_nfp::refine_candidates(
                &self.prob.layout(),
                item,
                &raster_candidates,
                self.lbf_x_multiplier,
                self.lbf_y_multiplier,
                (candidate_cap / 2).max(1),
            );
            let exact_nfp_generated = refined.len();
            if should_stop() {
                return None;
            }
            let before = seeds.len();
            append_unique_transforms(&mut seeds, refined);
            if let Some(stats) = self.structure_stats.as_mut() {
                stats.proposals.raster_generated += raster_generated;
                stats.proposals.exact_nfp_generated += exact_nfp_generated;
                stats.proposals.raster_exact_retained_after_dedup += seeds.len() - before;
            }
        }
        let mut sample_config = self.sample_config;
        // P1 construction honours the item's declared allowed rotations. A
        // caller's relaxed separator tilt setting must not leak into a fresh
        // structure-first incumbent.
        sample_config.tilt_range = 0.0;
        // Keep an explicit part of the bounded pool for stochastic insurance;
        // deterministic profile/Raster seeds must not consume the whole cap.
        let random_reserve = (candidate_cap / 4)
            .max(1)
            .min(sample_config.n_container_samples);
        seeds.truncate(candidate_cap.saturating_sub(random_reserve));
        let random_budget = candidate_cap.saturating_sub(seeds.len());
        sample_config.n_focussed_samples = 0;
        sample_config.n_container_samples = sample_config.n_container_samples.min(random_budget);
        sample_config.n_coord_descents = sample_config.n_coord_descents.min(candidate_cap);
        if let Some(stats) = self.structure_stats.as_mut() {
            stats.proposals.random_sample_budget += sample_config.n_container_samples;
        }
        let evaluator = match self.profile_frontier.as_ref() {
            Some(frontier) => {
                ProfilePlacementEvaluator::with_frontier(&self.prob.layout(), item, frontier, config)
            }
            None => ProfilePlacementEvaluator::new(&self.prob.layout(), item, config),
        }
        .ok()?;
        let (best, _) = search_placement_seeded_stochastic_top_k_prepared_with_control(
            &self.prob.layout(),
            item,
            None,
            &seeds,
            evaluator,
            sample_config,
            1,
            &mut self.rng,
            should_stop,
            &self.prepared,
        );
        let proposal = match best {
            Some((proposal, SampleEval::Clear { .. })) => proposal,
            _ if !should_stop() => {
                // A bounded profile pool can miss every feasible basin even
                // though the current strip is wide enough. Treating that
                // sampling miss as a width failure causes repeated 1.2x
                // expansion and can hit the runaway guard. Use the historical
                // exact-authorized LBF sampler as an insurance source before
                // changing geometry. This remains structure-only and does not
                // alter the legacy constructor path.
                let mut fallback_config = self.sample_config;
                fallback_config.tilt_range = 0.0;
                let fallback = LBFEvaluator::new(
                    &self.prob.layout(),
                    item,
                    self.lbf_x_multiplier,
                    self.lbf_y_multiplier,
                );
                if let Some(stats) = self.structure_stats.as_mut() {
                    stats.proposals.legacy_fallback_attempted += 1;
                }
                let (fallback, _) = search_placement_prepared_with_control(
                    &self.prob.layout(),
                    item,
                    None,
                    fallback,
                    fallback_config,
                    &mut self.rng,
                    should_stop,
                    &self.prepared,
                );
                match fallback {
                    Some((proposal, SampleEval::Clear { .. })) => {
                        if let Some(stats) = self.structure_stats.as_mut() {
                            stats.proposals.legacy_fallback_exact_feasible += 1;
                        }
                        proposal
                    }
                    _ => return None,
                }
            }
            _ => return None,
        };
        let refined = refine_profile_candidate_with_control(
            &self.prob.layout(),
            item,
            proposal,
            config.row_height,
            config.slide_refine_steps,
            should_stop,
        )?;
        if !rotation_is_allowed(item, refined.rotation()) {
            return None;
        }
        if should_stop() {
            return None;
        }
        let mut evaluator = match self.profile_frontier.as_ref() {
            Some(frontier) => {
                ProfilePlacementEvaluator::with_frontier(&self.prob.layout(), item, frontier, config)
            }
            None => ProfilePlacementEvaluator::new(&self.prob.layout(), item, config),
        }
        .ok()?;
        // 접촉 자리가 원본 겹침 계약을 어기면 조금 물러난 자리를 쓴다. 술어가 없으면 후보는
        // `refined` 하나라 기존과 같다.
        let (refined, loss) = self.contract_backoff_poses(refined).into_iter().find_map(|pose| {
            let placement = SPPlacement {
                item_idx: item_id,
                d_transf: pose,
            };
            if !self.original_contract_allows(&[placement]) {
                return None;
            }
            match evaluator.evaluate_sample(pose, None) {
                SampleEval::Clear { loss } => Some((pose, loss)),
                _ => None,
            }
        })?;
        Some(StructurePlacementCandidate {
            members: vec![(
                item_ref,
                SPPlacement {
                    item_idx: item_id,
                    d_transf: refined,
                },
            )],
            loss,
            tag: StructureCandidateTag::Single,
        })
    }

    fn structure_motif_candidates(
        &self,
        anchor_ref: MotifItemRef,
        remaining: &[MotifItemRef],
        candidate_cap: usize,
        should_stop: &mut dyn FnMut() -> bool,
    ) -> (Vec<StructurePlacementCandidate>, MotifCandidateBatchStats) {
        let Some(config) = self.profile_cfg else {
            return (vec![], MotifCandidateBatchStats::default());
        };
        let mut out = vec![];
        let mut batch_stats = MotifCandidateBatchStats::default();
        for motif in &self.motifs {
            if should_stop() || out.len() >= candidate_cap.max(1) {
                break;
            }
            if motif.members.first().map(|member| member.item) != Some(anchor_ref)
                || !motif
                    .members
                    .iter()
                    .all(|member| remaining.contains(&member.item))
            {
                continue;
            }
            let Some(anchor_member) = motif.members.first() else {
                continue;
            };
            let anchor_item = self.instance.item(anchor_ref.item_id);
            let mut anchor_proposals = vec![anchor_member.relative_pose];
            if let Some(frontier) = &self.profile_frontier {
                append_unique_transforms(
                    &mut anchor_proposals,
                    profile_alignment_candidates(frontier, anchor_item, 8)
                        .into_iter()
                        .filter(|dt| {
                            angular_distance(dt.rotation(), anchor_member.relative_pose.rotation())
                                < 1.0e-4
                        }),
                );
            }
            let motif_kind = structure_motif_kind(&motif.kind);
            for anchor_dt in anchor_proposals.into_iter().take(8) {
                if should_stop() {
                    break;
                }
                batch_stats.motif_kinds.for_kind_mut(motif_kind).attempted += 1;
                if let Some(candidate) =
                    self.evaluate_motif_unit(motif, anchor_dt, config, should_stop)
                {
                    batch_stats
                        .motif_kinds
                        .for_kind_mut(motif_kind)
                        .exact_feasible += 1;
                    out.push(candidate);
                    if out.len() >= candidate_cap.max(1) {
                        break;
                    }
                }
            }
        }
        (out, batch_stats)
    }

    fn evaluate_motif_unit(
        &self,
        motif: &PlacementMotif,
        anchor_dt: DTransformation,
        config: ProfilePlacementConfig,
        should_stop: &mut dyn FnMut() -> bool,
    ) -> Option<StructurePlacementCandidate> {
        if should_stop() {
            return None;
        }
        let bounds = self.prob.layout().container().outer_cd().bbox();
        let (x, y) = anchor_dt.translation();
        let steps = config.slide_refine_steps.clamp(1, 128);
        let refinement = TranslationRefinementConfig {
            coarse_step: config.row_height.max(1.0e-4),
            max_left: (x - bounds.x_min).max(0.0),
            max_down: (y - bounds.y_min).max(0.0),
            tolerance: 1.0e-4,
            max_coarse_steps_per_axis: steps,
            max_bisection_steps: steps,
        };
        let refined = {
            let mut authority = |candidate| {
                !should_stop()
                    && self
                        .evaluate_motif_unit_at(motif, candidate, config)
                        .is_some()
            };
            refine_translation_exact(anchor_dt, refinement, &mut authority)?.transform()
        };
        if should_stop() {
            return None;
        }
        self.contract_backoff_poses(refined).into_iter().find_map(|pose| {
            let unit = self.evaluate_motif_unit_at(motif, pose, config)?;
            let members = unit.members.iter().map(|(_, placement)| *placement).collect_vec();
            self.original_contract_allows(&members).then_some(unit)
        })
    }

    fn evaluate_motif_unit_at(
        &self,
        motif: &PlacementMotif,
        anchor_dt: DTransformation,
        config: ProfilePlacementConfig,
    ) -> Option<StructurePlacementCandidate> {
        let anchor = motif.members.first()?;
        if angular_distance(anchor_dt.rotation(), anchor.relative_pose.rotation()) > 1.0e-4 {
            return None;
        }
        let (ax, ay) = anchor_dt.translation();
        let (rx, ry) = anchor.relative_pose.translation();
        let delta = (ax - rx, ay - ry);
        let mut trial = self.prob.clone();
        let mut members = Vec::with_capacity(motif.members.len());
        let mut total_loss = 0.0;
        for member in &motif.members {
            if trial.item_demand_qtys()[member.item.item_id] == 0 {
                return None;
            }
            let item = self.instance.item(member.item.item_id);
            let (mx, my) = member.relative_pose.translation();
            let transform = DTransformation::new(
                member.relative_pose.rotation(),
                (mx + delta.0, my + delta.1),
            );
            if !rotation_is_allowed(item, transform.rotation()) {
                return None;
            }
            let mut evaluator = ProfilePlacementEvaluator::new(&trial.layout(), item, config).ok()?;
            let SampleEval::Clear { loss } = evaluator.evaluate_sample(transform, None) else {
                return None;
            };
            total_loss += loss;
            let placement = SPPlacement {
                item_idx: member.item.item_id,
                d_transf: transform,
            };
            trial.place_item(placement);
            members.push((member.item, placement));
        }
        // Small normalized preference lets a valid multi-item hypothesis
        // compete as a unit without making it rigid after construction.
        let mean_loss = total_loss / members.len().max(1) as f32;
        Some(StructurePlacementCandidate {
            loss: mean_loss
                - 0.02 * motif.score.total.clamp(0.0, 1.0)
                - 1.0e-3 * (members.len().saturating_sub(1) as f32),
            members,
            tag: StructureCandidateTag::Motif(structure_motif_kind(&motif.kind)),
        })
    }

    fn commit_structure_candidate(&mut self, candidate: StructurePlacementCandidate) {
        let mut placed = Vec::with_capacity(candidate.members.len());
        for (item_ref, placement) in candidate.members {
            let pk = self.prob.place_item(placement);
            self.placed_copies.insert(item_ref, placement.d_transf);
            placed.push(pk);
        }
        if let Some(frontier) = self.profile_frontier.as_mut() {
            for pk in &placed {
                if let Ok(tagged) = tag_spolygon(&self.prob.layout().placed_items()[*pk].shape()) {
                    frontier.add_polygon(&tagged);
                }
            }
        }
        #[cfg(feature = "capmods")]
        if let Some(raster) = self.raster.as_mut() {
            for pk in &placed {
                raster.mark_placed(&self.prob.layout().placed_items()[*pk].shape());
            }
        }
    }

    fn structure_candidate_is_bidirectionally_clear(
        &self,
        candidate: &StructurePlacementCandidate,
    ) -> bool {
        let mut candidate_shapes = Vec::with_capacity(candidate.members.len());
        for (_, placement) in &candidate.members {
            let item = self.instance.item(placement.item_idx);
            let mut shape = item.shape_cd().as_ref().clone();
            shape.transform_from(&item.shape_cd(), &placement.d_transf.compose());
            if self.prob.layout().placed_items().values().any(|placed| {
                !self
                    .place_groups
                    .same_group(placement.item_idx, placed.item().idx())
                    && polygons_collide_bidirectionally(&shape, &placed.shape())
            }) || candidate_shapes.iter().any(|(id, other)| {
                !self.place_groups.same_group(placement.item_idx, *id)
                    && polygons_collide_bidirectionally(&shape, other)
            }) {
                return false;
            }
            candidate_shapes.push((placement.item_idx, shape));
        }
        true
    }

    fn record_structure_commit(
        &mut self,
        tag: StructureCandidateTag,
        anchor: MotifItemRef,
        member_count: usize,
    ) {
        let Some(stats) = self.structure_stats.as_mut() else {
            return;
        };
        let (source, motif_kind, source_stats) = match tag {
            StructureCandidateTag::Single => (
                StructureSelectionSource::Single,
                None,
                &mut stats.sources.single,
            ),
            StructureCandidateTag::Motif(kind) => (
                StructureSelectionSource::Motif,
                Some(kind),
                &mut stats.sources.motif,
            ),
        };
        source_stats.committed_units += 1;
        source_stats.committed_members += member_count;
        if let Some(kind) = motif_kind {
            let kind_stats = stats.motif_kinds.for_kind_mut(kind);
            kind_stats.committed_units += 1;
            kind_stats.committed_members += member_count;
        }
        let selection_index = stats.start.selection_order.len();
        stats.start.committed_units += 1;
        stats.start.committed_members += member_count;
        stats.start.selection_order.push(StructureSelectionStats {
            selection_index,
            source,
            motif_kind,
            anchor_item_id: anchor.item_id,
            anchor_copy_index: anchor.copy_index,
            member_count,
        });
    }

    fn record_structure_bidirectional_rejection(&mut self, tag: StructureCandidateTag) {
        let Some(stats) = self.structure_stats.as_mut() else {
            return;
        };
        let (source, motif_kind) = match tag {
            StructureCandidateTag::Single => (&mut stats.sources.single, None),
            StructureCandidateTag::Motif(kind) => (&mut stats.sources.motif, Some(kind)),
        };
        source.exact_feasible = source.exact_feasible.saturating_sub(1);
        source.bidirectional_rejected += 1;
        if let Some(kind) = motif_kind {
            let kind_stats = stats.motif_kinds.for_kind_mut(kind);
            kind_stats.exact_feasible = kind_stats.exact_feasible.saturating_sub(1);
            kind_stats.bidirectional_rejected += 1;
        }
    }

    /// Marks this batch as stalled and decides whether P1 may continue.
    ///
    /// Returns `false` only after all remaining item/copy anchors have been
    /// visited and the bounded strip recovery is exhausted. The caller then
    /// returns the current partial problem; this is a normal P1 outcome, not a
    /// panic. A successful commit clears both the stalled set and the per-step
    /// expansion counter.
    fn recover_structure_stall(
        &mut self,
        beam: &[MotifItemRef],
        remaining_len: usize,
        stalled: &mut HashSet<MotifItemRef>,
        recovery_expansions: &mut usize,
    ) -> bool {
        stalled.extend(beam.iter().copied());
        if let Some(stats) = self.structure_stats.as_mut() {
            stats.start.recovery_rounds += 1;
        }
        if stalled.len() < remaining_len {
            return true;
        }

        if *recovery_expansions < MAX_STRUCTURE_RECOVERY_EXPANSIONS
            && self.try_expand_structure_strip()
        {
            *recovery_expansions += 1;
            stalled.clear();
            return true;
        }

        if let Some(stats) = self.structure_stats.as_mut() {
            stats.start.recovery_exhausted = true;
        }
        false
    }

    /// Structure-only bounded width growth. Unlike the historical recursive
    /// item path this never asserts: an invalid/non-finite next width simply
    /// ends P1 with a partial result.
    fn try_expand_structure_strip(&mut self) -> bool {
        let next_width = self.prob.strip_width() * 1.2;
        let max_width = 2.0
            * self
                .prob
                .instance()
                .items
                .iter()
                .map(|(item, qty)| item.shape_cd().diameter() * *qty as f32)
                .sum::<f32>();
        if !next_width.is_finite() || !max_width.is_finite() || next_width >= max_width {
            return false;
        }
        if let Some(stats) = self.structure_stats.as_mut() {
            stats.start.strip_expansions += 1;
        }
        crate::util::frozen::change_strip_width(&mut self.prob, next_width);
        #[cfg(feature = "capmods")]
        self.init_raster();
        if let Some(config) = self.profile_cfg {
            self.profile_frontier = build_profile_frontier(&self.prob.layout(), config).ok();
        }
        true
    }

    /// 현재 컨테이너에 맞는 점유 격자를 만들고 이미 놓인 아이템을 반영한다.
    #[cfg(feature = "capmods")]
    fn init_raster(&mut self) {
    #[cfg(feature = "init-profile")]
    let _profile = crate::init_profile::Scope::new("detail.raster_init");

        let n_items = self.instance.items.len();
        let min_item_dim = (0..n_items)
            .map(|id| {
                let bbox = self.instance.item(id).shape_cd().bbox();
                OrderedFloat(f32::min(bbox.width(), bbox.height()))
            })
            .min()
            .map(|v| v.0)
            .unwrap_or(0.0);

        let container = self.prob.layout().container().outer_cd().bbox();
        self.raster = RasterNfp::new(container, min_item_dim, &self.raster_cfg);
        if let Some(raster) = self.raster.as_mut() {
            raster.rebuild(&self.prob.layout());
        }
    }

    fn place_item(&mut self, item_ref: MotifItemRef, should_stop: &mut dyn FnMut() -> bool) {
        if should_stop() {
            return;
        }
        let item_id = item_ref.item_id;
        match self.find_placement(item_ref, should_stop) {
            Some(p_opt) => {
                let targets = self
                    .place_groups
                    .target_transforms(None, item_id, p_opt.d_transf)
                    .expect(crate::dmsg!("validated place group definition became inconsistent"));

    #[cfg(feature = "init-profile")]
    let _profile_commit = crate::init_profile::Scope::new("detail.commit_placement");
                let mut placed = Vec::with_capacity(targets.len());
                for (member_id, member_dt) in targets {
                    let pk = self.prob.place_item(SPPlacement {
                        item_idx: member_id,
                        d_transf: member_dt,
                    });
                    placed.push(pk);
                }
                if self.place_groups.is_empty() && placed.len() == 1 {
                    self.placed_copies.insert(item_ref, p_opt.d_transf);
                }
                if let Some(frontier) = self.profile_frontier.as_mut() {
                    for pk in &placed {
                        if let Ok(tagged) = tag_spolygon(&self.prob.layout().placed_items()[*pk].shape())
                        {
                            frontier.add_polygon(&tagged);
                        }
                    }
                }

                // 새로 놓인 도형을 점유 격자에 반영한다 (증분 갱신)
                #[cfg(feature = "capmods")]
                if let Some(raster) = self.raster.as_mut() {
                    for pk in &placed {
                        raster.mark_placed(&self.prob.layout().placed_items()[*pk].shape());
                    }
                }
                let _ = placed;
            }
            None => {
                if should_stop() {
                    return;
                }
                // 구간 상한이 걸린 동안의 실패는 "이 구간에 안 들어간다" 이지
                // "원단이 좁다" 가 아니다. 여기서 넓히면 상한 때문에 영원히 못 넣고
                // 스트립만 폭주한다. 호출자(구간 루프)가 구간을 넘기게 그대로 돌려준다.
                if self.band_x_max.is_some() {
                    return;
                }
                #[cfg(feature = "init-profile")]
                crate::init_profile::count("lbf.width_expansions", 1);
                let width = self.prob.strip_width() * 1.2;
                crate::util::frozen::change_strip_width(&mut self.prob, width);
                assert!(
                    assertions::strip_width_is_in_check(&self.prob),
                    "strip-width is running away (>{:.3}), item {item_id} does not seem to fit into the strip",
                    self.prob.strip_width()
                );

                // 컨테이너가 바뀌면 격자 좌표계가 무효가 되므로 다시 만든다
                #[cfg(feature = "capmods")]
                self.init_raster();
                if let Some(profile_cfg) = self.profile_cfg {
                    self.profile_frontier =
                        build_profile_frontier(&self.prob.layout(), profile_cfg).ok();
                }

                self.place_item(item_ref, should_stop);
            }
        }
    }

    fn find_placement(
        &mut self,
        item_ref: MotifItemRef,
        should_stop: &mut dyn FnMut() -> bool,
    ) -> Option<SPPlacement> {
        let candidate = self.find_placement_unchecked(item_ref, should_stop)?;
        self.certify_legacy_placement(candidate, should_stop)
    }

    fn certify_legacy_placement(
        &mut self,
        candidate: SPPlacement,
        should_stop: &mut dyn FnMut() -> bool,
    ) -> Option<SPPlacement> {
    #[cfg(feature = "init-profile")]
    let _profile = crate::init_profile::Scope::new("detail.placement_certificate");

        let Some(mut guard) = self.original_guard.take() else {
            return Some(candidate);
        };
        let accepted = (|| {
            if should_stop() || guard.sync_construction_layout(&self.prob.layout()).is_err() {
                return None;
            }
            let members_at = |pose| -> Vec<SPPlacement> {
                self.place_groups
                    .target_transforms(None, candidate.item_idx, pose)
                    .expect(crate::dmsg!("validated place group definition became inconsistent"))
                    .into_iter()
                    .map(|(item_id, d_transf)| SPPlacement { item_idx: item_id, d_transf })
                    .collect()
            };
            let members = members_at(candidate.d_transf);
            if matches!(guard.construction_allows(&members), Ok(true)) {
                return Some(candidate);
            }

            // Only rejected winners pay for another exact CDE. Never spend original
            // outline intersection work on every sampler proposal.
            let mut evaluator = GroupLBFEvaluator::new(
                &self.prob.layout(),
                &self.instance,
                &self.place_groups,
                candidate.item_idx,
                self.lbf_x_multiplier,
                self.lbf_y_multiplier,
            );
            let mut allowed = |pose| {
                if !matches!(
                    evaluator.evaluate_sample(pose, None),
                    SampleEval::Clear { .. }
                ) {
                    return false;
                }
                if self.band_x_max.is_some_and(|limit| {
                    evaluator
                        .shape_buffers
                        .iter()
                        .any(|(_, shape)| shape.bbox().x_max > limit)
                }) {
                    return false;
                }
                matches!(guard.construction_allows(&members_at(pose)), Ok(true))
            };
            let (x, y) = candidate.d_transf.translation();
            // At 100m, an f32 coordinate advances in ~0.008mm increments. A fixed
            // micrometre nudge can round back to the same rejected pose.
            let quantum = (2.0 * f32::EPSILON * x.abs().max(y.abs())).max(0.002);
            for step in [
                quantum,
                quantum.max(0.005),
                quantum.max(0.02),
                quantum.max(0.1),
                quantum.max(0.5),
                quantum.max(2.0),
            ] {
                for (dx, dy) in [
                    (1.0, 0.0),
                    (0.0, 1.0),
                    (1.0, 1.0),
                    (-1.0, 0.0),
                    (0.0, -1.0),
                    (1.0, -1.0),
                    (-1.0, 1.0),
                    (-1.0, -1.0),
                ] {
                    if should_stop() {
                        return None;
                    }
                    let pose = DTransformation::new(
                        candidate.d_transf.rotation(),
                        (x + dx * step, y + dy * step),
                    );
                    if allowed(pose) {
                        return Some(SPPlacement {
                            d_transf: pose,
                            ..candidate
                        });
                    }
                }
            }

            // If a tight pocket has no legal nudge, try unused space to the right.
            // Existing width/band recovery still owns any container expansion.
            let right = guard.construction_right_edge().max(
                self.prob
                    .layout()
                    .placed_items()
                    .values()
                    .map(|p| p.shape().bbox().x_max as f64)
                    .fold(0.0, f64::max),
            );
            let left = members
                .iter()
                .map(|p| guard.outline(p.item_idx, p.d_transf).map(|o| o.bounds()[0]))
                .collect::<Result<Vec<_>, _>>()
                .ok()?
                .into_iter()
                .fold(f64::INFINITY, f64::min);
            let clearance = (4.0 * f32::EPSILON as f64 * right.abs()).max(0.01);
            let pose = DTransformation::new(
                candidate.d_transf.rotation(),
                ((x as f64 + right - left + clearance) as f32, y),
            );
            if !should_stop() && allowed(pose) {
                Some(SPPlacement {
                    d_transf: pose,
                    ..candidate
                })
            } else {
                None
            }
        })();
        self.original_guard = Some(guard);
        accepted
    }

    fn find_placement_unchecked(
        &mut self,
        item_ref: MotifItemRef,
        should_stop: &mut dyn FnMut() -> bool,
    ) -> Option<SPPlacement> {
    #[cfg(feature = "init-profile")]
    let _profile = crate::init_profile::Scope::new("detail.find_placement");

        if should_stop() {
            return None;
        }
        let item_id = item_ref.item_id;
        if self.place_groups.group_id_of(item_id).is_some() {
            let layout = &self.prob.layout();
            let item = self.instance.item(item_id);
            let evaluator = GroupLBFEvaluator::new(
                layout,
                &self.instance,
                &self.place_groups,
                item_id,
                self.lbf_x_multiplier,
                self.lbf_y_multiplier,
            );
            let (best_sample, _) = search_placement_prepared_with_control(
                layout,
                item,
                None,
                evaluator,
                self.sample_config,
                &mut self.rng,
                should_stop,
                &self.prepared,
            );
            return match best_sample {
                Some((d_transf, SampleEval::Clear { .. })) => {
                    Some(SPPlacement { item_idx: item_id, d_transf })
                }
                _ => None,
            };
        }

        // P1 profile construction. P0 motif poses are evaluated first, but
        // remain ordinary seeds: the exact-authorized evaluator can reject
        // them and the normal random candidate pool remains available.
        if let Some(profile_cfg) = self.profile_cfg {
            let seeds = self.motif_seeds(item_ref);
            let layout = &self.prob.layout();
            let item = self.instance.item(item_id);
            let evaluator = match self.profile_frontier.as_ref() {
                Some(frontier) => {
                    ProfilePlacementEvaluator::with_frontier(layout, item, frontier, profile_cfg)
                }
                None => ProfilePlacementEvaluator::new(layout, item, profile_cfg),
            };
            match evaluator {
                Ok(evaluator) => {
                    let (best_sample, _) =
                        search_placement_seeded_stochastic_top_k_prepared_with_control(
                            layout,
                            item,
                            None,
                            &seeds,
                            evaluator,
                            self.sample_config,
                            profile_cfg.top_k,
                            &mut self.rng,
                            should_stop,
                            &self.prepared,
                        );
                    if let Some((d_transf, SampleEval::Clear { .. })) = best_sample {
                        return Some(SPPlacement { item_idx: item_id, d_transf });
                    }
                }
                Err(_) => {}
            }
        }

        // capmods: 래스터 NFP가 "겹치지 않는 격자 위치"를 손실 순으로 뽑아 시드로 넣는다.
        // 시드는 아래 CDE 평가기(LBFEvaluator)로 전부 검증되므로, 격자가 근사여도
        // 잘못된 배치가 채택될 수 없다. 좁은 포켓을 난수로 놓치는 문제만 사라진다.
        #[cfg(feature = "capmods")]
        {
            let (x_mult, y_mult) = (self.lbf_x_multiplier, self.lbf_y_multiplier);
            // `double_sided_first_n` 은 **앞쪽 N 조각**에만 적용한다. 전 구간에
            // 적용하면 위·아래로 몰려 가운데가 비고 오히려 나빠진다(실측
            // 73.350% -> 72.696%). 래스터 쪽은 `> 0` 만 보므로 여기서 꺼 준다.
            let band_x_max = self.band_x_max;
            let mut raster_cfg = self.raster_cfg;
            let double_sided =
                self.prob.layout().placed_items().len() < raster_cfg.double_sided_first_n;
            if !double_sided {
                raster_cfg.double_sided_first_n = 0;
            }
            let item = self.instance.item(item_id);
            let (seeds, n_pocket) = match self.raster.as_mut() {
                Some(raster) => raster.candidates_split(item, x_mult, y_mult, &raster_cfg),
                None => (vec![], 0),
            };

            // ── 홀 우선 부착 ────────────────────────────────────────────────
            //
            // 포켓 후보를 목록 앞에 놓기만 해서는 아무 일도 일어나지 않는다. 아래의
            // `search_placement_seeded_prepared_with_control` 이 시드 전체를 `LBFEvaluator`
            // (좌·하단 손실 최소)로 채점해 **하나**를 고르는데, 포켓은 이미 배치된
            // 조각들 사이에 있어 x 가 크고 따라서 손실이 크다 — argmin 에서 언제나 진다.
            // 실측이 그것을 확인해 줬다: 순서만 바꿨을 때 폭·밀도가 한 비트도 안 변했고,
            // 진단 카운터는 포켓 후보가 203개 실제로 만들어졌음을 보여줬다.
            //
            // 그래서 포켓 구간을 **먼저 따로** 시도한다. 난수 샘플을 0으로 두어 이
            // 시도가 프론티어 자리로 새지 않게 한다. 하나라도 CDE 를 통과하면 그중
            // 손실이 가장 작은 것(= 가장 왼쪽·아래 포켓)을 채택한다.
            //
            // 폴리시 단계의 `optimizer::polish::holes` 가 사후에 같은 일을 하지만,
            // 그때는 배치가 굳어 있어 넣을 조각이 얼마 남지 않는다. 여기서 먼저 시도한다.
            if raster_cfg.hole_first && n_pocket > 0 {
                let layout = &self.prob.layout();
                let evaluator = LBFEvaluator::new(layout, item, x_mult, y_mult)
                    .double_sided(double_sided)
                    .band_x_max(band_x_max);
                let mut pocket_config = self.sample_config;
                pocket_config.n_container_samples = 0;
                let (best_pocket, _) = search_placement_seeded_prepared_with_control(
                    layout,
                    item,
                    None,
                    &seeds[..n_pocket],
                    evaluator,
                    pocket_config,
                    &mut self.rng,
                    should_stop,
                    &self.prepared,
                );
                if let Some((d_transf, SampleEval::Clear { .. })) = best_pocket {
                    crate::sample::raster_nfp::hole_stats::POCKET_ACCEPTED
                        .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    return Some(SPPlacement { item_idx: item_id, d_transf });
                }
                // 포켓 후보가 전부 CDE 에서 막혔다 — 아래 기존 경로가 그대로 이어받는다.
            }

            if !seeds.is_empty() {
                let layout = &self.prob.layout();
                let evaluator = LBFEvaluator::new(layout, item, x_mult, y_mult)
                    .double_sided(double_sided)
                    .band_x_max(band_x_max);
                // 래스터 샘플은 보험 수준으로 줄인다.
                let mut sample_config = self.sample_config;
                sample_config.n_container_samples = raster_cfg.n_random_samples;

                // exact NFP: 후반부 항목(전체의 90% 이상 배치 완료)에만 접촉 슬라이딩을 둘러준다.
                // 초반은 공간이 넣넨해 greedy bias만 강화되므로 raster-only로 둔다.
                let n_total = self.instance.total_item_qty();
                let n_placed = self.prob.layout().placed_items().len();
                let seeds = if n_placed as f32 / n_total as f32 >= 0.9 {
                    exact_nfp::refine_candidates(
                        layout,
                        item,
                        &seeds,
                        x_mult,
                        y_mult,
                        raster_cfg.n_candidates,
                    )
                } else {
                    seeds
                };

                let (best_sample, _) = search_placement_seeded_prepared_with_control(
                    layout,
                    item,
                    None,
                    &seeds,
                    evaluator,
                    sample_config,
                    &mut self.rng,
                    should_stop,
                    &self.prepared,
                );
                if let Some((d_transf, SampleEval::Clear { .. })) = best_sample {
                    return Some(SPPlacement { item_idx: item_id, d_transf });
                }
                // 시드가 전부 CDE에서 막히면 아래 원본 경로로 한 번 더 시도한다.
            }
        }

        let layout = &self.prob.layout();
        let item = self.instance.item(item_id);
        // 이 폴백에도 구간 상한을 건다. 빠뜨리면 위의 경로가 전부 막힌 조각이
        // 여기서 구간 밖에 놓여 구간 완성이 이름만 남는다.
        let evaluator =
            LBFEvaluator::new(layout, item, self.lbf_x_multiplier, self.lbf_y_multiplier)
                .band_x_max(self.band_x_max);

        let (best_sample, _) = search_placement_prepared_with_control(
            layout,
            item,
            None,
            evaluator,
            self.sample_config,
            &mut self.rng,
            should_stop,
            &self.prepared,
        );

        match best_sample {
            Some((d_transf, SampleEval::Clear { .. })) => Some(SPPlacement { item_idx: item_id, d_transf }),
            _ => None,
        }
    }

    /// Translate every applicable relative P0 pose from an already placed
    /// member. If this is the first member, the motif's normalized origin is a
    /// useful left/bottom seed. The later exact evaluator is the sole
    /// acceptance authority.
    fn motif_seeds(&self, item_ref: MotifItemRef) -> Vec<DTransformation> {
        let mut seeds = vec![];
        for motif in &self.motifs {
            let Some(member) = motif.members.iter().find(|member| member.item == item_ref) else {
                continue;
            };

            let anchor = motif.members.iter().find_map(|known| {
                let actual = self.placed_copies.get(&known.item)?;
                if angular_distance(actual.rotation(), known.relative_pose.rotation())
                    > 5.0_f32.to_radians()
                {
                    return None;
                }
                let (actual_x, actual_y) = actual.translation();
                let (relative_x, relative_y) = known.relative_pose.translation();
                Some((actual_x - relative_x, actual_y - relative_y))
            });
            let (anchor_x, anchor_y) = anchor.unwrap_or((0.0, 0.0));
            let (relative_x, relative_y) = member.relative_pose.translation();
            let candidate = DTransformation::new(
                member.relative_pose.rotation(),
                (relative_x + anchor_x, relative_y + anchor_y),
            );
            if !seeds.iter().any(|seed: &DTransformation| {
                angular_distance(seed.rotation(), candidate.rotation()) < 1.0e-5
                    && (seed.translation().0 - candidate.translation().0).abs() < 1.0e-4
                    && (seed.translation().1 - candidate.translation().1).abs() < 1.0e-4
            }) {
                seeds.push(candidate);
            }
        }
        seeds
    }
}

fn structure_motif_kind(kind: &MotifKind) -> StructureMotifKind {
    match kind {
        MotifKind::Single(_) => StructureMotifKind::Single,
        MotifKind::Pair(_) => StructureMotifKind::Pair,
        MotifKind::RectangleBlock(_) => StructureMotifKind::RectangleBlock,
        MotifKind::RepeatedBand(_) => StructureMotifKind::RepeatedBand,
    }
}

fn motif_kind_attempted(stats: StructureMotifKindStats) -> usize {
    stats.single.attempted
        + stats.pair.attempted
        + stats.rectangle_block.attempted
        + stats.repeated_band.attempted
}

fn motif_kind_exact_feasible(stats: StructureMotifKindStats) -> usize {
    stats.single.exact_feasible
        + stats.pair.exact_feasible
        + stats.rectangle_block.exact_feasible
        + stats.repeated_band.exact_feasible
}

fn angular_distance(a: f32, b: f32) -> f32 {
    let delta = a - b;
    delta.sin().atan2(delta.cos()).abs()
}

fn append_unique_transforms(
    destination: &mut Vec<DTransformation>,
    source: impl IntoIterator<Item = DTransformation>,
) {
    for candidate in source {
        if !destination.iter().any(|existing| {
            angular_distance(existing.rotation(), candidate.rotation()) < 1.0e-5
                && (existing.translation().0 - candidate.translation().0).abs() < 1.0e-4
                && (existing.translation().1 - candidate.translation().1).abs() < 1.0e-4
        }) {
            destination.push(candidate);
        }
    }
}

fn rotation_is_allowed(item: &jagua_rs::entities::Item, rotation: f32) -> bool {
    use jagua_rs::geometry::geo_enums::RotationRange;
    match item.allowed_orientations().rotations() {
        RotationRange::None => angular_distance(rotation, 0.0) <= 1.0e-4,
        RotationRange::Continuous => true,
        RotationRange::Discrete(rotations) => rotations
            .iter()
            .any(|allowed| angular_distance(rotation, *allowed) <= 1.0e-4),
    }
}

/// Evaluates an anchor sample against every member polygon. The group is not
/// yet present in the layout during fresh LBF, so `NoFilter` correctly checks
/// container and already placed units. User-defined internal overlaps are
/// exempt because the members form one rigid placement unit.
struct GroupLBFEvaluator<'a> {
    layout: &'a Layout,
    instance: &'a SPInstance,
    groups: &'a PlaceGroups,
    anchor_item_id: usize,
    shape_buffers: Vec<(usize, SPolygon)>,
    x_multiplier: f32,
    y_multiplier: f32,
    n_evals: usize,
    exact_cde: CDEngine,
}

impl<'a> GroupLBFEvaluator<'a> {
    fn new(
        layout: &'a Layout,
        instance: &'a SPInstance,
        groups: &'a PlaceGroups,
        anchor_item_id: usize,
        x_multiplier: f32,
        y_multiplier: f32,
    ) -> Self {
        let shape_buffers = groups
            .member_item_ids(anchor_item_id)
            .into_iter()
            .map(|id| (id, instance.item(id).shape_cd().as_ref().clone()))
            .collect();
        Self {
            layout,
            instance,
            groups,
            anchor_item_id,
            shape_buffers,
            x_multiplier,
            y_multiplier,
            n_evals: 0,
            exact_cde: flat_exact_cde(layout),
        }
    }
}

impl SampleEvaluator for GroupLBFEvaluator<'_> {
    fn evaluate_sample(
        &mut self,
        dt: DTransformation,
        _upper_bound: Option<SampleEval>,
    ) -> SampleEval {
        self.n_evals += 1;
        let targets = self
            .groups
            .target_transforms(None, self.anchor_item_id, dt)
            .expect(crate::dmsg!("validated place group definition became inconsistent"));
        let cde = self.layout.cde();
        let mut loss = 0.0;

        for (item_id, member_dt) in targets {
            let item = self.instance.item(item_id);
            let transf = member_dt.compose();
            if cde.detect_surrogate_collision(item.shape_cd().surrogate(), &transf, &NoFilter) {
                return SampleEval::Invalid;
            }
            let (_, shape) = self
                .shape_buffers
                .iter_mut()
                .find(|(id, _)| *id == item_id)
                .expect(crate::dmsg!("shape buffer must exist for every group member"));
            shape.transform_from(&item.shape_cd(), &transf);
            if self.exact_cde.detect_poly_collision(shape, &NoFilter) {
                return SampleEval::Invalid;
            }
            let poi = shape.poi().center;
            let bbox_corner = shape.bbox().corners()[0];
            loss += self.x_multiplier * (poi.0 + bbox_corner.0)
                + self.y_multiplier * (poi.1 + bbox_corner.1);
        }
        SampleEval::Clear { loss }
    }

    fn n_evals(&self) -> usize {
        self.n_evals
    }
}

#[cfg(test)]
mod order_mode_tests {
    use super::*;

    /// 인덱스 0 이 가장 큰 조각인, 크기 내림차순 목록.
    fn desc(n: usize) -> Vec<usize> {
        (0..n).collect()
    }

    #[test]
    fn largest_first_is_identity() {
        assert_eq!(LbfOrderMode::LargestFirst.arrange(desc(8)), desc(8));
    }

    #[test]
    fn smallest_first_reverses() {
        assert_eq!(
            LbfOrderMode::SmallestFirst.arrange(desc(8)),
            vec![7, 6, 5, 4, 3, 2, 1, 0]
        );
    }

    #[test]
    fn middle_out_large_first_alternates_from_centre() {
        // n=8 이면 중앙은 4. 큰 쪽(작은 인덱스)을 먼저 집는다.
        assert_eq!(
            LbfOrderMode::MiddleOutLargeFirst.arrange(desc(8)),
            vec![4, 3, 5, 2, 6, 1, 7, 0]
        );
        // 홀수도 같은 규칙. 한쪽이 먼저 바닥나면 남은 쪽을 그대로 잇는다.
        assert_eq!(
            LbfOrderMode::MiddleOutLargeFirst.arrange(desc(7)),
            vec![3, 2, 4, 1, 5, 0, 6]
        );
    }

    #[test]
    fn middle_out_small_first_alternates_from_centre() {
        assert_eq!(
            LbfOrderMode::MiddleOutSmallFirst.arrange(desc(8)),
            vec![4, 5, 3, 6, 2, 7, 1, 0]
        );
        assert_eq!(
            LbfOrderMode::MiddleOutSmallFirst.arrange(desc(7)),
            vec![3, 4, 2, 5, 1, 6, 0]
        );
    }

    /// 재배열은 **순열**이어야 한다. 조각을 하나라도 잃거나 복제하면
    /// 그 뒤의 수량 검증(`QuantityMismatch`)에서야 터지고, 원인이 여기라는 걸
    /// 알아내기 어렵다.
    #[test]
    fn every_mode_is_a_permutation_at_every_length() {
        let modes = [
            LbfOrderMode::LargestFirst,
            LbfOrderMode::SmallestFirst,
            LbfOrderMode::MiddleOutLargeFirst,
            LbfOrderMode::MiddleOutSmallFirst,
            LbfOrderMode::ConcavityFirst,
            LbfOrderMode::SizeTimesConcavity,
        ];
        for n in 0..40usize {
            for mode in modes {
                let out = mode.arrange(desc(n));
                assert_eq!(out.len(), n, "{:?} n={n} 길이가 변했다", mode);
                let mut sorted = out.clone();
                sorted.sort_unstable();
                assert_eq!(sorted, desc(n), "{:?} n={n} 원소 집합이 변했다", mode);
            }
        }
    }

    /// 오목도 모드는 **키**가 다를 뿐이다 — 재배열은 항등이어야 한다. 여기가 뒤집히면
    /// "오목한 것부터" 가 조용히 "오목한 것 나중에" 가 된다.
    #[test]
    fn concavity_modes_keep_the_sorted_order() {
        for mode in [
            LbfOrderMode::ConcavityFirst,
            LbfOrderMode::SizeTimesConcavity,
        ] {
            assert_eq!(mode.arrange(desc(9)), desc(9), "{mode:?}");
        }
    }

    #[test]
    fn degenerate_lengths_are_untouched() {
        for mode in [
            LbfOrderMode::SmallestFirst,
            LbfOrderMode::MiddleOutLargeFirst,
            LbfOrderMode::MiddleOutSmallFirst,
        ] {
            assert!(mode.arrange(desc(0)).is_empty());
            assert_eq!(mode.arrange(desc(1)), vec![0]);
        }
    }

    /// 두 중간부터 모드는 서로의 거울이 **아니다** (같은 중앙에서 출발하므로).
    /// 실수로 한쪽을 다른 쪽의 `rev()` 로 구현하는 걸 막는다.
    #[test]
    fn the_two_middle_out_modes_differ() {
        let a = LbfOrderMode::MiddleOutLargeFirst.arrange(desc(9));
        let b = LbfOrderMode::MiddleOutSmallFirst.arrange(desc(9));
        assert_ne!(a, b);
        assert_eq!(a[0], b[0], "출발점(중앙)은 같아야 한다");
    }

    #[test]
    fn from_raw_maps_abi_values_and_clamps_out_of_range() {
        assert_eq!(LbfOrderMode::from_raw(0), LbfOrderMode::LargestFirst);
        assert_eq!(LbfOrderMode::from_raw(1), LbfOrderMode::SmallestFirst);
        assert_eq!(LbfOrderMode::from_raw(2), LbfOrderMode::MiddleOutLargeFirst);
        assert_eq!(LbfOrderMode::from_raw(3), LbfOrderMode::MiddleOutSmallFirst);
        assert_eq!(LbfOrderMode::from_raw(4), LbfOrderMode::ConcavityFirst);
        assert_eq!(LbfOrderMode::from_raw(5), LbfOrderMode::SizeTimesConcavity);
        // 범위 밖은 현행 동작으로. panic 하지 않는다.
        for v in [-1, 6, 99, i32::MIN, i32::MAX] {
            assert_eq!(LbfOrderMode::from_raw(v), LbfOrderMode::LargestFirst);
        }
    }

    /// 함정 1 방어: 모듈만 만들고 **호출하는 줄을 빠뜨리는** 실수.
    /// `arrange` 가 실제 정렬 경로에서 불리는지 소스에서 단정한다.
    #[test]
    fn arrange_is_actually_called_in_the_sort_path() {
        let src = include_str!("lbf.rs");
        // 테스트 모듈 자신의 문자열에 걸리지 않도록 생산 코드만 자른다.
        let production = &src[..src.find("#[cfg(test)]").expect(crate::dmsg!("테스트 모듈 경계"))];
        assert!(
            !production.contains("order_mode_tests"),
            "생산 코드 절단이 잘못됐다 — 테스트 모듈이 포함됐다"
        );
        assert!(
            production.contains("self.order_mode.arrange(sorted_item_ids)"),
            "정렬 결과에 order_mode 가 적용되지 않는다"
        );
    }
}

#[cfg(test)]
mod structure_hint_contract_tests {
    use super::*;
    use crate::config::DEFAULT_SWARM_CONFIG;
    use crate::consts::LBF_SAMPLE_CONFIG;
    use crate::original_overlap::OriginalOverlapGuard;
    use crate::structure::pipeline::solution_validation_error;
    use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
    use rand::SeedableRng;

    /// 같은 직사각형 12장, 조각 겹침 허용 0.4mm.
    fn rectangles_with_overlap_allowance() -> SPInstance {
        let items: Vec<_> = (0..12)
            .map(|id| {
                serde_json::json!({
                    "id": id, "demand": 1, "allowed_orientations": [0.0],
                    "shape": {"type": "simple_polygon",
                              "data": [[0.0, 0.0], [40.0, 0.0], [40.0, 30.0], [0.0, 30.0]]}
                })
            })
            .collect();
        let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
            "name": "hint-contract", "strip_height": 100.0, "items": items
        }))
        .unwrap();
        let c = DEFAULT_SWARM_CONFIG;
        crate::util::io::import_spp_instance(
            &ext,
            c.cde_config,
            c.poly_simpl_tolerance,
            Some(-0.4),
            c.narrow_concavity_cutoff_ratio,
        )
        .unwrap()
    }

    #[test]
    fn legacy_construction_certifies_original_outlines_without_repair() {
        let instance = rectangles_with_overlap_allowance();
        for seed in 1..=8 {
            let prob = LBFBuilder::new(
                instance.clone(),
                Xoshiro256PlusPlus::seed_from_u64(seed),
                LBF_SAMPLE_CONFIG,
                5.0,
                1.0,
            )
            .construct()
            .prob;
            assert!(prob.item_demand_qtys().iter().all(|qty| *qty == 0));
            let error = solution_validation_error(&instance, &prob.save());
            assert!(error.is_none(), "seed {seed}: {error:?}");
        }
    }

    #[test]
    fn legacy_backoff_certifies_large_coordinates_and_preserves_existing_pose() {
        let instance = rectangles_with_overlap_allowance();
        let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
        prob.change_strip_width(140_000.0).expect(crate::dmsg!("strip width must be positive"));
        let fixed = SPPlacement {
            item_idx: 0,
            d_transf: DTransformation::new(0.0, (100_000.0, 50.0)),
        };
        prob.place_item(fixed);
        let mut builder = LBFBuilder::from_problem(
            instance.clone(),
            prob,
            Xoshiro256PlusPlus::seed_from_u64(1),
            LBF_SAMPLE_CONFIG,
            5.0,
            1.0,
        );
        builder.original_guard = Some(OriginalOverlapGuard::new(&instance));
        let candidate = SPPlacement {
            item_idx: 1,
            d_transf: DTransformation::new(0.0, (100_039.59375, 50.0)),
        };
        let mut guard = OriginalOverlapGuard::new(&instance);
        guard
            .sync_construction_layout(&builder.prob.layout())
            .unwrap();
        assert!(!guard.construction_allows(&[candidate]).unwrap());
        let accepted = builder
            .certify_legacy_placement(candidate, &mut || false)
            .unwrap();
        assert_ne!(accepted.d_transf, candidate.d_transf);
        assert!(
            (accepted.d_transf.translation().0 - candidate.d_transf.translation().0).abs() < 0.1
        );
        builder.prob.place_item(accepted);
        assert!(crate::original_overlap::validate(&instance, &builder.prob.save(), &[]).is_ok());
        assert_eq!(
            crate::eval::lbf_evaluator::flat_layout_collision_counts(&builder.prob.layout()).1,
            0
        );
        let unchanged = builder
            .prob
            .layout()
            .placed_items()
            .values()
            .find(|p| p.item().idx() == 0)
            .unwrap();
        assert_eq!(unchanged.d_transf(), fixed.d_transf);
    }

    #[test]
    fn construction_outline_cache_follows_removed_and_moved_placements() {
        let instance = rectangles_with_overlap_allowance();
        let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
        prob.change_strip_width(300.0).expect(crate::dmsg!("strip width must be positive"));
        let pose = DTransformation::new(0.0, (50.0, 50.0));
        let key = prob.place_item(SPPlacement {
            item_idx: 0,
            d_transf: pose,
        });
        let candidate = SPPlacement {
            item_idx: 1,
            d_transf: pose,
        };
        let mut guard = OriginalOverlapGuard::new(&instance);
        guard.sync_construction_layout(&prob.layout()).unwrap();
        assert!(!guard.construction_allows(&[candidate]).unwrap());
        prob.remove_item(key);
        let moved = prob.place_item(SPPlacement {
            item_idx: 0,
            d_transf: DTransformation::new(0.0, (150.0, 50.0)),
        });
        guard.sync_construction_layout(&prob.layout()).unwrap();
        assert!(guard.construction_allows(&[candidate]).unwrap());
        prob.remove_item(moved);
        prob.place_item(SPPlacement {
            item_idx: 2,
            d_transf: pose,
        });
        guard.sync_construction_layout(&prob.layout()).unwrap();
        assert!(!guard.construction_allows(&[candidate]).unwrap());
    }

    #[test]
    fn legacy_backoff_respects_band_limits_and_cancellation() {
        let instance = rectangles_with_overlap_allowance();
        let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
        prob.change_strip_width(300.0).expect(crate::dmsg!("strip width must be positive"));
        prob.place_item(SPPlacement {
            item_idx: 0,
            d_transf: DTransformation::new(0.0, (50.0, 50.0)),
        });
        let mut builder = LBFBuilder::from_problem(
            instance.clone(),
            prob,
            Xoshiro256PlusPlus::seed_from_u64(1),
            LBF_SAMPLE_CONFIG,
            5.0,
            1.0,
        );
        builder.original_guard = Some(OriginalOverlapGuard::new(&instance));
        let candidate = SPPlacement {
            item_idx: 1,
            d_transf: DTransformation::new(0.0, (89.5995, 50.0)),
        };
        let mut shape = instance.item(1).shape_cd().as_ref().clone();
        shape.transform_from(&instance.item(1).shape_cd(), &candidate.d_transf.compose());
        builder.band_x_max = Some(shape.bbox().x_max);
        assert!(
            builder
                .certify_legacy_placement(candidate, &mut || false)
                .is_none()
        );
        builder.band_x_max = None;
        assert!(
            builder
                .certify_legacy_placement(candidate, &mut || true)
                .is_none()
        );
        assert_eq!(builder.prob.layout().placed_items().len(), 1);
        assert!(
            builder
                .certify_legacy_placement(candidate, &mut || false)
                .is_some()
        );
    }

    #[test]
    fn rigid_groups_keep_their_existing_construction_and_final_validation_path() {
        use crate::optimizer::place_groups::{PlaceGroupMember, PlaceGroupSpec};
        let instance = rectangles_with_overlap_allowance();
        // Eroded members are disjoint, but their source overlap exceeds the
        // original guard's effective tolerance. Translation cannot repair this.
        let groups = PlaceGroups::new(
            &instance,
            vec![PlaceGroupSpec {
                id: 7,
                members: vec![
                    PlaceGroupMember {
                        item_id: 0,
                        initial_transform: DTransformation::new(0.0, (30.0, 30.0)),
                    },
                    PlaceGroupMember {
                        item_id: 1,
                        initial_transform: DTransformation::new(0.0, (69.6005, 30.0)),
                    },
                ],
            }],
        )
        .unwrap();
        let builder = LBFBuilder::new(
            instance.clone(),
            Xoshiro256PlusPlus::seed_from_u64(1),
            LBF_SAMPLE_CONFIG,
            5.0,
            1.0,
        )
        .with_place_groups(groups.clone())
        .unwrap()
        .construct();
        assert!(builder.prob.item_demand_qtys().iter().all(|q| *q == 0));
        assert!(groups.solution_preserves_relative_poses(&builder.prob.save()));
        assert!(crate::original_overlap::validate(&instance, &builder.prob.save(), &[]).is_err());
        assert!(
            crate::original_overlap::validate_with_groups(
                &instance,
                &builder.prob.save(),
                &[],
                &groups
            )
            .is_ok()
        );
        assert!(
            crate::structure::pipeline::solution_validation_error_with_frozen_and_groups(
                &instance,
                &builder.prob.save(),
                &[],
                &groups
            )
            .is_none()
        );
    }

    fn hint_start(instance: &SPInstance, keep_contract: bool) -> SPProblem {
        let c = DEFAULT_SWARM_CONFIG;
        let mut builder = LBFBuilder::new(
            instance.clone(),
            Xoshiro256PlusPlus::seed_from_u64(1),
            LBF_SAMPLE_CONFIG,
            c.lbf_x_multiplier,
            c.lbf_y_multiplier,
        )
        .with_preselected_structure_hints(
            ProfilePlacementConfig {
                row_height: 3.0,
                ..ProfilePlacementConfig::default()
            },
            vec![],
        );
        if !keep_contract {
            builder.original_guard = None;
        }
        let mut never = || false;
        builder.construct_with_control(&mut never).prob
    }

    /// 구조 힌트 구성기는 조각을 접촉까지 민다. 접촉 자리는 원본 겹침 계약을 격자 여유만큼
    /// 어기므로, 자리를 고를 때 계약을 보지 않으면 후보가 통째로 기각된다.
    #[test]
    fn a_hint_start_keeps_the_original_overlap_contract_without_any_repair() {
        let instance = rectangles_with_overlap_allowance();
        assert!(OriginalOverlapGuard::is_active(&instance));

        let kept = hint_start(&instance, true);
        assert!(kept.item_demand_qtys().iter().all(|qty| *qty == 0));
        let error = solution_validation_error(&instance, &kept.save());
        assert!(error.is_none(), "{error:?}");

        // 술어를 끄면 예전 동작이다: 같은 입력에서 접촉 자리가 계약을 어긴다.
        let ignored = hint_start(&instance, false);
        assert!(ignored.item_demand_qtys().iter().all(|qty| *qty == 0));
        assert!(solution_validation_error(&instance, &ignored.save()).is_some());
        // 물러난 거리는 수 마이크로미터라 폭은 사실상 같다.
        assert!(
            (kept.strip_width() - ignored.strip_width()).abs() < 1.0,
            "{} vs {}",
            kept.strip_width(),
            ignored.strip_width()
        );
    }

    /// 조각 겹침 허용이 없는 입력에서는 술어가 없고, 물러날 자리도 만들지 않는다.
    #[test]
    fn without_an_overlap_allowance_the_hint_path_is_unchanged() {
        let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
            "name": "hint-no-allowance", "strip_height": 100.0,
            "items": [{"id": 0, "demand": 1, "allowed_orientations": [0.0],
                       "shape": {"type": "simple_polygon",
                                 "data": [[0.0, 0.0], [40.0, 0.0], [40.0, 30.0], [0.0, 30.0]]}}]
        }))
        .unwrap();
        let c = DEFAULT_SWARM_CONFIG;
        let instance = crate::util::io::import_spp_instance(
            &ext,
            c.cde_config,
            c.poly_simpl_tolerance,
            None,
            c.narrow_concavity_cutoff_ratio,
        )
        .unwrap();
        let builder = LBFBuilder::new(
            instance,
            Xoshiro256PlusPlus::seed_from_u64(1),
            LBF_SAMPLE_CONFIG,
            c.lbf_x_multiplier,
            c.lbf_y_multiplier,
        )
        .with_preselected_structure_hints(ProfilePlacementConfig::default(), vec![]);
        assert!(builder.original_guard.is_none());
        let pose = DTransformation::new(0.0, (10.0, 20.0));
        assert_eq!(builder.contract_backoff_poses(pose), vec![pose]);
    }
}

#[cfg(test)]
mod construction_settle_tests {
    use super::*;
    use crate::config::DEFAULT_SWARM_CONFIG;
    use crate::consts::LBF_SAMPLE_CONFIG;
    use crate::structure::pipeline::solution_validation_error;
    use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
    use rand::SeedableRng;

    fn builder(placed: usize) -> LBFBuilder {
        let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
            "name": "construction-settle", "strip_height": 20.0,
            "items": (0..4).map(|id| serde_json::json!({
                "id": id, "demand": 1, "allowed_orientations": [0.0],
                "shape": {"type": "rectangle", "data": {
                    "x_min": 0.0, "y_min": 0.0, "width": 4.0, "height": 4.0
                }}
            })).collect::<Vec<_>>()
        })).unwrap();
        let c = DEFAULT_SWARM_CONFIG;
        let instance = crate::util::io::import_spp_instance(
            &ext, c.cde_config, c.poly_simpl_tolerance, None,
            c.narrow_concavity_cutoff_ratio,
        ).unwrap();
        let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
        crate::util::frozen::change_strip_width(&mut prob, 150.0);
        for id in 0..placed {
            prob.place_item(SPPlacement {
                item_idx: id,
                d_transf: DTransformation::new(0.0, (25.0 * id as f32 + 10.0, 5.0)),
            });
        }
        LBFBuilder::from_problem(
            instance, prob, Xoshiro256PlusPlus::seed_from_u64(123),
            LBF_SAMPLE_CONFIG, c.lbf_x_multiplier, c.lbf_y_multiplier,
        )
    }

    fn settle_config() -> SettleDuringConstruction {
        let c = DEFAULT_SWARM_CONFIG;
        SettleDuringConstruction {
            every: 1, budget_ms: 300, lp: c.polish_cfg.lp,
            hole_fill: c.polish_cfg.hole_fill, separator: c.expl_cfg.separator_config,
        }
    }

    #[test]
    fn settle_skips_incomplete_demand_before_polish_and_preserves_rng_sequence() {
        let mut builder = builder(2);
        let before = builder.prob.save();
        let mut expected_rng = builder.rng.clone();
        let _: u64 = expected_rng.random_range(0..u64::MAX);
        let mut polls = 0;
        builder.settle_partial_layout(settle_config(), &mut || {
            polls += 1;
            false
        });
        assert_eq!(polls, 1, "incomplete demand must return before the post-hole/LP poll");
        assert_eq!(builder.prob.item_demand_qtys(), vec![0, 0, 1, 1]);
        assert_eq!(builder.prob.save().strip(), before.strip());
        for (key, pi) in before.layout_snapshot().placed_items() {
            assert_eq!(builder.prob.layout().placed_items()[key].d_transf(), pi.d_transf());
        }
        assert_eq!(builder.rng.random::<u64>(), expected_rng.random::<u64>());
    }

    #[test]
    fn completed_movable_settle_still_compacts_and_certifies() {
        let mut builder = builder(4);
        let before = builder.prob.strip_width();
        builder.settle_partial_layout(settle_config(), &mut || false);
        assert!(builder.prob.strip_width() < before);
        assert!(solution_validation_error(&builder.instance, &builder.prob.save()).is_none());
    }

    #[test]
    fn completed_settle_keeps_rigid_members_relative_poses() {
        use crate::optimizer::place_groups::{PlaceGroupMember, PlaceGroupSpec};
        let mut builder = builder(4);
        let groups = PlaceGroups::new(
            &builder.instance,
            vec![PlaceGroupSpec {
                id: 7,
                members: (0..2)
                    .map(|id| PlaceGroupMember {
                        item_id: id,
                        initial_transform: DTransformation::new(
                            0.0,
                            (25.0 * id as f32 + 10.0, 5.0),
                        ),
                    })
                    .collect(),
            }],
        )
        .unwrap();
        builder = builder.with_place_groups(groups.clone()).unwrap();
        builder.settle_partial_layout(settle_config(), &mut || false);
        assert!(groups.solution_preserves_relative_poses(&builder.prob.save()));
        assert!(solution_validation_error(&builder.instance, &builder.prob.save()).is_none());
    }

    #[test]
    fn settle_cancelled_after_hole_pass_keeps_the_incumbent() {
        let mut builder = builder(4);
        let before = builder.prob.save();
        let mut polls = 0;
        builder.settle_partial_layout(settle_config(), &mut || {
            polls += 1;
            polls >= 2
        });
        assert_eq!(polls, 2);
        assert_eq!(builder.prob.strip_width(), before.strip().width);
        for (key, pi) in before.layout_snapshot().placed_items() {
            assert_eq!(builder.prob.layout().placed_items()[key].d_transf(), pi.d_transf());
        }
    }

    #[test]
    #[ignore = "broadcast Init cancellation; run issue2_ --include-ignored --test-threads=1"]
    fn issue2_settle_init_cancel_before_lp_keeps_incumbent() {
        let mut builder = builder(4);
        let before = builder.prob.save();
        let _scope = crate::config::runtime::begin_init();
        let mut polls = 0;
        builder.settle_partial_layout(settle_config(), &mut || {
            polls += 1;
            if polls == 2 { crate::config::runtime::request_init_cancel(); }
            false
        });
        assert_eq!(polls, 2);
        assert_eq!(builder.prob.strip_width(), before.strip_width(),
            "an Init cancellation at the post-hole boundary must reach LP and prevent adoption");
        for (key, item) in before.layout_snapshot().placed_items() {
            assert_eq!(builder.prob.layout().placed_items()[key].d_transf(), item.d_transf());
        }
        drop(_scope); // Re-certify the retained incumbent after leaving cancelled Init.
        assert!(solution_validation_error(&builder.instance, &builder.prob.save()).is_none());
    }
    #[test]
    fn cancelled_settle_leaves_layout_and_rng_unchanged() {
        let mut builder = builder(4);
        let before = builder.prob.save();
        let mut expected_rng = builder.rng.clone();
        builder.settle_partial_layout(settle_config(), &mut || true);
        assert_eq!(builder.prob.strip_width(), before.strip().width);
        for (key, pi) in before.layout_snapshot().placed_items() {
            assert_eq!(builder.prob.layout().placed_items()[key].d_transf(), pi.d_transf());
        }
        assert_eq!(builder.rng.random::<u64>(), expected_rng.random::<u64>());
    }
}
