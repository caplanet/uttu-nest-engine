use crate::eval::group_sep_evaluator::GroupSeparationEvaluator;
use crate::eval::sep_evaluator::{PooledSeparationEvaluator, SeparationEvaluatorPool};
use crate::optimizer::place_groups::PlaceGroups;
use crate::quantify::tracker::CollisionTracker;
use crate::sample::search;
use crate::sample::search::SampleConfig;
use crate::util::assertions::tracker_matches_layout;
use crate::util::terminator::Terminator;
use itertools::Itertools;
use jagua_rs::entities::{PItemKey};
use jagua_rs::geometry::DTransformation;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution};
use rand::prelude::SliceRandom;
use rand::rngs::Xoshiro256PlusPlus;
use tap::Tap;

pub struct SeparatorWorker {
    pub(crate) prepared: std::sync::Arc<crate::prepared_geometry::PreparedGeometry>,
    pub(crate) evaluator_pool: SeparationEvaluatorPool,
    pub instance: SPInstance,
    pub prob: SPProblem,
    pub ct: CollisionTracker,
    pub rng: Xoshiro256PlusPlus,
    pub sample_config: SampleConfig,
    /// 고정할 item_id 비트마스크. true면 separation 중 이동하지 않는다.
    pub frozen_mask: Vec<bool>,
    /// Rigid placement-unit lookup copied from the owning Separator.
    pub place_groups: PlaceGroups,
    /// ① NFP 샘플러용 워커 전용 캐시. `None` 이면 기존 난수 샘플러만 쓴다.
    ///
    /// `Separator::new` 가 준비본을 한 번 만들고 `fork()` 로 워커마다 나눠 준다.
    /// 워커별 사본이라 병렬 separation 에서 `&mut` 경합이 없다.
    pub nfp: Option<crate::nfp_lbf::cache::NfpLbfCache>,
}

/// ① NFP 샘플러를 먼저 시도하고, 후보가 없으면 기존 난수 샘플러로 폴백한다.
///
/// 폴백은 선택이 아니라 **계약**이다 — `search_placement_nfp` 는 겹친 상태에서 후보를
/// 0개 낼 수 있고(그 모듈의 "겹친 상태에서의 계약"), 그때 호출자가 되돌아가지 않으면
/// 조각이 영영 안 움직인다.
///
/// `nfp` 가 `None`(기본값)이면 기존 경로를 그대로 부른다 — **한 비트도 바뀌지 않는다.**
/// 그것이 A/B 측정의 전제다.
///
/// evaluator 를 값이 아니라 **팩토리**로 받는 이유: `SeparationEvaluator` 와
/// `GroupSeparationEvaluator` 는 `Clone` 이 아니고, 두 경로가 각각 하나씩 소비한다.
/// 팩토리는 필요할 때만 만들므로 폴백이 없을 때 낭비도 없다.
fn search_placement_maybe_nfp<E: crate::eval::sample_eval::SampleEvaluator>(
    layout: &jagua_rs::entities::Layout,
    nfp: &mut Option<crate::nfp_lbf::cache::NfpLbfCache>,
    item: &jagua_rs::entities::Item,
    pk: Option<PItemKey>,
    mut make_evaluator: impl FnMut() -> E,
    sample_config: SampleConfig,
    rng: &mut Xoshiro256PlusPlus,
    term: &impl Terminator,
    prepared: &crate::prepared_geometry::PreparedGeometry,
) -> Option<(DTransformation, crate::eval::sample_eval::SampleEval)> {
    if term.kill() {
        return None;
    }
    if let Some(cache) = nfp.as_mut() {
        let (best, _) = crate::sample::nfp_sampler::search_placement_nfp_with_control(
            layout,
            item,
            pk,
            make_evaluator(),
            sample_config,
            cache,
            rng,
            &mut || term.kill(),
        );
        // NFP 샘플러는 현재 자세를 기준으로 평가하므로, 어떤 후보도 나아지지 않으면
        // 현재 자세를 그대로 돌려준다. 그때는 무작위 샘플러에 기회를 넘긴다 —
        // 그쪽도 현재 자세에서 출발하므로 결과가 나빠지지 않는다.
        let unchanged = matches!(
            (best, pk.and_then(|pk| layout.placed_items().get(pk))),
            (Some((dt, _)), Some(pi)) if dt == pi.d_transf()
        );
        if (best.is_some() && !unchanged) || term.kill() {
            return best;
        }
    }
    search::search_placement_prepared_with_control(
        layout,
        item,
        pk,
        make_evaluator(),
        sample_config,
        rng,
        &mut || term.kill(),
        prepared,
    )
    .0
}

impl SeparatorWorker {
    pub fn load(&mut self, sol: &SPSolution, ct: &CollisionTracker) {
        crate::util::frozen::restore(&mut self.prob, sol);
        // clone() 대신 버퍼 재사용 복사 — 매 iteration마다 O(n²) 재할당을 피한다
        self.ct.copy_from(ct);
    }

    /// Algorithm 5 from https://doi.org/10.48550/arXiv.2509.13329
    pub fn move_items(&mut self, term: &impl Terminator) {
        if self.place_groups.is_empty() {
            self.move_items_legacy(term);
            return;
        }
        let mask = &self.frozen_mask;
        let candidates = self
            .place_groups
            .placed_unit_representatives(&self.prob.layout())
            .into_iter()
            .filter(|&item_id| item_id >= mask.len() || !mask[item_id])
            .filter(|&item_id| self.unit_loss(item_id) > 0.0)
            .collect_vec()
            .tap_mut(|v| v.shuffle(&mut self.rng));

        for &unit_id in candidates.iter() {
            if term.kill() {
                break;
            }
            // Always sample through the group's representative. Its allowed
            // rotations are the precomputed group-wide intersection; sampling
            // through an arbitrary colliding member could violate another
            // member's orientation constraint. The group evaluator still sums
            // collision loss over every member.
            let item_id = unit_id;
            let Some(pk) = self.placement_key(item_id) else {
                continue;
            };
            if self.unit_loss(unit_id) > 0.0 {
                let item = self.instance.item(item_id);

                // 필드를 하나씩 나눠 빌린다. `&mut self.nfp` 와 `&self.prob.layout` 는
                // 서로 다른 필드라 대여가 겹치지 않는다 — 메서드로 감싸면 `self` 전체를
                // 빌리게 되어 컴파일되지 않는다.
                let layout = &self.prob.layout();
                let ct = &self.ct;
                let instance = &self.instance;
                let place_groups = &self.place_groups;
                let sample_config = self.sample_config;
                let evaluator_pool = &self.evaluator_pool;
                let best_sample = if place_groups.group_id_of(item_id).is_some() {
                    search_placement_maybe_nfp(
                        layout,
                        &mut self.nfp,
                        item,
                        Some(pk),
                        || {
                            GroupSeparationEvaluator::new(
                                layout,
                                instance,
                                place_groups,
                                item_id,
                                ct,
                            )
                        },
                        sample_config,
                        &mut self.rng,
                        term,
                        &self.prepared,
                    )
                } else {
                    search_placement_maybe_nfp(
                        layout,
                        &mut self.nfp,
                        item,
                        Some(pk),
                        || PooledSeparationEvaluator::new(layout, item, pk, ct, evaluator_pool),
                        sample_config,
                        &mut self.rng,
                        term,
                        &self.prepared,
                    )
                };

                if term.kill() {
                    break;
                }
                let Some((new_dt, _eval)) = best_sample else {
                    continue;
                };

                self.move_item(pk, new_dt);
            }
        }
    }

    fn move_items_legacy(&mut self, term: &impl Terminator) {
        let mask = &self.frozen_mask;
        let candidates = self
            .prob
            .layout()
            .placed_items()
            .keys()
            .filter(|pk| {
                let item_id = self.prob.layout().placed_items()[*pk].item().idx();
                item_id >= mask.len() || !mask[item_id]
            })
            .filter(|pk| self.ct.get_loss(*pk) > 0.0)
            .collect_vec()
            .tap_mut(|v| v.shuffle(&mut self.rng));

        for pk in candidates {
            if term.kill() {
                break;
            }
            if self.ct.get_loss(pk) == 0.0 {
                continue;
            }
            let item_id = self.prob.layout().placed_items()[pk].item().idx();
            let item = self.instance.item(item_id);
            let layout = &self.prob.layout();
            let ct = &self.ct;
            let sample_config = self.sample_config;
            let evaluator_pool = &self.evaluator_pool;
            let best_sample = search_placement_maybe_nfp(
                layout,
                &mut self.nfp,
                item,
                Some(pk),
                || PooledSeparationEvaluator::new(layout, item, pk, ct, evaluator_pool),
                sample_config,
                &mut self.rng,
                term,
                &self.prepared,
            );
            if term.kill() {
                break;
            }
            let Some((new_dt, _)) = best_sample else {
                continue;
            };
            self.move_item(pk, new_dt);
        }
    }

    fn placement_key(&self, item_id: usize) -> Option<PItemKey> {
        self.place_groups.placement_key(&self.prob.layout(), item_id)
    }

    fn unit_loss(&self, item_id: usize) -> f32 {
        self.place_groups
            .member_item_ids(item_id)
            .into_iter()
            .filter_map(|id| self.placement_key(id))
            .map(|pk| self.ct.get_loss(pk))
            .sum()
    }

    pub fn move_item(&mut self, pk: PItemKey, d_transf: DTransformation) -> PItemKey {
        let item_id = self.prob.layout().placed_items()[pk].item().idx();
        if item_id < self.frozen_mask.len() && self.frozen_mask[item_id] {
            return pk;
        }

        debug_assert!(tracker_matches_layout(&self.ct, &self.prob.layout()));

        if self.place_groups.is_empty() {
            self.prob.remove_item(pk);
            let new_pk = self.prob.place_item(SPPlacement { d_transf, item_idx: item_id });
            self.ct.register_item_move(&self.prob.layout(), pk, new_pk);
            debug_assert!(tracker_matches_layout(&self.ct, &self.prob.layout()));
            return new_pk;
        }

        let targets = self
            .place_groups
            .target_transforms(Some(&self.prob.layout()), item_id, d_transf)
            .expect(crate::dmsg!("validated place group became incomplete"));
        let moves = targets
            .into_iter()
            .map(|(id, target)| {
                let old_pk = self
                    .placement_key(id)
                    .expect(crate::dmsg!("validated place group member disappeared"));
                (old_pk, id, target)
            })
            .collect_vec();

        let mut selected_new_pk = pk;
        for (old_pk, moved_item_id, target) in moves {
            self.prob.remove_item(old_pk);
            let new_pk = self.prob.place_item(SPPlacement {
                d_transf: target,
                item_idx: moved_item_id,
            });
            self.ct
                .register_item_move(&self.prob.layout(), old_pk, new_pk);
            if moved_item_id == item_id {
                selected_new_pk = new_pk;
            }
        }

        debug_assert!(tracker_matches_layout(&self.ct, &self.prob.layout()));

        selected_new_pk
    }
}
