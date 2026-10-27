use crate::config::{CompressionConfig, ShrinkDecayStrategy, normalize_shrink_range};
use crate::optimizer::separator::Separator;
use crate::util::listener::{ReportType, SolutionListener};
use crate::util::terminator::Terminator;
use jagua_rs::Instant;
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use rand::RngExt;

/// Algorithm 13 from https://doi.org/10.48550/arXiv.2509.13329
pub fn compression_phase(
    instance: &SPInstance,
    sep: &mut Separator,
    init_sol: &SPSolution,
    sol_listener: &mut Option<&mut dyn SolutionListener>,
    term: &impl Terminator,
    config: &CompressionConfig,
) -> SPSolution {
    let corrected_initial = if crate::original_overlap::OriginalOverlapGuard::is_active(instance) {
        sep.rollback(init_sol, None);
        sep.repair_original_overlap(term).then(|| sep.prob.save())
    } else {
        None
    };
    let init_sol = corrected_initial.as_ref().unwrap_or(init_sol);
    let mut best_sol = init_sol.clone();
    if sep
        .min_feasible_width()
        .is_some_and(|width| width >= init_sol.strip_width())
    {
        return best_sol;
    }
    // The time-based decay starts here. Anchoring it at `deadline - time_limit`
    // was tried, but callers that pass a shorter slice terminator (grouped
    // engine quanta) would then start near the end of the schedule.
    let start = Instant::now();
    let mut n_failed_attempts = 0;
    let Some((shrink_start, shrink_stop)) = normalize_shrink_range(config.shrink_range) else {
        return best_sol;
    };

    // 시작 걸음이 과도하면 시간 기반 감쇠를 실패 기반으로 바꾼다 (`OVERSIZED_START`).
    let shrink_decay = effective_shrink_decay(config.shrink_decay, shrink_start);

    // Create the function to calculate the shrink step size.
    let shrink_step_size = |n_failed_attempts: i32| -> f32 {
        match shrink_decay {
            ShrinkDecayStrategy::TimeBased => {
                let range = shrink_stop - shrink_start;
                let elapsed = start.elapsed();
                let ratio = elapsed.as_secs_f32() / config.time_limit.as_secs_f32();
                shrink_start + ratio * range
            }
            ShrinkDecayStrategy::FailureBased(r) => shrink_start * r.powi(n_failed_attempts),
        }
    };

    // As long as the shrink step size is above the stop threshold, keep attempting to compress.
    while !term.kill()
        && let step = shrink_step_size(n_failed_attempts)
        && step >= shrink_stop
    {
        // 과도한 걸음의 시도는 거의 전부 실패하고, 실패하는 분리는 오래 돈다 — 짧게 끊는다.
        let cap = if step > OVERSIZED_START {
            Some(attempt_cap().map_or(OVERSIZED_ATTEMPT_CAP, |c| c.min(OVERSIZED_ATTEMPT_CAP)))
        } else {
            attempt_cap()
        };
        let attempt_term = AttemptTerminator {
            inner: term,
            until: cap.map(|cap| Instant::now() + cap),
        };
        match attempt_to_compress(sep, &best_sol, step, &attempt_term, sol_listener) {
            Some(compacted_sol) => {
                if let Some(l) = sol_listener.as_deref_mut() {
                    l.report(ReportType::CmprFeas, &compacted_sol, instance);
                }
                best_sol = compacted_sol;
                // AIMD-style recovery for FailureBased: one successful repair
                // earns back one shrink level. Repeated failures still reduce
                // the step geometrically, while success does not leave the
                // compressor permanently stuck at the tiniest step.
                n_failed_attempts = (n_failed_attempts - 1).max(0);
            }
            None => {
                n_failed_attempts += 1;
            }
        }
    }
    sep.rollback(&best_sol, None);
    best_sol
}

/// 압축의 시작 걸음이 이 비율을 넘으면 **과도하다**고 본다(네이티브 기본 0.05%, WPF 기본 0.1%).
///
/// 시간 기반 감쇠는 걸음을 시작값에서 종료값까지 **시간에 비례해** 줄인다. 시작값이 10% 면
/// 예산의 95% 가 지날 때까지 걸음이 0.5% 를 넘는다 — 조밀한 배치에서는 전부 실패하는 걸음이라
/// 단계 예산 거의 전부가 실패한 시도에 쓰인다. 그런 설정에서는 감쇠를 실패 기반으로 바꾼다:
/// 실패하면 걸음을 [`OVERSIZED_DECAY`] 배로 줄이고 성공하면 한 단계 되돌린다(기존
/// `FailureBased` 와 같은 규칙). 이 문턱 이하의 설정은 한 비트도 달라지지 않는다.
const OVERSIZED_START: f32 = 0.002;

/// 과도한 시작 걸음에 쓰는 실패 기반 감쇠 비율.
const OVERSIZED_DECAY: f32 = 0.5;

/// 과도한 걸음([`OVERSIZED_START`] 초과)의 시도 하나가 쓸 수 있는 시간.
const OVERSIZED_ATTEMPT_CAP: std::time::Duration = std::time::Duration::from_secs(3);

/// 설정된 감쇠 전략을 시작 걸음에 맞춰 고른다 — [`OVERSIZED_START`] 참조.
#[cfg_attr(not(feature = "cpguard-policy-off"), cpguard::obfuscate)]
fn effective_shrink_decay(configured: ShrinkDecayStrategy, shrink_start: f32) -> ShrinkDecayStrategy {
    match configured {
        ShrinkDecayStrategy::TimeBased if shrink_start > OVERSIZED_START => {
            ShrinkDecayStrategy::FailureBased(OVERSIZED_DECAY)
        }
        other => other,
    }
}

/// 시도 하나의 시간 상한 — **테스트 빌드 전용 실험 스위치**. 생산 빌드는 항상 상한이 없다.
///
/// 2026-10-01 측정(`compress_attempt_cap_ab_on_a_saved_layout`, 압축 60초 + 폴리시, 시드 2개):
/// 여성WSjk 156 은 상한 없음 6253.1 / 10초 6250.3 / 5초 6247.0mm, babel 겉감은
/// 12350.5 / 12357.5 / 12351.8mm. 시드 간 편차(±10mm)보다 작아 **차이 없음** — 기본 동작에
/// 넣지 않았다.
#[cfg(test)]
pub(crate) static ATTEMPT_CAP_MS: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(0);

#[cfg(test)]
fn attempt_cap() -> Option<std::time::Duration> {
    match ATTEMPT_CAP_MS.load(std::sync::atomic::Ordering::Relaxed) {
        0 => None,
        ms => Some(std::time::Duration::from_millis(ms)),
    }
}

#[cfg(not(test))]
fn attempt_cap() -> Option<std::time::Duration> {
    None
}

/// 단계 터미네이터에 시도별 마감을 얹는다. 바깥의 중단·마감은 그대로 전파된다.
struct AttemptTerminator<'a, T: Terminator + ?Sized> {
    inner: &'a T,
    until: Option<Instant>,
}

impl<T: Terminator + ?Sized> Terminator for AttemptTerminator<'_, T> {
    fn kill(&self) -> bool {
        self.inner.kill() || self.until.is_some_and(|until| Instant::now() >= until)
    }
    fn stop_requested(&self) -> bool {
        self.inner.stop_requested()
    }
    fn new_timeout(&mut self, timeout: std::time::Duration) {
        self.until = Some(Instant::now() + timeout);
    }
    fn timeout_at(&self) -> Option<Instant> {
        match (self.until, self.inner.timeout_at()) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }
}

pub(crate) fn attempt_to_compress(
    sep: &mut Separator,
    init_sol: &SPSolution,
    r_shrink: f32,
    term: &impl Terminator,
    sol_listener: &mut Option<&mut dyn SolutionListener>,
) -> Option<SPSolution> {
    // Restore to the initial solution and width
    sep.rollback(init_sol, None);

    // Shrink the container by the provided amount at a random position
    let mut new_width = init_sol.strip_width() * (1.0 - r_shrink);
    // 고정 아이템이 경계를 침범하지 않도록 축소 폭의 하한을 clamp
    if let Some(min_w) = sep.min_feasible_width()
        && new_width < min_w
    {
        new_width = min_w;
    }
    if new_width >= init_sol.strip_width() {
        // 고정 아이템이 이미 strip 우측 경계에 붙어 있어 압축이 불가능
        return None;
    }
    let random_split = sep.rng.random_range(0.0..sep.prob.strip_width());
    // Keep random proposals for diversity; evaluate three additional cheap
    // frontier/gap proposals without changing the public Config contract.
    let split_pos = if sep.rng.random::<bool>() {
        random_split
    } else {
        choose_split(sep, random_split, init_sol.strip_width() - new_width)
    };
    if !sep.change_strip_width_with_control(new_width, Some(split_pos), &mut || term.kill()) {
        return None;
    }

    // Try to separate layout, if all collisions are eliminated, return the solution
    let (compacted_sol, ot) = sep.separate(term, sol_listener);
    match ot.get_total_loss() == 0.0 && sep.solution_is_certified(&compacted_sol, init_sol) {
        true => Some(compacted_sol),
        false => None,
    }
}

#[cfg(test)]
std::thread_local! {
    static ISSUE7_SPLIT_WORK: std::cell::Cell<[usize; 2]> =
        const { std::cell::Cell::new([0; 2]) };
}

#[cfg(test)]
fn issue7_record_split_work(index: usize) {
    ISSUE7_SPLIT_WORK.with(|cell| {
        let mut counts = cell.get();
        counts[index] += 1;
        cell.set(counts);
    });
}

fn choose_split(sep: &Separator, random: f32, shrink: f32) -> f32 {
    let width = sep.prob.strip_width();
    let mut proposals = vec![random];
    let mut frontier: Vec<_> = sep
        .prob
        .layout()
        .placed_items()
        .values()
        .filter(|p| sep.can_move_individually(p.item().idx()))
        .collect();
    frontier.sort_by(|a, b| b.shape().bbox().x_max.total_cmp(&a.shape().bbox().x_max));
    proposals.extend(
        frontier
            .iter()
            .take(3)
            .map(|p| (p.shape().bbox().x_min - shrink).clamp(0., width)),
    );
    // Iterator::min_by does no comparisons for a singleton. Keep that
    // zero-score path for layouts with no individually movable frontier.
    if proposals.len() == 1 {
        return random;
    }
    let score = |x: f32| {
        #[cfg(test)]
        issue7_record_split_work(0);
        sep.prob
            .layout()
            .placed_items()
            .values()
            .map(|p| {
                #[cfg(test)]
                issue7_record_split_work(1);
                let r = p.shape().bbox();
                if r.x_min <= x + shrink && r.x_max >= x {
                    // Amount of vertical material crossing the proposed compression seam.
                    r.height() as f64
                } else {
                    0.0
                }
            })
            .sum::<f64>()
    };
    proposals
        .into_iter()
        .map(|x| (score(x), x))
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, x)| x)
        .unwrap_or(random)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::DEFAULT_SWARM_CONFIG;
    use crate::util::terminator::BasicTerminator;
    use std::time::Duration;

    fn issue7_split_fixture(boxes: &[(f32, f32, f32)]) -> Separator {
        use jagua_rs::geometry::DTransformation;
        use jagua_rs::probs::spp::entities::{SPPlacement, SPProblem};
        use rand::{SeedableRng, rngs::Xoshiro256PlusPlus};
        let cfg = DEFAULT_SWARM_CONFIG;
        let items: Vec<_> = boxes.iter().enumerate().map(|(id, &(xmin, xmax, height))| {
            serde_json::json!({"id":id,"demand":1,"allowed_orientations":[0.0],
                "shape":{"type":"rectangle","data":{"x_min":0.0,"y_min":0.0,
                    "width":xmax-xmin,"height":height}}})
        }).collect();
        let ext = serde_json::from_value(serde_json::json!({
            "name":"issue7-splits","strip_height":40.0,"items":items
        })).unwrap();
        let instance = crate::util::io::import_spp_instance(&ext, cfg.cde_config, None, None, None).unwrap();
        let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
        prob.change_strip_width(10000.0).expect(crate::dmsg!("strip width must be positive"));
        for (id, &(xmin, xmax, height)) in boxes.iter().enumerate() {
            let initial = prob.place_item(SPPlacement { item_idx:id,
                d_transf:DTransformation::new(0.0,(0.0,20.0)) });
            let dx = xmin - prob.layout().placed_items()[initial].shape().bbox().x_min;
            prob.remove_item(initial);
            let placed = prob.place_item(SPPlacement { item_idx:id,
                d_transf:DTransformation::new(0.0,(dx,20.0)) });
            let bbox = prob.layout().placed_items()[placed].shape().bbox();
            assert_eq!(bbox.x_min, xmin);
            assert_eq!(bbox.x_max, xmax);
            assert_eq!(bbox.height(), height);
        }
        let mut sep_cfg = cfg.cmpr_cfg.separator_config;
        sep_cfg.n_workers = 1;
        sep_cfg.nfp_sampler = false;
        Separator::new(instance, prob, Xoshiro256PlusPlus::seed_from_u64(41), sep_cfg)
    }

    // Frozen predecessor choose_split oracle, independent of optimized scoring.
    fn issue7_reference_split(sep: &Separator, random: f32, shrink: f32) -> f32 {
        let width = sep.prob.strip_width();
        let mut proposals = vec![random];
        let mut frontier: Vec<_> = sep.prob.layout().placed_items().values()
            .filter(|p| sep.can_move_individually(p.item().idx())).collect();
        frontier.sort_by(|a, b| b.shape().bbox().x_max.total_cmp(&a.shape().bbox().x_max));
        proposals.extend(frontier.iter().take(3)
            .map(|p| (p.shape().bbox().x_min - shrink).clamp(0., width)));
        proposals.into_iter().min_by(|a,b| {
            let score = |x:f32| sep.prob.layout().placed_items().values().map(|p| {
                let r=p.shape().bbox();
                if r.x_min <= x+shrink && r.x_max >= x { r.height() as f64 } else {0.0}
            }).sum::<f64>();
            score(*a).total_cmp(&score(*b))
        }).unwrap_or(random)
    }

    #[test]
    fn issue7_split_scores_each_proposal_once_with_identical_winner_and_rng() {
        use rand::RngExt;
        // Debug-assertion builds rebuild the whole quadtree on every jagua
        // place/remove, so the 800-piece fixture alone takes minutes there;
        // 200 pieces already show the linear work bound.
        let sizes: &[(usize, usize)] = if cfg!(debug_assertions) {
            &[(1,2),(2,3),(3,4),(10,4),(200,4)]
        } else {
            &[(1,2),(2,3),(3,4),(10,4),(200,4),(800,4)]
        };
        for &(n, expected_scores) in sizes {
            let boxes: Vec<_> = (0..n).map(|i| (10.0+i as f32*12.0,20.0+i as f32*12.0,5.0)).collect();
            let sep=issue7_split_fixture(&boxes);
            let expected=issue7_reference_split(&sep,0.25,1.0);
            let rng_before=sep.rng.clone().random::<[u64;4]>();
            ISSUE7_SPLIT_WORK.with(|c| c.set([0;2]));
            let actual=choose_split(&sep,0.25,1.0);
            let work=ISSUE7_SPLIT_WORK.with(|c| c.get());
            assert_eq!(actual.to_bits(),expected.to_bits());
            assert_eq!(actual,0.25,"random seam is in an empty gap");
            assert_eq!(sep.rng.clone().random::<[u64;4]>(),rng_before);
            assert_eq!(work,[expected_scores,n*expected_scores],"n={n}");
            println!("ISSUE7_SPLIT n={n} scores={} bbox_visits={}",work[0],work[1]);
        }
    }

    #[test]
    fn issue7_split_keeps_first_ties_stable_frontier_duplicates_and_immovable_score() {
        let cases: &[(&[(f32,f32,f32)],f32,f32,f32)] = &[
            (&[(10.0,20.0,5.0),(40.0,50.0,5.0),(70.0,80.0,5.0)],15.0,1.0,15.0),
            // Equal xmax keeps SlotMap order: first frontier's xmin20 -> seam19.
            (&[(20.0,80.0,5.0),(40.0,80.0,5.0),(60.0,80.0,5.0)],70.0,1.0,19.0),
            (&[(0.0,10.0,5.0),(0.0,20.0,5.0),(0.0,30.0,5.0)],0.0,2.0,0.0),
        ];
        for &(boxes,random,shrink,want) in cases {
            let sep=issue7_split_fixture(boxes);
            assert_eq!(choose_split(&sep,random,shrink),want);
            assert_eq!(choose_split(&sep,random,shrink).to_bits(),issue7_reference_split(&sep,random,shrink).to_bits());
        }
        let mut sep=issue7_split_fixture(&[(10.0,20.0,5.0),(40.0,50.0,5.0),
            (70.0,80.0,5.0),(89.0,95.0,30.0)]);
        sep.freeze_items(&[3]);
        assert_eq!(choose_split(&sep,90.0,1.0),69.0,
            "frozen material is excluded from frontier but included in every score");
        assert_eq!(choose_split(&sep,90.0,1.0).to_bits(),issue7_reference_split(&sep,90.0,1.0).to_bits());
    }

    #[test]
    fn issue7_public_empty_setters_clear_groups_reset_weights_and_keep_additive_locks() {
        use crate::optimizer::place_groups::{PlaceGroupMember,PlaceGroupSpec,PlaceGroups};
        use jagua_rs::geometry::DTransformation;
        let mut sep=issue7_split_fixture(&[(10.0,20.0,5.0),(15.0,25.0,5.0)]);
        let keys:Vec<_>=sep.prob.layout().placed_items().keys().collect();
        let group=PlaceGroups::new(&sep.instance,vec![PlaceGroupSpec {
            id:42, members:sep.prob.layout().placed_items().values().map(|p|PlaceGroupMember {
                item_id:p.item().idx(),initial_transform:p.d_transf(),
            }).collect(),
        }]).unwrap();
        assert!(sep.ct.get_total_loss()>0.0,"fixture must really overlap");
        sep.set_place_groups(group.clone()).unwrap();
        assert_eq!(sep.ct.get_total_loss(),0.0,"same-group internal overlap is allowed");
        let idx=sep.ct.pk_idx_map[keys[0]];
        sep.ct.container_collisions_mut()[idx].weight=3.25;
        sep.set_place_groups(PlaceGroups::default()).unwrap();
        assert!(sep.place_groups.is_empty());
        assert!(sep.workers.iter().all(|w|w.place_groups.is_empty() && w.ct.place_groups.is_empty()));
        assert!(sep.ct.get_total_loss()>0.0,"clearing group must restore member-member collision");
        assert_eq!(sep.ct.get_container_weight(keys[0]),1.0,"public setter resets inherited weights");
        // Even already-empty public group setting retains its weight-reset effect.
        let idx=sep.ct.pk_idx_map[keys[0]];
        sep.ct.container_collisions_mut()[idx].weight=3.25;
        sep.set_place_groups(PlaceGroups::default()).unwrap();
        assert_eq!(sep.ct.get_container_weight(keys[0]),1.0);
        sep.set_place_groups(group).unwrap();
        sep.freeze_items(&[1]);
        assert!(sep.frozen_mask[0] && sep.frozen_mask[1]);
        let frozen=sep.prob.save();
        let idx=sep.ct.pk_idx_map[keys[0]];
        sep.ct.container_collisions_mut()[idx].weight=4.5;
        sep.freeze_items(&[]);
        assert_eq!(sep.ct.get_container_weight(keys[0]),1.0);
        assert!(sep.frozen_mask[0] && sep.frozen_mask[1],"[] adds no locks and clears none");
        sep.set_place_groups(PlaceGroups::default()).unwrap();
        assert!(sep.frozen_mask[0] && sep.frozen_mask[1],"group-policy clear is not an unlock");
        let key=sep.placement_key(0).unwrap();
        sep.move_item(key,DTransformation::new(0.0,(50.0,20.0)));
        assert!(crate::util::assertions::frozen_poses_preserved(&frozen,&sep.prob.save(),&sep.frozen_mask));
        assert!(crate::util::assertions::tracker_matches_layout(&sep.ct,&sep.prob.layout()));
    }

    #[test]
    fn issue7_single_random_proposal_scores_nothing_for_frozen_and_rigid_layouts() {
        use crate::optimizer::place_groups::{PlaceGroupMember,PlaceGroupSpec,PlaceGroups};
        let boxes: Vec<_>=(0..200).map(|i|(10.0+i as f32*12.0,20.0+i as f32*12.0,5.0)).collect();
        for rigid in [false,true] {
            let mut sep=issue7_split_fixture(&boxes);
            if rigid {
                let specs: Vec<_>=(0..100).map(|group|PlaceGroupSpec {
                    id:group as u64,
                    members:(group*2..group*2+2).map(|id|PlaceGroupMember {
                        item_id:id,
                        initial_transform:sep.prob.layout().placed_items().values().find(|p|p.item().idx()==id).unwrap().d_transf(),
                    }).collect(),
                }).collect();
                sep.set_place_groups(PlaceGroups::new(&sep.instance,specs).unwrap()).unwrap();
            } else {
                sep.freeze_items(&(0..200).collect::<Vec<_>>());
            }
            ISSUE7_SPLIT_WORK.with(|c|c.set([0;2]));
            assert_eq!(choose_split(&sep,15.0,1.0),15.0);
            assert_eq!(ISSUE7_SPLIT_WORK.with(|c|c.get()),[0,0]);
        }
    }

    /// 시작 걸음이 과도할 때만 시간 기반 감쇠가 실패 기반으로 바뀐다. 기본값들(네이티브 0.05%,
    /// WPF 0.1%)과 호출자가 고른 실패 기반 설정은 그대로다.
    #[test]
    fn only_an_oversized_time_based_start_switches_to_failure_based_decay() {
        use ShrinkDecayStrategy::{FailureBased, TimeBased};
        assert!(matches!(effective_shrink_decay(TimeBased, 0.0005), TimeBased));
        assert!(matches!(effective_shrink_decay(TimeBased, 0.001), TimeBased));
        assert!(matches!(effective_shrink_decay(TimeBased, OVERSIZED_START), TimeBased));
        assert!(matches!(
            effective_shrink_decay(TimeBased, 0.1),
            FailureBased(r) if r == OVERSIZED_DECAY
        ));
        assert!(matches!(
            effective_shrink_decay(FailureBased(0.9), 0.1),
            FailureBased(r) if r == 0.9
        ));
    }

    /// **작은 조각을 빈 포켓에 «겹친 채» 옮겨 넣은 뒤 분리가 풀어 주는가, 그리고 길이가 줄어드는가.**
    ///
    /// 옮길 목록은 밖에서 만든다(스크래치패드 `spot_fit.py`: 빈 공간의 가장 넓은 자리마다 가장 잘
    /// 맞는 작은 조각과 이동량). 여기서는 목록을 하나씩 적용한다 — 조각을 옮기고, 폭을 그대로 둔
    /// 채 분리를 `AB_TUCK_CAP_MS` 동안 돌려 충돌이 0 이 되고 인증을 통과하면 채택, 아니면 되돌린다.
    /// 그 뒤 압축과 폴리시를 돌려, 아무것도 옮기지 않은 대조군과 최종 폭을 비교한다.
    ///
    /// 노브: `AB_SOLUTION`·`AB_MOVES`(필수), `AB_INPUT`, `AB_OVERLAP_MM`(0.4), `AB_CMPR_SECS`(60),
    /// `AB_SEEDS`(`1,2`), `AB_TUCK_CAP_MS`(3000).
    #[test]
    #[ignore]
    fn tuck_small_pieces_into_pockets_ab_on_a_saved_layout() {
        use jagua_rs::geometry::DTransformation;
        use rand::SeedableRng;
        use rand::rngs::Xoshiro256PlusPlus;

        let (Ok(solution_path), Ok(moves_path)) =
            (std::env::var("AB_SOLUTION"), std::env::var("AB_MOVES"))
        else {
            println!("[TUCK] AB_SOLUTION / AB_MOVES missing; skipped");
            return;
        };
        let env = |key: &str, default: &str| std::env::var(key).unwrap_or_else(|_| default.into());
        let list = |key: &str, default: &str| -> Vec<u64> {
            env(key, default).split(',').filter_map(|v| v.trim().parse().ok()).collect()
        };
        let input = std::env::var("AB_INPUT")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|_| {
                std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input/wsjk156.json")
            });
        let overlap: f32 = env("AB_OVERLAP_MM", "0.4").parse().unwrap();
        let secs: u64 = env("AB_CMPR_SECS", "60").parse().unwrap();
        let tuck_cap = Duration::from_millis(env("AB_TUCK_CAP_MS", "3000").parse().unwrap());

        let (ext, _) = crate::util::io::read_spp_input(&input).unwrap();
        let c = DEFAULT_SWARM_CONFIG;
        let instance = crate::util::io::import_spp_instance(
            &ext,
            c.cde_config,
            c.poly_simpl_tolerance,
            (overlap > 0.0).then_some(-overlap),
            c.narrow_concavity_cutoff_ratio,
        )
        .unwrap();
        let ext_sol: jagua_rs::probs::spp::io::ext_repr::ExtSPSolution =
            serde_json::from_str(&std::fs::read_to_string(&solution_path).unwrap()).unwrap();
        let start = crate::util::frozen::import_solution(&instance, &ext_sol);
        let start_sol = start.save();
        let moves: Vec<serde_json::Value> =
            serde_json::from_str(&std::fs::read_to_string(&moves_path).unwrap()).unwrap();
        println!(
            "\n[TUCK] start {:.4} ({:.3}%), {} proposed moves  ({solution_path})",
            start_sol.strip_width(),
            start_sol.density() * 100.0,
            moves.len()
        );

        let mut cmpr = c.cmpr_cfg;
        cmpr.time_limit = Duration::from_secs(secs);
        cmpr.separator_config.n_workers = 3;
        let mut plsh = c.polish_cfg.clone();
        plsh.reclaim.enabled = false;
        for seed in list("AB_SEEDS", "1,2") {
            for tuck in [false, true] {
                let mut sep = Separator::new(
                    instance.clone(),
                    start.clone(),
                    Xoshiro256PlusPlus::seed_from_u64(seed),
                    cmpr.separator_config,
                );
                let mut accepted = 0usize;
                let mut tuck_secs = 0.0f32;
                if tuck {
                    let started = Instant::now();
                    for mv in &moves {
                        let item_id = mv["item_id"].as_u64().unwrap() as usize;
                        let (dx, dy) = (mv["dx"].as_f64().unwrap() as f32, mv["dy"].as_f64().unwrap() as f32);
                        let Some((pk, dt)) = sep
                            .prob
                            .layout()
                            .placed_items()
                            .iter()
                            .find(|(_, p)| p.item().idx() == item_id)
                            .map(|(pk, p)| (pk, p.d_transf()))
                        else {
                            continue;
                        };
                        let before = sep.prob.save();
                        let (x, y) = dt.translation();
                        sep.move_item(pk, DTransformation::new(dt.rotation(), (x + dx, y + dy)));
                        let loss_after_move = sep.ct.get_total_loss();
                        let mut term = BasicTerminator::new();
                        term.new_timeout(tuck_cap);
                        let (sol, ct) = sep.separate(&term, &mut None);
                        let ok = ct.get_total_loss() == 0.0 && sep.solution_is_certified(&sol, &start_sol);
                        println!(
                            "[TUCK]   item {item_id:3}  ({:5.1}% proposed overlap, x {:6.0} -> {:6.0})  loss after move {:9.1}  -> {}",
                            mv["overlap_frac"].as_f64().unwrap_or(0.0) * 100.0,
                            mv["from_x"].as_f64().unwrap_or(0.0),
                            mv["to_x"].as_f64().unwrap_or(0.0),
                            loss_after_move,
                            if ok { "resolved" } else { "rolled back" }
                        );
                        if ok {
                            sep.rollback(&sol, Some(&ct));
                            accepted += 1;
                        } else {
                            sep.rollback(&before, None);
                        }
                    }
                    tuck_secs = started.elapsed().as_secs_f32();
                }
                let tucked = sep.prob.save();
                let mut term = BasicTerminator::new();
                term.new_timeout(cmpr.time_limit);
                let compressed = compression_phase(&instance, &mut sep, &tucked, &mut None, &term, &cmpr);
                let mut term = BasicTerminator::new();
                term.new_timeout(plsh.time_limit);
                let polished = crate::optimizer::polish::polish_phase_with_nfp(
                    &instance,
                    &mut sep,
                    &compressed,
                    &mut None,
                    &term,
                    &plsh,
                    &c.nfp_integration,
                    &crate::lp_solver::DEFAULT_LP_SOLVER_CONFIG,
                );
                println!(
                    "[TUCK] seed {seed} {}  accepted {accepted}/{} in {tuck_secs:.1}s  compress {:.4} ({:+.3}mm)  polished {:.4} ({:.3}%)",
                    if tuck { "tuck   " } else { "control" },
                    if tuck { moves.len() } else { 0 },
                    compressed.strip_width(),
                    compressed.strip_width() - start_sol.strip_width(),
                    polished.strip_width(),
                    polished.density() * 100.0
                );
                assert!(sep.solution_is_certified(&polished, &start_sol));
            }
        }
    }

    /// **큰 패널 두 장의 자리를 바꾸면 길이가 어떻게 변하는가** — SAHA(Gomes & Oliveira 2006)식 이웃의 값어치를 잰다.
    ///
    /// 폴리시가 끝난 배치에서 출발한다. 시도마다: bbox 가 비슷한 큰 조각 두 장을 골라 bbox 중심을
    /// 맞바꾸고, 폭을 그대로 둔 채 분리를 `AB_SWAP_CAP_MS` 동안 돌린다. 풀리면 폴리시(LP 압축)를
    /// `AB_SWAP_POLISH_SECS` 동안 돌려 폭 변화를 적고, **매번 출발 배치로 되돌린다**(독립 시행).
    /// 개선되는 시행의 비율과 크기가 담금질·반복 국소 탐색이 이 이웃에서 얻을 수 있는 것의 상한이다.
    ///
    /// 노브: `AB_SOLUTION`(필수), `AB_INPUT`, `AB_OVERLAP_MM`(0.4), `AB_SWAPS`(40), `AB_SEED`(1),
    /// `AB_SWAP_CAP_MS`(5000), `AB_SWAP_POLISH_SECS`(12), `AB_SWAP_BBOX_TOL_MM`(80).
    #[test]
    #[ignore]
    fn swap_two_large_panels_then_polish_on_a_saved_layout() {
        use jagua_rs::geometry::DTransformation;
        use rand::RngExt;
        use rand::SeedableRng;
        use rand::rngs::Xoshiro256PlusPlus;

        let Ok(solution_path) = std::env::var("AB_SOLUTION") else {
            println!("[SWAP] AB_SOLUTION missing; skipped");
            return;
        };
        let env = |key: &str, default: &str| std::env::var(key).unwrap_or_else(|_| default.into());
        let input = std::env::var("AB_INPUT")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|_| {
                std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input/wsjk156.json")
            });
        let overlap: f32 = env("AB_OVERLAP_MM", "0.4").parse().unwrap();
        let n_swaps: usize = env("AB_SWAPS", "40").parse().unwrap();
        let seed: u64 = env("AB_SEED", "1").parse().unwrap();
        let cap = Duration::from_millis(env("AB_SWAP_CAP_MS", "5000").parse().unwrap());
        let polish_secs: u64 = env("AB_SWAP_POLISH_SECS", "12").parse().unwrap();
        let tol: f32 = env("AB_SWAP_BBOX_TOL_MM", "80").parse().unwrap();

        let (ext, _) = crate::util::io::read_spp_input(&input).unwrap();
        let c = DEFAULT_SWARM_CONFIG;
        let instance = crate::util::io::import_spp_instance(
            &ext,
            c.cde_config,
            c.poly_simpl_tolerance,
            (overlap > 0.0).then_some(-overlap),
            c.narrow_concavity_cutoff_ratio,
        )
        .unwrap();
        let ext_sol: jagua_rs::probs::spp::io::ext_repr::ExtSPSolution =
            serde_json::from_str(&std::fs::read_to_string(&solution_path).unwrap()).unwrap();
        let start = crate::util::frozen::import_solution(&instance, &ext_sol);

        let mut sep_config = c.cmpr_cfg.separator_config;
        sep_config.n_workers = 3;
        let mut plsh = c.polish_cfg.clone();
        plsh.reclaim.enabled = false;
        plsh.time_limit = Duration::from_secs(polish_secs);
        let polish = |sep: &mut Separator, from: &SPSolution| {
            let mut term = BasicTerminator::new();
            term.new_timeout(plsh.time_limit);
            crate::optimizer::polish::polish_phase_with_nfp(
                &instance,
                sep,
                from,
                &mut None,
                &term,
                &plsh,
                &c.nfp_integration,
                &crate::lp_solver::DEFAULT_LP_SOLVER_CONFIG,
            )
        };

        let mut rng = Xoshiro256PlusPlus::seed_from_u64(seed);
        let mut sep = Separator::new(instance.clone(), start.clone(), rng.clone(), sep_config);
        // 출발점을 폴리시의 고정점으로 맞춘다 — 그래야 자리 바꾸기의 효과만 남는다.
        let loaded = start.save();
        let base = polish(&mut sep, &loaded);
        let base_width = base.strip_width();
        println!(
            "\n[SWAP] loaded {:.3}, polished base {:.3} ({:.3}%)  ({solution_path})",
            loaded.strip_width(),
            base_width,
            base.density() * 100.0
        );

        // 큰 조각: 면적이 가장 큰 조각의 25% 이상.
        sep.rollback(&base, None);
        let max_area = sep.prob.layout().placed_items().values().map(|p| p.shape().area()).fold(0.0, f32::max);
        let large: Vec<(usize, f32, f32, f32, f32)> = sep
            .prob
            .layout()
            .placed_items()
            .values()
            .filter(|p| p.shape().area() >= 0.25 * max_area)
            .map(|p| {
                let b = p.shape().bbox();
                (p.item().idx(), b.x_max - b.x_min, b.y_max - b.y_min, 0.5 * (b.x_min + b.x_max), 0.5 * (b.y_min + b.y_max))
            })
            .collect();
        let mut pairs = vec![];
        for (i, a) in large.iter().enumerate() {
            for b in &large[i + 1..] {
                if (a.1 - b.1).abs() <= tol && (a.2 - b.2).abs() <= tol {
                    pairs.push((*a, *b));
                }
            }
        }
        println!("[SWAP] {} large pieces, {} similar pairs (bbox within {tol}mm)", large.len(), pairs.len());

        let (mut unresolved, mut better, mut same, mut worse) = (0usize, 0usize, 0usize, 0usize);
        let mut deltas = vec![];
        for trial in 0..n_swaps.min(pairs.len()) {
            let (a, b) = pairs.swap_remove(rng.random_range(0..pairs.len()));
            sep.rollback(&base, None);
            for (item_id, dx, dy) in [(a.0, b.3 - a.3, b.4 - a.4), (b.0, a.3 - b.3, a.4 - b.4)] {
                let (pk, dt) = sep
                    .prob
                    .layout()
                    .placed_items()
                    .iter()
                    .find(|(_, p)| p.item().idx() == item_id)
                    .map(|(pk, p)| (pk, p.d_transf()))
                    .unwrap();
                let (x, y) = dt.translation();
                sep.move_item(pk, DTransformation::new(dt.rotation(), (x + dx, y + dy)));
            }
            let loss = sep.ct.get_total_loss();
            let started = Instant::now();
            let mut term = BasicTerminator::new();
            term.new_timeout(cap);
            let (sol, ct) = sep.separate(&term, &mut None);
            let sep_secs = started.elapsed().as_secs_f32();
            if ct.get_total_loss() != 0.0 || !sep.solution_is_certified(&sol, &base) {
                unresolved += 1;
                println!(
                    "[SWAP] {trial:2}  items {:3} <-> {:3}  (dx {:6.0}, dy {:5.0})  loss {:9.0}  unresolved in {sep_secs:.1}s",
                    a.0, b.0, b.3 - a.3, b.4 - a.4, loss
                );
                continue;
            }
            sep.rollback(&sol, Some(&ct));
            let polished = polish(&mut sep, &sol);
            let delta = polished.strip_width() - base_width;
            deltas.push(delta);
            if delta < -0.05 {
                better += 1;
            } else if delta > 0.05 {
                worse += 1;
            } else {
                same += 1;
            }
            println!(
                "[SWAP] {trial:2}  items {:3} <-> {:3}  (dx {:6.0}, dy {:5.0})  loss {:9.0}  resolved in {sep_secs:.1}s  width {:+8.3}mm",
                a.0,
                b.0,
                b.3 - a.3,
                b.4 - a.4,
                loss,
                delta
            );
        }
        deltas.sort_by(f32::total_cmp);
        println!(
            "[SWAP] trials {}  unresolved {unresolved}  better {better}  same {same}  worse {worse}  best {:+.3}mm  median {:+.3}mm",
            unresolved + deltas.len(),
            deltas.first().copied().unwrap_or(f32::NAN),
            deltas.get(deltas.len() / 2).copied().unwrap_or(f32::NAN),
        );
    }

    /// **저장된 탐색 해 위에서 압축 시도 상한 A/B.** 압축 뒤 폴리시까지 돌려 최종 폭으로 비교한다.
    ///
    /// 노브: `AB_SOLUTION`(필수), `AB_INPUT`(기본 wsjk156), `AB_OVERLAP_MM`(기본 0.4),
    /// `AB_CMPR_SECS`(기본 60), `AB_CAPS_MS`(기본 `0,10000`; 0 = 상한 없음), `AB_SEEDS`(기본 `1,2`).
    #[test]
    #[ignore]
    fn compress_attempt_cap_ab_on_a_saved_layout() {
        use rand::SeedableRng;
        use rand::rngs::Xoshiro256PlusPlus;

        let Ok(solution_path) = std::env::var("AB_SOLUTION") else {
            println!("[A/B] AB_SOLUTION missing; skipped");
            return;
        };
        let env = |key: &str, default: &str| std::env::var(key).unwrap_or_else(|_| default.into());
        let list = |key: &str, default: &str| -> Vec<u64> {
            env(key, default).split(',').filter_map(|v| v.trim().parse().ok()).collect()
        };
        let input = std::env::var("AB_INPUT")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|_| {
                std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input/wsjk156.json")
            });
        let overlap: f32 = env("AB_OVERLAP_MM", "0.4").parse().unwrap();
        let secs: u64 = env("AB_CMPR_SECS", "60").parse().unwrap();

        let (ext, _) = crate::util::io::read_spp_input(&input).unwrap();
        let c = DEFAULT_SWARM_CONFIG;
        let instance = crate::util::io::import_spp_instance(
            &ext,
            c.cde_config,
            c.poly_simpl_tolerance,
            (overlap > 0.0).then_some(-overlap),
            c.narrow_concavity_cutoff_ratio,
        )
        .unwrap();
        let ext_sol: jagua_rs::probs::spp::io::ext_repr::ExtSPSolution =
            serde_json::from_str(&std::fs::read_to_string(&solution_path).unwrap()).unwrap();
        let start = crate::util::frozen::import_solution(&instance, &ext_sol);
        let start_sol = start.save();
        println!("\n[A/B] start {:.4} ({:.3}%)  ({solution_path})", start_sol.strip_width(), start_sol.density() * 100.0);

        let mut cmpr = c.cmpr_cfg;
        cmpr.time_limit = Duration::from_secs(secs);
        cmpr.separator_config.n_workers = 3;
        let mut plsh = c.polish_cfg.clone();
        plsh.reclaim.enabled = false;
        for cap in list("AB_CAPS_MS", "0,10000") {
            for seed in list("AB_SEEDS", "1,2") {
                ATTEMPT_CAP_MS.store(cap, std::sync::atomic::Ordering::Relaxed);
                let mut sep = Separator::new(
                    instance.clone(),
                    start.clone(),
                    Xoshiro256PlusPlus::seed_from_u64(seed),
                    cmpr.separator_config,
                );
                let mut term = BasicTerminator::new();
                term.new_timeout(cmpr.time_limit);
                let compressed = compression_phase(&instance, &mut sep, &start_sol, &mut None, &term, &cmpr);
                let mut term = BasicTerminator::new();
                term.new_timeout(plsh.time_limit);
                let polished = crate::optimizer::polish::polish_phase_with_nfp(
                    &instance,
                    &mut sep,
                    &compressed,
                    &mut None,
                    &term,
                    &plsh,
                    &c.nfp_integration,
                    &crate::lp_solver::DEFAULT_LP_SOLVER_CONFIG,
                );
                println!(
                    "[A/B] cap {cap:5}ms seed {seed}  compress {:.4} ({:+.3}mm)  polished {:.4} ({:.3}%)",
                    compressed.strip_width(),
                    compressed.strip_width() - start_sol.strip_width(),
                    polished.strip_width(),
                    polished.density() * 100.0
                );
                assert!(sep.solution_is_certified(&polished, &start_sol));
            }
        }
        ATTEMPT_CAP_MS.store(0, std::sync::atomic::Ordering::Relaxed);
    }
}
