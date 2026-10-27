use crate::optimizer::Terminator;
use crate::optimizer::place_groups::{PlaceGroupError, PlaceGroups};
use crate::optimizer::worker::SeparatorWorker;
use crate::quantify::tracker::{CTSnapshot, CollisionTracker};
use crate::sample::search::SampleConfig;
use crate::util::assertions::tracker_matches_layout;
use crate::util::listener::{ReportType, SolutionListener};
use itertools::Itertools;
use jagua_rs::entities::PItemKey;
use jagua_rs::geometry::DTransformation;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution};
use ordered_float::OrderedFloat;
use rand::rngs::Xoshiro256PlusPlus;
use rand::{RngExt, SeedableRng};
use rayon::ThreadPool;
use rayon::iter::IndexedParallelIterator;
use rayon::iter::IntoParallelRefMutIterator;
use rayon::iter::ParallelIterator;

mod early_report;
mod repair_config;
pub use repair_config::SeparationRepairConfig;

#[cfg(test)]
mod explore_code_tests {
    use super::*;
    use crate::config::DEFAULT_SWARM_CONFIG;
    use crate::consts::LBF_SAMPLE_CONFIG;
    use crate::optimizer::lbf::LBFBuilder;
    use crate::util::{io::read_spp_input, terminator::BasicTerminator};
    use jagua_rs::Instant;
    use std::path::Path;

    fn simple_separator(workers: usize) -> Separator {
        let cfg = DEFAULT_SWARM_CONFIG;
        let ext = serde_json::from_value(serde_json::json!({
            "name":"early-certification", "strip_height":20.0,
            "items":[{"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":5.0,"height":5.0}
            }}]
        }))
        .unwrap();
        let importer = crate::util::io::new_importer(cfg.cde_config, cfg.poly_simpl_tolerance, cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
        let instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext).unwrap();
        let mut problem = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
        problem.change_strip_width(20.0).expect(crate::dmsg!("strip width must be positive"));
        problem.place_item(SPPlacement {
            item_idx: 0,
            d_transf: DTransformation::new(0.0, (8.0, 8.0)),
        });
        let mut config = cfg.expl_cfg.separator_config;
        config.n_workers = workers;
        Separator::new(
            instance,
            problem,
            Xoshiro256PlusPlus::seed_from_u64(42),
            config,
        )
    }

    #[test]
    fn place_group_policy_survives_worker_reload() {
        use crate::optimizer::place_groups::{PlaceGroupMember, PlaceGroupSpec};
        let cfg = DEFAULT_SWARM_CONFIG;
        let items: Vec<_>=(0..2).map(|id|serde_json::json!({
            "id":id,"demand":1,"allowed_orientations":[0.0],
            "shape":{"type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":10.0,"height":10.0}}
        })).collect();
        let ext = serde_json::from_value(
            serde_json::json!({"name":"worker-policy","strip_height":40.0,"items":items}),
        )
        .unwrap();
        let instance =
            crate::util::io::import_spp_instance(&ext, cfg.cde_config, None, None, None).unwrap();
        let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
        prob.change_strip_width(100.0).expect(crate::dmsg!("strip width must be positive"));
        for (id, x) in [(0, 20.0), (1, 25.0)] {
            prob.place_item(SPPlacement {
                item_idx: id,
                d_transf: DTransformation::new(0.0, (x, 20.0)),
            });
        }
        let groups = PlaceGroups::new(
            &instance,
            vec![PlaceGroupSpec {
                id: 42,
                members: prob
                    .layout()
                    .placed_items()
                    .values()
                    .map(|pi| PlaceGroupMember {
                        item_id: pi.item().idx(),
                        initial_transform: pi.d_transf(),
                    })
                    .collect(),
            }],
        )
        .unwrap();
        let mut config = cfg.expl_cfg.separator_config;
        config.n_workers = 2;
        config.nfp_sampler = false;
        let mut sep = Separator::new(
            instance,
            prob,
            Xoshiro256PlusPlus::seed_from_u64(42),
            config,
        );
        sep.set_place_groups(groups.clone()).unwrap();
        let solution = sep.prob.save();
        for worker in &mut sep.workers {
            worker.ct = CollisionTracker::new(&worker.prob.layout());
            assert!(worker.ct.get_total_loss() > 0.0);
            worker.load(&solution, &sep.ct);
            assert_eq!(worker.ct.place_groups, groups);
            assert_eq!(worker.ct.get_total_loss(), 0.0);
            assert!(tracker_matches_layout(&worker.ct, &worker.prob.layout()));
            assert!(
                worker
                    .place_groups
                    .solution_preserves_relative_poses(&worker.prob.save())
            );
        }
    }

    #[test]
    fn early_callback_stop_retains_full_certified_solution() {
        use std::sync::atomic::{AtomicBool, Ordering};
        struct Stop(AtomicBool);
        impl Terminator for Stop {
            fn kill(&self) -> bool {
                self.0.load(Ordering::Relaxed)
            }
            fn new_timeout(&mut self, _: std::time::Duration) {}
            fn timeout_at(&self) -> Option<Instant> {
                None
            }
        }
        struct Listener<'a> {
            stop: &'a Stop,
            saved: Option<SPSolution>,
            owner: std::thread::ThreadId,
        }
        impl SolutionListener for Listener<'_> {
            fn report(&mut self, _: ReportType, _: &SPSolution, _: &SPInstance) {}
            fn report_certified_early(&mut self, solution: &SPSolution, _: &SPInstance) {
                assert_eq!(self.owner, std::thread::current().id());
                self.saved = Some(solution.clone());
                self.stop.0.store(true, Ordering::Relaxed);
            }
        }
        for workers in [1, 2, 4] {
            let mut sep = simple_separator(workers);
            let initial = sep.prob.save();
            let stop = Stop(AtomicBool::new(false));
            let mut listener = Listener {
                stop: &stop,
                saved: None,
                owner: std::thread::current().id(),
            };
            let result = sep.separate_with_early_reports(&stop, &mut Some(&mut listener));
            assert!(listener.saved.is_some());
            assert!(stop.kill());
            assert_eq!(result.1.get_total_loss(), 0.0);
            assert!(sep.solution_is_certified(&result.0, &initial));
            assert_eq!(
                result.0.strip_width(),
                listener.saved.unwrap().strip_width()
            );
        }
    }

    #[test]
    fn interrupted_return_preserves_early_poses_without_changing_normal_ties() {
        let mut sep = simple_separator(2);
        let early = (sep.prob.save(), sep.ct.save());
        let reference = early.0.clone();
        let key = sep.prob.layout().placed_items().keys().next().unwrap();
        sep.move_item(key, DTransformation::new(0.0, (9.0, 8.0)));
        let normal = (sep.prob.save(), sep.ct.save());
        assert_eq!(early.0.strip_width(), normal.0.strip_width());
        assert!(sep.solution_is_certified(&early.0, &reference));
        assert!(sep.solution_is_certified(&normal.0, &reference));
        let returned_x = |solution: &SPSolution| {
            solution
                .layout_snapshot()
                .placed_items()
                .values()
                .next()
                .unwrap()
                .d_transf()
                .translation()
                .0
        };
        let uninterrupted =
            sep.select_returned_incumbent(normal.clone(), early.clone(), &reference, false);
        assert_eq!(
            returned_x(&uninterrupted.0),
            9.0,
            "normal equal-width round winner remains authoritative"
        );
        let interrupted = sep.select_returned_incumbent(normal, early.clone(), &reference, true);
        assert_eq!(
            returned_x(&interrupted.0),
            8.0,
            "Stop/timeout must preserve already-published coordinates"
        );

        sep.change_strip_width(19.0, None);
        let shorter = (sep.prob.save(), sep.ct.save());
        assert!(sep.solution_is_certified(&shorter.0, &reference));
        let interrupted = sep.select_returned_incumbent(shorter, early, &reference, true);
        assert_eq!(
            interrupted.0.strip_width(),
            19.0,
            "a shorter certified result still takes precedence"
        );
    }

    #[test]
    fn zero_tracker_loss_cannot_publish_disallowed_rotation() {
        struct Listener(usize);
        impl SolutionListener for Listener {
            fn report(&mut self, _: ReportType, _: &SPSolution, _: &SPInstance) {
                self.0 += 1;
            }
        }
        let mut sep = simple_separator(2);
        // A geometrically clear whole layout may still violate allowed rotations.
        let key = sep.prob.layout().placed_items().keys().next().unwrap();
        sep.move_item(key, DTransformation::new(0.5, (8.0, 8.0)));
        assert_eq!(sep.ct.get_total_loss(), 0.0);
        let mut listener = Listener(0);
        let mut retained = None;
        sep.move_items_round(&BasicTerminator::new(), Some(&mut listener), None, &mut retained);
        assert_eq!(listener.0, 0);
        assert!(retained.is_none());
    }

    #[test]
    fn single_worker_round_does_not_copy_the_collision_matrix() {
        let mut sep = simple_separator(1);
        assert!(sep.thread_pool.is_none());
        let key = sep.prob.layout().placed_items().keys().next().unwrap();
        sep.move_item(key, DTransformation::new(0.0, (1.0, 8.0)));
        assert!(sep.ct.get_total_loss() > 0.0);
        sep.move_items_multi(&BasicTerminator::new());
        assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));
    }

    #[test]
    fn winner_transfer_matches_snapshot_restore_across_iterations() {
        for workers in [1, 2, 4] {
            let cfg = DEFAULT_SWARM_CONFIG;
            let (ext, _) = read_spp_input(
                &Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input/fu.json"),
            )
            .unwrap();
            let importer = crate::util::io::new_importer(cfg.cde_config, cfg.poly_simpl_tolerance, cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
            let instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext).unwrap();
            let builder = LBFBuilder::new(
                instance.clone(),
                Xoshiro256PlusPlus::seed_from_u64(17),
                LBF_SAMPLE_CONFIG,
                cfg.lbf_x_multiplier,
                cfg.lbf_y_multiplier,
            )
            .construct();
            let mut sep_cfg = cfg.expl_cfg.separator_config;
            sep_cfg.n_workers = workers;
            let mut actual = Separator::new(
                instance.clone(),
                builder.prob.clone(),
                Xoshiro256PlusPlus::seed_from_u64(42),
                sep_cfg,
            );
            let mut reference = Separator::new(
                instance,
                builder.prob,
                Xoshiro256PlusPlus::seed_from_u64(42),
                sep_cfg,
            );
            let term = BasicTerminator::new();
            for iteration in 0..12 {
                if actual.ct.get_total_loss() == 0.0 {
                    let width = actual.prob.strip_width() * (1.0 - cfg.expl_cfg.shrink_step);
                    actual.change_strip_width(width, None);
                    reference.change_strip_width(width, None);
                }
                struct Listener;
                impl SolutionListener for Listener {
                    fn report(&mut self, _: ReportType, _: &SPSolution, _: &SPInstance) {}
                }
                // Exercise both the fast path and early-delivery path against the
                // original sequential winner/restore oracle with identical seeds.
                if iteration % 2 == 0 {
                    actual.move_items_round(&term, Some(&mut Listener), None, &mut None);
                } else {
                    actual.move_items_multi(&term);
                }
                let master = reference.prob.save();
                for worker in &mut reference.workers {
                    worker.load(&master, &reference.ct);
                    worker.move_items(&term);
                }
                let winner = reference
                    .workers
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, w)| OrderedFloat(w.ct.get_total_weighted_loss()))
                    .unwrap()
                    .0;
                reference
                    .prob
                    .restore(&reference.workers[winner].prob.save());
                reference.ct.copy_from(&reference.workers[winner].ct);
                assert_eq!(
                    actual.ct.get_total_loss().to_bits(),
                    reference.ct.get_total_loss().to_bits()
                );
                for (key, pi) in actual.prob.layout().placed_items() {
                    assert_eq!(
                        pi.d_transf(),
                        reference.prob.layout().placed_items()[key].d_transf()
                    );
                }
                for (a, b) in actual.workers.iter().zip(&reference.workers) {
                    assert_eq!(a.rng.clone().random::<u64>(), b.rng.clone().random::<u64>());
                }
                assert!(tracker_matches_layout(&actual.ct, &actual.prob.layout()));
                actual.ct.update_weights_with(actual.config.gls);
                reference.ct.update_weights_with(reference.config.gls);
            }
        }
    }

    /// Fixed work and identical RNG streams; excludes import and initial construction.
    #[test]
    #[ignore]
    fn low_core_round_benchmark() {
        run_round_benchmark(false);
    }

    #[test]
    #[ignore]
    fn gls_simd_round_benchmark() {
        run_round_benchmark(true);
    }

    fn run_round_benchmark(gls_only: bool) {
        for input in ["fu", "wsjk156"] {
            let mut cfg = DEFAULT_SWARM_CONFIG;
            cfg.min_item_separation = Some(-0.08);
            cfg.fabric_edge_margin_mm = Some(0.2);
            let (ext, _) = read_spp_input(
                &Path::new(env!("CARGO_MANIFEST_DIR")).join(format!("../data/input/{input}.json")),
            )
            .unwrap();
            let importer = crate::util::io::new_importer(cfg.cde_config, cfg.poly_simpl_tolerance, cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
            let instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext).unwrap();
            let builder = LBFBuilder::new(
                instance.clone(),
                Xoshiro256PlusPlus::seed_from_u64(17),
                LBF_SAMPLE_CONFIG,
                cfg.lbf_x_multiplier,
                cfg.lbf_y_multiplier,
            )
            .construct();
            for workers in [1, 2, 4] {
                for seed in [42, 71, 1234] {
                    let mut sep_cfg = cfg.expl_cfg.separator_config;
                    sep_cfg.n_workers = workers;
                    let mut reference = Separator::new(
                        instance.clone(),
                        builder.prob.clone(),
                        Xoshiro256PlusPlus::seed_from_u64(seed),
                        sep_cfg,
                    );
                    // The legacy implementation always created a private pool,
                    // even with one worker. Reproduce it for a fair baseline.
                    if !gls_only {
                        reference.thread_pool = Some(
                            rayon::ThreadPoolBuilder::new()
                                .num_threads(workers)
                                .build()
                                .unwrap(),
                        );
                    }
                    let mut actual = Separator::new(
                        instance.clone(),
                        builder.prob.clone(),
                        Xoshiro256PlusPlus::seed_from_u64(seed),
                        sep_cfg,
                    );
                    let term = BasicTerminator::new();
                    let mut measurements = [0.; 2];
                    let order = if gls_only && seed == 71 {
                        [(false, &mut actual), (true, &mut reference)]
                    } else {
                        [(true, &mut reference), (false, &mut actual)]
                    };
                    for (legacy, sep) in order {
                        sep.change_strip_width(sep.prob.strip_width() * 0.95, None);
                        let start = std::time::Instant::now();
                        for _ in 0..100 {
                            if sep.ct.get_total_loss() == 0.0 {
                                sep.change_strip_width(sep.prob.strip_width() * 0.999, None);
                            }
                            if legacy && !gls_only {
                                sep.move_items_round_legacy_benchmark(&term, None, &mut None);
                            } else {
                                sep.move_items_multi(&term);
                            }
                            if legacy && gls_only {
                                sep.ct.update_weights_scalar_for_benchmark(sep.config.gls);
                            } else {
                                sep.ct.update_weights_with(sep.config.gls);
                            }
                        }
                        measurements[usize::from(!legacy)] = start.elapsed().as_secs_f64();
                    }
                    for (old, new) in reference.workers.iter().zip(&actual.workers) {
                        assert_eq!(
                            old.rng.clone().random::<[u64; 4]>(),
                            new.rng.clone().random::<[u64; 4]>()
                        );
                    }
                    assert_eq!(
                        reference.ct.get_total_loss().to_bits(),
                        actual.ct.get_total_loss().to_bits()
                    );
                    for (key, placed) in reference.prob.layout().placed_items() {
                        assert_eq!(
                            placed.d_transf(),
                            actual.prob.layout().placed_items()[key].d_transf()
                        );
                    }
                    println!(
                        "{} input={input} workers={workers} seed={seed} legacy_s={:.6} optimized_s={:.6} speedup={:.3}",
                        if gls_only { "GLS_ROUND" } else { "LOW_CORE" },
                        measurements[0],
                        measurements[1],
                        measurements[0] / measurements[1]
                    );
                }
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SeparatorConfig {
    /// 개선 없이 허용할 separation 내부 루프 반복 수.
    /// capmods 빌드에서는 `max(이 값, 배치된 아이템 수 × 2.5)`가 실제 한도로 쓰인다
    /// (대형 인스턴스 벤치마크 근거는 [`Separator::separate`] 내부 주석 참조).
    pub iter_no_imprv_limit: usize,
    pub strike_limit: usize,
    pub n_workers: usize,
    pub sample_config: SampleConfig,
    /// 스왑(교환) 이웃 연산 주기: separation 내부 루프 몇 iteration마다 시도할지.
    /// 0이면 비활성. capmods 빌드에서만 반영. 기본값 20.
    pub swap_period: usize,
    /// 스왑 패스 1회당 시도할 스왑 수. 기본값 8.
    pub swap_attempts: usize,
    /// 스왑 파트너로 허용할 면적비 범위 (기준 아이템 대비 min, max). 기본값 (0.4, 2.5).
    pub swap_area_ratio: (f32, f32),
    /// ① NFP 샘플러 — separator 의 난수 후보를 **정확한 NFP 후보**로 교체할지.
    ///
    /// [`crate::config::NfpIntegrationConfig::nfp_sampler`] 에서 전파된 값이다.
    /// 꺼져 있으면 [`crate::sample::search::search_placement`] 경로가 한 비트도 바뀌지 않는다.
    ///
    /// # 왜 여기 있나
    ///
    /// 이 플래그는 오래도록 WPF 체크박스 -> C# -> C ABI -> `SwarmConfig` 까지 도달한 뒤
    /// **읽는 코드가 없어** 아무 일도 하지 않았다(`search_placement_nfp` 의 호출자가
    /// 자기 파일 테스트뿐이었다). 후보를 만드는 주체가 `Separator` 이므로 설정도 여기 둔다.
    /// 전파가 끊기면 `nfp_sampler_flag_reaches_the_separator_config` 가 깨진다.
    pub nfp_sampler: bool,
    /// GLS 가중치 학습 파라미터. 기본값은 [`GlsWeightConfig::DEFAULT`] 로
    /// `consts.rs` 의 기존 상수와 비트 단위로 같다.
    pub gls: GlsWeightConfig,
    /// ★ 정체 재출발 창. `0` = 꺼짐(기본, 종전 동작).
    ///
    /// 한 strike 안에서 개선 없이 이 라운드 수가 지나면 **최선 해(인컴번트)로 되돌아가
    /// 새 난수 궤적으로 다시 출발**한다. strike 카운터(`iter_no_imprv_limit`)는 그대로
    /// 누적되므로 포기 시점은 바뀌지 않는다 — 바뀌는 것은 한 strike 안에서 인컴번트를
    /// 떠나 헤매는 시간뿐이다.
    ///
    /// # 근거 (여성WSjk 156, 겹침 0.8mm, Explore 240초 로그)
    ///
    /// 벽에 가까운 폭(6391~6438mm)에서 분리 한 번이 17~58초를 쓰는데, 개선(`(*)`)은
    /// **strike 롤백 직후 수십 라운드 안에** 몰려 있고 그 뒤 390라운드는 개선이 없다.
    /// 예: `i:27 → i:418`, `i:419 → i:811`, `i:1608 → i:2000`. 즉 인컴번트를 떠난 궤적은
    /// 돌아오지 못하고, 인컴번트에서 다시 출발할 때마다 곧바로 개선이 나온다.
    /// 그 롤백을 390라운드가 아니라 이 창마다 걸어 준다.
    ///
    /// # 실측 결과 — 기각 (`docs/EXPLORE_STALL_AND_STOP_2026-09-30.md`)
    ///
    /// 창 60 으로 3시드 240초: 평균 **−1.0%p** (78.17% vs 79.19%), 3/3 시드에서 나쁘다.
    /// 30초 시점부터 이미 뒤진다 — 잦은 롤백이 GLS 가중치 학습을 끊는다. 위 가설은
    /// 틀렸다. 노브는 기본 꺼짐으로 두고 근거로 재제안하지 말 것.
    pub restart_window: usize,
    /// ★ capmods 의 `iter_no_imprv_limit` 하한(`아이템 수 × 2.5`)을 덮어쓴다.
    /// `0` = 종전 동작. 양수면 `max(iter_no_imprv_limit, 이 값)` 을 쓴다.
    pub iter_floor_override: usize,
}

/// GLS(Guided Local Search) 쌍 가중치의 학습률과 상한.
///
/// # 왜 설정으로 뺐나
///
/// 기존 상수(`증가 1.2~2.0`, `상한 1e6`, `감쇠 0.95`)로는 충돌이 지속되는 쌍의
/// 가중치가 **20~76 라운드 만에 상한에 닿는다** (`1.2^76 ≈ 2.0^20 ≈ 1.05e6`).
/// 벽에서의 한 번의 separation 은 `strike_limit × iter_no_imprv_limit` 로 천 라운드를
/// 넘기므로, 호출의 대부분 구간에서 어려운 쌍은 **전부 정확히 1e6** 이고 한 번도
/// 충돌한 적 없는 쌍은 1.0 이다. 그 상태에서는
///
/// - 가중 손실이 사실상 `1e6 × 비가중 손실` 이라 GLS 의 변별력이 0 이고,
/// - **새 충돌을 만드는 후보가 기존 충돌을 유지하는 것보다 1e6 배 싸다** — 평가기가
///   갈등을 해소하는 대신 새 영역으로 밀어내도록 유도된다.
///
/// 두 번째 성질은 측정된 현상("낭비가 가운데 구간 전역에 얇게 퍼진다")과 직접
/// 대응한다. 폭이 바뀔 때마다 `resync` 가 트래커를 1.0 으로 되돌리므로 이 포화
/// 곡선은 사다리 칸마다 똑같이 재생되고, 그래서 시드가 달라도 같은 자리에서
/// 벽을 만난다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GlsWeightConfig {
    /// 충돌 중인 쌍의 최소 증가율 (손실이 최악 대비 0 일 때).
    pub min_inc: f32,
    /// 충돌 중인 쌍의 최대 증가율 (손실이 최악과 같을 때).
    pub max_inc: f32,
    /// 충돌하지 않는 쌍의 감쇠율.
    pub decay: f32,
    /// 가중치 상한. 반복 곱셈이 f32 범위를 넘지 않게 한다.
    pub max_weight: f32,
}

impl GlsWeightConfig {
    /// `consts.rs` 의 기존 상수와 같은 값. 이 기본값에서는 동작이 한 비트도 바뀌지 않는다.
    pub const DEFAULT: Self = Self {
        min_inc: crate::consts::GLS_WEIGHT_MIN_INC_RATIO,
        max_inc: crate::consts::GLS_WEIGHT_MAX_INC_RATIO,
        decay: crate::consts::GLS_WEIGHT_DECAY,
        max_weight: crate::consts::GLS_WEIGHT_MAX,
    };

    /// 비정상 값은 조용히 기본값으로 되돌린다 — 죽은 노브보다 낫다.
    pub fn sanitized(self) -> Self {
        let d = Self::DEFAULT;
        Self {
            min_inc: if self.min_inc.is_finite() && self.min_inc > 1.0 {
                self.min_inc
            } else {
                d.min_inc
            },
            max_inc: if self.max_inc.is_finite() && self.max_inc >= self.min_inc {
                self.max_inc
            } else {
                d.max_inc
            },
            decay: if self.decay.is_finite() && self.decay > 0.0 && self.decay <= 1.0 {
                self.decay
            } else {
                d.decay
            },
            max_weight: if self.max_weight.is_finite() && self.max_weight > 1.0 {
                self.max_weight
            } else {
                d.max_weight
            },
        }
    }
}

impl Default for GlsWeightConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

pub struct Separator {
    pub(crate) prepared: std::sync::Arc<crate::prepared_geometry::PreparedGeometry>,
    pub instance: SPInstance,
    pub run_control: std::sync::Arc<crate::config::runtime::RunControl>,
    pub rng: Xoshiro256PlusPlus,
    pub prob: SPProblem,
    pub ct: CollisionTracker,
    pub workers: Vec<SeparatorWorker>,
    pub config: SeparatorConfig,
    pub thread_pool: Option<ThreadPool>,
    /// 고정할 item_id 비트마스크. true면 separation 중 이동하지 않는다.
    pub frozen_mask: Vec<bool>,
    /// Rigid placement units. Empty keeps the legacy single-item behavior.
    pub place_groups: PlaceGroups,
    original_guard: Option<crate::original_overlap::OriginalOverlapGuard>,
}

/// ① NFP 샘플러용 캐시를 만든다. 준비할 수 있는 형상이 하나도 없으면 `None`.
///
/// 실패해도 결함이 아니다 — 호출부가 `None` 을 받으면 기존 난수 샘플러가 그대로 돈다.
/// 회전 목록은 [`crate::optimizer::lns::instance_rotation_degrees`] 와 같은 규칙을 쓴다.
/// 두 소비자가 다른 회전 집합을 쓰면 같은 형상의 준비본이 두 벌 생겨 비용만 는다.
pub fn build_nfp_sampler_cache(
    instance: &SPInstance,
) -> Option<crate::nfp_lbf::cache::NfpLbfCache> {
    let rotations = crate::optimizer::lns::instance_rotation_degrees(instance)?;
    let library = crate::nfp_lbf::cache::PreparedLibrary::build(
        instance,
        &rotations,
        crate::nfp_lbf::DEFAULT_CONCAVE_SIMPLIFY_TOLERANCE,
    )?;
    Some(crate::nfp_lbf::cache::NfpLbfCache::new(library))
}

impl Separator {
    pub fn new(
        instance: SPInstance,
        prob: SPProblem,
        mut rng: Xoshiro256PlusPlus,
        config: SeparatorConfig,
    ) -> Self {
        let prepared =
            std::sync::Arc::new(crate::prepared_geometry::PreparedGeometry::new(&instance));
        let n_items = instance.total_item_qty();
        let n_workers = config.n_workers.max(1);
        let ct = CollisionTracker::new_with_frozen_prepared(&prob.layout(), &[], &prepared);
        // ① NFP 샘플러: 준비본(pair NFP·마스크·정규화 영역)은 **한 번만** 만들고
        // 워커마다 `fork()` 로 나눈다. `fork` 는 `Arc` 사본이라 기하를 다시 만들지 않으면서
        // 워커별 가변 상태를 분리한다 — 병렬 separation 이 `&mut` 로 막히지 않는다.
        let nfp_seed = config
            .nfp_sampler
            .then(|| build_nfp_sampler_cache(&instance))
            .flatten();
        let workers = (0..n_workers)
            .map(|_| SeparatorWorker {
                prepared: prepared.clone(),
                evaluator_pool: Default::default(),
                instance: instance.clone(),
                prob: prob.clone(),
                ct: ct.clone(),
                rng: Xoshiro256PlusPlus::seed_from_u64(rng.random()),
                sample_config: config.sample_config,
                frozen_mask: vec![false; n_items],
                place_groups: PlaceGroups::default(),
                nfp: nfp_seed.as_ref().map(|c| c.fork()),
            })
            .collect();

        let pool = if cfg!(target_arch = "wasm32") || n_workers == 1 {
            None
        } else {
            Some(
                rayon::ThreadPoolBuilder::new()
                    .num_threads(n_workers)
                    .build()
                    .unwrap(),
            )
        };

        let original_guard = crate::original_overlap::OriginalOverlapGuard::is_active(&instance)
            .then(|| crate::original_overlap::OriginalOverlapGuard::new(&instance));
        Self {
            prepared,
            run_control: crate::config::runtime::current_control(),
            prob,
            instance,
            rng,
            ct,
            workers,
            config,
            thread_pool: pool,
            frozen_mask: vec![false; n_items],
            place_groups: PlaceGroups::default(),
            original_guard,
        }
    }

    /// Individual operators leave rigid members in place as collision obstacles.
    pub fn can_move_individually(&self, item_id: usize) -> bool {
        !self.frozen_mask.get(item_id).copied().unwrap_or(false)
            && self.place_groups.group_id_of(item_id).is_none()
    }

    /// Certify/correct only the promising layout, retaining cheap search geometry
    /// for the candidate sampler. Failure keeps the incoming layout unchanged.
    /// 겹침 허용 모드의 원본 외곽 검증기. 겹침을 허용하지 않으면 `None`.
    pub fn original_guard(&self) -> Option<&crate::original_overlap::OriginalOverlapGuard> {
        self.original_guard.as_ref()
    }

    pub fn repair_original_overlap(&mut self, term: &impl Terminator) -> bool {
        let Some(mut guard) = self.original_guard.take() else {
            return true;
        };
        let result = guard.repair(self, term, false);
        self.original_guard = Some(guard);
        matches!(result, Ok(true))
    }

    /// Reserved compatibility entry point. Reinsertion operators were removed.
    pub fn configure_reinsertion(&mut self, _config: &crate::config::NfpIntegrationConfig) {}

    /// A zero surrogate loss is only a proposal: certify geometry and constraints.
    pub fn solution_is_certified(&self, candidate: &SPSolution, reference: &SPSolution) -> bool {
        self.place_groups
            .solution_preserves_relative_poses(candidate)
            && crate::util::assertions::frozen_poses_preserved(
                reference,
                candidate,
                &self.frozen_mask,
            )
            && crate::structure::pipeline::solution_validation_error_with_frozen_and_groups(
                &self.instance,
                candidate,
                &self.frozen_mask,
                &self.place_groups,
            )
            .is_none()
    }

    /// Installs validated rigid placement units for this phase and every
    /// worker. Call this whenever a new `Separator` is created for the same
    /// grouped run.
    pub fn set_place_groups(&mut self, groups: PlaceGroups) -> Result<(), PlaceGroupError> {
        groups.validate_layout(&self.prob.layout())?;
        self.place_groups = groups;
        if let Some(guard) = self.original_guard.as_mut() {
            guard.set_place_groups(&self.place_groups);
        }
        self.place_groups.expand_frozen_mask(&mut self.frozen_mask);
        self.ct = CollisionTracker::new_with_frozen_prepared_and_groups(
            &self.prob.layout(),
            &self.frozen_mask,
            &self.prepared,
            &self.place_groups,
        );
        for worker in &mut self.workers {
            worker.place_groups = self.place_groups.clone();
            worker.frozen_mask.clone_from(&self.frozen_mask);
            worker.ct = CollisionTracker::new_with_frozen_prepared_and_groups(
                &worker.prob.layout(),
                &worker.frozen_mask,
                &self.prepared,
                &self.place_groups,
            );
        }
        Ok(())
    }

    pub fn has_place_groups(&self) -> bool {
        !self.place_groups.is_empty()
    }

    pub fn placement_key(&self, item_id: usize) -> Option<PItemKey> {
        self.place_groups.placement_key(&self.prob.layout(), item_id)
    }

    /// Sum of current collision losses for all members of a placement unit.
    pub fn unit_loss(&self, item_id: usize) -> f32 {
        self.place_groups
            .member_item_ids(item_id)
            .into_iter()
            .filter_map(|id| self.placement_key(id))
            .map(|pk| self.ct.get_loss(pk))
            .sum()
    }

    /// 고정할 item_id 목록을 등록한다. 고정된 아이템은 separation 중 이동하지 않는다.
    /// 고정-고정 / 고정-경계 충돌은 해소 불가능하므로 loss 집계에서 제외되도록
    /// collision tracker를 재구축한다.
    pub fn freeze_items(&mut self, ids: &[usize]) {
        for &id in ids {
            if id < self.frozen_mask.len() {
                self.frozen_mask[id] = true;
            }
        }
        self.place_groups.expand_frozen_mask(&mut self.frozen_mask);
        self.ct = CollisionTracker::new_with_frozen_prepared_and_groups(
            &self.prob.layout(),
            &self.frozen_mask,
            &self.prepared,
            &self.place_groups,
        );
    }

    /// User locks may extend outside the strip and impose no width floor.
    pub fn min_feasible_width(&self) -> Option<f32> {
        // With no movable placement there is nothing to optimize. Retaining
        // the current positive width also avoids unlimited empty-strip shrink.
        self.prob
            .layout()
            .placed_items()
            .values()
            .all(|p| crate::util::frozen::is_frozen(&self.frozen_mask, p.item().idx()))
            .then_some(self.prob.strip_width())
    }

    /// Algorithm 9 from https://doi.org/10.48550/arXiv.2509.13329
    pub fn separate(
        &mut self,
        term: &impl Terminator,
        sol_listener: &mut Option<&mut dyn SolutionListener>,
    ) -> (SPSolution, CTSnapshot) {
        self.separate_impl(term, sol_listener, false)
    }

    /// Explore-only opt-in; compression keeps its existing report contract.
    pub fn separate_with_early_reports(
        &mut self,
        term: &impl Terminator,
        sol_listener: &mut Option<&mut dyn SolutionListener>,
    ) -> (SPSolution, CTSnapshot) {
        self.separate_impl(term, sol_listener, true)
    }

    /// Compatibility entry point. Auxiliary repair operators have been removed.
    pub fn separate_with_repair(
        &mut self,
        term: &impl Terminator,
        sol_listener: &mut Option<&mut dyn SolutionListener>,
        _config: &SeparationRepairConfig,
        early_reports: bool,
    ) -> (SPSolution, CTSnapshot) {
        self.separate_impl(term, sol_listener, early_reports)
    }

    fn separate_impl(
        &mut self,
        term: &impl Terminator,
        sol_listener: &mut Option<&mut dyn SolutionListener>,
        enable_early: bool,
    ) -> (SPSolution, CTSnapshot) {
        let mut min_loss_sol = (self.prob.save(), self.ct.save());
        let reference = (enable_early && sol_listener.is_some()).then(|| min_loss_sol.0.clone());
        let mut early_certified = None;
        let mut min_loss = self.ct.get_total_loss();

        // capmods: 아이템 수 비례 적응형 반복 한도.
        // 아이템이 많을수록 한 번의 분리에서 풀어야 할 매듭이 많아 개선 사이 간격이
        // 길어진다. 고정 한도(기본 200)는 대형 인스턴스에서 분리를 너무 일찍 포기한다.
        // 여성WSjk(156 items) 벤치마크에서 한도 400이 +1.1%p 밀도 개선을 보였고
        // (60초 기준 77.1% → 78.2%), 소형 인스턴스는 기존 한도가 그대로 유지된다.
        #[cfg(feature = "capmods")]
        let iter_no_imprv_limit =
            self.config
                .iter_no_imprv_limit
                .max(if self.config.iter_floor_override > 0 {
                    self.config.iter_floor_override
                } else {
                    self.prob.layout().placed_items().len() * 5 / 2
                });
        #[cfg(not(feature = "capmods"))]
        let iter_no_imprv_limit = self.config.iter_no_imprv_limit;

        let mut n_strikes = 0;
        // 정체 재출발 (`SeparatorConfig::restart_window`): 마지막 개선/재출발 이후 라운드 수.
        let restart_window = self.config.restart_window;
        let mut n_since_restart = 0usize;

        'outer: while n_strikes < self.config.strike_limit && !term.kill() {
            let mut n_iter_no_improvement = 0;

            let initial_strike_loss = self.ct.get_total_loss();

            while n_iter_no_improvement < iter_no_imprv_limit {
                // 중단 요청은 여기서도 확인해야 한다. 이 루프는 개선이 없어도
                // `iter_no_imprv_limit`회(대형 인스턴스는 수백 회) 계속 돌기 때문에,
                // 바깥 strike 루프에서만 확인하면 정지까지 수 초가 걸린다.
                // 여기서 빠져나가면 아래 strike 마무리(rollback)를 정상적으로 거친 뒤
                // 바깥 루프 조건이 종료를 확정한다.
                if term.kill() {
                    break;
                }

                if enable_early && let Some(listener) = sol_listener.as_deref_mut() {
                    self.move_items_round(
                        term,
                        Some(listener),
                        reference.as_ref(),
                        &mut early_certified,
                    );
                } else {
                    self.move_items_multi(term);
                }
                let loss = self.ct.get_total_loss();

                if loss == 0.0 {
                    min_loss_sol.0 = self.prob.save();
                    min_loss_sol.1.copy_from(&self.ct);
                    break 'outer;
                } else if loss < min_loss {
                    // One snapshot serves both the callback and rollback incumbent. Previously
                    // listener-enabled runs saved the complete layout twice per improvement.
                    let improved_solution = self.prob.save();
                    if let Some(l) = sol_listener.as_deref_mut() {
                        l.report(
                            ReportType::ExplImproving,
                            &improved_solution,
                            &self.instance,
                        );
                    }
                    if loss < min_loss * 0.98 {
                        n_iter_no_improvement = 0;
                    }
                    min_loss_sol.0 = improved_solution;
                    min_loss_sol.1.copy_from(&self.ct);
                    min_loss = loss;
                    n_since_restart = 0;
                } else {
                    n_iter_no_improvement += 1;
                    n_since_restart += 1;
                }

                self.ct.update_weights_with(self.config.gls);

                // 정체 재출발: 창 안에 개선이 없으면 인컴번트(배치 + 그 시점의 GLS
                // 가중치)로 되돌아가 다른 난수 궤적으로 계속한다. strike 판정용
                // `n_iter_no_improvement` 는 건드리지 않는다.
                if restart_window > 0
                    && n_since_restart >= restart_window
                    && n_iter_no_improvement < iter_no_imprv_limit
                {
                    self.rollback(&min_loss_sol.0, Some(&min_loss_sol.1));
                    n_since_restart = 0;
                }
            }

            if initial_strike_loss * 0.98 <= min_loss {
                n_strikes += 1;
            } else {
                n_strikes = 0;
            }
            self.rollback(&min_loss_sol.0, Some(&min_loss_sol.1));
        }

        if self.original_guard.is_some() && min_loss_sol.1.get_total_loss() == 0.0 && !term.kill() {
            self.rollback(&min_loss_sol.0, Some(&min_loss_sol.1));
            if self.repair_original_overlap(term) {
                min_loss_sol = (self.prob.save(), self.ct.save());
            }
        }
        // Callback-triggered Stop and a tied surrogate winner must not discard
        // a result already certified and published during this round.
        if let Some(early) = early_certified {
            min_loss_sol = self.select_returned_incumbent(
                min_loss_sol,
                early,
                reference.as_ref().unwrap(),
                term.kill(),
            );
        }
        (min_loss_sol.0, min_loss_sol.1)
    }

    fn select_returned_incumbent(
        &self,
        normal: (SPSolution, CTSnapshot),
        early: (SPSolution, CTSnapshot),
        reference: &SPSolution,
        interrupted: bool,
    ) -> (SPSolution, CTSnapshot) {
        if early.0.strip_width() < normal.0.strip_width()
            || (interrupted && early.0.strip_width() <= normal.0.strip_width())
            || !self.solution_is_certified(&normal.0, reference)
        {
            early
        } else {
            normal
        }
    }

    fn move_items_multi(&mut self, term: &impl Terminator) {
        self.move_items_round(term, None, None, &mut None)
    }

    /// `reference` is any snapshot with the same frozen poses as the live
    /// state (frozen items never move within a separation); with it, early
    /// certification needs no fresh master snapshot for a single worker.
    fn move_items_round(
        &mut self,
        term: &impl Terminator,
        listener: Option<&mut dyn SolutionListener>,
        reference: Option<&SPSolution>,
        early_certified: &mut Option<(SPSolution, CTSnapshot)>,
    ) {
        // A single worker owns the live state directly. Peers and early
        // certification need snapshots, but worker 0 never restores from them.
        let master_sol = (self.workers.len() > 1 || (listener.is_some() && reference.is_none()))
            .then(|| self.prob.save());
        // For small parallel layouts an extra tracker snapshot costs more than
        // one restore. Use direct ownership for a single worker or large layouts.
        let direct = self.workers.len() == 1 || self.prob.layout().placed_items().len() >= 64;
        let master_ct = (direct && self.workers.len() > 1).then(|| self.ct.save());
        if direct {
            std::mem::swap(&mut self.prob, &mut self.workers[0].prob);
            std::mem::swap(&mut self.ct, &mut self.workers[0].ct);
        }

        let ct = master_ct.as_ref().unwrap_or(&self.ct);
        let frozen_mask = &self.frozen_mask;
        let place_groups = &self.place_groups;
        let instance = &self.instance;
        let report_early = listener.is_some();
        let work = |(index, worker): (usize, &mut SeparatorWorker)| {
            if index > 0 || !direct {
                worker.load(master_sol.as_ref().unwrap(), ct);
            }
            worker.frozen_mask.clone_from(frozen_mask);
            if &worker.place_groups != place_groups {
                worker.place_groups.clone_from(place_groups);
            }
            worker.move_items(term);
            (report_early && worker.ct.get_total_loss() == 0.0)
                .then(|| (worker.prob.save(), worker.ct.save()))
        };

        if let Some(listener) = listener {
            let frozen_reference = reference.unwrap_or_else(|| master_sol.as_ref().unwrap());
            let mut deliver = |candidate: (SPSolution, CTSnapshot)| {
                if early_certified
                    .as_ref()
                    .is_some_and(|best| best.0.strip_width() <= candidate.0.strip_width())
                {
                    return;
                }
                if place_groups.solution_preserves_relative_poses(&candidate.0)
                    && crate::util::assertions::frozen_poses_preserved(
                        frozen_reference,
                        &candidate.0,
                        frozen_mask,
                    )
                    && crate::structure::pipeline::solution_validation_error_with_frozen_and_groups(
                        instance,
                        &candidate.0,
                        frozen_mask,
                        place_groups,
                    )
                    .is_none()
                {
                    // Commit before entering external code, which may request Stop.
                    *early_certified = Some(candidate);
                    listener.report_certified_early(&early_certified.as_ref().unwrap().0, instance);
                }
            };
            match &self.thread_pool {
                Some(pool) => {
                    let mut indexed = self.workers.iter_mut().enumerate().collect_vec();
                    early_report::run_round(
                        pool,
                        &mut indexed,
                        |(index, worker)| ((), work((*index, &mut **worker))),
                        deliver,
                    );
                }
                None => {
                    for worker in self.workers.iter_mut().enumerate() {
                        if let Some(candidate) = work(worker) {
                            deliver(candidate);
                        }
                    }
                }
            }
        } else {
            match &self.thread_pool {
                Some(pool) => pool.install(|| {
                    self.workers.par_iter_mut().enumerate().for_each(|worker| {
                        work(worker);
                    })
                }),
                None => self.workers.iter_mut().enumerate().for_each(|worker| {
                    work(worker);
                }),
            }
        }

        let best_idx = self
            .workers
            .iter()
            .map(|opt| OrderedFloat(opt.ct.get_total_weighted_loss()))
            .position_min()
            .unwrap();

        // Transfer the matching problem/tracker together. Worker RNG and NFP
        // caches remain worker-local; the next iteration reloads every worker.
        let best = &mut self.workers[best_idx];
        std::mem::swap(&mut self.prob, &mut best.prob);
        std::mem::swap(&mut self.ct, &mut best.ct);
    }

    // Exact pre-change round for the opt-in fixed-work benchmark.
    #[cfg(test)]
    fn move_items_round_legacy_benchmark(
        &mut self,
        term: &impl Terminator,
        listener: Option<&mut dyn SolutionListener>,
        early_certified: &mut Option<(SPSolution, CTSnapshot)>,
    ) {
        let master_sol = self.prob.save();

        let ct = &self.ct;
        let frozen_mask = &self.frozen_mask;
        let place_groups = &self.place_groups;
        let instance = &self.instance;
        let report_early = listener.is_some();
        let work = |worker: &mut SeparatorWorker| {
            worker.load(&master_sol, ct);
            worker.frozen_mask.clone_from(frozen_mask);
            if &worker.place_groups != place_groups {
                worker.place_groups.clone_from(place_groups);
            }
            worker.move_items(term);
            let candidate = (report_early && worker.ct.get_total_loss() == 0.0)
                .then(|| (worker.prob.save(), worker.ct.save()));
            ((), candidate)
        };

        if let Some(listener) = listener {
            let mut deliver = |candidate: (SPSolution, CTSnapshot)| {
                if early_certified
                    .as_ref()
                    .is_some_and(|best| best.0.strip_width() <= candidate.0.strip_width())
                {
                    return;
                }
                if place_groups.solution_preserves_relative_poses(&candidate.0)
                    && crate::util::assertions::frozen_poses_preserved(
                        &master_sol,
                        &candidate.0,
                        frozen_mask,
                    )
                    && crate::structure::pipeline::solution_validation_error_with_frozen_and_groups(
                        instance,
                        &candidate.0,
                        frozen_mask,
                        place_groups,
                    )
                    .is_none()
                {
                    // Commit before entering external code, which may request Stop.
                    *early_certified = Some(candidate);
                    listener.report_certified_early(&early_certified.as_ref().unwrap().0, instance);
                }
            };
            match &self.thread_pool {
                Some(pool) => {
                    early_report::run_round(pool, &mut self.workers, work, deliver);
                }
                None => {
                    for worker in self.workers.iter_mut() {
                        if let (_, Some(candidate)) = work(worker) {
                            deliver(candidate);
                        }
                    }
                }
            }
        } else {
            let mut separate_multi = || {
                self.workers.par_iter_mut().for_each(|worker| {
                    work(worker);
                })
            };
            match &self.thread_pool {
                Some(pool) => pool.install(&mut separate_multi),
                None => separate_multi(),
            }
        }

        let best_idx = self
            .workers
            .iter()
            .map(|opt| OrderedFloat(opt.ct.get_total_weighted_loss()))
            .position_min()
            .unwrap();

        // Transfer the matching problem/tracker together. Worker RNG and NFP
        // caches remain worker-local; the next iteration reloads every worker.
        let best = &mut self.workers[best_idx];
        std::mem::swap(&mut self.prob, &mut best.prob);
        std::mem::swap(&mut self.ct, &mut best.ct);
    }

    pub fn rollback(&mut self, sol: &SPSolution, ots: Option<&CTSnapshot>) {
        // SPProblem::restore installs the target container and layout together.
        // Shrinking the old layout first can strand frozen items outside the CDE.
        crate::util::frozen::restore(&mut self.prob, sol);

        match ots {
            Some(ots) => {
                self.ct.restore_but_keep_weights(ots, &self.prob.layout());
            }
            None => {
                self.ct = CollisionTracker::new_with_frozen_prepared_and_groups(
                    &self.prob.layout(),
                    &self.frozen_mask,
                    &self.prepared,
                    &self.place_groups,
                );
            }
        }
    }

    pub fn move_item(&mut self, pk: PItemKey, dt: DTransformation) -> PItemKey {
        debug_assert!(tracker_matches_layout(&self.ct, &self.prob.layout()));
        let result = self.move_item_inner(pk, dt);
        debug_assert!(tracker_matches_layout(&self.ct, &self.prob.layout()));
        result
    }

    /// Apply a batch with a consistent tracker after every move. The caller must
    /// restore its snapshot if cancellation interrupts a transactional proposal.
    pub(crate) fn move_items_with_control(
        &mut self,
        moves: &[(PItemKey, DTransformation)],
        should_stop: &mut dyn FnMut() -> bool,
    ) -> bool {
        debug_assert!(tracker_matches_layout(&self.ct, &self.prob.layout()));
        for &(pk, dt) in moves {
            if should_stop() {
                return false;
            }
            self.move_item_inner(pk, dt);
        }
        debug_assert!(tracker_matches_layout(&self.ct, &self.prob.layout()));
        !should_stop()
    }

    fn move_item_inner(&mut self, pk: PItemKey, d_transf: DTransformation) -> PItemKey {
        let item_id = self.prob.layout().placed_items()[pk].item().idx();
        if item_id < self.frozen_mask.len() && self.frozen_mask[item_id] {
            return pk;
        }

        // Preserve the exact legacy behavior, including support for item kinds
        // with demand > 1 (where item_id cannot identify a placement key).
        if self.place_groups.is_empty() {
            self.prob.remove_item(pk);
            let new_pk = self.prob.place_item(SPPlacement { d_transf, item_idx: item_id });
            self.ct.register_item_move(&self.prob.layout(), pk, new_pk);
            return new_pk;
        }

        // Resolve every key and every target before mutating the problem. This
        // makes a group move transactional from the caller's perspective: an
        // invalid/partial group cannot leave half of its members moved.
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

        selected_new_pk
    }

    pub fn change_strip_width(&mut self, new_width: f32, split_position: Option<f32>) {
        self.change_strip_width_with_control(new_width, split_position, &mut || false);
    }

    /// A cancelled width proposal restores the original layout before returning.
    pub fn change_strip_width_with_control(
        &mut self,
        new_width: f32,
        split_position: Option<f32>,
        should_stop: &mut dyn FnMut() -> bool,
    ) -> bool {
        if !new_width.is_finite() || new_width <= 0.0 || should_stop() {
            return false;
        }
        let split_position = split_position.unwrap_or(self.prob.strip_width() / 2.0);
        let delta = new_width - self.prob.strip_width();
        let saved = self.prob.save();
        let shifted = if self.place_groups.is_empty() {
            let moves = self
                .prob
                .layout()
                .placed_items()
                .iter()
                .filter(|(_, pi)| pi.shape().centroid().0 > split_position)
                .map(|(pk, pi)| {
                    (
                        pk,
                        pi.d_transf().compose().translate((delta, 0.0)).decompose(),
                    )
                })
                .collect_vec();
            // `resync()` rebuilds the tracker right after the shift, so the
            // per-move tracker update of `move_items_with_control` is wasted
            // here. Same frozen handling and cancellation checks as there.
            let mut completed = true;
            for (pk, d_transf) in moves {
                if should_stop() {
                    completed = false;
                    break;
                }
                let item_id = self.prob.layout().placed_items()[pk].item().idx();
                if item_id < self.frozen_mask.len() && self.frozen_mask[item_id] {
                    continue;
                }
                self.prob.remove_item(pk);
                self.prob.place_item(SPPlacement { d_transf, item_idx: item_id });
            }
            completed && !should_stop()
        } else {
            let moves = self
                .place_groups
                .placed_unit_representatives(&self.prob.layout())
                .into_iter()
                .filter_map(|unit_id| {
                    let keys = self
                        .place_groups
                        .member_item_ids(unit_id)
                        .into_iter()
                        .filter_map(|id| self.placement_key(id))
                        .collect_vec();
                    let x_min = keys
                        .iter()
                        .map(|pk| self.prob.layout().placed_items()[*pk].shape().bbox().x_min)
                        .fold(f32::INFINITY, f32::min);
                    let x_max = keys
                        .iter()
                        .map(|pk| self.prob.layout().placed_items()[*pk].shape().bbox().x_max)
                        .fold(f32::NEG_INFINITY, f32::max);
                    let pk = self.placement_key(unit_id)?;
                    ((x_min + x_max) * 0.5 > split_position).then(|| {
                        (
                            pk,
                            self.prob.layout().placed_items()[pk]
                                .d_transf()
                                .compose()
                                .translate((delta, 0.0))
                                .decompose(),
                        )
                    })
                })
                .collect_vec();
            self.move_items_with_control(&moves, should_stop)
        };
        if !shifted {
            self.rollback(&saved, None);
            return false;
        }
        crate::util::frozen::change_strip_width(&mut self.prob, new_width);
        self.resync();
        true
    }

    /// 아이템을 전혀 이동시키지 않고 strip 폭만 현재 배치에 맞춰 축소한다.
    /// [`Self::change_strip_width`]와 달리 split position 기준의 시프트가 없으므로,
    /// 이미 feasible한 배치를 그대로 둔 채 남는 폭만 회수할 때 사용한다.
    /// 폭이 실제로 바뀐 경우 `true`를 반환한다.
    pub fn refit_strip(&mut self) -> bool {
        if self.prob.layout().placed_items().is_empty() {
            return false;
        }
        // Retain the existing fast path when no lock needs an exception.
        //
        // 쿼드트리 판정(`is_feasible`)은 접촉도 충돌로 센다. 인증은 접촉(<= 1e-4mm)을 허용하므로,
        // 조각을 접촉까지 미는 구성기나 폴리시가 만든 배치는 «인증은 통과하는데 쿼드트리는 거짓»
        // 이 될 수 있다. 그때 여기서 조용히 물러나면 호출자는 폭이 그대로인 이유를 알 수 없다
        // (LP 는 걸음을 «이득 없음» 으로 센다). 빠른 경로가 거짓일 때만 인증 기준으로 다시 본다.
        let certified = |sep: &Self| {
            crate::structure::pipeline::solution_validation_error_with_frozen_and_groups(
                &sep.instance,
                &sep.prob.save(),
                &sep.frozen_mask,
                &sep.place_groups,
            )
            .is_none()
        };
        let feasible = if self.frozen_mask.iter().any(|&locked| locked) {
            certified(self)
        } else {
            self.prob.layout().is_collision_free() || certified(self)
        };
        if !feasible {
            return false;
        }
        let before = self.prob.strip_width();
        crate::util::frozen::fit_strip_with_frozen(&mut self.prob, &self.frozen_mask);
        let after = self.prob.strip_width();
        if after == before {
            return false;
        }
        self.resync();
        true
    }

    /// 컨테이너/레이아웃이 바뀐 뒤 collision tracker와 worker들을 현재 상태로 재구축한다.
    /// worker를 통째로 새로 만들지 않고 기존 필드(인스턴스, 버퍼)를 재사용해 갱신한다.
    fn resync(&mut self) {
        self.ct = CollisionTracker::new_with_frozen_prepared_and_groups(
            &self.prob.layout(),
            &self.frozen_mask,
            &self.prepared,
            &self.place_groups,
        );

        for w in self.workers.iter_mut() {
            w.rng = Xoshiro256PlusPlus::seed_from_u64(self.rng.random());
            w.sample_config = self.config.sample_config;
            w.frozen_mask.clone_from(&self.frozen_mask);
            w.place_groups = self.place_groups.clone();
        }
    }
}
