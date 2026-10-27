//! 중단(stop) 요청 후 실제로 멈출 때까지 걸리는 시간 측정.
//!
//! 실행:  cargo test --release -p swarm-core --features capmods --test stop_latency -- --nocapture
//!
//! 배경: 중단 플래그는 `Terminator::kill()`로 전파되는데, 이를 확인하지 않는 긴 루프가
//! 하나라도 있으면 그 루프가 끝날 때까지 정지가 지연된다. 특히 separation의 안쪽 반복
//! 루프는 개선이 없어도 `iter_no_imprv_limit`회(대형 인스턴스는 수백 회) 계속 돌기 때문에
//! 여기서 확인하지 않으면 정지에 수 초가 걸린다.

use jagua_rs::probs::spp::entities::SPInstance;
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::optimizer::lbf::LBFBuilder;
use swarm_core::optimizer::separator::Separator;
use swarm_core::util::io::read_spp_input;
use swarm_core::util::terminator::{BasicTerminator, Terminator};
use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

/// 외부에서 플래그로 중단시킬 수 있는 terminator (ffi_step의 `CancellableTerminator`와 동일 구조).
struct FlagTerminator {
    stop: Arc<AtomicBool>,
    inner: BasicTerminator,
}

impl Terminator for FlagTerminator {
    fn kill(&self) -> bool {
        self.stop.load(Ordering::Relaxed) || self.inner.kill()
    }
    fn stop_requested(&self) -> bool {
        self.stop.load(Ordering::Relaxed)
    }
    fn new_timeout(&mut self, timeout: Duration) {
        self.inner.new_timeout(timeout)
    }
    fn timeout_at(&self) -> Option<jagua_rs::Instant> {
        self.inner.timeout_at()
    }
}

fn load(name: &str) -> SPInstance {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(format!("../data/input/{name}.json"));
    let (ext_inst, _) = read_spp_input(&path).expect("failed to read instance");
    let cfg = DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(cfg.cde_config, cfg.poly_simpl_tolerance, cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&importer, &ext_inst)
        .expect("failed to import instance")
}

/// 탐색이 한창일 때 중단을 요청하고, 반환까지 걸린 시간을 잰다.
#[test]
fn exploration_stops_promptly() {
    // 아이템이 많을수록 안쪽 반복 한도가 커져(capmods 적응형 한도) 지연이 잘 드러난다.
    let instance = load("shirts");
    let builder = LBFBuilder::new(
        instance.clone(),
        Xoshiro256PlusPlus::seed_from_u64(0),
        LBF_SAMPLE_CONFIG,
        10.0,
        1.0,
    )
    .construct();

    let expl_cfg = DEFAULT_SWARM_CONFIG.expl_cfg;
    let mut sep = Separator::new(
        instance.clone(),
        builder.prob,
        Xoshiro256PlusPlus::seed_from_u64(1),
        expl_cfg.separator_config,
    );
    // 폭을 조여 separation이 쉽게 성공하지 못하게 만든다 → 안쪽 루프가 한도까지 돈다.
    let w = sep.prob.strip_width();
    sep.change_strip_width(w * 0.80, None);

    let stop = Arc::new(AtomicBool::new(false));
    let mut term = FlagTerminator {
        stop: stop.clone(),
        inner: BasicTerminator::new(),
    };
    // 자체 종료가 아니라 중단 플래그로 멈추는지 보려고 제한 시간은 넉넉히 준다.
    term.new_timeout(Duration::from_secs(120));

    // 탐색이 충분히 진행된 뒤 중단을 요청한다.
    let requested_at = Arc::new(std::sync::Mutex::new(None::<Instant>));
    {
        let stop = stop.clone();
        let requested_at = requested_at.clone();
        std::thread::spawn(move || {
            std::thread::sleep(Duration::from_secs(3));
            *requested_at.lock().unwrap() = Some(Instant::now());
            stop.store(true, Ordering::Relaxed);
        });
    }

    #[cfg(feature = "adaptive")]
    let sols = swarm_core::optimizer::explore_mods::exploration_phase_adaptive(
        &instance, &mut sep, &mut None, &term, &expl_cfg,
    );
    #[cfg(not(feature = "adaptive"))]
    let sols = swarm_core::optimizer::explore::exploration_phase(
        &instance, &mut sep, &mut None, &term, &expl_cfg,
    );
    let returned_at = Instant::now();

    let requested = requested_at
        .lock()
        .unwrap()
        .expect("stop was never requested");
    let latency = returned_at.duration_since(requested);
    println!(
        "stop latency: {:.0} ms ({} solutions)",
        latency.as_secs_f32() * 1000.0,
        sols.len()
    );

    // Entry was deliberately infeasible; cancellation may leave no certified solution.
    assert!(sols.iter().all(|sol| sep.solution_is_certified(sol, sol)));
    // 안쪽 루프에서 중단을 확인하지 않으면 여기서 수 초가 나온다.
    assert!(
        latency < Duration::from_millis(1500),
        "stop took too long: {:.0} ms — 중단을 확인하지 않는 긴 루프가 남아 있다",
        latency.as_secs_f32() * 1000.0
    );
}

/// 정지 플래그를 `sleep` 뒤에 세우는 스레드를 띄우고, 세운 시각을 돌려준다.
fn arm_stop_after(
    stop: &Arc<AtomicBool>,
    delay: Duration,
) -> Arc<std::sync::Mutex<Option<Instant>>> {
    let requested_at = Arc::new(std::sync::Mutex::new(None::<Instant>));
    let stop = stop.clone();
    let at = requested_at.clone();
    std::thread::spawn(move || {
        std::thread::sleep(delay);
        *at.lock().unwrap() = Some(Instant::now());
        stop.store(true, Ordering::Relaxed);
    });
    requested_at
}

/// **폴리시 잔여 회수(`reclaim_leftover`)가 정지 요청을 즉시 보는가.**
///
/// # 실제로 겪은 결함
///
/// 회수는 남은 폴리시 예산을 압축↔폴리시 사이클로 쓴다. 예전에는 사이클마다
/// `BasicTerminator::new()` 에 시간만 걸어 안쪽에 넘겼고, 바깥 정지 플래그는 사이클
/// **시작** 시점에만 봤다. 그래서 정지가 사이클 도중에 들어오면 조각 시간(남은 예산의
/// 75%)이 다할 때까지 안 멈췄다 — 폴리시 예산이 클수록 길어졌다.
///
/// 여기서는 마감을 60초 뒤로 두고(압축 조각 45초), 사이클이 돌기 시작한 2초 뒤에
/// 정지를 넣는다. 고치기 전에는 ~45초, 고친 뒤에는 separate 한 iteration 안이다.
#[test]
fn reclaim_stops_promptly() {
    let instance = load("shirts");
    let c = DEFAULT_SWARM_CONFIG;
    let seed = LBFBuilder::new(
        instance.clone(),
        Xoshiro256PlusPlus::seed_from_u64(0),
        LBF_SAMPLE_CONFIG,
        c.lbf_x_multiplier,
        c.lbf_y_multiplier,
    )
    .construct()
    .prob
    .save();
    let mut prob = jagua_rs::probs::spp::entities::SPProblem::new(instance.clone()).expect("strip forms a valid container");
    prob.restore(&seed);
    let mut sep = Separator::new(
        instance.clone(),
        prob,
        Xoshiro256PlusPlus::seed_from_u64(1),
        c.cmpr_cfg.separator_config,
    );

    let stop = Arc::new(AtomicBool::new(false));
    let mut outer = FlagTerminator {
        stop: stop.clone(),
        inner: BasicTerminator::new(),
    };
    // 바깥 폴리시 시계는 회수 시작 시점에 이미 만료돼 있는 것이 실제 상황이다 —
    // 회수는 `outer.kill()` 이 아니라 `outer.stop_requested()` 만 봐야 한다.
    outer.new_timeout(Duration::ZERO);
    let deadline = Instant::now() + Duration::from_secs(60);
    let reclaim_cfg = c.polish_cfg.reclaim; // 기본 켜짐, CompressReentry, min_slice 500ms
    assert!(
        reclaim_cfg.enabled,
        "기본값이 꺼짐이면 이 테스트는 아무것도 재지 않는다"
    );

    let requested_at = arm_stop_after(&stop, Duration::from_secs(2));
    let (_best, stats) = swarm_core::optimizer::reclaim::reclaim_leftover(
        &instance,
        &mut sep,
        seed,
        deadline,
        &reclaim_cfg,
        &c.cmpr_cfg,
        &c.expl_cfg,
        &c.polish_cfg,
        &c.nfp_integration,
        &c.escape,
        &c.lp_solver,
        &outer,
    );
    let returned_at = Instant::now();
    let requested = requested_at
        .lock()
        .unwrap()
        .expect("stop was never requested");
    let latency = returned_at.duration_since(requested);
    println!(
        "reclaim stop latency: {:.0} ms (사이클 {}, 미사용 {} ms)",
        latency.as_secs_f32() * 1000.0,
        stats.cycles,
        stats.unused_ms
    );
    assert!(
        stats.cycles >= 1,
        "회수 사이클이 시작되기 전에 끝났다 — 정지 경로를 재지 못했다"
    );
    assert!(
        latency < Duration::from_millis(1500),
        "reclaim stop took too long: {:.0} ms — 조각 터미네이터가 바깥 정지 플래그를 안 본다",
        latency.as_secs_f32() * 1000.0
    );
}

/// Microlp resumes in short slices and movement batches poll the stop flag.
/// Check the same cancellation envelope as Explore/Reclaim on this fixture.
#[test]
fn polish_stops_promptly() {
    let instance = load("shirts");
    let c = DEFAULT_SWARM_CONFIG;
    let seed = LBFBuilder::new(
        instance.clone(),
        Xoshiro256PlusPlus::seed_from_u64(0),
        LBF_SAMPLE_CONFIG,
        c.lbf_x_multiplier,
        c.lbf_y_multiplier,
    )
    .construct()
    .prob
    .save();
    let mut prob = jagua_rs::probs::spp::entities::SPProblem::new(instance.clone()).expect("strip forms a valid container");
    prob.restore(&seed);
    let mut sep = Separator::new(
        instance.clone(),
        prob,
        Xoshiro256PlusPlus::seed_from_u64(1),
        c.cmpr_cfg.separator_config,
    );

    let stop = Arc::new(AtomicBool::new(false));
    let mut term = FlagTerminator {
        stop: stop.clone(),
        inner: BasicTerminator::new(),
    };
    term.new_timeout(Duration::from_secs(60));
    let mut plsh = c.polish_cfg.clone();
    plsh.time_limit = Duration::from_secs(60);
    plsh.n_rounds = 1; // Legacy round count must not bypass the stop path at a fixed point.
    plsh.reclaim.enabled = false; // 회수는 위 테스트가 따로 잰다

    let requested_at = arm_stop_after(&stop, Duration::from_secs(1));
    let _ = swarm_core::optimizer::polish::polish_phase_with_nfp(
        &instance,
        &mut sep,
        &seed,
        &mut None,
        &term,
        &plsh,
        &c.nfp_integration,
        &c.lp_solver,
    );
    let returned_at = Instant::now();
    let requested = requested_at
        .lock()
        .unwrap()
        .expect("active Polish returned before the stop request");
    let latency = returned_at.duration_since(requested);
    println!(
        "polish stop latency: {:.0} ms",
        latency.as_secs_f32() * 1000.0
    );
    let lp_bound = Duration::from_millis(1500);
    assert!(
        latency < lp_bound,
        "polish stop took {:.0} ms — 중단 응답 기준({:?})을 넘었다",
        latency.as_secs_f32() * 1000.0,
        lp_bound
    );
}
