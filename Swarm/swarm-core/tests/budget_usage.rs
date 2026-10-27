//! 각 단계가 **부여받은 예산 대비 실제로 얼마나 쓰는지** 재는 하네스.
//!
//! # 무엇을 묻나
//!
//! "네스팅이 최대 시간에 도달하기 전에 끝나는가?" 소스에는 시계와 무관한 종료
//! 조건이 여럿 있다(수렴 판정, 연속 실패 상한, 라운드 상한, 축소 스텝 하한).
//! 그것들이 **실제로** 언제 먼저 걸리는지는 재 봐야 안다.
//!
//! 단계를 하나씩 직접 불러(step API 와 같은 순서) 각각의 소요를 잰다.
//! `optimize()` 로 한 번에 돌리면 경계 시각을 알 수 없다.
//!
//! ```powershell
//! $env:SWARM_INPUT='C:\Caplanet\AiWorks\InputData\여성WSjk_비교용-G_New_SwarmV1.json'
//! cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" `
//!   --test budget_usage -- --ignored --nocapture --test-threads=1
//! ```
//!
//! 예산은 `BUDGET_EXPLORE_SECS` / `BUDGET_COMPRESS_SECS` / `BUDGET_POLISH_SECS`
//! 로 바꾼다(기본 60 / 30 / 20).

use jagua_rs::probs::spp::entities::{SPInstance, SPProblem};
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::optimizer::separator::Separator;
use swarm_core::util::io::read_spp_input;
use swarm_core::util::terminator::{BasicTerminator, Terminator};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

fn env_secs(key: &str, default: u64) -> u64 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

fn input_path() -> PathBuf {
    match std::env::var("SWARM_INPUT") {
        Ok(p) => PathBuf::from(p),
        Err(_) => Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../data/input")
            .join("wsjk156.json"),
    }
}

fn load(path: &Path) -> SPInstance {
    let (ext, _) = read_spp_input(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let c = DEFAULT_SWARM_CONFIG;
    let imp = swarm_core::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&imp, &ext).expect("import")
}

fn line(name: &str, budget: Duration, used: Duration, note: &str) {
    let b = budget.as_secs_f32();
    let u = used.as_secs_f32();
    let pct = if b > 0.0 { u / b * 100.0 } else { f32::NAN };
    let verdict = if pct < 90.0 { "◀ 예산 전에 끝남" } else { "" };
    println!("  {name:10}  예산 {b:6.1}s  사용 {u:6.1}s  ({pct:5.1}%)  {verdict}  {note}");
}

#[test]
#[ignore = "실측 하네스 — 예산만큼 실제로 돈다"]
fn phases_report_how_much_of_their_budget_they_actually_use() {
    let path = input_path();
    let instance = load(&path);
    let mut c = DEFAULT_SWARM_CONFIG;
    c.expl_cfg.time_limit = Duration::from_secs(env_secs("BUDGET_EXPLORE_SECS", 60));
    c.cmpr_cfg.time_limit = Duration::from_secs(env_secs("BUDGET_COMPRESS_SECS", 30));
    c.polish_cfg.time_limit = Duration::from_secs(env_secs("BUDGET_POLISH_SECS", 20));
    // `n_rounds` 는 최소 라운드 수다. 올려서 "예산이 남았는데 멈춘" 구간을
    // 계속 돌게 하면, 그 조기 종료가 품질을 깎는지 잴 수 있다.
    c.polish_cfg.n_rounds = env_secs("BUDGET_POLISH_ROUNDS", 3) as usize;
    // 결정성을 위해 워커 1개. 병렬이면 스케줄 잡음이 측정에 섞인다.
    c.expl_cfg.separator_config.n_workers = 1;
    c.cmpr_cfg.separator_config.n_workers = 1;

    println!("\n══════ 단계별 예산 소진율 ══════");
    println!("  입력    {}", path.display());
    println!(
        "  조각    {}개 / seed 0 / 워커 1",
        instance.total_item_qty()
    );
    println!("  판정    사용률 90% 미만이면 '예산 전에 끝남' 으로 표시한다\n");

    // ── 초기 배치 (예산 없음 — 참고용) ──
    let t = Instant::now();
    let lbf = swarm_core::optimizer::lbf::LBFBuilder::new(
        instance.clone(),
        Xoshiro256PlusPlus::seed_from_u64(0),
        swarm_core::consts::LBF_SAMPLE_CONFIG,
        c.lbf_x_multiplier,
        c.lbf_y_multiplier,
    )
    .with_raster_config(c.lbf_raster)
    .construct()
    .prob
    .save();
    println!(
        "  {:10}  예산    없음   사용 {:6.1}s          폭 {:.2}",
        "초기배치",
        t.elapsed().as_secs_f32(),
        lbf.strip_width()
    );

    let mut prob = SPProblem::new(instance.clone()).expect("strip forms a valid container");
    prob.restore(&lbf);
    let mut sep = Separator::new(
        instance.clone(),
        prob,
        Xoshiro256PlusPlus::seed_from_u64(1),
        c.expl_cfg.separator_config,
    );

    // ── 탐색 ──
    let mut term = BasicTerminator::new();
    term.new_timeout(c.expl_cfg.time_limit);
    let t = Instant::now();
    let feasible =
        swarm_core::optimizer::explore::exploration_phase(&instance, &mut sep, &mut None, &term, &c.expl_cfg);
    let expl_used = t.elapsed();
    let expl_sol = feasible.last().cloned().unwrap_or(lbf.clone());
    line(
        "탐색",
        c.expl_cfg.time_limit,
        expl_used,
        &format!("폭 {:.2}  실행가능해 {}개", expl_sol.strip_width(), feasible.len()),
    );

    // ── 압축 ──
    sep.change_strip_width(expl_sol.strip_width(), None);
    sep.rollback(&expl_sol, None);
    let mut term = BasicTerminator::new();
    term.new_timeout(c.cmpr_cfg.time_limit);
    let t = Instant::now();
    let cmpr_sol = swarm_core::optimizer::compress::compression_phase(
        &instance,
        &mut sep,
        &expl_sol,
        &mut None,
        &term,
        &c.cmpr_cfg,
    );
    let cmpr_used = t.elapsed();
    line(
        "압축",
        c.cmpr_cfg.time_limit,
        cmpr_used,
        &format!("폭 {:.2}", cmpr_sol.strip_width()),
    );

    // ── 폴리시: 같은 시작 레이아웃으로 n_rounds A/B ──
    //
    // 시간 예산 기반이라 실행마다 압축 결과가 달라진다. 팔마다 새로 만들면
    // 속도 차이가 배치 차이로 번져 무엇을 재는지 알 수 없다 — `cmpr_sol` 하나를
    // 두 팔에 똑같이 먹인다.
    let mut plsh_used = Duration::ZERO;
    let mut plsh_sol = cmpr_sol.clone();
    // 폴리시가 예산을 안 쓰는 이유가 "정말 할 일이 없어서" 인지 "내부 패스의 자체
    // 상한에 걸려서" 인지 가른다. 라운드 수만 늘리는 것으로는 구분되지 않는다 —
    // 매 라운드가 같은 상한에 같은 방식으로 걸리면 결과도 같기 때문이다.
    type Arm = (
        String,
        swarm_core::config::PolishConfig,
        swarm_core::config::NfpIntegrationConfig,
    );
    let sweep: Vec<Arm> = {
        let base = c.polish_cfg.clone();
        let mut v = vec![("기본".to_string(), base.clone(), c.nfp_integration)];
        let mut more_rounds = base.clone();
        more_rounds.n_rounds = 100;
        v.push(("라운드 100".to_string(), more_rounds, c.nfp_integration));
        let mut more_lp = base.clone();
        more_lp.n_rounds = 100;
        more_lp.lp.n_iterations = 400;
        v.push(("+ LP 반복 400".to_string(), more_lp, c.nfp_integration));
        let mut bigger_trust = base.clone();
        bigger_trust.n_rounds = 100;
        bigger_trust.lp.n_iterations = 400;
        bigger_trust.lp.trust_region_ratio = base.lp.trust_region_ratio * 4.0;
        v.push(("+ 신뢰영역 x4".to_string(), bigger_trust, c.nfp_integration));
        let mut more_slide = base.clone();
        more_slide.n_rounds = 100;
        more_slide.lp.n_iterations = 400;
        more_slide.slide.n_passes = 30;
        more_slide.slide.min_move_ratio = base.slide.min_move_ratio / 10.0;
        v.push(("+ slide 30회·미세이동".to_string(), more_slide, c.nfp_integration));
        // 검토가 지목한 대립 가설: 폴리시가 일찍 멈추는 건 시간이 아니라 **홀 탐색기**
        // 때문일 수 있다. 기본 raster 격자는 가장 작은 조각 단변의 1/4 보다 얇은
        // 포켓을 구조적으로 못 본다. 정확 NFP 홀 채움(`nfp_hole_filling`, 기본 꺼짐)을
        // 켜서 고정점이 움직이는지 본다 — 움직이면 예산 문제가 아니다.
        let mut nfp_holes = base.clone();
        nfp_holes.n_rounds = 100;
        let mut nfp_cfg = c.nfp_integration;
        nfp_cfg.nfp_hole_filling = true;
        v.push(("NFP 홀채움 ON".to_string(), nfp_holes, nfp_cfg));

        let mut all = base.clone();
        all.n_rounds = 100;
        all.lp.n_iterations = 400;
        all.lp.trust_region_ratio = base.lp.trust_region_ratio * 4.0;
        all.slide.n_passes = 30;
        all.slide.min_move_ratio = base.slide.min_move_ratio / 10.0;
        all.hole_fill.max_candidates = base.hole_fill.max_candidates * 4;
        all.hole_fill.n_samples_per_pocket = base.hole_fill.n_samples_per_pocket * 4;
        v.push(("전부 완화".to_string(), all, c.nfp_integration));
        v
    };
    for (arm, (name, cfg, nfp_cfg)) in sweep.iter().enumerate() {
        let cfg = cfg.clone();
        sep.change_strip_width(cmpr_sol.strip_width(), None);
        sep.rollback(&cmpr_sol, None);
        let mut term = BasicTerminator::new();
        term.new_timeout(cfg.time_limit);
        let t = Instant::now();
        let (sol, stats) = swarm_core::optimizer::polish::polish_phase_measured(
            &instance,
            &mut sep,
            &cmpr_sol,
            &mut None,
            &term,
            &cfg,
            nfp_cfg,
            &c.lp_solver,
        );
        let used = t.elapsed();
        line(
            &format!("PLSH {name}"),
            cfg.time_limit,
            used,
            &format!(
                "폭 {:.2}  라운드 {}  변경 {}  종료사유 {:?}",
                sol.strip_width(),
                stats.rounds,
                stats.changes,
                stats.exit
            ),
        );
        if arm == 0 {
            plsh_used = used;
            plsh_sol = sol;
        }
    }

    // ── 바깥 루프: 폴리시가 남긴 예산을 압축에 되먹인다 ──
    //
    // 폴리시는 고정점에 도달해 끝나지만, 그 고정점은 **더 좋은 레이아웃**이다.
    // 압축을 그 위에서 다시 돌리면 더 좁은 폭에서 축소 스케줄을 새로 시작하게 된다.
    // 소스만 봐서는 압축이 재진입 가능한지, 재진입이 이득인지 알 수 없다 — 잰다.
    //
    // 정지 보장: 예산이 남아 있고 **직전 사이클이 폭을 실제로 줄인** 동안만 돈다.
    // ── 실제 배선된 회수 루프: 세 모드 비교 ──
    //
    // 같은 압축 결과에서 출발해 폴리시를 돌린 뒤, **같은 폴리시 출력**에 세 모드를
    // 각각 먹인다. 팔마다 새로 만들면 속도 차이가 배치 차이로 번져 오염된다.
    if std::env::var("BUDGET_RECLAIM").is_ok() {
        use swarm_core::config::ReclaimMode;
        for mode in [
            ReclaimMode::CompressReentry,
            ReclaimMode::RuinRecreate,
            ReclaimMode::Escape,
        ] {
        let mut cfg = c.polish_cfg.clone();
        cfg.reclaim.enabled = true;
        cfg.reclaim.mode = mode;
        sep.change_strip_width(cmpr_sol.strip_width(), None);
        sep.rollback(&cmpr_sol, None);
        let mut term = BasicTerminator::new();
        term.new_timeout(cfg.time_limit);
        let deadline = term.timeout_at().expect("폴리시 마감");
        let t = Instant::now();
        let (polished, pstats) = swarm_core::optimizer::polish::polish_phase_measured(
            &instance, &mut sep, &cmpr_sol, &mut None, &term, &cfg,
            &c.nfp_integration, &c.lp_solver,
        );
        let plsh_only = t.elapsed();
        let (reclaimed, rstats) = swarm_core::optimizer::reclaim::reclaim_leftover(
            &instance, &mut sep, polished, deadline, &cfg.reclaim,
            &c.cmpr_cfg, &c.expl_cfg, &cfg, &c.nfp_integration, &c.escape,
            &c.lp_solver, &term,
        );
        let total = t.elapsed();
        println!(
            "  {:18}  폴리시 {:.1}s({:?}) {:.2} -> 회수 {}사이클/{}채택 {:.2}              ({:+.2}mm)  총 {:.1}s/{:.1}s  미사용 {}ms",
            mode.label(),
            plsh_only.as_secs_f32(),
            pstats.exit,
            rstats.width_before,
            rstats.cycles,
            rstats.accepted,
            rstats.width_after,
            rstats.width_after - rstats.width_before,
            total.as_secs_f32(),
            cfg.time_limit.as_secs_f32(),
            rstats.unused_ms
        );
        assert!(
            total <= cfg.time_limit + Duration::from_secs(3),
            "회수가 폴리시 예산을 넘겼다: {:.1}s > {:.1}s",
            total.as_secs_f32(),
            cfg.time_limit.as_secs_f32()
        );
        assert!(
            reclaimed.strip_width() <= rstats.width_before + 1.0e-4,
            "{} 가 폭을 늘렸다",
            mode.label()
        );
        }
    }

    if std::env::var("BUDGET_OUTER_LOOP").is_ok() {
        let leftover = c
            .polish_cfg
            .time_limit
            .saturating_sub(plsh_used)
            .saturating_add(c.cmpr_cfg.time_limit.saturating_sub(cmpr_used));
        println!("
  ── 바깥 루프 (남은 예산 {:.1}s 재투입) ──", leftover.as_secs_f32());
        let mut budget_left = leftover;
        let mut best = plsh_sol.clone();
        let mut cycle = 0usize;
        while budget_left > Duration::from_millis(500) {
            cycle += 1;
            // 사이클마다 남은 예산의 절반을 압축에, 나머지를 폴리시에 준다.
            let cmpr_slice = budget_left / 2;
            let plsh_slice = budget_left - cmpr_slice;

            let mut cmpr_cfg = c.cmpr_cfg.clone();
            cmpr_cfg.time_limit = cmpr_slice;
            sep.change_strip_width(best.strip_width(), None);
            sep.rollback(&best, None);
            let mut term = BasicTerminator::new();
            term.new_timeout(cmpr_slice);
            let t = Instant::now();
            let c2 = swarm_core::optimizer::compress::compression_phase(
                &instance, &mut sep, &best, &mut None, &term, &cmpr_cfg,
            );
            let c2_used = t.elapsed();

            let mut plsh_cfg = c.polish_cfg.clone();
            plsh_cfg.time_limit = plsh_slice;
            sep.change_strip_width(c2.strip_width(), None);
            sep.rollback(&c2, None);
            let mut term = BasicTerminator::new();
            term.new_timeout(plsh_slice);
            let t = Instant::now();
            let (p2, p2_stats) = swarm_core::optimizer::polish::polish_phase_measured(
                &instance, &mut sep, &c2, &mut None, &term, &plsh_cfg,
                &c.nfp_integration, &c.lp_solver,
            );
            let p2_used = t.elapsed();

            let improved = p2.strip_width() < best.strip_width() - 1.0e-4;
            println!(
                "  사이클 {cycle}: 압축 {:.1}s -> {:.2}, 폴리시 {:.1}s -> {:.2} ({:?})  {}",
                c2_used.as_secs_f32(),
                c2.strip_width(),
                p2_used.as_secs_f32(),
                p2.strip_width(),
                p2_stats.exit,
                if improved { "개선" } else { "개선 없음 — 중단" }
            );
            let spent = c2_used + p2_used;
            budget_left = budget_left.saturating_sub(spent);
            if p2.strip_width() < best.strip_width() {
                best = p2;
            }
            if !improved {
                break;
            }
        }
        println!(
            "  바깥 루프 결과: {:.2} -> {:.2} ({:+.2}mm), 남은 예산 {:.1}s
",
            plsh_sol.strip_width(),
            best.strip_width(),
            best.strip_width() - plsh_sol.strip_width(),
            budget_left.as_secs_f32()
        );
    }

    let total_budget = c.expl_cfg.time_limit + c.cmpr_cfg.time_limit + c.polish_cfg.time_limit;
    let total_used = expl_used + cmpr_used + plsh_used;
    println!();
    line("합계", total_budget, total_used, "");
    println!(
        "\n  최종 폭 {:.2}  밀도 {:.2}%\n",
        plsh_sol.strip_width(),
        plsh_sol.density() * 100.0
    );

    // 이 하네스는 측정이지 단정이 아니다. 다만 배치가 깨지면 수치가 무의미하므로
    // 그것만 지킨다.
    assert!(plsh_sol.strip_width() <= lbf.strip_width() + 1.0e-3);
}
