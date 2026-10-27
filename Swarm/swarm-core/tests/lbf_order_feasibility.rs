//! LBF 배치 순서 모드별로 **초기 해가 실제로 겹침 없는지** 검증하는 하네스.
//!
//! # 왜 필요한가
//!
//! 순서 모드를 도입할 때는 폭·밀도만 쟀고 **실행 가능성은 재지 않았다**.
//! LBF 는 후보를 CDE 로 검증하고 채택하므로 원리상 겹침이 없어야 하지만,
//! 그건 검증이 아니라 가정이다. 여기서 두 오라클로 직접 확인한다:
//!
//! * `CollisionTracker::get_total_loss()` — 솔버가 쓰는 판정 (jagua CDE).
//! * `flat_layout_collision_counts` — 인증기·콜백 진단이 쓰는 판정
//!   (양방향 다각형 재검사).
//!
//! 배치 **수량**도 함께 본다. 겹침이 없어도 조각을 다 못 놓으면 그건 다른 결함이다.
//!
//! ```powershell
//! $env:SWARM_INPUT='C:\Caplanet\AiWorks\InputData\여성WSjk_비교용-G_New_SwarmV1.json'
//! cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" `
//!   --test lbf_order_feasibility -- --ignored --nocapture --test-threads=1
//! ```

use jagua_rs::probs::spp::entities::SPInstance;
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::eval::lbf_evaluator::flat_layout_collision_counts;
use swarm_core::optimizer::lbf::{LBFBuilder, LbfOrderMode};
use swarm_core::optimizer::separator::Separator;
use swarm_core::util::io::read_spp_input;
use std::path::{Path, PathBuf};

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

#[test]
#[ignore = "실측 하네스 — 여섯 모드를 모두 구성한다"]
fn every_lbf_order_mode_produces_a_collision_free_initial_layout() {
    let path = input_path();
    let instance = load(&path);
    let c = DEFAULT_SWARM_CONFIG;
    let n_items = instance.total_item_qty();
    let seeds: Vec<u64> = std::env::var("ORDER_SEEDS")
        .unwrap_or_else(|_| "0,1,2".into())
        .split(',')
        .filter_map(|v| v.trim().parse().ok())
        .collect();

    println!("\n══════ LBF 배치 순서별 초기 해 실행 가능성 ══════");
    println!("  입력    {}", path.display());
    println!(
        "  조각    {n_items}개 / seed {seeds:?} / lbf_x={} lbf_y={}",
        c.lbf_x_multiplier, c.lbf_y_multiplier
    );
    println!("  솔버판정  CollisionTracker::get_total_loss (jagua CDE)");
    println!("  엄격판정  flat_layout_collision_counts (양방향 다각형)\n");

    let mut failures = vec![];
    for mode in [
        LbfOrderMode::LargestFirst,
        LbfOrderMode::SmallestFirst,
        LbfOrderMode::MiddleOutLargeFirst,
        LbfOrderMode::MiddleOutSmallFirst,
        LbfOrderMode::ConcavityFirst,
        LbfOrderMode::SizeTimesConcavity,
    ] {
        for &seed in &seeds {
            let built = LBFBuilder::new(
                instance.clone(),
                Xoshiro256PlusPlus::seed_from_u64(seed),
                LBF_SAMPLE_CONFIG,
                c.lbf_x_multiplier,
                c.lbf_y_multiplier,
            )
            .with_order_mode(mode)
            .with_raster_config(c.lbf_raster)
            .construct();

            let exact_error = swarm_core::structure::pipeline::solution_validation_error(
                &instance,
                &built.prob.save(),
            );
            assert!(
                exact_error.is_none(),
                "{} seed {seed}: {exact_error:?}",
                mode.label()
            );
            let width = built.prob.strip_width();
            let placed = built.prob.layout().placed_items().len();
            let (exterior, pairs, other, first) = flat_layout_collision_counts(&built.prob.layout());
            let sep = Separator::new(
                instance.clone(),
                built.prob,
                Xoshiro256PlusPlus::seed_from_u64(seed + 1000),
                c.expl_cfg.separator_config,
            );
            let loss = sep.ct.get_total_loss();

            let bad = loss != 0.0 || pairs != 0 || other != 0 || placed != n_items;
            println!(
                "  {:20} seed {seed}  폭 {width:9.2}  배치 {placed}/{n_items}  \
                 솔버손실 {loss:.4}  엄격[외곽 {exterior} / 조각쌍 {pairs} / 기타 {other}]{}",
                mode.label(),
                if bad { "  ◀ 문제" } else { "" }
            );
            if let Some((a, b)) = first {
                println!("  {:20}   첫 충돌 쌍: item {a} vs item {b}", "");
            }
            if bad {
                failures.push((mode.label(), seed, loss, pairs, other, placed));
            }
        }
    }

    println!();
    if failures.is_empty() {
        println!("  모든 모드에서 겹침 없음 · 전량 배치.\n");
    } else {
        println!("  문제 있는 조합 {}건:", failures.len());
        for (label, seed, loss, pairs, other, placed) in &failures {
            println!(
                "    {label} seed {seed}: 손실 {loss:.4}, 조각쌍 {pairs}, 기타 {other}, 배치 {placed}/{n_items}"
            );
        }
        println!();
    }
    assert!(
        failures.is_empty(),
        "초기 해에 겹침이 있거나 조각을 다 놓지 못한 모드가 있다: {failures:?}"
    );
}
