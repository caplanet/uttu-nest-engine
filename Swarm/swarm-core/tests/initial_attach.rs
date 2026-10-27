//! 신규 초기배치 엔진 측정. 기존 LBF/structure 경로를 쓰지 않는다.
//! 실행: cargo test --release -p swarm-core --test initial_attach -- --ignored --nocapture

use jagua_rs::collision_detection::hazards::BasicHazardEntity;
use jagua_rs::collision_detection::hazards::filter::NoFilter;
use jagua_rs::collision_detection::hazards::collector::{BasicHazardCollector, HazardCollector};
use jagua_rs::probs::spp::entities::{SPInstance, SPProblem, SPSolution};
use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
use jagua_rs::probs::spp::io::import_instance;
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::initial::{
    AttachCfg, AttachStats, CompactCfg, Order, Stage, Structure, build_initial_restarts,
    build_initial_staged_cc,
};

fn instance_path() -> String {
    std::env::var("NEST_INSTANCE").expect("NEST_INSTANCE 환경변수를 설정할 것")
}

fn load_instance() -> SPInstance {
    let path = instance_path();
    let json = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let ext: ExtSPInstance = serde_json::from_str(&json).expect("인스턴스 JSON 파싱 실패");
    let c = &DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
    import_instance(&importer, &ext).expect("인스턴스 임포트 실패")
}

/// 진단: 겹친 조각과 그 상대(Exterior 인지 다른 조각인지)를 찍는다.
fn dump_collisions(instance: &SPInstance, sol: &SPSolution) {
    let mut prob = SPProblem::new(instance.clone()).expect("strip forms a valid container");
    prob.restore(sol);
    let layout = &prob.layout();
    let sw = sol.strip_width();
    let sh = instance.base_strip.fixed_height;
    for (pk, pi) in layout.placed_items().iter() {
        let mut c = BasicHazardCollector::with_capacity(layout.placed_items().len() + 1);
        layout.cde().collect_poly_collisions(&pi.shape(), &NoFilter, &mut c);
        c.remove_by_entity(&BasicHazardEntity::from((pk, pi)));
        if c.is_empty() {
            continue;
        }
        let bb = pi.shape().bbox();
        let partners: Vec<String> = c
            .iter()
            .map(|(_, e)| match e {
                BasicHazardEntity::PlacedItem { id, .. } => format!("item#{id}"),
                BasicHazardEntity::Exterior => "EXTERIOR".to_string(),
                other => format!("{other:?}"),
            })
            .collect();
        println!(
            "DIAG collide item#{} bbox=({:.3},{:.3})-({:.3},{:.3}) strip=({:.3},{:.3}) partners={:?}",
            pi.item().idx(), bb.x_min, bb.y_min, bb.x_max, bb.y_max, sw, sh, partners
        );
    }
}

fn collisions(instance: &SPInstance, sol: &SPSolution) -> usize {
    let mut prob = SPProblem::new(instance.clone()).expect("strip forms a valid container");
    prob.restore(sol);
    let layout = &prob.layout();
    let mut n = 0usize;
    for (pk, pi) in layout.placed_items().iter() {
        let mut c = BasicHazardCollector::with_capacity(layout.placed_items().len() + 1);
        layout.cde().collect_poly_collisions(&pi.shape(), &NoFilter, &mut c);
        c.remove_by_entity(&BasicHazardEntity::from((pk, pi)));
        if !c.is_empty() {
            n += 1;
        }
    }
    n
}

struct Row {
    dens: f32,
    width: f32,
    secs: f64,
    coll: usize,
    stats: AttachStats,
}

fn run(instance: &SPInstance, cfg: &AttachCfg, stage: Stage, seed: u64) -> Row {
    run_cc(instance, cfg, &CompactCfg::default(), stage, seed, 1)
}

fn run_cc(
    instance: &SPInstance,
    cfg: &AttachCfg,
    ccfg: &CompactCfg,
    stage: Stage,
    seed: u64,
    restarts: u32,
) -> Row {
    let mut rng = Xoshiro256PlusPlus::seed_from_u64(seed);
    let (sol, stats) = if restarts > 1 {
        build_initial_restarts(instance, cfg, ccfg, stage, &mut rng, restarts)
    } else {
        build_initial_staged_cc(instance, cfg, ccfg, stage, &mut rng)
    };
    let coll = collisions(instance, &sol);
    LAST_SOL.with(|c| *c.borrow_mut() = Some(sol.clone()));
    if coll > 0 {
        dump_collisions(instance, &sol);
    }
    Row {
        dens: stats.density,
        width: stats.width,
        secs: stats.secs,
        coll,
        stats,
    }
}

fn mean(v: &[f32]) -> f32 {
    v.iter().sum::<f32>() / v.len() as f32
}

fn stddev(v: &[f32]) -> f32 {
    if v.len() < 2 {
        return 0.0;
    }
    let m = mean(v);
    (v.iter().map(|x| (x - m) * (x - m)).sum::<f32>() / (v.len() - 1) as f32).sqrt()
}

/// 한 단계를 seed 목록으로 돌리고 RESULT 행을 출력한다. (평균밀도, 평균폭, 평균시간, 총겹침)
fn report(
    label: &str,
    instance: &SPInstance,
    cfg: &AttachCfg,
    stage: Stage,
    seeds: &[u64],
) -> (f32, f32, f64, usize) {
    report_cc(label, instance, cfg, &CompactCfg::default(), stage, seeds, 1)
}

#[allow(clippy::too_many_arguments)]
fn report_cc(
    label: &str,
    instance: &SPInstance,
    cfg: &AttachCfg,
    ccfg: &CompactCfg,
    stage: Stage,
    seeds: &[u64],
    restarts: u32,
) -> (f32, f32, f64, usize) {
    let mut dens = vec![];
    let mut widths = vec![];
    let mut secs = vec![];
    let mut coll_total = 0usize;
    for &seed in seeds {
        let r = run_cc(instance, cfg, ccfg, stage, seed, restarts);
        coll_total += r.coll;
        dens.push(r.dens);
        widths.push(r.width);
        secs.push(r.secs);
        let s = &r.stats;
        let hit = if s.nfp_cache_hits + s.nfp_cache_misses > 0 {
            100.0 * s.nfp_cache_hits as f64 / (s.nfp_cache_hits + s.nfp_cache_misses) as f64
        } else {
            0.0
        };
        let verts_avg = if s.nfp_count > 0 {
            s.nfp_verts_total as f64 / s.nfp_count as f64
        } else {
            0.0
        };
        let pieces_avg = if s.nfp_count > 0 {
            s.nfp_pieces_total as f64 / s.nfp_count as f64
        } else {
            0.0
        };
        let fallback = if s.nfp_cache_misses > 0 {
            100.0 * s.hull_fallbacks as f64 / s.nfp_cache_misses as f64
        } else {
            0.0
        };
        println!(
            "RESULT step={label} seed={seed} placed {}/{} dens {:.2}% width {:.1} collisions {} {:.2}s \
             | nfp_cache_hit {:.0}% nfp_verts_avg {:.1} nfp_pieces_avg {:.1} hull_fallback {:.0}% \
             cand {} cde {} 2nd {} tail {} lp_fail {} lp_iters_used {} lp_interrupted {} compact +{:.2}pp              | self_coll {} repaired {} unrepairable {} lp_rows {} lp_pairs {} lp_skipped {} lp_applied {} lp_reverted {} block_item {:?} block_travel {:.1} budget_hits {} budget_dropped {} lp_frozen {}",
            s.placed, s.total, r.dens, r.width, r.coll, r.secs,
            hit, verts_avg, pieces_avg, fallback,
            s.cand_generated, s.cde_checks, s.second_pass, s.tail_appends,
            s.lp_fail, s.lp_iters_used, s.lp_interrupted, s.compact_gain_pp,
            s.self_collisions, s.repaired, s.unrepairable,
            s.lp_rows, s.lp_pairs, s.lp_skipped_pairs, s.lp_applied, s.lp_reverted,
            s.blocking_item, s.blocking_travel, s.budget_hits, s.budget_dropped, s.lp_frozen_pairs,
        );
    }
    println!(
        "RESULT step={label} MEAN dens {:.2}% width {:.1} time {:.2}s collisions {} stddev {:.2}pp",
        mean(&dens),
        mean(&widths),
        secs.iter().sum::<f64>() / secs.len() as f64,
        coll_total,
        stddev(&dens)
    );
    (
        mean(&dens),
        mean(&widths),
        secs.iter().sum::<f64>() / secs.len() as f64,
        coll_total,
    )
}

thread_local! {
    static LAST_SOL: std::cell::RefCell<Option<SPSolution>> = const { std::cell::RefCell::new(None) };
}

const SEEDS: [u64; 3] = [1, 2, 3];

#[test]
#[ignore]
fn step_a_skeleton() {
    let inst = load_instance();
    println!(
        "METRIC instance items={} qty={} strip_h={} full_width={:.1}",
        inst.items.len(),
        inst.total_item_qty(),
        inst.base_strip.fixed_height,
        inst.item_area() / inst.base_strip.fixed_height
    );
    let cfg = AttachCfg::default();
    let (_d, _w, _t, coll) = report("A", &inst, &cfg, Stage::A, &SEEDS);
    assert_eq!(coll, 0, "A단계에서 겹침이 0이 아니다 — 여기서 멈춘다");
    let _ = Order::Area;
}

#[test]
#[ignore]
fn step_b_nfp() {
    let inst = load_instance();
    let cfg = AttachCfg::default();
    let (_d, _w, _t, coll) = report("B", &inst, &cfg, Stage::B, &SEEDS);
    assert_eq!(coll, 0, "B단계에서 겹침이 0이 아니다");
}

#[test]
#[ignore]
fn step_c_slide() {
    let inst = load_instance();
    let cfg = AttachCfg::default();
    let (_d, _w, _t, coll) = report("C", &inst, &cfg, Stage::C, &SEEDS);
    assert_eq!(coll, 0, "C단계에서 겹침이 0이 아니다");
}

#[test]
#[ignore]
fn step_d_score() {
    let inst = load_instance();
    // 좌하단 항 가중치 3조합. 뒤집은 조합도 반드시 잰다.
    for (wx, wy) in [(0.0f32, 0.0f32), (10.0, 1.0), (1.0, 10.0)] {
        let mut cfg = AttachCfg::default();
        cfg.lbf_x_weight = wx;
        cfg.lbf_y_weight = wy;
        let label = format!("D lbf_w={wx},{wy}");
        let (_d, _w, _t, coll) = report(&label, &inst, &cfg, Stage::D, &SEEDS);
        assert_eq!(coll, 0, "D단계 {label} 에서 겹침이 0이 아니다");
    }
}

/// F단계 규모 측정: 실제 156개 도형의 볼록 분해 조각 수 분포.
#[test]
#[ignore]
fn probe_decomposition() {
    use swarm_core::initial::nfp;
    let inst = load_instance();
    for cap in [64usize, 16, 8, 6, 4] {
        let t0 = std::time::Instant::now();
        let mut counts: Vec<usize> = vec![];
        let mut fallback = 0usize;
        let mut verts: Vec<usize> = vec![];
        for id in 0..inst.items.len() {
            let sh = &inst.item(id).shape_cd();
            verts.push(sh.n_vertices());
            match nfp::decompose_convex(&sh.vertices(), cap) {
                Some(pcs) => counts.push(pcs.len()),
                None => {
                    fallback += 1;
                    counts.push(1);
                }
            }
        }
        counts.sort_unstable();
        let sum: usize = counts.iter().sum();
        let mean = sum as f64 / counts.len() as f64;
        let med = counts[counts.len() / 2];
        let mx = *counts.last().unwrap();
        // 쌍당 sub-NFP 개수 기대값 = E[p]^2
        println!(
            "METRIC decomp cap={cap} pieces mean {mean:.1} med {med} max {mx} \
             hull_fallback {fallback}/{} subnfp_per_pair~{:.0} build {:.2}s",
            counts.len(),
            mean * mean,
            t0.elapsed().as_secs_f64()
        );
        if cap == 64 {
            verts.sort_unstable();
            println!(
                "METRIC shape_verts med {} max {} sum {}",
                verts[verts.len() / 2],
                verts.last().unwrap(),
                verts.iter().sum::<usize>()
            );
        }
    }
}

#[test]
#[ignore]
fn step_e_rotations() {
    let inst = load_instance();
    let cfg = AttachCfg::default();
    let (_d, _w, _t, coll) = report("E", &inst, &cfg, Stage::E, &SEEDS);
    assert_eq!(coll, 0, "E단계에서 겹침이 0이 아니다");
}

#[test]
#[ignore]
fn step_f_nonconvex() {
    let inst = load_instance();
    let cfg = AttachCfg::default();
    // F단계는 1회가 120초를 넘길 수 있어 seed 1개로 잰다(지시서 규정).
    let (_d, _w, _t, coll) = report("F", &inst, &cfg, Stage::F, &SEEDS[..1]);
    assert_eq!(coll, 0, "F단계에서 겹침이 0이 아니다");
}


#[test]
#[ignore]
fn step_g1_compact_once() {
    let inst = load_instance();
    let cfg = AttachCfg::default();
    let cc = CompactCfg::default();
    let (_d, _w, _t, coll) = report_cc("G1 once", &inst, &cfg, &cc, Stage::G1, &SEEDS[..1], 1);
    assert_eq!(coll, 0, "G1 에서 겹침이 0이 아니다");
}

#[test]
#[ignore]
fn step_g2_compact_iters() {
    let inst = load_instance();
    let cfg = AttachCfg::default();
    for it in [5u32, 20, 50] {
        let mut cc = CompactCfg::default();
        cc.max_iters = it;
        let label = format!("G2 iters={it}");
        let (_d, _w, _t, coll) = report_cc(&label, &inst, &cfg, &cc, Stage::G2, &SEEDS[..1], 1);
        assert_eq!(coll, 0, "G2 {label} 에서 겹침이 0이 아니다");
    }
}

#[test]
#[ignore]
fn step_g3_rollback() {
    let inst = load_instance();
    let cfg = AttachCfg::default();
    let cc = CompactCfg::default();
    let (_d, _w, _t, coll) = report_cc("G3 rollback", &inst, &cfg, &cc, Stage::G3, &SEEDS[..1], 1);
    assert_eq!(coll, 0, "G3 에서 겹침이 0이 아니다");
}

#[test]
#[ignore]
fn step_g4_restarts() {
    let inst = load_instance();
    let cfg = AttachCfg::default();
    let cc = CompactCfg::default();
    for r in [1u32, 3] {
        let label = format!("G4 restarts={r}");
        let (_d, _w, _t, coll) = report_cc(&label, &inst, &cfg, &cc, Stage::G4, &SEEDS[..1], r);
        assert_eq!(coll, 0, "G4 {label} 에서 겹침이 0이 아니다");
    }
}

/// 함께 출력할 지표 4개.
#[test]
#[ignore]
fn final_metrics() {
    let inst = load_instance();
    let cfg = AttachCfg::default();
    let cc = CompactCfg::default();

    // F단계 기준 지표 (시간이 길어 seed 1개)
    let r = run_cc(&inst, &cfg, &cc, Stage::F, 1, 1);
    println!(
        "METRIC unplaced = {}   (F, seed 1)",
        r.stats.total - r.stats.placed
    );
    let mut dens = vec![];
    for &seed in SEEDS.iter() {
        dens.push(run_cc(&inst, &cfg, &cc, Stage::F, seed, 1).dens);
    }
    println!("METRIC dens_stddev = {:.3} pp   (F, seeds 1..3)", stddev(&dens));

    let mut c48 = AttachCfg::default();
    c48.rotations = 48;
    let r48 = run_cc(&inst, &c48, &cc, Stage::F, 1, 1);
    println!(
        "METRIC rot48_dens = {:.2}%  (vs rot24 {:.2}%)",
        r48.dens, r.dens
    );

    let sol = LAST_SOL.with(|c| c.borrow().clone());
    if let Some(sol) = sol {
        let ss = swarm_core::initial::compact::slack_sum(&inst, &sol, 20);
        println!("METRIC slack_sum = {ss:.1} mm");
    }
}

/// 진단: 볼록 분해가 원 도형을 제대로 덮는지(면적 합 vs 원 면적).
#[test]
#[ignore]
fn probe_decomposition_coverage() {
    use swarm_core::initial::nfp;
    let inst = load_instance();
    for cap in [16usize, 64] {
        let mut worst: Vec<(f32, usize, usize)> = vec![];
        for id in 0..inst.items.len() {
            let sh = &inst.item(id).shape_cd();
            let a0 = nfp::signed_area(&sh.vertices()).abs();
            let pcs = match nfp::decompose_convex(&sh.vertices(), cap) {
                Some(v) => v,
                None => continue,
            };
            let a1: f32 = pcs.iter().map(|p| nfp::signed_area(p).abs()).sum();
            worst.push((((a1 - a0) / a0) * 100.0, id, pcs.len()));
        }
        worst.sort_by(|a, b| b.0.abs().partial_cmp(&a.0.abs()).unwrap());
        println!(
            "METRIC coverage cap={cap} n={} worst5={:?}",
            worst.len(),
            &worst[..worst.len().min(5)]
        );
        let bad = worst.iter().filter(|w| w.0.abs() > 1.0).count();
        println!("METRIC coverage cap={cap} |area err|>1% : {bad}/{}", worst.len());
    }
}


// ===========================================================================
// TASK N02C
// ===========================================================================

/// N02C 측정 1회. seed 1 고정 (N01 에서 dens_stddev 0.000pp 확인).
/// 겹침은 전 조각 CDE 전수 검사로 독립 확인한다.
fn n02c(label: &str, inst: &SPInstance, cfg: &AttachCfg) -> Row {
    let r = run_cc(inst, cfg, &CompactCfg::default(), Stage::S2C, 1, 1);
    let s = &r.stats;
    println!(
        "RESULT step={label} seed=1 placed {}/{} dens {:.2}% width {:.1} collisions {} {:.2}s \
         slides {} slide_cancel {} dedup_skip {} evals {} place_revert {} cands_total {} cde {}",
        s.placed, s.total, r.dens, r.width, r.coll, r.secs,
        s.slides, s.slide_cancelled, s.dedup_skipped, s.cands_evaluated,
        s.place_reverted, s.cand_generated, s.cde_checks,
    );
    r
}

#[test]
#[ignore]
fn s2ca_score_fix() {
    let inst = load_instance();
    let cfg = AttachCfg::n02c();
    let r = n02c("2CA", &inst, &cfg);
    assert_eq!(r.coll, 0, "2C-A 겹침이 0이 아니다");
    assert_eq!(r.stats.placed, r.stats.total, "2C-A 156/156 이 아니다");
}


#[test]
#[ignore]
fn s2cb_sweeps() {
    let inst = load_instance();

    // (1) 좌하단 가중치 — lbf_y_weight = 0.1 고정
    let mut best_w = (0.0f32, 0.0f32); // (weight, dens)
    for w in [10.0f32, 100.0, 500.0, 1000.0] {
        let mut cfg = AttachCfg::n02c();
        cfg.lbf_x_weight = w;
        cfg.lbf_y_weight = 0.1;
        let r = n02c(&format!("2CB w={w}"), &inst, &cfg);
        if r.stats.placed == r.stats.total && r.coll == 0 && r.dens > best_w.1 {
            best_w = (w, r.dens);
        }
    }
    println!("METRIC 2CB best lbf_x_weight = {} ({:.2}%)", best_w.0, best_w.1);

    // (2) 후보 평가 상한
    let mut best_cap = (0usize, 0.0f32);
    for cap in [8usize, 16, 32] {
        let mut cfg = AttachCfg::n02c();
        cfg.cand_eval_cap = cap;
        let r = n02c(&format!("2CB cap={cap}"), &inst, &cfg);
        if r.stats.placed == r.stats.total && r.coll == 0 && r.dens > best_cap.1 {
            best_cap = (cap, r.dens);
        }
    }
    println!("METRIC 2CB best cand_eval_cap = {} ({:.2}%)", best_cap.0, best_cap.1);

    // (3) 배치 구조
    for (name, st) in [("global", Structure::Global), ("sorted", Structure::Sorted)] {
        let mut cfg = AttachCfg::n02c();
        cfg.structure = st;
        n02c(&format!("2CB struct={name}"), &inst, &cfg);
    }

    // (4) 최선값 조합
    let mut cfg = AttachCfg::n02c();
    cfg.lbf_x_weight = best_w.0;
    cfg.lbf_y_weight = 0.1;
    cfg.cand_eval_cap = best_cap.0.max(1);
    let r = n02c("2CB combined", &inst, &cfg);
    println!(
        "METRIC 2CB combined = w {} cap {} -> {:.2}% / {:.1} / {:.2}s",
        best_w.0, best_cap.0, r.dens, r.width, r.secs
    );

    // (5) 최선값 조합 x 정렬 순서 구조
    let mut cfg2 = cfg;
    cfg2.structure = Structure::Sorted;
    n02c("2CB combined+sorted", &inst, &cfg2);
}

/// 2C-B 구조 비교에서 정렬 순서가 이겼으므로, 선택된 구조 위에서 같은 두 스윕을 다시 잰다.
#[test]
#[ignore]
fn s2cb_sorted_sweeps() {
    let inst = load_instance();
    let mut best_w = (0.0f32, 0.0f32);
    for w in [10.0f32, 100.0, 500.0, 1000.0] {
        let mut cfg = AttachCfg::n02c();
        cfg.structure = Structure::Sorted;
        cfg.lbf_x_weight = w;
        cfg.lbf_y_weight = 0.1;
        let r = n02c(&format!("2CB sorted w={w}"), &inst, &cfg);
        if r.stats.placed == r.stats.total && r.coll == 0 && r.dens > best_w.1 {
            best_w = (w, r.dens);
        }
    }
    let mut best_cap = (0usize, 0.0f32);
    for cap in [8usize, 16, 32] {
        let mut cfg = AttachCfg::n02c();
        cfg.structure = Structure::Sorted;
        cfg.lbf_x_weight = best_w.0;
        cfg.lbf_y_weight = 0.1;
        cfg.cand_eval_cap = cap;
        let r = n02c(&format!("2CB sorted w={} cap={cap}", best_w.0), &inst, &cfg);
        if r.stats.placed == r.stats.total && r.coll == 0 && r.dens > best_cap.1 {
            best_cap = (cap, r.dens);
        }
    }
    println!(
        "METRIC 2CB sorted best = lbf_x {} cap {} -> {:.2}%",
        best_w.0, best_cap.0, best_cap.1
    );
}

/// 2C-B 최선 설정 (구조 비교 + 스윕 결과).
fn best_cfg() -> AttachCfg {
    let mut c = AttachCfg::n02c();
    c.structure = Structure::Sorted;
    c.lbf_x_weight = 500.0;
    c.lbf_y_weight = 0.1;
    c.cand_eval_cap = 32;
    c
}

fn n02c_w(label: &str, inst: &SPInstance, cfg: &AttachCfg) -> Row {
    let r = run_cc(inst, cfg, &CompactCfg::default(), Stage::S2C, 1, 1);
    let s = &r.stats;
    println!(
        "RESULT step={label} W={:.1} placed {}/{} dens {:.2}% width {:.1} collisions {} {:.2}s",
        cfg.fixed_width.unwrap_or(0.0),
        s.placed, s.total, r.dens, r.width, r.coll, r.secs,
    );
    r
}

#[test]
#[ignore]
fn s2cc_fixed_width() {
    use swarm_core::initial::nfp;
    let inst = load_instance();

    // (1) 목표 폭 고정
    let mut cfg = best_cfg();
    cfg.fixed_width = Some(6521.9);
    let r = n02c_w("2CC", &inst, &cfg);
    println!("METRIC unplaced_ids    = {:?}", r.stats.unplaced_ids);
    for &id in r.stats.unplaced_ids.iter() {
        let sh = &inst.item(id).shape_cd();
        let b = sh.bbox();
        let hull = nfp::hull_ccw(sh);
        let conc = nfp::signed_area(&hull).abs() / sh.area();
        println!(
            "METRIC unplaced_detail id={id} area={:.0} bbox={:.1}x{:.1} concavity={:.3}",
            inst.item(id).area(),
            b.x_max - b.x_min,
            b.y_max - b.y_min,
            conc
        );
    }

    // (2) 156/156 이 되는 가장 좁은 폭 이분 탐색
    let mut lo = 5087.1f32; // 이론 하한
    let mut hi = 7000.7f32; // 2C-B 최선 폭
    for it in 0..8 {
        let w = 0.5 * (lo + hi);
        let mut c = best_cfg();
        c.fixed_width = Some(w);
        let r = n02c_w(&format!("2CC probe{it}"), &inst, &c);
        let ok = r.stats.placed == r.stats.total && r.coll == 0;
        println!(
            "METRIC width_probe it={it} W={w:.1} placed={}/{} ok={ok} {:.2}s",
            r.stats.placed, r.stats.total, r.secs
        );
        if ok {
            hi = w;
        } else {
            lo = w;
        }
    }
    println!("METRIC narrowest_full_width = {hi:.1}  (dens {:.2}%)", 
        inst.item_area() / (hi * inst.base_strip.fixed_height) * 100.0);
}

fn n02c_ins(label: &str, inst: &SPInstance, cfg: &AttachCfg) -> Row {
    let r = run_cc(inst, cfg, &CompactCfg::default(), Stage::S2C, 1, 1);
    let s = &r.stats;
    println!(
        "RESULT step={label} W={:.1} placed {}/{} inserted {}/{} ruined_total {} reverted {} \
         attempts {} depth={} dens {:.2}% width {:.1} collisions {} {:.2}s",
        cfg.fixed_width.unwrap_or(0.0),
        s.placed, s.total,
        s.inserted, s.inserted + s.insert_fail_ids.len(),
        s.ruined_total, s.insert_reverted, s.insert_attempts, cfg.insert_depth,
        r.dens, r.width, r.coll, r.secs,
    );
    if !s.insert_fail_ids.is_empty() {
        println!("METRIC insert_fail_ids = {:?}", s.insert_fail_ids);
    }
    r
}

#[test]
#[ignore]
fn s2cd_force_insert() {
    let inst = load_instance();
    for depth in [1u32, 2, 3] {
        let mut cfg = best_cfg();
        cfg.fixed_width = Some(6521.9);
        cfg.force_insert = true;
        cfg.insert_depth = depth;
        n02c_ins(&format!("2CD depth={depth}"), &inst, &cfg);
    }
}

/// 2C-D 게이트 진단: 실패가 예산 때문인지 구조 때문인지 가른다.
#[test]
#[ignore]
fn s2cd_budget() {
    let inst = load_instance();
    for (d, ms) in [(2u32, 20000u64), (3, 20000)] {
        let mut cfg = best_cfg();
        cfg.fixed_width = Some(6521.9);
        cfg.force_insert = true;
        cfg.insert_depth = d;
        cfg.insert_time_budget_ms = ms;
        n02c_ins(&format!("2CD depth={d} budget={ms}ms"), &inst, &cfg);
    }
}

#[test]
#[ignore]
fn s2ce_separate() {
    use swarm_core::initial::separate_at_width;
    let inst = load_instance();
    let cfg = best_cfg();
    // 무작위 후보를 쓰므로 seed 3개로 잰다.
    for w in [6521.9f32, 6800.0] {
        for seed in [1u64, 2, 3] {
            let (sol, st, rep) =
                separate_at_width(&inst, &cfg, w, 50, 200, 1.1, 20000, seed);
            let coll = collisions(&inst, &sol);
            println!(
                "RESULT step=2CE W={w:.1} seed={seed} placed {}/{} penalty_start {:.0} \
                 penalty_end {:.0} iters {} resolved {} collisions {} dens {:.2}% width {:.1} {:.2}s",
                st.placed, st.total, rep.penalty_start, rep.penalty_end, rep.iters,
                if rep.resolved { "yes" } else { "no" },
                coll, st.density, st.width, st.secs
            );
        }
    }
}

/// 2C-E 를 결손이 작은 폭에서 시험한다 (2C-C 사다리에서 155/154/153 이 나온 지점).
#[test]
#[ignore]
fn s2ce_narrow_ladder() {
    use swarm_core::initial::separate_at_width;
    let inst = load_instance();
    let cfg = best_cfg();
    for w in [6993.2f32, 6970.8, 6940.9] {
        for seed in [1u64, 2, 3] {
            let (sol, st, rep) = separate_at_width(&inst, &cfg, w, 50, 200, 1.1, 20000, seed);
            let coll = collisions(&inst, &sol);
            println!(
                "RESULT step=2CE W={w:.1} seed={seed} placed {}/{} penalty_start {:.0} \
                 penalty_end {:.0} iters {} resolved {} collisions {} dens {:.2}% width {:.1} {:.2}s",
                st.placed, st.total, rep.penalty_start, rep.penalty_end, rep.iters,
                if rep.resolved { "yes" } else { "no" },
                coll, st.density, st.width, st.secs
            );
        }
    }
}

/// 최종 확인: 2C-B 최선 설정 재현 + C단계 기준선 회귀 + 못 넣은 조각 상세.
#[test]
#[ignore]
fn s2c_final() {
    use swarm_core::initial::nfp;
    let inst = load_instance();

    // (1) 기준선 회귀 (N01 C단계가 그대로인지)
    let r = run_cc(&inst, &AttachCfg::default(), &CompactCfg::default(), Stage::C, 1, 1);
    println!(
        "RESULT step=BASE-C seed=1 placed {}/{} dens {:.2}% width {:.1} collisions {} {:.2}s",
        r.stats.placed, r.stats.total, r.dens, r.width, r.coll, r.secs
    );

    // (2) 2C 최선 설정 (자유 폭)
    let r = n02c("2C-BEST", &inst, &best_cfg());
    assert_eq!(r.coll, 0);
    assert_eq!(r.stats.placed, r.stats.total);

    // (3) 7000.7mm 바로 아래에서 못 들어가는 조각
    for w in [6993.2f32, 6970.8] {
        let mut c = best_cfg();
        c.fixed_width = Some(w);
        let r = n02c_w(&format!("2C-EDGE W={w:.1}"), &inst, &c);
        println!("METRIC edge_unplaced W={w:.1} ids={:?}", r.stats.unplaced_ids);
        for &id in r.stats.unplaced_ids.iter() {
            let sh = &inst.item(id).shape_cd();
            let b = sh.bbox();
            let hull = nfp::hull_ccw(sh);
            println!(
                "METRIC edge_detail W={w:.1} id={id} area={:.0} bbox={:.1}x{:.1} concavity={:.3}",
                inst.item(id).area(),
                b.x_max - b.x_min,
                b.y_max - b.y_min,
                nfp::signed_area(&hull).abs() / sh.area()
            );
        }
    }
}

// ===========================================================================
// TASK N03C
// ===========================================================================

/// 3C-A 형상 등가 클래스 키.
/// ponytail: 위 4개 값이 모두 같으면 사실상 같은 형상이다. 어긋나면 그때 좌표 비교.
fn shape_class_key(sh: &jagua_rs::geometry::primitives::SPolygon) -> (usize, i64, i64, i64, i64) {
    use swarm_core::initial::nfp;
    let b = sh.bbox();
    let hull = nfp::hull_ccw(sh);
    let conc = nfp::signed_area(&hull).abs() / sh.area();
    (
        sh.n_vertices(),
        (sh.area()).round() as i64,
        ((b.x_max - b.x_min) * 10.0).round() as i64,
        ((b.y_max - b.y_min) * 10.0).round() as i64,
        (conc * 1000.0).round() as i64,
    )
}

#[test]
#[ignore]
fn s3ca_shape_classes() {
    use std::collections::HashMap;
    let inst = load_instance();
    let n = inst.items.len();

    let mut map: HashMap<(usize, i64, i64, i64, i64), Vec<usize>> = HashMap::new();
    for id in 0..n {
        map.entry(shape_class_key(&inst.item(id).shape_cd()))
            .or_default()
            .push(id);
    }
    let mut classes: Vec<(_, Vec<usize>)> = map.into_iter().collect();
    classes.sort_by(|a, b| b.1.len().cmp(&a.1.len()).then(a.0.cmp(&b.0)));

    // 목표 폭에서 미배치인 조각
    let mut cfg = best_cfg();
    cfg.fixed_width = Some(6521.9);
    let r = run_cc(&inst, &cfg, &CompactCfg::default(), Stage::S2C, 1, 1);
    let unplaced: std::collections::HashSet<usize> =
        r.stats.unplaced_ids.iter().copied().collect();

    println!("METRIC shape_classes = 총 {}개 클래스", classes.len());
    println!(
        "METRIC class_sizes   = {:?}",
        classes.iter().map(|c| c.1.len()).collect::<Vec<_>>()
    );
    let dup: usize = classes.iter().filter(|c| c.1.len() >= 2).map(|c| c.1.len()).sum();
    println!(
        "METRIC dup_ratio     = {}/{} = {:.3}",
        dup,
        n,
        dup as f32 / n as f32
    );
    for (k, ids) in classes.iter() {
        if ids.len() < 2 {
            continue;
        }
        let nu = ids.iter().filter(|i| unplaced.contains(i)).count();
        println!(
            "METRIC class_detail  size={} verts={} area={} bbox={:.1}x{:.1} conc={:.3} unplaced={}/{} ids={:?}",
            ids.len(), k.0, k.1,
            k.2 as f32 / 10.0, k.3 as f32 / 10.0,
            k.4 as f32 / 1000.0,
            nu, ids.len(), ids
        );
    }
    // 크기 1 클래스 요약
    let singles: usize = classes.iter().filter(|c| c.1.len() == 1).count();
    let single_unplaced: usize = classes
        .iter()
        .filter(|c| c.1.len() == 1)
        .filter(|c| unplaced.contains(&c.1[0]))
        .count();
    println!(
        "METRIC class_singles = {singles}개 (그중 목표폭 미배치 {single_unplaced}개)"
    );
    println!(
        "METRIC unplaced_total = {} (W=6521.9, placed {}/{})",
        r.stats.unplaced_ids.len(), r.stats.placed, r.stats.total
    );
}

#[test]
#[ignore]
fn s3cb_interlock() {
    use swarm_core::initial::analyze_classes;
    let inst = load_instance();
    let cfg = best_cfg();
    let classes = analyze_classes(&inst, &cfg);
    let mut valid = 0usize;
    let mut invalid = 0usize;
    for c in classes.iter() {
        if c.ids.len() < 2 {
            continue;
        }
        let f = |o: &Option<swarm_core::initial::Interlock>| match o {
            Some(i) => format!("({:.1},{:.1}) ratio={:.3}", i.d.0, i.d.1, i.ratio),
            None => "none".to_string(),
        };
        let r = c.best.map(|i| i.ratio).unwrap_or(f32::NAN);
        if r < 1.3 {
            valid += 1;
        } else if r >= 1.8 {
            invalid += 1;
        }
        println!(
            "METRIC interlock size={} area={:.0} bbox={:.1}x{:.1} conc={:.3} \
             d_best={} d_x={} d_y={} ids0={}",
            c.ids.len(), c.area, c.bbox_w, c.bbox_h, c.concavity,
            f(&c.best), f(&c.best_x), f(&c.best_y), c.ids[0]
        );
    }
    println!("METRIC interlock_summary valid(<1.3)={valid} invalid(>=1.8)={invalid}");
}

#[test]
#[ignore]
fn s3cc_group_place() {
    use swarm_core::initial::build_with_groups;
    let inst = load_instance();
    let cfg = best_cfg();
    for ratio_max in [1.3f32, 1.5] {
        for k in [2usize, 3, 4, 12] {
            let (sol, st, gs) = build_with_groups(&inst, &cfg, k, ratio_max, None);
            let coll = collisions(&inst, &sol);
            let fb = if gs.groups_tried > 0 {
                100.0 * gs.group_fallback as f32 / gs.groups_tried as f32
            } else {
                0.0
            };
            println!(
                "RESULT step=3CC ratio<{ratio_max} k={k} placed {}/{} dens {:.2}% width {:.1} \
                 collisions {} {:.2}s group_fallback {:.0}% classes {} groups {}/{} \
                 grouped {} single {}",
                st.placed, st.total, st.density, st.width, coll, st.secs,
                fb, gs.classes_grouped, gs.groups_full, gs.groups_tried,
                gs.members_grouped, gs.members_single,
            );
        }
    }
}

#[test]
#[ignore]
fn s3cd_shrink() {
    use swarm_core::initial::shrink_and_separate;
    let inst = load_instance();
    let cfg = best_cfg();
    for f in [0.005f32, 0.002, 0.001] {
        let (sol, st, rep) =
            shrink_and_separate(&inst, &cfg, f, 0.0005, 3000, 50, 1.1, 180_000, 1);
        let coll = collisions(&inst, &sol);
        for s in rep.steps.iter() {
            println!(
                "TRACE step0={:.1}% W={:.1} ok={} pairs {}->{} pen {:.0}->{:.0} iters {} {:.1}s",
                f * 100.0, s.w_try, s.ok, s.pairs_start, s.pairs_end,
                s.pen_start, s.pen_end, s.iters, s.secs
            );
        }
        println!(
            "RESULT step=3CD step0={:.1}% placed {}/{} dens {:.2}% width {:.1} collisions {} \
             shrinks_ok {} shrinks_fail {} W {:.1}->{:.1} {:.1}s",
            f * 100.0, st.placed, st.total, st.density, st.width, coll,
            rep.ok, rep.fail, rep.w_start, rep.w_end, st.secs
        );
    }
}

/// 3C-D 최선 설정(step0=0.5%)의 seed 재현성 + 기준선 회귀.
#[test]
#[ignore]
fn s3cd_seeds() {
    use swarm_core::initial::shrink_and_separate;
    let inst = load_instance();
    let cfg = best_cfg();
    let r = n02c("BASE-N02C", &inst, &cfg);
    assert_eq!(r.coll, 0);
    for seed in [1u64, 2, 3] {
        let (sol, st, rep) =
            shrink_and_separate(&inst, &cfg, 0.005, 0.0005, 3000, 50, 1.1, 180_000, seed);
        let coll = collisions(&inst, &sol);
        println!(
            "RESULT step=3CD final seed={seed} placed {}/{} dens {:.2}% width {:.1} \
             collisions {} shrinks_ok {} shrinks_fail {} {:.1}s",
            st.placed, st.total, st.density, st.width, coll, rep.ok, rep.fail, st.secs
        );
        assert_eq!(coll, 0, "3C-D seed={seed} 겹침이 0이 아니다");
        assert_eq!(st.placed, st.total, "3C-D seed={seed} 156/156 이 아니다");
    }
}

// ===========================================================================
// TASK N04C
// ===========================================================================

const STUCK_W: [f32; 2] = [6792.6, 6797.0];

fn esc_line(tag: &str, w: f32, g: &swarm_core::initial::GlsReport, coll: usize, full: bool, secs: f64) {
    println!(
        "RESULT step={tag} W={w:.1} pairs {}->{} pen {:.0}->{:.0} iters {} moves {} \
         worse_accepted {} disrupts {} ruins {}/{} chains {}/{} resolved {} collisions {} \
         full {} {:.1}s",
        g.pairs_start, g.pairs_end, g.pen_start, g.pen_end, g.iters, g.moves,
        g.worse_accepted, g.disrupts, g.ruins_ok, g.ruins_tried, g.chains_ok, g.chains_tried,
        if g.resolved { "yes" } else { "no" }, coll, full, secs
    );
}

#[test]
#[ignore]
fn s4ca_anneal_probe() {
    use swarm_core::initial::{EscapeCfg, stuck_probe};
    let inst = load_instance();
    let cfg = best_cfg();
    for t0 in [0.0f32, 0.05, 0.2, 0.5] {
        let esc = EscapeCfg {
            t0,
            ..EscapeCfg::default()
        };
        let rows = stuck_probe(&inst, &cfg, &esc, &STUCK_W, 3000, 50, 1.1, 60_000, 1);
        for (w, g, coll, full, secs) in rows.iter() {
            esc_line(&format!("4CA T0={t0}"), *w, g, *coll, *full, *secs);
        }
    }
}

fn shrink_row(tag: &str, inst: &SPInstance, cfg: &AttachCfg,
              esc: &swarm_core::initial::EscapeCfg, budget_ms: u64, seed: u64, trace: bool) -> f32 {
    use swarm_core::initial::shrink_and_separate_esc;
    let (sol, st, rep) =
        shrink_and_separate_esc(inst, cfg, esc, 0.005, 0.0005, 3000, 50, 1.1, budget_ms, seed);
    let coll = collisions(inst, &sol);
    if trace {
        for s in rep.steps.iter() {
            println!(
                "TRACE {tag} W={:.1} ok={} pairs {}->{} pen {:.0}->{:.0} iters {} \
                 moves {} worse {} disrupt {} ruin {} chain {} {:.1}s",
                s.w_try, s.ok, s.pairs_start, s.pairs_end, s.pen_start, s.pen_end,
                s.iters, s.moves, s.worse_accepted, s.disrupts, s.ruins_ok, s.chains_ok, s.secs
            );
        }
    }
    println!(
        "RESULT step={tag} seed={seed} placed {}/{} dens {:.2}% width {:.1} collisions {} \
         shrinks_ok {} shrinks_fail {} W {:.1}->{:.1} {:.1}s",
        st.placed, st.total, st.density, st.width, coll,
        rep.ok, rep.fail, rep.w_start, rep.w_end, st.secs
    );
    assert_eq!(coll, 0, "{tag} 겹침이 0이 아니다");
    assert_eq!(st.placed, st.total, "{tag} 156/156 이 아니다");
    st.density
}

#[test]
#[ignore]
fn s4ca_anneal_shrink() {
    use swarm_core::initial::EscapeCfg;
    let inst = load_instance();
    let cfg = best_cfg();
    for t0 in [0.0f32, 0.05, 0.2, 0.5] {
        let esc = EscapeCfg { t0, ..EscapeCfg::default() };
        shrink_row(&format!("4CA-shrink T0={t0}"), &inst, &cfg, &esc, 300_000, 1, t0 == 0.5);
    }
}

#[test]
#[ignore]
fn s4cb_disrupt() {
    use swarm_core::initial::EscapeCfg;
    let inst = load_instance();
    let cfg = best_cfg();
    for after in [100usize, 300, 1000] {
        for (kname, kind, radii) in [
            ("a", 0u8, vec![0.0f32]),
            ("b", 1u8, vec![1.0f32, 2.5]),
            ("c", 2u8, vec![0.0f32]),
        ] {
            for rad in radii {
                let esc = EscapeCfg {
                    t0: 0.0,
                    disrupt_after: after,
                    disrupt_kind: kind,
                    disrupt_radius: rad,
                    ..EscapeCfg::default()
                };
                let tag = if kind == 1 {
                    format!("4CB kind={kname} radius={rad} after={after}")
                } else {
                    format!("4CB kind={kname} after={after}")
                };
                shrink_row(&tag, &inst, &cfg, &esc, 300_000, 1, false);
            }
        }
    }
}

#[test]
#[ignore]
fn s4cc_ruin() {
    use swarm_core::initial::EscapeCfg;
    let inst = load_instance();
    let cfg = best_cfg();
    for r in [1.0f32, 2.0, 3.0] {
        let esc = EscapeCfg { t0: 0.0, ruin_radius: r, ruin_after: 200, ..EscapeCfg::default() };
        shrink_row(&format!("4CC r={r}"), &inst, &cfg, &esc, 300_000, 1, r == 2.0);
    }
}

#[test]
#[ignore]
fn s4cd_chain() {
    use swarm_core::initial::EscapeCfg;
    let inst = load_instance();
    let cfg = best_cfg();
    for d in [1u32, 2, 3] {
        let esc = EscapeCfg { t0: 0.0, chain_depth: d, chain_after: 200, ..EscapeCfg::default() };
        shrink_row(&format!("4CD depth={d}"), &inst, &cfg, &esc, 300_000, 1, false);
    }
}

#[test]
#[ignore]
fn s4ce_seeds() {
    use swarm_core::initial::EscapeCfg;
    let inst = load_instance();
    let cfg = best_cfg();
    // 4C-A~4C-D 각 단계 최선: T0=0, 교란 kind=a after=1000, 파괴/연쇄는 음의 기여라 끈다.
    let combos: [(&str, EscapeCfg); 2] = [
        ("plain", EscapeCfg::default()),
        (
            "disrupt-a-1000",
            EscapeCfg { disrupt_after: 1000, disrupt_kind: 0, ..EscapeCfg::default() },
        ),
    ];
    for (name, esc) in combos {
        let mut ds: Vec<f32> = Vec::new();
        for seed in 1u64..=8 {
            let d = shrink_row(&format!("4CE {name}"), &inst, &cfg, &esc, 300_000, seed, false);
            ds.push(d);
        }
        let mean = ds.iter().sum::<f32>() / ds.len() as f32;
        let sd = (ds.iter().map(|x| (x - mean) * (x - mean)).sum::<f32>()
            / (ds.len() - 1) as f32)
            .sqrt();
        let best = ds.iter().cloned().fold(f32::MIN, f32::max);
        println!(
            "METRIC 4CE {name} seed_best {:.2}% seed_mean {:.2}% seed_stddev {:.2}pp",
            best, mean, sd
        );
    }
}

// ====================================================================
// N06 — 슬라이드 재압축 + 전수 구조 재배치 + reseat + GRASP 구성
// ====================================================================

#[allow(clippy::too_many_arguments)]
fn s6_row(
    tag: &str,
    inst: &SPInstance,
    cfg: &AttachCfg,
    esc: &swarm_core::initial::EscapeCfg,
    ccfg: &swarm_core::initial::SlideCompactCfg,
    grasp: usize,
    kicks: usize,
    budget_ms: u64,
    seed: u64,
    trace: bool,
) -> f32 {
    use swarm_core::initial::shrink_compact;
    let (sol, st, rep, infos) = shrink_compact(
        inst, cfg, esc, ccfg, grasp, kicks, 0.005, 0.0005, 3000, 50, 1.1, budget_ms, seed,
    );
    let coll = collisions(inst, &sol);
    if trace {
        for s in rep.steps.iter() {
            println!(
                "TRACE {tag} W={:.1} ok={} pairs {}->{} pen {:.0}->{:.0} iters {} \
                 moves {} rescues {} {:.1}s",
                s.w_try, s.ok, s.pairs_start, s.pairs_end, s.pen_start, s.pen_end, s.iters,
                s.moves, s.rescues_ok, s.secs
            );
        }
        for i in infos.iter() {
            println!(
                "TRACE {tag} compact w {:.1}->{:.1} passes {} travel {:.1} reseats {}",
                i.w_before, i.w_after, i.passes, i.travel, i.reseats
            );
        }
    }
    // 시간-폭 프로파일: 각 성공 시점의 누적 시간과 도달 폭
    let mut cum = 0.0f64;
    for s in rep.steps.iter() {
        cum += s.secs;
        if s.ok {
            println!("PROFILE {tag} seed={seed} t={cum:.1}s w={:.1}", s.w_try);
        }
    }
    let reclaim: f32 = infos.iter().map(|i| (i.w_before - i.w_after).max(0.0)).sum();
    let reseats: usize = infos.iter().map(|i| i.reseats).sum();
    let rescues: usize = rep.steps.iter().map(|s| s.rescues_ok).sum();
    println!(
        "RESULT step={tag} seed={seed} placed {}/{} dens {:.2}% width {:.1} collisions {} \
         shrinks_ok {} shrinks_fail {} W {:.1}->{:.1} reclaim {:.1}mm reseats {} rescues {} {:.1}s",
        st.placed, st.total, st.density, st.width, coll,
        rep.ok, rep.fail, rep.w_start, rep.w_end, reclaim, reseats, rescues, st.secs
    );
    assert_eq!(coll, 0, "{tag} 겹침이 0이 아니다");
    assert_eq!(st.placed, st.total, "{tag} 156/156 이 아니다");
    st.density
}

/// 6A: 구성 해 재압축만 (축소 없음). 슬라이드/reseat 각각의 회수량을 잰다.
#[test]
#[ignore]
fn s6a_compact_probe() {
    use swarm_core::initial::{SlideCompactCfg, compact_probe};
    let inst = load_instance();
    let cfg = best_cfg();
    let h = inst.base_strip.fixed_height;
    let area = inst.item_area();
    for (name, ccfg) in [
        ("slide-only", SlideCompactCfg { reseat_top: 0, ..SlideCompactCfg::default() }),
        ("slide+reseat", SlideCompactCfg::default()),
        ("reseat8", SlideCompactCfg { reseat_top: 8, reseat_rounds: 4, ..SlideCompactCfg::default() }),
    ] {
        for seed in 1u64..=3 {
            let (sol, st, info) = compact_probe(&inst, &cfg, &ccfg, seed);
            let coll = collisions(&inst, &sol);
            let d0 = area / (info.w_before * h) * 100.0;
            println!(
                "RESULT step=6A-{name} seed={seed} placed {}/{} dens {:.2}%->{:.2}% \
                 w {:.1}->{:.1} passes {} travel {:.1} reseats {} collisions {} {:.2}s",
                st.placed, st.total, d0, st.density, info.w_before, info.w_after,
                info.passes, info.travel, info.reseats, coll, st.secs
            );
            assert_eq!(coll, 0, "6A-{name} 겹침이 0이 아니다");
        }
    }
}

/// 6B: 재압축을 축소 루프에 끼운다 (rescue 끔, grasp 1). 기준선 seed1 75.80% 비교.
#[test]
#[ignore]
fn s6b_shrink_compact() {
    use swarm_core::initial::{EscapeCfg, SlideCompactCfg};
    let inst = load_instance();
    let cfg = best_cfg();
    s6_row(
        "6B", &inst, &cfg, &EscapeCfg::default(), &SlideCompactCfg::default(), 1, 0, 300_000, 1,
        true,
    );
}

/// 6C: rescue(전수 재배치) 주기 스윕 (재압축 켬, grasp 1).
#[test]
#[ignore]
fn s6c_rescue() {
    use swarm_core::initial::{EscapeCfg, SlideCompactCfg};
    let inst = load_instance();
    let cfg = best_cfg();
    for after in [300usize, 700, 1500] {
        let esc = EscapeCfg { rescue_after: after, ..EscapeCfg::default() };
        s6_row(
            &format!("6C-rescue{after}"), &inst, &cfg, &esc, &SlideCompactCfg::default(),
            1, 0, 300_000, 1, after == 700,
        );
    }
}

/// 6D: GRASP 구성 폭 스윕 (최선 esc 는 6C 결과로 정한다 — 우선 rescue 700).
#[test]
#[ignore]
fn s6d_grasp() {
    use swarm_core::initial::{EscapeCfg, SlideCompactCfg};
    let inst = load_instance();
    let cfg = best_cfg();
    let esc = EscapeCfg { rescue_after: 700, ..EscapeCfg::default() };
    for grasp in [2usize, 4] {
        s6_row(
            &format!("6D-grasp{grasp}"), &inst, &cfg, &esc, &SlideCompactCfg::default(),
            grasp, 0, 300_000, 1, false,
        );
    }
}

/// 6E: N06 확정 조합 seed 1~8 (순차). 4C-E(75.89% best / 75.14% mean) 비교.
/// 병렬 실행이 필요하면 s6z_env 를 프로세스별로 띄울 것 (NEST_SEED / NEST_BUDGET_MS).
#[test]
#[ignore]
fn s6e_seeds() {
    use swarm_core::initial::{EscapeCfg, SlideCompactCfg};
    let inst = load_instance();
    let cfg = best_cfg();
    let esc = EscapeCfg::n06();
    let ccfg = SlideCompactCfg::default();
    let mut ds: Vec<f32> = Vec::new();
    for seed in 1u64..=8 {
        let d = s6_row("6E", &inst, &cfg, &esc, &ccfg, 1, 100_000, 300_000, seed, false);
        ds.push(d);
    }
    let m = mean(&ds);
    let sd = stddev(&ds);
    let best = ds.iter().cloned().fold(f32::MIN, f32::max);
    println!("METRIC 6E seed_best {best:.2}% seed_mean {m:.2}% seed_stddev {sd:.2}pp");
}

/// 6F: rescue 만 켜고 재압축 없이 (기존 shrink_and_separate_esc 경로).
/// 재압축(6B)과 rescue(6F)의 기여를 분리한다.
#[test]
#[ignore]
fn s6f_rescue_only() {
    use swarm_core::initial::EscapeCfg;
    let inst = load_instance();
    let cfg = best_cfg();
    let combos: [(&str, EscapeCfg); 4] = [
        ("6F-rescue300", EscapeCfg { rescue_after: 300, ..EscapeCfg::default() }),
        (
            "6G-stall1500",
            EscapeCfg { rescue_after: 300, stall_cap: 1500, ..EscapeCfg::default() },
        ),
        (
            "6G-cdfine",
            EscapeCfg { rescue_after: 300, cd_min: 0.03, ..EscapeCfg::default() },
        ),
        (
            "6G-all",
            EscapeCfg { rescue_after: 300, stall_cap: 1500, cd_min: 0.03, ..EscapeCfg::default() },
        ),
    ];
    for (tag, esc) in combos {
        shrink_row(tag, &inst, &cfg, &esc, 300_000, 1, false);
    }
}

/// 6H: kick(step 재확장 재시도)으로 남은 예산을 소진한다.
/// esc 는 6G-all 조합. 재압축 켬/끔 비교.
#[test]
#[ignore]
fn s6h_kick() {
    use swarm_core::initial::{EscapeCfg, SlideCompactCfg};
    let inst = load_instance();
    let cfg = best_cfg();
    let esc = EscapeCfg { rescue_after: 300, stall_cap: 1500, cd_min: 0.03, ..EscapeCfg::default() };
    let no_compact = SlideCompactCfg { passes_max: 0, reseat_top: 0, ..SlideCompactCfg::default() };
    s6_row("6H-kick-nc", &inst, &cfg, &esc, &no_compact, 1, 1000, 300_000, 1, false);
    s6_row("6H-kick-c", &inst, &cfg, &esc, &SlideCompactCfg::default(), 1, 1000, 300_000, 1, false);
}

/// 6Z: 환경변수로 seed/예산을 받는 단일 실행 (병렬 시드 스윕용).
/// NEST_SEED (기본 1), NEST_BUDGET_MS (기본 300000), NEST_SWAP=1 이면 swap-rescue.
#[test]
#[ignore]
fn s6z_env() {
    use swarm_core::initial::{EscapeCfg, SlideCompactCfg};
    let inst = load_instance();
    let cfg = best_cfg();
    let seed: u64 = std::env::var("NEST_SEED").ok().and_then(|s| s.parse().ok()).unwrap_or(1);
    let budget: u64 = std::env::var("NEST_BUDGET_MS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(300_000);
    let swap = std::env::var("NEST_SWAP").map_or(false, |s| s == "1");
    let esc = EscapeCfg {
        rescue_after: 300,
        stall_cap: 1500,
        cd_min: 0.03,
        swap_rescue: swap,
        ..EscapeCfg::default()
    };
    let tag = if swap { "6Z-swap" } else { "6Z" };
    s6_row(tag, &inst, &cfg, &esc, &SlideCompactCfg::default(), 1, 100_000, budget, seed, false);
}

/// 최종 최선 해(plain, seed 6)의 축소 로그 + 기준선 회귀.
#[test]
#[ignore]
fn s4c_final() {
    use swarm_core::initial::EscapeCfg;
    let inst = load_instance();
    let cfg = best_cfg();
    let r = n02c("BASE-N02C", &inst, &cfg);
    assert_eq!(r.coll, 0);
    shrink_row("4C-FINAL plain", &inst, &cfg, &EscapeCfg::default(), 300_000, 6, true);
}
