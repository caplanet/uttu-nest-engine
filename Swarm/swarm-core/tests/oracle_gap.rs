//! 솔버 오라클과 보고 오라클이 **실제로** 어긋나는지 재는 하네스.
//!
//! # 무엇을 재나
//!
//! 탐색이 `ReportType::ExplFeas` 로 보고하는 해마다 두 판정을 나란히 놓는다:
//!
//! * **솔버**: `CollisionTracker::get_total_loss()` — jagua CDE 기반이고,
//!   고정-고정 쌍과 고정-외곽 충돌을 loss 에서 제외한다(`tracker.rs`).
//! * **인증기/진단기**: `flat_layout_collision_counts` — 조각쌍을
//!   `polygons_collide_bidirectionally` 로 **양방향** 재검사한다
//!   (`lbf_evaluator.rs`). 콜백 페이로드의 `loss`/`collision_status` 와
//!   `solution_validation_error` 가 모두 이 쪽을 쓴다.
//!
//! 솔버가 "다 됐다(loss 0)"고 보고한 해를 진단기가 충돌로 보면, 콜백에는
//! `collision = true` 가 실린다.
//!
//! # 실측 결과 — 어긋나지 않는다
//!
//! wsjk156 기준, 탐색 단독 69건 / 전체 파이프라인 42건(ExplFeas 38 · CmprFeas 3 ·
//! Final 1) 모두에서 **불일치 0건**이다.
//!
//! 그리고 두 오라클의 관계는 처음 생각과 **반대**다. 직접 쌍 검사는 CDE 보다
//! 엄격하지 않고 **관대**하다 — `contact_tolerance`(WSjk 좌표대에서 약 8e-4mm)
//! 아래의 관통을 접촉으로 본다. `lbf_evaluator.rs` 의
//! `wsjk_item_59_and_6_sub_tolerance_sliver_is_treated_as_contact` 가 그것을
//! 못박아 두었다: 그 쌍에서 CDE 는 충돌이라 답하고 직접 검사는 양방향 모두
//! 깨끗하다고 답한다.
//!
//! 따라서 남아 있는 위험은 "실행가능 해가 충돌로 보고되는" 쪽이 아니라
//! **"충돌이 보고되지 않는"** 쪽이다. 이 하네스는 양쪽 다 재지 못한다 —
//! 같은 술어(`flat_layout_collision_counts`)로 인증기와 진단기를 대조할 뿐이다.
//! 반대 방향(CDE 가 잡고 직접 검사가 놓치는 경우)은 위 단위 테스트가 담당한다.
//!
//! 유일하게 남는 구조적 불일치는 **고정(frozen) 항목 예외**다. 트래커는
//! 고정-고정 쌍과 고정-외곽 충돌을 loss 에서 빼지만(해소 불가능한 상수라서)
//! 진단기는 빼지 않는다. WPF 샘플은 고정 항목을 보내지 않으므로 이 경로는
//! 지금 물리지 않는다.
//!
//! # 왜 별도 하네스인가
//!
//! 이건 단정이 아니라 **측정**이다. 어긋남이 0이면 그것도 결과이므로
//! `#[ignore]` 로 두고 필요할 때만 돌린다.
//!
//! ```powershell
//! $env:SWARM_INPUT='C:\Caplanet\AiWorks\InputData\여성WSjk_비교용-G_New_SwarmV1.json'
//! cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" `
//!   --test oracle_gap -- --ignored --nocapture --test-threads=1
//! ```

use jagua_rs::probs::spp::entities::{SPInstance, SPProblem, SPSolution};
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::eval::lbf_evaluator::flat_layout_collision_counts;
use swarm_core::util::io::read_spp_input;
use swarm_core::util::listener::{ReportType, SolutionListener};
use swarm_core::util::terminator::BasicTerminator;
use std::path::{Path, PathBuf};
use std::time::Duration;

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

/// 한 리포트에서 두 오라클이 무엇이라 답했는지.
#[derive(Debug, Clone, Copy)]
struct Sample {
    report: &'static str,
    /// 인증기/진단기가 센 조각쌍 충돌 수.
    pair_collisions: usize,
    /// 인증기/진단기가 센 홀·저품질구역 충돌 수.
    other_collisions: usize,
    /// 인증기/진단기가 센 외곽 충돌 수(참고용 — bbox 규칙과 다를 수 있다).
    exterior_collisions: usize,
}

struct OracleProbe {
    instance: SPInstance,
    samples: Vec<Sample>,
}

impl SolutionListener for OracleProbe {
    fn report(&mut self, report: ReportType, solution: &SPSolution, _instance: &SPInstance) {
        // 솔버가 "실행 가능" 이라고 라벨한 것만 본다. 그 라벨이 맞는지가 질문이다.
        let label = match report {
            ReportType::ExplFeas => "ExplFeas",
            ReportType::CmprFeas => "CmprFeas",
            ReportType::Final => "Final",
            _ => return,
        };
        let mut prob = SPProblem::new(self.instance.clone()).expect("strip forms a valid container");
        prob.restore(solution);
        let (exterior, pairs, other, _) = flat_layout_collision_counts(&prob.layout());
        self.samples.push(Sample {
            report: label,
            pair_collisions: pairs,
            other_collisions: other,
            exterior_collisions: exterior,
        });
    }
}

#[test]
#[ignore = "실측 하네스 — 탐색을 실제로 돌린다"]
fn feasible_labels_agree_with_the_reporting_oracle() {
    let path = input_path();
    let instance = load(&path);
    let mut c = DEFAULT_SWARM_CONFIG;
    let secs: u64 = std::env::var("ORACLE_EXPLORE_SECS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(30);
    c.expl_cfg.time_limit = Duration::from_secs(secs);
    c.expl_cfg.separator_config.n_workers = 1;

    println!("\n══════ 오라클 대조 ══════");
    println!("  입력   {}", path.display());
    println!("  조건   탐색 {secs}s / 워커 1 / seed 0 / 고정 항목 없음");
    println!("  솔버   CollisionTracker::get_total_loss (jagua CDE, 고정 예외 있음)");
    println!("  진단   flat_layout_collision_counts (양방향 다각형, 고정 예외 없음)\n");

    let mut probe = OracleProbe {
        instance: instance.clone(),
        samples: vec![],
    };
    let mut term = BasicTerminator::new();

    // 탐색만 보면 절반이다 — `CmprFeas` 와 `Final` 도 JSON 을 싣는다.
    // 전체 파이프라인을 돌려 세 종류를 모두 표본에 넣는다.
    c.cmpr_cfg.time_limit = Duration::from_secs(secs.max(1) / 2);
    c.polish_cfg.time_limit = Duration::from_secs(secs.max(1) / 2);
    c.cmpr_cfg.separator_config.n_workers = 1;

    let _ = swarm_core::optimizer::optimize(
        instance.clone(),
        Xoshiro256PlusPlus::seed_from_u64(0),
        Some(&mut probe),
        &mut term,
        &c.expl_cfg,
        &c.cmpr_cfg,
        &c.polish_cfg,
        None,
        c.lbf_x_multiplier,
        c.lbf_y_multiplier,
        &c.lbf_raster,
    );

    let total = probe.samples.len();
    let disagreeing = probe
        .samples
        .iter()
        .filter(|s| s.pair_collisions > 0 || s.other_collisions > 0)
        .collect::<Vec<_>>();

    let by_kind = |k: &str| probe.samples.iter().filter(|s| s.report == k).count();
    println!(
        "  실행가능으로 라벨된 리포트: {total}건          (ExplFeas {} / CmprFeas {} / Final {})",
        by_kind("ExplFeas"),
        by_kind("CmprFeas"),
        by_kind("Final")
    );
    for s in &probe.samples {
        if s.pair_collisions > 0 || s.other_collisions > 0 {
            println!(
                "  ▲ {} — 조각쌍 충돌 {} / 기타 {} / 외곽 {}",
                s.report, s.pair_collisions, s.other_collisions, s.exterior_collisions
            );
        }
    }
    println!("  두 오라클이 어긋난 건수: {}\n", disagreeing.len());

    assert!(total > 0, "실행가능 리포트가 하나도 안 나왔다 — 하네스가 무의미하다");
    assert!(
        disagreeing.is_empty(),
        "솔버가 실행가능이라 라벨한 해 {}건에서 진단 오라클이 충돌을 찾았다 — \
         콜백에는 collision=true 가 실린다",
        disagreeing.len()
    );
}
