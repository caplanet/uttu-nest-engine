//! 누적 배치(accretion) 벤치 — 5초 예산 안에서 얼마나 나오는가.
//!
//! `PROBES` 탐침 예산, `SLIDE` 슬라이딩 라운드 수(없으면 생략).
use jagua_rs::Instant;
use jagua_rs::geometry::DTransformation;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem};
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::nfp_lbf::{NfpLbfCache, PlacementScoreWeights, PreparedLibrary};
use swarm_core::nfp_strategy::accrete::accrete;
use swarm_core::nfp_strategy::select::SelectionRule;
use swarm_core::optimizer::lbf::LBFBuilder;
use swarm_core::util::io::read_spp_input;
use std::path::Path;
use std::time::Duration;

/// 입력 경로. `SWARM_INPUT` 이 있으면 **절대 경로**로 그것을 쓴다.
///
/// 예전에는 `data/input` 상대 경로만 받아, 저장소 밖의 지정 데이터로는 아예 잴 수
/// 없었다. 어느 파일로 잰 수치인지도 출력에 남지 않았다.
fn resolve_input(name: &str) -> std::path::PathBuf {
    match std::env::var("SWARM_INPUT") {
        Ok(p) => std::path::PathBuf::from(p),
        Err(_) => Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../data/input")
            .join(name),
    }
}

fn load(name: &str) -> SPInstance {
    let path = resolve_input(name);
    let (ext_inst, _) = read_spp_input(&path).unwrap();
    let c = DEFAULT_SWARM_CONFIG;
    let imp = swarm_core::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&imp, &ext_inst).unwrap()
}

#[test]
#[ignore = "분 단위 — --ignored 로 명시 실행"]
fn accretion_within_five_seconds() {
    let probes: usize = std::env::var("PROBES").ok().and_then(|v| v.parse().ok()).unwrap_or(4096);
    let names: Vec<String> = match std::env::var("SWARM_INPUT") {
        // 절대 경로를 주면 그 하나만 돈다 — 두 인스턴스에 같은 파일을 먹이면 안 된다.
        Ok(p) => vec![p],
        Err(_) => vec!["wsjk156.json".into(), "albano.json".into()],
    };
    for name in names.iter().map(String::as_str) {
        let inst = load(name);
        let h = inst.base_strip.fixed_height;
        // **측정 조건을 먼저 찍는다.** 조건 없는 수치는 다음 사람이 쓸 수 없다.
        println!(
            "
[조건] 입력 {} / 탐침 {probes} / 슬라이딩 {} / seed 0 / 회전 [0] 고정",
            resolve_input(name).display(),
            std::env::var("SLIDE").unwrap_or_else(|_| "끔".into()),
        );
        println!(
            "[정의] 밀도 = fit_strip 후 SPSolution::density (jagua). frontier 기준 아님.              시간 = NFP 라이브러리 build + 콜드 prewarm 포함"
        );

        // 기준선.
        let rng = Xoshiro256PlusPlus::seed_from_u64(0);
        let t = Instant::now();
        // 좌측편향 계수는 **프로덕션 값**을 쓴다. `(1.0, 1.0)` 은 기준선을 실제보다
        // 나쁘게 만들어 모든 델타를 약 1.16%p 부풀린다(69.45% vs 70.61%).
        let lbf = LBFBuilder::new(
            inst.clone(),
            rng,
            LBF_SAMPLE_CONFIG,
            DEFAULT_SWARM_CONFIG.lbf_x_multiplier,
            DEFAULT_SWARM_CONFIG.lbf_y_multiplier,
        )
        .construct()
        .prob
        .save();
        println!("\n══ {name} ══");
        println!("  LBF 단독            폭 {:9.2}  밀도 {:5.2}%  {:>6}ms",
            lbf.strip_width(), lbf.density()*100.0, t.elapsed().as_millis());

        // 라이브러리 + 캐시.
        let t0 = Instant::now();
        let rotations = vec![0i64];
        let Some(library) = PreparedLibrary::build(&inst, &rotations, 0.02) else {
            println!("  라이브러리 생성 실패"); continue;
        };
        let mut cache = NfpLbfCache::new(library);
        cache.prewarm(Duration::from_secs(10));
        let prewarm_ms = t0.elapsed().as_millis();

        // item_rotations: 전부 [0].
        let max_id = inst.items.iter().map(|(i,_)| i.idx()).max().unwrap_or(0);
        let item_rotations: Vec<Vec<i64>> = (0..=max_id).map(|_| vec![0i64]).collect();
        let cap: f32 = inst.items.iter().map(|(it,q)| it.shape_cd().bbox().width() * (*q as f32)).sum();

        // remaining: 수요만큼 반복.
        let mut remaining: Vec<usize> = Vec::new();
        for (item, qty) in inst.items.iter() { for _ in 0..*qty { remaining.push(item.idx()); } }

        let t1 = Instant::now();
        let out = accrete(&cache, &remaining, h, cap, PlacementScoreWeights::default(),
            &item_rotations, probes, SelectionRule::MaxContact, None);
        let accrete_ms = t1.elapsed().as_millis();

        // ⑦ 슬라이딩 압축을 얹는다 — 남은 예산을 쓴다.
        let mut pieces = out.pieces.clone();
        let mut slide_ms = 0u128;
        let mut moved = 0usize;
        if std::env::var("SLIDE").is_ok() {
            let t2 = Instant::now();
            let mut placer = swarm_core::nfp_lbf::SequentialPlacer::with_item_rotations(
                &mut cache, h, cap, PlacementScoreWeights::default(), item_rotations.clone(),
            ).with_probe_budget(probes);
            let mut ctx = swarm_core::nfp_strategy::StrategyContext::new(
                &mut placer,
                swarm_core::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG,
                item_rotations.clone(),
                None,
            );
            let rounds: usize = std::env::var("SLIDE").ok().and_then(|v| v.parse().ok()).unwrap_or(2);
            let sl = swarm_core::nfp_strategy::sliding::compact(&mut ctx, &pieces, rounds);
            moved = sl.moved;
            pieces = sl.pieces;
            slide_ms = t2.elapsed().as_millis();
        }

        // SPSolution 으로 굳혀 밀도를 정확히 잰다.
        let mut prob = SPProblem::new(inst.clone()).expect("strip forms a valid container");
        // 작업 폭은 `cap` 으로 잡고 `fit_strip` 이 조인다.
        //
        // 예전에는 `sl_len = max(translation.x) + cap` 를 계산한 뒤
        // `sl_len.min(cap).max(strip_length)` 를 썼는데, `sl_len >= cap` 이 항상 참이라
        // `.min(cap)` 이 언제나 `cap` 으로 접혔다 — **`sl_len` 은 결과에 아무 영향이 없는
        // 죽은 식**이었다. 읽는 사람이 "폭을 어떻게 정하는가" 를 오해하게 만든다.
        prob.change_strip_width(cap.max(out.strip_length as f32) * 1.0001 + 0.001).expect("strip width must be positive");
        let mut left: Vec<usize> = inst.items.iter().map(|(_,q)| *q).collect();
        for p in pieces.iter() {
            if left.get(p.item_id).copied().unwrap_or(0) == 0 { continue; }
            left[p.item_id] -= 1;
            prob.place_item(SPPlacement {
                d_transf: DTransformation::new(0.0, p.translation),
                item_idx: p.item_id,
            });
        }
        if prob.n_placed_items() == 0 { println!("  누적 배치: 아무것도 못 놓음"); continue; }
        prob.fit_strip().expect("strip width must be positive");
        let sol = prob.save();
        let err = swarm_core::structure::pipeline::solution_validation_error(&inst, &sol);

        println!("  누적 배치           폭 {:9.2}  밀도 {:5.2}%  {:>6}ms  (prewarm {prewarm_ms} + 배치 {accrete_ms} + 슬라이딩 {slide_ms}, 이동 {moved})",
            sol.strip_width(), sol.density()*100.0, prewarm_ms + accrete_ms + slide_ms);
        println!("    놓은 조각 {}/{}, 미배치 {}, 검증 {}",
            out.pieces.len(), remaining.len(), out.unplaced.len(),
            match &err { None => "통과".to_string(), Some(e) => format!("실패 {e:?}") });
        println!("    ** 5초 목표: {} **",
            if prewarm_ms + accrete_ms + slide_ms <= 5000 { "안에 들어옴" } else { "초과" });
    }
}
