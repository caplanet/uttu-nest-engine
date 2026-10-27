//! **겹침 허용이 정확히 무엇을 허용하는가.** 공정에 나가는 값이므로 추론이 아니라
//! 수치로 못 박는다. (`scale-collapse` 899a451/fb4ca00 에서 이식.)
//!
//! # 왜 이 파일이 필요한가
//!
//! `piece_overlap_mm = X` 는 `min_item_separation = -X` 로 들어가고, jagua importer 는
//! 그 절반(`offset = -X/2`)을 **아이템과 컨테이너 양쪽에** 적용한다. 그런데 두 곳의
//! 모드가 다르다.
//!
//! ```text
//! 아이템   ShapeModifyMode::Inflate  offset -X/2  ->  도형이 X/2 작아진다
//! 컨테이너 ShapeModifyMode::Deflate  offset -X/2  ->  경계가 X/2 넓어진다
//! ```
//!
//! 그러므로 "조각끼리 X mm 겹친다" 는 맞지만, **원단 경계에서도 X mm 만큼 여유가
//! 생긴다.** 조각의 실제 도형이 원단 밖으로 나갈 수 있다는 뜻이고, 그건 조각끼리
//! 겹치는 것과 **다른 종류의 공차**다 — 겹침은 두 조각이 2mm 를 나눠 쓰는 것이고,
//! 경계 초과는 조각 하나가 원단에 없는 부분을 요구하는 것이다.
//!
//! 재단 공차 판단을 하는 사람이 이 둘을 구분해서 볼 수 있어야 한다. 그래서 잰다.
//!
//! # 어느 배치에서 재는가가 중요하다
//!
//! 기본은 LBF 시드 배치다(빠르다). 그런데 **사용자가 받는 것은 탐색을 거친 배치**이고,
//! 탐색은 조각을 경계 쪽으로 더 세게 민다. 그러므로 공정에 넘길 수치는 `OVERLAP_SECS`
//! 를 줘서 **실제 파이프라인을 거친 배치**에서 재야 한다.
//!
//! ```text
//! OVERLAP_SECS=480 cargo test ...   # 탐색 480s + 압축 160s 뒤에 잰다
//! ```
//!
//! ```text
//! cargo test -p swarm-core --release \
//!     --features "simd,capmods,adaptive,structure-first" \
//!     --test overlap_semantics -- --ignored --nocapture --test-threads=1
//! ```
#![cfg(feature = "adaptive")]

use jagua_rs::geometry::geo_traits::Transformable;
use jagua_rs::io::export::int_to_ext_transformation;
use jagua_rs::probs::spp::entities::{SPInstance, SPProblem, SPSolution};
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::optimizer::lbf::LBFBuilder;
use swarm_core::optimizer::separator::Separator;
use swarm_core::util::io::read_spp_input;
use swarm_core::util::terminator::{BasicTerminator, Terminator};
use std::path::Path;
use std::time::Duration;

/// `overlap_mm > 0` 이면 겹침 허용으로 import 한다.
fn load(name: &str, overlap_mm: f32) -> SPInstance {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../data/input")
        .join(name);
    let (ext, _) = read_spp_input(&path).unwrap_or_else(|e| panic!("{name}: {e}"));
    let c = DEFAULT_SWARM_CONFIG;
    let separation = if overlap_mm > 0.0 {
        Some(-overlap_mm)
    } else {
        c.min_item_separation
    };
    swarm_core::util::io::import_spp_instance(
        &ext,
        c.cde_config,
        c.poly_simpl_tolerance,
        separation,
        c.narrow_concavity_cutoff_ratio,
    )
    .unwrap_or_else(|e| panic!("{name}: {e:?}"))
}

/// 원단 경계(`[0, strip_height]`) 를 조각의 **실제 도형**이 얼마나 넘는가.
///
/// 놓인 도형(`PlacedItem::shape`)은 **충돌용 도형**(줄어든 것)이다. 실제로 재단되는
/// 것은 원본이므로, 원본을 같은 자세로 놓아 다시 재야 한다. 이 구분을 놓치면
/// "경계를 안 넘는다" 는 잘못된 결론이 나온다.
struct Overhang {
    below: f32,
    above: f32,
    left: f32,
    /// 충돌 도형 기준(참고용). 실제 도형과의 차이가 곧 도형 축소분이다.
    below_cd: f32,
    above_cd: f32,
}

fn overhang(inst: &SPInstance, sol: &SPSolution, strip_height: f32) -> Overhang {
    let mut o = Overhang {
        below: 0.0,
        above: 0.0,
        left: 0.0,
        below_cd: 0.0,
        above_cd: 0.0,
    };
    for pi in sol.layout_snapshot().placed_items().values() {
        let item = inst.item(pi.item().idx());

        // 충돌 도형 기준.
        let b_cd = pi.shape().bbox();
        o.below_cd = o.below_cd.max(-b_cd.y_min);
        o.above_cd = o.above_cd.max(b_cd.y_max - strip_height);

        // 실제(원본) 도형 기준. 내부 변환을 외부 변환으로 되돌려 원본에 먹인다.
        let ext = int_to_ext_transformation(&pi.d_transf(), &item.shape_orig().pre_transform);
        let placed = item.shape_orig().shape.transform_clone(&ext.compose());
        let b = placed.bbox();
        o.below = o.below.max(-b.y_min);
        o.above = o.above.max(b.y_max - strip_height);
        o.left = o.left.max(-b.x_min);
    }
    o
}

/// 실제 파이프라인(탐색 -> 압축)을 돌린 배치를 돌려준다.
///
/// 공정에 넘길 경계 초과 수치는 **사용자가 실제로 받는 배치**에서 나와야 한다.
/// 시드 배치에서 잰 값은 하한일 뿐이다 — 탐색은 조각을 경계 쪽으로 더 민다.
fn run_pipeline(inst: &SPInstance, seed: &SPSolution, secs: u64) -> SPSolution {
    let expl = DEFAULT_SWARM_CONFIG.expl_cfg;
    let mut prob = SPProblem::new(inst.clone()).expect("strip forms a valid container");
    prob.restore(seed);
    let mut sep = Separator::new(
        inst.clone(),
        prob,
        Xoshiro256PlusPlus::seed_from_u64(0),
        expl.separator_config,
    );
    let mut term = BasicTerminator::new();
    term.new_timeout(Duration::from_secs(secs));
    let sols = swarm_core::optimizer::explore::exploration_phase(
        inst, &mut sep, &mut None, &term, &expl,
    );
    let explored = sols.last().cloned().unwrap_or_else(|| seed.clone());

    let cmpr = DEFAULT_SWARM_CONFIG.cmpr_cfg;
    let mut prob = SPProblem::new(inst.clone()).expect("strip forms a valid container");
    prob.restore(&explored);
    let mut sep = Separator::new(
        inst.clone(),
        prob,
        Xoshiro256PlusPlus::seed_from_u64(1),
        cmpr.separator_config,
    );
    let mut term = BasicTerminator::new();
    term.new_timeout(Duration::from_secs((secs / 3).max(1)));
    swarm_core::optimizer::compress::compression_phase(
        inst, &mut sep, &explored, &mut None, &term, &cmpr,
    )
}

#[test]
#[ignore]
fn how_far_outside_the_fabric_does_the_overlap_setting_let_pieces_go() {
    let name = std::env::var("OVERLAP_INSTANCE").unwrap_or_else(|_| "wsjk156.json".into());
    println!("╔══ {name} — 겹침 허용의 실제 의미 ══");
    println!(
        "║ {:<8} {:>12} {:>12} {:>12} {:>12} {:>10}",
        "겹침(mm)", "컨테이너높이", "아래초과(실)", "위초과(실)", "왼쪽초과(실)", "밀도"
    );

    let secs = std::env::var("OVERLAP_SECS")
        .ok()
        .and_then(|v| v.trim().parse::<u64>().ok())
        .unwrap_or(0);
    if secs == 0 {
        println!("║ ⚠ **시드 배치에서 잰다.** 공정에 넘길 수치는 `OVERLAP_SECS=480` 으로");
        println!("║   실제 파이프라인을 거친 배치에서 재라 — 탐색이 조각을 경계로 더 민다.");
    } else {
        println!("║ 탐색 {secs}s + 압축 {}s 를 거친 배치에서 잰다.", secs / 3);
    }

    for overlap in [0.0f32, 1.0, 2.0] {
        let inst = load(&name, overlap);
        let strip_height = inst.base_strip.fixed_height;

        let seed = LBFBuilder::new(
            inst.clone(),
            Xoshiro256PlusPlus::seed_from_u64(0),
            LBF_SAMPLE_CONFIG,
            1.0,
            1.0,
        )
        .construct()
        .prob
        .save();
        let sol = if secs > 0 {
            run_pipeline(&inst, &seed, secs)
        } else {
            seed
        };
        let cd_h = sol.layout_snapshot().container().outer_cd().bbox().y_max
            - sol.layout_snapshot().container().outer_cd().bbox().y_min;
        let o = overhang(&inst, &sol, strip_height);

        println!(
            "║ {overlap:<8.1} {cd_h:>12.3} {:>12.3} {:>12.3} {:>12.3} {:>9.2}%",
            o.below,
            o.above,
            o.left,
            sol.density() * 100.0
        );
        println!(
            "║          (충돌도형 기준 아래 {:.3} / 위 {:.3} — 실제와의 차이가 도형 축소분 {:.3}mm)",
            o.below_cd,
            o.above_cd,
            overlap / 2.0
        );
    }

    println!("║");
    println!("║ 읽는 법 ─────────────────────────────────────────────────────────────");
    println!("║  · 컨테이너높이는 공칭에서 겹침만큼 **줄어야** 한다(1524 -> 1523 -> 1522).");
    println!("║    자라고 있으면 `import_spp_instance` 를 안 거친 경로다 — 그 경로는 원단을 연다.");
    println!("║  · '초과(실)' 는 **실제 재단 도형**이 원단 경계를 넘는 최대 mm 다.");
    println!("║    이것은 조각끼리의 겹침과 **다른 공차**다 — 원단에 없는 부분을 요구한다.");
    println!("║    0 이 아닌 이유는 오목 도형의 축소가 모든 경계점에서 균일하지 않아서다.");
    println!("║    **공정 판단에는 이 수치를 써라** — 승인한 겹침 값과 따로 봐야 한다.");
    println!("║  · **밀도 열은 참고용이다.** 이 하네스는 후처리를 안 돌리고 시드도 1개다.");
    println!("║    밀도 비교는 `pipeline_sweep` 으로 하라 — 여기 값과 다르다.");
    println!(
        "║  · 밀도 분모는 공칭 폭 x 공칭 높이다(`Container::area()` 가 `outer_orig` 를 쓴다)."
    );
    println!("║    컨테이너를 줄였으므로 밀도는 과대평가되지 않는다 — 오히려 보수적이다.");
    println!("╚══════════════════════════════════════════════════════════════════════");
}

/// **원단 경계가 열리지 않는지**를 단정한다. 위 하네스는 `#[ignore]` 라 CI 에서
/// 안 돌므로, 이 사실만은 항상 돌아가는 테스트로 남긴다.
///
/// 이 단정이 깨지면 공정에 **재단 불가능한 마커**가 나간다 — 조각이 원단 밖을
/// 요구한다. 겹침 기능 전체에서 가장 위험한 지점이라 여기를 지킨다.
#[test]
fn enabling_overlap_must_not_open_the_fabric_edge() {
    let nominal = load("wsjk156.json", 0.0);
    let nominal_h = nominal.base_strip.fixed_height;

    for overlap in [1.0f32, 2.0] {
        let inst = load("wsjk156.json", overlap);
        assert_eq!(
            inst.base_strip.fixed_height, nominal_h,
            "공칭 높이는 겹침과 무관하게 같아야 한다 — 밀도 분모가 이 값이다"
        );

        let sol = LBFBuilder::new(
            inst.clone(),
            Xoshiro256PlusPlus::seed_from_u64(0),
            LBF_SAMPLE_CONFIG,
            1.0,
            1.0,
        )
        .construct()
        .prob
        .save();

        // 충돌용 컨테이너는 공칭보다 `overlap` 만큼 **작아야** 한다 (위아래 각 overlap/2).
        // 그래야 overlap/2 만큼 줄어든 충돌 도형이 그 안에 들어갔을 때 실제 도형이
        // 정확히 공칭 경계에 닿는다.
        let bbox = sol.layout_snapshot().container().outer_cd().bbox();
        let cd_h = bbox.y_max - bbox.y_min;
        let shrank = nominal_h - cd_h;
        assert!(
            (shrank - overlap).abs() < 0.05,
            "겹침 {overlap}mm 에서 컨테이너가 {shrank}mm 줄었다 — {overlap}mm 여야 한다 \
(위아래 각 {}mm). 고치기 전에는 반대로 자랐다.",
            overlap / 2.0
        );
        assert!(
            bbox.y_min > 0.0,
            "컨테이너 아래 경계가 {} 다 — 0 위여야 한다",
            bbox.y_min
        );

        // **그리고 실제 재단 도형의 원단 초과가 승인한 공차 안이어야 한다.**
        // 이것이 진짜 요구사항이고 위의 컨테이너 단정은 그 수단일 뿐이다.
        //
        // 왜 0 이 아닌가: 오목 다각형을 X/2 만큼 축소할 때 **모든 경계점이 정확히
        // X/2 안으로 들어가지는 않는다.** 좁은 목이나 오목 모서리에서는 그보다 덜
        // 줄어들고, 그만큼 실제 도형이 충돌 도형 밖으로 X/2 이상 나온다.
        // wsjk156 실측: 1mm 에서 0.10mm, 2mm 에서 0.85mm (고치기 전에는 2.71mm).
        //
        // 보증하는 것은 이것이다 — **경계 초과가 승인한 겹침 값을 넘지 않는다.**
        // 고치기 전에는 2mm 설정에서 2.71mm 로 승인값 **밖**이었다.
        // 인스턴스별 실제 값은 `#[ignore]` 하네스가 찍는다. 공정 판단에는 그 수치를 써라.
        let o = overhang(&inst, &sol, nominal_h);
        for (name, v) in [("아래", o.below), ("위", o.above), ("왼쪽", o.left)] {
            assert!(
                v < overlap,
                "겹침 {overlap}mm 에서 실제 도형이 원단 {name}으로 {v}mm 넘어간다 — 승인한 공차 밖이다"
            );
        }
    }
}

/// **원단이 겹침보다 좁아도 패닉하지 않는다.**
///
/// # 두 가지 실패 방식이 있고 하나만 위험하다
///
/// | 무엇이 비는가 | 결과 |
/// |---|---|
/// | **조각** 도형 (겹침 > 조각 크기) | jagua 가 깔끔한 `Err` 를 돌려준다. 호스트가 오류를 받는다 |
/// | **컨테이너** (겹침 > 원단 짧은 변) | `From<Strip> for Container` 의 `unwrap` 에서 **패닉**한다 |
///
/// 두 번째가 위험하다 — 그 패닉은 C ABI 경계를 넘어가므로 호스트(WPF)가 죽는다.
/// 그리고 컨테이너의 **폭**은 `총면적 / 높이` 라 높이보다 훨씬 작을 수 있다.
/// 아래 픽스처가 정확히 그 경우다: 높이 20, 기준 폭 **1.6**.
///
/// 이 결함은 겹침 수정을 배선하다 실제로 밟았다. **고치기 전에는 컨테이너가 자랐으므로
/// 이 경로가 아예 없었다** — 내가 만든 위험이라 내가 막는다.
#[test]
fn a_fabric_narrower_than_the_overlap_is_clamped_instead_of_panicking() {
    use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;

    // 높이 20, 조각 4x4 두 개 -> 기준 폭 = 32/20 = 1.6.
    // 겹침 2.0 이면 컨테이너를 양쪽에서 1.0 씩 줄여 폭이 -0.4 가 된다(빈 다각형).
    let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
        "name":"narrow-strip",
        "strip_height":20.0,
        "items":[
            {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
            }},
            {"id":1,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
            }}
        ]
    }))
    .unwrap();

    let c = DEFAULT_SWARM_CONFIG;
    let inst = swarm_core::util::io::import_spp_instance(
        &ext,
        c.cde_config,
        c.poly_simpl_tolerance,
        Some(-2.0),
        c.narrow_concavity_cutoff_ratio,
    )
    .expect("좁은 원단에서 import 가 실패하면 안 된다");

    // **여기서 패닉하면 안 된다.** 고치기 전에는 이 줄에서 죽었다.
    let sol = LBFBuilder::new(
        inst.clone(),
        Xoshiro256PlusPlus::seed_from_u64(0),
        LBF_SAMPLE_CONFIG,
        1.0,
        1.0,
    )
    .construct()
    .prob
    .save();

    let bbox = sol.layout_snapshot().container().outer_cd().bbox();
    assert!(
        bbox.y_max - bbox.y_min > 0.0 && bbox.x_max - bbox.x_min > 0.0,
        "충돌 컨테이너가 비었다 ({:.3} x {:.3}) — 상한이 안 걸렸다",
        bbox.x_max - bbox.x_min,
        bbox.y_max - bbox.y_min
    );
    // 공칭 높이는 그대로여야 한다 — 밀도 분모다.
    assert!(
        (inst.base_strip.fixed_height - 20.0).abs() < 1e-6,
        "공칭 높이가 바뀌었다"
    );
    // 그리고 상한이 걸렸어도 컨테이너는 여전히 **줄어든** 쪽이어야 한다.
    assert!(
        bbox.y_min > 0.0,
        "컨테이너 아래 경계가 {} 다 — 줄이는 방향이 맞다면 0 위다",
        bbox.y_min
    );
}

/// Compare actual source polygons after both postprocessing phases, not just config fields.
#[test]
fn compress_and_polish_preserve_piece_overlap_geometry_and_certify_callbacks() {
    use jagua_rs::geometry::DTransformation;
    use jagua_rs::probs::spp::entities::SPPlacement;
    use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
    use swarm_core::structure::pipeline::solution_validation_error;
    use swarm_core::util::listener::{ReportType, SolutionListener};
    struct ExactListener(usize);
    impl SolutionListener for ExactListener {
        fn report(&mut self, kind: ReportType, sol: &SPSolution, inst: &SPInstance) {
            if matches!(
                kind,
                ReportType::CmprFeas | ReportType::ExplFeas | ReportType::Final
            ) {
                assert!(solution_validation_error(inst, sol).is_none());
                self.0 += 1;
            }
        }
    }
    let raw_overlap = |inst: &SPInstance, sol: &SPSolution| {
        let boxes: Vec<_> = sol
            .layout_snapshot()
            .placed_items()
            .values()
            .map(|p| {
                let item = inst.item(p.item().idx());
                let ext = int_to_ext_transformation(&p.d_transf(), &item.shape_orig().pre_transform);
                item.shape_orig().shape.transform_clone(&ext.compose()).bbox()
            })
            .collect();
        let dx = boxes[0].x_max.min(boxes[1].x_max) - boxes[0].x_min.max(boxes[1].x_min);
        let dy = boxes[0].y_max.min(boxes[1].y_max) - boxes[0].y_min.max(boxes[1].y_min);
        dx.min(dy).max(0.0)
    };
    for lp_enabled in [false, true] {
        for overlap in [0.0_f32, 2.0] {
            let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
                "name":"postprocess-overlap", "strip_height":50.0,
                "items":(0..2).map(|id| serde_json::json!({"id":id,"demand":1,
                    "allowed_orientations":[0.0],"shape":{"type":"rectangle","data":{
                        "x_min":0.0,"y_min":0.0,"width":40.0,"height":40.0}}})).collect::<Vec<_>>()
            }))
            .unwrap();
            let c = DEFAULT_SWARM_CONFIG;
            let instance = swarm_core::util::io::import_spp_instance(
                &ext,
                c.cde_config,
                None,
                Some(-overlap),
                None,
            )
            .unwrap();
            assert!((instance.item(0).shape_cd().bbox().width() - (40.0 - overlap)).abs() < 0.01);
            let mut prob = SPProblem::new(instance.clone()).expect("strip forms a valid container");
            prob.change_strip_width(90.0).expect("strip width must be positive");
            for (item_id, x) in [(0, 25.0), (1, 66.0)] {
                prob.place_item(SPPlacement {
                    item_idx: item_id,
                    d_transf: DTransformation::new(0.0, (x, 25.0)),
                });
            }
            let seed = prob.save();
            assert!(solution_validation_error(&instance, &seed).is_none());
            let mut cmpr = c.cmpr_cfg;
            cmpr.time_limit = Duration::from_secs(1);
            cmpr.shrink_range = (0.03, 0.001);
            cmpr.shrink_decay = swarm_core::config::ShrinkDecayStrategy::FailureBased(0.5);
            cmpr.separator_config.n_workers = 1;
            cmpr.separator_config.iter_no_imprv_limit = 15;
            cmpr.separator_config.strike_limit = 1;
            let mut sep = Separator::new(
                instance.clone(),
                prob,
                Xoshiro256PlusPlus::seed_from_u64(31),
                cmpr.separator_config,
            );
            let mut listener = ExactListener(0);
            let mut term = BasicTerminator::new();
            term.new_timeout(cmpr.time_limit);
            let compressed = swarm_core::optimizer::compress::compression_phase(
                &instance,
                &mut sep,
                &seed,
                &mut Some(&mut listener),
                &term,
                &cmpr,
            );
            assert!(compressed.strip_width() < seed.strip_width());
            assert!(solution_validation_error(&instance, &compressed).is_none());
            let compressed_overlap = raw_overlap(&instance, &compressed);
            let mut polish = c.polish_cfg;
            polish.enabled = true;
            polish.lp.enabled = lp_enabled;
            polish.slide.enabled = true;
            polish.hole_fill.enabled = true;
            polish.reclaim.enabled = false;
            polish.time_limit = Duration::from_secs(1);
            term.new_timeout(polish.time_limit);
            let polished = swarm_core::optimizer::polish::polish_phase_with_nfp(
                &instance,
                &mut sep,
                &compressed,
                &mut Some(&mut listener),
                &term,
                &polish,
                &c.nfp_integration,
                &c.lp_solver,
            );
            let polished_overlap = raw_overlap(&instance, &polished);
            assert!(solution_validation_error(&instance, &polished).is_none());
            assert!(polished.strip_width() <= compressed.strip_width());
            for actual in [compressed_overlap, polished_overlap] {
                assert!(
                    actual <= overlap + 0.01,
                    "overlap budget exceeded: {actual} > {overlap}"
                );
                if overlap > 0.0 {
                    assert!(actual > 0.5, "postprocess discarded the allowed overlap");
                }
            }
            let bounds = overhang(&instance, &polished, 50.0);
            assert!(bounds.below <= 0.01 && bounds.above <= 0.01 && bounds.left <= 0.01);
            assert!(listener.0 > 0);
            println!(
                "LP={lp_enabled}, allowance={overlap}mm: Compress overlap={compressed_overlap:.4}, Polish overlap={polished_overlap:.4}, width={:.4}->{:.4}",
                compressed.strip_width(),
                polished.strip_width()
            );
        }
    }
}
