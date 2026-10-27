//! 압축 LP 를 **어떤 엔진으로 푸는지** 골라도 결과가 성립하는지 본다.
//!
//! 단위 테스트(`lp_solver::*`)는 작은 LP 를 백엔드에 직접 넘겨 본다. 이 파일은 그
//! 위층 — 실제 압축 모델을 세우고, 백엔드를 갈아 끼우고, 그 해를 레이아웃에 적용해
//! **CDE 검증까지 통과하는지**를 본다. 백엔드가 그럴듯한 숫자를 돌려주더라도 모델
//! 변환이 어긋나 있으면 여기서 걸린다.
//!
//! ```text
//! cargo test -p swarm-core --release \
//!     --features "simd,capmods,adaptive,structure-first" --test lp_backends
//! ```

use jagua_rs::geometry::DTransformation;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem};
use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::{DEFAULT_POLISH_CONFIG, DEFAULT_SWARM_CONFIG};
use swarm_core::lp_solver::{DEFAULT_LP_SOLVER_CONFIG, LpSolverConfig, LpSolverKind};
use swarm_core::optimizer::polish::lp::lp_compaction_pass_with_solver;
use swarm_core::optimizer::separator::Separator;
use swarm_core::structure::pipeline::solution_validation_error;
use swarm_core::util::terminator::BasicTerminator;

/// 두 정사각형이 넓은 strip 안에 멀찍이 떨어져 있다 — 압축의 여지가 확실히 있다.
fn fixture() -> (SPInstance, SPProblem) {
    let external: ExtSPInstance = serde_json::from_value(serde_json::json!({
        "name": "lp-backend",
        "strip_height": 16.0,
        "items": [
            {"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
            }},
            {"id":1,"demand":1,"allowed_orientations":[0.0],"shape":{
                "type":"rectangle","data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}
            }}
        ]
    }))
    .expect("fixture 는 유효한 인스턴스여야 한다");

    let c = DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
    let instance = jagua_rs::probs::spp::io::import_instance(&importer, &external)
        .expect("fixture import");

    let mut problem = SPProblem::new(instance.clone()).expect("strip forms a valid container");
    problem.change_strip_width(30.0).expect("strip width must be positive");
    for (item_id, x, y) in [(0usize, 8.0f32, 3.0f32), (1, 20.0, 11.0)] {
        problem.place_item(SPPlacement {
            item_idx: item_id,
            d_transf: DTransformation::new(0.0, (x, y)),
        });
    }
    assert!(problem.layout().is_collision_free(), "fixture 는 실행 가능해야 한다");
    (instance, problem)
}

fn run(solver: LpSolverConfig) -> (usize, f32, Option<String>) {
    let (instance, problem) = fixture();
    let width_before = problem.strip_width();
    let mut sep = Separator::new(
        instance.clone(),
        problem,
        Xoshiro256PlusPlus::seed_from_u64(303),
        DEFAULT_SWARM_CONFIG.expl_cfg.separator_config,
    );
    let mut cfg = DEFAULT_POLISH_CONFIG.lp;
    cfg.n_iterations = 1;
    cfg.trust_region_ratio = 0.2;

    let applied =
        lp_compaction_pass_with_solver(&mut sep, &cfg, &solver, &BasicTerminator::new());
    let after = sep.prob.strip_width();
    let saved = sep.prob.save();
    let err = solution_validation_error(&instance, &saved).map(|e| format!("{e:?}"));
    assert!(
        after <= width_before + 1.0e-4,
        "압축이 폭을 늘리면 안 된다: {width_before} -> {after}"
    );
    (applied, after, err)
}

/// 항상 쓸 수 있는 두 엔진은 **같은 모델에서 둘 다 압축에 성공**해야 한다.
///
/// 폭이 정확히 같을 필요는 없다 — 단체법은 꼭짓점 해를, 내점법은 중심 경로의 해를
/// 준다. 퇴화한 LP 에서 두 해가 다른 것은 정상이다. 요구하는 것은 "둘 다 줄인다" 와
/// "둘 다 유효한 배치를 남긴다" 두 가지다.
#[test]
fn both_builtin_backends_compact_the_layout_and_leave_it_valid() {
    for backend in [LpSolverKind::Microlp, LpSolverKind::Clarabel] {
        let (applied, width, err) = run(LpSolverConfig {
            backend,
            ..DEFAULT_LP_SOLVER_CONFIG
        });
        assert_eq!(applied, 1, "{}: 압축 스텝이 적용되어야 한다", backend.label());
        assert!(
            width < 30.0 - 1.0e-4,
            "{}: 폭이 줄어야 한다 (30.0 -> {width})",
            backend.label()
        );
        assert!(err.is_none(), "{}: 검증 실패 {err:?}", backend.label());
    }
}

/// 2차 정규화를 켜도 압축이 성립해야 한다.
///
/// QP 는 "같은 폭이면 덜 움직이는 해" 를 고른다. 폭이 LP 보다 조금 클 수는 있어도
/// **줄어들기는 해야** 하고, 배치는 여전히 유효해야 한다.
#[test]
fn the_quadratic_objective_still_compacts_and_stays_valid() {
    let (applied, width, err) = run(LpSolverConfig {
        backend: LpSolverKind::Clarabel,
        quadratic_regularization: 0.1,
        ..DEFAULT_LP_SOLVER_CONFIG
    });
    assert_eq!(applied, 1, "QP 도 압축 스텝을 적용해야 한다");
    assert!(width < 30.0 - 1.0e-4, "QP 도 폭을 줄여야 한다 (-> {width})");
    assert!(err.is_none(), "QP 결과가 검증을 통과해야 한다: {err:?}");
}

/// 벤더 라이브러리가 없을 때 **조용히 폴백**해서 여전히 압축이 된다.
///
/// Clp/Xpress 는 빌드에 링크하지 않고 런타임에 DLL 을 찾는다. 없는 환경이 기본이므로,
/// 그 경우에도 사용자가 아무것도 잃지 않아야 한다 — panic 도, 배치 손상도 없어야 한다.
#[test]
fn a_missing_vendor_library_falls_back_without_losing_the_compaction() {
    for backend in [LpSolverKind::Clp, LpSolverKind::Xpress] {
        let (applied, width, err) = run(LpSolverConfig {
            backend,
            fallback: true,
            ..DEFAULT_LP_SOLVER_CONFIG
        });
        assert_eq!(
            applied,
            1,
            "{}: 라이브러리가 없어도 폴백으로 압축되어야 한다",
            backend.label()
        );
        assert!(width < 30.0 - 1.0e-4, "{}: -> {width}", backend.label());
        assert!(err.is_none(), "{}: {err:?}", backend.label());
    }
}

/// 폴백을 끄면 압축을 **건너뛴다** — 배치는 그대로 유효하다.
///
/// 상용 솔버 결과를 재현해야 하는 검증 실행에서는 조용한 폴백이 오히려 해롭다.
/// 그때 기대하는 동작은 "아무것도 안 하기" 이지 "다른 엔진으로 몰래 풀기" 가 아니다.
///
/// 라이브러리가 실제로 설치된 환경에서는 압축이 일어날 수 있으므로 둘 다 허용하되,
/// **배치가 유효하고 폭이 늘지 않는 것**은 어느 쪽이든 반드시 지켜져야 한다.
#[test]
fn disabling_the_fallback_skips_compaction_instead_of_switching_engines() {
    for backend in [LpSolverKind::Clp, LpSolverKind::Xpress] {
        let (applied, width, err) = run(LpSolverConfig {
            backend,
            fallback: false,
            ..DEFAULT_LP_SOLVER_CONFIG
        });
        assert!(err.is_none(), "{}: 배치는 항상 유효해야 한다 {err:?}", backend.label());
        assert!(width <= 30.0 + 1.0e-4, "{}: 폭이 늘면 안 된다", backend.label());
        // 라이브러리가 없으면 0, 있으면 1. 둘 다 정상이다.
        assert!(applied <= 1, "{}: 스텝 수가 이상하다", backend.label());
    }
}

/// 기본 설정은 **단체법 + 순수 LP** 다.
///
/// 예전에는 "기존 인라인 경로" 와 "일반화 경로" 가 갈려 있었고 이 테스트가 그 분기를
/// 지켰다. 지금은 등가성이 증명되어 경로가 하나로 합쳐졌으므로, 지키는 것은
/// **기본 엔진과 QP 꺼짐**이다 — 기본값이 조용히 바뀌면 지금까지의 압축 실측치가
/// 전부 무효가 된다.
#[test]
fn the_default_configuration_is_the_simplex_backend_without_quadratic() {
    let cfg = DEFAULT_LP_SOLVER_CONFIG;
    assert_eq!(cfg.backend, LpSolverKind::Microlp);
    assert!(!cfg.uses_quadratic());

    let (applied, width, err) = run(cfg);
    assert_eq!(applied, 1);
    assert!(width < 30.0 - 1.0e-4);
    assert!(err.is_none(), "{err:?}");
}
