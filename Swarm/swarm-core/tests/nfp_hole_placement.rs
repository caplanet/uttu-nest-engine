//! NFP 내부 구멍(hole) 지원 검증.
//!
//! "오목한 조각 사이의 빈 공간에 다른 조각을 넣는 배치"가 실제로 되는지 본다.
//! 볼록 근사 NFP였다면 오목부가 메워져 구멍이 사라지고, 그 안쪽 위치는 전부
//! 금지 영역으로 잘못 판정된다.
#![cfg(feature = "adaptive")]

use swarm_nfp::{Location, NfpEngine, Point as NfpPoint, PolygonSet, Ring};

fn ring(points: &[(f64, f64)]) -> Ring {
    Ring::new(points.iter().map(|&(x, y)| NfpPoint::new(x, y)))
}

/// 안쪽에 정사각형 공동을 가진 `ㅁ` 모양 프레임.
/// 바깥 40x40, 안쪽 공동 10..30 (20x20).
fn frame() -> PolygonSet {
    PolygonSet::new(vec![swarm_nfp::Polygon::with_holes(
        ring(&[(0.0, 0.0), (40.0, 0.0), (40.0, 40.0), (0.0, 40.0)]),
        vec![ring(&[
            (10.0, 10.0),
            (30.0, 10.0),
            (30.0, 30.0),
            (10.0, 30.0),
        ])],
    )])
}

/// 프레임 공동(20x20)에 여유 있게 들어가는 8x8 정사각형.
fn small_square() -> PolygonSet {
    PolygonSet::from_outer(ring(&[(0.0, 0.0), (8.0, 0.0), (8.0, 8.0), (0.0, 8.0)]))
}

/// `U` 자 오목 도형. 두 다리 사이가 열린 공간이다.
fn u_shape() -> PolygonSet {
    PolygonSet::from_outer(ring(&[
        (0.0, 0.0),
        (30.0, 0.0),
        (30.0, 40.0),
        (20.0, 40.0),
        (20.0, 10.0),
        (10.0, 10.0),
        (10.0, 40.0),
        (0.0, 40.0),
    ]))
}

/// 프레임의 내부 공동은 NFP에 **구멍**으로 남아야 한다.
///
/// 그 구멍 안의 앵커는 "작은 사각형을 프레임 공동 한가운데 놓는" 배치이며 실제로
/// 충돌하지 않는다. 볼록 근사였다면 이 위치가 금지 영역이 된다.
#[test]
fn nfp_keeps_the_interior_void_as_a_hole() {
    let engine = NfpEngine::new();
    let fixed = engine.prepare(&frame()).unwrap();
    let moving = engine.prepare(&small_square()).unwrap();

    let nfp = engine.pair_nfp(&fixed, &moving).unwrap();

    let holes: usize = nfp.components.iter().map(|c| c.holes.len()).sum();
    assert!(
        holes >= 1,
        "프레임 내부 공동은 NFP 구멍으로 남아야 한다 (구멍 {holes}개)"
    );

    // 공동 중앙에 놓는 앵커. NFP(F, M) = F + (-M) 규약에서 moving의 로컬 원점이
    // (0,0)이고 크기가 8x8이므로, 공동 10..30 안에 완전히 들어가는 앵커는
    // 10 <= t <= 22 구간이다. 그 한가운데인 (16, 16)을 본다.
    let inside_void = NfpPoint::new(16.0, 16.0);
    assert_eq!(
        engine.locate(&nfp, inside_void).unwrap(),
        Location::Outside,
        "공동 안쪽 앵커는 금지 영역 밖이어야 한다 (= 배치 가능)"
    );

    // 실제 도형 관계로도 교차 검증한다.
    assert_eq!(
        engine
            .classify_pair(&fixed, &moving, swarm_nfp::Translation::new(16.0, 16.0))
            .unwrap(),
        swarm_nfp::PairRelation::Disjoint,
        "공동 한가운데 배치는 실제로 겹치지 않는다"
    );

    // 프레임 살(material) 위에 겹치는 앵커는 금지되어야 한다.
    assert_eq!(
        engine.locate(&nfp, NfpPoint::new(2.0, 2.0)).unwrap(),
        Location::Inside,
        "프레임 살과 겹치는 앵커는 금지 영역 안이어야 한다"
    );
}

/// `U` 자의 열린 홈에 들어가는 배치도 지원되어야 한다.
///
/// 홈은 닫힌 구멍이 아니라 오목한 만입부다. 볼록 근사(convex hull)를 쓰면 이 만입부가
/// 통째로 메워져 홈 안쪽 배치가 전부 사라진다.
#[test]
fn nfp_preserves_the_reentrant_notch_of_a_concave_shape() {
    let engine = NfpEngine::new();
    let fixed = engine.prepare(&u_shape()).unwrap();
    // 홈(10..20 x 10..40)에 들어가는 8x8
    let moving = engine.prepare(&small_square()).unwrap();

    let nfp = engine.pair_nfp(&fixed, &moving).unwrap();

    // 홈 한가운데에 놓는 앵커: x는 10..12, y는 10..32 구간이 유효하다.
    let in_notch = NfpPoint::new(11.0, 20.0);
    assert_eq!(
        engine.locate(&nfp, in_notch).unwrap(),
        Location::Outside,
        "오목 만입부 안쪽 앵커가 살아 있어야 한다 (볼록 근사가 아님)"
    );
    assert_eq!(
        engine
            .classify_pair(&fixed, &moving, swarm_nfp::Translation::new(11.0, 20.0))
            .unwrap(),
        swarm_nfp::PairRelation::Disjoint,
    );

    // 볼록 껍질이었다면 이 점도 금지가 된다 — 대조군으로 다리 위를 확인한다.
    assert_eq!(
        engine.locate(&nfp, NfpPoint::new(2.0, 20.0)).unwrap(),
        Location::Inside,
        "다리와 겹치는 앵커는 여전히 금지여야 한다"
    );
}

/// NFP 구멍은 캐시를 거쳐도(부호 반전 포함) 보존되어야 한다.
#[test]
fn nfp_holes_survive_the_swap_negation_identity() {
    let engine = NfpEngine::new();
    let fixed = engine.prepare(&frame()).unwrap();
    let moving = engine.prepare(&small_square()).unwrap();

    let ab = engine.pair_nfp(&fixed, &moving).unwrap();
    let ba = engine.pair_nfp(&moving, &fixed).unwrap();

    let holes_ab: usize = ab.components.iter().map(|c| c.holes.len()).sum();
    let holes_ba: usize = ba.components.iter().map(|c| c.holes.len()).sum();
    assert_eq!(
        holes_ab, holes_ba,
        "NFP(A,B) = -NFP(B,A) 이므로 구멍 수가 같아야 한다"
    );

    // -NFP(B,A) 안에서 같은 앵커가 같은 판정을 받아야 한다.
    assert_eq!(
        engine.locate(&ab, NfpPoint::new(16.0, 16.0)).unwrap(),
        engine.locate(&ba, NfpPoint::new(-16.0, -16.0)).unwrap(),
    );
}
