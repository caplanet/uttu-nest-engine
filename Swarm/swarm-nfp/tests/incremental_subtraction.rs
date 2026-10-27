//! `subtract_obstacle` 가 `feasible_region` 과 **같은 결과**를 내는지 고정한다.
//!
//! 이 동치가 증분 유지의 근거다. 집합 차는 합집합에 분배되므로
//!
//! ```text
//!     A - (B u C) = (A - B) - C
//! ```
//!
//! 이고, 따라서 배치 루프가 영역을 들고 다니며 새로 놓인 것만 빼도 결과가 같다.
//! 이 등식이 깨지면 증분 경로가 조용히 다른 배치를 내므로, 여기서 못박는다.

use swarm_nfp::{Clearance, NfpEngine, PlacedObstacle, Point, PolygonSet, Ring, Translation};

fn rect(w: f64, h: f64) -> PolygonSet {
    PolygonSet::from_outer(Ring::new(vec![
        Point::new(0.0, 0.0),
        Point::new(w, 0.0),
        Point::new(w, h),
        Point::new(0.0, h),
    ]))
}

/// 오목한 L 자 — 볼록 도형만으로 시험하면 실제 마커의 어려움이 빠진다.
fn ell(a: f64, b: f64) -> PolygonSet {
    PolygonSet::from_outer(Ring::new(vec![
        Point::new(0.0, 0.0),
        Point::new(a, 0.0),
        Point::new(a, b),
        Point::new(b, b),
        Point::new(b, a),
        Point::new(0.0, a),
    ]))
}

/// 장애물을 **하나씩 빼도** 한꺼번에 뺀 것과 같은 영역이 나온다.
#[test]
fn subtracting_one_at_a_time_equals_subtracting_the_union() {
    let engine = NfpEngine::new();
    let container = engine.prepare(&rect(200.0, 120.0)).unwrap();
    let moving = engine.prepare(&ell(30.0, 10.0)).unwrap();
    let obstacle_shape = engine.prepare(&ell(25.0, 8.0)).unwrap();

    let placements = [(20.0, 15.0), (70.0, 40.0), (130.0, 20.0), (95.0, 80.0)];
    let obstacles: Vec<PlacedObstacle<'_>> = placements
        .iter()
        .map(|&(x, y)| PlacedObstacle {
            shape: &obstacle_shape,
            translation: Translation { x, y },
        })
        .collect();

    // 한꺼번에.
    let batch = engine
        .feasible_region(swarm_nfp::FeasibleRequest {
            container: &container,
            moving: &moving,
            obstacles: &obstacles,
            clearance: Clearance::None,
        })
        .expect("일괄 계산이 성공해야 한다");

    // 하나씩 — IFP 에서 출발해 차례로 뺀다.
    let mut incremental = engine.inner_fit(&container, &moving).expect("IFP");
    for obstacle in &obstacles {
        incremental = engine
            .subtract_obstacle(&incremental, obstacle, &moving, &Clearance::None)
            .expect("증분 차감이 성공해야 한다");
    }

    assert_eq!(
        canonical(&batch),
        canonical(&incremental),
        "증분 차감이 일괄 계산과 달라지면 배치가 조용히 갈린다"
    );
}

/// 빼는 **순서**가 결과를 바꾸지 않는다 — 집합 차는 순서에 무관하다.
#[test]
fn the_order_of_subtraction_does_not_matter() {
    let engine = NfpEngine::new();
    let container = engine.prepare(&rect(150.0, 90.0)).unwrap();
    let moving = engine.prepare(&rect(20.0, 15.0)).unwrap();
    let obstacle_shape = engine.prepare(&rect(18.0, 12.0)).unwrap();

    let places = [(10.0, 10.0), (60.0, 30.0), (100.0, 55.0)];
    let mk = |x: f64, y: f64| PlacedObstacle {
        shape: &obstacle_shape,
        translation: Translation { x, y },
    };

    let ifp = engine.inner_fit(&container, &moving).unwrap();
    let mut forward = ifp.clone();
    for &(x, y) in &places {
        forward = engine
            .subtract_obstacle(&forward, &mk(x, y), &moving, &Clearance::None)
            .unwrap();
    }
    let mut backward = ifp;
    for &(x, y) in places.iter().rev() {
        backward = engine
            .subtract_obstacle(&backward, &mk(x, y), &moving, &Clearance::None)
            .unwrap();
    }
    assert_eq!(canonical(&forward), canonical(&backward));
}

/// 빈 영역에서 더 뺄 것은 없다 — 조용히 빈 채로 둔다.
#[test]
fn subtracting_from_an_empty_region_is_a_no_op() {
    let engine = NfpEngine::new();
    let moving = engine.prepare(&rect(10.0, 10.0)).unwrap();
    let obstacle_shape = engine.prepare(&rect(10.0, 10.0)).unwrap();
    let empty = PolygonSet::new(Vec::new());
    let out = engine
        .subtract_obstacle(
            &empty,
            &PlacedObstacle {
                shape: &obstacle_shape,
                translation: Translation { x: 0.0, y: 0.0 },
            },
            &moving,
            &Clearance::None,
        )
        .unwrap();
    assert!(out.is_empty());
}

/// 비교용 정규형 — 성분·정점 순서가 달라도 같은 영역이면 같게 나오도록.
fn canonical(set: &PolygonSet) -> Vec<Vec<(i64, i64)>> {
    let mut comps: Vec<Vec<(i64, i64)>> = set
        .components
        .iter()
        .map(|c| {
            let mut ring: Vec<(i64, i64)> = c
                .outer
                .vertices
                .iter()
                .map(|p| ((p.x * 1000.0).round() as i64, (p.y * 1000.0).round() as i64))
                .collect();
            ring.sort_unstable();
            ring
        })
        .collect();
    comps.sort();
    comps
}
