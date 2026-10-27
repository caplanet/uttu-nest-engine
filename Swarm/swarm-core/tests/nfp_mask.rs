//! [`NfpMask`] 정확성 검증.
//!
//! 실행: `cargo test --release -p swarm-core --test nfp_mask`
//!
//! 마스크는 **단축 경로일 뿐 판정 주체가 아니다.** 따라서 검증할 명제는 하나다.
//!
//! ```text
//! mask.classify(p) == Inside   =>  engine.locate(set, p) == Inside
//! mask.classify(p) == Outside  =>  engine.locate(set, p) == Outside
//! mask.classify(p) == Boundary =>  (무엇이든 좋다 — locate가 판정한다)
//! ```
//!
//! 셀 하나를 Inside/Outside로 확정하려면 그 셀이 정말로 전부 안/밖이어야 하므로,
//! 셀 중심뿐 아니라 **셀 안 여러 지점**을 찍어 본다. 격자 양자화(0.001mm)로 판정이
//! 갈리는 지점을 노리기 위해 bbox를 조밀하게 훑는 표본도 함께 돌린다.

use swarm_core::nfp_lbf::{MaskCell, NfpMask};
use swarm_nfp::{Clearance, Location, NfpEngine, Point, PolygonSet, Polygon, Ring};

/// 마스크와 `locate`가 어긋나는 표본이 하나도 없어야 한다.
fn assert_mask_agrees(label: &str, set: &PolygonSet, samples_per_axis: usize) {
    assert_mask_agrees_opt(label, set, samples_per_axis, true)
}

/// `require_inside`가 `false`면 "Inside 확정이 하나라도 있어야 한다"를 요구하지 않는다.
/// 셀 한 변보다 얇은 도형은 전부 Boundary가 되는 것이 **정상**이다(안전 쪽).
fn assert_mask_agrees_opt(
    label: &str,
    set: &PolygonSet,
    samples_per_axis: usize,
    require_inside: bool,
) {
    let engine = NfpEngine::new();
    let mask = NfpMask::build(&engine, set);
    let Some((min_x, min_y, max_x, max_y)) = mask.bounds() else {
        panic!("{label}: 정점이 있는 집합인데 bbox가 없다");
    };

    // bbox를 10% 넉넉히 감싸 격자 밖 경로(무조건 Outside)까지 함께 검증한다.
    let pad_x = (max_x - min_x) * 0.1 + 1.0;
    let pad_y = (max_y - min_y) * 0.1 + 1.0;
    let (lo_x, hi_x) = (min_x - pad_x, max_x + pad_x);
    let (lo_y, hi_y) = (min_y - pad_y, max_y + pad_y);

    let mut checked = 0usize;
    let mut inside = 0usize;
    let mut outside = 0usize;
    for iy in 0..=samples_per_axis {
        for ix in 0..=samples_per_axis {
            let x = lo_x + (hi_x - lo_x) * ix as f64 / samples_per_axis as f64;
            let y = lo_y + (hi_y - lo_y) * iy as f64 / samples_per_axis as f64;
            let verdict = mask.classify(x, y);
            if verdict == MaskCell::Boundary {
                continue; // locate가 판정한다 — 마스크는 아무것도 주장하지 않았다
            }
            let truth = engine
                .locate(set, Point::new(x, y))
                .unwrap_or_else(|e| panic!("{label}: locate 실패 ({e})"));
            checked += 1;
            match verdict {
                MaskCell::Inside => {
                    inside += 1;
                    assert_eq!(
                        truth,
                        Location::Inside,
                        "{label}: 마스크가 Inside로 확정한 점 ({x}, {y})을 locate는 {truth:?}로 본다"
                    );
                }
                MaskCell::Outside => {
                    outside += 1;
                    assert_eq!(
                        truth,
                        Location::Outside,
                        "{label}: 마스크가 Outside로 확정한 점 ({x}, {y})을 locate는 {truth:?}로 본다"
                    );
                }
                MaskCell::Boundary => unreachable!(),
            }
        }
    }

    assert!(checked > 0, "{label}: 확정된 표본이 하나도 없다");
    if require_inside {
        assert!(
            inside > 0,
            "{label}: Inside 확정이 하나도 없다 — 단축 경로가 전혀 동작하지 않는다"
        );
    }
    assert!(outside > 0, "{label}: Outside 확정이 하나도 없다");
}

fn ring(points: &[(f64, f64)]) -> Ring {
    Ring::new(points.iter().map(|(x, y)| Point::new(*x, *y)))
}

/// 반시계 방향 직사각형.
fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Ring {
    ring(&[(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
}

#[test]
fn mask_agrees_with_locate_on_a_convex_polygon() {
    assert_mask_agrees(
        "convex",
        &PolygonSet::from_outer(ring(&[
            (0.0, 0.0),
            (120.0, 0.0),
            (170.0, 60.0),
            (140.0, 130.0),
            (40.0, 150.0),
            (-20.0, 70.0),
        ])),
        140,
    );
}

/// 회귀 테스트 — bbox 가장자리에 딱 붙은 **수직 변**.
///
/// 예전 래스터는 열 경계 클리핑에서 `lo > hi`가 나오면 그 열을 통째로 건너뛰었다.
/// `x = max_x`에 놓인 수직 변이 이 조건에 걸려 경계에 구멍이 생겼고, 내부가 외부와
/// 이어져 폴리곤 전체가 Outside로 오판됐다 (albano NFP에서 실측).
#[test]
fn mask_closes_a_vertical_edge_on_the_bbox_border() {
    let set = PolygonSet::from_outer(ring(&[
        (2898.752, -981.463),
        (2632.752, -1531.463),
        (1881.752, -1748.463),
        (-2695.268, -1662.463),
        (-2777.268, -1222.463),
        (-2777.268, 865.557),
        (-1992.268, 1791.557),
        (-9.268, 1877.557),
        (1825.732, 1804.557),
        (2576.732, 1587.557),
        (2842.752, 1037.557),
        (2898.752, -255.443),
    ]));
    let engine = NfpEngine::new();
    let mask = NfpMask::build(&engine, &set);
    // 도형 한가운데는 반드시 Inside로 확정되어야 한다.
    assert_eq!(mask.classify(-101.258, -382.453), MaskCell::Inside);
    assert_mask_agrees("bbox 경계 수직 변", &set, 160);
}

/// 마찬가지로 bbox 가장자리에 붙은 **수평 변**.
#[test]
fn mask_closes_a_horizontal_edge_on_the_bbox_border() {
    let set = PolygonSet::from_outer(ring(&[
        (0.0, 0.0),
        (900.0, 0.0),
        (900.0, 40.0),
        (500.0, 300.0),
        (400.0, 300.0),
        (0.0, 40.0),
    ]));
    assert_mask_agrees("bbox 경계 수평 변", &set, 160);
}

#[test]
fn mask_agrees_with_locate_on_a_polygon_with_a_hole() {
    // 구멍 내부는 바깥과 연결되지 않은 별도 성분이다 — "바깥에서 flood fill"류
    // 접근이라면 여기서 Inside로 오판한다.
    let set = PolygonSet::new(vec![Polygon::with_holes(
        rect(0.0, 0.0, 400.0, 300.0),
        vec![ring(&[(280.0, 80.0), (280.0, 220.0), (120.0, 220.0), (120.0, 80.0)])],
    )]);
    assert_mask_agrees("hole", &set, 160);

    let engine = NfpEngine::new();
    let mask = NfpMask::build(&engine, &set);
    // 구멍 한가운데는 Outside거나(확정) 최소한 Inside로 오판하지 않아야 한다.
    assert_ne!(mask.classify(200.0, 150.0), MaskCell::Inside);
}

#[test]
fn mask_agrees_with_locate_on_a_multi_component_set() {
    let set = PolygonSet::new(vec![
        Polygon::with_holes(rect(0.0, 0.0, 200.0, 200.0), Vec::new()),
        Polygon::with_holes(rect(600.0, 300.0, 900.0, 380.0), Vec::new()),
    ]);
    assert_mask_agrees("multi component", &set, 160);
}

/// 아주 납작한(종횡비가 큰) 도형에서도 셀 격자가 무너지지 않아야 한다.
///
/// 두께가 셀 한 변보다 얇으면 전부 Boundary가 된다 — 느릴 뿐 **틀리지 않는 것**이
/// 이 마스크의 계약이다.
#[test]
fn mask_agrees_with_locate_on_a_sliver() {
    assert_mask_agrees_opt(
        "sliver (셀보다 얇음)",
        &PolygonSet::from_outer(rect(0.0, 0.0, 5000.0, 12.0)),
        200,
        false,
    );
    // 두께가 셀 몇 개는 되는 납작한 도형이면 Inside 확정도 나와야 한다.
    assert_mask_agrees("sliver", &PolygonSet::from_outer(rect(0.0, 0.0, 5000.0, 400.0)), 200);
}

/// 실제 pair NFP — 커널이 만든 기하 그대로.
#[test]
fn mask_agrees_with_locate_on_a_real_pair_nfp() {
    let engine = NfpEngine::new();
    let fixed_src = PolygonSet::from_outer(ring(&[
        (0.0, 0.0),
        (300.0, 0.0),
        (300.0, 80.0),
        (180.0, 80.0),
        (180.0, 220.0),
        (300.0, 220.0),
        (300.0, 300.0),
        (0.0, 300.0),
    ]));
    let moving_src = PolygonSet::from_outer(ring(&[
        (0.0, 0.0),
        (140.0, 20.0),
        (110.0, 130.0),
        (30.0, 90.0),
    ]));
    let fixed = engine.prepare(&fixed_src).expect("prepare fixed");
    let moving = engine.prepare(&moving_src).expect("prepare moving");

    let clearance = Clearance::ConvexKernel(ring(&[
        (-0.01, -0.01),
        (0.01, -0.01),
        (0.01, 0.01),
        (-0.01, 0.01),
    ]));
    let nfp = engine
        .pair_nfp_with_clearance(&fixed, &moving, &clearance)
        .expect("pair nfp");
    assert_mask_agrees("pair NFP", &nfp, 200);

    // `NFP(A, B) = -NFP(B, A)` 경로 — 캐시는 부호 반전본에도 마스크를 붙인다.
    let negated = PolygonSet::new(
        nfp.components
            .iter()
            .map(|c| {
                Polygon::with_holes(
                    Ring::new(c.outer.vertices.iter().rev().map(|p| Point::new(-p.x, -p.y))),
                    c.holes
                        .iter()
                        .map(|h| {
                            Ring::new(h.vertices.iter().rev().map(|p| Point::new(-p.x, -p.y)))
                        })
                        .collect(),
                )
            })
            .collect(),
    );
    assert_mask_agrees("pair NFP (부호 반전)", &negated, 200);
}

/// 빈 집합은 `locate`가 항상 `Outside`를 주므로 마스크도 그래야 한다.
#[test]
fn empty_set_is_always_outside() {
    let engine = NfpEngine::new();
    let mask = NfpMask::build(&engine, &PolygonSet::new(Vec::new()));
    assert!(mask.bounds().is_none());
    assert_eq!(mask.classify(0.0, 0.0), MaskCell::Outside);
    assert_eq!(mask.classify(-1.0e6, 5.0e5), MaskCell::Outside);
}
