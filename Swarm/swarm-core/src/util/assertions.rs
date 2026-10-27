use crate::eval::specialized_jaguars_pipeline::SpecializedHazardCollector;
use crate::quantify::tracker::CollisionTracker;
use float_cmp::{approx_eq, assert_approx_eq};
use itertools::Itertools;
use jagua_rs::collision_detection::hazards::BasicHazardEntity;
use jagua_rs::collision_detection::hazards::filter::NoFilter;
use jagua_rs::collision_detection::hazards::collector::{BasicHazardCollector, HazardCollector};
use jagua_rs::entities::Layout;
use jagua_rs::geometry::primitives::SPolygon;
use jagua_rs::probs::spp::entities::SPProblem;
use jagua_rs::util::assertions;
use std::collections::HashSet;

pub fn tracker_matches_layout(ct: &CollisionTracker, l: &Layout) -> bool {
    assert!(l.placed_items().keys().all(|k| ct.pk_idx_map.contains_key(k)));
    assert!(assertions::layout_qt_matches_fresh_qt(l));

    // A multipart placement is one hazard per part: collect over every part and
    // drop all of its own parts; expected losses add up over part pairs with
    // the envelope (hull) shape factors the tracker uses.
    let collect_all = |pk: jagua_rs::entities::PItemKey, pi: &jagua_rs::entities::PlacedItem| {
        let mut collector = BasicHazardCollector::new();
        for shape in pi.shapes() {
            l.cde().collect_poly_collisions(shape, &NoFilter, &mut collector);
        }
        collector.retain(|_, e| !matches!(e, BasicHazardEntity::PlacedItem { pk: p, .. } if *p == pk));
        collector
    };
    let touches = |c: &BasicHazardCollector, pk: jagua_rs::entities::PItemKey| {
        c.iter().any(|(_, e)| matches!(e, BasicHazardEntity::PlacedItem { pk: p, .. } if *p == pk))
    };
    // Only part pairs the engine reports as colliding count, as in the tracker:
    // the overlap proxy is positive even for disjoint parts.
    let pair_loss = |pk_a: jagua_rs::entities::PItemKey, a: &jagua_rs::entities::PlacedItem, pk_b: jagua_rs::entities::PItemKey, b: &jagua_rs::entities::PlacedItem| {
        let penalty = (crate::util::parts::hull_root(a) * crate::util::parts::hull_root(b)).sqrt();
        let mut total = 0.0f32;
        for sa in a.shapes() {
            let mut collector = BasicHazardCollector::new();
            l.cde().collect_poly_collisions(sa, &NoFilter, &mut collector);
            for (_, e) in collector.iter() {
                if let BasicHazardEntity::PlacedItem { pk, part, .. } = e {
                    if *pk == pk_b && *pk != pk_a {
                        total += crate::quantify::quantify_collision_poly_poly_with_penalty(sa, &b.shapes()[*part], penalty);
                    }
                }
            }
        }
        total
    };
    for (pk1, pi1) in l.placed_items().iter() {
        let collector = collect_all(pk1, pi1);
        assert_eq!(ct.get_pair_loss(pk1, pk1), 0.0);
        let pk1_frozen = ct.frozen[ct.pk_idx_map[pk1]];
        for (pk2, pi2) in l.placed_items().iter().filter(|(k, _)| *k != pk1) {
            // 고정-고정 쌍은 loss 집계에서 제외되므로 항상 0이어야 한다
            if ct.place_groups.same_group(pi1.item().idx(), pi2.item().idx())
                || (pk1_frozen && ct.frozen[ct.pk_idx_map[pk2]])
            {
                assert_eq!(ct.get_pair_loss(pk1, pk2), 0.0);
                continue;
            }
            let stored_loss = ct.get_pair_loss(pk1, pk2);
            match touches(&collector, pk2) {
                true => {
                    let calc_loss = pair_loss(pk1, pi1, pk2, pi2);
                    let calc_loss_r = pair_loss(pk2, pi2, pk1, pi1);
                    if !approx_eq!(f32, calc_loss, stored_loss, epsilon = 0.10 * stored_loss)
                        && !approx_eq!(f32, calc_loss_r, stored_loss, epsilon = 0.10 * stored_loss)
                    {
                        let opp_collector = collect_all(pk2, pi2);
                        if touches(&opp_collector, pk1) {
                            dbg!(&pi1.shape().vertices(), &pi2.shape().vertices());
                            dbg!(
                                stored_loss,
                                calc_loss,
                                calc_loss_r,
                                opp_collector.iter().collect_vec(),
                                BasicHazardEntity::from((pk1, pi1)),
                                BasicHazardEntity::from((pk2, pi2))
                            );
                            panic!("{}", crate::dmsg!("tracker error"));
                        } else if stored_loss == 0.0 {
                            //detecting collisions is not symmetrical (in edge cases)
                        } else {
                            dbg!(&pi1.shape().vertices(), &pi2.shape().vertices());
                            dbg!(
                                stored_loss,
                                calc_loss,
                                calc_loss_r,
                                opp_collector.iter().collect_vec(),
                                BasicHazardEntity::from((pk1, pi1)),
                                BasicHazardEntity::from((pk2, pi2))
                            );
                            panic!("{}", crate::dmsg!("tracker error"));
                        }
                    }
                }
                false => {
                    if stored_loss != 0.0 {
                        let calc_loss = pair_loss(pk1, pi1, pk2, pi2);
                        let opp_collector = collect_all(pk2, pi2);
                        //detecting collisions is not symmetrical (in edge cases)
                        if !touches(&opp_collector, pk1) {
                            dbg!(&pi1.shape().vertices(), &pi2.shape().vertices());
                            dbg!(
                                stored_loss,
                                calc_loss,
                                opp_collector.iter().collect_vec(),
                                BasicHazardEntity::from((pk1, pi1)),
                                BasicHazardEntity::from((pk2, pi2))
                            );
                            panic!("{}", crate::dmsg!("tracker error"));
                        }
                    }
                }
            }
        }
        if collector.contains_entity(&BasicHazardEntity::Exterior) {
            // 고정 아이템의 경계 충돌은 loss 집계에서 제외되므로 항상 0이어야 한다
            if pk1_frozen {
                assert_eq!(ct.get_container_loss(pk1), 0.0);
            } else {
                let stored_loss = ct.get_container_loss(pk1);
                let root = crate::util::parts::hull_root(pi1);
                // only the parts that actually leave the container count
                let calc_loss = pi1
                    .shapes()
                    .iter()
                    .filter(|shape| {
                        let mut c = BasicHazardCollector::new();
                        l.cde().collect_poly_collisions(shape, &NoFilter, &mut c);
                        c.contains_entity(&BasicHazardEntity::Exterior)
                    })
                    .map(|shape| {
                        crate::quantify::quantify_collision_poly_container_with_penalty(
                            shape,
                            l.container().outer_cd().bbox(),
                            (root * root).sqrt(),
                        )
                    })
                    .sum::<f32>();
                assert_approx_eq!(f32, stored_loss, calc_loss, ulps = 5);
            }
        } else {
            assert_eq!(ct.get_container_loss(pk1), 0.0);
        }
    }

    true
}

pub fn custom_pipeline_matches_jaguars(shape: &SPolygon, det: &SpecializedHazardCollector) -> bool {
    //Standard colllision collection, provided by jagua-rs, for comparison
    let cde = det.layout.cde();
    let base_detector = {
        let pk = det.current_pk;
        let mut coll = BasicHazardCollector::new();
        cde.collect_poly_collisions(shape, &NoFilter, &mut coll);
        // every part of the moving item is its own
        coll.retain(|_, e| !matches!(e, BasicHazardEntity::PlacedItem { pk: p, .. } if *p == pk));
        coll
    };

    //make sure these detection maps are equivalent
    let default_set: HashSet<BasicHazardEntity> = base_detector.entities().cloned().collect();
    let custom_set: HashSet<BasicHazardEntity> = det.entities().cloned().collect();

    // 커스텀 파이프라인의 pole(내접원) fast-path는 원-대-변 검사의 닫힌집합 의미론
    // (접촉 = 충돌)을 따르는 반면, jagua의 다각형 정밀 검사는 접촉을 비충돌로 본다.
    // 그래서 아이템이 정확히 맞닿은 배치에서는 커스텀 쪽이 **접촉 hazard를 추가로**
    // 검출할 수 있다 (pole ⊆ shape 이므로 기하학적으로 거짓 검출은 아니며, 해당
    // hazard의 정량화 loss는 ~0이라 탐색 결과에 실질 영향이 없다).
    // 따라서 요구 불변식은 집합 동일성이 아니라 "실제 충돌을 절대 놓치지 않는다"
    // 즉 default ⊆ custom 이다.
    assert!(
        default_set.is_subset(&custom_set),
        "custom cde pipeline MISSED collisions detected by jagua-rs! missing: {:?}, for pk: {:?}, current pi: {:?}, eval shape bbox: {:?}",
        default_set.difference(&custom_set).collect_vec(),
        det.current_pk,
        det.layout
            .placed_items()
            .get(det.current_pk)
            .map(|pi| (pi.item().idx(), pi.d_transf())),
        shape.bbox(),
    );
    true
}

pub fn strip_width_is_in_check(prob: &SPProblem) -> bool {
    let diameters_of_all_items = prob
        .instance()
        .items
        .iter()
        .map(|(i, q)| i.shape_cd().diameter() * *q as f32)
        .sum::<f32>();

    prob.strip_width() < 2.0 * (diameters_of_all_items)
}

/// Match every locked copy by item id and exact public pose, independent of
/// SlotMap keys or iteration order. Collision freedom alone cannot prove this.
pub fn frozen_poses_preserved(
    reference: &jagua_rs::probs::spp::entities::SPSolution,
    candidate: &jagua_rs::probs::spp::entities::SPSolution,
    frozen: &[bool],
) -> bool {
    let signature = |sol: &jagua_rs::probs::spp::entities::SPSolution| {
        let mut poses: Vec<_> = sol
            .layout_snapshot()
            .placed_items()
            .values()
            .filter(|p| frozen.get(p.item().idx()).copied().unwrap_or(false))
            .map(|p| {
                let (x, y) = p.d_transf().translation();
                (
                    p.item().idx(),
                    p.d_transf().rotation().to_bits(),
                    x.to_bits(),
                    y.to_bits(),
                )
            })
            .collect();
        poses.sort_unstable();
        poses
    };
    signature(reference) == signature(candidate)
}
