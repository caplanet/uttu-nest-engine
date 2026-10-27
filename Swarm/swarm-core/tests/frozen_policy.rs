use jagua_rs::geometry::{DTransformation, geo_traits::Transformable};
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem};
use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
use rand::{SeedableRng, rngs::Xoshiro256PlusPlus};
use swarm_core::{
    config::DEFAULT_SWARM_CONFIG, optimizer::separator::Separator,
    quantify::tracker::CollisionTracker,
    structure::pipeline::solution_validation_error_with_frozen, util::frozen,
};

fn problem(positions: &[(f32, f32)]) -> (SPInstance, SPProblem) {
    let items = (0..positions.len())
        .map(|id| {
            serde_json::json!({"id":id,"demand":1,
        "allowed_orientations":[0.0],"shape":{"type":"rectangle","data":{
            "x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}}})
        })
        .collect::<Vec<_>>();
    let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({"name":"frozen-policy",
        "strip_height":20.0,"items":items}))
    .unwrap();
    let cfg = DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(cfg.cde_config, cfg.poly_simpl_tolerance, cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
    let instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext).unwrap();
    let mut prob = SPProblem::new(instance.clone()).expect("strip forms a valid container");
    frozen::change_strip_width(&mut prob, 30.0);
    let placements = positions
        .iter()
        .enumerate()
        .map(|(id, &xy)| SPPlacement {
            item_idx: id,
            d_transf: DTransformation::new(0.0, xy),
        })
        .collect::<Vec<_>>();
    frozen::prepare_placements(
        &mut prob,
        placements.iter().map(|p| {
            instance
                .item(p.item_idx)
                .shape_cd()
                .transform_clone(&p.d_transf.compose())
                .bbox()
        }),
    );
    for p in placements {
        prob.place_item(p);
    }
    (instance, prob)
}

#[test]
fn locks_outside_every_strip_edge_and_mutual_overlap_are_exempt() {
    for xy in [
        (-1000.0, -1000.0),
        (-2.0, 4.0),
        (10.0, 1000.0),
        (1000.0, 5.0),
    ] {
        let (instance, prob) = problem(&[xy, xy, (7.0, 7.0)]);
        let mask = [true, true, false];
        assert_eq!(
            CollisionTracker::new_with_frozen(&prob.layout(), &mask).get_total_loss(),
            0.0
        );
        assert!(solution_validation_error_with_frozen(&instance, &prob.save(), &mask).is_none());
        assert!(
            swarm_core::structure::pipeline::solution_validation_error(&instance, &prob.save())
                .is_some()
        );
        let saved = prob.save();
        let mut sep = Separator::new(
            instance.clone(),
            prob,
            Xoshiro256PlusPlus::seed_from_u64(42),
            DEFAULT_SWARM_CONFIG.expl_cfg.separator_config,
        );
        sep.freeze_items(&[0, 1]);
        assert!(sep.solution_is_certified(&sep.prob.save(), &saved));
        assert_eq!(sep.min_feasible_width(), None);
        sep.change_strip_width(15.0, None);
        assert!(swarm_core::util::assertions::frozen_poses_preserved(
            &saved,
            &sep.prob.save(),
            &mask
        ));
        assert!(
            solution_validation_error_with_frozen(&instance, &sep.prob.save(), &mask).is_none()
        );
        sep.rollback(&saved, None);
        assert_eq!(sep.ct.get_total_loss(), 0.0);
        assert!(
            solution_validation_error_with_frozen(&instance, &sep.prob.save(), &mask).is_none()
        );
    }
}

#[test]
fn movable_against_lock_and_movable_exterior_are_still_rejected() {
    for positions in [[(-1.0, 4.0), (1.0, 4.0)], [(5.0, 5.0), (1000.0, 5.0)]] {
        let (instance, prob) = problem(&positions);
        let mask = [true, false];
        assert!(CollisionTracker::new_with_frozen(&prob.layout(), &mask).get_total_loss() > 0.0);
        assert!(solution_validation_error_with_frozen(&instance, &prob.save(), &mask).is_some());
    }
}

#[test]
fn frozen_nonfinite_export_transform_is_rejected() {
    let (instance, prob) = problem(&[(5.0, 5.0)]);
    for xy in [(f32::NEG_INFINITY, 5.0), (5.0, f32::INFINITY)] {
        let mut solution = prob.save();
        solution
            .layout_snapshot_mut()
            .placed_items_mut()
            .values_mut()
            .next()
            .unwrap()
            .set_d_transf(DTransformation::new(0.0, xy));
        assert!(solution_validation_error_with_frozen(&instance, &solution, &[true]).is_some());
    }
}

#[test]
fn all_frozen_layout_keeps_positive_width_and_rejects_invalid_resize() {
    let (instance, prob) = problem(&[(-1000.0, -1000.0), (-1000.0, -1000.0)]);
    let mut sep = Separator::new(
        instance,
        prob,
        Xoshiro256PlusPlus::seed_from_u64(42),
        DEFAULT_SWARM_CONFIG.expl_cfg.separator_config,
    );
    sep.freeze_items(&[0, 1]);
    let saved = sep.prob.save();
    assert_eq!(sep.min_feasible_width(), Some(saved.strip_width()));
    for width in [0.0, -1.0, f32::INFINITY, f32::NAN] {
        assert!(!sep.change_strip_width_with_control(width, None, &mut || false));
        assert_eq!(sep.prob.strip_width(), saved.strip_width());
        assert!(swarm_core::util::assertions::frozen_poses_preserved(
            &saved,
            &sep.prob.save(),
            &sep.frozen_mask
        ));
    }
}
