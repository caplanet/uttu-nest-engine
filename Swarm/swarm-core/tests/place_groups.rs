use jagua_rs::geometry::{DTransformation, Transformation};
use jagua_rs::probs::spp::entities::SPInstance;
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::optimizer::lbf::LBFBuilder;
use swarm_core::optimizer::place_groups::{PlaceGroupMember, PlaceGroupSpec, PlaceGroups};
use swarm_core::optimizer::separator::Separator;
use swarm_core::util::assertions::tracker_matches_layout;
use swarm_core::util::io::read_spp_input;
use swarm_core::util::terminator::{BasicTerminator, Terminator};
use std::path::Path;
use std::time::Duration;

fn load_fu_instance() -> SPInstance {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input/fu.json");
    let (ext_inst, _) = read_spp_input(&path).expect("failed to read fu.json");
    let cfg = DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(cfg.cde_config, cfg.poly_simpl_tolerance, cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&importer, &ext_inst)
        .expect("failed to import instance")
}

fn two_square_group(instance: &SPInstance) -> PlaceGroups {
    PlaceGroups::new(
        instance,
        vec![PlaceGroupSpec {
            id: 77,
            members: vec![
                PlaceGroupMember {
                    item_id: 0,
                    initial_transform: DTransformation::empty(),
                },
                PlaceGroupMember {
                    item_id: 1,
                    initial_transform: DTransformation::new(0.0, (12.0, 0.0)),
                },
            ],
        }],
    )
    .expect("valid group")
}

fn relative(anchor: DTransformation, member: DTransformation) -> Transformation {
    member.compose().transform(&anchor.compose().inverse())
}

fn assert_transform_close(a: &Transformation, b: &Transformation) {
    for row in 0..3 {
        for col in 0..3 {
            let av = a.matrix()[row][col].into_inner();
            let bv = b.matrix()[row][col].into_inner();
            assert!(
                (av - bv).abs() < 1e-4,
                "matrix[{row}][{col}] differs: {av} vs {bv}"
            );
        }
    }
}

#[test]
fn fresh_lbf_places_group_as_one_rigid_unit() {
    let instance = load_fu_instance();
    let groups = two_square_group(&instance);
    let builder = LBFBuilder::new(
        instance.clone(),
        Xoshiro256PlusPlus::seed_from_u64(7),
        LBF_SAMPLE_CONFIG,
        10.0,
        1.0,
    )
    .with_place_groups(groups.clone())
    .expect("grouped builder")
    .construct();

    assert!(
        builder.prob.layout().is_collision_free(),
        "group-aware LBF must validate every member"
    );
    assert_eq!(
        builder.prob.layout().placed_items().len(),
        instance.total_item_qty()
    );

    let pk0 = groups.placement_key(&builder.prob.layout(), 0).unwrap();
    let pk1 = groups.placement_key(&builder.prob.layout(), 1).unwrap();
    let dt0 = builder.prob.layout().placed_items()[pk0].d_transf();
    let dt1 = builder.prob.layout().placed_items()[pk1].d_transf();
    let expected = relative(
        DTransformation::empty(),
        DTransformation::new(0.0, (12.0, 0.0)),
    );
    assert_transform_close(&relative(dt0, dt1), &expected);
}

#[test]
fn separator_move_and_freeze_apply_to_the_whole_group() {
    let instance = load_fu_instance();
    let groups = two_square_group(&instance);
    let builder = LBFBuilder::new(
        instance.clone(),
        Xoshiro256PlusPlus::seed_from_u64(11),
        LBF_SAMPLE_CONFIG,
        10.0,
        1.0,
    )
    .with_place_groups(groups.clone())
    .unwrap()
    .construct();
    let mut sep = Separator::new(
        instance,
        builder.prob,
        Xoshiro256PlusPlus::seed_from_u64(12),
        DEFAULT_SWARM_CONFIG.expl_cfg.separator_config,
    );
    sep.set_place_groups(groups).unwrap();

    let pk0 = sep.placement_key(0).unwrap();
    let pk1 = sep.placement_key(1).unwrap();
    let before0 = sep.prob.layout().placed_items()[pk0].d_transf();
    let before1 = sep.prob.layout().placed_items()[pk1].d_transf();
    let before_relative = relative(before0, before1);

    let target0 = DTransformation::new(
        before0.rotation() + std::f32::consts::FRAC_PI_2,
        (before0.translation().0 + 3.0, before0.translation().1 + 2.0),
    );
    sep.move_item(pk0, target0);
    let moved0 = sep.prob.layout().placed_items()[sep.placement_key(0).unwrap()].d_transf();
    let moved1 = sep.prob.layout().placed_items()[sep.placement_key(1).unwrap()].d_transf();
    assert_transform_close(&relative(moved0, moved1), &before_relative);
    assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));

    // Freezing either member expands to the complete placement unit.
    sep.freeze_items(&[1]);
    assert!(sep.frozen_mask[0] && sep.frozen_mask[1]);
    let frozen0 = sep.prob.layout().placed_items()[sep.placement_key(0).unwrap()].d_transf();
    let frozen1 = sep.prob.layout().placed_items()[sep.placement_key(1).unwrap()].d_transf();
    let frozen_pk0 = sep.placement_key(0).unwrap();
    sep.move_item(
        frozen_pk0,
        DTransformation::new(frozen0.rotation(), (0.0, 0.0)),
    );
    assert_eq!(
        sep.prob.layout().placed_items()[sep.placement_key(0).unwrap()].d_transf(),
        frozen0
    );
    assert_eq!(
        sep.prob.layout().placed_items()[sep.placement_key(1).unwrap()].d_transf(),
        frozen1
    );
}

#[test]
fn grouped_separation_preserves_relative_pose_and_tracker() {
    let instance = load_fu_instance();
    let groups = two_square_group(&instance);
    let builder = LBFBuilder::new(
        instance.clone(),
        Xoshiro256PlusPlus::seed_from_u64(21),
        LBF_SAMPLE_CONFIG,
        10.0,
        1.0,
    )
    .with_place_groups(groups.clone())
    .unwrap()
    .construct();
    let mut sep = Separator::new(
        instance,
        builder.prob,
        Xoshiro256PlusPlus::seed_from_u64(22),
        DEFAULT_SWARM_CONFIG.expl_cfg.separator_config,
    );
    sep.set_place_groups(groups).unwrap();
    let before0 = sep.prob.layout().placed_items()[sep.placement_key(0).unwrap()].d_transf();
    let before1 = sep.prob.layout().placed_items()[sep.placement_key(1).unwrap()].d_transf();
    let before_relative = relative(before0, before1);

    sep.change_strip_width(sep.prob.strip_width() * 0.85, None);
    let mut term = BasicTerminator::new();
    term.new_timeout(Duration::from_secs(1));
    sep.separate(&term, &mut None);

    let after0 = sep.prob.layout().placed_items()[sep.placement_key(0).unwrap()].d_transf();
    let after1 = sep.prob.layout().placed_items()[sep.placement_key(1).unwrap()].d_transf();
    assert_transform_close(&relative(after0, after1), &before_relative);
    assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));
}
