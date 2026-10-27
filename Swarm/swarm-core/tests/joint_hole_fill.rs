use jagua_rs::geometry::DTransformation;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution};
use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
use rand::{SeedableRng, rngs::Xoshiro256PlusPlus};
use swarm_core::config::{DEFAULT_POLISH_CONFIG, DEFAULT_SWARM_CONFIG, HoleFillConfig};
use swarm_core::optimizer::place_groups::{PlaceGroupMember, PlaceGroupSpec, PlaceGroups};
use swarm_core::optimizer::polish::holes::{hole_fill_pass, hole_fill_pass_with_nfp};
use swarm_core::optimizer::separator::Separator;
use swarm_core::util::assertions::tracker_matches_layout;
use swarm_core::util::terminator::BasicTerminator;

// Two incompatible pockets: left is 6 x 7, right is 5 x 4. The rightmost
// 3 x 3 donor fits either, while the 5 x 5 donor can use only the left pocket.
fn fixture(group: bool, repeated: bool) -> (SPInstance, Separator) {
    let walls = [
        (0., 0., 14., 1.),
        (0., 8., 14., 1.),
        (0., 1., 1., 7.),
        (7., 1., 1., 7.),
        (13., 1., 1., 7.),
        (8., 5., 5., 3.),
    ];
    let mut specs: Vec<(f32, f32, usize)> = walls.iter().map(|w| (w.2, w.3, 1)).collect();
    if repeated {
        specs.push((3., 3., 2));
    } else {
        specs.push((5., 5., 1));
        specs.push((if group { 1. } else { 3. }, 3., 1));
        if group {
            specs.push((1., 3., 1));
        }
    }
    let items: Vec<_> = specs
        .iter()
        .enumerate()
        .map(|(id, &(w, h, demand))| {
            serde_json::json!({
        "id":id,"demand":demand,"allowed_orientations":[0.],"shape":{"type":"rectangle",
        "data":{"x_min":0.,"y_min":0.,"width":w,"height":h}}})
        })
        .collect();
    let ext: ExtSPInstance = serde_json::from_value(
        serde_json::json!({"name":"joint-pocket-trap","strip_height":9.,"items":items}),
    )
    .unwrap();
    let c = DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
    let instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext).unwrap();
    let mut problem = SPProblem::new(instance.clone()).expect("strip forms a valid container");
    problem.change_strip_width(32.).expect("strip width must be positive");
    let mut place = |id: usize, x: f32, y: f32| {
        let b = instance.item(id).shape_cd().bbox();
        problem.place_item(SPPlacement {
            item_idx: id,
            d_transf: DTransformation::new(0., (x - b.x_min, y - b.y_min)),
        });
    };
    for (id, &(x, y, _, _)) in walls.iter().enumerate() {
        place(id, x, y);
    }
    place(6, 20., 1.);
    place(if repeated { 6 } else { 7 }, 28., 1.);
    if group {
        place(8, 30., 1.);
    }
    let mut sc = c.expl_cfg.separator_config;
    sc.n_workers = 1;
    let mut sep = Separator::new(
        instance.clone(),
        problem,
        Xoshiro256PlusPlus::seed_from_u64(109),
        sc,
    );
    sep.freeze_items(&[0, 1, 2, 3, 4, 5]);
    if group {
        let members = [7, 8]
            .map(|id| PlaceGroupMember {
                item_id: id,
                initial_transform: pose(&sep.prob.save(), id),
            })
            .to_vec();
        sep.set_place_groups(
            PlaceGroups::new(&instance, vec![PlaceGroupSpec { id: 10, members }]).unwrap(),
        )
        .unwrap();
    }
    assert!(sep.solution_is_certified(&sep.prob.save(), &sep.prob.save()));
    (instance, sep)
}
fn cfg() -> HoleFillConfig {
    let mut c = DEFAULT_POLISH_CONFIG.hole_fill;
    c.candidate_band = 1.;
    c.max_candidates = 4;
    c.max_pockets_per_item = 8;
    c.n_samples_per_pocket = 512;
    c.n_coord_descents = 1;
    c.cell_size_ratio = 0.25;
    c.max_cells = 100_000;
    c
}
fn pose(sol: &SPSolution, id: usize) -> DTransformation {
    sol.layout_snapshot()
        .placed_items()
        .values()
        .find(|p| p.item().idx() == id)
        .unwrap()
        .d_transf()
}
fn certify(sep: &Separator, before: &SPSolution) {
    assert!(sep.solution_is_certified(&sep.prob.save(), before));
    assert!(sep.prob.strip_width() <= before.strip_width());
    assert_eq!(
        sep.prob.layout().placed_items().len(),
        before.layout_snapshot().placed_items().len()
    );
    assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout()));
}

#[test]
fn retired_hole_fill_is_inert_even_with_enabled_raster_and_nfp_flags() {
    use rand::RngExt;
    use swarm_core::optimizer::polish::holes::NfpHoleCache;
    for (group, repeated) in [(false, false), (true, false), (false, true)] {
        for exact in [false, true] {
            let (_, mut sep) = fixture(group, repeated);
            let before = sep.prob.save();
            let mut rng = sep.rng.clone();
            let mut config = cfg();
            config.enabled = true;
            let mut nfp = DEFAULT_SWARM_CONFIG.nfp_integration;
            nfp.nfp_hole_filling = exact;
            let mut cache = NfpHoleCache::new();
            assert_eq!(
                hole_fill_pass(&mut sep, &config, &BasicTerminator::new()),
                0
            );
            assert_eq!(
                hole_fill_pass_with_nfp(
                    &mut sep,
                    &config,
                    &nfp,
                    Some(&mut cache),
                    &BasicTerminator::new()
                ),
                0
            );
            assert_eq!(cache.feasible_region_calls(), 0);
            assert_eq!(sep.rng.random::<u64>(), rng.random::<u64>());
            assert_eq!(before.strip_width(), sep.prob.strip_width());
            for (key, item) in before.layout_snapshot().placed_items() {
                assert_eq!(item.d_transf(), sep.prob.layout().placed_items()[key].d_transf());
            }
            certify(&sep, &before);
        }
    }
}
#[test]
fn a_retired_hole_only_polish_phase_returns_without_rounds() {
    let (instance, mut sep) = fixture(true, false);
    let before = sep.prob.save();
    let mut config = DEFAULT_POLISH_CONFIG;
    config.enabled = true;
    config.lp.enabled = false;
    config.slide.enabled = false;
    config.hole_fill.enabled = true;
    let (result, stats) = swarm_core::optimizer::polish::polish_phase_measured(
        &instance,
        &mut sep,
        &before,
        &mut None,
        &BasicTerminator::new(),
        &config,
        &DEFAULT_SWARM_CONFIG.nfp_integration,
        &DEFAULT_SWARM_CONFIG.lp_solver,
    );
    assert_eq!(stats.rounds, 0);
    assert_eq!(before.strip_width(), result.strip_width());
    assert_eq!(pose(&before, 7), pose(&result, 7));
    certify(&sep, &before);
}
