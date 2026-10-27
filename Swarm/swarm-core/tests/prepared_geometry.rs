#[allow(dead_code)]
#[path = "support/legacy_uniform_sampler.rs"]
mod legacy_uniform_sampler;

use jagua_rs::entities::{Item};
use jagua_rs::geometry::geo_enums::RotationRange;
use jagua_rs::geometry::geo_traits::Transformable;
use jagua_rs::geometry::primitives::Rect;
use jagua_rs::geometry::{DTransformation, Transformation};
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem};
use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
use rand::{RngExt, SeedableRng, rngs::Xoshiro256PlusPlus};
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::prepared_geometry::{PreparedGeometry, PreparedItemGeometry, ShapeFactor};
use swarm_core::quantify::{
    quantify_collision_poly_container, quantify_collision_poly_container_with_penalty,
    quantify_collision_poly_poly, quantify_collision_poly_poly_with_penalty,
};
use swarm_core::sample::uniform_sampler::UniformBBoxSampler;

/// jagua-rs 1.x items carry `AllowedOrientations`; tests only use rotations.
fn ao(rotations: RotationRange) -> jagua_rs::geometry::AllowedOrientations {
    jagua_rs::geometry::AllowedOrientations::new(rotations, vec![]).unwrap()
}

fn fixture() -> SPInstance {
    let ext: ExtSPInstance =
        serde_json::from_value(serde_json::json!({"name":"prepared","strip_height":30.,
        "items":[{"id":0,"demand":1,"allowed_orientations":[0.],"shape":{"type":"rectangle",
        "data":{"x_min":0.,"y_min":0.,"width":7.,"height":5.}}},
        {"id":1,"demand":1,"allowed_orientations":[0.],"shape":{"type":"rectangle",
        "data":{"x_min":0.,"y_min":0.,"width":9.,"height":3.}}}]}))
        .unwrap();
    let c = DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&importer, &ext).unwrap()
}
fn bits(dt: DTransformation) -> [u32; 3] {
    [
        dt.rotation().to_bits(),
        dt.translation().0.to_bits(),
        dt.translation().1.to_bits(),
    ]
}
fn compare_sampler(item: &Item, prepared: &PreparedItemGeometry, sample: Rect, container: Rect) {
    let old = legacy_uniform_sampler::UniformBBoxSampler::new(sample, item, container);
    let new = UniformBBoxSampler::new_with_prepared(sample, item, container, prepared);
    assert_eq!(old.is_some(), new.is_some());
    if let (Some(old), Some(new)) = (old, new) {
        for bias in [1., 2.7] {
            let mut a = Xoshiro256PlusPlus::seed_from_u64(982);
            let mut b = a.clone();
            for _ in 0..128 {
                let (x, y) = if bias == 1. {
                    (old.sample(&mut a), new.sample(&mut b))
                } else {
                    (
                        old.sample_x_biased(&mut a, bias),
                        new.sample_x_biased(&mut b, bias),
                    )
                };
                assert_eq!(bits(x), bits(y));
            }
            assert_eq!(a.random::<[u64; 4]>(), b.random::<[u64; 4]>());
        }
    }
}

#[test]
fn prepared_sampler_preserves_rotation_order_sample_bits_and_rng() {
    let instance = fixture();
    let container = Rect::try_new(0., 0., 40., 30.).unwrap();
    for rotation in [
        RotationRange::None,
        RotationRange::Discrete(vec![0., -0., 1.7, 0., 0.4]),
        RotationRange::Continuous,
    ] {
        let item = instance.item(0).as_ref().clone().with_allowed_orientations(ao(rotation));
        let prepared = PreparedItemGeometry::new(&item);
        for sample in [
            container,
            Rect::try_new(8., 9., 13., 14.).unwrap(),
            Rect::try_new(-100., -100., -99., -99.).unwrap(),
        ] {
            compare_sampler(&item, &prepared, sample, container);
        }
        compare_sampler(&item, &prepared, item.shape_cd().bbox(), item.shape_cd().bbox());
    }
}

#[test]
fn prepared_sampler_rejects_replaced_arcs_and_bit_changed_rotations() {
    let instance = fixture();
    // jagua-rs 1.x normalizes and deduplicates rotation lists at construction, so a
    // bit-level change (0. vs -0.) can no longer be expressed; a value change is used.
    let mut item = instance.item(0).as_ref().clone().with_allowed_orientations(ao(RotationRange::Discrete(vec![0., 0.7])));
    let prepared = PreparedItemGeometry::new(&item);
    assert!(prepared.matches(&item));
    let container = Rect::try_new(0., 0., 40., 30.).unwrap();
    item = item.with_allowed_orientations(ao(RotationRange::Discrete(vec![0., 0.71])));
    assert!(!prepared.matches(&item));
    compare_sampler(&item, &prepared, container, container);
    item = item.with_shape_cd(instance.item(1).shape_cd().clone());
    assert!(!prepared.matches(&item));
    compare_sampler(&item, &prepared, container, container);
    // A same-valued replacement Arc must be treated as a new geometry owner.
    let newer = PreparedItemGeometry::new(&item);
    let replaced = std::sync::Arc::new(item.shape_cd().as_ref().clone());
    item = item.with_shape_cd(replaced);
    assert!(!newer.matches(&item));
}

#[test]
fn cached_shape_factors_preserve_scalar_container_and_replacement_bits() {
    let instance = fixture();
    let a = instance.item(0).shape_cd().as_ref();
    let b = instance.item(1).shape_cd().as_ref();
    let fa = ShapeFactor::new(a);
    let fb = ShapeFactor::new(b);
    let container = Rect::try_new(0., 0., 40., 30.).unwrap();
    for angle in [0., 0.31, 1.7] {
        let moved = b
            .clone()
            .transform(&Transformation::from_rotation(angle))
            .clone();
        let penalty = (fa.root_for(a) * fb.root_for(&moved)).sqrt();
        assert_eq!(
            quantify_collision_poly_poly(a, &moved).to_bits(),
            quantify_collision_poly_poly_with_penalty(a, &moved, penalty).to_bits()
        );
        let root = fb.root_for(&moved);
        assert_eq!(
            quantify_collision_poly_container(&moved, container).to_bits(),
            quantify_collision_poly_container_with_penalty(&moved, container, (root * root).sqrt())
                .to_bits()
        );
        // Geometry replacement with the same slot/id must not reuse old area bits.
        assert_eq!(
            fa.root_for(&moved).to_bits(),
            moved.surrogate().convex_hull_area.sqrt().to_bits()
        );
    }
}

#[test]
fn prepared_full_search_preserves_result_evaluations_and_rng() {
    use swarm_core::eval::sample_eval::{SampleEval, SampleEvaluator};
    use swarm_core::sample::search::{self, SampleConfig};
    #[derive(Default)]
    struct Eval(usize);
    impl SampleEvaluator for Eval {
        fn evaluate_sample(&mut self, dt: DTransformation, _: Option<SampleEval>) -> SampleEval {
            self.0 += 1;
            let (x, y) = dt.translation();
            SampleEval::Clear {
                loss: (x - 8.3).powi(2) + (y - 9.7).powi(2) + dt.rotation().sin().abs(),
            }
        }
        fn n_evals(&self) -> usize {
            self.0
        }
    }
    let mut instance = fixture();
    instance.items[0].0 = std::sync::Arc::new(instance.items[0].0.as_ref().clone().with_allowed_orientations(ao(RotationRange::Discrete(vec![0., 0.7]))));
    let prepared = PreparedGeometry::new(&instance);
    let mut problem = SPProblem::new(instance.clone()).expect("strip forms a valid container");
    problem.change_strip_width(40.).expect("strip width must be positive");
    let pk = problem.place_item(SPPlacement {
        item_idx: 0,
        d_transf: DTransformation::new(0., (12., 13.)),
    });
    let cfg = SampleConfig {
        n_container_samples: 16,
        n_focussed_samples: 8,
        n_coord_descents: 2,
        left_bias_gain: 2.,
        tilt_range: 0.,
    };
    for reference in [None, Some(pk)] {
        let mut a = Xoshiro256PlusPlus::seed_from_u64(765);
        let mut b = a.clone();
        let (old, n_old) = search::search_placement_with_control(
            &problem.layout(),
            instance.item(0),
            reference,
            Eval::default(),
            cfg,
            &mut a,
            &mut || false,
        );
        let (new, n_new) = search::search_placement_prepared_with_control(
            &problem.layout(),
            instance.item(0),
            reference,
            Eval::default(),
            cfg,
            &mut b,
            &mut || false,
            &prepared,
        );
        assert_eq!(n_old, n_new);
        assert_eq!(
            old.map(|(dt, ev)| (bits(dt), ev)),
            new.map(|(dt, ev)| (bits(dt), ev))
        );
        assert_eq!(a.random::<[u64; 4]>(), b.random::<[u64; 4]>());
    }
}

#[test]
fn tracker_prepared_losses_match_reference_before_and_after_rotated_moves() {
    use swarm_core::optimizer::separator::Separator;
    use swarm_core::quantify::tracker::CollisionTracker;
    fn check(ct: &CollisionTracker, problem: &SPProblem) {
        for (pk, pi) in problem.layout().placed_items() {
            let idx = ct.pk_idx_map[pk];
            let entry = ct.container_collisions()[idx];
            if entry.loss != 0. {
                assert_eq!(
                    entry.loss.to_bits(),
                    quantify_collision_poly_container(
                        &pi.shape(),
                        problem.layout().container().outer_cd().bbox()
                    )
                    .to_bits()
                );
            }
            for (other, other_pi) in problem.layout().placed_items() {
                let value = ct.pair_collisions()[(idx, ct.pk_idx_map[other])].loss;
                if value != 0. {
                    // The tracker stores one value per unordered pair, computed
                    // with whichever shape it was quantifying at the time. The
                    // overlap proxy sums pole pairs in operand order, so the two
                    // orders can differ in the last bit; either one is exact.
                    let ab = quantify_collision_poly_poly(&pi.shape(), &other_pi.shape()).to_bits();
                    let ba = quantify_collision_poly_poly(&other_pi.shape(), &pi.shape()).to_bits();
                    assert!(
                        value.to_bits() == ab || value.to_bits() == ba,
                        "pair loss {value} matches neither operand order ({}, {})",
                        f32::from_bits(ab),
                        f32::from_bits(ba)
                    );
                }
            }
        }
    }
    let instance = fixture();
    let mut problem = SPProblem::new(instance.clone()).expect("strip forms a valid container");
    problem.change_strip_width(40.).expect("strip width must be positive");
    for id in 0..2 {
        problem.place_item(SPPlacement {
            item_idx: id,
            d_transf: DTransformation::new(0., (2. + id as f32, 2.)),
        });
    }
    let mut cfg = DEFAULT_SWARM_CONFIG.expl_cfg.separator_config;
    cfg.n_workers = 1;
    let mut sep = Separator::new(
        instance,
        problem,
        Xoshiro256PlusPlus::seed_from_u64(129),
        cfg,
    );
    check(&sep.ct, &sep.prob);
    for angle in [0.2, 1.1, 2.3] {
        let pk = sep
            .prob
            .layout()
            .placed_items()
            .iter()
            .find(|(_, pi)| pi.item().idx() == 0)
            .unwrap()
            .0;
        sep.move_item(pk, DTransformation::new(angle, (2., 1.)));
        check(&sep.ct, &sep.prob);
        let fresh = CollisionTracker::new(&sep.prob.layout());
        assert_eq!(
            fresh.get_total_loss().to_bits(),
            sep.ct.get_total_loss().to_bits()
        );
    }
}

#[test]
#[ignore = "isolated immutable preparation microbenchmark; coordinate Cargo slot"]
fn prepared_sampler_construction_microbenchmark() {
    let instance = fixture();
    let item = instance.item(0).as_ref().clone().with_allowed_orientations(ao(RotationRange::Continuous));
    let prepared = PreparedItemGeometry::new(&item);
    let bbox = Rect::try_new(0., 0., 40., 30.).unwrap();
    let mut expected_checksum = None;
    for cached in [false, true] {
        let start = std::time::Instant::now();
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(333);
        let mut checksum = 0u32;
        for _ in 0..20000 {
            let dt = if cached {
                UniformBBoxSampler::new_with_prepared(bbox, &item, bbox, &prepared)
                    .unwrap()
                    .sample(&mut rng)
            } else {
                legacy_uniform_sampler::UniformBBoxSampler::new(bbox, &item, bbox)
                    .unwrap()
                    .sample(&mut rng)
            };
            checksum = checksum.wrapping_add(std::hint::black_box(bits(dt)[1]));
        }
        println!(
            "prepared={cached} iterations=20000 elapsed_ms={:.3} checksum={checksum}",
            start.elapsed().as_secs_f64() * 1000.
        );
        if let Some(expected) = expected_checksum {
            assert_eq!(checksum, expected);
        } else {
            expected_checksum = Some(checksum);
        }
    }
}
