use swarm_core::lp_solver::{
    LpBackend, LpProblem, LpRow, LpStatus, clarabel_backend::ClarabelBackend,
    microlp_backend::MicrolpBackend,
};
use std::time::Duration;

fn bounded_lp() -> LpProblem {
    LpProblem {
        objective: vec![0.0],
        lower: vec![0.0],
        upper: vec![1.0],
        rows: vec![],
        quadratic: 0.0,
    }
}

#[test]
fn impossible_infinity_directions_are_rejected_before_solver_entry() {
    for (lower, upper) in [
        (f64::INFINITY, f64::INFINITY),
        (f64::NEG_INFINITY, f64::NEG_INFINITY),
    ] {
        let mut p = bounded_lp();
        p.lower[0] = lower;
        p.upper[0] = upper;
        assert!(
            p.validate().is_err(),
            "invalid variable bounds {lower}..{upper}"
        );
        let mut p = bounded_lp();
        p.rows.push(LpRow {
            cols: vec![0],
            vals: vec![1.0],
            lower,
            upper,
        });
        assert!(p.validate().is_err(), "invalid row bounds {lower}..{upper}");
    }
}

#[test]
fn large_finite_bound_is_never_silently_dropped() {
    let mut p = bounded_lp();
    p.lower[0] = 1.0e21;
    p.upper[0] = 2.0e21;
    let sol = ClarabelBackend.solve(&p, Duration::from_secs(1));
    assert!(
        !sol.status.is_usable()
            || sol.x.len() == 1 && sol.x[0] >= p.lower[0] && sol.x[0] <= p.upper[0],
        "{sol:?}"
    );
}

#[test]
fn zero_coefficient_constraints_do_not_hide_infeasibility() {
    for backend in [&MicrolpBackend as &dyn LpBackend, &ClarabelBackend] {
        let mut p = bounded_lp();
        p.rows
            .push(LpRow::greater_or_equal(vec![0], vec![0.0], 1.0));
        let sol = backend.solve(&p, Duration::from_secs(1));
        assert_eq!(
            sol.status,
            LpStatus::Infeasible,
            "{}: {sol:?}",
            backend.name()
        );
        p.rows[0].lower = 0.0;
        assert_eq!(
            backend.solve(&p, Duration::from_secs(1)).status,
            LpStatus::Optimal
        );
    }
}

#[test]
fn uniform_sampler_keeps_zero_width_feasible_intervals() {
    use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
    use rand::{SeedableRng, rngs::Xoshiro256PlusPlus};
    use swarm_core::sample::uniform_sampler::UniformBBoxSampler;
    let ext: ExtSPInstance =
        serde_json::from_value(serde_json::json!({"name":"exact-fit","strip_height":4.0,
        "items":[{"id":0,"demand":1,"allowed_orientations":[0.0],"shape":{"type":"rectangle",
        "data":{"x_min":0.0,"y_min":0.0,"width":4.0,"height":4.0}}}]}))
        .unwrap();
    let c = swarm_core::config::DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
    let instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext).unwrap();
    let item = instance.item(0);
    let bbox = item.shape_cd().bbox();
    let sampler =
        UniformBBoxSampler::new(bbox, item, bbox).expect("an exact-fitting placement is feasible");
    let mut rng = Xoshiro256PlusPlus::seed_from_u64(1);
    for _ in 0..16 {
        assert_eq!(sampler.sample(&mut rng).translation(), (0.0, 0.0));
        assert_eq!(
            sampler.sample_x_biased(&mut rng, 2.0).translation(),
            (0.0, 0.0)
        );
    }
    let mut too_small = bbox;
    too_small.x_max -= 0.25;
    assert!(UniformBBoxSampler::new(bbox, item, too_small).is_none());
    let mut wider = bbox;
    wider.x_max += 4.0;
    let one_axis = UniformBBoxSampler::new(wider, item, wider).unwrap();
    for _ in 0..32 {
        let (x, y) = one_axis.sample(&mut rng).translation();
        assert!((0.0..4.0).contains(&x));
        assert_eq!(y, 0.0);
    }
}

#[test]
fn primal_validation_rejects_nonfinite_truncated_and_infeasible_results() {
    let mut p = bounded_lp();
    p.rows
        .push(LpRow::greater_or_equal(vec![0], vec![2.0], 1.0));
    assert!(p.is_primal_feasible(&[0.75]));
    for x in [
        vec![],
        vec![0.75, 0.0],
        vec![f64::NAN],
        vec![f64::INFINITY],
        vec![0.0],
        vec![2.0],
    ] {
        assert!(!p.is_primal_feasible(&x), "{x:?}");
    }
    p.rows[0].cols[0] = 99;
    assert!(!p.is_primal_feasible(&[0.75]));
}

#[test]
fn sparse_row_duplicate_check_matches_set_oracle() {
    use std::collections::HashSet;
    for len in 1..=24 {
        for modulus in 1..=27 {
            let cols: Vec<_> = (0..len).map(|i| (i * 7) % modulus).collect();
            let expected = cols.iter().collect::<HashSet<_>>().len() == cols.len();
            let row = LpRow::less_or_equal(cols, vec![1.0; len], 3.0);
            assert_eq!(row.is_well_formed(), expected);
        }
    }
}

#[test]
fn built_in_lp_backends_match_vertex_enumeration() {
    for backend in [&MicrolpBackend as &dyn LpBackend, &ClarabelBackend] {
        for i in 1..=30 {
            let upper = 2.0 + (i % 4) as f64;
            let threshold = upper * (0.2 + (i % 5) as f64 * 0.3);
            let objective = vec![(i % 7) as f64 - 3.0, (i % 3) as f64 - 1.0];
            let p = LpProblem {
                objective,
                lower: vec![0.0; 2],
                upper: vec![upper; 2],
                rows: vec![LpRow::greater_or_equal(
                    vec![0, 1],
                    vec![1.0, 1.0],
                    threshold,
                )],
                quadratic: 0.0,
            };
            let points = [
                [0.0, 0.0],
                [0.0, upper],
                [upper, 0.0],
                [upper, upper],
                [0.0, threshold],
                [threshold, 0.0],
                [upper, threshold - upper],
                [threshold - upper, upper],
            ];
            let best = points
                .iter()
                .filter(|x| x.iter().all(|v| *v >= 0.0 && *v <= upper) && x[0] + x[1] >= threshold)
                .map(|x| x[0] * p.objective[0] + x[1] * p.objective[1])
                .reduce(f64::min)
                .unwrap();
            let sol = backend.solve(&p, Duration::from_secs(1));
            assert_eq!(
                sol.status,
                LpStatus::Optimal,
                "{} case {i}: {sol:?}",
                backend.name()
            );
            assert!(p.is_primal_feasible(&sol.x));
            assert!(
                (sol.objective - best).abs() < 1.0e-5,
                "{} case {i}: {sol:?}, oracle={best}",
                backend.name()
            );
        }
    }
}

#[test]
fn clarabel_quadratic_solution_matches_analytic_minimum() {
    let p = LpProblem {
        objective: vec![-2.0, -4.0, 1.0],
        lower: vec![0.0; 3],
        upper: vec![10.0; 3],
        rows: vec![],
        quadratic: 2.0,
    };
    let sol = ClarabelBackend.solve(&p, Duration::from_secs(1));
    assert_eq!(sol.status, LpStatus::Optimal);
    for (actual, expected) in sol.x.iter().zip([1.0, 2.0, 0.0]) {
        assert!((actual - expected).abs() < 1.0e-4, "{sol:?}");
    }
}

#[test]
fn irrelevant_free_column_does_not_turn_bounded_lp_into_unbounded() {
    let mut p = LpProblem {
        objective: vec![1.0, 0.0],
        lower: vec![f64::NEG_INFINITY; 2],
        upper: vec![f64::INFINITY; 2],
        rows: vec![LpRow::greater_or_equal(vec![0], vec![1.0], -3.0)],
        quadratic: 0.0,
    };
    let sol = MicrolpBackend.solve(&p, Duration::from_secs(1));
    assert_eq!(sol.status, LpStatus::Optimal, "{sol:?}");
    assert!((sol.objective + 3.0).abs() < 1.0e-7);
    assert!(p.is_primal_feasible(&sol.x));
    // A nonzero objective on the free column really is unbounded.
    p.objective[1] = -1.0;
    assert_eq!(
        MicrolpBackend.solve(&p, Duration::from_secs(1)).status,
        LpStatus::Unbounded
    );
}

#[test]
fn one_shot_backend_result_is_discarded_after_cancellation() {
    use std::sync::atomic::{AtomicBool, Ordering};
    struct CompletingBackend(AtomicBool);
    impl LpBackend for CompletingBackend {
        fn name(&self) -> &'static str {
            "test completion"
        }
        fn availability(&self) -> Result<(), String> {
            Ok(())
        }
        fn supports_quadratic(&self) -> bool {
            false
        }
        fn solve(&self, _: &LpProblem, _: Duration) -> swarm_core::lp_solver::LpSolution {
            self.0.store(true, Ordering::SeqCst);
            swarm_core::lp_solver::LpSolution {
                status: LpStatus::Optimal,
                x: vec![0.0],
                objective: 0.0,
                solved_by: self.name(),
            }
        }
    }
    let backend = CompletingBackend(AtomicBool::new(false));
    let result = backend.solve_with_control(&bounded_lp(), Duration::from_secs(1), &|| {
        backend.0.load(Ordering::SeqCst)
    });
    assert_eq!(result.status, LpStatus::Limit);
    assert!(result.x.is_empty());
}
