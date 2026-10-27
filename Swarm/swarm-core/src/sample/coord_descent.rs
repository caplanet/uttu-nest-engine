use crate::consts::{CD_STEP_FAIL, CD_STEP_SUCCESS};
use crate::eval::sample_eval::{SampleEval, SampleEvaluator};
use jagua_rs::geometry::DTransformation;
use rand::{Rng, RngExt};
use std::cmp::Ordering;
use std::fmt::Debug;

#[derive(Clone, Debug, Copy)]
pub struct CDConfig {
    /// Initial step size for the coordinate descent
    pub t_step_init: f32,
    /// Limit for the step size, below which no more candidates are generated
    pub t_step_limit: f32,
    /// Initial step size for the rotation wiggle axis
    pub r_step_init: f32,
    /// Limit for the rotation wiggle step size, below which no more candidates are generated
    pub r_step_limit: f32,
    /// Defines whether the wiggle axis (rotation) is enabled
    pub wiggle: bool,
    /// 시작 회전으로부터 허용되는 최대 회전 편차 (rad). `None`이면 무제한 (연속 회전).
    /// discrete 회전 아이템에 소각도 틸트를 허용할 때 쓴다 (capmods tilt 옵션).
    pub r_range: Option<f32>,
}

/// Returns the best evaluated pose when cancellation interrupts a refinement.
pub fn refine_coord_desc_with_control(
    start: (DTransformation, SampleEval),
    evaluator: &mut impl SampleEvaluator,
    cd_config: CDConfig,
    rng: &mut impl Rng,
    should_stop: &mut dyn FnMut() -> bool,
) -> (DTransformation, SampleEval) {
    refine_coord_desc_impl(start, evaluator, cd_config, None, rng, should_stop)
}

/// Refine around an immutable declared rotation, so repeated searches cannot
/// walk the allowed tilt window away from the configured orientation.
pub(crate) fn refine_coord_desc_around_rotation_with_control(
    start: (DTransformation, SampleEval),
    evaluator: &mut impl SampleEvaluator,
    cd_config: CDConfig,
    rotation_anchor: f32,
    rng: &mut impl Rng,
    should_stop: &mut dyn FnMut() -> bool,
) -> (DTransformation, SampleEval) {
    refine_coord_desc_impl(
        start,
        evaluator,
        cd_config,
        Some(rotation_anchor),
        rng,
        should_stop,
    )
}

fn refine_coord_desc_impl(
    (init_dt, init_eval): (DTransformation, SampleEval),
    evaluator: &mut impl SampleEvaluator,
    cd_config: CDConfig,
    rotation_anchor: Option<f32>,
    rng: &mut impl Rng,
    should_stop: &mut dyn FnMut() -> bool,
) -> (DTransformation, SampleEval) {
    let n_evals_init = evaluator.n_evals();
    let init_pos = init_dt;

    // Initialize the coordinate descent.
    let mut cd = CoordinateDescent {
        pos: init_pos,
        eval: init_eval,
        axis: CDAxis::random(rng, cd_config.wiggle),
        t_steps: (cd_config.t_step_init, cd_config.t_step_init),
        t_step_limit: cd_config.t_step_limit,
        r_step: cd_config.r_step_init,
        r_step_limit: cd_config.r_step_limit,
        wiggle: cd_config.wiggle,
        // Direct callers retain the historical start-relative clamp. Search
        // passes an immutable allowed-orientation anchor instead.
        r_anchor_range: cd_config.r_range.map(|range| {
            (
                rotation_anchor.unwrap_or_else(|| init_pos.rotation()),
                range.max(0.0),
            )
        }),
    };

    // From the CD state, ask for candidate positions to evaluate. If none provided, stop.
    while let Some(c) = cd.ask() {
        if should_stop() || evaluator.n_evals().saturating_sub(n_evals_init) >= 1000 {
            break;
        }
        // Evaluate the candidates using the evaluator.
        let c_eval = c.map(|c| evaluator.evaluate_descent_sample(c, Some(cd.eval)));

        let best = c
            .into_iter()
            .zip(c_eval)
            .min_by_key(|(_, eval)| *eval)
            .expect(crate::dmsg!("At least one candidate should be present"));

        // Report the best candidate to the coordinate descent state.
        cd.tell(best, rng);
    }
    // Return the best transformation found by the coordinate descent.
    (cd.pos, cd.eval)
}

#[derive(Debug)]
struct CoordinateDescent {
    /// The current position in the coordinate descent
    pub pos: DTransformation,
    /// The current evaluation of the position
    pub eval: SampleEval,
    /// The current axis on which new candidates are generated
    pub axis: CDAxis,
    /// The current step size for x and y axes
    pub t_steps: (f32, f32),
    /// The current step size for the rotation wiggle axis
    pub r_step: f32,
    /// The limit for the step size, below which no more candidates are generated
    pub t_step_limit: f32,
    /// The limit for the rotation wiggle step size, below which no more candidates are generated
    pub r_step_limit: f32,
    /// Defines whether the wiggle axis is enabled
    pub wiggle: bool,
    /// 허용 회전의 `(anchor, half_range)` (rad). `None`이면 무제한.
    pub r_anchor_range: Option<(f32, f32)>,
}

impl CoordinateDescent {
    /// Generates candidates to be evaluated.
    pub fn ask(&self) -> Option<[DTransformation; 2]> {
        let (sx, sy) = self.t_steps;
        let sr = self.r_step;

        if sx < self.t_step_limit
            && sy < self.t_step_limit
            && (sr < self.r_step_limit || !self.wiggle)
        {
            // Stop generating candidates if both steps have reached the limit
            None
        } else {
            // Generate two candidates on either side of the current position, according to the active axis.
            let (tx, ty) = self.pos.translation();
            let r = self.pos.rotation();
            let transformations = match self.axis {
                CDAxis::Horizontal => [(tx + sx, ty, r), (tx - sx, ty, r)],
                CDAxis::Vertical => [(tx, ty + sy, r), (tx, ty - sy, r)],
                CDAxis::ForwardDiag => [(tx + sx, ty + sy, r), (tx - sx, ty - sy, r)],
                CDAxis::BackwardDiag => [(tx - sx, ty + sy, r), (tx + sx, ty - sy, r)],
                CDAxis::Wiggle => {
                    // 틸트 허용 범위가 있으면 각도 wrap을 고려해 anchor 주변으로
                    // 클램프한다. 경계에서는 제자리 후보가 되어 스텝만 줄어든다.
                    let clamp = |r: f32| match self.r_anchor_range {
                        Some((anchor, range)) => clamp_rotation_around_anchor(r, anchor, range),
                        None => r,
                    };
                    [(tx, ty, clamp(r + sr)), (tx, ty, clamp(r - sr))]
                }
            };

            let c = transformations.map(|(tx, ty, r)| DTransformation::new(r, (tx, ty)));

            Some(c)
        }
    }

    /// Updates the coordinate descent state with the new position and evaluation.
    pub fn tell(&mut self, (pos, eval): (DTransformation, SampleEval), rng: &mut impl Rng) {
        // Check if the reported evaluation is better or worse than the current one.
        let eval_cmp = eval.cmp(&self.eval);
        let better = eval_cmp == Ordering::Less;
        let worse = eval_cmp == Ordering::Greater;

        if !worse {
            // Update the current position if not worse
            (self.pos, self.eval) = (pos, eval);
        }

        // Determine the step size multiplier depending on whether the new evaluation is better or worse.
        let m = if better {
            CD_STEP_SUCCESS
        } else {
            CD_STEP_FAIL
        };

        // Apply the step size multiplier to the relevant steps for the current axis
        match self.axis {
            CDAxis::Horizontal => self.t_steps.0 *= m,
            CDAxis::Vertical => self.t_steps.1 *= m,
            CDAxis::ForwardDiag | CDAxis::BackwardDiag => {
                //Since both axis are involved, adjust both steps but less severely
                self.t_steps.0 *= m.sqrt();
                self.t_steps.1 *= m.sqrt();
            }
            CDAxis::Wiggle => {
                self.r_step *= m;
            }
        }

        // Every time a state is not improved, the axis gets changed to a new random one.
        if !better {
            self.axis = CDAxis::random(rng, self.wiggle);
        }
    }
}

/// Clamp `rotation` to `anchor ± half_range` on the circle.
///
/// The returned representation stays close to `rotation` (rather than jumping
/// by 2π) while its circular distance from the declared anchor is bounded.
pub(crate) fn clamp_rotation_around_anchor(rotation: f32, anchor: f32, half_range: f32) -> f32 {
    let delta = signed_angular_delta(rotation, anchor);
    let clamped_delta = delta.clamp(-half_range, half_range);
    rotation + (clamped_delta - delta)
}

fn signed_angular_delta(rotation: f32, anchor: f32) -> f32 {
    use std::f32::consts::PI;

    (rotation - anchor + PI).rem_euclid(2.0 * PI) - PI
}

#[derive(Clone, Debug, Copy)]
enum CDAxis {
    /// Left and right
    Horizontal,
    /// Up and down
    Vertical,
    /// Up-right and down-left
    ForwardDiag,
    /// Up-left and down-right
    BackwardDiag,
    /// Wiggle left and right (if allowed)
    Wiggle,
}

impl CDAxis {
    fn random(rng: &mut impl Rng, rotate: bool) -> Self {
        let range = if rotate {
            0..6 // Include wiggle as a possible axis
        } else {
            0..4 // Exclude wiggle if not allowed
        };
        match rng.random_range(range) {
            0 => CDAxis::Horizontal,
            1 => CDAxis::Vertical,
            2 => CDAxis::ForwardDiag,
            3 => CDAxis::BackwardDiag,
            4..6 => CDAxis::Wiggle,
            _ => unreachable!(),
        }
    }
}

#[cfg(test)]
mod budget_tests {
    use super::*;
    use rand::{SeedableRng, rngs::Xoshiro256PlusPlus};

    struct AlwaysImproving(usize);
    struct ApproximateTiePair(usize);
    impl SampleEvaluator for ApproximateTiePair {
        fn evaluate_sample(&mut self, _: DTransformation, _: Option<SampleEval>) -> SampleEval {
            panic!("descent must use its pair-preserving evaluation entry point")
        }
        fn evaluate_descent_sample(
            &mut self,
            _: DTransformation,
            _: Option<SampleEval>,
        ) -> SampleEval {
            let offset = if self.0 == 0 { 5 } else { 1 };
            self.0 += 1;
            SampleEval::Clear {
                loss: f32::from_bits(1000f32.to_bits() + offset),
            }
        }
        fn n_evals(&self) -> usize {
            self.0
        }
    }

    #[test]
    fn nontransitive_score_ties_preserve_the_rejected_first_candidate() {
        let mut evaluator = ApproximateTiePair(0);
        let mut iterations = 0;
        let start = (DTransformation::empty(), SampleEval::Clear { loss: 1000. });
        let result = refine_coord_desc_with_control(
            start,
            &mut evaluator,
            CDConfig {
                t_step_init: 1.,
                t_step_limit: 0.001,
                r_step_init: 0.1,
                r_step_limit: 0.001,
                wiggle: false,
                r_range: None,
            },
            &mut Xoshiro256PlusPlus::seed_from_u64(1),
            &mut || {
                iterations += 1;
                iterations > 1
            },
        );
        assert_eq!(evaluator.n_evals(), 2);
        assert_eq!(result, start);
    }

    impl SampleEvaluator for AlwaysImproving {
        fn evaluate_sample(&mut self, _: DTransformation, _: Option<SampleEval>) -> SampleEval {
            self.0 += 1;
            SampleEval::Clear {
                loss: -(self.0 as f32),
            }
        }
        fn n_evals(&self) -> usize {
            self.0
        }
    }

    #[test]
    fn pathological_refinement_has_a_release_budget_and_returns_the_best() {
        let config = CDConfig {
            t_step_init: 1.0,
            t_step_limit: 0.001,
            r_step_init: 0.1,
            r_step_limit: 0.001,
            wiggle: false,
            r_range: None,
        };
        let mut evaluator = AlwaysImproving(0);
        let (_, best) = refine_coord_desc_with_control(
            (DTransformation::empty(), SampleEval::Clear { loss: 0.0 }),
            &mut evaluator,
            config,
            &mut Xoshiro256PlusPlus::seed_from_u64(1),
            &mut || false,
        );
        assert_eq!(evaluator.n_evals(), 1000);
        assert_eq!(best, SampleEval::Clear { loss: -1000.0 });
    }
}
