//! Literal Draft area scoring; separate from the legacy weighted-shortfall score.

use swarm_nfp::Point;

pub(super) fn value(area_mm2: f64, count: usize, reference_area_mm2: f64) -> Option<f64> {
    if !area_mm2.is_finite()
        || area_mm2 < 0.0
        || count == 0
        || !reference_area_mm2.is_finite()
        || reference_area_mm2 <= 0.0
    {
        return None;
    }
    let denominator = count as f64 * reference_area_mm2;
    denominator.is_finite().then_some(area_mm2 / denominator)
}

pub(super) fn surface_average(area_mm2: f64, strip_count: usize) -> Option<f64> {
    (area_mm2.is_finite() && area_mm2 >= 0.0 && strip_count != 0)
        .then(|| area_mm2 / strip_count as f64)
}

pub(super) fn candidate_score(v_max: f64, averages_mm2: &[f64]) -> Option<f64> {
    if !v_max.is_finite()
        || v_max < 0.0
        || averages_mm2.is_empty()
        || averages_mm2.iter().any(|a| !a.is_finite() || *a < 0.0)
    {
        return None;
    }
    let score = v_max - 8.0 * averages_mm2.iter().copied().fold(0.0, f64::max);
    score.is_finite().then_some(score)
}

/// Sutherland-Hodgman against the strip rectangle. Concave components may be
/// joined by cancelling boundary walks; signed shoelace still gives their sum.
/// This is a scoring calculation, never a substitute for collision validation.
pub(super) fn clipped_area(ring: &[Point], shift: (f64, f64), bounds: [f64; 4]) -> Option<f64> {
    if !shift.0.is_finite()
        || !shift.1.is_finite()
        || bounds.iter().any(|v| !v.is_finite())
        || ring.iter().any(|p| !p.x.is_finite() || !p.y.is_finite())
    {
        return None;
    }
    if ring.len() < 3 || bounds[0] >= bounds[2] || bounds[1] >= bounds[3] {
        return Some(0.0);
    }
    let mut points: Vec<_> = ring
        .iter()
        .map(|p| Point::new(p.x + shift.0, p.y + shift.1))
        .collect();
    let mut output = Vec::with_capacity(points.len() + 4);
    for (axis, edge, lower) in [
        (0, bounds[0], true),
        (0, bounds[2], false),
        (1, bounds[1], true),
        (1, bounds[3], false),
    ] {
        if points.is_empty() {
            return Some(0.0);
        }
        let coordinate = |p: Point| if axis == 0 { p.x } else { p.y };
        let inside = |p: Point| {
            if lower {
                coordinate(p) >= edge
            } else {
                coordinate(p) <= edge
            }
        };
        let mut previous = *points.last()?;
        let mut previous_inside = inside(previous);
        output.clear();
        for &current in &points {
            let current_inside = inside(current);
            if current_inside != previous_inside {
                let t =
                    (edge - coordinate(previous)) / (coordinate(current) - coordinate(previous));
                let mut p = Point::new(
                    previous.x + t * (current.x - previous.x),
                    previous.y + t * (current.y - previous.y),
                );
                if axis == 0 {
                    p.x = edge;
                } else {
                    p.y = edge;
                }
                output.push(p);
            }
            if current_inside {
                output.push(current);
            }
            previous = current;
            previous_inside = current_inside;
        }
        std::mem::swap(&mut points, &mut output);
    }
    // The 0.01 mm area grid matches Column Draft's existing area convention.
    // Reject overflow instead of silently wrapping or ranking a saturated area.
    let quantize = |v: f64| {
        let q = (v * 100.0).round();
        (q.is_finite() && q > i64::MIN as f64 && q < i64::MAX as f64).then_some(q as i64)
    };
    let mut twice = 0i128;
    for (&a, &b) in points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
    {
        let (ax, ay, bx, by) = (
            quantize(a.x)?,
            quantize(a.y)?,
            quantize(b.x)?,
            quantize(b.y)?,
        );
        twice =
            twice.checked_add((ax as i128 * by as i128).checked_sub(bx as i128 * ay as i128)?)?;
    }
    let area = i64::try_from(twice.checked_abs()? / 2).ok()?;
    Some(area as f64 / 10_000.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use swarm_nfp::Point;

    #[test]
    fn literal_area_average_is_not_efficiency_shortfall() {
        assert_eq!(value(2400.0, 3, 4000.0), Some(0.2));
        assert_eq!(surface_average(2400.0, 2), Some(1200.0));
        assert_eq!(candidate_score(0.8, &[1200.0, 900.0]), Some(-9599.2));
    }

    #[test]
    fn rejects_undefined_or_nonfinite_scores() {
        assert_eq!(value(10.0, 0, 100.0), None);
        assert_eq!(value(10.0, 1, 0.0), None);
        assert_eq!(value(f64::NAN, 1, 100.0), None);
        assert_eq!(value(10.0, 1, f64::INFINITY), None);
        assert_eq!(surface_average(10.0, 0), None);
        assert_eq!(candidate_score(1.0, &[]), None);
        assert_eq!(candidate_score(1.0, &[f64::NAN]), None);
    }

    #[test]
    fn clips_actual_concave_area_instead_of_its_box() {
        // L outline, area 32; clipping to x <= 5 leaves 5*2 + 2*7 = 24.
        let ring = [(0., 0.), (9., 0.), (9., 2.), (2., 2.), (2., 9.), (0., 9.)]
            .map(|(x, y)| Point::new(x, y));
        assert_eq!(clipped_area(&ring, (0., 0.), [0., 0., 5., 9.]), Some(24.0));
        assert_eq!(clipped_area(&ring, (10., 0.), [0., 0., 5., 9.]), Some(0.0));
    }

    #[test]
    fn clipping_preserves_disconnected_parts_of_a_concave_ring() {
        let ring = [
            (0., 0.),
            (6., 0.),
            (6., 6.),
            (4., 6.),
            (4., 2.),
            (2., 2.),
            (2., 6.),
            (0., 6.),
        ]
        .map(|(x, y)| Point::new(x, y));
        assert_eq!(clipped_area(&ring, (0., 0.), [0., 3., 6., 6.]), Some(12.0));
        let reversed: Vec<_> = ring.into_iter().rev().collect();
        assert_eq!(
            clipped_area(&reversed, (0., 0.), [0., 3., 6., 6.]),
            Some(12.0)
        );
    }

    #[test]
    fn large_area_uses_wide_intermediates_and_translation_is_applied() {
        let ring = [
            (0., 0.),
            (100_000., 0.),
            (100_000., 100_000.),
            (0., 100_000.),
        ]
        .map(|(x, y)| Point::new(x, y));
        assert_eq!(
            clipped_area(&ring, (100., 200.), [0., 0., 200_000., 200_000.]),
            Some(10_000_000_000.0)
        );
        assert_eq!(clipped_area(&ring, (f64::NAN, 0.), [0., 0., 1., 1.]), None);
    }
}
