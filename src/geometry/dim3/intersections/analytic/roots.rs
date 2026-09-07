//! Closed-form root solves shared by the analytic curve entries.

use std::f64::consts::TAU;

/// Real roots of `a t^2 + b t + c`, ordered, with a near-double root reported once.
///
/// `tolerance` is on the discriminant relative to the coefficients: a pair
/// closer than that is a tangency, and reporting it twice would make a
/// grazing contact look like a crossing.
pub(super) fn quadratic_roots(a: f64, b: f64, c: f64, tolerance: f64) -> Vec<f64> {
    let scale = a.abs().max(b.abs()).max(c.abs());
    if scale <= f64::MIN_POSITIVE {
        return Vec::new();
    }
    if a.abs() <= tolerance * scale {
        if b.abs() <= tolerance * scale {
            return Vec::new();
        }
        return vec![-c / b];
    }
    let discriminant = b * b - 4.0 * a * c;
    // Comparing against the coefficient scale rather than zero keeps a
    // tangency from resolving into two roots a rounding error apart.
    if discriminant < -tolerance * scale * scale {
        return Vec::new();
    }
    if discriminant <= tolerance * scale * scale {
        return vec![-b / (2.0 * a)];
    }
    // The stable pairing: computing both roots from the same square root loses
    // the small one to cancellation when b dominates.
    let root = discriminant.sqrt();
    let stable = -0.5 * (b + b.signum() * root);
    let mut roots = vec![stable / a, c / stable];
    roots.sort_by(f64::total_cmp);
    roots
}

/// Angles in `[0, 2pi)` solving `a cos(t) + b sin(t) = c`.
///
/// Returns `None` when `a` and `b` both vanish, which is not a failure to
/// solve but a statement that the equation does not depend on `t`: either
/// every angle is a solution or none is, and only the caller knows which of
/// those its geometry means.
pub(super) fn harmonic_roots(a: f64, b: f64, c: f64, tolerance: f64) -> Option<Vec<f64>> {
    let amplitude = a.hypot(b);
    let scale = amplitude.max(c.abs());
    if scale <= f64::MIN_POSITIVE || amplitude <= tolerance * scale {
        return None;
    }
    let phase = b.atan2(a);
    let ratio = c / amplitude;
    if ratio > 1.0 + tolerance {
        return Some(Vec::new());
    }
    if ratio < -1.0 - tolerance {
        return Some(Vec::new());
    }
    let clamped = ratio.clamp(-1.0, 1.0);
    // A ratio at the limit is a tangency: one angle, not two coincident ones.
    if (clamped.abs() - 1.0).abs() <= tolerance {
        return Some(vec![wrapped(
            phase + if clamped > 0.0 { 0.0 } else { TAU / 2.0 },
        )]);
    }
    let offset = clamped.acos();
    let mut roots = vec![wrapped(phase + offset), wrapped(phase - offset)];
    roots.sort_by(f64::total_cmp);
    Some(roots)
}

/// Folds an angle into `[0, 2pi)`.
pub(super) fn wrapped(angle: f64) -> f64 {
    angle.rem_euclid(TAU)
}
