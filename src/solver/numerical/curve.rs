use super::differentiate::{Jet, JetVector2};
use nalgebra::Vector2;

/// A curve primitive as a differentiable map from its own parameter `t` to
/// the plane. Shape parameters are expressions of the unknowns, so `point_at`
/// and `derivative_at` differentiate through both `t` and the shape: a
/// relation written against them holds for every kind of curve.
// Built and consumed in place for each evaluation; boxing would allocate on the hot path.
#[allow(clippy::large_enum_variant)]
pub(super) enum Curve {
    /// `t` runs from 0 at `start` to 1 at `end`.
    Segment { start: JetVector2, end: JetVector2 },
    /// `t` is the angle from the imported frame's `axis`, turning with `sense`.
    Circle {
        center: JetVector2,
        radius: Jet,
        axis: Vector2<f64>,
        sense: f64,
    },
    /// `t` runs from 0 at the start to 1 at the end of the swept span.
    Arc {
        center: JetVector2,
        radius: Jet,
        start_angle: Jet,
        sweep: Jet,
        sense: f64,
    },
}

impl Curve {
    pub fn point_at(&self, t: Jet) -> JetVector2 {
        match self {
            Curve::Segment { start, end } => start.clone() + (end.clone() - start.clone()).scale(t),
            Curve::Circle {
                center,
                radius,
                axis,
                sense,
            } => {
                let (cos, sin) = (t.clone().cos(), t.sin());
                let radial = JetVector2 {
                    x: cos.clone() * axis.x - sin.clone() * (sense * axis.y),
                    y: cos * axis.y + sin * (sense * axis.x),
                };
                center.clone() + radial.scale(radius.clone())
            }
            Curve::Arc {
                center,
                radius,
                start_angle,
                sweep,
                sense,
            } => {
                let angle = swept_angle(start_angle, sweep, *sense, t);
                let radial = JetVector2 {
                    x: angle.clone().cos(),
                    y: angle.sin(),
                };
                center.clone() + radial.scale(radius.clone())
            }
        }
    }
    /// The derivative of `point_at` with respect to `t`.
    pub fn derivative_at(&self, t: Jet) -> JetVector2 {
        match self {
            Curve::Segment { start, end } => end.clone() - start.clone(),
            Curve::Circle {
                radius,
                axis,
                sense,
                ..
            } => {
                let (cos, sin) = (t.clone().cos(), t.sin());
                let tangent = JetVector2 {
                    x: -(sin.clone() * axis.x) - cos.clone() * (sense * axis.y),
                    y: cos * (sense * axis.x) - sin * axis.y,
                };
                tangent.scale(radius.clone())
            }
            Curve::Arc {
                radius,
                start_angle,
                sweep,
                sense,
                ..
            } => {
                let angle = swept_angle(start_angle, sweep, *sense, t);
                let tangent = JetVector2 {
                    x: -angle.clone().sin(),
                    y: angle.cos(),
                };
                tangent.scale(radius.clone() * sweep.clone() * *sense)
            }
        }
    }
}

/// The angle a fraction `t` of the way along an arc's sweep.
fn swept_angle(start_angle: &Jet, sweep: &Jet, sense: f64, t: Jet) -> Jet {
    start_angle.clone() + sweep.clone() * (t * sense)
}
