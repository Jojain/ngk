use crate::geometry::dim2::utils::Point2;
use nalgebra::Vector2;
use std::f64::consts::{PI, TAU};

/// How a stretch of a drawn curve runs in the plane, in the direction it is walked.
///
/// Curves are lines and circles, so what the walk needs is exact: which way a path leaves
/// its start, how sharply it turns, the area it sweeps, the angle it subtends.
#[derive(Clone, Copy, Debug)]
pub(super) enum Path {
    Line {
        start: Point2,
        end: Point2,
    },
    /// A run of a circle between two distinct points. `sweep` is signed, positive
    /// counter-clockwise.
    Arc {
        center: Point2,
        radius: f64,
        start: Point2,
        end: Point2,
        sweep: f64,
    },
    /// All the way round. `corner` is where it closes, if anything else meets it there.
    Circle {
        center: Point2,
        radius: f64,
        counterclockwise: bool,
        corner: Option<Point2>,
    },
}

impl Path {
    /// The arc from `start` to `end` going round `center` the given way.
    pub fn arc(
        center: Point2,
        radius: f64,
        start: Point2,
        end: Point2,
        counterclockwise: bool,
    ) -> Self {
        let (from, to) = (start - center, end - center);
        let angle = cross(from, to).atan2(from.dot(&to));
        let sweep = match (counterclockwise, angle < 0.) {
            (true, true) => angle + TAU,
            (false, false) => angle - TAU,
            _ => angle,
        };
        Self::Arc {
            center,
            radius,
            start,
            end,
            sweep,
        }
    }

    /// The same path walked the other way.
    pub fn reversed(self) -> Self {
        match self {
            Self::Line { start, end } => Self::Line {
                start: end,
                end: start,
            },
            Self::Arc {
                center,
                radius,
                start,
                end,
                sweep,
            } => Self::Arc {
                center,
                radius,
                start: end,
                end: start,
                sweep: -sweep,
            },
            Self::Circle {
                center,
                radius,
                counterclockwise,
                corner,
            } => Self::Circle {
                center,
                radius,
                counterclockwise: !counterclockwise,
                corner,
            },
        }
    }

    /// This path continued by `next`, which starts where this one ends. Both run along
    /// one curve. `closes` says the two ends they leave free are one point.
    pub fn joined(self, next: Self, closes: bool) -> Self {
        match (self, next) {
            (Self::Line { start, .. }, Self::Line { end, .. }) => Self::Line { start, end },
            (
                Self::Arc {
                    center,
                    radius,
                    start,
                    sweep,
                    ..
                },
                Self::Arc {
                    end, sweep: more, ..
                },
            ) => {
                if closes {
                    Self::Circle {
                        center,
                        radius,
                        counterclockwise: sweep > 0.,
                        corner: Some(start),
                    }
                } else {
                    Self::Arc {
                        center,
                        radius,
                        start,
                        end,
                        sweep: sweep + more,
                    }
                }
            }
            _ => unreachable!("the stretches of one curve share one kind"),
        }
    }

    /// The direction this path leaves its start in, as an angle, and its signed curvature
    /// there. Together they order the paths leaving one node: a tangent alone ties where
    /// an arc leaves a segment tangentially.
    pub fn leaving(self) -> (f64, f64) {
        match self {
            Self::Line { start, end } => {
                let direction = end - start;
                (direction.y.atan2(direction.x), 0.)
            }
            Self::Arc {
                center,
                radius,
                start,
                sweep,
                ..
            } => round_leaving(center, radius, start, sweep > 0.),
            Self::Circle {
                center,
                radius,
                counterclockwise,
                corner,
            } => round_leaving(
                center,
                radius,
                corner.unwrap_or(center + Vector2::x() * radius),
                counterclockwise,
            ),
        }
    }

    /// The signed area this path contributes to a closed walk: positive overall means the
    /// walk runs counter-clockwise.
    pub fn signed_area(self) -> f64 {
        match self {
            Self::Line { start, end } => chord_area(start, end),
            Self::Arc {
                radius,
                start,
                end,
                sweep,
                ..
            } => chord_area(start, end) + radius * radius / 2. * (sweep - sweep.sin()),
            Self::Circle {
                radius,
                counterclockwise,
                ..
            } => PI * radius * radius * if counterclockwise { 1. } else { -1. },
        }
    }

    /// A point on this path.
    pub fn point(self) -> Point2 {
        match self {
            Self::Line { start, .. } | Self::Arc { start, .. } => start,
            Self::Circle {
                center,
                radius,
                corner,
                ..
            } => corner.unwrap_or(center + Vector2::x() * radius),
        }
    }

    /// The angle this path turns through as seen from `point`, which is not on it. Summed
    /// over a closed walk this is a multiple of a full turn, the walk's winding number.
    pub fn subtended_at(self, point: Point2) -> f64 {
        match self {
            Self::Line { start, end } => chord_angle(start, end, point),
            Self::Arc {
                center,
                radius,
                start,
                end,
                sweep,
            } => {
                let chord = chord_angle(start, end, point);
                // An arc and its chord close a simple loop. Only a point inside that
                // loop sees the arc turn a full turn further than the chord.
                if between_chord_and_arc(point, center, radius, start, end, sweep) {
                    chord + TAU * sweep.signum()
                } else {
                    chord
                }
            }
            Self::Circle {
                center,
                radius,
                counterclockwise,
                ..
            } => {
                if (point - center).norm() >= radius {
                    0.
                } else if counterclockwise {
                    TAU
                } else {
                    -TAU
                }
            }
        }
    }
}

fn round_leaving(center: Point2, radius: f64, at: Point2, counterclockwise: bool) -> (f64, f64) {
    let outward = at - center;
    let (direction, curvature) = if counterclockwise {
        (Vector2::new(-outward.y, outward.x), 1. / radius)
    } else {
        (Vector2::new(outward.y, -outward.x), -1. / radius)
    };
    (direction.y.atan2(direction.x), curvature)
}

fn chord_area(start: Point2, end: Point2) -> f64 {
    (start.x * end.y - start.y * end.x) / 2.
}

fn chord_angle(start: Point2, end: Point2, point: Point2) -> f64 {
    let (from, to) = (start - point, end - point);
    cross(from, to).atan2(from.dot(&to))
}

fn between_chord_and_arc(
    point: Point2,
    center: Point2,
    radius: f64,
    start: Point2,
    end: Point2,
    sweep: f64,
) -> bool {
    if (point - center).norm() >= radius {
        return false;
    }
    let (sine, cosine) = (sweep / 2.).sin_cos();
    let outward = start - center;
    let middle = center
        + Vector2::new(
            outward.x * cosine - outward.y * sine,
            outward.x * sine + outward.y * cosine,
        );
    let chord = end - start;
    cross(chord, point - start) * cross(chord, middle - start) > 0.
}

fn cross(a: Vector2<f64>, b: Vector2<f64>) -> f64 {
    a.x * b.y - a.y * b.x
}
