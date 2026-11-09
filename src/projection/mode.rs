use nalgebra::UnitVector3;

use crate::geometry::IntoUnit;

/// How a point is carried onto a two-dimensional target.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Projection {
    /// Along the perpendicular to the target: the nearest point.
    Normal,
    /// Along the half-line from the point in the given direction, to its first
    /// hit.
    Along(UnitVector3<f64>),
}

impl Projection {
    /// A directional projection along `direction`, normalized.
    ///
    /// # Panics
    ///
    /// When `direction` is the zero vector.
    pub fn along(direction: impl IntoUnit<3>) -> Self {
        Projection::Along(direction.normalized())
    }
}
