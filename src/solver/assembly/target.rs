use super::BodyId;
use crate::geometry::dim3::utils::Point3;

/// A temporary objective of one `solve_toward` call: bring a point of a body toward a
/// world position, as far as the joints let it go.
#[derive(Clone, Debug)]
pub struct Target3d {
    pub(super) body: BodyId,
    pub(super) point: Point3,
    pub(super) position: Point3,
    pub(super) weight: f64,
}

impl Target3d {
    /// Pulls `point`, in the body's own coordinates, toward the world `position`.
    pub fn point(body: BodyId, point: Point3, position: Point3) -> Self {
        Self {
            body,
            point,
            position,
            weight: 1.,
        }
    }
    /// Sets a positive relative importance among targets.
    pub fn with_weight(mut self, weight: f64) -> Self {
        self.weight = weight;
        self
    }
    pub(super) fn valid(&self) -> bool {
        self.point.coords.iter().all(|v| v.is_finite())
            && self.position.coords.iter().all(|v| v.is_finite())
            && self.weight.is_finite()
            && self.weight > 0.
    }
}
