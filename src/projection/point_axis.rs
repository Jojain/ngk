use super::error::ProjectionError;
use super::options::ProjectOptions;
use super::project::Project;
use super::result::{PointProjection, TargetParameters};
use crate::geometry::{Axis3, NativeParam, Point3};

impl Project<Point3> for Axis3 {
    type Mode = ();
    type Hit = ();
    type Output = PointProjection<()>;

    /// The foot of the perpendicular from the point to the axis's line.
    ///
    /// An axis is a line, so a point has one nearest point on it and the
    /// projection never refuses. The reported parameter is the signed distance
    /// of the foot from the origin along the direction.
    fn project_with(
        &self,
        source: &Point3,
        _mode: (),
        _options: &ProjectOptions,
    ) -> Result<PointProjection<()>, ProjectionError> {
        let point = self.closest_point(*source);
        Ok(PointProjection {
            point,
            parameters: Some(TargetParameters::Curve(NativeParam::new(
                (point - self.origin).dot(&self.direction),
            ))),
            distance: (point - source).norm(),
            hit: (),
        })
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;
    use crate::projection::{project, project_point};

    #[test]
    fn a_point_projects_onto_an_axis_perpendicularly() {
        let axis = Axis3::new(Point3::new(1.0, 0.0, 0.0), Vector3::y());
        let result = axis.project(&Point3::new(4.0, 3.0, 2.0), ()).unwrap();
        assert!((result.point - Point3::new(1.0, 3.0, 0.0)).norm() < 1e-12);
        assert!((result.distance - 13.0_f64.sqrt()).abs() < 1e-12);
    }

    #[test]
    fn the_parameter_is_the_signed_distance_along_the_direction() {
        let axis = Axis3::new(Point3::new(0.0, 0.0, 1.0), Vector3::z());
        for (z, expected) in [(5.0, 4.0), (-2.0, -3.0)] {
            let result = axis.project(&Point3::new(1.0, 1.0, z), ()).unwrap();
            let Some(TargetParameters::Curve(t)) = result.parameters else {
                panic!("an axis's nearest point reports its parameter");
            };
            assert!((t.value() - expected).abs() < 1e-12);
        }
    }

    #[test]
    fn a_point_on_the_axis_projects_to_itself() {
        let point = Point3::new(0.0, 0.0, 7.0);
        let result = project(&point, &Axis3::z(), ()).unwrap();
        assert!((result.point - point).norm() < 1e-12);
        assert!(result.distance < 1e-12);
    }

    #[test]
    fn an_axis_through_the_origin_in_a_slanted_direction_has_one_foot_for_every_point() {
        let axis = Axis3::new(Point3::origin(), Vector3::new(1.0, 1.0, 0.0));
        let point = project_point(&Point3::new(2.0, 0.0, 5.0), &axis, ()).unwrap();
        assert!((point - Point3::new(1.0, 1.0, 0.0)).norm() < 1e-12);
    }
}
