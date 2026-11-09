use super::error::ProjectionError;
use super::mode::Projection;
use super::options::ProjectOptions;
use super::project::Project;
use super::result::PointProjection;
use crate::geometry::{ANGULAR_TOLERANCE, Plane, Point3};

impl Project<Plane> for Point3 {
    type Mode = Projection;
    type Hit = ();
    type Output = PointProjection<()>;

    /// A plane has a closed form in both modes. Its `(u, v)` follow from the
    /// projected point, so they are not returned.
    ///
    /// A directional projection parallel to the plane is a
    /// [`Miss`](ProjectionError::Miss), or a
    /// [`RayInSurface`](ProjectionError::RayInSurface) when the point lies in
    /// the plane.
    fn project(
        &self,
        plane: &Plane,
        mode: Projection,
        options: &ProjectOptions,
    ) -> Result<PointProjection<()>, ProjectionError> {
        let normal = plane.normal();
        let offset = (self - plane.origin()).dot(&normal);
        let (point, distance) = match mode {
            Projection::Normal => (self - *normal * offset, offset.abs()),
            Projection::Along(direction) => {
                let slope = direction.dot(&normal);
                if slope.abs() <= ANGULAR_TOLERANCE {
                    return Err(if offset.abs() <= options.tolerance {
                        ProjectionError::RayInSurface
                    } else {
                        ProjectionError::Miss
                    });
                }
                let t = -offset / slope;
                if t < -options.tolerance {
                    return Err(ProjectionError::Miss);
                }
                let t = t.max(0.0);
                (self + *direction * t, t)
            }
        };
        Ok(PointProjection {
            point,
            parameters: None,
            distance,
            hit: (),
        })
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;
    use crate::projection::{project, project_point};

    fn tilted_plane() -> Plane {
        Plane::new(
            Point3::new(0.0, 0.0, 1.0),
            Vector3::x(),
            Vector3::new(0.0, -1.0, 1.0),
        )
    }

    #[test]
    fn normal_projection_drops_the_perpendicular() {
        let options = ProjectOptions::default();
        let result = project(
            &Point3::new(3.0, 4.0, 5.0),
            &Plane::xy(),
            Projection::Normal,
            &options,
        )
        .unwrap();
        assert!((result.point - Point3::new(3.0, 4.0, 0.0)).norm() < 1e-12);
        assert!((result.distance - 5.0).abs() < 1e-12);
        assert_eq!(result.parameters, None);
    }

    #[test]
    fn normal_projection_onto_a_tilted_plane_lands_on_the_plane() {
        let plane = tilted_plane();
        let result = project(
            &Point3::new(1.0, 2.0, 3.0),
            &plane,
            Projection::Normal,
            &ProjectOptions::default(),
        )
        .unwrap();
        assert!(((result.point - plane.origin()).dot(&plane.normal())).abs() < 1e-12);
        assert!(
            ((result.point - Point3::new(1.0, 2.0, 3.0)).norm() - result.distance).abs() < 1e-12
        );
    }

    #[test]
    fn directional_projection_follows_the_direction() {
        let result = project(
            &Point3::new(1.0, 2.0, 4.0),
            &Plane::xy(),
            Projection::along(Vector3::new(1.0, 0.0, -1.0)),
            &ProjectOptions::default(),
        )
        .unwrap();
        assert!((result.point - Point3::new(5.0, 2.0, 0.0)).norm() < 1e-12);
        assert!((result.distance - 32.0_f64.sqrt()).abs() < 1e-12);
    }

    #[test]
    fn directional_projection_away_from_the_plane_is_a_miss() {
        let result = project(
            &Point3::new(0.0, 0.0, 4.0),
            &Plane::xy(),
            Projection::along(Vector3::z()),
            &ProjectOptions::default(),
        );
        assert!(matches!(result, Err(ProjectionError::Miss)));
    }

    #[test]
    fn directional_projection_parallel_to_the_plane_is_a_miss() {
        let result = project(
            &Point3::new(0.0, 0.0, 4.0),
            &Plane::xy(),
            Projection::along(Vector3::x()),
            &ProjectOptions::default(),
        );
        assert!(matches!(result, Err(ProjectionError::Miss)));
    }

    #[test]
    fn directional_projection_inside_the_plane_lies_in_the_surface() {
        let result = project(
            &Point3::new(1.0, 1.0, 0.0),
            &Plane::xy(),
            Projection::along(Vector3::x()),
            &ProjectOptions::default(),
        );
        assert!(matches!(result, Err(ProjectionError::RayInSurface)));
    }

    #[test]
    fn a_point_in_the_plane_projects_to_itself() {
        let point = Point3::new(1.0, 1.0, 0.0);
        let found = project_point(
            &point,
            &Plane::xy(),
            Projection::along(Vector3::z()),
            &ProjectOptions::default(),
        );
        assert!((found.unwrap() - point).norm() < 1e-12);
    }
}
