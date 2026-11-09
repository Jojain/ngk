use super::error::ProjectionError;
use super::options::ProjectOptions;
use super::result::PointProjection;
use crate::geometry::Point3;

/// A target that receives what is projected onto it.
///
/// Implemented once per (target, source) pair: `plane.project(&point, mode)`,
/// `curve.project(&point, ())`, `face_list.project(&profile, mode)`. The free
/// functions [`project`] and [`project_with`] take the source first.
pub trait Project<Source: ?Sized> {
    /// How the source is carried onto the target:
    /// [`Projection`](super::Projection) for a two-dimensional target, `()`
    /// for a curve, which only has a perpendicular.
    type Mode;
    /// What the target reports as touched; `()` for pure geometry.
    type Hit;
    /// The result of a successful projection.
    type Output;

    /// Projects `source` onto `self` with the given options.
    fn project_with(
        &self,
        source: &Source,
        mode: Self::Mode,
        options: &ProjectOptions,
    ) -> Result<Self::Output, ProjectionError>;

    /// Projects `source` onto `self` with the default options.
    fn project(&self, source: &Source, mode: Self::Mode) -> Result<Self::Output, ProjectionError> {
        self.project_with(source, mode, &ProjectOptions::default())
    }
}

/// Projects `source` onto `target` in the given mode, with the default options.
pub fn project<S, T>(
    source: &S,
    target: &T,
    mode: <T as Project<S>>::Mode,
) -> Result<<T as Project<S>>::Output, ProjectionError>
where
    S: ?Sized,
    T: Project<S> + ?Sized,
{
    target.project(source, mode)
}

/// Projects `source` onto `target` in the given mode.
pub fn project_with<S, T>(
    source: &S,
    target: &T,
    mode: <T as Project<S>>::Mode,
    options: &ProjectOptions,
) -> Result<<T as Project<S>>::Output, ProjectionError>
where
    S: ?Sized,
    T: Project<S> + ?Sized,
{
    target.project_with(source, mode, options)
}

/// Projects a point onto `target` and keeps only the projected point, with the
/// default options.
pub fn project_point<T>(
    point: &Point3,
    target: &T,
    mode: <T as Project<Point3>>::Mode,
) -> Result<Point3, ProjectionError>
where
    T: Project<Point3, Output = PointProjection<<T as Project<Point3>>::Hit>> + ?Sized,
{
    project_point_with(point, target, mode, &ProjectOptions::default())
}

/// Projects a point onto `target` and keeps only the projected point.
pub fn project_point_with<T>(
    point: &Point3,
    target: &T,
    mode: <T as Project<Point3>>::Mode,
    options: &ProjectOptions,
) -> Result<Point3, ProjectionError>
where
    T: Project<Point3, Output = PointProjection<<T as Project<Point3>>::Hit>> + ?Sized,
{
    project_with(point, target, mode, options).map(PointProjection::into_point)
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;
    use crate::builders::faces::add_rectangle;
    use crate::geometry::{Curve, Frame, Plane};
    use crate::model::Model;
    use crate::projection::{CurveProjection, FaceList, Projection, ProjectionError};
    use crate::topology::StandardPayload;

    #[test]
    fn a_plane_receives_a_point() {
        let result = Plane::xy()
            .project(&Point3::new(3.0, 4.0, 5.0), Projection::Normal)
            .unwrap();
        assert!((result.point - Point3::new(3.0, 4.0, 0.0)).norm() < 1e-12);
        assert!((result.distance - 5.0).abs() < 1e-12);
    }

    #[test]
    fn a_curve_receives_a_point_with_a_unit_mode() {
        let line = Curve::line(Point3::origin(), Point3::new(2.0, 0.0, 0.0));
        let result = line.project(&Point3::new(1.0, 3.0, 0.0), ()).unwrap();
        assert!((result.point - Point3::new(1.0, 0.0, 0.0)).norm() < 1e-12);
    }

    #[test]
    fn a_plane_receives_a_curve() {
        let circle = Curve::circle(Plane::xy(), 2.0);
        let plane = Plane::new(
            Point3::new(0.0, 0.0, 1.0),
            Vector3::x(),
            Vector3::new(0.0, -1.0, 1.0),
        );
        let projection: CurveProjection<()> = plane.project(&circle, Projection::Normal).unwrap();
        assert_eq!(projection.chains.len(), 1);
    }

    #[test]
    fn a_face_list_receives_a_curve() {
        let mut model = Model::<StandardPayload>::new();
        let floor = add_rectangle(&mut model, Plane::xy(), 4.0, 4.0).unwrap();
        let faces = [floor];
        let list = FaceList::new(&model, &faces);
        let ring = Curve::circle(
            Plane::from_frame(Frame::at(Point3::new(2.0, 2.0, 3.0))),
            1.0,
        );
        let projection = list
            .project(&ring, Projection::along(-Vector3::z()))
            .unwrap();
        assert_eq!(projection.chains.len(), 1);
        assert_eq!(projection.chains[0].segments[0].hit, floor);
    }

    #[test]
    fn project_uses_the_default_tolerance_and_project_with_the_given_one() {
        let plane = Plane::xy();
        let above = Point3::new(0.0, 0.0, 1.0e-3);
        let along = || Projection::along(Vector3::z());
        let default = plane.project(&above, along());
        assert!(matches!(default, Err(ProjectionError::Miss)));
        let loose = ProjectOptions { tolerance: 1.0e-2 };
        let found = plane.project_with(&above, along(), &loose).unwrap();
        assert!(found.distance.abs() < 1e-12);
    }

    #[test]
    fn the_free_functions_are_the_source_first_form_of_the_methods() {
        let plane = Plane::xy();
        let point = Point3::new(1.0, 2.0, 3.0);
        let method = plane.project(&point, Projection::Normal).unwrap();
        let free = project(&point, &plane, Projection::Normal).unwrap();
        assert_eq!(method, free);
        let loose = ProjectOptions { tolerance: 1.0e-2 };
        let with = project_with(&point, &plane, Projection::Normal, &loose).unwrap();
        assert_eq!(method, with);
        let kept = project_point_with(&point, &plane, Projection::Normal, &loose).unwrap();
        assert_eq!(kept, method.point);
    }
}
