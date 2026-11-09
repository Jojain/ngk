use super::ambiguity::curve_feet_tie;
use super::error::ProjectionError;
use super::options::ProjectOptions;
use super::project::Project;
use super::result::{PointProjection, TargetParameters};
use crate::geometry::{Curve, Point3};

impl Project<Curve> for Point3 {
    type Mode = ();
    type Hit = ();
    type Output = PointProjection<()>;

    /// The foot of the perpendicular on the whole support curve.
    ///
    /// Analytic curves are searched in closed form and report a tie as
    /// [`Ambiguous`](ProjectionError::Ambiguous). A NURBS curve is searched
    /// numerically and is not checked for ties.
    fn project(
        &self,
        curve: &Curve,
        _mode: (),
        options: &ProjectOptions,
    ) -> Result<PointProjection<()>, ProjectionError> {
        if curve_feet_tie(curve, *self, options.tolerance) {
            return Err(ProjectionError::Ambiguous);
        }
        let t = curve.parameter_at(*self);
        let point = curve.point_at(t);
        Ok(PointProjection {
            point,
            parameters: Some(TargetParameters::Curve(t)),
            distance: (point - self).norm(),
            hit: (),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Ellipse, Frame, Plane};
    use crate::projection::{project, project_point};

    #[test]
    fn a_point_projects_onto_a_line_perpendicularly() {
        let line = Curve::line(Point3::origin(), Point3::new(2.0, 0.0, 0.0));
        let result = project(
            &Point3::new(5.0, 3.0, 0.0),
            &line,
            (),
            &ProjectOptions::default(),
        )
        .unwrap();
        assert!((result.point - Point3::new(5.0, 0.0, 0.0)).norm() < 1e-12);
        assert!((result.distance - 3.0).abs() < 1e-12);
        let Some(TargetParameters::Curve(t)) = result.parameters else {
            panic!("a curve foot reports its parameter");
        };
        assert!((t.value() - 2.5).abs() < 1e-12);
    }

    #[test]
    fn a_point_projects_onto_a_circle_radially() {
        let circle = Curve::circle(Plane::xy(), 2.0);
        let result = project(
            &Point3::new(0.0, 5.0, 1.0),
            &circle,
            (),
            &ProjectOptions::default(),
        )
        .unwrap();
        assert!((result.point - Point3::new(0.0, 2.0, 0.0)).norm() < 1e-12);
        assert!((result.distance - (9.0_f64 + 1.0).sqrt()).abs() < 1e-12);
    }

    #[test]
    fn a_point_on_a_circles_axis_is_ambiguous() {
        let circle = Curve::circle(Plane::xy(), 2.0);
        let result = project(
            &Point3::new(0.0, 0.0, 3.0),
            &circle,
            (),
            &ProjectOptions::default(),
        );
        assert!(matches!(result, Err(ProjectionError::Ambiguous)));
    }

    #[test]
    fn a_point_on_an_ellipses_long_axis_inside_its_evolute_is_ambiguous() {
        let ellipse = Curve::Ellipse(Ellipse::new(Frame::xyz(), 4.0, 2.0));
        let options = ProjectOptions::default();
        let inside = project(&Point3::new(1.0, 0.0, 0.0), &ellipse, (), &options);
        assert!(matches!(inside, Err(ProjectionError::Ambiguous)));
        let outside = project(&Point3::new(3.5, 0.0, 0.0), &ellipse, (), &options).unwrap();
        assert!((outside.point - Point3::new(4.0, 0.0, 0.0)).norm() < 1e-9);
    }

    #[test]
    fn the_project_helper_returns_only_the_point() {
        let line = Curve::line(Point3::origin(), Point3::new(0.0, 0.0, 1.0));
        let point = project_point(
            &Point3::new(1.0, 0.0, 7.0),
            &line,
            (),
            &ProjectOptions::default(),
        )
        .unwrap();
        assert!((point - Point3::new(0.0, 0.0, 7.0)).norm() < 1e-12);
    }
}
