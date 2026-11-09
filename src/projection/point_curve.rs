use super::ambiguity::has_several_nearest_points_on_curve;
use super::error::ProjectionError;
use super::options::ProjectOptions;
use super::project::Project;
use super::result::{PointProjection, TargetParameters};
use crate::geometry::{Curve, Point3};

impl Project<Point3> for Curve {
    type Mode = ();
    type Hit = ();
    type Output = PointProjection<()>;

    /// The nearest point on the whole support curve.
    ///
    /// A point with several equally near points on the curve is
    /// [`Ambiguous`](ProjectionError::Ambiguous). Analytic curves are checked
    /// in closed form; a NURBS curve is checked on the local nearest points
    /// its numerical search finds.
    fn project_with(
        &self,
        source: &Point3,
        _mode: (),
        options: &ProjectOptions,
    ) -> Result<PointProjection<()>, ProjectionError> {
        let t = self.parameter_at(*source);
        let point = self.point_at(t);
        if has_several_nearest_points_on_curve(self, *source, point, options.tolerance) {
            return Err(ProjectionError::Ambiguous);
        }
        Ok(PointProjection {
            point,
            parameters: Some(TargetParameters::Curve(t)),
            distance: (point - source).norm(),
            hit: (),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{
        Circle, ControlPolygon, Degree, Ellipse, Frame, HPoint, KnotVector, NurbsCurve, Plane,
    };
    use crate::projection::{project, project_point};

    #[test]
    fn a_point_projects_onto_a_line_perpendicularly() {
        let line = Curve::line(Point3::origin(), Point3::new(2.0, 0.0, 0.0));
        let result = project(&Point3::new(5.0, 3.0, 0.0), &line, ()).unwrap();
        assert!((result.point - Point3::new(5.0, 0.0, 0.0)).norm() < 1e-12);
        assert!((result.distance - 3.0).abs() < 1e-12);
        let Some(TargetParameters::Curve(t)) = result.parameters else {
            panic!("a curve's nearest point reports its parameter");
        };
        assert!((t.value() - 2.5).abs() < 1e-12);
    }

    #[test]
    fn a_point_projects_onto_a_circle_radially() {
        let circle = Curve::circle(Plane::xy(), 2.0);
        let result = project(&Point3::new(0.0, 5.0, 1.0), &circle, ()).unwrap();
        assert!((result.point - Point3::new(0.0, 2.0, 0.0)).norm() < 1e-12);
        assert!((result.distance - (9.0_f64 + 1.0).sqrt()).abs() < 1e-12);
    }

    #[test]
    fn a_point_on_a_circles_axis_is_ambiguous() {
        let circle = Curve::circle(Plane::xy(), 2.0);
        let result = project(&Point3::new(0.0, 0.0, 3.0), &circle, ());
        assert!(matches!(result, Err(ProjectionError::Ambiguous)));
    }

    #[test]
    fn a_point_on_a_nurbs_circles_axis_is_ambiguous_like_the_analytic_circle() {
        let circle = Circle::new(Plane::xy(), 2.0);
        let nurbs = Curve::Nurbs(circle.to_nurbs().unwrap());
        let on_axis = Point3::new(0.0, 0.0, 3.0);
        let result = project(&on_axis, &nurbs, ());
        assert!(matches!(result, Err(ProjectionError::Ambiguous)));
        let analytic = project(&on_axis, &Curve::Circle(circle), ());
        assert!(matches!(analytic, Err(ProjectionError::Ambiguous)));
    }

    #[test]
    fn a_point_off_a_nurbs_circles_axis_has_one_nearest_point() {
        let circle = Circle::new(Plane::xy(), 2.0);
        let nurbs = Curve::Nurbs(circle.to_nurbs().unwrap());
        let result = project(&Point3::new(0.0, 5.0, 1.0), &nurbs, ()).unwrap();
        assert!((result.point - Point3::new(0.0, 2.0, 0.0)).norm() < 1e-9);
    }

    #[test]
    fn a_point_equally_near_two_bends_of_a_nurbs_curve_is_ambiguous() {
        let bends = NurbsCurve::new(
            Degree::new(2).unwrap(),
            ControlPolygon::new(vec![
                HPoint::from_cartesian(Point3::new(-4.0, 0.0, 0.0), 1.0),
                HPoint::from_cartesian(Point3::new(-2.0, 6.0, 0.0), 1.0),
                HPoint::from_cartesian(Point3::new(0.0, 0.0, 0.0), 1.0),
                HPoint::from_cartesian(Point3::new(2.0, 6.0, 0.0), 1.0),
                HPoint::from_cartesian(Point3::new(4.0, 0.0, 0.0), 1.0),
            ])
            .unwrap(),
            KnotVector::new(vec![0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0]).unwrap(),
        )
        .unwrap();
        let curve = Curve::Nurbs(bends);
        let between = project(&Point3::new(0.0, 10.0, 0.0), &curve, ());
        assert!(matches!(between, Err(ProjectionError::Ambiguous)));
        let beside = project(&Point3::new(1.0, 10.0, 0.0), &curve, ()).unwrap();
        assert!(beside.point.x > 0.0);
    }

    #[test]
    fn a_point_on_an_ellipses_long_axis_inside_its_evolute_is_ambiguous() {
        let ellipse = Curve::Ellipse(Ellipse::new(Frame::xyz(), 4.0, 2.0));
        let inside = project(&Point3::new(1.0, 0.0, 0.0), &ellipse, ());
        assert!(matches!(inside, Err(ProjectionError::Ambiguous)));
        let outside = project(&Point3::new(3.5, 0.0, 0.0), &ellipse, ()).unwrap();
        assert!((outside.point - Point3::new(4.0, 0.0, 0.0)).norm() < 1e-9);
    }

    #[test]
    fn the_project_helper_returns_only_the_point() {
        let line = Curve::line(Point3::origin(), Point3::new(0.0, 0.0, 1.0));
        let point = project_point(&Point3::new(1.0, 0.0, 7.0), &line, ()).unwrap();
        assert!((point - Point3::new(0.0, 0.0, 7.0)).norm() < 1e-12);
    }
}
