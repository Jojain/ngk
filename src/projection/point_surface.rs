use nalgebra::UnitVector3;

use super::ambiguity::has_several_nearest_points_on_surface;
use super::error::ProjectionError;
use super::mode::Projection;
use super::options::ProjectOptions;
use super::project::Project;
use super::result::{PointProjection, TargetParameters};
use crate::geometry::{
    Axis3, Curve, CurveSurfaceIntersection, IntersectionOptions, Line, Point2, Point3, Surface,
    intersect_analytic_curve_surface, line_surface_is_analytic,
};

/// The sine of the angle between a ray and a surface below which the ray
/// grazes it. A tangent contact is located only to about the square root of
/// the working precision, so the normal there is perpendicular to the ray no
/// better than that.
const GRAZING_SINE: f64 = 1.0e-6;

impl Project<Surface> for Point3 {
    type Mode = Projection;
    type Hit = ();
    type Output = PointProjection<()>;

    /// Projects onto the whole support surface.
    ///
    /// [`Normal`](Projection::Normal) takes the surface's nearest point; a
    /// point with several equally near points is
    /// [`Ambiguous`](ProjectionError::Ambiguous). Analytic surfaces are
    /// checked in closed form; a ruled or NURBS surface is checked on the
    /// local nearest points its numerical search finds.
    ///
    /// [`Along`](Projection::Along) takes the first hit of the half-line from
    /// the point, for the surfaces the kernel intersects with a line in closed
    /// form (plane, sphere, cylinder, cone) and refuses the others. A surface
    /// only behind the point is a [`Miss`](ProjectionError::Miss).
    fn project(
        &self,
        surface: &Surface,
        mode: Projection,
        options: &ProjectOptions,
    ) -> Result<PointProjection<()>, ProjectionError> {
        match mode {
            Projection::Normal => project_normal(self, surface, options),
            Projection::Along(direction) => project_along(self, surface, direction, options),
        }
    }
}

pub(super) fn project_normal(
    source: &Point3,
    surface: &Surface,
    options: &ProjectOptions,
) -> Result<PointProjection<()>, ProjectionError> {
    let uv = surface.param_at(*source)?;
    let point = surface.point_at(uv.x, uv.y);
    if has_several_nearest_points_on_surface(surface, *source, point, options.tolerance) {
        return Err(ProjectionError::Ambiguous);
    }
    Ok(PointProjection {
        point,
        parameters: Some(TargetParameters::Surface(uv)),
        distance: (point - source).norm(),
        hit: (),
    })
}

fn project_along(
    source: &Point3,
    surface: &Surface,
    direction: UnitVector3<f64>,
    options: &ProjectOptions,
) -> Result<PointProjection<()>, ProjectionError> {
    let hits = ray_hits(source, surface, direction, options)?;
    let first = hits.first().ok_or(ProjectionError::Miss)?;
    reject_grazing(surface, first, direction)?;
    Ok(first.into_projection(()))
}

/// A hit of a half-line on a surface.
#[derive(Debug, Clone, Copy)]
pub(super) struct RayHit {
    /// The distance from the source along the ray, never negative.
    pub(super) distance: f64,
    pub(super) point: Point3,
    pub(super) uv: Point2,
}

impl RayHit {
    pub(super) fn into_projection<Hit>(self, hit: Hit) -> PointProjection<Hit> {
        PointProjection {
            point: self.point,
            parameters: Some(TargetParameters::Surface(self.uv)),
            distance: self.distance,
            hit,
        }
    }
}

/// Every hit of the half-line from `source` along `direction` on `surface`,
/// nearest first.
///
/// Only the surfaces the kernel intersects with a line in closed form are
/// searched; the others are refused. A ray lying in the surface is refused too.
pub(super) fn ray_hits(
    source: &Point3,
    surface: &Surface,
    direction: UnitVector3<f64>,
    options: &ProjectOptions,
) -> Result<Vec<RayHit>, ProjectionError> {
    if !line_surface_is_analytic(surface) {
        return Err(ProjectionError::UnsupportedSurface {
            surface: surface_kind(surface),
        });
    }
    // The line's parameter is arc length from the source, so a hit's parameter
    // is its distance along the ray.
    let ray = Curve::Line(Line::new(Axis3::new(*source, direction)));
    let intersection_options = IntersectionOptions {
        linear_tolerance: options.tolerance,
        ..IntersectionOptions::default()
    };
    // The closed-form table declines a line only when it lies in the surface
    // over its whole length.
    let intersections = intersect_analytic_curve_surface(&ray, surface, intersection_options)
        .ok_or(ProjectionError::RayInSurface)??;
    let mut hits = Vec::new();
    for intersection in intersections {
        match intersection {
            CurveSurfaceIntersection::Overlap { .. } => return Err(ProjectionError::RayInSurface),
            CurveSurfaceIntersection::Point {
                point,
                curve_u,
                surface_u,
                surface_v,
            } if curve_u >= -options.tolerance => hits.push(RayHit {
                distance: curve_u.max(0.0),
                point,
                uv: Point2::new(surface_u, surface_v),
            }),
            CurveSurfaceIntersection::Point { .. } => {}
        }
    }
    hits.sort_by(|a, b| a.distance.total_cmp(&b.distance));
    Ok(hits)
}

/// Refuses a hit where the ray touches the surface without crossing it.
pub(super) fn reject_grazing(
    surface: &Surface,
    hit: &RayHit,
    direction: UnitVector3<f64>,
) -> Result<(), ProjectionError> {
    let normal = surface.normal_at(hit.uv.x, hit.uv.y);
    if normal.dot(&direction).abs() <= GRAZING_SINE {
        return Err(ProjectionError::GrazingRay);
    }
    Ok(())
}

pub(super) fn surface_kind(surface: &Surface) -> &'static str {
    match surface {
        Surface::Plane(_) => "plane",
        Surface::Cylinder(_) => "cylinder",
        Surface::Sphere(_) => "sphere",
        Surface::Cone(_) => "cone",
        Surface::Torus(_) => "torus",
        Surface::Ruled(_) => "ruled surface",
        Surface::Revolution(_) => "surface of revolution",
        Surface::Nurbs(_) => "NURBS surface",
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;
    use crate::geometry::{Cone, Cylinder, Frame, Plane, RuledSurface, Sphere};
    use crate::projection::project;

    fn sphere() -> Surface {
        Surface::Sphere(Sphere::new(Frame::xyz(), 2.0))
    }

    fn cylinder() -> Surface {
        Surface::Cylinder(Cylinder::new(
            Point3::origin(),
            Vector3::x(),
            Vector3::z(),
            2.0,
        ))
    }

    fn options() -> ProjectOptions {
        ProjectOptions::default()
    }

    #[test]
    fn normal_projection_onto_a_sphere_lands_on_the_radial_point() {
        let result = project(
            &Point3::new(5.0, 0.0, 0.0),
            &sphere(),
            Projection::Normal,
            &options(),
        )
        .unwrap();
        assert!((result.point - Point3::new(2.0, 0.0, 0.0)).norm() < 1e-9);
        assert!((result.distance - 3.0).abs() < 1e-9);
        assert!(matches!(
            result.parameters,
            Some(TargetParameters::Surface(_))
        ));
    }

    #[test]
    fn normal_projection_from_inside_a_sphere_lands_on_the_same_radial_point() {
        let result = project(
            &Point3::new(0.0, 0.5, 0.0),
            &sphere(),
            Projection::Normal,
            &options(),
        )
        .unwrap();
        assert!((result.point - Point3::new(0.0, 2.0, 0.0)).norm() < 1e-9);
        assert!((result.distance - 1.5).abs() < 1e-9);
    }

    #[test]
    fn normal_projection_from_the_centre_of_a_sphere_is_ambiguous() {
        let result = project(&Point3::origin(), &sphere(), Projection::Normal, &options());
        assert!(matches!(result, Err(ProjectionError::Ambiguous)));
    }

    #[test]
    fn normal_projection_onto_a_cylinder_lands_on_the_radial_point() {
        let result = project(
            &Point3::new(0.0, 5.0, 3.0),
            &cylinder(),
            Projection::Normal,
            &options(),
        )
        .unwrap();
        assert!((result.point - Point3::new(0.0, 2.0, 3.0)).norm() < 1e-9);
        assert!((result.distance - 3.0).abs() < 1e-9);
    }

    #[test]
    fn normal_projection_from_a_cylinders_axis_is_ambiguous() {
        let result = project(
            &Point3::new(0.0, 0.0, 1.0),
            &cylinder(),
            Projection::Normal,
            &options(),
        );
        assert!(matches!(result, Err(ProjectionError::Ambiguous)));
    }

    #[test]
    fn normal_projection_from_the_axis_of_a_nurbs_cylinder_is_ambiguous() {
        let Surface::Cylinder(cylinder) = cylinder() else {
            unreachable!()
        };
        let nurbs = Surface::Nurbs(cylinder.to_nurbs().unwrap());
        let on_axis = Point3::new(0.0, 0.0, 1.0);
        let result = project(&on_axis, &nurbs, Projection::Normal, &options());
        assert!(matches!(result, Err(ProjectionError::Ambiguous)));
    }

    #[test]
    fn normal_projection_from_the_centre_of_a_nurbs_sphere_is_ambiguous() {
        let Surface::Sphere(sphere) = sphere() else {
            unreachable!()
        };
        let nurbs = Surface::Nurbs(sphere.to_nurbs().unwrap());
        let result = project(&Point3::origin(), &nurbs, Projection::Normal, &options());
        assert!(matches!(result, Err(ProjectionError::Ambiguous)));
    }

    #[test]
    fn normal_projection_off_the_axis_of_a_nurbs_cylinder_keeps_its_one_nearest_point() {
        let Surface::Cylinder(cylinder) = cylinder() else {
            unreachable!()
        };
        let nurbs = Surface::Nurbs(cylinder.to_nurbs().unwrap());
        for source in [Point3::new(0.0, 5.0, 1.0), Point3::new(0.001, 0.0, 1.0)] {
            let result = project(&source, &nurbs, Projection::Normal, &options()).unwrap();
            let uv = nurbs.param_at(source).unwrap();
            assert!((result.point - nurbs.point_at(uv.x, uv.y)).norm() < 1e-12);
        }
    }

    #[test]
    fn normal_projection_off_the_centre_of_a_nurbs_sphere_keeps_its_one_nearest_point() {
        let Surface::Sphere(sphere) = sphere() else {
            unreachable!()
        };
        let nurbs = Surface::Nurbs(sphere.to_nurbs().unwrap());
        for source in [Point3::new(0.0, 0.5, 0.0), Point3::new(0.0, 0.0, 0.01)] {
            let result = project(&source, &nurbs, Projection::Normal, &options()).unwrap();
            let uv = nurbs.param_at(source).unwrap();
            assert!((result.point - nurbs.point_at(uv.x, uv.y)).norm() < 1e-12);
        }
    }

    #[test]
    fn normal_projection_from_the_axis_of_a_ruled_cylinder_is_ambiguous() {
        let ruled = Surface::Ruled(RuledSurface::new(
            Curve::circle(Plane::xy(), 2.0),
            Vector3::z() * 4.0,
        ));
        let on_axis = project(
            &Point3::new(0.0, 0.0, 1.0),
            &ruled,
            Projection::Normal,
            &options(),
        );
        assert!(matches!(on_axis, Err(ProjectionError::Ambiguous)));
        let off_axis = project(
            &Point3::new(0.0, 5.0, 1.0),
            &ruled,
            Projection::Normal,
            &options(),
        )
        .unwrap();
        assert!((off_axis.point - Point3::new(0.0, 2.0, 1.0)).norm() < 1e-9);
    }

    #[test]
    fn normal_projection_from_a_cones_axis_is_ambiguous_except_at_its_apex() {
        let cone = Surface::Cone(Cone::new(Frame::xyz(), 0.0, 0.4));
        let on_axis = project(
            &Point3::new(0.0, 0.0, 2.0),
            &cone,
            Projection::Normal,
            &options(),
        );
        assert!(matches!(on_axis, Err(ProjectionError::Ambiguous)));
        let apex = project(&Point3::origin(), &cone, Projection::Normal, &options()).unwrap();
        assert!(apex.distance < 1e-9);
    }

    #[test]
    fn along_hits_the_first_wall_of_a_sphere() {
        let result = project(
            &Point3::new(-5.0, 0.0, 0.0),
            &sphere(),
            Projection::along(Vector3::x()),
            &options(),
        )
        .unwrap();
        assert!((result.point - Point3::new(-2.0, 0.0, 0.0)).norm() < 1e-9);
        assert!((result.distance - 3.0).abs() < 1e-9);
    }

    #[test]
    fn along_from_inside_a_sphere_hits_the_wall_ahead() {
        let result = project(
            &Point3::new(1.0, 0.0, 0.0),
            &sphere(),
            Projection::along(Vector3::x()),
            &options(),
        )
        .unwrap();
        assert!((result.point - Point3::new(2.0, 0.0, 0.0)).norm() < 1e-9);
        assert!((result.distance - 1.0).abs() < 1e-9);
    }

    #[test]
    fn along_a_ray_that_misses_is_a_miss() {
        let result = project(
            &Point3::new(-5.0, 3.0, 0.0),
            &sphere(),
            Projection::along(Vector3::x()),
            &options(),
        );
        assert!(matches!(result, Err(ProjectionError::Miss)));
    }

    #[test]
    fn a_surface_behind_the_point_casts_no_shadow() {
        let result = project(
            &Point3::new(5.0, 0.0, 0.0),
            &sphere(),
            Projection::along(Vector3::x()),
            &options(),
        );
        assert!(matches!(result, Err(ProjectionError::Miss)));
    }

    #[test]
    fn a_ray_tangent_to_a_sphere_grazes() {
        let result = project(
            &Point3::new(-5.0, 2.0, 0.0),
            &sphere(),
            Projection::along(Vector3::x()),
            &options(),
        );
        assert!(matches!(result, Err(ProjectionError::GrazingRay)));
    }

    #[test]
    fn a_ray_tangent_to_a_cylinder_grazes() {
        let result = project(
            &Point3::new(-5.0, 2.0, 1.0),
            &cylinder(),
            Projection::along(Vector3::x()),
            &options(),
        );
        assert!(matches!(result, Err(ProjectionError::GrazingRay)));
    }

    #[test]
    fn a_ray_along_a_cylinders_ruling_lies_in_the_surface() {
        let result = project(
            &Point3::new(2.0, 0.0, -1.0),
            &cylinder(),
            Projection::along(Vector3::z()),
            &options(),
        );
        assert!(matches!(result, Err(ProjectionError::RayInSurface)));
    }

    #[test]
    fn along_onto_a_cylinder_reports_its_parameters() {
        let result = project(
            &Point3::new(-5.0, 0.0, 1.5),
            &cylinder(),
            Projection::along(Vector3::x()),
            &options(),
        )
        .unwrap();
        assert!((result.point - Point3::new(-2.0, 0.0, 1.5)).norm() < 1e-9);
        let Some(TargetParameters::Surface(uv)) = result.parameters else {
            panic!("a surface hit reports its parameters");
        };
        assert!((uv.y - 1.5).abs() < 1e-9);
    }

    #[test]
    fn along_onto_a_plane_variant_of_the_surface_enum_hits_it() {
        let surface = Surface::Plane(Plane::xy());
        let result = project(
            &Point3::new(1.0, 2.0, 4.0),
            &surface,
            Projection::along(-Vector3::z()),
            &options(),
        )
        .unwrap();
        assert!((result.point - Point3::new(1.0, 2.0, 0.0)).norm() < 1e-9);
        assert!((result.distance - 4.0).abs() < 1e-9);
    }

    #[test]
    fn along_onto_a_torus_is_refused() {
        let torus = Surface::Torus(crate::geometry::Torus::new(Frame::xyz(), 3.0, 1.0));
        let result = project(
            &Point3::new(-9.0, 0.0, 0.0),
            &torus,
            Projection::along(Vector3::x()),
            &options(),
        );
        assert!(matches!(
            result,
            Err(ProjectionError::UnsupportedSurface { .. })
        ));
    }
}
