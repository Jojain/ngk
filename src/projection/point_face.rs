use nalgebra::UnitVector3;

use super::error::ProjectionError;
use super::mode::Projection;
use super::options::ProjectOptions;
use super::point_surface::{project_normal, ray_hits, reject_grazing};
use super::project::Project;
use super::result::{PointProjection, TargetParameters};
use crate::geometry::{Point2, Point3};
use crate::topology::face::Face;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::FaceKey;
use crate::topology::trim_domain::{FaceTrimDomain, TrimLocation};

impl<P: Payload> Project<Point3> for Face<'_, P> {
    type Mode = Projection;
    type Hit = FaceKey;
    type Output = PointProjection<FaceKey>;

    /// Projects onto the face: onto its support, where the support is inside
    /// the face's trim.
    ///
    /// [`Normal`](Projection::Normal) takes the nearest point on the support and
    /// misses when it is outside the trim. [`Along`](Projection::Along) takes
    /// the first hit of the half-line on the support that lies inside the trim,
    /// so a hit in a hole or past the boundary lets the ray go on to the next
    /// one. A point on the trim's boundary is inside it. The refusals of the
    /// support (see the projection onto a surface) apply to the hits that are
    /// kept.
    fn project_with(
        &self,
        source: &Point3,
        mode: Projection,
        options: &ProjectOptions,
    ) -> Result<PointProjection<FaceKey>, ProjectionError> {
        let trim = FaceTrimDomain::new(self, options.tolerance)?;
        match mode {
            Projection::Normal => nearest_point_in_face(source, self, &trim, options),
            Projection::Along(direction) => wall_of_face(source, self, &trim, direction, options),
        }
    }
}

/// Whether `uv` is on the face, boundary included.
pub(super) fn on_face(trim: &FaceTrimDomain, uv: Point2) -> bool {
    !matches!(trim.classify(uv), TrimLocation::Outside { .. })
}

/// The nearest point on the face's support, if inside the trim.
pub(super) fn nearest_point_in_face<P: Payload>(
    source: &Point3,
    face: &Face<'_, P>,
    trim: &FaceTrimDomain,
    options: &ProjectOptions,
) -> Result<PointProjection<FaceKey>, ProjectionError> {
    let nearest = project_normal(source, face.surface(), options)?;
    let Some(TargetParameters::Surface(uv)) = nearest.parameters else {
        unreachable!("a surface's nearest point reports its parameters");
    };
    if !on_face(trim, uv) {
        return Err(ProjectionError::Miss);
    }
    Ok(PointProjection {
        point: nearest.point,
        parameters: nearest.parameters,
        distance: nearest.distance,
        hit: face.key(),
    })
}

/// The first hit of the half-line on the face's support that is inside the
/// trim.
pub(super) fn wall_of_face<P: Payload>(
    source: &Point3,
    face: &Face<'_, P>,
    trim: &FaceTrimDomain,
    direction: UnitVector3<f64>,
    options: &ProjectOptions,
) -> Result<PointProjection<FaceKey>, ProjectionError> {
    let hits = ray_hits(source, face.surface(), direction, options)?;
    let wall = hits
        .into_iter()
        .find(|hit| on_face(trim, hit.uv))
        .ok_or(ProjectionError::Miss)?;
    reject_grazing(face.surface(), &wall, direction)?;
    Ok(wall.into_projection(face.key()))
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;
    use crate::builders::faces::add_polygon_with_holes;
    use crate::builders::solids::add_cylinder;
    use crate::geometry::{Frame, Plane};
    use crate::model::Model;
    use crate::projection::project;
    use crate::topology::StandardPayload;

    fn square(low: f64, high: f64) -> Vec<Point3> {
        vec![
            Point3::new(low, low, 0.0),
            Point3::new(high, low, 0.0),
            Point3::new(high, high, 0.0),
            Point3::new(low, high, 0.0),
        ]
    }

    fn square_with_hole() -> (Model<StandardPayload>, FaceKey) {
        let mut model = Model::<StandardPayload>::new();
        let face = add_polygon_with_holes(
            &mut model,
            Plane::xy(),
            &square(0.0, 4.0),
            &[&square(1.0, 3.0)],
        )
        .unwrap();
        (model, face)
    }

    #[test]
    fn a_point_above_a_faces_material_projects_onto_it_in_both_modes() {
        let (model, key) = square_with_hole();
        let face = model.face_unchecked(key);
        let source = Point3::new(0.5, 0.5, 2.0);
        for mode in [Projection::Normal, Projection::along(-Vector3::z())] {
            let result = project(&source, &face, mode).unwrap();
            assert!((result.point - Point3::new(0.5, 0.5, 0.0)).norm() < 1e-9);
            assert!((result.distance - 2.0).abs() < 1e-9);
            assert_eq!(result.hit, key);
        }
    }

    #[test]
    fn a_point_above_a_faces_hole_misses_in_both_modes() {
        let (model, key) = square_with_hole();
        let face = model.face_unchecked(key);
        let source = Point3::new(2.0, 2.0, 2.0);
        for mode in [Projection::Normal, Projection::along(-Vector3::z())] {
            let result = project(&source, &face, mode);
            assert!(matches!(result, Err(ProjectionError::Miss)));
        }
    }

    #[test]
    fn a_point_beyond_a_faces_boundary_misses_in_both_modes() {
        let (model, key) = square_with_hole();
        let face = model.face_unchecked(key);
        let source = Point3::new(6.0, 0.5, 2.0);
        for mode in [Projection::Normal, Projection::along(-Vector3::z())] {
            let result = project(&source, &face, mode);
            assert!(matches!(result, Err(ProjectionError::Miss)));
        }
    }

    #[test]
    fn a_ray_continues_past_a_support_hit_outside_the_trim() {
        let mut model = Model::<StandardPayload>::new();
        let cylinder = add_cylinder(&mut model, Frame::xyz(), 1.0, 2.0).unwrap();
        let face = model.face_unchecked(cylinder.laterals[0].face);
        // The ray meets the infinite cylinder first below the wall, at
        // z = -0.2, then on the wall at z = 0.2.
        let source = Point3::new(-5.0, 0.0, -1.0);
        let result = project(
            &source,
            &face,
            Projection::along(Vector3::new(1.0, 0.0, 0.2)),
        )
        .unwrap();
        assert!((result.point - Point3::new(1.0, 0.0, 0.2)).norm() < 1e-9);
        assert_eq!(result.hit, cylinder.laterals[0].face);
    }

    #[test]
    fn a_ray_whose_every_support_hit_is_outside_the_trim_misses() {
        let mut model = Model::<StandardPayload>::new();
        let cylinder = add_cylinder(&mut model, Frame::xyz(), 1.0, 2.0).unwrap();
        let face = model.face_unchecked(cylinder.laterals[0].face);
        let source = Point3::new(-5.0, 0.0, 5.0);
        let result = project(&source, &face, Projection::along(Vector3::x()));
        assert!(matches!(result, Err(ProjectionError::Miss)));
    }

    #[test]
    fn a_nearest_point_past_a_cylinders_height_misses() {
        let mut model = Model::<StandardPayload>::new();
        let cylinder = add_cylinder(&mut model, Frame::xyz(), 1.0, 2.0).unwrap();
        let face = model.face_unchecked(cylinder.laterals[0].face);
        let beside = Point3::new(4.0, 0.0, 1.0);
        let result = project(&beside, &face, Projection::Normal).unwrap();
        assert!((result.point - Point3::new(1.0, 0.0, 1.0)).norm() < 1e-9);
        let above = Point3::new(4.0, 0.0, 3.0);
        let result = project(&above, &face, Projection::Normal);
        assert!(matches!(result, Err(ProjectionError::Miss)));
    }
}
