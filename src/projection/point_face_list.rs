use super::error::ProjectionError;
use super::mode::Projection;
use super::options::ProjectOptions;
use super::point_face::{foot_in_face, wall_of_face};
use super::project::Project;
use super::result::PointProjection;
use crate::geometry::Point3;
use crate::model::Model;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::FaceKey;
use crate::topology::trim_domain::FaceTrimDomain;

/// A list of faces of one model, projected onto as a whole.
pub struct FaceList<'a, P: Payload> {
    model: &'a Model<P>,
    faces: &'a [FaceKey],
}

impl<'a, P: Payload> FaceList<'a, P> {
    /// The faces `faces` of `model`.
    pub fn new(model: &'a Model<P>, faces: &'a [FaceKey]) -> Self {
        Self { model, faces }
    }

    /// The model the faces belong to.
    pub(super) fn model(&self) -> &'a Model<P> {
        self.model
    }

    /// The keys of the faces, in list order.
    pub(super) fn faces(&self) -> &'a [FaceKey] {
        self.faces
    }
}

impl<P: Payload> Project<FaceList<'_, P>> for Point3 {
    type Mode = Projection;
    type Hit = FaceKey;
    type Output = PointProjection<FaceKey>;

    /// Projects onto the faces of the list, each existing only where it is
    /// trimmed.
    ///
    /// [`Normal`](Projection::Normal) keeps the nearest of the faces' feet that
    /// are inside their trims. Feet that are equally near within tolerance but
    /// are different points are [`Ambiguous`](ProjectionError::Ambiguous); the
    /// same foot on two faces, as on an edge they share, is one answer.
    ///
    /// [`Along`](Projection::Along) keeps the first wall the half-line meets.
    /// A wall on an edge two faces share is one point; the face earlier in the
    /// list is reported.
    ///
    /// A face the point misses is skipped, and the list is a
    /// [`Miss`](ProjectionError::Miss) when it misses them all. Any other
    /// refusal of a face's support refuses the whole list.
    fn project(
        &self,
        list: &FaceList<'_, P>,
        mode: Projection,
        options: &ProjectOptions,
    ) -> Result<PointProjection<FaceKey>, ProjectionError> {
        let mut candidates = Vec::new();
        for &key in list.faces {
            let face = list.model.face_unchecked(key);
            let trim = FaceTrimDomain::new(&face, options.tolerance)?;
            let candidate = match mode {
                Projection::Normal => foot_in_face(self, &face, &trim, options),
                Projection::Along(direction) => {
                    wall_of_face(self, &face, &trim, direction, options)
                }
            };
            match candidate {
                Ok(candidate) => candidates.push(candidate),
                Err(ProjectionError::Miss) => {}
                Err(error) => return Err(error),
            }
        }
        nearest(candidates, matches!(mode, Projection::Normal), options)
    }
}

/// The candidate nearest the source. Candidates closer than the tolerance keep
/// the earlier one; when `refuse_ties`, near ones at different points are a
/// tie.
pub(super) fn nearest(
    candidates: Vec<PointProjection<FaceKey>>,
    refuse_ties: bool,
    options: &ProjectOptions,
) -> Result<PointProjection<FaceKey>, ProjectionError> {
    let mut candidates = candidates.into_iter();
    let mut best = candidates.next().ok_or(ProjectionError::Miss)?;
    let mut tied_elsewhere = false;
    for candidate in candidates {
        if candidate.distance < best.distance - options.tolerance {
            best = candidate;
            tied_elsewhere = false;
        } else if candidate.distance <= best.distance + options.tolerance
            && (candidate.point - best.point).norm() > options.tolerance
        {
            tied_elsewhere = true;
        }
    }
    if refuse_ties && tied_elsewhere {
        return Err(ProjectionError::Ambiguous);
    }
    Ok(best)
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;
    use crate::builders::faces::{add_polygon_with_holes, add_rectangle};
    use crate::geometry::{Frame, Plane};
    use crate::projection::project;
    use crate::topology::StandardPayload;

    fn options() -> ProjectOptions {
        ProjectOptions::default()
    }

    /// A 4 x 4 square in the plane `z = height`, with a hole when `hole`.
    fn plate(model: &mut Model<StandardPayload>, height: f64, hole: bool) -> FaceKey {
        let at = |x: f64, y: f64| Point3::new(x, y, height);
        let outer = [at(0.0, 0.0), at(4.0, 0.0), at(4.0, 4.0), at(0.0, 4.0)];
        let inner = [at(1.0, 1.0), at(3.0, 1.0), at(3.0, 3.0), at(1.0, 3.0)];
        let plane = Plane::from_frame(Frame::at(at(0.0, 0.0)));
        let holes: &[&[Point3]] = if hole { &[&inner] } else { &[] };
        add_polygon_with_holes(model, plane, &outer, holes).unwrap()
    }

    #[test]
    fn the_nearest_face_of_the_list_wins_in_both_modes() {
        let mut model = Model::<StandardPayload>::new();
        let low = plate(&mut model, 0.0, false);
        let high = plate(&mut model, 1.0, false);
        let source = Point3::new(0.5, 0.5, 3.0);
        for faces in [[low, high], [high, low]] {
            let list = FaceList::new(&model, &faces);
            for mode in [Projection::Normal, Projection::along(-Vector3::z())] {
                let result = project(&source, &list, mode, &options()).unwrap();
                assert_eq!(result.hit, high);
                assert!((result.distance - 2.0).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn a_ray_through_a_hole_reaches_the_next_face() {
        let mut model = Model::<StandardPayload>::new();
        let low = plate(&mut model, 0.0, false);
        let high = plate(&mut model, 1.0, true);
        let faces = [high, low];
        let list = FaceList::new(&model, &faces);
        let source = Point3::new(2.0, 2.0, 3.0);
        let result = project(&source, &list, Projection::along(-Vector3::z()), &options()).unwrap();
        assert_eq!(result.hit, low);
        assert!((result.distance - 3.0).abs() < 1e-9);
    }

    #[test]
    fn a_face_the_point_misses_is_skipped_in_normal_mode() {
        let mut model = Model::<StandardPayload>::new();
        let low = plate(&mut model, 0.0, false);
        let high = plate(&mut model, 1.0, true);
        let faces = [high, low];
        let list = FaceList::new(&model, &faces);
        let source = Point3::new(2.0, 2.0, 3.0);
        let result = project(&source, &list, Projection::Normal, &options()).unwrap();
        assert_eq!(result.hit, low);
    }

    #[test]
    fn a_point_beside_every_face_misses() {
        let mut model = Model::<StandardPayload>::new();
        let low = plate(&mut model, 0.0, false);
        let faces = [low];
        let list = FaceList::new(&model, &faces);
        let source = Point3::new(9.0, 9.0, 1.0);
        for mode in [Projection::Normal, Projection::along(-Vector3::z())] {
            let result = project(&source, &list, mode, &options());
            assert!(matches!(result, Err(ProjectionError::Miss)));
        }
    }

    #[test]
    fn an_empty_list_misses() {
        let model = Model::<StandardPayload>::new();
        let list = FaceList::new(&model, &[]);
        let result = project(&Point3::origin(), &list, Projection::Normal, &options());
        assert!(matches!(result, Err(ProjectionError::Miss)));
    }

    #[test]
    fn equally_near_feet_at_different_points_are_ambiguous() {
        let mut model = Model::<StandardPayload>::new();
        let floor = add_rectangle(&mut model, Plane::xy(), 4.0, 4.0).unwrap();
        let upper = Plane::from_frame(Frame::at(Point3::new(0.0, 0.0, 2.0)));
        let ceiling = add_rectangle(&mut model, upper, 4.0, 4.0).unwrap();
        let faces = [floor, ceiling];
        let list = FaceList::new(&model, &faces);
        let source = Point3::new(2.0, 2.0, 1.0);
        let result = project(&source, &list, Projection::Normal, &options());
        assert!(matches!(result, Err(ProjectionError::Ambiguous)));
    }
}
