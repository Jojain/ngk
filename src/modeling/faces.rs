use crate::builders::boolean::{BooleanError, BooleanOperation, BooleanOptions, face_boolean};
use crate::builders::errors::FaceCreationError;
use crate::builders::faces::{
    FaceEdgeSplitError, add_annulus, add_circle, add_face_edit, add_polygon_edit,
    add_polygon_with_holes, add_rectangle, add_square, split_face_edge,
};
use crate::geometry::{Fraction, Plane, Point3};
use crate::model::Cell2;
use crate::model::Model;
use crate::modeling::edges::EdgeSplitResult;
use crate::topology::ModelEditError;
use crate::topology::face::Face;
use crate::topology::payload::{Payload, StandardPayload};
use crate::topology::profile::Profile;
use crate::topology::shape::{FaceTag, ProfileTag, Shape};
use crate::topology::shape_keys::{EdgeKey, FaceKey};

/// Creates a planar rectangular face whose first corner is `plane.origin()`.
pub fn rectangle(
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<Shape<FaceTag>, FaceCreationError> {
    rectangle_with::<StandardPayload>(plane, x_size, y_size)
}

/// As [`rectangle`], with the payload chosen by the caller.
pub fn rectangle_with<P: Payload>(
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<Shape<FaceTag, P>, FaceCreationError> {
    Shape::build(|model| add_rectangle(model, plane, x_size, y_size))
}

/// Creates a planar square face whose first corner is `plane.origin()`.
pub fn square(plane: Plane, size: f64) -> Result<Shape<FaceTag>, FaceCreationError> {
    square_with::<StandardPayload>(plane, size)
}

/// As [`square`], with the payload chosen by the caller.
pub fn square_with<P: Payload>(
    plane: Plane,
    size: f64,
) -> Result<Shape<FaceTag, P>, FaceCreationError> {
    Shape::build(|model| add_square(model, plane, size))
}

/// Creates a planar circular face with the specified radius.
pub fn circle(plane: Plane, radius: f64) -> Result<Shape<FaceTag>, FaceCreationError> {
    circle_with::<StandardPayload>(plane, radius)
}

/// As [`circle`], with the payload chosen by the caller.
pub fn circle_with<P: Payload>(
    plane: Plane,
    radius: f64,
) -> Result<Shape<FaceTag, P>, FaceCreationError> {
    Shape::build(|model| add_circle(model, plane, radius))
}

/// Creates a planar annular face with the specified outer and inner radii.
pub fn annulus(
    plane: Plane,
    outer_radius: f64,
    inner_radius: f64,
) -> Result<Shape<FaceTag>, FaceCreationError> {
    annulus_with::<StandardPayload>(plane, outer_radius, inner_radius)
}

/// As [`annulus`], with the payload chosen by the caller.
pub fn annulus_with<P: Payload>(
    plane: Plane,
    outer_radius: f64,
    inner_radius: f64,
) -> Result<Shape<FaceTag, P>, FaceCreationError> {
    Shape::build(|model| add_annulus(model, plane, outer_radius, inner_radius))
}

/// Creates a planar face bounded by the supplied polygon corners.
pub fn polygon(points: &[Point3]) -> Result<Shape<FaceTag>, FaceCreationError> {
    polygon_with::<StandardPayload>(points)
}

/// As [`polygon`], with the payload chosen by the caller.
pub fn polygon_with<P: Payload>(points: &[Point3]) -> Result<Shape<FaceTag, P>, FaceCreationError> {
    if points.len() < 3 {
        return Err(FaceCreationError::InvalidPolygon {
            point_count: points.len(),
        });
    }
    Shape::build(|model| {
        model.transaction(|edit| {
            let profile = add_polygon_edit(edit, points);
            add_face_edit(edit, profile)
        })
    })
}

/// Builds an owned face bounded by an existing profile's loop.
///
/// The profile must be closed and planar. The returned shape owns a copy of
/// the profile; the source shape is unchanged.
pub fn from_profile<P: Payload>(
    profile: &Shape<ProfileTag, P>,
) -> Result<Shape<FaceTag, P>, FaceCreationError> {
    let mut g = Model::new();
    let face_key = g.transaction(|edit| {
        let dart = edit.merge(profile.profile());
        let profile_key = Profile::from_dart(edit.model(), dart)
            .expect("a merged profile is registered under its own key")
            .key();
        add_face_edit(edit, profile_key)
    })?;
    Ok(Shape::new(g, face_key))
}

/// Creates a planar face bounded by an outer polygon and zero or more holes.
pub fn polygon_with_holes(
    plane: Plane,
    outer: &[Point3],
    holes: &[&[Point3]],
) -> Result<Shape<FaceTag>, FaceCreationError> {
    polygon_with_holes_with::<StandardPayload>(plane, outer, holes)
}

/// As [`polygon_with_holes`], with the payload chosen by the caller.
pub fn polygon_with_holes_with<P: Payload>(
    plane: Plane,
    outer: &[Point3],
    holes: &[&[Point3]],
) -> Result<Shape<FaceTag, P>, FaceCreationError> {
    Shape::build(|model| add_polygon_with_holes(model, plane, outer, holes))
}

/// A face Boolean answer: one model and every surviving face handle.
pub struct FaceBooleanResult<P: Payload = StandardPayload> {
    model: Model<P>,
    faces: Vec<FaceKey>,
}

impl<P: Payload> FaceBooleanResult<P> {
    pub fn model(&self) -> &Model<P> {
        &self.model
    }

    pub fn face_keys(&self) -> &[FaceKey] {
        &self.faces
    }

    pub fn faces(&self) -> Vec<Face<'_, P>> {
        self.faces
            .iter()
            .map(|key| self.model.face_unchecked(*key))
            .collect()
    }

    pub fn into_model(self) -> (Model<P>, Vec<FaceKey>) {
        (self.model, self.faces)
    }
}

/// Fuses two owned coplanar faces, returning every surviving face in one model.
pub fn fuse<P: Payload>(
    first: Shape<FaceTag, P>,
    second: Shape<FaceTag, P>,
) -> Result<FaceBooleanResult<P>, BooleanError> {
    combine_shapes(first, second, BooleanOperation::Union)
}

/// Subtracts the second owned coplanar face from the first.
pub fn cut<P: Payload>(
    first: Shape<FaceTag, P>,
    second: Shape<FaceTag, P>,
) -> Result<FaceBooleanResult<P>, BooleanError> {
    combine_shapes(first, second, BooleanOperation::Difference)
}

/// Intersects two owned coplanar faces.
pub fn intersect<P: Payload>(
    first: Shape<FaceTag, P>,
    second: Shape<FaceTag, P>,
) -> Result<FaceBooleanResult<P>, BooleanError> {
    combine_shapes(first, second, BooleanOperation::Intersection)
}

fn combine_shapes<P: Payload>(
    first: Shape<FaceTag, P>,
    second: Shape<FaceTag, P>,
    operation: BooleanOperation,
) -> Result<FaceBooleanResult<P>, BooleanError> {
    let (mut model, first_key) = first.into_model();
    let (second_model, second_key) = second.into_model();
    let second_key = model.transaction(|edit| {
        let dart = edit.merge(second_model.face_unchecked(second_key));
        Ok::<_, ModelEditError>(edit.cell_key_unchecked::<Cell2>(dart))
    })?;
    let result = face_boolean(
        &mut model,
        first_key,
        second_key,
        operation,
        BooleanOptions::default(),
    )?;
    Ok(FaceBooleanResult {
        model,
        faces: result.faces,
    })
}

/// Copies two borrowed face views and evaluates their Boolean in one model.
pub(crate) fn combine_views<P: Payload>(
    first: Face<'_, P>,
    second: Face<'_, P>,
    operation: BooleanOperation,
) -> Result<FaceBooleanResult<P>, BooleanError> {
    let mut model = Model::new();
    let (first_key, second_key) = model.transaction(|edit| {
        let first_dart = edit.merge(first);
        let second_dart = edit.merge(second);
        Ok::<_, ModelEditError>((
            edit.cell_key_unchecked::<Cell2>(first_dart),
            edit.cell_key_unchecked::<Cell2>(second_dart),
        ))
    })?;
    let result = face_boolean(
        &mut model,
        first_key,
        second_key,
        operation,
        BooleanOptions::default(),
    )?;
    Ok(FaceBooleanResult {
        model,
        faces: result.faces,
    })
}

/// Cuts a boundary edge and all incident face pcurves in one owned model.
pub fn split_boundary_edge<P: Payload>(
    face: Shape<FaceTag, P>,
    edge: EdgeKey,
    fraction: Fraction,
) -> Result<EdgeSplitResult<P>, FaceEdgeSplitError> {
    let (mut model, face_key) = face.into_model();
    let split = split_face_edge(&mut model, face_key, edge, fraction)?;
    Ok(EdgeSplitResult {
        model,
        split,
        face: Some(face_key),
    })
}
