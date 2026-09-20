use crate::builders::errors::FaceCreationError;
use crate::builders::faces::{
    add_annulus, add_circle, add_face_edit, add_polygon_edit, add_polygon_with_holes,
    add_rectangle, add_square,
};
use crate::geometry::{Plane, Point3};
use crate::model::Model;
use crate::topology::payload::{Payload, StandardPayload};
use crate::topology::profile::Profile;
use crate::topology::shape::{FaceTag, ProfileTag, Shape};

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
