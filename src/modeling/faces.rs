use crate::builders::errors::FaceCreationError;
use crate::builders::faces::{
    add_annulus, add_circle, add_face_staged, add_polygon_staged, add_polygon_with_holes,
    add_rectangle, add_square,
};
use crate::geometry::{Plane, Point3};
use crate::model::Model;
use crate::topology::payload::{DefaultPayload, StandardPayload};
use crate::topology::profile::Profile;
use crate::topology::shape::{FaceTag, ProfileTag, Shape};

pub fn rectangle(
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<Shape<FaceTag, StandardPayload>, FaceCreationError> {
    let mut g = Model::new();
    let face_key = add_rectangle(&mut g, plane, x_size, y_size)?;
    Ok(Shape::new(g, face_key))
}

pub fn square(
    plane: Plane,
    size: f64,
) -> Result<Shape<FaceTag, StandardPayload>, FaceCreationError> {
    let mut g = Model::new();
    let handle = add_square(&mut g, plane, size)?;
    Ok(Shape::new(g, handle))
}

pub fn circle(
    plane: Plane,
    radius: f64,
) -> Result<Shape<FaceTag, StandardPayload>, FaceCreationError> {
    let mut g = Model::new();
    let face_key = add_circle(&mut g, plane, radius)?;
    Ok(Shape::new(g, face_key))
}

pub fn annulus(
    plane: Plane,
    outer_radius: f64,
    inner_radius: f64,
) -> Result<Shape<FaceTag, StandardPayload>, FaceCreationError> {
    let mut g = Model::new();
    let face_key = add_annulus(&mut g, plane, outer_radius, inner_radius)?;
    Ok(Shape::new(g, face_key))
}

pub fn polygon(points: &[Point3]) -> Result<Shape<FaceTag, StandardPayload>, FaceCreationError> {
    if points.len() < 3 {
        return Err(FaceCreationError::InvalidPolygon {
            point_count: points.len(),
        });
    }

    let mut g = Model::new();
    let face_key = g.transaction(|edit| {
        let profile_key = add_polygon_staged(edit, points);
        add_face_staged(edit, profile_key)
    })?;
    Ok(Shape::new(g, face_key))
}

/// Builds an owned face bounded by an existing profile's loop.
///
/// The profile must be closed and planar. The returned shape owns a copy of
/// the profile; the source shape is unchanged.
pub fn from_profile<P: DefaultPayload>(
    profile: &Shape<ProfileTag, P>,
) -> Result<Shape<FaceTag, P>, FaceCreationError> {
    let mut g = Model::new();
    let face_key = g.transaction(|edit| {
        let dart = edit.merge(profile.profile());
        let profile_key = Profile::from_dart(edit.model(), dart)
            .expect("a merged profile is registered under its own key")
            .key();
        add_face_staged(edit, profile_key)
    })?;
    Ok(Shape::new(g, face_key))
}

pub fn polygon_with_holes(
    plane: Plane,
    outer: &[Point3],
    holes: &[&[Point3]],
) -> Result<Shape<FaceTag, StandardPayload>, FaceCreationError> {
    let mut g = Model::new();
    let face_key = add_polygon_with_holes(&mut g, plane, outer, holes)?;
    Ok(Shape::new(g, face_key))
}
