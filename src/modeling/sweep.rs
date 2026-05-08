use std::collections::HashMap;

use nalgebra::Vector3;

use crate::builders::solids::add_extruded_face;
use crate::builders::{errors::ExtrudeError, sheets::add_extruded_profile};
use crate::geometry::{
    Curve, Curve2, Line, Line2, NurbsError, Plane, Point2, Point3, RuledSurface, Surface,
};
use crate::topology::attributes::{EdgeAttr, FaceAttr, SolidAttr, VertexAttr};
use crate::topology::closed::Closeable;
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::gmap::{Dart, Dim, GMap};
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape::{FaceShape, Shape, SheetShape, SolidShape};

pub fn extrude_profile<P: Payload>(
    profile: Profile<'_, P>,
    direction: Vector3<f64>,
) -> Result<Shape<SheetShape, P>, ExtrudeError> {
    let mut g = GMap::<P>::new();
    add_extruded_profile(&mut g, profile.dart, direction)?;
    let sheet_dart = add_extruded_profile(&mut g, profile.dart, direction)?;
    Ok(Shape::new(g, sheet_dart))
}

pub fn extrude_face<P: Payload>(
    face: Shape<FaceShape, P>,
    direction: Vector3<f64>,
) -> Result<Shape<SolidShape, P>, ExtrudeError> {
    let (mut g, face_key) = face.into_map();
    let solid_key = add_extruded_face(&mut g, face_key, direction)?;
    Ok(Shape::new(g, solid_key))
}
