use std::collections::HashMap;

use nalgebra::Vector3;

use crate::builders::sheets::{ExtrudeError, add_extruded_profile};
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
use crate::topology::shape::{Shape, SheetShape, SolidShape};

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
    face: Face<'_, P>,
    direction: Vector3<f64>,
) -> Result<Shape<SolidShape, P>, ExtrudeError> {
    if direction.norm_squared() <= f64::EPSILON {
        return Err(ExtrudeError::ZeroDirection);
    }
    todo!()

    // let mut g = GMap::<P>::new();
    // let sheet_dart = add_extruded_profile(&mut g, face.outer_loop().into_inner().dart, direction)?;
    // let bot_cap = g.merge(&face);

    // let solid_key = add_face(&mut g, face.outer_loop())?;
    // Ok(Shape::new(g, solid_key))
}
