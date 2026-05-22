use crate::StandardPayload;
use crate::builders::errors::FaceCreationError;
use crate::builders::profiles::{
    add_rectangle as add_rectangle_profile, add_square as add_square_profile, profile_pcurves,
};
use crate::geometry::{Curve, Line, Plane, Point3, Surface};
use crate::topology::attributes::{EdgeAttr, FaceAttr, VertexAttr};
use crate::topology::closed::Closed;
use crate::topology::gmap::{Dart, Dim, GMap};
use crate::topology::payload::Payload;
use crate::topology::planar::Planar;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::FaceKey;

pub fn add_face<P: Payload>(
    g: &mut GMap<P>,
    loop_dart: Dart,
) -> Result<FaceKey, FaceCreationError> {
    let (plane, pcurves) = {
        let profile = Profile::new(g, loop_dart);
        let closed =
            Closed::new(profile).ok_or(FaceCreationError::OpenProfile { dart: loop_dart })?;
        let planar = Planar::new(closed)?;
        let (closed, plane) = planar.into_parts();
        let pcurves = profile_pcurves(g, closed.inner(), &plane)?;
        (plane, pcurves)
    };

    let face_key = g.add_face(FaceAttr::with_pcurves(
        Surface::Plane(plane),
        P::F::default(),
        loop_dart,
        Vec::new(),
        pcurves,
    ));
    Ok(face_key)
}

pub fn add_rectangle(
    g: &mut GMap<StandardPayload>,
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<FaceKey, FaceCreationError> {
    let loop_dart = add_rectangle_profile(g, plane, x_size, y_size)?;
    add_face(g, loop_dart)
}

pub fn add_square(
    g: &mut GMap<StandardPayload>,
    plane: Plane,
    size: f64,
) -> Result<FaceKey, FaceCreationError> {
    let loop_dart = add_square_profile(g, plane, size)?;
    add_face(g, loop_dart)
}

/// Adds a single polygon face to `g` with the given corner points (in order).
///
/// Sews alpha0 and alpha1 to form a closed `n`-gon, stamps the vertex positions on
/// every dart of each corner's vertex orbit, and attaches a straight
/// [`Curve::Line`] on every 1-cell so downstream consumers (edge tessellation,
/// dart geometry) have a curve to follow. Does not touch alpha2; the face is
/// returned with free boundary, ready to be stitched to neighbors.
///
/// Returns a dart on the outer <alpha0, alpha1> loop (same as the first corner dart).
pub fn add_polygon<P: Payload>(g: &mut GMap<P>, corners: &[Point3]) -> Dart {
    assert!(
        corners.len() >= 3,
        "add_polygon requires at least 3 corners, got {}",
        corners.len()
    );
    let n = corners.len();
    let darts: Vec<Dart> = (0..2 * n).map(|_| g.add_dart()).collect();

    for i in 0..n {
        g.sew(Dim::Zero, darts[2 * i], darts[2 * i + 1])
            .expect("fresh dart pair should be alpha0-sewable");
    }
    for i in 0..n {
        let a = darts[2 * i + 1];
        let b = darts[(2 * i + 2) % (2 * n)];
        g.sew(Dim::One, a, b)
            .expect("fresh dart pair should be alpha1-sewable");
    }

    for i in 0..n {
        let dart = g.cell_representative(darts[2 * i], Dim::Zero);
        g.add_vertex(VertexAttr::new(dart, corners[i], P::V::default()));
    }

    for i in 0..n {
        let edge_dart = g.cell_representative(darts[2 * i], Dim::One);
        let curve = Curve::Line(Line::new(corners[i], corners[(i + 1) % n]));
        g.add_edge(EdgeAttr::new(edge_dart, curve, P::E::default()));
    }
    darts[0]
}
