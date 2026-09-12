//! Periodic faces written the way an import carries them, with a seam.
//!
//! No builder in this tree makes a seam: a swept or revolved wall comes out a
//! ring, a sphere comes out one face with no boundary at all. STEP AP242 and
//! every other B-Rep interchange format writes a periodic face with its
//! parameterization cut open, so a seam now arrives only from outside — which
//! makes these hand-built maps the honest subject for anything about seams,
//! rather than a builder that correctly refuses to produce one.
//!
//! The shapes differ in what is left once the cut is gone: a wall keeps two rims
//! and becomes a ring, a cap keeps one and is closed on its far side by a pole,
//! and a sphere keeps none at all. A torus is the one that carries two cuts, so
//! the first removal leaves a face that still needs the second.
//!
//! Shared by the `builders` and `healing` test binaries, each of which uses a
//! subset: the allow below is for the half the other binary needs.
#![allow(dead_code)]

use std::collections::HashMap;

use nalgebra::Vector3;
use ngk::geometry::{
    Circle, Curve, Cylinder, Frame, Plane, Point2, Point3, Sphere, Surface, Torus, TrimmedCurve2,
};
use ngk::topology::attributes::{
    EdgeAttr, FaceAttr, ProfileAttr, SheetAttr, ShellRoot, SolidAttr, VertexAttr,
};
use ngk::topology::gmap::{Dart, Dim, GMap};
use ngk::topology::shape_keys::{EdgeKey, FaceKey, SolidKey};
use ngk::topology::{StandardPayload, TopologyEditError};

/// An edge one face's boundary walks twice, with a dart on it.
pub fn seam_of(g: &GMap<StandardPayload>, face: FaceKey) -> Option<(EdgeKey, Dart)> {
    let view = g.face(face)?;
    let edges = view
        .loops()
        .iter()
        .flat_map(|l| l.edges())
        .collect::<Vec<_>>();
    edges
        .iter()
        .find(|edge| edges.iter().filter(|o| o.key() == edge.key()).count() == 2)
        .map(|edge| (edge.key(), edge.dart()))
}

/// Builds a sphere the way a seamed import carries one.
///
/// A meridian arc with both ends on the axis, revolved a whole turn: the swept
/// copy is sewn back onto the arc, so the face's single loop walks the one
/// meridian edge twice between two pole vertices. That is the shape STEP writes
/// for a sphere, and the one `solids::sphere` no longer builds.
pub fn seamed_revolved_sphere(radius: f64) -> (GMap<StandardPayload>, FaceKey, SolidKey) {
    use ngk::builders::edges::add_arc;
    use ngk::builders::revolve::add_revolved_edge;
    use radians::Rad64;
    use std::f64::consts::FRAC_PI_2;

    let mut g = GMap::<StandardPayload>::new();
    let meridian = add_arc(
        &mut g,
        Plane::from_xy(Point3::origin(), Vector3::x(), Vector3::z()),
        radius,
        FRAC_PI_2,
        -FRAC_PI_2,
    )
    .expect("meridian arc should build");
    let face = add_revolved_edge(
        &mut g,
        meridian,
        ngk::geometry::axis::Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    )
    .expect("a full revolution of the meridian should build");
    let solid = g
        .transaction(|edit| {
            let seed = edit
                .face(face)
                .expect("the revolved face is registered")
                .dart()
                .expect("a seamed face has a boundary to root at");
            let shell = ShellRoot::Dart(seed);
            edit.add_sheet(SheetAttr::new(shell, ()));
            Ok::<_, TopologyEditError>(edit.add_solid(SolidAttr::new((), shell, None)))
        })
        .expect("the revolved sphere should close into a solid");
    (g, face, solid)
}

/// Builds a spherical cap the way a seamed import carries one.
///
/// The northern cap above `latitude`: one latitude circle, and a meridian run
/// from it up to the pole and back down, which is the seam. No builder makes
/// this — `add_revolved_edge` writes the cap's loop as `Capping` outright — but
/// a STEP face cut open along `u = 0` arrives exactly like this.
pub fn seamed_spherical_cap(radius: f64, latitude: f64) -> (GMap<StandardPayload>, FaceKey) {
    let mut g = GMap::<StandardPayload>::new();
    let sphere = Sphere::new(Frame::xyz(), radius);
    let surface = Surface::Sphere(sphere.clone());
    let turn = std::f64::consts::TAU;
    let pole = std::f64::consts::FRAC_PI_2;

    let face = g
        .transaction(|edit| {
            // The boundary in order: the latitude circle, the meridian up to the
            // pole, and the same meridian back down.
            let d: [Dart; 6] = std::array::from_fn(|_| edit.add_dart());
            for pair in 0..3 {
                edit.link(Dim::Zero, d[2 * pair], d[2 * pair + 1])?;
            }
            for pair in 0..3 {
                edit.link(Dim::One, d[2 * pair + 1], d[(2 * pair + 2) % 6])?;
            }

            let seam_foot = sphere.point_at(0.0, latitude);
            let apex = sphere.point_at(0.0, pole);
            edit.add_vertex(VertexAttr::new(d[0], seam_foot, ()));
            edit.add_vertex(VertexAttr::new(d[3], apex, ()));

            edit.add_edge(EdgeAttr::new(
                d[0],
                Curve::Circle(Circle::new(
                    Plane::from_xy(
                        Point3::new(0.0, 0.0, radius * latitude.sin()),
                        Vector3::x(),
                        Vector3::y(),
                    ),
                    radius * latitude.cos(),
                )),
                (),
            ));
            // One edge for both meridian sides: that is what makes it a seam.
            edit.add_edge(EdgeAttr::new(d[2], Curve::line(seam_foot, apex), ()));
            edit.add_profile(ProfileAttr::new(d[0], ()));

            // The pole is a whole row of the domain collapsed to a point, so the
            // loop turns through it carrying no pcurve: the meridian's two
            // occurrences run up at `u = turn` and back down at `u = 0`.
            let pcurves = HashMap::from([
                (
                    d[0],
                    TrimmedCurve2::segment(Point2::new(0.0, latitude), Point2::new(turn, latitude)),
                ),
                (
                    d[2],
                    TrimmedCurve2::segment(Point2::new(turn, latitude), Point2::new(turn, pole)),
                ),
                (
                    d[4],
                    TrimmedCurve2::segment(Point2::new(0.0, pole), Point2::new(0.0, latitude)),
                ),
            ]);
            let face = edit.add_face(FaceAttr::with_pcurves(
                surface.clone(),
                (),
                d[0],
                Vec::new(),
                pcurves,
            ));

            // The two meridian sides meet along the seam: foot to foot, pole to
            // pole.
            edit.sew(Dim::Two, d[2], d[5])?;
            Ok::<_, TopologyEditError>(face)
        })
        .expect("a seamed cap should commit");
    (g, face)
}

/// Builds a cylinder wall the way a seamed import carries one.
///
/// No builder in the tree makes this any more — a swept or revolved wall comes
/// out as a ring — but STEP and every other B-Rep interchange format writes a
/// periodic face with its parameterization cut open, so the canonicalizer has to
/// be able to take one apart. The wall is one quad face whose two vertical sides
/// are the same edge, sewn to itself: that self-sew is the seam.
pub fn seamed_cylinder_wall(radius: f64, height: f64) -> (GMap<StandardPayload>, FaceKey) {
    let mut g = GMap::<StandardPayload>::new();
    let surface = Surface::Cylinder(Cylinder::new(
        Point3::origin(),
        Vector3::x(),
        Vector3::z(),
        radius,
    ));
    let turn = std::f64::consts::TAU;
    let face = g
        .transaction(|edit| {
            // The quad boundary: bottom circle, seam up, top circle, seam down.
            let d: [Dart; 8] = std::array::from_fn(|_| edit.add_dart());
            for pair in 0..4 {
                edit.link(Dim::Zero, d[2 * pair], d[2 * pair + 1])?;
            }
            for pair in 0..4 {
                edit.link(Dim::One, d[2 * pair + 1], d[(2 * pair + 2) % 8])?;
            }

            let bottom = Point3::new(radius, 0.0, 0.0);
            let top = Point3::new(radius, 0.0, height);
            edit.add_vertex(VertexAttr::new(d[0], bottom, ()));
            edit.add_vertex(VertexAttr::new(d[4], top, ()));

            let circle = |z: f64| {
                Curve::Circle(Circle::new(
                    Plane::from_xy(Point3::new(0.0, 0.0, z), Vector3::x(), Vector3::y()),
                    radius,
                ))
            };
            edit.add_edge(EdgeAttr::new(d[0], circle(0.0), ()));
            edit.add_edge(EdgeAttr::new(d[4], circle(height), ()));
            // One edge for both vertical sides: that is what makes it a seam.
            let seam = edit.add_edge(EdgeAttr::new(d[2], Curve::line(bottom, top), ()));
            edit.add_profile(ProfileAttr::new(d[0], ()));

            let pcurves = HashMap::from([
                (
                    d[0],
                    TrimmedCurve2::segment(Point2::origin(), Point2::new(turn, 0.0)),
                ),
                (
                    d[2],
                    TrimmedCurve2::segment(Point2::new(turn, 0.0), Point2::new(turn, height)),
                ),
                (
                    d[4],
                    TrimmedCurve2::segment(Point2::new(turn, height), Point2::new(0.0, height)),
                ),
                (
                    d[6],
                    TrimmedCurve2::segment(Point2::new(0.0, height), Point2::origin()),
                ),
            ]);
            let face = edit.add_face(FaceAttr::with_pcurves(
                surface.clone(),
                (),
                d[0],
                Vec::new(),
                pcurves,
            ));

            // The two sides meet along the seam: bottom to bottom, top to top.
            edit.sew(Dim::Two, d[2], d[7])?;
            let _ = seam;
            Ok::<_, TopologyEditError>(face)
        })
        .expect("a seamed wall should commit");
    (g, face)
}

/// Builds a torus the way a seamed import carries one.
///
/// A torus closes in both parameters, so cutting it open takes two cuts: the
/// face is one quad whose bottom and top sides are the same outer-equator edge
/// and whose left and right sides are the same tube-circle edge, with the single
/// vertex at all four corners. That is two seams on one face — the shape that
/// needs the removals to compose, since taking the first cut off leaves a face
/// still carrying the second.
pub fn seamed_torus(major: f64, minor: f64) -> (GMap<StandardPayload>, FaceKey, SolidKey) {
    let mut g = GMap::<StandardPayload>::new();
    let torus = Torus::new(Frame::xyz(), major, minor);
    let surface = Surface::Torus(torus.clone());
    let turn = std::f64::consts::TAU;

    let face = g
        .transaction(|edit| {
            // The quad boundary: equator across, tube circle up, equator back,
            // tube circle down.
            let d: [Dart; 8] = std::array::from_fn(|_| edit.add_dart());
            for pair in 0..4 {
                edit.link(Dim::Zero, d[2 * pair], d[2 * pair + 1])?;
            }
            for pair in 0..4 {
                edit.link(Dim::One, d[2 * pair + 1], d[(2 * pair + 2) % 8])?;
            }

            // Both cuts pass through `(u, v) = (0, 0)`, so the quad's four
            // corners are one point.
            let corner = torus.point_at(0.0, 0.0);
            edit.add_vertex(VertexAttr::new(d[0], corner, ()));

            // One edge for the bottom and top sides, one for the left and right:
            // that is what makes each of them a seam.
            edit.add_edge(EdgeAttr::new(
                d[0],
                Curve::Circle(Circle::new(
                    Plane::from_xy(Point3::origin(), Vector3::x(), Vector3::y()),
                    major + minor,
                )),
                (),
            ));
            edit.add_edge(EdgeAttr::new(
                d[2],
                Curve::Circle(Circle::new(
                    Plane::from_xy(Point3::new(major, 0.0, 0.0), Vector3::x(), Vector3::z()),
                    minor,
                )),
                (),
            ));
            edit.add_profile(ProfileAttr::new(d[0], ()));

            let pcurves = HashMap::from([
                (
                    d[0],
                    TrimmedCurve2::segment(Point2::origin(), Point2::new(turn, 0.0)),
                ),
                (
                    d[2],
                    TrimmedCurve2::segment(Point2::new(turn, 0.0), Point2::new(turn, turn)),
                ),
                (
                    d[4],
                    TrimmedCurve2::segment(Point2::new(turn, turn), Point2::new(0.0, turn)),
                ),
                (
                    d[6],
                    TrimmedCurve2::segment(Point2::new(0.0, turn), Point2::origin()),
                ),
            ]);
            let face = edit.add_face(FaceAttr::with_pcurves(
                surface.clone(),
                (),
                d[0],
                Vec::new(),
                pcurves,
            ));

            // The equator's two occurrences meet along the tube cut, and the
            // tube circle's two along the equator cut.
            edit.sew(Dim::Two, d[0], d[5])?;
            edit.sew(Dim::Two, d[2], d[7])?;
            Ok::<_, TopologyEditError>(face)
        })
        .expect("a seamed torus should commit");

    let solid = g
        .transaction(|edit| {
            let seed = edit
                .face(face)
                .expect("the torus face is registered")
                .dart()
                .expect("a seamed face has a boundary to root at");
            let shell = ShellRoot::Dart(seed);
            edit.add_sheet(SheetAttr::new(shell, ()));
            Ok::<_, TopologyEditError>(edit.add_solid(SolidAttr::new((), shell, None)))
        })
        .expect("the seamed torus should close into a solid");
    (g, face, solid)
}
