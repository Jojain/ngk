//! Meshing a trimmed face, by the support it is trimmed on.
//!
//! `modeling::solids` already covers the faces a primitive builds, which are
//! whole supports: a sphere, a torus, a cylinder wall. What is left to this file
//! is the other kind — a *patch*, a rectangle of a support's parameter space
//! bounded by an outer loop, which is what a fillet or a blend arrives as. No
//! builder in this tree makes one: `add_revolved_edge` writes its sweeps on
//! `Surface::Revolution`, and a boolean leaves the cut piece carrying an inner
//! loop rather than an outer one. A patch on a closed-form support comes only
//! from an import, so — as with the seamed faces in `tests/support` — the honest
//! subject is a hand-built map.

use std::collections::HashMap;

use nalgebra::Vector3;
use ngk::exchange::step::{StepReadOptions, read_step};
use ngk::geometry::{Circle, Curve, Frame, Plane, Point2, Point3, Surface, Torus, TrimmedCurve2};
use ngk::model::Model;
use ngk::tessellate::{SurfaceOpts, TessellateOpts, face::tessellate_face_key};
use ngk::topology::LoopKind;
use ngk::topology::attributes::{EdgeAttr, FaceAttr, ProfileAttr, VertexAttr};
use ngk::topology::gmap::{Dart, Dim};
use ngk::topology::shape_keys::FaceKey;
use ngk::topology::unwrapped_face_domain::UnwrappedFaceDomain;
use ngk::topology::{ModelEditError, StandardPayload};

/// The `ppp0104` angle bracket: a boss filleted into a plate, and the one
/// fixture in the tree carrying a torus patch that is not a parameter rectangle.
const ANGLE_BRACKET: &str = include_str!("../exchange/foreign/files/ppp0104_angle_bracket.step");

/// A quarter-by-quarter patch of a torus, the way a fillet arrives from STEP.
///
/// The boundary is the rectangle `[0, pi/2]^2` in `(longitude, tube)`, walked
/// counter-clockwise: along the widened circle at `v = 0`, up the tube at
/// `u = pi/2`, back at `v = pi/2`, down the tube at `u = 0`. Four distinct
/// corners, four distinct edges, nothing sewn — an open patch, not a seamed
/// ring.
fn torus_patch(major: f64, minor: f64) -> (Model<StandardPayload>, FaceKey) {
    let mut g = Model::<StandardPayload>::new();
    let torus = Torus::new(Frame::xyz(), major, minor);
    let surface = Surface::Torus(torus.clone());
    let quarter = std::f64::consts::FRAC_PI_2;

    // The rectangle's corners in parameter space, in boundary order.
    let corners = [
        Point2::new(0.0, 0.0),
        Point2::new(quarter, 0.0),
        Point2::new(quarter, quarter),
        Point2::new(0.0, quarter),
    ];
    let sides = || corners.iter().zip(corners.iter().cycle().skip(1));

    let face = g
        .transaction(|edit| {
            let d: [Dart; 8] = std::array::from_fn(|_| edit.add_dart());
            for pair in 0..4 {
                edit.link(Dim::Zero, d[2 * pair], d[2 * pair + 1])?;
            }
            for pair in 0..4 {
                edit.link(Dim::One, d[2 * pair + 1], d[(2 * pair + 2) % 8])?;
            }

            for (pair, corner) in corners.iter().enumerate() {
                edit.add_vertex(VertexAttr::new(
                    d[2 * pair],
                    torus.point_at(corner.x, corner.y),
                ));
            }

            // A side at constant `v` runs along a circle about the main axis,
            // widened by the tube and lifted off the equatorial plane; a side at
            // constant `u` is the tube circle itself, in the plane the radial
            // direction at `u` and the main axis span.
            for (pair, (from, to)) in sides().enumerate() {
                let curve = if from.y == to.y {
                    Curve::Circle(Circle::new(
                        Plane::from_xy(
                            Point3::new(0.0, 0.0, minor * from.y.sin()),
                            Vector3::x(),
                            Vector3::y(),
                        ),
                        major + minor * from.y.cos(),
                    ))
                } else {
                    let (sin, cos) = from.x.sin_cos();
                    Curve::Circle(Circle::new(
                        Plane::from_xy(
                            Point3::new(major * cos, major * sin, 0.0),
                            Vector3::new(cos, sin, 0.0),
                            Vector3::z(),
                        ),
                        minor,
                    ))
                };
                edit.add_edge(EdgeAttr::new(d[2 * pair], curve));
            }
            edit.add_profile(ProfileAttr::new(d[0]));

            let pcurves = sides()
                .enumerate()
                .map(|(pair, (from, to))| (d[2 * pair], TrimmedCurve2::segment(*from, *to)))
                .collect::<HashMap<_, _>>();
            let face = edit.add_face(FaceAttr::with_pcurves(
                surface.clone(),
                d[0],
                Vec::new(),
                pcurves,
            ));
            Ok::<_, ModelEditError>(face)
        })
        .expect("an open torus patch should commit");
    (g, face)
}

/// A torus an outer loop bounds is meshed onto the tube, not flattened across it.
///
/// A whole torus is meshed by the boundaryless path, over the support's own
/// domain, and always has been. A patch takes the other path, which dispatches
/// on the surface — and a support left off that dispatch used to fall through to
/// a fallback that spanned the patch's four corners with a single flat quad. On
/// the `ppp0104` bracket that read as the fillet gone, and a plane cutting
/// through the solid where it should have been.
///
/// Refining is what tells the two apart. Every corner of the flat quad *does*
/// sit on the torus, so sitting on it is no evidence on its own; what a quad
/// cannot do is get closer when asked for more triangles. So the assertion is on
/// the chords: sample each triangle edge at its midpoint and require that
/// halving the step pulls those midpoints back onto the tube.
#[test]
fn a_torus_patch_is_meshed_onto_the_tube() {
    let (major, minor) = (3.0, 1.0);
    let (g, face) = torus_patch(major, minor);
    assert!(
        g.face(face)
            .expect("the patch's face resolves")
            .loops()
            .iter()
            .any(|boundary| !matches!(boundary.kind(), LoopKind::Inner)),
        "the patch is bounded from outside, so it takes the loop-bounded path"
    );

    // Distance from the tube of radius `minor` about the circle of radius
    // `major` in the plane `z = 0`.
    let off_tube =
        |point: &Point3| ((point.coords.xy().norm() - major).hypot(point.z) - minor).abs();
    let mesh_at = |surface: SurfaceOpts| {
        tessellate_face_key(
            &g,
            face,
            TessellateOpts {
                surface,
                ..TessellateOpts::default()
            },
        )
        .expect("a torus patch should tessellate")
    };
    let worst_chord = |surface: SurfaceOpts| {
        let mesh = mesh_at(surface);
        mesh.indices
            .chunks_exact(3)
            .flat_map(|triangle| {
                [(0, 1), (1, 2), (2, 0)].map(|(a, b)| {
                    off_tube(&nalgebra::center(
                        &mesh.positions[triangle[a] as usize],
                        &mesh.positions[triangle[b] as usize],
                    ))
                })
            })
            .fold(0.0f64, f64::max)
    };

    let coarse = SurfaceOpts { nu: 16, nv: 8 };
    assert!(
        mesh_at(coarse)
            .positions
            .iter()
            .all(|p| off_tube(p) <= 1e-9),
        "every mesh vertex sits on the tube"
    );

    let (coarse, fine) = (
        worst_chord(coarse),
        worst_chord(SurfaceOpts { nu: 32, nv: 16 }),
    );
    assert!(
        fine < 0.5 * coarse,
        "halving the step should pull the chords back onto the tube, \
         but they barely moved: {coarse} -> {fine}"
    );
}

/// A face stops where its boundary stops, even when that boundary is not a
/// rectangle.
///
/// A grid covers the whole parameter rectangle it is handed, so it is the right
/// answer only for a face whose boundary *is* that rectangle. `ppp0104`'s fillet
/// is the other kind: its boundary runs along `v = pi/2` only between
/// `u = +/-0.97`, then slants out to `+/-pi/2` by the time it reaches
/// `v = pi`. Meshed as a grid over the bounding rectangle, those two slanted
/// corners got filled in — surface hanging past the trim, which reads as a sheet
/// crossing the solid. The same went for the boss wall it meets, notched where
/// the plate joins it and meshed as a full cylinder.
///
/// The assertion is the property that was broken, not the triangle count that
/// happened to go with it: project each triangle back to parameter space and
/// require that it sits inside the face's own boundary.
#[test]
fn a_face_bounded_by_more_than_a_rectangle_is_meshed_only_inside_it() {
    let import = read_step(ANGLE_BRACKET, &StepReadOptions::default())
        .expect("the bracket fixture should import");
    let mut checked = 0;
    for shape in &import.shapes {
        let g = shape.model();
        for (key, _) in g.iter_faces() {
            let face = g.face(key).expect("a listed face resolves");
            let Ok(domain) = UnwrappedFaceDomain::of_face(&face) else {
                continue;
            };
            let boundary = domain.loops()[0].polyline(16);
            // Only the faces the bug was about: those a rectangle does not
            // describe. On the rest a grid is exact and there is nothing to say.
            if boundary_is_a_rectangle(&boundary) {
                continue;
            }
            let mesh = tessellate_face_key(g, key, TessellateOpts::default())
                .expect("the bracket's faces should tessellate");

            let strays = mesh
                .indices
                .chunks_exact(3)
                .filter(|triangle| {
                    let centroid = Point3::from(
                        triangle
                            .iter()
                            .map(|i| mesh.positions[*i as usize].coords)
                            .sum::<Vector3<f64>>()
                            / 3.0,
                    );
                    let Ok(uv) = face.surface().param_at(centroid) else {
                        return false;
                    };
                    // `param_at` answers in the support's own period, while the
                    // boundary is written in the unwrapped domain's, so the
                    // same point can be a whole turn away from its image here.
                    !(-1..=1).any(|turn| {
                        let turn = f64::from(turn) * std::f64::consts::TAU;
                        point_in_polygon(&boundary, Point2::new(uv.x + turn, uv.y))
                    })
                })
                .count();
            assert_eq!(
                strays,
                0,
                "{key:?} on a {:?} spills {strays} triangles past its own boundary",
                face.surface()
            );
            checked += 1;
        }
    }
    assert!(
        checked >= 2,
        "the bracket carries the trimmed fillet and the notched boss wall, \
         so at least two faces should have been checked, not {checked}"
    );
}

/// Whether the sampled boundary encloses the whole of its own bounding box.
fn boundary_is_a_rectangle(boundary: &[Point2]) -> bool {
    let (mut u_min, mut u_max) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut v_min, mut v_max) = (f64::INFINITY, f64::NEG_INFINITY);
    for point in boundary {
        u_min = u_min.min(point.x);
        u_max = u_max.max(point.x);
        v_min = v_min.min(point.y);
        v_max = v_max.max(point.y);
    }
    let box_area = (u_max - u_min) * (v_max - v_min);
    box_area > 0.0 && (signed_area(boundary).abs() - box_area).abs() / box_area <= 1e-6
}

fn signed_area(polygon: &[Point2]) -> f64 {
    let n = polygon.len();
    let mut area = 0.0;
    for i in 0..n {
        let (p, q) = (polygon[i], polygon[(i + 1) % n]);
        area += p.x * q.y - q.x * p.y;
    }
    0.5 * area
}

fn point_in_polygon(polygon: &[Point2], point: Point2) -> bool {
    let mut inside = false;
    let n = polygon.len();
    for i in 0..n {
        let (a, b) = (polygon[i], polygon[(i + 1) % n]);
        if (a.y > point.y) != (b.y > point.y)
            && point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x
        {
            inside = !inside;
        }
    }
    inside
}
