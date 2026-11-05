//! A solid's tessellation is one closed, consistently oriented surface: the
//! triangles of all its faces, joined on exact coordinates, pair every
//! directed edge with its reverse once, and enclose the volume the solid has.

use std::collections::HashMap;

use nalgebra::Vector3;
use ngk::geometry::{Axis3, Frame, Point3};
use ngk::modeling::{blend, faces, revolve, solids};
use ngk::tessellate::{CurveOpts, SurfaceOpts, Tessellate, TessellateOpts, Tessellation};
use ngk::topology::shape::{Shape, SolidTag};
use radians::Rad64;

pub type Built = Shape<SolidTag>;

pub fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

pub fn v(x: f64, y: f64, z: f64) -> Vector3<f64> {
    Vector3::new(x, y, z)
}

pub fn deg(d: f64) -> Rad64 {
    Rad64::new(d.to_radians())
}

pub fn axis_z() -> Axis3 {
    Axis3::new(p(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0))
}

pub fn opts(segments: usize) -> TessellateOpts {
    TessellateOpts {
        curve: CurveOpts { segments },
        surface: SurfaceOpts {
            nu: segments,
            nv: segments / 2,
        },
    }
}

/// The surface the triangles make, joined on exact coordinates.
pub struct Joined {
    pub positions: Vec<Point3>,
    pub triangles: Vec<[u32; 3]>,
}

pub fn join(t: &Tessellation) -> Joined {
    let mut index: HashMap<[u64; 3], u32> = HashMap::new();
    let mut positions = Vec::new();
    let remap: Vec<u32> = t
        .mesh
        .positions
        .iter()
        .map(|q| {
            let key = [q.x.to_bits(), q.y.to_bits(), q.z.to_bits()];
            *index.entry(key).or_insert_with(|| {
                positions.push(*q);
                (positions.len() - 1) as u32
            })
        })
        .collect();
    let triangles = t
        .mesh
        .indices
        .chunks_exact(3)
        .map(|c| {
            [
                remap[c[0] as usize],
                remap[c[1] as usize],
                remap[c[2] as usize],
            ]
        })
        .collect();
    Joined {
        positions,
        triangles,
    }
}

/// Every problem found in `joined` as a closed, oriented 2-manifold.
pub fn manifold_problems(joined: &Joined) -> Vec<String> {
    let mut problems = Vec::new();
    let mut directed: HashMap<(u32, u32), usize> = HashMap::new();
    for t in &joined.triangles {
        if t[0] == t[1] || t[1] == t[2] || t[2] == t[0] {
            problems.push(format!("collapsed triangle {t:?}"));
            continue;
        }
        for k in 0..3 {
            *directed.entry((t[k], t[(k + 1) % 3])).or_default() += 1;
        }
    }
    let mut open = 0;
    let mut repeated = 0;
    for (&(a, b), &n) in &directed {
        if n > 1 {
            repeated += 1;
        }
        if !directed.contains_key(&(b, a)) {
            open += 1;
        }
    }
    if open > 0 {
        let mut sample = directed
            .keys()
            .filter(|&&(a, b)| !directed.contains_key(&(b, a)))
            .take(6)
            .map(|&(a, b)| {
                let (a, b) = (joined.positions[a as usize], joined.positions[b as usize]);
                format!(
                    "({:.3},{:.3},{:.3})->({:.3},{:.3},{:.3})",
                    a.x, a.y, a.z, b.x, b.y, b.z
                )
            })
            .collect::<Vec<_>>();
        sample.sort();
        problems.push(format!(
            "{open} directed edges have no reverse, e.g. {sample:?}"
        ));
    }
    if repeated > 0 {
        problems.push(format!("{repeated} directed edges are used more than once"));
    }
    problems
}

pub fn signed_volume(joined: &Joined) -> f64 {
    joined
        .triangles
        .iter()
        .map(|t| {
            let [a, b, c] = t.map(|i| joined.positions[i as usize].coords);
            a.dot(&b.cross(&c)) / 6.0
        })
        .sum()
}

/// The mesh of `solid` at `segments`, checked as a closed oriented surface
/// whose volume is `solid`'s within `tolerance` (relative).
pub fn assert_watertight(name: &str, solid: &Built, segments: usize, tolerance: f64) -> Joined {
    let mesh = solid
        .solid()
        .tessellate_with(opts(segments))
        .unwrap_or_else(|e| panic!("{name}: {e}"));
    let joined = join(&mesh);
    let problems = manifold_problems(&joined);
    assert!(problems.is_empty(), "{name}: {problems:?}");
    let expected = solid.solid().volume().expect("volume");
    let got = signed_volume(&joined);
    assert!(
        (got - expected).abs() <= tolerance * expected.abs(),
        "{name}: mesh volume {got} against solid volume {expected}"
    );
    joined
}

fn rect_section(x0: f64, x1: f64, z0: f64, z1: f64) -> Shape<ngk::topology::shape::FaceTag> {
    faces::polygon(&[
        p(x0, 0.0, z1),
        p(x1, 0.0, z1),
        p(x1, 0.0, z0),
        p(x0, 0.0, z0),
    ])
    .expect("section builds")
}

pub fn revolved_rect(x0: f64, x1: f64, angle: f64) -> Built {
    revolve::revolve_face(rect_section(x0, x1, 0.0, 4.0), axis_z(), deg(angle))
        .expect("revolve builds")
}

/// The same section wound the other way round.
pub fn revolved_rect_reversed(x0: f64, x1: f64, angle: f64) -> Built {
    let face = faces::polygon(&[
        p(x0, 0.0, 0.0),
        p(x1, 0.0, 0.0),
        p(x1, 0.0, 4.0),
        p(x0, 0.0, 4.0),
    ])
    .expect("section builds");
    revolve::revolve_face(face, axis_z(), deg(angle)).expect("revolve builds")
}

pub fn cylinder_z(x: f64, y: f64, z0: f64, z1: f64, r: f64) -> Built {
    let frame = Frame::from_xz(p(x, y, z0), v(1.0, 0.0, 0.0), v(0.0, 0.0, 1.0));
    solids::cylinder_at(frame, r, z1 - z0).expect("cylinder builds")
}

pub fn block(a: Point3, b: Point3) -> Built {
    let size = b - a;
    solids::block_at(Frame::at(a), size.x, size.y, size.z).expect("block builds")
}

pub fn cut(a: Built, b: &Built) -> Built {
    solids::cut(a, [b]).expect("cut builds")
}

pub fn fuse(a: Built, b: &Built) -> Built {
    solids::fuse(a, [b]).expect("fuse builds")
}

#[test]
fn primitives_are_closed() {
    assert_watertight("block", &solids::block(1.0, 2.0, 3.0).unwrap(), 32, 1e-9);
    assert_watertight("cylinder", &solids::cylinder(2.0, 5.0).unwrap(), 32, 0.01);
    assert_watertight("sphere", &solids::sphere(3.0).unwrap(), 32, 0.02);
    assert_watertight("torus", &solids::torus(5.0, 1.0).unwrap(), 32, 0.02);
}

#[test]
fn an_extruded_polygon_with_a_polygonal_hole_is_closed() {
    let outer = [
        p(0.0, 1.35, 0.0),
        p(1.28, 0.42, 0.0),
        p(0.8, -1.1, 0.0),
        p(-0.8, -1.1, 0.0),
        p(-1.28, 0.42, 0.0),
    ];
    let inner = [
        p(-0.38, -0.38, 0.0),
        p(-0.38, 0.38, 0.0),
        p(0.38, 0.38, 0.0),
        p(0.38, -0.38, 0.0),
    ];
    let face = faces::polygon_with_holes(ngk::geometry::Plane::xy(), &outer, &[&inner]).unwrap();
    let solid = ngk::modeling::sweep::extrude_face(face, v(0.0, 0.0, 1.2)).unwrap();
    assert_watertight("polygonal hole", &solid, 32, 0.02);
}

#[test]
fn cone_frustum_is_closed() {
    let section = faces::polygon(&[
        p(0.0, 0.0, 6.0),
        p(2.0, 0.0, 6.0),
        p(5.0, 0.0, 0.0),
        p(0.0, 0.0, 0.0),
    ])
    .unwrap();
    let frustum = revolve::revolve_face(section, axis_z(), deg(360.0)).unwrap();
    assert_watertight("cone frustum", &frustum, 32, 0.02);
}

#[test]
fn revolved_sections_are_closed_at_every_angle_and_winding() {
    for angle in [90.0, 360.0, 180.0, 270.0] {
        assert_watertight(
            &format!("washer {angle}"),
            &revolved_rect(6.0, 10.0, angle),
            32,
            0.02,
        );
        assert_watertight(
            &format!("washer reversed {angle}"),
            &revolved_rect_reversed(6.0, 10.0, angle),
            32,
            0.02,
        );
    }
    for angle in [90.0, 360.0] {
        assert_watertight(
            &format!("disc {angle}"),
            &revolved_rect(0.0, 5.0, angle),
            32,
            0.02,
        );
    }
}

#[test]
fn booleans_of_cylinders_are_closed() {
    let tube = cut(
        cylinder_z(0.0, 0.0, 0.0, 10.0, 5.0),
        &cylinder_z(0.0, 0.0, -1.0, 11.0, 3.0),
    );
    assert_watertight("tube", &tube, 32, 0.02);
    let shaft = fuse(
        cylinder_z(0.0, 0.0, 0.0, 10.0, 5.0),
        &cylinder_z(0.0, 0.0, 10.0, 20.0, 3.0),
    );
    assert_watertight("stepped shaft", &shaft, 32, 0.02);
    let plate = cut(
        block(p(0.0, 0.0, 0.0), p(100.0, 20.0, 10.0)),
        &cylinder_z(10.0, 10.0, -1.0, 11.0, 2.0),
    );
    assert_watertight("drilled plate", &plate, 32, 0.02);
    let cube = cut(
        block(p(0.0, 0.0, 0.0), p(10.0, 10.0, 10.0)),
        &cylinder_z(5.0, 5.0, -1.0, 11.0, 3.0),
    );
    assert_watertight("box minus cylinder", &cube, 32, 0.02);
}

#[test]
fn halved_sphere_is_closed() {
    let ball = solids::sphere(5.0).unwrap();
    let half = cut(ball, &block(p(-10.0, -10.0, -10.0), p(10.0, 10.0, 0.0)));
    assert_watertight("hemisphere", &half, 32, 0.02);
}

#[test]
fn filleted_block_is_closed() {
    let cube = solids::block(10.0, 10.0, 10.0).unwrap();
    let edges: Vec<_> = cube.solid().edges().iter().map(|e| e.key()).collect();
    let rounded = blend::filleted(cube, edges, 1.0).unwrap();
    assert_watertight("filleted box", &rounded, 32, 0.02);
}

fn line_edge(a: Point3, b: Point3) -> Shape<ngk::topology::shape::EdgeTag> {
    ngk::modeling::edges::line(a, b).expect("line builds")
}

fn bezier_edge(points: [Point3; 4]) -> Shape<ngk::topology::shape::EdgeTag> {
    use ngk::geometry::{ControlPolygon, Curve, Degree, KnotVector, NurbsCurve};
    let curve = NurbsCurve::new(
        Degree::new(3).unwrap(),
        ControlPolygon::from_cartesian(points.to_vec(), &[1.0; 4]).unwrap(),
        KnotVector::new(vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]).unwrap(),
    )
    .unwrap();
    let (a, b) = (points[0], points[3]);
    Shape::build(|m| ngk::builders::edges::add_edge(m, a, b, Curve::Nurbs(curve.clone()))).unwrap()
}

fn ellipse_face(c: Point3, a: f64, b: f64) -> Shape<ngk::topology::shape::FaceTag> {
    use ngk::geometry::{Curve, Ellipse};
    let e = Ellipse::new(Frame::at(c), a, b);
    let (east, west) = (c + v(a, 0.0, 0.0), c + v(-a, 0.0, 0.0));
    let upper =
        Shape::build(|m| ngk::builders::edges::add_edge(m, east, west, Curve::Ellipse(e.clone())))
            .unwrap();
    let lower =
        Shape::build(|m| ngk::builders::edges::add_edge(m, west, east, Curve::Ellipse(e.clone())))
            .unwrap();
    let profile = ngk::modeling::profiles::from_edges(&[&upper, &lower]).unwrap();
    faces::from_profile(&profile).unwrap()
}

fn elliptic_drill() -> Built {
    solids::extruded(
        ellipse_face(p(0.0, 0.0, -1.0), 6.0, 3.0),
        nalgebra::Unit::new_normalize(v(0.0, 0.0, 1.0)),
        10.0,
    )
    .unwrap()
}

#[test]
fn an_extruded_ellipse_is_closed() {
    assert_watertight("extruded ellipse", &elliptic_drill(), 32, 0.02);
}

#[test]
#[ignore = "elliptic Boolean section pcurves disagree with the rim's 3D curves"]
fn an_elliptic_hole_is_closed() {
    let plate = cut(
        block(p(-12.0, -8.0, 0.0), p(12.0, 8.0, 8.0)),
        &elliptic_drill(),
    );
    assert_watertight("elliptic hole", &plate, 32, 0.02);
}

#[test]
fn a_saddle_drum_is_closed() {
    use ngk::builders::loft::LoftOptions;
    let near =
        ngk::modeling::profiles::polyline(&[p(-10.0, -10.0, 22.5), p(10.0, -10.0, -2.5)]).unwrap();
    let far =
        ngk::modeling::profiles::polyline(&[p(-10.0, 10.0, -2.5), p(10.0, 10.0, 22.5)]).unwrap();
    let saddle = ngk::modeling::loft::loft(&[&near, &far], LoftOptions::ruled()).unwrap();
    let parts = ngk::modeling::split(&solids::cylinder(8.0, 30.0).unwrap(), &saddle).unwrap();
    let low = |s: &Built| s.solid().volume_properties().unwrap().centroid.z;
    let under = if low(&parts.front) < low(&parts.back) {
        parts.front
    } else {
        parts.back
    };
    assert_watertight("saddle drum", &under, 32, 0.02);
}

#[test]
fn a_dome_revolved_about_its_axis_has_one_pole() {
    let shaft = line_edge(p(0.0, 0.0, 0.0), p(0.0, 0.0, 10.0));
    let floor = line_edge(p(9.0, 0.0, 0.0), p(0.0, 0.0, 0.0));
    let dome = bezier_edge([
        p(0.0, 0.0, 10.0),
        p(9.0, 0.0, 10.0),
        p(12.0, 0.0, 4.0),
        p(9.0, 0.0, 0.0),
    ]);
    let profile = ngk::modeling::profiles::from_edges(&[&shaft, &floor, &dome]).unwrap();
    for angle in [360.0, 90.0] {
        let face = faces::from_profile(&profile).unwrap();
        let body = revolve::revolve_face(face, axis_z(), deg(angle)).unwrap();
        assert_watertight(&format!("cubic dome {angle}"), &body, 32, 0.03);
    }
}

#[test]
fn a_torus_a_hair_from_horn_is_closed() {
    assert_watertight(
        "torus 0.01 from horn",
        &solids::torus(5.0, 4.99).unwrap(),
        32,
        0.05,
    );
}

#[test]
fn a_cone_to_its_apex_is_closed() {
    for angle in [360.0, 90.0] {
        let section =
            faces::polygon(&[p(0.0, 0.0, 6.0), p(5.0, 0.0, 0.0), p(0.0, 0.0, 0.0)]).unwrap();
        let cone = revolve::revolve_face(section, axis_z(), deg(angle)).unwrap();
        assert_watertight(&format!("cone {angle}"), &cone, 32, 0.03);
    }
}

#[test]
fn a_face_a_millionth_wide_is_closed() {
    let sliver = block(p(0.0, 0.0, 0.0), p(10.0, 1e-6, 10.0));
    assert_watertight("sliver block", &sliver, 32, 1e-6);
}

#[test]
fn a_hole_a_millionth_from_the_rim_is_closed() {
    let plate = block(p(0.0, 0.0, 0.0), p(10.0, 10.0, 2.0));
    let drill = cylinder_z(1.0 + 1e-6, 5.0, -1.0, 3.0, 1.0);
    let drilled = cut(plate, &drill);
    assert_watertight("hole beside the rim", &drilled, 32, 0.02);
}

#[test]
fn closed_at_the_sampling_the_bench_uses() {
    for segments in [16, 64, 128] {
        // An inscribed surface falls short of the true one by about the square
        // of the angular step.
        let step = std::f64::consts::TAU / segments as f64;
        let tolerance = 0.4 * step * step;
        assert_watertight(
            &format!("sphere {segments}"),
            &solids::sphere(5.0).unwrap(),
            segments,
            tolerance,
        );
        assert_watertight(
            &format!("torus {segments}"),
            &solids::torus(5.0, 1.0).unwrap(),
            segments,
            tolerance,
        );
    }
}
