//! Small helpers the case implementations share: points, frames, angles, and picking edges and
//! faces of a solid by a point they pass through, the way an NGK user selects by key.
use crate::{Refusal, Solid, refuse};
use nalgebra::Vector3;
use ngk::geometry::{Axis3, Frame, Plane, Point3, SurfaceGeometry};
use ngk::topology::shape_keys::{EdgeKey, FaceKey};
use radians::Rad64;

pub fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

pub fn v(x: f64, y: f64, z: f64) -> Vector3<f64> {
    Vector3::new(x, y, z)
}

/// An angle given in degrees.
pub fn deg(d: f64) -> Rad64 {
    Rad64::new(d.to_radians())
}

/// A frame at `origin` whose z axis runs along `z`, its x axis any direction square to it.
pub fn frame(origin: Point3, z: Vector3<f64>) -> Frame {
    let z = z.normalize();
    let helper = if z.x.abs() < 0.9 { v(1.0, 0.0, 0.0) } else { v(0.0, 1.0, 0.0) };
    let x = (helper - z * helper.dot(&z)).normalize();
    Frame::from_xz(origin, x, z)
}

/// The plane through `origin` square to `normal`.
pub fn plane(origin: Point3, normal: Vector3<f64>) -> Plane {
    Plane::from_frame(frame(origin, normal))
}

/// The plane through `origin` spanned by `x` and `y`.
pub fn plane_xy(origin: Point3, x: Vector3<f64>, y: Vector3<f64>) -> Plane {
    Plane::from_xy(origin, x, y)
}

pub fn axis(origin: Point3, direction: Vector3<f64>) -> Axis3 {
    Axis3::new(origin, direction)
}

/// Every edge of the solid.
pub fn edges(s: &Solid) -> Vec<EdgeKey> {
    s.solid().edges().iter().map(|e| e.key()).collect()
}

/// The edge of the solid that passes through `at`.
pub fn edge_through(s: &Solid, at: Point3) -> Result<EdgeKey, Refusal> {
    let found: Vec<EdgeKey> = s
        .solid()
        .edges()
        .iter()
        .filter(|e| e.trimmed_curve().contains(at, 1e-6))
        .map(|e| e.key())
        .collect();
    match found.as_slice() {
        [one] => Ok(*one),
        [] => refuse(&format!("no edge passes through ({}, {}, {})", at.x, at.y, at.z)),
        _ => refuse(&format!("several edges pass through ({}, {}, {})", at.x, at.y, at.z)),
    }
}

/// The edges of the solid through each of the points.
pub fn edges_through(s: &Solid, at: &[Point3]) -> Result<Vec<EdgeKey>, Refusal> {
    at.iter().map(|&a| edge_through(s, a)).collect()
}

/// The face of the solid whose surface passes through `at`, inside its bounding box.
pub fn face_through(s: &Solid, at: Point3) -> Result<FaceKey, Refusal> {
    let mut found = Vec::new();
    for f in s.solid().faces() {
        let surface = f.surface();
        let Ok(uv) = surface.closest_parameter(at) else { continue };
        if (surface.point_at(uv.x, uv.y) - at).norm() > 1e-6 {
            continue;
        }
        let Ok(bb) = f.bounding_box() else { continue };
        if bb.contains_point(at, 1e-6) {
            found.push(f.key());
        }
    }
    found.sort();
    found.dedup();
    match found.as_slice() {
        [one] => Ok(*one),
        [] => refuse(&format!("no face passes through ({}, {}, {})", at.x, at.y, at.z)),
        _ => refuse(&format!("several faces pass through ({}, {}, {})", at.x, at.y, at.z)),
    }
}

/// The block between the corners `a` and `b`, its sides along the axes.
pub fn block(a: Point3, b: Point3) -> Result<Solid, Refusal> {
    let lo = p(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z));
    let size = (b - a).abs();
    Ok(ngk::modeling::solids::block_at(Frame::at(lo), size.x, size.y, size.z)?)
}

/// The cylinder of radius `r` whose axis runs from `base` along `dir` for `h`.
pub fn cylinder(base: Point3, dir: Vector3<f64>, r: f64, h: f64) -> Result<Solid, Refusal> {
    Ok(ngk::modeling::solids::cylinder_at(frame(base, dir), r, h)?)
}

/// The cylinder of radius `r` on the vertical through (x, y), from z0 to z1.
pub fn cylinder_z(x: f64, y: f64, z0: f64, z1: f64, r: f64) -> Result<Solid, Refusal> {
    cylinder(p(x, y, z0), v(0.0, 0.0, 1.0), r, z1 - z0)
}

/// The ball of radius `r` about `c`, its poles on the vertical through `c`.
pub fn ball(c: Point3, r: f64) -> Result<Solid, Refusal> {
    Ok(ngk::modeling::solids::sphere_at(Frame::at(c), r)?)
}

/// A Boolean's refusal, with what its intersection coverage says when it is incomplete.
fn boolean_refusal(e: ngk::builders::boolean::BooleanError) -> Refusal {
    use ngk::builders::boolean::BooleanError;
    let detail = match &e {
        BooleanError::Imprint { source, .. } => match source.as_ref() {
            BooleanError::IncompleteIntersections { diagnostics } => Some(format!(
                " ({:?}; {} unresolved overlaps, {} uncertified branches)",
                diagnostics.coverage,
                diagnostics.unresolved_overlaps.len(),
                diagnostics.branches_uncertified
            )),
            _ => None,
        },
        _ => None,
    };
    let Refusal(why) = Refusal::from(e);
    Refusal(why + &detail.unwrap_or_default())
}

pub fn cut(a: Solid, b: &Solid) -> Result<Solid, Refusal> {
    ngk::modeling::solids::cut(a, [b]).map_err(boolean_refusal)
}

pub fn fuse(a: Solid, b: &Solid) -> Result<Solid, Refusal> {
    ngk::modeling::solids::fuse(a, [b]).map_err(boolean_refusal)
}

pub fn common(a: Solid, b: &Solid) -> Result<Solid, Refusal> {
    ngk::modeling::solids::intersect(a, [b]).map_err(boolean_refusal)
}

/// `a` less each tool, one cut after another.
pub fn cut_each(mut a: Solid, tools: &[Solid]) -> Result<Solid, Refusal> {
    for t in tools {
        a = cut(a, t)?;
    }
    Ok(a)
}

/// The solid rounded at the edges through each point, to radius `r`.
pub fn fillet_through(s: Solid, at: &[Point3], r: f64) -> Result<Solid, Refusal> {
    let keys = edges_through(&s, at)?;
    Ok(ngk::modeling::blend::filleted(s, keys, r)?)
}

/// The solid chamfered at the edges through each point, `d` back on both faces.
pub fn chamfer_through(s: Solid, at: &[Point3], d: f64) -> Result<Solid, Refusal> {
    let keys = edges_through(&s, at)?;
    Ok(ngk::modeling::blend::chamfered(s, keys, d)?)
}

/// The prism of the polygon `outline` extruded along `dir` by its length.
pub fn prism(outline: &[Point3], dir: Vector3<f64>) -> Result<Solid, Refusal> {
    let face = ngk::modeling::faces::polygon(outline)?;
    let length = dir.norm();
    Ok(ngk::modeling::solids::extruded(face, nalgebra::Unit::new_normalize(dir), length)?)
}

/// The solid bounded by the planar polygons, each wound either way: a polyhedron.
pub fn polyhedron(polygons: &[Vec<Point3>]) -> Result<Solid, Refusal> {
    let faces = polygons
        .iter()
        .map(|corners| ngk::modeling::faces::polygon(corners))
        .collect::<Result<Vec<_>, _>>()?;
    let refs: Vec<_> = faces.iter().collect();
    let sheet = ngk::modeling::sheets::from_faces(&refs)?;
    Ok(ngk::modeling::solids::from_sheets(&sheet, &[])?)
}

/// The solid of revolution of the polygon `section` (in a plane through the axis), turned
/// `angle` degrees about the axis through `origin` along `dir`.
pub fn revolved(section: &[Point3], origin: Point3, dir: Vector3<f64>, angle: f64) -> Result<Solid, Refusal> {
    let face = ngk::modeling::faces::polygon(section)?;
    Ok(ngk::modeling::revolve::revolve_face(face, axis(origin, dir), deg(angle))?)
}

/// The octahedron |x| + |y| + |z| ≤ r, from its eight triangles.
pub fn octahedron(r: f64) -> Result<Solid, Refusal> {
    let mut faces = Vec::new();
    for sx in [-1.0, 1.0] {
        for sy in [-1.0, 1.0] {
            for sz in [-1.0, 1.0] {
                let (a, b, c) = (p(sx * r, 0.0, 0.0), p(0.0, sy * r, 0.0), p(0.0, 0.0, sz * r));
                // Wound counter-clockwise seen from outside.
                faces.push(if sx * sy * sz > 0.0 { vec![a, b, c] } else { vec![a, c, b] });
            }
        }
    }
    polyhedron(&faces)
}

/// The disc of radius `r` about `c`, square to `normal`.
pub fn disc(c: Point3, normal: Vector3<f64>, r: f64) -> Result<ngk::topology::shape::Shape<ngk::topology::shape::FaceTag>, Refusal> {
    Ok(ngk::modeling::faces::circle(plane(c, normal), r)?)
}

/// `face` swept along the polyline through `path`, sharp corners treated by `transition`.
pub fn sweep_along_polyline(
    face: ngk::topology::shape::Shape<ngk::topology::shape::FaceTag>,
    path: &[Point3],
    transition: ngk::builders::sweep::SweepTransition,
) -> Result<Solid, Refusal> {
    let spine = ngk::modeling::profiles::polyline(path)?;
    let options = ngk::builders::sweep::SweepOptions { transition, ..Default::default() };
    Ok(ngk::modeling::sweep::sweep_face(face, &spine.profile(), options)?)
}

/// The solid cone or frustum on the axis through `base` along `dir`: radius `r0` at `base`,
/// `r1` at `h` along the axis (either may be 0), turned from its half section a full turn.
pub fn cone(base: Point3, dir: Vector3<f64>, r0: f64, r1: f64, h: f64) -> Result<Solid, Refusal> {
    let f = frame(base, dir);
    let at = |x: f64, z: f64| f.point_at(v(x, 0.0, z));
    let mut section = vec![at(0.0, 0.0)];
    if r0 > 0.0 {
        section.push(at(r0, 0.0));
    }
    if r1 > 0.0 {
        section.push(at(r1, h));
    }
    section.push(at(0.0, h));
    // Wound this way, NGK's tessellator turns a whole-turn face's triangles outward; the solid
    // itself is the same either way.
    section.reverse();
    revolved(&section, base, dir, 360.0)
}

/// A frame at `origin` with its z axis along `z` and its x axis along `x`.
pub fn frame_zx(origin: Point3, z: Vector3<f64>, x: Vector3<f64>) -> Frame {
    Frame::from_xz(origin, x, z)
}

pub type Edge = ngk::topology::shape::Shape<ngk::topology::shape::EdgeTag>;
pub type Face = ngk::topology::shape::Shape<ngk::topology::shape::FaceTag>;

/// The NURBS edge of degree `degree` on the control points with their weights and knots.
pub fn nurbs_edge(degree: usize, points: &[Point3], weights: &[f64], knots: &[f64]) -> Result<Edge, Refusal> {
    use ngk::geometry::{ControlPolygon, Degree, KnotVector, NurbsCurve};
    let curve = NurbsCurve::new(
        Degree::new(degree)?,
        ControlPolygon::from_cartesian(points.to_vec(), weights)?,
        KnotVector::new(knots.to_vec())?,
    )?;
    nurbs_curve_edge(curve)
}

/// The edge along the whole of a NURBS curve.
pub fn nurbs_curve_edge(curve: ngk::geometry::NurbsCurve) -> Result<Edge, Refusal> {
    let d = curve.domain();
    let (a, b) = (curve.point_at(d.start.value()), curve.point_at(d.end.value()));
    Ok(ngk::topology::shape::Shape::build(|m| {
        ngk::builders::edges::add_edge(m, a, b, ngk::geometry::Curve::Nurbs(curve))
    })?)
}

/// The cubic Bézier edge on four control points.
pub fn bezier_edge(points: [Point3; 4]) -> Result<Edge, Refusal> {
    nurbs_edge(3, &points, &[1.0; 4], &[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0])
}

pub fn line(a: Point3, b: Point3) -> Result<Edge, Refusal> {
    Ok(ngk::modeling::edges::line(a, b)?)
}

/// The planar face bounded by the edges, joined into one closed profile.
pub fn face_from_edges(edges: &[&Edge]) -> Result<Face, Refusal> {
    let profile = ngk::modeling::profiles::from_edges(edges)?;
    Ok(ngk::modeling::faces::from_profile(&profile)?)
}

/// `face` turned a full turn about the z axis.
pub fn revolve_about_z(face: Face, angle: f64) -> Result<Solid, Refusal> {
    Ok(ngk::modeling::revolve::revolve_face(face, axis(p(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)), deg(angle))?)
}

/// The edge along the curve from `a` to `b`.
pub fn curve_edge(a: Point3, b: Point3, curve: ngk::geometry::Curve) -> Result<Edge, Refusal> {
    Ok(ngk::topology::shape::Shape::build(|m| ngk::builders::edges::add_edge(m, a, b, curve))?)
}

/// The rational quadratic Bézier edge from `a` over `b` (weight `w`) to `c`: a conic arc.
pub fn conic_edge(a: Point3, b: Point3, w: f64, c: Point3) -> Result<Edge, Refusal> {
    nurbs_edge(2, &[a, b, c], &[1.0, w, 1.0], &[0.0, 0.0, 0.0, 1.0, 1.0, 1.0])
}

/// The closed uniform periodic B-spline of degree `degree` on the control points, as the edges
/// of its two halves (NGK's edges join two distinct corners or none).
pub fn periodic_bspline_edges(degree: usize, points: &[Point3]) -> Result<[Edge; 2], Refusal> {
    use ngk::geometry::{ControlPolygon, Degree, KnotVector, NurbsCurve};
    let n = points.len();
    let mut wrapped = points.to_vec();
    wrapped.extend_from_slice(&points[..degree]);
    let knots: Vec<f64> = (0..wrapped.len() + degree + 1).map(|i| i as f64).collect();
    let curve = NurbsCurve::new(
        Degree::new(degree)?,
        ControlPolygon::from_cartesian(wrapped.clone(), &vec![1.0; wrapped.len()])?,
        KnotVector::new(knots)?,
    )?;
    let (lo, hi) = (degree as f64, (degree + n) as f64);
    let curve = curve.trimmed(lo, hi)?;
    let (first, second) = curve.split_at((lo + hi) / 2.0)?;
    Ok([nurbs_curve_edge(first)?, nurbs_curve_edge(second)?])
}

/// A planar face extruded along `dir` by its length.
pub fn extrude(face: Face, dir: Vector3<f64>) -> Result<Solid, Refusal> {
    let length = dir.norm();
    Ok(ngk::modeling::solids::extruded(face, nalgebra::Unit::new_normalize(dir), length)?)
}

/// The face inside the ellipse of semi-axes `a` along x and `b` along y about `c`, square to z.
pub fn ellipse_face(c: Point3, a: f64, b: f64) -> Result<Face, Refusal> {
    use ngk::geometry::{Curve, Ellipse};
    let e = Ellipse::new(Frame::at(c), a, b);
    let (east, west) = (c + v(a, 0.0, 0.0), c + v(-a, 0.0, 0.0));
    let upper = curve_edge(east, west, Curve::Ellipse(e.clone()))?;
    let lower = curve_edge(west, east, Curve::Ellipse(e))?;
    face_from_edges(&[&upper, &lower])
}

/// The stadium in the plane z = `z`: the rectangle |x − cx| ≤ `half`, |y − cy| ≤ `r` with a half
/// disc of radius `r` on each short side.
pub fn stadium(cx: f64, cy: f64, z: f64, half: f64, r: f64) -> Result<Face, Refusal> {
    let xy = |x: f64, y: f64| p(cx + x, cy + y, z);
    let east = ngk::modeling::edges::arc(plane_xy(xy(half, 0.0), v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0)), r, deg(-90.0), deg(90.0))?;
    let west = ngk::modeling::edges::arc(plane_xy(xy(-half, 0.0), v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0)), r, deg(90.0), deg(270.0))?;
    let north = line(xy(half, r), xy(-half, r))?;
    let south = line(xy(-half, -r), xy(half, -r))?;
    face_from_edges(&[&east, &north, &west, &south])
}
