//! What the reproduction suite in `tests/robustness/` shares: terse constructors for the solids
//! the reproductions are made of, outcome helpers that turn a panic into a refusal, closed forms
//! for the volumes the answers are checked against, and the check every tessellation must pass.

#![allow(dead_code)]

use std::collections::HashMap;
use std::f64::consts::PI;
use std::panic::{AssertUnwindSafe, catch_unwind};

use nalgebra::{Unit, Vector3};
use ngk::geometry::{Axis3, Frame, Plane, Point3};
use ngk::modeling::{faces, solids};
use ngk::tessellate::{Tessellate, TessellateOpts, tessellate};
use ngk::topology::shape::{FaceTag, Shape, SolidTag};
use ngk::topology::validation::{validate_solid_manifold, validate_solid_orientation};
use radians::Rad64;

pub type Solid = Shape<SolidTag>;
pub type Face = Shape<FaceTag>;

/// Why an operation gave no solid: the error and every error it wraps, outermost first, or the
/// message of a panic, which starts with `panicked:`.
#[derive(Debug)]
pub struct Refusal(pub String);

impl<E: std::error::Error> From<E> for Refusal {
    fn from(e: E) -> Self {
        let mut why = e.to_string();
        let mut source = e.source();
        while let Some(inner) = source {
            why.push_str(": ");
            why.push_str(&inner.to_string());
            source = inner.source();
        }
        Refusal(why)
    }
}

impl Refusal {
    pub fn panicked(&self) -> bool {
        self.0.starts_with("panicked:")
    }
}

/// `f`'s result, or a refusal naming the panic when it panics, so one reproduction that panics
/// reports itself without taking its siblings with it.
pub fn attempt<T>(f: impl FnOnce() -> Result<T, Refusal>) -> Result<T, Refusal> {
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(result) => result,
        Err(payload) => {
            let message = payload
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| payload.downcast_ref::<&str>().map(|s| (*s).to_owned()))
                .unwrap_or_else(|| "no message".to_owned());
            Err(Refusal(format!("panicked: {message}")))
        }
    }
}

/// How long a reproduction may run before it is reported as not finishing, in seconds: the
/// environment variable `ROBUSTNESS_TIMEOUT`, or 120.
fn time_limit() -> u64 {
    std::env::var("ROBUSTNESS_TIMEOUT")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(120)
}

/// Runs a reproduction on a thread of its own, with a stack deep enough for the kernel's
/// recursion, and reports a panic or a run past the time limit as a refusal. A thread that is
/// still running at the limit is left behind and ends with the test binary.
pub fn guarded(f: impl FnOnce() -> Result<(), Refusal> + Send + 'static) -> Result<(), Refusal> {
    let (reply, wait) = std::sync::mpsc::channel();
    std::thread::Builder::new()
        .stack_size(256 << 20)
        .spawn(move || {
            let _ = reply.send(attempt(f));
        })
        .map_err(|e| Refusal(format!("no thread: {e}")))?;
    let limit = time_limit();
    wait.recv_timeout(std::time::Duration::from_secs(limit))
        .unwrap_or_else(|_| Err(Refusal(format!("did not finish within {limit} s"))))
}

pub fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

pub fn v(x: f64, y: f64, z: f64) -> Vector3<f64> {
    Vector3::new(x, y, z)
}

pub fn deg(d: f64) -> Rad64 {
    Rad64::new(d.to_radians())
}

/// A frame at `origin` whose z axis runs along `z`.
pub fn frame(origin: Point3, z: Vector3<f64>) -> Frame {
    let z = z.normalize();
    let helper = if z.x.abs() < 0.9 {
        v(1.0, 0.0, 0.0)
    } else {
        v(0.0, 1.0, 0.0)
    };
    let x = (helper - z * helper.dot(&z)).normalize();
    Frame::from_xz(origin, x, z)
}

/// The plane through `origin` square to `normal`.
pub fn plane(origin: Point3, normal: Vector3<f64>) -> Plane {
    Plane::from_frame(frame(origin, normal))
}

pub fn z_axis() -> Axis3 {
    Axis3::new(p(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0))
}

/// The block between the corners `a` and `b`, its sides along the axes.
pub fn block(a: Point3, b: Point3) -> Result<Solid, Refusal> {
    let lo = p(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z));
    let size = (b - a).abs();
    Ok(solids::block_at(Frame::at(lo), size.x, size.y, size.z)?)
}

/// The cylinder of radius `r` whose axis runs from `base` along `dir` for `h`.
pub fn cylinder(base: Point3, dir: Vector3<f64>, r: f64, h: f64) -> Result<Solid, Refusal> {
    Ok(solids::cylinder_at(frame(base, dir), r, h)?)
}

/// The cylinder of radius `r` on the vertical through (x, y), from z0 to z1.
pub fn cylinder_z(x: f64, y: f64, z0: f64, z1: f64, r: f64) -> Result<Solid, Refusal> {
    cylinder(p(x, y, z0), v(0.0, 0.0, 1.0), r, z1 - z0)
}

/// The ball of radius `r` about `c`, its poles on the vertical through `c`.
pub fn ball(c: Point3, r: f64) -> Result<Solid, Refusal> {
    Ok(solids::sphere_at(Frame::at(c), r)?)
}

/// The ball of radius `r` about `c`, its poles on the line through `c` along `axis`.
pub fn ball_on(c: Point3, axis: Vector3<f64>, r: f64) -> Result<Solid, Refusal> {
    Ok(solids::sphere_at(frame(c, axis), r)?)
}

pub fn disc(c: Point3, normal: Vector3<f64>, r: f64) -> Result<Face, Refusal> {
    Ok(faces::circle(plane(c, normal), r)?)
}

pub fn extrude(face: Face, dir: Vector3<f64>) -> Result<Solid, Refusal> {
    Ok(solids::extruded(
        face,
        Unit::new_normalize(dir),
        dir.norm(),
    )?)
}

/// The polygon `section`, in a plane through the axis, turned `angle` degrees about the z axis.
pub fn revolved_about_z(section: &[Point3], angle: f64) -> Result<Solid, Refusal> {
    let face = faces::polygon(section)?;
    Ok(ngk::modeling::revolve::revolve_face(
        face,
        z_axis(),
        deg(angle),
    )?)
}

fn boolean_with(
    op: fn(Solid, [&Solid; 1]) -> Result<Solid, ngk::builders::boolean::BooleanError>,
    a: Solid,
    b: &Solid,
) -> Result<Solid, Refusal> {
    attempt(|| Ok(op(a, [b])?))
}

pub fn cut(a: Solid, b: &Solid) -> Result<Solid, Refusal> {
    boolean_with(|a, b| solids::cut(a, b), a, b)
}

pub fn fuse(a: Solid, b: &Solid) -> Result<Solid, Refusal> {
    boolean_with(|a, b| solids::fuse(a, b), a, b)
}

pub fn common(a: Solid, b: &Solid) -> Result<Solid, Refusal> {
    boolean_with(|a, b| solids::intersect(a, b), a, b)
}

/// The solid's volume as the kernel integrates it over the B-rep.
pub fn volume(s: &Solid) -> Result<f64, Refusal> {
    Ok(s.solid().volume()?)
}

/// The solid passes the kernel's own manifold and orientation checks.
pub fn valid(s: &Solid) -> Result<(), Refusal> {
    validate_solid_manifold(s.model(), s.key())?;
    validate_solid_orientation(s.model(), s.key())?;
    Ok(())
}

/// The solid is valid and has the volume `want`, to a relative `rel`.
pub fn has_volume(s: &Solid, want: f64, rel: f64) -> Result<(), Refusal> {
    valid(s)?;
    let got = volume(s)?;
    if (got - want).abs() <= rel * want.abs() {
        Ok(())
    } else {
        Err(Refusal(format!(
            "volume {got} where {want} was expected (relative error {:e})",
            (got - want).abs() / want.abs()
        )))
    }
}

/// The number of faces of the solid.
pub fn face_count(s: &Solid) -> usize {
    s.solid().faces().len()
}

/// Closed forms the answers are checked against.
pub mod exact {
    use super::*;

    pub fn ball(r: f64) -> f64 {
        4.0 / 3.0 * PI * r.powi(3)
    }

    /// The cap of a ball of radius `r` that is `h` high.
    pub fn cap(r: f64, h: f64) -> f64 {
        PI * h * h * (3.0 * r - h) / 3.0
    }

    /// The part common to two balls of radii `big` and `small` whose centres are `d` apart.
    pub fn lens(big: f64, small: f64, d: f64) -> f64 {
        let (r, s) = (big, small);
        PI * (r + s - d).powi(2)
            * (d * d + 2.0 * d * s - 3.0 * s * s + 2.0 * d * r + 6.0 * s * r - 3.0 * r * r)
            / (12.0 * d)
    }

    /// The area common to two discs of radii `r` and `s` whose centres are `d` apart.
    pub fn disc_overlap(r: f64, s: f64, d: f64) -> f64 {
        let a = ((d * d + r * r - s * s) / (2.0 * d * r)).acos();
        let b = ((d * d + s * s - r * r) / (2.0 * d * s)).acos();
        r * r * (a - a.sin() * a.cos()) + s * s * (b - b.sin() * b.cos())
    }

    /// The area two concentric squares of side `side` share, the second turned `degrees`.
    pub fn turned_squares_overlap(side: f64, degrees: f64) -> f64 {
        let a = side / 2.0;
        let t = degrees.to_radians();
        4.0 * a * a * ((t / 2.0).tan() + (PI / 4.0 - t / 2.0).tan())
    }

    /// Simpson's rule on `f` over [a, b] with `n` (even) intervals.
    pub fn simpson(f: impl Fn(f64) -> f64, a: f64, b: f64, n: usize) -> f64 {
        let h = (b - a) / n as f64;
        let mut sum = f(a) + f(b);
        for i in 1..n {
            sum += f(a + i as f64 * h) * if i % 2 == 1 { 4.0 } else { 2.0 };
        }
        sum * h / 3.0
    }

    /// The integral of `f(x, y)` over the disc of radius `radius` about (`cx`, `cy`), by the
    /// midpoint rule in polar coordinates.
    pub fn disc_integral(cx: f64, cy: f64, radius: f64, f: impl Fn(f64, f64) -> f64) -> f64 {
        let (n_rho, n_phi) = (3000, 720);
        let mut sum = 0.0;
        for i in 0..n_rho {
            let rho = (i as f64 + 0.5) / n_rho as f64 * radius;
            let mut ring = 0.0;
            for j in 0..n_phi {
                let phi = (j as f64 + 0.5) / n_phi as f64 * 2.0 * PI;
                ring += f(cx + rho * phi.cos(), cy + rho * phi.sin());
            }
            sum += ring / n_phi as f64 * 2.0 * PI * rho;
        }
        sum * radius / n_rho as f64
    }

    /// The part of the cone of radius `r0` at z = 0 and `r1` at z = `h` below the plane
    /// x sin(t) + z cos(t) = `d`, t in degrees.
    pub fn cone_below_tilted_plane(r0: f64, r1: f64, h: f64, d: f64, tilt: f64) -> f64 {
        let t = tilt.to_radians();
        disc_integral(0.0, 0.0, r0.max(r1), |x, y| {
            let rho = x.hypot(y);
            let wall = if r0 == r1 {
                h
            } else {
                ((r0 - rho) / (r0 - r1) * h).min(h)
            };
            (wall.min((d - x * t.sin()) / t.cos())).max(0.0)
        })
    }

    /// The part of the torus (axis z, major radius `major`, minor `minor`) below the plane
    /// z cos(t) − y sin(t) = `d`, t in degrees.
    pub fn torus_below_tilted_plane(major: f64, minor: f64, d: f64, tilt: f64) -> f64 {
        let t = tilt.to_radians();
        disc_integral(0.0, 0.0, major + minor, |x, y| {
            let half = (minor * minor - (x.hypot(y) - major).powi(2))
                .max(0.0)
                .sqrt();
            let top = (d + y * t.sin()) / t.cos();
            (top + half).min(2.0 * half).max(0.0)
        })
    }

    /// The part common to the cylinder of radius `big` along x and the one of radius `small`
    /// along y whose axis is `offset` higher, both axes meeting the origin's vertical.
    pub fn crossing_cylinders(big: f64, small: f64, offset: f64) -> f64 {
        // z = offset + small sin t; at height z the cut is 2 sqrt(small² − (z − offset)²) wide in
        // x and 2 sqrt(big² − z²) in y.
        simpson(
            |t| {
                let z = offset + small * t.sin();
                4.0 * small * t.cos() * small * t.cos() * (big * big - z * z).max(0.0).sqrt()
            },
            -PI / 2.0,
            PI / 2.0,
            4000,
        )
    }
}

/// The cone or frustum on the z axis: radius `r0` at z = 0 and `r1` at z = `h` (either may be 0).
pub fn cone_z(r0: f64, r1: f64, h: f64) -> Result<Solid, Refusal> {
    let mut section = vec![p(0.0, 0.0, 0.0)];
    if r0 > 0.0 {
        section.push(p(r0, 0.0, 0.0));
    }
    if r1 > 0.0 {
        section.push(p(r1, 0.0, h));
    }
    section.push(p(0.0, 0.0, h));
    // Wound this way the whole-turn face's triangles come out facing outward.
    section.reverse();
    revolved_about_z(&section, 360.0)
}

/// A closed, oriented triangle mesh, found by joining the tessellation's vertices on exact
/// coordinates.
pub struct Mesh {
    pub triangles: Vec<[Point3; 3]>,
}

/// The coordinates of a point as bits, with -0.0 and 0.0 the same.
fn key(a: Point3) -> [u64; 3] {
    [a.x, a.y, a.z].map(|c| {
        if c == 0.0 {
            0.0f64.to_bits()
        } else {
            c.to_bits()
        }
    })
}

impl Mesh {
    pub fn of(s: &Solid) -> Result<Mesh, Refusal> {
        Self::of_with(s, None)
    }

    pub fn of_with(s: &Solid, opts: Option<TessellateOpts>) -> Result<Mesh, Refusal> {
        let solid = s.solid();
        let t = match opts {
            Some(o) => solid.tessellate_with(o)?,
            None => tessellate(&solid)?,
        };
        let at = |i: u32| t.mesh.positions[i as usize];
        Ok(Mesh {
            triangles: t
                .mesh
                .indices
                .chunks_exact(3)
                .map(|c| [at(c[0]), at(c[1]), at(c[2])])
                .collect(),
        })
    }

    /// The signed volume the triangles enclose (positive when they wind counter-clockwise seen
    /// from outside).
    pub fn signed_volume(&self) -> f64 {
        self.triangles
            .iter()
            .map(|[a, b, c]| a.coords.dot(&b.coords.cross(&c.coords)) / 6.0)
            .sum()
    }

    pub fn area(&self) -> f64 {
        self.triangles
            .iter()
            .map(|[a, b, c]| (b - a).cross(&(c - a)).norm() / 2.0)
            .sum()
    }

    /// Why the triangles are not a closed, consistently oriented 2-manifold, if they are not:
    /// every directed edge must occur once and its reverse once.
    pub fn manifold_defect(&self) -> Option<String> {
        let mut directed: HashMap<([u64; 3], [u64; 3]), usize> = HashMap::new();
        let mut degenerate = 0;
        for [a, b, c] in &self.triangles {
            let ks = [key(*a), key(*b), key(*c)];
            if ks[0] == ks[1] || ks[1] == ks[2] || ks[0] == ks[2] {
                degenerate += 1;
                continue;
            }
            for i in 0..3 {
                *directed.entry((ks[i], ks[(i + 1) % 3])).or_default() += 1;
            }
        }
        let mut unmatched = 0;
        let mut doubled = 0;
        for (&(a, b), &n) in &directed {
            if n > 1 {
                doubled += 1;
            }
            if directed.get(&(b, a)).copied().unwrap_or(0) != n {
                unmatched += 1;
            }
        }
        if degenerate + unmatched + doubled == 0 {
            None
        } else {
            Some(format!(
                "{} triangles: {degenerate} degenerate, {unmatched} edges without their reverse, {doubled} edges used twice the same way",
                self.triangles.len()
            ))
        }
    }
}

/// The solid's tessellation is a closed, consistently oriented 2-manifold whose signed volume is
/// positive and within `sag` of the solid's volume, per unit of its area: the most a chord can
/// lie inside the surface, the mesh's own error.
pub fn assert_closed_mesh(s: &Solid, sag: f64) -> Result<(), Refusal> {
    let mesh = attempt(|| Mesh::of(s))?;
    if let Some(defect) = mesh.manifold_defect() {
        return Err(Refusal(format!(
            "mesh is not a closed oriented 2-manifold: {defect}"
        )));
    }
    let (mesh_volume, exact) = (mesh.signed_volume(), volume(s)?);
    if mesh_volume <= 0.0 {
        return Err(Refusal(format!(
            "mesh volume {mesh_volume} is not positive"
        )));
    }
    let allowed = sag * mesh.area() + 1e-9 * exact.abs();
    if (mesh_volume - exact).abs() > allowed {
        return Err(Refusal(format!(
            "mesh volume {mesh_volume} differs from the solid's {exact} by more than {allowed}"
        )));
    }
    Ok(())
}

/// The sag of a circle of radius `r` meshed in `segments` chords.
pub fn chord_sag(r: f64, segments: usize) -> f64 {
    r * (1.0 - (PI / segments as f64).cos())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tetrahedron(flip: bool, missing: bool) -> Mesh {
        let [a, b, c, d] = [
            p(0.0, 0.0, 0.0),
            p(1.0, 0.0, 0.0),
            p(0.0, 1.0, 0.0),
            p(0.0, 0.0, 1.0),
        ];
        let mut triangles = vec![[a, c, b], [a, b, d], [b, c, d], [c, a, d]];
        if flip {
            triangles[0] = [a, b, c];
        }
        if missing {
            triangles.pop();
        }
        Mesh { triangles }
    }

    #[test]
    fn a_closed_tetrahedron_is_a_manifold_of_positive_volume() {
        let t = tetrahedron(false, false);
        assert_eq!(t.manifold_defect(), None);
        assert!((t.signed_volume() - 1.0 / 6.0).abs() < 1e-15);
    }

    #[test]
    fn a_missing_triangle_is_a_defect() {
        assert!(tetrahedron(false, true).manifold_defect().is_some());
    }

    #[test]
    fn a_flipped_triangle_is_a_defect() {
        assert!(tetrahedron(true, false).manifold_defect().is_some());
    }

    #[test]
    fn the_closed_forms_agree_with_known_volumes() {
        // Two unit balls a unit apart share 5π/12; the Steinmetz solid of unit cylinders is 16/3.
        assert!((exact::lens(1.0, 1.0, 1.0) - 5.0 * PI / 12.0).abs() < 1e-12);
        assert!((exact::crossing_cylinders(1.0, 1.0, 0.0) - 16.0 / 3.0).abs() < 1e-9);
        assert!((exact::turned_squares_overlap(2.0, 0.0) - 4.0).abs() < 1e-12);
        let frustum = PI * 4.0 * (25.0 + 5.0 * 3.4 + 3.4 * 3.4) / 3.0;
        assert!(
            (exact::cone_below_tilted_plane(5.0, 1.0, 10.0, 4.0, 0.0) - frustum).abs()
                < 1e-4 * frustum
        );
        let torus = 2.0 * PI * PI * 10.0 * 4.0;
        assert!(
            (exact::torus_below_tilted_plane(10.0, 2.0, 5.0, 0.0) - torus).abs() < 1e-4 * torus
        );
        assert!(
            (exact::disc_overlap(1.0, 1.0, 1.0) - (2.0 * PI / 3.0 - 3f64.sqrt() / 2.0)).abs()
                < 1e-12
        );
    }
}

pub type Edge = Shape<ngk::topology::shape::EdgeTag>;

pub fn line(a: Point3, b: Point3) -> Result<Edge, Refusal> {
    Ok(ngk::modeling::edges::line(a, b)?)
}

/// The arc of radius `r` about the origin of `plane`, from angle `a` to `b` of its frame.
pub fn arc(plane: Plane, r: f64, a: f64, b: f64) -> Result<Edge, Refusal> {
    Ok(ngk::modeling::edges::arc(plane, r, deg(a), deg(b))?)
}

/// The plane through `origin` spanned by `x` and `y`.
pub fn plane_xy(origin: Point3, x: Vector3<f64>, y: Vector3<f64>) -> Plane {
    Plane::from_xy(origin, x, y)
}

/// The planar face bounded by the edges, joined into one closed profile.
pub fn face_from_edges(edges: &[&Edge]) -> Result<Face, Refusal> {
    let profile = ngk::modeling::profiles::from_edges(edges)?;
    Ok(faces::from_profile(&profile)?)
}

/// The edge along a NURBS curve of degree `degree` on the control points.
pub fn nurbs_edge(
    degree: usize,
    points: &[Point3],
    weights: &[f64],
    knots: &[f64],
) -> Result<Edge, Refusal> {
    use ngk::geometry::{ControlPolygon, Curve, Degree, KnotVector, NurbsCurve};
    let curve = NurbsCurve::new(
        Degree::new(degree)?,
        ControlPolygon::from_cartesian(points.to_vec(), weights)?,
        KnotVector::new(knots.to_vec())?,
    )?;
    let d = curve.domain();
    let (a, b) = (
        curve.point_at(d.start.value()),
        curve.point_at(d.end.value()),
    );
    Ok(Shape::build(|m| {
        ngk::builders::edges::add_edge(m, a, b, Curve::Nurbs(curve))
    })?)
}

/// The cubic Bézier edge on four control points.
pub fn bezier_edge(points: [Point3; 4]) -> Result<Edge, Refusal> {
    nurbs_edge(
        3,
        &points,
        &[1.0; 4],
        &[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
    )
}

/// The length of the cubic Bézier curve on four control points.
pub fn bezier_length(c: [Point3; 4]) -> f64 {
    exact::simpson(
        |t| {
            let u = 1.0 - t;
            let d = (c[1] - c[0]) * (3.0 * u * u)
                + (c[2] - c[1]) * (6.0 * u * t)
                + (c[3] - c[2]) * (3.0 * t * t);
            d.norm()
        },
        0.0,
        1.0,
        2000,
    )
}
