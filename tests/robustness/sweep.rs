//! Extrude, revolve, sweep and loft: faces whose boundary
//! has a closed edge with no corner, edges stored against the loop, meridians that meet the axis,
//! frames along a spline.

use std::f64::consts::PI;

use crate::robust::*;
use ngk::builders::loft::LoftOptions;
use ngk::builders::sweep::SweepTransition;
use ngk::geometry::Plane;
use ngk::geometry::{Fraction, Point3};
use ngk::modeling::{edges, faces, loft, profiles, revolve, sweep};
use ngk::topology::shape::Shape;

// B1: closed edges with no corner

/// The disc of radius 1 about (5, 0, 0) in the xz plane, turned `angle` degrees about z.
fn turned_disc(angle: f64) -> Result<Solid, Refusal> {
    let section = disc(p(5.0, 0.0, 0.0), v(0.0, 1.0, 0.0), 1.0)?;
    Ok(revolve::revolve_face(section, z_axis(), deg(angle))?)
}

#[test]
#[ignore = "closed edge with no corner in a builder: MissingVertexPoint (plan/robustness/wsB-sweep-family.md, B1)"]
fn revolving_a_disc_a_full_turn_makes_a_torus() -> Result<(), Refusal> {
    guarded(|| has_volume(&attempt(|| turned_disc(360.0))?, 2.0 * PI * PI * 5.0, 1e-6))
}

#[test]
#[ignore = "closed edge with no corner in a builder: MissingVertexPoint (plan/robustness/wsB-sweep-family.md, B1)"]
fn revolving_a_disc_a_quarter_turn_makes_a_quarter_torus() -> Result<(), Refusal> {
    guarded(|| {
        has_volume(
            &attempt(|| turned_disc(90.0))?,
            2.0 * PI * PI * 5.0 / 4.0,
            1e-6,
        )
    })
}

#[test]
#[ignore = "closed edge with no corner in a builder: sweeping a disc panics (plan/robustness/wsB-sweep-family.md, B1)"]
fn sweeping_a_disc_along_a_line_makes_a_cylinder() -> Result<(), Refusal> {
    guarded(|| {
        let path = line(p(0.0, 0.0, 0.0), p(0.0, 0.0, 10.0))?;
        let section = disc(p(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0), 2.0)?;
        let pipe = attempt(|| {
            Ok(sweep::sweep_face(
                section,
                &path.edge(),
                Default::default(),
            )?)
        })?;
        has_volume(&pipe, PI * 4.0 * 10.0, 1e-9)
    })
}

/// A disc of radius 2 swept along the quarter arc of radius 10 about z, from (10, 0, 0).
pub fn quarter_arc_pipe() -> Result<Solid, Refusal> {
    let path = arc(Plane::xy(), 10.0, 0.0, 90.0)?;
    let section = disc(p(10.0, 0.0, 0.0), v(0.0, 1.0, 0.0), 2.0)?;
    attempt(|| {
        Ok(sweep::sweep_face(
            section,
            &path.edge(),
            Default::default(),
        )?)
    })
}

#[test]
#[ignore = "closed edge with no corner in a builder: sweeping a disc panics (plan/robustness/wsB-sweep-family.md, B1)"]
fn sweeping_a_disc_along_a_quarter_arc_makes_a_quarter_torus() -> Result<(), Refusal> {
    guarded(|| has_volume(&quarter_arc_pipe()?, PI * 4.0 * 10.0 * PI / 2.0, 1e-6))
}

#[test]
#[ignore = "closed edge with no corner in a builder: sweeping a disc panics (plan/robustness/wsB-sweep-family.md, B1)"]
fn sweeping_a_disc_along_a_helix_makes_a_pipe_of_the_helixs_length() -> Result<(), Refusal> {
    guarded(|| {
        let path = edges::helix(z_axis(), 5.0, 4.0, deg(0.0), deg(720.0))?;
        let curve = path.edge().trimmed_curve();
        let section = disc(curve.start(), curve.derivative_at(Fraction::START, 1), 1.0)?;
        let pipe = attempt(|| {
            Ok(sweep::sweep_face(
                section,
                &path.edge(),
                Default::default(),
            )?)
        })?;
        let length = 2.0 * ((2.0 * PI * 5.0).powi(2) + 4.0f64.powi(2)).sqrt();
        has_volume(&pipe, PI * length, 1e-6)
    })
}

/// A disc of radius 2 swept along three edges of a cube of side 30, mitred at both corners.
pub fn mitred_pipe() -> Result<Solid, Refusal> {
    let section = disc(p(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), 2.0)?;
    let path = [
        p(0.0, 0.0, 0.0),
        p(30.0, 0.0, 0.0),
        p(30.0, 30.0, 0.0),
        p(30.0, 30.0, 30.0),
    ];
    let spine = profiles::polyline(&path)?;
    let options = ngk::builders::sweep::SweepOptions {
        transition: SweepTransition::Straight,
        ..Default::default()
    };
    attempt(|| Ok(sweep::sweep_face(section, &spine.profile(), options)?))
}

#[test]
#[ignore = "closed edge with no corner in a builder: sweeping a disc panics (plan/robustness/wsB-sweep-family.md, B1)"]
fn sweeping_a_disc_along_a_mitred_polyline_makes_a_pipe_of_its_length() -> Result<(), Refusal> {
    guarded(|| has_volume(&mitred_pipe()?, PI * 4.0 * 90.0, 1e-6))
}

/// A circle of radius 3 at z = 0 lofted, ruled, to one at z = 10 whose start is turned `turn`
/// degrees.
fn lofted_between_circles(turn: f64) -> Result<Solid, Refusal> {
    let below = disc(p(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0), 3.0)?;
    let turned = plane_xy(
        p(0.0, 0.0, 10.0),
        v(turn.to_radians().cos(), turn.to_radians().sin(), 0.0),
        v(-turn.to_radians().sin(), turn.to_radians().cos(), 0.0),
    );
    let above = faces::circle(turned, 3.0)?;
    attempt(|| Ok(loft::loft(&[&below, &above], LoftOptions::ruled())?))
}

/// A loft between two equal circles is a cylinder when each point meets the point of its own
/// angle, and a ruled solid pinched by `1 − (1 − cos turn)/3` of its area when it meets the point
/// `turn` away.
fn assert_cylinder_or_twisted(turn: f64) -> Result<(), Refusal> {
    let solid = lofted_between_circles(turn)?;
    let cylinder = PI * 9.0 * 10.0;
    let twisted = cylinder * (1.0 - (1.0 - turn.to_radians().cos()) / 3.0);
    valid(&solid)?;
    let got = volume(&solid)?;
    if [cylinder, twisted]
        .iter()
        .any(|want| (got - want).abs() <= 1e-6 * want)
    {
        Ok(())
    } else {
        Err(Refusal(format!(
            "volume {got} is neither the cylinder's {cylinder} nor the twisted loft's {twisted}"
        )))
    }
}

#[test]
fn lofting_between_circles_that_start_together_makes_a_cylinder() -> Result<(), Refusal> {
    guarded(|| has_volume(&lofted_between_circles(0.0)?, PI * 9.0 * 10.0, 1e-6))
}

#[test]
#[ignore = "closed edge with no corner in a builder: CapEdgeMissing (plan/robustness/wsB-sweep-family.md, B1)"]
fn lofting_between_circles_turned_37_degrees_pairs_them_one_way_or_the_other() -> Result<(), Refusal>
{
    guarded(|| assert_cylinder_or_twisted(37.0))
}

#[test]
#[ignore = "closed edge with no corner in a builder: CapEdgeMissing (plan/robustness/wsB-sweep-family.md, B1)"]
fn lofting_between_circles_turned_90_degrees_pairs_them_one_way_or_the_other() -> Result<(), Refusal>
{
    guarded(|| assert_cylinder_or_twisted(90.0))
}

// B2: an edge stored against the loop

/// The unit square less the quarter disc of radius 1/2 about its corner `corner`, as the points
/// `at(a, b)` of the plane it lies in; the arc is stored running along the loop when `along`, and
/// against it otherwise.
fn square_less_quarter_disc(
    at: impl Fn(f64, f64) -> Point3,
    sheet: (Vector3, Vector3),
    corner: (f64, f64),
    along: bool,
) -> Result<Face, Refusal> {
    let (c0, c1) = corner;
    let (x, y) = sheet;
    let centre = at(c0, c1);
    // The loop runs counter-clockwise: the arc leaves the first side at (c0 + 1/2, c1) and comes
    // back from the second at (c0, c1 + 1/2).
    let arc = if along {
        arc(plane_xy(centre, x, -y), 0.5, -90.0, 0.0)?
    } else {
        arc(plane_xy(centre, x, y), 0.5, 0.0, 90.0)?
    };
    let lines = [
        line(at(c0 + 0.5, c1), at(c0 + 1.0, c1))?,
        line(at(c0 + 1.0, c1), at(c0 + 1.0, c1 + 1.0))?,
        line(at(c0 + 1.0, c1 + 1.0), at(c0, c1 + 1.0))?,
        line(at(c0, c1 + 1.0), at(c0, c1 + 0.5))?,
    ];
    face_from_edges(&[&lines[0], &lines[1], &lines[2], &lines[3], &arc])
}

type Vector3 = nalgebra::Vector3<f64>;

fn extruded_square(along: bool) -> Result<Solid, Refusal> {
    let face = square_less_quarter_disc(
        |a, b| p(a, b, 0.0),
        (v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0)),
        (0.0, 0.0),
        along,
    )?;
    attempt(|| extrude(face, v(0.0, 0.0, 1.0)))
}

fn revolved_square(along: bool) -> Result<Solid, Refusal> {
    let face = square_less_quarter_disc(
        |a, b| p(a, 0.0, b),
        (v(1.0, 0.0, 0.0), v(0.0, 0.0, 1.0)),
        (2.0, 0.0),
        along,
    )?;
    attempt(|| Ok(revolve::revolve_face(face, z_axis(), deg(360.0))?))
}

const SQUARE_LESS_QUARTER_DISC: f64 = 1.0 - PI / 16.0;

/// Pappus: the area times the circle its centroid turns, about z, for the square [2, 3] × [0, 1]
/// less the quarter disc of radius 1/2 about (2, 0).
fn revolved_square_volume() -> f64 {
    2.0 * PI * (2.5 - PI / 16.0 * (2.0 + 2.0 / (3.0 * PI)))
}

#[test]
#[ignore = "edge sense ignored when an edge becomes a lateral face (plan/robustness/wsB-sweep-family.md, B2)"]
fn extruding_a_face_whose_arc_is_stored_against_its_loop_keeps_the_arc() -> Result<(), Refusal> {
    guarded(|| has_volume(&extruded_square(false)?, SQUARE_LESS_QUARTER_DISC, 1e-9))
}

#[test]
fn extruding_a_face_whose_arc_is_stored_along_its_loop_keeps_the_arc() -> Result<(), Refusal> {
    guarded(|| has_volume(&extruded_square(true)?, SQUARE_LESS_QUARTER_DISC, 1e-9))
}

#[test]
#[ignore = "edge sense ignored when an edge becomes a lateral face (plan/robustness/wsB-sweep-family.md, B2)"]
fn revolving_a_face_whose_arc_is_stored_against_its_loop_keeps_the_arc() -> Result<(), Refusal> {
    guarded(|| has_volume(&revolved_square(false)?, revolved_square_volume(), 1e-9))
}

#[test]
fn revolving_a_face_whose_arc_is_stored_along_its_loop_keeps_the_arc() -> Result<(), Refusal> {
    guarded(|| has_volume(&revolved_square(true)?, revolved_square_volume(), 1e-9))
}

// B3: meridians that meet the axis, negative angles

/// The edges of a meridian arc and the axis segment closing it, as a face in the xz plane.
fn meridian_face(arc: &Edge, from: Point3, to: Point3) -> Result<Face, Refusal> {
    face_from_edges(&[arc, &line(to, from)?])
}

#[test]
#[ignore = "meridian with both ends on the axis refused: ApexRevolveUnsupported (plan/robustness/wsB-sweep-family.md, B3)"]
fn revolving_a_half_disc_makes_a_ball() -> Result<(), Refusal> {
    guarded(|| {
        let xz = plane_xy(p(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.0, 0.0, 1.0));
        let half = face_from_edges(&[
            &arc(xz, 3.0, -90.0, 90.0)?,
            &line(p(0.0, 0.0, 3.0), p(0.0, 0.0, -3.0))?,
        ])?;
        let ball = attempt(|| Ok(revolve::revolve_face(half, z_axis(), deg(360.0))?))?;
        has_volume(&ball, exact::ball(3.0), 1e-6)
    })
}

#[test]
#[ignore = "meridian with both ends on the axis refused: ApexRevolveUnsupported (plan/robustness/wsB-sweep-family.md, B3)"]
fn revolving_a_half_ellipse_makes_a_spheroid() -> Result<(), Refusal> {
    guarded(|| {
        use ngk::geometry::{Curve, Ellipse};
        // Semi-axes 2 along x and 3 along z, in the xz plane.
        let e = Ellipse::new(
            frame_zx(p(0.0, 0.0, 0.0), v(0.0, -1.0, 0.0), v(1.0, 0.0, 0.0)),
            2.0,
            3.0,
        );
        let (south, north) = (p(0.0, 0.0, -3.0), p(0.0, 0.0, 3.0));
        let half =
            Shape::build(|m| ngk::builders::edges::add_edge(m, south, north, Curve::Ellipse(e)))?;
        let spheroid = attempt(|| {
            Ok(revolve::revolve_face(
                meridian_face(&half, south, north)?,
                z_axis(),
                deg(360.0),
            )?)
        })?;
        has_volume(&spheroid, exact::ball(1.0) * 2.0 * 2.0 * 3.0, 1e-6)
    })
}

fn frame_zx(origin: Point3, z: Vector3, x: Vector3) -> ngk::geometry::Frame {
    ngk::geometry::Frame::from_xz(origin, x, z)
}

#[test]
#[ignore = "circle tangent to the axis: MissingVertexPoint (plan/robustness/wsB-sweep-family.md, B3)"]
fn revolving_a_disc_tangent_to_the_axis_makes_a_horn_torus() -> Result<(), Refusal> {
    guarded(|| {
        let section = disc(p(2.0, 0.0, 0.0), v(0.0, 1.0, 0.0), 2.0)?;
        let horn = attempt(|| Ok(revolve::revolve_face(section, z_axis(), deg(360.0))?))?;
        has_volume(&horn, 2.0 * PI * PI * 8.0, 1e-6)
    })
}

#[test]
#[ignore = "revolving by a negative angle commits a solid whose shell faces inward (plan/robustness/wsB-sweep-family.md, B3)"]
fn revolving_by_a_negative_angle_builds_the_mirror_or_refuses() -> Result<(), Refusal> {
    guarded(|| {
        let section = [
            p(2.0, 0.0, 0.0),
            p(5.0, 0.0, 0.0),
            p(5.0, 0.0, 3.0),
            p(2.0, 0.0, 3.0),
        ];
        match revolved_about_z(&section, -90.0) {
            Err(_) => Ok(()),
            Ok(solid) => has_volume(&solid, PI * (25.0 - 4.0) * 3.0 / 4.0, 1e-6),
        }
    })
}

// B4: frames along a spline

pub const PLANAR_CUBIC: fn() -> [Point3; 4] = || {
    [
        p(0.0, 0.0, 0.0),
        p(12.0, -16.0, 0.0),
        p(32.0, -16.0, 0.0),
        p(44.0, 0.0, 0.0),
    ]
};

pub fn pipe_along(points: [Point3; 4], radius: f64) -> Result<Solid, Refusal> {
    let path = bezier_edge(points)?;
    let curve = path.edge().trimmed_curve();
    let section = disc(
        curve.start(),
        curve.derivative_at(Fraction::START, 1),
        radius,
    )?;
    attempt(|| {
        Ok(sweep::sweep_face(
            section,
            &path.edge(),
            Default::default(),
        )?)
    })
}

#[test]
#[ignore = "sweep frame along a NURBS spine panics (plan/robustness/wsB-sweep-family.md, B4)"]
fn a_pipe_along_a_planar_cubic_has_the_volume_of_its_length() -> Result<(), Refusal> {
    guarded(|| {
        let c = PLANAR_CUBIC();
        has_volume(&pipe_along(c, 2.0)?, PI * 4.0 * bezier_length(c), 1e-6)
    })
}

#[test]
#[ignore = "sweep frame along a NURBS spine panics (plan/robustness/wsB-sweep-family.md, B4)"]
fn a_pipe_along_a_twisted_cubic_has_the_volume_of_its_length() -> Result<(), Refusal> {
    guarded(|| {
        let c = [
            p(0.0, 0.0, 0.0),
            p(10.0, 0.0, 5.0),
            p(20.0, 10.0, 10.0),
            p(30.0, 10.0, 20.0),
        ];
        has_volume(&pipe_along(c, 1.0)?, PI * bezier_length(c), 1e-6)
    })
}
