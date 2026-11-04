//! Surface/surface intersection, analytic (D) and NURBS (E): the Booleans of cylinders, balls,
//! cones and tori whose sections have no closed form the kernel reads, checked against closed
//! forms or a numeric integral of the answer.

use std::f64::consts::PI;

use crate::robust::*;
use ngk::modeling::solids;

// D: analytic pairs

/// Two cylinders crossing at right angles, radius 5 along x and `small` along y, both 30 long and
/// the second `offset` higher.
fn crossing(small: f64, offset: f64) -> Result<(Solid, Solid), Refusal> {
    Ok((
        cylinder(p(-15.0, 0.0, 0.0), v(1.0, 0.0, 0.0), 5.0, 30.0)?,
        cylinder(p(0.0, -15.0, offset), v(0.0, 1.0, 0.0), small, 30.0)?,
    ))
}

#[test]
#[ignore = "intersection coverage incomplete, analytic pair (plan/robustness/wsD-analytic-ssi.md)"]
fn two_cylinders_of_one_radius_share_a_steinmetz_solid() -> Result<(), Refusal> {
    guarded(|| {
        let (a, b) = crossing(5.0, 0.0)?;
        has_volume(&common(a, &b)?, 16.0 * 125.0 / 3.0, 1e-6)
    })
}

#[test]
#[ignore = "intersection coverage incomplete, analytic pair (plan/robustness/wsD-analytic-ssi.md)"]
fn two_cylinders_of_one_radius_unite_to_the_sum_less_the_steinmetz_solid() -> Result<(), Refusal> {
    guarded(|| {
        let (a, b) = crossing(5.0, 0.0)?;
        has_volume(
            &fuse(a, &b)?,
            2.0 * PI * 25.0 * 30.0 - 16.0 * 125.0 / 3.0,
            1e-6,
        )
    })
}

#[test]
#[ignore = "intersection coverage incomplete, analytic pair (plan/robustness/wsD-analytic-ssi.md)"]
fn two_cylinders_of_two_radii_unite_to_the_sum_less_what_they_share() -> Result<(), Refusal> {
    guarded(|| {
        let (a, b) = crossing(3.0, 0.0)?;
        let shared = exact::crossing_cylinders(5.0, 3.0, 0.0);
        has_volume(&fuse(a, &b)?, PI * (25.0 + 9.0) * 30.0 - shared, 1e-6)
    })
}

#[test]
#[cfg_attr(
    debug_assertions,
    ignore = "passes with optimisation and takes minutes without it: run with --release"
)]
fn two_cylinders_whose_axes_miss_by_one_unite_to_the_sum_less_what_they_share()
-> Result<(), Refusal> {
    guarded(|| {
        let (a, b) = crossing(3.0, 1.0)?;
        let shared = exact::crossing_cylinders(5.0, 3.0, 1.0);
        has_volume(&fuse(a, &b)?, PI * (25.0 + 9.0) * 30.0 - shared, 1e-6)
    })
}

#[test]
#[ignore = "intersection coverage incomplete, analytic pair (plan/robustness/wsD-analytic-ssi.md)"]
fn two_coaxial_cylinders_that_overlap_unite_to_one() -> Result<(), Refusal> {
    guarded(|| {
        let (a, b) = (
            cylinder_z(0.0, 0.0, 0.0, 10.0, 5.0)?,
            cylinder_z(0.0, 0.0, 6.0, 16.0, 5.0)?,
        );
        has_volume(&fuse(a, &b)?, PI * 25.0 * 16.0, 1e-9)
    })
}

#[test]
fn two_balls_cutting_each_other_unite_to_the_sum_less_their_lens() -> Result<(), Refusal> {
    guarded(|| {
        let (a, b) = (ball(p(0.0, 0.0, 0.0), 5.0)?, ball(p(3.0, 2.0, 1.0), 4.0)?);
        let d = 14.0f64.sqrt();
        has_volume(
            &fuse(a, &b)?,
            exact::ball(5.0) + exact::ball(4.0) - exact::lens(5.0, 4.0, d),
            1e-6,
        )
    })
}

#[test]
fn a_cylinder_standing_coaxially_on_a_ball_unites_to_its_part_outside() -> Result<(), Refusal> {
    guarded(|| {
        // The ball of radius 5 meets the cylinder of radius 3 from z = 0 to 10 up to z = 4, and its
        // cap above that lies inside the cylinder.
        let (a, b) = (
            ball(p(0.0, 0.0, 0.0), 5.0)?,
            cylinder_z(0.0, 0.0, 0.0, 10.0, 3.0)?,
        );
        has_volume(&fuse(a, &b)?, 216.0 * PI, 1e-6)
    })
}

#[test]
fn a_frustum_cut_by_a_plane_square_to_its_axis_loses_the_part_above() -> Result<(), Refusal> {
    guarded(|| {
        let frustum = cone_z(5.0, 1.0, 10.0)?;
        let above = block(p(-10.0, -10.0, 4.0), p(10.0, 10.0, 20.0))?;
        has_volume(
            &cut(frustum, &above)?,
            PI * 4.0 * (25.0 + 5.0 * 3.4 + 3.4 * 3.4) / 3.0,
            1e-6,
        )
    })
}

#[test]
fn a_cylinder_cut_by_a_tilted_plane_keeps_the_volume_under_it() -> Result<(), Refusal> {
    guarded(|| {
        let tilted = block(p(-10.0, -10.0, 4.0), p(10.0, 10.0, 20.0))?
            .rotated(ngk::geometry::Axis3::y(), deg(10.0));
        has_volume(
            &cut(cylinder_z(0.0, 0.0, 0.0, 10.0, 5.0)?, &tilted)?,
            PI * 25.0 * 4.0 / 10f64.to_radians().cos(),
            1e-6,
        )
    })
}

#[test]
#[cfg_attr(
    debug_assertions,
    ignore = "passes with optimisation and takes minutes without it: run with --release"
)]
fn a_torus_cut_by_a_plane_square_to_its_axis_loses_the_part_above() -> Result<(), Refusal> {
    guarded(|| {
        let above = block(p(-20.0, -20.0, 0.5), p(20.0, 20.0, 5.0))?;
        // The plane cuts the circle of radius 2 at distance 1/2 from its centre; the segment above
        // is symmetric about the axis the torus turns about, 10 away.
        let segment = 4.0 * (0.5f64 / 2.0).acos() - 0.5 * (4.0f64 - 0.25).sqrt();
        has_volume(
            &cut(solids::torus(10.0, 2.0)?, &above)?,
            2.0 * PI * PI * 10.0 * 4.0 - 2.0 * PI * 10.0 * segment,
            1e-6,
        )
    })
}

/// The cylinder of radius 1.5 standing on the wall of a cylinder of radius 5, a ball of radius
/// 1.5 centred on the wall: the ball's part inside the cylinder.
fn ball_inside_the_wall() -> f64 {
    let r = 1.5;
    exact::simpson(
        |t| {
            let rho = r * t.sin();
            2.0 * (r * r - rho * rho).sqrt() * rho * (PI - 2.0 * (rho / 10.0).asin()) * r * t.cos()
        },
        0.0,
        PI / 2.0,
        4000,
    )
}

#[test]
#[ignore = "intersection coverage incomplete, analytic pair (plan/robustness/wsD-analytic-ssi.md)"]
fn a_small_ball_centred_on_a_cylinders_wall_is_cut_from_it() -> Result<(), Refusal> {
    guarded(|| {
        let wall = cylinder_z(0.0, 0.0, 0.0, 10.0, 5.0)?;
        has_volume(
            &cut(wall, &ball(p(0.0, 5.0, 5.0), 1.5)?)?,
            PI * 25.0 * 10.0 - ball_inside_the_wall(),
            1e-6,
        )
    })
}

#[test]
#[ignore = "intersection coverage incomplete, analytic pair (plan/robustness/wsD-analytic-ssi.md)"]
fn a_small_ball_centred_on_a_cylinders_seam_is_cut_from_it() -> Result<(), Refusal> {
    guarded(|| {
        let wall = cylinder_z(0.0, 0.0, 0.0, 10.0, 5.0)?;
        has_volume(
            &cut(wall, &ball(p(5.0, 0.0, 5.0), 1.5)?)?,
            PI * 25.0 * 10.0 - ball_inside_the_wall(),
            1e-6,
        )
    })
}

// E: NURBS and fitted sections

/// A block of side 20 drilled through along x by a hole of radius 3 and then along y by a hole of
/// radius `small` whose axis is `offset` above the first's, at model scale `scale`.
fn crossing_holes(small: f64, offset: f64, scale: f64) -> Result<(Solid, f64), Refusal> {
    let s = scale;
    let plate = block(
        p(-10.0 * s, -10.0 * s, -10.0 * s),
        p(10.0 * s, 10.0 * s, 10.0 * s),
    )?;
    let first = cylinder(p(-15.0 * s, 0.0, 0.0), v(1.0, 0.0, 0.0), 3.0 * s, 30.0 * s)?;
    let second = cylinder(
        p(0.0, -15.0 * s, offset * s),
        v(0.0, 1.0, 0.0),
        small * s,
        30.0 * s,
    )?;
    let drilled = cut(cut(plate, &first)?, &second)?;
    let volume =
        8000.0 - 20.0 * PI * (9.0 + small * small) + exact::crossing_cylinders(3.0, small, offset);
    Ok((drilled, volume * s.powi(3)))
}

fn crossing_holes_have_the_volume(small: f64, offset: f64, scale: f64) -> Result<(), Refusal> {
    let (solid, volume) = crossing_holes(small, offset, scale)?;
    has_volume(&solid, volume, 1e-4)
}

#[test]
#[ignore = "surface intersection coverage is incomplete (plan/robustness/wsE-nurbs-ssi.md)"]
fn two_holes_of_radii_3_and_2_cross_with_their_axes_meeting() -> Result<(), Refusal> {
    guarded(|| crossing_holes_have_the_volume(2.0, 0.0, 1.0))
}

#[test]
#[cfg_attr(
    debug_assertions,
    ignore = "passes with optimisation and takes minutes without it: run with --release"
)]
fn two_holes_cross_with_their_axes_1e_3_apart() -> Result<(), Refusal> {
    guarded(|| crossing_holes_have_the_volume(2.0, 1e-3, 1.0))
}

#[test]
#[cfg_attr(
    debug_assertions,
    ignore = "passes with optimisation and takes minutes without it: run with --release"
)]
fn two_holes_cross_with_their_axes_a_tenth_apart() -> Result<(), Refusal> {
    guarded(|| crossing_holes_have_the_volume(2.0, 0.1, 1.0))
}

#[test]
#[cfg_attr(
    debug_assertions,
    ignore = "passes with optimisation and takes minutes without it: run with --release"
)]
fn two_holes_cross_with_their_axes_half_a_unit_apart() -> Result<(), Refusal> {
    guarded(|| crossing_holes_have_the_volume(2.0, 0.5, 1.0))
}

#[test]
#[ignore = "surface intersection coverage is incomplete (plan/robustness/wsE-nurbs-ssi.md)"]
fn two_holes_of_radii_ratio_0_99_cross() -> Result<(), Refusal> {
    guarded(|| crossing_holes_have_the_volume(2.97, 0.0, 1.0))
}

#[test]
#[ignore = "surface intersection coverage is incomplete (plan/robustness/wsE-nurbs-ssi.md)"]
fn two_holes_of_radii_ratio_0_2_cross() -> Result<(), Refusal> {
    guarded(|| crossing_holes_have_the_volume(0.6, 0.0, 1.0))
}

#[test]
#[ignore = "surface intersection coverage is incomplete (plan/robustness/wsE-nurbs-ssi.md)"]
fn two_holes_cross_in_a_part_a_hundredth_the_size() -> Result<(), Refusal> {
    guarded(|| crossing_holes_have_the_volume(2.0, 0.0, 1e-2))
}

#[test]
#[ignore = "surface intersection coverage is incomplete (plan/robustness/wsE-nurbs-ssi.md)"]
fn two_holes_cross_in_a_part_a_thousand_times_the_size() -> Result<(), Refusal> {
    guarded(|| crossing_holes_have_the_volume(2.0, 0.0, 1e3))
}
