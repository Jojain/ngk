//! Face splitting: periodic faces cut by several loops or through a pole, a bridge between two
//! loops of a face, and coincident regions with holes.

use std::f64::consts::PI;

use crate::robust::*;
use ngk::modeling::{faces, solids};

// F1: wrapping loops, poles

/// The ring a cylinder of radius `hole` leaves of a ball of radius 5, 4/3 π (25 − hole²)^(3/2).
fn napkin_ring(hole: f64) -> f64 {
    4.0 / 3.0 * PI * (25.0 - hole * hole).powf(1.5)
}

#[test]
fn a_ball_bored_along_its_poles_leaves_a_napkin_ring() -> Result<(), Refusal> {
    guarded(|| {
        has_volume(
            &cut(
                ball(p(0.0, 0.0, 0.0), 5.0)?,
                &cylinder_z(0.0, 0.0, -10.0, 10.0, 3.0)?,
            )?,
            napkin_ring(3.0),
            1e-6,
        )
    })
}

#[test]
fn a_ball_bored_across_its_poles_leaves_a_napkin_ring() -> Result<(), Refusal> {
    guarded(|| {
        let ball = ball_on(p(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), 5.0)?;
        has_volume(
            &cut(ball, &cylinder_z(0.0, 0.0, -10.0, 10.0, 3.0)?)?,
            napkin_ring(3.0),
            1e-6,
        )
    })
}

#[test]
fn a_ball_bored_from_its_centre_to_one_pole_is_a_ball_less_a_half_bore() -> Result<(), Refusal> {
    guarded(|| {
        // Half the ball, 500π/3, less the part of the bore in it: the bore's cylinder up to z = 4 and
        // the cap above.
        let kept = 500.0 * PI / 3.0 - 36.0 * PI - 14.0 * PI / 3.0;
        has_volume(
            &cut(
                ball(p(0.0, 0.0, 0.0), 5.0)?,
                &cylinder_z(0.0, 0.0, -10.0, 0.0, 3.0)?,
            )?,
            kept,
            1e-6,
        )
    })
}

/// A ball of radius 5 cut to the slab |`axis`| ≤ 1.
fn slab_of_a_ball(axis: usize) -> Result<Solid, Refusal> {
    let mut lo = [-10.0; 3];
    let mut hi = [10.0; 3];
    lo[axis] = -1.0;
    hi[axis] = 1.0;
    common(
        ball(p(0.0, 0.0, 0.0), 5.0)?,
        &block(p(lo[0], lo[1], lo[2]), p(hi[0], hi[1], hi[2]))?,
    )
}

#[test]
fn a_ball_cut_by_two_parallel_planes_square_to_its_poles_keeps_a_slab() -> Result<(), Refusal> {
    guarded(|| has_volume(&slab_of_a_ball(2)?, PI * (50.0 - 2.0 / 3.0), 1e-6))
}

#[test]
fn a_ball_cut_by_two_parallel_planes_through_its_poles_keeps_a_slab() -> Result<(), Refusal> {
    guarded(|| has_volume(&slab_of_a_ball(0)?, PI * (50.0 - 2.0 / 3.0), 1e-6))
}

#[test]
fn a_ball_cut_by_a_plane_through_its_centre_square_to_its_poles_keeps_a_half() -> Result<(), Refusal>
{
    guarded(|| {
        let lower = block(p(-10.0, -10.0, -10.0), p(10.0, 10.0, 0.0))?;
        has_volume(
            &cut(ball(p(0.0, 0.0, 0.0), 5.0)?, &lower)?,
            exact::ball(5.0) / 2.0,
            1e-6,
        )
    })
}

#[test]
#[ignore = "section through a pole of a periodic face: a ball halved through its poles (plan/robustness/wsF-splitter-regions.md, F1)"]
fn a_ball_halved_through_its_poles_keeps_a_half() -> Result<(), Refusal> {
    guarded(|| {
        let side = block(p(-10.0, -10.0, -10.0), p(0.0, 10.0, 10.0))?;
        has_volume(
            &cut(ball(p(0.0, 0.0, 0.0), 5.0)?, &side)?,
            exact::ball(5.0) / 2.0,
            1e-6,
        )
    })
}

#[test]
fn a_cube_intersected_with_a_ball_past_its_faces_is_the_ball_less_six_caps() -> Result<(), Refusal>
{
    guarded(|| {
        let cube = block(p(-5.0, -5.0, -5.0), p(5.0, 5.0, 5.0))?;
        has_volume(
            &common(cube, &ball(p(0.0, 0.0, 0.0), 6.0)?)?,
            exact::ball(6.0) - 6.0 * exact::cap(6.0, 1.0),
            1e-6,
        )
    })
}

#[test]
fn a_ball_cut_by_a_plane_off_its_centre_keeps_a_cap() -> Result<(), Refusal> {
    guarded(|| {
        let side = block(p(-10.0, -10.0, -10.0), p(1.0, 10.0, 10.0))?;
        has_volume(
            &cut(ball(p(0.0, 0.0, 0.0), 5.0)?, &side)?,
            exact::cap(5.0, 4.0),
            1e-6,
        )
    })
}

#[test]
fn a_small_ball_cut_from_the_pole_of_a_ball_leaves_a_dimple() -> Result<(), Refusal> {
    guarded(|| {
        let dimple = ball(p(0.0, 0.0, 10.6), 1.2)?;
        has_volume(
            &cut(ball(p(0.0, 0.0, 0.0), 10.0)?, &dimple)?,
            exact::ball(10.0) - exact::lens(10.0, 1.2, 10.6),
            1e-6,
        )
    })
}

#[test]
fn a_small_ball_cut_from_the_equator_of_a_ball_leaves_a_dimple() -> Result<(), Refusal> {
    guarded(|| {
        let dimple = ball(p(10.6, 0.0, 0.0), 1.2)?;
        has_volume(
            &cut(ball(p(0.0, 0.0, 0.0), 10.0)?, &dimple)?,
            exact::ball(10.0) - exact::lens(10.0, 1.2, 10.6),
            1e-6,
        )
    })
}

#[test]
fn a_small_ball_cut_from_a_ball_away_from_the_poles_leaves_a_dimple() -> Result<(), Refusal> {
    guarded(|| {
        let dimple = ball(p(0.0, 5.57, 9.02), 1.2)?;
        let d = (5.57f64 * 5.57 + 9.02 * 9.02).sqrt();
        has_volume(
            &cut(ball(p(0.0, 0.0, 0.0), 10.0)?, &dimple)?,
            exact::ball(10.0) - exact::lens(10.0, 1.2, d),
            1e-6,
        )
    })
}

#[test]
#[ignore = "several wrapping loops on one periodic face: a groove turned into a shaft, result shell open (plan/robustness/wsF-splitter-regions.md, F1)"]
fn a_groove_turned_into_a_shaft_leaves_the_shaft_less_the_ring() -> Result<(), Refusal> {
    guarded(|| {
        // A ring of radii 4 and 8 from z = 4 to 6 taken from a cylinder of radius 5 and height 10.
        let ring = cut(
            cylinder_z(0.0, 0.0, 4.0, 6.0, 8.0)?,
            &cylinder_z(0.0, 0.0, 3.0, 7.0, 4.0)?,
        )?;
        has_volume(
            &cut(cylinder_z(0.0, 0.0, 0.0, 10.0, 5.0)?, &ring)?,
            232.0 * PI,
            1e-6,
        )
    })
}

#[test]
fn a_frustum_cut_by_a_plane_tilted_10_degrees_keeps_the_part_under_it() -> Result<(), Refusal> {
    guarded(|| {
        let tilted = block(p(-10.0, -10.0, 4.0), p(10.0, 10.0, 20.0))?
            .rotated(ngk::geometry::Axis3::y(), deg(10.0));
        has_volume(
            &cut(cone_z(5.0, 1.0, 10.0)?, &tilted)?,
            exact::cone_below_tilted_plane(5.0, 1.0, 10.0, 4.0, 10.0),
            1e-4,
        )
    })
}

#[test]
fn a_frustum_cut_by_a_plane_tilted_2_degrees_keeps_the_part_under_it() -> Result<(), Refusal> {
    guarded(|| {
        let tilted = block(p(-10.0, -10.0, 4.0), p(10.0, 10.0, 20.0))?
            .rotated(ngk::geometry::Axis3::y(), deg(2.0));
        has_volume(
            &cut(cone_z(5.0, 1.0, 10.0)?, &tilted)?,
            exact::cone_below_tilted_plane(5.0, 1.0, 10.0, 4.0, 2.0),
            1e-4,
        )
    })
}

#[test]
fn a_torus_cut_by_a_plane_tilted_3_degrees_keeps_the_part_under_it() -> Result<(), Refusal> {
    guarded(|| {
        let tilted = block(p(-20.0, -20.0, 0.5), p(20.0, 20.0, 5.0))?
            .rotated(ngk::geometry::Axis3::x(), deg(3.0));
        has_volume(
            &cut(solids::torus(10.0, 2.0)?, &tilted)?,
            exact::torus_below_tilted_plane(10.0, 2.0, 0.5, 3.0),
            1e-4,
        )
    })
}

/// The torus (10, 2) less the part of the vertical cylinder of radius 1 about (`x`, `y`).
fn torus_bored(x: f64, y: f64) -> Result<Solid, Refusal> {
    cut(
        solids::torus(10.0, 2.0)?,
        &cylinder_z(x, y, -5.0, 5.0, 1.0)?,
    )
}

/// What the torus (10, 2) holds inside the vertical cylinder of radius 1 about (10, 0).
fn torus_inside_cylinder() -> f64 {
    exact::disc_integral(10.0, 0.0, 1.0, |x, y| {
        2.0 * (4.0 - (x.hypot(y) - 10.0).powi(2)).sqrt()
    })
}

#[test]
#[ignore = "several wrapping loops on one periodic face: a cylinder on the torus's seam (plan/robustness/wsF-splitter-regions.md, F1)"]
fn a_torus_bored_on_its_seam_loses_the_bore() -> Result<(), Refusal> {
    guarded(|| {
        has_volume(
            &torus_bored(10.0, 0.0)?,
            2.0 * PI * PI * 40.0 - torus_inside_cylinder(),
            1e-4,
        )
    })
}

#[test]
#[cfg_attr(
    debug_assertions,
    ignore = "passes with optimisation and takes minutes without it: run with --release"
)]
fn a_torus_bored_beside_its_seam_loses_the_bore() -> Result<(), Refusal> {
    guarded(|| {
        has_volume(
            &torus_bored(0.0, 10.0)?,
            2.0 * PI * PI * 40.0 - torus_inside_cylinder(),
            1e-4,
        )
    })
}

/// What the torus (10, 2) holds inside the ball of radius 3/2 about (10, 0, 1).
fn torus_inside_ball() -> f64 {
    exact::disc_integral(10.0, 0.0, 1.5, |x, y| {
        let reach = (2.25 - (x - 10.0).powi(2) - y * y).max(0.0).sqrt();
        let half = (4.0 - (x.hypot(y) - 10.0).powi(2)).max(0.0).sqrt();
        ((1.0 + reach).min(half) - (1.0 - reach).max(-half)).max(0.0)
    })
}

#[test]
#[ignore = "several wrapping loops on one periodic face: a ball on the torus's seam (plan/robustness/wsF-splitter-regions.md, F1)"]
fn a_torus_dented_by_a_ball_on_its_seam_loses_the_dent() -> Result<(), Refusal> {
    guarded(|| {
        let dented = cut(solids::torus(10.0, 2.0)?, &ball(p(10.0, 0.0, 1.0), 1.5)?)?;
        has_volume(&dented, 2.0 * PI * PI * 40.0 - torus_inside_ball(), 1e-4)
    })
}

#[test]
fn a_torus_dented_by_a_ball_beside_its_seam_loses_the_dent() -> Result<(), Refusal> {
    guarded(|| {
        let dented = cut(solids::torus(10.0, 2.0)?, &ball(p(0.0, 10.0, 1.0), 1.5)?)?;
        has_volume(&dented, 2.0 * PI * PI * 40.0 - torus_inside_ball(), 1e-4)
    })
}

// F2: a bridge between two loops of a face

/// A tube of radii 5 and 3 and height 10 with a hole of radius 3/2 drilled parallel to its axis
/// through the middle of its wall: the hole cuts both walls from one rim to the other.
#[test]
fn a_hole_drilled_through_a_tubes_wall_parallel_to_its_axis_removes_the_overlap()
-> Result<(), Refusal> {
    guarded(|| {
        let tube = cut(
            cylinder_z(0.0, 0.0, 0.0, 10.0, 5.0)?,
            &cylinder_z(0.0, 0.0, -1.0, 11.0, 3.0)?,
        )?;
        let hole = cylinder_z(6.0, 0.0, -1.0, 11.0, 1.5)?;
        has_volume(
            &cut(tube, &hole)?,
            10.0 * (PI * 16.0 - exact::disc_overlap(5.0, 1.5, 6.0)),
            1e-6,
        )
    })
}

#[test]
fn a_wedge_turned_across_a_hub_as_high_as_the_hub_unites_to_the_sum_less_the_overlap()
-> Result<(), Refusal> {
    guarded(|| {
        // A cylinder of radius 10 and height 10 and the sector of 120 degrees between radii 5 and 20
        // and the same height: the sector's part between radii 5 and 10 is in the cylinder.
        let hub = cylinder_z(0.0, 0.0, 0.0, 10.0, 10.0)?;
        let wedge = revolved_about_z(
            &[
                p(5.0, 0.0, 10.0),
                p(20.0, 0.0, 10.0),
                p(20.0, 0.0, 0.0),
                p(5.0, 0.0, 0.0),
            ],
            120.0,
        )?;
        has_volume(
            &fuse(hub, &wedge)?,
            PI * 100.0 * 10.0 + PI * (400.0 - 100.0) * 10.0 / 3.0,
            1e-6,
        )
    })
}

// F3: coincident regions with holes

/// A block of 20 by 20 by 5 whose top is the plane z = 0.
fn slab() -> Result<Solid, Refusal> {
    block(p(-10.0, -10.0, -5.0), p(10.0, 10.0, 0.0))
}

#[test]
fn a_cylinder_standing_on_a_face_unites_to_the_sum() -> Result<(), Refusal> {
    guarded(|| {
        has_volume(
            &fuse(slab()?, &cylinder_z(0.0, 0.0, 0.0, 5.0, 5.0)?)?,
            2000.0 + PI * 25.0 * 5.0,
            1e-9,
        )
    })
}

#[test]
#[ignore = "coincident region with a hole: a tube on a face (plan/robustness/wsF-splitter-regions.md, F3)"]
fn a_tube_standing_on_a_face_unites_to_the_sum() -> Result<(), Refusal> {
    guarded(|| {
        let tube = extrude(
            faces::annulus(plane(p(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)), 5.0, 2.0)?,
            v(0.0, 0.0, 5.0),
        )?;
        has_volume(&fuse(slab()?, &tube)?, 2000.0 + PI * 21.0 * 5.0, 1e-9)
    })
}

#[test]
#[ignore = "coincident region with a hole: a square tube on a face (plan/robustness/wsF-splitter-regions.md, F3)"]
fn a_square_tube_standing_on_a_face_unites_to_the_sum() -> Result<(), Refusal> {
    guarded(|| {
        let tube = cut(
            block(p(-5.0, -5.0, 0.0), p(5.0, 5.0, 5.0))?,
            &block(p(-2.0, -2.0, -1.0), p(2.0, 2.0, 6.0))?,
        )?;
        has_volume(&fuse(slab()?, &tube)?, 2000.0 + 84.0 * 5.0, 1e-9)
    })
}

#[test]
#[ignore = "coincident region with a hole: a block on a face drilled under it (plan/robustness/wsF-splitter-regions.md, F3)"]
fn a_block_standing_on_a_face_drilled_under_it_unites_to_the_sum() -> Result<(), Refusal> {
    guarded(|| {
        let drilled = cut(slab()?, &cylinder_z(0.0, 0.0, -6.0, 1.0, 2.0)?)?;
        let block = block(p(-5.0, -5.0, 0.0), p(5.0, 5.0, 5.0))?;
        has_volume(
            &fuse(drilled, &block)?,
            2000.0 - PI * 4.0 * 5.0 + 500.0,
            1e-9,
        )
    })
}
