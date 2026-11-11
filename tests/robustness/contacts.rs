//! Contacts and tolerances: features far smaller than the part, surfaces that graze or touch,
//! and Booleans taking a Boolean's result as an operand.

use std::f64::consts::PI;

use crate::robust::*;

// G1: the grazing tolerance applied to transverse contacts

/// A cube of side 10 cut through by a slot `w` wide along y, from z = 5 up.
fn slotted(w: f64) -> Result<Solid, Refusal> {
    cut(
        block(p(0.0, 0.0, 0.0), p(10.0, 10.0, 10.0))?,
        &block(p(5.0, -1.0, 5.0), p(5.0 + w, 11.0, 11.0))?,
    )
}

fn slot_removes_its_width(w: f64) -> Result<(), Refusal> {
    has_volume(&slotted(w)?, 1000.0 - w * 50.0, 1e-12)
}

#[test]
fn a_slot_a_hundredth_of_the_part_wide_is_cut() -> Result<(), Refusal> {
    guarded(|| slot_removes_its_width(1e-2))
}

#[test]
#[ignore = "feature under about 1e-4 of the part's size, imprint not realized: a slot 1e-3 wide (plan/robustness/wsG-contacts-tolerances.md, G1)"]
fn a_slot_a_thousandth_of_the_part_wide_is_cut() -> Result<(), Refusal> {
    guarded(|| slot_removes_its_width(1e-3))
}

#[test]
#[ignore = "feature under about 1e-4 of the part's size, imprint not realized: a slot 1e-4 wide (plan/robustness/wsG-contacts-tolerances.md, G1)"]
fn a_slot_a_ten_thousandth_of_the_part_wide_is_cut() -> Result<(), Refusal> {
    guarded(|| slot_removes_its_width(1e-4))
}

#[test]
#[ignore = "feature under about 1e-4 of the part's size, imprint not realized: a slot 1e-6 wide (plan/robustness/wsG-contacts-tolerances.md, G1)"]
fn a_slot_a_millionth_of_the_part_wide_is_cut() -> Result<(), Refusal> {
    guarded(|| slot_removes_its_width(1e-6))
}

#[test]
#[ignore = "feature under about 1e-4 of the part's size, imprint not realized: a slot 1e-3 wide that stops inside the part (plan/robustness/wsG-contacts-tolerances.md, G1)"]
fn a_slot_1e_3_wide_that_stops_inside_the_part_is_cut() -> Result<(), Refusal> {
    guarded(|| {
        let w = 1e-3;
        let slot = block(p(5.0, 2.0, 5.0), p(5.0 + w, 8.0, 11.0))?;
        has_volume(
            &cut(block(p(0.0, 0.0, 0.0), p(10.0, 10.0, 10.0))?, &slot)?,
            1000.0 - w * 6.0 * 5.0,
            1e-12,
        )
    })
}

#[test]
fn a_cube_united_with_a_copy_turned_10_degrees_is_the_union_of_the_two_squares_prisms()
-> Result<(), Refusal> {
    guarded(|| {
        let (a, b) = turned_cubes(10.0, 0.0)?;
        has_volume(
            &fuse(a, &b)?,
            (2.0 * 100.0 - exact::turned_squares_overlap(10.0, 10.0)) * 10.0,
            1e-9,
        )
    })
}

#[test]
#[ignore = "feature under about 1e-4 of the part's size, imprint not realized: a cube turned 0.001 degrees (plan/robustness/wsG-contacts-tolerances.md, G1)"]
fn a_cube_united_with_a_copy_turned_a_thousandth_of_a_degree_is_the_union_of_the_two_squares_prisms()
-> Result<(), Refusal> {
    guarded(|| {
        let (a, b) = turned_cubes(0.001, 0.0)?;
        has_volume(
            &fuse(a, &b)?,
            (2.0 * 100.0 - exact::turned_squares_overlap(10.0, 0.001)) * 10.0,
            1e-9,
        )
    })
}

#[test]
fn a_cube_united_with_a_taller_copy_turned_10_degrees_keeps_the_overhang() -> Result<(), Refusal> {
    guarded(|| {
        let (a, b) = turned_cubes(10.0, 1.0)?;
        let overlap = exact::turned_squares_overlap(10.0, 10.0) * 10.0;
        has_volume(&fuse(a, &b)?, 1000.0 + 1200.0 - overlap, 1e-9)
    })
}

/// A cube of side 10 centred on the z axis from z = 0, and a copy `grow` taller at each end,
/// turned `degrees` about z.
fn turned_cubes(degrees: f64, grow: f64) -> Result<(Solid, Solid), Refusal> {
    let a = block(p(-5.0, -5.0, 0.0), p(5.0, 5.0, 10.0))?;
    let b = block(p(-5.0, -5.0, -grow), p(5.0, 5.0, 10.0 + grow))?.rotated(z_axis(), deg(degrees));
    Ok((a, b))
}

/// A cube of side 10 less the half space above a plane turned `degrees` about x from z = 5.
fn cube_under_a_turned_plane(degrees: f64) -> Result<Solid, Refusal> {
    let above = block(p(-5.0, -5.0, 5.0), p(15.0, 15.0, 15.0))?
        .rotated(ngk::geometry::Axis3::x(), deg(degrees));
    cut(block(p(0.0, 0.0, 0.0), p(10.0, 10.0, 10.0))?, &above)
}

fn plane_turned(degrees: f64) -> Result<(), Refusal> {
    let t = degrees.to_radians();
    has_volume(
        &cube_under_a_turned_plane(degrees)?,
        500.0 * (1.0 + t.sin()) / t.cos(),
        1e-9,
    )
}

#[test]
fn a_cube_cut_by_a_plane_turned_a_tenth_of_a_degree_keeps_the_wedge_under_it() -> Result<(), Refusal>
{
    guarded(|| plane_turned(0.1))
}

#[test]
fn a_cube_cut_by_a_plane_turned_a_thousandth_of_a_degree_keeps_the_wedge_under_it()
-> Result<(), Refusal> {
    guarded(|| plane_turned(0.001))
}

#[test]
fn a_notch_a_thousandth_of_the_part_wide_is_cut_from_a_block_of_a_thousand() -> Result<(), Refusal>
{
    guarded(|| {
        let big = block(p(0.0, 0.0, 0.0), p(1000.0, 1000.0, 1000.0))?;
        let notch = block(p(1.0, 1.0, 999.999), p(1.001, 1.001, 1001.0))?;
        let notched = cut(big, &notch)?;
        valid(&notched)?;
        assert_eq!(face_count(&notched), 11, "a block with a pocket in its top");
        Ok(())
    })
}

// G2: tangent contacts

#[test]
fn a_cube_united_with_a_half_cylinder_tangent_to_two_of_its_faces_gains_the_half()
-> Result<(), Refusal> {
    guarded(|| {
        let cube = block(p(0.0, 0.0, 0.0), p(10.0, 10.0, 10.0))?;
        has_volume(
            &fuse(cube, &cylinder_z(10.0, 5.0, 0.0, 10.0, 5.0)?)?,
            1000.0 + 125.0 * PI,
            1e-9,
        )
    })
}

#[test]
fn a_cube_united_with_a_half_cylinder_inside_its_faces_gains_the_half() -> Result<(), Refusal> {
    guarded(|| {
        let cube = block(p(0.0, 0.0, 0.0), p(10.0, 10.0, 10.0))?;
        has_volume(
            &fuse(cube, &cylinder_z(10.0, 5.0, 0.0, 10.0, 4.0)?)?,
            1000.0 + 80.0 * PI,
            1e-9,
        )
    })
}

#[test]
#[ignore = "tangent contact imprinted as a section that separates nothing: a cube and its midsphere (plan/robustness/wsG-contacts-tolerances.md, G2)"]
fn a_cube_intersected_with_its_midsphere_is_the_ball_less_six_caps() -> Result<(), Refusal> {
    guarded(|| {
        let r = 50f64.sqrt();
        let cube = block(p(-5.0, -5.0, -5.0), p(5.0, 5.0, 5.0))?;
        has_volume(
            &common(cube, &ball(p(0.0, 0.0, 0.0), r)?)?,
            exact::ball(r) - 6.0 * exact::cap(r, r - 5.0),
            1e-6,
        )
    })
}

#[test]
#[ignore = "tangent contact imprinted as a section that separates nothing: a cut by a cylinder touching a face along a line (plan/robustness/wsG-contacts-tolerances.md, G2)"]
fn a_cube_less_a_cylinder_that_only_touches_it_is_the_cube() -> Result<(), Refusal> {
    guarded(|| {
        // The cylinder of radius 5 about (15, 5) touches the cube's face x = 10 along a line.
        let cube = block(p(0.0, 0.0, 0.0), p(10.0, 10.0, 10.0))?;
        has_volume(
            &cut(cube, &cylinder_z(15.0, 5.0, -1.0, 11.0, 5.0)?)?,
            1000.0,
            1e-9,
        )
    })
}

#[test]
fn a_cube_less_a_block_that_only_touches_it_is_the_cube() -> Result<(), Refusal> {
    guarded(|| {
        let cube = block(p(0.0, 0.0, 0.0), p(10.0, 10.0, 10.0))?;
        has_volume(
            &cut(cube, &block(p(10.0, 0.0, 0.0), p(20.0, 10.0, 10.0))?)?,
            1000.0,
            1e-9,
        )
    })
}

#[test]
fn two_cubes_that_share_only_an_edge_do_not_unite_to_a_solid() -> Result<(), Refusal> {
    guarded(|| {
        // The union is pinched along the shared line: not a manifold solid, so the Boolean answers
        // with an error rather than a panic or a solid.
        let a = block(p(0.0, 0.0, 0.0), p(1.0, 1.0, 1.0))?;
        match fuse(a, &block(p(1.0, 1.0, 0.0), p(2.0, 2.0, 1.0))?) {
            Err(e) if !e.panicked() => Ok(()),
            Err(e) => Err(e),
            Ok(solid) => Err(Refusal(format!(
                "a pinched union came back as a solid of volume {}",
                volume(&solid)?
            ))),
        }
    })
}

// G3: a point contact read as two solids

#[test]
#[ignore = "point contact read as two solids (plan/robustness/wsG-contacts-tolerances.md, G3)"]
fn a_cone_whose_tip_touches_a_cubes_face_from_inside_unites_to_the_sum_less_the_overlap()
-> Result<(), Refusal> {
    guarded(|| {
        // The cone, apex at the origin and base of radius 5 at z = 15, is inside the cube up to z = 10,
        // where its radius is 10/3.
        let cube = block(p(-10.0, -10.0, 0.0), p(10.0, 10.0, 10.0))?;
        let cone = cone_z(0.0, 5.0, 15.0)?;
        has_volume(
            &fuse(cube, &cone)?,
            8000.0 + 125.0 * PI - PI * 1000.0 / 27.0,
            1e-6,
        )
    })
}

// G4: Booleans on Booleans' results

/// The ball of radius 20 less the half space below z = 0.
fn half_ball(poles: nalgebra::Vector3<f64>) -> Result<Solid, Refusal> {
    let lower = block(p(-30.0, -30.0, -30.0), p(30.0, 30.0, 0.0))?;
    cut(ball_on(p(0.0, 0.0, 0.0), poles, 20.0)?, &lower)
}

#[test]
fn a_ball_less_the_half_space_below_its_centre_is_a_half_ball() -> Result<(), Refusal> {
    guarded(|| has_volume(&half_ball(v(0.0, 0.0, 1.0))?, exact::ball(20.0) / 2.0, 1e-6))
}

#[test]
#[ignore = "Boolean on a Boolean's result, a half ball's degenerate span: NURBS conversion of an empty interval (plan/robustness/wsG-contacts-tolerances.md, G4)"]
fn a_half_ball_united_with_a_plate_under_it_gains_the_plate() -> Result<(), Refusal> {
    guarded(|| {
        let plate = cylinder_z(0.0, 0.0, -2.0, 0.0, 20.0)?;
        has_volume(
            &fuse(plate, &half_ball(v(0.0, 0.0, 1.0))?)?,
            exact::ball(20.0) / 2.0 + PI * 400.0 * 2.0,
            1e-6,
        )
    })
}

#[test]
#[ignore = "Boolean on a Boolean's result: a half ball with poles along x, imprint not realized (plan/robustness/wsG-contacts-tolerances.md, G4)"]
fn a_half_ball_with_poles_along_x_united_with_a_plate_under_it_gains_the_plate()
-> Result<(), Refusal> {
    guarded(|| {
        let plate = cylinder_z(0.0, 0.0, -2.0, 0.0, 20.0)?;
        has_volume(
            &fuse(half_ball(v(1.0, 0.0, 0.0))?, &plate)?,
            exact::ball(20.0) / 2.0 + PI * 400.0 * 2.0,
            1e-6,
        )
    })
}

#[test]
#[ignore = "Boolean on a Boolean's result, a half ball's degenerate span: NURBS conversion of an empty interval (plan/robustness/wsG-contacts-tolerances.md, G4)"]
fn a_half_ball_intersected_with_a_post_keeps_the_post_up_to_its_dome() -> Result<(), Refusal> {
    guarded(|| {
        let half = cut(
            ball(p(0.0, 0.0, 0.0), 18.0)?,
            &block(p(-30.0, -30.0, -30.0), p(30.0, 30.0, 0.0))?,
        )?;
        let post = cylinder_z(0.0, 0.0, 0.0, 18.0, 3.0)?;
        let top = (18.0f64 * 18.0 - 9.0).sqrt();
        has_volume(
            &common(half, &post)?,
            PI * 9.0 * top + exact::cap(18.0, 18.0 - top),
            1e-6,
        )
    })
}
