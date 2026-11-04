//! Tessellation: every solid meshes into a closed, consistently oriented 2-manifold whose volume
//! is the solid's, within the mesh's sag.

use crate::robust::*;
use ngk::modeling::{faces, solids};

/// The viewer samples a circle in 64 chords and a meridian in 32.
const SAG: usize = 32;

#[test]
fn a_block_meshes_closed() -> Result<(), Refusal> {
    guarded(|| assert_closed_mesh(&block(p(0.0, 0.0, 0.0), p(4.0, 3.0, 2.0))?, 0.0))
}

#[test]
#[ignore = "mesh not closed, neighbouring faces do not share their edge vertices (plan/robustness/wsA-tessellation.md)"]
fn a_cylinder_meshes_closed() -> Result<(), Refusal> {
    guarded(|| assert_closed_mesh(&solids::cylinder(3.0, 10.0)?, chord_sag(3.0, SAG)))
}

#[test]
#[ignore = "mesh not closed, neighbouring faces do not share their edge vertices (plan/robustness/wsA-tessellation.md)"]
fn a_tube_meshes_closed() -> Result<(), Refusal> {
    guarded(|| {
        let ring = faces::annulus(plane(p(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)), 5.0, 2.0)?;
        assert_closed_mesh(&extrude(ring, v(0.0, 0.0, 5.0))?, chord_sag(5.0, SAG))
    })
}

#[test]
#[ignore = "mesh not closed, neighbouring faces do not share their edge vertices (plan/robustness/wsA-tessellation.md)"]
fn a_drilled_plate_meshes_closed() -> Result<(), Refusal> {
    guarded(|| {
        let plate = block(p(-10.0, -10.0, 0.0), p(10.0, 10.0, 10.0))?;
        assert_closed_mesh(
            &cut(plate, &cylinder_z(0.0, 0.0, -1.0, 11.0, 3.0)?)?,
            chord_sag(3.0, SAG),
        )
    })
}

#[test]
fn a_sphere_meshes_closed() -> Result<(), Refusal> {
    guarded(|| assert_closed_mesh(&solids::sphere(5.0)?, chord_sag(5.0, SAG)))
}

#[test]
fn a_torus_meshes_closed() -> Result<(), Refusal> {
    guarded(|| assert_closed_mesh(&solids::torus(10.0, 2.0)?, chord_sag(12.0, SAG)))
}

/// The washer's half section, wound either way.
fn washer_section(reversed: bool) -> Vec<ngk::geometry::Point3> {
    let mut section = vec![
        p(2.0, 0.0, 0.0),
        p(5.0, 0.0, 0.0),
        p(5.0, 0.0, 3.0),
        p(2.0, 0.0, 3.0),
    ];
    if reversed {
        section.reverse();
    }
    section
}

fn washer_meshes_closed_at(angle: f64) -> Result<(), Refusal> {
    for reversed in [false, true] {
        let washer = revolved_about_z(&washer_section(reversed), angle)?;
        assert_closed_mesh(&washer, chord_sag(5.0, SAG)).map_err(|e| {
            Refusal(format!(
                "section wound {}: {}",
                if reversed {
                    "the other way"
                } else {
                    "as given"
                },
                e.0
            ))
        })?;
    }
    Ok(())
}

#[test]
#[ignore = "mesh not closed, neighbouring faces do not share their edge vertices (plan/robustness/wsA-tessellation.md)"]
fn a_revolved_washer_meshes_closed_at_360_degrees() -> Result<(), Refusal> {
    guarded(|| washer_meshes_closed_at(360.0))
}

#[test]
#[ignore = "mesh not closed, neighbouring faces do not share their edge vertices (plan/robustness/wsA-tessellation.md)"]
fn a_revolved_washer_meshes_closed_at_90_degrees() -> Result<(), Refusal> {
    guarded(|| washer_meshes_closed_at(90.0))
}

#[test]
#[ignore = "revolving by a negative angle commits an inward solid that does not tessellate (plan/robustness/wsB-sweep-family.md, B3)"]
fn a_revolved_washer_meshes_closed_at_minus_90_degrees() -> Result<(), Refusal> {
    guarded(|| washer_meshes_closed_at(-90.0))
}

#[test]
#[ignore = "mesh not closed, neighbouring faces do not share their edge vertices (plan/robustness/wsA-tessellation.md)"]
fn a_long_thin_cylinder_meshes_closed() -> Result<(), Refusal> {
    guarded(|| assert_closed_mesh(&solids::cylinder(0.01, 1000.0)?, chord_sag(0.01, SAG)))
}
