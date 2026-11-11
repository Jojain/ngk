//! A builder that cannot finish returns an error with the model as it found it; it never panics
//! mid-edit and never commits a solid that fails the orientation check.

use std::f64::consts::PI;

use crate::robust::*;
use crate::sweep::{PLANAR_CUBIC, mitred_pipe, pipe_along, quarter_arc_pipe};

/// The result is a solid or a refusal, but not a panic.
fn never_panics<T>(result: Result<T, Refusal>) -> Result<(), Refusal> {
    match result {
        Err(e) if e.panicked() => Err(e),
        _ => Ok(()),
    }
}

#[test]
fn a_disc_swept_along_an_arc_is_a_solid_or_an_error() -> Result<(), Refusal> {
    guarded(|| never_panics(quarter_arc_pipe()))
}

#[test]
fn a_disc_swept_along_mitred_edges_is_a_solid_or_an_error() -> Result<(), Refusal> {
    guarded(|| never_panics(mitred_pipe()))
}

#[test]
fn a_disc_swept_along_a_cubic_is_a_solid_or_an_error() -> Result<(), Refusal> {
    guarded(|| never_panics(pipe_along(PLANAR_CUBIC(), 2.0)))
}

/// A hub, a rim and a bore with a V groove turned into the rim.
fn v_belt_pulley() -> Result<Solid, Refusal> {
    let body = fuse(
        cylinder_z(0.0, 0.0, -14.0, 14.0, 12.0)?,
        &cylinder_z(0.0, 0.0, -9.0, 9.0, 28.0)?,
    )?;
    let body = cut(body, &cylinder_z(0.0, 0.0, -15.0, 15.0, 6.0)?)?;
    let groove = revolved_about_z(
        &[p(22.0, 0.0, 0.0), p(29.0, 0.0, 5.0), p(29.0, 0.0, -5.0)],
        360.0,
    )?;
    cut(body, &groove)
}

#[test]
fn a_groove_turned_into_a_pulley_is_a_solid_or_an_error() -> Result<(), Refusal> {
    guarded(|| never_panics(v_belt_pulley()))
}

#[test]
fn a_groove_turned_into_a_pulley_has_the_volume_of_the_parts() -> Result<(), Refusal> {
    guarded(|| {
        // Hub and rim joined, less the bore, less the part of the groove inside the rim.
        has_volume(
            &v_belt_pulley()?,
            PI * (15552.0 - 1008.0 - 9360.0 / 7.0),
            1e-6,
        )
    })
}

#[test]
fn a_solid_that_is_committed_passes_the_orientation_check() -> Result<(), Refusal> {
    guarded(|| {
        let section = [
            p(2.0, 0.0, 0.0),
            p(5.0, 0.0, 0.0),
            p(5.0, 0.0, 3.0),
            p(2.0, 0.0, 3.0),
        ];
        let mut section = ngk::modeling::faces::polygon(&section)?;
        section
            .model_mut()
            .set_commit_checks(ngk::model::CommitChecks {
                solid_orientation: true,
            });
        match ngk::modeling::revolve::revolve_face(section, z_axis(), deg(-90.0)) {
            Ok(solid) => valid(&solid),
            Err(_) => Ok(()),
        }
    })
}
