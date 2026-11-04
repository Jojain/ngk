//! The horn torus about z of radii 5 and 5: the disc (x − 5)² + z² ≤ 25 of the xz plane turned a
//! whole turn about z, which it touches at the origin.
use crate::bench::{p, plane_xy, revolve_about_z, v};
use crate::{Refusal, Solid};
use ngk::modeling::faces;

pub fn build() -> Result<Solid, Refusal> {
    let section = faces::circle(plane_xy(p(5.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.0, 0.0, 1.0)), 5.0)?;
    revolve_about_z(section, 360.0)
}
