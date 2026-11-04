//! The common part of two spherical sectors of a ball of radius 10 about the origin, each within
//! 45° of its axis: the first about +z, the second the first turned 60° about y. A sector is its
//! half section, the triangle-and-arc from the apex out the ruling to the sphere and back down
//! the axis, turned a full turn about its axis.
use crate::bench::{common, deg, face_from_edges, line, p, plane_xy, revolve_about_z, v};
use crate::{Refusal, Solid};
use ngk::geometry::Axis3;
use ngk::modeling::edges;

fn sector() -> Result<Solid, Refusal> {
    let r = 10.0 / 2f64.sqrt();
    let ruling = line(p(0.0, 0.0, 0.0), p(r, 0.0, r))?;
    let rim = edges::arc(plane_xy(p(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.0, 0.0, 1.0)), 10.0, deg(45.0), deg(90.0))?;
    let shaft = line(p(0.0, 0.0, 10.0), p(0.0, 0.0, 0.0))?;
    revolve_about_z(face_from_edges(&[&ruling, &rim, &shaft])?, 360.0)
}

pub fn build() -> Result<Solid, Refusal> {
    let turned = sector()?.rotated(Axis3::y(), deg(60.0));
    common(sector()?, &turned)
}
