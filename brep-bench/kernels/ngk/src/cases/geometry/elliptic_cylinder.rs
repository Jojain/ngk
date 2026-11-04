//! The ellipse of semi-axes 5 along x and 3 along y about the origin in z = 0, extruded to z = 10.
use crate::bench::{ellipse_face, extrude, p, v};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    extrude(ellipse_face(p(0.0, 0.0, 0.0), 5.0, 3.0)?, v(0.0, 0.0, 10.0))
}
