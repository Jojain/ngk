//! Sheets of planar quads, the usual tool to split along.

use ngk::geometry::Point3;
use ngk::modeling::{faces, sheets};
use ngk::topology::shape::{Shape, SheetTag};

/// A sheet of planar quads, each listed counter-clockwise seen from the side
/// its normal points to.
pub fn quad_sheet(quads: &[[(f64, f64, f64); 4]]) -> Shape<SheetTag> {
    let faces = quads
        .iter()
        .map(|quad| {
            let points = quad
                .iter()
                .map(|&(x, y, z)| Point3::new(x, y, z))
                .collect::<Vec<_>>();
            faces::polygon(&points).expect("a quad should build")
        })
        .collect::<Vec<_>>();
    sheets::from_faces(&faces.iter().collect::<Vec<_>>()).expect("quads join")
}
