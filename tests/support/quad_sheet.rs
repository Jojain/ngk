//! Sheets and faces to split along and to split.

use ngk::geometry::Point3;
use ngk::model::{Cell2, Model};
use ngk::modeling::{faces, sheets, solids};
use ngk::topology::shape::{FaceTag, Shape, SheetTag};

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

/// The bottom face of the unit block at the origin, on its own.
///
/// It faces down, out of the block, while its support plane faces up: the
/// one face here whose own normal and its support's disagree.
pub fn reversed_bottom_face() -> Shape<FaceTag> {
    let block = solids::block(1.0, 1.0, 1.0).expect("the block should build");
    let bottom = block
        .solid()
        .faces()
        .into_iter()
        .find(|face| face.normal_at(0.0, 0.0).z < -0.5)
        .expect("the block should have a bottom face");
    assert!(
        bottom.surface().normal_at(0.0, 0.0).z > 0.5,
        "the bottom face's support should face up"
    );
    let (model, dart) = Model::isolate(bottom);
    let key = model.cell_key_unchecked::<Cell2>(dart);
    Shape::new(model, key)
}
