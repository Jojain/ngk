use std::collections::HashMap;

use ngk::geometry::LINEAR_TOLERANCE;
use ngk::modeling::solids::block;
use ngk::topology::gmap::Dim;
use ngk::topology::sheet::Sheet;

#[test]
fn opposite_sheet_roots_reverse_all_face_normals() {
    let shape = block(1.0, 2.0, 3.0).expect("block should build");
    let g = shape.map();
    let shell = shape.solid().outer_shell();
    let same = Sheet::from_dart(g, shell.dart).expect("shell should have a registered sheet");
    let reversed = Sheet::from_dart(g, g.alpha(Dim::Zero, shell.dart))
        .expect("opposite shell root should resolve the same sheet");
    let same_normals = same
        .faces()
        .into_iter()
        .map(|face| (face.key(), face.normal_at(0.0, 0.0)))
        .collect::<HashMap<_, _>>();
    let reversed_normals = reversed
        .faces()
        .into_iter()
        .map(|face| (face.key(), face.normal_at(0.0, 0.0)))
        .collect::<HashMap<_, _>>();

    assert_eq!(same_normals.len(), reversed_normals.len());
    for (key, normal) in same_normals {
        assert!(
            normal.dot(&reversed_normals[&key]) < -1.0 + LINEAR_TOLERANCE,
            "choosing the opposite sheet root should reverse face {key:?}'s normal"
        );
    }
}
