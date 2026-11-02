use ngk::builders::boolean::{BooleanError, BooleanOptions, split_face};
use ngk::builders::errors::SheetFromFacesError;
use ngk::builders::faces::FaceImprintSplitError;
use ngk::geometry::{LINEAR_TOLERANCE, Point3};
use ngk::model::{Cell2, Model};
use ngk::modeling::{faces, sheets, solids};
use ngk::topology::closed::Closeable;
use ngk::topology::shape::{FaceTag, Shape, SheetTag};

use crate::cube_faces::{Winding, cube_faces, refs};
use crate::quad_sheet::quad_sheet;

fn square(min: (f64, f64), size: f64) -> Shape<FaceTag> {
    let (x, y) = min;
    faces::polygon(&[
        Point3::new(x, y, 0.0),
        Point3::new(x + size, y, 0.0),
        Point3::new(x + size, y + size, 0.0),
        Point3::new(x, y + size, 0.0),
    ])
    .expect("a square should build")
}

#[test]
fn from_faces_makes_one_face_a_sheet() {
    let face = square((0.0, 0.0), 1.0);

    let sheet = sheets::from_faces(&[&face]).expect("one face is a sheet");

    assert_eq!(sheet.sheet().faces().len(), 1);
    assert!(!sheet.sheet().is_closed());
    assert_eq!(face.model().iter_sheets().count(), 0);
}

#[test]
fn from_faces_joins_neighbours_along_their_shared_edge() {
    let left = square((0.0, 0.0), 1.0);
    let right = square((1.0, 0.0), 1.0);

    let sheet = sheets::from_faces(&[&right, &left]).expect("neighbours should join");

    let view = sheet.sheet();
    assert_eq!(view.faces().len(), 2);
    assert_eq!(view.edges().len(), 7);
    assert_eq!(view.vertices().len(), 6);
    let area = view.area().expect("the sheet should measure");
    assert!((area - 2.0).abs() <= LINEAR_TOLERANCE);
}

#[test]
fn from_faces_closes_a_cube_into_a_shell() {
    let faces = cube_faces(Point3::origin(), 1.0, Winding::Outward);

    let sheet = sheets::from_faces(&refs(&faces)).expect("the cube faces should join");

    let view = sheet.sheet();
    assert!(view.is_closed());
    assert_eq!(view.faces().len(), 6);
    assert_eq!(view.edges().len(), 12);
    assert_eq!(view.vertices().len(), 8);
}

#[test]
fn from_faces_joins_the_curved_faces_of_a_cylinder() {
    let cylinder = solids::cylinder(1.0, 2.0).expect("the cylinder should build");
    let faces = cylinder
        .solid()
        .faces()
        .into_iter()
        .map(|face| {
            let (model, dart) = Model::isolate(face);
            let key = model.cell_key_unchecked::<Cell2>(dart);
            Shape::<FaceTag>::new(model, key)
        })
        .collect::<Vec<_>>();

    let sheet = sheets::from_faces(&refs(&faces)).expect("the cylinder faces should join");

    assert!(sheet.sheet().is_closed());
    assert_eq!(sheet.sheet().faces().len(), 3);
}

#[test]
fn from_faces_refuses_no_faces() {
    assert_eq!(
        sheets::from_faces::<ngk::StandardPayload>(&[]).err(),
        Some(SheetFromFacesError::NoFaces)
    );
}

#[test]
fn from_faces_refuses_faces_no_edge_joins() {
    let first = square((0.0, 0.0), 1.0);
    let second = square((3.0, 0.0), 1.0);

    assert_eq!(
        sheets::from_faces(&[&first, &second]).err(),
        Some(SheetFromFacesError::Disconnected { components: 2 })
    );
}

#[test]
fn from_faces_refuses_neighbours_wound_against_each_other() {
    let left = square((0.0, 0.0), 1.0);
    let right = faces::polygon(&[
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(2.0, 1.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
    ])
    .expect("a reversed square should build");

    assert!(matches!(
        sheets::from_faces(&[&left, &right]),
        Err(SheetFromFacesError::InconsistentOrientation { .. })
    ));
}

#[test]
fn from_faces_refuses_three_faces_on_one_edge() {
    let floor = square((0.0, 0.0), 1.0);
    let other = square((-1.0, 0.0), 1.0);
    let fin = faces::polygon(&[
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(1.0, 0.0, 1.0),
    ])
    .expect("the fin should build");
    let beyond = square((1.0, 0.0), 1.0);

    assert!(matches!(
        sheets::from_faces(&[&floor, &other, &fin, &beyond]),
        Err(SheetFromFacesError::NonManifoldEdge { .. })
    ));
}

/// Two unit squares side by side on z = 0, facing up, joined along x = 1.
fn strip() -> Shape<SheetTag> {
    let (left, right) = (square((0.0, 0.0), 1.0), square((1.0, 0.0), 1.0));
    sheets::from_faces(&[&left, &right]).expect("neighbours should join")
}

/// The plane x = `x` across the strip, facing `+x` or `-x`.
fn wall(x: f64, facing_plus_x: bool) -> Shape<SheetTag> {
    if facing_plus_x {
        quad_sheet(&[[
            (x, -1.0, -1.0),
            (x, 2.0, -1.0),
            (x, 2.0, 1.0),
            (x, -1.0, 1.0),
        ]])
    } else {
        quad_sheet(&[[
            (x, -1.0, -1.0),
            (x, -1.0, 1.0),
            (x, 2.0, 1.0),
            (x, 2.0, -1.0),
        ]])
    }
}

fn area_of(sheet: &Shape<SheetTag>) -> f64 {
    sheet.sheet().area().expect("the part should measure")
}

#[test]
fn split_cuts_a_sheet_along_a_crossing_plane() {
    let target = strip();

    let parts = sheets::split(&target, &wall(0.5, true)).expect("the plane should split the strip");

    assert!((area_of(&parts.front) - 1.5).abs() <= LINEAR_TOLERANCE);
    assert!((area_of(&parts.back) - 0.5).abs() <= LINEAR_TOLERANCE);
    // The square the plane missed is on the front only by being joined to
    // the half that is.
    assert_eq!(parts.front.sheet().faces().len(), 2);
    assert_eq!(parts.back.sheet().faces().len(), 1);
    assert!(!parts.front.sheet().is_closed());
    assert!((area_of(&target) - 2.0).abs() <= LINEAR_TOLERANCE);
}

#[test]
fn split_swaps_the_sheet_parts_with_the_tool_normal() {
    let parts = sheets::split(&strip(), &wall(0.5, false)).expect("the plane should split");

    assert!((area_of(&parts.front) - 0.5).abs() <= LINEAR_TOLERANCE);
    assert!((area_of(&parts.back) - 1.5).abs() <= LINEAR_TOLERANCE);
}

#[test]
fn split_refuses_a_tool_that_misses_the_sheet() {
    assert!(matches!(
        sheets::split(&strip(), &wall(5.0, true)),
        Err(BooleanError::SplitMissesTarget)
    ));
}

#[test]
fn split_refuses_a_tool_that_stops_inside_the_sheet() {
    let short = quad_sheet(&[[
        (0.5, -1.0, -1.0),
        (0.5, 0.5, -1.0),
        (0.5, 0.5, 1.0),
        (0.5, -1.0, 1.0),
    ]]);

    // The section is an open slit in the strip, which no face split makes.
    let result = sheets::split(&strip(), &short);

    assert!(
        matches!(
            &result,
            Err(BooleanError::Imprint { source, .. })
                if matches!(**source, BooleanError::FaceSplit(FaceImprintSplitError::ImprintNotRealized { .. }))
        ),
        "{:?}",
        result.err()
    );
}

#[test]
fn split_face_refuses_a_face_joined_into_a_sheet() {
    let target = strip();
    let mut model = target.model().clone();
    let face = target.sheet().faces()[0].key();

    let result = split_face(&mut model, face, target.key(), BooleanOptions::default());

    assert!(matches!(
        result,
        Err(BooleanError::SplitFaceNotLoose { face: refused }) if refused == face
    ));
}
