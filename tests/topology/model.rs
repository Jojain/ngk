//! Copying topology from one model into another.
//!
//! A merge carries across the darts it was given, the alpha links between them,
//! and every entity whose own darts all came with them. What it must not do is
//! carry a reference to something left behind, which is what these check.

use std::collections::HashMap;

use nalgebra::Vector3;

use ngk::builders::edges::add_edge;
use ngk::builders::faces::add_polygon;
use ngk::builders::profiles::add_rectangle;
use ngk::builders::sheets::add_extruded_profile;
use ngk::geometry::{Curve, Plane, Point2, Point3, Surface, TrimmedCurve2};
use ngk::model::{Cell0, Cell1, Cell2, MergeTopology, Model};
use ngk::modeling::solids;
use ngk::topology::ModelEditError;
use ngk::topology::attributes::{FaceAttr, SheetAttr, SolidAttr};
use ngk::topology::gmap::{Dart, Dim};
use ngk::topology::payload::{Payload, StandardPayload};
use ngk::topology::planar::Planar;
use ngk::topology::profile::Profile;
use ngk::topology::sheet::Sheet;

#[derive(Clone)]
struct DataPayload;

impl Payload for DataPayload {
    type V = ();
    type E = ();
    type Profile = String;
    type F = ();
    type Sheet = String;
    type S = ();
}

#[test]
fn profile_and_sheet_payloads_are_exposed_and_preserved_by_merge() {
    let mut source = Model::<DataPayload>::new();
    let profile_key =
        add_rectangle(&mut source, Plane::xy(), 2.0, 1.0).expect("profile should build");
    source
        .transaction(|edit| {
            edit.profile_attr_mut_unchecked(profile_key).data = "profile".to_owned();
            Ok::<_, ModelEditError>(())
        })
        .unwrap();
    let sheet_key =
        add_extruded_profile(&mut source, profile_key, Vector3::z()).expect("sheet should build");
    source
        .transaction(|edit| {
            edit.sheet_attr_mut_unchecked(sheet_key).data = "sheet".to_owned();
            Ok::<_, ModelEditError>(())
        })
        .unwrap();

    assert_eq!(source.profile(profile_key).unwrap().data(), "profile");
    assert_eq!(source.sheet(sheet_key).unwrap().data(), "sheet");

    source
        .transaction(|edit| {
            edit.profile_attr_mut_unchecked(profile_key).data = "updated profile".to_owned();
            edit.sheet_attr_mut_unchecked(sheet_key).data = "updated sheet".to_owned();
            Ok::<_, ModelEditError>(())
        })
        .unwrap();

    let mut profile_target = Model::<DataPayload>::new();
    profile_target
        .transaction(|edit| {
            Ok::<_, ModelEditError>(edit.merge(source.profile(profile_key).unwrap()))
        })
        .unwrap();
    let mut sheet_target = Model::<DataPayload>::new();
    sheet_target
        .transaction(|edit| Ok::<_, ModelEditError>(edit.merge(source.sheet(sheet_key).unwrap())))
        .unwrap();

    assert_eq!(
        profile_target.iter_profiles().next().unwrap().1.data,
        "updated profile"
    );
    assert_eq!(
        sheet_target.iter_sheets().next().unwrap().1.data,
        "updated sheet"
    );
}

#[test]
fn merge_edge_copies_topology_and_geometry() {
    let mut target = Model::<StandardPayload>::new();
    let mut source = Model::<StandardPayload>::new();
    let edge_key = add_edge(
        &mut source,
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
    )
    .expect("source edge should build");

    let edge = source.edge_unchecked(edge_key);
    let merged_dart = target
        .transaction(|edit| Ok::<_, ModelEditError>(edit.merge(edge)))
        .unwrap();
    let merged_edge = target.attribute_unchecked::<Cell1>(merged_dart);

    assert_eq!(target.dart_count(), 2);
    assert_eq!(merged_edge.dart, Dart::new(0));
    assert_eq!(target.alpha(Dim::Zero, Dart::new(0)), Dart::new(1));
    assert!(target.attribute::<Cell0>(Dart::new(0)).is_some());
    assert!(target.attribute::<Cell0>(Dart::new(1)).is_some());
}

#[test]
fn merge_face_remaps_stored_darts_and_pcurves() {
    let mut target = Model::<StandardPayload>::new();
    add_edge(
        &mut target,
        Point3::new(-1.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 0.0),
        Curve::line(Point3::new(-1.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0)),
    )
    .expect("target edge should build");

    let mut source = Model::<StandardPayload>::new();
    let profile_key = add_polygon(
        &mut source,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
    );
    let loop_dart = source
        .profile_attr(profile_key)
        .expect("polygon profile should exist")
        .dart;
    let mut pcurves = HashMap::new();
    pcurves.insert(
        loop_dart,
        TrimmedCurve2::segment(Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)),
    );
    let face_key = source
        .transaction(|edit| {
            Ok::<_, ModelEditError>(edit.add_face(FaceAttr::with_pcurves(
                Surface::Plane(Plane::from_xy(
                    Point3::new(0.0, 0.0, 0.0),
                    Vector3::x(),
                    Vector3::y(),
                )),
                (),
                loop_dart,
                Vec::new(),
                pcurves,
            )))
        })
        .unwrap();

    let face = source.face_unchecked(face_key);
    let merged_dart = target
        .transaction(|edit| Ok::<_, ModelEditError>(edit.merge(face)))
        .unwrap();
    let merged_key = *target.attribute_unchecked::<Cell2>(merged_dart);
    let merged_face = target.face_unchecked(merged_key);
    let outer = merged_face
        .outer_loop()
        .expect("the merged face keeps its outer loop")
        .edges()[0]
        .dart();

    assert_eq!(target.dart_count(), 10);
    assert_eq!(outer, Dart::new(2));
    assert!(merged_face.pcurve(outer).is_some());
    assert!(merged_face.pcurve(loop_dart).is_none());
    // A copy takes the face's whole region, and the order that enumerates its
    // darts in is not the order the source's chain ran in, so the ids are
    // renumbered. What has to survive is the loop itself: the same edges, every
    // copied dart wired into it.
    assert_eq!(
        merged_face
            .outer_loop()
            .expect("the merged face keeps its outer loop")
            .edges()
            .len(),
        source
            .face_unchecked(face_key)
            .outer_loop()
            .expect("the source face has an outer loop")
            .edges()
            .len()
    );
    assert_eq!(target.alpha(Dim::Zero, outer), Dart::new(3));
    assert!(
        (2..target.dart_count()).all(|id| {
            let dart = Dart::new(id);
            !target.is_free(dart, Dim::Zero) && !target.is_free(dart, Dim::One)
        }),
        "every copied dart should be wired into the copied loop"
    );
}

#[test]
fn merge_profile_sheet_and_solid_return_remapped_darts() {
    let mut source = Model::<StandardPayload>::new();
    let profile_key = add_polygon(
        &mut source,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
    );

    let profile_dart = source.profile_attr_unchecked(profile_key).dart;
    let mut target = Model::<StandardPayload>::new();
    let merged_profile = target
        .transaction(|edit| Ok::<_, ModelEditError>(edit.merge(Profile::new(&source, profile_key))))
        .unwrap();
    assert_eq!(merged_profile, Dart::new(0));
    assert_eq!(target.dart_count(), 6);

    let sheet_key = source
        .transaction(|edit| {
            Ok::<_, ModelEditError>(edit.add_sheet(SheetAttr::new(profile_dart, ())))
        })
        .unwrap();
    let mut sheet_target = Model::<StandardPayload>::new();
    let merged_sheet = sheet_target
        .transaction(|edit| Ok::<_, ModelEditError>(edit.merge(Sheet::new(&source, sheet_key))))
        .unwrap();
    assert_eq!(merged_sheet, Dart::new(0));
    assert_eq!(sheet_target.dart_count(), 6);

    let solid_key = source
        .transaction(|edit| {
            Ok::<_, ModelEditError>(edit.add_solid(SolidAttr::new((), profile_dart, None)))
        })
        .unwrap();
    let mut second_target = Model::<StandardPayload>::new();
    let solid = source.solid_unchecked(solid_key);
    let merged_solid = second_target
        .transaction(|edit| Ok::<_, ModelEditError>(edit.merge(solid)))
        .unwrap();
    assert_eq!(merged_solid, Dart::new(0));
    assert_eq!(
        second_target
            .iter_solids()
            .next()
            .expect("merged solid should exist")
            .1
            .outer_shell,
        Dart::new(0)
    );
}

#[test]
fn isolate_face_copies_it_into_a_fresh_map() {
    let mut source = Model::<StandardPayload>::new();
    let profile_key = add_polygon(
        &mut source,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
    );
    let loop_dart = source.profile_attr_unchecked(profile_key).dart;
    let face_key = source
        .transaction(|edit| {
            Ok::<_, ModelEditError>(edit.add_face(FaceAttr::new(
                Surface::Plane(Plane::from_xy(
                    Point3::new(0.0, 0.0, 0.0),
                    Vector3::x(),
                    Vector3::y(),
                )),
                (),
                loop_dart,
                Vec::new(),
            )))
        })
        .unwrap();
    let face = source.face_unchecked(face_key);

    let (isolated, isolated_dart) = face.isolate();

    assert_eq!(isolated_dart, Dart::new(0));
    assert_eq!(isolated.dart_count(), 8);
    assert_eq!(isolated.iter_faces().count(), 1);
    assert!(isolated.attribute::<Cell2>(isolated_dart).is_some());
    assert_eq!(isolated.alpha(Dim::Zero, Dart::new(0)), Dart::new(1));
    // Isolating renumbers onto the face's region rather than its chain, so the
    // loop's shape is what carries over, not particular dart ids.
    assert!(
        (0..isolated.dart_count()).all(|id| {
            let dart = Dart::new(id);
            !isolated.is_free(dart, Dim::Zero) && !isolated.is_free(dart, Dim::One)
        }),
        "every isolated dart should be wired into the isolated loop"
    );
}

#[test]
fn isolate_associated_function_accepts_any_merge_topology() {
    let mut source = Model::<StandardPayload>::new();
    let profile_key = add_polygon(
        &mut source,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
    );

    let (isolated, isolated_dart) = Model::isolate(source.profile_unchecked(profile_key));

    assert_eq!(isolated_dart, Dart::new(0));
    assert_eq!(isolated.dart_count(), 6);
}

#[test]
fn isolate_planar_topology_forwards_to_inner_topology() {
    let mut source = Model::<StandardPayload>::new();
    let profile_key = add_polygon(
        &mut source,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
    );
    let planar = Planar::new_unchecked(
        source.profile_unchecked(profile_key),
        Plane::from_xy(Point3::new(0.0, 0.0, 0.0), Vector3::x(), Vector3::y()),
    );

    let (isolated, isolated_dart) = planar.isolate();

    assert_eq!(isolated_dart, Dart::new(0));
    assert_eq!(isolated.dart_count(), 6);
}

#[test]
fn isolating_an_edge_of_a_solid_leaves_the_faces_meeting_there_behind() {
    // An edge's darts run through the seeds of the faces on either side, so a
    // rule that copied a face on its seed alone pulled those faces along —
    // without their profiles, which commit then refused.
    let shape = solids::block(1.0, 2.0, 3.0).expect("block should build");
    let edge = shape
        .solid()
        .edges()
        .into_iter()
        .next()
        .expect("a block has edges");

    let (isolated, isolated_dart) = edge.isolate();

    assert!(isolated.attribute::<Cell1>(isolated_dart).is_some());
    assert_eq!(isolated.iter_edges().count(), 1);
    assert_eq!(isolated.iter_faces().count(), 0);
    assert_eq!(isolated.iter_profiles().count(), 0);
    assert_eq!(isolated.iter_solids().count(), 0);
}
