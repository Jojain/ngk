use std::sync::Arc;

use ngk::geometry::{Plane, Point3, Surface};
use ngk::model::{Model, RealizationError, RealizationPurpose};
use ngk::modeling::{edges, faces};
use ngk::topology::gmap::{Dart, Dim};
use ngk::topology::shape_keys::{EdgeKey, FaceKey};
use ngk::topology::embedding::EntityOwner;
use ngk::topology::{ModelEditError, Orientation, StandardPayload, UnwrappedFaceDomain};

#[test]
fn repeated_face_requests_share_an_immutable_realization() {
    let shape = faces::annulus(Plane::xy(), 2.0, 1.0).unwrap();
    let face = shape.face();
    let model = shape.model();
    let cold = model
        .realize_face(face.key(), Orientation::Same, RealizationPurpose::Geometry)
        .unwrap();
    let warm = model
        .realize_face(face.key(), Orientation::Same, RealizationPurpose::Geometry)
        .unwrap();
    assert!(Arc::ptr_eq(&cold, &warm));
    assert_eq!(cold.domain(), &UnwrappedFaceDomain::of_face(&face).unwrap());
    assert_eq!(cold.domain().loops().len(), 2);
    let reversed = model
        .realize_face(
            face.key(),
            Orientation::Reversed,
            RealizationPurpose::Geometry,
        )
        .unwrap();
    assert_eq!(
        reversed.domain(),
        &UnwrappedFaceDomain::of_face(&face.reversed()).unwrap()
    );
    assert!(!Arc::ptr_eq(&cold, &reversed));
}

#[test]
fn reversed_edge_realizations_keep_the_same_native_section() {
    let shape = edges::arc(Plane::xy(), 2.0, 0.3, 5.4).unwrap();
    let key = shape.edge().key();
    let model = shape.model();
    let forward = model
        .realize_edge(key, Orientation::Same, RealizationPurpose::Geometry)
        .unwrap();
    let backward = model
        .realize_edge(key, Orientation::Reversed, RealizationPurpose::Geometry)
        .unwrap();
    for t in [0.0, 0.2, 0.5, 0.8, 1.0] {
        assert!((forward.point_at(t) - backward.point_at(1.0 - t)).norm() < 1e-12);
    }
    let another_purpose = model
        .realize_edge(key, Orientation::Same, RealizationPurpose::Exchange)
        .unwrap();
    assert!(!Arc::ptr_eq(&forward, &another_purpose));
    assert_eq!(*forward, *another_purpose);
}

#[test]
fn staged_geometry_changes_invalidate_reads_before_revision_advances() {
    let shape = edges::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)).unwrap();
    let (mut model, key) = shape.into_model();
    let endpoint = model.edge(key).unwrap().bounded_unchecked().end().key();
    let old = model
        .realize_edge(key, Orientation::Same, RealizationPurpose::Geometry)
        .unwrap();
    let revision = model.revision();
    model
        .transaction(|edit| {
            edit.vertex_attr_mut_unchecked(endpoint).point.x = 2.0;
            let staged = edit
                .model()
                .realize_edge(key, Orientation::Same, RealizationPurpose::Geometry)
                .unwrap();
            assert_eq!(edit.model().revision(), revision);
            assert_eq!(staged.point_at(1.0), Point3::new(2.0, 0.0, 0.0));
            edit.vertex_attr_mut_unchecked(endpoint).point.x = 3.0;
            let staged = edit
                .model()
                .realize_edge(key, Orientation::Same, RealizationPurpose::Geometry)
                .unwrap();
            assert_eq!(staged.point_at(1.0), Point3::new(3.0, 0.0, 0.0));
            Ok::<_, ModelEditError>(())
        })
        .unwrap();
    let current = model
        .realize_edge(key, Orientation::Same, RealizationPurpose::Geometry)
        .unwrap();
    assert_eq!(model.revision(), revision + 1);
    assert_eq!(current.point_at(1.0), Point3::new(3.0, 0.0, 0.0));
    assert_eq!(old.point_at(1.0), Point3::new(1.0, 0.0, 0.0));
}

#[test]
fn rollback_discards_realizations_of_aborted_geometry() {
    let shape = edges::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)).unwrap();
    let (mut model, key) = shape.into_model();
    let endpoint = model.edge(key).unwrap().bounded_unchecked().end().key();
    let before = model
        .realize_edge(key, Orientation::Same, RealizationPurpose::Geometry)
        .unwrap();
    let revision = model.revision();
    let result = model.transaction(|edit| {
        edit.vertex_attr_mut_unchecked(endpoint).point.x = 4.0;
        let staged = edit
            .model()
            .realize_edge(key, Orientation::Same, RealizationPurpose::Geometry)
            .unwrap();
        assert_eq!(staged.point_at(1.0), Point3::new(4.0, 0.0, 0.0));
        Err::<(), _>(ModelEditError::SameDart {
            dart: edit.model().edge_attr_unchecked(key).dart,
        })
    });
    assert!(result.is_err());
    assert_eq!(model.revision(), revision);
    let restored = model
        .realize_edge(key, Orientation::Same, RealizationPurpose::Geometry)
        .unwrap();
    assert_eq!(*before, *restored);
}

#[test]
fn serialization_and_clone_rebuild_realizations_from_authoritative_geometry() {
    let shape = faces::annulus(Plane::xy(), 2.0, 1.0).unwrap();
    let model = shape.model();
    let key = shape.face().key();
    let cold_json = serde_json::to_value(model).unwrap();
    let warm = model
        .realize_face(key, Orientation::Same, RealizationPurpose::Geometry)
        .unwrap();
    assert_eq!(cold_json, serde_json::to_value(model).unwrap());
    let loaded: Model<StandardPayload> = serde_json::from_value(cold_json).unwrap();
    for other in [loaded, model.clone()] {
        let rebuilt = other
            .realize_face(key, Orientation::Same, RealizationPurpose::Geometry)
            .unwrap();
        assert!(!Arc::ptr_eq(&warm, &rebuilt));
        assert_eq!(warm.domain(), rebuilt.domain());
    }
}

#[test]
fn concurrent_cold_reads_share_one_published_result() {
    let shape = faces::annulus(Plane::xy(), 2.0, 1.0).unwrap();
    let key = shape.face().key();
    let model = shape.model().clone();
    let barrier = std::sync::Barrier::new(8);
    std::thread::scope(|scope| {
        let barrier = &barrier;
        let model = &model;
        let readers: Vec<_> = (0..8)
            .map(|_| {
                scope.spawn(move || {
                    barrier.wait();
                    model
                        .realize_face(key, Orientation::Same, RealizationPurpose::Geometry)
                        .unwrap()
                })
            })
            .collect();
        let values: Vec<_> = readers
            .into_iter()
            .map(|reader| reader.join().unwrap())
            .collect();
        for value in &values[1..] {
            assert!(Arc::ptr_eq(&values[0], value));
        }
    });
}

#[test]
fn commit_validation_failure_discards_staged_realizations() {
    let shape = edges::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)).unwrap();
    let (mut model, key) = shape.into_model();
    let endpoint = model.edge(key).unwrap().bounded_unchecked().end().key();
    let before = model
        .realize_edge(key, Orientation::Same, RealizationPurpose::Geometry)
        .unwrap();
    let revision = model.revision();
    let result = model.transaction(|edit| {
        edit.vertex_attr_mut_unchecked(endpoint).point.x = 4.0;
        let staged = edit
            .model()
            .realize_edge(key, Orientation::Same, RealizationPurpose::Geometry)
            .unwrap();
        assert_eq!(staged.point_at(1.0), Point3::new(4.0, 0.0, 0.0));
        let missing = Dart::new(edit.model().dart_count());
        edit.own_cell(Dim::One, missing, EntityOwner::Edge(key));
        Ok::<_, ModelEditError>(())
    });
    assert!(matches!(result, Err(ModelEditError::InvalidEmbedding(_))));
    assert_eq!(model.revision(), revision);
    let restored = model
        .realize_edge(key, Orientation::Same, RealizationPurpose::Geometry)
        .unwrap();
    assert_eq!(*before, *restored);
}

#[test]
fn held_face_realizations_keep_their_original_support_after_an_edit() {
    let shape = faces::rectangle(Plane::xy(), 2.0, 3.0).unwrap();
    let (mut model, key) = shape.into_model();
    let before = model
        .realize_face(key, Orientation::Same, RealizationPurpose::Geometry)
        .unwrap();
    model
        .transaction(|edit| {
            edit.face_attr_mut_unchecked(key).surface = Surface::Plane(Plane::yz());
            Ok::<_, ModelEditError>(())
        })
        .unwrap();
    let after = model
        .realize_face(key, Orientation::Same, RealizationPurpose::Geometry)
        .unwrap();
    assert_eq!(before.surface(), &Surface::Plane(Plane::xy()));
    assert_eq!(after.surface(), &Surface::Plane(Plane::yz()));
    assert!(!Arc::ptr_eq(&before, &after));
}

#[test]
fn missing_entities_report_errors_without_publishing_realizations() {
    let model = Model::<StandardPayload>::new();
    assert!(matches!(
        model.realize_edge(
            EdgeKey::default(),
            Orientation::Same,
            RealizationPurpose::Geometry
        ),
        Err(RealizationError::MissingEdge(_))
    ));
    assert!(matches!(
        model.realize_face(
            FaceKey::default(),
            Orientation::Same,
            RealizationPurpose::Geometry
        ),
        Err(RealizationError::MissingFace(_))
    ));
}
