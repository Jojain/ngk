use std::convert::Infallible;

use ngk::builders::faces::{FaceImprint, add_face, split_face_by_imprints_staged};
use ngk::builders::profiles::add_rectangle as add_rectangle_profile;
use ngk::geometry::{Curve, Plane, Point2, Point3, TrimmedCurve2};
use ngk::model::Model;
use ngk::topology::ModelEditError;
use ngk::topology::edit::{EditKey, EditPolicy, Origin};
use ngk::topology::payload::Payload;
use ngk::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey};

#[derive(Clone, Default)]
struct FacePayload;

impl Payload for FacePayload {
    type V = ();
    type E = ();
    type Profile = String;
    type F = String;
    type Sheet = ();
    type S = ();
}

#[derive(Default)]
struct RecordFaceSplits {
    splits: Vec<(FaceKey, FaceKey)>,
    profile_splits: Vec<(ProfileKey, ProfileKey)>,
}

impl EditPolicy<FacePayload> for RecordFaceSplits {
    type Error = Infallible;

    fn vertex_created(
        &mut self,
        _key: VertexKey,
        _origin: Origin,
        _before: &Model<FacePayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn edge_created(
        &mut self,
        _key: EdgeKey,
        _origin: Origin,
        _before: &Model<FacePayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn profile_created(
        &mut self,
        key: ProfileKey,
        origin: Origin,
        before: &Model<FacePayload>,
    ) -> Result<String, Self::Error> {
        match origin {
            Origin::Split(EditKey::Profile(source)) => {
                self.profile_splits.push((source, key));
                Ok(format!(
                    "{}:split",
                    before.profile_attr_unchecked(source).data()
                ))
            }
            _ => Ok(String::new()),
        }
    }

    fn face_created(
        &mut self,
        key: FaceKey,
        origin: Origin,
        before: &Model<FacePayload>,
    ) -> Result<String, Self::Error> {
        match origin {
            Origin::Split(EditKey::Face(source)) => {
                self.splits.push((source, key));
                Ok(format!(
                    "{}:split",
                    before.face_attr_unchecked(source).data()
                ))
            }
            _ => Ok(String::new()),
        }
    }

    fn sheet_created(
        &mut self,
        _key: SheetKey,
        _origin: Origin,
        _before: &Model<FacePayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn solid_created(
        &mut self,
        _key: SolidKey,
        _origin: Origin,
        _before: &Model<FacePayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[test]
fn boundary_chord_split_preserves_source_face_and_applies_payload_policy() {
    let mut g = attributed_rectangle();
    let source = g.iter_faces().next().expect("face should exist").0;
    let source_profile = g
        .profile_key(g.face_unchecked(source).dart())
        .expect("source face should have a profile");
    let imprint = planar_line_imprint(Point2::new(0.0, 0.0), Point2::new(2.0, 2.0));
    let mut policy = RecordFaceSplits::default();

    let splits = g
        .transaction_with_policy(&mut policy, |edit| {
            split_face_by_imprints_staged(edit, source, &[imprint])
        })
        .expect("face imprint split should commit");

    assert_eq!(splits.len(), 1);
    assert_eq!(splits[0].first, source);
    assert_eq!(g.face_attr_unchecked(source).data(), "source");
    assert_eq!(
        g.face_attr_unchecked(splits[0].second).data(),
        "source:split"
    );
    assert_eq!(policy.splits, vec![(source, splits[0].second)]);
    assert_eq!(policy.profile_splits.len(), 1);
    assert_eq!(policy.profile_splits[0].0, source_profile);
    assert_eq!(
        g.profile_attr_unchecked(policy.profile_splits[0].1).data(),
        "source profile:split"
    );
}

#[test]
fn closed_loop_split_declares_the_island_as_a_source_face_split() {
    let mut g = attributed_rectangle();
    let source = g.iter_faces().next().expect("face should exist").0;
    let points = [
        Point2::new(0.5, 0.5),
        Point2::new(1.5, 0.5),
        Point2::new(1.5, 1.5),
        Point2::new(0.5, 1.5),
        Point2::new(0.5, 0.5),
    ];
    let imprints = points
        .windows(2)
        .map(|pair| planar_line_imprint(pair[0], pair[1]))
        .collect::<Vec<_>>();
    let mut policy = RecordFaceSplits::default();

    let splits = g
        .transaction_with_policy(&mut policy, |edit| {
            split_face_by_imprints_staged(edit, source, &imprints)
        })
        .expect("closed face imprint split should commit");

    assert_eq!(splits.len(), 1);
    assert_eq!(splits[0].first, source);
    assert_eq!(g.face_attr_unchecked(source).data(), "source");
    assert_eq!(
        g.face_attr_unchecked(splits[0].second).data(),
        "source:split"
    );
    assert_eq!(policy.splits, vec![(source, splits[0].second)]);
}

#[test]
fn late_face_policy_failure_restores_the_complete_source_face() {
    let mut g = attributed_rectangle();
    let source = g.iter_faces().next().expect("face should exist").0;
    let imprint = planar_line_imprint(Point2::new(0.0, 0.0), Point2::new(2.0, 2.0));
    let original_dart_count = g.dart_count();
    let mut policy = RejectFaceSplit;

    let result = g.transaction_with_policy(&mut policy, |edit| {
        split_face_by_imprints_staged(edit, source, &[imprint])
    });

    assert!(result.is_err());
    assert_eq!(g.dart_count(), original_dart_count);
    assert_eq!(g.iter_faces().count(), 1);
    assert_eq!(g.iter_edges().count(), 4);
    assert_eq!(g.face_attr_unchecked(source).data(), "source");
    assert_eq!(
        g.face_unchecked(source)
            .outer_loop()
            .expect("face should have an outer loop")
            .edges()
            .len(),
        4
    );
}

struct RejectFaceSplit;

impl EditPolicy<FacePayload> for RejectFaceSplit {
    type Error = std::io::Error;

    fn vertex_created(
        &mut self,
        _key: VertexKey,
        _origin: Origin,
        _before: &Model<FacePayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn edge_created(
        &mut self,
        _key: EdgeKey,
        _origin: Origin,
        _before: &Model<FacePayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn profile_created(
        &mut self,
        _key: ProfileKey,
        _origin: Origin,
        _before: &Model<FacePayload>,
    ) -> Result<String, Self::Error> {
        Ok(String::new())
    }

    fn face_created(
        &mut self,
        _key: FaceKey,
        _origin: Origin,
        _before: &Model<FacePayload>,
    ) -> Result<String, Self::Error> {
        Err(std::io::Error::other("reject face split"))
    }

    fn sheet_created(
        &mut self,
        _key: SheetKey,
        _origin: Origin,
        _before: &Model<FacePayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn solid_created(
        &mut self,
        _key: SolidKey,
        _origin: Origin,
        _before: &Model<FacePayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }
}

fn attributed_rectangle() -> Model<FacePayload> {
    let mut g = Model::new();
    let profile = add_rectangle_profile(&mut g, Plane::xy(), 2.0, 2.0)
        .expect("rectangle profile should build");
    let face = add_face(&mut g, profile).expect("rectangle face should build");
    g.transaction(|edit| {
        *edit.profile_attr_mut_unchecked(profile).data_mut() = "source profile".to_owned();
        *edit.face_attr_mut_unchecked(face).data_mut() = "source".to_owned();
        Ok::<_, ModelEditError>(())
    })
    .unwrap();
    g
}

fn planar_line_imprint(start: Point2, end: Point2) -> FaceImprint {
    FaceImprint::new(
        Curve::line(
            Point3::new(start.x, start.y, 0.0),
            Point3::new(end.x, end.y, 0.0),
        ),
        TrimmedCurve2::segment(start, end),
    )
}
