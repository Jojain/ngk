use std::convert::Infallible;

use ngk::builders::faces::{FaceImprint, add_face, split_face_by_imprints_staged};
use ngk::builders::profiles::add_rectangle as add_rectangle_profile;
use ngk::geometry::{Curve, Curve2, Line2, Plane, Point2, Point3};
use ngk::topology::TopologyEditError;
use ngk::topology::gmap::{EditPolicy, GMap};
use ngk::topology::payload::Payload;
use ngk::topology::shape_keys::{FaceKey, ProfileKey};

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

    fn split_face_data(
        &mut self,
        source: FaceKey,
        source_data: &String,
        created: FaceKey,
        created_data: &mut String,
    ) -> Result<(), Self::Error> {
        self.splits.push((source, created));
        *created_data = format!("{source_data}:split");
        Ok(())
    }

    fn split_profile_data(
        &mut self,
        source: ProfileKey,
        source_data: &String,
        created: ProfileKey,
        created_data: &mut String,
    ) -> Result<(), Self::Error> {
        self.profile_splits.push((source, created));
        *created_data = format!("{source_data}:split");
        Ok(())
    }
}

#[test]
fn boundary_chord_split_preserves_source_face_and_applies_payload_policy() {
    let mut g = attributed_rectangle();
    let source = g.iter_faces().next().expect("face should exist").0;
    let source_profile = g
        .profile_key(g.face_attr_unchecked(source).outer_loop)
        .expect("source face should have a profile");
    let imprint = planar_line_imprint(Point2::new(0.0, 0.0), Point2::new(2.0, 2.0));
    let mut policy = RecordFaceSplits::default();

    let splits = g
        .transaction_with_policy(&mut policy, |g| {
            split_face_by_imprints_staged(g, source, &[imprint])
        })
        .expect("face imprint split should commit");

    assert_eq!(splits.len(), 1);
    assert_eq!(splits[0].first, source);
    assert_eq!(g.face_attr_unchecked(source).data, "source");
    assert_eq!(g.face_attr_unchecked(splits[0].second).data, "source:split");
    assert_eq!(policy.splits, vec![(source, splits[0].second)]);
    assert_eq!(policy.profile_splits.len(), 1);
    assert_eq!(policy.profile_splits[0].0, source_profile);
    assert_eq!(
        g.profile_attr_unchecked(policy.profile_splits[0].1).data,
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
        .transaction_with_policy(&mut policy, |g| {
            split_face_by_imprints_staged(g, source, &imprints)
        })
        .expect("closed face imprint split should commit");

    assert_eq!(splits.len(), 1);
    assert_eq!(splits[0].first, source);
    assert_eq!(g.face_attr_unchecked(source).data, "source");
    assert_eq!(g.face_attr_unchecked(splits[0].second).data, "source:split");
    assert_eq!(policy.splits, vec![(source, splits[0].second)]);
}

#[test]
fn late_face_policy_failure_restores_the_complete_source_face() {
    let mut g = attributed_rectangle();
    let source = g.iter_faces().next().expect("face should exist").0;
    let imprint = planar_line_imprint(Point2::new(0.0, 0.0), Point2::new(2.0, 2.0));
    let original_dart_count = g.dart_count();
    let mut policy = RejectFaceSplit;

    let result = g.transaction_with_policy(&mut policy, |g| {
        split_face_by_imprints_staged(g, source, &[imprint])
    });

    assert!(result.is_err());
    assert_eq!(g.dart_count(), original_dart_count);
    assert_eq!(g.iter_faces().count(), 1);
    assert_eq!(g.iter_edges().count(), 4);
    assert_eq!(g.face_attr_unchecked(source).data, "source");
    assert_eq!(g.face_unchecked(source).outer_loop().edges().len(), 4);
}

struct RejectFaceSplit;

impl EditPolicy<FacePayload> for RejectFaceSplit {
    type Error = std::io::Error;

    fn split_face_data(
        &mut self,
        _source: FaceKey,
        _source_data: &String,
        _created: FaceKey,
        _created_data: &mut String,
    ) -> Result<(), Self::Error> {
        Err(std::io::Error::other("reject face split"))
    }
}

fn attributed_rectangle() -> GMap<FacePayload> {
    let mut g = GMap::new();
    let profile = add_rectangle_profile(&mut g, Plane::xy(), 2.0, 2.0)
        .expect("rectangle profile should build");
    let face = add_face(&mut g, profile).expect("rectangle face should build");
    g.transaction(|edit| {
        edit.profile_attr_mut_unchecked(profile).data = "source profile".to_owned();
        edit.face_attr_mut_unchecked(face).data = "source".to_owned();
        Ok::<_, TopologyEditError>(())
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
        Curve2::Line(Line2::new(start, end)),
    )
}
