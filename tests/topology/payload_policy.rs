//! A payload that names its own maintenance policy.
//!
//! The point of `Payload::Policy` is that a caller writes the policy once, on
//! the type, and every builder in the kernel then runs it with nothing said at
//! any call site — including for a payload that has no `Default` to fall back
//! on.

use std::convert::Infallible;

use ngk::builders::faces::add_face;
use ngk::builders::profiles::add_polyline;
use ngk::geometry::Point3;
use ngk::model::Model;
use ngk::topology::edit::{EditKey, EditPolicy, Origin};
use ngk::topology::payload::Payload;
use ngk::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey};

/// Carries no `Default` at any dimension, so it cannot go through
/// `PreservePayload` at all.
#[derive(Clone, Debug, PartialEq, Eq)]
struct Tag(&'static str);

#[derive(Clone)]
struct Tagged;

impl Payload for Tagged {
    type V = Tag;
    type E = Tag;
    type Profile = Tag;
    type F = Tag;
    type Sheet = Tag;
    type S = Tag;

    type Policy = TagEverything;
}

/// Stamps what each entity is, and clones a split source's tag.
#[derive(Default)]
struct TagEverything;

impl EditPolicy<Tagged> for TagEverything {
    type Error = Infallible;

    fn vertex_created(
        &mut self,
        _key: VertexKey,
        _origin: Origin,
        _before: &Model<Tagged>,
    ) -> Result<Tag, Infallible> {
        Ok(Tag("vertex"))
    }

    fn edge_created(
        &mut self,
        _key: EdgeKey,
        _origin: Origin,
        _before: &Model<Tagged>,
    ) -> Result<Tag, Infallible> {
        Ok(Tag("edge"))
    }

    fn profile_created(
        &mut self,
        _key: ProfileKey,
        _origin: Origin,
        _before: &Model<Tagged>,
    ) -> Result<Tag, Infallible> {
        Ok(Tag("profile"))
    }

    fn face_created(
        &mut self,
        _key: FaceKey,
        origin: Origin,
        before: &Model<Tagged>,
    ) -> Result<Tag, Infallible> {
        Ok(match origin {
            Origin::Split(EditKey::Face(source)) => {
                before.face_attr_unchecked(source).data().clone()
            }
            _ => Tag("face"),
        })
    }

    fn sheet_created(
        &mut self,
        _key: SheetKey,
        _origin: Origin,
        _before: &Model<Tagged>,
    ) -> Result<Tag, Infallible> {
        Ok(Tag("sheet"))
    }

    fn solid_created(
        &mut self,
        _key: SolidKey,
        _origin: Origin,
        _before: &Model<Tagged>,
    ) -> Result<Tag, Infallible> {
        Ok(Tag("solid"))
    }
}

fn square() -> [Point3; 5] {
    [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.0, 0.0, 0.0),
    ]
}

#[test]
fn an_ordinary_builder_runs_the_payloads_own_policy() {
    let mut model = Model::<Tagged>::new();

    // No policy is named here. `add_polyline` and `add_face` are the same
    // builders `StandardPayload` uses; they open `Model::transaction`, which
    // resolves `Tagged::Policy` on its own.
    let profile = add_polyline(&mut model, &square()).unwrap();
    let face = add_face(&mut model, profile).unwrap();

    assert_eq!(model.face_attr_unchecked(face).data(), &Tag("face"));
    assert_eq!(
        model.profile_attr_unchecked(profile).data(),
        &Tag("profile")
    );
    assert!(
        model
            .iter_edges()
            .all(|(_, attr)| attr.data() == &Tag("edge"))
    );
    assert!(
        model
            .iter_vertices()
            .all(|(_, attr)| attr.data() == &Tag("vertex"))
    );
}

#[test]
fn a_payload_without_default_needs_no_transaction_of_its_own() {
    // The whole point: `Tag` has no `Default`, so before `Payload::Policy` this
    // model could not reach a builder at all — every one of them was bounded
    // `P: DefaultPayload`.
    let mut model = Model::<Tagged>::new();
    let profile = add_polyline(&mut model, &square()).unwrap();

    assert_eq!(
        model.profile_attr_unchecked(profile).data(),
        &Tag("profile")
    );
    assert_eq!(model.revision(), 1, "one builder is one transaction");
}
