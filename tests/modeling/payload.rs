use std::convert::Infallible;

use ngk::geometry::{Plane, Point3};
use ngk::model::Model;
use ngk::modeling::{edges, faces, solids};
use ngk::topology::edit::{EditPolicy, Origin};
use ngk::topology::payload::Payload;
use ngk::topology::shape::{EdgeTag, FaceTag, Shape, SolidTag};
use ngk::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey};

#[derive(Clone)]
struct ModelingPayload;

#[derive(Default)]
struct ModelingPolicy;

impl Payload for ModelingPayload {
    type V = ();
    type E = ();
    type Profile = ();
    type F = ();
    type Sheet = ();
    type S = ();
    type Policy = ModelingPolicy;
}

impl EditPolicy<ModelingPayload> for ModelingPolicy {
    type Error = Infallible;

    fn vertex_created(
        &mut self,
        _key: VertexKey,
        _origin: Origin,
        _before: &Model<ModelingPayload>,
    ) -> Result<(), Infallible> {
        Ok(())
    }

    fn edge_created(
        &mut self,
        _key: EdgeKey,
        _origin: Origin,
        _before: &Model<ModelingPayload>,
    ) -> Result<(), Infallible> {
        Ok(())
    }

    fn profile_created(
        &mut self,
        _key: ProfileKey,
        _origin: Origin,
        _before: &Model<ModelingPayload>,
    ) -> Result<(), Infallible> {
        Ok(())
    }

    fn face_created(
        &mut self,
        _key: FaceKey,
        _origin: Origin,
        _before: &Model<ModelingPayload>,
    ) -> Result<(), Infallible> {
        Ok(())
    }

    fn sheet_created(
        &mut self,
        _key: SheetKey,
        _origin: Origin,
        _before: &Model<ModelingPayload>,
    ) -> Result<(), Infallible> {
        Ok(())
    }

    fn solid_created(
        &mut self,
        _key: SolidKey,
        _origin: Origin,
        _before: &Model<ModelingPayload>,
    ) -> Result<(), Infallible> {
        Ok(())
    }
}

#[test]
fn constructors_accept_payloads_other_than_standard_payload() {
    let _: Shape<SolidTag, ModelingPayload> =
        solids::block_with::<ModelingPayload>(1.0, 2.0, 3.0).expect("block should build");
    let _: Shape<FaceTag, ModelingPayload> =
        faces::rectangle_with::<ModelingPayload>(Plane::xy(), 2.0, 3.0)
            .expect("rectangle should build");
    let _: Shape<EdgeTag, ModelingPayload> =
        edges::line_with::<ModelingPayload>(Point3::origin(), Point3::new(1.0, 0.0, 0.0))
            .expect("line should build");
}
