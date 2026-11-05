use ngk::geometry::{Curve, Point3};
use ngk::model::{Cell1, Model};
use ngk::topology::StandardPayload;
use ngk::topology::attributes::{EdgeAttr, VertexAttr};
use ngk::topology::edit::ModelEditError;
use ngk::topology::gmap::Dim;
#[test]
fn typed_views_resolve_transaction_local_cells_between_passes() {
    let mut g = Model::<StandardPayload>::new();

    g.transaction(|edit| {
        let start = edit.deferred(|raw| {
            let start = raw.add_dart();
            let end = raw.add_dart();
            raw.link(Dim::Zero, start, end)?;
            raw.add_vertex(VertexAttr::new(start, Point3::origin()));
            raw.add_vertex(VertexAttr::new(end, Point3::new(1.0, 0.0, 0.0)));
            raw.add_edge(EdgeAttr::new(
                start,
                Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
            ));

            Ok::<_, ModelEditError>(start)
        })?;
        let key = edit
            .cell_key::<Cell1>(start)
            .expect("the staged edge should be available through typed lookup");
        assert_eq!(edit.edge_unchecked(key).dart(), start);
        Ok::<_, ModelEditError>(())
    })
    .expect("transaction should commit");
}
