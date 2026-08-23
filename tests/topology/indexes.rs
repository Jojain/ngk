use ngk::geometry::{Curve, Point3};
use ngk::topology::StandardPayload;
use ngk::topology::attributes::{EdgeAttr, VertexAttr};
use ngk::topology::gmap::{Cell1, Dim, GMap, TopologyEditError};

#[test]
fn typed_views_resolve_transaction_local_cells_between_passes() {
    let mut g = GMap::<StandardPayload>::new();

    g.transaction(|g| {
        let start = g.add_dart();
        let end = g.add_dart();
        g.link(Dim::Zero, start, end)?;
        g.add_vertex(VertexAttr::new(start, Point3::origin(), ()));
        g.add_vertex(VertexAttr::new(end, Point3::new(1.0, 0.0, 0.0), ()));
        g.add_edge(EdgeAttr::new(
            start,
            Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
            (),
        ));

        let key = g
            .cell_key::<Cell1>(start)
            .expect("the staged edge should be available through typed lookup");
        assert_eq!(g.edge_unchecked(key).dart(), start);
        Ok::<_, TopologyEditError>(())
    })
    .expect("transaction should commit");
}
