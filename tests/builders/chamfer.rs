use ngk::builders::chamfer::chamfer_profile_vertex;
use ngk::builders::errors::ChamferError;
use ngk::builders::profiles::add_polyline;
use ngk::geometry::Point3;
use ngk::topology::StandardPayload;
use ngk::topology::gmap::GMap;

#[test]
fn late_outer_failure_rolls_back_a_completed_chamfer_builder() {
    let mut g = GMap::<StandardPayload>::new();
    let profile = add_polyline(
        &mut g,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
    )
    .expect("profile should build");
    let corner = g.profile_unchecked(profile).edges()[0].end().dart;
    let before_darts = g.dart_count();
    let before_edges = g.iter_edges().count();
    let before_vertices = g.iter_vertices().count();

    let result = g.transaction(|g| {
        chamfer_profile_vertex(g, corner, 0.25)?;
        Err::<(), _>(ChamferError::InvalidDistance { distance: -1.0 })
    });

    assert!(
        matches!(result, Err(ChamferError::InvalidDistance { .. })),
        "unexpected result: {result:?}"
    );
    assert_eq!(g.dart_count(), before_darts);
    assert_eq!(g.iter_edges().count(), before_edges);
    assert_eq!(g.iter_vertices().count(), before_vertices);
}
