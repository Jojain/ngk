use ngk::modeling::solids::block;
use ngk::topology::facet::Facet;

#[test]
fn facet_loop_returns_its_profile_loop() {
    let shape = block(1.0, 2.0, 3.0).expect("block should build");
    let face = shape.solid().faces()[0].clone();
    let facet = Facet::new(shape.map(), face.outer_loop().dart);

    let loop_ = facet.r#loop();

    assert_eq!(loop_.edges().len(), 4);
    assert_eq!(loop_.vertices().len(), 4);
}

#[test]
fn facet_edges_match_its_boundary_loop_edges() {
    let shape = block(1.0, 2.0, 3.0).expect("block should build");
    let face = shape.solid().faces()[0].clone();
    let facet = Facet::new(shape.map(), face.outer_loop().dart);

    let facet_edges = facet
        .edges()
        .into_iter()
        .map(|edge| edge.key())
        .collect::<Vec<_>>();
    let face_edges = face
        .outer_loop()
        .edges()
        .into_iter()
        .map(|edge| edge.key())
        .collect::<Vec<_>>();

    assert_eq!(facet_edges, face_edges);
}
