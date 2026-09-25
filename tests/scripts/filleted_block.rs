use ngk::scripts::filleted_block::build;

#[test]
fn filleted_block_builds_across_its_radius_range() {
    for radius in [0.05, 0.2, 0.35] {
        let result = build(radius).unwrap_or_else(|err| panic!("radius {radius}: {err}"));

        assert!(!result.scene.faces.is_empty());
        assert!(!result.scene.edges.is_empty());
        assert!(result.gmap.is_some());
    }
}
