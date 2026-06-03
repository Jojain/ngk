use ngk::scripts::boolean_configs::{BooleanConfig, build};

#[test]
fn boolean_configurations_build_visualizable_results() {
    for config in [
        BooleanConfig::UnionOverlap,
        BooleanConfig::IntersectionOverlap,
        BooleanConfig::DifferenceOverlap,
    ] {
        let result = build(config).expect("boolean configuration should build");

        assert!(!result.scene.faces.is_empty());
        assert!(!result.scene.edges.is_empty());
        assert!(result.gmap.is_some());
    }
}
