use nalgebra::Vector3;
use ngk::scripts::interactive_extrusion::build;

#[test]
fn interactive_extrusion_builds_scene_from_parameters() {
    let result = build(6, Vector3::new(0.4, 0.1, 1.4)).unwrap();

    assert!(!result.scene.faces.is_empty());
    assert!(!result.scene.edges.is_empty());
    assert!(result.gmap.is_some());
}
