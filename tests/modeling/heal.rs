use ngk::healing::HealingOptions;
use ngk::modeling::{heal, solids};

#[test]
fn healing_owned_solid_preserves_its_handle_and_returns_report() {
    let source = solids::block(2.0, 3.0, 4.0).expect("block");
    let key = source.key();
    let result = heal::solid(source, HealingOptions::default()).expect("heal");
    assert_eq!(result.shape.key(), key);
    assert!(result.shape.model().solid(key).is_some());
    assert!(result.report.iterations >= 1);
}
