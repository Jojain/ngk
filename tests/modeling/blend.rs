use ngk::modeling::blend::{chamfered, filleted};
use ngk::modeling::solids::block;

#[test]
fn owned_solid_blends_preserve_the_source_solid_identity() {
    let fillet_source = block(2.0, 2.0, 2.0).expect("block should build");
    let fillet_key = fillet_source.key();
    let edge = fillet_source.solid().edges()[0].key();
    let filleted = filleted(fillet_source, vec![edge], 0.1).expect("fillet should build");
    assert_eq!(filleted.key(), fillet_key);
    assert!(filleted.solid().faces().len() > 6);

    let chamfer_source = block(2.0, 2.0, 2.0).expect("block should build");
    let chamfer_key = chamfer_source.key();
    let edge = chamfer_source.solid().edges()[0].key();
    let chamfered = chamfered(chamfer_source, vec![edge], 0.1).expect("chamfer should build");
    assert_eq!(chamfered.key(), chamfer_key);
    assert!(chamfered.solid().faces().len() > 6);
}
