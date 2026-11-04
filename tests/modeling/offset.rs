use nalgebra::Vector3;
use ngk::geometry::Plane;
use ngk::modeling::offset::{offset, offset_along};
use ngk::modeling::profiles::rectangle;
use ngk::modeling::solids::block;

#[test]
fn a_shape_offset_is_its_moved_copy() {
    let grown = offset(block(2.0, 2.0, 2.0).unwrap(), 1.0).unwrap();
    let volume = grown.solid().volume().unwrap();
    assert!((volume - 64.0).abs() <= 1e-9, "volume {volume}");

    let loop_ = offset_along(
        rectangle(Plane::xy(), 2.0, 2.0).unwrap(),
        Vector3::z_axis(),
        -0.5,
    )
    .unwrap();
    let length = loop_.profile().length();
    assert!((length - 4.0).abs() <= 1e-9, "length {length}");
}

#[test]
fn a_rounded_block_keeps_its_round_concentric() {
    use ngk::modeling::blend::filleted;
    use std::f64::consts::PI;

    let source = block(4.0, 4.0, 4.0).unwrap();
    let edge = source.solid().edges()[0].key();
    let rounded = filleted(source, vec![edge], 1.0).unwrap();
    let corner = |radius: f64| radius * radius * (1.0 - PI / 4.0);
    let before = rounded.solid().volume().unwrap();
    assert!(
        (before - (64.0 - corner(1.0) * 4.0)).abs() <= 1e-6,
        "volume {before}"
    );

    // Each face moves out by 0.5: the block is 5 wide, the round's radius 1.5.
    let grown = offset(rounded, 0.5).unwrap();
    let volume = grown.solid().volume().unwrap();
    let expected = 125.0 - corner(1.5) * 5.0;
    assert!(
        (volume - expected).abs() <= 1e-6,
        "volume {volume}, expected {expected}"
    );
}

#[test]
fn a_rounded_block_shrinks_its_round_until_the_round_collapses() {
    use ngk::builders::offset::OffsetBuildError;
    use ngk::modeling::blend::filleted;
    use std::f64::consts::PI;

    let rounded = || {
        let source = block(4.0, 4.0, 4.0).unwrap();
        let edge = source.solid().edges()[0].key();
        filleted(source, vec![edge], 1.0).unwrap()
    };
    let shrunk = offset(rounded(), -0.5).unwrap();
    let volume = shrunk.solid().volume().unwrap();
    let expected = 27.0 - 0.25 * (1.0 - PI / 4.0) * 3.0;
    assert!(
        (volume - expected).abs() <= 1e-6,
        "volume {volume}, expected {expected}"
    );
    assert!(matches!(
        offset(rounded(), -1.0),
        Err(OffsetBuildError::FaceSupport { .. })
    ));
}

#[test]
fn a_chamfered_block_moves_its_bevel_too() {
    use ngk::modeling::blend::chamfered;
    use ngk::topology::validation::validate_solid_orientation;

    let source = block(4.0, 4.0, 4.0).unwrap();
    let edge = source.solid().edges()[0].key();
    let bevelled = chamfered(source, vec![edge], 1.0).unwrap();
    let before = bevelled.solid().volume().unwrap();
    let grown = offset(bevelled, 0.5).unwrap();
    let after = grown.solid().volume().unwrap();
    // The bevel's plane x + y = 1 moves to x + y = 1 - 0.5·√2, and the
    // grown cube's corner to (-0.5, -0.5): the prism cut from the 5³ cube has
    // legs 2 - 0.5·√2 and length 5.
    let leg = 2.0 - 0.5 * 2.0_f64.sqrt();
    let expected = 125.0 - 0.5 * leg * leg * 5.0;
    assert!(after > before);
    assert!(
        (after - expected).abs() <= 1e-6,
        "volume {after}, expected {expected}"
    );
    validate_solid_orientation(grown.model(), grown.key()).unwrap();
}
