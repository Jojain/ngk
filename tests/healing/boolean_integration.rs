use ngk::StandardPayload;
use ngk::builders::boolean::{BooleanOperation, BooleanOptions, boolean};
use ngk::geometry::Frame;
use ngk::healing::{HealingOptions, HealingScope, remove_redundant_cells};
use ngk::modeling::solids;
use ngk::topology::TopologyEditError;
use ngk::topology::gmap::GMap;
use ngk::topology::shape::{Shape, SolidTag};
use ngk::topology::shape_keys::SolidKey;

/// The tolerance a Boolean's fitted sections actually meet.
const BOOLEAN_TOLERANCE: f64 = 1.0e-7;

/// Imports `tool` into `host`'s map and returns the imported solid's key.
fn import(host: &mut GMap<StandardPayload>, tool: Shape<SolidTag>) -> SolidKey {
    let (map, key) = tool.into_map();
    host.transaction(|edit| {
        let dart = edit.merge(map.solid_unchecked(key));
        Ok::<_, TopologyEditError>(
            edit.solid_key(dart)
                .expect("imported solid should register"),
        )
    })
    .expect("importing a solid should commit")
}

/// Runs the tangent Boolean and returns the map with the result's cell counts.
///
/// The cylinder's radius equals the block's size, so it touches two of the
/// block's faces along a line instead of crossing them — the configuration that
/// makes an imprint land on geometry the result keeps.
fn evaluate(
    operation: BooleanOperation,
    heal: bool,
) -> (GMap<StandardPayload>, SolidKey, (usize, usize, usize)) {
    let size = 2.0;
    let (mut map, block) = solids::block_at(Frame::xyz(), size, size, size)
        .expect("block")
        .into_map();
    let cylinder = import(
        &mut map,
        solids::cylinder_at(Frame::xyz(), size, 2.0 * size).expect("cylinder"),
    );
    let result = boolean(
        &mut map,
        block,
        cylinder,
        operation,
        BooleanOptions {
            heal,
            ..BooleanOptions::default()
        },
    )
    .expect("the tangent Boolean should succeed");
    let counts = {
        let solid = map.solid_unchecked(result.solid);
        (
            solid.faces().len(),
            solid.edges().len(),
            solid.vertices().len(),
        )
    };
    (map, result.solid, counts)
}

/// Heals a result that the Boolean already healed, to check it is a fixed point.
fn reheal(map: &mut GMap<StandardPayload>, solid: SolidKey) -> ngk::healing::HealingReport {
    remove_redundant_cells(
        map,
        HealingOptions {
            scope: HealingScope::Solid(solid),
            linear_tolerance: BOOLEAN_TOLERANCE,
            angular_tolerance: BOOLEAN_TOLERANCE,
            ..HealingOptions::default()
        },
    )
    .expect("a healed result should re-heal cleanly")
}

#[test]
fn a_block_intersected_with_a_tangent_cylinder_keeps_no_redundant_topology() {
    let (mut map, solid, counts) = evaluate(BooleanOperation::Intersection, true);
    assert_eq!(counts, (5, 9, 6));
    assert_eq!(map.iter_solids().count(), 1);

    let leftover = reheal(&mut map, solid);
    assert!(
        leftover.is_empty(),
        "nothing should be left to remove, but a second pass removed {leftover:?}"
    );
}

#[test]
fn healing_a_tangent_union_fuses_the_fragments_the_imprint_created() {
    let (_, _, raw) = evaluate(BooleanOperation::Union, false);
    let (mut map, solid, healed) = evaluate(BooleanOperation::Union, true);

    assert_eq!(raw, (8, 14, 8), "splitting leaves eight fragments");
    // The disc, the block's bottom square and the corner it pokes out with all
    // describe one plane, so the whole bottom comes back as a single face; the
    // block's top keeps only the corner, and the cylinder keeps its wall and
    // its far cap.
    assert_eq!(
        healed,
        (6, 11, 7),
        "healing should fuse the union back to its six real faces"
    );
    assert_eq!(map.iter_solids().count(), 1);
    assert!(reheal(&mut map, solid).is_empty(), "healing is idempotent");
}

#[test]
fn healing_a_boolean_result_is_opt_in() {
    let (_, _, raw) = evaluate(BooleanOperation::Union, false);
    let (_, _, healed) = evaluate(BooleanOperation::Union, true);

    assert!(
        raw.0 > healed.0,
        "the default must leave the raw result untouched: raw {raw:?}, healed {healed:?}"
    );
}
