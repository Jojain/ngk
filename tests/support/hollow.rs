//! Solids with a cavity inside them, which no builder in this tree makes.
//!
//! `SolidAttr` has carried inner shells all along, and the validators check
//! them — a void's faces must point *into* the void, which is to say away from
//! the material, exactly as the outer shell's point away from it. Nothing
//! produces one, though: no primitive is hollow and no boolean yet leaves a
//! cavity behind, so a solid with a void reaches the kernel from a file or
//! from here.
//!
//! Two concentric spheres are the smallest such solid, and the one with the
//! fewest moving parts: both shells are a single boundaryless face, so the
//! fixture states the shell orientation and nothing else.

use ngk::geometry::Frame;
use ngk::model::Model;
use ngk::topology::attributes::ShellRoot;
use ngk::topology::orientation::Orientation;
use ngk::topology::shape::{Shape, SolidTag};
use ngk::topology::shape_keys::FaceKey;
use ngk::topology::{ModelEditError, StandardPayload};

/// A sphere of radius `outer` with a concentric spherical cavity of radius
/// `inner` in it.
///
/// The cavity's face is the same support a solid sphere would use, read the
/// other way round: its own parameterization faces away from the centre, and a
/// void is bounded from the material side.
pub fn hollow_sphere(outer: f64, inner: f64) -> Shape<SolidTag, StandardPayload> {
    use ngk::builders::solids::add_sphere;

    let mut g = Model::<StandardPayload>::new();
    let outer_solid =
        add_sphere(&mut g, Frame::xyz(), outer).expect("an outer sphere should build");
    let cavity = add_sphere(&mut g, Frame::xyz(), inner).expect("a cavity sphere should build");

    // `add_sphere` registers a solid of its own around each face; the cavity's
    // is dissolved into the one that surrounds it.
    let cavity_face = boundaryless_face(&g, cavity);
    let solid = g
        .transaction(|edit| {
            edit.remove_solid(cavity);
            let void = ShellRoot::Face {
                face: cavity_face,
                sense: Orientation::Reversed,
            };
            edit.sheet_attr_mut_unchecked(
                edit.model()
                    .sheet_key_at_face(cavity_face)
                    .expect("the cavity's face is registered as a sheet"),
            )
            .root = void;
            let attr = edit.solid_attr_mut_unchecked(outer_solid);
            attr.inner_shells = Some(vec![void]);
            Ok::<_, ModelEditError>(outer_solid)
        })
        .expect("a hollow sphere should commit");

    Shape::new(g, solid)
}

/// The one face a boundaryless solid's outer shell is.
fn boundaryless_face(
    g: &Model<StandardPayload>,
    solid: ngk::topology::shape_keys::SolidKey,
) -> FaceKey {
    g.solid(solid)
        .expect("the solid is registered")
        .outer_shell()
        .faces()
        .first()
        .expect("a shell has a face")
        .key()
}
