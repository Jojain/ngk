//! Hand-built cavity solids used by topology and STEP tests.
//!
//! The production path for creating this shape is a Boolean difference. This
//! fixture writes the smallest equivalent GMap directly so tests can isolate
//! cavity topology without depending on Boolean evaluation.

use std::collections::HashMap;

use ngk::geometry::{Frame, Sphere, Surface};
use ngk::model::Model;
use ngk::topology::attributes::{FaceAttr, SheetAttr, SolidAttr};
use ngk::topology::embedding::EntityOwner;
use ngk::topology::gmap::{Dart, Dim};
use ngk::topology::shape::{Shape, SolidTag};
use ngk::topology::shape_keys::SolidKey;
use ngk::topology::{ModelEdit, ModelEditError, StandardPayload};

/// A sphere of radius `outer` with a concentric spherical cavity of radius
/// `inner` in it.
pub fn hollow_sphere(outer: f64, inner: f64) -> Shape<SolidTag, StandardPayload> {
    let mut model = Model::<StandardPayload>::new();
    let solid = model
        .transaction(|edit| {
            let outer = sphere_face(edit, outer)?;
            let inner_face = sphere_face(edit, inner)?;
            let inner = edit.alpha(Dim::Zero, inner_face);
            edit.add_sheet(SheetAttr::new(outer, ()));
            edit.add_sheet(SheetAttr::new(inner, ()));
            let solid = edit.add_solid(SolidAttr::new((), outer, Some(vec![inner])));
            add_cavity_cut(edit, solid, outer, inner)?;
            Ok::<_, ModelEditError>(solid)
        })
        .expect("a hand-built hollow sphere should commit");
    Shape::new(model, solid)
}

/// Adds one spherical boundary face on its four-dart bigon.
fn sphere_face(
    edit: &mut ModelEdit<'_, StandardPayload>,
    radius: f64,
) -> Result<Dart, ModelEditError> {
    let d = [
        edit.add_dart(),
        edit.add_dart(),
        edit.add_dart(),
        edit.add_dart(),
    ];
    edit.link(Dim::Zero, d[0], d[1])?;
    edit.link(Dim::Zero, d[2], d[3])?;
    edit.link(Dim::One, d[1], d[2])?;
    edit.link(Dim::One, d[3], d[0])?;
    edit.link(Dim::Two, d[0], d[3])?;
    edit.link(Dim::Two, d[1], d[2])?;

    let face = edit.add_face(FaceAttr::closed(
        Surface::Sphere(Sphere::new(Frame::xyz(), radius)),
        (),
        d[0],
        HashMap::new(),
    ));
    let owner = EntityOwner::Face(face);
    edit.own_cell(Dim::One, d[0], owner);
    edit.own_cell(Dim::Zero, d[0], owner);
    edit.own_cell(Dim::Zero, d[1], owner);
    Ok(d[0])
}

/// Opens one edge on each sphere and inserts the solid-owned pillow face.
fn add_cavity_cut(
    edit: &mut ModelEdit<'_, StandardPayload>,
    solid: SolidKey,
    outer: Dart,
    inner: Dart,
) -> Result<(), ModelEditError> {
    let front = polygon(edit, 4)?;
    let back = polygon(edit, 4)?;

    for (index, shell) in [outer, inner].into_iter().enumerate() {
        let first = [shell, edit.alpha(Dim::Zero, shell)];
        let second = [
            edit.alpha(Dim::Zero, edit.alpha(Dim::Two, shell)),
            edit.alpha(Dim::Two, shell),
        ];
        edit.unlink(Dim::Two, first[0])?;
        edit.unlink(Dim::Two, first[1])?;
        link_edge_uses(edit, first, front[2 * index])?;
        link_edge_uses(edit, second, back[(4 - 2 * index) % 4])?;
    }

    link_edge_uses(edit, front[1], back[3])?;
    link_edge_uses(edit, front[3], back[1])?;
    edit.sew(Dim::Three, front[0][0], back[0][1])?;

    let owner = EntityOwner::Solid(solid);
    edit.own_cell(Dim::Two, front[0][0], owner);
    edit.own_cell(Dim::One, front[1][0], owner);
    edit.own_cell(Dim::One, front[3][0], owner);
    Ok(())
}

fn polygon(
    edit: &mut ModelEdit<'_, StandardPayload>,
    slots: usize,
) -> Result<Vec<[Dart; 2]>, ModelEditError> {
    let uses = (0..slots)
        .map(|_| [edit.add_dart(), edit.add_dart()])
        .collect::<Vec<_>>();
    for use_ in &uses {
        edit.link(Dim::Zero, use_[0], use_[1])?;
    }
    for slot in 0..slots {
        edit.link(Dim::One, uses[slot][1], uses[(slot + 1) % slots][0])?;
    }
    Ok(uses)
}

fn link_edge_uses(
    edit: &mut ModelEdit<'_, StandardPayload>,
    first: [Dart; 2],
    second: [Dart; 2],
) -> Result<(), ModelEditError> {
    edit.link(Dim::Two, first[0], second[1])?;
    edit.link(Dim::Two, first[1], second[0])?;
    Ok(())
}
