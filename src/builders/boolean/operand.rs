//! Resolution, validation, and import of Boolean operands.

use std::collections::HashSet;

use crate::topology::TopologyEdit;
use crate::topology::gmap::{Cell0, Cell1, Cell2, GMap};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};

use super::{BooleanCell, BooleanError, BooleanOperand};

#[derive(Clone, Default)]
pub(crate) struct OperandCells {
    pub(crate) vertices: Vec<VertexKey>,
    pub(crate) edges: Vec<EdgeKey>,
    pub(crate) faces: Vec<FaceKey>,
}

pub(crate) fn operand_cells<P: Payload>(
    g: &GMap<P>,
    operand: BooleanOperand,
) -> Result<OperandCells, BooleanError> {
    let cells = match operand {
        BooleanOperand::Vertex(key) => {
            g.vertex(key)
                .ok_or(BooleanError::MissingOperand { operand })?;
            OperandCells {
                vertices: vec![key],
                ..Default::default()
            }
        }
        BooleanOperand::Edge(key) => {
            let edge = g
                .edge(key)
                .ok_or(BooleanError::MissingOperand { operand })?;
            OperandCells {
                vertices: edge
                    .vertices()
                    .into_iter()
                    .map(|vertex| vertex.key())
                    .collect(),
                edges: vec![key],
                faces: Vec::new(),
            }
        }
        BooleanOperand::Profile(key) => {
            let profile = g
                .profile(key)
                .ok_or(BooleanError::MissingOperand { operand })?;
            OperandCells {
                vertices: unique(profile.vertices().into_iter().map(|vertex| vertex.key())),
                edges: unique(profile.edges().into_iter().map(|edge| edge.key())),
                faces: Vec::new(),
            }
        }
        BooleanOperand::Face(key) => {
            let face = g
                .face(key)
                .ok_or(BooleanError::MissingOperand { operand })?;
            OperandCells {
                vertices: unique(face.vertices().into_iter().map(|vertex| vertex.key())),
                edges: unique(face.edges().into_iter().map(|edge| edge.key())),
                faces: vec![key],
            }
        }
        BooleanOperand::Sheet(key) => {
            let sheet = g
                .sheet(key)
                .ok_or(BooleanError::MissingOperand { operand })?;
            OperandCells {
                vertices: unique(sheet.vertices().into_iter().map(|vertex| vertex.key())),
                edges: unique(sheet.edges().into_iter().map(|edge| edge.key())),
                faces: unique(sheet.faces().into_iter().map(|face| face.key())),
            }
        }
        BooleanOperand::Solid(key) => {
            let solid = g
                .solid(key)
                .ok_or(BooleanError::MissingOperand { operand })?;
            OperandCells {
                vertices: unique(solid.vertices().into_iter().map(|vertex| vertex.key())),
                edges: unique(solid.edges().into_iter().map(|edge| edge.key())),
                faces: unique(solid.faces().into_iter().map(|face| face.key())),
            }
        }
    };
    for vertex in cells.vertices.iter().copied() {
        if g.vertex_unchecked(vertex).point().is_none() {
            return Err(BooleanError::MissingGeometry {
                cell: BooleanCell::Vertex(vertex),
            });
        }
    }
    for edge in cells.edges.iter().copied() {
        if g.edge_unchecked(edge).curve().is_none() {
            return Err(BooleanError::MissingGeometry {
                cell: BooleanCell::Edge(edge),
            });
        }
    }
    Ok(cells)
}

pub(crate) fn import_operand<P: Payload>(
    target: &mut TopologyEdit<'_, P>,
    source: &GMap<P>,
    operand: BooleanOperand,
) -> Result<BooleanOperand, BooleanError> {
    let imported = match operand {
        BooleanOperand::Vertex(key) => {
            let view = source
                .vertex(key)
                .ok_or(BooleanError::MissingOperand { operand })?;
            let dart = target.merge(view);
            BooleanOperand::Vertex(target.cell_key_unchecked::<Cell0>(dart))
        }
        BooleanOperand::Edge(key) => {
            let view = source
                .edge(key)
                .ok_or(BooleanError::MissingOperand { operand })?;
            let dart = target.merge(view);
            BooleanOperand::Edge(target.cell_key_unchecked::<Cell1>(dart))
        }
        BooleanOperand::Profile(key) => {
            let view = source
                .profile(key)
                .ok_or(BooleanError::MissingOperand { operand })?;
            let dart = target.merge(view);
            BooleanOperand::Profile(target.profile_key_unchecked(dart))
        }
        BooleanOperand::Face(key) => {
            let view = source
                .face(key)
                .ok_or(BooleanError::MissingOperand { operand })?;
            let dart = target.merge(view);
            BooleanOperand::Face(target.cell_key_unchecked::<Cell2>(dart))
        }
        BooleanOperand::Sheet(key) => {
            let view = source
                .sheet(key)
                .ok_or(BooleanError::MissingOperand { operand })?;
            let dart = target.merge(view);
            BooleanOperand::Sheet(target.sheet_key_unchecked(dart))
        }
        BooleanOperand::Solid(key) => {
            let view = source
                .solid(key)
                .ok_or(BooleanError::MissingOperand { operand })?;
            let dart = target.merge(view);
            BooleanOperand::Solid(
                target
                    .solid_key(dart)
                    .expect("merged solid must retain its registration"),
            )
        }
    };
    Ok(imported)
}

fn unique<T: Copy + Eq + std::hash::Hash>(values: impl IntoIterator<Item = T>) -> Vec<T> {
    let mut seen = HashSet::new();
    values
        .into_iter()
        .filter(|value| seen.insert(*value))
        .collect()
}
