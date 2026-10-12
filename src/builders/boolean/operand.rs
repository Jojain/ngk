//! Resolution, validation, and import of Boolean operands.

use std::collections::{HashMap, HashSet};

use crate::model::{Cell0, Cell1, Cell2, Model};
use crate::topology::ModelEdit;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};

use super::{
    BooleanError, BooleanInput, BooleanOperand, BooleanOperation, BooleanOptions,
    BooleanTolerancePolicy, BooleanTolerances,
};
use crate::topology::shape_keys::SolidKey;
use crate::topology::validation::{validate_solid_manifold, validate_solid_orientation};

#[derive(Clone, Default)]
pub(crate) struct OperandCells {
    pub(crate) vertices: Vec<VertexKey>,
    pub(crate) edges: Vec<EdgeKey>,
    pub(crate) faces: Vec<FaceKey>,
}

pub(crate) fn operand_cells<P: Payload>(
    g: &Model<P>,
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
    Ok(cells)
}

pub(crate) fn import_operand<P: Payload>(
    target: &mut ModelEdit<'_, P>,
    source: &Model<P>,
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

/// The distinct operands of one call, the target first.
///
/// A key passed more than once is one operand: the set rules collapse it
/// (`T ∪ T = T`, `T ∩ T = T`), so it is intersected and classified once, and
/// every input that named it reports that one operand's lineage.
pub(crate) struct Operands<K> {
    /// In index order, the target first.
    pub(crate) operands: Vec<Operand<K>>,
    /// For each tool, the index of the operand it is.
    tools: Vec<usize>,
}

/// One distinct operand, and the input it was first passed as.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Operand<K> {
    pub(crate) key: K,
    pub(crate) input: BooleanInput,
}

impl<K: Copy + Eq> Operands<K> {
    pub(crate) fn new(target: K, tools: &[K]) -> Self {
        let mut operands = Self {
            operands: vec![Operand {
                key: target,
                input: BooleanInput::Target,
            }],
            tools: Vec::with_capacity(tools.len()),
        };
        for (index, &tool) in tools.iter().enumerate() {
            let existing = operands
                .operands
                .iter()
                .position(|operand| operand.key == tool);
            let operand = existing.unwrap_or_else(|| {
                operands.operands.push(Operand {
                    key: tool,
                    input: BooleanInput::Tool(index),
                });
                operands.operands.len() - 1
            });
            operands.tools.push(operand);
        }
        operands
    }

    pub(crate) fn target(&self) -> K {
        self.operands[0].key
    }

    pub(crate) fn keys(&self) -> impl Iterator<Item = K> + '_ {
        self.operands.iter().map(|operand| operand.key)
    }

    /// The same operands, each key turned into another of the same operand.
    pub(crate) fn map<T>(&self, f: impl Fn(K) -> T) -> Vec<Operand<T>> {
        self.operands
            .iter()
            .map(|operand| Operand {
                key: f(operand.key),
                input: operand.input,
            })
            .collect()
    }

    /// Whether some tool is the target itself.
    pub(crate) fn target_is_a_tool(&self) -> bool {
        self.tools.contains(&0)
    }

    /// Spreads one value per operand back over the inputs: the target's, then
    /// one per tool in the order the tools were passed.
    pub(crate) fn per_input<T: Clone>(&self, mut values: Vec<T>) -> (T, Vec<T>) {
        let tools = self
            .tools
            .iter()
            .map(|&operand| values[operand].clone())
            .collect();
        (values.swap_remove(0), tools)
    }
}

/// Resolves every operand's cells, naming the input that is missing.
pub(crate) fn operands_cells<P: Payload, K: Copy>(
    map: &Model<P>,
    operands: &Operands<K>,
    operand: impl Fn(K) -> BooleanOperand,
) -> Result<Vec<OperandCells>, BooleanError> {
    operands
        .operands
        .iter()
        .map(|&Operand { key, input }| {
            operand_cells(map, operand(key)).map_err(|error| match error {
                BooleanError::MissingOperand { operand } => {
                    BooleanError::MissingInput { input, operand }
                }
                other => other,
            })
        })
        .collect()
}

/// Rejects two distinct operands registering the same boundary face.
///
/// `cells` holds each operand's cells, in the operands' order.
pub(crate) fn reject_shared_faces<K>(
    operands: &Operands<K>,
    cells: &[OperandCells],
) -> Result<(), BooleanError> {
    let mut owners = HashMap::<FaceKey, usize>::new();
    for (operand, cells) in cells.iter().enumerate() {
        for &face in &cells.faces {
            if let Some(&owner) = owners.get(&face)
                && owner != operand
            {
                return Err(BooleanError::SharedOperandBoundary {
                    first: operands.operands[owner].input,
                    second: operands.operands[operand].input,
                });
            }
            owners.insert(face, operand);
        }
    }
    Ok(())
}

pub(crate) struct BooleanContext {
    pub(crate) operands: Operands<SolidKey>,
    pub(crate) operation: BooleanOperation,
    pub(crate) options: BooleanOptions,
    pub(crate) tolerances: BooleanTolerances,
}

impl BooleanContext {
    /// Validates every closed oriented operand before any mutation and fixes
    /// one budget for all of them.
    pub(crate) fn admit<P: Payload>(
        map: &Model<P>,
        target: SolidKey,
        tools: &[SolidKey],
        operation: BooleanOperation,
        mut options: BooleanOptions,
    ) -> Result<Self, BooleanError> {
        if !options.intersections.validate() || options.max_classification_rays < 2 {
            return Err(crate::geometry::IntersectionError::InvalidOptions.into());
        }
        let operands = Operands::new(target, tools);
        let cells = operands_cells(map, &operands, BooleanOperand::Solid)?;
        for &Operand { key: solid, input } in &operands.operands {
            validate_solid_manifold(map, solid)
                .and_then(|()| validate_solid_orientation(map, solid))
                .map_err(|source| BooleanError::InvalidOperand {
                    input,
                    solid,
                    source,
                })?;
        }
        reject_shared_faces(&operands, &cells)?;
        let tolerances = BooleanTolerances::from_operands(map, &cells, options.tolerances)?;
        tolerances.apply(&mut options.intersections);
        options.tolerances = BooleanTolerancePolicy::Fixed(tolerances);
        Ok(Self {
            operands,
            operation,
            options,
            tolerances,
        })
    }
}
