//! Chooses a vertex's treatment from how the network meets it.

use super::super::errors::BlendError;
use super::super::law::{BlendLaw, ChamferLaw};
use super::super::network::BlendNetwork;
use super::super::section::EdgeSection;
use super::blend::{VertexBlend, VertexContext};
use super::corner_cut::corner_cut;
use super::mitre::mitre;
use super::run_out::run_out;
use super::trihedral::trihedral;
use crate::model::Model;
use crate::topology::payload::Payload;

/// Plans the local surgery at network vertex `index`.
///
/// The treatment is chosen by how many selected edges end at the vertex and
/// how many faces meet there. Each treatment refuses, naming the vertex, a
/// configuration it has no closed form for.
pub(crate) fn treat_vertex<P: Payload>(
    model: &Model<P>,
    network: &BlendNetwork,
    index: usize,
    sections: &[EdgeSection],
    law: BlendLaw,
) -> Result<VertexBlend, BlendError> {
    let vertex = &network.vertices[index];
    let context = VertexContext {
        model,
        network,
        sections,
        index,
        vertex,
    };
    if vertex.cut {
        return match law {
            BlendLaw::Chamfer(ChamferLaw::Distance(distance)) => corner_cut(&context, distance),
            BlendLaw::Fillet(_) => Err(BlendError::SolidVertexFillet { vertex: vertex.key }),
        };
    }
    if vertex.ring.len() != 3 {
        return Err(context.unsupported("only vertices where three faces meet are blended"));
    }
    match vertex.selected_count() {
        1 => run_out(&context),
        2 => mitre(&context),
        3 => trihedral(&context),
        _ => Err(BlendError::InconsistentSurgery {
            reason: "a network vertex has no selected edge",
        }),
    }
}
