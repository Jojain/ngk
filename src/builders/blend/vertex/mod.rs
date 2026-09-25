//! Vertex treatments: how the blends meeting at one vertex are closed.

mod blend;
mod conic;
mod corner_cut;
mod mitre;
mod run_out;
mod smooth;
mod treat;
mod trihedral;

pub(crate) use blend::VertexBlend;
pub(crate) use treat::treat_vertex;
