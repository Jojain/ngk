//! Chamfers and fillets: one engine for every blend of a crease or a corner.
//!
//! A call resolves its target into sets of planar corners, solid edges and
//! solid corner cuts; plans each against the model as the call found it —
//! a section per edge, a treatment per vertex, a trim per planar corner —
//! into one [`surgery`](surgery::Surgery); builds that surgery; and checks the
//! result. Only the section solvers and the planar corner solver read the
//! [`law`](law::BlendLaw), so a chamfer and a fillet differ there and nowhere
//! else.

mod check;
mod corner;
mod errors;
mod execute;
mod law;
mod network;
mod operations;
mod pcurve;
mod section;
mod solid;
mod surgery;
mod target;
mod vertex;

pub use errors::BlendError;
pub(crate) use law::{BlendLaw, ChamferLaw, FilletLaw};
pub(crate) use operations::blend_edit;
pub use target::{BlendSelection, BlendTarget};
