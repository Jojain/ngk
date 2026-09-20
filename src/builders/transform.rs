//! Moving a model's geometry over a fixed topology.
//!
//! A transform is a pure geometry rewrite: the GMap, the embedding records,
//! every key and the revision semantics are untouched. Nothing here rebuilds a
//! map, and nothing here is allowed to grow into something that does.

use crate::geometry::transform::Rigid;
use crate::model::Model;
use crate::topology::edit::ModelEditError;
use crate::topology::payload::DefaultPayload;

/// Moves every piece of geometry in `model` by a rigid motion.
///
/// Three loops and nothing else: vertex positions, edge curves, face surfaces.
/// A rigid motion preserves every parameterization, so no pcurve is read and no
/// [`LoopDefinition`](crate::topology::attributes::LoopDefinition) is touched —
/// the stored parameter-space data is bit-identical afterwards. No dart is
/// created or destroyed, no cell changes owner, and no key moves.
///
/// Returns nothing because there is nothing here that could fail. See
/// [`Rigid`] for why.
pub fn rigid<P: DefaultPayload>(model: &mut Model<P>, r: &Rigid) {
    let vertices = model
        .iter_vertices()
        .map(|(key, _)| key)
        .collect::<Vec<_>>();
    let edges = model.iter_edges().map(|(key, _)| key).collect::<Vec<_>>();
    let faces = model.iter_faces().map(|(key, _)| key).collect::<Vec<_>>();

    model
        .transaction(|edit| {
            for key in vertices {
                let vertex = edit.vertex_attr_mut_unchecked(key);
                vertex.point = r.apply(vertex.point);
            }

            for key in edges {
                let edge = edit.edge_attr_mut_unchecked(key);
                edge.curve = edge.curve.moved(r);
            }

            for key in faces {
                let face = edit.face_attr_mut_unchecked(key);
                face.surface = face.surface.moved(r);
            }

            Ok::<_, ModelEditError>(())
        })
        .expect("a rigid motion rewrites geometry over an unchanged topology");
}
