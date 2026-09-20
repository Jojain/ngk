//! Face builders and their edit-scoped implementations.
//!
//! Each operation lives in its own submodule. This file only declares the
//! family and keeps the public and crate-private operation names together.

mod annulus;
mod circle;
mod face;
mod imprints;
mod polygon;
mod split;
mod support;

pub use annulus::add_annulus;
pub use circle::add_circle;
pub use face::{add_face, add_rectangle, add_square};
pub use imprints::split_face_by_imprints;
pub use polygon::{add_polygon, add_polygon_with_holes};
pub use split::split_face_edge;
pub use support::{
    FaceEdgeSplitError, FaceImprint, FaceImprintGraph, FaceImprintGraphEdge, FaceImprintSection,
    FaceImprintSplit, FaceImprintSplitError,
};

pub(crate) use annulus::{
    assign_rebased_pcurve, assign_split_pcurves, closed_boundary_curve_reversed, face_edge_dart,
    incident_face_pcurves, periodic_image_near_pcurve, rebased_face_pcurves,
};
pub(crate) use face::add_face_edit;
pub(crate) use imprints::{chord_loop_kinds, split_face_by_imprints_edit};
pub(crate) use polygon::add_polygon_edit;
pub(crate) use polygon::reverse_face_winding_edit;
pub(crate) use split::split_face_edge_edit;
pub(crate) use support::{
    FaceImprintCut, IncidentFacePcurve, RebasedFacePcurve, apply_face_chord_split,
    boundary_edge_at_uv, bounding_loops, edge_curve, face_boundary_edges, face_boundary_uvs,
    loop_boundary_edges, snap_boundary_corner, snap_boundary_corner_in,
};
