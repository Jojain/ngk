//! Boolean preparation: contact computation and two-sided B-Rep splitting.

mod assemble;
mod broad_phase;
mod classify;
mod clip;
mod contacts;
mod diagnostics;
mod domain;
mod errors;
mod face_boolean;
mod neighborhood;
mod select;
pub use diagnostics::BooleanDiagnostics;
mod graph;
mod imprint;
mod operand;
mod pair;
mod planar_domain;
mod result;
mod solid_domain;
mod tolerance;
mod trim;
pub use tolerance::{BooleanTolerancePolicy, BooleanTolerances};

pub use classify::solid_contains_point;
use contacts::{compute_contacts, normalize_face_imprint_chains, reroute_boundary_imprints};
pub use errors::BooleanError;
pub use face_boolean::{FaceBoolean, face_boolean};
pub use graph::{
    IntersectionEvent, IntersectionEventId, IntersectionEventLocation, IntersectionEventUse,
    IntersectionNetwork, IntersectionNetworkValidationError, IntersectionOrientation,
    IntersectionRegion, IntersectionSpan, IntersectionSpanId, IntersectionSpanKind,
    IntersectionSpanUse, validate_solid_network,
};
use graph::{IntersectionNetworkBuilder, edge_use, face_use, vertex_use};
use operand::{BooleanContext, OperandCells, import_operand, operand_cells};
pub use result::{
    BooleanCell, BooleanLineage, BooleanOperand, BooleanOperandPreparation, BooleanOperation,
    BooleanSide, PointContactKind, SolidBoolean, SolidBooleanLineage,
};
use solid_domain::SolidDomain;

use std::collections::{BTreeSet, HashMap, HashSet};

use crate::builders::edges::split_edge_edit;
use crate::builders::faces::{FaceImprint, split_face_by_imprints_edit, split_face_edge_edit};
use crate::geometry::parameter::{Fraction, NativeParam};
use crate::geometry::{
    ControlPolygon, ControlPolygon2, Curve, Curve2, CurveCurveIntersection,
    CurveSurfaceIntersection, Degree, HPoint, HPoint2, IntersectionOptions, Interval, KnotVector,
    NurbsCurve, NurbsCurve2, NurbsError, Periodicity, Point2, Point3, PointCoincidence,
    PreparedCurve, PreparedSurface, Surface, SurfaceSurfaceIntersection, TrimmedCurve,
    intersect_prepared_curve_surface,
};
use crate::model::Model;
use crate::topology::ModelEdit;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey, SolidKey, VertexKey};
use nalgebra::Vector2;
use slotmap::Key;

/// Raw narrow-phase observation consumed during network canonicalization.
mod operations;
pub use operations::*;
pub(crate) use operations::{IntersectionAccumulator, RawIntersection};
