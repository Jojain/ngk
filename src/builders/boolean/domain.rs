//! What one Boolean operation runs on, stated without naming a dimension.
//!
//! An operation has two operands. Splitting replaces each operand's cells with
//! the pieces the other operand cut them into, and `BooleanLineage` records
//! which source cell each piece came from. The boolean code calls one such
//! piece a **fragment**, and `BoundaryFragment` in `super::neighborhood` is the
//! record of one.
//!
//! Which cells those are depends on the operand. `operand_cells` gives a solid
//! its boundary faces, so a solid's fragments are pieces of its boundary. It
//! gives a face operand the face itself, so a face's fragments are pieces of
//! that face. Both are `FaceKey`s out of the same `lineage.faces`; see
//! `super::planar_domain` for what follows from the difference.
//!
//! The stages between splitting and assembly -- grouping fragments, locating
//! each one against the other operand, and deciding which survive -- are
//! written against `BooleanDomain` so that they need not know which of those
//! two cases they are running.

use std::fmt::Debug;
use std::hash::Hash;

use nalgebra::Vector3;

use crate::geometry::Point3;
use crate::model::Model;
use crate::topology::payload::Payload;

use super::{
    BooleanError, BooleanLineage, BooleanOperation, BooleanOptions, BooleanSide, BooleanTolerances,
};

/// What the operation table does with one classified fragment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Selection {
    Keep,
    /// Kept, but bounding the result the other way round than it bounds its
    /// own operand — the second operand's boundary under a difference.
    KeepReversed,
    Drop,
}

/// Where a point sits relative to one whole operand.
///
/// The same four answers serve a solid and a face, so every domain's table
/// reads them without knowing which kind of operand produced them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RelativeLocation {
    Inside,
    Outside,
    /// On the operand's boundary, with both boundaries facing the same way.
    OnBoundarySame,
    /// On the operand's boundary, with the two boundaries facing opposite ways.
    OnBoundaryOpposite,
}

/// Answers where a point lies relative to the operand it was built from.
///
/// One implementation per domain, and they share no strategy: `SolidRayCaster`
/// casts rays and counts crossings, while `PlanarOperandClassifier` asks each
/// face's `FaceTrimDomain` a winding question in the support surface's
/// parameter space. They share only the question, which is what lets
/// `classify::run` be one stage.
pub(crate) trait OperandClassifier<F> {
    /// Locates `point` — an interior point of `source`, whose outward sense is
    /// `normal` — relative to this classifier's operand.
    ///
    /// `source` names the fragment being probed and is used only to say which
    /// one failed.
    fn locate(
        &self,
        point: Point3,
        normal: Vector3<f64>,
        source: F,
    ) -> Result<RelativeLocation, BooleanError>;
}

/// One kind of operand pair a regularized Boolean is defined on.
///
/// Implementors supply what the shared stages cannot derive: which cells an
/// operand's fragments are, how two fragments are adjacent, which cells the
/// section barred between them, how to ask the other operand where a point
/// lies, and which fragments each operation keeps.
pub(crate) trait BooleanDomain {
    /// One piece an operand's cell was split into, kept or dropped whole.
    ///
    /// A `FaceKey` for both domains today, meaning a piece of a solid's
    /// boundary in one and a piece of a face in the other.
    type Fragment: Copy + Eq + Ord + Hash + Debug;

    /// Answers whether a point lies inside one whole operand.
    type Classifier<'a, P: Payload + 'a>: OperandClassifier<Self::Fragment>;

    /// The (source cell, fragment) pairs one operand's lineage recorded.
    ///
    /// Splitting replaces one cell with several, and the lineage is the only
    /// record of which source each came from; a fragment that outlives
    /// assembly is reported against that source.
    fn fragments(lineage: &BooleanLineage) -> Vec<(Self::Fragment, Self::Fragment)>;

    /// A point interior to `fragment`, and the outward normal there.
    ///
    /// The point seeds a classification, so it is chosen for clearance from the
    /// fragment's own edges rather than merely for lying inside it: a ray cast
    /// from next to an edge is the one that comes back undecidable.
    fn probe<P: Payload>(
        map: &Model<P>,
        fragment: Self::Fragment,
        tolerances: BooleanTolerances,
    ) -> Result<(Point3, Vector3<f64>), BooleanError>;

    /// Whether a fragment on `side`, located at `location`, survives
    /// `operation`.
    fn keeps(
        operation: BooleanOperation,
        side: BooleanSide,
        location: RelativeLocation,
    ) -> Selection;

    /// Builds the classifier for the operand `lineage` describes.
    fn classifier<'a, P: Payload>(
        map: &'a Model<P>,
        lineage: &BooleanLineage,
        options: BooleanOptions,
        tolerances: BooleanTolerances,
    ) -> Result<Self::Classifier<'a, P>, BooleanError>;
}
