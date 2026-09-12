//! What a STEP read had to give up on.
//!
//! Import is best-effort (D9): real files contain faces that do not close,
//! loops with duplicated edges, and references into nothing, and aborting on
//! the first bad face is useless in practice. So what could not be carried
//! across is *reported* rather than thrown, in the same shape
//! `HealingReport.skipped` uses.
//!
//! A skip is never silent. Every one names the entity and the line it came
//! from, so a user can find it in their file.

use super::part21::EntityId;

/// Everything one read could not carry across faithfully.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ImportReport {
    /// What was dropped, demoted or flagged, in the order it was met.
    pub skipped: Vec<ImportSkip>,
}

impl ImportReport {
    /// Reports whether the file came across with nothing lost.
    pub fn is_clean(&self) -> bool {
        self.skipped.is_empty()
    }

    /// Returns the skips whose reason matches a predicate.
    ///
    /// Most callers want one class — "did anything lose its analytic type?" —
    /// rather than the whole list.
    pub fn matching(
        &self,
        predicate: impl Fn(&ImportSkipReason) -> bool,
    ) -> impl Iterator<Item = &ImportSkip> {
        self.skipped
            .iter()
            .filter(move |skip| predicate(&skip.reason))
    }
}

/// One thing a read could not carry across, and where it was.
#[derive(Debug, Clone, PartialEq)]
pub struct ImportSkip {
    /// The entity responsible, when one instance is to blame.
    pub entity: Option<EntityId>,
    /// The 1-based source line that entity starts on.
    pub line: u32,
    /// Why it could not be carried across as written.
    pub reason: ImportSkipReason,
}

/// Why one entity did not survive a read intact.
///
/// Marked non-exhaustive: the later stages of `plan/step_interop.md` add
/// reasons as they add coverage, and a caller matching on this should not
/// break when they do.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum ImportSkipReason {
    /// An entity NGK has no representation for, refused by name rather than
    /// silently ignored: `VERTEX_LOOP`, `POLY_LOOP` and their like (D9).
    UnrepresentableEntity {
        /// The entity keyword, as the file spells it.
        keyword: String,
    },

    /// An edge used by more than two faces. NGK is a 3-GMap and cannot hold a
    /// non-manifold edge, so the solid carrying it is rejected (§6.3).
    NonManifoldEdge {
        /// How many faces use it.
        uses: usize,
    },

    /// An edge used by exactly one face, leaving the shell open (§6.3).
    OpenShell,

    /// A face whose geometry or loops could not be assembled. Recorded rather
    /// than thrown so that one bad face does not cost the whole file (D9).
    FaceNotConstructible {
        /// What went wrong, in the words of the layer that found out.
        detail: String,
    },

    /// A whole `MANIFOLD_SOLID_BREP` that could not be built (D9).
    ///
    /// One transaction per solid is what makes this survivable: the file's
    /// other solids are unaffected.
    SolidNotConstructible {
        /// What went wrong, in the words of the layer that found out.
        detail: String,
    },

    /// A face with several bounds and no `FACE_OUTER_BOUND` among them, whose
    /// outer boundary had to be taken as the one enclosing the most area.
    ///
    /// `FACE_OUTER_BOUND` is optional and OpenCascade omits it entirely, so
    /// this is a vendor deviation that is handled rather than refused — but
    /// the guess is only as good as the winding, so it is said out loud.
    GuessedOuterBound {
        /// How many bounds the face carried.
        bounds: usize,
    },

    /// Geometry carried across exactly, but as NURBS rather than as the
    /// analytic type the file named (D3).
    ///
    /// The point set is preserved to tolerance; only the *type* is lost, and
    /// writing the file back out will spell it as a B-spline.
    DemotedToNurbs {
        /// The entity keyword that was demoted, such as `HYPERBOLA`.
        keyword: String,
    },

    /// The file's `same_sense` disagreed with the winding NGK derives from the
    /// boundary (D5).
    ///
    /// The flag is redundant on import, so this is a free consistency check
    /// rather than a failure — but a disagreement means one of the two is
    /// wrong, and it is worth knowing which face raised it.
    SenseMismatch,

    /// A parameter curve the file did not carry, rebuilt by approximation
    /// rather than in closed form (D8).
    ApproximatedPcurve {
        /// The greatest distance found between the rebuilt pcurve and the
        /// 3D curve it must follow.
        deviation: f64,
    },
}
