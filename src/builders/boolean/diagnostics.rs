//! What a Boolean's contact computation settled on and left unresolved.

use crate::geometry::IntersectionIncompleteReason;
use crate::topology::shape_keys::FaceKey;

/// Tolerances, broad-phase pruning and coverage limitations retained with the
/// contact plan.
#[derive(Debug, Clone, Default)]
pub struct BooleanDiagnostics {
    pub tolerances: super::BooleanTolerances,
    pub candidate_pairs_tested: usize,
    pub candidate_pairs_pruned: usize,
    pub edge_face_pairs_tested: usize,
    pub edge_face_pairs_pruned: usize,
    pub branches_uncertified: usize,
    /// Candidate overlap is not proof of a coincident trimmed region.
    pub unresolved_overlaps: Vec<(FaceKey, FaceKey)>,
    pub coverage: Vec<IntersectionIncompleteReason>,
}
