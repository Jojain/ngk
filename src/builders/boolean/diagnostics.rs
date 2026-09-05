//! Observable work and unresolved geometry from Boolean preparation.

use crate::geometry::IntersectionIncompleteReason;
use crate::topology::shape_keys::FaceKey;

/// Work counters and coverage limitations retained with the contact plan.
#[derive(Debug, Clone, Default)]
pub struct BooleanDiagnostics {
    pub tolerances: super::BooleanTolerances,
    pub fragments: usize,
    pub components: usize,
    pub classification_rays: usize,
    pub candidate_pairs_tested: usize,
    pub candidate_pairs_pruned: usize,
    pub edge_face_pairs_tested: usize,
    pub edge_face_pairs_pruned: usize,
    pub branches_found: usize,
    pub branches_uncertified: usize,
    pub spans: usize,
    pub events: usize,
    pub regions: usize,
    /// Candidate overlap is not proof of a coincident trimmed region.
    pub unresolved_overlaps: Vec<(FaceKey, FaceKey)>,
    pub coverage: Vec<IntersectionIncompleteReason>,
}
