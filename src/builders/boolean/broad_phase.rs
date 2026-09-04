//! Candidate generation before expensive geometric intersection.

use crate::topology::shape_keys::FaceKey;

/// Returns distinct face pairs that must enter the narrow phase.
///
/// This initial implementation is deliberately exhaustive. Keeping the
/// contract in its own module lets us add bounding-volume pruning without
/// changing contact computation or network construction.
pub(crate) fn candidate_face_pairs(
    first: &[FaceKey],
    second: &[FaceKey],
) -> Vec<(FaceKey, FaceKey)> {
    first
        .iter()
        .copied()
        .flat_map(|first_face| {
            second
                .iter()
                .copied()
                .filter(move |second_face| first_face != *second_face)
                .map(move |second_face| (first_face, second_face))
        })
        .collect()
}
