use std::collections::HashSet;

use crate::topology::gmap::{Dart, Dim, GMap};

use super::cells::{EmbeddingIndex, Embedding};

/// Turns from `dart` to the next raw `dimension`-cell around the cell one
/// dimension below that the two share, passing over interior scaffold.
///
/// The cells incident to a shared boundary form a fan, and the sewing
/// involutions walk it: `alpha_dimension` steps to the next cell, and the
/// involution above it steps to the other slot of that same cell. A cell owned
/// by something of higher dimension — a cut face inside a solid, a bridge edge
/// inside a face — is not a neighbour but a thing to turn across, so the fan is
/// followed until a cell that is not scaffold appears.
///
/// Every path this takes is an odd number of alpha steps, which is what lets a
/// caller keep track of which way round it is reading the map while it turns.
///
/// Returns `None` at the open end of a fan, and when the whole fan is scaffold.
pub fn turn(map: &GMap, index: &EmbeddingIndex, dimension: Dim, dart: Dart) -> Option<Dart> {
    turn_where(map, dimension, dart, |at| index.is_embedded(dimension, at))
}

/// [`turn`], asked of a caller's own scaffold test rather than of an
/// [`EmbeddingIndex`].
///
/// Building that index validates the whole classification and refuses an
/// inconsistent one, which is the right answer for a committed model and the
/// wrong one part way through an edit, where a builder may legitimately have
/// laid down darts it has not classified yet. A caller that has to turn during
/// an edit supplies a test that reads the stored records directly.
pub fn turn_where(
    map: &GMap,
    dimension: Dim,
    dart: Dart,
    is_embedded: impl Fn(Dart) -> bool,
) -> Option<Dart> {
    let up = above(dimension);
    let mut current = dart;
    let mut visited = HashSet::new();

    loop {
        if !visited.insert(current) {
            return None;
        }
        let next = map.alpha(dimension, current);
        if next == current {
            return None;
        }
        if !is_embedded(next) {
            return Some(next);
        }
        let hop = map.alpha(up?, next);
        if hop == next {
            return None;
        }
        current = hop;
    }
}

/// Reports whether the raw `dimension`-cell at `dart` is interior scaffold,
/// asked of the stored records.
///
/// [`EmbeddingIndex::is_embedded`] answers the same question, but building that
/// index validates the whole classification and refuses an inconsistent one.
/// That is right for a committed model and wrong part way through an edit,
/// where a builder may legitimately have laid down darts it has not classified
/// yet, so a traversal that runs during an edit asks this instead.
///
/// An unlabelled cell is **not** scaffold. A record sits on whichever dart of
/// the orbit the labeller handed over, so every dart of the cell is asked
/// rather than only its representative.
pub fn is_embedded_cell(map: &GMap, embedding: &Embedding, dimension: Dim, dart: Dart) -> bool {
    map.orbit(dart, map.orbit_indices(dimension)).any(|d| {
        embedding
            .owner_at(dimension, d)
            .is_some_and(|owner| owner.dimension().index() > dimension.index())
    })
}

/// Returns the dimension one below `dimension`, or `None` at dimension zero.
pub fn below(dimension: Dim) -> Option<Dim> {
    match dimension {
        Dim::Zero => None,
        other => Some(Dim::from_index(other.index() - 1)),
    }
}

/// Returns the dimension one above `dimension`, or `None` at dimension three.
pub fn above(dimension: Dim) -> Option<Dim> {
    match dimension {
        Dim::Three => None,
        other => Some(Dim::from_index(other.index() + 1)),
    }
}
