use std::collections::HashSet;

use crate::topology::gmap::{Dart, Dim, GMap};

use super::ownership::OwnershipIndex;

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
pub fn turn(map: &GMap, index: &OwnershipIndex, dimension: Dim, dart: Dart) -> Option<Dart> {
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
        if !index.is_scaffold(dimension, next) {
            return Some(next);
        }
        let hop = map.alpha(up?, next);
        if hop == next {
            return None;
        }
        current = hop;
    }
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
