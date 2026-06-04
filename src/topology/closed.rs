use std::ops::Deref;

use super::gmap::{MergeTopology, TopologyMerge};
use super::payload::Payload;

/// Trait for topology views that can report whether they are closed.
///
/// Each implementor defines the invariant in its own dimension. For example,
/// a profile is closed when it has no free alpha0/alpha1 endpoints, while a
/// sheet is closed when it has no free alpha0/alpha1/alpha2 boundary darts.
pub trait Closeable {
    /// Returns whether the topology currently satisfies its closedness
    /// invariant.
    fn is_closed(&self) -> bool;
}

/// Wrapper carrying the invariant that a topology value is closed.
///
/// The invariant is checked at construction by [`Closed::new`] or trusted by
/// [`Closed::new_unchecked`]. The wrapper does not observe later mutations of
/// the underlying map, so recreate it after structural edits.
pub struct Closed<T>(T);

impl<T: Clone> Clone for Closed<T> {
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl<T: Closeable> Closed<T> {
    /// Validates `inner.is_closed()` and wraps the value on success.
    ///
    /// Returns `None` when the value is currently open.
    pub fn new(inner: T) -> Option<Self> {
        if inner.is_closed() {
            Some(Self(inner))
        } else {
            None
        }
    }
}

impl<T> Closed<T> {
    /// Wraps a value without checking its closedness invariant.
    ///
    /// Use this only when the caller has a structural guarantee, such as a
    /// builder that produces closed topology by construction.
    pub fn new_unchecked(inner: T) -> Self {
        Self(inner)
    }

    /// Consumes the wrapper and returns the inner topology value.
    pub fn into_inner(self) -> T {
        self.0
    }

    /// Returns the wrapped topology value by shared reference.
    pub fn inner(&self) -> &T {
        &self.0
    }
}

impl<T> Deref for Closed<T> {
    type Target = T;
    fn deref(&self) -> &T {
        &self.0
    }
}

impl<P, T> MergeTopology<P> for Closed<T>
where
    P: Payload,
    T: MergeTopology<P>,
{
    fn merge_topology(&self) -> TopologyMerge<'_, P> {
        self.0.merge_topology()
    }
}
