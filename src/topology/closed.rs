use std::ops::Deref;

/// Types that have a notion of being "closed" — e.g. a profile with no dangling
/// endpoints, or a sheet with no free edge. Each implementor decides what
/// "closed" means for itself.
pub trait Closeable {
    fn is_closed(&self) -> bool;
}

/// Zero-cost wrapper that statically carries "the inner value was verified
/// closed at construction time". Reached via [`Closed::new`] (checked) or
/// [`Closed::new_unchecked`] (trusted).
///
/// The invariant is only validated at construction — if the underlying
/// topology mutates afterwards, the wrapper does not re-check.
pub struct Closed<T>(T);

impl<T: Clone> Clone for Closed<T> {
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl<T: Closeable> Closed<T> {
    /// Validates `inner.is_closed()` and wraps on success.
    pub fn new(inner: T) -> Option<Self> {
        if inner.is_closed() {
            Some(Self(inner))
        } else {
            None
        }
    }
}

impl<T> Closed<T> {
    /// Bypass the closedness check. Use only when the caller has a structural
    /// guarantee (e.g. the value was produced by an operation that preserves
    /// closure by construction).
    pub fn new_unchecked(inner: T) -> Self {
        Self(inner)
    }

    pub fn into_inner(self) -> T {
        self.0
    }

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
