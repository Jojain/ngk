use std::ops::Deref;

use crate::geometry::Plane;

use super::closed::Closed;
use super::edge::Edge;
use super::payload::Payload;
use super::profile::Profile;

/// Marker for topology views that can carry a verified support plane.
pub trait PlanarGeometry {}

impl<'a, P: Payload> PlanarGeometry for Edge<'a, P> {}
impl<'a, P: Payload> PlanarGeometry for Profile<'a, P> {}
impl<'a, P: Payload> PlanarGeometry for Closed<Profile<'a, P>> {}

/// Wrapper that statically carries "the inner value lies on this plane".
///
/// The invariant is trusted at construction time. If the underlying topology or
/// geometry mutates afterwards, the wrapper does not re-check planarity.
pub struct Planar<T: PlanarGeometry> {
    inner: T,
    plane: Plane,
}

impl<T: PlanarGeometry + Clone> Clone for Planar<T> {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
            plane: self.plane.clone(),
        }
    }
}

impl<T: PlanarGeometry> Planar<T> {
    pub fn new_unchecked(inner: T, plane: Plane) -> Self {
        Self { inner, plane }
    }

    pub fn into_inner(self) -> T {
        self.inner
    }

    pub fn inner(&self) -> &T {
        &self.inner
    }

    pub fn plane(&self) -> &Plane {
        &self.plane
    }

    pub fn into_parts(self) -> (T, Plane) {
        (self.inner, self.plane)
    }
}

impl<T: PlanarGeometry> Deref for Planar<T> {
    type Target = T;

    fn deref(&self) -> &T {
        &self.inner
    }
}

pub type PlanarLoop<'a, P> = Planar<Closed<Profile<'a, P>>>;
pub type PlanarProfile<'a, P> = Planar<Profile<'a, P>>;
