use serde::{Deserialize, Serialize};

/// Identifier for one dart in a generalized map.
///
/// Darts are the atomic elements of the topology. Higher-level cells are
/// represented as orbits of darts under selected alpha involutions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Ord, PartialOrd, Serialize, Deserialize)]
pub struct Dart(usize);

impl Dart {
    /// Creates a dart identifier from a zero-based storage index.
    pub fn new(id: usize) -> Self {
        Self(id)
    }

    /// Returns this dart's zero-based storage index.
    pub fn id(&self) -> usize {
        self.0
    }
}

/// A dart known to be isolated from all alpha links.
///
/// This wrapper is used for operations that require the caller to prove a dart
/// can be safely removed without breaking adjacent topology.
pub struct IsolatedDart(Dart);

impl IsolatedDart {
    /// Wraps a dart as isolated.
    ///
    /// The invariant is trusted by the type. Callers should only construct this
    /// value after checking that every alpha maps the dart to itself.
    pub fn new(dart: Dart) -> Self {
        Self(dart)
    }

    /// Returns the wrapped dart.
    pub fn dart(&self) -> Dart {
        self.0
    }

    /// Returns the wrapped dart's zero-based storage index.
    pub fn id(&self) -> usize {
        self.0.id()
    }
}
