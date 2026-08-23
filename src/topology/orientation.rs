use nalgebra::UnitVector3;

/// A relative orientation flag used by topology views.
///
/// When a traversal resolves a keyed entity inside a larger context, the local
/// direction may match the entity's default direction or be opposite.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Orientation {
    /// The local traversal direction matches the entity's default orientation.
    Same,
    /// The local traversal direction is opposite to the entity's default
    /// orientation.
    Reversed,
}

impl Orientation {
    /// Returns the flipped orientation.
    pub fn flip(self) -> Self {
        match self {
            Orientation::Same => Orientation::Reversed,
            Orientation::Reversed => Orientation::Same,
        }
    }

    /// Compose two orientations.
    ///
    /// `a.compose(b)` returns the result of applying `b` after `a`.
    pub fn compose(self, other: Self) -> Self {
        match (self, other) {
            (Orientation::Same, o) => o,
            (Orientation::Reversed, Orientation::Same) => Orientation::Reversed,
            (Orientation::Reversed, Orientation::Reversed) => Orientation::Same,
        }
    }

    /// Applies the orientation to a unit vector, negating when [`Reversed`](Orientation::Reversed).
    pub fn apply_vector(self, v: UnitVector3<f64>) -> UnitVector3<f64> {
        match self {
            Orientation::Same => v,
            Orientation::Reversed => -v,
        }
    }

    /// Applies the orientation to a scalar factor, negating when [`Reversed`](Orientation::Reversed).
    pub fn apply_scalar(self, s: f64) -> f64 {
        match self {
            Orientation::Same => s,
            Orientation::Reversed => -s,
        }
    }
}

impl Default for Orientation {
    fn default() -> Self {
        Orientation::Same
    }
}

impl std::ops::Not for Orientation {
    type Output = Self;
    fn not(self) -> Self::Output {
        self.flip()
    }
}
