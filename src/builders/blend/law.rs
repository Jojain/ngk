//! The cross-section a blend puts along an edge or across a corner.
//!
//! A law is read in exactly two places: the edge-section solvers and the 2D
//! corner solver. Vertex treatments never read it, only the sections it
//! produced, so blends of different laws meeting at one vertex are decided by
//! what they built rather than by what was asked for.

use super::errors::BlendError;

/// A chamfer's cross-section rule.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum ChamferLaw {
    /// Both rails set back the same distance from the edge or corner.
    Distance(f64),
}

/// A fillet's cross-section rule.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum FilletLaw {
    /// A ball of constant radius touching both faces, or a circle touching
    /// both edges of a corner.
    Radius(f64),
}

/// What a blend puts where it removes a crease or a corner.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum BlendLaw {
    Chamfer(ChamferLaw),
    Fillet(FilletLaw),
}

impl BlendLaw {
    /// Refuses a law no blend could be built from.
    pub(crate) fn validate(self) -> Result<(), BlendError> {
        match self {
            Self::Chamfer(ChamferLaw::Distance(distance)) => (distance.is_finite()
                && distance > 0.0)
                .then_some(())
                .ok_or(BlendError::InvalidDistance { distance }),
            Self::Fillet(FilletLaw::Radius(radius)) => (radius.is_finite() && radius > 0.0)
                .then_some(())
                .ok_or(BlendError::InvalidRadius { radius }),
        }
    }

    /// Whether this law rounds rather than bevels.
    pub(crate) fn is_fillet(self) -> bool {
        matches!(self, Self::Fillet(_))
    }
}
