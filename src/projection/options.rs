use crate::geometry::LINEAR_TOLERANCE;

/// What a projection tolerates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProjectOptions {
    /// The distance within which two points coincide: nearest points this close in
    /// distance to the source are equally near, and a hit this far behind the
    /// source's origin is still on the half-line.
    pub tolerance: f64,
}

impl Default for ProjectOptions {
    fn default() -> Self {
        Self {
            tolerance: LINEAR_TOLERANCE,
        }
    }
}
