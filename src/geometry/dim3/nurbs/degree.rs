use crate::geometry::nurbs::error::NurbsError;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct Degree(usize);

impl Degree {
    pub fn new(p: usize) -> Result<Self, NurbsError> {
        if p == 0 {
            Err(NurbsError::DegreeZero)
        } else {
            Ok(Self(p))
        }
    }

    pub const fn get(self) -> usize {
        self.0
    }
}

impl From<Degree> for usize {
    fn from(d: Degree) -> usize {
        d.0
    }
}
