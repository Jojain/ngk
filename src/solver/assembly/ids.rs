use serde::{Deserialize, Serialize};

/// Identity of a body, issued by its assembly and never issued twice.
///
/// Its value can be stored, and `new` gives the id back to rebuild an assembly from
/// it, where it means the same thing. An assembly refuses an id already in use.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct BodyId(u64);

impl BodyId {
    pub const fn new(value: u64) -> Self {
        Self(value)
    }
    pub const fn value(self) -> u64 {
        self.0
    }
}

/// Identity of a joint, issued by its assembly and never issued twice.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct JointId(u64);

impl JointId {
    pub const fn new(value: u64) -> Self {
        Self(value)
    }
    pub const fn value(self) -> u64 {
        self.0
    }
}
