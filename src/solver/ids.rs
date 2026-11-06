use super::primitives::Kind;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

/// Identity of a geometric primitive: its typed id, of any kind.
///
/// A system issues it when the primitive is added. Its value can be stored, and `new`
/// gives the id back to rebuild a system from it, where it means the same thing.
/// A system refuses an id already in use.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct GeometryId {
    pub(crate) kind: Kind,
    pub(crate) value: u64,
}

macro_rules! geometry_id {
    ($name:ident, $kind:expr, $doc:literal) => {
        #[doc = $doc]
        #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
        pub struct $name(pub(crate) GeometryId);
        impl $name {
            pub const fn new(value: u64) -> Self {
                Self(GeometryId { kind: $kind, value })
            }
            pub const fn value(self) -> u64 {
                self.0.value
            }
        }
        impl From<$name> for GeometryId {
            fn from(id: $name) -> Self {
                id.0
            }
        }
        impl Serialize for $name {
            fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
                self.0.value.serialize(serializer)
            }
        }
        impl<'de> Deserialize<'de> for $name {
            fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
                u64::deserialize(deserializer).map(Self::new)
            }
        }
    };
}
geometry_id!(PointId, Kind::Point, "Identity of an independent point.");
geometry_id!(SegmentId, Kind::Segment, "Identity of a directed segment.");
geometry_id!(CircleId, Kind::Circle, "Identity of a full circle.");
geometry_id!(ArcId, Kind::Arc, "Identity of a directed circular arc.");

/// Identity of a permanent constraint, issued by its system.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ConstraintId(u64);

impl ConstraintId {
    pub const fn new(value: u64) -> Self {
        Self(value)
    }
    pub const fn value(self) -> u64 {
        self.0
    }
}

/// A point's geometric role, independent of the current numerical layout.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(try_from = "StoredPointRef")]
pub struct PointRef {
    pub(crate) geometry: GeometryId,
    pub(crate) role: PointRole,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub(crate) enum PointRole {
    Point,
    Start,
    End,
    Center,
}

/// A point reference as read from storage, before its role is checked against its kind.
#[derive(Deserialize)]
struct StoredPointRef {
    geometry: GeometryId,
    role: PointRole,
}

impl TryFrom<StoredPointRef> for PointRef {
    type Error = &'static str;

    fn try_from(stored: StoredPointRef) -> Result<Self, Self::Error> {
        let valid = matches!(
            (stored.geometry.kind, stored.role),
            (Kind::Point, PointRole::Point)
                | (Kind::Segment, PointRole::Start | PointRole::End)
                | (Kind::Circle, PointRole::Center)
                | (
                    Kind::Arc,
                    PointRole::Start | PointRole::End | PointRole::Center
                )
        );
        if !valid {
            return Err("a primitive of this kind has no such point");
        }
        Ok(Self {
            geometry: stored.geometry,
            role: stored.role,
        })
    }
}

impl From<PointId> for PointRef {
    fn from(id: PointId) -> Self {
        Self {
            geometry: id.0,
            role: PointRole::Point,
        }
    }
}
impl SegmentId {
    pub fn start(self) -> PointRef {
        PointRef {
            geometry: self.0,
            role: PointRole::Start,
        }
    }
    pub fn end(self) -> PointRef {
        PointRef {
            geometry: self.0,
            role: PointRole::End,
        }
    }
}
impl CircleId {
    pub fn center(self) -> PointRef {
        PointRef {
            geometry: self.0,
            role: PointRole::Center,
        }
    }
}
impl ArcId {
    pub fn start(self) -> PointRef {
        PointRef {
            geometry: self.0,
            role: PointRole::Start,
        }
    }
    pub fn end(self) -> PointRef {
        PointRef {
            geometry: self.0,
            role: PointRole::End,
        }
    }
    pub fn center(self) -> PointRef {
        PointRef {
            geometry: self.0,
            role: PointRole::Center,
        }
    }
}

/// A circle or the circle supporting an arc.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(try_from = "GeometryId", into = "GeometryId")]
pub struct CircularId(pub(crate) GeometryId);
impl From<CircleId> for CircularId {
    fn from(id: CircleId) -> Self {
        Self(id.0)
    }
}
impl From<ArcId> for CircularId {
    fn from(id: ArcId) -> Self {
        Self(id.0)
    }
}
impl From<CircularId> for GeometryId {
    fn from(id: CircularId) -> Self {
        id.0
    }
}
impl TryFrom<GeometryId> for CircularId {
    type Error = &'static str;

    fn try_from(id: GeometryId) -> Result<Self, Self::Error> {
        match id.kind {
            Kind::Circle | Kind::Arc => Ok(Self(id)),
            Kind::Point | Kind::Segment => Err("expected a circle or arc"),
        }
    }
}
