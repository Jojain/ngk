use slotmap::new_key_type;

new_key_type! { pub(crate) struct GeometrySlot; pub(crate) struct ConstraintSlot; }

/// Identity of a geometric primitive within its originating solver.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct GeometryKey {
    pub(crate) owner: u64,
    pub(crate) slot: GeometrySlot,
    pub(crate) serial: u64,
}

macro_rules! geometry_key {
    ($name:ident, $doc:literal) => {
        #[doc = $doc]
        #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
        pub struct $name(pub(crate) GeometryKey);
        impl From<$name> for GeometryKey {
            fn from(key: $name) -> Self {
                key.0
            }
        }
    };
}
geometry_key!(PointKey, "Identity of an independent point.");
geometry_key!(SegmentKey, "Identity of a directed segment.");
geometry_key!(CircleKey, "Identity of a full circle.");
geometry_key!(ArcKey, "Identity of a directed circular arc.");

/// Identity of a permanent constraint within its originating solver.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ConstraintKey {
    pub(crate) owner: u64,
    pub(crate) slot: ConstraintSlot,
    pub(crate) serial: u64,
}

/// A point's geometric role, independent of the current numerical layout.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct PointRef {
    pub(crate) geometry: GeometryKey,
    pub(crate) role: PointRole,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) enum PointRole {
    Point,
    Start,
    End,
    Center,
}

impl From<PointKey> for PointRef {
    fn from(key: PointKey) -> Self {
        Self {
            geometry: key.0,
            role: PointRole::Point,
        }
    }
}
impl SegmentKey {
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
impl CircleKey {
    pub fn center(self) -> PointRef {
        PointRef {
            geometry: self.0,
            role: PointRole::Center,
        }
    }
}
impl ArcKey {
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
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct CircularKey(pub(crate) GeometryKey);
impl From<CircleKey> for CircularKey {
    fn from(key: CircleKey) -> Self {
        Self(key.0)
    }
}
impl From<ArcKey> for CircularKey {
    fn from(key: ArcKey) -> Self {
        Self(key.0)
    }
}
impl From<CircularKey> for GeometryKey {
    fn from(key: CircularKey) -> Self {
        key.0
    }
}
