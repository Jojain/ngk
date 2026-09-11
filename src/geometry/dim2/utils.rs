pub type Point2 = nalgebra::Point2<f64>;
pub type Vector2 = nalgebra::Vector2<f64>;

/// One of a surface's two parameter directions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum Axis2 {
    /// The first surface parameter.
    U,
    /// The second surface parameter.
    V,
}

impl Axis2 {
    /// Both axes, in parameter order.
    pub const ALL: [Axis2; 2] = [Axis2::U, Axis2::V];

    /// Returns this axis' index into a parameter pair.
    pub fn index(self) -> usize {
        match self {
            Axis2::U => 0,
            Axis2::V => 1,
        }
    }

    /// Returns the axis this one is transverse to.
    pub fn transverse(self) -> Self {
        match self {
            Axis2::U => Axis2::V,
            Axis2::V => Axis2::U,
        }
    }

    /// Reads this axis' coordinate out of a parameter-space point.
    pub fn of(self, point: Point2) -> f64 {
        point[self.index()]
    }
}

/// Which way along a parameter axis is meant.
///
/// A *side*, not a position: "below here" or "above here". Named rather than
/// signed because the two directions are not interchangeable — a sphere's `v`
/// runs from one pole to the other, and which one closes a face is the whole
/// difference between its two caps.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum DomainSide {
    /// Toward decreasing parameter.
    Low,
    /// Toward increasing parameter.
    High,
}

impl DomainSide {
    /// Returns the opposite side.
    pub fn flip(self) -> Self {
        match self {
            Self::Low => Self::High,
            Self::High => Self::Low,
        }
    }

    /// Returns which side of `from` the parameter `at` lies on.
    pub fn of(from: f64, at: f64) -> Self {
        if at < from { Self::Low } else { Self::High }
    }

    /// Returns whether `at` lies on this side of `from`.
    pub fn holds(self, from: f64, at: f64) -> bool {
        match self {
            Self::Low => at < from,
            Self::High => at > from,
        }
    }

    /// Returns the candidate on this side of `from` that is nearest to it.
    ///
    /// A surface can collapse more than once along one axis — a sphere does,
    /// once at each pole — so a side alone does not name a row; the nearest one
    /// on that side is what bounds the face.
    pub fn nearest(self, from: f64, candidates: impl IntoIterator<Item = f64>) -> Option<f64> {
        candidates
            .into_iter()
            .filter(|at| self.holds(from, *at))
            .min_by(|a, b| (a - from).abs().total_cmp(&(b - from).abs()))
    }
}
