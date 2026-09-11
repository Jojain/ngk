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
