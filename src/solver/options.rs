/// Physical tolerances and bounded work for local 2D solving.
#[derive(Clone, Debug)]
pub struct SolveOptions {
    /// Eliminates exact coordinate equalities and captured constants before factorization.
    pub linear_reduction: bool,
    pub linear_tolerance: f64,
    pub angular_tolerance: f64,
    /// Relative threshold for the scaled Jacobian's numerical rank.
    pub rank_tolerance: f64,
    pub target_tolerance: f64,
    pub max_iterations: usize,
    pub max_target_iterations: usize,
}

impl Default for SolveOptions {
    fn default() -> Self {
        Self {
            linear_reduction: true,
            linear_tolerance: 1e-8,
            angular_tolerance: 1e-8,
            rank_tolerance: 1e-10,
            target_tolerance: 1e-6,
            max_iterations: 120,
            max_target_iterations: 80,
        }
    }
}
impl SolveOptions {
    pub(crate) fn valid(&self) -> bool {
        [
            self.linear_tolerance,
            self.angular_tolerance,
            self.rank_tolerance,
            self.target_tolerance,
        ]
        .iter()
        .all(|v| v.is_finite() && *v > 0.)
            && self.rank_tolerance < 1.
            && self.max_iterations > 0
            && self.max_target_iterations > 0
    }
}
