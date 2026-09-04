use crate::geometry::LINEAR_TOLERANCE;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IntersectionOptions {
    /// Attempts to replace fitted NURBS output curves with supported analytical forms.
    pub simplify_curves: bool,
    pub linear_tolerance: f64,
    pub residual_tolerance: f64,
    pub fit_tolerance: f64,
    pub parameter_tolerance: f64,
    pub angular_tolerance: f64,
    pub bbox_tolerance: f64,
    pub max_subdivision_depth: usize,
    pub leaf_diagonal_tolerance: f64,
    pub newton_max_iterations: usize,
    pub min_trace_step: f64,
    pub max_trace_step: f64,
    pub max_trace_steps: usize,
}

impl IntersectionOptions {
    pub fn validate(self) -> bool {
        self.linear_tolerance.is_finite()
            && self.linear_tolerance > 0.0
            && self.residual_tolerance.is_finite()
            && self.residual_tolerance > 0.0
            && self.fit_tolerance.is_finite()
            && self.fit_tolerance > 0.0
            && self.parameter_tolerance.is_finite()
            && self.parameter_tolerance > 0.0
            && self.angular_tolerance.is_finite()
            && self.angular_tolerance > 0.0
            && self.bbox_tolerance.is_finite()
            && self.bbox_tolerance >= 0.0
            && self.leaf_diagonal_tolerance.is_finite()
            && self.leaf_diagonal_tolerance > 0.0
            && self.max_subdivision_depth > 0
            && self.newton_max_iterations > 0
            && self.min_trace_step.is_finite()
            && self.min_trace_step > 0.0
            && self.max_trace_step.is_finite()
            && self.max_trace_step >= self.min_trace_step
            && self.max_trace_steps > 0
    }

    pub fn linear_tolerance_squared(self) -> f64 {
        self.linear_tolerance * self.linear_tolerance
    }
}

impl Default for IntersectionOptions {
    fn default() -> Self {
        Self {
            simplify_curves: true,
            linear_tolerance: LINEAR_TOLERANCE,
            residual_tolerance: LINEAR_TOLERANCE,
            fit_tolerance: LINEAR_TOLERANCE.sqrt(),
            parameter_tolerance: 1.0e-10,
            angular_tolerance: 1.0e-8,
            bbox_tolerance: LINEAR_TOLERANCE,
            max_subdivision_depth: 32,
            leaf_diagonal_tolerance: LINEAR_TOLERANCE * 10.0,
            newton_max_iterations: 20,
            min_trace_step: 1.0e-6,
            max_trace_step: 2.0e-2,
            max_trace_steps: 4096,
        }
    }
}
