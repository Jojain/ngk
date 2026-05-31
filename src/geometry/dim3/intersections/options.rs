use crate::geometry::LINEAR_TOLERANCE;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IntersectionOptions {
    pub linear_tolerance: f64,
    pub parameter_tolerance: f64,
    pub bbox_tolerance: f64,
    pub max_subdivision_depth: usize,
    pub leaf_diagonal_tolerance: f64,
    pub newton_max_iterations: usize,
    pub curve_sample_count: usize,
    pub surface_u_sample_count: usize,
    pub surface_v_sample_count: usize,
}

impl IntersectionOptions {
    pub fn validate(self) -> bool {
        self.linear_tolerance.is_finite()
            && self.linear_tolerance > 0.0
            && self.parameter_tolerance.is_finite()
            && self.parameter_tolerance > 0.0
            && self.bbox_tolerance.is_finite()
            && self.bbox_tolerance >= 0.0
            && self.leaf_diagonal_tolerance.is_finite()
            && self.leaf_diagonal_tolerance > 0.0
            && self.max_subdivision_depth > 0
            && self.newton_max_iterations > 0
            && self.curve_sample_count > 0
            && self.surface_u_sample_count > 0
            && self.surface_v_sample_count > 0
    }

    pub fn linear_tolerance_squared(self) -> f64 {
        self.linear_tolerance * self.linear_tolerance
    }
}

impl Default for IntersectionOptions {
    fn default() -> Self {
        Self {
            linear_tolerance: LINEAR_TOLERANCE,
            parameter_tolerance: 1.0e-10,
            bbox_tolerance: LINEAR_TOLERANCE,
            max_subdivision_depth: 32,
            leaf_diagonal_tolerance: LINEAR_TOLERANCE * 10.0,
            newton_max_iterations: 20,
            curve_sample_count: 64,
            surface_u_sample_count: 32,
            surface_v_sample_count: 32,
        }
    }
}
