//! One immutable tolerance budget for a Boolean operation.

use crate::geometry::{IntersectionOptions, Point3, Surface};
use crate::topology::gmap::GMap;
use crate::topology::payload::Payload;

use super::{BooleanError, operand::OperandCells};

/// How the operation derives its geometric tolerances.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BooleanTolerancePolicy {
    ModelScaled { base_linear: f64 },
    Fixed(BooleanTolerances),
}

impl Default for BooleanTolerancePolicy {
    fn default() -> Self {
        Self::ModelScaled {
            base_linear: IntersectionOptions::default().linear_tolerance,
        }
    }
}

/// Resolved distances and dimensionless parameter/angular tolerances.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BooleanTolerances {
    pub linear: f64,
    pub residual: f64,
    pub parameter: f64,
    pub angular: f64,
    pub bbox: f64,
    pub probe_margin: f64,
    /// How far a fitted section may sit from its own supporting surfaces.
    ///
    /// A curved section is an approximation produced by the surface/surface
    /// fitter, so the network cannot demand more agreement between a section
    /// curve and its pcurves than that fitter promises. Exact planar sections
    /// stay far inside this budget.
    pub section_fit: f64,
    pub model_scale: f64,
}

impl Default for BooleanTolerances {
    fn default() -> Self {
        Self::resolve(BooleanTolerancePolicy::default(), 1.0).expect("valid default tolerances")
    }
}

impl BooleanTolerances {
    /// Resolves and validates a policy. Fixed values, including scale, are preserved.
    /// Model scale must be finite and positive; invalid or overflowing budgets fail.
    pub fn resolve(policy: BooleanTolerancePolicy, model_scale: f64) -> Result<Self, BooleanError> {
        if !model_scale.is_finite() || model_scale <= 0.0 {
            return Err(BooleanError::InvalidTolerances);
        }
        let result = match policy {
            BooleanTolerancePolicy::Fixed(tolerances) => tolerances,
            BooleanTolerancePolicy::ModelScaled { base_linear } => {
                let defaults = IntersectionOptions::default();
                let linear = base_linear * model_scale;
                Self {
                    linear,
                    residual: linear,
                    parameter: defaults.parameter_tolerance,
                    angular: defaults.angular_tolerance,
                    bbox: linear,
                    probe_margin: linear * 100.0,
                    section_fit: defaults.fit_tolerance * model_scale,
                    model_scale,
                }
            }
        };
        if [
            result.linear,
            result.residual,
            result.parameter,
            result.angular,
            result.probe_margin,
            result.section_fit,
            result.model_scale,
        ]
        .iter()
        .any(|value| !value.is_finite() || *value <= 0.0)
            || !result.bbox.is_finite()
            || result.bbox < 0.0
        {
            return Err(BooleanError::InvalidTolerances);
        }
        Ok(result)
    }

    /// Estimates combined model extent from geometric control hulls, never just vertices.
    pub(crate) fn from_cells<P: Payload>(
        map: &GMap<P>,
        first: &OperandCells,
        second: &OperandCells,
        policy: BooleanTolerancePolicy,
    ) -> Result<Self, BooleanError> {
        let mut points = Vec::new();
        for cells in [first, second] {
            for &vertex in &cells.vertices {
                points.push(
                    *map.vertex_unchecked(vertex)
                        .point()
                        .expect("admitted geometry"),
                );
            }
            for &edge in &cells.edges {
                let curve = map
                    .edge_unchecked(edge)
                    .curve()
                    .expect("admitted geometry")
                    .to_nurbs()?;
                points.extend(
                    curve
                        .control_points()
                        .as_slice()
                        .iter()
                        .map(|point| point.to_cartesian()),
                );
            }
            for &face in &cells.faces {
                if let Surface::Nurbs(surface) = map.face_unchecked(face).surface() {
                    points.extend(
                        surface
                            .control_points()
                            .as_slice()
                            .iter()
                            .map(|point| point.to_cartesian()),
                    );
                }
            }
        }
        let Some(&first) = points.first() else {
            return Err(BooleanError::InvalidTolerances);
        };
        let (mut min, mut max): (Point3, Point3) = (first, first);
        for point in points {
            if !point.coords.iter().all(|value| value.is_finite()) {
                return Err(BooleanError::InvalidTolerances);
            }
            min = Point3::from(min.coords.inf(&point.coords));
            max = Point3::from(max.coords.sup(&point.coords));
        }
        let scale = (max - min).norm();
        // Coincident isolated vertices have no length scale; preparation still admits them.
        Self::resolve(policy, if scale == 0.0 { 1.0 } else { scale })
    }

    /// Applies geometric budgets while preserving the caller's iteration and fit controls.
    pub(crate) fn apply(self, options: &mut IntersectionOptions) {
        options.linear_tolerance = self.linear;
        options.residual_tolerance = self.residual;
        options.parameter_tolerance = self.parameter;
        options.angular_tolerance = self.angular;
        options.bbox_tolerance = self.bbox;
    }
}
