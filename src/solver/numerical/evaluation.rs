use super::Jet;
use crate::solver::SolveOptions;
use nalgebra::{DMatrix, DVector};
use std::ops::Range;

/// One residual and whether it is measured as an angle or as a normalized length.
pub(crate) struct Row {
    pub expression: Jet,
    pub angular: bool,
}
impl Row {
    pub fn linear(expression: Jet) -> Self {
        Self {
            expression,
            angular: false,
        }
    }
    pub fn angular(expression: Jet) -> Self {
        Self {
            expression,
            angular: true,
        }
    }
}

/// Residuals and their dense Jacobian at one point, with the rows each relation emitted.
pub(crate) struct Evaluation<K> {
    pub values: DVector<f64>,
    pub jacobian: DMatrix<f64>,
    pub angular: Vec<bool>,
    pub groups: Vec<(K, Range<usize>)>,
}
impl<K> Evaluation<K> {
    pub fn new(rows: Vec<Row>, groups: Vec<(K, Range<usize>)>, columns: usize) -> Self {
        let mut jacobian = DMatrix::zeros(rows.len(), columns);
        for (i, row) in rows.iter().enumerate() {
            for &(j, v) in &row.expression.gradient {
                jacobian[(i, j)] = v;
            }
        }
        Self {
            values: DVector::from_iterator(rows.len(), rows.iter().map(|r| r.expression.value)),
            jacobian,
            angular: rows.iter().map(|r| r.angular).collect(),
            groups,
        }
    }
    pub fn finite(&self) -> bool {
        self.values
            .iter()
            .chain(self.jacobian.iter())
            .all(|v| v.is_finite())
    }
    /// Every row within tolerance, linear rows measured in units of `scale`.
    pub fn feasible(&self, scale: f64, options: &SolveOptions) -> bool {
        self.finite()
            && self.values.iter().zip(&self.angular).all(|(v, a)| {
                v.abs()
                    <= if *a {
                        options.angular_tolerance
                    } else {
                        options.linear_tolerance / scale
                    }
            })
    }
}

/// Per relation, its residuals in physical units and their largest ratio to tolerance.
pub(crate) fn residuals<K: Copy>(
    scale: f64,
    evaluation: &Evaluation<K>,
    options: &SolveOptions,
) -> Vec<(K, Vec<f64>, f64)> {
    evaluation
        .groups
        .iter()
        .map(|(key, range)| {
            let mut values = vec![];
            let mut max_ratio = 0_f64;
            for row in range.clone() {
                let angular = evaluation.angular[row];
                let physical = evaluation.values[row] * if angular { 1. } else { scale };
                let tolerance = if angular {
                    options.angular_tolerance
                } else {
                    options.linear_tolerance
                };
                let ratio = physical.abs() / tolerance;
                max_ratio = if ratio.is_finite() {
                    max_ratio.max(ratio)
                } else {
                    f64::INFINITY
                };
                values.push(physical);
            }
            (*key, values, max_ratio)
        })
        .collect()
}

/// Per relation in order, how many of its rows depend on the rows before them.
///
/// The count is a local linearized one: it reads the Jacobian at this point and says
/// nothing of whether a nonlinear relation could be removed.
pub(crate) fn dependent_rows<K>(evaluation: &Evaluation<K>, tolerance: f64) -> Vec<usize> {
    let mut basis: Vec<DVector<f64>> = vec![];
    evaluation
        .groups
        .iter()
        .map(|(_, range)| {
            let mut dependent = 0_usize;
            for row in range.clone() {
                let mut v = evaluation.jacobian.row(row).transpose();
                let norm = v.norm();
                for _ in 0..2 {
                    for b in &basis {
                        let projection = b.dot(&v);
                        v -= b * projection;
                    }
                }
                let remaining = v.norm();
                if remaining <= tolerance * norm.max(1.) {
                    dependent += 1;
                } else {
                    basis.push(v / remaining);
                }
            }
            dependent
        })
        .collect()
}
