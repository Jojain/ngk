use nalgebra::{DMatrix, DVector};

/// The smallest eigenvalue counted as a free direction of a parameter group. Eigenvalues
/// are squared singular values of the null-space projection, whose rounding noise is near
/// 1e-16, while a real freedom shared across a whole component stays near 1/columns.
const FREE_DIRECTION: f64 = 1e-12;

/// Thin SVD supplies minimum-norm corrections and projections even at deficient rank.
pub(crate) struct Factor {
    u: DMatrix<f64>,
    vt: DMatrix<f64>,
    singular: DVector<f64>,
    threshold: f64,
    columns: usize,
}
impl Factor {
    pub fn new(j: &DMatrix<f64>, tolerance: f64) -> Self {
        if j.nrows() == 0 || j.ncols() == 0 {
            return Self {
                u: DMatrix::zeros(j.nrows(), 0),
                vt: DMatrix::zeros(0, j.ncols()),
                singular: DVector::zeros(0),
                threshold: tolerance,
                columns: j.ncols(),
            };
        }
        let svd = j.clone().svd(true, true);
        let threshold = tolerance * svd.singular_values.max().max(1.);
        Self {
            u: svd.u.expect("requested left singular vectors"),
            vt: svd.v_t.expect("requested right singular vectors"),
            singular: svd.singular_values,
            threshold,
            columns: j.ncols(),
        }
    }
    pub fn rank(&self) -> usize {
        self.significant_rows().count()
    }
    /// The rows of `vt` spanning the Jacobian's row space.
    fn significant_rows(&self) -> impl Iterator<Item = usize> + '_ {
        (0..self.singular.len()).filter(|i| self.singular[*i] > self.threshold)
    }
    pub fn correction(&self, rhs: &DVector<f64>, damping: f64) -> DVector<f64> {
        let mut output = DVector::zeros(self.columns);
        for i in 0..self.singular.len() {
            let s = self.singular[i];
            if s <= self.threshold {
                continue;
            }
            let coefficient = self.u.column(i).dot(rhs) * s / (s * s + damping);
            for j in 0..self.columns {
                output[j] += self.vt[(i, j)] * coefficient;
            }
        }
        output
    }
    pub fn null_projection(&self, v: &DVector<f64>) -> DVector<f64> {
        let mut output = v.clone();
        for i in 0..self.singular.len() {
            if self.singular[i] <= self.threshold {
                continue;
            }
            let coefficient = self.vt.row(i).transpose().dot(v);
            for j in 0..self.columns {
                output[j] -= coefficient * self.vt[(i, j)];
            }
        }
        output
    }
    /// The rank of the null-space projection restricted to a group of parameters.
    ///
    /// Each slot of the group is either a constant (`None`) or `multiplier` times one
    /// unknown `column`. A motion shared with other groups counts in each of them.
    pub fn freedom(&self, slots: &[Option<(usize, f64)>]) -> usize {
        let count = slots.len();
        // M Mᵀ - (M Vᵣ)(M Vᵣ)ᵀ, where M maps the unknowns to this group's parameters.
        let mut gram = DMatrix::zeros(count, count);
        for a in 0..count {
            let Some((column, multiplier)) = slots[a] else {
                continue;
            };
            for b in 0..count {
                if let Some((other, coefficient)) = slots[b]
                    && other == column
                {
                    gram[(a, b)] = multiplier * coefficient;
                }
            }
        }
        for row in self.significant_rows() {
            let projected = DVector::from_fn(count, |a, _| {
                slots[a].map_or(0., |(column, multiplier)| {
                    multiplier * self.vt[(row, column)]
                })
            });
            gram -= &projected * projected.transpose();
        }
        gram.symmetric_eigenvalues()
            .iter()
            .filter(|value| **value > FREE_DIRECTION)
            .count()
    }
}
