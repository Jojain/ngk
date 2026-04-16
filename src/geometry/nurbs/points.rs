use crate::geometry::utils::Point3;
use nalgebra::Point4;

use super::error::NurbsError;

/// A homogeneous control point stored as `(x·w, y·w, z·w, w)`.
///
/// This is the canonical rational-spline representation: evaluation algorithms
/// (Cox-de Boor, de Boor, knot insertion) all run directly on `Point4`, with a
/// single perspective divide at the very end to recover the cartesian point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HPoint(pub Point4<f64>);

impl HPoint {
    pub fn new(x_w: f64, y_w: f64, z_w: f64, w: f64) -> Self {
        Self(Point4::new(x_w, y_w, z_w, w))
    }

    pub fn from_cartesian(p: Point3, weight: f64) -> Self {
        Self(Point4::new(
            p.x * weight,
            p.y * weight,
            p.z * weight,
            weight,
        ))
    }

    pub fn to_cartesian(self) -> Point3 {
        let w = self.0.w;
        Point3::new(self.0.x / w, self.0.y / w, self.0.z / w)
    }

    pub fn weight(self) -> f64 {
        self.0.w
    }

    /// The pre-divide xyz components (i.e. `x·w, y·w, z·w`). Useful for linear
    /// combinations in Cox-de Boor before the final divide.
    pub fn weighted_xyz(self) -> nalgebra::Vector3<f64> {
        nalgebra::Vector3::new(self.0.x, self.0.y, self.0.z)
    }
}

impl From<Point4<f64>> for HPoint {
    fn from(p: Point4<f64>) -> Self {
        Self(p)
    }
}

/// The 1D control-point sequence of a NURBS curve.
#[derive(Debug, Clone)]
pub struct ControlPolygon(Vec<HPoint>);

impl ControlPolygon {
    pub fn new(points: Vec<HPoint>) -> Result<Self, NurbsError> {
        if points.is_empty() {
            Err(NurbsError::EmptyControlPolygon)
        } else {
            Ok(Self(points))
        }
    }

    pub fn from_cartesian(points: Vec<Point3>, weights: &[f64]) -> Result<Self, NurbsError> {
        if points.len() != weights.len() {
            return Err(NurbsError::WeightCountMismatch {
                expected: points.len(),
                got: weights.len(),
            });
        }
        Self::new(
            points
                .into_iter()
                .zip(weights.iter().copied())
                .map(|(p, w)| HPoint::from_cartesian(p, w))
                .collect(),
        )
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    pub fn as_slice(&self) -> &[HPoint] {
        &self.0
    }

    pub fn as_mut_slice(&mut self) -> &mut [HPoint] {
        &mut self.0
    }

    pub fn get(&self, i: usize) -> Option<&HPoint> {
        self.0.get(i)
    }

    pub fn set(&mut self, i: usize, p: HPoint) {
        self.0[i] = p;
    }

    pub fn push(&mut self, p: HPoint) {
        self.0.push(p);
    }

    pub fn iter(&self) -> std::slice::Iter<'_, HPoint> {
        self.0.iter()
    }

    pub(crate) fn into_inner(self) -> Vec<HPoint> {
        self.0
    }

    pub(crate) fn inner_mut(&mut self) -> &mut Vec<HPoint> {
        &mut self.0
    }
}

/// A 2D control grid for NURBS surfaces, stored flat in row-major order
/// (u varies fastest). Has `nu * nv` entries.
#[derive(Debug, Clone)]
pub struct ControlNet {
    points: Vec<HPoint>,
    nu: usize,
    nv: usize,
}

impl ControlNet {
    pub fn new(points: Vec<HPoint>, nu: usize, nv: usize) -> Result<Self, NurbsError> {
        if nu == 0 || nv == 0 {
            return Err(NurbsError::EmptyControlPolygon);
        }
        let expected = nu * nv;
        if points.len() != expected {
            return Err(NurbsError::ControlNetDimensionMismatch {
                expected,
                got: points.len(),
            });
        }
        Ok(Self { points, nu, nv })
    }

    pub fn from_cartesian(
        points: Vec<Point3>,
        weights: &[f64],
        nu: usize,
        nv: usize,
    ) -> Result<Self, NurbsError> {
        if points.len() != weights.len() {
            return Err(NurbsError::WeightCountMismatch {
                expected: points.len(),
                got: weights.len(),
            });
        }
        Self::new(
            points
                .into_iter()
                .zip(weights.iter().copied())
                .map(|(p, w)| HPoint::from_cartesian(p, w))
                .collect(),
            nu,
            nv,
        )
    }

    pub fn nu(&self) -> usize {
        self.nu
    }

    pub fn nv(&self) -> usize {
        self.nv
    }

    pub fn get(&self, iu: usize, iv: usize) -> HPoint {
        self.points[iv * self.nu + iu]
    }

    pub fn set(&mut self, iu: usize, iv: usize, p: HPoint) {
        self.points[iv * self.nu + iu] = p;
    }

    pub fn as_slice(&self) -> &[HPoint] {
        &self.points
    }
}
