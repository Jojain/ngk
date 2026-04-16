use nalgebra::{Point4, Vector3, Vector4};

use super::basis::{basis_function_derivatives, basis_functions};
use super::degree::Degree;
use super::error::NurbsError;
use super::knots::KnotVector;
use super::points::{ControlNet, HPoint};
use crate::geometry::utils::Point3;

#[derive(Debug, Clone)]
pub struct NurbsSurface {
    degree_u: Degree,
    degree_v: Degree,
    control_points: ControlNet,
    knots_u: KnotVector,
    knots_v: KnotVector,
}

impl NurbsSurface {
    pub fn new(
        degree_u: Degree,
        degree_v: Degree,
        control_points: ControlNet,
        knots_u: KnotVector,
        knots_v: KnotVector,
    ) -> Result<Self, NurbsError> {
        let expected_u = control_points.nu() + degree_u.get() + 1;
        if knots_u.len() != expected_u {
            return Err(NurbsError::KnotCountMismatch {
                expected: expected_u,
                got: knots_u.len(),
            });
        }
        let expected_v = control_points.nv() + degree_v.get() + 1;
        if knots_v.len() != expected_v {
            return Err(NurbsError::KnotCountMismatch {
                expected: expected_v,
                got: knots_v.len(),
            });
        }
        Ok(Self {
            degree_u,
            degree_v,
            control_points,
            knots_u,
            knots_v,
        })
    }

    pub fn with_uniform_knots(
        degree_u: Degree,
        degree_v: Degree,
        control_points: ControlNet,
    ) -> Result<Self, NurbsError> {
        let knots_u = KnotVector::uniform_clamped(control_points.nu(), degree_u);
        let knots_v = KnotVector::uniform_clamped(control_points.nv(), degree_v);
        Self::new(degree_u, degree_v, control_points, knots_u, knots_v)
    }

    pub fn degree_u(&self) -> Degree {
        self.degree_u
    }
    pub fn degree_v(&self) -> Degree {
        self.degree_v
    }
    pub fn control_points(&self) -> &ControlNet {
        &self.control_points
    }
    pub fn knots_u(&self) -> &KnotVector {
        &self.knots_u
    }
    pub fn knots_v(&self) -> &KnotVector {
        &self.knots_v
    }

    pub fn domain_u(&self) -> (f64, f64) {
        self.knots_u.domain(self.degree_u)
    }
    pub fn domain_v(&self) -> (f64, f64) {
        self.knots_v.domain(self.degree_v)
    }

    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        let (min_u, max_u) = self.domain_u();
        let (min_v, max_v) = self.domain_v();
        let u = u.clamp(min_u, max_u);
        let v = v.clamp(min_v, max_v);

        let p = self.degree_u.get();
        let q = self.degree_v.get();
        let n = self.control_points.nu() - 1;
        let m = self.control_points.nv() - 1;

        let span_u = self.knots_u.find_span(n, self.degree_u, u);
        let span_v = self.knots_v.find_span(m, self.degree_v, v);
        let nu = basis_functions(span_u, u, self.degree_u, &self.knots_u);
        let nv = basis_functions(span_v, v, self.degree_v, &self.knots_v);

        let mut acc = Point4::origin();
        for j in 0..=q {
            let mut temp = Point4::origin();
            for i in 0..=p {
                let hp = self.control_points.get(span_u - p + i, span_v - q + j);
                let contrib: Vector4<f64> = hp.0.coords * nu[i];
                temp.coords += contrib;
            }
            acc.coords += temp.coords * nv[j];
        }
        HPoint(acc).to_cartesian()
    }

    /// Unit surface normal at `(u, v)` using first partial derivatives in
    /// homogeneous space, then the quotient rule for the w-divide.
    pub fn normal_at(&self, u: f64, v: f64) -> Vector3<f64> {
        let (du, dv) = self.derivatives_uv(u, v);
        du.cross(&dv).normalize()
    }

    /// Returns `(dS/du, dS/dv)` in cartesian space.
    pub fn derivatives_uv(&self, u: f64, v: f64) -> (Vector3<f64>, Vector3<f64>) {
        let (min_u, max_u) = self.domain_u();
        let (min_v, max_v) = self.domain_v();
        let u = u.clamp(min_u, max_u);
        let v = v.clamp(min_v, max_v);

        let p = self.degree_u.get();
        let q = self.degree_v.get();
        let n = self.control_points.nu() - 1;
        let m = self.control_points.nv() - 1;

        let span_u = self.knots_u.find_span(n, self.degree_u, u);
        let span_v = self.knots_v.find_span(m, self.degree_v, v);
        let du_basis = basis_function_derivatives(span_u, u, self.degree_u, &self.knots_u, 1);
        let dv_basis = basis_function_derivatives(span_v, v, self.degree_v, &self.knots_v, 1);

        let mut s = Point4::origin();
        let mut s_u = Vector4::zeros();
        let mut s_v = Vector4::zeros();

        for j in 0..=q {
            let mut row = Point4::origin();
            let mut row_u = Vector4::zeros();
            for i in 0..=p {
                let hp = self.control_points.get(span_u - p + i, span_v - q + j).0;
                row.coords += hp.coords * du_basis[0][i];
                row_u += hp.coords * du_basis[1][i];
            }
            s.coords += row.coords * dv_basis[0][j];
            s_u += row_u * dv_basis[0][j];
            s_v += row.coords * dv_basis[1][j];
        }

        let w = s.w;
        let s_xyz = Vector3::new(s.x, s.y, s.z);
        let ds_u_xyz = Vector3::new(s_u.x, s_u.y, s_u.z);
        let ds_v_xyz = Vector3::new(s_v.x, s_v.y, s_v.z);
        let du = (ds_u_xyz - s_xyz * (s_u.w / w)) / w;
        let dv = (ds_v_xyz - s_xyz * (s_v.w / w)) / w;
        (du, dv)
    }
}
