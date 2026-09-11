//! What a support curve or surface owes the kernel.
//!
//! Every analytic type in [`crate::geometry`] answers the same questions, and
//! the enums ([`Curve`](crate::geometry::Curve),
//! [`Curve2`](crate::geometry::Curve2), [`Surface`](crate::geometry::Surface))
//! forward to whichever variant they hold. Stating that set as a trait turns
//! "what does a new analytic type have to implement?" into a reviewable
//! checklist instead of something you learn by reading a dozen `match` bodies.
//!
//! The enums stay concrete rather than becoming trait objects: `GMap`
//! serialization, healing's value comparisons and cheap cloning all depend on
//! the derived `Serialize`/`Deserialize`/`Clone`/`PartialEq`.
//!
//! # Parameterization is not preserved by NURBS conversion
//!
//! `to_nurbs` reproduces a support **as a point set**. It does not generally
//! reproduce its parameterization: a circle is not a rational function of its
//! angle, so the rational quadratic's parameter is a projective — not linear —
//! function of the angle, agreeing only at knots and span midpoints. Code that
//! carries a parameter across the conversion must reparameterize; code that
//! only needs points is safe. See `plan/analytical_geometry.md`.

use crate::geometry::axis::Axis3;
use crate::geometry::dim2::nurbs::NurbsCurve2;
use crate::geometry::dim2::utils::Point2;
use crate::geometry::dim3::bbox::BBox;
use crate::geometry::dim3::curves::Periodicity;
use crate::geometry::dim3::nurbs::{NurbsCurve, NurbsSurface};
use crate::geometry::dim3::surfaces::SurfacePeriodicity;
use crate::geometry::dim3::utils::Point3;
use crate::geometry::interval::Interval;
use crate::geometry::nurbs::error::NurbsError;
use crate::geometry::reparam::ParamMap;
use nalgebra::{UnitVector3, Vector2, Vector3};

/// The behaviour every 3D support curve provides.
///
/// Implemented by each concrete curve type and forwarded by
/// [`Curve`](crate::geometry::Curve).
pub trait CurveGeometry: Sized {
    /// The parameter range over which the curve is defined.
    ///
    /// Unbounded supports return [`Interval::unbounded`]; callers needing a
    /// finite window clamp it with [`Interval::or_extent`].
    fn domain(&self) -> Interval;

    /// Whether the parameter wraps, and with what period.
    fn periodicity(&self) -> Periodicity;

    /// The point at parameter `t`.
    fn point_at(&self, t: f64) -> Point3;

    /// The `order`-th derivative at parameter `t`.
    fn derivative_at(&self, t: f64, order: usize) -> Vector3<f64>;

    /// The parameter of the curve point nearest `point`.
    fn param_at(&self, point: Point3) -> f64;

    /// The point on the curve nearest `point`.
    fn project(&self, point: Point3) -> Point3;

    /// Arc length between two parameters, in distance units.
    fn length(&self, t0: f64, t1: f64) -> f64;

    /// An exact NURBS representation of the curve as a point set.
    ///
    /// The parameterization is **not** generally preserved — see the module
    /// documentation.
    fn to_nurbs(&self) -> Result<NurbsCurve, NurbsError>;

    /// A conservative bounding box of the curve restricted to `interval`.
    ///
    /// `None` means the representation cannot prove a finite bound, for
    /// example a rational NURBS with a non-positive control weight.
    fn bbox_over(&self, interval: Interval) -> Option<BBox>;

    /// The curve rotated by `angle` radians around `axis`.
    ///
    /// The parameterization is preserved, so a parameter interval computed on
    /// the source curve stays valid on the result.
    fn rotated(&self, axis: Axis3, angle: f64) -> Result<Self, NurbsError>;

    /// The curve translated along `direction`, preserving parameterization.
    fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError>;
}

/// The behaviour every 2D support curve provides.
///
/// This mirrors [`CurveGeometry`] in a surface's parameter space: a 2D support
/// is unbounded in exactly the same way a 3D one is, and the portion of it that
/// is meant is stated by [`TrimmedCurve2`](crate::geometry::TrimmedCurve2)
/// rather than baked into the support. Trimming and splitting therefore live on
/// that type, not here.
///
/// [`NurbsCurve2`] deliberately does not implement this trait: its own methods
/// already speak its native knot domain, so the
/// [`Curve2`](crate::geometry::Curve2) enum forwards to them directly in its
/// `Nurbs` arm.
pub trait Curve2Geometry: Sized {
    /// The parameter range over which the support is defined.
    ///
    /// Unbounded supports return [`Interval::unbounded`]; callers needing a
    /// finite window clamp it with [`Interval::or_extent`].
    fn domain(&self) -> Interval;

    /// Whether the parameter wraps, and with what period.
    fn periodicity(&self) -> Periodicity;

    /// The point at native parameter `t`.
    fn point_at(&self, t: f64) -> Point2;

    /// The `order`-th derivative at native parameter `t`.
    fn derivative_at(&self, t: f64, order: usize) -> Vector2<f64>;

    /// The native parameter of the support point nearest `point`.
    fn param_at(&self, point: Point2) -> f64;

    /// The point on the support nearest `point`.
    fn project(&self, point: Point2) -> Point2;

    /// Arc length between two native parameters, in parameter-space units.
    fn length(&self, t0: f64, t1: f64) -> f64;

    /// The same support traversed in the opposite direction.
    ///
    /// The parameterization is **not** preserved.
    fn reversed(&self) -> Self;

    /// The support translated by `offset`, preserving parameterization.
    fn translated(&self, offset: Vector2<f64>) -> Result<Self, NurbsError>;

    /// An exact NURBS representation of the support as a point set.
    ///
    /// The parameterization is **not** generally preserved — see the module
    /// documentation.
    fn to_nurbs(&self) -> Result<NurbsCurve2, NurbsError>;
}

/// The behaviour every support surface provides.
///
/// Implemented by each concrete surface type and forwarded by
/// [`Surface`](crate::geometry::Surface).
pub trait SurfaceGeometry: Sized {
    /// The `(u, v)` parameter ranges over which the surface is defined.
    ///
    /// Unbounded directions return [`Interval::unbounded`].
    fn domain(&self) -> (Interval, Interval);

    /// Which parameter directions wrap, and with what period.
    fn periodicity(&self) -> SurfacePeriodicity;

    /// The point at parameters `(u, v)`.
    fn point_at(&self, u: f64, v: f64) -> Point3;

    /// The outward unit normal at `(u, v)`.
    ///
    /// At a degenerate point — a sphere's pole, a cone's apex — the normal is
    /// the limit taken along the meridian through `u`.
    fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64>;

    /// Whether the surface's parameterization collapses at `(u, v)`.
    fn is_degenerate_at(&self, u: f64, v: f64) -> bool;

    /// The parameters of the surface point nearest `point`.
    fn closest_parameter(&self, point: Point3) -> Result<Point2, NurbsError>;

    /// An exact NURBS representation of the surface as a point set.
    ///
    /// The parameterization is **not** generally preserved — see the module
    /// documentation.
    fn to_nurbs(&self) -> Result<NurbsSurface, NurbsError>;

    /// An exact NURBS representation realized over the requested parameter box.
    ///
    /// An unbounded analytic surface must span the box: returning its default
    /// patch instead silently drops everything outside it. Surfaces that carry
    /// their own finite parameterization may ignore the box.
    fn to_nurbs_over(&self, u: Interval, v: Interval) -> Result<NurbsSurface, NurbsError>;

    /// Maps analytic parameters in the requested box to the NURBS patch.
    fn param_map_over(&self, u: Interval, v: Interval) -> ParamMap;

    /// A conservative bounding box of the surface restricted to a finite box.
    fn bbox_over(&self, u: Interval, v: Interval) -> Option<BBox>;

    /// The surface rotated by `angle` radians around `axis`.
    ///
    /// The parameterization is preserved, so parameter curves expressed in
    /// this surface's space stay valid on the result.
    fn rotated(&self, axis: Axis3, angle: f64) -> Result<Self, NurbsError>;

    /// The surface translated along `direction`, preserving parameterization.
    fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError>;
}
