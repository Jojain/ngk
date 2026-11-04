//! Moving a curve that lies on a surface along that surface's normal.
//!
//! The curve keeps its parameterization: the moved curve at `t` is the source
//! curve's point at `t` moved along the surface normal there. That is the
//! curve a face's boundary becomes when the face's surface is offset, which is
//! why the face's pcurve for it stays valid unchanged.

use crate::geometry::dim3::offset::fit_curve;
use crate::geometry::parameter::NativeParam;
use crate::geometry::{
    Circle, Curve, Interval, LINEAR_TOLERANCE, OffsetError, Plane, Point2, Point3, Rigid, Surface,
};
use nalgebra::{UnitVector3, Vector3};

/// Points a curve's normals are sampled at to decide it moves rigidly.
const PROBES: usize = 5;

/// `curve` moved `distance` along `surface`'s normal at each of its points,
/// fitted over `window` where the move is not exact.
///
/// Exact where the normal along the curve is rigid with it: a curve on a
/// plane translates, a line along which the normal is constant translates,
/// and a circle about which the normal turns with the circle (a rim of a
/// cylinder, sphere, cone or torus) moves along its axis and changes radius.
pub(crate) fn offset_on_surface(
    curve: &Curve,
    window: Interval,
    surface: &Surface,
    distance: f64,
) -> Result<Curve, OffsetError> {
    if distance == 0.0 {
        return Ok(curve.clone());
    }
    let normal_at = |point: Point3| -> Result<UnitVector3<f64>, OffsetError> {
        let uv: Point2 = surface.param_at(point)?;
        Ok(surface.normal_at(uv.x, uv.y))
    };
    if let Surface::Plane(plane) = surface {
        return Ok(curve.moved(&Rigid::translation(*plane.normal() * distance)));
    }
    let probe = window.or_extent(1.0);
    let probes = (0..PROBES)
        .map(|index| {
            probe.start.value()
                + (probe.end.value() - probe.start.value()) * index as f64 / (PROBES - 1) as f64
        })
        .collect::<Vec<_>>();
    match curve {
        Curve::Line(_) => {
            let first = normal_at(curve.point_at(NativeParam::new(probes[0])))?;
            let constant = probes.iter().try_fold(true, |all, &t| {
                let normal = normal_at(curve.point_at(NativeParam::new(t)))?;
                Ok::<_, OffsetError>(all && (normal.into_inner() - *first).norm() <= 1e-9)
            })?;
            if constant {
                return Ok(curve.moved(&Rigid::translation(*first * distance)));
            }
        }
        Curve::Circle(circle) => {
            if let Some(moved) = rigid_circle_offset(circle, &probes, &normal_at, distance)? {
                return Ok(Curve::Circle(moved));
            }
        }
        _ => {}
    }
    fit_curve(window, |t| {
        let point = curve.point_at(NativeParam::new(t));
        Ok(point + *normal_at(point)? * distance)
    })
}

/// A circle's move when the surface normal keeps one axial and one radial
/// component all round it, or `None` when it does not.
fn rigid_circle_offset(
    circle: &Circle,
    probes: &[f64],
    normal_at: &impl Fn(Point3) -> Result<UnitVector3<f64>, OffsetError>,
    distance: f64,
) -> Result<Option<Circle>, OffsetError> {
    let plane = circle.plane();
    let centre = plane.origin();
    let axis = plane.normal();
    let mut components: Option<(f64, f64)> = None;
    for &t in probes {
        let point = Curve::Circle(circle.clone()).point_at(NativeParam::new(t));
        let radial: Vector3<f64> = (point - centre).normalize();
        let normal = normal_at(point)?;
        let (axial, outward) = (normal.dot(&axis), normal.dot(&radial));
        let tangential = (*normal - *axis * axial - radial * outward).norm();
        if tangential > 1e-9 {
            return Ok(None);
        }
        match components {
            None => components = Some((axial, outward)),
            Some((a, r)) if (a - axial).abs() > 1e-9 || (r - outward).abs() > 1e-9 => {
                return Ok(None);
            }
            Some(_) => {}
        }
    }
    let Some((axial, outward)) = components else {
        return Ok(None);
    };
    let radius = circle.radius() + outward * distance;
    if radius <= LINEAR_TOLERANCE {
        return Err(OffsetError::Collapsed {
            distance,
            radius: circle.radius(),
        });
    }
    let frame = plane
        .frame
        .moved(&Rigid::translation(*axis * (axial * distance)));
    Ok(Some(Circle::new(Plane::from_frame(frame), radius)))
}
