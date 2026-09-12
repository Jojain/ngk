//! B-splines.
//!
//! The mapping is a straight copy in principle — both sides are degree, knots,
//! control points and weights — and four things in it corrupt without failing.
//!
//! **Knots are run-length coded on one side only.** STEP states each distinct
//! value once with a multiplicity beside it; NGK's [`KnotVector`] is fully
//! expanded. Expanding from the file's own multiplicities rather than
//! rediscovering them by comparison is what keeps the values bit-identical, so
//! [`KnotVector::multiplicity`]'s exact `==` stays sound.
//!
//! **Weights live somewhere else.** STEP writes Cartesian control points and a
//! parallel list of weights; NGK's `HPoint` is homogeneous, `(x·w, y·w, z·w,
//! w)`. Neither direction may scale a weight by the document's unit — it is a
//! ratio — and both must scale the points.
//!
//! **A control net is transposed.** STEP's `control_points_list` is a list of
//! lists indexed `[u][v]`; NGK's [`ControlNet`] is flat with `u` varying
//! fastest. Only a patch whose two counts differ catches a mix-up, which is why
//! the test for it is not square.
//!
//! **A knot vector need not be clamped.** A periodic surface arrives with its
//! ends unrepeated, describing a surface that runs on past its own domain. NGK
//! has no periodic flag and nothing downstream expects the extra control
//! points, so both are clamped on the way in — exactly, by knot insertion.

use crate::geometry::{
    ControlNet, ControlPolygon, Curve, Degree, KnotVector, NurbsCurve, NurbsSurface, Point3,
    Surface,
};

use super::super::builder::InstanceBuilder;
use super::super::error::{GeometryError, StepError};
use super::super::part21::{EntityId, Instance};
use super::super::schema::bspline::{
    BSplineCurve, BSplineSurface, CurveKnots, CurveSpline, CurveWeights, SurfaceKnots,
    SurfaceSpline, SurfaceWeights,
};
use super::super::schema::resolver::{Origin, Resolver, SchemaError};
use super::placement::{read_point, write_point};

/// Reads a B-spline curve, or declines an instance that is not one.
pub fn read_bspline_curve(
    resolver: &Resolver<'_>,
    origin: Origin,
    instance: &Instance,
) -> Option<Result<Curve, StepError>> {
    let read = BSplineCurve::read(origin, instance)?;
    Some(read.map_err(StepError::from).and_then(|curve| {
        let points = curve
            .spline
            .control_points
            .iter()
            .map(|id| read_point(resolver, origin, *id))
            .collect::<Result<Vec<Point3>, SchemaError>>()?;
        let weights = weights_for(
            curve.weights.as_ref().map(|rational| &rational.weights[..]),
            points.len(),
            origin,
        )?;
        let knots = expand_knots(&curve.knots.multiplicities, &curve.knots.knots, origin)?;

        let curve = NurbsCurve::new(
            degree(curve.spline.degree)?,
            ControlPolygon::from_cartesian(points, &weights).map_err(unreadable)?,
            KnotVector::new(knots).map_err(unreadable)?,
        )
        .map_err(unreadable)?;
        Ok(Curve::Nurbs(curve.clamped().map_err(unreadable)?))
    }))
}

/// Reads a B-spline surface, or declines an instance that is not one.
pub fn read_bspline_surface(
    resolver: &Resolver<'_>,
    origin: Origin,
    instance: &Instance,
) -> Option<Result<Surface, StepError>> {
    let read = BSplineSurface::read(origin, instance)?;
    Some(read.map_err(StepError::from).and_then(|surface| {
        let net = &surface.spline.control_points;
        let nu = net.len();
        let nv = net.first().map_or(0, Vec::len);
        if nu == 0 || nv == 0 || net.iter().any(|row| row.len() != nv) {
            return Err(ragged(origin));
        }

        // The file's rows run along `u`, so the net is filled `v` outermost to
        // land in NGK's order, where `u` varies fastest.
        let mut points = Vec::with_capacity(nu * nv);
        let mut weights = Vec::with_capacity(nu * nv);
        for v in 0..nv {
            for u in 0..nu {
                points.push(read_point(resolver, origin, net[u][v])?);
                weights.push(match &surface.weights {
                    Some(rational) => *rational
                        .weights
                        .get(u)
                        .and_then(|row| row.get(v))
                        .ok_or_else(|| ragged(origin))?,
                    None => 1.0,
                });
            }
        }

        let knots = &surface.knots;
        let surface = NurbsSurface::new(
            degree(surface.spline.degree_u)?,
            degree(surface.spline.degree_v)?,
            ControlNet::from_cartesian(points, &weights, nu, nv).map_err(unreadable)?,
            KnotVector::new(expand_knots(
                &knots.multiplicities_u,
                &knots.knots_u,
                origin,
            )?)
            .map_err(unreadable)?,
            KnotVector::new(expand_knots(
                &knots.multiplicities_v,
                &knots.knots_v,
                origin,
            )?)
            .map_err(unreadable)?,
        )
        .map_err(unreadable)?;
        Ok(Surface::Nurbs(surface.clamped().map_err(unreadable)?))
    }))
}

/// Writes a B-spline curve, in whichever spelling its weights call for.
pub fn write_bspline_curve(builder: &mut InstanceBuilder, curve: &NurbsCurve) -> EntityId {
    let points = curve.control_points();
    let control_points = points
        .iter()
        .map(|point| write_point(builder, point.to_cartesian()))
        .collect();
    let weights = curve.is_rational().then(|| CurveWeights {
        weights: points.iter().map(|point| point.weight()).collect(),
    });
    let (multiplicities, knots) = compress_knots(curve.knots().as_slice());

    builder.add_complex(
        BSplineCurve {
            spline: CurveSpline {
                degree: curve.degree().get(),
                control_points,
                closed: closes(points.iter().map(|point| point.to_cartesian())),
            },
            knots: CurveKnots {
                multiplicities,
                knots,
            },
            weights,
        }
        .records(),
    )
}

/// Writes a B-spline surface, in whichever spelling its weights call for.
pub fn write_bspline_surface(builder: &mut InstanceBuilder, surface: &NurbsSurface) -> EntityId {
    let net = surface.control_points();
    let (nu, nv) = (net.nu(), net.nv());
    // Back into the file's own order: one list per `u`, each running along `v`.
    let control_points = (0..nu)
        .map(|u| {
            (0..nv)
                .map(|v| write_point(builder, net.get(u, v).to_cartesian()))
                .collect()
        })
        .collect();
    let weights = surface.is_rational().then(|| SurfaceWeights {
        weights: (0..nu)
            .map(|u| (0..nv).map(|v| net.get(u, v).weight()).collect())
            .collect(),
    });

    let (multiplicities_u, knots_u) = compress_knots(surface.knots_u().as_slice());
    let (multiplicities_v, knots_v) = compress_knots(surface.knots_v().as_slice());

    builder.add_complex(
        BSplineSurface {
            spline: SurfaceSpline {
                degree_u: surface.degree_u().get(),
                degree_v: surface.degree_v().get(),
                control_points,
                closed_u: closes((0..nu).map(|u| net.get(u, 0).to_cartesian())),
                closed_v: closes((0..nv).map(|v| net.get(0, v).to_cartesian())),
            },
            knots: SurfaceKnots {
                multiplicities_u,
                multiplicities_v,
                knots_u,
                knots_v,
            },
            weights,
        }
        .records(),
    )
}

/// Expands run-length coded knots into the vector NGK stores.
///
/// The file's multiplicities are used as written rather than rediscovered by
/// comparing neighbours, so every repeat of a knot is the same `f64` to the
/// last bit — which is what [`KnotVector::multiplicity`] then relies on.
fn expand_knots(
    multiplicities: &[i64],
    knots: &[f64],
    origin: Origin,
) -> Result<Vec<f64>, StepError> {
    if multiplicities.len() != knots.len() {
        return Err(SchemaError::UnreadableUnit {
            origin,
            detail: format!(
                "{} knot multiplicities for {} knots",
                multiplicities.len(),
                knots.len(),
            ),
        }
        .into());
    }
    let mut expanded = Vec::new();
    for (&multiplicity, &knot) in multiplicities.iter().zip(knots) {
        let count = usize::try_from(multiplicity).map_err(|_| SchemaError::UnreadableUnit {
            origin,
            detail: format!("a knot multiplicity of {multiplicity}"),
        })?;
        expanded.extend(std::iter::repeat_n(knot, count));
    }
    Ok(expanded)
}

/// Run-length codes a knot vector into the distinct values and their counts.
///
/// Neighbours are compared exactly, which is the same comparison the expansion
/// above makes exact by construction: a vector this crate built repeats a knot
/// by copying it, so equal knots are equal to the last bit.
fn compress_knots(knots: &[f64]) -> (Vec<i64>, Vec<f64>) {
    let mut values: Vec<f64> = Vec::new();
    let mut multiplicities: Vec<i64> = Vec::new();
    for &knot in knots {
        match values.last() {
            Some(&last) if last == knot => {
                *multiplicities
                    .last_mut()
                    .expect("a value has a multiplicity beside it") += 1
            }
            _ => {
                values.push(knot);
                multiplicities.push(1);
            }
        }
    }
    (multiplicities, values)
}

/// One weight per control point, defaulting a polynomial curve's to unity.
fn weights_for(
    weights: Option<&[f64]>,
    points: usize,
    origin: Origin,
) -> Result<Vec<f64>, StepError> {
    let Some(weights) = weights else {
        return Ok(vec![1.0; points]);
    };
    if weights.len() != points {
        return Err(SchemaError::UnreadableUnit {
            origin,
            detail: format!("{} weights for {points} control points", weights.len()),
        }
        .into());
    }
    Ok(weights.to_vec())
}

/// Whether a run of control points ends where it started.
///
/// `closed_curve` and its two surface counterparts are hints rather than
/// geometry — the knots and control points say the same thing — so this answers
/// the question the schema asks without anything depending on the answer.
fn closes(points: impl Iterator<Item = Point3>) -> bool {
    let points: Vec<Point3> = points.collect();
    match (points.first(), points.last()) {
        (Some(first), Some(last)) => {
            points.len() > 1 && (last - first).norm() <= crate::geometry::LINEAR_TOLERANCE
        }
        _ => false,
    }
}

fn degree(degree: usize) -> Result<Degree, StepError> {
    Degree::new(degree).map_err(|error| {
        GeometryError::UnreadableNurbs {
            detail: error.to_string(),
        }
        .into()
    })
}

fn unreadable(error: crate::geometry::NurbsError) -> StepError {
    GeometryError::UnreadableNurbs {
        detail: error.to_string(),
    }
    .into()
}

fn ragged(origin: Origin) -> StepError {
    SchemaError::UnreadableUnit {
        origin,
        detail: "a control-point grid whose rows are not all the same length".to_string(),
    }
    .into()
}
