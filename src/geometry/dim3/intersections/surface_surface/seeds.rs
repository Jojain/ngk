use nalgebra::Vector4;

use super::super::curve_surface::{
    PreparedCurve, PreparedSurface, intersect_prepared_curve_surface,
};
use super::super::{
    CurveSurfaceIntersection, IntersectionCoverage, IntersectionError,
    IntersectionIncompleteReason, IntersectionOptions,
};
use super::tracer::TraceState;
use crate::geometry::{ControlPolygon, Curve, NurbsCurve, NurbsSurface, Surface};

#[derive(Clone, Copy)]
enum Boundary {
    UMin,
    UMax,
    VMin,
    VMax,
}

pub(super) struct SeedSearch {
    pub seeds: Vec<TraceState>,
    pub overlap_boundary_found: bool,
    /// Reasons the shared curve/surface solver could not certify a boundary
    /// search. Surface/surface coverage cannot exceed the coverage of the
    /// boundary searches it is built from.
    pub incomplete_reasons: Vec<IntersectionIncompleteReason>,
}

/// Finds regular branch seeds where either surface boundary meets the other surface.
pub(super) fn boundary_seeds(
    a: &NurbsSurface,
    b: &NurbsSurface,
    options: IntersectionOptions,
) -> Result<SeedSearch, IntersectionError> {
    let mut search = SeedSearch {
        seeds: Vec::new(),
        overlap_boundary_found: false,
        incomplete_reasons: Vec::new(),
    };
    collect_surface_boundaries(a, b, true, options, &mut search)?;
    collect_surface_boundaries(b, a, false, options, &mut search)?;
    dedup_seeds(&mut search.seeds, options);
    Ok(search)
}

fn collect_surface_boundaries(
    boundary_surface: &NurbsSurface,
    other_surface: &NurbsSurface,
    boundary_belongs_to_a: bool,
    options: IntersectionOptions,
    search: &mut SeedSearch,
) -> Result<(), IntersectionError> {
    // The other surface is decomposed once and reused across all four
    // boundaries rather than re-decomposed per boundary curve.
    let other_prepared = PreparedSurface::new(&Surface::Nurbs(other_surface.clone()))?;
    for boundary in [
        Boundary::UMin,
        Boundary::UMax,
        Boundary::VMin,
        Boundary::VMax,
    ] {
        let curve = boundary_curve(boundary_surface, boundary)?;
        let results = intersect_prepared_curve_surface(
            &PreparedCurve::new(&Curve::Nurbs(curve))?,
            &other_prepared,
            options,
        )?;
        if let IntersectionCoverage::Incomplete(reasons) = results.coverage() {
            for reason in reasons {
                if !search.incomplete_reasons.contains(reason) {
                    search.incomplete_reasons.push(*reason);
                }
            }
        }
        for result in results {
            match result {
                CurveSurfaceIntersection::Point {
                    curve_u,
                    surface_u,
                    surface_v,
                    ..
                } => {
                    let boundary_uv = boundary_parameters(boundary_surface, boundary, curve_u);
                    let parameters = if boundary_belongs_to_a {
                        Vector4::new(boundary_uv.0, boundary_uv.1, surface_u, surface_v)
                    } else {
                        Vector4::new(surface_u, surface_v, boundary_uv.0, boundary_uv.1)
                    };
                    search.seeds.push(TraceState::new(
                        if boundary_belongs_to_a {
                            boundary_surface
                        } else {
                            other_surface
                        },
                        if boundary_belongs_to_a {
                            other_surface
                        } else {
                            boundary_surface
                        },
                        parameters,
                    ));
                }
                CurveSurfaceIntersection::Overlap { .. } => {
                    search.overlap_boundary_found = true;
                }
            }
        }
    }
    Ok(())
}

fn boundary_curve(
    surface: &NurbsSurface,
    boundary: Boundary,
) -> Result<NurbsCurve, IntersectionError> {
    let control_net = surface.control_points();
    let (degree, knots, points) = match boundary {
        Boundary::UMin => (
            surface.degree_v(),
            surface.knots_v().clone(),
            (0..control_net.nv())
                .map(|v| control_net.get(0, v))
                .collect(),
        ),
        Boundary::UMax => (
            surface.degree_v(),
            surface.knots_v().clone(),
            (0..control_net.nv())
                .map(|v| control_net.get(control_net.nu() - 1, v))
                .collect(),
        ),
        Boundary::VMin => (
            surface.degree_u(),
            surface.knots_u().clone(),
            (0..control_net.nu())
                .map(|u| control_net.get(u, 0))
                .collect(),
        ),
        Boundary::VMax => (
            surface.degree_u(),
            surface.knots_u().clone(),
            (0..control_net.nu())
                .map(|u| control_net.get(u, control_net.nv() - 1))
                .collect(),
        ),
    };
    Ok(NurbsCurve::new(
        degree,
        ControlPolygon::new(points)?,
        knots,
    )?)
}

fn boundary_parameters(
    surface: &NurbsSurface,
    boundary: Boundary,
    curve_parameter: f64,
) -> (f64, f64) {
    match boundary {
        Boundary::UMin => (surface.domain_u().start, curve_parameter),
        Boundary::UMax => (surface.domain_u().end, curve_parameter),
        Boundary::VMin => (curve_parameter, surface.domain_v().start),
        Boundary::VMax => (curve_parameter, surface.domain_v().end),
    }
}

fn dedup_seeds(seeds: &mut Vec<TraceState>, options: IntersectionOptions) {
    let mut unique = Vec::new();
    for seed in seeds.drain(..) {
        if unique.iter().any(|existing: &TraceState| {
            (existing.parameters - seed.parameters).norm() <= options.parameter_tolerance * 10.0
                || (existing.point - seed.point).norm() <= options.linear_tolerance
        }) {
            continue;
        }
        unique.push(seed);
    }
    *seeds = unique;
}
