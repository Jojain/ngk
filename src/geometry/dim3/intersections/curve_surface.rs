use nalgebra::{Matrix2, Matrix3, Vector2};

use super::error::IntersectionError;
use super::options::IntersectionOptions;
use super::{CurveSurfaceIntersection, CurveSurfaceIntersections};
use crate::geometry::{
    Curve, Interval, NurbsCurve, NurbsSurface, Point3, PointCoincidence, Surface,
};

#[derive(Debug, Clone, Copy)]
struct SurfaceSample {
    point: Point3,
    u: f64,
    v: f64,
}

#[derive(Debug, Clone, Copy)]
struct SurfaceTriangle {
    a: SurfaceSample,
    b: SurfaceSample,
    c: SurfaceSample,
}

#[derive(Debug, Clone, Copy)]
struct CurveSample {
    point: Point3,
    u: f64,
}

pub fn intersect_curve_surface(
    curve: &Curve,
    surface: &Surface,
) -> Result<CurveSurfaceIntersections, IntersectionError> {
    intersect_curve_surface_with_options(curve, surface, IntersectionOptions::default())
}

pub fn intersect_curve_surface_with_options(
    curve: &Curve,
    surface: &Surface,
    options: IntersectionOptions,
) -> Result<CurveSurfaceIntersections, IntersectionError> {
    if !options.validate() {
        return Err(IntersectionError::InvalidOptions);
    }

    let curve = curve.to_nurbs()?;
    let surface = surface.to_nurbs()?;

    if curve_lies_on_surface(&curve, &surface, options) {
        return Ok(vec![CurveSurfaceIntersection::Overlap {
            curve_interval: curve.domain(),
        }]);
    }

    let curve_samples = sample_curve(&curve, options.curve_sample_count);
    let surface_triangles = surface_triangles(&surface, options);
    let mut intersections = Vec::new();

    for segment in curve_samples.windows(2) {
        let start = segment[0];
        let end = segment[1];
        for triangle in &surface_triangles {
            let Some(seed) = segment_triangle_seed(start, end, *triangle, options) else {
                continue;
            };
            if let Some(point) = refine_curve_surface_point(&curve, &surface, seed, options) {
                intersections.push(point);
            }
        }
    }

    Ok(dedup_intersections(intersections, options))
}

fn sample_curve(curve: &NurbsCurve, count: usize) -> Vec<CurveSample> {
    let domain = curve.domain();
    (0..=count)
        .map(|i| {
            let t = i as f64 / count as f64;
            let u = domain.start + (domain.end - domain.start) * t;
            CurveSample {
                point: curve.point_at(u),
                u,
            }
        })
        .collect()
}

fn surface_triangles(surface: &NurbsSurface, options: IntersectionOptions) -> Vec<SurfaceTriangle> {
    let domain_u = surface.domain_u();
    let domain_v = surface.domain_v();
    let mut grid = Vec::with_capacity(
        (options.surface_u_sample_count + 1) * (options.surface_v_sample_count + 1),
    );

    for j in 0..=options.surface_v_sample_count {
        let tv = j as f64 / options.surface_v_sample_count as f64;
        let v = domain_v.start + (domain_v.end - domain_v.start) * tv;
        for i in 0..=options.surface_u_sample_count {
            let tu = i as f64 / options.surface_u_sample_count as f64;
            let u = domain_u.start + (domain_u.end - domain_u.start) * tu;
            grid.push(SurfaceSample {
                point: surface.point_at(u, v),
                u,
                v,
            });
        }
    }

    let stride = options.surface_u_sample_count + 1;
    let mut triangles =
        Vec::with_capacity(options.surface_u_sample_count * options.surface_v_sample_count * 2);
    for j in 0..options.surface_v_sample_count {
        for i in 0..options.surface_u_sample_count {
            let a = grid[j * stride + i];
            let b = grid[j * stride + i + 1];
            let c = grid[(j + 1) * stride + i];
            let d = grid[(j + 1) * stride + i + 1];
            triangles.push(SurfaceTriangle { a, b, c });
            triangles.push(SurfaceTriangle { a: b, b: d, c });
        }
    }
    triangles
}

fn segment_triangle_seed(
    start: CurveSample,
    end: CurveSample,
    triangle: SurfaceTriangle,
    options: IntersectionOptions,
) -> Option<(f64, f64, f64)> {
    let direction = end.point - start.point;
    let edge_1 = triangle.b.point - triangle.a.point;
    let edge_2 = triangle.c.point - triangle.a.point;
    let h = direction.cross(&edge_2);
    let det = edge_1.dot(&h);
    if det.abs() <= options.linear_tolerance {
        return None;
    }

    let inv_det = 1.0 / det;
    let s = start.point - triangle.a.point;
    let beta = inv_det * s.dot(&h);
    if beta < -options.linear_tolerance || beta > 1.0 + options.linear_tolerance {
        return None;
    }

    let q = s.cross(&edge_1);
    let gamma = inv_det * direction.dot(&q);
    if gamma < -options.linear_tolerance || beta + gamma > 1.0 + options.linear_tolerance {
        return None;
    }

    let segment_t = inv_det * edge_2.dot(&q);
    if segment_t < -options.linear_tolerance || segment_t > 1.0 + options.linear_tolerance {
        return None;
    }

    let alpha = 1.0 - beta - gamma;
    let curve_u = start.u + (end.u - start.u) * segment_t.clamp(0.0, 1.0);
    let surface_u = alpha * triangle.a.u + beta * triangle.b.u + gamma * triangle.c.u;
    let surface_v = alpha * triangle.a.v + beta * triangle.b.v + gamma * triangle.c.v;
    Some((curve_u, surface_u, surface_v))
}

fn refine_curve_surface_point(
    curve: &NurbsCurve,
    surface: &NurbsSurface,
    seed: (f64, f64, f64),
    options: IntersectionOptions,
) -> Option<CurveSurfaceIntersection> {
    let curve_domain = curve.domain();
    let surface_domain_u = surface.domain_u();
    let surface_domain_v = surface.domain_v();
    let (mut curve_u, mut surface_u, mut surface_v) = seed;
    curve_u = clamp_interval(curve_u, curve_domain);
    surface_u = clamp_interval(surface_u, surface_domain_u);
    surface_v = clamp_interval(surface_v, surface_domain_v);

    for _ in 0..options.newton_max_iterations {
        let curve_point = curve.point_at(curve_u);
        let surface_point = surface.point_at(surface_u, surface_v);
        let residual = curve_point - surface_point;
        let curve_derivative = curve.derivative_at(curve_u, 1);
        let (surface_du, surface_dv) = surface.derivatives_uv(surface_u, surface_v);
        let jacobian = Matrix3::from_columns(&[curve_derivative, -surface_du, -surface_dv]);
        let Some(delta) = jacobian.lu().solve(&(-residual)) else {
            break;
        };

        curve_u = clamp_interval(curve_u + delta.x, curve_domain);
        surface_u = clamp_interval(surface_u + delta.y, surface_domain_u);
        surface_v = clamp_interval(surface_v + delta.z, surface_domain_v);
        if delta.norm() <= options.parameter_tolerance {
            break;
        }
    }

    let curve_point = curve.point_at(curve_u);
    let surface_point = surface.point_at(surface_u, surface_v);
    (curve_point - surface_point)
        .norm_squared()
        .le(&options.linear_tolerance_squared())
        .then(|| CurveSurfaceIntersection::Point {
            point: Point3::from((curve_point.coords + surface_point.coords) * 0.5),
            curve_u,
            surface_u,
            surface_v,
        })
}

fn curve_lies_on_surface(
    curve: &NurbsCurve,
    surface: &NurbsSurface,
    options: IntersectionOptions,
) -> bool {
    let curve_domain = curve.domain();
    let sample_count = options.curve_sample_count.min(16);
    (0..=sample_count).all(|i| {
        let t = i as f64 / sample_count as f64;
        let u = curve_domain.start + (curve_domain.end - curve_domain.start) * t;
        let point = curve.point_at(u);
        distance_to_surface(surface, point, options) <= options.linear_tolerance * 10.0
    })
}

pub(super) fn distance_to_surface(
    surface: &NurbsSurface,
    point: Point3,
    options: IntersectionOptions,
) -> f64 {
    let (_, _, distance) = closest_surface_parameter(surface, point, options);
    distance
}

pub(super) fn closest_surface_parameter(
    surface: &NurbsSurface,
    point: Point3,
    options: IntersectionOptions,
) -> (f64, f64, f64) {
    let domain_u = surface.domain_u();
    let domain_v = surface.domain_v();
    let scan_u = options.surface_u_sample_count.clamp(2, 12);
    let scan_v = options.surface_v_sample_count.clamp(2, 12);
    let mut best = (domain_u.start, domain_v.start, f64::INFINITY);

    for j in 0..=scan_v {
        let tv = j as f64 / scan_v as f64;
        let v = domain_v.start + (domain_v.end - domain_v.start) * tv;
        for i in 0..=scan_u {
            let tu = i as f64 / scan_u as f64;
            let u = domain_u.start + (domain_u.end - domain_u.start) * tu;
            let distance = (point - surface.point_at(u, v)).norm();
            if distance < best.2 {
                best = (u, v, distance);
            }
        }
    }

    let (mut u, mut v, _) = best;
    for _ in 0..options.newton_max_iterations {
        let surface_point = surface.point_at(u, v);
        let residual = point - surface_point;
        let (du, dv) = surface.derivatives_uv(u, v);
        let normal = Vector2::new(residual.dot(&du), residual.dot(&dv));
        let gram = Matrix2::new(du.dot(&du), du.dot(&dv), du.dot(&dv), dv.dot(&dv));
        let Some(delta) = gram.lu().solve(&normal) else {
            break;
        };
        u = clamp_interval(u + delta.x, domain_u);
        v = clamp_interval(v + delta.y, domain_v);
        if delta.norm() <= options.parameter_tolerance {
            break;
        }
    }

    let distance = (point - surface.point_at(u, v)).norm();
    (u, v, distance)
}

fn dedup_intersections(
    intersections: CurveSurfaceIntersections,
    options: IntersectionOptions,
) -> CurveSurfaceIntersections {
    let mut deduped = Vec::new();
    for intersection in intersections {
        if !deduped
            .iter()
            .any(|existing| same_intersection(existing, &intersection, options))
        {
            deduped.push(intersection);
        }
    }
    deduped
}

fn same_intersection(
    a: &CurveSurfaceIntersection,
    b: &CurveSurfaceIntersection,
    options: IntersectionOptions,
) -> bool {
    match (a, b) {
        (
            CurveSurfaceIntersection::Point { point: a, .. },
            CurveSurfaceIntersection::Point { point: b, .. },
        ) => a.coincides(*b, options.linear_tolerance.sqrt() * 10.0),
        (
            CurveSurfaceIntersection::Overlap { curve_interval: a },
            CurveSurfaceIntersection::Overlap { curve_interval: b },
        ) => same_interval(*a, *b, options),
        _ => false,
    }
}

fn same_interval(a: Interval, b: Interval, options: IntersectionOptions) -> bool {
    let a = a.ordered();
    let b = b.ordered();
    (a.start - b.start).abs() <= options.parameter_tolerance
        && (a.end - b.end).abs() <= options.parameter_tolerance
}

fn clamp_interval(value: f64, interval: Interval) -> f64 {
    value.clamp(interval.start, interval.end)
}
