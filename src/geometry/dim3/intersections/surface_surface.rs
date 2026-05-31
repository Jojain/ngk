use super::curve_surface::{closest_surface_parameter, distance_to_surface};
use super::error::IntersectionError;
use super::options::IntersectionOptions;
use super::{SurfaceSurfaceIntersection, SurfaceSurfaceIntersections};
use crate::geometry::{NurbsSurface, Point3, PointCoincidence, Surface};

#[derive(Debug, Clone, Copy)]
struct SurfaceSample {
    point: Point3,
}

#[derive(Debug, Clone, Copy)]
struct SurfaceTriangle {
    a: SurfaceSample,
    b: SurfaceSample,
    c: SurfaceSample,
}

pub fn intersect_surfaces(
    a: &Surface,
    b: &Surface,
) -> Result<SurfaceSurfaceIntersections, IntersectionError> {
    intersect_surfaces_with_options(a, b, IntersectionOptions::default())
}

pub fn intersect_surfaces_with_options(
    a: &Surface,
    b: &Surface,
    options: IntersectionOptions,
) -> Result<SurfaceSurfaceIntersections, IntersectionError> {
    if !options.validate() {
        return Err(IntersectionError::InvalidOptions);
    }

    let a = a.to_nurbs()?;
    let b = b.to_nurbs()?;

    if surfaces_are_coincident(&a, &b, options) {
        return Ok(vec![SurfaceSurfaceIntersection::Region]);
    }

    let triangles_a = surface_triangles(&a, options);
    let triangles_b = surface_triangles(&b, options);
    let mut points = Vec::new();

    for triangle_a in &triangles_a {
        for triangle_b in &triangles_b {
            triangle_triangle_points(*triangle_a, *triangle_b, options, &mut points);
        }
    }

    let points = dedup_points(points, options);
    if points.is_empty() {
        Ok(Vec::new())
    } else if points.len() == 1 {
        let point = points[0];
        let (surface_a_u, surface_a_v, _) = closest_surface_parameter(&a, point, options);
        let (surface_b_u, surface_b_v, _) = closest_surface_parameter(&b, point, options);
        Ok(vec![SurfaceSurfaceIntersection::Point {
            point,
            surface_a_u,
            surface_a_v,
            surface_b_u,
            surface_b_v,
        }])
    } else {
        Ok(vec![SurfaceSurfaceIntersection::Curve {
            points: sort_curve_points(points),
        }])
    }
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

fn triangle_triangle_points(
    a: SurfaceTriangle,
    b: SurfaceTriangle,
    options: IntersectionOptions,
    points: &mut Vec<Point3>,
) {
    for (start, end) in [(a.a, a.b), (a.b, a.c), (a.c, a.a)] {
        if let Some(point) = segment_triangle_intersection(start.point, end.point, b, options) {
            points.push(point);
        }
    }
    for (start, end) in [(b.a, b.b), (b.b, b.c), (b.c, b.a)] {
        if let Some(point) = segment_triangle_intersection(start.point, end.point, a, options) {
            points.push(point);
        }
    }
}

fn segment_triangle_intersection(
    start: Point3,
    end: Point3,
    triangle: SurfaceTriangle,
    options: IntersectionOptions,
) -> Option<Point3> {
    let direction = end - start;
    let edge_1 = triangle.b.point - triangle.a.point;
    let edge_2 = triangle.c.point - triangle.a.point;
    let h = direction.cross(&edge_2);
    let det = edge_1.dot(&h);
    if det.abs() <= options.linear_tolerance {
        return None;
    }

    let inv_det = 1.0 / det;
    let s = start - triangle.a.point;
    let beta = inv_det * s.dot(&h);
    if beta < -options.linear_tolerance || beta > 1.0 + options.linear_tolerance {
        return None;
    }

    let q = s.cross(&edge_1);
    let gamma = inv_det * direction.dot(&q);
    if gamma < -options.linear_tolerance || beta + gamma > 1.0 + options.linear_tolerance {
        return None;
    }

    let t = inv_det * edge_2.dot(&q);
    if t < -options.linear_tolerance || t > 1.0 + options.linear_tolerance {
        return None;
    }

    Some(start + direction * t.clamp(0.0, 1.0))
}

fn surfaces_are_coincident(
    a: &NurbsSurface,
    b: &NurbsSurface,
    options: IntersectionOptions,
) -> bool {
    surface_samples(a, options)
        .all(|point| distance_to_surface(b, point, options) <= options.linear_tolerance * 10.0)
        && surface_samples(b, options)
            .all(|point| distance_to_surface(a, point, options) <= options.linear_tolerance * 10.0)
}

fn surface_samples(
    surface: &NurbsSurface,
    options: IntersectionOptions,
) -> impl Iterator<Item = Point3> + '_ {
    let domain_u = surface.domain_u();
    let domain_v = surface.domain_v();
    let sample_u = options.surface_u_sample_count.clamp(2, 8);
    let sample_v = options.surface_v_sample_count.clamp(2, 8);

    (0..=sample_v).flat_map(move |j| {
        let tv = j as f64 / sample_v as f64;
        let v = domain_v.start + (domain_v.end - domain_v.start) * tv;
        (0..=sample_u).map(move |i| {
            let tu = i as f64 / sample_u as f64;
            let u = domain_u.start + (domain_u.end - domain_u.start) * tu;
            surface.point_at(u, v)
        })
    })
}

fn dedup_points(points: Vec<Point3>, options: IntersectionOptions) -> Vec<Point3> {
    let tolerance = options.linear_tolerance.sqrt() * 10.0;
    let mut deduped = Vec::new();
    for point in points {
        if !deduped
            .iter()
            .any(|existing| point.coincides(*existing, tolerance))
        {
            deduped.push(point);
        }
    }
    deduped
}

fn sort_curve_points(mut points: Vec<Point3>) -> Vec<Point3> {
    let (min, max) = points.iter().fold(
        (points[0].coords, points[0].coords),
        |(mut min, mut max), point| {
            min.x = min.x.min(point.x);
            min.y = min.y.min(point.y);
            min.z = min.z.min(point.z);
            max.x = max.x.max(point.x);
            max.y = max.y.max(point.y);
            max.z = max.z.max(point.z);
            (min, max)
        },
    );
    let range = max - min;
    if range.x >= range.y && range.x >= range.z {
        points.sort_by(|a, b| a.x.total_cmp(&b.x));
    } else if range.y >= range.z {
        points.sort_by(|a, b| a.y.total_cmp(&b.y));
    } else {
        points.sort_by(|a, b| a.z.total_cmp(&b.z));
    }
    points
}
