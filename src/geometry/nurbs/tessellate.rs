use super::curve::NurbsCurve;
use super::surface::NurbsSurface;
use crate::geometry::utils::Point3;

/// A triangle mesh suitable for direct rendering. Indices reference `positions`
/// (and, index-aligned, `normals`).
#[derive(Debug, Clone)]
pub struct IndexedMesh {
    pub positions: Vec<Point3>,
    pub normals: Vec<nalgebra::Vector3<f64>>,
    pub indices: Vec<u32>,
}

/// Uniform sample of a curve into `n + 1` points across its domain.
pub fn sample_curve_uniform(curve: &NurbsCurve, n: usize) -> Vec<Point3> {
    let (a, b) = curve.domain();
    if n == 0 {
        return vec![curve.point_at(a)];
    }
    (0..=n)
        .map(|i| {
            let t = i as f64 / n as f64;
            curve.point_at(a + (b - a) * t)
        })
        .collect()
}

/// Adaptive subdivision: stops refining when the chord-midpoint deviation is
/// below `tolerance`. `max_depth` caps recursion per segment.
pub fn tessellate_curve_adaptive(
    curve: &NurbsCurve,
    tolerance: f64,
    max_depth: usize,
) -> Vec<Point3> {
    let (a, b) = curve.domain();
    let mut out = vec![curve.point_at(a)];
    subdivide(curve, a, b, tolerance, max_depth, &mut out);
    out
}

fn subdivide(
    curve: &NurbsCurve,
    a: f64,
    b: f64,
    tolerance: f64,
    depth: usize,
    out: &mut Vec<Point3>,
) {
    let m = 0.5 * (a + b);
    let pa = *out.last().unwrap();
    let pb = curve.point_at(b);
    let pm = curve.point_at(m);
    let chord_mid = Point3::from((pa.coords + pb.coords) * 0.5);
    let deviation = (pm - chord_mid).norm();
    if depth == 0 || deviation <= tolerance {
        out.push(pb);
    } else {
        subdivide(curve, a, m, tolerance, depth - 1, out);
        subdivide(curve, m, b, tolerance, depth - 1, out);
    }
}

/// Regular `nu × nv` grid tessellation of a surface. Outputs an indexed
/// triangle mesh with per-vertex normals.
pub fn tessellate_surface_grid(surface: &NurbsSurface, nu: usize, nv: usize) -> IndexedMesh {
    let (u_min, u_max) = surface.domain_u();
    let (v_min, v_max) = surface.domain_v();
    let nu = nu.max(1);
    let nv = nv.max(1);

    let mut positions = Vec::with_capacity((nu + 1) * (nv + 1));
    let mut normals = Vec::with_capacity((nu + 1) * (nv + 1));
    for j in 0..=nv {
        let tv = j as f64 / nv as f64;
        let v = v_min + (v_max - v_min) * tv;
        for i in 0..=nu {
            let tu = i as f64 / nu as f64;
            let u = u_min + (u_max - u_min) * tu;
            positions.push(surface.point_at(u, v));
            normals.push(surface.normal_at(u, v));
        }
    }

    let stride = nu + 1;
    let mut indices = Vec::with_capacity(nu * nv * 6);
    for j in 0..nv {
        for i in 0..nu {
            let i00 = (j * stride + i) as u32;
            let i10 = i00 + 1;
            let i01 = i00 + stride as u32;
            let i11 = i01 + 1;
            indices.extend_from_slice(&[i00, i10, i11, i00, i11, i01]);
        }
    }

    IndexedMesh {
        positions,
        normals,
        indices,
    }
}
