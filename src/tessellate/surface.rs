//! Surface patch tessellation: a [`Surface`] over a `(u_range, v_range)`
//! rectangle → [`IndexedMesh`].
//!
//! Always emits a regular `(nu+1) x (nv+1)` grid with per-vertex normals and
//! consistently-wound triangles. Used directly by [`crate::tessellate::face`]
//! for cylinder shortcuts and indirectly by anything that needs a quick patch
//! preview.

use nalgebra::{Rotation3, Vector3};

use super::{IndexedMesh, SurfaceOpts};
use crate::geometry::{Surface, dim3::surfaces::Cylinder};

/// Uniformly sample `surface` over `(u_range, v_range)` into a
/// `nu x nv` quad grid (= 2·nu·nv triangles). Triangles are wound CCW
/// from the surface's natural normal side.
pub fn tessellate_surface_patch(
    surface: &Surface,
    u_range: (f64, f64),
    v_range: (f64, f64),
    opts: SurfaceOpts,
) -> IndexedMesh {
    let nu = opts.nu.max(1);
    let nv = opts.nv.max(1);

    let mut positions = Vec::with_capacity((nu + 1) * (nv + 1));
    let mut normals = Vec::with_capacity((nu + 1) * (nv + 1));

    for j in 0..=nv {
        let tv = j as f64 / nv as f64;
        let v = lerp(v_range.0, v_range.1, tv);
        for i in 0..=nu {
            let tu = i as f64 / nu as f64;
            let u = lerp(u_range.0, u_range.1, tu);
            positions.push(surface.point_at(u, v));
            normals.push(surface_normal_at(surface, u, v));
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

/// Natural (unflipped) outward unit normal for `surface` at `(u, v)`.
pub fn surface_normal_at(surface: &Surface, u: f64, v: f64) -> Vector3<f64> {
    match surface {
        Surface::Plane(p) => *p.normal(),
        Surface::Cylinder(c) => cylinder_radial_dir(c, u),
        Surface::Nurbs(n) => n.normal_at(u, v),
    }
}

fn cylinder_radial_dir(c: &Cylinder, u: f64) -> Vector3<f64> {
    let rot = Rotation3::from_axis_angle(&c.axis(), u);
    *(rot * c.x_dir())
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}
