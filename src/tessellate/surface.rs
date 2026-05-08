//! Surface patch tessellation: a [`Surface`] over a `(u_range, v_range)`
//! rectangle → [`IndexedMesh`].
//!
//! Always emits a regular `(nu+1) x (nv+1)` grid with per-vertex normals and
//! consistently-wound triangles. Used directly by [`crate::tessellate::face`]
//! for cylinder shortcuts and indirectly by anything that needs a quick patch
//! preview.

use super::{IndexedMesh, SurfaceOpts};
use crate::geometry::Surface;

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

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}
