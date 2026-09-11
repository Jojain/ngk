//! Surface patch tessellation: a [`Surface`] over a `(u_range, v_range)`
//! rectangle → [`IndexedMesh`].
//!
//! Always emits a regular grid with per-vertex normals and consistently-wound
//! triangles: `(nu+1) x (nv+1)` samples, one fewer along any axis whose range
//! covers a whole period, where the closing row of samples would only repeat the
//! opening one. Used directly by [`crate::tessellate::face`] for cylinder
//! shortcuts and indirectly by anything that needs a quick patch preview.

use super::{IndexedMesh, SurfaceOpts};
use crate::geometry::{LINEAR_TOLERANCE, Surface, SurfacePeriodicity};

/// Uniformly sample `surface` over `(u_range, v_range)` into a
/// `nu x nv` quad grid (= 2·nu·nv triangles). Triangles are wound CCW
/// from the surface's natural normal side.
///
/// A range spanning a whole period is meshed closed: the quads that would reach
/// its far edge index back to its near one instead, so a ring comes out
/// watertight rather than cracked along the chart's cut.
pub fn tessellate_surface_patch(
    surface: &Surface,
    u_range: (f64, f64),
    v_range: (f64, f64),
    opts: SurfaceOpts,
) -> IndexedMesh {
    let nu = opts.nu.max(1);
    let nv = opts.nv.max(1);

    // A range covering a whole period closes on itself: its last row of samples
    // *is* its first, at the same surface points. Emitting both would leave the
    // mesh split down whichever parameter the chart happened to cut, so the
    // closing row is dropped and the quads that reach it wrap back to index 0.
    let [u_period, v_period] = periods_of(surface);
    let columns = if spans_period(u_period, u_range) {
        nu
    } else {
        nu + 1
    };
    let rows = if spans_period(v_period, v_range) {
        nv
    } else {
        nv + 1
    };

    let at = |i: usize, j: usize| {
        (
            lerp(u_range.0, u_range.1, i as f64 / nu as f64),
            lerp(v_range.0, v_range.1, j as f64 / nv as f64),
        )
    };

    let mut positions = Vec::with_capacity(columns * rows);
    let mut normals = Vec::with_capacity(columns * rows);
    for j in 0..rows {
        for i in 0..columns {
            let (u, v) = at(i, j);
            positions.push(surface.point_at(u, v));
            normals.push(surface.normal_at(u, v));
        }
    }

    let index = |i: usize, j: usize| ((j % rows) * columns + (i % columns)) as u32;
    let mut indices = Vec::with_capacity(nu * nv * 6);
    for j in 0..nv {
        for i in 0..nu {
            let (u0, v0) = at(i, j);
            let (u1, v1) = at(i + 1, j + 1);
            let i00 = index(i, j);
            let i10 = index(i + 1, j);
            let i01 = index(i, j + 1);
            let i11 = index(i + 1, j + 1);
            let bottom_collapsed =
                surface.is_degenerate_at(u0, v0) && surface.is_degenerate_at(u1, v0);
            let top_collapsed =
                surface.is_degenerate_at(u0, v1) && surface.is_degenerate_at(u1, v1);

            if !bottom_collapsed {
                indices.extend_from_slice(&[i00, i10, i11]);
            }
            if !top_collapsed {
                indices.extend_from_slice(&[i00, i11, i01]);
            }
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

/// The support's periods, in parameter order.
fn periods_of(surface: &Surface) -> [Option<f64>; 2] {
    match surface.periodicity() {
        SurfacePeriodicity::None => [None, None],
        SurfacePeriodicity::UPeriodic(period) => [Some(period), None],
        SurfacePeriodicity::VPeriodic(period) => [None, Some(period)],
        SurfacePeriodicity::UVPeriodic(u, v) => [Some(u), Some(v)],
    }
}

/// Whether `range` covers a whole period of a support that has one.
fn spans_period(period: Option<f64>, range: (f64, f64)) -> bool {
    period.is_some_and(|period| ((range.1 - range.0).abs() - period).abs() <= LINEAR_TOLERANCE)
}
