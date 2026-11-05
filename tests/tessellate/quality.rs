//! Triangles are well shaped in 3D, whatever the parameterisation: the metric
//! of the surface, not the raw parameters, decides what a good triangle is.

use ngk::modeling::{revolve, solids};
use ngk::tessellate::{Tessellate, Tessellation};

use super::watertight::{Built, axis_z, deg, opts, p, revolved_rect};
use ngk::modeling::faces;

/// The smallest angle of each triangle of `mesh`, in degrees, with its area.
fn angles(mesh: &Tessellation) -> Vec<(f64, f64)> {
    mesh.mesh
        .indices
        .chunks_exact(3)
        .map(|c| {
            let q = [c[0], c[1], c[2]].map(|i| mesh.mesh.positions[i as usize]);
            let mut least = f64::INFINITY;
            for k in 0..3 {
                let (u, v) = (q[(k + 1) % 3] - q[k], q[(k + 2) % 3] - q[k]);
                least = least.min((u.dot(&v) / (u.norm() * v.norm())).clamp(-1.0, 1.0).acos());
            }
            let area = 0.5 * (q[1] - q[0]).cross(&(q[2] - q[0])).norm();
            (least.to_degrees(), area)
        })
        .collect()
}

/// The share of the surface's area, in triangles with an angle under `bound`.
fn area_below(solid: &Built, segments: usize, bound: f64) -> (f64, usize) {
    let mesh = solid
        .solid()
        .tessellate_with(opts(segments))
        .expect("tessellates");
    let all = angles(&mesh);
    let total: f64 = all.iter().map(|a| a.1).sum();
    let bad: f64 = all.iter().filter(|a| a.0 < bound).map(|a| a.1).sum();
    (bad / total, all.len())
}

#[test]
fn round_parts_have_no_slivers() {
    let cases: Vec<(&str, Built)> = vec![
        ("cylinder", solids::cylinder(2.0, 5.0).unwrap()),
        ("sphere", solids::sphere(3.0).unwrap()),
        ("torus", solids::torus(5.0, 1.0).unwrap()),
        ("washer", revolved_rect(6.0, 10.0, 360.0)),
        ("sector", revolved_rect(6.0, 10.0, 90.0)),
    ];
    for (name, solid) in cases {
        let (share, triangles) = area_below(&solid, 32, 10.0);
        assert!(
            share < 0.05,
            "{name}: {:.1}% of the area is in triangles under 10 degrees ({triangles} triangles)",
            100.0 * share
        );
    }
}

#[test]
fn a_needle_is_not_meshed_in_its_parameters() {
    // Radius 0.01, length 1000: a unit of u is a hundred thousandth of a unit
    // of v in 3D. Triangles are shaped by what they are in space.
    let needle = solids::cylinder(0.01, 1000.0).unwrap();
    let mesh = needle
        .solid()
        .tessellate_with(opts(32))
        .expect("tessellates");
    let triangles = mesh.mesh.indices.len() / 3;
    assert!(triangles < 200_000, "{triangles} triangles for a needle");
    let all = angles(&mesh);
    let total: f64 = all.iter().map(|a| a.1).sum();
    let obtuse_free = all.iter().filter(|a| a.0 > 0.0).map(|a| a.1).sum::<f64>();
    assert!(
        (obtuse_free - total).abs() <= 1e-9 * total,
        "no triangle collapsed"
    );
}

#[test]
fn a_cone_apex_is_a_single_point() {
    let section = faces::polygon(&[p(0.0, 0.0, 6.0), p(5.0, 0.0, 0.0), p(0.0, 0.0, 0.0)]).unwrap();
    let cone = revolve::revolve_face(section, axis_z(), deg(360.0)).unwrap();
    let mesh = cone.solid().tessellate_with(opts(32)).unwrap();
    let apex = mesh
        .mesh
        .positions
        .iter()
        .filter(|q| (q.x.abs() + q.y.abs()) < 1e-12 && (q.z - 6.0).abs() < 1e-12)
        .count();
    assert!(apex >= 1);
}
