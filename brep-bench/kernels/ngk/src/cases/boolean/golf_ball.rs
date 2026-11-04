//! A ball of radius 10 about the origin with 62 dimples, each cut by a ball of radius 1.2 whose
//! centre is 10.6 out along a vertex, edge midpoint or face centre of the icosahedron with
//! vertices (0, ±1, ±φ), (±1, ±φ, 0), (±φ, 0, ±1). Each small ball's poles are on the vertical
//! through its centre; each is its own cut.
use crate::bench::{ball, cut, p};
use crate::{Refusal, Solid};
use nalgebra::Vector3;

pub fn build() -> Result<Solid, Refusal> {
    let phi = (1.0 + 5f64.sqrt()) / 2.0;
    let mut vertices = Vec::new();
    for s1 in [-1.0, 1.0] {
        for s2 in [-1.0, 1.0] {
            vertices.push(Vector3::new(0.0, s1, s2 * phi));
            vertices.push(Vector3::new(s1, s2 * phi, 0.0));
            vertices.push(Vector3::new(s1 * phi, 0.0, s2));
        }
    }
    // Edges join vertices 2 apart; faces are triangles of mutually adjacent vertices.
    let adjacent = |a: &Vector3<f64>, b: &Vector3<f64>| ((a - b).norm() - 2.0).abs() < 1e-9;
    let mut directions: Vec<Vector3<f64>> = vertices.clone();
    let n = vertices.len();
    for i in 0..n {
        for j in i + 1..n {
            if adjacent(&vertices[i], &vertices[j]) {
                directions.push(vertices[i] + vertices[j]);
                for k in j + 1..n {
                    if adjacent(&vertices[i], &vertices[k]) && adjacent(&vertices[j], &vertices[k]) {
                        directions.push(vertices[i] + vertices[j] + vertices[k]);
                    }
                }
            }
        }
    }
    assert_eq!(directions.len(), 62);
    let mut golf = ball(p(0.0, 0.0, 0.0), 10.0)?;
    for d in directions {
        let c = d.normalize() * 10.6;
        golf = cut(golf, &ball(p(c.x, c.y, c.z), 1.2)?)?;
    }
    Ok(golf)
}
