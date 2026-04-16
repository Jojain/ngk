use super::degree::Degree;
use super::knots::KnotVector;

/// Piegl & Tiller A2.2 — compute the `p+1` non-zero B-spline basis functions
/// at parameter `u`, where `span = knots.find_span(n, degree, u)`.
/// Returns `N[0..=p]`.
pub fn basis_functions(span: usize, u: f64, degree: Degree, knots: &KnotVector) -> Vec<f64> {
    let p = degree.get();
    let mut n = vec![0.0f64; p + 1];
    let mut left = vec![0.0f64; p + 1];
    let mut right = vec![0.0f64; p + 1];
    n[0] = 1.0;
    for j in 1..=p {
        left[j] = u - knots.get(span + 1 - j);
        right[j] = knots.get(span + j) - u;
        let mut saved = 0.0;
        for r in 0..j {
            let denom = right[r + 1] + left[j - r];
            let temp = if denom == 0.0 { 0.0 } else { n[r] / denom };
            n[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        n[j] = saved;
    }
    n
}

/// Piegl & Tiller A2.3 — basis functions and their derivatives up to order `nth`.
/// Returns a `(nth+1) x (p+1)` table: `ders[k][j]` is the k-th derivative of N_{span-p+j,p}(u).
pub fn basis_function_derivatives(
    span: usize,
    u: f64,
    degree: Degree,
    knots: &KnotVector,
    nth: usize,
) -> Vec<Vec<f64>> {
    let p = degree.get();
    let mut ndu = vec![vec![0.0f64; p + 1]; p + 1];
    let mut left = vec![0.0f64; p + 1];
    let mut right = vec![0.0f64; p + 1];
    ndu[0][0] = 1.0;
    for j in 1..=p {
        left[j] = u - knots.get(span + 1 - j);
        right[j] = knots.get(span + j) - u;
        let mut saved = 0.0;
        for r in 0..j {
            ndu[j][r] = right[r + 1] + left[j - r];
            let temp = if ndu[j][r] == 0.0 {
                0.0
            } else {
                ndu[r][j - 1] / ndu[j][r]
            };
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }

    let mut ders = vec![vec![0.0f64; p + 1]; nth + 1];
    for j in 0..=p {
        ders[0][j] = ndu[j][p];
    }

    let mut a = vec![vec![0.0f64; p + 1]; 2];
    for r in 0..=p {
        let mut s1 = 0usize;
        let mut s2 = 1usize;
        a[0][0] = 1.0;
        for k in 1..=nth {
            let mut d = 0.0;
            let rk = r as isize - k as isize;
            let pk = p as isize - k as isize;
            if r >= k {
                a[s2][0] = a[s1][0] / ndu[(pk + 1) as usize][rk as usize];
                d += a[s2][0] * ndu[rk as usize][pk as usize];
            }
            let j1 = if rk >= -1 { 1 } else { (-rk) as usize };
            let j2 = if (r as isize) - 1 <= pk {
                k - 1
            } else {
                p - r
            };
            for j in j1..=j2 {
                let idx = (rk + j as isize) as usize;
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[(pk + 1) as usize][idx];
                d += a[s2][j] * ndu[idx][pk as usize];
            }
            if r <= pk as usize {
                a[s2][k] = -a[s1][k - 1] / ndu[(pk + 1) as usize][r];
                d += a[s2][k] * ndu[r][pk as usize];
            }
            ders[k][r] = d;
            std::mem::swap(&mut s1, &mut s2);
        }
    }

    let mut factor = p as f64;
    for k in 1..=nth {
        for j in 0..=p {
            ders[k][j] *= factor;
        }
        factor *= (p - k) as f64;
    }

    ders
}
