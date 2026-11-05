use super::*;

fn ring(cdt: &mut Cdt, points: &[(f64, f64)]) -> Vec<u32> {
    let ids: Vec<u32> = points
        .iter()
        .map(|&(x, y)| match cdt.insert(Point2::new(x, y), false) {
            Ok(v) | Err(Refused::Duplicate(v)) => v,
            Err(e) => panic!("{e:?}"),
        })
        .collect();
    ids
}

fn constrain(cdt: &mut Cdt, ids: &[u32]) {
    for k in 0..ids.len() {
        cdt.insert_segment(ids[k], ids[(k + 1) % ids.len()])
            .unwrap();
    }
}

fn area(cdt: &Cdt) -> f64 {
    cdt.inside()
        .map(|(_, [a, b, c])| {
            let (a, b, c) = (cdt.point(a), cdt.point(b), cdt.point(c));
            0.5 * ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x))
        })
        .sum()
}

fn assert_delaunay(cdt: &Cdt) {
    for (t, v) in cdt.inside() {
        for i in 0..3 {
            if cdt.is_boundary(t, i) {
                continue;
            }
            let u = cdt.triangles[t as usize].n[i];
            if u == NONE {
                continue;
            }
            let j = (0..3)
                .find(|&j| cdt.triangles[u as usize].n[j] == t)
                .unwrap();
            let d = cdt.triangles[u as usize].v[j];
            let inside = incircle(
                coord(cdt.point(v[0])),
                coord(cdt.point(v[1])),
                coord(cdt.point(v[2])),
                coord(cdt.point(d)),
            );
            assert!(
                inside <= 0.0,
                "edge opposite {i} of {t} is not locally Delaunay"
            );
        }
    }
}

fn rand(seed: &mut u64) -> f64 {
    *seed = seed
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    ((*seed >> 11) as f64) / ((1u64 << 53) as f64)
}

#[test]
fn a_square_with_random_points_has_its_area() {
    let mut cdt = Cdt::new(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0));
    let ids = ring(&mut cdt, &[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]);
    constrain(&mut cdt, &ids);
    cdt.classify();
    let mut seed = 7;
    for _ in 0..500 {
        let _ = cdt.insert(Point2::new(rand(&mut seed), rand(&mut seed)), true);
    }
    assert!((area(&cdt) - 1.0).abs() < 1e-12);
    assert_delaunay(&cdt);
}

#[test]
fn a_ring_with_a_hole_and_a_slanted_constraint() {
    let n = 64;
    let circle = |r: f64| -> Vec<(f64, f64)> {
        (0..n)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / n as f64;
                (r * a.cos(), r * a.sin())
            })
            .collect()
    };
    let mut cdt = Cdt::new(Point2::new(-2.0, -2.0), Point2::new(2.0, 2.0));
    let outer = ring(&mut cdt, &circle(2.0));
    let mut inner_points = circle(1.0);
    inner_points.reverse();
    let inner = ring(&mut cdt, &inner_points);
    constrain(&mut cdt, &outer);
    constrain(&mut cdt, &inner);
    cdt.classify();
    let expected = |n: f64, r: f64| 0.5 * n * r * r * (std::f64::consts::TAU / n).sin();
    let want = expected(n as f64, 2.0) - expected(n as f64, 1.0);
    assert!(
        (area(&cdt) - want).abs() < 1e-9,
        "{} against {}",
        area(&cdt),
        want
    );
    let mut seed = 3;
    for _ in 0..300 {
        let r = 1.0 + rand(&mut seed);
        let a = std::f64::consts::TAU * rand(&mut seed);
        let _ = cdt.insert(Point2::new(r * a.cos(), r * a.sin()), true);
    }
    assert!((area(&cdt) - want).abs() < 1e-9);
    assert_delaunay(&cdt);
}

#[test]
fn a_boundary_segment_is_kept_when_points_are_inserted_beside_it() {
    let mut cdt = Cdt::new(Point2::new(0.0, 0.0), Point2::new(4.0, 1.0));
    let ids = ring(&mut cdt, &[(0.0, 0.0), (4.0, 0.0), (4.0, 1.0), (0.0, 1.0)]);
    constrain(&mut cdt, &ids);
    cdt.classify();
    for k in 1..40 {
        // Exactly on the boundary, and a hair inside it.
        let x = k as f64 * 0.1;
        assert_eq!(
            cdt.insert(Point2::new(x, 0.0), true),
            Err(Refused::OnBoundary)
        );
        let _ = cdt.insert(Point2::new(x, 1e-9), true);
    }
    assert!((area(&cdt) - 4.0).abs() < 1e-12);
    assert_delaunay(&cdt);
}

#[test]
fn a_segment_crossing_many_triangles_is_forced_in_whichever_way_it_runs() {
    for (from, to) in [
        ((0.0, 0.5), (10.0, 0.5)),
        ((10.0, 9.5), (0.0, 0.5)),
        ((5.0, 0.0), (5.0, 10.0)),
        ((5.0, 10.0), (5.0, 0.0)),
    ] {
        let mut cdt = Cdt::new(Point2::new(0.0, 0.0), Point2::new(10.0, 10.0));
        let mut seed = 7u64;
        let a = cdt.insert(Point2::new(from.0, from.1), false).unwrap();
        let b = cdt.insert(Point2::new(to.0, to.1), false).unwrap();
        for _ in 0..300 {
            let p = Point2::new(10.0 * rand(&mut seed), 10.0 * rand(&mut seed));
            let _ = cdt.insert(p, false);
        }
        cdt.insert_segment(a, b).unwrap();
        let (t, i) = cdt
            .edge(a, b)
            .or_else(|| cdt.edge(b, a))
            .expect("the segment is an edge");
        assert!(cdt.triangles[t as usize].c[i] > 0);
    }
}
