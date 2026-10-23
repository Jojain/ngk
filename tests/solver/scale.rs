use ngk::geometry::dim2::{trimmed::TrimmedCurve2, utils::Point2};
use ngk::solver::{Constraint2d as C, SegmentKey, System2d, Target2d};
use std::time::Instant;

#[test]
fn a_connected_chain_of_one_hundred_segments_preserves_dimensions_when_dragged() {
    connected_chain(100);
}

#[test]
fn a_connected_chain_of_three_hundred_segments_preserves_dimensions_when_dragged() {
    connected_chain(300);
}

fn connected_chain(count: usize) {
    let mut system = System2d::default();
    let mut lines: Vec<SegmentKey> = vec![];
    for i in 0..count {
        let x = i as f64 * 10.;
        let line = system
            .add_segment(TrimmedCurve2::segment(
                Point2::new(x, 0.),
                Point2::new(x + 10., 0.),
            ))
            .unwrap();
        system.add_constraint(C::length(line, 10.)).unwrap();
        system.add_constraint(C::horizontal(line)).unwrap();
        if let Some(previous) = lines.last().copied() {
            system
                .add_constraint(C::coincident(previous.end(), line.start()))
                .unwrap();
        }
        lines.push(line);
    }
    let started = Instant::now();
    let initial = system.solve().unwrap();
    let solve_time = started.elapsed();
    assert_eq!(initial.report().degrees_of_freedom, 2);
    assert!(initial.report().redundancies.is_empty());
    let started = Instant::now();
    let moved = system
        .solve_toward(&[Target2d::point(lines[0].start(), Point2::new(3., 4.))])
        .unwrap();
    let move_time = started.elapsed();
    for line in lines {
        let curve = moved.segment(line).unwrap();
        assert!(((curve.end() - curve.start()).norm() - 10.).abs() < 1e-7);
        assert!((curve.start().y - 4.).abs() < 1e-5);
    }
    eprintln!("connected {count}-segment chain: solve={solve_time:?}, drag={move_time:?}");
}
