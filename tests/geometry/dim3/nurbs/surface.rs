use nalgebra::Vector3;
use ngk::geometry::{Cylinder, Point3, Surface};

#[test]
fn cylinder_decomposes_into_exact_rational_bezier_spans() {
    let surface = Surface::Cylinder(Cylinder::new(
        Point3::origin(),
        Vector3::x(),
        Vector3::z(),
        1.0,
    ))
    .to_nurbs()
    .unwrap();

    let spans = surface.bezier_spans().unwrap();

    assert_eq!(spans.len(), 4);
    for span in spans {
        let domain_u = span.domain_u();
        let domain_v = span.domain_v();
        for local_u in [0.0, 0.25, 0.5, 0.75, 1.0] {
            for local_v in [0.0, 0.5, 1.0] {
                let u = domain_u.start + domain_u.length() * local_u;
                let v = domain_v.start + domain_v.length() * local_v;
                let expected = surface.point_at(u, v);
                let actual = span.point_at(u, v);
                assert!((actual - expected).norm() <= 1.0e-9);
                assert!(span.bbox().contains_point(expected, 1.0e-9));
            }
        }
    }
}
