use nalgebra::Vector3;
use ngk::geometry::surfaces::{Cylinder, Plane, Surface};
use ngk::geometry::utils::Point3;

fn assert_point_near(actual: Point3, expected: Point3) {
    let err = (actual - expected).norm();
    assert!(
        err < 1e-10,
        "expected {expected:?}, got {actual:?}, err={err}"
    );
}

#[test]
fn plane_new_orthonormalizes_frame() {
    let plane = Plane::new(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 1.0),
        Vector3::new(0.0, 0.0, 1.0),
    );

    assert!(plane.frame.x_dir.dot(&plane.frame.z_dir).abs() < 1e-10);
    assert!(plane.x_dir().dot(&plane.normal()).abs() < 1e-10);
    assert!(plane.y_dir().dot(&plane.normal()).abs() < 1e-10);
    assert_point_near(plane.point_at(2.0, 3.0), Point3::new(2.0, 3.0, 0.0));
}

#[test]
fn cylinder_point_at_wraps_around_axis() {
    let cylinder = Cylinder::new(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        2.0,
    );

    assert_point_near(cylinder.origin(), Point3::new(0.0, 0.0, 0.0));
    assert!(cylinder.x_dir().dot(&cylinder.axis()).abs() < 1e-10);
    assert_point_near(cylinder.point_at(0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
    assert_point_near(
        cylinder.point_at(std::f64::consts::FRAC_PI_2, 0.0),
        Point3::new(0.0, 2.0, 0.0),
    );
}

#[test]
fn cylinder_point_at_moves_along_axis() {
    let surface = Surface::Cylinder(Cylinder::new(
        Point3::new(1.0, 2.0, 3.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        0.5,
    ));

    assert_point_near(surface.point_at(0.0, 4.0), Point3::new(1.5, 2.0, 7.0));
}
