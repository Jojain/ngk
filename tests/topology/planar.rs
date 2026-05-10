use nalgebra::Vector3;
use ngk::builders::profiles::{add_polygon, add_polyline};
use ngk::geometry::{Curve, LINEAR_TOLERANCE, Line, Point3};
use ngk::topology::gmap::GMap;
use ngk::topology::payload::StandardPayload;
use ngk::topology::planar::{Planar, PlanarityError};
use ngk::topology::profile::Profile;

#[test]
fn planar_new_wraps_planar_profile_and_infers_plane() {
    let mut g = GMap::<StandardPayload>::new();
    let dart = add_polygon(
        &mut g,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
    );

    let planar = Planar::new(Profile::new(&g, dart)).expect("xy profile should be planar");

    assert!(planar.plane().normal().dot(&Vector3::z()).abs() > 1.0 - LINEAR_TOLERANCE);
}

#[test]
fn planar_new_rejects_profile_with_off_plane_vertex() {
    let mut g = GMap::<StandardPayload>::new();
    let dart = add_polygon(
        &mut g,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.5),
        ],
    );

    let err = Planar::new(Profile::new(&g, dart))
        .err()
        .expect("profile is not planar");

    assert!(matches!(
        err,
        PlanarityError::NonPlanarPoint { dart: _, .. }
    ));
}

#[test]
fn planar_new_accepts_collinear_profiles_with_fallback_plane() {
    let mut g = GMap::<StandardPayload>::new();
    let segments = [
        (
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Curve::Line(Line::new(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            )),
        ),
        (
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Curve::Line(Line::new(
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
            )),
        ),
    ];
    let dart = add_polyline(&mut g, &segments).expect("test profile should build");

    let planar = Planar::new(Profile::new(&g, dart)).expect("a line profile is planar");

    assert!(planar.plane().normal().norm() > 1.0 - LINEAR_TOLERANCE);
}
