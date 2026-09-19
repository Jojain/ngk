use nalgebra::Vector3;
use ngk::builders::faces::{add_annulus, add_circle, add_face, add_polygon};
use ngk::builders::loft::{
    CappedSection, ClosedSection, LoftError, LoftOptions, OpenSection, add_loft,
};
use ngk::builders::profiles::add_polyline;
use ngk::geometry::{Degree, LINEAR_TOLERANCE, Plane, Point3};
use ngk::model::Model;
use ngk::topology::StandardPayload;
use ngk::topology::closed::{Closeable, Closed};
use ngk::topology::edge::Edge;
use ngk::topology::face::Face;
use ngk::topology::shape_keys::{FaceKey, ProfileKey};
use ngk::topology::validation::{validate_all_solid_manifolds, validate_all_solid_orientations};

/// A square of side 2 centred on the z axis at height `z`.
fn square(model: &mut Model<StandardPayload>, z: f64) -> ProfileKey {
    add_polygon(
        model,
        &[
            Point3::new(-1.0, -1.0, z),
            Point3::new(1.0, -1.0, z),
            Point3::new(1.0, 1.0, z),
            Point3::new(-1.0, 1.0, z),
        ],
    )
}

/// A circular face of `radius` at height `z`.
fn disk(model: &mut Model<StandardPayload>, radius: f64, z: f64) -> FaceKey {
    let plane = Plane::new(
        Point3::new(0.0, 0.0, z),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
    );
    add_circle(model, plane, radius).unwrap()
}

/// The closed section a registered closed profile makes.
fn closed(model: &Model<StandardPayload>, key: ProfileKey) -> ClosedSection {
    ClosedSection::new(
        &Closed::new(model.profile_unchecked(key)).expect("profile should be closed"),
    )
}

/// The open section a registered open profile makes.
fn open(model: &Model<StandardPayload>, key: ProfileKey) -> OpenSection {
    OpenSection::new(&model.profile_unchecked(key)).expect("profile should be open")
}

/// The capped section a registered face makes.
fn capped(model: &Model<StandardPayload>, key: FaceKey) -> CappedSection {
    CappedSection::new(&Face::new(model, key)).expect("face should be cappable")
}

/// The outer boundary of a circular face, as a closed profile.
fn circle(model: &mut Model<StandardPayload>, radius: f64, z: f64) -> ProfileKey {
    let plane = Plane::new(
        Point3::new(0.0, 0.0, z),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
    );
    let face = add_circle(model, plane, radius).unwrap();
    Face::new(model, face)
        .outer_loop()
        .unwrap()
        .profile_key()
        .unwrap()
}

/// Asserts every face of the loft reaches both end sections.
///
/// Each column face spans the whole run, so sampling its surface at `v = 0`
/// and `v = 1` has to land on the sections the loft was asked to pass through.
fn assert_faces_reach(
    model: &Model<StandardPayload>,
    faces: &[FaceKey],
    on_first: impl Fn(Point3) -> f64,
    on_last: impl Fn(Point3) -> f64,
) {
    for key in faces {
        let face = Face::new(model, *key);
        for step in 0..=8 {
            let u = step as f64 / 8.0;
            let bottom = face.point_at(u, 0.0);
            let top = face.point_at(u, 1.0);
            assert!(
                on_first(bottom).abs() <= LINEAR_TOLERANCE,
                "face {key:?} at u={u} misses the first section at {bottom:?}"
            );
            assert!(
                on_last(top).abs() <= LINEAR_TOLERANCE,
                "face {key:?} at u={u} misses the last section at {top:?}"
            );
        }
    }
}

#[test]
fn two_squares_loft_to_one_face_per_side() {
    let mut model = Model::<StandardPayload>::new();
    let bottom = square(&mut model, 0.0);
    let top = square(&mut model, 2.0);

    let sections = [closed(&model, bottom), closed(&model, top)];
    let result = add_loft(&mut model, &sections, LoftOptions::default()).unwrap();

    let lofted = model.sheet_unchecked(result);
    let faces = lofted
        .faces()
        .iter()
        .map(|face| face.key())
        .collect::<Vec<_>>();
    assert_eq!(faces.len(), 4);
    // Four columns sewn into a ring: four rails, and four edges on each of
    // the two sections. The source profiles are left in the map, as every
    // sweep in this crate leaves them.
    assert_eq!(lofted.edges().len(), 12);
    assert_eq!(lofted.vertices().len(), 8);
    assert_faces_reach(&model, &faces, |point| point.z, |point| point.z - 2.0);
}

#[test]
fn a_circle_and_a_rectangle_loft_to_a_face_per_rectangle_side() {
    let mut model = Model::<StandardPayload>::new();
    let rectangle = square(&mut model, 0.0);
    let disk = circle(&mut model, 1.0, 3.0);

    let sections = [closed(&model, rectangle), closed(&model, disk)];
    let result = add_loft(&mut model, &sections, LoftOptions::default()).unwrap();

    let lofted = model.sheet_unchecked(result);
    let faces = lofted
        .faces()
        .iter()
        .map(|face| face.key())
        .collect::<Vec<_>>();
    // The circle is one unmarked edge and contributes no breakpoint of its
    // own, so the union is the rectangle's four corners and each corner runs
    // a rail out to the circle.
    assert_eq!(faces.len(), 4);
    assert_faces_reach(
        &model,
        &faces,
        |point| point.z,
        |point| (point.coords.xy().norm() - 1.0).max(point.z - 3.0),
    );
}

#[test]
fn a_loft_through_four_sections_passes_through_the_middle_two() {
    let mut model = Model::<StandardPayload>::new();
    let keys = [0.0, 1.0, 2.0, 3.0]
        .iter()
        .map(|z| square(&mut model, *z))
        .collect::<Vec<_>>();
    let sections = keys
        .iter()
        .map(|key| closed(&model, *key))
        .collect::<Vec<_>>();

    let result = add_loft(&mut model, &sections, LoftOptions::default()).unwrap();

    // Four columns and no more: an intermediate section contributes no face,
    // no edge and no vertex to the result, only its breakpoints.
    let lofted = model.sheet_unchecked(result);
    let faces = lofted
        .faces()
        .iter()
        .map(|face| face.key())
        .collect::<Vec<_>>();
    assert_eq!(faces.len(), 4);
    assert_eq!(lofted.edges().len(), 12);
    assert_eq!(lofted.vertices().len(), 8);
    for key in &faces {
        let face = Face::new(&model, *key);
        for step in 0..=8 {
            let u = step as f64 / 8.0;
            // The two intermediate sections are square rings at z = 1 and 2.
            // The surface climbs monotonically, so the height each sits at
            // names one `v`, and the point there has to be on that ring.
            for height in [1.0, 2.0] {
                let point = face.point_at(u, height_parameter(&face, u, height));
                assert!(
                    (point.x.abs().max(point.y.abs()) - 1.0).abs() <= LINEAR_TOLERANCE,
                    "face {key:?} misses the section at z = {height}: {point:?}"
                );
            }
        }
    }
}

/// The `v` at which a face's column at `u` reaches `height`.
fn height_parameter(face: &Face<'_, StandardPayload>, u: f64, height: f64) -> f64 {
    let (mut low, mut high) = (0.0, 1.0);
    for _ in 0..80 {
        let middle = 0.5 * (low + high);
        if face.point_at(u, middle).z < height {
            low = middle;
        } else {
            high = middle;
        }
    }
    0.5 * (low + high)
}

#[test]
fn a_corner_in_an_intermediate_section_lands_on_a_rail() {
    let mut model = Model::<StandardPayload>::new();
    // Two triangles with a square between them: the square's corners fall
    // where the triangles have none, so they must become rails of their own.
    let bottom = add_polygon(
        &mut model,
        &[
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
    );
    let middle = square(&mut model, 1.0);
    let top = add_polygon(
        &mut model,
        &[
            Point3::new(-1.0, -1.0, 2.0),
            Point3::new(1.0, -1.0, 2.0),
            Point3::new(0.0, 1.0, 2.0),
        ],
    );

    let sections = [
        closed(&model, bottom),
        closed(&model, middle),
        closed(&model, top),
    ];
    let result = add_loft(&mut model, &sections, LoftOptions::default()).unwrap();

    // Three triangle corners and four square corners, none coincident in
    // parameter, so the union opens a column at each.
    let lofted = model.sheet_unchecked(result);
    let faces = lofted
        .faces()
        .iter()
        .map(|face| face.key())
        .collect::<Vec<_>>();
    assert!(
        faces.len() >= 7,
        "the square's corners should each open a column, got {} faces",
        faces.len()
    );

    // Every crease is on a rail, which is an edge of two faces, so none sits
    // in a face interior: the normal is continuous across each face's own
    // domain.
    for key in &faces {
        let face = Face::new(&model, *key);
        let mut previous = face.normal_at(0.0, 0.5);
        for step in 1..=32 {
            let normal = face.normal_at(step as f64 / 32.0, 0.5);
            assert!(
                normal.dot(&previous) > 0.9,
                "face {key:?} creases inside its own domain at u = {}",
                step as f64 / 32.0
            );
            previous = normal;
        }
    }
}

#[test]
fn capped_sections_loft_to_a_valid_solid() {
    let mut model = Model::<StandardPayload>::new();
    let bottom_profile = square(&mut model, 0.0);
    let top_profile = square(&mut model, 2.0);
    let bottom = add_face(&mut model, bottom_profile).unwrap();
    let top = add_face(&mut model, top_profile).unwrap();

    let sections = [capped(&model, bottom), capped(&model, top)];
    let result = add_loft(&mut model, &sections, LoftOptions::default()).unwrap();

    let solid = result;
    // Four lateral columns plus the two caps.
    let shell = model.solid_unchecked(solid).shells().remove(0);
    assert_eq!(shell.faces().len(), 6);
    validate_all_solid_manifolds(&model).expect("a capped loft should be manifold");
    validate_all_solid_orientations(&model).expect("a capped loft should face outward");
}

#[test]
fn a_rectangle_and_a_circle_loft_to_a_capped_solid() {
    let mut model = Model::<StandardPayload>::new();
    let bottom_profile = square(&mut model, 0.0);
    let bottom = add_face(&mut model, bottom_profile).unwrap();
    let top = disk(&mut model, 1.0, 2.0);

    let sections = [capped(&model, bottom), capped(&model, top)];
    let solid = add_loft(&mut model, &sections, LoftOptions::default()).unwrap();

    // The square's four corners each open a column, and the circular cap is
    // cut at all four of them: one closed edge becomes four arcs, so the four
    // walls and two caps meet along a closed ring of eight edges.
    let shell = model.solid_unchecked(solid).shells().remove(0);
    assert_eq!(shell.faces().len(), 6);
    assert_eq!(shell.edges().len(), 12);
    validate_all_solid_manifolds(&model).expect("a capped loft should be manifold");
    validate_all_solid_orientations(&model).expect("a capped loft should face outward");
}

#[test]
fn a_ruled_loft_is_straight_between_consecutive_sections() {
    let mut model = Model::<StandardPayload>::new();
    let keys = [0.0, 1.0, 3.0]
        .iter()
        .map(|z| square(&mut model, *z))
        .collect::<Vec<_>>();
    let sections = keys
        .iter()
        .map(|key| closed(&model, *key))
        .collect::<Vec<_>>();

    let result = add_loft(&mut model, &sections, LoftOptions::ruled()).unwrap();

    for face in model.sheet_unchecked(result).faces() {
        let key = face.key();
        // Linear in v means every column of the surface is a straight line in
        // space, whichever pair of sections the sample falls between.
        for step in 0..=4 {
            let u = step as f64 / 4.0;
            let start = face.point_at(u, 0.0);
            let end = face.point_at(u, 1.0);
            for sample in 0..=16 {
                let v = sample as f64 / 16.0;
                let point = face.point_at(u, v);
                let along = end - start;
                let offset = point - start;
                let across = offset - along * (offset.dot(&along) / along.norm_squared());
                assert!(
                    across.norm() <= 1.0e-9,
                    "face {key:?} bends off its ruling at ({u}, {v}) by {}",
                    across.norm()
                );
            }
        }
    }
}

#[test]
fn a_loft_of_open_profiles_gives_an_open_sheet() {
    let mut model = Model::<StandardPayload>::new();
    let bottom = add_polyline(
        &mut model,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ],
    )
    .unwrap();
    let top = add_polyline(
        &mut model,
        &[
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(1.0, 1.5, 2.0),
            Point3::new(2.0, 0.0, 2.0),
        ],
    )
    .unwrap();

    let sections = [open(&model, bottom), open(&model, top)];
    let result = add_loft(&mut model, &sections, LoftOptions::default()).unwrap();

    // Two columns side by side and nothing joining their far ends: the sheet
    // stays open, where a closed section would have closed the ring.
    let lofted = model.sheet_unchecked(result);
    assert_eq!(lofted.faces().len(), 2);
    assert!(!lofted.is_closed());
}

#[test]
fn a_run_cannot_mix_open_and_closed_sections() {
    let mut model = Model::<StandardPayload>::new();
    let ring = square(&mut model, 0.0);
    let wire = add_polyline(
        &mut model,
        &[Point3::new(0.0, 0.0, 2.0), Point3::new(1.0, 0.0, 2.0)],
    )
    .unwrap();

    // Mixing the two is not an error the loft reports; it is a slice that
    // cannot be written, because `OpenSection` and `ClosedSection` are
    // different types. What is left to check is that neither can be made from
    // the wrong profile.
    assert_eq!(
        OpenSection::new(&model.profile_unchecked(ring)).unwrap_err(),
        LoftError::ClosedProfileAsOpenSection
    );
    assert!(
        Closed::new(model.profile_unchecked(wire)).is_none(),
        "an open profile should carry no proof of closedness to build a section from"
    );
}

#[test]
fn a_section_authored_backwards_lofts_to_the_same_shape() {
    let forward = {
        let mut model = Model::<StandardPayload>::new();
        let bottom = square(&mut model, 0.0);
        let top = square(&mut model, 2.0);
        let sections = [closed(&model, bottom), closed(&model, top)];
        add_loft(&mut model, &sections, LoftOptions::default()).unwrap();
        sampled_surface(&model)
    };

    let backward = {
        let mut model = Model::<StandardPayload>::new();
        let bottom = square(&mut model, 0.0);
        // The same square, walked the other way round.
        let top = add_polygon(
            &mut model,
            &[
                Point3::new(-1.0, -1.0, 2.0),
                Point3::new(-1.0, 1.0, 2.0),
                Point3::new(1.0, 1.0, 2.0),
                Point3::new(1.0, -1.0, 2.0),
            ],
        );
        let sections = [closed(&model, bottom), closed(&model, top)];
        add_loft(&mut model, &sections, LoftOptions::default()).unwrap();
        sampled_surface(&model)
    };

    assert_eq!(forward.len(), backward.len());
    for point in &backward {
        assert!(
            forward
                .iter()
                .any(|other| (other - point).norm() <= LINEAR_TOLERANCE),
            "a reversed section lofted to a bowtie: {point:?} is not on the forward shape"
        );
    }
}

/// Every loft face sampled on a fixed grid, for comparing two builds.
fn sampled_surface(model: &Model<StandardPayload>) -> Vec<Point3> {
    let mut points = Vec::new();
    for (key, _) in model.iter_faces() {
        let face = Face::new(model, key);
        for u in 0..=4 {
            for v in 0..=4 {
                points.push(face.point_at(u as f64 / 4.0, v as f64 / 4.0));
            }
        }
    }
    points
}

#[test]
fn a_loft_needs_at_least_two_sections() {
    let mut model = Model::<StandardPayload>::new();
    let only = square(&mut model, 0.0);
    let sections = [closed(&model, only)];
    assert_eq!(
        add_loft(&mut model, &sections, LoftOptions::default()).unwrap_err(),
        LoftError::TooFewSections { got: 1 }
    );
}

#[test]
fn a_v_degree_the_section_count_cannot_carry_is_refused() {
    let mut model = Model::<StandardPayload>::new();
    let bottom = square(&mut model, 0.0);
    let top = square(&mut model, 2.0);

    let sections = [closed(&model, bottom), closed(&model, top)];
    let error = add_loft(
        &mut model,
        &sections,
        LoftOptions {
            v_degree: Some(Degree::new(3).unwrap()),
        },
    )
    .unwrap_err();
    assert!(
        matches!(error, LoftError::Column { .. }),
        "expected a column failure, got {error}"
    );
}

#[test]
fn a_face_with_a_hole_is_refused_rather_than_lofted_by_its_outer_loop() {
    let mut model = Model::<StandardPayload>::new();
    let bottom = add_annulus(&mut model, Plane::xy(), 2.0, 1.0).unwrap();
    let top = add_annulus(
        &mut model,
        Plane::new(
            Point3::new(0.0, 0.0, 2.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
        ),
        3.0,
        1.5,
    )
    .unwrap();

    // Refused when the section is built, not when the loft runs: a face with
    // a hole never becomes a `CappedSection` at all.
    assert_eq!(
        CappedSection::new(&Face::new(&model, bottom)).unwrap_err(),
        LoftError::FaceHasInnerLoops { count: 1 }
    );
    assert_eq!(
        CappedSection::new(&Face::new(&model, top)).unwrap_err(),
        LoftError::FaceHasInnerLoops { count: 1 }
    );
}

#[test]
fn two_circles_loft_to_one_wrapping_face() {
    let mut model = Model::<StandardPayload>::new();
    let bottom = circle(&mut model, 1.0, 0.0);
    let top = circle(&mut model, 2.0, 3.0);

    let sections = [closed(&model, bottom), closed(&model, top)];
    let result = add_loft(&mut model, &sections, LoftOptions::ruled()).unwrap();

    // Neither section carries a breakpoint, so the union is the whole run and
    // there is one column. It closes onto itself, so it is built as a band:
    // bounded by the two section circles and by nothing else. No rail, no
    // seam edge, and no vertex — neither circle had one to keep.
    let lofted = model.sheet_unchecked(result);
    assert_eq!(lofted.faces().len(), 1);
    assert_eq!(lofted.edges().len(), 2);
    assert!(lofted.vertices().is_empty());
    for edge in lofted.edges() {
        assert!(
            matches!(edge, Edge::Unmarked(_)),
            "a lofted circle should come back unmarked, as it went in"
        );
    }

    let face = Face::new(&model, lofted.faces()[0].key());
    for step in 0..=16 {
        let u = step as f64 / 16.0;
        for (v, radius, z) in [(0.0, 1.0, 0.0), (1.0, 2.0, 3.0)] {
            let point = face.point_at(u, v);
            assert!(
                (point.coords.xy().norm() - radius).abs() <= LINEAR_TOLERANCE
                    && (point.z - z).abs() <= LINEAR_TOLERANCE,
                "the wall misses the circle of radius {radius} at ({u}, {v}): {point:?}"
            );
        }
    }
}

#[test]
fn two_circular_faces_loft_to_a_seamless_frustum() {
    let mut model = Model::<StandardPayload>::new();
    let bottom = disk(&mut model, 2.0, 0.0);
    let top = disk(&mut model, 1.0, 3.0);

    let sections = [capped(&model, bottom), capped(&model, top)];
    let solid = add_loft(&mut model, &sections, LoftOptions::ruled()).unwrap();

    // A wall and two caps, joined along the two circles and nothing else: a
    // frustum written the way a revolved one is, with no seam up its side.
    let shell = model.solid_unchecked(solid).shells().remove(0);
    assert_eq!(shell.faces().len(), 3);
    assert_eq!(shell.edges().len(), 2);
    assert!(shell.vertices().is_empty());
    validate_all_solid_manifolds(&model).expect("a lofted frustum should be manifold");
    validate_all_solid_orientations(&model).expect("a lofted frustum should face outward");
}
