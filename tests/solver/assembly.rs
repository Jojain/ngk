use nalgebra::Vector3;
use ngk::geometry::Axis3;
use ngk::geometry::dim3::frame::Frame;
use ngk::geometry::dim3::utils::Point3;
use ngk::geometry::transform::Rigid;
use ngk::solver::assembly::{
    Assembly3d, Attachment, BodyId, Bound, Coordinate, EditError, Joint3d, JointId, JointKind,
    Limit, Solution3d, SolveError, Target3d,
};
use radians::Rad64;

const CLOSE: f64 = 1e-8;

fn turn(axis: Vector3<f64>, degrees: f64) -> Rigid {
    Rigid::rotation(
        Axis3::new(Point3::origin(), axis),
        Rad64::new(degrees.to_radians()),
    )
}

fn shift(x: f64, y: f64, z: f64) -> Rigid {
    Rigid::translation(Vector3::new(x, y, z))
}

/// A frame at `origin` with the world's axes.
fn at(x: f64, y: f64, z: f64) -> Frame {
    Frame::at(Point3::new(x, y, z))
}

fn hold(body: BodyId, frame: Frame) -> Attachment {
    Attachment::new(body, frame)
}

/// The world frame an attachment lands on in a solution.
fn landed(solution: &Solution3d, attachment: &Attachment) -> Frame {
    attachment
        .frame
        .moved(&solution.pose(attachment.body).unwrap())
}

fn assert_point(actual: Point3, expected: Point3) {
    assert!(
        (actual - expected).norm() < CLOSE,
        "{actual:?} is not {expected:?}"
    );
}

fn assert_direction(actual: Vector3<f64>, expected: Vector3<f64>) {
    assert!(
        (actual - expected).norm() < CLOSE,
        "{actual:?} is not {expected:?}"
    );
}

/// The angle that turns the first frame's X onto the second's, about the first's Z.
fn turned_about_z(a: &Frame, b: &Frame) -> f64 {
    let x = b.x_dir.into_inner();
    x.dot(&a.y_dir).atan2(x.dot(&a.x_dir)).to_degrees()
}

/// A ground fixed at the origin and a free body seeded at `seed`.
fn ground_and_body(seed: Rigid) -> (Assembly3d, BodyId, BodyId) {
    let mut assembly = Assembly3d::default();
    let ground = assembly.add_body(Rigid::identity()).unwrap();
    assembly.fix(ground).unwrap();
    let body = assembly.add_body(seed).unwrap();
    (assembly, ground, body)
}

#[test]
fn a_revolute_joint_brings_origins_and_axes_together_and_leaves_one_turn() {
    let seed = turn(Vector3::new(1., 2., 0.5), 25.).compose(shift(0.3, -0.2, 0.4));
    let (mut assembly, ground, body) = ground_and_body(seed);
    let joint = Joint3d::new(
        JointKind::Revolute,
        hold(ground, at(1., 2., 3.)),
        hold(body, at(0., 0., 1.)),
    );
    assembly.add_joint(joint.clone()).unwrap();

    let solution = assembly.solve().unwrap();

    let (a, b) = (
        landed(&solution, &joint.first),
        landed(&solution, &joint.second),
    );
    assert_point(b.origin, a.origin);
    assert_direction(b.z_dir.into_inner(), a.z_dir.into_inner());
    let report = solution.report();
    assert_eq!(report.mobility, 1);
    assert_eq!(report.redundancy, 0);
    assert_eq!(report.freedoms.len(), 2);
    let freedom = |id| {
        report
            .freedoms
            .iter()
            .find(|f| f.body == id)
            .unwrap()
            .degrees
    };
    assert_eq!(freedom(ground), 0);
    assert_eq!(freedom(body), 1);
    assert!(report.redundancies.is_empty());
}

#[test]
fn a_prismatic_joint_keeps_orientation_and_leaves_the_slide() {
    let seed = turn(Vector3::new(0., 1., 1.), -15.).compose(shift(1., 1., 2.));
    let (mut assembly, ground, body) = ground_and_body(seed);
    let joint = Joint3d::new(
        JointKind::Prismatic,
        hold(ground, at(0., 0., 0.)),
        hold(body, at(0.5, 0., 0.)),
    );
    assembly.add_joint(joint.clone()).unwrap();

    let solution = assembly.solve().unwrap();

    let (a, b) = (
        landed(&solution, &joint.first),
        landed(&solution, &joint.second),
    );
    assert_direction(b.x_dir.into_inner(), a.x_dir.into_inner());
    assert_direction(b.z_dir.into_inner(), a.z_dir.into_inner());
    let offset = b.origin - a.origin;
    assert!(offset.x.abs() < CLOSE && offset.y.abs() < CLOSE);
    assert_eq!(solution.report().mobility, 1);
}

#[test]
fn a_fixed_joint_makes_the_frames_coincide_and_leaves_nothing() {
    let seed = turn(Vector3::new(1., -1., 2.), 70.).compose(shift(-2., 5., 1.));
    let (mut assembly, ground, body) = ground_and_body(seed);
    let first = Frame::from_xz(Point3::new(1., 0., 0.), Vector3::y(), Vector3::x());
    let joint = Joint3d::new(
        JointKind::Fixed,
        hold(ground, first),
        hold(body, at(0., 0., 2.)),
    );
    assembly.add_joint(joint.clone()).unwrap();

    let solution = assembly.solve().unwrap();

    let (a, b) = (
        landed(&solution, &joint.first),
        landed(&solution, &joint.second),
    );
    assert_point(b.origin, a.origin);
    assert_direction(b.x_dir.into_inner(), a.x_dir.into_inner());
    assert_direction(b.y_dir.into_inner(), a.y_dir.into_inner());
    assert_eq!(solution.report().mobility, 0);
}

#[test]
fn a_satisfied_revolute_joint_keeps_its_angle() {
    let seed = turn(Vector3::z(), 30.);
    let (mut assembly, ground, body) = ground_and_body(seed);
    let joint = Joint3d::new(
        JointKind::Revolute,
        hold(ground, at(0., 0., 0.)),
        hold(body, at(0., 0., 0.)),
    );
    assembly.add_joint(joint.clone()).unwrap();

    let solution = assembly.solve().unwrap();

    let angle = turned_about_z(
        &landed(&solution, &joint.first),
        &landed(&solution, &joint.second),
    );
    assert!((angle - 30.).abs() < 1e-6, "{angle}");
    assert_eq!(solution.report().iterations, 0);
}

#[test]
fn an_axis_turned_nearly_over_comes_back_rather_than_settling_upside_down() {
    let seed = turn(Vector3::x(), 170.).compose(shift(0.2, 0.1, -0.3));
    let (mut assembly, ground, body) = ground_and_body(seed);
    let joint = Joint3d::new(
        JointKind::Revolute,
        hold(ground, at(0., 0., 0.)),
        hold(body, at(0., 0., 0.)),
    );
    assembly.add_joint(joint.clone()).unwrap();

    let solution = assembly.solve().unwrap();

    let b = landed(&solution, &joint.second);
    assert_direction(b.z_dir.into_inner(), Vector3::z());
}

#[test]
fn without_a_fixed_body_the_assembly_keeps_its_global_motion() {
    let mut assembly = Assembly3d::default();
    let a = assembly.add_body(Rigid::identity()).unwrap();
    let b = assembly
        .add_body(turn(Vector3::y(), 10.).compose(shift(0.1, 0., 0.)))
        .unwrap();
    assembly
        .add_joint(Joint3d::new(
            JointKind::Revolute,
            hold(a, at(0., 0., 0.)),
            hold(b, at(0., 0., 0.)),
        ))
        .unwrap();

    let solution = assembly.solve().unwrap();

    assert_eq!(solution.report().mobility, 7);
}

/// Where the rocker of a four-bar linkage meets the coupler: `|b - a| = coupler` and
/// `|b - pivot| = rocker`, on the upper side.
fn closing_point(a: Point3, pivot: Point3, coupler: f64, rocker: f64) -> Point3 {
    let d = (pivot - a).norm();
    let along = (coupler * coupler - rocker * rocker + d * d) / (2. * d);
    let height = (coupler * coupler - along * along).sqrt();
    let u = (pivot - a) / d;
    let n = Vector3::new(-u.y, u.x, 0.);
    a + u * along + n * height
}

struct FourBar {
    assembly: Assembly3d,
    crank: BodyId,
    coupler: BodyId,
    joints: Vec<Joint3d>,
    /// The coupler's point between its two pins, in its own coordinates.
    middle: Point3,
}

/// A planar four-bar linkage: ground 4, crank 1, coupler 4, rocker 2, crank at 60°,
/// every body seeded slightly off its closed position.
fn four_bar() -> FourBar {
    let a0 = Point3::origin();
    let b0 = Point3::new(4., 0., 0.);
    let a = Point3::new(60_f64.to_radians().cos(), 60_f64.to_radians().sin(), 0.);
    let b = closing_point(a, b0, 4., 2.);
    let pin = |p: Point3| Frame::at(p);
    let mut assembly = Assembly3d::default();
    let ground = assembly.add_body(Rigid::identity()).unwrap();
    assembly.fix(ground).unwrap();
    let crank = assembly
        .add_body(turn(Vector3::new(0.2, 0.1, 1.), 4.).compose(shift(0.05, -0.02, 0.03)))
        .unwrap();
    let coupler = assembly
        .add_body(turn(Vector3::new(-0.1, 0.3, 1.), -3.).compose(shift(-0.04, 0.06, -0.02)))
        .unwrap();
    let rocker = assembly
        .add_body(turn(Vector3::new(0.1, -0.2, 1.), 5.).compose(shift(0.03, 0.05, 0.04)))
        .unwrap();
    let joints = vec![
        Joint3d::new(
            JointKind::Revolute,
            hold(ground, pin(a0)),
            hold(crank, pin(a0)),
        ),
        Joint3d::new(
            JointKind::Revolute,
            hold(crank, pin(a)),
            hold(coupler, pin(a)),
        ),
        Joint3d::new(
            JointKind::Revolute,
            hold(coupler, pin(b)),
            hold(rocker, pin(b)),
        ),
        Joint3d::new(
            JointKind::Revolute,
            hold(rocker, pin(b0)),
            hold(ground, pin(b0)),
        ),
    ];
    for joint in &joints {
        assembly.add_joint(joint.clone()).unwrap();
    }
    FourBar {
        assembly,
        crank,
        coupler,
        joints,
        middle: Point3::from((a.coords + b.coords) / 2.),
    }
}

fn assert_closed(solution: &Solution3d, joints: &[Joint3d]) {
    for joint in joints {
        let (a, b) = (
            landed(solution, &joint.first),
            landed(solution, &joint.second),
        );
        assert_point(b.origin, a.origin);
        assert_direction(b.z_dir.into_inner(), a.z_dir.into_inner());
    }
}

#[test]
fn a_four_bar_linkage_closes_with_one_mobility_and_three_redundant_equations() {
    let mut linkage = four_bar();

    let solution = linkage.assembly.solve().unwrap();

    assert_closed(&solution, &linkage.joints);
    let report = solution.report();
    assert_eq!(report.mobility, 1);
    // Four revolute joints in a plane: 20 equations for 18 unknowns and one mobility.
    assert_eq!(report.redundancy, 3);
    assert!(!report.redundancies.is_empty());
}

#[test]
fn dragging_the_coupler_turns_the_crank_and_keeps_the_loop_closed() {
    let mut linkage = four_bar();
    let first = linkage.assembly.solve().unwrap();
    let before = first
        .pose(linkage.crank)
        .unwrap()
        .apply_vector(Vector3::x());
    let start = first.pose(linkage.coupler).unwrap().apply(linkage.middle);
    let goal = start + Vector3::new(-0.6, 0.4, 0.);

    let moved = linkage
        .assembly
        .solve_toward(&[Target3d::point(linkage.coupler, linkage.middle, goal)])
        .unwrap();

    assert_closed(&moved, &linkage.joints);
    let reached = moved.pose(linkage.coupler).unwrap().apply(linkage.middle);
    assert!((reached - goal).norm() < (start - goal).norm());
    let after = moved
        .pose(linkage.crank)
        .unwrap()
        .apply_vector(Vector3::x());
    assert!(before.angle(&after) > 1e-3, "the crank did not turn");
    let report = moved.report();
    assert_eq!(report.targets.len(), 1);
    assert!((report.targets[0].achieved - reached).norm() < CLOSE);
    // A success becomes the next seed.
    assert_eq!(
        linkage.assembly.body(linkage.coupler).unwrap(),
        moved.pose(linkage.coupler).unwrap()
    );
}

#[test]
fn a_target_beyond_reach_stops_where_the_joints_allow() {
    let (mut assembly, ground, body) = ground_and_body(Rigid::identity());
    assembly
        .add_joint(Joint3d::new(
            JointKind::Revolute,
            hold(ground, at(0., 0., 0.)),
            hold(body, at(0., 0., 0.)),
        ))
        .unwrap();
    let tip = Point3::new(1., 0., 0.);

    let solution = assembly
        .solve_toward(&[Target3d::point(body, tip, Point3::new(0., 5., 3.))])
        .unwrap();

    let reached = solution.pose(body).unwrap().apply(tip);
    // Near the best reachable point the distance changes with the square of the
    // remaining turn, so the improvement stops once that gain is negligible.
    assert!(
        (reached - Point3::new(0., 1., 0.)).norm() < 1e-5,
        "{reached:?}"
    );
    assert!(!solution.report().targets[0].reached);
    assert_closed(
        &solution,
        &[Joint3d::new(
            JointKind::Revolute,
            hold(ground, at(0., 0., 0.)),
            hold(body, at(0., 0., 0.)),
        )],
    );
}

#[test]
fn two_fixed_bodies_joined_where_they_agree_are_redundant() {
    let mut assembly = Assembly3d::default();
    let a = assembly.add_body(Rigid::identity()).unwrap();
    let b = assembly.add_body(shift(1., 0., 0.)).unwrap();
    assembly.fix(a).unwrap();
    assembly.fix(b).unwrap();
    assembly
        .add_joint(Joint3d::new(
            JointKind::Fixed,
            hold(a, at(1., 0., 0.)),
            hold(b, at(0., 0., 0.)),
        ))
        .unwrap();

    let solution = assembly.solve().unwrap();

    assert_eq!(solution.report().mobility, 0);
    assert_eq!(solution.report().redundancy, 6);
}

#[test]
fn two_fixed_bodies_joined_where_they_disagree_fail_naming_the_joint() {
    let mut assembly = Assembly3d::default();
    let a = assembly.add_body(Rigid::identity()).unwrap();
    let b = assembly.add_body(shift(1., 0., 0.)).unwrap();
    assembly.fix(a).unwrap();
    assembly.fix(b).unwrap();
    let joint = assembly
        .add_joint(Joint3d::new(
            JointKind::Fixed,
            hold(a, at(2., 0., 0.)),
            hold(b, at(0., 0., 0.)),
        ))
        .unwrap();

    let error = assembly.solve().unwrap_err();

    let SolveError::NonConvergence { joints, residuals } = error else {
        panic!("{error:?}");
    };
    assert_eq!(joints, vec![joint]);
    assert_eq!(residuals[0].joint, joint);
    assert!(assembly.last_solution().is_none());
}

#[test]
fn a_fixing_holds_the_pose_it_captured() {
    let mut assembly = Assembly3d::default();
    let body = assembly.add_body(shift(1., 2., 3.)).unwrap();
    assembly.fix(body).unwrap();
    assembly.update_body(body, shift(9., 9., 9.)).unwrap();

    let solution = assembly.solve().unwrap();

    assert_eq!(solution.pose(body).unwrap(), shift(1., 2., 3.));
    assert_eq!(assembly.fixing(body).unwrap(), Some(shift(1., 2., 3.)));
    assembly.fix(body).unwrap();
    assert_eq!(assembly.fixing(body).unwrap(), Some(shift(1., 2., 3.)));
}

#[test]
fn an_assembly_far_from_the_origin_still_closes_in_world_units() {
    let far = shift(1e6, -2e6, 5e5);
    let mut assembly = Assembly3d::default();
    let ground = assembly.add_body(far).unwrap();
    assembly.fix(ground).unwrap();
    let body = assembly
        .add_body(turn(Vector3::new(1., 1., 0.), 20.).compose(far.compose(shift(0.5, 0., 0.))))
        .unwrap();
    let joint = Joint3d::new(
        JointKind::Revolute,
        hold(ground, at(1., 0., 0.)),
        hold(body, at(0., 0., 0.)),
    );
    assembly.add_joint(joint.clone()).unwrap();

    let solution = assembly.solve().unwrap();

    let (a, b) = (
        landed(&solution, &joint.first),
        landed(&solution, &joint.second),
    );
    assert!((b.origin - a.origin).norm() < 1e-8);
    assert!(solution.report().residuals[0].max_tolerance_ratio <= 1.);
}

#[test]
fn a_joint_must_hold_two_bodies_by_proper_frames() {
    let mut assembly = Assembly3d::default();
    let a = assembly.add_body(Rigid::identity()).unwrap();
    let b = assembly.add_body(Rigid::identity()).unwrap();
    let same = Joint3d::new(
        JointKind::Revolute,
        hold(a, at(0., 0., 0.)),
        hold(a, at(1., 0., 0.)),
    );
    assert!(matches!(
        assembly.add_joint(same),
        Err(EditError::InvalidJoint(_))
    ));
    let mut skewed = at(0., 0., 0.);
    skewed.y_dir = nalgebra::Unit::new_normalize(Vector3::new(1., 1., 0.));
    let crooked = Joint3d::new(JointKind::Fixed, hold(a, skewed), hold(b, at(0., 0., 0.)));
    assert!(matches!(
        assembly.add_joint(crooked),
        Err(EditError::InvalidJoint(_))
    ));
    let missing = Joint3d::new(
        JointKind::Fixed,
        hold(a, at(0., 0., 0.)),
        hold(BodyId::new(99), at(0., 0., 0.)),
    );
    assert!(matches!(
        assembly.add_joint(missing),
        Err(EditError::InvalidBodyReference(_))
    ));
    assert_eq!(assembly.joint_count(), 0);
}

#[test]
fn a_held_body_is_removed_only_with_its_joints() {
    let mut assembly = Assembly3d::default();
    let a = assembly.add_body(Rigid::identity()).unwrap();
    let b = assembly.add_body(Rigid::identity()).unwrap();
    let joint = assembly
        .add_joint(Joint3d::new(
            JointKind::Fixed,
            hold(a, at(0., 0., 0.)),
            hold(b, at(0., 0., 0.)),
        ))
        .unwrap();

    let refused = assembly.remove_body(b).unwrap_err();
    assert!(matches!(
        refused,
        EditError::ReferencedBody { joints, .. } if joints == vec![joint]
    ));
    let removal = assembly.remove_cascade(b).unwrap();

    assert_eq!(removal.joints, vec![joint]);
    assert_eq!(assembly.body_count(), 1);
    assert_eq!(assembly.joint_count(), 0);
}

#[test]
fn an_aborted_batch_leaves_the_assembly_and_does_not_reissue_its_ids() {
    let mut assembly = Assembly3d::default();
    let a = assembly.add_body(Rigid::identity()).unwrap();
    let revision = assembly.revision();

    let aborted = assembly.edit(|assembly| {
        let b = assembly.add_body(Rigid::identity())?;
        assembly.add_joint(Joint3d::new(
            JointKind::Fixed,
            hold(a, at(0., 0., 0.)),
            hold(a, at(0., 0., 0.)),
        ))?;
        Ok(b)
    });

    assert!(aborted.is_err());
    assert_eq!(assembly.body_count(), 1);
    assert_eq!(assembly.revision(), revision);
    let next = assembly.add_body(Rigid::identity()).unwrap();
    assert!(next.value() > a.value() + 1);
}

#[test]
fn a_stored_id_is_taken_back_and_never_issued_again() {
    let mut assembly = Assembly3d::default();
    assembly
        .insert_body(BodyId::new(7), Rigid::identity())
        .unwrap();
    assert!(matches!(
        assembly.insert_body(BodyId::new(7), Rigid::identity()),
        Err(EditError::DuplicateBody(_))
    ));
    let next = assembly.add_body(Rigid::identity()).unwrap();
    assert!(next.value() > 7);
}

fn degrees(radians: f64) -> f64 {
    radians.to_degrees()
}

/// One joint between a fixed ground and a body seeded at `seed`, both frames at their
/// body's origin.
fn joint_on_ground(kind: JointKind, seed: Rigid) -> (Assembly3d, Joint3d, JointId, BodyId) {
    let (mut assembly, ground, body) = ground_and_body(seed);
    let joint = Joint3d::new(
        kind,
        hold(ground, at(0., 0., 0.)),
        hold(body, at(0., 0., 0.)),
    );
    let id = assembly.add_joint(joint.clone()).unwrap();
    (assembly, joint, id, body)
}

#[test]
fn every_joint_holds_its_frames_as_its_kind_says_and_leaves_its_freedoms() {
    let seed = turn(Vector3::new(0.3, -0.5, 1.), 20.).compose(shift(0.4, -0.3, 0.25));
    let pitch = 0.8;
    let cases: [(JointKind, usize); 8] = [
        (JointKind::Helical { pitch }, 1),
        (JointKind::Cylindrical, 2),
        (JointKind::Universal, 2),
        (JointKind::Spherical, 3),
        (JointKind::Planar, 3),
        (JointKind::CylinderOnPlane, 4),
        (JointKind::SphereInCylinder, 4),
        (JointKind::SphereOnPlane, 5),
    ];
    for (kind, freedoms) in cases {
        let (mut assembly, joint, id, _) = joint_on_ground(kind, seed);
        let solution = assembly.solve().unwrap();
        let (a, b) = (
            landed(&solution, &joint.first),
            landed(&solution, &joint.second),
        );
        let d = b.origin - a.origin;
        let (xa, ya, za) = (
            a.x_dir.into_inner(),
            a.y_dir.into_inner(),
            a.z_dir.into_inner(),
        );
        let (xb, yb, zb) = (
            b.x_dir.into_inner(),
            b.y_dir.into_inner(),
            b.z_dir.into_inner(),
        );
        let zero = |v: f64, what: &str| assert!(v.abs() < CLOSE, "{kind:?}: {what} is {v}");
        match kind {
            JointKind::Helical { pitch } => {
                zero(d.dot(&xa), "x");
                zero(d.dot(&ya), "y");
                zero((zb - za).norm(), "axis");
                let angle = solution.coordinate(id, Coordinate::Angle).unwrap();
                zero(d.dot(&za) - pitch * angle / std::f64::consts::TAU, "lead");
                zero(
                    solution.coordinate(id, Coordinate::Slide).unwrap() - d.dot(&za),
                    "slide",
                );
            }
            JointKind::Cylindrical => {
                zero(d.dot(&xa), "x");
                zero(d.dot(&ya), "y");
                zero((zb - za).norm(), "axis");
            }
            JointKind::Universal => {
                zero(d.norm(), "origin");
                zero(xa.dot(&yb), "cross");
            }
            JointKind::Spherical => zero(d.norm(), "origin"),
            JointKind::Planar => {
                zero(d.dot(&za), "height");
                zero((zb - za).norm(), "normal");
            }
            JointKind::CylinderOnPlane => {
                zero(d.dot(&za), "height");
                zero(xb.dot(&za), "line");
            }
            JointKind::SphereInCylinder => {
                zero(d.dot(&xa), "x");
                zero(d.dot(&ya), "y");
            }
            JointKind::SphereOnPlane => zero(d.dot(&za), "height"),
            _ => unreachable!(),
        }
        assert_eq!(solution.report().mobility, freedoms, "{kind:?}");
        assert_eq!(solution.report().redundancy, 0, "{kind:?}");
    }
}

#[test]
fn coordinates_read_the_second_frame_in_the_first() {
    let (x, y, angle, roll) = (1.5, -0.5, 40_f64, -25_f64);
    let cylinder = turn(Vector3::x(), roll)
        .compose(turn(Vector3::z(), angle))
        .compose(shift(x, y, 0.));
    let (mut assembly, _, id, _) = joint_on_ground(JointKind::CylinderOnPlane, cylinder);
    let solution = assembly.solve().unwrap();
    assert_eq!(solution.report().iterations, 0);
    let read = |c| solution.coordinate(id, c).unwrap();
    assert!((read(Coordinate::X) - x).abs() < CLOSE);
    assert!((read(Coordinate::Y) - y).abs() < CLOSE);
    assert!((degrees(read(Coordinate::Angle)) - angle).abs() < 1e-6);
    assert!((degrees(read(Coordinate::Roll)) - roll).abs() < 1e-6);

    let (alpha, beta) = (35_f64, -50_f64);
    let cardan = turn(Vector3::y(), beta).compose(turn(Vector3::x(), alpha));
    let (mut assembly, _, id, _) = joint_on_ground(JointKind::Universal, cardan);
    let solution = assembly.solve().unwrap();
    let read = |c| degrees(solution.coordinate(id, c).unwrap());
    assert!((read(Coordinate::Alpha) - alpha).abs() < 1e-6);
    assert!((read(Coordinate::Beta) - beta).abs() < 1e-6);
    assert!(solution.coordinate(id, Coordinate::Slide).is_err());

    let ball = turn(Vector3::new(1., 2., 3.), 70.);
    let (mut assembly, _, id, _) = joint_on_ground(JointKind::Spherical, ball);
    let solution = assembly.solve().unwrap();
    let relative = solution.relative(id).unwrap();
    let probe = Vector3::new(0.3, -1., 2.);
    assert!((relative.apply_vector(probe) - ball.apply_vector(probe)).norm() < CLOSE);
}

#[test]
fn an_angle_is_read_nearest_its_seed_and_the_solve_keeps_it_as_the_next_seed() {
    let (mut assembly, _, id, _) = joint_on_ground(JointKind::Revolute, turn(Vector3::z(), 10.));
    let unseeded = assembly.solve().unwrap();
    assert!((degrees(unseeded.coordinate(id, Coordinate::Angle).unwrap()) - 10.).abs() < 1e-6);
    assert!(
        assembly
            .coordinate_seed(id, Coordinate::Angle)
            .unwrap()
            .is_some()
    );

    assembly
        .set_coordinate_seed(id, Coordinate::Angle, 365_f64.to_radians())
        .unwrap();
    let seeded = assembly.solve().unwrap();

    assert!((degrees(seeded.coordinate(id, Coordinate::Angle).unwrap()) - 370.).abs() < 1e-6);
    let kept = assembly
        .coordinate_seed(id, Coordinate::Angle)
        .unwrap()
        .unwrap();
    assert!((degrees(kept) - 370.).abs() < 1e-6);
    assert!(matches!(
        assembly.set_coordinate_seed(id, Coordinate::Slide, 1.),
        Err(EditError::InvalidJoint(_))
    ));
}

#[test]
fn dragging_a_crank_past_half_a_turn_keeps_counting_the_angle() {
    let (mut assembly, _, id, body) = joint_on_ground(JointKind::Revolute, Rigid::identity());
    let tip = Point3::new(1., 0., 0.);
    let mut angle = 0_f64;
    for step in 1..=8 {
        let goal = 45_f64 * step as f64;
        let position = Point3::new(goal.to_radians().cos(), goal.to_radians().sin(), 0.);
        let solution = assembly
            .solve_toward(&[Target3d::point(body, tip, position)])
            .unwrap();
        angle = degrees(solution.coordinate(id, Coordinate::Angle).unwrap());
    }
    assert!((angle - 360.).abs() < 1e-4, "{angle}");
}

#[test]
fn an_imposed_angle_drives_a_screw_its_whole_way() {
    let pitch = 1.5;
    let (mut assembly, ground, body) = ground_and_body(shift(0.1, 0.1, 0.));
    let joint = Joint3d::new(
        JointKind::Helical { pitch },
        hold(ground, at(0., 0., 0.)),
        hold(body, at(0., 0., 0.)),
    )
    .with_value(Coordinate::Angle, 720_f64.to_radians());
    let id = assembly.add_joint(joint).unwrap();

    let solution = assembly.solve().unwrap();

    let angle = degrees(solution.coordinate(id, Coordinate::Angle).unwrap());
    assert!((angle - 720.).abs() < 1e-6, "{angle}");
    assert!((solution.coordinate(id, Coordinate::Slide).unwrap() - 2. * pitch).abs() < CLOSE);
    assert_eq!(solution.report().mobility, 0);
}

#[test]
fn imposed_values_act_as_dimensions() {
    let seed = turn(Vector3::z(), 5.).compose(shift(0.2, 0.1, 0.3));
    let (mut assembly, ground, body) = ground_and_body(seed);
    let joint = Joint3d::new(
        JointKind::Cylindrical,
        hold(ground, at(0., 0., 0.)),
        hold(body, at(0., 0., 0.)),
    )
    .with_value(Coordinate::Angle, 30_f64.to_radians())
    .with_value(Coordinate::Slide, 2.5);
    let id = assembly.add_joint(joint.clone()).unwrap();

    let solution = assembly.solve().unwrap();

    assert!((degrees(solution.coordinate(id, Coordinate::Angle).unwrap()) - 30.).abs() < 1e-6);
    assert!((solution.coordinate(id, Coordinate::Slide).unwrap() - 2.5).abs() < CLOSE);
    let b = landed(&solution, &joint.second);
    assert_point(b.origin, Point3::new(0., 0., 2.5));
    assert_eq!(solution.report().mobility, 0);
}

#[test]
fn only_a_free_coordinate_of_a_sound_joint_is_imposed() {
    let mut assembly = Assembly3d::default();
    let a = assembly.add_body(Rigid::identity()).unwrap();
    let b = assembly.add_body(Rigid::identity()).unwrap();
    let joint = |kind| Joint3d::new(kind, hold(a, at(0., 0., 0.)), hold(b, at(0., 0., 0.)));
    let refused = [
        joint(JointKind::Revolute).with_value(Coordinate::Slide, 1.),
        joint(JointKind::Helical { pitch: 1. }).with_value(Coordinate::Slide, 1.),
        joint(JointKind::Spherical).with_value(Coordinate::Angle, 1.),
        joint(JointKind::Prismatic).with_value(Coordinate::Slide, f64::NAN),
        joint(JointKind::Helical { pitch: 0. }),
    ];
    for joint in refused {
        assert!(
            matches!(
                assembly.add_joint(joint.clone()),
                Err(EditError::InvalidJoint(_))
            ),
            "{joint:?}"
        );
    }
    assert_eq!(assembly.joint_count(), 0);
}

#[test]
fn replacing_a_joint_keeps_the_seeds_of_the_angles_it_still_has() {
    let (mut assembly, joint, id, _) = joint_on_ground(JointKind::Revolute, Rigid::identity());
    assembly
        .set_coordinate_seed(id, Coordinate::Angle, 400_f64.to_radians())
        .unwrap();
    let mut cylindrical = joint.clone();
    cylindrical.kind = JointKind::Cylindrical;
    assembly.update_joint(id, cylindrical).unwrap();
    assert!(
        assembly
            .coordinate_seed(id, Coordinate::Angle)
            .unwrap()
            .is_some()
    );
    let mut sliding = joint;
    sliding.kind = JointKind::Prismatic;
    assembly.update_joint(id, sliding).unwrap();
    assert!(
        assembly
            .coordinate_seed(id, Coordinate::Angle)
            .unwrap()
            .is_none()
    );
}

#[test]
fn a_slider_crank_closes_and_its_slider_follows_the_crank() {
    let a = Point3::new(30_f64.to_radians().cos(), 30_f64.to_radians().sin(), 0.);
    let c = Point3::new(a.x + (9. - a.y * a.y).sqrt(), 0., 0.);
    // Slide frames: Z along the world X.
    let rail = |p: Point3| Frame::from_xz(p, Vector3::y(), Vector3::x());
    let mut assembly = Assembly3d::default();
    let ground = assembly.add_body(Rigid::identity()).unwrap();
    assembly.fix(ground).unwrap();
    let crank = assembly.add_body(turn(Vector3::z(), 3.)).unwrap();
    let rod = assembly.add_body(shift(0.05, -0.04, 0.02)).unwrap();
    let slider = assembly
        .add_body(turn(Vector3::y(), 2.).compose(shift(0.1, 0.05, 0.)))
        .unwrap();
    let revolute = |p: Point3, first, second| {
        Joint3d::new(
            JointKind::Revolute,
            hold(first, Frame::at(p)),
            hold(second, Frame::at(p)),
        )
    };
    let crank_angle = Joint3d::new(
        JointKind::Revolute,
        hold(ground, Frame::at(Point3::origin())),
        hold(crank, Frame::at(Point3::origin())),
    );
    let joints = vec![
        crank_angle.clone(),
        revolute(a, crank, rod),
        revolute(c, rod, slider),
        Joint3d::new(
            JointKind::Prismatic,
            hold(ground, rail(Point3::origin())),
            hold(slider, rail(c)),
        ),
    ];
    let mut ids = vec![];
    for joint in &joints {
        ids.push(assembly.add_joint(joint.clone()).unwrap());
    }

    let first = assembly.solve().unwrap();
    assert_eq!(first.report().mobility, 1);
    assert_eq!(first.report().redundancy, 3);

    assembly
        .update_joint(
            ids[0],
            // The crank carries its pin at 30° in its own coordinates.
            crank_angle.with_value(Coordinate::Angle, 60_f64.to_radians()),
        )
        .unwrap();
    let driven = assembly.solve().unwrap();

    assert_eq!(driven.report().mobility, 0);
    let pin = driven.pose(crank).unwrap().apply(Point3::new(a.x, a.y, 0.));
    assert_point(pin, Point3::new(0., 1., 0.));
    // The rod is 3 long and the slider stays on the X axis: it sits at √8.
    let slide = driven.coordinate(ids[3], Coordinate::Slide).unwrap();
    assert!((slide - 8_f64.sqrt()).abs() < CLOSE, "{slide}");
}

/// A revolute joint on a fixed ground whose angle is bounded, the body seeded at `start`
/// degrees about Z.
fn bounded_crank(start: f64, limit: Limit) -> (Assembly3d, JointId, BodyId) {
    let (mut assembly, ground, body) = ground_and_body(turn(Vector3::z(), start));
    let joint = Joint3d::new(
        JointKind::Revolute,
        hold(ground, at(0., 0., 0.)),
        hold(body, at(0., 0., 0.)),
    )
    .with_limit(Coordinate::Angle, limit);
    let id = assembly.add_joint(joint).unwrap();
    (assembly, id, body)
}

fn bounds(limit: f64, upper: f64) -> Limit {
    Limit::between(limit.to_radians(), upper.to_radians())
}

fn active(solution: &Solution3d) -> Vec<Bound> {
    solution
        .report()
        .active_limits
        .iter()
        .map(|limit| limit.bound)
        .collect()
}

fn angle_of(solution: &Solution3d, id: JointId) -> f64 {
    degrees(solution.coordinate(id, Coordinate::Angle).unwrap())
}

/// The world position at `degrees` about Z on the unit circle.
fn on_circle(degrees: f64) -> Point3 {
    Point3::new(degrees.to_radians().cos(), degrees.to_radians().sin(), 0.)
}

#[test]
fn a_seed_beyond_a_limit_comes_back_to_it() {
    let (mut assembly, id, _) = bounded_crank(120., bounds(0., 90.));

    let solution = assembly.solve().unwrap();

    assert!((angle_of(&solution, id) - 90.).abs() < 1e-6);
    assert_eq!(active(&solution), vec![Bound::Max(Coordinate::Angle)]);
    // Limits do not count as removed freedoms.
    assert_eq!(solution.report().mobility, 1);
}

#[test]
fn a_seed_within_its_limits_stays_and_holds_none() {
    let (mut assembly, id, _) = bounded_crank(45., bounds(0., 90.));

    let solution = assembly.solve().unwrap();

    assert!((angle_of(&solution, id) - 45.).abs() < 1e-6);
    assert!(active(&solution).is_empty());
}

#[test]
fn a_drag_stops_against_a_limit_and_leaves_it_when_pulled_back() {
    let (mut assembly, id, body) = bounded_crank(45., bounds(0., 90.));
    let tip = Point3::new(1., 0., 0.);

    let stopped = assembly
        .solve_toward(&[Target3d::point(body, tip, on_circle(150.))])
        .unwrap();

    assert!((angle_of(&stopped, id) - 90.).abs() < 1e-6);
    assert!(!stopped.report().targets[0].reached);
    assert_eq!(active(&stopped), vec![Bound::Max(Coordinate::Angle)]);

    let back = assembly
        .solve_toward(&[Target3d::point(body, tip, on_circle(30.))])
        .unwrap();

    assert!((angle_of(&back, id) - 30.).abs() < 1e-4);
    assert!(active(&back).is_empty());
}

#[test]
fn a_limit_past_a_turn_stops_a_crank_dragged_round() {
    let (mut assembly, id, body) = bounded_crank(0., bounds(-30., 400.));
    let tip = Point3::new(1., 0., 0.);
    let mut angle = 0.;
    for step in 1..=10 {
        let solution = assembly
            .solve_toward(&[Target3d::point(body, tip, on_circle(45. * step as f64))])
            .unwrap();
        angle = angle_of(&solution, id);
    }
    assert!((angle - 400.).abs() < 1e-6, "{angle}");
}

#[test]
fn a_slide_limit_bounds_a_screw_through_its_angle() {
    let (mut assembly, ground, body) = ground_and_body(Rigid::identity());
    let joint = Joint3d::new(
        JointKind::Helical { pitch: 1. },
        hold(ground, at(0., 0., 0.)),
        hold(body, at(0., 0., 0.)),
    )
    .with_limit(Coordinate::Slide, Limit::at_most(0.5));
    let id = assembly.add_joint(joint).unwrap();
    let tip = Point3::new(1., 0., 0.);
    let mut solution = None;
    for step in 1..=6 {
        let goal = on_circle(45. * step as f64) + Vector3::new(0., 0., 0.125 * step as f64);
        solution = Some(
            assembly
                .solve_toward(&[Target3d::point(body, tip, goal)])
                .unwrap(),
        );
    }
    let solution = solution.unwrap();
    assert!((solution.coordinate(id, Coordinate::Slide).unwrap() - 0.5).abs() < CLOSE);
    assert!((angle_of(&solution, id) - 180.).abs() < 1e-6);
    assert_eq!(active(&solution), vec![Bound::Max(Coordinate::Slide)]);
}

#[test]
fn a_prismatic_seed_past_its_stroke_comes_back_to_its_end() {
    let (mut assembly, ground, body) = ground_and_body(shift(0., 0., 3.));
    let joint = Joint3d::new(
        JointKind::Prismatic,
        hold(ground, at(0., 0., 0.)),
        hold(body, at(0., 0., 0.)),
    )
    .with_limit(Coordinate::Slide, Limit::between(0., 2.));
    let id = assembly.add_joint(joint).unwrap();

    let solution = assembly.solve().unwrap();

    assert!((solution.coordinate(id, Coordinate::Slide).unwrap() - 2.).abs() < CLOSE);
    assert_point(
        solution.pose(body).unwrap().apply(Point3::origin()),
        Point3::new(0., 0., 2.),
    );
}

#[test]
fn a_cone_limit_holds_a_ball_joint_within_its_angle() {
    let (mut assembly, ground, body) = ground_and_body(turn(Vector3::x(), 50.));
    let joint = Joint3d::new(
        JointKind::Spherical,
        hold(ground, at(0., 0., 0.)),
        hold(body, at(0., 0., 0.)),
    )
    .with_cone(30_f64.to_radians());
    let first = joint.first.clone();
    let second = joint.second.clone();
    assembly.add_joint(joint).unwrap();

    let solution = assembly.solve().unwrap();

    let (a, b) = (landed(&solution, &first), landed(&solution, &second));
    let tilt = degrees(a.z_dir.angle(&b.z_dir));
    assert!((tilt - 30.).abs() < 1e-6, "{tilt}");
    assert_eq!(active(&solution), vec![Bound::Cone]);
}

#[test]
fn a_loop_forcing_a_coordinate_past_its_limit_fails_naming_it() {
    let (mut assembly, ground, body) = ground_and_body(turn(Vector3::z(), 10.));
    let revolute = assembly
        .add_joint(
            Joint3d::new(
                JointKind::Revolute,
                hold(ground, at(0., 0., 0.)),
                hold(body, at(0., 0., 0.)),
            )
            .with_limit(Coordinate::Angle, bounds(-30., 30.)),
        )
        .unwrap();
    let welded = Frame::from_xz(
        Point3::origin(),
        Vector3::new(60_f64.to_radians().cos(), 60_f64.to_radians().sin(), 0.),
        Vector3::z(),
    );
    assembly
        .add_joint(Joint3d::new(
            JointKind::Fixed,
            hold(ground, welded),
            hold(body, at(0., 0., 0.)),
        ))
        .unwrap();

    let error = assembly.solve().unwrap_err();

    assert!(
        matches!(
            error,
            SolveError::Limit { joint, bound: Bound::Max(Coordinate::Angle) } if joint == revolute
        ),
        "{error:?}"
    );
}

#[test]
fn unsound_limits_are_refused() {
    let mut assembly = Assembly3d::default();
    let a = assembly.add_body(Rigid::identity()).unwrap();
    let b = assembly.add_body(Rigid::identity()).unwrap();
    let joint = |kind| Joint3d::new(kind, hold(a, at(0., 0., 0.)), hold(b, at(0., 0., 0.)));
    let refused = [
        joint(JointKind::Revolute).with_limit(Coordinate::Angle, Limit::between(1., 0.)),
        joint(JointKind::Revolute).with_limit(Coordinate::Slide, Limit::at_most(1.)),
        joint(JointKind::Revolute).with_limit(
            Coordinate::Angle,
            Limit {
                min: None,
                max: None,
            },
        ),
        joint(JointKind::Prismatic).with_limit(Coordinate::Slide, Limit::at_least(f64::NAN)),
        joint(JointKind::Revolute)
            .with_value(Coordinate::Angle, 2.)
            .with_limit(Coordinate::Angle, Limit::between(0., 1.)),
        joint(JointKind::Helical { pitch: 1. })
            .with_value(Coordinate::Angle, 10.)
            .with_limit(Coordinate::Slide, Limit::at_most(1.)),
        joint(JointKind::Revolute).with_cone(0.5),
        joint(JointKind::Spherical).with_cone(0.),
        joint(JointKind::Spherical).with_cone(4.),
    ];
    for joint in refused {
        assert!(
            matches!(
                assembly.add_joint(joint.clone()),
                Err(EditError::InvalidJoint(_))
            ),
            "{joint:?}"
        );
    }
    assert_eq!(assembly.joint_count(), 0);
}
