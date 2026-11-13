use nalgebra::Vector3;
use ngk::geometry::Axis3;
use ngk::geometry::dim3::frame::Frame;
use ngk::geometry::dim3::utils::Point3;
use ngk::geometry::transform::Rigid;
use ngk::solver::assembly::{
    Assembly3d, Attachment, BodyId, EditError, Joint3d, JointKind, Solution3d, SolveError, Target3d,
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
    let joint = Joint3d::revolute(hold(ground, at(1., 2., 3.)), hold(body, at(0., 0., 1.)));
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
    let joint = Joint3d::prismatic(hold(ground, at(0., 0., 0.)), hold(body, at(0.5, 0., 0.)));
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
    let joint = Joint3d::fixed(hold(ground, first), hold(body, at(0., 0., 2.)));
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
    let joint = Joint3d::revolute(hold(ground, at(0., 0., 0.)), hold(body, at(0., 0., 0.)));
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
    let joint = Joint3d::revolute(hold(ground, at(0., 0., 0.)), hold(body, at(0., 0., 0.)));
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
        .add_joint(Joint3d::revolute(
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
        Joint3d::revolute(hold(ground, pin(a0)), hold(crank, pin(a0))),
        Joint3d::revolute(hold(crank, pin(a)), hold(coupler, pin(a))),
        Joint3d::revolute(hold(coupler, pin(b)), hold(rocker, pin(b))),
        Joint3d::revolute(hold(rocker, pin(b0)), hold(ground, pin(b0))),
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
        .add_joint(Joint3d::revolute(
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
        &[Joint3d::revolute(
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
        .add_joint(Joint3d::fixed(
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
        .add_joint(Joint3d::fixed(
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
    let joint = Joint3d::revolute(hold(ground, at(1., 0., 0.)), hold(body, at(0., 0., 0.)));
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
    let same = Joint3d::revolute(hold(a, at(0., 0., 0.)), hold(a, at(1., 0., 0.)));
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
    let missing = Joint3d::fixed(
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
        .add_joint(Joint3d::fixed(
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
        assembly.add_joint(Joint3d::fixed(
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
