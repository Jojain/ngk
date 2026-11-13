import math

import pytest

from ngk.geometry import Axis, Frame, Point, Rigid, Vector
from ngk.solver.assembly import (
    Assembly3d,
    Attachment,
    Coordinate,
    Joint,
    JointKind,
    Limit,
    LimitError,
    NonConvergenceError,
    Target,
)


def at(x, y, z):
    return Frame.from_xy(Point(x, y, z), Vector(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0))


def turn(degrees):
    return Rigid.rotation(Axis(Point(0.0, 0.0, 0.0), Vector(0.0, 0.0, 1.0)), math.radians(degrees))


def crank(limits=None):
    assembly = Assembly3d()
    ground = assembly.add_body(Rigid.identity())
    assembly.fix(ground)
    body = assembly.add_body(turn(20.0))
    joint = assembly.add_joint(
        Joint(
            JointKind.Revolute,
            Attachment(ground, at(0.0, 0.0, 0.0)),
            Attachment(body, at(0.0, 0.0, 0.0)),
            limits=limits,
        )
    )
    return assembly, ground, body, joint


def test_a_revolute_joint_reports_its_angle_and_one_mobility():
    assembly, ground, body, joint = crank()

    solution = assembly.solve()

    assert math.degrees(solution.coordinate(joint, Coordinate.Angle)) == pytest.approx(20.0)
    assert solution.report.mobility == 1
    assert solution.report.freedoms[body] == 1
    assert solution.report.freedoms[ground] == 0


def test_dragging_stops_against_a_limit():
    limits = {Coordinate.Angle: Limit(max=math.radians(90.0))}
    assembly, _, body, joint = crank(limits)
    goal = Point(math.cos(math.radians(150.0)), math.sin(math.radians(150.0)), 0.0)

    solution = assembly.solve_toward([Target(body, Point(1.0, 0.0, 0.0), goal)])

    assert math.degrees(solution.coordinate(joint, Coordinate.Angle)) == pytest.approx(90.0)
    [limit] = solution.report.active_limits
    assert (limit.joint, limit.side, limit.coordinate) == (joint, "max", Coordinate.Angle)
    assert not solution.report.targets[0].reached


def test_a_helical_joint_takes_a_pitch_and_an_imposed_angle():
    assembly = Assembly3d()
    ground = assembly.add_body(Rigid.identity())
    assembly.fix(ground)
    body = assembly.add_body(Rigid.identity())
    joint = assembly.add_joint(
        Joint(
            JointKind.Helical,
            Attachment(ground, at(0.0, 0.0, 0.0)),
            Attachment(body, at(0.0, 0.0, 0.0)),
            pitch=2.0,
            values={Coordinate.Angle: math.radians(540.0)},
        )
    )

    solution = assembly.solve()

    assert solution.coordinate(joint, Coordinate.Slide) == pytest.approx(3.0)
    assert assembly.joint(joint).pitch == 2.0
    with pytest.raises(ValueError):
        Joint(
            JointKind.Revolute,
            Attachment(ground, at(0.0, 0.0, 0.0)),
            Attachment(body, at(0.0, 0.0, 0.0)),
            pitch=1.0,
        )


def test_failures_name_what_is_at_fault():
    assembly, ground, body, joint = crank({Coordinate.Angle: Limit(-0.5, 0.5)})
    welded = Frame.from_xz(Point(0.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0), Vector(0.0, 0.0, 1.0))
    assembly.add_joint(
        Joint(JointKind.Fixed, Attachment(ground, welded), Attachment(body, at(0.0, 0.0, 0.0)))
    )
    with pytest.raises(LimitError) as raised:
        assembly.solve()
    assert raised.value.limit.joint == joint

    clash = Assembly3d()
    a = clash.add_body(Rigid.identity())
    b = clash.add_body(Rigid.identity())
    clash.fix(a)
    clash.fix(b)
    weld = clash.add_joint(
        Joint(JointKind.Fixed, Attachment(a, at(1.0, 0.0, 0.0)), Attachment(b, at(0.0, 0.0, 0.0)))
    )
    with pytest.raises(NonConvergenceError) as raised:
        clash.solve()
    assert raised.value.joints == [weld]


def test_an_assembly_round_trips_through_json():
    assembly, _, body, joint = crank()
    assembly.set_coordinate_seed(joint, Coordinate.Angle, math.radians(370.0))

    restored = Assembly3d.from_json(assembly.to_json())

    assert restored.body_count == 2
    assert restored.coordinate_seed(joint, Coordinate.Angle) == pytest.approx(math.radians(370.0))
    solved = restored.solve()
    assert math.degrees(solved.coordinate(joint, Coordinate.Angle)) == pytest.approx(380.0)
    assert solved.pose(body).apply(Point(1.0, 0.0, 0.0)).x == pytest.approx(
        math.cos(math.radians(20.0))
    )
