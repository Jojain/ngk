import pytest

from ngk.geometry import Plane, Point2, Vector, Vector2
from ngk.modeling import faces, profiles, sweep
from ngk.solver import (
    Arc2,
    Circle2,
    CircleId,
    ConflictError,
    NonConvergenceError,
    PointId,
    PointTarget,
    Segment2,
    SegmentId,
    SegmentTarget,
    System2d,
)
from ngk.solver.constraints import (
    Angle,
    Coincident,
    Constraint,
    Distance,
    FixedPoint,
    Horizontal,
    Length,
    Radius,
    TangencyMode,
    TangentCircles,
    Vertical,
)


def segment(a, b):
    return Segment2(Point2(*a), Point2(*b))


def add_square(system):
    bottom = system.add_segment(segment((0.0, 0.0), (9.0, 0.0)))
    right = system.add_segment(segment((9.2, 0.1), (9.4, 9.0)))
    top = system.add_segment(segment((9.1, 9.2), (0.2, 9.0)))
    left = system.add_segment(segment((-0.2, 9.1), (0.1, 0.1)))
    for constraint in [
        Coincident(bottom.end(), right.start()),
        Coincident(right.end(), top.start()),
        Coincident(top.end(), left.start()),
        Coincident(left.end(), bottom.start()),
        FixedPoint(bottom.start()),
        Horizontal(bottom),
        Horizontal(top),
        Vertical(right),
        Vertical(left),
        Length(bottom, 10.0),
        Length(left, 10.0),
    ]:
        system.add_constraint(constraint)
    return bottom, right, top, left


def solved_square():
    system = System2d()
    ids = add_square(system)
    system.solve()
    return system, ids


def ends(edge):
    return sorted(tuple(round(c, 6) for c in vertex.point.as_tuple()) for vertex in edge.vertices())


def test_ids_are_typed_values():
    assert SegmentId(1) == SegmentId(1)
    assert SegmentId(1) != CircleId(1)
    assert len({SegmentId(1), SegmentId(1), CircleId(1)}) == 2
    assert SegmentId(7).value == 7
    assert repr(SegmentId(7)) == "SegmentId(7)"


def test_add_issues_ids_and_insert_takes_them_back():
    system = System2d()
    first = system.add_point(Point2(0.0, 0.0))
    assert isinstance(first, PointId)
    system.remove_geometry(first)
    assert system.add_point(Point2(0.0, 0.0)) != first
    system.insert_segment(SegmentId(50), segment((0.0, 0.0), (1.0, 0.0)))
    assert system.segment(SegmentId(50)).length == 1.0
    assert system.add_segment(segment((2.0, 0.0), (3.0, 0.0))).value > 50
    with pytest.raises(ValueError):
        system.insert_segment(SegmentId(50), segment((0.0, 0.0), (1.0, 0.0)))


def test_a_solution_is_read_by_ids():
    system, (bottom, right, _, _) = solved_square()
    solution = system.solve()
    assert solution.segment(right).start.as_tuple() == pytest.approx((10.0, 0.0))
    assert solution.segment(right).end.as_tuple() == pytest.approx((10.0, 10.0))
    assert solution.point(bottom.end()).as_tuple() == pytest.approx((10.0, 0.0))
    assert solution.report.degrees_of_freedom == 0
    assert solution.report.freedom(right) == 0
    assert solution.report.freedoms[right] == 0


def test_constraints_are_classes_deriving_from_constraint():
    system = System2d()
    line = system.add_segment(segment((0.0, 0.0), (1.0, 0.0)))
    relation = Length(line, 2.0)
    assert isinstance(relation, Constraint)
    assert relation.line == line
    assert relation.value == 2.0
    assert relation == Length(line, 2.0)
    assert relation != Length(line, 3.0)
    assert repr(relation).startswith("Length(SegmentId(")
    width = system.add_constraint(relation)
    match system.constraint(width):
        case Length(found, value):
            assert found == line
            assert value == 2.0
        case other:
            pytest.fail(f"read back {other!r}")


def test_a_circular_field_keeps_what_it_was_given():
    system = System2d()
    circle = system.add_circle(Circle2(Point2(0.0, 0.0), 2.0))
    arc = system.add_arc(Arc2(Point2(5.0, 0.0), Vector2(1.0, 0.0), 2.0, 1.0))
    assert Radius(circle, 3.0).circle == circle
    assert Radius(arc, 3.0).circle == arc
    read = system.constraint(system.add_constraint(Radius(arc, 3.0)))
    assert read.circle == arc


def test_defaults_of_angle_and_tangency():
    system = System2d()
    a = system.add_segment(segment((0.0, 0.0), (1.0, 0.0)))
    b = system.add_segment(segment((0.0, 0.0), (0.0, 1.0)))
    assert Angle(a, b, 1.0).oriented is False
    c = system.add_circle(Circle2(Point2(0.0, 0.0), 1.0))
    d = system.add_circle(Circle2(Point2(5.0, 0.0), 1.0))
    assert TangentCircles(c, d).mode == TangencyMode.Automatic


def test_a_wrong_kind_is_refused():
    system = System2d()
    circle = system.add_circle(Circle2(Point2(0.0, 0.0), 2.0))
    with pytest.raises(TypeError):
        Length(circle, 1.0)


def test_a_drag_keeps_the_constraints():
    system = System2d()
    line = system.add_segment(segment((0.0, 0.0), (10.0, 0.0)))
    system.add_constraint(FixedPoint(line.start()))
    system.add_constraint(Length(line, 10.0))
    solution = system.solve_toward([PointTarget(line.end(), Point2(0.0, 20.0))])
    assert solution.segment(line).end.as_tuple() == pytest.approx((0.0, 10.0), abs=1e-3)
    assert solution.report.targets[0].reached is False

    moved = system.solve_toward([SegmentTarget(line, 0.5, Point2(3.0, 4.0))])
    assert moved.segment(line).length == pytest.approx(10.0)


def test_failures_name_the_constraints():
    system = System2d()
    line = system.add_segment(segment((0.0, 0.0), (7.0, 2.0)))
    ten = system.add_constraint(Length(line, 10.0))
    twenty = system.add_constraint(Length(line, 20.0))
    with pytest.raises(ConflictError) as raised:
        system.solve()
    assert set(raised.value.constraints) == {ten, twenty}
    assert isinstance(raised.value, ValueError)

    system = System2d()
    a = system.add_point(Point2(0.0, 0.0))
    b = system.add_point(Point2(1.0, 0.0))
    c = system.add_point(Point2(5.0, 5.0))
    system.add_constraint(FixedPoint(a))
    system.add_constraint(FixedPoint(b))
    near = system.add_constraint(Distance(a, c, 1.0))
    far = system.add_constraint(Distance(b, c, 9.0))
    with pytest.raises(NonConvergenceError) as raised:
        system.solve()
    assert near in raised.value.constraints
    assert far in {residual.constraint for residual in raised.value.residuals}


def test_a_system_goes_through_json():
    system, (_, right, _, _) = solved_square()
    restored = System2d.from_json(system.to_json())
    assert restored.constraint_count == 11
    solution = restored.solve()
    assert solution.report.iterations == 0
    assert solution.segment(right).end.as_tuple() == pytest.approx((10.0, 10.0))
    with pytest.raises(ValueError):
        System2d.from_json('{"not": "a system"}')


def test_from_system_draws_a_closed_profile():
    system, ids = solved_square()
    built = profiles.from_system(system, Plane.xy(), list(ids))
    assert len(built.profiles) == 1
    run = built.profiles[0]
    assert run.profile.is_closed
    assert run.curves == list(ids)
    assert ends(built.edges[ids[1]]) == [(10.0, 0.0, 0.0), (10.0, 10.0, 0.0)]


def test_from_system_makes_a_face_and_a_solid():
    system, ids = solved_square()
    built = profiles.from_system(system, Plane.xy(), list(ids))
    face = faces.from_profile(built.profiles[0].profile)
    solid = sweep.extrude_face(face, Vector(0.0, 0.0, 5.0))
    assert abs(solid.volume) == pytest.approx(500.0)


def test_the_edge_keys_of_a_lone_profile_survive_making_its_face():
    system, ids = solved_square()
    built = profiles.from_system(system, Plane.xy(), list(ids))
    expected = {built.edges[curve].key: ends(built.edges[curve]) for curve in ids}
    face = faces.from_profile(built.profiles[0].profile)
    assert {edge.key: ends(edge) for edge in face.edges()} == expected


def test_separate_runs_are_separate_profiles():
    system = System2d()
    first = add_square(system)
    corners = [(20.0, 0.0), (30.0, 0.0), (30.0, 10.0), (20.0, 10.0)]
    second = [system.add_segment(segment(corners[i], corners[(i + 1) % 4])) for i in range(4)]
    for i in range(4):
        system.add_constraint(Coincident(second[i].end(), second[(i + 1) % 4].start()))
    system.solve()

    built = profiles.from_system(system, Plane.xy(), [*first, *second])
    assert [run.curves for run in built.profiles] == [list(first), second]
    assert all(run.profile.is_closed for run in built.profiles)


def test_refusals_name_the_curves():
    system = System2d()
    a = system.add_segment(segment((0.0, 0.0), (2.0, 2.0)))
    b = system.add_segment(segment((0.0, 2.0), (2.0, 0.0)))
    system.solve()
    with pytest.raises(profiles.CrossingError) as raised:
        profiles.from_system(system, Plane.xy(), [a, b])
    assert raised.value.first == a
    assert raised.value.second == b
    assert raised.value.point.as_tuple() == pytest.approx((1.0, 1.0))
    assert isinstance(raised.value, ValueError)

    unsolved = System2d()
    line = unsolved.add_segment(segment((0.0, 0.0), (1.0, 0.0)))
    with pytest.raises(profiles.NotSolvedError):
        profiles.from_system(unsolved, Plane.xy(), [line])
