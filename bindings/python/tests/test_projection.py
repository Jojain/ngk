import math

import pytest

from ngk.geometry import Axis, Plane, Point, Vector
from ngk.modeling import edges, faces, profiles, sheets
from ngk.projection import (
    CurveProjection,
    MissError,
    PointProjection,
    ProjectionError,
    project,
    project_point,
)

DOWN = Vector(0.0, 0.0, -1.0)


def inclined_plane():
    return Plane(Point(0.0, 0.0, 1.0), Vector(1.0, 0.0, 0.0), Vector(0.0, -1.0, 1.0))


def circle_over(x, height):
    plane = Plane(Point(x, 1.0, height), Vector(1.0, 0.0, 0.0), Vector(0.0, 0.0, 1.0))
    return profiles.from_edge(edges.circle(plane, 0.8))


def distance_to(plane, point):
    return abs(plane.normal.x * (point.x - plane.origin.x)
               + plane.normal.y * (point.y - plane.origin.y)
               + plane.normal.z * (point.z - plane.origin.z))


def test_a_point_projects_onto_a_plane_along_the_perpendicular_and_along_a_direction():
    plane = inclined_plane()
    point = Point(1.0, 1.0, 5.0)

    nearest = project(point, plane)
    assert isinstance(nearest, PointProjection)
    assert nearest.point.as_tuple() == pytest.approx((1.0, 2.5, 3.5))
    assert nearest.distance == pytest.approx(3.0 / math.sqrt(2.0))
    assert nearest.hit is None

    shadow = project(point, plane, along=DOWN)
    assert shadow.point.as_tuple() == pytest.approx((1.0, 1.0, 2.0))
    assert shadow.distance == pytest.approx(3.0)

    assert project_point(point, plane, along=DOWN).as_tuple() == pytest.approx((1.0, 1.0, 2.0))


def test_a_ray_that_meets_nothing_is_a_miss_and_a_projection_error():
    with pytest.raises(MissError) as raised:
        project(Point(0.0, 0.0, 1.0), Plane.xy(), along=Vector(0.0, 0.0, 1.0))
    assert isinstance(raised.value, ProjectionError)
    assert isinstance(raised.value, ValueError)


def test_a_direction_for_a_curve_target_and_an_unsupported_pair_are_refused_clearly():
    edge = edges.line((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="only the perpendicular"):
        project(Point(0.5, 1.0, 0.0), edge, along=DOWN)
    with pytest.raises(TypeError, match="not supported"):
        project(circle_over(1.0, 1.0), edge)


def test_a_point_onto_an_edge_names_the_edge_it_landed_on():
    edge = edges.line((0.0, 0.0, 0.0), (2.0, 0.0, 0.0))
    nearest = project(Point(1.0, 3.0, 0.0), edge)
    assert nearest.point.as_tuple() == pytest.approx((1.0, 0.0, 0.0))
    assert nearest.hit == edge


def test_a_circle_profile_on_an_inclined_plane_is_one_closed_profile():
    plane = inclined_plane()
    for along in (None, DOWN):
        projection = project(circle_over(1.0, 5.0), plane, along=along)
        assert isinstance(projection, CurveProjection)
        assert len(projection.chains) == 1
        chain = projection.chains[0]
        assert chain.closed
        assert chain.start.kind == "closed" and chain.end.kind == "closed"
        assert len(chain.segments) == 1
        assert chain.segments[0].fit.exact
        assert chain.segments[0].source.edge is not None
        assert len(projection.curves()) == len(projection.pcurves()) == 1
        for step in range(8):
            image = projection.curves()[0].point_at(step / 8.0)
            assert distance_to(plane, image) < 1e-9

        drawn = profiles.from_projection(projection)
        assert len(drawn.profiles) == 1
        profile = drawn.profiles[0].profile
        assert profile.is_closed
        assert len(profile.edges()) == 1
        assert len(drawn.profiles[0].edges) == 1
        assert drawn.profiles[0].edges[0].length == pytest.approx(projection.curves()[0].length)


def test_a_circle_over_a_sheet_fold_is_one_closed_chain_of_two_segments():
    tilt = math.radians(20.0)
    across, height = 3.0 * math.cos(tilt), 3.0 * math.sin(tilt)
    flat = faces.polygon([(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (2.0, 2.0, 0.0), (0.0, 2.0, 0.0)])
    hinged = faces.polygon([
        (2.0, 2.0, 0.0),
        (2.0, 0.0, 0.0),
        (2.0 + across, 0.0, height),
        (2.0 + across, 2.0, height),
    ])
    sheet = sheets.from_faces([flat, hinged])
    sheet_faces = sheet.faces()
    assert len(sheet_faces) == 2

    projection = project(circle_over(2.0, 5.0), sheet_faces, along=DOWN)

    assert len(projection.chains) == 1
    chain = projection.chains[0]
    assert chain.closed
    assert len(chain.segments) == 2
    assert all(segment.hit in sheet_faces for segment in chain.segments)
    assert chain.segments[0].hit != chain.segments[1].hit

    drawn = profiles.from_projection(projection)
    assert len(drawn.profiles) == 1
    profile = drawn.profiles[0].profile
    assert profile.is_closed
    assert len(profile.edges()) == 2
    for vertex in profile.vertices():
        point = vertex.point
        assert point.x == pytest.approx(2.0, abs=1e-7)
        assert point.z == pytest.approx(0.0, abs=1e-7)


def test_a_plane_receives_a_point_like_the_free_function():
    plane = inclined_plane()
    point = Point(1.0, 1.0, 5.0)

    method = plane.project(point)
    free = project(point, plane)
    assert isinstance(method, PointProjection)
    assert method.point.as_tuple() == pytest.approx(free.point.as_tuple())
    assert method.distance == pytest.approx(free.distance)

    shadow = plane.project(point, along=DOWN)
    assert shadow.point.as_tuple() == pytest.approx((1.0, 1.0, 2.0))
    with pytest.raises(MissError):
        Plane.xy().project(Point(0.0, 0.0, 1.0), along=Vector(0.0, 0.0, 1.0))


def test_a_plane_receives_a_curve_as_a_curve_projection():
    plane = inclined_plane()
    projection = plane.project(circle_over(1.0, 5.0), along=DOWN)
    assert isinstance(projection, CurveProjection)
    assert len(projection.chains) == 1
    assert projection.chains[0].closed


def test_a_tolerance_reaches_the_target_method():
    with pytest.raises(ValueError, match="tolerance"):
        Plane.xy().project(Point(0.0, 0.0, 1.0), tolerance=-1.0)


def test_an_edge_and_a_curve_receive_a_point_perpendicularly():
    edge = edges.line((0.0, 0.0, 0.0), (2.0, 0.0, 0.0))
    nearest = edge.project(Point(1.0, 3.0, 0.0))
    assert nearest.point.as_tuple() == pytest.approx((1.0, 0.0, 0.0))
    assert nearest.hit == edge

    curve_nearest = edge.curve.project((1.0, 3.0, 0.0))
    assert curve_nearest.point.as_tuple() == pytest.approx((1.0, 0.0, 0.0))
    assert curve_nearest.hit is None

    with pytest.raises(TypeError):
        edge.project(Point(1.0, 3.0, 0.0), DOWN)
    with pytest.raises(TypeError, match="not supported"):
        edge.project(circle_over(1.0, 1.0))


def test_a_face_receives_a_point_and_a_curve():
    face = faces.polygon([(0.0, 0.0, 0.0), (4.0, 0.0, 0.0), (4.0, 4.0, 0.0), (0.0, 4.0, 0.0)])
    landed = face.project(Point(1.0, 1.0, 2.0))
    assert landed.point.as_tuple() == pytest.approx((1.0, 1.0, 0.0))
    assert landed.hit == face
    with pytest.raises(MissError):
        face.project(Point(9.0, 1.0, 2.0), along=DOWN)

    projection = face.project(circle_over(2.0, 3.0), along=DOWN)
    assert isinstance(projection, CurveProjection)
    assert projection.chains[0].segments[0].hit == face


def test_an_axis_receives_a_point_as_a_point_projection():
    axis = Axis(Point(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0))
    nearest = axis.project(Point(4.0, 3.0, 2.0))
    assert isinstance(nearest, PointProjection)
    assert nearest.point.as_tuple() == pytest.approx((1.0, 3.0, 0.0))
    assert nearest.distance == pytest.approx(math.sqrt(13.0))
    assert nearest.parameters == pytest.approx(3.0)
    assert nearest.hit is None

    free = project(Point(4.0, 3.0, 2.0), axis)
    assert free.point.as_tuple() == pytest.approx(nearest.point.as_tuple())
    with pytest.raises(ValueError, match="only the perpendicular"):
        project(Point(4.0, 3.0, 2.0), axis, along=DOWN)
