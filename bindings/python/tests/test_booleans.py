"""Booleans take one target and any number of tools, in a copy of the target's model."""

import itertools

import pytest

from ngk import measurement
from ngk.geometry import Frame, Plane, Point, Vector
from ngk.modeling import booleans, faces, solids


def at(x, y, z):
    return Frame.from_xz(Point(x, y, z), Vector(1, 0, 0), Vector(0, 0, 1))


def volume(solid):
    return measurement.solid_properties(solid).amount


def test_tools_bridged_only_by_a_later_tool_fuse_the_same_in_every_order():
    def operands():
        return [
            solids.block(10, 10, 10, at(0, 0, 0)),
            solids.block(10, 10, 10, at(30, 0, 0)),
            solids.block(30, 4, 4, at(5, 3, 3)),
        ]

    counts = set()
    for order in itertools.permutations(range(3)):
        shapes = operands()
        target, *tools = (shapes[index] for index in order)
        fused = booleans.fuse(target, tools)
        assert volume(fused) == pytest.approx(2320), order
        counts.add((fused.face_count, fused.edge_count, fused.vertex_count))
    assert len(counts) == 1


def test_a_single_tool_needs_no_list():
    target = solids.block(10, 10, 10)
    tool = solids.block(4, 4, 12, at(2, 2, -1))
    assert volume(booleans.cut(target, tool)) == pytest.approx(1000 - 160)


def test_no_tools_return_the_target_unchanged():
    target = solids.block(1, 2, 3)
    for operation in (booleans.fuse, booleans.cut, booleans.intersect):
        result = operation(target, [])
        assert volume(result) == pytest.approx(6)
        assert sorted(face.key for face in result.faces()) == sorted(
            face.key for face in target.faces()
        )


def test_a_face_boolean_keeps_the_other_faces_of_the_target_model():
    square = faces.rectangle(40, 30)
    triangle = faces.polygon([(-30, -20, 0), (-20, -20, 0), (-25, -12, 0)])
    both = booleans.fuse_faces(square, triangle)
    assert len(both.model.faces()) == 2
    triangle_key = next(face.key for face in both.model.faces() if len(face.vertices()) == 3)

    circle = faces.circle(5, Plane(Point(40, 15, 0), Vector(1, 0, 0), Vector(0, 0, 1)))
    square_in_both = next(face for face in both.model.faces() if len(face.vertices()) == 4)
    grown = booleans.fuse_faces(square_in_both, circle)

    assert len(grown.model.faces()) == 2
    assert triangle_key in {face.key for face in grown.model.faces()}
    assert len(square_in_both.model.faces()) == 2, "the input model is not mutated"


def test_three_disjoint_faces_gather_in_one_model_by_successive_fuses():
    def square(x):
        return faces.rectangle(1, 1, Plane(Point(x, 0, 0), Vector(1, 0, 0), Vector(0, 0, 1)))

    first = booleans.fuse_faces(square(0), square(5))
    anchor = first.faces()[0]
    gathered = booleans.fuse_faces(anchor, square(10))
    assert len(gathered.model.faces()) == 3


def test_a_face_an_earlier_tool_left_keeps_its_key_through_a_later_cut():
    def hole(x):
        return solids.cylinder(3, 12, at(x, 10, -1))

    def wall(solid):
        return next(
            face.key
            for face in solid.faces()
            if type(face.surface).__name__ == "Cylinder"
            and face.surface.origin.as_tuple()[0] < 20
        )

    one_hole = booleans.cut(solids.block(40, 20, 10), hole(10))
    first_wall = wall(one_hole)
    two_holes = booleans.cut(one_hole, hole(30))
    assert wall(two_holes) == first_wall
    assert len(one_hole.faces()) == 7, "the input model is not mutated"


def test_a_tool_in_the_target_model_is_used_in_place():
    square = faces.rectangle(4, 4)
    other = faces.rectangle(4, 4, Plane(Point(10, 0, 0), Vector(1, 0, 0), Vector(0, 0, 1)))
    both = booleans.fuse_faces(square, other)
    first, second = both.faces()
    # The second face is in the first's model: fusing it in again is a union
    # of two disjoint faces already there, so nothing is copied and nothing
    # changes.
    again = booleans.fuse_faces(first, second)
    assert len(again.model.faces()) == 2


def test_a_face_boolean_with_no_area_left_returns_no_faces():
    def xy(x, y):
        return Plane(Point(x, y, 0), Vector(1, 0, 0), Vector(0, 0, 1))

    small = faces.rectangle(10, 10)
    large = faces.rectangle(100, 100, xy(-50, -50))
    far = faces.rectangle(10, 10, xy(50, 0))
    assert booleans.cut_faces(small, large).faces() == []
    assert booleans.intersect_faces(small, far).faces() == []


def test_a_face_boolean_of_faces_on_different_planes_still_raises():
    tilted = faces.rectangle(10, 10, Plane(Point(0, 0, 0), Vector(1, 0, 0), Vector(0, 1, 0)))
    with pytest.raises(ValueError):
        booleans.cut_faces(faces.rectangle(10, 10), tilted)
