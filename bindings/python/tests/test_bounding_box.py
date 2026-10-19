"""Bounding boxes are measured on the entities themselves."""

import math

import pytest

from ngk.geometry import Frame, Point, Vector
from ngk.measurement import Bound, BoundingBox
from ngk.modeling import edges, solids


def oblique_frame():
    return Frame.from_xz(Point(1, 2, 3), Vector(1, 1, 0), Vector(-1, 1, 1))


def test_a_block_in_the_world_gives_back_its_corners_exactly():
    box = solids.block(2, 3, 4).bounding_box()

    assert isinstance(box, BoundingBox)
    assert box.min.as_tuple() == (0, 0, 0)
    assert box.max.as_tuple() == (2, 3, 4)
    assert box.size.as_tuple() == (2, 3, 4)
    assert box.center.as_tuple() == (1, 1.5, 2)
    assert box.corner(Bound.Max, Bound.Min, Bound.Max).as_tuple() == (2, 0, 4)
    assert len(box.corners()) == 8


def test_a_block_is_tight_in_its_own_frame():
    frame = oblique_frame()
    block = solids.block(2, 3, 4, frame)

    box = block.bounding_box(frame)

    assert (box.x_size, box.y_size, box.z_size) == pytest.approx((2, 3, 4), abs=1e-12)


def test_an_oblique_block_is_its_own_oriented_box():
    box = solids.block(2, 3, 4, oblique_frame()).oriented_bounding_box()

    assert box.size.as_tuple() == pytest.approx((4, 3, 2), abs=1e-12)
    assert box.diagonal_length == pytest.approx(math.sqrt(29), abs=1e-12)


def test_the_world_box_of_a_cylinder_reaches_its_radius():
    box = solids.cylinder(4, 10).bounding_box()

    assert box.min.as_tuple() == pytest.approx((-4, -4, 0), abs=1e-12)
    assert box.max.as_tuple() == pytest.approx((4, 4, 10), abs=1e-12)


def test_an_oblique_segment_is_boxed_along_itself():
    box = edges.line((0, 0, 0), (1, 2, 2)).oriented_bounding_box()

    assert box.size.as_tuple() == pytest.approx((3, 0, 0), abs=1e-12)
    assert box.center.as_tuple() == pytest.approx((0.5, 1, 1), abs=1e-12)
