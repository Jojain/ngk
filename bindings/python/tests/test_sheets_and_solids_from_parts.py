"""Building each shape from the one below it: sheets from faces, solids from sheets."""

import math

import pytest

from ngk.geometry import Frame, Point, Vector
from ngk.modeling import faces, sheets, solids
from ngk.topology import Edge, Face, Profile, Shape, Sheet, Solid, Vertex


def test_from_faces_rebuilds_a_block_shell_as_a_sheet():
    block = solids.block(1.0, 2.0, 3.0)

    sheet = sheets.from_faces(block.faces())

    assert isinstance(sheet, Sheet)
    assert len(sheet.faces()) == 6
    assert math.isclose(sheet.area, 22.0)
    assert block.face_count == 6


def test_from_faces_makes_one_face_a_sheet():
    sheet = sheets.from_faces([faces.square(1.0)])

    assert len(sheet.faces()) == 1


def test_from_faces_refuses_disconnected_faces():
    near = faces.square(1.0)
    far = faces.polygon([(5.0, 0.0, 0.0), (6.0, 0.0, 0.0), (6.0, 1.0, 0.0)])

    with pytest.raises(ValueError, match="groups"):
        sheets.from_faces([near, far])


def test_from_sheets_rebuilds_a_solid_from_its_faces():
    cylinder = solids.cylinder(1.0, 2.0)

    solid = solids.from_sheets(sheets.from_faces(cylinder.faces()))

    assert isinstance(solid, Solid)
    assert math.isclose(solid.volume, 2.0 * math.pi, rel_tol=1e-6)


def test_from_sheets_hollows_a_solid_with_a_void():
    outer = sheets.from_faces(solids.block(4.0, 4.0, 4.0).faces())
    centred = solids.sphere(
        1.0,
        frame=Frame.from_xy(Point(2.0, 2.0, 2.0), Vector(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0)),
    )
    void = sheets.from_faces(centred.faces())

    solid = solids.from_sheets(outer, [void])

    assert math.isclose(solid.volume, 64.0 - 4.0 / 3.0 * math.pi, rel_tol=1e-6)


def test_from_sheets_refuses_an_open_sheet():
    with pytest.raises(ValueError, match="open"):
        solids.from_sheets(sheets.from_faces([faces.square(1.0)]))


def test_shape_names_every_topology_kind():
    assert set(Shape.__value__.__args__) == {Vertex, Edge, Profile, Face, Sheet, Solid}
