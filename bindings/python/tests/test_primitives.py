import pytest

from ngk.geometry import Frame, Point, Vector
from ngk.modeling import solids


def test_cylinder_exposes_its_placement():
    solid = solids.cylinder(2.0, 5.0)

    assert solid.face_count == 3
    assert solid.edge_count == 2


def test_cylinder_at_places_the_frame_origin():
    frame = Frame.from_xy(
        Point(1.0, 2.0, 3.0),
        Vector(1.0, 0.0, 0.0),
        Vector(0.0, 1.0, 0.0),
    )
    solid = solids.cylinder(1.0, 2.0, frame=frame)

    assert solid.faces()[0].surface.origin.as_tuple() == (1.0, 2.0, 3.0)


def test_sphere_and_torus_are_closed_solids():
    sphere = solids.sphere(2.0)
    torus = solids.torus(3.0, 1.0)

    assert sphere.face_count == 1
    assert torus.face_count == 1
    assert sphere.outer_shell == sphere.shells()[0]


def test_invalid_sizes_are_rejected():
    with pytest.raises(ValueError):
        solids.cylinder(0.0, 1.0)
    with pytest.raises(ValueError):
        solids.sphere(-1.0)
    with pytest.raises(ValueError):
        solids.torus(1.0, 0.0)
