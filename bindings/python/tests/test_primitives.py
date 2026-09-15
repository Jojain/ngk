import pytest

import ngk


def test_cylinder_exposes_its_placement():
    solid = ngk.cylinder(2.0, 5.0)

    assert solid.face_count == 3
    assert solid.edge_count == 2


def test_cylinder_at_places_the_frame_origin():
    frame = ngk.Frame.from_xy(
        ngk.Point(1.0, 2.0, 3.0),
        ngk.Vector(1.0, 0.0, 0.0),
        ngk.Vector(0.0, 1.0, 0.0),
    )
    solid = ngk.cylinder(1.0, 2.0, frame=frame)

    assert solid.faces()[0].surface.origin.as_tuple() == (1.0, 2.0, 3.0)


def test_sphere_and_torus_are_closed_solids():
    sphere = ngk.sphere(2.0)
    torus = ngk.torus(3.0, 1.0)

    assert sphere.face_count == 1
    assert torus.face_count == 1
    assert sphere.outer_shell == sphere.shells()[0]


def test_invalid_sizes_are_rejected():
    with pytest.raises(ValueError):
        ngk.cylinder(0.0, 1.0)
    with pytest.raises(ValueError):
        ngk.sphere(-1.0)
    with pytest.raises(ValueError):
        ngk.torus(1.0, 0.0)
