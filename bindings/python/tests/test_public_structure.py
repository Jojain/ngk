"""The Python package follows NGK's domain structure rather than root aliases."""

import ngk
from ngk.geometry import Frame, Point, Vector
from ngk.model import Model
from ngk.modeling import booleans, solids
from ngk.topology import Solid


def test_structured_modules_own_construction_and_boolean_operations():
    first = solids.block(
        2.0,
        2.0,
        2.0,
        frame=Frame.from_xy(
            Point(0.0, 0.0, 0.0),
            Vector(1.0, 0.0, 0.0),
            Vector(0.0, 1.0, 0.0),
        ),
    )
    second = solids.block(1.0, 1.0, 1.0)

    result = booleans.fuse(first, second)

    assert isinstance(result, Solid)
    assert isinstance(result.model, Model)
    assert result.face_count == 6
    assert not hasattr(ngk, "block")
    assert not hasattr(ngk, "fuse")
