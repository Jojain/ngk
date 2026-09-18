import ngk
import ngk.modeling.edges
import ngk.modeling.faces
import ngk.modeling.profiles
import ngk.modeling.solids
import ngk.modeling.booleans
from ngk.geometry import Axis, Frame, Point, Point2, Vector
from ngk.topology import Solid


def test_geometry_and_modeling_modules_keep_domain_ownership():
    frame = Frame.from_xy(
        Point(10.0, 20.0, 30.0),
        Vector(1.0, 0.0, 0.0),
        Vector(0.0, 1.0, 0.0),
    )

    solid = ngk.modeling.solids.block(1.0, 2.0, 3.0, frame=frame)

    assert ngk.geometry.Frame is Frame
    assert ngk.geometry.Point is Point
    assert ngk.geometry.Vector is Vector
    assert ngk.geometry.Axis is Axis
    assert Point2(1.0, 2.0).as_tuple() == (1.0, 2.0)
    assert not hasattr(ngk, "block")
    assert solid.faces()[0].surface.origin.as_tuple() == (10.0, 20.0, 30.0)
    assert ngk.modeling.edges.line((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)).length == 1.0
    assert len(ngk.modeling.profiles.rectangle(2.0, 3.0).edges()) == 4
    assert len(ngk.modeling.faces.rectangle(2.0, 3.0).edges()) == 4
    tool = ngk.modeling.solids.block(
        1.0,
        1.0,
        1.0,
        frame=Frame.from_xy(
            Point(10.25, 20.25, 30.25),
            Vector(1.0, 0.0, 0.0),
            Vector(0.0, 1.0, 0.0),
        ),
    )
    assert isinstance(ngk.modeling.booleans.fuse(solid, tool), Solid)


def test_profile_from_edges_orders_connected_edges():
    first = ngk.modeling.edges.line((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    second = ngk.modeling.edges.line((1.0, 0.0, 0.0), (1.0, 1.0, 0.0))
    third = ngk.modeling.edges.line((1.0, 1.0, 0.0), (0.0, 1.0, 0.0))

    profile = ngk.modeling.profiles.from_edges([second, third, first])

    assert len(profile.edges()) == 3


def test_face_from_profile_builds_a_face_from_a_profile():
    profile = ngk.modeling.profiles.rectangle(2.0, 3.0)

    face = ngk.modeling.faces.from_profile(profile)

    assert len(face.edges()) == 4
