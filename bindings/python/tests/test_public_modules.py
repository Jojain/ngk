import ngk
import ngk.geometry
import ngk.modeling.edges
import ngk.modeling.faces
import ngk.modeling.profiles
import ngk.modeling.solids


def test_geometry_and_modeling_modules_keep_root_convenience_imports():
    frame = ngk.geometry.Frame.from_xy(
        ngk.geometry.Point(10.0, 20.0, 30.0),
        ngk.geometry.Vector(1.0, 0.0, 0.0),
        ngk.geometry.Vector(0.0, 1.0, 0.0),
    )

    solid = ngk.modeling.solids.block(1.0, 2.0, 3.0, frame=frame)

    assert ngk.Frame is ngk.geometry.Frame
    assert ngk.Point is ngk.geometry.Point
    assert ngk.Vector is ngk.geometry.Vector
    assert ngk.Axis is ngk.geometry.Axis
    assert not hasattr(ngk.geometry, "Point3")
    assert ngk.geometry.Point2(1.0, 2.0).as_tuple() == (1.0, 2.0)
    assert ngk.block is ngk.modeling.solids.block
    assert solid.faces()[0].surface.origin.as_tuple() == (10.0, 20.0, 30.0)
    assert ngk.modeling.edges.line((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)).length == 1.0
    assert len(ngk.modeling.profiles.rectangle(2.0, 3.0).edges()) == 4
    assert len(ngk.modeling.faces.rectangle(2.0, 3.0).edges()) == 4
