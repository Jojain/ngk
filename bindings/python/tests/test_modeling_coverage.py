"""Public modeling operations return inspectable owned shapes."""

import pytest

from ngk.geometry import Axis, Plane, Point, Rigid, Vector
from ngk.modeling import blend, revolve, sweep, transform
from ngk.modeling import edges, faces, profiles
from ngk.topology import Sheet, Solid


def test_revolve_profile_and_face():
    axis = Axis(Point(0, 0, 0), Vector(0, 0, 1))
    plane = Plane(Point(2, 0, 0), Vector(1, 0, 0), Vector(0, 1, 0))
    profile = profiles.rectangle(2, 1, plane)
    face = faces.rectangle(2, 1, plane)

    assert isinstance(revolve.profile(profile, axis, 1.0), Sheet)
    assert isinstance(revolve.face(face, axis, 1.0), Solid)


def test_extrude_profile_and_sweep_face():
    profile = profiles.rectangle(2, 1)
    face = faces.rectangle(2, 1)
    spine = edges.helix(Axis(Point(0, 0, 0), Vector(0, 0, 1)), 2, 1, 0, 1)

    assert isinstance(sweep.extrude_profile(profile, Vector(0, 0, 2)), Sheet)
    assert isinstance(sweep.extrude_face(face, Vector(0, 0, 2)), Solid)
    options = sweep.SweepOptions(frame="axial", axis=Axis(Point(0, 0, 0), Vector(0, 0, 1)))
    assert isinstance(sweep.face_along_edge(face, spine, options), Solid)

    section = faces.rectangle(
        1, 1, Plane(Point(1, 0, 0), Vector(1, 0, 0), Vector(0, 0, 1))
    )
    path = profiles.polyline([(0, 0, 0), (0, 0, 4), (4, 0, 4)])
    assert isinstance(
        sweep.face_along_profile(section, path, sweep.SweepOptions(transition="straight")),
        Solid,
    )


def test_rigid_motion_preserves_shape_kind_and_source():
    source = faces.rectangle(2, 1)
    motion = Rigid.translation(Vector(0, 0, 3))
    moved = transform.moved_face(source, motion)

    assert moved.surface.origin.as_tuple() == (0, 0, 3)
    assert source.surface.origin.as_tuple() == (0, 0, 0)
    assert motion.inverse().apply(Point(0, 0, 3)).as_tuple() == (0, 0, 0)


def test_blend_target_uses_entities_from_one_model():
    from ngk.modeling import solids

    source = solids.block(2, 2, 2)
    target = blend.BlendTarget()
    target.add_edge(source.edges()[0])
    rounded = blend.filleted_solid(source, target, 0.1)

    assert rounded.face_count > source.face_count
    assert source.face_count == 6


def test_polygon_with_holes_exposes_both_boundary_components():
    face = faces.polygon_with_holes(
        [(0, 0, 0), (4, 0, 0), (4, 4, 0), (0, 4, 0)],
        [[(1, 1, 0), (1, 2, 0), (2, 2, 0), (2, 1, 0)]],
    )
    assert len(face.loops()) == 2


def test_profile_append_returns_extended_owned_profile():
    first = edges.line((0, 0, 0), (1, 0, 0))
    second = edges.line((1, 0, 0), (1, 1, 0))
    profile = profiles.from_edge(first)

    extended = profiles.appended(profile, second)

    assert len(profile.edges()) == 1
    assert len(extended.edges()) == 2


def test_extruded_rejects_non_finite_direction():
    face = faces.rectangle(1, 1)
    from ngk.modeling import solids

    with pytest.raises(ValueError, match="finite coordinates"):
        solids.extruded(face, Vector(float("nan"), 0, 0), 1)


def test_face_boolean_returns_one_model_and_result_faces():
    from ngk.modeling import booleans

    first = faces.square(2)
    second = faces.square(
        2, Plane(Point(1, 1, 0), Vector(1, 0, 0), Vector(0, 0, 1))
    )
    result = booleans.intersect_faces(first, second)

    assert len(result.faces()) == 1
    assert result.faces()[0].model == result.model


def test_edge_split_exposes_one_model_and_all_handles():
    edge = edges.line((0, 0, 0), (2, 0, 0))
    result = edges.split(edge, 0.5)

    assert result.separated
    assert len(result.edges()) == 2
    assert result.vertex().model == result.model


def test_measurements_expose_centroid_and_inertia():
    from ngk import measurement
    from ngk.modeling import solids

    face = faces.rectangle(2, 3)
    solid = solids.block(2, 3, 4)

    area = measurement.face_properties(face)
    volume = measurement.solid_properties(solid)
    assert area.amount == pytest.approx(6)
    assert volume.amount == pytest.approx(24)
    assert len(volume.inertia) == 9
    assert volume.centroid.as_tuple() == pytest.approx((1, 1.5, 2))


def test_face_boundary_split_updates_the_face_in_one_model():
    face = faces.rectangle(2, 2)
    result = faces.split_boundary_edge(face, face.edges()[0], 0.5)

    assert result.separated
    assert len(result.edges()) == 2
    assert result.face().model == result.model
    assert len(result.face().edges()) == 5


def test_healing_returns_solid_and_report():
    from ngk.modeling import heal, solids

    source = solids.block(2, 3, 4)
    result = heal.solid(source)

    assert isinstance(result.solid, Solid)
    assert result.report.iterations >= 1
