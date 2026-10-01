import pytest

from ngk import geometry as g
from ngk.measurement import face_properties
from ngk.modeling import booleans, edges, faces, profiles, solids
from ngk.tessellation import Range, Tessellation, tessellate


def holed_block():
    frame = g.Frame.from_xz(g.Point(10, 10, -1), g.Vector(1, 0, 0), g.Vector(0, 0, 1))
    return booleans.cut(solids.block(30, 20, 10), solids.cylinder(4, 12, frame))


def assert_covers(ranges, total):
    start = 0
    for item in ranges:
        assert isinstance(item, Range)
        assert item.start == start
        start += item.count
    assert start == total


def test_a_solid_tessellates_into_ranges_keyed_like_its_cells():
    solid = holed_block()
    mesh = tessellate(solid)

    assert isinstance(mesh, Tessellation)
    assert [r.key for r in mesh.faces] == [f.key for f in solid.faces()]
    assert [r.key for r in mesh.edges] == [e.key for e in solid.edges()]
    assert mesh.vertices == [v.key for v in solid.vertices()]
    assert_covers(mesh.faces, len(mesh.indices))
    assert_covers(mesh.edges, len(mesh.edge_points) // 3)
    assert len(mesh.normals) == len(mesh.positions)
    assert len(mesh.vertex_points) == 3 * len(mesh.vertices)


def test_a_picked_triangle_names_the_face_it_lies_on():
    solid = holed_block()
    mesh = tessellate(solid)
    by_key = {face.key: face for face in solid.faces()}
    top = next(f for f in solid.faces() if face_properties(f).centroid.as_tuple()[2] == pytest.approx(10))

    hits = [r for r in mesh.faces if r.key == top.key]
    assert len(hits) == 1
    triangle = hits[0].start // 3
    corners = mesh.indices[3 * triangle : 3 * triangle + 3]
    assert all(mesh.positions[3 * i + 2] == pytest.approx(10) for i in corners)
    assert by_key[hits[0].key] == top


def test_a_cell_tessellates_with_only_its_own_cells():
    face = faces.rectangle(2.0, 3.0)
    mesh = tessellate(face)
    assert [r.key for r in mesh.faces] == [face.key]
    assert len(mesh.edges) == 4 and len(mesh.vertices) == 4

    mesh = tessellate(profiles.rectangle(2.0, 3.0))
    assert mesh.faces == [] and mesh.indices == []
    assert len(mesh.edges) == 4

    line = edges.line((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    mesh = tessellate(line)
    assert [r.key for r in mesh.edges] == [line.key]
    assert mesh.edge_points == [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]


def test_tessellating_is_deterministic():
    solid = holed_block()
    assert tessellate(solid) == tessellate(solid)
    assert tessellate(solid) == tessellate(holed_block())


def test_only_cells_are_tessellated():
    with pytest.raises(TypeError):
        tessellate(g.Point(0, 0, 0))
