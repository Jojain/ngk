import ngk


def test_gmap_serialization_and_raw_topology_exploration():
    solid = ngk.block(1.0, 2.0, 3.0)
    gmap = solid.gmap
    restored = ngk.GMap.deserialize(gmap.serialize())

    assert gmap.dimension == 3
    assert gmap.involution_count == 4
    assert restored.dart_count == gmap.dart_count
    assert len(restored.vertices()) == 8
    assert len(restored.edges()) == 12
    assert len(restored.faces()) == 6
    assert len(restored.solids()) == 1

    dart = restored.darts()[0]
    assert restored.alpha(0, restored.alpha(0, dart)) == dart
    assert len(restored.cell_darts(dart, 0)) > 0
    assert restored.cell_representative(dart, 0) in restored.cells(0)


def test_typed_lookup_and_traversal_preserve_contextual_orientation():
    solid = ngk.block(1.0, 2.0, 3.0)
    face = solid.faces()[0]
    edge = face.edges()[0]
    reversed_edge = edge.reversed()

    assert solid.gmap.face(face.dart_id) == face
    assert solid.gmap.edge(edge.dart_id) == edge
    assert edge == reversed_edge
    assert edge.dart_id != reversed_edge.dart_id
    assert edge.start == reversed_edge.end
    assert edge.end == reversed_edge.start
    assert edge.curve.point_at(0.5) is not None

    shell = solid.outer_shell
    reversed_shell = shell.reversed()
    assert shell == reversed_shell
    assert shell.dart_id != reversed_shell.dart_id

    profile = ngk.rectangle_profile(1.0, 2.0)
    reversed_profile = profile.reversed()
    assert profile == reversed_profile
    assert profile.dart_id != reversed_profile.dart_id
    assert profile.edges()[0].start == reversed_profile.edges()[0].end


def test_all_typed_objects_retain_their_shared_gmap():
    solid = ngk.block(1.0, 2.0, 3.0)
    shell = solid.shells()[0]
    sheet = solid.gmap.sheets()[0]
    face = shell.faces()[0]
    loop = face.loops()[0]
    edge = loop.edges()[0]
    vertex = edge.start
    profile = ngk.rectangle_profile(1.0, 2.0)

    assert shell.gmap.solids()[0] == solid
    assert sheet.gmap.dart_count == solid.gmap.dart_count
    assert face.gmap.faces()[0].gmap.dart_count == solid.gmap.dart_count
    assert loop.gmap.dart_count == solid.gmap.dart_count
    assert edge.gmap.dart_count == solid.gmap.dart_count
    assert vertex.gmap.dart_count == solid.gmap.dart_count
    assert profile.gmap.profiles()[0] == profile

