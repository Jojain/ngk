from ngk.model import Model
from ngk.modeling import profiles, solids


def test_model_serialization_and_raw_topology_exploration():
    solid = solids.block(1.0, 2.0, 3.0)
    model = solid.model
    restored = Model.deserialize(model.serialize())

    assert model.dimension == 3
    assert model.involution_count == 4
    assert restored.dart_count == model.dart_count
    assert len(restored.vertices()) == 8
    assert len(restored.edges()) == 12
    assert len(restored.faces()) == 6
    assert len(restored.solids()) == 1

    dart = restored.darts()[0]
    assert restored.alpha(0, restored.alpha(0, dart)) == dart
    assert len(restored.cell_darts(dart, 0)) > 0
    assert restored.cell_representative(dart, 0) in restored.cells(0)


def test_typed_lookup_and_traversal_preserve_contextual_orientation():
    solid = solids.block(1.0, 2.0, 3.0)
    face = solid.faces()[0]
    edge = face.edges()[0]
    reversed_edge = edge.reversed()

    assert solid.model.face(face.dart_id) == face
    assert solid.model.edge(edge.dart_id) == edge
    assert edge == reversed_edge
    assert edge.dart_id != reversed_edge.dart_id
    assert edge.start == reversed_edge.end
    assert edge.end == reversed_edge.start
    assert edge.curve.point_at(0.5) is not None

    shell = solid.outer_shell
    reversed_shell = shell.reversed()
    assert shell == reversed_shell
    assert shell.dart_id != reversed_shell.dart_id

    profile = profiles.rectangle(1.0, 2.0)
    reversed_profile = profile.reversed()
    assert profile == reversed_profile
    assert profile.dart_id != reversed_profile.dart_id
    assert profile.edges()[0].start == reversed_profile.edges()[0].end


def test_all_typed_objects_retain_their_shared_model():
    solid = solids.block(1.0, 2.0, 3.0)
    shell = solid.shells()[0]
    sheet = solid.model.sheets()[0]
    face = shell.faces()[0]
    loop = face.loops()[0]
    edge = loop.edges()[0]
    vertex = edge.start
    profile = profiles.rectangle(1.0, 2.0)

    assert shell.model.solids()[0] == solid
    assert sheet.model.dart_count == solid.model.dart_count
    assert face.model.faces()[0].model.dart_count == solid.model.dart_count
    assert loop.model.dart_count == solid.model.dart_count
    assert edge.model.dart_count == solid.model.dart_count
    assert vertex.model.dart_count == solid.model.dart_count
    assert profile.model.profiles()[0] == profile

