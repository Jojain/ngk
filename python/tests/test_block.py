import ngk


def test_block_traversal_counts():
    solid = ngk.block(1.0, 2.0, 3.0)

    faces = solid.faces()
    edge_occurrences = [edge for face in faces for edge in face.edges()]
    vertex_occurrences = [vertex for edge in edge_occurrences for vertex in edge.vertices()]

    assert len(faces) == 6
    assert solid.face_count == 6
    assert solid.edge_count == 12
    assert solid.vertex_count == 8
    assert len(edge_occurrences) == 24
    assert len(vertex_occurrences) == 48


def test_block_properties_expose_geometry():
    solid = ngk.block(1.0, 2.0, 3.0)
    face = solid.faces()[0]
    edge = face.edges()[0]

    assert face.surface.kind == "plane"
    assert edge.curve.kind == "line"
    assert edge.start.point is not None
    assert edge.end.point is not None
    assert edge.length is not None


def test_nested_traversal_returns_vertices():
    solid = ngk.block(1.0, 2.0, 3.0)

    vertices = solid.faces()[0].edges()[0].vertices()

    assert len(vertices) == 2
    assert all(vertex.point is not None for vertex in vertices)
