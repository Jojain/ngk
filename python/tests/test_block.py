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


def test_edges_compare_by_topological_identity():
    solid = ngk.block(1.0, 2.0, 3.0)
    edge_occurrences = [edge for face in solid.faces() for edge in face.edges()]

    found_equal_pair = False
    for index, edge in enumerate(edge_occurrences):
        for other in edge_occurrences[index + 1 :]:
            if edge == other:
                found_equal_pair = True
                assert hash(edge) == hash(other)
                assert edge.key == other.key
                assert edge.start.point.as_tuple() == other.start.point.as_tuple()
                assert edge.end.point.as_tuple() == other.end.point.as_tuple()
                break
        if found_equal_pair:
            break

    assert found_equal_pair
    assert len(set(edge_occurrences)) == solid.edge_count


def test_topology_wrappers_compare_by_identity():
    solid = ngk.block(1.0, 2.0, 3.0)
    other_solid = ngk.block(1.0, 2.0, 3.0)

    assert solid == solid
    assert solid != other_solid

    faces = solid.faces()
    assert faces[0] == solid.faces()[0]
    assert len(set(faces)) == solid.face_count
    assert faces[0] != other_solid.faces()[0]

    vertices = [
        vertex
        for face in faces
        for edge in face.edges()
        for vertex in edge.vertices()
    ]
    assert len(set(vertices)) == solid.vertex_count
    assert vertices[0] == vertices[0]
    assert vertices[0] != other_solid.faces()[0].edges()[0].vertices()[0]

    assert solid.outer_shell == solid.shells[0]
    assert solid.outer_shell != other_solid.outer_shell

    first_loop = faces[0].outer_loop
    assert first_loop == faces[0].outer_loop
    assert first_loop != other_solid.faces()[0].outer_loop


def test_vertices_and_edges_traverse_to_faces():
    solid = ngk.block(1.0, 2.0, 3.0)
    face = solid.faces()[0]
    edge = face.edges()[0]
    vertex = edge.start

    assert face in edge.faces()
    assert face in vertex.faces()
    assert edge in vertex.edges()
    assert len(edge.faces()) == 2
    assert len(vertex.facets()) == 3
