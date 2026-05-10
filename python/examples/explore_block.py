
#%%
import ngk

#%%


solid = ngk.block(1.0, 2.0, 3.0)

print(solid)
for face_index, face in enumerate(solid.faces()):
    print(f"face {face_index}: {face} surface={face.surface.kind}")
    for edge_index, edge in enumerate(face.edges()):
        start = edge.start.point
        end = edge.end.point
        print(
            "  edge "
            f"{edge_index}: dart={edge.dart_id} "
            f"length={edge.length} "
            f"start={start.as_tuple()} "
            f"end={end.as_tuple()}"
        )

#%%