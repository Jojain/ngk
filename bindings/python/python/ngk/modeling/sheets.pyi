from ..topology import Face, Sheet

# Joins faces along the edges they share whole. They must make one connected,
# consistently wound sheet with no edge run by more than two of them; the sheet
# is oriented like the first face. The sources are unchanged.
def from_faces(faces: list[Face]) -> Sheet: ...

__all__ = [
    "from_faces",
]
