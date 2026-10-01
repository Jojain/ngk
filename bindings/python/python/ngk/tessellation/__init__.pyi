from ..topology import Edge, Face, Profile, Sheet, Solid

class Range:
    """The run of a buffer one face or edge owns."""

    @property
    def key(self) -> str:
        """The cell's key, equal to its `Face.key` or `Edge.key`."""
    @property
    def start(self) -> int: ...
    @property
    def count(self) -> int: ...

class Tessellation:
    """A shape's mesh in three.js layout.

    Faces, edges and vertices come in the shape's own `faces()`, `edges()`
    and `vertices()` order, once each.
    """

    @property
    def positions(self) -> list[float]:
        """Flat xyz. Shared within a face, never across two."""
    @property
    def normals(self) -> list[float]:
        """Flat xyz, one per position."""
    @property
    def indices(self) -> list[int]:
        """Three per triangle."""
    @property
    def faces(self) -> list[Range]:
        """One range of `indices` per face, in index units: `geometry.addGroup(start, count)`."""
    @property
    def edge_points(self) -> list[float]:
        """Flat xyz, every edge's polyline one after another."""
    @property
    def edges(self) -> list[Range]:
        """One range of `edge_points` per edge, in point units. A closed edge ends on its first point."""
    @property
    def vertex_points(self) -> list[float]:
        """Flat xyz, one point per vertex: its stored position."""
    @property
    def vertices(self) -> list[str]:
        """The `Vertex.key` of each vertex point."""

def tessellate(shape: Solid | Sheet | Face | Profile | Edge) -> Tessellation:
    """Meshes `shape` at the viewer's sampling; raises naming a face that cannot be meshed."""
