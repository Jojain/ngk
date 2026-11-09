from ..geometry import Plane, Point2
from ..projection import CurveProjection
from ..solver import CurveId, System2d
from ..topology import Edge, Profile

def rectangle(x_size: float, y_size: float, plane: Plane | None = None) -> Profile: ...
def square(size: float, plane: Plane | None = None) -> Profile: ...
def polyline(points: list[tuple[float, float, float]]) -> Profile: ...
def polygon(points: list[tuple[float, float, float]]) -> Profile: ...
def arc(plane: Plane, radius: float, start_angle: float, end_angle: float) -> Profile: ...
def from_edges(edges: list[Edge]) -> Profile: ...
def from_edge(edge: Edge) -> Profile: ...
def appended(profile: Profile, edge: Edge) -> Profile: ...

class SystemProfile:
    @property
    def profile(self) -> Profile: ...
    @property
    def curves(self) -> list[CurveId]: ...

class SystemProfiles:
    @property
    def edges(self) -> dict[CurveId, Edge]: ...
    @property
    def profiles(self) -> list[SystemProfile]: ...

def from_system(system: System2d, plane: Plane, curves: list[CurveId]) -> SystemProfiles: ...

class ProjectionProfile:
    @property
    def profile(self) -> Profile: ...
    @property
    def edges(self) -> list[Edge]: ...

class ProjectionProfiles:
    @property
    def profiles(self) -> list[ProjectionProfile]: ...

# Draws a curve projection in a fresh model: one profile per chain, in the chain
# order, closed when the chain is, with one edge per segment. Raises ValueError for
# a projection without chains or a chain whose segments do not continue one another.
def from_projection(projection: CurveProjection) -> ProjectionProfiles: ...

class NotSolvedError(ValueError): ...

class BranchingError(ValueError):
    curves: list[CurveId]

class JointGapError(ValueError):
    first: CurveId
    second: CurveId
    gap: float

class CrossingError(ValueError):
    first: CurveId
    second: CurveId
    point: Point2

class OverlappingError(ValueError):
    first: CurveId
    second: CurveId

class UncertainCrossingError(ValueError):
    first: CurveId
    second: CurveId

__all__ = [
    "BranchingError",
    "CrossingError",
    "JointGapError",
    "NotSolvedError",
    "OverlappingError",
    "ProjectionProfile",
    "ProjectionProfiles",
    "SystemProfile",
    "SystemProfiles",
    "UncertainCrossingError",
    "appended",
    "arc",
    "from_edge",
    "from_edges",
    "from_projection",
    "from_system",
    "polygon",
    "polyline",
    "rectangle",
    "square",
]
