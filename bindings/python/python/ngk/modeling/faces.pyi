from ..geometry import Plane, Point2
from ..solver import CurveId, System2d
from ..topology import Edge, Face, Profile
from .edges import EdgeSplitResult
from .profiles import JointGapError, NotSolvedError, OverlappingError, UncertainCrossingError

def rectangle(x_size: float, y_size: float, plane: Plane | None = None) -> Face: ...
def square(size: float, plane: Plane | None = None) -> Face: ...
def polygon(points: list[tuple[float, float, float]]) -> Face: ...
def polygon_with_holes(
    outer: list[tuple[float, float, float]],
    holes: list[list[tuple[float, float, float]]],
    plane: Plane | None = None,
) -> Face: ...
def circle(radius: float, plane: Plane | None = None) -> Face: ...
def annulus(outer_radius: float, inner_radius: float, plane: Plane | None = None) -> Face: ...
def from_profile(profile: Profile) -> Face: ...
def split_boundary_edge(face: Face, edge: Edge, fraction: float) -> EdgeSplitResult: ...

class SystemRegion:
    @property
    def face(self) -> Face: ...
    @property
    def boundary(self) -> list[tuple[CurveId, Edge]]: ...

class SystemRegions:
    @property
    def regions(self) -> list[SystemRegion]: ...

def from_system(system: System2d, plane: Plane, curves: list[CurveId]) -> SystemRegions: ...

class CutOffCurveError(ValueError):
    curve: CurveId
    point: Point2
    gap: float

class PinchedRegionError(ValueError):
    point: Point2

__all__ = [
    "CutOffCurveError",
    "JointGapError",
    "NotSolvedError",
    "OverlappingError",
    "PinchedRegionError",
    "SystemRegion",
    "SystemRegions",
    "UncertainCrossingError",
    "annulus",
    "circle",
    "from_profile",
    "from_system",
    "polygon",
    "polygon_with_holes",
    "rectangle",
    "split_boundary_edge",
    "square",
]
