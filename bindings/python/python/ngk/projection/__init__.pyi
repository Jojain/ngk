from typing import Literal

from ..geometry import (
    Axis,
    Circle,
    Cone,
    Cylinder,
    Ellipse,
    Helix,
    Line,
    NurbsCurve,
    NurbsSurface,
    Plane,
    Point,
    Point2,
    RuledSurface,
    Sphere,
    SurfaceOfRevolution,
    Torus,
    Vector,
)
from ..topology import Edge, Face, Profile, Vertex

type Curve = Line | Circle | Ellipse | Helix | NurbsCurve
type Surface = Plane | Cylinder | Sphere | Cone | Torus | RuledSurface | SurfaceOfRevolution | NurbsSurface

# A projection that has no single answer. Every refusal below derives from it,
# and it derives from ValueError.
class ProjectionError(ValueError): ...

# Nothing lies where the projection looks.
class MissError(ProjectionError): ...

# Several nearest points are equally near the point.
class AmbiguousError(ProjectionError): ...

# The ray touches the surface without crossing it.
class GrazingRayError(ProjectionError): ...

# The ray lies in the surface.
class RayInSurfaceError(ProjectionError): ...

class UnsupportedSurfaceError(ProjectionError):
    surface: str

class UnsupportedCurveError(ProjectionError):
    curve: str

# A point carried onto a target. `parameters` is the parameter on a curve or the
# (u, v) on a surface, when the projection computed it. `hit` is the Edge or Face
# of the target the point landed on, None for a plane, a surface or a curve.
class PointProjection:
    @property
    def point(self) -> Point: ...
    @property
    def parameters(self) -> float | Point2 | None: ...
    @property
    def distance(self) -> float: ...
    @property
    def hit(self) -> Edge | Face | None: ...

# A span of a curve in 3D.
class TrimmedCurve:
    @property
    def curve(self) -> Curve: ...
    @property
    def interval(self) -> tuple[float, float]: ...
    @property
    def start(self) -> Point: ...
    @property
    def end(self) -> Point: ...
    @property
    def length(self) -> float: ...
    def point_at(self, fraction: float) -> Point: ...

# A span of a curve in the parameter space of the target.
class TrimmedCurve2:
    @property
    def interval(self) -> tuple[float, float]: ...
    @property
    def start(self) -> Point2: ...
    @property
    def end(self) -> Point2: ...
    @property
    def length(self) -> float: ...
    def point_at(self, fraction: float) -> Point2: ...

# How closely a segment follows the image of its source: `exact` is the image
# itself, otherwise `error` is the largest distance between the fit and the image.
class Exactness:
    @property
    def exact(self) -> bool: ...
    @property
    def error(self) -> float | None: ...

# Why a chain begins or stops where it does; `vertex` is the vertex of the source
# a "source_vertex" end is on.
class ChainEnd:
    @property
    def kind(self) -> Literal["closed", "source_end", "source_vertex", "miss", "jump"]: ...
    @property
    def vertex(self) -> Vertex | None: ...

# The part of the source a segment is the image of: `edge` is None when the source
# is a curve, and `interval` is directed.
class SourceSpan:
    @property
    def edge(self) -> Edge | None: ...
    @property
    def interval(self) -> tuple[float, float]: ...

class ProjectedSegment:
    @property
    def curve(self) -> TrimmedCurve: ...
    @property
    def pcurve(self) -> TrimmedCurve2: ...
    @property
    def hit(self) -> Face | None: ...
    @property
    def source(self) -> SourceSpan: ...
    @property
    def fit(self) -> Exactness: ...

class ProjectedChain:
    @property
    def segments(self) -> list[ProjectedSegment]: ...
    @property
    def start(self) -> ChainEnd: ...
    @property
    def end(self) -> ChainEnd: ...
    @property
    def closed(self) -> bool: ...
    def curves(self) -> list[TrimmedCurve]: ...
    def pcurves(self) -> list[TrimmedCurve2]: ...

class CurveProjection:
    @property
    def chains(self) -> list[ProjectedChain]: ...
    def segments(self) -> list[ProjectedSegment]: ...
    def curves(self) -> list[TrimmedCurve]: ...
    def pcurves(self) -> list[TrimmedCurve2]: ...

# Projects a point onto a plane, a surface, a curve, an axis, an edge, a face or a
# list of faces of one model; a curve, an edge or a profile onto a plane, a face or
# a list of faces. Without `along` the projection is the perpendicular; with it,
# the first hit of the half-line from the source in that direction. A curve, an
# axis or an edge has only the perpendicular: `along` for it raises ValueError.
# Each target class (Plane, a surface, a curve, Axis, Edge, Face) also has a
# `project(source, along=None, tolerance=None)` method doing the same with itself
# as the target; a curve, an axis or an edge takes no `along`. `tolerance` is the
# distance within which two points coincide. Raises ProjectionError or one of its
# subclasses when the projection has no single answer, and TypeError for a pair
# that is not projected.
def project(
    source: Point | tuple[float, float, float] | Curve | Edge | Profile,
    target: Plane | Surface | Curve | Axis | Edge | Face | list[Face],
    along: Vector | None = None,
    tolerance: float | None = None,
) -> PointProjection | CurveProjection: ...

# As `project` for a point source, keeping only the projected point.
def project_point(
    source: Point | tuple[float, float, float],
    target: Plane | Surface | Curve | Axis | Edge | Face | list[Face],
    along: Vector | None = None,
    tolerance: float | None = None,
) -> Point: ...

__all__ = [
    "AmbiguousError",
    "ChainEnd",
    "CurveProjection",
    "Exactness",
    "GrazingRayError",
    "MissError",
    "PointProjection",
    "ProjectedChain",
    "ProjectedSegment",
    "ProjectionError",
    "RayInSurfaceError",
    "SourceSpan",
    "TrimmedCurve",
    "TrimmedCurve2",
    "UnsupportedCurveError",
    "UnsupportedSurfaceError",
    "project",
    "project_point",
]
