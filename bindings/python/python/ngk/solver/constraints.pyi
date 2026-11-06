from enum import Enum
from typing import ClassVar

from . import CircularId, GeometryId, PointLike, SegmentId

class TangencyMode(Enum):
    Automatic = ...
    External = ...
    Internal = ...
    FirstContainsSecond = ...
    SecondContainsFirst = ...

class Constraint:
    __match_args__: ClassVar[tuple[str, ...]]

class Coincident(Constraint):
    def __init__(self, first: PointLike, second: PointLike) -> None: ...
    first: PointLike
    second: PointLike

class PointOnLine(Constraint):
    def __init__(self, point: PointLike, line: SegmentId) -> None: ...
    point: PointLike
    line: SegmentId

class PointOnCircle(Constraint):
    def __init__(self, point: PointLike, circle: CircularId) -> None: ...
    point: PointLike
    circle: CircularId

class Horizontal(Constraint):
    def __init__(self, line: SegmentId) -> None: ...
    line: SegmentId

class Vertical(Constraint):
    def __init__(self, line: SegmentId) -> None: ...
    line: SegmentId

class Distance(Constraint):
    def __init__(self, first: PointLike, second: PointLike, value: float) -> None: ...
    first: PointLike
    second: PointLike
    value: float

class DistanceToLine(Constraint):
    def __init__(self, point: PointLike, line: SegmentId, value: float) -> None: ...
    point: PointLike
    line: SegmentId
    value: float

class OffsetX(Constraint):
    def __init__(self, first: PointLike, second: PointLike, value: float) -> None: ...
    first: PointLike
    second: PointLike
    value: float

class OffsetY(Constraint):
    def __init__(self, first: PointLike, second: PointLike, value: float) -> None: ...
    first: PointLike
    second: PointLike
    value: float

class Length(Constraint):
    def __init__(self, line: SegmentId, value: float) -> None: ...
    line: SegmentId
    value: float

class Radius(Constraint):
    def __init__(self, circle: CircularId, value: float) -> None: ...
    circle: CircularId
    value: float

class EqualLength(Constraint):
    def __init__(self, first: SegmentId, second: SegmentId) -> None: ...
    first: SegmentId
    second: SegmentId

class EqualRadius(Constraint):
    def __init__(self, first: CircularId, second: CircularId) -> None: ...
    first: CircularId
    second: CircularId

class Parallel(Constraint):
    def __init__(self, first: SegmentId, second: SegmentId) -> None: ...
    first: SegmentId
    second: SegmentId

class Perpendicular(Constraint):
    def __init__(self, first: SegmentId, second: SegmentId) -> None: ...
    first: SegmentId
    second: SegmentId

class Angle(Constraint):
    def __init__(self, first: SegmentId, second: SegmentId, radians: float, oriented: bool = False) -> None: ...
    first: SegmentId
    second: SegmentId
    radians: float
    oriented: bool

class Concentric(Constraint):
    def __init__(self, first: CircularId, second: CircularId) -> None: ...
    first: CircularId
    second: CircularId

class Midpoint(Constraint):
    def __init__(self, point: PointLike, start: PointLike, end: PointLike) -> None: ...
    point: PointLike
    start: PointLike
    end: PointLike

class SymmetricAboutLine(Constraint):
    def __init__(self, first: PointLike, second: PointLike, line: SegmentId) -> None: ...
    first: PointLike
    second: PointLike
    line: SegmentId

class SymmetricAboutPoint(Constraint):
    def __init__(self, first: PointLike, second: PointLike, center: PointLike) -> None: ...
    first: PointLike
    second: PointLike
    center: PointLike

class FixedPoint(Constraint):
    def __init__(self, point: PointLike) -> None: ...
    point: PointLike

class Fixed(Constraint):
    def __init__(self, primitive: GeometryId) -> None: ...
    primitive: GeometryId

class TangentLineCircle(Constraint):
    def __init__(self, line: SegmentId, circle: CircularId) -> None: ...
    line: SegmentId
    circle: CircularId

class TangentCircles(Constraint):
    def __init__(self, first: CircularId, second: CircularId, mode: TangencyMode = ...) -> None: ...
    first: CircularId
    second: CircularId
    mode: TangencyMode
