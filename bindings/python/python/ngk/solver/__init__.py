"""The 2D constraint solver."""

from ..core.solver import (
    Arc2,
    ArcId,
    ArcTarget,
    Circle2,
    CircleId,
    CircleTarget,
    ConflictError,
    ConstraintId,
    ConstraintResidual,
    NonConvergenceError,
    PointId,
    PointRef,
    PointTarget,
    Redundancy,
    RedundancyKind,
    Segment2,
    SegmentId,
    SegmentTarget,
    Solution2d,
    SolveReport,
    System2d,
    Target,
    TargetReport,
)
from . import assembly, constraints

# The id kinds a parameter or a result may hold, for annotations.
type GeometryId = PointId | SegmentId | CircleId | ArcId
type CircularId = CircleId | ArcId
type CurveId = SegmentId | CircleId | ArcId
type PointLike = PointRef | PointId

__all__ = [
    "Arc2",
    "ArcId",
    "ArcTarget",
    "Circle2",
    "CircleId",
    "CircleTarget",
    "CircularId",
    "ConflictError",
    "ConstraintId",
    "ConstraintResidual",
    "CurveId",
    "GeometryId",
    "NonConvergenceError",
    "PointId",
    "PointLike",
    "PointRef",
    "PointTarget",
    "Redundancy",
    "RedundancyKind",
    "Segment2",
    "SegmentId",
    "SegmentTarget",
    "Solution2d",
    "SolveReport",
    "System2d",
    "Target",
    "TargetReport",
    "assembly",
    "constraints",
]
