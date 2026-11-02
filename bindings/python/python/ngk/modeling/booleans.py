"""Boolean operations on modelling shapes."""

from ..core.modeling.booleans import (
    FaceBooleanResult,
    SplitResult,
    cut,
    cut_faces,
    fuse,
    fuse_faces,
    intersect,
    intersect_faces,
    split,
)

__all__ = ["FaceBooleanResult", "cut", "cut_faces", "fuse", "fuse_faces", "intersect", "intersect_faces", "SplitResult", "split"]
