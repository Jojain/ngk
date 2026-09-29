"""Boolean operations on modelling shapes."""

from ..core.modeling.booleans import (
    FaceBooleanResult,
    cut,
    cut_faces,
    fuse,
    fuse_faces,
    intersect,
    intersect_faces,
)

__all__ = ["FaceBooleanResult", "cut", "cut_faces", "fuse", "fuse_faces", "intersect", "intersect_faces"]
