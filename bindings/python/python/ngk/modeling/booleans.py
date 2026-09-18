"""Boolean operations on modelling shapes.

The current operations accept solids. Their placement here preserves the
dimension-agnostic modelling namespace that future Boolean operations will use.
"""

from ..core.modeling.booleans import cut, fuse, intersect

__all__ = ["cut", "fuse", "intersect"]
