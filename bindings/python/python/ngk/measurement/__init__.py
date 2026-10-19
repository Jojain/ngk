"""The bundles that topology entities return from their measures.

Edge and Profile have ``linear_properties``; Face, Sheet and Solid have
``area`` and ``surface_properties``; Solid also has ``volume`` and
``volume_properties``; all five have ``bounding_box`` and
``oriented_bounding_box``, which return a ``BoundingBox``.
"""

from ..core.measurement import (
    Bound,
    BoundingBox,
    LinearProperties,
    SurfaceProperties,
    VolumeProperties,
)

__all__ = [
    "Bound", "BoundingBox", "LinearProperties", "SurfaceProperties", "VolumeProperties",
]
