"""The bundles that topology entities return from their measures.

Edge and Profile have ``linear_properties``; Face, Sheet and Solid have
``area`` and ``surface_properties``; Solid also has ``volume`` and
``volume_properties``.
"""

from ..core.measurement import LinearProperties, SurfaceProperties, VolumeProperties

__all__ = ["LinearProperties", "SurfaceProperties", "VolumeProperties"]
