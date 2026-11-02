"""Typed topology views over an NGK model."""

from ..core import Edge, Face, Loop, Profile, Shell, Sheet, Solid, Vertex

# A model plus its primary entity, whatever that entity's kind.
type Shape = Vertex | Edge | Profile | Face | Sheet | Solid

__all__ = ["Edge", "Face", "Loop", "Profile", "Shape", "Shell", "Sheet", "Solid", "Vertex"]
