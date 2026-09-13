"""Regenerates frustum.step in the sibling `files/` directory.

Run with `uv run python tests/exchange/foreign/generate/generate_frustum.py`
from the repo root. See skills/build123d-step-testing/SKILL.md for background
on using build123d to produce STEP fixtures.

A truncated cone is the only way to get a `CONICAL_SURFACE` in front of ngk:
its own revolve builder refuses a profile that touches the axis, so nothing
in the kernel produces a cone solid. The cone is also the one analytic
support whose parameterization differs from STEP's, which makes reading one
the check that the difference is handled rather than assumed away.
"""

import build123d as bd

frustum = bd.Cone(bottom_radius=5, top_radius=2, height=10)
bd.export_step(frustum, "tests/exchange/foreign/files/frustum.step")
