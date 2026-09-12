"""Regenerates cylinder.step in this directory.

Run with `uv run python tests/fixtures/step/generate_cylinder.py` from the
repo root. See skills/build123d-step-testing/SKILL.md for background on using
build123d to produce STEP fixtures.

The cylinder is the smallest shape whose wall STEP cannot spell without a
seam, so this is the fixture that shows ngk reading a *foreign* kernel's cut
rather than only its own.
"""

import build123d as bd

cylinder = bd.Cylinder(5, 10)
bd.export_step(cylinder, "tests/fixtures/step/cylinder.step")
