"""Regenerates sphere.step in the sibling `files/` directory.

Run with `uv run python tests/exchange/foreign/generate/generate_sphere.py`
from the repo root. See skills/build123d-step-testing/SKILL.md for background
on using build123d to produce STEP fixtures.

OpenCascade writes a whole sphere as one face with no real boundary at all:
its single `FACE_BOUND` holds a `VERTEX_LOOP`, which names a point on the
face rather than a loop around it. That is the same face with no loops ngk
stores, so this is the fixture that shows the two kernels agreeing a support
can be covered entirely.
"""

import build123d as bd

sphere = bd.Sphere(5)
bd.export_step(sphere, "tests/exchange/foreign/files/sphere.step")
