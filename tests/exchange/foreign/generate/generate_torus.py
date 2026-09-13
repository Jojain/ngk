"""Regenerates torus.step in the sibling `files/` directory.

Run with `uv run python tests/exchange/foreign/generate/generate_torus.py` from
the repo root. See skills/build123d-step-testing/SKILL.md for background on
using build123d to produce STEP fixtures.

A torus is the shape that needs two cuts rather than one, so this is the
fixture that shows ngk taking both of a foreign kernel's seams off and
arriving at the one face with no boundary that a torus actually is.
"""

import build123d as bd

torus = bd.Torus(3, 1)
bd.export_step(torus, "tests/exchange/foreign/files/torus.step")
