"""Regenerates box.step in the sibling `files/` directory.

Run with `uv run python tests/exchange/foreign/generate/generate_box.py` from
the repo root. See skills/build123d-step-testing/SKILL.md for background on
using build123d to produce STEP fixtures.
"""

import build123d as bd

box = bd.Box(10, 20, 30)
bd.export_step(box, "tests/exchange/foreign/files/box.step")
