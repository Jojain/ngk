"""Regenerates box.step in this directory.

Run with `uv run python tests/fixtures/step/generate_box.py` from the repo
root. See skills/build123d-step-testing/SKILL.md for background on using
build123d to produce STEP fixtures.
"""

import build123d as bd

box = bd.Box(10, 20, 30)
bd.export_step(box, "tests/fixtures/step/box.step")
