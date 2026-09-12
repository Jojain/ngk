"""Regenerates holed_slab.step in this directory.

A 4 × 3 × 3 slab with a 1 × 1 square hole bored through it. Where box.step
covers the ordinary path, this one covers the two vendor deviations that a
single-bound solid never reaches: an `ADVANCED_FACE` carrying *two* bounds
with neither marked `FACE_OUTER_BOUND`, so the importer has to pick the outer
one by winding, and an inner loop whose direction must come back as a hole
rather than as a second outer boundary.

Run with `uv run python tests/fixtures/step/generate_holed_slab.py` from the
repo root. See skills/build123d-step-testing/SKILL.md for background.
"""

import build123d as bd

slab = bd.Box(4, 3, 3) - bd.Box(1, 1, 10)
bd.export_step(slab, "tests/fixtures/step/holed_slab.step")
