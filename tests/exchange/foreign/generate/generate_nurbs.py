"""Regenerates lofted.step and swept_circle.step in the sibling `files/` dir.

Run with `uv run python tests/exchange/foreign/generate/generate_nurbs.py` from
the repo root. See skills/build123d-step-testing/SKILL.md for background on
using build123d to produce STEP fixtures.

Two fixtures, one per spelling, because Part 21 writes a B-spline two ways and
the two share no attribute layout at all.

`lofted.step` is polynomial throughout, so every spline in it is a *simple*
instance: one `B_SPLINE_CURVE_WITH_KNOTS` or `B_SPLINE_SURFACE_WITH_KNOTS`
record carrying its inherited attributes as well as its own.

`swept_circle.step` is rational throughout, so every spline in it is a
*complex* instance, each record carrying only what its own supertype declares.
It is the harder of the two in three more ways: the surface's two degrees
differ, so a transposed control net is visible; its u knots are unclamped; and
its wall is periodic, so it arrives cut open along a seam as well.
"""

import build123d as bd

with bd.BuildPart() as lofted:
    with bd.BuildSketch(bd.Plane.XY):
        bd.Rectangle(20, 12)
    with bd.BuildSketch(bd.Plane.XY.offset(10)):
        bd.Circle(5)
    bd.loft()
bd.export_step(lofted.part, "tests/exchange/foreign/files/lofted.step")

with bd.BuildPart() as swept:
    with bd.BuildLine():
        bd.Spline((0, 0, 0), (5, 3, 4), (10, -2, 8))
    with bd.BuildSketch(bd.Plane(origin=(0, 0, 0), z_dir=(0.6, 0.36, 0.48))):
        bd.Circle(2)
    bd.sweep()
bd.export_step(swept.part, "tests/exchange/foreign/files/swept_circle.step")
