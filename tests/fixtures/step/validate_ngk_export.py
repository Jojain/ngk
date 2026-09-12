"""Checks ngk's STEP output against OpenCascade, via build123d.

Run `cargo run --example step_export_fixtures` first; it writes the files
this reads. A clean import is itself the signal that matters — OCCT rejecting
the file, or reading it as loose faces rather than one solid, is what the
Rust-side structural tests cannot catch.

Both directions are covered: the plain cases are shapes ngk built, and the
`reexported_*` ones are OpenCascade's own fixtures that ngk read in and wrote
back out, so the whole loop is checked by the kernel that started it.

    cargo run --example step_export_fixtures
    uv run python tests/fixtures/step/validate_ngk_export.py
"""

import math
import sys
from pathlib import Path

import build123d as bd

OUT_DIR = Path(__file__).resolve().parents[3] / "target" / "step_export"

CASES = [
    # file, volume, area, (faces, edges, vertices), bbox
    ("block.step", 6000.0, 2200.0, (6, 12, 8), (10.0, 20.0, 30.0)),
    ("holed_slab.step", 33.0, 76.0, (10, 24, 16), (4.0, 3.0, 3.0)),
    # The seam case: ngk stores the wall as one ring face with no seam edge
    # at all, so the cut in this file was synthesized on the way out. OCCT
    # counting three faces and one closed solid of the right volume is what
    # says the cut landed on the cylinder and sewed back up.
    (
        "cylinder.step",
        math.pi * 25.0 * 10.0,
        2.0 * math.pi * 25.0 + 2.0 * math.pi * 5.0 * 10.0,
        (3, 3, 2),
        (10.0, 10.0, 10.0),
    ),
    # The boundaryless cases. ngk stores a sphere and a torus as one face with
    # no loop, edge or vertex at all, so every topological entity in these
    # files was synthesized from the support's domain. A sphere's cut collapses
    # at both poles and leaves one edge between two vertices; a torus closes in
    # both parameters and leaves two edges meeting at one.
    (
        "sphere.step",
        4.0 / 3.0 * math.pi * 125.0,
        4.0 * math.pi * 25.0,
        (1, 1, 2),
        (10.0, 10.0, 10.0),
    ),
    (
        "torus.step",
        2.0 * math.pi**2 * 3.0 * 1.0,
        4.0 * math.pi**2 * 3.0 * 1.0,
        (1, 2, 1),
        (8.0, 8.0, 2.0),
    ),
    # The void case, where the volume is the whole assertion: a cavity written
    # as a second outer shell gives a valid file describing a solid ball.
    (
        "hollow_sphere.step",
        4.0 / 3.0 * math.pi * (125.0 - 8.0),
        4.0 * math.pi * (25.0 + 4.0),
        (2, 2, 4),
        (10.0, 10.0, 10.0),
    ),
    # The spline case: a 4 x 4 channel cut through the corner of a 10-cube.
    # Every support involved is planar, and most of the edges are still
    # free-form — an imprint's section is fitted rather than recognized — so
    # this is the shape that could not be written at all until B-splines could.
    (
        "cut_block.step",
        1000.0 - 4.0 * 4.0 * 10.0,
        2.0 * 100.0 + 2.0 * 60.0 + 2.0 * 84.0 + 2.0 * 40.0,
        (8, 18, 12),
        (10.0, 10.0, 10.0),
    ),
    # The read direction. These are ngk's re-exports of the OpenCascade
    # fixtures in this directory, so a passing case means the import
    # understood what OCCT wrote rather than merely producing a map that
    # satisfies ngk's own validators — which cell counts alone cannot show.
    ("reexported_box.step", 6000.0, 2200.0, (6, 12, 8), (10.0, 20.0, 30.0)),
    # OpenCascade's own cylinder, read in and written back. Its wall arrives
    # cut open along a SEAM_CURVE that ngk has no edge for, heals into a ring
    # face, and is cut open again on the way out — so a matching volume here
    # says both halves of the seam handling agree with the kernel that wrote
    # the file.
    # The cone, which only exists in ngk by way of a file: no builder makes
    # one, so this is the only route by which a CONICAL_SURFACE is both read
    # and written. Volume is the assertion that matters here, because the v
    # parameterizations differ between the two kernels and a mishandled scale
    # produces a cone of the wrong taper that is otherwise entirely plausible.
    (
        "reexported_frustum.step",
        math.pi * 10.0 / 3.0 * (25.0 + 10.0 + 4.0),
        math.pi * (25.0 + 4.0) + math.pi * 7.0 * math.hypot(3.0, 10.0),
        (3, 3, 2),
        (10.0, 10.0, 10.0),
    ),
    (
        "reexported_cylinder.step",
        math.pi * 25.0 * 10.0,
        2.0 * math.pi * 25.0 + 2.0 * math.pi * 5.0 * 10.0,
        (3, 3, 2),
        (10.0, 10.0, 10.0),
    ),
    ("reexported_holed_slab.step", 33.0, 76.0, (10, 24, 16), (4.0, 3.0, 3.0)),
    # OpenCascade's own sphere, which it writes with no cut at all: one face
    # whose only bound is a VERTEX_LOOP. ngk reads that as the face with no
    # boundary it is, and writes it back cut open along a meridian — so a
    # matching volume says the two spellings describe the same sphere.
    (
        "reexported_sphere.step",
        4.0 / 3.0 * math.pi * 125.0,
        4.0 * math.pi * 25.0,
        (1, 1, 2),
        (10.0, 10.0, 10.0),
    ),
    # OpenCascade.s own torus, read in and written back. Its face arrives cut
    # open twice and has to lose both cuts to become the one boundaryless face
    # ngk stores, then be cut open twice again on the way out — so a matching
    # volume here says the two seams were understood in both directions.
    (
        "reexported_torus.step",
        2.0 * math.pi**2 * 3.0 * 1.0,
        4.0 * math.pi**2 * 3.0 * 1.0,
        (1, 2, 1),
        (8.0, 8.0, 2.0),
    ),
]

# Free-form shapes, checked against the file they came from rather than
# against a formula. A lofted or swept solid has no closed-form volume worth
# writing down, and a constant copied from one run would only ever assert that
# nothing changed — where what matters is that ngk's round trip returned the
# shape OpenCascade started with. Both are ngk re-exports of a committed
# fixture, so the comparison is between two files OCCT reads.
AGAINST_SOURCE = [
    # The simple spelling: a polynomial B-spline is one record per entity,
    # carrying its inherited attributes as well as its own.
    ("reexported_lofted.step", "lofted.step"),
    # The complex spelling, and the awkward one: rational throughout, two
    # differing degrees, unclamped u knots, and a periodic wall that arrives
    # cut open along a seam.
    ("reexported_swept_circle.step", "swept_circle.step"),
]

TOLERANCE = 1e-6

# Free-form geometry goes through a fit on the way in and out, so the two sides
# of a source comparison agree to a model-scale tolerance rather than to the
# last bit.
FREE_FORM_TOLERANCE = 1e-6


def check(name, volume, area, counts, bbox):
    path = OUT_DIR / name
    if not path.exists():
        raise SystemExit(f"missing {path}; run the example first")

    shape = bd.import_step(str(path))
    solids = shape.solids()
    assert len(solids) == 1, f"{name}: read {len(solids)} solids, expected one"
    solid = solids[0]

    assert solid.is_valid, f"{name}: OCCT reports an invalid solid"
    assert abs(solid.volume - volume) < TOLERANCE, (
        f"{name}: volume {solid.volume}, expected {volume}"
    )
    assert abs(solid.area - area) < TOLERANCE, (
        f"{name}: area {solid.area}, expected {area}"
    )

    got = (len(solid.faces()), len(solid.edges()), len(solid.vertices()))
    assert got == counts, f"{name}: faces/edges/vertices {got}, expected {counts}"

    size = solid.bounding_box().size
    for axis, (actual, expected) in enumerate(zip((size.X, size.Y, size.Z), bbox)):
        assert abs(actual - expected) < TOLERANCE, (
            f"{name}: bbox axis {axis} is {actual}, expected {expected}"
        )

    print(f"  {name}: volume {solid.volume}, area {solid.area}, {got} — ok")


def check_against_source(name, source):
    path = OUT_DIR / name
    if not path.exists():
        raise SystemExit(f"missing {path}; run the example first")
    origin = Path(__file__).resolve().parent / source

    written = bd.import_step(str(path)).solids()
    original = bd.import_step(str(origin)).solids()
    assert len(written) == 1, f"{name}: read {len(written)} solids, expected one"
    written, original = written[0], original[0]

    assert written.is_valid, f"{name}: OCCT reports an invalid solid"
    for what, got, want in (
        ("volume", written.volume, original.volume),
        ("area", written.area, original.area),
    ):
        relative = abs(got - want) / max(abs(want), 1.0)
        assert relative < FREE_FORM_TOLERANCE, (
            f"{name}: {what} {got}, but {source} has {want}"
        )

    got = (len(written.faces()), len(written.edges()), len(written.vertices()))
    want = (len(original.faces()), len(original.edges()), len(original.vertices()))
    assert got == want, f"{name}: faces/edges/vertices {got}, but {source} has {want}"

    print(f"  {name}: volume {written.volume}, area {written.area}, {got} — matches {source}")


def main():
    print(f"validating ngk STEP output in {OUT_DIR}")
    for case in CASES:
        check(*case)
    for case in AGAINST_SOURCE:
        check_against_source(*case)
    print("all cases passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
