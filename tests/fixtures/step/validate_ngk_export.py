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
]

TOLERANCE = 1e-6


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


def main():
    print(f"validating ngk STEP output in {OUT_DIR}")
    for case in CASES:
        check(*case)
    print("all cases passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
