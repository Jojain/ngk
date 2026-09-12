"""Checks ngk's STEP output against OpenCascade, via build123d.

Run `cargo run --example step_export_fixtures` first; it writes the files
this reads. A clean import is itself the signal that matters — OCCT rejecting
the file, or reading it as loose faces rather than one solid, is what the
Rust-side structural tests cannot catch.

    cargo run --example step_export_fixtures
    uv run python tests/fixtures/step/validate_ngk_export.py
"""

import sys
from pathlib import Path

import build123d as bd

OUT_DIR = Path(__file__).resolve().parents[3] / "target" / "step_export"

CASES = [
    # file, volume, area, (faces, edges, vertices), bbox
    ("block.step", 6000.0, 2200.0, (6, 12, 8), (10.0, 20.0, 30.0)),
    ("holed_slab.step", 33.0, 76.0, (10, 24, 16), (4.0, 3.0, 3.0)),
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
