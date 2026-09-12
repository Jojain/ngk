---
name: build123d-step-testing
description: Use when testing, debugging, or validating ngk's STEP import/export (src/exchange/step/) against a known-good reference implementation. Covers using build123d (Python, wraps OCP/OpenCascade, already installed in .venv) to generate known-geometry STEP fixtures for testing ngk's importer, and to open/validate STEP files produced by ngk's exporter by comparing volume, bounding box, and vertex/face/edge counts. Use this whenever the user mentions build123d, STEP round-trip testing, STEP fixtures, or wants an independent oracle for STEP import/export correctness.
---

# build123d STEP Testing

build123d is a trusted, independent CAD kernel (OCP/OpenCascade under the
hood) used here purely as a **test oracle** for ngk's own STEP
import/export (`src/exchange/step/`). Never use it to implement ngk
behavior — only to generate fixtures and to check what ngk produced/consumed
against ground truth.

Already installed in the project venv (`.venv`, build123d 0.11.1). Run
scripts with `uv run python script.py`, which resolves to that same venv —
do not `pip install` into the system Python. `.venv/Scripts/python.exe`
(Windows) works too if `uv` isn't on `PATH`.

## Two testing directions

1. **ngk importer**: build a known shape in build123d → `export_step()` →
   feed the file into ngk's importer → assert ngk's result matches the
   shape's known properties (volume, bbox, vertex/face/edge count).
2. **ngk exporter**: run ngk's exporter on an ngk shape → `import_step()`
   the result back in build123d → assert build123d's reading of the file
   matches what the ngk shape should be. This also catches STEP files that
   are malformed enough that a real OCCT-based reader rejects them, which is
   a stronger check than ngk being able to re-read its own output.

## Fixture naming convention

When a fixture is checked in rather than generated on the fly in a test,
keep the generating script beside the file it produces, under
`tests/fixtures/step/`: `generate_<name>.py` produces `<name>.step`. This
lets a fixture be regenerated or extended without reverse-engineering it
from the STEP text. Keep the script itself uncommented — it's a few lines
of straight-line build123d calls, not something that needs explaining.

## Building known-geometry fixtures

```python
import build123d as bd

box = bd.Box(10, 20, 30)                       # centered at origin
cyl = bd.Cylinder(radius=5, height=10)
combined = box + cyl                            # boolean union
cut = box - cyl                                  # boolean subtraction
common = box & cyl                               # boolean intersection
```

Prefer primitives whose analytic properties are easy to state (volume,
bbox, counts) over free-form/imported sketches — the point of a fixture is
that its correct answer is obvious and independent of any kernel.

## Exporting a fixture to STEP

```python
bd.export_step(box, "fixture_box.step")
```

`unit` defaults to `Unit.MM`, matching ngk's convention — pass `unit=` only
if a test deliberately needs a different working unit. `write_pcurves`
defaults to `True`; keep it on unless a test specifically targets
pcurve-less STEP files.

## Importing an ngk-produced STEP file

```python
result = bd.import_step("ngk_output.step")   # -> build123d Compound
```

`import_step` raises / returns an unusable result on structurally invalid
STEP — a clean import is itself a signal ngk's exporter produced something
a real reader accepts.

## Inspecting a shape as a test oracle

```python
shape.volume                 # float, exact for analytic solids
shape.bounding_box().size    # Vector(X, Y, Z)
len(shape.vertices())
len(shape.faces())
len(shape.edges())
```

Compare these against ngk's own reported values (or against the origin
shape's known values, e.g. `bd.Box(10, 20, 30).volume == 6000.0`) with a
tolerance appropriate to `LINEAR_TOLERANCE` for lengths/bbox and a
relative tolerance for volume — don't assert exact float equality across
two different kernels.

## Where this fits in the repo

- Treat build123d scripts as throwaway test harnesses, not part of the
  `ngk` crate or its Python bindings (`bindings/python/`) — those bind
  *ngk's own* geometry, unrelated to build123d.
- Rust-side STEP round-trip tests still belong under `tests/exchange/step/`
  per the project's normal test-location rule; build123d is the tool you
  reach for to *generate the fixture file* or *independently check* what
  those tests assert against, not a replacement for them.
