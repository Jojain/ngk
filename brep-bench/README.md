# brep-bench

## How to run

### Dedicated comparison gallery

The gallery shows all 231 cases, with an OCCT/OCP preview beside the NGK
preview. Clicking a card opens two 3D viewers with orbit, pan, zoom, linked
cameras, a triangle view, measurements, diagnostics and STL downloads.
Search, category and NGK outcome filters apply to the gallery. A case can
be bookmarked through its `?case=category/name` URL.

From the repository root:

```powershell
python brep-bench/viewer.py --refresh
npm.cmd --prefix visualization run benchmark:dev
```

Open `http://127.0.0.1:3942/`. To export existing local runs without running
the benchmark again, use `python brep-bench/viewer.py`. `--run-reference`
runs only OCCT first. The reference interpreter defaults to the repository's
`.venv` interpreter when present, otherwise the Python running the script.
It must have OCP installed; override it with `--reference-python PATH`.

The OCCT implementations come from the exact upstream commit in `UPSTREAM`.
`occt_reference.py` supplies the collection type aliases that those cases
use when run with OCP 7.9 instead of OCP 8, and records the actual installed
OCP version. It does not alter their constructions. A reference can itself
fail or be incorrect; its own status appears beside its model.

`viewer.py` exports metadata and the actual STL artifacts into the ignored
`viewer-data/` directory. A refused case gets no mesh, even if an older STL
remains on disk. No kernel substitutes for a missing result. Previews load
as cards approach the viewport using one shared WebGL renderer; the full
viewers open only for the selected case. Both full viewers use the same
spatial frame, so geometric size and placement differences remain visible.

`npm.cmd --prefix visualization run benchmark:build` typechecks and builds
a standalone static site in `visualization/benchmark-dist/`, including the
exported data. These commands do not build WASM.

The existing Pages workflow also publishes the gallery at `/ngk/benchmark/`
on pushes to `master`. Its benchmark job installs the pinned OCP dependency
from `reference-requirements.txt`, builds NGK, runs both kernels and exports
their real results and models. The site build downloads these generated
artifacts and includes the standalone gallery beside `/ngk/sandbox/`.
No locally generated STL files need to be committed. The benchmark's
regression gate must pass before a new Pages artifact can be deployed.

`VITE_BASE_PATH` configures the gallery's asset prefix, using the same
convention as the visualization and sandbox. For a local Pages build:

```powershell
$env:VITE_BASE_PATH = '/ngk/benchmark/'
npm.cmd --prefix visualization run benchmark:build
Remove-Item Env:VITE_BASE_PATH
```

Two things guard the robustness work in `plan/robustness/`: the brep-bench gate, which runs the
231 cases of the neutral suite on this tree, and the reproduction suite, plain `cargo test` tests
of the failures the bench exposed, on plainer parts.

### The gate

```
python3 brep-bench/gate.py                  # whole suite, fails on any regression
python3 brep-bench/gate.py --case boolean/  # the cases whose id starts with this
python3 brep-bench/gate.py --update         # whole suite, then make it the new baseline
```

It needs `git`, `cargo` and Python 3.11 or later, and network access the first time: it fetches
brep-bench at the commit in `UPSTREAM` into `brep-bench/.upstream/` (not tracked), builds the
runner in `kernels/ngk` in release against this tree, runs every case with a 60 s limit
(`--timeout`) and compares each outcome (`pass`, `built`, `wrong`, `refused`, `crashed`) with
`results/ngk.json`. A full run takes about five minutes on two cores, most of it the build of NGK.

It ends with the baseline's tally and this run's, the cases that moved (now passing, changed
outcome, regressed) and the exit status: 1 when a case the baseline grades `pass` or `built` no
longer does, or is not reported. A change that moves cases forward runs `--update` and commits
the new `results/ngk.json` with its pull request, saying which cases moved. `--update` writes the
cases in id order with no timings or dates, so the same tree gives the same file; it refuses to
run over a regression unless `--accept-regressions` is given.

The gate runs in CI (`.github/workflows/brep-bench.yml`) on pull requests touching `src/`, the
manifests or `brep-bench/`, and on pushes to `master`.

### The reproduction suite

```
cargo test --test robustness                           # what passes today
cargo test --test robustness -- --ignored              # what is still open
cargo test --test robustness -- --include-ignored sweep::   # one area, open or not
cargo test --release --test robustness -- --include-ignored # everything, quickly
```

Without optimisation the Boolean reproductions are about a hundred times slower than with it, so
the open ones are best run with `--release` (the first build takes several minutes); the few
that pass with optimisation and take minutes without are ignored in debug builds only.

`tests/robustness/` holds one module per area of work: `tessellation` (A), `sweep` (B),
`edit_safety` (C), `intersection` (D, E), `splitter` (F) and `contacts` (G). Each test builds a
plain part and asserts the right answer: a closed-form volume (or a numeric integral of the
answer where there is no closed form), a solid that passes the kernel's own manifold and
orientation checks, a mesh that closes, or, where the right answer is a refusal, an error that is
not a panic. A test that fails today is `#[ignore]`d with the class it belongs to; the change
that fixes the class removes the attribute. A reproduction that panics or runs past
`ROBUSTNESS_TIMEOUT` seconds (120 by default) reports that as its failure and does not stop the
others. `tests/support/robust.rs` holds the helpers, among them `assert_closed_mesh`, the check
every tessellation must pass: its triangles, joined on exact coordinates, form a closed,
consistently oriented 2-manifold of positive signed volume within the mesh's sag of
`Solid::volume`.

`plan/robustness/probes/boolean_probe.rs` is the reduced Boolean reproductions the plan was
written from, as one runner case: drop it into `kernels/ngk/src/cases/boolean/` as
`zz_probe.rs` and run `PROBE=<name> kernels/ngk/target/release/brep-bench-ngk boolean/zz-probe
out.stl`.

## The runner and the baseline

[brep-bench](https://github.com/bmander/brep-bench) is a neutral conformance suite for B-rep
kernels: 231 cases described in words, each with its volume and area in closed form, graded the
same way for every kernel (volume and area within 1e-6, and the written triangles closed,
consistently oriented and bounding the expected volume). Results of every kernel:
https://bmander.github.io/brep-bench/

This folder holds NGK's runner and its implementation of every case, and the gate that runs the
suite on this tree.

- `kernels/ngk/`: the runner (`brep-bench-ngk CASE OUT.stl`), one file per case under
  `src/cases/<category>/<name>.rs`, shared helpers in `src/bench.rs`, and `workarounds.toml`, the
  cases built by working around a feature NGK lacks. It builds against this tree.
- `results/ngk.json`: the baseline the gate compares with. Update it (`--update`) in the change
  that moves cases forward.
- `UPSTREAM`: the brep-bench commit whose harness and cases are used. The gate fetches it into
  `.upstream/` (not tracked) and gives it a `kernels.json` naming NGK alone.

The work the gate tracks is planned in `plan/robustness/`.

## Current local run — 5 November 2026

The full 231-case gate on NGK 0.0.13 after the edit/operator corrections
reports **11 pass, 51 wrong, 169 refused and 0 crashed**. No case changed
outcome against `results/ngk.json`, and no passing case regressed. These
corrections secure editing; they do not raise the benchmark score yet.
Of the 51 wrong results, 43 fail only the mesh-closure check. The other
eight also fail area, volume or mesh-volume checks.

The local OCP 7.9.3.1 reference produces 215 meshes: 200 pass, 14 wrong
and one built for an unsettled case. Fifteen cases are refused and one
raises an OCCT construction error. The gallery lists all 231 cases and
shows those unavailable results explicitly. NGK produces 62 meshes,
including its 51 wrong results, which remain inspectable.

## Historical analysis — 4 November 2026

brep-bench (https://bmander.github.io/brep-bench/): 231 cases described in words, with closed-form volume and area, each kernel implementing them in its own idiom. A case **passes** if volume and area are within 1e-6 **and** the STL mesh it writes is closed (each edge used twice, once in each direction, vertices welded only when bit-identical) with a volume within 2%. A refusal ranks above a wrong answer.

Version tested: NGK 0.0.12. `results/ngk.json` was produced at commit `424b06c` plus uncommitted edits to `src/builders/boolean/contacts.rs` and `src/topology/unwrapped_face_domain.rs`, which are not part of that commit. Replaying all 231 cases through the runner on a clean `424b06c`, without the harness, every case builds, refuses or crashes as recorded, with the same volume and area (to 1e-9) and the same mesh closedness. Only 11 triangle counts and 3 refusal messages differ.

#### Result

| | pass | wrong | refused | crashed |
|---|---|---|---|---|
| NGK, official | **11** | 48 | 162 | 10 |
| NGK, if the mesh were watertight | **53** | 6 | 162 | 10 |

42 of the 48 "wrong" have an exact volume **and** area to 1e-6, and a mesh with the right volume: only the watertightness of the STL fails. The kernel computes correctly; the tessellator is what fails these cases.

For context (official passes out of 231): cad-kernel 204, kentron 183, next.BREP 153, brepkit 119, OCCT 91 (its runner does not implement everything), SolidCraft 77, arris 76, popkern 55, vcad 44, Aetheris 25, **NGK 11**. With a watertight mesh, NGK would be around 53, between popkern and vcad.

#### By category (passes)

| category | n | NGK | cad-kernel | kentron | next.BREP | OCCT | arris |
|---|---|---|---|---|---|---|---|
| primitives | 8 | 4 | 8 | 8 | 8 | 8 | 8 |
| boolean | 22 | 2 | 21 | 21 | 21 | 22 | 21 |
| tangent | 19 | 2 | 18 | 14 | 9 | 14 | 9 |
| singular | 7 | 0 | 0 | 0 | 0 | 7 | 0 |
| fillet | 12 | 0 | 9 | 11 | 11 | 7 | 5 |
| chamfer | 8 | 0 | 6 | 7 | 5 | 3 | 1 |
| sweep | 11 | 1 | 11 | 11 | 5 | 7 | 7 |
| loft | 6 | 0 | 6 | 6 | 4 | 0 | 0 |
| spline | 7 | 1 | 7 | 7 | 7 | 6 | 7 |
| nurbs | 6 | 0 | 5 | 5 | 6 | 4 | 6 |
| geometry | 20 | 1 | 20 | 16 | 18 | 0 | 0 |
| parts | 7 | 0 | 7 | 7 | 7 | 7 | 7 |
| feature | 4 | 0 | 4 | 3 | 0 | 0 | 0 |
| swept | 12 | 0 | 6 | 4 | 5 | 6 | 5 |
| draft / edit / offset / shell / thicken | 82 | 0 | 76 | 63 | 47 | 0 | 0 |

#### What blocks, by order of payoff

1. **Non-watertight tessellation (42 cases).** Two causes seen:
   - on a bounded curved face (cylinder wall, torus, cone…), the cell subdivision adds points on the boundary where its cut lines cross it: the cylinder wall has 256 points on the bottom circle, the neighbouring disc 128, offset by half a step;
   - boundary points are lifted from the pcurve rather than taken on the 3D edge: a corner of the `boxes-common` result comes out at `z = -2.2e-19` on one face and `0` on its neighbour.

   The fix: sample each edge once in 3D, share those points between its two faces, and never insert a point on a boundary edge. On its own, it would take NGK from 11 to about 53.
2. **Boolean imprint on degenerate contacts (32 refusals)**: `imprinting the target and tools[0] on each other failed`. This hits the whole `singular` category (a cut through a pole or an apex), almost all of the `tangent` category (tangent contacts, coincident faces, `cube-turned-a-hair`, `hairline-slot`), but also `napkin-ring`, both Steinmetz cases, `golf-ball` and `pocketed-tray`.
3. **Crashes (10).**
   - `ConflictingOwnership` (`model.rs:424`, "a committed model's labelling should describe its own map") as soon as a `faces::circle` face is swept, even in a straight line, and on the revolution of the `v-belt-pulley` triangle.
   - `frame.rs:82` "frame directions are not perpendicular" when sweeping along a NURBS edge.
4. **Revolution (11 refusals).** A curve that reaches the axis at one end is refused ("touches the revolution axis in an unsupported configuration", or "shell 0 has the wrong orientation" at measurement). Correction of 4 November: the B-rep of a full-turn revolution is correctly oriented whatever the winding of the section; the tessellator is what then flips the triangles.
5. **Fillets and chamfers (10 refusals)**: two fillets that meet (`slab-half-round`, `hole-waist`), fillet equal to the radius (`cylinder-rim-to-dome`), consumed faces (`l-block-inside-consumed`), cut corner plus edges in the same selection (`cube-all-edges`), 4-face vertex (octahedron), fillet on a closed circular edge meeting a corner (`boss-s-curve`).
6. **Accuracy bugs.**
   - An arc traversed backwards in a profile gives a wrong face via `faces::from_profile`: on `shelf-bracket`, the area is off by a whole disc. Worked around in the implementation by orienting the arc along the contour.
   - Loft realigns seams by proximity and ignores the given start vertex: `twisted-duct` is wrong. For `square-to-octagon` and `square-to-round`, loft silently builds a different solid instead of refusing.
   - Measurements over fitted intersection curves are only accurate to about 2e-6 relative (`sphere-lens`), 4e-5 on `thin-lens`.
7. **Missing features (102 refusals).** Shell, offset, thicken, draft, face moves (82 cases), face on a given NURBS surface, ellipsoid or affine stretch, loft with guides, sweep with a scale law, sweep on a closed spine, two-distance chamfer, variable-radius fillet, sweep of a solid (envelopes).

#### The runner

`kernels/ngk/` follows the same layout as the existing Rust runners: one file per case in `src/cases/<category>/<name>.rs`, a table generated by `build.rs`, shared helpers in `src/bench.rs`. Measurements come from `Solid::volume`/`area` (exact integration over the B-rep), validity from `validate_solid_manifold` + `validate_solid_orientation`, the STL from `Tessellate::tessellate_with` (128 segments per turn). No home-made welding: that is the bench's rule. 15 cases are built by workaround and listed in `workarounds.toml` (extrusion up to a face, sweep of a solid by elementary geometry, NURBS vault made by extruding a rational arc, etc.).

To run it again: copy `kernels/ngk` into a brep-bench clone, add the `kernels-entry.json` entry to `kernels.json`, point the `ngk` dependency of `kernels/ngk/Cargo.toml` at your tree (or at `git = "https://github.com/Jojain/ngk"` for a PR), then:

```
python3 harness/build.py
python3 harness/run.py --kernel ngk          # about 4 min, writes results/ngk.json
python3 harness/run.py --kernel ngk --case boolean/
```
