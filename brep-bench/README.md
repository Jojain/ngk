# brep-bench

## How to run

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

## NGK on brep-bench — status as of 4 November 2026

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
