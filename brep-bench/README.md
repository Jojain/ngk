# NGK on brep-bench — status as of 4 November 2026

brep-bench (https://bmander.github.io/brep-bench/): 231 cases described in words, with closed-form volume and area, each kernel implementing them in its own idiom. A case **passes** if volume and area are within 1e-6 **and** the STL mesh it writes is closed (each edge used twice, once in each direction, vertices welded only when bit-identical) with a volume within 2%. A refusal ranks above a wrong answer.

Version tested: NGK 0.0.12. `results/ngk.json` was produced at commit `424b06c` plus uncommitted edits to `src/builders/boolean/contacts.rs` and `src/topology/unwrapped_face_domain.rs`, which are not part of that commit. Replaying all 231 cases through the runner on a clean `424b06c`, without the harness, every case builds, refuses or crashes as recorded, with the same volume and area (to 1e-9) and the same mesh closedness. Only 11 triangle counts and 3 refusal messages differ.

## Result

| | pass | wrong | refused | crashed |
|---|---|---|---|---|
| NGK, official | **11** | 48 | 162 | 10 |
| NGK, if the mesh were watertight | **53** | 6 | 162 | 10 |

42 of the 48 "wrong" have an exact volume **and** area to 1e-6, and a mesh with the right volume: only the watertightness of the STL fails. The kernel computes correctly; the tessellator is what fails these cases.

For context (official passes out of 231): cad-kernel 204, kentron 183, next.BREP 153, brepkit 119, OCCT 91 (its runner does not implement everything), SolidCraft 77, arris 76, popkern 55, vcad 44, Aetheris 25, **NGK 11**. With a watertight mesh, NGK would be around 53, between popkern and vcad.

### By category (passes)

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

## What blocks, by order of payoff

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

## The runner

`kernels/ngk/` follows the same layout as the existing Rust runners: one file per case in `src/cases/<category>/<name>.rs`, a table generated by `build.rs`, shared helpers in `src/bench.rs`. Measurements come from `Solid::volume`/`area` (exact integration over the B-rep), validity from `validate_solid_manifold` + `validate_solid_orientation`, the STL from `Tessellate::tessellate_with` (128 segments per turn). No home-made welding: that is the bench's rule. 15 cases are built by workaround and listed in `workarounds.toml` (extrusion up to a face, sweep of a solid by elementary geometry, NURBS vault made by extruding a rational arc, etc.).

To run it again: copy `kernels/ngk` into a brep-bench clone, add the `kernels-entry.json` entry to `kernels.json`, point the `ngk` dependency of `kernels/ngk/Cargo.toml` at your tree (or at `git = "https://github.com/Jojain/ngk"` for a PR), then:

```
python3 harness/build.py
python3 harness/run.py --kernel ngk          # about 4 min, writes results/ngk.json
python3 harness/run.py --kernel ngk --case boolean/
```
