# Robustness: closing the gaps brep-bench exposed

**Status:** plan, nothing landed. Written 2026-11-04 from a full brep-bench run
(`brep-bench/`, 231 cases) and about thirty reduced reproductions built
alongside it. Missing features (shell, offset, thicken, draft, face move,
guided lofts, variable blends, solid sweeps) are **out of scope** here; this
plan is about operations NGK already offers that fail, crash, or answer wrong.

## Where NGK stands

| | pass | wrong | refused | crashed |
|---|---|---|---|---|
| brep-bench, NGK 0.0.12 at `424b06c` | 11 | 48 | 162 | 10 |

Of the 117 cases that do not pass and are not a missing feature, every one
falls in one of the classes below. The bench is only where the classes showed
up: each class was then reproduced on plainer parts, and those reproductions
are what the workstreams must fix, not the bench cases. Several are ordinary
modelling steps:

- revolving a disc into a torus (`MissingVertexPoint`);
- sweeping a circle along a line or an arc (panic);
- two drilled holes crossing inside a block (`TraceBudgetExhausted`);
- a groove turned into a shaft, or a ball cut by two parallel planes
  (two wrapping loops on one periodic face);
- a tube, or any part with a hole, standing on a face (coincident region
  with a hole);
- a block standing on a face that has a hole drilled in it;
- a small ball cut from a cylinder's wall (`LoopFreedomNotCertified`);
- a slot narrower than about 1e-4 of the part's size;
- a Boolean taking a previous Boolean's half ball as an operand;
- extruding or revolving a face whose arc edge is stored running against the
  loop (silently wrong volume).

## Workstreams

Each workstream owns a disjoint set of files, so they can run in parallel.
Where two must touch one file it is said in both.

| WS | Title | Owns | Bench cases | Needs decision |
|---|---|---|---|---|
| [0](ws0-bench-gate.md) | Bench and probe corpus as a regression gate | `brep-bench/`, CI | (all) | none |
| [A](wsA-tessellation.md) | Watertight, correctly wound tessellation | `src/tessellate/` | 43 | D1 |
| [B](wsB-sweep-family.md) | Extrude, revolve, sweep and loft correctness | `src/builders/{revolve,sweep,loft}/`, `src/builders/solids/` (extrude) | 25 | D6 |
| [C](wsC-edit-safety.md) | No panic on an open edit; post-conditions at commit | `src/model.rs`, `src/topology/edit*` | (enables B, F) | D5 |
| [D](wsD-analytic-ssi.md) | Analytic surface/surface intersection | `src/geometry/dim3/intersections/analytic/` (+ new module) | ~16 | D2 |
| [E](wsE-nurbs-ssi.md) | NURBS surface/surface robustness | `src/geometry/dim3/intersections/surface_surface/` | ~4 + crossing holes | none |
| [F](wsF-splitter-regions.md) | Face splitting: periodic faces, poles, bridges, coincident regions with holes | `src/builders/faces/imprints.rs`, `src/topology/unwrapped_face_domain.rs`, `src/topology/trim_domain.rs`, coincident-region code in `src/builders/boolean/graph.rs` | ~9 | D3, D4 |
| [G](wsG-contacts-tolerances.md) | Contacts and tolerances: graze, tangency, degenerate spans | `src/builders/boolean/{contacts,tolerance,clip,neighborhood}.rs` | ~6 | D4 |
| [H](wsH-measure-accuracy.md) | Measure accuracy on fitted pcurves (postponed) | `src/measure/`, pcurve fitting | 2 | D7 (postponed) |
| [J](wsJ-blend.md) | Blends that consume faces or meet marked edges | `src/builders/blend/` | 10 | none |

Recommended order of effort, by what each unlocks:

1. **0** first, and small: it is how every other workstream proves it fixed a
   class and broke nothing.
2. **A** alone moves the bench from 11 to about 53 passes, and every viewer,
   STL and downstream mesh user is affected by the same defect.
3. **C** and **B** in parallel: B's crash fixes are only safe once C makes a
   bad edit an error rather than a panic.
4. **D**, **E**, **F**, **G** in parallel: they are the Boolean, and most of
   the remaining refusals. D can start on topology and points before the one
   question D2 leaves open (how a section with no closed form is stored).
5. **J** whenever a hand is free. **H** waits for D7.

## Decisions to discuss before starting

These are the points where fixing a class means choosing an architecture,
not fixing a bug. Each workstream file states the options in full; this is
the short form.

- **D1. How a shape is meshed** (WS A). **Decided 2026-11-04:** the full
  robust design, no intermediate step. Edges are discretised once, in 3D,
  and every face uses those vertices; each face is triangulated by a
  constrained Delaunay triangulation in its parameters with exact
  predicates, refined in the surface's own metric to sag and angle bounds;
  the output is a closed, oriented 2-manifold or an error. Slab cuts and ear
  clipping go. See `wsA-tessellation.md`.
- **D2. How analytic surface pairs are intersected** (WS D). Options:
  (a) keep adding closed forms pair by pair; (b) one implicit/parametric
  engine for every pair of plane, sphere, cylinder, cone and torus: one
  surface's implicit equation on the other's parameters gives a curve
  g(u, v) = 0 in a plane, whose topology (loops, branches, tangent and
  singular points) is found from the critical points of g and then traced
  and lifted. Closed forms stay for the conic results and coaxial pairs.
  **Decided 2026-11-04**, as three tiers: closed forms (fast,
  exact, and the oracle the new tier is tested against), the implicit
  tier for every pair with an analytic surface (recognising conic results
  as such), the NURBS solver for NURBS × NURBS. Open: how a section with no
  closed form is stored (fitted NURBS or procedural curve).
- **D3.** Not a decision: NGK's specification already says a seam is only
  map scaffold, never an entity. What follows from it (the splitter reasons
  on the cylinder or torus with winding numbers) is implementation, in WS F.
- **D4. What a Boolean does when the true answer is not a manifold solid**
  (WS F, G): two solids touching along a line or at a point, a cut leaving a
  pinched vertex (`tangent/viviani-cut`, `tangent/villarceau-half-torus`).
  Options: refuse with a named error; or represent the non-manifold result.
  **Decided 2026-11-04:** refuse by name, and make sure that cut and
  intersection of tangent operands, whose answer *is* manifold, succeed.
- **D5. How reads during an open edit stay valid** (WS C). Builders read
  `Face::loops` and the like in the middle of a transaction, and the
  embedding index they need panics when the labelling is half updated: the
  index assumes constraints that are only checked at commit. Options, by
  analogy with a database: (a) fallible reads; (b) immediate constraints,
  the edit primitive refusing the conflicting label at the step that makes
  it, with the index kept up to date on each write; (c) reads only from the
  transaction-start snapshot, as the blend engine's plan-then-execute
  already does; (d) savepoints, each stage of a staged operation (the
  Boolean) validated so the next one reads a valid model. **Decided
  2026-11-04**, as the one rule that holds for every builder to come: (b) built as a small set of
  invariant-preserving operators (in the spirit of Euler operators), each
  leaving the model valid, the index maintained on each write; a deferred
  block for surgery that must pass through invalid states, with no views
  inside it; (a) as the last safety net. See `wsC-edit-safety.md`.
- **D6. How a loft pairs its sections** (WS B). **Decided 2026-11-04:**
  where an operation has several valid results, the choice is an explicit
  option. The loft exposes how sections are paired (as given, aligned
  automatically, or by an explicit correspondence, vertex to vertex or
  vertex to edge) and never changes a pairing it was handed. The same rule
  applies to every builder with more than one valid answer.
- **D7. How exact a measure must be on fitted pcurves** (WS H).
  **Postponed 2026-11-04**, to study later. Two ways, both measured or read
  in the source: tighter pcurves everywhere, as OCCT (its measure reads the
  stored pcurves, `BRepGProp_Face::Load`), which here costs ×5 to ×10 on
  curved Booleans as long as the measure splits at every knot; or, in the
  measure only, the exact inverse of an analytic surface, as cad-kernel.
  WS H waits for this decision.

## Rules every workstream follows

- **A class, not a case.** Each workstream starts by writing its family of
  reproductions as tests (the files list them), red first, and is done when
  the family is green, not when the bench case is.
- **No panic is ever a result.** A reproduction that panics becomes a test
  asserting an error first; the fix then turns the error into the right
  answer.
- **A wrong answer is worse than a refusal.** Where a fix is not in reach,
  the operation must at least detect the case and refuse it by name (several
  classes below currently return a solid that is wrong).
- **The bench gate (WS 0) stays green**: no previously passing case may
  regress, and each workstream records the cases it moved.
- Architecture documents change in the same commit as the code they
  describe; a decision taken from the list above gets an ADR.

## Case map

Which class each failing bench case belongs to, from the run of 2026-11-04.
Refusal reasons now carry their whole error chain, so a Boolean's
"imprinting failed" says why.

| Class | Workstream | Cases |
|---|---|---|
| Mesh not watertight, measures exact | A | 43: `boolean/{box-minus-through-cylinder, boxes-common, boxes-cut, boxes-union, drilled-plate, half-torus, hemisphere, indexed-pattern, offset-hole-sphere, sphere-pocket, spheres-union, stepped-shaft, tube, tube-flush-ends}`, `chamfer/cube-all-edges-mitred`, `feature/{hole-through-the-upper-arm, pad-down-to-cylinder}`, `fillet/{ball-and-post, box-all-edges, box-one-edge, cylinder-rim, rounded-block-top-rim, sliver-strip}`, `geometry/{elliptic-cylinder, elliptic-hole, hyperboloid, paraboloid, periodic-bspline-prism, periodic-bspline-ring, saddle-drum}`, `nurbs/vault-tangent-plane`, `parts/{flange, pin-lug, shelf-bracket, turned-shaft}`, `primitives/{cone-frustum, cylinder, revolved-sector, revolved-washer}`, `sweep/duct-elbow`, `swept/box-slid-aslant`, `tangent/{grazing-cylinders, ice-cream-cone}` |
| Intersection coverage incomplete, analytic pair | D | `boolean/steinmetz-{common,union}`, `singular/{cone-cut-aslant-through-its-apex, cone-halved-through-its-apex, sectors-sharing-an-apex}`, `tangent/{ball-in-band, coaxial-overlap, knife-edge-dish, tangent-tee-common, tangent-tee-union, torus-in-drum, villarceau-half-torus, viviani-common, viviani-cut, viviani-union}` |
| Intersection coverage incomplete, fitted sections through poles | D, E | `singular/{ball-cut-aslant-through-a-pole, balls-meeting-through-a-pole, napkin-ring-poles-on-its-rims}` |
| Intersection coverage incomplete, NURBS operand | E | `geometry/saddle-drum-bored`, `nurbs/groin-vault` |
| Imprint not realized: wrapping loops, poles | F | `boolean/{napkin-ring, golf-ball}`, `singular/ball-halved-through-its-poles` |
| Imprint not realized: bridge between two loops | F | `feature/rib-revolved-to-a-stop`, `swept/{torus-slid-through-rod, two-beads-slid-through-rod}` |
| Coincident region with a hole | F | `parts/pocketed-tray` |
| Imprint not realized: tangent contact | G | `boolean/half-cylinder-boss-tangent`, `tangent/midsphere-common` |
| Imprint not realized: features under ~1e-4 of the size | G | `tangent/{cube-turned-a-hair, hairline-slot}` |
| Point contact read as two solids | G | `tangent/cone-tip-on-face` |
| Degenerate span on a Boolean's result | G | `feature/post-up-to-dome` |
| Unmarked edge in a builder, panic mid-edit | B, C | `parts/v-belt-pulley`, `sweep/{circle-along-arc, helical-pipe, helical-pipe-round-ends, mitred-pipe-on-cube-edges}`, `swept/{ball-along-spline-groove, ball-thread-groove, ball-swung-groove}`, `geometry/{hyperboloid-ruled, horn-torus}` |
| Sweep frame along a NURBS spine panics | B | `spline/{pipe-on-planar-cubic, pipe-on-twisted-cubic, square-on-planar-cubic}` |
| Revolve: meridian meeting the axis | B | `geometry/{hyperboloid-of-two-sheets, lemon, nurbs-cone, nurbs-sphere, oblate-spheroid, spindle-torus}`, `spline/{rational-cap, rational-spheroid, revolved-cubic-dome}` |
| Loft correspondence | B | `loft/{square-to-octagon, square-to-round}`, `sweep/twisted-duct` |
| Measure accuracy on fitted pcurves | H | `boolean/sphere-lens`, `tangent/thin-lens` |
| Blend consumes a face, meets a marked edge, or a 4-valent vertex | J | `chamfer/{block-ridge, cube-all-edges, cylinder-rim-to-point, l-block-inside-consumed, octahedron-all-edges-mitred}`, `fillet/{boss-s-curve, cylinder-rim-to-dome, hole-waist, l-block-inside-consumed, slab-half-round}` |
