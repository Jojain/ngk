# WS E: NURBS surface/surface robustness

**Owns:** `src/geometry/dim3/intersections/surface_surface/` (`seeds.rs`,
`tracer.rs`, `fitting.rs`, `normals.rs`). **Decision:** none.
**Bench:** `geometry/saddle-drum-bored`, `nurbs/groin-vault`, and every
Boolean on a lofted, swept or imported face.

## Symptoms

- **Two drilled holes crossing inside a block** (radius 3 along x, then
  radius 2 along y, axes meeting) and two crossing cylinders of radii 5 and 3
  fail with `TraceBudgetExhausted`. The same pair with the axes 1 apart works.
- A small ball on a cylinder's wall: `LoopFreedomNotCertified` and
  `SubdivisionBudgetExhausted` (WS D will route this pair away from NURBS,
  but the same configuration with a lofted or imported face stays here).
- A bilinear (saddle) face cut by a cylinder, the groin vault's two vaults:
  `SubdivisionBudgetExhausted`.

Cylinder × cylinder reaches this solver only because the analytic table lacks
it; WS D moves that pair out. The tracer and certification limits below stay
for every pair WS D does not cover: lofted, swept, revolved-spline and
imported faces.

## Root causes, to confirm and fix

1. **Tracer budget in absolute units.** `max_trace_step = 2e-2` and
   `max_trace_steps = 4096` (`IntersectionOptions::default`) are fixed
   numbers in parameter units, whatever the parameter ranges (a cylinder's
   height runs 0 to 30, its angle 0 to 2π). Crossing holes with meeting axes
   are symmetric: the curve passes where the step controller shrinks (the
   curve runs along a parameter-box boundary or near a seam), and the budget
   runs out before the loop closes. Step bounds should scale with each
   surface's parameter extent and the curve's curvature, and the budget with
   the curve's expected length.
2. **Loop certification under a fixed node budget.** `pair_seeds` subdivides
   until normal cones separate, sharing one 4096-node budget over every pair
   of Bézier spans. A rational patch's cone from its control net is loose, so
   close-to-parallel normals take many levels to separate. Options within the
   current design: tighter normal bounds (exact Gauss map bounds for rational
   patches), a budget per span pair, and seeding from critical points of the
   distance between the patches where cones overlap, which also gives the
   tangent points.
3. **Fit tolerance.** `fit_tolerance = sqrt(LINEAR_TOLERANCE)` ≈ 3e-5: the
   fitted section may stray 3e-5 from the true curve, which WS H sees as
   measure error and WS F as pcurves that miss each other near poles
   (`SynchronizedFitToleranceExceeded` in the pole cases).

## Tests

Crossing drilled holes with axes meeting, and offset by 1e-3, 1e-1, 1;
radii ratio 0.99, 0.6, 0.2. A small ball on a lofted wall. A saddle face
bored by a cylinder. The groin vault. Each at model scales 1e-2, 1, 1e3.
