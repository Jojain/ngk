# Analytical curves and surfaces

## 1. Objective

Give the kernel an OCCT-comparable set of analytic supports — `Sphere`, `Cone`,
`Torus` surfaces and `Ellipse`, `Hyperbola`, `Parabola` curves in 2D and 3D — behind a
written contract, and close the paths where an unrecognized support silently degrades
instead of failing.

## 2. Why this work is required

Everything outside `Line`/`Circle`/`Plane`/`Cylinder`/`Ruled`/`Revolution` is approximated
today. A sphere is a `SurfaceOfRevolution` of an arc (`src/builders/solids.rs:44`); there is
no cone or torus at all; a plane section of a cylinder comes back as a sampled NURBS rather
than an ellipse.

That costs the kernel in three concrete places:

- Booleans lose their broad-phase cull on every non-planar, non-NURBS face —
  `src/builders/boolean/broad_phase.rs` returns "unbounded" for them;
- healing never fuses faces that are not plane/plane or cylinder/cylinder
  (`src/healing/predicates/surface.rs`);
- tessellation renders an unknown surface as one degenerate quad
  (`src/tessellate/face.rs`).

## 3. Scope

### Included

- Surfaces: `Sphere`, `Cone`, `Torus`.
- Curves: `Ellipse`, `Hyperbola`, `Parabola` — 3D (`Curve`) and 2D (`Curve2`).
- `CurveGeometry` / `Curve2Geometry` / `SurfaceGeometry` traits, with the enum
  forwarding written by hand.
- `domain()`, `bbox_over()`, box-honouring `to_nurbs_over()`, `ParamMap`,
  analytic `closest_parameter`.
- Integration: healing predicates, tessellation, boolean broad-phase and
  classification, periodic seam closure, viz, bindings.

### Initially excluded

- An analytic surface/surface intersection pair table (plane×sphere → circle, and so
  on). The extension point is named — `CURVE_3_RECOGNIZERS` in
  `src/geometry/dim3/intersections/surface_surface/simplification.rs` — but stays
  unfilled. This keeps the NURBS-first policy intact.
- Offset curves and surfaces, explicit Bezier variants, `RectangularTrimmedSurface`.
- Widening the boolean's "two validated closed solids" precondition.
- Chamfer support for the new surfaces; it stays plane/ruled-only.

## 4. The constraint that shapes the contract

The obvious contract to demand — *"`to_nurbs_over(u, v)` returns a NURBS whose parameter
domain equals the requested box in the analytic parameterization, so pcurves stay
valid"* — **is not achievable, and assuming it is would be the main way this work goes
wrong.**

A circle is not a rational function of its angle. `Circle::to_nurbs_between`
(`src/geometry/dim3/curves.rs`) builds the correct rational quadratic, but the Bézier
parameter of a rational quadratic is a **projective**, not linear, function of the angle.
For the unit quarter arc, evaluating the NURBS at parameter `π/8` lands at angle
`0.3769 rad`, not `0.3927` — the point is exactly on the circle, at the wrong parameter.
At radius 2.5 that is a ~0.04-unit positional discrepancy for the same parameter value.

The pre-existing tests did not see this because they sampled only `{0, π/4, π/2, π, τ}` —
every one a knot or a span midpoint, which are exactly the two families where the
projective map is the identity.

So the contract is:

> `to_nurbs_over` agrees with the analytic support **as a point set**, and matches the
> requested box at its **endpoints**. Interior parameter correspondence is a documented,
> monotone, closed-form reparameterization — never the identity for rational conic
> directions.

For a rational quadratic arc with weight `w` spanning angle `Δ` from `θ₀`:

```
tan((θ − θ₀)/2) = w·s·tan(Δ/2) / ((1 − s) + w·s)
```

closed-form and cheaply invertible. This is why `broad_phase.rs` says *"Analytical-to-NURBS
conversion may change the UV parameterization"* and gives up. `ParamMap` (M1) makes that map
explicit instead of a reason to give up.

`tests/geometry/dim3/curves.rs::circle_nurbs_conversion_stays_on_the_circle_between_knots`
pins the half of this that does hold — point-set exactness off-knot.

## 5. Trait design

`src/geometry/traits.rs` defines `CurveGeometry`, `Curve2Geometry` and `SurfaceGeometry`.
Each concrete type implements the trait; each enum keeps **inherent** methods that match
over its variants and delegate, and also implements the trait by forwarding to those
inherent methods.

Inherent methods shadow trait methods, so call sites keep working without importing the
trait, while generic code can still be written once over any support. The enums stay
concrete: `GMap` serialization, healing's value comparisons and cheap cloning all depend
on the derived `Serialize`/`Deserialize`/`Clone`/`PartialEq`.

`NurbsCurve2` deliberately does not implement `Curve2Geometry`: its own methods are
expressed in its native knot domain, and `Curve2` is normalized to `[0, 1]`. Making it an
implementor would give it two parameter conventions under one name, so the `Curve2::Nurbs`
arm does the remapping instead.

`domain()` returns possibly-infinite `Interval`s. `Interval::or_extent(extent)` substitutes
`±extent` for *infinite* endpoints only — a bounded domain keeps its real extent even when
wider than the window. (Clamping instead would have truncated a circle's debug view to
`[0, 1]`, since `DEBUG_EXTENT` is `1.0`.)

## 6. The six new types

Frame-based, OCCT-compatible parameterizations. `Circle` is **not** re-expressed as a
degenerate `Ellipse`: it keeps its closed-form `length` and its healing predicate.

| Type | Fields | `P(u,v)` / `P(t)` | Domain | Periodicity |
|---|---|---|---|---|
| `Sphere` | `frame`, `radius` | `O + R cos v (cos u X + sin u Y) + R sin v Z` | `u∈[0,τ)`, `v∈[−π/2,π/2]` | `UPeriodic(τ)` |
| `Cone` | `frame`, `ref_radius`, `half_angle` | `O + (R + v sin α)(cos u X + sin u Y) + v cos α Z` | `u∈[0,τ)`, `v∈ℝ` | `UPeriodic(τ)` |
| `Torus` | `frame`, `major`, `minor` | `O + (R + r cos v)(cos u X + sin u Y) + r sin v Z` | both `[0,τ)` | `UVPeriodic(τ,τ)` |
| `Ellipse` | `frame`, `major`, `minor` | `O + a cos t X + b sin t Y` | `[0,τ)` | `Periodic(τ)` |
| `Hyperbola` | `frame`, `a`, `b` | `O + a cosh t X + b sinh t Y` | `ℝ` | `None` |
| `Parabola` | `frame`, `focal` | `O + (t²/4f) X + t Y` | `ℝ` | `None` |

One shared conic-arc builder replaces three copy-pastes: every conic arc is the rational
quadratic through `P₀`, `P₂` with tangent intersection `P₁` and a known on-curve midpoint
`M`, with `w = |A − M| / |M − P₁|` where `A = (P₀+P₂)/2`. `Circle::to_nurbs_between`
generalizes into `conic_arc_nurbs` in a new `dim3/conics.rs`; `Circle`, `Ellipse` and
`Hyperbola` all call it. `Parabola` is **non-rational** — a parabola *is* a quadratic
polynomial, so it converts exactly with unit weights over any `[t₀, t₁]`.

`Ellipse::to_nurbs` needs no new weights: an ellipse is the affine image of a circle and
rational Bézier is affine-invariant, so build the circle net in-frame and scale
`(x, y) → (a·x, b·y)`.

`Torus::new` rejects `minor >= major` — the self-intersecting inner torus is not a valid
support.

## 7. Milestones

### Milestone 0 — trait refactor, no behaviour change ✅

- [x] `Interval::unbounded`, `is_finite`, `or_extent`.
- [x] `CurveGeometry`, `Curve2Geometry`, `SurfaceGeometry` in `src/geometry/traits.rs`.
- [x] Implemented for `Line`, `Circle`, `NurbsCurve`, `Bounded<Curve>`; `Line2`,
      `Circle2`; `Plane`, `Cylinder`, `RuledSurface`, `SurfaceOfRevolution`,
      `NurbsSurface`.
- [x] `rotated`/`translated` moved out of the enum matches into the concrete types.
- [x] `Curve::project` implemented for `Circle` and `NurbsCurve` — removes the two
      `todo!()`s that panicked.
- [x] `Curve::domain`, `Surface::domain`, `Surface::is_degenerate_at`.
- [x] `Debug` + `PartialEq` on `Surface`, `Cylinder`, `RuledSurface`,
      `SurfaceOfRevolution`, `NurbsSurface`, `ControlNet`.
- [x] `curve_interval`, `surface_intervals` (`src/viz/geometry.rs`) and
      `finite_curve_domain` (`src/tessellate/face.rs`) collapsed onto `domain()`.
- [x] `Surface::to_nurbs_over` and `Surface::closest_parameter` wildcards replaced by
      explicit per-variant arms.

### Milestone 1 — honest `to_nurbs_over` + `ParamMap` 🚧

- [x] `Cylinder::to_nurbs_over` spans the requested height. `Cylinder::point_at` moves
      `v` *units* along the axis while `to_nurbs` offset its second control-point row by
      one unit over knots `[0,0,1,1]`, so every cylindrical face taller than one unit
      silently lost the geometry above `v = 1` on entering the intersection engine.
      Regression test:
      `tests/geometry/dim3/surfaces.rs::cylinder_nurbs_patch_spans_the_requested_height_interval`.
- [ ] `ParamMap { u: Reparam, v: Reparam }` with `Reparam::{Identity, ConicArc}`.
- [ ] `bbox_over` on both traits, exact for the quadrics, conservative elsewhere.
- [ ] Wire both into `broad_phase.rs::face_bounds`, replacing its `None` fallback.
- [ ] `Curve::reversed` in closed form, so trimming and reversal stop degrading analytic
      curves to NURBS.

### Milestone 2 — `conic_arc_nurbs` + `Ellipse` (3D and 2D) 🚧

Establishes the full per-type checklist on the easiest type.

- [x] Shared 3D `conic_arc_nurbs`; trimmed circles and ellipses use it.
- [x] `Ellipse` / `Ellipse2` concrete types, enum and trait forwarding,
      transformations, exact NURBS conversion, bindings, and focused tests.
- [ ] Add the Milestone 1 `ParamMap` and `bbox_over` contracts, then cover those
      cross-cutting invariants for ellipse.

### Milestone 3 — `Sphere`, `Cone`, `Torus`

One per pass. Includes switching `builders/solids.rs::add_sphere` to `Surface::Sphere`
(which moves raw dart/face counts in existing sphere tests) and extending
`builders/faces.rs::periodic_boundary_curve`.

### Milestone 4 — `Hyperbola`, `Parabola` (3D and 2D)

Unbounded domains exercise `domain()`'s infinite intervals and `to_nurbs_over`'s box
honouring.

### Milestone 5 — cleanup

`boolean/graph.rs` analytic trim, `boolean/tolerance.rs` scale estimate, healing's
ellipse joiner, docs.

## 8. Test plan

Mirrors `tests/geometry/`. Per type, the existing triplet: point/normal against a
hand-computed expectation; `periodicity()`; `to_nurbs` agreement at sampled parameters with
asserted `degree_u`/`degree_v`/`is_rational`. Point equality uses
`PointCoincidence::coincides(_, LINEAR_TOLERANCE)`.

New files: `tests/geometry/dim3/conics.rs`, `sphere.rs`, `cone.rs`, `torus.rs`; extend
`tests/geometry/dim2.rs`.

Cross-cutting invariants for every analytic type — each sampling **off-knot,
off-midpoint** parameters, per §4:

1. `to_nurbs_over(box)` reproduces `point_at` over the box **after applying `param_map`**,
   and its knot domain matches the box at the endpoints;
2. `closest_parameter(point_at(u, v))` round-trips modulo periodicity;
3. `bbox_over(u, v)` contains a dense sample of the patch;
4. `rotated` / `translated` commute with `point_at`;
5. `domain().is_finite()` matches the type's mathematical extent;
6. `normal_at` at a pole or apex equals the meridian limit, and `is_degenerate_at` is true
   there.

## 9. Risks

- **Parameterization preservation (§4)** — the trap this plan exists to avoid. Mitigated by
  `ParamMap` and by the off-knot sampling rule in every test.
- **Pole/apex degeneracy** leaking into tessellation triangles and intersection seeding.
  Mitigated by `is_degenerate_at` and explicit collapse.
- **Cone apex in `closest_parameter`** — `u` is undefined there; pick `u = 0` and document.
- **Torus with `minor >= major`** — rejected at construction.
- **`add_sphere` raw-count churn** — existing sphere tests will move; update deliberately
  rather than rubber-stamping.
- **Serde is forward-incompatible** — new JSON cannot be read by older binaries. Acceptable
  under the break-freely policy, but worth stating.

## 10. Definition of done

Six new types implemented behind the three traits, every cross-cutting invariant green, the
cylinder truncation and the circle parameterization constraint covered by regression tests,
healing / tessellation / broad-phase handling the new supports, and bindings plus the
TypeScript unions updated.
