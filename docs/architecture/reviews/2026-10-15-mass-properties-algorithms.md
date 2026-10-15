# Area and volume algorithms: ngk, OCCT, SMLib, literature

> **Status: accepted and built.** Option B was chosen; see [ADR 0001](../../adr/0001-mass-properties-by-green-boundary-integral.md) and [measure.md](../measure.md).

A survey to decide how `src/measure/` should integrate. It covers what ngk
does now, what OCCT and SMLib (NVIDIA USD BRep, open-sourced October 2026)
do, and what the literature offers. The last section lists the options.

## Summary

| | ngk today | OCCT `BRepGProp` | SMLib (USD BRep) |
|---|---|---|---|
| Area / volume | tessellate at a fixed 64×32 resolution, sum triangles / tetrahedra | Green's theorem on each face's pcurves, two nested Gauss–Legendre rules | Green's theorem on each face's trims, two nested adaptive Runge–Kutta integrations |
| Volume integrand | signed tetrahedra | `(P−O)·N / 3` | `z · N_z` |
| Error control | none (fixed grid) | fixed order by default; with `Eps`: two Gauss orders compared, intervals bisected, achieved error returned | requested relative accuracy, clamped to `[1e-4, 1e-1]` (volume `[1e-8, 1e-1]`) |
| Curve length | adaptive Simpson per knot span | Gauss–Legendre, order from curve type | Romberg per segment |
| Mesh alternative | — (it *is* the mesh) | `UseTriangulation` → `BRepGProp_MeshProps` | `ComputeProperties` on the tessellation, recommended for previews |

**Measured on the four reference primitives** (relative error against closed
forms; time per solid, release build; OCCT timings include Python call
overhead):

| solid | ngk V | ngk A | ngk Izz | ngk time (V) | OCCT V / A (default) | OCCT time (V) |
|---|---|---|---|---|---|---|
| block 2×3×4 | exact | exact | exact | 0.21 ms | 1e-16 | 0.04 ms |
| cylinder r1 h2 | −1.2e-3 | −6.0e-4 | −2.2e-3 | 0.71 ms | 1e-16 | 0.04 ms |
| sphere r1 | −4.0e-3 | −2.0e-3 | −7.2e-3 | 0.94 ms | 2e-16 | 0.04 ms |
| torus R3 r1 | −8.0e-3 | −2.6e-3 | −1.0e-2 | 0.97 ms | 0 | 0.04 ms |

OCCT reaches machine precision on every primitive and is about 20 times
faster. That is not because of special-casing for analytic surfaces. A Green
boundary integral with Gauss–Legendre converges **spectrally** on smooth
integrands, while a flat-triangle mesh converges as O(h²). A NumPy version of
the same scheme on a whole torus face (no OCCT involved) shows the gap:

| Gauss points per direction | total evaluations | area error | volume error |
|---|---|---|---|
| 4 | 32 | 3.6e-4 | 4.0e-2 |
| 8 | 128 | 2.9e-11 | 1.3e-6 |
| 12 | 288 | 0 | 4.8e-13 |
| 16 | 512 | 2.4e-16 | 1.7e-15 |

ngk lays a 64×32 grid (about 2 000 surface evaluations, plus boundary
samples) on each curved face to reach 1e-3. The
boundary integral reaches 1e-15 with 512.

**Recommendation:** Option B below. Use a Green-theorem boundary integral with
Gauss–Legendre (fixed order for validation, Gauss–Kronrod adaptive for public
mass properties), split at pcurve and surface knots. This is OCCT's scheme.
Gunderman–Weiss–Evans (2021) formalized it, and it fits `UnwrappedFaceDomain`
directly.

---

## 1. ngk today (`src/measure/integrate.rs`)

**Pipeline.** Every surface and volume measure goes through the display
tessellator, run with a fixed `COMPUTATION_TESSELLATION` (64 curve segments,
64×32 surface grid):

- **Area, area moments.** `tessellate_face` → `triangle_surface_moments`, which
  gives exact first and second moments of each *flat* triangle.
- **Volume, volume moments.** Signed tetrahedra from a reference point
  (`triangle_volume_moments`), summed over every face of every shell. The
  outer shell must come out positive and cavities negative, otherwise
  `InvalidOrientation`.
- **Planar shortcut** (`face_signed_volume`, validation only). A planar face
  contributes `height · area / 3`, with the area read off its boundary polygon
  (`planar_mesh_area`) without meshing. It is exact for straight edges and
  chordal for curved ones.
- **Validation fallback** (`fallback_face_signed_volume`). When the mesher
  refuses a face, the old sampled pcurve fan (32 samples per edge, 24×24 grid
  for boundaryless faces) supplies a sign. Public properties report the
  tessellation error instead.
- **Length.** Exact per curve type. NURBS curves use adaptive Simpson per knot
  span (`NurbsCurve::length`). `linear_properties_for_edge` takes the centroid
  and inertia from a 64-segment polyline rescaled to the exact length.

**Error sources.**

1. Chordal error of flat triangles: O(h²), where h is fixed by the grid, not by
   a tolerance. The sphere and torus errors above are this.
2. Boundary chords. Curved edges are polylines, so the trim region itself is
   approximated. This affects the planar shortcut and the cylinder caps.
3. Volume error is about twice the area error, and inertia about twice again,
   as expected for O(h²) on higher moments.

**Other properties.**

- Cost is fixed at about 2 000 grid evaluations, plus ear-clipping, welding
  and hashing per curved face, regardless of how simple the face is.
- It is not total. Any `TessellateError` (unreadable boundary, untriangulable
  polygon) becomes a `MeasureError`, and the validation path falls back to a
  cruder estimate.
- Two internal consumers depend on it: `validate_oriented_shell_volume` (sign
  only) and Boolean shell assembly (`assemble.rs::signed_volume`, which uses
  the magnitude to classify components). Both run on every Boolean, so speed
  matters as much as accuracy.
- `plan/mass_properties.md` is partly stale. It predates `src/measure/`, but
  its API design (three bundles, branded `Inertia`, no density) is what was
  built.

## 2. OCCT (`BRepGProp`, read from V7_9_0 sources)

**Reduction.** Volume becomes a surface integral by the divergence theorem,
and each face's surface integral becomes a line integral over its pcurves by
Green's theorem.

```
∬_D f(u,v) du dv  =  ∮_∂D F(u,v) dv,    F(u,v) = ∫_{u0}^{u} f(s,v) ds
```

`u0` is the surface's lower u bound. `BRepGProp_Gauss::Compute` evaluates
this literally:

- **outer loop:** over each edge's pcurve `l ↦ (u(l), v(l))`, Gauss points in
  `l`, weighted by `dv/dl`;
- **inner loop:** Gauss points in `u` from `u0` to `u(l)` at fixed `v`, each
  evaluating `S(u,v)` and the unnormalized normal `S_u × S_v`.

**Integrands** (`computeSInertiaOfElementaryPart` / `…VInertia…`):

- area: `|S_u × S_v|`, with moments `x·|N|`, `x²·|N|`, and so on;
- volume relative to a point O: `(P−O)·N / 3`, first moments `¼ x (P−O)·N`,
  and so on;
- volume relative to a plane: `((P−O)·n̂)(N·n̂)`, used to measure open shells
  against a plane.

**Orders** (`BRepGProp_Face::{U,V}IntegrationOrder`, `IntegrationOrder`):

| | points |
|---|---|
| plane | 8 |
| other analytic surface | 18 |
| Bézier / B-spline surface | `max(8, 2·max(4, (deg+1)·spans))` |
| line pcurve | 2 (raised to the surface order) |
| conic pcurve | 9 |
| B-spline pcurve | degree- and span-based |

There is no special case per analytic type. Every surface is treated as
"evaluate `S`, `S_u`, `S_v`".

**Adaptive mode** (`Eps < 0.001`):

- run each rule at two orders (`n` and `ceil(ratio·n)`); their difference is
  the error estimate;
- split u and l intervals at the surface and pcurve knots, then bisect
  sub-intervals (up to `SUBS_POWER`) until the estimate is under `Eps`;
- return the achieved relative error.

`VolumePropertiesGK` is a separate entry point using Gauss–Kronrod
(`math_KronrodSingleIntegration`) through `BRepGProp_TFunction` /
`UFunction`.

**Other entry points.**

- `LinearProperties`: per-edge Gauss–Legendre on the 3D curve
  (`BRepGProp_Cinert`).
- `UseTriangulation = true` (≥ 7.6): integrates the stored `Poly_Triangulation`
  instead (`BRepGProp_MeshProps`). It is also used automatically when a face
  has a mesh but no surface.

**Weak points.**

- The inner integral starts at `u0`, the surface's bound. On a large surface
  trimmed to a small face, the inner range is much longer than the face. The
  integrand is still smooth, but more points are wasted.
- It depends on pcurve fidelity. A pcurve that misses its 3D edge by δ moves
  the area by about δ times the edge length.

## 3. SMLib / NVIDIA USD BRep

Read from `NVIDIA-Omniverse/usd-brep`, `source/SMLib`, Apache-2.0 (released
2026-10).

**`SmFace::ComputePreciseProperties`** (`SmFace.cpp`) uses the same Green
reduction as OCCT with different machinery:

- **Trims.** `CreateOrientedTrimBoundary` converts every trim to an oriented
  **B-spline** UV curve first.
- **Outer integral.** Along each trim, as an ODE `dY/dt = −u'(t)·G(t)`,
  integrated by adaptive Runge–Kutta Cash–Karp (Numerical Recipes `rkqs`),
  with tolerance `accuracy/10`. Retries use smaller initial steps
  (`span/2 … span/4096`).
- **Inner integral.** `G(t)` is itself an RK ODE along the v-isocurve from
  `v_min` to `v(t)` at `u = u(t)`.
- **Integrated quantities.** Up to 27 at once (area, volume, first and second
  moments, products). Selection flags skip unneeded groups. Component 1 is
  always the area, and it drives the step control, scaled by an estimated
  total area (bounding-box based if not given).
- **Volume field.** `F = (0, 0, z)`, so `dV = z · N_z dA` (moments use `z/2`,
  `z²/3`), not OCCT's symmetric `r/3`.
- **Thin-face mode.** A `dFaceThickness > 0` treats a face as a thin solid
  between two offsets.

**Other paths.**

- `SmFace::ComputeProperties` / `SmRegion::ComputeProperties`: the same
  quantities summed over the tessellation.
- The API docs recommend this tessellation path for interactive previews
  ("~1e-1 is much cheaper than 1e-3").
- Curve length and untrimmed surface area use Romberg (`SmIntegrator`).

**Assessment.**

- Robust and battle-tested. It has run in commercial CAD (ClassCAD) for about
  20 years.
- Numerically it is a weaker choice than OCCT's. An explicit RK scheme
  integrating a pure quadrature has low order (5), so it needs many steps.
- The accuracy envelope it can promise is narrow: area and mass properties
  clamped at `1e-4`.
- Not something to copy directly. Two ideas are worth keeping: up-front
  conversion of trims to B-splines, so every pcurve has knots to split at, and
  the property-selection mask.

## 4. Literature

### A. Mesh-based (what ngk does)

- **Polyhedral mass properties:** Lien & Kajiya 1984 (symbolic, nonconvex
  polyhedra); Messner & Taylor 1980 (ACM Alg. 550); **Mirtich 1996** "Fast and
  accurate computation of polyhedral mass properties" (projection plus Green);
  Eberly's revisited formulas. Exact for the mesh, O(h²) for the surface.
- **Cheap upgrade: Richardson extrapolation.** Measure at h and h/2, then
  `(4·M(h/2) − M(h))/3` gives O(h⁴) when the mesh refines uniformly. ngk's
  mesher is not uniform-by-construction (adaptive splitting, boundary
  sampling), so the gain is not guaranteed.

### B. Boundary reduction plus Gaussian quadrature (OCCT's family)

- **Timmer & Stern 1980**, "Computation of global geometric properties of
  solid objects" (CAD 12). Divergence theorem plus Gauss on surface patches,
  the origin of the scheme.
- **Lee & Requicha 1982**, "Algorithms for computing the volume and other
  integral properties of solids" (CACM 25). The classic survey: boundary
  integration versus decomposition (cell, octree, ray casting, Monte Carlo).
- **Sommariva & Vianello 2009**, "Gauss–Green cubature and moment computation
  over arbitrary geometries" (JCAM). Green plus Gauss–Legendre over
  spline-bounded planar domains ("SplineGauss"), exact for polynomials of
  degree 2n−1. Follow-ups:
  - ChebfunGauss (2011);
  - NURBS-bounded domains;
  - positive-interior rules (Dolomites Res. Notes 2021);
  - a 2026 comparison of four constructions, arXiv
    [2607.10485](https://arxiv.org/abs/2607.10485).
- **Gunderman, Weiss & Evans 2021**: "Spectral mesh-free quadrature for planar
  regions bounded by rational parametric curves" (CAD 130), and **"High-accuracy
  mesh-free quadrature for trimmed parametric surfaces and volumes"** (CAD 141,
  [arXiv 2101.06497](https://arxiv.org/abs/2101.06497)).
  - The modern formulation of the OCCT scheme through the generalized Stokes
    theorem.
  - Spectral convergence for polynomial or rational trims.
  - Beats adaptive octree and linear or quadratic meshing per quadrature
    point.
  - Gives the practical rules: split at trim knots, choose the antiderivative
    reference line well, use a per-Bézier-segment order.
  - **The closest match to what ngk needs.**

### C. Exact (closed-form) integration

- **Gonzalez-Ochoa, McCammon & Peters 1997/98**, "Computing moments of
  objects enclosed by piecewise polynomial surfaces" (Purdue TR 97-009,
  ACM TOG).
  - With polynomial (non-rational) patches, `P·(P_u × P_v)` is a polynomial,
    so volume and its moments have exact Bernstein-form closed forms.
  - Area never does, because of the square root in `|P_u × P_v|`.
- **Consequence that matters for ngk.** *Volume* integrands are polynomial on
  polynomial patches, so Gauss–Legendre with enough points is **exact** for
  them. *Area* integrands are only smooth. For rational patches (all conics
  and quadrics as NURBS) neither is polynomial, but both are analytic, so
  convergence is still spectral.
- **Per-type closed forms.** Plane, cylinder, cone, sphere and torus have
  analytic inner integrals, so the face integral reduces to one 1D integral
  along the pcurves. Fully exact when the pcurves are lines (most faces from
  primitives and extrusions). This is an optimization, not a method.

### D. Quadrature on trimmed elements (isogeometric analysis)

The same problem one level down: integrating over a trimmed knot span.

- Kim, Seo & Youn 2009 (trimmed surface quadrature); Schmidt, Wüchner &
  Bletzinger 2012 "Isogeometric analysis of trimmed NURBS geometries"; Nagy &
  Benson 2015 "On the numerical integration of trimmed isogeometric elements"
  (CMAME).
- Moment fitting gives interior nodes with positive weights.
- **Marussig & Hughes 2018**, "A review of trimming in isogeometric analysis"
  (ICES 17-03 / ARCME). Survey of local reconstruction, approximate and exact
  treatment.
- Reduced and optimal spline quadrature uses fewer points across smooth knot
  spans: Hiemstra, Calabrò, Schillinger & Hughes 2017; Johannessen 2017.
  Patch-wise trimmed rules: arXiv
  [2306.14521](https://arxiv.org/abs/2306.14521) (2023).

These matter only if ngk ever integrates over the face *interior* (option C
below). The boundary approach sidesteps trimmed cells entirely.

### E. One-dimensional rules (the inner building block)

- **Gauss–Legendre:** optimal for smooth integrands, no error estimate.
- **Gauss–Kronrod (7–15, 10–21):** QUADPACK, Piessens et al. 1983. Nested
  points give an error estimate for free, which is what a `Fidelity`
  (`Bounded(err)`) needs.
- **Clenshaw–Curtis:** nested, nearly as accurate.
- **Tanh-sinh (double-exponential):** robust to endpoint singularities. Not
  needed here: sphere poles and cone apexes make `|N| → 0` smoothly, and
  Gauss never samples endpoints.
- **Romberg** (SMLib) and **adaptive Simpson** (ngk's NURBS length) are both
  far less efficient on smooth integrands.

### F. Volumetric and decomposition methods

- Cell enumeration, octrees, ray casting and Monte Carlo (Lee & Requicha
  1982): robust to bad topology, but low accuracy and slow convergence.
- **Generalized winding numbers** for trimmed NURBS (Spainhour, Gunderman &
  Weiss 2025, [arXiv 2504.11435](https://arxiv.org/abs/2504.11435)) are the
  robust containment counterpart.
- Implicit-domain quadrature (Saye 2015, SISC) assumes a level-set
  description and does not fit a B-rep.
- Useful only as a fallback for non-watertight input. ngk refuses such input
  at commit anyway.

### G. Commercial kernels (documentation only, no source)

- Parasolid (`PK_TOPOL_eval_mass_props`) and ACIS (`api_ent_area` /
  `api_body_mass_props`) both take a requested relative accuracy and report
  the achieved one.
- That is the same contract as OCCT's `Eps` path, and the reason
  `plan/mass_properties.md` anticipates a `Fidelity` beside each scalar.

## 5. What matters specifically in ngk

- **`UnwrappedFaceDomain` already does the hard part.** It makes each loop a
  continuous UV curve, including across periodic seams, which is exactly what
  the Green line integral needs. Seam and bridge edges are run twice in
  opposite directions, so their contributions cancel. Nothing special is
  needed.
- **Faces with no enclosing loop** (whole sphere, torus) integrate directly
  over the surface's parameter rectangle with a tensor Gauss rule, minus any
  hole loops. ngk's tessellator already branches on the same condition.
- **Reference line.** Take `u0` (or `v0`) at the face's own UV bounding box,
  not the surface's domain. That avoids OCCT's long inner ranges and works
  for unbounded planes and cylinders.
- **Which direction to antidifferentiate.** Pick the one with fewer knot
  spans or the shorter extent. For a revolved or extruded face, integrate
  along the direction that is analytic.
- **Field choice.** OCCT's `r/3` is symmetric. Center it at the shell's
  bounding-box center to avoid cancellation on far-from-origin parts. That
  matters more for the Boolean `assemble.rs` sign test than for public
  numbers.
- **Totality.** No polygon, no ear-clipping, no welding. The method needs only
  closed, correctly oriented pcurve loops and surface derivatives. Every
  `TessellateError` case where the boundary is readable becomes measurable.
  An unreadable boundary is still a named error, which keeps the "refuse
  rather than approximate" rule.
- **Pcurve fidelity becomes the error floor.** After a Boolean, intersection
  pcurves are approximations. The tessellator already depends on them
  equally, so this is not a regression, but it bounds what "exact" can mean.
  Document it.
- **Validation speed.** At a fixed low order (for example 8×8 per span),
  `signed_shell_volume` drops from meshing every curved face to a few hundred
  evaluations per face. The sign needs no precision.

## 6. Options

| | approach | accuracy | cost to build | risk |
|---|---|---|---|---|
| **A** | Keep tessellation; add a tolerance-driven resolution and Richardson extrapolation | O(h²) → maybe O(h⁴) | small | still not total, still slow, extrapolation not guaranteed on non-uniform meshes |
| **B** | **Green boundary integral + Gauss–Legendre / Gauss–Kronrod** (OCCT / Gunderman) | spectral; exact volume on polynomial patches | medium: one module, reuses `UnwrappedFaceDomain`, surface derivatives, pcurves | pcurve fidelity; needs tests on Boolean output and imported STEP |
| **C** | Reuse the mesher's UV triangulation, but integrate the *true* surface integrand on each UV triangle (Dunavant / Gauss rules) | interior exact-ish; boundary still chordal unless boundary cells are curved | small–medium | inherits every tessellator failure; boundary error remains |
| **D** | B plus closed-form inner integrals per analytic surface | exact on primitives with line pcurves | B + medium | special cases to maintain; only a speed-up over B |
| **E** | SMLib-style nested adaptive Runge–Kutta | to about 1e-8 | medium | slower than B for the same accuracy; no reason to prefer it |

**Suggested path:** B, built in two layers.

1. **Fixed-order Gauss–Legendre.**
   - Orders from surface and pcurve degree, OCCT-style.
   - Split at pcurve knots, then at surface knots along the inner direction.
   - Replaces the tessellation for validation and Boolean assembly.
   - Deletes the pcurve-fan fallback and the planar shortcut.
2. **Adaptive Gauss–Kronrod 7–15.**
   - Uses the same integrand closures, with per-interval bisection to a
     requested relative tolerance.
   - Serves the public `*_properties()` and returns the achieved error, which
     is the `Fidelity` hook.

Edge length and linear moments move to the same 1D Gauss–Kronrod rule on the
3D curve, which replaces both adaptive Simpson and the rescaled polyline.

D can follow if profiling shows that analytic faces dominate. A could remain
only as an opt-in `from_mesh` path, the way OCCT and SMLib both keep one.

### Questions for the decision

1. Should the public API take a tolerance now (`volume_properties_with(tol)`)
   and return the achieved error? Or keep the current signatures with a
   default 1e-10 and add `Fidelity` later?
2. Is it acceptable to replace the measurement under
   `validate_oriented_shell_volume` and `assemble.rs` in the same change? They
   then share the fixed-order path, and Boolean results depend on it.
3. Should the tessellation measure be kept as an explicit, opt-in mesh-based
   path, or deleted?

## Reproducing the numbers

- ngk: `cargo run --release --example measure_accuracy_probe`.
- OCCT and the NumPy Green+Gauss demo come from a throwaway script run with
  the repo's `.venv` (`OCP.BRepGProp`). It is not committed. Its OCCT half is
  the `BRepGProp.VolumeProperties_s` / `SurfaceProperties_s` calls on
  `BRepPrimAPI_Make{Box,Cylinder,Sphere,Torus}`.
