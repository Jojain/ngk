# WS D: analytic surface/surface intersection

**Owns:** `src/geometry/dim3/intersections/analytic/` and a new module for the
implicit/parametric engine; the dispatch in `surface_surface.rs`.
**Decision:** D2. **Bench:** about 16 cases, plus most real-part Booleans.

## Symptoms

`surface intersection coverage is incomplete` on pairs of analytic surfaces,
with `SubdivisionBudgetExhausted`, `LoopFreedomNotCertified`,
`TangentOrSingularContact`:

- coaxial cylinders of equal radius (`tangent/coaxial-overlap`);
- a sphere inside a cylinder of its own radius, a torus inside a drum, a ball
  in a cylinder's dish (`ball-in-band`, `torus-in-drum`, `knife-edge-dish`);
- two cylinders of equal radius crossing (Steinmetz), a cylinder touching
  another from inside (`tangent-tee`), a cylinder touching a sphere from
  inside (Viviani);
- a plane through a cone's apex, a plane bitangent to a torus (Villarceau);
- a small ball centred on a cylinder's wall (`cyl_sphere` probe): a closed
  loop inside both faces.

## Root cause

The closed-form table covers plane × {plane, sphere, cylinder} and
sphere × sphere only (`docs/architecture/geometry.md`). Every other pair is
converted to NURBS and handed to the subdivision solver, which certifies
absence of closed loops by disjoint normal cones (Sederberg–Meyers) under a
4096-node budget. Analytic pairs are where tangency, coincidence and
singular crossings happen by construction (coaxial, equal radii, apex, pole),
and there the normal cones never separate: the solver can only give up, and
says so honestly.

## The class

Every pair among plane, sphere, cylinder, cone and torus, in every relative
position: disjoint, transverse, tangent along a curve, tangent at a point,
crossing at a singular point, coincident, coaxial.

## Where it sits

The architecture stays "a general NURBS solver, and special cases for
analytic surfaces". What changes is the special-case layer: today four
pairs with a closed form, then straight to NURBS; proposed, three tiers.

1. Closed forms (unchanged, the fast path): pairs whose answer is a line or
   a conic, and coaxial surfaces of revolution.
2. **New:** any pair where at least one surface is analytic, by the method
   below. This covers every analytic × analytic pair and also analytic ×
   NURBS (a hole drilled into a lofted face, a plane through a swept wall),
   since only one implicit equation is needed.
3. The NURBS solver, for NURBS × NURBS only: there no usable implicit
   equation exists (a bicubic patch implicitises to degree 18), which is why
   that solver certifies by normal cones.

A worked example, cylinders along x (radius R) and along y (radius r): on
the second, S(θ, h) = (r cos θ, h, r sin θ), and the first's equation
y² + z² = R² gives g(θ, h) = h² + r² sin²θ − R². Its critical points are at
h = 0, θ ∈ {0, π/2, π, 3π/2}. For r < R (crossing drilled holes) g < 0 at
all of them: two loops, each wrapping θ, nothing tangent. For r = R
(Steinmetz) g = 0 at θ = ±π/2: two points on the curve where its branches
cross, found and classified, which is exactly where the NURBS solver gives
up today.

## Approach (D2)

**Recommended: one implicit/parametric engine.** Every analytic surface has
both an implicit equation F(x) = 0 (a quadric, or the torus quartic) and a
parameterisation S(u, v). For a pair, substitute S₂ into F₁:
g(u, v) = F₁(S₂(u, v)), a smooth function on S₂'s parameter rectangle whose
zero set is the intersection, in S₂'s own parameters. Then:

1. **Coincidence first.** Same surface (same axis and radius, same sphere,
   same plane) is an overlap, resolved exactly; g ≡ 0.
2. **Coaxial surfaces of revolution** (same axis): the intersection is
   circles at the roots of a one-variable equation in the axial parameter,
   exact, with tangency read from root multiplicity. This covers
   ball-in-band, torus-in-drum, the knife-edge dish, the coaxial cylinders.
3. **Topology in 2D.** Critical points of g (∇g = 0) on the rectangle are
   found by interval Newton on a 2×2 system; a critical point with g = 0 is a
   tangent or singular point of the curve, classified by the Hessian
   (isolated point, crossing, cusp). A loop in the interior of a region free
   of critical points cannot close (Morse theory), so seeds come from the
   boundary of the rectangle and from the critical points, and coverage is
   certified without normal cones.
4. **Trace in 2D** along g = 0 with the critical points as known stops, lift
   by S₂ (exact on S₂), and project onto S₁ for its pcurve, exact where S₁
   inverts in closed form (all five do).
5. **Closed forms stay** where the result is a conic or a line: plane ×
   quadric, sphere × sphere, coaxial pairs, cylinder rulings.

The alternative, more closed forms pair by pair, is cheaper per pair but
gives no general answer for tangency and singular points, which is where the
cases fail.

## How a section curve is stored (part of D2)

A cylinder × sphere section is a quartic: none of `Curve`'s kinds (line,
circle, ellipse, helix, NURBS) holds it exactly. Two options:

- **A fitted NURBS**, from the exact points, to a tight tolerance (1e-10
  relative). `Curve` is unchanged; more control points; and a later
  Boolean intersecting this curve again starts from the fit, not the truth.
- **A procedural intersection curve**: the two surfaces and a guide
  polyline, each point found by Newton projection onto both surfaces, as
  Parasolid and ACIS hold them. Exact, and no drift over chained
  operations; every consumer of `Curve` (measure, tessellation, rigid
  motions, STEP export through a B-spline) must handle the new kind.

## Tests

For each pair of the five supports: disjoint, transverse at two loops, one
loop entirely inside both patches, tangent along a curve (from inside, from
outside), tangent at one point, crossing at a singular point, coincident.
Check the curves against closed forms where they exist (Viviani, Steinmetz
ellipses, Villarceau circles), and run each through the four Booleans.
The bench's `tangent/` and `singular/` categories are the integration test.
