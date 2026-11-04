# WS H: measure accuracy on fitted pcurves

**Status:** deferred (D7 to be studied later).
**Status: postponed** (D7, 2026-11-04).

**Owns:** `src/measure/`, and the pcurve fitting it relies on.
**Decision:** D7. **Bench:** `boolean/sphere-lens`, `tangent/thin-lens`.

## Symptom

Two balls' lens: the section is an exact circle (sphere × sphere is in the
closed-form table), but its pcurve on each sphere is a fitted spline (the
circle is not an isoline there), and the area comes out 2.5e-6 relative off;
the thin lens 4e-5 off. `measure.md` says so: "where they are fitted … a
pcurve that strays δ from its edge moves the area by about δ times the edge's
length". The fit is to `sqrt(LINEAR_TOLERANCE)` ≈ 3e-5.

## How other kernels do it

The integration method is not the difference. OCCT integrates each face by
adaptive 2D Gauss–Kronrod over its parameter domain bounded by the edges'
2D curves, to a requested relative error (`BRepGProp::SurfaceProperties`
with `Eps`); kentron follows OCCT; cad-kernel uses Green's theorem round
the loops as NGK does. What differs is the pcurve fed to it. OCCT and
kentron store 2D curves fitted far more tightly than NGK's; cad-kernel's
`Pcurve::Inverse` is "the edge's curve inverted onto the surface",
evaluated at each node. On `boolean/sphere-lens`: OCCT 2e-13, kentron
6e-12, cad-kernel 1e-16, NGK 2.5e-6. On `tangent/thin-lens`: cad-kernel
1.3e-9, OCCT 1.2e-7, NGK 3.7e-5. NGK's adaptive quadrature already targets
1e-10; its input is the limit.

## Measured: tightening the fit

The analytic pcurve fit (`fitted_over` in
`geometry/dim3/intersections/analytic/surface_surface.rs`) was given a
tighter target, on the bench's cases with exact measures:

| target | sphere-lens | thin-lens | spheres-union | spheres-union time |
|---|---|---|---|---|
| 3e-5 (today) | 2.5e-6 ✗ | 3.7e-5 ✗ | 6.2e-7 | 0.27 s |
| 1e-7 | ok | 2.3e-6 ✗ | ok | 0.80 s |
| 1e-8 | ok | 1.4e-7 ok | 6e-13 | 1.5 s |
| 1e-9 | 1.5e-13 | 1e-8 | 4e-14 | 2.7 s |

At 1e-9 on `spheres-union` the measure takes 1.56 s instead of 0.12 s (it
splits its quadrature at every knot) and the Boolean 0.56 s instead of
0.08 s; replacing the dense LU
of `interpolate_open` by a banded solve changed nothing measurable. Cases
without fitted pcurves are unaffected. So a tighter fit fixes accuracy at a
cost of ×5 to ×10 on curved Booleans.

## What pcurves remain for

The inverse map replaces the pcurve in one place only: the measure, on an
analytic face, along an edge whose 3D curve is exact. Pcurves stay the
reference for everything else: the face's domain in its parameters (point
in face, trim domain, imprints and splits are written in (u, v)); the branch
of a periodic support a loop runs on, which no inverse can tell; faces on
NURBS, revolved-spline, swept and lofted supports, which have no closed-form
inverse and would need a Newton projection at every node; meshing the
boundary in (u, v); STEP, whose edges carry pcurves. Even in the measure the
pcurve picks the branch: the inverse is taken nearest the pcurve's point.

## Approach (D7)

- **Recommended on analytic supports:** integrate Green's form over the
  exact preimage of the 3D edge, inverting the surface in closed form at each
  quadrature node (plane, sphere, cylinder, cone and torus all invert), with
  derivatives by the chain rule. No fitted pcurve enters the integral, and
  the answer is exact to the quadrature tolerance.
- **Elsewhere:** fit pcurves adaptively to the measure's own tolerance, or
  refine the fit on demand when the measure asks for more.

## Tests

Lens of two balls at overlaps 1, 1e-3, 1e-6; a ball less a ball off its
pole axis; a cylinder less a ball; each area against its closed form to
1e-9 relative.
