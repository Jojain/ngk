---
status: accepted
date: 2026-10-15
---

# Mass properties by Green boundary integral

Area, volume, centroid and inertia are computed by reducing each integral to the
boundary: the divergence theorem takes a solid's volume integrals to its faces,
and Green's theorem takes each face's integral over its trimmed parameter domain
to a line integral along its pcurves, evaluated with nested Gauss quadrature
split at pcurve and surface knots. This replaces measuring the display
tessellation, which converges as O(h²) on a fixed grid, fails wherever the
mesher fails, and on the reference primitives is about twenty times slower than
OCCT while reaching 1e-3 to 1e-2 relative error where OCCT reaches machine
precision. The boundary integral
converges spectrally on smooth integrands, is exact for volume on polynomial
patches, and needs nothing but closed, oriented pcurve loops and surface
derivatives, which every committed face already has.

## Considered options

The survey behind this decision is
[docs/architecture/reviews/2026-10-15-mass-properties-algorithms.md](../architecture/reviews/2026-10-15-mass-properties-algorithms.md).

- **Refine the tessellation and extrapolate (Richardson).** Rejected: still not
  total, still slow, and the extrapolation is not guaranteed on ngk's
  non-uniform meshes.
- **Integrate the true integrand on the mesher's UV triangles.** Rejected: it
  inherits every tessellator failure and keeps the chordal boundary error.
- **Closed-form inner integrals per analytic surface.** Not rejected but
  deferred: only a speed-up over the chosen scheme, at the cost of special cases
  to maintain.
- **Nested adaptive Runge–Kutta (SMLib).** Rejected: slower than Gauss for the
  same accuracy, with no compensating benefit.

## Consequences

- Pcurve fidelity becomes the error floor. After a Boolean, intersection
  pcurves are fitted, so "exact" means exact to the pcurves. The tessellator
  depended on them equally, so this is not a regression.
- Shell-orientation validation and Boolean shell assembly use the same
  measurement as the public API, at a fixed low order: one implementation,
  three callers.
- The sampled pcurve fan, the planar-face shortcut and the computation
  tessellation are deleted.
