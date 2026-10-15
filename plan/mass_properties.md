# Mass properties: Green boundary integral

Replace the tessellation inside `src/measure/` with the boundary integral
decided in [ADR 0001](../docs/adr/0001-mass-properties-by-green-boundary-integral.md).
The public API (`length` / `linear_properties`, `area` / `surface_properties`,
`volume` / `volume_properties` on the typed views, and the three property
bundles with branded `Inertia`) stays as it is.

Background and measurements:
[the survey](../docs/architecture/reviews/2026-10-15-mass-properties-algorithms.md).

## Current state

Everything in `src/measure/integrate.rs` measures the display tessellation at a
fixed `COMPUTATION_TESSELLATION` (64 curve segments, 64×32 surface grid):

- `face_surface_properties`, `sheet_surface_properties`: triangle moments.
- `solid_volume_properties`, `signed_shell_volume`, `signed_volume_for_faces`:
  signed tetrahedra from a reference point.
- `face_signed_volume` (planar shortcut) and `fallback_face_signed_volume` (the
  32-sample pcurve fan) back the validation path when the mesher refuses a
  face.
- `linear_properties_for_edge(s)`: exact length, centroid and inertia from a
  64-segment polyline rescaled to that length.

Callers outside the module: `validation.rs` (`signed_shell_volume`, sign only)
and `builders/boolean/assemble.rs` (`signed_volume_for_faces`, magnitude used to
classify components). Both run on every Boolean.

Measured error today: block exact; cylinder −1.2e-3 (V); sphere −4.0e-3;
torus −8.0e-3. `cargo run --release --example measure_accuracy_probe`
reproduces it.

## The algorithm

For a face with surface `S(u, v)` and trimmed parameter domain `D`:

```text
∬_D g(u,v) du dv  =  ∮_∂D G(u,v) dv,      G(u,v) = ∫_{u_ref}^{u} g(s,v) ds
```

- **Outer loop**: along each pcurve of each loop, Gauss points in the pcurve's
  fraction, weighted by `v'(t)`. Split at pcurve knots.
- **Inner loop**: at each outer point, Gauss points in `u` from `u_ref` to
  `u(t)` at fixed `v`, evaluating `S`, `S_u`, `S_v`. Split at surface knots.
- **Integrands**: `|S_u × S_v|` times `[1, x, y, z, x², …]` for surface
  properties; for volume, `F(S) · (S_u × S_v)` with one divergence field per
  moment (`div F = 1, x, y, z, x², y², z², xy, xz, yz`), the face's sense
  multiplying the normal.
- **Loops** come from `UnwrappedFaceDomain`, which already makes each loop
  continuous across periodic seams. Seam and bridge edges are run twice in
  opposite directions and cancel; nothing special-cases them.
- **Boundaryless faces** (whole sphere, whole torus) integrate over the
  surface's parameter rectangle with a tensor Gauss rule, minus any hole loops.
- **Reference line** `u_ref` at the face's own UV bounding box, not the
  surface's domain: short inner ranges, and it works on unbounded planes and
  cylinders.
- **Volume field** centred at the shell's bounding-box centre, to avoid
  cancellation on parts far from the origin.
- **Inertia** is assembled about the origin and moved to the centroid by the
  parallel-axis theorem.

Two quadrature modes share the integrand closures:

- **Fixed order** Gauss–Legendre, order from surface and pcurve degree in the
  manner of OCCT (`BRepGProp_Face::IntegrationOrder`). Used for the shell-sign
  check and Boolean assembly, where speed matters and precision does not.
- **Adaptive** Gauss–Kronrod 7–15, bisecting each interval until the estimate
  meets a relative tolerance. Used for the public properties.

Edge length and linear moments use the same 1D Gauss–Kronrod rule on the 3D
curve, per knot span; this replaces both adaptive Simpson and the rescaled
polyline.

## Decisions

Taken without a design discussion, from the house rules; reopen any that is
wrong.

1. **Public signatures do not change.** The public properties run the adaptive
   rule at a fixed relative tolerance of `1e-10`. No tolerance parameter and no
   `Fidelity` on the result until a caller needs one (no speculative API).
2. **Validation and Boolean assembly switch in the same change** as the public
   properties: one implementation, three callers.
3. **The tessellation measure is deleted**, not kept as an opt-in mesh path:
   nothing needs it, and deleted behaviour takes its tests with it.

## Tasks, in order

1. **Go red.** Tighten `tests/mass_properties.rs` to the reference table below
   at `1e-10` relative for volume, area, centroid and inertia on all four
   primitives. Everything curved fails today.
2. **Quadrature.** Gauss–Legendre nodes and weights, Gauss–Kronrod 7–15, and
   interval splitting at a sorted knot list, in their own file. Tests: exact on
   polynomials of degree `2n − 1`; Kronrod estimate bounds the true error on
   smooth integrands.
3. **Face integral.** The Green double loop over an `UnwrappedFaceDomain`, generic
   over an integrand returning a fixed-size moment vector. Switch
   `face_surface_properties` and `sheet_surface_properties` onto it.
4. **Volume.** The divergence fields; switch `solid_volume_properties`,
   `signed_shell_volume` and `signed_volume_for_faces`, the last two at fixed
   order.
5. **Length.** Gauss–Kronrod per knot span for `linear_properties_for_edge(s)`.
6. **Delete** `COMPUTATION_TESSELLATION`, the triangle and tetrahedron moment
   functions, `face_signed_volume`, `fallback_face_signed_volume` and
   `MeasureError::FaceTessellation`; adapt every test that names them.
7. **Document.** `src/measure/mod.rs` module doc states which inputs are exact
   (volume on polynomial patches with polynomial pcurves; planar faces with
   straight edges) and what bounds the rest (the tolerance, and pcurve
   fidelity). Write `docs/architecture/measure.md`.

## Testing

- The four primitives against the reference table, at `1e-10` relative.
- Additivity: a block equals the sum of the two pieces a Boolean cut makes.
- Invariance: volume, area and principal moments unchanged by
  `builders::transform::rigid`; the centroid moves with it.
- Cavity: a block with a block-shaped cavity measures the difference.
- Sign: an inside-out solid is refused by `validate_solid_orientation` and never
  reaches a caller as a negative volume.
- The swept elbow in `tests/builders/sweep.rs`: miter 80.0 and round corner
  79.5708 (both confirmed against OCCT); the self-intersecting version measures
  32.
- Boolean output and imported STEP (`tests/exchange/step_import.rs`) against
  build123d, whose pcurves are fitted: this is where pcurve fidelity shows.
- Speed: `measure_accuracy_probe` must not regress against today's timings
  (0.2–1.0 ms per solid).

## Reference values

From OCCT (`OCP.BRepGProp`), density 1, inertia about the centre of mass. Each
is checkable against a closed form: `m(b²+c²)/12` for the block, `m(3r²+h²)/12`
and `mr²/2` for the cylinder, `2mr²/5` for the sphere, `m(4R²+5r²)/8` and
`m(R²+3r²/4)` for the torus.

| shape | volume | area | edge length | centroid | Ixx, Iyy, Izz |
|---|---|---|---|---|---|
| block 2×3×4 | 24.000000000 | 52.000000000 | 72.000000000 | (1, 1.5, 2) | 50, 40, 26 |
| cylinder r1 h2 | 6.283185307 | 18.849555922 | 29.132741229 | (0, 0, 1) | 3.665191429, 3.665191429, 3.141592654 |
| sphere r1 | 4.188790205 | 12.566370614 | 6.283185307 | (0, 0, 0) | 1.675516082 ×3 |
| torus R3 r1 | 59.217626407 | 118.435252813 | 62.831853072 | (0, 0, 0) | 303.490335333, 303.490335333, 577.371857464 |

## Constraints

- Measurement is a read: it takes `&Model<P>` and opens no transaction.
- Comments state what the code does; none references this plan.
- Vocabulary: this measures faces, sheets and solids, never 2-cells.
