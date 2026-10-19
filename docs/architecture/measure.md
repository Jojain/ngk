# Measure

Length, area, volume, centroid and inertia of entities, under `src/measure/`.
The decision behind the method is
[ADR 0001](../adr/0001-mass-properties-by-green-boundary-integral.md); the
survey of alternatives is
[reviews/2026-10-15-mass-properties-algorithms.md](reviews/2026-10-15-mass-properties-algorithms.md).

## API

The typed views carry the measures; the module itself exports only the
bundles and the error.

| View | Scalar | Bundle |
|---|---|---|
| `Edge`, `Profile` | `length()` | `linear_properties()` → `LinearProperties` |
| `Face`, `Sheet`, `Solid` | `area()` | `surface_properties()` → `SurfaceProperties` |
| `Solid` | `volume()` | `volume_properties()` → `VolumeProperties` |

Each bundle holds the measure, the centroid and an `Inertia<D>` about the
centroid, branded by dimension (`Length`, `Area`, `Volume`) so a surface's
inertia cannot be added to a solid's. There is no density: mass properties are
at density one. Measurement is a read of `&Model<P>` and opens no transaction.

`Edge::length()` is the trimmed curve's own length (`TrimmedCurve::length`);
`linear_properties().length` is integrated with the centroid and inertia.

### Bounding box

`Edge`, `Profile`, `Face`, `Sheet` and `Solid` have `bounding_box()` →
`BBox`, the tightest box along the world axes, and `bounding_box_in(&Frame)`,
the tightest box whose ranges run along that frame's axes. `oriented_bounding_box()`
is the smallest of the exact boxes over a fixed list of candidate frames -- the
directions the geometry names (plane normals, straight edges, the axes of
conics, cylinders, cones, tori, helices, revolutions and extrusions), the
principal axes of inertia and the world axes, paired with each other -- centred
on the box with its axes from the longest extent to the shortest. Its extents
are exact; its orientation is the best candidate, not a proven minimum, and the
same input always gives the same box. It is not integrated but solved: an
extreme along an axis lies at a vertex, at an edge point whose tangent is square
to the axis, or at a face point whose normal is parallel to it, and those
points have closed forms on lines, conics, helices and the analytic surfaces
(`bounding_box.rs` lists each case). Interior face points are kept only where
the face's trim holds them. NURBS curves and surfaces, and revolutions of a
profile other than a line, circle or ellipse in a meridian plane, are refused
with `EdgeBoundingBoxNotImplemented` or `FaceBoundingBoxNotImplemented`.

## Method

**Curves** are integrated along their 3D span: `|c'(t)|` times the moment
monomials, with Gauss–Kronrod over the span's fraction, split at NURBS knots
and at quarter turns of circles, ellipses and helices.

**Faces** use Green's theorem on the trimmed parameter domain `D`
(`green.rs`):

```text
∬_D g(u, v) du dv  =  ∮_∂D G(u, v) dv,      G(u, v) = ∫_{u_ref}^{u} g(s, v) ds
```

- `FaceDomain::of_face` reads the loops from `UnwrappedFaceDomain`, which
  places each loop continuously across periodic cuts and closes ring and cap
  faces with straight runs across the cut and along a collapsed row. Each
  loop becomes a list of pieces: pcurves, moved by their whole-period offset,
  and straight segments for those runs. Seams and bridges are run twice in
  opposite directions and cancel without special handling.
- The **outer** integral runs along all pieces as one parameter, piece `i`
  occupying `[i, i + 1]`, so one adaptive integral spends its panels where the
  face needs them. Breakpoints are the pcurve's knots or quarter turns, and
  every fraction where the piece crosses a `v` knot row of the surface (found
  by sampling and bisection): the inner integral runs at the outer point's
  `v`, so a surface that is only `C²` across a row hands the outer integrand a
  kink there.
- The **inner** integral runs in `u` from `u_ref` to the boundary point,
  split at the surface's `u` knots (NURBS, or the profile curve of a ruled or
  revolved surface) and at quarter turns on angular parameters. `u_ref` is
  the left edge of the face's own parameter extent, not of the support's
  domain, which keeps inner ranges as short as the face is wide and works on
  unbounded planes and cylinders.
- Pieces that run at constant `v` contribute nothing and are dropped.
- A face **nothing encloses** (whole sphere, torus) covers its support's
  parameter rectangle, integrated as one more rectangular loop, minus any
  holes. The rectangle winds by the face's sense, or against the first hole
  when there is one, since a Boolean flips such a face by rewinding its holes.

The result is signed by the loops' winding, which is the face's sense: the
outer loop runs counter-clockwise when the face's normal is `S_u × S_v`, and
holes run the other way round.

**Integrands** (`integrate.rs`) take the surface point and its partials
(`SurfaceGeometry::point_and_derivative_at`, which each surface evaluates in
one pass):

- surface moments: `|S_u × S_v|` times `1, x, y, z, x², y², z², xy, xz, yz`,
  negated for a face whose loops wind clockwise;
- volume moments: by the divergence theorem, the flux through `S_u × S_v` of
  one axis-aligned field per moment (`(x, 0, 0)` for the volume,
  `(x²/2, 0, 0)` for the first moment in `x`, and so on), summed over every
  face of every shell.

Offsets are taken from the centre of the box around points sampled on the
boundaries, in units of half its diagonal. The centre keeps moments the size
of the shape, not of its distance from the origin; the scaling makes all ten
components commensurate so one relative tolerance holds each of them to the
shape's size. Inertia is assembled about that centre and moved to the
centroid by the parallel-axis theorem.

A solid's outer shell must enclose positive volume and each cavity shell
negative; anything else is `MeasureError::InvalidOrientation`, never a
negative volume.

## Quadrature

`quadrature.rs` integrates vector-valued integrands over an ordered list of
breakpoints, never letting a panel straddle one.

- `Quadrature::Adaptive(tol)`: Gauss–Kronrod 7–15 per panel, the panel error
  taken as `|K15 − G7|`. The panel with the largest error is bisected until
  every component's total error is within `tol` of the largest component's
  integral of magnitude. The public properties use `1e-10`; inner integrals
  run ten times tighter than their outer one. More than 400 bisections beyond
  the initial panels is `Unconverged`, reported as `FaceUnconverged` or
  `EdgeUnconverged`.
- `Quadrature::Fixed`: eight-point Gauss–Legendre per panel, no error
  estimate.

## Callers inside the crate

- `validation.rs` (`validate_solid_orientation`) asks `signed_shell_volume`
  for each shell's sign.
- Boolean assembly (`builders/boolean/assemble.rs`) asks
  `signed_volume_for_faces` to classify result components as outer or cavity
  shells, and refuses one whose volume is below the linear tolerance cubed.

Both use `Quadrature::Fixed` with the volume field alone: one implementation
for all three callers, at the cost the sign check can afford.

## Accuracy

| Input | Error |
|---|---|
| Planar faces with straight edges, low-degree polynomial patches (volume) | rounding |
| Analytic primitives (plane, cylinder, cone, sphere, torus) | rounding in practice; bounded by the tolerance |
| Rational and fitted NURBS surfaces | the tolerance |
| Faces trimmed by fitted pcurves | the pcurves' fidelity: a pcurve `δ` from its edge moves area by about `δ` × the edge's length |

Measured: block, cylinder, sphere and torus agree with the closed forms to
about `1e-16`, at 0.1–0.4 ms per solid in release
(`cargo run --release --example measure_accuracy_probe`). The foreign STEP
fixtures agree with OCCT's `BRepGProp` to rounding for the primitives and
within `4e-7` for parts trimmed by fitted splines; Boolean results with
curved intersections are exact to their fitted pcurves, which are held to the
square root of the linear tolerance.

## Deliberately not done

- No tolerance parameter and no achieved-error report on the bundles: no
  caller needs one yet.
- No closed-form inner integrals per analytic surface. They would only speed
  up what the quadrature already gets exact.
- No mesh-based measure. The display tessellation is not used for any
  measurement.
