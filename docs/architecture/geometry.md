# Geometry

Pure math under `src/geometry/`: no topology, no payload.

## Types

- 2D (`geometry/dim2/`): `Curve2` (`Line2`, `Circle2`, `Ellipse2`,
  `NurbsCurve2`), `TrimmedCurve2`, 2D intersections (used for pcurves and
  imprints).
- 3D (`geometry/dim3/`): `Curve` (`Line`, `Circle`, NURBS), `Surface` (`Plane`,
  `Cylinder`, `RuledSurface`, `SurfaceOfRevolution`, `NurbsSurface`), `BBox`,
  `Frame`, `Interval`.
- Intersections: curve/curve, curve/surface, surface/surface with
  `IntersectionOptions`.
- Tolerances: `LINEAR_TOLERANCE`, `ANGULAR_TOLERANCE`; point equality through
  `PointCoincidence::coincides`.

## Supports and spans

**A `Curve` is an unbounded support, never trimmed to the entity carrying it.**
There is no `Curve::Bounded` variant: a line runs to infinity, a circle closes.
Which part is meant is said by **`TrimmedCurve`** (`geometry/dim3/trimmed.rs`):
support plus an `Interval` of its native parameters, kept as one value so the
halves cannot drift apart. It normalizes traversal (fraction 0 → 1 over the
span), unwraps periodic branches, and answers `contains` / `length` / `sub` /
`reversed`. `to_curve()` is the cut-down copy: exact, but NURBS, so reach for it
only when a curve that *is* the section is required (fitting a pcurve).

**2D mirrors 3D exactly.** A `Curve2` is an unbounded support in a surface's
parameter space with native parameters; nothing is renormalized to `[0, 1]`.
**`TrimmedCurve2`** (`geometry/dim2/trimmed.rs`) is support plus `Interval`, with
the same API as `TrimmedCurve`. Both carry the same shorthand constructors
(`segment`, `arc`, `ellipse_arc`, each anchoring the support so the span is just
the sweep, and `whole` for a support that already is the section), so a common
span is never spelled out as a support and an interval side by side.

- **Every pcurve is a `TrimmedCurve2`.** Unlike an edge it has no bounding
  vertices to derive a span from, since a face stores no 2D vertex positions.
- **2D intersection takes two spans, not two supports.** The subdivision search
  needs control polygons, which an infinite support has none of; it returns
  fractions of each span.
- **The span is derived only on an edge.** `EdgeAttr` stores no interval;
  `Edge::trimmed_curve()` derives it from the bounding vertices and the view's
  orientation. Everything without vertices (`AnalyticSection`,
  `SurfaceIntersectionBranch`, `FaceImprint`, `IntersectionSpan`) must carry
  its span: the minor and major arc between two points share both endpoints, so
  endpoints alone name neither, and direction is load-bearing besides.
- **A solver answers for the support.** Clip its result to the entities' own
  spans (`TrimmedCurve::contains`) before treating it as a contact, or an edge
  picks up hits from the part of its line no edge occupies.
- **`FaceImprint`'s two halves are synchronized** (same fraction, same point),
  which constrains the support: a `Circle` spans its arc in angle, the rational
  quadratic its pcurve is fitted from does not, so an imprinted arc carries
  NURBS.

## A parameter carries the space it lives in (`geometry/parameter.rs`)

`Param<S>` wraps one `f64` with a marker, and `Interval<S>` is a range of them:

- **`Native`** is the support's own parameterization: a line's affine, a conic's
  radians, a NURBS curve's knot domain.
- **`Normalized`** is a traversal fraction of a named span, `0` its start and
  `1` its end.

The aliases are `NativeParam` and `Fraction`; `Interval` alone means
`Interval<Native>`. `Interval::at` and `Interval::fraction_of` are the **only**
two conversions, and each is asked of the span the fraction is a fraction of, so
"a fraction of what" always has an answer at the call site.

- **Only a `TrimmedCurve` / `TrimmedCurve2` produces or consumes a `Fraction`.**
  A bare `Curve` has no span to be a fraction of, which is why there is no
  `Curve::trimmed`: naming a portion of a support is `trimmed_native`.
- **A `Fraction` is not confined to `[0, 1]`.** A solver hit just off the end of
  a span, a clipped overlap, a point projected past an edge: each is a real value
  outside the unit interval, and clamping it would destroy the news that the hit
  was off the span. Ask `is_inside_unit` where that matters.
- **Surfaces are not branded yet, and not because they are safe.** A surface has
  two parameter spaces (its analytic `(u, v)` and the knot domain of
  `to_nurbs_over`, hence `ParamMap` and `Reparam`), but no trimmed surface, so
  `Normalized` has nothing to say about one. `u` and `v` stay `f64`, as does the
  `Point2` of its parameter space; `domain()` is `Interval<Native>`. The NURBS
  evaluator and the subdivision solvers are plain `f64` for the same reason: one
  space each, unwrapped at their edges.

## Intersections: analytic first, certified NURBS fallback

`geometry/dim3/intersections/analytic/` answers a recognized pair in closed form
**before** anything is converted or decomposed. Everything else takes the NURBS
solver, which is also the differential-test oracle.

`intersect_analytic_*` return `Option<Result<..>>`: `None` declines the pair
(fall back), `Ok(..Empty)` certifies disjointness. A pair in the table that
reaches a case the `Curve` types cannot carry also declines.

| Kind | Covered |
|---|---|
| surface/surface | plane × {plane, sphere, cylinder}; sphere × sphere |
| curve/surface | line × {plane, sphere, cylinder, cone}; circle × {plane, sphere} |
| curve/curve | line × line, line × circle, circle × circle |

Not covered: any cone surface pair, cylinder × cylinder, circle × {cylinder,
cone}.

A section's 3D curve is exact; its pcurve is exact only where a closed form
exists for it on that surface (a plane, an isoline), and a measured fit
otherwise.
