# Parameter units: native, normalized, and what an `f64` is not allowed to mean

Status: **In progress** — stages 1 and 2 are implemented. `Param<S>` and
`Interval<S>` live in `geometry::parameter` and `geometry::interval`, the curve
layer and everything downstream of it carry the brand, `Curve::trimmed` is gone
and the arc-split bug it caused is fixed and covered by a regression test.
`Delta<S>` (stage 3) and `Knot` (stage 4) do not exist yet.

## The conflation

Every parameter in the kernel was an `f64`, and `f64` was asked to mean four
different things:

| Meaning | Reference | Where it comes from | Value on the arc `[1, 2] rad` of a unit circle |
|---|---|---|---|
| **native parameter** | the support's own parameterization | `Curve::param_at`, `Curve::domain`, `TrimmedCurve::interval` | `1.5` |
| **normalized fraction** | *a named span* — `0` is its start, `1` its end | `TrimmedCurve::point_at`, `::sub`, `::parameter_at` | `0.5` |
| **NURBS knot parameter** | the knot domain of `to_nurbs()` | `NurbsCurve::point_at`, the target of `Reparam` | neither, and not a linear function of either |
| **arc length** | distance along the curve | `Curve::length(t0, t1)` | `0.5` — and that it agrees with the fraction here is an accident of radius 1 |

The prose separated these carefully: `dim3/trimmed.rs`, `dim2/trimmed.rs`,
`traits.rs` and `reparam.rs` said "native" or "normalized" in nearly every doc
comment. The *types* separated none of them. Nor did the containers — `Interval`
carried native spans, fraction spans and knot spans alike, with no way to tell
which a value was except by reading the function that produced it.

Arc length is the odd one out and is **not** a parameterization here: nothing
evaluates a curve at an arc length, so it appears only as a measured distance.
It still belongs in the table, because it is the unit
`TrimmedCurve::parameter_slack` converts *from*, and because `LINEAR_TOLERANCE`
— a distance — is compared directly against fractions and native parameters in
several places to this day.

Two methods on `Curve` differed in nothing but the unit of their argument:

```rust
pub fn trimmed(&self, interval: Interval) -> Result<Self, NurbsError>        // normalized
pub fn trimmed_native(&self, interval: Interval) -> Result<Self, NurbsError> // native
```

Same type, same shape, same name to within a suffix, opposite meanings, and no
diagnostic if the wrong one was called. (`Curve2` had only the native one, so
the 2D side was already clean.)

## The bug — this was not hypothetical

`builders::edges::split_edge` takes a **native** parameter. It validated it
against the edge's native span, then handed it to `split_curve_at_parameter`,
which did this:

```rust
let interval = edge_reference_interval(g, edge, first_dart, second_dart, curve)?;
let fraction = (parameter - interval.start) / (interval.end - interval.start);
...
trim(Interval::new(0.0, fraction))?,
trim(Interval::new(fraction, 1.0))?,
```

The fraction was taken **against the edge's span** and then spent by
`Curve::trimmed`, which resolved it **against the whole support's `to_nurbs`
domain**. Both are legitimate normalized fractions. They are fractions of
different things, and nothing said so.

Splitting the arc `[1 rad, 2 rad]` of a unit circle at `1.5 rad` therefore
produced, as the two stored supports, the circle's **top half and bottom half**:

| | expected | produced |
|---|---|---|
| first piece | the arc `1 → 1.5` | a NURBS over knots `[0, π]` |
| second piece | the arc `1.5 → 2` | a NURBS over knots `[π, 2π]` |

The new corner landed correctly, because it is placed by
`curve.point_at(parameter)` with the native value. Only the curves were wrong,
so nothing failed loudly: the vertices agreed, the map was valid, and the shape
was quietly the wrong shape. `split_face_boundary_edge` took the same path,
which put this on the Boolean and imprint routes too.

`splitting_a_bounded_arc_keeps_each_piece_on_its_own_sweep` in
`tests/topology/edge_split.rs` is the regression test. It states the property
over the point set rather than over the parameter, because trimming an arc
yields a NURBS and a NURBS does not span an arc in angle: the pieces owe the
right geometry, not the circle's own parameterization.

## The design

Two moves. The first stops native and normalized being interchangeable. The
second stops "a fraction of *what*" being an unanswerable question.

### 1. Brand the scalar, and the interval with it

```rust
pub struct Param<S = Native> { value: f64, space: PhantomData<S> }
pub struct Interval<S = Native> { pub start: Param<S>, pub end: Param<S> }

pub struct Native;      // a support's own parameterization
pub struct Normalized;  // traversal of a named span: 0 its start, 1 its end

pub type NativeParam = Param<Native>;
pub type Fraction = Param<Normalized>;
```

`Param<S>` is `#[repr(transparent)]` and `Copy`, so it costs nothing at runtime.
The kernel had accepted a typed scalar once already — `radians::Rad64` carries
every angle through `builders::revolve` — so this is the same move applied to
the scalar that is a great deal easier to get wrong than an angle.

The two conversions are the only bridge between brands, and they belong to the
interval because the interval *is* the reference:

```rust
impl Interval<S> {
    pub fn at(self, fraction: Fraction) -> Param<S>;
    pub fn fraction_of(self, parameter: Param<S>) -> Fraction;
}
```

`fraction_of` was open-coded in five places before this —
`reparam::interval_fraction`, `edges.rs`, both `TrimmedCurve::parameter_at`
implementations, and `intersections::curve_curve::normalize_parameter` — each
with its own answer for a zero-length span. Naming it once removed all five, and
the same happened to the hand-rolled `start + (end - start) * t` and
`0.5 * (start + end)`, which are now `at` and `midpoint`. Several call sites got
*shorter*, which is the shape to keep aiming for.

### 2. A fraction belongs to a span, structurally

The bug above was not a swapped unit. It was a fraction spent against the wrong
reference, and the brand alone does not see it. The rule that does:

> **A bare `Curve` has no fractions**, because it has no span to be a fraction
> of. Only `TrimmedCurve` and `TrimmedCurve2` produce or consume a `Fraction`,
> and they resolve it against the span they carry.

`Curve::trimmed` cannot be written under that rule — a `Curve` holds no
`Interval<Native>` to resolve an `Interval<Normalized>` against — so it is gone.
Its callers now say which span they mean, and `split_curve_at_parameter` turns
out not to need a fraction at all: the split parameter and the edge's span ends
are all native on the same support, so each piece is named directly.

`boolean::graph::normalized_subcurve` was the one caller that genuinely wanted a
different space: the numeric curve/surface solver answers in the *prepared
NURBS'* knot domain. It is now `nurbs_subcurve`, which says so, and the fraction
round-trip it used to make through `Curve::trimmed` — native knots to a fraction
and straight back to native knots — is gone.

**Not** branded per instance. Giving each `TrimmedCurve` its own brand — a
lifetime, or a const generic — would catch "a fraction of span A used on span
B", but it would also make `FaceImprint` inexpressible: its whole invariant is
that the same fraction of its 3D span and of its pcurve is the same point.
Cross-span fraction use is a deliberate *synchronization contract*, and it
already has the right home — a named type (`FaceImprint`,
`SurfaceIntersectionBranch`) whose documentation states it.

## What the brand deliberately does not do

- **It does not enforce `0 ≤ fraction ≤ 1`.** A solver hit just off the end of a
  span, a clipped overlap, a point projected beyond an edge — each produces a
  fraction outside the unit interval, and that is *information*. A constructor
  refusing them would either force an unwrap at every producer or, worse, clamp
  and destroy the news that the hit was off the span. `Fraction` is unbounded as
  a value and carries only the statement "`0` is the start, `1` is the end",
  with `is_inside_unit` and `clamped_to_unit` for callers that want a window.
  Refuse rather than approximate — and do not refuse where nothing is wrong.
- **It does not reach surfaces — yet, and not because they are safe.** A
  surface has two parameter spaces exactly as a curve does: its analytic
  `(u, v)` and the knot domain of `to_nurbs_over`. `ParamMap` and `Reparam`
  exist precisely because those two disagree everywhere but the knots. What a
  surface does *not* have is a **fraction**: there is no trimmed surface and
  nothing that means "halfway across this patch", so the `Normalized` brand has
  nothing to say about one. Its second space is `Knot`, which is stage 4.

  So surfaces stay `f64` — as does the `Point2` that is a point of their
  parameter space — and `domain()` stays `Interval<Native>`, with
  `Param::value` reading an endpoint out at the last hop. The same holds for
  the NURBS evaluator and the subdivision solvers: one space each, unwrapped at
  their edges.

  *(The original plan had surfaces branded as `Param<Native>` in both
  directions, and an earlier draft of this section claimed a surface has only
  one parameter space. That claim was wrong. The correct statement is the one
  above: the confusion over a surface is native-versus-knot, and it is deferred
  rather than absent.)*

  The shape that answer wants is **`Uv<S>` as `dim2`'s point type**, taken up
  with stage 4 — see the note there.
- **It does not brand `u` against `v`.** Both are a surface's own
  parameterization, and separating them into two `Param` arguments would fork
  every axis-generic helper over `Axis2` in two for a mistake that produces
  visibly wrong geometry and fails loudly. A single `Uv` argument disposes of
  the same mistake for nothing, since one argument cannot be swapped with
  itself — another reason that is the right shape when surfaces are branded.
- **It does not replace `Interval` for non-parameter ranges.** There are none:
  every `Interval` in the tree is a parameter range.

## Rejected: `Normalized<Interval>`

Wrapping the container rather than the scalar — `Normalized<Interval>` beside a
bare `Interval` — is the smaller change, and it is worth saying why it is not
enough. The scalar is what flows: `point_at(f64)`, `param_at(..) -> f64`,
`sub(..)`, every solver output. Branding only the pair leaves all of those
unbranded, which is where most of the mixing happens. And the central operation
is a *conversion* — `Interval<Native>::at(Fraction) -> NativeParam` — which has
no honest signature unless the scalars carry the brand too. `Interval<S>`
subsumes `Normalized<Interval>` and costs nothing more.

## Rejected: `Deref<Target = f64>` on `Param`

It would remove some `.value()` calls and is the wrong trade. Deref coercion
applies to `&Param → &f64`, so it helps only *method* calls, not the two cases
that dominate — passing a `Param` where an `f64` is expected, and operators. And
it leaks all of `f64`'s API onto `Param`, so a `Fraction` would silently answer
`clamp`, `rem_euclid` and every other float method: the brand would stop meaning
anything at exactly the sites where mixing happens. `.value()` is the explicit
marker for leaving the branded world, and it is concentrated at real boundaries.

## Stages

1. **`Param<S>` and `Interval<S>`, with `Native` and `Normalized`.** *Done.*
   `Interval<S = Native>` defaults its space, so `Interval` still spells the
   common case. Roughly 900 sites across `src/`, `tests/` and `bindings/`, most
   of them `.value()` at a boundary; the whole migration was compiler-driven.
2. **Delete `Curve::trimmed`; fix `split_curve_at_parameter`.** *Done*, with the
   regression test written red first.
3. **`Delta<S>`.** A difference of parameters is not a parameter:
   `Interval::delta`, `Periodicity::Periodic(period)` and
   `TrimmedCurve::parameter_slack` all return one, and
   `Interval::contains(value, tolerance)` takes one. Typing it makes
   `Param + Delta` — the whole-period shift in `Param::on_branch_near` — legal
   and `Param + Param` impossible, and, more usefully, turns every place that
   compares a distance `LINEAR_TOLERANCE` against a parameter into a compile
   error to be looked at. `Param::on_branch_near` and `Interval::midpoint`
   already localize most of that arithmetic, so the stage is smaller now than it
   would have been before stage 1.
4. **`Knot`, and `Uv<S>` with it.** `Reparam::map` becomes
   `NativeParam -> Param<Knot>` and its inverse the other way, which is what
   `Reparam` exists to be. `graph::nurbs_subcurve` and
   `dim2::intersections::KnotHit` already name the knot space in prose and would
   carry it in their types. The friction is that `Curve::Nurbs` has
   `Native == Knot` and needs an explicit identity cast in each forwarding arm.

   This is also where surfaces get the brand, and the shape it wants is a
   **`Uv<S>` point type** rather than two branded scalars:

   - `ParamMap::map` becomes `Uv<Native> -> Uv<Knot>`, which is the surface
     confusion that actually exists and the whole reason the stage pays.
   - One argument cannot be swapped with itself, so `surface.point_at(uv)`
     disposes of `u`/`v` transposition without forking anything over `Axis2`.
   - It has to be **`dim2`'s point type**, not a wrapper at the surface
     boundary. `Curve2::point_at` and `Surface::param_at` both hand back a
     `Point2`, and a pcurve evaluating into a surface is the hop a uv actually
     travels; a `Uv` built by an unchecked conversion at each call site would
     check nothing on that hop. The codebase already calls these values uv in
     every variable name — `inner_uv`, `outer_uv`, `boundary_uv` — and the type
     is the only thing not saying it.

   The cost is the size of that rename: ~459 `Point2` mentions in `src/`, 118 in
   `dim2`, plus forwarding the nalgebra surface they lean on (`norm`, `dot`,
   `coords`, `lerp`, `Point2::origin`, `HPoint2`, the intersection solvers'
   control polygons). Done before `Knot` exists it buys only the transposition
   check, which is why the two belong in one stage.

   What it will still not answer: whether a uv is *inside* the surface's domain.
   `face.normal_at(0.0, 0.0)` appears in `chamfer.rs`, `revolve.rs` and
   `solids.rs`, sound only because those faces are planar. That is a runtime
   question, and no brand settles it.
5. **Bindings.** *Done.* `bindings/wasm` and `bindings/python` keep taking and
   returning bare `f64` across the FFI boundary, wrapping on entry and
   unwrapping on exit.

## Notes for the implementation

- **The serde wire format does not change.** `Interval` is serialized inside
  `TrimmedCurve` inside `Model`. `#[serde(transparent)]` on `Param<S>`, with the
  `PhantomData` skipped, keeps a parameter on the wire as a bare number and an
  interval as `{start, end}`, exactly as before. `bound = ""` is required, or
  the derive demands `S: Serialize`.
- **`Clone`, `Copy` and `PartialEq` are written out, not derived.** A derive
  bounds each impl on the *marker* — `S: Copy`, `S: PartialEq` — which is both
  wrong and load-bearing: inside code generic over `S` those bounds do not hold,
  and `Param<S>` would stop being `Copy` exactly where `Interval<S>` needs it.
- **`Interval::new` takes `impl Into<Param<S>>`**, so `Interval::new(0.0, 1.0)`
  still compiles wherever the space is inferable. Where it is not — a `let` with
  no typed destination — the space has to be named: `Interval::<Native>::new`.
  Evaluation methods stay strict and take `Param<S>` outright, which is the line
  that matters: a raw number at a *construction* site is an assertion the author
  is making, while a raw number flowing into `point_at` is the mistake.
- **A one-sided bound is not a two-sided one.** Rewriting
  `t <= MIN || t >= 1.0 - MIN` as `(t - START).abs() <= MIN || (END - t).abs() <= MIN`
  looks like the same guard and is not: it stops rejecting parameters outside
  `[0, 1]`. Two of these slipped in during the migration and both changed
  Boolean results; `git diff` normalized for the mechanical rewrites is what
  found them.
