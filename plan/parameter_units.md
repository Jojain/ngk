# Parameter units: native, normalized, and what an `f64` is not allowed to mean

Status: **Proposed** — nothing here is implemented. The conflation it describes
is live, and [the bug below](#the-bug--this-is-not-hypothetical) is a
reproducible consequence of it.

## The conflation

Every parameter in the kernel is an `f64`, and `f64` is currently asked to mean
four different things:

| Meaning | Reference | Where it comes from | Value on the arc `[1, 2] rad` of a unit circle |
|---|---|---|---|
| **native parameter** | the support's own parameterization | `Curve::param_at`, `Curve::domain`, `TrimmedCurve::interval` | `1.5` |
| **normalized fraction** | *a named span* — `0` is its start, `1` its end | `TrimmedCurve::point_at`, `::sub`, `::parameter_at` | `0.5` |
| **NURBS knot parameter** | the knot domain of `to_nurbs()` | `NurbsCurve::point_at`, the target of `Reparam` | neither, and not a linear function of either |
| **arc length** | distance along the curve | `Curve::length(t0, t1)` | `0.5` — and that it agrees with the fraction here is an accident of radius 1 |

The prose already separates these carefully: `dim3/trimmed.rs`,
`dim2/trimmed.rs`, `traits.rs` and `reparam.rs` say "native" or "normalized" in
nearly every doc comment. The *types* separate none of them. Nor do the
containers — `Interval` carries native spans (`TrimmedCurve::new`), fraction
spans (`TrimmedCurve::sub`, `FaceImprint::trimmed`,
`FaceImprintGraphEdge::interval`) and knot spans alike, with no way to tell
which a value is except by reading the function that produced it.

Arc length is the odd one out and is **not** a parameterization here: nothing
evaluates a curve at an arc length, so it appears only as a measured distance.
It still belongs in the table, because it is the unit
`TrimmedCurve::parameter_slack` converts *from*, and because `LINEAR_TOLERANCE`
— a distance — is compared directly against fractions and native parameters in
several places today.

Two methods on `Curve` differ in nothing but the unit of their argument:

```rust
pub fn trimmed(&self, interval: Interval) -> Result<Self, NurbsError>        // normalized
pub fn trimmed_native(&self, interval: Interval) -> Result<Self, NurbsError> // native
```

Same type, same shape, same name to within a suffix, opposite meanings, and no
diagnostic if the wrong one is called.

## The bug — this is not hypothetical

`builders::edges::split_edge` takes a **native** parameter. It validates it
against the edge's native span, then hands it to `split_curve_at_parameter`,
which does this (`src/builders/edges.rs:505`):

```rust
let interval = edge_reference_interval(g, edge, first_dart, second_dart, curve)?;
let fraction = (parameter - interval.start) / (interval.end - interval.start);
...
trim(Interval::new(0.0, fraction))?,
trim(Interval::new(fraction, 1.0))?,
```

The fraction is taken **against the edge's span** and then spent by
`Curve::trimmed`, which resolves it **against the whole support's `to_nurbs`
domain**. Both are legitimate normalized fractions. They are fractions of
different things, and nothing says so.

Splitting the arc `[1 rad, 2 rad]` of a unit circle at `1.5 rad` therefore
produces, as the two stored supports, the circle's **top half and bottom half**:

| | expected | produced |
|---|---|---|
| first piece | the arc `1 → 1.5` | a NURBS over knots `[0, π]` |
| second piece | the arc `1.5 → 2` | a NURBS over knots `[π, 2π]` |

The new corner lands correctly, because it is placed by
`curve.point_at(parameter)` with the native value. Only the curves are wrong, so
nothing fails loudly: the vertices agree, the map is valid, and the shape is
quietly the wrong shape. `split_face_boundary_edge` takes the same path, which
puts this on the Boolean and imprint routes too.

No type system catches a wrong *reference span*. What catches it is the
structural half of this proposal, below: a fraction that cannot be spelled
without naming the span it is a fraction of.

## The design

Two moves. The first stops native and normalized being interchangeable. The
second stops "a fraction of *what*" being an unanswerable question.

### 1. Brand the scalar, and the interval with it

```rust
/// A parameter in the space `S`.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
#[serde(transparent, bound = "")]
pub struct Param<S> {
    value: f64,
    #[serde(skip)]
    space: PhantomData<S>,
}

/// A directed range of parameters in the space `S`.
pub struct Interval<S> {
    pub start: Param<S>,
    pub end: Param<S>,
}

/// A support's own parameterization: a line's affine, a conic's radians,
/// a NURBS curve's knot domain.
pub struct Native;

/// Normalized traversal of a named span: `0` is its start, `1` its end.
pub struct Normalized;

pub type NativeParam = Param<Native>;
pub type Fraction = Param<Normalized>;
```

The kernel has accepted a typed scalar once already — `radians::Rad64` carries
every angle through `builders::revolve` — so this is the same move applied to
the scalar that is a great deal easier to get wrong than an angle.

The two conversions become the only bridge between brands, and they belong to
the interval because the interval *is* the reference:

```rust
impl Interval<Native> {
    pub fn at(self, fraction: Fraction) -> NativeParam;           // exists today
    pub fn fraction_of(self, parameter: NativeParam) -> Fraction; // the missing inverse
}
```

`fraction_of` is open-coded in at least five places today —
`reparam::interval_fraction`, `edges.rs:505`, both `TrimmedCurve::parameter_at`
implementations, and `intersections::curve_curve::normalized_parameter` — each
with its own answer for a zero-length span. Naming it once removes that
duplication, so the migration **deletes more affine arithmetic than it adds
annotations**. That is the shape to aim for throughout: call sites should get
shorter, not more decorated.

Signatures then say what the prose already says:

```rust
Curve::point_at(NativeParam) -> Point3
Curve::domain() -> Interval<Native>
Curve::trimmed_native(Interval<Native>) -> Result<Curve, NurbsError>

TrimmedCurve::new(Curve, Interval<Native>)
TrimmedCurve::interval() -> Interval<Native>
TrimmedCurve::point_at(Fraction) -> Point3
TrimmedCurve::sub(Interval<Normalized>) -> TrimmedCurve
TrimmedCurve::parameter_at(Point3) -> Fraction
TrimmedCurve::native_parameter_at(Point3) -> NativeParam

FaceImprint::trimmed(Interval<Normalized>) -> Result<FaceImprint, NurbsError>
FaceImprintGraphEdge { interval: Interval<Normalized>, .. }
SurfaceOverlapCandidate { domain_a_u: Interval<Native>, .. }
```

Implementations unwrap at the top of the body and do ordinary `f64` arithmetic
from there. The brand lives on the boundary; nothing inside `Circle::point_at`
changes. `#[repr(transparent)]` and `Copy` mean it costs nothing at runtime.

### 2. A fraction belongs to a span, structurally

The bug above is not a swapped unit. It is a fraction spent against the wrong
reference, and the brand alone does not see it. The fix is a rule the types can
then enforce:

> **A bare `Curve` has no fractions**, because it has no span to be a fraction
> of. Only `TrimmedCurve` and `TrimmedCurve2` produce or consume a `Fraction`,
> and they resolve it against the span they carry.

Under that rule `Curve::trimmed(Interval)` cannot be written — a `Curve` holds
no `Interval<Native>` to resolve an `Interval<Normalized>` against — so it goes.
Its callers then say which span they mean, which is what they always had to
mean:

```rust
// before: a fraction of an unnamed reference
curve.trimmed(Interval::new(0.0, fraction))

// after: the span is named, so the fraction has one referent
TrimmedCurve::new(curve.clone(), edge_span)
    .sub(Interval::new(0.0, fraction))
    .to_curve()
```

That last expression is also the correct fix for `split_curve_at_parameter`, and
it falls out of the design rather than being patched in beside it.

**Not** branded per instance. Giving each `TrimmedCurve` its own brand — a
lifetime, or a const generic — would catch "a fraction of span A used on span
B", but it would also make `FaceImprint` inexpressible: its whole invariant is
that the same fraction of its 3D span and of its pcurve is the same point.
Cross-span fraction use is a deliberate *synchronization contract*, and it
already has the right home — a named type (`FaceImprint`,
`SurfaceIntersectionBranch`) whose documentation states it. Leave it there.

## What the brand deliberately does not do

- **It does not enforce `0 ≤ fraction ≤ 1`.** A solver hit just off the end of a
  span, a clipped overlap, a point projected beyond an edge — each produces a
  fraction outside the unit interval, and that is *information*. A constructor
  refusing them would either force an unwrap at every producer or, worse, clamp
  and destroy the news that the hit was off the span. `Fraction` is unbounded as
  a value and carries only the statement "`0` is the start, `1` is the end",
  with `is_inside()` and `clamped()` for callers that want a window. Refuse
  rather than approximate — and do not refuse where nothing is wrong.
- **It does not brand `u` against `v`.** Both are `Param<Native>`. Separating
  them would catch `point_at(v, u)`, but it forks every axis-generic helper —
  `Axis2`, `DomainSide::nearest`, `degenerate_rows` — into two, and a swapped
  `u`/`v` produces visibly wrong geometry that a test catches at once. The
  confusion this plan targets is the silent one. `Param<S>` generalizes, so the
  decision can be revisited without a redesign.
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

## Stages

Compiler-driven throughout: each stage ends with the tree building, and every
site the compiler flags is a site whose unit was previously undocumented.

1. **`Param<S>` and `Interval<S>`, with `Native` and `Normalized`.** Introduce
   the types, add `fraction_of`, and start from `type Interval = Interval<Native>`
   so the tree still builds. Then flip the genuinely-normalized sites —
   `TrimmedCurve::sub`, `FaceImprint::trimmed`, `FaceImprintGraphEdge`,
   `Curve::trimmed` — to `Interval<Normalized>` and let the compiler walk the
   callers out. ~495 `Interval` references across 57 files, but most need a
   turbofish or nothing at all.
2. **Delete `Curve::trimmed` and `Curve2::trimmed`; fix
   `split_curve_at_parameter`.** This is the bug fix, and it wants its
   regression test first: split a bounded arc at a native parameter and assert
   both stored supports evaluate to the arc they should. Write it red against
   today's tree.
3. **`Delta<S>`.** A difference of parameters is not a parameter:
   `Interval::delta`, `Periodicity::Periodic(period)` and
   `TrimmedCurve::parameter_slack` all return one, and
   `Interval::contains(value, tolerance)` takes one. Typing it makes
   `Param + Delta` — the whole-period shift in `native_parameter_at` — legal and
   `Param + Param` impossible, and, more usefully, turns every place that
   compares a distance `LINEAR_TOLERANCE` against a parameter into a compile
   error to be looked at. Separable from stages 1–2, and the stage most likely
   to spread, so do it after they land.
4. **`Knot`.** `Reparam::map` becomes `NativeParam -> Param<Knot>` and its
   inverse the other way, which is what `Reparam` exists to be. The friction is
   that `Curve::Nurbs` has `Native == Knot` and needs an explicit identity cast
   in each forwarding arm. Worth it where `conic_arc_nurbs` and `trimmed_native`
   live; judge after stage 3.
5. **Bindings.** `bindings/wasm` and `bindings/python` take and return bare
   `f64` across the FFI boundary and should keep doing so. Wrap on entry, unwrap
   on exit, and let the doc comments keep saying which unit the JS or Python
   caller is handing over.

## Notes for the implementation

- **The serde wire format must not change.** `Interval` is serialized inside
  `TrimmedCurve` inside `Model`. `#[serde(transparent)]` on `Param<S>`, with the
  `PhantomData` skipped, keeps a parameter on the wire as a bare number and an
  interval as `{start, end}`, exactly as today. `bound = ""` is required, or the
  derive demands `S: Serialize`.
- **The derives need the markers to derive too.** `#[derive(Clone, Copy, Debug,
  PartialEq)]` on a type holding `PhantomData<S>` generates `impl<S: Clone>`, so
  the marker structs derive the same set and no manual impl is needed.
- **`Interval::start` and `::end` are public and read everywhere.** Keeping them
  public as `Param<S>` is what makes the migration tractable; the open-coded
  arithmetic over them is what `at`, `fraction_of` and (stage 3) `Delta`
  replace.
