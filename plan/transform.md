# Transforms: a rigid motion, a general affine, and what the model owes each

Status: **In progress** — step 1 below is implemented. `Rigid` lives in
`geometry::transform`, `moved` has replaced `rotated` and `translated` on
`CurveGeometry` and `SurfaceGeometry`, `builders::transform::rigid` moves a
`Model`, and `modeling::transform` carries the four chainable `Shape` verbs.
`Transform<D>` and everything downstream of it (steps 2–6) do not exist yet.

This plan states the architecture in two halves, because the problem has two
halves. **`Rigid`** is the motion a user asks for almost every time — place this
here, turn it there, sit this part on that face — and it is total, exact and
free of every difficulty below. **`Transform3`** is the general affine map, and
it is where scale, shear and mirror live, along with all of the cost.

They are separate types on purpose. The rest of this document is the argument
for that and the consequences of it.

## The shape of the problem

Moving a shape is not "map every point". A `Model` stores three kinds of thing,
and a transform touches them differently:

| Stored | Where | What a transform does to it |
|---|---|---|
| **positions** | `VertexAttr::point` | mapped directly; nothing else to say |
| **3D supports** | `EdgeAttr::curve`, `FaceAttr::surface` | mapped, but the support's *own parameter* may change meaning |
| **parameter-space data** | `FaceAttr::pcurves`, `LoopDefinition` | lives in a surface's `(u, v)`, so it moves only when that parameterization moves |

The third row is the whole difficulty, and it exists because of a decision the
kernel already made: **a span is derived on an edge and stored on a pcurve.**
`EdgeAttr` holds no interval — `Edge::trimmed_curve()` re-derives it from the
bounding vertices — so a 3D curve whose parameterization shifts under a
transform costs nothing, because the span re-derives from the transformed
vertices. A pcurve has no vertices to derive from and therefore stores its
`Interval` outright. So pcurves, and only pcurves, have to be pushed through
whatever the transform did to their surface's parameterization.

That gives the central contract of the general design:

> A support does not merely report its transformed self. It reports its
> transformed self **together with the map from its old parameters to its new
> ones**, and the two are returned as one value so they cannot drift apart.

And it gives the reason the rigid case deserves its own type: under a rigid
motion that map is the identity on every support in the kernel, so the entire
contract collapses to nothing.

## `Rigid`

```rust
/// A rigid motion: a rotation followed by a translation.
///
/// No scale, no shear, no reflection. Distances, angles and handedness are all
/// preserved — which is exactly what makes every operation on this type total.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Rigid(Isometry3<f64>);
```

Everything that makes the general case hard is absent here, and not by
coincidence:

- every 3D curve's parameterization is preserved, so derived spans are
  untouched;
- every surface's parameterization is preserved, so **no pcurve moves and no
  `LoopDefinition` changes** — the stored parameter-space data is bit-identical
  afterwards;
- the determinant is `+1`, so no orientation reversal and no inside-out solid;
- every analytic type survives as itself, so nothing degrades to NURBS and there
  is nothing to report;
- and there is no failure mode at all.

So the rigid path is not the general path with a faster branch. It is a smaller
operation that shares a word. Giving it a type lets the signatures say so, and
`Result` disappears from the whole surface:

```rust
let bracket = block(40.0, 20.0, 6.0)?
    .rotated(Axis3::z(), Rad64::QUARTER_TURN)
    .translated(Vector3::new(0.0, 0.0, 12.0));
```

No `?` after the first line, nothing to unwrap, no report to ignore. That
ergonomics is a consequence of the type, not decoration added on top of a
fallible API.

### Why a quaternion rather than a matrix

`Rigid` wraps `nalgebra::Isometry3` — a unit quaternion plus a translation
vector — rather than holding the same motion as a 3×3 matrix.

The reason is drift. A `Frame` is orthonormal by construction: `from_xy` and
`from_xz` re-derive the third axis with a cross product, and every analytic
surface in the kernel holds one. Push a rigid motion through a general matrix
path and those axes are re-derived on every application. Composing a rotation
with itself sixty times to lay out a circular pattern is a routine thing to
write, and with a matrix the sixtieth frame is measurably out of square. A unit
quaternion renormalizes to exactly the constraint it must satisfy, so
composition stays rigid however long the chain.

So the rigid path is the numerically better one as well as the shorter one, and
the general path should **delegate to it** rather than reimplement it:

```rust
impl Surface {
    pub fn transformed(&self, t: &Transform3) -> Result<TransformedSurface, TransformError> {
        if let Some(rigid) = t.as_rigid() {
            return Ok(TransformedSurface::parameters_unchanged(self.moved(&rigid)));
        }
        // ... the general arms
    }
}
```

One check, written once at the enum level rather than in each of the eight
surface arms. It makes a rigid `Transform3` *exact* rather than merely correct,
which is what matters for code that composes a placement out of parts before
applying it.

### Constructors and algebra

```rust
Rigid::identity()
Rigid::translation(Vector3<f64>)
Rigid::rotation(Axis3, Rad64)
Rigid::between_frames(from: &Frame, to: &Frame)

impl Rigid {
    pub fn compose(self, other: Self) -> Self;   // `a.compose(b)` applies `b` after `a`
    pub fn inverse(self) -> Self;                // total
    pub fn apply(self, point: Point3) -> Point3;
    pub fn apply_vector(self, vector: Vector3<f64>) -> Vector3<f64>;
}
```

`compose` follows `Orientation::compose`'s argument order exactly — the house
convention already exists and there is no reason to have two. A `Mul` impl may
sit on top, but `compose` is the documented one, because `*` has no argument
names to hang the order on and every kernel gets this wrong once.

`Rad64` rather than a bare `f64`: `revolve` already takes it, and a rotation is
precisely where a unit mistake is silent and expensive.

`between_frames` is rigid with no caveat *because* `Frame` is always
right-handed — there is no pair of frames it could be handed that would need a
reflection. That is a guarantee the existing type gives this one for free, and
it makes `Rigid::between_frames` the natural spelling of "put this part where
that mating face is".

### Which side applies to which

The transform applies **primitives**; ngk's own types apply **themselves**. That
split is forced rather than chosen:

```rust
r.apply(point);        r.apply_vector(v);     // nalgebra types
curve.moved(&r);       surface.moved(&r);     frame.moved(&r);
```

`Point3` and `Vector3` are type aliases for nalgebra types
(`pub type Point3 = NPoint3<f64>`), so no inherent method can be added to them.
Writing `point.moved(&r)` would mean introducing an extension trait, and an
import at every call site, to spell what `r.apply(point)` already says. So the
transform applies them.

For `Curve`, `Surface`, `Frame`, `BBox` the argument runs the other way. The
per-type work is a match over eight surface variants; that dispatch belongs on
`Surface`, not inside `Rigid`, where it would make the transform type depend on
every geometric type in the crate. And the object-first spelling is what lets a
chain read left to right.

The rule to state in the doc comments: **whoever owns the match owns the
method.** A primitive has no match, so the transform applies it.

`Rigid` is closed under composition and inversion, so a user cannot fall out of
the infallible world by accident. Widening is explicit:

```rust
impl From<Rigid> for Transform3 { .. }
impl Transform3 { pub fn as_rigid(&self) -> Option<Rigid>; }   // tolerance-based
```

There is deliberately **no `Mul` between `Rigid` and `Transform3`**. Implicit
mixing is how a user ends up holding a `Transform3` without noticing their code
just became fallible. `Transform3::from(rigid).compose(scale)` costs one line
and says what happened.

### Two small additions that pay for themselves

`Axis3::new(Point3::origin(), Vector3::z())` already appears eight times in the
revolve tests alone. The rigid API makes it the most common expression in the
crate, so:

```rust
impl<const D: usize> Axis<D> { /* ... */ }
impl Axis3 { pub fn x() -> Self; pub fn y() -> Self; pub fn z() -> Self; }  // through the origin
```

And `Frame::moved(&self, r: &Rigid) -> Frame`, which is what every analytic
surface's own implementation is built out of.

### It is also the type an assembly will need

`exchange/step/mod.rs` records that exporting an assembly "needs a hierarchy of
named solids with transforms, which NGK has [not]". The transform in that
sentence is a rigid one: STEP's own `AXIS2_PLACEMENT_3D` is a frame, and an
assembly instance is a placement. See
[Not a `Location`](#not-a-location-where-a-placement-is-allowed-to-live) for
where such a layer may and may not sit; the point here is only that `Rigid` is
shaped to be its field and should not have to be replaced when it arrives.

## `Transform<D>`

The general affine map, for the cases `Rigid` deliberately excludes.

```rust
pub struct Transform<const D: usize> {
    linear: SMatrix<f64, D, D>,
    translation: SVector<f64, D>,
}

pub type Transform3 = Transform<3>;
pub type Transform2 = Transform<2>;
```

Generic over dimension, mirroring the existing `Axis<D>`. `Transform2` is not
decoration: a surface's induced parameter remap *is* a 2D affine map, and
pushing a pcurve through it is the same operation one dimension down over the
same `Curve2` / `TrimmedCurve2` types the 2D side already has. Writing the 2D
case as a separate ad-hoc struct would mean writing the curve transformation
logic twice.

### Invertible by construction

`Transform::new` **refuses a singular linear part.** A collapsing map is not a
transform of a shape: it destroys the topology it is handed, and no downstream
operation has a sensible answer for a face whose surface is a line. Refusing at
construction buys a real invariant — `inverse()` is total, composition never
degrades, and no application site has to re-check.

Singularity is decided against `LINEAR_TOLERANCE` on the determinant scaled by
the matrix norm, not against exact zero.

### Constructors

```rust
Transform3::identity()
Transform3::scaling(factor: f64)                    // uniform, about the world origin
Transform3::scaling_about(Point3, factor: f64)
Transform3::scaling_axes(Frame, Vector3<f64>)       // non-uniform, in a frame's axes
Transform3::mirror(Plane)
Transform3::shear(Frame, /* spelling TBD when shear lands */)
```

Translation and rotation are absent on purpose: they are `Rigid`, and
`Transform3::from(Rigid::translation(v))` is how you get one here. One spelling
per concept.

`scaling_about` and `scaling_axes` taking a centre and a frame, rather than
making the caller sandwich a translation, is not sugar: a scale about a point
written as `translate(-c) ∘ scale ∘ translate(c)` is three matrix products whose
round-off leaves the centre slightly moved, and the centre is exactly the point
a user placed deliberately.

### Classification is derived, never stored

```rust
pub enum TransformKind { Identity, Rigid, Similarity, Affine }

impl<const D: usize> Transform<D> {
    pub fn kind(self) -> TransformKind;
    pub fn is_orientation_reversing(self) -> bool;  // determinant < 0
    pub fn uniform_scale(self) -> Option<f64>;      // Some for Similarity and tighter
}
```

These are **computed from the matrix on demand**, not carried as a second field
beside it. A stored flag is a second source of truth that every constructor and
every `compose` has to maintain, and the first one that forgets produces a
"rigid" transform that is not rigid — which the supports below would then
believe. The tests are a few dot products and cost nothing next to the geometry
work they gate.

Classification is tolerance-based (`ANGULAR_TOLERANCE` on axis orthogonality,
`LINEAR_TOLERANCE` on unit length). Composing two rigid transforms drifts, and
the drift must not silently promote a rigid motion to a general affine and lose
every analytic type with it. This is also what `as_rigid()` is built on, so its
tolerance is load-bearing rather than cosmetic.

## What a support owes

`CurveGeometry` and `SurfaceGeometry` already exist as the reviewable checklist
of what a new analytic type must implement. Both transforms belong there, and
**`rotated` and `translated` are replaced rather than joined** — three ways to
move a curve is three places to be inconsistent, and those two are exactly
`Rigid::rotation` and `Rigid::translation` applied.

```rust
trait CurveGeometry {
    /// Rigid: total, exact, parameterization preserved.
    fn moved(&self, r: &Rigid) -> Self;

    /// General affine: may change the parameterization, may change the type.
    fn transformed(&self, t: &Transform3) -> Result<Self, TransformError>;
}

trait SurfaceGeometry {
    fn moved(&self, r: &Rigid) -> Self;
    fn transformed(&self, t: &Transform3) -> Result<TransformedSurface, TransformError>;
}
```

`moved` returns the value bare everywhere: no `Result`, no wrapper, because
there is no failure and no remap to carry.

### Exactly one wrapper, and the rule that puts it there

> A transform returns a parameter remap **only where a stored parameter has to
> survive it.** Everywhere else it returns the plain value.

Apply that rule and the kernel has exactly two stored parameters:

| Stored parameter | Survives via |
|---|---|
| `FaceAttr::pcurves` — a pcurve lives in its surface's `(u, v)` | the surface's remap, which must escape to the caller |
| `TrimmedCurve2::interval` — a pcurve has no vertices to derive a span from | the 2D support's remap, which never escapes `geometry::dim2` |

So **one public wrapper**:

```rust
/// A transformed surface together with the map its parameters underwent.
pub struct TransformedSurface { pub surface: Surface, pub parameters: Transform2 }
```

It exists because the remap is consumed by data stored *next to* the surface
rather than inside it. `builders::transform::affine` gets a new surface and, in
the same value, the map it must push that face's pcurves through. The
alternative is two calls — `surface.transformed(t)` and
`surface.param_remap(t)` — which is a pair a caller can forget half of, and
forgetting it produces pcurves that are silently on the wrong part of the right
surface.

**3D curves return a bare `Curve`.** Their remap has no consumer: an edge's span
re-derives from its transformed vertices, a marked edge's from its corner, an
unmarked edge's is its whole support. `TrimmedCurve` appears only in transient
solver output (`AnalyticSection`, `FaceImprint`, `IntersectionSpan`), never in
model storage, so nothing outlives a transform holding a 3D parameter. Returning
a remap nobody reads is a wrapper to unwrap at every call site for nothing.

**The 2D remap never becomes public.** Every pcurve is a `TrimmedCurve2` — the
kernel already guarantees that — so `TrimmedCurve2::transformed(&Transform2)`
can be the only public 2D entry point, mapping support and interval together
behind its own signature. The `ParamRemap { scale, offset }` that carries the
interval across is crate-private inside `geometry::dim2`.

### Under a similarity, every 3D *curve* is remap-free

| Curve | Under a similarity of factor `s` | Remap |
|---|---|---|
| `Line` | axis maps; the `scale` field absorbs `s` | identity |
| `Circle` | plane maps, radius `× s` | identity (the parameter is the angle) |
| `Ellipse` | frame maps, both radii `× s` | identity |
| `NurbsCurve` | control points map affinely, knots untouched | identity |

Not a coincidence — `Line` carries a `scale` field precisely so its affine
parameter can absorb one — and it matters: an edge's derived span, a
`TrimmedCurve`'s stored interval, and a `RuledSurface`'s or
`SurfaceOfRevolution`'s `u` direction all survive a similarity untouched.

### Under a similarity, surfaces remap by a diagonal scale

| Surface | `(u, v)` are | Remap under factor `s` |
|---|---|---|
| `Plane` | two distances | `(s, s)` |
| `Cylinder` | angle, distance | `(1, s)` |
| `Sphere` | two angles | `(1, 1)` |
| `Cone` | angle, distance | `(1, s)` |
| `Torus` | two angles | `(1, 1)` |
| `RuledSurface` | curve parameter, ruling multiple | `(1, 1)` — the direction vector absorbs `s` |
| `SurfaceOfRevolution` | curve parameter, angle | `(1, 1)` |
| `NurbsSurface` | knot domains | `(1, 1)` |

At `s = 1` every row is the identity, which is the table's other job: it is the
proof of the claim that opened the `Rigid` section. Under a uniform scale
exactly two rows are non-trivial.

This table is the implementation checklist, and each row wants a test asserting
`transformed(t).geometry.point_at(remap(p)) == t * self.point_at(p)`, plus the
`moved` equivalent with no remap at all.

### When the analytic type cannot carry the transform

A non-uniform scale or a shear leaves `Sphere`, `Cylinder`, `Cone` and `Torus`
with no analytic type to land on — an ellipsoid and a general quadric are not in
the `Surface` enum, and putting them there is a much larger project than this
one. `Circle → Ellipse` survives; `Sphere → ellipsoid` does not.

**The decision is to convert, not to refuse.** `to_nurbs` is exact as a point
set, an affine map of a rational NURBS is again a rational NURBS with the same
weights and the same knots, and therefore:

- the transformed surface is geometrically exact, and
- its remap is the **NURBS** parameterization, so the stored pcurve interval
  must be carried through the support's existing `nurbs_param_map()` — the
  projective conic map in `reparam.rs` — before the new remap is applied.

That second point is the whole hazard of this path and the reason the conversion
must go *through* the remap machinery rather than around it. `to_nurbs` does not
preserve parameterization; the module doc in `traits.rs` says so, and a pcurve
whose interval was not carried through `Reparam::conic_arc` lands on the wrong
arc of the right curve.

Nothing is refused. `TransformError` is reserved for what genuinely has no
answer: a singular transform, or a NURBS conversion that itself fails.

### The degradation is a test obligation, not a return value

The analytic-first intersection dispatch is a correctness and robustness
mechanism, not only a speed one, so a shape that quietly stops having any
analytic support is a shape whose Booleans get worse for reasons nobody can see.
That is a real risk and it is worth writing down. It is **not** worth a
`TransformReport` value threaded through every signature, because the consumer
that would read one does not exist: nothing in the crate logs, no UI surfaces
it, and a `Vec` allocated on every transform to be dropped unread is cost
without a reader.

The risk that actually bites is not "the user was not told". It is **an
implementation that converts when it did not have to** — reaching for the NURBS
path on a uniform scale because it is one arm instead of eight. That is a
regression a test catches and a report does not:

> Every row of the similarity table above gets a test asserting the transformed
> support is still the **same enum variant**. A uniformly scaled `Cylinder` is
> still `Surface::Cylinder`; a rotated `Torus` is still `Surface::Torus`.

Keep the conversion in **one function**, so that if a caller ever does need to
know, a report or a callback can be threaded through it without touching a
single call site. That is the whole cost of deferring this, and it is close to
zero.

## What the model owes

### Under a rigid motion: three loops

Map every `VertexAttr::point`, every `EdgeAttr::curve`, every
`FaceAttr::surface`. That is the entire operation. No pcurve is read, no
`LoopDefinition` is touched, no dart is created or destroyed, no cell changes
owner, no key moves. It is the smallest builder in the crate and it cannot fail.

### Under a general affine: vertices and edges are still nearly free

`VertexAttr::point` maps. `EdgeAttr::curve` maps through
`CurveGeometry::transformed`, and its span re-derives from the transformed
vertices, so the curve's remap is *discarded* at this level. A marked edge's
span re-derives from its corner; an unmarked edge's span is its whole support.
Nothing stored needs fixing.

### Under a general affine: faces carry the work

For each face:

1. `surface.transformed(t)` yields the new surface and a `Transform2` remap;
2. every pcurve in `FaceAttr::pcurves` is pushed through that remap — support
   *and* interval, through `TrimmedCurve2`'s own transform, the 2D mirror of the
   3D one;
3. every `LoopDefinition` is checked against the remap: a `Wrapping { axis }` or
   `Capping { axis, .. }` follows an axis swap, and a `Capping { side }` flips
   when the remap reverses that axis.

Step 3 is small but not optional. `DomainSide::Low` means "toward decreasing
parameter", and a remap with a negative scale on that axis makes it `High`. A
spherical cap that keeps `Low` through a mirror is closed by the wrong pole.

### The sting: an orientation-reversing transform turns a solid inside out

This is the one place where mapping the geometry correctly still produces a
wrong shape, and it has to be understood before mirror is implemented.

`Face::normal_at` does not read the surface normal. It reads the surface normal
**and flips it when the outer loop's signed area in `(u, v)` is negative.** The
winding of the stored pcurves is what says which side of the support is out.

Work a mirror through. Take the unit cube's top face: plane at `z = 1`,
`x_dir = +X`, `y_dir = +Y`, normal `+Z`, outer loop counter-clockwise, outward
normal `+Z`. Mirror in `z = 0`. The image's outward normal must be `-Z`.

`Frame` forces right-handedness — `from_xy` and `from_xz` both re-derive the
third axis with a cross product — so the mirrored plane comes out as
`origin (0,0,-1)`, `x_dir +X`, `y_dir -Y`, normal `-Z`. Correct surface. The
induced remap is `(u, v) → (u, -v)`, honestly reported. Push the pcurves through
it and the loop's signed area flips sign, so `normal_at` flips the surface
normal and returns `+Z`.

Inward. Every face, every mirrored solid.

The compensation is topological, and it is exactly the handedness the remap
reported: **when the induced remap on a face has negative determinant, that
face's boundary loops must be reversed.** A reversed loop's winding flips back,
`normal_at` stops flipping, and the outward normal is `-Z` as it should be.
Composing two mirrors reverses twice and so reverses nothing, which is right.

Concretely, reversing a face's loops means replacing each `LoopDefinition`'s
seed by its `alpha0` partner, and re-keying each pcurve onto its `alpha0`
partner with `TrimmedCurve2::reversed()` applied. It is the static form of what
`Orientation::Reversed` already does dynamically.

**Apply the compensation in exactly one place.** `Face::normal_at` combines the
stored pcurve winding with the `sense` the traversal context supplies, and that
`sense` comes from the shell walk rooted at `SolidAttr::outer_shell`. Flipping
the shell anchor *and* the loop seeds double-flips and lands back on inward.
Normalizing it into the stored loops is the right choice: a transformed model is
then in the same canonical state whether or not a solid happens to own the face,
and a free-standing `Shape<FaceTag>` — which has no solid and so no anchor to
flip — is handled by the same code.

The `alpha` links themselves are never touched. A GMap carries no handedness;
only the geometry and the oriented anchors do.

**This needs its own verification against the code.** The reasoning above is
derived from `Face::normal_at`, `boundary_signed_area` and `FaceAttr::boundary`
as they stand, and the claim that no other stored anchor needs flipping —
`ProfileAttr::dart`, `SheetAttr::root`, `SolidAttr::outer_shell`,
`EdgeAttr::dart` — is argued, not yet tested. A mirrored solid whose volume
comes out negative is the first test to write, before any of the machinery.

### Not touched, by either

The GMap, the embedding records, every key, and the revision semantics. A
transform is a pure geometry rewrite over a fixed topology. Stating that up front
is what keeps it from growing into something that rebuilds the map.

The realization and derived-index caches invalidate on mutation as they already
do.

## Layering and entry points

Three modules, following the existing layering rather than inventing one.

### `src/geometry/transform.rs`

`Rigid`, `Transform<D>`, their constructors and algebra, `TransformKind`,
`TransformedSurface`, `TransformError`. Plus `moved` / `transformed` on `Frame`,
`Axis<D>` and `BBox`, which are one-liners and belong with the types; `Point3`
and `Vector3` are applied by the transform instead, for the reason given above.

`moved` and `transformed` on `CurveGeometry`, `Curve2Geometry` and
`SurfaceGeometry` replace `rotated` and `translated` on the traits and on the
enums. Every call site of the old methods moves; `revolve` and `sweep` are the
main ones.

### `src/builders/transform.rs`

```rust
pub fn rigid<P: Payload>(model: &mut Model<P>, r: &Rigid);

pub fn affine<P: Payload>(model: &mut Model<P>, t: &Transform3)
    -> Result<(), TransformError>;
```

Two functions and no more. Every named transform — mirror, scale, shear — is one
of these two with a `Transform3` built for it, and a builder that re-spelled each
one would be five wrappers over a match that has already happened inside
`Transform3`. One transaction each, per the rule that nothing outside a builder
opens one. `rigid` returns nothing because there is nothing it could fail at.

A `pub fn reverse_orientation<P: Payload>(model: &mut Model<P>)` falls out of the
mirror work and is independently useful — `Solid::reversed()` has wanted it for
other reasons — so it should be written as its own builder that `affine` calls,
not buried inside it.

### `src/modeling/transform.rs`

**Builders stay at two; modeling spells out every named operation.** This is
where a user looks for "how do I mirror a thing", and `mirrored(shape, plane)`
is the answer, not `transformed(shape, &Transform3::mirror(plane))`. Each is
three lines over the two builders.

```rust
// rigid — infallible
pub fn moved     <K, P>(shape: Shape<K, P>, r: Rigid)                        -> Shape<K, P>;
pub fn translated<K, P>(shape: Shape<K, P>, offset: Vector3<f64>)            -> Shape<K, P>;
pub fn rotated   <K, P>(shape: Shape<K, P>, axis: Axis3, angle: Rad64)       -> Shape<K, P>;
pub fn placed    <K, P>(shape: Shape<K, P>, from: &Frame, to: &Frame)        -> Shape<K, P>;

// affine — fallible
pub fn scaled      <K, P>(shape: Shape<K, P>, factor: f64)                   -> Result<Shape<K, P>, TransformError>;
pub fn scaled_about<K, P>(shape: Shape<K, P>, centre: Point3, factor: f64)   -> Result<Shape<K, P>, TransformError>;
pub fn scaled_axes <K, P>(shape: Shape<K, P>, frame: Frame, f: Vector3<f64>) -> Result<Shape<K, P>, TransformError>;
pub fn mirrored    <K, P>(shape: Shape<K, P>, plane: Plane)                  -> Result<Shape<K, P>, TransformError>;
pub fn sheared     <K, P>(shape: Shape<K, P>, /* … */)                       -> Result<Shape<K, P>, TransformError>;
pub fn transformed <K, P>(shape: Shape<K, P>, t: &Transform3)                -> Result<Shape<K, P>, TransformError>;
```

All of them return the transformed shape and nothing else. There is no outcome
type to unwrap, because there is no report to carry — see above.

`mirrored` is fallible along with the rest even though a reflection is a
similarity and degrades nothing, so that one signature covers everything built
on `Transform3`. Splitting it out would make `Rigid` no longer mean "the total
ones".

### The `Shape` verbs

```rust
impl<K: ShapeKind, P: Payload> Shape<K, P> {
    pub fn moved(self, r: Rigid) -> Self;
    pub fn translated(self, offset: Vector3<f64>) -> Self;
    pub fn rotated(self, axis: Axis3, angle: Rad64) -> Self;
    pub fn placed(self, from: &Frame, to: &Frame) -> Self;
}
```

**Inherent methods exist for the rigid verbs only, and the rule is: inherent
means chainable means infallible.** Chaining is the entire thing an inherent
method buys over a free function, and a `?` in the middle of a chain gives it
back. The affine operations are free functions in `modeling::transform`, where
they sit beside every other fallible operation in the crate.

**No `Transformable` trait at the shape level.** There is one implementor family
— `Shape<K, P>` is already generic over every shape kind — so a trait would add
an import to every call site and state nothing the inherent methods do not. The
trait is earned on `CurveGeometry` / `SurfaceGeometry`, where a dozen analytic
types must be held to the same contract, and nowhere else.

### Considered and rejected: one method name for both

A sealed trait with a generic associated type would let a single `transformed`
return `Self` for a `Rigid` and `Result<…>` for a `Transform3`, so there is one
verb. Rejected: the signature stops saying what it does, a reader has to resolve
an associated type to learn whether the call can fail, and the compiler's errors
end up being about the trait rather than about the geometry. Two operations that
differ in totality, in cost and in what they can destroy are what two names are
for.

### Bindings

Both types map cleanly onto a Python class with the same constructors and a
`__mul__`, and onto wasm. Not on the critical path, but both were designed as
plain values with no lifetimes for this reason.

## Not a `Location`: where a placement is allowed to live

OCCT gives every `TopoDS_Shape` a `TopLoc_Location` — a transform carried
*beside* shared underlying geometry and composed lazily on every query. It buys
two things: copying a shape to a new position is O(1) and shares its geometry,
and an assembly is a tree of located shapes.

**NGK should not do this to `Model`,** and the reason is the invariant the
kernel is built on: `Model` owns everything a shape is, and a query answers from
what it owns. Add a location and that stops being true. Every `point_at`, every
`Face::normal_at`, every tessellation, every intersection and every Boolean
would have to compose it, and "did you remember to apply the location" becomes a
question that can be asked at several thousand call sites — which is exactly the
bug class OCCT is known for. It also splits identity in two: OCCT needs both
`IsSame` (ignoring location) and `IsEqual` (including it), and users trip over
the distinction permanently. NGK has one identity, the key, and it should keep
having one.

So: **geometry in a `Model` is in world coordinates, always, with no pending
transform.** That is what makes `builders::transform::rigid` three loops rather
than a pervasive redesign.

### Where instancing does belong

The saving is real, it is just one layer up. An **assembly** is a tree of
placed shapes that are never merged into one another — a plate and the thousand
identical bolts sitting in it. There, one `Model` tessellated once and drawn a
thousand times with a thousand `Rigid`s is an enormous win, and it is exactly
what STEP's `MAPPED_ITEM` wants on export.

```rust
struct Instance<P: Payload> { shape: Arc<Model<P>>, placement: Rigid }
```

`Rigid` was designed to be this node's field: `Copy`, serializable, no
lifetimes, no model reference, and stable under long composition chains up a
tree. Nothing in this plan builds an assembly layer, but nothing in it has to be
replaced when one arrives.

**An `Instance` must not behave like a `Shape`.** That is the trap: give it the
same traversal API and every query composes a placement again, and the location
has been reinvented one layer up. Keep them distinct — an assembly is a
container of placed shapes, and `instance.baked()` applies the `Rigid` once and
hands back a `Model` you can model with. Baking is the door between the two
worlds, and it is cheap precisely because the rigid builder is.

### The plate with a thousand holes is the other case

Worth being explicit, because it is the example that motivated the question and
it is the one instancing does **not** help. A plate with a thousand holes is
*one solid*: the holes have to be merged into its topology for the result to be
a valid B-Rep, so there are a thousand Booleans and one `Model` with a few
thousand faces at the end. No placement layer changes that — the cost is the
Boolean, not the positioning.

What helps there is a different thing with a similar name: a **pattern**
operation that generates the transforms and drives the cuts,
`circular_pattern(shape, axis, count)` and `rectangular_pattern(…)`, living in
`modeling` on top of `Rigid` and the Boolean. And beneath it, an optimization
worth noting and not pursuing here: a thousand identical holes cut into a planar
face produce a thousand *identical* imprints differing by a `Rigid`, so the
imprint geometry could be computed once and moved. That is a Boolean-internals
improvement, not an architectural one, and it should be measured before it is
believed.

## Ordering

`Rigid` is not step one of the general design. It is a complete, useful,
shippable slice that needs almost none of it.

1. **`Rigid`, and the whole vertical slice through it.** *(Done.)* `Rigid` and its
   algebra, `Axis3::{x,y,z}`, `Frame::moved`, `moved` on every curve and surface
   (replacing `rotated` / `translated`), `builders::transform::rigid`, the four
   `Shape` verbs. No `Transform3`, no remap type, no error type, no report.
   Tests: round-trip `r` then `r.inverse()`; a drift test that composes a
   rotation sixty times and checks the last frame is still orthonormal; and the
   assertion that every pcurve is **bit-identical** afterwards, which is the
   claim this whole design rests on.
2. **`Transform<D>`, algebra, classification, `Transform2`.** Value-level only,
   no geometry touched. Composition associativity, inverse round-trip,
   classification stability under composition, `as_rigid` agreeing with `Rigid`.
3. **`transformed` on the geometry traits, similarities only**, with the rigid
   delegation at the enum level. The remap table is the test matrix.
   Non-similarity arms return `TransformError::NotYetSupported` naming the
   support — a stated gap rather than a wrong answer.
4. **`builders::transform::affine` for orientation-preserving transforms.**
   Pcurve remap and loop-definition fixups. A scaled block's volume; a scaled
   cylinder's face count with its analytic supports intact.
5. **`reverse_orientation` and mirror.** Write the failing test first — mirror a
   block, assert positive volume and every face normal outward — then build the
   reversal under it.
6. **Non-uniform scale and shear, with the NURBS fallback.** The largest piece
   and the one with the least pressure behind it: it needs the conic
   reparameterization carried correctly on every converted pcurve, which is
   where this can quietly go wrong. Keep the conversion in one function.

Steps 2–6 can wait indefinitely behind step 1 without leaving anything
inconsistent, because `moved` is a complete contract on its own rather than a
special case of one that does not exist yet.

## Open questions

- **Is `Rigid` the right name?** `Motion` and `Placement` both read well at call
  sites. `Isometry` is rejected: a reflection is an isometry, and this type
  exists precisely to exclude one.
- **Does `Profile` have a direction that must follow a loop reversal?**
  `ProfileAttr::dart` carries a default traversal direction. A standalone wire
  has no normal and no side, so reversal is meaningless there; a profile
  bounding a face may want to follow. Decide when `reverse_orientation` is
  written, and say which in its doc comment.
- **Does a pattern operation belong in this plan or its own?** It needs nothing
  from here beyond `Rigid`, and everything else it needs is the Boolean. Most
  likely its own, once `Rigid` exists.
- **Should `Transform3::scaling_axes` take a `Frame` or a bare `Vector3`?** A
  frame is strictly more general and is what makes "scale this along that edge"
  expressible, but it makes the common axis-aligned case wordier. A bare-vector
  convenience alongside is probably right.
- **Does anything outside `geometry` call `rotated` / `translated` in a way that
  wants a remap it currently cannot report?** `revolve` and `sweep` build
  supports by rotating and translating curves and rely on the parameterization
  being preserved. Under `moved` they are unaffected by construction — but that
  should be confirmed by reading them, not assumed.
