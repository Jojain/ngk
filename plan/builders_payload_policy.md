# Builders, payload, policy

Conclusions from an architecture review of the two user-facing layers
(`src/builders/`, `src/modeling/`) and the payload/policy mechanism they run
under. Decisions are recorded here; the "Ideas to keep in mind" section is
explicitly not part of the decided work.

## 1. One modeling layer

`src/modeling/` is not a layer. Every function in it does the same three
things: `Model::new()`, call a builder, wrap in `Shape`. It has no concept of
its own, and it has already drifted from the layer it mirrors:

- `modeling::solids` validates its arguments and collapses every builder error
  into `PrimitiveError::FaceCreationFailed` / `SolidCreationFailed`, discarding
  the cause. `modeling::faces` validates nothing and forwards the real error.
  Two adjacent modules, two contradictory error policies, for the same job.
- `block`, `cylinder`, `rectangle`, `circle`, `annulus` and `polygon` are pinned
  to `StandardPayload`, while `extrude_face`, `revolve_face`, `fuse` and
  `from_profile` are generic over `P`. The boundary is invisible to a reader.

**`builders` is the modeling API**: `f(&mut Model<P>, keys, params) ->
Result<TypedResult, E>`, generic over `P`, one transaction each.


The value-semantics API that `modeling` was reaching for belongs on `Shape`, as
`impl` blocks that forward to the builders and change nothing else — no second
error type, no narrower payload bound. An operation is then declared in exactly
one place and cannot drift.

```rust
impl<K: ShapeKind, P: Payload> Shape<K, P> {
    /// Runs one builder into a fresh model.
    pub fn build<E>(f: impl FnOnce(&mut Model<P>) -> Result<K::Handle, E>) -> Result<Self, E> {
        let mut m = Model::new();
        let h = f(&mut m)?;
        Ok(Shape::new(m, h))
    }

    /// Runs a builder over this shape's model, rebinding the primary handle.
    pub fn then<K2: ShapeKind, E>(
        mut self,
        f: impl FnOnce(&mut Model<P>, K::Handle) -> Result<K2::Handle, E>,
    ) -> Result<Shape<K2, P>, E> {
        let h = f(&mut self.model, self.handle)?;
        Ok(Shape::new(self.model, h))
    }
}
```

`PrimitiveError` goes with `src/modeling/`; builder errors reach the caller
unchanged.

### Non-goal: traversal and selection

The typed views already carry the traversal an application needs —
`Solid::faces/edges/vertices/shells`, `Face::edges/vertices/loops/outer_loop/
inner_loops`, `Edge::faces/vertices/sheets`. Nothing is added here. In
particular there are no filter/sort selection combinators: a caller writes its
own iterator chain over the `Vec` a view hands back.

## 2. `ModelEdit` is crate-internal

Every mutation method on `ModelEdit` is `pub` today — `add_dart`, `link`,
`unlink`, `sew`, `own_cell`, the `add_*_derived_from` family, and the six
`*_attr_mut` accessors — and `Model::transaction` hands `&mut ModelEdit` to any
caller. Application code can therefore build arbitrary topology and declare
arbitrary lineage.

Lineage declaration in particular must not be reachable from outside: a caller
declaring `Origin::Derived` asserts something only the kernel can verify, and
the policy would then be reading a claim rather than a fact.

**`ModelEdit` and its mutation surface become `pub(crate)`.** The rule this
enforces:

> The kernel declares what happened; the application decides what it means.

## 3. An operation returns everything it knew while building

`chamfer` returns `()`. `add_extruded_face` returns one `SolidKey` while
internally holding every cap and lateral face it made. The caller cannot
recover either — which is the real cost behind "a key is a poor API", not the
fact that keys are ids.

A builder runs under `&mut Model<P>`, so it can only return keys. It can
however return *all* of them, in a named structure:

```rust
pub struct Extrusion {
    pub solid: SolidKey,
    pub start_cap: FaceKey,
    pub end_cap: FaceKey,
    /// One per swept boundary edge, in the base face's outer-loop order,
    /// then each inner loop in `FaceAttr::inner` order.
    pub laterals: Vec<Lateral>,
}

pub struct Lateral {
    /// The base-face boundary edge this was swept from.
    pub swept_from: EdgeKey,
    pub face: FaceKey,
    pub start_edge: EdgeKey,
    pub end_edge: EdgeKey,
}
```

### What belongs in a result

Everything the operation computed that the caller cannot cheaply recompute, and
nothing else.

- `swept_from` belongs: the correspondence from a base edge to the lateral face
  it produced exists only inside the extrusion, and no traversal recovers it.
- The lateral face's own edges do not belong: `Face::edges()` already answers
  that in one step.

A result is a structure of keys and carries no lifetime, so it is `'static` and
storable. Keys are storable; views are not. That split is the point.

### Resolving a result to views

A result struct is paired with a view struct of the same shape, and one call
resolves the whole thing:

```rust
pub struct ExtrusionView<'m, P: Payload> {
    pub solid: Solid<'m, P>,
    pub start_cap: Face<'m, P>,
    pub end_cap: Face<'m, P>,
    pub laterals: Vec<LateralView<'m, P>>,
}

pub struct LateralView<'m, P: Payload> {
    pub swept_from: Edge<'m, P>,
    pub face: Face<'m, P>,
    pub start_edge: Edge<'m, P>,
    pub end_edge: Edge<'m, P>,
}

impl Extrusion {
    pub fn view<'m, P: Payload>(
        &self,
        model: &'m Model<P>,
    ) -> Result<ExtrusionView<'m, P>, StaleResult> { /* … */ }
}
```

`&self` carries no lifetime and `'m` comes from the model, so the borrow the
view holds is the model's alone — the result can be kept, moved, or stored, and
re-resolved against the model whenever a view is wanted.

**A result names the revision it was made at, and `view` refuses a mismatch.**
`Model::revision` already exists and increments on every commit.

```rust
pub struct StaleResult {
    pub made_at: u64,
    pub now: u64,
}
```

Refusing rather than resolving key by key is the point. After a later
transaction, a key in the result may still be live while the entity it names
has been split, merged, or re-anchored; handing back a view of whatever
survived would answer a question the caller did not ask. A caller that
genuinely wants the survivors reads the public key fields and asks the model
itself, where every lookup returns `Option` and absence is visible.

### When a result needs a view struct

Proportionality, not uniformity:

- One or two keys and no nesting: return the keys. The caller writes
  `model.face(key)` once and nothing is gained by a second struct.
- More than that, or any nesting: pair it with a view struct.

A derive macro generating the view struct and `view()` from the key struct is
possible; ten hand-written pairs are not a problem.

## 4. Payload

### `StandardPayload` pins must go

`builders::faces::{add_rectangle, add_square, add_circle, add_annulus,
add_polygon_with_holes}` and their staged helpers take
`&mut Model<StandardPayload>`. Nothing in them requires it — they were not
migrated when the rest of the tree became generic. Until they are
`<P: Payload>`, a user who defines a payload cannot build a rectangle, which
makes the kernel's own primitives unavailable to exactly the users the payload
mechanism exists for.

**To do:** make each of them, and every private helper they reach, generic over
`P: Payload`.

**Not to do:** do not add `Default` bounds to builder signatures to make this
work. `PreservePayload` requires `P::V: Default` and its five siblings, and that
obligation is discharged at the `impl Payload` that names `PreservePayload` as
its `Policy`. A payload naming its own policy has no such obligation, and a
builder that spelled the bound would impose one on every payload regardless.

### Keys do not cross a model

A key identifies an entity within one `Model`. Slotmap keys are
generation-counted, so a key held past the removal of what it named resolves to
`None` rather than to another entity — staleness is loud, which is the
behaviour to rely on.

What a key does not carry is which model it came from: a `FaceKey` produced by
one model type-checks against another and will resolve there, to an unrelated
face, whenever the slot and generation happen to coincide. Anything that stores
a reference to an entity across a model boundary must therefore pair the key
with the identity of the model it belongs to, rather than passing the key
alone.

## 5. Policy — current state

Recorded as it is, pending a separate decision.

`EditPolicy<P>` has eighteen methods: `*_created` / `*_merged` / `*_consumed`
for vertex, edge, profile, face, sheet and solid. The six `*_created` hooks are
required; the twelve others default to a no-op. `P::Policy` names the policy on
the payload and `Model::transaction` default-constructs it, so no call site
chooses one — except `Model::transaction_with_policy`, which exists precisely so
a call site can.

A policy produces payload data and nothing else. It reads a source's data out
of the transaction-start snapshot and returns the value a created entity
carries; it never touches topology.

### What a split does today

`add_face_split_from(source, attr)` records `Created { Split(source) }`, so
`face_created` fires for the new piece, reads the source's data from `before`,
and — under `PreservePayload` — clones it. The source itself, when it survives
the split, receives no call: `apply_policy_events` iterates `Created`, `Merged`
and `Consumed` only, so a surviving entity keeps the payload it already had.

Carrying a value from the old entity to the new portions is therefore already
what happens.

### Merge combines; split does not divide

`*_merged` takes `&mut` on the survivor's data and ownership of the removed
data, so it can sum, fold or concatenate. The split path has no counterpart:
the new piece clones, the survivor is untouched, and no hook sees the source
and its pieces together.

That fixes the kind of data a payload can currently hold:

| payload is a… | on a split | supported |
|---|---|---|
| **label** — colour, material, name, an owning feature | every piece carries it | yes |
| **quantity** — area, mass, cost, a token meant to be unique | every piece claims the whole of it | no |

For a label the current design is complete. For a quantity the surviving
entity keeps a value that is no longer true, and there is no hook through
which to correct it — the missing piece is not information reaching a hook, but
a hook for a surviving entity whose extent changed.

### Other frictions visible in the current tree

- **Two contradictory rules about who chooses the policy.** `edit.md` states
  that no call site chooses it, and then documents the call site that does. The
  reason `transaction_with_policy` exists is real — `P::Policy` is
  default-constructed and dropped at commit, so it can carry neither parameters
  in nor state out — but the two statements cannot both be the contract.
- **`Origin` carries `EditKey`, which is untyped.** `face_created` receives an
  `Origin::Split(EditKey)` whose kind it already knows, and must match it back
  to `EditKey::Face(_)` with an unreachable arm. `preserve_created`'s six
  closures in `edit.rs` are that pattern, written out six times.
- **`Origin::Derived(Vec<EditKey>)` orders its sources by convention.** The
  order is documented in each builder's prose, so a policy that reads position
  is coupled to a builder's documentation rather than to a type.
- **The `Default` bound on `PreservePayload` surfaces far from its cause.** A
  payload missing `Default` at one dimension fails trait resolution at
  `Model::transaction`, not at its `impl Payload`.
- **Six required `*_created` hooks regardless of payload shape.** A payload
  carrying data at one dimension still implements all six.

## Ideas to keep in mind

Not part of the decided work; recorded so the shape is not lost.

### A narrower public mutation capability

Making `ModelEdit` crate-internal closes application access to raw topology,
and also closes composition: an application feature that runs several kernel
operations gets one transaction per operation, so it has no atomicity, no
single policy pass, and no single rollback.

A second capability could restore composition without reopening topology
construction — a transaction scope that can invoke kernel operations and read
the staged model, holding `ModelEdit` in a private field:

```rust
pub struct Ops<'m, P: Payload> {
    edit: ModelEdit<'m, P>,
}

impl<'m, P: Payload> Ops<'m, P> {
    pub fn model(&self) -> &Model<P> { self.edit.model() }
    pub fn add_circle(&mut self, plane: Plane, radius: f64) -> Result<FaceKey, FaceCreationError>;
    pub fn extrude(&mut self, face: FaceKey, direction: Vector3<f64>) -> Result<Extrusion, ExtrudeError>;
    pub fn chamfer(&mut self, edges: &[EdgeKey], distance: f64) -> Result<Chamfer, ChamferError>;
}

impl<P: Payload> Model<P> {
    /// Runs one atomic feature: one snapshot, one commit, one policy pass.
    pub fn feature<T, E>(&mut self, f: impl FnOnce(&mut Ops<'_, P>) -> Result<T, E>) -> Result<T, E>
    where
        E: From<ModelEditError>;
}
```

Application features would then be free functions over `&mut Ops`, composing
with each other and nesting inside a larger feature within one transaction,
while still being unable to add a dart, sew a link, or declare a lineage.
