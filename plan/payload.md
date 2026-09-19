# Payload: the user data a shape carries, and what every operation owes it

Status: **Proposed** — nothing below is implemented. The storage layer described
in "What is there" exists today; the contract, the hooks and the enforcement do
not.

`Model<P>` has been generic over a `Payload` since the logical-topology
migration, and in the whole of `src/` nothing has ever instantiated it with
anything but `StandardPayload`. The parameter is not decoration — the storage
under it is complete and correct — but the two halves that would make it usable,
**a way for a caller to say what a new entity's payload should be** and **a
guarantee that no operation drops one silently**, were never built.

The cost of that compounds. Every builder added since is one more place that
creates entities with `Default` and destroys them without a word, and each is
written by copying the one before it. Twenty-eight removal sites now drop
attributes with no record; five public face builders are pinned to
`StandardPayload` because the pattern they were copied from was. This plan
stops that by making the contract impossible to forget rather than by fixing the
sites one at a time: commit refuses a transaction that will not say where its
payloads came from and where they went.

That is the point of the whole exercise. The payload layer is not the goal —
**an operation-authoring pattern that a future builder can copy and get right by
default** is the goal, and payload is the thing that pattern is about.

## What is there

Three layers, of very unequal completeness.

| Layer | Where | State |
|---|---|---|
| **Storage** | `*Attr::data`, six slotmaps on `Model<P>`, threaded through `Clone` and the serde bounds | Complete |
| **Lineage** | `add_*_split_from`, `merge_*_into`, resolved at commit against identity reconciliation | Built, barely used |
| **Policy** | `EditPolicy`, twelve hooks, applied to net externally-visible change | Built, unreachable |

The storage needs nothing. The lineage machinery is careful and correct — merge
chains resolve, transaction-start keys beat local ones, policy sees only net
change — and it is declared at a small fraction of the sites that should declare
it:

| | `add_*` | `add_*_split_from` |
|---|---|---|
| vertex | 21 | 1 |
| edge | 21 | 2 |
| profile | 27 | 3 |
| face | 21 | 4 |
| sheet | 12 | 0 |
| solid | 8 | 1 |

The policy layer is reachable only from tests. Outside `model.rs`, `edit.rs` and
the re-export, the name `EditPolicy` does not appear anywhere in `src/` or
`bindings/`: all fifty builder transactions call `Model::transaction`, which
hardcodes `PreservePayload`.

## The idea

> **Every identity a transaction creates was created for a reason, and every
> identity it destroys was destroyed for a reason. Commit refuses a transaction
> that will not name both.**

This is the same trade the kernel already makes for embedding records and for
cell occupancy, and it is made for the same reason: a builder that forgets is
caught at the boundary with a named error, rather than producing a shape that
quietly reports something false. A payload dropped without a word is exactly
that kind of falsehood — invisible while `P = StandardPayload`, and data loss
the moment it is not.

Two small vocabularies make it sayable.

**Why a transaction created an entity.** Three cases, and they are exhaustive —
an entity that arrived by being copied was not created, and is covered below:

| `Origin` | Means | Example |
|---|---|---|
| `New` | nothing in the model before this transaction explains it | a block's first corner |
| `Split(source)` | it carries on from `source`, **of the same kind** | one of the two edges a cut produces |
| `Derived(sources)` | it was produced by `sources`, **of another kind** | a loft's wall face, from the two section edges it spans |

`Split` and `Derived` differ only in whether the source has the created entity's
own kind, which is why they are one hook family and two constructors rather than
two families. `Derived` is the one genuinely new capability: the operations
users most want to carry data across are dimension-crossing — a sweep's wall
face comes from a profile edge, a loft's from two section edges, a chamfer's
from an edge — and today every one of them is recorded as `New`.

**`Derived` takes several sources, not one.** A chamfer's face comes from one
edge and a loft's wall face from two, and a single-source variant would force
the loft either to name one section arbitrarily or to fall back to `New` and
lose both. Neither is acceptable for the case this is mostly for, so the
variant carries a `Vec<EditKey>` and the one-source spelling is a constructor
over it rather than a second variant. The order is the builder's to define and
to document: a loft names its sections in traversal order, and a policy that
cares reads position, not identity.

**Copying is neither.** `Model::merge` clones attributes out of another model,
payload included, and the copy is *transport*: nothing about what an entity is
has changed, only which model holds it. So a copied entity does not reach the
creation hook at all, and its payload arrives verbatim. This is today's
behaviour and it has to stay — it is the whole mechanism by which a shape built
in one model keeps its data when it becomes an input to an operation in
another, which the use case below depends on completely.

The alternative — an `Origin::Copied` that hands the incoming payload to the
hook so a policy can rewrite it — was considered and rejected: it makes every
creation hook take a payload that is present for exactly one variant, and a
caller who wants to re-stamp what it just merged in can do so directly through
the payload write path. Transport stays silent.

**Why an entity stopped existing.** Two cases:

| Spelling | Means |
|---|---|
| `merge_*_into(survivor, removed)` | `removed`'s identity carries on inside `survivor`; its payload reaches the merge hook |
| `remove_*(key)` | nothing inherits it; its payload reaches the consume hook and stops there |

There is no third spelling and no unrecorded removal. `remove_*` keeps its
present name and meaning — it is already the honest one — but it stops being
silent.

## The policy trait

`EditPolicy` grows from two hook families to three, and loses the requirement
that a payload have a default.

```rust
/// Why a transaction created an entity.
///
/// A copy is not here: `Model::merge` transports payload verbatim and never
/// reaches this trait.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Origin {
    /// Nothing in the model before this transaction explains it.
    New,
    /// It carries on from an entity of its own kind.
    Split(EditKey),
    /// It was produced by entities of another kind, in an order the builder
    /// documents — a loft names its sections as it traverses them.
    Derived(Vec<EditKey>),
}

pub trait EditPolicy<P: Payload> {
    type Error: Error + Send + Sync + 'static;

    /// Supplies the payload for a face this transaction created.
    ///
    /// `before` is the model as the transaction found it, so a policy reads a
    /// source's payload from there for any origin that names one. A source the
    /// transaction itself created is not in `before`; such an origin is
    /// resolved to the transaction-start identities it descends from, or
    /// reported as `New` when it descends from none.
    fn face_created(
        &mut self,
        key: FaceKey,
        origin: Origin,
        before: &Model<P>,
    ) -> Result<P::F, Self::Error>;

    /// Folds a consumed face's payload into the one that survived it.
    fn face_merged(
        &mut self,
        survivor: FaceKey,
        survivor_data: &mut P::F,
        removed: FaceKey,
        removed_data: P::F,
    ) -> Result<(), Self::Error>;

    /// Disposes of a face payload nothing inherits.
    fn face_consumed(&mut self, key: FaceKey, data: P::F) -> Result<(), Self::Error>;

    // …and the same three for vertex, edge, profile, sheet and solid.
}
```

Eighteen methods where there are twelve. Three things follow, and each is worth
more than the six extra signatures.

**A creation hook returns the payload rather than mutating a default.** That is
what removes `Default` from the `Payload` trait's associated types: a builder no
longer needs a value to put in an attribute before the policy has spoken, so a
payload that *must* be assigned — a stable id, an owning feature — becomes
expressible. `Default` moves to where it belongs, as a bound on the default
policy's impl:

```rust
impl<P: Payload> EditPolicy<P> for PreservePayload
where
    P::V: Default, P::E: Default, P::Profile: Default,
    P::F: Default, P::Sheet: Default, P::S: Default,
{
    type Error = Infallible;
}
```

A payload without defaults simply has to supply its own policy, which is the
right refusal: the kernel declines to invent a value rather than approximating
one.

**`PreservePayload` spells out today's behaviour.** It clones across `Split`,
keeps the survivor on a merge, drops on a consume, and defaults on both `New`
and `Derived`. It defaults on `Derived` because it has no choice: a derived
entity's sources are of another kind, so there is no payload of the right type
to clone — a face derived from edges would need a `P::F` out of two `P::E`, and
only a policy that knows what they mean can produce one. That asymmetry is the
clearest statement of the division of labour here. `PreservePayload` is both the
default and the worked example a caller's own policy gets written from.

**The hook is where the caller's logic goes.** This is the surface the plan
exists to provide: a caller with a `Model<MyPayload>` supplies one policy object
per operation and receives a call for every identity the operation creates,
merges or consumes, with the reason attached. Nothing else in the kernel needs
to know that the caller's data exists.

## The use case: a feature timeline over the kernel

The application this is mostly for is a history-based modeler built on top of
ngk — a timeline of features, each associated with the shape it produced. It is
worth writing down because it decides two of the choices above, and because the
shape it wants is not the one a reader first expects.

**Each feature owns its own `Model`; there is no shared arena.** A rectangle, a
circle, and a loft between them are three models, and the loft merges its two
inputs before building anything. Three reasons, in order of weight:

- **A shared arena's advantage does not exist.** The reason to keep one model is
  stable keys, and keys are not stable across the operations that matter. A
  Boolean removes both operand solids outright, and a face the tool cuts comes
  back as a survivor key plus new ones — precisely the entities worth tracking
  are the ones whose keys do not survive. `edit.md` already says it: a key
  returned during an operation is stable only when it survives reconciliation.
  The application needs payload-carried identity either way, and once it has
  that, the arena buys nothing.
- **It is with the grain of the kernel's own API.** `Shape<K, P>` owns a
  `Model`, `fuse` consumes two shapes and returns one, and `into_model` hands
  the model back. The `&mut Model` style belongs to `builders`, one layer below
  what an application builds on.
- **History is the operation graph, not the arena.** What a parametric modeler
  needs is a replayable DAG: change a parameter, recompute the dirty subtree.
  One mutable model does not provide that — it would still store the operations
  and replay them from a base — so the functional structure is there regardless,
  and the only question is whether each node keeps its result.

What that costs is memory: one full `Model` per materialized node, each a GMap
and six slotmaps, in a kernel that already clones a whole model for every
transaction snapshot. That is a caching decision — materialize checkpoints,
recompute the rest — not an argument against the structure.

**The loft, concretely**, with a payload carrying a feature tag:

1. The rectangle is built in its own model under a policy that stamps
   `FeatureId(1)`. Every entity is `New`, so every entity is stamped 1.
2. The circle is built the same way, stamped 2.
3. The loft merges both models in. Their entities **arrive still stamped 1 and
   2**, because a copy is transport and reaches no hook. It then creates each
   wall face `Derived` from the two section edges it spans. The policy reads
   those edges' tags out of `before` and writes `{ feature: 3, from: [1, 2] }`.

Every entity in the result then answers *which feature made me, and what was I
made from*, with no key lookup anywhere, and the answer survives whatever
Boolean runs over it next. This is what `Derived` is for, and the wall face
spanning two section edges is why it carries several sources rather than one.

One consequence worth stating: `Model::merge` builds old-key-to-new-key maps for
vertices, edges, faces and solids, uses them to remap the embedding, and
discards them — it returns a `Dart` and nothing else. Under this design that is
correct rather than a gap. Correlating the two models by key is the approach
being rejected; the payload is the correlation, and it is the only one that
survives the operations a timeline is made of.

## What a builder owes

This is the part future builders copy, and it belongs in `src/topology/edit.md`
next to the rest of the transaction contract:

1. **Create with the most specific constructor that is true.**
   `add_face_derived_from(edge, …)` over `add_face_split_from(face, …)` over
   `add_face(…)`. `add_*` is not the cheap default — it is the assertion that
   *nothing in the model explains this entity*, and it is wrong far more often
   than it is used today.
2. **Destroy with the constructor that says who inherits.** `merge_*_into` when
   an identity carries on, `remove_*` when none does. Never leave an attribute
   to be discarded by reconciliation as a way of avoiding the choice.
3. **Never write `data` directly.** A builder constructs attributes without
   payload; the policy supplies every one. A builder that sets `data` itself has
   decided something that is the caller's to decide.
4. **One public operation, one transaction, one policy application.** Unchanged,
   and it is why a composite builder passes `&mut ModelEdit` down rather than
   opening its own transaction: the policy sees the net effect of the whole
   operation, not of each helper.

Commit enforces 1 and 2 and cannot enforce 3, so 3 becomes an API decision
rather than a review item: the `data` field on the `*Attr` types goes private,
and their constructors stop taking one.

## Stages

Each stage is independently useful and leaves the tree green.

### 1. Complete the event vocabulary and refuse what it cannot explain

Add `Origin` and the `Derived` constructors; make `remove_*` record a
consumption; make commit reject a transaction where a transaction-start
attribute is absent and no event explains it, with a named
`ModelEditError::UnexplainedRemoval { key }`.

`EditEvent` is `pub(crate)`, so its shape is free to change: thirteen variants
collapse to `Created { key, origin }`, `Merged { survivor, removed }` and
`Consumed { key }`. The typed constructors on `ModelEdit` keep the same-kind
check at the call site, where it costs nothing; the event itself is uniform over
`EditKey`, which is what lets `Derived` cross kinds at all.

**A fourth event says a copy is not a creation.** `Model::merge` today records
`EditEvent::Created` for every attribute it clones out of the source model. That
is harmless while `Created` reaches no hook, and it becomes payload loss the
moment stage 2 lands: the creation hook would be asked to supply a payload for
every copied entity and would throw the transported one away. So merge records
`Copied { key }` instead — enough for reconciliation and for the unexplained-
removal check to see the attribute, and never routed to the policy. Stage 2
depends on this having landed first, and a test asserting that a merged-in
payload survives a later policy-driven transaction is the one that holds it.

This stage is where the twenty-eight silent removal sites get read and answered
one by one. Two are already known to be wrong rather than merely undeclared:

- `builders/boolean/assemble.rs` removes both operand solids and clones `P::S`
  off the first without declaring `merge_solids_into`, so the second operand's
  solid payload vanishes with no hook called; every operand sheet is removed and
  replaced with `SheetAttr::new(root, P::Sheet::default())`, discarding sheet
  payload from both sides. This is the `sheet: 0` row of the lineage table.
- `builders/removal.rs:1177` removes two profiles outright where the face keeps
  a third, dropping both payloads unremarked.

**Done when**: no `remove_*` is silent; commit names an unexplained removal; the
boolean declares its solid merge and its sheet lineage; `merge` records a copy
rather than a creation; a test asserts that a transaction removing an attribute
without a reason is rejected, and another that a copied payload crosses a model
boundary unchanged.

### 2. Rebuild `EditPolicy` on that vocabulary

The eighteen hooks, `Origin` resolution through transaction-local ancestry, and
`Default` off the `Payload` trait and onto `PreservePayload`.

**Done when**: `Payload`'s associated types require `Clone + 'static` only;
`PreservePayload` carries a doc example; the existing
`tests/builders/face_lineage.rs` payload test is retargeted onto the new shape
and joined by one covering `Derived` and one covering `*_consumed`.

### 3. Route a policy through the public API

Today no public operation accepts one. Every builder and every `modeling` verb
gains a `*_with_policy` counterpart, and the pair is generated rather than
hand-written where the signature allows. The boolean already runs healing inside
its own transaction, so one policy covers the whole operation including the
healing pass — which is the behaviour a caller wants and is worth stating in the
doc comment.

**Done when**: `modeling::solids::fuse_with_policy` and its siblings exist; the
boolean's options carry no policy, because a policy is not a setting and belongs
in the signature; a test fuses two `Model<MyPayload>` solids and observes the
hooks fire.

### 4. Close genericity at the source

Of 37 public `modeling` functions, 9 are generic over `P` and 24 are pinned to
`StandardPayload`, and the split is not arbitrary: everything that *transforms*
an existing shape is generic, everything that *creates from nothing* is pinned.
There is therefore no entry door into a custom-payload model — a caller can
transform a `Model<MyPayload>` but cannot obtain one with anything in it.

Make the creators generic, along with the five pinned face builders
(`add_rectangle`, `add_square`, `add_circle`, `add_annulus`,
`add_polygon_with_holes`), two of which write a literal `()` into
`ProfileAttr::new`. With stage 2 done they seed through the policy rather than
through `Default`, so this is not merely a signature change.

Also in this stage: `exchange::step::read_step` is pinned, and its stated reason
— that an imported shape "has to stay compatible with `modeling::fuse`, which is
`StandardPayload`" — is false on both halves. `fuse` is generic, and the importer
needs no `Default` once creation goes through a policy. The export side is
already fully generic. An importer that is generic over `P` is also where a STEP
`PRODUCT` name or a presentation colour could finally land somewhere.

**Done when**: no `Model<StandardPayload>` remains in `src/` outside tests and
the `StandardPayload` definition itself; a test builds a block directly into a
`Model<MyPayload>`.

### 5. Ergonomics

Small, and worth doing before anything depends on the layer being pleasant.

- `Vertex::data()` and `Edge::data()`, so payload is read the same way at all six
  kinds instead of `.attr().data` at two of them.
- A payload-only write path. Setting a string on one face today requires a
  transaction, which clones the entire `Model` for the snapshot and runs the
  gmap axioms, embedding validation and cell-occupancy validation over the whole
  map. Payload is not topology: a `Model::set_face_data`-shaped API can bump the
  revision and invalidate the realization cache without revalidating anything,
  because it cannot make the map wrong.

**Done when**: the six `data()` accessors are uniform; a payload write does not
run topology validation; a test records that it does not.

### 6. Retire `BooleanLineage`

`builders/boolean/result.rs` carries a second, bespoke lineage system —
`HashMap<VertexKey, Vec<VertexKey>>` and the same for edges and faces, computed
during assembly and handed back to callers. That is the payload layer's job,
implemented once more, for one operation, in a shape no other operation can
reuse. It is also the best evidence that the need is real.

With stages 1–3 done, a caller wanting the boolean's lineage supplies a policy
that records it, and gets the same answer for every other operation for free.
Whether `BooleanResult` keeps a convenience field populated by an internal
policy, or simply stops carrying one, is the one open call here; it is a
public-API question rather than a design question.

**Done when**: `BooleanLineage` is either gone or a thin projection of recorded
policy events, and `neighborhood.rs` reads the general mechanism.

### 7. Styles as payload — the first real use case

Explicitly last, and explicitly not urgent. Everything above is worth doing on
its own; this is what proves it.

`viz::hints::VizHints` is `HashMap<VertexKey, Style>` plus the same for edges,
faces and raw dart ids — colour, opacity, label, width — that the caller keeps in
sync with the model by hand. It is the loose-pair shape the kernel avoids
elsewhere: keys change across a boolean or a healing pass, so a style silently
detaches from the entity it described, and nothing reports it. A `Style` carried
as `P::F` rides the lineage instead — a face split inherits its parent's colour,
a merged face resolves two colours through a hook the caller wrote, and the
debug viewer reads what the shape says about itself rather than consulting a
side table.

The dart styles do not move. A dart is a traversal locator with no stable
identity and no attribute to hang anything on, so `VizHints` survives for the
overlay and loses its three entity maps.

**Done when**: `viz::scene_from_model` reads entity styles from the payload when
`P` supplies them; `VizHints` keeps only `dart_styles`; the debug viewer shows a
boolean result whose operand colours survived the operation, which is the thing
that cannot be demonstrated today.

## Non-goals

- **A dynamic payload for the bindings.** Python and wasm pin every type to
  `StandardPayload` and a Rust type parameter has no spelling in either. Serving
  those surfaces needs one `P` whose associated types are an attribute map, not
  more genericity, and it is a separate design. Nothing in this plan forecloses
  it; stages 1–3 are its prerequisite.
- **Topological naming.** Persistent selection across a rebuild is what a payload
  makes *possible*, not what this plan delivers. The kernel is not a
  history-based modeler; it supplies the identity an application carries, and
  the naming scheme belongs to that application. "The use case" above is the
  shape of the application, not a component of the kernel.
- **Cross-model lineage.** `Model::merge` transports payload values across models
  and records the copies. Keys are per-model, so an event cannot name a source in
  another model, and no stage here tries to make one — the payload is what
  crosses, which is the point of the transport rule rather than a limitation
  around it.

## Open questions

- **Does `Payload` need `Clone + 'static` on the marker type itself?** It is
  never instantiated — `StandardPayload` is a unit struct and exists only to name
  six associated types. The real bounds are all on the associated types, and the
  bound on `P` looks like a leftover.
- **Is a `Derived` source order a builder can document good enough?** The variant
  carries several sources and the order is the builder's contract, which is the
  lightest thing that works. A policy that wants to know *which* section an edge
  came from reads position, and a builder that reorders its traversal silently
  changes meaning. Whether that needs a role tag per source rather than a
  position is the one part of `Derived` left open, and the loft is the case to
  decide it against.
- **Where does the policy live for a composite public operation that calls
  another public operation?** Today none do — composites call staged helpers —
  but `heal` inside `boolean` is close to the line, and the rule that keeps it
  clean is worth writing down before something crosses it.
