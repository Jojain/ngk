# Payload: the user data a shape carries

Status: **stages 1–2 and A done; B onward proposed.** The vocabulary
(`Origin`, `Copied`, `UnexplainedRemoval`), the policy trait, and
`Payload::Policy` are in the tree. What is still missing is the entry door and
the builder lineage declarations.

`plan/payload_api.md` records why the API below has this shape and what was
rejected. This file is the plan: what to build, in what order.

## The idea

A shape carries user data — a colour, a feature tag, a material — and every
operation has to say what happens to that data. A face is cut in two: do both
halves keep the colour? Two faces fuse: which colour wins? A loft builds a wall
between two sections: what colour is the wall?

The kernel cannot answer those. The caller can. So the kernel's job is to
**ask**, every time, with the reason attached — and to refuse to proceed if a
builder will not say where an entity came from or where it went.

## What already works

Every creation names an **origin**, and there are three:

| `Origin` | Means | Example |
|---|---|---|
| `New` | nothing in the model explains it | a block's first corner |
| `Split(source)` | it carries on from `source`, same kind | one of the two edges a cut produces |
| `Derived(sources)` | it was produced by `sources`, of any kind | a loft's wall face, from the two section edges it spans |

Every destruction names one of two:

| Spelling | Means |
|---|---|
| `merge_*_into(survivor, removed)` | `removed`'s identity carries on inside `survivor` |
| `remove_*(key)` | nothing inherits it |

Commit rejects a transaction where a pre-existing attribute vanished and no
event explains it (`ModelEditError::UnexplainedRemoval`). A **copy** made by
`Model::merge` is neither: it transports the payload verbatim and never reaches
the policy at all. That rule is load-bearing — it is how data survives crossing
from one model into another.

`EditPolicy` has three hooks per kind: `*_created(key, origin, before)` returns
the new payload, `*_merged` folds a consumed payload into the survivor,
`*_consumed` disposes of one nothing inherits. `before` is the whole model as
the transaction found it, so a hook can read a source's payload *and* traverse
from it.

**What is still missing.** Two things. A builder can only be as informative as
its declarations, and there is exactly **one** `add_*_derived_from` call in the
tree — so a loft wall or an extrusion wall still reaches the policy as
`Origin::New`, carrying no information at all (stage C). And of 37 public
`modeling` functions, 24 are pinned to `StandardPayload`, so there is no
convenient way to get a model with custom data in the first place (stage B).

## The API, in two halves

Two different questions were being answered by one mechanism, which is why
neither had a good answer.

| | Question | Where it lives | Stateful? |
|---|---|---|---|
| **Maintenance** | how does *this kind of data* survive an edit? | on the payload type, once | no |
| **Stamping / recording** | what should *this particular call* write down? | in the operation's signature | yes |

### Half 1 — the payload names its own policy

```rust
pub trait Payload: Clone + 'static {
    type V; type E; type Profile; type F; type Sheet; type S;

    /// How this data maintains itself across an edit that names no policy.
    type Policy: EditPolicy<Self> + Default;
}
```

`Model::transaction` runs `P::Policy::default()`. So once you have written your
policy, **every operation in the kernel uses it and no call site says anything**.

`StandardPayload::Policy = PreservePayload`, so today's behaviour is unchanged.
`DefaultPayload` is gone: the `Default` obligation is discharged once, where
`PreservePayload` is named, instead of riding on 80-odd signatures.

### Half 2 — a call can override it

`Model::transaction_with_policy(&mut my_policy, …)` already exists. It serves
the cases half 1 cannot: a policy carrying parameters in (*this loft is feature
3*) or state out (*record everything this boolean did*). The transforming
`modeling` verbs get a `*_with_policy` twin for it.

## Worked example A — per-face colour

The maintenance case. Nothing is passed at any call site.

```rust
#[derive(Clone)]
struct Colored;

impl Payload for Colored {
    type F = Color;       // faces are coloured
    type E = Color;       // so are edges — a loft wall reads these
    type V = (); type Profile = (); type Sheet = (); type S = ();

    type Policy = ColorPolicy;
}

struct ColorPolicy;

impl EditPolicy<Colored> for ColorPolicy {
    type Error = Infallible;

    fn face_created(&mut self, _key: FaceKey, origin: Origin, before: &Model<Colored>)
        -> Result<Color, Infallible>
    {
        Ok(match origin {
            // A face cut in two: both halves keep the original colour.
            Origin::Split(EditKey::Face(src)) => *before.face_attr_unchecked(src).data(),

            // A loft wall, from the two section edges it spans. Blend them.
            Origin::Derived(sources) => Color::blend(sources.iter().filter_map(|k| match k {
                EditKey::Edge(e) => Some(*before.edge_attr_unchecked(*e).data()),
                _ => None,
            })),

            _ => Color::UNSET,
        })
    }

    // Two coloured faces fuse: the larger contributor wins, rather than
    // whichever key happened to survive reconciliation.
    fn face_merged(&mut self, _s: FaceKey, survivor: &mut Color, _r: FaceKey, removed: Color)
        -> Result<(), Infallible>
    { if removed.area > survivor.area { *survivor = removed; } Ok(()) }
}
```

Using it:

```rust
// Build with the plain API. Nothing about payload here.
let rect   = modeling::faces::rectangle(bottom, 10.0, 10.0)?;
let circle = modeling::faces::circle(top, 4.0)?;

// Enter the colour world. One line each.
let rect   = rect.map_payload(&mut Uniform(RED))?;
let circle = circle.map_payload(&mut Uniform(BLUE))?;

// Loft. ColorPolicy runs because `Colored` named it. Nothing said.
let solid = modeling::loft::loft(&[&rect, &circle], LoftOptions::default())?;

// And it keeps running, for free, through everything after.
let bored = modeling::solids::cut(solid, drill)?;
```

What comes out, and why:

| Entity | How the builder makes it | Colour |
|---|---|---|
| bottom cap (the rect) | copied into the loft's model | **RED** — a copy carries data verbatim |
| top cap (the circle) | copied in | **BLUE** |
| wall faces | `add_face_derived_from(&[bottom_edge, top_edge], …)` | **purple** — the hook blends the two |
| vertical edges | `add_edge_derived_from(&[…])` | blended |

The wall row is the one that does not work yet. `builders/loft.rs:839` creates
its walls with plain `add_face`, which declares `Origin::New` — *nothing
explains this face* — so the hook is called with nothing to read. Stage C fixes
that, and it is the stage this use case is actually blocked on.

## Worked example B — feature tags over a timeline

The stamping-and-recording case, and the reason half 2 exists. A history-based
modeler wants every entity to answer *which feature made me, and what was I made
from*, and wants the timeline to learn what each operation destroyed.

```rust
#[derive(Clone, Debug)]
struct Provenance { feature: FeatureId, from: Vec<FeatureId> }

#[derive(Clone)]
struct Tagged;

impl Payload for Tagged {
    type F = Provenance;
    type E = Provenance;
    type V = (); type Profile = (); type Sheet = (); type S = ();

    type Policy = KeepProvenance;
}
```

`KeepProvenance` is the maintenance half: a split keeps its parent's tag. But it
**cannot** answer `New` — there is no feature in scope, and `Provenance` has no
meaningful default. So it refuses:

```rust
impl EditPolicy<Tagged> for KeepProvenance {
    type Error = NoFeatureInScope;

    fn face_created(&mut self, key: FaceKey, origin: Origin, before: &Model<Tagged>)
        -> Result<Provenance, NoFeatureInScope>
    {
        match origin {
            Origin::Split(EditKey::Face(src)) => Ok(before.face_attr_unchecked(src).data().clone()),
            // Refuse rather than approximate: a new entity with no feature
            // named is a caller error, not a value to invent.
            _ => Err(NoFeatureInScope { key: key.into() }),
        }
    }
}
```

So any operation that *creates* has to name its feature, through half 2:

```rust
struct StampFeature {
    id: FeatureId,
    /// What this operation destroyed — the timeline reads it back out.
    retired: Vec<FeatureId>,
}

impl EditPolicy<Tagged> for StampFeature {
    type Error = Infallible;

    fn face_created(&mut self, _key: FaceKey, origin: Origin, before: &Model<Tagged>)
        -> Result<Provenance, Infallible>
    {
        Ok(Provenance {
            feature: self.id,
            from: sources_of(origin, before),   // the tags this was built out of
        })
    }

    // The hook example A never needed: record what died.
    fn face_consumed(&mut self, _key: FaceKey, data: Provenance) -> Result<(), Infallible> {
        self.retired.push(data.feature);
        Ok(())
    }
}
```

and the call site names it once:

```rust
let mut stamp = StampFeature { id: FeatureId(3), retired: Vec::new() };
let solid = modeling::loft::loft_with_policy(&mut stamp, &[&rect, &circle], opts)?;

timeline.record(FeatureId(3), stamp.retired);   // state came back out
```

Three things this example pins down that example A does not:

- a payload with **no `Default` at any dimension** — impossible before
  `Payload::Policy`, because `Model::transaction` demanded one;
- a default policy that **refuses** rather than inventing, which is the kernel's
  house rule applied to user data;
- a policy that **carries state out**, which is why `*_with_policy` has to exist
  on the transforming verbs and is the argument for generating all of them.

## The entry door

`map_payload` converts a model's payloads to another set of types. It touches no
topology, so every key stays valid and no policy runs — it is a conversion, not
an edit.

```rust
impl<P: Payload> Model<P> {
    pub fn map_payload<Q: Payload, S: PayloadSeed<P, Q>>(self, seed: &mut S)
        -> Result<Model<Q>, S::Error>;
}
```

This is why the `modeling` creators stay pinned to `StandardPayload` and gain no
twins: a policy running inside `block()` could observe nothing anyway — every
entity there is `Origin::New`, so the hook gets a key and an empty `before` and
cannot tell one face from another. Stamping the same value afterwards is exactly
as expressive, and it is one function instead of twenty-four.

**Primitives are conveniences; decompose when you need lineage.** A caller who
wants the extrusion's walls to differ from its caps writes
`rectangle → map_payload → extrude_face`, where the extrusion is a transform and
runs the policy properly.

## Stages

Reordered from the original plan. The rule is that each stage leaves the tree
green and is useful on its own, and that the stage the use cases are actually
blocked on (C) is not last.

### A. `Payload::Policy` — **done**

`Payload` gained `type Policy: EditPolicy<Self> + Default`;
`StandardPayload::Policy = PreservePayload`; `Model::transaction` runs
`P::Policy::default()`; `DefaultPayload` is deleted and its 80-odd bounds across
`src/` and `bindings/` are plain `P: Payload`.

`tests/topology/payload_policy.rs` holds the two tests that pin it: an ordinary
builder running a custom payload's own policy with nothing said at the call
site, and a payload with **no `Default` at any dimension** reaching a builder at
all — which was impossible before, since every builder demanded one.

The five payloads in `tests/` each named `type Policy = PreservePayload;`, and
the compiler found every one of them. That is the property this stage was for:
a payload cannot exist without having answered how it is maintained.

### B. `map_payload`, and the door it opens

`Model::map_payload` and `Shape::map_payload`, the `PayloadSeed` trait, and a
`Uniform` helper. Then the five pinned face builders (`add_rectangle`,
`add_square`, `add_circle`, `add_annulus`, `add_polygon_with_holes`) become
generic, since with stage A they need no `Default`.

Also here: `exchange::step::read_step` is pinned, and its stated reason — that an
imported shape must stay compatible with `modeling::fuse` — is false on both
halves. `fuse` is generic and the importer needs no `Default` once stage A
lands. A generic importer is where a STEP `PRODUCT` name or a presentation
colour could finally land.

**Done when**: example A's first four lines compile and run; no
`Model<StandardPayload>` remains in `src/` outside the `StandardPayload`
definition and tests.

### C. Builders declare where things came from

This is what the use cases are blocked on. Today there is exactly **one**
`add_*_derived_from` call in the tree, in `boolean/assemble.rs`, and eleven
`*_split_from`.

- **extrusion** (`builders/solids.rs:381`): each wall face is `Derived` from the
  boundary edge it was raised on; each vertical edge is `Derived` from the vertex
  it was raised on. The two caps need nothing — the bottom is the original face
  untouched and the top is a copy, so both already carry their data.
- **loft** (`builders/loft.rs:839`, `:915`): each wall face is `Derived` from the
  two section edges it spans, named in section order. Each rung edge likewise.
- **sweep** and **revolve**: the same shape as extrusion.
- **chamfer**: the new face is `Derived` from the edge it replaced.

Each one needs its source order documented on the builder, because a policy that
cares reads position.

**Done when**: example A's wall faces come out purple; a test asserts the origin
each of these builders declares, not just the payload that results.

### D. One public operation, one transaction

`edit.md` says one operation is one transaction and one policy application.
Eight public functions break it, and `revision()` is an exact transaction count
(it increments once per successful commit, from `0`):

| Function | Transactions | Policy applied twice? |
|---|---|---|
| `block`, `block_at`, `cylinder`, `cylinder_at` | 2 — build the base face, then extrude | **yes** |
| `fuse`, `cut`, `intersect` | 2 — copy the tool in, then the boolean | no; a copy fires no hook |
| `loft` | 2 — copy the sections in, then skin | no |

No correctness bug today: each works on a fresh local `Model` it owns, so a
failure in the second transaction drops the whole thing. The cost is a full
model clone and a full validation pass per extra transaction, plus a double
policy application on the four primitives.

The fix is to write them the way `faces::polygon` already is — one `transaction`
block calling staged helpers. `fuse`/`cut`/`intersect` need `boolean_staged`,
because `BooleanContext::admit` currently runs *before* the transaction and reads
both solids out of the model.

**Done when**: one test asserts `revision() == 1` after every public `modeling`
operation, and `edit.md` states the rule as *a public modeling verb contains at
most one `transaction` call and calls only staged helpers inside it*.

### E. `*_with_policy` on the transforming verbs

`fuse`, `cut`, `intersect`, `loft`, `revolve_*`, `extrude_*`, `chamfer`, `heal` —
about ten, generated rather than hand-written. The boolean runs healing inside
its own transaction, so one policy covers the whole operation including the
healing pass; say so in the doc comment.

The creators get none: `map_payload` is their entry door, per stage B.

**Done when**: example B runs end to end and `stamp.retired` is non-empty.

### F. Ergonomics

- `Vertex::data()` and `Edge::data()`, so payload reads the same way at all six
  kinds instead of `.attr().data` at two of them.
- A payload-only write path. Setting one face's colour today needs a transaction,
  which clones the whole `Model` and re-runs the gmap axioms, embedding
  validation and cell-occupancy validation. Payload is not topology: a
  `Model::set_face_data` can bump the revision and invalidate the realization
  cache without revalidating anything, because it cannot make the map wrong.

**Done when**: the six `data()` accessors are uniform; a test records that a
payload write runs no topology validation.

### G. Retire `BooleanLineage`

`builders/boolean/result.rs` carries a second, bespoke lineage system —
`HashMap<VertexKey, Vec<VertexKey>>` and the same for edges and faces — computed
during assembly and handed to callers. That is this layer's job, implemented
once more, for one operation, in a shape nothing else can reuse. It is also the
best evidence the need is real.

With A–E done, a caller wanting it supplies a recording policy and gets the same
answer for every other operation for free.

**Done when**: `BooleanLineage` is gone or a thin projection of recorded policy
events, and `neighborhood.rs` reads the general mechanism.

### H. Styles as payload

`viz::hints::VizHints` is `HashMap<VertexKey, Style>` plus the same for edges and
faces, kept in sync with the model by hand. Keys change across a boolean or a
healing pass, so a style silently detaches from what it described and nothing
reports it. A `Style` carried as `P::F` rides the lineage instead.

Dart styles do not move: a dart is a traversal locator with no stable identity
and nothing to hang an attribute on, so `VizHints` survives for the overlay and
loses its three entity maps.

**Done when**: `viz::scene_from_model` reads entity styles from the payload when
`P` supplies them; the debug viewer shows a boolean result whose operand colours
survived, which cannot be demonstrated today.

## Non-goals

- **A dynamic dict payload for the bindings.** Python and wasm pin every type to
  `StandardPayload` and a Rust type parameter has no spelling in either. That
  needs one `P` whose associated types are an attribute map — and with stage A it
  is a single `impl Payload for DynPayload { type Policy = RuleRegistry; }`
  rather than a competing design. Additive, and A–B are its prerequisite.
- **Topological naming.** Persistent selection across a rebuild is what a payload
  makes *possible*, not what this delivers. The kernel supplies the identity an
  application carries; the naming scheme belongs to that application. Example B
  is the shape of the application, not a component of the kernel.
- **Cross-model lineage.** Keys are per-model, so an event cannot name a source
  in another model. The payload is what crosses — that is the point of the
  transport rule, not a limitation around it.

## Open questions

- **`PayloadSeed`'s shape.** Six methods on a trait, one closure per kind, or
  reuse `EditPolicy::*_created` with `Origin::New`? A `Uniform` helper covers the
  common case either way.
- **Is a documented source order good enough for `Derived`?** The variant carries
  several sources and the order is the builder's contract. A policy that wants to
  know *which* section an edge came from reads position, and a builder that
  reorders its traversal silently changes meaning. Whether that needs a role tag
  per source is the one part left open, and the loft is the case to decide it
  against.
- **Where does the policy live for a composite public operation that calls
  another public operation?** None do today — composites call staged helpers —
  but `heal` inside `boolean` is close to the line.
