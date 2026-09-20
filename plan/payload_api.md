# Payload API — decisions and what was rejected

Companion to `plan/payload.md`, which is the plan. This file records *why* the
API has the shape it does, so the reasoning is not re-litigated.

## The problem

Three questions were welded into one answer:

| Question | Answered by | Could a caller change it? |
|---|---|---|
| Who opens the transaction? | every builder, on its own | no |
| Which policy runs? | `Model::transaction`, hardcoded `PreservePayload` | no |
| Which payload is in the model? | whichever the entry point was pinned to | only 13 of 37 `modeling` verbs |

Separating them is the whole design.

## Decision 1 — the payload names its own policy

`Payload::Policy: EditPolicy<Self> + Default`, and `Model::transaction` runs
`P::Policy::default()`.

Maintenance (*how does a colour survive a split*) is a property of the data and
belongs on the type. Stamping and recording (*this loft is feature 3*) is a
property of the call and belongs in the signature. Both were in the signature,
which is why neither was anywhere.

Consequences: `PreservePayload` becomes unreachable by accident; `DefaultPayload`
is deleted and its ~30 bounds become `P: Payload`; nothing is stored on `Model`,
so `Clone`, the transaction snapshot and the serde bounds are untouched; and no
builder error type changes, because a policy error is already erased into
`ModelEditError::Policy`.

**Verified by spike, not assumed.** The recursive bound — `Payload::Policy:
EditPolicy<Self>` where `EditPolicy<P>` requires `P: Payload` — could have
overflowed the trait solver. It compiles under edition 2024: one `P: Payload`
bound drives both `StandardPayload`'s `PreservePayload` and a custom payload
whose `P::F` has no `Default` at all.

## Decision 2 — the entry door is one conversion, not twenty-four twins

**Rejected: `*_with_policy` on all 24 pinned creators.** A policy running inside
a primitive creator can observe *nothing* — every entity there is `Origin::New`,
so the hook receives a key and an empty `before` and cannot tell one face from
another. It is exactly as expressive as stamping the same value afterwards.

**Rejected: make the creators generic.** Rust has no default type parameter on a
function, so `pub fn block<P: Payload>(…)` makes `let b = block(1.0, 1.0, 1.0)?`
ambiguous, and `cut(block()?, cylinder()?)` ambiguous three times over. A
spelling that mentions no payload must pin one, and `StandardPayload` is the one
to pin.

**Chosen: `map_payload`.** One function, `Model<P> → Model<Q>`, no topology
touched, keys preserved, no policy run. It also earns its place independently —
stripping payload before export, attaching viz styles to a model built without
them.

**Also free, and worth documenting:** under decision 1 the builders are
`P: Payload`, so `Model::<Colored>::new()` plus `builders::*` already works with
no new API. That is the full-control path; `modeling` + `map_payload` is the
convenient one.

## Decision 3 — `builders::*_staged` stays `pub(crate)`

An earlier revision proposed making them public so a caller could compose
several builder calls under one policy. The tree says otherwise: the 5 staged
helpers that are `pub` today are called **only from `tests/`** — no example,
script, binding or downstream consumer touches one.

Both per-call policy cases are *modeling*-layer verbs — `loft`, `fuse`,
`chamfer` — so the override belongs there, on about ten functions, and the
builders layer needs no publicity change. Inventing a public surface for a
demand that does not exist is how the current sprawl started.

The 5-`pub`/10-`pub(crate)` split is arbitrary and the `_staged` suffix
duplicates what a module path would say. That is an internal tidy-up, not part
of this decision.

## Decision 4 — a dynamic dict payload is additive, not an alternative

Making the payload a `HashMap<String, Value>` at every dimension would remove
the type parameter, and with it the inference problem, the entry door and the
pinned/generic split. That half is real. The other half is not:

- The policy problem is untouched — something still decides, per key, whether a
  split clones, drops or recomputes.
- It gets *harder*: one dict means one policy object for every concern at once,
  so colour and feature-id have to share it. The way out is a registry of
  per-key rules consulted for every entity of every commit — more machinery, and
  a runtime cost on every edit.
- It gives up the refusal. `type F = Color` with no `Default` makes "you did not
  say what colour" a compile error; a dict makes it a missing key that reads as
  absent later, which is the failure this whole plan exists to prevent.

Under decision 1 it is one `impl Payload for DynPayload { type Policy =
RuleRegistry; }`. It slots on top; building it first would leave the typed users
worse off for nothing.

## Decision 5 — stage order

The original plan put the builder lineage declarations inside stage 1 and left
the caller-facing API for stages 3–4. That is backwards for the use cases: a
policy can only be as smart as the lineage a builder declares, and there is
exactly **one** `add_*_derived_from` call in the tree. Colour on loft walls is
blocked on the declaration, not on the API.

So the API lands first because it is mechanical and unblocks everything
(stages A–B), and the declarations follow immediately (stage C) rather than
last.
