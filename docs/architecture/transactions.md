# Model operation transactions

`Model::transaction` is the atomic boundary for a complete modeling operation.
Its closure receives a `ModelEdit`, which is the public mutation capability for
the staged model. Returning an error, failing validation, failing identity
reconciliation, or failing payload policy restores the complete
transaction-start snapshot — the map, the entity stores, the embedding
labelling and the revision counter alike.

`Model::transaction` runs `P::Policy`, the maintenance policy the payload named
for itself, default-constructed for this transaction. No call site chooses it:
how a colour survives a split is a property of the data, so it is named once on
the payload type and every builder here picks it up. `StandardPayload` names
`PreservePayload`, which is why a payload naming that one needs `Default` at
every dimension — an obligation discharged at its `impl Payload`, not carried by
any signature in this module.

`Model::transaction_with_policy` uses the same boundary with a caller-provided
`EditPolicy`, overriding `P::Policy` for one transaction. That is what a policy
carrying parameters *in* (this loft is feature 3) or state *out* (record what
this boolean did) requires, since `P::Policy` is default-constructed and dropped
at commit and can do neither. Policy event application happens only after the
complete staged operation passes topology validation and identity
reconciliation.

Operation code uses `Result` for recoverable failure. Transactions do not catch
panics, including a panic after a recorded read fault or inside a payload
policy. Panics propagate without recovery; rollback and model reuse are not
guaranteed after a panic. A recoverable condition must have an explicit error
path, and an internal invariant failure requires a correction to the design.

## Builder composition

Each public builder accepts `&mut Model` and starts one transaction. Its
`_edit`-suffixed operation accepts `&mut ModelEdit` and performs the actual
work. A composite builder calls other `_edit` operations with the same edit
capability, so the whole modeling operation has one snapshot, one journal, and
one commit.

The public wrapper is a one-line transaction boundary; the `_edit` operation
owns validation and construction and is `pub(crate)`. The suffix names the
parameter that makes it composable (an already-open `&mut ModelEdit`). It
replaced a leading underscore, which also hid the item from `dead_code`: an
`_edit` operation orphaned by the deletion of its public wrapper is now reported
like any other unused item.

Operation results are stamped with `transaction_result` only after commit.
Their view methods reject an unstamped or stale result.

Raw map mutation is model-internal. `Model::topology()` hands out `&GMap` and
nothing hands out `&mut GMap`, so builders can inspect the map through the
immutable access exposed by `ModelEdit` but cannot bypass the transaction when
adding darts, changing alpha links, labelling cells, or mutating attributes.

`ModelEdit` owns no snapshot and has no independent commit. It provides edit
operators, attribute creation, removal and mutation, lineage declarations, and
the deferred block described below. Raw topology primitives are restricted to
the topology implementation. Builders and external callers access them only
through `RawEdit` inside a deferred block, which exposes no model or typed view.

Profile and sheet registration follows the same explicit model as edge and
vertex registration. Adding a face does not synthesize profiles for its
boundary loops, and adding a solid does not synthesize sheets for its shells.
Builders register fresh components with `add_profile` / `add_sheet`, and use
the corresponding `*_split_from` or `merge_*_into` operation when identity is
derived from existing topology. Commit rejects faces or solids whose referenced
components have not been registered.

## Reads on an open edit

A view (`Face::loops`, `Sheet::faces`) reads the model through the embedding
index, the dart-to-owner lookup derived from the labelling. The labelling is
only checked at commit, so on an open edit it can describe nothing the index
can be built for. Three layers keep that from becoming a panic:

1. **Operators and deferred blocks** keep it from happening in the first
   place (next two sections).
2. **`Model::try_embedding_index`** returns the build's `Result`. A failure is
   never cached.
3. **A read that cannot return a `Result`** (`Model::embedding_index`, the
   boundary walk behind `Face::loops`) records the first such fault in the
   model, answers with what an empty labelling would (no owner, no loops) and
   goes on. The transaction looks for a recorded fault when the operation
   returns, whatever the operation returned, and when the commit checks have
   run: it rolls back and returns the fault as `InvalidEmbedding` or
   `UnreadableBoundary`. If the operation panics before returning, the panic
   propagates; the recorded fault does not turn it into an error.

The fault names the first read that failed, which is later than the step that
caused it. Calling `ModelEdit::check_consistent` between two steps narrows that.

## Edit operators

An operator (`src/topology/edit_operators.rs`) is a change to the map that
leaves the model valid: the map, the labelling and the entity attributes agree
when it returns, so a view read between two operators answers. It reads what it
needs before it writes, refuses a precondition that does not hold with the
model untouched (`OperatorError`), does the rewiring and the relabelling in the
one call. Operators preserve their invariants by construction; their tests
validate the map and embedding between calls. They do not run a complete model
validation on every return. Refusable inputs use explicit errors; fresh darts
and already-established internal invariants are not recoverable failure modes.

| Operator | What it does |
|---|---|
| `mark_edge` | gives an unmarked edge a vertex on the 0-cell where it closes, and drops the edge's scaffold label there |
| `cut_edge` | cuts an edge that has a corner in two: doubles every dart of its 1-cell, gives the new 0-cell a vertex and the second half a new key |
| `sew_edges` | sews two edges where 2-cells meet, fuses the edges and the vertices at their ends, and settles who labels a fused closure: a vertex on it means no scaffold, so an unmarked edge sewn to a marked one is marked |
| `unsew_edge` | the inverse for an edge shared by two sides: cuts the `alpha2` links, splits the vertex of each end that no longer holds together, and gives the other side a new edge key |

Geometry stays with the caller: `cut_edge` takes the two curves and the point,
`mark_edge` the point. The operators are one-step units, not the transaction's
unit; a builder composes them with deferred raw surgery where a specialized
rewiring cannot be expressed by the existing operators.

`RawEdit::sew`, `own_cell`, `link`, `unlink` and dart allocation are raw
primitives: the caller settles the labelling before closing its block.
`check_consistent` is an optional embedding diagnostic, not a complete topology,
geometry or solid validation.

## Deferred surgery

`edit.deferred(|raw| ..)` runs a rewiring that passes through invalid states.
`RawEdit` has the primitives -- linking and sewing darts, labelling cells,
registering, merging and removing entities, compacting isolated darts -- and the
stored attributes, but no view, which the type enforces: it does not dereference to `Model`, so nothing in the block can
walk the map through a labelling that is allowed to be wrong. When the block
returns, the map's involution axioms and the labelling are checked, as a
database checks its deferred constraints at the end of the statement; a block
that leaves either broken returns an error and its transaction rolls back.

The block reads the map (`raw.topology()`) and nothing derived from the
labelling, so a builder reads what it needs first and then opens the block. A
helper that runs inside other builders' blocks takes the `RawEdit`
(`splice_cut_between_loops`), and its `ModelEdit` form opens a block of its own
(`cut_between_loops`). The close costs one pass over the map and one rebuild of
the embedding index, so a builder opens one block per rewiring, not one per
primitive.

## Post-conditions at commit

After reconciliation and the structural checks, commit validates every solid
in the model when `CommitChecks::solid_orientation` is enabled. The check is
`validate_solid_orientation`: each shell is closed, the windings agree across
every edge, and the signed volume has the right sign (positive for an outer
shell, negative for a cavity). Failure returns `InvalidSolid`.

Validation does not select solids from creation or merge events: changing a
link, a winding or geometry in place need not declare such an event. Checking
all registered solids also covers an existing solid opened by `unsew_edge`.
Profiles and open sheets remain valid entities outside solids.

The check costs a pass over all solids' faces. `CommitChecks` travels with model
snapshots; the default enables this check with `debug_assertions` and disables
it in release builds, including release tests.

Splitting a boundaryless face replaces its polygon scaffold. Its sheet and solid
anchors follow the replacement with their orientation preserved, and detached
old scaffold is removed. Demoting the last seam of a revolved face retains the
meridian span in an exact finite support so that the boundaryless face covers
the same geometry rather than the untrimmed profile swept twice.

## Lineage and policy

Every creation names an `Origin`: plain `add_*` records `Origin::New`,
`add_*_split_from` records `Origin::Split` from a source of the created
entity's own kind, and `add_*_derived_from` records `Origin::Derived` from one
or more sources of any kind. `merge_*_into` explicitly names the surviving and
consumed identities. `remove_*` records that the removed identity's payload is
inherited by nothing. Both `remove_*` and `merge_*_into` record their own
event, so a transaction-start attribute cannot go missing at commit without
one explaining it -- see "Commit order" below. A copy made by `Model::merge`
is recorded so reconciliation can see it, but it is neither a creation nor a
removal and never reaches `EditPolicy`.

`EditPolicy` has three hook families per kind (vertex, edge, profile, face,
sheet, solid): a `*_created` hook that returns the new payload, a `*_merged`
hook that folds a consumed payload into the survivor, and a `*_consumed` hook
that disposes of a payload nothing inherits. At commit, merge chains are
resolved to their final survivor, an origin's sources are resolved to
transaction-start identities (or reported as `Origin::New` when none survive
that far back), and policy is applied only to net changes visible outside the
operation:

- every surviving creation, with its origin resolved against the
  transaction-start snapshot;
- a surviving merge's consumed identity;
- an explicitly consumed transaction-start identity;
- never a fresh, split, derived or consumed identity that was created and
  discarded inside the operation.

Policy callbacks run in declaration order. A `*_created` hook receives the
model as the transaction found it, so it reads a named source's payload from
there; a `*_merged` or `*_consumed` hook receives the removed identity's
payload from that same snapshot. `PreservePayload` clones a `Split`'s source
payload, keeps the merge survivor, drops on a consume, and defaults on both
`New` and `Derived` — a derived entity's sources need not even share its kind,
so there is no single payload of the right type to clone, and only a caller's
own policy knows how to produce one. A policy error restores topology and
payloads.

**A builder is only as informative as its declarations.** A hook handed
`Origin::New` is told that nothing explains the entity, and can do nothing but
invent a value; one handed `Origin::Derived { sources: vec![edge] }` can read that edge out of
`before` and carry its data across. So reach for the most specific constructor
that is true — `add_*_derived_from` over `add_*_split_from` over `add_*` — and
document the order the sources are named in, because a policy that cares reads
position.

## Identity reconciliation

After structural validation, attributes that now describe the same final cell
are grouped for each attribute kind. One transaction-start key beats local
keys. If all keys are local, the earliest-created key wins. Multiple
transaction-start keys require explicit lineage naming a survivor. Losing
local attributes are discarded without invoking merge policy.

Keys returned during an operation are stable only when they survive this
reconciliation. A temporary local key may be removed at commit; staged typed
lookups resolve the operation's logical survivor.

## Commit order

1. the raw gmap axioms, on `Model::topology()` alone;
2. the required profile/sheet registrations;
3. edit-event lineage, then identity reconciliation;
4. every transaction-start attribute is either still present, named by a
   `Merged` or `Consumed` event, or dropped by reconciliation's own structural
   bookkeeping (a profile that lost its last edge) -- otherwise commit rejects
   the transaction with `UnexplainedRemoval`;
5. the embedding labels, which must describe the reconciled map: no record
   anchored off it, no owner of lower dimension than the cell it claims, and
   no cell two entities disagree about;
6. one same-dimensional raw cell per logical vertex, edge, face and solid;
7. closed, outward shells for every registered solid, when
   `CommitChecks::solid_orientation` is on;
8. payload policy on net externally-visible changes;
9. a read fault recorded while committing counts as a failed commit;
10. `revision += 1` and cache invalidation.

A failure at any step restores the transaction-start snapshot whole.

## Derived indexes

The six dart-to-key maps are one lazy `DerivedCellIndexes` cache and the
dart-to-owner lookup is a second one. Any mutation invalidates both; the first
typed lookup or traversal rebuilds what it needs, subsequent reads reuse it, and
commit materializes the cell indexes after reconciliation. The owner lookup is
the only one that can fail to build (see "Reads on an open edit"). Neither is
serialized: a deserialized model rebuilds them from its authoritative stores.
Builders use typed APIs such as `cell_key`, `attribute`, `ownership` and
topology views rather than accessing indexes directly.
