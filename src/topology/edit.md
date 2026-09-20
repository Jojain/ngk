# Model operation transactions

`Model::transaction` is the atomic boundary for a complete modeling operation.
Its closure receives a `ModelEdit`, which is the public mutation capability for
the staged model. Returning an error, failing validation, failing identity
reconciliation, or failing payload policy restores the complete
transaction-start snapshot — the map, the entity stores, the embedding
labelling and the revision counter alike. `Model::transaction` runs the
default policy, `PreservePayload`, which requires every payload dimension to
have a `Default`; a payload without one for some dimension cannot use this
entry point at all and must supply its own policy instead.

`Model::transaction_with_policy` uses the same boundary with a caller-provided
`EditPolicy`, which is what a payload without a `Default` at every dimension
requires. Policy event application happens only after the complete staged
operation passes topology validation and identity reconciliation.

Transactions intentionally do not catch panics. Operation code uses `Result`
for recoverable failure.

## Builder composition

Each public builder accepts `&mut Model` and starts one transaction. Its private
staged helper accepts `&mut ModelEdit` and performs the actual work. A composite
builder calls other staged helpers with the same edit capability, so the whole
modeling operation has one snapshot, one journal, and one commit.

Raw map mutation is model-internal. `Model::topology()` hands out `&GMap` and
nothing hands out `&mut GMap`, so builders can inspect the map through the
immutable access exposed by `ModelEdit` but cannot bypass the transaction when
adding darts, changing alpha links, labelling cells, or mutating attributes.

`ModelEdit` owns no snapshot and has no independent commit. It provides the
checked alpha operations `add_dart`, `remove_dart`, `link`, `unlink`, and `sew`,
the embedding label `own_cell`, plus attribute creation, removal, mutation,
split, and merge declarations.

Profile and sheet registration follows the same explicit model as edge and
vertex registration. Adding a face does not synthesize profiles for its
boundary loops, and adding a solid does not synthesize sheets for its shells.
Builders register fresh components with `add_profile` / `add_sheet`, and use
the corresponding `*_split_from` or `merge_*_into` operation when identity is
derived from existing topology. Commit rejects faces or solids whose referenced
components have not been registered.

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
7. payload policy on net externally-visible changes;
8. `revision += 1` and cache invalidation.

A failure at any step restores the transaction-start snapshot whole.

## Derived indexes

The six dart-to-key maps are one lazy `DerivedCellIndexes` cache and the
dart-to-owner lookup is a second one. Any mutation invalidates both; the first
typed lookup or traversal rebuilds what it needs, subsequent reads reuse it, and
commit materializes the cell indexes after reconciliation. Neither is
serialized: a deserialized model rebuilds them from its authoritative stores.
Builders use typed APIs such as `cell_key`, `attribute`, `ownership` and
topology views rather than accessing indexes directly.
