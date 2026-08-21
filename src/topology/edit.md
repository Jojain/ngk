# GMap operation transactions

`GMap::transaction` is the atomic boundary for a complete modeling operation.
Builders keep accepting `&mut GMap`, so a builder invoked inside another
builder automatically joins the transaction already active on that map. Only
the outermost operation validates, reconciles identities, applies payload
policy, and commits.

`GMap::transaction_with_policy` gives the outermost operation a custom
`EditPolicy`. A nested operation cannot replace that policy. If any nested
closure returns an error, the transaction is poisoned; catching the nested
error does not permit the outer operation to commit. Returning an error from
the outer closure, poisoning, validation failure, reconciliation failure, or
policy failure restores the complete transaction-start snapshot.

Transactions intentionally do not catch panics. Operation code uses `Result`
for recoverable failure.

## Low-level edits

`GMap::edit` opens a short `TopologyEdit` alpha-mutation batch. Used alone, it
creates an implicit operation transaction with `PreservePayload`. Used inside
a builder transaction, it mutates the staged map and appends lineage events to
the outer journal without validating or committing independently.

`TopologyEdit` owns no snapshot and has no independent commit. It provides the
checked alpha operations `add_dart`, `remove_dart`, `link`, `unlink`, and `sew`,
plus topology-associated attribute creation, removal, mutation, split, and
merge declarations.

## Lineage and policy

Plain `add_*` records a fresh transaction-local identity. `add_*_split_from`
records an identity derived from a source key. `merge_*_into` explicitly names
the surviving and consumed identities.

At outer commit, merge chains are resolved to their final survivor. Policy is
then applied only to net changes visible outside the operation:

- a surviving split derived from a transaction-start identity;
- an explicitly consumed transaction-start identity;
- never a fresh or split identity that was created and discarded inside the
  operation.

Policy callbacks run in declaration order and receive payloads from the
transaction-start snapshot. `PreservePayload` clones split payloads and keeps
the merge survivor payload. A policy error restores topology and payloads.

## Identity reconciliation

After structural validation, attributes that now describe the same final cell
are grouped for each attribute kind. One transaction-start key beats local
keys. If all keys are local, the earliest-created key wins. Multiple
transaction-start keys require explicit lineage naming a survivor. Losing
local attributes are discarded without invoking merge policy.

Keys returned during an operation are stable only when they survive this
reconciliation. A temporary local key may be removed at commit; staged typed
lookups resolve the operation's logical survivor.

## Derived cell indexes

The six dart-to-key maps are one lazy `DerivedCellIndexes` cache. Topology or
topology-associated attribute mutation invalidates it. The first typed lookup
or traversal rebuilds it, subsequent reads reuse it, and the outer commit
materializes it after reconciliation. Builders use typed APIs such as
`cell_key`, `attribute`, and topology views rather than accessing indexes
directly.
