# GMap operation transactions

`GMap::transaction` is the atomic boundary for a complete modeling operation.
Its closure receives a `TopologyEdit`, which is the public mutation capability
for the staged map. Returning an error, failing validation, failing identity
reconciliation, or failing payload policy restores the complete
transaction-start snapshot.

`GMap::transaction_with_policy` uses the same boundary with a caller-provided
`EditPolicy`. Policy event application happens only after the complete staged
operation passes topology validation and identity reconciliation.

Transactions intentionally do not catch panics. Operation code uses `Result`
for recoverable failure.

## Builder composition

Each public builder accepts `&mut GMap` and starts one transaction. Its private
staged helper accepts `&mut TopologyEdit` and performs the actual work. A
composite builder calls other staged helpers with the same edit capability, so
the whole modeling operation has one snapshot, one journal, and one commit.

Raw `GMap` mutation is topology-internal. Builders can inspect the map through
the immutable access exposed by `TopologyEdit`, but cannot bypass the
transaction when adding darts, changing alpha links, or mutating attributes.

`TopologyEdit` owns no snapshot and has no independent commit. It provides the
checked alpha operations `add_dart`, `remove_dart`, `link`, `unlink`, and `sew`,
plus topology-associated attribute creation, removal, mutation, split, and
merge declarations.

## Lineage and policy

Plain `add_*` records a fresh transaction-local identity. `add_*_split_from`
records an identity derived from a source key. `merge_*_into` explicitly names
the surviving and consumed identities.

At commit, merge chains are resolved to their final survivor. Policy is
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
or traversal rebuilds it, subsequent reads reuse it, and commit
materializes it after reconciliation. Builders use typed APIs such as
`cell_key`, `attribute`, and topology views rather than accessing indexes
directly.
