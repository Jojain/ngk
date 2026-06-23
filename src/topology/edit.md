# Topology edit transactions

`GMap::edit` is the only public API meant to mutate alpha topology. It gives a
builder a temporary `TopologyEdit`, then commits all topology and attribute-index
updates as one operation.

The current implementation is intentionally clone-backed:

1. Clone the complete `GMap` before the edit closure runs.
2. Let the closure mutate the staged map through `TopologyEdit`.
3. If the closure returns an operation error, drop the transaction and restore
   the clone.
4. If the closure succeeds, validate the staged alpha relations with
   `validate_gmap`.
5. Ensure mandatory structural attributes exist, then rebuild dart-to-key
   indexes.
6. If validation or reconciliation fails, drop the transaction and restore the
   clone.
7. If commit succeeds, discard the clone and keep the staged map.

This gives builders a context-manager-like API: they cannot forget to commit or
roll back. `TopologyEdit::Drop` restores the backup unless `commit` has marked
the transaction as successful.

## Staged topology operations

`TopologyEdit` exposes safe wrappers around low-level alpha changes:

- `add_dart` and `remove_dart` own dart lifecycle during an edit.
- `link` checks both darts exist, are distinct, and are free for the selected
  alpha.
- `unlink` checks the dart exists and is currently linked for the selected
  alpha.
- `sew` checks the GMap sewability condition first, then links the full dart
  mapping returned by `GMap::is_sewable`.

The raw alpha functions stay private to the topology module. This prevents
callers from changing alpha relations without also rebuilding the derived
indexes.

## Reconciliation at commit

After topology validation, commit rebuilds every derived dart-to-key index:

- vertices: grouped by canonical 0-cell representative;
- edges: grouped by canonical 1-cell representative;
- profiles: grouped by profile representative;
- faces: grouped by canonical 2-cell representative of each outer loop;
- sheets: grouped by canonical 3-cell representative;
- solids: grouped by shared shell representatives.

This pass is deliberately structural only. It does not infer that two keys
should merge, and it does not infer that a new key came from a split. If several
attributes point to the same representative, the current index rebuild keeps the
attributes and lets the last inserted representative mapping win. That is an
intermediate refactor state; explicit builder-declared merge/split events should
replace this.

Before reconciliation, commit also materializes mandatory container attributes:

- every face loop must have a profile;
- every solid shell must have a sheet.

These default attributes are structural placeholders.

## Payload policy

`EditPolicy` currently exists as a boundary for future semantic edit events. The
commit path does not automatically call payload merge or split hooks.

The intended next step is for builders to declare intent explicitly, for example
“this edge key was split from that edge key” or “these face keys merged into
this survivor.” Only those explicit events should drive payload policy.

## Important constraints

- Mid-edit dart-to-key indexes may be stale. `TopologyEdit` therefore does not
  dereference to `GMap`; callers should not rely on normal view lookups while
  alpha topology is being staged.
- Attribute mutation APIs still exist on `GMap` for non-topological data writes.
  Alpha topology mutation should go through `GMap::edit`.
- The clone-backed design is simple and safe, not final. A future journal-based
  implementation can keep the same external edit/policy model while replacing
  the rollback mechanism.
