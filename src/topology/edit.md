# Topology edit transactions

`GMap::edit` is the default public API for alpha topology mutation. It gives a
builder a temporary `TopologyEdit`, then commits all topology, semantic identity,
payload, and attribute-index updates as one operation. Use
`GMap::edit_with_policy` when payload propagation needs a custom policy.

The current implementation is intentionally clone-backed:

1. Clone the complete `GMap` before the edit closure runs.
2. Let the closure mutate the staged map through `TopologyEdit`.
3. If the closure returns an operation error, drop the transaction and restore
   the clone.
4. If the closure succeeds, validate the staged alpha relations with
   `validate_gmap`.
5. Ensure mandatory structural attributes exist.
6. Apply explicit split/merge events through the edit policy.
7. Rebuild dart-to-key indexes and reject duplicate keys for the same cell.
8. If validation, policy, or reconciliation fails, drop the transaction and
   restore the clone.
9. If commit succeeds, discard the clone and keep the staged map.

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

## Semantic identity operations

Builders declare domain intent explicitly. The edit layer does not infer that a
key was split or merged by looking at topology.

Creation methods mean "this is a new entity":

- `add_vertex`, `add_edge`, `add_profile`, `add_face`, `add_sheet`, `add_solid`

Split methods mean "this created key derives from this source key":

- `add_vertex_split_from(source, attr)`
- `add_edge_split_from(source, attr)`
- `add_profile_split_from(source, attr)`
- `add_face_split_from(source, attr)`
- `add_sheet_split_from(source, attr)`
- `add_solid_split_from(source, attr)`

Merge methods mean "remove this key and merge its payload into this survivor":

- `merge_vertices_into(survivor, removed)`
- `merge_edges_into(survivor, removed)`
- `merge_profiles_into(survivor, removed)`
- `merge_faces_into(survivor, removed)`
- `merge_sheets_into(survivor, removed)`
- `merge_solids_into(survivor, removed)`

These methods only record intent during the edit closure. They are applied at
commit, after the staged alpha topology has validated.

## Reconciliation at commit

After topology validation and semantic event application, commit rebuilds every
derived dart-to-key index:

- vertices: grouped by canonical 0-cell representative;
- edges: grouped by canonical 1-cell representative;
- profiles: grouped by profile representative;
- faces: grouped by canonical 2-cell representative of each outer loop;
- sheets: grouped by canonical 3-cell representative;
- solids: grouped by shared shell representatives.

If two live keys point to the same representative after explicit merge events
have been applied, commit fails. The builder must declare which key survives
with the appropriate `merge_*_into` method.

Before reconciliation, commit also materializes mandatory container attributes:

- every face loop must have a profile;
- every solid shell must have a sheet.

These default attributes are structural placeholders.

## Payload policy

`EditPolicy` is called only from explicit semantic events:

- split events initialize the created payload from the source payload;
- merge events combine the removed payload into the survivor payload.

`PreservePayload` is the default policy:

- split: clone the source payload into the created payload;
- merge: keep the survivor payload and discard the removed payload.

Custom policies are passed through `GMap::edit_with_policy`.

## Important constraints

- Mid-edit dart-to-key indexes may be stale. `TopologyEdit` therefore does not
  dereference to `GMap`; callers should not rely on normal view lookups while
  alpha topology is being staged.
- Attribute mutation APIs still exist on `GMap` for non-topological data writes.
  Alpha topology mutation should go through `GMap::edit`.
- The clone-backed design is simple and safe, not final. A future journal-based
  implementation can keep the same external edit/policy model while replacing
  the rollback mechanism.
