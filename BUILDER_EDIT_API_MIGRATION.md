# Builder edit API migration

## Summary

This plan documents how to migrate `src/builders/*` and their script/test consumers to the new `GMap::edit` / `TopologyEdit` API. The target state is:

- all alpha topology mutation happens inside `GMap::edit` or `edit_with_policy`;
- builders explicitly declare semantic splits and merges;
- commit no longer fails with duplicate domain attributes after sewing;
- payload propagation is handled through `EditPolicy`, defaulting to `PreservePayload`.

## Current status

- Profile sewing, edge splitting, sheet sewing, and revolve sewing declare their
  split and merge lineage through `TopologyEdit`.
- Face imprint splitting now preserves the source `FaceKey` on the result that
  contains its oriented root and declares every other result with
  `add_face_split_from`.
- Closed-loop imprint islands use the same face-split policy path instead of
  copying payloads directly.
- Periodic seam face merging keeps the original face identity when it
  participates in the merge and declares the consumed face with
  `merge_faces_into`.
- Direct builder-only `GMap::remove_edge` and `GMap::remove_face` escape hatches
  have been removed; topology-associated removal goes through `TopologyEdit`.

The remaining periodic-imprint algorithm needs its own behavior milestone. A
standard ruled circular extrusion currently does not produce a split for an
individual constant-`u` imprint, so the periodic two-imprint seam path still
lacks a representative end-to-end regression test. This is separate from the
face identity and payload-lineage migration above.

## Key migration rules

- Fresh independent entities use plain creation:
  - `edit.add_vertex`
  - `edit.add_edge`
  - `edit.add_profile`
  - `edit.add_face`
  - `edit.add_sheet`
  - `edit.add_solid`

- Split operations keep the original key as one surviving result whenever possible, mutate its attr, and create the new result with:
  - `add_vertex_split_from`
  - `add_edge_split_from`
  - `add_profile_split_from`
  - `add_face_split_from`
  - `add_sheet_split_from`
  - `add_solid_split_from`

- Merge/sew operations must precompute affected keys before the edit closure, then declare consumed keys inside the closure:
  - alpha1/profile sewing may merge vertex keys;
  - alpha2/face-edge sewing may merge edge keys and endpoint vertex keys;
  - alpha3/shell sewing may merge face keys and lower-dimensional boundary keys if the sewn topology collapses them.

- Do not rely on commit to infer semantic lineage from topology. If two live keys end up on the same representative, the builder must call the relevant `merge_*_into`.

- Avoid direct builder calls to `g.add_*` / `g.remove_*` for topology-associated attributes when the surrounding topology is being edited. Move those attribute changes into the same edit closure unless they are clearly non-topological data updates.

## Implementation sequence

1. Migrate profile and polyline builders first.
   - Fix alpha1 sewing between adjacent segment endpoints.
   - Precompute the two vertex keys that will become one vertex.
   - Call `edit.merge_vertices_into(survivor, removed)` after the sew.
   - Use this to resolve the current open-profile duplicate vertex failure.

2. Migrate edge split builders.
   - Keep the original edge key for the first split part.
   - Mutate the original edge attr to the first curve.
   - Create the second edge with `edit.add_edge_split_from(source_edge, attr)`.
   - Add the new midpoint vertex as a fresh vertex unless it is explicitly derived from an existing vertex.

3. Migrate sheet/revolve/solid face sewing.
   - For each alpha2 sew between matching lateral faces, precompute:
     - surviving edge key;
     - removed edge key;
     - endpoint vertex key pairs that become identical.
   - In the edit closure, perform `edit.sew(Dim::Two, ...)`, then call `merge_edges_into` and the required `merge_vertices_into`.
   - Use a consistent survivor rule: keep the key from the earlier/source profile or earlier-created face, consume the translated/generated counterpart.

4. Migrate face imprint and face split operations.
   - Do not remove the source face when it semantically continues.
   - Update the source face attr to represent the first resulting face.
   - Add the second resulting face with `edit.add_face_split_from(source_face, attr)`.
   - When boundary edges are split, use `add_edge_split_from`; when edges are only section/chord edges, use plain `add_edge`.

5. Migrate builder consumers.
   - Update scripts and tests only after the underlying builder behavior is fixed.
   - Keep script code simple; scripts should call builders, not manually compensate for missing merge declarations.

## Test plan

- Keep current failing full-suite tests as regression drivers:
  - open profile adjacency;
  - two-face alpha2 shared edge;
  - sheet extrusion;
  - open polyline extrusion;
  - revolved triangle;
  - solid/sheet payload preservation.

- Add focused builder tests where useful:
  - profile sewing merges duplicate endpoint vertex keys;
  - edge split preserves source edge payload and initializes split edge payload through policy;
  - alpha2 sewing merges edge keys and endpoint vertex keys;
  - face split keeps source face key and creates one split-derived face key.

- Run after each migration chunk:
  - `cargo fmt`
  - focused test for the changed builder area
  - `cargo clippy --all-targets --all-features`
  - `cargo test --all-targets --all-features`

## Assumptions

- File path: `/D:/Projets/ngk/BUILDER_EDIT_API_MIGRATION.md`.
- Scope includes builders plus scripts/tests that consume the migrated builder behavior.
- The current `TopologyEdit` API is kept as-is for this migration.
- `PreservePayload` remains the default policy unless a test explicitly uses a custom policy.
- The migration should prefer preserving existing semantic keys over deleting and recreating entities.
