# Builder API rework

Implements the builder-side conclusions of
[builders_payload_policy.md](builders_payload_policy.md): one public entry per
operation, one private kernel function that takes the edit, one file per
operation, and a typed result carrying everything the operation learned.

Payload and policy work is not part of this. Neither is the
`builders` / `modeling` rename.

## Decisions

- **Naming.** The private kernel function is the public one with an `_edit`
  suffix: `add_rectangle` / `add_rectangle_edit`, `split_face_edge` /
  `split_face_edge_edit`. Mechanical, works for every verb, both live in the
  same file, and the suffix names the actual difference — it takes
  `&mut ModelEdit` (already open) instead of `&mut Model`. A leading
  underscore did the same job but is a known-bad idea in Rust: see below.
- **Layout.** A builder family is a directory when it holds more than one
  public operation; each operation gets its own file. Families with one
  operation stay a single file.
- **Module names.** `src/builders/` keeps its name. `src/modeling/` and `Shape`
  are untouched.
- **`_staged` disappears** as a word and as a suffix.

### Why not a leading underscore

The first pass of this work used a leading underscore (`_add_rectangle`) for
the kernel half of the pair. `rustc`'s `dead_code` lint skips any item whose
name begins with `_`, so a `_add_rectangle` left behind after its public
wrapper is deleted goes unreported — removing an operation would mean
deleting both halves by hand, with the compiler only catching the public one.
This was not hypothetical: renaming the pattern away from the underscore
(2026-09-20) immediately surfaced `add_profile_darts_edit` in
`src/builders/profiles/operations.rs` as genuinely dead — no public wrapper
called it and nothing else did either. It was deleted the same day. The
`_edit` suffix does not start with `_`, so this class of bug can't recur.

## The convention

Written into `AGENTS.md` under "Transactions".

Every kernel operation is a pair in one file:

```rust
/// Adds a planar rectangular face whose first corner is `plane.origin()`.
pub fn add_rectangle<P: Payload>(
    g: &mut Model<P>,
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|edit| add_rectangle_edit(edit, plane, x_size, y_size))
}

/// Builds the rectangle's profile and face inside an open edit.
pub(crate) fn add_rectangle_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<FaceKey, FaceCreationError> {
    let profile = add_rectangle_profile_edit(edit, plane, x_size, y_size)?;
    add_face_edit(edit, profile)
}
```

Rules:

1. **The public function opens one transaction and does nothing else.** No
   validation, no logic, no error mapping — a one-line body.
2. **The `_edit` function holds the logic and takes `&mut ModelEdit`.** It is
   `pub(crate)`, so any other kernel operation can compose it.
3. **A composite passes the same `edit` down.** Never open a transaction inside
   one: `run_transaction` asserts no transaction is active.
4. **Close view borrows before mutating again.** `edit.face(k)` borrows the
   edit; collect what is needed into keys and end the borrow before the next
   `_edit` call.
5. **Errors compose with `#[from]`**, never flatten. A composite's error wraps
   each sub-operation's error unchanged.
6. **The result carries everything the operation learned** and nothing a single
   traversal step recovers.
7. **Do not hold a key across a step that may reconcile it away.** Re-resolve
   through a typed lookup when a later step merges or splits an earlier step's
   output.

## Inventory (done)

Every operation below now returns a typed result: `split_edge` → `EdgeSplit`,
`split_face_edge` → `EdgeSplit`, `split_face_by_imprints` →
`Vec<FaceImprintSplit>`, `remove_cell` → `CellRemoval`, `boolean` →
`BooleanResult`, `add_loft` → `S::Output`, `chamfer` → `Chamfer`,
`add_extruded_face` → `Extrusion`, `add_extruded_profile` →
`ProfileExtrusionView`-backed result, `add_revolved_face` → `RevolvedFace`,
`add_revolved_profile` → `RevolvedProfile`, `add_revolved_edge` →
`RevolvedEdge`, `add_sphere`/`add_torus` → `ClosedSolid`, `append_edge` →
`AppendEdge`.

Primitives whose whole result is one key — `add_line`, `add_arc`, `add_circle`,
`add_face`, `add_rectangle`, `add_square` — still just return the key, per the
rule that a result struct earns its place at more than two keys or any
nesting.

### Public functions that take `ModelEdit` and must stop being public (done)

`reverse_face_winding_edit`, `split_face_by_imprints_edit`, `remove_cell_edit`,
`add_polyline_edit`, `add_profile_from_edges_edit`, `add_profile_darts_edit`
were all renamed to the `_edit` convention and are now `pub(crate)`.

Fixing the last three required moving the tests that called them directly
instead of through the public wrapper: `tests/builders/removal.rs` turned out
to only need `remove_cell` (its `map.transaction(|edit| remove_cell_edit(edit,
...))` calls are exactly what `remove_cell` already does — no test behavior
changed, just less code). `tests/builders/face_lineage.rs` and one test in
`tests/builders/profiles.rs` genuinely needed kernel-level access — they
exercise a builder composed with a *custom* `EditPolicy`
(`transaction_with_policy`), which the public wrapper never exposes since it
always runs `P::Policy::default()`. Those moved to `#[cfg(test)] mod tests`
inside `src/builders/faces/imprints.rs` and
`src/builders/profiles/operations.rs` respectively, next to the kernel
function they test. `add_profile_darts_edit` had no caller at all by the time
its visibility was checked — see "Why not a leading underscore" — and was
deleted rather than restored, since nothing in the codebase or its tests
needed it.

Helpers that take `&Model` and are genuinely useful to a caller
(`is_removable`, `can_remove_cell`, `planned_merge`, `solid_contains_point`,
`face_key_for_dart`, `profile_pcurves`, `plane_uv`) stay public and are not
renamed.

## Work (done)

Each family was one reviewable change: split the files, rename the pair, add the
result type, update call sites. Items 1–9 below are all landed (verified against
the current file layout and signatures); item 10 was landed for the leading-underscore
convention and has now been updated in place for `_edit` (see "Why not a leading
underscore" above) rather than re-run as a separate step.

### 1. `Model::transaction_result` and `StaleResult`

A result names the revision it was made at, but the revision only increments at
commit, so an `_edit` function cannot stamp its own result. The public wrapper can:

```rust
pub trait OpResult { fn stamp(&mut self, revision: u64); }

impl<P: Payload> Model<P> {
    /// Runs one operation and stamps its result with the committed revision.
    pub fn transaction_result<T, E, F>(&mut self, operation: F) -> Result<T, E>
    where
        T: OpResult,
        E: From<ModelEditError>,
        F: FnOnce(&mut ModelEdit<'_, P>) -> Result<T, E>,
    {
        let mut out = self.transaction(operation)?;
        out.stamp(self.revision);
        Ok(out)
    }
}
```

An `_edit` function returns an unstamped result; only the public wrapper stamps it.
`view()` on an unstamped result must fail rather than compare against zero, so
the stamped revision is `Option<u64>` internally and `StaleResult` distinguishes
the two cases.

### 2. `builders/edges/` — establish the pattern

`edges.rs` (617 lines) → `edges/{mod,line,arc,circle,split,support}.rs`.
Rename four pairs. No new result types (`EdgeSplit` already exists). This is the
reference change every later family copies.

### 3. `builders/profiles/`

`profiles.rs` (545) → `{mod,polyline,rectangle,from_edges,append,pcurves}.rs`.
`append_edge` gains a result. `add_polyline_edit`,
`add_profile_from_edges_edit` and `add_profile_darts_edit` stop being public
(the first is still `pub` — see "Public functions that take `ModelEdit`..."
above).

### 4. `builders/solids/` + extrusion results

`solids.rs` (630) → `{mod,extrude,sphere,torus,translate,lateral,support}.rs`.
Add `Extrusion` / `Lateral` and their views. `add_extruded_face` and
`add_extruded_profile` return them. This is the first family where a real result
type is designed, and the old `add_extruded_face_staged` becoming `pub(crate)
add_extruded_face_edit` is what makes composites possible at all.

### 5. `builders/faces/`

`faces.rs` (3253) → `faces/{mod,face,rectangle,square,circle,annulus,polygon,
split_edge,imprints/…,support}.rs`. The imprint machinery is most of the file
and becomes its own subdirectory. Drop the `StandardPayload` pins on
`add_rectangle`, `add_square`, `add_circle`, `add_annulus` and
`add_polygon_with_holes` while each is moved — the change is local to a file
that is being rewritten anyway.

### 6. `builders/chamfer/` + a chamfer result

`chamfer.rs` (1349) → `chamfer/{mod,solid_edge,vertex,profile,support}.rs`.
`chamfer` stops returning `()`.

### 7. `builders/revolve/`

`revolve.rs` (1665) → `revolve/{mod,edge,profile,face,seam,support}.rs`, with
results for all three entry points.

### 8. `builders/removal/`, `builders/loft/`

`removal.rs` (1713) and `loft.rs` (1093) split; renames only, results already
exist.

### 9. `sheets.rs`, `scaffold.rs`, `transform.rs`, `boolean/`

Renames and visibility only. `transform.rs` (50 lines) and `vertices.rs` stay
single files.

### 10. Write the convention into `AGENTS.md`

Done, under "Transactions" in `AGENTS.md` and the "Builder composition"
section of `src/topology/edit.md`. Both were updated again (2026-09-20) to
describe `_edit` instead of the leading underscore.

## Open design points

### A view-deriving macro

The view type and the accessor are both recoverable from the key type's name —
`FaceKey` gives `Face<'m, P>` and `Model::face` — but only a proc macro can read
an identifier as text and strip or case-fold it. `macro_rules!` cannot
manipulate identifiers at all.

So the two shapes are:

**`macro_rules!`, spelling the kind instead of the key type.** One token per
field, no new crate:

```rust
op_result! {
    pub struct Extrusion => ExtrusionView {
        pub solid: solid,
        pub start_cap: face,
        pub laterals: [Lateral],
    }
}
```

with helper arms mapping `face` to `FaceKey`, to `Face<'m, P>`, and to
`model.face(key)`. The cost is that the struct definition stops being readable
Rust: knowing that `face` means `FaceKey` requires knowing the macro.

**A `derive`, over an ordinary struct.** `ngk` becomes a workspace with an
`ngk-macros` member:

```rust
#[derive(OpResult)]
pub struct Extrusion {
    pub solid: SolidKey,
    pub start_cap: FaceKey,
    #[op_result(nested)]
    pub laterals: Vec<Lateral>,
}
```

The derive strips the `Key` suffix for the view type and case-folds it for the
accessor, and fails with a named error on a field type it does not recognise.
`syn`, `quote` and `proc-macro2` are already in the dependency graph through
`serde`'s derive and `thiserror`, so the marginal build cost is one small crate;
the real costs are publishing two crates in version lockstep and re-checking the
maturin and wasm builds.

**Hand-write the first three pairs, then take the `derive` if it is still
dull.** Not `macro_rules!`: it buys terseness by inventing a private DSL for
struct definitions, which is the one thing the derive avoids. And the derive's
attribute surface — nested results, `Vec`, whatever revolve's seams need — is
guesswork until the real cases exist.

### Whether `view()` is worth it for small results

`Extrusion` clearly wants one. `EdgeSplit` has three keys and no nesting, so
`model.edge(split.first)` at the call site may read better than a view struct.
Decide per result as each family lands, and record the outcome so the threshold
stops being re-argued.

## Out of scope

- The `builders` → `modeling` rename and the `Shape` facade.
- `ModelEdit` becoming `pub(crate)` in full. This work removes the public
  `ModelEdit`-taking *functions*; closing `ModelEdit` itself is a separate
  change, and doing it before an `Ops`-style capability exists would leave
  application code unable to compose at all.
- Selection and traversal combinators. The existing views are what there is.
- Every payload and policy item.
