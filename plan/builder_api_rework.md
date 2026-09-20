# Builder API rework

Implements the builder-side conclusions of
[builders_payload_policy.md](builders_payload_policy.md): one public entry per
operation, one private kernel function that takes the edit, one file per
operation, and a typed result carrying everything the operation learned.

Payload and policy work is not part of this. Neither is the
`builders` / `modeling` rename.

## Decisions

- **Naming.** The private kernel function is the public one with a leading
  underscore: `add_rectangle` / `_add_rectangle`, `split_face_edge` /
  `_split_face_edge`. Mechanical, works for every verb, and both live in the
  same file.
- **Layout.** A builder family is a directory when it holds more than one
  public operation; each operation gets its own file. Families with one
  operation stay a single file.
- **Module names.** `src/builders/` keeps its name. `src/modeling/` and `Shape`
  are untouched.
- **`_staged` disappears** as a word and as a suffix.

### Known cost of the underscore prefix

`rustc`'s `dead_code` lint skips items whose name begins with `_`. A
`_add_rectangle` left behind after its public wrapper is deleted will not be
reported. Removing an operation therefore means deleting both halves by hand;
the compiler will only catch the public one.

## The convention

To be written into `AGENTS.md` once the first family lands.

Every kernel operation is a pair in one file:

```rust
/// Adds a planar rectangular face whose first corner is `plane.origin()`.
pub fn add_rectangle<P: Payload>(
    g: &mut Model<P>,
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|edit| _add_rectangle(edit, plane, x_size, y_size))
}

/// Builds the rectangle's profile and face inside an open edit.
pub(crate) fn _add_rectangle<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<FaceKey, FaceCreationError> {
    let profile = _add_rectangle_profile(edit, plane, x_size, y_size)?;
    _add_face(edit, profile)
}
```

Rules:

1. **The public function opens one transaction and does nothing else.** No
   validation, no logic, no error mapping — a one-line body.
2. **The `_` function holds the logic and takes `&mut ModelEdit`.** It is
   `pub(crate)`, so any other kernel operation can compose it.
3. **A composite passes the same `edit` down.** Never open a transaction inside
   one: `run_transaction` asserts no transaction is active.
4. **Close view borrows before mutating again.** `edit.face(k)` borrows the
   edit; collect what is needed into keys and end the borrow before the next
   staged call.
5. **Errors compose with `#[from]`**, never flatten. A composite's error wraps
   each sub-operation's error unchanged.
6. **The result carries everything the operation learned** and nothing a single
   traversal step recovers.
7. **Do not hold a key across a step that may reconcile it away.** Re-resolve
   through a typed lookup when a later step merges or splits an earlier step's
   output.

## Inventory

### Already returning a typed result

`split_edge` → `EdgeSplit`, `split_face_edge` → `EdgeSplit`,
`split_face_by_imprints` → `Vec<FaceImprintSplit>`, `remove_cell` →
`CellRemoval`, `boolean` → `BooleanResult`, `add_loft` → `S::Output`. These are
the precedent; they need the file move and the rename, not a new result type.

### Returning less than they know

| operation | today | needs |
|---|---|---|
| `chamfer` | `()` | the chamfer faces, and the edge each replaced |
| `add_extruded_face` | `SolidKey` | caps, laterals, and the base edge each lateral was swept from |
| `add_extruded_profile` | `SheetKey` | the same, one dimension down |
| `add_revolved_face` | `SolidKey` | caps, laterals, seam |
| `add_revolved_profile` | `SheetKey` | laterals, seam |
| `add_revolved_edge` | `FaceKey` | the face plus its start/end/seam edges |
| `add_sphere`, `add_torus` | `SolidKey` | the faces and seams they synthesized |
| `append_edge` | `()` | at minimum the profile it extended |

Primitives whose whole result is one key — `add_line`, `add_arc`, `add_circle`,
`add_face`, `add_rectangle`, `add_square` — keep returning the key. A result
struct earns its place at more than two keys or any nesting.

### Public functions that take `ModelEdit` and must stop being public

`reverse_face_winding`, `split_face_by_imprints_staged`, `remove_cell_staged`,
`add_polyline_staged`, `add_profile_from_edges_staged`, `add_profile_darts`.
These become `_`-prefixed and `pub(crate)`. Helpers that take `&Model` and are
genuinely useful to a caller (`is_removable`, `can_remove_cell`,
`planned_merge`, `solid_contains_point`, `face_key_for_dart`, `profile_pcurves`,
`plane_uv`) stay public and are not renamed.

## Work

Each family is one reviewable change: split the files, rename the pair, add the
result type, update call sites. Ordered so the pattern is established on a small
family before the large ones.

### 1. `Model::transaction_result` and `StaleResult`

A result names the revision it was made at, but the revision only increments at
commit, so a `_` function cannot stamp its own result. The public wrapper can:

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

A `_` function returns an unstamped result; only the public wrapper stamps it.
`view()` on an unstamped result must fail rather than compare against zero, so
the stamped revision is `Option<u64>` internally and `StaleResult` distinguishes
the two cases.

### 2. `builders/edges/` — establish the pattern

`edges.rs` (617 lines) → `edges/{mod,line,arc,circle,split,support}.rs`.
Rename four pairs. No new result types (`EdgeSplit` already exists). This is the
reference change every later family copies.

### 3. `builders/profiles/`

`profiles.rs` (545) → `{mod,polyline,rectangle,from_edges,append,pcurves}.rs`.
`append_edge` gains a result. `add_polyline_staged`,
`add_profile_from_edges_staged` and `add_profile_darts` stop being public.

### 4. `builders/solids/` + extrusion results

`solids.rs` (630) → `{mod,extrude,sphere,torus,translate,lateral,support}.rs`.
Add `Extrusion` / `Lateral` and their views. `add_extruded_face` and
`add_extruded_profile` return them. This is the first family where a real result
type is designed, and `add_extruded_face_staged` becoming `pub(crate)
_add_extruded_face` is what makes composites possible at all.

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

Under "Transactions", replacing the "Builder composition" paragraph in
`src/topology/edit.md` that still says `*_staged`.

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
