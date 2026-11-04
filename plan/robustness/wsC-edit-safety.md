# WS C: no panic on an open edit, and post-conditions at commit

**Owns:** `src/model.rs`, `src/topology/edit*`, `src/topology/validation.rs`.
**Decision:** D5 ([ADR 0003](../../docs/adr/0003-invariant-preserving-edit-operators.md)).
**Size:** small to medium. Enables B and F.
**Status:** the infrastructure is in; builder migration and the panics audit are
partly done. "Done" and "Remains" below say exactly what.

## Done

- **Fallible reads** (option a). `Model::try_embedding_index`; the views that
  cannot return a `Result` record the first fault, the transaction rolls back
  and reports it (`InvalidEmbedding`, `UnreadableBoundary`) whatever the
  operation returned, and a panic that follows a fault is reported as the
  fault. Any other panic rolls the model back and continues to unwind.
- **Operators** (option b), `src/topology/edit_operators.rs`: `mark_edge`,
  `cut_edge`, `sew_edges`, `unsew_edge`, `check_consistent`. Tests in
  `tests/topology/edit_operators.rs` call `check_consistent` and read faces
  after every call, and check that a refused call changes nothing.
- **Deferred block**, `src/topology/edit_deferred.rs`: `ModelEdit::deferred`
  and `RawEdit`, which has the primitives and the stored attributes and no
  view.
- **Commit post-condition**: `CommitChecks::solid_orientation` (default on in
  debug builds and tests), enforced on every solid created or touched, error
  `InvalidSolid`.
- **Builders ported**: `CutAttachment::move_after` (scaffold cuts moved between
  loops) runs in a deferred block; `sweep`'s `sew_edges` is the operator (this fixes the
  disc swept along a line: the labelling no longer breaks between the two cap
  sews); `edges::split_edge`/`split_face_boundary_edge` are `mark_edge` and
  `cut_edge`; sweep, revolve and extrude use the fallible frame constructors
  (`SweepError::DegenerateFrame`); `add_polygon_edit` returns an error instead
  of asserting.
- **The reproductions** are tests in `tests/topology/edit_safety.rs`, with a
  property test: random sequences of block, cylinder, sphere, torus, prism,
  revolve and swept-disc builders and Booleans never panic, and every `Ok` solid
  passes `validate_solid_orientation`.

What the reproductions answer now, until the builders they exercise are fixed:

| Case | Answer | Left to fix |
|---|---|---|
| disc swept along a straight line | `InvalidSolid` (`SolidFaceNormalNotOutward`): the labelling is right now, the swept solid's windings are not | WS B: sweep orientation |
| disc swept along a cubic Bézier | `DegenerateFrame`: `initial_frame` seeds the frame with a world axis that is not square to the tangent | WS B: orthogonalize the seed |
| pulley body cut by the revolved groove | `InvalidEmbedding(ConflictingOwnership)` when the deferred block of `CutAttachment::move_after` closes | WS F: the ring split leaves a vertex and an edge's scaffold on one 0-cell |
| revolve by a negative angle | `InvalidSolid`: the shell's volume is not positive | WS B: orient by the sign of the angle |

## Remains

- **Move builders off the raw primitives.** About two hundred calls to `link`,
  `unlink`, `sew`, `own_cell`, `disown_cell` are left, in `builders/scaffold.rs`
  (cuts and bridges), `builders/faces/{imprints,support,annulus}.rs`,
  `builders/revolve`, `builders/loft`, `builders/blend/execute.rs`,
  `builders/removal`, `exchange/step/topology/import.rs`. Candidates for new
  operators, from what they do: insert an edge across a face (the section
  chords of `imprints`), split and merge faces, make a face on a loop and bridge
  its holes (`cut_between_loops`), move a cut attachment
  (`CutAttachment::move_after`, now a deferred block, is where the pulley's
  labelling breaks: the error names it). The raw primitives leave `ModelEdit` when the last builder is off them.
- **Index maintained on write.** The index is a derived cache rebuilt on the
  next read after an invalidation; operators leave the labelling valid, so the
  rebuild succeeds. Updating it in place is possible per operator and is not
  needed for correctness.
- **Release builds do not check solids by default.** `CommitChecks` can turn it
  on; the bench runner could.

## Panics audit

`expect`, `unwrap`, `panic!`, `unreachable!` and `assert!` outside test code,
counted per area (about 260 in all). Disposition:

| Area | Count | Disposition |
|---|---|---|
| `model.rs`, `topology/edit.rs`, `topology/validation.rs` | 46 | `*_unchecked` accessors (`vertex_unchecked`, `edge_attr_unchecked`, ...) panic by contract and have a fallible twin (`vertex`, `edge_attr`, ...); the commit-internal `expect`s follow a check that precedes them in `commit_model_transaction` (`validate_edit_events` before reconciliation). `validate_shell` no longer expects a registered sheet. Done. |
| `topology/attributes.rs`, `face.rs`, `edge.rs`, `profile.rs`, `sheet.rs`, `solid.rs` | ~35 | payload read before commit assigned it (`data()`, unreachable outside the commit order), and views that document `# Panics`. `Face::boundary_walks` no longer panics. Remaining: `Loop::loop_from_seed`, `Solid::shells`, `Sheet::faces` on a state where the sheet is unregistered. |
| `geometry/dim3/frame.rs` | 2 | `from_xz`, `from_xy`, `from_yz` panic by contract; `try_` twins exist. Builders use them in sweep, revolve and extrude. Remaining: `builders/blend/{section/revolved,vertex/conic,vertex/trihedral}.rs`, `measure/bounding_box.rs`, `exchange/step/convert`, `topology/planar.rs`. |
| `builders/faces` | 40 | 12 are `Profile::from_dart(..).expect("..registered profile")` on a loop the same builder just registered; the `fresh ...` links in `support.rs:1100-1151` and `imprints.rs:1245` act on darts added a few lines above; `source face must remain staged` follow `edit.face_attr(face)` checks. To convert to errors. |
| `builders/boolean` | 20 | `pair.rs` panics on a mis-addressed contact slot (internal addressing); `broad_phase.rs:259-270` are in test helpers; `operand.rs`, `split.rs` expect a registration they copied. To convert to `BooleanError`. |
| `builders/revolve`, `builders/solids`, `builders/profiles`, `builders/sheets` | 27 | `registered` and `non-empty` expects after validation in the same function. To convert. |
| `builders/sweep` | 5 | one converted (`EmptySpine`); the other four carry a comment proving them unreachable. |
| `builders/blend`, `builders/loft`, `builders/removal`, `builders/transform.rs`, `modeling/*` | ~15 | as above; `transform.rs:49` rewrites geometry over an unchanged topology. |
| `geometry`, `exchange`, `tessellate`, `solver`, `measure` | ~60 | outside this workstream's paths; not audited. |

## Symptoms

- `Model::embedding_index` panics ("a committed model's labelling should
  describe its own map") when a builder reads a view (`Face::loops`) on a model
  whose transaction is still open and whose labelling a previous step left
  half updated. Seen in `sweep::sew_cap` and in
  `faces::split_face_by_imprints` under the Boolean. The panic escapes the
  transaction, so nothing is rolled back and no error reaches the caller.
- `Frame::from_xz` and similar constructors `expect` their preconditions and
  panic deep inside builders.
- A revolve by a negative angle commits a solid whose shell has zero signed
  volume: commit validates topology and embedding, not orientation.

## The class

Any builder bug that leaves the model inconsistent mid-edit, or produces an
invalid solid, must surface as an error from that builder with the model
restored, never as a panic or a committed invalid solid.

## Approach (D5)

Today a transaction works like a database transaction with deferred
constraints: it restores a snapshot on failure, and `validate_embedding`
checks the labelling only at commit. The embedding index, though, is built
lazily from the labelling and assumes those deferred constraints already
hold, so a read between two writes can meet a state no database index
would ever be in, and panics. The options, by analogy:

- **(a) Fallible reads.** The index build returns `Result` and views on an
  open edit propagate it; the transaction rolls back. A safety net, not a
  design: the error surfaces far from the step that caused it.
- **(b) Immediate constraints.** `own_cell` and the other labelling
  primitives refuse a conflicting claim when it is made (a 0-cell claimed by
  a vertex while an edge owns it as scaffold), and the index is maintained
  incrementally on each write, as a database maintains its indexes. The
  error then names the faulty step, and reads are always valid.
- **(c) Snapshot reads.** Builders plan from the transaction-start snapshot,
  which is immutable and valid, and only write into the edit; the blend
  engine already works this way (everything up to `assemble` reads the
  model as the call found it, only `execute` writes). Enforced by types:
  the plan phase gets `&Model`, the apply phase a `ModelEdit` with no views.
- **(d) Savepoints.** An operation that must read its own writes (the
  Boolean splits faces, then classifies the split model) validates at the
  end of each stage, and the next stage reads that validated state. A
  failure still restores the transaction-start snapshot.

Proposed, as one rule for every builder to come: (b), built as a small set
of **invariant-preserving operators** in the spirit of Euler operators
(Mäntylä): cut an edge (marking it and relabelling its 0-cell in one step),
insert an edge across a face, split and merge faces, sew and unsew along an
edge, make a face on a loop, and so on. Each leaves the model valid, and the
embedding index is maintained on each write. Builders compose them and may
read their own writes at any point. Raw dart and labelling calls stay
inside the operators. Surgery that has to pass through invalid states (the
blend's cut, re-embed and fill) runs in an explicit deferred block that
exposes no views, validated when it closes, like a database's deferred
constraints. (a) remains as the last net; (c) remains good practice; (d)
becomes unnecessary.

**Prior art, for orientation only (NGK writes its own).** CGoGN, the
combinatorial-maps library from Strasbourg, splits every operator in two: a
protected `*_topo` half that only rewires darts (`cut_edge_topo` unsews,
splits, re-sews) and a public half that calls it and updates every embedded
orbit in the same call (`cut_edge` in `cmap2.h`: new vertex and edge
embeddings, face and volume embeddings copied). The raw sews and `_topo`
operations are reachable only through a separate builder class
(`CMap2Builder_T`, a friend of the map) used by importers and surgery, and
`check_map_integrity` validates topology and embeddings afterwards. In
CGoGN 3 the same split is a `set_indices` flag on each operator, with
`check_integrity` for GMaps. The proposal above is the same shape: public
operators that keep labelling and index in step, a builder-like deferred
block for raw surgery, and a check when the block closes.

Whatever is chosen, fix the builders that leave the labelling inconsistent
(WS B1 for unmarked edges).

## Post-conditions at commit

Add a cheap orientation and closedness check for every solid a transaction
creates or touches (`validate_solid_orientation` already exists), on in
debug builds and in tests always, optionally in release. A builder that fails
it returns an error instead of committing.

## Panics audit

List every `expect`, `unwrap` and `panic!` reachable from a public builder,
and give each one an error path or a comment proving it unreachable.
`Frame::from_xz` gets a fallible twin used everywhere inside builders.

## Tests

- The crashes this workstream owns are tests in
  `tests/topology/edit_safety.rs`, asserting an `Err` or an outward solid, until
  WS B turns them into the right solid. Still to add when WS B lands:
  `sweep/mitred-pipe-on-cube-edges` and `sweep/circle-along-arc` as tests (the
  bench holds them for now).
- The property test is in the same file; `NGK_PROPTEST_CASES=300` runs a longer
  hunt (about seven minutes in a debug build).
