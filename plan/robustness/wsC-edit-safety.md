# WS C: no panic on an open edit, and post-conditions at commit

**Owns:** `src/model.rs`, `src/topology/edit*`, `src/topology/validation.rs`.
**Decision:** D5 ([ADR 0003](../../docs/adr/0003-invariant-preserving-edit-operators.md)).
**Size:** small to medium. Enables B and F.
**Status:** the infrastructure, raw-capability migration and the verified
commit/alpha3 corrections are in. Panic investigation is driven by reproduced
contract violations, not by blanket replacement of `expect`.

## Done

- **Raw capabilities separated.** The topology primitives on `ModelEdit` are
  inaccessible to builders and external callers. All raw builder and fixture
  surgery uses `RawEdit` without typed views, checked at the block boundary.
- **Verified safety gaps corrected.** Commit checks all solids, including a
  solid whose connectivity changed without lineage events. `cut_edge` mirrors
  alpha3 partners on its new darts. Boundaryless face replacements update
  sheet/solid anchors and remove detached scaffold; seam demotion retains a
  revolved meridian's finite span.

- **Fallible reads** (option a). `Model::try_embedding_index`; the views that
  cannot return a `Result` record the first fault, the transaction rolls back
  and reports it (`InvalidEmbedding`, `UnreadableBoundary`) whatever the
  operation returned. Panics propagate without recovery, including after a
  recorded fault; rollback and model reuse are not guaranteed after a panic
  ([ADR 0004](../../docs/adr/0004-panics-are-not-recoverable-errors.md)).
- **Operators** (option b), `src/topology/edit_operators.rs`: `mark_edge`,
  `cut_edge`, `sew_edges`, `unsew_edge`, `check_consistent`. Tests in
  `tests/topology/edit_operators.rs` call `check_consistent` and read faces
  after every call, and check that a refused call changes nothing.
- **Deferred block**, `src/topology/edit_deferred.rs`: `ModelEdit::deferred`
  and `RawEdit`, which has the primitives and the stored attributes and no
  view.
- **Commit post-condition**: `CommitChecks::solid_orientation` (default on
  with debug assertions; explicit in the release regressions), enforced on
  every registered solid, error
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
| disc swept along a straight line | builds an outward solid; success and volume regressions enabled | none for this reproduction |
| disc swept along a cubic Bézier | `DegenerateFrame`: `initial_frame` seeds the frame with a world axis that is not square to the tangent | WS B: orthogonalize the seed |
| pulley body cut by the revolved groove | `InvalidEmbedding(ConflictingOwnership)` when the deferred block of `CutAttachment::move_after` closes | WS F: the ring split leaves a vertex and an edge's scaffold on one 0-cell |
| revolve by a negative angle | `InvalidSolid`: the shell's volume is not positive | WS B: orient by the sign of the angle |

## Remains

- **Index maintained on write.** The index is a derived cache rebuilt on the
  next read after an invalidation; operators leave the labelling valid, so the
  rebuild succeeds. Updating it in place is possible per operator and is not
  needed for correctness.
- **Release builds do not check solids by default.** `CommitChecks` can turn it
  on; the bench runner could.
## Panics audit

`expect`, `unwrap`, `panic!`, `unreachable!` and `assert!` outside test code,
counted per area (about 260 in the initial inventory). These are not presumed
bugs. Keep an `expect` when its invariant follows from the construction. A
reproduced panic calls for explaining the violated contract before choosing a
correction; use `Result` only for a legitimate recoverable condition. Disposition:

| Area | Count | Disposition |
|---|---|---|
| `model.rs`, `topology/edit.rs`, `topology/validation.rs` | 46 | `*_unchecked` accessors (`vertex_unchecked`, `edge_attr_unchecked`, ...) panic by contract and have a fallible twin (`vertex`, `edge_attr`, ...); the commit-internal `expect`s follow a check that precedes them in `commit_model_transaction` (`validate_edit_events` before reconciliation). `validate_shell` no longer expects a registered sheet. Done. |
| `topology/attributes.rs`, `face.rs`, `edge.rs`, `profile.rs`, `sheet.rs`, `solid.rs` | ~35 | payload read before commit assigned it (`data()`, unreachable outside the commit order), and views that document `# Panics`. `Face::boundary_walks` no longer panics. Remaining: `Loop::loop_from_seed`, `Solid::shells`, `Sheet::faces` on a state where the sheet is unregistered. |
| `geometry/dim3/frame.rs` | 2 | `from_xz`, `from_xy`, `from_yz` panic by contract; `try_` twins exist. Builders use them in sweep, revolve and extrude. Remaining: `builders/blend/{section/revolved,vertex/conic,vertex/trihedral}.rs`, `measure/bounding_box.rs`, `exchange/step/convert`, `topology/planar.rs`. |
| `builders/faces` | 40 | 12 are `Profile::from_dart(..).expect("..registered profile")` on a loop the same builder just registered; the `fresh ...` links in `support.rs:1100-1151` and `imprints.rs:1245` act on darts added a few lines above; `source face must remain staged` follow `edit.face_attr(face)` checks. Investigate only on evidence of a violated contract. |
| `builders/boolean` | 20 | `pair.rs` panics on a mis-addressed contact slot (internal addressing); `broad_phase.rs:259-270` are in test helpers; `operand.rs`, `split.rs` expect a registration they copied. Review the contract when a reproduction fails. |
| `builders/revolve`, `builders/solids`, `builders/profiles`, `builders/sheets` | 27 | `registered` and `non-empty` expects after validation in the same function. Review on a reproduced failure. |
| `builders/sweep` | 5 | one converted (`EmptySpine`); the other four carry a comment proving them unreachable. |
| `builders/blend`, `builders/loft`, `builders/removal`, `builders/transform.rs`, `modeling/*` | ~15 | as above; `transform.rs:49` rewrites geometry over an unchanged topology. |
| `geometry`, `exchange`, `tessellate`, `solver`, `measure` | ~60 | outside this workstream's paths; not audited. |

## Validation

- `tests/topology/edit_safety.rs` checks that opening an existing solid is
  refused at commit and restores the complete transaction-start model. It
  enables the solid check explicitly, including in release builds.
- `tests/topology/edit_operators.rs` checks the alpha3 connections created by
  `cut_edge`, GMap validity and the resulting entity keys.
- `tests/builders/faces.rs` checks aggregate references and volume when a
  boundary replaces a boundaryless face's scaffold.
- `tests/builders/removal.rs` checks volume preservation when a revolved
  sphere loses its last seam.
- Seven formerly ignored robustness tests are enabled. Pulley construction
  and the quarter-torus/helix volume checks remain ignored for their actual
  ownership or numerical failures, with updated reasons.

## Design rationale

A builder must not read derived views while its topology and labelling are
half updated. Operators restore that coherence by construction; a deferred
block supplies raw capabilities without views and checks coherence when it
closes. The transaction checks its final post-conditions. The alternatives
and the operator/deferred decision are recorded in
[ADR 0003](../../docs/adr/0003-invariant-preserving-edit-operators.md).
Panic propagation follows
[ADR 0004](../../docs/adr/0004-panics-are-not-recoverable-errors.md).
