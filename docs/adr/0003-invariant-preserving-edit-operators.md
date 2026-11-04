---
status: accepted
date: 2026-11-04
---

# Invariant-preserving edit operators, and fallible reads on an open edit

A view read in the middle of a transaction (`Face::loops`, `Sheet::faces`)
goes through the embedding index, which is derived from the labelling and
assumes the labelling describes the map. Commit is the only place that
assumption was checked, so a builder that had written its first step and not
its second could read a state no index can be built for, and the read
panicked. The panic left the transaction without rollback and the caller
without an error. Three builders met it: a disc swept along a line, a Boolean
on a V-belt pulley, and every other place a closed edge is sewn to a marked
one.

The model now keeps its labelling valid between the steps a builder takes, and
turns what still goes wrong into an error:

- **Operators.** `ModelEdit` has a small set of operators (`mark_edge`,
  `cut_edge`, `sew_edges`, `unsew_edge`) that rewire the map *and* relabel in
  one call. Each reads what it needs first, refuses a precondition that does
  not hold before it changes anything, and checks the cells it touched before
  it returns. Between two operators the labelling describes the map, so every
  view answers.
- **Deferred surgery.** A rewiring that must pass through invalid states runs
  in `edit.deferred(|raw| ..)`. `RawEdit` has the primitives and the stored
  attributes and no view, so nothing in the block can read through a labelling
  that is allowed to be wrong. The map's axioms and the labelling are checked
  when the block closes, and a block that leaves either broken is an error
  naming that block.
- **Fallible reads.** `Model::try_embedding_index` returns a `Result`. The
  views that cannot return one (`Face::loops`) record the fault and answer
  with nothing; the transaction reports the first fault as its error, rolls
  back, and also converts a panic that follows one.
- **Commit post-condition.** Every solid a transaction created or touched must
  have closed, consistently oriented shells with outward normals and positive
  volume, or the transaction does not commit. On in debug builds and tests, off
  in release, switchable per model (`CommitChecks`).

## Considered options

The survey is in `plan/robustness/wsC-edit-safety.md`; the four options it
weighs, by analogy with a database:

- **(a) Fallible reads only.** Kept, as the last net. Alone it is not a
  design: the error surfaces at the read, far from the step that broke the
  labelling, and says nothing about which step that was.
- **(b) Immediate constraints.** Chosen as the rule: the step that would break
  the labelling is the one that refuses, so the error names it, and a read is
  always valid. Built as operators rather than as a check inside each
  primitive, because a primitive rewires one thing and a state valid after
  every primitive does not exist (`sew` links two darts, and the labelling is
  wrong until the corner it merged is relabelled).
- **(c) Snapshot reads.** Good practice, and the blend engine already works
  this way, but it cannot serve a builder that reads what it just wrote (the
  Boolean splits faces and then classifies the split model).
- **(d) Savepoints.** Unnecessary once each stage is made of operators.

Maintaining the index *incrementally* on each write was also considered. The
index is a cache derived from the labelling, and every primitive already
invalidates it; an operator relabels in the same call, and the next read
derives the index from a labelling that is valid. Updating the map from dart to
owner in place would save one rebuild per read after a write and add a second
description of the labelling that has to be kept in step with the first, which
is the failure this decision removes. It stays a derived cache.

## Consequences

- The raw primitives (`add_dart`, `link`, `unlink`, `sew`, `own_cell`,
  `disown_cell`) remain on `ModelEdit`, because about two hundred call sites
  use them. A builder that uses them is covered by the fallible reads, not by
  the operators. A builder moves onto operators or into a deferred block one at
  a time (`sew_edges` in the sweep, `mark_edge` and `cut_edge` in the edge
  splits, `CutAttachment::move_after` as a deferred block); the primitives leave
  `ModelEdit`, and `RawEdit` keeps them, when the last builder is off them.
- A builder's mistake in the labelling now surfaces as an error from the
  transaction (`InvalidEmbedding`, `UnreadableBoundary`, `InvalidSolid`), and
  its model is restored. The bench cases that crashed move to refused, and
  those a builder bug had left unfinished stay refused until the bug is fixed.
- Commit costs one orientation check per touched solid in debug builds. A
  release build that wants the same guarantee turns it on.
- `Transactions intentionally do not catch panics` is no longer the whole
  story: a transaction rolls the model back when a panic crosses it, so a
  caller that catches the panic holds a valid model, and reports a panic that
  follows a recorded read fault as that fault. A panic with no such fault
  continues to unwind.
