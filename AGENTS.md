# AGENTS.md — NGK (Nales Geometry Kernel)

Working memory for agents (Claude Code, Codex, and others) in `D:\Projets\ngk`.
Read this first, then the pointed-to docs.

Shared skills live in `skills/` at the repo root — `.claude/skills` is a local
directory junction onto it, so skill content is edited in one place only.
The junction is machine-local (not tracked in git); recreate it after a fresh
clone with:

```powershell
New-Item -ItemType Junction -Path ".claude\skills" -Target "skills"
```

OPUS AGENTS :
Keep responses focused, brief, and concise. Keep disclaimers and caveats short, and spend most of the response on the main answer. When asked to explain something, give a high-level summary unless an in-depth explanation is specifically requested.

## What this project is

NGK is an experimental CAD geometry kernel in Rust whose topology is a
generalized map (3-GMap), with geometry and user payload attached to the logical
entities read off it. Author: Jojain (romain.ferru@gmail.com). Pages demo:
https://jojain.github.io/ngk/

## Where knowledge lives

- **[CONTEXT.md](CONTEXT.md)**: the vocabulary. Read it before naming anything.
  The one rule: **a bare noun is always the logical entity**. A thing the map
  holds is an `N-cell` or a raw cell, never just "a face" — in doc comments,
  inline comments, test names, error messages and plan prose alike.
- **[docs/architecture/](docs/architecture/README.md)**: how each subsystem
  works, from [overview.md](docs/architecture/overview.md) down. Read the one
  for the area you touch.
- **[docs/adr/](docs/adr/)**: decisions; read those touching your area.
- **[docs/architecture/reviews/](docs/architecture/README.md#reviews)**: dated
  surveys and reviews behind the decisions.
- **[plan/](plan/)**: work not done yet.

A change that alters what an architecture document describes updates that
document in the same change. A decision that is hard to reverse, surprising
without context, and the result of a real trade-off gets an ADR.

## Working rules

- **Deleting behaviour means deleting or adapting its tests.** A change is not
  finished while a test still asserts, names, or explains the thing that was
  removed. Grep for the concept by name, not just for compile errors: a test that
  still passes can be testing nothing, and one whose name or comment describes the
  old design is worse than no test, because it teaches the next reader something
  false. Three outcomes, in order of preference — the test states a property that
  survives, so keep it and fix the wording; the property moved to another shape,
  so retarget it there; the property is genuinely gone, so delete the test. Never
  leave the fourth. The same goes for doc comments, error-variant docs, `#[ignore]`
  reasons, and plan files: an `#[ignore]` whose stated reason no longer applies
  must be re-checked, since the test may now pass or fail for a new reason.
- **Never `git checkout`, `git restore`, or `git stash` a file to undo your own
  edit.** The working tree holds uncommitted work that is not recoverable. Undo
  by making the inverse edit, and use a marker comment you can grep for when
  adding temporary instrumentation.
- **A comment describes the code as it is, never a plan for it.** Doc comments
  and inline comments state what the thing does, why it is shaped that way, and
  what it refuses — all in the present tense, all checkable against the code
  next to them. They must not reference a plan document, a milestone, a stage
  number, or a decision id (`plan/foo.md`, "stage 4", "D7", "arrives later",
  "not implemented yet"), and must not describe behaviour that does not exist
  yet. Keep the *reason* and drop the schedule: "a cylinder wall closes on
  itself, which STEP cannot spell without a synthesized seam" is factual and
  stays true; "…which needs seam synthesis (stage 4)" is a promise that rots the
  moment the plan moves, and the reader cannot tell from the code whether it
  still holds. Plans belong in `plan/`, which is where they can be revised in
  one place. This applies to module docs, error-variant docs, test comments and
  commit-adjacent prose alike.
- **One public builder, one transaction.** The public function is a one-line
  transaction boundary over a `pub(crate)` `_edit` operation taking
  `&mut ModelEdit`; composite builders pass the same edit down to other `_edit`
  operations. See [transactions.md](docs/architecture/transactions.md).
- **`mod.rs` files are module manifests**: they declare modules and re-export
  their API. Non-trivial logic belongs in named child files.
- **Traverse with typed views** (`Face`, `Edge`, `Profile`, …), not raw darts;
  select entities by key, never by dart.

## Agent skills

### Issue tracker

Issues live in GitHub Issues on `Jojain/ngk` (via the `gh` CLI). See `docs/agents/issue-tracker.md`.

### Triage labels

Default vocabulary: `needs-triage`, `needs-info`, `ready-for-agent`, `ready-for-human`, `wontfix`. See `docs/agents/triage-labels.md`.

### Domain docs

Single-context: one root `CONTEXT.md` plus `docs/adr/`. See `docs/agents/domain.md`.
