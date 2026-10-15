# Architecture refactors

A memo of refactors someone noticed while working on something else and did
not pursue. Each entry says what overlaps or fits badly, where, and what a
refactor might look like. Nobody has checked how far the change reaches or
whether it is worth doing.

Read the entries touching your area before starting work there. If the work
already goes through the same code, that is the time to look at the refactor
properly, then do it, reject it, or record what you learned.

## Writing an entry

- One file per refactor, named `YYYY-MM-DD-short-subject.md`.
- Say what was seen, with paths, and the refactor it suggests.
- Say what was **not** checked: callers, behaviour that may differ, cost.
- When the refactor lands or is rejected, delete the entry. Move any
  lasting reasoning into the architecture document or an ADR.

## Entries

| Date | Entry | Area |
|---|---|---|
| 2026-10-15 | [One boundary walk for face domains](2026-10-15-face-domain-boundary-walk.md) | `topology::unwrapped_face_domain`, `measure`, `tessellate`, `builders::boolean::trim` |
