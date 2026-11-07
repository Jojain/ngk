# Regions from a constraint system: what comes next

`add_sketch_from_system` draws one loose face per closed area a solved system's
curves enclose. It is described as built in
[builders.md](../docs/architecture/builders.md#regions-from-a-constraint-system),
and [ADR 0005](../docs/adr/0005-a-system-draws-loose-regions.md) records why the
result is loose faces. This plan keeps what is still open.

## Options on the builder

The caller decides what a sketch should become, and today has to assemble it:

- **One face from several regions.** A cross as a single outline: sew the regions
  with `add_sheet_from_faces`, then fuse them with `remove_redundant_cells`.
- **One sheet.** Sew the regions, per connected group, since a disconnected set is
  refused.

Both already work when called by hand, so an option only composes them.

## Limits still refused

- **Overlapping curves.** Two rectangles with part of a side drawn twice are refused.
  This is the first limit users are likely to hit given how loosely sketches are
  meant to be drawn. Lifting it means merging the overlap into one piece while
  cutting.
- **Pinched regions.** A hole touching the outer boundary at one point is refused,
  until a face boundary visiting a vertex twice has been shown to work.

## Open design

- **A stable identity for regions across edits.** A region has no id in the system
  and the number of regions changes with dimensions. Each region reports the curves
  bounding it, which is enough to build on. Saying "this arm" after an edit needs its
  own design, and is the hardest question here.
- **Extruding a whole sheet.** Today this goes through healing, which only works when
  one fused face is what the caller wants. `add_extruded_face` on a sewn face also
  fails with an unnamed `SewFailed` rather than a refusal saying why.

## Not started

- **The visualization playground.** `solver-2d` could show regions.
