# Offsets: what remains on the way to shelling a solid

**Status:** support offsets (`geometry/dim3/offset.rs`) and entity offsets for
edges, profiles, faces, sheets and solids (`builders/offset/`,
`modeling::offset`) have landed; both are described in
`docs/architecture/geometry.md#offsets` and `docs/architecture/offset.md`.
Entity offsets copy the source and re-embed the copy, joining sharp edges by
intersection. What follows needs new topology, which re-embedding refuses.

## Decisions kept

1. **An offset keeps its source's parameterization**, so a face's pcurves stay
   valid on its moved surface wherever its boundary moves along the normal.
2. **A fold is refused at the geometry level, never trimmed there.**
3. **Every offset is total over its type's variants**: only geometry refuses.
4. **Offsets are copies beside their source**, so thicken and shell have both
   skins in one model.
5. **Refusing topology changes is a v1 limitation, not the design.** The
   re-embedding engine keeps the source's topology and refuses everything
   else by name, which is acceptable for a first version. The target is an
   offset that changes topology when the geometry asks for it: removing
   vanishing edges and faces, splitting a self-crossing result, inserting
   arc joins and rolling-ball faces, merging what the offset makes touch.
   New work on offsets should keep the plan/apply split so that a topology
   surgery can replace the re-embedding step without rewriting the planning.

## Next: shell

`shell_solid(model, solid, open_faces, thickness) -> SolidShelling`, inward
for a positive thickness. Direct construction, no Boolean:

1. **Inner skin**: `offset_solid` by `-thickness` on a copy — already what
   `offset_solid_edit` builds. Open faces are not moved: their copies stay on
   the source's support, so the inner skin's neighbours meet them there.
2. **Rims**: each open face's support carries one rim face, bounded outside by
   the open face's loop on the outer skin and inside by the matching loop on
   the inner skin.
3. **Sew** the outer skin minus the open faces, the rims, and the inner skin
   reversed minus its open faces. With no open face, the inner skin reversed
   is a cavity of the same solid (`BREP_WITH_VOIDS` already models one).

`plan_faces` takes a per-face distance (open faces at zero) rather than one
distance, which is also what variable-thickness shelling needs.

## Then

- **Thicken**: a sheet, its offset reversed, and ruled side walls along the
  free boundary, sewn into one solid.
- **Arc join**: at a convex edge moving outward, the rolling-ball surface
  (the fillet's section with its spine on the edge) and a sphere at a convex
  corner, instead of extending the faces to meet. Needs faces added between
  the moved ones, so it is a surgery, not a re-embedding.
- **Vanishing edges**: an edge whose moved neighbours meet past it is removed
  and its neighbours joined, instead of refusing. A profile that crosses
  itself splits into several.
