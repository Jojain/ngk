# Offsets: from curves and surfaces to shelling a solid

An offset moves a support a signed distance along its normal. It is the
geometric primitive under four modeling operations, built here in order,
each on the one before:

1. **Support offset** (geometry): a planar curve within its plane, a surface
   along its normal.
2. **Profile offset**: a planar wire moved within its plane, its edges joined
   again where the moved edges no longer meet.
3. **Face and sheet offset / thicken**: a face moved along its surface
   normal; a sheet whose faces moved apart or into each other at their shared
   edges; a sheet closed into a solid by its own offset and side walls.
4. **Shell** (hollow): a solid emptied to a wall of given thickness, open
   where the caller removes faces.

**Status:** stage 1 has landed, as `src/geometry/dim3/offset.rs`, described in
`docs/architecture/geometry.md#offsets`.

## Decisions

1. **An offset keeps its source's parameterization.** `S_d(u, v) = S(u, v) +
   d·N(u, v)` at the same `(u, v)`. A face's pcurves therefore stay valid on
   its offset surface, and moving one face never re-derives its boundary.
   Analytic offsets are built by moving or resizing the source for this
   reason; fits sample at the source's native parameters.
2. **A fold is refused at the geometry level, never trimmed there.** Which
   part of a self-intersecting offset survives is a topological decision
   that needs the entity; the geometry reports where it folds.
3. **Sign follows `Surface::normal_at` and `T × D`.** A face whose sense is
   reversed against its surface flips the distance before asking the
   geometry; the geometry knows nothing of faces.
5. **Every offset is total over its type's variants.** A curve offsets along
   `T × D` for a reference direction `D`, which is defined for planar and
   non-planar curves alike, so no variant fails for being the variant it is;
   only geometry (a collapsed radius, a fold, a tangent along `D`) refuses.
4. **Edges between offset faces are joined in one of two modes**, named by
   the caller, as every kernel does:
   - **intersection**: extend the neighbouring offsets until they meet. The
     topology mirrors the source's as long as no face vanishes.
   - **arc**: fill the gap at a convex edge with the surface a ball of radius
     `|d|` rolling on the edge sweeps — a cylinder on a line, a torus on a
     circle, a pipe in general — and a sphere at a convex corner. This is the
     fillet's rolling-ball section with its spine on the edge, and reuses
     `builders/blend/section`.

## Stage 2: profile offset

`offset_profile(model, profile, distance, join) -> ProfileOffset`.

- The profile must be planar (a property of the profile, checked here, not
  of a curve type); its plane's normal is the `D` of `Curve::offset`. A closed profile's winding decides which side is
  outward, so a positive distance grows the region it bounds.
- Each edge's support is offset; consecutive offsets are joined at the
  corner: **intersection** trims or extends both to their 2D intersection in
  the plane (the `dim2` solvers), **arc** inserts an arc centred on the
  original corner. Tangent corners need neither.
- First iteration refuses an edge that vanishes (its offset's span inverts)
  and a result that crosses itself. Removing vanished edges and splitting a
  self-crossing loop into several profiles comes after.

## Stage 3: face offset, sheet offset, thicken

- **Face**: copy the face's topology, swap its surface for the offset, keep
  the pcurves (decision 1), recompute edge curves from the pcurves on the new
  surface and vertex points from those.
- **Sheet**: classify every shared edge by the dihedral angle on the side
  the sheet moves toward: tangent (offsets meet), opening (gap: arc mode
  fills it, intersection mode extends), closing (overlap: intersect the two
  offset surfaces, analytic first). New edge curves come from the surface
  intersection, so their pcurves are re-derived (`pcurve_on_surface`) rather
  than kept.
- **Thicken**: a sheet, its offset reversed, and ruled side walls along the
  free boundary, sewn into one solid.

## Stage 4: shell

`shell_solid(model, solid, open_faces, thickness) -> SolidShelling`, inward
for a positive thickness.

Direct construction, no Boolean:

1. **Inner skin**: offset every face but the open ones inward, in
   intersection mode for convex edges and arc mode for concave ones (inward,
   a concave edge opens a gap). Faces and edges keep the source topology, so
   the inner skin is a copy of the solid's map re-embedded: each face gets its
   offset surface, each edge the intersection of its two faces' offsets, each
   vertex the point where its faces' offsets meet.
2. **Rims**: each open face's support carries one rim face, bounded outside
   by the open face's own loop and inside by where the inner skin's
   neighbours cut that support.
3. **Sew** outer skin minus open faces, rims, and inner skin reversed. With no
   open face the inner skin is a cavity of the same solid (`BREP_WITH_VOIDS`
   already models one).

First iteration: valence-3 vertices, faces that do not vanish, analytic
surface pairs the intersection table covers. Refused by name otherwise: a
face whose offset vanishes or inverts (thickness ≥ a local radius, a face
narrower than twice the thickness), a vertex of higher valence whose inner
point is over-determined.

## Open questions

- Whether the inner skin is built in a copy of the map (via the Boolean's
  `import_operand`) or directly as `PlannedFace`s through the STEP importer's
  sewing, which already assembles a solid from supports and loops.
- Variable thickness per face (`open_faces` today, a map of face to thickness
  later) and outward shelling.
