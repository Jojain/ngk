# WS A: watertight, correctly wound tessellation

**Owns:** `src/tessellate/`, and `UnwrappedFaceDomainLoop::polyline_on` in
`src/topology/unwrapped_face_domain.rs`. **Decision:** D1, decided.
**Bench:** 43 cases whose exact measures already pass.

## Symptoms

- A solid's triangles do not close: brep-bench joins only bit-identical
  vertices, and 42 cases fail on that alone, with volume and area exact.
- Whole-turn revolved faces come out with inward triangles: a rectangle
  revolved 360° gives mesh volume −803 for a solid of +804. The B-rep is
  right (`validate_solid_orientation` passes, `Solid::volume` is positive);
  the section's winding decides which way the mesh faces.

## Root causes

1. **Slab cuts put new vertices on the face boundary.** `cut_into_slabs`
   (`strips.rs`) cuts every loop where it crosses a slab line and closes the
   pieces along the line. Where a slab line crosses a boundary edge, the cut
   inserts a vertex on that edge that the neighbouring face does not have.
   A cylinder wall between two circles shows it plainly: the rim carries 256
   points on the wall (128 edge samples plus 128 slab crossings, half a step
   apart) against 128 on the cap. Every curved bounded face is affected:
   cylinders, cones, tori, revolved and swept walls, Boolean results.
2. **Boundary points are lifted, not copied.** `polyline_on` samples the 3D
   edge, finds the nearest pcurve fraction, and the face then evaluates its
   own surface there. Two faces each lift the same edge sample through their
   own surface and agree only to rounding: a corner comes out at
   `z = −2.2e-19` on one face and `0` on the next (`boxes-common`). Welding
   inside a face (`welded`) hides this within one face and never across two
   (`Tessellation` says so: "Positions are shared within a face, never across
   two").
3. **Mesh winding is read from the unwrapped loop's signed area**
   (`ccw = signed_area(outer_uv) > 0` in `mesh_face`). For a face whose loops
   wrap a period, the outer loop in the unwrapped domain is a band closed
   across the seam, and its signed area depends on which rim the walk starts
   from, not on the side the material is on. The face's sense, not the loop's
   area, says which way it faces.

Measured on the 48 "wrong" meshes: snapping vertices to 1e-5 closes 11 of
them (rounding only, cause 2), and leaves 37 open (T-junctions, cause 1).

## The class to cover

Any two faces meeting along an edge must produce triangles whose shared
boundary is the same list of 3D points, bit for bit, in opposite orders. That
covers plane/plane, plane/curved, curved/curved, seams, poles and degenerate
edges, faces from every builder and from the Boolean, and faces with holes.

## Design (D1, decided 2026-11-04)

The robust design in one step, replacing slab cuts and ear clipping. This is
the structure OCCT's BRepMesh has (edges discretised first and their
polygons reused by every face; incremental Delaunay in parameter space,
refined to a deflection), with the two weaknesses it is known for removed:
an isotropic triangulation in (u, v), which leaves slivers in 3D where the
surface stretches its parameters, and inexact predicates.

1. **Edges first, once.** Every edge is discretised in 3D at a density that
   satisfies every face it bounds: chord sag against the edge curve, sag of
   each incident surface across it, and the angular bound. Vertices are
   points of the shape, each with one identity. A seam scaffold is
   discretised once and serves both sides of the unwrapped domain; a pole
   is one vertex; a degenerate edge contributes no segment.
2. **Constrained Delaunay triangulation per face, in its parameters**, with
   the boundary as constraints, built on exact predicates (Shewchuk's
   adaptive `orient2d` and `incircle`, or the `robust` crate), so that near
   collinear and near cocircular inputs are decided exactly and the
   triangulation cannot fail on them. Holes and wrapping loops are
   constraints like any other; a face whose loops wrap is triangulated in
   its unwrapped domain with the seam scaffold as a constraint whose two
   copies share vertices.
3. **Refinement in the surface's metric.** Delaunay refinement
   (Ruppert / Chew insertion of circumcentres) measured in the first
   fundamental form of the surface rather than in raw (u, v), so triangles
   are well shaped in 3D; stopping on chord sag to the surface, angle
   between normals, and a minimum angle. A boundary segment is never split
   by one face alone: if a face needs a finer boundary, the edge is
   refined, and both faces take the new vertices.
4. **Orientation** from the face's sense and the support's normal, never
   from a loop's signed area.
5. **Output contract.** A `Tessellation` is a closed, consistently oriented
   2-manifold whose vertices are shared across faces, with per-face-corner
   normals; or an error naming the face. Never a partial mesh. The keyed
   ranges the viewer uses stay.
6. **Exports** (STL, OBJ) write the shared vertices as they are; no consumer
   welds anything.

Whether to write the CDT and its refinement in NGK or build on a crate
(`spade` has a CDT with exact predicates and isotropic refinement; the
metric refinement would still be NGK's) is decided at the start of the
workstream, after a spike on the test family below.

## Tests (red first)

- Every primitive, every builder's output and every Boolean result in the
  test suite: the assembled triangles, joined on exact coordinates, form a
  closed, consistently oriented 2-manifold, and their signed volume is
  positive and within the sag of `Solid::volume`. This is one generic check;
  run it over every solid the tests already build.
- Specifically: cylinder, cone frustum, torus, revolved washer and sector,
  tube, drilled plate, box minus cylinder, sphere halved, stepped shaft,
  filleted box, elliptic hole, saddle drum, a swept elbow.
- Both windings of every revolved section, for 90°, 360° and −90°.

- Degenerate inputs: a face 1e-6 wide, a face with a hole 1e-6 from its
  rim, a cone apex, a sphere pole, a torus 0.01 from horn, a B-spline face
  with a collapsed row.
- Triangle quality in 3D: minimum angle and aspect ratio bounds on every
  face above, including strongly stretched parameterisations (a cylinder
  of radius 0.01 and length 1000, a NURBS patch with uneven knots).
