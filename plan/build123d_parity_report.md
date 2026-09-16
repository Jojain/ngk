# build123d → ngk feature-parity report

Status: **gap analysis** (informational, not a plan commit). Last updated against
`build123d == 0.11.1` (`.venv\Lib\site-packages\build123d`) and the ngk tree as
of this writing.

## Scope and method

build123d today is a thin Python layer over OpenCascade, reached through the
`OCP` bindings. The goal is a build123d that runs on ngk. The Python side of
build123d does a lot of *its own* math (numpy/sympy/scipy point solving, sorting,
convex hull, data-structure bookkeeping); that logic is **out of scope** here and
moves over unchanged. This report only catalogs what build123d **delegates to
OCP/OCCT** — those are the kernel features ngk must supply, and each one is a
hard dependency for feature parity.

Method: every `.py` file under `build123d/` was read for `OCP.*` imports and the
actual method calls behind them. Each OCP capability is mapped below to ngk's
current state, with file evidence. A ✓ means implemented and exposed, ◑ means
present but partial or not exposed, and ✗ means absent.

Legend for ngk evidence paths is relative to the repo root (`D:\Projets\ngk`).

---

## Verdict in one paragraph

ngk is a real kernel: GMap topology, six logical entity types, exact/analytic
curve-and-surface intersections, certified solid Boolean (planar/quadric cases),
STEP import/export, chamfer, revolve, straight extrusion, and tessellation all
work. What it lacks to be build123d's backbone is concentrated in five clusters:
**(1)** the `gp`-style shape-transformation layer (move/rotate/mirror/scale on
shapes — there is none today), **(2)** the higher modelling operations that
build123d's UX is built on (fillet, offset, shell/thicken, draft, path-sweep,
loft, filling, ruled-between-edges), **(3)** measurement/analysis (mass
properties, oriented bounding box, shape→shape distance, point-in-solid exposed
per shape), **(4)** the exchange formats and auxiliary kernels build123d assumes
(STL, glTF, IGES, native BRep, hidden-line removal, fonts/text), and **(5)** a
set of geometry types build123d constructs directly (hyperbola/parabola,
Bezier surfaces, offset surfaces, trimmed/periodic handling, 2D tangent
constraint solvers).

---

## 1. Core geometry primitives (`gp`)

build123d's `Vector`, `Axis`, `Plane`, `Location`, `Matrix`, `Rotation` are
wrappers over `gp_Pnt/Vec/Dir/XYZ`, `gp_Ax1/Ax2/Ax3`, `gp_Pln/Lin`,
`gp_Trsf/GTrsf/Quaternion`.

| OCP capability | ngk status | Evidence / gap |
|---|---|---|
| Points, vectors, directions (`gp_Pnt/Vec/Dir/XYZ`, dot/cross/normalize/angle, distance, `AngleWithRef`) | ✓ | `nalgebra` `Point3`/`Vector3` in `src/geometry/dim3/utils.rs`; angle/coincidence helpers present. `AngleWithRef` (signed angle around an axis) needs a small helper, not a kernel gap. |
| Axes and frames (`gp_Ax1/Ax2/Ax3`, `gp_Pln/Lin`, `Contains`, `Distance`, parallelism/coaxiality predicates) | ◑ | `Axis<D>` (`src/geometry/axis.rs`) and `Frame` (`src/geometry/dim3/frame.rs`) exist with point/axis projection, but the predicate set (`IsParallel/IsNormal/IsCoaxial/IsOpposite`, `Angle` between axes, `gp_Pln::Contains`) is not there. `Plane` surface (`src/geometry/dim3/surfaces.rs`) has no point/line containment or `Distance` accessor. |
| **Rigid transforms on shapes** (`gp_Trsf`: move/rotate/mirror/scale; applied to `TopoDS_Shape` via `BRepBuilderAPI_Transform`, and `Shape.moved/rotated/mirrored/scaled`) | ✗ | No transform is applied to `Model` or `Shape` anywhere: `src/topology/shape.rs` exposes only `new/model/into_model/vertex/edge/face/profile/sheet/solid`; grep for `moved/rotated/mirrored/scaled/transformed` across `src/modeling` and `src/topology/shape.rs` returns nothing. This is the single most structural gap: build123d applies a `Location` to **every** shape type as a first-class operation. |
| **Affine transforms** (`gp_GTrsf`, skew/stretch, `Shape.transform_geometry`) | ✗ | Same as above; no `GTransform` equivalent. |
| Quaternion / Euler decomposition (`gp_Quaternion`, `SetEulerAngles`/`GetEulerAngles`, intrinsic/extrinsic) | ◑ | `nalgebra` quaternions exist but there is no Euler-sequence helper exposed at the ngk geometry layer; `Location` orientation decomposition used by `persistence.py` has no counterpart. |
| 2D primitives (`gp_Pnt2d/Dir2d/Ax2d/Lin2d/Circ2d`) | ◑ | `Point2`/`Vector2`/`Axis2` exist; `Circle2`/`Line2` exist (`src/geometry/dim2/`). No `gp_Lin2d`/`gp_Circ2d` value types per se — the constrained-solver would need its own 2D value math, which is thin. |

---

## 2. Curves and surfaces (`Geom`, `Geom2d`)

| OCP capability | ngk status | Evidence / gap |
|---|---|---|
| Analytic 3D curves: line, circle, ellipse | ✓ | `Curve::{Line, Circle, Ellipse}` (`src/geometry/dim3/curves.rs`). |
| Analytic 3D curves: **hyperbola, parabola** (`gp_Hypr/gp_Parab`, `Edge.make_hyperbola/make_parabola`) | ✗ | No hyperbola/parabola variant in `Curve`; only representable as generic NURBS. build123d exposes these as first-class edge constructors. |
| Bezier curves (`Geom_BezierCurve`) | ◑ | Bezier exists as a NURBS specialization (`src/geometry/dim3/nurbs/bezier.rs`, `src/geometry/dim2/bezier.rs`), but there is no dedicated `BezierCurve` curve variant with rational-weighted constructor matching `Edge.make_bezier`. |
| B-spline curves (`Geom_BSplineCurve`: poles/knots/mults/weights, periodic, `Reversed`, `Transform`) | ✓ | `NurbsCurve` (`src/geometry/dim3/nurbs/curve.rs`), `NurbsCurve2`. |
| Elementary surfaces: plane, cylinder, sphere, cone, torus, revolution | ✓ | `Surface::{Plane, Cylinder, Sphere, Cone, Torus, Revolution}` (`src/geometry/dim3/surfaces.rs`). |
| Bezier surfaces (`Geom_BezierSurface`, `Face.make_bezier_surface`) | ◑ | Bezier surface machinery exists under `src/geometry/dim3/nurbs/bezier.rs` but no public `BezierSurface` type / `make_bezier_surface` path. |
| Offset surface (`Geom_OffsetSurface`, used to unwrap rotation axis) | ✗ | No offset-surface type. |
| Trimmed/rectangular-trimmed surfaces (`Geom_RectangularTrimmedSurface` basis-unwrap) | ◑ | ngk has no trimmed-surface concept; faces always carry their full support. The axis/curvature queries build123d does by unwrapping a trimmed surface must instead read the face's own `Surface`. |
| **Ruled surface** (for `BRepFill` and prism side walls) | ◑ | `Surface::Ruled` exists, but it is **not exported** to STEP (`write_surface` in `src/exchange/step/convert/surfaces.rs:54` errors `UnsupportedSurface` for `Ruled`). |
| 2D curves / pcurves (`Geom2d_Line/Circle/TrimmedCurve`, `Period`) | ✓ | `Curve2::{Line2, Circle2, Ellipse2, Nurbs}` + `TrimmedCurve2` (`src/geometry/dim2/`); every pcurve is a `TrimmedCurve2`. |
| Evaluation `D0/D1/D2` on curves and surfaces | ✓ | `CurveGeometry`/`SurfaceGeometry` traits (`src/geometry/traits.rs`) with `point_at`, tangents, etc. |

---

## 3. Adaptors and evaluation

build123d routes **all** position/tangent/radius/arc-center/normal queries
through `BRepAdaptor_Curve/Surface` (`geom_adaptor`), which also do type
introspection (`GetType`) and return the underlying analytic curve/surface.

| OCP capability | ngk status | Evidence / gap |
|---|---|---|
| Curve/surface adaptor with type introspection and analytic downcast (`Circle()/Ellipse()/BSpline()/…`) | ◑ | ngk keeps the analytic form directly on the enum, so "adaptor" is unnecessary for its own types. But the *edge* needs a `geom_adaptor`-equivalent: ngk's `Edge::trimmed_curve()` derives span from corners (`src/topology/edge.rs`), which is the intended substitute. Missing is the ability to recover the *support type* (line/circle/ellipse/spline) of an arbitrary imported edge for build123d's radius/arc-center/normal introspection — ngk can derive this from `Curve`, so this is a binding-layer job, not a kernel gap. |
| Composite curve adaptor (`BRepAdaptor_CompCurve`) for a whole wire | ✗ | No wire-level "as a single curve" evaluation; a `Wire.combine`/`_to_bspline` equivalent (`GeomConvert_CompCurveToBSplineCurve`) is absent. |
| Arc-length → parameter (`GCPnts_AbscissaPoint`) | ◑ | ngk has `Curve::param_at` / `project` but no abscissa-at-length primitive exposed; needed by `trim_to_length` and spline sorting. |
| Uniform/quasi-uniform deflection discretization (`GCPnts_UniformDeflection`) | ◑ | Tessellation exists (`src/tessellate/curve.rs`) but is not deflection-parameterized the way `positions(deflection=…)` expects. |
| Curve construction helpers (`GC_MakeArcOf*`, `gce_MakeLin`) | ✓ | Arc/tangent-arc/three-point-arc exist via `Edge::arc` etc. (`src/modeling/edges.rs`); line-from-axis via `gce_MakeLin` has a `Line`/`Axis` counterpart. |
| Spline interpolation/approximation (`GeomAPI_Interpolate`, `GeomAPI_PointsToBSpline`, `GeomAPI_PointsToBSplineSurface`) | ◑ | `NurbsCurve` interpolation exists (`src/geometry/dim3/nurbs/`); surface-from-point-grid (`Face.make_surface_from_array_of_points`) is absent. |

---

## 4. Intersection and projection

| OCP capability | ngk status | Evidence / gap |
|---|---|---|
| curve/curve (3D) | ✓ | `intersect_curves`; analytic `line×line`, `line×circle`, `circle×circle`, NURBS fallback (`src/geometry/dim3/intersections/`). |
| curve/surface (`GeomAPI_IntCS`) | ✓ | `intersect_curve_surface`; analytic `line×{plane,sphere,cylinder,cone}`, `circle×{plane,sphere}`, NURBS fallback. |
| surface/surface (`GeomAPI_IntSS`) | ✓/◑ | `intersect_surfaces`; analytic `plane×plane/sphere/cylinder`, `sphere×sphere`; NURBS tracer for the rest. Gap: any cone pair and `cylinder×cylinder` go through the (still in-progress) tracer, and this is exactly the pairing that blocks cylinder×cylinder Boolean. |
| 2D curve/curve (`Geom2dAPI_InterCurveCurve`) | ✓ | `CurveCurveIntersection2` (`src/geometry/dim2/intersections.rs`). |
| Point projection onto curve (`GeomAPI_ProjectPointOnCurve`) | ✓ | `Curve::project` / `param_at` (`src/geometry/dim3/curves.rs`). |
| Point projection onto surface (`GeomAPI_ProjectPointOnSurf`) | ✓ | `Surface::closest_parameter` / `param_at` (`src/geometry/dim3/surfaces.rs`). |
| curve↔curve extrema for wire closure (`GeomAPI_ExtremaCurveCurve`) | ✗ | No curve/curve extrema (only curve/point). |
| 2D tangent constraint solvers (`Geom2dGcc_Circ2d*`, `Geom2dGcc_Lin2d*`, `GccEnt` qualifiers, `IntAna2d_AnaIntersection`) | ✗ | This backs build123d's `make_constrained_arcs`/`make_constrained_lines` (`constrained_lines.py`). It is pure 2D algebra and could be implemented on ngk's `Curve2`, but nothing exists today. |
| Project 3D curve to 2D plane (`GeomAPI.To2d`) | ✗ | No curve-to-plane projection helper exposed. |
| Project curve onto surface (`GeomProjLib.Project`) | ✗ | Used by `Edge.project_to_shape`/`_make_edges` wrapping; ngk has no curve-on-surface projection operator. |
| Moving-frame laws along a curve (`GeomFill_Frenet`, `GeomFill_CorrectedFrenet`) | ✗ | `location_at` FRENET frames; no counterpart. |

---

## 5. Topology construction and traversal

| OCP capability | ngk status | Evidence / gap |
|---|---|---|
| Vertex / edge / wire / face / shell / solid / compound construction | ✓ | `src/builders/{vertices,edges,profiles,faces,sheets,solids}.rs`, `src/modeling/*`. Compound/named assembly is the gap (see §14). |
| Sewing (`BRepBuilderAPI_Sewing`) | ✓ | Sewing via `sew` on `ModelEdit` (`src/topology/edit.rs`) and face stitching in `builders/faces.rs`. |
| Face with holes (`MakeFace` + wire list) | ✓ | `polygon_with_holes`, `add_face` with bridge edges (`src/builders/faces.rs`). |
| Explorer / sub-shape enumeration (`TopExp_Explorer`, `TopoDS_Iterator`) | ✓ | `Face::outer_loop`, `Solid::faces/edges/vertices`, `Profile::edges`, `Model::cells(Dim)`. |
| Ancestor/parent maps (`TopExp::MapShapesAndAncestors`) | ✓ | `src/topology/embedding/` + `turn` walk and the dart→owner index. |
| Orientation and reversal (`TopoDS_Shape::Reversed/Complemented/Orientation`) | ✓ | `Orientation::{Same, Reversed}` (`src/topology/orientation.rs`). |
| Shape identity / hashing / `IsSame`/`IsEqual` | ◑ | Keys (`*Key`) are durable identity; no exposed `is_same`/`is_equal` semantics beyond key equality. |
| Transform of a shape's location (`TopLoc_Location`) | ✗ | See §1 — no location transform on shapes. |

---

## 6. Boolean operations and feature splitting

| OCP capability | ngk status | Evidence / gap |
|---|---|---|
| Fuse / Cut / Common (solids) | ✓ | `fuse/cut/intersect` (`src/modeling/solids.rs`, `src/builders/boolean/`). Certified for planar/quadric; `cylinder×cylinder` still fails (`AmbiguousClassification`). |
| **Boolean on faces/edges/wires** (`Shape.intersect` for face sets, `Face.split`) | ✗ | `boolean` refuses non-solid inputs by design (`plan/boolean_evaluation.md`). build123d calls `BRepAlgoAPI_Common`/`Section`/`Splitter` on faces and edges too. |
| Section (`BRepAlgoAPI_Section`, pcurve-on-1/2) | ◑ | Surface/surface intersection produces section curves, but no `BRepAlgoAPI_Section` face-set/edge operation with pcurves on both operands is exposed. |
| Splitter (`BRepAlgoAPI_Splitter`, `BOPAlgo_Splitter`) | ◑ | Face splitting by imprints exists (`split_face_by_imprints`); no general shape split by a tool. |
| Glue / fuzzy / parallel flags | ◑ | No glue/fuzzy-value/parallel options. |
| Feature split (`BRepFeat_SplitShape`, `split_by_perimeter`) | ✗ | No equivalent; build123d uses it for `Face.split_by_perimeter`. |
| `BRepAlgo::ConvertFace` (face → planar arcs) | ✗ | `Face.to_arcs` has no counterpart. |

---

## 7. Modelling operations

| OCP capability | ngk status | Evidence / gap |
|---|---|---|
| Chamfer (2D + 3D) | ✓ | `src/builders/chamfer.rs` (edges/profile/vertices; straight + extruded-NURBS; 2D profile corners). More constrained than OCCT's, but present. |
| **Fillet** (`BRepFilletAPI_MakeFillet`, 3D; `BRepFilletAPI_MakeFillet2d`, 2D) | ✗ | No fillet anywhere; only `chamfer.rs`. This is the biggest single modelling gap given fillet is the most-used build123d feature after extrude/boolean. |
| **2D offset** (`BRepOffsetAPI_MakeOffset`, join modes) | ✗ | No wire/edge offset. |
| **3D offset / shell / thicken** (`BRepOffset_MakeOffset`, `MakeThickSolid`) | ✗ | No hollow/offset/thicken. (ngk's "shell" is a boundary component, unrelated.) |
| **Draft angle** (`BRepOffsetAPI_DraftAngle`) | ✗ | No draft. |
| **Sweep along path / pipe** (`BRepOffsetAPI_MakePipeShell`, Frenet/aux-spine, transition modes) | ✗ | Only straight extrusion (`src/modeling/sweep.rs`); no `sweep_multi`, no `extrude_linear_with_rotation`. |
| **Loft / skin / ThruSections** | ✗ | No loft. |
| **Filling (N-sided)** (`BRepOffsetAPI_MakeFilling`) | ✗ | No `Face.make_surface`/`make_surface_patch`. |
| **Ruled between two wires** (`BRepFill`) | ✗ | No `Face.make_surface_from_curves`; though `Surface::Ruled` exists as a type. |
| Prism / extrusion (`BRepPrimAPI_MakePrism`) | ✓ | `extrude_profile`/`extrude_face` (`src/modeling/sweep.rs`). |
| Revolve (`BRepPrimAPI_MakeRevol`) | ✓ | `revolve_profile`/`revolve_face` (`src/modeling/revolve.rs`). |
| Half-space | ◑ | Half-space solids for split classification (`src/builders/boolean/classify.rs`) but not as a user primitive. |
| **Projection** (`BRepProj_Projection`, dir or center) | ✗ | No `Edge/Wire.project` / `project_to_shape`. |
| **Prismatic feature to face/depth/thru-all** (`BRepFeat_MakeDPrism`, `LocOpe_DPrism`) | ✗ | `Solid.dprism`/`extrude_taper` absent. |
| Primitives: box, cylinder, sphere, torus | ✓ | `block/cylinder/sphere/torus` (`src/modeling/solids.rs`). |
| Primitives: **cone, wedge** | ✗ | No `cone(...)`/wedge builder (a `Cone` *surface* exists, and revolve can produce a cone, but no primitive). |

---

## 8. Analysis and measurement

| OCP capability | ngk status | Evidence / gap |
|---|---|---|
| Bounding box (`Bnd_Box`, `BRepBndLib::AddOptimal/Add`) | ◑ | `BBox` and per-curve/surface `bbox_over` exist, but there is **no `Model`/`Solid`/`Face`-level bbox** exposed (`Model::bbox()` absent). |
| Oriented bounding box (`Bnd_OBB`) | ✗ | No OBB. |
| **Mass properties** (`GProp_GProps` + `BRepGProp::{Linear,Surface,Volume}Properties`: length/area/volume/centroid/inertia/static-moments/principal-axes/gyration) | ✗ | No public mass API; only `pub(crate)` signed-volume helpers in `src/topology/face.rs` for validation. |
| Face point+normal (`BRepGProp_Face::Normal`, `normal_at`/`position_at`) | ✓ | `Surface` evaluation gives point+normal. |
| Shape→shape min distance (`BRepExtrema_DistShapeShape`, closest points, `SetDeflection`) | ✗ | No distance-between-shapes; only point projection. build123d uses it for `distance`/`closest_points`/touch detection. |
| Point→curve extrema (`Extrema_ExtPC`) | ✓ | `Curve::project`. |
| Point-in-solid classification (`BRepClass3d_SolidClassifier`, `State`, `IsOnAFace`) | ◑ | `solid_contains_point` (`src/builders/boolean/classify.rs`) exists but is ray-cast based and geometry-restricted; boundary returns `Ambiguous` rather than `ON`. |
| Line-through-face intersection (`BRepIntCurveSurface_Inter`) | ◑ | Covered by curve/surface intersection but not as a shape-level `faces_intersected_by_axis`. |
| Edge planarity / seam detection (`ShapeAnalysis_Curve`, `ShapeAnalysis_Edge::IsSeam`) | ◑ | Planarity via `Face::is_planar`; seam synthesis exists on the STEP path but no general `is_seam` query. |
| Continuity between edges (`BRepLProp::Continuity`) | ✗ | No G0/G1/G2 continuity measure exposed. |

---

## 9. Fixing, healing and validation

| OCP capability | ngk status | Evidence / gap |
|---|---|---|
| Shape fixing (`ShapeFix_Shape::Perform`) | ◑ | ngk has a healing layer (`src/healing/`) removing redundant topology, but no general "fix orientation/geometry" pass equivalent to `Shape.fix`. |
| Face orientation fix (`ShapeFix_Face`) | ◑ | Orientation is enforced by commit invariants (`src/topology/validation.rs`), a different (stronger) mechanism. |
| Wire fixing (`ShapeFix_Wire` reorder/connect) | ◑ | Profile construction handles connectivity at build time. |
| Wireframe small-edge/gap repair (`ShapeFix_Wireframe`) | ✗ | No `fix_degenerate_edges`. |
| Shell→solid (`ShapeFix_Solid::SolidFromShell`) | ◑ | `add_solid` from shell exists (`src/builders/solids.rs`). |
| **Unify same domain / merge coplanar faces** (`ShapeUpgrade_UnifySameDomain`, `Shape.clean`) | ✗ | No face-merging/cleanup operator. |
| Force geometry to splines (`ShapeCustom::BSplineRestriction`, `Shape.to_splines`) | ◑ | NURBS conversion helpers exist but no whole-shape "to splines" pass. |
| Validity check (`BRepCheck_Analyzer::IsValid`) | ◑ | `validation` checks commit invariants, not OCCT-style geometric validity. |
| Free-bounds connect (`ShapeAnalysis_FreeBounds::ConnectEdgesToWires`) | ✗ | No `Wire.combine` / edges-to-wires. |
| Rebuild 3D curves (`BRepLib::BuildCurves3d`) | ✗ | No equivalent (only relevant to HLR-projected edges). |
| Find surface for planar wire (`BRepLib_FindSurface`) | ◑ | Planar face detection exists but not exposed as a query. |
| Hole removal (`BRepTools_ReShape`, `Face.without_holes`) | ✗ | No `without_holes`. |

---

## 10. Tessellation / meshing

| OCP capability | ngk status | Evidence / gap |
|---|---|---|
| Mesh generation (`BRepMesh_IncrementalMesh` + triangulation) | ◑ | `src/tessellate/` produces indexed triangle meshes, but the documented gap is a real constrained-Delaunay triangulation (`TessellateError::UntriangulableBoundary`). |
| Triangulation extraction (`BRep_Tool::Triangulation`, `Poly_Triangulation` nodes/triangles) | ✓ | `IndexedMesh { positions, normals, indices }`. |
| Native BRep mesh presence/cleanup (`BRepTools::Clean`) | ✗ | N/A (no native BRep). |

---

## 11. Hidden-line removal / drafting

| OCP capability | ngk status | Evidence / gap |
|---|---|---|
| HLR (`HLRBRep_Algo`, `HLRAlgo_Projector`, `HLRBRep_HLRToShape` visible/hidden compounds) | ✗ | No HLR. Required by build123d's `Drawing`, `project_to_viewport`, and SVG/2D export. This is a self-contained but substantial subsystem. |

---

## 12. Import / export

| OCP capability | ngk status | Evidence / gap |
|---|---|---|
| STEP (CAF/XDE reader+writer, names/colors/layers/assemblies) | ◑ | STEP import/export works for single solids with a defined entity set (`src/exchange/step/`), but **assembly structure (names/nesting/placements/transforms) and color/name metadata are dropped**. `Ruled` surfaces are not exported. |
| STL (read `RWStl`, write `StlAPI_Writer`) | ✗ | Not implemented (`src/exchange/mod.rs` names it as planned). |
| glTF (`RWGltf_CafWriter`) | ✗ | Not implemented. |
| IGES (`IGESControl_Controller`) | ✗ | Not implemented. |
| Native BRep (`BRepTools::Write/Read`, `BinTools` for persistence) | ✗ | ngk has no native format yet; build123d uses native BRep for `.brep` export and BinTools for pickling. |
| 3MF (build123d `export_3mf` via OCP/`lib3mf`) | ✗ | Not implemented. |
| SVG / DXF export | ◑ | These rely on HLR (§11) + `BRepTools_WireExplorer` + curve evaluation; the actual file writing is pure-Python (`svgpathtools`/`ezdxf`) and out of scope, but the HLR + wire-explorer kernel pieces are missing. |
| SVG import (`ocpsvg`) and Gordon interpolation (`ocp_gordon`) | ✗ | These are **external OCP-backed packages** build123d imports directly; ngk would need equivalent functionality (SVG/curve-network-to-surface). |

---

## 13. Text and fonts

| OCP capability | ngk status | Evidence / gap |
|---|---|---|
| Font registry (`Font_FontMgr`) and glyph geometry (`StdPrs_BRepFont`/`BRepTextBuilder`) | ✗ | No font/glyph/text-to-geometry. build123d's `text.py`/`composite.py` depend on OCCT's font subsystem. |

---

## 14. Color, metadata and assembly

| OCP capability | ngk status | Evidence / gap |
|---|---|---|
| Color (`Quantity_Color/RGBA`, sRGB) | ◑ | ngk has no color type in the model; color lives only in `VizHints` (`src/viz/hints.rs`). build123d's `Color`/`Shape.color` round-trip through XDE. |
| Per-shape payload/metadata | ✓ (type-level) | `Payload` trait (`src/topology/payload.rs`) + `*Attr.data`; `StandardPayload` is `()` so there is no built-in color/material field. |
| Assembly / compound with named sub-shapes and placements (`XCAFDoc_ShapeTool`, `TDataStd_Name`, `TDF_Label`) | ✗ | No assembly concept; ngk models one map, no named compound hierarchy or per-part location. |

---

## 15. Errors and status

| OCP capability | ngk status | Evidence / gap |
|---|---|---|
| Exception hierarchy (`Standard_ConstructionError/Failure/TypeMismatch`, `StdFail_NotDone`) | ✓ | ngk uses `thiserror` result types throughout (`src/builders/errors.rs`, `src/modeling/errors.rs`, `src/exchange/step/error.rs`), which is a cleaner equivalent; the binding layer maps them to Python exceptions. |

---

## Prioritised gap list (what to build, in rough order of build123d importance)

1. **Shape transforms** (`Location` on every shape: move/rotate/mirror/scale, affine) — structural, needed by nearly everything else.
2. **Fillet** (2D + 3D) — highest-value modelling feature build123d users rely on.
3. **2D offset + 3D offset/thicken/shell**.
4. **Mass properties** (length/area/volume/centroid) + shape-level **bbox** — needed for measurement UX and for the build123d test oracle.
5. **Loft + path-sweep (pipe)** — core build123d `loft`/`sweep` builders.
6. **Draft angle**, **prism-to-face/depth (dprism)**, **projection**.
7. **STL + glTF + native BRep + IGES** import/export (STL and glTF are the most-used exporters after STEP).
8. **Boolean on faces/edges + section + splitter** (currently solid-only).
9. **Hidden-line removal** (unblocks `Drawing`, SVG/DXF export, viewport projection).
10. **Assembly / named compounds + color metadata** (unblocks STEP round-trip fidelity and multi-part workflows).
11. **Cone/wedge primitives, hyperbola/parabola, Bezier surfaces, offset surfaces**.
12. **Text/font**, **filling (N-sided)**, **ruled-between-wires**, **2D tangent constraint solvers**, **curve/curve extrema**, **unify-same-domain clean**.
13. **Constrained-Delaunay tessellation** (completes meshing/STL/glTF).
