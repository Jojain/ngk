export type XY = [number, number];
export type Geometry =
  | { kind: "point"; position: XY }
  | { kind: "segment"; start: XY; end: XY }
  | { kind: "circle"; center: XY; radius: number }
  | { kind: "arc"; center: XY; radius: number; start_angle: number; sweep: number };
export type Shape = Geometry & { id: number };
export type Ref = { id: number; role: "whole" | "point" | "start" | "end" | "center" };
export type RefType = "point" | "line" | "circle" | "geometry";
export const relations = {
  coincident: { label: "Coïncidence", refs: ["point", "point"] },
  distance: { label: "Distance", refs: ["point", "point"], unit: "u" },
  length: { label: "Longueur", refs: ["line"], unit: "u" },
  radius: { label: "Rayon", refs: ["circle"], unit: "u" },
  horizontal: { label: "Horizontal", refs: ["line"] },
  vertical: { label: "Vertical", refs: ["line"] },
  parallel: { label: "Parallèle", refs: ["line", "line"] },
  perpendicular: { label: "Perpendiculaire", refs: ["line", "line"] },
  angle: { label: "Angle non orienté", refs: ["line", "line"], unit: "°" },
  oriented_angle: { label: "Angle orienté", refs: ["line", "line"], unit: "°" },
  equal_length: { label: "Égalité des longueurs", refs: ["line", "line"] },
  equal_radius: { label: "Égalité des rayons", refs: ["circle", "circle"] },
  fixed_point: { label: "Bloquer un point", refs: ["point"] },
  fixed: { label: "Bloquer une primitive", refs: ["geometry"] },
  point_on_line: { label: "Point sur droite", refs: ["point", "line"] },
  point_on_circle: { label: "Point sur cercle / arc", refs: ["point", "circle"] },
  midpoint: { label: "Milieu", refs: ["point", "line"] },
  concentric: { label: "Concentrique", refs: ["circle", "circle"] },
  tangent_line_circle: { label: "Tangence droite / cercle", refs: ["line", "circle"] },
  tangent_automatic: { label: "Tangence cercles · auto", refs: ["circle", "circle"] },
  tangent_external: { label: "Tangence cercles · externe", refs: ["circle", "circle"] },
  tangent_internal: { label: "Tangence cercles · interne", refs: ["circle", "circle"] },
  offset_x: { label: "Écart signé X", refs: ["point", "point"], unit: "u" },
  offset_y: { label: "Écart signé Y", refs: ["point", "point"], unit: "u" },
  distance_to_line: { label: "Distance à une droite", refs: ["point", "line"], unit: "u" },
  symmetric_about_line: { label: "Symétrie par une droite", refs: ["point", "point", "line"] },
  symmetric_about_point: { label: "Symétrie par un point", refs: ["point", "point", "point"] },
} satisfies Record<string, { label: string; refs: RefType[]; unit?: string }>;
export type RelationKind = keyof typeof relations;
export type Relation = { kind: RelationKind; a: Ref; b?: Ref; c?: Ref; value?: number };
export type Constraint = { id: number; relation: Relation; warning: boolean; suspect: boolean; ratio: number | null };
export type View = {
  geometry: Shape[]; constraints: Constraint[]; status: "edited" | "solved" | "error";
  message: string; dof: number | null; target_distance: number | null; iterations: number;
};
export type Command =
  | { op: "snapshot" }
  | { op: "add_geometry"; geometry: Geometry }
  | { op: "remove_geometry" | "remove_constraint"; id: number }
  | { op: "add_constraint"; relation: Relation }
  | { op: "update_constraint"; id: number; relation: Relation }
  | { op: "solve"; target?: { point: Ref; position: XY } };
export type Session = { execute(request: string): string; free(): void };
export const empty: View = { geometry: [], constraints: [], status: "edited", message: "", dof: null, target_distance: null, iterations: 0 };
export function execute(session: Session, command: Command): View {
  return JSON.parse(session.execute(JSON.stringify(command))) as View;
}
export const refKey = (r: Ref) => `${r.id}:${r.role}`;
const names = { point: "Point", segment: "Segment", circle: "Cercle", arc: "Arc" };
const roles = { whole: "", point: "", start: "début", end: "fin", center: "centre" };
export const shapeName = (shape: Shape) => `${names[shape.kind]} ${shape.id}`;
export function refName(r: Ref, shapes: Shape[]) {
  const shape = shapes.find((s) => s.id === r.id);
  return `${shape ? shapeName(shape) : r.id} ${roles[r.role]}`.trim();
}
export function points(shape: Shape): { ref: Ref; position: XY }[] {
  const point = (role: Ref["role"], position: XY) => ({ ref: { id: shape.id, role }, position });
  if (shape.kind === "point") return [point("point", shape.position)];
  if (shape.kind === "segment") return [point("start", shape.start), point("end", shape.end)];
  if (shape.kind === "circle") return [point("center", shape.center)];
  const at = (a: number): XY => [shape.center[0] + shape.radius * Math.cos(a), shape.center[1] + shape.radius * Math.sin(a)];
  return [point("center", shape.center), point("start", at(shape.start_angle)), point("end", at(shape.start_angle + shape.sweep))];
}
export function choices(shapes: Shape[], type: RefType): Ref[] {
  if (type === "point") return shapes.flatMap((s) => points(s).map((p) => p.ref));
  return shapes.filter((s) => type === "geometry" || (type === "line" ? s.kind === "segment" : s.kind === "circle" || s.kind === "arc"))
    .map((s) => ({ id: s.id, role: "whole" }));
}
export function curvePoints(shape: Shape): XY[] {
  if (shape.kind === "point") return [];
  if (shape.kind === "segment") return [shape.start, shape.end];
  const start = shape.kind === "arc" ? shape.start_angle : 0;
  const sweep = shape.kind === "arc" ? shape.sweep : Math.PI * 2;
  return Array.from({ length: 97 }, (_, i) => {
    const a = start + sweep * i / 96;
    return [shape.center[0] + shape.radius * Math.cos(a), shape.center[1] + shape.radius * Math.sin(a)];
  });
}

export type Preset = "rectangle" | "circle" | "arc" | "empty";
export function preset(session: Session, name: Preset): View {
  const add = (geometry: Geometry) => execute(session, { op: "add_geometry", geometry }).geometry.at(-1)!.id;
  const ref = (id: number, role: Ref["role"] = "whole"): Ref => ({ id, role });
  const constrain = (kind: RelationKind, a: Ref, b?: Ref, value?: number) => execute(session, { op: "add_constraint", relation: { kind, a, b, value } });
  if (name === "rectangle") {
    const p: XY[] = [[-20, -12], [20, -12], [20, 12], [-20, 12]];
    const ids = p.map((start, i) => add({ kind: "segment", start, end: p[(i + 1) % 4] }));
    ids.forEach((id, i) => {
      constrain(i % 2 ? "vertical" : "horizontal", ref(id));
      constrain("coincident", ref(id, "end"), ref(ids[(i + 1) % 4], "start"));
    });
    constrain("length", ref(ids[0]), undefined, 40);
    constrain("length", ref(ids[1]), undefined, 24);
  } else if (name === "circle") {
    const circle = add({ kind: "circle", center: [0, 0], radius: 20 });
    const p = add({ kind: "point", position: [20, 0] });
    constrain("fixed", ref(circle));
    constrain("point_on_circle", ref(p, "point"), ref(circle));
  } else if (name === "arc") {
    const arc = add({ kind: "arc", center: [0, 0], radius: 16, start_angle: 0, sweep: Math.PI });
    const a = add({ kind: "segment", start: [16, -18], end: [16, 0] });
    const b = add({ kind: "segment", start: [-16, 0], end: [-16, -18] });
    constrain("coincident", ref(a, "end"), ref(arc, "start"));
    constrain("coincident", ref(b, "start"), ref(arc, "end"));
    constrain("tangent_line_circle", ref(a), ref(arc));
    constrain("tangent_line_circle", ref(b), ref(arc));
    constrain("radius", ref(arc), undefined, 16);
    constrain("length", ref(a), undefined, 18);
    constrain("length", ref(b), undefined, 18);
  }
  return execute(session, { op: "solve" });
}
