import type {
  Circle,
  Cylinder,
  Edge,
  Face,
  Line,
  Model,
  NurbsCurve,
  NurbsSurface,
  Plane,
  Point3,
  Profile,
  RuledSurface,
  Sheet,
  Solid,
  SurfaceOfRevolution,
  Vertex,
  Vector3,
} from "../wasm/ngk";
import type { Kernel } from "./useKernel";
import type { VizScene } from "./viz";

export type DebugViewerEnvelope = {
  receivedAt: string;
  sequence: number;
  payload: DebugViewerPayload;
};

export type DebugViewerPayload = {
  kind: "ngk.debug.v4";
  name: string;
  nodes: DebugNodePayload[];
};

/**
 * One entry of the transported object tree. A leaf carries the value it
 * transports; a group carries children instead, and the viewer shows or hides
 * a whole group at once.
 */
export type DebugNodePayload = {
  name: string;
  object?: SerializedDebugObject;
  children?: DebugNodePayload[];
};

export type DebugObjectKind =
  | "model"
  | "vertex"
  | "edge"
  | "profile"
  | "face"
  | "sheet"
  | "solid"
  | "point"
  | "vector"
  | "plane"
  | "curve"
  | "surface";

export type SerializedDebugObject = {
  kind: DebugObjectKind;
  primaryDart?: number;
  serialized: string;
};

export type DebugGeometry =
  | Point3
  | Vector3
  | Plane
  | Line
  | Circle
  | NurbsCurve
  | Cylinder
  | DebugSphere
  | DebugCone
  | RuledSurface
  | SurfaceOfRevolution
  | NurbsSurface;

/** Structural view of the analytical sphere returned by the WASM binding. */
export type DebugSphere = {
  readonly origin: Point3;
  readonly xDir: Vector3;
  readonly axis: Vector3;
  readonly radius: number;
  pointAt: (u: number, v: number) => Point3;
  normalAt: (u: number, v: number) => Vector3;
};
/** Structural view of the analytical cone returned by the WASM binding. */
export type DebugCone = {
  readonly origin: Point3;
  readonly xDir: Vector3;
  readonly axis: Vector3;
  readonly referenceRadius: number;
  readonly halfAngle: number;
  readonly apexParameter?: number;
  pointAt: (u: number, v: number) => Point3;
  normalAt: (u: number, v: number) => Vector3;
};
export type DebugObject =
  | Model
  | Vertex
  | Edge
  | Profile
  | Face
  | Sheet
  | Solid
  | DebugGeometry;
export type DebugTopologyEntity = Vertex | Edge | Face;
export type DebugTopologyKind = "vertex" | "edge" | "face";
export type DebugTopologySelection = {
  kind: DebugTopologyKind;
  id: number;
};

export type DebugPcurveCurve = {
  kind: string;
  sample: (segments: number) => Float64Array;
  pointAt: (parameter: number) => Float64Array;
  degree?: number;
  domain?: Float64Array;
  radius?: number;
  sweep?: number;
  center?: Float64Array;
  start?: Float64Array;
  end?: Float64Array;
  weights?: Float64Array;
  controlPoints?: unknown;
};

export type DebugFacePcurve = {
  loopIndex: number;
  dartId: number;
  edgeKey: string;
  startVertexKey: string;
  endVertexKey: string;
  curve: DebugPcurveCurve;
};

export type HydratedObject = {
  kind: DebugObjectKind;
  value: DebugObject;
  model?: Model;
};

/**
 * A transported node with its value restored and its share of the render
 * scene resolved. A leaf owns a scene whose ids are already offset into the
 * dump's global numbering; a group owns children and no scene of its own, so
 * hiding it hides everything below it.
 */
export type HydratedDebugNode = {
  id: string;
  name: string;
  kind: DebugObjectKind | null;
  value: DebugObject | null;
  model: Model | null;
  scene: VizScene | null;
  children: HydratedDebugNode[];
};

export type DebugEntityEntry<T> = {
  id: number;
  value: T;
  model: Model;
};

export type DebugSelectionIndex = {
  vertices: DebugEntityEntry<Vertex>[];
  edges: DebugEntityEntry<Edge>[];
  faces: DebugEntityEntry<Face>[];
  darts: Array<{ id: number; dart: number; model: Model }>;
};

export type HydratedDebugDump = {
  name: string;
  nodes: HydratedDebugNode[];
  object: DebugObject | undefined;
  objects: DebugObject[];
  shape: DebugObject | undefined;
  shapes: DebugObject[];
  model: Model | undefined;
  models: Model[];
  scene: VizScene;
  selection: DebugSelectionIndex;
};

/** Returns the render selection corresponding to a real topology handle. */
export function debugSelectionForTopology(
  dump: HydratedDebugDump,
  entity: DebugTopologyEntity,
): DebugTopologySelection | null {
  const kind = debugTopologyKind(entity);
  const entry =
    kind === "vertex"
      ? dump.selection.vertices.find(({ value }) => sameTopology(value, entity))
      : kind === "edge"
        ? dump.selection.edges.find(({ value }) => sameTopology(value, entity))
        : kind === "face"
          ? dump.selection.faces.find(({ value }) => sameTopology(value, entity))
          : undefined;
  return kind && entry ? { kind, id: entry.id } : null;
}

/** Returns the real topology handle represented by a render selection. */
export function debugTopologyForSelection(
  dump: HydratedDebugDump,
  selection: { kind: string; id: number } | null,
): DebugTopologyEntity | null {
  if (selection?.kind === "vertex") {
    return (
      dump.selection.vertices.find(({ id }) => id === selection.id)?.value ?? null
    );
  }
  if (selection?.kind === "edge") {
    return dump.selection.edges.find(({ id }) => id === selection.id)?.value ?? null;
  }
  if (selection?.kind === "face") {
    return dump.selection.faces.find(({ id }) => id === selection.id)?.value ?? null;
  }
  return null;
}

export function debugTopologyKind(value: unknown): DebugTopologyKind | null {
  if (typeof value !== "object" || value === null) return null;
  const candidate = value as {
    constructor?: { name?: string };
    key?: unknown;
    equals?: unknown;
  };
  if (typeof candidate.key !== "string" || typeof candidate.equals !== "function") {
    return null;
  }
  if (candidate.constructor?.name === "Vertex") return "vertex";
  if (candidate.constructor?.name === "Edge") return "edge";
  if (candidate.constructor?.name === "Face") return "face";
  return null;
}

/** Compares stable keys and owning maps through the typed WASM handle API. */
export function sameTopology(
  left: DebugTopologyEntity,
  right: DebugTopologyEntity,
): boolean {
  const kind = debugTopologyKind(left);
  if (kind !== debugTopologyKind(right) || left.key !== right.key) return false;
  if (kind === "vertex") return (left as Vertex).equals(right as Vertex);
  if (kind === "edge") return (left as Edge).equals(right as Edge);
  return kind === "face" && (left as Face).equals(right as Face);
}

const ENDPOINT = "/__ngk_debug/dumps";

export async function fetchDebugDumps(): Promise<DebugViewerEnvelope[]> {
  const response = await fetch(ENDPOINT);
  if (!response.ok) throw new Error(`debug object fetch failed: ${response.status}`);
  return (await response.json()) as DebugViewerEnvelope[];
}

export async function clearDebugDumps(): Promise<void> {
  const response = await fetch(ENDPOINT, { method: "DELETE" });
  if (!response.ok) throw new Error(`debug object clear failed: ${response.status}`);
}

/** Restores transported values as real WASM topology and geometry objects. */
export function hydrateDebugDump(
  payload: DebugViewerPayload,
  kernel: Kernel,
): HydratedDebugDump {
  if (payload.kind !== "ngk.debug.v4") {
    throw new Error(`unsupported debug object payload: ${String(payload.kind)}`);
  }

  const selection: DebugSelectionIndex = {
    vertices: [],
    edges: [],
    faces: [],
    darts: [],
  };
  const hydrated: HydratedObject[] = [];
  const bases: SceneOffsets = {
    vertexBase: 0,
    edgeBase: 0,
    faceBase: 0,
    dartBase: 0,
  };
  const nodes = payload.nodes.map((node, index) =>
    hydrateNode(node, String(index), kernel, selection, bases, hydrated),
  );

  const objects = hydrated.map(({ value }) => value);
  const models = hydrated.flatMap(({ model }) => (model ? [model] : []));
  return {
    name: payload.name,
    nodes,
    object: objects[0],
    objects,
    shape: objects[0],
    shapes: objects,
    model: models[0],
    models,
    scene: debugScene(nodes, new Set()),
    selection,
  };
}

/**
 * The scene of every leaf whose node and ancestors are all shown.
 *
 * Leaf scenes already carry the dump's global ids, so a visible subset is a
 * concatenation and selections keep meaning the same entity as the tree is
 * toggled.
 */
export function debugScene(
  nodes: readonly HydratedDebugNode[],
  hidden: ReadonlySet<string>,
): VizScene {
  const scene = emptyScene();
  for (const node of nodes) appendVisibleNode(node, hidden, scene);
  return scene;
}

/** Every leaf below a node, itself included when it is one. */
export function debugNodeLeaves(node: HydratedDebugNode): HydratedDebugNode[] {
  if (node.scene) return [node];
  return node.children.flatMap(debugNodeLeaves);
}

function appendVisibleNode(
  node: HydratedDebugNode,
  hidden: ReadonlySet<string>,
  target: VizScene,
) {
  if (hidden.has(node.id)) return;
  if (node.scene) appendScene(target, node.scene);
  for (const child of node.children) appendVisibleNode(child, hidden, target);
}

function hydrateNode(
  node: DebugNodePayload,
  id: string,
  kernel: Kernel,
  selection: DebugSelectionIndex,
  bases: SceneOffsets,
  hydrated: HydratedObject[],
): HydratedDebugNode {
  if (node.object) {
    const leaf = hydrateObject(node.object, kernel, selection, bases);
    hydrated.push({
      kind: leaf.kind,
      value: leaf.value,
      model: leaf.model ?? undefined,
    });
    return { id, name: node.name, children: [], ...leaf };
  }
  return {
    id,
    name: node.name,
    kind: null,
    value: null,
    model: null,
    scene: null,
    children: (node.children ?? []).map((child, index) =>
      hydrateNode(child, `${id}/${index}`, kernel, selection, bases, hydrated),
    ),
  };
}

/**
 * Restores one transported value and advances `bases`, so each leaf's scene
 * and selection entries occupy their own range of the dump's id space.
 */
function hydrateObject(
  serialized: SerializedDebugObject,
  kernel: Kernel,
  selection: DebugSelectionIndex,
  bases: SceneOffsets,
): {
  kind: DebugObjectKind;
  value: DebugObject;
  model: Model | null;
  scene: VizScene;
} {
  const { vertexBase, edgeBase, faceBase, dartBase } = bases;
  let localScene: VizScene;
  let value: DebugObject;
  let model: Model | null = null;

  if (isTopologyKind(serialized.kind)) {
    model = kernel.Model.deserialize(serialized.serialized);
    const vertices = model.vertices();
    const edges = model.edges();
    const faces = model.faces();
    localScene = kernel.sceneFromGMap(model) as VizScene;

    selection.vertices.push(
      ...vertices.map((entry, id) => ({ id: vertexBase + id, value: entry, model: model! })),
    );
    selection.edges.push(
      ...edges.map((entry, id) => ({ id: edgeBase + id, value: entry, model: model! })),
    );
    selection.faces.push(
      ...faces.map((entry, id) => ({ id: faceBase + id, value: entry, model: model! })),
    );
    selection.darts.push(
      ...Array.from(model.darts(), (dart) => ({
        id: dartBase + dart,
        dart,
        model: model!,
      })),
    );

    value = resolvePrimaryTopology(model, serialized);
  } else {
    const geometry = kernel.hydrateDebugGeometry(
      serialized.kind,
      serialized.serialized,
    ) as { value: DebugGeometry; scene: VizScene };
    localScene = geometry.scene;
    value = geometry.value;
  }

  const scene = offsetScene(localScene, bases);
  bases.vertexBase += sceneIdSpan(localScene.vertices, "vertexId");
  bases.edgeBase += sceneIdSpan(localScene.edges, "edgeId");
  bases.faceBase += sceneIdSpan(localScene.faces, "faceId");
  bases.dartBase += sceneIdSpan(localScene.darts, "dartId");
  return { kind: serialized.kind, value, model, scene };
}

function resolvePrimaryTopology(
  model: Model,
  serialized: SerializedDebugObject,
): Model | Vertex | Edge | Profile | Face | Sheet | Solid {
  if (serialized.kind === "model") return model;
  const dart = serialized.primaryDart;
  if (dart === undefined) {
    throw new Error(`${serialized.kind} debug object has no primary dart`);
  }

  const value =
    serialized.kind === "vertex"
      ? model.vertex(dart)
      : serialized.kind === "edge"
        ? model.edge(dart)
        : serialized.kind === "profile"
          ? model.profile(dart)
          : serialized.kind === "face"
            ? model.face(dart)
            : serialized.kind === "sheet"
              ? model.sheet(dart)
              : model.solid(dart);
  if (!value) throw new Error(`could not restore ${serialized.kind} at dart ${dart}`);
  return value;
}

function isTopologyKind(
  kind: DebugObjectKind,
): kind is "model" | "vertex" | "edge" | "profile" | "face" | "sheet" | "solid" {
  return (
    kind === "model" ||
    kind === "vertex" ||
    kind === "edge" ||
    kind === "profile" ||
    kind === "face" ||
    kind === "sheet" ||
    kind === "solid"
  );
}

type SceneOffsets = {
  vertexBase: number;
  edgeBase: number;
  faceBase: number;
  dartBase: number;
};

/** Renumbers a leaf's scene into the dump's global id space. */
function offsetScene(source: VizScene, offsets: SceneOffsets): VizScene {
  return {
    vertices: source.vertices.map((vertex) => ({
      ...vertex,
      vertexId: vertex.vertexId + offsets.vertexBase,
    })),
    edges: source.edges.map((edge) => ({
      ...edge,
      edgeId: edge.edgeId + offsets.edgeBase,
    })),
    faces: source.faces.map((face) => ({
      ...face,
      faceId: face.faceId + offsets.faceBase,
    })),
    darts: source.darts.map((dart) => ({
      ...dart,
      dartId: dart.dartId + offsets.dartBase,
      edgeId: dart.edgeId + offsets.edgeBase,
    })),
    alphaLinks: source.alphaLinks.map((link) => ({
      ...link,
      dartA: link.dartA + offsets.dartBase,
      dartB: link.dartB + offsets.dartBase,
    })),
    labels: [...source.labels],
  };
}

function appendScene(target: VizScene, source: VizScene) {
  target.vertices.push(...source.vertices);
  target.edges.push(...source.edges);
  target.faces.push(...source.faces);
  target.darts.push(...source.darts);
  target.alphaLinks.push(...source.alphaLinks);
  target.labels.push(...source.labels);
}

function sceneIdSpan<T extends Record<K, number>, K extends string>(
  entries: T[],
  key: K,
): number {
  return entries.reduce((largest, entry) => Math.max(largest, entry[key] + 1), 0);
}

function emptyScene(): VizScene {
  return {
    vertices: [],
    edges: [],
    faces: [],
    darts: [],
    alphaLinks: [],
    labels: [],
  };
}
