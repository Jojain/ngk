import type {
  Circle,
  Cylinder,
  Edge,
  Face,
  GMap,
  Line,
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
  kind: "ngk.debug.v3";
  name: string;
  objects: SerializedDebugObject[];
};

export type DebugObjectKind =
  | "gmap"
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
  | GMap
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
  gmap?: GMap;
};

export type DebugEntityEntry<T> = {
  id: number;
  value: T;
  gmap: GMap;
};

export type DebugSelectionIndex = {
  vertices: DebugEntityEntry<Vertex>[];
  edges: DebugEntityEntry<Edge>[];
  faces: DebugEntityEntry<Face>[];
  darts: Array<{ id: number; dart: number; gmap: GMap }>;
};

export type HydratedDebugDump = {
  name: string;
  object: DebugObject | undefined;
  objects: DebugObject[];
  shape: DebugObject | undefined;
  shapes: DebugObject[];
  gmap: GMap | undefined;
  gmaps: GMap[];
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
  if (payload.kind !== "ngk.debug.v3") {
    throw new Error(`unsupported debug object payload: ${String(payload.kind)}`);
  }

  const scene = emptyScene();
  const selection: DebugSelectionIndex = {
    vertices: [],
    edges: [],
    faces: [],
    darts: [],
  };
  const hydrated: HydratedObject[] = [];
  let vertexBase = 0;
  let edgeBase = 0;
  let faceBase = 0;
  let dartBase = 0;

  for (const serialized of payload.objects) {
    let localScene: VizScene;
    if (isTopologyKind(serialized.kind)) {
      const gmap = kernel.GMap.deserialize(serialized.serialized);
      const vertices = gmap.vertices();
      const edges = gmap.edges();
      const faces = gmap.faces();
      localScene = kernel.sceneFromGMap(gmap) as VizScene;

      selection.vertices.push(
        ...vertices.map((value, id) => ({ id: vertexBase + id, value, gmap })),
      );
      selection.edges.push(
        ...edges.map((value, id) => ({ id: edgeBase + id, value, gmap })),
      );
      selection.faces.push(
        ...faces.map((value, id) => ({ id: faceBase + id, value, gmap })),
      );
      selection.darts.push(
        ...Array.from(gmap.darts(), (dart) => ({
          id: dartBase + dart,
          dart,
          gmap,
        })),
      );

      hydrated.push({
        kind: serialized.kind,
        value: resolvePrimaryTopology(gmap, serialized),
        gmap,
      });
    } else {
      const geometry = kernel.hydrateDebugGeometry(
        serialized.kind,
        serialized.serialized,
      ) as { value: DebugGeometry; scene: VizScene };
      localScene = geometry.scene;
      hydrated.push({ kind: serialized.kind, value: geometry.value });
    }

    appendScene(scene, localScene, { vertexBase, edgeBase, faceBase, dartBase });
    vertexBase += sceneIdSpan(localScene.vertices, "vertexId");
    edgeBase += sceneIdSpan(localScene.edges, "edgeId");
    faceBase += sceneIdSpan(localScene.faces, "faceId");
    dartBase += sceneIdSpan(localScene.darts, "dartId");
  }

  const objects = hydrated.map(({ value }) => value);
  const gmaps = hydrated.flatMap(({ gmap }) => (gmap ? [gmap] : []));
  return {
    name: payload.name,
    object: objects[0],
    objects,
    shape: objects[0],
    shapes: objects,
    gmap: gmaps[0],
    gmaps,
    scene,
    selection,
  };
}

function resolvePrimaryTopology(
  gmap: GMap,
  serialized: SerializedDebugObject,
): GMap | Vertex | Edge | Profile | Face | Sheet | Solid {
  if (serialized.kind === "gmap") return gmap;
  const dart = serialized.primaryDart;
  if (dart === undefined) {
    throw new Error(`${serialized.kind} debug object has no primary dart`);
  }

  const value =
    serialized.kind === "vertex"
      ? gmap.vertex(dart)
      : serialized.kind === "edge"
        ? gmap.edge(dart)
        : serialized.kind === "profile"
          ? gmap.profile(dart)
          : serialized.kind === "face"
            ? gmap.face(dart)
            : serialized.kind === "sheet"
              ? gmap.sheet(dart)
              : gmap.solid(dart);
  if (!value) throw new Error(`could not restore ${serialized.kind} at dart ${dart}`);
  return value;
}

function isTopologyKind(
  kind: DebugObjectKind,
): kind is "gmap" | "vertex" | "edge" | "profile" | "face" | "sheet" | "solid" {
  return (
    kind === "gmap" ||
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

function appendScene(target: VizScene, source: VizScene, offsets: SceneOffsets) {
  target.vertices.push(
    ...source.vertices.map((vertex) => ({
      ...vertex,
      vertexId: vertex.vertexId + offsets.vertexBase,
    })),
  );
  target.edges.push(
    ...source.edges.map((edge) => ({
      ...edge,
      edgeId: edge.edgeId + offsets.edgeBase,
    })),
  );
  target.faces.push(
    ...source.faces.map((face) => ({
      ...face,
      faceId: face.faceId + offsets.faceBase,
    })),
  );
  target.darts.push(
    ...source.darts.map((dart) => ({
      ...dart,
      dartId: dart.dartId + offsets.dartBase,
      edgeId: dart.edgeId + offsets.edgeBase,
    })),
  );
  target.alphaLinks.push(
    ...source.alphaLinks.map((link) => ({
      ...link,
      dartA: link.dartA + offsets.dartBase,
      dartB: link.dartB + offsets.dartBase,
    })),
  );
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
