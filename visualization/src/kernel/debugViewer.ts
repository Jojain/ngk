import type {
  Edge,
  Face,
  GMap,
  Profile,
  Sheet,
  Solid,
  Vertex,
} from "../wasm/ngk";
import type { Kernel } from "./useKernel";
import type { VizScene } from "./viz";

export type DebugViewerEnvelope = {
  receivedAt: string;
  sequence: number;
  payload: DebugViewerPayload;
};

export type DebugViewerPayload = {
  kind: "ngk.debug.v2";
  name: string;
  shapes: SerializedDebugShape[];
};

export type DebugShapeKind =
  | "gmap"
  | "vertex"
  | "edge"
  | "profile"
  | "face"
  | "sheet"
  | "solid";

export type SerializedDebugShape = {
  kind: DebugShapeKind;
  primaryDart?: number;
  serialized: string;
};

export type DebugShape = GMap | Vertex | Edge | Profile | Face | Sheet | Solid;

export type HydratedShape = {
  kind: DebugShapeKind;
  value: DebugShape;
  gmap: GMap;
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
  shape: DebugShape | undefined;
  shapes: DebugShape[];
  gmap: GMap | undefined;
  gmaps: GMap[];
  scene: VizScene;
  selection: DebugSelectionIndex;
};

const ENDPOINT = "/__ngk_debug/dumps";

export async function fetchDebugDumps(): Promise<DebugViewerEnvelope[]> {
  const response = await fetch(ENDPOINT);
  if (!response.ok) throw new Error(`debug shape fetch failed: ${response.status}`);
  return (await response.json()) as DebugViewerEnvelope[];
}

export async function clearDebugDumps(): Promise<void> {
  const response = await fetch(ENDPOINT, { method: "DELETE" });
  if (!response.ok) throw new Error(`debug shape clear failed: ${response.status}`);
}

/** Restores the transported maps as real WASM topology objects. */
export function hydrateDebugDump(
  payload: DebugViewerPayload,
  kernel: Kernel,
): HydratedDebugDump {
  if (payload.kind !== "ngk.debug.v2") {
    throw new Error(`unsupported debug shape payload: ${String(payload.kind)}`);
  }

  const scene = emptyScene();
  const selection: DebugSelectionIndex = {
    vertices: [],
    edges: [],
    faces: [],
    darts: [],
  };
  const hydrated: HydratedShape[] = [];
  let vertexBase = 0;
  let edgeBase = 0;
  let faceBase = 0;
  let dartBase = 0;

  for (const serialized of payload.shapes) {
    const gmap = kernel.GMap.deserialize(serialized.serialized);
    const vertices = gmap.vertices();
    const edges = gmap.edges();
    const faces = gmap.faces();
    const localScene = kernel.sceneFromGMap(gmap) as VizScene;
    appendScene(scene, localScene, { vertexBase, edgeBase, faceBase, dartBase });

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
      value: resolvePrimaryShape(gmap, serialized),
      gmap,
    });

    vertexBase += vertices.length;
    edgeBase += edges.length;
    faceBase += faces.length;
    dartBase += gmap.dartCount;
  }

  return {
    name: payload.name,
    shape: hydrated[0]?.value,
    shapes: hydrated.map(({ value }) => value),
    gmap: hydrated[0]?.gmap,
    gmaps: hydrated.map(({ gmap }) => gmap),
    scene,
    selection,
  };
}

function resolvePrimaryShape(
  gmap: GMap,
  serialized: SerializedDebugShape,
): DebugShape {
  if (serialized.kind === "gmap") return gmap;
  const dart = serialized.primaryDart;
  if (dart === undefined) {
    throw new Error(`${serialized.kind} debug shape has no primary dart`);
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
