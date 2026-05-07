import type { Kernel } from "./useKernel";

export type Vec3 = [number, number, number];

// ---------- BRep-typed scene ----------

export type VizVertex = {
  vertexId: number;
  position: Vec3;
  color?: string;
  size?: number;
  label?: string;
};

export type VizEdge = {
  edgeId: number;
  polyline: Vec3[];
  color?: string;
  width?: number;
  label?: string;
};

export type VizFace = {
  faceId: number;
  positions: Vec3[];
  normals: Vec3[];
  indices: number[];
  color?: string;
  opacity?: number;
  doubleSided?: boolean;
  label?: string;
};

export type VizDart = {
  dartId: number;
  edgeId: number;
  shaft: Vec3[];
  tipDir: Vec3;
  color?: string;
  label?: string;
};

export type VizAlphaLink = {
  involution: number;
  dartA: number;
  dartB: number;
  a: Vec3;
  b: Vec3;
};

export type VizLabel = {
  position: Vec3;
  text: string;
  color?: string;
};

export type VizScene = {
  vertices: VizVertex[];
  edges: VizEdge[];
  faces: VizFace[];
  darts: VizDart[];
  alphaLinks: VizAlphaLink[];
  labels: VizLabel[];
};

// ---------- GMap snapshot ----------

export type VertexPointEntry = {
  dart: number;
  position: Vec3;
};

export type GMapSnapshot = {
  dimension: number;
  dartCount: number;
  alphas: number[][]; // alphas[i][d] = αᵢ(d)
  vertexPoints: VertexPointEntry[];
};

export type ScriptResult = {
  scene: VizScene;
  gmap?: GMapSnapshot;
};

export function listScripts(kernel: Kernel): string[] {
  return (kernel.listScripts() as string[]) ?? [];
}

export function runScript(kernel: Kernel, name: string): ScriptResult {
  const raw = kernel.runScript(name) as ScriptResult;
  return {
    scene: {
      vertices: raw.scene?.vertices ?? [],
      edges: raw.scene?.edges ?? [],
      faces: raw.scene?.faces ?? [],
      darts: raw.scene?.darts ?? [],
      alphaLinks: raw.scene?.alphaLinks ?? [],
      labels: raw.scene?.labels ?? [],
    },
    gmap: raw.gmap,
  };
}

// ---------- GMap snapshot helpers (pure JS, console-friendly) ----------

/** αᵢ(d). Returns `d` when the dart is free on that involution. */
export function alphaOf(snap: GMapSnapshot, i: number, dart: number): number {
  return snap.alphas[i][dart];
}

/** All dart ids. */
export function darts(snap: GMapSnapshot): number[] {
  return Array.from({ length: snap.dartCount }, (_, i) => i);
}

/** Stored vertex position for `dart`, if any. */
export function vertexPoint(snap: GMapSnapshot, dart: number): Vec3 | undefined {
  return snap.vertexPoints.find((e) => e.dart === dart)?.position;
}

/** Darts reached from `dart` using only the given involutions (BFS). */
export function orbit(
  snap: GMapSnapshot,
  dart: number,
  involutions: number[],
): number[] {
  const visited = new Set<number>([dart]);
  const queue = [dart];
  const out: number[] = [];
  while (queue.length) {
    const d = queue.shift()!;
    out.push(d);
    for (const i of involutions) {
      const n = snap.alphas[i][d];
      if (!visited.has(n)) {
        visited.add(n);
        queue.push(n);
      }
    }
  }
  return out;
}

/**
 * Orbit indices for a `cellDim`-cell in this gmap (`{0..dim-1} \ {cellDim}`).
 * Example: `cellOrbitIndices(snap, 0)` in a dim-3 gmap gives `[1, 2]` — the
 * classical vertex orbit.
 */
export function cellOrbitIndices(snap: GMapSnapshot, cellDim: number): number[] {
  const out: number[] = [];
  for (let i = 0; i < snap.dimension; i++) if (i !== cellDim) out.push(i);
  return out;
}

/** Minimum-id dart of the `cellDim`-cell containing `dart`. */
export function cellRepresentative(
  snap: GMapSnapshot,
  dart: number,
  cellDim: number,
): number {
  return Math.min(...orbit(snap, dart, cellOrbitIndices(snap, cellDim)));
}

/** All darts in the `cellDim`-cell containing `dart`. */
export function cellDarts(
  snap: GMapSnapshot,
  dart: number,
  cellDim: number,
): number[] {
  return orbit(snap, dart, cellOrbitIndices(snap, cellDim));
}

/** Unique representative darts of `targetDim`-cells incident to the
 * `containerDim`-cell at `dart`. */
export function incidentCells(
  snap: GMapSnapshot,
  dart: number,
  containerDim: number,
  targetDim: number,
): number[] {
  const seen = new Set<number>();
  for (const d of cellDarts(snap, dart, containerDim)) {
    seen.add(cellRepresentative(snap, d, targetDim));
  }
  return [...seen].sort((a, b) => a - b);
}

/** Is `dart` free on involution `i` (αᵢ(d) = d)? */
export function isFree(snap: GMapSnapshot, i: number, dart: number): boolean {
  return snap.alphas[i][dart] === dart;
}

/**
 * Bundles a snapshot with the helpers curried on it, for convenient console
 * inspection: `window.$gmap.orbit(2, [0,1])` etc.
 */
export function gmapConsoleApi(snap: GMapSnapshot) {
  return {
    snap,
    darts: () => darts(snap),
    alpha: (i: number, d: number) => alphaOf(snap, i, d),
    isFree: (i: number, d: number) => isFree(snap, i, d),
    vertexPoint: (d: number) => vertexPoint(snap, d),
    orbit: (d: number, involutions: number[]) => orbit(snap, d, involutions),
    cellDarts: (d: number, cellDim: number) => cellDarts(snap, d, cellDim),
    cellRepresentative: (d: number, cellDim: number) =>
      cellRepresentative(snap, d, cellDim),
    incidentCells: (d: number, containerDim: number, targetDim: number) =>
      incidentCells(snap, d, containerDim, targetDim),
  };
}
