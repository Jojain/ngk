import type { Vec3, VizScene } from "./viz";

export type DebugViewerEnvelope = {
  receivedAt: string;
  sequence: number;
  payload: DebugViewerPayload;
};

export type DebugViewerPayload = {
  kind: "ngk.debug.v1";
  name: string;
  scene: VizScene;
  gmap: GMapDebugSnapshot;
  selection: SelectionIndex;
  metadata: DebugMetadata;
};

export type GMapDebugSnapshot = {
  dimension: number;
  dartCount: number;
  alphas: number[][];
  darts: DartMetadata[];
};

export type DartMetadata = {
  dart: number;
  vertex?: string;
  edge?: string;
  profile?: string;
  face?: string;
  sheet?: string;
  solid?: string;
};

export type SelectionIndex = {
  vertices: EntitySelection[];
  edges: EntitySelection[];
  faces: EntitySelection[];
};

export type EntitySelection = {
  renderId: number;
  key: string;
  representativeDart: number;
};

export type DebugMetadata = {
  vertices: VertexMetadata[];
  edges: EdgeMetadata[];
  profiles: ProfileMetadata[];
  faces: FaceMetadata[];
  sheets: SheetMetadata[];
  solids: SolidMetadata[];
};

export type VertexMetadata = {
  key: string;
  representativeDart: number;
  darts: number[];
  point: Vec3;
  payload: PayloadSummary;
};

export type EdgeMetadata = {
  key: string;
  representativeDart: number;
  darts: number[];
  curve: GeometrySummary;
  payload: PayloadSummary;
};

export type ProfileMetadata = {
  key: string;
  representativeDart: number;
  darts: number[];
  closed: boolean;
  edgeKeys: string[];
  vertexKeys: string[];
  payload: PayloadSummary;
};

export type FaceMetadata = {
  key: string;
  representativeDart: number;
  darts: number[];
  outerLoop: number[];
  innerLoops: number[][];
  surface: GeometrySummary;
  normals: NormalSample[];
  pcurves: PcurveMetadata[];
  payload: PayloadSummary;
};

export type NormalSample = {
  origin: Vec3;
  direction: Vec3;
};

export type SheetMetadata = {
  key: string;
  representativeDart: number;
  darts: number[];
  closed: boolean;
  faceKeys: string[];
  edgeKeys: string[];
  vertexKeys: string[];
  payload: PayloadSummary;
};

export type SolidMetadata = {
  key: string;
  representativeDart: number;
  darts: number[];
  innerShells?: number[];
  payload: PayloadSummary;
};

export type PcurveMetadata = {
  dart: number;
  edgeKey: string;
  startVertexKey: string;
  endVertexKey: string;
  curve: GeometrySummary;
  samples: [number, number][];
};

export type GeometrySummary = {
  kind: string;
  details?: string;
};

export type PayloadSummary = {
  typeName: string;
  debug: string;
};

const ENDPOINT = "/__ngk_debug/dumps";

export async function fetchDebugDumps(): Promise<DebugViewerEnvelope[]> {
  const response = await fetch(ENDPOINT);
  if (!response.ok) throw new Error(`debug dump fetch failed: ${response.status}`);
  return (await response.json()) as DebugViewerEnvelope[];
}

export async function clearDebugDumps(): Promise<void> {
  const response = await fetch(ENDPOINT, { method: "DELETE" });
  if (!response.ok) throw new Error(`debug dump clear failed: ${response.status}`);
}
