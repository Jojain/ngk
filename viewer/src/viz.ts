export type Vec3 = [number, number, number];

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
  arrowHead?: boolean;
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
