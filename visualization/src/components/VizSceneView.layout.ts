import type { Vec3, VizAlphaLink, VizDart } from "../kernel/viz";

const DART_FACE_LANE_RADIUS = 0.045;
const DART_EDGE_LANE_RADIUS = 0.028;
const WORLD_UP: Vec3 = [0, 0, 1];
const WORLD_UP_FALLBACK: Vec3 = [0, 1, 0];

export type DartLaneLayout = {
  shaftsByDartId: Map<number, Vec3[]>;
  midpointsByDartId: Map<number, Vec3>;
  alphaEndpoint(dartId: number): Vec3 | undefined;
};

export function layoutDartLanes(
  darts: VizDart[],
  alphaLinks: VizAlphaLink[],
): DartLaneLayout {
  const shaftsByDartId = new Map<number, Vec3[]>();
  const midpointsByDartId = new Map<number, Vec3>();
  const faceCenters = faceLaneCenters(darts, alphaLinks);
  const edgeOffsets = edgeLaneOffsets(darts);

  for (const dart of darts) {
    const faceCenter = faceCenters.get(dart.dartId);
    const edgeOffset = edgeOffsets.get(dart.dartId);
    let shaft = faceCenter
      ? offsetShaftTowardCenter(dart.shaft, faceCenter, DART_FACE_LANE_RADIUS)
      : dart.shaft;
    if (!faceCenter && edgeOffset) shaft = offsetShaft(shaft, edgeOffset);
    shaftsByDartId.set(dart.dartId, shaft);
    midpointsByDartId.set(dart.dartId, shaftMidpoint(shaft));
  }

  return {
    shaftsByDartId,
    midpointsByDartId,
    alphaEndpoint: (dartId) => midpointsByDartId.get(dartId),
  };
}

function faceLaneCenters(
  darts: VizDart[],
  alphaLinks: VizAlphaLink[],
): Map<number, Vec3> {
  const byId = new Map(darts.map((dart) => [dart.dartId, dart]));
  const adjacency = new Map<number, number[]>();
  for (const dart of darts) adjacency.set(dart.dartId, []);

  for (const link of alphaLinks) {
    if (link.involution !== 0 && link.involution !== 1) continue;
    adjacency.get(link.dartA)?.push(link.dartB);
    adjacency.get(link.dartB)?.push(link.dartA);
  }

  const centers = new Map<number, Vec3>();
  const seen = new Set<number>();
  for (const dart of darts) {
    if (seen.has(dart.dartId)) continue;
    const component = connectedDarts(dart.dartId, adjacency, seen)
      .map((id) => byId.get(id))
      .filter((d): d is VizDart => Boolean(d));
    if (component.length < 3) continue;

    const center = averagePoints(component.map((d) => shaftMidpoint(d.shaft)));
    for (const d of component) {
      centers.set(d.dartId, center);
    }
  }
  return centers;
}

function connectedDarts(
  start: number,
  adjacency: Map<number, number[]>,
  seen: Set<number>,
): number[] {
  const out: number[] = [];
  const queue = [start];
  seen.add(start);
  while (queue.length) {
    const current = queue.shift()!;
    out.push(current);
    for (const next of adjacency.get(current) ?? []) {
      if (seen.has(next)) continue;
      seen.add(next);
      queue.push(next);
    }
  }
  return out;
}

function edgeLaneOffsets(darts: VizDart[]): Map<number, Vec3> {
  const offsets = new Map<number, Vec3>();
  const groups = new Map<number, VizDart[]>();
  for (const dart of darts) {
    const group = groups.get(dart.edgeId);
    if (group) group.push(dart);
    else groups.set(dart.edgeId, [dart]);
  }

  for (const group of groups.values()) {
    if (group.length <= 1) continue;
    group.sort((a, b) => a.dartId - b.dartId);
    const axis = edgeLaneAxis(group);
    if (!axis) continue;
    const step = DART_EDGE_LANE_RADIUS * 2;
    const center = (group.length - 1) / 2;
    for (let i = 0; i < group.length; i++) {
      offsets.set(group[i].dartId, scale(axis, (i - center) * step));
    }
  }
  return offsets;
}

function edgeLaneAxis(darts: VizDart[]): Vec3 | null {
  for (const dart of darts) {
    const tangent = shaftChordTangent(dart.shaft) ?? shaftTangent(dart.shaft);
    if (!tangent) continue;
    const axis = perpendicularFromTangent(tangent);
    if (axis) return axis;
  }
  return null;
}

function offsetShaft(shaft: Vec3[], offset: Vec3): Vec3[] {
  return shaft.map((point) => add(point, offset));
}

function offsetShaftTowardCenter(
  shaft: Vec3[],
  center: Vec3,
  radius: number,
): Vec3[] {
  const tangent = shaftMidpointTangent(shaft) ?? shaftTangent(shaft);
  const midpoint = shaftMidpoint(shaft);
  const inward = sub(center, midpoint);
  const projected = tangent ? subtractProjection(inward, tangent) : inward;
  const direction =
    normalize(projected) ?? (tangent ? perpendicularFromTangent(tangent) : null);
  if (!direction) return shaft;

  if (isLinearShaft(shaft)) {
    return offsetShaft(shaft, scale(direction, radius));
  }

  return offsetCurvedShaftTowardCenter(shaft, center, direction, radius);
}

function offsetCurvedShaftTowardCenter(
  shaft: Vec3[],
  center: Vec3,
  baseDirection: Vec3,
  radius: number,
): Vec3[] {
  let previousDirection = baseDirection;
  return shaft.map((point, index) => {
    const tangent = shaftSampleTangent(shaft, index);
    const inward = sub(center, point);
    const projected = tangent ? subtractProjection(inward, tangent) : inward;
    let direction = normalize(projected) ?? previousDirection;
    if (dot(direction, previousDirection) < 0) {
      direction = scale(direction, -1);
    }
    previousDirection = direction;
    return add(point, scale(direction, radius));
  });
}

function isLinearShaft(shaft: Vec3[]): boolean {
  if (shaft.length <= 2) return true;
  const start = shaft[0];
  const end = shaft[shaft.length - 1];
  const chord = sub(end, start);
  const chordLen = length(chord);
  if (chordLen < 1e-12) return false;
  const chordUnit = scale(chord, 1 / chordLen);
  const tolerance = Math.max(1e-6, chordLen * 1e-4);
  for (let i = 1; i < shaft.length - 1; i++) {
    const offset = sub(shaft[i], start);
    const distance = length(cross(offset, chordUnit));
    if (distance > tolerance) return false;
  }
  return true;
}

function shaftTangent(shaft: Vec3[]): Vec3 | null {
  for (let i = 1; i < shaft.length; i++) {
    const tangent = normalize(sub(shaft[i], shaft[i - 1]));
    if (tangent) return tangent;
  }
  return null;
}

function shaftChordTangent(shaft: Vec3[]): Vec3 | null {
  if (shaft.length < 2) return null;
  return normalize(sub(shaft[shaft.length - 1], shaft[0]));
}

function shaftMidpointTangent(shaft: Vec3[]): Vec3 | null {
  if (shaft.length < 2) return null;
  const midpoint = shaftTotalLength(shaft) / 2;
  let distance = 0;
  for (let i = 1; i < shaft.length; i++) {
    const segment = sub(shaft[i], shaft[i - 1]);
    const segmentLength = length(segment);
    if (segmentLength < 1e-12) continue;
    if (distance + segmentLength >= midpoint) {
      return scale(segment, 1 / segmentLength);
    }
    distance += segmentLength;
  }
  return shaftTangent(shaft);
}

function shaftSampleTangent(shaft: Vec3[], index: number): Vec3 | null {
  if (shaft.length < 2) return null;
  const prev = shaft[Math.max(0, index - 1)];
  const next = shaft[Math.min(shaft.length - 1, index + 1)];
  return normalize(sub(next, prev));
}

function shaftMidpoint(shaft: Vec3[]): Vec3 {
  if (shaft.length === 0) return [0, 0, 0];
  if (shaft.length === 1) return shaft[0];

  const midpoint = shaftTotalLength(shaft) / 2;
  let distance = 0;
  for (let i = 1; i < shaft.length; i++) {
    const a = shaft[i - 1];
    const b = shaft[i];
    const segmentLength = length(sub(b, a));
    if (segmentLength < 1e-12) continue;
    if (distance + segmentLength >= midpoint) {
      const t = (midpoint - distance) / segmentLength;
      return add(a, scale(sub(b, a), t));
    }
    distance += segmentLength;
  }
  return shaft[shaft.length - 1];
}

function shaftTotalLength(shaft: Vec3[]): number {
  let total = 0;
  for (let i = 1; i < shaft.length; i++) {
    total += length(sub(shaft[i], shaft[i - 1]));
  }
  return total;
}

function perpendicularFromTangent(tangent: Vec3): Vec3 | null {
  const up =
    Math.abs(dot(tangent, WORLD_UP)) > 0.92 ? WORLD_UP_FALLBACK : WORLD_UP;
  return normalize(cross(tangent, up));
}

function averagePoints(points: Vec3[]): Vec3 {
  if (points.length === 0) return [0, 0, 0];
  let sum: Vec3 = [0, 0, 0];
  for (const point of points) sum = add(sum, point);
  return scale(sum, 1 / points.length);
}

function subtractProjection(v: Vec3, ontoUnit: Vec3): Vec3 {
  return sub(v, scale(ontoUnit, dot(v, ontoUnit)));
}

function add(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function sub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function scale(a: Vec3, s: number): Vec3 {
  return [a[0] * s, a[1] * s, a[2] * s];
}

function dot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross(a: Vec3, b: Vec3): Vec3 {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function length(v: Vec3): number {
  return Math.hypot(v[0], v[1], v[2]);
}

function normalize(v: Vec3): Vec3 | null {
  const len = length(v);
  if (len < 1e-12) return null;
  return [v[0] / len, v[1] / len, v[2] / len];
}
