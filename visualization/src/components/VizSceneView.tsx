import { useMemo } from "react";
import * as THREE from "three";
import { Html, Line } from "@react-three/drei";
import type {
  VizAlphaLink,
  VizDart,
  VizEdge,
  VizFace,
  VizScene,
  VizVertex,
  Vec3,
} from "../kernel/viz";

export type VizSceneViewProps = {
  scene: VizScene;
  vertexColor?: string;
  edgeColor?: string;
  faceColor?: string;
  /**
   * When true, mesh albedo is always `faceColor`. When false, use each face's
   * `color` from the scene when set, otherwise `faceColor`.
   */
  viewerFaceColorOverridesScene?: boolean;
  dartColor?: string;
  vertexSize?: number;
  edgeWidth?: number;
  arrowHeadRatio?: number;
  showVertices?: boolean;
  showEdges?: boolean;
  showFaces?: boolean;
  /** When false, hides dart arrows only; α-links follow `visibleAlphas`. */
  showDarts?: boolean;
  showDartLabels?: boolean;
  showLabels?: boolean;
  /** Involution index → color for α-links. Missing indices get a default. */
  alphaColors?: Record<number, string>;
  /** If set, only α-links whose `involution` is in this set are drawn. */
  visibleAlphas?: Set<number>;
};

const DEFAULT_ALPHA_COLORS: Record<number, string> = {
  0: "#ff1744",
  1: "#00e676",
  2: "#00b0ff",
  3: "#ffea00",
};

function hasVisibleAlphaLinks(
  links: VizAlphaLink[],
  visibleAlphas?: Set<number>,
): boolean {
  if (links.length === 0) return false;
  if (!visibleAlphas) return true;
  return links.some((l) => visibleAlphas.has(l.involution));
}

/**
 * BRep-typed renderer for a [`VizScene`]. Splits into two logical layers:
 *
 * - **BRep** (vertices, edges, faces): the actual shape, tessellated.
 * - **GMap** (darts, α-links, labels): the combinatorial debugging overlay.
 *
 * Every entity attaches its topology id to three.js `userData` so a future
 * picking pass can correlate hover events with kernel state without changing
 * the IR.
 */
export default function VizSceneView({
  scene,
  vertexColor = "#ffc857",
  edgeColor = "#9aa0a6",
  faceColor = "#4a7bc8",
  viewerFaceColorOverridesScene = false,
  dartColor = "#cfd2d6",
  vertexSize = 0.06,
  edgeWidth = 4,
  arrowHeadRatio = 0.28,
  showVertices = true,
  showEdges = true,
  showFaces = true,
  showDarts = true,
  showDartLabels = false,
  showLabels = true,
  alphaColors = DEFAULT_ALPHA_COLORS,
  visibleAlphas,
}: VizSceneViewProps) {
  const alphaColor = (i: number) =>
    alphaColors[i] ?? DEFAULT_ALPHA_COLORS[i] ?? "#bbbbbb";

  const showGMapOverlay =
    showDarts || hasVisibleAlphaLinks(scene.alphaLinks, visibleAlphas);

  return (
    <group>
      {showFaces && (
        <BrepLayer
          faces={scene.faces}
          edges={[]}
          vertices={[]}
          edgeColor={edgeColor}
          edgeWidth={edgeWidth}
          faceColor={faceColor}
          viewerFaceColorOverridesScene={viewerFaceColorOverridesScene}
          vertexColor={vertexColor}
          vertexSize={vertexSize}
        />
      )}
      {showEdges && (
        <BrepLayer
          faces={[]}
          edges={scene.edges}
          vertices={[]}
          edgeColor={edgeColor}
          edgeWidth={edgeWidth}
          faceColor={faceColor}
          viewerFaceColorOverridesScene={viewerFaceColorOverridesScene}
          vertexColor={vertexColor}
          vertexSize={vertexSize}
        />
      )}
      {showVertices && (
        <BrepLayer
          faces={[]}
          edges={[]}
          vertices={scene.vertices}
          edgeColor={edgeColor}
          edgeWidth={edgeWidth}
          faceColor={faceColor}
          viewerFaceColorOverridesScene={viewerFaceColorOverridesScene}
          vertexColor={vertexColor}
          vertexSize={vertexSize}
        />
      )}

      {showGMapOverlay && (
        <GMapLayer
          showDarts={showDarts}
          darts={scene.darts}
          alphaLinks={scene.alphaLinks}
          dartColor={dartColor}
          arrowHeadRatio={arrowHeadRatio}
          showDartLabels={showDartLabels}
          alphaColor={alphaColor}
          visibleAlphas={visibleAlphas}
        />
      )}

      {showLabels &&
        scene.labels.map((l, i) => (
          <Html
            key={`lab-${i}`}
            position={l.position}
            center
            distanceFactor={8}
            style={{
              color: l.color ?? "#e8e8ef",
              fontSize: 10,
              pointerEvents: "none",
              whiteSpace: "nowrap",
            }}
          >
            {l.text}
          </Html>
        ))}
    </group>
  );
}

// ---------- BRep layer ----------

function BrepLayer({
  faces,
  edges,
  vertices,
  edgeColor,
  edgeWidth,
  faceColor,
  viewerFaceColorOverridesScene,
  vertexColor,
  vertexSize,
}: {
  faces: VizFace[];
  edges: VizEdge[];
  vertices: VizVertex[];
  edgeColor: string;
  edgeWidth: number;
  faceColor: string;
  viewerFaceColorOverridesScene: boolean;
  vertexColor: string;
  vertexSize: number;
}) {
  return (
    <group>
      {vertices.map((v) => (
        <VertexPoint
          key={`v-${v.vertexId}`}
          vertex={v}
          color={v.color ?? vertexColor}
          size={v.size ?? vertexSize}
        />
      ))}
      {edges.map((e) => (
        <EdgePolyline
          key={`e-${e.edgeId}`}
          edge={e}
          color={e.color ?? edgeColor}
          width={e.width ?? edgeWidth}
        />
      ))}
      {faces.map((f) => (
        <FaceMesh
          key={`f-${f.faceId}`}
          face={f}
          defaultColor={faceColor}
          viewerOverridesScene={viewerFaceColorOverridesScene}
        />
      ))}
    </group>
  );
}

function VertexPoint({
  vertex,
  color,
  size,
}: {
  vertex: VizVertex;
  color: string;
  size: number;
}) {
  return (
    <mesh
      position={vertex.position}
      userData={{ kind: "vertex", vertexId: vertex.vertexId }}
    >
      <sphereGeometry args={[size, 16, 12]} />
      <meshStandardMaterial color={color} />
    </mesh>
  );
}

function EdgePolyline({
  edge,
  color,
  width,
}: {
  edge: VizEdge;
  color: string;
  width: number;
}) {
  if (edge.polyline.length < 2) return null;
  return (
    <Line
      points={edge.polyline}
      color={color}
      lineWidth={width}
      userData={{ kind: "edge", edgeId: edge.edgeId }}
    />
  );
}

function FaceMesh({
  face,
  defaultColor,
  viewerOverridesScene,
}: {
  face: VizFace;
  defaultColor: string;
  viewerOverridesScene: boolean;
}) {
  const geometry = useMemo(() => {
    const geom = new THREE.BufferGeometry();
    const positions = new Float32Array(face.positions.length * 3);
    for (let i = 0; i < face.positions.length; i++) {
      positions[i * 3 + 0] = face.positions[i][0];
      positions[i * 3 + 1] = face.positions[i][1];
      positions[i * 3 + 2] = face.positions[i][2];
    }
    geom.setAttribute("position", new THREE.BufferAttribute(positions, 3));

    if (face.normals.length === face.positions.length) {
      const normals = new Float32Array(face.normals.length * 3);
      for (let i = 0; i < face.normals.length; i++) {
        normals[i * 3 + 0] = face.normals[i][0];
        normals[i * 3 + 1] = face.normals[i][1];
        normals[i * 3 + 2] = face.normals[i][2];
      }
      geom.setAttribute("normal", new THREE.BufferAttribute(normals, 3));
    } else {
      geom.computeVertexNormals();
    }

    geom.setIndex(face.indices);
    geom.computeBoundingSphere();
    return geom;
  }, [face]);

  const materialColor = viewerOverridesScene
    ? defaultColor
    : (face.color ?? defaultColor);

  return (
    <mesh
      geometry={geometry}
      userData={{ kind: "face", faceId: face.faceId }}
    >
      <meshStandardMaterial
        color={materialColor}
        opacity={face.opacity ?? 1}
        transparent={face.opacity !== undefined && face.opacity < 1}
        roughness={0.55}
        metalness={0.08}
        side={face.doubleSided ? THREE.DoubleSide : THREE.FrontSide}
      />
    </mesh>
  );
}

// ---------- GMap overlay ----------

function GMapLayer({
  showDarts,
  darts,
  alphaLinks,
  dartColor,
  arrowHeadRatio,
  showDartLabels,
  alphaColor,
  visibleAlphas,
}: {
  showDarts: boolean;
  darts: VizDart[];
  alphaLinks: VizAlphaLink[];
  dartColor: string;
  arrowHeadRatio: number;
  showDartLabels: boolean;
  alphaColor: (i: number) => string;
  visibleAlphas?: Set<number>;
}) {
  const display = useMemo(
    () => layoutDartLanes(darts, alphaLinks),
    [darts, alphaLinks],
  );

  return (
    <group>
      {showDarts &&
        darts.map((d) => (
          <Dart
            key={`d-${d.dartId}`}
            dart={d}
            shaft={display.shaftsByDartId.get(d.dartId) ?? d.shaft}
            color={d.color ?? dartColor}
            headRatio={arrowHeadRatio}
            showLabel={showDartLabels}
          />
        ))}
      {alphaLinks.map((l, i) =>
        visibleAlphas && !visibleAlphas.has(l.involution) ? null : (
          <AlphaLink
            key={`l-${i}`}
            link={l}
            a={display.midpointsByDartId.get(l.dartA) ?? l.a}
            b={display.midpointsByDartId.get(l.dartB) ?? l.b}
            color={alphaColor(l.involution)}
          />
        ),
      )}
    </group>
  );
}

const DART_LANE_RADIUS = 0.045;
const WORLD_UP: Vec3 = [0, 0, 1];
const WORLD_UP_FALLBACK: Vec3 = [0, 1, 0];

type DartLaneLayout = {
  shaftsByDartId: Map<number, Vec3[]>;
  midpointsByDartId: Map<number, Vec3>;
};

function layoutDartLanes(
  darts: VizDart[],
  alphaLinks: VizAlphaLink[],
): DartLaneLayout {
  const shaftsByDartId = new Map<number, Vec3[]>();
  const midpointsByDartId = new Map<number, Vec3>();
  const faceOffsets = faceInwardOffsets(darts, alphaLinks);
  const edgeFallback = edgeLaneFallbacks(darts);

  for (const dart of darts) {
    const offset = faceOffsets.get(dart.dartId) ?? edgeFallback.get(dart.dartId);
    const shaft = offset ? offsetShaft(dart.shaft, offset) : dart.shaft;
    shaftsByDartId.set(dart.dartId, shaft);
    midpointsByDartId.set(dart.dartId, shaftMidpoint(shaft));
  }

  return { shaftsByDartId, midpointsByDartId };
}

function faceInwardOffsets(
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

  const offsets = new Map<number, Vec3>();
  const seen = new Set<number>();
  for (const dart of darts) {
    if (seen.has(dart.dartId)) continue;
    const component = connectedDarts(dart.dartId, adjacency, seen)
      .map((id) => byId.get(id))
      .filter((d): d is VizDart => Boolean(d));
    if (component.length < 3) continue;

    const center = averagePoints(component.flatMap((d) => d.shaft));
    for (const d of component) {
      const midpoint = shaftMidpoint(d.shaft);
      const tangent = shaftTangent(d.shaft);
      if (!tangent) continue;
      const inward = sub(center, midpoint);
      const projected = subtractProjection(inward, tangent);
      const direction = normalize(projected);
      if (!direction) continue;
      offsets.set(d.dartId, scale(direction, DART_LANE_RADIUS));
    }
  }
  return offsets;
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

function edgeLaneFallbacks(darts: VizDart[]): Map<number, Vec3> {
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
    const frame = dartLaneFrame(group);
    if (!frame) continue;
    for (let i = 0; i < group.length; i++) {
      offsets.set(group[i].dartId, laneOffset(frame, i, group.length));
    }
  }
  return offsets;
}

type DartLaneFrame = {
  u: Vec3;
  v: Vec3;
};

function dartLaneFrame(darts: VizDart[]): DartLaneFrame | null {
  for (const dart of darts) {
    const tangent = shaftTangent(dart.shaft);
    if (!tangent) continue;
    const up =
      Math.abs(dot(tangent, WORLD_UP)) > 0.92 ? WORLD_UP_FALLBACK : WORLD_UP;
    const u = normalize(cross(tangent, up));
    if (!u) continue;
    const v = normalize(cross(tangent, u));
    if (!v) continue;
    return { u, v };
  }
  return null;
}

function shaftTangent(shaft: Vec3[]): Vec3 | null {
  for (let i = 1; i < shaft.length; i++) {
    const tangent = normalize(sub(shaft[i], shaft[i - 1]));
    if (tangent) return tangent;
  }
  return null;
}

function laneOffset(frame: DartLaneFrame, index: number, count: number): Vec3 {
  const angle = (Math.PI * 2 * index) / count;
  const cu = Math.cos(angle) * DART_LANE_RADIUS;
  const sv = Math.sin(angle) * DART_LANE_RADIUS;
  return [
    frame.u[0] * cu + frame.v[0] * sv,
    frame.u[1] * cu + frame.v[1] * sv,
    frame.u[2] * cu + frame.v[2] * sv,
  ];
}

function offsetShaft(shaft: Vec3[], offset: Vec3): Vec3[] {
  return shaft.map((point) => add(point, offset));
}

function shaftMidpoint(shaft: Vec3[]): Vec3 {
  if (shaft.length === 0) return [0, 0, 0];
  const n = shaft.length;
  if (n % 2 === 1) return shaft[Math.floor(n / 2)];
  return scale(add(shaft[n / 2 - 1], shaft[n / 2]), 0.5);
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

function normalize(v: Vec3): Vec3 | null {
  const len = Math.hypot(v[0], v[1], v[2]);
  if (len < 1e-12) return null;
  return [v[0] / len, v[1] / len, v[2] / len];
}

/**
 * One half-edge arrow: a display shaft derived from the edge curve plus a
 * cone tip oriented along `tipDir`. Shared-edge darts are lane-offset before
 * this component receives them.
 */
function Dart({
  dart,
  shaft,
  color,
  headRatio,
  showLabel,
}: {
  dart: VizDart;
  shaft: Vec3[];
  color: string;
  headRatio: number;
  showLabel: boolean;
}) {
  const geom = useMemo(() => {
    if (shaft.length < 2) return null;

    const last = shaft[shaft.length - 1];
    const origin = shaft[0];
    let length = 0;
    for (let i = 1; i < shaft.length; i++) {
      const a = shaft[i - 1];
      const b = shaft[i];
      length += Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    }
    if (length < 1e-12) return null;

    const headLength = Math.min(length * headRatio, length * 0.9);
    const headRadius = headLength * 0.4;

    const dir = new THREE.Vector3(...dart.tipDir);
    if (dir.lengthSq() < 1e-12) {
      const a = shaft[shaft.length - 2];
      dir.set(last[0] - a[0], last[1] - a[1], last[2] - a[2]).normalize();
    }
    const coneCenter = new THREE.Vector3(...last).addScaledVector(
      dir,
      -headLength / 2,
    );
    const quaternion = new THREE.Quaternion().setFromUnitVectors(
      new THREE.Vector3(0, 1, 0),
      dir,
    );

    return {
      origin,
      coneCenter,
      quaternion,
      headRadius,
      headLength,
      labelPos: last,
    };
  }, [shaft, dart.tipDir, headRatio]);

  if (!geom) return null;

  return (
    <group userData={{ kind: "dart", dartId: dart.dartId, edgeId: dart.edgeId }}>
      <Line points={shaft} color={color} lineWidth={1.5} />
      <mesh position={geom.coneCenter} quaternion={geom.quaternion}>
        <coneGeometry args={[geom.headRadius, geom.headLength, 12]} />
        <meshStandardMaterial color={color} />
      </mesh>
      {showLabel && dart.label && (
        <Html
          position={geom.labelPos}
          center
          distanceFactor={8}
          style={{
            color,
            fontSize: 9,
            pointerEvents: "none",
            whiteSpace: "nowrap",
            transform: "translate(8px, -4px)",
          }}
        >
          {dart.label}
        </Html>
      )}
    </group>
  );
}

function AlphaLink({
  link,
  a,
  b,
  color,
}: {
  link: VizAlphaLink;
  a: Vec3;
  b: Vec3;
  color: string;
}) {
  return (
    <Line
      points={[a, b]}
      color={color}
      lineWidth={5}
      dashed
      dashSize={0.04}
      gapSize={0.03}
      userData={{
        kind: "alphaLink",
        involution: link.involution,
        dartA: link.dartA,
        dartB: link.dartB,
      }}
    />
  );
}
