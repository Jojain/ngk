import { useMemo } from "react";
import * as THREE from "three";
import { Html, Line } from "@react-three/drei";
import type { ThreeEvent } from "@react-three/fiber";
import type {
  VizAlphaLink,
  VizDart,
  VizEdge,
  VizFace,
  VizScene,
  VizVertex,
  Vec3,
} from "../kernel/viz";
import { layoutDartLanes } from "./VizSceneView.layout";

export type VizSelection =
  | { kind: "vertex"; id: number }
  | { kind: "edge"; id: number }
  | { kind: "face"; id: number }
  | { kind: "dart"; id: number }
  | { kind: "alphaLink"; id: number; involution: number };

export type VizSceneViewProps = {
  scene: VizScene;
  vertexColor?: string;
  edgeColor?: string;
  faceColor?: string;
  faceOpacity?: number;
  /**
   * When true, mesh albedo is always `faceColor`. When false, use each face's
   * `color` from the scene when set, otherwise `faceColor`.
   */
  viewerFaceColorOverridesScene?: boolean;
  dartColor?: string;
  vertexSize?: number;
  edgeWidth?: number;
  arrowHeadRatio?: number;
  /** Shows a world-space XYZ frame rooted at [0, 0, 0]. */
  showWorldFrame?: boolean;
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
  selected?: VizSelection | null;
  hovered?: VizSelection | null;
  onSelect?: (selection: VizSelection) => void;
  onHover?: (selection: VizSelection | null) => void;
};

const DEFAULT_ALPHA_COLORS: Record<number, string> = {
  0: "#ff1744",
  1: "#00e676",
  2: "#00b0ff",
  3: "#ffea00",
};

const WORLD_FRAME_SIZE = 1;
const WORLD_FRAME_LABELS: Array<{
  axis: "X" | "Y" | "Z";
  position: Vec3;
  color: string;
}> = [
  { axis: "X", position: [1.12, 0, 0], color: "#ff5f5f" },
  { axis: "Y", position: [0, 1.12, 0], color: "#62d26f" },
  { axis: "Z", position: [0, 0, 1.12], color: "#6ea8ff" },
];

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
  faceOpacity = 1,
  viewerFaceColorOverridesScene = false,
  dartColor = "#cfd2d6",
  vertexSize = 0.04,
  edgeWidth = 6,
  arrowHeadRatio = 0.28,
  showWorldFrame = true,
  showVertices = true,
  showEdges = true,
  showFaces = true,
  showDarts = false,
  showDartLabels = false,
  showLabels = true,
  alphaColors = DEFAULT_ALPHA_COLORS,
  visibleAlphas,
  selected,
  hovered,
  onSelect,
  onHover,
}: VizSceneViewProps) {
  const alphaColor = (i: number) =>
    alphaColors[i] ?? DEFAULT_ALPHA_COLORS[i] ?? "#bbbbbb";

  const showGMapOverlay =
    showDarts || hasVisibleAlphaLinks(scene.alphaLinks, visibleAlphas);

  return (
    <group>
      {showWorldFrame && <WorldFrame />}
      {showFaces && (
        <BrepLayer
          faces={scene.faces}
          edges={[]}
          vertices={[]}
          edgeColor={edgeColor}
          edgeWidth={edgeWidth}
          faceColor={faceColor}
          faceOpacity={faceOpacity}
          viewerFaceColorOverridesScene={viewerFaceColorOverridesScene}
          vertexColor={vertexColor}
          vertexSize={vertexSize}
          selected={selected}
          hovered={hovered}
          onSelect={onSelect}
          onHover={onHover}
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
          faceOpacity={faceOpacity}
          viewerFaceColorOverridesScene={viewerFaceColorOverridesScene}
          vertexColor={vertexColor}
          vertexSize={vertexSize}
          selected={selected}
          hovered={hovered}
          onSelect={onSelect}
          onHover={onHover}
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
          faceOpacity={faceOpacity}
          viewerFaceColorOverridesScene={viewerFaceColorOverridesScene}
          vertexColor={vertexColor}
          vertexSize={vertexSize}
          selected={selected}
          hovered={hovered}
          onSelect={onSelect}
          onHover={onHover}
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
          selected={selected}
          onSelect={onSelect}
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

function WorldFrame() {
  return (
    <group userData={{ kind: "worldFrame" }}>
      <axesHelper args={[WORLD_FRAME_SIZE]} />
      {WORLD_FRAME_LABELS.map(({ axis, position, color }) => (
        <Html
          key={axis}
          position={position}
          center
          distanceFactor={8}
          style={{
            color,
            fontSize: 11,
            fontWeight: 700,
            pointerEvents: "none",
            textShadow: "0 1px 2px #000",
          }}
        >
          {axis}
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
  faceOpacity,
  viewerFaceColorOverridesScene,
  vertexColor,
  vertexSize,
  selected,
  hovered,
  onSelect,
  onHover,
}: {
  faces: VizFace[];
  edges: VizEdge[];
  vertices: VizVertex[];
  edgeColor: string;
  edgeWidth: number;
  faceColor: string;
  faceOpacity: number;
  viewerFaceColorOverridesScene: boolean;
  vertexColor: string;
  vertexSize: number;
  selected?: VizSelection | null;
  hovered?: VizSelection | null;
  onSelect?: (selection: VizSelection) => void;
  onHover?: (selection: VizSelection | null) => void;
}) {
  return (
    <group>
      {vertices.map((v) => (
        <VertexPoint
          key={`v-${v.vertexId}`}
          vertex={v}
          color={v.color ?? vertexColor}
          size={v.size ?? vertexSize}
          selected={selected?.kind === "vertex" && selected.id === v.vertexId}
          hovered={hovered?.kind === "vertex" && hovered.id === v.vertexId}
          onSelect={onSelect}
          onHover={onHover}
        />
      ))}
      {edges.map((e) => (
        <EdgePolyline
          key={`e-${e.edgeId}`}
          edge={e}
          color={e.color ?? edgeColor}
          width={e.width ?? edgeWidth}
          selected={selected?.kind === "edge" && selected.id === e.edgeId}
          hovered={hovered?.kind === "edge" && hovered.id === e.edgeId}
          onSelect={onSelect}
          onHover={onHover}
        />
      ))}
      {faces.map((f) => (
        <FaceMesh
          key={`f-${f.faceId}`}
          face={f}
          defaultColor={faceColor}
          defaultOpacity={faceOpacity}
          viewerOverridesScene={viewerFaceColorOverridesScene}
          selected={selected?.kind === "face" && selected.id === f.faceId}
          hovered={hovered?.kind === "face" && hovered.id === f.faceId}
          onSelect={onSelect}
          onHover={onHover}
        />
      ))}
    </group>
  );
}

function VertexPoint({
  vertex,
  color,
  size,
  selected,
  hovered,
  onSelect,
  onHover,
}: {
  vertex: VizVertex;
  color: string;
  size: number;
  selected: boolean;
  hovered: boolean;
  onSelect?: (selection: VizSelection) => void;
  onHover?: (selection: VizSelection | null) => void;
}) {
  const displaySize = selected ? size * 1.45 : hovered ? size * 1.25 : size;
  const displayColor = selected ? "#f7e36b" : hovered ? "#69d8ff" : color;
  return (
    <mesh
      position={vertex.position}
      userData={{ kind: "vertex", vertexId: vertex.vertexId }}
      onClick={selecting(onSelect, { kind: "vertex", id: vertex.vertexId })}
      onPointerOver={hovering(onHover, { kind: "vertex", id: vertex.vertexId })}
      onPointerOut={leaving(onHover)}
    >
      <sphereGeometry args={[displaySize, 16, 12]} />
      <meshStandardMaterial
        color={displayColor}
        emissive={selected ? "#3b3300" : hovered ? "#003747" : "#000000"}
      />
    </mesh>
  );
}

function EdgePolyline({
  edge,
  color,
  width,
  selected,
  hovered,
  onSelect,
  onHover,
}: {
  edge: VizEdge;
  color: string;
  width: number;
  selected: boolean;
  hovered: boolean;
  onSelect?: (selection: VizSelection) => void;
  onHover?: (selection: VizSelection | null) => void;
}) {
  const arrow = useMemo(
    () => (edge.arrowHead ? polylineArrowHead(edge.polyline) : null),
    [edge.arrowHead, edge.polyline],
  );
  if (edge.polyline.length < 2) return null;
  const displayColor = selected ? "#f7e36b" : hovered ? "#69d8ff" : color;
  return (
    <group
      userData={{ kind: "edge", edgeId: edge.edgeId }}
      onClick={selecting(onSelect, { kind: "edge", id: edge.edgeId })}
      onPointerOver={hovering(onHover, { kind: "edge", id: edge.edgeId })}
      onPointerOut={leaving(onHover)}
    >
      <Line
        points={edge.polyline}
        color={displayColor}
        lineWidth={selected ? width + 3 : hovered ? width + 2 : width}
      />
      {arrow && (
        <mesh position={arrow.center} quaternion={arrow.quaternion}>
          <coneGeometry args={[arrow.radius, arrow.length, 16]} />
          <meshStandardMaterial color={displayColor} />
        </mesh>
      )}
    </group>
  );
}

function polylineArrowHead(points: Vec3[]) {
  let totalLength = 0;
  for (let index = 1; index < points.length; index++) {
    const previous = points[index - 1];
    const current = points[index];
    totalLength += Math.hypot(
      current[0] - previous[0],
      current[1] - previous[1],
      current[2] - previous[2],
    );
  }
  if (totalLength < 1e-12) return null;

  const end = new THREE.Vector3(...points[points.length - 1]);
  let direction: THREE.Vector3 | null = null;
  for (let index = points.length - 1; index > 0; index--) {
    const previous = new THREE.Vector3(...points[index - 1]);
    const candidate = end.clone().sub(previous);
    if (candidate.lengthSq() > 1e-24) {
      direction = candidate.normalize();
      break;
    }
  }
  if (!direction) return null;

  const length = totalLength * 0.18;
  return {
    center: end.clone().addScaledVector(direction, -length / 2),
    quaternion: new THREE.Quaternion().setFromUnitVectors(
      new THREE.Vector3(0, 1, 0),
      direction,
    ),
    radius: length * 0.4,
    length,
  };
}

function FaceMesh({
  face,
  defaultColor,
  defaultOpacity,
  viewerOverridesScene,
  selected,
  hovered,
  onSelect,
  onHover,
}: {
  face: VizFace;
  defaultColor: string;
  defaultOpacity: number;
  viewerOverridesScene: boolean;
  selected: boolean;
  hovered: boolean;
  onSelect?: (selection: VizSelection) => void;
  onHover?: (selection: VizSelection | null) => void;
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
  const materialOpacity = (face.opacity ?? 1) * defaultOpacity;

  return (
    <mesh
      geometry={geometry}
      userData={{ kind: "face", faceId: face.faceId }}
      onClick={selecting(onSelect, { kind: "face", id: face.faceId })}
      onPointerOver={hovering(onHover, { kind: "face", id: face.faceId })}
      onPointerOut={leaving(onHover)}
    >
      <meshStandardMaterial
        color={selected ? "#f7e36b" : hovered ? "#69d8ff" : materialColor}
        emissive={selected ? "#3b3300" : hovered ? "#003747" : "#000000"}
        opacity={materialOpacity}
        transparent={materialOpacity < 1}
        roughness={0.55}
        metalness={0.08}
        side={face.doubleSided === false ? THREE.FrontSide : THREE.DoubleSide}
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
  selected,
  onSelect,
}: {
  showDarts: boolean;
  darts: VizDart[];
  alphaLinks: VizAlphaLink[];
  dartColor: string;
  arrowHeadRatio: number;
  showDartLabels: boolean;
  alphaColor: (i: number) => string;
  visibleAlphas?: Set<number>;
  selected?: VizSelection | null;
  onSelect?: (selection: VizSelection) => void;
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
            selected={selected?.kind === "dart" && selected.id === d.dartId}
            onSelect={onSelect}
          />
        ))}
      {alphaLinks.map((l, i) =>
        visibleAlphas && !visibleAlphas.has(l.involution) ? null : (
          <AlphaLink
            key={`l-${i}`}
            link={l}
            a={display.alphaEndpoint(l.dartA) ?? l.a}
            b={display.alphaEndpoint(l.dartB) ?? l.b}
            color={alphaColor(l.involution)}
            selected={selected?.kind === "alphaLink" && selected.id === i}
            selectionId={i}
            onSelect={onSelect}
          />
        ),
      )}
    </group>
  );
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
  selected,
  onSelect,
}: {
  dart: VizDart;
  shaft: Vec3[];
  color: string;
  headRatio: number;
  showLabel: boolean;
  selected: boolean;
  onSelect?: (selection: VizSelection) => void;
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
  const displayColor = selected ? "#f7e36b" : color;

  return (
    <group
      userData={{ kind: "dart", dartId: dart.dartId, edgeId: dart.edgeId }}
      onClick={selecting(onSelect, { kind: "dart", id: dart.dartId })}
    >
      <Line points={shaft} color={displayColor} lineWidth={selected ? 3 : 1.5} />
      <mesh position={geom.coneCenter} quaternion={geom.quaternion}>
        <coneGeometry args={[geom.headRadius, geom.headLength, 12]} />
        <meshStandardMaterial
          color={displayColor}
          emissive={selected ? "#3b3300" : "#000000"}
        />
      </mesh>
      {showLabel && dart.label && (
        <Html
          position={geom.labelPos}
          center
          distanceFactor={8}
          style={{
            color: displayColor,
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
  selected,
  selectionId,
  onSelect,
}: {
  link: VizAlphaLink;
  a: Vec3;
  b: Vec3;
  color: string;
  selected: boolean;
  selectionId: number;
  onSelect?: (selection: VizSelection) => void;
}) {
  return (
    <Line
      points={[a, b]}
      color={selected ? "#f7e36b" : color}
      lineWidth={selected ? 8 : 5}
      dashed
      dashSize={0.04}
      gapSize={0.03}
      userData={{
        kind: "alphaLink",
        involution: link.involution,
        dartA: link.dartA,
        dartB: link.dartB,
      }}
      onClick={selecting(onSelect, {
        kind: "alphaLink",
        id: selectionId,
        involution: link.involution,
      })}
    />
  );
}

function selecting(
  onSelect: ((selection: VizSelection) => void) | undefined,
  selection: VizSelection,
) {
  if (!onSelect) return undefined;
  return (event: ThreeEvent<MouseEvent>) => {
    event.stopPropagation();
    onSelect(selection);
  };
}

function hovering(
  onHover: ((selection: VizSelection | null) => void) | undefined,
  selection: VizSelection,
) {
  if (!onHover) return undefined;
  return (event: ThreeEvent<PointerEvent>) => {
    event.stopPropagation();
    onHover(selection);
  };
}

function leaving(onHover: ((selection: VizSelection | null) => void) | undefined) {
  if (!onHover) return undefined;
  return (event: ThreeEvent<PointerEvent>) => {
    event.stopPropagation();
    onHover(null);
  };
}
