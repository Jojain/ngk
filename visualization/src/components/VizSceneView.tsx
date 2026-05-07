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
  showDarts?: boolean;
  showDartLabels?: boolean;
  showLabels?: boolean;
  /** Involution index → color for α-links. Missing indices get a default. */
  alphaColors?: Record<number, string>;
  /** If set, only α-links whose `involution` is in this set are drawn. */
  visibleAlphas?: Set<number>;
};

const DEFAULT_ALPHA_COLORS: Record<number, string> = {
  0: "#ff6b6b",
  1: "#4dd0a3",
  2: "#6ea8ff",
  3: "#e4c56e",
};

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

      {showDarts && (
        <GMapLayer
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
  darts,
  alphaLinks,
  dartColor,
  arrowHeadRatio,
  showDartLabels,
  alphaColor,
  visibleAlphas,
}: {
  darts: VizDart[];
  alphaLinks: VizAlphaLink[];
  dartColor: string;
  arrowHeadRatio: number;
  showDartLabels: boolean;
  alphaColor: (i: number) => string;
  visibleAlphas?: Set<number>;
}) {
  return (
    <group>
      {darts.map((d) => (
        <Dart
          key={`d-${d.dartId}`}
          dart={d}
          color={d.color ?? dartColor}
          headRatio={arrowHeadRatio}
          showLabel={showDartLabels}
        />
      ))}
      {alphaLinks.map((l, i) =>
        visibleAlphas && !visibleAlphas.has(l.involution) ? null : (
          <AlphaLink key={`l-${i}`} link={l} color={alphaColor(l.involution)} />
        ),
      )}
    </group>
  );
}

/**
 * One half-edge arrow: a polyline shaft that follows the edge curve plus a
 * cone tip oriented along `tipDir`. Origin = the dart's own vertex (offset);
 * tip = halfway along the edge, pointing toward the α0 vertex.
 */
function Dart({
  dart,
  color,
  headRatio,
  showLabel,
}: {
  dart: VizDart;
  color: string;
  headRatio: number;
  showLabel: boolean;
}) {
  const geom = useMemo(() => {
    const shaft = dart.shaft;
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
  }, [dart, headRatio]);

  if (!geom) return null;

  return (
    <group userData={{ kind: "dart", dartId: dart.dartId, edgeId: dart.edgeId }}>
      <Line points={dart.shaft} color={color} lineWidth={1.5} />
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

function AlphaLink({ link, color }: { link: VizAlphaLink; color: string }) {
  return (
    <Line
      points={[link.a, link.b]}
      color={color}
      lineWidth={1.5}
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
