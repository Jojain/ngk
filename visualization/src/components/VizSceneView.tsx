import { useMemo } from "react";
import * as THREE from "three";
import { Html, Line } from "@react-three/drei";
import type { VizArrow, VizLink, VizScene } from "../kernel/viz";

export type VizSceneViewProps = {
  scene: VizScene;
  pointColor?: string;
  segmentColor?: string;
  arrowColor?: string;
  pointSize?: number;
  arrowHeadRatio?: number;
  showDartLabels?: boolean;
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
 * Generic renderer for a `VizScene`. Shows points as small spheres, segments
 * as lines, arrows as line + cone tip, and α-involution links as colored
 * lines between dart-arrow midpoints.
 */
export default function VizSceneView({
  scene,
  pointColor = "#ffc857",
  segmentColor = "#9aa0a6",
  arrowColor = "#cfd2d6",
  pointSize = 0.04,
  arrowHeadRatio = 0.28,
  showDartLabels = false,
  alphaColors = DEFAULT_ALPHA_COLORS,
  visibleAlphas,
}: VizSceneViewProps) {
  const alphaColor = (i: number) =>
    alphaColors[i] ?? DEFAULT_ALPHA_COLORS[i] ?? "#bbbbbb";

  return (
    <group>
      {scene.points.map((p, i) => (
        <mesh key={`p-${i}`} position={p.position}>
          <sphereGeometry args={[p.size ?? pointSize, 16, 12]} />
          <meshStandardMaterial color={p.color ?? pointColor} />
        </mesh>
      ))}

      {scene.segments.map((s, i) => (
        <Line
          key={`s-${i}`}
          points={[s.start, s.end]}
          color={s.color ?? segmentColor}
          lineWidth={s.width ?? 1.5}
        />
      ))}

      {scene.arrows.map((a, i) => (
        <Arrow
          key={`a-${i}`}
          arrow={a}
          color={a.color ?? arrowColor}
          headRatio={arrowHeadRatio}
          showLabel={showDartLabels}
        />
      ))}

      {scene.alphaLinks.map((l, i) =>
        visibleAlphas && !visibleAlphas.has(l.involution) ? null : (
          <AlphaLink key={`l-${i}`} link={l} color={alphaColor(l.involution)} />
        ),
      )}

      {scene.labels.map((l, i) => (
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

function Arrow({
  arrow,
  color,
  headRatio,
  showLabel,
}: {
  arrow: VizArrow;
  color: string;
  headRatio: number;
  showLabel: boolean;
}) {
  const geom = useMemo(() => {
    const origin = new THREE.Vector3(...arrow.origin);
    const tip = new THREE.Vector3(...arrow.tip);
    const dir = new THREE.Vector3().subVectors(tip, origin);
    const length = dir.length();
    if (length < 1e-12) return null;
    const dirHat = dir.clone().multiplyScalar(1 / length);
    const headLength = Math.min(length * headRatio, length * 0.9);
    const headRadius = headLength * 0.4;
    const shaftEnd = origin
      .clone()
      .addScaledVector(dirHat, Math.max(length - headLength, 0));
    const coneCenter = shaftEnd
      .clone()
      .addScaledVector(dirHat, headLength / 2);
    const quaternion = new THREE.Quaternion().setFromUnitVectors(
      new THREE.Vector3(0, 1, 0),
      dirHat,
    );
    return { origin, shaftEnd, coneCenter, quaternion, headRadius, headLength };
  }, [arrow, headRatio]);

  if (!geom) return null;

  return (
    <group>
      <Line
        points={[
          [geom.origin.x, geom.origin.y, geom.origin.z],
          [geom.shaftEnd.x, geom.shaftEnd.y, geom.shaftEnd.z],
        ]}
        color={color}
        lineWidth={1.5}
      />
      <mesh position={geom.coneCenter} quaternion={geom.quaternion}>
        <coneGeometry args={[geom.headRadius, geom.headLength, 12]} />
        <meshStandardMaterial color={color} />
      </mesh>
      {showLabel && arrow.label && (
        <Html
          position={arrow.tip}
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
          {arrow.label}
        </Html>
      )}
    </group>
  );
}

function AlphaLink({ link, color }: { link: VizLink; color: string }) {
  return (
    <Line
      points={[link.a, link.b]}
      color={color}
      lineWidth={1.5}
      dashed
      dashSize={0.04}
      gapSize={0.03}
    />
  );
}
