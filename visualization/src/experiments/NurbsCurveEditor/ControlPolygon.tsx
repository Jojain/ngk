import { Line } from "@react-three/drei";
import * as THREE from "three";
import type { Vec3 } from "../../kernel/nurbs";

type Props = {
  points: Vec3[];
  color?: string;
};

export default function ControlPolygon({ points, color = "#6e6e78" }: Props) {
  if (points.length < 2) return null;
  const v3 = points.map((p) => new THREE.Vector3(...p));
  return <Line points={v3} color={color} dashed dashSize={0.15} gapSize={0.08} lineWidth={1} />;
}
