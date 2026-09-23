import { Canvas, useThree } from "@react-three/fiber";
import { GizmoHelper, GizmoViewport } from "@react-three/drei";
import { useEffect, useMemo, type PropsWithChildren } from "react";
import { Box3, Vector3, type PerspectiveCamera } from "three";
import CadControlsView from "./CadControlsView";
import type { VizScene, Vec3 } from "./viz";

type SceneShellProps = PropsWithChildren<{ scene?: VizScene | null }>;

export default function SceneShell({ children, scene }: SceneShellProps) {
  const bounds = useMemo(() => sceneBounds(scene), [scene]);
  return (
    <Canvas
      camera={{ position: [5, 5, 5], fov: 45, near: 0.01, far: 1000 }}
      dpr={[1, 2]}
      gl={{ antialias: true }}
    >
      <color attach="background" args={["#0f0f12"]} />
      <ambientLight intensity={0.6} />
      <directionalLight position={[8, 10, 6]} intensity={1.1} />
      <CadControlsView
        makeDefault
        showGizmos={false}
        rotateSpeed={1.5}
        scaleFactor={1.25}
        target={bounds?.center}
      />
      {bounds && <CameraFit center={bounds.center} radius={bounds.radius} />}
      <GizmoHelper alignment="bottom-left" margin={[80, 80]}>
        <GizmoViewport
          axisColors={["#ff5f5f", "#62d26f", "#6ea8ff"]}
          labelColor="#f2f2f4"
        />
      </GizmoHelper>
      {children}
    </Canvas>
  );
}

function sceneBounds(scene?: VizScene | null): { center: Vec3; radius: number } | null {
  if (!scene) return null;
  const box = new Box3();
  const include = (point: Vec3) => box.expandByPoint(new Vector3(...point));
  scene.vertices.forEach((vertex) => include(vertex.position));
  scene.edges.forEach((edge) => edge.polyline.forEach(include));
  scene.faces.forEach((face) => face.positions.forEach(include));
  if (box.isEmpty()) return null;
  const center = box.getCenter(new Vector3());
  const radius = Math.max(box.getSize(new Vector3()).length() / 2, 0.1);
  return { center: [center.x, center.y, center.z], radius };
}

function CameraFit({ center, radius }: { center: Vec3; radius: number }) {
  const camera = useThree((state) => state.camera) as PerspectiveCamera;
  const invalidate = useThree((state) => state.invalidate);
  useEffect(() => {
    const vertical = (camera.fov * Math.PI) / 180;
    const horizontal = 2 * Math.atan(Math.tan(vertical / 2) * camera.aspect);
    const distance = (radius / Math.sin(Math.min(vertical, horizontal) / 2)) * 1.35;
    const target = new Vector3(...center);
    camera.position.copy(target).add(new Vector3(1, 1, 1).normalize().multiplyScalar(distance));
    camera.near = Math.max(0.01, distance - radius * 3);
    camera.far = Math.max(1000, distance + radius * 3);
    camera.lookAt(target);
    camera.updateProjectionMatrix();
    invalidate();
  }, [camera, center, radius, invalidate]);
  return null;
}
