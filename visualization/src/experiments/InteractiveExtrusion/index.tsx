import { useEffect, useMemo, useRef, useState } from "react";
import { Line } from "@react-three/drei";
import { useThree, type ThreeEvent } from "@react-three/fiber";
import { useControls } from "leva";
import * as THREE from "three";
import VizSceneView from "../../components/VizSceneView";
import { useVizControls } from "../../components/useVizControls";
import { useKernel, type Kernel } from "../../kernel/useKernel";
import {
  gmapConsoleApi,
  type ScriptResult,
  type Vec3,
} from "../../kernel/viz";

const ARROW_COLOR = "#ffb454";
const HANDLE_RADIUS = 0.11;

type ExtrusionKernel = Kernel & {
  extrudePolygon: (
    pointCount: number,
    extrusionX: number,
    extrusionY: number,
    extrusionZ: number,
  ) => ScriptResult;
};

export default function InteractiveExtrusion() {
  const kernel = useKernel() as ExtrusionKernel | null;
  const [pointCount, setPointCount] = useState(5);
  const [extrusion, setExtrusion] = useState<Vec3>([0.6, 0.2, 1.8]);
  const vizControls = useVizControls({
    showDartLabels: false,
    showVertices: true,
    showEdges: true,
    showFaces: true,
  });

  useControls("Interactive extrusion", {
    points: {
      value: pointCount,
      min: 3,
      max: 12,
      step: 1,
      onChange: (value: number) => setPointCount(value),
    },
  });

  const result = useMemo<ScriptResult | null>(() => {
    if (!kernel?.extrudePolygon) return null;
    return kernel.extrudePolygon(pointCount, extrusion[0], extrusion[1], extrusion[2]);
  }, [kernel, pointCount, extrusion]);

  const center = useMemo(() => sceneCenter(result), [result]);
  const end = add(center, extrusion);

  useEffect(() => {
    if (!result?.gmap) return;
    const api = gmapConsoleApi(result.gmap);
    (window as unknown as { $gmap?: unknown }).$gmap = api;
    return () => {
      if ((window as unknown as { $gmap?: unknown }).$gmap === api) {
        delete (window as unknown as { $gmap?: unknown }).$gmap;
      }
    };
  }, [result]);

  if (!result) return null;

  return (
    <group>
      <VizSceneView scene={result.scene} {...vizControls} />
      <ExtrusionArrow
        start={center}
        end={end}
        onChange={(nextEnd) => setExtrusion(sub(nextEnd, center))}
      />
    </group>
  );
}

function ExtrusionArrow({
  start,
  end,
  onChange,
}: {
  start: Vec3;
  end: Vec3;
  onChange: (end: Vec3) => void;
}) {
  const { camera, gl } = useThree();
  const dragging = useRef(false);
  const raycaster = useRef(new THREE.Raycaster());
  const plane = useRef(new THREE.Plane());
  const hit = useRef(new THREE.Vector3());
  const offset = useRef(new THREE.Vector3());
  const direction = sub(end, start);
  const length = norm(direction);
  const unit = length > 1e-9 ? scale(direction, 1 / length) : ([0, 0, 1] as Vec3);
  const coneCenter = add(end, scale(unit, -0.16));
  const quaternion = useMemo(() => {
    return new THREE.Quaternion().setFromUnitVectors(
      new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(...unit),
    );
  }, [unit[0], unit[1], unit[2]]);

  const pointerToPlane = (event: ThreeEvent<PointerEvent>) => {
    const rect = gl.domElement.getBoundingClientRect();
    const pointer = new THREE.Vector2(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1,
    );
    raycaster.current.setFromCamera(pointer, camera);
    return raycaster.current.ray.intersectPlane(plane.current, hit.current);
  };

  const startDrag = (event: ThreeEvent<PointerEvent>) => {
    if (event.button !== 0) return;
    event.stopPropagation();
    (event.target as Element).setPointerCapture?.(event.pointerId);
    dragging.current = true;
    const normal = camera.getWorldDirection(new THREE.Vector3()).normalize();
    plane.current.setFromNormalAndCoplanarPoint(normal, new THREE.Vector3(...end));
    if (pointerToPlane(event)) {
      offset.current.copy(new THREE.Vector3(...end)).sub(hit.current);
    }
  };

  const moveDrag = (event: ThreeEvent<PointerEvent>) => {
    if (!dragging.current) return;
    event.stopPropagation();
    if (pointerToPlane(event)) {
      const next = hit.current.clone().add(offset.current);
      onChange([next.x, next.y, Math.max(0.15, next.z)]);
    }
  };

  const stopDrag = (event: ThreeEvent<PointerEvent>) => {
    if (!dragging.current) return;
    event.stopPropagation();
    (event.target as Element).releasePointerCapture?.(event.pointerId);
    dragging.current = false;
  };

  return (
    <group>
      <Line points={[start, end]} color={ARROW_COLOR} lineWidth={4} />
      <mesh position={coneCenter} quaternion={quaternion}>
        <coneGeometry args={[0.14, 0.32, 24]} />
        <meshStandardMaterial
          color={ARROW_COLOR}
          emissive={ARROW_COLOR}
          emissiveIntensity={0.25}
        />
      </mesh>
      <mesh
        position={end}
        onPointerDown={startDrag}
        onPointerMove={moveDrag}
        onPointerUp={stopDrag}
        onPointerCancel={stopDrag}
      >
        <sphereGeometry args={[HANDLE_RADIUS, 24, 16]} />
        <meshStandardMaterial
          color="#ffd28c"
          emissive={ARROW_COLOR}
          emissiveIntensity={0.45}
        />
      </mesh>
    </group>
  );
}

function sceneCenter(result: ScriptResult | null): Vec3 {
  const points = result?.scene.vertices.map((vertex) => vertex.position) ?? [];
  if (points.length === 0) return [0, 0, 0];
  let out: Vec3 = [0, 0, 0];
  for (const point of points) out = add(out, point);
  return scale(out, 1 / points.length);
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

function norm(a: Vec3): number {
  return Math.hypot(a[0], a[1], a[2]);
}
