import { useRef, useState } from "react";
import * as THREE from "three";
import { ThreeEvent, useThree } from "@react-three/fiber";

type Props = {
  position: [number, number, number];
  color?: string;
  radius?: number;
  /** Plane normal used for drag raycasts. Defaults to world Y (drag on XZ plane). */
  dragPlaneNormal?: [number, number, number];
  onChange: (position: [number, number, number]) => void;
};

/**
 * A draggable sphere. Drag happens on an infinite plane through the handle's
 * current position with the given normal. Left-click-drag.
 */
export default function DraggableHandle({
  position,
  color = "#6ea8ff",
  radius = 0.08,
  dragPlaneNormal = [0, 1, 0],
  onChange,
}: Props) {
  const meshRef = useRef<THREE.Mesh>(null);
  const { camera, gl } = useThree();
  const [dragging, setDragging] = useState(false);
  const [hover, setHover] = useState(false);
  const draggingRef = useRef(false);
  const raycaster = useRef(new THREE.Raycaster());
  const plane = useRef(new THREE.Plane());
  const hit = useRef(new THREE.Vector3());
  const offset = useRef(new THREE.Vector3());

  const onPointerDown = (e: ThreeEvent<PointerEvent>) => {
    if (e.button !== 0) return; // left button only
    e.stopPropagation();
    (e.target as Element).setPointerCapture?.(e.pointerId);
    draggingRef.current = true;
    setDragging(true);
    const n = new THREE.Vector3(...dragPlaneNormal).normalize();
    plane.current.setFromNormalAndCoplanarPoint(n, new THREE.Vector3(...position));
    const pointer = new THREE.Vector2(
      (e.clientX / gl.domElement.clientWidth) * 2 - 1,
      -(e.clientY / gl.domElement.clientHeight) * 2 + 1,
    );
    raycaster.current.setFromCamera(pointer, camera);
    if (raycaster.current.ray.intersectPlane(plane.current, hit.current)) {
      offset.current.copy(new THREE.Vector3(...position)).sub(hit.current);
    }
  };

  const onPointerMove = (e: ThreeEvent<PointerEvent>) => {
    if (!draggingRef.current) return;
    e.stopPropagation();
    const pointer = new THREE.Vector2(
      (e.clientX / gl.domElement.clientWidth) * 2 - 1,
      -(e.clientY / gl.domElement.clientHeight) * 2 + 1,
    );
    raycaster.current.setFromCamera(pointer, camera);
    if (raycaster.current.ray.intersectPlane(plane.current, hit.current)) {
      const p = hit.current.clone().add(offset.current);
      onChange([p.x, p.y, p.z]);
    }
  };

  const endDrag = (e: ThreeEvent<PointerEvent>) => {
    if (!draggingRef.current) return;
    e.stopPropagation();
    (e.target as Element).releasePointerCapture?.(e.pointerId);
    draggingRef.current = false;
    setDragging(false);
  };

  return (
    <mesh
      ref={meshRef}
      position={position}
      onPointerDown={onPointerDown}
      onPointerMove={onPointerMove}
      onPointerUp={endDrag}
      onPointerCancel={endDrag}
      onPointerOver={(e) => {
        e.stopPropagation();
        setHover(true);
      }}
      onPointerOut={() => setHover(false)}
    >
      <sphereGeometry args={[radius, 20, 20]} />
      <meshStandardMaterial
        color={color}
        emissive={hover || dragging ? color : "#000000"}
        emissiveIntensity={hover || dragging ? 0.6 : 0}
        roughness={0.3}
        metalness={0.1}
      />
    </mesh>
  );
}
