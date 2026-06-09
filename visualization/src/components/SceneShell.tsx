import { Canvas } from "@react-three/fiber";
import { GizmoHelper, GizmoViewport, OrbitControls } from "@react-three/drei";
import * as THREE from "three";
import type { PropsWithChildren } from "react";

export default function SceneShell({ children }: PropsWithChildren) {
  return (
    <Canvas
      camera={{ position: [5, 5, 5], fov: 45, near: 0.01, far: 1000 }}
      dpr={[1, 2]}
      gl={{ antialias: true }}
    >
      <color attach="background" args={["#0f0f12"]} />
      <ambientLight intensity={0.6} />
      <directionalLight position={[8, 10, 6]} intensity={1.1} />
      <OrbitControls
        makeDefault
        enableDamping
        dampingFactor={0.15}
        mouseButtons={{
          LEFT: undefined as unknown as THREE.MOUSE,
          MIDDLE: THREE.MOUSE.ROTATE,
          RIGHT: THREE.MOUSE.PAN,
        }}
      />
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
