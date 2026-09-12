import { Canvas } from "@react-three/fiber";
import { GizmoHelper, GizmoViewport } from "@react-three/drei";
import type { PropsWithChildren } from "react";
import CadControlsView from "./CadControlsView";

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
      <CadControlsView
        makeDefault
        showGizmos={false}
        rotateSpeed={1.5}
        scaleFactor={1.25}
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
