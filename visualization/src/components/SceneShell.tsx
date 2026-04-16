import { Canvas } from "@react-three/fiber";
import { Grid, OrbitControls } from "@react-three/drei";
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
      <Grid
        args={[20, 20]}
        cellSize={0.5}
        cellThickness={0.5}
        cellColor="#2a2a34"
        sectionSize={2}
        sectionThickness={1}
        sectionColor="#3a3a48"
        fadeDistance={30}
        fadeStrength={1}
        infiniteGrid
      />
      <axesHelper args={[1]} />
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
      {children}
    </Canvas>
  );
}
