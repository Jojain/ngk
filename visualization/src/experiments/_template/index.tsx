import { useControls } from "leva";
import { useKernel } from "../../kernel/useKernel";

/**
 * Minimal experiment template. Copy this folder, rename it, and register the
 * new entry in `visualization/src/experiments/registry.ts`.
 *
 * Anything returned here is mounted as a child of the shared R3F `<Canvas>`
 * (see `components/SceneShell.tsx`) — return three.js JSX (meshes, lines, etc.).
 */
export default function Template() {
  const kernel = useKernel();
  const { color, size } = useControls("Template", {
    color: "#6ea8ff",
    size: { value: 0.5, min: 0.05, max: 2, step: 0.01 },
  });

  if (!kernel) return null;

  return (
    <mesh>
      <boxGeometry args={[size, size, size]} />
      <meshStandardMaterial color={color} />
    </mesh>
  );
}
