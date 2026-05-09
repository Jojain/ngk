import { useEffect, useMemo, useState } from "react";
import { Html } from "@react-three/drei";
import { useControls } from "leva";
import VizSceneView from "../../components/VizSceneView";
import { useVizControls } from "../../components/useVizControls";
import { useKernel, type Kernel } from "../../kernel/useKernel";
import { gmapConsoleApi, type ScriptResult } from "../../kernel/viz";

type BlockKernel = Kernel & {
  createBlock: (xSize: number, ySize: number, zSize: number) => ScriptResult;
};

export default function BlockPrimitive() {
  const kernel = useKernel() as BlockKernel | null;
  const [size, setSize] = useState<[number, number, number]>([1.6, 1.1, 1.3]);
  const controls = useVizControls({
    showDartLabels: false,
    showVertices: true,
    showEdges: true,
    showFaces: true,
  });

  useControls("Block primitive", {
    x: {
      value: size[0],
      min: -1,
      max: 4,
      step: 0.1,
      onChange: (value: number) => setSize((s) => [value, s[1], s[2]]),
    },
    y: {
      value: size[1],
      min: 0.1,
      max: 4,
      step: 0.1,
      onChange: (value: number) => setSize((s) => [s[0], value, s[2]]),
    },
    z: {
      value: size[2],
      min: 0.1,
      max: 4,
      step: 0.1,
      onChange: (value: number) => setSize((s) => [s[0], s[1], value]),
    },
  });

  const blockState = useMemo<{ result: ScriptResult | null; error: string | null }>(() => {
    if (!kernel?.createBlock) return { result: null, error: null };
    try {
      return {
        result: kernel.createBlock(size[0], size[1], size[2]),
        error: null,
      };
    } catch (error) {
      console.error("createBlock failed", error);
      return {
        result: null,
        error: formatError(error),
      };
    }
  }, [kernel, size]);
  const { result, error } = blockState;

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

  if (!result) {
    return (
      <Html center className="experiment-error experiment-error-canvas" role="alert">
        <strong>Experiment error</strong>
        <span>{error ?? "Block dimensions must produce a valid solid."}</span>
      </Html>
    );
  }

  return <VizSceneView scene={result.scene} {...controls} />;
}

function formatError(error: unknown): string {
  if (error instanceof Error) return error.message;
  if (typeof error === "string") return error;
  try {
    return JSON.stringify(error);
  } catch {
    return String(error);
  }
}
