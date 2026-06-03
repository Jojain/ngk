import { useEffect, useMemo, useState } from "react";
import { Html } from "@react-three/drei";
import { useControls } from "leva";
import VizSceneView from "../../components/VizSceneView";
import { useVizControls } from "../../components/useVizControls";
import { useKernel, type Kernel } from "../../kernel/useKernel";
import { gmapConsoleApi, type ScriptResult } from "../../kernel/viz";

type BooleanConfigId =
  | "union-overlap"
  | "intersection-overlap"
  | "difference-overlap";

type BooleanKernel = Kernel & {
  booleanConfiguration: (config: BooleanConfigId) => ScriptResult;
};

const CONFIG_OPTIONS: Record<string, BooleanConfigId> = {
  "Union - overlapping blocks": "union-overlap",
  "Intersection - overlapping blocks": "intersection-overlap",
  "Difference - overlapping blocks": "difference-overlap",
};

export default function BooleanOperations() {
  const kernel = useKernel() as BooleanKernel | null;
  const [configuration, setConfiguration] =
    useState<BooleanConfigId>("union-overlap");
  const controls = useVizControls({
    showVertices: true,
    showEdges: true,
    showFaces: true,
    showDarts: false,
    showDartLabels: false,
    viewerFaceColorOverridesScene: false,
  });

  useControls("Boolean", {
    configuration: {
      value: configuration,
      options: CONFIG_OPTIONS,
      onChange: (value: BooleanConfigId) => setConfiguration(value),
    },
  });

  const state = useMemo<{ result: ScriptResult | null; error: string | null }>(() => {
    if (!kernel?.booleanConfiguration) return { result: null, error: null };
    try {
      return {
        result: kernel.booleanConfiguration(configuration),
        error: null,
      };
    } catch (error) {
      console.error("booleanConfiguration failed", error);
      return {
        result: null,
        error: formatError(error),
      };
    }
  }, [kernel, configuration]);
  const { result, error } = state;

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
        <span>{error ?? "Boolean configuration is still loading."}</span>
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
