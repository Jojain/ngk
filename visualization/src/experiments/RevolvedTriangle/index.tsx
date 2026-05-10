import { useEffect, useMemo, useState } from "react";
import { useControls } from "leva";
import VizSceneView from "../../components/VizSceneView";
import { useVizControls } from "../../components/useVizControls";
import { useKernel, type Kernel } from "../../kernel/useKernel";
import { gmapConsoleApi, type ScriptResult } from "../../kernel/viz";

type RevolveKernel = Kernel & {
  revolveTriangle: (angleRadians: number) => ScriptResult;
};

export default function RevolvedTriangle() {
  const kernel = useKernel() as RevolveKernel | null;
  const [angleDegrees, setAngleDegrees] = useState(90);
  const controls = useVizControls({
    showDartLabels: false,
    showVertices: true,
    showEdges: true,
    showFaces: true,
  });

  useControls("Triangle revolution", {
    angle: {
      value: angleDegrees,
      min: 0,
      max: 360,
      step: 1,
      onChange: (value: number) => setAngleDegrees(value),
    },
  });

  const result = useMemo<ScriptResult | null>(() => {
    if (!kernel?.revolveTriangle) return null;
    return kernel.revolveTriangle((angleDegrees * Math.PI) / 180);
  }, [kernel, angleDegrees]);

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

  return <VizSceneView scene={result.scene} {...controls} />;
}
