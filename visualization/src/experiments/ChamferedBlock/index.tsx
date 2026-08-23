import { useEffect, useMemo, useState } from "react";
import { useControls } from "leva";
import VizSceneView from "../../components/VizSceneView";
import { useVizControls } from "../../components/useVizControls";
import { useKernel, type Kernel } from "../../kernel/useKernel";
import {
  gmapConsoleApi,
  runScript,
  type ScriptResult,
} from "../../kernel/viz";

const SCRIPT_ID = "chamfered_block";

type ChamferKernel = Kernel & {
  chamferBlock: (distance: number) => ScriptResult;
};

export default function ChamferedBlock() {
  const kernel = useKernel() as ChamferKernel | null;
  const [distance, setDistance] = useState(0.45);
  const controls = useVizControls();

  useControls("3D chamfer", {
    distance: {
      value: distance,
      min: 0.05,
      max: 1.5,
      step: 0.05,
      onChange: (value: number) => setDistance(value),
    },
  });

  const result = useMemo<ScriptResult | null>(() => {
    if (!kernel) return null;
    return kernel.chamferBlock
      ? kernel.chamferBlock(distance)
      : runScript(kernel, SCRIPT_ID);
  }, [kernel, distance]);

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
