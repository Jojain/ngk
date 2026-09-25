import { useEffect, useMemo, useState } from "react";
import { useControls } from "leva";
import { VizSceneView } from "@ngk/viewer";
import { useVizControls } from "../../components/useVizControls";
import { useKernel, type Kernel } from "../../kernel/useKernel";
import {
  gmapConsoleApi,
  runScript,
  type ScriptResult,
} from "../../kernel/viz";

const SCRIPT_ID = "filleted_block";

type FilletKernel = Kernel & {
  filletBlock: (radius: number) => ScriptResult;
};

export default function FilletedBlock() {
  const kernel = useKernel() as FilletKernel | null;
  const [radius, setRadius] = useState(0.3);
  const controls = useVizControls();

  useControls("3D fillet", {
    radius: {
      value: radius,
      min: 0.05,
      max: 0.35,
      step: 0.05,
      onChange: (value: number) => setRadius(value),
    },
  });

  const result = useMemo<ScriptResult | null>(() => {
    if (!kernel) return null;
    return kernel.filletBlock
      ? kernel.filletBlock(radius)
      : runScript(kernel, SCRIPT_ID);
  }, [kernel, radius]);

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
