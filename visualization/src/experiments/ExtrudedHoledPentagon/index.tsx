import { useEffect, useMemo } from "react";
import VizSceneView from "../../components/VizSceneView";
import { useVizControls } from "../../components/useVizControls";
import { useKernel } from "../../kernel/useKernel";
import { gmapConsoleApi, runScript, type ScriptResult } from "../../kernel/viz";

const SCRIPT_ID = "extruded_holed_pentagon";

export default function ExtrudedHoledPentagon() {
  const kernel = useKernel();
  const controls = useVizControls();

  const result = useMemo<ScriptResult | null>(
    () => (kernel ? runScript(kernel, SCRIPT_ID) : null),
    [kernel],
  );

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
