import { useEffect, useMemo } from "react";
import VizSceneView from "../../components/VizSceneView";
import { useVizControls } from "../../components/useVizControls";
import { useKernel } from "../../kernel/useKernel";
import { gmapConsoleApi, runScript, type ScriptResult } from "../../kernel/viz";

const SCRIPT_ID = "cylinder";

/**
 * Smallest scene that exercises the curved-surface tessellation path
 * (`Surface::Cylinder`) without any α2-sewing. The four edges sit on a
 * cylinder, so two of them are arcs in 3D — the dart shafts on those
 * edges should visibly curve.
 */
export default function Cylinder() {
  const kernel = useKernel();
  const controls = useVizControls({ showDartLabels: true });

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
