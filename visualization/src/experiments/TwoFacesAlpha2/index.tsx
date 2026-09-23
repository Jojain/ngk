import { useEffect, useMemo } from "react";
import { VizSceneView } from "@ngk/viewer";
import { useVizControls } from "../../components/useVizControls";
import { useKernel } from "../../kernel/useKernel";
import { gmapConsoleApi, runScript, type ScriptResult } from "../../kernel/viz";

const SCRIPT_ID = "two_faces_alpha2";

export default function TwoFacesAlpha2() {
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
    console.info(
      "[ngk] $gmap ready —",
      `${result.gmap.dartCount} darts, dim ${result.gmap.dimension}.`,
      "Try: $gmap.orbit(0, [1,2]) / $gmap.alpha(2, 2) / $gmap.cellDarts(0, 2)",
    );
    return () => {
      if ((window as unknown as { $gmap?: unknown }).$gmap === api) {
        delete (window as unknown as { $gmap?: unknown }).$gmap;
      }
    };
  }, [result]);

  if (!result) return null;

  return <VizSceneView scene={result.scene} {...controls} />;
}
