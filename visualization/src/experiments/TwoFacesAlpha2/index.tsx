import { useEffect, useMemo } from "react";
import { useControls } from "leva";
import VizSceneView from "../../components/VizSceneView";
import { useKernel } from "../../kernel/useKernel";
import { gmapConsoleApi, runScript, type ScriptResult } from "../../kernel/viz";

const SCRIPT_ID = "two_faces_alpha2";

export default function TwoFacesAlpha2() {
  const kernel = useKernel();

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

  const { showDartLabels, showAlpha0, showAlpha1, showAlpha2, alpha0Color, alpha1Color, alpha2Color } =
    useControls("GMap view", {
      showDartLabels: { value: true, label: "dart labels" },
      showAlpha0: { value: true, label: "α0 links" },
      showAlpha1: { value: true, label: "α1 links" },
      showAlpha2: { value: true, label: "α2 links" },
      alpha0Color: { value: "#ff6b6b", label: "α0 color" },
      alpha1Color: { value: "#4dd0a3", label: "α1 color" },
      alpha2Color: { value: "#6ea8ff", label: "α2 color" },
    });

  const visibleAlphas = useMemo(() => {
    const s = new Set<number>();
    if (showAlpha0) s.add(0);
    if (showAlpha1) s.add(1);
    if (showAlpha2) s.add(2);
    return s;
  }, [showAlpha0, showAlpha1, showAlpha2]);

  const alphaColors = useMemo(
    () => ({ 0: alpha0Color, 1: alpha1Color, 2: alpha2Color }),
    [alpha0Color, alpha1Color, alpha2Color],
  );

  if (!result) return null;

  return (
    <VizSceneView
      scene={result.scene}
      showDartLabels={showDartLabels}
      visibleAlphas={visibleAlphas}
      alphaColors={alphaColors}
    />
  );
}
