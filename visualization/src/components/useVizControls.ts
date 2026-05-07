import { useMemo } from "react";
import { useControls, folder } from "leva";

/**
 * Shared leva controls for any experiment that renders a `VizScene`.
 *
 * Returns the props you can spread onto `<VizSceneView />`. Two folders:
 * - **BRep**: per-entity-type toggles (vertices, edges, faces).
 * - **GMap**: dart visibility, dart labels, per-involution α-link toggles.
 *
 * Override defaults via `initial` for experiment-specific tuning.
 */
export type VizControlsProps = {
  showVertices: boolean;
  showEdges: boolean;
  showFaces: boolean;
  showDarts: boolean;
  showDartLabels: boolean;
  visibleAlphas: Set<number>;
  alphaColors: Record<number, string>;
};

export type VizControlsInitial = Partial<{
  showVertices: boolean;
  showEdges: boolean;
  showFaces: boolean;
  showDarts: boolean;
  showDartLabels: boolean;
  showAlpha0: boolean;
  showAlpha1: boolean;
  showAlpha2: boolean;
  showAlpha3: boolean;
  alpha0Color: string;
  alpha1Color: string;
  alpha2Color: string;
  alpha3Color: string;
}>;

export function useVizControls(initial: VizControlsInitial = {}): VizControlsProps {
  const values = useControls("Viz", {
    BRep: folder({
      showVertices: { value: initial.showVertices ?? true, label: "vertices" },
      showEdges: { value: initial.showEdges ?? true, label: "edges" },
      showFaces: { value: initial.showFaces ?? true, label: "faces" },
    }),
    GMap: folder({
      showDarts: { value: initial.showDarts ?? true, label: "darts" },
      showDartLabels: {
        value: initial.showDartLabels ?? false,
        label: "dart labels",
      },
      showAlpha0: { value: initial.showAlpha0 ?? true, label: "α0 links" },
      showAlpha1: { value: initial.showAlpha1 ?? true, label: "α1 links" },
      showAlpha2: { value: initial.showAlpha2 ?? true, label: "α2 links" },
      showAlpha3: { value: initial.showAlpha3 ?? false, label: "α3 links" },
      alpha0Color: { value: initial.alpha0Color ?? "#ff6b6b", label: "α0 color" },
      alpha1Color: { value: initial.alpha1Color ?? "#4dd0a3", label: "α1 color" },
      alpha2Color: { value: initial.alpha2Color ?? "#6ea8ff", label: "α2 color" },
      alpha3Color: { value: initial.alpha3Color ?? "#e4c56e", label: "α3 color" },
    }),
  });

  const visibleAlphas = useMemo(() => {
    const s = new Set<number>();
    if (values.showAlpha0) s.add(0);
    if (values.showAlpha1) s.add(1);
    if (values.showAlpha2) s.add(2);
    if (values.showAlpha3) s.add(3);
    return s;
  }, [
    values.showAlpha0,
    values.showAlpha1,
    values.showAlpha2,
    values.showAlpha3,
  ]);

  const alphaColors = useMemo(
    () => ({
      0: values.alpha0Color,
      1: values.alpha1Color,
      2: values.alpha2Color,
      3: values.alpha3Color,
    }),
    [
      values.alpha0Color,
      values.alpha1Color,
      values.alpha2Color,
      values.alpha3Color,
    ],
  );

  return {
    showVertices: values.showVertices,
    showEdges: values.showEdges,
    showFaces: values.showFaces,
    showDarts: values.showDarts,
    showDartLabels: values.showDartLabels,
    visibleAlphas,
    alphaColors,
  };
}
