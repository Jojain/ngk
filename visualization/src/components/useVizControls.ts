import { useMemo } from "react";
import { useControls, folder } from "leva";

/**
 * Shared leva controls for any experiment that renders a `VizScene`.
 *
 * Returns the props you can spread onto `<VizSceneView />`. Two folders:
 * - **BRep**: toggles, sizes, colors, and whether the face color picker
 *   overrides per-face colors from the scene.
 * - **GMap**: dart visibility, dart labels, per-involution α-link toggles.
 *
 * Override defaults via `initial` for experiment-specific tuning.
 */
export type VizControlsProps = {
  showVertices: boolean;
  showEdges: boolean;
  showFaces: boolean;
  /** Sphere radius for vertex markers (world units). */
  vertexSize: number;
  /** Line width for BRep edge polylines (`@react-three/drei` Line). */
  edgeWidth: number;
  /** Default BRep colors when the scene entity omits `color`. */
  vertexColor: string;
  edgeColor: string;
  faceColor: string;
  /**
   * When true (default), every face uses the panel `faceColor`. Turn off to
   * use per-face `color` from the scene when set (`VizHints`), with `faceColor`
   * as fallback only.
   */
  viewerFaceColorOverridesScene: boolean;
  showDarts: boolean;
  showDartLabels: boolean;
  visibleAlphas: Set<number>;
  alphaColors: Record<number, string>;
};

export type VizControlsInitial = Partial<{
  showVertices: boolean;
  showEdges: boolean;
  showFaces: boolean;
  vertexSize: number;
  edgeWidth: number;
  vertexColor: string;
  edgeColor: string;
  faceColor: string;
  viewerFaceColorOverridesScene: boolean;
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
      vertexSize: {
        value: initial.vertexSize ?? 0.06,
        min: 0.01,
        max: 0.3,
        step: 0.005,
        label: "vertex size",
      },
      edgeWidth: {
        value: initial.edgeWidth ?? 4,
        min: 0.5,
        max: 16,
        step: 0.25,
        label: "edge width",
      },
      vertexColor: {
        value: initial.vertexColor ?? "#ffc857",
        label: "vertex color",
      },
      edgeColor: {
        value: initial.edgeColor ?? "#9aa0a6",
        label: "edge color",
      },
      faceColor: {
        value: initial.faceColor ?? "#4a7bc8",
        label: "face color",
      },
      viewerFaceColorOverridesScene: {
        value: initial.viewerFaceColorOverridesScene ?? true,
        label: "face color overrides scene",
      },
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
    vertexSize: values.vertexSize,
    edgeWidth: values.edgeWidth,
    vertexColor: values.vertexColor,
    edgeColor: values.edgeColor,
    faceColor: values.faceColor,
    viewerFaceColorOverridesScene: values.viewerFaceColorOverridesScene,
    showDarts: values.showDarts,
    showDartLabels: values.showDartLabels,
    visibleAlphas,
    alphaColors,
  };
}
