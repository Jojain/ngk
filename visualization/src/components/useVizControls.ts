import { useMemo } from "react";
import { button, useControls, folder } from "leva";

/**
 * Shared leva controls for any experiment that renders a `VizScene`.
 */
export type VizControlsProps = {
  showWorldFrame: boolean;
  showVertices: boolean;
  showEdges: boolean;
  showFaces: boolean;
  vertexSize: number;
  edgeWidth: number;
  vertexColor: string;
  edgeColor: string;
  faceColor: string;
  faceOpacity: number;
  viewerFaceColorOverridesScene: boolean;
  showDarts: boolean;
  showDartLabels: boolean;
  showAlphaLinks: boolean;
  visibleAlphas: Set<number>;
  alphaColors: Record<number, string>;
};

export type VizControlsInitial = Partial<{
  showWorldFrame: boolean;
  showVertices: boolean;
  showEdges: boolean;
  showFaces: boolean;
  vertexSize: number;
  edgeWidth: number;
  vertexColor: string;
  edgeColor: string;
  faceColor: string;
  faceOpacity: number;
  viewerFaceColorOverridesScene: boolean;
  showDarts: boolean;
  showDartLabels: boolean;
  showAlphaLinks: boolean;
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
  const [values, set] = useControls("Viz", () => ({
    Scene: folder({
      showWorldFrame: {
        value: initial.showWorldFrame ?? true,
        label: "world XYZ frame",
      },
    }),
    BRep: folder({
      showVertices: { value: initial.showVertices ?? true, label: "vertices" },
      showEdges: { value: initial.showEdges ?? true, label: "edges" },
      showFaces: { value: initial.showFaces ?? true, label: "faces" },
    }),
    Options: folder({
      vertexSize: {
        value: initial.vertexSize ?? 0.04,
        min: 0.01,
        max: 0.3,
        step: 0.005,
        label: "vertex size",
      },
      edgeWidth: {
        value: initial.edgeWidth ?? 6,
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
      faceOpacity: {
        value: initial.faceOpacity ?? 1,
        min: 0,
        max: 1,
        step: 0.02,
        label: "face opacity",
      },
      viewerFaceColorOverridesScene: {
        value: initial.viewerFaceColorOverridesScene ?? true,
        label: "face color overrides scene",
      },
      alpha0Color: { value: initial.alpha0Color ?? "#ff1744", label: "alpha0 color" },
      alpha1Color: { value: initial.alpha1Color ?? "#00e676", label: "alpha1 color" },
      alpha2Color: { value: initial.alpha2Color ?? "#00b0ff", label: "alpha2 color" },
      alpha3Color: { value: initial.alpha3Color ?? "#ffea00", label: "alpha3 color" },
    }),
    GMap: folder({
      alphaDisplay: {
        ...button(() => {
          set({
            showVertices: false,
            showEdges: false,
            showFaces: false,
            showDarts: false,
            showDartLabels: false,
            showAlphaLinks: true,
            showAlpha0: true,
            showAlpha1: true,
            showAlpha2: true,
            showAlpha3: true,
          });
        }),
        label: "alpha display",
      },
      showAlphaLinks: {
        value: initial.showAlphaLinks ?? true,
        label: "alpha links",
      },
      Details: folder({
        showDarts: { value: initial.showDarts ?? false, label: "darts" },
        showDartLabels: {
          value: initial.showDartLabels ?? false,
          label: "dart labels",
        },
        showAlpha0: { value: initial.showAlpha0 ?? true, label: "alpha0 links" },
        showAlpha1: { value: initial.showAlpha1 ?? true, label: "alpha1 links" },
        showAlpha2: { value: initial.showAlpha2 ?? true, label: "alpha2 links" },
        showAlpha3: { value: initial.showAlpha3 ?? false, label: "alpha3 links" },
      }),
    }),
  }));

  const visibleAlphas = useMemo(() => {
    const s = new Set<number>();
    if (!values.showAlphaLinks) return s;
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
    values.showAlphaLinks,
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
    showWorldFrame: values.showWorldFrame,
    showVertices: values.showVertices,
    showEdges: values.showEdges,
    showFaces: values.showFaces,
    vertexSize: values.vertexSize,
    edgeWidth: values.edgeWidth,
    vertexColor: values.vertexColor,
    edgeColor: values.edgeColor,
    faceColor: values.faceColor,
    faceOpacity: values.faceOpacity,
    viewerFaceColorOverridesScene: values.viewerFaceColorOverridesScene,
    showDarts: values.showDarts,
    showDartLabels: values.showDartLabels,
    showAlphaLinks: values.showAlphaLinks,
    visibleAlphas,
    alphaColors,
  };
}
