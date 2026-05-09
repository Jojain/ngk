import { lazy } from "react";
import type { ComponentType } from "react";

export type ExperimentMeta = {
  id: string;
  title: string;
  group?: string;
  component: ComponentType;
};

export const experiments: ExperimentMeta[] = [
  {
    id: "nurbs-curve-editor",
    title: "NURBS curve editor",
    group: "NURBS",
    component: lazy(() => import("./NurbsCurveEditor")),
  },
  {
    id: "nurbs-surface-editor",
    title: "NURBS surface editor",
    group: "NURBS",
    component: lazy(() => import("./NurbsSurfaceEditor")),
  },
  {
    id: "two-faces-alpha2",
    title: "Two faces α2-sewn",
    group: "GMap",
    component: lazy(() => import("./TwoFacesAlpha2")),
  },
  {
    id: "hollow-cylinder",
    title: "Hollow cylinder",
    group: "Display",
    component: lazy(() => import("./HollowCylinder")),
  },
  {
    id: "extruded-square",
    title: "Extruded square",
    group: "Display",
    component: lazy(() => import("./ExtrudedSquare")),
  },
  {
    id: "interactive-extrusion",
    title: "Interactive extrusion",
    group: "Display",
    component: lazy(() => import("./InteractiveExtrusion")),
  },
  {
    id: "extruded-holed-pentagon",
    title: "Extruded pentagon with square hole",
    group: "Display",
    component: lazy(() => import("./ExtrudedHoledPentagon")),
  },
  {
    id: "extruded-open-polyline",
    title: "Extruded open polyline",
    group: "Display",
    component: lazy(() => import("./ExtrudedOpenPolyline")),
  },
  {
    id: "cylinder",
    title: "Quarter cylinder (curved darts)",
    group: "Display",
    component: lazy(() => import("./Cylinder")),
  },
  {
    id: "template",
    title: "_template",
    group: "Other",
    component: lazy(() => import("./_template")),
  },
];
