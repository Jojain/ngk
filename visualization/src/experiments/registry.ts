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
    id: "debug-viewer",
    title: "Debug viewer",
    group: "Debug",
    component: lazy(() => import("./DebugViewer")),
  },
  {
    id: "nurbs-curve-editor",
    title: "NURBS curve editor",
    group: "NURBS",
    component: lazy(() => import("./NurbsCurveEditor")),
  },
  {
    id: "nurbs-knot-explorer",
    title: "NURBS knot explorer",
    group: "NURBS",
    component: lazy(() => import("./NurbsKnotExplorer")),
  },
  {
    id: "nurbs-intersection-explorer",
    title: "NURBS intersection explorer",
    group: "NURBS",
    component: lazy(() => import("./NurbsIntersectionExplorer")),
  },
  {
    id: "curve-surface-intersection-explorer",
    title: "Curve / surface intersections",
    group: "NURBS",
    component: lazy(() => import("./CurveSurfaceIntersectionExplorer")),
  },
  {
    id: "surface-surface-intersection-explorer",
    title: "Surface / surface intersections",
    group: "NURBS",
    component: lazy(() => import("./SurfaceSurfaceIntersectionExplorer")),
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
    id: "chamfered-rectangle",
    title: "Chamfered rectangle corner (2D)",
    group: "Display",
    component: lazy(() => import("./ChamferedRectangle")),
  },
  {
    id: "chamfered-block",
    title: "Chamfered block edge (3D)",
    group: "Display",
    component: lazy(() => import("./ChamferedBlock")),
  },
  {
    id: "block-primitive",
    title: "Block primitive",
    group: "Display",
    component: lazy(() => import("./BlockPrimitive")),
  },
  {
    id: "boolean-operations",
    title: "Boolean operations",
    group: "Display",
    component: lazy(() => import("./BooleanOperations")),
  },
  {
    id: "interactive-extrusion",
    title: "Interactive extrusion",
    group: "Display",
    component: lazy(() => import("./InteractiveExtrusion")),
  },
  {
    id: "revolved-triangle",
    title: "Triangle revolution",
    group: "Display",
    component: lazy(() => import("./RevolvedTriangle")),
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
