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
    id: "template",
    title: "_template",
    group: "Other",
    component: lazy(() => import("./_template")),
  },
];
