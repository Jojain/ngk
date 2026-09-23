import type { VizScene } from "@ngk/viewer";

export type WorkerRequest = { kind: "run"; code: string };
export type WorkerResponse =
  | { kind: "ready" }
  | { kind: "log"; level: "log" | "warn" | "error"; text: string }
  | { kind: "scene"; scene: VizScene }
  | { kind: "done"; shown: boolean }
  | { kind: "error"; text: string };
