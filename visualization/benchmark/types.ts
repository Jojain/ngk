export type Outcome = "pass" | "built" | "wrong" | "refused" | "crashed" | "timeout" | "missing";
export type Kernel = "occt" | "ngk";
export interface Result {
  outcome: Outcome;
  why: string;
  meshUrl: string | null;
  volume?: number | null;
  area?: number | null;
  seconds?: number;
  faces?: Record<string, number>;
  mesh?: { triangles: number; closed: boolean; unmatched_edges: number; volume: number };
}
export interface Case {
  id: string;
  title: string;
  category: string;
  description: string;
  expect: { volume: { value: number }; area: { value: number } } | null;
  baseline: Outcome | null;
  results: Record<Kernel, Result>;
}
export interface Manifest {
  generatedAt: string;
  upstreamCommit: string;
  kernels: Record<Kernel, { label: string; version: string | null; run: string | null; tally: Partial<Record<Outcome, number>> }>;
  cases: Case[];
}
export const labels: Record<Outcome, string> = {
  pass: "Pass", built: "Built", wrong: "Incorrect", refused: "Refused",
  crashed: "Crashed", timeout: "Timed out", missing: "Not run",
};
