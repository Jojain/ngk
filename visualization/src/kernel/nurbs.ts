import * as THREE from "three";
import type { Kernel } from "./useKernel";

export type Vec3 = [number, number, number];

export function flatToVec3Array(flat: Float64Array | number[]): Vec3[] {
  const out: Vec3[] = [];
  for (let i = 0; i < flat.length; i += 3) {
    out.push([flat[i], flat[i + 1], flat[i + 2]]);
  }
  return out;
}

export function vec3ArrayToFlat(points: Vec3[]): Float64Array {
  const flat = new Float64Array(points.length * 3);
  for (let i = 0; i < points.length; i++) {
    flat[i * 3 + 0] = points[i][0];
    flat[i * 3 + 1] = points[i][1];
    flat[i * 3 + 2] = points[i][2];
  }
  return flat;
}

export function makeUniformCurve(
  kernel: Kernel,
  degree: number,
  controlPoints: Vec3[],
  weights: number[],
) {
  return kernel.WasmNurbsCurve.uniform(
    degree,
    vec3ArrayToFlat(controlPoints),
    new Float64Array(weights),
  );
}

export function sampleCurveAsVector3(curve: {
  sample: (n: number) => Float64Array;
}, n: number): THREE.Vector3[] {
  const flat = curve.sample(n);
  const out: THREE.Vector3[] = [];
  for (let i = 0; i < flat.length; i += 3) {
    out.push(new THREE.Vector3(flat[i], flat[i + 1], flat[i + 2]));
  }
  return out;
}
