import wasmTypes from "./wasm/ngk.d.ts?raw";

export const ngkModuleTypes = `declare module "ngk-wasm" {\n${wasmTypes}\n}`;

export const scriptTypes = `
import type * as Wasm from "ngk-wasm";
declare global {
  const ngk: {
    geometry: {
      Point: typeof Wasm.Point3;
      Vector: typeof Wasm.Vector3;
      Axis: typeof Wasm.Axis3;
      Frame: typeof Wasm.Frame;
    };
    modeling: {
      edges: { helix: typeof Wasm.helix };
      faces: { polygon: typeof Wasm.polygonFace };
      solids: {
        cylinder_at: (radius: number, height: number, frame?: Wasm.Frame) => Wasm.Solid;
        cylinder: (radius: number, height: number, frame?: Wasm.Frame) => Wasm.Solid;
        extruded: (face: Wasm.Face, direction: Wasm.Vector3, distance: number) => Wasm.Solid;
        fuse: typeof Wasm.fuse;
      };
      sweep: { sweep_face: typeof Wasm.sweepFaceAxial };
    };
  };
  function show(shape: Wasm.Model | Wasm.Solid | Wasm.Face | Wasm.Edge): void;
}
export {};
`;
