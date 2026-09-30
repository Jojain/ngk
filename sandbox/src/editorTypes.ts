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
      blend: {
        BlendTarget: typeof Wasm.BlendTarget;
        chamfered_solid: typeof Wasm.chamferedSolid;
        filleted_solid: typeof Wasm.filletedSolid;
      };
      edges: { helix: typeof Wasm.helix };
      faces: { polygon: typeof Wasm.polygonFace };
      solids: {
        block: typeof Wasm.block;
        cylinder_at: (radius: number, height: number, frame?: Wasm.Frame) => Wasm.Solid;
        cylinder: (radius: number, height: number, frame?: Wasm.Frame) => Wasm.Solid;
        extruded: typeof Wasm.extruded;
        fuse: typeof Wasm.fuse;
        cut: typeof Wasm.cut;
        intersect: typeof Wasm.intersect;
      };
      sweep: { sweep_face: typeof Wasm.sweepFaceAlongEdge };
    };
  };
  function show(shape: Wasm.Model | Wasm.Solid | Wasm.Face | Wasm.Edge): void;
}
export {};
`;
