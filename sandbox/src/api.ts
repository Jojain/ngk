import * as wasm from "./wasm/ngk";

type Frame = InstanceType<typeof wasm.Frame>;

function cylinderAt(
  radius: number,
  height: number,
  frame?: Frame,
) {
  return wasm.cylinder(radius, height, frame);
}

/** The script-facing API follows NGK's modeling domains. */
export const ngk = {
  geometry: {
    Point: wasm.Point3,
    Vector: wasm.Vector3,
    Axis: wasm.Axis3,
    Frame: wasm.Frame,
  },
  modeling: {
    blend: {
      BlendTarget: wasm.BlendTarget,
      chamfered_solid: wasm.chamferedSolid,
      filleted_solid: wasm.filletedSolid,
    },
    edges: { helix: wasm.helix },
    faces: { polygon: wasm.polygonFace },
    solids: {
      block: wasm.block,
      cylinder_at: cylinderAt,
      cylinder: cylinderAt,
      extruded: wasm.extruded,
      fuse: wasm.fuse,
      cut: wasm.cut,
      intersect: wasm.intersect,
    },
    sweep: { sweep_face: wasm.sweepFaceAlongEdge },
  },
} as const;

export type Showable = wasm.Model | wasm.Solid | wasm.Face | wasm.Edge;

export function sceneFor(shape: Showable) {
  if (shape instanceof wasm.Model) return wasm.sceneFromGMap(shape);
  if (
    shape instanceof wasm.Solid ||
    shape instanceof wasm.Face ||
    shape instanceof wasm.Edge
  ) {
    return wasm.sceneFromGMap(shape.model);
  }
  throw new Error("show() expects an NGK model, solid, face, or edge");
}
