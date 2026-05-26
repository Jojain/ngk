import type { Vec3, VizAlphaLink, VizDart } from "../src/kernel/viz.ts";
import { layoutDartLanes } from "../src/components/VizSceneView.layout.ts";

function assert(condition: boolean, message: string): asserts condition {
  if (!condition) throw new Error(message);
}

function near(a: number, b: number, epsilon = 1e-9): boolean {
  return Math.abs(a - b) <= epsilon;
}

function shaftY(shaft: Vec3[]): number {
  assert(shaft.length >= 2, "expected a shaft with at least two points");
  assert(
    near(shaft[0][1], shaft[shaft.length - 1][1]),
    "linear shaft should keep a constant lane offset",
  );
  return shaft[0][1];
}

function shaftZ(shaft: Vec3[]): number {
  assert(shaft.length >= 2, "expected a shaft with at least two points");
  assert(
    near(shaft[0][2], shaft[shaft.length - 1][2]),
    "linear shaft should keep a constant z offset",
  );
  return shaft[0][2];
}

function makeDart(dartId: number, edgeId: number, shaft: Vec3[]): VizDart {
  return {
    dartId,
    edgeId,
    shaft,
    tipDir: [1, 0, 0],
  };
}

function sharedLinearEdgeUsesPlanarParallelLanes() {
  const darts = [
    makeDart(0, 7, [
      [0, 0, 0],
      [0.4, 0, 0],
    ]),
    makeDart(1, 7, [
      [1, 0, 0],
      [0.6, 0, 0],
    ]),
    makeDart(2, 7, [
      [0, 0, 0],
      [0.4, 0, 0],
    ]),
    makeDart(3, 7, [
      [1, 0, 0],
      [0.6, 0, 0],
    ]),
  ];

  const layout = layoutDartLanes(darts, []);
  const shafts = darts.map((d) => layout.shaftsByDartId.get(d.dartId));
  assert(shafts.every(Boolean), "expected every dart to have a display shaft");

  const yOffsets = shafts.map((shaft) => shaftY(shaft!));
  const zOffsets = shafts.map((shaft) => shaftZ(shaft!));
  assert(
    zOffsets.every((z) => near(z, 0)),
    `linear edge lanes should stay in the source plane, got ${zOffsets}`,
  );
  assert(near(yOffsets[0], -yOffsets[3]), "outer lanes should mirror");
  assert(near(yOffsets[1], -yOffsets[2]), "inner lanes should mirror");
}

function faceLaneKeepsLinearDartStraight() {
  const darts = [
    makeDart(0, 0, [
      [0, 0, 0],
      [0.4, 0, 0],
    ]),
    makeDart(1, 1, [
      [1, 0, 0],
      [1, 0.4, 0],
    ]),
    makeDart(2, 2, [
      [0, 1, 0],
      [0.4, 1, 0],
    ]),
  ];
  const alphaLinks: VizAlphaLink[] = [
    { involution: 0, dartA: 0, dartB: 1, a: [0, 0, 0], b: [0, 0, 0] },
    { involution: 1, dartA: 1, dartB: 2, a: [0, 0, 0], b: [0, 0, 0] },
  ];

  const layout = layoutDartLanes(darts, alphaLinks);
  const shaft = layout.shaftsByDartId.get(0);
  assert(shaft, "expected dart 0 to have a display shaft");
  assert(near(shaftY(shaft), shaft[1][1]), "face lane should be constant");
  assert(near(shaftZ(shaft), 0), "face lane should not lift a planar line");
}

function faceOwnedSharedEdgeUsesFaceLaneOnly() {
  const darts = [
    makeDart(0, 7, [
      [0, 0, 0],
      [0.4, 0, 0],
    ]),
    makeDart(1, 7, [
      [1, 0, 0],
      [0.6, 0, 0],
    ]),
    makeDart(2, 1, [
      [0, 1, 0],
      [0.4, 1, 0],
    ]),
  ];
  const alphaLinks: VizAlphaLink[] = [
    { involution: 0, dartA: 0, dartB: 1, a: [0, 0, 0], b: [0, 0, 0] },
    { involution: 1, dartA: 1, dartB: 2, a: [0, 0, 0], b: [0, 0, 0] },
  ];

  const layout = layoutDartLanes(darts, alphaLinks);
  const a = layout.shaftsByDartId.get(0);
  const b = layout.shaftsByDartId.get(1);
  assert(a && b, "expected face darts to have display shafts");
  assert(
    near(shaftY(a), shaftY(b)),
    "alpha0-paired darts in the same face should share one face lane",
  );
  assert(near(shaftZ(a), 0), "face lane should stay planar");
}

function alphaLinksUseLaneMidpoints() {
  const darts = [
    makeDart(0, 0, [
      [0, 0, 0],
      [0.4, 0, 0],
    ]),
    makeDart(1, 1, [
      [1, 0, 0],
      [0.6, 0, 0],
    ]),
  ];
  const layout = layoutDartLanes(darts, []);

  assert(
    near(layout.alphaEndpoint(0)![0], 0.2),
    "alpha links should anchor at the display lane midpoint",
  );
}

sharedLinearEdgeUsesPlanarParallelLanes();
faceLaneKeepsLinearDartStraight();
faceOwnedSharedEdgeUsesFaceLaneOnly();
alphaLinksUseLaneMidpoints();
