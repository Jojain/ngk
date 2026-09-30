export const bearingHousingExample = `// Bearing housing: build a plate, fuse a boss, then cut six holes.
const { Point, Vector, Frame } = ngk.geometry;
const { block, cylinder_at, fuse, cut } = ngk.modeling.solids;

const PLATE_SIZE = 14;
const PLATE_THICKNESS = 1.4;
const BOSS_RADIUS = 4;
const BOSS_HEIGHT = 3;
const BORE_RADIUS = 2;
const HOLE_RADIUS = 0.65;
const HOLE_OFFSET = 4.3;

function frameAt(x: number, y: number, z: number) {
  return new Frame(new Point(x, y, z), new Vector(1, 0, 0), new Vector(0, 1, 0));
}

console.log("Building the square mounting plate and boss…");
const plate = block(PLATE_SIZE, PLATE_SIZE, PLATE_THICKNESS);
const boss = cylinder_at(BOSS_RADIUS, BOSS_HEIGHT, frameAt(7, 7, PLATE_THICKNESS));
let housing = fuse(plate, boss);

console.log("Cutting the shaft bore and four mounting holes…");
housing = cut(housing, cylinder_at(BORE_RADIUS, 5.2, frameAt(7, 7, -0.2)));

for (const x of [7 - HOLE_OFFSET, 7 + HOLE_OFFSET]) {
  for (const y of [7 - HOLE_OFFSET, 7 + HOLE_OFFSET]) {
    housing = cut(housing, cylinder_at(HOLE_RADIUS, 1.8, frameAt(x, y, -0.2)));
  }
}

console.log("Adding a chamfer to the boss rim and a fillet to the plate top…");
const topBossEdge = housing.edges().find((edge) => {
  const curve = edge.curve as { radius?: number; plane?: { origin?: { z?: number } } };
  return (
    curve?.radius === BOSS_RADIUS &&
    Math.abs((curve.plane?.origin?.z ?? Number.NaN) - (PLATE_THICKNESS + BOSS_HEIGHT)) < 1e-6
  );
});
if (!topBossEdge) throw new Error("Top boss circular edge not found");
const chamferTarget = new ngk.modeling.blend.BlendTarget();
chamferTarget.addEdge(topBossEdge);
housing = ngk.modeling.blend.chamfered_solid(housing, chamferTarget, 0.35);

const plateTopFace = housing.faces().find((face) => {
  const surface = face.surface as {
    origin?: { z?: number };
    normal?: { z?: number };
  };
  return (
    surface?.normal !== undefined &&
    Math.abs((surface.origin?.z ?? Number.NaN) - PLATE_THICKNESS) < 1e-6
  );
});
if (!plateTopFace) throw new Error("Plate top face not found");
const plateBoundaryEdges = plateTopFace.edges().filter((edge) => {
  const curve = edge.curve as { constructor?: { name?: string } };
  return curve?.constructor?.name === "Line" && Math.abs(edge.length - PLATE_SIZE) < 1e-6;
});
if (plateBoundaryEdges.length !== 4) throw new Error("Plate top boundary edges not found");
const filletTarget = new ngk.modeling.blend.BlendTarget();
for (const edge of plateBoundaryEdges) filletTarget.addEdge(edge);
housing = ngk.modeling.blend.filleted_solid(housing, filletTarget, 0.25);

show(housing);
console.log("Bearing housing ready: " + housing.faceCount + " faces, " + housing.edgeCount + " edges");
`;
