// Rounded tray: round the four vertical edges of a block, then shell it open
// at the top. Try other thicknesses, other open faces (none gives a cavity),
// or `offset_solid(box, d)` to see the whole solid grow or shrink.
const { Plane } = ngk.geometry;
const { block } = ngk.modeling.solids;
const { BlendTarget, filleted_solid } = ngk.modeling.blend;
const { offset_solid, shelled_solid } = ngk.modeling.offset;

const WIDTH = 12;
const DEPTH = 8;
const HEIGHT = 4;
const CORNER_RADIUS = 1.5;
const WALL = 0.4;

console.log("Rounding the vertical edges…");
let box = block(WIDTH, DEPTH, HEIGHT);
const vertical = new BlendTarget();
for (const edge of box.edges()) {
  const heights = edge.vertices().map((vertex) => vertex.point?.z ?? 0);
  if (Math.abs(Math.max(...heights) - Math.min(...heights) - HEIGHT) < 1e-9) vertical.addEdge(edge);
}
box = filleted_solid(box, vertical, CORNER_RADIUS);

console.log("Shelling it open at the top…");
const top = box.faces().find((face) => {
  const surface = face.surface;
  return surface instanceof Plane && Math.abs(surface.origin.z - HEIGHT) < 1e-9;
});
if (!top) throw new Error("Top face not found");
const tray = shelled_solid(box, [top], WALL);

// The offset keeps its source beside it, so showing it shows both solids.
const grown = offset_solid(box, 0.5);
console.log("Grown copy volume: " + grown.volume.toFixed(3));

show(tray);
console.log("Tray ready: " + tray.faceCount + " faces, volume " + tray.volume.toFixed(3));
