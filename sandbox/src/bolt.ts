export const boltExample = `// NGK bolt: hex head, shank, core, and swept helical thread.
const { Point, Vector, Axis, Frame } = ngk.geometry;
const { polygon } = ngk.modeling.faces;
const { cylinder_at, extruded, fuse } = ngk.modeling.solids;
const { helix } = ngk.modeling.edges;
const { sweep_face } = ngk.modeling.sweep;

const TAU = 2 * Math.PI;
const HEAD_ACROSS_FLATS = 10;
const HEAD_HEIGHT = 4;
const BODY_RADIUS = 3;
const BODY_LENGTH = 8;
const CORE_RADIUS = 2.5;
const THREAD_LENGTH = 12;
const THREAD_PITCH = 1.25;
const THREAD_DEPTH = 0.5;
const THREAD_OVERLAP = 0.1;
const THREAD_BASE_WIDTH = 0.9;
const THREAD_MARGIN = 0.75;

function frameAt(z: number) {
  return new Frame(new Point(0, 0, z), new Vector(1, 0, 0), new Vector(0, 1, 0));
}

function head() {
  const radius = HEAD_ACROSS_FLATS / Math.sqrt(3);
  const corners = Array.from({ length: 6 }, (_, i) => {
    const angle = TAU * i / 6;
    return new Point(radius * Math.cos(angle), radius * Math.sin(angle), 0);
  });
  return extruded(polygon(corners), new Vector(0, 0, 1), HEAD_HEIGHT);
}

function body() {
  return cylinder_at(BODY_RADIUS, BODY_LENGTH, frameAt(-BODY_LENGTH));
}

function core() {
  return cylinder_at(CORE_RADIUS, THREAD_LENGTH, frameAt(-BODY_LENGTH - THREAD_LENGTH));
}

function thread() {
  const bottom = -BODY_LENGTH - THREAD_LENGTH + THREAD_MARGIN;
  const turns = (THREAD_LENGTH - 2 * THREAD_MARGIN) / THREAD_PITCH;
  const axis = new Axis(new Point(0, 0, bottom), new Vector(0, 0, 1));
  const spine = helix(axis, CORE_RADIUS, THREAD_PITCH, 0, TAU * turns);
  const start = spine.start.point;
  if (!start) throw new Error("Helix start has no point");
  const radialX = start.x / CORE_RADIUS;
  const radialY = start.y / CORE_RADIUS;
  const baseRadius = CORE_RADIUS - THREAD_OVERLAP;
  const crestRadius = CORE_RADIUS + THREAD_DEPTH;
  const half = THREAD_BASE_WIDTH / 2;
  const profile = polygon([
    new Point(radialX * baseRadius, radialY * baseRadius, start.z - half),
    new Point(radialX * crestRadius, radialY * crestRadius, start.z),
    new Point(radialX * baseRadius, radialY * baseRadius, start.z + half),
  ]);
  return sweep_face(profile, spine, axis, Math.ceil(turns * 16));
}

console.log("Building head and shank…");
let bolt = fuse(head(), body());
bolt = fuse(bolt, core());
console.log("Sweeping and fusing the thread…");
bolt = fuse(bolt, thread());
show(bolt);
console.log("Bolt ready: " + bolt.faceCount + " faces");
`;
