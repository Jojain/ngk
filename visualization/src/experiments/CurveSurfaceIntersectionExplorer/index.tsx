import { Html, Line } from "@react-three/drei";
import { useMemo, useState } from "react";
import * as THREE from "three";
import ControlPolygon from "../NurbsCurveEditor/ControlPolygon";
import DraggableHandle from "../NurbsCurveEditor/DraggableHandle";
import { useKernel, type Kernel } from "../../kernel/useKernel";
import { vec3ArrayToFlat, type Vec3 } from "../../kernel/nurbs";

type WasmCurve = ReturnType<Kernel["NurbsCurve"]["uniform"]>;
type WasmSurface = ReturnType<Kernel["NurbsSurface"]["uniform"]>;

type CurveSurfaceIntersection =
  | {
      kind: "point";
      point: [number, number, number];
      curve_u: number;
      surface_u: number;
      surface_v: number;
    }
  | {
      kind: "overlap";
      curve_interval: [number, number];
    };

type Preset = {
  id: string;
  label: string;
  curve: Vec3[];
  surface: Vec3[];
};

const SAMPLES = 96;
const SURFACE_U = 2;
const SURFACE_V = 2;
const SURFACE_WEIGHTS = [1, 1, 1, 1];

const PRESETS: Preset[] = [
  {
    id: "point",
    label: "Point",
    curve: [
      [0, 0, -1.2],
      [0, 0, 1.2],
    ],
    surface: [
      [-1.6, -1.0, 0],
      [1.6, -1.0, 0],
      [-1.6, 1.0, 0],
      [1.6, 1.0, 0],
    ],
  },
  {
    id: "overlap",
    label: "Courbe sur surface",
    curve: [
      [-1.2, -0.45, 0],
      [1.2, 0.45, 0],
    ],
    surface: [
      [-1.6, -1.0, 0],
      [1.6, -1.0, 0],
      [-1.6, 1.0, 0],
      [1.6, 1.0, 0],
    ],
  },
];

function vectorsFromFlat(flat: Float64Array): THREE.Vector3[] {
  const out: THREE.Vector3[] = [];
  for (let i = 0; i < flat.length; i += 3) {
    out.push(new THREE.Vector3(flat[i], flat[i + 1], flat[i + 2]));
  }
  return out;
}

function pointAt(curve: WasmCurve, u: number) {
  const flat = curve.pointAt(u);
  return new THREE.Vector3(flat[0], flat[1], flat[2]);
}

function sampleCurveInterval(curve: WasmCurve, interval: [number, number]) {
  const points: THREE.Vector3[] = [];
  for (let i = 0; i <= SAMPLES; i++) {
    const t = i / SAMPLES;
    points.push(pointAt(curve, interval[0] + (interval[1] - interval[0]) * t));
  }
  return points;
}

function surfaceGeometry(surface: WasmSurface) {
  const raw = surface.sampleGrid(24, 24) as {
    positions: ArrayLike<number>;
    normals: ArrayLike<number>;
    indices: ArrayLike<number>;
  };
  const positions = new Float32Array(raw.positions.length);
  positions.set(raw.positions);
  const normals = new Float32Array(raw.normals.length);
  normals.set(raw.normals);
  const indices = new Uint32Array(raw.indices.length);
  indices.set(raw.indices);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute("normal", new THREE.BufferAttribute(normals, 3));
  geometry.setIndex(Array.from(indices));
  geometry.computeBoundingSphere();
  return geometry;
}

function updatePoint(points: Vec3[], index: number, next: Vec3) {
  return points.map((point, i) => (i === index ? next : point));
}

export default function CurveSurfaceIntersectionExplorer() {
  const kernel = useKernel();
  const [presetId, setPresetId] = useState(PRESETS[0].id);
  const [curvePoints, setCurvePoints] = useState<Vec3[]>(PRESETS[0].curve);
  const [surfacePoints, setSurfacePoints] = useState<Vec3[]>(PRESETS[0].surface);

  function applyPreset(id: string) {
    const preset = PRESETS.find((candidate) => candidate.id === id) ?? PRESETS[0];
    setPresetId(preset.id);
    setCurvePoints(preset.curve.map((point) => [...point] as Vec3));
    setSurfacePoints(preset.surface.map((point) => [...point] as Vec3));
  }

  const state = useMemo(() => {
    if (!kernel) return null;
    let curve: WasmCurve | null = null;
    let surface: WasmSurface | null = null;
    try {
      curve = kernel.NurbsCurve.uniform(1, vec3ArrayToFlat(curvePoints), new Float64Array([1, 1]));
      surface = kernel.NurbsSurface.uniform(
        1,
        1,
        SURFACE_U,
        SURFACE_V,
        vec3ArrayToFlat(surfacePoints),
        new Float64Array(SURFACE_WEIGHTS),
      );
      const intersections = curve.intersectSurface(surface) as CurveSurfaceIntersection[];
      return {
        curve: vectorsFromFlat(curve.sample(SAMPLES)),
        surface: surfaceGeometry(surface),
        points: intersections.flatMap((intersection) =>
          intersection.kind === "point"
            ? [new THREE.Vector3(...intersection.point)]
            : [],
        ),
        overlaps: intersections.flatMap((intersection) =>
          intersection.kind === "overlap" ? [sampleCurveInterval(curve as WasmCurve, intersection.curve_interval)] : [],
        ),
        intersections,
        error: null as string | null,
      };
    } catch (e) {
      return {
        curve: [] as THREE.Vector3[],
        surface: null,
        points: [] as THREE.Vector3[],
        overlaps: [] as THREE.Vector3[][],
        intersections: [] as CurveSurfaceIntersection[],
        error: e instanceof Error ? e.message : String(e),
      };
    } finally {
      curve?.free();
      surface?.free();
    }
  }, [curvePoints, kernel, surfacePoints]);

  return (
    <group>
      <gridHelper args={[5, 10, "#30323a", "#202229"]} />
      {state?.surface && (
        <mesh geometry={state.surface}>
          <meshStandardMaterial color="#4a7bc8" opacity={0.42} transparent side={THREE.DoubleSide} />
        </mesh>
      )}
      {state && state.curve.length > 1 && <Line points={state.curve} color="#f4a261" lineWidth={3} />}
      <ControlPolygon points={curvePoints} color="#8b5a2b" />
      <ControlPolygon points={surfacePoints} color="#375f91" />

      {curvePoints.map((point, index) => (
        <DraggableHandle
          key={`curve-${index}`}
          position={point}
          color="#f4a261"
          radius={0.085}
          dragPlaneNormal={[0, 1, 0]}
          onChange={(next) => setCurvePoints((current) => updatePoint(current, index, next))}
        />
      ))}
      {surfacePoints.map((point, index) => (
        <DraggableHandle
          key={`surface-${index}`}
          position={point}
          color="#6ea8ff"
          radius={0.075}
          dragPlaneNormal={[0, 1, 0]}
          onChange={(next) => setSurfacePoints((current) => updatePoint(current, index, next))}
        />
      ))}

      {state?.overlaps.map((points, index) => (
        <Line key={index} points={points} color="#f7e36b" lineWidth={8} />
      ))}
      {state?.points.map((point, index) => (
        <mesh key={index} position={point}>
          <sphereGeometry args={[0.13, 24, 24]} />
          <meshStandardMaterial color="#f7e36b" emissive="#6c5b00" emissiveIntensity={0.45} />
        </mesh>
      ))}

      <Html fullscreen>
        <div className="intersection-explorer">
          <section className="intersection-panel">
            <div className="intersection-header">
              <div>
                <h2>Curve / surface</h2>
                <span>NURBS intersections</span>
              </div>
              <strong>{state?.intersections.length ?? 0}</strong>
            </div>
            <label className="intersection-select">
              <span>Cas</span>
              <select value={presetId} onChange={(event) => applyPreset(event.target.value)}>
                {PRESETS.map((preset) => (
                  <option key={preset.id} value={preset.id}>{preset.label}</option>
                ))}
              </select>
            </label>
            {state?.error && <div className="intersection-warning">{state.error}</div>}
          </section>
        </div>
      </Html>
    </group>
  );
}
