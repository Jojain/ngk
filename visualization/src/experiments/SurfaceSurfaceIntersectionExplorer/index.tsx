import { Html, Line } from "@react-three/drei";
import { useMemo, useState } from "react";
import * as THREE from "three";
import ControlPolygon from "../NurbsCurveEditor/ControlPolygon";
import DraggableHandle from "../NurbsCurveEditor/DraggableHandle";
import { useKernel, type Kernel } from "../../kernel/useKernel";
import { vec3ArrayToFlat, type Vec3 } from "../../kernel/nurbs";

type WasmSurface = ReturnType<Kernel["NurbsSurface"]["uniform"]>;

type SurfaceSurfaceIntersection =
  | { kind: "point"; point: [number, number, number] }
  | { kind: "curve"; points: [number, number, number][] }
  | { kind: "region" };

type Preset = {
  id: string;
  label: string;
  a: Vec3[];
  b: Vec3[];
};

const SURFACE_U = 2;
const SURFACE_V = 2;
const WEIGHTS = [1, 1, 1, 1];

const PRESETS: Preset[] = [
  {
    id: "curve",
    label: "Courbe",
    a: [
      [-1.5, -1.0, 0],
      [1.5, -1.0, 0],
      [-1.5, 1.0, 0],
      [1.5, 1.0, 0],
    ],
    b: [
      [-1.5, 0, -1.0],
      [1.5, 0, -1.0],
      [-1.5, 0, 1.0],
      [1.5, 0, 1.0],
    ],
  },
  {
    id: "region",
    label: "Region confondue",
    a: [
      [-1.5, -1.0, 0],
      [1.5, -1.0, 0],
      [-1.5, 1.0, 0],
      [1.5, 1.0, 0],
    ],
    b: [
      [-1.5, -1.0, 0],
      [1.5, -1.0, 0],
      [-1.5, 1.0, 0],
      [1.5, 1.0, 0],
    ],
  },
];

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

export default function SurfaceSurfaceIntersectionExplorer() {
  const kernel = useKernel();
  const [presetId, setPresetId] = useState(PRESETS[0].id);
  const [surfaceA, setSurfaceA] = useState<Vec3[]>(PRESETS[0].a);
  const [surfaceB, setSurfaceB] = useState<Vec3[]>(PRESETS[0].b);

  function applyPreset(id: string) {
    const preset = PRESETS.find((candidate) => candidate.id === id) ?? PRESETS[0];
    setPresetId(preset.id);
    setSurfaceA(preset.a.map((point) => [...point] as Vec3));
    setSurfaceB(preset.b.map((point) => [...point] as Vec3));
  }

  const state = useMemo(() => {
    if (!kernel) return null;
    let a: WasmSurface | null = null;
    let b: WasmSurface | null = null;
    try {
      a = kernel.NurbsSurface.uniform(
        1,
        1,
        SURFACE_U,
        SURFACE_V,
        vec3ArrayToFlat(surfaceA),
        new Float64Array(WEIGHTS),
      );
      b = kernel.NurbsSurface.uniform(
        1,
        1,
        SURFACE_U,
        SURFACE_V,
        vec3ArrayToFlat(surfaceB),
        new Float64Array(WEIGHTS),
      );
      const intersections = a.intersectSurface(b) as SurfaceSurfaceIntersection[];
      return {
        geometryA: surfaceGeometry(a),
        geometryB: surfaceGeometry(b),
        curves: intersections.flatMap((intersection) =>
          intersection.kind === "curve"
            ? [intersection.points.map((point) => new THREE.Vector3(...point))]
            : [],
        ),
        points: intersections.flatMap((intersection) =>
          intersection.kind === "point" ? [new THREE.Vector3(...intersection.point)] : [],
        ),
        region: intersections.some((intersection) => intersection.kind === "region"),
        intersections,
        error: null as string | null,
      };
    } catch (e) {
      return {
        geometryA: null,
        geometryB: null,
        curves: [] as THREE.Vector3[][],
        points: [] as THREE.Vector3[],
        region: false,
        intersections: [] as SurfaceSurfaceIntersection[],
        error: e instanceof Error ? e.message : String(e),
      };
    } finally {
      a?.free();
      b?.free();
    }
  }, [kernel, surfaceA, surfaceB]);

  return (
    <group>
      <gridHelper args={[5, 10, "#30323a", "#202229"]} />
      {state?.geometryA && (
        <mesh geometry={state.geometryA}>
          <meshStandardMaterial color="#4a7bc8" opacity={0.38} transparent side={THREE.DoubleSide} />
        </mesh>
      )}
      {state?.geometryB && (
        <mesh geometry={state.geometryB}>
          <meshStandardMaterial color="#d08a43" opacity={0.38} transparent side={THREE.DoubleSide} />
        </mesh>
      )}
      <ControlPolygon points={surfaceA} color="#375f91" />
      <ControlPolygon points={surfaceB} color="#8b5a2b" />

      {surfaceA.map((point, index) => (
        <DraggableHandle
          key={`a-${index}`}
          position={point}
          color="#6ea8ff"
          radius={0.075}
          dragPlaneNormal={[0, 1, 0]}
          onChange={(next) => setSurfaceA((current) => updatePoint(current, index, next))}
        />
      ))}
      {surfaceB.map((point, index) => (
        <DraggableHandle
          key={`b-${index}`}
          position={point}
          color="#f4a261"
          radius={0.075}
          dragPlaneNormal={[0, 1, 0]}
          onChange={(next) => setSurfaceB((current) => updatePoint(current, index, next))}
        />
      ))}

      {state?.curves.map((curve, index) => (
        <Line key={index} points={curve} color="#f7e36b" lineWidth={6} />
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
                <h2>Surface / surface</h2>
                <span>NURBS intersections</span>
              </div>
              <strong>{state?.region ? "region" : `${state?.intersections.length ?? 0}`}</strong>
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
