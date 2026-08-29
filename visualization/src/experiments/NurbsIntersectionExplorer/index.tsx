import { Html, Line } from "@react-three/drei";
import { useMemo, useState } from "react";
import * as THREE from "three";
import ControlPolygon from "../NurbsCurveEditor/ControlPolygon";
import DraggableHandle from "../NurbsCurveEditor/DraggableHandle";
import { useKernel, type Kernel } from "../../kernel/useKernel";
import { vec3ArrayToFlat, type Vec3 } from "../../kernel/nurbs";

type CurveSpec = {
  degree: number;
  points: Vec3[];
  weights: number[];
};

type Preset = {
  id: string;
  label: string;
  a: CurveSpec;
  b: CurveSpec;
};

type WasmCurve = ReturnType<Kernel["NurbsCurve"]["uniform"]>;

type CurveCurveIntersection =
  | {
      kind: "point";
      point: [number, number, number];
      u_a: number;
      u_b: number;
    }
  | {
      kind: "overlap";
      interval_a: [number, number];
      interval_b: [number, number];
    };

const SAMPLES = 160;
const OVERLAP_SAMPLES = 48;
const CONTROL_PLANE_NORMAL: Vec3 = [0, 0, 1];

const PRESETS: Preset[] = [
  {
    id: "point",
    label: "Intersection point",
    a: {
      degree: 2,
      points: [
        [-2.4, -0.9, 0],
        [0, 1.25, 0],
        [2.4, -0.9, 0],
      ],
      weights: [1, 1, 1],
    },
    b: {
      degree: 1,
      points: [
        [0, -1.7, 0],
        [0, 1.7, 0],
      ],
      weights: [1, 1],
    },
  },
  {
    id: "line-overlap",
    label: "Overlap lineaire",
    a: {
      degree: 1,
      points: [
        [-2.6, 0, 0],
        [2.6, 0, 0],
      ],
      weights: [1, 1],
    },
    b: {
      degree: 1,
      points: [
        [-0.9, 0, 0],
        [1.7, 0, 0],
      ],
      weights: [1, 1],
    },
  },
  {
    id: "coincident",
    label: "Courbes confondues",
    a: {
      degree: 2,
      points: [
        [-2.2, -0.45, 0],
        [0, 1.35, 0],
        [2.2, -0.45, 0],
      ],
      weights: [1, 1, 1],
    },
    b: {
      degree: 2,
      points: [
        [-2.2, -0.45, 0],
        [0, 1.35, 0],
        [2.2, -0.45, 0],
      ],
      weights: [1, 1, 1],
    },
  },
];

function vectorsFromFlat(flat: Float64Array): THREE.Vector3[] {
  const out: THREE.Vector3[] = [];
  for (let i = 0; i < flat.length; i += 3) {
    out.push(new THREE.Vector3(flat[i], flat[i + 1], flat[i + 2]));
  }
  return out;
}

function vec3FromTuple(point: [number, number, number]) {
  return new THREE.Vector3(point[0], point[1], point[2]);
}

function cloneSpec(spec: CurveSpec): CurveSpec {
  return {
    degree: spec.degree,
    points: spec.points.map((point) => [...point] as Vec3),
    weights: [...spec.weights],
  };
}

function pointAt(curve: WasmCurve, u: number) {
  const point = curve.pointAt(u);
  return new THREE.Vector3(point.x, point.y, point.z);
}

function sampleInterval(curve: WasmCurve, interval: [number, number]) {
  const start = interval[0];
  const end = interval[1];
  const samples: THREE.Vector3[] = [];
  for (let i = 0; i <= OVERLAP_SAMPLES; i++) {
    const t = i / OVERLAP_SAMPLES;
    samples.push(pointAt(curve, start + (end - start) * t));
  }
  return samples;
}

function updatePoint(points: Vec3[], index: number, next: Vec3) {
  return points.map((point, i) => (i === index ? next : point));
}

export default function NurbsIntersectionExplorer() {
  const kernel = useKernel();
  const [presetId, setPresetId] = useState(PRESETS[0].id);
  const [curveA, setCurveA] = useState(() => cloneSpec(PRESETS[0].a));
  const [curveB, setCurveB] = useState(() => cloneSpec(PRESETS[0].b));

  function applyPreset(id: string) {
    const preset = PRESETS.find((candidate) => candidate.id === id) ?? PRESETS[0];
    setPresetId(preset.id);
    setCurveA(cloneSpec(preset.a));
    setCurveB(cloneSpec(preset.b));
  }

  const state = useMemo(() => {
    if (!kernel) {
      return {
        curveAPoints: [] as THREE.Vector3[],
        curveBPoints: [] as THREE.Vector3[],
        pointIntersections: [] as THREE.Vector3[],
        overlapSegments: [] as THREE.Vector3[][],
        intersections: [] as CurveCurveIntersection[],
        error: null as string | null,
      };
    }

    let a: WasmCurve | null = null;
    let b: WasmCurve | null = null;

    try {
      a = kernel.NurbsCurve.uniform(
        curveA.degree,
        vec3ArrayToFlat(curveA.points),
        new Float64Array(curveA.weights),
      );
      b = kernel.NurbsCurve.uniform(
        curveB.degree,
        vec3ArrayToFlat(curveB.points),
        new Float64Array(curveB.weights),
      );

      const intersections = a.intersectCurve(b) as CurveCurveIntersection[];
      return {
        curveAPoints: vectorsFromFlat(a.sample(SAMPLES)),
        curveBPoints: vectorsFromFlat(b.sample(SAMPLES)),
        pointIntersections: intersections.flatMap((intersection) =>
          intersection.kind === "point" ? [vec3FromTuple(intersection.point)] : [],
        ),
        overlapSegments: intersections.flatMap((intersection) =>
          intersection.kind === "overlap" ? [sampleInterval(a as WasmCurve, intersection.interval_a)] : [],
        ),
        intersections,
        error: null,
      };
    } catch (e) {
      return {
        curveAPoints: [] as THREE.Vector3[],
        curveBPoints: [] as THREE.Vector3[],
        pointIntersections: [] as THREE.Vector3[],
        overlapSegments: [] as THREE.Vector3[][],
        intersections: [] as CurveCurveIntersection[],
        error: e instanceof Error ? e.message : String(e),
      };
    } finally {
      a?.free();
      b?.free();
    }
  }, [curveA, curveB, kernel]);

  const pointCount = state.intersections.filter((intersection) => intersection.kind === "point").length;
  const overlapCount = state.intersections.filter((intersection) => intersection.kind === "overlap").length;

  return (
    <group>
      <gridHelper
        args={[7, 14, "#30323a", "#202229"]}
        rotation={[Math.PI / 2, 0, 0]}
      />

      {state.curveAPoints.length > 1 && (
        <Line points={state.curveAPoints} color="#6ea8ff" lineWidth={3} />
      )}
      {state.curveBPoints.length > 1 && (
        <Line points={state.curveBPoints} color="#f4a261" lineWidth={3} />
      )}

      <ControlPolygon points={curveA.points} color="#375f91" />
      <ControlPolygon points={curveB.points} color="#8b5a2b" />

      {curveA.points.map((point, index) => (
        <DraggableHandle
          key={`a-${index}`}
          position={point}
          color="#6ea8ff"
          radius={0.085}
          dragPlaneNormal={CONTROL_PLANE_NORMAL}
          onChange={(next) =>
            setCurveA((current) => ({
              ...current,
              points: updatePoint(current.points, index, next),
            }))
          }
        />
      ))}
      {curveB.points.map((point, index) => (
        <DraggableHandle
          key={`b-${index}`}
          position={point}
          color="#f4a261"
          radius={0.085}
          dragPlaneNormal={CONTROL_PLANE_NORMAL}
          onChange={(next) =>
            setCurveB((current) => ({
              ...current,
              points: updatePoint(current.points, index, next),
            }))
          }
        />
      ))}

      {state.overlapSegments.map((segment, index) => (
        <Line key={`overlap-${index}`} points={segment} color="#f7e36b" lineWidth={8} />
      ))}
      {state.pointIntersections.map((point, index) => (
        <mesh key={`point-${index}`} position={point}>
          <sphereGeometry args={[0.14, 24, 24]} />
          <meshStandardMaterial color="#f7e36b" emissive="#6c5b00" emissiveIntensity={0.45} />
        </mesh>
      ))}

      <Html fullscreen>
        <div className="intersection-explorer">
          <section className="intersection-panel">
            <div className="intersection-header">
              <div>
                <h2>NURBS intersections</h2>
                <span>curve / curve</span>
              </div>
              <strong>{pointCount} P / {overlapCount} O</strong>
            </div>

            <label className="intersection-select">
              <span>Cas</span>
              <select value={presetId} onChange={(event) => applyPreset(event.target.value)}>
                {PRESETS.map((preset) => (
                  <option key={preset.id} value={preset.id}>
                    {preset.label}
                  </option>
                ))}
              </select>
            </label>

            <div className="intersection-legend">
              <span><i className="curve-a" /> A</span>
              <span><i className="curve-b" /> B</span>
              <span><i className="curve-hit" /> intersection</span>
            </div>

            <div className="intersection-results">
              {state.intersections.length === 0 && <span>Aucune intersection</span>}
              {state.intersections.map((intersection, index) => (
                <div key={index} className="intersection-result-row">
                  {intersection.kind === "point" ? (
                    <>
                      <b>Point</b>
                      <span>
                        uA {intersection.u_a.toFixed(4)} / uB {intersection.u_b.toFixed(4)}
                      </span>
                    </>
                  ) : (
                    <>
                      <b>Overlap</b>
                      <span>
                        A [{intersection.interval_a[0].toFixed(4)}, {intersection.interval_a[1].toFixed(4)}]
                      </span>
                    </>
                  )}
                </div>
              ))}
            </div>

            {state.error && <div className="intersection-warning">{state.error}</div>}
          </section>
        </div>
      </Html>
    </group>
  );
}
