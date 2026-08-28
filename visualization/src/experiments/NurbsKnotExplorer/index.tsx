import { Html, Line } from "@react-three/drei";
import { useMemo, useState, type WheelEvent } from "react";
import * as THREE from "three";
import ControlPolygon from "../NurbsCurveEditor/ControlPolygon";
import { useKernel } from "../../kernel/useKernel";
import { vec3ArrayToFlat, type Vec3 } from "../../kernel/nurbs";

type CurvePreset = {
  id: string;
  label: string;
  degree: number;
  points: Vec3[];
  weights: number[];
  knots?: number[];
};

const W_ARC = Math.SQRT1_2;
const DEFAULT_POINTS: Vec3[] = [
  [-3, 0, 0],
  [-1.8, 1.4, 0.5],
  [-0.6, -1.1, -0.2],
  [0.8, 1.2, -0.5],
  [1.9, -0.9, 0.3],
  [3, 0.2, 0],
];

const SAMPLES = 160;
const KNOT_MIN = 0;
const KNOT_MAX = 10;
const KNOT_STEP = 0.1;
const PARAM_STEP = 0.1;
const PRESETS: CurvePreset[] = [
  {
    id: "wave",
    label: "Vague libre",
    degree: 3,
    points: DEFAULT_POINTS,
    weights: DEFAULT_POINTS.map(() => 1),
  },
  {
    id: "quarter-circle",
    label: "Quart de cercle exact",
    degree: 2,
    points: [
      [2, 0, 0],
      [2, 2, 0],
      [0, 2, 0],
    ],
    weights: [1, W_ARC, 1],
    knots: [0, 0, 0, 10, 10, 10],
  },
  {
    id: "full-circle",
    label: "Cercle exact",
    degree: 2,
    points: [
      [2, 0, 0],
      [2, 2, 0],
      [0, 2, 0],
      [-2, 2, 0],
      [-2, 0, 0],
      [-2, -2, 0],
      [0, -2, 0],
      [2, -2, 0],
      [2, 0, 0],
    ],
    weights: [1, W_ARC, 1, W_ARC, 1, W_ARC, 1, W_ARC, 1],
    knots: [0, 0, 0, 2, 2, 5, 5, 8, 8, 10, 10, 10],
  },
  {
    id: "wide-local",
    label: "Controle local",
    degree: 3,
    points: [
      [-3.2, -0.4, 0],
      [-2.3, 1.5, 0],
      [-1.1, -0.7, 0.2],
      [0, 1.4, -0.3],
      [1.1, -1.1, 0.3],
      [2.2, 0.9, 0],
      [3.2, -0.1, 0],
    ],
    weights: [1, 1, 1, 1, 1, 1, 1],
  },
];

function makeClampedUniformKnots(pointCount: number, degree: number): number[] {
  const interiorCount = Math.max(0, pointCount - degree - 1);
  const knots: number[] = [];
  for (let i = 0; i <= degree; i++) knots.push(KNOT_MIN);
  for (let i = 1; i <= interiorCount; i++) {
    knots.push((KNOT_MAX * i) / (interiorCount + 1));
  }
  for (let i = 0; i <= degree; i++) knots.push(KNOT_MAX);
  return knots;
}

function multiplicityAt(knots: number[], value: number) {
  return knots.filter((k) => Math.abs(k - value) < 1e-8).length;
}

function displayNumber(value: number) {
  return Number.isFinite(value) ? value.toFixed(3).replace(/\.?0+$/, "") : "nan";
}

function isUniformBySpans(knots: number[], degree: number) {
  const domain = getDomain(knots, degree);
  if (!domain) return false;
  const distinct = Array.from(new Set(knots.filter((k) => k >= domain[0] && k <= domain[1])));
  if (distinct.length <= 2) return true;
  const spans = distinct.slice(1).map((k, i) => k - distinct[i]);
  const first = spans[0];
  return spans.every((span) => Math.abs(span - first) < 1e-8);
}

function getDomain(knots: number[], degree: number): [number, number] | null {
  if (knots.length <= degree * 2) return null;
  const min = knots[degree];
  const max = knots[knots.length - degree - 1];
  if (!Number.isFinite(min) || !Number.isFinite(max) || max <= min) return null;
  return [min, max];
}

function isSorted(values: number[]) {
  return values.every((value, i) => i === 0 || values[i - 1] <= value);
}

function clampKnotValue(value: number) {
  if (!Number.isFinite(value)) return KNOT_MIN;
  return Math.min(Math.max(value, KNOT_MIN), KNOT_MAX);
}

function clampParameterValue(value: number, domain: [number, number] | null) {
  if (!domain || !Number.isFinite(value)) return 0;
  return Math.min(Math.max(value, domain[0]), domain[1]);
}

function clampWeight(value: number) {
  if (!Number.isFinite(value)) return 1;
  return Math.min(Math.max(value, 0.05), 10);
}

function wheelStep(e: WheelEvent<HTMLInputElement>, current: number, step: number) {
  e.preventDefault();
  e.stopPropagation();
  const direction = e.deltaY < 0 ? 1 : -1;
  return current + direction * step;
}

export default function NurbsKnotExplorer() {
  const kernel = useKernel();
  const initialPreset = PRESETS[0];
  const [presetId, setPresetId] = useState(initialPreset.id);
  const [degree, setDegree] = useState(initialPreset.degree);
  const [points, setPoints] = useState<Vec3[]>(initialPreset.points);
  const [weights, setWeights] = useState<number[]>(initialPreset.weights);
  const [knots, setKnots] = useState(() =>
    initialPreset.knots ?? makeClampedUniformKnots(initialPreset.points.length, initialPreset.degree),
  );
  const [u, setU] = useState(5);

  const maxDegree = Math.min(5, points.length - 1);
  const expectedKnotCount = points.length + degree + 1;
  const domain = getDomain(knots, degree);
  const sorted = isSorted(knots);

  const curveState = useMemo(() => {
    if (!kernel || knots.length !== expectedKnotCount || !sorted || !domain) {
      return { points: [] as THREE.Vector3[], point: null as THREE.Vector3 | null, error: null };
    }

    try {
      const curve = new kernel.NurbsCurve(
        degree,
        vec3ArrayToFlat(points),
        new Float64Array(weights),
        new Float64Array(knots),
      );
      const flat = curve.sample(SAMPLES);
      const curvePoints: THREE.Vector3[] = [];
      for (let i = 0; i < flat.length; i += 3) {
        curvePoints.push(new THREE.Vector3(flat[i], flat[i + 1], flat[i + 2]));
      }
      const clampedU = Math.min(Math.max(u, domain[0]), domain[1]);
      const at = curve.pointAt(clampedU);
      curve.free();
      return {
        points: curvePoints,
        point: new THREE.Vector3(at.x, at.y, at.z),
        error: null,
      };
    } catch (e) {
      const message = e instanceof Error ? e.message : String(e);
      return { points: [] as THREE.Vector3[], point: null, error: message };
    }
  }, [kernel, degree, domain, expectedKnotCount, knots, points, sorted, u, weights]);

  function applyPreset(id: string) {
    const preset = PRESETS.find((p) => p.id === id) ?? PRESETS[0];
    setPresetId(preset.id);
    setDegree(preset.degree);
    setPoints(preset.points);
    setWeights(preset.weights);
    const nextKnots = preset.knots ?? makeClampedUniformKnots(preset.points.length, preset.degree);
    setKnots(nextKnots);
    const nextDomain = getDomain(nextKnots, preset.degree);
    setU(nextDomain ? 0.5 * (nextDomain[0] + nextDomain[1]) : 0);
  }

  function updateDegree(next: number) {
    const clampedDegree = Math.min(next, maxDegree);
    setDegree(clampedDegree);
    const nextKnots = makeClampedUniformKnots(points.length, clampedDegree);
    setKnots(nextKnots);
    const nextDomain = getDomain(nextKnots, clampedDegree);
    setU(nextDomain ? 0.5 * (nextDomain[0] + nextDomain[1]) : 0);
  }

  function updateKnot(index: number, value: number) {
    setKnots((current) => current.map((k, i) => (i === index ? value : k)));
  }

  function setClampedUniform() {
    const next = makeClampedUniformKnots(points.length, degree);
    setKnots(next);
    const nextDomain = getDomain(next, degree);
    setU(nextDomain ? 0.5 * (nextDomain[0] + nextDomain[1]) : 0);
  }

  function updateWeight(index: number, value: number) {
    setWeights((current) => current.map((w, i) => (i === index ? clampWeight(value) : w)));
  }

  function setPointerFromClientX(clientX: number, rect: DOMRect) {
    if (!domain) return;
    const t = Math.min(Math.max((clientX - rect.left) / rect.width, 0), 1);
    setU(domain[0] + t * (domain[1] - domain[0]));
  }

  const normalizedU = domain ? (u - domain[0]) / (domain[1] - domain[0]) : 0;
  const uniformLabel = isUniformBySpans(knots, degree) ? "uniforme" : "non uniforme";
  const validationError =
    knots.length !== expectedKnotCount
      ? `Il faut ${expectedKnotCount} noeuds.`
      : !sorted
        ? "Les noeuds doivent rester tries."
        : !domain
          ? "Le domaine est vide. Baisse la multiplicite des bords."
          : curveState.error;

  return (
    <group position={[1.5, 0, 0]}>
      {curveState.points.length > 1 && (
        <Line points={curveState.points} color="#5fd6a2" lineWidth={3} />
      )}
      <ControlPolygon points={points} color="#747780" />
      {points.map((p, i) => (
        <mesh key={i} position={p}>
          <sphereGeometry args={[0.07 + 0.025 * Math.sqrt(weights[i] ?? 1), 18, 18]} />
          <meshStandardMaterial color={i === 0 || i === points.length - 1 ? "#f4a261" : "#8ab4ff"} />
        </mesh>
      ))}
      {curveState.point && (
        <mesh position={curveState.point}>
          <sphereGeometry args={[0.13, 24, 24]} />
          <meshStandardMaterial color="#f7e36b" emissive="#6c5b00" emissiveIntensity={0.35} />
        </mesh>
      )}
      <gridHelper args={[6, 12, "#30323a", "#202229"]} position={[0, -1.35, 0]} />
      <Html fullscreen>
        <div className="knot-explorer">
          <section className="knot-panel">
            <div className="knot-header">
              <div>
                <h2>Knot vector</h2>
                <span>{points.length} points de controle</span>
              </div>
              <strong>{uniformLabel}</strong>
            </div>

            <label className="knot-select">
              <span>Disposition</span>
              <select value={presetId} onChange={(e) => applyPreset(e.target.value)}>
                {PRESETS.map((preset) => (
                  <option key={preset.id} value={preset.id}>
                    {preset.label}
                  </option>
                ))}
              </select>
            </label>

            <label className="knot-degree">
              <span>Degre</span>
              <input
                type="range"
                min={1}
                max={maxDegree}
                step={1}
                value={degree}
                onChange={(e) => updateDegree(Number(e.target.value))}
              />
              <b>{degree}</b>
            </label>

            <div
              className="knot-slider"
              onPointerDown={(e) => {
                e.currentTarget.setPointerCapture(e.pointerId);
                setPointerFromClientX(e.clientX, e.currentTarget.getBoundingClientRect());
              }}
              onPointerMove={(e) => {
                if (e.buttons === 1) {
                  setPointerFromClientX(e.clientX, e.currentTarget.getBoundingClientRect());
                }
              }}
            >
              <div className="knot-track" />
              {domain &&
                knots.map((k, i) => {
                  const left = ((k - domain[0]) / (domain[1] - domain[0])) * 100;
                  const mult = multiplicityAt(knots, k);
                  return (
                    <div
                      key={`${i}-${k}`}
                      className="knot-tick"
                      style={{
                        left: `${Math.min(Math.max(left, 0), 100)}%`,
                        height: `${18 + Math.min(mult - 1, 4) * 7}px`,
                      }}
                      title={`knot ${i}: ${displayNumber(k)}, multiplicite ${mult}`}
                    />
                  );
                })}
              <div className="knot-cursor" style={{ left: `${Math.min(Math.max(normalizedU, 0), 1) * 100}%` }} />
            </div>

            <div className="knot-readout">
              <label>
                <span>u</span>
                <input
                  type="number"
                  min={domain?.[0] ?? KNOT_MIN}
                  max={domain?.[1] ?? KNOT_MAX}
                  step={PARAM_STEP}
                  value={Number.isFinite(u) ? String(u) : "0"}
                  onChange={(e) => setU(clampParameterValue(Number(e.target.value), domain))}
                  onWheel={(e) => setU(clampParameterValue(wheelStep(e, u, PARAM_STEP), domain))}
                />
              </label>
              <span>domaine = {domain ? `[${displayNumber(domain[0])}, ${displayNumber(domain[1])}]` : "vide"}</span>
              <span>longueur = {knots.length} / {expectedKnotCount}</span>
            </div>

            <div className="knot-input-grid">
              {knots.map((k, i) => (
                <label key={i}>
                  <span>U{i}</span>
                  <input
                    type="number"
                    min={KNOT_MIN}
                    max={KNOT_MAX}
                    step={KNOT_STEP}
                    value={Number.isFinite(k) ? String(k) : "0"}
                    onChange={(e) => updateKnot(i, clampKnotValue(Number(e.target.value)))}
                    onWheel={(e) => updateKnot(i, clampKnotValue(wheelStep(e, k, KNOT_STEP)))}
                  />
                </label>
              ))}
            </div>

            <div className="knot-actions">
              <button type="button" onClick={setClampedUniform}>Reset uniforme</button>
              <button type="button" onClick={() => setKnots((current) => [...current].sort((a, b) => a - b))}>
                Trier
              </button>
            </div>

            <div className="knot-section-title">Poids</div>
            <div className="knot-input-grid knot-weight-grid">
              {weights.map((w, i) => (
                <label key={i}>
                  <span>P{i}</span>
                  <input
                    type="number"
                    min={0.05}
                    max={10}
                    step={0.05}
                    value={Number.isFinite(w) ? String(w) : "1"}
                    onChange={(e) => updateWeight(i, Number(e.target.value))}
                    onWheel={(e) => updateWeight(i, wheelStep(e, w, 0.05))}
                  />
                </label>
              ))}
            </div>
            <div className="knot-actions">
              <button type="button" onClick={() => setWeights(points.map(() => 1))}>
                Poids uniformes
              </button>
            </div>

            {validationError && <div className="knot-warning">{validationError}</div>}
          </section>
        </div>
      </Html>
    </group>
  );
}
