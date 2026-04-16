import { useEffect, useMemo, useRef, useState } from "react";
import { Line } from "@react-three/drei";
import { button, useControls } from "leva";
import * as THREE from "three";
import ControlPolygon from "./ControlPolygon";
import DraggableHandle from "./DraggableHandle";
import { useKernel, type Kernel } from "../../kernel/useKernel";
import { vec3ArrayToFlat, type Vec3 } from "../../kernel/nurbs";

const DEFAULT_POINTS: Vec3[] = [
  [-2, 0, 0],
  [-1, 1.5, 0],
  [1, -1.5, 0],
  [2, 0, 0],
];

export default function NurbsCurveEditor() {
  const kernel = useKernel();
  const [points, setPoints] = useState<Vec3[]>(DEFAULT_POINTS);
  const [weights, setWeights] = useState<number[]>(() => DEFAULT_POINTS.map(() => 1));
  const [knotsState, setKnotsState] = useState<number[] | null>(null);

  // leva's button callbacks are registered with a stale closure. Keep the
  // live state (and the kernel, which loads asynchronously) in a ref so
  // button handlers always see fresh values.
  const stateRef = useRef<{
    points: Vec3[];
    weights: number[];
    knotsState: number[] | null;
    degree: number;
    kernel: Kernel | null;
  }>({ points, weights, knotsState, degree: 3, kernel: null });

  function buildCurveFrom(
    k: Kernel,
    pts: Vec3[],
    ws: number[],
    ks: number[] | null,
    deg: number,
  ) {
    const p = Math.min(deg, pts.length - 1);
    if (p < 1) return null;
    if (ks) {
      return new k.WasmNurbsCurve(
        p,
        vec3ArrayToFlat(pts),
        new Float64Array(ws),
        new Float64Array(ks),
      );
    }
    return k.WasmNurbsCurve.uniform(p, vec3ArrayToFlat(pts), new Float64Array(ws));
  }

  const { degree, samples, dragPlane, showWeights } = useControls("Curve (left-drag points · middle-drag to rotate)", {
    degree: { value: 3, min: 1, max: 5, step: 1 },
    samples: { value: 128, min: 16, max: 512, step: 1 },
    dragPlane: {
      value: "xz",
      options: { "xz (ground)": "xz", "xy (front)": "xy", "yz (side)": "yz" },
    },
    showWeights: false,
    reset: button(() => {
      setPoints(DEFAULT_POINTS);
      setWeights(DEFAULT_POINTS.map(() => 1));
      setKnotsState(null);
    }),
    addPoint: button(() => {
      setPoints((pts) => {
        const last = pts[pts.length - 1];
        const next: Vec3 = [last[0] + 1, last[1], last[2]];
        return [...pts, next];
      });
      setWeights((ws) => [...ws, 1]);
      setKnotsState(null);
    }),
    removePoint: button(() => {
      setPoints((pts) => (pts.length > 2 ? pts.slice(0, -1) : pts));
      setWeights((ws) => (ws.length > 2 ? ws.slice(0, -1) : ws));
      setKnotsState(null);
    }),
    insertKnotAt: button(() => {
      const s = stateRef.current;
      if (!s.kernel) return;
      const curve = buildCurveFrom(s.kernel, s.points, s.weights, s.knotsState, s.degree);
      if (!curve) return;
      const domain = curve.domain();
      const mid = 0.5 * (domain[0] + domain[1]);
      curve.insertKnot(mid);
      const flat = curve.controlPointsXyz();
      const newPts: Vec3[] = [];
      for (let i = 0; i < flat.length; i += 3) {
        newPts.push([flat[i], flat[i + 1], flat[i + 2]]);
      }
      const newWeights = Array.from(curve.weights());
      const newKnots = Array.from(curve.knots());
      curve.free();
      setPoints(newPts);
      setWeights(newWeights);
      setKnotsState(newKnots);
    }),
  });

  stateRef.current = { points, weights, knotsState, degree, kernel };

  function buildCurve() {
    if (!kernel) return null;
    return buildCurveFrom(kernel, points, weights, knotsState, degree);
  }

  const [curvePoints, knotsDisplay] = useMemo<[THREE.Vector3[], number[]]>(() => {
    if (!kernel) return [[], []];
    try {
      const curve = buildCurve();
      if (!curve) return [[], []];
      const flat = curve.sample(samples);
      const out: THREE.Vector3[] = [];
      for (let i = 0; i < flat.length; i += 3) {
        out.push(new THREE.Vector3(flat[i], flat[i + 1], flat[i + 2]));
      }
      const ks = Array.from(curve.knots());
      curve.free();
      return [out, ks];
    } catch (e) {
      console.warn("curve build failed", e);
      return [[], []];
    }
  }, [kernel, points, weights, degree, samples, knotsState]);

  useEffect(() => {
    if (weights.length !== points.length) {
      setWeights(points.map((_, i) => weights[i] ?? 1));
    }
  }, [points.length]); // eslint-disable-line react-hooks/exhaustive-deps

  const dragNormal: [number, number, number] =
    dragPlane === "xy" ? [0, 0, 1] : dragPlane === "yz" ? [1, 0, 0] : [0, 1, 0];

  return (
    <group>
      {curvePoints.length > 1 && (
        <Line points={curvePoints} color="#6ea8ff" lineWidth={2} />
      )}
      <ControlPolygon points={points} />
      {points.map((p, i) => (
        <DraggableHandle
          key={i}
          position={p}
          color={i === 0 || i === points.length - 1 ? "#ffa86e" : "#6ea8ff"}
          radius={0.08 * (showWeights ? Math.max(weights[i] ?? 1, 0.2) : 1)}
          dragPlaneNormal={dragNormal}
          onChange={(np) =>
            setPoints((pts) => {
              const copy = pts.slice();
              copy[i] = np;
              return copy;
            })
          }
        />
      ))}
      <KnotReadout knots={knotsDisplay} />
    </group>
  );
}

function KnotReadout({ knots }: { knots: number[] }) {
  useEffect(() => {
    if (knots.length) {
      // readout in console so users can see state; keeps scene uncluttered
      // eslint-disable-next-line no-console
      console.debug("knots:", knots.map((k) => k.toFixed(3)).join(", "));
    }
  }, [knots]);
  return null;
}
