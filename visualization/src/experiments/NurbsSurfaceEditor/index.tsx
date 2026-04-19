import { useEffect, useMemo, useState } from "react";
import { Line } from "@react-three/drei";
import { button, useControls } from "leva";
import * as THREE from "three";
import DraggableHandle from "../NurbsCurveEditor/DraggableHandle";
import { useKernel, type Kernel } from "../../kernel/useKernel";
import { vec3ArrayToFlat, type Vec3 } from "../../kernel/nurbs";

const NU = 4;
const NV = 4;

function makeDefaultNet(): Vec3[] {
  const out: Vec3[] = [];
  for (let iv = 0; iv < NV; iv++) {
    for (let iu = 0; iu < NU; iu++) {
      const x = (iu / (NU - 1) - 0.5) * 4;
      const z = (iv / (NV - 1) - 0.5) * 4;
      const y = 0.9 * Math.sin((iu / (NU - 1)) * Math.PI) * Math.cos((iv / (NV - 1)) * Math.PI);
      out.push([x, y, z]);
    }
  }
  return out;
}

const DEFAULT_POINTS = makeDefaultNet();

function buildSurfaceFrom(
  k: Kernel,
  pts: Vec3[],
  ws: number[],
  degU: number,
  degV: number,
) {
  const pu = Math.min(degU, NU - 1);
  const pv = Math.min(degV, NV - 1);
  if (pu < 1 || pv < 1) return null;
  return k.NurbsSurface.uniform(pu, pv, NU, NV, vec3ArrayToFlat(pts), new Float64Array(ws));
}

function isCorner(iu: number, iv: number) {
  return (
    (iu === 0 || iu === NU - 1) &&
    (iv === 0 || iv === NV - 1)
  );
}

function ControlNetGrid({ points }: { points: Vec3[] }) {
  const uLines: THREE.Vector3[][] = [];
  for (let iv = 0; iv < NV; iv++) {
    const seg: THREE.Vector3[] = [];
    for (let iu = 0; iu < NU; iu++) {
      seg.push(new THREE.Vector3(...points[iv * NU + iu]));
    }
    uLines.push(seg);
  }
  const vLines: THREE.Vector3[][] = [];
  for (let iu = 0; iu < NU; iu++) {
    const seg: THREE.Vector3[] = [];
    for (let iv = 0; iv < NV; iv++) {
      seg.push(new THREE.Vector3(...points[iv * NU + iu]));
    }
    vLines.push(seg);
  }
  return (
    <group>
      {uLines.map((seg, i) => (
        <Line key={`u-${i}`} points={seg} color="#6e6e78" dashed dashSize={0.12} gapSize={0.06} lineWidth={1} />
      ))}
      {vLines.map((seg, i) => (
        <Line key={`v-${i}`} points={seg} color="#5a5a64" dashed dashSize={0.12} gapSize={0.06} lineWidth={1} />
      ))}
    </group>
  );
}

export default function NurbsSurfaceEditor() {
  const kernel = useKernel();
  const [points, setPoints] = useState<Vec3[]>(DEFAULT_POINTS);
  const [weights, setWeights] = useState<number[]>(() => DEFAULT_POINTS.map(() => 1));

  const { degreeU, degreeV, gridU, gridV, dragPlane, showWeights, wireframe } = useControls(
    "Surface (left-drag points · middle-drag to rotate)",
    {
      degreeU: { value: 3, min: 1, max: 5, step: 1 },
      degreeV: { value: 3, min: 1, max: 5, step: 1 },
      gridU: { value: 48, min: 8, max: 128, step: 1 },
      gridV: { value: 48, min: 8, max: 128, step: 1 },
      dragPlane: {
        value: "xz",
        options: { "xz (ground)": "xz", "xy (front)": "xy", "yz (side)": "yz" },
      },
      showWeights: false,
      wireframe: false,
      reset: button(() => {
        setPoints(makeDefaultNet());
        setWeights(DEFAULT_POINTS.map(() => 1));
      }),
    },
  );

  const surfaceGeometry = useMemo(() => {
    if (!kernel) return null;
    try {
      const surface = buildSurfaceFrom(kernel, points, weights, degreeU, degreeV);
      if (!surface) return null;
      const raw = surface.sampleGrid(gridU, gridV) as {
        positions: ArrayLike<number>;
        normals: ArrayLike<number>;
        indices: ArrayLike<number>;
      };
      surface.free();

      const pos = new Float32Array(raw.positions.length);
      pos.set(raw.positions);
      const nrm = new Float32Array(raw.normals.length);
      nrm.set(raw.normals);
      const idx = new Uint32Array(raw.indices.length);
      idx.set(raw.indices);

      const geom = new THREE.BufferGeometry();
      geom.setAttribute("position", new THREE.BufferAttribute(pos, 3));
      geom.setAttribute("normal", new THREE.BufferAttribute(nrm, 3));
      geom.setIndex(Array.from(idx));
      geom.computeBoundingSphere();
      return geom;
    } catch (e) {
      console.warn("surface build failed", e);
      return null;
    }
  }, [kernel, points, weights, degreeU, degreeV, gridU, gridV]);

  useEffect(() => {
    return () => {
      surfaceGeometry?.dispose();
    };
  }, [surfaceGeometry]);

  useEffect(() => {
    if (weights.length !== points.length) {
      setWeights(points.map((_, i) => weights[i] ?? 1));
    }
  }, [points.length]); // eslint-disable-line react-hooks/exhaustive-deps

  const dragNormal: [number, number, number] =
    dragPlane === "xy" ? [0, 0, 1] : dragPlane === "yz" ? [1, 0, 0] : [0, 1, 0];

  return (
    <group>
      {surfaceGeometry && (
        <mesh geometry={surfaceGeometry}>
          <meshStandardMaterial
            color="#4a7bc8"
            emissive="#1a3058"
            emissiveIntensity={0.15}
            roughness={0.45}
            metalness={0.15}
            side={THREE.DoubleSide}
            wireframe={wireframe}
          />
        </mesh>
      )}
      <ControlNetGrid points={points} />
      {points.map((p, idx) => {
        const iu = idx % NU;
        const iv = Math.floor(idx / NU);
        const corner = isCorner(iu, iv);
        return (
          <DraggableHandle
            key={idx}
            position={p}
            color={corner ? "#ffa86e" : "#8eb8ff"}
            radius={0.07 * (showWeights ? Math.max(weights[idx] ?? 1, 0.2) : 1)}
            dragPlaneNormal={dragNormal}
            onChange={(np) =>
              setPoints((pts) => {
                const copy = pts.slice();
                copy[idx] = np;
                return copy;
              })
            }
          />
        );
      })}
    </group>
  );
}
