import { useEffect, useLayoutEffect, useMemo, useRef, useState, type CSSProperties } from "react";
import { Edges, Html } from "@react-three/drei";
import { useThree, type ThreeEvent } from "@react-three/fiber";
import { useControls } from "leva";
import * as THREE from "three";
import { useKernel } from "../../kernel/useKernel";

type Vec3 = [number, number, number];

type Part =
  | { kind: "box"; center: Vec3; size: Vec3 }
  | { kind: "cylinder"; center: Vec3; axis: Vec3; radius: number; height: number };

type BodyView = { id: number; name: string; color: string; fixed: boolean; parts: Part[] };

type State = {
  poses: { id: number; matrix: number[] }[];
  coordinates: { joint: string; coordinate: string; value: number; unit: string }[];
  mobility: number;
  redundancy: number;
  active_limits: string[];
  iterations: number;
  reached: boolean | null;
  error: string | null;
};

type Session = {
  load(preset: string): string;
  drag(body: bigint, lx: number, ly: number, lz: number, tx: number, ty: number, tz: number): string;
  free(): void;
};

const PRESETS = {
  "Bielle-manivelle": "slider-crank",
  "Bras robot (butées)": "arm",
  "Pendule sur rotule (cône)": "pendulum",
};

/** A pending drag: the body, the grabbed point in its own coordinates, and the plane the
 * pointer moves on, facing the camera through the grabbed point. */
type Drag = { body: number; local: THREE.Vector3; plane: THREE.Plane };

export default function Assembly3d() {
  const kernel = useKernel() as unknown as { AssemblyPlayground?: new () => Session } | null;
  const session = useRef<Session | null>(null);
  const root = useRef<THREE.Group>(null);
  const drag = useRef<Drag | null>(null);
  const pending = useRef<THREE.Vector3 | null>(null);
  const frame = useRef<number | null>(null);
  const [bodies, setBodies] = useState<BodyView[]>([]);
  const [state, setState] = useState<State | null>(null);
  const [target, setTarget] = useState<Vec3 | null>(null);
  const [elapsed, setElapsed] = useState(0);
  const [error, setError] = useState("");
  const controls = useThree((s) => s.controls) as { enabled: boolean } | null;

  const { exemple } = useControls("Assemblage 3D", {
    exemple: { options: PRESETS, value: "slider-crank" },
  });

  useEffect(() => {
    if (!kernel) return;
    if (!kernel.AssemblyPlayground) {
      setError("Le noyau chargé ne contient pas l'AssemblyPlayground : reconstruisez le WebAssembly.");
      return;
    }
    session.current?.free();
    session.current = new kernel.AssemblyPlayground();
    try {
      const loaded = JSON.parse(session.current.load(exemple)) as { bodies: BodyView[]; state: State };
      setBodies(loaded.bodies);
      setState(loaded.state);
      setError("");
    } catch (e) {
      setError(String(e));
    }
    return () => {
      if (frame.current !== null) cancelAnimationFrame(frame.current);
      frame.current = null;
      session.current?.free();
      session.current = null;
    };
  }, [kernel, exemple]);

  const poses = useMemo(() => {
    const map = new Map<number, number[]>();
    for (const pose of state?.poses ?? []) map.set(pose.id, pose.matrix);
    return map;
  }, [state]);

  /** Solves toward the latest pointer position, at most once per animation frame. */
  function schedule() {
    if (frame.current !== null) return;
    frame.current = requestAnimationFrame(() => {
      frame.current = null;
      const current = drag.current;
      const world = pending.current;
      if (!current || !world || !session.current || !root.current) return;
      const goal = root.current.worldToLocal(world.clone());
      const started = performance.now();
      try {
        const next = JSON.parse(
          session.current.drag(
            BigInt(current.body),
            current.local.x,
            current.local.y,
            current.local.z,
            goal.x,
            goal.y,
            goal.z,
          ),
        ) as State;
        setElapsed(performance.now() - started);
        setState(next);
        setTarget([goal.x, goal.y, goal.z]);
      } catch (e) {
        setError(String(e));
      }
    });
  }

  function grab(body: BodyView, event: ThreeEvent<PointerEvent>, object: THREE.Object3D) {
    if (body.fixed || event.button !== 0) return;
    event.stopPropagation();
    (event.target as Element).setPointerCapture?.(event.pointerId);
    if (controls) controls.enabled = false;
    const normal = event.camera.getWorldDirection(new THREE.Vector3());
    drag.current = {
      body: body.id,
      local: object.worldToLocal(event.point.clone()),
      plane: new THREE.Plane().setFromNormalAndCoplanarPoint(normal, event.point),
    };
  }

  function move(event: ThreeEvent<PointerEvent>) {
    if (!drag.current) return;
    event.stopPropagation();
    const hit = event.ray.intersectPlane(drag.current.plane, new THREE.Vector3());
    if (!hit) return;
    pending.current = hit;
    schedule();
  }

  function release(event: ThreeEvent<PointerEvent>) {
    if (!drag.current) return;
    event.stopPropagation();
    (event.target as Element).releasePointerCapture?.(event.pointerId);
    drag.current = null;
    pending.current = null;
    setTarget(null);
    if (controls) controls.enabled = true;
  }

  return (
    <group>
      {/* The kernel works with Z up. */}
      <group ref={root} rotation={[-Math.PI / 2, 0, 0]}>
        {bodies.map((body) => (
          <Body
            key={`${exemple}-${body.id}`}
            body={body}
            matrix={poses.get(body.id)}
            onGrab={grab}
            onMove={move}
            onRelease={release}
          />
        ))}
        {target && (
          <mesh position={target}>
            <sphereGeometry args={[0.06, 16, 12]} />
            <meshBasicMaterial color="#ffd28c" />
          </mesh>
        )}
        <gridHelper args={[10, 20, "#3a3d44", "#24262b"]} rotation={[Math.PI / 2, 0, 0]} />
      </group>
      <Html fullscreen style={{ pointerEvents: "none" }}>
        <Panel state={state} elapsed={elapsed} error={error} />
      </Html>
    </group>
  );
}

function Body({
  body,
  matrix,
  onGrab,
  onMove,
  onRelease,
}: {
  body: BodyView;
  matrix: number[] | undefined;
  onGrab: (body: BodyView, event: ThreeEvent<PointerEvent>, object: THREE.Object3D) => void;
  onMove: (event: ThreeEvent<PointerEvent>) => void;
  onRelease: (event: ThreeEvent<PointerEvent>) => void;
}) {
  const group = useRef<THREE.Group>(null);
  const [hovered, setHovered] = useState(false);

  useLayoutEffect(() => {
    const object = group.current;
    if (!object || !matrix) return;
    object.matrix.fromArray(matrix);
    object.matrixWorldNeedsUpdate = true;
  }, [matrix]);

  const color = hovered && !body.fixed ? new THREE.Color(body.color).offsetHSL(0, 0, 0.12) : body.color;

  return (
    <group
      ref={group}
      matrixAutoUpdate={false}
      onPointerDown={(event) => group.current && onGrab(body, event, group.current)}
      onPointerMove={onMove}
      onPointerUp={onRelease}
      onPointerCancel={onRelease}
      onPointerOver={(event) => {
        event.stopPropagation();
        setHovered(true);
        if (!body.fixed) document.body.style.cursor = "grab";
      }}
      onPointerOut={() => {
        setHovered(false);
        document.body.style.cursor = "";
      }}
    >
      {body.parts.map((part, index) => (
        <PartMesh key={index} part={part} color={color} />
      ))}
    </group>
  );
}

function PartMesh({ part, color }: { part: Part; color: THREE.ColorRepresentation }) {
  if (part.kind === "box") {
    return (
      <mesh position={part.center}>
        <boxGeometry args={part.size} />
        <meshStandardMaterial color={color} />
        <Edges color="#101114" />
      </mesh>
    );
  }
  const quaternion = new THREE.Quaternion().setFromUnitVectors(
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(...part.axis).normalize(),
  );
  return (
    <mesh position={part.center} quaternion={quaternion}>
      <cylinderGeometry args={[part.radius, part.radius, part.height, 40]} />
      <meshStandardMaterial color={color} />
      <Edges color="#101114" threshold={30} />
    </mesh>
  );
}

function Panel({ state, elapsed, error }: { state: State | null; elapsed: number; error: string }) {
  const box: CSSProperties = {
    position: "absolute",
    bottom: 12,
    right: 12,
    width: 260,
    padding: "10px 12px",
    background: "rgba(20, 21, 25, 0.88)",
    border: "1px solid #2c2f36",
    borderRadius: 8,
    color: "#e6e7ea",
    font: "12px/1.5 ui-monospace, monospace",
  };
  return (
    <div style={box}>
      <div style={{ color: "#9aa0aa", marginBottom: 6 }}>
        Attrapez une pièce de couleur et glissez.
      </div>
      {state && (
        <>
          <div>
            mobilité {state.mobility} · hyperstatisme {state.redundancy}
          </div>
          <div style={{ color: "#9aa0aa" }}>
            {state.iterations} itérations · {elapsed.toFixed(1)} ms
            {state.reached === false ? " · cible hors d'atteinte" : ""}
          </div>
          <div style={{ marginTop: 8 }}>
            {state.coordinates.map((c) => (
              <div key={`${c.joint}-${c.coordinate}`} style={{ display: "flex", justifyContent: "space-between" }}>
                <span style={{ color: "#9aa0aa" }}>
                  {c.joint} · {c.coordinate}
                </span>
                <span>
                  {c.value.toFixed(c.unit ? 1 : 3)}
                  {c.unit}
                </span>
              </div>
            ))}
          </div>
          {state.active_limits.length > 0 && (
            <div style={{ marginTop: 8, color: "#ff8a7a" }}>
              en butée : {state.active_limits.join(", ")}
            </div>
          )}
          {state.error && <div style={{ marginTop: 8, color: "#ff8a7a" }}>{state.error}</div>}
        </>
      )}
      {error && <div style={{ marginTop: 8, color: "#ff8a7a" }}>{error}</div>}
    </div>
  );
}
