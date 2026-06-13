import { useEffect, useMemo, useRef, useState, type ReactNode } from "react";
import { createRoot, type Root } from "react-dom/client";
import { Line } from "@react-three/drei";
import VizSceneView, {
  type VizSelection,
} from "../../components/VizSceneView";
import { useVizControls } from "../../components/useVizControls";
import {
  clearDebugDumps,
  fetchDebugDumps,
  type DebugViewerEnvelope,
  type DebugViewerPayload,
  type FaceMetadata,
  type PcurveMetadata,
} from "../../kernel/debugViewer";

export default function DebugViewer() {
  const controls = useVizControls({
    showDarts: false,
    showDartLabels: false,
    viewerFaceColorOverridesScene: false,
  });
  const [dumps, setDumps] = useState<DebugViewerEnvelope[]>([]);
  const [activeSequence, setActiveSequence] = useState<number | null>(null);
  const [followLatest, setFollowLatest] = useState(true);
  const [selected, setSelected] = useState<VizSelection | null>(null);
  const [hovered, setHovered] = useState<VizSelection | null>(null);
  const [showNormals, setShowNormals] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    let cancelled = false;
    const refresh = async () => {
      try {
        const next = await fetchDebugDumps();
        if (cancelled) return;
        setDumps(next);
        setError(null);
        if (next.length > 0 && followLatest) {
          setActiveSequence(next[next.length - 1].sequence);
        }
      } catch (err) {
        if (!cancelled) {
          setError(err instanceof Error ? err.message : String(err));
        }
      }
    };
    void refresh();
    const id = window.setInterval(refresh, 1000);
    return () => {
      cancelled = true;
      window.clearInterval(id);
    };
  }, [followLatest]);

  const active = useMemo(
    () =>
      dumps.find((dump) => dump.sequence === activeSequence) ??
      dumps[dumps.length - 1] ??
      null,
    [activeSequence, dumps],
  );

  useEffect(() => {
    setSelected(null);
    setHovered(null);
  }, [active?.sequence]);

  const clear = async () => {
    await clearDebugDumps();
    setDumps([]);
    setActiveSequence(null);
    setFollowLatest(true);
    setSelected(null);
    setHovered(null);
  };

  const selectDump = (sequence: number) => {
    setFollowLatest(false);
    setActiveSequence(sequence);
  };

  const selectLatest = () => {
    setFollowLatest(true);
    setActiveSequence(dumps[dumps.length - 1]?.sequence ?? null);
  };

  const selectedFace =
    active?.payload && selected ? selectedFaceInfo(active.payload, selected) : null;
  const inspected = selected ?? hovered;

  const hud = (
    <div className="debug-viewer">
      <aside className="debug-side-panel">
        <TimelinePanel
          dumps={dumps}
          activeSequence={active?.sequence ?? null}
          followLatest={followLatest}
          error={error}
          onSelect={selectDump}
          onLatest={selectLatest}
          onClear={() => void clear()}
        />
        <InspectorPanel
          payload={active?.payload ?? null}
          inspected={inspected}
          selected={selected}
          hovered={hovered}
          showNormals={showNormals}
          onSelect={setSelected}
          onHover={setHovered}
          onShowNormalsChange={setShowNormals}
        />
      </aside>
    </div>
  );

  return (
    <>
      {active && (
        <VizSceneView
          scene={active.payload.scene}
          {...controls}
          selected={selected}
          hovered={hovered}
          onSelect={setSelected}
          onHover={setHovered}
        />
      )}
      {showNormals && selectedFace && (
        <FaceNormals samples={selectedFace.normals} />
      )}
      <BodyHud>{hud}</BodyHud>
    </>
  );
}

function BodyHud({ children }: { children: ReactNode }) {
  const rootRef = useRef<Root | null>(null);
  const hostRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    const host = document.createElement("div");
    host.className = "debug-hud-root";
    document.body.appendChild(host);
    hostRef.current = host;
    rootRef.current = createRoot(host);

    return () => {
      rootRef.current?.unmount();
      rootRef.current = null;
      host.remove();
      hostRef.current = null;
    };
  }, []);

  useEffect(() => {
    rootRef.current?.render(children);
  }, [children]);

  return null;
}

function FaceNormals({ samples }: { samples: FaceMetadata["normals"] }) {
  return (
    <group>
      {samples.map((sample, index) => {
        const end: [number, number, number] = [
          sample.origin[0] + sample.direction[0] * 0.4,
          sample.origin[1] + sample.direction[1] * 0.4,
          sample.origin[2] + sample.direction[2] * 0.4,
        ];
        return (
          <Line
            key={index}
            points={[sample.origin, end]}
            color="#ff5fb7"
            lineWidth={3}
          />
        );
      })}
    </group>
  );
}

function TimelinePanel({
  dumps,
  activeSequence,
  followLatest,
  error,
  onSelect,
  onLatest,
  onClear,
}: {
  dumps: DebugViewerEnvelope[];
  activeSequence: number | null;
  followLatest: boolean;
  error: string | null;
  onSelect: (sequence: number) => void;
  onLatest: () => void;
  onClear: () => void;
}) {
  return (
    <section className="debug-section debug-timeline">
      <div className="debug-panel-header">
        <h2>Debug dumps</h2>
        <div className="debug-header-actions">
          <button type="button" onClick={onLatest} disabled={dumps.length === 0 || followLatest}>
            Latest
          </button>
          <button type="button" onClick={onClear} disabled={dumps.length === 0}>
            Clear
          </button>
        </div>
      </div>
      {error && <div className="debug-error">{error}</div>}
      {dumps.length === 0 ? (
        <div className="debug-empty">Waiting for `debug_viewer::show(...)`</div>
      ) : (
        <div className="debug-dump-list">
          {dumps.map((dump) => (
            <button
              key={dump.sequence}
              type="button"
              className={dump.sequence === activeSequence ? "active" : ""}
              onClick={() => onSelect(dump.sequence)}
            >
              <b>#{dump.sequence}</b>
              <span>{dump.payload.name}</span>
              <small>{new Date(dump.receivedAt).toLocaleTimeString()}</small>
            </button>
          ))}
        </div>
      )}
    </section>
  );
}

function InspectorPanel({
  payload,
  inspected,
  selected,
  hovered,
  showNormals,
  onSelect,
  onHover,
  onShowNormalsChange,
}: {
  payload: DebugViewerPayload | null;
  inspected: VizSelection | null;
  selected: VizSelection | null;
  hovered: VizSelection | null;
  showNormals: boolean;
  onSelect: (selection: VizSelection) => void;
  onHover: (selection: VizSelection | null) => void;
  onShowNormalsChange: (value: boolean) => void;
}) {
  const info = payload && inspected ? selectedInfo(payload, inspected) : null;
  const face = info?.kind === "face" ? info.face : null;
  const uvGroups =
    payload && inspected ? associatedPcurveGroups(payload, selected ?? inspected) : [];

  return (
    <section className="debug-section debug-inspector">
      <div className="debug-panel-header">
        <h2>Inspector</h2>
        {payload && <span>{payload.gmap.dartCount} darts</span>}
      </div>
      {!payload && <div className="debug-empty">No dump loaded</div>}
      {payload && !inspected && <Summary payload={payload} />}
      {info && (
        <>
          <KeyValue label="kind" value={info.kind} />
          <KeyValue label="id" value={String(info.id)} />
          {info.key && <KeyValue label="key" value={info.key} />}
          {info.representativeDart !== undefined && (
            <KeyValue label="repr dart" value={String(info.representativeDart)} />
          )}
          {info.darts && <KeyValue label="darts" value={info.darts.join(", ")} />}
          {face?.normals[0] && (
            <KeyValue
              label="normal"
              value={formatVec3(face.normals[0].direction)}
            />
          )}
          {face && (
            <label className="debug-checkbox">
              <input
                type="checkbox"
                checked={showNormals}
                onChange={(event) => onShowNormalsChange(event.currentTarget.checked)}
              />
              <span>Display normals</span>
            </label>
          )}
          {payload && uvGroups.length > 0 && (
            <AssociatedUvPanels
              payload={payload}
              groups={uvGroups}
              selected={selected}
              hovered={hovered}
              onSelect={onSelect}
              onHover={onHover}
            />
          )}
        </>
      )}
    </section>
  );
}

function Summary({ payload }: { payload: DebugViewerPayload }) {
  return (
    <div className="debug-summary">
      <KeyValue label="name" value={payload.name} />
      <KeyValue label="faces" value={String(payload.metadata.faces.length)} />
      <KeyValue label="edges" value={String(payload.metadata.edges.length)} />
      <KeyValue label="vertices" value={String(payload.metadata.vertices.length)} />
      <KeyValue label="solids" value={String(payload.metadata.solids.length)} />
    </div>
  );
}

type FacePcurveGroup = {
  face: FaceMetadata;
  curves: PcurveMetadata[];
};

function AssociatedUvPanels({
  payload,
  groups,
  selected,
  hovered,
  onSelect,
  onHover,
}: {
  payload: DebugViewerPayload;
  groups: FacePcurveGroup[];
  selected: VizSelection | null;
  hovered: VizSelection | null;
  onSelect: (selection: VizSelection) => void;
  onHover: (selection: VizSelection | null) => void;
}) {
  return (
    <div className="debug-uv">
      {groups.map(({ face, curves }) => (
        <div className="debug-uv-face" key={face.key}>
          <h3>UV · {face.key}</h3>
          <UvSvg
            payload={payload}
            curves={curves}
            selected={selected}
            hovered={hovered}
            onSelect={onSelect}
            onHover={onHover}
          />
        </div>
      ))}
    </div>
  );
}

function UvSvg({
  payload,
  curves,
  selected,
  hovered,
  onSelect,
  onHover,
}: {
  payload: DebugViewerPayload;
  curves: PcurveMetadata[];
  selected: VizSelection | null;
  hovered: VizSelection | null;
  onSelect: (selection: VizSelection) => void;
  onHover: (selection: VizSelection | null) => void;
}) {
  const endMarkerRadius = 2.75;
  const points = curves.flatMap((curve) => curve.samples);
  const minU = Math.min(...points.map((point) => point[0]));
  const maxU = Math.max(...points.map((point) => point[0]));
  const minV = Math.min(...points.map((point) => point[1]));
  const maxV = Math.max(...points.map((point) => point[1]));
  const width = Math.max(maxU - minU, 1e-9);
  const height = Math.max(maxV - minV, 1e-9);
  const project = ([u, v]: [number, number]) => {
    const x = 16 + ((u - minU) / width) * 208;
    const y = 16 + (1 - (v - minV) / height) * 168;
    return [x, y] as const;
  };
  const svgPoint = ([x, y]: readonly [number, number]) =>
    `${x.toFixed(2)},${y.toFixed(2)}`;

  return (
    <svg className="debug-uv-svg" viewBox="0 0 240 200" role="img">
      <defs>
        <marker
          id="debug-uv-arrow"
          viewBox="0 0 10 10"
          refX="8"
          refY="5"
          markerWidth="5"
          markerHeight="5"
          orient="auto-start-reverse"
        >
          <path d="M 1 1 L 8 5 L 1 9" />
        </marker>
      </defs>
      <rect x="1" y="1" width="238" height="198" />
      {curves.map((curve, index) => {
        const projected = curve.samples.map(project);
        const start = projected[0];
        const end = projected[projected.length - 1];
        const edgeSelection = selectionForKey(payload, "edge", curve.edgeKey);
        const startSelection = selectionForKey(
          payload,
          "vertex",
          curve.startVertexKey,
        );
        const endSelection = selectionForKey(payload, "vertex", curve.endVertexKey);
        const edgeSelected = selectionMatchesKey(
          payload,
          selected,
          "edge",
          curve.edgeKey,
        );
        const edgeHovered = selectionMatchesKey(
          payload,
          hovered,
          "edge",
          curve.edgeKey,
        );
        const startSelected = selectionMatchesKey(
          payload,
          selected,
          "vertex",
          curve.startVertexKey,
        );
        const startHovered = selectionMatchesKey(
          payload,
          hovered,
          "vertex",
          curve.startVertexKey,
        );
        const endSelected = selectionMatchesKey(
          payload,
          selected,
          "vertex",
          curve.endVertexKey,
        );
        const endHovered = selectionMatchesKey(
          payload,
          hovered,
          "vertex",
          curve.endVertexKey,
        );
        const startRadius = interactionRadius(5, startSelected, startHovered);
        const endRadius = interactionRadius(
          endMarkerRadius,
          endSelected,
          endHovered,
        );
        const points = projected.map(svgPoint).join(" ");
        return (
          <g key={`${curve.dart}-${index}`}>
            <polyline
              className={interactionClass(
                "debug-uv-pcurve",
                edgeSelected,
                edgeHovered,
              )}
              points={points}
            />
            {orientationSegments(projected, endRadius).map((points, arrowIndex) => (
              <polyline
                key={arrowIndex}
                className="debug-uv-arrow-segment"
                points={points.map(svgPoint).join(" ")}
                markerEnd="url(#debug-uv-arrow)"
              />
            ))}
            <polyline
              className="debug-uv-pcurve-hit"
              points={points}
              onPointerEnter={() => onHover(edgeSelection)}
              onPointerLeave={() => onHover(null)}
              onClick={(event) => {
                event.stopPropagation();
                if (edgeSelection) onSelect(edgeSelection);
              }}
            >
              <title>{`edge ${curve.edgeKey} · dart ${curve.dart}`}</title>
            </polyline>
            <circle
              className={interactionClass(
                "debug-uv-start",
                startSelected,
                startHovered,
              )}
              cx={start[0]}
              cy={start[1]}
              r={startRadius}
              onPointerEnter={(event) => {
                event.stopPropagation();
                onHover(startSelection);
              }}
              onPointerLeave={(event) => {
                event.stopPropagation();
                onHover(null);
              }}
              onClick={(event) => {
                event.stopPropagation();
                if (startSelection) onSelect(startSelection);
              }}
            >
              <title>{`vertex ${curve.startVertexKey} · dart ${curve.dart} start`}</title>
            </circle>
            <circle
              className={interactionClass(
                "debug-uv-end",
                endSelected,
                endHovered,
              )}
              cx={end[0]}
              cy={end[1]}
              r={endRadius}
              onPointerEnter={(event) => {
                event.stopPropagation();
                onHover(endSelection);
              }}
              onPointerLeave={(event) => {
                event.stopPropagation();
                onHover(null);
              }}
              onClick={(event) => {
                event.stopPropagation();
                if (endSelection) onSelect(endSelection);
              }}
            >
              <title>{`vertex ${curve.endVertexKey} · dart ${curve.dart} end`}</title>
            </circle>
          </g>
        );
      })}
    </svg>
  );
}

function associatedPcurveGroups(
  payload: DebugViewerPayload,
  selection: VizSelection,
): FacePcurveGroup[] {
  const entity = selectionEntity(payload, selection);
  if (!entity) return [];

  return payload.metadata.faces.flatMap((face) => {
    const associated = face.pcurves.some((curve) => {
      if (entity.kind === "face") return face.key === entity.key;
      if (entity.kind === "edge") return curve.edgeKey === entity.key;
      if (entity.kind === "vertex") {
        return (
          curve.startVertexKey === entity.key || curve.endVertexKey === entity.key
        );
      }
      return false;
    });
    const curves = face.pcurves.filter((curve) => curve.samples.length > 0);
    return associated && curves.length > 0 ? [{ face, curves }] : [];
  });
}

function selectionEntity(
  payload: DebugViewerPayload,
  selection: VizSelection | null,
): { kind: "vertex" | "edge" | "face"; key: string } | null {
  if (!selection || selection.kind === "dart" || selection.kind === "alphaLink") {
    return null;
  }
  const entries = selectionEntries(payload, selection.kind);
  const entry = entries.find((item) => item.renderId === selection.id);
  return entry ? { kind: selection.kind, key: entry.key } : null;
}

function selectionForKey(
  payload: DebugViewerPayload,
  kind: "vertex" | "edge",
  key: string,
): VizSelection | null {
  const entry = selectionEntries(payload, kind).find((item) => item.key === key);
  return entry ? { kind, id: entry.renderId } : null;
}

function selectionEntries(
  payload: DebugViewerPayload,
  kind: "vertex" | "edge" | "face",
) {
  if (kind === "vertex") return payload.selection.vertices;
  if (kind === "edge") return payload.selection.edges;
  return payload.selection.faces;
}

function selectionMatchesKey(
  payload: DebugViewerPayload,
  selection: VizSelection | null,
  kind: "vertex" | "edge",
  key: string,
) {
  const entity = selectionEntity(payload, selection);
  return entity?.kind === kind && entity.key === key;
}

function interactionClass(base: string, selected: boolean, hovered: boolean) {
  return `${base}${selected ? " selected" : ""}${hovered ? " hovered" : ""}`;
}

function interactionRadius(base: number, selected: boolean, hovered: boolean) {
  if (selected) return base * 1.6;
  if (hovered) return base * 1.35;
  return base;
}

function orientationSegments(
  points: readonly (readonly [number, number])[],
  endInset: number,
): [readonly [number, number], readonly [number, number]][] {
  if (points.length < 2) return [];
  const step = Math.max(2, Math.ceil(points.length / 3));
  const segments: [readonly [number, number], readonly [number, number]][] = [];
  for (let i = step; i < points.length; i += step) {
    segments.push([points[i - 1], points[i]]);
  }
  const last = points.length - 1;
  if (segments.length === 0 || segments[segments.length - 1][1] !== points[last]) {
    segments.push([points[last - 1], points[last]]);
  }
  const final = segments.length - 1;
  segments[final] = [
    segments[final][0],
    insetSegmentEnd(segments[final][0], segments[final][1], endInset),
  ];
  return segments;
}

function insetSegmentEnd(
  start: readonly [number, number],
  end: readonly [number, number],
  inset: number,
): readonly [number, number] {
  const dx = end[0] - start[0];
  const dy = end[1] - start[1];
  const length = Math.hypot(dx, dy);
  if (length <= inset) return start;
  const scale = (length - inset) / length;
  return [start[0] + dx * scale, start[1] + dy * scale];
}

type SelectedInfo = {
  kind: string;
  id: number;
  key?: string;
  representativeDart?: number;
  darts?: number[];
  payload?: { typeName: string; debug: string };
  alpha?: number[];
  face?: FaceMetadata;
};

function selectedInfo(
  payload: DebugViewerPayload,
  selected: VizSelection,
): SelectedInfo | null {
  if (selected.kind === "face") {
    const selection = payload.selection.faces.find((item) => item.renderId === selected.id);
    const face = payload.metadata.faces.find((item) => item.key === selection?.key);
    if (!selection || !face) return null;
    return {
      kind: "face",
      id: selected.id,
      key: face.key,
      representativeDart: face.representativeDart,
      darts: face.darts,
      payload: face.payload,
      alpha: alphasAt(payload, face.representativeDart),
      face,
    };
  }
  if (selected.kind === "edge") {
    const selection = payload.selection.edges.find((item) => item.renderId === selected.id);
    const edge = payload.metadata.edges.find((item) => item.key === selection?.key);
    if (!selection || !edge) return null;
    return {
      kind: "edge",
      id: selected.id,
      key: edge.key,
      representativeDart: edge.representativeDart,
      darts: edge.darts,
      payload: edge.payload,
      alpha: alphasAt(payload, edge.representativeDart),
    };
  }
  if (selected.kind === "vertex") {
    const selection = payload.selection.vertices.find((item) => item.renderId === selected.id);
    const vertex = payload.metadata.vertices.find((item) => item.key === selection?.key);
    if (!selection || !vertex) return null;
    return {
      kind: "vertex",
      id: selected.id,
      key: vertex.key,
      representativeDart: vertex.representativeDart,
      darts: vertex.darts,
      payload: vertex.payload,
      alpha: alphasAt(payload, vertex.representativeDart),
    };
  }
  if (selected.kind === "dart") {
    return {
      kind: "dart",
      id: selected.id,
      representativeDart: selected.id,
      alpha: alphasAt(payload, selected.id),
    };
  }
  const link = payload.scene.alphaLinks[selected.id];
  return link
    ? {
        kind: `alpha ${link.involution}`,
        id: selected.id,
        darts: [link.dartA, link.dartB],
      }
    : null;
}

function selectedFaceInfo(
  payload: DebugViewerPayload,
  selected: VizSelection,
): FaceMetadata | null {
  if (selected.kind !== "face") return null;
  const selection = payload.selection.faces.find((item) => item.renderId === selected.id);
  return payload.metadata.faces.find((item) => item.key === selection?.key) ?? null;
}

function alphasAt(payload: DebugViewerPayload, dart: number) {
  return payload.gmap.alphas.map((alpha) => alpha[dart]);
}

function formatVec3(value: [number, number, number]) {
  return value.map((coord) => coord.toFixed(3)).join(", ");
}

function KeyValue({ label, value }: { label: string; value: string }) {
  return (
    <div className="debug-kv">
      <span>{label}</span>
      <b>{value}</b>
    </div>
  );
}
