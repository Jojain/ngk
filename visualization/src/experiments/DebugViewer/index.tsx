import { useEffect, useMemo, useRef, useState, type ReactNode } from "react";
import { createRoot, type Root } from "react-dom/client";

import VizSceneView, { type VizSelection } from "../../components/VizSceneView";
import { useVizControls } from "../../components/useVizControls";
import {
  clearDebugDumps,
  debugSelectionForTopology,
  debugTopologyForSelection,
  fetchDebugDumps,
  hydrateDebugDump,
  type DebugFacePcurve,
  type DebugEntityEntry,
  type DebugTopologyEntity,
  type DebugViewerEnvelope,
  type HydratedDebugDump,
} from "../../kernel/debugViewer";
import { useKernel } from "../../kernel/useKernel";
import type { Edge, Face, Vertex } from "../../wasm/ngk";
import { ConsolePane } from "./ConsolePane";

export default function DebugViewer() {
  const kernel = useKernel();
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
  const [fetchError, setFetchError] = useState<string | null>(null);
  const hydrationCache = useRef(
    new Map<string, { dump: HydratedDebugDump | null; error: string | null }>(),
  );

  useEffect(() => {
    let cancelled = false;
    const refresh = async () => {
      try {
        const next = await fetchDebugDumps();
        if (cancelled) return;
        setDumps(next);
        setFetchError(null);
        if (next.length > 0 && followLatest) {
          setActiveSequence(next[next.length - 1].sequence);
        }
      } catch (error) {
        if (!cancelled) {
          setFetchError(error instanceof Error ? error.message : String(error));
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
  const hydrated = useMemo(() => {
    if (!active || !kernel) return { dump: null, error: null };
    const cacheKey = `${active.sequence}:${active.receivedAt}`;
    const cached = hydrationCache.current.get(cacheKey);
    if (cached) return cached;
    try {
      const result = { dump: hydrateDebugDump(active.payload, kernel), error: null };
      hydrationCache.current.set(cacheKey, result);
      return result;
    } catch (error) {
      const result = {
        dump: null,
        error: error instanceof Error ? error.message : String(error),
      };
      hydrationCache.current.set(cacheKey, result);
      return result;
    }
  }, [active, kernel]);

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
    hydrationCache.current.clear();
  };

  const selectDump = (sequence: number) => {
    setFollowLatest(false);
    setActiveSequence(sequence);
  };

  const selectLatest = () => {
    setFollowLatest(true);
    setActiveSequence(dumps[dumps.length - 1]?.sequence ?? null);
  };

  const inspected = selected ?? hovered;
  const highlightedTopology = hydrated.dump
    ? debugTopologyForSelection(hydrated.dump, selected)
    : null;
  const toggleTopologyHighlight = (entity: DebugTopologyEntity) => {
    if (!hydrated.dump) return;
    const next = debugSelectionForTopology(hydrated.dump, entity);
    if (!next) return;
    setSelected((current) =>
      current?.kind === next.kind && current.id === next.id ? null : next,
    );
  };
  const hud = (
    <div className="debug-viewer">
      <aside className="debug-side-panel">
        <TimelinePanel
          dumps={dumps}
          activeSequence={active?.sequence ?? null}
          followLatest={followLatest}
          error={fetchError ?? hydrated.error}
          onSelect={selectDump}
          onLatest={selectLatest}
          onClear={() => void clear()}
        />
        <InspectorPanel
          dump={hydrated.dump}
          inspected={inspected}
          selected={selected}
          hovered={hovered}
          onSelect={setSelected}
          onHover={setHovered}
        />
      </aside>
      <ConsolePane
        dump={hydrated.dump}
        kernel={kernel}
        highlightedTopology={highlightedTopology}
        onToggleTopologyHighlight={toggleTopologyHighlight}
      />
    </div>
  );

  return (
    <>
      {hydrated.dump && (
        <VizSceneView
          scene={hydrated.dump.scene}
          {...controls}
          selected={selected}
          hovered={hovered}
          onSelect={setSelected}
          onHover={setHovered}
        />
      )}
      <BodyHud>{hud}</BodyHud>
    </>
  );
}

function BodyHud({ children }: { children: ReactNode }) {
  const rootRef = useRef<Root | null>(null);

  useEffect(() => {
    const host = document.createElement("div");
    host.className = "debug-hud-root";
    document.body.appendChild(host);
    rootRef.current = createRoot(host);
    return () => {
      rootRef.current?.unmount();
      rootRef.current = null;
      host.remove();
    };
  }, []);

  useEffect(() => {
    rootRef.current?.render(children);
  }, [children]);

  return null;
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
        <h2>Debug objects</h2>
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
  dump,
  inspected,
  selected,
  hovered,
  onSelect,
  onHover,
}: {
  dump: HydratedDebugDump | null;
  inspected: VizSelection | null;
  selected: VizSelection | null;
  hovered: VizSelection | null;
  onSelect: (selection: VizSelection) => void;
  onHover: (selection: VizSelection | null) => void;
}) {
  const entity = dump && inspected ? inspectedEntity(dump, inspected) : null;
  return (
    <section className="debug-section debug-inspector">
      <div className="debug-panel-header">
        <h2>Inspector</h2>
        {dump && <span>{dump.objects.length} object{dump.objects.length === 1 ? "" : "s"}</span>}
      </div>
      {!dump && <div className="debug-empty">No object loaded</div>}
      {dump && (!inspected || !entity) && <Summary dump={dump} />}
      {entity?.kind === "vertex" && <VertexInfo entry={entity.entry} />}
      {entity?.kind === "edge" && (
        <EdgeInfo
          dump={dump!}
          entry={entity.entry}
          selected={selected}
          hovered={hovered}
          onSelect={onSelect}
          onHover={onHover}
        />
      )}
      {entity?.kind === "face" && (
        <FaceInfo
          dump={dump!}
          entry={entity.entry}
          selected={selected}
          hovered={hovered}
          onSelect={onSelect}
          onHover={onHover}
        />
      )}
      {entity?.kind === "dart" && (
        <>
          <KeyValue label="kind" value="dart" />
          <KeyValue label="id" value={String(entity.entry.dart)} />
          <KeyValue
            label="alphas"
            value={Array.from(
              { length: entity.entry.gmap.involutionCount },
              (_, alpha) => `α${alpha}: ${entity.entry.gmap.alpha(alpha, entity.entry.dart)}`,
            ).join(", ")}
          />
        </>
      )}
      {entity?.kind === "alphaLink" && (
        <>
          <KeyValue label="kind" value={`alpha ${entity.involution}`} />
          <KeyValue label="darts" value={`${entity.dartA}, ${entity.dartB}`} />
        </>
      )}
    </section>
  );
}

function Summary({ dump }: { dump: HydratedDebugDump }) {
  return (
    <div className="debug-summary">
      <KeyValue label="name" value={dump.name} />
      <KeyValue label="objects" value={String(dump.objects.length)} />
      <KeyValue
        label="types"
        value={dump.objects.map((object) => object.constructor?.name ?? "Object").join(", ")}
      />
      <KeyValue label="faces" value={String(dump.selection.faces.length)} />
      <KeyValue label="edges" value={String(dump.selection.edges.length)} />
      <KeyValue label="vertices" value={String(dump.selection.vertices.length)} />
    </div>
  );
}

type InspectedEntity =
  | { kind: "vertex"; entry: DebugEntityEntry<Vertex> }
  | { kind: "edge"; entry: DebugEntityEntry<Edge> }
  | { kind: "face"; entry: DebugEntityEntry<Face> }
  | { kind: "dart"; entry: HydratedDebugDump["selection"]["darts"][number] }
  | { kind: "alphaLink"; involution: number; dartA: number; dartB: number };

function inspectedEntity(
  dump: HydratedDebugDump,
  selection: VizSelection,
): InspectedEntity | null {
  if (selection.kind === "vertex") {
    const entry = dump.selection.vertices.find(({ id }) => id === selection.id);
    return entry ? { kind: "vertex", entry } : null;
  }
  if (selection.kind === "edge") {
    const entry = dump.selection.edges.find(({ id }) => id === selection.id);
    return entry ? { kind: "edge", entry } : null;
  }
  if (selection.kind === "face") {
    const entry = dump.selection.faces.find(({ id }) => id === selection.id);
    return entry ? { kind: "face", entry } : null;
  }
  if (selection.kind === "dart") {
    const entry = dump.selection.darts.find(({ id }) => id === selection.id);
    return entry ? { kind: "dart", entry } : null;
  }
  const link = dump.scene.alphaLinks[selection.id];
  return link
    ? {
        kind: "alphaLink",
        involution: link.involution,
        dartA: link.dartA,
        dartB: link.dartB,
      }
    : null;
}

function VertexInfo({ entry }: { entry: DebugEntityEntry<Vertex> }) {
  return (
    <>
      <EntityIdentity kind="vertex" entry={entry} dimension={0} />
      <KeyValue label="point" value={pointText(entry.value.point)} />
      <KeyValue label="edges" value={String(entry.value.edges().length)} />
      <KeyValue label="faces" value={String(entry.value.faces().length)} />
    </>
  );
}

function EdgeInfo({
  dump,
  entry,
  selected,
  hovered,
  onSelect,
  onHover,
}: {
  dump: HydratedDebugDump;
  entry: DebugEntityEntry<Edge>;
  selected: VizSelection | null;
  hovered: VizSelection | null;
  onSelect: (selection: VizSelection) => void;
  onHover: (selection: VizSelection | null) => void;
}) {
  const incidentFaces = entry.value.faces();
  const faceEntries = incidentFaces
    .map((face) => dump.selection.faces.find(({ value }) => value.equals(face)))
    .filter((face): face is DebugEntityEntry<Face> => face !== undefined);

  return (
    <>
      <EntityIdentity kind="edge" entry={entry} dimension={1} />
      <KeyValue label="start" value={pointText(entry.value.start.point)} />
      <KeyValue label="end" value={pointText(entry.value.end.point)} />
      <KeyValue label="length" value={entry.value.length?.toFixed(6) ?? "unknown"} />
      <KeyValue label="faces" value={String(incidentFaces.length)} />
      <div className="debug-edge-faces">
        <h3>Incident face inspectors</h3>
        {faceEntries.length === 0 ? (
          <div className="debug-empty">No face inspector available</div>
        ) : (
          faceEntries.map((faceEntry, index) => (
            <FaceInfo
              key={faceEntry.id}
              dump={dump}
              entry={faceEntry}
              heading={`Face ${index + 1} of ${faceEntries.length}`}
              selected={selected}
              hovered={hovered}
              onSelect={onSelect}
              onHover={onHover}
            />
          ))
        )}
      </div>
    </>
  );
}

type FaceInfoProps = {
  dump: HydratedDebugDump;
  entry: DebugEntityEntry<Face>;
  heading?: string;
  selected: VizSelection | null;
  hovered: VizSelection | null;
  onSelect: (selection: VizSelection) => void;
  onHover: (selection: VizSelection | null) => void;
};

function FaceInfo({ dump, entry, heading, selected, hovered, onSelect, onHover }: FaceInfoProps) {
  const pcurves = facePcurves(entry.value);
  return (
    <div className="debug-face-inspector">
      {heading && <h3>{heading}</h3>}
      <EntityIdentity kind="face" entry={entry} dimension={2} />
      <KeyValue label="loops" value={String(entry.value.loops().length)} />
      <KeyValue label="edges" value={String(entry.value.edges().length)} />
      <KeyValue label="vertices" value={String(entry.value.vertices().length)} />
      <KeyValue label="surface" value={entry.value.surface.constructor.name} />
      <KeyValue label="pcurves" value={String(pcurves.length)} />
      {pcurves.length > 0 && (
        <FaceUvPanel
          dump={dump}
          face={entry.value}
          pcurves={pcurves}
          selected={selected}
          hovered={hovered}
          onSelect={onSelect}
          onHover={onHover}
        />
      )}
    </div>
  );
}

type DebugFaceWithPcurves = Face & {
  pcurves: () => DebugFacePcurve[];
};

function facePcurves(face: Face): DebugFacePcurve[] {
  try {
    return (face as DebugFaceWithPcurves).pcurves() ?? [];
  } catch {
    return [];
  }
}

function FaceUvPanel({
  dump,
  face,
  pcurves,
  selected,
  hovered,
  onSelect,
  onHover,
}: {
  dump: HydratedDebugDump;
  face: Face;
  pcurves: DebugFacePcurve[];
  selected: VizSelection | null;
  hovered: VizSelection | null;
  onSelect: (selection: VizSelection) => void;
  onHover: (selection: VizSelection | null) => void;
}) {
  const samples = pcurves.flatMap((pcurve) => curveSamples(pcurve.curve));
  const bounds = uvBounds(samples);
  if (!bounds) return null;

  const [minU, maxU, minV, maxV] = bounds;
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
    <div className="debug-uv">
      <div className="debug-uv-face">
        <h3>UV · {surfaceTitle(face.surface.constructor?.name ?? "surface")}</h3>
        <KeyValue
          label="box"
          value={`${formatScalar(minU)} .. ${formatScalar(maxU)} × ${formatScalar(minV)} .. ${formatScalar(maxV)}`}
        />
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
          {pcurves.map((pcurve, index) => {
            const projected = curveSamples(pcurve.curve).map(project);
            const points = projected.map(svgPoint).join(" ");
            const edgeSelection = edgeSelectionForPcurve(dump, face, pcurve);
            const isSelected =
              edgeSelection &&
              selected?.kind === edgeSelection.kind &&
              selected.id === edgeSelection.id;
            const isHovered =
              edgeSelection &&
              hovered?.kind === edgeSelection.kind &&
              hovered.id === edgeSelection.id;
            const stroke = curveStroke(pcurve.curve.kind);

            return (
              <g key={`${pcurve.dartId}-${index}`}>
                <polyline
                  className={interactionClass(
                    "debug-uv-pcurve",
                    Boolean(isSelected),
                    Boolean(isHovered),
                  )}
                  points={points}
                  stroke={stroke}
                />
                <polyline
                  className="debug-uv-pcurve-hit"
                  points={points}
                  onPointerEnter={() => onHover(edgeSelection ?? null)}
                  onPointerLeave={() => onHover(null)}
                  onClick={(event) => {
                    event.stopPropagation();
                    if (edgeSelection) onSelect(edgeSelection);
                  }}
                >
                  <title>
                    {`${pcurve.curve.kind} edge ${pcurve.edgeKey} · dart ${pcurve.dartId}`}
                  </title>
                </polyline>
                {orientationSegments(projected, 2.75).map((segment, arrowIndex) => (
                  <polyline
                    key={arrowIndex}
                    className="debug-uv-arrow-segment"
                    points={segment.map(svgPoint).join(" ")}
                    markerEnd="url(#debug-uv-arrow)"
                  />
                ))}
              </g>
            );
          })}
        </svg>
      </div>
    </div>
  );
}

function curveSamples(curve: DebugFacePcurve["curve"], segments = 64): [number, number][] {
  const raw = curve.sample(segments) as ArrayLike<number>;
  const out: [number, number][] = [];
  for (let index = 0; index + 1 < raw.length; index += 2) {
    out.push([Number(raw[index]), Number(raw[index + 1])]);
  }
  return out;
}

function interactionClass(base: string, selected: boolean, hovered: boolean) {
  return `${base}${selected ? " selected" : ""}${hovered ? " hovered" : ""}`;
}

function orientationSegments(
  points: readonly (readonly [number, number])[],
  endInset: number,
): [readonly [number, number], readonly [number, number]][] {
  if (points.length < 2) return [];
  const step = Math.max(2, Math.ceil(points.length / 3));
  const segments: [readonly [number, number], readonly [number, number]][] = [];
  for (let index = step; index < points.length; index += step) {
    segments.push([points[index - 1], points[index]]);
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

function uvBounds(points: [number, number][]) {
  if (points.length === 0) return null;
  let minU = points[0][0];
  let maxU = points[0][0];
  let minV = points[0][1];
  let maxV = points[0][1];
  for (const [u, v] of points) {
    minU = Math.min(minU, u);
    maxU = Math.max(maxU, u);
    minV = Math.min(minV, v);
    maxV = Math.max(maxV, v);
  }
  const padU = Math.max((maxU - minU) * 0.08, 1e-6);
  const padV = Math.max((maxV - minV) * 0.08, 1e-6);
  return [minU - padU, maxU + padU, minV - padV, maxV + padV] as const;
}

function edgeSelectionForPcurve(
  dump: HydratedDebugDump,
  face: Face,
  pcurve: DebugFacePcurve,
): VizSelection | null {
  const edge = face.edges().find((edge) => edge.key === pcurve.edgeKey);
  return edge ? debugSelectionForTopology(dump, edge) : null;
}

function curveStroke(kind: string) {
  if (kind === "line") return "#aeb4c0";
  if (kind === "circle") return "#69d8ff";
  return "#c7a8ff";
}

function surfaceTitle(name: string) {
  return name.replace(/^Wasm/, "");
}

function formatScalar(value: number) {
  if (!Number.isFinite(value)) return String(value);
  if (Math.abs(value) < 1e-12) return "0";
  return String(Number(value.toPrecision(6)));
}

function EntityIdentity<T extends { key: string; dartId: number }>({
  kind,
  entry,
  dimension,
}: {
  kind: string;
  entry: DebugEntityEntry<T>;
  dimension: number;
}) {
  return (
    <>
      <KeyValue label="kind" value={kind} />
      <KeyValue label="key" value={entry.value.key} />
      <KeyValue label="dart" value={String(entry.value.dartId)} />
      <KeyValue
        label="cell darts"
        value={Array.from(entry.gmap.cellDarts(entry.value.dartId, dimension)).join(", ")}
      />
    </>
  );
}

function pointText(point: { x: number; y: number; z: number } | undefined) {
  return point
    ? [point.x, point.y, point.z].map((coordinate) => coordinate.toFixed(6)).join(", ")
    : "none";
}

function KeyValue({ label, value }: { label: string; value: string }) {
  return (
    <div className="debug-kv">
      <span>{label}</span>
      <b>{value}</b>
    </div>
  );
}
