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
        <InspectorPanel dump={hydrated.dump} inspected={inspected} />
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
        <h2>Debug shapes</h2>
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
}: {
  dump: HydratedDebugDump | null;
  inspected: VizSelection | null;
}) {
  const entity = dump && inspected ? inspectedEntity(dump, inspected) : null;
  return (
    <section className="debug-section debug-inspector">
      <div className="debug-panel-header">
        <h2>Inspector</h2>
        {dump && <span>{dump.gmaps.reduce((sum, gmap) => sum + gmap.dartCount, 0)} darts</span>}
      </div>
      {!dump && <div className="debug-empty">No shape loaded</div>}
      {dump && !inspected && <Summary dump={dump} />}
      {entity?.kind === "vertex" && <VertexInfo entry={entity.entry} />}
      {entity?.kind === "edge" && <EdgeInfo entry={entity.entry} />}
      {entity?.kind === "face" && <FaceInfo entry={entity.entry} />}
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
      <KeyValue label="shapes" value={String(dump.shapes.length)} />
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

function EdgeInfo({ entry }: { entry: DebugEntityEntry<Edge> }) {
  return (
    <>
      <EntityIdentity kind="edge" entry={entry} dimension={1} />
      <KeyValue label="start" value={pointText(entry.value.start.point)} />
      <KeyValue label="end" value={pointText(entry.value.end.point)} />
      <KeyValue label="length" value={entry.value.length?.toFixed(6) ?? "unknown"} />
      <KeyValue label="faces" value={String(entry.value.faces().length)} />
    </>
  );
}

function FaceInfo({ entry }: { entry: DebugEntityEntry<Face> }) {
  return (
    <>
      <EntityIdentity kind="face" entry={entry} dimension={2} />
      <KeyValue label="loops" value={String(entry.value.loops().length)} />
      <KeyValue label="edges" value={String(entry.value.edges().length)} />
      <KeyValue label="vertices" value={String(entry.value.vertices().length)} />
      <KeyValue label="surface" value={entry.value.surface.constructor.name} />
    </>
  );
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
